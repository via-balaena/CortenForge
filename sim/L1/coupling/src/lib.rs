//! Staggered forward soft↔rigid coupling — the Layer-2 keystone, forward half.
//!
//! Couples a [`sim_soft`] deformable body and a [`sim_core`] rigid body into one
//! simulation where they exchange contact force *both* ways, once per lockstep
//! step (a *partitioned* / staggered scheme). Each [`StaggeredCoupling::step`]:
//!
//! 1. reads the rigid body's pose (`sim_core::Data::xpos`) and poses the soft
//!    body's penalty-contact plane from it (rigid → soft);
//! 2. takes one *dynamic* backward-Euler soft step against that plane (the
//!    inertia term regularises the contact Newton solve — a quasi-static solve
//!    cannot make the no-contact→contact jump, see `docs/keystone/recon.md`);
//! 3. sums the contact `force_on_soft` over the active pairs;
//! 4. routes the Newton's-3rd-law reaction `−force_on_soft` onto the rigid
//!    body's `xfrc_applied` (soft → rigid);
//! 5. steps the rigid body.
//!
//! The two engines own their own solvers (the soft Newton-to-equilibrium step
//! and the rigid MuJoCo-aligned integrator); this crate only exchanges pose and
//! force at the interface. Validated by the forward gates in `tests/`: the
//! interface force balance holds and a rigid body settles when the soft
//! reaction matches its weight.
//!
//! Scope: penalty contact (a non-smooth stepping stone to IPC); a single
//! body-posed plane against a hand-built soft block; the soft solver is rebuilt
//! per step (re-posing in place is a deferred optimisation). Differentiability
//! was added incrementally and now spans both engines:
//! - the *explicit* (fixed-soft-position) coupled-step factors — the analytic
//!   contact-force-vs-pose derivative ([`StaggeredCoupling::contact_force_height_jacobian`])
//!   and the rigid force response ([`StaggeredCoupling::rigid_step_probe`]);
//! - the *implicit* soft re-equilibration term lifting them to the total
//!   single-step derivative ([`StaggeredCoupling::contact_force_height_total_jacobian`],
//!   via the soft solver's `equilibrium_pose_sensitivity`);
//! - the full **soft-tape `VjpOp` crossing** — ONE `tape.backward` across both
//!   engines ([`StaggeredCoupling::coupled_step_load_gradient`], chaining the
//!   soft Newton load adjoint with [`ContactForceVjp`] and [`RigidStepVjp`]);
//! - the soft **material-parameter** gradient
//!   ([`StaggeredCoupling::coupled_step_material_gradient`]) and its multi-step
//!   **time-adjoint** ([`StaggeredCoupling::coupled_trajectory_material_gradient`]),
//!   one `tape.backward` over an N-step coupled rollout — the gradient the
//!   co-design optimizer's *design* half consumes;
//! - the multi-step **open-loop control gradient**
//!   ([`StaggeredCoupling::coupled_trajectory_control_gradient`]) — `∂z_N/∂u_k`
//!   for a per-step platen control force (the control input adds to
//!   `xfrc_applied`, so it rides the same rigid carry as the contact reaction);
//! - the multi-step **closed-loop policy gradient**
//!   ([`StaggeredCoupling::coupled_trajectory_policy_gradient`]) — `∂z_N/∂θ` for a
//!   state-feedback policy `u_k = π_θ(state_k)` ([`DiffPolicy`]/[`LinearFeedback`]),
//!   backprop-through-time across the state→control recurrence on the same tape,
//!   the gradient the co-design optimizer's *policy* half consumes;
//! - the **joint design+policy gradient**
//!   ([`StaggeredCoupling::coupled_trajectory_joint_gradient`]) — BOTH the soft
//!   material design variable `μ` (λ = 4μ) AND the policy parameters `θ` live on
//!   ONE tape, read `(∂z_N/∂μ_total, ∂z_N/∂θ)` from one `tape.backward` — the
//!   mission's "one outer loop differentiating w.r.t. *both* design and policy".

use nalgebra::Point3;
use sim_core::{
    DMatrix, DVector, Data, Matrix3, MjJointType, Model, SpatialVector,
    mass_directional_derivative, mj_differentiate_pos, mj_integrate_pos_explicit, mj_jac_point,
};
use sim_ml_chassis::autograd::VjpOp;
use sim_ml_chassis::{Tape, Tensor, Var};
use sim_soft::{
    ActivePairsFor, BoundaryConditions, ContactModel, ContactPair, ContactPairReadout,
    CpuNewtonSolver, FrictionVertexForce, HandBuiltTetMesh, IpcRigidContact, LoadAxis,
    MaterialField, Mesh, NeoHookean, PenaltyRigidContact, RigidPlane, RigidTwist, Sdf, Solver,
    SolverConfig, SphereSdf, Tet4, TranslatedSdf, Vec3, VertexId,
};
use std::marker::PhantomData;

/// A rigid-primitive contact constructible from a single posed [`Sdf`] primitive +
/// `(κ, d̂)`, with an active-pair readout over the keystone block — the bridge that
/// lets [`StaggeredCoupling`] run over either [`PenaltyRigidContact`] (the stepping
/// stone) or [`IpcRigidContact`] (the C²-barrier successor). Defined here (a local
/// trait) so the soft-side contact types need no change.
///
/// The primitive is generic over [`Sdf`] (not fixed to [`RigidPlane`]) so the
/// coupling can pose a FINITE collider — a curved end-effector (`PosedSphere`)
/// indenting the soft block — and not just the infinite downward half-space. The
/// plane path constructs `from_primitive(RigidPlane, …)`, byte-identical to the
/// pre-generalization `with_params(vec![plane], …)` (a single boxing of the same
/// `RigidPlane`). The finite-collider path is the L1 finite-contact carry (the
/// gradient of #415's curved-contact term composed through the coupling adjoint).
pub trait PlaneContact: ContactModel + ActivePairsFor<NeoHookean> + Sized {
    /// Build the contact from a single posed rigid primitive (`RigidPlane` for the
    /// keystone half-space, `PosedSphere` for the finite collider) and the
    /// penalty/barrier parameters `(κ, d̂)`.
    fn from_primitive<S: Sdf + 'static>(primitive: S, kappa: f64, d_hat: f64) -> Self;
    /// Per-active-pair readout at `positions` (the inherent `per_pair_readout`,
    /// surfaced through the trait so the coupling can call it generically).
    fn pair_readout(&self, mesh: &HandBuiltTetMesh, positions: &[Vec3]) -> Vec<ContactPairReadout>;
}

impl PlaneContact for PenaltyRigidContact {
    fn from_primitive<S: Sdf + 'static>(primitive: S, kappa: f64, d_hat: f64) -> Self {
        Self::with_params(vec![primitive], kappa, d_hat)
    }
    fn pair_readout(&self, mesh: &HandBuiltTetMesh, positions: &[Vec3]) -> Vec<ContactPairReadout> {
        self.per_pair_readout(mesh, positions)
    }
}

impl PlaneContact for IpcRigidContact {
    fn from_primitive<S: Sdf + 'static>(primitive: S, kappa: f64, d_hat: f64) -> Self {
        Self::with_params(vec![primitive], kappa, d_hat)
    }
    fn pair_readout(&self, mesh: &HandBuiltTetMesh, positions: &[Vec3]) -> Vec<ContactPairReadout> {
        self.per_pair_readout(mesh, positions)
    }
}

/// The keystone's finite rigid collider: a [`SphereSdf`] posed at a world centre
/// via [`TranslatedSdf`] (the shared `sim_soft` posing wrapper). `SphereSdf` is
/// origin-centred and rotation-invariant, so a query-point translation `p ↦ p −
/// center` is the full pose; `eval`/`grad`/`hessian` forward to the inner sphere
/// (translation leaves the Hessian — the curvature #415 added — unchanged, so the
/// curved-normal pose sensitivity is the sphere's own `∇²sd`).
type PosedSphere = TranslatedSdf<SphereSdf>;

/// Pose a [`SphereSdf`] of the given `radius` at world `center` — the keystone's
/// finite curved end-effector (`PosedSphere`).
fn posed_sphere(radius: f64, center: Vec3) -> PosedSphere {
    TranslatedSdf {
        inner: SphereSdf { radius },
        offset: center,
    }
}

/// The rigid collider geometry [`StaggeredCoupling`] poses against the soft block.
#[derive(Clone, Copy, Debug)]
enum Collider {
    /// The infinite downward half-space (the keystone #402–#406 scene; default) —
    /// `build_contact` constructs a [`RigidPlane`], optionally orientation-tracking
    /// under [`StaggeredCoupling::with_rotating_normal`].
    Plane,
    /// A finite posed sphere of the given radius (the L1 finite-contact gate) — a
    /// curved end-effector indenting the block, exercising #415's curvature term
    /// (`∇²sd ≠ 0`) through the coupling adjoint. The scalar `height` still drives
    /// the vertical carry (the sphere centre rides `+ẑ` with `height`); the lateral
    /// centre defaults over the block for a central + stable patch, and is re-pointable
    /// to a moving end-effector (the arm tip) via
    /// [`StaggeredCoupling::with_contact_geom`].
    Sphere { radius: f64 },
}

/// The soft Newton solver the coupling builds per step, over contact type `C`.
type SoftSolver<C> = CpuNewtonSolver<Tet4, HandBuiltTetMesh, C, NeoHookean, 4, 1>;

/// Scatter the precomputed contact-force-vs-position factor `∂fz/∂x_v` onto `slot`,
/// scaled by the upstream scalar cotangent `cot`. `factors` is per active pair
/// `(vertex_id, ∂fz/∂x_v, ∂fz/∂height)` from [`StaggeredCoupling::active_pair_force_factors`]
/// — curvature-correct for any collider (`∂fz/∂x_v = −cᵥ·n̂_z·n̂ + f_mag·H·ẑ`, the z-row
/// of `−cᵥ·n̂⊗n̂ + f_mag·H`; flat `(0,0,−cᵥ)` for the plane). Shared by [`ContactForceVjp`]
/// (single-step crossing, x* parent only) and [`ContactForceTrajVjp`] (which additionally
/// adds the `∂fz/∂height` factor onto its z parent).
fn scatter_dfz_dxstar(factors: &[(usize, Vec3, f64)], cot: f64, slot: &mut [f64]) {
    for &(v, dfz_dxv, _dfz_dz) in factors {
        slot[3 * v] += cot * dfz_dxv.x;
        slot[3 * v + 1] += cot * dfz_dxv.y;
        slot[3 * v + 2] += cot * dfz_dxv.z;
    }
}

/// The rigid engine's **multi-DOF** velocity response to an applied spatial force
/// on `body` — the matrix successor to the scalar free-body `∂vz'/∂fz = dt/m`
/// ([`StaggeredCoupling::rigid_vz_response`]).
///
/// Returns `∂qvel'/∂xfrc_applied[body] = Δt · M_impl⁻¹ · J_comᵀ` — the `nv × 6` input
/// (`G`) block of the coupled-step Jacobian `[A | G]` (the "xfrc column" of the
/// recon), one column per spatial-force component. `J_com` is the body's COM
/// spatial Jacobian (`mj_jac_point` at `xipos`, rows 0–2 angular / 3–5 linear —
/// the `[τ; f]` layout the integrator projects through `mj_apply_ft`), and `Δt =
/// model.timestep`. `M_impl = M + Δt·D` is the Euler `eulerdamp` matrix — the
/// joint-space mass `M = data.qM` plus the implicit joint **damping** `D =
/// model.implicit_damping` on the diagonal (the integrator solves `(M + Δt·D)·qacc =
/// F` then `qvel += Δt·qacc`, so the wrench reaches `qvel'` through `M_impl⁻¹`).
/// Undamped (`D = 0`) ⇒ `M_impl = M`, the original bare form, exactly.
///
/// Reads the live `data.qM` / `data.xipos` (it does not re-step), so `data` must be
/// at a **forwarded** configuration (call `data.forward(model)` — or any `step` —
/// first); a stale/un-forwarded `data` yields a wrong column or trips the `M`
/// invertibility panic.
///
/// This is the rigid factor of the coupled-step Jacobian generalized off the
/// free-body platen: for a single free body the column collapses to the scalar
/// `dt/m` on the contact axis; for an **articulated** mechanism a contact force at
/// a point maps to a generalized joint acceleration coupled across joints (the
/// off-diagonal terms the scalar drops — e.g. a force on a distal link accelerates
/// the proximal joint). FD-validated against a real scratch step in
/// `tests/rigid_multidof_response.rs` (hinge + 2-link, machine-exact).
///
/// **Damping / integrator scope.** The `M + Δt·D` form is exact for the Euler
/// integrator (the keystone's, MuJoCo's default), whose `eulerdamp` treats joint
/// *damping* implicitly but joint *stiffness* `K` explicitly — so `K` does NOT enter
/// this matrix. (Joint *armature* is already folded into `M = qM`.) A stiffness-
/// implicit integrator (`ImplicitSpringDamper`, `M + Δt·D + Δt²·K`) or `RungeKutta4`
/// changes the velocity update and is out of scope — see
/// `docs/keystone/damped_joints_recon.md`. FD-validated under damping in
/// `tests/rigid_multidof_response.rs`.
///
/// **Velocity vs position.** This is the *velocity* response only. Composing it
/// into a multi-step coupled carry (which threads it with the dense state
/// transition `A`, [`Data::transition_derivatives`], for the position rows) is the
/// FD-gated follow-on leaf — and the staggered-coupling position carry does NOT
/// follow naively from the bare integrator: the merged scalar carry integrates the
/// height with the step's STARTING (pre-update) velocity (the convention
/// `ZCarryVjp` encodes, FD-validated; flipping it to the freshly-updated velocity
/// breaks the trajectory gate — the #307 fix). The multi-DOF composition must be
/// FD-gated against a re-rolled full-coupled oracle, not derived from the bare-step
/// linearization; see `docs/keystone/multidof_rigid_recon.md` §8a.
///
/// To route a pure contact force `f` applied at an off-COM point `r_c`, the caller
/// sets `xfrc_applied[body] = [(r_c − xipos) × f ; f]` (the contact moment) so the
/// column — defined w.r.t. the COM-interpreted `xfrc` the integrator consumes —
/// maps it correctly.
///
/// # Panics
/// Panics if `M` is singular (a degenerate model — should not occur for a
/// well-posed mechanism).
/// `M_impl⁻¹` where `M_impl = M + Δt·D` is the Euler `eulerdamp` matrix — the
/// joint-space mass `M = data.qM` plus the implicit joint damping `D =
/// model.implicit_damping` on the diagonal. The shared implicit factor for the
/// wrench response ([`rigid_xfrc_column`]) and the actuator-input response
/// (`StaggeredCoupling::actuator_velocity_column`). `D = 0` ⇒ bare `M⁻¹`, exactly.
///
/// # Panics
/// Panics if `M_impl` is singular (a malformed model).
// expect_used: a singular mass matrix is a malformed-model programmer error
// surfaced loudly, mirroring `rigid_step_probe`'s divergence-panic rationale.
#[allow(clippy::expect_used)]
#[must_use]
fn implicit_mass_inverse(model: &Model, data: &Data) -> DMatrix<f64> {
    let mut m_impl = data.qM.clone();
    for i in 0..model.nv {
        m_impl[(i, i)] += model.timestep * model.implicit_damping[i];
    }
    m_impl
        .try_inverse()
        .expect("implicit mass matrix M + Δt·D must be invertible")
}

// expect_used: a singular mass matrix is a malformed-model programmer error
// surfaced loudly, mirroring `rigid_step_probe`'s divergence-panic rationale.
#[allow(clippy::expect_used)]
#[must_use]
pub fn rigid_xfrc_column(model: &Model, data: &Data, body: usize) -> DMatrix<f64> {
    // J at the body COM (xipos): 6×nv, rows 0–2 angular, 3–5 linear — the same
    // point and frame the integrator's `mj_apply_ft` uses for `xfrc_applied`.
    let jac = mj_jac_point(model, data, body, &data.xipos[body]); // 6 × nv
    let m_impl_inv = implicit_mass_inverse(model, data);
    model.timestep * m_impl_inv * jac.transpose() // (nv × nv)·(nv × 6) = nv × 6
}

/// The **right Jacobian of SO(3)** `J_r(φ)` (`φ` = a rotation vector, `θ = ‖φ‖`):
/// `J_r(φ) = I − (1−cosθ)/θ² · [φ]× + (θ−sinθ)/θ³ · [φ]×²`.
///
/// It is the exact tangent map of the quaternion exp-map step the integrator takes:
/// for `q' = q ⊕ exp(φ)` (right-multiply, body frame), perturbing `φ → φ + δφ` and
/// measuring the output tangent AT the nominal `q'` gives
/// `log(q'⁻¹ · q'(φ+δφ)) = J_r(φ)·δφ`. This is the position-row factor the multi-DOF
/// carry needs for a quaternion joint (the body-frame tangent convention
/// [`mj_differentiate_pos`] reads, which the FD [`StaggeredCoupling::loaded_state_jacobian`]
/// also uses), as distinct from the `h·I` / left-Jacobian forms
/// [`sim_core::mjd_quat_integrate`] returns for its own (tangent-at-`q_old`) convention.
/// Reduces to `I` as `θ → 0` (the linear / hinge limit). FD-validated against the real
/// quaternion step in `tests/`.
fn right_jacobian_so3(phi: Vec3) -> Matrix3<f64> {
    let theta = phi.norm();
    #[rustfmt::skip]
    let skew = Matrix3::new(
        0.0, -phi.z, phi.y,
        phi.z, 0.0, -phi.x,
        -phi.y, phi.x, 0.0,
    );
    let skew_sq = skew * skew;
    // Coefficients of −[φ]× and [φ]×². Below ~1e-8 the closed forms lose precision to
    // catastrophic cancellation, so use the small-angle Taylor limits (½ and 1/6).
    let (a, b) = if theta < 1e-8 {
        (0.5, 1.0 / 6.0)
    } else {
        let theta2 = theta * theta;
        (
            (1.0 - theta.cos()) / theta2,
            (theta - theta.sin()) / (theta2 * theta),
        )
    };
    Matrix3::identity() - a * skew + b * skew_sq
}

/// Chassis-tape [`VjpOp`] adapting a `sim-core` rigid step's response into the
/// soft autograd tape — the soft↔rigid crossing's rigid half (keystone S4).
///
/// The rigid engine has no reverse-mode tape; its sensitivity is a dense
/// Jacobian. This wraps the single scalar factor `∂vz'/∂xfrc_z` (the platen's
/// next vertical velocity vs an applied vertical force) so a chassis
/// `Tape::backward` flowing into the rigid node continues back through the
/// applied force. Parent = the `xfrc_z` (shape `[1]`) node; output = `vz'`
/// (shape `[1]`); the VJP accumulates `∂L/∂vz' · ∂vz'/∂xfrc_z` into the parent.
///
/// For the free-joint platen under semi-implicit Euler the factor is the
/// closed-form `dt/m` (S2-validated); [`StaggeredCoupling::rigid_vz_response`]
/// computes it by central FD over [`StaggeredCoupling::rigid_step_probe`] so the
/// op is not hard-wired to the free-body case.
#[derive(Clone, Copy, Debug)]
pub struct RigidStepVjp {
    dvz_dfz: f64,
}

impl RigidStepVjp {
    /// Construct from the rigid response factor `∂vz'/∂xfrc_z` (e.g. from
    /// [`StaggeredCoupling::rigid_vz_response`]).
    #[must_use]
    pub const fn new(dvz_dfz: f64) -> Self {
        Self { dvz_dfz }
    }
}

impl VjpOp for RigidStepVjp {
    fn op_id(&self) -> &'static str {
        "sim_coupling::RigidStepVjp"
    }

    // Shape mismatches are programmer bugs (wrong node shape pushed) — assert.
    #[allow(clippy::panic)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        assert!(
            cotangent.shape() == [1] && parent_cotans.len() == 1,
            "RigidStepVjp: expected scalar cotangent [1] and 1 parent (xfrc_z [1]); \
             got cot {:?}, {} parents",
            cotangent.shape(),
            parent_cotans.len(),
        );
        parent_cotans[0].as_mut_slice()[0] += cotangent.as_slice()[0] * self.dvz_dfz;
    }
}

/// Chassis-tape [`VjpOp`] for the contact-force readout — the crossing's
/// soft-contact half (keystone S4).
///
/// Parent = the soft positions `x*` (shape `3·n_vertices`); output = the total
/// `force_on_soft.z` (shape `[1]`). The VJP scatters the per-active-pair precomputed
/// factor `∂fz/∂x_v` (`= −cᵥ·n̂_z·n̂ + f_mag·H·ẑ`, curvature-correct for any collider;
/// flat `−cᵥ·n̂⊗n̂` z-row for the plane, where `cᵥ = d²E/dsd²` is `κ` for penalty,
/// `κ·b''(sd)` for IPC), turning a downstream `∂L/∂fz` into the `∂L/∂x*` cotangent that
/// the soft `NewtonStepVjp` then carries back to the soft parameters. Factors captured at
/// construction from `StaggeredCoupling::active_pair_force_factors` (engaged regime).
#[derive(Clone, Debug)]
pub struct ContactForceVjp {
    /// `(vertex_id, ∂fz/∂x_v, ∂fz/∂height)` per active contact pair at the linearization
    /// positions (the single-step crossing reads only `∂fz/∂x_v`; the `∂fz/∂height` slot
    /// is unused here — its z pose is held fixed). From
    /// `StaggeredCoupling::active_pair_force_factors`.
    factors: Vec<(usize, Vec3, f64)>,
}

impl ContactForceVjp {
    /// Construct from the per-pair precomputed-factor list
    /// `(vertex_id, ∂fz/∂x_v, ∂fz/∂height)` (from
    /// `StaggeredCoupling::active_pair_force_factors`).
    #[must_use]
    pub fn new(factors: Vec<(usize, Vec3, f64)>) -> Self {
        Self { factors }
    }
}

impl VjpOp for ContactForceVjp {
    fn op_id(&self) -> &'static str {
        "sim_coupling::ContactForceVjp"
    }

    // Shape mismatches are programmer bugs (wrong node shape pushed) — assert.
    #[allow(clippy::panic)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        assert!(
            cotangent.shape() == [1] && parent_cotans.len() == 1,
            "ContactForceVjp: expected scalar cotangent [1] and 1 parent (x* [n_dof]); \
             got cot {:?}, {} parents",
            cotangent.shape(),
            parent_cotans.len(),
        );
        let c = cotangent.as_slice()[0];
        let slot = parent_cotans[0].as_mut_slice();
        scatter_dfz_dxstar(&self.factors, c, slot);
    }
}

/// Chassis-tape [`VjpOp`] for the backward-Euler velocity readout
/// `v = (x_curr − x_prev)/Δt` — the linear node threading consecutive soft
/// positions into the next step's `v_prev` over a coupled trajectory
/// (keystone time-adjoint, PR2). Parents `[x_curr, x_prev]` (each `[n_dof]`),
/// output `v` (`[n_dof]`); `∂L/∂x_curr += g/Δt`, `∂L/∂x_prev += −g/Δt`.
#[derive(Clone, Copy, Debug)]
struct VelVjp {
    inv_dt: f64,
    n_dof: usize,
}

impl VjpOp for VelVjp {
    fn op_id(&self) -> &'static str {
        "sim_coupling::VelVjp"
    }

    // Shape mismatches are programmer bugs (wrong node shape pushed) — assert.
    #[allow(clippy::panic)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        assert!(
            cotangent.shape() == [self.n_dof]
                && parent_cotans.len() == 2
                && parent_cotans[0].shape() == [self.n_dof]
                && parent_cotans[1].shape() == [self.n_dof],
            "VelVjp: expected cot [{n}] + 2 parents [{n}]",
            n = self.n_dof,
        );
        let g = cotangent.as_slice();
        let (curr, prev) = parent_cotans.split_at_mut(1);
        let c = curr[0].as_mut_slice();
        let p = prev[0].as_mut_slice();
        for i in 0..self.n_dof {
            c[i] += g[i] * self.inv_dt;
            p[i] -= g[i] * self.inv_dt;
        }
    }
}

/// Chassis-tape [`VjpOp`] for the contact-force readout along a trajectory:
/// `fz = Σ force_on_soft.z` at the post-step soft config `x*` with the collider posed at
/// height `z − clearance`. Parents `[x_star, z]` (`[n_dof]`, `[1]`), output `fz` (`[1]`).
/// Both factors come precomputed per active pair from
/// [`StaggeredCoupling::active_pair_force_factors`] (curvature-correct for any collider):
/// `∂fz/∂x* = −cᵥ·n̂_z·n̂ + f_mag·H·ẑ` (the S3 factor + #415's geometric stiffness) and
/// `∂fz/∂z = ∂fz/∂height = Σ (cᵥ·n̂_z² − f_mag·(H·ẑ)_z)` (the S1 explicit factor,
/// `∂height/∂z=1`). `H = 0` for the plane, where `n̂ = −ẑ` ⇒ this reduces to the flat
/// `−cᵥ·n̂⊗n̂` / `+Σ cᵥ` (penalty `cᵥ = κ`, IPC `κ·b''(sd)`), byte-identical.
#[derive(Clone, Debug)]
struct ContactForceTrajVjp {
    /// `(vertex_id, ∂fz/∂x_v, ∂fz/∂height)` per active contact pair, from
    /// [`StaggeredCoupling::active_pair_force_factors`].
    factors: Vec<(usize, Vec3, f64)>,
    n_dof: usize,
}

impl VjpOp for ContactForceTrajVjp {
    fn op_id(&self) -> &'static str {
        "sim_coupling::ContactForceTrajVjp"
    }

    // Shape mismatches are programmer bugs (wrong node shape pushed) — assert.
    #[allow(clippy::panic)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        assert!(
            cotangent.shape() == [1]
                && parent_cotans.len() == 2
                && parent_cotans[0].shape() == [self.n_dof]
                && parent_cotans[1].shape() == [1],
            "ContactForceTrajVjp: expected cot [1] + parents (x* [{}], z [1])",
            self.n_dof,
        );
        let c = cotangent.as_slice()[0];
        let (xstar, z) = parent_cotans.split_at_mut(1);
        // ∂fz/∂x* = −cᵥ·n̂_z·n̂ + f_mag·H·ẑ (shared with ContactForceVjp).
        scatter_dfz_dxstar(&self.factors, c, xstar[0].as_mut_slice());
        // ∂fz/∂z = Σ (cᵥ·n̂_z² − f_mag·(H·ẑ)_z) (∂height/∂z = 1) — the trajectory-only
        // term (Σ cᵥ for the flat plane). The precomputed per-pair ∂fz/∂height factor.
        let sum_dfz_dz: f64 = self.factors.iter().map(|&(_, _, dfz_dz)| dfz_dz).sum();
        z[0].as_mut_slice()[0] += c * sum_dfz_dz;
    }
}

/// Chassis-tape [`VjpOp`] for the rigid platen's semi-implicit-Euler velocity
/// update `vz' = a·vz − (Δt/m)·fz + Δt·g` along a trajectory, with
/// `a = 1 − Δt·c/m` (linear contact-axis damping `c`) — the rigid carry's
/// velocity half. Parents `[vz, fz]` (`[1]`, `[1]`), output `vz'` (`[1]`); the
/// constant gravity term drops out. `∂vz'/∂vz = a`, `∂vz'/∂fz = −Δt/m`.
#[derive(Clone, Copy, Debug)]
struct VzCarryVjp {
    a: f64,
    neg_dt_over_m: f64,
}

impl VjpOp for VzCarryVjp {
    fn op_id(&self) -> &'static str {
        "sim_coupling::VzCarryVjp"
    }

    // Shape mismatches are programmer bugs (wrong node shape pushed) — assert.
    #[allow(clippy::panic)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        assert!(
            cotangent.shape() == [1]
                && parent_cotans.len() == 2
                && parent_cotans[0].shape() == [1]
                && parent_cotans[1].shape() == [1],
            "VzCarryVjp: expected cot [1] + 2 scalar parents (vz, fz)",
        );
        let c = cotangent.as_slice()[0];
        parent_cotans[0].as_mut_slice()[0] += c * self.a;
        parent_cotans[1].as_mut_slice()[0] += c * self.neg_dt_over_m;
    }
}

/// Chassis-tape [`VjpOp`] for the rigid platen's velocity update when a per-step
/// **control force** `u` is applied to the platen alongside the contact reaction:
/// `vz' = a·vz − (Δt/m)·fz + (Δt/m)·u + Δt·g`. The control parent's coefficient
/// is `∂vz'/∂u = +Δt/m` — the same free-body `Δt/m` factor as the contact term
/// but with the opposite sign (the reaction enters as `−fz`, the control pushes
/// `+`). Parents `[vz, fz, u]` (all `[1]`), output `vz'` (`[1]`); the constant
/// gravity term drops out. This is the control analogue of [`VzCarryVjp`] — kept
/// separate so the passive (material-gradient) trajectory path stays a 2-parent
/// node, byte-unchanged.
#[derive(Clone, Copy, Debug)]
struct VzControlCarryVjp {
    a: f64,
    neg_dt_over_m: f64,
    dt_over_m: f64,
}

impl VjpOp for VzControlCarryVjp {
    fn op_id(&self) -> &'static str {
        "sim_coupling::VzControlCarryVjp"
    }

    // Shape mismatches are programmer bugs (wrong node shape pushed) — assert.
    #[allow(clippy::panic)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        assert!(
            cotangent.shape() == [1]
                && parent_cotans.len() == 3
                && parent_cotans.iter().all(|c| c.shape() == [1]),
            "VzControlCarryVjp: expected cot [1] + 3 scalar parents (vz, fz, u)",
        );
        let c = cotangent.as_slice()[0];
        parent_cotans[0].as_mut_slice()[0] += c * self.a;
        parent_cotans[1].as_mut_slice()[0] += c * self.neg_dt_over_m;
        parent_cotans[2].as_mut_slice()[0] += c * self.dt_over_m;
    }
}

/// Chassis-tape [`VjpOp`] for the rigid platen's position update
/// `z' = z + Δt·vz` along a trajectory — the rigid carry's position half. The
/// velocity is the PRE-update `vz` (the step's starting velocity), NOT the freshly
/// updated `vz'`: sim-core integrates position with the old velocity, so this step's
/// contact force reaches the height only on the NEXT step (verified `z_next ==
/// z + dt·vz_prev` to machine zero). Parents `[z, vz]` (`[1]`, `[1]`), output `z'`
/// (`[1]`); `∂z'/∂z = 1`, `∂z'/∂vz = Δt`.
#[derive(Clone, Copy, Debug)]
struct ZCarryVjp {
    dt: f64,
}

impl VjpOp for ZCarryVjp {
    fn op_id(&self) -> &'static str {
        "sim_coupling::ZCarryVjp"
    }

    // Shape mismatches are programmer bugs (wrong node shape pushed) — assert.
    #[allow(clippy::panic)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        assert!(
            cotangent.shape() == [1]
                && parent_cotans.len() == 2
                && parent_cotans[0].shape() == [1]
                && parent_cotans[1].shape() == [1],
            "ZCarryVjp: expected cot [1] + 2 scalar parents (z, vz)",
        );
        let c = cotangent.as_slice()[0];
        parent_cotans[0].as_mut_slice()[0] += c;
        parent_cotans[1].as_mut_slice()[0] += c * self.dt;
    }
}

/// Chassis-tape [`VjpOp`] for the within-step collider drift `Δ_surf = vx·dt` from the
/// rigid platen's tangential velocity — the moving-collider grip's velocity→drift map
/// (the friction tape's tangential analogue of the position carry). One parent `[vx]`
/// (`[1]`), output `Δ_surf` (`[1]`); `∂Δ_surf/∂vx = dt`. `vx` is the step's STARTING
/// tangential velocity (the drift is read before the rigid step, like `ZCarryVjp`'s
/// pre-update velocity).
#[derive(Clone, Copy, Debug)]
struct DriftFromVelVjp {
    dt: f64,
}

impl VjpOp for DriftFromVelVjp {
    fn op_id(&self) -> &'static str {
        "sim_coupling::DriftFromVelVjp"
    }

    // Shape mismatches are programmer bugs (wrong node shape pushed) — assert.
    #[allow(clippy::panic)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        assert!(
            cotangent.shape() == [1] && parent_cotans.len() == 1 && parent_cotans[0].shape() == [1],
            "DriftFromVelVjp: expected cot [1] + 1 scalar parent (vx)",
        );
        let c = cotangent.as_slice()[0];
        parent_cotans[0].as_mut_slice()[0] += c * self.dt;
    }
}

/// Chassis-tape [`VjpOp`] for the within-step collider drift `Δ_surf,x = (J_lin·qvel)_x·dt`
/// from the ARTICULATED rigid state — the multi-DOF successor to [`DriftFromVelVjp`] (which
/// reads a free platen's scalar `vx`). The tangential drift's x-component is the contact
/// body's COM linear velocity along x̂ times `dt`; `J_lin = ∂c/∂q` is the COM linear Jacobian
/// ([`StaggeredCoupling::com_linear_jacobian`]), so `∂Δ_surf,x/∂qvel = dt·J_lin[0,:]` (the
/// x-row) and `∂Δ_surf,x/∂qpos = 0` (the drift is linear in `qvel` at the linearization
/// config; `J_lin(q)` enters only at second order, captured by the per-step fresh FK). One
/// parent `[s]` (the rigid state `[qpos(nv); qvel(nv)]`, `[2·nv]`), output `Δ_surf,x` (`[1]`).
#[derive(Clone, Debug)]
struct DriftFromStateVjp {
    dt: f64,
    /// The x-row of the COM linear Jacobian `J_lin[0,:]` (length `nv`).
    jlin_x: Vec<f64>,
}

impl VjpOp for DriftFromStateVjp {
    fn op_id(&self) -> &'static str {
        "sim_coupling::DriftFromStateVjp"
    }

    // Shape mismatches are programmer bugs (wrong node shape pushed) — assert.
    #[allow(clippy::panic)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        let nv = self.jlin_x.len();
        assert!(
            cotangent.shape() == [1]
                && parent_cotans.len() == 1
                && parent_cotans[0].shape() == [2 * nv],
            "DriftFromStateVjp: expected cot [1] + 1 parent (s [{}])",
            2 * nv,
        );
        let c = cotangent.as_slice()[0];
        let s_slot = parent_cotans[0].as_mut_slice();
        for j in 0..nv {
            // qvel rows only (nv + j); qpos rows untouched (∂Δ_surf/∂qpos = 0).
            s_slot[nv + j] += c * self.dt * self.jlin_x[j];
        }
    }
}

/// Chassis-tape [`VjpOp`] extracting one scalar COMPONENT of the rigid state `s = [qpos; qvel]`:
/// `out = s[idx]` (`[1]`), `∂out/∂s[idx] = 1` (all other rows 0). The articulated POLICY
/// gradient's observation extractor — projecting the carried `2·nv` state var to a scalar the
/// [`DiffPolicy`] observes (e.g. a single hinge's joint angle `s[0]` or rate `s[nv]`), so the
/// closed-loop control `u = π_θ(state)` feeds back through the matrix carry.
#[derive(Clone, Copy, Debug)]
struct StateComponentVjp {
    /// The component index into `s = [qpos(nv); qvel(nv)]`.
    idx: usize,
    /// The state length `2·nv` (parent shape).
    n_state: usize,
}

impl VjpOp for StateComponentVjp {
    fn op_id(&self) -> &'static str {
        "sim_coupling::StateComponentVjp"
    }

    // Shape mismatches are programmer bugs (wrong node shape pushed) — assert.
    #[allow(clippy::panic)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        assert!(
            cotangent.shape() == [1]
                && parent_cotans.len() == 1
                && parent_cotans[0].shape() == [self.n_state],
            "StateComponentVjp: expected cot [1] + 1 parent (s [{}])",
            self.n_state,
        );
        parent_cotans[0].as_mut_slice()[self.idx] += cotangent.as_slice()[0];
    }
}

/// Chassis-tape [`VjpOp`] for the **tangential friction-reaction** readout along a
/// trajectory: the scalar `fx = (Σ ∇D)·react_dir` (the rigid-side reaction `F·react_dir`
/// from [`CpuNewtonSolver::friction_reaction_gradients`]) at the post-step soft config `x*`,
/// with the moving-collider drift `Δ_surf` and the plane at height `z − clearance` — the
/// friction successor to [`ContactForceTrajVjp`] (which reads the normal `fz`). The caller
/// passes `react_dir = −x̂` so this equals `force_on_soft.x`, letting the tangential rigid
/// carry reuse the normal carry's `−(Δt/m)·fx` reaction sign. Parents
/// `[x*, z, Δ_surf, x_prev]` (`[n_dof]`, `[1]`, `[1]`, `[n_dof]`), output `fx` (`[1]`). The
/// sensitivities are precomputed once by [`CpuNewtonSolver::friction_reaction_gradients`]:
/// `dforce_dx = ∂fx/∂x*` (frozen-lag slip + normal-force λ-coupling), `dforce_dheight =
/// ∂fx/∂height` (`∂height/∂z = 1`), `dforce_ddrift = ∂fx/∂Δ_surf`, and `dforce_dxprev =
/// ∂fx/∂x_prev` (the friction reference `x_start = x_prev + Δ_surf` makes `fx` depend on the
/// step-start config — the state companion of the drift term).
///
/// `dforce_dmu_c` adds an OPTIONAL fifth parent: the Coulomb friction coefficient `μ_c`. The
/// reaction is `fx = (Σ μ_c·λⁿ·t·grad2)·react_dir`, LINEAR in `μ_c`, so `∂fx/∂μ_c = fx/μ_c` at
/// fixed `x*` — a DIRECT channel that does not flow through `x*` (the material parameter has no
/// such term; its only path to `fx` is via `x*`). `None` ⇒ four parents, byte-identical to the
/// material path; `Some` ⇒ five parents `[x*, z, Δ_surf, x_prev, μ_c]` for the `∂/∂μ_c` gradient.
#[derive(Clone, Debug)]
struct FrictionReactionTrajVjp {
    dforce_dx: Vec<f64>,
    dforce_dxprev: Vec<f64>,
    dforce_dheight: f64,
    dforce_ddrift: f64,
    dforce_dmu_c: Option<f64>,
    n_dof: usize,
}

impl VjpOp for FrictionReactionTrajVjp {
    fn op_id(&self) -> &'static str {
        "sim_coupling::FrictionReactionTrajVjp"
    }

    // Shape mismatches are programmer bugs (wrong node shape pushed) — assert.
    #[allow(clippy::panic)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        let want = if self.dforce_dmu_c.is_some() { 5 } else { 4 };
        assert!(
            cotangent.shape() == [1]
                && parent_cotans.len() == want
                && parent_cotans[0].shape() == [self.n_dof]
                && parent_cotans[1].shape() == [1]
                && parent_cotans[2].shape() == [1]
                && parent_cotans[3].shape() == [self.n_dof]
                && self
                    .dforce_dmu_c
                    .is_none_or(|_| parent_cotans[4].shape() == [1]),
            "FrictionReactionTrajVjp: expected cot [1] + parents (x* [{n}], z [1], Δ_surf [1], \
             x_prev [{n}]{mu})",
            n = self.n_dof,
            mu = if self.dforce_dmu_c.is_some() {
                ", μ_c [1]"
            } else {
                ""
            },
        );
        let c = cotangent.as_slice()[0];
        let (xstar, rest) = parent_cotans.split_at_mut(1);
        let xs = xstar[0].as_mut_slice();
        for (i, &g) in self.dforce_dx.iter().enumerate() {
            xs[i] += c * g;
        }
        rest[0].as_mut_slice()[0] += c * self.dforce_dheight; // z (∂height/∂z = 1)
        rest[1].as_mut_slice()[0] += c * self.dforce_ddrift; // Δ_surf
        let xprev = rest[2].as_mut_slice(); // x_prev
        for (i, &g) in self.dforce_dxprev.iter().enumerate() {
            xprev[i] += c * g;
        }
        if let Some(dmu) = self.dforce_dmu_c {
            rest[3].as_mut_slice()[0] += c * dmu; // μ_c (∂fx/∂μ_c = fx/μ_c)
        }
    }
}

/// Chassis-tape [`VjpOp`] for the **multi-DOF** pose seam of an articulated rigid
/// body: the contact-plane height tracks the contacting point's world height
/// `h(q) = (FK of the contact point).z`, so `∂h/∂q = J_z` (the world-z row of the
/// body Jacobian at the contact point). Parent `[s]` (the rigid state `[qpos(nv);
/// qvel(nv)]`, shape `[2·nv]`), output `h` (`[1]`); `∂h/∂qpos = J_z`, `∂h/∂qvel = 0`.
/// Generalizes the platen's scalar `∂plane/∂z = 1` (the 1-DOF special case where
/// `J_z = [1]`). Keystone multi-DOF coupling, PR2.
#[derive(Clone, Debug)]
struct PoseSeamVjp {
    /// `J_z`: the world-z row of the contact-point body Jacobian (length `nv`).
    jz: Vec<f64>,
}

impl VjpOp for PoseSeamVjp {
    fn op_id(&self) -> &'static str {
        "sim_coupling::PoseSeamVjp"
    }

    // Shape mismatches are programmer bugs (wrong node shape pushed) — assert.
    #[allow(clippy::panic)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        let nv = self.jz.len();
        assert!(
            cotangent.shape() == [1]
                && parent_cotans.len() == 1
                && parent_cotans[0].shape() == [2 * nv],
            "PoseSeamVjp: expected cot [1] + 1 parent (state [2·nv])",
        );
        let c = cotangent.as_slice()[0];
        let slot = parent_cotans[0].as_mut_slice();
        // ∂h/∂qpos = J_z (position half); ∂h/∂qvel = 0 (velocity half untouched).
        for (i, &j) in self.jz.iter().enumerate() {
            slot[i] += c * j;
        }
    }
}

/// The **moving-end-effector** pose seam — the 3-vector generalization of
/// [`PoseSeamVjp`] (scalar height). When the contact sphere rides a rigid geom (the arm
/// tip, [`StaggeredCoupling::with_contact_geom`]), its centre translates in x/y/z as the
/// body moves, so the pose channel is the 3-vector centre `c = geom_xpos(q)` rather than
/// the scalar height. Threads `∂L/∂qpos = J_geomᵀ·∂L/∂centre` (position half;
/// `∂centre/∂qvel = 0`, velocity half untouched), where `J_geom = ∂centre/∂qpos` is
/// [`StaggeredCoupling::pose_centre_jacobian`] (the geom's linear world Jacobian).
#[derive(Debug)]
struct PoseCentreVjp {
    /// `J_geom`: the contact-geom centre's linear world Jacobian (`3 × nv`).
    j_geom: DMatrix<f64>,
}

impl VjpOp for PoseCentreVjp {
    fn op_id(&self) -> &'static str {
        "sim_coupling::PoseCentreVjp"
    }

    // Shape mismatches are programmer bugs (wrong node shape pushed) — assert.
    // needless_range_loop: the `∂L/∂qpos = J_geomᵀ·cot` contraction indexes the 2-D `j_geom`
    // by (row, col j) and `slot` by j; explicit indices read clearer (as in `ContactWrenchTrajVjp`).
    #[allow(clippy::panic, clippy::needless_range_loop)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        let nv = self.j_geom.ncols();
        assert!(
            cotangent.shape() == [3]
                && parent_cotans.len() == 1
                && parent_cotans[0].shape() == [2 * nv],
            "PoseCentreVjp: expected cot [3] + 1 parent (state [2·nv])",
        );
        let cot = cotangent.as_slice();
        let slot = parent_cotans[0].as_mut_slice();
        // ∂L/∂qpos = J_geomᵀ·cot (position half); ∂L/∂qvel = 0 (velocity half untouched).
        for j in 0..nv {
            let mut acc = 0.0_f64;
            for row in 0..3 {
                acc += self.j_geom[(row, j)] * cot[row];
            }
            slot[j] += acc;
        }
    }
}

/// The 6 canonical spatial-twist basis directions `(ω, v)` — 3 angular then 3 linear,
/// matching [`StaggeredCoupling::pose_twist_jacobian`]'s row layout (`mj_jac_point`
/// rows 0–2 angular, 3–5 linear). The rotating-normal pose channel is expressed in
/// this rigid-agnostic basis (the soft pose-adjoint and the wrench `∂w/∂T`); the
/// [`PoseTwistSeamVjp`] seam then maps the twist cotangent through `J_spatial`.
fn twist_basis() -> [RigidTwist; 6] {
    let e = |i: usize| {
        let mut v = Vec3::zeros();
        v[i] = 1.0;
        v
    };
    [
        RigidTwist {
            angular: e(0),
            linear: Vec3::zeros(),
        },
        RigidTwist {
            angular: e(1),
            linear: Vec3::zeros(),
        },
        RigidTwist {
            angular: e(2),
            linear: Vec3::zeros(),
        },
        RigidTwist {
            angular: Vec3::zeros(),
            linear: e(0),
        },
        RigidTwist {
            angular: Vec3::zeros(),
            linear: e(1),
        },
        RigidTwist {
            angular: Vec3::zeros(),
            linear: e(2),
        },
    ]
}

/// Chassis-tape [`VjpOp`] for the **rotating-normal** pose seam: the contact plane is
/// rigidly attached to the body, so its 6-DOF spatial **twist** `T` (3 angular + 3
/// linear, the soft side's [`sim_soft::RigidTwist`] basis) is a function of the rigid
/// state, `∂T/∂qpos = J_spatial` (the body's spatial Jacobian at the world origin —
/// rows 0–2 angular, 3–5 linear `v_O`). Parent `[s]` (`[2·nv]`), output `T` (`[6]`);
/// `∂L/∂qpos = J_spatialᵀ·∂L/∂T`, `∂L/∂qvel = 0`.
///
/// This is the rotating-normal generalization of [`PoseSeamVjp`] (which threads only
/// the scalar height `h`, `∂h/∂q = J_z`): the soft solve's pose-adjoint and the
/// contact-wrench readout both consume the 6-DOF twist (`δn̂ = ω×n̂`, `δoffset = v·n̂`),
/// and this seam maps the assembled twist cotangent back to the rigid DOFs. The twist
/// node's value is the all-zero perturbation at the linearization point (the soft /
/// wrench nodes carry the real `x*` / wrench values); only its cotangent is used.
/// FD-validated via the full-coupled rotating-normal gradient gate. Keystone
/// rotating-normal leaf, PR2.
#[derive(Clone, Debug)]
struct PoseTwistSeamVjp {
    /// `J_spatial`: the body's spatial Jacobian at the world origin (`6 × nv`,
    /// `mj_jac_point` rows 0–2 angular / 3–5 linear). `∂T/∂qpos`.
    j_spatial: DMatrix<f64>,
}

impl VjpOp for PoseTwistSeamVjp {
    fn op_id(&self) -> &'static str {
        "sim_coupling::PoseTwistSeamVjp"
    }

    // Shape mismatches are programmer bugs (wrong node shape pushed) — assert.
    // needless_range_loop: the `∂L/∂qpos = J_spatialᵀ·cot` contraction indexes the
    // 2-D `j_spatial` by (row k, col i) and `cot` by k; explicit indices read clearer.
    #[allow(clippy::panic, clippy::needless_range_loop)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        let nv = self.j_spatial.ncols();
        assert!(
            cotangent.shape() == [6]
                && parent_cotans.len() == 1
                && parent_cotans[0].shape() == [2 * nv],
            "PoseTwistSeamVjp: expected cot [6] + 1 parent (state [2·nv])",
        );
        let cot = cotangent.as_slice();
        let slot = parent_cotans[0].as_mut_slice();
        // ∂L/∂qpos = J_spatialᵀ·∂L/∂T (position half); ∂L/∂qvel = 0.
        for i in 0..nv {
            let mut acc = 0.0_f64;
            for k in 0..6 {
                acc += self.j_spatial[(k, i)] * cot[k];
            }
            slot[i] += acc;
        }
    }
}

/// Chassis-tape [`VjpOp`] for the contact **wrench** readout along an articulated
/// trajectory: the full reaction spatial force `w = [τ; f]` the soft body applies
/// to the rigid body at its COM `c = xipos`, generalizing the scalar
/// [`ContactForceTrajVjp`] (which carries only `force_on_soft.z`) to the off-COM
/// contact MOMENT. Parents `[x*, h, s]` (`[n_dof]`, `[1]`, `[2·nv]`), output `w`
/// (`[6]`, layout `[τ(3); f(3)]`):
///
/// ```text
/// f = −Σ gᵢ                       (reaction force, rows 3–5)
/// τ = −Σ (rᵢ − c) × gᵢ           (moment about the COM c, rows 0–2)
/// ```
///
/// with per-pair soft-force `gᵢ = force_on_softᵢ` at world point `rᵢ = x*[vᵢ]`.
/// `∂w/∂x*` has TWO parts per pair (`d = rᵢ − c`, curvature `cᵥ`, normal `n̂`,
/// `∂gᵢ/∂x_v = −cᵥ n̂⊗n̂`):
/// - force rows: `∂f/∂x_v = +cᵥ n̂⊗n̂`;
/// - torque rows: `∂τ/∂x_v = [gᵢ]_× + cᵥ [d]_× (n̂⊗n̂)` — the explicit `rᵢ`-part
///   `[gᵢ]_×` plus the `gᵢ`-part `cᵥ[d]_×(n̂⊗n̂)`.
///
/// `w` also depends on the plane height `h` (the pose seam, `= h(q)`) through each
/// pair's force magnitude `gᵢ = cᵥ(d̂ − sdᵢ)n̂` with `sdᵢ = h − z_vᵢ`, so
/// `∂gᵢ/∂h = −cᵥ n̂` (the S1 explicit force-vs-height factor): `∂f/∂h = Σ cᵥ n̂` and
/// `∂τ/∂h = Σ cᵥ (dᵢ × n̂)`. This is the moment generalization of the merged scalar
/// `ContactForceTrajVjp`'s `∂fz/∂h = +Σ cᵥ`; the implicit soft re-equilibration
/// w.r.t. `h` flows separately through the soft node's pose parent.
///
/// `∂w/∂s` enters only through `c = xipos(q)`: `∂τ/∂qpos = [f]_× · J_lin` (with
/// `J_lin` the COM linear Jacobian, `∂τ/∂qvel = 0`, `∂f/∂s = 0`). This c(q) channel
/// is distinct from the h(q) channel and is small in the keystone scene; it does
/// NOT double-count the loaded `J_state` (which holds the wrench fixed while
/// perturbing `q`). See `docs/keystone/contact_moment_recon.md` §3.
///
/// **Rotating-normal (`WrenchPose::Twist`).** When the plane normal tracks the body
/// orientation, the pose parent is the 6-DOF spatial **twist** `T` (not the scalar
/// `h`), and `w` gains the normal-rotation feedback the flat `∂w/∂h` drops. Per basis
/// twist `e_k = (ω_k, v_k)` (`dn_k = ω_k×n̂`, `dsd_k = rᵢ·dn_k − v_k·n̂`), the contact
/// force varies by `∂gᵢ/∂T_k = −cᵥ·dsd_k·n̂ + (gᵢ·n̂)·dn_k` (the `(gᵢ·n̂)·dn_k` term —
/// the force redirecting as `n̂` rotates — is NEW vs the flat path), so
/// `∂L/∂T_k = −Σᵢ (∂gᵢ/∂T_k)·(cot_f + cot_τ×dᵢ)`.
#[derive(Clone, Debug)]
struct ContactWrenchTrajVjp {
    /// Per active pair `(vertex_id, soft-force gᵢ, normal n̂, curvature cᵥ,
    /// moment arm d = rᵢ − c, collider normal-curvature Hᵢ = ∇²sd)` at the
    /// linearization config. `Hᵢ = 0` for the constant-normal plane; for a finite curved
    /// collider it carries the geometric-stiffness term `f_mag·H` of `∂gᵢ/∂x_v` and `∂gᵢ/∂h`.
    active: Vec<(usize, Vec3, Vec3, f64, Vec3, Matrix3<f64>)>,
    /// The reaction force `f = −Σ gᵢ` (cached for `∂τ/∂c = [f]_×`).
    force: Vec3,
    /// The COM linear Jacobian `J_lin = ∂c/∂qpos` (`3 × nv`).
    jlin: DMatrix<f64>,
    /// Soft-DOF count `3·n_vertices` (parent `x*` length).
    n_dof: usize,
    /// Rigid-DOF count `nv` (parent `s` length is `2·nv`).
    nv: usize,
    /// The pose channel: scalar height `h` (flat) or 6-DOF twist `T` (rotating).
    pose: WrenchPose,
}

/// The contact-wrench node's pose dependence — the middle parent of
/// [`ContactWrenchTrajVjp`]. `Height` is the flat scalar-`h` factor (parent `[1]`);
/// `Twist` is the rotating-normal 6-DOF spatial-twist factor (parent `[6]`).
#[derive(Clone, Debug)]
enum WrenchPose {
    /// Flat plane: `∂w/∂h` (the S1 explicit force/moment-vs-height factor).
    Height,
    /// Moving end-effector: `∂w/∂(centre)` per translation basis direction — the
    /// 3-vector generalization of [`WrenchPose::Height`] (which is the `ẑ`-only
    /// channel). The contact sphere rides a rigid geom (the arm tip) that translates
    /// in x/y/z as the body moves, so the pose parent is the 3-vector centre
    /// (`∂centre/∂q = J_geom`, the `PoseCentreVjp` seam) rather than the scalar
    /// height. `basis = [x̂, ŷ, ẑ]` recovers the `Height` branch on its z component.
    Centre { basis: Vec<Vec3> },
    /// Rotating normal: `∂w/∂T` per spatial-twist basis direction. `com = xipos`
    /// recovers the soft contact point `rᵢ = dᵢ + com` the `sd` derivative needs.
    Twist { basis: Vec<RigidTwist>, com: Vec3 },
}

impl VjpOp for ContactWrenchTrajVjp {
    fn op_id(&self) -> &'static str {
        "sim_coupling::ContactWrenchTrajVjp"
    }

    // Shape mismatches are programmer bugs (wrong node shape pushed) — assert.
    // needless_range_loop: the `∂L/∂qpos = J_linᵀ·m` contraction indexes the 2-D
    // `jlin` by (row, col j) and `s_slot` by j; explicit indices read clearer here.
    #[allow(clippy::panic, clippy::needless_range_loop)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        let n_pose = match &self.pose {
            WrenchPose::Height => 1,
            WrenchPose::Centre { basis } => basis.len(),
            WrenchPose::Twist { basis, .. } => basis.len(),
        };
        assert!(
            cotangent.shape() == [6]
                && parent_cotans.len() == 3
                && parent_cotans[0].shape() == [self.n_dof]
                && parent_cotans[1].shape() == [n_pose]
                && parent_cotans[2].shape() == [2 * self.nv],
            "ContactWrenchTrajVjp: expected cot [6] + parents (x* [{}], pose [{n_pose}], s [{}])",
            self.n_dof,
            2 * self.nv,
        );
        let cot = cotangent.as_slice();
        let cot_t = Vec3::new(cot[0], cot[1], cot[2]); // cotangent on τ
        let cot_f = Vec3::new(cot[3], cot[4], cot[5]); // cotangent on f

        // ∂L/∂x*: per pair, the transpose-applied 6×3 block (see the type doc).
        //   force rows:  +cᵥ (n̂·cot_f) n̂
        //   torque rows: −(gᵢ × cot_τ)  −  cᵥ (n̂·(d × cot_τ)) n̂
        let xs = parent_cotans[0].as_mut_slice();
        for &(v, g, n, curv, d, h) in &self.active {
            // Flat part: ∂g/∂x_v = −cᵥ n̂⊗n̂ (constant normal).
            let mut term =
                curv * n.dot(&cot_f) * n - g.cross(&cot_t) - curv * n.dot(&d.cross(&cot_t)) * n;
            // Curved-normal geometric stiffness: the tangent frame turns as the vertex slides
            // over the primitive, so ∂g/∂x_v gains `+f_mag·H` (`f_mag = gᵢ·n̂`, `H = ∇²sd`, the
            // [`Self::collider_hessian`]). The force rows pick up `−f_mag·H·cot_f`, the torque
            // g-part `+f_mag·H·(d×cot_t)`. `H = 0` for a plane ⇒ `+0` (byte-identical).
            let f_mag = g.dot(&n);
            term += f_mag * (h * (d.cross(&cot_t) - cot_f));
            xs[3 * v] += term.x;
            xs[3 * v + 1] += term.y;
            xs[3 * v + 2] += term.z;
        }

        // ∂L/∂pose: flat scalar `h` or the rotating-normal 6-DOF twist.
        let pose_slot = parent_cotans[1].as_mut_slice();
        match &self.pose {
            WrenchPose::Height => {
                // ∂L/∂h: the explicit force/moment-vs-height feedback (∂gᵢ/∂h = −cᵥ n̂ + curved).
                //   flat:    ∂f/∂h = Σ cᵥ n̂ ,  ∂τ/∂h = Σ cᵥ (d × n̂).
                //   curved:  ∂n̂/∂h = −H·ẑ, so ∂gᵢ/∂h gains `f_mag·(−H·ẑ)` ⇒ the readout gains
                //   `f_mag·(H·ẑ)·(cot_f + cot_t×d)`. `H = 0` for a plane ⇒ `+0` (byte-identical).
                let zhat = Vec3::new(0.0, 0.0, 1.0);
                let dh: f64 = self
                    .active
                    .iter()
                    .map(|&(_, g, n, curv, d, h)| {
                        // Magnitude: ∂sd/∂h = −n̂·ẑ = −n̂_z (the primitive translates +ẑ), so the
                        // force-magnitude feedback is `−cᵥ·n̂_z` per pair — NOT the plane-baked
                        // `cᵥ` (the flat downward plane has n̂_z = −1 ⇒ `−cᵥ·(−1) = cᵥ`,
                        // byte-identical; a sphere's tilted off-pole normals need the n̂_z factor).
                        let flat = -curv * n.z * (n.dot(&cot_f) + d.cross(&n).dot(&cot_t));
                        let curved = g.dot(&n) * (h * zhat).dot(&(cot_f + cot_t.cross(&d)));
                        flat + curved
                    })
                    .sum();
                pose_slot[0] += dh;
            }
            WrenchPose::Centre { basis } => {
                // ∂L/∂(centre translation along dir): the moving-end-effector generalization
                // of the `Height` (ẑ-only) branch. Per axis `dir`, the EXPLICIT force change
                // (x* held — the implicit ∂x*/∂centre rides the soft node's pose parent) is
                //   ∂gᵢ/∂(centre·dir) = cᵥ·(n̂·dir)·n̂ − f_mag·(H·dir)
                // (magnitude `∂sd/∂(centre·dir) = −n̂·dir` ⇒ `+cᵥ·(n̂·dir)·n̂`; normal rotation
                // `∂n̂/∂centre = −H·dir` ⇒ `−f_mag·(H·dir)`), folded into the wrench readout as
                //   ∂L/∂dir = Σᵢ −∂gᵢ·(cot_f + cot_τ×dᵢ).
                // `dir = ẑ` reproduces `Height` term-for-term (`n̂·ẑ = n̂_z`, `H·ẑ`); `H = 0`
                // for a plane ⇒ only the magnitude term (also exact). See
                // [`StaggeredCoupling::contact_force_centre_total_jacobian`] (the EXPLICIT half).
                for (k, dir) in basis.iter().enumerate() {
                    let mut acc = 0.0_f64;
                    for &(_, g, n, curv, d, h) in &self.active {
                        let co = cot_f + cot_t.cross(&d);
                        let f_mag = g.dot(&n);
                        acc += -curv * n.dot(dir) * n.dot(&co) + f_mag * (h * dir).dot(&co);
                    }
                    pose_slot[k] += acc;
                }
            }
            WrenchPose::Twist { basis, com } => {
                // ∂L/∂T_k = −Σᵢ (∂gᵢ/∂T_k)·(cot_f + cot_τ×dᵢ), with
                //   dn_k = ω_k×n̂,  dsd_k = rᵢ·dn_k − v_k·n̂  (rᵢ = dᵢ + com),
                //   ∂gᵢ/∂T_k = −cᵥ·dsd_k·n̂ + (gᵢ·n̂)·dn_k.
                for (k, tw) in basis.iter().enumerate() {
                    let mut acc = 0.0_f64;
                    for &(_, g, n, curv, d, _h) in &self.active {
                        let dn = tw.angular.cross(&n);
                        let r = d + com;
                        let dsd = r.dot(&dn) - tw.linear.dot(&n);
                        let dg = -curv * dsd * n + g.dot(&n) * dn;
                        let co = cot_f + cot_t.cross(&d);
                        acc -= dg.dot(&co);
                    }
                    pose_slot[k] += acc;
                }
            }
        }

        // ∂L/∂s: only the qpos rows, only the moment's c(q)-dependence.
        //   ∂L/∂qpos = J_linᵀ · m,  m = −(f × cot_τ)  (= [f]_×ᵀ cot_τ).
        let m = -self.force.cross(&cot_t);
        let s_slot = parent_cotans[2].as_mut_slice();
        for j in 0..self.nv {
            s_slot[j] +=
                self.jlin[(0, j)] * m.x + self.jlin[(1, j)] * m.y + self.jlin[(2, j)] * m.z;
            // qvel rows (nv + j) untouched: ∂w/∂qvel = 0.
        }
    }
}

/// Chassis-tape [`VjpOp`] for the **multi-DOF** rigid state carry of an articulated
/// body: `s' = J_state · s + G · w`, where `s = [qpos(nv); qvel(nv)]` is the rigid
/// state, `w = [τ; f]` is the contact **wrench** ([`ContactWrenchTrajVjp`]),
/// `J_state` is the **loaded** single-step transition Jacobian `∂(state')/∂(state)`
/// (with the contact wrench held — it includes the applied-force geometric/load
/// stiffness `∂(Jᵀw)/∂q` that the unloaded `transition_derivatives` drops; computed
/// analytically for a single hinge, [`StaggeredCoupling::analytic_state_jacobian`],
/// else by FD, [`StaggeredCoupling::loaded_state_jacobian`]), and `G = ∂(state')/∂w` is the
/// 6-component wrench response. `G`'s VELOCITY rows are the full `nv × 6`
/// [`rigid_xfrc_column`] (`Δt·M⁻¹·Jᵀ`); its POSITION rows are `G_pos = D·G_vel`, where
/// `D = ∂(tangent qpos')/∂qvel'` is the integrator's tangent Jacobian — semi-implicit
/// Euler maps `qpos' = qpos ⊕ exp(Δt·qvel')`, so this step's wrench reaches `qpos` through
/// `qvel'`. For a Euclidean DOF (hinge/slide/translation) `D = Δt·I`, so `G_pos = Δt·G_vel`;
/// for a quaternion DOF (ball/free orientation) `D = Δt·J_r(Δt·qvel')`, the SO(3) right
/// Jacobian ([`right_jacobian_so3`]) the curved-manifold integrator demands. NO sign flip —
/// `w` is the reaction wrench directly (the negation lives in [`ContactWrenchTrajVjp`]).
///
/// (Historical: an earlier formulation read the contact pose from the one-step-stale FK
/// and ZEROED these position rows — a self-consistent pair, but exact only for `nv = 1`.
/// The fresh-FK formulation + this true `G_pos` term is machine-exact for single-hinge AND
/// multi-link; the `J_r` generalizes the Euclidean `Δt·G_vel` to quaternion joints. See
/// `docs/keystone/moment_residual_recon.md` and `docs/keystone/quaternion_joints_recon.md`.)
///
/// Parents `[s, w]` (`[2·nv]`, `[6]`), output `s'` (`[2·nv]`);
/// `∂L/∂s += J_stateᵀ·cot`, `∂L/∂w[k] += Σᵢ G_vel[(i,k)]·cot[nv+i] + G_pos[(i,k)]·cot[i]`. The
/// matrix-valued successor to the scalar `VzCarryVjp` + `ZCarryVjp` the free-body
/// platen uses; generalized from the f_z-only column to the full spatial wrench.
#[derive(Clone, Debug)]
struct RigidStateCarryVjp {
    /// The loaded transition Jacobian `J_state` (`2·nv × 2·nv`), indexed logically `j_state[(row, col)]`.
    j_state: DMatrix<f64>,
    /// The wrench velocity response `G_vel = ∂(qvel')/∂w` (`nv × 6` = [`rigid_xfrc_column`]).
    g_vel: DMatrix<f64>,
    /// The wrench POSITION-row response `G_pos = ∂(tangent qpos')/∂w = D·G_vel` (`nv × 6`),
    /// `D` the integrator's tangent Jacobian (`Δt·I` Euclidean, `Δt·J_r` quaternion).
    g_pos: DMatrix<f64>,
    /// Optional ACTUATOR-control channel `G_act = ∂s'/∂ctrl` (`2·nv × nu`): the
    /// actuator drives `s' = J_state·s + G·w + G_act·u` where `u = ctrl`. When `Some`,
    /// the node takes a third parent `u` (`[nu]`) and `∂L/∂u[a] += Σᵢ G_act[(i,a)]·cot[i]`.
    /// `None` (the material/passive path) ⇒ two parents, byte-identical to the pre-actuator
    /// carry. Rows are `[G_act_pos; G_act_vel]` with `G_act_vel = Δt·M_impl⁻¹·∂qfrc_act/∂ctrl`
    /// and `G_act_pos = D·G_act_vel` (the same integrator tangent `D` as `G_pos`).
    g_act: Option<DMatrix<f64>>,
}

impl VjpOp for RigidStateCarryVjp {
    fn op_id(&self) -> &'static str {
        "sim_coupling::RigidStateCarryVjp"
    }

    // Shape mismatches are programmer bugs (wrong node shape pushed) — assert.
    // needless_range_loop: the Jᵀ·cot contraction indexes a 2-D matrix by (row i,
    // col j); explicit indices read clearer than zipped row/col iterators here.
    #[allow(clippy::panic, clippy::needless_range_loop)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        let n = self.j_state.nrows(); // 2·nv
        let nv = self.g_vel.nrows();
        let nu = self.g_act.as_ref().map_or(0, DMatrix::ncols);
        let want_parents = if self.g_act.is_some() { 3 } else { 2 };
        let u_shape_ok = self.g_act.is_none() || parent_cotans[2].shape() == [nu];
        assert!(
            cotangent.shape() == [n]
                && parent_cotans.len() == want_parents
                && parent_cotans[0].shape() == [n]
                && parent_cotans[1].shape() == [6]
                && u_shape_ok,
            "RigidStateCarryVjp: expected cot [2·nv] + parents (s [2·nv], w [6][, u [nu]])",
        );
        let cot = cotangent.as_slice();
        // ∂L/∂s += J_stateᵀ·cot.
        let s_slot = parent_cotans[0].as_mut_slice();
        for j in 0..n {
            let mut acc = 0.0;
            for i in 0..n {
                acc += self.j_state[(i, j)] * cot[i];
            }
            s_slot[j] += acc;
        }
        // ∂L/∂w[k] += Σᵢ G_vel[(i,k)]·cot[nv+i] + G_pos[(i,k)]·cot[i].
        let w_slot = parent_cotans[1].as_mut_slice();
        for k in 0..6 {
            let mut acc = 0.0;
            for i in 0..nv {
                acc += self.g_vel[(i, k)] * cot[nv + i]; // velocity rows: G_vel
                acc += self.g_pos[(i, k)] * cot[i]; // position rows: G_pos = D·G_vel
            }
            w_slot[k] += acc;
        }
        // ∂L/∂u[a] += Σᵢ G_act[(i,a)]·cot[i] — the actuator-control channel (full 2·nv
        // rows: G_act already stacks [pos; vel]).
        if let Some(g_act) = &self.g_act {
            let u_slot = parent_cotans[2].as_mut_slice();
            for a in 0..nu {
                let mut acc = 0.0;
                for i in 0..n {
                    acc += g_act[(i, a)] * cot[i];
                }
                u_slot[a] += acc;
            }
        }
    }
}

/// One active friction vertex carried by [`FrictionWrenchTrajVjp`] — the per-vertex friction
/// force on the rigid body and its Jacobians, plus the moment arm about the COM.
#[derive(Clone, Debug)]
struct FrictionWrenchVert {
    /// The contacted soft vertex.
    vid: usize,
    /// `∇D_v` — the friction force on the rigid body at `v`.
    force: Vec3,
    /// `d_v = r_v − c` — the moment arm about the COM `c` (`r_v = x*_v`, the soft vertex pos).
    arm: Vec3,
    /// `∂force/∂x*` — row-major `3 × n_dof` ([`sim_soft::FrictionVertexForce::dforce_dx`]).
    dforce_dx: Vec<f64>,
    /// `∂force/∂x_prev` — row-major `3 × n_dof`.
    dforce_dxprev: Vec<f64>,
    /// `∂force/∂Δ_surf` along the build's `drift_dir`.
    dforce_ddrift: Vec3,
    /// `∂force/∂(centre·e_k)` per pose-basis axis — `[∂force/∂height]` (length 1) for the scalar
    /// height channel, or the 3 translation axes `[x̂, ŷ, ẑ]` for a moving end-effector centre.
    dforce_dpose: Vec<Vec3>,
    /// `∂force/∂μ_c = ∇D_v/μ_c` — the DIRECT Coulomb-coefficient channel (only threaded when the
    /// node carries a `μ_c` parent, i.e. the friction-coefficient gradient).
    dforce_dmu_c: Vec3,
}

/// Chassis-tape [`VjpOp`] folding the full TANGENTIAL friction GRIP wrench into the spatial
/// wrench: `w = w_normal + [τ_fric; f_fric]` with `f_fric = Σ_v ∇D_v` and the off-COM friction
/// MOMENT `τ_fric = Σ_v (r_v − c) × ∇D_v`. The moment successor to the force-only fold: it
/// threads the per-vertex friction Jacobians ([`sim_soft::FrictionVertexForce`], from
/// [`CpuNewtonSolver::friction_force_jacobians`]) through BOTH the force AND moment rows.
///
/// Per vertex the effective cotangent on `∇D_v` is `co_v = cot_f + cot_τ × d_v` (the force part
/// plus the moment-via-`∇D_v` part, since `(d×f)·cot_τ = f·(cot_τ×d)`); the moment's ARM part
/// adds `∇D_v × cot_τ` to `v`'s own soft coords (`r_v = x*_v`); and the COM `c(q)` feedback is
/// `m = −f_fric_total × cot_τ` threaded through `J_lin` (as in [`ContactWrenchTrajVjp`]'s
/// `∂L/∂s`). Parents `[w_normal, x*, pose, s, drift, x_prev]`
/// (`[6]`, `[3n]`, `[1]`, `[2·nv]`, `[1]`, `[3n]`), output `w` (`[6]`).
///
/// With [`Self::mu_c`] set, the node takes a 7th parent — the Coulomb coefficient `μ_c` — and
/// threads the DIRECT channel `∂L/∂μ_c += Σ_v co_v · (∇D_v/μ_c)` (the friction-coefficient
/// gradient); without it, the material/passive path has 6 parents (byte-identical).
#[derive(Clone, Debug)]
struct FrictionWrenchTrajVjp {
    /// Per active friction vertex.
    verts: Vec<FrictionWrenchVert>,
    /// `f_fric_total = Σ ∇D_v` — the total friction force (for the COM `c(q)` feedback).
    f_total: Vec3,
    /// The COM linear Jacobian `J_lin = ∂c/∂qpos` (`3 × nv`).
    jlin: DMatrix<f64>,
    /// Soft-DOF count `3·n_vertices`.
    n_dof: usize,
    /// Rigid-DOF count `nv`.
    nv: usize,
    /// Pose-parent component count: `1` for the scalar height channel, `3` for a moving
    /// end-effector centre (`[x̂, ŷ, ẑ]`). Must equal every vertex's `dforce_dpose.len()`.
    n_pose: usize,
    /// When `true`, a 7th parent `μ_c` (`[1]`) is present and the `∂L/∂μ_c` direct channel is
    /// threaded; `false` ⇒ 6 parents, no `μ_c` channel.
    mu_c: bool,
}

impl VjpOp for FrictionWrenchTrajVjp {
    fn op_id(&self) -> &'static str {
        "sim_coupling::FrictionWrenchTrajVjp"
    }

    // Shape mismatches are programmer bugs (wrong node shape pushed) — assert.
    // needless_range_loop: the per-coord contractions index flat 3×n_dof / 2-D `jlin` blocks;
    // explicit indices read clearer than zipped iterators here.
    #[allow(clippy::panic, clippy::needless_range_loop)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        let nd = self.n_dof;
        let want_parents = if self.mu_c { 7 } else { 6 };
        let mu_c_shape_ok = !self.mu_c || parent_cotans[6].shape() == [1];
        assert!(
            cotangent.shape() == [6]
                && parent_cotans.len() == want_parents
                && parent_cotans[0].shape() == [6]
                && parent_cotans[1].shape() == [nd]
                && parent_cotans[2].shape() == [self.n_pose]
                && parent_cotans[3].shape() == [2 * self.nv]
                && parent_cotans[4].shape() == [1]
                && parent_cotans[5].shape() == [nd]
                && mu_c_shape_ok,
            "FrictionWrenchTrajVjp: expected cot [6] + parents (w_normal [6], x* [{nd}], \
             pose [{}], s [{}], drift [1], x_prev [{nd}][, μ_c [1]])",
            self.n_pose,
            2 * self.nv,
        );
        let cot = cotangent.as_slice();
        let cot_t = Vec3::new(cot[0], cot[1], cot[2]); // cotangent on τ
        let cot_f = Vec3::new(cot[3], cot[4], cot[5]); // cotangent on f

        // ∂L/∂w_normal: pass-through (the friction wrench adds onto the normal wrench).
        let wn = parent_cotans[0].as_mut_slice();
        for i in 0..6 {
            wn[i] += cot[i];
        }

        // Per-vertex friction contributions.
        let mut pose_acc = vec![0.0_f64; self.n_pose];
        let mut drift_acc = 0.0_f64;
        let mut mu_c_acc = 0.0_f64;
        for v in &self.verts {
            debug_assert_eq!(
                v.dforce_dpose.len(),
                self.n_pose,
                "every vertex's dforce_dpose must have n_pose axes"
            );
            // Effective cotangent on ∇D_v: force part + moment-via-∇D_v part.
            let co = cot_f + cot_t.cross(&v.arm);
            // ∂L/∂x* via the per-vertex Jacobian, plus the moment ARM term on v's own coords.
            let xs = parent_cotans[1].as_mut_slice();
            for k in 0..nd {
                xs[k] += co.x * v.dforce_dx[k]
                    + co.y * v.dforce_dx[nd + k]
                    + co.z * v.dforce_dx[2 * nd + k];
            }
            let arm_grad = v.force.cross(&cot_t); // ∂[(x*_v−c)×∇D_v]·cot_τ / ∂x*_v
            xs[3 * v.vid] += arm_grad.x;
            xs[3 * v.vid + 1] += arm_grad.y;
            xs[3 * v.vid + 2] += arm_grad.z;
            // ∂L/∂x_prev via the per-vertex Jacobian (no arm term — x_prev does not move r_v).
            let xp = parent_cotans[5].as_mut_slice();
            for k in 0..nd {
                xp[k] += co.x * v.dforce_dxprev[k]
                    + co.y * v.dforce_dxprev[nd + k]
                    + co.z * v.dforce_dxprev[2 * nd + k];
            }
            for (k, dfp) in v.dforce_dpose.iter().enumerate() {
                pose_acc[k] += co.dot(dfp);
            }
            drift_acc += co.dot(&v.dforce_ddrift);
            mu_c_acc += co.dot(&v.dforce_dmu_c);
        }
        let pose_slot = parent_cotans[2].as_mut_slice();
        for (k, &a) in pose_acc.iter().enumerate() {
            pose_slot[k] += a;
        }
        parent_cotans[4].as_mut_slice()[0] += drift_acc;
        // Direct μ_c channel (only when the node carries the μ_c parent).
        if self.mu_c {
            parent_cotans[6].as_mut_slice()[0] += mu_c_acc;
        }

        // ∂L/∂s via the COM c(q): m = −f_fric_total × cot_τ, ∂L/∂qpos = J_linᵀ·m (qvel rows 0).
        let m = -self.f_total.cross(&cot_t);
        let s_slot = parent_cotans[3].as_mut_slice();
        for j in 0..self.nv {
            s_slot[j] +=
                self.jlin[(0, j)] * m.x + self.jlin[(1, j)] * m.y + self.jlin[(2, j)] * m.z;
        }
    }
}

/// Assemble the per-vertex friction grip onto a wrench: adds the friction force `Σ∇D_v` and the
/// off-COM moment `Σ(r_v−c)×∇D_v` (`r_v = positions[vid]`, `c` the COM) onto `wrench`, and returns
/// the per-vertex carry [`FrictionWrenchVert`]s plus the total friction force `Σ∇D_v` (the node's
/// `c(q)` feedback). Shared forward+tape assembly for the material and friction-coefficient
/// articulated gradients.
///
/// `pose_dforce` is the per-vertex pose-axis sensitivity, index-aligned with `pv`: `None` ⇒ the
/// scalar height channel (`dforce_dpose = [p.dforce_dheight]`, one axis); `Some(rows)` ⇒ the moving
/// end-effector centre channel, `rows[i]` the `[∂force/∂(centre·x̂), …·ŷ, …·ẑ]` for `pv[i]` (each
/// `friction_force_jacobians(pose_dir = e_k).dforce_dheight`).
fn assemble_friction_wrench(
    pv: Vec<FrictionVertexForce>,
    positions: &[Vec3],
    c: Vec3,
    wrench: &mut SpatialVector,
    pose_dforce: Option<Vec<Vec<Vec3>>>,
) -> (Vec<FrictionWrenchVert>, Vec3) {
    let mut f_total = Vec3::zeros();
    let mut fverts = Vec::with_capacity(pv.len());
    for (i, p) in pv.into_iter().enumerate() {
        let arm = positions[p.vid as usize] - c; // d_v = r_v − c
        let tau = arm.cross(&p.force);
        wrench[0] += tau.x;
        wrench[1] += tau.y;
        wrench[2] += tau.z;
        wrench[3] += p.force.x;
        wrench[4] += p.force.y;
        wrench[5] += p.force.z;
        f_total += p.force;
        let dforce_dpose = pose_dforce
            .as_ref()
            .map_or_else(|| vec![p.dforce_dheight], |rows| rows[i].clone());
        fverts.push(FrictionWrenchVert {
            vid: p.vid as usize,
            force: p.force,
            arm,
            dforce_dx: p.dforce_dx,
            dforce_dxprev: p.dforce_dxprev,
            dforce_ddrift: p.dforce_ddrift,
            dforce_dpose,
            dforce_dmu_c: p.dforce_dmu_c,
        });
    }
    (fverts, f_total)
}

/// Result of one coupled step.
#[derive(Clone, Copy, Debug)]
pub struct CoupledStep {
    /// Total contact force the soft body exerts (the reaction on the rigid body
    /// is its negation). In newtons, world frame.
    pub force_on_soft: Vec3,
    /// Current height of the contacting rigid body's reference point.
    pub rigid_z: f64,
}

/// One captured frame of a closed-loop **friction-grip** rollout — the per-step
/// state a viewer replays (see
/// [`StaggeredCoupling::coupled_trajectory_policy_gripped_capture`]).
///
/// The capturing rollout shares its loop body with the scalar forward
/// [`StaggeredCoupling::coupled_trajectory_policy_gripped_x`] (the FD oracle for the
/// design+policy-friction gradient), so the animated scene is the *same* physics the
/// gradient differentiates — not a re-derivation. One frame is emitted before the
/// first step (the rest state) and one after each step, so a rollout of `n_steps`
/// yields `n_steps + 1` frames.
///
/// **Framing.** A frame pairs the post-step `soft_positions` and contact `fist_center`
/// (both from this step's *contact* configuration — the correct staggered pairing: the
/// soft solve responds to the step-start sphere) with the post-step rigid pose
/// (`arm_pivot`, `arm_tip`). Render the swinging limb as the segment `arm_pivot →
/// arm_tip` (the pivot is the fixed hinge for the grip scene; the tip swings). For the
/// centroid grip the `fist_center` is static over the block (the contact is abstracted
/// to the block centroid — the moving-end-effector follow-on would instead pose it at
/// `arm_tip`), so draw it as the abstracted contact patch, not the limb's end.
#[derive(Clone, Debug)]
pub struct GripRolloutFrame {
    /// Deformed soft-body vertex positions after this step, flat
    /// `[x0, y0, z0, x1, y1, z1, …]` (length `3 · n_vertices`). Pair with
    /// [`StaggeredCoupling::soft_boundary_faces`] (the fixed rest topology) to build
    /// the surface mesh.
    pub soft_positions: Vec<f64>,
    /// The gripped rigid body's world **origin** (`data().xpos[body]`) after this step
    /// — the limb's anchor (the fixed hinge pivot for the grip scene). Pair with
    /// `arm_tip` to draw the limb.
    pub arm_pivot: [f64; 3],
    /// The gripped rigid body's world **inertial origin** (`data().xipos[body]`) after
    /// this step — the swinging end of the limb, the same quantity the scalar forward
    /// tracks (`tip_x = xipos[body].x`). Render the limb as `arm_pivot → arm_tip`.
    pub arm_tip: [f64; 3],
    /// The contact sphere's world centre this step — the block centroid (centroid
    /// grip) or the arm-tip geom (moving end-effector). This is what the soft body
    /// actually grips; render it as the fist. Sphere-only: a plane collider has no
    /// fist, and this defaults to the block-centre point on the contact plane.
    pub fist_center: [f64; 3],
    /// The gripped limb's primary joint coordinate `qpos[0]` this step — the hinge angle
    /// the holding objective regulates (and the FD oracle for
    /// [`StaggeredCoupling::coupled_trajectory_design_policy_hold_gradient`]).
    pub qpos0: f64,
}

/// One built design+policy friction-grip tape, ready to seed with an objective.
///
/// The expensive per-step coupled machinery — the #406 design+policy-friction tape (soft
/// re-equilibration, contact + friction wrench nodes, the closed-loop state→control recurrence,
/// the `μ`/`θ` leaves) — is shared by every objective; only the *seeding* differs. The
/// terminal-outcome objective seeds `tip_x` (via `s_final`/`jx_final`); a trajectory-integrated
/// objective seeds a cost summed over `qpos_steps` (the per-step policy-observed hinge angle). Built
/// by `build_design_policy_tape`; consumed by the public design+policy gradients.
struct DesignPolicyTape {
    tape: Tape,
    /// The material design leaf (`μ`, with the `λ = 4μ` tie folded into its grip node).
    p_var: Var,
    /// The policy parameter leaves (`θ`), shared across steps.
    param_vars: Vec<Var>,
    /// The policy-observed hinge angle `qpos[0]` at the start of each step — the leaf a
    /// trajectory-integrated objective (e.g. a holding cost) regulates.
    qpos_steps: Vec<Var>,
    /// The terminal carried state (`[qpos; qvel]`), the parent of the terminal-`tip_x` seam node.
    s_final: Var,
    /// `∂tip_x/∂state` at the terminal config (the linear COM-Jacobian row), for `PoseSeamVjp`.
    jx_final: Vec<f64>,
    /// The terminal tangential drag `tip_x = xipos[body].x` (the terminal-outcome value).
    tip_x: f64,
}

/// The state a [`DiffPolicy`] observes each step — two loop-carried chassis-tape scalar [`Var`]s.
/// On the free platen these are the platen height `z` and vertical velocity `vz`
/// ([`StaggeredCoupling::coupled_trajectory_policy_gradient`]); on the articulated grip they are
/// the hinge's joint angle and rate (`s[0]`, `s[nv]`,
/// [`StaggeredCoupling::coupled_trajectory_policy_friction_gradient`]). The fields are generic
/// position/velocity-style observations — the `z`/`vz` names are the platen legacy. Richer
/// observations (contact force, soft-state summaries) are a documented follow-on; they would add
/// fields here.
#[derive(Clone, Copy, Debug)]
pub struct PolicyState {
    /// The position-like observation at the step start (platen height `z`, or joint angle) — a
    /// `[1]`-shaped var.
    pub z: Var,
    /// The velocity-like observation at the step start (platen `vz`, or joint rate) — a
    /// `[1]`-shaped var.
    pub vz: Var,
}

/// A differentiable closed-loop feedback policy `u_k = π_θ(state_k)`: it maps the
/// per-step platen state to a vertical **control force** on the platen, with
/// parameters θ shared across every step.
///
/// Two views of the same function are required, and **must agree**:
/// - [`eval`](Self::eval) — a plain (non-tape) `f64` evaluation, used by the
///   forward oracle [`StaggeredCoupling::coupled_trajectory_policy_z`] and to drive
///   the real physics.
/// - [`emit`](Self::emit) — builds the policy as a sub-expression on the chassis
///   tape from the θ parameter leaves and the [`PolicyState`] vars, returning the
///   control output var. The chassis autograd carries both `∂u/∂θ` and `∂u/∂state`,
///   so [`StaggeredCoupling::coupled_trajectory_policy_gradient`] gets the
///   closed-loop gradient `∂z_N/∂θ` (backprop-through-time across the
///   state→control recurrence) from one `tape.backward` — no hand-rolled adjoint.
///
/// The two views must compute the identical function of `(θ, z, vz)`; the gate's
/// forward-match assertion (tape `z_N` == oracle `z_N`) catches any divergence.
pub trait DiffPolicy {
    /// Number of policy parameters θ (the gradient length).
    fn n_params(&self) -> usize;

    /// Plain evaluation `u = π_θ(z, vz)` from real state — drives the physics and
    /// the forward oracle.
    fn eval(&self, params: &[f64], z: f64, vz: f64) -> f64;

    /// Emit the control output var onto `tape` from the parameter leaves `params`
    /// and the current platen `state`. The returned var is `[1]`-shaped.
    fn emit(&self, tape: &mut Tape, params: &[Var], state: PolicyState) -> Var;
}

/// A linear state-feedback policy `u = w_z·z + w_vz·vz + b`, parameters
/// θ = `[w_z, w_vz, b]` (all **signed**). A genuine feedback law — the control
/// depends on the platen state — physically a PD-style controller about the
/// implicit setpoint `z_ref = −b/w_z` (with `w_z` the proportional and `w_vz` the
/// derivative gain, both as signed weights). The smallest closed-loop policy that
/// exercises the recurrence; a multilayer policy is a follow-on `DiffPolicy` impl
/// (e.g. via the chassis `affine`/`tanh` primitives).
#[derive(Clone, Copy, Debug, Default)]
pub struct LinearFeedback;

impl DiffPolicy for LinearFeedback {
    fn n_params(&self) -> usize {
        3
    }

    fn eval(&self, params: &[f64], z: f64, vz: f64) -> f64 {
        params[0] * z + params[1] * vz + params[2]
    }

    fn emit(&self, tape: &mut Tape, params: &[Var], state: PolicyState) -> Var {
        let t1 = tape.mul(params[0], state.z);
        let t2 = tape.mul(params[1], state.vz);
        let s = tape.add(t1, t2);
        tape.add(s, params[2])
    }
}

/// A staggered forward coupling of a hand-built soft block (bottom face pinned)
/// and a `sim_core` rigid body that presses on it from above through a
/// body-posed downward penalty-contact plane.
///
/// The soft body is a [`HandBuiltTetMesh::uniform_block`] (Neo-Hookean); the
/// rigid body is identified by `body` in `model`/`data`. The contact plane is a
/// downward half-space (`RigidPlane` normal `−z`) whose height tracks the rigid
/// body's reference height minus `contact_clearance` (e.g. a platen's half
/// thickness), so the soft block's top feels the descending body.
pub struct StaggeredCoupling<C: PlaneContact = PenaltyRigidContact> {
    model: Model,
    data: Data,
    body: usize,
    contact_clearance: f64,

    // soft block (rebuilt per step; topology + pinned set are fixed)
    field: MaterialField,
    /// Neo-Hookean Lamé parameters of the (uniform) block — stored alongside
    /// `field` so the material-gradient FD oracle can rebuild the block with one
    /// parameter perturbed (keystone S5).
    mu: f64,
    lambda: f64,
    n_per_edge: usize,
    edge: f64,
    pinned: Vec<VertexId>,
    n_vertices: usize,
    x: Vec<f64>,
    v: Vec<f64>,

    cfg: SolverConfig,
    kappa: f64,
    d_hat: f64,
    rigid_damping: f64,
    /// When `true`, the contact plane's normal tracks the rigid body's orientation
    /// (`n̂ = R·(0,0,−1)`) instead of the fixed downward `(0,0,−1)` — a tilted body
    /// presents a tilted, differentiable contact face (the rotating-normal leaf).
    /// Default `false` (flat, byte-identical to the pre-leaf coupling); enabled via
    /// [`Self::with_rotating_normal`]. See `docs/keystone/rotating_normal_recon.md`.
    rotating_normal: bool,
    /// The rigid collider geometry (default [`Collider::Plane`], the byte-identical
    /// keystone half-space; [`Collider::Sphere`] is the finite-contact gate). Set
    /// via [`Self::with_sphere_collider`].
    collider: Collider,
    /// When `Some`, the **full** world centre the finite [`Collider::Sphere`] is posed
    /// at — overriding the default block-centroid-over-`height` posing so the fist
    /// tracks a moving end-effector (the arm tip). `None` (default) poses over the
    /// block's top-face centroid at `z = height + radius` (a stable central patch,
    /// byte-identical to the pre-end-effector posing — the gradient gates' scene).
    /// [`Self::step_articulated`] refreshes it each frame from [`Self::contact_geom`];
    /// the trajectory adjoints hold it fixed (forward fidelity, not a gradient carry).
    /// Ignored for the infinite [`Collider::Plane`] (a half-space has no lateral pose).
    sphere_center_override: Option<Vec3>,
    /// When `Some`, the rigid **geom** whose world centre [`Self::step_articulated`]
    /// poses the finite sphere collider at each frame (the contact end-effector — the
    /// fist at the arm tip), via [`Self::sphere_center_override`]. `None` (default)
    /// keeps the block-centroid posing. Set via [`Self::with_contact_geom`].
    contact_geom: Option<usize>,
    _contact: PhantomData<C>,
}

impl<C: PlaneContact> StaggeredCoupling<C> {
    /// Build a coupling of a Neo-Hookean soft block and the rigid `body` in
    /// `model`/`data`.
    ///
    /// `n_per_edge`/`edge`/`mu` define the block (bottom `z = 0` face pinned);
    /// `dt` is the lockstep step (used for both the dynamic soft solve and the
    /// rigid step); `kappa`/`d_hat` are the penalty-contact parameters;
    /// `contact_clearance` offsets the contact plane below the rigid body's
    /// reference point; `rigid_damping` is a linear velocity damping applied to
    /// the rigid body's contact axis so it settles to a static equilibrium.
    ///
    /// # Panics
    /// Panics if the block has no `z = 0` vertices to pin (a malformed block).
    #[must_use]
    // 11 independent physical knobs (rigid body + contact + soft block + sim
    // params); a config-struct bundle is a deferred ergonomics refactor.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        model: Model,
        data: Data,
        body: usize,
        contact_clearance: f64,
        n_per_edge: usize,
        edge: f64,
        mu: f64,
        dt: f64,
        kappa: f64,
        d_hat: f64,
        rigid_damping: f64,
    ) -> Self {
        let lambda = 4.0 * mu;
        let field = MaterialField::uniform(mu, lambda);
        let mesh = HandBuiltTetMesh::uniform_block(n_per_edge, edge, &field);
        let n_vertices = mesh.n_vertices();
        let pinned: Vec<VertexId> =
            sim_soft::pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
        assert!(!pinned.is_empty(), "soft block has no z=0 base to pin");
        let mut x = vec![0.0_f64; 3 * n_vertices];
        for (chunk, p) in x.chunks_exact_mut(3).zip(mesh.positions()) {
            chunk[0] = p.x;
            chunk[1] = p.y;
            chunk[2] = p.z;
        }
        let mut cfg = SolverConfig::skeleton();
        cfg.dt = dt;
        Self {
            model,
            data,
            body,
            contact_clearance,
            field,
            mu,
            lambda,
            n_per_edge,
            edge,
            pinned,
            n_vertices,
            x,
            v: vec![0.0_f64; 3 * n_vertices],
            cfg,
            kappa,
            d_hat,
            rigid_damping,
            rotating_normal: false,
            collider: Collider::Plane,
            sphere_center_override: None,
            contact_geom: None,
            _contact: PhantomData,
        }
    }

    /// Enable the **rotating contact normal**: the contact plane's outward normal
    /// tracks the rigid body's orientation (`n̂ = R(q)·(0,0,−1)`, `offset = xipos·n̂
    /// + clearance`) rather than the fixed downward `(0,0,−1)`, so a tilted body
    /// presents a tilted contact face and the coupled gradient flows through the
    /// normal's `q`-dependence (`∂n̂/∂q = −[n̂]×·J_ang`). Reduces to the flat plane at
    /// `R = I`. Opt-in (default off) because it changes the contact physics and is
    /// gated at gentle tilt (the flat-tuned large-tilt keystone scenes can diverge
    /// under a rotating normal). The single-step S3 height probes
    /// ([`Self::contact_force_at_height`] etc.) remain `height`-parameterized — the
    /// plane tilts with `R` but still translates with `height` — so their FD contract
    /// holds in both modes. See `docs/keystone/rotating_normal_recon.md`.
    #[must_use]
    pub const fn with_rotating_normal(mut self, on: bool) -> Self {
        self.rotating_normal = on;
        self
    }

    /// Replace the infinite contact plane with a **finite posed sphere** of the
    /// given `radius` — the L1 finite-contact collider (a curved end-effector
    /// indenting the soft block) the de-escalation viz ladder shows. The scalar
    /// `height` still drives the vertical carry (the sphere centre rides `+ẑ` with
    /// `height`, so the existing `RigidTwist::translation(ẑ)` pose-sensitivity probe
    /// and `∂sd/∂h = −n̂·ẑ` formula remain exactly correct — `n̂` is now the sphere's
    /// per-vertex varying normal rather than the plane's constant `−ẑ`); the lateral
    /// centre defaults to the block's top-face centroid so the contact patch is
    /// central and the active set stays stable across an FD perturbation, and is
    /// re-pointable to a moving end-effector via [`Self::with_contact_geom`].
    ///
    /// Opt-in (default the infinite [`RigidPlane`], byte-identical to the
    /// pre-finite-collider keystone scenes #402–#406). Curved contact engages more
    /// deeply per Newton step, so this also widens the soft solve's iteration budget
    /// (matching the L0 `sphere_pose_sensitivity` gate). Takes precedence over
    /// [`Self::with_rotating_normal`] if both are set (a sphere's normal already turns
    /// with the query, so `build_contact` ignores the plane-tilt flag).
    ///
    /// **Scope (L1a).** Only the SINGLE-STEP pose-sensitivity channel is
    /// curvature-correct for the sphere — [`Self::contact_force_height_jacobian`] and
    /// [`Self::contact_force_height_total_jacobian`], gated by
    /// `tests/sphere_contact_total_jacobian.rs` (the coupling-level lift of #415's L0
    /// curved-pose sensitivity, machine-exact). The MULTI-step trajectory gradients
    /// (`coupled_trajectory_*`) still assemble the contact-force Jacobian as the
    /// flat `−cᵥ·n̂⊗n̂` (exact for the plane, missing the curved `dE·H` term) — threading
    /// the curvature term through their adjoints is the L1b follow-on; see
    /// `docs/keystone/` for the finite-contact carry.
    #[must_use]
    pub fn with_sphere_collider(mut self, radius: f64) -> Self {
        self.collider = Collider::Sphere { radius };
        self.cfg.max_newton_iter = self.cfg.max_newton_iter.max(80);
        self
    }

    /// Enable **tangential (smoothed-Coulomb) friction GRIP** at the soft↔rigid contact.
    /// The soft Newton solve becomes friction-aware (`μ_c = mu`, stick-band velocity
    /// threshold `ε_v = eps_v`), and — crucially for two-way grip — each step feeds the
    /// rigid body's within-step tangential motion to the soft solve as the collider drift
    /// `Δ_surf` (so a sliding device DRAGS the held soft body), then routes the soft body's
    /// friction reaction (and its off-COM moment) back onto the rigid body. The result: the
    /// rigid body feels tangential grip — it is held / dragged by the gripping soft body
    /// rather than sliding freely over it.
    ///
    /// Opt-in (default `μ = 0`, frictionless, byte-identical to the pre-friction coupling):
    /// the grip is exercised by the friction-aware forward rollout
    /// [`Self::coupled_trajectory_grip`].
    ///
    /// **Forward-only** (this leaf, PR3a): the *gradient* of a friction-coupled trajectory
    /// is not yet supported — the soft adjoint panics on a nonzero collider drift with
    /// friction (`sim_soft`'s `friction_surface_drift` guard). PR3b threads the drift
    /// through the adjoint.
    #[must_use]
    pub fn with_friction(mut self, mu: f64, eps_v: f64) -> Self {
        self.cfg.friction_mu = mu;
        self.cfg.friction_eps_v = eps_v;
        // The friction stick regime stiffens the Newton tangent (condition `~1/(ε_v·dt)`),
        // so the default 10-iter / 20-backtrack budget is too tight; match PR1's converging
        // `friction_stick_slip` budget. A no-op when `mu == 0` (the friction scatter is
        // short-circuited, so the solve converges in the usual 3-5 iters either way).
        if mu != 0.0 {
            self.cfg.max_newton_iter = 80;
            self.cfg.max_line_search_backtracks = 60;
        }
        self
    }

    /// Read-only access to the rigid engine state.
    #[must_use]
    pub fn data(&self) -> &Data {
        &self.data
    }

    /// The soft body's current deformed vertex positions, flat stride-3
    /// (`[x₀,y₀,z₀, x₁,…]`, length `3·n_vertices`) — the live `x*` the last
    /// [`Self::step`] / grip rollout produced. A read surface for visualization
    /// (capture per step into a deformed-mesh trajectory) and instrumentation.
    #[must_use]
    pub fn soft_positions(&self) -> &[f64] {
        &self.x
    }

    /// The soft body's surface triangulation (`boundary_faces`) for the rest tet
    /// mesh — the topology a renderer pairs with [`Self::soft_positions`] frames to
    /// build and animate the deformed surface mesh. Constant across the rollout
    /// (only vertex positions move), so capture it once.
    #[must_use]
    pub fn soft_boundary_faces(&self) -> Vec<[VertexId; 3]> {
        self.fresh_mesh().boundary_faces().to_vec()
    }

    /// The contact plane's height for the current rigid pose (a downward
    /// half-space whose surface sits at the rigid body's reference point minus
    /// the clearance).
    fn plane_height(&self) -> f64 {
        self.data.xpos[self.body].z - self.contact_clearance
    }

    /// The finite sphere collider's world centre for a given `radius` and scalar
    /// `height`. The default (no [`Self::sphere_center_override`]) poses over the
    /// block's top-face centroid `(edge/2, edge/2)` at centre z `= height + radius`, so
    /// the south pole sits at `height` (the same engagement depth the plane's surface
    /// would) — a pure `+ẑ` translation of the primitive, exactly the
    /// `RigidTwist::translation(ẑ)` the pose-sensitivity probes (lifting #415's L0
    /// curved gate into the coupling). When the override is set (end-effector posing,
    /// [`Self::step_articulated`]) it is the centre directly. The single source of the
    /// centre formula (shared by [`Self::build_contact`] and [`Self::collider_hessian`]).
    fn sphere_center(&self, radius: f64, height: f64) -> Vec3 {
        self.sphere_center_override
            .unwrap_or_else(|| Vec3::new(self.edge / 2.0, self.edge / 2.0, height + radius))
    }

    /// Pose the finite sphere collider at a rigid **geom**'s world centre each frame —
    /// the contact end-effector (the fist at the arm tip), so the fist tracks the
    /// striker instead of sitting over the block centroid. [`Self::step_articulated`]
    /// reads `geom_xpos[geom_id]` (fresh-FK) into the sphere-centre override per
    /// step. Pair the `radius` with the geom's own (`with_sphere_collider(r)` where the
    /// geom's `size = r`) so the contact sphere is the fist.
    ///
    /// The ARTICULATED gradients — material/friction (#428/#429) AND actuator/policy (#431) — thread
    /// the moving centre through the adjoint (the 3-vector `PoseCentreVjp` seam, `WrenchPose::Centre`,
    /// the grip soft node's 3-axis pose, and the friction wrench's 3-vector `dforce_dpose`; gated by
    /// `sphere_moving_ee_{,friction_}trajectory_gradient.rs` + `actuator_moving_ee_gradient.rs`);
    /// their forward oracles (`coupled_trajectory_articulated_z` / `_gripped_articulated` / the
    /// actuated/policy `*_z` / `*_gripped_x`) pose at the same geom. The FREE-BODY gradients (+ grip
    /// forward) are guarded (`require_no_moving_ee`) — a moving EE there is DEGENERATE, not a
    /// follow-on (flat-block translation-invariance + no off-COM moment; the sweep is the drift
    /// channel). A no-op for the infinite plane collider. Default (unset) reproduces the byte-identical
    /// block-centroid posing the finite-contact gates use.
    #[must_use]
    pub const fn with_contact_geom(mut self, geom_id: usize) -> Self {
        self.contact_geom = Some(geom_id);
        self
    }

    /// Pose the finite sphere collider at an explicit world `center` — for a
    /// **kinematically driven** striker whose end-effector path is scripted in the
    /// caller (no rigid body to read a geom from), paired with [`Self::step_kinematic`].
    /// Call before each soft step with the striker's world centre; the contact then
    /// engages there. The direct counterpart to [`Self::with_contact_geom`] (which
    /// reads the centre from a rigid geom). A no-op for the infinite plane collider.
    pub fn set_sphere_center(&mut self, center: Vec3) {
        self.sphere_center_override = Some(center);
    }

    /// The contact primitive's curvature `H = ∂n̂/∂p = ∇²sd` at world point `p` for
    /// the current collider posed at `height` — the geometric-stiffness factor the
    /// curved-contact force Jacobian needs (#415's `dE·H` term). `0` for the infinite
    /// plane (constant normal, in both flat and rotating-normal modes — the tilt
    /// depends on the body pose `q`, not the query `p`), the sphere's
    /// `(I − n̂n̂ᵀ)/‖p − c‖` for the finite collider.
    fn collider_hessian(&self, height: f64, p: Vec3) -> Matrix3<f64> {
        match self.collider {
            Collider::Plane => Matrix3::zeros(),
            Collider::Sphere { radius } => {
                posed_sphere(radius, self.sphere_center(radius, height)).hessian(Point3::from(p))
            }
        }
    }

    /// Assert no contact end-effector geom is set ([`Self::with_contact_geom`]) — the scope
    /// guard for every sphere-capable trajectory method that does NOT thread the
    /// moving-end-effector centre carry.
    ///
    /// Every sphere-capable gradient is curvature-correct on a CENTROID sphere (the contact nodes
    /// carry the #415–#429 curvature; the actuator `g_act` channel is contact-independent). The
    /// *moving* end-effector (the 3-vector centre channel, posing at + differentiating through
    /// `geom_xpos(q)`) is threaded by the ARTICULATED normal/friction gradients (#428/#429) AND the
    /// ACTUATOR/POLICY gradients (and all their forward oracles —
    /// [`Self::coupled_trajectory_articulated_z`], [`Self::coupled_trajectory_gripped_articulated`],
    /// the actuated/policy `*_z`/`*_gripped_x` oracles). The remaining guarded set — which would
    /// otherwise silently pose the sphere at the block centroid (ignoring the tip), a silent
    /// contract violation on a public API (ship-blocking even if currently unused) — is the
    /// FREE-BODY gradients (`material`/`peak_force`/`control`/`policy`/`joint`/`tangential_*`) and
    /// the free-body grip forward ([`Self::coupled_trajectory_grip`]). A no-op when no geom is set
    /// (the centroid-posed gates + the plane).
    ///
    /// **The free-body guard is a DELIBERATE boundary, not an unfinished follow-on** (verified by a
    /// spike, 2026-06-27). A moving end-effector on the free platen is DEGENERATE — there is no real
    /// centre channel to thread:
    /// - The free-body coupling threads scalar lanes (vertical `z`/`vz`, plus lateral `xx`/`vx` for
    ///   friction) with NO off-COM moment, so the contact's absolute lateral *position* feeds no
    ///   gradient channel (unlike the articulated arm, whose tip arcs and whose off-COM moment makes
    ///   `tip_z` pose-sensitive).
    /// - Over the FLAT block the contact is laterally translation-invariant (a sphere staying over
    ///   the block feels the same force at any x), and the collider's relative sweep is ALREADY the
    ///   drift channel (`Δ_surf = v_collider·dt`). So a `geom_xpos` centre channel adds nothing the
    ///   drift + height don't (consistent with #429's pose-insensitive friction `tip_x`).
    /// - Naive geom-posing isn't even the same model: the geom centre is `xpos.z`, vs the keystone's
    ///   `height + r` south-pole reference — a vertical-convention mismatch, not a tip-tracking gain.
    ///
    /// So this guard stays as the correct LOUD rejection; "completing" the free body would mean a
    /// full 6-DOF rigid model (a different effort), not a moving-EE carry.
    fn require_no_moving_ee(&self) {
        assert!(
            self.contact_geom.is_none(),
            "a moving end-effector (with_contact_geom) is not supported on the free-body path — it \
             is DEGENERATE there (flat-block translation-invariance + no off-COM moment; the \
             collider sweep is already the drift channel). Use the ARTICULATED or ACTUATOR/POLICY \
             gradients for a tip-posed sphere, or the block-centroid default here."
        );
    }

    fn build_contact(&self, height: f64) -> C {
        if let Collider::Sphere { radius } = self.collider {
            let center = self.sphere_center(radius, height);
            return C::from_primitive(posed_sphere(radius, center), self.kappa, self.d_hat);
        }
        let plane = if self.rotating_normal {
            // Rotating normal: the plane tracks the body ORIENTATION (`n̂ = R·(0,0,−1)`)
            // while still honoring the scalar `height` like the flat branch — the plane
            // passes through the reference point at the body's lateral position and world
            // z = `height + clearance`, so `offset = p_ref·n̂ + clearance`. This keeps
            // `height` a live parameter (the S3 single-step probes translate the tilted
            // plane vertically by varying it) AND reduces to the flat `((0,0,−1),
            // −height)` at `R = I`. In the coupled path `height = xipos.z − clearance`, so
            // `p_ref = xipos` exactly (byte-identical to reading the live COM pose).
            let n = self.data.xquat[self.body] * Vec3::new(0.0, 0.0, -1.0);
            let com = self.data.xipos[self.body];
            let p_ref = Vec3::new(com.x, com.y, height + self.contact_clearance);
            let offset = p_ref.dot(&n) + self.contact_clearance;
            RigidPlane::new(n, offset)
        } else {
            // Downward ceiling at `height`: normal −z, offset −height ⇒ a soft
            // vertex below the plane has positive signed distance `height − z`.
            RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), -height)
        };
        C::from_primitive(plane, self.kappa, self.d_hat)
    }

    /// A fresh rest-topology soft mesh (the solver consumes one per step, and
    /// `per_pair_readout` needs one too; topology + rest geometry are fixed).
    fn fresh_mesh(&self) -> HandBuiltTetMesh {
        HandBuiltTetMesh::uniform_block(self.n_per_edge, self.edge, &self.field)
    }

    /// Current soft vertex positions as points (vertex-major xyz → `Vec3`s).
    fn positions(&self) -> Vec<Vec3> {
        self.x
            .chunks_exact(3)
            .map(|c| Vec3::new(c[0], c[1], c[2]))
            .collect()
    }

    /// `(total force_on_soft, active-pair count)` at the current soft
    /// configuration with the contact plane placed at `height` — does not
    /// re-solve or mutate. The shared building block for the contact-force
    /// readout and its analytic pose-sensitivity.
    fn contact_readout(&self, height: f64) -> (Vec3, usize) {
        let positions = self.positions();
        let readout = self
            .build_contact(height)
            .pair_readout(&self.fresh_mesh(), &positions);
        (readout.iter().map(|r| r.force_on_soft).sum(), readout.len())
    }

    /// Total contact force the soft body exerts, evaluated at the current soft
    /// configuration with the contact plane at `height` (no re-solve, no
    /// mutation). The forward building block for finite-difference and analytic
    /// contact-force sensitivities w.r.t. the rigid pose.
    #[must_use]
    pub fn contact_force_at_height(&self, height: f64) -> Vec3 {
        self.contact_readout(height).0
    }

    /// Per active contact pair at `positions` with the collider posed at `height`:
    /// `(vertex_id, ∂fz/∂x_v, ∂fz/∂height)` — the PRECOMPUTED z-force gradient factors
    /// the free-body contact crossing scatters ([`ContactForceVjp`], [`ContactForceTrajVjp`]),
    /// curvature-correct for any collider (the L1b carry, dual to the FD-validated
    /// single-step [`Self::contact_force_height_total_jacobian`]).
    ///
    /// Differentiating the total normal force `fz = Σ gᵢ·ẑ = Σ f_mag·n̂_z` (per-pair force
    /// `gᵢ = f_mag·n̂`, `f_mag = gᵢ·n̂`) splits into a MAGNITUDE change (`cᵥ = d²E/dsd²`, the
    /// flat term) and a NORMAL-ROTATION change (`f_mag·∂n̂`, #415's geometric stiffness
    /// `H = ∇²sd` = [`Self::collider_hessian`]). Taking the z-row of the per-pair force
    /// Jacobian `∂force/∂x_v = −cᵥ·n̂⊗n̂ + f_mag·H` and the explicit height partial
    /// `∂force/∂h = cᵥ·n̂_z·n̂ + f_mag·(−H·ẑ)` (raising the height translates the primitive
    /// `+ẑ`, `∂sd/∂h = −n̂_z`, `∂n̂/∂h = −H·ẑ`):
    /// - `∂fz/∂x_v = −cᵥ·n̂_z·n̂ + f_mag·(H·ẑ)`
    /// - `∂fz/∂height = cᵥ·n̂_z² − f_mag·(H·ẑ)_z`
    ///
    /// `H = 0` for the plane, where additionally `n̂ = −ẑ` exactly ⇒ `∂fz/∂x_v = (0,0,−cᵥ)`
    /// and `∂fz/∂height = cᵥ` — BYTE-IDENTICAL to the old flat `−cᵥ·n̂⊗n̂` / `Σ cᵥ` factors
    /// (#402–#406 untouched). For the sphere the `H·ẑ` terms are what make the free-body
    /// trajectory gradients match the curved re-solve FD. Read from
    /// [`Self::active_pair_wrench_data`] + [`Self::collider_hessian`]; does not re-solve/mutate.
    ///
    /// NOT a guard: the NORMAL contact crossing is curvature-correct for the sphere, as are the
    /// free-body + articulated FRICTION wrench paths (curved `DN·C` carried by #419 / the
    /// articulated-friction rung) AND the actuator/policy gradients. The MOVING end-effector centre
    /// channel is threaded by the articulated normal/friction + actuator/policy gradients; only the
    /// FREE-BODY gradients still guard it ([`Self::require_no_moving_ee`]).
    fn active_pair_force_factors(
        &self,
        height: f64,
        positions: &[Vec3],
    ) -> Vec<(usize, Vec3, f64)> {
        let zhat = Vec3::new(0.0, 0.0, 1.0);
        self.active_pair_wrench_data(height, positions)
            .into_iter()
            .map(|(v, g, n, curv, _r)| {
                let f_mag = g.dot(&n);
                // H·ẑ = the z-row of the symmetric contact Hessian H = ∇²sd (#415);
                // 0 for the plane, the sphere's `(I − n̂n̂ᵀ)/‖p − c‖ · ẑ` for the finite collider.
                let h_zhat = self.collider_hessian(height, positions[v]) * zhat;
                let dfz_dxv = -curv * n.z * n + f_mag * h_zhat;
                let dfz_dz = curv * n.z * n.z - f_mag * h_zhat.z;
                (v, dfz_dxv, dfz_dz)
            })
            .collect()
    }

    /// Per active contact pair at `positions` with the plane at `height`:
    /// `(vertex_id, soft-force gᵢ, normal n̂, curvature cᵥ, world position rᵢ)` where
    /// the local curvature `cᵥ = d²E/dsd² = n̂ᵀ·H_pair·n̂` is read from the contact's
    /// Hessian (`κ` for penalty, `κ·b''(sd)` for IPC). The single source of the
    /// per-pair readout + curvature extraction (projected by
    /// [`Self::active_pair_force_factors`] into the free-body z-force gradient factors);
    /// the moment routing + its [`ContactWrenchTrajVjp`] gradient additionally use the
    /// force `gᵢ` and contact point `rᵢ`. Does not re-solve/mutate.
    fn active_pair_wrench_data(
        &self,
        height: f64,
        positions: &[Vec3],
    ) -> Vec<(usize, Vec3, Vec3, f64, Vec3)> {
        let contact = self.build_contact(height);
        contact
            .pair_readout(&self.fresh_mesh(), positions)
            .iter()
            .map(|r| {
                let ContactPair::Vertex { vertex_id, .. } = r.pair;
                // H_pair = cᵥ·n̂⊗n̂ ⇒ cᵥ = n̂·(H·n̂), the per-pair contact stiffness.
                let curv = contact
                    .hessian(&r.pair, positions)
                    .contributions
                    .first()
                    .map_or(0.0, |e| r.normal.dot(&(e.2 * r.normal)));
                (
                    vertex_id as usize,
                    r.force_on_soft,
                    r.normal,
                    curv,
                    r.position,
                )
            })
            .collect()
    }

    /// The reaction **wrench** `[τ; f]` the soft body applies to the rigid body at
    /// its COM `c = xipos`, for the contact plane at `height` and the current soft
    /// positions: `f = −Σ gᵢ`, `τ = −Σ (rᵢ − c) × gᵢ` (`gᵢ = force_on_softᵢ`). The
    /// shared forward building block for the articulated oracle and the
    /// moment-routing gradient. Does not re-solve/mutate.
    fn contact_wrench(&self, height: f64) -> SpatialVector {
        let c = self.data.xipos[self.body];
        let mut wrench = SpatialVector::zeros();
        for &(_, g, _, _, r) in &self.active_pair_wrench_data(height, &self.positions()) {
            let f = -g;
            wrench[3] += f.x;
            wrench[4] += f.y;
            wrench[5] += f.z;
            let tau = (r - c).cross(&f);
            wrench[0] += tau.x;
            wrench[1] += tau.y;
            wrench[2] += tau.z;
        }
        wrench
    }

    /// The reaction **wrench** `[τ; f]` INCLUDING the tangential friction grip — the friction
    /// successor to [`Self::contact_wrench`]. On top of the normal contact reaction it folds
    /// in the soft body's per-pair friction force `friction[i] = −∇Dᵢ` (the `force_on_soft`
    /// sign, from [`CpuNewtonSolver::friction_forces_on_soft`]): the reaction on the rigid
    /// body is `−friction[i]` at the contacted vertex `rᵢ = positions[vᵢ]`, with the off-COM
    /// moment `(rᵢ − c) × (−friction[i])` about the COM `c = xipos`, so the same wrench
    /// routing carries the tangential grip in any direction. `friction` is read at the
    /// post-step soft config (the `self.positions()` the normal term also uses). Empty
    /// `friction` ⇒ byte-identical to [`Self::contact_wrench`]. Does not re-solve/mutate.
    fn contact_wrench_gripped(&self, height: f64, friction: &[(VertexId, Vec3)]) -> SpatialVector {
        let c = self.data.xipos[self.body];
        let positions = self.positions();
        let mut wrench = self.contact_wrench(height);
        for &(vid, f_fric) in friction {
            let r = positions[vid as usize];
            let f = -f_fric; // Newton's-3rd-law reaction on the rigid body
            wrench[3] += f.x;
            wrench[4] += f.y;
            wrench[5] += f.z;
            let tau = (r - c).cross(&f);
            wrench[0] += tau.x;
            wrench[1] += tau.y;
            wrench[2] += tau.z;
        }
        wrench
    }

    /// Analytic `∂(total force_on_soft)/∂(plane height)` at the current soft
    /// configuration, holding the soft positions fixed.
    ///
    /// Raising the height translates the plane `+ẑ`, so per active pair
    /// `∂sd/∂height = −n̂·ẑ = −n̂.z` and `∂force/∂height = −cᵥ·(∂sd/∂height)·n̂ =
    /// cᵥ·n̂.z·n̂` (curvature `cᵥ = d²E/dsd²`). For the FLAT downward plane (`n̂ = −ẑ`)
    /// this is `+cᵥ·ẑ` per pair, i.e. the total `+(Σ cᵥ)·ẑ` (penalty `cᵥ = κ` ⇒
    /// `κ·N_active·ẑ`; IPC `cᵥ = κ·b''(sd)`); under a [rotating
    /// normal](Self::with_rotating_normal) the tilted `n̂` redirects it. This is the
    /// explicit (fixed-position) partial — one factor of the coupled step's Jacobian,
    /// not the total settled-system derivative. Valid in the contact-engaged regime
    /// where the active set is stable across the perturbation; at the active-set
    /// boundary the penalty derivative is non-smooth (the IPC barrier smooths it).
    /// FD-checked against [`Self::contact_force_at_height`].
    ///
    /// For a CURVED collider ([`Self::with_sphere_collider`]) the normal also rotates
    /// as the primitive translates, adding the geometric-stiffness term `f_mag·(−H·ẑ)`
    /// (`H = ∇²sd`, #415); for the plane `H = 0`, so the added term is exactly `+0` and
    /// the plane result is numerically unchanged (`x + 0 = x`).
    #[must_use]
    pub fn contact_force_height_jacobian(&self, height: f64) -> Vec3 {
        // ∂force/∂h|_x = Σ [ cᵥ·n̂.z·n̂  +  f_mag·(∂n̂/∂h) ]. The first term (= (0,0,Σcᵥ)
        // for the flat n̂ = −ẑ) is the magnitude change; the second is the curved-normal
        // rotation as the primitive translates `+ẑ` — `∂n̂/∂h = −H·ẑ` (#415's geometric
        // stiffness, `H = ∇²sd`). H = 0 for the plane, so that term vanishes
        // (byte-identical); for the sphere it is the per-pair force magnitude `f_mag =
        // gᵢ·n̂` times `−H·ẑ`.
        let positions = self.positions();
        let zhat = Vec3::new(0.0, 0.0, 1.0);
        self.active_pair_wrench_data(height, &positions)
            .iter()
            .map(|&(v, g, n, c, _r)| {
                let f_mag = g.dot(&n);
                c * n.z * n + f_mag * (self.collider_hessian(height, positions[v]) * (-zhat))
            })
            .sum()
    }

    /// One-off rigid step from the *current* rigid state with an externally
    /// supplied vertical force `applied_fz` (newtons, world `+z`), returning the
    /// rigid body's `(next height, next vertical velocity)`. Does NOT advance
    /// `self` — it reconstructs a scratch `Data` at the current `(qpos, qvel)`,
    /// so it is a pure probe of the rigid step's response to an applied force
    /// (the rigid factor `∂s'/∂xfrc` of the coupled-step Jacobian). For the
    /// free-body platen the velocity response is analytically `∂vz'/∂fz = dt/m`
    /// (the quantity actually consumed). The single-step height response is `0`, not
    /// `dt²/m`: sim-core integrates position with the step's STARTING velocity, so the
    /// force reaches the height only on the next step (see `ZCarryVjp`).
    ///
    /// The scratch reproduces the current `(qpos, qvel)` only — faithful for the
    /// keystone scene (a free-joint body with no actuators, `ctrl`, `act`, or
    /// mocap state). `step` runs its own `forward`, which is what consumes the
    /// `xfrc_applied` set here.
    ///
    /// # Panics
    /// Panics if the scratch step diverges (a mis-constructed model).
    // A scratch step on a valid model does not fail; a divergence is a
    // programmer error surfaced loudly (see `step`'s rationale).
    #[allow(clippy::expect_used)]
    #[must_use]
    pub fn rigid_step_probe(&self, applied_fz: f64) -> (f64, f64) {
        let mut scratch = self.model.make_data();
        scratch.qpos.copy_from(&self.data.qpos);
        scratch.qvel.copy_from(&self.data.qvel);
        let mut sf = SpatialVector::zeros();
        sf[5] = applied_fz; // linear z (SpatialVector layout [angular(3), linear(3)])
        scratch.xfrc_applied[self.body] = sf;
        scratch.step(&self.model).expect("probe step");
        (scratch.xpos[self.body].z, scratch.qvel[2])
    }

    /// Non-mutating soft re-solve: build the soft solver with the contact
    /// plane at `height` and take one dynamic step from the CURRENT
    /// `(self.x, self.v)`, returning the solver (so the caller can read its
    /// pose sensitivity) and the converged `x_final`. Does not advance `self`.
    fn soft_resolve(&self, height: f64) -> (SoftSolver<C>, Vec<f64>) {
        let n = self.n_vertices;
        let bc = BoundaryConditions::new(self.pinned.clone(), Vec::new());
        let solver: SoftSolver<C> = CpuNewtonSolver::new(
            Tet4,
            self.fresh_mesh(),
            self.build_contact(height),
            self.cfg,
            bc,
        );
        let x_final = solver
            .replay_step(
                &Tensor::from_slice(&self.x, &[3 * n]),
                &Tensor::from_slice(&self.v, &[3 * n]),
                &Tensor::zeros(&[0]),
                self.cfg.dt,
            )
            .x_final;
        (solver, x_final)
    }

    /// Build a soft solver over `self`'s block with the contact plane at
    /// `height` and a scalar `+ẑ` (`LoadAxis::AxisZ`) load on `loaded` (atop
    /// the fixed pinned base). Shared by [`Self::coupled_step_load_gradient`]
    /// (tape `step`) and [`Self::coupled_step_load_vz`] (tape-free
    /// `replay_step`) so the loaded-BC mapping has one source of truth.
    fn build_loaded_solver(&self, height: f64, loaded: &[VertexId]) -> SoftSolver<C> {
        let bc = BoundaryConditions::new(
            self.pinned.clone(),
            loaded.iter().map(|&v| (v, LoadAxis::AxisZ)).collect(),
        );
        CpuNewtonSolver::new(
            Tet4,
            self.fresh_mesh(),
            self.build_contact(height),
            self.cfg,
            bc,
        )
    }

    /// Total contact force on the soft body AFTER one *re-equilibrated*
    /// dynamic soft step with the plane at `height`, evaluated from the
    /// current `(self.x, self.v)` — does not advance `self`.
    ///
    /// Unlike [`Self::contact_force_at_height`] (which holds the soft
    /// positions fixed and only re-poses the plane), this RE-SOLVES the
    /// soft equilibrium `x*(height)` first, then reads the force there. So
    /// finite-differencing it w.r.t. `height` captures the implicit
    /// soft-re-equilibration that [`Self::contact_force_height_total_jacobian`]
    /// supplies analytically — it is the black-box oracle for the total
    /// derivative.
    #[must_use]
    pub fn resolved_contact_force(&self, height: f64) -> Vec3 {
        let (_solver, x_final) = self.soft_resolve(height);
        let positions: Vec<Vec3> = x_final
            .chunks_exact(3)
            .map(|c| Vec3::new(c[0], c[1], c[2]))
            .collect();
        self.build_contact(height)
            .pair_readout(&self.fresh_mesh(), &positions)
            .iter()
            .map(|r| r.force_on_soft)
            .sum()
    }

    /// Total single-step derivative `d(force_on_soft)/d(plane height)`
    /// INCLUDING soft re-equilibration — the keystone S3 deliverable that
    /// lifts the explicit (fixed-soft-position) coupled-step Jacobian to the
    /// TOTAL:
    ///
    /// ```text
    /// d force/d h  =  ∂force/∂h|_x   +   (∂force/∂x)·(∂x*/∂h)
    ///                 └ explicit (S1) ┘    └──── implicit (S3) ────┘
    /// ```
    ///
    /// evaluated at the post-step equilibrium `x*` re-solved from the
    /// current `(self.x, self.v)` and plane height. The explicit term is the
    /// S1 fixed-position partial (`−κ·(∂sd/∂h)·n̂` per active pair); the
    /// implicit term contracts the penalty force Jacobian
    /// `∂force/∂x = −κ·n̂⊗n̂` (active, hard penalty) against the soft solver's
    /// pose sensitivity `∂x*/∂h` (`equilibrium_pose_sensitivity`). Does not
    /// advance `self`.
    ///
    /// For a CURVED collider ([`Self::with_sphere_collider`]) both terms additionally
    /// carry the normal-rotation contribution `f_mag·∂n̂` (the geometric stiffness
    /// `dE·H`, `H = ∇²sd`, #415): explicit `+ f_mag·(−H·ẑ)` as the primitive translates,
    /// implicit `+ f_mag·(H·∂x*/∂h)` as the vertex slides. Both vanish for the plane
    /// (`H = 0`), so the plane result is unchanged; for the sphere they are what make
    /// this Jacobian match the curved re-solve FD (the `sphere_contact_total_jacobian`
    /// gate, machine-exact).
    ///
    /// Physically the implicit term largely cancels the explicit one in the
    /// engaged regime: when the plane rises the soft body follows, so the
    /// signed distances — hence the force — barely change. The explicit-only
    /// factor ([`Self::contact_force_height_jacobian`]) therefore grossly
    /// overestimates the true sensitivity; this method is the corrected
    /// total. Validated against a black-box central FD of
    /// [`Self::resolved_contact_force`] in the contact-engaged, stable-
    /// active-set regime (the penalty active-set boundary is non-smooth;
    /// IPC the deferred cure). Hard-penalty scope (`d²E/dsd² = κ`); see
    /// `docs/keystone/s3_soft_pose_sensitivity_recon.md`.
    ///
    /// `height` is the contact-plane pose (the same parameter
    /// [`Self::contact_force_height_jacobian`] / [`Self::resolved_contact_force`]
    /// take) — supplied explicitly rather than read from the rigid pose so the
    /// derivative can be probed at a deeply-engaged height regardless of the
    /// current platen position.
    // `kappa` is the hard-penalty `d²E/dsd²`; the coupling runs hard penalty
    // (no smoothing window), so it is the exact per-active-pair curvature.
    #[must_use]
    pub fn contact_force_height_total_jacobian(&self, height: f64) -> Vec3 {
        // Raising the plane height = translating the rigid primitive +ẑ.
        self.contact_force_centre_total_jacobian(height, Vec3::new(0.0, 0.0, 1.0))
    }

    /// The total `∂(contact force)/∂(sphere-centre translation along `dir`)` — the
    /// axis-generic generalization of [`Self::contact_force_height_total_jacobian`]
    /// (which is the `dir = ẑ` height channel). This is the **moving-end-effector
    /// leaf**: tracking the arm tip moves the contact sphere centre in x/y as well as
    /// z, so the trajectory carry needs the sensitivity along each axis, not just the
    /// scalar height. `dir = ẑ` reproduces the height Jacobian exactly
    /// (`−n̂·ẑ = −n̂_z`); the lateral `x̂`/`ŷ` channels reuse the SAME explicit + implicit
    /// structure with `dir` substituted (the L0 `−H·u` pose sensitivity is already
    /// axis-generic — `soft_pose_sensitivity::sphere_pose_sensitivity_lateral_matches_resolve_fd`).
    ///
    /// The collider is posed at the current sphere-centre override (the end-effector,
    /// [`Self::set_sphere_center`]); `height` only feeds the default block-centroid
    /// posing when no override is set. Validated against a black-box re-solve FD per axis
    /// (`sphere_contact_total_jacobian.rs`). Same engaged / stable-active-set /
    /// hard-penalty scope as the height Jacobian.
    #[must_use]
    pub fn contact_force_centre_total_jacobian(&self, height: f64, dir: Vec3) -> Vec3 {
        let (solver, x_final) = self.soft_resolve(height);
        let dxstar = solver.equilibrium_pose_sensitivity(
            &x_final,
            None,
            self.cfg.dt,
            RigidTwist::translation(dir),
        );
        let positions: Vec<Vec3> = x_final
            .chunks_exact(3)
            .map(|c| Vec3::new(c[0], c[1], c[2]))
            .collect();
        let mut explicit = Vec3::zeros();
        let mut implicit = Vec3::zeros();
        for (v, g, n_hat, curv, _r) in self.active_pair_wrench_data(height, &positions) {
            // The per-pair soft force is gᵢ = f_mag·n̂; differentiating force = Σ f_mag·n̂
            // splits into a MAGNITUDE change (the cᵥ = d²E/dsd² terms, exact for a plane)
            // and a NORMAL-ROTATION change (the f_mag·∂n̂ terms — #415's geometric
            // stiffness `dE·H`, zero for the plane's constant normal, live for the sphere).
            let f_mag = g.dot(&n_hat);
            let h_mat = self.collider_hessian(height, positions[v]);
            let dxs = Vec3::new(dxstar[3 * v], dxstar[3 * v + 1], dxstar[3 * v + 2]);
            // explicit ∂force/∂δ|_x: magnitude `−cᵥ·(∂sd/∂δ)·n̂` (∂sd/∂δ = −n̂·dir as the
            // primitive translates +dir) + normal rotation `f_mag·∂n̂/∂δ` (∂n̂/∂δ = −H·dir).
            let dsd_ddir = -n_hat.dot(&dir);
            explicit += -curv * dsd_ddir * n_hat + f_mag * (h_mat * (-dir));
            // implicit (∂force/∂x)·∂x*/∂δ: magnitude `−cᵥ·n̂⊗n̂` + normal rotation `f_mag·H`
            // (∂n̂/∂x = H), contracted against the IFT pose sensitivity ∂x*/∂δ.
            implicit += -curv * n_hat.dot(&dxs) * n_hat + f_mag * (h_mat * dxs);
        }
        explicit + implicit
    }

    /// The rigid body's `(next vertical velocity vz', ∂vz'/∂xfrc_z)` for an
    /// applied vertical force `applied_fz`, from the current rigid state — the
    /// scalar factor for [`RigidStepVjp`]. Does NOT advance `self`.
    ///
    /// `vz'` is [`Self::rigid_step_probe`]; the factor is a central FD over it.
    /// For the free-joint platen under semi-implicit Euler the response is
    /// *exactly affine* in the force (`vz' = vz0 + (dt/m)·fz`), so the FD step
    /// `h` is immaterial and recovers the closed-form `dt/m` to machine
    /// precision (S2). **Scope:** the single scalar factor `∂vz'/∂xfrc_z` and
    /// its `h`-independence are valid only for a free / axis-decoupled body. A
    /// constrained or rotationally-coupled rigid configuration is non-affine in
    /// the force (then `h` is a truncation knob and a vertical force also
    /// induces horizontal/rotational response the scalar factor drops) — that
    /// case needs the full `∂s'/∂xfrc` (sim-core's `transition_derivatives`
    /// Jacobian) and is out of this keystone scene's scope.
    #[must_use]
    pub fn rigid_vz_response(&self, applied_fz: f64) -> (f64, f64) {
        let vz = self.rigid_step_probe(applied_fz).1;
        // Affine free body ⇒ the FD step size is immaterial; pick a benign one.
        let h = 1.0e-2;
        let dvz_dfz = (self.rigid_step_probe(applied_fz + h).1
            - self.rigid_step_probe(applied_fz - h).1)
            / (2.0 * h);
        (vz, dvz_dfz)
    }

    /// **The keystone soft-tape `VjpOp` crossing.** One chassis `Tape::backward`
    /// crossing BOTH engines: the gradient of the platen's next vertical
    /// velocity `vz'` w.r.t. a scalar `+ẑ` load `theta` applied to the
    /// `loaded` soft vertices. Returns `(vz', ∂vz'/∂theta)`. Does NOT advance
    /// `self`.
    ///
    /// Builds one differentiable coupled step on a single tape:
    /// ```text
    /// theta ─(soft Solver::step, NewtonStepVjp)→ x* ─(ContactForceVjp)→ fz
    ///        ─(neg)→ xfrc_z ─(RigidStepVjp)→ vz'
    /// ```
    /// then `tape.backward(vz')` and reads `grad(theta)`. The cotangent flows
    /// `vz' → xfrc → force → x* → theta`: the rigid + contact-force `VjpOp`s
    /// compose with the soft Newton load adjoint, so the reverse pass crosses
    /// the soft↔rigid interface on one tape — the gradient substrate the
    /// co-design optimizer consumes. (The handle is the soft LOAD `theta`, which
    /// the existing soft tape supports; differentiating soft MATERIAL params is
    /// the next leaf — it needs a soft material VJP — and rides the same
    /// crossing.)
    ///
    /// Reuses `self`'s soft block (rest geometry / material / pinned base) and
    /// rigid state; the contact plane is held at `height` and fixed across the
    /// step (staggered) — supplied explicitly (the same convention as
    /// [`Self::contact_force_height_jacobian`]) so the gradient can be probed at
    /// a deeply-engaged height regardless of the current platen position.
    /// FD-validated against the full coupled step in
    /// `tests/coupled_load_gradient.rs`. Engaged, stable-active-set,
    /// hard-penalty scope (the penalty active-set boundary is non-smooth — IPC
    /// deferred). See `docs/keystone/s4_vjp_crossing_recon.md`.
    ///
    /// # Panics
    /// Panics if the soft solver's tape step does not expose `x_final_var` (a
    /// solver-contract violation — `Solver::step` always pushes it).
    // expect_used: the missing-var case is a solver-contract bug surfaced
    // loudly, mirroring `step`'s divergence-panic rationale.
    #[allow(clippy::expect_used)]
    #[must_use]
    pub fn coupled_step_load_gradient(
        &self,
        height: f64,
        loaded: &[VertexId],
        theta: f64,
    ) -> (f64, f64) {
        let n = self.n_vertices;
        let mut solver = self.build_loaded_solver(height, loaded);

        // Build the one-step tape: theta → x* → fz → xfrc → vz'.
        let mut tape = Tape::new();
        let theta_var = tape.param_tensor(Tensor::from_slice(&[theta], &[1]));
        let step = solver.step(
            &mut tape,
            &Tensor::from_slice(&self.x, &[3 * n]),
            &Tensor::from_slice(&self.v, &[3 * n]),
            theta_var,
            self.cfg.dt,
        );
        let xstar_var = step
            .x_final_var
            .expect("Solver::step must push x_final_var onto the tape");
        let vz = self.run_crossing_tail(&mut tape, xstar_var, height, &step.x_final);
        let grad = tape.grad_tensor(theta_var).as_slice()[0];
        (vz, grad)
    }

    /// The S4 crossing tail on the shared tape: from the soft `x*` node
    /// (`xstar_var`, value shape `[3·n_vertices]`, evaluated at `x_final`),
    /// append `x* →[ContactForceVjp] fz →[neg] xfrc →[RigidStepVjp] vz'`, run
    /// `tape.backward(vz')`, and return `vz'`. The caller reads `grad` on
    /// whatever leaf feeds `xstar_var` (the load `θ` or a material parameter).
    /// Shared by [`Self::coupled_step_load_gradient`] and
    /// [`Self::coupled_step_material_gradient`].
    fn run_crossing_tail(
        &self,
        tape: &mut Tape,
        xstar_var: Var,
        height: f64,
        x_final: &[f64],
    ) -> f64 {
        // Contact force on the soft body at x*, with the active set + normals.
        let positions: Vec<Vec3> = x_final
            .chunks_exact(3)
            .map(|c| Vec3::new(c[0], c[1], c[2]))
            .collect();
        let fz: f64 = self
            .build_contact(height)
            .pair_readout(&self.fresh_mesh(), &positions)
            .iter()
            .map(|r| r.force_on_soft.z)
            .sum();
        let factors = self.active_pair_force_factors(height, &positions);
        let force_var = tape.push_custom(
            &[xstar_var],
            Tensor::from_slice(&[fz], &[1]),
            Box::new(ContactForceVjp::new(factors)),
        );
        // Reaction onto the rigid body, then the rigid step's velocity response.
        let xfrc_var = tape.neg(force_var); // xfrc_z = −force_on_soft.z
        let xfrc_z = tape.value_tensor(xfrc_var).as_slice()[0];
        let (vz, dvz_dfz) = self.rigid_vz_response(xfrc_z);
        let vz_var = tape.push_custom(
            &[xfrc_var],
            Tensor::from_slice(&[vz], &[1]),
            Box::new(RigidStepVjp::new(dvz_dfz)),
        );
        tape.backward(vz_var);
        vz
    }

    /// **The co-design gradient via the keystone crossing.** One chassis
    /// `Tape::backward` across both engines: the gradient of the platen's next
    /// vertical velocity `vz'` w.r.t. the soft block's Neo-Hookean material
    /// parameter (`param_idx`: `0 = μ`, `1 = λ`), holding the other fixed.
    /// Returns `(vz', ∂vz'/∂p)`. Does NOT advance `self`.
    ///
    /// Routes the material parameter through the SAME S4 chain the load gradient
    /// uses — only the soft node differs (the reverse-mode `MaterialStepVjp`
    /// from `sim-soft` in place of the load adjoint):
    /// ```text
    /// p ─[MaterialStepVjp]→ x* ─[ContactForceVjp]→ fz ─[neg]→ xfrc ─[RigidStepVjp]→ vz'
    /// ```
    /// `tape.backward(vz')` flows `vz' → xfrc → force → x* → p`. A stiffer soft
    /// body deforms less under the platen, changing the contact force — hence
    /// the platen's motion: this is the gradient the co-design optimizer
    /// consumes. (The keystone block ties `λ = 4μ`; a total `d/dμ` along that
    /// line is `∂/∂μ + 4·∂/∂λ` — a linear combination of the two `param_idx`
    /// results.) FD-validated against [`Self::coupled_step_material_vz`].
    /// Engaged / stable-active-set / hard-penalty scope.
    ///
    /// # Panics
    /// Panics if `param_idx` is not a valid Neo-Hookean parameter index
    /// (`0 = μ`, `1 = λ`).
    #[must_use]
    pub fn coupled_step_material_gradient(&self, height: f64, param_idx: usize) -> (f64, f64) {
        assert!(
            param_idx <= 1,
            "material param index {param_idx} out of range (0 = μ, 1 = λ)"
        );
        let n = self.n_vertices;
        let (solver, x_final) = self.soft_resolve(height);

        // p (leaf) → x* via the soft material VJP, then the shared crossing tail.
        let mut tape = Tape::new();
        let p0 = if param_idx == 0 { self.mu } else { self.lambda };
        let p_var = tape.param_tensor(Tensor::from_slice(&[p0], &[1]));
        let xstar_var = tape.push_custom(
            &[p_var],
            Tensor::from_slice(&x_final, &[3 * n]),
            Box::new(solver.material_step_vjp(&x_final, self.cfg.dt, param_idx)),
        );
        let vz = self.run_crossing_tail(&mut tape, xstar_var, height, &x_final);
        let grad = tape.grad_tensor(p_var).as_slice()[0];
        (vz, grad)
    }

    /// Forward-only companion to [`Self::coupled_step_material_gradient`]: the
    /// platen's next `vz'` with the soft block's material parameter `param_idx`
    /// set to `value` (the other held at `self`'s value), via a tape-free
    /// re-solve (soft step → contact force → rigid step). The black-box oracle
    /// for finite-differencing `∂vz'/∂p`. Does NOT advance `self`.
    ///
    /// # Panics
    /// Panics if `param_idx` is not a valid Neo-Hookean parameter index
    /// (`0 = μ`, `1 = λ`).
    #[must_use]
    pub fn coupled_step_material_vz(&self, height: f64, param_idx: usize, value: f64) -> f64 {
        assert!(
            param_idx <= 1,
            "material param index {param_idx} out of range (0 = μ, 1 = λ)"
        );
        let n = self.n_vertices;
        let (mu, lambda) = if param_idx == 0 {
            (value, self.lambda)
        } else {
            (self.mu, value)
        };
        let field = MaterialField::uniform(mu, lambda);
        let mesh = HandBuiltTetMesh::uniform_block(self.n_per_edge, self.edge, &field);
        let bc = BoundaryConditions::new(self.pinned.clone(), Vec::new());
        let solver: SoftSolver<C> =
            CpuNewtonSolver::new(Tet4, mesh, self.build_contact(height), self.cfg, bc);
        let st = solver.replay_step(
            &Tensor::from_slice(&self.x, &[3 * n]),
            &Tensor::from_slice(&self.v, &[3 * n]),
            &Tensor::zeros(&[0]),
            self.cfg.dt,
        );
        let positions: Vec<Vec3> = st
            .x_final
            .chunks_exact(3)
            .map(|c| Vec3::new(c[0], c[1], c[2]))
            .collect();
        let fz: f64 = self
            .build_contact(height)
            .pair_readout(
                &HandBuiltTetMesh::uniform_block(self.n_per_edge, self.edge, &field),
                &positions,
            )
            .iter()
            .map(|r| r.force_on_soft.z)
            .sum();
        self.rigid_step_probe(-fz).1
    }

    /// Forward-only companion to [`Self::coupled_step_load_gradient`]: the
    /// platen's next vertical velocity `vz'` for the load `theta` (re-solve the
    /// soft step → contact force → rigid step), with NO tape. The black-box
    /// oracle for finite-differencing the cross-engine gradient. Does NOT
    /// advance `self`.
    #[must_use]
    pub fn coupled_step_load_vz(&self, height: f64, loaded: &[VertexId], theta: f64) -> f64 {
        let n = self.n_vertices;
        let solver = self.build_loaded_solver(height, loaded);
        let st = solver.replay_step(
            &Tensor::from_slice(&self.x, &[3 * n]),
            &Tensor::from_slice(&self.v, &[3 * n]),
            &Tensor::from_slice(&[theta], &[1]),
            self.cfg.dt,
        );
        let positions: Vec<Vec3> = st
            .x_final
            .chunks_exact(3)
            .map(|c| Vec3::new(c[0], c[1], c[2]))
            .collect();
        let fz: f64 = self
            .build_contact(height)
            .pair_readout(&self.fresh_mesh(), &positions)
            .iter()
            .map(|r| r.force_on_soft.z)
            .sum();
        self.rigid_step_probe(-fz).1
    }

    /// **The keystone multi-step time-adjoint.** Roll the coupled system forward
    /// `n_steps` (advancing `self`, exactly as repeated [`Self::step`]) while
    /// recording a single chassis tape, then ONE `tape.backward` gives the
    /// gradient of the platen's final height `z_N` w.r.t. the soft block's
    /// Neo-Hookean material parameter (`param_idx`: `0 = μ`, `1 = λ`). Returns
    /// `(z_N, ∂z_N/∂p)`.
    ///
    /// Unlike the single-step [`Self::coupled_step_material_gradient`] (a
    /// non-mutating `&self` probe), this takes `&mut self` because it MUTATES the
    /// coupling: it runs the rollout in place (the `sim-core` `Data` is not
    /// `Clone`, so there is no copy to roll on). Build a fresh coupling per call.
    ///
    /// The tape threads the full coupled recurrence so the reverse pass crosses
    /// BOTH step boundaries and the soft↔rigid interface over the whole rollout:
    /// per step the soft solve is a `TrajectoryStepVjp` node with parents
    /// `[x_prev, v_prev, p, z_prev]` (the prev soft state, the material param, and
    /// the plane height `z_prev − clearance`); the next velocity
    /// `v = (x* − x_prev)/Δt` (`VelVjp`), the contact force
    /// `fz(x*, z_prev)` (`ContactForceTrajVjp`), and the rigid carry
    /// `vz' = a·vz − (Δt/m)·fz` (`VzCarryVjp`), `z' = z + Δt·vz`
    /// (`ZCarryVjp`, position integrated with the step's STARTING velocity — see
    /// its doc) chain the rigid state forward. `tape.backward(z_N)` then
    /// accumulates every per-step ∂/∂p (direct material + the state/contact/rigid
    /// feedback) into `p`. FD-validated against the full real coupled re-rollout.
    ///
    /// Forward values come from the real coupled dynamics (identical to
    /// [`Self::step`]); the per-node Jacobians are the analytic/factored
    /// sensitivities (the soft IFT factor, the contact penalty factors, the
    /// free-body `Δt/m` and damping `a = 1 − Δt·c/m`). See
    /// `docs/keystone/time_adjoint_recon.md`.
    ///
    /// **Accuracy / scope.** Each per-step factor is machine-exact
    /// (`TrajectoryStepVjp` is gated against re-solve FD in
    /// `sim-soft/tests/trajectory_step_vjp.rs`; the rigid carry against sim-core),
    /// and the composed multi-step gradient is machine-exact too — it matches the
    /// full-coupled FD to ~3e-8 (rel) at every rollout length and engagement
    /// depth, through genuine contact make/break alike (gated in
    /// `tests/coupled_trajectory_gradient.rs` for penalty and
    /// `tests/ipc_trajectory_gradient.rs` for IPC). The earlier ~3e-4 "penalty
    /// floor" — and the make/break degradation the keystone time-adjoint reported
    /// — was a rigid position-carry off-by-one, NOT penalty non-smoothness:
    /// sim-core integrates the height with the step's STARTING velocity
    /// (`z_{k+1} = z_k + Δt·vz_k`), so this step's contact force reaches `z` only
    /// next step, but `ZCarryVjp` had wired it to the freshly-updated `vz'` (see
    /// `docs/ipc/recon.md` §9). With the carry corrected both penalty and IPC are
    /// machine-clean. Free-body rigid factor (the keystone platen).
    ///
    /// # Panics
    /// Panics if the rigid step diverges (a mis-constructed coupling), or if the
    /// soft solver does not converge — surfaced loudly as in [`Self::step`].
    #[must_use]
    // One coherent per-step tape-construction loop (5 chained nodes + the real
    // coupled step); splitting it would scatter the recurrence's shared state.
    // expect_used: a rigid-step divergence is a programmer error surfaced loudly,
    // not recoverable for keystone-v1 (mirrors `step`'s rationale).
    #[allow(clippy::too_many_lines, clippy::expect_used)]
    pub fn coupled_trajectory_material_gradient(
        &mut self,
        n_steps: usize,
        param_idx: usize,
    ) -> (f64, f64) {
        // Free-body normal: sphere-capable (#418) but no moving-EE centre carry yet.
        self.require_no_moving_ee();
        assert!(
            param_idx <= 1,
            "material param index {param_idx} out of range (0 = μ, 1 = λ)"
        );
        let n = self.n_vertices;
        let dt = self.cfg.dt;
        let dir = Vec3::new(0.0, 0.0, 1.0);
        // Rigid carry coefficients: dt/m from the free-body probe (machine-gated
        // in tests/rigid_step_vjp.rs); the damping factor a = 1 − Δt·c/m = 1 −
        // c·(Δt/m). The full carry vz' = a·vz − (Δt/m)·fz + Δt·g reproduces the
        // real sim-core damped step to ~1e-16 (verified during development); a
        // regression is caught by the tightened end-to-end gate.
        let dt_over_m = self.rigid_vz_response(0.0).1;
        let a = 1.0 - self.rigid_damping * dt_over_m;

        let mut tape = Tape::new();
        // Material leaf (the gradient target) + the constant initial state.
        let p0 = if param_idx == 0 { self.mu } else { self.lambda };
        let p_var = tape.param_tensor(Tensor::from_slice(&[p0], &[1]));
        let mut x_var = tape.constant_tensor(Tensor::from_slice(&self.x, &[3 * n]));
        let mut v_var = tape.constant_tensor(Tensor::from_slice(&self.v, &[3 * n]));
        let mut z_var =
            tape.constant_tensor(Tensor::from_slice(&[self.data.xpos[self.body].z], &[1]));
        let mut vz_var = tape.constant_tensor(Tensor::from_slice(&[self.data.qvel[2]], &[1]));
        let mut z_final = self.data.xpos[self.body].z;

        for _ in 0..n_steps {
            let height = self.plane_height();
            let vz_k = self.data.qvel[2];

            // (1)+(2) one dynamic soft step from the current (self.x, self.v).
            let (solver, x_next) = self.soft_resolve(height);
            let v_next: Vec<f64> = x_next
                .iter()
                .zip(&self.x)
                .map(|(xf, xo)| (xf - xo) / dt)
                .collect();

            // Soft node: x* with parents [x_prev, v_prev, p, z_prev].
            let x_next_var = tape.push_custom(
                &[x_var, v_var, p_var, z_var],
                Tensor::from_slice(&x_next, &[3 * n]),
                Box::new(solver.trajectory_step_vjp(&x_next, dt, param_idx, dir)),
            );
            // Velocity node: v = (x* − x_prev)/Δt.
            let v_next_var = tape.push_custom(
                &[x_next_var, x_var],
                Tensor::from_slice(&v_next, &[3 * n]),
                Box::new(VelVjp {
                    inv_dt: 1.0 / dt,
                    n_dof: 3 * n,
                }),
            );

            // (3) contact force at the post-step config, plane at `height`.
            let positions: Vec<Vec3> = x_next
                .chunks_exact(3)
                .map(|c| Vec3::new(c[0], c[1], c[2]))
                .collect();
            let force_on_soft: Vec3 = self
                .build_contact(height)
                .pair_readout(&self.fresh_mesh(), &positions)
                .iter()
                .map(|r| r.force_on_soft)
                .sum();
            let factors = self.active_pair_force_factors(height, &positions);
            let fz_var = tape.push_custom(
                &[x_next_var, z_var],
                Tensor::from_slice(&[force_on_soft.z], &[1]),
                Box::new(ContactForceTrajVjp {
                    factors,
                    n_dof: 3 * n,
                }),
            );

            // (4)+(5) route the reaction (+ damping) onto the rigid body and
            // step it (the real coupled dynamics, identical to `step`).
            let mut sf = SpatialVector::zeros();
            sf[3] = -force_on_soft.x;
            sf[4] = -force_on_soft.y;
            sf[5] = -force_on_soft.z - self.rigid_damping * vz_k;
            self.data.xfrc_applied[self.body] = sf;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in coupled trajectory");
            let z_next = self.data.xpos[self.body].z;
            let vz_next = self.data.qvel[2];

            // Rigid carry nodes: vz' = a·vz − (Δt/m)·fz; z' = z + Δt·vz (the OLD
            // velocity — sim-core integrates position with the pre-update velocity,
            // so this step's contact force reaches z only NEXT step; verified
            // `z_next == z + dt·vz_prev` to machine zero). The z-update's velocity
            // parent is therefore `vz_var` (vz at the step's start), NOT the
            // freshly-updated `vz_next_var`.
            let vz_next_var = tape.push_custom(
                &[vz_var, fz_var],
                Tensor::from_slice(&[vz_next], &[1]),
                Box::new(VzCarryVjp {
                    a,
                    neg_dt_over_m: -dt_over_m,
                }),
            );
            let z_next_var = tape.push_custom(
                &[z_var, vz_var],
                Tensor::from_slice(&[z_next], &[1]),
                Box::new(ZCarryVjp { dt }),
            );

            // Advance the soft state and the tape handles.
            self.v = v_next;
            self.x = x_next;
            x_var = x_next_var;
            v_var = v_next_var;
            z_var = z_next_var;
            vz_var = vz_next_var;
            z_final = z_next;
        }

        tape.backward(z_var);
        let grad = tape.grad_tensor(p_var).as_slice()[0];
        (z_final, grad)
    }

    /// Analytic `∂(peak contact force fz over the trajectory)/∂param` for the normal-only coupled
    /// impact rollout — the de-escalation RQ2 design gradient (a buffer's peak strike force vs its
    /// material stiffness). Builds the SAME time-adjoint tape as
    /// [`Self::coupled_trajectory_material_gradient`] — the per-step `fz_var` (contact
    /// `force_on_soft.z`) is already a differentiable node — but backpropagates from the PEAK
    /// step's `fz_var` instead of the final height. This is the subgradient of `max_k |fz_k|` at
    /// its argmax, valid where the peak step is stable across the perturbation (it is for the
    /// impact scene; FD-validated to rel ~1e-9 by `cf-codesign`'s `peak_force_inverse_design`
    /// gate, which differences this against a re-rolled forward oracle). Returns
    /// `(peak_fz, ∂peak_fz/∂param, peak_step)`. `param_idx`: 0 = μ, 1 = λ; the coupling's λ=4μ tie
    /// ⇒ the design gradient is the total `grad(0) + 4·grad(1)`. Normal-only (no friction) ⇒ no
    /// PR3b adjoint gate. `&mut self`: advances the rollout, so rebuild a fresh coupling per call.
    #[must_use]
    // One coherent per-step tape-construction loop (mirrors `coupled_trajectory_material_gradient`,
    // tracking the peak fz_var); splitting it would scatter the recurrence's shared state.
    // expect_used: a rigid-step divergence is a programmer error surfaced loudly (mirrors `step`).
    #[allow(clippy::too_many_lines, clippy::expect_used)]
    pub fn coupled_trajectory_peak_force_gradient(
        &mut self,
        n_steps: usize,
        param_idx: usize,
    ) -> (f64, f64, usize) {
        // Free-body normal: sphere-capable but no moving-EE centre carry yet.
        self.require_no_moving_ee();
        assert!(
            param_idx <= 1,
            "material param index {param_idx} out of range (0 = μ, 1 = λ)"
        );
        let n = self.n_vertices;
        let dt = self.cfg.dt;
        let dir = Vec3::new(0.0, 0.0, 1.0);
        let dt_over_m = self.rigid_vz_response(0.0).1;
        let a = 1.0 - self.rigid_damping * dt_over_m;

        let mut tape = Tape::new();
        let p0 = if param_idx == 0 { self.mu } else { self.lambda };
        let p_var = tape.param_tensor(Tensor::from_slice(&[p0], &[1]));
        let mut x_var = tape.constant_tensor(Tensor::from_slice(&self.x, &[3 * n]));
        let mut v_var = tape.constant_tensor(Tensor::from_slice(&self.v, &[3 * n]));
        let mut z_var =
            tape.constant_tensor(Tensor::from_slice(&[self.data.xpos[self.body].z], &[1]));
        let mut vz_var = tape.constant_tensor(Tensor::from_slice(&[self.data.qvel[2]], &[1]));

        // Track the peak |fz| step and its tape node as we roll forward.
        let mut peak_abs = -1.0_f64;
        let mut peak_signed = 0.0_f64;
        let mut peak_step = 0_usize;
        let mut peak_fz_var: Option<Var> = None;

        for k in 0..n_steps {
            let height = self.plane_height();
            let vz_k = self.data.qvel[2];

            let (solver, x_next) = self.soft_resolve(height);
            let v_next: Vec<f64> = x_next
                .iter()
                .zip(&self.x)
                .map(|(xf, xo)| (xf - xo) / dt)
                .collect();

            let x_next_var = tape.push_custom(
                &[x_var, v_var, p_var, z_var],
                Tensor::from_slice(&x_next, &[3 * n]),
                Box::new(solver.trajectory_step_vjp(&x_next, dt, param_idx, dir)),
            );
            let v_next_var = tape.push_custom(
                &[x_next_var, x_var],
                Tensor::from_slice(&v_next, &[3 * n]),
                Box::new(VelVjp {
                    inv_dt: 1.0 / dt,
                    n_dof: 3 * n,
                }),
            );

            let positions: Vec<Vec3> = x_next
                .chunks_exact(3)
                .map(|c| Vec3::new(c[0], c[1], c[2]))
                .collect();
            let force_on_soft: Vec3 = self
                .build_contact(height)
                .pair_readout(&self.fresh_mesh(), &positions)
                .iter()
                .map(|r| r.force_on_soft)
                .sum();
            let factors = self.active_pair_force_factors(height, &positions);
            let fz_var = tape.push_custom(
                &[x_next_var, z_var],
                Tensor::from_slice(&[force_on_soft.z], &[1]),
                Box::new(ContactForceTrajVjp {
                    factors,
                    n_dof: 3 * n,
                }),
            );

            if force_on_soft.z.abs() > peak_abs {
                peak_abs = force_on_soft.z.abs();
                peak_signed = force_on_soft.z;
                peak_step = k;
                peak_fz_var = Some(fz_var);
            }

            let mut sf = SpatialVector::zeros();
            sf[3] = -force_on_soft.x;
            sf[4] = -force_on_soft.y;
            sf[5] = -force_on_soft.z - self.rigid_damping * vz_k;
            self.data.xfrc_applied[self.body] = sf;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in coupled trajectory");
            let z_next = self.data.xpos[self.body].z;
            let vz_next = self.data.qvel[2];

            let vz_next_var = tape.push_custom(
                &[vz_var, fz_var],
                Tensor::from_slice(&[vz_next], &[1]),
                Box::new(VzCarryVjp {
                    a,
                    neg_dt_over_m: -dt_over_m,
                }),
            );
            let z_next_var = tape.push_custom(
                &[z_var, vz_var],
                Tensor::from_slice(&[z_next], &[1]),
                Box::new(ZCarryVjp { dt }),
            );

            self.v = v_next;
            self.x = x_next;
            x_var = x_next_var;
            v_var = v_next_var;
            z_var = z_next_var;
            vz_var = vz_next_var;
        }

        let pv = peak_fz_var.expect("trajectory has at least one step with contact force");
        tape.backward(pv);
        let grad = tape.grad_tensor(p_var).as_slice()[0];
        (peak_signed, grad, peak_step)
    }

    // ===================== Multi-DOF (articulated) coupling (PR2) =====================

    /// The contact-plane height for an ARTICULATED body: the contacting point's
    /// world height (the body COM `xipos`, the contact point for a point-mass tip)
    /// minus the clearance. Unlike [`Self::plane_height`] (which reads the body
    /// ORIGIN `xpos` — the fixed joint pivot for a hinge), this tracks the moving
    /// tip. For the free-body platen `xipos == xpos`, so the two agree.
    fn tip_plane_height(&self) -> f64 {
        self.data.xipos[self.body].z - self.contact_clearance
    }

    /// `J_z`: the world-z row of the contacting body's COM spatial Jacobian
    /// (`mj_jac_point` at `xipos`, row 5) — `∂(tip height)/∂q`, the multi-DOF pose
    /// seam. Length `nv`.
    fn pose_seam_jz(&self) -> Vec<f64> {
        let jac = mj_jac_point(
            &self.model,
            &self.data,
            self.body,
            &self.data.xipos[self.body],
        );
        (0..self.model.nv).map(|c| jac[(5, c)]).collect()
    }

    /// `J_spatial`: the body's spatial Jacobian AT THE WORLD ORIGIN (`mj_jac_point` at
    /// the origin) — `6 × nv`, rows 0–2 angular `ω`, rows 3–5 linear `v_O` (the body
    /// material point at the origin). This is exactly `∂(plane spatial twist)/∂qpos`
    /// for the rotating normal: column `j` is the [`sim_soft::RigidTwist`] the plane
    /// undergoes per unit `qvel_j` (`δn̂ = ω×n̂`, `δoffset = v_O·n̂` reproduce
    /// `∂n̂/∂q = −[n̂]×·J_ang` and `∂offset/∂q = n̂ᵀ·J_lin + xiposᵀ·∂n̂/∂q`, S0-validated).
    /// Threaded by [`PoseTwistSeamVjp`] (`∂L/∂qpos = J_spatialᵀ·∂L/∂T`). Rotating-normal
    /// pose seam, PR2; see `docs/keystone/rotating_normal_recon.md`.
    fn pose_twist_jacobian(&self) -> DMatrix<f64> {
        mj_jac_point(&self.model, &self.data, self.body, &Vec3::zeros())
    }

    /// `J_lin = ∂c/∂qpos`: the body COM's linear (translational) world Jacobian
    /// (`mj_jac_point` at `xipos`, rows 3–5) — `3 × nv`. The moment about the COM
    /// `c = xipos(q)` depends on `q` through `c`, so [`ContactWrenchTrajVjp`] needs
    /// this to thread `∂τ/∂qpos = [f]_× · J_lin`. Read at the same (stale) FK config
    /// as the pose seam.
    fn com_linear_jacobian(&self) -> DMatrix<f64> {
        let jac = mj_jac_point(
            &self.model,
            &self.data,
            self.body,
            &self.data.xipos[self.body],
        );
        let nv = self.model.nv;
        let mut jlin = DMatrix::zeros(3, nv);
        for row in 0..3 {
            for col in 0..nv {
                jlin[(row, col)] = jac[(row + 3, col)]; // linear rows 3–5
            }
        }
        jlin
    }

    /// `J_geom = ∂(contact-geom centre)/∂qpos`: the contact end-effector geom's linear
    /// world Jacobian (`mj_jac_point` at `geom_xpos[contact_geom]`, rows 3–5) — `3 × nv`.
    /// The moving-end-effector pose seam: with [`Self::with_contact_geom`] the sphere
    /// centre rides the geom, so the trajectory adjoint threads `∂centre/∂q = J_geom`
    /// (the [`PoseCentreVjp`] seam), the 3-vector generalization of [`Self::pose_seam_jz`]'s
    /// scalar height channel. Read at the same (fresh-FK) config as the wrench node.
    ///
    /// # Panics
    /// Panics if no contact geom is set (`with_contact_geom` not called) — the centre
    /// channel is only meaningful for an end-effector-posed sphere — or if the contact geom is
    /// not rigidly attached to the contacting body (the coupling routes the reaction wrench to
    /// `self.body`, so the centre must ride it for the model to be coherent; `mj_jac_point`
    /// would otherwise return the Jacobian of the wrong body's material point).
    fn pose_centre_jacobian(&self) -> DMatrix<f64> {
        let g = self
            .contact_geom
            .expect("pose_centre_jacobian requires with_contact_geom");
        assert_eq!(
            self.model.geom_body[g], self.body,
            "the contact geom must be attached to the contacting body (self.body) — the moving-EE \
             centre Jacobian ∂(geom_xpos)/∂q treats the geom as rigidly fixed to self.body"
        );
        let jac = mj_jac_point(&self.model, &self.data, self.body, &self.data.geom_xpos[g]);
        let nv = self.model.nv;
        let mut jlin = DMatrix::zeros(3, nv);
        for row in 0..3 {
            for col in 0..nv {
                jlin[(row, col)] = jac[(row + 3, col)]; // linear rows 3–5
            }
        }
        jlin
    }

    /// One scratch rigid step from a supplied `(qpos, qvel)` with a held spatial
    /// `wrench` on `body`, returning `(qpos', qvel')`. Like [`Self::rigid_step_probe`]
    /// but for the full generalized state (the LOADED step map whose Jacobian is the
    /// multi-DOF carry). Does NOT advance `self` (`Data` is not `Clone`, so a fresh
    /// scratch is built).
    ///
    /// # Panics
    /// Panics if the scratch step diverges (a mis-constructed model).
    // expect_used: a scratch step on a valid model does not fail; a divergence is a
    // programmer error surfaced loudly (mirrors `rigid_step_probe`).
    #[allow(clippy::expect_used)]
    fn scratch_state_step(
        &self,
        qpos: &nalgebra::DVector<f64>,
        qvel: &nalgebra::DVector<f64>,
        wrench: &SpatialVector,
    ) -> (nalgebra::DVector<f64>, nalgebra::DVector<f64>) {
        let mut scratch = self.model.make_data();
        scratch.qpos.copy_from(qpos);
        scratch.qvel.copy_from(qvel);
        scratch.xfrc_applied[self.body] = *wrench;
        // Replicate the held control so the actuator force is present during the FD: on a
        // CHAIN the actuator force interacts with `∂M⁻¹/∂q`, so a `ctrl`-blind step would
        // drop that from the loaded `J_state`. `ctrl = 0` (no actuator / unactuated leaf)
        // ⇒ byte-identical to the original. (Single hinge: the constant force leaves
        // `J_state` unchanged, so this is a no-op there too.)
        scratch.ctrl.copy_from(&self.data.ctrl);
        scratch.step(&self.model).expect("articulated probe step");
        (scratch.qpos.clone(), scratch.qvel.clone())
    }

    /// [`rigid_xfrc_column`] (the wrench→qvel response `Δt·M⁻¹·Jᵀ`) at the current
    /// config — the config the real `step` maps the wrench through, matching the eval
    /// point of [`Self::loaded_state_jacobian`].
    ///
    /// Under the fully-fresh formulation the caller has already re-forwarded `self.data`
    /// at the current `qpos` this step (the per-step `forward` in
    /// `coupled_trajectory_material_gradient_articulated`), so `self.data`'s
    /// `xipos`/`qM`/`J` are current and `rigid_xfrc_column` reads them directly — no
    /// scratch forward needed. (Before the fresh-FK fix, `self.data` lagged `qpos` by one
    /// step, so this routine forwarded a scratch to remove a one-step skew that was
    /// material for the rotational/moment Jacobian; the fresh-FK formulation makes that
    /// scratch redundant. See `docs/keystone/moment_residual_recon.md` §3f.)
    fn fresh_xfrc_column(&self) -> DMatrix<f64> {
        rigid_xfrc_column(&self.model, &self.data, self.body)
    }

    /// The actuator-control VELOCITY response `∂(qvel')/∂ctrl = Δt·M_impl⁻¹·∂qfrc_actuator/∂ctrl`
    /// (`nv × nu`) at the current (fresh-FK) config — the actuator analog of the wrench
    /// column [`Self::fresh_xfrc_column`], routed through the actuator transmission instead
    /// of an applied wrench.
    ///
    /// `∂qfrc_actuator/∂ctrl` is computed by a forward-only central FD of `qfrc_actuator`
    /// (a scratch `forward` at `qpos`/`qvel` with `ctrl_a ± ε`). For the affine force law
    /// `force = gain·ctrl + bias` this is EXACT to roundoff (`qfrc_actuator` is affine in
    /// `ctrl`, so the central difference has no truncation error) and robust to how the
    /// transmission `moment`/`gain` are stored (the persisted `data.actuator_moment` is
    /// transient — cleared after forward — whereas `qfrc_actuator` is the reliable output).
    /// The POSITION rows are `D·(this)` (the same integrator tangent `D` as the wrench's
    /// `G_pos`), assembled by the caller. See `docs/keystone/actuator_dynamics_recon.md`.
    // expect_used: a scratch forward on a valid model does not fail (mirrors `scratch_state_step`).
    #[allow(clippy::expect_used)]
    fn actuator_velocity_column(&self) -> DMatrix<f64> {
        let (nv, nu) = (self.model.nv, self.model.nu);
        let eps = 1e-6;
        let qfrc = |a: usize, du: f64| -> DVector<f64> {
            let mut d = self.model.make_data();
            d.qpos.copy_from(&self.data.qpos);
            d.qvel.copy_from(&self.data.qvel);
            d.ctrl[a] += du;
            d.forward(&self.model)
                .expect("scratch forward for actuator Jacobian");
            d.qfrc_actuator.clone()
        };
        let mut dqfrc = DMatrix::zeros(nv, nu);
        for a in 0..nu {
            let col = (qfrc(a, eps) - qfrc(a, -eps)) / (2.0 * eps);
            for i in 0..nv {
                dqfrc[(i, a)] = col[i];
            }
        }
        self.model.timestep * implicit_mass_inverse(&self.model, &self.data) * dqfrc
    }

    /// The LOADED single-step state Jacobian `∂[qpos'; qvel']/∂[qpos; qvel]`
    /// (`2·nv × 2·nv`) at the current rigid state, with the contact `wrench` HELD —
    /// computed by central finite differences over [`Self::scratch_state_step`].
    ///
    /// This is the multi-DOF state carry. It is the *loaded* Jacobian (the wrench
    /// is held during the perturbation), so it includes the applied-force geometric
    /// stiffness `∂(Jᵀw)/∂q` that `sim-core`'s unloaded `transition_derivatives`
    /// drops — a real effect for an articulated body (zero for the free-body platen,
    /// where `J = I`).
    ///
    /// This is the **general fallback** path (any `nv`): the single-hinge AND the undamped
    /// serial-hinge-chain scopes use the machine-exact analytic [`Self::analytic_state_jacobian`]
    /// instead, while a free/quaternion joint or a DAMPED chain uses this FD form (FD-carry
    /// precision ~1e-6). It is also the reference the analytic form is FD-validated against;
    /// see `docs/keystone/geometric_stiffness_recon.md` and `docs/keystone/multilink_recon.md`.
    ///
    /// **Tangent-space FD (SO(3)-correct for quaternion joints).** When the body has a
    /// quaternion joint (`nq ≠ nv` — `qpos` holds un-normalized quaternion components on
    /// a curved manifold) the position-coordinate FD must move ALONG the manifold and
    /// difference in its tangent space, not a raw `qpos` add/subtract: position COLUMNS
    /// step via [`Self::loaded_state_jacobian_tangent`]'s [`mj_integrate_pos_explicit`]
    /// (`qpos ⊕ exp(±ε·e_c)`) and position ROWS difference via [`mj_differentiate_pos`]
    /// (the SO(3) log, the body-frame tangent at the nominal output). A purely Euclidean
    /// body (`nq == nv`: hinge/slide/translation only — e.g. a damped chain or a
    /// translation path) keeps the raw FD verbatim, BYTE-IDENTICAL. (The single hinge and
    /// the undamped serial-hinge chain use the analytic Jacobian and never reach here.) See
    /// `docs/keystone/quaternion_joints_recon.md`.
    fn loaded_state_jacobian(&self, wrench: &SpatialVector) -> DMatrix<f64> {
        if self.model.nq != self.model.nv {
            return self.loaded_state_jacobian_tangent(wrench);
        }
        let nv = self.model.nv;
        let eps = 1e-6;
        let qpos = self.data.qpos.clone();
        let qvel = self.data.qvel.clone();
        let mut j = DMatrix::zeros(2 * nv, 2 * nv);
        for c in 0..2 * nv {
            let (mut qp, mut vp) = (qpos.clone(), qvel.clone());
            let (mut qm, mut vm) = (qpos.clone(), qvel.clone());
            if c < nv {
                qp[c] += eps;
                qm[c] -= eps;
            } else {
                vp[c - nv] += eps;
                vm[c - nv] -= eps;
            }
            let (qpp, qvp) = self.scratch_state_step(&qp, &vp, wrench);
            let (qpm, qvm) = self.scratch_state_step(&qm, &vm, wrench);
            for r in 0..nv {
                j[(r, c)] = (qpp[r] - qpm[r]) / (2.0 * eps);
                j[(nv + r, c)] = (qvp[r] - qvm[r]) / (2.0 * eps);
            }
        }
        j
    }

    /// The quaternion-joint (`nq ≠ nv`) branch of [`Self::loaded_state_jacobian`]: the
    /// same loaded central-FD state Jacobian, but the position coordinates are perturbed
    /// and differenced in the SO(3)/SE(3) **tangent** space (the only correct linearization
    /// of a curved-manifold coordinate). The result is the `2·nv × 2·nv` tangent Jacobian
    /// `∂[tangent qpos'; qvel']/∂[tangent qpos; qvel]` — drop-in for the carry, whose state
    /// `s = [qpos(nv-tangent); qvel(nv)]` is already tangent-dimensioned. Velocity stays a
    /// raw `nv` tangent quantity (no quaternion). See `docs/keystone/quaternion_joints_recon.md`.
    fn loaded_state_jacobian_tangent(&self, wrench: &SpatialVector) -> DMatrix<f64> {
        let nv = self.model.nv;
        let eps = 1e-6;
        let qpos = self.data.qpos.clone();
        let qvel = self.data.qvel.clone();
        let mut j = DMatrix::zeros(2 * nv, 2 * nv);
        let mut tang = DVector::zeros(nv);
        let mut dq = DVector::zeros(nv);
        for c in 0..2 * nv {
            let (mut qp, mut vp) = (qpos.clone(), qvel.clone());
            let (mut qm, mut vm) = (qpos.clone(), qvel.clone());
            if c < nv {
                // Position column: step ±ε along the tangent basis vector e_c on SO(3)
                // (a hinge/slide/translation DOF reduces to `qpos[c] ± ε`).
                tang[c] = eps;
                mj_integrate_pos_explicit(&self.model, &mut qp, &qpos, &tang, 1.0);
                tang[c] = -eps;
                mj_integrate_pos_explicit(&self.model, &mut qm, &qpos, &tang, 1.0);
                tang[c] = 0.0;
            } else {
                vp[c - nv] += eps;
                vm[c - nv] -= eps;
            }
            let (qpp, qvp) = self.scratch_state_step(&qp, &vp, wrench);
            let (qpm, qvm) = self.scratch_state_step(&qm, &vm, wrench);
            // Position rows: tangent (SO(3) log) difference of the `qpos'` outputs.
            mj_differentiate_pos(&self.model, &mut dq, &qpm, &qpp, 2.0 * eps);
            for r in 0..nv {
                j[(r, c)] = dq[r];
                j[(nv + r, c)] = (qvp[r] - qvm[r]) / (2.0 * eps);
            }
        }
        j
    }

    /// The integrator's tangent position Jacobian `D = ∂(tangent qpos')/∂qvel'` (`nv × nv`,
    /// block-diagonal per joint) at the post-step velocity `qvel_next`. Semi-implicit Euler
    /// maps `qpos' = qpos ⊕ exp(Δt·qvel')`; differentiating w.r.t. `qvel'` (output tangent
    /// at the nominal `qpos'`) gives, per joint: `Δt` for a hinge/slide/free-translation DOF
    /// (the linear limit `J_r = I`), and `Δt·J_r(Δt·ω)` for a ball / free-orientation block
    /// (`ω` = that joint's angular velocity, [`right_jacobian_so3`]). The carry's position-row
    /// wrench response is then `G_pos = D·G_vel`. Only the quaternion path calls this; the
    /// Euclidean path uses `Δt·G_vel` directly (byte-identical). See
    /// `docs/keystone/quaternion_joints_recon.md`.
    fn integrator_pos_jacobian(&self, qvel_next: &DVector<f64>) -> DMatrix<f64> {
        let nv = self.model.nv;
        let dt = self.model.timestep;
        let mut d = DMatrix::zeros(nv, nv);
        for jnt in 0..self.model.njnt {
            let dof = self.model.jnt_dof_adr[jnt];
            match self.model.jnt_type[jnt] {
                MjJointType::Hinge | MjJointType::Slide => {
                    d[(dof, dof)] = dt;
                }
                MjJointType::Ball => {
                    let omega = Vec3::new(qvel_next[dof], qvel_next[dof + 1], qvel_next[dof + 2]);
                    d.view_mut((dof, dof), (3, 3))
                        .copy_from(&(dt * right_jacobian_so3(dt * omega)));
                }
                MjJointType::Free => {
                    for i in 0..3 {
                        d[(dof + i, dof + i)] = dt; // linear (translation) block
                    }
                    let omega =
                        Vec3::new(qvel_next[dof + 3], qvel_next[dof + 4], qvel_next[dof + 5]);
                    d.view_mut((dof + 3, dof + 3), (3, 3))
                        .copy_from(&(dt * right_jacobian_so3(dt * omega)));
                }
            }
        }
        d
    }

    /// The body's joint id if its kinematic chain to the world is exactly one
    /// **hinge** with no other DOFs (`nv == 1`, hence `nq == nv` — raw `qpos` is a
    /// valid coordinate, no quaternion). `None` for a free joint, a multi-link chain,
    /// or any non-hinge case. The single hinge uses the scalar closed-form geometric
    /// stiffness in [`Self::analytic_state_jacobian`]; the multi-link chain (damped or
    /// undamped) uses the off-diagonal Hessian + `∂M_impl⁻¹/∂q` form in
    /// [`Self::chain_state_jacobian`]; the remaining cases (free/quaternion) keep the FD
    /// [`Self::loaded_state_jacobian`] fallback.
    fn single_hinge(&self) -> Option<usize> {
        if self.model.nv != 1 || self.model.body_jnt_num[self.body] != 1 {
            return None;
        }
        let jnt = self.model.body_jnt_adr[self.body];
        (self.model.jnt_type[jnt] == sim_core::MjJointType::Hinge).then_some(jnt)
    }

    /// The LOADED single-step state Jacobian `∂[qpos'; qvel']/∂[qpos; qvel]` computed
    /// **analytically** for a single hinge — the machine-exact successor to the FD'd
    /// [`Self::loaded_state_jacobian`]. `None` for any non-single-hinge body (the
    /// caller falls back to the FD form).
    ///
    /// `J_state = A + Δt·M⁻¹·∂(Jᵀw)/∂q` (velocity rows; the position rows follow the
    /// semi-implicit integrator, scaled by `Δt`), where `A` is the **unloaded**
    /// transition ([`Data::transition_derivatives`]) and the second term is the
    /// applied wrench's geometric/load stiffness that `A` drops. This replaces the FD
    /// loaded Jacobian's noise with the exact term (deterministic, no eps) — making the
    /// single-hinge articulated gradient machine-exact at every horizon (paired with the
    /// fully-fresh formulation; see `coupled_trajectory_material_gradient_articulated`
    /// and `docs/keystone/moment_residual_recon.md`). The undamped multi-link chain is
    /// handled by [`Self::chain_state_jacobian`] (this method dispatches to it).
    ///
    /// For a single hinge the geometric stiffness is the closed form
    /// `∂(Jᵀw)/∂θ = (â×(â×r))·f`: with world axis `â`, moment arm `r = xipos − anchor`,
    /// and the wrench's linear part `f`, only `r` rotates with `θ` (`∂â/∂θ = 0`,
    /// `∂anchor/∂θ = 0`), so `∂r/∂θ = â×r` and `∂(â×r)/∂θ = â×(â×r)`; the torque part
    /// drops (`∂â/∂θ = 0`). `M` is configuration-independent for a single hinge (the
    /// body inertia about the fixed axis), so `∂M⁻¹/∂q = 0` and the `Δt·M⁻¹·…` form is
    /// exact. Evaluated at a fresh scratch forward at `qpos` (the config the real
    /// `step` maps the wrench through, matching [`Self::fresh_xfrc_column`]).
    ///
    /// **Joint damping (eulerdamp).** Under the Euler integrator MuJoCo solves the velocity
    /// update with the IMPLICIT factor `M_impl = M + Δt·D` (`D = implicit_damping`), not bare
    /// `M`. The unloaded `A` from `transition_derivatives` already accounts for this — sim-core
    /// routes its Euler velocity solve through `M_impl` when any DOF is damped (see
    /// `derivatives/hybrid.rs`). So the carry only adds the LOADED geometric stiffness, routed
    /// through the same `M_impl` (`∂(M_impl⁻¹·Jᵀw)/∂θ = geom_stiff / M_impl`), and the position
    /// rows follow the semi-implicit chain `θ' = θ + Δt·ω'`. `M_impl` is configuration-independent
    /// for a single hinge (constant `M`, constant `D`), so `∂M_impl⁻¹/∂q = 0` and the form stays
    /// exact — machine-exact vs the FD [`Self::loaded_state_jacobian`] (which differentiates the
    /// real eulerdamp step). `D = 0 ⇒ M_impl = M`, recovering the bare-`M` path BYTE-FOR-BYTE.
    // expect_used: a transition-derivative / forward on a valid model does not fail; a
    // failure is a programmer error surfaced loudly (mirrors `scratch_state_step`).
    #[allow(clippy::expect_used)]
    fn analytic_state_jacobian(&self, wrench: &SpatialVector) -> Option<DMatrix<f64>> {
        // The analytic carry assumes the Euler (eulerdamp) integrator: damping enters the
        // velocity solve as `M_impl = M + Δt·D`, with joint stiffness applied EXPLICITLY. A
        // stiffness-implicit integrator (`ImplicitSpringDamper`: `M + Δt·D + Δt²·K`) needs a
        // different solve matrix, so defer to the always-correct FD `loaded_state_jacobian`.
        // (Before damped chains were analytic, the damping guard incidentally excluded such
        // models — which are damped — so this preserves that protection explicitly.)
        if self.model.integrator != sim_core::Integrator::Euler {
            return None;
        }
        let Some(jnt) = self.single_hinge() else {
            // Not a single hinge: try the analytic multi-link chain carry (damped or undamped,
            // `Self::chain_state_jacobian`); a free joint / quaternion / multi-joint-per-body
            // body returns `None` there → the caller's FD `loaded_state_jacobian` fallback.
            return self.chain_state_jacobian(wrench);
        };
        let dt = self.model.timestep;
        let nv = self.model.nv; // == 1 (single_hinge), so the state block is 2×2
        // Unloaded transition A at the current state. The real `step` re-forwards at
        // `qpos`, so A's internal perturbations evaluate at the same fresh config the
        // carry uses (`self.data`'s `xipos`/`qM` lag a step, but `step`/`forward` redo FK).
        // A is (2·nv + na)²; take the [qpos; qvel] block (the carry's 2·nv × 2·nv state).
        let a = self
            .data
            .transition_derivatives(&self.model, &sim_core::DerivativeConfig::default())
            .expect("unloaded transition derivatives")
            .A;
        let mut j = a.view((0, 0), (2 * nv, 2 * nv)).into_owned();
        // Geometric stiffness at a fresh scratch forward at `qpos` (matching the FD
        // `loaded_state_jacobian` / `fresh_xfrc_column` eval point).
        let mut scratch = self.model.make_data();
        scratch.qpos.copy_from(&self.data.qpos);
        scratch.qvel.copy_from(&self.data.qvel);
        scratch.forward(&self.model).expect("scratch forward");
        let jb = self.model.jnt_body[jnt];
        let axis = scratch.xquat[jb] * self.model.jnt_axis[jnt];
        let anchor = scratch.xpos[jb] + scratch.xquat[jb] * self.model.jnt_pos[jnt];
        let r = scratch.xipos[self.body] - anchor;
        let f = Vec3::new(wrench[3], wrench[4], wrench[5]);
        let geom_stiff = axis.cross(&axis.cross(&r)).dot(&f); // ∂(Jᵀw)/∂θ
        let m = scratch.qM[(0, 0)];
        let damp = self.model.implicit_damping[0]; // single hinge ⇒ the only DOF
        // `A` (= `j`, from `transition_derivatives`) is ALREADY the eulerdamp-correct
        // unloaded transition: sim-core routes the Euler velocity solve through
        // `M_impl = M + Δt·D` under damping (and through bare `M` when undamped). So the
        // carry only patches the LOADED geometric stiffness onto the qpos column, routed
        // through the same `M_impl` (`∂(M_impl⁻¹·Jᵀw)/∂θ = geom_stiff / M_impl`); the
        // position rows follow the semi-implicit chain `θ' = θ + Δt·ω'`. `M_impl = M`
        // when undamped, so the `D = 0` result is unchanged BYTE-FOR-BYTE.
        let m_impl = m + dt * damp;
        let vel_corr = dt / m_impl * geom_stiff; // ∂qvel'/∂qpos correction
        j[(1, 0)] += vel_corr; // velocity row, qpos col
        j[(0, 0)] += dt * vel_corr; // position row, qpos col (semi-implicit chain)
        Some(j)
    }

    /// The LOADED single-step state Jacobian computed **analytically** for a serial HINGE chain
    /// (`nv ≥ 2`), with or without joint damping — the multi-link successor to the single-hinge
    /// [`Self::analytic_state_jacobian`], removing the last FD fallback for the articulated
    /// coupling. `None` (→ FD `loaded_state_jacobian`) for any body that is not a pure serial
    /// hinge chain spanning all DOFs (one joint per body) or a quaternion joint (`nq ≠ nv`).
    /// Joint damping is handled by routing the loaded term through `M_impl = M + Δt·D` (the Euler
    /// eulerdamp solve matrix); the unloaded `A` is eulerdamp-correct from sim-core. Machine-exact
    /// for 2-link through 4-link spatial chains (FD-validated damped and undamped); the multi-hop
    /// Coriolis derivative it relies on is complete in `mjd_rne_pos`.
    ///
    /// **The decomposition.** The loaded single-step transition is exactly
    /// `loaded_J = A + Δt·∂(M_impl⁻¹·Jᵀw)/∂q` on the velocity-position block and `+ Δt²·∂(…)` on
    /// the position-position block (semi-implicit `q' = q + Δt·v'`), where `M_impl = M + Δt·D` is
    /// the Euler eulerdamp solve matrix (`= M` when undamped), and:
    /// - `A` is the **unloaded** transition ([`Data::transition_derivatives`]) — already
    ///   machine-exact (its `∂v'/∂q` block captures the mass-matrix config dependence via
    ///   `sim-core`'s `mjd_rne_pos` inertia-transport derivatives, and it routes the Euler
    ///   velocity solve through `M_impl` under damping).
    /// - `∂(M_impl⁻¹·Jᵀw)/∂q = M_impl⁻¹·(G − dMu)`, with `G = ∂(Jᵀw)/∂q` the applied-wrench
    ///   geometric stiffness Hessian and `dMu = (∂M/∂q)·u` (`u = M_impl⁻¹·Jᵀw`; `∂M_impl/∂q =
    ///   ∂M/∂q` since `D` is constant) the mass-matrix directional derivative
    ///   [`sim_core::mass_directional_derivative`]. The `(∂M_impl⁻¹/∂q)·Jᵀw = −M_impl⁻¹·dMu`
    ///   term VANISHES for a single hinge (constant `M`) but is MATERIAL for a chain (~14% of
    ///   the wrench contribution); dropping it ships a ~10%-wrong gradient.
    ///
    /// **The geometric-stiffness Hessian `G[j,k] = ∂(Jᵀw)_j/∂q_k`.** With `(Jᵀw)_j = â_j·(τ +
    /// r_j×f)` (joint `j`: world axis `â_j`, anchor `o_j`, arm `r_j = p − o_j` to the contact
    /// COM `p`; wrench `w = [τ; f]`):
    /// `G[j,k] = (∂â_j/∂q_k)·(τ + r_j×f) + â_j·((∂r_j/∂q_k)×f)`, with the ancestor-dependent
    /// kinematics `∂â_j/∂q_k = â_k×â_j` and `∂o_j/∂q_k = â_k×(o_j−o_k)` (both only when `k` is a
    /// STRICT ancestor of `j`, else `0`), and `∂p/∂q_k = â_k×(p−o_k)` (for any chain joint `k`),
    /// so `∂r_j/∂q_k = ∂p/∂q_k − ∂o_j/∂q_k`. For a single hinge (`j=k`) this reduces to the
    /// closed form `(â×(â×r))·f` of [`Self::analytic_state_jacobian`]. ✓
    ///
    /// All kinematics are read at a FRESH scratch `forward` at `qpos` (matching the FD
    /// `loaded_state_jacobian` / [`Self::fresh_xfrc_column`] eval point). See
    /// `docs/keystone/multilink_recon.md` and `project-analytic-chain-jstate-subpr2` (the spec).
    // expect_used: a transition-derivative / forward on a valid model does not fail; a
    // failure is a programmer error surfaced loudly (mirrors `analytic_state_jacobian`).
    #[allow(clippy::expect_used)]
    fn chain_state_jacobian(&self, wrench: &SpatialVector) -> Option<DMatrix<f64>> {
        let model = &self.model;
        // Scope guard: undamped serial HINGE chain spanning all DOFs, Euclidean (nq == nv),
        // nv ≥ 2. Validated machine-exact for 2-link through (at least) 4-link spatial chains —
        // the sim-core Coriolis derivative (the `∂S/∂q` ancestor term AND the bias-acceleration
        // X_b transport that the multi-hop case needs) is now complete for any serial hinge
        // chain. See `docs/keystone/multilink_recon.md`.
        if model.nq != model.nv || model.nv < 2 {
            return None;
        }
        let chain = &model.body_ancestor_joints[self.body];
        // The chain must be exactly the model's DOFs (each hinge = 1 DOF), all hinges, one
        // joint per body. Joint DAMPING is supported: the loaded term routes through `M_impl`
        // below, and the unloaded `A` is eulerdamp-correct from sim-core. The one-joint-per-body
        // guard matters: with multiple joints on a body, this body's world axis/anchor read
        // (`xquat[body] * jnt_axis`, the FINAL body frame) differs from FK's running-frame axis
        // for a non-last same-body joint, and `is_ancestor` flags same-body joints as mutual
        // strict ancestors — so such a model would get a wrong analytic `J_state` instead of the
        // safe FD fallback.
        if chain.len() != model.nv
            || !chain
                .iter()
                .all(|&j| model.jnt_type[j] == MjJointType::Hinge)
            || !chain
                .iter()
                .all(|&j| model.body_jnt_num[model.jnt_body[j]] == 1)
        {
            return None;
        }

        let dt = model.timestep;
        let nv = model.nv;

        // Fresh scratch at `qpos`/`qvel` with the CONTACT WRENCH CLEARED (`xfrc_applied = 0`)
        // but the held control copied — the UNLOADED operating point. `self.data` carries the
        // previous step's `xfrc_applied` (sim-core's `step` does not clear it), which would
        // poison the analytical transition's `∂(M·qacc)/∂q` term via the contaminated
        // operating-point `qacc` (∂M/∂q ≠ 0 for a chain — harmless for a single hinge where
        // ∂M/∂q = 0, which is why `analytic_state_jacobian` can read `self.data` directly).
        // A is the unloaded transition; the held contact wrench enters only through `addend`.
        let mut scratch = model.make_data();
        scratch.qpos.copy_from(&self.data.qpos);
        scratch.qvel.copy_from(&self.data.qvel);
        scratch.ctrl.copy_from(&self.data.ctrl); // actuator input is a held input of the transition
        scratch.forward(model).expect("scratch forward");

        // Unloaded transition A at the clean operating point (undamped ⇒ machine-exact for the
        // chain, given the sim-core Coriolis `∂S/∂q` ancestor term).
        let a = scratch
            .transition_derivatives(model, &sim_core::DerivativeConfig::default())
            .expect("unloaded transition derivatives")
            .A;
        let mut j = a.view((0, 0), (2 * nv, 2 * nv)).into_owned();

        let p = scratch.xipos[self.body]; // contact-body COM
        let tau = Vec3::new(wrench[0], wrench[1], wrench[2]);
        let f = Vec3::new(wrench[3], wrench[4], wrench[5]);

        // Per-joint world axis â and anchor o (mirror the single-hinge eval).
        let axis: Vec<Vec3> = chain
            .iter()
            .map(|&jj| scratch.xquat[model.jnt_body[jj]] * model.jnt_axis[jj])
            .collect();
        let anchor: Vec<Vec3> = chain
            .iter()
            .map(|&jj| {
                scratch.xpos[model.jnt_body[jj]]
                    + scratch.xquat[model.jnt_body[jj]] * model.jnt_pos[jj]
            })
            .collect();

        // Geometric-stiffness Hessian G[dof_j, dof_k] = ∂(Jᵀw)_j/∂q_k (case-split closed form).
        let mut g = DMatrix::zeros(nv, nv);
        for (ji, &jj) in chain.iter().enumerate() {
            let a_j = axis[ji];
            let r_j = p - anchor[ji];
            let moment_j = tau + r_j.cross(&f); // τ + r_j×f
            let dof_j = model.jnt_dof_adr[jj];
            for (ki, &kk) in chain.iter().enumerate() {
                let a_k = axis[ki];
                // ∂p/∂q_k = â_k×(p−o_k) (k is always an ancestor of the contact body).
                let dp = a_k.cross(&(p - anchor[ki]));
                // ∂â_j/∂q_k, ∂o_j/∂q_k nonzero only when k is a STRICT ancestor of j.
                let strict_anc = kk != jj && model.is_ancestor(model.jnt_body[jj], kk);
                let (daxis_j, do_j) = if strict_anc {
                    (a_k.cross(&a_j), a_k.cross(&(anchor[ji] - anchor[ki])))
                } else {
                    (Vec3::zeros(), Vec3::zeros())
                };
                let dr_j = dp - do_j; // ∂r_j/∂q_k
                let dof_k = model.jnt_dof_adr[kk];
                g[(dof_j, dof_k)] = daxis_j.dot(&moment_j) + a_j.dot(&dr_j.cross(&f));
            }
        }

        // The loaded velocity-row term is Δt·∂(M_impl⁻¹·Jᵀw)/∂q, where `M_impl = M + Δt·D` is the
        // Euler eulerdamp solve matrix (= M when undamped). Expanding with ∂M_impl/∂q = ∂M/∂q
        // (D constant): ∂(M_impl⁻¹·Jᵀw)/∂q = M_impl⁻¹·(G − dMu), with u = M_impl⁻¹·Jᵀw and
        // dMu = (∂M/∂q)·u. The unloaded `A` is already eulerdamp-correct from sim-core (it routes
        // the Euler velocity solve through M_impl under damping), so the carry only adds this
        // loaded term over the SAME M_impl — mirroring the single-hinge unification. `D = 0 ⇒
        // M_impl = M`, recovering the bare-M form BYTE-FOR-BYTE.
        let mut m_impl = scratch.qM.clone();
        for i in 0..nv {
            m_impl[(i, i)] += dt * model.implicit_damping[i];
        }
        let m_inv = m_impl.try_inverse().expect("M_impl invertible");
        let jac = mj_jac_point(model, &scratch, self.body, &p); // 6×nv
        let mut w_vec = DVector::zeros(6);
        for i in 0..6 {
            w_vec[i] = wrench[i];
        }
        let jtw = jac.transpose() * &w_vec; // nv
        let u = &m_inv * &jtw; // u = M_impl⁻¹·Jᵀw
        let d_mu = mass_directional_derivative(model, &mut scratch, &u); // nv×nv
        let addend = dt * &m_inv * (&g - &d_mu); // Δt·M_impl⁻¹·(G − dMu)

        // Velocity rows + semi-implicit position rows get the loaded correction on the qpos
        // columns (mirrors the single-hinge `vel_corr` / `dt·vel_corr` patch).
        for r in 0..nv {
            for c in 0..nv {
                j[(nv + r, c)] += addend[(r, c)]; // velocity row, qpos col
                j[(r, c)] += dt * addend[(r, c)]; // position row, qpos col (semi-implicit)
            }
        }
        Some(j)
    }

    /// Forward-only oracle for [`Self::coupled_trajectory_material_gradient_articulated`]:
    /// roll the ARTICULATED coupled system forward `n_steps` (pose the plane from
    /// the moving tip, soft solve, route the contact reaction at the body COM, step)
    /// and return the tip world height `xipos[body].z` after the rollout. No tape;
    /// the black-box oracle for finite-differencing the multi-DOF coupled gradient.
    /// Advances `self` (build a fresh coupling per call).
    ///
    /// # Panics
    /// Panics if the rigid step diverges or the soft solver fails (as in [`Self::step`]).
    #[must_use]
    // expect_used: a rigid-step divergence / soft non-convergence is a programmer
    // error surfaced loudly, not recoverable for keystone-v1 (mirrors `step`).
    #[allow(clippy::expect_used)]
    pub fn coupled_trajectory_articulated_z(&mut self, n_steps: usize) -> f64 {
        for _ in 0..n_steps {
            // FRESH FK: pose the contact plane / COM from the CURRENT config (q_k).
            // sim-core's `step` is forward-then-integrate with no trailing FK, so
            // `self.data`'s `xipos`/`qM` lag `qpos` one step; re-forward so the contact
            // is posed at the actual current tip (no one-step lag). This fresh
            // formulation (paired with the gradient method's fresh output + true
            // position-row carry) is machine-exact for single-hinge AND multi-link; the
            // earlier stale-FK convention was a single-hinge-only calibration. See
            // `docs/keystone/moment_residual_recon.md`.
            self.data.forward(&self.model).expect("fresh FK");
            // MOVING END-EFFECTOR: pose the sphere at the contact geom (the arm tip) each
            // step — the forward companion of the moving-EE centre gradient, so the FD
            // oracle and the adjoint share the per-step posing. Sphere-only (the plane
            // ignores the override); a no-op (override stays None ⇒ block-centroid) when
            // `with_contact_geom` is not set or the collider is the plane (byte-identical).
            if let (Some(g), Collider::Sphere { .. }) = (self.contact_geom, self.collider) {
                self.sphere_center_override = Some(self.data.geom_xpos[g]);
            }
            let height = self.tip_plane_height();
            let (_solver, x_next) = self.soft_resolve(height);
            for (vi, (&xf, &xo)) in self.v.iter_mut().zip(x_next.iter().zip(self.x.iter())) {
                *vi = (xf - xo) / self.cfg.dt;
            }
            self.x = x_next;
            // Route the full contact wrench [τ; f] at the COM (the off-COM moment
            // τ = Σ rᵢ × fᵢ, NOT a pure force at the COM), about the fresh-FK COM.
            self.data.xfrc_applied[self.body] = self.contact_wrench(height);
            self.data
                .step(&self.model)
                .expect("rigid step diverged in articulated rollout");
        }
        // Return the FRESH post-rollout tip height (forward at q_N — the same fresh
        // convention the gradient method's objective reads).
        self.data.forward(&self.model).expect("fresh FK (output)");
        self.data.xipos[self.body].z
    }

    /// **The multi-DOF (articulated) coupled trajectory gradient — with the off-COM
    /// contact MOMENT.** The articulated successor to
    /// [`Self::coupled_trajectory_material_gradient`]: the rigid body is an
    /// ARTICULATED mechanism (e.g. a hinge), not a free platen, so the soft contact
    /// reaction maps to a generalized joint acceleration coupled across joints (the
    /// matrix `Δt·M⁻¹·Jᵀ`, [`rigid_xfrc_column`], not the scalar `dt/m`) and the
    /// contact-plane pose tracks the moving tip (`∂(tip height)/∂q = J_z`, the
    /// `PoseSeamVjp` seam). The reaction is routed as the full spatial **wrench**
    /// `[τ; f]` about the body COM — including the off-COM moment
    /// `τ = −Σ(rᵢ − c) × gᵢ` (`gᵢ = force_on_softᵢ`, `c = xipos`) that the symmetric
    /// platen drops and an off-center articulated contact does not. Rolls forward
    /// `n_steps` (advancing `self`) on one chassis tape, then ONE `tape.backward`
    /// gives `(tip_z_N, ∂tip_z_N/∂p)` — the tip's final world height vs the soft
    /// block's material (`param_idx`: `0 = μ`, `1 = λ`).
    ///
    /// **Moving end-effector** ([`Self::with_contact_geom`]). When the finite sphere rides a
    /// rigid geom (the arm tip), its centre `c = geom_xpos(q)` translates in x/y/z as the body
    /// swings, so the scalar height pose channel generalizes to the 3-vector centre: the pose
    /// seam becomes `∂c/∂q = J_geom` (`PoseCentreVjp`, `pose_centre_jacobian`), the
    /// soft node feeds the three translation axes, and the wrench node's pose parent is
    /// `WrenchPose::Centre` (the lateral `f_mag·H` + magnitude feedback the height channel
    /// drops). Machine-exact vs the geom-posed re-rollout FD (`sphere_moving_ee_trajectory_gradient.rs`).
    /// With no contact geom set the centre defaults to the block centroid and the channel reduces
    /// to the scalar height (byte-identical). Combining it with `with_rotating_normal` is not yet
    /// supported (asserted).
    ///
    /// Per step the tape threads: the pose seam `h = PoseSeam(s)` (plane height from
    /// the rigid state `s = [qpos; qvel]`), the soft solve `x*` (the same
    /// `TrajectoryStepVjp` as the platen path, pose parent `= h`), the velocity
    /// readout, the contact **wrench** `w = [τ; f](x*, h, s)` (`ContactWrenchTrajVjp`
    /// — `∂w/∂x*` the moment's two parts, `∂w/∂h` the force/moment-vs-height feedback,
    /// `∂w/∂s` the moment's `c(q)` feedback), and the multi-DOF rigid carry
    /// `s' = J_state·s + G·w` (`RigidStateCarryVjp`) where `J_state` is the LOADED
    /// single-step Jacobian — for a single hinge the **analytic** geometric stiffness
    /// `A + Δt·M⁻¹·∂(Jᵀw)/∂q` (`analytic_state_jacobian`), else the FD
    /// `loaded_state_jacobian` fallback — and `G`'s VELOCITY rows are the full `nv×6`
    /// `rigid_xfrc_column` while its POSITION rows are `Δt·G_vel` (the true semi-implicit
    /// term `∂qpos'/∂w = Δt·G_vel`, since `qpos' = qpos + Δt·qvel'`).
    ///
    /// **The fully-fresh formulation (machine-exact at every n, single-hinge through
    /// multi-link).** The contact plane / COM are posed from a FRESH forward at the
    /// current `qpos` each step (no one-step FK lag — also more physically faithful), the
    /// output is read fresh (forward at `q_N`), and the carry uses the true position-row
    /// term above. This triple is the correct differentiable formulation; the composed
    /// gradient matches the full-coupled FD to ~1e-9 for the single hinge, a free-joint
    /// platen (nv = 6), AND undamped serial-hinge chains (2-link at n = 10, 3-link multi-hop
    /// at n = 2 — `threelink_chain_gradient_matches_fd`). The earlier long-rollout
    /// moment residual (~1e-3 at n = 10) and the 74%-at-n=2 multi-link error were the SAME
    /// defect: the stale-FK pose + §8a position-row drop was a self-consistent pair
    /// calibrated ONLY for nv = 1 (`∂qpos'/∂qvel = Δt·I`, false for a chain). See
    /// `docs/keystone/moment_residual_recon.md` §3f and `docs/keystone/multilink_recon.md`.
    ///
    /// The wrench node and pose seam are analytic (FD-validated machine-exact vs the real
    /// contact readout, `tests/`). `J_state` is the machine-exact ANALYTIC carry for the
    /// single hinge (`analytic_state_jacobian`) and the undamped serial-hinge chain
    /// (`chain_state_jacobian` — the case-split geometric-stiffness Hessian `G`, the
    /// `∂M⁻¹/∂q` directional derivative `dMu`, and the unloaded transition `A` at the CLEAN
    /// `xfrc = 0` operating point; the chain `A` relies on the sim-core Coriolis derivative —
    /// the `∂S/∂q` ancestor term plus the bias-acceleration X_b transport for multi-hop chains).
    /// Free/quaternion joints use the FD `loaded_state_jacobian` (FD-carry).
    /// **Joint damping** is supported: the Euler `eulerdamp` wrench→velocity factor `G_vel`
    /// is `Δt·(M + Δt·D)⁻¹·Jᵀ` ([`rigid_xfrc_column`], `D = implicit_damping`). The damped
    /// single HINGE uses the ANALYTIC `J_state` (the `M → M_impl` correction reconciles the
    /// unloaded `A`'s bare-`M` velocity rows with eulerdamp → machine-exact, ~1e-9); a DAMPED
    /// chain falls back to the FD `loaded_state_jacobian` (damping-correct, FD-carry precision).
    /// FD-gated under damping. Out
    /// of scope: stiffness-implicit / non-Euler integrators
    /// (`ImplicitSpringDamper`'s `M + Δt·D + Δt²·K`, RK4); the coupling's own free-platen
    /// `rigid_damping` knob (asserted 0 here — distinct from model joint damping);
    /// state-dependent actuator forces (the follow-on toward the powered exo); and a flat
    /// constant-normal plane unless `with_rotating_normal`. See
    /// `docs/keystone/damped_joints_recon.md`, `geometric_stiffness_recon.md`, and
    /// `contact_moment_recon.md`.
    ///
    /// # Panics
    /// Panics if `param_idx > 1`, if `rigid_damping != 0`, or if a rigid/soft step
    /// diverges (surfaced loudly as in [`Self::step`]).
    #[must_use]
    // One coherent per-step tape-construction loop (the pose seam + soft + carry
    // nodes + the real coupled step); splitting it would scatter the recurrence.
    // expect_used: a rigid/soft divergence is a programmer error surfaced loudly,
    // not recoverable for keystone-v1 (mirrors `step`'s rationale).
    #[allow(clippy::too_many_lines, clippy::expect_used)]
    pub fn coupled_trajectory_material_gradient_articulated(
        &mut self,
        n_steps: usize,
        param_idx: usize,
    ) -> (f64, f64) {
        // Curvature-correct on a finite sphere (L1b-articulated NORMAL): the contact wrench
        // (`ContactWrenchTrajVjp`) now carries the curved-normal `f_mag·H` term in `∂w/∂x*` and
        // `∂w/∂h` (FD-exact in `sphere_contact_wrench_node_matches_readout_fd`); the soft node +
        // pose seam are SDF-generic. Curvature-correct on a sphere — no plane guard; threads the
        // moving-EE centre channel directly (no `require_no_moving_ee`).
        assert!(
            param_idx <= 1,
            "material param index {param_idx} out of range (0 = μ, 1 = λ)"
        );
        // The loaded carry holds the contact wrench during its FD; a damping force
        // −c·vz would not have its velocity-coupling captured. v1 scope: no damping.
        // EXACT-zero is the intended semantics (not a tolerance): any nonzero damping
        // must fail, and 0.0 is exactly representable / passed as a literal.
        assert!(
            self.rigid_damping == 0.0,
            "articulated path requires rigid_damping = 0 (v1 scope)"
        );
        let n = self.n_vertices;
        let nv = self.model.nv;
        let dt = self.cfg.dt;
        let dir = Vec3::new(0.0, 0.0, 1.0);

        let mut tape = Tape::new();
        let p0 = if param_idx == 0 { self.mu } else { self.lambda };
        let p_var = tape.param_tensor(Tensor::from_slice(&[p0], &[1]));
        let mut x_var = tape.constant_tensor(Tensor::from_slice(&self.x, &[3 * n]));
        let mut v_var = tape.constant_tensor(Tensor::from_slice(&self.v, &[3 * n]));
        // Rigid state s = [qpos(nv); qvel(nv)] (raw — hinge scope, no quaternion).
        let mut s0 = vec![0.0_f64; 2 * nv];
        for i in 0..nv {
            s0[i] = self.data.qpos[i];
            s0[nv + i] = self.data.qvel[i];
        }
        let mut s_var = tape.constant_tensor(Tensor::from_slice(&s0, &[2 * nv]));

        for _ in 0..n_steps {
            // FRESH FK (the matched fresh formulation): re-forward so `height`, `jz`,
            // the moment COM `c`/`jlin`, the wrench response `G_vel`, and `J_state` all
            // read the CURRENT config q_k (sim-core's `step` leaves `xipos`/`qM` lagging
            // `qpos` one step). Paired with the fresh output read + the true position-row
            // carry (`∂qpos'/∂w = Δt·G_vel`, below), this is machine-exact for single-hinge
            // AND multi-link. (The earlier stale-FK + §8a-position-drop convention was a
            // self-consistent pair that calibrated ONLY for nv=1 — Coriolis/`∂qacc/∂qvel`
            // coupling at nv>1 broke it; see `docs/keystone/moment_residual_recon.md`.)
            self.data.forward(&self.model).expect("fresh FK");
            // MOVING END-EFFECTOR: pose the contact sphere at the contact geom (the arm
            // tip) — the centre rides `geom_xpos(q)`, so the pose channel is the 3-vector
            // centre (`PoseCentreVjp`), the lateral generalization of the scalar height.
            // Refreshed each step from the fresh-FK geom pose (mirrors `step_articulated`).
            // Requires the SPHERE collider: `with_contact_geom` is a no-op for the plane
            // (`build_contact` ignores the override for a plane → forward poses at `xipos`),
            // so a plane must take the scalar-height channel, NOT the geom-Jacobian centre
            // channel, or the adjoint would route `∂h/∂q` through `geom_xpos` ≠ `xipos`.
            let moving_ee =
                self.contact_geom.is_some() && matches!(self.collider, Collider::Sphere { .. });
            if moving_ee {
                let g = self.contact_geom.expect("moving_ee ⇒ contact geom set");
                self.sphere_center_override = Some(self.data.geom_xpos[g]);
            }
            let height = self.tip_plane_height();
            // Pose seam: the contact primitive's pose from the rigid state. FLAT — the
            // scalar tip height `h` (`∂h/∂q = J_z`). MOVING-EE — the 3-vector sphere
            // centre `c = geom_xpos(q)` (`∂c/∂q = J_geom`, the `PoseCentreVjp`). ROTATING —
            // the 6-DOF spatial twist `T` of the body-attached plane (`∂T/∂qpos =
            // J_spatial`, the `PoseTwistSeamVjp`); its value is the all-zero perturbation
            // at the linearization point (only the cotangent is threaded). The soft node
            // AND the wrench node share this one pose node (as the flat path shares `h`).
            let rotating = self.rotating_normal;
            // DELIBERATE boundary, not an unfinished follow-on: `rotating_normal` tilts the PLANE
            // with the body, but `moving_ee` requires the SPHERE collider, whose contact normal is
            // set by the geometry (centre + vertex) and is orientation-INVARIANT — `rotating_normal`
            // is a no-op on a sphere (`build_contact`'s sphere branch ignores it). So the combo is
            // meaningless; it would only matter for a non-spherical oriented end-effector (capsule/
            // ellipsoid), which does not exist. Reject it loudly rather than silently no-op.
            assert!(
                !(rotating && moving_ee),
                "rotating-normal + moving-end-effector is unsupported: rotating-normal tilts the \
                 plane, but a moving EE is a sphere whose normal is orientation-invariant (the \
                 rotating-normal would be a silent no-op). Use one or the other."
            );
            // The moving-EE centre channel's three translation axes (`x̂, ŷ, ẑ`); `ẑ`
            // reproduces the scalar height channel.
            let centre_basis = [
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ];
            let pose_var = if rotating {
                tape.push_custom(
                    &[s_var],
                    Tensor::zeros(&[6]),
                    Box::new(PoseTwistSeamVjp {
                        j_spatial: self.pose_twist_jacobian(),
                    }),
                )
            } else if moving_ee {
                let centre = self.sphere_center_override.expect("moving-EE override set");
                tape.push_custom(
                    &[s_var],
                    Tensor::from_slice(centre.as_slice(), &[3]),
                    Box::new(PoseCentreVjp {
                        j_geom: self.pose_centre_jacobian(),
                    }),
                )
            } else {
                tape.push_custom(
                    &[s_var],
                    Tensor::from_slice(&[height], &[1]),
                    Box::new(PoseSeamVjp {
                        jz: self.pose_seam_jz(),
                    }),
                )
            };

            // (1)+(2) one dynamic soft step from the current (self.x, self.v).
            let (solver, x_next) = self.soft_resolve(height);
            let v_next: Vec<f64> = x_next
                .iter()
                .zip(&self.x)
                .map(|(xf, xo)| (xf - xo) / dt)
                .collect();
            // Soft node x*: parents [x_prev, v_prev, p, pose]. ROTATING feeds the 6
            // canonical spatial-twist basis directions (the `δn̂ = ω×n̂` adjoint);
            // MOVING-EE the 3 translation axes of the centre (reusing the twist node's
            // per-direction pose RHS with pure-translation twists); FLAT the single scalar
            // translation along `dir`.
            let soft_vjp: Box<dyn VjpOp> = if rotating {
                Box::new(solver.trajectory_step_vjp_twist(&x_next, dt, param_idx, &twist_basis()))
            } else if moving_ee {
                let twists: Vec<RigidTwist> = centre_basis
                    .iter()
                    .map(|&d| RigidTwist::translation(d))
                    .collect();
                Box::new(solver.trajectory_step_vjp_twist(&x_next, dt, param_idx, &twists))
            } else {
                Box::new(solver.trajectory_step_vjp(&x_next, dt, param_idx, dir))
            };
            let x_next_var = tape.push_custom(
                &[x_var, v_var, p_var, pose_var],
                Tensor::from_slice(&x_next, &[3 * n]),
                soft_vjp,
            );
            let v_next_var = tape.push_custom(
                &[x_next_var, x_var],
                Tensor::from_slice(&v_next, &[3 * n]),
                Box::new(VelVjp {
                    inv_dt: 1.0 / dt,
                    n_dof: 3 * n,
                }),
            );

            // (3) contact WRENCH [τ; f] at the post-step soft config, plane at
            // `height`, about the body COM c = xipos. The off-COM moment
            // τ = −Σ(rᵢ−c)×gᵢ (gᵢ = force_on_softᵢ) is the contact-moment leaf's
            // addition: the merged path routed only the scalar f_z (pure force at
            // the COM); here the full spatial wrench drives the articulated body.
            let positions: Vec<Vec3> = x_next
                .chunks_exact(3)
                .map(|c| Vec3::new(c[0], c[1], c[2]))
                .collect();
            let c = self.data.xipos[self.body];
            let pairs = self.active_pair_wrench_data(height, &positions);
            let mut wrench = SpatialVector::zeros();
            // ContactWrenchTrajVjp active list (vertex, g, n̂, cᵥ, d = rᵢ−c) +
            // the wrench value, in one pass.
            let mut active: Vec<(usize, Vec3, Vec3, f64, Vec3, Matrix3<f64>)> =
                Vec::with_capacity(pairs.len());
            for (v, g, n, curv, r) in pairs {
                let f = -g;
                wrench[3] += f.x;
                wrench[4] += f.y;
                wrench[5] += f.z;
                let tau = (r - c).cross(&f);
                wrench[0] += tau.x;
                wrench[1] += tau.y;
                wrench[2] += tau.z;
                active.push((v, g, n, curv, r - c, self.collider_hessian(height, r)));
            }
            let force = Vec3::new(wrench[3], wrench[4], wrench[5]);
            let jlin = self.com_linear_jacobian();
            // Wrench node w = [τ; f] with parents [x*, pose, s]: ∂w/∂x* (the moment's
            // explicit-rᵢ + via-gᵢ parts), ∂w/∂pose (FLAT: the force/moment-vs-height
            // S1 factor; ROTATING: the per-twist feedback incl. the normal redirect),
            // ∂w/∂s (the moment's c(q) feedback — independent of the normal channel).
            let wrench_pose = if rotating {
                WrenchPose::Twist {
                    basis: twist_basis().to_vec(),
                    com: c,
                }
            } else if moving_ee {
                WrenchPose::Centre {
                    basis: centre_basis.to_vec(),
                }
            } else {
                WrenchPose::Height
            };
            let w_var = tape.push_custom(
                &[x_next_var, pose_var, s_var],
                Tensor::from_slice(wrench.as_slice(), &[6]),
                Box::new(ContactWrenchTrajVjp {
                    active,
                    force,
                    jlin,
                    n_dof: 3 * n,
                    nv,
                    pose: wrench_pose,
                }),
            );

            // Multi-DOF carry at the CURRENT (fresh-FK) state, with the contact wrench
            // held, BEFORE stepping.
            //   J_state = the FULL loaded single-step transition Jacobian
            //     ∂[qpos';qvel']/∂[qpos;qvel] (both blocks — incl the position-state
            //     coupling Δt·∂qvel'/∂qpos and the applied-force geometric stiffness
            //     ∂(Jᵀw)/∂q that the unloaded `transition_derivatives` drops; zero for
            //     the free body, real for the hinge). Single hinge OR serial-hinge chain
            //     (damped or undamped) ⇒ the machine-exact ANALYTIC carry
            //     (`analytic_state_jacobian`, which dispatches to `chain_state_jacobian` for
            //     the chain); free/quaternion joints and non-Euler integrators ⇒ the FD
            //     `loaded_state_jacobian`.
            //   G_vel = ∂(qvel')/∂w = rigid_xfrc_column (nv×6, the full wrench column).
            //     The full G's POSITION rows are G_pos = D·G_vel (the carry below):
            //     semi-implicit Euler integrates qpos with the UPDATED velocity
            //     (`qpos' = qpos ⊕ exp(Δt·qvel')`), so this step's wrench reaches qpos
            //     through `qvel'`. D = Δt·I for a Euclidean DOF (`G_pos = Δt·G_vel`, the
            //     true term that replaced the §8a ZERO — correct only for nv=1) and
            //     Δt·J_r(Δt·qvel') for a quaternion DOF (the SO(3) right Jacobian).
            let j_state = self
                .analytic_state_jacobian(&wrench)
                .unwrap_or_else(|| self.loaded_state_jacobian(&wrench));
            let g_vel = self.fresh_xfrc_column();

            // (4)+(5) route the wrench and step the real rigid body.
            self.data.xfrc_applied[self.body] = wrench;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in articulated trajectory");

            // Position-row wrench response G_pos = D·G_vel (∂(tangent qpos')/∂w). Euclidean
            // (nq==nv): D = Δt·I, so G_pos = Δt·G_vel — byte-identical to the merged
            // hinge/chain path. Quaternion (nq≠nv): D carries the SO(3) right Jacobian
            // J_r(Δt·qvel') at the POST-step velocity (`self.data.qvel`, the ω the
            // integrator actually integrated). See `docs/keystone/quaternion_joints_recon.md`.
            // Δt is the RIGID integrator's `model.timestep` (matching G_vel + the real step
            // + the quaternion branch's `integrator_pos_jacobian`), NOT the soft `cfg.dt`.
            let g_pos = if self.model.nq == self.model.nv {
                self.model.timestep * &g_vel
            } else {
                self.integrator_pos_jacobian(&self.data.qvel) * &g_vel
            };

            let mut s_next = vec![0.0_f64; 2 * nv];
            for i in 0..nv {
                s_next[i] = self.data.qpos[i];
                s_next[nv + i] = self.data.qvel[i];
            }
            let s_next_var = tape.push_custom(
                &[s_var, w_var],
                Tensor::from_slice(&s_next, &[2 * nv]),
                Box::new(RigidStateCarryVjp {
                    j_state,
                    g_vel,
                    g_pos,
                    g_act: None, // passive/material path: no actuator-control channel
                }),
            );

            // Advance the soft state and the tape handles.
            self.v = v_next;
            self.x = x_next;
            x_var = x_next_var;
            v_var = v_next_var;
            s_var = s_next_var;
        }

        // Objective: the tip world height after the rollout = PoseSeam(s_N). Read it
        // FRESH (forward at q_N) — the matched fresh convention the oracle also uses;
        // the stale read's `s_N` attribution relied on ∂qpos'/∂qvel = Δt·I (true for
        // nv=1, false for a chain). See `docs/keystone/moment_residual_recon.md`.
        self.data.forward(&self.model).expect("fresh FK (output)");
        let tip_z = self.data.xipos[self.body].z;
        let jz_final = self.pose_seam_jz();
        let obj_var = tape.push_custom(
            &[s_var],
            Tensor::from_slice(&[tip_z], &[1]),
            Box::new(PoseSeamVjp { jz: jz_final }),
        );
        tape.backward(obj_var);
        let grad = tape.grad_tensor(p_var).as_slice()[0];
        (tip_z, grad)
    }

    /// **The articulated FRICTION-grip trajectory gradient — full matrix-carry wrench (force +
    /// off-COM moment).** The
    /// articulated successor to the free-platen [`Self::coupled_trajectory_tangential_material_gradient`]:
    /// the rigid body is an ARTICULATED mechanism (hinge/slide chain), so the tangential grip
    /// reaction maps to a generalized joint acceleration through the full matrix carry
    /// `Δt·M_impl⁻¹·Jᵀ` (`RigidStateCarryVjp`) — NOT the free-platen scalar `dt/m` lanes — and
    /// the collider drift is read from the articulated state (`Δ_surf,x = (J_lin·qvel)_x·dt`,
    /// `DriftFromStateVjp`). Rolls forward `n_steps` on one chassis tape, then ONE
    /// `tape.backward` gives `(tip_x_N, ∂tip_x_N/∂p)` — the tip's final world x (the tangential
    /// drag) vs the soft block's Neo-Hookean material (`param_idx`: `0 = μ`, `1 = λ`).
    ///
    /// Per step the tape threads: the pose seam `h = PoseSeam(s)`, the articulated drift
    /// `Δ_surf,x = DriftFromState(s)`, the friction-aware soft solve `x*` (drift + height
    /// parents), the NORMAL contact wrench `w_normal = [τ; f](x*, h, s)` (`ContactWrenchTrajVjp`),
    /// the FRICTION wrench fold `w = w_normal + [τ_fric; f_fric]` (`FrictionWrenchTrajVjp` —
    /// the per-vertex friction force `Σ∇D_v` AND its off-COM moment `Σ(r_v−c)×∇D_v`, with the
    /// per-vertex Jacobians from [`CpuNewtonSolver::friction_force_jacobians`]), and the
    /// multi-DOF carry `s' = J_state·s + G·w`.
    ///
    /// **Scope.** The full grip wrench (force + off-COM moment) is routed — FD-gated against the
    /// full forward [`Self::coupled_trajectory_gripped_articulated`]. EUCLIDEAN joints
    /// (`nq == nv` — hinge/slide chains), flat normal, friction active, `rigid_damping = 0`.
    /// `J_state` is the analytic single-hinge / undamped-chain carry (`analytic_state_jacobian`),
    /// else the FD `loaded_state_jacobian` (damped chain / free / quaternion).
    ///
    /// Curvature-correct on a FINITE sphere collider ([`Self::with_sphere_collider`]): both the
    /// NORMAL wrench (`ContactWrenchTrajVjp`'s `f_mag·H`) and the FRICTION wrench
    /// (`friction_force_jacobians`'s `DN·C`) carry the curved-normal term, so this gradient is
    /// curvature-correct on a centroid sphere (FD-gated end-to-end by
    /// `sphere_articulated_friction_trajectory_gradient.rs`).
    ///
    /// MOVING END-EFFECTOR ([`Self::with_contact_geom`]): when the sphere rides the contact geom,
    /// the pose channel is the 3-vector centre (`PoseCentreVjp` + `WrenchPose::Centre` + the grip
    /// soft node's 3-axis pose + the friction wrench's 3-vector `dforce_dpose`), posed at
    /// `geom_xpos(q)` each step. The single-step lateral channels are machine-exact in
    /// `friction_sphere_tangent.rs`; the composition is gated by
    /// `sphere_moving_ee_friction_trajectory_gradient.rs`. No geom set ⇒ scalar-height channel
    /// (byte-identical).
    ///
    /// # Panics
    /// Panics if `param_idx > 1`, if `nq != nv`, if friction is inactive, if `rigid_damping != 0`,
    /// if the rotating normal is enabled, or if a rigid/soft step diverges.
    #[must_use]
    // One coherent per-step tape-construction loop (pose + drift + soft + normal-wrench +
    // friction-wrench + carry nodes + the real step); splitting it would scatter the recurrence.
    // expect_used: a rigid/soft divergence is a programmer error surfaced loudly, not
    // recoverable for keystone-v1 (mirrors `coupled_trajectory_material_gradient_articulated`).
    #[allow(clippy::too_many_lines, clippy::expect_used)]
    pub fn coupled_trajectory_tangential_material_gradient_articulated(
        &mut self,
        n_steps: usize,
        param_idx: usize,
    ) -> (f64, f64) {
        assert!(
            param_idx <= 1,
            "material param index {param_idx} out of range (0 = μ, 1 = λ)"
        );
        assert!(
            self.model.nq == self.model.nv,
            "articulated friction gradient scope: Euclidean joints (nq == nv — hinge/slide chains)"
        );
        assert!(
            self.cfg.friction_mu > 0.0,
            "articulated friction gradient requires friction active (cfg.friction_mu > 0)"
        );
        assert!(
            self.rigid_damping == 0.0,
            "articulated path requires rigid_damping = 0 (the coupling's free-platen knob)"
        );
        assert!(
            !self.rotating_normal,
            "articulated friction gradient is flat-normal only (rotating normal unsupported)"
        );
        let n = self.n_vertices;
        let nv = self.model.nv;
        let dt = self.cfg.dt;
        let pose_dir = Vec3::new(0.0, 0.0, 1.0);
        let drift_dir = Vec3::new(1.0, 0.0, 0.0);
        // Moving-EE centre channel axes (ẑ reproduces the scalar height channel).
        let centre_basis = [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ];

        let mut tape = Tape::new();
        let p0 = if param_idx == 0 { self.mu } else { self.lambda };
        let p_var = tape.param_tensor(Tensor::from_slice(&[p0], &[1]));
        let mut x_var = tape.constant_tensor(Tensor::from_slice(&self.x, &[3 * n]));
        let mut v_var = tape.constant_tensor(Tensor::from_slice(&self.v, &[3 * n]));
        let mut s0 = vec![0.0_f64; 2 * nv];
        for i in 0..nv {
            s0[i] = self.data.qpos[i];
            s0[nv + i] = self.data.qvel[i];
        }
        let mut s_var = tape.constant_tensor(Tensor::from_slice(&s0, &[2 * nv]));

        for _ in 0..n_steps {
            self.data.forward(&self.model).expect("fresh FK");
            // MOVING END-EFFECTOR: pose the finite sphere at the contact geom (the arm tip), so
            // the contact centre rides geom_xpos(q) (the 3-vector PoseCentreVjp channel). Sphere
            // only; for the plane/centroid it stays the scalar height channel (byte-identical).
            let moving_ee =
                self.contact_geom.is_some() && matches!(self.collider, Collider::Sphere { .. });
            if moving_ee {
                let g = self.contact_geom.expect("moving_ee ⇒ contact geom set");
                self.sphere_center_override = Some(self.data.geom_xpos[g]);
            }
            let height = self.tip_plane_height();
            let jlin = self.com_linear_jacobian();
            let jlin_x: Vec<f64> = (0..nv).map(|j| jlin[(0, j)]).collect();
            // Δ_surf from the articulated state: the full 3-vector drives the soft solve; only
            // the differentiated x-component is threaded (the y/z drift carry no μ-sensitivity
            // in the y-symmetric flat scene, as in the free-platen tangential gradient).
            let v_com = &jlin * &self.data.qvel;
            let drift = Vec3::new(v_com[0], v_com[1], v_com[2]) * dt;
            let x_start = self.x.clone();

            // Pose seam: scalar height (PoseSeamVjp) or, for a moving end-effector, the 3-vector
            // centre c = geom_xpos(q) (PoseCentreVjp). Shared by the soft + both wrench nodes.
            let pose_var = if moving_ee {
                let centre = self.sphere_center_override.expect("moving-EE override set");
                tape.push_custom(
                    &[s_var],
                    Tensor::from_slice(centre.as_slice(), &[3]),
                    Box::new(PoseCentreVjp {
                        j_geom: self.pose_centre_jacobian(),
                    }),
                )
            } else {
                tape.push_custom(
                    &[s_var],
                    Tensor::from_slice(&[height], &[1]),
                    Box::new(PoseSeamVjp {
                        jz: self.pose_seam_jz(),
                    }),
                )
            };
            let drift_var = tape.push_custom(
                &[s_var],
                Tensor::from_slice(&[drift.x], &[1]),
                Box::new(DriftFromStateVjp { dt, jlin_x }),
            );

            // (1) one friction-aware soft step with the moving-collider drift.
            let bc = BoundaryConditions::new(self.pinned.clone(), Vec::new());
            let solver: SoftSolver<C> = CpuNewtonSolver::new(
                Tet4,
                self.fresh_mesh(),
                self.build_contact(height),
                self.cfg,
                bc,
            )
            .with_friction_surface_drift(drift);
            let x_next = solver
                .replay_step(
                    &Tensor::from_slice(&self.x, &[3 * n]),
                    &Tensor::from_slice(&self.v, &[3 * n]),
                    &Tensor::zeros(&[0]),
                    dt,
                )
                .x_final;
            let v_next: Vec<f64> = x_next
                .iter()
                .zip(&self.x)
                .map(|(xf, xo)| (xf - xo) / dt)
                .collect();

            // Soft node x*: parents [x_prev, v_prev, p, pose, Δ_surf]. The pose parent is the
            // scalar height (grip) or the 3-vector centre (grip_centre, moving-EE).
            let soft_grip = if moving_ee {
                solver.trajectory_step_vjp_grip_centre(
                    &x_next,
                    &x_start,
                    dt,
                    param_idx,
                    &centre_basis,
                    drift_dir,
                )
            } else {
                solver
                    .trajectory_step_vjp_grip(&x_next, &x_start, dt, param_idx, pose_dir, drift_dir)
            };
            let x_next_var = tape.push_custom(
                &[x_var, v_var, p_var, pose_var, drift_var],
                Tensor::from_slice(&x_next, &[3 * n]),
                Box::new(soft_grip),
            );
            let v_next_var = tape.push_custom(
                &[x_next_var, x_var],
                Tensor::from_slice(&v_next, &[3 * n]),
                Box::new(VelVjp {
                    inv_dt: 1.0 / dt,
                    n_dof: 3 * n,
                }),
            );

            // (2) NORMAL contact wrench [τ; f] about the fresh-FK COM (friction is folded in
            // separately below — `pair_readout` is the contact-penalty normal force only).
            let positions: Vec<Vec3> = x_next
                .chunks_exact(3)
                .map(|cc| Vec3::new(cc[0], cc[1], cc[2]))
                .collect();
            let c = self.data.xipos[self.body];
            let pairs = self.active_pair_wrench_data(height, &positions);
            let mut w_normal = SpatialVector::zeros();
            let mut active: Vec<(usize, Vec3, Vec3, f64, Vec3, Matrix3<f64>)> =
                Vec::with_capacity(pairs.len());
            for (v, g, nrm, curv, r) in pairs {
                let f = -g;
                w_normal[3] += f.x;
                w_normal[4] += f.y;
                w_normal[5] += f.z;
                let tau = (r - c).cross(&f);
                w_normal[0] += tau.x;
                w_normal[1] += tau.y;
                w_normal[2] += tau.z;
                active.push((v, g, nrm, curv, r - c, self.collider_hessian(height, r)));
            }
            let normal_force = Vec3::new(w_normal[3], w_normal[4], w_normal[5]);
            let normal_pose = if moving_ee {
                WrenchPose::Centre {
                    basis: centre_basis.to_vec(),
                }
            } else {
                WrenchPose::Height
            };
            let w_normal_var = tape.push_custom(
                &[x_next_var, pose_var, s_var],
                Tensor::from_slice(w_normal.as_slice(), &[6]),
                Box::new(ContactWrenchTrajVjp {
                    active,
                    force: normal_force,
                    jlin: jlin.clone(),
                    n_dof: 3 * n,
                    nv,
                    pose: normal_pose,
                }),
            );

            // (3)+(4) FRICTION wrench: per-vertex friction force `∇D_v` (on the rigid body) folded
            // into the normal wrench's force rows PLUS its off-COM moment `Σ(r_v−c)×∇D_v`. The
            // per-vertex Jacobians (frozen-lag slip + λⁿ coupling + drift + height) come from
            // sim-soft's `friction_force_jacobians`; the node threads them through force AND moment.
            let pv = solver.friction_force_jacobians(&x_next, &x_start, dt, drift_dir, pose_dir);
            let mut w_total = w_normal;
            // Moving-EE: the friction wrench's pose parent becomes the 3-vector centre — per-vertex
            // ∂force/∂(centre·e_k) for each axis (reusing the validated `friction_force_jacobians`,
            // same active set/order), transposed [axis][vertex] → [vertex][axis].
            let pose_dforce = moving_ee.then(|| {
                let per_axis: Vec<Vec<Vec3>> = centre_basis
                    .iter()
                    .map(|&e| {
                        solver
                            .friction_force_jacobians(&x_next, &x_start, dt, drift_dir, e)
                            .into_iter()
                            .map(|p| p.dforce_dheight)
                            .collect()
                    })
                    .collect();
                (0..pv.len())
                    .map(|i| per_axis.iter().map(|axis| axis[i]).collect())
                    .collect::<Vec<Vec<Vec3>>>()
            });
            let n_pose = if moving_ee { 3 } else { 1 };
            let (fverts, f_fric_total) =
                assemble_friction_wrench(pv, &positions, c, &mut w_total, pose_dforce);
            let w_total_var = tape.push_custom(
                &[w_normal_var, x_next_var, pose_var, s_var, drift_var, x_var],
                Tensor::from_slice(w_total.as_slice(), &[6]),
                Box::new(FrictionWrenchTrajVjp {
                    verts: fverts,
                    f_total: f_fric_total,
                    jlin,
                    n_dof: 3 * n,
                    nv,
                    n_pose,
                    mu_c: false,
                }),
            );

            // (5) multi-DOF carry at the current (fresh-FK) state, with the TOTAL wrench held.
            let j_state = self
                .analytic_state_jacobian(&w_total)
                .unwrap_or_else(|| self.loaded_state_jacobian(&w_total));
            let g_vel = self.fresh_xfrc_column();

            self.data.xfrc_applied[self.body] = w_total;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in articulated friction trajectory");

            let g_pos = self.model.timestep * &g_vel; // Euclidean (nq == nv): D = Δt·I.
            let mut s_next = vec![0.0_f64; 2 * nv];
            for i in 0..nv {
                s_next[i] = self.data.qpos[i];
                s_next[nv + i] = self.data.qvel[i];
            }
            let s_next_var = tape.push_custom(
                &[s_var, w_total_var],
                Tensor::from_slice(&s_next, &[2 * nv]),
                Box::new(RigidStateCarryVjp {
                    j_state,
                    g_vel,
                    g_pos,
                    g_act: None,
                }),
            );

            self.v = v_next;
            self.x = x_next;
            x_var = x_next_var;
            v_var = v_next_var;
            s_var = s_next_var;
        }

        // Objective: the tip world x after the rollout = (COM x-Jacobian)·δq — the tangential
        // drag. Read fresh (forward at q_N), the matched fresh convention the oracle uses.
        self.data.forward(&self.model).expect("fresh FK (output)");
        let tip_x = self.data.xipos[self.body].x;
        let jx_final: Vec<f64> = {
            let jlin = self.com_linear_jacobian();
            (0..nv).map(|j| jlin[(0, j)]).collect()
        };
        let obj_var = tape.push_custom(
            &[s_var],
            Tensor::from_slice(&[tip_x], &[1]),
            Box::new(PoseSeamVjp { jz: jx_final }),
        );
        tape.backward(obj_var);
        let grad = tape.grad_tensor(p_var).as_slice()[0];
        (tip_x, grad)
    }

    /// **The articulated FRICTION-grip trajectory gradient w.r.t. the Coulomb COEFFICIENT `μ_c`.**
    /// The friction-coefficient sibling of
    /// [`Self::coupled_trajectory_tangential_material_gradient_articulated`] (which differentiates
    /// the soft block's `μ`/`λ`) and the articulated successor to the free-platen
    /// [`Self::coupled_trajectory_tangential_friction_coeff_gradient`]: rolls the grip system
    /// forward `n_steps` on one chassis tape, then one `tape.backward` gives `(tip_x_N, ∂tip_x_N/∂μ_c)`
    /// — the tip's tangential drag vs the contact's grip-surface friction coefficient.
    ///
    /// `μ_c` enters through the SAME two channels as the free platen, now on the matrix carry:
    /// 1. **Soft `x*`** — the friction potential `D ∝ μ_c` shifts the soft equilibrium; routed by
    ///    [`CpuNewtonSolver::trajectory_step_vjp_grip_fric_coeff`] into the soft node's param slot.
    ///    TINY in deep slip (`x*` barely moves).
    /// 2. **Direct through the friction force** — `∇D_v = μ_c·λⁿ_v·…` is LINEAR in `μ_c`, so
    ///    `∂∇D_v/∂μ_c = ∇D_v/μ_c` ([`sim_soft::FrictionVertexForce::dforce_dmu_c`]); the
    ///    `FrictionWrenchTrajVjp` node threads it through BOTH force and off-COM moment via the same
    ///    per-vertex `co_v` as the material path. This DOMINATES (`≈ λⁿ` in the saturated regime).
    ///
    /// Every other node (pose seam, articulated drift, normal wrench, matrix carry, tip-x objective)
    /// is identical to the material gradient. Same scope: EUCLIDEAN joints (`nq == nv`), flat normal,
    /// friction active, `rigid_damping = 0`. FD-gated against [`Self::coupled_trajectory_gripped_articulated`].
    /// Curvature-correct on a FINITE sphere collider (same curved-normal carry as the material
    /// sibling — normal `f_mag·H` + friction `DN·C`), so curvature-correct on a centroid sphere. Like
    /// the material sibling it also threads the MOVING-END-EFFECTOR 3-vector centre channel under
    /// [`Self::with_contact_geom`] (the same `grip_centre` soft node + `WrenchPose::Centre` + 3-vector
    /// friction `dforce_dpose`); gated by `sphere_moving_ee_friction_trajectory_gradient.rs`.
    ///
    /// # Panics
    /// Panics if `nq != nv`, if friction is inactive, if `rigid_damping != 0`, if the rotating
    /// normal is enabled, or if a rigid/soft step diverges.
    #[must_use]
    // One coherent per-step tape-construction loop (mirrors the material gradient; μ_c swaps the
    // param leaf, the soft VJP, and adds the friction node's μ_c parent).
    // expect_used: a rigid/soft divergence is a programmer error surfaced loudly, not recoverable.
    #[allow(clippy::too_many_lines, clippy::expect_used)]
    pub fn coupled_trajectory_tangential_friction_coeff_gradient_articulated(
        &mut self,
        n_steps: usize,
    ) -> (f64, f64) {
        assert!(
            self.model.nq == self.model.nv,
            "articulated friction gradient scope: Euclidean joints (nq == nv — hinge/slide chains)"
        );
        assert!(
            self.cfg.friction_mu > 0.0,
            "∂/∂μ_c requires friction active (cfg.friction_mu > 0)"
        );
        assert!(
            self.rigid_damping == 0.0,
            "articulated path requires rigid_damping = 0 (the coupling's free-platen knob)"
        );
        assert!(
            !self.rotating_normal,
            "articulated friction gradient is flat-normal only (rotating normal unsupported)"
        );
        let n = self.n_vertices;
        let nv = self.model.nv;
        let dt = self.cfg.dt;
        let pose_dir = Vec3::new(0.0, 0.0, 1.0);
        let drift_dir = Vec3::new(1.0, 0.0, 0.0);
        // Moving-EE centre channel axes (ẑ reproduces the scalar height channel).
        let centre_basis = [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ];

        let mut tape = Tape::new();
        let p_var = tape.param_tensor(Tensor::from_slice(&[self.cfg.friction_mu], &[1]));
        let mut x_var = tape.constant_tensor(Tensor::from_slice(&self.x, &[3 * n]));
        let mut v_var = tape.constant_tensor(Tensor::from_slice(&self.v, &[3 * n]));
        let mut s0 = vec![0.0_f64; 2 * nv];
        for i in 0..nv {
            s0[i] = self.data.qpos[i];
            s0[nv + i] = self.data.qvel[i];
        }
        let mut s_var = tape.constant_tensor(Tensor::from_slice(&s0, &[2 * nv]));

        for _ in 0..n_steps {
            self.data.forward(&self.model).expect("fresh FK");
            // MOVING END-EFFECTOR: pose the finite sphere at the contact geom (see the material
            // sibling). Sphere only; plane/centroid stays the scalar height channel.
            let moving_ee =
                self.contact_geom.is_some() && matches!(self.collider, Collider::Sphere { .. });
            if moving_ee {
                let g = self.contact_geom.expect("moving_ee ⇒ contact geom set");
                self.sphere_center_override = Some(self.data.geom_xpos[g]);
            }
            let height = self.tip_plane_height();
            let jlin = self.com_linear_jacobian();
            let jlin_x: Vec<f64> = (0..nv).map(|j| jlin[(0, j)]).collect();
            let v_com = &jlin * &self.data.qvel;
            let drift = Vec3::new(v_com[0], v_com[1], v_com[2]) * dt;
            let x_start = self.x.clone();

            let pose_var = if moving_ee {
                let centre = self.sphere_center_override.expect("moving-EE override set");
                tape.push_custom(
                    &[s_var],
                    Tensor::from_slice(centre.as_slice(), &[3]),
                    Box::new(PoseCentreVjp {
                        j_geom: self.pose_centre_jacobian(),
                    }),
                )
            } else {
                tape.push_custom(
                    &[s_var],
                    Tensor::from_slice(&[height], &[1]),
                    Box::new(PoseSeamVjp {
                        jz: self.pose_seam_jz(),
                    }),
                )
            };
            let drift_var = tape.push_custom(
                &[s_var],
                Tensor::from_slice(&[drift.x], &[1]),
                Box::new(DriftFromStateVjp { dt, jlin_x }),
            );

            // (1) one friction-aware soft step; the soft node's param slot is μ_c (channel 1).
            let bc = BoundaryConditions::new(self.pinned.clone(), Vec::new());
            let solver: SoftSolver<C> = CpuNewtonSolver::new(
                Tet4,
                self.fresh_mesh(),
                self.build_contact(height),
                self.cfg,
                bc,
            )
            .with_friction_surface_drift(drift);
            let x_next = solver
                .replay_step(
                    &Tensor::from_slice(&self.x, &[3 * n]),
                    &Tensor::from_slice(&self.v, &[3 * n]),
                    &Tensor::zeros(&[0]),
                    dt,
                )
                .x_final;
            let v_next: Vec<f64> = x_next
                .iter()
                .zip(&self.x)
                .map(|(xf, xo)| (xf - xo) / dt)
                .collect();

            let soft_grip = if moving_ee {
                solver.trajectory_step_vjp_grip_fric_coeff_centre(
                    &x_next,
                    &x_start,
                    dt,
                    &centre_basis,
                    drift_dir,
                )
            } else {
                solver
                    .trajectory_step_vjp_grip_fric_coeff(&x_next, &x_start, dt, pose_dir, drift_dir)
            };
            let x_next_var = tape.push_custom(
                &[x_var, v_var, p_var, pose_var, drift_var],
                Tensor::from_slice(&x_next, &[3 * n]),
                Box::new(soft_grip),
            );
            let v_next_var = tape.push_custom(
                &[x_next_var, x_var],
                Tensor::from_slice(&v_next, &[3 * n]),
                Box::new(VelVjp {
                    inv_dt: 1.0 / dt,
                    n_dof: 3 * n,
                }),
            );

            // (2) NORMAL contact wrench — μ_c does NOT enter the normal force, so no μ_c parent.
            let positions: Vec<Vec3> = x_next
                .chunks_exact(3)
                .map(|cc| Vec3::new(cc[0], cc[1], cc[2]))
                .collect();
            let c = self.data.xipos[self.body];
            let pairs = self.active_pair_wrench_data(height, &positions);
            let mut w_normal = SpatialVector::zeros();
            let mut active: Vec<(usize, Vec3, Vec3, f64, Vec3, Matrix3<f64>)> =
                Vec::with_capacity(pairs.len());
            for (v, g, nrm, curv, r) in pairs {
                let f = -g;
                w_normal[3] += f.x;
                w_normal[4] += f.y;
                w_normal[5] += f.z;
                let tau = (r - c).cross(&f);
                w_normal[0] += tau.x;
                w_normal[1] += tau.y;
                w_normal[2] += tau.z;
                active.push((v, g, nrm, curv, r - c, self.collider_hessian(height, r)));
            }
            let normal_force = Vec3::new(w_normal[3], w_normal[4], w_normal[5]);
            let normal_pose = if moving_ee {
                WrenchPose::Centre {
                    basis: centre_basis.to_vec(),
                }
            } else {
                WrenchPose::Height
            };
            let w_normal_var = tape.push_custom(
                &[x_next_var, pose_var, s_var],
                Tensor::from_slice(w_normal.as_slice(), &[6]),
                Box::new(ContactWrenchTrajVjp {
                    active,
                    force: normal_force,
                    jlin: jlin.clone(),
                    n_dof: 3 * n,
                    nv,
                    pose: normal_pose,
                }),
            );

            // (3)+(4) FRICTION wrench WITH the μ_c direct channel (7th parent = μ_c). Moving-EE:
            // the pose parent is the 3-vector centre — per-vertex ∂force/∂(centre·e_k) per axis.
            let pv = solver.friction_force_jacobians(&x_next, &x_start, dt, drift_dir, pose_dir);
            let mut w_total = w_normal;
            let pose_dforce = moving_ee.then(|| {
                let per_axis: Vec<Vec<Vec3>> = centre_basis
                    .iter()
                    .map(|&e| {
                        solver
                            .friction_force_jacobians(&x_next, &x_start, dt, drift_dir, e)
                            .into_iter()
                            .map(|p| p.dforce_dheight)
                            .collect()
                    })
                    .collect();
                (0..pv.len())
                    .map(|i| per_axis.iter().map(|axis| axis[i]).collect())
                    .collect::<Vec<Vec<Vec3>>>()
            });
            let n_pose = if moving_ee { 3 } else { 1 };
            let (fverts, f_fric_total) =
                assemble_friction_wrench(pv, &positions, c, &mut w_total, pose_dforce);
            let w_total_var = tape.push_custom(
                &[
                    w_normal_var,
                    x_next_var,
                    pose_var,
                    s_var,
                    drift_var,
                    x_var,
                    p_var,
                ],
                Tensor::from_slice(w_total.as_slice(), &[6]),
                Box::new(FrictionWrenchTrajVjp {
                    verts: fverts,
                    f_total: f_fric_total,
                    jlin,
                    n_dof: 3 * n,
                    nv,
                    n_pose,
                    mu_c: true,
                }),
            );

            // (5) multi-DOF carry with the TOTAL wrench held.
            let j_state = self
                .analytic_state_jacobian(&w_total)
                .unwrap_or_else(|| self.loaded_state_jacobian(&w_total));
            let g_vel = self.fresh_xfrc_column();

            self.data.xfrc_applied[self.body] = w_total;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in articulated friction-coeff trajectory");

            let g_pos = self.model.timestep * &g_vel;
            let mut s_next = vec![0.0_f64; 2 * nv];
            for i in 0..nv {
                s_next[i] = self.data.qpos[i];
                s_next[nv + i] = self.data.qvel[i];
            }
            let s_next_var = tape.push_custom(
                &[s_var, w_total_var],
                Tensor::from_slice(&s_next, &[2 * nv]),
                Box::new(RigidStateCarryVjp {
                    j_state,
                    g_vel,
                    g_pos,
                    g_act: None,
                }),
            );

            self.v = v_next;
            self.x = x_next;
            x_var = x_next_var;
            v_var = v_next_var;
            s_var = s_next_var;
        }

        self.data.forward(&self.model).expect("fresh FK (output)");
        let tip_x = self.data.xipos[self.body].x;
        let jx_final: Vec<f64> = {
            let jlin = self.com_linear_jacobian();
            (0..nv).map(|j| jlin[(0, j)]).collect()
        };
        let obj_var = tape.push_custom(
            &[s_var],
            Tensor::from_slice(&[tip_x], &[1]),
            Box::new(PoseSeamVjp { jz: jx_final }),
        );
        tape.backward(obj_var);
        let grad = tape.grad_tensor(p_var).as_slice()[0];
        (tip_x, grad)
    }

    /// Forward-only oracle for [`Self::coupled_trajectory_actuator_gradient`]: roll the
    /// articulated coupled system forward applying the per-step motor control
    /// `controls[k]` to the single actuator (`data.ctrl[0]`), and return the tip world
    /// height after the rollout. No tape — the independent black-box oracle for
    /// finite-differencing the actuator-control gradient. Advances `self`.
    ///
    /// # Panics
    /// Panics if a rigid/soft step diverges (as in [`Self::step`]).
    #[must_use]
    // expect_used: a rigid/soft divergence is a programmer error surfaced loudly, not
    // recoverable for keystone-v1 (mirrors `step` / `coupled_trajectory_articulated_z`).
    #[allow(clippy::expect_used)]
    pub fn coupled_trajectory_actuated_z(&mut self, controls: &[f64]) -> f64 {
        for &u_k in controls {
            self.data.forward(&self.model).expect("fresh FK");
            // MOVING END-EFFECTOR: pose the sphere at the contact geom each step (the FD oracle
            // for the moving-EE actuator gradient — forward + adjoint share posing). Sphere-only;
            // no-op for the plane / no geom (byte-identical centroid default).
            if let (Some(g), Collider::Sphere { .. }) = (self.contact_geom, self.collider) {
                self.sphere_center_override = Some(self.data.geom_xpos[g]);
            }
            let height = self.tip_plane_height();
            let (_solver, x_next) = self.soft_resolve(height);
            for (vi, (&xf, &xo)) in self.v.iter_mut().zip(x_next.iter().zip(self.x.iter())) {
                *vi = (xf - xo) / self.cfg.dt;
            }
            self.x = x_next;
            // Route the contact wrench AND the motor control, then step (the actuator
            // drives the integration, not the pre-step contact, so set ctrl after the
            // wrench readout — mirrors the gradient method's staggered order).
            self.data.xfrc_applied[self.body] = self.contact_wrench(height);
            self.data.ctrl[0] = u_k;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in actuated rollout");
        }
        self.data.forward(&self.model).expect("fresh FK (output)");
        self.data.xipos[self.body].z
    }

    /// Forward-only oracle for the actuator CONTROL gradient THROUGH the friction grip — the
    /// gripped sibling of [`Self::coupled_trajectory_actuated_z`] (normal-only) and the actuated
    /// sibling of [`Self::coupled_trajectory_gripped_articulated`]. Rolls the coupled system
    /// forward applying the per-step motor control `controls[k]` (`data.ctrl[0]`) with the full
    /// GRIPPED contact wrench (normal + friction + off-COM moment) and the collider drift read
    /// from the articulated state (`Δ_surf = J_lin·qvel·dt`). Returns the body COM (`xipos`) so a
    /// gate can read the tangential drag (`.x`); the independent black-box reference for
    /// finite-differencing `∂tip_x/∂u_k`. Advances `self`.
    ///
    /// Mirrors the gradient method's staggered per-step order — soft solve → wrench readout → set
    /// `ctrl` → step (the actuator drives the integration, not the pre-step contact).
    ///
    /// # Panics
    /// Panics if a rigid/soft step diverges (as in [`Self::step`]).
    #[must_use]
    // expect_used: a rigid/soft divergence is a programmer error surfaced loudly, not recoverable
    // for keystone-v1 (mirrors `coupled_trajectory_actuated_z` / `coupled_trajectory_gripped_articulated`).
    #[allow(clippy::expect_used)]
    pub fn coupled_trajectory_actuated_gripped_x(&mut self, controls: &[f64]) -> Vec3 {
        let dt = self.cfg.dt;
        let n = self.n_vertices;
        for &u_k in controls {
            self.data.forward(&self.model).expect("fresh FK");
            // MOVING END-EFFECTOR: pose the sphere at the contact geom (FD oracle for the moving-EE
            // actuator-friction gradient). Sphere-only; no-op for the plane / no geom.
            if let (Some(g), Collider::Sphere { .. }) = (self.contact_geom, self.collider) {
                self.sphere_center_override = Some(self.data.geom_xpos[g]);
            }
            let height = self.tip_plane_height();
            let x_start = self.x.clone();
            let v_com = &self.com_linear_jacobian() * &self.data.qvel;
            let drift = Vec3::new(v_com[0], v_com[1], v_com[2]) * dt;
            let bc = BoundaryConditions::new(self.pinned.clone(), Vec::new());
            let solver: SoftSolver<C> = CpuNewtonSolver::new(
                Tet4,
                self.fresh_mesh(),
                self.build_contact(height),
                self.cfg,
                bc,
            )
            .with_friction_surface_drift(drift);
            let x_next = solver
                .replay_step(
                    &Tensor::from_slice(&self.x, &[3 * n]),
                    &Tensor::from_slice(&self.v, &[3 * n]),
                    &Tensor::zeros(&[0]),
                    dt,
                )
                .x_final;
            let friction = solver.friction_forces_on_soft(&x_next, &x_start, dt);
            for (vi, (&xf, &xo)) in self.v.iter_mut().zip(x_next.iter().zip(self.x.iter())) {
                *vi = (xf - xo) / dt;
            }
            self.x = x_next;
            // Route the gripped reaction wrench AND the motor control, then step (ctrl set after
            // the wrench readout — the staggered order the gradient method also uses).
            self.data.xfrc_applied[self.body] = self.contact_wrench_gripped(height, &friction);
            self.data.ctrl[0] = u_k;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in actuated grip rollout");
        }
        self.data.forward(&self.model).expect("fresh FK (output)");
        self.data.xipos[self.body]
    }

    /// **The articulated actuator CONTROL gradient — the powered-exo substrate.** Rolls
    /// the coupled system forward applying a per-step MuJoCo `<actuator>` motor control
    /// (`controls[k]` at step k, on `data.ctrl[0]`) on ONE chassis tape, then ONE
    /// `tape.backward(tip_z_N)` gives `(tip_z_N, [∂tip_z_N/∂u_0 … ∂u_{N−1}])` — the tip's
    /// final height vs every actuator control. The articulated + real-`<actuator>`
    /// successor to [`Self::coupled_trajectory_control_gradient`] (which applies a Cartesian
    /// force on the free platen): here the control routes through the actuator transmission
    /// (`qfrc_actuator = moment·gain·ctrl`) onto the joint, and the multi-DOF carry gains
    /// the actuator-input channel — `s' = J_state·s + G·w + G_act·u`,
    /// `G_act = ∂s'/∂ctrl = [Δt·G_act_vel; G_act_vel]` with
    /// `G_act_vel = Δt·M_impl⁻¹·∂qfrc_actuator/∂ctrl` (`Self::actuator_velocity_column`).
    ///
    /// Each `u_k` is a tape parameter feeding the carry's third (control) parent, so the
    /// reverse pass accumulates BOTH paths each control has on `tip_z_N`: the DIRECT joint
    /// drive (`u_k → qvel' → tip_z`) AND the INDIRECT coupled path (`u_k` moves the arm →
    /// changes the contact penetration → soft re-equilibration → reaction). The material
    /// `μ` rides as a constant leaf (a joint actuator+design gradient on one tape is a
    /// follow-on).
    ///
    /// **Scope.** A single AFFINE actuator (`force = gain·ctrl + bias`) on a EUCLIDEAN
    /// mechanism (`nq == nv` — a hinge OR a hinge/slide CHAIN; no quaternion ball/free),
    /// flat normal — a MOTOR or a state-feedback SERVO (position/velocity/PD). The control
    /// channel `∂qfrc/∂ctrl = gain` is the carry's `G_act`. The actuator's state-feedback
    /// `∂qfrc/∂(qpos,qvel)` enters `J_state`: on a CHAIN the (even constant) actuator force
    /// also interacts with `∂M⁻¹/∂q`, so the FD `loaded_state_jacobian` replicates `ctrl`
    /// in its scratch steps (set above, before the carry) — `coupled_trajectory_actuated_z`
    /// drops a `ctrl`-blind scratch from ≈5e-5 to ~1e-8. (Single hinge: the analytic
    /// `J_state` already captures a servo's `ctrl`-independent slope and a motor leaves it
    /// unchanged — machine-exact. An UNDAMPED chain likewise has an analytic `J_state`
    /// (`chain_state_jacobian`, which copies `ctrl` into its clean scratch so the
    /// servo state-feedback enters `A`); a DAMPED chain — as in these actuator tests — keeps
    /// the FD `loaded_state_jacobian` ⇒ FD-carry precision.)
    /// Joint damping IS supported (via `M_impl`). Follow-ons: muscles (`act`-state,
    /// nonlinear gain), quaternion (ball/free) joints, the analytic DAMPED chain carry
    /// (machine-exact `nv > 1` under damping), and the actuator+design gradient on one tape. See
    /// `docs/keystone/actuator_dynamics_recon.md`.
    ///
    /// # Panics
    /// Panics if the scene is not a single hinge with exactly one actuator, if
    /// `rigid_damping ≠ 0`, if the rotating normal is enabled, or if a rigid/soft step
    /// diverges.
    #[must_use]
    // One coherent per-step tape-construction loop (the pose seam + soft + wrench + carry
    // nodes + the real actuated step); splitting it would scatter the recurrence.
    // expect_used: a rigid/soft divergence is a programmer error surfaced loudly, not
    // recoverable for keystone-v1 (mirrors `coupled_trajectory_material_gradient_articulated`).
    #[allow(clippy::too_many_lines, clippy::expect_used)]
    pub fn coupled_trajectory_actuator_gradient(&mut self, controls: &[f64]) -> (f64, Vec<f64>) {
        assert!(
            self.model.nq == self.model.nv,
            "actuator gradient scope: Euclidean joints (nq == nv — hinge/slide chains; \
             no quaternion ball/free)"
        );
        assert!(
            self.model.nu == 1,
            "actuator gradient scope: exactly one actuator"
        );
        assert!(
            self.rigid_damping == 0.0,
            "articulated path requires rigid_damping = 0 (the coupling's free-platen knob)"
        );
        assert!(
            !self.rotating_normal,
            "actuator gradient v1 scope: flat normal"
        );
        let n = self.n_vertices;
        let nv = self.model.nv;
        let nu = self.model.nu;
        let dt = self.cfg.dt;
        let dir = Vec3::new(0.0, 0.0, 1.0);
        // Moving-EE centre channel axes (ẑ reproduces the scalar height channel).
        let centre_basis = [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ];

        let mut tape = Tape::new();
        // Material μ is a CONSTANT leaf here (not differentiated — only the control
        // leaves are read), so its unread gradient stays valid.
        let p_var = tape.constant_tensor(Tensor::from_slice(&[self.mu], &[1]));
        // One control parameter leaf per step (the gradient targets).
        let control_vars: Vec<Var> = controls
            .iter()
            .map(|&u| tape.param_tensor(Tensor::from_slice(&[u], &[1])))
            .collect();
        let mut x_var = tape.constant_tensor(Tensor::from_slice(&self.x, &[3 * n]));
        let mut v_var = tape.constant_tensor(Tensor::from_slice(&self.v, &[3 * n]));
        let mut s0 = vec![0.0_f64; 2 * nv];
        for i in 0..nv {
            s0[i] = self.data.qpos[i];
            s0[nv + i] = self.data.qvel[i];
        }
        let mut s_var = tape.constant_tensor(Tensor::from_slice(&s0, &[2 * nv]));

        for (k, &u_k) in controls.iter().enumerate() {
            // FRESH FK (the matched fresh formulation, as in the material gradient).
            self.data.forward(&self.model).expect("fresh FK");
            // MOVING END-EFFECTOR: pose the sphere at the contact geom (the 3-vector centre
            // channel, mirroring the material-normal gradient #428). Sphere-only; plane/no-geom
            // stays the scalar height channel (byte-identical). The `g_act` channel is unchanged.
            let moving_ee =
                self.contact_geom.is_some() && matches!(self.collider, Collider::Sphere { .. });
            if moving_ee {
                let g = self.contact_geom.expect("moving_ee ⇒ contact geom set");
                self.sphere_center_override = Some(self.data.geom_xpos[g]);
            }
            let height = self.tip_plane_height();
            let h_var = if moving_ee {
                let centre = self.sphere_center_override.expect("moving-EE override set");
                tape.push_custom(
                    &[s_var],
                    Tensor::from_slice(centre.as_slice(), &[3]),
                    Box::new(PoseCentreVjp {
                        j_geom: self.pose_centre_jacobian(),
                    }),
                )
            } else {
                tape.push_custom(
                    &[s_var],
                    Tensor::from_slice(&[height], &[1]),
                    Box::new(PoseSeamVjp {
                        jz: self.pose_seam_jz(),
                    }),
                )
            };

            // (1)+(2) one dynamic soft step (μ a constant leaf; param_idx 0 unread).
            let (solver, x_next) = self.soft_resolve(height);
            let v_next: Vec<f64> = x_next
                .iter()
                .zip(&self.x)
                .map(|(xf, xo)| (xf - xo) / dt)
                .collect();
            let soft_vjp: Box<dyn VjpOp> = if moving_ee {
                let twists: Vec<RigidTwist> = centre_basis
                    .iter()
                    .map(|&d| RigidTwist::translation(d))
                    .collect();
                Box::new(solver.trajectory_step_vjp_twist(&x_next, dt, 0, &twists))
            } else {
                Box::new(solver.trajectory_step_vjp(&x_next, dt, 0, dir))
            };
            let x_next_var = tape.push_custom(
                &[x_var, v_var, p_var, h_var],
                Tensor::from_slice(&x_next, &[3 * n]),
                soft_vjp,
            );
            let v_next_var = tape.push_custom(
                &[x_next_var, x_var],
                Tensor::from_slice(&v_next, &[3 * n]),
                Box::new(VelVjp {
                    inv_dt: 1.0 / dt,
                    n_dof: 3 * n,
                }),
            );

            // (3) contact wrench [τ; f] at the post-step soft config (curvature-correct on a
            // sphere — `active` carries `collider_hessian`; centroid posing).
            let positions: Vec<Vec3> = x_next
                .chunks_exact(3)
                .map(|c| Vec3::new(c[0], c[1], c[2]))
                .collect();
            let c = self.data.xipos[self.body];
            let pairs = self.active_pair_wrench_data(height, &positions);
            let mut wrench = SpatialVector::zeros();
            let mut active: Vec<(usize, Vec3, Vec3, f64, Vec3, Matrix3<f64>)> =
                Vec::with_capacity(pairs.len());
            for (v, g, nrm, curv, r) in pairs {
                let f = -g;
                wrench[3] += f.x;
                wrench[4] += f.y;
                wrench[5] += f.z;
                let tau = (r - c).cross(&f);
                wrench[0] += tau.x;
                wrench[1] += tau.y;
                wrench[2] += tau.z;
                active.push((v, g, nrm, curv, r - c, self.collider_hessian(height, r)));
            }
            let force = Vec3::new(wrench[3], wrench[4], wrench[5]);
            let jlin = self.com_linear_jacobian();
            let wrench_pose = if moving_ee {
                WrenchPose::Centre {
                    basis: centre_basis.to_vec(),
                }
            } else {
                WrenchPose::Height
            };
            let w_var = tape.push_custom(
                &[x_next_var, h_var, s_var],
                Tensor::from_slice(wrench.as_slice(), &[6]),
                Box::new(ContactWrenchTrajVjp {
                    active,
                    force,
                    jlin,
                    n_dof: 3 * n,
                    nv,
                    pose: wrench_pose,
                }),
            );

            // Set the control BEFORE the carry: the FD `loaded_state_jacobian` replicates
            // `self.data.ctrl` in its scratch steps so the actuator force is present in
            // `J_state` (on a CHAIN the force interacts with `∂M⁻¹/∂q`). For the single
            // hinge the constant force leaves `J_state` unchanged, so this ordering is
            // byte-identical there.
            self.data.ctrl[0] = u_k;

            // Carry at the current (fresh-FK) state. J_state + the wrench response G_vel
            // are taken BEFORE the step; the actuator-input velocity column too.
            let j_state = self
                .analytic_state_jacobian(&wrench)
                .unwrap_or_else(|| self.loaded_state_jacobian(&wrench));
            let g_vel = self.fresh_xfrc_column();
            let g_act_vel = self.actuator_velocity_column(); // nv × nu

            // (4)+(5) route the wrench and step (control already set above).
            self.data.xfrc_applied[self.body] = wrench;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in actuator trajectory");

            // Position rows: D = Δt·I (single hinge ⇒ Euclidean), so G_pos = Δt·G_vel and
            // G_act = [Δt·G_act_vel; G_act_vel] (2·nv × nu). Δt here is the RIGID
            // integrator's `model.timestep` (the same dt the velocity columns + the real
            // step use), NOT the soft `cfg.dt` — they are equal under lockstep but the
            // integrator term must track the rigid step.
            let rigid_dt = self.model.timestep;
            let g_pos = rigid_dt * &g_vel;
            let mut g_act = DMatrix::zeros(2 * nv, nu);
            g_act
                .view_mut((0, 0), (nv, nu))
                .copy_from(&(rigid_dt * &g_act_vel));
            g_act.view_mut((nv, 0), (nv, nu)).copy_from(&g_act_vel);

            let mut s_next = vec![0.0_f64; 2 * nv];
            for i in 0..nv {
                s_next[i] = self.data.qpos[i];
                s_next[nv + i] = self.data.qvel[i];
            }
            let s_next_var = tape.push_custom(
                &[s_var, w_var, control_vars[k]],
                Tensor::from_slice(&s_next, &[2 * nv]),
                Box::new(RigidStateCarryVjp {
                    j_state,
                    g_vel,
                    g_pos,
                    g_act: Some(g_act),
                }),
            );

            self.v = v_next;
            self.x = x_next;
            x_var = x_next_var;
            v_var = v_next_var;
            s_var = s_next_var;
        }

        // Objective: the fresh tip height = PoseSeam(s_N).
        self.data.forward(&self.model).expect("fresh FK (output)");
        let tip_z = self.data.xipos[self.body].z;
        let obj_var = tape.push_custom(
            &[s_var],
            Tensor::from_slice(&[tip_z], &[1]),
            Box::new(PoseSeamVjp {
                jz: self.pose_seam_jz(),
            }),
        );
        tape.backward(obj_var);
        let grad: Vec<f64> = control_vars
            .iter()
            .map(|&u| tape.grad_tensor(u).as_slice()[0])
            .collect();
        (tip_z, grad)
    }

    /// **The actuator CONTROL gradient THROUGH the friction grip — the powered-grip substrate.**
    /// The friction successor to [`Self::coupled_trajectory_actuator_gradient`] (normal-only): the
    /// actuated articulated limb now GRIPS the soft body via friction, so the per-step motor
    /// control `controls[k]` drives the limb against the full gripped contact wrench (normal +
    /// friction + off-COM moment) with the collider drift read from the articulated state. One
    /// `tape.backward(tip_x_N)` gives `(tip_x_N, [∂tip_x_N/∂u_0 … ∂u_{N−1}])` — the tip's final
    /// tangential drag vs every actuator control: the differentiable lever for how the agent should
    /// actuate to grip recoverably.
    ///
    /// Reuses the friction machinery (the gripped soft node + drift + the `FrictionWrenchTrajVjp`
    /// node) on the same tape as the actuator channel `s' = J_state·s + G·w + G_act·u`. `J_state` holds
    /// the GRIPPED wrench `w` with `ctrl` replicated (the FD `loaded_state_jacobian` captures the
    /// friction-loaded AND actuator-loaded geometric stiffness together); the single hinge uses the
    /// analytic friction-loaded carry. The objective is the tip's tangential `x` (the friction-
    /// relevant coordinate), not the `z` height of the normal-only gradient.
    ///
    /// **Scope.** A single AFFINE actuator (`force = gain·ctrl + bias`) on a EUCLIDEAN mechanism
    /// (`nq == nv` — hinge/slide chains), flat normal, friction active, `rigid_damping = 0`.
    /// FD-gated against [`Self::coupled_trajectory_actuated_gripped_x`]. The material `μ` rides as a
    /// constant leaf (a control+design gradient on one tape is a follow-on).
    ///
    /// # Panics
    /// Panics if `nq != nv`, if `nu != 1`, if friction is inactive, if `rigid_damping != 0`, if the
    /// rotating normal is enabled, or if a rigid/soft step diverges.
    #[must_use]
    // One coherent per-step tape-construction loop (mirrors the actuator gradient with the friction
    // grip swaps: gripped soft node + drift + friction wrench, keeping the actuator g_act channel).
    // expect_used: a rigid/soft divergence is a programmer error surfaced loudly, not recoverable.
    #[allow(clippy::too_many_lines, clippy::expect_used)]
    pub fn coupled_trajectory_actuator_friction_gradient(
        &mut self,
        controls: &[f64],
    ) -> (f64, Vec<f64>) {
        assert!(
            self.model.nq == self.model.nv,
            "actuator-friction gradient scope: Euclidean joints (nq == nv — hinge/slide chains)"
        );
        assert!(
            self.model.nu == 1,
            "actuator-friction gradient scope: exactly one actuator"
        );
        assert!(
            self.cfg.friction_mu > 0.0,
            "actuator-friction gradient requires friction active (cfg.friction_mu > 0)"
        );
        assert!(
            self.rigid_damping == 0.0,
            "articulated path requires rigid_damping = 0 (the coupling's free-platen knob)"
        );
        assert!(
            !self.rotating_normal,
            "actuator-friction gradient is flat-normal only (rotating normal unsupported)"
        );
        let n = self.n_vertices;
        let nv = self.model.nv;
        let nu = self.model.nu;
        let dt = self.cfg.dt;
        let pose_dir = Vec3::new(0.0, 0.0, 1.0);
        let drift_dir = Vec3::new(1.0, 0.0, 0.0);
        let centre_basis = [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ];

        let mut tape = Tape::new();
        // Material μ is a CONSTANT leaf here (only the control leaves are differentiated).
        let p_var = tape.constant_tensor(Tensor::from_slice(&[self.mu], &[1]));
        let control_vars: Vec<Var> = controls
            .iter()
            .map(|&u| tape.param_tensor(Tensor::from_slice(&[u], &[1])))
            .collect();
        let mut x_var = tape.constant_tensor(Tensor::from_slice(&self.x, &[3 * n]));
        let mut v_var = tape.constant_tensor(Tensor::from_slice(&self.v, &[3 * n]));
        let mut s0 = vec![0.0_f64; 2 * nv];
        for i in 0..nv {
            s0[i] = self.data.qpos[i];
            s0[nv + i] = self.data.qvel[i];
        }
        let mut s_var = tape.constant_tensor(Tensor::from_slice(&s0, &[2 * nv]));

        for (k, &u_k) in controls.iter().enumerate() {
            self.data.forward(&self.model).expect("fresh FK");
            // MOVING END-EFFECTOR (mirrors #429 friction + the g_act channel). Sphere-only.
            let moving_ee =
                self.contact_geom.is_some() && matches!(self.collider, Collider::Sphere { .. });
            if moving_ee {
                let g = self.contact_geom.expect("moving_ee ⇒ contact geom set");
                self.sphere_center_override = Some(self.data.geom_xpos[g]);
            }
            let height = self.tip_plane_height();
            let jlin = self.com_linear_jacobian();
            let jlin_x: Vec<f64> = (0..nv).map(|j| jlin[(0, j)]).collect();
            let v_com = &jlin * &self.data.qvel;
            let drift = Vec3::new(v_com[0], v_com[1], v_com[2]) * dt;
            let x_start = self.x.clone();

            let pose_var = if moving_ee {
                let centre = self.sphere_center_override.expect("moving-EE override set");
                tape.push_custom(
                    &[s_var],
                    Tensor::from_slice(centre.as_slice(), &[3]),
                    Box::new(PoseCentreVjp {
                        j_geom: self.pose_centre_jacobian(),
                    }),
                )
            } else {
                tape.push_custom(
                    &[s_var],
                    Tensor::from_slice(&[height], &[1]),
                    Box::new(PoseSeamVjp {
                        jz: self.pose_seam_jz(),
                    }),
                )
            };
            let drift_var = tape.push_custom(
                &[s_var],
                Tensor::from_slice(&[drift.x], &[1]),
                Box::new(DriftFromStateVjp { dt, jlin_x }),
            );

            // (1)+(2) one friction-aware soft step (μ a constant leaf; param_idx 0 unread).
            let bc = BoundaryConditions::new(self.pinned.clone(), Vec::new());
            let solver: SoftSolver<C> = CpuNewtonSolver::new(
                Tet4,
                self.fresh_mesh(),
                self.build_contact(height),
                self.cfg,
                bc,
            )
            .with_friction_surface_drift(drift);
            let x_next = solver
                .replay_step(
                    &Tensor::from_slice(&self.x, &[3 * n]),
                    &Tensor::from_slice(&self.v, &[3 * n]),
                    &Tensor::zeros(&[0]),
                    dt,
                )
                .x_final;
            let v_next: Vec<f64> = x_next
                .iter()
                .zip(&self.x)
                .map(|(xf, xo)| (xf - xo) / dt)
                .collect();
            let soft_grip = if moving_ee {
                solver.trajectory_step_vjp_grip_centre(
                    &x_next,
                    &x_start,
                    dt,
                    0,
                    &centre_basis,
                    drift_dir,
                )
            } else {
                solver.trajectory_step_vjp_grip(&x_next, &x_start, dt, 0, pose_dir, drift_dir)
            };
            let x_next_var = tape.push_custom(
                &[x_var, v_var, p_var, pose_var, drift_var],
                Tensor::from_slice(&x_next, &[3 * n]),
                Box::new(soft_grip),
            );
            let v_next_var = tape.push_custom(
                &[x_next_var, x_var],
                Tensor::from_slice(&v_next, &[3 * n]),
                Box::new(VelVjp {
                    inv_dt: 1.0 / dt,
                    n_dof: 3 * n,
                }),
            );

            // (3) NORMAL contact wrench (friction folds on separately below).
            let positions: Vec<Vec3> = x_next
                .chunks_exact(3)
                .map(|cc| Vec3::new(cc[0], cc[1], cc[2]))
                .collect();
            let c = self.data.xipos[self.body];
            let pairs = self.active_pair_wrench_data(height, &positions);
            let mut w_normal = SpatialVector::zeros();
            let mut active: Vec<(usize, Vec3, Vec3, f64, Vec3, Matrix3<f64>)> =
                Vec::with_capacity(pairs.len());
            for (v, g, nrm, curv, r) in pairs {
                let f = -g;
                w_normal[3] += f.x;
                w_normal[4] += f.y;
                w_normal[5] += f.z;
                let tau = (r - c).cross(&f);
                w_normal[0] += tau.x;
                w_normal[1] += tau.y;
                w_normal[2] += tau.z;
                active.push((v, g, nrm, curv, r - c, self.collider_hessian(height, r)));
            }
            let normal_force = Vec3::new(w_normal[3], w_normal[4], w_normal[5]);
            let normal_pose = if moving_ee {
                WrenchPose::Centre {
                    basis: centre_basis.to_vec(),
                }
            } else {
                WrenchPose::Height
            };
            let w_normal_var = tape.push_custom(
                &[x_next_var, pose_var, s_var],
                Tensor::from_slice(w_normal.as_slice(), &[6]),
                Box::new(ContactWrenchTrajVjp {
                    active,
                    force: normal_force,
                    jlin: jlin.clone(),
                    n_dof: 3 * n,
                    nv,
                    pose: normal_pose,
                }),
            );

            // (4) friction wrench (force + off-COM moment), folded onto the normal wrench.
            let pv = solver.friction_force_jacobians(&x_next, &x_start, dt, drift_dir, pose_dir);
            let mut w_total = w_normal;
            let pose_dforce = moving_ee.then(|| {
                let per_axis: Vec<Vec<Vec3>> = centre_basis
                    .iter()
                    .map(|&e| {
                        solver
                            .friction_force_jacobians(&x_next, &x_start, dt, drift_dir, e)
                            .into_iter()
                            .map(|p| p.dforce_dheight)
                            .collect()
                    })
                    .collect();
                (0..pv.len())
                    .map(|i| per_axis.iter().map(|axis| axis[i]).collect())
                    .collect::<Vec<Vec<Vec3>>>()
            });
            let n_pose = if moving_ee { 3 } else { 1 };
            let (fverts, f_fric_total) =
                assemble_friction_wrench(pv, &positions, c, &mut w_total, pose_dforce);
            let w_total_var = tape.push_custom(
                &[w_normal_var, x_next_var, pose_var, s_var, drift_var, x_var],
                Tensor::from_slice(w_total.as_slice(), &[6]),
                Box::new(FrictionWrenchTrajVjp {
                    verts: fverts,
                    f_total: f_fric_total,
                    jlin: jlin.clone(),
                    n_dof: 3 * n,
                    nv,
                    n_pose,
                    mu_c: false,
                }),
            );

            // Set the control BEFORE the carry (the FD `loaded_state_jacobian` replicates
            // `self.data.ctrl` in its scratch steps so the actuator force enters `J_state`).
            self.data.ctrl[0] = u_k;

            // (5) carry with the GRIPPED wrench held + the actuator-input channel.
            let j_state = self
                .analytic_state_jacobian(&w_total)
                .unwrap_or_else(|| self.loaded_state_jacobian(&w_total));
            let g_vel = self.fresh_xfrc_column();
            let g_act_vel = self.actuator_velocity_column(); // nv × nu

            self.data.xfrc_applied[self.body] = w_total;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in actuator-friction trajectory");

            let rigid_dt = self.model.timestep;
            let g_pos = rigid_dt * &g_vel;
            let mut g_act = DMatrix::zeros(2 * nv, nu);
            g_act
                .view_mut((0, 0), (nv, nu))
                .copy_from(&(rigid_dt * &g_act_vel));
            g_act.view_mut((nv, 0), (nv, nu)).copy_from(&g_act_vel);

            let mut s_next = vec![0.0_f64; 2 * nv];
            for i in 0..nv {
                s_next[i] = self.data.qpos[i];
                s_next[nv + i] = self.data.qvel[i];
            }
            let s_next_var = tape.push_custom(
                &[s_var, w_total_var, control_vars[k]],
                Tensor::from_slice(&s_next, &[2 * nv]),
                Box::new(RigidStateCarryVjp {
                    j_state,
                    g_vel,
                    g_pos,
                    g_act: Some(g_act),
                }),
            );

            self.v = v_next;
            self.x = x_next;
            x_var = x_next_var;
            v_var = v_next_var;
            s_var = s_next_var;
        }

        // Objective: the fresh tip tangential x = (COM x-Jacobian)·δq — the grip drag.
        self.data.forward(&self.model).expect("fresh FK (output)");
        let tip_x = self.data.xipos[self.body].x;
        let jx_final: Vec<f64> = {
            let jlin = self.com_linear_jacobian();
            (0..nv).map(|j| jlin[(0, j)]).collect()
        };
        let obj_var = tape.push_custom(
            &[s_var],
            Tensor::from_slice(&[tip_x], &[1]),
            Box::new(PoseSeamVjp { jz: jx_final }),
        );
        tape.backward(obj_var);
        let grad: Vec<f64> = control_vars
            .iter()
            .map(|&u| tape.grad_tensor(u).as_slice()[0])
            .collect();
        (tip_x, grad)
    }

    /// **The CLOSED-LOOP policy gradient THROUGH the friction grip — the de-escalation agent.**
    /// The policy successor to [`Self::coupled_trajectory_actuator_friction_gradient`] (explicit
    /// controls): the per-step actuator control is now a differentiable feedback policy
    /// `u_k = π_θ(qpos₀, qvel₀)` ([`DiffPolicy`] over the single hinge's joint angle + rate), so one
    /// `tape.backward(tip_x_N)` gives `(tip_x_N, ∂tip_x_N/∂θ)` — the tip's tangential drag vs every
    /// POLICY parameter, across the state→control recurrence (backprop-through-time). The
    /// differentiable lever for how the agent should DECIDE to actuate the grip from the limb's
    /// state.
    ///
    /// Reuses the entire #404 friction+actuator tape (gripped soft node + drift + friction wrench +
    /// the `G_act` carry channel); the only change is the control's provenance: each step the
    /// observation `(qpos, qvel)` is extracted from the carried state `s` (the `StateComponentVjp`
    /// node) and `u_var = policy.emit(θ, obs)` replaces the explicit control leaf as the carry's actuator
    /// parent. The chassis autograd threads `θ → u → G_act → s' → next obs → next u` automatically
    /// (no hand-rolled adjoint). The material `μ` rides as a constant leaf.
    ///
    /// **Scope.** A single AFFINE actuator on a EUCLIDEAN mechanism (`nq == nv`), flat normal,
    /// friction active, `rigid_damping = 0`. The policy observes the single hinge's joint state
    /// (`s[0]`, `s[nv]`). FD-gated against [`Self::coupled_trajectory_policy_gripped_x`]. The
    /// `eval`/`emit` views of the policy must agree (the gate's forward-match anchor catches drift).
    ///
    /// # Panics
    /// Panics if `params.len() != policy.n_params()`, if `nq != nv`, if `nu != 1`, if friction is
    /// inactive, if `rigid_damping != 0`, if the rotating normal is enabled, or if a step diverges.
    #[must_use]
    // One coherent per-step tape loop (mirrors the actuator-friction gradient; the explicit control
    // leaf is replaced by the policy node `emit(θ, obs)` reading the carried state).
    // expect_used: a rigid/soft divergence is a programmer error surfaced loudly, not recoverable.
    #[allow(clippy::too_many_lines, clippy::expect_used)]
    pub fn coupled_trajectory_policy_friction_gradient<P: DiffPolicy>(
        &mut self,
        policy: &P,
        params: &[f64],
        n_steps: usize,
    ) -> (f64, Vec<f64>) {
        assert_eq!(
            params.len(),
            policy.n_params(),
            "policy params length {} != n_params {}",
            params.len(),
            policy.n_params(),
        );
        assert!(
            self.model.nq == self.model.nv,
            "policy-friction gradient scope: Euclidean joints (nq == nv — hinge/slide chains)"
        );
        assert!(
            self.model.nu == 1,
            "policy-friction gradient scope: exactly one actuator"
        );
        assert!(
            self.cfg.friction_mu > 0.0,
            "policy-friction gradient requires friction active (cfg.friction_mu > 0)"
        );
        assert!(
            self.rigid_damping == 0.0,
            "articulated path requires rigid_damping = 0 (the coupling's free-platen knob)"
        );
        assert!(
            !self.rotating_normal,
            "policy-friction gradient is flat-normal only (rotating normal unsupported)"
        );
        let n = self.n_vertices;
        let nv = self.model.nv;
        let nu = self.model.nu;
        let dt = self.cfg.dt;
        let pose_dir = Vec3::new(0.0, 0.0, 1.0);
        let drift_dir = Vec3::new(1.0, 0.0, 0.0);
        let centre_basis = [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ];

        let mut tape = Tape::new();
        let p_var = tape.constant_tensor(Tensor::from_slice(&[self.mu], &[1]));
        // The policy parameter leaves (the gradient targets), shared across steps.
        let param_vars: Vec<Var> = params
            .iter()
            .map(|&p| tape.param_tensor(Tensor::from_slice(&[p], &[1])))
            .collect();
        let mut x_var = tape.constant_tensor(Tensor::from_slice(&self.x, &[3 * n]));
        let mut v_var = tape.constant_tensor(Tensor::from_slice(&self.v, &[3 * n]));
        let mut s0 = vec![0.0_f64; 2 * nv];
        for i in 0..nv {
            s0[i] = self.data.qpos[i];
            s0[nv + i] = self.data.qvel[i];
        }
        let mut s_var = tape.constant_tensor(Tensor::from_slice(&s0, &[2 * nv]));

        for _ in 0..n_steps {
            self.data.forward(&self.model).expect("fresh FK");
            // MOVING END-EFFECTOR (mirrors #429 friction + the g_act/policy channel). Sphere-only.
            let moving_ee =
                self.contact_geom.is_some() && matches!(self.collider, Collider::Sphere { .. });
            if moving_ee {
                let g = self.contact_geom.expect("moving_ee ⇒ contact geom set");
                self.sphere_center_override = Some(self.data.geom_xpos[g]);
            }

            // Closed-loop observation: extract the joint state (qpos[0] = s[0], qvel[0] = s[nv])
            // from the carried state, then emit u = π_θ(qpos, qvel) as a tape sub-expression. Its
            // forward value drives the physics; its var feeds the carry's actuator parent.
            let qpos_obs = tape.push_custom(
                &[s_var],
                Tensor::from_slice(&[self.data.qpos[0]], &[1]),
                Box::new(StateComponentVjp {
                    idx: 0,
                    n_state: 2 * nv,
                }),
            );
            let qvel_obs = tape.push_custom(
                &[s_var],
                Tensor::from_slice(&[self.data.qvel[0]], &[1]),
                Box::new(StateComponentVjp {
                    idx: nv,
                    n_state: 2 * nv,
                }),
            );
            let u_var = policy.emit(
                &mut tape,
                &param_vars,
                PolicyState {
                    z: qpos_obs,
                    vz: qvel_obs,
                },
            );
            let u_k = tape.value_tensor(u_var).as_slice()[0];

            let height = self.tip_plane_height();
            let jlin = self.com_linear_jacobian();
            let jlin_x: Vec<f64> = (0..nv).map(|j| jlin[(0, j)]).collect();
            let v_com = &jlin * &self.data.qvel;
            let drift = Vec3::new(v_com[0], v_com[1], v_com[2]) * dt;
            let x_start = self.x.clone();

            let pose_var = if moving_ee {
                let centre = self.sphere_center_override.expect("moving-EE override set");
                tape.push_custom(
                    &[s_var],
                    Tensor::from_slice(centre.as_slice(), &[3]),
                    Box::new(PoseCentreVjp {
                        j_geom: self.pose_centre_jacobian(),
                    }),
                )
            } else {
                tape.push_custom(
                    &[s_var],
                    Tensor::from_slice(&[height], &[1]),
                    Box::new(PoseSeamVjp {
                        jz: self.pose_seam_jz(),
                    }),
                )
            };
            let drift_var = tape.push_custom(
                &[s_var],
                Tensor::from_slice(&[drift.x], &[1]),
                Box::new(DriftFromStateVjp { dt, jlin_x }),
            );

            // (1)+(2) one friction-aware soft step (μ a constant leaf; param_idx 0 unread).
            let bc = BoundaryConditions::new(self.pinned.clone(), Vec::new());
            let solver: SoftSolver<C> = CpuNewtonSolver::new(
                Tet4,
                self.fresh_mesh(),
                self.build_contact(height),
                self.cfg,
                bc,
            )
            .with_friction_surface_drift(drift);
            let x_next = solver
                .replay_step(
                    &Tensor::from_slice(&self.x, &[3 * n]),
                    &Tensor::from_slice(&self.v, &[3 * n]),
                    &Tensor::zeros(&[0]),
                    dt,
                )
                .x_final;
            let v_next: Vec<f64> = x_next
                .iter()
                .zip(&self.x)
                .map(|(xf, xo)| (xf - xo) / dt)
                .collect();
            let soft_grip = if moving_ee {
                solver.trajectory_step_vjp_grip_centre(
                    &x_next,
                    &x_start,
                    dt,
                    0,
                    &centre_basis,
                    drift_dir,
                )
            } else {
                solver.trajectory_step_vjp_grip(&x_next, &x_start, dt, 0, pose_dir, drift_dir)
            };
            let x_next_var = tape.push_custom(
                &[x_var, v_var, p_var, pose_var, drift_var],
                Tensor::from_slice(&x_next, &[3 * n]),
                Box::new(soft_grip),
            );
            let v_next_var = tape.push_custom(
                &[x_next_var, x_var],
                Tensor::from_slice(&v_next, &[3 * n]),
                Box::new(VelVjp {
                    inv_dt: 1.0 / dt,
                    n_dof: 3 * n,
                }),
            );

            // (3) NORMAL contact wrench (friction folds on separately below).
            let positions: Vec<Vec3> = x_next
                .chunks_exact(3)
                .map(|cc| Vec3::new(cc[0], cc[1], cc[2]))
                .collect();
            let c = self.data.xipos[self.body];
            let pairs = self.active_pair_wrench_data(height, &positions);
            let mut w_normal = SpatialVector::zeros();
            let mut active: Vec<(usize, Vec3, Vec3, f64, Vec3, Matrix3<f64>)> =
                Vec::with_capacity(pairs.len());
            for (v, g, nrm, curv, r) in pairs {
                let f = -g;
                w_normal[3] += f.x;
                w_normal[4] += f.y;
                w_normal[5] += f.z;
                let tau = (r - c).cross(&f);
                w_normal[0] += tau.x;
                w_normal[1] += tau.y;
                w_normal[2] += tau.z;
                active.push((v, g, nrm, curv, r - c, self.collider_hessian(height, r)));
            }
            let normal_force = Vec3::new(w_normal[3], w_normal[4], w_normal[5]);
            let normal_pose = if moving_ee {
                WrenchPose::Centre {
                    basis: centre_basis.to_vec(),
                }
            } else {
                WrenchPose::Height
            };
            let w_normal_var = tape.push_custom(
                &[x_next_var, pose_var, s_var],
                Tensor::from_slice(w_normal.as_slice(), &[6]),
                Box::new(ContactWrenchTrajVjp {
                    active,
                    force: normal_force,
                    jlin: jlin.clone(),
                    n_dof: 3 * n,
                    nv,
                    pose: normal_pose,
                }),
            );

            // (4) friction wrench (force + off-COM moment), folded onto the normal wrench.
            let pv = solver.friction_force_jacobians(&x_next, &x_start, dt, drift_dir, pose_dir);
            let mut w_total = w_normal;
            let pose_dforce = moving_ee.then(|| {
                let per_axis: Vec<Vec<Vec3>> = centre_basis
                    .iter()
                    .map(|&e| {
                        solver
                            .friction_force_jacobians(&x_next, &x_start, dt, drift_dir, e)
                            .into_iter()
                            .map(|p| p.dforce_dheight)
                            .collect()
                    })
                    .collect();
                (0..pv.len())
                    .map(|i| per_axis.iter().map(|axis| axis[i]).collect())
                    .collect::<Vec<Vec<Vec3>>>()
            });
            let n_pose = if moving_ee { 3 } else { 1 };
            let (fverts, f_fric_total) =
                assemble_friction_wrench(pv, &positions, c, &mut w_total, pose_dforce);
            let w_total_var = tape.push_custom(
                &[w_normal_var, x_next_var, pose_var, s_var, drift_var, x_var],
                Tensor::from_slice(w_total.as_slice(), &[6]),
                Box::new(FrictionWrenchTrajVjp {
                    verts: fverts,
                    f_total: f_fric_total,
                    jlin: jlin.clone(),
                    n_dof: 3 * n,
                    nv,
                    n_pose,
                    mu_c: false,
                }),
            );

            // Set the control BEFORE the carry (the FD `loaded_state_jacobian` replicates
            // `self.data.ctrl` so the actuator force enters `J_state`).
            self.data.ctrl[0] = u_k;

            // (5) carry with the GRIPPED wrench held + the policy control as the actuator parent.
            let j_state = self
                .analytic_state_jacobian(&w_total)
                .unwrap_or_else(|| self.loaded_state_jacobian(&w_total));
            let g_vel = self.fresh_xfrc_column();
            let g_act_vel = self.actuator_velocity_column();

            self.data.xfrc_applied[self.body] = w_total;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in policy-friction trajectory");

            let rigid_dt = self.model.timestep;
            let g_pos = rigid_dt * &g_vel;
            let mut g_act = DMatrix::zeros(2 * nv, nu);
            g_act
                .view_mut((0, 0), (nv, nu))
                .copy_from(&(rigid_dt * &g_act_vel));
            g_act.view_mut((nv, 0), (nv, nu)).copy_from(&g_act_vel);

            let mut s_next = vec![0.0_f64; 2 * nv];
            for i in 0..nv {
                s_next[i] = self.data.qpos[i];
                s_next[nv + i] = self.data.qvel[i];
            }
            let s_next_var = tape.push_custom(
                &[s_var, w_total_var, u_var],
                Tensor::from_slice(&s_next, &[2 * nv]),
                Box::new(RigidStateCarryVjp {
                    j_state,
                    g_vel,
                    g_pos,
                    g_act: Some(g_act),
                }),
            );

            self.v = v_next;
            self.x = x_next;
            x_var = x_next_var;
            v_var = v_next_var;
            s_var = s_next_var;
        }

        // expect_used: a fresh-FK divergence is a programmer error surfaced loudly (mirrors `step`);
        // a statement-level allow because the moving-EE threading pushes this tail past the fn-level
        // allow's reach in the grade safety scanner's 300-line back-window.
        #[allow(clippy::expect_used)]
        self.data.forward(&self.model).expect("fresh FK (output)");
        let tip_x = self.data.xipos[self.body].x;
        let jx_final: Vec<f64> = {
            let jlin = self.com_linear_jacobian();
            (0..nv).map(|j| jlin[(0, j)]).collect()
        };
        let obj_var = tape.push_custom(
            &[s_var],
            Tensor::from_slice(&[tip_x], &[1]),
            Box::new(PoseSeamVjp { jz: jx_final }),
        );
        tape.backward(obj_var);
        let grad: Vec<f64> = param_vars
            .iter()
            .map(|&p| tape.grad_tensor(p).as_slice()[0])
            .collect();
        (tip_x, grad)
    }

    /// Build the shared design+policy friction-grip tape (the #406 per-step machinery), ready to seed
    /// with an objective. Both public design+policy gradients call this and then seed terminal
    /// `tip_x` or a trajectory-integrated cost over `qpos_steps` before one backward — the expensive
    /// coupled recurrence is built once, the objective is a thin tail. See [`DesignPolicyTape`].
    ///
    /// # Panics
    /// Panics if `params.len() != policy.n_params()`, if `nq != nv`, if `nu != 1`, if friction is
    /// inactive, if `rigid_damping != 0`, if the rotating normal is enabled, or if a step diverges.
    // One coherent per-step tape loop (fuses the friction-grip material design leaf and the closed-
    // loop policy leaves). expect_used: a rigid/soft divergence is a programmer error surfaced loudly.
    #[allow(clippy::too_many_lines, clippy::expect_used)]
    fn build_design_policy_tape<P: DiffPolicy>(
        &mut self,
        policy: &P,
        params: &[f64],
        n_steps: usize,
    ) -> DesignPolicyTape {
        assert_eq!(
            params.len(),
            policy.n_params(),
            "policy params length {} != n_params {}",
            params.len(),
            policy.n_params(),
        );
        assert!(
            self.model.nq == self.model.nv,
            "design+policy-friction gradient scope: Euclidean joints (nq == nv — hinge/slide chains)"
        );
        assert!(
            self.model.nu == 1,
            "design+policy-friction gradient scope: exactly one actuator"
        );
        assert!(
            self.cfg.friction_mu > 0.0,
            "design+policy-friction gradient requires friction active (cfg.friction_mu > 0)"
        );
        assert!(
            self.rigid_damping == 0.0,
            "articulated path requires rigid_damping = 0 (the coupling's free-platen knob)"
        );
        assert!(
            !self.rotating_normal,
            "design+policy-friction gradient is flat-normal only (rotating normal unsupported)"
        );
        let n = self.n_vertices;
        let nv = self.model.nv;
        let nu = self.model.nu;
        let dt = self.cfg.dt;
        let pose_dir = Vec3::new(0.0, 0.0, 1.0);
        let drift_dir = Vec3::new(1.0, 0.0, 0.0);
        let centre_basis = [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ];

        let mut tape = Tape::new();
        // The material design variable `μ` is now a DIFFERENTIATED leaf (the `λ = 4μ` tie rides the
        // combined grip node below), not a constant — this is change (1) vs the policy-only gradient.
        let p_var = tape.param_tensor(Tensor::from_slice(&[self.mu], &[1]));
        // The policy parameter leaves (the other gradient targets), shared across steps.
        let param_vars: Vec<Var> = params
            .iter()
            .map(|&p| tape.param_tensor(Tensor::from_slice(&[p], &[1])))
            .collect();
        let mut x_var = tape.constant_tensor(Tensor::from_slice(&self.x, &[3 * n]));
        let mut v_var = tape.constant_tensor(Tensor::from_slice(&self.v, &[3 * n]));
        let mut s0 = vec![0.0_f64; 2 * nv];
        for i in 0..nv {
            s0[i] = self.data.qpos[i];
            s0[nv + i] = self.data.qvel[i];
        }
        let mut s_var = tape.constant_tensor(Tensor::from_slice(&s0, &[2 * nv]));
        // Collect the per-step policy-observed hinge angle for trajectory-integrated objectives
        // (e.g. the holding cost). Pure observation of an already-taped Var — the terminal-`tip_x`
        // objective never reads it, so that path is unchanged.
        let mut qpos_steps: Vec<Var> = Vec::with_capacity(n_steps);

        for _ in 0..n_steps {
            self.data.forward(&self.model).expect("fresh FK");
            // MOVING END-EFFECTOR (mirrors #429 friction + the g_act/policy channel). Sphere-only.
            let moving_ee =
                self.contact_geom.is_some() && matches!(self.collider, Collider::Sphere { .. });
            if moving_ee {
                let g = self.contact_geom.expect("moving_ee ⇒ contact geom set");
                self.sphere_center_override = Some(self.data.geom_xpos[g]);
            }

            // Closed-loop observation: extract the joint state (qpos[0] = s[0], qvel[0] = s[nv])
            // from the carried state, then emit u = π_θ(qpos, qvel) as a tape sub-expression. Its
            // forward value drives the physics; its var feeds the carry's actuator parent.
            let qpos_obs = tape.push_custom(
                &[s_var],
                Tensor::from_slice(&[self.data.qpos[0]], &[1]),
                Box::new(StateComponentVjp {
                    idx: 0,
                    n_state: 2 * nv,
                }),
            );
            qpos_steps.push(qpos_obs);
            let qvel_obs = tape.push_custom(
                &[s_var],
                Tensor::from_slice(&[self.data.qvel[0]], &[1]),
                Box::new(StateComponentVjp {
                    idx: nv,
                    n_state: 2 * nv,
                }),
            );
            let u_var = policy.emit(
                &mut tape,
                &param_vars,
                PolicyState {
                    z: qpos_obs,
                    vz: qvel_obs,
                },
            );
            let u_k = tape.value_tensor(u_var).as_slice()[0];

            let height = self.tip_plane_height();
            let jlin = self.com_linear_jacobian();
            let jlin_x: Vec<f64> = (0..nv).map(|j| jlin[(0, j)]).collect();
            let v_com = &jlin * &self.data.qvel;
            let drift = Vec3::new(v_com[0], v_com[1], v_com[2]) * dt;
            let x_start = self.x.clone();

            let pose_var = if moving_ee {
                let centre = self.sphere_center_override.expect("moving-EE override set");
                tape.push_custom(
                    &[s_var],
                    Tensor::from_slice(centre.as_slice(), &[3]),
                    Box::new(PoseCentreVjp {
                        j_geom: self.pose_centre_jacobian(),
                    }),
                )
            } else {
                tape.push_custom(
                    &[s_var],
                    Tensor::from_slice(&[height], &[1]),
                    Box::new(PoseSeamVjp {
                        jz: self.pose_seam_jz(),
                    }),
                )
            };
            let drift_var = tape.push_custom(
                &[s_var],
                Tensor::from_slice(&[drift.x], &[1]),
                Box::new(DriftFromStateVjp { dt, jlin_x }),
            );

            // (1)+(2) one friction-aware soft step (μ a DIFFERENTIATED leaf via the combined grip
            // node — change (2): `grip_combined(&[1, 4])` carries the λ = 4μ total in one backward).
            let bc = BoundaryConditions::new(self.pinned.clone(), Vec::new());
            let solver: SoftSolver<C> = CpuNewtonSolver::new(
                Tet4,
                self.fresh_mesh(),
                self.build_contact(height),
                self.cfg,
                bc,
            )
            .with_friction_surface_drift(drift);
            let x_next = solver
                .replay_step(
                    &Tensor::from_slice(&self.x, &[3 * n]),
                    &Tensor::from_slice(&self.v, &[3 * n]),
                    &Tensor::zeros(&[0]),
                    dt,
                )
                .x_final;
            let v_next: Vec<f64> = x_next
                .iter()
                .zip(&self.x)
                .map(|(xf, xo)| (xf - xo) / dt)
                .collect();
            let soft_grip = if moving_ee {
                solver.trajectory_step_vjp_grip_combined_centre(
                    &x_next,
                    &x_start,
                    dt,
                    &[1.0, 4.0],
                    &centre_basis,
                    drift_dir,
                )
            } else {
                solver.trajectory_step_vjp_grip_combined(
                    &x_next,
                    &x_start,
                    dt,
                    &[1.0, 4.0],
                    pose_dir,
                    drift_dir,
                )
            };
            let x_next_var = tape.push_custom(
                &[x_var, v_var, p_var, pose_var, drift_var],
                Tensor::from_slice(&x_next, &[3 * n]),
                Box::new(soft_grip),
            );
            let v_next_var = tape.push_custom(
                &[x_next_var, x_var],
                Tensor::from_slice(&v_next, &[3 * n]),
                Box::new(VelVjp {
                    inv_dt: 1.0 / dt,
                    n_dof: 3 * n,
                }),
            );

            // (3) NORMAL contact wrench (friction folds on separately below).
            let positions: Vec<Vec3> = x_next
                .chunks_exact(3)
                .map(|cc| Vec3::new(cc[0], cc[1], cc[2]))
                .collect();
            let c = self.data.xipos[self.body];
            let pairs = self.active_pair_wrench_data(height, &positions);
            let mut w_normal = SpatialVector::zeros();
            let mut active: Vec<(usize, Vec3, Vec3, f64, Vec3, Matrix3<f64>)> =
                Vec::with_capacity(pairs.len());
            for (v, g, nrm, curv, r) in pairs {
                let f = -g;
                w_normal[3] += f.x;
                w_normal[4] += f.y;
                w_normal[5] += f.z;
                let tau = (r - c).cross(&f);
                w_normal[0] += tau.x;
                w_normal[1] += tau.y;
                w_normal[2] += tau.z;
                active.push((v, g, nrm, curv, r - c, self.collider_hessian(height, r)));
            }
            let normal_force = Vec3::new(w_normal[3], w_normal[4], w_normal[5]);
            let normal_pose = if moving_ee {
                WrenchPose::Centre {
                    basis: centre_basis.to_vec(),
                }
            } else {
                WrenchPose::Height
            };
            let w_normal_var = tape.push_custom(
                &[x_next_var, pose_var, s_var],
                Tensor::from_slice(w_normal.as_slice(), &[6]),
                Box::new(ContactWrenchTrajVjp {
                    active,
                    force: normal_force,
                    jlin: jlin.clone(),
                    n_dof: 3 * n,
                    nv,
                    pose: normal_pose,
                }),
            );

            // (4) friction wrench (force + off-COM moment), folded onto the normal wrench.
            let pv = solver.friction_force_jacobians(&x_next, &x_start, dt, drift_dir, pose_dir);
            let mut w_total = w_normal;
            let pose_dforce = moving_ee.then(|| {
                let per_axis: Vec<Vec<Vec3>> = centre_basis
                    .iter()
                    .map(|&e| {
                        solver
                            .friction_force_jacobians(&x_next, &x_start, dt, drift_dir, e)
                            .into_iter()
                            .map(|p| p.dforce_dheight)
                            .collect()
                    })
                    .collect();
                (0..pv.len())
                    .map(|i| per_axis.iter().map(|axis| axis[i]).collect())
                    .collect::<Vec<Vec<Vec3>>>()
            });
            let n_pose = if moving_ee { 3 } else { 1 };
            let (fverts, f_fric_total) =
                assemble_friction_wrench(pv, &positions, c, &mut w_total, pose_dforce);
            let w_total_var = tape.push_custom(
                &[w_normal_var, x_next_var, pose_var, s_var, drift_var, x_var],
                Tensor::from_slice(w_total.as_slice(), &[6]),
                Box::new(FrictionWrenchTrajVjp {
                    verts: fverts,
                    f_total: f_fric_total,
                    jlin: jlin.clone(),
                    n_dof: 3 * n,
                    nv,
                    n_pose,
                    mu_c: false,
                }),
            );

            // Set the control BEFORE the carry (the FD `loaded_state_jacobian` replicates
            // `self.data.ctrl` so the actuator force enters `J_state`).
            self.data.ctrl[0] = u_k;

            // (5) carry with the GRIPPED wrench held + the policy control as the actuator parent.
            let j_state = self
                .analytic_state_jacobian(&w_total)
                .unwrap_or_else(|| self.loaded_state_jacobian(&w_total));
            let g_vel = self.fresh_xfrc_column();
            let g_act_vel = self.actuator_velocity_column();

            self.data.xfrc_applied[self.body] = w_total;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in design+policy-friction trajectory");

            let rigid_dt = self.model.timestep;
            let g_pos = rigid_dt * &g_vel;
            let mut g_act = DMatrix::zeros(2 * nv, nu);
            g_act
                .view_mut((0, 0), (nv, nu))
                .copy_from(&(rigid_dt * &g_act_vel));
            g_act.view_mut((nv, 0), (nv, nu)).copy_from(&g_act_vel);

            let mut s_next = vec![0.0_f64; 2 * nv];
            for i in 0..nv {
                s_next[i] = self.data.qpos[i];
                s_next[nv + i] = self.data.qvel[i];
            }
            let s_next_var = tape.push_custom(
                &[s_var, w_total_var, u_var],
                Tensor::from_slice(&s_next, &[2 * nv]),
                Box::new(RigidStateCarryVjp {
                    j_state,
                    g_vel,
                    g_pos,
                    g_act: Some(g_act),
                }),
            );

            self.v = v_next;
            self.x = x_next;
            x_var = x_next_var;
            v_var = v_next_var;
            s_var = s_next_var;
        }

        // expect_used: a fresh-FK divergence is a programmer error surfaced loudly (mirrors `step`);
        // a statement-level allow because the moving-EE threading pushes this tail past the fn-level
        // allow's reach in the grade safety scanner's 300-line back-window.
        #[allow(clippy::expect_used)]
        self.data.forward(&self.model).expect("fresh FK (output)");
        let tip_x = self.data.xipos[self.body].x;
        let jx_final: Vec<f64> = {
            let jlin = self.com_linear_jacobian();
            (0..nv).map(|j| jlin[(0, j)]).collect()
        };
        DesignPolicyTape {
            tape,
            p_var,
            param_vars,
            qpos_steps,
            s_final: s_var,
            jx_final,
            tip_x,
        }
    }

    /// **Design + policy on ONE friction-grip tape — the mission's "one outer loop over BOTH".**
    /// The articulated-grip analog of [`Self::coupled_trajectory_joint_gradient`] (the free-platen
    /// design+policy gradient): roll the coupled system forward `n_steps` under a closed-loop
    /// feedback policy `u_k = π_θ(qpos₀, qvel₀)` ([`DiffPolicy`]) on ONE chassis tape, with **both**
    /// the soft material design variable `μ` (the stiffness scale, with the `λ = 4μ` tie) AND the
    /// policy parameters `θ` live as tape leaves, then a single `tape.backward(tip_x_N)` reads BOTH
    /// gradients at once: `(tip_x_N, ∂tip_x_N/∂μ_total, [∂tip_x_N/∂θ_0 …])`, where `∂tip_x_N/∂μ_total`
    /// is the total `∂/∂μ + 4·∂/∂λ` along the stiffness-scale line. The differentiable co-design lever
    /// for tuning the de-escalation buffer's stiffness AND the agent's feedback policy together,
    /// through an articulated friction grip (force + off-COM moment).
    ///
    /// This is the union of [`Self::coupled_trajectory_tangential_material_gradient_articulated`]
    /// (the friction-grip material design leaf) and
    /// [`Self::coupled_trajectory_policy_friction_gradient`] (the closed-loop policy leaves): the
    /// shared per-step tape (`build_design_policy_tape`) carries the material `μ` as a leaf
    /// (the `λ = 4μ` tie folded into the combined grip node) AND the policy leaves, so the reverse
    /// pass accumulates the material gradient (through the soft re-equilibration at every step) AND
    /// the policy gradient (backprop-through-time across the state→control recurrence) simultaneously
    /// — both design and policy, one friction-grip tape, one backward seeding the terminal `tip_x`.
    ///
    /// **Scope.** Identical to [`Self::coupled_trajectory_policy_friction_gradient`]: a single AFFINE
    /// actuator on a EUCLIDEAN mechanism (`nq == nv`), flat normal, friction active,
    /// `rigid_damping = 0`. The policy observes the single hinge's joint state (`s[0]`, `s[nv]`).
    /// FD-gated against [`Self::coupled_trajectory_policy_gripped_x`] on couplings rebuilt at `μ ± ε`
    /// (the material/design channel — `λ = 4μ` tied, so the build perturbation measures
    /// `∂/∂μ_total`) and at `θ ± ε` (the policy channel).
    ///
    /// # Panics
    /// Panics if `params.len() != policy.n_params()`, if `nq != nv`, if `nu != 1`, if friction is
    /// inactive, if `rigid_damping != 0`, if the rotating normal is enabled, or if a step diverges.
    #[must_use]
    pub fn coupled_trajectory_design_policy_friction_gradient<P: DiffPolicy>(
        &mut self,
        policy: &P,
        params: &[f64],
        n_steps: usize,
    ) -> (f64, f64, Vec<f64>) {
        let mut t = self.build_design_policy_tape(policy, params, n_steps);
        // Seed the terminal tangential drag `tip_x = xipos[body].x` and backward once.
        let obj_var = t.tape.push_custom(
            &[t.s_final],
            Tensor::from_slice(&[t.tip_x], &[1]),
            Box::new(PoseSeamVjp { jz: t.jx_final }),
        );
        t.tape.backward(obj_var);
        let mu_grad = t.tape.grad_tensor(t.p_var).as_slice()[0];
        let theta_grad: Vec<f64> = t
            .param_vars
            .iter()
            .map(|&p| t.tape.grad_tensor(p).as_slice()[0])
            .collect();
        (t.tip_x, mu_grad, theta_grad)
    }

    /// **Trajectory-integrated HOLDING gradient** — the first sustained-behavior co-design lever.
    /// Where [`Self::coupled_trajectory_design_policy_friction_gradient`] differentiates a *terminal*
    /// outcome (`tip_x_N`), this differentiates a cost summed over the WHOLE rollout: the squared
    /// deviation of the gripped limb's hinge angle from a held setpoint,
    /// `L = Σₖ (qₖ − q_hold)²` (k = 0 … n_steps−1, `qₖ` = the policy-observed `qpos[0]` at the *start*
    /// of step k). Minimizing `L` over `(μ, θ)` co-designs a buffer + holding policy that keep the
    /// limb *pressed at `q_hold`* throughout — a sustained de-escalation grip, not a transient touch.
    /// (The k = 0 term is the fixed initial angle — a constant offset in `L` with zero gradient, so it
    /// does not affect the minimizer; it is kept so `L` matches the forward oracle term-for-term.)
    ///
    /// Returns `(L, ∂L/∂μ_total, [∂L/∂θ_0 …])` from ONE backward over the shared design+policy tape
    /// (`build_design_policy_tape`): the per-step `qₖ` are already taped Vars, so the cost is
    /// a thin sum-of-squares tail on the same #406 machinery (no new physics). The `λ = 4μ` tie rides
    /// the same combined grip node, so `∂L/∂μ_total` is the total `∂/∂μ + 4·∂/∂λ`.
    ///
    /// **Scope.** Identical to the terminal sibling (`nq == nv`, one affine actuator, flat normal,
    /// friction active, `rigid_damping = 0`). FD-gated against a forward holding cost
    /// (`Σ (qₖ − q_hold)²` over [`Self::coupled_trajectory_policy_gripped_capture`]).
    ///
    /// # Panics
    /// Panics under the same conditions as [`Self::coupled_trajectory_design_policy_friction_gradient`].
    #[must_use]
    pub fn coupled_trajectory_design_policy_hold_gradient<P: DiffPolicy>(
        &mut self,
        policy: &P,
        params: &[f64],
        n_steps: usize,
        q_hold: f64,
    ) -> (f64, f64, Vec<f64>) {
        let mut t = self.build_design_policy_tape(policy, params, n_steps);
        // L = Σ (qₖ − q_hold)², built on the shared tape: subtract the q_hold constant, square via
        // mul, accumulate via add. One backward over the sum reads ∂L/∂μ and ∂L/∂θ.
        let neg_q_hold = t.tape.constant_tensor(Tensor::from_slice(&[-q_hold], &[1]));
        let mut cost_var = t.tape.constant_tensor(Tensor::from_slice(&[0.0], &[1]));
        for &q_var in &t.qpos_steps {
            let dev = t.tape.add(q_var, neg_q_hold);
            let sq = t.tape.mul(dev, dev);
            cost_var = t.tape.add(cost_var, sq);
        }
        // Read L from the tape's forward value (single source of truth — the same node `backward`
        // differentiates), not a parallel hand-sum.
        let cost_val = t.tape.value_tensor(cost_var).as_slice()[0];
        t.tape.backward(cost_var);
        let mu_grad = t.tape.grad_tensor(t.p_var).as_slice()[0];
        let theta_grad: Vec<f64> = t
            .param_vars
            .iter()
            .map(|&p| t.tape.grad_tensor(p).as_slice()[0])
            .collect();
        (cost_val, mu_grad, theta_grad)
    }

    /// **The keystone control-gradient (the policy half's substrate).** Roll the
    /// coupled system forward applying a per-step vertical **control force** to
    /// the platen — `controls[k]` newtons (world `+z`) at step k — while
    /// recording one chassis tape, then ONE `tape.backward(z_N)` gives the
    /// gradient of the platen's final height w.r.t. EVERY control input:
    /// `(z_N, [∂z_N/∂u_0 … ∂z_N/∂u_{N−1}])`. `controls.len()` is the rollout
    /// length.
    ///
    /// The control force adds to the same `xfrc_applied[body].z` the contact
    /// reaction uses, so the rigid carry becomes
    /// `vz' = a·vz − (Δt/m)·fz + (Δt/m)·u_k + Δt·g` and `∂vz'/∂u_k = +Δt/m` (the
    /// free-body factor, opposite sign to the contact term). Each `u_k` is a tape
    /// parameter leaf feeding a 3-parent `VzControlCarryVjp` velocity node; the
    /// rest of the per-step tape is identical to
    /// [`Self::coupled_trajectory_material_gradient`] (the soft
    /// `TrajectoryStepVjp`, the velocity/contact readouts, the position carry), so
    /// the reverse pass accumulates BOTH gradient paths each control input has on
    /// `z_N`: the direct rigid push (`u_k → vz' → z`) AND the indirect coupled
    /// path (`u_k` moves the platen → changes contact penetration → soft
    /// re-equilibration → reaction). Material parameters are held fixed (a *joint*
    /// design+policy gradient — both the material leaf and the control leaves on
    /// one tape — is a documented follow-on).
    ///
    /// Like [`Self::coupled_trajectory_material_gradient`] this takes `&mut self`
    /// (it runs the real rollout in place; the `sim-core` `Data` is not `Clone`),
    /// so build a fresh coupling per call. Forward values come from the real
    /// coupled dynamics (identical to [`Self::step`] with the control force added);
    /// the per-node Jacobians are the analytic/factored sensitivities. FD-validated
    /// against [`Self::coupled_trajectory_control_z`] (the real re-rolled coupled
    /// oracle) in `tests/coupled_control_gradient.rs`.
    ///
    /// # Panics
    /// Panics if the rigid step diverges or the soft solver does not converge —
    /// surfaced loudly as in [`Self::step`].
    #[must_use]
    // One coherent per-step tape-construction loop (mirrors the material gradient);
    // splitting it would scatter the recurrence's shared state.
    // expect_used: a rigid-step divergence is a programmer error surfaced loudly,
    // not recoverable for keystone-v1 (mirrors `step`'s rationale).
    #[allow(clippy::too_many_lines, clippy::expect_used)]
    pub fn coupled_trajectory_control_gradient(&mut self, controls: &[f64]) -> (f64, Vec<f64>) {
        // Free-body control: sphere-capable but no moving-EE centre carry yet.
        self.require_no_moving_ee();
        let n = self.n_vertices;
        let dt = self.cfg.dt;
        let dir = Vec3::new(0.0, 0.0, 1.0);
        // Free-body rigid carry coefficients (see coupled_trajectory_material_gradient).
        let dt_over_m = self.rigid_vz_response(0.0).1;
        let a = 1.0 - self.rigid_damping * dt_over_m;

        let mut tape = Tape::new();
        // Material leaf is NOT differentiated here; the soft node still needs a
        // parent for it, so make it a constant (μ; param_idx 0 is irrelevant to
        // the unused gradient since we read only the control leaves).
        let mu_var = tape.constant_tensor(Tensor::from_slice(&[self.mu], &[1]));
        // One control parameter leaf per step (the gradient targets).
        let control_vars: Vec<Var> = controls
            .iter()
            .map(|&u| tape.param_tensor(Tensor::from_slice(&[u], &[1])))
            .collect();
        let mut x_var = tape.constant_tensor(Tensor::from_slice(&self.x, &[3 * n]));
        let mut v_var = tape.constant_tensor(Tensor::from_slice(&self.v, &[3 * n]));
        let mut z_var =
            tape.constant_tensor(Tensor::from_slice(&[self.data.xpos[self.body].z], &[1]));
        let mut vz_var = tape.constant_tensor(Tensor::from_slice(&[self.data.qvel[2]], &[1]));
        let mut z_final = self.data.xpos[self.body].z;

        for (k, &u_k) in controls.iter().enumerate() {
            let height = self.plane_height();
            let vz_k = self.data.qvel[2];

            // (1)+(2) one dynamic soft step from the current (self.x, self.v).
            let (solver, x_next) = self.soft_resolve(height);
            let v_next: Vec<f64> = x_next
                .iter()
                .zip(&self.x)
                .map(|(xf, xo)| (xf - xo) / dt)
                .collect();
            // Soft node: x* with parents [x_prev, v_prev, μ, z_prev]. μ is a
            // constant leaf here so its (unread) gradient stays valid.
            let x_next_var = tape.push_custom(
                &[x_var, v_var, mu_var, z_var],
                Tensor::from_slice(&x_next, &[3 * n]),
                Box::new(solver.trajectory_step_vjp(&x_next, dt, 0, dir)),
            );
            let v_next_var = tape.push_custom(
                &[x_next_var, x_var],
                Tensor::from_slice(&v_next, &[3 * n]),
                Box::new(VelVjp {
                    inv_dt: 1.0 / dt,
                    n_dof: 3 * n,
                }),
            );

            // (3) contact force at the post-step config, plane at `height`.
            let positions: Vec<Vec3> = x_next
                .chunks_exact(3)
                .map(|c| Vec3::new(c[0], c[1], c[2]))
                .collect();
            let force_on_soft: Vec3 = self
                .build_contact(height)
                .pair_readout(&self.fresh_mesh(), &positions)
                .iter()
                .map(|r| r.force_on_soft)
                .sum();
            let factors = self.active_pair_force_factors(height, &positions);
            let fz_var = tape.push_custom(
                &[x_next_var, z_var],
                Tensor::from_slice(&[force_on_soft.z], &[1]),
                Box::new(ContactForceTrajVjp {
                    factors,
                    n_dof: 3 * n,
                }),
            );

            // (4)+(5) route the reaction + damping + the control force onto the
            // rigid body and step it (the real coupled dynamics).
            let mut sf = SpatialVector::zeros();
            sf[3] = -force_on_soft.x;
            sf[4] = -force_on_soft.y;
            sf[5] = -force_on_soft.z - self.rigid_damping * vz_k + u_k;
            self.data.xfrc_applied[self.body] = sf;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in coupled control trajectory");
            let z_next = self.data.xpos[self.body].z;
            let vz_next = self.data.qvel[2];

            // Rigid carry: vz' = a·vz − (Δt/m)·fz + (Δt/m)·u; z' = z + Δt·vz (the
            // OLD velocity — sim-core integrates position with the pre-update
            // velocity, so this step's force/control reaches z only NEXT step).
            let vz_next_var = tape.push_custom(
                &[vz_var, fz_var, control_vars[k]],
                Tensor::from_slice(&[vz_next], &[1]),
                Box::new(VzControlCarryVjp {
                    a,
                    neg_dt_over_m: -dt_over_m,
                    dt_over_m,
                }),
            );
            let z_next_var = tape.push_custom(
                &[z_var, vz_var],
                Tensor::from_slice(&[z_next], &[1]),
                Box::new(ZCarryVjp { dt }),
            );

            self.v = v_next;
            self.x = x_next;
            x_var = x_next_var;
            v_var = v_next_var;
            z_var = z_next_var;
            vz_var = vz_next_var;
            z_final = z_next;
        }

        tape.backward(z_var);
        let grad = control_vars
            .iter()
            .map(|&u_var| tape.grad_tensor(u_var).as_slice()[0])
            .collect();
        (z_final, grad)
    }

    /// Forward-only companion to [`Self::coupled_trajectory_control_gradient`]:
    /// the platen's final height `z_N` after the real coupled rollout applying the
    /// control schedule `controls` (no tape). The black-box oracle for
    /// finite-differencing `∂z_N/∂u_k`. Advances `self` (build a fresh coupling
    /// per call).
    ///
    /// # Panics
    /// Panics if the rigid step diverges or the soft solver does not converge.
    #[must_use]
    // expect_used: a rigid-step divergence / soft non-convergence is a programmer
    // error surfaced loudly, not recoverable for keystone-v1 (mirrors `step`).
    #[allow(clippy::expect_used)]
    pub fn coupled_trajectory_control_z(&mut self, controls: &[f64]) -> f64 {
        let dt = self.cfg.dt;
        for &u_k in controls {
            let height = self.plane_height();
            let vz_k = self.data.qvel[2];
            let (_solver, x_next) = self.soft_resolve(height);
            for (vi, (&xf, &xo)) in self.v.iter_mut().zip(x_next.iter().zip(self.x.iter())) {
                *vi = (xf - xo) / dt;
            }
            self.x = x_next;
            let force_on_soft = self.contact_force_at_height(height);
            let mut sf = SpatialVector::zeros();
            sf[3] = -force_on_soft.x;
            sf[4] = -force_on_soft.y;
            sf[5] = -force_on_soft.z - self.rigid_damping * vz_k + u_k;
            self.data.xfrc_applied[self.body] = sf;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in coupled control rollout");
        }
        self.data.xpos[self.body].z
    }

    /// Roll the coupled soft↔rigid system forward `n_steps` under a **closed-loop
    /// feedback policy** `u_k = π_θ(state_k)` ([`DiffPolicy`]), recording one
    /// chassis tape, then ONE `tape.backward(z_N)` gives the gradient of the
    /// platen's final height w.r.t. the policy parameters θ:
    /// `(z_N, [∂z_N/∂θ_0 … ∂z_N/∂θ_{P−1}])`, where `P = policy.n_params()`.
    ///
    /// This is the **closed-loop** analogue of
    /// [`Self::coupled_trajectory_control_gradient`]. There, each control `u_k` is
    /// an *independent* parameter leaf (an open-loop schedule). Here the policy
    /// parameters θ are leaves created once and **shared across every step**, and
    /// the control at step k is a *tape node* `u_k = π_θ(z_k, vz_k)` built from θ
    /// and the **loop-carried** state vars `z_var`/`vz_var` (already on the tape).
    /// Because those state vars depend on the policy's own outputs at earlier steps
    /// (via the velocity carry), the reverse pass accumulates `∂z_N/∂θ` over the
    /// whole **recurrence** — backprop-through-time — for free: the policy is just
    /// extra tape edges, not a new gradient primitive (the chassis autograd carries
    /// `∂u/∂θ` and `∂u/∂state` through [`DiffPolicy::emit`]). Material parameters are
    /// held fixed (a *joint* design+policy gradient is
    /// [`Self::coupled_trajectory_joint_gradient`]).
    ///
    /// Like the sibling trajectory-gradient methods this takes `&mut self` (it runs
    /// the real rollout in place; `Data` is not `Clone`), so build a fresh coupling
    /// per call. Forward values come from the real coupled dynamics (the policy's
    /// [`DiffPolicy::eval`] feeds the real `xfrc_applied`, identical to the tape
    /// node's value); the per-node Jacobians are the analytic/factored
    /// sensitivities. FD-validated against [`Self::coupled_trajectory_policy_z`]
    /// (the real re-rolled closed-loop oracle) in `tests/coupled_policy_gradient.rs`.
    ///
    /// # Panics
    /// Panics if the rigid step diverges or the soft solver does not converge —
    /// surfaced loudly as in [`Self::step`].
    #[must_use]
    // One coherent per-step tape-construction loop (mirrors the control gradient);
    // splitting it would scatter the recurrence's shared state.
    // expect_used: a rigid-step divergence is a programmer error surfaced loudly,
    // not recoverable for keystone-v1 (mirrors `step`'s rationale).
    #[allow(clippy::too_many_lines, clippy::expect_used)]
    pub fn coupled_trajectory_policy_gradient<P: DiffPolicy>(
        &mut self,
        policy: &P,
        params: &[f64],
        n_steps: usize,
    ) -> (f64, Vec<f64>) {
        // Free-body policy: sphere-capable but no moving-EE centre carry yet.
        self.require_no_moving_ee();
        assert_eq!(
            params.len(),
            policy.n_params(),
            "policy params length {} != n_params {}",
            params.len(),
            policy.n_params(),
        );
        let n = self.n_vertices;
        let dt = self.cfg.dt;
        let dir = Vec3::new(0.0, 0.0, 1.0);
        // Free-body rigid carry coefficients (see coupled_trajectory_material_gradient).
        let dt_over_m = self.rigid_vz_response(0.0).1;
        let a = 1.0 - self.rigid_damping * dt_over_m;

        let mut tape = Tape::new();
        // Material leaf is NOT differentiated here; the soft node still needs a
        // parent for it, so make it a constant (its unread gradient stays valid).
        let mu_var = tape.constant_tensor(Tensor::from_slice(&[self.mu], &[1]));
        // The policy parameter leaves (the gradient targets), shared across steps.
        let param_vars: Vec<Var> = params
            .iter()
            .map(|&p| tape.param_tensor(Tensor::from_slice(&[p], &[1])))
            .collect();
        let mut x_var = tape.constant_tensor(Tensor::from_slice(&self.x, &[3 * n]));
        let mut v_var = tape.constant_tensor(Tensor::from_slice(&self.v, &[3 * n]));
        let mut z_var =
            tape.constant_tensor(Tensor::from_slice(&[self.data.xpos[self.body].z], &[1]));
        let mut vz_var = tape.constant_tensor(Tensor::from_slice(&[self.data.qvel[2]], &[1]));
        let mut z_final = self.data.xpos[self.body].z;

        for _ in 0..n_steps {
            let height = self.plane_height();
            let vz_k = self.data.qvel[2];

            // Policy NODE: u_k = π_θ(z_k, vz_k) from the CURRENT loop-carried state
            // vars. Its forward value (== eval at the real state) drives the
            // physics; its var feeds the control parent of `VzControlCarryVjp`.
            let u_var = policy.emit(
                &mut tape,
                &param_vars,
                PolicyState {
                    z: z_var,
                    vz: vz_var,
                },
            );
            let u_k = tape.value_tensor(u_var).as_slice()[0];

            // (1)+(2) one dynamic soft step from the current (self.x, self.v).
            let (solver, x_next) = self.soft_resolve(height);
            let v_next: Vec<f64> = x_next
                .iter()
                .zip(&self.x)
                .map(|(xf, xo)| (xf - xo) / dt)
                .collect();
            let x_next_var = tape.push_custom(
                &[x_var, v_var, mu_var, z_var],
                Tensor::from_slice(&x_next, &[3 * n]),
                Box::new(solver.trajectory_step_vjp(&x_next, dt, 0, dir)),
            );
            let v_next_var = tape.push_custom(
                &[x_next_var, x_var],
                Tensor::from_slice(&v_next, &[3 * n]),
                Box::new(VelVjp {
                    inv_dt: 1.0 / dt,
                    n_dof: 3 * n,
                }),
            );

            // (3) contact force at the post-step config, plane at `height`.
            let positions: Vec<Vec3> = x_next
                .chunks_exact(3)
                .map(|c| Vec3::new(c[0], c[1], c[2]))
                .collect();
            let force_on_soft: Vec3 = self
                .build_contact(height)
                .pair_readout(&self.fresh_mesh(), &positions)
                .iter()
                .map(|r| r.force_on_soft)
                .sum();
            let factors = self.active_pair_force_factors(height, &positions);
            let fz_var = tape.push_custom(
                &[x_next_var, z_var],
                Tensor::from_slice(&[force_on_soft.z], &[1]),
                Box::new(ContactForceTrajVjp {
                    factors,
                    n_dof: 3 * n,
                }),
            );

            // (4)+(5) route the reaction + damping + the policy control force onto
            // the rigid body and step it (the real coupled dynamics).
            let mut sf = SpatialVector::zeros();
            sf[3] = -force_on_soft.x;
            sf[4] = -force_on_soft.y;
            sf[5] = -force_on_soft.z - self.rigid_damping * vz_k + u_k;
            self.data.xfrc_applied[self.body] = sf;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in coupled policy trajectory");
            let z_next = self.data.xpos[self.body].z;
            let vz_next = self.data.qvel[2];

            // Rigid carry: vz' = a·vz − (Δt/m)·fz + (Δt/m)·u; z' = z + Δt·vz (the
            // OLD velocity — sim-core integrates position with the pre-update
            // velocity, so this step's force/control reaches z only NEXT step).
            let vz_next_var = tape.push_custom(
                &[vz_var, fz_var, u_var],
                Tensor::from_slice(&[vz_next], &[1]),
                Box::new(VzControlCarryVjp {
                    a,
                    neg_dt_over_m: -dt_over_m,
                    dt_over_m,
                }),
            );
            let z_next_var = tape.push_custom(
                &[z_var, vz_var],
                Tensor::from_slice(&[z_next], &[1]),
                Box::new(ZCarryVjp { dt }),
            );

            self.v = v_next;
            self.x = x_next;
            x_var = x_next_var;
            v_var = v_next_var;
            z_var = z_next_var;
            vz_var = vz_next_var;
            z_final = z_next;
        }

        tape.backward(z_var);
        let grad = param_vars
            .iter()
            .map(|&p_var| tape.grad_tensor(p_var).as_slice()[0])
            .collect();
        (z_final, grad)
    }

    /// Forward-only companion to [`Self::coupled_trajectory_policy_gradient`]: the
    /// platen's final height `z_N` after the real `n_steps` closed-loop coupled
    /// rollout under `policy`/`params` (no tape; the policy re-evaluated each step
    /// from the real state via [`DiffPolicy::eval`]). The black-box oracle for
    /// finite-differencing `∂z_N/∂θ`. Advances `self` (build a fresh coupling per
    /// call).
    ///
    /// # Panics
    /// Panics if `params.len() != policy.n_params()`, or if the rigid step diverges
    /// / the soft solver does not converge.
    #[must_use]
    // expect_used: a rigid-step divergence / soft non-convergence is a programmer
    // error surfaced loudly, not recoverable for keystone-v1 (mirrors `step`).
    #[allow(clippy::expect_used)]
    pub fn coupled_trajectory_policy_z<P: DiffPolicy>(
        &mut self,
        policy: &P,
        params: &[f64],
        n_steps: usize,
    ) -> f64 {
        assert_eq!(
            params.len(),
            policy.n_params(),
            "policy params length {} != n_params {}",
            params.len(),
            policy.n_params(),
        );
        let dt = self.cfg.dt;
        for _ in 0..n_steps {
            let height = self.plane_height();
            let z_k = self.data.xpos[self.body].z;
            let vz_k = self.data.qvel[2];
            let u_k = policy.eval(params, z_k, vz_k);
            let (_solver, x_next) = self.soft_resolve(height);
            for (vi, (&xf, &xo)) in self.v.iter_mut().zip(x_next.iter().zip(self.x.iter())) {
                *vi = (xf - xo) / dt;
            }
            self.x = x_next;
            let force_on_soft = self.contact_force_at_height(height);
            let mut sf = SpatialVector::zeros();
            sf[3] = -force_on_soft.x;
            sf[4] = -force_on_soft.y;
            sf[5] = -force_on_soft.z - self.rigid_damping * vz_k + u_k;
            self.data.xfrc_applied[self.body] = sf;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in coupled policy rollout");
        }
        self.data.xpos[self.body].z
    }

    /// Forward-only oracle for the POLICY gradient THROUGH the friction grip — the policy-driven
    /// sibling of [`Self::coupled_trajectory_actuated_gripped_x`] (explicit controls) and the
    /// articulated+gripped successor to [`Self::coupled_trajectory_policy_z`] (free platen). Each
    /// step the closed-loop control `u_k = π_θ(qpos₀, qvel₀)` ([`DiffPolicy::eval`] on the single
    /// hinge's joint angle + rate) drives the gripped articulated limb; returns the body COM
    /// (`xipos`) so a gate can read the tangential drag (`.x`). The independent black-box reference
    /// for finite-differencing `∂tip_x/∂θ`. Mirrors the gradient's staggered order: observe state →
    /// soft solve → wrench readout → set ctrl → step.
    ///
    /// # Panics
    /// Panics if `params.len() != policy.n_params()`, or if a rigid/soft step diverges.
    #[must_use]
    pub fn coupled_trajectory_policy_gripped_x<P: DiffPolicy>(
        &mut self,
        policy: &P,
        params: &[f64],
        n_steps: usize,
    ) -> Vec3 {
        self.policy_gripped_rollout(policy, params, n_steps, None)
    }

    /// The closed-loop friction-grip rollout, capturing every frame — the
    /// **visualization** sibling of [`Self::coupled_trajectory_policy_gripped_x`].
    ///
    /// Runs the SAME physics (the shared `policy_gripped_rollout` loop body, so the
    /// byte-identical scalar `tip_x` is the first element of the tuple), but
    /// also returns one [`GripRolloutFrame`] before the first step (the rest state)
    /// and one after each step — `n_steps + 1` frames total. A viewer replays these
    /// to render the *exact* co-designed encounter the design+policy-friction
    /// gradient differentiates (see `examples/integration/`).
    ///
    /// # Panics
    /// Panics if `params.len() != policy.n_params()`, or if a rigid/soft step diverges.
    #[must_use]
    pub fn coupled_trajectory_policy_gripped_capture<P: DiffPolicy>(
        &mut self,
        policy: &P,
        params: &[f64],
        n_steps: usize,
    ) -> (Vec3, Vec<GripRolloutFrame>) {
        let mut frames = Vec::with_capacity(n_steps + 1);
        let tip = self.policy_gripped_rollout(policy, params, n_steps, Some(&mut frames));
        (tip, frames)
    }

    /// Shared loop body for the closed-loop friction-grip rollout. With `capture =
    /// None` this is the FD-oracle forward (byte-identical to the pre-refactor
    /// `coupled_trajectory_policy_gripped_x`); with `Some(sink)` it additionally
    /// pushes a [`GripRolloutFrame`] before the first step and after each step. The
    /// capture work is fully guarded behind `Some`, so the `None` path — the
    /// gradient's reference oracle — is unchanged (no extra `forward`, no clones).
    //
    // expect_used: a rigid/soft divergence is a programmer error surfaced loudly, not
    // recoverable (mirrors `coupled_trajectory_actuated_gripped_x` / `_policy_z`).
    #[allow(clippy::expect_used)]
    fn policy_gripped_rollout<P: DiffPolicy>(
        &mut self,
        policy: &P,
        params: &[f64],
        n_steps: usize,
        mut capture: Option<&mut Vec<GripRolloutFrame>>,
    ) -> Vec3 {
        assert_eq!(
            params.len(),
            policy.n_params(),
            "policy params length {} != n_params {}",
            params.len(),
            policy.n_params(),
        );
        let dt = self.cfg.dt;
        let n = self.n_vertices;
        // Rest frame (capture only): the undeformed soft body + the starting pose.
        if let Some(sink) = capture.as_deref_mut() {
            self.data
                .forward(&self.model)
                .expect("fresh FK (capture rest)");
            let h0 = self.tip_plane_height();
            sink.push(self.grip_frame(h0));
        }
        for _ in 0..n_steps {
            self.data.forward(&self.model).expect("fresh FK");
            // MOVING END-EFFECTOR: pose the sphere at the contact geom (FD oracle for the moving-EE
            // policy-friction gradients). Sphere-only; no-op for the plane / no geom.
            if let (Some(g), Collider::Sphere { .. }) = (self.contact_geom, self.collider) {
                self.sphere_center_override = Some(self.data.geom_xpos[g]);
            }
            // Observe the joint state (qpos[0], qvel[0]) at the step start — the closed-loop
            // convention (the gradient extracts the same scalars from the carried `s_var`).
            let u_k = policy.eval(params, self.data.qpos[0], self.data.qvel[0]);
            let height = self.tip_plane_height();
            let x_start = self.x.clone();
            let v_com = &self.com_linear_jacobian() * &self.data.qvel;
            let drift = Vec3::new(v_com[0], v_com[1], v_com[2]) * dt;
            let bc = BoundaryConditions::new(self.pinned.clone(), Vec::new());
            let solver: SoftSolver<C> = CpuNewtonSolver::new(
                Tet4,
                self.fresh_mesh(),
                self.build_contact(height),
                self.cfg,
                bc,
            )
            .with_friction_surface_drift(drift);
            let x_next = solver
                .replay_step(
                    &Tensor::from_slice(&self.x, &[3 * n]),
                    &Tensor::from_slice(&self.v, &[3 * n]),
                    &Tensor::zeros(&[0]),
                    dt,
                )
                .x_final;
            let friction = solver.friction_forces_on_soft(&x_next, &x_start, dt);
            for (vi, (&xf, &xo)) in self.v.iter_mut().zip(x_next.iter().zip(self.x.iter())) {
                *vi = (xf - xo) / dt;
            }
            self.x = x_next;
            self.data.xfrc_applied[self.body] = self.contact_wrench_gripped(height, &friction);
            self.data.ctrl[0] = u_k;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in policy grip rollout");
            // Post-step frame (capture only): the deformed soft body + the integrated pose.
            // The trailing `forward` refreshes `xpos` for the capture; it does not touch
            // `qpos`/`qvel`/`x`, so the `None` oracle path (which skips it) is unaffected.
            if let Some(sink) = capture.as_deref_mut() {
                self.data
                    .forward(&self.model)
                    .expect("fresh FK (capture step)");
                sink.push(self.grip_frame(height));
            }
        }
        self.data.forward(&self.model).expect("fresh FK (output)");
        self.data.xipos[self.body]
    }

    /// Snapshot the current state as a [`GripRolloutFrame`]: the deformed soft mesh,
    /// the gripped body's world origin, and the contact sphere centre posed at
    /// `height` (the value `build_contact` used this step).
    fn grip_frame(&self, height: f64) -> GripRolloutFrame {
        let radius = match self.collider {
            Collider::Sphere { radius } => radius,
            Collider::Plane => 0.0,
        };
        let fist = self.sphere_center(radius, height);
        let pivot = self.data.xpos[self.body];
        let tip = self.data.xipos[self.body];
        GripRolloutFrame {
            soft_positions: self.x.clone(),
            arm_pivot: [pivot.x, pivot.y, pivot.z],
            arm_tip: [tip.x, tip.y, tip.z],
            fist_center: [fist.x, fist.y, fist.z],
            qpos0: self.data.qpos[0],
        }
    }

    /// **Joint design + policy gradient** — the mission's "one outer loop
    /// differentiating w.r.t. **both** design and policy parameters". Roll the
    /// coupled system forward `n_steps` under a closed-loop feedback policy
    /// `u_k = π_θ(state_k)` ([`DiffPolicy`]) on ONE chassis tape, with **both** the
    /// soft material design variable `μ` (the stiffness scale, with the `λ = 4μ`
    /// tie) AND the policy parameters `θ` live as tape leaves, then a single
    /// `tape.backward(z_N)` reads BOTH gradients at once:
    /// `(z_N, ∂z_N/∂μ_total, [∂z_N/∂θ_0 …])`, where `∂z_N/∂μ_total` is the total
    /// `∂z_N/∂μ + 4·∂z_N/∂λ` along the stiffness-scale line.
    ///
    /// This is the union of [`Self::coupled_trajectory_material_gradient`] (the
    /// design leaf) and [`Self::coupled_trajectory_policy_gradient`] (the policy
    /// leaves): per step the soft node carries the material design variable as a
    /// parent built with `trajectory_step_vjp_combined(&[1, 4], …)` (so its single
    /// parent's cotangent is the λ = 4μ total in ONE backward), and the policy node
    /// feeds the control parent of `VzControlCarryVjp`. The reverse pass therefore
    /// accumulates the material gradient (through the soft re-equilibration at every
    /// step) and the policy gradient (backprop-through-time across the state→control
    /// recurrence) simultaneously — both design and policy, one tape, one backward.
    ///
    /// Takes `&mut self` (runs the real rollout in place), so build a fresh coupling
    /// per call. FD-validated against `coupled_trajectory_policy_z` on couplings
    /// rebuilt at `μ ± ε` (material block) and at `θ ± ε` (policy) in
    /// `tests/coupled_joint_gradient.rs`.
    ///
    /// # Panics
    /// Panics if `params.len() != policy.n_params()`, or if the rigid step diverges
    /// / the soft solver does not converge — surfaced loudly as in [`Self::step`].
    #[must_use]
    // One coherent per-step tape-construction loop (fuses the material design leaf
    // and the policy leaves); splitting it would scatter the recurrence's state.
    // expect_used: a rigid-step divergence is a programmer error surfaced loudly.
    #[allow(clippy::too_many_lines, clippy::expect_used)]
    pub fn coupled_trajectory_joint_gradient<P: DiffPolicy>(
        &mut self,
        policy: &P,
        params: &[f64],
        n_steps: usize,
    ) -> (f64, f64, Vec<f64>) {
        // Free-body joint design+policy: sphere-capable but no moving-EE centre carry yet.
        self.require_no_moving_ee();
        assert_eq!(
            params.len(),
            policy.n_params(),
            "policy params length {} != n_params {}",
            params.len(),
            policy.n_params(),
        );
        let n = self.n_vertices;
        let dt = self.cfg.dt;
        let dir = Vec3::new(0.0, 0.0, 1.0);
        let dt_over_m = self.rigid_vz_response(0.0).1;
        let a = 1.0 - self.rigid_damping * dt_over_m;

        let mut tape = Tape::new();
        // The MATERIAL design leaf (value = μ; its gradient is the λ = 4μ total via
        // the combined-weights soft VJP) AND the policy param leaves — both live.
        let mu_var = tape.param_tensor(Tensor::from_slice(&[self.mu], &[1]));
        let param_vars: Vec<Var> = params
            .iter()
            .map(|&p| tape.param_tensor(Tensor::from_slice(&[p], &[1])))
            .collect();
        let mut x_var = tape.constant_tensor(Tensor::from_slice(&self.x, &[3 * n]));
        let mut v_var = tape.constant_tensor(Tensor::from_slice(&self.v, &[3 * n]));
        let mut z_var =
            tape.constant_tensor(Tensor::from_slice(&[self.data.xpos[self.body].z], &[1]));
        let mut vz_var = tape.constant_tensor(Tensor::from_slice(&[self.data.qvel[2]], &[1]));
        let mut z_final = self.data.xpos[self.body].z;

        for _ in 0..n_steps {
            let height = self.plane_height();
            let vz_k = self.data.qvel[2];

            // Policy NODE: u_k = π_θ(z_k, vz_k) from the current state vars.
            let u_var = policy.emit(
                &mut tape,
                &param_vars,
                PolicyState {
                    z: z_var,
                    vz: vz_var,
                },
            );
            let u_k = tape.value_tensor(u_var).as_slice()[0];

            // (1)+(2) one dynamic soft step. The soft node's material parent is the
            // stiffness-scale design variable: weights [1, 4] ⇒ ∂/∂μ_total.
            let (solver, x_next) = self.soft_resolve(height);
            let v_next: Vec<f64> = x_next
                .iter()
                .zip(&self.x)
                .map(|(xf, xo)| (xf - xo) / dt)
                .collect();
            let x_next_var = tape.push_custom(
                &[x_var, v_var, mu_var, z_var],
                Tensor::from_slice(&x_next, &[3 * n]),
                Box::new(solver.trajectory_step_vjp_combined(&x_next, dt, &[1.0, 4.0], dir)),
            );
            let v_next_var = tape.push_custom(
                &[x_next_var, x_var],
                Tensor::from_slice(&v_next, &[3 * n]),
                Box::new(VelVjp {
                    inv_dt: 1.0 / dt,
                    n_dof: 3 * n,
                }),
            );

            // (3) contact force at the post-step config, plane at `height`.
            let positions: Vec<Vec3> = x_next
                .chunks_exact(3)
                .map(|c| Vec3::new(c[0], c[1], c[2]))
                .collect();
            let force_on_soft: Vec3 = self
                .build_contact(height)
                .pair_readout(&self.fresh_mesh(), &positions)
                .iter()
                .map(|r| r.force_on_soft)
                .sum();
            let factors = self.active_pair_force_factors(height, &positions);
            let fz_var = tape.push_custom(
                &[x_next_var, z_var],
                Tensor::from_slice(&[force_on_soft.z], &[1]),
                Box::new(ContactForceTrajVjp {
                    factors,
                    n_dof: 3 * n,
                }),
            );

            // (4)+(5) reaction + damping + policy control force onto the platen.
            let mut sf = SpatialVector::zeros();
            sf[3] = -force_on_soft.x;
            sf[4] = -force_on_soft.y;
            sf[5] = -force_on_soft.z - self.rigid_damping * vz_k + u_k;
            self.data.xfrc_applied[self.body] = sf;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in coupled joint trajectory");
            let z_next = self.data.xpos[self.body].z;
            let vz_next = self.data.qvel[2];

            let vz_next_var = tape.push_custom(
                &[vz_var, fz_var, u_var],
                Tensor::from_slice(&[vz_next], &[1]),
                Box::new(VzControlCarryVjp {
                    a,
                    neg_dt_over_m: -dt_over_m,
                    dt_over_m,
                }),
            );
            let z_next_var = tape.push_custom(
                &[z_var, vz_var],
                Tensor::from_slice(&[z_next], &[1]),
                Box::new(ZCarryVjp { dt }),
            );

            self.v = v_next;
            self.x = x_next;
            x_var = x_next_var;
            v_var = v_next_var;
            z_var = z_next_var;
            vz_var = vz_next_var;
            z_final = z_next;
        }

        tape.backward(z_var);
        let dz_dmu = tape.grad_tensor(mu_var).as_slice()[0];
        let dz_dtheta = param_vars
            .iter()
            .map(|&p_var| tape.grad_tensor(p_var).as_slice()[0])
            .collect();
        (z_final, dz_dmu, dz_dtheta)
    }

    /// Advance the coupled system by one lockstep step. Returns the contact
    /// force on the soft body and the rigid body's current height.
    ///
    /// # Panics
    /// Panics if the rigid engine's step diverges — for the validated forward
    /// scene this does not occur; a panic signals a mis-constructed or unstable
    /// coupling, surfaced loudly rather than silently corrupting state.
    //
    // A rigid-step failure is divergence / a programmer error surfaced loudly,
    // not a recoverable condition for keystone-v1 (a Result-returning step is a
    // deferred robustness upgrade alongside the LM / non-smooth-contact work).
    #[allow(clippy::expect_used)]
    pub fn step(&mut self) -> CoupledStep {
        let height = self.plane_height();
        let n = self.n_vertices;

        // (1)+(2) pose the contact from the rigid body; one dynamic soft step.
        let bc = BoundaryConditions::new(self.pinned.clone(), Vec::new());
        let contact = self.build_contact(height);
        let solver: SoftSolver<C> =
            CpuNewtonSolver::new(Tet4, self.fresh_mesh(), contact, self.cfg, bc);
        let res = solver.replay_step(
            &Tensor::from_slice(&self.x, &[3 * n]),
            &Tensor::from_slice(&self.v, &[3 * n]),
            &Tensor::zeros(&[0]),
            self.cfg.dt,
        );
        let x_final: Vec<f64> = res.x_final;
        let dt = self.cfg.dt;
        for (vi, (&xf, &xo)) in self.v.iter_mut().zip(x_final.iter().zip(self.x.iter())) {
            *vi = (xf - xo) / dt;
        }
        self.x = x_final;

        // (3) total contact force on the soft body.
        let force_on_soft = self.contact_force_at_height(height);

        // (4) route the reaction (−force_on_soft) + axis damping → rigid xfrc as
        // a pure force at the body COM. The contact moment (Σ rᵢ × fᵢ) is omitted
        // — exact for the ~symmetric scene here; a deferred generalization for
        // off-centre contact (alongside general posed primitives).
        let v_axis = self.data.qvel[2]; // free-joint linear z velocity
        let mut sf = SpatialVector::zeros(); // [angular(3), linear(3)]
        sf[3] = -force_on_soft.x;
        sf[4] = -force_on_soft.y;
        sf[5] = -force_on_soft.z - self.rigid_damping * v_axis;
        self.data.xfrc_applied[self.body] = sf;

        // (5) step the rigid body.
        self.data
            .step(&self.model)
            .expect("rigid step diverged in coupled solve");

        CoupledStep {
            force_on_soft,
            rigid_z: self.data.xpos[self.body].z,
        }
    }

    /// Advance the coupled system by one lockstep step for an **articulated** body,
    /// routing the full off-COM contact **wrench** `[τ; f]` (not the pure force at the
    /// COM that [`Self::step`] uses) — the per-frame forward companion to the
    /// articulated trajectory rollouts ([`Self::coupled_trajectory_articulated_z`]),
    /// exposing the per-step state a viewer captures.
    ///
    /// Per step: fresh FK (so the contact poses at the *current* tip, no one-step lag),
    /// pose the finite sphere collider at the contact end-effector
    /// ([`Self::with_contact_geom`]'s geom centre, else the block-centroid default),
    /// one dynamic soft step, route the reaction wrench `[(rᵢ − c) × fᵢ ; fᵢ]` about the
    /// body COM `c = xipos` to `xfrc_applied`, then step the rigid body. Returns the
    /// contact force on the soft body and the body reference height; read
    /// [`Self::soft_positions`] / [`Self::data`] for the deformed mesh + arm pose.
    ///
    /// # Panics
    /// Panics if the rigid step diverges (as in [`Self::step`]).
    //
    // expect_used: a rigid-step divergence / soft non-convergence is a programmer error surfaced
    // loudly, not recoverable for keystone-v1 (mirrors `step`).
    #[allow(clippy::expect_used)]
    pub fn step_articulated(&mut self) -> CoupledStep {
        // Fresh FK: pose the contact + COM from the CURRENT config (no one-step lag —
        // sim-core's `step` integrates without trailing FK; see
        // `coupled_trajectory_articulated_z`).
        self.data.forward(&self.model).expect("fresh FK");
        // Pose the finite sphere at the contact end-effector geom (the arm tip), so the
        // fist tracks the striker rather than sitting over the block centroid.
        if let Some(g) = self.contact_geom {
            self.sphere_center_override = Some(self.data.geom_xpos[g]);
        }
        let height = self.tip_plane_height();

        let (_solver, x_next) = self.soft_resolve(height);
        let dt = self.cfg.dt;
        for (vi, (&xf, &xo)) in self.v.iter_mut().zip(x_next.iter().zip(self.x.iter())) {
            *vi = (xf - xo) / dt;
        }
        self.x = x_next;

        let force_on_soft = self.contact_force_at_height(height);
        self.data.xfrc_applied[self.body] = self.contact_wrench(height);
        self.data
            .step(&self.model)
            .expect("rigid step diverged in articulated coupled solve");
        // Trailing FK so the post-step pose (`data().xpos` / `geom_xpos`) and `rigid_z` are fresh
        // for the caller's per-frame capture (sim-core's `step` integrates without trailing FK).
        self.data
            .forward(&self.model)
            .expect("fresh FK (post-step)");

        CoupledStep {
            force_on_soft,
            rigid_z: self.data.xpos[self.body].z,
        }
    }

    /// Advance the SOFT body one lockstep step under a **kinematically driven**
    /// collider — a ONE-WAY coupling: the finite sphere is posed at the caller's
    /// scripted end-effector ([`Self::set_sphere_center`], or [`Self::with_contact_geom`]
    /// off a caller-posed rigid geom), the soft body responds, and NO reaction is fed
    /// back (no rigid integration). The per-frame forward stepper the *kinematic*
    /// striker viewers drive (a scripted fist path; the dynamic two-way sibling is
    /// [`Self::step_articulated`]).
    ///
    /// Pose the collider via [`Self::set_sphere_center`] **before** each call. Returns
    /// the contact force on the soft body; read [`Self::soft_positions`] for the
    /// deformed mesh. The `height` carry is unused — the override governs posing.
    pub fn step_kinematic(&mut self) -> CoupledStep {
        // A caller-posed rigid geom (if any) drives the collider; else the explicit
        // `set_sphere_center` override does. No rigid step / FK — the collider is external.
        if let Some(g) = self.contact_geom {
            self.sphere_center_override = Some(self.data.geom_xpos[g]);
        }
        // The override governs posing, so the `height` arg is ignored (see `sphere_center`).
        let height = 0.0;
        let (_solver, x_next) = self.soft_resolve(height);
        let dt = self.cfg.dt;
        for (vi, (&xf, &xo)) in self.v.iter_mut().zip(x_next.iter().zip(self.x.iter())) {
            *vi = (xf - xo) / dt;
        }
        self.x = x_next;

        let force_on_soft = self.contact_force_at_height(height);
        CoupledStep {
            force_on_soft,
            rigid_z: self.data.xpos[self.body].z,
        }
    }

    /// Roll the coupled system forward `n_steps` **with the tangential friction grip**,
    /// returning the rigid body's final world position `xpos[body]`. The friction-coupled
    /// forward companion to [`Self::step`] (which is normal-only, force-at-COM): per step it
    /// poses the contact from the current rigid config, takes one friction-aware soft step
    /// fed the collider drift `Δ_surf = v_rigid_tangential·dt` (so the moving rigid surface
    /// DRAGS the soft body), then routes the soft body's friction reaction (normal + the
    /// tangential grip, with the off-COM moment) back onto the rigid body via the gripped
    /// contact wrench, plus the contact-axis (z) damping `step` applies.
    ///
    /// Friction must do tangential WORK for grip to register, so the scene needs a tangential
    /// rigid DOF/load: a free-joint platen under a sideways push (e.g. tilted gravity) slides
    /// freely when frictionless and is HELD once friction grips. Returns the full position so
    /// a gate can read the tangential slide. With `friction_mu == 0` the drift and the
    /// friction reaction are both empty, so the rollout reduces to the plain coupled dynamics
    /// (the platen still slides). Advances `self` (build a fresh coupling per call).
    ///
    /// Assumes the rigid `body` has a **free joint** (the keystone platen): the collider drift
    /// reads `qvel[0..3]` as the world-frame linear velocity, and the omitted `ω×r` per-contact
    /// rotation term is negligible for the COM-centered platen contact. A non-free-joint body
    /// (hinge/slide) would mis-read `qvel`, so this is scoped to the free-joint platen for now.
    ///
    /// # Panics
    /// Panics if the rigid step diverges or the soft solver fails (as in [`Self::step`]).
    #[must_use]
    // expect_used: a rigid-step divergence / soft non-convergence is a programmer error
    // surfaced loudly, not recoverable for keystone-v1 (mirrors `step`).
    #[allow(clippy::expect_used)]
    pub fn coupled_trajectory_grip(&mut self, n_steps: usize) -> Vec3 {
        // Free-body grip forward: sphere-capable but does not pose at the contact geom
        // (no moving-EE) — reject a set geom so it can't silently centroid-pose vs a caller's
        // tip-posed expectation (its friction gradient sibling is likewise guarded).
        self.require_no_moving_ee();
        let dt = self.cfg.dt;
        let n = self.n_vertices;
        for _ in 0..n_steps {
            // FRESH FK: pose the contact / COM from the CURRENT config (no one-step lag,
            // matching `coupled_trajectory_articulated_z`).
            self.data.forward(&self.model).expect("fresh FK");
            let height = self.plane_height();
            // Step-start soft config xᵗ — captured BEFORE the solve overwrites `self.x`;
            // the friction potential differences the post-step config against it.
            let x_start = self.x.clone();
            // Δ_surf: the platen's within-step tangential sweep (linear velocity × dt). The
            // friction tangent projects out the normal component, so the full linear
            // displacement is correct for the flat downward plane (rotation ω×r omitted).
            let drift = Vec3::new(self.data.qvel[0], self.data.qvel[1], self.data.qvel[2]) * dt;
            // One friction-aware dynamic soft step with the moving-collider drift.
            let bc = BoundaryConditions::new(self.pinned.clone(), Vec::new());
            let solver: SoftSolver<C> = CpuNewtonSolver::new(
                Tet4,
                self.fresh_mesh(),
                self.build_contact(height),
                self.cfg,
                bc,
            )
            .with_friction_surface_drift(drift);
            let x_next = solver
                .replay_step(
                    &Tensor::from_slice(&self.x, &[3 * n]),
                    &Tensor::from_slice(&self.v, &[3 * n]),
                    &Tensor::zeros(&[0]),
                    dt,
                )
                .x_final;
            // The friction reaction (with the same drift) at the post-step config.
            let friction = solver.friction_forces_on_soft(&x_next, &x_start, dt);
            for (vi, (&xf, &xo)) in self.v.iter_mut().zip(x_next.iter().zip(self.x.iter())) {
                *vi = (xf - xo) / dt;
            }
            self.x = x_next;
            // Route the gripped reaction wrench [τ; f] (normal + friction) about the fresh-FK
            // COM, plus the contact-axis (z) damping so the platen settles vertically.
            let mut wrench = self.contact_wrench_gripped(height, &friction);
            wrench[5] -= self.rigid_damping * self.data.qvel[2];
            self.data.xfrc_applied[self.body] = wrench;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in grip rollout");
        }
        self.data.forward(&self.model).expect("fresh FK (output)");
        self.data.xpos[self.body]
    }

    /// **Forward-only oracle for the articulated FRICTION (gripped) trajectory gradient.**
    /// The articulated successor to [`Self::coupled_trajectory_grip`] (free platen) and the
    /// gripped sibling of [`Self::coupled_trajectory_articulated_z`] (normal-only articulated):
    /// rolls the coupled system forward `n_steps`, routing the full GRIPPED reaction wrench
    /// `[τ; f]` (normal + friction + off-COM moment, `contact_wrench_gripped`) onto the
    /// ARTICULATED body. The moving-collider drift is read from the articulated state —
    /// `Δ_surf = (J_lin·qvel)·dt`, the COM linear velocity (the multi-DOF successor to the free
    /// platen's `qvel[0..3]`), with `J_lin = ∂c/∂q` the COM linear Jacobian
    /// (`com_linear_jacobian`, the same one the wrench moment uses). Returns the body
    /// COM (`xipos` — the swept TIP, not the fixed hinge pivot `xpos`) after the rollout so a
    /// gate can read both the tangential drag (`.x`) and the engagement height (`.z`).
    ///
    /// No tape — the independent black-box reference for finite-differencing the articulated
    /// friction gradient (sub-leaf 1 of the friction → matrix-carry lift). Advances `self`
    /// (build a fresh coupling per call). With [`Self::with_contact_geom`] + a sphere collider it
    /// poses the contact at the geom each step (the moving-EE friction forward oracle, matching the
    /// gradient's posing); a no-op for the plane / no geom.
    ///
    /// **Scope (v1).** A EUCLIDEAN articulated mechanism (hinge/slide chain — `nq == nv`), flat
    /// normal, friction active. The drift uses the COM linear velocity (matching the gradient's
    /// drift-from-state channel and the wrench moment's `c(q)`); the per-contact `ω×r` rotation
    /// term is omitted, as in [`Self::coupled_trajectory_grip`]. No `rigid_damping` z-term — the
    /// articulated path damps through the model's joint damping (`M_impl`), not the free-platen
    /// settling knob.
    ///
    /// # Panics
    /// Panics if the rigid step diverges or the soft solver fails (as in [`Self::step`]).
    #[must_use]
    // expect_used: a rigid-step divergence / soft non-convergence is a programmer error
    // surfaced loudly, not recoverable for keystone-v1 (mirrors `coupled_trajectory_grip`).
    #[allow(clippy::expect_used)]
    pub fn coupled_trajectory_gripped_articulated(&mut self, n_steps: usize) -> Vec3 {
        let dt = self.cfg.dt;
        let n = self.n_vertices;
        for _ in 0..n_steps {
            // FRESH FK: pose the contact / COM from the CURRENT config (no one-step lag,
            // matching `coupled_trajectory_articulated_z`).
            self.data.forward(&self.model).expect("fresh FK");
            // MOVING END-EFFECTOR: pose the finite sphere at the contact geom each step — the
            // friction-grip forward companion of the moving-EE friction gradient (its FD oracle),
            // so forward and adjoint share posing. Sphere-only; no-op for the plane/no-geom.
            if let (Some(g), Collider::Sphere { .. }) = (self.contact_geom, self.collider) {
                self.sphere_center_override = Some(self.data.geom_xpos[g]);
            }
            let height = self.tip_plane_height();
            // Step-start soft config xᵗ — captured BEFORE the solve overwrites `self.x`;
            // the friction potential differences the post-step config against it.
            let x_start = self.x.clone();
            // Δ_surf: the contacting body's within-step tangential sweep from the ARTICULATED
            // state — the COM linear velocity `J_lin·qvel` (the multi-DOF successor to the free
            // platen's `qvel[0..3]`), times dt. The friction tangent projects out the normal
            // component (rotation `ω×r` omitted, as in `coupled_trajectory_grip`).
            let v_com = &self.com_linear_jacobian() * &self.data.qvel;
            let drift = Vec3::new(v_com[0], v_com[1], v_com[2]) * dt;
            // One friction-aware dynamic soft step with the moving-collider drift.
            let bc = BoundaryConditions::new(self.pinned.clone(), Vec::new());
            let solver: SoftSolver<C> = CpuNewtonSolver::new(
                Tet4,
                self.fresh_mesh(),
                self.build_contact(height),
                self.cfg,
                bc,
            )
            .with_friction_surface_drift(drift);
            let x_next = solver
                .replay_step(
                    &Tensor::from_slice(&self.x, &[3 * n]),
                    &Tensor::from_slice(&self.v, &[3 * n]),
                    &Tensor::zeros(&[0]),
                    dt,
                )
                .x_final;
            // The friction reaction (with the same drift) at the post-step config.
            let friction = solver.friction_forces_on_soft(&x_next, &x_start, dt);
            for (vi, (&xf, &xo)) in self.v.iter_mut().zip(x_next.iter().zip(self.x.iter())) {
                *vi = (xf - xo) / dt;
            }
            self.x = x_next;
            // Route the full gripped reaction wrench [τ; f] (normal + friction + off-COM moment)
            // about the fresh-FK COM, then step the articulated body.
            self.data.xfrc_applied[self.body] = self.contact_wrench_gripped(height, &friction);
            self.data
                .step(&self.model)
                .expect("rigid step diverged in articulated grip rollout");
        }
        self.data.forward(&self.model).expect("fresh FK (output)");
        self.data.xipos[self.body]
    }

    /// One friction-aware soft step from the CURRENT coupling state with the rigid body's
    /// tangential velocity perturbed by `dv` along `dir`, returning the converged soft `x*`.
    /// The platen velocity enters the soft solve ONLY through the collider drift
    /// `Δ_surf = v_rigid·dt`, so this re-solves with `Δ_surf = (v_rigid + dv·dir)·dt`. The
    /// black-box forward oracle for finite-differencing the coupled drift Jacobian
    /// `∂x*/∂v_rigid`. Non-mutating (scratch solver; does not advance `self`).
    ///
    /// # Panics
    /// Panics if the soft solver does not converge (as in [`Self::step`]).
    #[must_use]
    pub fn coupled_step_x_at_velocity_perturbation(&self, dir: Vec3, dv: f64) -> Vec<f64> {
        let n = self.n_vertices;
        let dt = self.cfg.dt;
        let height = self.plane_height();
        let v_rigid = Vec3::new(self.data.qvel[0], self.data.qvel[1], self.data.qvel[2]) + dir * dv;
        let bc = BoundaryConditions::new(self.pinned.clone(), Vec::new());
        let solver: SoftSolver<C> = CpuNewtonSolver::new(
            Tet4,
            self.fresh_mesh(),
            self.build_contact(height),
            self.cfg,
            bc,
        )
        .with_friction_surface_drift(v_rigid * dt);
        solver
            .replay_step(
                &Tensor::from_slice(&self.x, &[3 * n]),
                &Tensor::from_slice(&self.v, &[3 * n]),
                &Tensor::zeros(&[0]),
                dt,
            )
            .x_final
    }

    /// The coupled-step **drift Jacobian** `∂x*/∂v_rigid` along `dir`: how the post-step soft
    /// equilibrium responds to the rigid body's tangential velocity, through the moving-collider
    /// grip. The velocity enters the soft solve only via the drift `Δ_surf = v_rigid·dt`, so by
    /// the chain rule `∂x*/∂v_rigid = ∂x*/∂Δ_surf · dt` — the sim-soft drift sensitivity
    /// ([`CpuNewtonSolver::equilibrium_drift_sensitivity`]) scaled by `dt`. This is the new
    /// two-way feedback edge the full friction-coupled trajectory gradient chains; here it is
    /// validated in isolation against [`Self::coupled_step_x_at_velocity_perturbation`] FD.
    /// Non-mutating. Length `n_dof`; zeros when frictionless / no active pair.
    ///
    /// # Panics
    /// Panics if the soft solver does not converge (as in [`Self::step`]).
    #[must_use]
    pub fn coupled_step_drift_jacobian(&self, dir: Vec3) -> Vec<f64> {
        let n = self.n_vertices;
        let dt = self.cfg.dt;
        let height = self.plane_height();
        let drift = Vec3::new(self.data.qvel[0], self.data.qvel[1], self.data.qvel[2]) * dt;
        let bc = BoundaryConditions::new(self.pinned.clone(), Vec::new());
        let solver: SoftSolver<C> = CpuNewtonSolver::new(
            Tet4,
            self.fresh_mesh(),
            self.build_contact(height),
            self.cfg,
            bc,
        )
        .with_friction_surface_drift(drift);
        let x_final = solver
            .replay_step(
                &Tensor::from_slice(&self.x, &[3 * n]),
                &Tensor::from_slice(&self.v, &[3 * n]),
                &Tensor::zeros(&[0]),
                dt,
            )
            .x_final;
        // ∂x*/∂v_rigid = ∂x*/∂Δ_surf · ∂Δ_surf/∂v_rigid = ∂x*/∂Δ_surf · dt.
        solver
            .equilibrium_drift_sensitivity(&x_final, &self.x, dt, dir)
            .iter()
            .map(|d| d * dt)
            .collect()
    }

    /// **The full friction-coupled trajectory gradient.** `∂(rigid final x)/∂p` for a soft
    /// material parameter `p` (`param_idx`: `0 = μ`, `1 = λ`) over an `n_steps` grip rollout
    /// — the tangential successor to [`Self::coupled_trajectory_material_gradient`] (which
    /// tracks the normal height `z`). Returns `(rigid final x, ∂x/∂p)`. Built on a fresh
    /// coupling (`with_friction`); advances `self`.
    ///
    /// The forward pass is [`Self::coupled_trajectory_grip`] (friction-aware soft step fed
    /// the collider drift `Δ_surf = v_rigid·dt`, gripped wrench routed back); the reverse
    /// tape adds, on top of the keystone z-chain, the three friction edges:
    /// - the soft step's **drift parent** (`trajectory_step_vjp_grip`'s fifth parent
    ///   `∂x*/∂Δ_surf`), fed by the `Δ_surf = vx·dt` node (`DriftFromVelVjp`);
    /// - the **tangential reaction** `fx = force_on_soft·x̂` (`FrictionReactionTrajVjp`,
    ///   parents `[x*, z, Δ_surf, x_prev]`), the friction analogue of the normal `fz` node;
    /// - the **tangential rigid carry** `vx' = vx − (Δt/m)·fx`, `x' = x + Δt·vx` (the
    ///   z-carry structs reused with `a = 1`: no contact-axis damping on the tangent, and
    ///   the constant lateral gravity drops from the derivative).
    ///
    /// This closes the two-way grip loop — `vx → Δ_surf → x* → fx → vx'` — so `∂x/∂p`
    /// flows through both the soft material stiffness and the moving-collider feedback. The
    /// normal `z`-chain is retained because the contact height sets the friction normal
    /// force `λⁿ`. Free-joint platen / engaged-contact scope, as
    /// [`Self::coupled_trajectory_grip`]. FD-gated machine-exact in
    /// `tests/friction_coupled_trajectory_gradient.rs`.
    ///
    /// # Panics
    /// Panics if `param_idx > 1`, or if the rigid step diverges / the soft solver fails to
    /// converge (surfaced loudly, as in [`Self::step`]).
    #[must_use]
    // One coherent per-step tape-construction loop (the grip forward + 9 chained nodes);
    // splitting it would scatter the recurrence's shared state. expect_used: a divergence is
    // a programmer error surfaced loudly (mirrors `step`).
    #[allow(clippy::too_many_lines, clippy::expect_used)]
    pub fn coupled_trajectory_tangential_material_gradient(
        &mut self,
        n_steps: usize,
        param_idx: usize,
    ) -> (f64, f64) {
        // Free-body friction: sphere-capable (#419) but no moving-EE centre carry yet.
        self.require_no_moving_ee();
        assert!(
            param_idx <= 1,
            "material param index {param_idx} out of range (0 = μ, 1 = λ)"
        );
        // Curvature-correct on a finite sphere (L1b-tangential): the friction reaction
        // (`FrictionReactionTrajVjp`), the soft adjoint's friction tangent, and the friction
        // pose-residual grad all carry the curved-normal term `DN·H` now (per-term machine-exact,
        // `sim_soft/tests/friction_sphere_tangent.rs`). Curvature-correct on a centroid sphere.
        let n = self.n_vertices;
        let dt = self.cfg.dt;
        let pose_dir = Vec3::new(0.0, 0.0, 1.0);
        let drift_dir = Vec3::new(1.0, 0.0, 0.0);
        // react_dir = −x̂ so the reaction node outputs `force_on_soft.x` (= −reaction.x),
        // letting the tangential carry reuse `VzCarryVjp`'s `−(Δt/m)·fx` reaction sign.
        let react_dir = Vec3::new(-1.0, 0.0, 0.0);
        // Free-body `Δt/m` (isotropic mass ⇒ the same scalar serves the tangential carry) +
        // the contact-axis damping factor `a = 1 − Δt·c/m` for the normal carry.
        let dt_over_m = self.rigid_vz_response(0.0).1;
        let a = 1.0 - self.rigid_damping * dt_over_m;

        let mut tape = Tape::new();
        let p0 = if param_idx == 0 { self.mu } else { self.lambda };
        let p_var = tape.param_tensor(Tensor::from_slice(&[p0], &[1]));
        let mut x_var = tape.constant_tensor(Tensor::from_slice(&self.x, &[3 * n]));
        let mut v_var = tape.constant_tensor(Tensor::from_slice(&self.v, &[3 * n]));
        let mut z_var =
            tape.constant_tensor(Tensor::from_slice(&[self.data.xpos[self.body].z], &[1]));
        let mut vz_var = tape.constant_tensor(Tensor::from_slice(&[self.data.qvel[2]], &[1]));
        let mut xx_var =
            tape.constant_tensor(Tensor::from_slice(&[self.data.xpos[self.body].x], &[1]));
        let mut vx_var = tape.constant_tensor(Tensor::from_slice(&[self.data.qvel[0]], &[1]));
        let mut x_final_rigid = self.data.xpos[self.body].x;

        for _ in 0..n_steps {
            self.data.forward(&self.model).expect("fresh FK");
            let height = self.plane_height();
            let vz_k = self.data.qvel[2];
            let x_start = self.x.clone();
            let drift = Vec3::new(self.data.qvel[0], self.data.qvel[1], self.data.qvel[2]) * dt;

            // Δ_surf_x = vx·dt — the differentiable tangential drift (vx at step start). Only
            // the x̂ component is differentiated: the forward solve gets the full 3-vector
            // drift, but the friction tangent projects out Δ_surf.z (⊥ the flat plane), and
            // the scene is y-symmetric (gravity gx,gz only ⇒ vy ≡ 0), so the y/z drift carry
            // no μ-sensitivity to the tracked x. (The gate is machine-exact, confirming this.)
            let drift_var = tape.push_custom(
                &[vx_var],
                Tensor::from_slice(&[drift.x], &[1]),
                Box::new(DriftFromVelVjp { dt }),
            );

            // (1) one friction-aware soft step with the moving-collider drift.
            let bc = BoundaryConditions::new(self.pinned.clone(), Vec::new());
            let solver: SoftSolver<C> = CpuNewtonSolver::new(
                Tet4,
                self.fresh_mesh(),
                self.build_contact(height),
                self.cfg,
                bc,
            )
            .with_friction_surface_drift(drift);
            let x_next = solver
                .replay_step(
                    &Tensor::from_slice(&self.x, &[3 * n]),
                    &Tensor::from_slice(&self.v, &[3 * n]),
                    &Tensor::zeros(&[0]),
                    dt,
                )
                .x_final;
            let v_next: Vec<f64> = x_next
                .iter()
                .zip(&self.x)
                .map(|(xf, xo)| (xf - xo) / dt)
                .collect();

            // Soft node x*: parents [x_prev, v_prev, p, z, Δ_surf].
            let x_next_var = tape.push_custom(
                &[x_var, v_var, p_var, z_var, drift_var],
                Tensor::from_slice(&x_next, &[3 * n]),
                Box::new(solver.trajectory_step_vjp_grip(
                    &x_next, &x_start, dt, param_idx, pose_dir, drift_dir,
                )),
            );
            let v_next_var = tape.push_custom(
                &[x_next_var, x_var],
                Tensor::from_slice(&v_next, &[3 * n]),
                Box::new(VelVjp {
                    inv_dt: 1.0 / dt,
                    n_dof: 3 * n,
                }),
            );

            // (2) NORMAL contact force fz (drives the z-chain → height → friction λⁿ).
            let positions: Vec<Vec3> = x_next
                .chunks_exact(3)
                .map(|c| Vec3::new(c[0], c[1], c[2]))
                .collect();
            let force_on_soft: Vec3 = self
                .build_contact(height)
                .pair_readout(&self.fresh_mesh(), &positions)
                .iter()
                .map(|r| r.force_on_soft)
                .sum();
            let factors = self.active_pair_force_factors(height, &positions);
            let fz_var = tape.push_custom(
                &[x_next_var, z_var],
                Tensor::from_slice(&[force_on_soft.z], &[1]),
                Box::new(ContactForceTrajVjp {
                    factors,
                    n_dof: 3 * n,
                }),
            );

            // (3) TANGENTIAL friction reaction fx = force_on_soft.x, with parents [x*, z, Δ_surf].
            let friction = solver.friction_forces_on_soft(&x_next, &x_start, dt);
            let rg = solver
                .friction_reaction_gradients(&x_next, &x_start, dt, react_dir, drift_dir, pose_dir);
            let fx_var = tape.push_custom(
                &[x_next_var, z_var, drift_var, x_var],
                Tensor::from_slice(&[rg.force], &[1]),
                Box::new(FrictionReactionTrajVjp {
                    dforce_dx: rg.dforce_dx,
                    dforce_dxprev: rg.dforce_dxprev,
                    dforce_dheight: rg.dforce_dheight,
                    dforce_ddrift: rg.dforce_ddrift,
                    dforce_dmu_c: None,
                    n_dof: 3 * n,
                }),
            );

            // Advance the soft state BEFORE the wrench — `contact_wrench_gripped` reads
            // `self.positions()` for the post-step moment arms and normal reaction, exactly
            // as `coupled_trajectory_grip` does (the forward must match bit-for-bit).
            self.v = v_next;
            self.x = x_next;

            // (4) route the gripped wrench (normal + friction + moment) + z-damping; real step.
            let mut wrench = self.contact_wrench_gripped(height, &friction);
            wrench[5] -= self.rigid_damping * vz_k;
            self.data.xfrc_applied[self.body] = wrench;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in tangential trajectory");
            // Fresh FK so xpos reflects the freshly-integrated qpos (sim-core's `step` leaves
            // xpos at the PRE-step pose), matching `coupled_trajectory_grip`'s refreshed pose.
            self.data
                .forward(&self.model)
                .expect("fresh FK (post-step)");
            let z_next = self.data.xpos[self.body].z;
            let vz_next = self.data.qvel[2];
            let x_next_rigid = self.data.xpos[self.body].x;
            let vx_next = self.data.qvel[0];

            // (5) rigid carries — the velocity halves (semi-implicit Euler), THEN the position
            // halves integrated with the POST-step velocity. sim-core integrates `qpos` with the
            // UPDATED `qvel` (`x_{k+1} = x_k + Δt·v_{k+1}`), so under the fresh-FK convention the
            // position carry's velocity parent is the freshly-updated `v'`, NOT the pre-step `v`
            // (the opposite of the stale-FK keystone z-tape). No contact-axis damping on the
            // tangent (`a = 1`) and the constant lateral gravity drops from the derivative.
            let vz_next_var = tape.push_custom(
                &[vz_var, fz_var],
                Tensor::from_slice(&[vz_next], &[1]),
                Box::new(VzCarryVjp {
                    a,
                    neg_dt_over_m: -dt_over_m,
                }),
            );
            let vx_next_var = tape.push_custom(
                &[vx_var, fx_var],
                Tensor::from_slice(&[vx_next], &[1]),
                Box::new(VzCarryVjp {
                    a: 1.0,
                    neg_dt_over_m: -dt_over_m,
                }),
            );
            let z_next_var = tape.push_custom(
                &[z_var, vz_next_var],
                Tensor::from_slice(&[z_next], &[1]),
                Box::new(ZCarryVjp { dt }),
            );
            let xx_next_var = tape.push_custom(
                &[xx_var, vx_next_var],
                Tensor::from_slice(&[x_next_rigid], &[1]),
                Box::new(ZCarryVjp { dt }),
            );

            // Advance the tape handles (the soft state was advanced before the wrench).
            x_var = x_next_var;
            v_var = v_next_var;
            z_var = z_next_var;
            vz_var = vz_next_var;
            xx_var = xx_next_var;
            vx_var = vx_next_var;
            x_final_rigid = x_next_rigid;
        }

        tape.backward(xx_var);
        let grad = tape.grad_tensor(p_var).as_slice()[0];
        (x_final_rigid, grad)
    }

    /// The coupled tangential-trajectory gradient w.r.t. the Coulomb friction COEFFICIENT `μ_c`:
    /// `(platen x after `n_steps`, ∂(platen x)/∂μ_c)`. Mirror of
    /// [`Self::coupled_trajectory_tangential_material_gradient`] — same nine-node-per-step tape,
    /// same fresh-FK timing, same drift/reaction/carry — differing ONLY in the two places `μ_c`
    /// enters and a material parameter does not:
    ///
    /// 1. **Through `x*`** (the soft equilibrium): the soft node uses
    ///    [`CpuNewtonSolver::trajectory_step_vjp_grip_fric_coeff`], routing `∂r/∂μ_c = ∇D/μ_c`
    ///    into the generic param slot. This lever is TINY in deep slip (`x*` barely moves).
    /// 2. **Directly through the reaction** `fx = (Σ μ_c·λⁿ·…)·react_dir`: `μ_c` is an extra
    ///    parent on the friction-reaction node with `∂fx/∂μ_c = fx/μ_c`. This DOMINATES — in the
    ///    Coulomb-saturated regime `fx ≈ μ_c·λⁿ`, so `∂fx/∂μ_c = λⁿ` is the large, direct term.
    ///
    /// The single `μ_c` param leaf is parent to BOTH nodes; the tape sums the two channels. The
    /// gradient is machine-exact; the FD validation uses a COMPLIANT block (like the material
    /// trajectory gradient) — on a stiff block the FD re-solve's friction-term cancellation
    /// (`∇²D ~ 1e4`) floors the FD-vs-analytic agreement at ~3e-4 for `μ_c` and the material
    /// parameter alike (the linear-lever stiff advantage is for the soft-only `∂x*/∂μ_c`, not
    /// the coupled FD oracle). Requires friction active (`self.cfg.friction_mu > 0`).
    ///
    /// # Panics
    /// Panics if friction is inactive (`self.cfg.friction_mu ≤ 0`), or if the rigid step
    /// diverges / the soft solver fails to converge (surfaced loudly, as in [`Self::step`]).
    #[must_use]
    // One coherent per-step tape-construction loop (the grip forward + 9 chained nodes);
    // splitting it would scatter the recurrence's shared state. expect_used: a divergence is
    // a programmer error surfaced loudly (mirrors `step`).
    #[allow(clippy::too_many_lines, clippy::expect_used)]
    pub fn coupled_trajectory_tangential_friction_coeff_gradient(
        &mut self,
        n_steps: usize,
    ) -> (f64, f64) {
        // Free-body friction coeff: sphere-capable (#419) but no moving-EE centre carry yet.
        self.require_no_moving_ee();
        let mu_c = self.cfg.friction_mu;
        assert!(
            mu_c > 0.0,
            "∂/∂μ_c requires friction active (cfg.friction_mu = {mu_c} ≤ 0)"
        );
        // Curvature-correct on a finite sphere (L1b-tangential) — see the material-gradient
        // sibling; the friction adjoints all carry the curved-normal `DN·H` term. No guard.
        let n = self.n_vertices;
        let dt = self.cfg.dt;
        let pose_dir = Vec3::new(0.0, 0.0, 1.0);
        let drift_dir = Vec3::new(1.0, 0.0, 0.0);
        let react_dir = Vec3::new(-1.0, 0.0, 0.0);
        let dt_over_m = self.rigid_vz_response(0.0).1;
        let a = 1.0 - self.rigid_damping * dt_over_m;

        let mut tape = Tape::new();
        // The gradient target: the friction coefficient (seeded from the linearization point).
        let p_var = tape.param_tensor(Tensor::from_slice(&[mu_c], &[1]));
        let mut x_var = tape.constant_tensor(Tensor::from_slice(&self.x, &[3 * n]));
        let mut v_var = tape.constant_tensor(Tensor::from_slice(&self.v, &[3 * n]));
        let mut z_var =
            tape.constant_tensor(Tensor::from_slice(&[self.data.xpos[self.body].z], &[1]));
        let mut vz_var = tape.constant_tensor(Tensor::from_slice(&[self.data.qvel[2]], &[1]));
        let mut xx_var =
            tape.constant_tensor(Tensor::from_slice(&[self.data.xpos[self.body].x], &[1]));
        let mut vx_var = tape.constant_tensor(Tensor::from_slice(&[self.data.qvel[0]], &[1]));
        let mut x_final_rigid = self.data.xpos[self.body].x;

        for _ in 0..n_steps {
            self.data.forward(&self.model).expect("fresh FK");
            let height = self.plane_height();
            let vz_k = self.data.qvel[2];
            let x_start = self.x.clone();
            let drift = Vec3::new(self.data.qvel[0], self.data.qvel[1], self.data.qvel[2]) * dt;

            let drift_var = tape.push_custom(
                &[vx_var],
                Tensor::from_slice(&[drift.x], &[1]),
                Box::new(DriftFromVelVjp { dt }),
            );

            // (1) one friction-aware soft step with the moving-collider drift.
            let bc = BoundaryConditions::new(self.pinned.clone(), Vec::new());
            let solver: SoftSolver<C> = CpuNewtonSolver::new(
                Tet4,
                self.fresh_mesh(),
                self.build_contact(height),
                self.cfg,
                bc,
            )
            .with_friction_surface_drift(drift);
            let x_next = solver
                .replay_step(
                    &Tensor::from_slice(&self.x, &[3 * n]),
                    &Tensor::from_slice(&self.v, &[3 * n]),
                    &Tensor::zeros(&[0]),
                    dt,
                )
                .x_final;
            let v_next: Vec<f64> = x_next
                .iter()
                .zip(&self.x)
                .map(|(xf, xo)| (xf - xo) / dt)
                .collect();

            // Channel 1 — soft node x*: parents [x_prev, v_prev, μ_c, z, Δ_surf], param slot = μ_c.
            let x_next_var = tape.push_custom(
                &[x_var, v_var, p_var, z_var, drift_var],
                Tensor::from_slice(&x_next, &[3 * n]),
                Box::new(solver.trajectory_step_vjp_grip_fric_coeff(
                    &x_next, &x_start, dt, pose_dir, drift_dir,
                )),
            );
            let v_next_var = tape.push_custom(
                &[x_next_var, x_var],
                Tensor::from_slice(&v_next, &[3 * n]),
                Box::new(VelVjp {
                    inv_dt: 1.0 / dt,
                    n_dof: 3 * n,
                }),
            );

            // (2) NORMAL contact force fz — frictionless, NO μ_c parent.
            let positions: Vec<Vec3> = x_next
                .chunks_exact(3)
                .map(|c| Vec3::new(c[0], c[1], c[2]))
                .collect();
            let force_on_soft: Vec3 = self
                .build_contact(height)
                .pair_readout(&self.fresh_mesh(), &positions)
                .iter()
                .map(|r| r.force_on_soft)
                .sum();
            let factors = self.active_pair_force_factors(height, &positions);
            let fz_var = tape.push_custom(
                &[x_next_var, z_var],
                Tensor::from_slice(&[force_on_soft.z], &[1]),
                Box::new(ContactForceTrajVjp {
                    factors,
                    n_dof: 3 * n,
                }),
            );

            // (3) TANGENTIAL friction reaction fx = force_on_soft.x. Channel 2 — μ_c is a DIRECT
            // parent here (`∂fx/∂μ_c = fx/μ_c`, the dominant term), on top of [x*, z, Δ_surf, x_prev].
            let friction = solver.friction_forces_on_soft(&x_next, &x_start, dt);
            let rg = solver
                .friction_reaction_gradients(&x_next, &x_start, dt, react_dir, drift_dir, pose_dir);
            let fx_var = tape.push_custom(
                &[x_next_var, z_var, drift_var, x_var, p_var],
                Tensor::from_slice(&[rg.force], &[1]),
                Box::new(FrictionReactionTrajVjp {
                    dforce_dx: rg.dforce_dx,
                    dforce_dxprev: rg.dforce_dxprev,
                    dforce_dheight: rg.dforce_dheight,
                    dforce_ddrift: rg.dforce_ddrift,
                    dforce_dmu_c: Some(rg.force / mu_c),
                    n_dof: 3 * n,
                }),
            );

            self.v = v_next;
            self.x = x_next;

            // (4) route the gripped wrench (normal + friction + moment) + z-damping; real step.
            let mut wrench = self.contact_wrench_gripped(height, &friction);
            wrench[5] -= self.rigid_damping * vz_k;
            self.data.xfrc_applied[self.body] = wrench;
            self.data
                .step(&self.model)
                .expect("rigid step diverged in tangential trajectory");
            self.data
                .forward(&self.model)
                .expect("fresh FK (post-step)");
            let z_next = self.data.xpos[self.body].z;
            let vz_next = self.data.qvel[2];
            let x_next_rigid = self.data.xpos[self.body].x;
            let vx_next = self.data.qvel[0];

            // (5) rigid carries (semi-implicit Euler), identical to the material driver.
            let vz_next_var = tape.push_custom(
                &[vz_var, fz_var],
                Tensor::from_slice(&[vz_next], &[1]),
                Box::new(VzCarryVjp {
                    a,
                    neg_dt_over_m: -dt_over_m,
                }),
            );
            let vx_next_var = tape.push_custom(
                &[vx_var, fx_var],
                Tensor::from_slice(&[vx_next], &[1]),
                Box::new(VzCarryVjp {
                    a: 1.0,
                    neg_dt_over_m: -dt_over_m,
                }),
            );
            let z_next_var = tape.push_custom(
                &[z_var, vz_next_var],
                Tensor::from_slice(&[z_next], &[1]),
                Box::new(ZCarryVjp { dt }),
            );
            let xx_next_var = tape.push_custom(
                &[xx_var, vx_next_var],
                Tensor::from_slice(&[x_next_rigid], &[1]),
                Box::new(ZCarryVjp { dt }),
            );

            x_var = x_next_var;
            v_var = v_next_var;
            z_var = z_next_var;
            vz_var = vz_next_var;
            xx_var = xx_next_var;
            vx_var = vx_next_var;
            x_final_rigid = x_next_rigid;
        }

        tape.backward(xx_var);
        let grad = tape.grad_tensor(p_var).as_slice()[0];
        (x_final_rigid, grad)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_mjcf::load_model;
    use sim_ml_chassis::autograd::VjpOp;

    /// [`right_jacobian_so3`] is the exact tangent map of the quaternion exp-map step,
    /// in the "output tangent at the nominal `q'`" convention: for `q' = q ⊕ exp(φ)`,
    /// perturbing `φ → φ + δ` gives `log(q'⁻¹ · q'(φ+δ)) = J_r(φ)·δ`. FD-validated against
    /// the real SO(3) integration at several angles (small, moderate, large).
    #[test]
    fn right_jacobian_so3_matches_quaternion_expmap_fd() {
        use sim_core::UnitQuaternion;
        // exp-map of a rotation vector → unit quaternion (right tangent).
        let expq = |v: Vec3| -> UnitQuaternion<f64> {
            let a = v.norm();
            if a < 1e-12 {
                UnitQuaternion::identity()
            } else {
                UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(v), a)
            }
        };
        for phi in [
            Vec3::new(1e-7, 0.0, 0.0),    // near-zero (small-angle branch)
            Vec3::new(0.03, -0.02, 0.05), // moderate
            Vec3::new(0.8, -0.6, 0.4),    // large
        ] {
            let jr = right_jacobian_so3(phi);
            let qp = expq(phi); // the nominal q' (base q = identity, WLOG for the tangent map)
            let eps = 1e-6;
            for k in 0..3 {
                let mut d = Vec3::zeros();
                d[k] = eps;
                // log(q'⁻¹ · exp(φ+δ)) and · exp(φ−δ), central difference → column k.
                let plus = qp.inverse() * expq(phi + d);
                let minus = qp.inverse() * expq(phi - d);
                let logv = |q: UnitQuaternion<f64>| -> Vec3 {
                    let v = Vec3::new(q.i, q.j, q.k);
                    let s = v.norm();
                    if s < 1e-12 {
                        Vec3::zeros()
                    } else {
                        v * (2.0 * s.atan2(q.w) / s)
                    }
                };
                let col_fd = (logv(plus) - logv(minus)) / (2.0 * eps);
                let col_an = jr.column(k);
                let err = (col_fd - col_an).norm();
                assert!(
                    err < 1e-5,
                    "J_r column {k} at φ={phi:?}: analytic {col_an:?} != FD {col_fd:?} (err {err:.3e})"
                );
            }
        }
    }

    /// The Vjp ops scatter their PRECOMPUTED per-pair factors `(vertex_id, ∂fz/∂x_v,
    /// ∂fz/∂height)` straight through: `ContactForceVjp` scatters `∂fz/∂x_v` onto x*, and
    /// `ContactForceTrajVjp` additionally sums `∂fz/∂height` onto its z parent. Distinct
    /// per-pair values verify both ops route the factors correctly (the curvature-aware
    /// generalization beyond the constant-`κ` penalty path the keystone gates exercise);
    /// the flat plane values `∂fz/∂x_v = (0,0,−cᵥ)`, `∂fz/∂height = cᵥ` here mirror what
    /// `active_pair_force_factors` produces for `n̂ = −ẑ`, `H = 0`.
    #[test]
    fn contact_force_factors_scatter_per_pair() {
        // Precomputed factors for two pairs with distinct stiffness cᵥ = 2, 5 on the flat
        // plane: ∂fz/∂x_v = (0,0,−cᵥ), ∂fz/∂height = cᵥ.
        let factors = vec![
            (0_usize, Vec3::new(0.0, 0.0, -2.0), 2.0),
            (1_usize, Vec3::new(0.0, 0.0, -5.0), 5.0),
        ];
        // ContactForceVjp scatters ∂fz/∂x_v (cot=1): z-components −2, −5.
        let mut parent = vec![Tensor::zeros(&[6])];
        ContactForceVjp::new(factors.clone()).vjp(&Tensor::from_slice(&[1.0], &[1]), &mut parent);
        let g = parent[0].as_slice();
        assert!(
            (g[2] + 2.0).abs() < 1e-12 && (g[5] + 5.0).abs() < 1e-12,
            "∂fz/∂x* should carry the per-pair ∂fz/∂x_v factor, got {g:?}"
        );
        // ContactForceTrajVjp: ∂fz/∂z = Σ ∂fz/∂height = 7.
        let traj = ContactForceTrajVjp { factors, n_dof: 6 };
        let mut parents = vec![Tensor::zeros(&[6]), Tensor::zeros(&[1])];
        traj.vjp(&Tensor::from_slice(&[1.0], &[1]), &mut parents);
        assert!(
            (parents[1].as_slice()[0] - 7.0).abs() < 1e-12,
            "∂fz/∂z should be Σ ∂fz/∂height = 7, got {}",
            parents[1].as_slice()[0]
        );
    }

    const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.125">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

    // `.expect` surfaces a malformed fixture as a test panic — the canonical
    // fixture idiom; the coupling itself is graded by the integration gate.
    #[allow(clippy::expect_used)]
    fn coupling() -> StaggeredCoupling {
        let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
        let mut data = model.make_data();
        data.forward(&model).expect("initial forward");
        StaggeredCoupling::new(
            model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 8.0,
        )
    }

    #[test]
    fn coupled_step_is_bounded_and_engages_contact() {
        let mut c = coupling();
        let z0 = c.data().xpos[1].z;
        let mut last = c.step();
        for _ in 0..150 {
            last = c.step();
        }
        // platen descended under gravity, loop stayed finite + bounded.
        assert!(
            last.rigid_z.is_finite() && last.rigid_z < z0 && last.rigid_z > 0.0,
            "platen z out of range: {} (z0={z0})",
            last.rigid_z
        );
        // contact has engaged: a nonzero upward reaction on the platen.
        assert!(
            -last.force_on_soft.z > 0.0,
            "expected an upward contact reaction once engaged; got {:?}",
            last.force_on_soft
        );
    }

    /// Lib-level smoke test of the differentiability probes (the scientific
    /// FD validation lives in the `tests/` gates `contact_force_jacobian`,
    /// `coupled_step_jacobian`, `coupled_total_jacobian`). Exercises every
    /// probe at a deeply-engaged height (the rest top face is at z=0.1; the
    /// plane at h=0.099 penetrates ~1 mm so the 25 top vertices are active)
    /// and pins the qualitative relationships the gates quantify.
    #[test]
    fn differentiability_probes_are_finite_and_consistent() {
        let c = coupling();
        let h = 0.099;

        // S1 explicit (fixed-soft-position) factor — `+κ·N_active·ẑ`, positive.
        let explicit = c.contact_force_height_jacobian(h);
        assert!(
            explicit.z > 0.0 && explicit.z.is_finite(),
            "explicit ∂force/∂h.z should be +κ·N > 0, got {explicit:?}"
        );

        // Forward building blocks: fixed-position vs re-solved contact force.
        let f_fixed = c.contact_force_at_height(h);
        let f_resolved = c.resolved_contact_force(h);
        assert!(
            f_fixed.z != 0.0 && f_resolved.z.is_finite(),
            "engaged contact force should be nonzero/finite: fixed {f_fixed:?}, resolved {f_resolved:?}"
        );

        // S2 rigid factor probe: a free body accelerates under an applied force.
        let (z_up, vz_up) = c.rigid_step_probe(1.0);
        let (_z_dn, vz_dn) = c.rigid_step_probe(-1.0);
        assert!(
            z_up.is_finite() && vz_up > vz_dn,
            "more upward force ⇒ higher next vz: vz(+1)={vz_up}, vz(-1)={vz_dn}"
        );

        // S3 total factor: the implicit soft re-equilibration reduces the
        // explicit-only sensitivity (the soft body follows the rising plane).
        let total = c.contact_force_height_total_jacobian(h);
        assert!(
            total.z.is_finite() && total.z.abs() < explicit.z.abs(),
            "total ∂force/∂h.z ({}) should be finite and smaller in magnitude \
             than the explicit-only partial ({}) — implicit re-equilibration",
            total.z,
            explicit.z,
        );
    }

    /// Lib-level smoke test of the S4 cross-engine tape crossing (the scientific
    /// FD validation is in `tests/{rigid_step_vjp,coupled_load_gradient}.rs`).
    /// Exercises `rigid_vz_response`, `coupled_step_load_gradient` (one
    /// `tape.backward` across both engines), and the forward oracle
    /// `coupled_step_load_vz` at a deeply-engaged height with a loaded top face.
    #[test]
    fn cross_engine_crossing_smoke() {
        let c = coupling();
        let mesh = HandBuiltTetMesh::uniform_block(4, 0.1, &MaterialField::uniform(3.0e4, 1.2e5));
        let loaded: Vec<VertexId> =
            sim_soft::pick_vertices_by_predicate(&mesh, |p| (p.z - 0.1).abs() < 1e-9);
        assert!(!loaded.is_empty());

        // Rigid response factor is the free-body dt/m (= 5e-3).
        let (_vz, dvz_dfz) = c.rigid_vz_response(5.0);
        assert!(
            (dvz_dfz - 1.0e-3 / 0.2).abs() < 1e-9,
            "∂vz'/∂fz should be dt/m"
        );

        let h = 0.099;
        let theta0 = 5.0;
        let (vz, grad) = c.coupled_step_load_gradient(h, &loaded, theta0);
        assert!(vz.is_finite() && grad.is_finite());
        // The cross-engine gradient is nonzero (load couples through to vz')
        // and consistent with the forward oracle's local slope sign.
        assert!(
            grad.abs() > 1e-4,
            "expected a nonzero cross-engine gradient"
        );
        let eps = 1.0e-4;
        let fd = (c.coupled_step_load_vz(h, &loaded, theta0 + eps)
            - c.coupled_step_load_vz(h, &loaded, theta0 - eps))
            / (2.0 * eps);
        assert!(
            (grad - fd).abs() / fd.abs() < 1e-6,
            "smoke: tape grad {grad} vs FD {fd}"
        );
    }

    /// Lib-level smoke test of the multi-step time-adjoint (the scientific FD
    /// validation is in `tests/coupled_trajectory_gradient.rs`): one
    /// `tape.backward` over a coupled rollout that crosses the contact make event
    /// gives a finite gradient, and the tape's forward rollout reproduces the
    /// real `step` dynamics exactly.
    #[test]
    fn trajectory_gradient_smoke() {
        // Independent couplings: one for the tape gradient, one for the real
        // reference rollout (the gradient call advances its own coupling).
        let (z_tape, grad) = coupling().coupled_trajectory_material_gradient(80, 0);
        let mut c_ref = coupling();
        let mut z_ref = c_ref.data().xpos[1].z;
        for _ in 0..80 {
            z_ref = c_ref.step().rigid_z;
        }
        assert!(
            (z_tape - z_ref).abs() < 1e-12,
            "tape forward z_N {z_tape} != real rollout {z_ref}"
        );
        // Past the make event the gradient is finite and nonzero (the block
        // stiffness bears the platen).
        assert!(
            grad.is_finite() && grad.abs() > 1e-12,
            "expected a finite nonzero ∂z_N/∂μ"
        );
    }

    /// Lib-level smoke test of the friction-coefficient trajectory gradient (the
    /// scientific FD validation is in `tests/friction_coupled_trajectory_coeff_gradient.rs`):
    /// the `μ_c` driver's tape forward reproduces the real grip rollout exactly, and one
    /// `tape.backward` gives a finite nonzero `∂x_N/∂μ_c` through both channels (the soft
    /// equilibrium shift and the direct reaction `∂fx/∂μ_c`). Uses a tilted-gravity,
    /// contact-engaged scene (the default `coupling()` has no lateral drive ⇒ no grip slip).
    #[test]
    #[allow(clippy::expect_used)]
    fn friction_coeff_trajectory_gradient_smoke() {
        // Tilted gravity + a platen started engaged (z = 0.115) so the grip slides.
        const GRIP_MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.115">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
        let build = || -> StaggeredCoupling {
            let model = load_model(GRIP_MJCF).expect("grip MJCF loads");
            let mut data = model.make_data();
            data.forward(&model).expect("initial forward");
            StaggeredCoupling::new(
                model, data, 1, 0.005, 4, 0.1, 3.0e3, 1.0e-3, 3.0e4, 1.0e-2, 8.0,
            )
            .with_friction(2.5, 0.1)
        };
        let (x_tape, grad) = build().coupled_trajectory_tangential_friction_coeff_gradient(20);
        let x_ref = build().coupled_trajectory_grip(20).x;
        assert!(
            (x_tape - x_ref).abs() < 1e-12,
            "tape forward x_N {x_tape} != real grip rollout {x_ref}"
        );
        assert!(
            grad.is_finite() && grad.abs() > 1e-12,
            "expected a finite nonzero ∂x_N/∂μ_c"
        );
    }

    /// Lib-level smoke test of the S5 co-design crossing `∂vz'/∂μ` (the
    /// scientific FD validation is in `tests/coupled_material_gradient.rs`):
    /// the material parameter rides the same crossing via `MaterialStepVjp`.
    #[test]
    fn material_crossing_smoke() {
        let c = coupling();
        let h = 0.099;
        let (vz, grad_mu) = c.coupled_step_material_gradient(h, 0);
        assert!(vz.is_finite() && grad_mu.is_finite());
        assert!(grad_mu.abs() > 1e-9, "expected a nonzero ∂vz'/∂μ");
        let eps = 3.0e4 * 1e-6;
        let fd = (c.coupled_step_material_vz(h, 0, 3.0e4 + eps)
            - c.coupled_step_material_vz(h, 0, 3.0e4 - eps))
            / (2.0 * eps);
        assert!(
            (grad_mu - fd).abs() / fd.abs() < 1e-5,
            "smoke: material tape grad {grad_mu} vs FD {fd}"
        );
    }

    /// Lib-level smoke test of the multi-DOF rigid factor `rigid_xfrc_column`
    /// (the scientific FD validation — hinge / 2-link / free-body — is in
    /// `tests/rigid_multidof_response.rs`): on the free platen the column reduces
    /// to the scalar `dt/m` on the contact axis, tying the matrix factor to the
    /// merged `rigid_vz_response`.
    #[test]
    fn rigid_xfrc_column_free_body_smoke() {
        let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        let col = rigid_xfrc_column(&model, &data, 1);
        assert_eq!(col.shape(), (6, 6)); // free joint: nv=6, 6 spatial-force columns
        // ∂vz'/∂f_z (qvel[2] vs xfrc[5]) is the free-body dt/m (read from the model,
        // not hardcoded, so it can't go stale if the fixture changes).
        let dt_over_m = model.timestep / model.body_mass[1];
        assert!(
            (col[(2, 5)] - dt_over_m).abs() < 1e-12,
            "free-body ∂vz'/∂f_z must be dt/m, got {}",
            col[(2, 5)]
        );
    }

    /// Lib-level smoke of the multi-DOF (articulated) coupled trajectory gradient
    /// (the scientific hinge FD validation is in
    /// `tests/articulated_trajectory_gradient.rs`). Exercises the full articulated
    /// path — the `PoseSeamVjp`, the loaded `RigidStateCarryVjp`, and the forward
    /// oracle — on a free-joint body (nv = 6, `rigid_damping = 0`): the tape forward
    /// `tip_z_N` reproduces the real rollout and the gradient is finite.
    #[test]
    fn articulated_trajectory_smoke() {
        // A free-joint platen with rigid_damping = 0 (the articulated path's scope).
        let build = || {
            let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
            let mut data = model.make_data();
            data.forward(&model).expect("forward");
            StaggeredCoupling::<PenaltyRigidContact>::new(
                model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
            )
        };
        let (tip_z, grad) = build().coupled_trajectory_material_gradient_articulated(5, 0);
        assert!(tip_z.is_finite() && grad.is_finite());
        let z_ref = build().coupled_trajectory_articulated_z(5);
        assert!(
            (tip_z - z_ref).abs() < 1e-12,
            "tape forward tip_z {tip_z} != real rollout {z_ref}"
        );
    }

    /// The analytic single-hinge state Jacobian ([`StaggeredCoupling::analytic_state_jacobian`])
    /// is MACHINE-EXACT against the FD [`StaggeredCoupling::loaded_state_jacobian`] for an
    /// arbitrary held wrench (force AND torque components, nonzero `qvel`) — pinning the
    /// closed-form geometric stiffness `Δt·M⁻¹·∂(Jᵀw)/∂q = Δt·M⁻¹·(â×(â×r))·f` added to the
    /// unloaded transition `A`. It also confirms the scope predicate: `single_hinge` selects
    /// the hinge and rejects the free joint (which falls back to the FD form, J = I ⇒ no
    /// geometric stiffness). This replaces the FD Jacobian's noise with the exact term; see
    /// `docs/keystone/geometric_stiffness_recon.md`.
    #[test]
    fn analytic_state_jacobian_matches_fd_loaded() {
        const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
        let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
        let mut data = model.make_data();
        data.qpos[0] = 0.3;
        data.qvel[0] = -1.5; // nonzero velocity exercises the full Jacobian
        data.forward(&model).expect("forward");
        let c: StaggeredCoupling = StaggeredCoupling::new(
            model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
        );
        assert!(
            c.single_hinge().is_some(),
            "hinge must select the analytic path"
        );

        for wrench in [
            SpatialVector::from_row_slice(&[0.1, 0.2, 0.0, 5.0, -3.0, 800.0]),
            SpatialVector::from_row_slice(&[0.0, 0.0, 0.0, 0.0, 0.0, 600.0]),
            SpatialVector::from_row_slice(&[0.3, -0.4, 0.2, -10.0, 7.0, -250.0]),
        ] {
            let loaded = c.loaded_state_jacobian(&wrench);
            let analytic = c
                .analytic_state_jacobian(&wrench)
                .expect("single hinge → analytic");
            let (err, loc) = sim_core::max_relative_error(&loaded, &analytic, 1e-3);
            assert!(
                err < 1e-6,
                "analytic J_state must match FD loaded to FD precision, got rel {err:.3e} at {loc:?}"
            );
        }

        // Free-joint platen: no single hinge ⇒ the analytic path declines (FD fallback).
        let pmodel = load_model(PLATEN_MJCF).expect("platen MJCF loads");
        let mut pdata = pmodel.make_data();
        pdata.forward(&pmodel).expect("forward");
        let pc: StaggeredCoupling = StaggeredCoupling::new(
            pmodel, pdata, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
        );
        assert!(
            pc.single_hinge().is_none()
                && pc
                    .analytic_state_jacobian(&SpatialVector::zeros())
                    .is_none(),
            "free joint must fall back to the FD loaded Jacobian"
        );
    }

    /// Lib smoke for the articulated FRICTION-grip path: the full gripped forward oracle and the
    /// matrix-carry friction material gradient (exercising the per-vertex friction wrench node,
    /// the articulated drift node, and sim-soft's `friction_force_jacobians`). Machine-exact
    /// accuracy lives in the `friction_articulated_material_gradient` integration test; this
    /// keeps the `--lib` coverage honest over the new public surface.
    #[test]
    fn articulated_friction_grip_smoke() {
        const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
        let build = || -> StaggeredCoupling {
            let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
            let mut data = model.make_data();
            data.qpos[0] = 0.3;
            data.forward(&model).expect("forward");
            StaggeredCoupling::new(
                model, data, 1, 0.005, 4, 0.1, 3.0e3, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
            )
            .with_friction(2.5, 0.1)
        };
        let tip = build().coupled_trajectory_gripped_articulated(3);
        assert!(
            tip.x.is_finite() && (0.10..0.12).contains(&tip.z),
            "gripped articulated rollout must stay finite and engaged, got {tip:?}"
        );
        let (tip_x, grad) =
            build().coupled_trajectory_tangential_material_gradient_articulated(3, 0);
        assert!(
            tip_x.is_finite() && grad.is_finite(),
            "articulated friction material gradient must be finite, got ({tip_x}, {grad})"
        );
        let (cx, cgrad) =
            build().coupled_trajectory_tangential_friction_coeff_gradient_articulated(3);
        assert!(
            cx.is_finite() && cgrad.is_finite(),
            "articulated friction-coeff gradient must be finite, got ({cx}, {cgrad})"
        );
    }

    /// Lib smoke for the actuator CONTROL gradient through the friction grip: the gripped-actuated
    /// forward oracle and the control gradient (exercising the friction wrench node + the actuator
    /// `g_act` channel on one tape). Machine-exact accuracy lives in the
    /// `actuator_friction_gradient` integration test; this keeps `--lib` coverage honest.
    #[test]
    fn actuator_friction_grip_smoke() {
        const MOTOR_MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
  <actuator><motor joint="j" gear="1"/></actuator>
</mujoco>"#;
        let build = || -> StaggeredCoupling {
            let model = load_model(MOTOR_MJCF).expect("motor MJCF loads");
            let mut data = model.make_data();
            data.qpos[0] = 0.3;
            data.forward(&model).expect("forward");
            StaggeredCoupling::new(
                model, data, 1, 0.005, 4, 0.1, 3.0e3, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
            )
            .with_friction(2.5, 0.1)
        };
        let controls = [0.03_f64; 3];
        let tip = build().coupled_trajectory_actuated_gripped_x(&controls);
        assert!(
            tip.x.is_finite() && (0.10..0.12).contains(&tip.z),
            "gripped-actuated rollout must stay finite and engaged, got {tip:?}"
        );
        let (tx, grad) = build().coupled_trajectory_actuator_friction_gradient(&controls);
        assert!(
            tx.is_finite() && grad.iter().all(|g| g.is_finite()),
            "actuator-friction gradient must be finite, got ({tx}, {grad:?})"
        );
        // Closed-loop policy gradient through friction (backprop-through-time).
        let params = [0.08, -0.02, 0.01];
        let pol = build().coupled_trajectory_policy_gripped_x(&LinearFeedback, &params, 3);
        assert!(
            pol.x.is_finite() && (0.10..0.12).contains(&pol.z),
            "policy-gripped rollout must stay finite and engaged, got {pol:?}"
        );
        // The frame-capturing sibling: byte-identical tip_x + n_steps+1 frames (the R5 viz
        // surface). Accuracy/identity gates live in `tests/grip_rollout_capture.rs`.
        let (cap_tip, frames) =
            build().coupled_trajectory_policy_gripped_capture(&LinearFeedback, &params, 3);
        assert!(
            cap_tip.x == pol.x && frames.len() == 4,
            "capture must byte-match the scalar oracle and yield n_steps+1 frames, \
             got tip {} vs {} / {} frames",
            cap_tip.x,
            pol.x,
            frames.len()
        );
        let (px, pgrad) =
            build().coupled_trajectory_policy_friction_gradient(&LinearFeedback, &params, 3);
        assert!(
            px.is_finite() && pgrad.len() == 3 && pgrad.iter().all(|g| g.is_finite()),
            "policy-friction gradient must be finite, got ({px}, {pgrad:?})"
        );
        // Design + policy on ONE friction-grip tape: the material μ leaf AND the policy θ leaves,
        // both differentiated in one backward.
        let (dx, dmu, dgrad) =
            build().coupled_trajectory_design_policy_friction_gradient(&LinearFeedback, &params, 3);
        assert!(
            dx.is_finite()
                && dmu.is_finite()
                && dgrad.len() == 3
                && dgrad.iter().all(|g| g.is_finite()),
            "design+policy-friction gradient must be finite, got ({dx}, {dmu}, {dgrad:?})"
        );
        // Trajectory-integrated HOLDING gradient (Σ (qₖ − q_hold)²) on the SAME shared tape —
        // accuracy is gated in `design_policy_hold_gradient.rs`; this keeps --lib coverage honest.
        let (lcost, lmu, lgrad) = build().coupled_trajectory_design_policy_hold_gradient(
            &LinearFeedback,
            &params,
            3,
            0.25,
        );
        assert!(
            lcost.is_finite()
                && lmu.is_finite()
                && lgrad.len() == 3
                && lgrad.iter().all(|g| g.is_finite()),
            "holding gradient must be finite, got ({lcost}, {lmu}, {lgrad:?})"
        );
    }

    /// A DAMPED single hinge's analytic `J_state` matches the FD `loaded_state_jacobian` to the
    /// FD floor: the `M → M_impl = M + Δt·D` correction (rescale `A`'s bare-`M` velocity rows by
    /// `M/M_impl`, geom-stiff over `M_impl`, position rows from `θ' = θ + Δt·ω'`) reconciles the
    /// unloaded `A` with the real eulerdamp step. A nonzero `qvel` exercises the damping term.
    #[test]
    fn analytic_state_jacobian_damped_matches_fd() {
        const DAMPED_HINGE: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0" damping="0.7"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
        let model = load_model(DAMPED_HINGE).expect("damped hinge loads");
        let mut data = model.make_data();
        data.qpos[0] = 0.3;
        data.qvel[0] = 0.4; // nonzero velocity so the implicit-damping term is exercised
        data.forward(&model).expect("forward");
        assert!(model.implicit_damping[0] > 0.0, "damping must be live");
        let c: StaggeredCoupling = StaggeredCoupling::new(
            model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
        );
        assert!(
            c.single_hinge().is_some(),
            "geometry is a single hinge (the analytic path's predicate)"
        );
        let wrench = SpatialVector::from_row_slice(&[0.1, 0.2, 0.0, 5.0, -3.0, 800.0]);
        let analytic = c
            .analytic_state_jacobian(&wrench)
            .expect("a damped single hinge now has an analytic J_state");
        let fd = c.loaded_state_jacobian(&wrench);
        let rel = (&analytic - &fd).norm() / fd.norm().max(1e-30);
        eprintln!("damped analytic J_state vs FD: rel={rel:.3e}");
        assert!(
            rel < 1e-6,
            "damped analytic J_state must match the FD loaded Jacobian, got rel {rel:.3e}"
        );
    }

    /// The analytic UNDAMPED hinge **chain** `J_state`
    /// ([`StaggeredCoupling::chain_state_jacobian`]) is MACHINE-EXACT against the FD
    /// [`StaggeredCoupling::loaded_state_jacobian`] for arbitrary held wrenches (force AND
    /// torque, nonzero per-joint `qvel`). Run on a planar (parallel-axis) 2-link, a SPATIAL
    /// (non-parallel-axis, off-axis COM) 2-link, and a SPATIAL 3-link — the spatial cases
    /// exercise the off-diagonal kinematics and depend on the sim-core Coriolis fixes (the
    /// `∂S/∂q` ancestor term + the multi-hop bias-acceleration X_b transport) for their unloaded
    /// `A`; the 3-link additionally exercises multi-hop ancestors. Pins the full chain assembly:
    /// the case-split geometric-stiffness Hessian `G`, the mass-directional derivative `dMu` (the
    /// `∂M⁻¹/∂q` term that vanishes for a single hinge), and the `A + Δt·M⁻¹·(G − dMu)`
    /// decomposition on both the velocity and semi-implicit position rows. See
    /// `docs/keystone/multilink_recon.md`.
    #[test]
    fn chain_state_jacobian_matches_fd_loaded() {
        const PLANAR_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="upper" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.025" size="0.004" mass="0.3"/>
      <body name="lower" pos="0 0 -0.05">
        <joint type="hinge" axis="0 1 0"/>
        <geom type="sphere" pos="0 0 -0.04" size="0.004" mass="0.4"/>
      </body>
    </body>
  </worldbody>
</mujoco>"#;
        const SPATIAL_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="upper" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0.01 0 -0.025" size="0.004" mass="0.3"/>
      <body name="lower" pos="0.02 0 -0.05">
        <joint type="hinge" axis="1 0.3 0"/>
        <geom type="sphere" pos="0.015 0.01 -0.04" size="0.004" mass="0.4"/>
      </body>
    </body>
  </worldbody>
</mujoco>"#;
        // 3-link, all axes non-parallel → multi-hop strict ancestors (joint 0 is an ancestor
        // of joint 2 two links up).
        const THREELINK_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="upper" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0.01 0 -0.025" size="0.004" mass="0.3"/>
      <body name="mid" pos="0.02 0 -0.05">
        <joint type="hinge" axis="1 0.3 0"/>
        <geom type="sphere" pos="0.015 0.01 -0.03" size="0.004" mass="0.35"/>
        <body name="lower" pos="0.01 0.005 -0.05">
          <joint type="hinge" axis="0.2 1 0.1"/>
          <geom type="sphere" pos="0.012 0.008 -0.03" size="0.004" mass="0.4"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>"#;

        // (mjcf, contact body = tip, nv). Planar + spatial 2-link, and spatial 3-link (multi-hop).
        for (mjcf, body, nv) in [
            (PLANAR_MJCF, 2, 2),
            (SPATIAL_MJCF, 2, 2),
            (THREELINK_MJCF, 3, 3),
        ] {
            let model = load_model(mjcf).expect("chain MJCF loads");
            let mut data = model.make_data();
            let q0 = [0.2, -0.15, 0.1];
            let v0 = [-1.5, 0.8, -1.1]; // nonzero velocities exercise the full transition (Coriolis)
            for i in 0..nv {
                data.qpos[i] = q0[i];
                data.qvel[i] = v0[i];
            }
            data.forward(&model).expect("forward");
            let c: StaggeredCoupling = StaggeredCoupling::new(
                model, data, body, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
            );
            assert!(
                c.single_hinge().is_none(),
                "a multi-link chain must NOT take the single-hinge path"
            );

            for wrench in [
                SpatialVector::from_row_slice(&[0.1, 0.2, 0.0, 5.0, -3.0, 800.0]),
                SpatialVector::from_row_slice(&[0.0, 0.0, 0.0, 0.0, 0.0, 600.0]),
                SpatialVector::from_row_slice(&[0.3, -0.4, 0.2, -10.0, 7.0, -250.0]),
            ] {
                let loaded = c.loaded_state_jacobian(&wrench);
                let analytic = c
                    .analytic_state_jacobian(&wrench)
                    .expect("multi-link chain → analytic chain J_state");
                // Tolerance is the FD REFERENCE's own central-difference floor (~1e-6, looser
                // on the chain's velocity-velocity block); the analytic form is exact — the
                // end-to-end gradient gate validates machine-exactness against re-rolled FD.
                let (err, loc) = sim_core::max_relative_error(&loaded, &analytic, 1e-3);
                assert!(
                    err < 1e-5,
                    "analytic chain J_state (nv={nv}) must match FD loaded to FD precision, \
                     got rel {err:.3e} at {loc:?}"
                );
            }
        }
    }

    /// A DAMPED serial hinge chain's analytic `J_state` matches the FD
    /// [`StaggeredCoupling::loaded_state_jacobian`] — the multi-link analogue of
    /// [`analytic_state_jacobian_damped_matches_fd`]. The loaded term routes through
    /// `M_impl = M + Δt·D`; the unloaded `A` is eulerdamp-correct from sim-core. Covers 2-link
    /// (spatial) and 3-link (multi-hop) chains at several damping levels; `damp = 0` confirms the
    /// damped code path collapses to the bare-M result.
    #[test]
    fn chain_state_jacobian_damped_matches_fd() {
        // Spatial 2-link and multi-hop 3-link, joints carrying `damping` (interpolated below).
        let mk = |damp: f64| -> [(String, usize, usize); 2] {
            let two = format!(
                r#"<mujoco><option gravity="0 0 -9.81" timestep="0.001"/><worldbody>
<body name="upper" pos="0 0 0.2"><joint type="hinge" axis="0 1 0" damping="{damp}"/>
<geom type="sphere" pos="0.01 0 -0.025" size="0.004" mass="0.3"/>
<body name="lower" pos="0.02 0 -0.05"><joint type="hinge" axis="1 0.3 0" damping="{damp}"/>
<geom type="sphere" pos="0.015 0.01 -0.04" size="0.004" mass="0.4"/></body></body></worldbody></mujoco>"#
            );
            let three = format!(
                r#"<mujoco><option gravity="0 0 -9.81" timestep="0.001"/><worldbody>
<body name="upper" pos="0 0 0.2"><joint type="hinge" axis="0 1 0" damping="{damp}"/>
<geom type="sphere" pos="0.01 0 -0.025" size="0.004" mass="0.3"/>
<body name="mid" pos="0.02 0 -0.05"><joint type="hinge" axis="1 0.3 0" damping="{damp}"/>
<geom type="sphere" pos="0.015 0.01 -0.03" size="0.004" mass="0.35"/>
<body name="lower" pos="0.01 0.005 -0.05"><joint type="hinge" axis="0.2 1 0.1" damping="{damp}"/>
<geom type="sphere" pos="0.012 0.008 -0.03" size="0.004" mass="0.4"/></body></body></body></worldbody></mujoco>"#
            );
            [(two, 2, 2), (three, 3, 3)]
        };
        for &damp in &[0.0_f64, 0.5, 2.0] {
            for (mjcf, body, nv) in mk(damp) {
                let model = load_model(&mjcf).expect("damped chain MJCF loads");
                let mut data = model.make_data();
                let q0 = [0.2, -0.15, 0.1];
                let v0 = [-1.5, 0.8, -1.1]; // nonzero ⇒ exercises Coriolis
                for i in 0..nv {
                    data.qpos[i] = q0[i];
                    data.qvel[i] = v0[i];
                }
                data.forward(&model).expect("forward");
                let c: StaggeredCoupling = StaggeredCoupling::new(
                    model, data, body, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
                );
                let wrench = SpatialVector::from_row_slice(&[0.3, -0.4, 0.2, -10.0, 7.0, -250.0]);
                let loaded = c.loaded_state_jacobian(&wrench);
                let analytic = c
                    .analytic_state_jacobian(&wrench)
                    .expect("damped multi-link chain → analytic chain J_state");
                let (err, loc) = sim_core::max_relative_error(&loaded, &analytic, 1e-3);
                assert!(
                    err < 1e-5,
                    "damped analytic chain J_state (nv={nv}, damp={damp}) must match FD loaded, \
                     got rel {err:.3e} at {loc:?}"
                );
            }
        }
    }

    /// The tangent FD `loaded_state_jacobian` for a ball joint (`nq = 4, nv = 3`) is
    /// SO(3)-correct: the position-VELOCITY block `∂(tangent qpos')/∂qvel` is the
    /// integrator's `≈ Δt·I` (NOT zero — the bug the lossy `sqrt(1−w²)+acos` log map in
    /// `mj_differentiate_pos` produced when a tiny FD rotation's `w` rounded to 1; fixed
    /// to the vector-norm + atan2 form), and the position-VELOCITY and velocity-rows
    /// match sim-core's analytic transition `A`. (The position-POSITION block legitimately
    /// differs from `A`: this carry measures the output tangent at the nominal `qpos'`
    /// (`mj_differentiate_pos`, the convention `jz`/`J_r` consume), whereas
    /// `transition_derivatives` references it at `q_old` — a different, internally
    /// consistent tangent. The end-to-end `ball_joint_gradient_matches_fd` trajectory gate
    /// validates the full convention-consistent composition.)
    #[test]
    fn loaded_state_jacobian_ball_velocity_block_matches_analytic() {
        const BALL_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="ball"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
        let model = load_model(BALL_MJCF).expect("ball MJCF loads");
        let mut data = model.make_data();
        let half = 0.15_f64;
        data.qpos[0] = half.cos();
        data.qpos[2] = half.sin();
        data.qvel[1] = -1.5; // nonzero ang vel exercises the integrator coupling
        data.forward(&model).expect("forward");
        let c: StaggeredCoupling = StaggeredCoupling::new(
            model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
        );
        // wrench = 0 ⇒ the FD loaded Jacobian is the unloaded transition. Two checks:
        //  (1) the velocity rows (`∂qvel'/∂·`, convention-free — qvel' has no quaternion)
        //      match the validated analytic transition `A`; and
        //  (2) the position-velocity block's DIAGONAL is ≈ Δt — the integrator's `J_r ≈ I`.
        //      This is the atan2-log-map regression guard: the lossy `sqrt(1−w²)+acos` form
        //      returned ZERO here (a tiny FD rotation's `w` rounds to 1). The off-diagonals
        //      are O(Δt²) and convention-dependent (see the doc), so are not compared.
        let loaded = c.loaded_state_jacobian(&SpatialVector::zeros());
        let a = c
            .data
            .transition_derivatives(&c.model, &sim_core::DerivativeConfig::default())
            .expect("transition derivatives")
            .A;
        let nv = 3;
        let dt = 1.0e-3;
        for r in nv..2 * nv {
            for col in 0..2 * nv {
                let (lv, av) = (loaded[(r, col)], a[(r, col)]);
                let rel = (lv - av).abs() / av.abs().max(1e-6);
                assert!(
                    rel < 1e-4,
                    "ball velocity row J_state[{r},{col}] = {lv:.6e} != A {av:.6e}"
                );
            }
        }
        for i in 0..nv {
            let d = loaded[(i, nv + i)];
            assert!(
                (d - dt).abs() < 1e-5,
                "ball position-velocity diagonal J_state[{i},{}] = {d:.6e} must be ≈Δt={dt:.0e} \
                 (atan2 log-map fix — was ZERO with the lossy sqrt(1−w²)+acos form)",
                nv + i
            );
        }
    }

    /// `ContactWrenchTrajVjp`'s analytic Jacobian (the contact-MOMENT leaf's core
    /// new math — `∂w/∂x*` with the moment's explicit-`rᵢ` + via-`gᵢ` parts, `∂w/∂h`
    /// the force/moment-vs-height feedback, and `∂w/∂s` the moment's `c(q)` feedback)
    /// is FD-EXACT against the REAL contact readout at a deeply-engaged off-COM hinge
    /// config (the tip COM `xipos` offset ~0.08 m from the block-top contact
    /// centroid, so the moment is large). This pins the wrench node independently of
    /// the multi-step composition (the trajectory gate validates the composition).
    #[test]
    #[allow(clippy::similar_names)]
    fn contact_wrench_node_matches_readout_fd() {
        const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
        let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
        let mut data = model.make_data();
        data.qpos[0] = 0.3; // tilt → off-COM contact (large moment)
        data.forward(&model).expect("forward");
        let c: StaggeredCoupling = StaggeredCoupling::new(
            model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
        );

        let height = c.tip_plane_height();
        let positions = c.positions();
        let com = c.data.xipos[c.body];
        let n_dof = 3 * c.n_vertices;
        let nv = c.model.nv;

        // The REAL reaction wrench from the contact readout (recomputed g + r).
        let wrench_of = |h: f64, pos: &[Vec3], cc: Vec3| -> [f64; 6] {
            let mut w = [0.0_f64; 6];
            for (_, g, _, _, r) in c.active_pair_wrench_data(h, pos) {
                let f = -g;
                w[3] += f.x;
                w[4] += f.y;
                w[5] += f.z;
                let tau = (r - cc).cross(&f);
                w[0] += tau.x;
                w[1] += tau.y;
                w[2] += tau.z;
            }
            w
        };

        // The analytic node at this config.
        let mut active = Vec::new();
        let mut force = Vec3::zeros();
        for (v, g, n, curv, r) in c.active_pair_wrench_data(height, &positions) {
            force += -g;
            active.push((v, g, n, curv, r - com, c.collider_hessian(height, r)));
        }
        assert!(!active.is_empty(), "config must be contact-engaged");
        let node = ContactWrenchTrajVjp {
            active,
            force,
            jlin: c.com_linear_jacobian(),
            n_dof,
            nv,
            pose: WrenchPose::Height,
        };
        // Row k of the Jacobian = the parent-cotangents for the unit cotangent e_k.
        let jac_row = |k: usize| -> (Vec<f64>, f64, Vec<f64>) {
            let mut cot = Tensor::zeros(&[6]);
            cot.as_mut_slice()[k] = 1.0;
            let mut pc = vec![
                Tensor::zeros(&[n_dof]),
                Tensor::zeros(&[1]),
                Tensor::zeros(&[2 * nv]),
            ];
            node.vjp(&cot, &mut pc);
            (
                pc[0].as_slice().to_vec(),
                pc[1].as_slice()[0],
                pc[2].as_slice().to_vec(),
            )
        };
        let rows: Vec<_> = (0..6).map(jac_row).collect();
        let d = 1.0e-7;

        let mut worst = 0.0_f64; // max |fd − analytic| / scale over all checked entries
        let mut check = |fd: f64, an: f64, scale: f64| {
            let rel = (fd - an).abs() / scale.max(1.0);
            worst = worst.max(rel);
        };
        // Scales per channel (the dominant Jacobian magnitudes seen in this scene).
        let sx = 1.0e3;
        let sh = 1.0e5;
        let sq = 1.0e3;

        // ∂w/∂x* (each soft DOF) — the stable engaged active set holds across ±d.
        for j in 0..n_dof {
            let mut pp = positions.clone();
            let mut pm = positions.clone();
            pp[j / 3][j % 3] += d;
            pm[j / 3][j % 3] -= d;
            let wp = wrench_of(height, &pp, com);
            let wm = wrench_of(height, &pm, com);
            for k in 0..6 {
                check((wp[k] - wm[k]) / (2.0 * d), rows[k].0[j], sx);
            }
        }
        // ∂w/∂h (plane height).
        let wp = wrench_of(height + d, &positions, com);
        let wm = wrench_of(height - d, &positions, com);
        for k in 0..6 {
            check((wp[k] - wm[k]) / (2.0 * d), rows[k].1, sh);
        }
        // ∂w/∂qpos (the moment's c(q) feedback — perturb xipos via qpos, height held).
        for jq in 0..nv {
            let scratch_com = |dq: f64| -> Vec3 {
                let mut s = c.model.make_data();
                s.qpos.copy_from(&c.data.qpos);
                s.qpos[jq] += dq;
                s.forward(&c.model).expect("forward");
                s.xipos[c.body]
            };
            let wp = wrench_of(height, &positions, scratch_com(d));
            let wm = wrench_of(height, &positions, scratch_com(-d));
            for k in 0..6 {
                check((wp[k] - wm[k]) / (2.0 * d), rows[k].2[jq], sq);
            }
        }
        assert!(
            worst < 1e-5,
            "ContactWrenchTrajVjp Jacobian must be FD-exact vs the real readout, \
             worst scaled error {worst:.3e}"
        );
    }

    /// The finite-sphere centre defaults to the block-face centroid (byte-identical to the
    /// pre-end-effector posing) and follows the `sphere_center_override` when set — the forward
    /// end-effector posing primitive.
    #[test]
    fn sphere_center_default_and_override() {
        const MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.125">
      <joint type="slide" axis="0 0 1"/>
      <geom type="box" size="0.05 0.05 0.02" mass="1"/>
    </body>
  </worldbody>
</mujoco>"#;
        const SPHERE_R: f64 = 0.08;
        const EDGE: f64 = 0.1;
        let model = load_model(MJCF).expect("MJCF loads");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        let mut c: StaggeredCoupling = StaggeredCoupling::new(
            model, data, 1, 0.005, 4, EDGE, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
        )
        .with_sphere_collider(SPHERE_R);

        let height = 0.09_f64;
        // Default = block top-face centroid; `z = height + radius`.
        assert_eq!(
            c.sphere_center(SPHERE_R, height),
            Vec3::new(EDGE / 2.0, EDGE / 2.0, height + SPHERE_R),
        );

        // Override = the full end-effector centre, `height`-independent.
        let ee = Vec3::new(0.02, 0.03, 0.12);
        c.sphere_center_override = Some(ee);
        assert_eq!(c.sphere_center(SPHERE_R, height), ee);
    }

    /// `step_articulated` + `with_contact_geom` poses the finite fist at the rigid geom's world
    /// centre each frame (tracking the arm tip), engages the soft block, and routes the off-COM
    /// reaction wrench — the per-frame forward stepper the striker viewers drive.
    #[test]
    fn step_articulated_poses_at_contact_geom() {
        // Slide arm directly over the block centre (xy = edge/2): the fist geom hangs `0.13` below
        // the body origin at `z = 0.25`, so its centre sits at `z ≈ 0.12` and its south pole
        // (`0.12 − 0.04 = 0.08`) penetrates the rest top face (`z = 0.1`).
        const MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0.05 0.05 0.25">
      <joint type="slide" axis="0 0 1"/>
      <geom name="fist" type="sphere" pos="0 0 -0.13" size="0.04" mass="0.5"/>
    </body>
  </worldbody>
</mujoco>"#;
        const FIST_R: f64 = 0.04;
        let model = load_model(MJCF).expect("MJCF loads");
        let fist_gid = model.geom_id("fist").expect("fist geom");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        // The fist's world centre at the (unchanged) start config — what `step_articulated`'s
        // leading FK poses the contact sphere at. (Captured before `data` moves into the coupling;
        // the step's trailing FK then advances the live pose.)
        let fist0 = data.geom_xpos[fist_gid];
        let mut c: StaggeredCoupling = StaggeredCoupling::new(
            model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
        )
        .with_sphere_collider(FIST_R)
        .with_contact_geom(fist_gid);

        let readout = c.step_articulated();
        // The fist posed at the geom's world centre (not the block centroid).
        assert_eq!(c.sphere_center_override, Some(fist0));
        // Engaged: a finite upward contact force on the soft block.
        assert!(readout.force_on_soft.z.is_finite());
        assert!(
            readout.force_on_soft.norm() > 1e-6,
            "fist over the block should engage the soft contact",
        );
    }

    /// `step_kinematic` + `set_sphere_center` pose the finite sphere at a scripted world point and
    /// take ONE soft step (no rigid integration) — the kinematic striker path. The fist over the
    /// block engages and dimples the soft body; no dynamic rigid body is needed (a static anchor
    /// just hosts the coupling).
    #[test]
    fn step_kinematic_poses_at_set_center_and_engages() {
        const MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="anchor">
      <geom type="sphere" size="0.01"/>
    </body>
  </worldbody>
</mujoco>"#;
        const FIST_R: f64 = 0.04;
        const EDGE: f64 = 0.1;
        let model = load_model(MJCF).expect("MJCF loads");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        let mut c: StaggeredCoupling = StaggeredCoupling::new(
            model, data, 1, 0.005, 4, EDGE, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
        )
        .with_sphere_collider(FIST_R);

        // Pose the fist over the block centre, south pole pressing into the rest top face (z = EDGE).
        let center = Vec3::new(EDGE / 2.0, EDGE / 2.0, EDGE + FIST_R - 0.01);
        c.set_sphere_center(center);
        let rest = c.soft_positions().to_vec();
        let readout = c.step_kinematic();

        assert_eq!(c.sphere_center_override, Some(center));
        assert!(
            readout.force_on_soft.norm() > 1e-6,
            "fist over the block should engage the soft contact",
        );
        let moved = c
            .soft_positions()
            .iter()
            .zip(&rest)
            .any(|(a, b)| (a - b).abs() > 1e-9);
        assert!(moved, "kinematic step should deform the soft body");
    }

    /// L1b articulated NORMAL wrench curvature: the contact-wrench node on a FINITE sphere
    /// collider is FD-exact against the real readout. Same engaged off-COM hinge config as
    /// `contact_wrench_node_matches_readout_fd`, but with `with_sphere_collider` — the sphere's
    /// contact normal turns as a soft vertex slides (`∂n̂/∂x = H`) and as the primitive
    /// translates with the tip height (`∂n̂/∂h = −H·ẑ`), so `∂w/∂x*` and `∂w/∂h` carry the
    /// geometric-stiffness term `f_mag·H` (zero for the plane's constant normal). A flat
    /// `ContactWrenchTrajVjp` misses it and disagrees with the curved-readout FD at the
    /// curvature scale.
    #[test]
    #[allow(clippy::similar_names)]
    fn sphere_contact_wrench_node_matches_readout_fd() {
        const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
        // Sphere radius: large vs the 0.1 m block so the south-pole patch spans several top-face
        // vertices yet stays curved enough that f_mag·H is materially nonzero.
        const SPHERE_R: f64 = 0.08;
        let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
        let mut data = model.make_data();
        data.qpos[0] = 0.3; // tilt → off-COM contact (large moment) + engaged sphere
        data.forward(&model).expect("forward");
        let c: StaggeredCoupling = StaggeredCoupling::new(
            model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
        )
        .with_sphere_collider(SPHERE_R);

        let height = c.tip_plane_height();
        let positions = c.positions();
        let com = c.data.xipos[c.body];
        let n_dof = 3 * c.n_vertices;
        let nv = c.model.nv;

        // The REAL reaction wrench from the curved-collider readout (recomputed g + r).
        let wrench_of = |h: f64, pos: &[Vec3], cc: Vec3| -> [f64; 6] {
            let mut w = [0.0_f64; 6];
            for (_, g, _, _, r) in c.active_pair_wrench_data(h, pos) {
                let f = -g;
                w[3] += f.x;
                w[4] += f.y;
                w[5] += f.z;
                let tau = (r - cc).cross(&f);
                w[0] += tau.x;
                w[1] += tau.y;
                w[2] += tau.z;
            }
            w
        };

        // The analytic node at this config (the curved H comes from `active_pair_wrench_curv`).
        let mut active = Vec::new();
        let mut force = Vec3::zeros();
        for (v, g, n, curv, r) in c.active_pair_wrench_data(height, &positions) {
            force += -g;
            active.push((
                v,
                g,
                n,
                curv,
                r - com,
                c.collider_hessian(height, positions[v]),
            ));
        }
        assert!(!active.is_empty(), "config must be contact-engaged");
        let node = ContactWrenchTrajVjp {
            active,
            force,
            jlin: c.com_linear_jacobian(),
            n_dof,
            nv,
            pose: WrenchPose::Height,
        };
        let jac_row = |k: usize| -> (Vec<f64>, f64, Vec<f64>) {
            let mut cot = Tensor::zeros(&[6]);
            cot.as_mut_slice()[k] = 1.0;
            let mut pc = vec![
                Tensor::zeros(&[n_dof]),
                Tensor::zeros(&[1]),
                Tensor::zeros(&[2 * nv]),
            ];
            node.vjp(&cot, &mut pc);
            (
                pc[0].as_slice().to_vec(),
                pc[1].as_slice()[0],
                pc[2].as_slice().to_vec(),
            )
        };
        let rows: Vec<_> = (0..6).map(jac_row).collect();
        let d = 1.0e-7;

        let mut worst = 0.0_f64;
        let mut check = |fd: f64, an: f64, scale: f64| {
            let rel = (fd - an).abs() / scale.max(1.0);
            worst = worst.max(rel);
        };
        let sx = 1.0e3;
        let sh = 1.0e5;
        let sq = 1.0e3;

        for j in 0..n_dof {
            let mut pp = positions.clone();
            let mut pm = positions.clone();
            pp[j / 3][j % 3] += d;
            pm[j / 3][j % 3] -= d;
            let wp = wrench_of(height, &pp, com);
            let wm = wrench_of(height, &pm, com);
            for k in 0..6 {
                check((wp[k] - wm[k]) / (2.0 * d), rows[k].0[j], sx);
            }
        }
        let wp = wrench_of(height + d, &positions, com);
        let wm = wrench_of(height - d, &positions, com);
        for k in 0..6 {
            check((wp[k] - wm[k]) / (2.0 * d), rows[k].1, sh);
        }
        for jq in 0..nv {
            let scratch_com = |dq: f64| -> Vec3 {
                let mut s = c.model.make_data();
                s.qpos.copy_from(&c.data.qpos);
                s.qpos[jq] += dq;
                s.forward(&c.model).expect("forward");
                s.xipos[c.body]
            };
            let wp = wrench_of(height, &positions, scratch_com(d));
            let wm = wrench_of(height, &positions, scratch_com(-d));
            for k in 0..6 {
                check((wp[k] - wm[k]) / (2.0 * d), rows[k].2[jq], sq);
            }
        }
        assert!(
            worst < 1e-5,
            "sphere ContactWrenchTrajVjp Jacobian must be FD-exact vs the curved readout, \
             worst scaled error {worst:.3e}"
        );
    }

    /// The CENTRE-channel companion to `sphere_contact_wrench_node_matches_readout_fd`:
    /// the moving-end-effector sphere is posed at an explicit centre OVERRIDE (the arm tip),
    /// and the wrench node's pose parent is the 3-vector centre channel ([`WrenchPose::Centre`])
    /// rather than the scalar height. FD: translate the centre along each axis, recompute the
    /// curved-collider wrench readout (soft positions held), compare to the analytic per-axis
    /// pose Jacobian. The z axis reproduces the `Height` channel; x/y are the lateral channels
    /// the moving-EE carry needs (the `f_mag·H` normal-rotation term is materially nonzero on
    /// the off-pole curved patch the off-centroid centre selects).
    #[test]
    #[allow(clippy::similar_names)]
    fn sphere_contact_wrench_node_centre_matches_readout_fd() {
        const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
        const SPHERE_R: f64 = 0.08;
        let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
        let mut data = model.make_data();
        data.qpos[0] = 0.3; // tilt → off-COM contact + engaged sphere
        data.forward(&model).expect("forward");
        let mut c: StaggeredCoupling = StaggeredCoupling::new(
            model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
        )
        .with_sphere_collider(SPHERE_R);

        let height = c.tip_plane_height();
        // Pose the fist at an explicit centre (the moving-EE scenario), OFF the block centroid
        // so n̂·x̂ and n̂·ŷ are nonzero on the curved patch ⇒ the lateral channels are exercised.
        let base = Vec3::new(
            c.edge / 2.0 + 0.012,
            c.edge / 2.0 - 0.009,
            height + SPHERE_R,
        );
        c.set_sphere_center(base);
        let positions = c.positions();
        let com = c.data.xipos[c.body];
        let n_dof = 3 * c.n_vertices;
        let nv = c.model.nv;

        // The curved-collider wrench readout at the current sphere-centre override, soft
        // positions held fixed (the explicit ∂w/∂centre the pose parent carries).
        let wrench_of = |c: &StaggeredCoupling, pos: &[Vec3]| -> [f64; 6] {
            let mut w = [0.0_f64; 6];
            for (_, g, _, _, r) in c.active_pair_wrench_data(height, pos) {
                let f = -g;
                w[3] += f.x;
                w[4] += f.y;
                w[5] += f.z;
                let tau = (r - com).cross(&f);
                w[0] += tau.x;
                w[1] += tau.y;
                w[2] += tau.z;
            }
            w
        };

        // Analytic node at the base centre (curv + H read here, before the FD mutates the override).
        let mut active = Vec::new();
        let mut force = Vec3::zeros();
        for (v, g, n, curv, r) in c.active_pair_wrench_data(height, &positions) {
            force += -g;
            active.push((
                v,
                g,
                n,
                curv,
                r - com,
                c.collider_hessian(height, positions[v]),
            ));
        }
        assert!(!active.is_empty(), "config must be contact-engaged");
        let basis = vec![
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ];
        let node = ContactWrenchTrajVjp {
            active,
            force,
            jlin: c.com_linear_jacobian(),
            n_dof,
            nv,
            pose: WrenchPose::Centre { basis },
        };
        // Pose-parent cotangent rows: ∂w_k/∂(centre axis), one [3] per wrench component k.
        let pose_rows: Vec<[f64; 3]> = (0..6)
            .map(|k| {
                let mut cot = Tensor::zeros(&[6]);
                cot.as_mut_slice()[k] = 1.0;
                let mut pc = vec![
                    Tensor::zeros(&[n_dof]),
                    Tensor::zeros(&[3]),
                    Tensor::zeros(&[2 * nv]),
                ];
                node.vjp(&cot, &mut pc);
                let s = pc[1].as_slice();
                [s[0], s[1], s[2]]
            })
            .collect();

        let d = 1.0e-7;
        // The centre channel is a translation like the height channel ⇒ the same 1e5 scale
        // the height gate calibrated (the z axis IS that channel; x/y are smaller).
        let scale = 1.0e5;
        let mut worst = 0.0_f64;
        for axis in 0..3 {
            let mut dir = Vec3::zeros();
            dir[axis] = 1.0;
            c.set_sphere_center(base + d * dir);
            let wp = wrench_of(&c, &positions);
            c.set_sphere_center(base - d * dir);
            let wm = wrench_of(&c, &positions);
            for k in 0..6 {
                let fd = (wp[k] - wm[k]) / (2.0 * d);
                worst = worst.max((fd - pose_rows[k][axis]).abs() / scale);
            }
        }
        assert!(
            worst < 1e-5,
            "sphere ContactWrenchTrajVjp CENTRE Jacobian must be FD-exact vs the curved readout, \
             worst scaled error {worst:.3e}"
        );
    }

    /// The FRICTION wrench node's 3-vector CENTRE pose channel is FD-exact — the friction analog of
    /// `sphere_contact_wrench_node_centre_matches_readout_fd`, covering the L1 friction-pose WIRING
    /// (the `[axis][vertex] → [vertex][axis]` `dforce_dpose` transpose + `n_pose` + the node's
    /// `pose_slot` loop) that the (pose-INSENSITIVE) friction trajectory gate cannot reach. FD:
    /// perturb the sphere centre per axis (soft x* held), recompute the friction wrench
    /// `Σ[∇D_v; (r_v−c)×∇D_v]`, and compare to the node's pose-parent cotangent rows.
    #[test]
    #[allow(clippy::similar_names)]
    fn friction_wrench_node_centre_matches_readout_fd() {
        const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="1.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
        const SPHERE_R: f64 = 0.08;
        let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
        let mut data = model.make_data();
        data.qpos[0] = 0.3;
        data.forward(&model).expect("forward");
        let mut c: StaggeredCoupling = StaggeredCoupling::new(
            model, data, 1, 0.005, 4, 0.1, 3.0e3, 1.2e4, 3.0e4, 1.0e-2, 0.0,
        )
        .with_sphere_collider(SPHERE_R)
        .with_friction(2.5, 0.1);

        let height = c.tip_plane_height();
        let dt = c.cfg.dt;
        let n = c.n_vertices;
        let nv = c.model.nv;
        let drift = Vec3::new(5.0e-4, 0.0, 0.0);
        let drift_dir = Vec3::new(1.0, 0.0, 0.0);
        let com = c.data.xipos[c.body];
        let x_start = c.x.clone();
        // Off-centroid centre (the moving-EE scenario) so the lateral axes are exercised.
        let base = Vec3::new(c.edge / 2.0 + 0.01, c.edge / 2.0 - 0.008, height + SPHERE_R);
        let basis = [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ];

        // One friction soft solve at the base centre (x* held fixed for the explicit-pose FD).
        c.sphere_center_override = Some(base);
        let x_next = {
            let bc = BoundaryConditions::new(c.pinned.clone(), Vec::new());
            let solver: SoftSolver<_> =
                CpuNewtonSolver::new(Tet4, c.fresh_mesh(), c.build_contact(height), c.cfg, bc)
                    .with_friction_surface_drift(drift);
            solver
                .replay_step(
                    &Tensor::from_slice(&c.x, &[3 * n]),
                    &Tensor::from_slice(&c.v, &[3 * n]),
                    &Tensor::zeros(&[0]),
                    dt,
                )
                .x_final
        };
        let positions: Vec<Vec3> = x_next
            .chunks_exact(3)
            .map(|q| Vec3::new(q[0], q[1], q[2]))
            .collect();

        // Friction wrench readout Σ[∇D_v; (r_v−c)×∇D_v] at sphere centre `centre`, x* held.
        let wrench_of = |c: &mut StaggeredCoupling, centre: Vec3| -> [f64; 6] {
            c.sphere_center_override = Some(centre);
            let bc = BoundaryConditions::new(c.pinned.clone(), Vec::new());
            let solver: SoftSolver<_> =
                CpuNewtonSolver::new(Tet4, c.fresh_mesh(), c.build_contact(height), c.cfg, bc)
                    .with_friction_surface_drift(drift);
            let pv = solver.friction_force_jacobians(&x_next, &x_start, dt, drift_dir, basis[2]);
            let mut w = [0.0_f64; 6];
            for p in pv {
                let f = p.force;
                let tau = (positions[p.vid as usize] - com).cross(&f);
                w[0] += tau.x;
                w[1] += tau.y;
                w[2] += tau.z;
                w[3] += f.x;
                w[4] += f.y;
                w[5] += f.z;
            }
            w
        };

        // Analytic node at the base centre: per-axis `dforce_dpose` (the transpose the gradient builds).
        c.sphere_center_override = Some(base);
        let bc = BoundaryConditions::new(c.pinned.clone(), Vec::new());
        let solver: SoftSolver<_> =
            CpuNewtonSolver::new(Tet4, c.fresh_mesh(), c.build_contact(height), c.cfg, bc)
                .with_friction_surface_drift(drift);
        let pv = solver.friction_force_jacobians(&x_next, &x_start, dt, drift_dir, basis[2]);
        let per_axis: Vec<Vec<Vec3>> = basis
            .iter()
            .map(|&e| {
                solver
                    .friction_force_jacobians(&x_next, &x_start, dt, drift_dir, e)
                    .into_iter()
                    .map(|p| p.dforce_dheight)
                    .collect()
            })
            .collect();
        let pose_dforce: Vec<Vec<Vec3>> = (0..pv.len())
            .map(|i| per_axis.iter().map(|a| a[i]).collect())
            .collect();
        let mut w_total = SpatialVector::zeros();
        let (fverts, f_total) =
            assemble_friction_wrench(pv, &positions, com, &mut w_total, Some(pose_dforce));
        assert!(!fverts.is_empty(), "config must be friction-engaged");
        let node = FrictionWrenchTrajVjp {
            verts: fverts,
            f_total,
            jlin: c.com_linear_jacobian(),
            n_dof: 3 * n,
            nv,
            n_pose: 3,
            mu_c: false,
        };
        // Pose-parent cotangent rows: ∂(w·e_k)/∂(centre axis), one [3] per wrench component k.
        let pose_rows: Vec<[f64; 3]> = (0..6)
            .map(|k| {
                let mut cot = Tensor::zeros(&[6]);
                cot.as_mut_slice()[k] = 1.0;
                let mut pc = vec![
                    Tensor::zeros(&[6]),
                    Tensor::zeros(&[3 * n]),
                    Tensor::zeros(&[3]),
                    Tensor::zeros(&[2 * nv]),
                    Tensor::zeros(&[1]),
                    Tensor::zeros(&[3 * n]),
                ];
                node.vjp(&cot, &mut pc);
                let s = pc[2].as_slice();
                [s[0], s[1], s[2]]
            })
            .collect();

        let d = 1.0e-7;
        let scale = 1.0e5; // friction-wrench-vs-centre derivatives ~1e5 (cf. the leaf reaction gate)
        let mut worst = 0.0_f64;
        for axis in 0..3 {
            let mut dir = Vec3::zeros();
            dir[axis] = 1.0;
            let wp = wrench_of(&mut c, base + d * dir);
            let wm = wrench_of(&mut c, base - d * dir);
            for k in 0..6 {
                let fd = (wp[k] - wm[k]) / (2.0 * d);
                worst = worst.max((fd - pose_rows[k][axis]).abs() / scale);
            }
        }
        assert!(
            worst < 1e-5,
            "friction wrench CENTRE pose Jacobian must be FD-exact vs the readout, worst {worst:.3e}"
        );
    }

    /// `VzControlCarryVjp` carries the three rigid-carry coefficients onto its
    /// parents: `∂vz'/∂vz = a`, `∂vz'/∂fz = −Δt/m`, `∂vz'/∂u = +Δt/m` (the control
    /// term, opposite sign to the contact term).
    #[test]
    fn control_carry_vjp_coefficients() {
        let op = VzControlCarryVjp {
            a: 0.7,
            neg_dt_over_m: -5e-3,
            dt_over_m: 5e-3,
        };
        let mut parents = vec![
            Tensor::zeros(&[1]),
            Tensor::zeros(&[1]),
            Tensor::zeros(&[1]),
        ];
        op.vjp(&Tensor::from_slice(&[1.0], &[1]), &mut parents);
        assert!(
            (parents[0].as_slice()[0] - 0.7).abs() < 1e-15,
            "∂vz'/∂vz = a"
        );
        assert!(
            (parents[1].as_slice()[0] + 5e-3).abs() < 1e-15,
            "∂vz'/∂fz = −Δt/m"
        );
        assert!(
            (parents[2].as_slice()[0] - 5e-3).abs() < 1e-15,
            "∂vz'/∂u = +Δt/m"
        );
    }

    /// Lib-level smoke test of the control gradient (the scientific FD validation
    /// is in `tests/coupled_control_gradient.rs`): one `tape.backward` over a
    /// short coupled rollout under a control schedule gives one finite gradient
    /// per control input, the forward replays the real dynamics, and a single
    /// control input matches an independent FD of the real re-rollout.
    #[test]
    fn control_gradient_smoke() {
        let controls = vec![-1.5_f64, 1.0, -1.5, 1.0, -1.5, 1.0];
        let (z_tape, grad) = coupling().coupled_trajectory_control_gradient(&controls);
        assert_eq!(grad.len(), controls.len());
        assert!(grad.iter().all(|g| g.is_finite()), "gradients finite");

        // Forward reproduces the real rollout.
        let z_ref = coupling().coupled_trajectory_control_z(&controls);
        assert!(
            (z_tape - z_ref).abs() < 1e-12,
            "tape forward z_N {z_tape} != real rollout {z_ref}"
        );

        // One control input vs an independent FD of the real coupled re-rollout.
        let k = 1;
        let eps = 1e-2;
        let mut up = controls.clone();
        let mut dn = controls.clone();
        up[k] += eps;
        dn[k] -= eps;
        let fd = (coupling().coupled_trajectory_control_z(&up)
            - coupling().coupled_trajectory_control_z(&dn))
            / (2.0 * eps);
        assert!(
            (grad[k] - fd).abs() / fd.abs().max(1e-30) < 1e-6,
            "smoke: control tape grad {} vs FD {fd}",
            grad[k]
        );
    }

    /// Lib-level smoke test of the closed-loop policy gradient (the scientific FD
    /// validation is in `tests/coupled_policy_gradient.rs`): one `tape.backward`
    /// over a short closed-loop rollout under `LinearFeedback` gives one finite
    /// gradient per policy parameter, the feedback-weight gradients are nonzero
    /// (the recurrence is live), the forward replays the real dynamics, and a
    /// feedback weight matches an independent FD of the real re-rollout.
    #[test]
    fn policy_gradient_smoke() {
        let theta = [-20.0_f64, -5.0, 2.0];
        let n = 8;
        let (z_tape, grad) =
            coupling().coupled_trajectory_policy_gradient(&LinearFeedback, &theta, n);
        assert_eq!(grad.len(), 3);
        assert!(grad.iter().all(|g| g.is_finite()), "gradients finite");
        // Feedback-weight gradients are nonzero: the recurrence is exercised, not
        // just the bias.
        assert!(
            grad[0].abs() > 1e-9 && grad[1].abs() > 1e-9,
            "feedback-weight gradients should be nonzero, got {grad:?}"
        );

        // Forward reproduces the real closed-loop rollout.
        let z_ref = coupling().coupled_trajectory_policy_z(&LinearFeedback, &theta, n);
        assert!(
            (z_tape - z_ref).abs() < 1e-12,
            "tape forward z_N {z_tape} != real rollout {z_ref}"
        );

        // The proportional weight w_z vs an independent FD of the real re-rollout
        // (its gradient flows only through the state→control recurrence).
        let eps = 1e-2;
        let mut up = theta;
        let mut dn = theta;
        up[0] += eps;
        dn[0] -= eps;
        let fd = (coupling().coupled_trajectory_policy_z(&LinearFeedback, &up, n)
            - coupling().coupled_trajectory_policy_z(&LinearFeedback, &dn, n))
            / (2.0 * eps);
        assert!(
            (grad[0] - fd).abs() / fd.abs().max(1e-30) < 1e-5,
            "smoke: policy w_z tape grad {} vs FD {fd}",
            grad[0]
        );
    }

    /// Lib-level smoke test of the JOINT design+policy gradient (the scientific FD
    /// validation is in `tests/coupled_joint_gradient.rs`): one `tape.backward`
    /// yields BOTH a finite material gradient `∂z_N/∂μ_total` AND the policy
    /// gradient `∂z_N/∂θ`, the forward replays the real rollout, and the joint
    /// policy block matches the policy-only method (the material leaf does not
    /// perturb the policy gradient).
    #[test]
    fn joint_gradient_smoke() {
        let theta = [-20.0_f64, -5.0, 2.0];
        let n = 8;
        let (z_tape, dz_dmu, dz_dtheta) =
            coupling().coupled_trajectory_joint_gradient(&LinearFeedback, &theta, n);
        assert_eq!(dz_dtheta.len(), 3);
        // Both blocks finite (the `coupling()` fixture starts above contact, so at
        // n=8 the block may be undeformed ⇒ ∂z/∂μ can be 0; the engaged μ gradient
        // is validated nonzero + FD-exact in `tests/coupled_joint_gradient.rs`). The
        // policy block is genuinely live (control always moves z).
        assert!(
            dz_dmu.is_finite() && dz_dtheta.iter().all(|g| g.is_finite()),
            "joint gradients finite: μ={dz_dmu}, θ={dz_dtheta:?}"
        );
        assert!(
            dz_dtheta.iter().any(|g| g.abs() > 1e-9),
            "policy block should be live, got {dz_dtheta:?}"
        );

        // Forward reproduces the real closed-loop rollout.
        let z_ref = coupling().coupled_trajectory_policy_z(&LinearFeedback, &theta, n);
        assert!(
            (z_tape - z_ref).abs() < 1e-12,
            "tape z_N {z_tape} != real {z_ref}"
        );

        // The policy block equals the policy-only method (fusion is sound).
        let (_zp, g_theta) =
            coupling().coupled_trajectory_policy_gradient(&LinearFeedback, &theta, n);
        for (&gj, &gp) in dz_dtheta.iter().zip(&g_theta) {
            assert!((gj - gp).abs() < 1e-14, "joint θ {gj} != policy-only {gp}");
        }
    }
}
