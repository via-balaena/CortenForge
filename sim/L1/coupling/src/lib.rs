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

use sim_core::{
    DMatrix, DVector, Data, Matrix3, MjJointType, Model, SpatialVector, mj_differentiate_pos,
    mj_integrate_pos_explicit, mj_jac_point,
};
use sim_ml_chassis::autograd::VjpOp;
use sim_ml_chassis::{Tape, Tensor, Var};
use sim_soft::{
    ActivePairsFor, BoundaryConditions, ContactModel, ContactPair, ContactPairReadout,
    CpuNewtonSolver, HandBuiltTetMesh, IpcRigidContact, LoadAxis, MaterialField, Mesh, NeoHookean,
    PenaltyRigidContact, RigidPlane, RigidTwist, Solver, SolverConfig, Tet4, Vec3, VertexId,
};
use std::marker::PhantomData;

/// A rigid-primitive contact constructible from a downward plane + `(κ, d̂)`, with
/// an active-pair readout over the keystone block — the bridge that lets
/// [`StaggeredCoupling`] run over either [`PenaltyRigidContact`] (the stepping
/// stone) or [`IpcRigidContact`] (the C²-barrier successor). Defined here (a local
/// trait) so the soft-side contact types need no change.
pub trait PlaneContact: ContactModel + ActivePairsFor<NeoHookean> + Sized {
    /// Build the contact from a single downward plane and the penalty/barrier
    /// parameters `(κ, d̂)`.
    fn from_plane(plane: RigidPlane, kappa: f64, d_hat: f64) -> Self;
    /// Per-active-pair readout at `positions` (the inherent `per_pair_readout`,
    /// surfaced through the trait so the coupling can call it generically).
    fn pair_readout(&self, mesh: &HandBuiltTetMesh, positions: &[Vec3]) -> Vec<ContactPairReadout>;
}

impl PlaneContact for PenaltyRigidContact {
    fn from_plane(plane: RigidPlane, kappa: f64, d_hat: f64) -> Self {
        Self::with_params(vec![plane], kappa, d_hat)
    }
    fn pair_readout(&self, mesh: &HandBuiltTetMesh, positions: &[Vec3]) -> Vec<ContactPairReadout> {
        self.per_pair_readout(mesh, positions)
    }
}

impl PlaneContact for IpcRigidContact {
    fn from_plane(plane: RigidPlane, kappa: f64, d_hat: f64) -> Self {
        Self::with_params(vec![plane], kappa, d_hat)
    }
    fn pair_readout(&self, mesh: &HandBuiltTetMesh, positions: &[Vec3]) -> Vec<ContactPairReadout> {
        self.per_pair_readout(mesh, positions)
    }
}

/// The soft Newton solver the coupling builds per step, over contact type `C`.
type SoftSolver<C> = CpuNewtonSolver<Tet4, HandBuiltTetMesh, C, NeoHookean, 4, 1>;

/// Scatter the contact-force-vs-position Jacobian `∂fz/∂x_v = −cᵥ·n̂_z·n̂` (the
/// z-row of the `−cᵥ·n̂⊗n̂` per-pair Jacobian) onto `slot`, scaled by the upstream
/// scalar cotangent `cot`. `active` is per active pair `(vertex_id, n̂, curvature)`
/// where `curvature = d²E/dsd² = n̂ᵀ·H_pair·n̂` is the contact's local stiffness
/// (`κ` for penalty, `κ·b''(sd)` for IPC). Shared by [`ContactForceVjp`] (single-
/// step crossing) and [`ContactForceTrajVjp`] (which adds its own `∂fz/∂z` term).
fn scatter_dfz_dxstar(active: &[(usize, Vec3, f64)], cot: f64, slot: &mut [f64]) {
    for &(v, n, curv) in active {
        let g = -curv * n.z;
        slot[3 * v] += cot * g * n.x;
        slot[3 * v + 1] += cot * g * n.y;
        slot[3 * v + 2] += cot * g * n.z;
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
// expect_used: a singular mass matrix is a malformed-model programmer error
// surfaced loudly, mirroring `rigid_step_probe`'s divergence-panic rationale.
#[allow(clippy::expect_used)]
#[must_use]
pub fn rigid_xfrc_column(model: &Model, data: &Data, body: usize) -> DMatrix<f64> {
    // J at the body COM (xipos): 6×nv, rows 0–2 angular, 3–5 linear — the same
    // point and frame the integrator's `mj_apply_ft` uses for `xfrc_applied`.
    let jac = mj_jac_point(model, data, body, &data.xipos[body]); // 6 × nv
    // M_impl = M + Δt·D (the Euler `eulerdamp` matrix; D = 0 ⇒ M_impl = M exactly).
    let mut m_impl = data.qM.clone();
    for i in 0..model.nv {
        m_impl[(i, i)] += model.timestep * model.implicit_damping[i];
    }
    let m_impl_inv = m_impl
        .try_inverse()
        .expect("implicit mass matrix M + Δt·D must be invertible");
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
/// `force_on_soft.z` (shape `[1]`). The VJP applies the per-active-pair contact-force
/// Jacobian `∂fz/∂x_v = −cᵥ·n̂_z·n̂` (the S3 `−cᵥ·n̂⊗n̂` factor, z-row, where the
/// per-pair curvature `cᵥ = d²E/dsd²` is `κ` for penalty, `κ·b''(sd)` for IPC),
/// turning a downstream `∂L/∂fz` into the `∂L/∂x*` cotangent that the soft
/// `NewtonStepVjp` then carries back to the soft parameters. Active set + curvature
/// captured at construction (engaged regime).
#[derive(Clone, Debug)]
pub struct ContactForceVjp {
    /// `(vertex_id, outward unit normal n̂, curvature cᵥ = d²E/dsd²)` for each
    /// active contact pair at the linearization positions.
    active: Vec<(usize, Vec3, f64)>,
}

impl ContactForceVjp {
    /// Construct from the active-pair `(vertex_id, normal, curvature)` list, where
    /// `curvature = d²E/dsd² = n̂ᵀ·H_pair·n̂` (`κ` for penalty, `κ·b''(sd)` for IPC).
    #[must_use]
    pub fn new(active: Vec<(usize, Vec3, f64)>) -> Self {
        Self { active }
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
        scatter_dfz_dxstar(&self.active, c, slot);
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
/// `fz = Σ force_on_soft.z` at the post-step soft config `x*` with the plane at
/// height `z − clearance`. Parents `[x_star, z]` (`[n_dof]`, `[1]`), output `fz`
/// (`[1]`). `∂fz/∂x* = −cᵥ·n̂_z·n̂` per active pair (the S3 factor) and
/// `∂fz/∂z = ∂fz/∂height = +Σ cᵥ` (the S1 explicit factor, `∂height/∂z=1`), where
/// the per-pair curvature `cᵥ = d²E/dsd²` is `κ` for penalty (so `Σ cᵥ = κ·N`),
/// `κ·b''(sd)` for IPC.
#[derive(Clone, Debug)]
struct ContactForceTrajVjp {
    active: Vec<(usize, Vec3, f64)>,
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
        // ∂fz/∂x* = −cᵥ·n̂_z·n̂ (shared with ContactForceVjp).
        scatter_dfz_dxstar(&self.active, c, xstar[0].as_mut_slice());
        // ∂fz/∂z = +Σ cᵥ (∂height/∂z = 1) — the trajectory-only term.
        let sum_curv: f64 = self.active.iter().map(|&(_, _, curv)| curv).sum();
        z[0].as_mut_slice()[0] += c * sum_curv;
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
    /// moment arm d = rᵢ − c)` at the linearization config.
    active: Vec<(usize, Vec3, Vec3, f64, Vec3)>,
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
        for &(v, g, n, curv, d) in &self.active {
            let term =
                curv * n.dot(&cot_f) * n - g.cross(&cot_t) - curv * n.dot(&d.cross(&cot_t)) * n;
            xs[3 * v] += term.x;
            xs[3 * v + 1] += term.y;
            xs[3 * v + 2] += term.z;
        }

        // ∂L/∂pose: flat scalar `h` or the rotating-normal 6-DOF twist.
        let pose_slot = parent_cotans[1].as_mut_slice();
        match &self.pose {
            WrenchPose::Height => {
                // ∂L/∂h: the explicit force/moment-vs-height feedback (∂gᵢ/∂h = −cᵥ n̂).
                //   ∂f/∂h = Σ cᵥ n̂ ;  ∂τ/∂h = Σ cᵥ (d × n̂).
                let dh: f64 = self
                    .active
                    .iter()
                    .map(|&(_, _, n, curv, d)| curv * (n.dot(&cot_f) + d.cross(&n).dot(&cot_t)))
                    .sum();
                pose_slot[0] += dh;
            }
            WrenchPose::Twist { basis, com } => {
                // ∂L/∂T_k = −Σᵢ (∂gᵢ/∂T_k)·(cot_f + cot_τ×dᵢ), with
                //   dn_k = ω_k×n̂,  dsd_k = rᵢ·dn_k − v_k·n̂  (rᵢ = dᵢ + com),
                //   ∂gᵢ/∂T_k = −cᵥ·dsd_k·n̂ + (gᵢ·n̂)·dn_k.
                for (k, tw) in basis.iter().enumerate() {
                    let mut acc = 0.0_f64;
                    for &(_, g, n, curv, d) in &self.active {
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
        assert!(
            cotangent.shape() == [n]
                && parent_cotans.len() == 2
                && parent_cotans[0].shape() == [n]
                && parent_cotans[1].shape() == [6],
            "RigidStateCarryVjp: expected cot [2·nv] + parents (s [2·nv], w [6])",
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
        // ∂L/∂w[k] += Σᵢ G_vel[(i,k)]·cot[nv+i] (position rows of G are zero).
        let w_slot = parent_cotans[1].as_mut_slice();
        for k in 0..6 {
            let mut acc = 0.0;
            for i in 0..nv {
                acc += self.g_vel[(i, k)] * cot[nv + i]; // velocity rows: G_vel
                acc += self.g_pos[(i, k)] * cot[i]; // position rows: G_pos = D·G_vel
            }
            w_slot[k] += acc;
        }
    }
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

/// The platen state a [`DiffPolicy`] observes each step — the minimal observation
/// (height `z` and vertical velocity `vz`, both already loop-carried chassis-tape
/// scalar [`Var`]s). Richer observations (contact force, soft-state summaries) are
/// a documented follow-on; they would add fields here.
#[derive(Clone, Copy, Debug)]
pub struct PolicyState {
    /// Platen reference height `z` at the start of the step (`[1]`-shaped var).
    pub z: Var,
    /// Platen vertical velocity `vz` at the start of the step (`[1]`-shaped var).
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

    /// Read-only access to the rigid engine state.
    #[must_use]
    pub fn data(&self) -> &Data {
        &self.data
    }

    /// The contact plane's height for the current rigid pose (a downward
    /// half-space whose surface sits at the rigid body's reference point minus
    /// the clearance).
    fn plane_height(&self) -> f64 {
        self.data.xpos[self.body].z - self.contact_clearance
    }

    fn build_contact(&self, height: f64) -> C {
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
        C::from_plane(plane, self.kappa, self.d_hat)
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

    /// Per active contact pair at `positions` with the plane at `height`:
    /// `(vertex_id, outward normal n̂, curvature cᵥ)` — the contact-force gradient
    /// factors' per-pair stiffness. A projection of [`Self::active_pair_wrench_data`]
    /// (the single source of the per-pair readout + curvature extraction), dropping
    /// the force `gᵢ` and position `rᵢ` the moment path additionally needs.
    fn active_pair_curvatures(&self, height: f64, positions: &[Vec3]) -> Vec<(usize, Vec3, f64)> {
        self.active_pair_wrench_data(height, positions)
            .into_iter()
            .map(|(v, _g, n, curv, _r)| (v, n, curv))
            .collect()
    }

    /// Per active contact pair at `positions` with the plane at `height`:
    /// `(vertex_id, soft-force gᵢ, normal n̂, curvature cᵥ, world position rᵢ)` where
    /// the local curvature `cᵥ = d²E/dsd² = n̂ᵀ·H_pair·n̂` is read from the contact's
    /// Hessian (`κ` for penalty, `κ·b''(sd)` for IPC). The single source of the
    /// per-pair readout + curvature extraction (projected by
    /// [`Self::active_pair_curvatures`]); the moment routing + its
    /// [`ContactWrenchTrajVjp`] gradient additionally use the force `gᵢ` and contact
    /// point `rᵢ`. Does not re-solve/mutate.
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
                // H_pair = cᵥ·n̂⊗n̂ ⇒ cᵥ = n̂·(H·n̂) (same as active_pair_curvatures).
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
    #[must_use]
    pub fn contact_force_height_jacobian(&self, height: f64) -> Vec3 {
        // ∂force/∂height = Σ cᵥ·n̂.z·n̂ (= (0,0,Σcᵥ) for the flat n̂ = −ẑ).
        self.active_pair_curvatures(height, &self.positions())
            .iter()
            .map(|&(_, n, c)| c * n.z * n)
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
        let dir = Vec3::new(0.0, 0.0, 1.0);
        let (solver, x_final) = self.soft_resolve(height);
        let dxstar = solver.equilibrium_pose_sensitivity(
            &x_final,
            self.cfg.dt,
            RigidTwist::translation(dir),
        );
        let positions: Vec<Vec3> = x_final
            .chunks_exact(3)
            .map(|c| Vec3::new(c[0], c[1], c[2]))
            .collect();
        let mut explicit = Vec3::zeros();
        let mut implicit = Vec3::zeros();
        for (v, n_hat, curv) in self.active_pair_curvatures(height, &positions) {
            // ∂sd/∂h = −n̂·ẑ for translating the plane +ẑ (here +1, n̂=−ẑ).
            let dsd_dh = -n_hat.z;
            // explicit: ∂force/∂h|_x = −cᵥ·(∂sd/∂h)·n̂ per active pair.
            explicit += -curv * dsd_dh * n_hat;
            // implicit: (∂force/∂x = −cᵥ·n̂⊗n̂) · ∂x*/∂h.
            let dxs = Vec3::new(dxstar[3 * v], dxstar[3 * v + 1], dxstar[3 * v + 2]);
            implicit += -curv * n_hat.dot(&dxs) * n_hat;
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
        let active = self.active_pair_curvatures(height, &positions);
        let force_var = tape.push_custom(
            &[xstar_var],
            Tensor::from_slice(&[fz], &[1]),
            Box::new(ContactForceVjp::new(active)),
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
            let active = self.active_pair_curvatures(height, &positions);
            let fz_var = tape.push_custom(
                &[x_next_var, z_var],
                Tensor::from_slice(&[force_on_soft.z], &[1]),
                Box::new(ContactForceTrajVjp {
                    active,
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
    /// This is the **general fallback** path (any `nv`): the single-hinge scope uses the
    /// machine-exact analytic [`Self::analytic_state_jacobian`] instead, but a free joint
    /// or a multi-link chain uses this FD form (FD-carry precision ~1e-6; the analytic
    /// chain carry is the documented follow-on). It is also the reference the analytic
    /// form is FD-validated against; see `docs/keystone/geometric_stiffness_recon.md` and
    /// `docs/keystone/multilink_recon.md`.
    ///
    /// **Tangent-space FD (SO(3)-correct for quaternion joints).** When the body has a
    /// quaternion joint (`nq ≠ nv` — `qpos` holds un-normalized quaternion components on
    /// a curved manifold) the position-coordinate FD must move ALONG the manifold and
    /// difference in its tangent space, not a raw `qpos` add/subtract: position COLUMNS
    /// step via [`Self::loaded_state_jacobian_tangent`]'s [`mj_integrate_pos_explicit`]
    /// (`qpos ⊕ exp(±ε·e_c)`) and position ROWS difference via [`mj_differentiate_pos`]
    /// (the SO(3) log, the body-frame tangent at the nominal output). A purely Euclidean
    /// body (`nq == nv`: hinge/slide/translation only — the merged single-hinge*,
    /// multi-link-chain, and translation paths) keeps the raw FD verbatim, BYTE-IDENTICAL.
    /// (*the single hinge uses the analytic Jacobian and never reaches here.) See
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
    /// or any non-hinge case — those keep the FD [`Self::loaded_state_jacobian`]
    /// fallback (the analytic geometric stiffness is the single-hinge closed form;
    /// the multi-link off-diagonal Jacobian Hessian + the `∂M⁻¹/∂q` term are the
    /// documented follow-on).
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
    /// and `docs/keystone/moment_residual_recon.md`). It is also the clean base for the
    /// analytic multi-link CHAIN carry (the documented follow-on).
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
    /// **Undamped scope.** Returns `None` when the model has joint damping — the unloaded
    /// `A` from `transition_derivatives` has a subtle Euler-`eulerdamp` mismatch (≈2.6% on
    /// the hinge gradient), so the damped single hinge falls back to the FD
    /// [`Self::loaded_state_jacobian`] (which differentiates the real eulerdamp step and is
    /// damping-correct — the 2-link chain confirms it). The analytic *damped* single-hinge
    /// `J_state` (reconciling `A` with eulerdamp for machine-exactness) is the documented
    /// follow-on; see `docs/keystone/damped_joints_recon.md`.
    // expect_used: a transition-derivative / forward on a valid model does not fail; a
    // failure is a programmer error surfaced loudly (mirrors `scratch_state_step`).
    #[allow(clippy::expect_used)]
    fn analytic_state_jacobian(&self, wrench: &SpatialVector) -> Option<DMatrix<f64>> {
        let jnt = self.single_hinge()?;
        // Joint damping → fall back to the FD loaded Jacobian (damping-correct): the
        // unloaded `A` has a subtle Euler-eulerdamp mismatch the FD path avoids.
        if self.model.implicit_damping.iter().any(|&d| d != 0.0) {
            return None;
        }
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
        let vel_corr = dt / scratch.qM[(0, 0)] * geom_stiff; // ∂qvel'/∂qpos correction
        j[(1, 0)] += vel_corr; // velocity row, qpos col
        j[(0, 0)] += dt * vel_corr; // position row, qpos col (semi-implicit chain)
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
    /// gradient matches the full-coupled FD to ~1e-9 at n = 1…15 for the single hinge, a
    /// free-joint platen (nv = 6), AND a 2-link chain (nv = 2, at FD-carry precision
    /// ~1e-6 since a chain uses the FD `J_state`). The earlier long-rollout moment
    /// residual (~1e-3 at n = 10) and the 74%-at-n=2 multi-link error were the SAME
    /// defect: the stale-FK pose + §8a position-row drop was a self-consistent pair
    /// calibrated ONLY for nv = 1 (`∂qpos'/∂qvel = Δt·I`, false for a chain). See
    /// `docs/keystone/moment_residual_recon.md` §3f and `docs/keystone/multilink_recon.md`.
    ///
    /// The wrench node and pose seam are analytic (FD-validated machine-exact vs the real
    /// contact readout, `tests/`). Tested scope: single-hinge (nv = 1) and free-joint
    /// (nv = 6) at ~1e-9, and the 2-link chain (nv = 2) at FD-carry ~1e-6. Higher chains
    /// (nv > 2) work via the same FD `J_state` carry (FD-carry precision) but are not
    /// explicitly gated; the analytic CHAIN carry (the Jacobian Hessian + `∂M⁻¹/∂q`) for
    /// machine-exactness at nv > 1 is the documented follow-on (`multilink_recon.md`).
    /// **Joint damping** is supported: the Euler `eulerdamp` carry factor is
    /// `Δt·(M + Δt·D)⁻¹·Jᵀ` ([`rigid_xfrc_column`], `D = implicit_damping`), and the
    /// analytic single-hinge `J_state` uses the same `M_impl` (the unloaded `A` already
    /// carries the damping velocity-coupling) — FD-gated under damping for the hinge and
    /// 2-link chain. Out of scope: stiffness-implicit / non-Euler integrators
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
            let height = self.tip_plane_height();
            // Pose seam: the contact plane's pose from the rigid state. FLAT — the
            // scalar tip height `h` (`∂h/∂q = J_z`). ROTATING — the 6-DOF spatial twist
            // `T` of the body-attached plane (`∂T/∂qpos = J_spatial`, the
            // `PoseTwistSeamVjp`); its value is the all-zero perturbation at the
            // linearization point (only the cotangent is threaded). The soft node AND
            // the wrench node share this one pose node (as the flat path shares `h`).
            let rotating = self.rotating_normal;
            let pose_var = if rotating {
                tape.push_custom(
                    &[s_var],
                    Tensor::zeros(&[6]),
                    Box::new(PoseTwistSeamVjp {
                        j_spatial: self.pose_twist_jacobian(),
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
            // FLAT the single scalar translation along `dir`.
            let soft_vjp: Box<dyn VjpOp> = if rotating {
                Box::new(solver.trajectory_step_vjp_twist(&x_next, dt, param_idx, &twist_basis()))
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
            let mut active: Vec<(usize, Vec3, Vec3, f64, Vec3)> = Vec::with_capacity(pairs.len());
            for (v, g, n, curv, r) in pairs {
                let f = -g;
                wrench[3] += f.x;
                wrench[4] += f.y;
                wrench[5] += f.z;
                let tau = (r - c).cross(&f);
                wrench[0] += tau.x;
                wrench[1] += tau.y;
                wrench[2] += tau.z;
                active.push((v, g, n, curv, r - c));
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
            //     the free body, real for the hinge). Single hinge ⇒ the machine-exact
            //     ANALYTIC `A + Δt·M⁻¹·∂(Jᵀw)/∂q` (`analytic_state_jacobian`); otherwise
            //     (free joint, multi-link chain) the FD `loaded_state_jacobian`.
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
            let g_pos = if self.model.nq == self.model.nv {
                dt * &g_vel
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
            let active = self.active_pair_curvatures(height, &positions);
            let fz_var = tape.push_custom(
                &[x_next_var, z_var],
                Tensor::from_slice(&[force_on_soft.z], &[1]),
                Box::new(ContactForceTrajVjp {
                    active,
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
            let active = self.active_pair_curvatures(height, &positions);
            let fz_var = tape.push_custom(
                &[x_next_var, z_var],
                Tensor::from_slice(&[force_on_soft.z], &[1]),
                Box::new(ContactForceTrajVjp {
                    active,
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
            let active = self.active_pair_curvatures(height, &positions);
            let fz_var = tape.push_custom(
                &[x_next_var, z_var],
                Tensor::from_slice(&[force_on_soft.z], &[1]),
                Box::new(ContactForceTrajVjp {
                    active,
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

    /// The generalized contact-force factors use PER-PAIR curvature `cᵥ = d²E/dsd²`
    /// (`κ·b''(sd)` for IPC), not just penalty's constant `κ`: `ContactForceVjp`
    /// scatters `−cᵥ·n̂_z·n̂` per pair, and `ContactForceTrajVjp`'s `∂fz/∂z` sums
    /// `cᵥ`. Distinct per-pair curvatures verify the generalization beyond the
    /// constant-`κ` penalty path the keystone gates exercise.
    #[test]
    fn contact_force_factors_use_per_pair_curvature() {
        let n = Vec3::new(0.0, 0.0, -1.0); // plane normal −ẑ
        let active = vec![(0_usize, n, 2.0), (1_usize, n, 5.0)]; // distinct cᵥ
        // ContactForceVjp: ∂fz/∂x_v z-component = −cᵥ·n̂_z·n̂_z = −cᵥ (cot=1).
        let mut parent = vec![Tensor::zeros(&[6])];
        ContactForceVjp::new(active.clone()).vjp(&Tensor::from_slice(&[1.0], &[1]), &mut parent);
        let g = parent[0].as_slice();
        assert!(
            (g[2] + 2.0).abs() < 1e-12 && (g[5] + 5.0).abs() < 1e-12,
            "∂fz/∂x* should carry per-pair curvature, got {g:?}"
        );
        // ContactForceTrajVjp: ∂fz/∂z = Σ cᵥ = 7.
        let traj = ContactForceTrajVjp { active, n_dof: 6 };
        let mut parents = vec![Tensor::zeros(&[6]), Tensor::zeros(&[1])];
        traj.vjp(&Tensor::from_slice(&[1.0], &[1]), &mut parents);
        assert!(
            (parents[1].as_slice()[0] - 7.0).abs() < 1e-12,
            "∂fz/∂z should be Σ cᵥ = 7, got {}",
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

    /// A DAMPED single hinge declines the analytic `J_state` (returns `None`) and falls
    /// back to the FD `loaded_state_jacobian`: the unloaded `A` from
    /// `transition_derivatives` has a subtle Euler-`eulerdamp` mismatch (≈2.6% on the
    /// hinge gradient), whereas the FD path differentiates the real eulerdamp step and is
    /// damping-correct (the 2-link chain confirms it). The analytic *damped* single-hinge
    /// is the documented follow-on. See `docs/keystone/damped_joints_recon.md`.
    #[test]
    fn analytic_state_jacobian_declines_under_damping() {
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
        data.forward(&model).expect("forward");
        assert!(model.implicit_damping[0] > 0.0, "damping must be live");
        let c: StaggeredCoupling = StaggeredCoupling::new(
            model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
        );
        assert!(
            c.single_hinge().is_some(),
            "geometry is a single hinge (the analytic path's predicate)"
        );
        assert!(
            c.analytic_state_jacobian(&SpatialVector::from_row_slice(&[
                0.1, 0.2, 0.0, 5.0, -3.0, 800.0
            ]))
            .is_none(),
            "a DAMPED single hinge must decline the analytic J_state (FD fallback)"
        );
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
            active.push((v, g, n, curv, r - com));
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
