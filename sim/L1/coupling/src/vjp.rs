//! The `VjpOp` tape-node adjoint family for the coupled soft↔rigid crossing —
//! the per-node vector-Jacobian products that let ONE `tape.backward` cross both
//! engines, plus the rigid-dynamics math helpers they and
//! [`crate::StaggeredCoupling`] share (mass inverse, `xfrc` response column,
//! SO(3) right-Jacobian, twist basis, wrench assembly).

use sim_core::{DMatrix, Data, Matrix3, Model, SpatialVector, mj_jac_point};
use sim_ml_chassis::Tensor;
use sim_ml_chassis::autograd::VjpOp;
use sim_soft::{FrictionVertexForce, RigidTwist, Vec3};

/// Scatter the precomputed contact-force-vs-position factor `∂fz/∂x_v` onto `slot`,
/// scaled by the upstream scalar cotangent `cot`. `factors` is per active pair
/// `(vertex_id, ∂fz/∂x_v, ∂fz/∂height)` from [`StaggeredCoupling::active_pair_force_factors`](crate::StaggeredCoupling::active_pair_force_factors)
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
/// ([`StaggeredCoupling::rigid_vz_response`](crate::StaggeredCoupling::rigid_vz_response)).
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
pub(super) fn implicit_mass_inverse(model: &Model, data: &Data) -> DMatrix<f64> {
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
/// [`mj_differentiate_pos`] reads, which the FD [`StaggeredCoupling::loaded_state_jacobian`](crate::StaggeredCoupling::loaded_state_jacobian)
/// also uses), as distinct from the `h·I` / left-Jacobian forms
/// [`sim_core::mjd_quat_integrate`] returns for its own (tangent-at-`q_old`) convention.
/// Reduces to `I` as `θ → 0` (the linear / hinge limit). FD-validated against the real
/// quaternion step in `tests/`.
pub(super) fn right_jacobian_so3(phi: Vec3) -> Matrix3<f64> {
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
/// closed-form `dt/m` (S2-validated); [`StaggeredCoupling::rigid_vz_response`](crate::StaggeredCoupling::rigid_vz_response)
/// computes it by central FD over [`StaggeredCoupling::rigid_step_probe`](crate::StaggeredCoupling::rigid_step_probe) so the
/// op is not hard-wired to the free-body case.
#[derive(Clone, Copy, Debug)]
pub struct RigidStepVjp {
    dvz_dfz: f64,
}

impl RigidStepVjp {
    /// Construct from the rigid response factor `∂vz'/∂xfrc_z` (e.g. from
    /// [`StaggeredCoupling::rigid_vz_response`](crate::StaggeredCoupling::rigid_vz_response)).
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
pub(super) struct VelVjp {
    pub(super) inv_dt: f64,
    pub(super) n_dof: usize,
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
/// [`StaggeredCoupling::active_pair_force_factors`](crate::StaggeredCoupling::active_pair_force_factors) (curvature-correct for any collider):
/// `∂fz/∂x* = −cᵥ·n̂_z·n̂ + f_mag·H·ẑ` (the S3 factor + #415's geometric stiffness) and
/// `∂fz/∂z = ∂fz/∂height = Σ (cᵥ·n̂_z² − f_mag·(H·ẑ)_z)` (the S1 explicit factor,
/// `∂height/∂z=1`). `H = 0` for the plane, where `n̂ = −ẑ` ⇒ this reduces to the flat
/// `−cᵥ·n̂⊗n̂` / `+Σ cᵥ` (penalty `cᵥ = κ`, IPC `κ·b''(sd)`), byte-identical.
#[derive(Clone, Debug)]
pub(super) struct ContactForceTrajVjp {
    /// `(vertex_id, ∂fz/∂x_v, ∂fz/∂height)` per active contact pair, from
    /// [`StaggeredCoupling::active_pair_force_factors`](crate::StaggeredCoupling::active_pair_force_factors).
    pub(super) factors: Vec<(usize, Vec3, f64)>,
    pub(super) n_dof: usize,
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
pub(super) struct VzCarryVjp {
    pub(super) a: f64,
    pub(super) neg_dt_over_m: f64,
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
pub(super) struct VzControlCarryVjp {
    pub(super) a: f64,
    pub(super) neg_dt_over_m: f64,
    pub(super) dt_over_m: f64,
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
pub(super) struct ZCarryVjp {
    pub(super) dt: f64,
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
pub(super) struct DriftFromVelVjp {
    pub(super) dt: f64,
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
/// ([`StaggeredCoupling::com_linear_jacobian`](crate::StaggeredCoupling::com_linear_jacobian)), so `∂Δ_surf,x/∂qvel = dt·J_lin[0,:]` (the
/// x-row) and `∂Δ_surf,x/∂qpos = 0` (the drift is linear in `qvel` at the linearization
/// config; `J_lin(q)` enters only at second order, captured by the per-step fresh FK). One
/// parent `[s]` (the rigid state `[qpos(nv); qvel(nv)]`, `[2·nv]`), output `Δ_surf,x` (`[1]`).
#[derive(Clone, Debug)]
pub(super) struct DriftFromStateVjp {
    pub(super) dt: f64,
    /// The x-row of the COM linear Jacobian `J_lin[0,:]` (length `nv`).
    pub(super) jlin_x: Vec<f64>,
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
/// [`DiffPolicy`](crate::DiffPolicy) observes (e.g. a single hinge's joint angle `s[0]` or rate `s[nv]`), so the
/// closed-loop control `u = π_θ(state)` feeds back through the matrix carry.
#[derive(Clone, Copy, Debug)]
pub(super) struct StateComponentVjp {
    /// The component index into `s = [qpos(nv); qvel(nv)]`.
    pub(super) idx: usize,
    /// The state length `2·nv` (parent shape).
    pub(super) n_state: usize,
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

/// Chassis-tape [`VjpOp`] reading one **quaternion vector component** `q_vec[axis]` (`qx`/`qy`/`qz`)
/// of the free body's final orientation from the rigid state `s` — the orientation target that
/// exercises the multi-DOF carry's *position* rows (`G_pos = Δt·J_r`), which the velocity-only
/// `ω_N` readout ([`StateComponentVjp`] on a `qvel` row) leaves untouched. The quaternion component
/// is smooth in the rotation (no angle-wrap, unlike `‖log q‖`), and its body-frame tangent
/// derivative is closed-form: for `q = [w, v]` a right perturbation `q ⊗ exp(δ)` gives
/// `∂q_vec/∂δ = ½(w·I + [v]_×)`, so `∂(q_vec[axis])/∂δ` is that matrix's `axis` row
/// (`grad_tang`, precomputed at the final orientation). One parent `s` (`[2·nv]`), scalar output;
/// `∂L/∂s[ang_dof + k] += cot · grad_tang[k]` over the joint's 3 ANGULAR tangent DOFs (the
/// `qpos` tangent rows the position carry feeds), translation/`qvel` rows untouched.
#[derive(Clone, Copy, Debug)]
pub(super) struct QuatComponentVjp {
    /// Start index of the free joint's angular tangent DOFs in `s` (`jnt_dof_adr + 3`).
    pub(super) ang_dof: usize,
    /// `∂(q_vec[axis])/∂(body angular tangent)` — the `axis` row of `½(w·I + [v]_×)` at the
    /// final orientation.
    pub(super) grad_tang: Vec3,
    /// The state length `2·nv` (parent shape).
    pub(super) n_state: usize,
}

impl VjpOp for QuatComponentVjp {
    fn op_id(&self) -> &'static str {
        "sim_coupling::QuatComponentVjp"
    }

    // Shape mismatches are programmer bugs (wrong node shape pushed) — assert.
    #[allow(clippy::panic)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        assert!(
            cotangent.shape() == [1]
                && parent_cotans.len() == 1
                && parent_cotans[0].shape() == [self.n_state],
            "QuatComponentVjp: expected cot [1] + 1 parent (s [{}])",
            self.n_state,
        );
        let c = cotangent.as_slice()[0];
        let s = parent_cotans[0].as_mut_slice();
        s[self.ang_dof] += c * self.grad_tang.x;
        s[self.ang_dof + 1] += c * self.grad_tang.y;
        s[self.ang_dof + 2] += c * self.grad_tang.z;
    }
}

/// Chassis-tape [`VjpOp`] for the **tangential friction-reaction** readout along a
/// trajectory: the scalar `fx = (Σ ∇D)·react_dir` (the rigid-side reaction `F·react_dir`
/// from [`CpuNewtonSolver::friction_reaction_gradients`](sim_soft::CpuNewtonSolver::friction_reaction_gradients)) at the post-step soft config `x*`,
/// with the moving-collider drift `Δ_surf` and the plane at height `z − clearance` — the
/// friction successor to [`ContactForceTrajVjp`] (which reads the normal `fz`). The caller
/// passes `react_dir = −x̂` so this equals `force_on_soft.x`, letting the tangential rigid
/// carry reuse the normal carry's `−(Δt/m)·fx` reaction sign. Parents
/// `[x*, z, Δ_surf, x_prev]` (`[n_dof]`, `[1]`, `[1]`, `[n_dof]`), output `fx` (`[1]`). The
/// sensitivities are precomputed once by [`CpuNewtonSolver::friction_reaction_gradients`](sim_soft::CpuNewtonSolver::friction_reaction_gradients):
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
pub(super) struct FrictionReactionTrajVjp {
    pub(super) dforce_dx: Vec<f64>,
    pub(super) dforce_dxprev: Vec<f64>,
    pub(super) dforce_dheight: f64,
    pub(super) dforce_ddrift: f64,
    pub(super) dforce_dmu_c: Option<f64>,
    pub(super) n_dof: usize,
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
pub(super) struct PoseSeamVjp {
    /// `J_z`: the world-z row of the contact-point body Jacobian (length `nv`).
    pub(super) jz: Vec<f64>,
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
/// tip, [`StaggeredCoupling::with_contact_geom`](crate::StaggeredCoupling::with_contact_geom)), its centre translates in x/y/z as the
/// body moves, so the pose channel is the 3-vector centre `c = geom_xpos(q)` rather than
/// the scalar height. Threads `∂L/∂qpos = J_geomᵀ·∂L/∂centre` (position half;
/// `∂centre/∂qvel = 0`, velocity half untouched), where `J_geom = ∂centre/∂qpos` is
/// [`StaggeredCoupling::pose_centre_jacobian`](crate::StaggeredCoupling::pose_centre_jacobian) (the geom's linear world Jacobian).
#[derive(Debug)]
pub(super) struct PoseCentreVjp {
    /// `J_geom`: the contact-geom centre's linear world Jacobian (`3 × nv`).
    pub(super) j_geom: DMatrix<f64>,
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
/// matching [`StaggeredCoupling::pose_twist_jacobian`](crate::StaggeredCoupling::pose_twist_jacobian)'s row layout (`mj_jac_point`
/// rows 0–2 angular, 3–5 linear). The rotating-normal pose channel is expressed in
/// this rigid-agnostic basis (the soft pose-adjoint and the wrench `∂w/∂T`); the
/// [`PoseTwistSeamVjp`] seam then maps the twist cotangent through `J_spatial`.
pub(super) fn twist_basis() -> [RigidTwist; 6] {
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
pub(super) struct PoseTwistSeamVjp {
    /// `J_spatial`: the body's spatial Jacobian at the world origin (`6 × nv`,
    /// `mj_jac_point` rows 0–2 angular / 3–5 linear). `∂T/∂qpos`.
    pub(super) j_spatial: DMatrix<f64>,
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
pub(super) struct ContactWrenchTrajVjp {
    /// Per active pair `(vertex_id, soft-force gᵢ, normal n̂, curvature cᵥ,
    /// moment arm d = rᵢ − c, collider normal-curvature Hᵢ = ∇²sd)` at the
    /// linearization config. `Hᵢ = 0` for the constant-normal plane; for a finite curved
    /// collider it carries the geometric-stiffness term `f_mag·H` of `∂gᵢ/∂x_v` and `∂gᵢ/∂h`.
    pub(super) active: Vec<(usize, Vec3, Vec3, f64, Vec3, Matrix3<f64>)>,
    /// The reaction force `f = −Σ gᵢ` (cached for `∂τ/∂c = [f]_×`).
    pub(super) force: Vec3,
    /// The COM linear Jacobian `J_lin = ∂c/∂qpos` (`3 × nv`).
    pub(super) jlin: DMatrix<f64>,
    /// Soft-DOF count `3·n_vertices` (parent `x*` length).
    pub(super) n_dof: usize,
    /// Rigid-DOF count `nv` (parent `s` length is `2·nv`).
    pub(super) nv: usize,
    /// The pose channel: scalar height `h` (flat) or 6-DOF twist `T` (rotating).
    pub(super) pose: WrenchPose,
}

/// The contact-wrench node's pose dependence — the middle parent of
/// [`ContactWrenchTrajVjp`]. `Height` is the flat scalar-`h` factor (parent `[1]`);
/// `Twist` is the rotating-normal 6-DOF spatial-twist factor (parent `[6]`).
#[derive(Clone, Debug)]
pub(super) enum WrenchPose {
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
/// analytically for a single hinge, [`StaggeredCoupling::analytic_state_jacobian`](crate::StaggeredCoupling::analytic_state_jacobian),
/// else by FD, [`StaggeredCoupling::loaded_state_jacobian`](crate::StaggeredCoupling::loaded_state_jacobian)), and `G = ∂(state')/∂w` is the
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
pub(super) struct RigidStateCarryVjp {
    /// The loaded transition Jacobian `J_state` (`2·nv × 2·nv`), indexed logically `j_state[(row, col)]`.
    pub(super) j_state: DMatrix<f64>,
    /// The wrench velocity response `G_vel = ∂(qvel')/∂w` (`nv × 6` = [`rigid_xfrc_column`]).
    pub(super) g_vel: DMatrix<f64>,
    /// The wrench POSITION-row response `G_pos = ∂(tangent qpos')/∂w = D·G_vel` (`nv × 6`),
    /// `D` the integrator's tangent Jacobian (`Δt·I` Euclidean, `Δt·J_r` quaternion).
    pub(super) g_pos: DMatrix<f64>,
    /// Optional ACTUATOR-control channel `G_act = ∂s'/∂ctrl` (`2·nv × nu`): the
    /// actuator drives `s' = J_state·s + G·w + G_act·u` where `u = ctrl`. When `Some`,
    /// the node takes a third parent `u` (`[nu]`) and `∂L/∂u[a] += Σᵢ G_act[(i,a)]·cot[i]`.
    /// `None` (the material/passive path) ⇒ two parents, byte-identical to the pre-actuator
    /// carry. Rows are `[G_act_pos; G_act_vel]` with `G_act_vel = Δt·M_impl⁻¹·∂qfrc_act/∂ctrl`
    /// and `G_act_pos = D·G_act_vel` (the same integrator tangent `D` as `G_pos`).
    pub(super) g_act: Option<DMatrix<f64>>,
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
pub(super) struct FrictionWrenchVert {
    /// The contacted soft vertex.
    pub(super) vid: usize,
    /// `∇D_v` — the friction force on the rigid body at `v`.
    pub(super) force: Vec3,
    /// `d_v = r_v − c` — the moment arm about the COM `c` (`r_v = x*_v`, the soft vertex pos).
    pub(super) arm: Vec3,
    /// `∂force/∂x*` — row-major `3 × n_dof` ([`sim_soft::FrictionVertexForce::dforce_dx`]).
    pub(super) dforce_dx: Vec<f64>,
    /// `∂force/∂x_prev` — row-major `3 × n_dof`.
    pub(super) dforce_dxprev: Vec<f64>,
    /// `∂force/∂Δ_surf` along the build's `drift_dir`.
    pub(super) dforce_ddrift: Vec3,
    /// `∂force/∂(centre·e_k)` per pose-basis axis — `[∂force/∂height]` (length 1) for the scalar
    /// height channel, or the 3 translation axes `[x̂, ŷ, ẑ]` for a moving end-effector centre.
    pub(super) dforce_dpose: Vec<Vec3>,
    /// `∂force/∂μ_c = ∇D_v/μ_c` — the DIRECT Coulomb-coefficient channel (only threaded when the
    /// node carries a `μ_c` parent, i.e. the friction-coefficient gradient).
    pub(super) dforce_dmu_c: Vec3,
}

/// Chassis-tape [`VjpOp`] folding the full TANGENTIAL friction GRIP wrench into the spatial
/// wrench: `w = w_normal + [τ_fric; f_fric]` with `f_fric = Σ_v ∇D_v` and the off-COM friction
/// MOMENT `τ_fric = Σ_v (r_v − c) × ∇D_v`. The moment successor to the force-only fold: it
/// threads the per-vertex friction Jacobians ([`sim_soft::FrictionVertexForce`], from
/// [`CpuNewtonSolver::friction_force_jacobians`](sim_soft::CpuNewtonSolver::friction_force_jacobians)) through BOTH the force AND moment rows.
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
pub(super) struct FrictionWrenchTrajVjp {
    /// Per active friction vertex.
    pub(super) verts: Vec<FrictionWrenchVert>,
    /// `f_fric_total = Σ ∇D_v` — the total friction force (for the COM `c(q)` feedback).
    pub(super) f_total: Vec3,
    /// The COM linear Jacobian `J_lin = ∂c/∂qpos` (`3 × nv`).
    pub(super) jlin: DMatrix<f64>,
    /// Soft-DOF count `3·n_vertices`.
    pub(super) n_dof: usize,
    /// Rigid-DOF count `nv`.
    pub(super) nv: usize,
    /// Pose-parent component count: `1` for the scalar height channel, `3` for a moving
    /// end-effector centre (`[x̂, ŷ, ẑ]`). Must equal every vertex's `dforce_dpose.len()`.
    pub(super) n_pose: usize,
    /// When `true`, a 7th parent `μ_c` (`[1]`) is present and the `∂L/∂μ_c` direct channel is
    /// threaded; `false` ⇒ 6 parents, no `μ_c` channel.
    pub(super) mu_c: bool,
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
/// Accumulate the off-COM contact **moment** of a single reaction force `f` applied at world
/// point `r`, about the COM `c`, into a wrench's angular part (`w[0..3]` of the `[angular; linear]`
/// [`SpatialVector`] layout): `τ += (r − c) × f`. The single source of the `(r − c) × f` moment
/// convention (reference point, sign, index layout) shared by the forward reaction-wrench builders
/// [`StaggeredCoupling::contact_wrench`](crate::StaggeredCoupling::contact_wrench), `contact_wrench_gripped`, and the free-body `step_core`,
/// so a change to the definition lives in one place.
pub(super) fn add_contact_moment(w: &mut SpatialVector, r: Vec3, f: Vec3, c: Vec3) {
    let tau = (r - c).cross(&f);
    w[0] += tau.x;
    w[1] += tau.y;
    w[2] += tau.z;
}

pub(super) fn assemble_friction_wrench(
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
