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
    DMatrix, DVector, Data, Matrix3, MjJointType, Model, SpatialVector,
    mass_directional_derivative, mj_differentiate_pos, mj_integrate_pos_explicit, mj_jac_point,
};
use sim_ml_chassis::autograd::VjpOp;
use sim_ml_chassis::{Tape, Tensor, Var};
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, MaterialField, PenaltyRigidContact, RigidTwist, Solver,
    SolverConfig, Tet4, Vec3, VertexId,
};
use std::marker::PhantomData;

mod construct;
mod contact;
mod contact_readout;
mod error;
mod freebody;
mod policy;
mod single_step;
mod step;
mod tangential;
mod types;
mod vjp;

pub use contact::PlaneContact;
pub use error::RolloutError;
pub use policy::{DiffPolicy, LinearFeedback};
pub use types::{CoupledStep, GripRolloutFrame, PolicyState, TrajectoryPeakPressure};
pub use vjp::{ContactForceVjp, RigidStepVjp, rigid_xfrc_column};

use contact::{Collider, SoftSolver};
use types::DesignPolicyTape;
use vjp::{
    ContactForceTrajVjp, ContactWrenchTrajVjp, DriftFromStateVjp, FrictionWrenchTrajVjp,
    PoseCentreVjp, PoseSeamVjp, PoseTwistSeamVjp, RigidStateCarryVjp, StateComponentVjp, VelVjp,
    VzControlCarryVjp, WrenchPose, ZCarryVjp, add_contact_moment, assemble_friction_wrench,
    implicit_mass_inverse, right_jacobian_so3, twist_basis,
};

/// A staggered forward coupling of a hand-built soft block (bottom face pinned)
/// and a `sim_core` rigid body that presses on it from above through a
/// body-posed downward penalty-contact plane.
///
/// The soft body is a [`HandBuiltTetMesh::uniform_block`](sim_soft::HandBuiltTetMesh::uniform_block) (Neo-Hookean); the
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
    /// When `true`, the free-body [`Self::step`] routes the off-COM contact **moment**
    /// `Σ (rᵢ − c) × (−gᵢ)` (about the body COM `c = xipos`) alongside the linear
    /// reaction, so an off-centre strike spins the body. Default `false` (the linear-only
    /// routing, byte-identical to the pre-moment coupling); enabled via
    /// [`Self::with_contact_moment`]. **Scaffolding, not a permanent knob** — the moment is
    /// correct physics (not an opt-in modeling choice like [`Self::with_rotating_normal`]),
    /// so the end state is moment-always-on with this flag retired, once the free-body
    /// trajectory gradients carry the wrench and the keystone scenes are re-centred. See
    /// [`Self::with_contact_moment`].
    contact_moment: bool,
    _contact: PhantomData<C>,
}

impl<C: PlaneContact> StaggeredCoupling<C> {
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
    // The `expect` enforces the documented `# Panics` contract (a centre Jacobian
    // is only meaningful for an end-effector-posed sphere); a misconfigured call
    // is a loud programmer error, not a recoverable condition.
    #[allow(clippy::expect_used)]
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
    /// **Linear-axis damping.** When `rigid_damping != 0` (the free-platen contact-axis damping
    /// the wrench carry now supports), the velocity-dependent damping force `−c·vz` is added on
    /// the world-z linear force from the SUPPLIED `qvel` — so a position/velocity FD over this
    /// step sees the damping vary with the perturbed state and the loaded `J_state` picks up its
    /// `a = 1 − c·Δt/m` velocity-coupling. The damped vz is `qvel[2]`, matching [`Self::step`]'s
    /// `sf[5] = −fz − c·qvel[2]` exactly (the free platen's z velocity; `rigid_damping` is the
    /// free-body knob, so this branch only fires for a free body). This is what lets the general
    /// carry reproduce the scalar `VzCarryVjp`/`ZCarryVjp` z-carry under damping. `c = 0` ⇒ the
    /// branch is skipped, byte-identical for the articulated/hinge paths (which assert `c = 0`).
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
        let mut w = *wrench;
        if self.rigid_damping != 0.0 {
            w[5] += -self.rigid_damping * qvel[2]; // velocity-dependent contact-axis damping (free platen z)
        }
        scratch.xfrc_applied[self.body] = w;
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
    /// platen (nv = 6), AND undamped serial-hinge chains (2-link and 3-link multi-hop — the
    /// `2link·material[μ]` / `3link·material[μ]` rows of `tests/coupling_grad_harness.rs`).
    /// The earlier long-rollout
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
    /// curvature-correct on a centroid sphere (FD-gated end-to-end by the
    /// `sphere-hinge·friction-material[μ]` row of the coupling gradient harness,
    /// `tests/coupling_grad_harness.rs`).
    ///
    /// MOVING END-EFFECTOR ([`Self::with_contact_geom`]): when the sphere rides the contact geom,
    /// the pose channel is the 3-vector centre (`PoseCentreVjp` + `WrenchPose::Centre` + the grip
    /// soft node's 3-axis pose + the friction wrench's 3-vector `dforce_dpose`), posed at
    /// `geom_xpos(q)` each step. The single-step lateral channels are machine-exact in
    /// `friction_sphere_tangent.rs`; the composition is gated by the
    /// `sphere-moving-ee·friction-material[μ]` / `sphere-moving-ee·friction-coeff[μ_c]` rows of the
    /// coupling gradient harness (`tests/coupling_grad_harness.rs`). No geom set ⇒ scalar-height
    /// channel (byte-identical).
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
    /// friction `dforce_dpose`); gated by the `sphere-moving-ee·friction-coeff[μ_c]` row of the
    /// coupling gradient harness (`tests/coupling_grad_harness.rs`).
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
                add_contact_moment(&mut wrench, r, f, c);
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
    /// # Errors
    /// Returns [`RolloutError`] if the per-step soft solve fails to converge (the
    /// infeasible-design fail-close, surfaced as a value for the `try_` gradient path).
    ///
    /// # Panics
    /// Panics if `params.len() != policy.n_params()`, if `nq != nv`, if `nu != 1`, if friction is
    /// inactive, if `rigid_damping != 0`, if the rotating normal is enabled, or if the *rigid* step
    /// diverges (a separate fail-closed surface from the soft solve).
    // One coherent per-step tape loop (fuses the friction-grip material design leaf and the closed-
    // loop policy leaves). expect_used: a rigid-step / FK divergence is a programmer error surfaced
    // loudly (the *soft* non-convergence is the recoverable fail-close — returned as `RolloutError`).
    #[allow(clippy::too_many_lines, clippy::expect_used)]
    fn build_design_policy_tape<P: DiffPolicy>(
        &mut self,
        policy: &P,
        params: &[f64],
        n_steps: usize,
    ) -> Result<DesignPolicyTape, RolloutError> {
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

        for k in 0..n_steps {
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
            // Fallible soft solve: a non-convergent step is the infeasible-design
            // fail-close — surfaced as `Err` (tagged with the rollout step) so the
            // `try_` gradient path returns it as a value. On success this is
            // byte-identical to the panic-path `replay_step` (same `solve_impl`).
            let x_next = solver
                .try_replay_step(
                    &Tensor::from_slice(&self.x, &[3 * n]),
                    &Tensor::from_slice(&self.v, &[3 * n]),
                    &Tensor::zeros(&[0]),
                    dt,
                )
                .map_err(|failure| RolloutError { step: k, failure })?
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
        Ok(DesignPolicyTape {
            tape,
            p_var,
            param_vars,
            qpos_steps,
            s_final: s_var,
            jx_final,
            tip_x,
        })
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
    /// inactive, if `rigid_damping != 0`, if the rotating normal is enabled, if the rigid step
    /// diverges, **or if the soft solve fails to converge** (the infeasible-design fail-close). Use
    /// [`Self::try_coupled_trajectory_design_policy_friction_gradient`] to handle that last case as a
    /// value instead of a panic.
    // panic: the fail-closed contract — this convenience wrapper re-panics the `try_` sibling's
    // `RolloutError` (the soft solver's own fail-close surface) for callers that want it loud.
    #[allow(clippy::panic)]
    #[must_use]
    pub fn coupled_trajectory_design_policy_friction_gradient<P: DiffPolicy>(
        &mut self,
        policy: &P,
        params: &[f64],
        n_steps: usize,
    ) -> (f64, f64, Vec<f64>) {
        self.try_coupled_trajectory_design_policy_friction_gradient(policy, params, n_steps)
            .unwrap_or_else(|e| panic!("{e}"))
    }

    /// Fallible sibling of [`Self::coupled_trajectory_design_policy_friction_gradient`]: returns the
    /// terminal-`tip_x` loss-gradient blocks, or [`RolloutError`] if a per-step soft solve fails to
    /// converge (the infeasible-design fail-close). On success the returned tuple is byte-identical to
    /// the panic-path method (same tape, same backward). The co-design optimizer routes through this
    /// to skip infeasible `(μ, θ)` instead of unwinding a panic.
    ///
    /// # Errors
    /// Returns [`RolloutError`] when an aggressive design/policy tears the coarse buffer at some
    /// rollout step — either a Newton iter-cap or a validity-domain violation (a tet over-stretching).
    /// See [`RolloutError`] for exactly which [`SolverFailure`](sim_soft::SolverFailure) variants convert to a value here vs
    /// the one residual (`ArmijoStall`) that still panics — the pending robustness work the grip does
    /// not reach.
    ///
    /// # Panics
    /// Still panics on the *non-recoverable* preconditions (`params.len() != policy.n_params()`,
    /// `nq != nv`, `nu != 1`, friction inactive, `rigid_damping != 0`, rotating normal, or a rigid-step
    /// divergence) — those are programmer/scope errors, not infeasible designs.
    pub fn try_coupled_trajectory_design_policy_friction_gradient<P: DiffPolicy>(
        &mut self,
        policy: &P,
        params: &[f64],
        n_steps: usize,
    ) -> Result<(f64, f64, Vec<f64>), RolloutError> {
        let mut t = self.build_design_policy_tape(policy, params, n_steps)?;
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
        Ok((t.tip_x, mu_grad, theta_grad))
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
    /// Panics under the same conditions as [`Self::coupled_trajectory_design_policy_friction_gradient`]
    /// — including a non-convergent soft solve. Use
    /// [`Self::try_coupled_trajectory_design_policy_hold_gradient`] to handle the fail-close as a value.
    // panic: the fail-closed contract — re-panics the `try_` sibling's `RolloutError` for callers
    // that want the soft solver's fail-close loud (mirrors the friction sibling).
    #[allow(clippy::panic)]
    #[must_use]
    pub fn coupled_trajectory_design_policy_hold_gradient<P: DiffPolicy>(
        &mut self,
        policy: &P,
        params: &[f64],
        n_steps: usize,
        q_hold: f64,
    ) -> (f64, f64, Vec<f64>) {
        self.try_coupled_trajectory_design_policy_hold_gradient(policy, params, n_steps, q_hold)
            .unwrap_or_else(|e| panic!("{e}"))
    }

    /// Fallible sibling of [`Self::coupled_trajectory_design_policy_hold_gradient`]: returns the
    /// holding-cost `L = Σ (qₖ − q_hold)²` and its `(∂L/∂μ_total, ∂L/∂θ)` blocks, or [`RolloutError`]
    /// if a per-step soft solve fails to converge. On success the returned tuple is byte-identical to
    /// the panic-path method. The Hold co-design optimizer routes through this to survive the
    /// infeasible `(μ, θ)` an aggressive holding policy explores.
    ///
    /// # Errors
    /// Returns [`RolloutError`] when the soft buffer cannot be re-equilibrated at some rollout step.
    /// See [`RolloutError`] for which [`SolverFailure`](sim_soft::SolverFailure) variants convert vs still panic (same coverage
    /// as the fallible friction sibling).
    ///
    /// # Panics
    /// Still panics on the non-recoverable preconditions / a rigid-step divergence (see the fallible
    /// friction sibling).
    pub fn try_coupled_trajectory_design_policy_hold_gradient<P: DiffPolicy>(
        &mut self,
        policy: &P,
        params: &[f64],
        n_steps: usize,
        q_hold: f64,
    ) -> Result<(f64, f64, Vec<f64>), RolloutError> {
        let mut t = self.build_design_policy_tape(policy, params, n_steps)?;
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
        Ok((cost_val, mu_grad, theta_grad))
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
    /// oracle) by the `control` row of `tests/coupling_grad_harness.rs`.
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
    /// (the real re-rolled closed-loop oracle) by the `policy(θ)` row of
    /// `tests/coupling_grad_harness.rs`.
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
    /// rebuilt at `μ ± ε` (material block) and at `θ ± ε` (policy) by the `joint(μ+θ)`
    /// row of `tests/coupling_grad_harness.rs`; the fusion-soundness invariant (the
    /// policy block equals the standalone policy method) lives in
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
}

#[cfg(test)]
mod tests;
