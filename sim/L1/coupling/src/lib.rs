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

use sim_core::{Data, Model, SpatialVector};
use sim_ml_chassis::autograd::VjpOp;
use sim_ml_chassis::{Tape, Tensor, Var};
use sim_soft::{
    ActivePairsFor, BoundaryConditions, ContactModel, ContactPair, ContactPairReadout,
    CpuNewtonSolver, HandBuiltTetMesh, IpcRigidContact, LoadAxis, MaterialField, Mesh, NeoHookean,
    PenaltyRigidContact, RigidPlane, Solver, SolverConfig, Tet4, Vec3, VertexId,
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
            _contact: PhantomData,
        }
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
        // Downward ceiling at `height`: normal −z, offset −height ⇒ a soft
        // vertex below the plane has positive signed distance `height − z`.
        let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), -height);
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
    /// `(vertex_id, outward normal n̂, curvature cᵥ)` where the local contact
    /// curvature `cᵥ = d²E/dsd² = n̂ᵀ·H_pair·n̂` is read from the contact's Hessian
    /// (`κ` for penalty, `κ·b''(sd)` for IPC) — the general per-pair stiffness the
    /// contact-force gradient factors use, in place of a hard-coded `κ`. Does not
    /// re-solve or mutate.
    fn active_pair_curvatures(&self, height: f64, positions: &[Vec3]) -> Vec<(usize, Vec3, f64)> {
        let contact = self.build_contact(height);
        contact
            .pair_readout(&self.fresh_mesh(), positions)
            .iter()
            .map(|r| {
                let ContactPair::Vertex { vertex_id, .. } = r.pair;
                // H_pair = cᵥ·n̂⊗n̂ ⇒ cᵥ = n̂·(H·n̂).
                let curv = contact
                    .hessian(&r.pair, positions)
                    .contributions
                    .first()
                    .map_or(0.0, |e| r.normal.dot(&(e.2 * r.normal)));
                (vertex_id as usize, r.normal, curv)
            })
            .collect()
    }

    /// Analytic `∂(total force_on_soft)/∂(plane height)` at the current soft
    /// configuration, holding the soft positions fixed.
    ///
    /// For the downward plane, each active pair has `∂force/∂height = −cᵥ·n̂` with
    /// curvature `cᵥ = d²E/dsd²` and unit normal `n̂ = −ẑ`, so the per-pair
    /// contribution is `+cᵥ·ẑ`; over the active set the total is `+(Σ cᵥ)·ẑ`. For
    /// penalty `cᵥ = κ` so this reduces to `κ·N_active·ẑ`; for IPC `cᵥ = κ·b''(sd)`.
    /// This is the explicit (fixed-position) partial — one factor of the coupled
    /// step's Jacobian, not the total settled-system derivative. Valid in the
    /// contact-engaged regime where the active set is stable across the
    /// perturbation; at the active-set boundary the penalty derivative is
    /// non-smooth (the IPC barrier smooths it). FD-checked against
    /// [`Self::contact_force_at_height`].
    #[must_use]
    pub fn contact_force_height_jacobian(&self, height: f64) -> Vec3 {
        let sum_curv: f64 = self
            .active_pair_curvatures(height, &self.positions())
            .iter()
            .map(|&(_, _, c)| c)
            .sum();
        Vec3::new(0.0, 0.0, sum_curv)
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
        let dxstar = solver.equilibrium_pose_sensitivity(&x_final, self.cfg.dt, dir);
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
