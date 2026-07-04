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

use sim_core::{Data, Model};
use sim_soft::{MaterialField, PenaltyRigidContact, SolverConfig, Vec3, VertexId};
use std::marker::PhantomData;

mod articulated;
mod articulated_grad;
mod construct;
mod contact;
mod contact_readout;
mod control;
mod error;
mod freebody;
mod policy;
mod policy_grad;
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

use contact::Collider;

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

#[cfg(test)]
mod tests;
