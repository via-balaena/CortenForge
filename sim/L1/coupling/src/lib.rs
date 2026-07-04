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
    BoundaryConditions, ContactPair, ContactPairReadout, CpuNewtonSolver, HandBuiltTetMesh,
    LoadAxis, MaterialField, Mesh, PenaltyRigidContact, RigidPlane, RigidTwist, Sdf, Solver,
    SolverConfig, Tet4, Vec3, VertexId, peak_contact_pressure,
};
use std::marker::PhantomData;

mod contact;
mod error;
mod policy;
mod types;
mod vjp;

pub use contact::PlaneContact;
pub use error::RolloutError;
pub use policy::{DiffPolicy, LinearFeedback};
pub use types::{CoupledStep, GripRolloutFrame, PolicyState, TrajectoryPeakPressure};
pub use vjp::{ContactForceVjp, RigidStepVjp, rigid_xfrc_column};

use contact::{Collider, SoftSolver, posed_sphere};
use types::DesignPolicyTape;
use vjp::{
    ContactForceTrajVjp, ContactWrenchTrajVjp, DriftFromStateVjp, DriftFromVelVjp,
    FrictionReactionTrajVjp, FrictionWrenchTrajVjp, PoseCentreVjp, PoseSeamVjp, PoseTwistSeamVjp,
    QuatComponentVjp, RigidStateCarryVjp, StateComponentVjp, VelVjp, VzCarryVjp, VzControlCarryVjp,
    WrenchPose, ZCarryVjp, add_contact_moment, assemble_friction_wrench, implicit_mass_inverse,
    right_jacobian_so3, twist_basis,
};

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
            contact_moment: false,
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

    /// Route the off-COM contact **moment** in the free-body [`Self::step`]: on top of the
    /// linear reaction `−Σ gᵢ`, fill the wrench's angular part with `τ = Σ (rᵢ − c) × (−gᵢ)`
    /// about the body COM `c = xipos` (`gᵢ = force_on_softᵢ`, `rᵢ` the contact point). With
    /// this on, an **off-centre** strike spins the free body (the angular impulse a torque-free
    /// `[0; f]` routing drops); a centred strike is moment-free in the continuum (the corner-origin
    /// tet mesh is not mirror-symmetric, so a small discretisation residual remains). The same
    /// moment definition `contact_wrench` / [`Self::step_articulated`] already route — here
    /// reduced from [`Self::step`]'s own contact readouts (no second contact eval).
    ///
    /// Opt-in, default `false` (linear-only routing, **byte-identical** to the pre-moment
    /// coupling), because turning it on changes the rotational physics of every free-body scene
    /// and the flat-tuned keystone fixtures are geometrically off-centre (the soft block's
    /// contact centroid `(edge/2, edge/2)` sits ~`edge/2` from the platen COM at the origin), so
    /// a large undamped tipping moment would diverge them (`rigid_damping` damps only the linear
    /// contact axis). Exercised by a re-centred off-centre-strike gate.
    ///
    /// **Scaffolding, not a permanent modeling knob.** Unlike [`Self::with_rotating_normal`] (a
    /// legitimate normal-tracking approximation with regimes where off is correct), the contact
    /// moment is simply correct-vs-wrong physics: a free body under an off-centre force *must*
    /// gain angular velocity. The target end state is moment-always-on with this flag deleted,
    /// once the free-body trajectory gradients carry the wrench (mirroring `ContactWrenchTrajVjp`)
    /// and the keystone scenes are re-centred (an x,y-translation that is byte-identical under the
    /// current linear-only routing). Default off keeps the forward step and its force-only
    /// gradient *consistent* in the interim.
    #[must_use]
    pub const fn with_contact_moment(mut self, on: bool) -> Self {
        self.contact_moment = on;
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
    /// `sphere_moving_ee_{,friction_}trajectory_gradient.rs` + the `moving-ee·actuator` (pose-sensitive
    /// `tip_z` discriminator), `moving-ee·actuator-friction`, `moving-ee·policy-friction`, and
    /// `moving-ee·design-policy-friction` (the grip co-design μ+θ) rows of `coupling_grad_harness.rs`);
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

    /// Per-active-pair readouts at the current soft configuration with the
    /// collider posed at `height` — does not re-solve or mutate. The shared
    /// building block for the total-force readout, its pose-sensitivity, and
    /// the peak-pressure readout (so all three see the same posed contact).
    fn pair_readouts_at_height(&self, height: f64) -> Vec<ContactPairReadout> {
        self.build_contact(height)
            .pair_readout(&self.fresh_mesh(), &self.positions())
    }

    /// `(total force_on_soft, active-pair count)` at the current soft
    /// configuration with the contact plane placed at `height` — does not
    /// re-solve or mutate. The shared building block for the contact-force
    /// readout and its analytic pose-sensitivity.
    fn contact_readout(&self, height: f64) -> (Vec3, usize) {
        let readout = self.pair_readouts_at_height(height);
        (readout.iter().map(|r| r.force_on_soft).sum(), readout.len())
    }

    /// Peak contact *pressure* (Pa) — the max per-contact-face stress over the
    /// active pairs ([`peak_contact_pressure`]) — at the current soft
    /// configuration with the collider posed at `height`. Companion to
    /// [`Self::contact_force_at_height`]; does not re-solve or mutate. The
    /// measured local concentration total force cannot see (a finite sphere:
    /// high peak pressure at low total force; a broad slab: the reverse).
    #[must_use]
    pub fn contact_peak_pressure_at_height(&self, height: f64) -> f64 {
        peak_contact_pressure(&self.pair_readouts_at_height(height))
    }

    /// Per-vertex contact pressure field (Pa) at the current soft configuration with the collider
    /// posed at `height` — the per-vertex form of [`Self::contact_peak_pressure_at_height`], for a
    /// pressure *field* (e.g. a deformed-surface heatmap). Length `n_vertices`, indexed by
    /// `VertexId`: each entry is the max per-face stress over that vertex's active pairs, `0.0`
    /// where the vertex is not in contact, or `f64::NAN` where it IS in contact but every pair is
    /// degenerate (zero tributary area) — the same NaN-degenerate sentinel
    /// [`peak_contact_pressure`] uses, so a degenerate contact reads off-nominal rather than as a
    /// danger-masking zero, and the field's finite max equals the scalar peak. Does not re-solve or
    /// mutate. Indices line up with `soft_positions` / `soft_boundary_faces`, so a renderer drops
    /// the values straight onto the surface mesh's vertices.
    ///
    /// ⚠ For a forward-replay heatmap that must match the just-solved deformation, use
    /// [`Self::step_with_pressure_field`] instead of calling this after [`Self::step`] — `step`
    /// advances the rigid body, so the live `plane_height` is one timestep ahead of the contact the
    /// field should describe.
    #[must_use]
    pub fn contact_vertex_pressures_at_height(&self, height: f64) -> Vec<f64> {
        self.vertex_pressures_from(&self.pair_readouts_at_height(height))
    }

    /// Scatter a set of active-pair readouts into a per-vertex pressure field (the shared core of
    /// [`Self::contact_vertex_pressures_at_height`] and [`Self::step_with_pressure_field`], so both
    /// see the same max-of-faces + NaN-degenerate reduction as [`peak_contact_pressure`]).
    fn vertex_pressures_from(&self, readouts: &[ContactPairReadout]) -> Vec<f64> {
        let mut field = vec![0.0_f64; self.n_vertices];
        let mut any_pair = vec![false; self.n_vertices];
        let mut any_finite = vec![false; self.n_vertices];
        for r in readouts {
            let ContactPair::Vertex { vertex_id, .. } = r.pair;
            let v = vertex_id as usize;
            any_pair[v] = true;
            if r.pressure.is_finite() {
                any_finite[v] = true;
                field[v] = field[v].max(r.pressure);
            }
        }
        // A vertex in contact whose every pair was degenerate (no finite pressure) is off-nominal —
        // the NaN sentinel, not a reassuring 0.0 (matches `peak_contact_pressure`'s altitude).
        for v in 0..self.n_vertices {
            if any_pair[v] && !any_finite[v] {
                field[v] = f64::NAN;
            }
        }
        field
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
            add_contact_moment(&mut wrench, r, f, c);
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
            add_contact_moment(&mut wrench, r, f, c);
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
    /// FD-validated against the full coupled step by the `load·plane`/`load·sphere`
    /// rows of `tests/coupling_grad_harness.rs`. Engaged, stable-active-set,
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
    /// `tests/coupled_trajectory_gradient.rs` for penalty and the `ipc-traj·material`
    /// rows of `tests/coupling_grad_harness.rs` for IPC). The earlier ~3e-4 "penalty
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

    /// The contacting body's free-joint `(qpos_adr, dof_adr)` start addresses — asserting the body
    /// carries exactly one free joint, so the spin/orientation readouts index its angular `qvel`
    /// (`dof_adr + 3 + axis`) and quaternion (`qpos_adr + 3..7`) at the right offsets even when the
    /// free body is not the model's first joint.
    fn free_joint_adrs(&self) -> (usize, usize) {
        let jnt = self.model.body_jnt_adr[self.body];
        assert!(
            self.model.body_jnt_num[self.body] == 1
                && self.model.jnt_type[jnt] == MjJointType::Free,
            "free-body wrench gradient needs the contacting body to be a single free joint"
        );
        (self.model.jnt_qpos_adr[jnt], self.model.jnt_dof_adr[jnt])
    }

    /// Build the **free-body wrench-carry** time-adjoint tape for one coupled impact rollout, and
    /// return `(tape, p_var, s_final, s_prev)` for an objective node to read the final state and
    /// `tape.backward`. The shared body of the orientation-target gradients
    /// ([`Self::coupled_trajectory_angular_velocity_gradient`] reads a `qvel` row of `s_final` for
    /// `ω_N`; [`Self::coupled_trajectory_orientation_gradient`] reads a quaternion component) — only
    /// the objective differs, so the per-step recurrence lives here once.
    ///
    /// Mirrors the force-only [`Self::coupled_trajectory_material_gradient`]'s stale-FK loop (the
    /// contact is posed at the one-step-lagged height, matching [`Self::step`]) but routes the full
    /// contact **wrench** `[τ; f]` (`ContactWrenchTrajVjp`) through the multi-DOF `RigidStateCarryVjp`
    /// carry `s' = J_state·s + G·w` with the free joint's tangent-space state (`J_state` =
    /// `loaded_state_jacobian`, `G_pos = Δt·J_r·G_vel`).
    ///
    /// **Fresh- vs stale-FK readout — and why both carries coexist.** `s_final = s_N` holds the
    /// freshly-integrated `qpos_N`; `s_prev = s_{N-1}` is the second-to-last state. The two matter
    /// because `step` leaves `xpos` lagging `qpos` by a step, so its `rigid_z = xpos.z = qpos_{N-1}.z`
    /// — the stale-FK height the force-only `z_N` path tracks lives in `s_prev`, NOT `s_final`. This
    /// is the bridge between the two free-body carries: the scalar `VzCarryVjp`/`ZCarryVjp` path
    /// ([`Self::coupled_trajectory_material_gradient`], the keystone z/force target) is the
    /// **z-special-case of this general wrench carry** — reading `s_prev`'s z-translation row here
    /// reproduces its `z_N` gradient to machine precision (proven in
    /// `wrench_carry_subsumes_scalar_z_carry`, both undamped and at `rigid_damping = 60`). They are
    /// one dynamics at two readout conventions, not redundant implementations.
    ///
    /// **Contact-axis damping.** The free-body linear-z damping `−c·vz` is routed in the forward
    /// (matching `step`'s `sf[5] = −fz − c·vz`) and captured in the loaded `J_state` (the
    /// velocity-dependent term `scratch_state_step` re-derives). `c = 0` ⇒ byte-identical to the
    /// undamped carry the ω/orientation gates use.
    ///
    /// # Panics
    /// Panics if `param_idx > 1`, the contacting body is not a single free joint,
    /// `with_contact_moment` is off, a moving end-effector is set, the rigid step diverges, or the
    /// soft solver does not converge.
    // One coherent per-step tape-construction loop (the soft/pose/wrench/carry nodes share the
    // recurrence's state); splitting it would scatter that shared state.
    // expect_used: a rigid-step divergence is a programmer error surfaced loudly (mirrors `step`).
    #[allow(clippy::too_many_lines, clippy::expect_used)]
    fn build_freebody_wrench_tape(
        &mut self,
        n_steps: usize,
        param_idx: usize,
    ) -> (Tape, Var, Var, Var) {
        self.require_no_moving_ee();
        assert!(
            param_idx <= 1,
            "material param index {param_idx} out of range (0 = μ, 1 = λ)"
        );
        // A gradient needs ≥ 1 step; this also keeps `s_prev` meaningful (with 0 steps the loop
        // never runs and `s_prev` would alias the initial state rather than `s_{N-1}`).
        assert!(n_steps >= 1, "free-body wrench gradient needs n_steps >= 1");
        // The forward always routes the contact MOMENT (the orientation target's whole source); a
        // moment-off `step` would leave the body unspun, so the gradient would not match the
        // caller's own rollout. Require the moment so forward and gradient agree.
        assert!(
            self.contact_moment,
            "free-body wrench gradient requires with_contact_moment(true) (the moment drives the spin)"
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
        let mut s0 = vec![0.0_f64; 2 * nv];
        for i in 0..nv {
            s0[i] = self.data.qpos[i];
            s0[nv + i] = self.data.qvel[i];
        }
        let mut s_var = tape.constant_tensor(Tensor::from_slice(&s0, &[2 * nv]));
        // Track the second-to-last state: after the loop `s_prev = s_{N-1}`, whose FK is the
        // stale `xpos.z` that `step`'s `rigid_z` returns (see the doc's fresh-vs-stale-FK note).
        let mut s_prev = s_var;

        for _ in 0..n_steps {
            // STALE-FK: `step` poses the contact at the one-step-lagged `xpos` (= the PREVIOUS
            // state `s_{k-1}`, which `s_prev` holds), so the pose seam — and the wrench's COM
            // `c = xipos` below — attach to `s_prev`, NOT the current `s_var`. (For the flat plane
            // this is invisible to ω/z, but the sphere's curved pose term `f_mag·H` reaches the
            // moment, so the lagged attribution is load-bearing — it is the n≥2 fix.)
            let height = self.plane_height();
            let pose_var = tape.push_custom(
                &[s_prev],
                Tensor::from_slice(&[height], &[1]),
                Box::new(PoseSeamVjp {
                    jz: self.pose_seam_jz(),
                }),
            );
            let (solver, x_next) = self.soft_resolve(height);
            let v_next: Vec<f64> = x_next
                .iter()
                .zip(&self.x)
                .map(|(xf, xo)| (xf - xo) / dt)
                .collect();
            let x_next_var = tape.push_custom(
                &[x_var, v_var, p_var, pose_var],
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
            let w_var = tape.push_custom(
                &[x_next_var, pose_var, s_prev],
                Tensor::from_slice(wrench.as_slice(), &[6]),
                Box::new(ContactWrenchTrajVjp {
                    active,
                    force,
                    jlin,
                    n_dof: 3 * n,
                    nv,
                    pose: WrenchPose::Height,
                }),
            );
            // Advance the lagged-state handle now that the stale-FK contact nodes (pose, wrench)
            // have consumed `s_prev = s_{k-1}`: it becomes `s_k` for the next step's lag, and after
            // the loop holds `s_{N-1}` (the stale-FK `xpos.z` readout node — see the doc).
            s_prev = s_var;
            // `J_state` holds the CONTACT `wrench` fixed; the contact-axis damping `−c·vz` is a
            // velocity-dependent force `scratch_state_step` re-derives from the perturbed velocity,
            // so the loaded Jacobian picks up its `a = 1 − c·Δt/m` coupling (`c = 0` ⇒ unchanged).
            let j_state = self
                .analytic_state_jacobian(&wrench)
                .unwrap_or_else(|| self.loaded_state_jacobian(&wrench));
            let g_vel = rigid_xfrc_column(&self.model, &self.data, self.body);
            // Route the contact wrench + the forward damping `−c·qvel[2]` on the world-z linear
            // force, matching `step_core`'s `sf[5] = −fz − c·qvel[2]` (gated like `scratch_state_step`
            // so the `c = 0` path is structurally byte-identical).
            let mut applied = wrench;
            if self.rigid_damping != 0.0 {
                applied[5] += -self.rigid_damping * self.data.qvel[2];
            }
            self.data.xfrc_applied[self.body] = applied;
            self.data.step(&self.model).expect("rigid step");
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
                    g_act: None,
                }),
            );

            self.v = v_next;
            self.x = x_next;
            x_var = x_next_var;
            v_var = v_next_var;
            s_var = s_next_var;
        }
        (tape, p_var, s_var, s_prev)
    }

    /// Analytic `∂(final angular velocity ω_N about a body axis)/∂(material param)` for the
    /// **free-body** coupled impact rollout — the contact-MOMENT made differentiable. The
    /// orientation-dependent successor to [`Self::coupled_trajectory_material_gradient`]: that
    /// path's `z_N` target is moment-DECOUPLED (a free body's z-translation and the flat-plane
    /// contact never see its orientation, so the moment leaves `z_N` bit-identical — verified),
    /// so it cannot exercise the contact moment; the final spin `ω_N = qvel[dof + 3 + axis]` (the
    /// body angular-velocity component the off-COM strike drives) is the natural target that does.
    ///
    /// Routes the full contact wrench through the shared multi-DOF carry
    /// (`build_freebody_wrench_tape`); the objective is a plain `qvel` component select
    /// (the spin is Euclidean), so this exercises the wrench MOMENT carry (`τ → ω`) but not the
    /// position carry `G_pos` — [`Self::coupled_trajectory_orientation_gradient`] gates that.
    /// `axis ∈ {0,1,2}` (x/y/z body angular). Returns `(ω_N, ∂ω_N/∂param)` (`param_idx`: 0 = μ,
    /// 1 = λ; the block's λ=4μ tie ⇒ the design gradient is `grad(0) + 4·grad(1)`). FD-exact
    /// (machine floor, the ~1e-6 FD-`J_state` precision floor at extreme lengths) vs a re-rolled
    /// forward oracle; gated by the `freebody·angular-velocity[μ]` (plane) and
    /// `sphere-freebody·angular-velocity[μ]` (curved) rows of `tests/coupling_grad_harness.rs`
    /// (single length n = 16) plus the all-lengths sweep in
    /// `tests/freebody_angular_velocity_gradient.rs`.
    ///
    /// **Scope (v1).** As `build_freebody_wrench_tape` (free body, plane collider,
    /// `with_contact_moment` on, no moving EE), plus a single free joint and `rigid_damping = 0`
    /// (the carry supports damping, but the damped *angular* gradient is not yet FD-gated — only the
    /// z-translation row is). `&mut self`: advances the rollout, so rebuild per call.
    ///
    /// # Panics
    /// Panics if `param_idx > 1`, `axis >= 3`, `rigid_damping != 0`, the contacting body is not a
    /// single free joint, or any `build_freebody_wrench_tape` precondition fails; and if the rigid step diverges
    /// or the soft solver does not converge — surfaced loudly as in [`Self::step`].
    #[must_use]
    pub fn coupled_trajectory_angular_velocity_gradient(
        &mut self,
        n_steps: usize,
        param_idx: usize,
        axis: usize,
    ) -> (f64, f64) {
        assert!(
            axis < 3,
            "angular axis {axis} out of range (0 = x, 1 = y, 2 = z)"
        );
        // The carry supports contact-axis damping, but the ANGULAR target's damped gradient has no
        // FD oracle yet (only the z-translation row is gated under damping); keep this method to the
        // validated undamped scope until a damped angular gate lands.
        assert!(
            self.rigid_damping == 0.0,
            "angular-velocity gradient requires rigid_damping = 0 (damped angular target not yet gated)"
        );
        let (_, dadr) = self.free_joint_adrs();
        let nv = self.model.nv;
        let omega_idx = dadr + 3 + axis; // free-joint qvel layout: [v(3); ω(3)]
        let (mut tape, p_var, s_var, _) = self.build_freebody_wrench_tape(n_steps, param_idx);

        // Objective: the final spin ω_N = qvel[3 + axis] — a Euclidean `qvel` row of the state, so
        // the readout is a plain component select. It exercises the wrench MOMENT carry (τ → ω via
        // `G_vel` + the `J_state` qvel-block) but NOT the position carry `G_pos`; the orientation
        // readout ([`Self::coupled_trajectory_orientation_gradient`]) covers that.
        let omega = self.data.qvel[omega_idx];
        let obj_var = tape.push_custom(
            &[s_var],
            Tensor::from_slice(&[omega], &[1]),
            Box::new(StateComponentVjp {
                idx: nv + omega_idx,
                n_state: 2 * nv,
            }),
        );
        tape.backward(obj_var);
        let grad = tape.grad_tensor(p_var).as_slice()[0];
        (omega, grad)
    }

    /// Analytic `∂(final orientation component q_vec[axis])/∂(material param)` for the **free-body**
    /// coupled impact rollout — the orientation successor to
    /// [`Self::coupled_trajectory_angular_velocity_gradient`]. Where `ω_N` exercises only the wrench
    /// MOMENT carry (`τ → ω`), the body's final *orientation* depends on the position carry
    /// `G_pos = Δt·J_r·G_vel` (the SO(3) right-Jacobian integrating `qvel'` into the quaternion), so
    /// this target is the one that gates `G_pos`.
    ///
    /// The objective is a quaternion **vector component** `q_vec[axis]` (`qx`/`qy`/`qz`) of the free
    /// joint's final orientation — smooth in the rotation (no angle-wrap, unlike `‖log q‖`). Its
    /// body-frame tangent derivative is closed-form (`∂q_vec/∂δ = ½(w·I + [v]_×)`; the `axis` row
    /// seeds the state's angular `qpos` tangent rows, `QuatComponentVjp`) — no SO(3)-log in the
    /// adjoint. `axis ∈ {0,1,2}` (`qx`/`qy`/`qz`). Returns `(q_vec[axis], ∂/∂param)` (`param_idx`:
    /// 0 = μ, 1 = λ; the λ=4μ tie ⇒ the design gradient is `grad(0) + 4·grad(1)`). FD-exact
    /// (machine floor) vs a re-rolled forward oracle, on a rollout short enough to stay below a half
    /// turn (the smooth regime); gated by the `freebody·orientation[μ]` row of
    /// `tests/coupling_grad_harness.rs` (single length n = 16) plus the all-lengths sweep in
    /// `tests/freebody_orientation_gradient.rs`.
    ///
    /// **Scope (v1).** As [`Self::coupled_trajectory_angular_velocity_gradient`] (free body, plane
    /// collider, `with_contact_moment` on, `rigid_damping = 0`), plus the contacting body must carry
    /// a single free joint (the quaternion the readout reads). The ~1e-6 FD-`J_state` precision floor
    /// applies identically. `&mut self`: advances the rollout, so rebuild per call.
    ///
    /// # Panics
    /// Panics if `param_idx > 1`, `axis >= 3`, the contacting body is not a single free joint,
    /// `rigid_damping != 0`, `with_contact_moment` is off, a moving end-effector is set; and if the
    /// rigid step diverges or the soft solver does not converge — surfaced loudly as in [`Self::step`].
    #[must_use]
    pub fn coupled_trajectory_orientation_gradient(
        &mut self,
        n_steps: usize,
        param_idx: usize,
        axis: usize,
    ) -> (f64, f64) {
        assert!(
            axis < 3,
            "quaternion axis {axis} out of range (0 = x, 1 = y, 2 = z)"
        );
        // The carry supports contact-axis damping, but the orientation target's damped gradient has
        // no FD oracle yet (only the z-translation row is gated under damping); keep this method to
        // the validated undamped scope until a damped orientation gate lands.
        assert!(
            self.rigid_damping == 0.0,
            "orientation gradient requires rigid_damping = 0 (damped orientation target not yet gated)"
        );
        let (qadr, dadr) = self.free_joint_adrs(); // qpos [x,y,z, qw,qx,qy,qz], dof [v(3); ω(3)]
        let nv = self.model.nv;
        let (mut tape, p_var, s_var, _) = self.build_freebody_wrench_tape(n_steps, param_idx);

        // Objective: the final quaternion vector component q_vec[axis]. Its body-frame tangent
        // derivative is the `axis` row of ½(w·I + [v]_×) at the final orientation; seeding the
        // state's angular `qpos` tangent rows routes the adjoint through the position carry `G_pos`.
        let w = self.data.qpos[qadr + 3];
        let v = Vec3::new(
            self.data.qpos[qadr + 4],
            self.data.qpos[qadr + 5],
            self.data.qpos[qadr + 6],
        );
        // half = ½(w·I + [v]_×); row `axis` is ∂(q_vec[axis])/∂(body angular tangent).
        let half = 0.5 * (w * Matrix3::identity() + v.cross_matrix());
        let grad_tang = Vec3::new(half[(axis, 0)], half[(axis, 1)], half[(axis, 2)]);
        let q_comp = self.data.qpos[qadr + 4 + axis];
        let obj_var = tape.push_custom(
            &[s_var],
            Tensor::from_slice(&[q_comp], &[1]),
            Box::new(QuatComponentVjp {
                ang_dof: dadr + 3,
                grad_tang,
                n_state: 2 * nv,
            }),
        );
        tape.backward(obj_var);
        let grad = tape.grad_tensor(p_var).as_slice()[0];
        (q_comp, grad)
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

    /// Advance the coupled system by one lockstep step. Returns the contact
    /// force on the soft body and the rigid body's current height.
    ///
    /// # Panics
    /// Panics if the rigid engine's step diverges — for the validated forward
    /// scene this does not occur; a panic signals a mis-constructed or unstable
    /// coupling, surfaced loudly rather than silently corrupting state.
    pub fn step(&mut self) -> CoupledStep {
        self.step_core(false).0
    }

    /// Like [`Self::step`], but also returns the per-vertex contact-pressure field evaluated at the
    /// SAME posed contact `step` uses for its `peak_pressure` — so the field's finite max equals
    /// that peak and matches the just-solved deformation. This is the no-lag readout for a
    /// forward-replay heatmap: unlike calling [`Self::contact_vertex_pressures_at_height`] after
    /// `step` (whose `plane_height` has advanced a timestep), the field here is captured BEFORE the
    /// rigid body integrates. Length `n_vertices`, indexed by `VertexId`.
    ///
    /// # Panics
    /// Panics if the rigid engine's step diverges (as in [`Self::step`]).
    // expect_used: `step_core(true)` always populates the field, and the rigid-step divergence is
    // surfaced loudly (as in `step`); both are programmer errors, not recoverable for keystone-v1.
    #[allow(clippy::expect_used)]
    pub fn step_with_pressure_field(&mut self) -> (CoupledStep, Vec<f64>) {
        let (step, field) = self.step_core(true);
        (
            step,
            field.expect("step_core(true) always returns the field"),
        )
    }

    /// Shared body of [`Self::step`] / [`Self::step_with_pressure_field`]. Computes the per-vertex
    /// pressure field only when `want_field`, so the forward stepper that does not need it (e.g.
    /// [`Self::coupled_trajectory_peak_pressure`]) pays nothing.
    // expect_used: a rigid-step divergence is a programmer error surfaced loudly, not recoverable
    // for keystone-v1 (a Result-returning step is a deferred robustness upgrade — mirrors `step`).
    #[allow(clippy::expect_used)]
    fn step_core(&mut self, want_field: bool) -> (CoupledStep, Option<Vec<f64>>) {
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

        // (3) total contact force on the soft body + peak per-face pressure, from
        // ONE set of pair readouts (the force sum is byte-identical to
        // `contact_force_at_height`, which reduces the same readouts).
        let readouts = self.pair_readouts_at_height(height);
        let force_on_soft: Vec3 = readouts.iter().map(|r| r.force_on_soft).sum();
        let peak_pressure = peak_contact_pressure(&readouts);
        // Per-vertex field from the SAME readouts (so it matches the just-solved dent + the peak
        // above), captured here BEFORE the rigid body integrates below. Computed only on demand.
        let field = want_field.then(|| self.vertex_pressures_from(&readouts));

        // (4) route the reaction (−force_on_soft) + axis damping → rigid xfrc.
        // The linear part is a pure force at the body COM; the angular part (the
        // off-COM contact moment) is filled only when `with_contact_moment` is set
        // (default off ⇒ `sf[0..3] = 0`, byte-identical to the pre-moment routing).
        let v_axis = self.data.qvel[2]; // free-joint linear z velocity
        let mut sf = SpatialVector::zeros(); // [angular(3), linear(3)]
        sf[3] = -force_on_soft.x;
        sf[4] = -force_on_soft.y;
        sf[5] = -force_on_soft.z - self.rigid_damping * v_axis;
        // (4b) off-COM contact moment `Σ (rᵢ − c) × (−gᵢ)` about the body COM, from
        // step's OWN readouts (same definition as `contact_wrench`, no 2nd contact
        // eval). Opt-in: an off-centre strike spins the body, but it destabilises the
        // flat-tuned, off-centre-COM keystone scenes — see `with_contact_moment`.
        if self.contact_moment {
            let c = self.data.xipos[self.body];
            for r in &readouts {
                add_contact_moment(&mut sf, r.position, -r.force_on_soft, c);
            }
        }
        self.data.xfrc_applied[self.body] = sf;

        // (5) step the rigid body.
        self.data
            .step(&self.model)
            .expect("rigid step diverged in coupled solve");

        (
            CoupledStep {
                force_on_soft,
                rigid_z: self.data.xpos[self.body].z,
                peak_pressure,
            },
            field,
        )
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

        // Force + peak per-face pressure from ONE readout pass at the posed
        // contact (pre-rigid-step, sphere override still the current tip pose —
        // see `step`); the force sum is byte-identical to `contact_force_at_height`.
        let readouts = self.pair_readouts_at_height(height);
        let force_on_soft: Vec3 = readouts.iter().map(|r| r.force_on_soft).sum();
        let peak_pressure = peak_contact_pressure(&readouts);
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
            peak_pressure,
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

        // Force + peak pressure from ONE readout pass (force sum byte-identical
        // to `contact_force_at_height`; mirrors `step`).
        let readouts = self.pair_readouts_at_height(height);
        let force_on_soft: Vec3 = readouts.iter().map(|r| r.force_on_soft).sum();
        let peak_pressure = peak_contact_pressure(&readouts);
        CoupledStep {
            force_on_soft,
            rigid_z: self.data.xpos[self.body].z,
            peak_pressure,
        }
    }

    /// Roll the free-body coupled system forward `n_steps` (the [`Self::step`]
    /// path) and return the trajectory's peak contact **pressure** alongside its
    /// peak total **force** — the measured contrast at the heart of the
    /// de-escalation pressure story. A concentrated finite collider (a sphere)
    /// reads a HIGH peak pressure at LOW total force; a broad slab the reverse —
    /// the distinction total `force_on_soft` alone cannot make. Forward-only (no
    /// tape); advances `self`, so build a fresh coupling per call. Non-finite
    /// (all-degenerate) per-step pressures are ignored when tracking the peak
    /// (see [`peak_contact_pressure`]); the force peak still tracks them.
    ///
    /// # Panics
    /// Panics if the rigid step diverges or the soft solver fails (as in
    /// [`Self::step`]).
    #[must_use]
    pub fn coupled_trajectory_peak_pressure(&mut self, n_steps: usize) -> TrajectoryPeakPressure {
        let mut peak_pressure = 0.0_f64;
        let mut peak_total_force = 0.0_f64;
        let mut peak_step: Option<usize> = None;
        for k in 0..n_steps {
            let s = self.step();
            peak_total_force = peak_total_force.max(s.force_on_soft.norm());
            if s.peak_pressure.is_finite() && s.peak_pressure > peak_pressure {
                peak_pressure = s.peak_pressure;
                peak_step = Some(k);
            }
        }
        TrajectoryPeakPressure {
            peak_pressure,
            peak_total_force,
            peak_step,
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
    /// [`Self::coupled_trajectory_grip`]. FD-gated machine-exact by the
    /// `friction·tangential-material[μ]` row of the coupling gradient harness
    /// (`tests/coupling_grad_harness.rs`).
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
mod tests;
