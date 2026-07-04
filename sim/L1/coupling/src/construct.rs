//! Construction, configuration builders, and simple accessors for [`StaggeredCoupling`](crate::StaggeredCoupling).

use nalgebra::Point3;
use sim_core::{Data, Matrix3, Model};
use sim_soft::{HandBuiltTetMesh, MaterialField, Mesh, Sdf, SolverConfig, Vec3, VertexId};
use std::marker::PhantomData;

use crate::StaggeredCoupling;

use crate::contact::{Collider, PlaneContact, posed_sphere};

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
    /// Opt-in (default the infinite [`RigidPlane`](sim_soft::RigidPlane), byte-identical to the
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
    pub(super) fn plane_height(&self) -> f64 {
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
    pub(super) fn sphere_center(&self, radius: f64, height: f64) -> Vec3 {
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
    pub(super) fn collider_hessian(&self, height: f64, p: Vec3) -> Matrix3<f64> {
        match self.collider {
            Collider::Plane => Matrix3::zeros(),
            Collider::Sphere { radius } => {
                posed_sphere(radius, self.sphere_center(radius, height)).hessian(Point3::from(p))
            }
        }
    }
}
