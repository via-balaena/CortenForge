//! Two-rigid **bonded sandwich** — a soft disc tied between two rigid bodies by
//! pose-driven stiff Dirichlet bonds (rung 6b, forward only).
//!
//! The FSU keystone gap: [`StaggeredCoupling`](crate::StaggeredCoupling) bonds ONE
//! soft body to ONE rigid body through a *unilateral penalty plane* — it can only
//! push. An intervertebral disc is **bonded** between two vertebrae and must carry
//! **tension** in bending (the posterior anulus stretches while the anterior
//! compresses). This type supplies that two-way tie.
//!
//! ## The bond = stiff Dirichlet driven by the rigid pose
//!
//! Each rigid endplate is (relative to the soft disc) effectively rigid, so the
//! disc nodes glued to it must follow its pose *exactly*. We realise the bond as a
//! full Dirichlet constraint whose target tracks the body: for a bonded node at
//! rest offset `r_b` in the body frame, its target each step is
//! `x_body + R_body · r_b`. Writing those targets into the solver's `x_prev` pins
//! the nodes there (the soft solver holds pinned DOFs at their `x_prev` value), and
//! the reaction the disc exerts back on the endplate is read from the solver's own
//! assembly as `−f_int` at the bonded DOFs
//! ([`nodal_reaction_forces`](sim_soft::CpuNewtonSolver::nodal_reaction_forces)) —
//! oracle == SUT, no re-derivation. Because the soft internal force is
//! self-equilibrated (`Σ_all f_int ≡ 0`), the reactions on the two faces are
//! equal-and-opposite up to the free-node residual (Newton's third law across the
//! coupled interface) — measured to ~1e-12 in `tests/bonded_sandwich_fsu.rs`.
//!
//! ## Scope (6b / 6c)
//!
//! Forward only. The bond is **mesh-agnostic**: [`BondedSandwich`] is generic over
//! the tet-mesh type ([`Mesh`]). [`BondedSandwich::new`] bonds a **primitive** block
//! disc ([`HandBuiltTetMesh::uniform_block`], rung 6b); [`BondedSandwich::from_tet_mesh`]
//! bonds **any** tet mesh with caller-supplied endplate-vertex sets — the real
//! intervertebral-disc geometry ([`SdfMeshedTetMesh`](sim_soft::SdfMeshedTetMesh)
//! from the anatomical mesh, rung 6c). Differentiability across the bond is rung 6d
//! (the constrained DOFs carry zero cotangent today, and the existing pose→soft
//! adjoint is wired to contact, not Dirichlet). The soft solve is **quasi-static** (a
//! large `dt` so the inertial term is negligible): the disc equilibrates to the
//! current endplate poses each step while the rigid bodies integrate at their own
//! model timestep — the standard staggered scheme, with the penalty contact replaced
//! by the Dirichlet bond.

use nalgebra::UnitQuaternion;
use sim_core::{Data, MjJointType, Model, SpatialVector};
use sim_ml_chassis::Tensor;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, HandBuiltTetMesh, MaterialField, Mesh, NullContact,
    Solver, SolverConfig, Tet4, Vec3, VertexId,
};

use crate::vjp::add_contact_moment;

/// Newton-iteration budget for the quasi-static disc solve. Generous headroom over the
/// skeleton's block-tuned 10 so a real many-thousand-tet disc converges to the same
/// `tol` under stiff near-incompressible modes; easy problems early-exit unaffected.
const MAX_NEWTON_ITER: usize = 50;

/// One rigid endplate's bond: which rigid body, which disc nodes glue to it, and
/// each node's rest offset expressed in the body frame (so the target tracks the
/// body's full pose, translation *and* rotation).
struct Bond {
    body: usize,
    verts: Vec<VertexId>,
    offsets_body: Vec<Vec3>,
}

impl Bond {
    /// Build a bond from the bonded vertex set, snapshotting each node's rest
    /// offset in the body frame at the current (reference) pose.
    fn new(body: usize, verts: Vec<VertexId>, rest: &[Vec3], data: &Data) -> Self {
        let x0 = data.xpos[body];
        let q0_inv = data.xquat[body].inverse();
        let offsets_body = verts
            .iter()
            .map(|&v| q0_inv * (rest[v as usize] - x0))
            .collect();
        Self {
            body,
            verts,
            offsets_body,
        }
    }

    /// World-frame target of each bonded node at the body's current pose:
    /// `x_body + R_body · r_b`.
    fn targets(&self, data: &Data) -> Vec<Vec3> {
        let x = data.xpos[self.body];
        let q = data.xquat[self.body];
        self.offsets_body.iter().map(|&r| x + q * r).collect()
    }
}

/// The per-step wrench each endplate feels from the disc (the reaction routed onto
/// its rigid body), plus the readout the FSU gate measures.
#[derive(Clone, Debug)]
pub struct BondStep {
    /// Force the disc exerts on the lower endplate (`Σ −f_int` over its bonded face).
    pub force_lower: Vec3,
    /// Force the disc exerts on the upper endplate.
    pub force_upper: Vec3,
    /// Moment of the lower reaction about the lower body's COM (`Σ (rᵢ − c) × fᵢ`).
    pub moment_lower: Vec3,
    /// Moment of the upper reaction about the upper body's COM.
    pub moment_upper: Vec3,
}

impl BondStep {
    /// Unpack the two `[moment; force]` endplate wrenches into the public
    /// force/moment fields.
    fn from_wrenches(lower: SpatialVector, upper: SpatialVector) -> Self {
        Self {
            force_lower: wrench_force(&lower),
            force_upper: wrench_force(&upper),
            moment_lower: wrench_moment(&lower),
            moment_upper: wrench_moment(&upper),
        }
    }
}

/// A soft disc bonded between two rigid bodies (rung 6b / 6c).
///
/// Generic over the tet-mesh type `Msh` (defaults to [`HandBuiltTetMesh`], the rung-6b
/// primitive block). Construct a primitive block disc with [`BondedSandwich::new`] or
/// bond an arbitrary tet mesh — e.g. the real intervertebral disc
/// ([`SdfMeshedTetMesh`](sim_soft::SdfMeshedTetMesh)) — with
/// [`BondedSandwich::from_tet_mesh`]; then drive the coupled system with
/// [`BondedSandwich::step`] / [`BondedSandwich::probe`]. The lower bonded face glues to
/// `lower_body` and the upper to `upper_body`; both bodies must already be posed so
/// those faces coincide with their endplates (see the FSU gates for the canonical
/// scenes). Forward only — see the module docs for scope.
pub struct BondedSandwich<Msh = HandBuiltTetMesh>
where
    Msh: Mesh,
{
    model: Model,
    data: Data,

    /// The disc solver, built ONCE: the bonded (pinned) Dirichlet set and the mesh
    /// topology are constant, and [`Solver::replay_step`] / `nodal_reaction_forces`
    /// are pure functions of the per-step Dirichlet targets (`&self`), so only
    /// `x_prev` changes step to step — no per-step rebuild.
    solver: DiscSolver<Msh>,
    /// The large quasi-static timestep (inertial term `M/dt²` negligible).
    static_dt: f64,
    n_vertices: usize,
    /// Current soft configuration (vertex-major xyz) — warm-starts the next
    /// quasi-static solve; free interior nodes carry forward, bonded nodes are
    /// overwritten with their posed targets each step.
    x: Vec<f64>,

    lower: Bond,
    upper: Bond,

    /// The per-DOF reaction (`−f_int`) and the `x_prev` it was read at (bonded nodes
    /// at their posed targets), from the last [`Self::step`] OR [`Self::probe`] —
    /// both flow through `resolve`. Exposed for the gate's conservation +
    /// two-way-tension measurements.
    last_reaction: Vec<f64>,
    last_targets: Vec<f64>,
}

type DiscSolver<Msh> = CpuNewtonSolver<Tet4, Msh, NullContact>;

impl BondedSandwich<HandBuiltTetMesh> {
    /// Bond a primitive Neo-Hookean disc of `n_per_edge` cells and `edge` length
    /// between `lower_body` and `upper_body` in `model`/`data` (rung 6b).
    ///
    /// `mu` sets the disc's Neo-Hookean stiffness (`λ = 4μ`); `static_dt` is the
    /// large timestep that makes the soft solve quasi-static (the inertial term
    /// `M/dt²` negligible). `data` must already be `forward`-ed so body poses are
    /// current; the disc's bottom face (`z ≈ 0`) bonds to `lower_body` and its top
    /// face (`z ≈ edge`) to `upper_body`. A convenience wrapper over
    /// [`Self::from_tet_mesh`] that builds the block and picks its two planar faces.
    ///
    /// # Panics
    /// Panics if the disc has no `z ≈ 0` or no `z ≈ edge` face to bond (a malformed
    /// block).
    #[must_use]
    // 8 independent physical knobs (two bodies + disc geometry + material + static dt); a
    // config-struct bundle is a deferred ergonomics refactor (mirrors `StaggeredCoupling::new`).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        model: Model,
        data: Data,
        lower_body: usize,
        upper_body: usize,
        n_per_edge: usize,
        edge: f64,
        mu: f64,
        static_dt: f64,
    ) -> Self {
        let field = MaterialField::uniform(mu, 4.0 * mu);
        let mesh = HandBuiltTetMesh::uniform_block(n_per_edge, edge, &field);

        // The two planar block faces; `from_tet_mesh` is the single validator (asserts
        // them non-empty — a valid `n_per_edge ≥ 1` block always has both).
        let lower_verts = sim_soft::pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
        let upper_verts =
            sim_soft::pick_vertices_by_predicate(&mesh, |p| (p.z - edge).abs() < 1e-9);

        Self::from_tet_mesh(
            model,
            data,
            lower_body,
            upper_body,
            mesh,
            lower_verts,
            upper_verts,
            static_dt,
        )
    }
}

impl<Msh: Mesh> BondedSandwich<Msh> {
    /// Bond an **arbitrary** tet mesh between `lower_body` and `upper_body`, with the
    /// two endplate faces supplied as vertex-id sets (rung 6c — the real disc geometry).
    ///
    /// `lower_verts` glue to `lower_body`, `upper_verts` to `upper_body`; the caller
    /// derives them from the disc's own geometry (e.g. a band of the superior /
    /// inferior extent along the disc's short axis) — this constructor takes no axis
    /// convention. The mesh already carries its per-tet material (baked at mesh
    /// construction); `static_dt` is the large timestep that makes the soft solve
    /// quasi-static. `data` must already be `forward`-ed so body poses are current, and
    /// the two bodies posed so the endplate faces coincide with their surfaces (the
    /// rest state is deformation-free — each bonded node's offset is snapshot at the
    /// current pose).
    ///
    /// # Panics
    /// Panics if either bonded-vertex set is empty, contains a duplicate id (which would
    /// double-count that node's reaction), shares a vertex with the other set (a node
    /// cannot bond to both endplates), or contains an id out of range for `mesh`.
    #[must_use]
    // 8 independent physical inputs (two bodies + mesh + two face sets + static dt); a
    // config-struct bundle is a deferred ergonomics refactor (mirrors `StaggeredCoupling::new`).
    #[allow(clippy::too_many_arguments)]
    pub fn from_tet_mesh(
        model: Model,
        data: Data,
        lower_body: usize,
        upper_body: usize,
        mesh: Msh,
        lower_verts: Vec<VertexId>,
        upper_verts: Vec<VertexId>,
        static_dt: f64,
    ) -> Self {
        let n_vertices = mesh.n_vertices();
        assert!(!lower_verts.is_empty(), "lower bonded face is empty");
        assert!(!upper_verts.is_empty(), "upper bonded face is empty");
        assert!(
            lower_verts
                .iter()
                .chain(&upper_verts)
                .all(|&v| (v as usize) < n_vertices),
            "a bonded vertex id is out of range for the {n_vertices}-vertex disc mesh"
        );
        // Each face's ids must be unique (a repeat would double-count that node's
        // reaction in the endplate wrench), and the two faces disjoint (a node cannot
        // bond to both bodies). Deduped sets give both checks in one place.
        let lower_set: std::collections::HashSet<VertexId> = lower_verts.iter().copied().collect();
        let upper_set: std::collections::HashSet<VertexId> = upper_verts.iter().copied().collect();
        assert!(
            lower_set.len() == lower_verts.len(),
            "the lower bonded face has a duplicate vertex id"
        );
        assert!(
            upper_set.len() == upper_verts.len(),
            "the upper bonded face has a duplicate vertex id"
        );
        assert!(
            lower_set.is_disjoint(&upper_set),
            "the two endplate faces share a vertex (a node cannot bond to both bodies)"
        );

        let rest: Vec<Vec3> = mesh.positions().to_vec();
        let lower = Bond::new(lower_body, lower_verts, &rest, &data);
        let upper = Bond::new(upper_body, upper_verts, &rest, &data);

        let mut x = vec![0.0_f64; 3 * n_vertices];
        for (chunk, p) in x.chunks_exact_mut(3).zip(rest.iter()) {
            chunk[0] = p.x;
            chunk[1] = p.y;
            chunk[2] = p.z;
        }

        // Build the disc solver ONCE: both bonded faces are the constant pinned
        // Dirichlet set, so one symbolic factor + mesh serves every step (only the
        // Dirichlet targets in `x_prev` change, and `replay_step` takes those as args).
        let mut pinned = lower.verts.clone();
        pinned.extend_from_slice(&upper.verts);
        let mut cfg = SolverConfig::skeleton();
        cfg.dt = static_dt;
        cfg.max_newton_iter = MAX_NEWTON_ITER; // real-mesh headroom; see the const's doc.
        let solver: DiscSolver<Msh> = CpuNewtonSolver::new(
            Tet4,
            mesh,
            NullContact,
            cfg,
            BoundaryConditions::new(pinned, Vec::new()),
        );

        Self {
            model,
            data,
            solver,
            static_dt,
            n_vertices,
            x,
            lower,
            upper,
            last_reaction: vec![0.0; 3 * n_vertices],
            last_targets: vec![0.0; 3 * n_vertices],
        }
    }

    /// Quasi-static bond core, shared by [`Self::step`] and [`Self::probe`]: pose both
    /// bonded faces from the CURRENT rigid poses, solve the disc to equilibrium, read
    /// the reaction, and reduce it to the `[moment; force]` wrench each endplate feels
    /// about its body COM. Advances the disc warm-start + reaction readout; does NOT
    /// touch the rigid state. Returns `(lower_wrench, upper_wrench)`.
    fn resolve(&mut self) -> (SpatialVector, SpatialVector) {
        let n = self.n_vertices;

        // (1) pose the bonds: carry the free nodes, overwrite bonded nodes with
        //     their posed targets. `x_prev` doubles as the Dirichlet target buffer.
        let mut x_prev = self.x.clone();
        let lower_targets = self.lower.targets(&self.data);
        let upper_targets = self.upper.targets(&self.data);
        write_targets(&mut x_prev, &self.lower.verts, &lower_targets);
        write_targets(&mut x_prev, &self.upper.verts, &upper_targets);

        // (2) one quasi-static soft solve against the posed bonds (reused solver).
        let solved = self.solver.replay_step(
            &Tensor::from_slice(&x_prev, &[3 * n]),
            &Tensor::zeros(&[3 * n]),
            &Tensor::zeros(&[0]),
            self.static_dt,
        );

        // (3) reaction the disc exerts on its Dirichlet anchors (= −f_int).
        let react = self
            .solver
            .nodal_reaction_forces(&solved.x_final, &x_prev, self.static_dt);
        self.x = solved.x_final;

        // (4) reduce each face's reaction to a wrench about its body COM.
        let lower_wrench = face_wrench(
            &react,
            &self.lower.verts,
            &lower_targets,
            self.data.xipos[self.lower.body],
        );
        let upper_wrench = face_wrench(
            &react,
            &self.upper.verts,
            &upper_targets,
            self.data.xipos[self.upper.body],
        );

        self.last_reaction = react;
        self.last_targets = x_prev;
        (lower_wrench, upper_wrench)
    }

    /// Advance the coupled system one lockstep step: pose both bonded faces from the
    /// current rigid poses, solve the disc to quasi-static equilibrium, read the two
    /// reaction wrenches, route them onto the two bodies' `xfrc_applied`, and step
    /// the rigid engine. Returns the per-endplate wrench ([`BondStep`]).
    ///
    /// # Panics
    /// Panics — surfaced loudly rather than silently corrupting state (mirrors
    /// [`StaggeredCoupling::step`](crate::StaggeredCoupling::step)) — if either:
    /// - the rigid engine's step diverges; or
    /// - the disc solve fails to converge (`replay_step` is the panic-on-fail-close
    ///   primal — the graceful-`Err` `try_` path is a deferred rung-7 upgrade, see
    ///   [`Self::probe`]). For the validated forward scene neither occurs.
    // expect_used: a rigid-step / FK divergence is a programmer error surfaced loudly, not
    // recoverable here (mirrors `StaggeredCoupling::step`).
    #[allow(clippy::expect_used)]
    pub fn step(&mut self) -> BondStep {
        let (lower_wrench, upper_wrench) = self.resolve();
        self.data.xfrc_applied[self.lower.body] = lower_wrench;
        self.data.xfrc_applied[self.upper.body] = upper_wrench;
        // Step the rigid engine, then refresh FK so poses/COMs are current for the
        // next step and any caller readout.
        self.data
            .step(&self.model)
            .expect("rigid step diverged in bonded sandwich");
        self.data
            .forward(&self.model)
            .expect("fresh FK (post-step)");
        BondStep::from_wrenches(lower_wrench, upper_wrench)
    }

    /// Quasi-static **readout** at the current rigid poses: solve the disc and return
    /// the reaction wrench on each endplate WITHOUT integrating the rigid bodies — the
    /// disc's response at a prescribed segmental configuration (e.g. an imposed
    /// compression or flexion set via [`Self::set_body_pose`]). This is the
    /// FSU-characterisation readout (the segmental-stiffness probe rung 7 consumes).
    /// Advances the disc warm-start / reaction state; leaves the rigid state untouched.
    ///
    /// # Panics
    /// Panics if the disc solve fails to converge — `replay_step` is the
    /// panic-on-fail-close primal solver, so a segmental displacement large enough to
    /// exceed the Newton iteration cap / stall the line search aborts loudly rather
    /// than returning a graceful `Err`. Surfacing infeasible characterisation configs
    /// as `Err` (via the solver's `try_replay_step`, mirroring the co-design
    /// `InfeasibleDesign` path) is a deferred rung-7 robustness upgrade; the validated
    /// 6b configs converge.
    pub fn probe(&mut self) -> BondStep {
        let (lower_wrench, upper_wrench) = self.resolve();
        BondStep::from_wrenches(lower_wrench, upper_wrench)
    }

    /// Script a rigid body's world pose (position + orientation) and refresh forward
    /// kinematics — for imposing a controlled segmental configuration before a
    /// [`Self::probe`]. The bond reference (rest offsets) is fixed at construction and
    /// is NOT re-snapshot, so a displaced pose genuinely deforms the disc.
    ///
    /// # Panics
    /// Panics if `body` does not have a single free joint (the FSU endplates do), or
    /// if the forward-kinematics refresh diverges.
    // expect_used: a bad body index / FK divergence is a scene-wiring bug surfaced loudly.
    #[allow(clippy::expect_used)]
    pub fn set_body_pose(&mut self, body: usize, pos: Vec3, quat: UnitQuaternion<f64>) {
        assert!(
            self.model.body_jnt_num[body] == 1
                && matches!(
                    self.model.jnt_type[self.model.body_jnt_adr[body]],
                    MjJointType::Free
                ),
            "set_body_pose requires body {body} to have a single free joint"
        );
        let adr = self.model.jnt_qpos_adr[self.model.body_jnt_adr[body]];
        self.data.qpos[adr] = pos.x;
        self.data.qpos[adr + 1] = pos.y;
        self.data.qpos[adr + 2] = pos.z;
        self.data.qpos[adr + 3] = quat.w;
        self.data.qpos[adr + 4] = quat.i;
        self.data.qpos[adr + 5] = quat.j;
        self.data.qpos[adr + 6] = quat.k;
        self.data
            .forward(&self.model)
            .expect("fresh FK after set_body_pose");
    }

    /// The rigid state (body poses, velocities) after the last step.
    #[must_use]
    pub const fn data(&self) -> &Data {
        &self.data
    }

    /// The vertex IDs bonded to the lower / upper body (the bottom / top disc face).
    #[must_use]
    pub fn lower_face(&self) -> &[VertexId] {
        &self.lower.verts
    }
    /// See [`Self::lower_face`].
    #[must_use]
    pub fn upper_face(&self) -> &[VertexId] {
        &self.upper.verts
    }

    /// Reverse-mode gradient of the bonded reaction wrenches w.r.t. the two body poses
    /// (rung 6d — the differentiable bond). Resolves the disc at the current poses (like
    /// [`Self::probe`]) and, given a cotangent on each endplate's wrench, returns the
    /// gradient of that scalar loss w.r.t. each body's **world-frame twist**.
    ///
    /// Inputs `cot_lower` / `cot_upper` are `∂L/∂wrench` per endplate in the `[ang; lin]`
    /// [`SpatialVector`] layout (`ang = ∂L/∂moment`, `lin = ∂L/∂force`). The returned
    /// `[lower, upper]` gradients use the SAME layout as a **co-twist**: `ang = ∂L/∂ω`,
    /// `lin = ∂L/∂v`, with `(ω, v)` the body's world-frame angular / linear velocity about
    /// its frame origin (`xpos`). Also returns the forward [`BondStep`] (the wrench values).
    ///
    /// **The reverse pass** (one adjoint solve, both faces coupled). Per bonded vertex `i`
    /// with reaction `Rᵢ`, target `rᵢ`, arm `dᵢ = rᵢ − c`:
    /// 1. reaction cotangent `∂L/∂Rᵢ = cot_f + cot_τ × dᵢ` (force + moment readout adjoint,
    ///    the crate's `add_contact_moment` moment convention);
    /// 2. ONE Dirichlet reaction VJP
    ///    ([`CpuNewtonSolver::equilibrium_dirichlet_reaction_vjp`](sim_soft::CpuNewtonSolver::equilibrium_dirichlet_reaction_vjp))
    ///    maps `∂L/∂R` (both faces) → `∂L/∂(target)` through the symmetric reaction
    ///    Jacobian — this is the coupling term (moving one endplate changes the other's
    ///    reaction);
    /// 3. the geometric moment-arm term `∂L/∂ω += offsetᵢ × (Rᵢ × cot_τ)` (the moment arm
    ///    rotates with the body), added to the pose map of step 2's `∂L/∂(target)`
    ///    (`∂target/∂v = I`, `∂target/∂ω = −[offsetᵢ]×`, `offsetᵢ = rᵢ − xpos`).
    ///
    /// **Scope: centered bodies** (`xipos ≈ xpos` — the geom COM at the frame origin, as in
    /// every FSU scene). Then the moment reference `c = xipos` equals the rotation centre
    /// `xpos`, so `dᵢ = offsetᵢ` and the translation components of the moment-arm and COM
    /// paths cancel exactly. An off-centre body (`xipos ≠ xpos`) needs the extra `∂c/∂ω`
    /// term and is a follow-on; asserted here so a mis-posed scene fails loudly.
    ///
    /// # Panics
    /// Panics if either body is not centered (`‖xipos − xpos‖` above a small tolerance) or
    /// if the disc solve diverges.
    #[must_use]
    pub fn probe_with_pose_gradient(
        &mut self,
        cot_lower: SpatialVector,
        cot_upper: SpatialVector,
    ) -> (BondStep, [SpatialVector; 2]) {
        let (lower_w, upper_w) = self.resolve();
        let step = BondStep::from_wrenches(lower_w, upper_w);

        // (1) reaction cotangent ∂L/∂R (full-DOF, both bonded faces; zero on free DOFs).
        let mut dl_dr = vec![0.0_f64; 3 * self.n_vertices];
        face_reaction_cotangent(
            &mut dl_dr,
            &self.lower.verts,
            &self.last_targets,
            self.data.xipos[self.lower.body],
            cot_lower,
        );
        face_reaction_cotangent(
            &mut dl_dr,
            &self.upper.verts,
            &self.last_targets,
            self.data.xipos[self.upper.body],
            cot_upper,
        );

        // (2) one Dirichlet reaction VJP: ∂L/∂R → ∂L/∂(target), coupling both faces.
        let dl_dtarget =
            self.solver
                .equilibrium_dirichlet_reaction_vjp(&self.x, self.static_dt, &dl_dr);

        // (3) per body: geometric moment-arm term + pose map → world-frame co-twist.
        let grad_lower = self.body_pose_cotwist(&self.lower, &dl_dtarget, cot_lower);
        let grad_upper = self.body_pose_cotwist(&self.upper, &dl_dtarget, cot_upper);
        (step, [grad_lower, grad_upper])
    }

    /// Map a bonded face's `∂L/∂(target)` (reaction path) plus its geometric moment-arm
    /// term to the body's world-frame co-twist `[ang = ∂L/∂ω; lin = ∂L/∂v]`. See
    /// [`Self::probe_with_pose_gradient`] for the math; centered-body scope asserted here.
    fn body_pose_cotwist(
        &self,
        bond: &Bond,
        dl_dtarget: &[f64],
        cot: SpatialVector,
    ) -> SpatialVector {
        let pivot = self.data.xpos[bond.body]; // rotation centre (free-joint origin)
        // Centered-body guard: the moment reference (xipos) must coincide with the rotation
        // centre (xpos), else the off-centre ∂c/∂ω term (unmodeled) would bias the gradient.
        assert!(
            (self.data.xipos[bond.body] - pivot).norm() < 1e-7,
            "probe_with_pose_gradient requires a centered body (xipos ≈ xpos); body {} is off-centre",
            bond.body,
        );
        let cot_t = wrench_moment(&cot);
        let (mut d_ang, mut d_lin) = (Vec3::zeros(), Vec3::zeros());
        for &v in &bond.verts {
            let i = v as usize;
            let offset = vec3_at(&self.last_targets, i) - pivot;
            let dtr = vec3_at(dl_dtarget, i); // reaction-path ∂L/∂target
            let r_i = vec3_at(&self.last_reaction, i); // reaction force at this vertex
            // Reaction path: ∂target/∂v = I, ∂target/∂ω = −[offset]× ⇒ +offset × dtr.
            d_lin += dtr;
            d_ang += offset.cross(&dtr);
            // Geometric moment-arm: ∂L/∂ω += offset × (Rᵢ × cot_τ) (no linear term when centered).
            d_ang += offset.cross(&r_i.cross(&cot_t));
        }
        let mut g = SpatialVector::zeros();
        g[0] = d_ang.x;
        g[1] = d_ang.y;
        g[2] = d_ang.z;
        g[3] = d_lin.x;
        g[4] = d_lin.y;
        g[5] = d_lin.z;
        g
    }

    /// Per-DOF reaction (`−f_int`, vertex-major xyz) read at the last [`Self::step`]
    /// or [`Self::probe`] — the force the disc exerts on its Dirichlet anchors. Length
    /// `3 · n_vertices`.
    #[must_use]
    pub fn last_reaction(&self) -> &[f64] {
        &self.last_reaction
    }

    /// The world-frame target positions (vertex-major xyz) the bonded nodes were held
    /// at during the last [`Self::step`] or [`Self::probe`] — the deformed moment arms
    /// for a conservation check. Length `3 · n_vertices`.
    #[must_use]
    pub fn last_targets(&self) -> &[f64] {
        &self.last_targets
    }
}

/// Write world-frame `targets` into `x_prev` at the DOFs of `verts`.
fn write_targets(x_prev: &mut [f64], verts: &[VertexId], targets: &[Vec3]) {
    for (&v, t) in verts.iter().zip(targets) {
        let i = v as usize;
        x_prev[3 * i] = t.x;
        x_prev[3 * i + 1] = t.y;
        x_prev[3 * i + 2] = t.z;
    }
}

/// Reduce a bonded face's per-node reaction to the `[moment; force]` wrench about the
/// body COM `c` (`SpatialVector` layout `[ang(3), lin(3)]`), using each node's world
/// target as its moment arm. The `(r − c) × f` moment goes through
/// [`add_contact_moment`] — the crate's single source of that convention (shared with
/// the [`StaggeredCoupling`](crate::StaggeredCoupling) reaction-wrench path), so the
/// sign / reference-point / index layout can only ever be defined once.
fn face_wrench(react: &[f64], verts: &[VertexId], targets: &[Vec3], c: Vec3) -> SpatialVector {
    let mut w = SpatialVector::zeros();
    for (&v, &r) in verts.iter().zip(targets) {
        let i = v as usize;
        let f = Vec3::new(react[3 * i], react[3 * i + 1], react[3 * i + 2]);
        w[3] += f.x;
        w[4] += f.y;
        w[5] += f.z;
        add_contact_moment(&mut w, r, f, c);
    }
    w
}

/// Read a world-frame vertex vector from a vertex-major xyz slice.
fn vec3_at(s: &[f64], i: usize) -> Vec3 {
    Vec3::new(s[3 * i], s[3 * i + 1], s[3 * i + 2])
}

/// Scatter a bonded face's reaction cotangent `∂L/∂Rᵢ = cot_f + cot_τ × armᵢ` into `dl_dr`
/// (`armᵢ = targetᵢ − c`, `c` the body's moment reference). The reverse of the wrench
/// readout (force `Σ Rᵢ` + moment `Σ armᵢ × Rᵢ`, [`add_contact_moment`] convention). The two
/// faces are disjoint, so plain assignment per vertex is unambiguous.
fn face_reaction_cotangent(
    dl_dr: &mut [f64],
    verts: &[VertexId],
    targets: &[f64],
    c: Vec3,
    cot: SpatialVector,
) {
    let cot_t = wrench_moment(&cot); // ∂L/∂moment
    let cot_f = wrench_force(&cot); // ∂L/∂force
    for &v in verts {
        let i = v as usize;
        let arm = vec3_at(targets, i) - c;
        let d = cot_f + cot_t.cross(&arm);
        dl_dr[3 * i] = d.x;
        dl_dr[3 * i + 1] = d.y;
        dl_dr[3 * i + 2] = d.z;
    }
}

/// Linear (force) part of a `[ang(3), lin(3)]` wrench.
fn wrench_force(w: &SpatialVector) -> Vec3 {
    Vec3::new(w[3], w[4], w[5])
}

/// Angular (moment) part of a `[ang(3), lin(3)]` wrench.
fn wrench_moment(w: &SpatialVector) -> Vec3 {
    Vec3::new(w[0], w[1], w[2])
}
