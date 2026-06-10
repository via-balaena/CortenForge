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
//! is added incrementally: the *explicit* (fixed-soft-position) factors of the
//! coupled-step Jacobian live here now — the analytic contact-force-vs-pose
//! derivative ([`StaggeredCoupling::contact_force_height_jacobian`]) and the
//! rigid force response ([`StaggeredCoupling::rigid_step_probe`]). The *implicit*
//! soft re-equilibration term (`∂x*/∂pose`, needs a soft-pose VJP) and the full
//! soft-tape `VjpOp` crossing (one `tape.backward` across both engines) are the
//! next research leaves.

use sim_core::{Data, Model, SpatialVector};
use sim_ml_chassis::Tensor;
use sim_soft::{
    BoundaryConditions, ContactPair, CpuNewtonSolver, HandBuiltTetMesh, MaterialField, Mesh,
    PenaltyRigidContact, PenaltyRigidContactSolver, RigidPlane, Solver, SolverConfig, Tet4, Vec3,
    VertexId,
};

/// Result of one coupled step.
#[derive(Clone, Copy, Debug)]
pub struct CoupledStep {
    /// Total contact force the soft body exerts (the reaction on the rigid body
    /// is its negation). In newtons, world frame.
    pub force_on_soft: Vec3,
    /// Current height of the contacting rigid body's reference point.
    pub rigid_z: f64,
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
pub struct StaggeredCoupling {
    model: Model,
    data: Data,
    body: usize,
    contact_clearance: f64,

    // soft block (rebuilt per step; topology + pinned set are fixed)
    field: MaterialField,
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
}

impl StaggeredCoupling {
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
        let field = MaterialField::uniform(mu, 4.0 * mu);
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

    fn build_contact(&self, height: f64) -> PenaltyRigidContact {
        // Downward ceiling at `height`: normal −z, offset −height ⇒ a soft
        // vertex below the plane has positive signed distance `height − z`.
        let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), -height);
        PenaltyRigidContact::with_params(vec![plane], self.kappa, self.d_hat)
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
            .per_pair_readout(&self.fresh_mesh(), &positions);
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

    /// Analytic `∂(total force_on_soft)/∂(plane height)` at the current soft
    /// configuration, holding the soft positions fixed.
    ///
    /// For the downward penalty plane, each active pair has
    /// `force_on_soft = κ(d̂ − sd)·n̂` with `sd = height − z` and unit normal
    /// `n̂ = −ẑ`, so `∂force/∂height = −κ·n̂ = +κ·ẑ` per pair; over `N_active`
    /// pairs the total is `+κ·N_active·ẑ`.
    /// This is the explicit (fixed-position) partial — one factor of the coupled
    /// step's Jacobian, not the total settled-system derivative. Valid in the
    /// contact-engaged regime where the active set is stable across the
    /// perturbation; at the active-set boundary the true derivative is
    /// non-smooth (penalty cap; IPC is the deferred cure). FD-checked against
    /// [`Self::contact_force_at_height`].
    // `n_active` ≤ the soft vertex count (~125); the usize→f64 cast is exact here.
    #[allow(clippy::cast_precision_loss)]
    #[must_use]
    pub fn contact_force_height_jacobian(&self, height: f64) -> Vec3 {
        let n_active = self.contact_readout(height).1;
        // ∂force/∂height = −κ·n per active pair; plane normal n = −ẑ ⇒ +κ·ẑ.
        Vec3::new(0.0, 0.0, self.kappa * (n_active as f64))
    }

    /// One-off rigid step from the *current* rigid state with an externally
    /// supplied vertical force `applied_fz` (newtons, world `+z`), returning the
    /// rigid body's `(next height, next vertical velocity)`. Does NOT advance
    /// `self` — it reconstructs a scratch `Data` at the current `(qpos, qvel)`,
    /// so it is a pure probe of the rigid step's response to an applied force
    /// (the rigid factor `∂s'/∂xfrc` of the coupled-step Jacobian). For the
    /// free-body platen under semi-implicit Euler this response is analytically
    /// `∂vz'/∂fz = dt/m`, `∂z'/∂fz = dt²/m`.
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
    fn soft_resolve(&self, height: f64) -> (PenaltyRigidContactSolver<HandBuiltTetMesh>, Vec<f64>) {
        let n = self.n_vertices;
        let bc = BoundaryConditions::new(self.pinned.clone(), Vec::new());
        let solver: PenaltyRigidContactSolver<HandBuiltTetMesh> = CpuNewtonSolver::new(
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
            .per_pair_readout(&self.fresh_mesh(), &positions)
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
        let readout = self
            .build_contact(height)
            .per_pair_readout(&self.fresh_mesh(), &positions);
        let mut explicit = Vec3::zeros();
        let mut implicit = Vec3::zeros();
        for r in &readout {
            let ContactPair::Vertex { vertex_id, .. } = r.pair;
            let n_hat = r.normal;
            // ∂sd/∂h = −n̂·ẑ for translating the plane +ẑ (here +1, n̂=−ẑ).
            let dsd_dh = -n_hat.z;
            // explicit: ∂force/∂h|_x = −κ·(∂sd/∂h)·n̂ per active pair.
            explicit += -self.kappa * dsd_dh * n_hat;
            // implicit: (∂force/∂x = −κ·n̂⊗n̂) · ∂x*/∂h.
            let v = vertex_id as usize;
            let dxs = Vec3::new(dxstar[3 * v], dxstar[3 * v + 1], dxstar[3 * v + 2]);
            implicit += -self.kappa * n_hat.dot(&dxs) * n_hat;
        }
        explicit + implicit
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
        let solver: PenaltyRigidContactSolver<HandBuiltTetMesh> =
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
}
