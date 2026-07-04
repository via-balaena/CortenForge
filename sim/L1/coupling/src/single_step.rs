//! Single-step contact force, wrench, and the single-step differentiable factors —
//! the analytic pose/load/material derivatives and the one-step soft-tape `VjpOp` crossings.

use sim_core::SpatialVector;
use sim_ml_chassis::{Tape, Tensor, Var};
use sim_soft::{
    BoundaryConditions, ContactPair, CpuNewtonSolver, HandBuiltTetMesh, LoadAxis, MaterialField,
    RigidTwist, Solver, Tet4, Vec3, VertexId,
};

use crate::StaggeredCoupling;

use crate::contact::{PlaneContact, SoftSolver};
use crate::vjp::{ContactForceVjp, RigidStepVjp, add_contact_moment};

impl<C: PlaneContact> StaggeredCoupling<C> {
    /// Per active contact pair at `positions` with the plane at `height`:
    /// `(vertex_id, soft-force gᵢ, normal n̂, curvature cᵥ, world position rᵢ)` where
    /// the local curvature `cᵥ = d²E/dsd² = n̂ᵀ·H_pair·n̂` is read from the contact's
    /// Hessian (`κ` for penalty, `κ·b''(sd)` for IPC). The single source of the
    /// per-pair readout + curvature extraction (projected by
    /// [`Self::active_pair_force_factors`] into the free-body z-force gradient factors);
    /// the moment routing + its [`ContactWrenchTrajVjp`] gradient additionally use the
    /// force `gᵢ` and contact point `rᵢ`. Does not re-solve/mutate.
    pub(super) fn active_pair_wrench_data(
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
    pub(super) fn contact_wrench(&self, height: f64) -> SpatialVector {
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
    pub(super) fn contact_wrench_gripped(
        &self,
        height: f64,
        friction: &[(VertexId, Vec3)],
    ) -> SpatialVector {
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
    pub(super) fn soft_resolve(&self, height: f64) -> (SoftSolver<C>, Vec<f64>) {
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
}
