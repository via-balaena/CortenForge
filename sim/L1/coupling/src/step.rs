//! The forward step drivers and their multi-step forward rollouts — the lockstep
//! `step` family (free / articulated / kinematic) and the peak-pressure / grip rollouts.

use sim_core::SpatialVector;
use sim_ml_chassis::Tensor;
use sim_soft::{BoundaryConditions, CpuNewtonSolver, Solver, Tet4, Vec3, peak_contact_pressure};

use crate::StaggeredCoupling;

use crate::contact::{Collider, PlaneContact, SoftSolver};
use crate::types::{CoupledStep, TrajectoryPeakPressure};
use crate::vjp::add_contact_moment;

impl<C: PlaneContact> StaggeredCoupling<C> {
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
}
