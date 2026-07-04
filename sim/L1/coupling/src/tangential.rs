//! Tangential (drift) single-step and trajectory gradients — the velocity-perturbation
//! drift Jacobian and the friction-tangential material / coefficient time-adjoints.

use sim_ml_chassis::{Tape, Tensor};
use sim_soft::{BoundaryConditions, CpuNewtonSolver, Solver, Tet4, Vec3};

use crate::StaggeredCoupling;

use crate::contact::{PlaneContact, SoftSolver};
use crate::vjp::{
    ContactForceTrajVjp, DriftFromVelVjp, FrictionReactionTrajVjp, VelVjp, VzCarryVjp, ZCarryVjp,
};

impl<C: PlaneContact> StaggeredCoupling<C> {
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
