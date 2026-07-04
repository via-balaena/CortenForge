//! Free-body trajectory gradients — the multi-step time-adjoints for the free (6-DOF)
//! rigid body: material, angular-velocity, orientation, and peak-force gradients.

use sim_core::{Matrix3, MjJointType, SpatialVector};
use sim_ml_chassis::{Tape, Tensor, Var};
use sim_soft::Vec3;

use crate::StaggeredCoupling;

use crate::contact::PlaneContact;
use crate::vjp::{
    ContactForceTrajVjp, ContactWrenchTrajVjp, PoseSeamVjp, QuatComponentVjp, RigidStateCarryVjp,
    StateComponentVjp, VelVjp, VzCarryVjp, WrenchPose, ZCarryVjp, add_contact_moment,
    rigid_xfrc_column,
};

impl<C: PlaneContact> StaggeredCoupling<C> {
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
    pub(super) fn free_joint_adrs(&self) -> (usize, usize) {
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
    pub(super) fn build_freebody_wrench_tape(
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
}
