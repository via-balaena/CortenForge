//! Open-loop control trajectory gradients — the actuated (torque-driven) rollouts and
//! their `∂z_N/∂u_k` control-force time-adjoints (actuator, actuator-friction, and the
//! platen-control gradients).

use sim_core::{DMatrix, Matrix3, SpatialVector};
use sim_ml_chassis::autograd::VjpOp;
use sim_ml_chassis::{Tape, Tensor, Var};
use sim_soft::{BoundaryConditions, CpuNewtonSolver, RigidTwist, Solver, Tet4, Vec3};

use crate::StaggeredCoupling;
use crate::contact::{Collider, PlaneContact, SoftSolver};
use crate::vjp::{
    ContactForceTrajVjp, ContactWrenchTrajVjp, DriftFromStateVjp, FrictionWrenchTrajVjp,
    PoseCentreVjp, PoseSeamVjp, RigidStateCarryVjp, VelVjp, VzControlCarryVjp, WrenchPose,
    ZCarryVjp, add_contact_moment, assemble_friction_wrench,
};

impl<C: PlaneContact> StaggeredCoupling<C> {
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
}
