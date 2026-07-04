//! Closed-loop policy and joint design+policy trajectory gradients — backprop-through-time
//! across the state→control recurrence for a state-feedback policy `u_k = π_θ(state_k)`,
//! including the mission's joint `(∂z_N/∂μ, ∂z_N/∂θ)` from one `tape.backward`.

use sim_core::{DMatrix, Matrix3, SpatialVector};
use sim_ml_chassis::{Tape, Tensor, Var};
use sim_soft::{BoundaryConditions, CpuNewtonSolver, Solver, Tet4, Vec3};

use crate::RolloutError;
use crate::StaggeredCoupling;
use crate::contact::{Collider, PlaneContact, SoftSolver};
use crate::policy::DiffPolicy;
use crate::types::{DesignPolicyTape, GripRolloutFrame, PolicyState};
use crate::vjp::{
    ContactForceTrajVjp, ContactWrenchTrajVjp, DriftFromStateVjp, FrictionWrenchTrajVjp,
    PoseCentreVjp, PoseSeamVjp, RigidStateCarryVjp, StateComponentVjp, VelVjp, VzControlCarryVjp,
    WrenchPose, ZCarryVjp, assemble_friction_wrench,
};

impl<C: PlaneContact> StaggeredCoupling<C> {
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
}
