//! Articulated trajectory gradients — the multi-step time-adjoints for a hinge/chain
//! arm: the material gradient and the two tangential (friction) material / coefficient
//! gradients, backprop-through-time on one soft↔rigid tape.

use sim_core::{Matrix3, SpatialVector};
use sim_ml_chassis::autograd::VjpOp;
use sim_ml_chassis::{Tape, Tensor};
use sim_soft::{BoundaryConditions, CpuNewtonSolver, RigidTwist, Solver, Tet4, Vec3};

use crate::StaggeredCoupling;
use crate::contact::{Collider, PlaneContact, SoftSolver};
use crate::vjp::{
    ContactWrenchTrajVjp, DriftFromStateVjp, FrictionWrenchTrajVjp, PoseCentreVjp, PoseSeamVjp,
    PoseTwistSeamVjp, RigidStateCarryVjp, VelVjp, WrenchPose, assemble_friction_wrench,
    twist_basis,
};

impl<C: PlaneContact> StaggeredCoupling<C> {
    /// **The multi-DOF (articulated) coupled trajectory gradient — with the off-COM
    /// contact MOMENT.** The articulated successor to
    /// [`Self::coupled_trajectory_material_gradient`]: the rigid body is an
    /// ARTICULATED mechanism (e.g. a hinge), not a free platen, so the soft contact
    /// reaction maps to a generalized joint acceleration coupled across joints (the
    /// matrix `Δt·M⁻¹·Jᵀ`, [`rigid_xfrc_column`](crate::rigid_xfrc_column), not the scalar `dt/m`) and the
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
    /// is `Δt·(M + Δt·D)⁻¹·Jᵀ` ([`rigid_xfrc_column`](crate::rigid_xfrc_column), `D = implicit_damping`). The damped
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
}
