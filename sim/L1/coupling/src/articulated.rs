//! Articulated (multi-DOF) state-Jacobian machinery — the pose seams, the loaded
//! state-transition Jacobians (`∂state'/∂state`), and the single-hinge / chain analytic
//! Jacobians the articulated trajectory adjoints thread through.

use sim_core::{
    DMatrix, DVector, MjJointType, SpatialVector, mass_directional_derivative,
    mj_differentiate_pos, mj_integrate_pos_explicit, mj_jac_point,
};
use sim_soft::Vec3;

use crate::StaggeredCoupling;
use crate::contact::{Collider, PlaneContact};
use crate::vjp::{implicit_mass_inverse, right_jacobian_so3, rigid_xfrc_column};

impl<C: PlaneContact> StaggeredCoupling<C> {
    // ===================== Multi-DOF (articulated) coupling (PR2) =====================

    /// The contact-plane height for an ARTICULATED body: the contacting point's
    /// world height (the body COM `xipos`, the contact point for a point-mass tip)
    /// minus the clearance. Unlike [`Self::plane_height`] (which reads the body
    /// ORIGIN `xpos` — the fixed joint pivot for a hinge), this tracks the moving
    /// tip. For the free-body platen `xipos == xpos`, so the two agree.
    pub(super) fn tip_plane_height(&self) -> f64 {
        self.data.xipos[self.body].z - self.contact_clearance
    }

    /// `J_z`: the world-z row of the contacting body's COM spatial Jacobian
    /// (`mj_jac_point` at `xipos`, row 5) — `∂(tip height)/∂q`, the multi-DOF pose
    /// seam. Length `nv`.
    pub(super) fn pose_seam_jz(&self) -> Vec<f64> {
        let jac = mj_jac_point(
            &self.model,
            &self.data,
            self.body,
            &self.data.xipos[self.body],
        );
        (0..self.model.nv).map(|c| jac[(5, c)]).collect()
    }

    /// `J_spatial`: the body's spatial Jacobian AT THE WORLD ORIGIN (`mj_jac_point` at
    /// the origin) — `6 × nv`, rows 0–2 angular `ω`, rows 3–5 linear `v_O` (the body
    /// material point at the origin). This is exactly `∂(plane spatial twist)/∂qpos`
    /// for the rotating normal: column `j` is the [`sim_soft::RigidTwist`] the plane
    /// undergoes per unit `qvel_j` (`δn̂ = ω×n̂`, `δoffset = v_O·n̂` reproduce
    /// `∂n̂/∂q = −[n̂]×·J_ang` and `∂offset/∂q = n̂ᵀ·J_lin + xiposᵀ·∂n̂/∂q`, S0-validated).
    /// Threaded by [`PoseTwistSeamVjp`] (`∂L/∂qpos = J_spatialᵀ·∂L/∂T`). Rotating-normal
    /// pose seam, PR2; see `docs/keystone/rotating_normal_recon.md`.
    pub(super) fn pose_twist_jacobian(&self) -> DMatrix<f64> {
        mj_jac_point(&self.model, &self.data, self.body, &Vec3::zeros())
    }

    /// `J_lin = ∂c/∂qpos`: the body COM's linear (translational) world Jacobian
    /// (`mj_jac_point` at `xipos`, rows 3–5) — `3 × nv`. The moment about the COM
    /// `c = xipos(q)` depends on `q` through `c`, so [`ContactWrenchTrajVjp`] needs
    /// this to thread `∂τ/∂qpos = [f]_× · J_lin`. Read at the same (stale) FK config
    /// as the pose seam.
    pub(super) fn com_linear_jacobian(&self) -> DMatrix<f64> {
        let jac = mj_jac_point(
            &self.model,
            &self.data,
            self.body,
            &self.data.xipos[self.body],
        );
        let nv = self.model.nv;
        let mut jlin = DMatrix::zeros(3, nv);
        for row in 0..3 {
            for col in 0..nv {
                jlin[(row, col)] = jac[(row + 3, col)]; // linear rows 3–5
            }
        }
        jlin
    }

    /// `J_geom = ∂(contact-geom centre)/∂qpos`: the contact end-effector geom's linear
    /// world Jacobian (`mj_jac_point` at `geom_xpos[contact_geom]`, rows 3–5) — `3 × nv`.
    /// The moving-end-effector pose seam: with [`Self::with_contact_geom`] the sphere
    /// centre rides the geom, so the trajectory adjoint threads `∂centre/∂q = J_geom`
    /// (the [`PoseCentreVjp`] seam), the 3-vector generalization of [`Self::pose_seam_jz`]'s
    /// scalar height channel. Read at the same (fresh-FK) config as the wrench node.
    ///
    /// # Panics
    /// Panics if no contact geom is set (`with_contact_geom` not called) — the centre
    /// channel is only meaningful for an end-effector-posed sphere — or if the contact geom is
    /// not rigidly attached to the contacting body (the coupling routes the reaction wrench to
    /// `self.body`, so the centre must ride it for the model to be coherent; `mj_jac_point`
    /// would otherwise return the Jacobian of the wrong body's material point).
    // The `expect` enforces the documented `# Panics` contract (a centre Jacobian
    // is only meaningful for an end-effector-posed sphere); a misconfigured call
    // is a loud programmer error, not a recoverable condition.
    #[allow(clippy::expect_used)]
    pub(super) fn pose_centre_jacobian(&self) -> DMatrix<f64> {
        let g = self
            .contact_geom
            .expect("pose_centre_jacobian requires with_contact_geom");
        assert_eq!(
            self.model.geom_body[g], self.body,
            "the contact geom must be attached to the contacting body (self.body) — the moving-EE \
             centre Jacobian ∂(geom_xpos)/∂q treats the geom as rigidly fixed to self.body"
        );
        let jac = mj_jac_point(&self.model, &self.data, self.body, &self.data.geom_xpos[g]);
        let nv = self.model.nv;
        let mut jlin = DMatrix::zeros(3, nv);
        for row in 0..3 {
            for col in 0..nv {
                jlin[(row, col)] = jac[(row + 3, col)]; // linear rows 3–5
            }
        }
        jlin
    }

    /// One scratch rigid step from a supplied `(qpos, qvel)` with a held spatial
    /// `wrench` on `body`, returning `(qpos', qvel')`. Like [`Self::rigid_step_probe`]
    /// but for the full generalized state (the LOADED step map whose Jacobian is the
    /// multi-DOF carry). Does NOT advance `self` (`Data` is not `Clone`, so a fresh
    /// scratch is built).
    ///
    /// **Linear-axis damping.** When `rigid_damping != 0` (the free-platen contact-axis damping
    /// the wrench carry now supports), the velocity-dependent damping force `−c·vz` is added on
    /// the world-z linear force from the SUPPLIED `qvel` — so a position/velocity FD over this
    /// step sees the damping vary with the perturbed state and the loaded `J_state` picks up its
    /// `a = 1 − c·Δt/m` velocity-coupling. The damped vz is `qvel[2]`, matching [`Self::step`]'s
    /// `sf[5] = −fz − c·qvel[2]` exactly (the free platen's z velocity; `rigid_damping` is the
    /// free-body knob, so this branch only fires for a free body). This is what lets the general
    /// carry reproduce the scalar `VzCarryVjp`/`ZCarryVjp` z-carry under damping. `c = 0` ⇒ the
    /// branch is skipped, byte-identical for the articulated/hinge paths (which assert `c = 0`).
    ///
    /// # Panics
    /// Panics if the scratch step diverges (a mis-constructed model).
    // expect_used: a scratch step on a valid model does not fail; a divergence is a
    // programmer error surfaced loudly (mirrors `rigid_step_probe`).
    #[allow(clippy::expect_used)]
    fn scratch_state_step(
        &self,
        qpos: &nalgebra::DVector<f64>,
        qvel: &nalgebra::DVector<f64>,
        wrench: &SpatialVector,
    ) -> (nalgebra::DVector<f64>, nalgebra::DVector<f64>) {
        let mut scratch = self.model.make_data();
        scratch.qpos.copy_from(qpos);
        scratch.qvel.copy_from(qvel);
        let mut w = *wrench;
        if self.rigid_damping != 0.0 {
            w[5] += -self.rigid_damping * qvel[2]; // velocity-dependent contact-axis damping (free platen z)
        }
        scratch.xfrc_applied[self.body] = w;
        // Replicate the held control so the actuator force is present during the FD: on a
        // CHAIN the actuator force interacts with `∂M⁻¹/∂q`, so a `ctrl`-blind step would
        // drop that from the loaded `J_state`. `ctrl = 0` (no actuator / unactuated leaf)
        // ⇒ byte-identical to the original. (Single hinge: the constant force leaves
        // `J_state` unchanged, so this is a no-op there too.)
        scratch.ctrl.copy_from(&self.data.ctrl);
        scratch.step(&self.model).expect("articulated probe step");
        (scratch.qpos.clone(), scratch.qvel.clone())
    }

    /// [`rigid_xfrc_column`] (the wrench→qvel response `Δt·M⁻¹·Jᵀ`) at the current
    /// config — the config the real `step` maps the wrench through, matching the eval
    /// point of [`Self::loaded_state_jacobian`].
    ///
    /// Under the fully-fresh formulation the caller has already re-forwarded `self.data`
    /// at the current `qpos` this step (the per-step `forward` in
    /// `coupled_trajectory_material_gradient_articulated`), so `self.data`'s
    /// `xipos`/`qM`/`J` are current and `rigid_xfrc_column` reads them directly — no
    /// scratch forward needed. (Before the fresh-FK fix, `self.data` lagged `qpos` by one
    /// step, so this routine forwarded a scratch to remove a one-step skew that was
    /// material for the rotational/moment Jacobian; the fresh-FK formulation makes that
    /// scratch redundant. See `docs/keystone/moment_residual_recon.md` §3f.)
    pub(super) fn fresh_xfrc_column(&self) -> DMatrix<f64> {
        rigid_xfrc_column(&self.model, &self.data, self.body)
    }

    /// The actuator-control VELOCITY response `∂(qvel')/∂ctrl = Δt·M_impl⁻¹·∂qfrc_actuator/∂ctrl`
    /// (`nv × nu`) at the current (fresh-FK) config — the actuator analog of the wrench
    /// column [`Self::fresh_xfrc_column`], routed through the actuator transmission instead
    /// of an applied wrench.
    ///
    /// `∂qfrc_actuator/∂ctrl` is computed by a forward-only central FD of `qfrc_actuator`
    /// (a scratch `forward` at `qpos`/`qvel` with `ctrl_a ± ε`). For the affine force law
    /// `force = gain·ctrl + bias` this is EXACT to roundoff (`qfrc_actuator` is affine in
    /// `ctrl`, so the central difference has no truncation error) and robust to how the
    /// transmission `moment`/`gain` are stored (the persisted `data.actuator_moment` is
    /// transient — cleared after forward — whereas `qfrc_actuator` is the reliable output).
    /// The POSITION rows are `D·(this)` (the same integrator tangent `D` as the wrench's
    /// `G_pos`), assembled by the caller. See `docs/keystone/actuator_dynamics_recon.md`.
    // expect_used: a scratch forward on a valid model does not fail (mirrors `scratch_state_step`).
    #[allow(clippy::expect_used)]
    pub(super) fn actuator_velocity_column(&self) -> DMatrix<f64> {
        let (nv, nu) = (self.model.nv, self.model.nu);
        let eps = 1e-6;
        let qfrc = |a: usize, du: f64| -> DVector<f64> {
            let mut d = self.model.make_data();
            d.qpos.copy_from(&self.data.qpos);
            d.qvel.copy_from(&self.data.qvel);
            d.ctrl[a] += du;
            d.forward(&self.model)
                .expect("scratch forward for actuator Jacobian");
            d.qfrc_actuator.clone()
        };
        let mut dqfrc = DMatrix::zeros(nv, nu);
        for a in 0..nu {
            let col = (qfrc(a, eps) - qfrc(a, -eps)) / (2.0 * eps);
            for i in 0..nv {
                dqfrc[(i, a)] = col[i];
            }
        }
        self.model.timestep * implicit_mass_inverse(&self.model, &self.data) * dqfrc
    }

    /// The LOADED single-step state Jacobian `∂[qpos'; qvel']/∂[qpos; qvel]`
    /// (`2·nv × 2·nv`) at the current rigid state, with the contact `wrench` HELD —
    /// computed by central finite differences over [`Self::scratch_state_step`].
    ///
    /// This is the multi-DOF state carry. It is the *loaded* Jacobian (the wrench
    /// is held during the perturbation), so it includes the applied-force geometric
    /// stiffness `∂(Jᵀw)/∂q` that `sim-core`'s unloaded `transition_derivatives`
    /// drops — a real effect for an articulated body (zero for the free-body platen,
    /// where `J = I`).
    ///
    /// This is the **general fallback** path (any `nv`): the single-hinge AND the undamped
    /// serial-hinge-chain scopes use the machine-exact analytic [`Self::analytic_state_jacobian`]
    /// instead, while a free/quaternion joint or a DAMPED chain uses this FD form (FD-carry
    /// precision ~1e-6). It is also the reference the analytic form is FD-validated against;
    /// see `docs/keystone/geometric_stiffness_recon.md` and `docs/keystone/multilink_recon.md`.
    ///
    /// **Tangent-space FD (SO(3)-correct for quaternion joints).** When the body has a
    /// quaternion joint (`nq ≠ nv` — `qpos` holds un-normalized quaternion components on
    /// a curved manifold) the position-coordinate FD must move ALONG the manifold and
    /// difference in its tangent space, not a raw `qpos` add/subtract: position COLUMNS
    /// step via [`Self::loaded_state_jacobian_tangent`]'s [`mj_integrate_pos_explicit`]
    /// (`qpos ⊕ exp(±ε·e_c)`) and position ROWS difference via [`mj_differentiate_pos`]
    /// (the SO(3) log, the body-frame tangent at the nominal output). A purely Euclidean
    /// body (`nq == nv`: hinge/slide/translation only — e.g. a damped chain or a
    /// translation path) keeps the raw FD verbatim, BYTE-IDENTICAL. (The single hinge and
    /// the undamped serial-hinge chain use the analytic Jacobian and never reach here.) See
    /// `docs/keystone/quaternion_joints_recon.md`.
    pub(super) fn loaded_state_jacobian(&self, wrench: &SpatialVector) -> DMatrix<f64> {
        if self.model.nq != self.model.nv {
            return self.loaded_state_jacobian_tangent(wrench);
        }
        let nv = self.model.nv;
        let eps = 1e-6;
        let qpos = self.data.qpos.clone();
        let qvel = self.data.qvel.clone();
        let mut j = DMatrix::zeros(2 * nv, 2 * nv);
        for c in 0..2 * nv {
            let (mut qp, mut vp) = (qpos.clone(), qvel.clone());
            let (mut qm, mut vm) = (qpos.clone(), qvel.clone());
            if c < nv {
                qp[c] += eps;
                qm[c] -= eps;
            } else {
                vp[c - nv] += eps;
                vm[c - nv] -= eps;
            }
            let (qpp, qvp) = self.scratch_state_step(&qp, &vp, wrench);
            let (qpm, qvm) = self.scratch_state_step(&qm, &vm, wrench);
            for r in 0..nv {
                j[(r, c)] = (qpp[r] - qpm[r]) / (2.0 * eps);
                j[(nv + r, c)] = (qvp[r] - qvm[r]) / (2.0 * eps);
            }
        }
        j
    }

    /// The quaternion-joint (`nq ≠ nv`) branch of [`Self::loaded_state_jacobian`]: the
    /// same loaded central-FD state Jacobian, but the position coordinates are perturbed
    /// and differenced in the SO(3)/SE(3) **tangent** space (the only correct linearization
    /// of a curved-manifold coordinate). The result is the `2·nv × 2·nv` tangent Jacobian
    /// `∂[tangent qpos'; qvel']/∂[tangent qpos; qvel]` — drop-in for the carry, whose state
    /// `s = [qpos(nv-tangent); qvel(nv)]` is already tangent-dimensioned. Velocity stays a
    /// raw `nv` tangent quantity (no quaternion). See `docs/keystone/quaternion_joints_recon.md`.
    fn loaded_state_jacobian_tangent(&self, wrench: &SpatialVector) -> DMatrix<f64> {
        let nv = self.model.nv;
        let eps = 1e-6;
        let qpos = self.data.qpos.clone();
        let qvel = self.data.qvel.clone();
        let mut j = DMatrix::zeros(2 * nv, 2 * nv);
        let mut tang = DVector::zeros(nv);
        let mut dq = DVector::zeros(nv);
        for c in 0..2 * nv {
            let (mut qp, mut vp) = (qpos.clone(), qvel.clone());
            let (mut qm, mut vm) = (qpos.clone(), qvel.clone());
            if c < nv {
                // Position column: step ±ε along the tangent basis vector e_c on SO(3)
                // (a hinge/slide/translation DOF reduces to `qpos[c] ± ε`).
                tang[c] = eps;
                mj_integrate_pos_explicit(&self.model, &mut qp, &qpos, &tang, 1.0);
                tang[c] = -eps;
                mj_integrate_pos_explicit(&self.model, &mut qm, &qpos, &tang, 1.0);
                tang[c] = 0.0;
            } else {
                vp[c - nv] += eps;
                vm[c - nv] -= eps;
            }
            let (qpp, qvp) = self.scratch_state_step(&qp, &vp, wrench);
            let (qpm, qvm) = self.scratch_state_step(&qm, &vm, wrench);
            // Position rows: tangent (SO(3) log) difference of the `qpos'` outputs.
            mj_differentiate_pos(&self.model, &mut dq, &qpm, &qpp, 2.0 * eps);
            for r in 0..nv {
                j[(r, c)] = dq[r];
                j[(nv + r, c)] = (qvp[r] - qvm[r]) / (2.0 * eps);
            }
        }
        j
    }

    /// The integrator's tangent position Jacobian `D = ∂(tangent qpos')/∂qvel'` (`nv × nv`,
    /// block-diagonal per joint) at the post-step velocity `qvel_next`. Semi-implicit Euler
    /// maps `qpos' = qpos ⊕ exp(Δt·qvel')`; differentiating w.r.t. `qvel'` (output tangent
    /// at the nominal `qpos'`) gives, per joint: `Δt` for a hinge/slide/free-translation DOF
    /// (the linear limit `J_r = I`), and `Δt·J_r(Δt·ω)` for a ball / free-orientation block
    /// (`ω` = that joint's angular velocity, [`right_jacobian_so3`]). The carry's position-row
    /// wrench response is then `G_pos = D·G_vel`. Only the quaternion path calls this; the
    /// Euclidean path uses `Δt·G_vel` directly (byte-identical). See
    /// `docs/keystone/quaternion_joints_recon.md`.
    pub(super) fn integrator_pos_jacobian(&self, qvel_next: &DVector<f64>) -> DMatrix<f64> {
        let nv = self.model.nv;
        let dt = self.model.timestep;
        let mut d = DMatrix::zeros(nv, nv);
        for jnt in 0..self.model.njnt {
            let dof = self.model.jnt_dof_adr[jnt];
            match self.model.jnt_type[jnt] {
                MjJointType::Hinge | MjJointType::Slide => {
                    d[(dof, dof)] = dt;
                }
                MjJointType::Ball => {
                    let omega = Vec3::new(qvel_next[dof], qvel_next[dof + 1], qvel_next[dof + 2]);
                    d.view_mut((dof, dof), (3, 3))
                        .copy_from(&(dt * right_jacobian_so3(dt * omega)));
                }
                MjJointType::Free => {
                    for i in 0..3 {
                        d[(dof + i, dof + i)] = dt; // linear (translation) block
                    }
                    let omega =
                        Vec3::new(qvel_next[dof + 3], qvel_next[dof + 4], qvel_next[dof + 5]);
                    d.view_mut((dof + 3, dof + 3), (3, 3))
                        .copy_from(&(dt * right_jacobian_so3(dt * omega)));
                }
            }
        }
        d
    }

    /// The body's joint id if its kinematic chain to the world is exactly one
    /// **hinge** with no other DOFs (`nv == 1`, hence `nq == nv` — raw `qpos` is a
    /// valid coordinate, no quaternion). `None` for a free joint, a multi-link chain,
    /// or any non-hinge case. The single hinge uses the scalar closed-form geometric
    /// stiffness in [`Self::analytic_state_jacobian`]; the multi-link chain (damped or
    /// undamped) uses the off-diagonal Hessian + `∂M_impl⁻¹/∂q` form in
    /// [`Self::chain_state_jacobian`]; the remaining cases (free/quaternion) keep the FD
    /// [`Self::loaded_state_jacobian`] fallback.
    pub(super) fn single_hinge(&self) -> Option<usize> {
        if self.model.nv != 1 || self.model.body_jnt_num[self.body] != 1 {
            return None;
        }
        let jnt = self.model.body_jnt_adr[self.body];
        (self.model.jnt_type[jnt] == sim_core::MjJointType::Hinge).then_some(jnt)
    }

    /// The LOADED single-step state Jacobian `∂[qpos'; qvel']/∂[qpos; qvel]` computed
    /// **analytically** for a single hinge — the machine-exact successor to the FD'd
    /// [`Self::loaded_state_jacobian`]. `None` for any non-single-hinge body (the
    /// caller falls back to the FD form).
    ///
    /// `J_state = A + Δt·M⁻¹·∂(Jᵀw)/∂q` (velocity rows; the position rows follow the
    /// semi-implicit integrator, scaled by `Δt`), where `A` is the **unloaded**
    /// transition ([`Data::transition_derivatives`]) and the second term is the
    /// applied wrench's geometric/load stiffness that `A` drops. This replaces the FD
    /// loaded Jacobian's noise with the exact term (deterministic, no eps) — making the
    /// single-hinge articulated gradient machine-exact at every horizon (paired with the
    /// fully-fresh formulation; see `coupled_trajectory_material_gradient_articulated`
    /// and `docs/keystone/moment_residual_recon.md`). The undamped multi-link chain is
    /// handled by [`Self::chain_state_jacobian`] (this method dispatches to it).
    ///
    /// For a single hinge the geometric stiffness is the closed form
    /// `∂(Jᵀw)/∂θ = (â×(â×r))·f`: with world axis `â`, moment arm `r = xipos − anchor`,
    /// and the wrench's linear part `f`, only `r` rotates with `θ` (`∂â/∂θ = 0`,
    /// `∂anchor/∂θ = 0`), so `∂r/∂θ = â×r` and `∂(â×r)/∂θ = â×(â×r)`; the torque part
    /// drops (`∂â/∂θ = 0`). `M` is configuration-independent for a single hinge (the
    /// body inertia about the fixed axis), so `∂M⁻¹/∂q = 0` and the `Δt·M⁻¹·…` form is
    /// exact. Evaluated at a fresh scratch forward at `qpos` (the config the real
    /// `step` maps the wrench through, matching [`Self::fresh_xfrc_column`]).
    ///
    /// **Joint damping (eulerdamp).** Under the Euler integrator MuJoCo solves the velocity
    /// update with the IMPLICIT factor `M_impl = M + Δt·D` (`D = implicit_damping`), not bare
    /// `M`. The unloaded `A` from `transition_derivatives` already accounts for this — sim-core
    /// routes its Euler velocity solve through `M_impl` when any DOF is damped (see
    /// `derivatives/hybrid.rs`). So the carry only adds the LOADED geometric stiffness, routed
    /// through the same `M_impl` (`∂(M_impl⁻¹·Jᵀw)/∂θ = geom_stiff / M_impl`), and the position
    /// rows follow the semi-implicit chain `θ' = θ + Δt·ω'`. `M_impl` is configuration-independent
    /// for a single hinge (constant `M`, constant `D`), so `∂M_impl⁻¹/∂q = 0` and the form stays
    /// exact — machine-exact vs the FD [`Self::loaded_state_jacobian`] (which differentiates the
    /// real eulerdamp step). `D = 0 ⇒ M_impl = M`, recovering the bare-`M` path BYTE-FOR-BYTE.
    // expect_used: a transition-derivative / forward on a valid model does not fail; a
    // failure is a programmer error surfaced loudly (mirrors `scratch_state_step`).
    #[allow(clippy::expect_used)]
    pub(super) fn analytic_state_jacobian(&self, wrench: &SpatialVector) -> Option<DMatrix<f64>> {
        // The analytic carry assumes the Euler (eulerdamp) integrator: damping enters the
        // velocity solve as `M_impl = M + Δt·D`, with joint stiffness applied EXPLICITLY. A
        // stiffness-implicit integrator (`ImplicitSpringDamper`: `M + Δt·D + Δt²·K`) needs a
        // different solve matrix, so defer to the always-correct FD `loaded_state_jacobian`.
        // (Before damped chains were analytic, the damping guard incidentally excluded such
        // models — which are damped — so this preserves that protection explicitly.)
        if self.model.integrator != sim_core::Integrator::Euler {
            return None;
        }
        let Some(jnt) = self.single_hinge() else {
            // Not a single hinge: try the analytic multi-link chain carry (damped or undamped,
            // `Self::chain_state_jacobian`); a free joint / quaternion / multi-joint-per-body
            // body returns `None` there → the caller's FD `loaded_state_jacobian` fallback.
            return self.chain_state_jacobian(wrench);
        };
        let dt = self.model.timestep;
        let nv = self.model.nv; // == 1 (single_hinge), so the state block is 2×2
        // Unloaded transition A at the current state. The real `step` re-forwards at
        // `qpos`, so A's internal perturbations evaluate at the same fresh config the
        // carry uses (`self.data`'s `xipos`/`qM` lag a step, but `step`/`forward` redo FK).
        // A is (2·nv + na)²; take the [qpos; qvel] block (the carry's 2·nv × 2·nv state).
        let a = self
            .data
            .transition_derivatives(&self.model, &sim_core::DerivativeConfig::default())
            .expect("unloaded transition derivatives")
            .A;
        let mut j = a.view((0, 0), (2 * nv, 2 * nv)).into_owned();
        // Geometric stiffness at a fresh scratch forward at `qpos` (matching the FD
        // `loaded_state_jacobian` / `fresh_xfrc_column` eval point).
        let mut scratch = self.model.make_data();
        scratch.qpos.copy_from(&self.data.qpos);
        scratch.qvel.copy_from(&self.data.qvel);
        scratch.forward(&self.model).expect("scratch forward");
        let jb = self.model.jnt_body[jnt];
        let axis = scratch.xquat[jb] * self.model.jnt_axis[jnt];
        let anchor = scratch.xpos[jb] + scratch.xquat[jb] * self.model.jnt_pos[jnt];
        let r = scratch.xipos[self.body] - anchor;
        let f = Vec3::new(wrench[3], wrench[4], wrench[5]);
        let geom_stiff = axis.cross(&axis.cross(&r)).dot(&f); // ∂(Jᵀw)/∂θ
        let m = scratch.qM[(0, 0)];
        let damp = self.model.implicit_damping[0]; // single hinge ⇒ the only DOF
        // `A` (= `j`, from `transition_derivatives`) is ALREADY the eulerdamp-correct
        // unloaded transition: sim-core routes the Euler velocity solve through
        // `M_impl = M + Δt·D` under damping (and through bare `M` when undamped). So the
        // carry only patches the LOADED geometric stiffness onto the qpos column, routed
        // through the same `M_impl` (`∂(M_impl⁻¹·Jᵀw)/∂θ = geom_stiff / M_impl`); the
        // position rows follow the semi-implicit chain `θ' = θ + Δt·ω'`. `M_impl = M`
        // when undamped, so the `D = 0` result is unchanged BYTE-FOR-BYTE.
        let m_impl = m + dt * damp;
        let vel_corr = dt / m_impl * geom_stiff; // ∂qvel'/∂qpos correction
        j[(1, 0)] += vel_corr; // velocity row, qpos col
        j[(0, 0)] += dt * vel_corr; // position row, qpos col (semi-implicit chain)
        Some(j)
    }

    /// The LOADED single-step state Jacobian computed **analytically** for a serial HINGE chain
    /// (`nv ≥ 2`), with or without joint damping — the multi-link successor to the single-hinge
    /// [`Self::analytic_state_jacobian`], removing the last FD fallback for the articulated
    /// coupling. `None` (→ FD `loaded_state_jacobian`) for any body that is not a pure serial
    /// hinge chain spanning all DOFs (one joint per body) or a quaternion joint (`nq ≠ nv`).
    /// Joint damping is handled by routing the loaded term through `M_impl = M + Δt·D` (the Euler
    /// eulerdamp solve matrix); the unloaded `A` is eulerdamp-correct from sim-core. Machine-exact
    /// for 2-link through 4-link spatial chains (FD-validated damped and undamped); the multi-hop
    /// Coriolis derivative it relies on is complete in `mjd_rne_pos`.
    ///
    /// **The decomposition.** The loaded single-step transition is exactly
    /// `loaded_J = A + Δt·∂(M_impl⁻¹·Jᵀw)/∂q` on the velocity-position block and `+ Δt²·∂(…)` on
    /// the position-position block (semi-implicit `q' = q + Δt·v'`), where `M_impl = M + Δt·D` is
    /// the Euler eulerdamp solve matrix (`= M` when undamped), and:
    /// - `A` is the **unloaded** transition ([`Data::transition_derivatives`]) — already
    ///   machine-exact (its `∂v'/∂q` block captures the mass-matrix config dependence via
    ///   `sim-core`'s `mjd_rne_pos` inertia-transport derivatives, and it routes the Euler
    ///   velocity solve through `M_impl` under damping).
    /// - `∂(M_impl⁻¹·Jᵀw)/∂q = M_impl⁻¹·(G − dMu)`, with `G = ∂(Jᵀw)/∂q` the applied-wrench
    ///   geometric stiffness Hessian and `dMu = (∂M/∂q)·u` (`u = M_impl⁻¹·Jᵀw`; `∂M_impl/∂q =
    ///   ∂M/∂q` since `D` is constant) the mass-matrix directional derivative
    ///   [`sim_core::mass_directional_derivative`]. The `(∂M_impl⁻¹/∂q)·Jᵀw = −M_impl⁻¹·dMu`
    ///   term VANISHES for a single hinge (constant `M`) but is MATERIAL for a chain (~14% of
    ///   the wrench contribution); dropping it ships a ~10%-wrong gradient.
    ///
    /// **The geometric-stiffness Hessian `G[j,k] = ∂(Jᵀw)_j/∂q_k`.** With `(Jᵀw)_j = â_j·(τ +
    /// r_j×f)` (joint `j`: world axis `â_j`, anchor `o_j`, arm `r_j = p − o_j` to the contact
    /// COM `p`; wrench `w = [τ; f]`):
    /// `G[j,k] = (∂â_j/∂q_k)·(τ + r_j×f) + â_j·((∂r_j/∂q_k)×f)`, with the ancestor-dependent
    /// kinematics `∂â_j/∂q_k = â_k×â_j` and `∂o_j/∂q_k = â_k×(o_j−o_k)` (both only when `k` is a
    /// STRICT ancestor of `j`, else `0`), and `∂p/∂q_k = â_k×(p−o_k)` (for any chain joint `k`),
    /// so `∂r_j/∂q_k = ∂p/∂q_k − ∂o_j/∂q_k`. For a single hinge (`j=k`) this reduces to the
    /// closed form `(â×(â×r))·f` of [`Self::analytic_state_jacobian`]. ✓
    ///
    /// All kinematics are read at a FRESH scratch `forward` at `qpos` (matching the FD
    /// `loaded_state_jacobian` / [`Self::fresh_xfrc_column`] eval point). See
    /// `docs/keystone/multilink_recon.md` and `project-analytic-chain-jstate-subpr2` (the spec).
    // expect_used: a transition-derivative / forward on a valid model does not fail; a
    // failure is a programmer error surfaced loudly (mirrors `analytic_state_jacobian`).
    #[allow(clippy::expect_used)]
    fn chain_state_jacobian(&self, wrench: &SpatialVector) -> Option<DMatrix<f64>> {
        let model = &self.model;
        // Scope guard: undamped serial HINGE chain spanning all DOFs, Euclidean (nq == nv),
        // nv ≥ 2. Validated machine-exact for 2-link through (at least) 4-link spatial chains —
        // the sim-core Coriolis derivative (the `∂S/∂q` ancestor term AND the bias-acceleration
        // X_b transport that the multi-hop case needs) is now complete for any serial hinge
        // chain. See `docs/keystone/multilink_recon.md`.
        if model.nq != model.nv || model.nv < 2 {
            return None;
        }
        let chain = &model.body_ancestor_joints[self.body];
        // The chain must be exactly the model's DOFs (each hinge = 1 DOF), all hinges, one
        // joint per body. Joint DAMPING is supported: the loaded term routes through `M_impl`
        // below, and the unloaded `A` is eulerdamp-correct from sim-core. The one-joint-per-body
        // guard matters: with multiple joints on a body, this body's world axis/anchor read
        // (`xquat[body] * jnt_axis`, the FINAL body frame) differs from FK's running-frame axis
        // for a non-last same-body joint, and `is_ancestor` flags same-body joints as mutual
        // strict ancestors — so such a model would get a wrong analytic `J_state` instead of the
        // safe FD fallback.
        if chain.len() != model.nv
            || !chain
                .iter()
                .all(|&j| model.jnt_type[j] == MjJointType::Hinge)
            || !chain
                .iter()
                .all(|&j| model.body_jnt_num[model.jnt_body[j]] == 1)
        {
            return None;
        }

        let dt = model.timestep;
        let nv = model.nv;

        // Fresh scratch at `qpos`/`qvel` with the CONTACT WRENCH CLEARED (`xfrc_applied = 0`)
        // but the held control copied — the UNLOADED operating point. `self.data` carries the
        // previous step's `xfrc_applied` (sim-core's `step` does not clear it), which would
        // poison the analytical transition's `∂(M·qacc)/∂q` term via the contaminated
        // operating-point `qacc` (∂M/∂q ≠ 0 for a chain — harmless for a single hinge where
        // ∂M/∂q = 0, which is why `analytic_state_jacobian` can read `self.data` directly).
        // A is the unloaded transition; the held contact wrench enters only through `addend`.
        let mut scratch = model.make_data();
        scratch.qpos.copy_from(&self.data.qpos);
        scratch.qvel.copy_from(&self.data.qvel);
        scratch.ctrl.copy_from(&self.data.ctrl); // actuator input is a held input of the transition
        scratch.forward(model).expect("scratch forward");

        // Unloaded transition A at the clean operating point (undamped ⇒ machine-exact for the
        // chain, given the sim-core Coriolis `∂S/∂q` ancestor term).
        let a = scratch
            .transition_derivatives(model, &sim_core::DerivativeConfig::default())
            .expect("unloaded transition derivatives")
            .A;
        let mut j = a.view((0, 0), (2 * nv, 2 * nv)).into_owned();

        let p = scratch.xipos[self.body]; // contact-body COM
        let tau = Vec3::new(wrench[0], wrench[1], wrench[2]);
        let f = Vec3::new(wrench[3], wrench[4], wrench[5]);

        // Per-joint world axis â and anchor o (mirror the single-hinge eval).
        let axis: Vec<Vec3> = chain
            .iter()
            .map(|&jj| scratch.xquat[model.jnt_body[jj]] * model.jnt_axis[jj])
            .collect();
        let anchor: Vec<Vec3> = chain
            .iter()
            .map(|&jj| {
                scratch.xpos[model.jnt_body[jj]]
                    + scratch.xquat[model.jnt_body[jj]] * model.jnt_pos[jj]
            })
            .collect();

        // Geometric-stiffness Hessian G[dof_j, dof_k] = ∂(Jᵀw)_j/∂q_k (case-split closed form).
        let mut g = DMatrix::zeros(nv, nv);
        for (ji, &jj) in chain.iter().enumerate() {
            let a_j = axis[ji];
            let r_j = p - anchor[ji];
            let moment_j = tau + r_j.cross(&f); // τ + r_j×f
            let dof_j = model.jnt_dof_adr[jj];
            for (ki, &kk) in chain.iter().enumerate() {
                let a_k = axis[ki];
                // ∂p/∂q_k = â_k×(p−o_k) (k is always an ancestor of the contact body).
                let dp = a_k.cross(&(p - anchor[ki]));
                // ∂â_j/∂q_k, ∂o_j/∂q_k nonzero only when k is a STRICT ancestor of j.
                let strict_anc = kk != jj && model.is_ancestor(model.jnt_body[jj], kk);
                let (daxis_j, do_j) = if strict_anc {
                    (a_k.cross(&a_j), a_k.cross(&(anchor[ji] - anchor[ki])))
                } else {
                    (Vec3::zeros(), Vec3::zeros())
                };
                let dr_j = dp - do_j; // ∂r_j/∂q_k
                let dof_k = model.jnt_dof_adr[kk];
                g[(dof_j, dof_k)] = daxis_j.dot(&moment_j) + a_j.dot(&dr_j.cross(&f));
            }
        }

        // The loaded velocity-row term is Δt·∂(M_impl⁻¹·Jᵀw)/∂q, where `M_impl = M + Δt·D` is the
        // Euler eulerdamp solve matrix (= M when undamped). Expanding with ∂M_impl/∂q = ∂M/∂q
        // (D constant): ∂(M_impl⁻¹·Jᵀw)/∂q = M_impl⁻¹·(G − dMu), with u = M_impl⁻¹·Jᵀw and
        // dMu = (∂M/∂q)·u. The unloaded `A` is already eulerdamp-correct from sim-core (it routes
        // the Euler velocity solve through M_impl under damping), so the carry only adds this
        // loaded term over the SAME M_impl — mirroring the single-hinge unification. `D = 0 ⇒
        // M_impl = M`, recovering the bare-M form BYTE-FOR-BYTE.
        let mut m_impl = scratch.qM.clone();
        for i in 0..nv {
            m_impl[(i, i)] += dt * model.implicit_damping[i];
        }
        let m_inv = m_impl.try_inverse().expect("M_impl invertible");
        let jac = mj_jac_point(model, &scratch, self.body, &p); // 6×nv
        let mut w_vec = DVector::zeros(6);
        for i in 0..6 {
            w_vec[i] = wrench[i];
        }
        let jtw = jac.transpose() * &w_vec; // nv
        let u = &m_inv * &jtw; // u = M_impl⁻¹·Jᵀw
        let d_mu = mass_directional_derivative(model, &mut scratch, &u); // nv×nv
        let addend = dt * &m_inv * (&g - &d_mu); // Δt·M_impl⁻¹·(G − dMu)

        // Velocity rows + semi-implicit position rows get the loaded correction on the qpos
        // columns (mirrors the single-hinge `vel_corr` / `dt·vel_corr` patch).
        for r in 0..nv {
            for c in 0..nv {
                j[(nv + r, c)] += addend[(r, c)]; // velocity row, qpos col
                j[(r, c)] += dt * addend[(r, c)]; // position row, qpos col (semi-implicit)
            }
        }
        Some(j)
    }

    /// Forward-only oracle for [`Self::coupled_trajectory_material_gradient_articulated`]:
    /// roll the ARTICULATED coupled system forward `n_steps` (pose the plane from
    /// the moving tip, soft solve, route the contact reaction at the body COM, step)
    /// and return the tip world height `xipos[body].z` after the rollout. No tape;
    /// the black-box oracle for finite-differencing the multi-DOF coupled gradient.
    /// Advances `self` (build a fresh coupling per call).
    ///
    /// # Panics
    /// Panics if the rigid step diverges or the soft solver fails (as in [`Self::step`]).
    #[must_use]
    // expect_used: a rigid-step divergence / soft non-convergence is a programmer
    // error surfaced loudly, not recoverable for keystone-v1 (mirrors `step`).
    #[allow(clippy::expect_used)]
    pub fn coupled_trajectory_articulated_z(&mut self, n_steps: usize) -> f64 {
        for _ in 0..n_steps {
            // FRESH FK: pose the contact plane / COM from the CURRENT config (q_k).
            // sim-core's `step` is forward-then-integrate with no trailing FK, so
            // `self.data`'s `xipos`/`qM` lag `qpos` one step; re-forward so the contact
            // is posed at the actual current tip (no one-step lag). This fresh
            // formulation (paired with the gradient method's fresh output + true
            // position-row carry) is machine-exact for single-hinge AND multi-link; the
            // earlier stale-FK convention was a single-hinge-only calibration. See
            // `docs/keystone/moment_residual_recon.md`.
            self.data.forward(&self.model).expect("fresh FK");
            // MOVING END-EFFECTOR: pose the sphere at the contact geom (the arm tip) each
            // step — the forward companion of the moving-EE centre gradient, so the FD
            // oracle and the adjoint share the per-step posing. Sphere-only (the plane
            // ignores the override); a no-op (override stays None ⇒ block-centroid) when
            // `with_contact_geom` is not set or the collider is the plane (byte-identical).
            if let (Some(g), Collider::Sphere { .. }) = (self.contact_geom, self.collider) {
                self.sphere_center_override = Some(self.data.geom_xpos[g]);
            }
            let height = self.tip_plane_height();
            let (_solver, x_next) = self.soft_resolve(height);
            for (vi, (&xf, &xo)) in self.v.iter_mut().zip(x_next.iter().zip(self.x.iter())) {
                *vi = (xf - xo) / self.cfg.dt;
            }
            self.x = x_next;
            // Route the full contact wrench [τ; f] at the COM (the off-COM moment
            // τ = Σ rᵢ × fᵢ, NOT a pure force at the COM), about the fresh-FK COM.
            self.data.xfrc_applied[self.body] = self.contact_wrench(height);
            self.data
                .step(&self.model)
                .expect("rigid step diverged in articulated rollout");
        }
        // Return the FRESH post-rollout tip height (forward at q_N — the same fresh
        // convention the gradient method's objective reads).
        self.data.forward(&self.model).expect("fresh FK (output)");
        self.data.xipos[self.body].z
    }
}
