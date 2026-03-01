//! Integration dispatch — Euler, implicit tendon K/D, and RK4.
//!
//! Corresponds to MuJoCo's `engine_forward.c` integration section:
//! `mj_Euler`, `mj_RungeKutta`, and implicit spring/damper helpers.
//!
//! - `euler`: Position integration on SO(3) manifold + quaternion normalization
//! - `implicit`: Tendon implicit stiffness/damping helpers (K/D accumulation)
//! - `rk4`: Standard 4-stage Runge-Kutta integration

pub(crate) mod euler;
pub(crate) mod implicit;
pub(crate) mod rk4;

use crate::forward::mj_next_activation;
use crate::types::flags::{actuator_disabled, disabled};
use crate::types::{DISABLE_ACTUATION, Data, ENABLE_SLEEP, Integrator, Model};

use euler::{mj_integrate_pos, mj_normalize_quat};

impl Data {
    /// Integration step for Euler and implicit-spring-damper integrators.
    ///
    /// This is exposed as part of the split-step API ([`step1`](Self::step1) /
    /// [`step2`](Self::step2)). RK4 integration is handled by
    /// `mj_runge_kutta()` and does not call this method.
    ///
    /// # Integration Methods
    ///
    /// - **Euler**: Semi-implicit Euler. Updates velocity first (`qvel += qacc * h`),
    ///   then integrates position using the new velocity.
    ///
    /// - **Implicit**: Velocity was already updated in `mj_fwd_acceleration_implicit()`.
    ///   We only integrate positions here.
    pub fn integrate(&mut self, model: &Model) {
        let h = model.timestep;
        let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
        // §16.27: Use indirection array for cache-friendly iteration over awake DOFs.
        let use_dof_ind = sleep_enabled && self.nv_awake < model.nv;

        // Integrate activation per actuator via mj_next_activation() (§34).
        // Handles both integration (Euler/FilterExact) and actlimited clamping.
        // MuJoCo order: activation → velocity → position.
        // S4.8: Per-actuator disable gating — disabled actuators get act_dot=0,
        // freezing activation state without zeroing it.
        for i in 0..model.nu {
            let act_adr = model.actuator_act_adr[i];
            let act_num = model.actuator_act_num[i];
            let is_disabled = disabled(model, DISABLE_ACTUATION) || actuator_disabled(model, i);
            for k in 0..act_num {
                let j = act_adr + k;
                let act_dot_val = if is_disabled { 0.0 } else { self.act_dot[j] };
                self.act[j] = mj_next_activation(model, i, self.act[j], act_dot_val);
            }
        }

        // For Euler and new implicit variants, update velocity using computed acceleration.
        // For legacy ImplicitSpringDamper, velocity was already updated in mj_fwd_acceleration_implicit.
        match model.integrator {
            Integrator::Euler | Integrator::ImplicitFast | Integrator::Implicit => {
                let nv = if use_dof_ind { self.nv_awake } else { model.nv };
                for idx in 0..nv {
                    let i = if use_dof_ind {
                        self.dof_awake_ind[idx]
                    } else {
                        idx
                    };
                    self.qvel[i] += self.qacc[i] * h;
                }
            }
            Integrator::ImplicitSpringDamper => {
                if self.newton_solved {
                    // Newton already computed qacc with implicit spring/damper effects
                    // baked into the constraint solve via M_impl (DT-35: includes
                    // tendon K/D coupling). Update velocity explicitly.
                    let nv = if use_dof_ind { self.nv_awake } else { model.nv };
                    for idx in 0..nv {
                        let i = if use_dof_ind {
                            self.dof_awake_ind[idx]
                        } else {
                            idx
                        };
                        self.qvel[i] += self.qacc[i] * h;
                    }
                }
                // Otherwise: velocity already updated by mj_fwd_acceleration_implicit
                // (non-Newton path solves for v_new directly, not qacc)
            }
            Integrator::RungeKutta4 => {
                // Fallback to Euler when called from step2() split-step API.
                // Full RK4 is handled by mj_runge_kutta() in step().
                let nv = if use_dof_ind { self.nv_awake } else { model.nv };
                for idx in 0..nv {
                    let i = if use_dof_ind {
                        self.dof_awake_ind[idx]
                    } else {
                        idx
                    };
                    self.qvel[i] += self.qacc[i] * h;
                }
            }
        }

        // Update positions - quaternions need special handling!
        mj_integrate_pos(model, self, h);

        // Normalize quaternions to prevent drift
        mj_normalize_quat(model, self);

        // Advance time
        self.time += h;
    }

    /// Integration step without velocity update.
    ///
    /// Performs activation integration, position integration, quaternion
    /// normalization, and time advance. Skips `qvel += qacc * h` (assumed
    /// to have been done externally, e.g., on GPU).
    ///
    /// Used by the GPU backend (`sim-gpu`) where velocity integration
    /// is performed on GPU via compute shader.
    ///
    /// # Visibility
    ///
    /// This method is public but feature-gated behind `gpu-internals`.
    /// It is not part of the stable `sim-core` API — only `sim-gpu`
    /// should depend on this feature.
    #[cfg(feature = "gpu-internals")]
    #[doc(hidden)]
    pub fn integrate_without_velocity(&mut self, model: &Model) {
        // Guard: this method assumes velocity was updated externally (GPU Euler).
        // If a future phase relaxes the Euler-only check in GpuBatchSim::new(),
        // this assert will catch misuse before silent physics errors.
        debug_assert!(
            matches!(model.integrator, Integrator::Euler),
            "integrate_without_velocity only valid for Euler integrator, got {:?}",
            model.integrator
        );
        let h = model.timestep;

        // 1. Activation integration via mj_next_activation() (§34, identical to integrate())
        // S4.8: Per-actuator disable gating (mirrors integrate() above).
        for i in 0..model.nu {
            let act_adr = model.actuator_act_adr[i];
            let act_num = model.actuator_act_num[i];
            let is_disabled = disabled(model, DISABLE_ACTUATION) || actuator_disabled(model, i);
            for k in 0..act_num {
                let j = act_adr + k;
                let act_dot_val = if is_disabled { 0.0 } else { self.act_dot[j] };
                self.act[j] = mj_next_activation(model, i, self.act[j], act_dot_val);
            }
        }

        // 2. Skip velocity integration (done on GPU)

        // 3. Position integration + quaternion normalization + time advance
        mj_integrate_pos(model, self, h);
        mj_normalize_quat(model, self);
        self.time += h;
    }
}
