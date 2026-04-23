//! [`ThermCircuitEnvBuilder`] and MJCF generation for thermodynamic circuits.

use std::fmt::Write as _;
use std::sync::Arc;

use sim_core::{DVector, Data, Model};
use sim_ml_chassis::{ActionSpace, ObservationSpace, SimEnv, VecEnv};
use sim_thermostat::{LangevinThermostat, PassiveComponent, PassiveStack};

use crate::env::ThermCircuitEnv;
use crate::error::ThermCircuitError;

// ─── MJCF generation ───────────────────────────────────────────────────────

/// Generate a minimal MJCF XML string for a thermodynamic circuit.
///
/// Produces `n_particles` bodies, each with a single slide joint (`x0`…`x{N-1}`),
/// and `n_ctrl` zero-gain actuators (`ctrl_0`…`ctrl_{M-1}`).  Zero-gain
/// actuators exist only to allocate `data.ctrl` slots — they produce zero
/// force regardless of the control input.
#[must_use]
pub fn generate_mjcf(
    n_particles: usize,
    n_ctrl: usize,
    timestep: f64,
    ctrl_range: (f64, f64),
) -> String {
    let mut xml = format!(
        r#"<mujoco model="therm-circuit-{n_particles}">
  <option timestep="{timestep}" gravity="0 0 0" integrator="Euler">
    <flag contact="disable"/>
  </option>
  <worldbody>"#
    );

    for i in 0..n_particles {
        write!(
            xml,
            r#"
    <body name="p{i}">
      <joint name="x{i}" type="slide" axis="1 0 0" damping="0"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>"#
        )
        .ok();
    }

    xml.push_str("\n  </worldbody>");

    if n_ctrl > 0 {
        xml.push_str("\n  <actuator>");
        for i in 0..n_ctrl {
            write!(
                xml,
                r#"
    <general name="ctrl_{i}" joint="x{i}" gainprm="0" biasprm="0 0 0"
             ctrllimited="true" ctrlrange="{lo} {hi}"/>"#,
                lo = ctrl_range.0,
                hi = ctrl_range.1
            )
            .ok();
        }
        xml.push_str("\n  </actuator>");
    }

    xml.push_str("\n</mujoco>\n");
    xml
}

// ─── Builder ───────────────────────────────────────────────────────────────

/// Builder for [`ThermCircuitEnv`].
///
/// Construct via [`ThermCircuitEnv::builder(n_particles)`](ThermCircuitEnv::builder),
/// configure physics and RL parameters, then call [`.build()`](Self::build)
/// or [`.build_vec(n_envs)`](Self::build_vec).
pub struct ThermCircuitEnvBuilder {
    n_particles: usize,
    timestep: f64,
    gamma: f64,
    k_b_t: f64,
    seed: u64,
    landscape: Vec<Arc<dyn PassiveComponent>>,
    ctrl_temperature: bool,
    ctrl_range: (f64, f64),
    sub_steps: usize,
    episode_steps: usize,
    reward_fn: Option<Box<dyn Fn(&Model, &Data) -> f64 + Send + Sync>>,
    done_fn: Option<Box<dyn Fn(&Model, &Data) -> bool + Send + Sync>>,
    truncated_fn: Option<Box<dyn Fn(&Model, &Data) -> bool + Send + Sync>>,
    on_reset_fn: Option<Box<dyn FnMut(&Model, &mut Data) + Send + Sync>>,
}

impl ThermCircuitEnvBuilder {
    /// Create a new builder for `n_particles` particles.
    pub(crate) fn new(n_particles: usize) -> Self {
        Self {
            n_particles,
            timestep: 0.001,
            gamma: 0.1,
            k_b_t: 1.0,
            seed: 0,
            landscape: Vec::new(),
            ctrl_temperature: false,
            ctrl_range: (0.0, 10.0),
            sub_steps: 1,
            episode_steps: 1000,
            reward_fn: None,
            done_fn: None,
            truncated_fn: None,
            on_reset_fn: None,
        }
    }

    // ── Physics ────────────────────────────────────────────────────────

    /// Set the simulation timestep (default: 0.001).
    #[must_use]
    pub const fn timestep(mut self, h: f64) -> Self {
        self.timestep = h;
        self
    }

    /// Set the damping coefficient for all particles (default: 0.1).
    #[must_use]
    pub const fn gamma(mut self, gamma: f64) -> Self {
        self.gamma = gamma;
        self
    }

    /// Set the base bath temperature (default: 1.0).
    #[must_use]
    pub const fn k_b_t(mut self, k_b_t: f64) -> Self {
        self.k_b_t = k_b_t;
        self
    }

    /// Set the master RNG seed (default: 0).
    #[must_use]
    pub const fn seed(mut self, seed: u64) -> Self {
        self.seed = seed;
        self
    }

    // ── Energy landscape ──────────────────────────────────────────────

    /// Add a passive component to the energy landscape.
    #[must_use]
    pub fn with(mut self, component: impl PassiveComponent) -> Self {
        self.landscape.push(Arc::new(component));
        self
    }

    // ── Ctrl channels ─────────────────────────────────────────────────

    /// Enable runtime temperature modulation via `ctrl[0]`.
    #[must_use]
    pub const fn with_ctrl_temperature(mut self) -> Self {
        self.ctrl_temperature = true;
        self
    }

    /// Set the MJCF `ctrlrange` for all actuators (default: `(0.0, 10.0)`).
    ///
    /// Controls the range of `data.ctrl` values that the physics engine accepts.
    /// With `LinearPolicy` (tanh output ∈ [-1, 1]), the effective range
    /// is approximately `[max(lo, -1), min(hi, 1)]`.  Set a tighter range
    /// (e.g., `(0.0, 5.0)`) when the policy needs to reach specific
    /// temperature multipliers.
    #[must_use]
    pub const fn ctrl_range(mut self, lo: f64, hi: f64) -> Self {
        self.ctrl_range = (lo, hi);
        self
    }

    // ── RL config ─────────────────────────────────────────────────────

    /// Set the number of physics sub-steps per RL step (default: 1).
    #[must_use]
    pub const fn sub_steps(mut self, n: usize) -> Self {
        self.sub_steps = n;
        self
    }

    /// Set the episode length in RL steps (default: 1000).
    #[must_use]
    pub const fn episode_steps(mut self, n: usize) -> Self {
        self.episode_steps = n;
        self
    }

    /// Set the reward function (**required**).
    #[must_use]
    pub fn reward(mut self, f: impl Fn(&Model, &Data) -> f64 + Send + Sync + 'static) -> Self {
        self.reward_fn = Some(Box::new(f));
        self
    }

    /// Set the terminal-state function (default: always false).
    #[must_use]
    pub fn done(mut self, f: impl Fn(&Model, &Data) -> bool + Send + Sync + 'static) -> Self {
        self.done_fn = Some(Box::new(f));
        self
    }

    /// Set the truncation function (default: time-based from `episode_steps`).
    #[must_use]
    pub fn truncated(mut self, f: impl Fn(&Model, &Data) -> bool + Send + Sync + 'static) -> Self {
        self.truncated_fn = Some(Box::new(f));
        self
    }

    /// Set the on-reset hook for domain randomization.
    #[must_use]
    pub fn on_reset(mut self, f: impl FnMut(&Model, &mut Data) + Send + Sync + 'static) -> Self {
        self.on_reset_fn = Some(Box::new(f));
        self
    }

    // ── Build ─────────────────────────────────────────────────────────

    /// Validate and build the [`ThermCircuitEnv`].
    ///
    /// # Errors
    ///
    /// Returns [`ThermCircuitError`] if `n_particles` is zero or `reward`
    /// was not set.
    pub fn build(self) -> Result<ThermCircuitEnv, ThermCircuitError> {
        let p = self.prepare()?;

        let mut sim_builder = SimEnv::builder(p.model)
            .observation_space(p.obs_space)
            .action_space(p.act_space)
            .reward(p.reward_fn)
            .done(p.done_fn)
            .truncated(p.truncated_fn)
            .sub_steps(p.sub_steps);

        if let Some(on_reset) = p.on_reset_fn {
            sim_builder = sim_builder.on_reset(on_reset);
        }

        let inner = sim_builder.build()?;

        Ok(ThermCircuitEnv {
            inner,
            n_particles: p.n_particles,
            k_b_t: p.k_b_t,
            ctrl_temperature_idx: p.ctrl_temperature_idx,
        })
    }

    /// Validate and build a [`VecEnv`] with `n_envs` parallel environments.
    ///
    /// Same physics/thermostat/space setup as [`build()`](Self::build), but
    /// produces a batched [`VecEnv`] instead of a single [`ThermCircuitEnv`].
    ///
    /// # Errors
    ///
    /// Returns [`ThermCircuitError`] if `n_particles` is zero or `reward`
    /// was not set.
    pub fn build_vec(self, n_envs: usize) -> Result<VecEnv, ThermCircuitError> {
        let p = self.prepare()?;

        let mut vec_builder = VecEnv::builder(p.model, n_envs)
            .observation_space(p.obs_space)
            .action_space(p.act_space)
            .reward(p.reward_fn)
            .done(p.done_fn)
            .truncated(p.truncated_fn)
            .sub_steps(p.sub_steps);

        if let Some(mut on_reset) = p.on_reset_fn {
            vec_builder = vec_builder.on_reset(move |m, d, _idx| on_reset(m, d));
        }

        Ok(vec_builder.build()?)
    }

    // ── Private ───────────────────────────────────────────────────────

    /// Reject `NaN` or ±Inf on any user-supplied f64 parameter.
    fn validate_finite_params(&self) -> Result<(), ThermCircuitError> {
        let checks: [(&'static str, f64); 5] = [
            ("timestep", self.timestep),
            ("gamma", self.gamma),
            ("k_b_t", self.k_b_t),
            ("ctrl_range.lo", self.ctrl_range.0),
            ("ctrl_range.hi", self.ctrl_range.1),
        ];
        for (field, value) in checks {
            if !value.is_finite() {
                return Err(ThermCircuitError::NonFiniteParameter { field, value });
            }
        }
        Ok(())
    }

    /// Validate config and execute shared setup (MJCF, model, thermostat,
    /// passive stack, obs/act spaces, closure defaults).
    fn prepare(self) -> Result<PreparedCircuit, ThermCircuitError> {
        self.validate_finite_params()?;
        if self.n_particles == 0 {
            return Err(ThermCircuitError::ZeroParticles);
        }
        let reward_fn = self
            .reward_fn
            .ok_or(ThermCircuitError::MissingField { field: "reward" })?;

        // 1. Determine ctrl layout
        let n_ctrl = usize::from(self.ctrl_temperature);

        // 2. Generate MJCF
        let xml = generate_mjcf(self.n_particles, n_ctrl, self.timestep, self.ctrl_range);

        // 3. Parse model
        let mut model = sim_mjcf::load_model(&xml)?;

        // 4. Build thermostat
        let gamma_vec = DVector::from_element(self.n_particles, self.gamma);
        let thermostat = LangevinThermostat::new(gamma_vec, self.k_b_t, self.seed, 0);
        let thermostat = if self.ctrl_temperature {
            thermostat.with_ctrl_temperature(0)
        } else {
            thermostat
        };

        // 5. Build passive stack: thermostat first, then landscape components
        let mut stack_builder = PassiveStack::builder().with(thermostat);
        for component in self.landscape {
            stack_builder = stack_builder.with_arc(component);
        }
        let stack = stack_builder.build();

        // 6. Install onto model
        stack.install(&mut model);

        // 7. Arc the model
        let model = Arc::new(model);

        // 8. Build obs/act spaces
        let obs_space = ObservationSpace::builder()
            .all_qpos()
            .all_qvel()
            .build(&model)?;
        let act_space = ActionSpace::builder().all_ctrl().build(&model)?;

        // 9. Defaults for done/truncated
        let done_fn: Box<dyn Fn(&Model, &Data) -> bool + Send + Sync> =
            self.done_fn.unwrap_or_else(|| Box::new(|_m, _d| false));

        // Both episode_steps and sub_steps are small (< 2^52), so f64 is exact.
        #[allow(clippy::cast_precision_loss)]
        let max_time = (self.episode_steps * self.sub_steps) as f64 * self.timestep;
        let truncated_fn: Box<dyn Fn(&Model, &Data) -> bool + Send + Sync> = self
            .truncated_fn
            .unwrap_or_else(|| Box::new(move |_m, d| d.time > max_time));

        Ok(PreparedCircuit {
            model,
            obs_space,
            act_space,
            reward_fn,
            done_fn,
            truncated_fn,
            on_reset_fn: self.on_reset_fn,
            n_particles: self.n_particles,
            k_b_t: self.k_b_t,
            ctrl_temperature_idx: if self.ctrl_temperature { Some(0) } else { None },
            sub_steps: self.sub_steps,
        })
    }
}

// ─── PreparedCircuit ──────────────────────────────────────────────────────

/// Intermediate state from shared validation + physics setup.
/// Consumed by [`ThermCircuitEnvBuilder::build`] and
/// [`ThermCircuitEnvBuilder::build_vec`].
struct PreparedCircuit {
    model: Arc<Model>,
    obs_space: ObservationSpace,
    act_space: ActionSpace,
    reward_fn: Box<dyn Fn(&Model, &Data) -> f64 + Send + Sync>,
    done_fn: Box<dyn Fn(&Model, &Data) -> bool + Send + Sync>,
    truncated_fn: Box<dyn Fn(&Model, &Data) -> bool + Send + Sync>,
    on_reset_fn: Option<Box<dyn FnMut(&Model, &mut Data) + Send + Sync>>,
    n_particles: usize,
    k_b_t: f64,
    ctrl_temperature_idx: Option<usize>,
    sub_steps: usize,
}

// ─── Tests ─────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    fn minimal_valid() -> ThermCircuitEnvBuilder {
        ThermCircuitEnvBuilder::new(1).reward(|_, _| 0.0)
    }

    #[test]
    fn timestep_nan_rejected() {
        let err = minimal_valid().timestep(f64::NAN).build().unwrap_err();
        assert!(matches!(
            err,
            ThermCircuitError::NonFiniteParameter { field: "timestep", value }
                if value.is_nan()
        ));
    }

    #[test]
    fn gamma_nan_rejected() {
        let err = minimal_valid().gamma(f64::NAN).build().unwrap_err();
        assert!(matches!(
            err,
            ThermCircuitError::NonFiniteParameter { field: "gamma", value }
                if value.is_nan()
        ));
    }

    #[test]
    fn k_b_t_nan_rejected() {
        let err = minimal_valid().k_b_t(f64::NAN).build().unwrap_err();
        assert!(matches!(
            err,
            ThermCircuitError::NonFiniteParameter { field: "k_b_t", value }
                if value.is_nan()
        ));
    }

    #[test]
    fn ctrl_range_lo_nan_rejected() {
        let err = minimal_valid()
            .ctrl_range(f64::NAN, 10.0)
            .build()
            .unwrap_err();
        assert!(matches!(
            err,
            ThermCircuitError::NonFiniteParameter { field: "ctrl_range.lo", value }
                if value.is_nan()
        ));
    }

    #[test]
    fn ctrl_range_hi_nan_rejected() {
        let err = minimal_valid()
            .ctrl_range(0.0, f64::NAN)
            .build()
            .unwrap_err();
        assert!(matches!(
            err,
            ThermCircuitError::NonFiniteParameter { field: "ctrl_range.hi", value }
                if value.is_nan()
        ));
    }

    #[test]
    fn timestep_pos_inf_rejected() {
        let err = minimal_valid().timestep(f64::INFINITY).build().unwrap_err();
        assert!(matches!(
            err,
            ThermCircuitError::NonFiniteParameter { field: "timestep", value }
                if value.is_infinite() && value.is_sign_positive()
        ));
    }

    #[test]
    fn gamma_neg_inf_rejected() {
        let err = minimal_valid()
            .gamma(f64::NEG_INFINITY)
            .build()
            .unwrap_err();
        assert!(matches!(
            err,
            ThermCircuitError::NonFiniteParameter { field: "gamma", value }
                if value.is_infinite() && value.is_sign_negative()
        ));
    }

    #[test]
    fn minimal_valid_still_builds() {
        minimal_valid().build().unwrap();
    }

    #[test]
    fn build_vec_timestep_nan_rejected() {
        let err = minimal_valid().timestep(f64::NAN).build_vec(1).unwrap_err();
        assert!(matches!(
            err,
            ThermCircuitError::NonFiniteParameter { field: "timestep", value }
                if value.is_nan()
        ));
    }
}
