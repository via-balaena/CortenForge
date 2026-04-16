//! [`ThermCircuitEnvBuilder`] and MJCF generation for thermodynamic circuits.

use std::fmt::Write as _;
use std::sync::Arc;

use sim_core::{DVector, Data, Model};
use sim_ml_chassis::{ActionSpace, ObservationSpace, SimEnv};
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
pub fn generate_mjcf(n_particles: usize, n_ctrl: usize, timestep: f64) -> String {
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
             ctrllimited="true" ctrlrange="0 10"/>"#
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
/// configure physics and RL parameters, then call [`.build()`](Self::build).
pub struct ThermCircuitEnvBuilder {
    n_particles: usize,
    timestep: f64,
    gamma: f64,
    k_b_t: f64,
    seed: u64,
    landscape: Vec<Arc<dyn PassiveComponent>>,
    ctrl_temperature: bool,
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
        if self.n_particles == 0 {
            return Err(ThermCircuitError::ZeroParticles);
        }
        let reward_fn = self
            .reward_fn
            .ok_or(ThermCircuitError::MissingField { field: "reward" })?;

        // 1. Determine ctrl layout
        let n_ctrl = usize::from(self.ctrl_temperature);

        // 2. Generate MJCF
        let xml = generate_mjcf(self.n_particles, n_ctrl, self.timestep);

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

        // 10. Build SimEnv
        let mut sim_builder = SimEnv::builder(Arc::clone(&model))
            .observation_space(obs_space)
            .action_space(act_space)
            .reward(reward_fn)
            .done(done_fn)
            .truncated(truncated_fn)
            .sub_steps(self.sub_steps);

        if let Some(on_reset) = self.on_reset_fn {
            sim_builder = sim_builder.on_reset(on_reset);
        }

        let inner = sim_builder.build()?;

        // 11. Return ThermCircuitEnv
        Ok(ThermCircuitEnv {
            inner,
            n_particles: self.n_particles,
            k_b_t: self.k_b_t,
            ctrl_temperature_idx: if self.ctrl_temperature { Some(0) } else { None },
        })
    }
}
