//! The stepping loop, written once for every executor (plan §15c, §15f,
//! §16e).
//!
//! It holds the order of a step's phases, the stable time step, the
//! monitors, the stop on a non-finite read, and the generic validity gates.

use crate::executor::{Executor, Monitors};

/// How the loop steps. [`StepperConfig::new`] gives the plan's values.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepperConfig {
    /// The fraction of the stability limit the step uses (plan §15c: 0.9).
    pub safety: f64,
    /// Mass-proportional damping `α` (plan §15c).
    pub damping: f64,
    /// Steps between re-estimates of the stable step (plan §15c: 500).
    pub reestimate_every: u64,
    /// The largest factor the step may grow by at a re-estimate (1.05).
    pub growth_limit: f64,
    /// Steps between monitor reads (plan §16e: 100).
    pub monitor_every: u64,
    /// Power iterations per estimate, each from the same fixed start.
    /// `tube_release` checks this count on the 10k tube, at rest and loaded,
    /// against a converged estimate (plan §16m).
    pub power_iterations: usize,
}

impl StepperConfig {
    /// The plan's values, with mass damping `damping`.
    #[must_use]
    pub const fn new(damping: f64) -> Self {
        Self {
            safety: 0.9,
            damping,
            reestimate_every: 500,
            growth_limit: 1.05,
            monitor_every: 100,
            power_iterations: 100,
        }
    }

    /// The stable step for an elastic `ω_el²`: `Δt = safety · 2 / ω_el`. The
    /// kinematic contact law adds nothing to it (plan §16o).
    #[must_use]
    pub fn stable_step(&self, omega_squared: f64) -> f64 {
        2.0 * self.safety / omega_squared.sqrt()
    }

    /// The step after a re-estimate: the new limit when it is smaller (the
    /// step shrinks at once), and at most `growth_limit` times the current
    /// step when it is larger (plan §15c).
    #[must_use]
    pub fn next_step(&self, current: f64, limit: f64) -> f64 {
        limit.min(current * self.growth_limit)
    }
}

/// One monitor read, with when it was taken.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    /// The time after the step the read followed.
    pub time: f64,
    /// Steps taken so far.
    pub step: u64,
    /// The step size in use.
    pub dt: f64,
    /// What was read.
    pub monitors: Monitors,
}

/// Why a run stopped early.
#[derive(Clone, Copy, Debug, PartialEq, thiserror::Error)]
pub enum RunError {
    /// A monitor read, or a re-estimate of the stable step, was not finite:
    /// the run blew up. An explicit check on the host, since a shader may not
    /// propagate `NaN` (plan §13d rule 2).
    #[error("a monitor read or step estimate at step {step} (time {time}) is not finite")]
    NonFinite {
        /// The step of the read.
        step: u64,
        /// Its time.
        time: f64,
    },
}

/// The loop over an executor.
#[derive(Debug)]
pub struct Stepper<E> {
    executor: E,
    config: StepperConfig,
    dt: f64,
    time: f64,
    steps: u64,
    omega_squared: f64,
    estimates: u64,
    window: bool,
    samples: Vec<Sample>,
}

impl<E: Executor> Stepper<E> {
    /// Start a run at time `start`: estimate the stable step.
    ///
    /// # Panics
    /// If the estimate at the start is not a positive, finite step: the
    /// model itself is broken.
    pub fn new(executor: E, config: StepperConfig, start: f64) -> Self {
        let mut stepper = Self {
            executor,
            config,
            dt: 0.0,
            time: start,
            steps: 0,
            omega_squared: 0.0,
            estimates: 0,
            window: false,
            samples: Vec::new(),
        };
        stepper.dt = stepper.estimate().unwrap_or(f64::NAN);
        assert!(
            stepper.dt > 0.0,
            "the stable step at the start is not positive and finite; ω² = {}",
            stepper.omega_squared
        );
        stepper
    }

    /// Estimate `ω_el²` from the fixed start and return the stable step, or
    /// `None` if it is not positive and finite.
    fn estimate(&mut self) -> Option<f64> {
        let perturbation = self.executor.epsilon().sqrt() * self.executor.shortest_edge();
        self.omega_squared = self
            .executor
            .elastic_rayleigh_quotient(self.config.power_iterations, perturbation);
        self.estimates += 1;
        let dt = self.config.stable_step(self.omega_squared);
        (dt.is_finite() && dt > 0.0).then_some(dt)
    }

    /// One step: the phases in plan §15f's order, then the window sums and
    /// the monitors when due.
    ///
    /// # Errors
    /// [`RunError::NonFinite`] if a monitor read, or a re-estimate of the
    /// stable step, is not finite.
    pub fn step(&mut self) -> Result<(), RunError> {
        if self.steps > 0 && self.steps.is_multiple_of(self.config.reestimate_every) {
            let limit = self.estimate().ok_or(RunError::NonFinite {
                step: self.steps,
                time: self.time,
            })?;
            self.dt = self.config.next_step(self.dt, limit);
        }
        let (dt, damping) = (self.dt, self.config.damping);
        let e = &mut self.executor;
        e.element_dilations();
        e.gather_volume_changes();
        e.nodal_pressures();
        e.element_forces();
        e.gather_forces();
        e.contact(self.time, dt, damping);
        e.integrate(dt, damping);
        e.boundary_conditions(dt, damping);
        if self.window {
            e.accumulate();
        }
        self.time += dt;
        self.steps += 1;
        if self.steps.is_multiple_of(self.config.monitor_every) {
            self.read()?;
        }
        Ok(())
    }

    /// Read the monitors now, and stop if a value is not finite.
    fn read(&mut self) -> Result<(), RunError> {
        let monitors = self.executor.monitors();
        self.samples.push(Sample {
            time: self.time,
            step: self.steps,
            dt: self.dt,
            monitors,
        });
        if monitors.finite() {
            Ok(())
        } else {
            Err(RunError::NonFinite {
                step: self.steps,
                time: self.time,
            })
        }
    }

    /// Step until the time reaches `end`, then read the monitors once more
    /// if the last step was not read, so the reads cover every step.
    ///
    /// # Errors
    /// [`RunError::NonFinite`] if a monitor read, or a re-estimate of the
    /// stable step, is not finite.
    pub fn run_until(&mut self, end: f64) -> Result<(), RunError> {
        loop {
            if self.time >= end {
                break;
            }
            self.step()?;
        }
        if self.samples.last().is_none_or(|s| s.step != self.steps) {
            self.read()?;
        }
        Ok(())
    }

    /// Start a measurement window: empty the sums, then add every step's
    /// displacements and contact forces to them until
    /// [`Stepper::close_window`].
    pub fn open_window(&mut self) {
        self.executor.clear_accumulators();
        self.window = true;
    }

    /// Stop adding to the window sums.
    pub const fn close_window(&mut self) {
        self.window = false;
    }

    /// The executor, to read.
    pub const fn executor(&self) -> &E {
        &self.executor
    }

    /// The executor, to snapshot or to change its pose track.
    pub const fn executor_mut(&mut self) -> &mut E {
        &mut self.executor
    }

    /// The step size in use.
    #[must_use]
    pub const fn dt(&self) -> f64 {
        self.dt
    }

    /// The current time.
    #[must_use]
    pub const fn time(&self) -> f64 {
        self.time
    }

    /// Steps taken.
    #[must_use]
    pub const fn steps(&self) -> u64 {
        self.steps
    }

    /// The latest estimate of `ω_el²`.
    #[must_use]
    pub const fn omega_squared(&self) -> f64 {
        self.omega_squared
    }

    /// How many times the stable step has been estimated.
    #[must_use]
    pub const fn estimates(&self) -> u64 {
        self.estimates
    }

    /// The monitor reads so far.
    #[must_use]
    pub fn samples(&self) -> &[Sample] {
        &self.samples
    }
}

/// The generic validity gates over a run's monitor reads (plan §15a as
/// amended, §16e). Each refuses to judge (`None`) when a read it covers is
/// not finite.
pub mod gates {
    use super::Sample;

    /// Kinetic over internal energy on an interval: the interval's mean
    /// kinetic energy over its mean internal energy.
    ///
    /// `None` if no read falls in `[from, to]`, a read there is not finite,
    /// or the internal energy there is zero. A ratio per read would be 0/0
    /// before first contact (plan §16e).
    #[must_use]
    pub fn kinetic_over_internal(samples: &[Sample], from: f64, to: f64) -> Option<f64> {
        let inside: Vec<&Sample> = samples
            .iter()
            .filter(|s| (from..=to).contains(&s.time))
            .collect();
        if inside.iter().any(|s| !s.monitors.finite()) {
            return None;
        }
        let (kinetic, internal) = inside.iter().fold((0.0, 0.0), |(k, i), s| {
            (
                k + s.monitors.kinetic_energy,
                i + s.monitors.internal_energy,
            )
        });
        (internal > 0.0).then(|| kinetic / internal)
    }

    /// The energy balance's largest error over the run: `|W_contact −
    /// (U + K + D)|` at each read, over the run's peak internal energy `U`.
    /// `None` if a read is not finite, or before any internal energy.
    #[must_use]
    pub fn energy_balance(samples: &[Sample]) -> Option<f64> {
        if samples.iter().any(|s| !s.monitors.finite()) {
            return None;
        }
        let peak = samples
            .iter()
            .map(|s| s.monitors.internal_energy)
            .fold(0.0, f64::max);
        (peak > 0.0).then(|| {
            samples
                .iter()
                .map(|s| {
                    let m = s.monitors;
                    (m.contact_work - (m.internal_energy + m.kinetic_energy + m.damping_loss)).abs()
                })
                .fold(0.0, f64::max)
                / peak
        })
    }

    /// Whether any element reached `J ≤ 0` (K4), from the last read, which
    /// [`super::Stepper::run_until`] takes after the last step.
    #[must_use]
    pub fn inverted(samples: &[Sample]) -> bool {
        samples
            .last()
            .is_some_and(|s| s.monitors.inverted_element_steps > 0)
    }
}
