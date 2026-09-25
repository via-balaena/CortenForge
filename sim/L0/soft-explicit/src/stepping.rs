//! The stepping loop: the order of a step's phases, written once for every
//! executor, the stable time step, the monitors, and the generic validity
//! gates (plan §15c, §15f, §16e).

use crate::executor::{Executor, Monitors};

/// How the loop steps. [`StepperConfig::new`] gives the plan's values.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepperConfig {
    /// The penalty scale `s` the obstacle uses (plan §15c: 0.5).
    pub penalty_scale: f64,
    /// The largest friction coefficient in the run. Slipping friction's
    /// stiffness is not symmetric; its symmetric part bounds the step
    /// (plan §16e).
    pub friction: f64,
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
    /// Power iterations for the first estimate, from a cold start.
    pub first_iterations: usize,
    /// Power iterations for each warm re-estimate.
    pub warm_iterations: usize,
}

impl StepperConfig {
    /// The plan's values, for a run with penalty scale `penalty_scale`,
    /// largest friction `friction` and damping `damping`.
    #[must_use]
    pub const fn new(penalty_scale: f64, friction: f64, damping: f64) -> Self {
        Self {
            penalty_scale,
            friction,
            safety: 0.9,
            damping,
            reestimate_every: 500,
            growth_limit: 1.05,
            monitor_every: 100,
            first_iterations: 200,
            warm_iterations: 20,
        }
    }

    /// The stable step for an elastic `ω_el²`: `Δt = 0.9 · 2 / ω_max`, with
    /// the penalty added to `ω_el²` as a bound (plan §16e):
    ///
    /// `ω_max² ≤ ω_el² + s (1 + √(1 + μ_f²)) / 2 / Δt²`, so
    /// `Δt = √((2 · safety)² − s (1 + √(1 + μ_f²)) / 2) / ω_el`.
    ///
    /// # Panics
    /// If the penalty alone would use the whole stability budget.
    #[must_use]
    pub fn stable_step(&self, omega_squared: f64) -> f64 {
        let penalty = self.penalty_scale * 0.5 * (1.0 + self.friction.hypot(1.0));
        let budget = (2.0 * self.safety).powi(2) - penalty;
        assert!(
            budget > 0.0,
            "the penalty (s = {}, μ_f = {}) leaves no stability budget",
            self.penalty_scale,
            self.friction
        );
        (budget / omega_squared).sqrt()
    }
}

/// A step size the loop can use. A `NaN` or infinite estimate would stop time
/// from advancing, so `run_until` would never return; it fails here instead.
///
/// # Panics
/// If `dt` is not positive and finite.
fn checked_step(dt: f64) -> f64 {
    assert!(
        dt.is_finite() && dt > 0.0,
        "the stable step estimate is {dt}; the power iteration did not converge to a finite ω²"
    );
    dt
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

/// The loop over an executor.
#[derive(Debug)]
pub struct Stepper<E> {
    executor: E,
    config: StepperConfig,
    dt: f64,
    time: f64,
    steps: u64,
    omega_squared: f64,
    window: bool,
    samples: Vec<Sample>,
}

impl<E: Executor> Stepper<E> {
    /// Start a run at time `start`: estimate the stable step at rest.
    pub fn new(executor: E, config: StepperConfig, start: f64) -> Self {
        let mut stepper = Self {
            executor,
            config,
            dt: 0.0,
            time: start,
            steps: 0,
            omega_squared: 0.0,
            window: false,
            samples: Vec::new(),
        };
        stepper.omega_squared = stepper.estimate(config.first_iterations);
        stepper.dt = checked_step(config.stable_step(stepper.omega_squared));
        stepper
    }

    fn estimate(&mut self, iterations: usize) -> f64 {
        let perturbation = self.executor.epsilon().sqrt() * self.executor.shortest_edge();
        self.executor
            .elastic_rayleigh_quotient(iterations, perturbation)
    }

    /// One step: the phases in plan §15f's order, then the window sums and
    /// the monitors when due.
    pub fn step(&mut self) {
        if self.steps > 0 && self.steps.is_multiple_of(self.config.reestimate_every) {
            self.omega_squared = self.estimate(self.config.warm_iterations);
            let limit = checked_step(self.config.stable_step(self.omega_squared));
            self.dt = limit.min(self.dt * self.config.growth_limit);
        }
        let (dt, damping) = (self.dt, self.config.damping);
        let e = &mut self.executor;
        e.element_dilations();
        e.gather_volume_changes();
        e.nodal_pressures();
        e.element_forces();
        e.gather_forces();
        e.contact(self.time, dt);
        e.integrate(dt, damping);
        e.boundary_conditions(dt, damping);
        if self.window {
            e.accumulate();
        }
        self.time += dt;
        self.steps += 1;
        if self.steps.is_multiple_of(self.config.monitor_every) {
            self.samples.push(Sample {
                time: self.time,
                step: self.steps,
                dt,
                monitors: self.executor.monitors(),
            });
        }
    }

    /// Step until the time reaches `end`.
    pub fn run_until(&mut self, end: f64) {
        loop {
            if self.time >= end {
                break;
            }
            self.step();
        }
    }

    /// Start a measurement window: empty the sums, then add every step's
    /// contact forces to them until [`Stepper::close_window`].
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

    /// The executor, to read or snapshot.
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

    /// The monitor reads so far.
    #[must_use]
    pub fn samples(&self) -> &[Sample] {
        &self.samples
    }
}

/// The generic validity gates over a run's monitor reads (plan §15a as
/// amended, §16e).
pub mod gates {
    use super::Sample;

    /// Kinetic over internal energy on an interval, as the interval's mean
    /// kinetic energy over its mean internal energy; `None` if no read falls
    /// in `[from, to]` or the internal energy there is zero.
    ///
    /// A ratio per read would be 0/0 before first contact (plan §16e).
    #[must_use]
    pub fn kinetic_over_internal(samples: &[Sample], from: f64, to: f64) -> Option<f64> {
        let (kinetic, internal) = samples
            .iter()
            .filter(|s| (from..=to).contains(&s.time))
            .fold((0.0, 0.0), |(k, i), s| {
                (
                    k + s.monitors.kinetic_energy,
                    i + s.monitors.internal_energy,
                )
            });
        (internal > 0.0).then(|| kinetic / internal)
    }

    /// The energy balance's largest error over the run: `|W_contact −
    /// (U + K + D)|` at each read, over the run's peak internal energy `U`.
    /// `None` before any internal energy.
    #[must_use]
    pub fn energy_balance(samples: &[Sample]) -> Option<f64> {
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

    /// Whether any element reached `J ≤ 0` (K4).
    #[must_use]
    pub fn inverted(samples: &[Sample]) -> bool {
        samples
            .last()
            .is_some_and(|s| s.monitors.inverted_element_steps > 0)
    }
}
