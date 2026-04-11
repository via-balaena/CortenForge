//! Sinusoidal external force for stochastic resonance experiments.
//!
//! Implements the time-dependent force `F(t) = A₀ cos(ωt + φ₀)` as a
//! [`PassiveComponent`] that contributes a periodic driving force to the
//! `qfrc_passive` accumulator. Combined with a [`DoubleWellPotential`] and
//! a [`LangevinThermostat`] in a [`PassiveStack`], this provides the
//! sub-threshold periodic signal required for stochastic resonance (SR)
//! experiments.
//!
//! The signal enters as a force (not a potential modulation). It tilts the
//! double-well energy landscape periodically, lowering one barrier and
//! raising the other. When the signal amplitude is below the deterministic
//! switching threshold (`A₀ ≪ F_c`), switching between wells requires
//! thermal noise — and the SR peak occurs at the noise level where
//! noise-assisted switching synchronizes with the signal.
//!
//! D2 of the thermodynamic computing initiative validates this component
//! in combination with a [`DoubleWellPotential`], a [`LangevinThermostat`]
//! (with ctrl-temperature modulation), and an RL agent (CEM) that discovers
//! the SR-optimal noise level.
//!
//! [`PassiveComponent`]: crate::PassiveComponent
//! [`DoubleWellPotential`]: crate::DoubleWellPotential
//! [`LangevinThermostat`]: crate::LangevinThermostat
//! [`PassiveStack`]: crate::PassiveStack

use std::f64::consts::PI;

use sim_core::{DVector, Data, Model};

use crate::component::PassiveComponent;
use crate::diagnose::Diagnose;

/// Sinusoidal driving force: `F(t) = A₀ cos(ωt + φ₀)`.
///
/// Applies a time-dependent force to a single DOF by reading `data.time`
/// at each physics step. This is a deterministic force — it does not
/// implement [`Stochastic`](crate::Stochastic) and is unaffected by the
/// stochastic gating mechanism (Decision 7).
///
/// # Joint type constraint
///
/// Same as [`DoubleWellPotential`](crate::DoubleWellPotential): the `dof`
/// field indexes both `data.qpos` and `qfrc_out`. This is correct for
/// slide and hinge joints where `nq = nv = 1`. It does **not** support
/// ball or free joints.
///
/// # Signal phase and `data.time`
///
/// The signal phase is `ωt + φ₀` where `t = data.time`. On `Data::reset()`,
/// `data.time` resets to 0.0, so the signal restarts at phase `φ₀` every
/// episode. All episodes see the same signal trajectory — this is a
/// feature for CEM training (removes phase randomness across episodes).
pub struct OscillatingField {
    /// Signal amplitude A₀.
    amplitude: f64,
    /// Angular frequency ω (rad / time unit).
    omega: f64,
    /// Initial phase offset φ₀ (radians).
    phase: f64,
    /// DOF index (= qpos index for slide/hinge joints).
    dof: usize,
}

impl OscillatingField {
    /// Create a new oscillating field.
    ///
    /// # Parameters
    /// - `amplitude`: signal amplitude `A₀ ≥ 0`
    /// - `omega`: angular frequency `ω > 0` (rad / time unit)
    /// - `phase`: initial phase offset `φ₀` (radians)
    /// - `dof`: DOF index (must be valid for the target model)
    ///
    /// # Panics
    /// Panics if `amplitude < 0` or `omega <= 0`.
    #[must_use]
    pub fn new(amplitude: f64, omega: f64, phase: f64, dof: usize) -> Self {
        assert!(
            amplitude >= 0.0,
            "amplitude must be non-negative, got {amplitude}"
        );
        assert!(omega > 0.0, "omega must be positive, got {omega}");
        Self {
            amplitude,
            omega,
            phase,
            dof,
        }
    }

    /// Signal amplitude A₀.
    #[must_use]
    pub const fn amplitude(&self) -> f64 {
        self.amplitude
    }

    /// Angular frequency ω (rad / time unit).
    #[must_use]
    pub const fn omega(&self) -> f64 {
        self.omega
    }

    /// Initial phase offset φ₀ (radians).
    #[must_use]
    pub const fn phase(&self) -> f64 {
        self.phase
    }

    /// Signal period T = 2π/ω.
    #[must_use]
    pub fn period(&self) -> f64 {
        2.0 * PI / self.omega
    }

    /// Signal frequency f = ω/(2π).
    #[must_use]
    pub fn frequency(&self) -> f64 {
        self.omega / (2.0 * PI)
    }

    /// Signal value at time `t`: `A₀ · cos(ωt + φ₀)`.
    ///
    /// Exposed for reward computation and diagnostics — the reward closure
    /// needs the signal value to compute synchrony.
    #[must_use]
    pub fn signal_value(&self, t: f64) -> f64 {
        self.amplitude * self.omega.mul_add(t, self.phase).cos()
    }

    /// Normalized signal at time `t`: `cos(ωt + φ₀)` (unit amplitude).
    ///
    /// For synchrony reward: `reward = sign(x) · normalized_signal(t)`.
    #[must_use]
    pub fn normalized_signal(&self, t: f64) -> f64 {
        self.omega.mul_add(t, self.phase).cos()
    }
}

impl PassiveComponent for OscillatingField {
    fn apply(&self, _model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
        qfrc_out[self.dof] += self.signal_value(data.time);
    }
}

impl Diagnose for OscillatingField {
    fn diagnostic_summary(&self) -> String {
        format!(
            "OscillatingField(A\u{2080}={:.4}, \u{03c9}={:.4}, \u{03c6}\u{2080}={:.4}, T={:.4}, dof={})",
            self.amplitude,
            self.omega,
            self.phase,
            self.period(),
            self.dof,
        )
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::suboptimal_flops,
    clippy::cast_lossless
)]
mod tests {
    use super::*;

    // ── construction ────────────────────────────────────────────────────

    #[test]
    fn new_accepts_valid_params() {
        let f = OscillatingField::new(0.3, 0.1, 0.0, 0);
        assert_eq!(f.amplitude(), 0.3);
        assert_eq!(f.omega(), 0.1);
        assert_eq!(f.phase(), 0.0);
    }

    #[test]
    fn new_accepts_zero_amplitude() {
        let f = OscillatingField::new(0.0, 1.0, 0.0, 0);
        assert_eq!(f.amplitude(), 0.0);
    }

    #[test]
    #[should_panic(expected = "amplitude must be non-negative")]
    fn new_rejects_negative_amplitude() {
        #[allow(clippy::let_underscore_must_use)]
        let _ = OscillatingField::new(-1.0, 1.0, 0.0, 0);
    }

    #[test]
    #[should_panic(expected = "omega must be positive")]
    fn new_rejects_zero_omega() {
        #[allow(clippy::let_underscore_must_use)]
        let _ = OscillatingField::new(1.0, 0.0, 0.0, 0);
    }

    #[test]
    #[should_panic(expected = "omega must be positive")]
    fn new_rejects_negative_omega() {
        #[allow(clippy::let_underscore_must_use)]
        let _ = OscillatingField::new(1.0, -1.0, 0.0, 0);
    }

    // ── signal value correctness ────────────────────────────────────────

    #[test]
    fn signal_value_at_t_zero_with_zero_phase() {
        let f = OscillatingField::new(2.0, 1.0, 0.0, 0);
        // F(0) = 2.0 * cos(0) = 2.0
        assert!((f.signal_value(0.0) - 2.0).abs() < 1e-15);
    }

    #[test]
    fn signal_value_at_t_zero_with_phase() {
        let f = OscillatingField::new(1.0, 1.0, PI / 2.0, 0);
        // F(0) = 1.0 * cos(π/2) = 0.0
        assert!(f.signal_value(0.0).abs() < 1e-15);
    }

    #[test]
    fn signal_value_at_various_times() {
        let omega = 2.0 * PI; // period = 1.0
        let f = OscillatingField::new(1.5, omega, 0.0, 0);

        // t = 0: cos(0) = 1
        assert!((f.signal_value(0.0) - 1.5).abs() < 1e-12);
        // t = 0.25: cos(π/2) = 0
        assert!(f.signal_value(0.25).abs() < 1e-12);
        // t = 0.5: cos(π) = -1
        assert!((f.signal_value(0.5) - (-1.5)).abs() < 1e-12);
        // t = 1.0: cos(2π) = 1
        assert!((f.signal_value(1.0) - 1.5).abs() < 1e-12);
    }

    // ── signal periodicity ──────────────────────────────────────────────

    #[test]
    fn signal_is_periodic() {
        let f = OscillatingField::new(0.3, 0.15, 0.5, 0);
        let t_period = f.period();

        for &t in &[0.0, 1.7, 5.3, 17.9] {
            let v0 = f.signal_value(t);
            let v1 = f.signal_value(t + t_period);
            let v2 = f.signal_value(t + 5.0 * t_period);
            assert!(
                (v0 - v1).abs() < 1e-10,
                "signal_value({t}) = {v0} ≠ signal_value({}) = {v1}",
                t + t_period,
            );
            assert!(
                (v0 - v2).abs() < 1e-10,
                "signal_value({t}) = {v0} ≠ signal_value({}) = {v2}",
                t + 5.0 * t_period,
            );
        }
    }

    // ── zero force when amplitude zero ──────────────────────────────────

    #[test]
    fn zero_force_when_amplitude_zero() {
        let f = OscillatingField::new(0.0, 1.0, 0.0, 0);
        for &t in &[0.0, 0.5, 1.0, 7.3] {
            assert_eq!(
                f.signal_value(t),
                0.0,
                "signal should be exactly zero when A₀=0, got {} at t={t}",
                f.signal_value(t),
            );
        }
    }

    // ── normalized signal ───────────────────────────────────────────────

    #[test]
    fn normalized_signal_is_unit_amplitude() {
        let f = OscillatingField::new(5.0, 0.3, 0.7, 0);
        // Sample many time points
        for i in 0..100 {
            let t = i as f64 * 0.37;
            let ns = f.normalized_signal(t);
            assert!(
                ns.abs() <= 1.0 + 1e-15,
                "normalized_signal({t}) = {ns}, expected |ns| ≤ 1",
            );
        }
    }

    #[test]
    fn normalized_signal_matches_signal_value_divided_by_amplitude() {
        let f = OscillatingField::new(3.0, 0.5, 0.2, 0);
        for &t in &[0.0, 1.0, 2.5, 7.0] {
            let ns = f.normalized_signal(t);
            let sv = f.signal_value(t) / f.amplitude();
            assert!(
                (ns - sv).abs() < 1e-15,
                "normalized_signal({t}) = {ns} ≠ signal_value/A₀ = {sv}",
            );
        }
    }

    // ── period / frequency ──────────────────────────────────────────────

    #[test]
    fn period_and_frequency_consistent() {
        let f = OscillatingField::new(1.0, 2.0 * PI, 0.0, 0);
        assert!((f.period() - 1.0).abs() < 1e-15);
        assert!((f.frequency() - 1.0).abs() < 1e-15);

        let f2 = OscillatingField::new(1.0, PI, 0.0, 0);
        assert!((f2.period() - 2.0).abs() < 1e-15);
        assert!((f2.frequency() - 0.5).abs() < 1e-15);
    }

    // ── diagnostic summary ──────────────────────────────────────────────

    #[test]
    fn diagnostic_summary_format() {
        let f = OscillatingField::new(0.3, 0.0763, 0.0, 0);
        let s = f.diagnostic_summary();
        assert!(s.contains("OscillatingField"));
        assert!(s.contains("0.3000"));
        assert!(s.contains("0.0763"));
        assert!(s.contains("dof=0"));
    }
}
