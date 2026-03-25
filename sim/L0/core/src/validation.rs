//! Validation framework for physics examples and integration tests.
//!
//! Provides composable tracker structs that accumulate measurements over time
//! and produce pass/fail checks. Zero Bevy dependency — pure data structs
//! with `println!` formatting.
//!
//! # Usage
//!
//! ```rust,ignore
//! use sim_core::validation::{PeriodTracker, EnergyMonotonicityTracker, LimitTracker, EquilibriumTracker, print_report};
//!
//! let mut period = PeriodTracker::new();
//! let mut energy = EnergyMonotonicityTracker::new();
//! let mut limits = LimitTracker::new();
//!
//! // In your per-frame loop:
//! period.sample(position, sim_time);
//! energy.sample(total_energy);
//! limits.sample(position, lo, hi);
//!
//! // At validation time:
//! let checks = vec![
//!     period.check(expected_period, 2.0),
//!     energy.check(1e-6),
//!     limits.check(0.001),
//! ];
//! let all_ok = print_report("My Example (t=15s)", &checks);
//! ```

/// A single validation check result.
///
/// Each tracker pre-formats its own `detail` string because check types
/// have fundamentally different shapes (period has measured/expected/error,
/// energy has just max_increase, limits has just max_violation).
///
/// Raw values are accessible through each tracker's own accessor methods.
#[derive(Debug, Clone)]
pub struct Check {
    /// Short label for this check (e.g. "Period", "Energy", "Limits").
    pub name: &'static str,
    /// Whether the check passed.
    pub pass: bool,
    /// Pre-formatted detail string (units, values, thresholds).
    pub detail: String,
}

// ── Period Tracker ──────────────────────────────────────────────────────────

/// Tracks zero crossings of a scalar signal for period measurement.
///
/// Uses linear interpolation between consecutive samples for sub-frame
/// precision — this gives ~100x better accuracy than raw frame-rate sampling.
///
/// Detects positive-to-negative zero crossings only (one per full period
/// for an oscillating signal).
#[derive(Debug, Clone)]
pub struct PeriodTracker {
    last_value: f64,
    last_time: f64,
    initialized: bool,
    crossings: Vec<f64>,
}

impl PeriodTracker {
    /// Create a new period tracker with no recorded crossings.
    #[must_use]
    pub fn new() -> Self {
        Self {
            last_value: 0.0,
            last_time: 0.0,
            initialized: false,
            crossings: Vec::new(),
        }
    }

    /// Feed a new sample. Call once per frame with the current value and time.
    ///
    /// Gracefully handles edge cases:
    /// - Time going backwards (sim reset): sample is skipped.
    /// - First call: stores initial state, no crossing possible.
    pub fn sample(&mut self, value: f64, time: f64) {
        if !self.initialized {
            self.last_value = value;
            self.last_time = time;
            self.initialized = true;
            return;
        }

        // Skip if time went backwards (sim reset)
        if time <= self.last_time {
            return;
        }

        // Detect positive → negative zero crossing
        if self.last_value > 0.0 && value <= 0.0 {
            // Linear interpolation for sub-frame precision
            let denom = self.last_value - value;
            if denom.abs() > 1e-15 {
                let frac = self.last_value / denom;
                let t_cross = self.last_time + frac * (time - self.last_time);
                self.crossings.push(t_cross);
            }
        }

        self.last_value = value;
        self.last_time = time;
    }

    /// Compute the average period from recorded zero crossings.
    /// Returns `None` if fewer than 2 crossings recorded.
    #[must_use]
    pub fn measured_period(&self) -> Option<f64> {
        if self.crossings.len() < 2 {
            return None;
        }
        let periods: Vec<f64> = self.crossings.windows(2).map(|w| w[1] - w[0]).collect();
        let sum: f64 = periods.iter().sum();
        #[allow(clippy::cast_precision_loss)]
        Some(sum / periods.len() as f64)
    }

    /// Number of zero crossings recorded so far.
    #[must_use]
    pub fn crossing_count(&self) -> usize {
        self.crossings.len()
    }

    /// Check measured period against expected, with a percentage threshold.
    ///
    /// Returns a failing check with descriptive detail if fewer than 2
    /// crossings were recorded.
    #[must_use]
    pub fn check(&self, expected: f64, threshold_pct: f64) -> Check {
        match self.measured_period() {
            Some(measured) => {
                let err_pct = ((measured - expected) / expected).abs() * 100.0;
                Check {
                    name: "Period",
                    pass: err_pct < threshold_pct,
                    detail: format!(
                        "measured={measured:.4}s  expected={expected:.4}s  error={err_pct:.2}%"
                    ),
                }
            }
            None => Check {
                name: "Period",
                pass: false,
                detail: format!(
                    "insufficient zero crossings ({} recorded, need >= 2)",
                    self.crossings.len()
                ),
            },
        }
    }
}

impl Default for PeriodTracker {
    fn default() -> Self {
        Self::new()
    }
}

// ── Energy Monotonicity Tracker ─────────────────────────────────────────────

/// Tracks energy to verify monotonic decrease (for damped systems).
///
/// Records the largest single-frame energy increase. In a correctly damped
/// system, energy should never increase — any increase indicates a bug
/// (spurious energy injection).
#[derive(Debug, Clone)]
pub struct EnergyMonotonicityTracker {
    prev: Option<f64>,
    max_increase: f64,
}

impl EnergyMonotonicityTracker {
    /// Create a new energy tracker with no recorded samples.
    #[must_use]
    pub fn new() -> Self {
        Self {
            prev: None,
            max_increase: 0.0,
        }
    }

    /// Feed a new energy sample. Call once per frame.
    pub fn sample(&mut self, energy: f64) {
        if let Some(prev) = self.prev {
            let increase = energy - prev;
            if increase > self.max_increase {
                self.max_increase = increase;
            }
        }
        self.prev = Some(energy);
    }

    /// The largest single-frame energy increase observed.
    #[must_use]
    pub fn max_increase(&self) -> f64 {
        self.max_increase
    }

    /// Check that energy never increased by more than `threshold` (in Joules).
    ///
    /// Returns a passing check if no samples were recorded (vacuously true).
    #[must_use]
    pub fn check(&self, threshold: f64) -> Check {
        Check {
            name: "Energy",
            pass: self.max_increase < threshold,
            detail: format!("max increase={:.2e}J", self.max_increase),
        }
    }
}

impl Default for EnergyMonotonicityTracker {
    fn default() -> Self {
        Self::new()
    }
}

// ── Limit Tracker ───────────────────────────────────────────────────────────

/// Tracks joint or tendon limit violations over time.
///
/// Records the largest violation (overshoot past the allowed range).
/// A well-behaved constraint solver should keep violations below the
/// solver tolerance.
#[derive(Debug, Clone)]
pub struct LimitTracker {
    max_violation: f64,
}

impl LimitTracker {
    /// Create a new limit tracker with no recorded violations.
    #[must_use]
    pub fn new() -> Self {
        Self { max_violation: 0.0 }
    }

    /// Feed a new position sample with the allowed range `[lo, hi]`.
    pub fn sample(&mut self, value: f64, lo: f64, hi: f64) {
        let violation = (value - hi).max(lo - value).max(0.0);
        if violation > self.max_violation {
            self.max_violation = violation;
        }
    }

    /// The largest violation observed (in meters or radians).
    #[must_use]
    pub fn max_violation(&self) -> f64 {
        self.max_violation
    }

    /// Check that violations never exceeded `threshold`.
    ///
    /// Returns a passing check if no samples were recorded (vacuously true).
    #[must_use]
    pub fn check(&self, threshold: f64) -> Check {
        Check {
            name: "Limits",
            pass: self.max_violation < threshold,
            detail: format!("max violation={:.6}m", self.max_violation),
        }
    }
}

impl Default for LimitTracker {
    fn default() -> Self {
        Self::new()
    }
}

// ── Quaternion Norm Tracker ─────────────────────────────────────────────────

/// Tracks quaternion norm deviation from 1.0 over time.
///
/// Ball and free joints use unit quaternions for orientation. Numerical
/// integration should preserve `‖q‖ = 1` to machine precision. Any
/// significant drift indicates an integration bug.
#[derive(Debug, Clone)]
pub struct QuaternionNormTracker {
    max_deviation: f64,
}

impl QuaternionNormTracker {
    /// Create a new quaternion norm tracker with no recorded samples.
    #[must_use]
    pub fn new() -> Self {
        Self { max_deviation: 0.0 }
    }

    /// Feed a quaternion sample (w, x, y, z). Call once per frame per joint.
    pub fn sample(&mut self, w: f64, x: f64, y: f64, z: f64) {
        let norm = (w * w + x * x + y * y + z * z).sqrt();
        let deviation = (norm - 1.0).abs();
        if deviation > self.max_deviation {
            self.max_deviation = deviation;
        }
    }

    /// The largest deviation of `‖q‖` from 1.0 observed.
    #[must_use]
    pub fn max_deviation(&self) -> f64 {
        self.max_deviation
    }

    /// Check that quaternion norm never deviated by more than `threshold`.
    ///
    /// Returns a passing check if no samples were recorded (vacuously true).
    #[must_use]
    pub fn check(&self, threshold: f64) -> Check {
        Check {
            name: "Quat norm",
            pass: self.max_deviation < threshold,
            detail: format!("max deviation={:.2e}", self.max_deviation),
        }
    }
}

impl Default for QuaternionNormTracker {
    fn default() -> Self {
        Self::new()
    }
}

// ── Energy Conservation Tracker ────────────────────────────────────────────

/// Tracks energy drift from the initial value (for undamped/low-damped systems).
///
/// Unlike [`EnergyMonotonicityTracker`] (which checks that energy never
/// *increases*), this tracker checks that energy stays *close to its initial
/// value*. Use this for undamped systems where energy should be conserved,
/// not just monotonically decreasing.
#[derive(Debug, Clone)]
pub struct EnergyConservationTracker {
    initial: Option<f64>,
    max_drift: f64,
}

impl EnergyConservationTracker {
    /// Create a new energy conservation tracker with no recorded samples.
    #[must_use]
    pub fn new() -> Self {
        Self {
            initial: None,
            max_drift: 0.0,
        }
    }

    /// Feed a new energy sample. The first sample sets the reference value.
    pub fn sample(&mut self, energy: f64) {
        match self.initial {
            None => self.initial = Some(energy),
            Some(initial) => {
                let drift = (energy - initial).abs();
                if drift > self.max_drift {
                    self.max_drift = drift;
                }
            }
        }
    }

    /// The largest absolute drift from the initial energy value.
    #[must_use]
    pub fn max_drift(&self) -> f64 {
        self.max_drift
    }

    /// The initial energy value (first sample), if any.
    #[must_use]
    pub fn initial_energy(&self) -> Option<f64> {
        self.initial
    }

    /// Check that energy drift stays below `threshold_pct` of the initial value.
    ///
    /// Returns a passing check if no samples were recorded (vacuously true).
    /// Returns a failing check if initial energy is zero (can't compute percentage).
    #[must_use]
    pub fn check(&self, threshold_pct: f64) -> Check {
        match self.initial {
            None => Check {
                name: "Energy conservation",
                pass: true,
                detail: "no samples recorded".to_string(),
            },
            Some(initial) if initial.abs() < 1e-15 => Check {
                name: "Energy conservation",
                pass: false,
                detail: "initial energy ≈ 0, cannot compute percentage drift".to_string(),
            },
            Some(initial) => {
                let drift_pct = (self.max_drift / initial.abs()) * 100.0;
                Check {
                    name: "Energy conservation",
                    pass: drift_pct < threshold_pct,
                    detail: format!(
                        "max drift={:.4}J ({drift_pct:.4}% of E0={initial:.4}J)",
                        self.max_drift
                    ),
                }
            }
        }
    }
}

impl Default for EnergyConservationTracker {
    fn default() -> Self {
        Self::new()
    }
}

// ── Equilibrium Tracker ───────────────────────────────────────────────────────

/// Tracks whether a scalar signal converges to an expected equilibrium value.
///
/// Uses a trailing time window: only samples within the last `window_duration`
/// seconds are retained. The check computes the mean over that window and
/// compares it to the expected equilibrium.
///
/// Useful for systems where gravity or other constant forces shift the rest
/// position (e.g. a vertical spring-mass system settles at `x_eq = -mg/k`).
#[derive(Debug, Clone)]
pub struct EquilibriumTracker {
    window_duration: f64,
    samples: Vec<(f64, f64)>, // (time, value)
}

impl EquilibriumTracker {
    /// Create a new equilibrium tracker with the given trailing window duration.
    ///
    /// Only samples within the last `window_duration` seconds (relative to the
    /// most recent sample) are kept.
    #[must_use]
    pub fn new(window_duration: f64) -> Self {
        Self {
            window_duration,
            samples: Vec::new(),
        }
    }

    /// Feed a new sample. Call once per frame with the current value and time.
    ///
    /// Older samples outside the trailing window are pruned automatically.
    pub fn sample(&mut self, value: f64, time: f64) {
        self.samples.push((time, value));
        let cutoff = time - self.window_duration;
        self.samples.retain(|&(t, _)| t >= cutoff);
    }

    /// Compute the mean value over the trailing window.
    /// Returns `None` if no samples are in the window.
    #[must_use]
    pub fn mean(&self) -> Option<f64> {
        if self.samples.is_empty() {
            return None;
        }
        #[allow(clippy::cast_precision_loss)]
        let mean = self.samples.iter().map(|&(_, v)| v).sum::<f64>() / self.samples.len() as f64;
        Some(mean)
    }

    /// Number of samples currently in the trailing window.
    #[must_use]
    pub fn sample_count(&self) -> usize {
        self.samples.len()
    }

    /// Check that the trailing-window mean is within `tolerance_pct`% of `expected`.
    ///
    /// Computes `|mean - expected| / |expected| * 100` and compares to `tolerance_pct`.
    /// Returns a failing check if no samples are in the window or if `expected` is zero.
    #[must_use]
    pub fn check(&self, expected: f64, tolerance_pct: f64) -> Check {
        match self.mean() {
            None => Check {
                name: "Equilibrium",
                pass: false,
                detail: "no samples in trailing window".to_string(),
            },
            Some(_) if expected.abs() < 1e-15 => Check {
                name: "Equilibrium",
                pass: false,
                detail: "expected equilibrium ≈ 0, cannot compute percentage error".to_string(),
            },
            Some(mean) => {
                let err_pct = ((mean - expected) / expected).abs() * 100.0;
                Check {
                    name: "Equilibrium",
                    pass: err_pct < tolerance_pct,
                    detail: format!("mean={mean:.4}  expected={expected:.4}  error={err_pct:.2}%"),
                }
            }
        }
    }
}

// ── Quaternion Utilities ──────────────────────────────────────────────────────

/// Compute the rotation angle (in radians) from quaternion components.
///
/// Returns the angle in \[0, π\]. This is the axis-angle representation's
/// angle component, useful for cone-limit validation and diagnostics.
///
/// Handles numerical edge cases: uses `atan2` for stability near 0 and π.
#[inline]
#[must_use]
pub fn quat_rotation_angle(w: f64, x: f64, y: f64, z: f64) -> f64 {
    let sin_half = (x * x + y * y + z * z).sqrt();
    2.0 * sin_half.atan2(w.abs())
}

// ── Report formatting ───────────────────────────────────────────────────────

/// Print a formatted validation report and return whether all checks passed.
///
/// Output format:
/// ```text
/// === Validation Report (t=15s) ===
///   Period:  measured=1.4739s  expected=1.4735s  error=0.02%  PASS
///   Energy:  max increase=0.00e0J  PASS
///   Limits:  max violation=0.000000m  PASS
/// ==================================
/// ```
///
/// Returns `true` if every check passed.
#[must_use]
pub fn print_report(title: &str, checks: &[Check]) -> bool {
    let bar = "=".repeat(title.len() + 8);
    println!("\n=== {title} ===");
    let name_width = checks.iter().map(|c| c.name.len()).max().unwrap_or(0);
    for check in checks {
        let status = if check.pass { "PASS" } else { "FAIL" };
        println!(
            "  {:width$}  {}  {status}",
            format!("{}:", check.name),
            check.detail,
            width = name_width + 1,
        );
    }
    println!("{bar}\n");
    all_passed(checks)
}

/// Check whether all checks passed without printing.
///
/// For pure headless/test use where stdout is not desired.
#[must_use]
pub fn all_passed(checks: &[Check]) -> bool {
    checks.iter().all(|c| c.pass)
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn period_tracker_basic() {
        let mut t = PeriodTracker::new();
        // Simulate sin wave: period = 1.0s, sampled at 100 Hz
        for i in 0..500 {
            let time = f64::from(i) * 0.01;
            let value = (2.0 * std::f64::consts::PI * time).sin();
            t.sample(value, time);
        }
        let period = t.measured_period().unwrap_or(0.0);
        assert!(
            (period - 1.0).abs() < 0.01,
            "period={period}, expected ~1.0"
        );
    }

    #[test]
    fn period_tracker_no_crossings() {
        let t = PeriodTracker::new();
        assert!(t.measured_period().is_none());
        let check = t.check(1.0, 2.0);
        assert!(!check.pass);
        assert!(check.detail.contains("insufficient"));
    }

    #[test]
    fn period_tracker_time_backwards() {
        let mut t = PeriodTracker::new();
        t.sample(1.0, 1.0);
        t.sample(-1.0, 0.5); // backwards — should be skipped
        t.sample(-1.0, 2.0);
        // Only one crossing (1.0 → -1.0 at time 1.0→2.0), not two
        assert_eq!(t.crossing_count(), 1);
    }

    #[test]
    fn energy_monotonicity_decreasing() {
        let mut e = EnergyMonotonicityTracker::new();
        for i in (0..100).rev() {
            e.sample(f64::from(i));
        }
        assert!(e.max_increase() <= 0.0);
        assert!(e.check(1e-6).pass);
    }

    #[test]
    fn energy_monotonicity_spike() {
        let mut e = EnergyMonotonicityTracker::new();
        e.sample(10.0);
        e.sample(9.0);
        e.sample(11.0); // spike!
        assert!((e.max_increase() - 2.0).abs() < 1e-10);
        assert!(!e.check(1.0).pass);
    }

    #[test]
    fn energy_monotonicity_no_samples() {
        let e = EnergyMonotonicityTracker::new();
        assert!(e.check(1e-6).pass); // vacuously true
    }

    #[test]
    fn limit_tracker_within_range() {
        let mut l = LimitTracker::new();
        for i in 0..100 {
            let v = (f64::from(i) - 50.0) * 0.01; // -0.5 to 0.49
            l.sample(v, -1.0, 1.0);
        }
        assert!(l.max_violation() < 1e-10);
        assert!(l.check(0.001).pass);
    }

    #[test]
    fn limit_tracker_violation() {
        let mut l = LimitTracker::new();
        l.sample(1.5, -1.0, 1.0); // 0.5 over high limit
        assert!((l.max_violation() - 0.5).abs() < 1e-10);
        assert!(!l.check(0.001).pass);
    }

    #[test]
    fn limit_tracker_no_samples() {
        let l = LimitTracker::new();
        assert!(l.check(0.001).pass); // vacuously true
    }

    #[test]
    fn quat_norm_tracker_unit() {
        let mut q = QuaternionNormTracker::new();
        q.sample(1.0, 0.0, 0.0, 0.0); // identity
        q.sample(0.0, 1.0, 0.0, 0.0); // 180° about x
        q.sample(
            std::f64::consts::FRAC_1_SQRT_2,
            std::f64::consts::FRAC_1_SQRT_2,
            0.0,
            0.0,
        ); // 90° about x
        assert!(q.max_deviation() < 1e-15);
        assert!(q.check(1e-10).pass);
    }

    #[test]
    fn quat_norm_tracker_drift() {
        let mut q = QuaternionNormTracker::new();
        q.sample(1.0, 0.0, 0.0, 0.0);
        q.sample(1.001, 0.0, 0.0, 0.0); // norm = 1.001
        assert!((q.max_deviation() - 0.001).abs() < 1e-10);
        assert!(!q.check(1e-10).pass);
        assert!(q.check(0.01).pass);
    }

    #[test]
    fn quat_norm_tracker_no_samples() {
        let q = QuaternionNormTracker::new();
        assert!(q.check(1e-10).pass); // vacuously true
    }

    #[test]
    fn energy_conservation_stable() {
        let mut e = EnergyConservationTracker::new();
        for i in 0..100 {
            // oscillate around 10.0 with ±0.001
            let energy = 10.0 + 0.001 * (f64::from(i) * 0.1).sin();
            e.sample(energy);
        }
        assert!(e.max_drift() < 0.002);
        assert!(e.check(0.5).pass); // 0.5% of 10.0 = 0.05 >> 0.001
    }

    #[test]
    fn energy_conservation_drift() {
        let mut e = EnergyConservationTracker::new();
        e.sample(10.0);
        e.sample(10.0);
        e.sample(9.0); // 10% drift
        assert!((e.max_drift() - 1.0).abs() < 1e-10);
        assert!(!e.check(0.5).pass); // 10% > 0.5%
        assert!(e.check(15.0).pass); // 10% < 15%
    }

    #[test]
    fn energy_conservation_no_samples() {
        let e = EnergyConservationTracker::new();
        assert!(e.check(0.5).pass); // vacuously true
    }

    #[test]
    fn energy_conservation_zero_initial() {
        let mut e = EnergyConservationTracker::new();
        e.sample(0.0);
        e.sample(0.1);
        assert!(!e.check(0.5).pass); // can't compute percentage
    }

    #[test]
    fn equilibrium_tracker_converges() {
        let mut e = EquilibriumTracker::new(5.0);
        // Simulate damped oscillation converging to -0.245
        for i in 0..1000 {
            let t = f64::from(i) * 0.02; // 50 Hz, 20 seconds total
            let decay = (-0.3 * t).exp();
            let value = -0.245 + 0.5 * decay * (6.0 * t).sin();
            e.sample(value, t);
        }
        // After 20s with 5s window, mean should be very close to -0.245
        let check = e.check(-0.245, 5.0);
        assert!(check.pass, "detail: {}", check.detail);
    }

    #[test]
    fn equilibrium_tracker_window_prunes() {
        let mut e = EquilibriumTracker::new(2.0);
        // Phase 1: t=0..8, value=10.0
        for i in 0..80 {
            let t = f64::from(i) * 0.1;
            e.sample(10.0, t);
        }
        // Phase 2: t=8..12, value=1.0
        for i in 80..120 {
            let t = f64::from(i) * 0.1;
            e.sample(1.0, t);
        }
        // Window is [10.0, 12.0], all samples are value=1.0
        let mean = e.mean().unwrap_or(f64::NAN);
        assert!(
            (mean - 1.0).abs() < 0.01,
            "mean={mean}, expected ~1.0 (window should prune old samples)"
        );
    }

    #[test]
    fn equilibrium_tracker_no_samples() {
        let e = EquilibriumTracker::new(5.0);
        let check = e.check(-0.245, 5.0);
        assert!(!check.pass);
        assert!(check.detail.contains("no samples"));
    }

    #[test]
    fn equilibrium_tracker_zero_expected() {
        let mut e = EquilibriumTracker::new(5.0);
        e.sample(0.1, 1.0);
        let check = e.check(0.0, 5.0);
        assert!(!check.pass);
        assert!(check.detail.contains("cannot compute"));
    }

    #[test]
    fn quat_rotation_angle_identity() {
        let angle = quat_rotation_angle(1.0, 0.0, 0.0, 0.0);
        assert!(
            angle.abs() < 1e-15,
            "identity should give angle=0, got {angle}"
        );
    }

    #[test]
    fn quat_rotation_angle_90_about_x() {
        let half = std::f64::consts::FRAC_PI_4; // 45° = half of 90°
        let angle = quat_rotation_angle(half.cos(), half.sin(), 0.0, 0.0);
        assert!(
            (angle - std::f64::consts::FRAC_PI_2).abs() < 1e-12,
            "90° about X: expected pi/2, got {angle}"
        );
    }

    #[test]
    fn quat_rotation_angle_180_about_z() {
        let angle = quat_rotation_angle(0.0, 0.0, 0.0, 1.0);
        assert!(
            (angle - std::f64::consts::PI).abs() < 1e-12,
            "180° about Z: expected pi, got {angle}"
        );
    }

    #[test]
    fn all_passed_works() {
        let checks = vec![
            Check {
                name: "A",
                pass: true,
                detail: String::new(),
            },
            Check {
                name: "B",
                pass: true,
                detail: String::new(),
            },
        ];
        assert!(all_passed(&checks));

        let checks_fail = vec![
            Check {
                name: "A",
                pass: true,
                detail: String::new(),
            },
            Check {
                name: "B",
                pass: false,
                detail: String::new(),
            },
        ];
        assert!(!all_passed(&checks_fail));
    }
}
