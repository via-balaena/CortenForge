//! Public test utilities for validating stochastic passive components.
//!
//! Per chassis Decision 5 + M4, this module ships:
//!
//! - [`WelfordOnline`]: a numerically stable single-pass mean/variance
//!   accumulator (Welford 1962) with `reset` (M4: burn-in support) and
//!   `merge` (M4: Chan/Pébay parallel-accumulator combination). The
//!   Phase 1 equipartition gate (spec §7.3 option β) uses a **two-level
//!   Welford pattern** built on `push` and `mean` — a per-trajectory
//!   inner accumulator collects per-step `½v²` samples, and its scalar
//!   `mean()` is pushed into a top-level accumulator that holds the
//!   100 trajectory means as IID samples. `merge` is **not** used by
//!   the §7 gate (see
//!   `06_findings/2026-04-09_phase1_statistical_propagation_chain.md`
//!   for why merging per-step accumulators across trajectories
//!   underestimates the std error by `√(1+2·τ_int) ≈ 100`); it ships
//!   for IID parallel-reduce contexts in Phase 4+ batch reductions.
//!
//! - [`assert_within_n_sigma`]: a small assertion helper that fails
//!   with a clear diagnostic when a measured value deviates from its
//!   expected value by more than `n_sigma · standard_error`. The
//!   default `n_sigma = 3.0` is the chassis sub-decision N2 default;
//!   the Phase 1 tests use the default verbatim.
//!
//! - [`sample_stats`]: a one-shot helper that returns `(mean, variance)`
//!   over a slice — useful for tests that don't need the streaming
//!   accumulator's `merge`/`reset` machinery.
//!
//! ## Why this lives in `pub mod test_utils` and not `#[cfg(test)]`
//!
//! The chassis Decision-6 layout puts these helpers behind
//! `pub mod test_utils` so they are reachable from downstream
//! integration tests in other crates (notably the Phase 1
//! integration test in `tests/langevin_thermostat.rs` and any
//! Phase 2+ phase-specific test that wants the same statistical
//! machinery). Behind `#[cfg(test)]` they would be invisible to
//! integration tests, which compile against the crate as a normal
//! library consumer.

/// Hysteresis-based well-state classification for bistable elements.
///
/// Used by Phase 3+ tests to classify a 1-DOF position into one of three
/// regions: left well, right well, or barrier. The threshold `x_thresh`
/// defines the boundary between well and barrier regions.
///
/// At the Phase 3 central parameters (`κ = λ_r/ω_b = 0.313`), ~69% of
/// zero-crossings are recrossings. Hysteresis at `x_thresh = x₀/2` filters
/// these out and recovers the genuine committed-transition rate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WellState {
    /// Position is in the left well: `x < −x_thresh`.
    Left,
    /// Position is in the right well: `x > +x_thresh`.
    Right,
    /// Position is in the barrier region: `−x_thresh ≤ x ≤ +x_thresh`.
    Barrier,
}

impl WellState {
    /// Classify a position into a well state given the hysteresis threshold.
    #[must_use]
    pub fn from_position(x: f64, x_thresh: f64) -> Self {
        if x > x_thresh {
            Self::Right
        } else if x < -x_thresh {
            Self::Left
        } else {
            Self::Barrier
        }
    }

    /// Convert to a spin value: `+1.0` for Right, `−1.0` for Left.
    ///
    /// # Panics
    /// Panics if called on `Barrier` — callers must check
    /// [`is_in_well`](Self::is_in_well) first.
    #[must_use]
    #[allow(clippy::panic)]
    pub fn spin(self) -> f64 {
        match self {
            Self::Right => 1.0,
            Self::Left => -1.0,
            Self::Barrier => panic!("spin() called on Barrier state"),
        }
    }

    /// Returns `true` if the element is in a well (not in the barrier).
    #[must_use]
    pub fn is_in_well(self) -> bool {
        self != Self::Barrier
    }
}

/// Numerically stable single-pass mean/variance accumulator.
///
/// Uses Welford's 1962 incremental algorithm: store `count`, running
/// `mean`, and `m2` (the running sum of squared deviations from the
/// current mean), and update them per sample so the variance can be
/// recovered as `m2 / (count - 1)` without ever forming the naive
/// `Σx²` (which loses precision for samples with large mean and
/// small variance — exactly the regime the equipartition gate runs
/// in: `½v² ≈ 0.5` with std error `≈ 0.032`).
///
/// Supports both [`reset`](Self::reset) (M4: re-initialize for
/// burn-in) and [`merge`](Self::merge) (M4: combine two independent
/// accumulators via Chan/Pébay 1979/2008). The Phase 1 §7.3 gate
/// uses `push`/`mean` in a **two-level pattern** (per-trajectory
/// inner accumulator + across-trajectories top-level accumulator);
/// `merge` ships for Phase 4+ IID parallel-reduce contexts but is
/// not used by the §7 gate. See the module-level docstring and
/// `06_findings/2026-04-09_phase1_statistical_propagation_chain.md`
/// for why.
#[derive(Clone, Debug)]
pub struct WelfordOnline {
    count: usize,
    mean: f64,
    m2: f64,
}

impl WelfordOnline {
    /// Construct an empty accumulator.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            count: 0,
            mean: 0.0,
            m2: 0.0,
        }
    }

    /// Push a single sample. O(1) per call.
    pub fn push(&mut self, x: f64) {
        self.count += 1;
        let delta = x - self.mean;
        // count is at least 1 here, so the cast is exact for small
        // counts and benign for the large counts the Phase 1 gate
        // produces (~10⁷).
        #[allow(clippy::cast_precision_loss)]
        let count_f64 = self.count as f64;
        self.mean += delta / count_f64;
        let delta2 = x - self.mean;
        self.m2 += delta * delta2;
    }

    /// Re-initialize the accumulator. Required by M4 to support the
    /// "burn-in then measure" pattern: push N steps, call `reset`,
    /// then push the measurement window. Cheaper than constructing
    /// a fresh `WelfordOnline` because no allocation is involved.
    pub const fn reset(&mut self) {
        self.count = 0;
        self.mean = 0.0;
        self.m2 = 0.0;
    }

    /// Merge another independent accumulator into `self` via the
    /// `Chan/Pébay` parallel-merge formula (Chan, Golub, `LeVeque`
    /// 1979; Pébay 2008).
    ///
    /// For two accumulators `(n_a, μ_a, M2_a)` and `(n_b, μ_b, M2_b)`,
    /// the combined statistics are:
    ///
    /// - `n   = n_a + n_b`
    /// - `δ   = μ_b − μ_a`
    /// - `μ   = μ_a + δ · n_b / n`
    /// - `M2  = M2_a + M2_b + δ² · n_a · n_b / n`
    ///
    /// **`merge` is intended for IID parallel-reduce contexts** (e.g.
    /// folding per-env statistics across `BatchSim` envs in Phase 4+
    /// where every sample is independent of every other). It is
    /// **NOT** used by the Phase 1 §7 equipartition gate. The §7.3
    /// gate uses a two-level Welford pattern (`push`/`mean` only)
    /// because per-step `½v²` samples within a trajectory are
    /// autocorrelated (`τ_int ≈ 5000` steps for the central case),
    /// and merging per-step accumulators across trajectories then
    /// calling `std_error_of_mean` on the merged result yields the
    /// IID std error — which underestimates the true std error by
    /// `√(1+2·τ_int) ≈ 100`. See
    /// `06_findings/2026-04-09_phase1_statistical_propagation_chain.md`
    /// for the full propagation-chain post-mortem.
    ///
    /// The unit test
    /// `welford_merge_matches_one_pass_over_full_dataset` locks the
    /// formula by splitting a deterministic dataset into two halves,
    /// computing Welford on each half, merging, and asserting the
    /// merged `mean`/`variance` match a one-pass Welford over the
    /// whole dataset to within an absolute tolerance of `1e-12`. The
    /// merge formula is mathematically correct for IID samples; it
    /// just isn't the right primitive for the §7 gate's autocorrelated
    /// per-step samples.
    pub fn merge(&mut self, other: &Self) {
        if other.count == 0 {
            return;
        }
        if self.count == 0 {
            *self = other.clone();
            return;
        }
        let total_count = self.count + other.count;
        let delta = other.mean - self.mean;
        // u64 → f64 cast: counts are small enough that precision loss is negligible.
        #[allow(clippy::cast_precision_loss)]
        let weight_a = self.count as f64;
        #[allow(clippy::cast_precision_loss)]
        let weight_b = other.count as f64;
        // (same justification as weight_a/weight_b above)
        #[allow(clippy::cast_precision_loss)]
        let weight_total = total_count as f64;
        self.mean += delta * weight_b / weight_total;
        self.m2 += other.m2 + delta * delta * weight_a * weight_b / weight_total;
        self.count = total_count;
    }

    /// Number of samples observed so far.
    #[must_use]
    pub const fn count(&self) -> usize {
        self.count
    }

    /// Running sample mean. Returns `0.0` if no samples have been
    /// pushed (the empty mean is conventionally zero, not `NaN`, so
    /// callers can default-initialize without special-casing).
    #[must_use]
    pub const fn mean(&self) -> f64 {
        self.mean
    }

    /// Unbiased sample variance with `(n − 1)` denominator. Returns
    /// `f64::NAN` for `count < 2` because the unbiased estimator is
    /// mathematically undefined there — propagating `NaN` downstream
    /// produces a loud failure in `assert_within_n_sigma` rather
    /// than a silent pass on a degenerate sample.
    #[must_use]
    pub fn variance(&self) -> f64 {
        if self.count < 2 {
            return f64::NAN;
        }
        // u64 → f64: count is small enough that precision loss is negligible.
        #[allow(clippy::cast_precision_loss)]
        let denom = (self.count - 1) as f64;
        self.m2 / denom
    }

    /// Standard error of the mean: `sqrt(variance / count)`. Returns
    /// `f64::NAN` for `count < 2` (same reasoning as `variance`).
    #[must_use]
    pub fn std_error_of_mean(&self) -> f64 {
        if self.count < 2 {
            return f64::NAN;
        }
        // u64 → f64: count is small enough that precision loss is negligible.
        #[allow(clippy::cast_precision_loss)]
        let count_f64 = self.count as f64;
        (self.variance() / count_f64).sqrt()
    }
}

impl Default for WelfordOnline {
    fn default() -> Self {
        Self::new()
    }
}

/// Assert that `measured` is within `n_sigma · standard_error` of
/// `expected`.
///
/// Default convention: `n_sigma = 3.0` per chassis sub-decision N2.
/// At `n_sigma = 3.0` the false-rejection rate on a true Gaussian
/// distribution is ~0.27%, which is the right ballpark for
/// equipartition tests that run on every thermo-touching PR (a
/// 1-in-370 flake rate is acceptable; a 1-in-20 flake rate from
/// `n_sigma = 2.0` is not).
///
/// The `description` argument should name the test and parameter set
/// (e.g. `"equipartition central: 1-DOF damped harmonic oscillator,
/// 100 traj × 2×10⁵ steps"`) so failures in CI logs are immediately
/// identifiable without grepping back to the test source.
///
/// # Panics
///
/// Panics if `(measured - expected).abs() > n_sigma * standard_error`,
/// or if any input is `NaN` (the comparison `NaN <= bound` is `false`,
/// which trips the assertion). The latter case is intentional — a
/// `NaN` standard error from a degenerate `WelfordOnline` (count < 2)
/// will produce a loud failure here instead of a silent pass.
pub fn assert_within_n_sigma(
    measured: f64,
    expected: f64,
    standard_error: f64,
    n_sigma: f64,
    description: &str,
) {
    let deviation = (measured - expected).abs();
    let bound = n_sigma * standard_error;
    assert!(
        deviation <= bound,
        "assert_within_n_sigma failed [{description}]:\n  \
         measured = {measured:.6e}\n  \
         expected = {expected:.6e}\n  \
         deviation = {deviation:.6e}\n  \
         standard_error = {standard_error:.6e}\n  \
         n_sigma = {n_sigma}\n  \
         bound = {bound:.6e} (= {n_sigma} · standard_error)\n  \
         relative_deviation = {rel:.4}σ",
        rel = deviation / standard_error,
    );
}

/// One-shot `(mean, variance)` over a slice.
///
/// Useful for tests that don't need the streaming accumulator's
/// `merge`/`reset` machinery. Returns `(0.0, NaN)` for `data.len() < 2`
/// to match `WelfordOnline`'s degenerate-input convention.
#[must_use]
pub fn sample_stats(data: &[f64]) -> (f64, f64) {
    let mut acc = WelfordOnline::new();
    for &x in data {
        acc.push(x);
    }
    (acc.mean(), acc.variance())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn welford_basic_mean_and_variance() {
        let mut w = WelfordOnline::new();
        for &x in &[2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0] {
            w.push(x);
        }
        assert_eq!(w.count(), 8);
        assert!((w.mean() - 5.0).abs() < 1e-12);
        // hand-computed unbiased variance: m2/(n-1) = 32/7
        assert!((w.variance() - 32.0 / 7.0).abs() < 1e-12);
    }

    #[test]
    fn welford_empty_returns_zero_mean_and_nan_variance() {
        let w = WelfordOnline::new();
        assert_eq!(w.count(), 0);
        assert_eq!(w.mean(), 0.0);
        assert!(w.variance().is_nan());
        assert!(w.std_error_of_mean().is_nan());
    }

    #[test]
    fn welford_single_sample_returns_nan_variance() {
        let mut w = WelfordOnline::new();
        w.push(42.0);
        assert_eq!(w.count(), 1);
        assert_eq!(w.mean(), 42.0);
        // n=1 is degenerate for the unbiased estimator. NaN is the
        // mathematically correct answer; propagating it downstream
        // produces a loud failure in assert_within_n_sigma instead
        // of a silent pass-by-zero.
        assert!(w.variance().is_nan());
        assert!(w.std_error_of_mean().is_nan());
    }

    #[test]
    fn welford_reset_clears_state() {
        let mut w = WelfordOnline::new();
        w.push(1.0);
        w.push(2.0);
        w.push(3.0);
        assert_eq!(w.count(), 3);
        w.reset();
        assert_eq!(w.count(), 0);
        assert_eq!(w.mean(), 0.0);
        assert!(w.variance().is_nan());

        // After reset, push fresh samples — count restarts, mean is
        // computed only over the post-reset window.
        w.push(10.0);
        w.push(20.0);
        assert_eq!(w.count(), 2);
        assert_eq!(w.mean(), 15.0);
    }

    #[test]
    fn welford_merge_matches_one_pass_over_full_dataset() {
        // The load-bearing M4 test. Split a deterministic dataset
        // into two halves, compute Welford on each half, merge, and
        // assert the merged mean/variance match a single-pass Welford
        // over the whole dataset to within absolute 1e-12.
        //
        // The dataset includes both small and large values to exercise
        // the numerically stable update; equal halves and unequal
        // halves are tested via two sub-cases.
        let full = [1.0, 2.5, 3.5, 7.25, 11.125, 13.0, 17.875, 23.0];

        let mut full_acc = WelfordOnline::new();
        for &x in &full {
            full_acc.push(x);
        }

        // Sub-case 1: equal halves (4 + 4)
        {
            let mut a = WelfordOnline::new();
            for &x in &full[..4] {
                a.push(x);
            }
            let mut b = WelfordOnline::new();
            for &x in &full[4..] {
                b.push(x);
            }
            a.merge(&b);
            assert_eq!(a.count(), full_acc.count());
            assert!(
                (a.mean() - full_acc.mean()).abs() < 1e-12,
                "merge equal halves: mean drift {} vs {}",
                a.mean(),
                full_acc.mean(),
            );
            assert!(
                (a.variance() - full_acc.variance()).abs() < 1e-12,
                "merge equal halves: variance drift {} vs {}",
                a.variance(),
                full_acc.variance(),
            );
        }

        // Sub-case 2: unequal halves (3 + 5)
        {
            let mut a = WelfordOnline::new();
            for &x in &full[..3] {
                a.push(x);
            }
            let mut b = WelfordOnline::new();
            for &x in &full[3..] {
                b.push(x);
            }
            a.merge(&b);
            assert_eq!(a.count(), full_acc.count());
            assert!(
                (a.mean() - full_acc.mean()).abs() < 1e-12,
                "merge unequal halves: mean drift {} vs {}",
                a.mean(),
                full_acc.mean(),
            );
            assert!(
                (a.variance() - full_acc.variance()).abs() < 1e-12,
                "merge unequal halves: variance drift {} vs {}",
                a.variance(),
                full_acc.variance(),
            );
        }
    }

    #[test]
    fn welford_merge_with_empty_accumulator_is_identity() {
        // Edge case: merging an empty accumulator into a non-empty
        // one (or vice versa) should be the identity operation.
        let mut a = WelfordOnline::new();
        for &x in &[1.0, 2.0, 3.0] {
            a.push(x);
        }
        let snapshot = (a.count(), a.mean(), a.variance());

        let empty = WelfordOnline::new();
        a.merge(&empty);
        assert_eq!(a.count(), snapshot.0);
        assert_eq!(a.mean(), snapshot.1);
        assert!((a.variance() - snapshot.2).abs() < 1e-12);

        // The other direction: empty.merge(&non_empty) should yield
        // the non-empty accumulator.
        let mut empty = WelfordOnline::new();
        empty.merge(&a);
        assert_eq!(empty.count(), a.count());
        assert_eq!(empty.mean(), a.mean());
        assert!((empty.variance() - a.variance()).abs() < 1e-12);
    }

    #[test]
    fn welford_std_error_of_mean_basic() {
        // For [1, 2, 3, 4, 5]: mean=3, variance=2.5, sem=sqrt(2.5/5)=sqrt(0.5)
        let mut w = WelfordOnline::new();
        for &x in &[1.0, 2.0, 3.0, 4.0, 5.0] {
            w.push(x);
        }
        assert!((w.mean() - 3.0).abs() < 1e-12);
        assert!((w.variance() - 2.5).abs() < 1e-12);
        assert!((w.std_error_of_mean() - 0.5_f64.sqrt()).abs() < 1e-12);
    }

    #[test]
    fn assert_within_n_sigma_passes_at_threshold() {
        // measured exactly at the bound — should pass (uses <=)
        assert_within_n_sigma(3.0, 0.0, 1.0, 3.0, "exactly at threshold");
    }

    #[test]
    fn assert_within_n_sigma_passes_well_inside() {
        assert_within_n_sigma(0.5, 0.0, 1.0, 3.0, "well inside");
    }

    #[test]
    #[should_panic(expected = "assert_within_n_sigma failed")]
    fn assert_within_n_sigma_panics_when_outside() {
        assert_within_n_sigma(4.0, 0.0, 1.0, 3.0, "outside the bound");
    }

    #[test]
    #[should_panic(expected = "assert_within_n_sigma failed")]
    fn assert_within_n_sigma_panics_on_nan_standard_error() {
        // n=1 case from WelfordOnline: std_error is NaN, deviation
        // comparison NaN <= bound is false, so the assertion fires
        // — exactly the loud-failure-on-degenerate-sample property
        // we want.
        assert_within_n_sigma(1.0, 0.0, f64::NAN, 3.0, "NaN std error");
    }

    #[test]
    fn sample_stats_matches_welford() {
        let data = [1.0, 2.0, 3.0, 4.0, 5.0];
        let (mean, var) = sample_stats(&data);
        assert!((mean - 3.0).abs() < 1e-12);
        assert!((var - 2.5).abs() < 1e-12);
    }

    #[test]
    fn sample_stats_empty_returns_zero_and_nan() {
        let (mean, var) = sample_stats(&[]);
        assert_eq!(mean, 0.0);
        assert!(var.is_nan());
    }

    #[test]
    fn welford_default_equals_new() {
        let a = WelfordOnline::new();
        let b = WelfordOnline::default();
        assert_eq!(a.count(), b.count());
        assert_eq!(a.mean(), b.mean());
        assert!(a.variance().is_nan() && b.variance().is_nan());
    }

    // ─── WellState tests ──────────────────────────────────────────────

    #[test]
    fn well_state_from_position_classifies_correctly() {
        let thresh = 0.5;
        assert_eq!(WellState::from_position(1.0, thresh), WellState::Right);
        assert_eq!(WellState::from_position(-1.0, thresh), WellState::Left);
        assert_eq!(WellState::from_position(0.0, thresh), WellState::Barrier);
        assert_eq!(WellState::from_position(0.5, thresh), WellState::Barrier);
        assert_eq!(WellState::from_position(-0.5, thresh), WellState::Barrier);
        assert_eq!(
            WellState::from_position(0.500_001, thresh),
            WellState::Right
        );
        assert_eq!(
            WellState::from_position(-0.500_001, thresh),
            WellState::Left
        );
    }

    #[test]
    fn well_state_spin_values() {
        assert_eq!(WellState::Right.spin(), 1.0);
        assert_eq!(WellState::Left.spin(), -1.0);
    }

    #[test]
    #[should_panic(expected = "spin() called on Barrier")]
    fn well_state_spin_panics_on_barrier() {
        #[allow(clippy::let_underscore_must_use)]
        let _ = WellState::Barrier.spin();
    }

    #[test]
    fn well_state_is_in_well() {
        assert!(WellState::Right.is_in_well());
        assert!(WellState::Left.is_in_well());
        assert!(!WellState::Barrier.is_in_well());
    }
}
