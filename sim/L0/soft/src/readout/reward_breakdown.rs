//! `RewardBreakdown` — γ-locked per-term reward struct.
//!
//! Four fields per Part 10 Ch 00 00-forward.md:47 + Part 1 Ch 01.
//! Skeleton 1-tet scene leaves `pressure_uniformity` + `coverage` as
//! `f64::NAN` sentinels (see spec §2 reward table); `peak_bound` +
//! `stiffness_bound` are populated from `StressField` + material
//! tangent eigenspectrum.

// `apply_residuals` is `unimplemented!("skeleton phase 2")` by design;
// sim-to-real correction composition is post-Phase-I per γ Ch 03/04.
// Module-level allow matches the `lib.rs` override.
#![allow(clippy::unimplemented)]

use super::RewardWeights;

/// Per-term reward contributions. Scalar composition via `score_with`;
/// sim-to-real correction composition via `apply_residuals`.
#[derive(Clone, Debug, Default)]
pub struct RewardBreakdown {
    /// Uniformity of hydrostatic pressure across the domain. `NaN` on
    /// the 1-tet skeleton scene (structurally undefined with a single
    /// element).
    pub pressure_uniformity: f64,
    /// Fraction of the intended contact surface that is actively
    /// loaded. `NaN` on the 1-tet skeleton (no contact model).
    pub coverage: f64,
    /// Peak Cauchy stress magnitude vs. material bound (unitless).
    pub peak_bound: f64,
    /// Minimum eigenvalue of the material tangent vs. stiffness target
    /// (unitless).
    pub stiffness_bound: f64,
}

/// Residual corrections sourced from `SimToRealCorrection::correct`
/// (Part 10 Ch 05 §00). Unit stub; Phase G+ carries per-field
/// corrections consumed by `apply_residuals`.
#[derive(Clone, Debug, Default)]
pub struct ResidualCorrections;

impl RewardBreakdown {
    /// Scalar composition per Part 1 Ch 01 composition rule. Consumed
    /// by downstream optimizers that need a scalar reward.
    ///
    /// **`NaN`-sentinel contract.** Fields carrying `f64::NAN` are
    /// silently dropped from the sum (scope §2 1-tet gap: the skeleton
    /// encodes "structurally undefined" as `NaN` on `pressure_uniformity`
    /// and `coverage`). IEEE 754 makes `NaN × 0.0 = NaN`, so a plain
    /// weighted sum would poison the output; this method branches on
    /// `is_nan` instead. `SkeletonForwardMap::build_reward_on_tape`
    /// mirrors the same branch at tape-build time so the on-tape
    /// reward stays numerically consistent with `score_with`.
    #[must_use]
    pub fn score_with(&self, weights: &RewardWeights) -> f64 {
        let mut acc = 0.0;
        if !self.pressure_uniformity.is_nan() {
            acc += weights.pressure_uniformity * self.pressure_uniformity;
        }
        if !self.coverage.is_nan() {
            acc += weights.coverage * self.coverage;
        }
        if !self.peak_bound.is_nan() {
            acc += weights.peak_bound * self.peak_bound;
        }
        if !self.stiffness_bound.is_nan() {
            acc += weights.stiffness_bound * self.stiffness_bound;
        }
        acc
    }

    /// Sim-to-real correction composer per δ Ch 00 readout §3.
    /// Consumed by `SimToRealCorrection::correct`, post-Phase-I.
    #[must_use]
    pub fn apply_residuals(&self, _residuals: &ResidualCorrections) -> Self {
        unimplemented!("skeleton phase 2")
    }
}
