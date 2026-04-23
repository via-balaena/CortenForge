//! `RewardBreakdown` — γ-locked per-term reward struct.
//!
//! Four fields per Part 10 Ch 00 00-forward.md:47 + Part 1 Ch 01.
//! Skeleton 1-tet scene leaves `pressure_uniformity` + `coverage` as
//! `f64::NAN` sentinels (see spec §2 reward table); `peak_bound` +
//! `stiffness_bound` are populated from `StressField` + material
//! tangent eigenspectrum.

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
    #[must_use]
    pub fn score_with(&self, _weights: &RewardWeights) -> f64 {
        unimplemented!("skeleton phase 2")
    }

    /// Sim-to-real correction composer per δ Ch 00 readout §3.
    /// Consumed by `SimToRealCorrection::correct`, post-Phase-I.
    #[must_use]
    pub fn apply_residuals(&self, _residuals: &ResidualCorrections) -> Self {
        unimplemented!("skeleton phase 2")
    }
}
