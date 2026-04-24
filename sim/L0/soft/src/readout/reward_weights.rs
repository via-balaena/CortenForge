//! `RewardWeights` — γ-locked scalar-composition weights.
//!
//! Matches `RewardBreakdown` field-by-field; consumed by
//! `RewardBreakdown::score_with`.

/// Per-term scalar weights for reward composition.
#[derive(Clone, Copy, Debug, Default)]
pub struct RewardWeights {
    /// Weight on `RewardBreakdown::pressure_uniformity`.
    pub pressure_uniformity: f64,
    /// Weight on `RewardBreakdown::coverage`.
    pub coverage: f64,
    /// Weight on `RewardBreakdown::peak_bound`.
    pub peak_bound: f64,
    /// Weight on `RewardBreakdown::stiffness_bound`.
    pub stiffness_bound: f64,
}
