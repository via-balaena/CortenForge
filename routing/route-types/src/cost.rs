//! Cost types for multi-objective routing.
//!
//! This module defines types for specifying cost weights and computing
//! route costs across multiple objectives.
//!
//! # Example
//!
//! ```
//! use route_types::{CostWeights, RouteCost};
//!
//! let weights = CostWeights::default()
//!     .with_length(1.0)
//!     .with_bends(0.5)
//!     .with_clearance(0.3);
//!
//! let cost = RouteCost::new()
//!     .with_length(10.0)
//!     .with_bends(3.0);
//!
//! let total = cost.compute_weighted(&weights);
//! ```

/// Cost function weights for multi-objective routing.
///
/// These weights determine how different objectives are balanced
/// when computing the total path cost. Higher weights mean that
/// objective is more important.
///
/// # Example
///
/// ```
/// use route_types::CostWeights;
///
/// // Prioritize short paths with good clearance
/// let weights = CostWeights::default()
///     .with_length(1.0)
///     .with_clearance(0.8)
///     .with_bends(0.2);
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CostWeights {
    /// Weight for path length (default: 1.0).
    length: f64,
    /// Weight for number of bends/direction changes.
    bends: f64,
    /// Weight for curvature (penalizes sharp turns).
    curvature: f64,
    /// Weight for clearance (negative = reward more clearance).
    clearance: f64,
    /// Weight for deviation from preferred corridors.
    corridor_deviation: f64,
}

impl CostWeights {
    /// Creates cost weights with all values set to zero.
    #[must_use]
    pub const fn zero() -> Self {
        Self {
            length: 0.0,
            bends: 0.0,
            curvature: 0.0,
            clearance: 0.0,
            corridor_deviation: 0.0,
        }
    }

    /// Creates cost weights that only consider path length.
    ///
    /// This is the standard shortest-path objective.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::CostWeights;
    ///
    /// let weights = CostWeights::length_only();
    /// assert!((weights.length() - 1.0).abs() < 1e-10);
    /// assert!((weights.bends() - 0.0).abs() < 1e-10);
    /// ```
    #[must_use]
    pub const fn length_only() -> Self {
        Self {
            length: 1.0,
            bends: 0.0,
            curvature: 0.0,
            clearance: 0.0,
            corridor_deviation: 0.0,
        }
    }

    /// Sets the length weight.
    #[must_use]
    pub const fn with_length(mut self, weight: f64) -> Self {
        self.length = weight;
        self
    }

    /// Sets the bends weight.
    #[must_use]
    pub const fn with_bends(mut self, weight: f64) -> Self {
        self.bends = weight;
        self
    }

    /// Sets the curvature weight.
    #[must_use]
    pub const fn with_curvature(mut self, weight: f64) -> Self {
        self.curvature = weight;
        self
    }

    /// Sets the clearance weight.
    ///
    /// Note: A positive weight penalizes low clearance.
    /// To reward clearance, use negative clearance values in [`RouteCost`].
    #[must_use]
    pub const fn with_clearance(mut self, weight: f64) -> Self {
        self.clearance = weight;
        self
    }

    /// Sets the corridor deviation weight.
    #[must_use]
    pub const fn with_corridor_deviation(mut self, weight: f64) -> Self {
        self.corridor_deviation = weight;
        self
    }

    /// Returns the length weight.
    #[must_use]
    pub const fn length(&self) -> f64 {
        self.length
    }

    /// Returns the bends weight.
    #[must_use]
    pub const fn bends(&self) -> f64 {
        self.bends
    }

    /// Returns the curvature weight.
    #[must_use]
    pub const fn curvature(&self) -> f64 {
        self.curvature
    }

    /// Returns the clearance weight.
    #[must_use]
    pub const fn clearance(&self) -> f64 {
        self.clearance
    }

    /// Returns the corridor deviation weight.
    #[must_use]
    pub const fn corridor_deviation(&self) -> f64 {
        self.corridor_deviation
    }

    /// Normalizes the weights so they sum to 1.0.
    ///
    /// Useful for consistent cost scaling across different weight configurations.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::CostWeights;
    ///
    /// let weights = CostWeights::zero()
    ///     .with_length(2.0)
    ///     .with_bends(3.0)
    ///     .normalized();
    ///
    /// let sum = weights.length() + weights.bends();
    /// assert!((sum - 1.0).abs() < 1e-10);
    /// ```
    #[must_use]
    pub fn normalized(&self) -> Self {
        let sum =
            self.length + self.bends + self.curvature + self.clearance + self.corridor_deviation;
        if sum.abs() < f64::EPSILON {
            return Self::length_only();
        }
        Self {
            length: self.length / sum,
            bends: self.bends / sum,
            curvature: self.curvature / sum,
            clearance: self.clearance / sum,
            corridor_deviation: self.corridor_deviation / sum,
        }
    }
}

impl Default for CostWeights {
    /// Default weights: length=1.0, all others=0.0
    fn default() -> Self {
        Self::length_only()
    }
}

/// Computed cost breakdown for a route.
///
/// Contains individual cost components that can be combined
/// using [`CostWeights`] to compute a total weighted cost.
///
/// # Example
///
/// ```
/// use route_types::{RouteCost, CostWeights};
///
/// let cost = RouteCost::new()
///     .with_length(15.0)
///     .with_bends(4.0)
///     .with_clearance(-2.0);  // Negative = good clearance
///
/// let weights = CostWeights::default()
///     .with_length(1.0)
///     .with_bends(0.5)
///     .with_clearance(1.0);
///
/// let total = cost.compute_weighted(&weights);
/// // total = 1.0*15 + 0.5*4 + 1.0*(-2) = 15 + 2 - 2 = 15
/// ```
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RouteCost {
    /// Path length cost component.
    length: f64,
    /// Number of bends/direction changes.
    bends: f64,
    /// Curvature cost (sum or max of curvature values).
    curvature: f64,
    /// Clearance cost (negative values indicate good clearance).
    clearance: f64,
    /// Corridor deviation cost.
    corridor_deviation: f64,
}

impl RouteCost {
    /// Creates a new route cost with all components at zero.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            length: 0.0,
            bends: 0.0,
            curvature: 0.0,
            clearance: 0.0,
            corridor_deviation: 0.0,
        }
    }

    /// Creates a route cost with only the length component.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::RouteCost;
    ///
    /// let cost = RouteCost::from_length(10.5);
    /// assert!((cost.length() - 10.5).abs() < 1e-10);
    /// ```
    #[must_use]
    pub const fn from_length(length: f64) -> Self {
        Self {
            length,
            bends: 0.0,
            curvature: 0.0,
            clearance: 0.0,
            corridor_deviation: 0.0,
        }
    }

    /// Sets the length cost component.
    #[must_use]
    pub const fn with_length(mut self, length: f64) -> Self {
        self.length = length;
        self
    }

    /// Sets the bends cost component.
    #[must_use]
    pub const fn with_bends(mut self, bends: f64) -> Self {
        self.bends = bends;
        self
    }

    /// Sets the curvature cost component.
    #[must_use]
    pub const fn with_curvature(mut self, curvature: f64) -> Self {
        self.curvature = curvature;
        self
    }

    /// Sets the clearance cost component.
    ///
    /// Typically negative to indicate good clearance (rewarded).
    #[must_use]
    pub const fn with_clearance(mut self, clearance: f64) -> Self {
        self.clearance = clearance;
        self
    }

    /// Sets the corridor deviation cost component.
    #[must_use]
    pub const fn with_corridor_deviation(mut self, deviation: f64) -> Self {
        self.corridor_deviation = deviation;
        self
    }

    /// Returns the length cost component.
    #[must_use]
    pub const fn length(&self) -> f64 {
        self.length
    }

    /// Returns the bends cost component.
    #[must_use]
    pub const fn bends(&self) -> f64 {
        self.bends
    }

    /// Returns the curvature cost component.
    #[must_use]
    pub const fn curvature(&self) -> f64 {
        self.curvature
    }

    /// Returns the clearance cost component.
    #[must_use]
    pub const fn clearance(&self) -> f64 {
        self.clearance
    }

    /// Returns the corridor deviation cost component.
    #[must_use]
    pub const fn corridor_deviation(&self) -> f64 {
        self.corridor_deviation
    }

    /// Computes the total weighted cost.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::{RouteCost, CostWeights};
    ///
    /// let cost = RouteCost::new()
    ///     .with_length(10.0)
    ///     .with_bends(2.0);
    ///
    /// let weights = CostWeights::zero()
    ///     .with_length(1.0)
    ///     .with_bends(0.5);
    ///
    /// let total = cost.compute_weighted(&weights);
    /// // 1.0 * 10.0 + 0.5 * 2.0 = 11.0
    /// assert!((total - 11.0).abs() < 1e-10);
    /// ```
    #[must_use]
    pub fn compute_weighted(&self, weights: &CostWeights) -> f64 {
        self.corridor_deviation.mul_add(
            weights.corridor_deviation,
            self.clearance.mul_add(
                weights.clearance,
                self.curvature.mul_add(
                    weights.curvature,
                    self.length
                        .mul_add(weights.length, self.bends * weights.bends),
                ),
            ),
        )
    }

    /// Returns the cost as an array for Pareto dominance comparisons.
    ///
    /// The array contains: `[length, bends, curvature, -clearance, corridor_deviation]`
    ///
    /// Note: clearance is negated because lower clearance is worse, but
    /// Pareto dominance assumes lower is better for all objectives.
    #[must_use]
    pub fn as_objectives(&self) -> [f64; 5] {
        [
            self.length,
            self.bends,
            self.curvature,
            -self.clearance, // Negated: high clearance is good
            self.corridor_deviation,
        ]
    }

    /// Adds another cost to this one (component-wise addition).
    ///
    /// Useful for accumulating costs during path construction.
    #[must_use]
    pub fn add(&self, other: &Self) -> Self {
        Self {
            length: self.length + other.length,
            bends: self.bends + other.bends,
            curvature: self.curvature + other.curvature,
            clearance: self.clearance + other.clearance,
            corridor_deviation: self.corridor_deviation + other.corridor_deviation,
        }
    }
}

impl std::ops::Add for RouteCost {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self::add(&self, &other)
    }
}

impl std::ops::AddAssign for RouteCost {
    fn add_assign(&mut self, other: Self) {
        self.length += other.length;
        self.bends += other.bends;
        self.curvature += other.curvature;
        self.clearance += other.clearance;
        self.corridor_deviation += other.corridor_deviation;
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    // ==================== CostWeights Tests ====================

    #[test]
    fn test_cost_weights_zero() {
        let w = CostWeights::zero();
        assert_relative_eq!(w.length(), 0.0, epsilon = 1e-10);
        assert_relative_eq!(w.bends(), 0.0, epsilon = 1e-10);
        assert_relative_eq!(w.curvature(), 0.0, epsilon = 1e-10);
        assert_relative_eq!(w.clearance(), 0.0, epsilon = 1e-10);
        assert_relative_eq!(w.corridor_deviation(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_cost_weights_length_only() {
        let w = CostWeights::length_only();
        assert_relative_eq!(w.length(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(w.bends(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_cost_weights_default() {
        let w = CostWeights::default();
        assert_relative_eq!(w.length(), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_cost_weights_builder() {
        let w = CostWeights::zero()
            .with_length(1.0)
            .with_bends(0.5)
            .with_curvature(0.3)
            .with_clearance(0.2)
            .with_corridor_deviation(0.1);

        assert_relative_eq!(w.length(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(w.bends(), 0.5, epsilon = 1e-10);
        assert_relative_eq!(w.curvature(), 0.3, epsilon = 1e-10);
        assert_relative_eq!(w.clearance(), 0.2, epsilon = 1e-10);
        assert_relative_eq!(w.corridor_deviation(), 0.1, epsilon = 1e-10);
    }

    #[test]
    fn test_cost_weights_normalized() {
        let w = CostWeights::zero()
            .with_length(2.0)
            .with_bends(3.0)
            .normalized();

        let sum = w.length() + w.bends() + w.curvature() + w.clearance() + w.corridor_deviation();
        assert_relative_eq!(sum, 1.0, epsilon = 1e-10);
        assert_relative_eq!(w.length(), 0.4, epsilon = 1e-10); // 2/5
        assert_relative_eq!(w.bends(), 0.6, epsilon = 1e-10); // 3/5
    }

    #[test]
    fn test_cost_weights_normalized_zero() {
        let w = CostWeights::zero().normalized();
        // When all weights are zero, defaults to length_only
        assert_relative_eq!(w.length(), 1.0, epsilon = 1e-10);
    }

    // ==================== RouteCost Tests ====================

    #[test]
    fn test_route_cost_new() {
        let c = RouteCost::new();
        assert_relative_eq!(c.length(), 0.0, epsilon = 1e-10);
        assert_relative_eq!(c.bends(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_route_cost_from_length() {
        let c = RouteCost::from_length(10.5);
        assert_relative_eq!(c.length(), 10.5, epsilon = 1e-10);
        assert_relative_eq!(c.bends(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_route_cost_builder() {
        let c = RouteCost::new()
            .with_length(10.0)
            .with_bends(3.0)
            .with_curvature(0.5)
            .with_clearance(-2.0)
            .with_corridor_deviation(1.0);

        assert_relative_eq!(c.length(), 10.0, epsilon = 1e-10);
        assert_relative_eq!(c.bends(), 3.0, epsilon = 1e-10);
        assert_relative_eq!(c.curvature(), 0.5, epsilon = 1e-10);
        assert_relative_eq!(c.clearance(), -2.0, epsilon = 1e-10);
        assert_relative_eq!(c.corridor_deviation(), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_route_cost_compute_weighted() {
        let cost = RouteCost::new().with_length(10.0).with_bends(2.0);

        let weights = CostWeights::zero().with_length(1.0).with_bends(0.5);

        let total = cost.compute_weighted(&weights);
        // 1.0 * 10.0 + 0.5 * 2.0 = 11.0
        assert_relative_eq!(total, 11.0, epsilon = 1e-10);
    }

    #[test]
    fn test_route_cost_compute_weighted_with_clearance() {
        let cost = RouteCost::new().with_length(10.0).with_clearance(-5.0); // Good clearance (negative)

        let weights = CostWeights::zero().with_length(1.0).with_clearance(1.0);

        let total = cost.compute_weighted(&weights);
        // 1.0 * 10.0 + 1.0 * (-5.0) = 5.0
        assert_relative_eq!(total, 5.0, epsilon = 1e-10);
    }

    #[test]
    fn test_route_cost_as_objectives() {
        let cost = RouteCost::new()
            .with_length(10.0)
            .with_bends(3.0)
            .with_curvature(0.5)
            .with_clearance(-2.0)
            .with_corridor_deviation(1.0);

        let obj = cost.as_objectives();
        assert_relative_eq!(obj[0], 10.0, epsilon = 1e-10); // length
        assert_relative_eq!(obj[1], 3.0, epsilon = 1e-10); // bends
        assert_relative_eq!(obj[2], 0.5, epsilon = 1e-10); // curvature
        assert_relative_eq!(obj[3], 2.0, epsilon = 1e-10); // -clearance (negated)
        assert_relative_eq!(obj[4], 1.0, epsilon = 1e-10); // corridor_deviation
    }

    #[test]
    fn test_route_cost_add() {
        let c1 = RouteCost::new().with_length(5.0).with_bends(2.0);
        let c2 = RouteCost::new().with_length(3.0).with_bends(1.0);

        let sum = c1.add(&c2);
        assert_relative_eq!(sum.length(), 8.0, epsilon = 1e-10);
        assert_relative_eq!(sum.bends(), 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_route_cost_add_operator() {
        let c1 = RouteCost::new().with_length(5.0);
        let c2 = RouteCost::new().with_length(3.0);

        let sum = c1 + c2;
        assert_relative_eq!(sum.length(), 8.0, epsilon = 1e-10);
    }

    #[test]
    fn test_route_cost_add_assign() {
        let mut c1 = RouteCost::new().with_length(5.0);
        let c2 = RouteCost::new().with_length(3.0);

        c1 += c2;
        assert_relative_eq!(c1.length(), 8.0, epsilon = 1e-10);
    }

    #[test]
    fn test_route_cost_default() {
        let c = RouteCost::default();
        assert_relative_eq!(c.length(), 0.0, epsilon = 1e-10);
    }
}
