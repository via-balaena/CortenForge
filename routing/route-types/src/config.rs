//! Configuration types for routing algorithms.
//!
//! This module defines configuration structures for various pathfinding algorithms.
//!
//! # Example
//!
//! ```
//! use route_types::{AStarConfig, Heuristic, CostWeights};
//! use std::time::Duration;
//!
//! let config = AStarConfig::default()
//!     .with_heuristic(Heuristic::Euclidean)
//!     .with_diagonal(true)
//!     .with_timeout(Duration::from_secs(30));
//! ```

use std::time::Duration;

use crate::cost::CostWeights;

/// Heuristic functions for A*-based algorithms.
///
/// The heuristic estimates the cost from a node to the goal.
/// For optimality, the heuristic must be admissible (never overestimate).
///
/// # Example
///
/// ```
/// use route_types::Heuristic;
///
/// // Manhattan is admissible for 6-connectivity (face neighbors only)
/// let h = Heuristic::Manhattan;
///
/// // Chebyshev is admissible for 26-connectivity (all neighbors)
/// let h = Heuristic::Chebyshev;
///
/// // Euclidean is always admissible
/// let h = Heuristic::Euclidean;
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Heuristic {
    /// Manhattan distance (L1 norm).
    ///
    /// Sum of absolute coordinate differences: |dx| + |dy| + |dz|
    ///
    /// Admissible for 6-connectivity (face neighbors only).
    Manhattan,

    /// Chebyshev distance (L-infinity norm).
    ///
    /// Maximum of absolute coordinate differences: max(|dx|, |dy|, |dz|)
    ///
    /// Admissible for 26-connectivity (all neighbors including diagonals).
    Chebyshev,

    /// Euclidean distance (L2 norm).
    ///
    /// Straight-line distance: sqrt(dx² + dy² + dz²)
    ///
    /// Always admissible, best for any-angle pathfinding.
    #[default]
    Euclidean,

    /// Octile distance (diagonal distance).
    ///
    /// Optimal for 8-connectivity in 2D, extended to 3D.
    /// Accounts for diagonal move costs properly.
    Octile,

    /// Zero heuristic (Dijkstra's algorithm).
    ///
    /// Always returns 0. Guarantees shortest path but explores more nodes.
    Zero,
}

impl Heuristic {
    /// Returns `true` if this heuristic is admissible for the given connectivity.
    ///
    /// # Arguments
    ///
    /// * `allow_diagonal` - Whether diagonal moves are allowed
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::Heuristic;
    ///
    /// assert!(Heuristic::Manhattan.is_admissible_for(false));  // 6-connectivity
    /// assert!(!Heuristic::Manhattan.is_admissible_for(true)); // Not for 26-connectivity
    /// assert!(Heuristic::Chebyshev.is_admissible_for(true));  // 26-connectivity
    /// assert!(Heuristic::Euclidean.is_admissible_for(true));  // Always
    /// ```
    #[must_use]
    pub const fn is_admissible_for(&self, allow_diagonal: bool) -> bool {
        match self {
            Self::Manhattan => !allow_diagonal, // Only for 6-connectivity
            Self::Chebyshev | Self::Octile | Self::Euclidean | Self::Zero => true,
        }
    }
}

/// Configuration for A* pathfinding.
///
/// Controls the behavior of the A* algorithm including heuristic selection,
/// connectivity, cost weights, and resource limits.
///
/// # Example
///
/// ```
/// use route_types::{AStarConfig, Heuristic, CostWeights};
/// use std::time::Duration;
///
/// // High-performance configuration for large environments
/// let config = AStarConfig::default()
///     .with_heuristic(Heuristic::Euclidean)
///     .with_diagonal(true)
///     .with_max_nodes(1_000_000)
///     .with_timeout(Duration::from_secs(60));
///
/// // Multi-objective configuration
/// let weighted_config = AStarConfig::default()
///     .with_weights(CostWeights::default()
///         .with_length(1.0)
///         .with_bends(0.3));
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AStarConfig {
    /// Heuristic function to use.
    heuristic: Heuristic,
    /// Whether to allow diagonal movement (26-connectivity vs 6-connectivity).
    allow_diagonal: bool,
    /// Cost weights for multi-objective pathfinding.
    weights: CostWeights,
    /// Maximum number of nodes to expand before giving up.
    max_nodes: Option<usize>,
    /// Maximum time before timeout.
    timeout: Option<Duration>,
    /// Whether to use tie-breaking for more consistent paths.
    tie_breaking: bool,
}

impl AStarConfig {
    /// Creates a new A* configuration with default settings.
    ///
    /// Defaults:
    /// - Heuristic: Euclidean
    /// - Diagonal: true (26-connectivity)
    /// - Weights: length only
    /// - No node limit
    /// - No timeout
    /// - Tie-breaking: enabled
    #[must_use]
    pub fn new() -> Self {
        Self {
            heuristic: Heuristic::Euclidean,
            allow_diagonal: true,
            weights: CostWeights::default(),
            max_nodes: None,
            timeout: None,
            tie_breaking: true,
        }
    }

    /// Sets the heuristic function.
    #[must_use]
    pub const fn with_heuristic(mut self, heuristic: Heuristic) -> Self {
        self.heuristic = heuristic;
        self
    }

    /// Sets whether diagonal movement is allowed.
    ///
    /// - `true`: 26-connectivity (face, edge, and corner neighbors)
    /// - `false`: 6-connectivity (face neighbors only)
    #[must_use]
    pub const fn with_diagonal(mut self, allow: bool) -> Self {
        self.allow_diagonal = allow;
        self
    }

    /// Sets the cost weights for multi-objective pathfinding.
    #[must_use]
    pub const fn with_weights(mut self, weights: CostWeights) -> Self {
        self.weights = weights;
        self
    }

    /// Sets the maximum number of nodes to expand.
    ///
    /// If this limit is reached, pathfinding will fail with a timeout error.
    #[must_use]
    pub const fn with_max_nodes(mut self, max: usize) -> Self {
        self.max_nodes = Some(max);
        self
    }

    /// Removes the node limit.
    #[must_use]
    pub const fn without_max_nodes(mut self) -> Self {
        self.max_nodes = None;
        self
    }

    /// Sets the timeout duration.
    #[must_use]
    pub const fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = Some(timeout);
        self
    }

    /// Removes the timeout.
    #[must_use]
    pub const fn without_timeout(mut self) -> Self {
        self.timeout = None;
        self
    }

    /// Sets whether to use tie-breaking.
    ///
    /// Tie-breaking helps produce more consistent, direct paths when
    /// multiple paths have the same cost.
    #[must_use]
    pub const fn with_tie_breaking(mut self, enable: bool) -> Self {
        self.tie_breaking = enable;
        self
    }

    /// Returns the heuristic function.
    #[must_use]
    pub const fn heuristic(&self) -> Heuristic {
        self.heuristic
    }

    /// Returns whether diagonal movement is allowed.
    #[must_use]
    pub const fn allow_diagonal(&self) -> bool {
        self.allow_diagonal
    }

    /// Returns the cost weights.
    #[must_use]
    pub const fn weights(&self) -> &CostWeights {
        &self.weights
    }

    /// Returns the maximum number of nodes, if set.
    #[must_use]
    pub const fn max_nodes(&self) -> Option<usize> {
        self.max_nodes
    }

    /// Returns the timeout duration, if set.
    #[must_use]
    pub const fn timeout(&self) -> Option<Duration> {
        self.timeout
    }

    /// Returns whether tie-breaking is enabled.
    #[must_use]
    pub const fn tie_breaking(&self) -> bool {
        self.tie_breaking
    }

    /// Validates the configuration and returns any issues.
    ///
    /// Checks for potentially problematic combinations like
    /// using Manhattan heuristic with diagonal movement.
    #[must_use]
    pub fn validate(&self) -> Vec<String> {
        let mut issues = Vec::new();

        if !self.heuristic.is_admissible_for(self.allow_diagonal) {
            issues.push(format!(
                "Heuristic {:?} is not admissible for diagonal={}",
                self.heuristic, self.allow_diagonal
            ));
        }

        issues
    }
}

impl Default for AStarConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Configuration for Theta* (any-angle pathfinding).
///
/// Theta* extends A* by allowing paths to take any angle (not just
/// grid-aligned directions) when line-of-sight exists.
///
/// # Example
///
/// ```
/// use route_types::{ThetaStarConfig, AStarConfig, Heuristic};
///
/// let config = ThetaStarConfig::default()
///     .with_los_resolution(0.5);
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ThetaStarConfig {
    /// Base A* configuration.
    base: AStarConfig,
    /// Resolution for line-of-sight checks (smaller = more precise but slower).
    los_resolution: f64,
}

impl ThetaStarConfig {
    /// Creates a new Theta* configuration.
    #[must_use]
    pub fn new() -> Self {
        Self {
            base: AStarConfig::new().with_heuristic(Heuristic::Euclidean),
            los_resolution: 0.5,
        }
    }

    /// Sets the base A* configuration.
    #[must_use]
    pub const fn with_base(mut self, base: AStarConfig) -> Self {
        self.base = base;
        self
    }

    /// Sets the line-of-sight check resolution.
    ///
    /// Smaller values are more precise but slower.
    /// Default is 0.5 (half voxel size).
    #[must_use]
    pub const fn with_los_resolution(mut self, resolution: f64) -> Self {
        self.los_resolution = resolution;
        self
    }

    /// Returns the base A* configuration.
    #[must_use]
    pub const fn base(&self) -> &AStarConfig {
        &self.base
    }

    /// Returns a mutable reference to the base configuration.
    pub const fn base_mut(&mut self) -> &mut AStarConfig {
        &mut self.base
    }

    /// Returns the line-of-sight resolution.
    #[must_use]
    pub const fn los_resolution(&self) -> f64 {
        self.los_resolution
    }
}

impl Default for ThetaStarConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Configuration for RRT* (Rapidly-exploring Random Trees Star).
///
/// RRT* is a sampling-based algorithm suitable for high-dimensional
/// spaces or continuous planning problems.
///
/// # Example
///
/// ```
/// use route_types::RrtStarConfig;
///
/// let config = RrtStarConfig::default()
///     .with_max_iterations(10_000)
///     .with_step_size(0.5)
///     .with_goal_bias(0.1);
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RrtStarConfig {
    /// Maximum number of iterations.
    max_iterations: usize,
    /// Step size for tree expansion (in world units).
    step_size: f64,
    /// Probability of sampling the goal directly (0-1).
    goal_bias: f64,
    /// Radius for rewiring (in world units).
    rewire_radius: f64,
    /// Random seed for reproducibility (None = random).
    seed: Option<u64>,
    /// Timeout duration.
    timeout: Option<Duration>,
}

impl RrtStarConfig {
    /// Creates a new RRT* configuration with default settings.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            max_iterations: 10_000,
            step_size: 1.0,
            goal_bias: 0.05,
            rewire_radius: 2.0,
            seed: None,
            timeout: None,
        }
    }

    /// Sets the maximum number of iterations.
    #[must_use]
    pub const fn with_max_iterations(mut self, max: usize) -> Self {
        self.max_iterations = max;
        self
    }

    /// Sets the step size for tree expansion.
    #[must_use]
    pub const fn with_step_size(mut self, size: f64) -> Self {
        self.step_size = size;
        self
    }

    /// Sets the goal bias probability (0-1).
    ///
    /// Higher values make the tree grow toward the goal faster
    /// but may miss better paths.
    #[must_use]
    pub const fn with_goal_bias(mut self, bias: f64) -> Self {
        self.goal_bias = bias;
        self
    }

    /// Sets the rewiring radius.
    ///
    /// Larger values find shorter paths but are slower.
    #[must_use]
    pub const fn with_rewire_radius(mut self, radius: f64) -> Self {
        self.rewire_radius = radius;
        self
    }

    /// Sets the random seed for reproducibility.
    #[must_use]
    pub const fn with_seed(mut self, seed: u64) -> Self {
        self.seed = Some(seed);
        self
    }

    /// Removes the seed (use random initialization).
    #[must_use]
    pub const fn without_seed(mut self) -> Self {
        self.seed = None;
        self
    }

    /// Sets the timeout duration.
    #[must_use]
    pub const fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = Some(timeout);
        self
    }

    /// Returns the maximum number of iterations.
    #[must_use]
    pub const fn max_iterations(&self) -> usize {
        self.max_iterations
    }

    /// Returns the step size.
    #[must_use]
    pub const fn step_size(&self) -> f64 {
        self.step_size
    }

    /// Returns the goal bias.
    #[must_use]
    pub const fn goal_bias(&self) -> f64 {
        self.goal_bias
    }

    /// Returns the rewiring radius.
    #[must_use]
    pub const fn rewire_radius(&self) -> f64 {
        self.rewire_radius
    }

    /// Returns the seed, if set.
    #[must_use]
    pub const fn seed(&self) -> Option<u64> {
        self.seed
    }

    /// Returns the timeout, if set.
    #[must_use]
    pub const fn timeout(&self) -> Option<Duration> {
        self.timeout
    }
}

impl Default for RrtStarConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    // ==================== Heuristic Tests ====================

    #[test]
    fn test_heuristic_default() {
        let h = Heuristic::default();
        assert_eq!(h, Heuristic::Euclidean);
    }

    #[test]
    fn test_heuristic_admissibility() {
        // Manhattan: only 6-connectivity
        assert!(Heuristic::Manhattan.is_admissible_for(false));
        assert!(!Heuristic::Manhattan.is_admissible_for(true));

        // Chebyshev: both
        assert!(Heuristic::Chebyshev.is_admissible_for(false));
        assert!(Heuristic::Chebyshev.is_admissible_for(true));

        // Euclidean: both
        assert!(Heuristic::Euclidean.is_admissible_for(false));
        assert!(Heuristic::Euclidean.is_admissible_for(true));

        // Zero: always admissible
        assert!(Heuristic::Zero.is_admissible_for(false));
        assert!(Heuristic::Zero.is_admissible_for(true));
    }

    // ==================== AStarConfig Tests ====================

    #[test]
    fn test_astar_config_default() {
        let config = AStarConfig::default();
        assert_eq!(config.heuristic(), Heuristic::Euclidean);
        assert!(config.allow_diagonal());
        assert!(config.max_nodes().is_none());
        assert!(config.timeout().is_none());
        assert!(config.tie_breaking());
    }

    #[test]
    fn test_astar_config_builder() {
        let config = AStarConfig::new()
            .with_heuristic(Heuristic::Manhattan)
            .with_diagonal(false)
            .with_max_nodes(10_000)
            .with_timeout(Duration::from_secs(30))
            .with_tie_breaking(false);

        assert_eq!(config.heuristic(), Heuristic::Manhattan);
        assert!(!config.allow_diagonal());
        assert_eq!(config.max_nodes(), Some(10_000));
        assert_eq!(config.timeout(), Some(Duration::from_secs(30)));
        assert!(!config.tie_breaking());
    }

    #[test]
    fn test_astar_config_with_weights() {
        let config = AStarConfig::new().with_weights(CostWeights::default().with_bends(0.5));

        assert_relative_eq!(config.weights().bends(), 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_astar_config_without_limits() {
        let config = AStarConfig::new()
            .with_max_nodes(1000)
            .with_timeout(Duration::from_secs(10))
            .without_max_nodes()
            .without_timeout();

        assert!(config.max_nodes().is_none());
        assert!(config.timeout().is_none());
    }

    #[test]
    fn test_astar_config_validate_admissible() {
        let config = AStarConfig::new()
            .with_heuristic(Heuristic::Euclidean)
            .with_diagonal(true);

        let issues = config.validate();
        assert!(issues.is_empty());
    }

    #[test]
    fn test_astar_config_validate_inadmissible() {
        let config = AStarConfig::new()
            .with_heuristic(Heuristic::Manhattan)
            .with_diagonal(true);

        let issues = config.validate();
        assert!(!issues.is_empty());
    }

    // ==================== ThetaStarConfig Tests ====================

    #[test]
    fn test_theta_star_config_default() {
        let config = ThetaStarConfig::default();
        assert_eq!(config.base().heuristic(), Heuristic::Euclidean);
        assert_relative_eq!(config.los_resolution(), 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_theta_star_config_builder() {
        let config = ThetaStarConfig::new()
            .with_los_resolution(0.25)
            .with_base(AStarConfig::new().with_max_nodes(5000));

        assert_relative_eq!(config.los_resolution(), 0.25, epsilon = 1e-10);
        assert_eq!(config.base().max_nodes(), Some(5000));
    }

    #[test]
    fn test_theta_star_config_base_mut() {
        let mut config = ThetaStarConfig::new();
        config.base_mut().max_nodes = Some(1000);

        assert_eq!(config.base().max_nodes(), Some(1000));
    }

    // ==================== RrtStarConfig Tests ====================

    #[test]
    fn test_rrt_star_config_default() {
        let config = RrtStarConfig::default();
        assert_eq!(config.max_iterations(), 10_000);
        assert_relative_eq!(config.step_size(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(config.goal_bias(), 0.05, epsilon = 1e-10);
        assert_relative_eq!(config.rewire_radius(), 2.0, epsilon = 1e-10);
        assert!(config.seed().is_none());
        assert!(config.timeout().is_none());
    }

    #[test]
    fn test_rrt_star_config_builder() {
        let config = RrtStarConfig::new()
            .with_max_iterations(50_000)
            .with_step_size(0.5)
            .with_goal_bias(0.1)
            .with_rewire_radius(3.0)
            .with_seed(42)
            .with_timeout(Duration::from_secs(120));

        assert_eq!(config.max_iterations(), 50_000);
        assert_relative_eq!(config.step_size(), 0.5, epsilon = 1e-10);
        assert_relative_eq!(config.goal_bias(), 0.1, epsilon = 1e-10);
        assert_relative_eq!(config.rewire_radius(), 3.0, epsilon = 1e-10);
        assert_eq!(config.seed(), Some(42));
        assert_eq!(config.timeout(), Some(Duration::from_secs(120)));
    }

    #[test]
    fn test_rrt_star_config_without_seed() {
        let config = RrtStarConfig::new().with_seed(42).without_seed();

        assert!(config.seed().is_none());
    }
}
