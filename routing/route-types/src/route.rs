//! Route types combining paths with metadata.
//!
//! This module defines the [`Route`] type which represents a complete
//! routing solution including the path, computed costs, and statistics.
//!
//! # Example
//!
//! ```
//! use route_types::{Route, VoxelPath, RouteCost, RouteStats, Path};
//! use cf_spatial::VoxelCoord;
//!
//! let path = VoxelPath::new(vec![
//!     VoxelCoord::new(0, 0, 0),
//!     VoxelCoord::new(1, 0, 0),
//!     VoxelCoord::new(2, 0, 0),
//! ]);
//!
//! let route = Route::new(Path::Voxel(path))
//!     .with_cost(RouteCost::from_length(2.0));
//!
//! assert!(route.is_valid());
//! ```

use std::time::Duration;

use crate::cost::RouteCost;
use crate::path::Path;

/// Statistics about the routing process.
///
/// Contains information about the pathfinding computation
/// such as time taken and nodes explored.
///
/// # Example
///
/// ```
/// use route_types::RouteStats;
/// use std::time::Duration;
///
/// let stats = RouteStats::new("A*")
///     .with_nodes_expanded(1500)
///     .with_elapsed(Duration::from_millis(50));
/// ```
#[derive(Debug, Clone, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RouteStats {
    /// Number of nodes expanded during search.
    nodes_expanded: usize,
    /// Number of nodes in the open set at completion.
    open_set_size: usize,
    /// Time taken for pathfinding.
    time_elapsed: Duration,
    /// Name of the algorithm used.
    algorithm: String,
}

impl RouteStats {
    /// Creates new route statistics with the given algorithm name.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::RouteStats;
    ///
    /// let stats = RouteStats::new("Theta*");
    /// assert_eq!(stats.algorithm(), "Theta*");
    /// ```
    #[must_use]
    pub fn new(algorithm: impl Into<String>) -> Self {
        Self {
            nodes_expanded: 0,
            open_set_size: 0,
            time_elapsed: Duration::ZERO,
            algorithm: algorithm.into(),
        }
    }

    /// Sets the number of nodes expanded.
    #[must_use]
    pub const fn with_nodes_expanded(mut self, count: usize) -> Self {
        self.nodes_expanded = count;
        self
    }

    /// Sets the open set size at completion.
    #[must_use]
    pub const fn with_open_set_size(mut self, size: usize) -> Self {
        self.open_set_size = size;
        self
    }

    /// Sets the time elapsed.
    #[must_use]
    pub const fn with_elapsed(mut self, duration: Duration) -> Self {
        self.time_elapsed = duration;
        self
    }

    /// Returns the number of nodes expanded during search.
    #[must_use]
    pub const fn nodes_expanded(&self) -> usize {
        self.nodes_expanded
    }

    /// Returns the open set size at completion.
    #[must_use]
    pub const fn open_set_size(&self) -> usize {
        self.open_set_size
    }

    /// Returns the time taken for pathfinding.
    #[must_use]
    pub const fn time_elapsed(&self) -> Duration {
        self.time_elapsed
    }

    /// Returns the name of the algorithm used.
    #[must_use]
    pub fn algorithm(&self) -> &str {
        &self.algorithm
    }
}

/// A complete route solution with metadata.
///
/// Combines a path with cost information, constraint satisfaction status,
/// and statistics about the routing process.
///
/// # Example
///
/// ```
/// use route_types::{Route, VoxelPath, RouteCost, RouteStats, Path};
/// use cf_spatial::VoxelCoord;
/// use std::time::Duration;
///
/// let path = VoxelPath::new(vec![
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(5, 0, 0),
///     VoxelCoord::new(10, 0, 0),
/// ]);
///
/// let route = Route::new(Path::Voxel(path))
///     .with_cost(RouteCost::from_length(10.0))
///     .with_stats(RouteStats::new("A*").with_nodes_expanded(100))
///     .with_constraints_satisfied(true);
///
/// assert!(route.is_valid());
/// assert!((route.length() - 10.0).abs() < 1e-10);
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Route {
    /// The path geometry.
    path: Path,
    /// Computed cost breakdown.
    cost: RouteCost,
    /// Statistics about the routing process.
    stats: RouteStats,
    /// Whether all hard constraints are satisfied.
    constraints_satisfied: bool,
}

impl Route {
    /// Creates a new route from a path.
    ///
    /// Cost and stats are initialized to defaults.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::{Route, VoxelPath, Path};
    /// use cf_spatial::VoxelCoord;
    ///
    /// let path = VoxelPath::from_single(VoxelCoord::origin());
    /// let route = Route::new(Path::Voxel(path));
    /// ```
    #[must_use]
    pub fn new(path: Path) -> Self {
        Self {
            path,
            cost: RouteCost::new(),
            stats: RouteStats::default(),
            constraints_satisfied: true,
        }
    }

    /// Creates a route from a voxel path.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::{Route, VoxelPath};
    /// use cf_spatial::VoxelCoord;
    ///
    /// let path = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(1, 0, 0),
    /// ]);
    /// let route = Route::from_voxel_path(path);
    /// assert!(route.path().is_voxel());
    /// ```
    #[must_use]
    pub fn from_voxel_path(path: crate::path::VoxelPath) -> Self {
        Self::new(Path::Voxel(path))
    }

    /// Creates a route from a continuous path.
    #[must_use]
    pub fn from_continuous_path(path: crate::path::ContinuousPath) -> Self {
        Self::new(Path::Continuous(path))
    }

    /// Sets the cost breakdown.
    #[must_use]
    pub const fn with_cost(mut self, cost: RouteCost) -> Self {
        self.cost = cost;
        self
    }

    /// Sets the route statistics.
    #[must_use]
    pub fn with_stats(mut self, stats: RouteStats) -> Self {
        self.stats = stats;
        self
    }

    /// Sets whether constraints are satisfied.
    #[must_use]
    pub const fn with_constraints_satisfied(mut self, satisfied: bool) -> Self {
        self.constraints_satisfied = satisfied;
        self
    }

    /// Returns the path.
    #[must_use]
    pub const fn path(&self) -> &Path {
        &self.path
    }

    /// Returns a mutable reference to the path.
    pub const fn path_mut(&mut self) -> &mut Path {
        &mut self.path
    }

    /// Returns the cost breakdown.
    #[must_use]
    pub const fn cost(&self) -> &RouteCost {
        &self.cost
    }

    /// Returns a mutable reference to the cost.
    pub const fn cost_mut(&mut self) -> &mut RouteCost {
        &mut self.cost
    }

    /// Returns the route statistics.
    #[must_use]
    pub const fn stats(&self) -> &RouteStats {
        &self.stats
    }

    /// Returns a mutable reference to the stats.
    pub const fn stats_mut(&mut self) -> &mut RouteStats {
        &mut self.stats
    }

    /// Returns whether all constraints are satisfied.
    #[must_use]
    pub const fn constraints_satisfied(&self) -> bool {
        self.constraints_satisfied
    }

    /// Returns the path length (from the path geometry).
    #[must_use]
    pub const fn length(&self) -> f64 {
        self.path.length()
    }

    /// Returns the number of nodes/waypoints in the path.
    #[must_use]
    pub fn node_count(&self) -> usize {
        self.path.len()
    }

    /// Returns `true` if the route is valid (has a path and satisfies constraints).
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::{Route, VoxelPath, Path};
    /// use cf_spatial::VoxelCoord;
    ///
    /// let path = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(1, 0, 0),
    /// ]);
    /// let route = Route::new(Path::Voxel(path));
    /// assert!(route.is_valid());
    ///
    /// let invalid_route = Route::new(Path::Voxel(VoxelPath::empty()))
    ///     .with_constraints_satisfied(false);
    /// assert!(!invalid_route.is_valid());
    /// ```
    #[must_use]
    pub fn is_valid(&self) -> bool {
        !self.path.is_empty() && self.constraints_satisfied
    }

    /// Consumes the route and returns the path.
    #[must_use]
    pub fn into_path(self) -> Path {
        self.path
    }

    /// Consumes the route and returns its components.
    #[must_use]
    pub fn into_parts(self) -> (Path, RouteCost, RouteStats) {
        (self.path, self.cost, self.stats)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::path::{ContinuousPath, VoxelPath};
    use approx::assert_relative_eq;
    use cf_spatial::VoxelCoord;
    use nalgebra::Point3;

    // ==================== RouteStats Tests ====================

    #[test]
    fn test_route_stats_new() {
        let stats = RouteStats::new("A*");
        assert_eq!(stats.algorithm(), "A*");
        assert_eq!(stats.nodes_expanded(), 0);
        assert_eq!(stats.time_elapsed(), Duration::ZERO);
    }

    #[test]
    fn test_route_stats_builder() {
        let stats = RouteStats::new("Theta*")
            .with_nodes_expanded(1500)
            .with_open_set_size(200)
            .with_elapsed(Duration::from_millis(50));

        assert_eq!(stats.algorithm(), "Theta*");
        assert_eq!(stats.nodes_expanded(), 1500);
        assert_eq!(stats.open_set_size(), 200);
        assert_eq!(stats.time_elapsed(), Duration::from_millis(50));
    }

    #[test]
    fn test_route_stats_default() {
        let stats = RouteStats::default();
        assert!(stats.algorithm().is_empty());
        assert_eq!(stats.nodes_expanded(), 0);
    }

    // ==================== Route Tests ====================

    #[test]
    fn test_route_new() {
        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 0, 0)]);
        let route = Route::new(Path::Voxel(path));

        assert!(route.path().is_voxel());
        assert!(route.constraints_satisfied());
        assert!(route.is_valid());
    }

    #[test]
    fn test_route_from_voxel_path() {
        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 0, 0)]);
        let route = Route::from_voxel_path(path);
        assert!(route.path().is_voxel());
    }

    #[test]
    fn test_route_from_continuous_path() {
        let path = ContinuousPath::from_points(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        ]);
        let route = Route::from_continuous_path(path);
        assert!(route.path().is_continuous());
    }

    #[test]
    fn test_route_with_cost() {
        let path = VoxelPath::from_single(VoxelCoord::origin());
        let route = Route::new(Path::Voxel(path)).with_cost(RouteCost::from_length(10.0));

        assert_relative_eq!(route.cost().length(), 10.0, epsilon = 1e-10);
    }

    #[test]
    fn test_route_with_stats() {
        let path = VoxelPath::from_single(VoxelCoord::origin());
        let route = Route::new(Path::Voxel(path))
            .with_stats(RouteStats::new("A*").with_nodes_expanded(500));

        assert_eq!(route.stats().algorithm(), "A*");
        assert_eq!(route.stats().nodes_expanded(), 500);
    }

    #[test]
    fn test_route_with_constraints_satisfied() {
        let path = VoxelPath::from_single(VoxelCoord::origin());
        let route = Route::new(Path::Voxel(path)).with_constraints_satisfied(false);

        assert!(!route.constraints_satisfied());
        assert!(!route.is_valid());
    }

    #[test]
    fn test_route_length() {
        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(5, 0, 0)]);
        let route = Route::from_voxel_path(path);

        assert_relative_eq!(route.length(), 5.0, epsilon = 1e-10);
    }

    #[test]
    fn test_route_node_count() {
        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
        ]);
        let route = Route::from_voxel_path(path);

        assert_eq!(route.node_count(), 3);
    }

    #[test]
    fn test_route_is_valid_empty_path() {
        let path = VoxelPath::empty();
        let route = Route::from_voxel_path(path);

        assert!(!route.is_valid());
    }

    #[test]
    fn test_route_is_valid_constraints_not_satisfied() {
        let path = VoxelPath::from_single(VoxelCoord::origin());
        let route = Route::from_voxel_path(path).with_constraints_satisfied(false);

        assert!(!route.is_valid());
    }

    #[test]
    fn test_route_is_valid_success() {
        let path = VoxelPath::from_single(VoxelCoord::origin());
        let route = Route::from_voxel_path(path).with_constraints_satisfied(true);

        assert!(route.is_valid());
    }

    #[test]
    fn test_route_path_mut() {
        let path = VoxelPath::from_single(VoxelCoord::origin());
        let mut route = Route::from_voxel_path(path);

        // Can modify the path
        let _ = route.path_mut();
    }

    #[test]
    fn test_route_cost_mut() {
        let path = VoxelPath::from_single(VoxelCoord::origin());
        let mut route = Route::from_voxel_path(path);

        *route.cost_mut() = RouteCost::from_length(5.0);
        assert_relative_eq!(route.cost().length(), 5.0, epsilon = 1e-10);
    }

    #[test]
    fn test_route_stats_mut() {
        let path = VoxelPath::from_single(VoxelCoord::origin());
        let mut route = Route::from_voxel_path(path);

        *route.stats_mut() = RouteStats::new("Modified");
        assert_eq!(route.stats().algorithm(), "Modified");
    }

    #[test]
    fn test_route_into_path() {
        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 0, 0)]);
        let route = Route::from_voxel_path(path);

        let extracted = route.into_path();
        assert!(extracted.is_voxel());
        assert_eq!(extracted.len(), 2);
    }

    #[test]
    fn test_route_into_parts() {
        let path = VoxelPath::from_single(VoxelCoord::origin());
        let route = Route::from_voxel_path(path)
            .with_cost(RouteCost::from_length(10.0))
            .with_stats(RouteStats::new("A*"));

        let (path, cost, stats) = route.into_parts();
        assert!(path.is_voxel());
        assert_relative_eq!(cost.length(), 10.0, epsilon = 1e-10);
        assert_eq!(stats.algorithm(), "A*");
    }
}
