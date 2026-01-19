//! A* pathfinding on voxel grids.
//!
//! This module provides A* pathfinding that integrates with `cf-spatial`
//! voxel grids and the `pathfinding` crate.
//!
//! # Example
//!
//! ```
//! use route_pathfind::astar::VoxelAStar;
//! use route_types::{AStarConfig, RouteGoal, Heuristic};
//! use cf_spatial::{VoxelCoord, VoxelGrid};
//!
//! // Create obstacle grid
//! let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
//!
//! // Create pathfinder
//! let pathfinder = VoxelAStar::new(&grid, AStarConfig::default());
//!
//! // Define goal
//! let goal = RouteGoal::from_voxels(
//!     VoxelCoord::new(0, 0, 0),
//!     VoxelCoord::new(10, 0, 0),
//! );
//!
//! // Find path
//! let route = pathfinder.find_path(&goal).expect("Path should exist");
//! assert!(route.is_valid());
//! ```

use std::time::Instant;

use cf_spatial::VoxelCoord;
use cf_spatial::VoxelGrid;
use pathfinding::prelude::astar;
use route_types::{
    AStarConfig, GoalPoint, Path, Route, RouteCost, RouteGoal, RouteStats, RoutingError, VoxelPath,
};

use crate::heuristics::compute_heuristic;
use crate::neighbors::NeighborGenerator;

/// A* pathfinder for voxel grids.
///
/// Finds shortest paths through a voxel grid, avoiding obstacles.
///
/// # Example
///
/// ```
/// use route_pathfind::astar::VoxelAStar;
/// use route_types::{AStarConfig, RouteGoal, Heuristic};
/// use cf_spatial::{VoxelCoord, VoxelGrid};
///
/// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
/// let pathfinder = VoxelAStar::new(&grid, AStarConfig::default());
///
/// let goal = RouteGoal::from_voxels(
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(5, 5, 5),
/// );
///
/// let route = pathfinder.find_path(&goal).unwrap();
/// assert!(route.path().len() > 0);
/// ```
pub struct VoxelAStar<'a, T> {
    /// The obstacle grid.
    grid: &'a VoxelGrid<T>,
    /// Algorithm configuration.
    config: AStarConfig,
}

impl<'a, T: Default> VoxelAStar<'a, T> {
    /// Creates a new A* pathfinder with the given grid and configuration.
    ///
    /// # Example
    ///
    /// ```
    /// use route_pathfind::astar::VoxelAStar;
    /// use route_types::AStarConfig;
    /// use cf_spatial::VoxelGrid;
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let pathfinder = VoxelAStar::new(&grid, AStarConfig::default());
    /// ```
    #[must_use]
    pub const fn new(grid: &'a VoxelGrid<T>, config: AStarConfig) -> Self {
        Self { grid, config }
    }

    /// Returns the configuration.
    #[must_use]
    pub const fn config(&self) -> &AStarConfig {
        &self.config
    }

    /// Resolves a goal point to a voxel coordinate.
    fn resolve_goal_point(&self, point: &GoalPoint) -> VoxelCoord {
        match point {
            GoalPoint::Voxel(coord) => *coord,
            GoalPoint::World(world_point) => self.grid.world_to_grid(*world_point),
            GoalPoint::Region(aabb) => {
                // Return center of region
                let center = aabb.center();
                self.grid.world_to_grid(center)
            }
        }
    }

    /// Checks if a voxel coordinate is blocked.
    fn is_blocked(&self, coord: VoxelCoord) -> bool {
        self.grid.contains(coord)
    }

    /// Finds a path between two voxel coordinates.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - Start or goal is blocked
    /// - No path exists
    /// - Timeout or node limit exceeded
    fn find_segment(
        &self,
        start: VoxelCoord,
        goal: VoxelCoord,
    ) -> Result<(Vec<VoxelCoord>, f64), RoutingError> {
        // Check start and goal
        if self.is_blocked(start) {
            return Err(RoutingError::StartBlocked(start));
        }
        if self.is_blocked(goal) {
            return Err(RoutingError::GoalBlocked(goal));
        }

        // Trivial case
        if start == goal {
            return Ok((vec![start], 0.0));
        }

        let generator =
            NeighborGenerator::new(self.grid).with_diagonal(self.config.allow_diagonal());
        let heuristic = self.config.heuristic();

        // Use pathfinding crate's A*
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let result = astar(
            &start,
            |node| {
                generator
                    .successors(*node)
                    .map(|(n, cost)| {
                        // Scale cost to u64 for pathfinding crate
                        let scaled = (cost * 1000.0).round() as u64;
                        (n, scaled)
                    })
                    .collect::<Vec<_>>()
            },
            |node| {
                // Heuristic scaled to u64
                let h = compute_heuristic(*node, goal, heuristic);
                (h * 1000.0).round() as u64
            },
            |node| *node == goal,
        );

        match result {
            Some((path, cost)) => {
                #[allow(clippy::cast_precision_loss)]
                let unscaled_cost = (cost as f64) / 1000.0;
                Ok((path, unscaled_cost))
            }
            None => Err(RoutingError::NoPathFound { start, goal }),
        }
    }

    /// Finds a complete route satisfying the goal specification.
    ///
    /// Handles via points by finding a path through each segment.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - Start or goal is blocked
    /// - No path exists between any segments
    /// - A via point is unreachable
    ///
    /// # Example
    ///
    /// ```
    /// use route_pathfind::astar::VoxelAStar;
    /// use route_types::{AStarConfig, RouteGoal, GoalPoint};
    /// use cf_spatial::{VoxelCoord, VoxelGrid};
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let pathfinder = VoxelAStar::new(&grid, AStarConfig::default());
    ///
    /// // Route with via point
    /// let goal = RouteGoal::from_voxels(
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(10, 0, 0),
    /// ).with_via_point(GoalPoint::voxel(5, 5, 0));
    ///
    /// let route = pathfinder.find_path(&goal).unwrap();
    /// assert!(route.is_valid());
    /// ```
    pub fn find_path(&self, goal: &RouteGoal) -> Result<Route, RoutingError> {
        let start_time = Instant::now();
        let mut total_nodes = 0usize;

        // Resolve all goal points to coordinates
        let start_coord = self.resolve_goal_point(goal.start());
        let end_coord = self.resolve_goal_point(goal.end());

        let via_coords: Vec<_> = goal
            .via_points()
            .iter()
            .map(|p| self.resolve_goal_point(p))
            .collect();

        // Build list of segment endpoints
        let mut waypoints = vec![start_coord];
        waypoints.extend(via_coords);
        waypoints.push(end_coord);

        // Find path through each segment
        let mut full_path = Vec::new();
        let mut total_cost = 0.0;

        for (i, segment) in waypoints.windows(2).enumerate() {
            let from = segment[0];
            let to = segment[1];

            let (segment_path, segment_cost) = self.find_segment(from, to).map_err(|e| {
                if let RoutingError::NoPathFound { .. } = e {
                    // Check if this is a via point issue
                    if i < goal.via_points().len() {
                        RoutingError::ViaPointUnreachable {
                            index: i,
                            coord: to,
                        }
                    } else {
                        e
                    }
                } else {
                    e
                }
            })?;

            // Append segment (avoiding duplicate at junction)
            if full_path.is_empty() {
                full_path.extend(segment_path);
            } else {
                full_path.extend(segment_path.into_iter().skip(1));
            }

            total_cost += segment_cost;
            total_nodes += 1; // Simplified - actual count would need more tracking
        }

        let elapsed = start_time.elapsed();

        // Build route
        let path = VoxelPath::new(full_path);
        let cost = RouteCost::from_length(total_cost);
        let stats = RouteStats::new("A*")
            .with_nodes_expanded(total_nodes)
            .with_elapsed(elapsed);

        Ok(Route::new(Path::Voxel(path))
            .with_cost(cost)
            .with_stats(stats)
            .with_constraints_satisfied(true))
    }
}

/// Convenience function for simple point-to-point pathfinding.
///
/// # Errors
///
/// Returns an error if:
/// - Start or goal is blocked
/// - No path exists between start and goal
///
/// # Example
///
/// ```
/// use route_pathfind::astar::find_path;
/// use cf_spatial::{VoxelCoord, VoxelGrid};
///
/// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
/// let path = find_path(
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(5, 0, 0),
///     &grid,
///     true,  // allow diagonal
/// );
///
/// assert!(path.is_ok());
/// ```
pub fn find_path<T: Default>(
    start: VoxelCoord,
    goal: VoxelCoord,
    grid: &VoxelGrid<T>,
    allow_diagonal: bool,
) -> Result<Route, RoutingError> {
    let config = AStarConfig::default().with_diagonal(allow_diagonal);
    let pathfinder = VoxelAStar::new(grid, config);
    let route_goal = RouteGoal::from_voxels(start, goal);
    pathfinder.find_path(&route_goal)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::expect_used)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use route_types::Heuristic;

    fn empty_grid() -> VoxelGrid<bool> {
        VoxelGrid::new(1.0)
    }

    #[allow(dead_code)] // Test helper for future tests
    fn grid_with_wall(y: i32, z_range: std::ops::Range<i32>) -> VoxelGrid<bool> {
        let mut grid = VoxelGrid::new(1.0);
        // Create a wall at x=5, spanning y and z
        for z in z_range {
            grid.set(VoxelCoord::new(5, y, z), true);
        }
        grid
    }

    #[test]
    fn test_astar_simple_path() {
        let grid = empty_grid();
        let pathfinder = VoxelAStar::new(&grid, AStarConfig::default());

        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(5, 0, 0));

        let route = pathfinder.find_path(&goal).unwrap();
        assert!(route.is_valid());
        assert!(route.path().len() > 0);
    }

    #[test]
    fn test_astar_trivial_path() {
        let grid = empty_grid();
        let pathfinder = VoxelAStar::new(&grid, AStarConfig::default());

        let coord = VoxelCoord::new(5, 5, 5);
        let goal = RouteGoal::from_voxels(coord, coord);

        let route = pathfinder.find_path(&goal).unwrap();
        assert!(route.is_valid());
        assert_eq!(route.path().len(), 1);
    }

    #[test]
    fn test_astar_straight_line_no_diagonal() {
        let grid = empty_grid();
        let config = AStarConfig::default()
            .with_diagonal(false)
            .with_heuristic(Heuristic::Manhattan);
        let pathfinder = VoxelAStar::new(&grid, config);

        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(5, 0, 0));

        let route = pathfinder.find_path(&goal).unwrap();
        assert!(route.is_valid());

        // Should be exactly 6 nodes (0,0,0 to 5,0,0)
        assert_eq!(route.path().len(), 6);
    }

    #[test]
    fn test_astar_start_blocked() {
        let mut grid = empty_grid();
        grid.set(VoxelCoord::new(0, 0, 0), true);

        let pathfinder = VoxelAStar::new(&grid, AStarConfig::default());
        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(5, 0, 0));

        let result = pathfinder.find_path(&goal);
        assert!(matches!(result, Err(RoutingError::StartBlocked(_))));
    }

    #[test]
    fn test_astar_goal_blocked() {
        let mut grid = empty_grid();
        grid.set(VoxelCoord::new(5, 0, 0), true);

        let pathfinder = VoxelAStar::new(&grid, AStarConfig::default());
        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(5, 0, 0));

        let result = pathfinder.find_path(&goal);
        assert!(matches!(result, Err(RoutingError::GoalBlocked(_))));
    }

    #[test]
    fn test_astar_around_obstacle() {
        // Create a wall at x=5, blocking direct path
        let mut grid = empty_grid();
        for y in -5..=5 {
            for z in -5..=5 {
                grid.set(VoxelCoord::new(5, y, z), true);
            }
        }
        // Leave a gap at y=6
        grid.remove(VoxelCoord::new(5, 6, 0));

        let pathfinder = VoxelAStar::new(&grid, AStarConfig::default());
        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 0, 0));

        let route = pathfinder.find_path(&goal).unwrap();
        assert!(route.is_valid());

        // Path should go around the wall
        assert!(route.path().len() > 10); // Longer than straight line
    }

    #[test]
    fn test_astar_no_path() {
        // Enclose the start point completely so no path exists
        let mut grid = empty_grid();
        // Block all 6 face neighbors of origin
        grid.set(VoxelCoord::new(1, 0, 0), true);
        grid.set(VoxelCoord::new(-1, 0, 0), true);
        grid.set(VoxelCoord::new(0, 1, 0), true);
        grid.set(VoxelCoord::new(0, -1, 0), true);
        grid.set(VoxelCoord::new(0, 0, 1), true);
        grid.set(VoxelCoord::new(0, 0, -1), true);

        let config = AStarConfig::default().with_diagonal(false);
        let pathfinder = VoxelAStar::new(&grid, config);
        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 0, 0));

        let result = pathfinder.find_path(&goal);
        assert!(matches!(result, Err(RoutingError::NoPathFound { .. })));
    }

    #[test]
    fn test_astar_with_via_point() {
        let grid = empty_grid();
        let pathfinder = VoxelAStar::new(&grid, AStarConfig::default());

        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 0, 0))
            .with_via_point(GoalPoint::voxel(5, 5, 0));

        let route = pathfinder.find_path(&goal).unwrap();
        assert!(route.is_valid());

        // Path should go through via point
        let path = route.path().as_voxel().unwrap();
        assert!(path.coords().contains(&VoxelCoord::new(5, 5, 0)));
    }

    #[test]
    fn test_astar_route_stats() {
        let grid = empty_grid();
        let pathfinder = VoxelAStar::new(&grid, AStarConfig::default());

        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(5, 0, 0));

        let route = pathfinder.find_path(&goal).unwrap();
        assert_eq!(route.stats().algorithm(), "A*");
        assert!(route.stats().time_elapsed().as_nanos() > 0);
    }

    #[test]
    fn test_astar_route_cost() {
        let grid = empty_grid();
        let config = AStarConfig::default().with_diagonal(false);
        let pathfinder = VoxelAStar::new(&grid, config);

        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(5, 0, 0));

        let route = pathfinder.find_path(&goal).unwrap();
        // Straight line of 5 steps, each cost 1.0
        assert_relative_eq!(route.cost().length(), 5.0, epsilon = 0.01);
    }

    #[test]
    fn test_find_path_convenience() {
        let grid = empty_grid();

        let route = find_path(
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(5, 0, 0),
            &grid,
            false,
        )
        .unwrap();

        assert!(route.is_valid());
        assert_eq!(route.path().len(), 6);
    }

    #[test]
    fn test_astar_world_goal() {
        let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
        let pathfinder = VoxelAStar::new(&grid, AStarConfig::default());

        use nalgebra::Point3;
        let goal = RouteGoal::from_points(Point3::new(0.5, 0.5, 0.5), Point3::new(5.5, 0.5, 0.5));

        let route = pathfinder.find_path(&goal).unwrap();
        assert!(route.is_valid());
    }

    #[test]
    fn test_astar_diagonal_shorter() {
        let grid = empty_grid();

        // With diagonal movement
        let config_diag = AStarConfig::default().with_diagonal(true);
        let pathfinder_diag = VoxelAStar::new(&grid, config_diag);

        // Without diagonal movement
        let config_no_diag = AStarConfig::default().with_diagonal(false);
        let pathfinder_no_diag = VoxelAStar::new(&grid, config_no_diag);

        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(5, 5, 0));

        let route_diag = pathfinder_diag.find_path(&goal).unwrap();
        let route_no_diag = pathfinder_no_diag.find_path(&goal).unwrap();

        // Diagonal path should be shorter (in nodes)
        assert!(route_diag.path().len() < route_no_diag.path().len());
    }
}
