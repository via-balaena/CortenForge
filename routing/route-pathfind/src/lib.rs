//! Pathfinding algorithms for 3D voxel grids.
//!
//! This crate provides pathfinding algorithms that integrate with `cf-spatial`
//! voxel grids and the `route-types` domain types.
//!
//! # Overview
//!
//! The pathfinding crate offers several algorithms with different trade-offs:
//!
//! - **A\*** ([`astar::VoxelAStar`]): Classic A* on voxel grids with configurable
//!   heuristics and connectivity (6 or 26 neighbors)
//! - **Path Smoothing** ([`smooth::PathSmoother`]): Post-processing to simplify
//!   paths using line-of-sight checks
//!
//! # Quick Start
//!
//! ```
//! use route_pathfind::{astar::VoxelAStar, smooth::PathSmoother};
//! use route_types::{AStarConfig, RouteGoal, Heuristic};
//! use cf_spatial::{VoxelCoord, VoxelGrid};
//!
//! // Create an obstacle grid
//! let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
//!
//! // Add some obstacles
//! grid.set(VoxelCoord::new(5, 0, 0), true);
//!
//! // Create pathfinder
//! let pathfinder = VoxelAStar::new(&grid, AStarConfig::default());
//!
//! // Define start and goal
//! let goal = RouteGoal::from_voxels(
//!     VoxelCoord::new(0, 0, 0),
//!     VoxelCoord::new(10, 0, 0),
//! );
//!
//! // Find path (will route around obstacle)
//! let route = pathfinder.find_path(&goal).expect("Path should exist");
//! assert!(route.is_valid());
//!
//! // Optionally smooth the path
//! let smoother = PathSmoother::new(&grid);
//! let smoothed = smoother.smooth_route(&route);
//! ```
//!
//! # Algorithm Selection
//!
//! | Algorithm | Best For | Trade-offs |
//! |-----------|----------|------------|
//! | A* | General pathfinding | Optimal but can be slow in large spaces |
//! | A* (6-conn) | Grid-aligned paths | Fewer neighbors, faster but longer paths |
//! | A* (26-conn) | Natural-looking paths | More neighbors, allows diagonal movement |
//!
//! # Integration with cf-spatial
//!
//! This crate uses the following from `cf-spatial`:
//!
//! - [`cf_spatial::VoxelCoord`] for path nodes
//! - [`cf_spatial::VoxelGrid`] for obstacle storage
//! - [`cf_spatial::line_of_sight`] for visibility checks in smoothing
//!
//! # Heuristics
//!
//! The [`heuristics`] module provides distance functions:
//!
//! - **Manhattan**: Best for 6-connectivity (face neighbors only)
//! - **Chebyshev**: Works for 26-connectivity (all neighbors)
//! - **Euclidean**: Always admissible, good general-purpose
//! - **Octile**: Optimal for 26-connectivity with proper diagonal costs
//!
//! # Example: Path Around Obstacles
//!
//! ```
//! use route_pathfind::astar::find_path;
//! use cf_spatial::{VoxelCoord, VoxelGrid};
//!
//! // Create grid with a wall
//! let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
//! for y in -5..=5 {
//!     grid.set(VoxelCoord::new(5, y, 0), true);
//! }
//! // Leave a gap
//! grid.remove(VoxelCoord::new(5, 6, 0));
//!
//! // Find path around the wall
//! let route = find_path(
//!     VoxelCoord::new(0, 0, 0),
//!     VoxelCoord::new(10, 0, 0),
//!     &grid,
//!     true,  // allow diagonal
//! ).expect("Should find path through gap");
//!
//! assert!(route.path().len() > 10);  // Longer than straight line
//! ```

#![doc(html_root_url = "https://docs.rs/route-pathfind/0.7.0")]
#![deny(clippy::unwrap_used, clippy::expect_used)]

pub mod astar;
pub mod heuristics;
pub mod neighbors;
pub mod smooth;

// Re-export main types for convenience
pub use astar::{VoxelAStar, find_path};
pub use heuristics::compute_heuristic;
pub use neighbors::{NeighborGenerator, successors_for_grid};
pub use smooth::PathSmoother;

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::similar_names)]
mod integration_tests {
    use super::*;
    use cf_spatial::{VoxelCoord, VoxelGrid};
    use route_types::{AStarConfig, GoalPoint, Heuristic, RouteGoal};

    /// Test the full workflow: create grid, find path, smooth.
    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_full_workflow() {
        // Create grid with a wall
        let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
        for y in -5..=5 {
            grid.set(VoxelCoord::new(5, y, 0), true);
        }
        // Leave a gap
        grid.remove(VoxelCoord::new(5, 6, 0));

        // Configure pathfinder
        let config = AStarConfig::default()
            .with_heuristic(Heuristic::Euclidean)
            .with_diagonal(true);

        let pathfinder = VoxelAStar::new(&grid, config);

        // Find path
        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 0, 0));

        let route = pathfinder.find_path(&goal).unwrap();
        assert!(route.is_valid());
        let original_len = route.path().len();

        // Smooth the path
        let smoother = PathSmoother::new(&grid);
        let smoothed = smoother.smooth_route(&route);

        assert!(smoothed.is_valid());
        // Smoothed path should be shorter or equal
        assert!(smoothed.path().len() <= original_len);
    }

    /// Test via point routing.
    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_via_point_routing() {
        let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
        let pathfinder = VoxelAStar::new(&grid, AStarConfig::default());

        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 0, 0))
            .with_via_point(GoalPoint::voxel(0, 5, 0))
            .with_via_point(GoalPoint::voxel(10, 5, 0));

        let route = pathfinder.find_path(&goal).unwrap();
        assert!(route.is_valid());

        // Path should go through via points
        let path = route.path().as_voxel().unwrap();
        assert!(path.coords().contains(&VoxelCoord::new(0, 5, 0)));
        assert!(path.coords().contains(&VoxelCoord::new(10, 5, 0)));
    }

    /// Test heuristic selection.
    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_heuristic_selection() {
        let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);

        let configs = [
            AStarConfig::default().with_heuristic(Heuristic::Manhattan),
            AStarConfig::default().with_heuristic(Heuristic::Euclidean),
            AStarConfig::default().with_heuristic(Heuristic::Octile),
        ];

        for config in configs {
            let pathfinder = VoxelAStar::new(&grid, config);
            let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(5, 5, 5));

            let route = pathfinder.find_path(&goal);
            assert!(route.is_ok());
        }
    }

    /// Test convenience function.
    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_find_path_convenience() {
        let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);

        // With diagonal
        let route1 = find_path(
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(5, 5, 5),
            &grid,
            true,
        )
        .unwrap();

        // Without diagonal
        let route2 = find_path(
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(5, 5, 5),
            &grid,
            false,
        )
        .unwrap();

        // Diagonal path should be shorter (fewer nodes)
        assert!(route1.path().len() < route2.path().len());
    }

    /// Test blocked start/goal detection.
    #[test]
    fn test_blocked_detection() {
        let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
        grid.set(VoxelCoord::new(0, 0, 0), true);

        let pathfinder = VoxelAStar::new(&grid, AStarConfig::default());
        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(5, 0, 0));

        let result = pathfinder.find_path(&goal);
        assert!(result.is_err());
    }
}
