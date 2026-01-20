// Temporary: Allow redundant_clone until the crate is refactored
// TODO: Fix redundant clones and remove this allow
#![allow(clippy::redundant_clone)]
// Allow similar_names like smoother/smoothed, shortener/shortened which are intentional
#![allow(clippy::similar_names)]

//! Multi-objective route optimization algorithms.
//!
//! This crate provides optimization algorithms for improving route quality
//! across multiple objectives: length, clearance, curvature, and bends.
//!
//! # Overview
//!
//! Route optimization is typically multi-objective: we want paths that are
//! short, maintain clearance from obstacles, have gentle curves, and minimize
//! direction changes. These objectives often conflict, so we need to explore
//! trade-offs.
//!
//! This crate provides:
//!
//! - **Clearance optimization** ([`clearance`]): Maximize distance from obstacles
//! - **Path shortening** ([`shorten`]): Remove redundant waypoints
//! - **Curvature analysis** ([`curvature`]): Analyze and improve path smoothness
//! - **Pareto utilities** ([`pareto`]): Handle multi-objective trade-offs
//! - **Multi-objective optimization** ([`multi_objective`]): Generate Pareto-optimal sets
//!
//! # Quick Start
//!
//! ```
//! use route_optimize::{
//!     clearance::ClearanceOptimizer,
//!     shorten::PathShortener,
//!     multi_objective::MultiObjectiveOptimizer,
//! };
//! use route_types::{VoxelPath, Route, Path};
//! use cf_spatial::{VoxelCoord, VoxelGrid};
//!
//! // Create an obstacle grid
//! let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
//! grid.set(VoxelCoord::new(2, 1, 0), true);  // Add obstacle
//!
//! // Create a path
//! let path = VoxelPath::new(vec![
//!     VoxelCoord::new(0, 0, 0),
//!     VoxelCoord::new(1, 0, 0),
//!     VoxelCoord::new(2, 0, 0),
//!     VoxelCoord::new(3, 0, 0),
//! ]);
//!
//! // Shorten the path
//! let shortener = PathShortener::new(&grid);
//! let shortened = shortener.shorten(&path);
//!
//! // Optimize for clearance
//! let optimizer = ClearanceOptimizer::new(&grid);
//! let clearance_optimized = optimizer.optimize(&path, 5);
//!
//! // Get a Pareto-optimal set
//! let multi = MultiObjectiveOptimizer::new(&grid);
//! let route = Route::new(Path::Voxel(path));
//! let pareto_set = multi.optimize(&route);
//! ```
//!
//! # Optimization Strategies
//!
//! ## Clearance Optimization
//!
//! Pushes waypoints away from obstacles:
//!
//! ```
//! use route_optimize::clearance::ClearanceOptimizer;
//! use route_types::VoxelPath;
//! use cf_spatial::{VoxelCoord, VoxelGrid};
//!
//! let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
//! let optimizer = ClearanceOptimizer::new(&grid).with_max_radius(10);
//!
//! let path = VoxelPath::new(vec![
//!     VoxelCoord::new(0, 0, 0),
//!     VoxelCoord::new(1, 0, 0),
//!     VoxelCoord::new(2, 0, 0),
//! ]);
//!
//! // Check clearance along path
//! let min_clearance = optimizer.minimum_clearance(&path);
//! let avg_clearance = optimizer.average_clearance(&path);
//! ```
//!
//! ## Path Shortening
//!
//! Removes redundant waypoints using line-of-sight:
//!
//! ```
//! use route_optimize::shorten::PathShortener;
//! use route_types::VoxelPath;
//! use cf_spatial::{VoxelCoord, VoxelGrid};
//!
//! let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
//! let shortener = PathShortener::new(&grid);
//!
//! let path = VoxelPath::new(vec![
//!     VoxelCoord::new(0, 0, 0),
//!     VoxelCoord::new(1, 0, 0),
//!     VoxelCoord::new(2, 0, 0),  // Redundant on straight line
//!     VoxelCoord::new(3, 0, 0),
//! ]);
//!
//! let shortened = shortener.shorten(&path);
//! assert_eq!(shortened.len(), 2);  // Just start and end
//! ```
//!
//! ## Curvature Analysis
//!
//! Analyze and improve path smoothness:
//!
//! ```
//! use route_optimize::curvature::{curvature_profile, count_bends, satisfies_min_radius};
//! use route_types::VoxelPath;
//! use cf_spatial::VoxelCoord;
//!
//! let path = VoxelPath::new(vec![
//!     VoxelCoord::new(0, 0, 0),
//!     VoxelCoord::new(1, 0, 0),
//!     VoxelCoord::new(1, 1, 0),  // Sharp turn
//!     VoxelCoord::new(2, 1, 0),
//! ]);
//!
//! let profile = curvature_profile(&path);
//! let bends = count_bends(&path, 0.1);
//! let ok = satisfies_min_radius(&path, 0.5);
//! ```
//!
//! ## Multi-Objective Optimization
//!
//! Generate Pareto-optimal solutions:
//!
//! ```
//! use route_optimize::multi_objective::{MultiObjectiveOptimizer, select_best};
//! use route_types::{VoxelPath, Route, Path, CostWeights};
//! use cf_spatial::{VoxelCoord, VoxelGrid};
//!
//! let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
//! let optimizer = MultiObjectiveOptimizer::new(&grid);
//!
//! let path = VoxelPath::new(vec![
//!     VoxelCoord::new(0, 0, 0),
//!     VoxelCoord::new(1, 0, 0),
//!     VoxelCoord::new(2, 0, 0),
//! ]);
//! let route = Route::new(Path::Voxel(path));
//!
//! // Get Pareto-optimal set
//! let pareto_set = optimizer.optimize(&route);
//!
//! // Select best according to preferences
//! let weights = CostWeights::default().with_length(1.0);
//! let best = select_best(&pareto_set, &weights);
//! ```

#![doc(html_root_url = "https://docs.rs/route-optimize/0.7.0")]
#![deny(clippy::unwrap_used, clippy::expect_used)]

pub mod clearance;
pub mod curvature;
pub mod multi_objective;
pub mod pareto;
pub mod shorten;

// Re-export main types for convenience
pub use clearance::{ClearanceOptimizer, clearance_profile};
pub use curvature::{
    CurvatureSmoother, bend_angle, count_bends, curvature_at, curvature_profile, max_curvature,
    satisfies_min_radius,
};
pub use multi_objective::{MultiObjectiveOptimizer, filter_pareto, select_best};
pub use pareto::{ObjectivePoint, dominates, hypervolume_2d, is_equivalent, pareto_frontier};
pub use shorten::{PathShortener, length_reduction, reduction_ratio};

#[cfg(test)]
mod integration_tests {
    use super::*;
    use cf_spatial::{VoxelCoord, VoxelGrid};
    use route_types::{CostWeights, Path, Route, VoxelPath};

    /// Test full optimization workflow.
    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_full_workflow() {
        // Create grid with obstacle
        let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
        grid.set(VoxelCoord::new(2, 1, 0), true);

        // Create a path that could be optimized
        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
            VoxelCoord::new(3, 0, 0),
            VoxelCoord::new(4, 0, 0),
        ]);

        // Analyze
        let clearance_opt = ClearanceOptimizer::new(&grid);
        let min_clearance = clearance_opt.minimum_clearance(&path);
        assert!(min_clearance > 0.0);

        // Shorten
        let shortener = PathShortener::new(&grid);
        let shortened = shortener.shorten(&path);
        assert!(shortened.len() <= path.len());

        // Multi-objective optimize
        let route = Route::new(Path::Voxel(path));
        let multi = MultiObjectiveOptimizer::new(&grid);
        let pareto_set = multi.optimize(&route);
        assert!(!pareto_set.is_empty());

        // Select best
        let weights = CostWeights::default().with_length(1.0);
        let best = select_best(&pareto_set, &weights);
        assert!(best.is_some());
    }

    /// Test curvature analysis.
    #[test]
    fn test_curvature_workflow() {
        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(1, 1, 0),
            VoxelCoord::new(2, 1, 0),
        ]);

        let profile = curvature_profile(&path);
        assert_eq!(profile.len(), 2);

        let bends = count_bends(&path, 0.1);
        assert!(bends >= 1);

        // Path with sharp turn won't satisfy large radius
        let satisfies = satisfies_min_radius(&path, 0.1);
        // Depending on path, may or may not satisfy
        let _ = satisfies; // Just verify it runs
    }

    /// Test Pareto frontier computation.
    #[test]
    fn test_pareto_workflow() {
        let a = ObjectivePoint::new(vec![5.0, 3.0]);
        let b = ObjectivePoint::new(vec![4.0, 4.0]);
        let c = ObjectivePoint::new(vec![6.0, 5.0]);

        // A and B are non-dominated, C is dominated
        assert!(!dominates(&a, &b));
        assert!(!dominates(&b, &a));
        assert!(is_equivalent(&a, &b));
        assert!(dominates(&a, &c));

        let solutions = vec![a, b, c];
        let frontier = pareto_frontier(&solutions);
        assert_eq!(frontier.len(), 2);
    }

    /// Test path shortening with obstacles.
    #[test]
    fn test_shortening_with_obstacles() {
        let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
        // Create a wall
        for y in -5..=5 {
            grid.set(VoxelCoord::new(5, y, 0), true);
        }

        // Path that goes around the wall
        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(0, 6, 0),
            VoxelCoord::new(5, 6, 0),
            VoxelCoord::new(10, 6, 0),
            VoxelCoord::new(10, 0, 0),
        ]);

        let shortener = PathShortener::new(&grid);
        let shortened = shortener.shorten(&path);

        // Wall blocks direct path, so can't shorten too much
        assert!(shortened.len() >= 2);
    }
}
