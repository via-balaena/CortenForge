//! Core types for 3D routing: paths, constraints, goals, and costs.
//!
//! This crate provides the foundational types for the routing domain,
//! supporting physical 3D routing applications such as wire harnesses,
//! pipes, and tendons.
//!
//! # Overview
//!
//! The routing domain is organized into several conceptual areas:
//!
//! - **Paths**: Representations of routes in discrete voxel space ([`VoxelPath`])
//!   and continuous world space ([`ContinuousPath`])
//! - **Constraints**: Physical requirements like clearance and bend radius
//!   ([`PhysicalConstraints`], [`RouteConstraints`])
//! - **Goals**: Start/end points, via points, and attractors ([`RouteGoal`], [`GoalPoint`])
//! - **Costs**: Multi-objective cost computation ([`CostWeights`], [`RouteCost`])
//! - **Configuration**: Algorithm settings ([`AStarConfig`], [`ThetaStarConfig`])
//! - **Results**: Complete route solutions with metadata ([`Route`], [`RouteStats`])
//!
//! # Example
//!
//! ```
//! use route_types::{
//!     VoxelPath, RouteGoal, GoalPoint, RouteConstraints,
//!     PhysicalConstraints, AStarConfig, Heuristic,
//! };
//! use cf_spatial::VoxelCoord;
//!
//! // Define a routing goal
//! let goal = RouteGoal::from_voxels(
//!     VoxelCoord::new(0, 0, 0),
//!     VoxelCoord::new(10, 10, 10),
//! );
//!
//! // Define constraints
//! let constraints = RouteConstraints::new()
//!     .with_physical(PhysicalConstraints::new(0.05))  // 5cm clearance
//!     .with_max_length(50.0);
//!
//! // Configure the algorithm
//! let config = AStarConfig::default()
//!     .with_heuristic(Heuristic::Euclidean)
//!     .with_diagonal(true);
//!
//! // Paths and routes would be computed by route-pathfind crate
//! ```
//!
//! # Integration with cf-spatial
//!
//! This crate builds on the `cf-spatial` foundation, using:
//!
//! - [`cf_spatial::VoxelCoord`] for discrete path nodes
//! - [`cf_spatial::VoxelGrid`] for obstacle representation (in pathfind crate)
//! - [`cf_spatial::line_of_sight`] for visibility checks (in pathfind crate)
//!
//! # Feature Flags
//!
//! - `serde`: Enables serialization/deserialization for all types

#![doc(html_root_url = "https://docs.rs/route-types/0.7.0")]
#![deny(clippy::unwrap_used, clippy::expect_used)]

pub mod config;
pub mod constraint;
pub mod cost;
pub mod error;
pub mod goal;
pub mod path;
pub mod route;

// Re-export main types at crate root for convenience
pub use config::{AStarConfig, Heuristic, RrtStarConfig, ThetaStarConfig};
pub use constraint::{Aabb, KeepOutZone, PhysicalConstraints, RouteConstraints, Sphere};
pub use cost::{CostWeights, RouteCost};
pub use error::RoutingError;
pub use goal::{Attractor, GoalPoint, RouteGoal};
pub use path::{ContinuousPath, Path, VoxelPath, Waypoint};
pub use route::{Route, RouteStats};

#[cfg(test)]
mod integration_tests {
    use super::*;
    use cf_spatial::VoxelCoord;
    use nalgebra::Point3;

    /// Test that all types can be constructed and used together.
    #[test]
    fn test_full_workflow_types() {
        // Create a goal
        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 0, 0))
            .with_via_point(GoalPoint::voxel(5, 0, 0))
            .with_attractor(Attractor::new(Point3::new(5.0, 2.0, 0.0), 1.0, 5.0));

        assert_eq!(goal.num_segments(), 2);

        // Create constraints
        let constraints = RouteConstraints::new()
            .with_physical(PhysicalConstraints::new(0.05).with_min_bend_radius(0.1))
            .with_keep_out_zone(KeepOutZone::Aabb(Aabb::new(
                Point3::new(2.0, 2.0, 0.0),
                Point3::new(3.0, 3.0, 1.0),
            )))
            .with_max_length(20.0)
            .with_max_bends(10);

        assert!(constraints.physical().is_some());
        assert_eq!(constraints.keep_out_zones().len(), 1);

        // Create configuration
        let config = AStarConfig::default()
            .with_heuristic(Heuristic::Euclidean)
            .with_diagonal(true)
            .with_weights(
                CostWeights::default()
                    .with_length(1.0)
                    .with_bends(0.5)
                    .with_clearance(0.3),
            );

        assert!(config.validate().is_empty());

        // Create a path
        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
            VoxelCoord::new(3, 0, 0),
            VoxelCoord::new(4, 0, 0),
            VoxelCoord::new(5, 0, 0),
        ]);

        assert_eq!(path.len(), 6);
        assert!((path.length() - 5.0).abs() < 1e-10);

        // Create a route
        let route = Route::from_voxel_path(path)
            .with_cost(RouteCost::new().with_length(5.0).with_bends(2.0))
            .with_stats(
                RouteStats::new("A*")
                    .with_nodes_expanded(100)
                    .with_elapsed(std::time::Duration::from_millis(5)),
            )
            .with_constraints_satisfied(true);

        assert!(route.is_valid());
        assert_eq!(route.node_count(), 6);
    }

    /// Test error types.
    #[test]
    fn test_error_types() {
        let error = RoutingError::NoPathFound {
            start: VoxelCoord::new(0, 0, 0),
            goal: VoxelCoord::new(10, 10, 10),
        };
        assert!(error.is_no_path_found());
        assert!(!error.is_timeout());

        let error = RoutingError::Timeout(std::time::Duration::from_secs(30));
        assert!(error.is_timeout());
        assert!(!error.is_no_path_found());

        let error = RoutingError::constraint_violation("clearance too small");
        assert!(error.to_string().contains("clearance"));

        let error = RoutingError::invalid_config("negative timeout");
        assert!(error.to_string().contains("negative timeout"));
    }

    /// Test path conversions.
    #[test]
    fn test_path_enum() {
        let voxel_path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 0, 0)]);

        let path: Path = voxel_path.into();
        assert!(path.is_voxel());

        let continuous_path = ContinuousPath::from_points(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        ]);

        let path: Path = continuous_path.into();
        assert!(path.is_continuous());
    }

    /// Test cost computation.
    #[test]
    fn test_cost_computation() {
        let cost = RouteCost::new()
            .with_length(10.0)
            .with_bends(3.0)
            .with_curvature(0.5)
            .with_clearance(-2.0) // Negative = good clearance
            .with_corridor_deviation(1.0);

        let weights = CostWeights::default()
            .with_length(1.0)
            .with_bends(0.5)
            .with_curvature(0.2)
            .with_clearance(1.0)
            .with_corridor_deviation(0.1);

        let total = cost.compute_weighted(&weights);
        // 1.0*10 + 0.5*3 + 0.2*0.5 + 1.0*(-2) + 0.1*1.0
        // = 10 + 1.5 + 0.1 - 2 + 0.1 = 9.7
        assert!((total - 9.7).abs() < 1e-10);
    }
}
