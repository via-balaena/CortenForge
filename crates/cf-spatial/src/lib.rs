//! Spatial data structures for CortenForge.
//!
//! This crate provides foundational spatial data structures used across the CortenForge
//! ecosystem for pathfinding, collision detection, and spatial queries:
//!
//! - [`VoxelGrid`] - Regular 3D voxel lattice with efficient indexing
//! - [`VoxelCoord`] - Integer voxel coordinates
//! - [`GridBounds`] - Axis-aligned bounds in grid space
//! - [`OccupancyMap`] - Probabilistic occupancy grid for SLAM/sensor fusion
//! - [`Ray`] and [`raycast`] - Ray-voxel intersection and line of sight queries
//! - [`Aabb`] and [`Sphere`] - Geometric primitives for overlap queries
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//! - CLI tools
//! - Web applications (WASM)
//! - Servers
//! - Embedded systems
//! - Other game engines
//! - Python bindings
//!
//! # Use Cases
//!
//! - **Pathfinding**: A* and other algorithms on voxelized space
//! - **Collision detection**: Fast spatial queries with AABB/sphere overlap
//! - **Occupancy mapping**: Sensor fusion and SLAM
//! - **Raycasting**: Visibility queries and line of sight
//! - **Mesh voxelization**: Converting meshes to voxel representations
//!
//! # Coordinate Systems
//!
//! The grid uses a **right-handed coordinate system** consistent with mesh-types:
//! - X: width (left/right)
//! - Y: depth (front/back)
//! - Z: height (up/down)
//!
//! World coordinates are continuous `f64` values. Grid coordinates are discrete `i32` values.
//! The [`VoxelGrid`] handles conversion between the two.
//!
//! # Example
//!
//! ```
//! use cf_spatial::{VoxelGrid, VoxelCoord};
//! use nalgebra::Point3;
//!
//! // Create a 10x10x10 grid with 0.1 unit voxel size
//! let mut grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
//!
//! // Mark some voxels as occupied
//! let coord = VoxelCoord::new(5, 5, 5);
//! grid.set(coord, true);
//!
//! // Convert world coordinates to grid coordinates
//! let world_point = Point3::new(0.55, 0.55, 0.55);
//! let grid_coord = grid.world_to_grid(world_point);
//! assert_eq!(grid_coord, coord);
//!
//! // Query the grid
//! assert_eq!(grid.get(coord), Some(&true));
//! ```
//!
//! # Occupancy Mapping
//!
//! The [`OccupancyMap`] uses log-odds representation for Bayesian sensor fusion:
//!
//! ```
//! use cf_spatial::{OccupancyMap, VoxelCoord};
//!
//! let mut map = OccupancyMap::new(0.1);
//!
//! // Update from sensor observations
//! let coord = VoxelCoord::new(5, 5, 5);
//! map.update_occupied(coord);
//!
//! // Query occupancy status
//! assert!(map.is_occupied(coord));
//! assert!(map.probability(coord) > 0.5);
//! ```
//!
//! # Raycasting
//!
//! Cast rays through voxel grids for visibility and collision queries:
//!
//! ```
//! use cf_spatial::{VoxelGrid, VoxelCoord, Ray, raycast, line_of_sight};
//! use nalgebra::{Point3, Vector3};
//!
//! let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
//! grid.set(VoxelCoord::new(5, 0, 0), true);
//!
//! // Cast a ray and check for hits
//! let ray = Ray::new(Point3::origin(), Vector3::x());
//! if let Some(hit) = raycast(&ray, &grid, 100.0, |v| *v) {
//!     assert_eq!(hit.coord, VoxelCoord::new(5, 0, 0));
//! }
//!
//! // Check line of sight between two points
//! let blocked = !line_of_sight(
//!     &Point3::origin(),
//!     &Point3::new(10.0, 0.0, 0.0),
//!     &grid,
//!     |v| *v,
//! );
//! assert!(blocked);
//! ```
//!
//! # Overlap Queries
//!
//! Find voxels that intersect geometric primitives:
//!
//! ```
//! use cf_spatial::{VoxelGrid, VoxelCoord, Aabb, Sphere, query_aabb, query_sphere};
//! use nalgebra::Point3;
//!
//! let mut grid: VoxelGrid<u32> = VoxelGrid::new(1.0);
//! grid.set(VoxelCoord::new(5, 5, 5), 42);
//!
//! // Query by AABB
//! let aabb = Aabb::new(Point3::new(4.0, 4.0, 4.0), Point3::new(6.0, 6.0, 6.0));
//! let results: Vec<_> = query_aabb(&grid, &aabb).collect();
//! assert_eq!(results.len(), 1);
//!
//! // Query by sphere
//! let sphere = Sphere::new(Point3::new(5.5, 5.5, 5.5), 2.0);
//! for (coord, value) in query_sphere(&grid, &sphere) {
//!     println!("Found voxel {:?} with value {}", coord, value);
//! }
//! ```
//!
//! # Quality Standards
//!
//! This crate maintains A-grade standards per [STANDARDS.md](../../STANDARDS.md):
//! - ≥90% test coverage
//! - Zero clippy/doc warnings
//! - Zero `unwrap`/`expect` in library code

// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]
#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]

mod error;
mod grid;
mod occupancy;
mod overlap;
mod raycast;
mod voxel;

// Re-export core types
pub use error::SpatialError;
pub use grid::{GridBounds, VoxelGrid};
pub use occupancy::{OccupancyMap, log_odds_to_probability, probability_to_log_odds};
pub use overlap::{
    Aabb, Sphere, any_in_aabb, any_in_sphere, count_in_aabb, count_in_sphere, query_aabb,
    query_aabb_filter, query_sphere, query_sphere_filter,
};
pub use raycast::{Ray, RaycastHit, VoxelTraversal, line_of_sight, raycast, raycast_all};
pub use voxel::VoxelCoord;

// Re-export nalgebra types for convenience
pub use nalgebra::{Point3, Vector3};
