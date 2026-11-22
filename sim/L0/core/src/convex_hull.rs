//! Convex hull types — re-exported from [`cf_geometry`].
//!
//! The canonical implementation lives in `cf_geometry::convex_hull`.
//! This module re-exports for sim-core consumers. It is not the hull a
//! model's mesh collides on: that is MuJoCo's, made by qhull
//! ([`crate::mesh::TriangleMeshData::hull`]).

pub use cf_geometry::{ConvexHull, convex_hull};

/// Backward-compatible alias for [`convex_hull()`].
///
/// sim-core historically called this function `quickhull`. The canonical
/// name in cf-geometry is `convex_hull`.
pub use cf_geometry::convex_hull as quickhull;
