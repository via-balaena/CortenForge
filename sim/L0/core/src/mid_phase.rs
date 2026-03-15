//! Mid-phase collision detection — re-exported from [`cf_geometry`].
//!
//! The canonical BVH implementation lives in `cf_geometry::bvh`.
//! This module re-exports for sim-core consumers.

pub use cf_geometry::{Bvh, BvhPrimitive, bvh_from_mesh, bvh_from_triangle_mesh, query_bvh_pair};
