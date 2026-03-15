//! Geometric query functions.
//!
//! All functions operate in **local space** — shapes are centered at the origin
//! with identity orientation. The consuming layer (e.g. sim-core) handles any
//! world-space pose transforms.
//!
//! # Ray casting
//!
//! [`ray_cast`] dispatches on [`Shape`](crate::Shape) to the appropriate
//! per-shape analytic or numerical algorithm. [`ray_triangle`] is the
//! standalone Möller–Trumbore ray–triangle test.
//!
//! # Closest point
//!
//! [`closest_point`] dispatches on [`Shape`](crate::Shape) to per-shape
//! implementations. [`closest_point_on_triangle`], [`closest_point_segment`],
//! and [`closest_points_segments`] are standalone free functions for common
//! proximity queries.
//!
//! # GJK / EPA
//!
//! [`gjk_distance`] computes the minimum separating distance between two
//! convex shapes (any [`SupportMap`](crate::SupportMap) implementors).
//! [`gjk_intersection`] is a fast boolean overlap test.
//! [`epa_penetration`] computes penetration depth and contact information
//! for overlapping convex shapes.

mod closest_point;
mod epa;
mod gjk;
mod ray_cast;

pub use closest_point::{
    closest_point, closest_point_on_triangle, closest_point_segment, closest_points_segments,
};
pub use epa::{Penetration, epa_penetration};
pub use gjk::{GjkDistance, gjk_distance, gjk_intersection};
pub use ray_cast::{ray_cast, ray_triangle};
