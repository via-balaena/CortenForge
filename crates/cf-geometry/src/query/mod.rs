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
//! [`closest_point_on_triangle`], [`closest_point_segment`], and
//! [`closest_points_segments`] are free functions for common proximity queries.

mod closest_point;
mod ray_cast;

pub use closest_point::{
    closest_point_on_triangle, closest_point_segment, closest_points_segments,
};
pub use ray_cast::{ray_cast, ray_triangle};
