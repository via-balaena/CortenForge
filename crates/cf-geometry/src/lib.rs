//! Shared geometric kernel for CortenForge.
//!
//! `cf-geometry` provides the canonical geometric primitives used across all
//! CortenForge domains — mesh processing, physics simulation, spatial queries,
//! and route planning.
//!
//! # Design principles
//!
//! - **Geometry owns shape. Physics owns force.** This crate answers "what is
//!   this thing?" and "does it overlap that thing?" — never "what force results?"
//! - **One canonical type per concept.** One `Aabb`. One `Triangle`. One
//!   `IndexedMesh`. Domains use these types, not their own.
//! - **f64 everywhere. All types `Send + Sync`.** No interior mutability.
//!   The rendering layer converts to f32 at the boundary.
//! - **Data, not pipeline.** These are data structures with geometric queries,
//!   not ECS components or physics objects.
//!
//! # Layer 0
//!
//! Zero Bevy, zero GPU, zero framework dependencies. Pure `nalgebra` +
//! `thiserror` + optional `serde`.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod aabb;
mod bounded;
mod bvh;
mod convex_hull;
mod heightfield;
mod mesh;
mod ray;
mod sdf;
mod sphere;
mod triangle;

pub use aabb::{Aabb, Axis};
pub use bounded::Bounded;
pub use bvh::{Bvh, BvhPrimitive, bvh_from_mesh, bvh_from_triangle_mesh, query_bvh_pair};
pub use convex_hull::{ConvexHull, convex_hull};
pub use heightfield::HeightFieldData;
pub use mesh::IndexedMesh;
pub use ray::{Ray, RayHit};
pub use sdf::SdfGrid;
pub use sphere::Sphere;
pub use triangle::Triangle;
