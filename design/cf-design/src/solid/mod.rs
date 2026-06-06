//! Opaque design primitive wrapping a field expression tree.
//!
//! `Solid` is the public API for constructing and querying implicit surface
//! fields. Consumers never see `FieldNode` — they interact entirely through
//! `Solid` methods.
//!
//! The internal representation can change without breaking downstream code
//! (expression tree today, B-Rep in the future — see `CF_DESIGN_SPEC` §8).

use crate::field_node::FieldNode;

mod ops;
mod params;
mod primitives;
mod query;

/// Lattice type for infill operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InfillKind {
    /// Gyroid triply-periodic minimal surface — the standard bio-inspired infill.
    Gyroid,
    /// Schwarz P triply-periodic minimal surface — alternative with cubic symmetry.
    SchwarzP,
}

/// Hint for selecting the appropriate [`sim_core::PhysicsShape`] implementation
/// when building a simulation model from a [`Solid`].
///
/// The model builder uses this to wrap SDF grids in the right shape type:
/// - `Sphere(radius)` → `ShapeSphere` (analytical, rotation-invariant)
/// - `Convex` → `ShapeConvex` (ray-marched effective radius)
/// - `Concave` → `ShapeConcave` (multi-contact surface tracing)
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ShapeHint {
    /// Bare sphere primitive with known analytical radius.
    Sphere(f64),
    /// Convex shape — effective radius via ray-march.
    Convex,
    /// Concave shape — forces multi-contact fallback.
    Concave,
}

/// Opaque solid defined by an implicit surface field.
///
/// Construct with primitive factory methods (`sphere`, `cuboid`, etc.) and
/// compose with boolean operations (`union`, `subtract`, etc.) and transforms
/// (`translate`, `rotate`, etc.) — added in later sessions.
///
/// Convention: the field is negative inside, positive outside, zero on surface.
#[derive(Debug, Clone)]
pub struct Solid {
    pub(crate) node: FieldNode,
}

#[cfg(test)]
mod tests;
