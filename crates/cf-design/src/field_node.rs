//! Expression tree for implicit surface fields.
//!
//! `FieldNode` is the internal representation of a scalar field. Each variant
//! is a node in a composable expression tree that can be evaluated at a point
//! (returning f64) or over a bounding box (returning conservative interval
//! bounds).
//!
//! This type is `pub(crate)` — consumers interact through [`super::Solid`].

use nalgebra::Vector3;

/// Internal expression tree node for implicit surface fields.
///
/// Convention: negative = inside, positive = outside, zero = on surface.
#[derive(Debug, Clone)]
pub enum FieldNode {
    // ── Geometric primitives ──────────────────────────────────────────
    /// Sphere centered at origin. Exact SDF: `|p| - radius`.
    Sphere { radius: f64 },

    /// Axis-aligned box centered at origin. Exact SDF.
    Cuboid { half_extents: Vector3<f64> },

    /// Z-aligned cylinder centered at origin. Exact SDF.
    Cylinder { radius: f64, half_height: f64 },

    /// Z-aligned capsule (cylinder + hemispherical caps) centered at origin.
    /// Exact SDF.
    Capsule { radius: f64, half_height: f64 },

    /// Ellipsoid centered at origin. Approximate SDF (not exact distance).
    Ellipsoid { radii: Vector3<f64> },

    /// Torus in the XY plane centered at origin. Exact SDF.
    Torus { major: f64, minor: f64 },

    /// Cone with apex at origin extending downward along -Z.
    /// `radius` is the base radius, `height` is the total height.
    /// Exact SDF.
    Cone { radius: f64, height: f64 },

    /// Half-space. `dot(normal, p) - offset`. Normal must be unit length.
    Plane { normal: Vector3<f64>, offset: f64 },
}
