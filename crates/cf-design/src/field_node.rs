//! Expression tree for implicit surface fields.
//!
//! `FieldNode` is the internal representation of a scalar field. Each variant
//! is a node in a composable expression tree that can be evaluated at a point
//! (returning f64) or over a bounding box (returning conservative interval
//! bounds).
//!
//! This type is `pub(crate)` — consumers interact through [`super::Solid`].

use std::sync::Arc;

use cf_geometry::Aabb;
use nalgebra::{Point3, UnitQuaternion, Vector3};

/// Wrapper for user-provided evaluation closure.
///
/// Supports `Clone` (via `Arc`) and `Debug` (prints opaque marker).
#[derive(Clone)]
pub struct UserEvalFn(pub Arc<dyn Fn(Point3<f64>) -> f64 + Send + Sync>);

impl std::fmt::Debug for UserEvalFn {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("<user_eval_fn>")
    }
}

/// Wrapper for user-provided interval bound closure.
#[derive(Clone)]
pub struct UserIntervalFn(pub Arc<dyn Fn(&Aabb) -> (f64, f64) + Send + Sync>);

impl std::fmt::Debug for UserIntervalFn {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("<user_interval_fn>")
    }
}

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

    // ── Boolean operations ────────────────────────────────────────────
    /// Union: `min(a, b)`. Preserves SDF lower-bound property.
    Union(Box<Self>, Box<Self>),

    /// Subtraction: `max(a, -b)` — keeps first, removes second.
    Subtract(Box<Self>, Box<Self>),

    /// Intersection: `max(a, b)`. Preserves SDF lower-bound property.
    Intersect(Box<Self>, Box<Self>),

    /// Smooth union with blend radius `k`. Adds material in blend region.
    /// Polynomial smooth min (IQ). Produces a lower-bound field.
    SmoothUnion(Box<Self>, Box<Self>, f64),

    /// Smooth subtraction with blend radius `k`.
    /// `smooth_intersect(a, -b, k)` = `-smooth_union(-a, b, k)`.
    SmoothSubtract(Box<Self>, Box<Self>, f64),

    /// Smooth intersection with blend radius `k`. Removes material in blend
    /// region. `-smooth_union(-a, -b, k)` (De Morgan).
    SmoothIntersect(Box<Self>, Box<Self>, f64),

    /// N-ary smooth union (order-independent / symmetric blend).
    /// Uses log-sum-exp for truly symmetric results regardless of input order.
    SmoothUnionAll(Vec<Self>, f64),

    // ── Transforms ────────────────────────────────────────────────────
    /// Translation: `f(p - offset)`.
    Translate(Box<Self>, Vector3<f64>),

    /// Rotation: `f(R⁻¹ · p)`. Preserves SDF property.
    Rotate(Box<Self>, UnitQuaternion<f64>),

    /// Uniform scaling: `f(p / s) * s`. Preserves SDF property.
    ScaleUniform(Box<Self>, f64),

    /// Mirror across a plane through origin with the given unit normal.
    /// `f(p - 2·min(0, dot(p, n))·n)` — reflects the negative half-space.
    Mirror(Box<Self>, Vector3<f64>),

    // ── Domain operations ─────────────────────────────────────────────
    /// Shell: `|f(p)| - thickness`. Hollows a solid to the given wall
    /// thickness. Requires exact SDF child for uniform wall thickness.
    Shell(Box<Self>, f64),

    /// Round: `f(p) - radius`. Adds rounding to all edges by shifting the
    /// isosurface inward. Requires exact SDF child for uniform rounding.
    Round(Box<Self>, f64),

    /// Offset: `f(p) - distance`. Grows (positive distance) or shrinks
    /// (negative distance) the shape uniformly. Requires exact SDF child.
    Offset(Box<Self>, f64),

    /// Elongate: stretches a shape along axes by inserting flat sections.
    /// `q = p - clamp(p, -h, h)`, then `f(q)`. Preserves SDF property.
    Elongate(Box<Self>, Vector3<f64>),

    // ── User escape hatch ─────────────────────────────────────────────
    /// User-provided function leaf node. Allows custom implicit surface
    /// functions that the expression tree does not natively support.
    ///
    /// If `interval` is `None`, interval evaluation returns `(-∞, +∞)`,
    /// disabling octree pruning for this subtree.
    UserFn {
        eval: UserEvalFn,
        interval: Option<UserIntervalFn>,
        /// Bounding box of the geometry — used by the mesher (Session 5+)
        /// to define the evaluation domain.
        #[allow(dead_code)]
        bounds: Aabb,
    },
}
