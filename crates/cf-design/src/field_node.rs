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

use crate::param::ParamStoreData;

/// A scalar value that is either a compile-time literal or a runtime design parameter.
///
/// Used in [`FieldNode`] variants where a constant (radius, blend k, thickness, etc.)
/// may optionally be a design variable. `eval()` resolves the current value at near-zero
/// cost for literals and one `RwLock` read for parameters.
///
/// Session 25 extends this with `param_deriv()` for chain-rule gradient computation.
#[derive(Clone)]
pub enum Val {
    /// Compile-time constant.
    Literal(f64),
    /// Runtime parameter resolved from a shared store.
    Param {
        id: usize,
        store: Arc<ParamStoreData>,
    },
}

impl Val {
    /// Resolve the current scalar value.
    #[inline]
    pub(crate) fn eval(&self) -> f64 {
        match self {
            Self::Literal(v) => *v,
            Self::Param { id, store } => store.get_by_id(*id),
        }
    }

    /// For Session 25: `∂(this value)/∂param[target_id]`.
    /// Returns 1.0 if this `Val` IS the target parameter, 0.0 otherwise.
    #[allow(dead_code)] // Used in Session 25
    pub(crate) const fn param_deriv(&self, target_id: usize) -> f64 {
        match self {
            Self::Literal(_) => 0.0,
            Self::Param { id, .. } => {
                if *id == target_id {
                    1.0
                } else {
                    0.0
                }
            }
        }
    }

    /// Whether this value is a design parameter (vs a literal).
    #[allow(dead_code)] // Used in Session 25
    pub(crate) const fn is_param(&self) -> bool {
        matches!(self, Self::Param { .. })
    }
}

impl std::fmt::Debug for Val {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Literal(v) => write!(f, "{v}"),
            Self::Param { id, store } => write!(f, "Param({}, {:?})", id, store.name_of(*id)),
        }
    }
}

impl From<f64> for Val {
    fn from(v: f64) -> Self {
        Self::Literal(v)
    }
}

impl From<crate::param::ParamRef> for Val {
    fn from(p: crate::param::ParamRef) -> Self {
        Self::Param {
            id: p.id,
            store: p.store,
        }
    }
}

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
    Sphere { radius: Val },

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

    // ── Bio-inspired primitives ──────────────────────────────────────
    /// Superellipsoid centered at origin. Approximate SDF (like Ellipsoid).
    ///
    /// Field: `((|x/rx|^(2/n2) + |y/ry|^(2/n2))^(n2/n1) + |z/rz|^(2/n1))^(n1/2) - 1`
    ///
    /// `n1` controls Z-roundness, `n2` controls XY-roundness.
    /// `n1=n2=2` → ellipsoid. Large values → cuboid. `n1=n2=1` → octahedron.
    Superellipsoid {
        radii: Vector3<f64>,
        n1: f64,
        n2: f64,
    },

    /// Logarithmic spiral tube in the XY plane.
    ///
    /// The spiral curve: `r(θ) = a · exp(b · θ)` for `θ ∈ [0, turns·2π]`.
    /// Field is distance to spiral curve minus thickness (shell around curve).
    /// Approximate SDF.
    LogSpiral {
        a: f64,
        b: f64,
        thickness: f64,
        turns: f64,
    },

    /// Gyroid triply-periodic minimal surface.
    ///
    /// Field: `|sin(sx)·cos(sy) + sin(sy)·cos(sz) + sin(sz)·cos(sx)| - thickness`
    ///
    /// Infinite geometry (bounds = `None`). Intersect with a finite solid for
    /// meshing — follows the same pattern as `Plane`.
    Gyroid { scale: f64, thickness: f64 },

    /// Schwarz P triply-periodic minimal surface.
    ///
    /// Field: `|cos(sx) + cos(sy) + cos(sz)| - thickness`
    ///
    /// Infinite geometry (bounds = `None`). Intersect with a finite solid for
    /// meshing — follows the same pattern as `Plane`.
    SchwarzP { scale: f64, thickness: f64 },

    /// Helix tube along the Z axis.
    ///
    /// The helix curve: `H(t) = (R·cos(2πt), R·sin(2πt), P·t)` for
    /// `t ∈ [0, turns]`. Field is distance to helix curve minus thickness.
    /// Near-exact SDF via Newton refinement (like `PipeSpline`).
    Helix {
        radius: f64,
        pitch: f64,
        thickness: f64,
        turns: f64,
    },

    // ── Path-based primitives ────────────────────────────────────────
    /// Pipe along a polyline path with spherical cross-section.
    /// SDF: `min(distance_to_segment(p, seg_i)) - radius` over all segments.
    /// Exact SDF. Natural rounding at corners via min-of-segments.
    Pipe {
        vertices: Vec<Point3<f64>>,
        radius: f64,
    },

    /// Pipe along a Catmull-Rom spline with spherical cross-section.
    /// Near-exact SDF via dense subdivision + Newton refinement per span.
    PipeSpline {
        control_points: Vec<Point3<f64>>,
        radius: f64,
    },

    /// Loft along Z axis with variable circular cross-section.
    ///
    /// Each station is `[z_position, radius]`. Stations must be sorted by Z
    /// with at least 2 entries. Radius is cubic-interpolated (Catmull-Rom)
    /// between stations. Capped at both ends.
    ///
    /// Approximate SDF: exact on the barrel for non-tapered sections,
    /// approximate in tapered regions (Lipschitz > 1 when |dR/dz| > 0).
    ///
    /// Gradient (for Session 19):
    ///   Barrel: `∇f ≈ (x/r_xy, y/r_xy, −R'(z))` normalized.
    ///   Caps: (0, 0, ±1) for cap face, radial at cap edge.
    Loft { stations: Vec<[f64; 2]> },

    // ── Boolean operations ────────────────────────────────────────────
    /// Union: `min(a, b)`. Preserves SDF lower-bound property.
    Union(Box<Self>, Box<Self>),

    /// Subtraction: `max(a, -b)` — keeps first, removes second.
    Subtract(Box<Self>, Box<Self>),

    /// Intersection: `max(a, b)`. Preserves SDF lower-bound property.
    Intersect(Box<Self>, Box<Self>),

    /// Smooth union with blend radius `k`. Adds material in blend region.
    /// Polynomial smooth min (IQ). Produces a lower-bound field.
    SmoothUnion(Box<Self>, Box<Self>, Val),

    /// Smooth subtraction with blend radius `k`.
    /// `smooth_intersect(a, -b, k)` = `-smooth_union(-a, b, k)`.
    SmoothSubtract(Box<Self>, Box<Self>, Val),

    /// Smooth intersection with blend radius `k`. Removes material in blend
    /// region. `-smooth_union(-a, -b, k)` (De Morgan).
    SmoothIntersect(Box<Self>, Box<Self>, Val),

    /// N-ary smooth union (order-independent / symmetric blend).
    /// Uses log-sum-exp for truly symmetric results regardless of input order.
    SmoothUnionAll(Vec<Self>, Val),

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
    Shell(Box<Self>, Val),

    /// Round: `f(p) - radius`. Adds rounding to all edges by shifting the
    /// isosurface inward. Requires exact SDF child for uniform rounding.
    Round(Box<Self>, Val),

    /// Offset: `f(p) - distance`. Grows (positive distance) or shrinks
    /// (negative distance) the shape uniformly. Requires exact SDF child.
    Offset(Box<Self>, Val),

    /// Elongate: stretches a shape along axes by inserting flat sections.
    /// `q = p - clamp(p, -h, h)`, then `f(q)`. Preserves SDF property.
    Elongate(Box<Self>, Vector3<f64>),

    /// Twist: rotate XY cross-section proportionally to Z position.
    /// `rate` is radians per unit Z.
    ///
    /// Distorts the distance field — not an exact SDF even if the child is.
    /// Lipschitz constant ≈ √(1 + `(rate·r_xy)²`) where `r_xy` is distance
    /// from the Z axis. Thin features far from the axis may be lost during
    /// meshing.
    ///
    /// Gradient (for Session 19):
    ///   Let q(p) = `R_z(rate·z)·p` be the deformation. The Jacobian J has
    ///   spectral norm √(1 + rate²·(x²+y²)). Chain rule:
    ///   `∇f_twist = Jᵀ · ∇f_child(q(p))`.
    Twist(Box<Self>, f64),

    /// Bend: curve a Z-extended shape in the XZ plane.
    /// `rate` is the curvature (radians per unit Z).
    ///
    /// Same distance field distortion as Twist. Works best for moderate
    /// curvatures (rate · `z_extent` < π/2). For large bends, increase
    /// mesh resolution.
    ///
    /// Gradient (for Session 19):
    ///   Same Jacobian analysis as Twist but in the XZ plane.
    ///   `∇f_bend = Jᵀ · ∇f_child(q(p))`.
    Bend(Box<Self>, f64),

    /// Infinite repetition with spacing vector.
    ///
    /// `q = p - spacing · round(p / spacing)`, then `f(q)`.
    /// Exact SDF only when child geometry fits within one cell
    /// (`[-spacing/2, spacing/2]` per axis). Overlapping geometry is
    /// clipped at cell boundaries.
    ///
    /// Produces infinite geometry (bounds = `None`). Intersect with a
    /// finite solid for meshing — same pattern as `Plane` and `Gyroid`.
    ///
    /// Gradient (for Session 19):
    ///   The fold is piecewise-identity, so `∇f_repeat(p) = ∇f_child(q(p))`
    ///   at non-boundary points.
    Repeat(Box<Self>, Vector3<f64>),

    /// Finite repetition with count per axis.
    ///
    /// Creates `count[i]` copies along each axis, centered at origin.
    /// For count `[3, 1, 1]` with spacing `(5, 1, 1)`, copies are at
    /// x ∈ {−5, 0, 5}.
    ///
    /// Unlike [`Repeat`], has finite bounds and can be meshed directly.
    /// Uses clamped repetition index — outermost cells extend to infinity
    /// (no wrapping at boundaries).
    ///
    /// Gradient (for Session 19):
    ///   Same as child gradient at folded point (same reasoning as Repeat).
    RepeatBounded {
        child: Box<Self>,
        spacing: Vector3<f64>,
        count: [u32; 3],
    },

    /// Smooth union with spatially varying blend radius.
    ///
    /// Like [`SmoothUnion`] but the blend radius `k` is determined by a
    /// user-provided closure evaluated at each point. `max_k` is the upper
    /// bound on the radius function, used for conservative interval
    /// evaluation.
    SmoothUnionVariable {
        a: Box<Self>,
        b: Box<Self>,
        radius_fn: UserEvalFn,
        max_k: f64,
    },

    // ── User escape hatch ─────────────────────────────────────────────
    /// User-provided function leaf node. Allows custom implicit surface
    /// functions that the expression tree does not natively support.
    ///
    /// If `interval` is `None`, interval evaluation returns `(-∞, +∞)`,
    /// disabling octree pruning for this subtree.
    UserFn {
        eval: UserEvalFn,
        interval: Option<UserIntervalFn>,
        /// Bounding box of the geometry — used by the mesher to define
        /// the evaluation domain.
        bounds: Aabb,
    },
}

// ── Parameter tree-walk helpers ────────────────────────────────────────

impl FieldNode {
    /// Collect all `Val` references in this node (not recursing into children).
    fn local_vals(&self) -> Vec<&Val> {
        match self {
            Self::Sphere { radius } => vec![radius],
            Self::SmoothUnion(_, _, k)
            | Self::SmoothSubtract(_, _, k)
            | Self::SmoothIntersect(_, _, k)
            | Self::SmoothUnionAll(_, k) => vec![k],
            Self::Shell(_, v) | Self::Round(_, v) | Self::Offset(_, v) => vec![v],
            _ => vec![],
        }
    }

    /// Iterate over all child `FieldNode` references.
    fn children(&self) -> Vec<&Self> {
        match self {
            Self::Union(a, b)
            | Self::Subtract(a, b)
            | Self::Intersect(a, b)
            | Self::SmoothUnion(a, b, _)
            | Self::SmoothSubtract(a, b, _)
            | Self::SmoothIntersect(a, b, _)
            | Self::SmoothUnionVariable { a, b, .. } => vec![a.as_ref(), b.as_ref()],
            Self::SmoothUnionAll(children, _) => children.iter().collect(),
            Self::Translate(c, _)
            | Self::Rotate(c, _)
            | Self::ScaleUniform(c, _)
            | Self::Mirror(c, _)
            | Self::Shell(c, _)
            | Self::Round(c, _)
            | Self::Offset(c, _)
            | Self::Elongate(c, _)
            | Self::Twist(c, _)
            | Self::Bend(c, _)
            | Self::Repeat(c, _) => vec![c.as_ref()],
            Self::RepeatBounded { child, .. } => vec![child.as_ref()],
            // Leaf nodes
            _ => vec![],
        }
    }

    /// Find the store and id for a parameter by name. Searches the entire subtree.
    pub(crate) fn find_param(&self, name: &str) -> Option<(usize, Arc<ParamStoreData>)> {
        // Check local vals
        for val in self.local_vals() {
            if let Val::Param { id, store } = val {
                if store.find_name(name) == Some(*id) {
                    return Some((*id, Arc::clone(store)));
                }
            }
        }
        // Recurse into children
        for child in self.children() {
            if let Some(result) = child.find_param(name) {
                return Some(result);
            }
        }
        None
    }

    /// Collect all parameter (name, id, store) triples from the entire subtree.
    pub(crate) fn collect_params(&self) -> Vec<(String, usize, Arc<ParamStoreData>)> {
        let mut result = Vec::new();
        self.collect_params_inner(&mut result);
        result
    }

    fn collect_params_inner(&self, out: &mut Vec<(String, usize, Arc<ParamStoreData>)>) {
        for val in self.local_vals() {
            if let Val::Param { id, store } = val {
                let name = store.name_of(*id);
                // Avoid duplicates (same param referenced multiple times)
                if !out.iter().any(|(n, i, _)| n == &name && *i == *id) {
                    out.push((name, *id, Arc::clone(store)));
                }
            }
        }
        for child in self.children() {
            child.collect_params_inner(out);
        }
    }
}
