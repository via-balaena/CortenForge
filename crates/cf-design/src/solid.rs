//! Opaque design primitive wrapping a field expression tree.
//!
//! `Solid` is the public API for constructing and querying implicit surface
//! fields. Consumers never see `FieldNode` — they interact entirely through
//! `Solid` methods.
//!
//! The internal representation can change without breaking downstream code
//! (expression tree today, B-Rep in the future — see `CF_DESIGN_SPEC` §8).

use std::sync::Arc;

use cf_geometry::{Aabb, IndexedMesh, SdfGrid};
use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::field_node::{FieldNode, UserEvalFn, UserIntervalFn};

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

impl Solid {
    // ── Primitive constructors ────────────────────────────────────────

    /// Sphere centered at origin with the given radius.
    ///
    /// Exact SDF.
    ///
    /// # Panics
    ///
    /// Panics if `radius` is not positive and finite.
    #[must_use]
    pub fn sphere(radius: f64) -> Self {
        assert!(
            radius > 0.0 && radius.is_finite(),
            "sphere radius must be positive and finite, got {radius}"
        );
        Self {
            node: FieldNode::Sphere { radius },
        }
    }

    /// Axis-aligned box centered at origin with the given half-extents.
    ///
    /// Exact SDF.
    ///
    /// # Panics
    ///
    /// Panics if any half-extent is not positive and finite.
    #[must_use]
    pub fn cuboid(half_extents: Vector3<f64>) -> Self {
        assert!(
            half_extents.iter().all(|&v| v > 0.0 && v.is_finite()),
            "cuboid half_extents must be positive and finite, got {half_extents:?}"
        );
        Self {
            node: FieldNode::Cuboid { half_extents },
        }
    }

    /// Z-aligned cylinder centered at origin.
    ///
    /// Exact SDF. The cylinder extends from `z = -half_height` to
    /// `z = half_height` with the given radius.
    ///
    /// # Panics
    ///
    /// Panics if `radius` or `half_height` is not positive and finite.
    #[must_use]
    pub fn cylinder(radius: f64, half_height: f64) -> Self {
        assert!(
            radius > 0.0 && radius.is_finite(),
            "cylinder radius must be positive and finite, got {radius}"
        );
        assert!(
            half_height > 0.0 && half_height.is_finite(),
            "cylinder half_height must be positive and finite, got {half_height}"
        );
        Self {
            node: FieldNode::Cylinder {
                radius,
                half_height,
            },
        }
    }

    /// Z-aligned capsule (cylinder with hemispherical caps) centered at origin.
    ///
    /// Exact SDF. The cylindrical segment extends from `z = -half_height` to
    /// `z = half_height`. Total height is `2 * (half_height + radius)`.
    ///
    /// # Panics
    ///
    /// Panics if `radius` is not positive and finite, or `half_height` is
    /// negative or non-finite.
    #[must_use]
    pub fn capsule(radius: f64, half_height: f64) -> Self {
        assert!(
            radius > 0.0 && radius.is_finite(),
            "capsule radius must be positive and finite, got {radius}"
        );
        assert!(
            half_height >= 0.0 && half_height.is_finite(),
            "capsule half_height must be non-negative and finite, got {half_height}"
        );
        Self {
            node: FieldNode::Capsule {
                radius,
                half_height,
            },
        }
    }

    /// Ellipsoid centered at origin with the given axis radii.
    ///
    /// **Not an exact SDF** — the field magnitude is approximate, but the
    /// zero-isosurface is correct. Safe for meshing; `shell()` and `round()`
    /// will produce non-uniform results.
    ///
    /// # Panics
    ///
    /// Panics if any radius is not positive and finite.
    #[must_use]
    pub fn ellipsoid(radii: Vector3<f64>) -> Self {
        assert!(
            radii.iter().all(|&v| v > 0.0 && v.is_finite()),
            "ellipsoid radii must be positive and finite, got {radii:?}"
        );
        Self {
            node: FieldNode::Ellipsoid { radii },
        }
    }

    /// Torus in the XY plane centered at origin.
    ///
    /// Exact SDF. `major` is the distance from the center to the tube center.
    /// `minor` is the tube radius.
    ///
    /// # Panics
    ///
    /// Panics if `major` or `minor` is not positive and finite.
    #[must_use]
    pub fn torus(major: f64, minor: f64) -> Self {
        assert!(
            major > 0.0 && major.is_finite(),
            "torus major radius must be positive and finite, got {major}"
        );
        assert!(
            minor > 0.0 && minor.is_finite(),
            "torus minor radius must be positive and finite, got {minor}"
        );
        Self {
            node: FieldNode::Torus { major, minor },
        }
    }

    /// Cone with apex at origin, expanding downward along -Z.
    ///
    /// Exact SDF. The base is at `z = -height` with the given `radius`.
    ///
    /// # Panics
    ///
    /// Panics if `radius` or `height` is not positive and finite.
    #[must_use]
    pub fn cone(radius: f64, height: f64) -> Self {
        assert!(
            radius > 0.0 && radius.is_finite(),
            "cone radius must be positive and finite, got {radius}"
        );
        assert!(
            height > 0.0 && height.is_finite(),
            "cone height must be positive and finite, got {height}"
        );
        Self {
            node: FieldNode::Cone { radius, height },
        }
    }

    /// Half-space defined by a plane. Points on the `normal` side are outside
    /// (positive); points on the opposite side are inside (negative).
    ///
    /// Exact SDF when `normal` is unit length.
    ///
    /// # Panics
    ///
    /// Panics if `normal` is zero-length or non-finite, or `offset` is
    /// non-finite.
    #[must_use]
    pub fn plane(normal: Vector3<f64>, offset: f64) -> Self {
        let len = normal.norm();
        assert!(
            len > 1e-12 && normal.iter().all(|v| v.is_finite()),
            "plane normal must be non-zero and finite, got {normal:?}"
        );
        assert!(
            offset.is_finite(),
            "plane offset must be finite, got {offset}"
        );
        // Normalize the normal to ensure exact SDF property.
        let unit_normal = normal / len;
        let scaled_offset = offset / len;
        Self {
            node: FieldNode::Plane {
                normal: unit_normal,
                offset: scaled_offset,
            },
        }
    }

    // ── Path-based primitives ────────────────────────────────────────

    /// Pipe along a polyline path with spherical cross-section.
    ///
    /// Exact SDF. The pipe follows the straight-line segments connecting the
    /// vertices with the given radius. Corners are naturally rounded via the
    /// min-of-segments formulation.
    ///
    /// # Panics
    ///
    /// Panics if fewer than 2 vertices, if `radius` is not positive and finite,
    /// or if any vertex coordinate is non-finite.
    #[must_use]
    pub fn pipe(vertices: Vec<Point3<f64>>, radius: f64) -> Self {
        assert!(
            vertices.len() >= 2,
            "pipe requires at least 2 vertices, got {}",
            vertices.len()
        );
        assert!(
            radius > 0.0 && radius.is_finite(),
            "pipe radius must be positive and finite, got {radius}"
        );
        assert!(
            vertices
                .iter()
                .all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite()),
            "pipe vertices must have finite coordinates"
        );
        Self {
            node: FieldNode::Pipe { vertices, radius },
        }
    }

    /// Pipe along a Catmull-Rom spline with spherical cross-section.
    ///
    /// Near-exact SDF. The spline smoothly interpolates through the control
    /// points. Uses Catmull-Rom interpolation with open-curve endpoint
    /// handling.
    ///
    /// # Panics
    ///
    /// Panics if fewer than 2 control points, if `radius` is not positive and
    /// finite, or if any control point coordinate is non-finite.
    #[must_use]
    pub fn pipe_spline(control_points: Vec<Point3<f64>>, radius: f64) -> Self {
        assert!(
            control_points.len() >= 2,
            "pipe_spline requires at least 2 control points, got {}",
            control_points.len()
        );
        assert!(
            radius > 0.0 && radius.is_finite(),
            "pipe_spline radius must be positive and finite, got {radius}"
        );
        assert!(
            control_points
                .iter()
                .all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite()),
            "pipe_spline control points must have finite coordinates"
        );
        Self {
            node: FieldNode::PipeSpline {
                control_points,
                radius,
            },
        }
    }

    // ── Boolean operations ────────────────────────────────────────────

    /// Union of two solids: material where either solid exists.
    ///
    /// `min(self, other)`. Preserves SDF lower-bound property.
    #[must_use]
    pub fn union(self, other: Self) -> Self {
        Self {
            node: FieldNode::Union(Box::new(self.node), Box::new(other.node)),
        }
    }

    /// Subtract `other` from `self`: material where `self` exists but `other`
    /// does not.
    ///
    /// `max(self, -other)`.
    #[must_use]
    pub fn subtract(self, other: Self) -> Self {
        Self {
            node: FieldNode::Subtract(Box::new(self.node), Box::new(other.node)),
        }
    }

    /// Intersection of two solids: material where both solids exist.
    ///
    /// `max(self, other)`. Preserves SDF lower-bound property.
    #[must_use]
    pub fn intersect(self, other: Self) -> Self {
        Self {
            node: FieldNode::Intersect(Box::new(self.node), Box::new(other.node)),
        }
    }

    /// Smooth union — blends two solids with blend radius `k`.
    ///
    /// `k = 0` approaches sharp union. Larger `k` produces a wider organic
    /// fillet. The blend region adds material (field value decreases).
    ///
    /// # Panics
    ///
    /// Panics if `k` is not positive and finite.
    #[must_use]
    pub fn smooth_union(self, other: Self, k: f64) -> Self {
        assert!(
            k > 0.0 && k.is_finite(),
            "smooth_union blend radius k must be positive and finite, got {k}"
        );
        Self {
            node: FieldNode::SmoothUnion(Box::new(self.node), Box::new(other.node), k),
        }
    }

    /// Smooth subtraction — smoothly removes `other` from `self` with blend
    /// radius `k`.
    ///
    /// # Panics
    ///
    /// Panics if `k` is not positive and finite.
    #[must_use]
    pub fn smooth_subtract(self, other: Self, k: f64) -> Self {
        assert!(
            k > 0.0 && k.is_finite(),
            "smooth_subtract blend radius k must be positive and finite, got {k}"
        );
        Self {
            node: FieldNode::SmoothSubtract(Box::new(self.node), Box::new(other.node), k),
        }
    }

    /// Smooth intersection — smoothly intersects two solids with blend
    /// radius `k`. Removes material in the blend region.
    ///
    /// # Panics
    ///
    /// Panics if `k` is not positive and finite.
    #[must_use]
    pub fn smooth_intersect(self, other: Self, k: f64) -> Self {
        assert!(
            k > 0.0 && k.is_finite(),
            "smooth_intersect blend radius k must be positive and finite, got {k}"
        );
        Self {
            node: FieldNode::SmoothIntersect(Box::new(self.node), Box::new(other.node), k),
        }
    }

    /// Symmetric n-ary smooth union — blends multiple solids with blend
    /// radius `k`. Order-independent (unlike chaining binary `smooth_union`).
    ///
    /// Uses log-sum-exp internally for symmetric blending.
    ///
    /// # Panics
    ///
    /// Panics if `solids` is empty or `k` is not positive and finite.
    #[must_use]
    pub fn smooth_union_all(solids: Vec<Self>, k: f64) -> Self {
        assert!(
            !solids.is_empty(),
            "smooth_union_all requires at least one solid"
        );
        assert!(
            k > 0.0 && k.is_finite(),
            "smooth_union_all blend radius k must be positive and finite, got {k}"
        );
        let nodes: Vec<FieldNode> = solids.into_iter().map(|s| s.node).collect();
        Self {
            node: FieldNode::SmoothUnionAll(nodes, k),
        }
    }

    // ── Transforms ───────────────────────────────────────────────────

    /// Translate (move) the solid by the given offset.
    ///
    /// Preserves SDF property.
    ///
    /// # Panics
    ///
    /// Panics if any component of `offset` is non-finite.
    #[must_use]
    pub fn translate(self, offset: Vector3<f64>) -> Self {
        assert!(
            offset.iter().all(|v| v.is_finite()),
            "translate offset must be finite, got {offset:?}"
        );
        Self {
            node: FieldNode::Translate(Box::new(self.node), offset),
        }
    }

    /// Rotate the solid by the given unit quaternion.
    ///
    /// Preserves SDF property.
    #[must_use]
    pub fn rotate(self, rotation: UnitQuaternion<f64>) -> Self {
        Self {
            node: FieldNode::Rotate(Box::new(self.node), rotation),
        }
    }

    /// Uniformly scale the solid by the given factor.
    ///
    /// Preserves SDF property. Factor must be positive.
    ///
    /// # Panics
    ///
    /// Panics if `factor` is not positive and finite.
    #[must_use]
    pub fn scale_uniform(self, factor: f64) -> Self {
        assert!(
            factor > 0.0 && factor.is_finite(),
            "scale_uniform factor must be positive and finite, got {factor}"
        );
        Self {
            node: FieldNode::ScaleUniform(Box::new(self.node), factor),
        }
    }

    /// Mirror the solid across a plane through the origin with the given
    /// normal.
    ///
    /// The geometry on the positive side of the plane is reflected to the
    /// negative side.
    ///
    /// # Panics
    ///
    /// Panics if `normal` is zero-length or non-finite.
    #[must_use]
    pub fn mirror(self, normal: Vector3<f64>) -> Self {
        let len = normal.norm();
        assert!(
            len > 1e-12 && normal.iter().all(|v| v.is_finite()),
            "mirror normal must be non-zero and finite, got {normal:?}"
        );
        let unit_normal = normal / len;
        Self {
            node: FieldNode::Mirror(Box::new(self.node), unit_normal),
        }
    }

    // ── Domain operations ─────────────────────────────────────────────

    /// Shell — hollow out the solid to the given wall thickness.
    ///
    /// `|f(p)| - thickness`. The resulting solid is a thin shell around the
    /// original surface.
    ///
    /// **Requires exact SDF input** for uniform wall thickness. Applied to
    /// an f-rep or post-boolean field, wall thickness will be non-uniform.
    ///
    /// # Panics
    ///
    /// Panics if `thickness` is not positive and finite.
    #[must_use]
    pub fn shell(self, thickness: f64) -> Self {
        assert!(
            thickness > 0.0 && thickness.is_finite(),
            "shell thickness must be positive and finite, got {thickness}"
        );
        Self {
            node: FieldNode::Shell(Box::new(self.node), thickness),
        }
    }

    /// Round — add rounding to all edges.
    ///
    /// `f(p) - radius`. Shifts the isosurface outward, rounding all edges
    /// and corners. The solid grows by `radius` in all directions.
    ///
    /// **Requires exact SDF input** for uniform rounding.
    ///
    /// # Panics
    ///
    /// Panics if `radius` is not positive and finite.
    #[must_use]
    pub fn round(self, radius: f64) -> Self {
        assert!(
            radius > 0.0 && radius.is_finite(),
            "round radius must be positive and finite, got {radius}"
        );
        Self {
            node: FieldNode::Round(Box::new(self.node), radius),
        }
    }

    /// Offset — grow or shrink the shape uniformly.
    ///
    /// `f(p) - distance`. Positive distance grows (adds material), negative
    /// distance shrinks (removes material).
    ///
    /// Common use: manufacturing clearance adjustment:
    /// `pin.offset(-clearance / 2.0)` shrinks a pin by half the clearance.
    ///
    /// **Requires exact SDF input** for uniform offset.
    ///
    /// # Panics
    ///
    /// Panics if `distance` is not finite.
    #[must_use]
    pub fn offset(self, distance: f64) -> Self {
        assert!(
            distance.is_finite(),
            "offset distance must be finite, got {distance}"
        );
        Self {
            node: FieldNode::Offset(Box::new(self.node), distance),
        }
    }

    /// Elongate — stretch the shape along axes by inserting flat sections.
    ///
    /// `q = p - clamp(p, -h, h)`, then `f(q)`. Each axis is stretched by
    /// `2 * half_extents[axis]`. Preserves the SDF property.
    ///
    /// Example: `Solid::sphere(1.0).elongate(Vector3::new(2.0, 0.0, 0.0))`
    /// produces a capsule-like shape extending from x=-3 to x=3.
    ///
    /// # Panics
    ///
    /// Panics if any half-extent is negative or non-finite.
    #[must_use]
    pub fn elongate(self, half_extents: Vector3<f64>) -> Self {
        assert!(
            half_extents.iter().all(|&v| v >= 0.0 && v.is_finite()),
            "elongate half_extents must be non-negative and finite, got {half_extents:?}"
        );
        Self {
            node: FieldNode::Elongate(Box::new(self.node), half_extents),
        }
    }

    /// User-defined function leaf node.
    ///
    /// Escape hatch for custom implicit surface functions that the expression
    /// tree does not natively support (e.g., gyroids, superellipsoids,
    /// logarithmic spirals).
    ///
    /// - `eval` — closure mapping a point to a scalar field value.
    ///   Convention: negative inside, positive outside, zero on surface.
    /// - `bounds` — bounding box of the geometry (used by the mesher to
    ///   define the evaluation domain).
    ///
    /// For octree pruning, provide an interval function via
    /// [`Self::user_fn_with_interval`]. Without one, interval evaluation
    /// returns `(-∞, +∞)` and pruning is disabled for this subtree.
    #[must_use]
    pub fn user_fn(
        eval: impl Fn(Point3<f64>) -> f64 + Send + Sync + 'static,
        bounds: Aabb,
    ) -> Self {
        Self {
            node: FieldNode::UserFn {
                eval: UserEvalFn(Arc::new(eval)),
                interval: None,
                bounds,
            },
        }
    }

    /// User-defined function with interval bounds for octree pruning.
    ///
    /// Like [`Self::user_fn`], but with an additional closure that computes
    /// conservative `(min, max)` bounds of the field over a bounding box.
    ///
    /// The interval function must satisfy: for all points `p` in the `Aabb`,
    /// `lo <= eval(p) <= hi`. Loose bounds are safe (just reduce pruning
    /// efficiency); tight bounds improve meshing performance.
    #[must_use]
    pub fn user_fn_with_interval(
        eval: impl Fn(Point3<f64>) -> f64 + Send + Sync + 'static,
        interval: impl Fn(&Aabb) -> (f64, f64) + Send + Sync + 'static,
        bounds: Aabb,
    ) -> Self {
        Self {
            node: FieldNode::UserFn {
                eval: UserEvalFn(Arc::new(eval)),
                interval: Some(UserIntervalFn(Arc::new(interval))),
                bounds,
            },
        }
    }

    // ── Queries ──────────────────────────────────────────────────────

    /// Compute the axis-aligned bounding box of the geometry.
    ///
    /// Returns `None` for infinite geometry (e.g., a bare `Plane`).
    #[must_use]
    pub fn bounds(&self) -> Option<Aabb> {
        self.node.bounds()
    }

    /// Extract a triangle mesh at the given tolerance (voxel size).
    ///
    /// Smaller tolerance produces a finer mesh with more triangles.
    /// Returns an empty mesh for infinite geometry (bare `Plane`).
    ///
    /// The mesh is watertight, manifold, and uses CCW winding (outward
    /// normals) for all finite primitives and their compositions.
    ///
    /// # Panics
    ///
    /// Panics if `tolerance` is not positive and finite.
    #[must_use]
    pub fn mesh(&self, tolerance: f64) -> IndexedMesh {
        assert!(
            tolerance > 0.0 && tolerance.is_finite(),
            "mesh tolerance must be positive and finite, got {tolerance}"
        );
        let Some(bounds) = self.node.bounds() else {
            return IndexedMesh::new();
        };
        // Expand bounds by one cell to avoid surface clipping at edges
        let expanded = bounds.expanded(tolerance);
        let (mesh, _stats) = crate::mesher::mesh_field(&self.node, &expanded, tolerance);
        mesh
    }

    /// Evaluate the field at a point.
    ///
    /// Returns the signed distance (exact or approximate depending on the
    /// primitives involved). Negative = inside, positive = outside, zero =
    /// on surface.
    #[must_use]
    pub fn evaluate(&self, point: &Point3<f64>) -> f64 {
        self.node.evaluate(point)
    }

    /// Compute conservative (min, max) bounds of the field over an `Aabb`.
    ///
    /// The returned interval `(lo, hi)` satisfies:
    /// `lo <= self.evaluate(p) <= hi` for all `p` in the box.
    ///
    /// Used for octree pruning during meshing: if `lo > 0`, the box is fully
    /// outside; if `hi < 0`, fully inside.
    #[must_use]
    pub fn evaluate_interval(&self, aabb: &Aabb) -> (f64, f64) {
        self.node.evaluate_interval(aabb)
    }

    /// Evaluate the field on a uniform 3D grid, returning an
    /// [`SdfGrid`](cf_geometry::SdfGrid).
    ///
    /// `resolution` is the number of samples along the longest bounding-box
    /// axis (minimum 2). Other axes are scaled proportionally. The grid
    /// includes one cell of padding beyond the geometry bounds to capture
    /// the surface boundary cleanly.
    ///
    /// Returns `None` for infinite geometry (e.g., a bare `Plane`).
    ///
    /// # Panics
    ///
    /// Panics if `resolution < 2`.
    #[must_use]
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss
    )]
    pub fn sdf_grid(&self, resolution: usize) -> Option<SdfGrid> {
        assert!(
            resolution >= 2,
            "sdf_grid resolution must be at least 2, got {resolution}"
        );
        let bounds = self.node.bounds()?;
        let size = bounds.size();
        let longest = size.x.max(size.y).max(size.z);
        if longest <= 0.0 {
            return None;
        }
        let cell_size = longest / (resolution as f64 - 1.0);

        // Expand by one cell on each side for surface padding
        let expanded = bounds.expanded(cell_size);
        let exp_size = expanded.size();

        // +1 because N samples span N-1 intervals
        let nx = ((exp_size.x / cell_size).ceil() as usize + 1).max(2);
        let ny = ((exp_size.y / cell_size).ceil() as usize + 1).max(2);
        let nz = ((exp_size.z / cell_size).ceil() as usize + 1).max(2);

        let origin = expanded.min;
        let node = &self.node;

        Some(SdfGrid::from_fn(nx, ny, nz, cell_size, origin, |p| {
            node.evaluate(&p)
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    // ── Constructor validation ───────────────────────────────────────

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn sphere_rejects_zero_radius() {
        drop(Solid::sphere(0.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn sphere_rejects_negative_radius() {
        drop(Solid::sphere(-1.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn sphere_rejects_nan() {
        drop(Solid::sphere(f64::NAN));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn sphere_rejects_infinity() {
        drop(Solid::sphere(f64::INFINITY));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn cuboid_rejects_zero_extent() {
        drop(Solid::cuboid(Vector3::new(1.0, 0.0, 1.0)));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn cylinder_rejects_bad_radius() {
        drop(Solid::cylinder(-1.0, 2.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn capsule_rejects_bad_radius() {
        drop(Solid::capsule(0.0, 2.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn ellipsoid_rejects_bad_radii() {
        drop(Solid::ellipsoid(Vector3::new(1.0, -1.0, 1.0)));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn torus_rejects_bad_major() {
        drop(Solid::torus(0.0, 1.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn cone_rejects_bad_radius() {
        drop(Solid::cone(0.0, 2.0));
    }

    #[test]
    #[should_panic(expected = "non-zero and finite")]
    fn plane_rejects_zero_normal() {
        drop(Solid::plane(Vector3::zeros(), 0.0));
    }

    // ── Boolean validation ──────────────────────────────────────────

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn smooth_union_rejects_zero_k() {
        let a = Solid::sphere(1.0);
        let b = Solid::sphere(1.0);
        drop(a.smooth_union(b, 0.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn smooth_subtract_rejects_negative_k() {
        let a = Solid::sphere(1.0);
        let b = Solid::sphere(1.0);
        drop(a.smooth_subtract(b, -1.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn smooth_intersect_rejects_nan_k() {
        let a = Solid::sphere(1.0);
        let b = Solid::sphere(1.0);
        drop(a.smooth_intersect(b, f64::NAN));
    }

    #[test]
    #[should_panic(expected = "at least one solid")]
    fn smooth_union_all_rejects_empty() {
        drop(Solid::smooth_union_all(vec![], 1.0));
    }

    // ── Transform validation ────────────────────────────────────────

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn scale_uniform_rejects_zero() {
        drop(Solid::sphere(1.0).scale_uniform(0.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn scale_uniform_rejects_negative() {
        drop(Solid::sphere(1.0).scale_uniform(-1.0));
    }

    #[test]
    #[should_panic(expected = "non-zero and finite")]
    fn mirror_rejects_zero_normal() {
        drop(Solid::sphere(1.0).mirror(Vector3::zeros()));
    }

    // ── Evaluation through Solid ─────────────────────────────────────

    #[test]
    fn solid_sphere_evaluate() {
        let s = Solid::sphere(2.0);
        assert!((s.evaluate(&Point3::origin()) - (-2.0)).abs() < 1e-10);
        assert!((s.evaluate(&Point3::new(2.0, 0.0, 0.0))).abs() < 1e-10);
        assert!(s.evaluate(&Point3::new(5.0, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn solid_sphere_interval() {
        let s = Solid::sphere(2.0);
        let aabb = Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5));
        let (lo, hi) = s.evaluate_interval(&aabb);
        // Interval should be fully negative (inside sphere)
        assert!(hi < 0.0, "Expected negative interval for box inside sphere");
        assert!(lo < hi);
    }

    #[test]
    fn solid_cuboid_evaluate() {
        let c = Solid::cuboid(Vector3::new(1.0, 2.0, 3.0));
        assert!(c.evaluate(&Point3::origin()) < 0.0);
        assert!((c.evaluate(&Point3::new(1.0, 0.0, 0.0))).abs() < 1e-10);
    }

    #[test]
    fn solid_plane_normalizes() {
        // Non-unit normal should still produce correct SDF
        let p = Solid::plane(Vector3::new(0.0, 0.0, 2.0), 6.0);
        // Normalized: normal=(0,0,1), offset=3
        assert!((p.evaluate(&Point3::new(0.0, 0.0, 3.0))).abs() < 1e-10);
        assert!((p.evaluate(&Point3::new(0.0, 0.0, 5.0)) - 2.0).abs() < 1e-10);
    }

    #[test]
    fn capsule_zero_half_height_is_sphere() {
        let cap = Solid::capsule(2.0, 0.0);
        let sph = Solid::sphere(2.0);
        let test_points = [
            Point3::origin(),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ];
        for p in &test_points {
            assert!(
                (cap.evaluate(p) - sph.evaluate(p)).abs() < 1e-10,
                "Capsule(r=2, h=0) should match Sphere(r=2) at {p:?}"
            );
        }
    }

    // ── Boolean builder methods ──────────────────────────────────────

    #[test]
    fn solid_union_method() {
        let a = Solid::sphere(2.0);
        let b = Solid::sphere(3.0);
        let u = a.union(b);
        assert!((u.evaluate(&Point3::origin()) - (-3.0)).abs() < 1e-10);
    }

    #[test]
    fn solid_subtract_method() {
        let big = Solid::sphere(5.0);
        let small = Solid::sphere(2.0);
        let sub = big.subtract(small);
        // Origin: max(-5, 2) = 2
        assert!(sub.evaluate(&Point3::origin()) > 0.0);
        // Shell region: max(3-5, -(3-2)) = max(-2, -1) = -1
        assert!(sub.evaluate(&Point3::new(3.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn solid_intersect_method() {
        let a = Solid::sphere(5.0);
        let b = Solid::sphere(2.0);
        let inter = a.intersect(b);
        assert!((inter.evaluate(&Point3::origin()) - (-2.0)).abs() < 1e-10);
    }

    #[test]
    fn solid_smooth_union_method() {
        let a = Solid::sphere(2.0);
        let b = Solid::sphere(2.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let k = 1.0;
        let su = a.smooth_union(b, k);
        // In the blend region, smooth union adds material
        let p = Point3::new(1.5, 0.0, 0.0);
        // Should be more negative (more inside) than the sharper of the two
        assert!(su.evaluate(&p) < 1.0);
    }

    #[test]
    fn solid_smooth_union_all_method() {
        let solids = vec![
            Solid::sphere(2.0),
            Solid::sphere(2.0).translate(Vector3::new(3.0, 0.0, 0.0)),
            Solid::sphere(2.0).translate(Vector3::new(0.0, 3.0, 0.0)),
        ];
        let sua = Solid::smooth_union_all(solids, 1.0);
        // Should be inside near each sphere center
        assert!(sua.evaluate(&Point3::origin()) < 0.0);
        assert!(sua.evaluate(&Point3::new(3.0, 0.0, 0.0)) < 0.0);
        assert!(sua.evaluate(&Point3::new(0.0, 3.0, 0.0)) < 0.0);
    }

    // ── Transform builder methods ────────────────────────────────────

    #[test]
    fn solid_translate_method() {
        let s = Solid::sphere(1.0).translate(Vector3::new(5.0, 0.0, 0.0));
        assert!((s.evaluate(&Point3::new(5.0, 0.0, 0.0)) - (-1.0)).abs() < 1e-10);
        assert!((s.evaluate(&Point3::origin()) - 4.0).abs() < 1e-10);
    }

    #[test]
    fn solid_rotate_method() {
        let c = Solid::cuboid(Vector3::new(1.0, 2.0, 1.0));
        let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 2.0);
        let r = c.rotate(rot);
        // After 90° Z rotation: x-extent becomes 2, y-extent becomes 1
        assert!(r.evaluate(&Point3::new(1.5, 0.0, 0.0)) < 0.0);
        assert!(r.evaluate(&Point3::new(0.0, 1.5, 0.0)) > 0.0);
    }

    #[test]
    fn solid_scale_uniform_method() {
        let s = Solid::sphere(1.0).scale_uniform(3.0);
        assert!((s.evaluate(&Point3::origin()) - (-3.0)).abs() < 1e-10);
        assert!((s.evaluate(&Point3::new(3.0, 0.0, 0.0))).abs() < 1e-10);
    }

    #[test]
    fn solid_mirror_method() {
        let s = Solid::sphere(1.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let m = s.mirror(Vector3::x());
        // Mirrored: sphere at both (3,0,0) and (-3,0,0)
        assert!(m.evaluate(&Point3::new(3.0, 0.0, 0.0)) < 0.0);
        assert!(m.evaluate(&Point3::new(-3.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn solid_mirror_normalizes() {
        // Non-unit normal should still work correctly
        let s = Solid::sphere(1.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let m = s.mirror(Vector3::new(5.0, 0.0, 0.0)); // unnormalized
        assert!(m.evaluate(&Point3::new(-3.0, 0.0, 0.0)) < 0.0);
    }

    // ── Method chaining ──────────────────────────────────────────────

    #[test]
    fn solid_method_chaining() {
        // Build a hollowed, translated, scaled sphere.
        // Evaluation chain: scale(translate(subtract(sphere(5), sphere(3)), (10,0,0)), 2)
        // scale_uniform(2) evaluates f(p/2)*2, so effective center is (20,0,0),
        // effective outer radius = 10, effective inner radius = 6.
        let part = Solid::sphere(5.0)
            .subtract(Solid::sphere(3.0))
            .translate(Vector3::new(10.0, 0.0, 0.0))
            .scale_uniform(2.0);
        // Center at (20, 0, 0) — inside the hole (inner radius 6)
        assert!(part.evaluate(&Point3::new(20.0, 0.0, 0.0)) > 0.0);
        // On outer surface at x = 30 (center 20 + outer radius 10)
        assert!((part.evaluate(&Point3::new(30.0, 0.0, 0.0))).abs() < 1e-6);
        // Inside the shell at x = 28 (8 from center, between inner 6 and outer 10)
        assert!(part.evaluate(&Point3::new(28.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn solid_interval_for_union() {
        let u = Solid::sphere(2.0).union(Solid::sphere(3.0));
        let aabb = Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5));
        let (lo, hi) = u.evaluate_interval(&aabb);
        // Both spheres fully contain the box, interval should be negative
        assert!(hi < 0.0);
        assert!(lo < hi);
    }

    #[test]
    fn solid_interval_for_translate() {
        let s = Solid::sphere(2.0).translate(Vector3::new(5.0, 0.0, 0.0));
        // Box at origin: fully outside the translated sphere
        let aabb = Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5));
        let (lo, _hi) = s.evaluate_interval(&aabb);
        assert!(
            lo > 0.0,
            "Box at origin should be fully outside sphere at x=5"
        );
    }

    #[test]
    fn solid_interval_for_scale() {
        let s = Solid::sphere(1.0).scale_uniform(5.0);
        // Small box at origin: fully inside the scaled sphere
        let aabb = Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5));
        let (_lo, hi) = s.evaluate_interval(&aabb);
        assert!(hi < 0.0, "Box at origin should be fully inside sphere(r=5)");
    }

    // ── Domain operation builder methods ────────────────────────────

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn shell_rejects_zero_thickness() {
        drop(Solid::sphere(1.0).shell(0.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn shell_rejects_negative_thickness() {
        drop(Solid::sphere(1.0).shell(-1.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn round_rejects_zero_radius() {
        drop(Solid::sphere(1.0).round(0.0));
    }

    #[test]
    #[should_panic(expected = "must be finite")]
    fn offset_rejects_nan() {
        drop(Solid::sphere(1.0).offset(f64::NAN));
    }

    #[test]
    #[should_panic(expected = "non-negative and finite")]
    fn elongate_rejects_negative() {
        drop(Solid::sphere(1.0).elongate(Vector3::new(-1.0, 0.0, 0.0)));
    }

    #[test]
    fn solid_shell_method() {
        let s = Solid::sphere(5.0).shell(1.0);
        // Inner surface at r=4
        assert!((s.evaluate(&Point3::new(4.0, 0.0, 0.0))).abs() < 1e-10);
        // Outer surface at r=6
        assert!((s.evaluate(&Point3::new(6.0, 0.0, 0.0))).abs() < 1e-10);
        // Origin: outside the wall
        assert!(s.evaluate(&Point3::origin()) > 0.0);
        // Shell region: inside the wall
        assert!(s.evaluate(&Point3::new(5.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn solid_round_method() {
        let c = Solid::cuboid(Vector3::new(1.0, 1.0, 1.0)).round(0.5);
        // New surface at (1.5, 0, 0) — face moved out by radius
        assert!((c.evaluate(&Point3::new(1.5, 0.0, 0.0))).abs() < 1e-10);
        // Inside at (1.0, 0, 0)
        assert!(c.evaluate(&Point3::new(1.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn solid_offset_method() {
        // Grow sphere(3) by 2 → effective radius 5
        let s = Solid::sphere(3.0).offset(2.0);
        assert!((s.evaluate(&Point3::new(5.0, 0.0, 0.0))).abs() < 1e-10);
        assert!((s.evaluate(&Point3::origin()) - (-5.0)).abs() < 1e-10);

        // Shrink sphere(3) by 1 → effective radius 2
        let s = Solid::sphere(3.0).offset(-1.0);
        assert!((s.evaluate(&Point3::new(2.0, 0.0, 0.0))).abs() < 1e-10);
    }

    #[test]
    fn solid_elongate_method() {
        // Elongate sphere(1) by (2,0,0) → capsule from x=-3 to x=3
        let s = Solid::sphere(1.0).elongate(Vector3::new(2.0, 0.0, 0.0));
        // Surface at x = ±3
        assert!((s.evaluate(&Point3::new(3.0, 0.0, 0.0))).abs() < 1e-10);
        assert!((s.evaluate(&Point3::new(-3.0, 0.0, 0.0))).abs() < 1e-10);
        // Inside at x = 0
        assert!(s.evaluate(&Point3::origin()) < 0.0);
        // Y extent unchanged: surface at y = 1
        assert!((s.evaluate(&Point3::new(0.0, 1.0, 0.0))).abs() < 1e-10);
    }

    #[test]
    fn solid_elongate_zero_is_identity() {
        let s = Solid::sphere(2.0);
        let e = Solid::sphere(2.0).elongate(Vector3::new(0.0, 0.0, 0.0));
        let test_points = [
            Point3::origin(),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ];
        for p in &test_points {
            assert!(
                (s.evaluate(p) - e.evaluate(p)).abs() < 1e-10,
                "Elongate(0,0,0) should be identity at {p:?}"
            );
        }
    }

    // ── UserFn builder methods ──────────────────────────────────────

    #[test]
    fn solid_user_fn_method() {
        let s = Solid::user_fn(
            |p| p.coords.norm() - 3.0,
            Aabb::new(Point3::new(-4.0, -4.0, -4.0), Point3::new(4.0, 4.0, 4.0)),
        );
        assert!((s.evaluate(&Point3::origin()) - (-3.0)).abs() < 1e-10);
        assert!((s.evaluate(&Point3::new(3.0, 0.0, 0.0))).abs() < 1e-10);
        assert!(s.evaluate(&Point3::new(5.0, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn solid_user_fn_with_interval_method() {
        let s = Solid::user_fn_with_interval(
            |p| p.coords.norm() - 3.0,
            |aabb| {
                let closest = Point3::new(
                    0.0_f64.clamp(aabb.min.x, aabb.max.x),
                    0.0_f64.clamp(aabb.min.y, aabb.max.y),
                    0.0_f64.clamp(aabb.min.z, aabb.max.z),
                );
                let min_dist = closest.coords.norm();
                let max_dist = aabb
                    .corners()
                    .iter()
                    .map(|c| c.coords.norm())
                    .fold(0.0_f64, f64::max);
                (min_dist - 3.0, max_dist - 3.0)
            },
            Aabb::new(Point3::new(-4.0, -4.0, -4.0), Point3::new(4.0, 4.0, 4.0)),
        );
        // Interval should prune box fully outside
        let far_box = Aabb::new(Point3::new(5.0, 5.0, 5.0), Point3::new(6.0, 6.0, 6.0));
        let (lo, _) = s.evaluate_interval(&far_box);
        assert!(lo > 0.0, "Far box should be fully outside user sphere");
    }

    #[test]
    fn solid_user_fn_composes_with_booleans() {
        // UserFn sphere unioned with a regular sphere
        let custom = Solid::user_fn(
            |p| p.coords.norm() - 2.0,
            Aabb::new(Point3::new(-3.0, -3.0, -3.0), Point3::new(3.0, 3.0, 3.0)),
        );
        let regular = Solid::sphere(2.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let u = custom.union(regular);
        assert!(u.evaluate(&Point3::origin()) < 0.0);
        assert!(u.evaluate(&Point3::new(3.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn solid_user_fn_is_cloneable() {
        let s = Solid::user_fn(
            |p| p.coords.norm() - 1.0,
            Aabb::new(Point3::new(-2.0, -2.0, -2.0), Point3::new(2.0, 2.0, 2.0)),
        );
        let s2 = s.clone();
        assert!((s.evaluate(&Point3::origin()) - s2.evaluate(&Point3::origin())).abs() < 1e-10);
    }

    // ── Domain ops chaining ─────────────────────────────────────────

    #[test]
    fn shell_then_translate() {
        let s = Solid::sphere(5.0)
            .shell(1.0)
            .translate(Vector3::new(10.0, 0.0, 0.0));
        // Shell inner surface at x = 10+4 = 14, outer at x = 10+6 = 16
        assert!(s.evaluate(&Point3::new(10.0, 0.0, 0.0)) > 0.0); // center of shell (hollow)
        assert!(s.evaluate(&Point3::new(15.0, 0.0, 0.0)) < 0.0); // in the wall
        assert!((s.evaluate(&Point3::new(14.0, 0.0, 0.0))).abs() < 1e-10); // inner surface
        assert!((s.evaluate(&Point3::new(16.0, 0.0, 0.0))).abs() < 1e-10); // outer surface
    }

    #[test]
    fn offset_for_clearance() {
        // Pin-in-hole clearance test: pin shrinks, hole grows
        let pin = Solid::cylinder(2.0, 5.0).offset(-0.15);
        let hole = Solid::cylinder(2.0, 5.0).offset(0.15);
        // Pin surface at r = 1.85
        assert!((pin.evaluate(&Point3::new(1.85, 0.0, 0.0))).abs() < 1e-10);
        // Hole surface at r = 2.15
        assert!((hole.evaluate(&Point3::new(2.15, 0.0, 0.0))).abs() < 1e-10);
    }

    #[test]
    fn solid_interval_for_shell() {
        let s = Solid::sphere(10.0).shell(1.0);
        // Box at origin: deep inside sphere, so shell field is large positive
        let aabb = Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5));
        let (lo, _) = s.evaluate_interval(&aabb);
        assert!(lo > 0.0, "Box at origin should be outside shell wall");
    }

    // ── Pipe builder validation ──────────────────────────────────────

    #[test]
    #[should_panic(expected = "at least 2 vertices")]
    fn pipe_rejects_single_vertex() {
        drop(Solid::pipe(vec![Point3::origin()], 1.0));
    }

    #[test]
    #[should_panic(expected = "at least 2 vertices")]
    fn pipe_rejects_empty() {
        drop(Solid::pipe(vec![], 1.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn pipe_rejects_zero_radius() {
        drop(Solid::pipe(
            vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)],
            0.0,
        ));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn pipe_rejects_negative_radius() {
        drop(Solid::pipe(
            vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)],
            -1.0,
        ));
    }

    #[test]
    #[should_panic(expected = "finite coordinates")]
    fn pipe_rejects_nan_vertex() {
        drop(Solid::pipe(
            vec![Point3::new(f64::NAN, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
            1.0,
        ));
    }

    #[test]
    #[should_panic(expected = "at least 2 control points")]
    fn pipe_spline_rejects_single_point() {
        drop(Solid::pipe_spline(vec![Point3::origin()], 1.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn pipe_spline_rejects_inf_radius() {
        drop(Solid::pipe_spline(
            vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)],
            f64::INFINITY,
        ));
    }

    // ── Pipe builder methods ────────────────────────────────────────

    #[test]
    fn solid_pipe_evaluate() {
        let s = Solid::pipe(
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 0.0, 0.0)],
            1.0,
        );
        // Midpoint on axis: should be -radius
        assert!((s.evaluate(&Point3::new(5.0, 0.0, 0.0)) - (-1.0)).abs() < 1e-10);
        // On surface
        assert!((s.evaluate(&Point3::new(5.0, 1.0, 0.0))).abs() < 1e-10);
        // Outside
        assert!(s.evaluate(&Point3::new(5.0, 3.0, 0.0)) > 0.0);
    }

    #[test]
    fn solid_pipe_spline_evaluate() {
        let s = Solid::pipe_spline(
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 0.0, 0.0)],
            1.0,
        );
        // Start and end should be inside
        assert!(s.evaluate(&Point3::new(0.0, 0.0, 0.0)) < 0.0);
        assert!(s.evaluate(&Point3::new(10.0, 0.0, 0.0)) < 0.0);
        // Midpoint on surface
        assert!((s.evaluate(&Point3::new(5.0, 1.0, 0.0))).abs() < 1e-6);
    }

    #[test]
    fn solid_pipe_composes_with_translate() {
        let s = Solid::pipe(
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(5.0, 0.0, 0.0)],
            0.5,
        )
        .translate(Vector3::new(0.0, 0.0, 10.0));
        // Pipe should be at z=10 now
        assert!(s.evaluate(&Point3::new(2.5, 0.0, 10.0)) < 0.0);
        assert!(s.evaluate(&Point3::new(2.5, 0.0, 0.0)) > 0.0);
    }

    // ── Mesh validation helpers ────────────────────────────────────

    fn check_topology(mesh: &IndexedMesh) -> (bool, bool) {
        use std::collections::HashMap;
        let mut directed: HashMap<(u32, u32), usize> = HashMap::new();
        for face in &mesh.faces {
            for i in 0..3 {
                *directed.entry((face[i], face[(i + 1) % 3])).or_insert(0) += 1;
            }
        }
        let mut boundary = 0_usize;
        let mut non_manifold = 0_usize;
        for (&(a, b), &count) in &directed {
            if count > 1 {
                non_manifold += 1;
            }
            if directed.get(&(b, a)).copied().unwrap_or(0) == 0 {
                boundary += 1;
            }
        }
        (boundary == 0, non_manifold == 0)
    }

    fn assert_mesh_valid(mesh: &IndexedMesh, label: &str) {
        assert!(!mesh.is_empty(), "{label}: mesh should not be empty");
        let (watertight, manifold) = check_topology(mesh);
        assert!(watertight, "{label}: mesh should be watertight");
        assert!(manifold, "{label}: mesh should be manifold");
        assert!(
            mesh.signed_volume() > 0.0,
            "{label}: mesh should have positive signed volume (CCW winding), got {}",
            mesh.signed_volume()
        );
    }

    // ── Integration tests: composed trees → mesh → valid ───────

    #[test]
    fn integration_smooth_union_translated_spheres() {
        let a = Solid::sphere(3.0).translate(Vector3::new(-2.0, 0.0, 0.0));
        let b = Solid::sphere(3.0).translate(Vector3::new(2.0, 0.0, 0.0));
        let s = a.smooth_union(b, 1.0);
        let mesh = s.mesh(0.5);
        assert_mesh_valid(&mesh, "smooth_union_translated_spheres");
    }

    #[test]
    fn integration_subtract_rotated_cuboid_sphere() {
        let cuboid = Solid::cuboid(Vector3::new(3.0, 3.0, 3.0));
        let hole = Solid::sphere(2.0);
        let s = cuboid
            .subtract(hole)
            .rotate(UnitQuaternion::from_axis_angle(
                &Vector3::z_axis(),
                PI / 4.0,
            ));
        let mesh = s.mesh(0.4);
        assert_mesh_valid(&mesh, "subtract_rotated");
    }

    #[test]
    fn integration_shell_mirror() {
        let s = Solid::sphere(5.0)
            .shell(0.5)
            .translate(Vector3::new(3.0, 0.0, 0.0))
            .mirror(Vector3::x());
        let mesh = s.mesh(0.5);
        assert_mesh_valid(&mesh, "shell_mirror");
    }

    #[test]
    fn integration_elongate_smooth_intersect_cuboid() {
        let elongated = Solid::sphere(2.0).elongate(Vector3::new(3.0, 0.0, 0.0));
        let cuboid = Solid::cuboid(Vector3::new(4.0, 1.5, 1.5));
        let s = elongated.smooth_intersect(cuboid, 0.5);
        let mesh = s.mesh(0.3);
        assert_mesh_valid(&mesh, "elongate_smooth_intersect");
    }

    #[test]
    fn integration_pipe_union_sphere() {
        let pipe = Solid::pipe(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(5.0, 0.0, 0.0),
                Point3::new(5.0, 5.0, 0.0),
            ],
            0.8,
        );
        let ball = Solid::sphere(1.5).translate(Vector3::new(5.0, 5.0, 0.0));
        let s = pipe.union(ball);
        let mesh = s.mesh(0.3);
        assert_mesh_valid(&mesh, "pipe_union_sphere");
    }

    #[test]
    fn integration_multi_op_chain() {
        let s = Solid::sphere(3.0)
            .shell(0.5)
            .round(0.2)
            .translate(Vector3::new(5.0, 0.0, 0.0))
            .scale_uniform(2.0);
        let mesh = s.mesh(0.5);
        assert_mesh_valid(&mesh, "multi_op_chain");
    }

    // ── Interval pruning on composed trees ─────────────────────

    #[test]
    #[allow(clippy::cast_precision_loss)]
    fn pruning_ratio_union_translated_spheres() {
        let a = Solid::sphere(3.0).translate(Vector3::new(-3.0, 0.0, 0.0));
        let b = Solid::sphere(3.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let s = a.union(b);
        let bounds = s.bounds().map(|b| b.expanded(0.5));
        let (_, stats) = crate::mesher::mesh_field(&s.node, &bounds.unwrap_or(Aabb::empty()), 0.5);
        let ratio = stats.cells_pruned as f64 / stats.cells_total as f64;
        assert!(
            ratio > 0.70,
            "union pruning should be >70%, got {:.1}%",
            ratio * 100.0
        );
    }

    #[test]
    #[allow(clippy::cast_precision_loss)]
    fn pruning_ratio_subtract_spheres() {
        let big = Solid::sphere(5.0);
        let small = Solid::sphere(2.0);
        let s = big.subtract(small);
        let bounds = s.bounds().map(|b| b.expanded(0.5));
        let (_, stats) = crate::mesher::mesh_field(&s.node, &bounds.unwrap_or(Aabb::empty()), 0.5);
        let ratio = stats.cells_pruned as f64 / stats.cells_total as f64;
        assert!(
            ratio > 0.60,
            "subtract pruning should be >60%, got {:.1}%",
            ratio * 100.0
        );
    }

    #[test]
    #[allow(clippy::cast_precision_loss)]
    fn pruning_ratio_smooth_union_all_3_spheres() {
        let solids = vec![
            Solid::sphere(2.0),
            Solid::sphere(2.0).translate(Vector3::new(4.0, 0.0, 0.0)),
            Solid::sphere(2.0).translate(Vector3::new(0.0, 4.0, 0.0)),
        ];
        let s = Solid::smooth_union_all(solids, 1.0);
        let bounds = s.bounds().map(|b| b.expanded(0.5));
        let (_, stats) = crate::mesher::mesh_field(&s.node, &bounds.unwrap_or(Aabb::empty()), 0.5);
        let ratio = stats.cells_pruned as f64 / stats.cells_total as f64;
        assert!(
            ratio > 0.60,
            "smooth_union_all pruning should be >60%, got {:.1}%",
            ratio * 100.0
        );
    }

    // ── SdfGrid tests ─────────────────────────────────────────

    #[test]
    #[allow(clippy::cast_precision_loss, clippy::unwrap_used)]
    fn sdf_grid_matches_evaluate() {
        let s = Solid::sphere(3.0);
        let grid = s.sdf_grid(16).unwrap();
        let origin = grid.origin();
        let cs = grid.cell_size();
        // Spot-check grid values against point evaluation
        for &xi in &[0_usize, 5, 10] {
            for &yi in &[0_usize, 5, 10] {
                for &zi in &[0_usize, 5, 10] {
                    if xi < grid.width() && yi < grid.height() && zi < grid.depth() {
                        let p = Point3::new(
                            (xi as f64).mul_add(cs, origin.x),
                            (yi as f64).mul_add(cs, origin.y),
                            (zi as f64).mul_add(cs, origin.z),
                        );
                        let grid_val = grid.get(xi, yi, zi).unwrap();
                        let eval_val = s.evaluate(&p);
                        assert!(
                            (grid_val - eval_val).abs() < 1e-10,
                            "sdf_grid mismatch at ({xi},{yi},{zi}): grid={grid_val}, eval={eval_val}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn sdf_grid_resolution_dimensions() {
        // Cuboid half-extents (5,3,2) → full size (10,6,4). Longest axis = 10.
        let s = Solid::cuboid(Vector3::new(5.0, 3.0, 2.0));
        let grid = s.sdf_grid(20).unwrap();
        // Longest axis (x=10) gets 20 samples → cell_size = 10/19 ≈ 0.526
        // With 1-cell padding each side, width >= 20 + 2 = 22
        assert!(
            grid.width() >= 22,
            "expected width >= 22, got {}",
            grid.width()
        );
        assert!(grid.height() >= 2);
        assert!(grid.depth() >= 2);
    }

    #[test]
    fn sdf_grid_infinite_returns_none() {
        let s = Solid::plane(Vector3::z(), 0.0);
        assert!(s.sdf_grid(16).is_none());
    }

    #[test]
    #[should_panic(expected = "at least 2")]
    fn sdf_grid_rejects_resolution_1() {
        let s = Solid::sphere(1.0);
        drop(s.sdf_grid(1));
    }
}
