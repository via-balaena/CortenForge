//! Queries, mass properties, meshing, and SDF evaluation for [`Solid`].

use cf_geometry::{Aabb, SdfGrid};
use mesh_types::AttributedMesh;
use nalgebra::{Point3, Vector3};

use super::{ShapeHint, Solid};
use crate::field_node::{FieldNode, Val};

impl Solid {
    // ── Queries ──────────────────────────────────────────────────────

    /// Compute the axis-aligned bounding box of the geometry.
    ///
    /// Returns `None` for infinite geometry (e.g., a bare `Plane`).
    #[must_use]
    pub fn bounds(&self) -> Option<Aabb> {
        self.node.bounds()
    }

    /// Returns the sphere radius if this is a bare sphere primitive with a
    /// literal (non-parametric) radius, `None` otherwise.
    ///
    /// Used by the model builder to select `ShapeSphere` (analytical,
    /// rotation-invariant) over `ShapeConvex` (ray-marched).
    #[must_use]
    pub const fn sphere_radius(&self) -> Option<f64> {
        match &self.node {
            FieldNode::Sphere {
                radius: Val::Literal(r),
            } => Some(*r),
            _ => None,
        }
    }

    /// Inspect the expression tree to determine the physics shape category.
    ///
    /// Used by the model builder to select the appropriate `PhysicsShape`
    /// implementation:
    /// - `Sphere(radius)` → `ShapeSphere` (analytical, rotation-invariant)
    /// - `Convex` → `ShapeConvex` (ray-marched effective radius)
    /// - `Concave` → `ShapeConcave` (multi-contact surface tracing)
    ///
    /// Rules:
    /// - Bare sphere with literal radius → `Sphere(radius)`
    /// - Subtraction / smooth subtraction / shell → `Concave`
    /// - Transforms propagate the child's hint (translated sphere is still a sphere)
    /// - Unions / intersections: concave if any child is concave, else convex
    /// - All other primitives → `Convex`
    #[must_use]
    pub fn shape_hint(&self) -> ShapeHint {
        node_shape_hint(&self.node)
    }

    /// Compute the Lipschitz distortion factor of this solid's field.
    ///
    /// Returns the factor by which domain distortion operations (Twist, Bend,
    /// Loft taper) amplify the field gradient. For undistorted fields, returns
    /// 1.0.
    ///
    /// The mesher automatically divides cell size by this factor when calling
    /// [`mesh()`](Self::mesh), ensuring thin features in high-distortion
    /// regions are not silently lost. Use this value for manual resolution
    /// planning.
    #[must_use]
    pub fn lipschitz_factor(&self) -> f64 {
        self.node.lipschitz_factor()
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
    pub fn mesh(&self, tolerance: f64) -> AttributedMesh {
        assert!(
            tolerance > 0.0 && tolerance.is_finite(),
            "mesh tolerance must be positive and finite, got {tolerance}"
        );
        let Some(bounds) = self.node.bounds() else {
            return AttributedMesh::default();
        };
        // Scale cell size by Lipschitz distortion factor to ensure thin
        // features in high-distortion regions (Twist, Bend) are captured.
        let lip = self.node.lipschitz_factor();
        let cell_size = tolerance / lip;
        // Expand bounds by one cell to avoid surface clipping at edges
        let expanded = bounds.expanded(cell_size);
        let (mesh, _stats) = crate::mesher::mesh_field(&self.node, &expanded, cell_size);
        mesh
    }

    /// Extract a triangle mesh using dual contouring at the given tolerance.
    ///
    /// Like [`mesh`](Self::mesh), but uses dual contouring instead of
    /// marching cubes. DC places one vertex per sign-changing cell via QEF
    /// minimization, preserving sharp features (edges, corners) that MC
    /// rounds off.
    ///
    /// Returns an empty mesh for infinite geometry (bare `Plane`).
    ///
    /// # Panics
    ///
    /// Panics if `tolerance` is not positive and finite.
    #[must_use]
    pub fn mesh_dc(&self, tolerance: f64) -> AttributedMesh {
        assert!(
            tolerance > 0.0 && tolerance.is_finite(),
            "mesh tolerance must be positive and finite, got {tolerance}"
        );
        let Some(bounds) = self.node.bounds() else {
            return AttributedMesh::default();
        };
        let lip = self.node.lipschitz_factor();
        let cell_size = tolerance / lip;
        let expanded = bounds.expanded(cell_size);
        let (mesh, _stats) =
            crate::dual_contouring::mesh_field_dc(&self.node, &expanded, cell_size);
        mesh
    }

    /// Extract a triangle mesh using adaptive octree dual contouring.
    ///
    /// Like [`mesh_dc`](Self::mesh_dc), but uses an octree to subdivide only
    /// near the surface. Interior/exterior regions stop early, giving 10x+
    /// cell reduction vs the uniform grid while preserving QEF sharp features.
    ///
    /// Returns an empty mesh for infinite geometry (bare `Plane`).
    ///
    /// # Panics
    ///
    /// Panics if `tolerance` is not positive and finite.
    #[must_use]
    pub fn mesh_adaptive(&self, tolerance: f64) -> AttributedMesh {
        assert!(
            tolerance > 0.0 && tolerance.is_finite(),
            "mesh tolerance must be positive and finite, got {tolerance}"
        );
        let Some(bounds) = self.node.bounds() else {
            return AttributedMesh::default();
        };
        let lip = self.node.lipschitz_factor();
        let cell_size = tolerance / lip;
        let expanded = bounds.expanded(cell_size);
        let (mesh, _stats) =
            crate::adaptive_dc::mesh_field_adaptive(&self.node, &expanded, cell_size);
        mesh
    }

    /// Parallel adaptive octree dual contouring mesher.
    ///
    /// Like [`mesh_adaptive`](Self::mesh_adaptive) but uses rayon for:
    /// - Octree construction (parallel at top levels)
    /// - Phase 1 cell processing (parallel corner + gradient evaluation)
    /// - Batched evaluation (4 corners per tree walk via `evaluate_batch`)
    ///
    /// Returns an empty mesh for infinite geometry (bare `Plane`).
    ///
    /// Gated behind the `parallel-meshing` feature (default-on).
    /// Drop the feature to remove the rayon transitive chain; the
    /// sequential `mesh_adaptive` covers the same use case.
    ///
    /// # Panics
    ///
    /// Panics if `tolerance` is not positive and finite.
    #[cfg(feature = "parallel-meshing")]
    #[must_use]
    pub fn mesh_adaptive_par(&self, tolerance: f64) -> AttributedMesh {
        assert!(
            tolerance > 0.0 && tolerance.is_finite(),
            "mesh tolerance must be positive and finite, got {tolerance}"
        );
        let Some(bounds) = self.node.bounds() else {
            return AttributedMesh::default();
        };
        let lip = self.node.lipschitz_factor();
        let cell_size = tolerance / lip;
        let expanded = bounds.expanded(cell_size);
        let (mesh, _stats) =
            crate::adaptive_dc::mesh_field_adaptive_par(&self.node, &expanded, cell_size);
        mesh
    }

    /// Mesh the solid to a guaranteed maximum surface deviation.
    ///
    /// The output mesh surface is within `max_deviation` of the true implicit
    /// surface everywhere. Uses the parallel adaptive octree DC mesher with
    /// cell size derived from the requested deviation. For meshes under 50K
    /// faces, applies QEM simplification to reduce flat regions while staying
    /// within tolerance.
    ///
    /// Returns an empty mesh for infinite geometry (bare `Plane`).
    ///
    /// # Panics
    ///
    /// Panics if `max_deviation` is not positive and finite.
    #[must_use]
    pub fn mesh_to_tolerance(&self, max_deviation: f64) -> AttributedMesh {
        assert!(
            max_deviation > 0.0 && max_deviation.is_finite(),
            "max_deviation must be positive and finite, got {max_deviation}"
        );
        let Some(bounds) = self.node.bounds() else {
            return AttributedMesh::default();
        };

        let lip = self.node.lipschitz_factor();
        let cell_size = max_deviation / lip;
        let expanded = bounds.expanded(cell_size);
        // Sequential — keeps `mesh_to_tolerance` available without the
        // `parallel-meshing` feature. No production caller relies on
        // parallel here (grep confirms).
        let (mesh, _stats) =
            crate::adaptive_dc::mesh_field_adaptive(&self.node, &expanded, cell_size);

        // Only simplify when the mesh is small enough that QEM is fast.
        // Large meshes from fine tolerances are already well-resolved by DC.
        // Simplification operates on geometry only — it changes vertex
        // positions and counts, so any analytical normals we had become
        // stale and are dropped. Renderers fall back to crease-angle
        // splitting on the simplified mesh.
        if mesh.face_count() <= 50_000 {
            let simplified =
                crate::simplify::simplify_mesh_tolerance(&mesh.geometry, &self.node, max_deviation);
            AttributedMesh::from(simplified)
        } else {
            mesh
        }
    }

    /// Simplify this solid's mesh to a target face count using QEM edge collapse.
    ///
    /// Produces a lower-resolution mesh while preserving manifold topology
    /// and sharp features. The simplification uses Garland-Heckbert quadric
    /// error metrics.
    ///
    /// # Panics
    ///
    /// Panics if `tolerance` is not positive and finite, or if `target_faces` is zero.
    ///
    /// Simplification operates on geometry only; analytical normals from
    /// the underlying mesher are dropped (vertex count changes invalidate
    /// them). The returned mesh has `normals = None`.
    #[must_use]
    pub fn mesh_simplified(&self, tolerance: f64, target_faces: usize) -> AttributedMesh {
        // Use the sequential variant so this surface stays available
        // without the `parallel-meshing` feature. Callers needing the
        // parallel mesher must use `mesh_adaptive_par` directly with
        // the feature on.
        let mesh = self.mesh_adaptive(tolerance);
        AttributedMesh::from(crate::simplify::simplify_mesh(&mesh.geometry, target_faces))
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

    /// Compute the analytic gradient of the field at a point.
    ///
    /// Returns `∇f(p)` — the direction of steepest ascent. For exact SDFs,
    /// this is the outward unit normal on the surface. For approximate SDFs,
    /// the direction is correct but magnitude may differ from 1.
    ///
    /// Uses analytic derivatives for all built-in primitives and operations.
    /// Falls back to finite differences only for [`user_fn`](Self::user_fn).
    #[must_use]
    pub fn gradient(&self, point: &Point3<f64>) -> Vector3<f64> {
        self.node.gradient(point)
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
    // Precision loss acceptable for approximate values.
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

    /// Evaluate the field on a uniform 3D grid with a specific cell size,
    /// returning an [`SdfGrid`](cf_geometry::SdfGrid).
    ///
    /// This uses the same expansion and sizing logic that
    /// [`Mechanism::to_model()`](crate::Mechanism::to_model) uses, so the
    /// grid produced here is identical to the one physics will simulate.
    ///
    /// `cell_size` is in mm (e.g., 1.0 for 1 mm cells). The grid is padded
    /// by 2× `cell_size` on each side to capture the surface cleanly.
    ///
    /// Returns `None` for infinite geometry (e.g., a bare `Plane`).
    ///
    /// # Panics
    ///
    /// Panics if `cell_size` is not positive and finite.
    #[must_use]
    // Precision loss acceptable for approximate values.
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss
    )]
    pub fn sdf_grid_at(&self, cell_size: f64) -> Option<SdfGrid> {
        assert!(
            cell_size > 0.0 && cell_size.is_finite(),
            "cell_size must be positive and finite, got {cell_size}"
        );
        let bounds = self.node.bounds()?;
        let expanded = bounds.expanded(cell_size * 2.0);
        let size = expanded.size();

        let nx = ((size.x / cell_size).ceil() as usize).max(2);
        let ny = ((size.y / cell_size).ceil() as usize).max(2);
        let nz = ((size.z / cell_size).ceil() as usize).max(2);

        let node = &self.node;
        Some(SdfGrid::from_fn(nx, ny, nz, cell_size, expanded.min, |p| {
            node.evaluate(&p)
        }))
    }
}

/// Recursively inspect a `FieldNode` tree to determine the physics shape category.
fn node_shape_hint(node: &FieldNode) -> ShapeHint {
    match node {
        // ── Sphere primitive ─────────────────────────────────────────
        FieldNode::Sphere {
            radius: Val::Literal(r),
        } => ShapeHint::Sphere(*r),

        // ── Concave-producing operations ─────────────────────────────
        FieldNode::Subtract(..) | FieldNode::SmoothSubtract(..) | FieldNode::Shell(..) => {
            ShapeHint::Concave
        }

        // ── Transforms: propagate child's hint ───────────────────────
        FieldNode::Translate(child, _)
        | FieldNode::Rotate(child, _)
        | FieldNode::ScaleUniform(child, _)
        | FieldNode::Mirror(child, _)
        | FieldNode::Round(child, _)
        | FieldNode::Offset(child, _)
        | FieldNode::Elongate(child, _)
        | FieldNode::Twist(child, _)
        | FieldNode::Bend(child, _)
        | FieldNode::Repeat(child, _)
        | FieldNode::RepeatBounded { child, .. } => node_shape_hint(child),

        // ── Binary combinations: merge child hints ───────────────────
        FieldNode::Union(a, b)
        | FieldNode::SmoothUnion(a, b, _)
        | FieldNode::Intersect(a, b)
        | FieldNode::SmoothIntersect(a, b, _)
        | FieldNode::SmoothUnionVariable { a, b, .. } => {
            merge_hints(node_shape_hint(a), node_shape_hint(b))
        }

        // ── N-ary smooth union ───────────────────────────────────────
        FieldNode::SmoothUnionAll(children, _) => children
            .iter()
            .map(node_shape_hint)
            .fold(ShapeHint::Convex, merge_hints),

        // ── All other primitives/nodes: convex ───────────────────────
        _ => ShapeHint::Convex,
    }
}

/// Merge two shape hints. Concave wins over everything; Sphere is lost
/// when combined with anything else (union of sphere + box is not a sphere).
const fn merge_hints(a: ShapeHint, b: ShapeHint) -> ShapeHint {
    match (a, b) {
        (ShapeHint::Concave, _) | (_, ShapeHint::Concave) => ShapeHint::Concave,
        _ => ShapeHint::Convex,
    }
}
