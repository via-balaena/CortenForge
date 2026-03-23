//! `AnalyticalShape` — `PhysicsShape` backed by an analytical CSG tree.
//!
//! Wraps both an [`SdfGrid`] (for GPU/legacy) and a [`Solid`] (for exact
//! evaluation). All point queries (`distance`, `gradient`) delegate to the
//! Solid's CSG expression tree for machine-precision results. The
//! `evaluate_interval` method provides conservative field bounds for octree-
//! based contact detection.
//!
//! Created by the model builder for all cf-design shapes, replacing the
//! bare `ShapeSphere`/`ShapeConvex`/`ShapeConcave` types that delegate
//! everything to the grid.

use std::sync::Arc;

use cf_geometry::{Aabb, Bounded, SdfGrid};
use nalgebra::{Point3, Vector3};
use sim_core::PhysicsShape;

use crate::Solid;
use crate::solid::ShapeHint;

/// `PhysicsShape` backed by an analytical CSG tree.
///
/// Provides exact SDF evaluation, exact analytical gradients, and
/// conservative interval bounds for octree detection. The grid is
/// kept for GPU compute shaders and legacy primitive contacts.
#[derive(Debug)]
pub struct AnalyticalShape {
    grid: Arc<SdfGrid>,
    solid: Arc<Solid>,
    hint: ShapeHint,
}

impl AnalyticalShape {
    /// Create a new analytical shape.
    #[must_use]
    pub const fn new(grid: Arc<SdfGrid>, solid: Arc<Solid>, hint: ShapeHint) -> Self {
        Self { grid, solid, hint }
    }
}

impl PhysicsShape for AnalyticalShape {
    fn distance(&self, local_point: &Point3<f64>) -> Option<f64> {
        Some(self.solid.evaluate(local_point))
    }

    fn gradient(&self, local_point: &Point3<f64>) -> Option<Vector3<f64>> {
        let g = self.solid.gradient(local_point);
        let norm = g.norm();
        if norm > 1e-10 { Some(g / norm) } else { None }
    }

    fn bounds(&self) -> Aabb {
        self.solid.bounds().unwrap_or_else(|| self.grid.aabb())
    }

    fn effective_radius(&self, local_dir: &Vector3<f64>) -> Option<f64> {
        match self.hint {
            ShapeHint::Sphere(r) => Some(r),
            ShapeHint::Convex => ray_march_solid(&self.solid, local_dir),
            ShapeHint::Concave => None,
        }
    }

    fn prefers_single_contact(&self) -> bool {
        matches!(self.hint, ShapeHint::Sphere(_))
    }

    fn sdf_grid(&self) -> &SdfGrid {
        &self.grid
    }

    fn evaluate_interval(&self, local_aabb: &Aabb) -> Option<(f64, f64)> {
        let (lo, hi) = self.solid.evaluate_interval(local_aabb);

        // Loose-interval fallback (spec §5.7): if the interval span is much
        // wider than the AABB diagonal, the bounds are too conservative for
        // effective octree pruning (common for concave Subtract shapes with
        // high Lipschitz constants). Return None to force Tier 3 grid path.
        let span = hi - lo;
        let diagonal = local_aabb.diagonal();
        if span > diagonal * 10.0 {
            return None;
        }

        Some((lo, hi))
    }
}

/// Ray-march the analytical solid to find effective radius along a direction.
///
/// Same algorithm as `ShapeConvex::effective_radius` but uses the Solid's
/// exact `evaluate()` instead of the grid's trilinear interpolation.
#[allow(clippy::unnecessary_wraps)] // Returns Option to match effective_radius() trait contract
fn ray_march_solid(solid: &Solid, local_dir: &Vector3<f64>) -> Option<f64> {
    let Some(dir) = local_dir.try_normalize(1e-10) else {
        return Some(0.0);
    };
    let mut t = 0.0;
    // Use a small step minimum based on typical mechanism scales.
    // Grid-backed ShapeConvex uses cell_size * 0.01; we use a fixed
    // minimum since we don't have a cell size.
    let min_step = 1e-4;
    for _ in 0..200 {
        let point = Point3::from(dir * t);
        let dist = solid.evaluate(&point);
        if dist >= 0.0 {
            return Some(t);
        }
        t += (-dist).max(min_step);
    }
    Some(t)
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn sphere_shape(radius: f64) -> AnalyticalShape {
        let solid = Arc::new(Solid::sphere(radius));
        let grid = Arc::new(solid.sdf_grid_at(0.5).unwrap());
        AnalyticalShape::new(grid, solid, ShapeHint::Sphere(radius))
    }

    fn cylinder_shape(radius: f64, half_height: f64) -> AnalyticalShape {
        let solid = Arc::new(Solid::cylinder(radius, half_height));
        let grid = Arc::new(solid.sdf_grid_at(0.5).unwrap());
        AnalyticalShape::new(grid, solid, ShapeHint::Convex)
    }

    fn tube_shape(outer_r: f64, inner_r: f64, half_height: f64) -> AnalyticalShape {
        let solid = Arc::new(
            Solid::cylinder(outer_r, half_height).subtract(Solid::cylinder(inner_r, half_height)),
        );
        let grid = Arc::new(solid.sdf_grid_at(0.5).unwrap());
        AnalyticalShape::new(grid, solid, ShapeHint::Concave)
    }

    // ── distance() ──────────────────────────────────────────────────────

    #[test]
    fn sphere_distance_at_origin() {
        let shape = sphere_shape(5.0);
        let d = shape.distance(&Point3::origin()).unwrap();
        assert_relative_eq!(d, -5.0, epsilon = 1e-10);
    }

    #[test]
    fn sphere_distance_at_surface() {
        let shape = sphere_shape(5.0);
        let d = shape.distance(&Point3::new(5.0, 0.0, 0.0)).unwrap();
        assert_relative_eq!(d, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn sphere_distance_outside() {
        let shape = sphere_shape(5.0);
        let d = shape.distance(&Point3::new(7.0, 0.0, 0.0)).unwrap();
        assert_relative_eq!(d, 2.0, epsilon = 1e-10);
    }

    // ── gradient() ──────────────────────────────────────────────────────

    #[test]
    fn sphere_gradient_is_radial() {
        let shape = sphere_shape(5.0);
        let g = shape.gradient(&Point3::new(3.0, 4.0, 0.0)).unwrap();
        // Should point radially: (3, 4, 0) / 5
        assert_relative_eq!(g.x, 0.6, epsilon = 1e-10);
        assert_relative_eq!(g.y, 0.8, epsilon = 1e-10);
        assert_relative_eq!(g.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn cylinder_gradient_is_radial_on_barrel() {
        let shape = cylinder_shape(5.0, 10.0);
        // Point on the barrel surface, mid-height
        let g = shape.gradient(&Point3::new(5.0, 0.0, 0.0)).unwrap();
        // Should be purely radial: (1, 0, 0) — no axial component
        assert_relative_eq!(g.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(g.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(g.z, 0.0, epsilon = 1e-6);
    }

    // ── bounds() ────────────────────────────────────────────────────────

    #[test]
    fn sphere_bounds_correct() {
        let shape = sphere_shape(5.0);
        let b = shape.bounds();
        assert_relative_eq!(b.min.x, -5.0, epsilon = 1e-10);
        assert_relative_eq!(b.max.x, 5.0, epsilon = 1e-10);
    }

    // ── effective_radius() ──────────────────────────────────────────────

    #[test]
    fn sphere_effective_radius_is_constant() {
        let shape = sphere_shape(5.0);
        let dirs = [Vector3::x(), Vector3::y(), Vector3::z(), -Vector3::x()];
        for dir in &dirs {
            assert_relative_eq!(shape.effective_radius(dir).unwrap(), 5.0, epsilon = 1e-10);
        }
    }

    #[test]
    fn convex_effective_radius_ray_march() {
        let shape = cylinder_shape(5.0, 10.0);
        // Along X: should find the barrel at radius 5.0
        let r = shape.effective_radius(&Vector3::x()).unwrap();
        assert_relative_eq!(r, 5.0, epsilon = 0.1);
    }

    #[test]
    fn concave_effective_radius_is_none() {
        let shape = tube_shape(5.0, 3.0, 10.0);
        assert!(shape.effective_radius(&Vector3::x()).is_none());
    }

    // ── evaluate_interval() ─────────────────────────────────────────────

    #[test]
    fn sphere_interval_contains_surface() {
        let shape = sphere_shape(5.0);
        // AABB that straddles the +X surface
        let aabb = Aabb::new(Point3::new(4.0, -1.0, -1.0), Point3::new(6.0, 1.0, 1.0));
        let (lo, hi) = shape.evaluate_interval(&aabb).unwrap();
        // Surface (d=0) should be within the interval
        assert!(lo <= 0.0, "lo={lo} should be <= 0");
        assert!(hi >= 0.0, "hi={hi} should be >= 0");
    }

    #[test]
    fn sphere_interval_outside_prunes() {
        let shape = sphere_shape(5.0);
        // AABB entirely outside the sphere
        let aabb = Aabb::new(Point3::new(7.0, 7.0, 7.0), Point3::new(8.0, 8.0, 8.0));
        let (lo, _hi) = shape.evaluate_interval(&aabb).unwrap();
        assert!(lo > 0.0, "lo={lo} should be > 0 (entirely outside)");
    }

    #[test]
    fn sphere_interval_inside_prunes() {
        let shape = sphere_shape(5.0);
        // AABB entirely inside the sphere
        let aabb = Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5));
        let (_lo, hi) = shape.evaluate_interval(&aabb).unwrap();
        assert!(hi < 0.0, "hi={hi} should be < 0 (entirely inside)");
    }

    // ── sdf_grid() ──────────────────────────────────────────────────────

    #[test]
    fn sdf_grid_available() {
        let shape = sphere_shape(5.0);
        assert!(shape.sdf_grid().cell_size() > 0.0);
    }
}
