//! ShapeConvex — general convex shape using SDF ray-march for effective radius.

use std::sync::Arc;

use cf_geometry::{Aabb, Bounded};
use nalgebra::{Point3, Vector3};

use crate::sdf::SdfGrid;
use crate::sdf::shape::PhysicsShape;

/// General convex shape that ray-marches the SDF grid for effective radius.
///
/// Same behavior as the pre-trait code path. Used for CSG combinations
/// that are known-convex but have no analytical radius formula, and as
/// the default for MJCF-loaded SDF geoms (no shape metadata).
#[derive(Debug)]
pub struct ShapeConvex {
    grid: Arc<SdfGrid>,
}

impl ShapeConvex {
    /// Create a new convex shape wrapping an SDF grid.
    #[must_use]
    pub fn new(grid: Arc<SdfGrid>) -> Self {
        Self { grid }
    }
}

impl PhysicsShape for ShapeConvex {
    fn distance(&self, local_point: &Point3<f64>) -> Option<f64> {
        self.grid.distance(*local_point)
    }

    fn gradient(&self, local_point: &Point3<f64>) -> Option<Vector3<f64>> {
        self.grid.gradient(*local_point)
    }

    fn bounds(&self) -> Aabb {
        self.grid.aabb()
    }

    fn effective_radius(&self, local_dir: &Vector3<f64>) -> Option<f64> {
        // Sphere-trace from the SDF origin along the given direction to find
        // where the SDF value crosses zero — the effective radius along this
        // axis. Inlined from the former sdf_radius_along_axis().
        let Some(dir) = local_dir.try_normalize(1e-10) else {
            return Some(0.0);
        };
        let mut t = 0.0;
        for _ in 0..200 {
            let point = Point3::from(dir * t);
            let Some(dist) = self.grid.distance(point) else {
                return Some(t);
            };
            if dist >= 0.0 {
                return Some(t);
            }
            t += (-dist).max(self.grid.cell_size() * 0.01);
        }
        Some(t)
    }

    fn sdf_grid(&self) -> &SdfGrid {
        &self.grid
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    /// Verify that ShapeConvex::effective_radius produces accurate results
    /// for a sphere SDF grid (should be close to analytical radius).
    #[test]
    fn effective_radius_matches_analytical_for_sphere() {
        let radius = 5.0;
        let grid = Arc::new(SdfGrid::sphere(Point3::origin(), radius, 20, 2.0));
        let shape = ShapeConvex::new(grid);

        let directions = [
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            -Vector3::z(),
            Vector3::new(1.0, 1.0, 1.0).normalize(),
            Vector3::new(-0.3, 0.7, -0.5).normalize(),
        ];

        for dir in &directions {
            let shape_r = shape.effective_radius(dir).unwrap();
            // Sphere SDF: radius should be close to analytical value
            assert!(
                (shape_r - radius).abs() < 0.5,
                "radius {shape_r} too far from analytical {radius} for dir {dir:?}"
            );
        }
    }
}
