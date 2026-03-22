//! ShapeSphere — analytical sphere with constant effective radius.

use std::sync::Arc;

use nalgebra::Vector3;

use crate::sdf::SdfGrid;
use crate::sdf::shape::PhysicsShape;

/// Sphere shape with analytical (rotation-invariant) effective radius.
///
/// Uses the known sphere radius directly — no ray-marching, no grid
/// sampling. Eliminates the rotation instability problem for spheres.
#[derive(Debug)]
pub struct ShapeSphere {
    grid: Arc<SdfGrid>,
    radius: f64,
}

impl ShapeSphere {
    /// Create a new sphere shape wrapping an SDF grid.
    #[must_use]
    pub fn new(grid: Arc<SdfGrid>, radius: f64) -> Self {
        Self { grid, radius }
    }

    /// The analytical sphere radius.
    #[must_use]
    pub fn radius(&self) -> f64 {
        self.radius
    }
}

impl PhysicsShape for ShapeSphere {
    fn effective_radius(&self, _local_dir: &Vector3<f64>) -> Option<f64> {
        Some(self.radius)
    }

    fn sdf_grid(&self) -> &SdfGrid {
        &self.grid
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn effective_radius_is_constant_for_any_direction() {
        let grid = Arc::new(SdfGrid::sphere(Point3::origin(), 5.0, 16, 2.0));
        let shape = ShapeSphere::new(grid, 5.0);

        let directions = [
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            -Vector3::x(),
            Vector3::new(1.0, 1.0, 1.0).normalize(),
            Vector3::new(-0.3, 0.7, -0.5).normalize(),
        ];

        for dir in &directions {
            let r = shape.effective_radius(dir);
            assert_eq!(r, Some(5.0), "radius should be constant for dir {dir:?}");
        }
    }

    #[test]
    fn sdf_grid_returns_underlying_grid() {
        let grid = Arc::new(SdfGrid::sphere(Point3::origin(), 2.0, 8, 1.0));
        let shape = ShapeSphere::new(grid.clone(), 2.0);
        assert_eq!(shape.sdf_grid().cell_size(), grid.cell_size());
    }
}
