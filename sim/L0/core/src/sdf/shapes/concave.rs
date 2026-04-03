//! ShapeConcave — non-convex shape forcing multi-contact surface tracing.

use std::sync::Arc;

use cf_geometry::{Aabb, Bounded};
use nalgebra::{Point3, Vector3};

use crate::sdf::SdfGrid;
use crate::sdf::shape::PhysicsShape;

/// Non-convex shape that always uses grid-based multi-contact surface tracing.
///
/// Returns `None` from `effective_radius`, so `compute_shape_contact` and
/// `compute_shape_plane_contact` always take the grid fallback path. Used for
/// sockets, gear teeth, hollow bodies, and any CSG result that involves
/// subtraction (which can create concavities).
#[derive(Debug)]
pub struct ShapeConcave {
    grid: Arc<SdfGrid>,
}

impl ShapeConcave {
    /// Create a new concave shape wrapping an SDF grid.
    #[must_use]
    pub fn new(grid: Arc<SdfGrid>) -> Self {
        Self { grid }
    }
}

impl PhysicsShape for ShapeConcave {
    fn distance(&self, local_point: &Point3<f64>) -> Option<f64> {
        self.grid.distance(*local_point)
    }

    fn gradient(&self, local_point: &Point3<f64>) -> Option<Vector3<f64>> {
        self.grid.gradient(*local_point)
    }

    fn bounds(&self) -> Aabb {
        self.grid.aabb()
    }

    fn effective_radius(&self, _local_dir: &Vector3<f64>) -> Option<f64> {
        None // Forces multi-contact surface tracing
    }

    fn sdf_grid(&self) -> &SdfGrid {
        &self.grid
    }

    fn sdf_grid_arc(&self) -> Arc<SdfGrid> {
        self.grid.clone()
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn effective_radius_always_none() {
        let grid = Arc::new(SdfGrid::sphere(Point3::origin(), 5.0, 16, 2.0));
        let shape = ShapeConcave::new(grid);

        let directions = [
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            -Vector3::x(),
            Vector3::new(1.0, 1.0, 1.0).normalize(),
            Vector3::new(-0.3, 0.7, -0.5).normalize(),
        ];

        for dir in &directions {
            assert_eq!(
                shape.effective_radius(dir),
                None,
                "concave shape must always return None for dir {dir:?}"
            );
        }
    }

    #[test]
    fn sdf_grid_returns_underlying_grid() {
        let grid = Arc::new(SdfGrid::sphere(Point3::origin(), 2.0, 8, 1.0));
        let shape = ShapeConcave::new(grid.clone());
        assert_eq!(shape.sdf_grid().cell_size(), grid.cell_size());
    }
}
