//! ShapeSphere — analytical sphere with constant effective radius.

use std::sync::Arc;

use cf_geometry::Aabb;
use nalgebra::{Point3, Vector3};

use crate::sdf::SdfGrid;
use crate::sdf::shape::PhysicsShape;

/// Compute the min/max distance from the origin over an AABB.
///
/// Returns (min_distance, max_distance) where distance = |p|.
/// Used for sphere interval evaluation.
fn norm_interval(aabb: &Aabb) -> (f64, f64) {
    // Closest point to origin (clamped)
    let cx = 0.0_f64.clamp(aabb.min.x, aabb.max.x);
    let cy = 0.0_f64.clamp(aabb.min.y, aabb.max.y);
    let cz = 0.0_f64.clamp(aabb.min.z, aabb.max.z);
    let min_norm = (cx * cx + cy * cy + cz * cz).sqrt();

    // Farthest corner from origin
    let fx = aabb.min.x.abs().max(aabb.max.x.abs());
    let fy = aabb.min.y.abs().max(aabb.max.y.abs());
    let fz = aabb.min.z.abs().max(aabb.max.z.abs());
    let max_norm = (fx * fx + fy * fy + fz * fz).sqrt();

    (min_norm, max_norm)
}

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
    fn distance(&self, local_point: &Point3<f64>) -> Option<f64> {
        Some(local_point.coords.norm() - self.radius)
    }

    fn gradient(&self, local_point: &Point3<f64>) -> Option<Vector3<f64>> {
        let norm = local_point.coords.norm();
        if norm > 1e-10 {
            Some(local_point.coords / norm)
        } else {
            None
        }
    }

    fn bounds(&self) -> Aabb {
        let r = self.radius;
        Aabb::new(Point3::new(-r, -r, -r), Point3::new(r, r, r))
    }

    fn effective_radius(&self, _local_dir: &Vector3<f64>) -> Option<f64> {
        Some(self.radius)
    }

    fn sdf_grid(&self) -> &SdfGrid {
        &self.grid
    }

    fn evaluate_interval(&self, local_aabb: &Aabb) -> Option<(f64, f64)> {
        let (min_norm, max_norm) = norm_interval(local_aabb);
        Some((min_norm - self.radius, max_norm - self.radius))
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
