//! Trilinear interpolation and gradient queries on SDF grids.

// Allow casting for grid indices - these are small values and bounds are checked
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap
)]

use nalgebra::{Point3, Vector3};

use super::SdfCollisionData;

impl SdfCollisionData {
    /// Convert local coordinates to grid coordinates.
    pub(super) fn local_to_grid(&self, point: Point3<f64>) -> (f64, f64, f64) {
        let gx = (point.x - self.origin.x) / self.cell_size;
        let gy = (point.y - self.origin.y) / self.cell_size;
        let gz = (point.z - self.origin.z) / self.cell_size;
        (gx, gy, gz)
    }

    /// Check if a point is within the grid bounds.
    #[must_use]
    pub fn contains(&self, point: Point3<f64>) -> bool {
        let (gx, gy, gz) = self.local_to_grid(point);
        gx >= 0.0
            && gy >= 0.0
            && gz >= 0.0
            && gx <= (self.width - 1) as f64
            && gy <= (self.height - 1) as f64
            && gz <= (self.depth - 1) as f64
    }

    /// Get the interpolated signed distance at a local-space point.
    ///
    /// Uses trilinear interpolation for smooth distance queries.
    /// Returns `None` if the point is outside the grid bounds.
    #[must_use]
    #[allow(clippy::similar_names)] // v000, v100, v00, v10, etc. are intentional for trilinear interp
    pub fn distance(&self, point: Point3<f64>) -> Option<f64> {
        let (gx, gy, gz) = self.local_to_grid(point);

        // Check bounds
        if gx < 0.0
            || gy < 0.0
            || gz < 0.0
            || gx > (self.width - 1) as f64
            || gy > (self.height - 1) as f64
            || gz > (self.depth - 1) as f64
        {
            return None;
        }

        // Get integer cell indices
        let x0 = gx.floor() as usize;
        let y0 = gy.floor() as usize;
        let z0 = gz.floor() as usize;
        let x1 = (x0 + 1).min(self.width - 1);
        let y1 = (y0 + 1).min(self.height - 1);
        let z1 = (z0 + 1).min(self.depth - 1);

        // Get fractional parts
        let fx = gx - x0 as f64;
        let fy = gy - y0 as f64;
        let fz = gz - z0 as f64;

        // Get corner values (8 corners of the cell)
        let v000 = self.values[z0 * self.width * self.height + y0 * self.width + x0];
        let v100 = self.values[z0 * self.width * self.height + y0 * self.width + x1];
        let v010 = self.values[z0 * self.width * self.height + y1 * self.width + x0];
        let v110 = self.values[z0 * self.width * self.height + y1 * self.width + x1];
        let v001 = self.values[z1 * self.width * self.height + y0 * self.width + x0];
        let v101 = self.values[z1 * self.width * self.height + y0 * self.width + x1];
        let v011 = self.values[z1 * self.width * self.height + y1 * self.width + x0];
        let v111 = self.values[z1 * self.width * self.height + y1 * self.width + x1];

        // Trilinear interpolation
        let v00 = v000 + fx * (v100 - v000);
        let v10 = v010 + fx * (v110 - v010);
        let v01 = v001 + fx * (v101 - v001);
        let v11 = v011 + fx * (v111 - v011);

        let v0 = v00 + fy * (v10 - v00);
        let v1 = v01 + fy * (v11 - v01);

        let v = v0 + fz * (v1 - v0);

        Some(v)
    }

    /// Get the signed distance, clamping to bounds if outside.
    ///
    /// # Robustness
    ///
    /// For points outside the grid bounds:
    /// 1. The point is clamped to the nearest grid boundary
    /// 2. If the clamped query still fails, returns `max_value` (treating the point as "far away")
    ///
    /// This ensures collision queries never panic, even for degenerate inputs.
    #[must_use]
    pub fn distance_clamped(&self, point: Point3<f64>) -> f64 {
        let clamped = Point3::new(
            point
                .x
                .clamp(self.origin.x, self.origin.x + self.extent_x()),
            point
                .y
                .clamp(self.origin.y, self.origin.y + self.extent_y()),
            point
                .z
                .clamp(self.origin.z, self.origin.z + self.extent_z()),
        );
        // Safe fallback: treat out-of-bounds as "very far away"
        self.distance(clamped).unwrap_or(self.max_value)
    }

    /// Get the gradient (surface normal direction) at a local-space point.
    ///
    /// Uses finite differences on the interpolated distance field.
    /// Returns `None` if the point is outside the grid bounds.
    #[must_use]
    pub fn gradient(&self, point: Point3<f64>) -> Option<Vector3<f64>> {
        // Use small offset for finite difference
        let eps = self.cell_size * 0.5;

        let d = self.distance(point)?;
        let dx = self.distance_clamped(Point3::new(point.x + eps, point.y, point.z));
        let dy = self.distance_clamped(Point3::new(point.x, point.y + eps, point.z));
        let dz = self.distance_clamped(Point3::new(point.x, point.y, point.z + eps));

        let grad = Vector3::new((dx - d) / eps, (dy - d) / eps, (dz - d) / eps);

        // Return normalized gradient, or zero if gradient is too small
        let norm = grad.norm();
        if norm > 1e-10 {
            Some(grad / norm)
        } else {
            Some(Vector3::z()) // Default to +Z if gradient is degenerate
        }
    }

    /// Get the surface normal at a local-space point.
    ///
    /// The normal points outward from the surface (in the direction of increasing distance).
    #[must_use]
    pub fn normal(&self, point: Point3<f64>) -> Option<Vector3<f64>> {
        self.gradient(point)
    }

    /// Get the gradient, clamping to bounds if outside.
    ///
    /// # Robustness
    ///
    /// For points outside the grid bounds:
    /// 1. The point is clamped to the nearest grid boundary
    /// 2. If the gradient is degenerate (zero norm), returns `+Z` (up vector)
    ///
    /// This ensures normal queries never panic, even for degenerate inputs.
    #[must_use]
    pub fn gradient_clamped(&self, point: Point3<f64>) -> Vector3<f64> {
        let clamped = Point3::new(
            point
                .x
                .clamp(self.origin.x, self.origin.x + self.extent_x()),
            point
                .y
                .clamp(self.origin.y, self.origin.y + self.extent_y()),
            point
                .z
                .clamp(self.origin.z, self.origin.z + self.extent_z()),
        );
        // Safe fallback: default to up vector for degenerate gradients
        self.gradient(clamped).unwrap_or_else(Vector3::z)
    }

    /// Find the closest point on the surface to a query point.
    ///
    /// Uses gradient descent starting from the query point.
    /// Returns `None` if the point is outside the grid bounds.
    ///
    /// `max_iterations` controls the Newton iteration count for surface refinement.
    /// MuJoCo ref: `mjOption.sdf_iterations` (default 10).
    #[must_use]
    pub fn closest_surface_point(
        &self,
        point: Point3<f64>,
        max_iterations: usize,
    ) -> Option<Point3<f64>> {
        let mut p = point;

        for _ in 0..max_iterations {
            let d = self.distance(p)?;
            if d.abs() < self.cell_size * 0.01 {
                // Close enough to surface
                return Some(p);
            }

            let grad = self.gradient(p)?;
            p -= grad * d; // Move toward surface

            // Clamp to bounds
            p = Point3::new(
                p.x.clamp(self.origin.x, self.origin.x + self.extent_x()),
                p.y.clamp(self.origin.y, self.origin.y + self.extent_y()),
                p.z.clamp(self.origin.z, self.origin.z + self.extent_z()),
            );
        }

        Some(p)
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::cast_precision_loss,
    clippy::items_after_statements
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn sphere_sdf(resolution: usize) -> SdfCollisionData {
        SdfCollisionData::sphere(Point3::origin(), 1.0, resolution, 1.0)
    }

    #[test]
    fn test_sdf_sphere_creation() {
        let sdf = sphere_sdf(16);
        assert_eq!(sdf.width(), 16);
        assert_eq!(sdf.height(), 16);
        assert_eq!(sdf.depth(), 16);
        assert!(sdf.min_value() < 0.0); // Inside the sphere
        assert!(sdf.max_value() > 0.0); // Outside the sphere
    }

    #[test]
    fn test_sdf_distance_at_center() {
        let sdf = sphere_sdf(32);
        let dist = sdf.distance(Point3::origin()).unwrap();
        // At center of unit sphere, distance should be approximately -1.0
        // (exact value depends on grid resolution and padding)
        assert_relative_eq!(dist, -1.0, epsilon = 0.15);
    }

    #[test]
    fn test_sdf_distance_at_surface() {
        let sdf = sphere_sdf(32);
        // At surface of unit sphere
        let dist = sdf.distance(Point3::new(1.0, 0.0, 0.0)).unwrap();
        assert_relative_eq!(dist, 0.0, epsilon = 0.1);
    }

    #[test]
    fn test_sdf_distance_outside() {
        let sdf = sphere_sdf(32);
        let dist = sdf.distance(Point3::new(1.5, 0.0, 0.0)).unwrap();
        assert_relative_eq!(dist, 0.5, epsilon = 0.1);
    }

    #[test]
    fn test_sdf_gradient_at_surface() {
        let sdf = sphere_sdf(32);
        // Gradient at +X surface should point +X
        let grad = sdf.gradient(Point3::new(1.0, 0.0, 0.0)).unwrap();
        assert!(grad.x > 0.8, "gradient.x should be positive");
        assert_relative_eq!(grad.norm(), 1.0, epsilon = 0.01);
    }

    #[test]
    fn test_sdf_aabb() {
        let sdf = SdfCollisionData::sphere(Point3::new(1.0, 2.0, 3.0), 1.0, 16, 0.5);
        let (min, max) = sdf.aabb();

        // AABB should contain the sphere with padding
        assert!(min.x < 0.0);
        assert!(min.y < 1.5);
        assert!(min.z < 2.5);
        assert!(max.x > 2.0);
        assert!(max.y > 2.5);
        assert!(max.z > 3.5);
    }

    #[test]
    fn test_sdf_box_shape() {
        let sdf =
            SdfCollisionData::box_shape(Point3::origin(), Vector3::new(0.5, 0.5, 0.5), 32, 0.5);

        // At center, should be inside (negative distance)
        let dist = sdf.distance(Point3::origin()).unwrap();
        assert!(dist < 0.0, "center should be inside box");

        // At corner, should be at surface (near zero)
        let dist = sdf.distance(Point3::new(0.5, 0.5, 0.5)).unwrap();
        assert_relative_eq!(dist, 0.0, epsilon = 0.1);

        // Outside the box
        let dist = sdf.distance(Point3::new(1.0, 0.0, 0.0)).unwrap();
        assert!(dist > 0.0, "should be outside box");
    }

    #[test]
    fn test_sdf_from_fn() {
        // Create a plane SDF: distance to z=0 plane
        let sdf = SdfCollisionData::from_fn(
            8,
            8,
            8,
            0.5,
            Point3::new(-2.0, -2.0, -2.0),
            |p| p.z, // signed distance to z=0 plane
        );

        // Points above z=0 should be positive
        let dist = sdf.distance(Point3::new(0.0, 0.0, 1.0)).unwrap();
        assert!(dist > 0.0);

        // Points below z=0 should be negative
        let dist = sdf.distance(Point3::new(0.0, 0.0, -1.0)).unwrap();
        assert!(dist < 0.0);
    }

    #[test]
    fn test_closest_surface_point_respects_max_iterations() {
        let sdf = sphere_sdf(32);

        // With 0 iterations, the result should be the input point (no refinement).
        let point = Point3::new(0.5, 0.0, 0.0);
        let result_0 = sdf.closest_surface_point(point, 0);
        assert!(result_0.is_some());
        let r0 = result_0.unwrap();
        assert_relative_eq!(r0.x, 0.5, epsilon = 1e-10);

        // With default iterations (10), should converge closer to the surface (r=1).
        let result_10 = sdf.closest_surface_point(point, 10);
        assert!(result_10.is_some());
        let r10 = result_10.unwrap();
        let dist_to_surface_10 = (r10.coords.norm() - 1.0).abs();
        assert!(
            dist_to_surface_10 < 0.15,
            "With 10 iterations, should converge near surface; got dist={dist_to_surface_10}"
        );
    }
}
