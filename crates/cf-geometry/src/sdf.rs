//! Signed distance field grid for implicit surface representation.
//!
//! [`SdfGrid`] stores a 3D grid of signed distance values representing
//! geometry implicitly:
//! - Positive values → outside the surface
//! - Negative values → inside the surface
//! - Zero → on the surface boundary
//! - The gradient at any point gives the surface normal direction
//!
//! # Coordinate System
//!
//! The grid is defined in local coordinates:
//! - Origin at the minimum corner of the bounding box
//! - Grid cells are uniform in all dimensions
//! - Values are stored in ZYX order (Z varies slowest)
//!
//! # Performance
//!
//! - O(1) distance queries with trilinear interpolation
//! - O(1) gradient/normal computation
//! - Memory usage: O(n³) for n×n×n grid
//!
//! # Robustness
//!
//! All public query methods handle out-of-bounds points gracefully:
//!
//! | Method | Out-of-Bounds Behavior |
//! |--------|------------------------|
//! | [`SdfGrid::distance`] | Returns `None` |
//! | [`SdfGrid::distance_clamped`] | Clamps to grid bounds, returns `max_value` as fallback |
//! | [`SdfGrid::gradient`] | Returns `None` |
//! | [`SdfGrid::gradient_clamped`] | Clamps to grid bounds, returns `+Z` as fallback |

// Allow casting for grid indices — these are small values and bounds are checked.
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap
)]

use nalgebra::{Point3, Vector3};

use crate::Aabb;
use crate::bounded::Bounded;

/// Signed distance field data on a uniform 3D grid.
///
/// Stores signed distance values for efficient geometric queries.
/// Values are stored in ZYX order (Z varies slowest, X varies fastest).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SdfGrid {
    /// Signed distance values in ZYX order.
    /// Access pattern: `values[z * width * height + y * width + x]`
    values: Vec<f64>,
    /// Number of samples along X axis.
    width: usize,
    /// Number of samples along Y axis.
    height: usize,
    /// Number of samples along Z axis.
    depth: usize,
    /// Size of each cell in meters (uniform in all dimensions).
    cell_size: f64,
    /// Origin of the grid in local coordinates (minimum corner).
    origin: Point3<f64>,
    /// Cached minimum SDF value (most negative = deepest inside).
    min_value: f64,
    /// Cached maximum SDF value (most positive = furthest outside).
    max_value: f64,
}

impl SdfGrid {
    /// Create a new SDF grid from distance values.
    ///
    /// # Panics
    ///
    /// Panics if `values.len() != width * height * depth`, if dimensions
    /// are zero, or if `cell_size` is not positive.
    #[must_use]
    pub fn new(
        values: Vec<f64>,
        width: usize,
        height: usize,
        depth: usize,
        cell_size: f64,
        origin: Point3<f64>,
    ) -> Self {
        assert!(
            width > 0 && height > 0 && depth > 0,
            "SDF dimensions must be positive"
        );
        assert!(
            values.len() == width * height * depth,
            "SDF data length {} doesn't match dimensions {}x{}x{}",
            values.len(),
            width,
            height,
            depth
        );
        assert!(cell_size > 0.0, "SDF cell_size must be positive");

        let (min_value, max_value) = values
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), &v| {
                (min.min(v), max.max(v))
            });

        Self {
            values,
            width,
            height,
            depth,
            cell_size,
            origin,
            min_value,
            max_value,
        }
    }

    /// Create an SDF grid from a signed distance function.
    #[must_use]
    pub fn from_fn<F>(
        width: usize,
        height: usize,
        depth: usize,
        cell_size: f64,
        origin: Point3<f64>,
        f: F,
    ) -> Self
    where
        F: Fn(Point3<f64>) -> f64,
    {
        let mut values = Vec::with_capacity(width * height * depth);

        for z in 0..depth {
            for y in 0..height {
                for x in 0..width {
                    let p = Point3::new(
                        (x as f64).mul_add(cell_size, origin.x),
                        (y as f64).mul_add(cell_size, origin.y),
                        (z as f64).mul_add(cell_size, origin.z),
                    );
                    values.push(f(p));
                }
            }
        }

        Self::new(values, width, height, depth, cell_size, origin)
    }

    /// Create an SDF grid representing a sphere.
    ///
    /// # Panics
    ///
    /// Panics if resolution is less than 2.
    #[must_use]
    pub fn sphere(center: Point3<f64>, radius: f64, resolution: usize, padding: f64) -> Self {
        assert!(resolution >= 2, "SDF resolution must be at least 2");
        let extent = radius + padding;
        let cell_size = (2.0 * extent) / (resolution - 1) as f64;
        let origin = Point3::new(center.x - extent, center.y - extent, center.z - extent);

        Self::from_fn(resolution, resolution, resolution, cell_size, origin, |p| {
            (p - center).norm() - radius
        })
    }

    /// Create an SDF grid representing a box.
    ///
    /// # Panics
    ///
    /// Panics if resolution is less than 2.
    #[must_use]
    pub fn box_shape(
        center: Point3<f64>,
        half_extents: Vector3<f64>,
        resolution: usize,
        padding: f64,
    ) -> Self {
        assert!(resolution >= 2, "SDF resolution must be at least 2");
        let max_extent = half_extents.x.max(half_extents.y).max(half_extents.z) + padding;
        let cell_size = (2.0 * max_extent) / (resolution - 1) as f64;
        let origin = Point3::new(
            center.x - max_extent,
            center.y - max_extent,
            center.z - max_extent,
        );

        Self::from_fn(resolution, resolution, resolution, cell_size, origin, |p| {
            let q = Vector3::new(
                (p.x - center.x).abs() - half_extents.x,
                (p.y - center.y).abs() - half_extents.y,
                (p.z - center.z).abs() - half_extents.z,
            );
            let outside = Vector3::new(q.x.max(0.0), q.y.max(0.0), q.z.max(0.0)).norm();
            let inside = q.x.max(q.y).max(q.z).min(0.0);
            outside + inside
        })
    }

    /// Get the grid width (samples along X).
    #[must_use]
    pub const fn width(&self) -> usize {
        self.width
    }

    /// Get the grid height (samples along Y).
    #[must_use]
    pub const fn height(&self) -> usize {
        self.height
    }

    /// Get the grid depth (samples along Z).
    #[must_use]
    pub const fn depth(&self) -> usize {
        self.depth
    }

    /// Get the cell size in meters.
    #[must_use]
    pub const fn cell_size(&self) -> f64 {
        self.cell_size
    }

    /// Get the origin (minimum corner) of the grid.
    #[must_use]
    pub const fn origin(&self) -> Point3<f64> {
        self.origin
    }

    /// Get the total X extent in meters.
    #[must_use]
    pub fn extent_x(&self) -> f64 {
        (self.width - 1) as f64 * self.cell_size
    }

    /// Get the total Y extent in meters.
    #[must_use]
    pub fn extent_y(&self) -> f64 {
        (self.height - 1) as f64 * self.cell_size
    }

    /// Get the total Z extent in meters.
    #[must_use]
    pub fn extent_z(&self) -> f64 {
        (self.depth - 1) as f64 * self.cell_size
    }

    /// Get the minimum SDF value (deepest inside the surface).
    #[must_use]
    pub const fn min_value(&self) -> f64 {
        self.min_value
    }

    /// Get the maximum SDF value (furthest outside the surface).
    #[must_use]
    pub const fn max_value(&self) -> f64 {
        self.max_value
    }

    /// Get the SDF value at grid coordinates (x, y, z).
    ///
    /// Returns `None` if coordinates are out of bounds.
    #[must_use]
    pub fn get(&self, x: usize, y: usize, z: usize) -> Option<f64> {
        if x < self.width && y < self.height && z < self.depth {
            Some(self.values[z * self.width * self.height + y * self.width + x])
        } else {
            None
        }
    }

    /// Get the SDF value at grid coordinates, clamping to bounds.
    #[must_use]
    pub fn get_clamped(&self, x: i32, y: i32, z: i32) -> f64 {
        let x = x.clamp(0, (self.width - 1) as i32) as usize;
        let y = y.clamp(0, (self.height - 1) as i32) as usize;
        let z = z.clamp(0, (self.depth - 1) as i32) as usize;
        self.values[z * self.width * self.height + y * self.width + x]
    }

    // ── Coordinate conversion ───────────────────────────────────────────

    /// Convert local coordinates to grid coordinates.
    fn local_to_grid(&self, point: Point3<f64>) -> (f64, f64, f64) {
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

    // ── Trilinear interpolation ─────────────────────────────────────────

    /// Get the interpolated signed distance at a local-space point.
    ///
    /// Uses trilinear interpolation for smooth distance queries.
    /// Returns `None` if the point is outside the grid bounds.
    #[must_use]
    #[allow(clippy::similar_names)]
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
        let wh = self.width * self.height;
        let w = self.width;
        let v000 = self.values[z0 * wh + y0 * w + x0];
        let v100 = self.values[z0 * wh + y0 * w + x1];
        let v010 = self.values[z0 * wh + y1 * w + x0];
        let v110 = self.values[z0 * wh + y1 * w + x1];
        let v001 = self.values[z1 * wh + y0 * w + x0];
        let v101 = self.values[z1 * wh + y0 * w + x1];
        let v011 = self.values[z1 * wh + y1 * w + x0];
        let v111 = self.values[z1 * wh + y1 * w + x1];

        // Trilinear interpolation
        let v00 = fx.mul_add(v100 - v000, v000);
        let v10 = fx.mul_add(v110 - v010, v010);
        let v01 = fx.mul_add(v101 - v001, v001);
        let v11 = fx.mul_add(v111 - v011, v011);

        let v0 = fy.mul_add(v10 - v00, v00);
        let v1 = fy.mul_add(v11 - v01, v01);

        let v = fz.mul_add(v1 - v0, v0);

        Some(v)
    }

    /// Get the signed distance, clamping to bounds if outside.
    ///
    /// For points outside the grid bounds, clamps to the nearest boundary.
    /// Falls back to `max_value` (treating the point as "far away") if
    /// the clamped query still fails.
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
        self.distance(clamped).unwrap_or(self.max_value)
    }

    // ── Gradient / normal ───────────────────────────────────────────────

    /// Get the gradient (surface normal direction) at a local-space point.
    ///
    /// Uses finite differences on the interpolated distance field.
    /// Returns `None` if the point is outside the grid bounds.
    /// Returns `+Z` if the gradient is degenerate (zero norm).
    #[must_use]
    pub fn gradient(&self, point: Point3<f64>) -> Option<Vector3<f64>> {
        let eps = self.cell_size * 0.5;

        let d = self.distance(point)?;
        let dx = self.distance_clamped(Point3::new(point.x + eps, point.y, point.z));
        let dy = self.distance_clamped(Point3::new(point.x, point.y + eps, point.z));
        let dz = self.distance_clamped(Point3::new(point.x, point.y, point.z + eps));

        let grad = Vector3::new((dx - d) / eps, (dy - d) / eps, (dz - d) / eps);

        let norm = grad.norm();
        if norm > 1e-10 {
            Some(grad / norm)
        } else {
            Some(Vector3::z())
        }
    }

    /// Get the surface normal at a local-space point.
    ///
    /// Alias for [`gradient`](Self::gradient). The normal points outward
    /// from the surface (in the direction of increasing distance).
    #[must_use]
    pub fn normal(&self, point: Point3<f64>) -> Option<Vector3<f64>> {
        self.gradient(point)
    }

    /// Get the gradient, clamping to bounds if outside.
    ///
    /// Clamps the point to the nearest grid boundary.
    /// Falls back to `+Z` (up vector) if the gradient is degenerate.
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
        self.gradient(clamped).unwrap_or_else(Vector3::z)
    }

    // ── Surface queries ─────────────────────────────────────────────────

    /// Find the closest point on the surface to a query point.
    ///
    /// Uses gradient descent starting from the query point.
    /// Returns `None` if the point is outside the grid bounds.
    ///
    /// `max_iterations` controls the Newton iteration count for surface
    /// refinement.
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
                return Some(p);
            }

            let grad = self.gradient(p)?;
            p -= grad * d;

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

impl Bounded for SdfGrid {
    fn aabb(&self) -> Aabb {
        Aabb::new(
            self.origin,
            Point3::new(
                self.origin.x + self.extent_x(),
                self.origin.y + self.extent_y(),
                self.origin.z + self.extent_z(),
            ),
        )
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::cast_precision_loss,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn sphere_sdf(resolution: usize) -> SdfGrid {
        SdfGrid::sphere(Point3::origin(), 1.0, resolution, 1.0)
    }

    // ── Construction ────────────────────────────────────────────────────

    #[test]
    fn sphere_creation() {
        let sdf = sphere_sdf(16);
        assert_eq!(sdf.width(), 16);
        assert_eq!(sdf.height(), 16);
        assert_eq!(sdf.depth(), 16);
        assert!(sdf.min_value() < 0.0);
        assert!(sdf.max_value() > 0.0);
    }

    #[test]
    fn box_creation() {
        let sdf = SdfGrid::box_shape(Point3::origin(), Vector3::new(0.5, 0.5, 0.5), 32, 0.5);
        assert!(sdf.min_value() < 0.0);
        assert!(sdf.max_value() > 0.0);
    }

    #[test]
    fn from_fn_plane() {
        let sdf = SdfGrid::from_fn(8, 8, 8, 0.5, Point3::new(-2.0, -2.0, -2.0), |p| p.z);
        let dist = sdf.distance(Point3::new(0.0, 0.0, 1.0)).unwrap();
        assert!(dist > 0.0);
        let dist = sdf.distance(Point3::new(0.0, 0.0, -1.0)).unwrap();
        assert!(dist < 0.0);
    }

    #[test]
    #[should_panic(expected = "dimensions must be positive")]
    fn zero_dimension_panics() {
        let _ = SdfGrid::new(vec![], 0, 1, 1, 1.0, Point3::origin());
    }

    #[test]
    #[should_panic(expected = "doesn't match dimensions")]
    fn wrong_length_panics() {
        let _ = SdfGrid::new(vec![0.0; 4], 2, 2, 2, 1.0, Point3::origin());
    }

    #[test]
    #[should_panic(expected = "cell_size must be positive")]
    fn negative_cell_size_panics() {
        let _ = SdfGrid::new(vec![0.0; 8], 2, 2, 2, -1.0, Point3::origin());
    }

    #[test]
    #[should_panic(expected = "resolution must be at least 2")]
    fn sphere_resolution_too_low() {
        let _ = SdfGrid::sphere(Point3::origin(), 1.0, 1, 0.5);
    }

    // ── Accessors ───────────────────────────────────────────────────────

    #[test]
    fn extents() {
        let sdf = SdfGrid::new(
            vec![0.0; 3 * 4 * 5],
            3,
            4,
            5,
            0.5,
            Point3::new(1.0, 2.0, 3.0),
        );
        assert_relative_eq!(sdf.extent_x(), 1.0); // (3-1) * 0.5
        assert_relative_eq!(sdf.extent_y(), 1.5); // (4-1) * 0.5
        assert_relative_eq!(sdf.extent_z(), 2.0); // (5-1) * 0.5
        assert_relative_eq!(sdf.origin().x, 1.0);
    }

    // ── Grid access ─────────────────────────────────────────────────────

    #[test]
    fn get_and_get_clamped() {
        let sdf = SdfGrid::new(
            vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0],
            2,
            2,
            2,
            1.0,
            Point3::origin(),
        );
        assert_relative_eq!(sdf.get(0, 0, 0).unwrap(), 1.0);
        assert_relative_eq!(sdf.get(1, 0, 0).unwrap(), 2.0);
        assert_relative_eq!(sdf.get(0, 1, 0).unwrap(), 3.0);
        assert_relative_eq!(sdf.get(0, 0, 1).unwrap(), 5.0);
        assert!(sdf.get(2, 0, 0).is_none());

        // Clamped: negative → 0, over-max → max
        assert_relative_eq!(sdf.get_clamped(-1, -1, -1), 1.0);
        assert_relative_eq!(sdf.get_clamped(10, 10, 10), 8.0);
    }

    // ── Contains ────────────────────────────────────────────────────────

    #[test]
    fn contains_bounds() {
        let sdf = sphere_sdf(16);
        assert!(sdf.contains(Point3::origin()));
        assert!(!sdf.contains(Point3::new(100.0, 0.0, 0.0)));
    }

    // ── Trilinear interpolation ─────────────────────────────────────────

    #[test]
    fn distance_at_center() {
        let sdf = sphere_sdf(32);
        let dist = sdf.distance(Point3::origin()).unwrap();
        assert_relative_eq!(dist, -1.0, epsilon = 0.15);
    }

    #[test]
    fn distance_at_surface() {
        let sdf = sphere_sdf(32);
        let dist = sdf.distance(Point3::new(1.0, 0.0, 0.0)).unwrap();
        assert_relative_eq!(dist, 0.0, epsilon = 0.1);
    }

    #[test]
    fn distance_outside() {
        let sdf = sphere_sdf(32);
        let dist = sdf.distance(Point3::new(1.5, 0.0, 0.0)).unwrap();
        assert_relative_eq!(dist, 0.5, epsilon = 0.1);
    }

    #[test]
    fn distance_out_of_bounds() {
        let sdf = sphere_sdf(16);
        assert!(sdf.distance(Point3::new(100.0, 0.0, 0.0)).is_none());
    }

    #[test]
    fn distance_clamped_always_returns() {
        let sdf = sphere_sdf(16);
        let d = sdf.distance_clamped(Point3::new(100.0, 100.0, 100.0));
        assert!(d.is_finite());
    }

    #[test]
    fn distance_box_at_center() {
        let sdf = SdfGrid::box_shape(Point3::origin(), Vector3::new(0.5, 0.5, 0.5), 32, 0.5);
        let dist = sdf.distance(Point3::origin()).unwrap();
        assert!(dist < 0.0, "center should be inside box");
    }

    #[test]
    fn distance_box_at_corner() {
        let sdf = SdfGrid::box_shape(Point3::origin(), Vector3::new(0.5, 0.5, 0.5), 32, 0.5);
        let dist = sdf.distance(Point3::new(0.5, 0.5, 0.5)).unwrap();
        assert_relative_eq!(dist, 0.0, epsilon = 0.1);
    }

    #[test]
    fn distance_box_outside() {
        let sdf = SdfGrid::box_shape(Point3::origin(), Vector3::new(0.5, 0.5, 0.5), 32, 0.5);
        let dist = sdf.distance(Point3::new(1.0, 0.0, 0.0)).unwrap();
        assert!(dist > 0.0, "should be outside box");
    }

    // ── Gradient / normal ───────────────────────────────────────────────

    #[test]
    fn gradient_at_surface() {
        let sdf = sphere_sdf(32);
        let grad = sdf.gradient(Point3::new(1.0, 0.0, 0.0)).unwrap();
        assert!(grad.x > 0.8, "gradient.x should be positive at +X surface");
        assert_relative_eq!(grad.norm(), 1.0, epsilon = 0.01);
    }

    #[test]
    fn normal_is_gradient_alias() {
        let sdf = sphere_sdf(32);
        let p = Point3::new(0.5, 0.5, 0.0);
        assert_eq!(sdf.gradient(p), sdf.normal(p));
    }

    #[test]
    fn gradient_clamped_always_returns() {
        let sdf = sphere_sdf(16);
        let g = sdf.gradient_clamped(Point3::new(100.0, 100.0, 100.0));
        assert!(g.norm() > 0.9);
    }

    // ── Closest surface point ───────────────────────────────────────────

    #[test]
    fn closest_surface_point_converges() {
        let sdf = sphere_sdf(32);
        let point = Point3::new(0.5, 0.0, 0.0);
        let result = sdf.closest_surface_point(point, 10).unwrap();
        let dist_to_surface = (result.coords.norm() - 1.0).abs();
        assert!(
            dist_to_surface < 0.15,
            "should converge near surface; got dist={dist_to_surface}"
        );
    }

    #[test]
    fn closest_surface_point_zero_iterations() {
        let sdf = sphere_sdf(32);
        let point = Point3::new(0.5, 0.0, 0.0);
        let result = sdf.closest_surface_point(point, 0).unwrap();
        assert_relative_eq!(result.x, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn closest_surface_point_outside_bounds() {
        let sdf = sphere_sdf(16);
        assert!(
            sdf.closest_surface_point(Point3::new(100.0, 0.0, 0.0), 10)
                .is_none()
        );
    }

    // ── Bounded impl ────────────────────────────────────────────────────

    #[test]
    fn bounded_aabb() {
        let sdf = SdfGrid::sphere(Point3::new(1.0, 2.0, 3.0), 1.0, 16, 0.5);
        let aabb = sdf.aabb();
        // AABB should contain the sphere with padding
        assert!(aabb.min.x < 0.0);
        assert!(aabb.min.y < 1.5);
        assert!(aabb.min.z < 2.5);
        assert!(aabb.max.x > 2.0);
        assert!(aabb.max.y > 2.5);
        assert!(aabb.max.z > 3.5);
    }

    #[test]
    fn bounded_aabb_exact() {
        let sdf = SdfGrid::new(
            vec![0.0; 3 * 4 * 5],
            3,
            4,
            5,
            0.5,
            Point3::new(1.0, 2.0, 3.0),
        );
        let aabb = sdf.aabb();
        assert_relative_eq!(aabb.min.x, 1.0);
        assert_relative_eq!(aabb.min.y, 2.0);
        assert_relative_eq!(aabb.min.z, 3.0);
        assert_relative_eq!(aabb.max.x, 2.0); // 1.0 + (3-1)*0.5
        assert_relative_eq!(aabb.max.y, 3.5); // 2.0 + (4-1)*0.5
        assert_relative_eq!(aabb.max.z, 5.0); // 3.0 + (5-1)*0.5
    }

    // ── Trilinear accuracy ──────────────────────────────────────────────

    #[test]
    fn trilinear_on_linear_field() {
        // SDF = x → trilinear should reproduce exactly
        let sdf = SdfGrid::from_fn(5, 5, 5, 1.0, Point3::origin(), |p| p.x);
        assert_relative_eq!(
            sdf.distance(Point3::new(2.5, 1.0, 1.0)).unwrap(),
            2.5,
            epsilon = 1e-10
        );
    }

    #[test]
    fn trilinear_on_constant_field() {
        // SDF = 42.0 everywhere
        let sdf = SdfGrid::from_fn(4, 4, 4, 1.0, Point3::origin(), |_| 42.0);
        assert_relative_eq!(
            sdf.distance(Point3::new(1.5, 1.5, 1.5)).unwrap(),
            42.0,
            epsilon = 1e-10
        );
    }

    #[test]
    fn trilinear_at_grid_corners() {
        // Values should exactly match at grid points
        let sdf = SdfGrid::from_fn(3, 3, 3, 1.0, Point3::origin(), |p| {
            p.x + p.y * 10.0 + p.z * 100.0
        });
        assert_relative_eq!(
            sdf.distance(Point3::new(0.0, 0.0, 0.0)).unwrap(),
            0.0,
            epsilon = 1e-10
        );
        assert_relative_eq!(
            sdf.distance(Point3::new(1.0, 1.0, 1.0)).unwrap(),
            111.0,
            epsilon = 1e-10
        );
        assert_relative_eq!(
            sdf.distance(Point3::new(2.0, 2.0, 2.0)).unwrap(),
            222.0,
            epsilon = 1e-10
        );
    }
}
