//! Height field grid for terrain representation.
//!
//! [`HeightFieldData`] stores a 2D grid of height values defining a 3D surface.
//! Used for terrain, ground surfaces, and procedurally generated environments.
//!
//! # Coordinate System
//!
//! The height field is defined in the XY plane with heights along Z:
//! - Origin is at the corner (0, 0) of the grid
//! - X axis spans `[0, (width - 1) * cell_size]`
//! - Y axis spans `[0, (depth - 1) * cell_size]`
//! - Z values come from the height data
//!
//! # Performance
//!
//! - O(1) cell lookup for point queries
//! - O(k) for shape queries where k is the number of cells intersected
//! - Efficient for large terrains due to implicit spatial structure
//!
//! # Robustness
//!
//! All public query methods handle out-of-bounds and edge-case inputs gracefully:
//!
//! | Method | Out-of-Bounds / Edge Case Behavior |
//! |--------|-------------------------------------|
//! | [`HeightFieldData::get`] | Returns `None` |
//! | [`HeightFieldData::get_clamped`] | Clamps indices to valid range |
//! | [`HeightFieldData::sample`] | Returns `None` (also guards against NaN) |
//! | [`HeightFieldData::sample_clamped`] | Clamps coords, falls back to `min_height` |
//! | [`HeightFieldData::normal`] | Returns `None`; degenerate gradients → `+Z` |
//! | [`HeightFieldData::normal_clamped`] | Clamps coords, falls back to `+Z` |
//! | [`HeightFieldData::cell_at`] | Returns `None` (also guards against NaN) |
//! | [`HeightFieldData::cells_in_aabb`] | Guards against NaN, returns empty iterator |

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

/// Height field collision data.
///
/// Stores a 2D grid of height values for terrain representation.
/// Heights are stored in row-major order (X varies fastest).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct HeightFieldData {
    /// Height values in row-major order.
    /// Access pattern: `heights[y * width + x]`
    heights: Vec<f64>,
    /// Number of columns (samples along X axis).
    width: usize,
    /// Number of rows (samples along Y axis).
    depth: usize,
    /// Size of each cell in meters.
    cell_size: f64,
    /// Minimum height value (cached for AABB).
    min_height: f64,
    /// Maximum height value (cached for AABB).
    max_height: f64,
}

impl HeightFieldData {
    /// Create a new height field from height data.
    ///
    /// # Panics
    ///
    /// Panics if `heights.len() != width * depth`, if dimensions are zero,
    /// or if `cell_size` is not positive.
    #[must_use]
    pub fn new(heights: Vec<f64>, width: usize, depth: usize, cell_size: f64) -> Self {
        assert!(
            width > 0 && depth > 0,
            "HeightField dimensions must be positive"
        );
        assert!(
            heights.len() == width * depth,
            "HeightField data length {} doesn't match dimensions {}x{}",
            heights.len(),
            width,
            depth
        );
        assert!(cell_size > 0.0, "HeightField cell_size must be positive");

        let (min_height, max_height) = heights
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), &h| {
                (min.min(h), max.max(h))
            });

        Self {
            heights,
            width,
            depth,
            cell_size,
            min_height,
            max_height,
        }
    }

    /// Create a flat height field at a given height.
    #[must_use]
    pub fn flat(width: usize, depth: usize, cell_size: f64, height: f64) -> Self {
        let heights = vec![height; width * depth];
        Self::new(heights, width, depth, cell_size)
    }

    /// Create a height field from a function.
    ///
    /// The function receives world-space (x, y) coordinates and returns the height.
    #[must_use]
    pub fn from_fn<F>(width: usize, depth: usize, cell_size: f64, f: F) -> Self
    where
        F: Fn(f64, f64) -> f64,
    {
        let mut heights = Vec::with_capacity(width * depth);
        for y in 0..depth {
            for x in 0..width {
                let wx = x as f64 * cell_size;
                let wy = y as f64 * cell_size;
                heights.push(f(wx, wy));
            }
        }
        Self::new(heights, width, depth, cell_size)
    }

    /// Get the width (number of columns).
    #[must_use]
    pub const fn width(&self) -> usize {
        self.width
    }

    /// Get the depth (number of rows).
    #[must_use]
    pub const fn depth(&self) -> usize {
        self.depth
    }

    /// Get the cell size in meters.
    #[must_use]
    pub const fn cell_size(&self) -> f64 {
        self.cell_size
    }

    /// Get the total X extent in meters.
    #[must_use]
    pub fn extent_x(&self) -> f64 {
        (self.width - 1) as f64 * self.cell_size
    }

    /// Get the total Y extent in meters.
    #[must_use]
    pub fn extent_y(&self) -> f64 {
        (self.depth - 1) as f64 * self.cell_size
    }

    /// Get the minimum height value.
    #[must_use]
    pub const fn min_height(&self) -> f64 {
        self.min_height
    }

    /// Get the maximum height value.
    #[must_use]
    pub const fn max_height(&self) -> f64 {
        self.max_height
    }

    /// Get the height at grid coordinates (x, y).
    ///
    /// Returns `None` if coordinates are out of bounds.
    #[must_use]
    pub fn get(&self, x: usize, y: usize) -> Option<f64> {
        if x < self.width && y < self.depth {
            Some(self.heights[y * self.width + x])
        } else {
            None
        }
    }

    /// Get the height at grid coordinates, with bounds clamping.
    #[must_use]
    pub fn get_clamped(&self, x: i32, y: i32) -> f64 {
        let x = x.clamp(0, (self.width - 1) as i32) as usize;
        let y = y.clamp(0, (self.depth - 1) as i32) as usize;
        self.heights[y * self.width + x]
    }

    /// Get the interpolated height at world-space (x, y) coordinates.
    ///
    /// Uses bilinear interpolation for smooth height queries.
    /// Returns `None` if the point is outside the height field bounds.
    #[must_use]
    pub fn sample(&self, x: f64, y: f64) -> Option<f64> {
        // Guard against NaN and negative inputs
        if x.is_nan() || y.is_nan() || x < 0.0 || y < 0.0 {
            return None;
        }

        let max_x = self.extent_x();
        let max_y = self.extent_y();

        if x > max_x || y > max_y {
            return None;
        }

        // Convert to grid coordinates
        let gx = x / self.cell_size;
        let gy = y / self.cell_size;

        // Get integer cell indices
        let x0 = gx.floor() as usize;
        let y0 = gy.floor() as usize;
        let x1 = (x0 + 1).min(self.width - 1);
        let y1 = (y0 + 1).min(self.depth - 1);

        // Get fractional parts
        let fx = gx - x0 as f64;
        let fy = gy - y0 as f64;

        // Get corner heights
        let h00 = self.heights[y0 * self.width + x0];
        let h10 = self.heights[y0 * self.width + x1];
        let h01 = self.heights[y1 * self.width + x0];
        let h11 = self.heights[y1 * self.width + x1];

        // Bilinear interpolation
        let h0 = fx.mul_add(h10 - h00, h00);
        let h1 = fx.mul_add(h11 - h01, h01);
        let h = fy.mul_add(h1 - h0, h0);

        Some(h)
    }

    /// Get the interpolated height, clamping to bounds if outside.
    ///
    /// Clamps coordinates to `[0, extent]` before sampling.
    /// Falls back to `min_height` if sampling still fails (e.g., `NaN` propagation).
    #[must_use]
    pub fn sample_clamped(&self, x: f64, y: f64) -> f64 {
        let x = x.clamp(0.0, self.extent_x());
        let y = y.clamp(0.0, self.extent_y());
        self.sample(x, y).unwrap_or(self.min_height)
    }

    /// Get the surface normal at world-space (x, y) coordinates.
    ///
    /// Uses finite differences to compute the gradient and normal.
    /// Returns `None` if the point is outside the height field bounds.
    #[must_use]
    pub fn normal(&self, x: f64, y: f64) -> Option<Vector3<f64>> {
        let eps = self.cell_size * 0.1;

        let hc = self.sample(x, y)?;
        let hx = self.sample_clamped(x + eps, y);
        let hy = self.sample_clamped(x, y + eps);

        // Compute gradient
        let dx = (hx - hc) / eps;
        let dy = (hy - hc) / eps;

        // Normal is perpendicular to gradient: (-dz/dx, -dz/dy, 1) normalized
        let raw = Vector3::new(-dx, -dy, 1.0);
        let norm = raw.norm();
        let normal = if norm.is_finite() && norm > 1e-10 {
            raw / norm
        } else {
            Vector3::z()
        };
        Some(normal)
    }

    /// Get the surface normal, clamping to bounds if outside.
    ///
    /// Clamps coordinates to `[0, extent]` before computing normal.
    /// Falls back to `+Z` (up vector) if normal computation fails.
    #[must_use]
    pub fn normal_clamped(&self, x: f64, y: f64) -> Vector3<f64> {
        let x = x.clamp(0.0, self.extent_x());
        let y = y.clamp(0.0, self.extent_y());
        self.normal(x, y).unwrap_or_else(Vector3::z)
    }

    /// Get the cell indices containing a world-space point.
    ///
    /// Returns `None` if the point is outside the height field bounds.
    #[must_use]
    pub fn cell_at(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        // Guard against NaN and negative inputs
        if x.is_nan() || y.is_nan() || x < 0.0 || y < 0.0 {
            return None;
        }

        let cx = (x / self.cell_size).floor() as usize;
        let cy = (y / self.cell_size).floor() as usize;

        if cx >= self.width - 1 || cy >= self.depth - 1 {
            return None;
        }

        Some((cx, cy))
    }

    /// Get all cells that intersect with an AABB.
    ///
    /// Returns iterator over `(cell_x, cell_y)` pairs.
    pub fn cells_in_aabb(
        &self,
        min: Point3<f64>,
        max: Point3<f64>,
    ) -> impl Iterator<Item = (usize, usize)> + '_ {
        // Guard against NaN — if any coord is NaN, return empty iterator
        let has_nan =
            !min.x.is_finite() || !min.y.is_finite() || !max.x.is_finite() || !max.y.is_finite();

        let x_start = if has_nan {
            0
        } else {
            (min.x / self.cell_size).floor().max(0.0) as usize
        };
        let y_start = if has_nan {
            0
        } else {
            (min.y / self.cell_size).floor().max(0.0) as usize
        };
        let x_end = if has_nan {
            0
        } else {
            ((max.x / self.cell_size).ceil() as usize).min(self.width - 1)
        };
        let y_end = if has_nan {
            0
        } else {
            ((max.y / self.cell_size).ceil() as usize).min(self.depth - 1)
        };

        (y_start..y_end).flat_map(move |y| (x_start..x_end).map(move |x| (x, y)))
    }

    /// Get the four corner heights of a cell.
    ///
    /// Returns `None` if cell indices are out of bounds.
    #[must_use]
    pub fn cell_heights(&self, cx: usize, cy: usize) -> Option<[f64; 4]> {
        if cx >= self.width - 1 || cy >= self.depth - 1 {
            return None;
        }

        Some([
            self.heights[cy * self.width + cx],           // (cx, cy)
            self.heights[cy * self.width + cx + 1],       // (cx+1, cy)
            self.heights[(cy + 1) * self.width + cx],     // (cx, cy+1)
            self.heights[(cy + 1) * self.width + cx + 1], // (cx+1, cy+1)
        ])
    }

    /// Get the world-space position of a grid vertex.
    #[must_use]
    pub fn vertex_position(&self, x: usize, y: usize) -> Option<Point3<f64>> {
        let height = self.get(x, y)?;
        Some(Point3::new(
            x as f64 * self.cell_size,
            y as f64 * self.cell_size,
            height,
        ))
    }
}

impl Bounded for HeightFieldData {
    fn aabb(&self) -> Aabb {
        Aabb::new(
            Point3::new(0.0, 0.0, self.min_height),
            Point3::new(self.extent_x(), self.extent_y(), self.max_height),
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
    clippy::needless_collect
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    // ── Construction ────────────────────────────────────────────────────

    #[test]
    fn flat_field() {
        let hf = HeightFieldData::flat(10, 10, 1.0, 5.0);
        assert_eq!(hf.width(), 10);
        assert_eq!(hf.depth(), 10);
        assert_eq!(hf.cell_size(), 1.0);
        assert_relative_eq!(hf.min_height(), 5.0);
        assert_relative_eq!(hf.max_height(), 5.0);
    }

    #[test]
    fn from_fn_slope() {
        let hf = HeightFieldData::from_fn(5, 5, 1.0, |x, _y| x);
        assert_relative_eq!(hf.get(0, 0).unwrap(), 0.0);
        assert_relative_eq!(hf.get(1, 0).unwrap(), 1.0);
        assert_relative_eq!(hf.get(4, 0).unwrap(), 4.0);
    }

    #[test]
    #[should_panic(expected = "dimensions must be positive")]
    fn zero_width_panics() {
        let _ = HeightFieldData::new(vec![], 0, 1, 1.0);
    }

    #[test]
    #[should_panic(expected = "doesn't match dimensions")]
    fn wrong_length_panics() {
        let _ = HeightFieldData::new(vec![1.0, 2.0], 3, 3, 1.0);
    }

    #[test]
    #[should_panic(expected = "cell_size must be positive")]
    fn negative_cell_size_panics() {
        let _ = HeightFieldData::new(vec![0.0; 4], 2, 2, -1.0);
    }

    // ── Extents ─────────────────────────────────────────────────────────

    #[test]
    fn extents() {
        let hf = HeightFieldData::flat(10, 8, 2.0, 0.0);
        assert_relative_eq!(hf.extent_x(), 18.0); // (10-1) * 2.0
        assert_relative_eq!(hf.extent_y(), 14.0); // (8-1) * 2.0
    }

    // ── Grid access ─────────────────────────────────────────────────────

    #[test]
    fn get_out_of_bounds() {
        let hf = HeightFieldData::flat(3, 3, 1.0, 0.0);
        assert!(hf.get(3, 0).is_none());
        assert!(hf.get(0, 3).is_none());
        assert!(hf.get(2, 2).is_some());
    }

    #[test]
    fn get_clamped_clamps() {
        let hf = HeightFieldData::from_fn(3, 3, 1.0, |x, y| x + y);
        // Negative indices clamp to 0
        assert_relative_eq!(hf.get_clamped(-1, -1), hf.get(0, 0).unwrap());
        // Over-max clamps to (2, 2)
        assert_relative_eq!(hf.get_clamped(10, 10), hf.get(2, 2).unwrap());
    }

    // ── Bilinear interpolation ──────────────────────────────────────────

    #[test]
    fn bilinear_at_corners() {
        let hf = HeightFieldData::new(vec![0.0, 2.0, 1.0, 3.0], 2, 2, 1.0);
        assert_relative_eq!(hf.sample(0.0, 0.0).unwrap(), 0.0);
        assert_relative_eq!(hf.sample(1.0, 0.0).unwrap(), 2.0);
        assert_relative_eq!(hf.sample(0.0, 1.0).unwrap(), 1.0);
        assert_relative_eq!(hf.sample(1.0, 1.0).unwrap(), 3.0);
    }

    #[test]
    fn bilinear_at_center() {
        let hf = HeightFieldData::new(vec![0.0, 2.0, 1.0, 3.0], 2, 2, 1.0);
        // Center = average of all four = (0 + 2 + 1 + 3) / 4 = 1.5
        assert_relative_eq!(hf.sample(0.5, 0.5).unwrap(), 1.5);
    }

    #[test]
    fn bilinear_at_edge_midpoints() {
        let hf = HeightFieldData::new(vec![0.0, 2.0, 1.0, 3.0], 2, 2, 1.0);
        assert_relative_eq!(hf.sample(0.5, 0.0).unwrap(), 1.0); // avg of 0 and 2
        assert_relative_eq!(hf.sample(0.5, 1.0).unwrap(), 2.0); // avg of 1 and 3
    }

    #[test]
    fn sample_out_of_bounds() {
        let hf = HeightFieldData::flat(3, 3, 1.0, 5.0);
        assert!(hf.sample(-0.1, 0.0).is_none());
        assert!(hf.sample(0.0, -0.1).is_none());
        assert!(hf.sample(3.0, 0.0).is_none()); // extent_x = 2.0
        assert!(hf.sample(f64::NAN, 0.0).is_none());
    }

    #[test]
    fn sample_clamped_always_returns() {
        let hf = HeightFieldData::flat(3, 3, 1.0, 5.0);
        assert_relative_eq!(hf.sample_clamped(-10.0, -10.0), 5.0);
        assert_relative_eq!(hf.sample_clamped(100.0, 100.0), 5.0);
    }

    // ── Normal computation ──────────────────────────────────────────────

    #[test]
    fn flat_normal_is_z_up() {
        let hf = HeightFieldData::flat(5, 5, 1.0, 0.0);
        let n = hf.normal(2.0, 2.0).unwrap();
        assert_relative_eq!(n.z, 1.0, epsilon = 1e-3);
        assert_relative_eq!(n.x.abs(), 0.0, epsilon = 1e-3);
        assert_relative_eq!(n.y.abs(), 0.0, epsilon = 1e-3);
    }

    #[test]
    fn slope_normal_tilts() {
        // z = 0.5 * x → normal tilted in -X, +Z direction
        let hf = HeightFieldData::from_fn(10, 10, 1.0, |x, _y| x * 0.5);
        let n = hf.normal(5.0, 5.0).unwrap();
        assert!(n.x < 0.0, "normal.x should be negative for +X slope");
        assert!(n.z > 0.0, "normal.z should be positive");
    }

    #[test]
    fn normal_out_of_bounds() {
        let hf = HeightFieldData::flat(3, 3, 1.0, 0.0);
        assert!(hf.normal(-1.0, 0.0).is_none());
    }

    #[test]
    fn normal_clamped_always_returns() {
        let hf = HeightFieldData::flat(3, 3, 1.0, 0.0);
        let n = hf.normal_clamped(-10.0, -10.0);
        assert!(n.norm() > 0.9); // should be a valid unit vector
    }

    // ── Cell queries ────────────────────────────────────────────────────

    #[test]
    fn cell_at_valid() {
        let hf = HeightFieldData::flat(5, 5, 1.0, 0.0);
        assert_eq!(hf.cell_at(0.5, 0.5), Some((0, 0)));
        assert_eq!(hf.cell_at(1.5, 2.5), Some((1, 2)));
    }

    #[test]
    fn cell_at_boundary() {
        let hf = HeightFieldData::flat(5, 5, 1.0, 0.0);
        // At the max boundary, cell_at returns None (past last cell)
        assert!(hf.cell_at(4.0, 0.0).is_none());
        assert!(hf.cell_at(0.0, 4.0).is_none());
    }

    #[test]
    fn cell_at_nan() {
        let hf = HeightFieldData::flat(5, 5, 1.0, 0.0);
        assert!(hf.cell_at(f64::NAN, 0.0).is_none());
    }

    #[test]
    fn cells_in_aabb_coverage() {
        let hf = HeightFieldData::flat(10, 10, 1.0, 0.0);
        let cells: Vec<_> = hf
            .cells_in_aabb(Point3::new(2.5, 2.5, -1.0), Point3::new(5.5, 5.5, 1.0))
            .collect();
        assert!(!cells.is_empty());
        assert!(cells.contains(&(2, 2)));
        assert!(cells.contains(&(3, 3)));
        assert!(cells.contains(&(4, 4)));
        assert!(cells.contains(&(5, 5)));
    }

    #[test]
    fn cells_in_aabb_nan() {
        let hf = HeightFieldData::flat(10, 10, 1.0, 0.0);
        let cells: Vec<_> = hf
            .cells_in_aabb(Point3::new(f64::NAN, 0.0, 0.0), Point3::new(5.0, 5.0, 1.0))
            .collect();
        assert!(cells.is_empty());
    }

    #[test]
    fn cell_heights_valid() {
        let hf = HeightFieldData::new(vec![0.0, 1.0, 2.0, 3.0], 2, 2, 1.0);
        let h = hf.cell_heights(0, 0).unwrap();
        assert_relative_eq!(h[0], 0.0);
        assert_relative_eq!(h[1], 1.0);
        assert_relative_eq!(h[2], 2.0);
        assert_relative_eq!(h[3], 3.0);
    }

    #[test]
    fn cell_heights_out_of_bounds() {
        let hf = HeightFieldData::flat(3, 3, 1.0, 0.0);
        assert!(hf.cell_heights(2, 0).is_none()); // width-1 = 2, so cx=2 is invalid
        assert!(hf.cell_heights(0, 2).is_none());
    }

    // ── Vertex position ─────────────────────────────────────────────────

    #[test]
    fn vertex_position_valid() {
        let hf = HeightFieldData::from_fn(3, 3, 2.0, |x, y| x + y);
        let p = hf.vertex_position(1, 1).unwrap();
        assert_relative_eq!(p.x, 2.0); // 1 * 2.0
        assert_relative_eq!(p.y, 2.0); // 1 * 2.0
        assert_relative_eq!(p.z, 4.0); // 2.0 + 2.0
    }

    #[test]
    fn vertex_position_out_of_bounds() {
        let hf = HeightFieldData::flat(3, 3, 1.0, 0.0);
        assert!(hf.vertex_position(3, 0).is_none());
    }

    // ── Bounded impl ────────────────────────────────────────────────────

    #[test]
    fn bounded_aabb() {
        let hf = HeightFieldData::from_fn(10, 10, 2.0, |x, y| x + y);
        let aabb = hf.aabb();
        assert_relative_eq!(aabb.min.x, 0.0);
        assert_relative_eq!(aabb.min.y, 0.0);
        assert_relative_eq!(aabb.max.x, 18.0); // (10-1) * 2.0
        assert_relative_eq!(aabb.max.y, 18.0);
        assert_relative_eq!(aabb.min.z, hf.min_height());
        assert_relative_eq!(aabb.max.z, hf.max_height());
    }

    // ── Bilinear interpolation accuracy ─────────────────────────────────

    #[test]
    fn bilinear_on_flat_field() {
        let hf = HeightFieldData::flat(10, 10, 1.0, 5.0);
        // All points on a flat field should return exactly the height
        assert_relative_eq!(hf.sample(0.0, 0.0).unwrap(), 5.0);
        assert_relative_eq!(hf.sample(4.5, 4.5).unwrap(), 5.0);
        assert_relative_eq!(hf.sample(9.0, 9.0).unwrap(), 5.0);
    }

    #[test]
    fn bilinear_linear_in_x() {
        // z = x → bilinear should reproduce exactly along x
        let hf = HeightFieldData::from_fn(10, 10, 1.0, |x, _| x);
        assert_relative_eq!(hf.sample(3.7, 2.0).unwrap(), 3.7, epsilon = 1e-10);
        assert_relative_eq!(hf.sample(0.0, 0.0).unwrap(), 0.0, epsilon = 1e-10);
        assert_relative_eq!(hf.sample(9.0, 4.0).unwrap(), 9.0, epsilon = 1e-10);
    }

    #[test]
    fn bilinear_linear_in_y() {
        // z = y → bilinear should reproduce exactly along y
        let hf = HeightFieldData::from_fn(10, 10, 1.0, |_, y| y);
        assert_relative_eq!(hf.sample(2.0, 5.3).unwrap(), 5.3, epsilon = 1e-10);
    }

    #[test]
    fn bilinear_with_non_unit_cell_size() {
        // 3x3 grid with cell_size = 0.5 → extent = 1.0
        let hf = HeightFieldData::from_fn(3, 3, 0.5, |x, y| x + y);
        assert_relative_eq!(hf.sample(0.25, 0.25).unwrap(), 0.5, epsilon = 1e-10);
        assert_relative_eq!(hf.sample(1.0, 1.0).unwrap(), 2.0, epsilon = 1e-10);
    }
}
