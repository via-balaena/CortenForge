//! Curvature analysis and smoothing for paths.
//!
//! This module provides algorithms to analyze and improve path curvature,
//! ensuring paths respect minimum bend radius constraints.
//!
//! # Overview
//!
//! Curvature is the rate of change of direction along a path. For physical
//! routing (wires, pipes, tendons), there's often a minimum bend radius
//! constraint that must be satisfied.
//!
//! # Example
//!
//! ```
//! use route_optimize::curvature::{CurvatureSmoother, curvature_at};
//! use route_types::VoxelPath;
//! use cf_spatial::{VoxelCoord, VoxelGrid};
//!
//! // Analyze curvature at a point
//! let prev = VoxelCoord::new(0, 0, 0);
//! let curr = VoxelCoord::new(1, 0, 0);
//! let next = VoxelCoord::new(1, 1, 0);  // 90-degree turn
//!
//! let curvature = curvature_at(prev, curr, next);
//! assert!(curvature > 0.0);  // Non-zero curvature at the turn
//! ```

use cf_spatial::VoxelCoord;
use route_types::VoxelPath;

/// Computes the curvature at a point given three consecutive waypoints.
///
/// Curvature is estimated using the Menger curvature formula:
/// κ = 4 * Area(triangle) / (a * b * c)
///
/// where a, b, c are the side lengths of the triangle formed by the points.
///
/// # Arguments
///
/// * `prev` - The previous waypoint
/// * `curr` - The current waypoint
/// * `next` - The next waypoint
///
/// # Returns
///
/// The curvature at the current point. Returns 0.0 for collinear points.
///
/// # Example
///
/// ```
/// use route_optimize::curvature::curvature_at;
/// use cf_spatial::VoxelCoord;
///
/// // Straight line: zero curvature
/// let curvature = curvature_at(
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(1, 0, 0),
///     VoxelCoord::new(2, 0, 0),
/// );
/// assert!(curvature.abs() < 1e-10);
///
/// // Sharp turn: high curvature
/// let curvature = curvature_at(
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(1, 0, 0),
///     VoxelCoord::new(1, 1, 0),
/// );
/// assert!(curvature > 0.0);
/// ```
#[must_use]
pub fn curvature_at(prev: VoxelCoord, curr: VoxelCoord, next: VoxelCoord) -> f64 {
    // Convert to f64 vectors
    let p1 = [f64::from(prev.x), f64::from(prev.y), f64::from(prev.z)];
    let p2 = [f64::from(curr.x), f64::from(curr.y), f64::from(curr.z)];
    let p3 = [f64::from(next.x), f64::from(next.y), f64::from(next.z)];

    // Compute side lengths
    let a = distance(&p1, &p2);
    let b = distance(&p2, &p3);
    let c = distance(&p1, &p3);

    // Avoid division by zero
    let product = a * b * c;
    if product < f64::EPSILON {
        return 0.0;
    }

    // Compute area using Heron's formula
    let s = (a + b + c) / 2.0;
    let area_sq = s * (s - a) * (s - b) * (s - c);

    if area_sq < 0.0 {
        return 0.0;
    }

    let area = area_sq.sqrt();

    // Menger curvature: κ = 4 * Area / (a * b * c)
    4.0 * area / product
}

/// Computes the Euclidean distance between two 3D points.
fn distance(p1: &[f64; 3], p2: &[f64; 3]) -> f64 {
    let dx = p2[0] - p1[0];
    let dy = p2[1] - p1[1];
    let dz = p2[2] - p1[2];
    dx.mul_add(dx, dy.mul_add(dy, dz * dz)).sqrt()
}

/// Computes the bend angle at a waypoint in radians.
///
/// The angle is computed as the deviation from a straight line.
/// Returns 0 for straight segments and π for a complete reversal.
///
/// # Example
///
/// ```
/// use route_optimize::curvature::bend_angle;
/// use cf_spatial::VoxelCoord;
/// use std::f64::consts::PI;
///
/// // Straight line: zero angle
/// let angle = bend_angle(
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(1, 0, 0),
///     VoxelCoord::new(2, 0, 0),
/// );
/// assert!(angle.abs() < 1e-10);
///
/// // 90-degree turn
/// let angle = bend_angle(
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(1, 0, 0),
///     VoxelCoord::new(1, 1, 0),
/// );
/// assert!((angle - PI / 2.0).abs() < 0.1);
/// ```
#[must_use]
pub fn bend_angle(prev: VoxelCoord, curr: VoxelCoord, next: VoxelCoord) -> f64 {
    // Compute direction vectors
    let d1 = curr - prev;
    let d2 = next - curr;

    let v1 = [f64::from(d1.x), f64::from(d1.y), f64::from(d1.z)];
    let v2 = [f64::from(d2.x), f64::from(d2.y), f64::from(d2.z)];

    // Compute magnitudes
    let mag1 = v1[0]
        .mul_add(v1[0], v1[1].mul_add(v1[1], v1[2] * v1[2]))
        .sqrt();
    let mag2 = v2[0]
        .mul_add(v2[0], v2[1].mul_add(v2[1], v2[2] * v2[2]))
        .sqrt();

    if mag1 < f64::EPSILON || mag2 < f64::EPSILON {
        return 0.0;
    }

    // Compute dot product
    let dot = v1[0].mul_add(v2[0], v1[1].mul_add(v2[1], v1[2] * v2[2]));
    let cos_angle = (dot / (mag1 * mag2)).clamp(-1.0, 1.0);

    // Return the deviation angle (0 for straight, π for reversal)
    cos_angle.acos()
}

/// Counts the number of bends (direction changes) in a path.
///
/// A bend is counted when the direction changes by more than the threshold.
///
/// # Arguments
///
/// * `path` - The path to analyze
/// * `threshold` - Minimum angle (radians) to count as a bend
///
/// # Example
///
/// ```
/// use route_optimize::curvature::count_bends;
/// use route_types::VoxelPath;
/// use cf_spatial::VoxelCoord;
///
/// let path = VoxelPath::new(vec![
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(1, 0, 0),
///     VoxelCoord::new(1, 1, 0),  // First bend
///     VoxelCoord::new(2, 1, 0),
///     VoxelCoord::new(2, 2, 0),  // Second bend
/// ]);
///
/// let bends = count_bends(&path, 0.1);
/// assert!(bends >= 2);
/// ```
#[must_use]
pub fn count_bends(path: &VoxelPath, threshold: f64) -> usize {
    let coords = path.coords();

    if coords.len() < 3 {
        return 0;
    }

    let mut count = 0;
    for i in 1..coords.len() - 1 {
        let angle = bend_angle(coords[i - 1], coords[i], coords[i + 1]);
        if angle > threshold {
            count += 1;
        }
    }

    count
}

/// Computes the curvature profile along a path.
///
/// Returns the curvature at each interior waypoint.
///
/// # Example
///
/// ```
/// use route_optimize::curvature::curvature_profile;
/// use route_types::VoxelPath;
/// use cf_spatial::VoxelCoord;
///
/// let path = VoxelPath::new(vec![
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(1, 0, 0),
///     VoxelCoord::new(1, 1, 0),
///     VoxelCoord::new(2, 1, 0),
/// ]);
///
/// let profile = curvature_profile(&path);
/// assert_eq!(profile.len(), 2);  // Two interior points
/// ```
#[must_use]
pub fn curvature_profile(path: &VoxelPath) -> Vec<f64> {
    let coords = path.coords();

    if coords.len() < 3 {
        return Vec::new();
    }

    (1..coords.len() - 1)
        .map(|i| curvature_at(coords[i - 1], coords[i], coords[i + 1]))
        .collect()
}

/// Computes the maximum curvature along a path.
///
/// # Example
///
/// ```
/// use route_optimize::curvature::max_curvature;
/// use route_types::VoxelPath;
/// use cf_spatial::VoxelCoord;
///
/// let path = VoxelPath::new(vec![
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(1, 0, 0),
///     VoxelCoord::new(2, 0, 0),
/// ]);
///
/// let max_k = max_curvature(&path);
/// assert!(max_k < 1e-10);  // Straight line has zero curvature
/// ```
#[must_use]
pub fn max_curvature(path: &VoxelPath) -> f64 {
    curvature_profile(path).into_iter().fold(0.0, f64::max)
}

/// Checks if a path satisfies a minimum bend radius constraint.
///
/// The minimum bend radius is the reciprocal of maximum curvature.
///
/// # Arguments
///
/// * `path` - The path to check
/// * `min_radius` - Minimum allowed bend radius
///
/// # Returns
///
/// `true` if the path satisfies the constraint.
///
/// # Example
///
/// ```
/// use route_optimize::curvature::satisfies_min_radius;
/// use route_types::VoxelPath;
/// use cf_spatial::VoxelCoord;
///
/// // Straight line satisfies any radius constraint
/// let path = VoxelPath::new(vec![
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(1, 0, 0),
///     VoxelCoord::new(2, 0, 0),
/// ]);
///
/// assert!(satisfies_min_radius(&path, 0.5));
/// ```
#[must_use]
pub fn satisfies_min_radius(path: &VoxelPath, min_radius: f64) -> bool {
    let max_k = max_curvature(path);

    if max_k < f64::EPSILON {
        return true;
    }

    let actual_min_radius = 1.0 / max_k;
    actual_min_radius >= min_radius
}

/// A smoother that improves path curvature by inserting intermediate waypoints.
pub struct CurvatureSmoother {
    /// Target maximum curvature.
    max_curvature: f64,
    /// Maximum iterations for smoothing.
    max_iterations: usize,
}

impl Default for CurvatureSmoother {
    fn default() -> Self {
        Self::new()
    }
}

impl CurvatureSmoother {
    /// Creates a new curvature smoother with default settings.
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::curvature::CurvatureSmoother;
    ///
    /// let smoother = CurvatureSmoother::new();
    /// ```
    #[must_use]
    pub const fn new() -> Self {
        Self {
            max_curvature: 1.0,
            max_iterations: 10,
        }
    }

    /// Sets the maximum allowed curvature.
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::curvature::CurvatureSmoother;
    ///
    /// let smoother = CurvatureSmoother::new().with_max_curvature(0.5);
    /// ```
    #[must_use]
    pub const fn with_max_curvature(mut self, curvature: f64) -> Self {
        self.max_curvature = curvature;
        self
    }

    /// Sets the maximum number of smoothing iterations.
    #[must_use]
    pub const fn with_max_iterations(mut self, iterations: usize) -> Self {
        self.max_iterations = iterations;
        self
    }

    /// Smooths a path to reduce excessive curvature.
    ///
    /// Currently uses a simple averaging approach that may introduce
    /// small position changes.
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::curvature::CurvatureSmoother;
    /// use route_types::VoxelPath;
    /// use cf_spatial::VoxelCoord;
    ///
    /// let smoother = CurvatureSmoother::new();
    ///
    /// let path = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(1, 0, 0),
    ///     VoxelCoord::new(1, 1, 0),
    ///     VoxelCoord::new(2, 1, 0),
    /// ]);
    ///
    /// let smoothed = smoother.smooth(&path);
    /// ```
    #[must_use]
    pub fn smooth(&self, path: &VoxelPath) -> VoxelPath {
        let coords = path.coords();

        if coords.len() < 3 {
            return path.clone();
        }

        let mut result = coords.to_vec();

        for _ in 0..self.max_iterations {
            let mut changed = false;

            for i in 1..result.len() - 1 {
                let curvature = curvature_at(result[i - 1], result[i], result[i + 1]);

                if curvature > self.max_curvature {
                    // Simple averaging to reduce curvature
                    let prev = result[i - 1];
                    let next = result[i + 1];

                    // Move toward the midpoint
                    #[allow(clippy::cast_possible_truncation)]
                    let new_coord = VoxelCoord::new(
                        ((prev.x + next.x) / 2 + result[i].x) / 2,
                        ((prev.y + next.y) / 2 + result[i].y) / 2,
                        ((prev.z + next.z) / 2 + result[i].z) / 2,
                    );

                    if new_coord != result[i] {
                        result[i] = new_coord;
                        changed = true;
                    }
                }
            }

            if !changed {
                break;
            }
        }

        VoxelPath::new(result)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    #[test]
    fn test_curvature_straight_line() {
        let prev = VoxelCoord::new(0, 0, 0);
        let curr = VoxelCoord::new(1, 0, 0);
        let next = VoxelCoord::new(2, 0, 0);

        let curvature = curvature_at(prev, curr, next);
        assert!(curvature.abs() < 1e-10);
    }

    #[test]
    fn test_curvature_right_angle() {
        let prev = VoxelCoord::new(0, 0, 0);
        let curr = VoxelCoord::new(1, 0, 0);
        let next = VoxelCoord::new(1, 1, 0);

        let curvature = curvature_at(prev, curr, next);
        assert!(curvature > 0.0);
    }

    #[test]
    fn test_curvature_same_point() {
        let coord = VoxelCoord::new(1, 1, 1);

        let curvature = curvature_at(coord, coord, coord);
        assert_relative_eq!(curvature, 0.0);
    }

    #[test]
    fn test_bend_angle_straight() {
        let prev = VoxelCoord::new(0, 0, 0);
        let curr = VoxelCoord::new(1, 0, 0);
        let next = VoxelCoord::new(2, 0, 0);

        let angle = bend_angle(prev, curr, next);
        assert!(angle.abs() < 1e-10);
    }

    #[test]
    fn test_bend_angle_90_degrees() {
        let prev = VoxelCoord::new(0, 0, 0);
        let curr = VoxelCoord::new(1, 0, 0);
        let next = VoxelCoord::new(1, 1, 0);

        let angle = bend_angle(prev, curr, next);
        assert!((angle - PI / 2.0).abs() < 0.1);
    }

    #[test]
    fn test_bend_angle_180_degrees() {
        let prev = VoxelCoord::new(0, 0, 0);
        let curr = VoxelCoord::new(1, 0, 0);
        let next = VoxelCoord::new(0, 0, 0);

        let angle = bend_angle(prev, curr, next);
        assert!((angle - PI).abs() < 0.1);
    }

    #[test]
    fn test_count_bends() {
        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(1, 1, 0),
            VoxelCoord::new(2, 1, 0),
        ]);

        let bends = count_bends(&path, 0.1);
        assert!(bends >= 1);
    }

    #[test]
    fn test_count_bends_straight() {
        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
            VoxelCoord::new(3, 0, 0),
        ]);

        let bends = count_bends(&path, 0.1);
        assert_eq!(bends, 0);
    }

    #[test]
    fn test_count_bends_short_path() {
        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 0, 0)]);

        let bends = count_bends(&path, 0.1);
        assert_eq!(bends, 0);
    }

    #[test]
    fn test_curvature_profile() {
        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(1, 1, 0),
            VoxelCoord::new(2, 1, 0),
        ]);

        let profile = curvature_profile(&path);
        assert_eq!(profile.len(), 2);
    }

    #[test]
    fn test_curvature_profile_short() {
        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 0, 0)]);

        let profile = curvature_profile(&path);
        assert!(profile.is_empty());
    }

    #[test]
    fn test_max_curvature() {
        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
        ]);

        let max_k = max_curvature(&path);
        assert!(max_k < 1e-10);
    }

    #[test]
    fn test_satisfies_min_radius_straight() {
        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
        ]);

        assert!(satisfies_min_radius(&path, 0.5));
        assert!(satisfies_min_radius(&path, 10.0));
    }

    #[test]
    fn test_curvature_smoother_default() {
        let smoother = CurvatureSmoother::default();

        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
        ]);

        let smoothed = smoother.smooth(&path);
        assert!(!smoothed.is_empty());
    }

    #[test]
    fn test_curvature_smoother_with_settings() {
        let smoother = CurvatureSmoother::new()
            .with_max_curvature(0.5)
            .with_max_iterations(5);

        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(1, 1, 0),
            VoxelCoord::new(2, 1, 0),
        ]);

        let smoothed = smoother.smooth(&path);
        assert!(!smoothed.is_empty());
    }

    #[test]
    fn test_curvature_smoother_short_path() {
        let smoother = CurvatureSmoother::new();

        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 0, 0)]);

        let smoothed = smoother.smooth(&path);
        assert_eq!(smoothed.len(), 2);
    }
}
