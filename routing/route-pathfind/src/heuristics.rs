//! Heuristic functions for pathfinding algorithms.
//!
//! This module provides implementations of various distance heuristics
//! used by A* and related algorithms.
//!
//! # Example
//!
//! ```
//! use route_pathfind::heuristics::compute_heuristic;
//! use route_types::Heuristic;
//! use cf_spatial::VoxelCoord;
//!
//! let from = VoxelCoord::new(0, 0, 0);
//! let to = VoxelCoord::new(3, 4, 0);
//!
//! let manhattan = compute_heuristic(from, to, Heuristic::Manhattan);
//! assert!((manhattan - 7.0).abs() < 1e-10);  // 3 + 4
//!
//! let euclidean = compute_heuristic(from, to, Heuristic::Euclidean);
//! assert!((euclidean - 5.0).abs() < 1e-10);  // sqrt(9 + 16)
//! ```

use cf_spatial::VoxelCoord;
use route_types::Heuristic;

/// Computes the heuristic distance between two voxel coordinates.
///
/// The heuristic provides an estimate of the cost from one coordinate to another.
/// For A* to find optimal paths, the heuristic must be admissible (never overestimate).
///
/// # Arguments
///
/// * `from` - Starting coordinate
/// * `to` - Target coordinate
/// * `heuristic` - Which heuristic function to use
///
/// # Returns
///
/// The estimated distance/cost between the coordinates.
///
/// # Example
///
/// ```
/// use route_pathfind::heuristics::compute_heuristic;
/// use route_types::Heuristic;
/// use cf_spatial::VoxelCoord;
///
/// let a = VoxelCoord::new(0, 0, 0);
/// let b = VoxelCoord::new(10, 0, 0);
///
/// let dist = compute_heuristic(a, b, Heuristic::Euclidean);
/// assert!((dist - 10.0).abs() < 1e-10);
/// ```
#[must_use]
pub fn compute_heuristic(from: VoxelCoord, to: VoxelCoord, heuristic: Heuristic) -> f64 {
    match heuristic {
        Heuristic::Manhattan => manhattan_distance(from, to),
        Heuristic::Chebyshev => chebyshev_distance(from, to),
        Heuristic::Euclidean => euclidean_distance(from, to),
        Heuristic::Octile => octile_distance(from, to),
        Heuristic::Zero => 0.0,
    }
}

/// Manhattan distance (L1 norm).
///
/// Sum of absolute differences: |dx| + |dy| + |dz|
///
/// Admissible for 6-connectivity (face neighbors only).
#[must_use]
pub fn manhattan_distance(from: VoxelCoord, to: VoxelCoord) -> f64 {
    f64::from(from.manhattan_distance(to))
}

/// Chebyshev distance (L-infinity norm).
///
/// Maximum of absolute differences: max(|dx|, |dy|, |dz|)
///
/// Admissible for 26-connectivity (all neighbors).
#[must_use]
pub fn chebyshev_distance(from: VoxelCoord, to: VoxelCoord) -> f64 {
    f64::from(from.chebyshev_distance(to))
}

/// Euclidean distance (L2 norm).
///
/// Straight-line distance: sqrt(dx² + dy² + dz²)
///
/// Always admissible.
#[must_use]
pub fn euclidean_distance(from: VoxelCoord, to: VoxelCoord) -> f64 {
    let d = to - from;
    let dx = f64::from(d.x);
    let dy = f64::from(d.y);
    let dz = f64::from(d.z);
    dx.mul_add(dx, dy.mul_add(dy, dz * dz)).sqrt()
}

/// Octile distance (diagonal distance).
///
/// Optimal heuristic for 26-connectivity that properly accounts
/// for diagonal move costs.
///
/// Uses: sqrt(3)*min + (sqrt(2)-sqrt(3))*mid + (1-sqrt(2))*max + max
///
/// Simplified for 3D: the minimum of the three dimensions moves diagonally
/// in all three axes, then we handle the remainder.
#[must_use]
pub fn octile_distance(from: VoxelCoord, to: VoxelCoord) -> f64 {
    let d = to - from;
    let dx = f64::from(d.x.abs());
    let dy = f64::from(d.y.abs());
    let dz = f64::from(d.z.abs());

    // Sort the dimensions
    let mut dims = [dx, dy, dz];
    dims.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let [min, mid, max] = dims;

    // Cost of 3D diagonal: sqrt(3)
    // Cost of 2D diagonal: sqrt(2)
    // Cost of straight: 1
    let sqrt3 = 3.0_f64.sqrt();
    let sqrt2 = std::f64::consts::SQRT_2;

    // Move diagonally in 3D for 'min' steps
    // Then move diagonally in 2D for 'mid - min' steps
    // Then move straight for 'max - mid' steps
    sqrt3.mul_add(min, sqrt2.mul_add(mid - min, max - mid))
}

/// Computes the move cost between adjacent voxels.
///
/// Returns the Euclidean distance between the voxel centers,
/// which is 1.0 for face-adjacent, sqrt(2) for edge-adjacent,
/// and sqrt(3) for corner-adjacent voxels.
///
/// # Example
///
/// ```
/// use route_pathfind::heuristics::move_cost;
/// use cf_spatial::VoxelCoord;
///
/// let a = VoxelCoord::new(0, 0, 0);
/// let b = VoxelCoord::new(1, 0, 0);  // Face-adjacent
/// assert!((move_cost(a, b) - 1.0).abs() < 1e-10);
///
/// let c = VoxelCoord::new(1, 1, 0);  // Edge-adjacent
/// assert!((move_cost(a, c) - std::f64::consts::SQRT_2).abs() < 1e-10);
///
/// let d = VoxelCoord::new(1, 1, 1);  // Corner-adjacent
/// assert!((move_cost(a, d) - 3.0_f64.sqrt()).abs() < 1e-10);
/// ```
#[must_use]
pub fn move_cost(from: VoxelCoord, to: VoxelCoord) -> f64 {
    euclidean_distance(from, to)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_manhattan_distance() {
        let a = VoxelCoord::new(0, 0, 0);
        let b = VoxelCoord::new(3, 4, 5);
        assert_relative_eq!(manhattan_distance(a, b), 12.0, epsilon = 1e-10);
    }

    #[test]
    fn test_manhattan_distance_negative() {
        let a = VoxelCoord::new(5, 5, 5);
        let b = VoxelCoord::new(0, 0, 0);
        assert_relative_eq!(manhattan_distance(a, b), 15.0, epsilon = 1e-10);
    }

    #[test]
    fn test_chebyshev_distance() {
        let a = VoxelCoord::new(0, 0, 0);
        let b = VoxelCoord::new(3, 4, 5);
        assert_relative_eq!(chebyshev_distance(a, b), 5.0, epsilon = 1e-10);
    }

    #[test]
    fn test_euclidean_distance() {
        let a = VoxelCoord::new(0, 0, 0);
        let b = VoxelCoord::new(3, 4, 0);
        assert_relative_eq!(euclidean_distance(a, b), 5.0, epsilon = 1e-10);
    }

    #[test]
    fn test_euclidean_distance_3d() {
        let a = VoxelCoord::new(0, 0, 0);
        let b = VoxelCoord::new(1, 2, 2);
        // sqrt(1 + 4 + 4) = sqrt(9) = 3
        assert_relative_eq!(euclidean_distance(a, b), 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_octile_distance_straight() {
        let a = VoxelCoord::new(0, 0, 0);
        let b = VoxelCoord::new(5, 0, 0);
        // Pure straight line: 5 * 1.0 = 5.0
        assert_relative_eq!(octile_distance(a, b), 5.0, epsilon = 1e-10);
    }

    #[test]
    fn test_octile_distance_2d_diagonal() {
        let a = VoxelCoord::new(0, 0, 0);
        let b = VoxelCoord::new(3, 3, 0);
        // Pure 2D diagonal: 3 * sqrt(2)
        assert_relative_eq!(
            octile_distance(a, b),
            3.0 * std::f64::consts::SQRT_2,
            epsilon = 1e-10
        );
    }

    #[test]
    fn test_octile_distance_3d_diagonal() {
        let a = VoxelCoord::new(0, 0, 0);
        let b = VoxelCoord::new(2, 2, 2);
        // Pure 3D diagonal: 2 * sqrt(3)
        assert_relative_eq!(octile_distance(a, b), 2.0 * 3.0_f64.sqrt(), epsilon = 1e-10);
    }

    #[test]
    fn test_compute_heuristic_manhattan() {
        let a = VoxelCoord::new(0, 0, 0);
        let b = VoxelCoord::new(3, 4, 5);
        assert_relative_eq!(
            compute_heuristic(a, b, Heuristic::Manhattan),
            12.0,
            epsilon = 1e-10
        );
    }

    #[test]
    fn test_compute_heuristic_zero() {
        let a = VoxelCoord::new(0, 0, 0);
        let b = VoxelCoord::new(100, 100, 100);
        assert_relative_eq!(
            compute_heuristic(a, b, Heuristic::Zero),
            0.0,
            epsilon = 1e-10
        );
    }

    #[test]
    fn test_move_cost_face_adjacent() {
        let a = VoxelCoord::new(0, 0, 0);
        let b = VoxelCoord::new(1, 0, 0);
        assert_relative_eq!(move_cost(a, b), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_move_cost_edge_adjacent() {
        let a = VoxelCoord::new(0, 0, 0);
        let b = VoxelCoord::new(1, 1, 0);
        assert_relative_eq!(move_cost(a, b), std::f64::consts::SQRT_2, epsilon = 1e-10);
    }

    #[test]
    fn test_move_cost_corner_adjacent() {
        let a = VoxelCoord::new(0, 0, 0);
        let b = VoxelCoord::new(1, 1, 1);
        assert_relative_eq!(move_cost(a, b), 3.0_f64.sqrt(), epsilon = 1e-10);
    }

    #[test]
    fn test_heuristic_admissibility_euclidean() {
        // Euclidean should always be <= actual shortest path distance
        let a = VoxelCoord::new(0, 0, 0);
        let b = VoxelCoord::new(5, 5, 5);

        let heuristic = euclidean_distance(a, b);
        // Actual shortest path through diagonals: 5 * sqrt(3)
        let actual_diagonal = 5.0 * 3.0_f64.sqrt();

        assert!(heuristic <= actual_diagonal + 1e-10);
    }

    #[test]
    fn test_heuristic_admissibility_octile() {
        // Octile should equal the actual optimal path for diagonal movement
        let a = VoxelCoord::new(0, 0, 0);
        let b = VoxelCoord::new(3, 4, 5);

        let heuristic = octile_distance(a, b);
        // The octile distance should be admissible
        let euclidean = euclidean_distance(a, b);

        // Octile >= Euclidean (since Euclidean is lower bound)
        assert!(heuristic >= euclidean - 1e-10);
    }

    #[test]
    fn test_symmetry() {
        let a = VoxelCoord::new(1, 2, 3);
        let b = VoxelCoord::new(4, 5, 6);

        // All heuristics should be symmetric
        assert_relative_eq!(
            manhattan_distance(a, b),
            manhattan_distance(b, a),
            epsilon = 1e-10
        );
        assert_relative_eq!(
            chebyshev_distance(a, b),
            chebyshev_distance(b, a),
            epsilon = 1e-10
        );
        assert_relative_eq!(
            euclidean_distance(a, b),
            euclidean_distance(b, a),
            epsilon = 1e-10
        );
        assert_relative_eq!(
            octile_distance(a, b),
            octile_distance(b, a),
            epsilon = 1e-10
        );
    }

    #[test]
    fn test_zero_distance() {
        let a = VoxelCoord::new(5, 5, 5);

        assert_relative_eq!(manhattan_distance(a, a), 0.0, epsilon = 1e-10);
        assert_relative_eq!(chebyshev_distance(a, a), 0.0, epsilon = 1e-10);
        assert_relative_eq!(euclidean_distance(a, a), 0.0, epsilon = 1e-10);
        assert_relative_eq!(octile_distance(a, a), 0.0, epsilon = 1e-10);
    }
}
