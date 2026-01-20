//! Clearance optimization for route paths.
//!
//! This module provides algorithms to optimize the clearance of paths
//! from obstacles, pushing waypoints away from nearby obstacles while
//! maintaining path validity.
//!
//! # Example
//!
//! ```
//! use route_optimize::clearance::ClearanceOptimizer;
//! use route_types::{VoxelPath, Path, Route};
//! use cf_spatial::{VoxelCoord, VoxelGrid};
//!
//! // Create grid with some obstacles
//! let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
//! grid.set(VoxelCoord::new(1, 1, 0), true);
//!
//! // Create optimizer
//! let optimizer = ClearanceOptimizer::new(&grid);
//!
//! // Optimize a path
//! let path = VoxelPath::new(vec![
//!     VoxelCoord::new(0, 0, 0),
//!     VoxelCoord::new(2, 0, 0),
//! ]);
//!
//! let optimized = optimizer.optimize(&path, 2);
//! ```

use cf_spatial::{VoxelCoord, VoxelGrid};
use route_types::VoxelPath;

/// Optimizer that maximizes path clearance from obstacles.
///
/// The optimizer attempts to push path waypoints away from obstacles
/// while maintaining connectivity and validity.
pub struct ClearanceOptimizer<'a, T> {
    /// The obstacle grid.
    grid: &'a VoxelGrid<T>,
    /// Maximum search radius for obstacles.
    max_radius: i32,
}

impl<'a, T: Default> ClearanceOptimizer<'a, T> {
    /// Creates a new clearance optimizer.
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::clearance::ClearanceOptimizer;
    /// use cf_spatial::VoxelGrid;
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let optimizer = ClearanceOptimizer::new(&grid);
    /// ```
    #[must_use]
    pub const fn new(grid: &'a VoxelGrid<T>) -> Self {
        Self {
            grid,
            max_radius: 5,
        }
    }

    /// Sets the maximum search radius for obstacle detection.
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::clearance::ClearanceOptimizer;
    /// use cf_spatial::VoxelGrid;
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let optimizer = ClearanceOptimizer::new(&grid).with_max_radius(10);
    /// ```
    #[must_use]
    pub const fn with_max_radius(mut self, radius: i32) -> Self {
        self.max_radius = radius;
        self
    }

    /// Checks if a voxel coordinate is blocked.
    fn is_blocked(&self, coord: VoxelCoord) -> bool {
        self.grid.contains(coord)
    }

    /// Computes the clearance (distance to nearest obstacle) for a coordinate.
    ///
    /// Returns the distance to the nearest obstacle, up to `max_radius`.
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::clearance::ClearanceOptimizer;
    /// use cf_spatial::{VoxelCoord, VoxelGrid};
    ///
    /// let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// grid.set(VoxelCoord::new(3, 0, 0), true);
    ///
    /// let optimizer = ClearanceOptimizer::new(&grid);
    /// let clearance = optimizer.clearance_at(VoxelCoord::new(0, 0, 0));
    /// assert!(clearance <= 3.0);
    /// ```
    #[must_use]
    pub fn clearance_at(&self, coord: VoxelCoord) -> f64 {
        let mut min_dist = f64::from(self.max_radius);

        // Search in expanding shells
        for r in 1..=self.max_radius {
            let found = self.search_shell(coord, r, &mut min_dist);
            if found && f64::from(r) >= min_dist {
                break;
            }
        }

        min_dist
    }

    /// Searches a shell at radius `r` for obstacles.
    ///
    /// Returns true if any obstacle was found within `min_dist`.
    fn search_shell(&self, center: VoxelCoord, r: i32, min_dist: &mut f64) -> bool {
        let mut found = false;

        // Iterate over a cube shell at distance r
        for dx in -r..=r {
            for dy in -r..=r {
                for dz in -r..=r {
                    // Only check the shell surface
                    if dx.abs() != r && dy.abs() != r && dz.abs() != r {
                        continue;
                    }

                    let coord = VoxelCoord::new(
                        center.x.saturating_add(dx),
                        center.y.saturating_add(dy),
                        center.z.saturating_add(dz),
                    );

                    if self.is_blocked(coord) {
                        let dist = f64::from(dx * dx + dy * dy + dz * dz).sqrt();
                        if dist < *min_dist {
                            *min_dist = dist;
                            found = true;
                        }
                    }
                }
            }
        }

        found
    }

    /// Computes the average clearance along a path.
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::clearance::ClearanceOptimizer;
    /// use route_types::VoxelPath;
    /// use cf_spatial::{VoxelCoord, VoxelGrid};
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let optimizer = ClearanceOptimizer::new(&grid);
    ///
    /// let path = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(5, 0, 0),
    /// ]);
    ///
    /// let avg = optimizer.average_clearance(&path);
    /// assert!(avg > 0.0);
    /// ```
    #[must_use]
    pub fn average_clearance(&self, path: &VoxelPath) -> f64 {
        if path.is_empty() {
            return 0.0;
        }

        let total: f64 = path.coords().iter().map(|&c| self.clearance_at(c)).sum();
        #[allow(clippy::cast_precision_loss)]
        let avg = total / path.len() as f64;
        avg
    }

    /// Computes the minimum clearance along a path.
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::clearance::ClearanceOptimizer;
    /// use route_types::VoxelPath;
    /// use cf_spatial::{VoxelCoord, VoxelGrid};
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let optimizer = ClearanceOptimizer::new(&grid);
    ///
    /// let path = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(5, 0, 0),
    /// ]);
    ///
    /// let min = optimizer.minimum_clearance(&path);
    /// assert!(min > 0.0);
    /// ```
    #[must_use]
    pub fn minimum_clearance(&self, path: &VoxelPath) -> f64 {
        path.coords()
            .iter()
            .map(|&c| self.clearance_at(c))
            .fold(f64::INFINITY, f64::min)
    }

    /// Attempts to optimize a path for better clearance.
    ///
    /// Uses iterative local search to push waypoints away from obstacles.
    ///
    /// # Arguments
    ///
    /// * `path` - The path to optimize
    /// * `iterations` - Number of optimization iterations
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::clearance::ClearanceOptimizer;
    /// use route_types::VoxelPath;
    /// use cf_spatial::{VoxelCoord, VoxelGrid};
    ///
    /// let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// grid.set(VoxelCoord::new(1, 1, 0), true);
    ///
    /// let optimizer = ClearanceOptimizer::new(&grid);
    ///
    /// let path = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(2, 0, 0),
    /// ]);
    ///
    /// let optimized = optimizer.optimize(&path, 5);
    /// ```
    #[must_use]
    pub fn optimize(&self, path: &VoxelPath, iterations: usize) -> VoxelPath {
        if path.len() <= 2 {
            return path.clone();
        }

        let mut coords: Vec<VoxelCoord> = path.coords().to_vec();

        for _ in 0..iterations {
            // Try to improve each interior waypoint
            for i in 1..coords.len() - 1 {
                let prev = coords[i - 1];
                let curr = coords[i];
                let next = coords[i + 1];

                // Find the best neighbor that maintains connectivity
                if let Some(better) = self.find_better_position(curr, prev, next) {
                    coords[i] = better;
                }
            }
        }

        VoxelPath::new(coords)
    }

    /// Finds a better position for a waypoint that improves clearance.
    fn find_better_position(
        &self,
        current: VoxelCoord,
        prev: VoxelCoord,
        next: VoxelCoord,
    ) -> Option<VoxelCoord> {
        let current_clearance = self.clearance_at(current);
        let mut best_coord = current;
        let mut best_clearance = current_clearance;

        // Try all 26 neighbors
        for &neighbor in &current.all_neighbors() {
            // Skip if blocked
            if self.is_blocked(neighbor) {
                continue;
            }

            // Check connectivity: must be adjacent to both prev and next
            if !Self::is_adjacent(neighbor, prev) || !Self::is_adjacent(neighbor, next) {
                continue;
            }

            let clearance = self.clearance_at(neighbor);
            if clearance > best_clearance {
                best_clearance = clearance;
                best_coord = neighbor;
            }
        }

        if best_coord == current {
            None
        } else {
            Some(best_coord)
        }
    }

    /// Checks if two coordinates are adjacent (26-connectivity).
    fn is_adjacent(a: VoxelCoord, b: VoxelCoord) -> bool {
        let d = a - b;
        d.x.abs() <= 1 && d.y.abs() <= 1 && d.z.abs() <= 1 && (d.x != 0 || d.y != 0 || d.z != 0)
    }
}

/// Computes the clearance profile for a path.
///
/// Returns the clearance at each waypoint.
///
/// # Example
///
/// ```
/// use route_optimize::clearance::clearance_profile;
/// use route_types::VoxelPath;
/// use cf_spatial::{VoxelCoord, VoxelGrid};
///
/// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
///
/// let path = VoxelPath::new(vec![
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(1, 0, 0),
///     VoxelCoord::new(2, 0, 0),
/// ]);
///
/// let profile = clearance_profile(&path, &grid, 5);
/// assert_eq!(profile.len(), 3);
/// ```
#[must_use]
pub fn clearance_profile<T: Default>(
    path: &VoxelPath,
    grid: &VoxelGrid<T>,
    max_radius: i32,
) -> Vec<f64> {
    let optimizer = ClearanceOptimizer::new(grid).with_max_radius(max_radius);
    path.coords()
        .iter()
        .map(|&c| optimizer.clearance_at(c))
        .collect()
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn empty_grid() -> VoxelGrid<bool> {
        VoxelGrid::new(1.0)
    }

    fn grid_with_obstacle(coord: VoxelCoord) -> VoxelGrid<bool> {
        let mut grid = VoxelGrid::new(1.0);
        grid.set(coord, true);
        grid
    }

    #[test]
    fn test_clearance_empty_grid() {
        let grid = empty_grid();
        let optimizer = ClearanceOptimizer::new(&grid);

        let clearance = optimizer.clearance_at(VoxelCoord::new(0, 0, 0));
        assert_relative_eq!(clearance, 5.0, epsilon = 0.01); // max_radius
    }

    #[test]
    fn test_clearance_adjacent_obstacle() {
        let grid = grid_with_obstacle(VoxelCoord::new(1, 0, 0));
        let optimizer = ClearanceOptimizer::new(&grid);

        let clearance = optimizer.clearance_at(VoxelCoord::new(0, 0, 0));
        assert_relative_eq!(clearance, 1.0, epsilon = 0.01);
    }

    #[test]
    fn test_clearance_diagonal_obstacle() {
        let grid = grid_with_obstacle(VoxelCoord::new(1, 1, 0));
        let optimizer = ClearanceOptimizer::new(&grid);

        let clearance = optimizer.clearance_at(VoxelCoord::new(0, 0, 0));
        assert_relative_eq!(clearance, std::f64::consts::SQRT_2, epsilon = 0.01);
    }

    #[test]
    fn test_clearance_far_obstacle() {
        let grid = grid_with_obstacle(VoxelCoord::new(3, 0, 0));
        let optimizer = ClearanceOptimizer::new(&grid);

        let clearance = optimizer.clearance_at(VoxelCoord::new(0, 0, 0));
        assert_relative_eq!(clearance, 3.0, epsilon = 0.01);
    }

    #[test]
    fn test_average_clearance() {
        let grid = empty_grid();
        let optimizer = ClearanceOptimizer::new(&grid);

        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
        ]);

        let avg = optimizer.average_clearance(&path);
        assert!(avg > 0.0);
    }

    #[test]
    fn test_average_clearance_empty() {
        let grid = empty_grid();
        let optimizer = ClearanceOptimizer::new(&grid);

        let path = VoxelPath::new(vec![]);
        let avg = optimizer.average_clearance(&path);
        assert_relative_eq!(avg, 0.0);
    }

    #[test]
    fn test_minimum_clearance() {
        let mut grid = empty_grid();
        grid.set(VoxelCoord::new(2, 1, 0), true); // Obstacle near middle

        let optimizer = ClearanceOptimizer::new(&grid);

        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0), // Close to obstacle
            VoxelCoord::new(3, 0, 0),
        ]);

        let min = optimizer.minimum_clearance(&path);
        assert!(min < 2.0); // Should be close to the obstacle
    }

    #[test]
    fn test_optimize_short_path() {
        let grid = empty_grid();
        let optimizer = ClearanceOptimizer::new(&grid);

        // Path too short to optimize (only endpoints)
        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(5, 0, 0)]);

        let optimized = optimizer.optimize(&path, 5);
        assert_eq!(optimized.len(), 2);
    }

    #[test]
    fn test_optimize_with_obstacle() {
        let mut grid = empty_grid();
        grid.set(VoxelCoord::new(1, 1, 0), true);

        let optimizer = ClearanceOptimizer::new(&grid);

        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0), // Adjacent to obstacle
            VoxelCoord::new(2, 0, 0),
        ]);

        let optimized = optimizer.optimize(&path, 5);

        // Should still have 3 waypoints
        assert_eq!(optimized.len(), 3);
    }

    #[test]
    fn test_clearance_profile() {
        let grid = empty_grid();

        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
        ]);

        let profile = clearance_profile(&path, &grid, 5);

        assert_eq!(profile.len(), 3);
        for &c in &profile {
            assert!(c > 0.0);
        }
    }

    #[test]
    fn test_is_adjacent() {
        // Face adjacent
        assert!(ClearanceOptimizer::<bool>::is_adjacent(
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0)
        ));

        // Diagonal adjacent
        assert!(ClearanceOptimizer::<bool>::is_adjacent(
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 1, 1)
        ));

        // Not adjacent
        assert!(!ClearanceOptimizer::<bool>::is_adjacent(
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(2, 0, 0)
        ));

        // Same point
        assert!(!ClearanceOptimizer::<bool>::is_adjacent(
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(0, 0, 0)
        ));
    }

    #[test]
    fn test_with_max_radius() {
        let grid = empty_grid();
        let optimizer = ClearanceOptimizer::new(&grid).with_max_radius(10);

        let clearance = optimizer.clearance_at(VoxelCoord::new(0, 0, 0));
        assert_relative_eq!(clearance, 10.0, epsilon = 0.01);
    }
}
