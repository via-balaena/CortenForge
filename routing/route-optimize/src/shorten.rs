//! Path shortening algorithms.
//!
//! This module provides algorithms to reduce path length by removing
//! redundant waypoints and finding shortcuts.
//!
//! # Example
//!
//! ```
//! use route_optimize::shorten::PathShortener;
//! use route_types::VoxelPath;
//! use cf_spatial::{VoxelCoord, VoxelGrid};
//!
//! let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
//! let shortener = PathShortener::new(&grid);
//!
//! // Create a path with redundant waypoints
//! let path = VoxelPath::new(vec![
//!     VoxelCoord::new(0, 0, 0),
//!     VoxelCoord::new(1, 0, 0),
//!     VoxelCoord::new(2, 0, 0),
//!     VoxelCoord::new(3, 0, 0),
//! ]);
//!
//! let shortened = shortener.shorten(&path);
//! assert!(shortened.len() <= path.len());
//! ```

use cf_spatial::{VoxelCoord, VoxelGrid, line_of_sight};
use route_types::VoxelPath;

/// A path shortener that removes redundant waypoints.
///
/// Uses visibility checks to find direct paths between non-adjacent
/// waypoints, effectively "cutting corners" where possible.
pub struct PathShortener<'a, T> {
    /// The obstacle grid.
    grid: &'a VoxelGrid<T>,
    /// Maximum iterations for iterative shortening.
    max_iterations: usize,
}

impl<'a, T: Default> PathShortener<'a, T> {
    /// Creates a new path shortener.
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::shorten::PathShortener;
    /// use cf_spatial::VoxelGrid;
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let shortener = PathShortener::new(&grid);
    /// ```
    #[must_use]
    pub const fn new(grid: &'a VoxelGrid<T>) -> Self {
        Self {
            grid,
            max_iterations: 10,
        }
    }

    /// Sets the maximum number of shortening iterations.
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::shorten::PathShortener;
    /// use cf_spatial::VoxelGrid;
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let shortener = PathShortener::new(&grid).with_max_iterations(20);
    /// ```
    #[must_use]
    pub const fn with_max_iterations(mut self, iterations: usize) -> Self {
        self.max_iterations = iterations;
        self
    }

    /// Checks if there is clear line of sight between two coordinates.
    fn has_line_of_sight(&self, from: VoxelCoord, to: VoxelCoord) -> bool {
        let from_world = self.grid.grid_to_world_center(from);
        let to_world = self.grid.grid_to_world_center(to);
        line_of_sight(&from_world, &to_world, self.grid, |_| true)
    }

    /// Shortens a path by removing redundant waypoints.
    ///
    /// Uses a greedy forward algorithm: starting from the first waypoint,
    /// find the furthest waypoint with clear line of sight and skip
    /// all waypoints in between.
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::shorten::PathShortener;
    /// use route_types::VoxelPath;
    /// use cf_spatial::{VoxelCoord, VoxelGrid};
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let shortener = PathShortener::new(&grid);
    ///
    /// let path = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(1, 0, 0),
    ///     VoxelCoord::new(2, 0, 0),
    ///     VoxelCoord::new(3, 0, 0),
    /// ]);
    ///
    /// let shortened = shortener.shorten(&path);
    /// assert_eq!(shortened.len(), 2);  // Just start and end
    /// ```
    #[must_use]
    pub fn shorten(&self, path: &VoxelPath) -> VoxelPath {
        let coords = path.coords();

        if coords.len() <= 2 {
            return path.clone();
        }

        let mut result = Vec::new();
        let mut current_idx = 0;

        while current_idx < coords.len() {
            result.push(coords[current_idx]);

            if current_idx == coords.len() - 1 {
                break;
            }

            // Find furthest visible point
            let mut furthest = current_idx + 1;
            for i in (current_idx + 2)..coords.len() {
                if self.has_line_of_sight(coords[current_idx], coords[i]) {
                    furthest = i;
                }
            }

            current_idx = furthest;
        }

        VoxelPath::new(result)
    }

    /// Iteratively shortens a path until no more improvements are found.
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::shorten::PathShortener;
    /// use route_types::VoxelPath;
    /// use cf_spatial::{VoxelCoord, VoxelGrid};
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let shortener = PathShortener::new(&grid);
    ///
    /// let path = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(1, 1, 0),
    ///     VoxelCoord::new(2, 2, 0),
    ///     VoxelCoord::new(3, 3, 0),
    /// ]);
    ///
    /// let shortened = shortener.shorten_iterative(&path);
    /// ```
    #[must_use]
    pub fn shorten_iterative(&self, path: &VoxelPath) -> VoxelPath {
        let mut current = path.clone();

        for _ in 0..self.max_iterations {
            let shortened = self.shorten(&current);

            if shortened.len() >= current.len() {
                break;
            }

            current = shortened;
        }

        current
    }

    /// Shortens a path using random sampling.
    ///
    /// Randomly selects pairs of waypoints and removes intermediate
    /// points if there is clear line of sight.
    ///
    /// # Arguments
    ///
    /// * `path` - The path to shorten
    /// * `samples` - Number of random pairs to try
    /// * `rng` - Random number generator
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::shorten::PathShortener;
    /// use route_types::VoxelPath;
    /// use cf_spatial::{VoxelCoord, VoxelGrid};
    /// use rand::SeedableRng;
    /// use rand::rngs::StdRng;
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let shortener = PathShortener::new(&grid);
    ///
    /// let path = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(1, 0, 0),
    ///     VoxelCoord::new(2, 0, 0),
    ///     VoxelCoord::new(3, 0, 0),
    /// ]);
    ///
    /// let mut rng = StdRng::seed_from_u64(42);
    /// let shortened = shortener.shorten_random(&path, 10, &mut rng);
    /// ```
    #[must_use]
    pub fn shorten_random<R: rand::Rng>(
        &self,
        path: &VoxelPath,
        samples: usize,
        rng: &mut R,
    ) -> VoxelPath {
        let mut coords: Vec<VoxelCoord> = path.coords().to_vec();

        if coords.len() <= 2 {
            return path.clone();
        }

        for _ in 0..samples {
            if coords.len() <= 2 {
                break;
            }

            // Pick two random indices with at least one point between them
            let i = rng.gen_range(0..coords.len().saturating_sub(2));
            let j = rng.gen_range(i + 2..coords.len());

            // Check line of sight
            if self.has_line_of_sight(coords[i], coords[j]) {
                // Remove intermediate points
                coords.drain(i + 1..j);
            }
        }

        VoxelPath::new(coords)
    }
}

/// Computes the reduction ratio achieved by shortening.
///
/// Returns a value between 0.0 (no reduction) and 1.0 (maximum reduction).
///
/// # Example
///
/// ```
/// use route_optimize::shorten::reduction_ratio;
/// use route_types::VoxelPath;
/// use cf_spatial::VoxelCoord;
///
/// let original = VoxelPath::new(vec![
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(1, 0, 0),
///     VoxelCoord::new(2, 0, 0),
///     VoxelCoord::new(3, 0, 0),
/// ]);
///
/// let shortened = VoxelPath::new(vec![
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(3, 0, 0),
/// ]);
///
/// let ratio = reduction_ratio(&original, &shortened);
/// assert!((ratio - 0.5).abs() < 0.01);  // 50% reduction
/// ```
#[must_use]
pub fn reduction_ratio(original: &VoxelPath, shortened: &VoxelPath) -> f64 {
    if original.is_empty() || original.len() <= shortened.len() {
        return 0.0;
    }

    #[allow(clippy::cast_precision_loss)]
    let ratio = (original.len() - shortened.len()) as f64 / original.len() as f64;
    ratio
}

/// Computes the length reduction achieved by shortening.
///
/// Compares the actual path lengths (sum of segment distances).
///
/// # Example
///
/// ```
/// use route_optimize::shorten::length_reduction;
/// use route_types::VoxelPath;
/// use cf_spatial::VoxelCoord;
///
/// let original = VoxelPath::new(vec![
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(1, 0, 0),
///     VoxelCoord::new(2, 0, 0),
/// ]);
///
/// let shortened = VoxelPath::new(vec![
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(2, 0, 0),
/// ]);
///
/// let reduction = length_reduction(&original, &shortened);
/// assert!(reduction >= 0.0);  // Should be non-negative (paths are same length here)
/// ```
#[must_use]
pub fn length_reduction(original: &VoxelPath, shortened: &VoxelPath) -> f64 {
    let original_len = original.length();
    let shortened_len = shortened.length();

    if original_len <= shortened_len {
        0.0
    } else {
        original_len - shortened_len
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rand::SeedableRng;
    use rand::rngs::StdRng;

    fn empty_grid() -> VoxelGrid<bool> {
        VoxelGrid::new(1.0)
    }

    fn grid_with_wall() -> VoxelGrid<bool> {
        let mut grid = VoxelGrid::new(1.0);
        for y in -5..=5 {
            grid.set(VoxelCoord::new(5, y, 0), true);
        }
        grid
    }

    #[test]
    fn test_shorten_straight_line() {
        let grid = empty_grid();
        let shortener = PathShortener::new(&grid);

        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
            VoxelCoord::new(3, 0, 0),
        ]);

        let shortened = shortener.shorten(&path);

        // Straight line should reduce to just endpoints
        assert_eq!(shortened.len(), 2);
        assert_eq!(shortened.coords()[0], VoxelCoord::new(0, 0, 0));
        assert_eq!(shortened.coords()[1], VoxelCoord::new(3, 0, 0));
    }

    #[test]
    fn test_shorten_blocked() {
        let grid = grid_with_wall();
        let shortener = PathShortener::new(&grid);

        // Path that goes around wall
        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(0, 6, 0),
            VoxelCoord::new(5, 6, 0), // Through gap
            VoxelCoord::new(10, 6, 0),
            VoxelCoord::new(10, 0, 0),
        ]);

        let shortened = shortener.shorten(&path);

        // Wall blocks direct line of sight, so some waypoints should remain
        assert!(shortened.len() >= 2);
    }

    #[test]
    fn test_shorten_short_path() {
        let grid = empty_grid();
        let shortener = PathShortener::new(&grid);

        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(5, 0, 0)]);

        let shortened = shortener.shorten(&path);

        assert_eq!(shortened.len(), 2);
    }

    #[test]
    fn test_shorten_single_point() {
        let grid = empty_grid();
        let shortener = PathShortener::new(&grid);

        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0)]);

        let shortened = shortener.shorten(&path);

        assert_eq!(shortened.len(), 1);
    }

    #[test]
    fn test_shorten_empty() {
        let grid = empty_grid();
        let shortener = PathShortener::new(&grid);

        let path = VoxelPath::new(vec![]);
        let shortened = shortener.shorten(&path);

        assert!(shortened.is_empty());
    }

    #[test]
    fn test_shorten_iterative() {
        let grid = empty_grid();
        let shortener = PathShortener::new(&grid).with_max_iterations(5);

        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
            VoxelCoord::new(3, 0, 0),
            VoxelCoord::new(4, 0, 0),
        ]);

        let shortened = shortener.shorten_iterative(&path);

        assert_eq!(shortened.len(), 2);
    }

    #[test]
    fn test_shorten_random() {
        let grid = empty_grid();
        let shortener = PathShortener::new(&grid);

        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
            VoxelCoord::new(3, 0, 0),
        ]);

        let mut rng = StdRng::seed_from_u64(42);
        let shortened = shortener.shorten_random(&path, 10, &mut rng);

        assert!(shortened.len() <= path.len());
    }

    #[test]
    fn test_reduction_ratio() {
        let original = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
            VoxelCoord::new(3, 0, 0),
        ]);

        let shortened = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(3, 0, 0)]);

        let ratio = reduction_ratio(&original, &shortened);
        assert_relative_eq!(ratio, 0.5, epsilon = 0.01);
    }

    #[test]
    fn test_reduction_ratio_no_change() {
        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(3, 0, 0)]);

        let ratio = reduction_ratio(&path, &path);
        assert_relative_eq!(ratio, 0.0);
    }

    #[test]
    fn test_reduction_ratio_empty() {
        let empty = VoxelPath::new(vec![]);
        let non_empty = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0)]);

        let ratio = reduction_ratio(&empty, &non_empty);
        assert_relative_eq!(ratio, 0.0);
    }

    #[test]
    fn test_length_reduction() {
        let original = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(0, 1, 0),
            VoxelCoord::new(1, 1, 0),
        ]);

        let shortened = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 1, 0)]);

        let reduction = length_reduction(&original, &shortened);
        // Original: 2.0, Shortened: sqrt(2) ≈ 1.414
        assert!(reduction > 0.0);
    }

    #[test]
    fn test_length_reduction_same() {
        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 0, 0)]);

        let reduction = length_reduction(&path, &path);
        assert_relative_eq!(reduction, 0.0);
    }

    #[test]
    fn test_with_max_iterations() {
        let grid = empty_grid();
        let shortener = PathShortener::new(&grid).with_max_iterations(100);

        // Just verify it builds correctly
        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(5, 0, 0)]);

        let shortened = shortener.shorten_iterative(&path);
        assert!(!shortened.is_empty());
    }
}
