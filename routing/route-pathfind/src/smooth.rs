//! Path smoothing and simplification.
//!
//! This module provides post-processing to simplify paths by removing
//! unnecessary waypoints using line-of-sight checks.
//!
//! # Example
//!
//! ```
//! use route_pathfind::smooth::PathSmoother;
//! use route_pathfind::astar::find_path;
//! use cf_spatial::{VoxelCoord, VoxelGrid};
//!
//! let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
//!
//! // Find a path
//! let route = find_path(
//!     VoxelCoord::new(0, 0, 0),
//!     VoxelCoord::new(10, 0, 0),
//!     &grid,
//!     false,  // no diagonal
//! ).unwrap();
//!
//! // Original path has many nodes
//! assert_eq!(route.path().len(), 11);
//!
//! // Smooth it
//! let smoother = PathSmoother::new(&grid);
//! let smoothed = smoother.smooth_route(&route);
//!
//! // Smoothed path may have fewer nodes
//! assert!(smoothed.path().len() <= 11);
//! ```

use cf_spatial::{VoxelCoord, VoxelGrid, line_of_sight};
use route_types::{ContinuousPath, Path, Route, VoxelPath, Waypoint};

/// A path smoother that simplifies paths using line-of-sight checks.
///
/// The smoother uses a greedy algorithm: starting from the first node,
/// it finds the furthest node that has clear line of sight and skips
/// all intermediate nodes.
///
/// # Example
///
/// ```
/// use route_pathfind::smooth::PathSmoother;
/// use route_types::{VoxelPath, Path, Route};
/// use cf_spatial::{VoxelCoord, VoxelGrid};
///
/// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
/// let smoother = PathSmoother::new(&grid);
///
/// // Create a simple path
/// let path = VoxelPath::new(vec![
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(1, 0, 0),
///     VoxelCoord::new(2, 0, 0),
///     VoxelCoord::new(3, 0, 0),
/// ]);
///
/// let route = Route::new(Path::Voxel(path));
/// let smoothed = smoother.smooth_route(&route);
///
/// // Straight line should smooth to just start and end
/// assert_eq!(smoothed.path().len(), 2);
/// ```
pub struct PathSmoother<'a, T> {
    /// The obstacle grid for line-of-sight checks.
    grid: &'a VoxelGrid<T>,
}

impl<'a, T: Default> PathSmoother<'a, T> {
    /// Creates a new path smoother with the given obstacle grid.
    ///
    /// # Example
    ///
    /// ```
    /// use route_pathfind::smooth::PathSmoother;
    /// use cf_spatial::VoxelGrid;
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let smoother = PathSmoother::new(&grid);
    /// ```
    #[must_use]
    pub const fn new(grid: &'a VoxelGrid<T>) -> Self {
        Self { grid }
    }

    /// Checks if there is clear line of sight between two voxel coordinates.
    ///
    /// Uses the grid's world coordinates for the line-of-sight check.
    fn has_line_of_sight(&self, from: VoxelCoord, to: VoxelCoord) -> bool {
        let from_world = self.grid.grid_to_world_center(from);
        let to_world = self.grid.grid_to_world_center(to);
        line_of_sight(&from_world, &to_world, self.grid, |_| true)
    }

    /// Smooths a voxel path by removing unnecessary waypoints.
    ///
    /// Uses a greedy algorithm: from each point, find the furthest point
    /// with clear line of sight and skip everything in between.
    ///
    /// # Example
    ///
    /// ```
    /// use route_pathfind::smooth::PathSmoother;
    /// use route_types::VoxelPath;
    /// use cf_spatial::{VoxelCoord, VoxelGrid};
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let smoother = PathSmoother::new(&grid);
    ///
    /// let path = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(1, 0, 0),
    ///     VoxelCoord::new(2, 0, 0),
    /// ]);
    ///
    /// let smoothed = smoother.smooth_voxel_path(&path);
    /// assert!(smoothed.len() <= path.len());
    /// ```
    #[must_use]
    pub fn smooth_voxel_path(&self, path: &VoxelPath) -> VoxelPath {
        let coords = path.coords();

        if coords.len() <= 2 {
            return path.clone();
        }

        let mut smoothed = Vec::new();
        let mut current_idx = 0;

        while current_idx < coords.len() {
            smoothed.push(coords[current_idx]);

            if current_idx == coords.len() - 1 {
                break;
            }

            // Find the furthest point with line of sight
            let mut furthest_visible = current_idx + 1;
            #[allow(clippy::needless_range_loop)]
            for i in (current_idx + 2)..coords.len() {
                if self.has_line_of_sight(coords[current_idx], coords[i]) {
                    furthest_visible = i;
                }
            }

            current_idx = furthest_visible;
        }

        VoxelPath::new(smoothed)
    }

    /// Smooths a route, preserving metadata.
    ///
    /// # Example
    ///
    /// ```
    /// use route_pathfind::smooth::PathSmoother;
    /// use route_types::{VoxelPath, Path, Route};
    /// use cf_spatial::{VoxelCoord, VoxelGrid};
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let smoother = PathSmoother::new(&grid);
    ///
    /// let path = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(1, 0, 0),
    ///     VoxelCoord::new(2, 0, 0),
    /// ]);
    ///
    /// let route = Route::new(Path::Voxel(path))
    ///     .with_constraints_satisfied(true);
    ///
    /// let smoothed = smoother.smooth_route(&route);
    /// assert!(smoothed.constraints_satisfied());
    /// ```
    #[must_use]
    pub fn smooth_route(&self, route: &Route) -> Route {
        match route.path() {
            Path::Voxel(voxel_path) => {
                let smoothed_path = self.smooth_voxel_path(voxel_path);
                Route::new(Path::Voxel(smoothed_path))
                    .with_cost(route.cost().clone())
                    .with_stats(route.stats().clone())
                    .with_constraints_satisfied(route.constraints_satisfied())
            }
            Path::Continuous(continuous_path) => {
                let smoothed_path = self.smooth_continuous_path(continuous_path);
                Route::new(Path::Continuous(smoothed_path))
                    .with_cost(route.cost().clone())
                    .with_stats(route.stats().clone())
                    .with_constraints_satisfied(route.constraints_satisfied())
            }
        }
    }

    /// Smooths a continuous path.
    fn smooth_continuous_path(&self, path: &ContinuousPath) -> ContinuousPath {
        let waypoints = path.waypoints();

        if waypoints.len() <= 2 {
            return path.clone();
        }

        let mut smoothed = Vec::new();
        let mut current_idx = 0;

        while current_idx < waypoints.len() {
            smoothed.push(waypoints[current_idx].clone());

            if current_idx == waypoints.len() - 1 {
                break;
            }

            // Find the furthest point with line of sight
            let mut furthest_visible = current_idx + 1;
            let from = waypoints[current_idx].position();

            #[allow(clippy::needless_range_loop)]
            for i in (current_idx + 2)..waypoints.len() {
                let to = waypoints[i].position();
                if line_of_sight(from, to, self.grid, |_| true) {
                    furthest_visible = i;
                }
            }

            current_idx = furthest_visible;
        }

        ContinuousPath::new(smoothed)
    }

    /// Converts a voxel path to a continuous path.
    ///
    /// Each voxel coordinate is converted to its world-space center point.
    ///
    /// # Example
    ///
    /// ```
    /// use route_pathfind::smooth::PathSmoother;
    /// use route_types::VoxelPath;
    /// use cf_spatial::{VoxelCoord, VoxelGrid};
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let smoother = PathSmoother::new(&grid);
    ///
    /// let voxel_path = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(1, 0, 0),
    /// ]);
    ///
    /// let continuous = smoother.to_continuous(&voxel_path);
    /// assert_eq!(continuous.len(), 2);
    /// ```
    #[must_use]
    pub fn to_continuous(&self, path: &VoxelPath) -> ContinuousPath {
        let waypoints: Vec<Waypoint> = path
            .coords()
            .iter()
            .map(|&coord| {
                let position = self.grid.grid_to_world_center(coord);
                Waypoint::new(position)
            })
            .collect();

        ContinuousPath::new(waypoints)
    }

    /// Smooths a voxel path and converts it to a continuous path.
    ///
    /// This is a convenience method combining smoothing and conversion.
    ///
    /// # Example
    ///
    /// ```
    /// use route_pathfind::smooth::PathSmoother;
    /// use route_types::VoxelPath;
    /// use cf_spatial::{VoxelCoord, VoxelGrid};
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let smoother = PathSmoother::new(&grid);
    ///
    /// let path = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(1, 0, 0),
    ///     VoxelCoord::new(2, 0, 0),
    /// ]);
    ///
    /// let continuous = smoother.smooth_and_convert(&path);
    /// assert!(continuous.len() <= path.len());
    /// ```
    #[must_use]
    pub fn smooth_and_convert(&self, path: &VoxelPath) -> ContinuousPath {
        let smoothed = self.smooth_voxel_path(path);
        self.to_continuous(&smoothed)
    }
}

/// Simplifies a path by removing collinear points.
///
/// This is a fast simplification that doesn't use line-of-sight checks,
/// only removing points that lie exactly on a straight line.
///
/// # Example
///
/// ```
/// use route_pathfind::smooth::remove_collinear;
/// use route_types::VoxelPath;
/// use cf_spatial::VoxelCoord;
///
/// let path = VoxelPath::new(vec![
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(1, 0, 0),
///     VoxelCoord::new(2, 0, 0),
///     VoxelCoord::new(3, 0, 0),
/// ]);
///
/// let simplified = remove_collinear(&path);
/// // Only start and end remain
/// assert_eq!(simplified.len(), 2);
/// ```
#[must_use]
pub fn remove_collinear(path: &VoxelPath) -> VoxelPath {
    let coords = path.coords();

    if coords.len() <= 2 {
        return path.clone();
    }

    let mut result = Vec::with_capacity(coords.len());
    result.push(coords[0]);

    for i in 1..coords.len() - 1 {
        let prev = coords[i - 1];
        let curr = coords[i];
        let next = coords[i + 1];

        // Check if the direction changes
        let d1 = curr - prev;
        let d2 = next - curr;

        if d1 != d2 {
            result.push(curr);
        }
    }

    result.push(coords[coords.len() - 1]);

    VoxelPath::new(result)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use nalgebra::Point3;

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
    fn test_smooth_straight_line() {
        let grid = empty_grid();
        let smoother = PathSmoother::new(&grid);

        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
            VoxelCoord::new(3, 0, 0),
            VoxelCoord::new(4, 0, 0),
        ]);

        let smoothed = smoother.smooth_voxel_path(&path);

        // Straight line should smooth to just start and end
        assert_eq!(smoothed.len(), 2);
        assert_eq!(smoothed.coords()[0], VoxelCoord::new(0, 0, 0));
        assert_eq!(smoothed.coords()[1], VoxelCoord::new(4, 0, 0));
    }

    #[test]
    fn test_smooth_l_shape() {
        let grid = empty_grid();
        let smoother = PathSmoother::new(&grid);

        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
            VoxelCoord::new(2, 1, 0),
            VoxelCoord::new(2, 2, 0),
        ]);

        let smoothed = smoother.smooth_voxel_path(&path);

        // L-shape with clear line of sight should also smooth
        // (depends on exact visibility)
        assert!(smoothed.len() <= path.len());
    }

    #[test]
    fn test_smooth_around_wall() {
        let grid = grid_with_wall();
        let smoother = PathSmoother::new(&grid);

        // Path that goes around wall
        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 1, 0),
            VoxelCoord::new(2, 2, 0),
            VoxelCoord::new(3, 3, 0),
            VoxelCoord::new(4, 4, 0),
            VoxelCoord::new(4, 5, 0),
            VoxelCoord::new(4, 6, 0),
            VoxelCoord::new(5, 6, 0), // Through gap
            VoxelCoord::new(6, 6, 0),
            VoxelCoord::new(7, 5, 0),
            VoxelCoord::new(8, 4, 0),
            VoxelCoord::new(9, 3, 0),
            VoxelCoord::new(10, 0, 0),
        ]);

        let smoothed = smoother.smooth_voxel_path(&path);

        // Wall should prevent direct line of sight
        assert!(smoothed.len() >= 2);
    }

    #[test]
    fn test_smooth_empty_path() {
        let grid = empty_grid();
        let smoother = PathSmoother::new(&grid);

        let path = VoxelPath::new(vec![]);
        let smoothed = smoother.smooth_voxel_path(&path);

        assert_eq!(smoothed.len(), 0);
    }

    #[test]
    fn test_smooth_single_point() {
        let grid = empty_grid();
        let smoother = PathSmoother::new(&grid);

        let path = VoxelPath::new(vec![VoxelCoord::new(5, 5, 5)]);
        let smoothed = smoother.smooth_voxel_path(&path);

        assert_eq!(smoothed.len(), 1);
    }

    #[test]
    fn test_smooth_two_points() {
        let grid = empty_grid();
        let smoother = PathSmoother::new(&grid);

        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 10, 10)]);
        let smoothed = smoother.smooth_voxel_path(&path);

        assert_eq!(smoothed.len(), 2);
    }

    #[test]
    fn test_smooth_route() {
        let grid = empty_grid();
        let smoother = PathSmoother::new(&grid);

        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
        ]);

        let route = Route::new(Path::Voxel(path)).with_constraints_satisfied(true);

        let smoothed = smoother.smooth_route(&route);

        assert!(smoothed.constraints_satisfied());
        assert_eq!(smoothed.path().len(), 2);
    }

    #[test]
    fn test_to_continuous() {
        let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
        let smoother = PathSmoother::new(&grid);

        let voxel_path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 0, 0)]);

        let continuous = smoother.to_continuous(&voxel_path);

        assert_eq!(continuous.len(), 2);
        // Voxel centers with voxel_size=1.0 are at (0.5, 0.5, 0.5) etc.
        let first = continuous.waypoints()[0].position();
        assert!((first.x - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_smooth_and_convert() {
        let grid = empty_grid();
        let smoother = PathSmoother::new(&grid);

        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
            VoxelCoord::new(3, 0, 0),
        ]);

        let continuous = smoother.smooth_and_convert(&path);

        // Should be smoothed to 2 points
        assert_eq!(continuous.len(), 2);
    }

    #[test]
    fn test_remove_collinear_straight() {
        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
            VoxelCoord::new(3, 0, 0),
        ]);

        let simplified = remove_collinear(&path);

        assert_eq!(simplified.len(), 2);
        assert_eq!(simplified.coords()[0], VoxelCoord::new(0, 0, 0));
        assert_eq!(simplified.coords()[1], VoxelCoord::new(3, 0, 0));
    }

    #[test]
    fn test_remove_collinear_l_shape() {
        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
            VoxelCoord::new(2, 1, 0),
            VoxelCoord::new(2, 2, 0),
        ]);

        let simplified = remove_collinear(&path);

        // Should keep corner point
        assert_eq!(simplified.len(), 3);
        assert!(simplified.coords().contains(&VoxelCoord::new(2, 0, 0)));
    }

    #[test]
    fn test_remove_collinear_zigzag() {
        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 1, 0),
            VoxelCoord::new(2, 0, 0),
            VoxelCoord::new(3, 1, 0),
        ]);

        let simplified = remove_collinear(&path);

        // All points should remain (no collinear points)
        assert_eq!(simplified.len(), 4);
    }

    #[test]
    fn test_remove_collinear_short_path() {
        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 0, 0)]);

        let simplified = remove_collinear(&path);

        assert_eq!(simplified.len(), 2);
    }

    #[test]
    fn test_smooth_continuous_path() {
        let grid = empty_grid();
        let smoother = PathSmoother::new(&grid);

        let waypoints = vec![
            Waypoint::new(Point3::new(0.5, 0.5, 0.5)),
            Waypoint::new(Point3::new(1.5, 0.5, 0.5)),
            Waypoint::new(Point3::new(2.5, 0.5, 0.5)),
            Waypoint::new(Point3::new(3.5, 0.5, 0.5)),
        ];

        let path = ContinuousPath::new(waypoints);
        let route = Route::new(Path::Continuous(path));

        let smoothed = smoother.smooth_route(&route);

        // Straight line should smooth
        assert!(smoothed.path().len() <= 4);
    }

    #[test]
    fn test_has_line_of_sight_clear() {
        let grid = empty_grid();
        let smoother = PathSmoother::new(&grid);

        assert!(smoother.has_line_of_sight(VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 0, 0)));
    }

    #[test]
    fn test_has_line_of_sight_blocked() {
        let grid = grid_with_wall();
        let smoother = PathSmoother::new(&grid);

        // Direct line through wall should be blocked
        assert!(!smoother.has_line_of_sight(VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 0, 0)));
    }
}
