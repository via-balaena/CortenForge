//! Raycasting utilities for voxel grids.
//!
//! This module provides ray-voxel traversal using the DDA (Digital Differential Analyzer)
//! algorithm, commonly known as Amanatides & Woo's fast voxel traversal algorithm.
//!
//! # Algorithm
//!
//! The DDA algorithm efficiently traverses voxels along a ray by computing the
//! parametric distance to the next voxel boundary in each axis. At each step,
//! it advances to the nearest boundary, ensuring all intersected voxels are visited.
//!
//! # Example
//!
//! ```
//! use cf_spatial::{Ray, VoxelGrid, VoxelCoord};
//! use nalgebra::{Point3, Vector3};
//!
//! let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
//! grid.set(VoxelCoord::new(5, 0, 0), true);
//!
//! let ray = Ray::new(Point3::origin(), Vector3::x());
//! for (coord, t) in ray.traverse_grid(&grid).take(10) {
//!     if grid.get(coord).copied().unwrap_or(false) {
//!         println!("Hit obstacle at {:?}, t={}", coord, t);
//!         break;
//!     }
//! }
//! ```

use nalgebra::{Point3, Vector3};

use crate::grid::VoxelGrid;
use crate::voxel::VoxelCoord;

/// A ray defined by an origin point and a direction vector.
///
/// The direction does not need to be normalized, but must be non-zero.
///
/// # Example
///
/// ```
/// use cf_spatial::Ray;
/// use nalgebra::{Point3, Vector3};
///
/// let ray = Ray::new(
///     Point3::new(0.0, 0.0, 0.0),
///     Vector3::new(1.0, 0.0, 0.0),
/// );
///
/// let point = ray.point_at(5.0);
/// assert!((point.x - 5.0).abs() < 1e-10);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ray {
    /// The origin of the ray.
    pub origin: Point3<f64>,
    /// The direction of the ray (not necessarily normalized).
    pub direction: Vector3<f64>,
}

impl Ray {
    /// Creates a new ray with the given origin and direction.
    ///
    /// # Arguments
    ///
    /// * `origin` - The starting point of the ray
    /// * `direction` - The direction vector (not necessarily normalized)
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::Ray;
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let ray = Ray::new(Point3::origin(), Vector3::x());
    /// assert_eq!(ray.origin, Point3::origin());
    /// ```
    #[must_use]
    pub const fn new(origin: Point3<f64>, direction: Vector3<f64>) -> Self {
        Self { origin, direction }
    }

    /// Returns the point along the ray at parameter `t`.
    ///
    /// The point is computed as `origin + t * direction`.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::Ray;
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let ray = Ray::new(Point3::origin(), Vector3::new(2.0, 0.0, 0.0));
    /// let p = ray.point_at(3.0);
    /// assert!((p.x - 6.0).abs() < 1e-10);
    /// ```
    #[must_use]
    pub fn point_at(&self, t: f64) -> Point3<f64> {
        self.origin + self.direction * t
    }

    /// Returns a normalized version of this ray.
    ///
    /// The direction is normalized to unit length. If the direction is zero,
    /// returns the ray unchanged.
    #[must_use]
    pub fn normalized(&self) -> Self {
        let norm = self.direction.norm();
        if norm < f64::EPSILON {
            return *self;
        }
        Self {
            origin: self.origin,
            direction: self.direction / norm,
        }
    }

    /// Returns the direction normalized to unit length.
    ///
    /// If the direction is zero, returns the zero vector.
    #[must_use]
    pub fn direction_normalized(&self) -> Vector3<f64> {
        let norm = self.direction.norm();
        if norm < f64::EPSILON {
            return Vector3::zeros();
        }
        self.direction / norm
    }

    /// Creates an iterator that traverses voxels along this ray.
    ///
    /// Uses the DDA algorithm for efficient voxel traversal.
    ///
    /// # Arguments
    ///
    /// * `grid` - The voxel grid to traverse (used for coordinate conversion)
    ///
    /// # Returns
    ///
    /// An iterator yielding `(VoxelCoord, f64)` tuples where the second value
    /// is the parametric distance `t` at which the ray enters the voxel.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{Ray, VoxelGrid, VoxelCoord};
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let grid: VoxelGrid<()> = VoxelGrid::new(1.0);
    /// let ray = Ray::new(Point3::new(0.5, 0.5, 0.5), Vector3::x());
    ///
    /// let voxels: Vec<_> = ray.traverse_grid(&grid).take(5).collect();
    /// assert_eq!(voxels.len(), 5);
    /// assert_eq!(voxels[0].0, VoxelCoord::new(0, 0, 0));
    /// ```
    #[must_use]
    pub fn traverse_grid<T>(&self, grid: &VoxelGrid<T>) -> VoxelTraversal {
        VoxelTraversal::new(*self, grid.voxel_size(), grid.origin())
    }

    /// Creates an iterator that traverses voxels along this ray with custom parameters.
    ///
    /// # Arguments
    ///
    /// * `voxel_size` - The size of each voxel
    /// * `origin` - The world-space origin of the grid
    #[must_use]
    pub fn traverse(&self, voxel_size: f64, origin: &Point3<f64>) -> VoxelTraversal {
        VoxelTraversal::new(*self, voxel_size, origin)
    }
}

impl Default for Ray {
    fn default() -> Self {
        Self::new(Point3::origin(), Vector3::x())
    }
}

/// An iterator that traverses voxels along a ray using the DDA algorithm.
///
/// The iterator yields `(VoxelCoord, f64)` tuples where the second value
/// is the parametric distance `t` at which the ray enters the voxel.
///
/// # Example
///
/// ```
/// use cf_spatial::{Ray, VoxelGrid};
/// use nalgebra::{Point3, Vector3};
///
/// let grid: VoxelGrid<()> = VoxelGrid::new(0.5);
/// let ray = Ray::new(Point3::origin(), Vector3::new(1.0, 1.0, 0.0));
///
/// for (coord, t) in ray.traverse_grid(&grid).take(10) {
///     println!("Voxel {:?} at t={}", coord, t);
/// }
/// ```
#[derive(Debug, Clone)]
pub struct VoxelTraversal {
    /// Current voxel coordinate.
    current: VoxelCoord,
    /// Step direction for each axis (-1 or 1).
    step: [i32; 3],
    /// Parametric distance to the next boundary in each axis.
    t_max: [f64; 3],
    /// Parametric distance between boundaries in each axis.
    t_delta: [f64; 3],
    /// Current parametric distance along the ray.
    t_current: f64,
    /// Whether this is the first iteration.
    first: bool,
}

impl VoxelTraversal {
    /// Creates a new voxel traversal iterator.
    fn new(ray: Ray, voxel_size: f64, grid_origin: &Point3<f64>) -> Self {
        let voxel_size = voxel_size.abs().max(f64::EPSILON);
        let inv_voxel_size = 1.0 / voxel_size;

        // Convert ray origin to grid-relative coordinates
        let relative = ray.origin - grid_origin;

        // Find the starting voxel
        #[allow(clippy::cast_possible_truncation)]
        let current = VoxelCoord::new(
            (relative.x * inv_voxel_size).floor() as i32,
            (relative.y * inv_voxel_size).floor() as i32,
            (relative.z * inv_voxel_size).floor() as i32,
        );

        // Compute step direction and t_delta for each axis
        let mut step = [0i32; 3];
        let mut t_max = [f64::INFINITY; 3];
        let mut t_delta = [f64::INFINITY; 3];

        let dir = [ray.direction.x, ray.direction.y, ray.direction.z];
        let pos = [relative.x, relative.y, relative.z];
        let coord = [current.x, current.y, current.z];

        for i in 0..3 {
            if dir[i].abs() > f64::EPSILON {
                step[i] = if dir[i] > 0.0 { 1 } else { -1 };
                t_delta[i] = (voxel_size / dir[i]).abs();

                // Distance to the next voxel boundary
                let boundary = if dir[i] > 0.0 {
                    (f64::from(coord[i]) + 1.0) * voxel_size
                } else {
                    f64::from(coord[i]) * voxel_size
                };
                t_max[i] = (boundary - pos[i]) / dir[i];
            }
        }

        Self {
            current,
            step,
            t_max,
            t_delta,
            t_current: 0.0,
            first: true,
        }
    }
}

impl Iterator for VoxelTraversal {
    type Item = (VoxelCoord, f64);

    fn next(&mut self) -> Option<Self::Item> {
        // On first call, return the starting voxel
        if self.first {
            self.first = false;
            return Some((self.current, 0.0));
        }

        // Find the axis with the smallest t_max
        let min_axis = if self.t_max[0] < self.t_max[1] {
            if self.t_max[0] < self.t_max[2] { 0 } else { 2 }
        } else if self.t_max[1] < self.t_max[2] {
            1
        } else {
            2
        };

        // Advance to the next voxel
        self.t_current = self.t_max[min_axis];

        // Update the current coordinate
        match min_axis {
            0 => self.current.x = self.current.x.wrapping_add(self.step[0]),
            1 => self.current.y = self.current.y.wrapping_add(self.step[1]),
            _ => self.current.z = self.current.z.wrapping_add(self.step[2]),
        }

        // Update t_max for the axis we just crossed
        self.t_max[min_axis] += self.t_delta[min_axis];

        Some((self.current, self.t_current))
    }
}

/// Result of a raycast operation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RaycastHit {
    /// The voxel coordinate that was hit.
    pub coord: VoxelCoord,
    /// The parametric distance along the ray where the hit occurred.
    pub t: f64,
    /// The world-space point where the hit occurred.
    pub point: Point3<f64>,
    /// The normal of the voxel face that was hit.
    pub normal: Vector3<f64>,
}

/// Casts a ray against a voxel grid, returning the first occupied voxel hit.
///
/// # Arguments
///
/// * `ray` - The ray to cast
/// * `grid` - The voxel grid to test against
/// * `max_distance` - Maximum distance to search
/// * `is_solid` - Function to determine if a voxel blocks the ray
///
/// # Returns
///
/// `Some(RaycastHit)` if a solid voxel was hit, `None` otherwise.
///
/// # Example
///
/// ```
/// use cf_spatial::{raycast, Ray, VoxelGrid, VoxelCoord};
/// use nalgebra::{Point3, Vector3};
///
/// let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
/// grid.set(VoxelCoord::new(5, 0, 0), true);
///
/// let ray = Ray::new(Point3::origin(), Vector3::x());
/// let hit = raycast(&ray, &grid, 100.0, |v| *v);
///
/// assert!(hit.is_some());
/// let hit = hit.unwrap();
/// assert_eq!(hit.coord, VoxelCoord::new(5, 0, 0));
/// ```
#[must_use]
pub fn raycast<T, F>(
    ray: &Ray,
    grid: &VoxelGrid<T>,
    max_distance: f64,
    is_solid: F,
) -> Option<RaycastHit>
where
    F: Fn(&T) -> bool,
{
    let normalized = ray.normalized();
    let mut prev_coord: Option<VoxelCoord> = None;

    for (coord, t) in ray.traverse_grid(grid) {
        if t > max_distance {
            break;
        }

        if let Some(value) = grid.get(coord) {
            if is_solid(value) {
                // Compute the hit point
                let point = normalized.point_at(t);

                // Compute the normal from the face we entered
                let normal = prev_coord.map_or_else(
                    || -normalized.direction_normalized(),
                    |prev| {
                        let diff = coord - prev;
                        Vector3::new(-f64::from(diff.x), -f64::from(diff.y), -f64::from(diff.z))
                    },
                );

                return Some(RaycastHit {
                    coord,
                    t,
                    point,
                    normal,
                });
            }
        }

        prev_coord = Some(coord);
    }

    None
}

/// Casts a ray against a voxel grid, returning all occupied voxels hit.
///
/// # Arguments
///
/// * `ray` - The ray to cast
/// * `grid` - The voxel grid to test against
/// * `max_distance` - Maximum distance to search
/// * `is_solid` - Function to determine if a voxel blocks the ray
/// * `stop_at_first` - If true, stop after the first hit
///
/// # Returns
///
/// A vector of all `RaycastHit` results along the ray.
#[must_use]
pub fn raycast_all<T, F>(
    ray: &Ray,
    grid: &VoxelGrid<T>,
    max_distance: f64,
    is_solid: F,
    stop_at_first: bool,
) -> Vec<RaycastHit>
where
    F: Fn(&T) -> bool,
{
    let normalized = ray.normalized();
    let mut hits = Vec::new();
    let mut prev_coord: Option<VoxelCoord> = None;

    for (coord, t) in ray.traverse_grid(grid) {
        if t > max_distance {
            break;
        }

        if let Some(value) = grid.get(coord) {
            if is_solid(value) {
                let point = normalized.point_at(t);
                let normal = prev_coord.map_or_else(
                    || -normalized.direction_normalized(),
                    |prev| {
                        let diff = coord - prev;
                        Vector3::new(-f64::from(diff.x), -f64::from(diff.y), -f64::from(diff.z))
                    },
                );

                hits.push(RaycastHit {
                    coord,
                    t,
                    point,
                    normal,
                });

                if stop_at_first {
                    break;
                }
            }
        }

        prev_coord = Some(coord);
    }

    hits
}

/// Checks if there is a clear line of sight between two points.
///
/// # Arguments
///
/// * `from` - Starting point
/// * `to` - Target point
/// * `grid` - The voxel grid to test against
/// * `is_solid` - Function to determine if a voxel blocks visibility
///
/// # Returns
///
/// `true` if there is no solid voxel between the two points.
///
/// # Example
///
/// ```
/// use cf_spatial::{line_of_sight, VoxelGrid, VoxelCoord};
/// use nalgebra::Point3;
///
/// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
///
/// let from = Point3::new(0.0, 0.0, 0.0);
/// let to = Point3::new(10.0, 0.0, 0.0);
///
/// // Empty grid has clear line of sight
/// assert!(line_of_sight(&from, &to, &grid, |v| *v));
/// ```
#[must_use]
pub fn line_of_sight<T, F>(
    from: &Point3<f64>,
    to: &Point3<f64>,
    grid: &VoxelGrid<T>,
    is_solid: F,
) -> bool
where
    F: Fn(&T) -> bool,
{
    let direction = to - from;
    let distance = direction.norm();

    if distance < f64::EPSILON {
        return true;
    }

    let ray = Ray::new(*from, direction);
    raycast(&ray, grid, distance, is_solid).is_none()
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn test_ray_new() {
        let ray = Ray::new(Point3::new(1.0, 2.0, 3.0), Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(ray.origin, Point3::new(1.0, 2.0, 3.0));
        assert_eq!(ray.direction, Vector3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn test_ray_point_at() {
        let ray = Ray::new(Point3::origin(), Vector3::x());
        let p = ray.point_at(5.0);
        assert!((p.x - 5.0).abs() < 1e-10);
        assert!((p.y - 0.0).abs() < 1e-10);
        assert!((p.z - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_ray_normalized() {
        let ray = Ray::new(Point3::origin(), Vector3::new(3.0, 4.0, 0.0));
        let normalized = ray.normalized();
        assert!((normalized.direction.norm() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_ray_default() {
        let ray = Ray::default();
        assert_eq!(ray.origin, Point3::origin());
        assert_eq!(ray.direction, Vector3::x());
    }

    #[test]
    fn test_traverse_grid_along_x() {
        let grid: VoxelGrid<()> = VoxelGrid::new(1.0);
        let ray = Ray::new(Point3::new(0.5, 0.5, 0.5), Vector3::x());

        let voxels: Vec<_> = ray.traverse_grid(&grid).take(5).collect();

        assert_eq!(voxels.len(), 5);
        assert_eq!(voxels[0].0, VoxelCoord::new(0, 0, 0));
        assert_eq!(voxels[1].0, VoxelCoord::new(1, 0, 0));
        assert_eq!(voxels[2].0, VoxelCoord::new(2, 0, 0));
        assert_eq!(voxels[3].0, VoxelCoord::new(3, 0, 0));
        assert_eq!(voxels[4].0, VoxelCoord::new(4, 0, 0));
    }

    #[test]
    fn test_traverse_grid_diagonal() {
        let grid: VoxelGrid<()> = VoxelGrid::new(1.0);
        let ray = Ray::new(Point3::new(0.5, 0.5, 0.5), Vector3::new(1.0, 1.0, 0.0));

        let voxels: Vec<_> = ray.traverse_grid(&grid).take(10).collect();

        // Should traverse in a diagonal pattern
        assert_eq!(voxels[0].0, VoxelCoord::new(0, 0, 0));
        // Subsequent voxels should increment x and y
    }

    #[test]
    fn test_traverse_grid_negative_direction() {
        let grid: VoxelGrid<()> = VoxelGrid::new(1.0);
        let ray = Ray::new(Point3::new(5.5, 0.5, 0.5), Vector3::new(-1.0, 0.0, 0.0));

        let voxels: Vec<_> = ray.traverse_grid(&grid).take(5).collect();

        assert_eq!(voxels[0].0, VoxelCoord::new(5, 0, 0));
        assert_eq!(voxels[1].0, VoxelCoord::new(4, 0, 0));
        assert_eq!(voxels[2].0, VoxelCoord::new(3, 0, 0));
    }

    #[test]
    fn test_raycast_hit() {
        let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
        grid.set(VoxelCoord::new(5, 0, 0), true);

        let ray = Ray::new(Point3::new(0.5, 0.5, 0.5), Vector3::x());
        let hit = raycast(&ray, &grid, 100.0, |v| *v);

        assert!(hit.is_some());
        let hit = hit.unwrap();
        assert_eq!(hit.coord, VoxelCoord::new(5, 0, 0));
        assert!(hit.t > 0.0);
    }

    #[test]
    fn test_raycast_miss() {
        let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);

        let ray = Ray::new(Point3::new(0.5, 0.5, 0.5), Vector3::x());
        let hit = raycast(&ray, &grid, 10.0, |v| *v);

        assert!(hit.is_none());
    }

    #[test]
    fn test_raycast_max_distance() {
        let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
        grid.set(VoxelCoord::new(15, 0, 0), true);

        let ray = Ray::new(Point3::new(0.5, 0.5, 0.5), Vector3::x());
        let hit = raycast(&ray, &grid, 10.0, |v| *v);

        // Should miss because the target is beyond max_distance
        assert!(hit.is_none());
    }

    #[test]
    fn test_raycast_all() {
        let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
        grid.set(VoxelCoord::new(3, 0, 0), true);
        grid.set(VoxelCoord::new(5, 0, 0), true);
        grid.set(VoxelCoord::new(7, 0, 0), true);

        let ray = Ray::new(Point3::new(0.5, 0.5, 0.5), Vector3::x());
        let hits = raycast_all(&ray, &grid, 100.0, |v| *v, false);

        assert_eq!(hits.len(), 3);
        assert_eq!(hits[0].coord, VoxelCoord::new(3, 0, 0));
        assert_eq!(hits[1].coord, VoxelCoord::new(5, 0, 0));
        assert_eq!(hits[2].coord, VoxelCoord::new(7, 0, 0));
    }

    #[test]
    fn test_raycast_all_stop_at_first() {
        let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
        grid.set(VoxelCoord::new(3, 0, 0), true);
        grid.set(VoxelCoord::new(5, 0, 0), true);

        let ray = Ray::new(Point3::new(0.5, 0.5, 0.5), Vector3::x());
        let hits = raycast_all(&ray, &grid, 100.0, |v| *v, true);

        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].coord, VoxelCoord::new(3, 0, 0));
    }

    #[test]
    fn test_line_of_sight_clear() {
        let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);

        let from = Point3::new(0.0, 0.0, 0.0);
        let to = Point3::new(10.0, 0.0, 0.0);

        assert!(line_of_sight(&from, &to, &grid, |v| *v));
    }

    #[test]
    fn test_line_of_sight_blocked() {
        let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
        grid.set(VoxelCoord::new(5, 0, 0), true);

        let from = Point3::new(0.0, 0.0, 0.0);
        let to = Point3::new(10.0, 0.0, 0.0);

        assert!(!line_of_sight(&from, &to, &grid, |v| *v));
    }

    #[test]
    fn test_line_of_sight_same_point() {
        let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);

        let point = Point3::new(5.0, 5.0, 5.0);

        assert!(line_of_sight(&point, &point, &grid, |v| *v));
    }

    #[test]
    fn test_raycast_hit_normal() {
        let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
        grid.set(VoxelCoord::new(5, 0, 0), true);

        let ray = Ray::new(Point3::new(0.5, 0.5, 0.5), Vector3::x());
        let hit = raycast(&ray, &grid, 100.0, |v| *v).unwrap();

        // Normal should point in negative x direction (opposite to ray)
        assert!(hit.normal.x < 0.0);
    }

    #[test]
    fn test_traverse_with_origin() {
        let grid: VoxelGrid<()> = VoxelGrid::with_origin(1.0, Point3::new(10.0, 10.0, 10.0));
        let ray = Ray::new(Point3::new(10.5, 10.5, 10.5), Vector3::x());

        let voxels: Vec<_> = ray.traverse_grid(&grid).take(3).collect();

        assert_eq!(voxels[0].0, VoxelCoord::new(0, 0, 0));
        assert_eq!(voxels[1].0, VoxelCoord::new(1, 0, 0));
        assert_eq!(voxels[2].0, VoxelCoord::new(2, 0, 0));
    }

    #[test]
    fn test_traverse_custom_voxel_size() {
        let grid: VoxelGrid<()> = VoxelGrid::new(0.5);
        let ray = Ray::new(Point3::new(0.25, 0.25, 0.25), Vector3::x());

        let voxels: Vec<_> = ray.traverse_grid(&grid).take(3).collect();

        assert_eq!(voxels[0].0, VoxelCoord::new(0, 0, 0));
        assert_eq!(voxels[1].0, VoxelCoord::new(1, 0, 0));
    }

    #[test]
    fn test_raycast_at_start() {
        let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
        grid.set(VoxelCoord::new(0, 0, 0), true);

        let ray = Ray::new(Point3::new(0.5, 0.5, 0.5), Vector3::x());
        let hit = raycast(&ray, &grid, 100.0, |v| *v);

        assert!(hit.is_some());
        assert_eq!(hit.unwrap().coord, VoxelCoord::new(0, 0, 0));
        assert!((hit.unwrap().t - 0.0).abs() < f64::EPSILON);
    }
}
