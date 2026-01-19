//! Overlap query utilities for voxel grids.
//!
//! This module provides spatial overlap queries to find voxels that intersect
//! with geometric primitives like axis-aligned bounding boxes (AABBs) and spheres.
//!
//! # Example
//!
//! ```
//! use cf_spatial::{VoxelGrid, VoxelCoord, Aabb, Sphere, query_aabb, query_sphere};
//! use nalgebra::Point3;
//!
//! let mut grid: VoxelGrid<u32> = VoxelGrid::new(1.0);
//! grid.set(VoxelCoord::new(5, 5, 5), 42);
//! grid.set(VoxelCoord::new(6, 5, 5), 43);
//!
//! // Find voxels in an AABB
//! let aabb = Aabb::new(Point3::new(4.0, 4.0, 4.0), Point3::new(7.0, 6.0, 6.0));
//! let results: Vec<_> = query_aabb(&grid, &aabb).collect();
//! assert_eq!(results.len(), 2);
//!
//! // Find voxels in a sphere
//! let sphere = Sphere::new(Point3::new(5.5, 5.5, 5.5), 1.5);
//! let results: Vec<_> = query_sphere(&grid, &sphere).collect();
//! ```

use nalgebra::Point3;

use crate::grid::{GridBounds, VoxelGrid};
use crate::voxel::VoxelCoord;

/// An axis-aligned bounding box in world coordinates.
///
/// # Example
///
/// ```
/// use cf_spatial::Aabb;
/// use nalgebra::Point3;
///
/// let aabb = Aabb::new(
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(10.0, 10.0, 10.0),
/// );
///
/// assert!(aabb.contains(&Point3::new(5.0, 5.0, 5.0)));
/// assert!(!aabb.contains(&Point3::new(15.0, 5.0, 5.0)));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    /// Minimum corner of the bounding box.
    pub min: Point3<f64>,
    /// Maximum corner of the bounding box.
    pub max: Point3<f64>,
}

impl Aabb {
    /// Creates a new AABB from minimum and maximum corners.
    ///
    /// The corners are automatically reordered if necessary.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::Aabb;
    /// use nalgebra::Point3;
    ///
    /// // Corners can be specified in any order
    /// let aabb = Aabb::new(
    ///     Point3::new(10.0, 10.0, 10.0),
    ///     Point3::new(0.0, 0.0, 0.0),
    /// );
    /// assert_eq!(aabb.min, Point3::new(0.0, 0.0, 0.0));
    /// assert_eq!(aabb.max, Point3::new(10.0, 10.0, 10.0));
    /// ```
    #[must_use]
    pub fn new(a: Point3<f64>, b: Point3<f64>) -> Self {
        Self {
            min: Point3::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z)),
            max: Point3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z)),
        }
    }

    /// Creates an AABB centered at a point with the given half-extents.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::Aabb;
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let aabb = Aabb::from_center(
    ///     Point3::new(5.0, 5.0, 5.0),
    ///     Vector3::new(2.0, 2.0, 2.0),
    /// );
    /// assert_eq!(aabb.min, Point3::new(3.0, 3.0, 3.0));
    /// assert_eq!(aabb.max, Point3::new(7.0, 7.0, 7.0));
    /// ```
    #[must_use]
    pub fn from_center(center: Point3<f64>, half_extents: nalgebra::Vector3<f64>) -> Self {
        Self {
            min: center - half_extents,
            max: center + half_extents,
        }
    }

    /// Returns the center point of the AABB.
    #[must_use]
    pub fn center(&self) -> Point3<f64> {
        Point3::new(
            (self.min.x + self.max.x) * 0.5,
            (self.min.y + self.max.y) * 0.5,
            (self.min.z + self.max.z) * 0.5,
        )
    }

    /// Returns the half-extents (half-size) of the AABB.
    #[must_use]
    pub fn half_extents(&self) -> nalgebra::Vector3<f64> {
        nalgebra::Vector3::new(
            (self.max.x - self.min.x) * 0.5,
            (self.max.y - self.min.y) * 0.5,
            (self.max.z - self.min.z) * 0.5,
        )
    }

    /// Returns the full size (dimensions) of the AABB.
    #[must_use]
    pub fn size(&self) -> nalgebra::Vector3<f64> {
        nalgebra::Vector3::new(
            self.max.x - self.min.x,
            self.max.y - self.min.y,
            self.max.z - self.min.z,
        )
    }

    /// Checks if a point is inside the AABB.
    ///
    /// Points on the boundary are considered inside.
    #[must_use]
    pub fn contains(&self, point: &Point3<f64>) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    /// Checks if this AABB intersects another AABB.
    #[must_use]
    pub fn intersects(&self, other: &Self) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }

    /// Checks if this AABB intersects a sphere.
    #[must_use]
    pub fn intersects_sphere(&self, sphere: &Sphere) -> bool {
        sphere.intersects_aabb(self)
    }

    /// Expands this AABB to include a point.
    pub fn expand_to_include(&mut self, point: &Point3<f64>) {
        self.min.x = self.min.x.min(point.x);
        self.min.y = self.min.y.min(point.y);
        self.min.z = self.min.z.min(point.z);
        self.max.x = self.max.x.max(point.x);
        self.max.y = self.max.y.max(point.y);
        self.max.z = self.max.z.max(point.z);
    }

    /// Returns a new AABB that is the union of this AABB and another.
    #[must_use]
    pub fn union(&self, other: &Self) -> Self {
        Self {
            min: Point3::new(
                self.min.x.min(other.min.x),
                self.min.y.min(other.min.y),
                self.min.z.min(other.min.z),
            ),
            max: Point3::new(
                self.max.x.max(other.max.x),
                self.max.y.max(other.max.y),
                self.max.z.max(other.max.z),
            ),
        }
    }

    /// Returns the intersection of this AABB and another, if they overlap.
    #[must_use]
    pub fn intersection(&self, other: &Self) -> Option<Self> {
        let min = Point3::new(
            self.min.x.max(other.min.x),
            self.min.y.max(other.min.y),
            self.min.z.max(other.min.z),
        );
        let max = Point3::new(
            self.max.x.min(other.max.x),
            self.max.y.min(other.max.y),
            self.max.z.min(other.max.z),
        );

        if min.x <= max.x && min.y <= max.y && min.z <= max.z {
            Some(Self { min, max })
        } else {
            None
        }
    }
}

impl Default for Aabb {
    fn default() -> Self {
        Self::new(Point3::origin(), Point3::origin())
    }
}

/// A sphere in world coordinates.
///
/// # Example
///
/// ```
/// use cf_spatial::Sphere;
/// use nalgebra::Point3;
///
/// let sphere = Sphere::new(Point3::new(5.0, 5.0, 5.0), 2.0);
///
/// assert!(sphere.contains(&Point3::new(5.0, 5.0, 5.0)));
/// assert!(sphere.contains(&Point3::new(6.0, 5.0, 5.0)));
/// assert!(!sphere.contains(&Point3::new(8.0, 5.0, 5.0)));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sphere {
    /// The center of the sphere.
    pub center: Point3<f64>,
    /// The radius of the sphere.
    pub radius: f64,
}

impl Sphere {
    /// Creates a new sphere with the given center and radius.
    ///
    /// The radius is clamped to be non-negative.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::Sphere;
    /// use nalgebra::Point3;
    ///
    /// let sphere = Sphere::new(Point3::origin(), 5.0);
    /// assert_eq!(sphere.radius, 5.0);
    /// ```
    #[must_use]
    pub const fn new(center: Point3<f64>, radius: f64) -> Self {
        Self {
            center,
            radius: if radius < 0.0 { -radius } else { radius },
        }
    }

    /// Checks if a point is inside or on the sphere.
    #[must_use]
    pub fn contains(&self, point: &Point3<f64>) -> bool {
        let dx = point.x - self.center.x;
        let dy = point.y - self.center.y;
        let dz = point.z - self.center.z;
        dz.mul_add(dz, dx.mul_add(dx, dy * dy)) <= self.radius * self.radius
    }

    /// Checks if this sphere intersects another sphere.
    #[must_use]
    pub fn intersects(&self, other: &Self) -> bool {
        let dx = other.center.x - self.center.x;
        let dy = other.center.y - self.center.y;
        let dz = other.center.z - self.center.z;
        let dist_sq = dz.mul_add(dz, dx.mul_add(dx, dy * dy));
        let radius_sum = self.radius + other.radius;
        dist_sq <= radius_sum * radius_sum
    }

    /// Checks if this sphere intersects an AABB.
    ///
    /// Uses the closest point on the AABB to the sphere center.
    #[must_use]
    pub fn intersects_aabb(&self, aabb: &Aabb) -> bool {
        // Find the closest point on the AABB to the sphere center
        let closest = Point3::new(
            self.center.x.clamp(aabb.min.x, aabb.max.x),
            self.center.y.clamp(aabb.min.y, aabb.max.y),
            self.center.z.clamp(aabb.min.z, aabb.max.z),
        );

        // Check if the closest point is within the sphere
        self.contains(&closest)
    }

    /// Returns the bounding AABB of this sphere.
    #[must_use]
    pub fn bounding_aabb(&self) -> Aabb {
        Aabb {
            min: Point3::new(
                self.center.x - self.radius,
                self.center.y - self.radius,
                self.center.z - self.radius,
            ),
            max: Point3::new(
                self.center.x + self.radius,
                self.center.y + self.radius,
                self.center.z + self.radius,
            ),
        }
    }
}

impl Default for Sphere {
    fn default() -> Self {
        Self::new(Point3::origin(), 1.0)
    }
}

/// Queries a voxel grid for all voxels that overlap with an AABB.
///
/// Returns an iterator over `(VoxelCoord, &T)` for each occupied voxel
/// whose bounding box intersects the query AABB.
///
/// # Example
///
/// ```
/// use cf_spatial::{VoxelGrid, VoxelCoord, Aabb, query_aabb};
/// use nalgebra::Point3;
///
/// let mut grid: VoxelGrid<u32> = VoxelGrid::new(1.0);
/// grid.set(VoxelCoord::new(5, 5, 5), 42);
/// grid.set(VoxelCoord::new(100, 100, 100), 99);
///
/// let aabb = Aabb::new(Point3::new(4.0, 4.0, 4.0), Point3::new(6.0, 6.0, 6.0));
/// let results: Vec<_> = query_aabb(&grid, &aabb).collect();
///
/// assert_eq!(results.len(), 1);
/// assert_eq!(results[0].0, VoxelCoord::new(5, 5, 5));
/// assert_eq!(*results[0].1, 42);
/// ```
pub fn query_aabb<'a, T>(
    grid: &'a VoxelGrid<T>,
    aabb: &'a Aabb,
) -> impl Iterator<Item = (VoxelCoord, &'a T)> {
    // Convert AABB to grid bounds
    let min = grid.world_to_grid(aabb.min);
    let max = grid.world_to_grid(aabb.max);
    let bounds = GridBounds::new(min, max);

    // Iterate over bounds and filter to existing voxels
    bounds
        .into_iter()
        .filter_map(move |coord| grid.get(coord).map(|value| (coord, value)))
}

/// Queries a voxel grid for all voxels that overlap with a sphere.
///
/// This function finds all voxels whose bounding box intersects the sphere.
/// For more precise sphere-voxel intersection, use `query_sphere_precise`.
///
/// # Example
///
/// ```
/// use cf_spatial::{VoxelGrid, VoxelCoord, Sphere, query_sphere};
/// use nalgebra::Point3;
///
/// let mut grid: VoxelGrid<u32> = VoxelGrid::new(1.0);
/// grid.set(VoxelCoord::new(5, 5, 5), 42);
///
/// let sphere = Sphere::new(Point3::new(5.5, 5.5, 5.5), 1.5);
/// let results: Vec<_> = query_sphere(&grid, &sphere).collect();
/// assert!(results.iter().any(|(c, _)| *c == VoxelCoord::new(5, 5, 5)));
/// ```
pub fn query_sphere<'a, T>(
    grid: &'a VoxelGrid<T>,
    sphere: &'a Sphere,
) -> impl Iterator<Item = (VoxelCoord, &'a T)> + 'a {
    // Get bounding AABB for the sphere
    let bounding = sphere.bounding_aabb();
    let min = grid.world_to_grid(bounding.min);
    let max = grid.world_to_grid(bounding.max);
    let bounds = GridBounds::new(min, max);

    let voxel_size = grid.voxel_size();
    let origin = *grid.origin();

    // Iterate over bounds and filter by sphere intersection
    bounds.into_iter().filter_map(move |coord| {
        // Check if voxel intersects sphere
        let voxel_min = Point3::new(
            f64::from(coord.x).mul_add(voxel_size, origin.x),
            f64::from(coord.y).mul_add(voxel_size, origin.y),
            f64::from(coord.z).mul_add(voxel_size, origin.z),
        );
        let voxel_max = Point3::new(
            voxel_min.x + voxel_size,
            voxel_min.y + voxel_size,
            voxel_min.z + voxel_size,
        );
        let voxel_aabb = Aabb {
            min: voxel_min,
            max: voxel_max,
        };

        if sphere.intersects_aabb(&voxel_aabb) {
            grid.get(coord).map(|value| (coord, value))
        } else {
            None
        }
    })
}

/// Queries a voxel grid for voxels that intersect with an AABB and satisfy a predicate.
///
/// # Example
///
/// ```
/// use cf_spatial::{VoxelGrid, VoxelCoord, Aabb, query_aabb_filter};
/// use nalgebra::Point3;
///
/// let mut grid: VoxelGrid<u32> = VoxelGrid::new(1.0);
/// grid.set(VoxelCoord::new(5, 5, 5), 10);
/// grid.set(VoxelCoord::new(6, 5, 5), 50);
///
/// let aabb = Aabb::new(Point3::new(4.0, 4.0, 4.0), Point3::new(7.0, 6.0, 6.0));
///
/// // Only get voxels with value > 20
/// let results: Vec<_> = query_aabb_filter(&grid, &aabb, |v| *v > 20).collect();
/// assert_eq!(results.len(), 1);
/// assert_eq!(*results[0].1, 50);
/// ```
pub fn query_aabb_filter<'a, T, F>(
    grid: &'a VoxelGrid<T>,
    aabb: &'a Aabb,
    predicate: F,
) -> impl Iterator<Item = (VoxelCoord, &'a T)>
where
    F: Fn(&T) -> bool + 'a,
{
    query_aabb(grid, aabb).filter(move |(_, value)| predicate(value))
}

/// Queries a voxel grid for voxels that intersect with a sphere and satisfy a predicate.
///
/// # Example
///
/// ```
/// use cf_spatial::{VoxelGrid, VoxelCoord, Sphere, query_sphere_filter};
/// use nalgebra::Point3;
///
/// let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
/// grid.set(VoxelCoord::new(5, 5, 5), true);
/// grid.set(VoxelCoord::new(6, 5, 5), false);
///
/// let sphere = Sphere::new(Point3::new(5.5, 5.5, 5.5), 2.0);
///
/// // Only get occupied voxels
/// let results: Vec<_> = query_sphere_filter(&grid, &sphere, |v| *v).collect();
/// assert_eq!(results.len(), 1);
/// ```
pub fn query_sphere_filter<'a, T, F>(
    grid: &'a VoxelGrid<T>,
    sphere: &'a Sphere,
    predicate: F,
) -> impl Iterator<Item = (VoxelCoord, &'a T)> + 'a
where
    F: Fn(&T) -> bool + 'a,
{
    query_sphere(grid, sphere).filter(move |(_, value)| predicate(value))
}

/// Counts the number of occupied voxels in an AABB.
///
/// # Example
///
/// ```
/// use cf_spatial::{VoxelGrid, VoxelCoord, Aabb, count_in_aabb};
/// use nalgebra::Point3;
///
/// let mut grid: VoxelGrid<()> = VoxelGrid::new(1.0);
/// grid.set(VoxelCoord::new(5, 5, 5), ());
/// grid.set(VoxelCoord::new(6, 5, 5), ());
///
/// let aabb = Aabb::new(Point3::new(4.0, 4.0, 4.0), Point3::new(7.0, 6.0, 6.0));
/// assert_eq!(count_in_aabb(&grid, &aabb), 2);
/// ```
#[must_use]
pub fn count_in_aabb<T>(grid: &VoxelGrid<T>, aabb: &Aabb) -> usize {
    query_aabb(grid, aabb).count()
}

/// Counts the number of occupied voxels in a sphere.
///
/// # Example
///
/// ```
/// use cf_spatial::{VoxelGrid, VoxelCoord, Sphere, count_in_sphere};
/// use nalgebra::Point3;
///
/// let mut grid: VoxelGrid<()> = VoxelGrid::new(1.0);
/// grid.set(VoxelCoord::new(5, 5, 5), ());
///
/// let sphere = Sphere::new(Point3::new(5.5, 5.5, 5.5), 1.5);
/// assert!(count_in_sphere(&grid, &sphere) >= 1);
/// ```
#[must_use]
pub fn count_in_sphere<T>(grid: &VoxelGrid<T>, sphere: &Sphere) -> usize {
    query_sphere(grid, sphere).count()
}

/// Checks if any voxel exists in an AABB.
///
/// More efficient than `count_in_aabb` > 0 as it short-circuits on first match.
#[must_use]
pub fn any_in_aabb<T>(grid: &VoxelGrid<T>, aabb: &Aabb) -> bool {
    query_aabb(grid, aabb).next().is_some()
}

/// Checks if any voxel exists in a sphere.
///
/// More efficient than `count_in_sphere` > 0 as it short-circuits on first match.
#[must_use]
pub fn any_in_sphere<T>(grid: &VoxelGrid<T>, sphere: &Sphere) -> bool {
    query_sphere(grid, sphere).next().is_some()
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::needless_collect)]
mod tests {
    use super::*;

    // AABB tests
    #[test]
    fn test_aabb_new() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0));
        assert_eq!(aabb.min, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(aabb.max, Point3::new(10.0, 10.0, 10.0));
    }

    #[test]
    fn test_aabb_new_reorders() {
        let aabb = Aabb::new(Point3::new(10.0, 10.0, 10.0), Point3::new(0.0, 0.0, 0.0));
        assert_eq!(aabb.min, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(aabb.max, Point3::new(10.0, 10.0, 10.0));
    }

    #[test]
    fn test_aabb_from_center() {
        let aabb = Aabb::from_center(
            Point3::new(5.0, 5.0, 5.0),
            nalgebra::Vector3::new(2.0, 2.0, 2.0),
        );
        assert_eq!(aabb.min, Point3::new(3.0, 3.0, 3.0));
        assert_eq!(aabb.max, Point3::new(7.0, 7.0, 7.0));
    }

    #[test]
    fn test_aabb_center() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0));
        let center = aabb.center();
        assert!((center.x - 5.0).abs() < f64::EPSILON);
        assert!((center.y - 5.0).abs() < f64::EPSILON);
        assert!((center.z - 5.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_aabb_contains() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0));
        assert!(aabb.contains(&Point3::new(5.0, 5.0, 5.0)));
        assert!(aabb.contains(&Point3::new(0.0, 0.0, 0.0))); // On boundary
        assert!(aabb.contains(&Point3::new(10.0, 10.0, 10.0))); // On boundary
        assert!(!aabb.contains(&Point3::new(11.0, 5.0, 5.0)));
    }

    #[test]
    fn test_aabb_intersects() {
        let a = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0));
        let b = Aabb::new(Point3::new(5.0, 5.0, 5.0), Point3::new(15.0, 15.0, 15.0));
        let c = Aabb::new(Point3::new(20.0, 20.0, 20.0), Point3::new(30.0, 30.0, 30.0));

        assert!(a.intersects(&b));
        assert!(b.intersects(&a));
        assert!(!a.intersects(&c));
    }

    #[test]
    fn test_aabb_union() {
        let a = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(5.0, 5.0, 5.0));
        let b = Aabb::new(Point3::new(3.0, 3.0, 3.0), Point3::new(10.0, 10.0, 10.0));
        let union = a.union(&b);
        assert_eq!(union.min, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(union.max, Point3::new(10.0, 10.0, 10.0));
    }

    #[test]
    fn test_aabb_intersection() {
        let a = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0));
        let b = Aabb::new(Point3::new(5.0, 5.0, 5.0), Point3::new(15.0, 15.0, 15.0));
        let intersection = a.intersection(&b).unwrap();
        assert_eq!(intersection.min, Point3::new(5.0, 5.0, 5.0));
        assert_eq!(intersection.max, Point3::new(10.0, 10.0, 10.0));
    }

    #[test]
    fn test_aabb_intersection_none() {
        let a = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(5.0, 5.0, 5.0));
        let b = Aabb::new(Point3::new(10.0, 10.0, 10.0), Point3::new(15.0, 15.0, 15.0));
        assert!(a.intersection(&b).is_none());
    }

    // Sphere tests
    #[test]
    fn test_sphere_new() {
        let sphere = Sphere::new(Point3::new(1.0, 2.0, 3.0), 5.0);
        assert_eq!(sphere.center, Point3::new(1.0, 2.0, 3.0));
        assert!((sphere.radius - 5.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_sphere_negative_radius() {
        let sphere = Sphere::new(Point3::origin(), -5.0);
        assert!((sphere.radius - 5.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_sphere_contains() {
        let sphere = Sphere::new(Point3::new(5.0, 5.0, 5.0), 2.0);
        assert!(sphere.contains(&Point3::new(5.0, 5.0, 5.0))); // Center
        assert!(sphere.contains(&Point3::new(6.0, 5.0, 5.0))); // Inside
        assert!(sphere.contains(&Point3::new(7.0, 5.0, 5.0))); // On boundary
        assert!(!sphere.contains(&Point3::new(8.0, 5.0, 5.0))); // Outside
    }

    #[test]
    fn test_sphere_intersects() {
        let a = Sphere::new(Point3::new(0.0, 0.0, 0.0), 5.0);
        let b = Sphere::new(Point3::new(8.0, 0.0, 0.0), 5.0);
        let c = Sphere::new(Point3::new(20.0, 0.0, 0.0), 5.0);

        assert!(a.intersects(&b)); // Overlapping
        assert!(!a.intersects(&c)); // Not overlapping
    }

    #[test]
    fn test_sphere_intersects_aabb() {
        let sphere = Sphere::new(Point3::new(5.0, 5.0, 5.0), 2.0);
        let aabb1 = Aabb::new(Point3::new(4.0, 4.0, 4.0), Point3::new(6.0, 6.0, 6.0));
        let aabb2 = Aabb::new(Point3::new(10.0, 10.0, 10.0), Point3::new(12.0, 12.0, 12.0));

        assert!(sphere.intersects_aabb(&aabb1));
        assert!(!sphere.intersects_aabb(&aabb2));
    }

    #[test]
    fn test_sphere_bounding_aabb() {
        let sphere = Sphere::new(Point3::new(5.0, 5.0, 5.0), 2.0);
        let aabb = sphere.bounding_aabb();
        assert_eq!(aabb.min, Point3::new(3.0, 3.0, 3.0));
        assert_eq!(aabb.max, Point3::new(7.0, 7.0, 7.0));
    }

    // Query tests
    #[test]
    fn test_query_aabb() {
        let mut grid: VoxelGrid<u32> = VoxelGrid::new(1.0);
        grid.set(VoxelCoord::new(5, 5, 5), 42);
        grid.set(VoxelCoord::new(6, 5, 5), 43);
        grid.set(VoxelCoord::new(100, 100, 100), 99);

        let aabb = Aabb::new(Point3::new(4.0, 4.0, 4.0), Point3::new(7.0, 6.0, 6.0));
        let results: Vec<_> = query_aabb(&grid, &aabb).collect();

        assert_eq!(results.len(), 2);
    }

    #[test]
    fn test_query_aabb_empty() {
        let grid: VoxelGrid<u32> = VoxelGrid::new(1.0);
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0));
        let results: Vec<_> = query_aabb(&grid, &aabb).collect();
        assert!(results.is_empty());
    }

    #[test]
    fn test_query_sphere() {
        let mut grid: VoxelGrid<u32> = VoxelGrid::new(1.0);
        grid.set(VoxelCoord::new(5, 5, 5), 42);

        let sphere = Sphere::new(Point3::new(5.5, 5.5, 5.5), 1.5);
        let results: Vec<_> = query_sphere(&grid, &sphere).collect();

        assert!(results.iter().any(|(c, _)| *c == VoxelCoord::new(5, 5, 5)));
    }

    #[test]
    fn test_query_aabb_filter() {
        let mut grid: VoxelGrid<u32> = VoxelGrid::new(1.0);
        grid.set(VoxelCoord::new(5, 5, 5), 10);
        grid.set(VoxelCoord::new(6, 5, 5), 50);

        let aabb = Aabb::new(Point3::new(4.0, 4.0, 4.0), Point3::new(7.0, 6.0, 6.0));
        let results: Vec<_> = query_aabb_filter(&grid, &aabb, |v| *v > 20).collect();

        assert_eq!(results.len(), 1);
        assert_eq!(*results[0].1, 50);
    }

    #[test]
    fn test_count_in_aabb() {
        let mut grid: VoxelGrid<()> = VoxelGrid::new(1.0);
        grid.set(VoxelCoord::new(5, 5, 5), ());
        grid.set(VoxelCoord::new(6, 5, 5), ());

        let aabb = Aabb::new(Point3::new(4.0, 4.0, 4.0), Point3::new(7.0, 6.0, 6.0));
        assert_eq!(count_in_aabb(&grid, &aabb), 2);
    }

    #[test]
    fn test_any_in_aabb() {
        let mut grid: VoxelGrid<()> = VoxelGrid::new(1.0);
        let aabb = Aabb::new(Point3::new(4.0, 4.0, 4.0), Point3::new(7.0, 6.0, 6.0));

        assert!(!any_in_aabb(&grid, &aabb));

        grid.set(VoxelCoord::new(5, 5, 5), ());
        assert!(any_in_aabb(&grid, &aabb));
    }

    #[test]
    fn test_any_in_sphere() {
        let mut grid: VoxelGrid<()> = VoxelGrid::new(1.0);
        let sphere = Sphere::new(Point3::new(5.5, 5.5, 5.5), 1.5);

        assert!(!any_in_sphere(&grid, &sphere));

        grid.set(VoxelCoord::new(5, 5, 5), ());
        assert!(any_in_sphere(&grid, &sphere));
    }

    #[test]
    fn test_aabb_default() {
        let aabb = Aabb::default();
        assert_eq!(aabb.min, Point3::origin());
        assert_eq!(aabb.max, Point3::origin());
    }

    #[test]
    fn test_sphere_default() {
        let sphere = Sphere::default();
        assert_eq!(sphere.center, Point3::origin());
        assert!((sphere.radius - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_aabb_expand_to_include() {
        let mut aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(5.0, 5.0, 5.0));
        aabb.expand_to_include(&Point3::new(10.0, 3.0, -1.0));
        assert_eq!(aabb.min, Point3::new(0.0, 0.0, -1.0));
        assert_eq!(aabb.max, Point3::new(10.0, 5.0, 5.0));
    }

    #[test]
    fn test_aabb_size() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 20.0, 30.0));
        let size = aabb.size();
        assert!((size.x - 10.0).abs() < f64::EPSILON);
        assert!((size.y - 20.0).abs() < f64::EPSILON);
        assert!((size.z - 30.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_aabb_half_extents() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 20.0, 30.0));
        let half = aabb.half_extents();
        assert!((half.x - 5.0).abs() < f64::EPSILON);
        assert!((half.y - 10.0).abs() < f64::EPSILON);
        assert!((half.z - 15.0).abs() < f64::EPSILON);
    }
}
