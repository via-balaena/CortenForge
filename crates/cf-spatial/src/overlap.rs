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
//! let aabb = Aabb::from_corners(Point3::new(4.0, 4.0, 4.0), Point3::new(7.0, 6.0, 6.0));
//! let results: Vec<_> = query_aabb(&grid, &aabb).collect();
//! assert_eq!(results.len(), 2);
//!
//! // Find voxels in a sphere
//! let sphere = Sphere::new(Point3::new(5.5, 5.5, 5.5), 1.5);
//! let results: Vec<_> = query_sphere(&grid, &sphere).collect();
//! ```

use cf_geometry::Bounded;
use nalgebra::Point3;

// Re-export canonical types from cf-geometry.
pub use cf_geometry::{Aabb, Sphere};

use crate::grid::{GridBounds, VoxelGrid};
use crate::voxel::VoxelCoord;

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
/// let aabb = Aabb::from_corners(Point3::new(4.0, 4.0, 4.0), Point3::new(6.0, 6.0, 6.0));
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
    let bounding = sphere.aabb();
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
        let voxel_aabb = Aabb::new(voxel_min, voxel_max);

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
/// let aabb = Aabb::from_corners(Point3::new(4.0, 4.0, 4.0), Point3::new(7.0, 6.0, 6.0));
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
/// let aabb = Aabb::from_corners(Point3::new(4.0, 4.0, 4.0), Point3::new(7.0, 6.0, 6.0));
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
    fn test_aabb_from_corners() {
        let aabb = Aabb::from_corners(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0));
        assert_eq!(aabb.min, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(aabb.max, Point3::new(10.0, 10.0, 10.0));
    }

    #[test]
    fn test_aabb_from_corners_reorders() {
        let aabb = Aabb::from_corners(Point3::new(10.0, 10.0, 10.0), Point3::new(0.0, 0.0, 0.0));
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
        let aabb = Aabb::from_corners(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0));
        let center = aabb.center();
        assert!((center.x - 5.0).abs() < f64::EPSILON);
        assert!((center.y - 5.0).abs() < f64::EPSILON);
        assert!((center.z - 5.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_aabb_contains() {
        let aabb = Aabb::from_corners(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0));
        assert!(aabb.contains(&Point3::new(5.0, 5.0, 5.0)));
        assert!(aabb.contains(&Point3::new(0.0, 0.0, 0.0))); // On boundary
        assert!(aabb.contains(&Point3::new(10.0, 10.0, 10.0))); // On boundary
        assert!(!aabb.contains(&Point3::new(11.0, 5.0, 5.0)));
    }

    #[test]
    fn test_aabb_overlaps() {
        let a = Aabb::from_corners(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0));
        let b = Aabb::from_corners(Point3::new(5.0, 5.0, 5.0), Point3::new(15.0, 15.0, 15.0));
        let c = Aabb::from_corners(Point3::new(20.0, 20.0, 20.0), Point3::new(30.0, 30.0, 30.0));

        assert!(a.overlaps(&b));
        assert!(b.overlaps(&a));
        assert!(!a.overlaps(&c));
    }

    #[test]
    fn test_aabb_union() {
        let a = Aabb::from_corners(Point3::new(0.0, 0.0, 0.0), Point3::new(5.0, 5.0, 5.0));
        let b = Aabb::from_corners(Point3::new(3.0, 3.0, 3.0), Point3::new(10.0, 10.0, 10.0));
        let union = a.union(&b);
        assert_eq!(union.min, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(union.max, Point3::new(10.0, 10.0, 10.0));
    }

    #[test]
    fn test_aabb_intersection() {
        let a = Aabb::from_corners(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0));
        let b = Aabb::from_corners(Point3::new(5.0, 5.0, 5.0), Point3::new(15.0, 15.0, 15.0));
        let intersection = a.intersection(&b);
        assert!(!intersection.is_empty());
        assert_eq!(intersection.min, Point3::new(5.0, 5.0, 5.0));
        assert_eq!(intersection.max, Point3::new(10.0, 10.0, 10.0));
    }

    #[test]
    fn test_aabb_intersection_none() {
        let a = Aabb::from_corners(Point3::new(0.0, 0.0, 0.0), Point3::new(5.0, 5.0, 5.0));
        let b = Aabb::from_corners(Point3::new(10.0, 10.0, 10.0), Point3::new(15.0, 15.0, 15.0));
        assert!(a.intersection(&b).is_empty());
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
        // cf-geometry clamps negative radius to 0
        assert!((sphere.radius - 0.0).abs() < f64::EPSILON);
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
        let aabb1 = Aabb::from_corners(Point3::new(4.0, 4.0, 4.0), Point3::new(6.0, 6.0, 6.0));
        let aabb2 =
            Aabb::from_corners(Point3::new(10.0, 10.0, 10.0), Point3::new(12.0, 12.0, 12.0));

        assert!(sphere.intersects_aabb(&aabb1));
        assert!(!sphere.intersects_aabb(&aabb2));
    }

    #[test]
    fn test_sphere_bounding_aabb() {
        let sphere = Sphere::new(Point3::new(5.0, 5.0, 5.0), 2.0);
        let aabb = sphere.aabb();
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

        let aabb = Aabb::from_corners(Point3::new(4.0, 4.0, 4.0), Point3::new(7.0, 6.0, 6.0));
        let results: Vec<_> = query_aabb(&grid, &aabb).collect();

        assert_eq!(results.len(), 2);
    }

    #[test]
    fn test_query_aabb_empty() {
        let grid: VoxelGrid<u32> = VoxelGrid::new(1.0);
        let aabb = Aabb::from_corners(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0));
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

        let aabb = Aabb::from_corners(Point3::new(4.0, 4.0, 4.0), Point3::new(7.0, 6.0, 6.0));
        let results: Vec<_> = query_aabb_filter(&grid, &aabb, |v| *v > 20).collect();

        assert_eq!(results.len(), 1);
        assert_eq!(*results[0].1, 50);
    }

    #[test]
    fn test_count_in_aabb() {
        let mut grid: VoxelGrid<()> = VoxelGrid::new(1.0);
        grid.set(VoxelCoord::new(5, 5, 5), ());
        grid.set(VoxelCoord::new(6, 5, 5), ());

        let aabb = Aabb::from_corners(Point3::new(4.0, 4.0, 4.0), Point3::new(7.0, 6.0, 6.0));
        assert_eq!(count_in_aabb(&grid, &aabb), 2);
    }

    #[test]
    fn test_any_in_aabb() {
        let mut grid: VoxelGrid<()> = VoxelGrid::new(1.0);
        let aabb = Aabb::from_corners(Point3::new(4.0, 4.0, 4.0), Point3::new(7.0, 6.0, 6.0));

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
        // cf-geometry default is the empty sentinel (min=+INF, max=-INF)
        assert!(aabb.is_empty());
    }

    #[test]
    fn test_sphere_default() {
        let sphere = Sphere::default();
        assert_eq!(sphere.center, Point3::origin());
        assert!((sphere.radius - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_aabb_expand_to_include() {
        let mut aabb = Aabb::from_corners(Point3::new(0.0, 0.0, 0.0), Point3::new(5.0, 5.0, 5.0));
        aabb.expand_to_include(&Point3::new(10.0, 3.0, -1.0));
        assert_eq!(aabb.min, Point3::new(0.0, 0.0, -1.0));
        assert_eq!(aabb.max, Point3::new(10.0, 5.0, 5.0));
    }

    #[test]
    fn test_aabb_size() {
        let aabb = Aabb::from_corners(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 20.0, 30.0));
        let size = aabb.size();
        assert!((size.x - 10.0).abs() < f64::EPSILON);
        assert!((size.y - 20.0).abs() < f64::EPSILON);
        assert!((size.z - 30.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_aabb_half_extents() {
        let aabb = Aabb::from_corners(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 20.0, 30.0));
        let half = aabb.half_extents();
        assert!((half.x - 5.0).abs() < f64::EPSILON);
        assert!((half.y - 10.0).abs() < f64::EPSILON);
        assert!((half.z - 15.0).abs() < f64::EPSILON);
    }
}
