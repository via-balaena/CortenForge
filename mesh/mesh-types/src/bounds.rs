//! Axis-aligned bounding box.

use nalgebra::{Point3, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// An axis-aligned bounding box (AABB).
///
/// Represents a 3D box aligned with the coordinate axes, defined by
/// minimum and maximum corner points.
///
/// # Example
///
/// ```
/// use mesh_types::{Aabb, Point3};
///
/// let aabb = Aabb::new(
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(10.0, 10.0, 10.0),
/// );
///
/// assert_eq!(aabb.size(), Point3::new(10.0, 10.0, 10.0).coords);
/// assert!(aabb.contains(&Point3::new(5.0, 5.0, 5.0)));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Aabb {
    /// Minimum corner (smallest x, y, z values).
    pub min: Point3<f64>,
    /// Maximum corner (largest x, y, z values).
    pub max: Point3<f64>,
}

impl Aabb {
    /// Create a new AABB from minimum and maximum corners.
    ///
    /// The corners are automatically corrected if min > max for any axis.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{Aabb, Point3};
    ///
    /// let aabb = Aabb::new(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(1.0, 2.0, 3.0),
    /// );
    /// ```
    #[must_use]
    pub fn new(min: Point3<f64>, max: Point3<f64>) -> Self {
        Self {
            min: Point3::new(min.x.min(max.x), min.y.min(max.y), min.z.min(max.z)),
            max: Point3::new(min.x.max(max.x), min.y.max(max.y), min.z.max(max.z)),
        }
    }

    /// Create an AABB from a single point.
    ///
    /// The resulting box has zero volume.
    #[inline]
    #[must_use]
    pub const fn from_point(point: Point3<f64>) -> Self {
        Self {
            min: point,
            max: point,
        }
    }

    /// Create an empty (invalid) AABB.
    ///
    /// An empty AABB has min > max, which is useful as a starting point
    /// for expanding to include points.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{Aabb, Point3};
    ///
    /// let mut aabb = Aabb::empty();
    /// assert!(aabb.is_empty());
    ///
    /// aabb.expand_to_include(&Point3::new(1.0, 2.0, 3.0));
    /// assert!(!aabb.is_empty());
    /// ```
    #[must_use]
    #[allow(clippy::missing_const_for_fn)] // Point3::new is not const in nalgebra
    pub fn empty() -> Self {
        Self {
            min: Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
            max: Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
        }
    }

    /// Create an AABB from an iterator of points.
    ///
    /// Returns an empty AABB if the iterator is empty.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{Aabb, Point3};
    ///
    /// let points = vec![
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(10.0, 5.0, 3.0),
    ///     Point3::new(-2.0, 8.0, 1.0),
    /// ];
    ///
    /// let aabb = Aabb::from_points(points.iter());
    /// assert_eq!(aabb.min, Point3::new(-2.0, 0.0, 0.0));
    /// assert_eq!(aabb.max, Point3::new(10.0, 8.0, 3.0));
    /// ```
    #[must_use]
    pub fn from_points<'a>(points: impl Iterator<Item = &'a Point3<f64>>) -> Self {
        let mut aabb = Self::empty();
        for point in points {
            aabb.expand_to_include(point);
        }
        aabb
    }

    /// Check if the AABB is empty (has no valid volume).
    ///
    /// An AABB is empty if min > max for any axis.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y || self.min.z > self.max.z
    }

    /// Get the size (dimensions) of the AABB.
    ///
    /// Returns a vector with the width, height, and depth.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{Aabb, Point3};
    ///
    /// let aabb = Aabb::new(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(3.0, 4.0, 5.0),
    /// );
    /// let size = aabb.size();
    /// assert_eq!(size.x, 3.0);
    /// assert_eq!(size.y, 4.0);
    /// assert_eq!(size.z, 5.0);
    /// ```
    #[inline]
    #[must_use]
    pub fn size(&self) -> Vector3<f64> {
        self.max - self.min
    }

    /// Get the center of the AABB.
    #[inline]
    #[must_use]
    pub fn center(&self) -> Point3<f64> {
        Point3::new(
            (self.min.x + self.max.x) * 0.5,
            (self.min.y + self.max.y) * 0.5,
            (self.min.z + self.max.z) * 0.5,
        )
    }

    /// Get the volume of the AABB.
    ///
    /// Returns 0.0 for empty AABBs.
    #[inline]
    #[must_use]
    pub fn volume(&self) -> f64 {
        if self.is_empty() {
            return 0.0;
        }
        let s = self.size();
        s.x * s.y * s.z
    }

    /// Get the surface area of the AABB.
    ///
    /// Returns 0.0 for empty AABBs.
    #[inline]
    #[must_use]
    pub fn surface_area(&self) -> f64 {
        if self.is_empty() {
            return 0.0;
        }
        let s = self.size();
        2.0 * s.z.mul_add(s.x, s.x.mul_add(s.y, s.y * s.z))
    }

    /// Get the length of the longest edge.
    #[inline]
    #[must_use]
    pub fn max_extent(&self) -> f64 {
        let s = self.size();
        s.x.max(s.y).max(s.z)
    }

    /// Get the length of the shortest edge.
    #[inline]
    #[must_use]
    pub fn min_extent(&self) -> f64 {
        let s = self.size();
        s.x.min(s.y).min(s.z)
    }

    /// Get the diagonal length of the AABB.
    #[inline]
    #[must_use]
    pub fn diagonal(&self) -> f64 {
        self.size().norm()
    }

    /// Check if the AABB contains a point.
    ///
    /// Points on the boundary are considered inside.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{Aabb, Point3};
    ///
    /// let aabb = Aabb::new(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(10.0, 10.0, 10.0),
    /// );
    ///
    /// assert!(aabb.contains(&Point3::new(5.0, 5.0, 5.0)));
    /// assert!(aabb.contains(&Point3::new(0.0, 0.0, 0.0))); // boundary
    /// assert!(!aabb.contains(&Point3::new(-1.0, 5.0, 5.0)));
    /// ```
    #[inline]
    #[must_use]
    pub fn contains(&self, point: &Point3<f64>) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    /// Check if this AABB intersects another AABB.
    ///
    /// Touching AABBs are considered intersecting.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{Aabb, Point3};
    ///
    /// let a = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0));
    /// let b = Aabb::new(Point3::new(5.0, 5.0, 5.0), Point3::new(15.0, 15.0, 15.0));
    /// let c = Aabb::new(Point3::new(20.0, 20.0, 20.0), Point3::new(30.0, 30.0, 30.0));
    ///
    /// assert!(a.intersects(&b));
    /// assert!(!a.intersects(&c));
    /// ```
    #[inline]
    #[must_use]
    pub fn intersects(&self, other: &Self) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }

    /// Compute the intersection of two AABBs.
    ///
    /// Returns an empty AABB if they don't intersect.
    #[must_use]
    pub fn intersection(&self, other: &Self) -> Self {
        Self {
            min: Point3::new(
                self.min.x.max(other.min.x),
                self.min.y.max(other.min.y),
                self.min.z.max(other.min.z),
            ),
            max: Point3::new(
                self.max.x.min(other.max.x),
                self.max.y.min(other.max.y),
                self.max.z.min(other.max.z),
            ),
        }
    }

    /// Compute the union (enclosing AABB) of two AABBs.
    #[must_use]
    pub fn union(&self, other: &Self) -> Self {
        if self.is_empty() {
            return *other;
        }
        if other.is_empty() {
            return *self;
        }
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

    /// Expand the AABB to include a point.
    ///
    /// Modifies the AABB in place.
    pub fn expand_to_include(&mut self, point: &Point3<f64>) {
        self.min.x = self.min.x.min(point.x);
        self.min.y = self.min.y.min(point.y);
        self.min.z = self.min.z.min(point.z);
        self.max.x = self.max.x.max(point.x);
        self.max.y = self.max.y.max(point.y);
        self.max.z = self.max.z.max(point.z);
    }

    /// Expand the AABB by a uniform margin on all sides.
    ///
    /// # Arguments
    ///
    /// * `margin` - Distance to expand. Negative values shrink the AABB.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{Aabb, Point3};
    ///
    /// let aabb = Aabb::new(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(10.0, 10.0, 10.0),
    /// );
    ///
    /// let expanded = aabb.expanded(1.0);
    /// assert_eq!(expanded.min, Point3::new(-1.0, -1.0, -1.0));
    /// assert_eq!(expanded.max, Point3::new(11.0, 11.0, 11.0));
    /// ```
    #[must_use]
    pub fn expanded(&self, margin: f64) -> Self {
        Self {
            min: Point3::new(
                self.min.x - margin,
                self.min.y - margin,
                self.min.z - margin,
            ),
            max: Point3::new(
                self.max.x + margin,
                self.max.y + margin,
                self.max.z + margin,
            ),
        }
    }

    /// Get the eight corner points of the AABB.
    #[must_use]
    pub fn corners(&self) -> [Point3<f64>; 8] {
        [
            Point3::new(self.min.x, self.min.y, self.min.z),
            Point3::new(self.max.x, self.min.y, self.min.z),
            Point3::new(self.min.x, self.max.y, self.min.z),
            Point3::new(self.max.x, self.max.y, self.min.z),
            Point3::new(self.min.x, self.min.y, self.max.z),
            Point3::new(self.max.x, self.min.y, self.max.z),
            Point3::new(self.min.x, self.max.y, self.max.z),
            Point3::new(self.max.x, self.max.y, self.max.z),
        ]
    }
}

impl Default for Aabb {
    fn default() -> Self {
        Self::empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aabb_from_points() {
        let points = [Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 5.0, 3.0),
            Point3::new(-2.0, 8.0, 1.0)];

        let aabb = Aabb::from_points(points.iter());
        assert!((aabb.min.x - (-2.0)).abs() < f64::EPSILON);
        assert!((aabb.min.y - 0.0).abs() < f64::EPSILON);
        assert!((aabb.min.z - 0.0).abs() < f64::EPSILON);
        assert!((aabb.max.x - 10.0).abs() < f64::EPSILON);
        assert!((aabb.max.y - 8.0).abs() < f64::EPSILON);
        assert!((aabb.max.z - 3.0).abs() < f64::EPSILON);
    }

    #[test]
    fn aabb_empty() {
        let aabb = Aabb::empty();
        assert!(aabb.is_empty());
        assert!((aabb.volume() - 0.0).abs() < f64::EPSILON);
    }

    #[test]
    fn aabb_contains() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0));

        assert!(aabb.contains(&Point3::new(5.0, 5.0, 5.0)));
        assert!(aabb.contains(&Point3::new(0.0, 0.0, 0.0)));
        assert!(aabb.contains(&Point3::new(10.0, 10.0, 10.0)));
        assert!(!aabb.contains(&Point3::new(-1.0, 5.0, 5.0)));
    }

    #[test]
    fn aabb_intersects() {
        let a = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0));
        let b = Aabb::new(Point3::new(5.0, 5.0, 5.0), Point3::new(15.0, 15.0, 15.0));
        let c = Aabb::new(Point3::new(20.0, 20.0, 20.0), Point3::new(30.0, 30.0, 30.0));

        assert!(a.intersects(&b));
        assert!(b.intersects(&a));
        assert!(!a.intersects(&c));
        assert!(!c.intersects(&a));
    }

    #[test]
    fn aabb_volume() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 3.0, 4.0));
        assert!((aabb.volume() - 24.0).abs() < f64::EPSILON);
    }

    #[test]
    fn aabb_surface_area() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 3.0, 4.0));
        // 2 * (2*3 + 3*4 + 4*2) = 2 * (6 + 12 + 8) = 52
        assert!((aabb.surface_area() - 52.0).abs() < f64::EPSILON);
    }

    #[test]
    fn aabb_expanded() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0));
        let expanded = aabb.expanded(2.0);
        assert!((expanded.min.x - (-2.0)).abs() < f64::EPSILON);
        assert!((expanded.max.x - 12.0).abs() < f64::EPSILON);
    }

    #[test]
    fn aabb_union() {
        let a = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(5.0, 5.0, 5.0));
        let b = Aabb::new(Point3::new(3.0, 3.0, 3.0), Point3::new(10.0, 10.0, 10.0));
        let u = a.union(&b);
        assert!((u.min.x - 0.0).abs() < f64::EPSILON);
        assert!((u.max.x - 10.0).abs() < f64::EPSILON);
    }

    #[test]
    fn aabb_corners() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        let corners = aabb.corners();
        assert_eq!(corners.len(), 8);
        // Check one corner
        assert!((corners[7].x - 1.0).abs() < f64::EPSILON);
        assert!((corners[7].y - 1.0).abs() < f64::EPSILON);
        assert!((corners[7].z - 1.0).abs() < f64::EPSILON);
    }
}
