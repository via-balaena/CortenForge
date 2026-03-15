//! Axis-aligned bounding box and axis enumeration.
//!
//! [`Aabb`] is the canonical bounding box type for all CortenForge domains.
//! It replaces the 7 independent implementations that existed across
//! mesh-types, sim-core, cf-spatial, route-types, and 3 private crates.
//!
//! # Constructors
//!
//! - [`Aabb::new`] — pre-validated min/max, `const`, no normalization (hot path).
//! - [`Aabb::from_corners`] — normalizes: auto-determines min/max per axis.
//! - [`Aabb::from_center`] — center + half-extents.
//! - [`Aabb::from_point`] — zero-volume box at a single point, `const`.
//! - [`Aabb::from_points`] — builds from an iterator of points.
//! - [`Aabb::from_triangle`] — builds from three vertices.
//! - [`Aabb::empty`] — sentinel for accumulation (min=+INF, max=-INF).

use nalgebra::{Point3, Vector3};

use crate::bounded::Bounded;

/// Spatial axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Axis {
    /// X axis (index 0).
    X,
    /// Y axis (index 1).
    Y,
    /// Z axis (index 2).
    Z,
}

impl Axis {
    /// Returns all three axes.
    #[must_use]
    pub const fn all() -> [Self; 3] {
        [Self::X, Self::Y, Self::Z]
    }

    /// Returns the axis index (0, 1, 2).
    #[must_use]
    pub const fn index(self) -> usize {
        match self {
            Self::X => 0,
            Self::Y => 1,
            Self::Z => 2,
        }
    }

    /// Creates an axis from an index (0, 1, 2). Returns `None` for invalid indices.
    #[must_use]
    pub const fn from_index(index: usize) -> Option<Self> {
        match index {
            0 => Some(Self::X),
            1 => Some(Self::Y),
            2 => Some(Self::Z),
            _ => None,
        }
    }

    /// Returns a unit vector along this axis.
    #[must_use]
    pub const fn unit_vector(self) -> Vector3<f64> {
        match self {
            Self::X => Vector3::new(1.0, 0.0, 0.0),
            Self::Y => Vector3::new(0.0, 1.0, 0.0),
            Self::Z => Vector3::new(0.0, 0.0, 1.0),
        }
    }
}

/// Axis-aligned bounding box.
///
/// The canonical AABB type for all CortenForge domains. Stores min and max
/// corners. All coordinates are `f64`.
///
/// # Invariants
///
/// For a valid (non-empty) box: `min[i] <= max[i]` for all axes. The sentinel
/// [`Aabb::empty()`] intentionally violates this (min > max) to enable
/// accumulation via [`Aabb::union`] and [`Aabb::expand_to_include`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Aabb {
    /// Minimum corner (component-wise minimum).
    pub min: Point3<f64>,
    /// Maximum corner (component-wise maximum).
    pub max: Point3<f64>,
}

impl Aabb {
    /// Creates an AABB from pre-validated min/max corners.
    ///
    /// Caller ensures `min[i] <= max[i]` for all axes. This is `const` and
    /// performs no normalization — use [`from_corners`](Self::from_corners)
    /// if the ordering is unknown.
    #[must_use]
    pub const fn new(min: Point3<f64>, max: Point3<f64>) -> Self {
        Self { min, max }
    }

    /// Creates an AABB from two arbitrary corners, normalizing per axis.
    #[must_use]
    pub fn from_corners(a: Point3<f64>, b: Point3<f64>) -> Self {
        Self {
            min: Point3::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z)),
            max: Point3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z)),
        }
    }

    /// Creates an AABB from a center point and half-extents.
    #[must_use]
    pub fn from_center(center: Point3<f64>, half_extents: Vector3<f64>) -> Self {
        let he = Vector3::new(
            half_extents.x.abs(),
            half_extents.y.abs(),
            half_extents.z.abs(),
        );
        Self {
            min: center - he,
            max: center + he,
        }
    }

    /// Creates a zero-volume AABB containing a single point.
    #[must_use]
    pub const fn from_point(point: Point3<f64>) -> Self {
        Self {
            min: point,
            max: point,
        }
    }

    /// Creates an AABB enclosing all points in the iterator.
    ///
    /// Returns [`Aabb::empty()`] if the iterator is empty.
    #[must_use]
    pub fn from_points<'a>(points: impl Iterator<Item = &'a Point3<f64>>) -> Self {
        let mut result = Self::empty();
        for p in points {
            result.expand_to_include(p);
        }
        result
    }

    /// Creates an AABB enclosing a triangle defined by three vertices.
    #[must_use]
    pub fn from_triangle(v0: &Point3<f64>, v1: &Point3<f64>, v2: &Point3<f64>) -> Self {
        Self {
            min: Point3::new(
                v0.x.min(v1.x).min(v2.x),
                v0.y.min(v1.y).min(v2.y),
                v0.z.min(v1.z).min(v2.z),
            ),
            max: Point3::new(
                v0.x.max(v1.x).max(v2.x),
                v0.y.max(v1.y).max(v2.y),
                v0.z.max(v1.z).max(v2.z),
            ),
        }
    }

    /// Returns the empty sentinel AABB (min=+INF, max=-INF).
    ///
    /// This is the identity element for [`union`](Self::union) and
    /// [`expand_to_include`](Self::expand_to_include): any valid AABB unioned
    /// with `empty()` returns itself.
    #[must_use]
    pub const fn empty() -> Self {
        Self {
            min: Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
            max: Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
        }
    }

    /// Returns `true` if this is the empty sentinel (min > max on any axis).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y || self.min.z > self.max.z
    }

    /// Returns `true` if all components are finite and min <= max per axis.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.min.x.is_finite()
            && self.min.y.is_finite()
            && self.min.z.is_finite()
            && self.max.x.is_finite()
            && self.max.y.is_finite()
            && self.max.z.is_finite()
            && self.min.x <= self.max.x
            && self.min.y <= self.max.y
            && self.min.z <= self.max.z
    }

    /// Returns the center point.
    #[must_use]
    pub fn center(&self) -> Point3<f64> {
        nalgebra::center(&self.min, &self.max)
    }

    /// Returns the half-extents (half the size along each axis).
    #[must_use]
    pub fn half_extents(&self) -> Vector3<f64> {
        (self.max - self.min) * 0.5
    }

    /// Returns the full size along each axis.
    #[must_use]
    pub fn size(&self) -> Vector3<f64> {
        self.max - self.min
    }

    /// Returns the volume. Zero or negative if empty.
    #[must_use]
    pub fn volume(&self) -> f64 {
        let s = self.size();
        s.x * s.y * s.z
    }

    /// Returns the surface area. Zero or negative if empty.
    #[must_use]
    pub fn surface_area(&self) -> f64 {
        let s = self.size();
        2.0 * s.x.mul_add(s.y, s.z.mul_add(s.x, s.y * s.z))
    }

    /// Returns the diagonal length.
    #[must_use]
    pub fn diagonal(&self) -> f64 {
        (self.max - self.min).norm()
    }

    /// Returns the longest edge length.
    #[must_use]
    pub fn max_extent(&self) -> f64 {
        let s = self.size();
        s.x.max(s.y).max(s.z)
    }

    /// Returns the shortest edge length.
    #[must_use]
    pub fn min_extent(&self) -> f64 {
        let s = self.size();
        s.x.min(s.y).min(s.z)
    }

    /// Returns the extent along a specific axis.
    #[must_use]
    pub fn extent(&self, axis: Axis) -> f64 {
        self.max[axis.index()] - self.min[axis.index()]
    }

    /// Returns the axis with the longest extent.
    #[must_use]
    pub fn longest_axis(&self) -> Axis {
        let s = self.size();
        if s.x >= s.y && s.x >= s.z {
            Axis::X
        } else if s.y >= s.z {
            Axis::Y
        } else {
            Axis::Z
        }
    }

    /// Returns `true` if the point is inside or on the boundary.
    #[must_use]
    pub fn contains(&self, point: &Point3<f64>) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    /// Returns `true` if the two AABBs overlap (share any volume or touch).
    #[must_use]
    pub fn overlaps(&self, other: &Self) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }

    /// Returns the intersection of two AABBs.
    ///
    /// If the boxes do not overlap, the result is empty.
    #[must_use]
    pub fn intersection(&self, other: &Self) -> Self {
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
        Self { min, max }
    }

    /// Returns the smallest AABB enclosing both boxes.
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

    /// Expands this AABB in-place to include another AABB.
    pub fn merge(&mut self, other: &Self) {
        self.min.x = self.min.x.min(other.min.x);
        self.min.y = self.min.y.min(other.min.y);
        self.min.z = self.min.z.min(other.min.z);
        self.max.x = self.max.x.max(other.max.x);
        self.max.y = self.max.y.max(other.max.y);
        self.max.z = self.max.z.max(other.max.z);
    }

    /// Expands this AABB to include the given point.
    pub fn expand_to_include(&mut self, point: &Point3<f64>) {
        self.min.x = self.min.x.min(point.x);
        self.min.y = self.min.y.min(point.y);
        self.min.z = self.min.z.min(point.z);
        self.max.x = self.max.x.max(point.x);
        self.max.y = self.max.y.max(point.y);
        self.max.z = self.max.z.max(point.z);
    }

    /// Returns a copy expanded by `margin` in all directions.
    #[must_use]
    pub fn expanded(&self, margin: f64) -> Self {
        let v = Vector3::new(margin, margin, margin);
        Self {
            min: self.min - v,
            max: self.max + v,
        }
    }

    /// Returns the 8 corners of the box.
    ///
    /// Order: (min.x, min.y, min.z) first, (max.x, max.y, max.z) last,
    /// iterating Z fastest.
    #[must_use]
    pub fn corners(&self) -> [Point3<f64>; 8] {
        let (lo, hi) = (self.min, self.max);
        [
            Point3::new(lo.x, lo.y, lo.z),
            Point3::new(lo.x, lo.y, hi.z),
            Point3::new(lo.x, hi.y, lo.z),
            Point3::new(lo.x, hi.y, hi.z),
            Point3::new(hi.x, lo.y, lo.z),
            Point3::new(hi.x, lo.y, hi.z),
            Point3::new(hi.x, hi.y, lo.z),
            Point3::new(hi.x, hi.y, hi.z),
        ]
    }
}

impl Default for Aabb {
    /// Returns [`Aabb::empty()`] — the accumulation sentinel.
    fn default() -> Self {
        Self::empty()
    }
}

impl Bounded for Aabb {
    fn aabb(&self) -> Aabb {
        *self
    }
}
