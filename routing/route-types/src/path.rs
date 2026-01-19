//! Path representations for routing.
//!
//! This module defines types for representing paths in both discrete voxel space
//! and continuous world space, along with waypoint metadata.
//!
//! # Path Types
//!
//! - [`VoxelPath`]: A discrete path through voxel coordinates
//! - [`ContinuousPath`]: A smooth path in world space with waypoints
//! - [`Path`]: An enum unifying both representations
//!
//! # Example
//!
//! ```
//! use route_types::{VoxelPath, ContinuousPath, Path};
//! use cf_spatial::VoxelCoord;
//!
//! // Create a discrete voxel path
//! let coords = vec![
//!     VoxelCoord::new(0, 0, 0),
//!     VoxelCoord::new(1, 0, 0),
//!     VoxelCoord::new(2, 0, 0),
//! ];
//! let voxel_path = VoxelPath::new(coords);
//! assert_eq!(voxel_path.len(), 3);
//! ```

use cf_spatial::VoxelCoord;
use nalgebra::{Point3, Vector3};

/// A waypoint in a continuous path with optional metadata.
///
/// Waypoints represent points along a path with additional information
/// such as tangent direction and accumulated distance.
///
/// # Example
///
/// ```
/// use route_types::Waypoint;
/// use nalgebra::Point3;
///
/// let waypoint = Waypoint::new(Point3::new(1.0, 2.0, 3.0));
/// assert_eq!(waypoint.position(), &Point3::new(1.0, 2.0, 3.0));
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Waypoint {
    /// Position in world coordinates.
    position: Point3<f64>,
    /// Optional tangent direction at this point (normalized).
    tangent: Option<Vector3<f64>>,
    /// Accumulated distance from the path start.
    distance_from_start: f64,
}

impl Waypoint {
    /// Creates a new waypoint at the given position.
    ///
    /// The tangent and distance are unset and can be computed later.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::Waypoint;
    /// use nalgebra::Point3;
    ///
    /// let wp = Waypoint::new(Point3::new(1.0, 2.0, 3.0));
    /// assert!(wp.tangent().is_none());
    /// ```
    #[must_use]
    pub const fn new(position: Point3<f64>) -> Self {
        Self {
            position,
            tangent: None,
            distance_from_start: 0.0,
        }
    }

    /// Creates a waypoint with all fields specified.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::Waypoint;
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let wp = Waypoint::with_tangent(
    ///     Point3::new(1.0, 2.0, 3.0),
    ///     Some(Vector3::x()),
    ///     5.0,
    /// );
    /// assert!(wp.tangent().is_some());
    /// assert!((wp.distance_from_start() - 5.0).abs() < 1e-10);
    /// ```
    #[must_use]
    pub const fn with_tangent(
        position: Point3<f64>,
        tangent: Option<Vector3<f64>>,
        distance_from_start: f64,
    ) -> Self {
        Self {
            position,
            tangent,
            distance_from_start,
        }
    }

    /// Returns the position of this waypoint.
    #[must_use]
    pub const fn position(&self) -> &Point3<f64> {
        &self.position
    }

    /// Returns the tangent direction at this waypoint, if set.
    #[must_use]
    pub const fn tangent(&self) -> Option<&Vector3<f64>> {
        self.tangent.as_ref()
    }

    /// Returns the accumulated distance from the path start.
    #[must_use]
    pub const fn distance_from_start(&self) -> f64 {
        self.distance_from_start
    }

    /// Sets the tangent direction.
    pub const fn set_tangent(&mut self, tangent: Option<Vector3<f64>>) {
        self.tangent = tangent;
    }

    /// Sets the accumulated distance from start.
    pub const fn set_distance_from_start(&mut self, distance: f64) {
        self.distance_from_start = distance;
    }
}

impl Default for Waypoint {
    fn default() -> Self {
        Self::new(Point3::origin())
    }
}

/// A discrete path through voxel space.
///
/// Represents an ordered sequence of voxel coordinates forming a path.
/// The path length is computed based on the actual distances between
/// consecutive voxels (accounting for diagonal moves).
///
/// # Example
///
/// ```
/// use route_types::VoxelPath;
/// use cf_spatial::VoxelCoord;
///
/// let coords = vec![
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(1, 0, 0),
///     VoxelCoord::new(2, 0, 0),
/// ];
/// let path = VoxelPath::new(coords);
///
/// assert_eq!(path.len(), 3);
/// assert!(!path.is_empty());
/// assert!((path.length() - 2.0).abs() < 1e-10);
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VoxelPath {
    /// Ordered sequence of voxel coordinates.
    coords: Vec<VoxelCoord>,
    /// Cached path length (sum of segment lengths).
    length: f64,
}

impl VoxelPath {
    /// Creates a new voxel path from a sequence of coordinates.
    ///
    /// The path length is automatically computed.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::VoxelPath;
    /// use cf_spatial::VoxelCoord;
    ///
    /// let path = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(1, 1, 0),  // Diagonal move
    ///     VoxelCoord::new(2, 1, 0),
    /// ]);
    ///
    /// // Length includes sqrt(2) for diagonal + 1.0 for straight
    /// assert!(path.length() > 2.0);
    /// ```
    #[must_use]
    pub fn new(coords: Vec<VoxelCoord>) -> Self {
        let length = Self::compute_length(&coords);
        Self { coords, length }
    }

    /// Creates an empty voxel path.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::VoxelPath;
    ///
    /// let path = VoxelPath::empty();
    /// assert!(path.is_empty());
    /// assert!((path.length() - 0.0).abs() < 1e-10);
    /// ```
    #[must_use]
    pub const fn empty() -> Self {
        Self {
            coords: Vec::new(),
            length: 0.0,
        }
    }

    /// Creates a path from a single coordinate.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::VoxelPath;
    /// use cf_spatial::VoxelCoord;
    ///
    /// let path = VoxelPath::from_single(VoxelCoord::origin());
    /// assert_eq!(path.len(), 1);
    /// assert!((path.length() - 0.0).abs() < 1e-10);
    /// ```
    #[must_use]
    pub fn from_single(coord: VoxelCoord) -> Self {
        Self {
            coords: vec![coord],
            length: 0.0,
        }
    }

    /// Returns the number of coordinates in the path.
    #[must_use]
    pub fn len(&self) -> usize {
        self.coords.len()
    }

    /// Returns `true` if the path has no coordinates.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.coords.is_empty()
    }

    /// Returns the total path length (sum of segment lengths).
    ///
    /// Diagonal moves are counted as sqrt(2) for 2D diagonals
    /// and sqrt(3) for 3D diagonals.
    #[must_use]
    pub const fn length(&self) -> f64 {
        self.length
    }

    /// Returns the coordinates as a slice.
    #[must_use]
    pub fn coords(&self) -> &[VoxelCoord] {
        &self.coords
    }

    /// Returns the first coordinate, if any.
    #[must_use]
    pub fn first(&self) -> Option<&VoxelCoord> {
        self.coords.first()
    }

    /// Returns the last coordinate, if any.
    #[must_use]
    pub fn last(&self) -> Option<&VoxelCoord> {
        self.coords.last()
    }

    /// Returns the coordinate at the given index.
    #[must_use]
    pub fn get(&self, index: usize) -> Option<&VoxelCoord> {
        self.coords.get(index)
    }

    /// Returns an iterator over the coordinates.
    pub fn iter(&self) -> impl Iterator<Item = &VoxelCoord> {
        self.coords.iter()
    }

    /// Returns an iterator over consecutive coordinate pairs (segments).
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::VoxelPath;
    /// use cf_spatial::VoxelCoord;
    ///
    /// let path = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(1, 0, 0),
    ///     VoxelCoord::new(2, 0, 0),
    /// ]);
    ///
    /// let segments: Vec<_> = path.segments().collect();
    /// assert_eq!(segments.len(), 2);
    /// ```
    pub fn segments(&self) -> impl Iterator<Item = (&VoxelCoord, &VoxelCoord)> {
        self.coords.windows(2).map(|w| (&w[0], &w[1]))
    }

    /// Reverses the path direction.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::VoxelPath;
    /// use cf_spatial::VoxelCoord;
    ///
    /// let mut path = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(1, 0, 0),
    /// ]);
    /// path.reverse();
    ///
    /// assert_eq!(path.first(), Some(&VoxelCoord::new(1, 0, 0)));
    /// ```
    pub fn reverse(&mut self) {
        self.coords.reverse();
    }

    /// Returns a reversed copy of the path.
    #[must_use]
    pub fn reversed(&self) -> Self {
        let mut reversed = self.clone();
        reversed.reverse();
        reversed
    }

    /// Appends another path to this one.
    ///
    /// If the last coordinate of this path equals the first of the other,
    /// the duplicate is removed.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::VoxelPath;
    /// use cf_spatial::VoxelCoord;
    ///
    /// let mut path1 = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(1, 0, 0),
    /// ]);
    /// let path2 = VoxelPath::new(vec![
    ///     VoxelCoord::new(1, 0, 0),  // Duplicate
    ///     VoxelCoord::new(2, 0, 0),
    /// ]);
    ///
    /// path1.append(&path2);
    /// assert_eq!(path1.len(), 3);  // No duplicate
    /// ```
    pub fn append(&mut self, other: &Self) {
        if other.is_empty() {
            return;
        }

        // Check for duplicate at junction
        let skip_first = self.last() == other.first();

        let iter = other.coords.iter();
        let other_iter: Box<dyn Iterator<Item = &VoxelCoord>> = if skip_first {
            Box::new(iter.skip(1))
        } else {
            Box::new(iter)
        };

        self.coords.extend(other_iter.copied());
        self.length = Self::compute_length(&self.coords);
    }

    /// Computes the path length from a sequence of coordinates.
    fn compute_length(coords: &[VoxelCoord]) -> f64 {
        coords.windows(2).fold(0.0, |acc, w| {
            let diff = w[1] - w[0];
            let dx = f64::from(diff.x.abs());
            let dy = f64::from(diff.y.abs());
            let dz = f64::from(diff.z.abs());
            acc + dx.mul_add(dx, dy.mul_add(dy, dz * dz)).sqrt()
        })
    }
}

impl Default for VoxelPath {
    fn default() -> Self {
        Self::empty()
    }
}

impl FromIterator<VoxelCoord> for VoxelPath {
    fn from_iter<I: IntoIterator<Item = VoxelCoord>>(iter: I) -> Self {
        Self::new(iter.into_iter().collect())
    }
}

impl IntoIterator for VoxelPath {
    type Item = VoxelCoord;
    type IntoIter = std::vec::IntoIter<VoxelCoord>;

    fn into_iter(self) -> Self::IntoIter {
        self.coords.into_iter()
    }
}

impl<'a> IntoIterator for &'a VoxelPath {
    type Item = &'a VoxelCoord;
    type IntoIter = std::slice::Iter<'a, VoxelCoord>;

    fn into_iter(self) -> Self::IntoIter {
        self.coords.iter()
    }
}

/// A continuous path in world space.
///
/// Represents a smooth path defined by waypoints with world-space coordinates.
/// The path can include tangent information and accumulated distances.
///
/// # Example
///
/// ```
/// use route_types::{ContinuousPath, Waypoint};
/// use nalgebra::Point3;
///
/// let waypoints = vec![
///     Waypoint::new(Point3::new(0.0, 0.0, 0.0)),
///     Waypoint::new(Point3::new(1.0, 0.0, 0.0)),
///     Waypoint::new(Point3::new(2.0, 0.0, 0.0)),
/// ];
/// let path = ContinuousPath::new(waypoints);
///
/// assert_eq!(path.len(), 3);
/// assert!((path.length() - 2.0).abs() < 1e-10);
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ContinuousPath {
    /// Ordered sequence of waypoints.
    waypoints: Vec<Waypoint>,
    /// Cached path length.
    length: f64,
}

impl ContinuousPath {
    /// Creates a new continuous path from waypoints.
    ///
    /// The path length is automatically computed.
    #[must_use]
    pub fn new(waypoints: Vec<Waypoint>) -> Self {
        let length = Self::compute_length(&waypoints);
        Self { waypoints, length }
    }

    /// Creates an empty continuous path.
    #[must_use]
    pub const fn empty() -> Self {
        Self {
            waypoints: Vec::new(),
            length: 0.0,
        }
    }

    /// Creates a path from a sequence of points (without tangent info).
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::ContinuousPath;
    /// use nalgebra::Point3;
    ///
    /// let path = ContinuousPath::from_points(vec![
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(1.0, 0.0, 0.0),
    /// ]);
    ///
    /// assert_eq!(path.len(), 2);
    /// ```
    #[must_use]
    pub fn from_points(points: Vec<Point3<f64>>) -> Self {
        let waypoints: Vec<_> = points.into_iter().map(Waypoint::new).collect();
        Self::new(waypoints)
    }

    /// Returns the number of waypoints in the path.
    #[must_use]
    pub fn len(&self) -> usize {
        self.waypoints.len()
    }

    /// Returns `true` if the path has no waypoints.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.waypoints.is_empty()
    }

    /// Returns the total path length.
    #[must_use]
    pub const fn length(&self) -> f64 {
        self.length
    }

    /// Returns the waypoints as a slice.
    #[must_use]
    pub fn waypoints(&self) -> &[Waypoint] {
        &self.waypoints
    }

    /// Returns the first waypoint, if any.
    #[must_use]
    pub fn first(&self) -> Option<&Waypoint> {
        self.waypoints.first()
    }

    /// Returns the last waypoint, if any.
    #[must_use]
    pub fn last(&self) -> Option<&Waypoint> {
        self.waypoints.last()
    }

    /// Returns the waypoint at the given index.
    #[must_use]
    pub fn get(&self, index: usize) -> Option<&Waypoint> {
        self.waypoints.get(index)
    }

    /// Returns an iterator over the waypoints.
    pub fn iter(&self) -> impl Iterator<Item = &Waypoint> {
        self.waypoints.iter()
    }

    /// Extracts just the positions as points.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::{ContinuousPath, Waypoint};
    /// use nalgebra::Point3;
    ///
    /// let path = ContinuousPath::from_points(vec![
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(1.0, 0.0, 0.0),
    /// ]);
    ///
    /// let points = path.to_points();
    /// assert_eq!(points.len(), 2);
    /// ```
    #[must_use]
    pub fn to_points(&self) -> Vec<Point3<f64>> {
        self.waypoints.iter().map(|w| *w.position()).collect()
    }

    /// Reverses the path direction.
    pub fn reverse(&mut self) {
        self.waypoints.reverse();
        // Tangents would need to be negated for strict correctness,
        // but we leave them unchanged as they're optional metadata
    }

    /// Returns a reversed copy of the path.
    #[must_use]
    pub fn reversed(&self) -> Self {
        let mut reversed = self.clone();
        reversed.reverse();
        reversed
    }

    /// Computes the path length from waypoints.
    fn compute_length(waypoints: &[Waypoint]) -> f64 {
        waypoints.windows(2).fold(0.0, |acc, w| {
            acc + (w[1].position() - w[0].position()).norm()
        })
    }
}

impl Default for ContinuousPath {
    fn default() -> Self {
        Self::empty()
    }
}

/// Unified path representation supporting both discrete and continuous paths.
///
/// This enum allows algorithms to work with either representation
/// and convert between them as needed.
///
/// # Example
///
/// ```
/// use route_types::{Path, VoxelPath, ContinuousPath};
/// use cf_spatial::VoxelCoord;
///
/// let voxel_path = VoxelPath::new(vec![
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(1, 0, 0),
/// ]);
/// let path = Path::Voxel(voxel_path);
///
/// assert!(path.is_voxel());
/// assert!(!path.is_continuous());
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Path {
    /// A discrete path in voxel space.
    Voxel(VoxelPath),
    /// A continuous path in world space.
    Continuous(ContinuousPath),
}

impl Path {
    /// Returns `true` if this is a voxel path.
    #[must_use]
    pub const fn is_voxel(&self) -> bool {
        matches!(self, Self::Voxel(_))
    }

    /// Returns `true` if this is a continuous path.
    #[must_use]
    pub const fn is_continuous(&self) -> bool {
        matches!(self, Self::Continuous(_))
    }

    /// Returns the path length.
    #[must_use]
    pub const fn length(&self) -> f64 {
        match self {
            Self::Voxel(p) => p.length(),
            Self::Continuous(p) => p.length(),
        }
    }

    /// Returns the number of nodes/waypoints in the path.
    #[must_use]
    pub fn len(&self) -> usize {
        match self {
            Self::Voxel(p) => p.len(),
            Self::Continuous(p) => p.len(),
        }
    }

    /// Returns `true` if the path is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        match self {
            Self::Voxel(p) => p.is_empty(),
            Self::Continuous(p) => p.is_empty(),
        }
    }

    /// Returns the voxel path if this is one, `None` otherwise.
    #[must_use]
    pub const fn as_voxel(&self) -> Option<&VoxelPath> {
        match self {
            Self::Voxel(p) => Some(p),
            Self::Continuous(_) => None,
        }
    }

    /// Returns the continuous path if this is one, `None` otherwise.
    #[must_use]
    pub const fn as_continuous(&self) -> Option<&ContinuousPath> {
        match self {
            Self::Voxel(_) => None,
            Self::Continuous(p) => Some(p),
        }
    }

    /// Converts into the voxel path if this is one, `None` otherwise.
    #[must_use]
    pub fn into_voxel(self) -> Option<VoxelPath> {
        match self {
            Self::Voxel(p) => Some(p),
            Self::Continuous(_) => None,
        }
    }

    /// Converts into the continuous path if this is one, `None` otherwise.
    #[must_use]
    pub fn into_continuous(self) -> Option<ContinuousPath> {
        match self {
            Self::Voxel(_) => None,
            Self::Continuous(p) => Some(p),
        }
    }
}

impl From<VoxelPath> for Path {
    fn from(path: VoxelPath) -> Self {
        Self::Voxel(path)
    }
}

impl From<ContinuousPath> for Path {
    fn from(path: ContinuousPath) -> Self {
        Self::Continuous(path)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::needless_collect)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    // ==================== Waypoint Tests ====================

    #[test]
    fn test_waypoint_new() {
        let wp = Waypoint::new(Point3::new(1.0, 2.0, 3.0));
        assert_eq!(wp.position(), &Point3::new(1.0, 2.0, 3.0));
        assert!(wp.tangent().is_none());
        assert!((wp.distance_from_start() - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_waypoint_with_tangent() {
        let wp = Waypoint::with_tangent(Point3::new(1.0, 2.0, 3.0), Some(Vector3::x()), 5.0);
        assert!(wp.tangent().is_some());
        assert_eq!(wp.tangent().unwrap(), &Vector3::x());
        assert!((wp.distance_from_start() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_waypoint_setters() {
        let mut wp = Waypoint::new(Point3::origin());
        wp.set_tangent(Some(Vector3::y()));
        wp.set_distance_from_start(10.0);

        assert_eq!(wp.tangent().unwrap(), &Vector3::y());
        assert!((wp.distance_from_start() - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_waypoint_default() {
        let wp = Waypoint::default();
        assert_eq!(wp.position(), &Point3::origin());
    }

    // ==================== VoxelPath Tests ====================

    #[test]
    fn test_voxel_path_new() {
        let coords = vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
        ];
        let path = VoxelPath::new(coords);

        assert_eq!(path.len(), 3);
        assert!(!path.is_empty());
        assert_relative_eq!(path.length(), 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_voxel_path_empty() {
        let path = VoxelPath::empty();
        assert!(path.is_empty());
        assert_eq!(path.len(), 0);
        assert_relative_eq!(path.length(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_voxel_path_from_single() {
        let path = VoxelPath::from_single(VoxelCoord::origin());
        assert_eq!(path.len(), 1);
        assert_relative_eq!(path.length(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_voxel_path_diagonal() {
        let coords = vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 1, 0), // Diagonal
        ];
        let path = VoxelPath::new(coords);

        // sqrt(2) for 2D diagonal
        assert_relative_eq!(path.length(), std::f64::consts::SQRT_2, epsilon = 1e-10);
    }

    #[test]
    fn test_voxel_path_3d_diagonal() {
        let coords = vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 1, 1), // 3D diagonal
        ];
        let path = VoxelPath::new(coords);

        // sqrt(3) for 3D diagonal
        assert_relative_eq!(path.length(), 3.0_f64.sqrt(), epsilon = 1e-10);
    }

    #[test]
    fn test_voxel_path_first_last() {
        let coords = vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
        ];
        let path = VoxelPath::new(coords);

        assert_eq!(path.first(), Some(&VoxelCoord::new(0, 0, 0)));
        assert_eq!(path.last(), Some(&VoxelCoord::new(2, 0, 0)));
    }

    #[test]
    fn test_voxel_path_get() {
        let coords = vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 0, 0)];
        let path = VoxelPath::new(coords);

        assert_eq!(path.get(0), Some(&VoxelCoord::new(0, 0, 0)));
        assert_eq!(path.get(1), Some(&VoxelCoord::new(1, 0, 0)));
        assert_eq!(path.get(2), None);
    }

    #[test]
    fn test_voxel_path_segments() {
        let coords = vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
        ];
        let path = VoxelPath::new(coords);
        let segments: Vec<_> = path.segments().collect();

        assert_eq!(segments.len(), 2);
        assert_eq!(
            segments[0],
            (&VoxelCoord::new(0, 0, 0), &VoxelCoord::new(1, 0, 0))
        );
        assert_eq!(
            segments[1],
            (&VoxelCoord::new(1, 0, 0), &VoxelCoord::new(2, 0, 0))
        );
    }

    #[test]
    fn test_voxel_path_reverse() {
        let mut path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 0, 0)]);
        path.reverse();

        assert_eq!(path.first(), Some(&VoxelCoord::new(1, 0, 0)));
        assert_eq!(path.last(), Some(&VoxelCoord::new(0, 0, 0)));
    }

    #[test]
    fn test_voxel_path_reversed() {
        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 0, 0)]);
        let reversed = path.reversed();

        assert_eq!(reversed.first(), Some(&VoxelCoord::new(1, 0, 0)));
        // Original unchanged
        assert_eq!(path.first(), Some(&VoxelCoord::new(0, 0, 0)));
    }

    #[test]
    fn test_voxel_path_append() {
        let mut path1 = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 0, 0)]);
        let path2 = VoxelPath::new(vec![
            VoxelCoord::new(1, 0, 0), // Duplicate
            VoxelCoord::new(2, 0, 0),
        ]);

        path1.append(&path2);

        assert_eq!(path1.len(), 3); // Duplicate removed
        assert_relative_eq!(path1.length(), 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_voxel_path_append_no_overlap() {
        let mut path1 = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0)]);
        let path2 = VoxelPath::new(vec![VoxelCoord::new(2, 0, 0)]);

        path1.append(&path2);

        assert_eq!(path1.len(), 2);
    }

    #[test]
    fn test_voxel_path_append_empty() {
        let mut path1 = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0)]);
        let path2 = VoxelPath::empty();

        path1.append(&path2);

        assert_eq!(path1.len(), 1);
    }

    #[test]
    fn test_voxel_path_from_iter() {
        let coords = vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 0, 0)];
        let path: VoxelPath = coords.into_iter().collect();

        assert_eq!(path.len(), 2);
    }

    #[test]
    fn test_voxel_path_into_iter() {
        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 0, 0)]);
        let coords: Vec<_> = path.into_iter().collect();

        assert_eq!(coords.len(), 2);
    }

    #[test]
    fn test_voxel_path_iter_ref() {
        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 0, 0)]);
        let coords: Vec<_> = (&path).into_iter().collect();

        assert_eq!(coords.len(), 2);
        // path is still usable
        assert_eq!(path.len(), 2);
    }

    // ==================== ContinuousPath Tests ====================

    #[test]
    fn test_continuous_path_new() {
        let waypoints = vec![
            Waypoint::new(Point3::new(0.0, 0.0, 0.0)),
            Waypoint::new(Point3::new(1.0, 0.0, 0.0)),
            Waypoint::new(Point3::new(2.0, 0.0, 0.0)),
        ];
        let path = ContinuousPath::new(waypoints);

        assert_eq!(path.len(), 3);
        assert_relative_eq!(path.length(), 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_continuous_path_empty() {
        let path = ContinuousPath::empty();
        assert!(path.is_empty());
        assert_relative_eq!(path.length(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_continuous_path_from_points() {
        let path = ContinuousPath::from_points(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(3.0, 4.0, 0.0), // Distance = 5
        ]);

        assert_eq!(path.len(), 2);
        assert_relative_eq!(path.length(), 5.0, epsilon = 1e-10);
    }

    #[test]
    fn test_continuous_path_first_last() {
        let path = ContinuousPath::from_points(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        ]);

        assert!(path.first().is_some());
        assert!(path.last().is_some());
        assert_eq!(
            path.first().unwrap().position(),
            &Point3::new(0.0, 0.0, 0.0)
        );
    }

    #[test]
    fn test_continuous_path_to_points() {
        let path = ContinuousPath::from_points(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        ]);
        let points = path.to_points();

        assert_eq!(points.len(), 2);
        assert_eq!(points[0], Point3::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn test_continuous_path_reverse() {
        let mut path = ContinuousPath::from_points(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        ]);
        path.reverse();

        assert_eq!(
            path.first().unwrap().position(),
            &Point3::new(1.0, 0.0, 0.0)
        );
    }

    // ==================== Path Enum Tests ====================

    #[test]
    fn test_path_is_voxel() {
        let voxel = Path::Voxel(VoxelPath::empty());
        let continuous = Path::Continuous(ContinuousPath::empty());

        assert!(voxel.is_voxel());
        assert!(!voxel.is_continuous());
        assert!(!continuous.is_voxel());
        assert!(continuous.is_continuous());
    }

    #[test]
    fn test_path_length() {
        let voxel = Path::Voxel(VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
        ]));

        assert_relative_eq!(voxel.length(), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_path_len() {
        let path = Path::Voxel(VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
        ]));

        assert_eq!(path.len(), 2);
    }

    #[test]
    fn test_path_is_empty() {
        let empty = Path::Voxel(VoxelPath::empty());
        let non_empty = Path::Voxel(VoxelPath::from_single(VoxelCoord::origin()));

        assert!(empty.is_empty());
        assert!(!non_empty.is_empty());
    }

    #[test]
    fn test_path_as_voxel() {
        let path = Path::Voxel(VoxelPath::empty());
        assert!(path.as_voxel().is_some());
        assert!(path.as_continuous().is_none());
    }

    #[test]
    fn test_path_into_voxel() {
        let path = Path::Voxel(VoxelPath::empty());
        assert!(path.into_voxel().is_some());

        let path = Path::Continuous(ContinuousPath::empty());
        assert!(path.into_continuous().is_some());
    }

    #[test]
    fn test_path_from_voxel_path() {
        let voxel = VoxelPath::empty();
        let path: Path = voxel.into();
        assert!(path.is_voxel());
    }

    #[test]
    fn test_path_from_continuous_path() {
        let continuous = ContinuousPath::empty();
        let path: Path = continuous.into();
        assert!(path.is_continuous());
    }
}
