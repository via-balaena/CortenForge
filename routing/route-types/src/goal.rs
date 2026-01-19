//! Goal specification for routing.
//!
//! This module defines types for specifying routing goals including:
//!
//! - [`GoalPoint`]: A single point specification (voxel, world, or region)
//! - [`Attractor`]: A soft constraint biasing the path toward a point
//! - [`RouteGoal`]: Complete goal specification with start, end, and via points
//!
//! # Example
//!
//! ```
//! use route_types::{RouteGoal, GoalPoint};
//! use cf_spatial::VoxelCoord;
//!
//! let goal = RouteGoal::new(
//!     GoalPoint::Voxel(VoxelCoord::new(0, 0, 0)),
//!     GoalPoint::Voxel(VoxelCoord::new(10, 10, 10)),
//! );
//! ```

use cf_spatial::VoxelCoord;
use nalgebra::Point3;

use crate::constraint::Aabb;

/// A single goal point specification.
///
/// Goal points can be specified in different ways depending on the use case:
/// - Exact voxel coordinate
/// - World-space point (converted to nearest voxel)
/// - Region (any point within the region satisfies the goal)
///
/// # Example
///
/// ```
/// use route_types::GoalPoint;
/// use cf_spatial::VoxelCoord;
/// use nalgebra::Point3;
///
/// // Exact voxel
/// let exact = GoalPoint::Voxel(VoxelCoord::new(5, 5, 5));
///
/// // World-space point
/// let world = GoalPoint::World(Point3::new(1.5, 2.5, 3.5));
///
/// // Any point in a region
/// use route_types::Aabb;
/// let region = GoalPoint::Region(Aabb::new(
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 1.0, 1.0),
/// ));
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum GoalPoint {
    /// An exact voxel coordinate.
    Voxel(VoxelCoord),
    /// A world-space point (will be converted to nearest voxel).
    World(Point3<f64>),
    /// Any point within a region satisfies the goal.
    Region(Aabb),
}

impl GoalPoint {
    /// Creates a goal point from a voxel coordinate.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::GoalPoint;
    /// use cf_spatial::VoxelCoord;
    ///
    /// let goal = GoalPoint::voxel(5, 5, 5);
    /// assert!(goal.is_voxel());
    /// ```
    #[must_use]
    pub const fn voxel(x: i32, y: i32, z: i32) -> Self {
        Self::Voxel(VoxelCoord::new(x, y, z))
    }

    /// Creates a goal point from world coordinates.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::GoalPoint;
    ///
    /// let goal = GoalPoint::world(1.5, 2.5, 3.5);
    /// assert!(goal.is_world());
    /// ```
    #[must_use]
    pub const fn world(x: f64, y: f64, z: f64) -> Self {
        Self::World(Point3::new(x, y, z))
    }

    /// Returns `true` if this is a voxel goal.
    #[must_use]
    pub const fn is_voxel(&self) -> bool {
        matches!(self, Self::Voxel(_))
    }

    /// Returns `true` if this is a world-space goal.
    #[must_use]
    pub const fn is_world(&self) -> bool {
        matches!(self, Self::World(_))
    }

    /// Returns `true` if this is a region goal.
    #[must_use]
    pub const fn is_region(&self) -> bool {
        matches!(self, Self::Region(_))
    }

    /// Returns the voxel coordinate if this is a voxel goal.
    #[must_use]
    pub const fn as_voxel(&self) -> Option<&VoxelCoord> {
        match self {
            Self::Voxel(c) => Some(c),
            _ => None,
        }
    }

    /// Returns the world point if this is a world goal.
    #[must_use]
    pub const fn as_world(&self) -> Option<&Point3<f64>> {
        match self {
            Self::World(p) => Some(p),
            _ => None,
        }
    }

    /// Returns the region if this is a region goal.
    #[must_use]
    pub const fn as_region(&self) -> Option<&Aabb> {
        match self {
            Self::Region(r) => Some(r),
            _ => None,
        }
    }

    /// Checks if a voxel coordinate satisfies this goal.
    ///
    /// For voxel goals, checks exact match.
    /// For world goals, this requires grid context and returns `false`.
    /// For region goals, converts voxel to world point (using unit voxels) and checks containment.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::GoalPoint;
    /// use cf_spatial::VoxelCoord;
    ///
    /// let goal = GoalPoint::Voxel(VoxelCoord::new(5, 5, 5));
    /// assert!(goal.is_satisfied_by_voxel(VoxelCoord::new(5, 5, 5)));
    /// assert!(!goal.is_satisfied_by_voxel(VoxelCoord::new(4, 5, 5)));
    /// ```
    #[must_use]
    pub fn is_satisfied_by_voxel(&self, coord: VoxelCoord) -> bool {
        match self {
            Self::Voxel(target) => coord == *target,
            Self::World(_) => false, // Requires grid context
            Self::Region(aabb) => {
                // Convert voxel center to world point (assuming unit voxels at origin)
                let point = Point3::new(
                    f64::from(coord.x) + 0.5,
                    f64::from(coord.y) + 0.5,
                    f64::from(coord.z) + 0.5,
                );
                aabb.contains(&point)
            }
        }
    }
}

impl From<VoxelCoord> for GoalPoint {
    fn from(coord: VoxelCoord) -> Self {
        Self::Voxel(coord)
    }
}

impl From<Point3<f64>> for GoalPoint {
    fn from(point: Point3<f64>) -> Self {
        Self::World(point)
    }
}

impl From<Aabb> for GoalPoint {
    fn from(aabb: Aabb) -> Self {
        Self::Region(aabb)
    }
}

/// An attractor point that biases the path toward a location.
///
/// Attractors are soft constraints that encourage (but don't require)
/// the path to pass near certain points. Useful for routing through
/// preferred areas or creating symmetric paths.
///
/// # Example
///
/// ```
/// use route_types::Attractor;
/// use nalgebra::Point3;
///
/// // Strong attractor at a waypoint
/// let attractor = Attractor::new(Point3::new(5.0, 5.0, 5.0), 2.0, 10.0);
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Attractor {
    /// Center of the attractor.
    point: Point3<f64>,
    /// Strength of attraction (higher = stronger pull).
    strength: f64,
    /// Radius of influence (points outside this radius are unaffected).
    radius: f64,
}

impl Attractor {
    /// Creates a new attractor.
    ///
    /// # Arguments
    ///
    /// * `point` - Center of attraction
    /// * `strength` - How strongly the path is biased (typically 0.1 to 10.0)
    /// * `radius` - Maximum distance at which the attractor has effect
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::Attractor;
    /// use nalgebra::Point3;
    ///
    /// let attractor = Attractor::new(
    ///     Point3::new(5.0, 5.0, 5.0),
    ///     1.0,   // Moderate strength
    ///     5.0,   // 5-unit radius of influence
    /// );
    /// ```
    #[must_use]
    pub const fn new(point: Point3<f64>, strength: f64, radius: f64) -> Self {
        Self {
            point,
            strength,
            radius,
        }
    }

    /// Returns the center point of the attractor.
    #[must_use]
    pub const fn point(&self) -> &Point3<f64> {
        &self.point
    }

    /// Returns the strength of the attractor.
    #[must_use]
    pub const fn strength(&self) -> f64 {
        self.strength
    }

    /// Returns the radius of influence.
    #[must_use]
    pub const fn radius(&self) -> f64 {
        self.radius
    }

    /// Computes the attraction factor for a given point.
    ///
    /// Returns a value in [0, strength] based on distance:
    /// - At the center: returns `strength`
    /// - At the edge (distance = radius): returns 0
    /// - Beyond the radius: returns 0
    ///
    /// Uses linear falloff: `strength * (1 - distance/radius)`
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::Attractor;
    /// use nalgebra::Point3;
    ///
    /// let attractor = Attractor::new(Point3::origin(), 1.0, 10.0);
    ///
    /// // At center: full strength
    /// let at_center = attractor.attraction_at(&Point3::origin());
    /// assert!((at_center - 1.0).abs() < 1e-10);
    ///
    /// // At edge: zero
    /// let at_edge = attractor.attraction_at(&Point3::new(10.0, 0.0, 0.0));
    /// assert!((at_edge - 0.0).abs() < 1e-10);
    ///
    /// // Beyond radius: zero
    /// let beyond = attractor.attraction_at(&Point3::new(20.0, 0.0, 0.0));
    /// assert!((beyond - 0.0).abs() < 1e-10);
    /// ```
    #[must_use]
    pub fn attraction_at(&self, point: &Point3<f64>) -> f64 {
        let distance = (point - self.point).norm();
        if distance >= self.radius {
            return 0.0;
        }
        self.strength * (1.0 - distance / self.radius)
    }
}

impl Default for Attractor {
    fn default() -> Self {
        Self::new(Point3::origin(), 1.0, 1.0)
    }
}

/// Complete goal specification for routing.
///
/// Specifies the start point, end point, optional via points (waypoints),
/// and optional attractors that bias the path.
///
/// # Example
///
/// ```
/// use route_types::{RouteGoal, GoalPoint, Attractor};
/// use cf_spatial::VoxelCoord;
/// use nalgebra::Point3;
///
/// // Simple point-to-point routing
/// let simple_goal = RouteGoal::new(
///     GoalPoint::Voxel(VoxelCoord::new(0, 0, 0)),
///     GoalPoint::Voxel(VoxelCoord::new(10, 10, 10)),
/// );
///
/// // Multi-segment routing with via points
/// let complex_goal = RouteGoal::new(
///     GoalPoint::voxel(0, 0, 0),
///     GoalPoint::voxel(20, 0, 0),
/// )
/// .with_via_point(GoalPoint::voxel(10, 5, 0))
/// .with_attractor(Attractor::new(Point3::new(5.0, 2.5, 0.0), 0.5, 5.0));
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RouteGoal {
    /// Starting point (required).
    start: GoalPoint,
    /// Ending point (required).
    end: GoalPoint,
    /// Intermediate via points (must be visited in order).
    via_points: Vec<GoalPoint>,
    /// Attractor points (soft constraints).
    attractors: Vec<Attractor>,
}

impl RouteGoal {
    /// Creates a new route goal with start and end points.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::{RouteGoal, GoalPoint};
    /// use cf_spatial::VoxelCoord;
    ///
    /// let goal = RouteGoal::new(
    ///     GoalPoint::Voxel(VoxelCoord::new(0, 0, 0)),
    ///     GoalPoint::Voxel(VoxelCoord::new(10, 10, 10)),
    /// );
    /// ```
    #[must_use]
    pub const fn new(start: GoalPoint, end: GoalPoint) -> Self {
        Self {
            start,
            end,
            via_points: Vec::new(),
            attractors: Vec::new(),
        }
    }

    /// Creates a goal from voxel coordinates.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::RouteGoal;
    /// use cf_spatial::VoxelCoord;
    ///
    /// let goal = RouteGoal::from_voxels(
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(10, 10, 10),
    /// );
    /// ```
    #[must_use]
    pub const fn from_voxels(start: VoxelCoord, end: VoxelCoord) -> Self {
        Self::new(GoalPoint::Voxel(start), GoalPoint::Voxel(end))
    }

    /// Creates a goal from world points.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::RouteGoal;
    /// use nalgebra::Point3;
    ///
    /// let goal = RouteGoal::from_points(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(10.0, 10.0, 10.0),
    /// );
    /// ```
    #[must_use]
    pub const fn from_points(start: Point3<f64>, end: Point3<f64>) -> Self {
        Self::new(GoalPoint::World(start), GoalPoint::World(end))
    }

    /// Adds a via point (waypoint that must be visited).
    ///
    /// Via points are visited in the order they are added.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::{RouteGoal, GoalPoint};
    /// use cf_spatial::VoxelCoord;
    ///
    /// let goal = RouteGoal::from_voxels(
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(20, 0, 0),
    /// )
    /// .with_via_point(GoalPoint::voxel(10, 5, 0));  // Go through this point
    ///
    /// assert_eq!(goal.via_points().len(), 1);
    /// ```
    #[must_use]
    pub fn with_via_point(mut self, via: GoalPoint) -> Self {
        self.via_points.push(via);
        self
    }

    /// Sets all via points at once.
    #[must_use]
    pub fn with_via_points(mut self, via_points: Vec<GoalPoint>) -> Self {
        self.via_points = via_points;
        self
    }

    /// Adds an attractor point.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::{RouteGoal, GoalPoint, Attractor};
    /// use cf_spatial::VoxelCoord;
    /// use nalgebra::Point3;
    ///
    /// let goal = RouteGoal::from_voxels(
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(10, 0, 0),
    /// )
    /// .with_attractor(Attractor::new(Point3::new(5.0, 2.0, 0.0), 1.0, 5.0));
    ///
    /// assert_eq!(goal.attractors().len(), 1);
    /// ```
    #[must_use]
    pub fn with_attractor(mut self, attractor: Attractor) -> Self {
        self.attractors.push(attractor);
        self
    }

    /// Sets all attractors at once.
    #[must_use]
    pub fn with_attractors(mut self, attractors: Vec<Attractor>) -> Self {
        self.attractors = attractors;
        self
    }

    /// Returns the start point.
    #[must_use]
    pub const fn start(&self) -> &GoalPoint {
        &self.start
    }

    /// Returns the end point.
    #[must_use]
    pub const fn end(&self) -> &GoalPoint {
        &self.end
    }

    /// Returns the via points.
    #[must_use]
    pub fn via_points(&self) -> &[GoalPoint] {
        &self.via_points
    }

    /// Returns the attractors.
    #[must_use]
    pub fn attractors(&self) -> &[Attractor] {
        &self.attractors
    }

    /// Returns the total number of segments in this goal.
    ///
    /// A goal with no via points has 1 segment (start to end).
    /// Each via point adds one additional segment.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::{RouteGoal, GoalPoint};
    /// use cf_spatial::VoxelCoord;
    ///
    /// let goal = RouteGoal::from_voxels(
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(20, 0, 0),
    /// )
    /// .with_via_point(GoalPoint::voxel(10, 0, 0));
    ///
    /// assert_eq!(goal.num_segments(), 2);
    /// ```
    #[must_use]
    pub fn num_segments(&self) -> usize {
        self.via_points.len() + 1
    }

    /// Returns an iterator over all goal points in order: start, via points, end.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::{RouteGoal, GoalPoint};
    /// use cf_spatial::VoxelCoord;
    ///
    /// let goal = RouteGoal::from_voxels(
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(20, 0, 0),
    /// )
    /// .with_via_point(GoalPoint::voxel(10, 0, 0));
    ///
    /// let all_points: Vec<_> = goal.all_points().collect();
    /// assert_eq!(all_points.len(), 3);
    /// ```
    pub fn all_points(&self) -> impl Iterator<Item = &GoalPoint> {
        std::iter::once(&self.start)
            .chain(self.via_points.iter())
            .chain(std::iter::once(&self.end))
    }

    /// Returns an iterator over segment endpoints: (from, to) pairs.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::{RouteGoal, GoalPoint};
    /// use cf_spatial::VoxelCoord;
    ///
    /// let goal = RouteGoal::from_voxels(
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(20, 0, 0),
    /// )
    /// .with_via_point(GoalPoint::voxel(10, 0, 0));
    ///
    /// let segments: Vec<_> = goal.segments().collect();
    /// assert_eq!(segments.len(), 2);
    /// ```
    pub fn segments(&self) -> impl Iterator<Item = (&GoalPoint, &GoalPoint)> {
        let points: Vec<_> = self.all_points().collect();
        (0..points.len().saturating_sub(1)).map(move |i| (points[i], points[i + 1]))
    }

    /// Computes the total attraction at a point from all attractors.
    #[must_use]
    pub fn total_attraction_at(&self, point: &Point3<f64>) -> f64 {
        self.attractors.iter().map(|a| a.attraction_at(point)).sum()
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::redundant_clone,
    clippy::needless_collect
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::Point3;

    // ==================== GoalPoint Tests ====================

    #[test]
    fn test_goal_point_voxel() {
        let goal = GoalPoint::voxel(5, 10, 15);
        assert!(goal.is_voxel());
        assert!(!goal.is_world());
        assert!(!goal.is_region());
        assert_eq!(goal.as_voxel(), Some(&VoxelCoord::new(5, 10, 15)));
    }

    #[test]
    fn test_goal_point_world() {
        let goal = GoalPoint::world(1.5, 2.5, 3.5);
        assert!(!goal.is_voxel());
        assert!(goal.is_world());
        assert!(!goal.is_region());
        assert_eq!(goal.as_world(), Some(&Point3::new(1.5, 2.5, 3.5)));
    }

    #[test]
    fn test_goal_point_region() {
        let aabb = Aabb::new(Point3::origin(), Point3::new(1.0, 1.0, 1.0));
        let goal = GoalPoint::Region(aabb);
        assert!(!goal.is_voxel());
        assert!(!goal.is_world());
        assert!(goal.is_region());
        assert!(goal.as_region().is_some());
    }

    #[test]
    fn test_goal_point_satisfied_by_voxel_exact() {
        let goal = GoalPoint::Voxel(VoxelCoord::new(5, 5, 5));
        assert!(goal.is_satisfied_by_voxel(VoxelCoord::new(5, 5, 5)));
        assert!(!goal.is_satisfied_by_voxel(VoxelCoord::new(4, 5, 5)));
    }

    #[test]
    fn test_goal_point_satisfied_by_voxel_world() {
        let goal = GoalPoint::World(Point3::new(5.5, 5.5, 5.5));
        // World goals require grid context
        assert!(!goal.is_satisfied_by_voxel(VoxelCoord::new(5, 5, 5)));
    }

    #[test]
    fn test_goal_point_satisfied_by_voxel_region() {
        let goal = GoalPoint::Region(Aabb::new(
            Point3::new(4.0, 4.0, 4.0),
            Point3::new(6.0, 6.0, 6.0),
        ));
        // Voxel (5,5,5) has center at (5.5, 5.5, 5.5) in unit grid
        assert!(goal.is_satisfied_by_voxel(VoxelCoord::new(5, 5, 5)));
        // Voxel (0,0,0) has center at (0.5, 0.5, 0.5)
        assert!(!goal.is_satisfied_by_voxel(VoxelCoord::new(0, 0, 0)));
    }

    #[test]
    fn test_goal_point_from_voxel_coord() {
        let coord = VoxelCoord::new(1, 2, 3);
        let goal: GoalPoint = coord.into();
        assert!(goal.is_voxel());
    }

    #[test]
    fn test_goal_point_from_point3() {
        let point = Point3::new(1.0, 2.0, 3.0);
        let goal: GoalPoint = point.into();
        assert!(goal.is_world());
    }

    #[test]
    fn test_goal_point_from_aabb() {
        let aabb = Aabb::new(Point3::origin(), Point3::new(1.0, 1.0, 1.0));
        let goal: GoalPoint = aabb.into();
        assert!(goal.is_region());
    }

    // ==================== Attractor Tests ====================

    #[test]
    fn test_attractor_new() {
        let attractor = Attractor::new(Point3::new(5.0, 5.0, 5.0), 2.0, 10.0);
        assert_eq!(attractor.point(), &Point3::new(5.0, 5.0, 5.0));
        assert_relative_eq!(attractor.strength(), 2.0, epsilon = 1e-10);
        assert_relative_eq!(attractor.radius(), 10.0, epsilon = 1e-10);
    }

    #[test]
    fn test_attractor_attraction_at_center() {
        let attractor = Attractor::new(Point3::origin(), 1.0, 10.0);
        let attraction = attractor.attraction_at(&Point3::origin());
        assert_relative_eq!(attraction, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_attractor_attraction_at_edge() {
        let attractor = Attractor::new(Point3::origin(), 1.0, 10.0);
        let attraction = attractor.attraction_at(&Point3::new(10.0, 0.0, 0.0));
        assert_relative_eq!(attraction, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_attractor_attraction_beyond_radius() {
        let attractor = Attractor::new(Point3::origin(), 1.0, 10.0);
        let attraction = attractor.attraction_at(&Point3::new(20.0, 0.0, 0.0));
        assert_relative_eq!(attraction, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_attractor_attraction_midway() {
        let attractor = Attractor::new(Point3::origin(), 1.0, 10.0);
        let attraction = attractor.attraction_at(&Point3::new(5.0, 0.0, 0.0));
        // At distance 5 with radius 10: strength * (1 - 5/10) = 1 * 0.5 = 0.5
        assert_relative_eq!(attraction, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_attractor_attraction_with_strength() {
        let attractor = Attractor::new(Point3::origin(), 3.0, 10.0);
        let attraction = attractor.attraction_at(&Point3::new(5.0, 0.0, 0.0));
        // At distance 5 with radius 10: strength * (1 - 5/10) = 3 * 0.5 = 1.5
        assert_relative_eq!(attraction, 1.5, epsilon = 1e-10);
    }

    #[test]
    fn test_attractor_default() {
        let attractor = Attractor::default();
        assert_eq!(attractor.point(), &Point3::origin());
        assert_relative_eq!(attractor.strength(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(attractor.radius(), 1.0, epsilon = 1e-10);
    }

    // ==================== RouteGoal Tests ====================

    #[test]
    fn test_route_goal_new() {
        let goal = RouteGoal::new(GoalPoint::voxel(0, 0, 0), GoalPoint::voxel(10, 10, 10));
        assert!(goal.start().is_voxel());
        assert!(goal.end().is_voxel());
        assert!(goal.via_points().is_empty());
        assert!(goal.attractors().is_empty());
    }

    #[test]
    fn test_route_goal_from_voxels() {
        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 10, 10));
        assert!(goal.start().is_voxel());
        assert!(goal.end().is_voxel());
    }

    #[test]
    fn test_route_goal_from_points() {
        let goal =
            RouteGoal::from_points(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0));
        assert!(goal.start().is_world());
        assert!(goal.end().is_world());
    }

    #[test]
    fn test_route_goal_with_via_point() {
        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(20, 0, 0))
            .with_via_point(GoalPoint::voxel(10, 0, 0));

        assert_eq!(goal.via_points().len(), 1);
        assert_eq!(goal.num_segments(), 2);
    }

    #[test]
    fn test_route_goal_with_via_points() {
        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(30, 0, 0))
            .with_via_points(vec![GoalPoint::voxel(10, 0, 0), GoalPoint::voxel(20, 0, 0)]);

        assert_eq!(goal.via_points().len(), 2);
        assert_eq!(goal.num_segments(), 3);
    }

    #[test]
    fn test_route_goal_with_attractor() {
        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 0, 0))
            .with_attractor(Attractor::new(Point3::new(5.0, 2.0, 0.0), 1.0, 5.0));

        assert_eq!(goal.attractors().len(), 1);
    }

    #[test]
    fn test_route_goal_with_attractors() {
        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 0, 0))
            .with_attractors(vec![
                Attractor::new(Point3::new(2.5, 0.0, 0.0), 1.0, 3.0),
                Attractor::new(Point3::new(7.5, 0.0, 0.0), 1.0, 3.0),
            ]);

        assert_eq!(goal.attractors().len(), 2);
    }

    #[test]
    fn test_route_goal_num_segments() {
        let goal0 = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 0, 0));
        assert_eq!(goal0.num_segments(), 1);

        let goal1 = goal0.clone().with_via_point(GoalPoint::voxel(5, 0, 0));
        assert_eq!(goal1.num_segments(), 2);

        let goal2 = goal1.with_via_point(GoalPoint::voxel(7, 0, 0));
        assert_eq!(goal2.num_segments(), 3);
    }

    #[test]
    fn test_route_goal_all_points() {
        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(20, 0, 0))
            .with_via_point(GoalPoint::voxel(10, 0, 0));

        let all: Vec<_> = goal.all_points().collect();
        assert_eq!(all.len(), 3);
    }

    #[test]
    fn test_route_goal_segments() {
        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(20, 0, 0))
            .with_via_point(GoalPoint::voxel(10, 0, 0));

        let segs: Vec<_> = goal.segments().collect();
        assert_eq!(segs.len(), 2);

        // First segment: start to via
        assert!(segs[0].0.is_satisfied_by_voxel(VoxelCoord::new(0, 0, 0)));
        assert!(segs[0].1.is_satisfied_by_voxel(VoxelCoord::new(10, 0, 0)));

        // Second segment: via to end
        assert!(segs[1].0.is_satisfied_by_voxel(VoxelCoord::new(10, 0, 0)));
        assert!(segs[1].1.is_satisfied_by_voxel(VoxelCoord::new(20, 0, 0)));
    }

    #[test]
    fn test_route_goal_total_attraction() {
        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 0, 0))
            .with_attractor(Attractor::new(Point3::new(5.0, 0.0, 0.0), 1.0, 10.0))
            .with_attractor(Attractor::new(Point3::new(5.0, 0.0, 0.0), 2.0, 10.0));

        // At the attractor center, both contribute full strength
        let attraction = goal.total_attraction_at(&Point3::new(5.0, 0.0, 0.0));
        assert_relative_eq!(attraction, 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_route_goal_total_attraction_no_attractors() {
        let goal = RouteGoal::from_voxels(VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 0, 0));

        let attraction = goal.total_attraction_at(&Point3::new(5.0, 0.0, 0.0));
        assert_relative_eq!(attraction, 0.0, epsilon = 1e-10);
    }
}
