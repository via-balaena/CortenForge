//! Constraint types for routing.
//!
//! This module defines constraints that paths must satisfy, including:
//!
//! - [`PhysicalConstraints`]: Clearance radius, bend radius limits
//! - [`KeepOutZone`]: Regions to avoid
//! - [`RouteConstraints`]: Combined constraint specification
//!
//! # Example
//!
//! ```
//! use route_types::{RouteConstraints, PhysicalConstraints, KeepOutZone};
//! use nalgebra::Point3;
//!
//! let constraints = RouteConstraints::default()
//!     .with_physical(PhysicalConstraints::new(0.05))  // 5cm clearance
//!     .with_max_length(100.0);
//! ```

use cf_spatial::VoxelCoord;
use nalgebra::Point3;

/// An axis-aligned bounding box for constraint specification.
///
/// Represents a rectangular region in 3D world space.
///
/// # Example
///
/// ```
/// use route_types::Aabb;
/// use nalgebra::Point3;
///
/// let aabb = Aabb::new(Point3::origin(), Point3::new(1.0, 1.0, 1.0));
/// assert!(aabb.contains(&Point3::new(0.5, 0.5, 0.5)));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Aabb {
    /// Minimum corner.
    pub min: Point3<f64>,
    /// Maximum corner.
    pub max: Point3<f64>,
}

impl Aabb {
    /// Creates a new AABB from min and max corners.
    ///
    /// The corners are automatically ordered.
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
    /// use route_types::Aabb;
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let aabb = Aabb::from_center_extents(
    ///     Point3::new(5.0, 5.0, 5.0),
    ///     Vector3::new(1.0, 1.0, 1.0),
    /// );
    /// assert!(aabb.contains(&Point3::new(5.0, 5.0, 5.0)));
    /// ```
    #[must_use]
    pub fn from_center_extents(center: Point3<f64>, half_extents: nalgebra::Vector3<f64>) -> Self {
        Self {
            min: center - half_extents,
            max: center + half_extents,
        }
    }

    /// Checks if a point is inside the AABB.
    #[must_use]
    pub fn contains(&self, point: &Point3<f64>) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    /// Returns the center of the AABB.
    #[must_use]
    pub fn center(&self) -> Point3<f64> {
        Point3::new(
            (self.min.x + self.max.x) * 0.5,
            (self.min.y + self.max.y) * 0.5,
            (self.min.z + self.max.z) * 0.5,
        )
    }

    /// Returns the size (extents) of the AABB.
    #[must_use]
    pub fn size(&self) -> nalgebra::Vector3<f64> {
        self.max - self.min
    }

    /// Returns the volume of the AABB.
    #[must_use]
    pub fn volume(&self) -> f64 {
        let s = self.size();
        s.x * s.y * s.z
    }

    /// Expands the AABB by a margin in all directions.
    #[must_use]
    pub fn expanded(&self, margin: f64) -> Self {
        let m = nalgebra::Vector3::new(margin, margin, margin);
        Self {
            min: self.min - m,
            max: self.max + m,
        }
    }
}

impl Default for Aabb {
    fn default() -> Self {
        Self::new(Point3::origin(), Point3::origin())
    }
}

/// A sphere for constraint specification.
///
/// Represents a spherical region in 3D world space.
///
/// # Example
///
/// ```
/// use route_types::Sphere;
/// use nalgebra::Point3;
///
/// let sphere = Sphere::new(Point3::new(5.0, 5.0, 5.0), 1.0);
/// assert!(sphere.contains(&Point3::new(5.5, 5.0, 5.0)));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Sphere {
    /// Center of the sphere.
    pub center: Point3<f64>,
    /// Radius of the sphere.
    pub radius: f64,
}

impl Sphere {
    /// Creates a new sphere with the given center and radius.
    #[must_use]
    pub const fn new(center: Point3<f64>, radius: f64) -> Self {
        Self { center, radius }
    }

    /// Checks if a point is inside the sphere.
    #[must_use]
    pub fn contains(&self, point: &Point3<f64>) -> bool {
        (point - self.center).norm_squared() <= self.radius * self.radius
    }

    /// Returns the distance from a point to the sphere surface.
    ///
    /// Negative if inside, positive if outside.
    #[must_use]
    pub fn signed_distance(&self, point: &Point3<f64>) -> f64 {
        (point - self.center).norm() - self.radius
    }
}

impl Default for Sphere {
    fn default() -> Self {
        Self::new(Point3::origin(), 1.0)
    }
}

/// Physical constraints for routing (e.g., wire/pipe properties).
///
/// These constraints model the physical properties of the routed object,
/// such as wire diameter (requiring clearance) and bend radius limits.
///
/// # Example
///
/// ```
/// use route_types::PhysicalConstraints;
///
/// // Wire with 5mm radius requiring 10mm clearance and 20mm bend radius
/// let constraints = PhysicalConstraints::new(0.01)
///     .with_min_bend_radius(0.02);
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PhysicalConstraints {
    /// Minimum clearance radius from obstacles.
    ///
    /// The path centerline must maintain at least this distance from obstacles.
    /// For a wire with radius r, clearance should be at least r.
    clearance_radius: f64,

    /// Minimum bend radius (for curvature constraints).
    ///
    /// If set, the path curvature must not exceed `1/min_bend_radius`.
    min_bend_radius: Option<f64>,
}

impl PhysicalConstraints {
    /// Creates physical constraints with the given clearance radius.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::PhysicalConstraints;
    ///
    /// let constraints = PhysicalConstraints::new(0.05);
    /// assert!((constraints.clearance_radius() - 0.05).abs() < 1e-10);
    /// ```
    #[must_use]
    pub const fn new(clearance_radius: f64) -> Self {
        Self {
            clearance_radius,
            min_bend_radius: None,
        }
    }

    /// Sets the minimum bend radius.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::PhysicalConstraints;
    ///
    /// let constraints = PhysicalConstraints::new(0.01)
    ///     .with_min_bend_radius(0.05);
    /// assert!(constraints.min_bend_radius().is_some());
    /// ```
    #[must_use]
    pub const fn with_min_bend_radius(mut self, radius: f64) -> Self {
        self.min_bend_radius = Some(radius);
        self
    }

    /// Returns the clearance radius.
    #[must_use]
    pub const fn clearance_radius(&self) -> f64 {
        self.clearance_radius
    }

    /// Returns the minimum bend radius, if set.
    #[must_use]
    pub const fn min_bend_radius(&self) -> Option<f64> {
        self.min_bend_radius
    }

    /// Returns the maximum allowed curvature (`1 / min_bend_radius`).
    ///
    /// Returns `None` if no bend radius constraint is set.
    #[must_use]
    pub fn max_curvature(&self) -> Option<f64> {
        self.min_bend_radius.map(|r| 1.0 / r)
    }
}

impl Default for PhysicalConstraints {
    fn default() -> Self {
        Self::new(0.0)
    }
}

/// A spatial region to avoid during routing.
///
/// Keep-out zones define areas that the path must not enter.
///
/// # Example
///
/// ```
/// use route_types::{KeepOutZone, Aabb, Sphere};
/// use nalgebra::Point3;
///
/// let box_zone = KeepOutZone::Aabb(Aabb::new(
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 1.0, 1.0),
/// ));
///
/// let sphere_zone = KeepOutZone::Sphere(Sphere::new(
///     Point3::new(5.0, 5.0, 5.0),
///     1.0,
/// ));
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum KeepOutZone {
    /// An axis-aligned bounding box.
    Aabb(Aabb),
    /// A sphere.
    Sphere(Sphere),
    /// A set of specific voxel coordinates.
    Voxels(Vec<VoxelCoord>),
}

impl KeepOutZone {
    /// Checks if a point is inside this keep-out zone.
    ///
    /// For voxel zones, this only checks exact matches and should be used
    /// with voxel coordinates converted to points.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::{KeepOutZone, Aabb};
    /// use nalgebra::Point3;
    ///
    /// let zone = KeepOutZone::Aabb(Aabb::new(
    ///     Point3::origin(),
    ///     Point3::new(1.0, 1.0, 1.0),
    /// ));
    ///
    /// assert!(zone.contains_point(&Point3::new(0.5, 0.5, 0.5)));
    /// assert!(!zone.contains_point(&Point3::new(2.0, 0.5, 0.5)));
    /// ```
    #[must_use]
    pub fn contains_point(&self, point: &Point3<f64>) -> bool {
        match self {
            Self::Aabb(aabb) => aabb.contains(point),
            Self::Sphere(sphere) => sphere.contains(point),
            Self::Voxels(_) => false, // Voxel check requires grid context
        }
    }

    /// Checks if a voxel coordinate is in this keep-out zone.
    ///
    /// Only meaningful for voxel zones; returns `false` for other types.
    #[must_use]
    pub fn contains_voxel(&self, coord: VoxelCoord) -> bool {
        match self {
            Self::Voxels(voxels) => voxels.contains(&coord),
            _ => false,
        }
    }
}

/// Combined routing constraints.
///
/// Aggregates all constraint types into a single specification.
///
/// # Example
///
/// ```
/// use route_types::{RouteConstraints, PhysicalConstraints, KeepOutZone, Aabb};
/// use nalgebra::Point3;
///
/// let constraints = RouteConstraints::default()
///     .with_physical(PhysicalConstraints::new(0.05))
///     .with_keep_out_zone(KeepOutZone::Aabb(Aabb::new(
///         Point3::new(1.0, 1.0, 1.0),
///         Point3::new(2.0, 2.0, 2.0),
///     )))
///     .with_max_length(50.0);
/// ```
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RouteConstraints {
    /// Physical constraints (clearance, bend radius).
    physical: Option<PhysicalConstraints>,

    /// Keep-out zones to avoid.
    keep_out_zones: Vec<KeepOutZone>,

    /// Preferred corridors (soft constraint, path is biased toward these).
    preferred_corridors: Vec<Aabb>,

    /// Maximum allowed path length.
    max_length: Option<f64>,

    /// Maximum number of direction changes (bends).
    max_bends: Option<usize>,
}

impl RouteConstraints {
    /// Creates empty constraints (no restrictions).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            physical: None,
            keep_out_zones: Vec::new(),
            preferred_corridors: Vec::new(),
            max_length: None,
            max_bends: None,
        }
    }

    /// Sets the physical constraints.
    #[must_use]
    pub const fn with_physical(mut self, physical: PhysicalConstraints) -> Self {
        self.physical = Some(physical);
        self
    }

    /// Adds a keep-out zone.
    #[must_use]
    pub fn with_keep_out_zone(mut self, zone: KeepOutZone) -> Self {
        self.keep_out_zones.push(zone);
        self
    }

    /// Sets the keep-out zones.
    #[must_use]
    pub fn with_keep_out_zones(mut self, zones: Vec<KeepOutZone>) -> Self {
        self.keep_out_zones = zones;
        self
    }

    /// Adds a preferred corridor.
    #[must_use]
    pub fn with_preferred_corridor(mut self, corridor: Aabb) -> Self {
        self.preferred_corridors.push(corridor);
        self
    }

    /// Sets the maximum path length.
    #[must_use]
    pub const fn with_max_length(mut self, length: f64) -> Self {
        self.max_length = Some(length);
        self
    }

    /// Sets the maximum number of bends.
    #[must_use]
    pub const fn with_max_bends(mut self, bends: usize) -> Self {
        self.max_bends = Some(bends);
        self
    }

    /// Returns the physical constraints, if set.
    #[must_use]
    pub const fn physical(&self) -> Option<&PhysicalConstraints> {
        self.physical.as_ref()
    }

    /// Returns the keep-out zones.
    #[must_use]
    pub fn keep_out_zones(&self) -> &[KeepOutZone] {
        &self.keep_out_zones
    }

    /// Returns the preferred corridors.
    #[must_use]
    pub fn preferred_corridors(&self) -> &[Aabb] {
        &self.preferred_corridors
    }

    /// Returns the maximum path length, if set.
    #[must_use]
    pub const fn max_length(&self) -> Option<f64> {
        self.max_length
    }

    /// Returns the maximum number of bends, if set.
    #[must_use]
    pub const fn max_bends(&self) -> Option<usize> {
        self.max_bends
    }

    /// Returns the clearance radius from physical constraints, or 0.0 if none.
    #[must_use]
    pub fn clearance_radius(&self) -> f64 {
        self.physical
            .as_ref()
            .map_or(0.0, PhysicalConstraints::clearance_radius)
    }

    /// Checks if a point is inside any keep-out zone.
    #[must_use]
    pub fn is_in_keep_out(&self, point: &Point3<f64>) -> bool {
        self.keep_out_zones.iter().any(|z| z.contains_point(point))
    }

    /// Checks if a point is inside any preferred corridor.
    #[must_use]
    pub fn is_in_corridor(&self, point: &Point3<f64>) -> bool {
        if self.preferred_corridors.is_empty() {
            return true; // No corridors = everywhere is "in corridor"
        }
        self.preferred_corridors.iter().any(|c| c.contains(point))
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::Vector3;

    // ==================== Aabb Tests ====================

    #[test]
    fn test_aabb_new() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        assert_eq!(aabb.min, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(aabb.max, Point3::new(1.0, 1.0, 1.0));
    }

    #[test]
    fn test_aabb_new_auto_order() {
        let aabb = Aabb::new(Point3::new(1.0, 1.0, 1.0), Point3::new(0.0, 0.0, 0.0));
        assert_eq!(aabb.min, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(aabb.max, Point3::new(1.0, 1.0, 1.0));
    }

    #[test]
    fn test_aabb_from_center_extents() {
        let aabb =
            Aabb::from_center_extents(Point3::new(5.0, 5.0, 5.0), Vector3::new(1.0, 1.0, 1.0));
        assert_eq!(aabb.min, Point3::new(4.0, 4.0, 4.0));
        assert_eq!(aabb.max, Point3::new(6.0, 6.0, 6.0));
    }

    #[test]
    fn test_aabb_contains() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        assert!(aabb.contains(&Point3::new(0.5, 0.5, 0.5)));
        assert!(aabb.contains(&Point3::new(0.0, 0.0, 0.0)));
        assert!(aabb.contains(&Point3::new(1.0, 1.0, 1.0)));
        assert!(!aabb.contains(&Point3::new(1.5, 0.5, 0.5)));
    }

    #[test]
    fn test_aabb_center() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 4.0, 6.0));
        let center = aabb.center();
        assert_relative_eq!(center.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(center.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(center.z, 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_aabb_size() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 3.0, 4.0));
        let size = aabb.size();
        assert_relative_eq!(size.x, 2.0, epsilon = 1e-10);
        assert_relative_eq!(size.y, 3.0, epsilon = 1e-10);
        assert_relative_eq!(size.z, 4.0, epsilon = 1e-10);
    }

    #[test]
    fn test_aabb_volume() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 3.0, 4.0));
        assert_relative_eq!(aabb.volume(), 24.0, epsilon = 1e-10);
    }

    #[test]
    fn test_aabb_expanded() {
        let aabb = Aabb::new(Point3::new(1.0, 1.0, 1.0), Point3::new(2.0, 2.0, 2.0));
        let expanded = aabb.expanded(0.5);
        assert_eq!(expanded.min, Point3::new(0.5, 0.5, 0.5));
        assert_eq!(expanded.max, Point3::new(2.5, 2.5, 2.5));
    }

    // ==================== Sphere Tests ====================

    #[test]
    fn test_sphere_new() {
        let sphere = Sphere::new(Point3::new(1.0, 2.0, 3.0), 0.5);
        assert_eq!(sphere.center, Point3::new(1.0, 2.0, 3.0));
        assert_relative_eq!(sphere.radius, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_sphere_contains() {
        let sphere = Sphere::new(Point3::new(0.0, 0.0, 0.0), 1.0);
        assert!(sphere.contains(&Point3::new(0.0, 0.0, 0.0)));
        assert!(sphere.contains(&Point3::new(0.5, 0.0, 0.0)));
        assert!(sphere.contains(&Point3::new(1.0, 0.0, 0.0)));
        assert!(!sphere.contains(&Point3::new(1.5, 0.0, 0.0)));
    }

    #[test]
    fn test_sphere_signed_distance() {
        let sphere = Sphere::new(Point3::new(0.0, 0.0, 0.0), 1.0);

        // Inside
        let d_inside = sphere.signed_distance(&Point3::new(0.5, 0.0, 0.0));
        assert!(d_inside < 0.0);
        assert_relative_eq!(d_inside, -0.5, epsilon = 1e-10);

        // On surface
        let d_surface = sphere.signed_distance(&Point3::new(1.0, 0.0, 0.0));
        assert_relative_eq!(d_surface, 0.0, epsilon = 1e-10);

        // Outside
        let d_outside = sphere.signed_distance(&Point3::new(2.0, 0.0, 0.0));
        assert!(d_outside > 0.0);
        assert_relative_eq!(d_outside, 1.0, epsilon = 1e-10);
    }

    // ==================== PhysicalConstraints Tests ====================

    #[test]
    fn test_physical_constraints_new() {
        let pc = PhysicalConstraints::new(0.05);
        assert_relative_eq!(pc.clearance_radius(), 0.05, epsilon = 1e-10);
        assert!(pc.min_bend_radius().is_none());
    }

    #[test]
    fn test_physical_constraints_with_bend_radius() {
        let pc = PhysicalConstraints::new(0.01).with_min_bend_radius(0.1);
        assert!(pc.min_bend_radius().is_some());
        assert_relative_eq!(pc.min_bend_radius().unwrap(), 0.1, epsilon = 1e-10);
    }

    #[test]
    fn test_physical_constraints_max_curvature() {
        let pc = PhysicalConstraints::new(0.01).with_min_bend_radius(0.5);
        let max_curv = pc.max_curvature();
        assert!(max_curv.is_some());
        assert_relative_eq!(max_curv.unwrap(), 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_physical_constraints_max_curvature_none() {
        let pc = PhysicalConstraints::new(0.01);
        assert!(pc.max_curvature().is_none());
    }

    // ==================== KeepOutZone Tests ====================

    #[test]
    fn test_keep_out_zone_aabb() {
        let zone = KeepOutZone::Aabb(Aabb::new(Point3::origin(), Point3::new(1.0, 1.0, 1.0)));
        assert!(zone.contains_point(&Point3::new(0.5, 0.5, 0.5)));
        assert!(!zone.contains_point(&Point3::new(2.0, 0.5, 0.5)));
    }

    #[test]
    fn test_keep_out_zone_sphere() {
        let zone = KeepOutZone::Sphere(Sphere::new(Point3::new(5.0, 5.0, 5.0), 1.0));
        assert!(zone.contains_point(&Point3::new(5.5, 5.0, 5.0)));
        assert!(!zone.contains_point(&Point3::new(7.0, 5.0, 5.0)));
    }

    #[test]
    fn test_keep_out_zone_voxels() {
        let zone = KeepOutZone::Voxels(vec![VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 0, 0)]);
        assert!(zone.contains_voxel(VoxelCoord::new(0, 0, 0)));
        assert!(zone.contains_voxel(VoxelCoord::new(1, 0, 0)));
        assert!(!zone.contains_voxel(VoxelCoord::new(2, 0, 0)));
    }

    #[test]
    fn test_keep_out_zone_voxels_point_check() {
        let zone = KeepOutZone::Voxels(vec![VoxelCoord::new(0, 0, 0)]);
        // Voxel zones don't support point containment directly
        assert!(!zone.contains_point(&Point3::new(0.0, 0.0, 0.0)));
    }

    // ==================== RouteConstraints Tests ====================

    #[test]
    fn test_route_constraints_default() {
        let rc = RouteConstraints::default();
        assert!(rc.physical().is_none());
        assert!(rc.keep_out_zones().is_empty());
        assert!(rc.preferred_corridors().is_empty());
        assert!(rc.max_length().is_none());
        assert!(rc.max_bends().is_none());
    }

    #[test]
    fn test_route_constraints_with_physical() {
        let rc = RouteConstraints::new().with_physical(PhysicalConstraints::new(0.05));
        assert!(rc.physical().is_some());
        assert_relative_eq!(rc.clearance_radius(), 0.05, epsilon = 1e-10);
    }

    #[test]
    fn test_route_constraints_with_keep_out() {
        let rc = RouteConstraints::new().with_keep_out_zone(KeepOutZone::Aabb(Aabb::new(
            Point3::origin(),
            Point3::new(1.0, 1.0, 1.0),
        )));
        assert_eq!(rc.keep_out_zones().len(), 1);
    }

    #[test]
    fn test_route_constraints_with_corridor() {
        let rc = RouteConstraints::new()
            .with_preferred_corridor(Aabb::new(Point3::origin(), Point3::new(10.0, 10.0, 10.0)));
        assert_eq!(rc.preferred_corridors().len(), 1);
    }

    #[test]
    fn test_route_constraints_with_max_length() {
        let rc = RouteConstraints::new().with_max_length(100.0);
        assert_eq!(rc.max_length(), Some(100.0));
    }

    #[test]
    fn test_route_constraints_with_max_bends() {
        let rc = RouteConstraints::new().with_max_bends(10);
        assert_eq!(rc.max_bends(), Some(10));
    }

    #[test]
    fn test_route_constraints_is_in_keep_out() {
        let rc = RouteConstraints::new().with_keep_out_zone(KeepOutZone::Aabb(Aabb::new(
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(2.0, 2.0, 2.0),
        )));

        assert!(rc.is_in_keep_out(&Point3::new(1.5, 1.5, 1.5)));
        assert!(!rc.is_in_keep_out(&Point3::new(0.5, 0.5, 0.5)));
    }

    #[test]
    fn test_route_constraints_is_in_corridor() {
        let rc = RouteConstraints::new().with_preferred_corridor(Aabb::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5.0, 5.0, 5.0),
        ));

        assert!(rc.is_in_corridor(&Point3::new(2.5, 2.5, 2.5)));
        assert!(!rc.is_in_corridor(&Point3::new(10.0, 10.0, 10.0)));
    }

    #[test]
    fn test_route_constraints_is_in_corridor_no_corridors() {
        let rc = RouteConstraints::new();
        // No corridors means everywhere is "in corridor"
        assert!(rc.is_in_corridor(&Point3::new(100.0, 100.0, 100.0)));
    }

    #[test]
    fn test_route_constraints_clearance_radius_default() {
        let rc = RouteConstraints::new();
        assert_relative_eq!(rc.clearance_radius(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_route_constraints_builder_chain() {
        let rc = RouteConstraints::new()
            .with_physical(PhysicalConstraints::new(0.05))
            .with_keep_out_zone(KeepOutZone::Sphere(Sphere::new(Point3::origin(), 1.0)))
            .with_preferred_corridor(Aabb::new(Point3::origin(), Point3::new(10.0, 10.0, 10.0)))
            .with_max_length(100.0)
            .with_max_bends(20);

        assert!(rc.physical().is_some());
        assert_eq!(rc.keep_out_zones().len(), 1);
        assert_eq!(rc.preferred_corridors().len(), 1);
        assert_eq!(rc.max_length(), Some(100.0));
        assert_eq!(rc.max_bends(), Some(20));
    }
}
