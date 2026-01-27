//! Collision shape primitives for low-level collision detection.
//!
//! This module defines the `CollisionShape` enum used by GJK/EPA algorithms
//! and raycasting. For the Model/Data architecture, geometry is stored in
//! separate arrays (`geom_type`, `geom_size`, `geom_pos`, etc.).
//!
//! # Shape Types
//!
//! All 10 MuJoCo-compatible geometry types are supported:
//!
//! | Type | Description | Convex? |
//! |------|-------------|---------|
//! | [`Sphere`](CollisionShape::Sphere) | Ball with radius | Yes |
//! | [`Box`](CollisionShape::Box) | Axis-aligned cuboid | Yes |
//! | [`Capsule`](CollisionShape::Capsule) | Cylinder with hemispherical caps | Yes |
//! | [`Cylinder`](CollisionShape::Cylinder) | Cylinder without caps | Yes |
//! | [`Ellipsoid`](CollisionShape::Ellipsoid) | Scaled sphere | Yes |
//! | [`Plane`](CollisionShape::Plane) | Infinite half-space | No |
//! | [`ConvexMesh`](CollisionShape::ConvexMesh) | Convex hull of vertices | Yes |
//! | [`TriangleMesh`](CollisionShape::TriangleMesh) | Non-convex triangle soup | No |
//! | [`HeightField`](CollisionShape::HeightField) | Terrain grid | No |
//! | [`Sdf`](CollisionShape::Sdf) | Signed distance field | No |

use nalgebra::{Point3, Vector3};
use std::sync::Arc;

use crate::heightfield::HeightFieldData;
use crate::mesh::TriangleMeshData;
use crate::sdf::SdfCollisionData;
use sim_types::Pose;

/// Collision shape for a body.
///
/// This represents the geometry used for collision detection and response.
/// Each shape type has different performance characteristics and use cases.
#[derive(Debug, Clone)]
pub enum CollisionShape {
    /// Sphere with given radius.
    Sphere {
        /// Sphere radius in meters. Must be positive.
        radius: f64,
    },
    /// Infinite plane with normal and distance from origin.
    /// The plane equation is: normal · x = distance
    Plane {
        /// Unit normal vector of the plane.
        normal: Vector3<f64>,
        /// Distance from origin along the normal.
        distance: f64,
    },
    /// Axis-aligned box with half-extents.
    Box {
        /// Half-extents of the box in each axis. All components must be positive.
        half_extents: Vector3<f64>,
    },
    /// Capsule (cylinder with hemispherical caps).
    ///
    /// Defined by half-length along the local Z-axis and radius.
    Capsule {
        /// Half-length of the cylindrical portion along the Z-axis. Must be non-negative.
        half_length: f64,
        /// Radius of the capsule. Must be positive.
        radius: f64,
    },
    /// Convex mesh defined by a set of vertices.
    ConvexMesh {
        /// Vertices of the convex hull in local coordinates. Must have at least 4 vertices.
        vertices: Vec<Point3<f64>>,
    },
    /// Cylinder (without hemispherical caps).
    Cylinder {
        /// Half-length of the cylinder along the Z-axis. Must be positive.
        half_length: f64,
        /// Radius of the cylinder. Must be positive.
        radius: f64,
    },
    /// Ellipsoid (scaled sphere).
    Ellipsoid {
        /// Radii along each local axis (X, Y, Z). All components must be positive.
        radii: Vector3<f64>,
    },
    /// Height field for terrain collision.
    HeightField {
        /// The height field data.
        data: Arc<HeightFieldData>,
    },
    /// Signed Distance Field for collision with complex geometry.
    Sdf {
        /// The SDF collision data.
        data: Arc<SdfCollisionData>,
    },
    /// Non-convex triangle mesh.
    TriangleMesh {
        /// The triangle mesh data.
        data: Arc<TriangleMeshData>,
    },
}

impl CollisionShape {
    // ========================================================================
    // Constructors
    // ========================================================================

    /// Create a sphere shape.
    ///
    /// # Panics
    ///
    /// Debug builds panic if `radius` is not positive or not finite.
    #[must_use]
    pub fn sphere(radius: f64) -> Self {
        debug_assert!(radius > 0.0, "Sphere radius must be positive, got {radius}");
        debug_assert!(radius.is_finite(), "Sphere radius must be finite");
        Self::Sphere { radius }
    }

    /// Create a plane shape with a given normal and distance from origin.
    ///
    /// The plane divides space into two half-spaces. Points where `normal · x > distance`
    /// are considered "outside" the plane.
    ///
    /// # Panics
    ///
    /// Debug builds panic if `normal` is zero or `distance` is not finite.
    #[must_use]
    pub fn plane(normal: Vector3<f64>, distance: f64) -> Self {
        debug_assert!(normal.norm() > 1e-10, "Plane normal must be non-zero");
        debug_assert!(distance.is_finite(), "Plane distance must be finite");
        Self::Plane { normal, distance }
    }

    /// Create a ground plane (Z-up) at the specified height.
    ///
    /// This is a convenience method for creating a horizontal plane.
    #[must_use]
    pub fn ground_plane(height: f64) -> Self {
        debug_assert!(height.is_finite(), "Ground plane height must be finite");
        Self::Plane {
            normal: Vector3::new(0.0, 0.0, 1.0),
            distance: height,
        }
    }

    /// Create a box shape.
    ///
    /// # Panics
    ///
    /// Debug builds panic if any half-extent is not positive or not finite.
    #[must_use]
    pub fn box_shape(half_extents: Vector3<f64>) -> Self {
        debug_assert!(
            half_extents.x > 0.0 && half_extents.y > 0.0 && half_extents.z > 0.0,
            "Box half-extents must all be positive, got {half_extents:?}"
        );
        debug_assert!(
            half_extents.x.is_finite() && half_extents.y.is_finite() && half_extents.z.is_finite(),
            "Box half-extents must be finite"
        );
        Self::Box { half_extents }
    }

    /// Create a capsule shape (cylinder with hemispherical caps).
    ///
    /// The capsule is aligned along the local Z-axis.
    ///
    /// # Panics
    ///
    /// Debug builds panic if `half_length` is negative or `radius` is not positive.
    #[must_use]
    pub fn capsule(half_length: f64, radius: f64) -> Self {
        debug_assert!(
            half_length >= 0.0,
            "Capsule half_length must be non-negative, got {half_length}"
        );
        debug_assert!(
            radius > 0.0,
            "Capsule radius must be positive, got {radius}"
        );
        debug_assert!(
            half_length.is_finite() && radius.is_finite(),
            "Capsule dimensions must be finite"
        );
        Self::Capsule {
            half_length,
            radius,
        }
    }

    /// Create a cylinder shape (without hemispherical caps).
    ///
    /// The cylinder is aligned along the local Z-axis.
    ///
    /// # Panics
    ///
    /// Debug builds panic if `half_length` or `radius` is not positive.
    #[must_use]
    pub fn cylinder(half_length: f64, radius: f64) -> Self {
        debug_assert!(
            half_length > 0.0,
            "Cylinder half_length must be positive, got {half_length}"
        );
        debug_assert!(
            radius > 0.0,
            "Cylinder radius must be positive, got {radius}"
        );
        debug_assert!(
            half_length.is_finite() && radius.is_finite(),
            "Cylinder dimensions must be finite"
        );
        Self::Cylinder {
            half_length,
            radius,
        }
    }

    /// Create an ellipsoid shape.
    ///
    /// # Panics
    ///
    /// Debug builds panic if any radius is not positive.
    #[must_use]
    pub fn ellipsoid(radii: Vector3<f64>) -> Self {
        debug_assert!(
            radii.x > 0.0 && radii.y > 0.0 && radii.z > 0.0,
            "Ellipsoid radii must all be positive, got {radii:?}"
        );
        debug_assert!(
            radii.x.is_finite() && radii.y.is_finite() && radii.z.is_finite(),
            "Ellipsoid radii must be finite"
        );
        Self::Ellipsoid { radii }
    }

    /// Create an ellipsoid shape with separate radii for each axis.
    ///
    /// # Panics
    ///
    /// Debug builds panic if any radius is not positive.
    #[must_use]
    pub fn ellipsoid_xyz(rx: f64, ry: f64, rz: f64) -> Self {
        Self::ellipsoid(Vector3::new(rx, ry, rz))
    }

    /// Create a convex mesh shape from vertices.
    ///
    /// The vertices define a convex hull. For collision detection, the actual
    /// convex hull is computed from these vertices.
    ///
    /// # Panics
    ///
    /// Debug builds panic if fewer than 4 vertices are provided.
    #[must_use]
    pub fn convex_mesh(vertices: Vec<Point3<f64>>) -> Self {
        debug_assert!(
            vertices.len() >= 4,
            "ConvexMesh requires at least 4 vertices, got {}",
            vertices.len()
        );
        Self::ConvexMesh { vertices }
    }

    /// Create a regular tetrahedron centered at origin.
    ///
    /// The tetrahedron has vertices at alternating corners of a cube with
    /// half-edge length `edge_half_length`. All vertices are at distance
    /// `sqrt(3) * edge_half_length` from the origin.
    ///
    /// # Panics
    ///
    /// Debug builds panic if `edge_half_length` is not positive.
    #[must_use]
    pub fn tetrahedron(edge_half_length: f64) -> Self {
        debug_assert!(
            edge_half_length > 0.0,
            "Tetrahedron edge_half_length must be positive"
        );
        let h = edge_half_length;
        let vertices = vec![
            Point3::new(h, h, h),
            Point3::new(h, -h, -h),
            Point3::new(-h, h, -h),
            Point3::new(-h, -h, h),
        ];
        Self::ConvexMesh { vertices }
    }

    /// Create a height field shape.
    #[must_use]
    pub fn height_field(data: Arc<HeightFieldData>) -> Self {
        Self::HeightField { data }
    }

    /// Create an SDF shape.
    #[must_use]
    pub fn sdf(data: Arc<SdfCollisionData>) -> Self {
        Self::Sdf { data }
    }

    /// Create a triangle mesh shape.
    #[must_use]
    pub fn triangle_mesh(data: Arc<TriangleMeshData>) -> Self {
        Self::TriangleMesh { data }
    }

    /// Create a triangle mesh shape from vertices and indices.
    ///
    /// This is a convenience method that builds the `TriangleMeshData` internally.
    #[must_use]
    pub fn triangle_mesh_from_vertices(vertices: Vec<Point3<f64>>, indices: Vec<usize>) -> Self {
        let data = Arc::new(TriangleMeshData::new(vertices, indices));
        Self::TriangleMesh { data }
    }

    // ========================================================================
    // Type Predicates
    // ========================================================================

    /// Returns `true` if this shape is convex.
    ///
    /// Convex shapes can use faster collision algorithms (GJK/EPA).
    /// Non-convex shapes require decomposition or special handling.
    #[must_use]
    pub fn is_convex(&self) -> bool {
        matches!(
            self,
            Self::Sphere { .. }
                | Self::Box { .. }
                | Self::Capsule { .. }
                | Self::Cylinder { .. }
                | Self::Ellipsoid { .. }
                | Self::ConvexMesh { .. }
        )
    }

    /// Returns `true` if this shape is a simple primitive (not mesh-based).
    ///
    /// Primitives have analytical collision functions and are faster to process.
    #[must_use]
    pub fn is_primitive(&self) -> bool {
        matches!(
            self,
            Self::Sphere { .. }
                | Self::Box { .. }
                | Self::Capsule { .. }
                | Self::Cylinder { .. }
                | Self::Ellipsoid { .. }
                | Self::Plane { .. }
        )
    }

    /// Returns `true` if this shape is infinite (plane).
    #[must_use]
    pub fn is_infinite(&self) -> bool {
        matches!(self, Self::Plane { .. })
    }

    /// Returns `true` if this is a sphere shape.
    #[must_use]
    pub fn is_sphere(&self) -> bool {
        matches!(self, Self::Sphere { .. })
    }

    /// Returns `true` if this is a box shape.
    #[must_use]
    pub fn is_box(&self) -> bool {
        matches!(self, Self::Box { .. })
    }

    /// Returns `true` if this is a capsule shape.
    #[must_use]
    pub fn is_capsule(&self) -> bool {
        matches!(self, Self::Capsule { .. })
    }

    /// Returns `true` if this is a cylinder shape.
    #[must_use]
    pub fn is_cylinder(&self) -> bool {
        matches!(self, Self::Cylinder { .. })
    }

    /// Returns `true` if this is a plane shape.
    #[must_use]
    pub fn is_plane(&self) -> bool {
        matches!(self, Self::Plane { .. })
    }

    /// Returns `true` if this is a mesh-based shape (convex, triangle, or SDF).
    #[must_use]
    pub fn is_mesh_based(&self) -> bool {
        matches!(
            self,
            Self::ConvexMesh { .. } | Self::TriangleMesh { .. } | Self::Sdf { .. }
        )
    }

    // ========================================================================
    // Accessors
    // ========================================================================

    /// Get the radius if this is a sphere, capsule, or cylinder.
    #[must_use]
    pub fn radius(&self) -> Option<f64> {
        match self {
            Self::Sphere { radius }
            | Self::Capsule { radius, .. }
            | Self::Cylinder { radius, .. } => Some(*radius),
            _ => None,
        }
    }

    /// Get the half-length if this is a capsule or cylinder.
    #[must_use]
    pub fn half_length(&self) -> Option<f64> {
        match self {
            Self::Capsule { half_length, .. } | Self::Cylinder { half_length, .. } => {
                Some(*half_length)
            }
            _ => None,
        }
    }

    /// Get the half-extents if this is a box.
    #[must_use]
    pub fn half_extents(&self) -> Option<Vector3<f64>> {
        match self {
            Self::Box { half_extents } => Some(*half_extents),
            _ => None,
        }
    }

    /// Get the world-space endpoints of a capsule shape.
    ///
    /// Returns `None` if this shape is not a capsule.
    /// The capsule is defined along the local Z-axis, so the endpoints
    /// are at (0, 0, `-half_length`) and (0, 0, `+half_length`) in local space.
    #[must_use]
    pub fn capsule_endpoints(&self, pose: &Pose) -> Option<(Point3<f64>, Point3<f64>)> {
        match self {
            Self::Capsule { half_length, .. } => {
                let local_start = Point3::new(0.0, 0.0, -*half_length);
                let local_end = Point3::new(0.0, 0.0, *half_length);
                Some((
                    pose.transform_point(&local_start),
                    pose.transform_point(&local_end),
                ))
            }
            _ => None,
        }
    }

    /// Compute the bounding radius of this shape from its local origin.
    ///
    /// This is the maximum distance from the shape's center to any point
    /// on its surface in local coordinates. Useful for broad-phase culling.
    #[must_use]
    #[allow(clippy::cast_precision_loss)] // Grid dimensions won't exceed mantissa precision
    pub fn bounding_radius(&self) -> f64 {
        match self {
            Self::Sphere { radius } => *radius,
            Self::Plane { .. } => f64::INFINITY,
            Self::Box { half_extents } => half_extents.norm(),
            Self::Capsule {
                half_length,
                radius,
            } => half_length + radius,
            Self::Cylinder {
                half_length,
                radius,
            } => half_length.hypot(*radius),
            Self::Ellipsoid { radii } => radii.x.max(radii.y).max(radii.z),
            Self::ConvexMesh { vertices } => {
                vertices.iter().map(|v| v.coords.norm()).fold(0.0, f64::max)
            }
            Self::HeightField { data } => {
                let half_width = (data.width() as f64) * data.cell_size() * 0.5;
                let half_depth = (data.depth() as f64) * data.cell_size() * 0.5;
                let max_height = data.max_height().abs().max(data.min_height().abs());
                (half_width.powi(2) + half_depth.powi(2) + max_height.powi(2)).sqrt()
            }
            Self::Sdf { data } => {
                // Bounding radius from origin to furthest corner of the SDF grid
                let origin = data.origin();
                let max_corner = Point3::new(
                    origin.x + data.width() as f64 * data.cell_size(),
                    origin.y + data.height() as f64 * data.cell_size(),
                    origin.z + data.depth() as f64 * data.cell_size(),
                );
                origin.coords.norm().max(max_corner.coords.norm())
            }
            Self::TriangleMesh { data } => data
                .vertices()
                .iter()
                .map(|v| v.coords.norm())
                .fold(0.0, f64::max),
        }
    }
}

// ============================================================================
// Axis-Aligned Bounding Box
// ============================================================================

/// Axis-aligned bounding box (AABB).
///
/// Used for broad-phase collision detection. Two AABBs overlap if and only if
/// they overlap on all three axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    /// Minimum corner of the bounding box.
    pub min: Point3<f64>,
    /// Maximum corner of the bounding box.
    pub max: Point3<f64>,
}

impl Aabb {
    /// Create a new AABB from minimum and maximum corners.
    #[must_use]
    pub const fn new(min: Point3<f64>, max: Point3<f64>) -> Self {
        Self { min, max }
    }

    /// Create an AABB centered at a point with the given half-extents.
    #[must_use]
    pub fn from_center(center: Point3<f64>, half_extents: Vector3<f64>) -> Self {
        Self {
            min: center - half_extents,
            max: center + half_extents,
        }
    }

    /// Create an AABB from two points, normalizing min/max ordering.
    ///
    /// This constructor handles the case where min and max corners might be
    /// swapped (e.g., from user input or floating-point edge cases).
    /// The resulting AABB is guaranteed to have `min <= max` on all axes.
    #[must_use]
    pub fn from_points_normalized(p1: Point3<f64>, p2: Point3<f64>) -> Self {
        Self {
            min: Point3::new(p1.x.min(p2.x), p1.y.min(p2.y), p1.z.min(p2.z)),
            max: Point3::new(p1.x.max(p2.x), p1.y.max(p2.y), p1.z.max(p2.z)),
        }
    }

    /// Check if this AABB is valid (min <= max on all axes, no NaN/Inf).
    ///
    /// An invalid AABB can result from:
    /// - Swapped min/max corners
    /// - NaN or infinite values from numerical errors
    /// - Uninitialized or default values used incorrectly
    #[must_use]
    pub fn is_valid(&self) -> bool {
        // Check for NaN or Inf
        if !self.min.x.is_finite()
            || !self.min.y.is_finite()
            || !self.min.z.is_finite()
            || !self.max.x.is_finite()
            || !self.max.y.is_finite()
            || !self.max.z.is_finite()
        {
            return false;
        }
        // Check min <= max
        self.min.x <= self.max.x && self.min.y <= self.max.y && self.min.z <= self.max.z
    }

    /// Check if this AABB overlaps with another AABB.
    ///
    /// Returns `true` if the boxes share any interior or boundary points.
    #[must_use]
    #[inline]
    pub fn overlaps(&self, other: &Self) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }

    /// Check if this AABB contains a point.
    #[must_use]
    #[inline]
    pub fn contains_point(&self, point: Point3<f64>) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    /// Expand this AABB by a margin on all sides.
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

    /// Merge this AABB with another, returning the smallest AABB containing both.
    #[must_use]
    pub fn merged(&self, other: &Self) -> Self {
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

    /// Get the center point of this AABB.
    #[must_use]
    pub fn center(&self) -> Point3<f64> {
        Point3::new(
            (self.min.x + self.max.x) * 0.5,
            (self.min.y + self.max.y) * 0.5,
            (self.min.z + self.max.z) * 0.5,
        )
    }

    /// Get the half-extents (half-size) of this AABB.
    #[must_use]
    pub fn half_extents(&self) -> Vector3<f64> {
        Vector3::new(
            (self.max.x - self.min.x) * 0.5,
            (self.max.y - self.min.y) * 0.5,
            (self.max.z - self.min.z) * 0.5,
        )
    }

    /// Get the volume of this AABB.
    #[must_use]
    pub fn volume(&self) -> f64 {
        let size = self.max - self.min;
        size.x * size.y * size.z
    }

    /// Get the surface area of this AABB.
    #[must_use]
    pub fn surface_area(&self) -> f64 {
        let size = self.max - self.min;
        2.0 * (size.x * size.y + size.y * size.z + size.z * size.x)
    }

    /// Get the extent (size) along a specific axis.
    #[must_use]
    pub fn extent(&self, axis: Axis) -> f64 {
        match axis {
            Axis::X => self.max.x - self.min.x,
            Axis::Y => self.max.y - self.min.y,
            Axis::Z => self.max.z - self.min.z,
        }
    }

    /// Get the minimum value along a specific axis.
    #[must_use]
    pub fn min_on_axis(&self, axis: Axis) -> f64 {
        match axis {
            Axis::X => self.min.x,
            Axis::Y => self.min.y,
            Axis::Z => self.min.z,
        }
    }

    /// Get the maximum value along a specific axis.
    #[must_use]
    pub fn max_on_axis(&self, axis: Axis) -> f64 {
        match axis {
            Axis::X => self.max.x,
            Axis::Y => self.max.y,
            Axis::Z => self.max.z,
        }
    }

    /// Get the axis with the largest extent.
    #[must_use]
    pub fn longest_axis(&self) -> Axis {
        let size = self.max - self.min;
        if size.x >= size.y && size.x >= size.z {
            Axis::X
        } else if size.y >= size.z {
            Axis::Y
        } else {
            Axis::Z
        }
    }
}

impl Default for Aabb {
    fn default() -> Self {
        Self::new(Point3::origin(), Point3::origin())
    }
}

// ============================================================================
// Coordinate Axis
// ============================================================================

/// Coordinate axis for sweep direction and AABB queries.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Axis {
    /// X-axis.
    X,
    /// Y-axis.
    Y,
    /// Z-axis.
    Z,
}

impl Axis {
    /// Get all three axes.
    #[must_use]
    pub const fn all() -> [Self; 3] {
        [Self::X, Self::Y, Self::Z]
    }

    /// Get the index of this axis (X=0, Y=1, Z=2).
    #[must_use]
    pub const fn index(self) -> usize {
        match self {
            Self::X => 0,
            Self::Y => 1,
            Self::Z => 2,
        }
    }

    /// Create an axis from an index (0=X, 1=Y, 2=Z).
    ///
    /// # Panics
    ///
    /// Panics if `index > 2`.
    #[must_use]
    #[allow(clippy::panic)] // Panic is intentional for invalid index
    pub const fn from_index(index: usize) -> Self {
        match index {
            0 => Self::X,
            1 => Self::Y,
            2 => Self::Z,
            _ => panic!("Axis index must be 0, 1, or 2"),
        }
    }

    /// Get a unit vector along this axis.
    #[must_use]
    pub fn unit_vector(self) -> Vector3<f64> {
        match self {
            Self::X => Vector3::new(1.0, 0.0, 0.0),
            Self::Y => Vector3::new(0.0, 1.0, 0.0),
            Self::Z => Vector3::new(0.0, 0.0, 1.0),
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    // ------------------------------------------------------------------------
    // CollisionShape constructor tests
    // ------------------------------------------------------------------------

    #[test]
    fn test_sphere_creation() {
        let s = CollisionShape::sphere(2.5);
        assert!(s.is_sphere());
        assert!(s.is_convex());
        assert!(s.is_primitive());
        assert_eq!(s.radius(), Some(2.5));
        assert_relative_eq!(s.bounding_radius(), 2.5);
    }

    #[test]
    fn test_box_creation() {
        let b = CollisionShape::box_shape(Vector3::new(1.0, 2.0, 3.0));
        assert!(b.is_box());
        assert!(b.is_convex());
        assert_eq!(b.half_extents(), Some(Vector3::new(1.0, 2.0, 3.0)));
        // Bounding radius is the diagonal: sqrt(1 + 4 + 9) = sqrt(14)
        assert_relative_eq!(b.bounding_radius(), 14.0_f64.sqrt(), epsilon = 1e-10);
    }

    #[test]
    fn test_capsule_creation() {
        let c = CollisionShape::capsule(1.0, 0.5);
        assert!(c.is_capsule());
        assert!(c.is_convex());
        assert_eq!(c.radius(), Some(0.5));
        assert_eq!(c.half_length(), Some(1.0));
        assert_relative_eq!(c.bounding_radius(), 1.5);
    }

    #[test]
    fn test_cylinder_creation() {
        let c = CollisionShape::cylinder(2.0, 1.0);
        assert!(c.is_cylinder());
        assert!(c.is_convex());
        assert_eq!(c.radius(), Some(1.0));
        assert_eq!(c.half_length(), Some(2.0));
        // Bounding radius: sqrt(4 + 1) = sqrt(5)
        assert_relative_eq!(c.bounding_radius(), 5.0_f64.sqrt(), epsilon = 1e-10);
    }

    #[test]
    fn test_ellipsoid_creation() {
        let e = CollisionShape::ellipsoid_xyz(1.0, 2.0, 3.0);
        assert!(e.is_convex());
        assert!(e.is_primitive());
        assert_relative_eq!(e.bounding_radius(), 3.0);
    }

    #[test]
    fn test_plane_creation() {
        let p = CollisionShape::plane(Vector3::new(0.0, 0.0, 1.0), 5.0);
        assert!(p.is_plane());
        assert!(!p.is_convex());
        assert!(p.is_infinite());
        assert_eq!(p.bounding_radius(), f64::INFINITY);
    }

    #[test]
    fn test_ground_plane() {
        let p = CollisionShape::ground_plane(0.0);
        assert!(p.is_plane());
    }

    #[test]
    fn test_tetrahedron_creation() {
        let t = CollisionShape::tetrahedron(1.0);
        assert!(t.is_convex());
        assert!(!t.is_primitive());
        // Vertices are at distance sqrt(3) from origin
        assert_relative_eq!(t.bounding_radius(), 3.0_f64.sqrt(), epsilon = 1e-10);
    }

    #[test]
    fn test_convex_mesh_creation() {
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];
        let m = CollisionShape::convex_mesh(vertices);
        assert!(m.is_convex());
        assert!(m.is_mesh_based());
    }

    // ------------------------------------------------------------------------
    // Type predicate tests
    // ------------------------------------------------------------------------

    #[test]
    fn test_is_convex() {
        assert!(CollisionShape::sphere(1.0).is_convex());
        assert!(CollisionShape::box_shape(Vector3::new(1.0, 1.0, 1.0)).is_convex());
        assert!(CollisionShape::capsule(1.0, 0.5).is_convex());
        assert!(CollisionShape::cylinder(1.0, 0.5).is_convex());
        assert!(CollisionShape::ellipsoid_xyz(1.0, 1.0, 1.0).is_convex());
        assert!(!CollisionShape::plane(Vector3::z(), 0.0).is_convex());
    }

    #[test]
    fn test_is_primitive() {
        assert!(CollisionShape::sphere(1.0).is_primitive());
        assert!(CollisionShape::plane(Vector3::z(), 0.0).is_primitive());
        assert!(!CollisionShape::tetrahedron(1.0).is_primitive());
    }

    // ------------------------------------------------------------------------
    // Aabb tests
    // ------------------------------------------------------------------------

    #[test]
    fn test_aabb_from_center() {
        let aabb = Aabb::from_center(Point3::new(1.0, 2.0, 3.0), Vector3::new(0.5, 1.0, 1.5));
        assert_relative_eq!(aabb.min.x, 0.5);
        assert_relative_eq!(aabb.min.y, 1.0);
        assert_relative_eq!(aabb.min.z, 1.5);
        assert_relative_eq!(aabb.max.x, 1.5);
        assert_relative_eq!(aabb.max.y, 3.0);
        assert_relative_eq!(aabb.max.z, 4.5);
    }

    #[test]
    fn test_aabb_overlaps() {
        let a = Aabb::from_center(Point3::origin(), Vector3::new(1.0, 1.0, 1.0));
        let b = Aabb::from_center(Point3::new(1.5, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));
        let c = Aabb::from_center(Point3::new(3.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));

        assert!(a.overlaps(&b)); // Overlapping
        assert!(!a.overlaps(&c)); // Not overlapping
    }

    #[test]
    fn test_aabb_contains_point() {
        let aabb = Aabb::from_center(Point3::origin(), Vector3::new(1.0, 1.0, 1.0));
        assert!(aabb.contains_point(Point3::origin()));
        assert!(aabb.contains_point(Point3::new(0.5, 0.5, 0.5)));
        assert!(aabb.contains_point(Point3::new(1.0, 1.0, 1.0))); // On boundary
        assert!(!aabb.contains_point(Point3::new(2.0, 0.0, 0.0)));
    }

    #[test]
    fn test_aabb_merged() {
        let a = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        let b = Aabb::new(Point3::new(2.0, 2.0, 2.0), Point3::new(3.0, 3.0, 3.0));
        let merged = a.merged(&b);
        assert_relative_eq!(merged.min.x, 0.0);
        assert_relative_eq!(merged.max.x, 3.0);
    }

    #[test]
    fn test_aabb_center_and_half_extents() {
        let aabb = Aabb::new(Point3::new(0.0, 2.0, 4.0), Point3::new(2.0, 6.0, 10.0));
        let center = aabb.center();
        let he = aabb.half_extents();
        assert_relative_eq!(center.x, 1.0);
        assert_relative_eq!(center.y, 4.0);
        assert_relative_eq!(center.z, 7.0);
        assert_relative_eq!(he.x, 1.0);
        assert_relative_eq!(he.y, 2.0);
        assert_relative_eq!(he.z, 3.0);
    }

    #[test]
    fn test_aabb_volume_and_surface_area() {
        let aabb = Aabb::new(Point3::origin(), Point3::new(2.0, 3.0, 4.0));
        assert_relative_eq!(aabb.volume(), 24.0);
        // Surface area = 2*(2*3 + 3*4 + 4*2) = 2*(6 + 12 + 8) = 52
        assert_relative_eq!(aabb.surface_area(), 52.0);
    }

    #[test]
    fn test_aabb_longest_axis() {
        let a = Aabb::new(Point3::origin(), Point3::new(3.0, 2.0, 1.0));
        assert_eq!(a.longest_axis(), Axis::X);

        let b = Aabb::new(Point3::origin(), Point3::new(1.0, 3.0, 2.0));
        assert_eq!(b.longest_axis(), Axis::Y);

        let c = Aabb::new(Point3::origin(), Point3::new(1.0, 2.0, 3.0));
        assert_eq!(c.longest_axis(), Axis::Z);
    }

    #[test]
    fn test_aabb_is_valid() {
        // Valid AABB
        let valid = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        assert!(valid.is_valid());

        // Zero-size AABB is still valid (min == max)
        let zero_size = Aabb::new(Point3::new(1.0, 2.0, 3.0), Point3::new(1.0, 2.0, 3.0));
        assert!(zero_size.is_valid());

        // Swapped min/max is invalid
        let swapped = Aabb::new(Point3::new(1.0, 1.0, 1.0), Point3::new(0.0, 0.0, 0.0));
        assert!(!swapped.is_valid());

        // Partially swapped (only X) is invalid
        let partial_swap = Aabb::new(Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 1.0));
        assert!(!partial_swap.is_valid());
    }

    #[test]
    fn test_aabb_is_valid_nan_inf() {
        // NaN in min
        let nan_min = Aabb::new(Point3::new(f64::NAN, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        assert!(!nan_min.is_valid());

        // NaN in max
        let nan_max = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, f64::NAN, 1.0));
        assert!(!nan_max.is_valid());

        // Infinity in min
        let inf_min = Aabb::new(
            Point3::new(f64::NEG_INFINITY, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        );
        assert!(!inf_min.is_valid());

        // Infinity in max
        let inf_max = Aabb::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(f64::INFINITY, 1.0, 1.0),
        );
        assert!(!inf_max.is_valid());
    }

    #[test]
    fn test_aabb_from_points_normalized() {
        // Normal case (already ordered)
        let normal =
            Aabb::from_points_normalized(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        assert!(normal.is_valid());
        assert_relative_eq!(normal.min.x, 0.0);
        assert_relative_eq!(normal.max.x, 1.0);

        // Swapped case (needs normalization)
        let swapped =
            Aabb::from_points_normalized(Point3::new(1.0, 1.0, 1.0), Point3::new(0.0, 0.0, 0.0));
        assert!(swapped.is_valid());
        assert_relative_eq!(swapped.min.x, 0.0);
        assert_relative_eq!(swapped.max.x, 1.0);

        // Mixed case (some axes swapped)
        let mixed =
            Aabb::from_points_normalized(Point3::new(1.0, 0.0, 2.0), Point3::new(0.0, 1.0, -1.0));
        assert!(mixed.is_valid());
        assert_relative_eq!(mixed.min.x, 0.0);
        assert_relative_eq!(mixed.max.x, 1.0);
        assert_relative_eq!(mixed.min.y, 0.0);
        assert_relative_eq!(mixed.max.y, 1.0);
        assert_relative_eq!(mixed.min.z, -1.0);
        assert_relative_eq!(mixed.max.z, 2.0);
    }

    // ------------------------------------------------------------------------
    // Axis tests
    // ------------------------------------------------------------------------

    #[test]
    fn test_axis_index() {
        assert_eq!(Axis::X.index(), 0);
        assert_eq!(Axis::Y.index(), 1);
        assert_eq!(Axis::Z.index(), 2);
    }

    #[test]
    fn test_axis_from_index() {
        assert_eq!(Axis::from_index(0), Axis::X);
        assert_eq!(Axis::from_index(1), Axis::Y);
        assert_eq!(Axis::from_index(2), Axis::Z);
    }

    #[test]
    fn test_axis_unit_vector() {
        assert_eq!(Axis::X.unit_vector(), Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(Axis::Y.unit_vector(), Vector3::new(0.0, 1.0, 0.0));
        assert_eq!(Axis::Z.unit_vector(), Vector3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn test_axis_all() {
        assert_eq!(Axis::all(), [Axis::X, Axis::Y, Axis::Z]);
    }
}
