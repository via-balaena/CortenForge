//! Collision shape primitives for low-level collision detection.
//!
//! This module defines the [`CollisionShape`] enum used by GJK/EPA algorithms
//! and raycasting. For the Model/Data architecture, geometry is stored in
//! separate arrays (`geom_type`, `geom_size`, `geom_pos`, etc.).
//!
//! # Example
//!
//! ```
//! use sim_core::{CollisionShape, Aabb};
//! use nalgebra::{Point3, Vector3};
//!
//! // Create primitive shapes
//! let sphere = CollisionShape::sphere(1.0);
//! let box_shape = CollisionShape::box_shape(Vector3::new(0.5, 0.5, 0.5));
//! let capsule = CollisionShape::capsule(1.0, 0.25); // half_length, radius
//!
//! // Query shape properties
//! assert!(sphere.is_convex());
//! assert!(sphere.is_primitive());
//! assert_eq!(sphere.radius(), Some(1.0));
//!
//! // Compute bounding volumes
//! let radius = sphere.bounding_radius();
//! let aabb = box_shape.local_aabb();
//!
//! // Check if two AABBs overlap (broad-phase)
//! let aabb_a = Aabb::from_center(Point3::origin(), Vector3::new(1.0, 1.0, 1.0));
//! let aabb_b = Aabb::from_center(Point3::new(1.5, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));
//! assert!(aabb_a.overlaps(&aabb_b));
//! ```
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

use cf_geometry::{Aabb, Bounded};
use nalgebra::{Point3, Vector3};
use std::cell::Cell;
use std::sync::Arc;

use crate::heightfield::HeightFieldData;
use crate::mesh::TriangleMeshData;
use crate::sdf::SdfGrid;
use cf_geometry::ConvexHull;
use sim_types::Pose;

/// Minimum vertex count for hill-climbing support.
/// Matches MuJoCo's `mjMESH_HILLCLIMB_MIN = 10`.
pub const HILL_CLIMB_MIN: usize = 10;

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
        /// Vertex adjacency for hill-climbing support queries (from cf-geometry ConvexHull).
        /// `None` for manually constructed shapes or hulls with <10 vertices.
        adjacency: Option<Vec<Vec<u32>>>,
        /// Warm-start cache: last support vertex index for hill-climbing.
        /// `Cell` allows mutation through `&self` (interior mutability).
        warm_start: Cell<usize>,
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
        data: Arc<SdfGrid>,
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
        Self::ConvexMesh {
            vertices,
            adjacency: None,
            warm_start: Cell::new(0),
        }
    }

    /// Construct a `ConvexMesh` from a precomputed `ConvexHull`.
    ///
    /// Hill-climbing adjacency is included only for hulls with ≥ `HILL_CLIMB_MIN`
    /// vertices (matching MuJoCo's `mjMESH_HILLCLIMB_MIN = 10`).
    #[must_use]
    pub fn convex_mesh_from_hull(hull: &ConvexHull) -> Self {
        Self::ConvexMesh {
            vertices: hull.vertices.clone(),
            adjacency: if hull.vertices.len() >= HILL_CLIMB_MIN {
                Some(hull.adjacency.clone())
            } else {
                None
            },
            warm_start: Cell::new(0),
        }
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
        Self::ConvexMesh {
            vertices,
            adjacency: None,
            warm_start: Cell::new(0),
        }
    }

    /// Create a height field shape.
    #[must_use]
    pub fn height_field(data: Arc<HeightFieldData>) -> Self {
        Self::HeightField { data }
    }

    /// Create an SDF shape.
    #[must_use]
    pub fn sdf(data: Arc<SdfGrid>) -> Self {
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
    ///
    /// Note: HeightField is grid-based (regular 2D grid with height values) and
    /// is NOT considered mesh-based. SDF is included because it represents
    /// arbitrary geometry via a discrete grid of distance values.
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
            Self::ConvexMesh { vertices, .. } => {
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
                let aabb = data.aabb();
                aabb.min.coords.norm().max(aabb.max.coords.norm())
            }
            Self::TriangleMesh { data } => data
                .vertices()
                .iter()
                .map(|v| v.coords.norm())
                .fold(0.0, f64::max),
        }
    }

    /// Compute the local-space axis-aligned bounding box for this shape.
    ///
    /// The AABB is computed in the shape's local coordinate frame (centered at origin).
    /// For shapes with infinite extent (planes), returns an AABB with infinite bounds.
    ///
    /// # Returns
    ///
    /// An [`Aabb`] representing the tightest axis-aligned bounding box containing
    /// the shape in local coordinates.
    #[must_use]
    #[allow(clippy::cast_precision_loss)] // Grid dimensions won't exceed mantissa precision
    pub fn local_aabb(&self) -> Aabb {
        match self {
            Self::Sphere { radius } => {
                let r = *radius;
                Aabb::new(Point3::new(-r, -r, -r), Point3::new(r, r, r))
            }
            Self::Plane { .. } => {
                // Planes are infinite - return infinite AABB
                Aabb::new(
                    Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
                    Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
                )
            }
            Self::Box { half_extents } => {
                Aabb::new(Point3::from(-*half_extents), Point3::from(*half_extents))
            }
            Self::Capsule {
                half_length,
                radius,
            } => {
                // Capsule is along Z axis
                let r = *radius;
                let h = *half_length;
                Aabb::new(Point3::new(-r, -r, -(h + r)), Point3::new(r, r, h + r))
            }
            Self::Cylinder {
                half_length,
                radius,
            } => {
                // Cylinder is along Z axis
                let r = *radius;
                let h = *half_length;
                Aabb::new(Point3::new(-r, -r, -h), Point3::new(r, r, h))
            }
            Self::Ellipsoid { radii } => Aabb::new(Point3::from(-*radii), Point3::from(*radii)),
            Self::ConvexMesh { vertices, .. } => {
                if vertices.is_empty() {
                    return Aabb::default();
                }
                let mut min = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
                let mut max = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
                for v in vertices {
                    min.x = min.x.min(v.x);
                    min.y = min.y.min(v.y);
                    min.z = min.z.min(v.z);
                    max.x = max.x.max(v.x);
                    max.y = max.y.max(v.y);
                    max.z = max.z.max(v.z);
                }
                Aabb::new(min, max)
            }
            Self::TriangleMesh { data } => {
                let (min, max) = data.aabb();
                Aabb::new(min, max)
            }
            Self::HeightField { data } => data.aabb(),
            Self::Sdf { data } => data.aabb(),
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
    // TriangleMesh, HeightField, Sdf creation tests
    // ------------------------------------------------------------------------

    #[test]
    fn test_triangle_mesh_creation() {
        use crate::mesh::TriangleMeshData;
        use std::sync::Arc;

        // Create a simple tetrahedron
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
            Point3::new(0.5, 0.5, 1.0),
        ];
        let indices = vec![0, 1, 2, 0, 1, 3, 1, 2, 3, 0, 2, 3];

        let data = Arc::new(TriangleMeshData::new(vertices, indices));
        let shape = CollisionShape::triangle_mesh(data);

        assert!(!shape.is_convex());
        assert!(!shape.is_primitive());
        assert!(shape.is_mesh_based());
        assert!(!shape.is_infinite());

        // Bounding radius should be the max distance from origin
        // Vertex (0.5, 0.5, 1.0) has distance sqrt(0.25 + 0.25 + 1.0) = sqrt(1.5)
        // Vertex (0.5, 1.0, 0.0) has distance sqrt(0.25 + 1.0) = sqrt(1.25)
        // Vertex (1.0, 0.0, 0.0) has distance 1.0
        // Max is sqrt(1.5) ≈ 1.2247
        assert_relative_eq!(shape.bounding_radius(), 1.5_f64.sqrt(), epsilon = 1e-10);
    }

    #[test]
    fn test_triangle_mesh_from_vertices_convenience() {
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
            Point3::new(0.5, 0.5, 1.0),
        ];
        let indices = vec![0, 1, 2, 0, 1, 3, 1, 2, 3, 0, 2, 3];

        let shape = CollisionShape::triangle_mesh_from_vertices(vertices, indices);
        assert!(shape.is_mesh_based());
        assert!(!shape.is_convex());
    }

    #[test]
    fn test_height_field_creation() {
        use crate::heightfield::HeightFieldData;
        use std::sync::Arc;

        // Create a simple 3x3 height field
        let heights = vec![
            0.0, 1.0, 0.0, // row 0
            1.0, 2.0, 1.0, // row 1
            0.0, 1.0, 0.0, // row 2
        ];
        let data = Arc::new(HeightFieldData::new(heights, 3, 3, 1.0));
        let shape = CollisionShape::height_field(data);

        assert!(!shape.is_convex());
        assert!(!shape.is_primitive());
        assert!(!shape.is_mesh_based()); // HeightField is not mesh-based
        assert!(!shape.is_infinite());

        // Bounding radius computation for height field
        let radius = shape.bounding_radius();
        assert!(radius > 0.0);
        assert!(radius.is_finite());
    }

    #[test]
    fn test_sdf_creation() {
        use crate::sdf::SdfGrid;
        use std::sync::Arc;

        // Create a simple 2x2x2 SDF representing a sphere-ish shape
        let sphere_radius = 1.0;
        let data = Arc::new(SdfGrid::from_fn(
            4,
            4,
            4,
            0.5,
            Point3::new(-1.0, -1.0, -1.0),
            |p| p.coords.norm() - sphere_radius,
        ));
        let shape = CollisionShape::sdf(data);

        assert!(!shape.is_convex());
        assert!(!shape.is_primitive());
        assert!(shape.is_mesh_based()); // SDF is considered mesh-based
        assert!(!shape.is_infinite());

        // Bounding radius should be finite and positive
        let radius = shape.bounding_radius();
        assert!(radius > 0.0);
        assert!(radius.is_finite());
    }

    // ------------------------------------------------------------------------
    // Comprehensive bounding_radius test
    // ------------------------------------------------------------------------

    #[test]
    fn test_bounding_radius_all_types() {
        use crate::heightfield::HeightFieldData;
        use crate::mesh::TriangleMeshData;
        use crate::sdf::SdfGrid;
        use std::sync::Arc;

        // Sphere: radius = r
        let sphere = CollisionShape::sphere(2.0);
        assert_relative_eq!(sphere.bounding_radius(), 2.0);

        // Plane: infinite
        let plane = CollisionShape::plane(Vector3::z(), 0.0);
        assert_eq!(plane.bounding_radius(), f64::INFINITY);

        // Box: diagonal = sqrt(x² + y² + z²)
        let box_shape = CollisionShape::box_shape(Vector3::new(1.0, 2.0, 3.0));
        assert_relative_eq!(
            box_shape.bounding_radius(),
            14.0_f64.sqrt(),
            epsilon = 1e-10
        );

        // Capsule: half_length + radius
        let capsule = CollisionShape::capsule(2.0, 0.5);
        assert_relative_eq!(capsule.bounding_radius(), 2.5);

        // Cylinder: hypot(half_length, radius)
        let cylinder = CollisionShape::cylinder(3.0, 4.0);
        assert_relative_eq!(cylinder.bounding_radius(), 5.0, epsilon = 1e-10); // 3-4-5 triangle

        // Ellipsoid: max of radii
        let ellipsoid = CollisionShape::ellipsoid_xyz(1.0, 2.0, 5.0);
        assert_relative_eq!(ellipsoid.bounding_radius(), 5.0);

        // ConvexMesh: max vertex distance from origin
        let vertices = vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(0.0, 0.0, 3.0),
            Point3::new(0.0, 0.0, 0.0),
        ];
        let convex = CollisionShape::convex_mesh(vertices);
        assert_relative_eq!(convex.bounding_radius(), 3.0, epsilon = 1e-10);

        // TriangleMesh: max vertex distance from origin
        let tri_verts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(0.0, 3.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        ];
        let tri_data = Arc::new(TriangleMeshData::new(tri_verts, vec![0, 1, 2, 0, 2, 3]));
        let tri_mesh = CollisionShape::triangle_mesh(tri_data);
        assert_relative_eq!(tri_mesh.bounding_radius(), 4.0, epsilon = 1e-10);

        // HeightField: computed from grid extent and height range
        let hf_data = Arc::new(HeightFieldData::flat(5, 5, 1.0, 0.0));
        let hf = CollisionShape::height_field(hf_data);
        assert!(hf.bounding_radius().is_finite());
        assert!(hf.bounding_radius() > 0.0);

        // SDF: computed from grid corners
        let sdf_data = Arc::new(SdfGrid::from_fn(2, 2, 2, 1.0, Point3::origin(), |_| 0.0));
        let sdf = CollisionShape::sdf(sdf_data);
        assert!(sdf.bounding_radius().is_finite());
        assert!(sdf.bounding_radius() > 0.0);
    }

    // ------------------------------------------------------------------------
    // Capsule endpoints test
    // ------------------------------------------------------------------------

    #[test]
    fn test_capsule_endpoints() {
        let capsule = CollisionShape::capsule(2.0, 0.5);

        // Identity pose
        let pose = Pose::identity();
        let endpoints = capsule.capsule_endpoints(&pose);
        assert!(endpoints.is_some());
        let (start, end) = endpoints.unwrap();
        assert_relative_eq!(start.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(start.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(start.z, -2.0, epsilon = 1e-10);
        assert_relative_eq!(end.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(end.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(end.z, 2.0, epsilon = 1e-10);

        // Translated pose
        let translated_pose = Pose::from_position(Point3::new(1.0, 2.0, 3.0));
        let endpoints = capsule.capsule_endpoints(&translated_pose);
        assert!(endpoints.is_some());
        let (start, end) = endpoints.unwrap();
        assert_relative_eq!(start.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(start.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(start.z, 1.0, epsilon = 1e-10); // 3 - 2
        assert_relative_eq!(end.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(end.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(end.z, 5.0, epsilon = 1e-10); // 3 + 2

        // 90-degree rotation around Y axis (Z axis becomes X axis)
        let rotated_pose = Pose::from_position_rotation(
            Point3::origin(),
            nalgebra::UnitQuaternion::from_euler_angles(0.0, std::f64::consts::FRAC_PI_2, 0.0),
        );
        let endpoints = capsule.capsule_endpoints(&rotated_pose);
        assert!(endpoints.is_some());
        let (start, end) = endpoints.unwrap();
        // After 90° Y rotation: local Z maps to world X
        assert_relative_eq!(start.x, -2.0, epsilon = 1e-10);
        assert_relative_eq!(start.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(start.z, 0.0, epsilon = 1e-10);
        assert_relative_eq!(end.x, 2.0, epsilon = 1e-10);
        assert_relative_eq!(end.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(end.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_capsule_endpoints_non_capsule_returns_none() {
        let sphere = CollisionShape::sphere(1.0);
        let pose = Pose::identity();
        assert!(sphere.capsule_endpoints(&pose).is_none());

        let box_shape = CollisionShape::box_shape(Vector3::new(1.0, 1.0, 1.0));
        assert!(box_shape.capsule_endpoints(&pose).is_none());

        let cylinder = CollisionShape::cylinder(1.0, 0.5);
        assert!(cylinder.capsule_endpoints(&pose).is_none());
    }

    // ------------------------------------------------------------------------
    // Accessor negative tests
    // ------------------------------------------------------------------------

    #[test]
    fn test_accessors_return_none_for_wrong_types() {
        // Test radius accessor
        let box_shape = CollisionShape::box_shape(Vector3::new(1.0, 1.0, 1.0));
        assert!(box_shape.radius().is_none());

        let plane = CollisionShape::plane(Vector3::z(), 0.0);
        assert!(plane.radius().is_none());

        let ellipsoid = CollisionShape::ellipsoid_xyz(1.0, 2.0, 3.0);
        assert!(ellipsoid.radius().is_none());

        // Test half_length accessor
        let sphere = CollisionShape::sphere(1.0);
        assert!(sphere.half_length().is_none());

        assert!(box_shape.half_length().is_none());
        assert!(plane.half_length().is_none());

        // Test half_extents accessor
        assert!(sphere.half_extents().is_none());

        let capsule = CollisionShape::capsule(1.0, 0.5);
        assert!(capsule.half_extents().is_none());

        let cylinder = CollisionShape::cylinder(1.0, 0.5);
        assert!(cylinder.half_extents().is_none());
    }

    // ------------------------------------------------------------------------
    // AABB expanded test
    // ------------------------------------------------------------------------

    #[test]
    fn test_aabb_expanded() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0));

        let expanded = aabb.expanded(0.5);
        assert!(expanded.is_valid());
        assert_relative_eq!(expanded.min.x, -0.5, epsilon = 1e-10);
        assert_relative_eq!(expanded.min.y, -0.5, epsilon = 1e-10);
        assert_relative_eq!(expanded.min.z, -0.5, epsilon = 1e-10);
        assert_relative_eq!(expanded.max.x, 2.5, epsilon = 1e-10);
        assert_relative_eq!(expanded.max.y, 2.5, epsilon = 1e-10);
        assert_relative_eq!(expanded.max.z, 2.5, epsilon = 1e-10);

        // Negative margin (shrink)
        let shrunk = aabb.expanded(-0.5);
        assert!(shrunk.is_valid());
        assert_relative_eq!(shrunk.min.x, 0.5, epsilon = 1e-10);
        assert_relative_eq!(shrunk.max.x, 1.5, epsilon = 1e-10);

        // Zero margin (no change)
        let unchanged = aabb.expanded(0.0);
        assert_eq!(unchanged.min, aabb.min);
        assert_eq!(unchanged.max, aabb.max);
    }

    // ------------------------------------------------------------------------
    // Zero half-length capsule edge case
    // ------------------------------------------------------------------------

    #[test]
    fn test_zero_half_length_capsule() {
        // A capsule with zero half_length is effectively a sphere
        let capsule = CollisionShape::capsule(0.0, 1.0);

        assert!(capsule.is_capsule());
        assert!(capsule.is_convex());
        assert_eq!(capsule.radius(), Some(1.0));
        assert_eq!(capsule.half_length(), Some(0.0));

        // Bounding radius should equal the radius when half_length is 0
        assert_relative_eq!(capsule.bounding_radius(), 1.0);

        // Endpoints should be coincident
        let pose = Pose::identity();
        let (start, end) = capsule.capsule_endpoints(&pose).unwrap();
        assert_relative_eq!(start.x, end.x, epsilon = 1e-10);
        assert_relative_eq!(start.y, end.y, epsilon = 1e-10);
        assert_relative_eq!(start.z, end.z, epsilon = 1e-10);
    }

    // ------------------------------------------------------------------------
    // Predicate tests
    // ------------------------------------------------------------------------

    #[test]
    fn test_is_mesh_based() {
        use crate::mesh::TriangleMeshData;
        use crate::sdf::SdfGrid;
        use std::sync::Arc;

        // Mesh-based shapes
        let convex = CollisionShape::convex_mesh(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ]);
        assert!(convex.is_mesh_based());

        let tri_data = Arc::new(TriangleMeshData::new(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 0.0, 1.0),
            ],
            vec![0, 1, 2, 0, 1, 3],
        ));
        let tri_mesh = CollisionShape::triangle_mesh(tri_data);
        assert!(tri_mesh.is_mesh_based());

        let sdf_data = Arc::new(SdfGrid::from_fn(2, 2, 2, 1.0, Point3::origin(), |_| 0.0));
        let sdf = CollisionShape::sdf(sdf_data);
        assert!(sdf.is_mesh_based());

        // Non-mesh-based shapes
        assert!(!CollisionShape::sphere(1.0).is_mesh_based());
        assert!(!CollisionShape::box_shape(Vector3::new(1.0, 1.0, 1.0)).is_mesh_based());
        assert!(!CollisionShape::capsule(1.0, 0.5).is_mesh_based());
        assert!(!CollisionShape::cylinder(1.0, 0.5).is_mesh_based());
        assert!(!CollisionShape::ellipsoid_xyz(1.0, 1.0, 1.0).is_mesh_based());
        assert!(!CollisionShape::plane(Vector3::z(), 0.0).is_mesh_based());
    }

    #[test]
    fn test_is_infinite() {
        // Only planes are infinite
        assert!(CollisionShape::plane(Vector3::z(), 0.0).is_infinite());
        assert!(CollisionShape::ground_plane(0.0).is_infinite());

        // All other shapes are finite
        assert!(!CollisionShape::sphere(1.0).is_infinite());
        assert!(!CollisionShape::box_shape(Vector3::new(1.0, 1.0, 1.0)).is_infinite());
        assert!(!CollisionShape::capsule(1.0, 0.5).is_infinite());
        assert!(!CollisionShape::cylinder(1.0, 0.5).is_infinite());
        assert!(!CollisionShape::ellipsoid_xyz(1.0, 1.0, 1.0).is_infinite());
        assert!(!CollisionShape::tetrahedron(1.0).is_infinite());
    }

    // ------------------------------------------------------------------------
    // local_aabb tests
    // ------------------------------------------------------------------------

    #[test]
    fn test_local_aabb_primitives() {
        // Sphere
        let sphere = CollisionShape::sphere(2.0);
        let aabb = sphere.local_aabb();
        assert!(aabb.is_valid());
        assert_relative_eq!(aabb.min.x, -2.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.x, 2.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.min.y, -2.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.min.z, -2.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.z, 2.0, epsilon = 1e-10);

        // Box
        let box_shape = CollisionShape::box_shape(Vector3::new(1.0, 2.0, 3.0));
        let aabb = box_shape.local_aabb();
        assert!(aabb.is_valid());
        assert_relative_eq!(aabb.min.x, -1.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.min.y, -2.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.min.z, -3.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.z, 3.0, epsilon = 1e-10);

        // Capsule (half_length=2, radius=0.5, aligned along Z)
        let capsule = CollisionShape::capsule(2.0, 0.5);
        let aabb = capsule.local_aabb();
        assert!(aabb.is_valid());
        assert_relative_eq!(aabb.min.x, -0.5, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.x, 0.5, epsilon = 1e-10);
        assert_relative_eq!(aabb.min.z, -2.5, epsilon = 1e-10); // -(half_length + radius)
        assert_relative_eq!(aabb.max.z, 2.5, epsilon = 1e-10);

        // Cylinder (half_length=2, radius=0.5, aligned along Z)
        let cylinder = CollisionShape::cylinder(2.0, 0.5);
        let aabb = cylinder.local_aabb();
        assert!(aabb.is_valid());
        assert_relative_eq!(aabb.min.x, -0.5, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.x, 0.5, epsilon = 1e-10);
        assert_relative_eq!(aabb.min.z, -2.0, epsilon = 1e-10); // just half_length
        assert_relative_eq!(aabb.max.z, 2.0, epsilon = 1e-10);

        // Ellipsoid
        let ellipsoid = CollisionShape::ellipsoid_xyz(1.0, 2.0, 3.0);
        let aabb = ellipsoid.local_aabb();
        assert!(aabb.is_valid());
        assert_relative_eq!(aabb.min.x, -1.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.min.y, -2.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.min.z, -3.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.z, 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_local_aabb_plane_infinite() {
        let plane = CollisionShape::plane(Vector3::z(), 0.0);
        let aabb = plane.local_aabb();
        // Plane AABB should be infinite (not "valid" in the usual sense)
        assert_eq!(aabb.min.x, f64::NEG_INFINITY);
        assert_eq!(aabb.max.x, f64::INFINITY);
    }

    #[test]
    fn test_local_aabb_convex_mesh() {
        let vertices = vec![
            Point3::new(-1.0, -2.0, -3.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(0.0, 5.0, 0.0),
            Point3::new(0.0, 0.0, 6.0),
        ];
        let mesh = CollisionShape::convex_mesh(vertices);
        let aabb = mesh.local_aabb();
        assert!(aabb.is_valid());
        assert_relative_eq!(aabb.min.x, -1.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.x, 4.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.min.y, -2.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.y, 5.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.min.z, -3.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.z, 6.0, epsilon = 1e-10);
    }

    #[test]
    fn test_local_aabb_triangle_mesh() {
        use crate::mesh::TriangleMeshData;
        use std::sync::Arc;

        let vertices = vec![
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(2.0, -1.0, 0.0),
            Point3::new(0.0, 3.0, 0.0),
            Point3::new(0.0, 0.0, 4.0),
        ];
        let data = Arc::new(TriangleMeshData::new(vertices, vec![0, 1, 2, 0, 1, 3]));
        let shape = CollisionShape::triangle_mesh(data);
        let aabb = shape.local_aabb();

        assert!(aabb.is_valid());
        assert_relative_eq!(aabb.min.x, -1.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.x, 2.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.min.y, -1.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.y, 3.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.min.z, 0.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.z, 4.0, epsilon = 1e-10);
    }

    #[test]
    fn test_local_aabb_height_field() {
        use crate::heightfield::HeightFieldData;
        use std::sync::Arc;

        // 3x3 height field with cell_size=1.0
        // Grid spans from (0,0) to (2,2) in XY, heights from -1 to 2
        let heights = vec![
            0.0, 1.0, 0.0, // row 0
            -1.0, 2.0, 1.0, // row 1
            0.0, 0.0, 0.0, // row 2
        ];
        let data = Arc::new(HeightFieldData::new(heights, 3, 3, 1.0));
        let shape = CollisionShape::height_field(data);
        let aabb = shape.local_aabb();

        assert!(aabb.is_valid());
        // HeightFieldData::aabb() returns the actual bounds
        assert!(aabb.min.z <= -1.0); // min height
        assert!(aabb.max.z >= 2.0); // max height
    }

    #[test]
    fn test_local_aabb_sdf() {
        use crate::sdf::SdfGrid;
        use std::sync::Arc;

        // 3x3x3 SDF grid with cell_size=0.5, origin at (-1, -1, -1)
        // Grid extends from (-1,-1,-1) to (-1 + (3-1)*0.5, ...) = (0.0, 0.0, 0.0)
        // 3 samples = 2 cells = 2 * 0.5 = 1.0 extent per axis
        let data = Arc::new(SdfGrid::from_fn(
            3,
            3,
            3,
            0.5,
            Point3::new(-1.0, -1.0, -1.0),
            |p| p.coords.norm() - 0.5, // sphere SDF
        ));
        let shape = CollisionShape::sdf(data);
        let aabb = shape.local_aabb();

        assert!(aabb.is_valid());
        assert_relative_eq!(aabb.min.x, -1.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.min.y, -1.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.min.z, -1.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(aabb.max.z, 0.0, epsilon = 1e-10);
    }
}
