//! Collision shape primitives for low-level collision detection.
//!
//! This module defines the `CollisionShape` enum used by GJK/EPA algorithms
//! and raycasting. For the Model/Data architecture, geometry is stored in
//! separate arrays (`geom_type`, `geom_size`, `geom_pos`, etc.).

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
        /// Sphere radius in meters.
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
        /// Half-extents of the box in each axis.
        half_extents: Vector3<f64>,
    },
    /// Capsule (cylinder with hemispherical caps).
    ///
    /// Defined by half-length along the local Z-axis and radius.
    Capsule {
        /// Half-length of the cylindrical portion along the Z-axis.
        half_length: f64,
        /// Radius of the capsule.
        radius: f64,
    },
    /// Convex mesh defined by a set of vertices.
    ConvexMesh {
        /// Vertices of the convex hull in local coordinates.
        vertices: Vec<Point3<f64>>,
    },
    /// Cylinder (without hemispherical caps).
    Cylinder {
        /// Half-length of the cylinder along the Z-axis.
        half_length: f64,
        /// Radius of the cylinder.
        radius: f64,
    },
    /// Ellipsoid (scaled sphere).
    Ellipsoid {
        /// Radii along each local axis (X, Y, Z).
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
    /// Create a sphere shape.
    #[must_use]
    pub fn sphere(radius: f64) -> Self {
        Self::Sphere { radius }
    }

    /// Create a ground plane at height 0.
    #[must_use]
    pub fn ground_plane(height: f64) -> Self {
        Self::Plane {
            normal: Vector3::new(0.0, 0.0, 1.0),
            distance: height,
        }
    }

    /// Create a box shape.
    #[must_use]
    pub fn box_shape(half_extents: Vector3<f64>) -> Self {
        Self::Box { half_extents }
    }

    /// Create a capsule shape.
    #[must_use]
    pub fn capsule(half_length: f64, radius: f64) -> Self {
        Self::Capsule {
            half_length,
            radius,
        }
    }

    /// Create a cylinder shape.
    #[must_use]
    pub fn cylinder(half_length: f64, radius: f64) -> Self {
        Self::Cylinder {
            half_length,
            radius,
        }
    }

    /// Create an ellipsoid shape.
    #[must_use]
    pub fn ellipsoid(radii: Vector3<f64>) -> Self {
        Self::Ellipsoid { radii }
    }

    /// Create a convex mesh shape from vertices.
    #[must_use]
    pub fn convex_mesh(vertices: Vec<Point3<f64>>) -> Self {
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

    /// Create a plane shape with a given normal and distance from origin.
    #[must_use]
    pub fn plane(normal: Vector3<f64>, distance: f64) -> Self {
        Self::Plane { normal, distance }
    }

    /// Create an ellipsoid shape with separate radii for each axis.
    #[must_use]
    pub fn ellipsoid_xyz(rx: f64, ry: f64, rz: f64) -> Self {
        Self::Ellipsoid {
            radii: Vector3::new(rx, ry, rz),
        }
    }

    /// Get the world-space endpoints of a capsule shape.
    ///
    /// Returns `None` if this shape is not a capsule.
    /// The capsule is defined along the local Z-axis, so the endpoints
    /// are at (0, 0, -half_length) and (0, 0, +half_length) in local space.
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

    /// Create a regular tetrahedron centered at origin.
    ///
    /// The tetrahedron has vertices at positions where each coordinate
    /// is ±edge_half_length, forming a regular tetrahedron.
    #[must_use]
    pub fn tetrahedron(edge_half_length: f64) -> Self {
        // Regular tetrahedron with vertices on alternating corners of a cube
        let h = edge_half_length;
        let vertices = vec![
            Point3::new(h, h, h),
            Point3::new(h, -h, -h),
            Point3::new(-h, h, -h),
            Point3::new(-h, -h, h),
        ];
        Self::ConvexMesh { vertices }
    }

    /// Compute the bounding radius of this shape from its local origin.
    ///
    /// This is the maximum distance from the shape's center to any point
    /// on its surface in local coordinates.
    #[must_use]
    pub fn bounding_radius(&self) -> f64 {
        match self {
            Self::Sphere { radius } => *radius,
            Self::Plane { .. } => f64::INFINITY, // Planes are infinite
            Self::Box { half_extents } => half_extents.norm(),
            Self::Capsule {
                half_length,
                radius,
            } => half_length + radius,
            Self::Cylinder {
                half_length,
                radius,
            } => (half_length.powi(2) + radius.powi(2)).sqrt(),
            Self::Ellipsoid { radii } => radii.x.max(radii.y).max(radii.z),
            Self::ConvexMesh { vertices } => {
                vertices.iter().map(|v| v.coords.norm()).fold(0.0, f64::max)
            }
            Self::HeightField { data } => {
                // Compute from heightfield dimensions
                let half_width = (data.width() as f64) * data.cell_size() * 0.5;
                let half_depth = (data.depth() as f64) * data.cell_size() * 0.5;
                let max_height = data.max_height().abs().max(data.min_height().abs());
                (half_width.powi(2) + half_depth.powi(2) + max_height.powi(2)).sqrt()
            }
            Self::Sdf { data } => {
                // Compute half-extents from grid dimensions
                let half_x = (data.width() as f64) * data.cell_size() * 0.5;
                let half_y = (data.height() as f64) * data.cell_size() * 0.5;
                let half_z = (data.depth() as f64) * data.cell_size() * 0.5;
                // Compute from origin to furthest corner
                let origin = data.origin();
                let max_corner = Point3::new(
                    origin.x + data.width() as f64 * data.cell_size(),
                    origin.y + data.height() as f64 * data.cell_size(),
                    origin.z + data.depth() as f64 * data.cell_size(),
                );
                origin.coords.norm().max(max_corner.coords.norm())
            }
            Self::TriangleMesh { data } => {
                // Compute from mesh vertices
                data.vertices()
                    .iter()
                    .map(|v| v.coords.norm())
                    .fold(0.0, f64::max)
            }
        }
    }
}

/// Axis-aligned bounding box.
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

    /// Check if this AABB overlaps with another AABB.
    #[must_use]
    pub fn overlaps(&self, other: &Self) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
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
}

impl Default for Aabb {
    fn default() -> Self {
        Self::new(Point3::origin(), Point3::origin())
    }
}

/// Coordinate axis for sweep direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
}
