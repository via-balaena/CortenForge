//! Geometric shape enum — the canonical shape representation.
//!
//! [`Shape`] covers the 10 fundamental geometric primitives (matching `MuJoCo`'s
//! geom types). Each variant is a pure geometric description — physics properties
//! (mass, friction, warm-start cache) belong to the consuming layer (sim-core).
//!
//! All variants are `Send + Sync`. No `Cell`, no `RefCell`, no interior
//! mutability. The rendering layer converts to f32 at the boundary.
//!
//! # Variants
//!
//! | Variant | Convex | Primitive | Description |
//! |---------|--------|-----------|-------------|
//! | [`Shape::Sphere`] | Yes | Yes | Sphere centered at origin |
//! | [`Shape::Plane`] | No | Yes | Infinite half-space |
//! | [`Shape::Box`] | Yes | Yes | Axis-aligned box centered at origin |
//! | [`Shape::Capsule`] | Yes | Yes | Capsule along Z axis |
//! | [`Shape::Cylinder`] | Yes | Yes | Cylinder along Z axis |
//! | [`Shape::Ellipsoid`] | Yes | Yes | Ellipsoid centered at origin |
//! | [`Shape::ConvexMesh`] | Yes | No | Convex hull with adjacency graph |
//! | [`Shape::TriangleMesh`] | No | No | Triangle mesh with BVH |
//! | [`Shape::HeightField`] | No | No | 2D height field grid |
//! | [`Shape::Sdf`] | No | No | 3D signed distance field |
//!
//! # Phase 2 scope
//!
//! This is data only. Query dispatch (`Geometric`, `SupportMap` traits) is
//! added in Phase 5. No ray casting, no closest-point, no GJK here.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};

use crate::{Aabb, Bounded, Bvh, ConvexHull, HeightFieldData, IndexedMesh, SdfGrid};

/// Geometric shape — the canonical shape representation.
///
/// Replaces: `sim_core::CollisionShape`
///
/// Each variant is a pure geometric description. Physics properties
/// (mass, friction, restitution, warm-start cache) are NOT part of the
/// shape — those belong to the physics layer (sim-core).
///
/// All variants are `Send + Sync`. No `Cell`, no `RefCell`.
#[derive(Debug, Clone)]
pub enum Shape {
    /// Sphere centered at origin with the given radius.
    Sphere {
        /// Radius (must be positive).
        radius: f64,
    },

    /// Infinite half-space defined by a plane.
    ///
    /// The plane equation is `normal · x = distance`. Everything on the
    /// positive side of the plane is outside.
    Plane {
        /// Outward unit normal.
        normal: Vector3<f64>,
        /// Signed distance from origin along the normal.
        distance: f64,
    },

    /// Axis-aligned box centered at origin.
    Box {
        /// Half-extents along each axis (all positive).
        half_extents: Vector3<f64>,
    },

    /// Capsule (swept sphere) along the Z axis.
    Capsule {
        /// Half-length of the cylindrical segment along Z.
        half_length: f64,
        /// Radius of the spherical caps.
        radius: f64,
    },

    /// Cylinder along the Z axis.
    Cylinder {
        /// Half-length along Z.
        half_length: f64,
        /// Radius in the XY plane.
        radius: f64,
    },

    /// Ellipsoid centered at origin.
    Ellipsoid {
        /// Semi-axis lengths along X, Y, Z (all positive).
        radii: Vector3<f64>,
    },

    /// Convex mesh with adjacency graph for GJK hill-climbing.
    ///
    /// The hull contains vertices, triangulated faces, outward normals, and
    /// vertex adjacency. No vertex redundancy — `hull.vertices` IS the
    /// vertex data.
    ///
    /// # Note on warm-start
    ///
    /// sim-core's `CollisionShape::ConvexMesh` stored a `Cell<usize>` for
    /// GJK hill-climbing warm-start. Since cf-geometry requires `Send + Sync`,
    /// that cache lives in sim-core (e.g. on a per-geom `GeomState`), not here.
    ConvexMesh {
        /// The convex hull geometry + adjacency graph.
        hull: ConvexHull,
    },

    /// Triangle mesh with BVH acceleration structure.
    ///
    /// The mesh and BVH are `Arc`-wrapped for cheap cloning and sharing
    /// across multiple geoms that reference the same mesh data.
    TriangleMesh {
        /// The triangle mesh geometry.
        mesh: Arc<IndexedMesh>,
        /// Bounding volume hierarchy for spatial queries.
        bvh: Arc<Bvh>,
    },

    /// Height field terrain.
    ///
    /// `Arc`-wrapped for sharing across multiple geoms.
    HeightField {
        /// The height field grid data.
        data: Arc<HeightFieldData>,
    },

    /// Signed distance field.
    ///
    /// `Arc`-wrapped for sharing across multiple geoms.
    Sdf {
        /// The SDF grid data.
        data: Arc<SdfGrid>,
    },
}

// Static assertion: Shape must be Send + Sync.
// If Shape ever gains a !Send or !Sync field, this will fail to compile.
const fn assert_send_sync<T: Send + Sync>() {}
const _: () = assert_send_sync::<Shape>();

// ---------------------------------------------------------------------------
// Construction helpers
// ---------------------------------------------------------------------------

impl Shape {
    /// Creates a sphere shape.
    ///
    /// # Panics (debug only)
    ///
    /// Debug-asserts that `radius` is positive and finite.
    #[must_use]
    pub fn sphere(radius: f64) -> Self {
        debug_assert!(
            radius > 0.0 && radius.is_finite(),
            "radius must be positive and finite"
        );
        Self::Sphere { radius }
    }

    /// Creates a plane (infinite half-space).
    ///
    /// `normal` should be unit-length. `distance` is the signed offset from
    /// origin along the normal.
    ///
    /// # Panics (debug only)
    ///
    /// Debug-asserts that `normal` is non-zero and `distance` is finite.
    #[must_use]
    pub fn plane(normal: Vector3<f64>, distance: f64) -> Self {
        debug_assert!(normal.norm_squared() > 0.0, "normal must be non-zero");
        debug_assert!(distance.is_finite(), "distance must be finite");
        Self::Plane { normal, distance }
    }

    /// Creates a Z-up ground plane at the given height.
    #[must_use]
    pub fn ground_plane(height: f64) -> Self {
        Self::plane(Vector3::z(), height)
    }

    /// Creates a box shape from half-extents.
    ///
    /// # Panics (debug only)
    ///
    /// Debug-asserts that all half-extents are positive and finite.
    #[must_use]
    pub fn box_shape(half_extents: Vector3<f64>) -> Self {
        debug_assert!(
            half_extents.x > 0.0 && half_extents.y > 0.0 && half_extents.z > 0.0,
            "all half-extents must be positive"
        );
        debug_assert!(
            half_extents.x.is_finite() && half_extents.y.is_finite() && half_extents.z.is_finite(),
            "all half-extents must be finite"
        );
        Self::Box { half_extents }
    }

    /// Creates a capsule (swept sphere along Z).
    ///
    /// # Panics (debug only)
    ///
    /// Debug-asserts that `half_length >= 0` and `radius > 0`.
    #[must_use]
    pub fn capsule(half_length: f64, radius: f64) -> Self {
        debug_assert!(
            half_length >= 0.0 && half_length.is_finite(),
            "half_length must be non-negative and finite"
        );
        debug_assert!(
            radius > 0.0 && radius.is_finite(),
            "radius must be positive and finite"
        );
        Self::Capsule {
            half_length,
            radius,
        }
    }

    /// Creates a cylinder along Z.
    ///
    /// # Panics (debug only)
    ///
    /// Debug-asserts that both `half_length` and `radius` are positive and finite.
    #[must_use]
    pub fn cylinder(half_length: f64, radius: f64) -> Self {
        debug_assert!(
            half_length > 0.0 && half_length.is_finite(),
            "half_length must be positive and finite"
        );
        debug_assert!(
            radius > 0.0 && radius.is_finite(),
            "radius must be positive and finite"
        );
        Self::Cylinder {
            half_length,
            radius,
        }
    }

    /// Creates an ellipsoid from semi-axis lengths.
    ///
    /// # Panics (debug only)
    ///
    /// Debug-asserts that all radii are positive and finite.
    #[must_use]
    pub fn ellipsoid(radii: Vector3<f64>) -> Self {
        debug_assert!(
            radii.x > 0.0 && radii.y > 0.0 && radii.z > 0.0,
            "all radii must be positive"
        );
        debug_assert!(
            radii.x.is_finite() && radii.y.is_finite() && radii.z.is_finite(),
            "all radii must be finite"
        );
        Self::Ellipsoid { radii }
    }

    /// Creates an ellipsoid from individual semi-axis lengths.
    #[must_use]
    pub fn ellipsoid_xyz(rx: f64, ry: f64, rz: f64) -> Self {
        Self::ellipsoid(Vector3::new(rx, ry, rz))
    }

    /// Creates a convex mesh shape from a [`ConvexHull`].
    #[must_use]
    pub const fn convex_mesh(hull: ConvexHull) -> Self {
        Self::ConvexMesh { hull }
    }

    /// Creates a triangle mesh shape from shared mesh and BVH data.
    #[must_use]
    pub const fn triangle_mesh(mesh: Arc<IndexedMesh>, bvh: Arc<Bvh>) -> Self {
        Self::TriangleMesh { mesh, bvh }
    }

    /// Creates a height field shape from shared data.
    #[must_use]
    pub const fn height_field(data: Arc<HeightFieldData>) -> Self {
        Self::HeightField { data }
    }

    /// Creates an SDF shape from shared data.
    #[must_use]
    pub const fn sdf(data: Arc<SdfGrid>) -> Self {
        Self::Sdf { data }
    }
}

// ---------------------------------------------------------------------------
// Type predicates
// ---------------------------------------------------------------------------

impl Shape {
    /// Returns `true` if this shape is convex.
    ///
    /// Convex shapes support GJK/EPA algorithms directly.
    /// `Sphere`, `Box`, `Capsule`, `Cylinder`, `Ellipsoid`, and `ConvexMesh` are convex.
    #[must_use]
    pub const fn is_convex(&self) -> bool {
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

    /// Returns `true` if this shape is an analytic primitive.
    ///
    /// Primitives have closed-form solutions for most geometric queries.
    /// `Sphere`, `Plane`, `Box`, `Capsule`, `Cylinder`, and `Ellipsoid` are primitives.
    #[must_use]
    pub const fn is_primitive(&self) -> bool {
        matches!(
            self,
            Self::Sphere { .. }
                | Self::Plane { .. }
                | Self::Box { .. }
                | Self::Capsule { .. }
                | Self::Cylinder { .. }
                | Self::Ellipsoid { .. }
        )
    }

    /// Returns `true` if this shape has infinite extent.
    ///
    /// Only [`Shape::Plane`] is infinite.
    #[must_use]
    pub const fn is_infinite(&self) -> bool {
        matches!(self, Self::Plane { .. })
    }

    /// Returns `true` if this shape is mesh-based.
    ///
    /// `ConvexMesh`, `TriangleMesh`, and `Sdf` are mesh-based.
    #[must_use]
    pub const fn is_mesh_based(&self) -> bool {
        matches!(
            self,
            Self::ConvexMesh { .. } | Self::TriangleMesh { .. } | Self::Sdf { .. }
        )
    }

    /// Returns `true` if this is a [`Shape::Sphere`].
    #[must_use]
    pub const fn is_sphere(&self) -> bool {
        matches!(self, Self::Sphere { .. })
    }

    /// Returns `true` if this is a [`Shape::Plane`].
    #[must_use]
    pub const fn is_plane(&self) -> bool {
        matches!(self, Self::Plane { .. })
    }

    /// Returns `true` if this is a [`Shape::Box`].
    #[must_use]
    pub const fn is_box(&self) -> bool {
        matches!(self, Self::Box { .. })
    }

    /// Returns `true` if this is a [`Shape::Capsule`].
    #[must_use]
    pub const fn is_capsule(&self) -> bool {
        matches!(self, Self::Capsule { .. })
    }

    /// Returns `true` if this is a [`Shape::Cylinder`].
    #[must_use]
    pub const fn is_cylinder(&self) -> bool {
        matches!(self, Self::Cylinder { .. })
    }
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

impl Shape {
    /// Returns the radius if this shape has one (Sphere, Capsule, Cylinder).
    #[must_use]
    pub const fn radius(&self) -> Option<f64> {
        match self {
            Self::Sphere { radius }
            | Self::Capsule { radius, .. }
            | Self::Cylinder { radius, .. } => Some(*radius),
            _ => None,
        }
    }

    /// Returns the half-length if this shape has one (Capsule, Cylinder).
    #[must_use]
    pub const fn half_length(&self) -> Option<f64> {
        match self {
            Self::Capsule { half_length, .. } | Self::Cylinder { half_length, .. } => {
                Some(*half_length)
            }
            _ => None,
        }
    }

    /// Returns the half-extents if this is a [`Shape::Box`].
    #[must_use]
    pub const fn half_extents(&self) -> Option<Vector3<f64>> {
        match self {
            Self::Box { half_extents } => Some(*half_extents),
            _ => None,
        }
    }
}

// ---------------------------------------------------------------------------
// Bounding volume
// ---------------------------------------------------------------------------

impl Shape {
    /// Maximum distance from the local origin to the shape surface.
    ///
    /// Used for broad-phase culling and sphere-based bounding approximations.
    /// Returns [`f64::INFINITY`] for planes (infinite extent).
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
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
            Self::ConvexMesh { hull } => hull
                .vertices
                .iter()
                .map(|v| v.coords.norm())
                .fold(0.0, f64::max),
            Self::HeightField { data } => {
                let half_width = (data.width() as f64).mul_add(data.cell_size(), 0.0) * 0.5;
                let half_depth = (data.depth() as f64).mul_add(data.cell_size(), 0.0) * 0.5;
                let max_height = data.max_height().abs().max(data.min_height().abs());
                half_width
                    .mul_add(
                        half_width,
                        half_depth.mul_add(half_depth, max_height.powi(2)),
                    )
                    .sqrt()
            }
            Self::Sdf { data } => {
                let origin = data.origin();
                let max_corner = Point3::new(
                    (data.width() as f64).mul_add(data.cell_size(), origin.x),
                    (data.height() as f64).mul_add(data.cell_size(), origin.y),
                    (data.depth() as f64).mul_add(data.cell_size(), origin.z),
                );
                origin.coords.norm().max(max_corner.coords.norm())
            }
            Self::TriangleMesh { mesh, .. } => mesh
                .vertices
                .iter()
                .map(|v| v.coords.norm())
                .fold(0.0, f64::max),
        }
    }
}

// ---------------------------------------------------------------------------
// Bounded impl
// ---------------------------------------------------------------------------

impl Bounded for Shape {
    /// Returns the local-space axis-aligned bounding box for this shape.
    ///
    /// The AABB is computed in the shape's local coordinate frame (centered
    /// at origin). For planes, returns an infinite AABB.
    #[allow(clippy::cast_precision_loss)]
    fn aabb(&self) -> Aabb {
        match self {
            Self::Sphere { radius } => {
                let r = *radius;
                Aabb::new(Point3::new(-r, -r, -r), Point3::new(r, r, r))
            }
            Self::Plane { .. } => Aabb::new(
                Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
                Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
            ),
            Self::Box { half_extents } => {
                Aabb::new(Point3::from(-*half_extents), Point3::from(*half_extents))
            }
            Self::Capsule {
                half_length,
                radius,
            } => {
                let r = *radius;
                let h = *half_length;
                Aabb::new(Point3::new(-r, -r, -(h + r)), Point3::new(r, r, h + r))
            }
            Self::Cylinder {
                half_length,
                radius,
            } => {
                let r = *radius;
                let h = *half_length;
                Aabb::new(Point3::new(-r, -r, -h), Point3::new(r, r, h))
            }
            Self::Ellipsoid { radii } => Aabb::new(Point3::from(-*radii), Point3::from(*radii)),
            Self::ConvexMesh { hull } => hull.aabb(),
            Self::TriangleMesh { mesh, .. } => mesh.aabb(),
            Self::HeightField { data } => data.aabb(),
            Self::Sdf { data } => data.aabb(),
        }
    }
}
