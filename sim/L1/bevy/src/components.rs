//! ECS components for physics visualization.
//!
//! These components link Bevy entities to sim-core physics objects.
//! For Model/Data components, see [`crate::model_data`].

use bevy::prelude::*;

/// Visual representation of a collision shape.
///
/// This component is added to child entities to render collision geometry.
/// Used by both the legacy World API and the new Model/Data API.
#[derive(Component, Debug, Clone)]
pub struct CollisionShapeVisual {
    /// The shape type being visualized.
    pub shape_type: ShapeType,
    /// Whether to render as wireframe.
    pub wireframe: bool,
}

impl CollisionShapeVisual {
    /// Create a new collision shape visual.
    #[must_use]
    pub const fn new(shape_type: ShapeType) -> Self {
        Self {
            shape_type,
            wireframe: false,
        }
    }

    /// Set wireframe rendering mode.
    #[must_use]
    pub const fn with_wireframe(mut self, wireframe: bool) -> Self {
        self.wireframe = wireframe;
        self
    }
}

/// Type of collision shape being visualized.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum ShapeType {
    /// Sphere shape.
    Sphere,
    /// Box/cuboid shape.
    Box,
    /// Capsule shape (cylinder with hemispherical caps).
    Capsule,
    /// Cylinder shape.
    Cylinder,
    /// Ellipsoid shape.
    Ellipsoid,
    /// Infinite plane.
    Plane,
    /// Convex mesh.
    ConvexMesh,
    /// Triangle mesh.
    TriangleMesh,
    /// Height field terrain.
    HeightField,
    /// Signed distance field.
    Sdf,
}

impl ShapeType {
    /// Convert from a sim-core [`CollisionShape`](sim_core::CollisionShape) to `ShapeType`.
    #[must_use]
    pub fn from_collision_shape(shape: &sim_core::CollisionShape) -> Self {
        match shape {
            sim_core::CollisionShape::Sphere { .. } => Self::Sphere,
            sim_core::CollisionShape::Box { .. } => Self::Box,
            sim_core::CollisionShape::Capsule { .. } => Self::Capsule,
            sim_core::CollisionShape::Cylinder { .. } => Self::Cylinder,
            sim_core::CollisionShape::Ellipsoid { .. } => Self::Ellipsoid,
            sim_core::CollisionShape::Plane { .. } => Self::Plane,
            sim_core::CollisionShape::ConvexMesh { .. } => Self::ConvexMesh,
            sim_core::CollisionShape::TriangleMesh { .. } => Self::TriangleMesh,
            sim_core::CollisionShape::HeightField { .. } => Self::HeightField,
            sim_core::CollisionShape::Sdf { .. } => Self::Sdf,
        }
    }
}
