//! ECS components for physics visualization.
//!
//! These components link Bevy entities to sim-core physics objects.

#![allow(deprecated)] // Allow deprecated types within this module

use bevy::prelude::*;
use sim_types::BodyId;

/// Links a Bevy entity to a body in the physics simulation.
///
/// **Deprecated**: Use [`ModelBodyIndex`](crate::model_data::ModelBodyIndex) instead,
/// which works with the Model/Data architecture.
///
/// The transform of this entity will be synchronized with the body's pose
/// each frame by the [`sync_body_transforms`](crate::systems::sync_body_transforms) system.
#[deprecated(
    since = "0.8.0",
    note = "Use ModelBodyIndex from model_data module instead. This component works with the deprecated World API."
)]
#[derive(Component, Debug, Clone, Copy)]
pub struct PhysicsBody {
    /// The body ID in the sim-core World.
    pub body_id: BodyId,
}

#[allow(deprecated)]
impl PhysicsBody {
    /// Create a new physics body link.
    #[must_use]
    pub const fn new(body_id: BodyId) -> Self {
        Self { body_id }
    }
}

/// Marker for the root entity of a spawned physics world.
///
/// **Deprecated**: Use [`ModelDataRoot`](crate::model_data::ModelDataRoot) instead.
///
/// Used to identify the parent entity under which all physics bodies are spawned.
#[deprecated(
    since = "0.8.0",
    note = "Use ModelDataRoot from model_data module instead. This component works with the deprecated World API."
)]
#[derive(Component, Debug, Default, Clone, Copy)]
pub struct PhysicsWorldRoot;

/// Visual representation of a collision shape.
///
/// This component is added to child entities of [`PhysicsBody`] entities
/// to render their collision geometry.
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
    /// Convert from a sim-core world [`CollisionShape`](sim_core::world::CollisionShape) to `ShapeType`.
    #[must_use]
    pub fn from_collision_shape(shape: &sim_core::world::CollisionShape) -> Self {
        match shape {
            sim_core::world::CollisionShape::Sphere { .. } => Self::Sphere,
            sim_core::world::CollisionShape::Box { .. } => Self::Box,
            sim_core::world::CollisionShape::Capsule { .. } => Self::Capsule,
            sim_core::world::CollisionShape::Cylinder { .. } => Self::Cylinder,
            sim_core::world::CollisionShape::Ellipsoid { .. } => Self::Ellipsoid,
            sim_core::world::CollisionShape::Plane { .. } => Self::Plane,
            sim_core::world::CollisionShape::ConvexMesh { .. } => Self::ConvexMesh,
            sim_core::world::CollisionShape::TriangleMesh { .. } => Self::TriangleMesh,
            sim_core::world::CollisionShape::HeightField { .. } => Self::HeightField,
            sim_core::world::CollisionShape::Sdf { .. } => Self::Sdf,
        }
    }
}
