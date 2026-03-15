//! ECS components for physics visualization.
//!
//! These components link Bevy entities to sim-core physics objects.
//! For Model/Data components, see [`crate::model_data`].

use bevy::prelude::*;

/// Visual representation of a collision shape.
///
/// This component is added to child entities to render collision geometry.
/// Used by the Model/Data API.
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

/// Visualization group (0–5), matching `MuJoCo` semantics.
///
/// Entities with this component can be filtered by group in [`ViewerConfig`](crate::resources::ViewerConfig).
/// Spawners should read `model.geom_group[i]` and insert `VisGroup(group)` on geom entities.
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VisGroup(pub i32);

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
    /// Convert from a cf-geometry [`Shape`](cf_geometry::Shape) to `ShapeType`.
    #[must_use]
    pub fn from_shape(shape: &cf_geometry::Shape) -> Self {
        match shape {
            cf_geometry::Shape::Sphere { .. } => Self::Sphere,
            cf_geometry::Shape::Box { .. } => Self::Box,
            cf_geometry::Shape::Capsule { .. } => Self::Capsule,
            cf_geometry::Shape::Cylinder { .. } => Self::Cylinder,
            cf_geometry::Shape::Ellipsoid { .. } => Self::Ellipsoid,
            cf_geometry::Shape::Plane { .. } => Self::Plane,
            cf_geometry::Shape::ConvexMesh { .. } => Self::ConvexMesh,
            cf_geometry::Shape::TriangleMesh { .. } => Self::TriangleMesh,
            cf_geometry::Shape::HeightField { .. } => Self::HeightField,
            cf_geometry::Shape::Sdf { .. } => Self::Sdf,
        }
    }
}
