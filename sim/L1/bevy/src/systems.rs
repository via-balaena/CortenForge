//! ECS systems for physics visualization.
//!
//! These systems synchronize sim-core physics state with Bevy entities.

use bevy::prelude::*;
use std::collections::HashSet;

use crate::components::{CollisionShapeVisual, PhysicsBody, ShapeType};
use crate::convert::{quat_from_unit_quaternion, vec3_from_point};
use crate::mesh::mesh_from_collision_shape;
use crate::resources::{BodyEntityMap, SimulationHandle, ViewerConfig};

/// Synchronizes Bevy entities with sim-core bodies.
///
/// This system runs in `Update` and handles:
/// - Spawning entities for new bodies
/// - Despawning entities for removed bodies
///
/// It maintains the [`BodyEntityMap`] resource for O(1) lookups.
#[allow(clippy::needless_pass_by_value)] // Bevy system parameters are passed by value
pub fn sync_physics_entities(
    mut commands: Commands,
    sim_world: Res<SimulationHandle>,
    mut body_map: ResMut<BodyEntityMap>,
    config: Res<ViewerConfig>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let Some(world) = sim_world.world() else {
        return;
    };

    // Collect current body IDs from sim-core
    let current_bodies: HashSet<_> = world.body_ids().collect();
    let tracked_bodies: HashSet<_> = body_map.body_ids().collect();

    // Spawn new bodies
    for body_id in current_bodies.difference(&tracked_bodies) {
        let Some(body) = world.body(*body_id) else {
            continue;
        };

        // Determine material based on body type
        let material = if body.is_static {
            materials.add(StandardMaterial {
                base_color: config.colors.static_body,
                ..default()
            })
        } else {
            materials.add(StandardMaterial {
                base_color: config.colors.dynamic_body,
                ..default()
            })
        };

        // Create mesh from collision shape if available
        let mesh_handle = body
            .collision_shape
            .as_ref()
            .and_then(mesh_from_collision_shape)
            .map(|mesh| meshes.add(mesh));

        // Calculate initial transform
        let pose = &body.state.pose;
        let transform = Transform {
            translation: vec3_from_point(&pose.position),
            rotation: quat_from_unit_quaternion(&pose.rotation),
            scale: Vec3::ONE,
        };

        // Spawn body entity (transform synced with physics)
        let body_entity = commands
            .spawn((
                PhysicsBody::new(*body_id),
                transform,
                Visibility::default(),
                InheritedVisibility::default(),
                ViewVisibility::default(),
                GlobalTransform::default(),
            ))
            .id();

        // Add mesh as child entity with local shape pose offset
        if let Some(mesh) = mesh_handle {
            // Compute the shape's local transform (offset from body frame)
            let shape_transform = if let Some(ref shape_pose) = body.collision_shape_pose {
                Transform {
                    translation: vec3_from_point(&shape_pose.position),
                    rotation: quat_from_unit_quaternion(&shape_pose.rotation),
                    scale: Vec3::ONE,
                }
            } else {
                Transform::IDENTITY
            };

            // Spawn mesh as child with local offset
            let mesh_entity = commands
                .spawn((
                    Mesh3d(mesh),
                    MeshMaterial3d(material),
                    shape_transform,
                    Visibility::default(),
                    InheritedVisibility::default(),
                    ViewVisibility::default(),
                    GlobalTransform::default(),
                    CollisionShapeVisual::new(
                        body.collision_shape
                            .as_ref()
                            .map_or(ShapeType::Box, ShapeType::from_collision_shape),
                    ),
                ))
                .id();

            commands.entity(body_entity).add_child(mesh_entity);
        }

        body_map.insert(*body_id, body_entity);
    }

    // Despawn removed bodies
    for body_id in tracked_bodies.difference(&current_bodies) {
        if let Some(entity) = body_map.remove_by_body(*body_id) {
            commands.entity(entity).despawn();
        }
    }
}

/// Synchronizes Bevy transforms with sim-core body poses.
///
/// This system runs in `PostUpdate` to ensure physics has stepped
/// before transforms are updated for rendering.
#[allow(clippy::needless_pass_by_value)] // Bevy system parameters are passed by value
pub fn sync_body_transforms(
    sim_world: Res<SimulationHandle>,
    mut bodies: Query<(&PhysicsBody, &mut Transform)>,
) {
    let Some(world) = sim_world.world() else {
        return;
    };

    for (physics_body, mut transform) in &mut bodies {
        let Some(body) = world.body(physics_body.body_id) else {
            continue;
        };

        let pose = &body.state.pose;
        transform.translation = vec3_from_point(&pose.position);
        transform.rotation = quat_from_unit_quaternion(&pose.rotation);
    }
}

/// Updates collision shape visibility based on configuration.
#[allow(clippy::needless_pass_by_value)] // Bevy system parameters are passed by value
pub fn update_shape_visibility(
    config: Res<ViewerConfig>,
    mut shapes: Query<&mut Visibility, With<CollisionShapeVisual>>,
) {
    let visibility = if config.show_collision_shapes {
        Visibility::Inherited
    } else {
        Visibility::Hidden
    };

    for mut vis in &mut shapes {
        *vis = visibility;
    }
}

/// System set for physics visualization.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum SimViewerSet {
    /// Entity lifecycle management (spawn/despawn).
    EntitySync,
    /// Transform synchronization.
    TransformSync,
    /// Debug visualization.
    DebugRender,
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::World;
    use sim_types::{MassProperties, Pose, RigidBodyState};

    fn create_test_world() -> World {
        let mut world = World::default();

        // Add a dynamic body
        let pose = Pose::from_position(nalgebra::Point3::new(0.0, 5.0, 0.0));
        let state = RigidBodyState::at_rest(pose);
        let mass = MassProperties::sphere(1.0, 0.5);
        world.add_body(state, mass);

        world
    }

    #[test]
    fn body_entity_map_insert_and_lookup() {
        let mut map = BodyEntityMap::default();
        let body_id = sim_types::BodyId::new(1);
        let entity = Entity::from_bits(42);

        map.insert(body_id, entity);

        assert_eq!(map.get_entity(body_id), Some(entity));
        assert_eq!(map.get_body(entity), Some(body_id));
        assert!(map.contains_body(body_id));
        assert_eq!(map.len(), 1);
    }

    #[test]
    fn body_entity_map_remove() {
        let mut map = BodyEntityMap::default();
        let body_id = sim_types::BodyId::new(1);
        let entity = Entity::from_bits(42);

        map.insert(body_id, entity);
        let removed = map.remove_by_body(body_id);

        assert_eq!(removed, Some(entity));
        assert!(map.is_empty());
        assert_eq!(map.get_entity(body_id), None);
        assert_eq!(map.get_body(entity), None);
    }

    #[test]
    fn simulation_handle_world_access() {
        let world = create_test_world();
        let mut handle = SimulationHandle::new(world);

        assert!(handle.has_world());
        assert!(handle.world().is_some());
        assert!(handle.world_mut().is_some());

        let taken = handle.take_world();
        assert!(taken.is_some());
        assert!(!handle.has_world());
    }
}
