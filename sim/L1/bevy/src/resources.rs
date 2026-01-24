//! Bevy resources for physics visualization.

use bevy::prelude::*;
use sim_core::World;
use sim_types::BodyId;
use std::collections::HashMap;

/// Handle to the physics simulation world.
///
/// This resource wraps the sim-core [`World`] and provides access for
/// visualization systems. The user is responsible for stepping the simulation;
/// sim-bevy only reads from it.
///
/// # Example
///
/// ```ignore
/// fn setup(mut commands: Commands) {
///     let world = World::default();
///     commands.insert_resource(SimulationHandle::new(world));
/// }
///
/// fn step_physics(mut handle: ResMut<SimulationHandle>, time: Res<Time>) {
///     if let Some(world) = handle.world_mut() {
///         // User steps the simulation
///         world.step(time.delta_secs_f64());
///     }
/// }
/// ```
#[derive(Resource)]
pub struct SimulationHandle {
    world: Option<World>,
}

impl SimulationHandle {
    /// Create a new simulation handle with the given world.
    #[must_use]
    pub fn new(world: World) -> Self {
        Self { world: Some(world) }
    }

    /// Create an empty simulation handle.
    #[must_use]
    pub fn empty() -> Self {
        Self { world: None }
    }

    /// Get a reference to the physics world.
    #[must_use]
    pub fn world(&self) -> Option<&World> {
        self.world.as_ref()
    }

    /// Get a mutable reference to the physics world.
    pub fn world_mut(&mut self) -> Option<&mut World> {
        self.world.as_mut()
    }

    /// Set the physics world.
    pub fn set_world(&mut self, world: World) {
        self.world = Some(world);
    }

    /// Take the physics world, leaving None in its place.
    pub fn take_world(&mut self) -> Option<World> {
        self.world.take()
    }

    /// Check if a world is present.
    #[must_use]
    pub fn has_world(&self) -> bool {
        self.world.is_some()
    }
}

impl Default for SimulationHandle {
    fn default() -> Self {
        Self::empty()
    }
}

/// Tracks the mapping between sim-core bodies and Bevy entities.
///
/// This resource enables O(1) lookups in both directions, which is used
/// by sync systems to efficiently update transforms.
#[derive(Resource, Default)]
pub struct BodyEntityMap {
    body_to_entity: HashMap<BodyId, Entity>,
    entity_to_body: HashMap<Entity, BodyId>,
}

impl BodyEntityMap {
    /// Create a new empty map.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Insert a body-entity mapping.
    pub fn insert(&mut self, body_id: BodyId, entity: Entity) {
        self.body_to_entity.insert(body_id, entity);
        self.entity_to_body.insert(entity, body_id);
    }

    /// Remove a mapping by body ID.
    pub fn remove_by_body(&mut self, body_id: BodyId) -> Option<Entity> {
        if let Some(entity) = self.body_to_entity.remove(&body_id) {
            self.entity_to_body.remove(&entity);
            Some(entity)
        } else {
            None
        }
    }

    /// Remove a mapping by entity.
    pub fn remove_by_entity(&mut self, entity: Entity) -> Option<BodyId> {
        if let Some(body_id) = self.entity_to_body.remove(&entity) {
            self.body_to_entity.remove(&body_id);
            Some(body_id)
        } else {
            None
        }
    }

    /// Get the entity for a body ID.
    #[must_use]
    pub fn get_entity(&self, body_id: BodyId) -> Option<Entity> {
        self.body_to_entity.get(&body_id).copied()
    }

    /// Get the body ID for an entity.
    #[must_use]
    pub fn get_body(&self, entity: Entity) -> Option<BodyId> {
        self.entity_to_body.get(&entity).copied()
    }

    /// Check if a body ID is tracked.
    #[must_use]
    pub fn contains_body(&self, body_id: BodyId) -> bool {
        self.body_to_entity.contains_key(&body_id)
    }

    /// Get all tracked body IDs.
    pub fn body_ids(&self) -> impl Iterator<Item = BodyId> + '_ {
        self.body_to_entity.keys().copied()
    }

    /// Get the number of tracked bodies.
    #[must_use]
    pub fn len(&self) -> usize {
        self.body_to_entity.len()
    }

    /// Check if the map is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.body_to_entity.is_empty()
    }

    /// Clear all mappings.
    pub fn clear(&mut self) {
        self.body_to_entity.clear();
        self.entity_to_body.clear();
    }
}

/// Configuration for the physics viewer.
///
/// Controls what debug information is displayed and how it's rendered.
#[derive(Resource, Debug, Clone)]
#[allow(clippy::struct_excessive_bools)] // Config structs with multiple toggles are idiomatic
pub struct ViewerConfig {
    /// Whether to show collision shapes.
    pub show_collision_shapes: bool,
    /// Whether to show contact points.
    pub show_contacts: bool,
    /// Whether to show contact normals.
    pub show_contact_normals: bool,
    /// Whether to show applied forces.
    pub show_forces: bool,
    /// Whether to show joint axes.
    pub show_joint_axes: bool,
    /// Whether to show joint limits.
    pub show_joint_limits: bool,
    /// Whether to show velocity vectors.
    pub show_velocities: bool,
    /// Scale factor for force vectors (units per Newton).
    pub force_scale: f32,
    /// Scale factor for velocity vectors (units per m/s).
    pub velocity_scale: f32,
    /// Radius of contact point markers.
    pub contact_marker_radius: f32,
    /// Color scheme for debug visualization.
    pub colors: DebugColors,
}

impl Default for ViewerConfig {
    fn default() -> Self {
        Self {
            show_collision_shapes: true,
            show_contacts: true,
            show_contact_normals: true,
            show_forces: false,
            show_joint_axes: true,
            show_joint_limits: false,
            show_velocities: false,
            force_scale: 0.01,
            velocity_scale: 0.1,
            contact_marker_radius: 0.02,
            colors: DebugColors::default(),
        }
    }
}

/// Color scheme for debug visualization.
#[derive(Debug, Clone)]
pub struct DebugColors {
    /// Color for contact points.
    pub contact_point: Color,
    /// Color for contact normals.
    pub contact_normal: Color,
    /// Color for force vectors.
    pub force_vector: Color,
    /// Color for joint axes.
    pub joint_axis: Color,
    /// Color for joint limits.
    pub joint_limit: Color,
    /// Color for linear velocity vectors.
    pub linear_velocity: Color,
    /// Color for angular velocity vectors.
    pub angular_velocity: Color,
    /// Color for static bodies.
    pub static_body: Color,
    /// Color for dynamic bodies.
    pub dynamic_body: Color,
}

impl Default for DebugColors {
    fn default() -> Self {
        Self {
            contact_point: Color::srgb(1.0, 0.2, 0.2),
            contact_normal: Color::srgb(0.2, 1.0, 0.2),
            force_vector: Color::srgb(1.0, 1.0, 0.2),
            joint_axis: Color::srgb(0.2, 0.6, 1.0),
            joint_limit: Color::srgb(1.0, 0.5, 0.0),
            linear_velocity: Color::srgb(0.2, 0.8, 1.0),
            angular_velocity: Color::srgb(1.0, 0.2, 0.8),
            static_body: Color::srgb(0.5, 0.5, 0.5),
            dynamic_body: Color::srgb(0.8, 0.6, 0.4),
        }
    }
}
