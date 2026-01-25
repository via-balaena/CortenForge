//! Bevy resources for physics visualization.

use bevy::prelude::*;
use sim_contact::ContactPoint;
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

/// Cached contacts from the physics simulation.
///
/// This resource stores contact points detected during the last physics step,
/// avoiding the need to re-detect contacts (or clone the world) for visualization.
///
/// # Usage
///
/// After stepping the simulation, call [`CachedContacts::update`] with the detected
/// contacts. The gizmo systems will read from this cache automatically.
///
/// ```ignore
/// fn step_physics(
///     mut sim_handle: ResMut<SimulationHandle>,
///     mut cached_contacts: ResMut<CachedContacts>,
/// ) {
///     if let Some(world) = sim_handle.world_mut() {
///         // Step physics and cache contacts
///         let contacts = world.detect_contacts();
///         cached_contacts.update(contacts);
///         world.solve_contacts();
///     }
/// }
/// ```
#[derive(Resource, Default)]
pub struct CachedContacts {
    contacts: Vec<ContactPoint>,
}

impl CachedContacts {
    /// Create a new empty contact cache.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Update the cache with new contacts.
    pub fn update(&mut self, contacts: Vec<ContactPoint>) {
        self.contacts = contacts;
    }

    /// Get the cached contacts.
    #[must_use]
    pub fn contacts(&self) -> &[ContactPoint] {
        &self.contacts
    }

    /// Clear the cache.
    pub fn clear(&mut self) {
        self.contacts.clear();
    }

    /// Check if the cache is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.contacts.is_empty()
    }

    /// Get the number of cached contacts.
    #[must_use]
    pub fn len(&self) -> usize {
        self.contacts.len()
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
    /// Whether to show muscle visualization.
    pub show_muscles: bool,
    /// Whether to show tendon paths.
    pub show_tendons: bool,
    /// Whether to show sensor visualization.
    pub show_sensors: bool,
    /// Scale factor for force vectors (units per Newton).
    pub force_scale: f32,
    /// Scale factor for velocity vectors (units per m/s).
    pub velocity_scale: f32,
    /// Radius of contact point markers.
    pub contact_marker_radius: f32,
    /// Length of contact normal arrows.
    pub contact_normal_length: f32,
    /// Length of joint axis arrows.
    pub joint_axis_length: f32,
    /// Radius of joint limit arcs.
    pub joint_limit_arc_radius: f32,
    /// Number of segments for joint limit arcs.
    pub joint_limit_arc_segments: u32,
    /// Radius of joint marker spheres.
    pub joint_marker_radius: f32,
    /// Minimum force magnitude to display (filters noise).
    pub force_display_threshold: f32,
    /// Minimum velocity magnitude to display (filters noise).
    pub velocity_display_threshold: f32,
    /// Minimum distance between joint bodies to draw connecting line.
    pub joint_line_distance_threshold: f32,
    /// Muscle visualization line width (as radius).
    pub muscle_line_radius: f32,
    /// Tendon visualization line width (as radius).
    pub tendon_line_radius: f32,
    /// Scale factor for sensor force arrows.
    pub sensor_force_scale: f32,
    /// Length of sensor coordinate frame axes.
    pub sensor_axis_length: f32,
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
            show_muscles: false,
            show_tendons: false,
            show_sensors: false,
            force_scale: 0.01,
            velocity_scale: 0.1,
            contact_marker_radius: 0.02,
            contact_normal_length: 0.15,
            joint_axis_length: 0.2,
            joint_limit_arc_radius: 0.1,
            joint_limit_arc_segments: 16,
            joint_marker_radius: 0.02,
            force_display_threshold: 0.01,
            velocity_display_threshold: 0.01,
            joint_line_distance_threshold: 0.001,
            muscle_line_radius: 0.01,
            tendon_line_radius: 0.005,
            sensor_force_scale: 0.01,
            sensor_axis_length: 0.1,
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
    /// Color for relaxed muscle (activation = 0).
    pub muscle_relaxed: Color,
    /// Color for fully activated muscle (activation = 1).
    pub muscle_activated: Color,
    /// Color for tendons.
    pub tendon: Color,
    /// Color for IMU sensor frames.
    pub sensor_imu: Color,
    /// Color for force/torque sensors.
    pub sensor_force_torque: Color,
    /// Color for touch sensors when active.
    pub sensor_touch_active: Color,
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
            // Muscles: blue (relaxed) to red (activated)
            muscle_relaxed: Color::srgb(0.2, 0.4, 0.8),
            muscle_activated: Color::srgb(1.0, 0.2, 0.2),
            // Tendons: cyan/teal
            tendon: Color::srgb(0.2, 0.8, 0.8),
            // Sensors
            sensor_imu: Color::srgb(0.8, 0.8, 0.2),
            sensor_force_torque: Color::srgb(0.8, 0.4, 0.8),
            sensor_touch_active: Color::srgb(1.0, 0.6, 0.2),
        }
    }
}

// ============================================================================
// Musculoskeletal Visualization Resources
// ============================================================================

/// A muscle instance for visualization.
///
/// Since muscles are not stored in the physics World (they're standalone
/// actuators), users must provide visualization data via this resource.
#[derive(Debug, Clone)]
pub struct MuscleVisualData {
    /// Name of the muscle (for display).
    pub name: String,
    /// Origin attachment point in world coordinates.
    pub origin: nalgebra::Point3<f64>,
    /// Insertion attachment point in world coordinates.
    pub insertion: nalgebra::Point3<f64>,
    /// Current activation level (0.0 = relaxed, 1.0 = fully activated).
    pub activation: f64,
    /// Optional via points for curved muscle paths.
    pub via_points: Vec<nalgebra::Point3<f64>>,
    /// Optional: current muscle force (for force vector display).
    pub force: Option<f64>,
}

impl MuscleVisualData {
    /// Create a new muscle visualization with straight path.
    #[must_use]
    pub fn new(
        name: impl Into<String>,
        origin: nalgebra::Point3<f64>,
        insertion: nalgebra::Point3<f64>,
    ) -> Self {
        Self {
            name: name.into(),
            origin,
            insertion,
            activation: 0.0,
            via_points: Vec::new(),
            force: None,
        }
    }

    /// Set the activation level.
    #[must_use]
    pub fn with_activation(mut self, activation: f64) -> Self {
        self.activation = activation.clamp(0.0, 1.0);
        self
    }

    /// Add via points for curved muscle paths.
    #[must_use]
    pub fn with_via_points(mut self, points: Vec<nalgebra::Point3<f64>>) -> Self {
        self.via_points = points;
        self
    }

    /// Set the current muscle force.
    #[must_use]
    pub fn with_force(mut self, force: f64) -> Self {
        self.force = Some(force);
        self
    }
}

/// Resource containing muscle visualization data.
///
/// Users update this resource each frame with current muscle state.
///
/// # Example
///
/// ```ignore
/// fn update_muscle_visuals(
///     mut muscles_vis: ResMut<MuscleVisualization>,
///     my_muscles: Res<MyMuscleState>,
/// ) {
///     muscles_vis.clear();
///     for muscle in &my_muscles.muscles {
///         muscles_vis.add(MuscleVisualData::new(
///             &muscle.name,
///             muscle.origin_world_pos(),
///             muscle.insertion_world_pos(),
///         ).with_activation(muscle.activation()));
///     }
/// }
/// ```
#[derive(Resource, Default)]
pub struct MuscleVisualization {
    muscles: Vec<MuscleVisualData>,
}

impl MuscleVisualization {
    /// Create a new empty muscle visualization.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a muscle to visualize.
    pub fn add(&mut self, muscle: MuscleVisualData) {
        self.muscles.push(muscle);
    }

    /// Get all muscles for visualization.
    #[must_use]
    pub fn muscles(&self) -> &[MuscleVisualData] {
        &self.muscles
    }

    /// Clear all muscles (call before updating each frame).
    pub fn clear(&mut self) {
        self.muscles.clear();
    }

    /// Check if empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.muscles.is_empty()
    }

    /// Get the number of muscles.
    #[must_use]
    pub fn len(&self) -> usize {
        self.muscles.len()
    }
}

/// A tendon instance for visualization.
#[derive(Debug, Clone)]
pub struct TendonVisualData {
    /// Name of the tendon.
    pub name: String,
    /// Path points in world coordinates (in order from origin to insertion).
    pub path: Vec<nalgebra::Point3<f64>>,
    /// Current tension (for color/thickness scaling).
    pub tension: f64,
    /// Current length.
    pub length: f64,
    /// Rest length (for stretch visualization).
    pub rest_length: f64,
}

impl TendonVisualData {
    /// Create a new tendon visualization.
    #[must_use]
    pub fn new(name: impl Into<String>, path: Vec<nalgebra::Point3<f64>>) -> Self {
        Self {
            name: name.into(),
            path,
            tension: 0.0,
            length: 0.0,
            rest_length: 0.0,
        }
    }

    /// Set the tension.
    #[must_use]
    pub fn with_tension(mut self, tension: f64) -> Self {
        self.tension = tension;
        self
    }

    /// Set length properties.
    #[must_use]
    pub fn with_length(mut self, current: f64, rest: f64) -> Self {
        self.length = current;
        self.rest_length = rest;
        self
    }
}

/// Resource containing tendon visualization data.
#[derive(Resource, Default)]
pub struct TendonVisualization {
    tendons: Vec<TendonVisualData>,
}

impl TendonVisualization {
    /// Create a new empty tendon visualization.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a tendon to visualize.
    pub fn add(&mut self, tendon: TendonVisualData) {
        self.tendons.push(tendon);
    }

    /// Get all tendons for visualization.
    #[must_use]
    pub fn tendons(&self) -> &[TendonVisualData] {
        &self.tendons
    }

    /// Clear all tendons.
    pub fn clear(&mut self) {
        self.tendons.clear();
    }

    /// Check if empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.tendons.is_empty()
    }

    /// Get the number of tendons.
    #[must_use]
    pub fn len(&self) -> usize {
        self.tendons.len()
    }
}

/// Type of sensor for visualization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorVisualType {
    /// IMU sensor (shows coordinate frame).
    Imu,
    /// Force/torque sensor (shows force/torque arrows).
    ForceTorque,
    /// Touch sensor (shows contact highlight).
    Touch,
    /// Rangefinder (shows ray).
    Rangefinder,
    /// Magnetometer (shows field direction).
    Magnetometer,
}

/// A sensor instance for visualization.
#[derive(Debug, Clone)]
pub struct SensorVisualData {
    /// Sensor type.
    pub sensor_type: SensorVisualType,
    /// Position in world coordinates.
    pub position: nalgebra::Point3<f64>,
    /// Orientation in world coordinates.
    pub orientation: nalgebra::UnitQuaternion<f64>,
    /// Measured force (for force/torque sensors).
    pub force: Option<nalgebra::Vector3<f64>>,
    /// Measured torque (for force/torque sensors).
    pub torque: Option<nalgebra::Vector3<f64>>,
    /// Whether sensor is active/triggered (for touch sensors).
    pub is_active: bool,
    /// Ray direction and distance (for rangefinders).
    pub ray: Option<(nalgebra::Vector3<f64>, f64)>,
    /// Magnetic field direction (for magnetometers).
    pub magnetic_field: Option<nalgebra::Vector3<f64>>,
}

impl SensorVisualData {
    /// Create an IMU sensor visualization.
    #[must_use]
    pub fn imu(
        position: nalgebra::Point3<f64>,
        orientation: nalgebra::UnitQuaternion<f64>,
    ) -> Self {
        Self {
            sensor_type: SensorVisualType::Imu,
            position,
            orientation,
            force: None,
            torque: None,
            is_active: false,
            ray: None,
            magnetic_field: None,
        }
    }

    /// Create a force/torque sensor visualization.
    #[must_use]
    pub fn force_torque(
        position: nalgebra::Point3<f64>,
        orientation: nalgebra::UnitQuaternion<f64>,
        force: nalgebra::Vector3<f64>,
        torque: nalgebra::Vector3<f64>,
    ) -> Self {
        Self {
            sensor_type: SensorVisualType::ForceTorque,
            position,
            orientation,
            force: Some(force),
            torque: Some(torque),
            is_active: false,
            ray: None,
            magnetic_field: None,
        }
    }

    /// Create a touch sensor visualization.
    #[must_use]
    pub fn touch(position: nalgebra::Point3<f64>, is_active: bool) -> Self {
        Self {
            sensor_type: SensorVisualType::Touch,
            position,
            orientation: nalgebra::UnitQuaternion::identity(),
            force: None,
            torque: None,
            is_active,
            ray: None,
            magnetic_field: None,
        }
    }

    /// Create a rangefinder visualization.
    #[must_use]
    pub fn rangefinder(
        position: nalgebra::Point3<f64>,
        direction: nalgebra::Vector3<f64>,
        distance: f64,
    ) -> Self {
        Self {
            sensor_type: SensorVisualType::Rangefinder,
            position,
            orientation: nalgebra::UnitQuaternion::identity(),
            force: None,
            torque: None,
            is_active: false,
            ray: Some((direction, distance)),
            magnetic_field: None,
        }
    }

    /// Create a magnetometer visualization.
    #[must_use]
    pub fn magnetometer(
        position: nalgebra::Point3<f64>,
        magnetic_field: nalgebra::Vector3<f64>,
    ) -> Self {
        Self {
            sensor_type: SensorVisualType::Magnetometer,
            position,
            orientation: nalgebra::UnitQuaternion::identity(),
            force: None,
            torque: None,
            is_active: false,
            ray: None,
            magnetic_field: Some(magnetic_field),
        }
    }
}

/// Resource containing sensor visualization data.
#[derive(Resource, Default)]
pub struct SensorVisualization {
    sensors: Vec<SensorVisualData>,
}

impl SensorVisualization {
    /// Create a new empty sensor visualization.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a sensor to visualize.
    pub fn add(&mut self, sensor: SensorVisualData) {
        self.sensors.push(sensor);
    }

    /// Get all sensors for visualization.
    #[must_use]
    pub fn sensors(&self) -> &[SensorVisualData] {
        &self.sensors
    }

    /// Clear all sensors.
    pub fn clear(&mut self) {
        self.sensors.clear();
    }

    /// Check if empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.sensors.is_empty()
    }

    /// Get the number of sensors.
    #[must_use]
    pub fn len(&self) -> usize {
        self.sensors.len()
    }
}
