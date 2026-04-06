//! Bevy integration for `MuJoCo`-style Model/Data architecture.
//!
//! This module provides resources and systems for the new Model/Data physics
//! pipeline, which follows `MuJoCo`'s architecture:
//!
//! - [`Model`] is static (immutable after loading)
//! - [`Data`] is dynamic (qpos/qvel are the source of truth)
//! - Body poses (xpos/xquat) are computed via forward kinematics
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                     Bevy ECS Resources                          │
//! │                                                                 │
//! │  ┌─────────────────┐          ┌─────────────────────────────┐  │
//! │  │  PhysicsModel   │          │      PhysicsData            │  │
//! │  │  (Immutable)    │          │  (qpos, qvel → xpos, xquat) │  │
//! │  └────────┬────────┘          └──────────────┬──────────────┘  │
//! │           │                                   │                 │
//! └───────────┼───────────────────────────────────┼─────────────────┘
//!             │ step_model_data                   │ sync_model_data_to_bevy
//!             ▼                                   ▼
//!        Physics Step                      Bevy Transforms
//!      (forward dynamics)                (body visualization)
//! ```
//!
//! # Usage
//!
//! ```ignore
//! use bevy::prelude::*;
//! use sim_bevy::prelude::*;
//!
//! fn main() {
//!     let model = load_model(include_str!("robot.xml")).unwrap();
//!     let data = model.make_data();
//!
//!     App::new()
//!         .add_plugins(DefaultPlugins)
//!         .insert_resource(PhysicsModel(model))
//!         .insert_resource(PhysicsData(data))
//!         .add_systems(Update, step_model_data)
//!         .add_systems(PostUpdate, sync_model_data_to_bevy)
//!         .run();
//! }
//! ```

use std::sync::Arc;

use bevy::prelude::*;
use cf_geometry::Shape;
use sim_core::{Data, GeomType, Model};

use crate::convert::{quat_from_physics_matrix, quat_from_unit_quaternion, vec3_from_vector};
use crate::mesh::{mesh_from_shape, triangle_mesh_from_indexed};

// ============================================================================
// Resources
// ============================================================================

/// Bevy resource wrapping the static physics model.
///
/// The model is immutable after construction - it contains the kinematic tree,
/// joint definitions, collision geometries, and other static configuration.
///
/// # Example
///
/// ```ignore
/// use sim_bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     let model = load_model(include_str!("robot.xml")).unwrap();
///     commands.insert_resource(PhysicsModel(model));
/// }
/// ```
#[derive(Resource)]
pub struct PhysicsModel(pub Model);

impl PhysicsModel {
    /// Create a new physics model resource.
    #[must_use]
    pub fn new(model: Model) -> Self {
        Self(model)
    }

    /// Get a reference to the underlying model.
    #[must_use]
    pub fn model(&self) -> &Model {
        &self.0
    }
}

impl std::ops::Deref for PhysicsModel {
    type Target = Model;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

/// Bevy resource wrapping the dynamic physics data.
///
/// The data contains the current simulation state (qpos, qvel) and all
/// computed quantities (body poses, forces, contacts). The key invariant
/// is that `qpos`/`qvel` are the ONLY state variables - everything else
/// is computed from them via forward dynamics.
///
/// # Example
///
/// ```ignore
/// fn setup(model: Res<PhysicsModel>, mut commands: Commands) {
///     let data = model.make_data();
///     commands.insert_resource(PhysicsData(data));
/// }
///
/// fn step(model: Res<PhysicsModel>, mut data: ResMut<PhysicsData>) {
///     data.step(&model).expect("step failed");
/// }
/// ```
#[derive(Resource)]
pub struct PhysicsData(pub Data);

impl PhysicsData {
    /// Create a new physics data resource.
    #[must_use]
    pub fn new(data: Data) -> Self {
        Self(data)
    }

    /// Get a reference to the underlying data.
    #[must_use]
    pub fn data(&self) -> &Data {
        &self.0
    }

    /// Get a mutable reference to the underlying data.
    pub fn data_mut(&mut self) -> &mut Data {
        &mut self.0
    }
}

impl std::ops::Deref for PhysicsData {
    type Target = Data;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl std::ops::DerefMut for PhysicsData {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

// ============================================================================
// Components
// ============================================================================

/// Component linking a Bevy entity to a body in the Model/Data system.
///
/// This component uses a direct index into the Model's body arrays.
///
/// # Example
///
/// ```ignore
/// // Spawn entity for body 1 (index 0 is world)
/// commands.spawn((
///     ModelBodyIndex(1),
///     Transform::default(),
/// ));
/// ```
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ModelBodyIndex(pub usize);

impl ModelBodyIndex {
    /// Create a new body index component.
    #[must_use]
    pub const fn new(index: usize) -> Self {
        Self(index)
    }

    /// Get the body index.
    #[must_use]
    pub const fn index(self) -> usize {
        self.0
    }
}

/// Component linking a Bevy entity to a geom in the Model/Data system.
///
/// This is used for entities representing collision geometries.
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ModelGeomIndex(pub usize);

impl ModelGeomIndex {
    /// Create a new geom index component.
    #[must_use]
    pub const fn new(index: usize) -> Self {
        Self(index)
    }

    /// Get the geom index.
    #[must_use]
    pub const fn index(self) -> usize {
        self.0
    }
}

/// Component linking a Bevy entity to a site in the Model/Data system.
///
/// Sites are attachment points for sensors, actuators, etc.
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ModelSiteIndex(pub usize);

impl ModelSiteIndex {
    /// Create a new site index component.
    #[must_use]
    pub const fn new(index: usize) -> Self {
        Self(index)
    }

    /// Get the site index.
    #[must_use]
    pub const fn index(self) -> usize {
        self.0
    }
}

/// Marker component for the root entity of a Model/Data physics scene.
#[derive(Component, Debug, Default, Clone, Copy)]
pub struct ModelDataRoot;

// ============================================================================
// Systems
// ============================================================================

/// Step the physics simulation forward by one timestep.
///
/// This system calls `Data::step()` which performs:
/// 1. Forward kinematics (qpos → xpos, xquat)
/// 2. Mass matrix computation (CRBA)
/// 3. Bias force computation (RNE)
/// 4. Constraint solving (contacts, limits)
/// 5. Integration (semi-implicit Euler)
///
/// # System Ordering
///
/// This should run in `Update` before transform synchronization:
///
/// ```ignore
/// app.add_systems(Update, step_model_data);
/// app.add_systems(PostUpdate, sync_model_data_to_bevy);
/// ```
#[allow(clippy::needless_pass_by_value)] // Bevy system parameters
pub fn step_model_data(model: Res<PhysicsModel>, mut data: ResMut<PhysicsData>) {
    if let Err(e) = data.0.step(&model.0) {
        eprintln!("Physics step failed: {e}");
        return;
    }
    // Refresh derived quantities (sensors, geom/body poses, energy) so they
    // match the post-integration qpos/qvel. Without this, these fields lag
    // by one timestep — the MuJoCo pipeline computes them BEFORE integration.
    if let Err(e) = data.0.forward(&model.0) {
        eprintln!("Post-step forward failed: {e}");
    }
}

/// Time accumulator for real-time physics sub-stepping.
///
/// Insert this as a Bevy resource alongside [`step_physics_realtime`] to
/// run physics at wall-clock rate. Fractional timesteps carry over between
/// frames so no simulation time is lost.
///
/// # Example
///
/// ```ignore
/// app.insert_resource(PhysicsAccumulator::default())
///    .add_systems(Update, step_physics_realtime);
/// ```
#[derive(Resource)]
pub struct PhysicsAccumulator(pub f64);

impl Default for PhysicsAccumulator {
    fn default() -> Self {
        Self(0.0)
    }
}

/// Step physics enough times to keep up with wall-clock time.
///
/// Each render frame, accumulates the elapsed wall time and runs as many
/// physics steps as needed. Capped at 200 steps/frame to prevent a
/// spiral-of-death if a frame stalls.
///
/// Requires [`PhysicsAccumulator`] as a resource. Use this instead of
/// [`step_model_data`] for real-time playback.
#[allow(clippy::needless_pass_by_value)] // Bevy system parameters
pub fn step_physics_realtime(
    model: Res<PhysicsModel>,
    mut data: ResMut<PhysicsData>,
    time: Res<Time>,
    mut acc: ResMut<PhysicsAccumulator>,
) {
    acc.0 += time.delta_secs_f64();
    let dt_sim = model.0.timestep;
    let mut steps = 0;
    while acc.0 >= dt_sim && steps < 200 {
        if let Err(e) = data.0.step(&model.0) {
            eprintln!("Physics step failed: {e}");
            break;
        }
        acc.0 -= dt_sim;
        steps += 1;
    }
    // Refresh derived quantities (sensors, geom/body poses, energy) so they
    // match the post-integration qpos/qvel. Without this, these fields lag
    // by one timestep — the MuJoCo pipeline computes them BEFORE integration.
    if steps > 0 {
        if let Err(e) = data.0.forward(&model.0) {
            eprintln!("Post-step forward failed: {e}");
        }
    } else if data.0.time == 0.0 {
        // First frame: no steps taken (Bevy delta_t = 0), but ensure derived
        // quantities (actuator_moment, site_xpos, etc.) are populated so
        // PostUpdate systems don't see stale zeros.
        let _ = data.0.forward(&model.0);
    }
}

/// Copy only the fields needed for rendering and HUD from one [`Data`] to another.
///
/// This is a lighter alternative to `Data::clone()` for "Pattern B" examples
/// where a `SimEnv` owns the authoritative `Data` and [`PhysicsData`] is used
/// only for rendering.  In steady state (same model, same geom count) every
/// `clone_from` reuses existing allocations — zero heap alloc per frame.
///
/// Fields copied: `geom_xpos`, `geom_xmat`, `site_xpos`, `site_xmat`,
/// `time`, `qpos`, `qvel`, `ctrl`, `sensordata`.
pub fn sync_rendering_data(src: &Data, dst: &mut Data) {
    // Required for sync_geom_transforms:
    dst.geom_xpos.clone_from(&src.geom_xpos);
    dst.geom_xmat.clone_from(&src.geom_xmat);

    // Required for sync_site_transforms:
    dst.site_xpos.clone_from(&src.site_xpos);
    dst.site_xmat.clone_from(&src.site_xmat);

    // Required for HUD / validation:
    dst.time = src.time;
    dst.qpos.clone_from(&src.qpos);
    dst.qvel.clone_from(&src.qvel);
    dst.ctrl.clone_from(&src.ctrl);
    dst.sensordata.clone_from(&src.sensordata);
}

/// Synchronize body transforms from physics Data to Bevy entities.
///
/// This system reads `xpos` and `xquat` from the physics data (computed
/// by forward kinematics) and updates the Bevy `Transform` components.
///
/// Coordinate system conversion (Z-up → Y-up) is handled automatically.
///
/// # System Ordering
///
/// This should run in `PostUpdate` after physics stepping:
///
/// ```ignore
/// app.add_systems(Update, step_model_data);
/// app.add_systems(PostUpdate, sync_model_data_to_bevy);
/// ```
#[allow(clippy::needless_pass_by_value)] // Bevy system parameters
pub fn sync_model_data_to_bevy(
    data: Res<PhysicsData>,
    mut bodies: Query<(&ModelBodyIndex, &mut Transform)>,
) {
    for (body_idx, mut transform) in &mut bodies {
        let idx = body_idx.0;
        if idx < data.0.xpos.len() {
            let pos = &data.0.xpos[idx];
            let quat = &data.0.xquat[idx];

            // Convert Z-up (physics) to Y-up (Bevy)
            transform.translation = vec3_from_vector(pos);
            transform.rotation = quat_from_unit_quaternion(quat);
        }
    }
}

/// Synchronize geom transforms from physics Data to Bevy entities.
///
/// Geom poses are computed from body poses + local offsets.
#[allow(clippy::needless_pass_by_value)] // Bevy system parameters
pub fn sync_geom_transforms(
    data: Res<PhysicsData>,
    mut geoms: Query<(&ModelGeomIndex, &mut Transform)>,
) {
    for (geom_idx, mut transform) in &mut geoms {
        let idx = geom_idx.0;
        if idx < data.0.geom_xpos.len() {
            let pos = &data.0.geom_xpos[idx];
            let mat = &data.0.geom_xmat[idx];

            // Convert position
            transform.translation = vec3_from_vector(pos);

            // Convert rotation matrix directly (avoids double quaternion extraction)
            transform.rotation = quat_from_physics_matrix(mat);
        }
    }
}

/// Synchronize site transforms from physics Data to Bevy entities.
#[allow(clippy::needless_pass_by_value)] // Bevy system parameters
pub fn sync_site_transforms(
    data: Res<PhysicsData>,
    mut sites: Query<(&ModelSiteIndex, &mut Transform)>,
) {
    for (site_idx, mut transform) in &mut sites {
        let idx = site_idx.0;
        if idx < data.0.site_xpos.len() {
            let pos = &data.0.site_xpos[idx];
            let mat = &data.0.site_xmat[idx];

            // Convert position
            transform.translation = vec3_from_vector(pos);

            // Convert rotation matrix directly (avoids double quaternion extraction)
            transform.rotation = quat_from_physics_matrix(mat);
        }
    }
}

// ============================================================================
// System Sets
// ============================================================================

/// System set for Model/Data physics simulation.
///
/// These sets control the ordering of physics systems:
/// 1. `Step` - Advance physics simulation
/// 2. `Sync` - Synchronize transforms to Bevy
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum ModelDataSet {
    /// Physics stepping (forward dynamics + integration).
    Step,
    /// Transform synchronization (xpos/xquat → Bevy Transform).
    Sync,
}

// ============================================================================
// Plugin
// ============================================================================

/// Plugin for Model/Data physics integration.
///
/// This plugin adds the systems for stepping physics and synchronizing
/// transforms. Users must still insert the `PhysicsModel` and `PhysicsData`
/// resources themselves.
///
/// # Example
///
/// ```ignore
/// use bevy::prelude::*;
/// use sim_bevy::prelude::*;
///
/// fn main() {
///     App::new()
///         .add_plugins(DefaultPlugins)
///         .add_plugins(ModelDataPlugin)
///         .add_systems(Startup, setup_physics)
///         .run();
/// }
///
/// fn setup_physics(mut commands: Commands) {
///     let model = load_model(include_str!("robot.xml")).unwrap();
///     let data = model.make_data();
///     commands.insert_resource(PhysicsModel(model));
///     commands.insert_resource(PhysicsData(data));
/// }
/// ```
#[derive(Default)]
pub struct ModelDataPlugin {
    /// Whether to automatically step physics in Update.
    pub auto_step: bool,
}

impl ModelDataPlugin {
    /// Create a new plugin with default settings (no auto-step).
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Enable automatic physics stepping in Update schedule.
    #[must_use]
    pub fn with_auto_step(mut self) -> Self {
        self.auto_step = true;
        self
    }
}

impl Plugin for ModelDataPlugin {
    fn build(&self, app: &mut App) {
        // Configure system sets
        app.configure_sets(Update, (ModelDataSet::Step, ModelDataSet::Sync).chain());

        // Add physics step system if auto-step is enabled
        if self.auto_step {
            app.add_systems(
                Update,
                step_model_data
                    .in_set(ModelDataSet::Step)
                    .run_if(resource_exists::<PhysicsModel>)
                    .run_if(resource_exists::<PhysicsData>),
            );
        }

        // Add synchronization systems
        app.add_systems(
            PostUpdate,
            (
                sync_model_data_to_bevy,
                sync_geom_transforms,
                sync_site_transforms,
            )
                .run_if(resource_exists::<PhysicsData>),
        );
    }
}

// ============================================================================
// Utility Functions
// ============================================================================

/// Spawn Bevy entities for all bodies in the physics model.
///
/// Creates entities with `ModelBodyIndex` and `Transform` components.
/// Skips body 0 (world body) as it has no visual representation.
///
/// Returns a mapping from body index to entity.
#[allow(clippy::needless_range_loop)] // Range loop is clearer here for indexing multiple arrays
pub fn spawn_model_bodies(commands: &mut Commands, model: &Model, data: &Data) -> Vec<Entity> {
    let mut entities = vec![Entity::PLACEHOLDER; model.nbody];

    // Skip body 0 (world)
    for body_id in 1..model.nbody {
        let pos = &data.xpos[body_id];
        let quat = &data.xquat[body_id];

        let transform = Transform {
            translation: vec3_from_vector(pos),
            rotation: quat_from_unit_quaternion(quat),
            scale: Vec3::ONE,
        };

        let entity = commands
            .spawn((
                ModelBodyIndex(body_id),
                transform,
                Visibility::default(),
                InheritedVisibility::default(),
                ViewVisibility::default(),
                GlobalTransform::default(),
            ))
            .id();

        entities[body_id] = entity;
    }

    entities
}

/// Material override: geom name → material handle.
pub type GeomMaterialOverride<'a> = (&'a str, Handle<StandardMaterial>);

/// Spawn Bevy entities with meshes for all geoms in the physics model.
///
/// Creates entities with `ModelGeomIndex`, `Mesh3d`, `MeshMaterial3d`, and
/// `Transform` components. Materials default to `geom_rgba` but can be
/// overridden by name via the `overrides` slice. Meshes are generated from
/// geom type + size via [`mesh_from_shape`].
///
/// Planes get a large flat quad. Geom types without mesh support (`Sdf`,
/// `Hfield`) are skipped.
///
/// # Example
///
/// ```ignore
/// let steel = materials.add(MetalPreset::PolishedSteel.material());
/// spawn_model_geoms(&mut commands, &mut meshes, &mut materials,
///     &model, &data, &[("rod", steel)]);
/// ```
#[allow(clippy::needless_range_loop)]
pub fn spawn_model_geoms(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    model: &Model,
    data: &Data,
    overrides: &[GeomMaterialOverride<'_>],
) {
    spawn_model_geoms_inner(
        commands,
        meshes,
        materials,
        model,
        data,
        overrides,
        None::<fn(&mut bevy::ecs::system::EntityCommands, usize, &str)>,
    );
}

/// Like [`spawn_model_geoms`], but calls `per_entity` for each spawned geom.
///
/// The callback receives `(entity_commands, geom_id, geom_name)` where
/// `geom_name` is `""` for unnamed geoms. Use it to attach components like
/// [`TrailGizmo`](crate::gizmos::TrailGizmo) without a separate system.
#[allow(clippy::needless_range_loop)]
pub fn spawn_model_geoms_with<F>(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    model: &Model,
    data: &Data,
    overrides: &[GeomMaterialOverride<'_>],
    per_entity: F,
) where
    F: FnMut(&mut bevy::ecs::system::EntityCommands, usize, &str),
{
    spawn_model_geoms_inner(
        commands,
        meshes,
        materials,
        model,
        data,
        overrides,
        Some(per_entity),
    );
}

/// Shared implementation for `spawn_model_geoms` and `spawn_model_geoms_with`.
#[allow(clippy::needless_range_loop)]
fn spawn_model_geoms_inner<F>(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    model: &Model,
    data: &Data,
    overrides: &[GeomMaterialOverride<'_>],
    mut per_entity: Option<F>,
) where
    F: FnMut(&mut bevy::ecs::system::EntityCommands, usize, &str),
{
    // Build override map: geom_id → material handle
    let override_map: std::collections::HashMap<usize, Handle<StandardMaterial>> = overrides
        .iter()
        .filter_map(|(name, mat)| {
            model
                .geom_name_to_id
                .get(*name)
                .map(|&idx| (idx, mat.clone()))
        })
        .collect();

    for geom_id in 0..model.ngeom {
        let geom_type = model.geom_type[geom_id];
        let size = model.geom_size[geom_id];

        // Produce a Bevy mesh for this geom
        let mesh: Option<Mesh> = if geom_type == GeomType::Mesh {
            // Look up mesh asset: geom_mesh[geom_id] → mesh_data[mesh_id]
            model.geom_mesh[geom_id].map(|mesh_id| {
                let tri_data = &model.mesh_data[mesh_id];
                let indexed = Arc::new(tri_data.indexed_mesh().clone());
                triangle_mesh_from_indexed(&indexed)
            })
        } else {
            // Primitive shapes go through Shape → mesh_from_shape
            let shape = match geom_type {
                GeomType::Plane => Some(Shape::Plane {
                    normal: nalgebra::Vector3::z(),
                    distance: 0.0,
                }),
                GeomType::Sphere => Some(Shape::Sphere { radius: size.x }),
                GeomType::Box => Some(Shape::Box { half_extents: size }),
                GeomType::Capsule => Some(Shape::Capsule {
                    radius: size.x,
                    half_length: size.y,
                }),
                GeomType::Cylinder => Some(Shape::Cylinder {
                    radius: size.x,
                    half_length: size.y,
                }),
                GeomType::Ellipsoid => Some(Shape::Ellipsoid { radii: size }),
                _ => None,
            };
            shape.and_then(|s| mesh_from_shape(&s))
        };

        let Some(mesh) = mesh else { continue };

        // Material: override by name, or fall back to MJCF rgba
        let material = if let Some(ovr) = override_map.get(&geom_id) {
            ovr.clone()
        } else {
            let rgba = model.geom_rgba[geom_id];
            #[allow(clippy::cast_possible_truncation)]
            materials.add(StandardMaterial {
                base_color: Color::srgba(
                    rgba[0] as f32,
                    rgba[1] as f32,
                    rgba[2] as f32,
                    rgba[3] as f32,
                ),
                alpha_mode: if rgba[3] < 1.0 {
                    AlphaMode::Blend
                } else {
                    AlphaMode::Opaque
                },
                ..default()
            })
        };

        // Initial transform from computed world-frame geom pose
        let pos = &data.geom_xpos[geom_id];
        let mat = &data.geom_xmat[geom_id];

        let transform = Transform {
            translation: vec3_from_vector(pos),
            rotation: quat_from_physics_matrix(mat),
            scale: Vec3::ONE,
        };

        let mut entity_cmds = commands.spawn((
            ModelGeomIndex(geom_id),
            Mesh3d(meshes.add(mesh)),
            MeshMaterial3d(material),
            transform,
        ));

        if let Some(ref mut cb) = per_entity {
            let geom_name = model
                .geom_name
                .get(geom_id)
                .and_then(|n| n.as_deref())
                .unwrap_or("");
            cb(&mut entity_cmds, geom_id, geom_name);
        }
    }
}

/// Get the body name from the model, or a default name.
#[must_use]
pub fn body_name(model: &Model, body_id: usize) -> String {
    model
        .body_name
        .get(body_id)
        .and_then(Clone::clone)
        .unwrap_or_else(|| format!("body_{body_id}"))
}

/// Get the joint name from the model, or a default name.
#[must_use]
pub fn joint_name(model: &Model, joint_id: usize) -> String {
    model
        .jnt_name
        .get(joint_id)
        .and_then(Clone::clone)
        .unwrap_or_else(|| format!("joint_{joint_id}"))
}

// ============================================================================
// Muscle Visualization Building Blocks
// ============================================================================

/// Attachment points for a joint-transmission muscle actuator.
///
/// Derived from the model's kinematic tree: the joint's parent and child
/// bodies provide the attachment positions. Local offsets place the endpoints
/// on the body surface, offset from the body center by the geom radius.
///
/// # Usage
///
/// ```ignore
/// let attachment = MuscleAttachment::from_actuator(&model, 0, 1.0);
/// let (origin, insertion) = attachment.world_points_bevy(&data);
/// ```
pub struct MuscleAttachment {
    /// Index of the parent body (origin side).
    pub parent_body: usize,
    /// Index of the child body (insertion side).
    pub child_body: usize,
    /// Local offset from parent body origin (body-local frame).
    pub parent_local: [f64; 3],
    /// Local offset from child body origin (body-local frame).
    pub child_local: [f64; 3],
}

impl MuscleAttachment {
    /// Derive attachment from a joint-transmission actuator.
    ///
    /// `x_side`: +1.0 for front of arm, -1.0 for back (used for agonist/antagonist).
    /// The local offsets place the origin high on the parent body and the
    /// insertion partway down the child body, offset from center by the
    /// body's first geom radius.
    #[must_use]
    pub fn from_actuator(model: &Model, actuator_idx: usize, x_side: f64) -> Self {
        let jnt_id = model.actuator_trnid[actuator_idx][0];
        let child_body = model.jnt_body[jnt_id];
        let parent_body = model.body_parent[child_body];

        let parent_radius = Self::first_geom_radius(model, parent_body);
        let child_radius = Self::first_geom_radius(model, child_body);

        Self {
            parent_body,
            child_body,
            parent_local: [parent_radius * x_side, 0.0, 0.10],
            child_local: [child_radius * x_side, 0.0, -0.10],
        }
    }

    /// Compute world-space attachment points in Bevy coordinates (Y-up).
    #[must_use]
    pub fn world_points_bevy(&self, data: &Data) -> (bevy::prelude::Vec3, bevy::prelude::Vec3) {
        let origin = crate::convert::body_local_to_bevy(
            &data.xpos[self.parent_body],
            &data.xmat[self.parent_body],
            &self.parent_local,
        );
        let insertion = crate::convert::body_local_to_bevy(
            &data.xpos[self.child_body],
            &data.xmat[self.child_body],
            &self.child_local,
        );
        (origin, insertion)
    }

    fn first_geom_radius(model: &Model, body: usize) -> f64 {
        let geom_start = model.body_geom_adr[body];
        if geom_start < model.ngeom {
            model.geom_size[geom_start][0]
        } else {
            0.02
        }
    }
}

/// Component tagging a muscle mesh entity with its actuator index.
#[derive(bevy::prelude::Component)]
pub struct MuscleMeshIndex(pub usize);

/// Spawn cylinder meshes for muscle visualization.
///
/// Creates one unit cylinder per muscle, tagged with [`MuscleMeshIndex`].
/// Call [`update_muscle_meshes`] each frame to position and color them.
pub fn spawn_muscle_meshes(
    commands: &mut bevy::prelude::Commands,
    meshes: &mut bevy::prelude::ResMut<bevy::prelude::Assets<bevy::prelude::Mesh>>,
    materials: &mut bevy::prelude::ResMut<bevy::prelude::Assets<bevy::prelude::StandardMaterial>>,
    count: usize,
) {
    let mesh = meshes.add(bevy::prelude::Cylinder::new(1.0, 1.0));
    for i in 0..count {
        let mat = materials.add(bevy::prelude::StandardMaterial {
            base_color: bevy::prelude::Color::srgb(0.1, 0.1, 0.9),
            metallic: 0.3,
            perceptual_roughness: 0.6,
            ..Default::default()
        });
        commands.spawn((
            bevy::prelude::Mesh3d(mesh.clone()),
            bevy::prelude::MeshMaterial3d(mat),
            bevy::prelude::Transform::default(),
            MuscleMeshIndex(i),
        ));
    }
}

/// Update muscle mesh transforms and tension-driven color each frame.
///
/// Positions each cylinder between its attachment points, scales to match
/// the distance, and interpolates color from blue (no tension) to red
/// (peak tension, normalized against F0).
#[allow(clippy::cast_possible_truncation)] // f64→f32 is intentional for Bevy colors
pub fn update_muscle_meshes(
    model: &Model,
    data: &Data,
    attachments: &[MuscleAttachment],
    query: &mut bevy::prelude::Query<(
        &MuscleMeshIndex,
        &mut bevy::prelude::Transform,
        &bevy::prelude::MeshMaterial3d<bevy::prelude::StandardMaterial>,
    )>,
    materials: &mut bevy::prelude::ResMut<bevy::prelude::Assets<bevy::prelude::StandardMaterial>>,
) {
    for (idx, mut transform, mat_handle) in query {
        let i = idx.0;
        if i >= attachments.len() {
            continue;
        }

        let (o, ins) = attachments[i].world_points_bevy(data);
        let midpoint = (o + ins) * 0.5;
        let diff = ins - o;
        let length = diff.length();
        if length < 1e-6 {
            continue;
        }
        let dir = diff / length;
        let rotation = bevy::prelude::Quat::from_rotation_arc(bevy::prelude::Vec3::Y, dir);

        let muscle_radius = 0.008;
        transform.translation = midpoint;
        transform.rotation = rotation;
        transform.scale = bevy::prelude::Vec3::new(muscle_radius, length, muscle_radius);

        // Tension-driven color: blue (rest) → red (peak force)
        if i < model.nu {
            let f0 = model.actuator_gainprm[i][2].max(1.0);
            let tension = (data.actuator_force[i].abs() / f0).clamp(0.0, 1.0) as f32;
            if let Some(mat) = materials.get_mut(&mat_handle.0) {
                mat.base_color = bevy::prelude::Color::srgb(
                    0.8 * tension + 0.1,
                    0.1,
                    0.8 * (1.0 - tension) + 0.1,
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn model_body_index_accessors() {
        let idx = ModelBodyIndex::new(5);
        assert_eq!(idx.index(), 5);
        assert_eq!(idx.0, 5);
    }

    #[test]
    fn model_geom_index_accessors() {
        let idx = ModelGeomIndex::new(3);
        assert_eq!(idx.index(), 3);
    }

    #[test]
    fn model_site_index_accessors() {
        let idx = ModelSiteIndex::new(7);
        assert_eq!(idx.index(), 7);
    }

    #[test]
    fn physics_model_deref() {
        let model = Model::empty();
        let physics_model = PhysicsModel::new(model);

        // Should be able to access Model fields through deref
        assert_eq!(physics_model.nbody, 1); // World body
    }

    #[test]
    fn physics_data_deref_mut() {
        let model = Model::empty();
        let data = model.make_data();
        let mut physics_data = PhysicsData::new(data);

        // Should be able to mutate Data through deref
        physics_data.time = 1.0;
        assert!((physics_data.time - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn spawn_model_geoms_handles_mesh_geom_type() {
        use bevy::ecs::world::CommandQueue;
        use sim_core::mesh::TriangleMeshData;

        // Build a Model with 2 geoms: one Sphere, one Mesh
        let mut model = Model::empty();
        model.ngeom = 2;
        model.geom_type = vec![GeomType::Sphere, GeomType::Mesh];
        model.geom_size = vec![
            nalgebra::Vector3::new(1.0, 0.0, 0.0),
            nalgebra::Vector3::zeros(),
        ];
        model.geom_rgba = vec![[0.8, 0.2, 0.2, 1.0], [0.2, 0.8, 0.2, 1.0]];
        model.geom_body = vec![0, 0];
        model.geom_pos = vec![nalgebra::Vector3::zeros(); 2];
        model.geom_quat = vec![nalgebra::UnitQuaternion::identity(); 2];
        model.geom_mesh = vec![None, Some(0)];
        model.geom_hfield = vec![None, None];
        model.geom_shape = vec![None, None];

        // Provide a simple triangle mesh asset
        let vertices = vec![
            nalgebra::Point3::new(0.0, 0.0, 0.0),
            nalgebra::Point3::new(1.0, 0.0, 0.0),
            nalgebra::Point3::new(0.0, 1.0, 0.0),
            nalgebra::Point3::new(0.0, 0.0, 1.0),
        ];
        let indices = vec![0, 1, 2, 0, 1, 3, 1, 2, 3, 2, 0, 3];
        let tri_data = TriangleMeshData::new(vertices, indices);
        model.nmesh = 1;
        model.mesh_name = vec!["test_mesh".to_string()];
        model.mesh_data = vec![Arc::new(tri_data)];

        let data = model.make_data();

        // Spawn into a Bevy world
        let mut world = World::new();
        let mut meshes = Assets::<Mesh>::default();
        let mut materials = Assets::<StandardMaterial>::default();

        let mut queue = CommandQueue::default();
        {
            let mut commands = Commands::new(&mut queue, &world);
            spawn_model_geoms(
                &mut commands,
                &mut meshes,
                &mut materials,
                &model,
                &data,
                &[],
            );
        }
        queue.apply(&mut world);

        // Both geoms (sphere + mesh) should have spawned
        let mut query = world.query::<&ModelGeomIndex>();
        let spawned: Vec<usize> = query.iter(&world).map(|idx| idx.0).collect();
        assert_eq!(spawned.len(), 2, "expected 2 geoms, got {}", spawned.len());
        assert!(spawned.contains(&0), "sphere geom (id 0) should be spawned");
        assert!(spawned.contains(&1), "mesh geom (id 1) should be spawned");
    }
}
