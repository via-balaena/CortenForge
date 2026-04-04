//! Multi-scene physics support for side-by-side comparison examples.
//!
//! This module provides a **separate code path** from the single-scene
//! [`PhysicsModel`](crate::model_data::PhysicsModel) /
//! [`PhysicsData`](crate::model_data::PhysicsData) pattern. Users choose one
//! or the other. Multi-scene is needed for solver and integrator comparison
//! examples where multiple physics scenes run in lockstep.
//!
//! # Why a separate plugin?
//!
//! 1. **Accumulator fairness:** A shared accumulator with scenes at different
//!    timesteps gives the faster scene more steps per frame, silently
//!    invalidating comparisons. Each scene needs its own accumulator.
//! 2. **Query incompatibility:** Single-scene sync uses
//!    `Query<(&ModelGeomIndex, &mut Transform)>` reading `Res<PhysicsData>`.
//!    Multi-scene needs `Query<(&PhysicsSceneId, &ModelGeomIndex, &mut Transform)>`
//!    reading per-scene data. These query signatures are incompatible.
//!
//! # Usage
//!
//! ```ignore
//! use bevy::prelude::*;
//! use sim_bevy::prelude::*;
//!
//! fn main() {
//!     App::new()
//!         .add_plugins(DefaultPlugins)
//!         .add_plugins(MultiScenePlugin)  // NOT ModelDataPlugin
//!         .add_systems(Startup, setup)
//!         .run();
//! }
//!
//! fn setup(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>,
//!          mut materials: ResMut<Assets<StandardMaterial>>) {
//!     let mut scenes = PhysicsScenes::default();
//!     let spacing = 2.0;
//!
//!     for (i, solver) in ["PGS", "CG", "Newton"].iter().enumerate() {
//!         let model = load_model(MJCF).unwrap();
//!         let data = model.make_data();
//!         let offset = Vec3::new(i as f32 * spacing, 0.0, 0.0);
//!         let id = scenes.add(*solver, model, data);
//!         spawn_scene_geoms(&mut commands, &mut meshes, &mut materials,
//!             id, scenes.get(id).unwrap(), offset, &[]);
//!     }
//!
//!     commands.insert_resource(scenes);
//! }
//! ```

use std::sync::Arc;

use bevy::prelude::*;
use cf_geometry::Shape;
use sim_core::batch::BatchSim;
use sim_core::{Data, GeomType, Model};

use crate::convert::{quat_from_physics_matrix, vec3_from_vector};
use crate::mesh::{mesh_from_shape, triangle_mesh_from_indexed};
use crate::model_data::{GeomMaterialOverride, ModelGeomIndex};

// ============================================================================
// Types
// ============================================================================

/// A physics scene: model + data + its own time accumulator.
///
/// Each scene runs independently with its own accumulator, ensuring fair
/// comparison even when scenes have different timesteps.
pub struct PhysicsScene {
    /// The static physics model.
    pub model: Model,
    /// The dynamic physics data.
    pub data: Data,
    /// Human-readable label (e.g., "PGS", "CG", "Newton").
    pub label: String,
    /// Per-scene time accumulator for lockstep stepping.
    accumulator: f64,
    /// World-space offset for side-by-side layout (applied during sync).
    pub offset: Vec3,
}

/// Resource holding multiple physics scenes.
///
/// Mutually exclusive with [`PhysicsModel`](crate::model_data::PhysicsModel) /
/// [`PhysicsData`](crate::model_data::PhysicsData). Use
/// [`MultiScenePlugin`] instead of
/// [`ModelDataPlugin`](crate::model_data::ModelDataPlugin).
#[derive(Resource, Default)]
pub struct PhysicsScenes {
    scenes: Vec<PhysicsScene>,
}

impl PhysicsScenes {
    /// Add a scene and return its ID (sequential, starting at 0).
    pub fn add(&mut self, label: impl Into<String>, model: Model, data: Data) -> usize {
        let id = self.scenes.len();
        self.scenes.push(PhysicsScene {
            model,
            data,
            label: label.into(),
            accumulator: 0.0,
            offset: Vec3::ZERO,
        });
        id
    }

    /// Get a scene by ID.
    #[must_use]
    pub fn get(&self, id: usize) -> Option<&PhysicsScene> {
        self.scenes.get(id)
    }

    /// Get a mutable scene by ID.
    pub fn get_mut(&mut self, id: usize) -> Option<&mut PhysicsScene> {
        self.scenes.get_mut(id)
    }

    /// Number of scenes.
    #[must_use]
    pub fn len(&self) -> usize {
        self.scenes.len()
    }

    /// Whether there are no scenes.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.scenes.is_empty()
    }

    /// Iterate over scenes.
    pub fn iter(&self) -> impl Iterator<Item = &PhysicsScene> {
        self.scenes.iter()
    }

    /// Iterate over scenes mutably.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut PhysicsScene> {
        self.scenes.iter_mut()
    }
}

/// Identifies which physics scene an entity belongs to.
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PhysicsSceneId(pub usize);

// ============================================================================
// Systems
// ============================================================================

/// Step all scenes in lockstep: each scene advances by the same wall-clock
/// time, regardless of its internal timestep.
///
/// Each scene has its **own accumulator**. A scene with `dt=0.001` steps 5×
/// per frame compared to `dt=0.005`, but both advance the same wall-clock
/// duration. For comparison scenes (solvers), use the same timestep across
/// all scenes so they take the same number of steps.
///
/// Capped at 200 steps per frame per scene to prevent spiral-of-death.
#[allow(clippy::needless_pass_by_value)] // Bevy system parameters
pub fn step_scenes_lockstep(mut scenes: ResMut<PhysicsScenes>, time: Res<Time>) {
    let wall_dt = time.delta_secs_f64();
    for scene in scenes.iter_mut() {
        scene.accumulator += wall_dt;
        let dt = scene.model.timestep;
        let mut steps = 0;
        while scene.accumulator >= dt && steps < 200 {
            if let Err(e) = scene.data.step(&scene.model) {
                eprintln!("Multi-scene step failed ({}): {e}", scene.label);
                break;
            }
            scene.accumulator -= dt;
            steps += 1;
        }
        if steps > 0 {
            if let Err(e) = scene.data.forward(&scene.model) {
                eprintln!("Multi-scene forward failed ({}): {e}", scene.label);
            }
        }
    }
}

/// Sync geom transforms for multi-scene entities.
///
/// Reads from [`PhysicsScenes`] (not `Res<PhysicsData>`). Each entity is
/// tagged with [`PhysicsSceneId`] to select the correct scene's data.
/// The per-scene [`offset`](PhysicsScene::offset) is applied so scenes
/// appear side by side.
#[allow(clippy::needless_pass_by_value)] // Bevy system parameters
pub fn sync_scene_geom_transforms(
    scenes: Res<PhysicsScenes>,
    mut geoms: Query<(&PhysicsSceneId, &ModelGeomIndex, &mut Transform)>,
) {
    for (scene_id, geom_idx, mut transform) in &mut geoms {
        if let Some(scene) = scenes.get(scene_id.0) {
            let idx = geom_idx.0;
            if idx < scene.data.geom_xpos.len() {
                transform.translation = vec3_from_vector(&scene.data.geom_xpos[idx]) + scene.offset;
                transform.rotation = quat_from_physics_matrix(&scene.data.geom_xmat[idx]);
            }
        }
    }
}

/// Copy geom poses from a [`BatchSim`] into the corresponding [`PhysicsScenes`].
///
/// Call each frame **before** [`sync_scene_geom_transforms`] to keep
/// visual geoms in sync with batch simulation state. Each `BatchSim`
/// env maps to the `PhysicsScene` at the same index.
///
/// ```ignore
/// fn my_sync(res: Res<MyBatchResource>, mut scenes: ResMut<PhysicsScenes>) {
///     sync_batch_geoms(&res.batch, &mut scenes);
/// }
/// ```
pub fn sync_batch_geoms(batch: &BatchSim, scenes: &mut PhysicsScenes) {
    for i in 0..batch.len() {
        if let (Some(env), Some(scene)) = (batch.env(i), scenes.get_mut(i)) {
            for g in 0..env.geom_xpos.len() {
                if g < scene.data.geom_xpos.len() {
                    scene.data.geom_xpos[g] = env.geom_xpos[g];
                    scene.data.geom_xmat[g] = env.geom_xmat[g];
                }
            }
        }
    }
}

// ============================================================================
// Spawning
// ============================================================================

/// Spawn geoms for one scene, offset in world space for side-by-side layout.
///
/// Tags each entity with [`PhysicsSceneId`]. The `offset` is stored on the
/// scene automatically so [`sync_scene_geom_transforms`] applies it every
/// frame. Does nothing if `scene_id` is out of range.
#[allow(clippy::needless_range_loop)]
pub fn spawn_scene_geoms(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    scenes: &mut PhysicsScenes,
    scene_id: usize,
    offset: Vec3,
    overrides: &[GeomMaterialOverride<'_>],
) {
    spawn_scene_geoms_inner(
        commands,
        meshes,
        materials,
        scenes,
        scene_id,
        offset,
        overrides,
        None::<fn(&mut bevy::ecs::system::EntityCommands, usize, &str)>,
    );
}

/// Like [`spawn_scene_geoms`], but calls `per_entity` for each spawned geom.
///
/// The callback receives `(entity_commands, geom_id, geom_name)` where
/// `geom_name` is `""` for unnamed geoms. Does nothing if `scene_id` is
/// out of range.
#[allow(clippy::needless_range_loop, clippy::too_many_arguments)]
pub fn spawn_scene_geoms_with<F>(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    scenes: &mut PhysicsScenes,
    scene_id: usize,
    offset: Vec3,
    overrides: &[GeomMaterialOverride<'_>],
    per_entity: F,
) where
    F: FnMut(&mut bevy::ecs::system::EntityCommands, usize, &str),
{
    spawn_scene_geoms_inner(
        commands,
        meshes,
        materials,
        scenes,
        scene_id,
        offset,
        overrides,
        Some(per_entity),
    );
}

/// Shared implementation for scene geom spawning.
///
/// Stores the offset on the scene, then spawns entities tagged with
/// [`PhysicsSceneId`].
#[allow(clippy::needless_range_loop, clippy::too_many_arguments)]
fn spawn_scene_geoms_inner<F>(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    scenes: &mut PhysicsScenes,
    scene_id: usize,
    offset: Vec3,
    overrides: &[GeomMaterialOverride<'_>],
    mut per_entity: Option<F>,
) where
    F: FnMut(&mut bevy::ecs::system::EntityCommands, usize, &str),
{
    // Store offset so sync_scene_geom_transforms applies it every frame.
    let Some(scene) = scenes.get_mut(scene_id) else {
        return;
    };
    scene.offset = offset;

    let model = &scene.model;
    let data = &scene.data;

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
            model.geom_mesh[geom_id].map(|mesh_id| {
                let tri_data = &model.mesh_data[mesh_id];
                let indexed = Arc::new(tri_data.indexed_mesh().clone());
                triangle_mesh_from_indexed(&indexed)
            })
        } else {
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

        // Initial transform with offset applied
        let pos = &data.geom_xpos[geom_id];
        let mat = &data.geom_xmat[geom_id];

        let transform = Transform {
            translation: vec3_from_vector(pos) + offset,
            rotation: quat_from_physics_matrix(mat),
            scale: Vec3::ONE,
        };

        let mut entity_cmds = commands.spawn((
            PhysicsSceneId(scene_id),
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

// ============================================================================
// Plugin
// ============================================================================

/// Plugin for multi-scene physics.
///
/// Mutually exclusive with
/// [`ModelDataPlugin`](crate::model_data::ModelDataPlugin). Registers
/// [`step_scenes_lockstep`] in `Update` and [`sync_scene_geom_transforms`]
/// in `PostUpdate`, both gated on `resource_exists::<PhysicsScenes>`.
pub struct MultiScenePlugin;

impl Plugin for MultiScenePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            step_scenes_lockstep.run_if(resource_exists::<PhysicsScenes>),
        )
        .add_systems(
            PostUpdate,
            sync_scene_geom_transforms.run_if(resource_exists::<PhysicsScenes>),
        );
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add_returns_sequential_ids() {
        let mut scenes = PhysicsScenes::default();
        let m0 = Model::empty();
        let d0 = m0.make_data();
        let m1 = Model::empty();
        let d1 = m1.make_data();
        let m2 = Model::empty();
        let d2 = m2.make_data();

        assert_eq!(scenes.add("A", m0, d0), 0);
        assert_eq!(scenes.add("B", m1, d1), 1);
        assert_eq!(scenes.add("C", m2, d2), 2);
        assert_eq!(scenes.len(), 3);
        assert!(!scenes.is_empty());
    }

    #[test]
    #[allow(clippy::expect_used)]
    fn lockstep_identical_scenes_produce_identical_time() {
        // Two identical scenes with same timestep should produce identical
        // data.time after the same wall-clock accumulation.
        let m_a = Model::empty(); // timestep = 0.002
        let d_a = m_a.make_data();
        let m_b = Model::empty();
        let d_b = m_b.make_data();

        let mut scenes = PhysicsScenes::default();
        scenes.add("A", m_a, d_a);
        scenes.add("B", m_b, d_b);

        // Simulate 10 frames at ~60fps (wall_dt = 0.016)
        let wall_dt = 0.016;
        for _ in 0..10 {
            for scene in scenes.iter_mut() {
                scene.accumulator += wall_dt;
                let dt = scene.model.timestep;
                let mut steps = 0;
                while scene.accumulator >= dt && steps < 200 {
                    let _ = scene.data.step(&scene.model);
                    scene.accumulator -= dt;
                    steps += 1;
                }
                if steps > 0 {
                    let _ = scene.data.forward(&scene.model);
                }
            }
        }

        let time_a = scenes.get(0).expect("scene 0 missing").data.time;
        let time_b = scenes.get(1).expect("scene 1 missing").data.time;
        assert!(
            (time_a - time_b).abs() < 1e-12,
            "times should be identical: {time_a} vs {time_b}"
        );
        // Should have advanced (not stuck at zero)
        assert!(time_a > 0.0, "time should have advanced: {time_a}");
    }

    #[test]
    #[allow(clippy::expect_used)]
    fn three_scenes_spawn_tagged_entities() {
        use bevy::ecs::world::CommandQueue;

        // Build 3 scenes, each with 1 sphere geom
        let mut scenes = PhysicsScenes::default();
        for label in ["A", "B", "C"] {
            let mut model = Model::empty();
            model.ngeom = 1;
            model.geom_type = vec![GeomType::Sphere];
            model.geom_size = vec![nalgebra::Vector3::new(0.5, 0.0, 0.0)];
            model.geom_rgba = vec![[0.8, 0.2, 0.2, 1.0]];
            model.geom_body = vec![0];
            model.geom_pos = vec![nalgebra::Vector3::zeros()];
            model.geom_quat = vec![nalgebra::UnitQuaternion::identity()];
            model.geom_mesh = vec![None];
            model.geom_hfield = vec![None];
            model.geom_shape = vec![None];
            let data = model.make_data();
            scenes.add(label, model, data);
        }

        let mut world = World::new();
        let mut meshes = Assets::<Mesh>::default();
        let mut materials = Assets::<StandardMaterial>::default();

        let mut queue = CommandQueue::default();
        {
            let mut commands = Commands::new(&mut queue, &world);
            for id in 0..3 {
                #[allow(clippy::cast_precision_loss)]
                let offset = Vec3::new(id as f32 * 2.0, 0.0, 0.0);
                spawn_scene_geoms(
                    &mut commands,
                    &mut meshes,
                    &mut materials,
                    &mut scenes,
                    id,
                    offset,
                    &[],
                );
            }
        }
        queue.apply(&mut world);

        // Should have 3 entities (1 geom × 3 scenes)
        let mut query = world.query::<(&PhysicsSceneId, &ModelGeomIndex)>();
        let spawned: Vec<(usize, usize)> = query
            .iter(&world)
            .map(|(sid, gid)| (sid.0, gid.0))
            .collect();
        assert_eq!(
            spawned.len(),
            3,
            "expected 3 entities, got {}",
            spawned.len()
        );

        // Each scene ID should appear exactly once
        for scene_id in 0..3 {
            let count = spawned.iter().filter(|(s, _)| *s == scene_id).count();
            assert_eq!(
                count, 1,
                "scene {scene_id} should have 1 entity, got {count}"
            );
        }
    }
}
