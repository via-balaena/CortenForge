//! Full Pipeline — Design → Simulate → Stress → Lattice → Print
//!
//! Capstone example proving all three CortenForge domains compose end-to-end.
//! Press Space to advance through 5 visualization stages:
//!
//! 1. Design  — solid mechanism (palm + 2 fingers)
//! 2. Simulate — animated gripper closing
//! 3. Stress  — force distribution overlay
//! 4. Lattice — per-part stress-graded lattice infill
//! 5. Print   — shelled + lattice parts with printability report
//!
//! Run with: `cargo run -p example-full-pipeline --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use
)]

use std::sync::Arc;

use bevy::prelude::*;
use cf_design::{
    ActuatorDef, ActuatorKind, JointDef, JointKind, Material, Mechanism, Part, Solid, TendonDef,
    TendonWaypoint,
};
use cf_geometry::IndexedMesh;
use mesh_lattice::{DensityMap, LatticeParams, generate_lattice};
use mesh_measure::dimensions;
use mesh_printability::{PrinterConfig, validate_for_printing};
use mesh_repair::{RepairParams, repair_mesh};
use mesh_shell::ShellBuilder;
use nalgebra::{Point3, Vector3};
use sim_bevy::camera::{OrbitCamera, OrbitCameraPlugin};
use sim_bevy::convert::{quat_from_unit_quaternion, vec3_from_vector};
use sim_bevy::mesh::triangle_mesh_from_indexed;
use sim_bevy::model_data::{
    PhysicsData, PhysicsModel, spawn_model_geoms, step_model_data, sync_geom_transforms,
};
use sim_core::{Data, Model};

// ============================================================================
// Stage state machine
// ============================================================================

/// The 5 visualization stages.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
enum PipelineStage {
    #[default]
    Design,
    Simulate,
    Stress,
    Lattice,
    PrintReady,
}

impl PipelineStage {
    const fn label(self) -> &'static str {
        match self {
            Self::Design => "1. Design",
            Self::Simulate => "2. Simulate",
            Self::Stress => "3. Stress",
            Self::Lattice => "4. Lattice",
            Self::PrintReady => "5. Print-Ready",
        }
    }

    const fn next(self) -> Self {
        match self {
            Self::Design => Self::Simulate,
            Self::Simulate => Self::Stress,
            Self::Stress => Self::Lattice,
            Self::Lattice | Self::PrintReady => Self::PrintReady,
        }
    }
}

#[derive(Resource, Default)]
struct CurrentStage(PipelineStage);

/// Marker for entities belonging to a specific stage.
#[derive(Component)]
struct StageEntity(PipelineStage);

// ============================================================================
// Gripper mechanism (reused from design-to-sim)
// ============================================================================

fn build_gripper() -> Mechanism {
    let material = Material::new("PLA", 1250.0)
        .with_youngs_modulus(3.5e9)
        .with_color([0.8, 0.85, 0.9, 1.0]);

    let palm = Part::new(
        "palm",
        Solid::cuboid(Vector3::new(12.0, 8.0, 4.0)).round(1.0),
        material.clone(),
    );
    let left_finger = Part::new("finger_l", Solid::capsule(2.5, 8.0), material.clone());
    let right_finger = Part::new("finger_r", Solid::capsule(2.5, 8.0), material);

    Mechanism::builder("gripper")
        .part(palm)
        .part(left_finger)
        .part(right_finger)
        .joint(
            JointDef::new(
                "hinge_l",
                "palm",
                "finger_l",
                JointKind::Revolute,
                Point3::new(-10.0, 8.0, 0.0),
                Vector3::x(),
            )
            .with_range(-0.2, 1.2),
        )
        .joint(
            JointDef::new(
                "hinge_r",
                "palm",
                "finger_r",
                JointKind::Revolute,
                Point3::new(10.0, 8.0, 0.0),
                Vector3::x(),
            )
            .with_range(-0.2, 1.2),
        )
        .tendon(
            TendonDef::new(
                "flexor_l",
                vec![
                    TendonWaypoint::new("palm", Point3::new(-8.0, -5.0, 0.0)),
                    TendonWaypoint::new("palm", Point3::new(-8.0, 6.0, 0.0)),
                    TendonWaypoint::new("finger_l", Point3::new(0.0, 5.0, 0.0)),
                ],
                0.8,
            )
            .with_stiffness(150.0)
            .with_damping(8.0),
        )
        .tendon(
            TendonDef::new(
                "flexor_r",
                vec![
                    TendonWaypoint::new("palm", Point3::new(8.0, -5.0, 0.0)),
                    TendonWaypoint::new("palm", Point3::new(8.0, 6.0, 0.0)),
                    TendonWaypoint::new("finger_r", Point3::new(0.0, 5.0, 0.0)),
                ],
                0.8,
            )
            .with_stiffness(150.0)
            .with_damping(8.0),
        )
        .actuator(
            ActuatorDef::new("motor_l", "flexor_l", ActuatorKind::Motor, (-30.0, 30.0))
                .with_ctrl_range(-1.0, 1.0),
        )
        .actuator(
            ActuatorDef::new("motor_r", "flexor_r", ActuatorKind::Motor, (-30.0, 30.0))
                .with_ctrl_range(-1.0, 1.0),
        )
        .build()
}

// ============================================================================
// Stress field (reused from sim-informed-design)
// ============================================================================

fn build_stress_field(
    data: &Data,
    model: &Model,
    sigma: f64,
) -> Arc<dyn Fn(Point3<f64>) -> f64 + Send + Sync> {
    let mut stress_sources: Vec<(Point3<f64>, f64)> = Vec::new();

    for body_id in 1..model.nbody {
        let force = &data.cfrc_ext[body_id];
        let linear_mag = Vector3::new(force[3], force[4], force[5]).norm();
        if linear_mag > 1e-6 {
            let pos = &data.xpos[body_id];
            stress_sources.push((Point3::new(pos.x, pos.y, pos.z), linear_mag));
        }
    }

    for contact in &data.contacts {
        if contact.depth > 1e-8 {
            let pos = contact.pos;
            stress_sources.push((Point3::new(pos.x, pos.y, pos.z), contact.depth * 1000.0));
        }
    }

    let inv_2sigma2 = 1.0 / (2.0 * sigma * sigma);

    Arc::new(move |p: Point3<f64>| -> f64 {
        stress_sources
            .iter()
            .map(|(src, mag)| {
                let dist2 = (p - src).norm_squared();
                mag * (-dist2 * inv_2sigma2).exp()
            })
            .sum()
    })
}

// ============================================================================
// Pre-computed pipeline data
// ============================================================================

/// All pre-computed data for non-interactive stages (3, 4, 5).
#[derive(Resource)]
struct PipelineData {
    /// Stage 3: force indicator positions + magnitudes (in Bevy coords)
    force_indicators: Vec<(Vec3, f32)>,
    /// Stage 4: per-part lattice meshes
    lattice_meshes: Vec<(String, IndexedMesh)>,
    /// Stage 5: per-part shell meshes
    shell_meshes: Vec<(String, IndexedMesh)>,
}

fn run_pipeline(mechanism: &Mechanism) -> PipelineData {
    println!("=== Running full pipeline ===\n");

    // ── Design meshes ────────────────────────────────────────────────
    let stl_kit = mechanism.to_stl_kit(0.3);
    let mut design_meshes: Vec<(String, IndexedMesh)> = Vec::new();
    for (name, mut mesh) in stl_kit {
        let _ = repair_mesh(&mut mesh, &RepairParams::default());
        println!(
            "  Part '{name}': {} vertices, {} faces",
            mesh.vertex_count(),
            mesh.face_count()
        );
        design_meshes.push((name, mesh));
    }

    // ── Simulate ─────────────────────────────────────────────────────
    println!("\nSimulating 100 steps with applied forces...");
    let mjcf_xml = mechanism.to_mjcf(1.5);
    let model = sim_mjcf::load_model(&mjcf_xml).expect("MJCF should load");
    let mut data = model.make_data();

    // Apply forces and step
    for _ in 0..100 {
        if !data.ctrl.is_empty() {
            data.ctrl[0] = 0.8;
        }
        if data.ctrl.len() > 1 {
            data.ctrl[1] = 0.8;
        }
        // Apply load on fingers
        for body_id in 2..model.nbody {
            data.xfrc_applied[body_id] = nalgebra::Vector6::new(0.0, 5.0, 0.0, 0.0, 0.0, -15.0);
        }
        let _ = data.step(&model);
    }
    // Re-apply forces and call inverse() to populate cfrc_ext
    let _ = data.forward(&model);
    for body_id in 1..model.nbody {
        data.xfrc_applied[body_id] = nalgebra::Vector6::new(0.0, 5.0, 0.0, 0.0, 0.0, -15.0);
    }
    data.inverse(&model);
    println!("  Simulation complete");

    // ── Stress field ─────────────────────────────────────────────────
    println!("\nBuilding stress field...");
    let stress_fn = build_stress_field(&data, &model, 8.0);

    // Force indicators for stage 3
    let mut force_indicators = Vec::new();
    for body_id in 1..model.nbody {
        let force = &data.cfrc_ext[body_id];
        let linear_mag = Vector3::new(force[3], force[4], force[5]).norm();
        if linear_mag > 1e-6 {
            let pos = &data.xpos[body_id];
            force_indicators.push((vec3_from_vector(pos), linear_mag as f32));
        }
    }

    // ── Per-part lattice + shell ─────────────────────────────────────
    println!("\nPer-part processing:");
    let mut lattice_meshes = Vec::new();
    let mut shell_meshes = Vec::new();

    for (name, part_mesh) in &design_meshes {
        // Shell
        let shell_result = ShellBuilder::new(part_mesh)
            .wall_thickness(1.0)
            .fast()
            .build()
            .expect("shell");

        // Per-part stress density
        let stress_clone = stress_fn.clone();
        let density_map = DensityMap::from_stress_field(move |p| stress_clone(p), 0.15, 0.55);

        // Shape-conforming lattice
        let solid = mechanism
            .parts()
            .iter()
            .find(|p| p.name() == name)
            .expect("part exists")
            .solid()
            .clone();
        let sdf = Arc::new(move |p: Point3<f64>| solid.evaluate(&p));

        let dims = dimensions(part_mesh);
        let params = LatticeParams::gyroid(8.0)
            .with_density_map(density_map)
            .with_shape_sdf(sdf);
        let lattice = generate_lattice(&params, (dims.min, dims.max)).expect("lattice generation");

        // Printability
        let report = validate_for_printing(&lattice.mesh, &PrinterConfig::fdm_default())
            .expect("printability validation");
        println!(
            "  {name}: lattice={} verts, printable={} — {}",
            lattice.vertex_count(),
            report.is_printable(),
            report.summary()
        );

        lattice_meshes.push((name.clone(), lattice.mesh));
        shell_meshes.push((name.clone(), shell_result.mesh));
    }

    PipelineData {
        force_indicators,
        lattice_meshes,
        shell_meshes,
    }
}

// ============================================================================
// Bevy systems
// ============================================================================

fn spawn_mesh_entity(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    mesh_data: &IndexedMesh,
    color: Color,
    transform: Transform,
    stage: PipelineStage,
) -> Entity {
    let indexed = Arc::new(mesh_data.clone());
    let bevy_mesh = triangle_mesh_from_indexed(&indexed);

    commands
        .spawn((
            StageEntity(stage),
            Mesh3d(meshes.add(bevy_mesh)),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                metallic: 0.3,
                perceptual_roughness: 0.5,
                double_sided: true,
                cull_mode: None,
                ..default()
            })),
            transform,
            Visibility::Hidden,
        ))
        .id()
}

/// Spawn named meshes positioned at their geom transforms.
///
/// Uses geom transforms (not body transforms) because design meshes are
/// centered on the solid origin, and the MJCF generator may offset geoms
/// within their parent body.
fn spawn_part_meshes(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    parts: &[(String, IndexedMesh)],
    geom_transforms: &[Transform],
    colors: &[Color],
    stage: PipelineStage,
) {
    for (i, (_name, mesh)) in parts.iter().enumerate() {
        let transform = geom_transforms.get(i).copied().unwrap_or_default();
        spawn_mesh_entity(
            commands,
            meshes,
            materials,
            mesh,
            colors[i % colors.len()],
            transform,
            stage,
        );
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mechanism = build_gripper();

    println!("=== CortenForge: Full Pipeline ===");
    println!(
        "  Mechanism '{}': {} parts, {} joints\n",
        mechanism.name(),
        mechanism.parts().len(),
        mechanism.joints().len(),
    );

    // ── Run the full pipeline (offline) ──────────────────────────────
    let pipeline = run_pipeline(&mechanism);

    // ── Load physics model for initial poses ───────────────────────
    let mjcf_xml = mechanism.to_mjcf(1.5);
    let model = sim_mjcf::load_model(&mjcf_xml).expect("MJCF should load");
    let mut data = model.make_data();
    let _ = data.forward(&model);

    // Cache per-part geom transforms for lattice/shell stages.
    // Use geom_xpos/geom_xmat (not body xpos) because these include the
    // geom offset that makes parts extend outward from joints.
    let part_transforms: Vec<Transform> = (0..pipeline.lattice_meshes.len())
        .map(|part_idx| {
            let body_id = part_idx + 1; // skip world body
            (0..model.ngeom)
                .find(|&g| model.geom_body[g] == body_id)
                .map(|gid| {
                    let pos = &data.geom_xpos[gid];
                    let mat = &data.geom_xmat[gid];
                    let rotation = nalgebra::Rotation3::from_matrix_unchecked(*mat);
                    let quat = nalgebra::UnitQuaternion::from_rotation_matrix(&rotation);
                    Transform {
                        translation: vec3_from_vector(pos),
                        rotation: quat_from_unit_quaternion(&quat),
                        scale: Vec3::ONE,
                    }
                })
                .unwrap_or_default()
        })
        .collect();

    // ── Stages 1 & 2: Design + Simulate (same geoms, different behavior)
    // spawn_model_geoms positions meshes correctly via geom transforms.
    // Stage 1 shows them static, Stage 2 animates them.
    spawn_model_geoms(&mut commands, &mut meshes, &mut materials, &model, &data);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));

    // ── Stage 3: Stress (force indicators, initially hidden) ─────────
    let max_force = pipeline
        .force_indicators
        .iter()
        .map(|(_, m)| *m)
        .fold(0.0_f32, f32::max)
        .max(1.0);

    for (pos, mag) in &pipeline.force_indicators {
        let intensity = *mag / max_force;
        let radius = 1.5 + intensity * 4.0;
        commands.spawn((
            StageEntity(PipelineStage::Stress),
            Mesh3d(meshes.add(Sphere::new(radius))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(1.0, 0.2 * (1.0 - intensity), 0.1),
                emissive: LinearRgba::new(intensity * 2.0, 0.0, 0.0, 1.0),
                ..default()
            })),
            Transform::from_translation(*pos),
            Visibility::Hidden,
        ));
    }

    // ── Stages 4 & 5: Lattice + Print-ready (spawned only if meshes are small enough)
    let total_lattice_verts: usize = pipeline
        .lattice_meshes
        .iter()
        .map(|(_, m)| m.vertex_count())
        .sum();
    if total_lattice_verts < 500_000 {
        spawn_part_meshes(
            &mut commands,
            &mut meshes,
            &mut materials,
            &pipeline.lattice_meshes,
            &part_transforms,
            &[Color::srgb(0.3, 0.8, 0.4)],
            PipelineStage::Lattice,
        );
        spawn_part_meshes(
            &mut commands,
            &mut meshes,
            &mut materials,
            &pipeline.shell_meshes,
            &part_transforms,
            &[Color::srgb(0.9, 0.75, 0.5)],
            PipelineStage::PrintReady,
        );
    } else {
        println!(
            "  Skipping lattice/shell visualization ({total_lattice_verts} vertices too heavy for GPU)"
        );
    }

    // Scene setup (camera, lights, ground)
    spawn_scene(&mut commands, &mut meshes, &mut materials);

    // Initialize stage
    commands.insert_resource(CurrentStage::default());

    println!("\n  Press SPACE to advance stages");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");
    println!("  >>> Stage: 1. Design <<<");
}

fn spawn_scene(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) {
    let mut orbit = OrbitCamera::new()
        .with_target(Vec3::ZERO)
        .with_angles(0.5, 0.5);
    orbit.max_distance = 500.0;
    orbit.min_distance = 0.5;
    orbit.orbit_speed = 0.008;
    orbit.pan_speed = 0.015;
    orbit.zoom_speed = 0.15;
    orbit.distance = 80.0;
    let mut cam_transform = Transform::default();
    orbit.apply_to_transform(&mut cam_transform);
    commands.spawn((Camera3d::default(), orbit, cam_transform));

    commands.spawn((
        DirectionalLight {
            illuminance: 12000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(30.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 4000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(-20.0, 30.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(50.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.5, 0.5, 0.5, 0.3),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, -15.0, 0.0),
    ));
}

/// Advance stage on Space press.
fn advance_stage(keyboard: Res<ButtonInput<KeyCode>>, mut stage: ResMut<CurrentStage>) {
    if keyboard.just_pressed(KeyCode::Space) {
        let next = stage.0.next();
        if next != stage.0 {
            stage.0 = next;
            println!("  >>> Stage: {} <<<", stage.0.label());
        }
    }
}

/// Oscillate gripper in simulate stage.
fn actuate_gripper(mut data: ResMut<PhysicsData>) {
    let t = data.time;
    let signal = (t * 2.0).sin();
    if !data.ctrl.is_empty() {
        data.ctrl[0] = signal;
    }
    if data.ctrl.len() > 1 {
        data.ctrl[1] = signal;
    }
}

/// Show/hide entities based on current stage.
fn update_visibility(
    stage: Res<CurrentStage>,
    mut stage_entities: Query<(&StageEntity, &mut Visibility)>,
    mut sim_geoms: Query<
        &mut Visibility,
        (
            With<sim_bevy::model_data::ModelGeomIndex>,
            Without<StageEntity>,
        ),
    >,
) {
    let current = stage.0;

    // Stage-tagged entities
    for (tag, mut vis) in &mut stage_entities {
        *vis = if tag.0 == current
            || (current == PipelineStage::PrintReady && tag.0 == PipelineStage::Lattice)
        {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
    }

    // Sim geoms (from spawn_model_geoms) — visible during Design, Simulate, Stress
    let sim_visible = current == PipelineStage::Design
        || current == PipelineStage::Simulate
        || current == PipelineStage::Stress;
    for mut vis in &mut sim_geoms {
        *vis = if sim_visible {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
    }
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Full Pipeline".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, (advance_stage, actuate_gripper, step_model_data))
        .add_systems(PostUpdate, (sync_geom_transforms, update_visibility))
        .run();
}
