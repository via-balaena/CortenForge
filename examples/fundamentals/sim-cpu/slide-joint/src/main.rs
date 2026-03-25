//! Slide Joint — Prismatic Spring-Mass Oscillator
//!
//! A block on a frictionless rail, held between two springs attached to the
//! walls at each end. Displaced from center and released, it oscillates as a
//! damped harmonic oscillator.
//!
//! Demonstrates: `type="slide"`, joint axis, stiffness, damping, armature
//! (reflected inertia), joint limits.
//!
//! Validates:
//! - Oscillation period matches `T = 2pi * sqrt((m + armature) / k)`
//! - Energy monotonically decreases (damping dissipates, never injects)
//! - Joint limits never exceeded
//!
//! Run with: `cargo run -p example-slide-joint --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use
)]

use std::f32::consts::TAU;

use bevy::asset::RenderAssetUsages;
use bevy::mesh::PrimitiveTopology;
use bevy::prelude::*;
use sim_bevy::camera::OrbitCamera;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::model_data::{
    ModelGeomIndex, PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms,
    step_physics_realtime, sync_geom_transforms,
};
use sim_core::ENABLE_ENERGY;

// ── MJCF Model ──────────────────────────────────────────────────────────────
//
// A single slide joint along the X axis at z=0.
//   mass = 1.0 kg, armature = 0.1 → effective mass = 1.1 kg
//   stiffness = 20 N/m, damping = 0.2 Ns/m
//   range = [-1.0, 1.0] m (joint limits)
//   Initial displacement: 0.8 m
//
// Visual: block between two walls, a spring on each side.
// The joint spring models the net restoring force of both springs.
//
const MJCF: &str = r#"
<mujoco model="slide_joint">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <!-- Rail: polished rod along X axis -->
        <geom name="rail" type="capsule" size="0.025"
              fromto="-1.3 0 0  1.3 0 0" rgba="0.50 0.50 0.53 1"/>

        <!-- Walls at range boundaries -->
        <geom name="wall_lo" type="box" size="0.015 0.12 0.12"
              pos="-1.0 0 0" rgba="0.45 0.45 0.48 1"/>
        <geom name="wall_hi" type="box" size="0.015 0.12 0.12"
              pos="1.0 0 0" rgba="0.45 0.45 0.48 1"/>

        <!-- The sliding block -->
        <body name="block" pos="0 0 0">
            <joint name="slide" type="slide" axis="1 0 0"
                   stiffness="20" damping="0.2" armature="0.1"
                   limited="true" range="-1.0 1.0"/>
            <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="block" type="box" size="0.12 0.10 0.10"
                  rgba="0.82 0.22 0.15 1"
                  contype="1" conaffinity="1"/>
        </body>
    </worldbody>
</mujoco>
"#;

// ── Physics constants ───────────────────────────────────────────────────────

const MASS_EFF: f64 = 1.0 + 0.1; // mass + armature
const STIFFNESS: f64 = 20.0;
const INITIAL_DISP: f64 = 0.8;
const BLOCK_HALF: f32 = 0.12; // half-size of block along X
const WALL_X: f32 = 1.0; // wall position (matches joint range)

/// Analytical period: `T = 2pi * sqrt(m_eff / k)`
fn analytical_period() -> f64 {
    2.0 * std::f64::consts::PI * (MASS_EFF / STIFFNESS).sqrt()
}

// ── Spring coil mesh ────────────────────────────────────────────────────────

const COIL_TURNS: u32 = 10;
const COIL_RADIUS: f32 = 0.06;
const COIL_TUBE_RADIUS: f32 = 0.008;
const COIL_SEGMENTS_PER_TURN: u32 = 24;
const COIL_TUBE_SEGMENTS: u32 = 8;
const COIL_MIN_LENGTH: f32 = 0.05;

/// Generate a helical coil mesh stretching from `x=0` to `x=length` along X.
#[allow(clippy::suboptimal_flops, clippy::cast_precision_loss)]
fn make_coil_mesh(length: f32) -> Mesh {
    let length = length.max(COIL_MIN_LENGTH);
    let total_segs = COIL_TURNS * COIL_SEGMENTS_PER_TURN;
    let n_verts = (total_segs + 1) * (COIL_TUBE_SEGMENTS + 1);
    let mut positions = Vec::with_capacity(n_verts as usize);
    let mut normals = Vec::with_capacity(n_verts as usize);
    let mut indices = Vec::new();

    for i in 0..=total_segs {
        let t = i as f32 / total_segs as f32;
        let cx = t * length;
        let helix_angle = t * COIL_TURNS as f32 * TAU;
        let cy = COIL_RADIUS * helix_angle.cos();
        let cz = COIL_RADIUS * helix_angle.sin();

        // Radial direction (outward from helix axis)
        let radial_y = helix_angle.cos();
        let radial_z = helix_angle.sin();

        // Tangent of the helix (Frenet frame)
        let tangent_x = length / (total_segs as f32);
        let tangent_y =
            -COIL_RADIUS * helix_angle.sin() * COIL_TURNS as f32 * TAU / total_segs as f32;
        let tangent_z =
            COIL_RADIUS * helix_angle.cos() * COIL_TURNS as f32 * TAU / total_segs as f32;
        let tang_len = (tangent_x * tangent_x + tangent_y * tangent_y + tangent_z * tangent_z)
            .sqrt()
            .max(1e-6);
        let tx = tangent_x / tang_len;
        let ty = tangent_y / tang_len;
        let tz = tangent_z / tang_len;

        // Binormal = tangent x radial
        let bx = ty * radial_z - tz * radial_y;
        let by = tz * 0.0 - tx * radial_z;
        let bz = tx * radial_y - ty * 0.0;
        let b_len = (bx * bx + by * by + bz * bz).sqrt().max(1e-6);
        let bx = bx / b_len;
        let by = by / b_len;
        let bz = bz / b_len;

        for j in 0..=COIL_TUBE_SEGMENTS {
            let tube_angle = j as f32 / COIL_TUBE_SEGMENTS as f32 * TAU;
            let cos_t = tube_angle.cos();
            let sin_t = tube_angle.sin();

            let nx = sin_t * bx;
            let ny = cos_t * radial_y + sin_t * by;
            let nz = cos_t * radial_z + sin_t * bz;

            let px = cx + COIL_TUBE_RADIUS * (sin_t * bx);
            let py = cy + COIL_TUBE_RADIUS * (cos_t * radial_y + sin_t * by);
            let pz = cz + COIL_TUBE_RADIUS * (cos_t * radial_z + sin_t * bz);

            positions.push([px, py, pz]);
            let n_len = (nx * nx + ny * ny + nz * nz).sqrt().max(1e-6);
            normals.push([nx / n_len, ny / n_len, nz / n_len]);
        }
    }

    let ring = COIL_TUBE_SEGMENTS + 1;
    for i in 0..total_segs {
        for j in 0..COIL_TUBE_SEGMENTS {
            let a = i * ring + j;
            let b = a + ring;
            let c = b + 1;
            let d = a + 1;
            indices.extend_from_slice(&[a, b, d, b, c, d]);
        }
    }

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_indices(bevy::mesh::Indices::U32(indices));
    mesh
}

/// Which side of the block this spring is on.
#[derive(Component)]
enum SpringSide {
    Left,
    Right,
}

/// Pre-built metallic materials for geom overrides.
#[derive(Resource)]
struct MetalMaterials {
    rail: Handle<StandardMaterial>,
    wall: Handle<StandardMaterial>,
    block: Handle<StandardMaterial>,
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    let t = analytical_period();
    println!("=== CortenForge: Slide Joint ===");
    println!("  Mass between two springs on a frictionless rail");
    println!("  m=1.0kg  armature=0.1  k=20 N/m  c=0.2 Ns/m");
    println!("  Effective mass = {MASS_EFF:.1} kg");
    println!("  Analytical period T = {t:.4} s");
    println!("  Initial displacement = {INITIAL_DISP} m");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Slide Joint".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<DiagTimer>()
        .init_resource::<Validation>()
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (apply_metal_materials, step_physics_realtime).chain(),
        )
        .add_systems(
            PostUpdate,
            (sync_geom_transforms, update_springs, diagnostics),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mut model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    model.enableflags |= ENABLE_ENERGY;
    let mut data = model.make_data();

    data.qpos[0] = INITIAL_DISP;
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} DOFs\n",
        model.nbody, model.njnt, model.nv
    );

    // ── Spawn MJCF geometry ─────────────────────────────────────────────
    spawn_model_geoms(&mut commands, &mut meshes, &mut materials, &model, &data);

    // ── Metallic material overrides (by geom name) ──────────────────────
    let metal_rail = materials.add(StandardMaterial {
        base_color: Color::srgb(0.52, 0.53, 0.56),
        metallic: 0.9,
        perceptual_roughness: 0.25,
        ..default()
    });
    let metal_wall = materials.add(StandardMaterial {
        base_color: Color::srgb(0.48, 0.48, 0.50),
        metallic: 0.85,
        perceptual_roughness: 0.35,
        ..default()
    });
    let metal_block = materials.add(StandardMaterial {
        base_color: Color::srgb(0.82, 0.22, 0.15),
        metallic: 0.7,
        perceptual_roughness: 0.3,
        ..default()
    });
    let spring_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.55, 0.58, 0.60),
        metallic: 0.85,
        perceptual_roughness: 0.35,
        ..default()
    });

    // ── Left spring: from left wall to left face of block ───────────────
    let block_x = data.qpos[0] as f32;
    let left_start = -WALL_X;
    let left_end = block_x - BLOCK_HALF;
    let left_len = (left_end - left_start).max(COIL_MIN_LENGTH);
    commands.spawn((
        SpringSide::Left,
        Mesh3d(meshes.add(make_coil_mesh(left_len))),
        MeshMaterial3d(spring_mat.clone()),
        Transform::from_xyz(left_start, 0.0, 0.0),
    ));

    // ── Right spring: from right face of block to right wall ────────────
    let right_start = block_x + BLOCK_HALF;
    let right_len = (WALL_X - right_start).max(COIL_MIN_LENGTH);
    commands.spawn((
        SpringSide::Right,
        Mesh3d(meshes.add(make_coil_mesh(right_len))),
        MeshMaterial3d(spring_mat),
        Transform::from_xyz(right_start, 0.0, 0.0),
    ));

    // ── Camera + lights (no ground plane) ───────────────────────────────
    let mut orbit = OrbitCamera::new()
        .with_target(Vec3::new(0.0, 0.05, 0.0))
        .with_angles(0.3, 0.35);
    orbit.max_distance = 20.0;
    orbit.distance = 4.0;
    let mut cam_transform = Transform::default();
    orbit.apply_to_transform(&mut cam_transform);
    commands.spawn((Camera3d::default(), orbit, cam_transform));

    commands.insert_resource(GlobalAmbientLight {
        color: Color::WHITE,
        brightness: 800.0,
        ..default()
    });
    commands.spawn((
        DirectionalLight {
            illuminance: 15_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(30.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 5_000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(-20.0, 30.0, -30.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
    commands.insert_resource(MetalMaterials {
        rail: metal_rail,
        wall: metal_wall,
        block: metal_block,
    });
}

/// Apply metallic PBR materials to MJCF geoms (runs once, then removes itself).
fn apply_metal_materials(
    mut commands: Commands,
    model: Option<Res<PhysicsModel>>,
    mats: Option<Res<MetalMaterials>>,
    mut query: Query<(&ModelGeomIndex, &mut MeshMaterial3d<StandardMaterial>)>,
) {
    let (Some(model), Some(mats)) = (model, mats) else {
        return;
    };

    for (geom_idx, mut mat_handle) in &mut query {
        let name = model.geom_name.get(geom_idx.0).and_then(|n| n.as_deref());
        match name {
            Some("rail") => mat_handle.0 = mats.rail.clone(),
            Some("wall_lo" | "wall_hi") => mat_handle.0 = mats.wall.clone(),
            Some("block") => mat_handle.0 = mats.block.clone(),
            _ => {}
        }
    }

    // Run once then remove
    commands.remove_resource::<MetalMaterials>();
}

/// Rebuild both spring coil meshes each frame to match the block position.
fn update_springs(
    data: Res<PhysicsData>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut query: Query<(&SpringSide, &Mesh3d, &mut Transform)>,
) {
    let block_x = data.qpos[0] as f32;

    for (side, mesh_handle, mut transform) in &mut query {
        let (start, length) = match side {
            SpringSide::Left => {
                let s = -WALL_X;
                let len = (block_x - BLOCK_HALF) - s;
                (s, len)
            }
            SpringSide::Right => {
                let s = block_x + BLOCK_HALF;
                let len = WALL_X - s;
                (s, len)
            }
        };

        transform.translation.x = start;
        if let Some(mesh) = meshes.get_mut(&mesh_handle.0) {
            *mesh = make_coil_mesh(length);
        }
    }
}

// ── Diagnostics & Validation ────────────────────────────────────────────────

#[derive(Resource, Default)]
struct DiagTimer {
    last: f64,
}

#[derive(Resource, Default)]
struct Validation {
    last_pos: f64,
    last_time: f64,
    crossings: Vec<f64>,
    prev_energy: Option<f64>,
    max_energy_increase: f64,
    max_limit_violation: f64,
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut timer: ResMut<DiagTimer>,
    mut val: ResMut<Validation>,
) {
    let pos = data.qpos[0];
    let energy = data.energy_kinetic + data.energy_potential;
    let time = data.time;

    // ── Energy monotonicity ─────────────────────────────────────────────
    if let Some(prev) = val.prev_energy {
        let increase = energy - prev;
        if increase > val.max_energy_increase {
            val.max_energy_increase = increase;
        }
    }
    val.prev_energy = Some(energy);

    // ── Joint limit violation ───────────────────────────────────────────
    if model.jnt_limited[0] {
        let (lo, hi) = model.jnt_range[0];
        let violation = (pos - hi).max(lo - pos).max(0.0);
        if violation > val.max_limit_violation {
            val.max_limit_violation = violation;
        }
    }

    // ── Zero-crossing (positive → negative) with linear interpolation ──
    if val.last_pos > 0.0 && pos <= 0.0 && time > 0.01 {
        let frac = val.last_pos / (val.last_pos - pos);
        let t_cross = frac.mul_add(time - val.last_time, val.last_time);
        val.crossings.push(t_cross);
    }
    val.last_pos = pos;
    val.last_time = time;

    // ── Periodic printout ───────────────────────────────────────────────
    if time - timer.last > 1.0 {
        timer.last = time;
        println!("t={time:5.1}s  pos={pos:+.4}m  E={energy:.4}J");
    }

    // ── Validation report at t=15s ──────────────────────────────────────
    if time > 15.0 && !val.reported {
        val.reported = true;
        println!("\n=== Validation Report (t=15s) ===");

        if val.crossings.len() >= 2 {
            let periods: Vec<f64> = val.crossings.windows(2).map(|w| w[1] - w[0]).collect();
            #[allow(clippy::cast_precision_loss)]
            let measured = periods.iter().sum::<f64>() / periods.len() as f64;
            let expected = analytical_period();
            let err = ((measured - expected) / expected).abs() * 100.0;
            println!(
                "  Period:  measured={measured:.4}s  expected={expected:.4}s  error={err:.2}%  {}",
                if err < 2.0 { "PASS" } else { "FAIL" },
            );
        } else {
            println!("  Period:  insufficient zero crossings  FAIL");
        }

        {
            let pass = val.max_energy_increase < 1e-6;
            println!(
                "  Energy:  max increase={:.2e}J  {}",
                val.max_energy_increase,
                if pass { "PASS" } else { "FAIL" },
            );
        }

        {
            let pass = val.max_limit_violation < 0.001;
            println!(
                "  Limits:  max violation={:.6}m  {}",
                val.max_limit_violation,
                if pass { "PASS" } else { "FAIL" },
            );
        }

        println!("================================\n");
    }
}
