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

use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use sim_bevy::camera::{OrbitCamera, OrbitCameraPlugin};
use sim_bevy::materials::{MetalPreset, override_geom_materials_by_name};
use sim_bevy::mesh::{SpringCoilParams, spring_coil};
use sim_bevy::model_data::{
    ModelGeomIndex, PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms,
    step_physics_realtime, sync_geom_transforms,
};
use sim_core::ENABLE_ENERGY;
use sim_core::validation::{EnergyMonotonicityTracker, LimitTracker, PeriodTracker, print_report};

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

// ── Spring visual ───────────────────────────────────────────────────────────

const COIL_PARAMS: SpringCoilParams = SpringCoilParams {
    turns: 10,
    radius: 0.06,
    tube_radius: 0.008,
    segments_per_turn: 24,
    tube_segments: 8,
    min_length: 0.05,
};

/// Which side of the block this spring is on.
#[derive(Component)]
enum SpringSide {
    Left,
    Right,
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
        .add_systems(Update, (apply_materials, step_physics_realtime).chain())
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

    // ── Metallic materials (using presets) ───────────────────────────────
    let mat_rail = materials.add(MetalPreset::PolishedSteel.material());
    let mat_wall = materials.add(MetalPreset::BrushedMetal.material());
    let mat_block =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.82, 0.22, 0.15)));
    let mat_spring = materials.add(MetalPreset::SpringWire.material());

    // ── Springs: left wall → block, block → right wall ──────────────────
    let block_x = data.qpos[0] as f32;

    let left_start = -WALL_X;
    let left_len = (block_x - BLOCK_HALF) - left_start;
    commands.spawn((
        SpringSide::Left,
        Mesh3d(meshes.add(spring_coil(
            &COIL_PARAMS,
            left_len,
            RenderAssetUsages::default(),
        ))),
        MeshMaterial3d(mat_spring.clone()),
        Transform::from_xyz(left_start, 0.0, 0.0),
    ));

    let right_start = block_x + BLOCK_HALF;
    let right_len = WALL_X - right_start;
    commands.spawn((
        SpringSide::Right,
        Mesh3d(meshes.add(spring_coil(
            &COIL_PARAMS,
            right_len,
            RenderAssetUsages::default(),
        ))),
        MeshMaterial3d(mat_spring),
        Transform::from_xyz(right_start, 0.0, 0.0),
    ));

    // ── Camera + lights (no ground plane) ───────────────────────────────
    let mut orbit = OrbitCamera::new()
        .with_target(Vec3::new(0.0, 0.05, 0.0))
        .with_angles(std::f32::consts::FRAC_PI_2, 0.25);
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
    commands.insert_resource(MaterialOverrides {
        rail: mat_rail,
        wall: mat_wall,
        block: mat_block,
    });
}

// ── Material override (runs once) ───────────────────────────────────────────

#[derive(Resource)]
struct MaterialOverrides {
    rail: Handle<StandardMaterial>,
    wall: Handle<StandardMaterial>,
    block: Handle<StandardMaterial>,
}

fn apply_materials(
    mut commands: Commands,
    model: Option<Res<PhysicsModel>>,
    overrides: Option<Res<MaterialOverrides>>,
    mut query: Query<(&ModelGeomIndex, &mut MeshMaterial3d<StandardMaterial>)>,
) {
    let (Some(model), Some(ovr)) = (model, overrides) else {
        return;
    };

    override_geom_materials_by_name(
        &model,
        &[
            ("rail", ovr.rail.clone()),
            ("wall_lo", ovr.wall.clone()),
            ("wall_hi", ovr.wall.clone()),
            ("block", ovr.block.clone()),
        ],
        &mut query,
    );

    commands.remove_resource::<MaterialOverrides>();
}

// ── Spring update ───────────────────────────────────────────────────────────

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
            *mesh = spring_coil(&COIL_PARAMS, length, RenderAssetUsages::default());
        }
    }
}

// ── Diagnostics & Validation ────────────────────────────────────────────────

#[derive(Resource, Default)]
struct DiagTimer {
    last: f64,
}

#[derive(Resource)]
struct Validation {
    period: PeriodTracker,
    energy: EnergyMonotonicityTracker,
    limits: LimitTracker,
    reported: bool,
}

impl Default for Validation {
    fn default() -> Self {
        Self {
            period: PeriodTracker::new(),
            energy: EnergyMonotonicityTracker::new(),
            limits: LimitTracker::new(),
            reported: false,
        }
    }
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

    val.period.sample(pos, time);
    val.energy.sample(energy);

    if model.jnt_limited[0] {
        let (lo, hi) = model.jnt_range[0];
        val.limits.sample(pos, lo, hi);
    }

    if time - timer.last > 1.0 {
        timer.last = time;
        println!("t={time:5.1}s  pos={pos:+.4}m  E={energy:.4}J");
    }

    if time > 15.0 && !val.reported {
        val.reported = true;
        let checks = vec![
            val.period.check(analytical_period(), 2.0),
            val.energy.check(1e-6),
            val.limits.check(0.001),
        ];
        let _ = print_report("Validation Report (t=15s)", &checks);
    }
}
