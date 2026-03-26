//! Horizontal Slide Joint — Prismatic Spring-Mass Oscillator
//!
//! A block on a frictionless horizontal rail, held between two springs attached
//! to the walls at each end. Displaced from center and released, it oscillates
//! as a damped harmonic oscillator.
//!
//! Demonstrates: `type="slide"`, joint axis, stiffness, damping, armature
//! (reflected inertia), joint limits.
//!
//! Validates:
//! - Oscillation period matches `T = 2pi * sqrt((m + armature) / k)`
//! - Energy monotonically decreases (damping dissipates, never injects)
//! - Joint limits never exceeded
//!
//! Run with: `cargo run -p example-slide-joint-horizontal --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops
)]

use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{ValidationHarness, spawn_example_camera, validation_system};
use sim_bevy::materials::MetalPreset;
use sim_bevy::mesh::{SpringCoilParams, spring_coil};
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
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
    println!("=== CortenForge: Horizontal Slide Joint ===");
    println!("  Mass between two springs on a frictionless rail");
    println!("  m=1.0kg  armature=0.1  k=20 N/m  c=0.2 Ns/m");
    println!("  Effective mass = {MASS_EFF:.1} kg");
    println!("  Analytical period T = {t:.4} s");
    println!("  Initial displacement = {INITIAL_DISP} m");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Horizontal Slide Joint".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let pos = d.joint_qpos(m, 0)[0];
                    let energy = d.energy_kinetic + d.energy_potential;
                    format!("pos={pos:+.4}m  E={energy:.4}J")
                })
                .track_period(
                    "Period",
                    |m, d| (d.joint_qpos(m, 0)[0], d.time),
                    analytical_period(),
                    2.0,
                )
                .track_energy_monotonic(1e-6)
                .track_limit(
                    "Limits",
                    |m, d| {
                        let pos = d.joint_qpos(m, 0)[0];
                        let (lo, hi) = m.jnt_range[0];
                        (pos, lo, hi)
                    },
                    0.001,
                ),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (sync_geom_transforms, update_springs, validation_system),
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

    // ── Metallic materials (using presets) ───────────────────────────────
    let mat_rail = materials.add(MetalPreset::PolishedSteel.material());
    let mat_wall = materials.add(MetalPreset::BrushedMetal.material());
    let mat_block =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.82, 0.22, 0.15)));
    let mat_spring = materials.add(MetalPreset::SpringWire.material());

    // ── Spawn MJCF geometry with material overrides ─────────────────────
    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("rail", mat_rail),
            ("wall_lo", mat_wall.clone()),
            ("wall_hi", mat_wall),
            ("block", mat_block),
        ],
    );

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
    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.05, 0.0),
        4.0,
        std::f32::consts::FRAC_PI_2,
        0.25,
    );

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
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
