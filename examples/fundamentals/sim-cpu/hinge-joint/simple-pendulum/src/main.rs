//! Simple Pendulum — Hinge Joint
//!
//! A rigid rod with a tip mass, hanging from a hinge joint. Displaced 30° from
//! vertical and released, it swings as a simple pendulum — the simplest system
//! that demonstrates a **hinge (revolute) joint**.
//!
//! Demonstrates: `type="hinge"`, scalar angle (1 qpos, 1 qvel), axis vector,
//! energy conservation in an undamped system.
//!
//! Validates:
//! - Oscillation period matches physical pendulum formula (with amplitude correction)
//! - Energy conservation (undamped system, RK4 integrator)
//!
//! Run with: `cargo run -p example-hinge-joint-pendulum --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::convert::physics_pos;
use sim_bevy::examples::{ValidationHarness, spawn_example_camera, validation_system};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};

// ── MJCF Model ──────────────────────────────────────────────────────────────
//
// Single hinge joint pendulum. Zero damping for energy conservation testing.
//   mass = 1.0 kg at tip (inertial pos at rod end)
//   rod length = 1.0 m
//   diaginertia = 0.01 (compact sphere — keeps effective L ≈ 1.0 m)
//   axis = Y (swings in the XZ plane in MuJoCo)
//   Initial angle: 30° (π/6)
//
const MJCF: &str = r#"
<mujoco model="simple_pendulum">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
        <flag energy="enable"/>
    </option>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <!-- Pivot bracket (visual) -->
        <geom name="bracket" type="box" size="0.06 0.04 0.03"
              pos="0 0 0" rgba="0.45 0.45 0.48 1"/>

        <!-- Pendulum arm -->
        <body name="arm" pos="0 0 0">
            <joint name="hinge" type="hinge" axis="0 1 0"
                   damping="0"/>
            <inertial pos="0 0 -1.0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="socket" type="sphere" size="0.035"
                  pos="0 0 0" rgba="0.35 0.33 0.32 1"/>
            <geom name="rod" type="capsule" size="0.02"
                  fromto="0 0 0  0 0 -1.0" rgba="0.50 0.50 0.53 1"/>
            <geom name="mass" type="sphere" size="0.06"
                  pos="0 0 -1.0" rgba="0.82 0.22 0.15 1"/>
        </body>
    </worldbody>
</mujoco>
"#;

// ── Physics constants ───────────────────────────────────────────────────────

const MASS: f64 = 1.0;
const GRAVITY: f64 = 9.81;
const COM_DIST: f64 = 1.0; // distance from pivot to CoM
const I_CM: f64 = 0.01; // diaginertia about hinge axis
const INITIAL_ANGLE: f64 = std::f64::consts::FRAC_PI_6; // 30°

/// Analytical period for a physical pendulum with amplitude correction.
///
/// Base: `T₀ = 2π√(I_pivot / (m·g·d))`  where `I_pivot = I_cm + m·d²`
/// Correction: `T ≈ T₀·(1 + θ₀²/16)`  (accurate to ~0.01% at 30°)
fn analytical_period() -> f64 {
    let i_pivot = I_CM + MASS * COM_DIST * COM_DIST;
    let t0 = 2.0 * std::f64::consts::PI * (i_pivot / (MASS * GRAVITY * COM_DIST)).sqrt();
    let correction = 1.0 + INITIAL_ANGLE * INITIAL_ANGLE / 16.0;
    t0 * correction
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    let t = analytical_period();
    let i_pivot = I_CM + MASS * COM_DIST * COM_DIST;
    println!("=== CortenForge: Simple Pendulum (Hinge Joint) ===");
    println!("  Rigid rod + tip mass, undamped, RK4 integrator");
    println!("  m=1.0kg  L=1.0m  I_pivot={i_pivot:.2}  g={GRAVITY}");
    println!("  Analytical period T = {t:.4} s (with amplitude correction)");
    println!("  Initial angle = 30°");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Simple Pendulum (Hinge Joint)".into(),
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
                    let angle = d.joint_qpos(m, 0)[0];
                    let vel = d.joint_qvel(m, 0)[0];
                    let energy = d.energy_kinetic + d.energy_potential;
                    format!(
                        "angle={:+6.1}°  vel={vel:+.2} rad/s  E={energy:.4}J",
                        angle.to_degrees(),
                    )
                })
                .track_period(
                    "Period",
                    |m, d| (d.joint_qpos(m, 0)[0], d.time),
                    analytical_period(),
                    2.0,
                )
                .track_energy(0.5),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(PostUpdate, (sync_geom_transforms, validation_system))
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();

    // Start at 30° from vertical
    data.qpos[0] = INITIAL_ANGLE;
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} DOFs\n",
        model.nbody, model.njnt, model.nv
    );

    // ── Metallic materials ──────────────────────────────────────────────
    let mat_bracket = materials.add(MetalPreset::BrushedMetal.material());
    let mat_socket = materials.add(MetalPreset::CastIron.material());
    let mat_rod = materials.add(MetalPreset::PolishedSteel.material());
    let mat_mass =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.82, 0.22, 0.15)));

    // ── Spawn MJCF geometry with material overrides ─────────────────────
    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("bracket", mat_bracket),
            ("socket", mat_socket),
            ("rod", mat_rod),
            ("mass", mat_mass),
        ],
    );

    // ── Camera + lights ─────────────────────────────────────────────────
    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, -0.4),
        3.0,
        std::f32::consts::FRAC_PI_4,
        0.25,
    );

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}
