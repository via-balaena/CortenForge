//! Double Pendulum — Chaotic Hinge Chain
//!
//! Two rigid links chained by hinge joints, released from a dramatic initial
//! angle. The system exhibits deterministic chaos — extreme sensitivity to
//! initial conditions. No closed-form period exists.
//!
//! Demonstrates: multi-body kinematic chain (2 hinges, 2 DOF), chaotic
//! dynamics, energy conservation under non-periodic motion.
//!
//! Validates:
//! - Energy conservation over 30s of chaotic motion (undamped, RK4)
//!
//! Run with: `cargo run -p example-hinge-joint-double-pendulum --release`

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
// Two-link pendulum. Both joints undamped for energy conservation testing.
//   Link 1: m=1.0kg, L=1.0m, CoM at midpoint (uniform rod)
//   Link 2: m=0.5kg, L=0.7m, CoM at midpoint
//   Initial angles: 120°, 90° — large enough for chaotic regime
//
const MJCF: &str = r#"
<mujoco model="double_pendulum">
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

        <!-- Link 1 -->
        <body name="link1" pos="0 0 0">
            <joint name="hinge1" type="hinge" axis="0 1 0" damping="0"/>
            <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="socket1" type="sphere" size="0.035"
                  pos="0 0 0" rgba="0.35 0.33 0.32 1"/>
            <geom name="rod1" type="capsule" size="0.025"
                  fromto="0 0 0  0 0 -1.0" rgba="0.50 0.50 0.53 1"/>
            <geom name="joint_ball" type="sphere" size="0.03"
                  pos="0 0 -1.0" rgba="0.35 0.33 0.32 1"/>

            <!-- Link 2 -->
            <body name="link2" pos="0 0 -1.0">
                <joint name="hinge2" type="hinge" axis="0 1 0" damping="0"/>
                <inertial pos="0 0 -0.35" mass="0.5" diaginertia="0.005 0.005 0.005"/>
                <geom name="rod2" type="capsule" size="0.02"
                      fromto="0 0 0  0 0 -0.7" rgba="0.48 0.48 0.50 1"/>
                <geom name="mass2" type="sphere" size="0.05"
                      pos="0 0 -0.7" rgba="0.15 0.45 0.82 1"/>
            </body>
        </body>
    </worldbody>
</mujoco>
"#;

// ── Physics constants ───────────────────────────────────────────────────────

const INITIAL_ANGLE_1: f64 = 2.0 * std::f64::consts::FRAC_PI_3; // 120°
const INITIAL_ANGLE_2: f64 = std::f64::consts::FRAC_PI_2; // 90°

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Double Pendulum (Hinge Joints) ===");
    println!("  Two-link chain, undamped, RK4 integrator");
    println!("  Link 1: m=1.0kg, L=1.0m | Link 2: m=0.5kg, L=0.7m");
    println!("  Initial angles: 120°, 90° — chaotic regime");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Double Pendulum (Hinge Joints)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(30.0)
                .print_every(1.0)
                .display(|m, d| {
                    let a1 = d.joint_qpos(m, 0)[0];
                    let a2 = d.joint_qpos(m, 1)[0];
                    let energy = d.energy_kinetic + d.energy_potential;
                    format!(
                        "θ1={:+6.1}°  θ2={:+6.1}°  E={energy:.4}J",
                        a1.to_degrees(),
                        a2.to_degrees(),
                    )
                })
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

    data.qpos[0] = INITIAL_ANGLE_1;
    data.qpos[1] = INITIAL_ANGLE_2;
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} DOFs\n",
        model.nbody, model.njnt, model.nv
    );

    // ── Metallic materials ──────────────────────────────────────────────
    let mat_bracket = materials.add(MetalPreset::BrushedMetal.material());
    let mat_socket = materials.add(MetalPreset::CastIron.material());
    let mat_rod1 = materials.add(MetalPreset::PolishedSteel.material());
    let mat_rod2 = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.15, 0.45, 0.82)));

    // ── Spawn MJCF geometry with material overrides ─────────────────────
    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("bracket", mat_bracket),
            ("socket1", mat_socket.clone()),
            ("joint_ball", mat_socket),
            ("rod1", mat_rod1),
            ("rod2", mat_rod2),
            ("mass2", mat_tip),
        ],
    );

    // ── Camera + lights ─────────────────────────────────────────────────
    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, -0.6),
        4.5,
        std::f32::consts::FRAC_PI_4,
        0.2,
    );

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}
