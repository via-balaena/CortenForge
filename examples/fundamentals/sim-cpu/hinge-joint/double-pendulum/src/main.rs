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
use sim_bevy::camera::{OrbitCamera, OrbitCameraPlugin};
use sim_bevy::convert::physics_pos;
use sim_bevy::materials::{MetalPreset, override_geom_materials_by_name};
use sim_bevy::model_data::{
    ModelGeomIndex, PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms,
    step_physics_realtime, sync_geom_transforms,
};
use sim_core::ENABLE_ENERGY;
use sim_core::validation::{EnergyConservationTracker, print_report};

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
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

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
        .init_resource::<DiagTimer>()
        .init_resource::<Validation>()
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_materials, step_physics_realtime).chain())
        .add_systems(PostUpdate, (sync_geom_transforms, diagnostics))
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

    data.qpos[0] = INITIAL_ANGLE_1;
    data.qpos[1] = INITIAL_ANGLE_2;
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} DOFs\n",
        model.nbody, model.njnt, model.nv
    );

    // ── Spawn MJCF geometry ─────────────────────────────────────────────
    spawn_model_geoms(&mut commands, &mut meshes, &mut materials, &model, &data);

    // ── Metallic materials ──────────────────────────────────────────────
    let mat_bracket = materials.add(MetalPreset::BrushedMetal.material());
    let mat_socket = materials.add(MetalPreset::CastIron.material());
    let mat_rod1 = materials.add(MetalPreset::PolishedSteel.material());
    let mat_rod2 = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.15, 0.45, 0.82)));

    // ── Camera + lights ─────────────────────────────────────────────────
    let mut orbit = OrbitCamera::new()
        .with_target(physics_pos(0.0, 0.0, -0.6))
        .with_angles(std::f32::consts::FRAC_PI_4, 0.2);
    orbit.max_distance = 20.0;
    orbit.distance = 4.5;
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
        bracket: mat_bracket,
        socket: mat_socket,
        rod1: mat_rod1,
        rod2: mat_rod2,
        tip: mat_tip,
    });
}

// ── Material override (runs once) ───────────────────────────────────────────

#[derive(Resource)]
struct MaterialOverrides {
    bracket: Handle<StandardMaterial>,
    socket: Handle<StandardMaterial>,
    rod1: Handle<StandardMaterial>,
    rod2: Handle<StandardMaterial>,
    tip: Handle<StandardMaterial>,
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
            ("bracket", ovr.bracket.clone()),
            ("socket1", ovr.socket.clone()),
            ("joint_ball", ovr.socket.clone()),
            ("rod1", ovr.rod1.clone()),
            ("rod2", ovr.rod2.clone()),
            ("mass2", ovr.tip.clone()),
        ],
        &mut query,
    );

    commands.remove_resource::<MaterialOverrides>();
}

// ── Diagnostics & Validation ────────────────────────────────────────────────

#[derive(Resource, Default)]
struct DiagTimer {
    last: f64,
}

#[derive(Resource)]
struct Validation {
    energy: EnergyConservationTracker,
    reported: bool,
}

impl Default for Validation {
    fn default() -> Self {
        Self {
            energy: EnergyConservationTracker::new(),
            reported: false,
        }
    }
}

fn diagnostics(data: Res<PhysicsData>, mut timer: ResMut<DiagTimer>, mut val: ResMut<Validation>) {
    let time = data.time;
    let energy = data.energy_kinetic + data.energy_potential;

    val.energy.sample(energy);

    if time - timer.last > 1.0 {
        timer.last = time;
        println!(
            "t={time:5.1}s  θ1={:+6.1}°  θ2={:+6.1}°  E={energy:.4}J",
            data.qpos[0].to_degrees(),
            data.qpos[1].to_degrees(),
        );
    }

    if time > 30.0 && !val.reported {
        val.reported = true;
        let checks = vec![val.energy.check(0.5)];
        let _ = print_report("Validation Report (t=30s)", &checks);
    }
}
