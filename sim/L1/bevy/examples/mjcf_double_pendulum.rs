//! Double pendulum from MJCF: serial chain with coupling dynamics.
//!
//! Validates the full MJCF → chain topology → FK pipeline with:
//! - 2 hinge joints (axis Y) with spring on upper joint
//! - Off-diagonal mass matrix coupling (CRBA)
//! - Coriolis terms (RNE)
//! - Chaotic motion from sensitive initial conditions
//!
//! Based on conformance `double_pendulum.xml` with visual-friendly sizes.
//!
//! Run with: `cargo run -p sim-bevy --example mjcf_double_pendulum --release`

#![allow(clippy::needless_pass_by_value)]
#![allow(clippy::expect_used)]
#![allow(clippy::cast_precision_loss)]

use bevy::prelude::*;
use sim_bevy::camera::{OrbitCamera, OrbitCameraPlugin};
use sim_bevy::convert::{quat_from_unit_quaternion, vec3_from_vector};
use sim_bevy::model_data::{ModelDataPlugin, PhysicsData, PhysicsModel};
use sim_bevy::prelude::{load_model, spawn_model_geoms};
use sim_core::ENABLE_ENERGY;

// ============================================================================
// MJCF Model Definition
// ============================================================================

/// Double pendulum: two hinges in serial chain with spring on upper joint.
///
/// Based on conformance `double_pendulum.xml` with visual-friendly sizes.
/// Links hang DOWNWARD (along -Z in physics frame) so at qpos=0 both
/// links are at rest hanging straight down.
///
/// - Upper body: 0.8m capsule (blue), hinge Y, damping=1.2
/// - Lower body: 0.6m capsule (red), hinge Y, damping=0.3
///
/// No springs — pure gravity-driven chaotic motion with bearing friction
/// (damping only). Tuned for fluid, natural-looking motion that settles
/// over ~15s. RK4 integrator prevents numerical energy injection.
///
/// The off-diagonal mass matrix coupling means upper joint motion drives
/// lower joint and vice versa — the signature feature of double pendulums.
const DOUBLE_PENDULUM_MJCF: &str = r#"
<mujoco model="mjcf_double_pendulum">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4"/>

    <worldbody>
        <!-- Pivot at Z=2, links hang down along -Z -->
        <body name="upper" pos="0 0 2">
            <joint name="joint1" type="hinge" axis="0 1 0"
                   damping="1.2"/>
            <geom name="upper_geom" type="capsule" fromto="0 0 0 0 0 -0.8"
                  size="0.04" mass="1.0" rgba="0.3 0.5 0.85 1"/>

            <body name="lower" pos="0 0 -0.8">
                <joint name="joint2" type="hinge" axis="0 1 0"
                       damping="0.3"/>
                <geom name="lower_geom" type="capsule" fromto="0 0 0 0 0 -0.6"
                      size="0.035" mass="0.5" rgba="0.85 0.25 0.2 1"/>
            </body>
        </body>
    </worldbody>
</mujoco>
"#;

/// Maximum physics steps per frame (safety cap).
const MAX_STEPS_PER_FRAME: usize = 32;

// ============================================================================
// Resources
// ============================================================================

/// Initial total energy for drift tracking.
#[derive(Resource)]
struct InitialEnergy(f64);

/// Whether to show joint axis gizmos.
#[derive(Resource)]
struct ShowJointAxes(bool);

/// Previous total energy for monotonicity check.
#[derive(Resource)]
struct PrevEnergy(f64);

/// Timer for periodic console output.
#[derive(Resource)]
struct HudTimer {
    last_print_time: f64,
}

// ============================================================================
// Main
// ============================================================================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(OrbitCameraPlugin)
        .add_plugins(ModelDataPlugin::new()) // no auto_step — wall-clock stepping
        .insert_resource(ShowJointAxes(false))
        .add_systems(Startup, setup)
        .add_systems(Update, (step_realtime, toggle_gizmos))
        .add_systems(PostUpdate, (draw_joint_axes, window_hud, console_hud))
        .run();
}

// ============================================================================
// Setup
// ============================================================================

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Load MJCF and enable energy tracking
    let mut model = load_model(DOUBLE_PENDULUM_MJCF).expect("Failed to load MJCF");
    model.enableflags |= ENABLE_ENERGY;

    let mut data = model.make_data();

    // Start with upper link at π/3 (60°) — dramatic swing without excessive energy
    data.qpos[0] = std::f64::consts::FRAC_PI_3;
    let _ = data.forward(&model);

    // Initial energy
    let e0 = data.energy_kinetic + data.energy_potential;

    // Auto-spawn visual entities for all geoms (upper + lower capsules)
    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[],
    );

    // Pivot marker (small gray sphere at upper joint origin)
    // Physics (0, 0, 2) → Bevy (0, 2, 0)
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.06))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.4, 0.4, 0.4),
            metallic: 0.6,
            ..default()
        })),
        Transform::from_xyz(0.0, 2.0, 0.0),
    ));

    // Camera — side view (azimuth=π/2) at mid-chain height
    let orbit = OrbitCamera::new()
        .with_target(Vec3::new(0.0, 1.2, 0.0))
        .with_distance(5.0)
        .with_angles(std::f32::consts::FRAC_PI_2, 0.15);
    let mut cam_t = Transform::default();
    orbit.apply_to_transform(&mut cam_t);
    commands.spawn((Camera3d::default(), orbit, cam_t));

    // Lighting
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    println!("=============================================");
    println!("MJCF Double Pendulum — Serial Chain + Coupling");
    println!("=============================================");
    println!("Upper: hinge Y, damping=1.2");
    println!("Lower: hinge Y, damping=0.3");
    println!("Initial: upper link at 60deg, lower hanging (RK4)");
    println!("---------------------------------------------");
    println!("Press 'J' to toggle joint axis gizmos (yellow)");
    println!("=============================================");

    commands.insert_resource(PhysicsModel::new(model));
    commands.insert_resource(PhysicsData::new(data));
    commands.insert_resource(InitialEnergy(e0));
    commands.insert_resource(PrevEnergy(e0));
    commands.insert_resource(HudTimer {
        last_print_time: -1.0,
    });
}

// ============================================================================
// Systems
// ============================================================================

/// Step physics in sync with wall-clock time.
fn step_realtime(model: Res<PhysicsModel>, mut data: ResMut<PhysicsData>, time: Res<Time>) {
    let frame_dt = time.delta_secs_f64();
    let steps = ((frame_dt / model.timestep).round() as usize).clamp(1, MAX_STEPS_PER_FRAME);
    for _ in 0..steps {
        if let Err(e) = data.step(&model) {
            eprintln!("Physics step failed: {e}");
            return;
        }
    }
}

/// Toggle joint axis gizmos with J key.
fn toggle_gizmos(keyboard: Res<ButtonInput<KeyCode>>, mut show: ResMut<ShowJointAxes>) {
    if keyboard.just_pressed(KeyCode::KeyJ) {
        show.0 = !show.0;
        println!("Joint axis gizmos: {}", if show.0 { "ON" } else { "OFF" });
    }
}

/// Draw joint axis gizmos (yellow lines along joint axes at body positions).
fn draw_joint_axes(
    show: Res<ShowJointAxes>,
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut gizmos: Gizmos,
) {
    if !show.0 {
        return;
    }

    for jnt_id in 0..model.njnt {
        let body_id = model.jnt_body[jnt_id];
        let axis_local = &model.jnt_axis[jnt_id];

        let body_pos = vec3_from_vector(&data.xpos[body_id]);
        let body_quat = quat_from_unit_quaternion(&data.xquat[body_id]);
        let axis_world = body_quat * vec3_from_vector(axis_local);

        let half_len = 0.4;
        let start = body_pos - axis_world * half_len;
        let end = body_pos + axis_world * half_len;

        gizmos.line(start, end, Color::srgb(1.0, 0.9, 0.1));
        gizmos.sphere(
            Isometry3d::from_translation(start),
            0.015,
            Color::srgb(1.0, 0.9, 0.1),
        );
        gizmos.sphere(
            Isometry3d::from_translation(end),
            0.015,
            Color::srgb(1.0, 0.9, 0.1),
        );
    }
}

/// Window title HUD: compact live readout of angles, energy, time.
fn window_hud(
    data: Res<PhysicsData>,
    initial: Res<InitialEnergy>,
    mut windows: Query<&mut Window>,
) {
    let theta1 = data.qpos[0];
    let theta2 = data.qpos[1];
    let energy = data.energy_kinetic + data.energy_potential;
    let drift = if initial.0.abs() > 1e-10 {
        (energy - initial.0) / initial.0.abs() * 100.0
    } else {
        0.0
    };

    if let Ok(mut window) = windows.single_mut() {
        window.title = format!(
            "Double Pendulum | t={:.1}s | th1={:+.0} th2={:+.0} deg | E={:.3}J ({:+.1}%)",
            data.time,
            theta1.to_degrees(),
            theta2.to_degrees(),
            energy,
            drift,
        );
    }
}

/// Console HUD: periodic energy monotonicity check (every 2s sim time).
fn console_hud(
    data: Res<PhysicsData>,
    initial: Res<InitialEnergy>,
    mut prev: ResMut<PrevEnergy>,
    mut timer: ResMut<HudTimer>,
) {
    if data.time - timer.last_print_time < 2.0 {
        return;
    }
    timer.last_print_time = data.time;

    let theta1 = data.qpos[0];
    let theta2 = data.qpos[1];
    let energy = data.energy_kinetic + data.energy_potential;
    let drift = if initial.0.abs() > 1e-10 {
        (energy - initial.0) / initial.0.abs() * 100.0
    } else {
        0.0
    };

    let mono = if energy <= prev.0 + 1e-10 {
        "ok"
    } else {
        "INCREASE"
    };
    prev.0 = energy;

    println!(
        "t={:.1}s  th1={:+.1}deg  th2={:+.1}deg  E={:.4}J  drift={:+.2}%  {}",
        data.time,
        theta1.to_degrees(),
        theta2.to_degrees(),
        energy,
        drift,
        mono,
    );
}
