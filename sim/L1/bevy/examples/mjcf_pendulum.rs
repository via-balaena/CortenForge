//! Simplest articulated MJCF model: a single hinge pendulum with spring and damper.
//!
//! First example where a joint is *not* a freejoint. Demonstrates:
//! - Hinge joint from MJCF (axis Y)
//! - Passive spring (stiffness=2) + damper (0.3)
//! - Energy tracking (monotonically decreasing in damped system)
//! - Joint axis gizmo (toggle with `J`)
//!
//! Based on conformance `pendulum.xml` with visual-friendly sizes.
//! Link hangs downward so gravity acts as the restoring force and the
//! full swing arc is visible.
//!
//! Run with: `cargo run -p sim-bevy --example mjcf_pendulum --release`

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

/// Single hinge pendulum with spring and damper.
///
/// Based on conformance `pendulum.xml` with visual-friendly sizes.
/// Link hangs DOWNWARD (along -Z in physics frame) so at qpos=0 the
/// pendulum is at rest hanging straight down. Gravity acts as the primary
/// restoring force giving large, visible swing arcs. The spring provides
/// additional stiffness around springref.
///
/// - 1 body with hinge joint (axis Y), pivot at Z=2
/// - Spring: stiffness=2, springref=0 (equilibrium at hanging-down)
/// - Damper: damping=0.3
///
/// Spring parameters are scaled from conformance values (stiffness=10,
/// damping=0.5 for a 0.3m capsule) to match the ~20x larger inertia of
/// the visual-friendly geometry. springref=0 aligns the spring equilibrium
/// with gravity's (straight down) for symmetric, natural-looking swing.
const PENDULUM_MJCF: &str = r#"
<mujoco model="mjcf_pendulum">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.002"/>

    <worldbody>
        <!-- Pivot at Z=2, link hangs down along -Z -->
        <body name="link" pos="0 0 2">
            <joint name="hinge" type="hinge" axis="0 1 0"
                   stiffness="2" damping="0.3" springref="0"/>
            <geom name="link_geom" type="capsule" fromto="0 0 0 0 0 -0.8"
                  size="0.04" mass="1.0" rgba="0.3 0.55 0.85 1"/>
            <geom name="bob" type="sphere" pos="0 0 -0.8"
                  size="0.08" mass="0.5" rgba="0.9 0.25 0.2 1"/>
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

/// Whether to show joint axis gizmo.
#[derive(Resource)]
struct ShowJointAxis(bool);

/// Previous total energy for monotonicity check.
#[derive(Resource)]
struct PrevEnergy(f64);

/// Timer for periodic HUD printing.
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
        .add_plugins(ModelDataPlugin::new()) // no auto_step — we do substeps
        .insert_resource(ShowJointAxis(false))
        .add_systems(Startup, setup)
        .add_systems(Update, (step_realtime, toggle_joint_gizmo))
        .add_systems(PostUpdate, (draw_joint_axis, hud_info))
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
    let mut model = load_model(PENDULUM_MJCF).expect("Failed to load MJCF");
    model.enableflags |= ENABLE_ENERGY;

    let mut data = model.make_data();

    // Start at π/2 (90°) from hanging — dramatic visible swing
    data.qpos[0] = std::f64::consts::FRAC_PI_2;
    let _ = data.forward(&model);

    // Initial energy
    let e0 = data.energy_kinetic + data.energy_potential;

    // Auto-spawn visual entities for all geoms (capsule link + sphere bob)
    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[],
    );

    // Pivot marker (small gray sphere at joint origin)
    // Pivot is at (0, 0, 2) in physics Z-up → (0, 2, 0) in Bevy Y-up
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.06))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.4, 0.4, 0.4),
            metallic: 0.6,
            ..default()
        })),
        Transform::from_xyz(0.0, 2.0, 0.0),
    ));

    // Camera — orbit target at pivot height
    let orbit = OrbitCamera::new()
        .with_target(Vec3::new(0.0, 1.5, 0.0))
        .with_distance(5.0)
        .with_angles(0.0, 0.2);
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
    println!("MJCF Pendulum — Articulated Hinge + Spring/Damper");
    println!("=============================================");
    println!("Joint: hinge (axis Y), stiffness=2, damping=0.3, springref=0");
    println!("Initial angle: 90° from hanging, link swings in XZ plane");
    println!("---------------------------------------------");
    println!("Press 'J' to toggle joint axis gizmo (yellow line)");
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
/// Computes how many dt=0.002 steps fit in the frame's real elapsed time,
/// so the pendulum runs at 1:1 real-time regardless of frame rate.
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

/// Toggle joint axis gizmo with J key.
fn toggle_joint_gizmo(keyboard: Res<ButtonInput<KeyCode>>, mut show: ResMut<ShowJointAxis>) {
    if keyboard.just_pressed(KeyCode::KeyJ) {
        show.0 = !show.0;
        println!("Joint axis gizmo: {}", if show.0 { "ON" } else { "OFF" });
    }
}

/// Draw joint axis gizmo (yellow line along joint axis at pivot).
fn draw_joint_axis(
    show: Res<ShowJointAxis>,
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

        // Get body world position and orientation
        let body_pos = vec3_from_vector(&data.xpos[body_id]);
        let body_quat = quat_from_unit_quaternion(&data.xquat[body_id]);

        // Transform axis to world frame
        let axis_world = body_quat * vec3_from_vector(axis_local);

        let half_len = 0.4;
        let start = body_pos - axis_world * half_len;
        let end = body_pos + axis_world * half_len;

        // Yellow line for joint axis
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

/// HUD: angle, velocity, energy — printed every 2s of sim time.
fn hud_info(
    data: Res<PhysicsData>,
    initial: Res<InitialEnergy>,
    mut prev: ResMut<PrevEnergy>,
    mut timer: ResMut<HudTimer>,
) {
    if data.time - timer.last_print_time < 2.0 {
        return;
    }
    timer.last_print_time = data.time;

    let angle = data.qpos[0];
    let velocity = data.qvel[0];
    let energy = data.energy_kinetic + data.energy_potential;
    let energy_drift = if initial.0.abs() > 1e-10 {
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
        "t={:.1}s  angle={:+.1}deg  vel={:+.2}rad/s  E={:.4}J  drift={:+.2}%  {}",
        data.time,
        angle.to_degrees(),
        velocity,
        energy,
        energy_drift,
        mono,
    );
}
