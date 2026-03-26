//! 2-DOF planar robot arm: FK verification against analytical formula.
//!
//! Two-link arm with motor actuators and keyboard control. The window title
//! shows measured end-effector position vs analytical forward kinematics:
//!   x = L₁cos(θ₁) + L₂cos(θ₁+θ₂)
//!   z = L₁sin(θ₁) + L₂sin(θ₁+θ₂)
//!
//! Parameters: L₁=1.0m, L₂=0.8m, workspace annulus r∈[0.2, 1.8].
//!
//! Controls: Q/A = shoulder, W/S = elbow, R = reset
//!
//! Run with: `cargo run -p sim-bevy --example 2dof_arm --release`

#![allow(clippy::needless_pass_by_value)]
#![allow(clippy::expect_used)]
#![allow(clippy::cast_precision_loss)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCamera;
use sim_bevy::model_data::{ModelDataPlugin, PhysicsData, PhysicsModel};
use sim_bevy::prelude::{SimViewerPlugin, load_model, spawn_model_geoms};

// ============================================================================
// MJCF Model Definition
// ============================================================================

/// 2-DOF planar arm: 2 hinge joints (axis -Y), 2 motor actuators.
///
/// Upper arm (L₁=1.0m, 1.0 kg) and forearm (L₂=0.8m, 0.5 kg) in the XZ plane
/// (physics frame). Shoulder pivot at (0, 0, 1.5). At qpos=(0,0) both links
/// point along +X (horizontal). No gravity — the arm holds its position and
/// only moves under motor torque (gear 80/40). High damping (10/5) for
/// snappy, servo-like response — stops immediately when keys are released.
///
/// Hinge axis is -Y so that positive θ rotates from +X toward +Z (upward),
/// matching the standard robotics FK convention:
///   x = L₁cos(θ₁) + L₂cos(θ₁+θ₂)
///   z = L₁sin(θ₁) + L₂sin(θ₁+θ₂)
///
/// Joint limits: shoulder ±π, elbow ±2.6 rad (~149°). Elbow limit prevents
/// the forearm from folding back through the upper arm.
///
/// Contact disabled — pure articulated kinematics with motor control. RK4
/// integrator for accuracy.
const ARM_MJCF: &str = r#"
<mujoco model="2dof_arm">
    <compiler angle="radian" inertiafromgeom="true"/>
    <option gravity="0 0 0" timestep="0.002" integrator="RK4"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <body name="upper_arm" pos="0 0 1.5">
            <joint name="shoulder" type="hinge" axis="0 -1 0"
                   limited="true" range="-3.14159 3.14159" damping="10.0"/>
            <geom name="upper_geom" type="capsule" fromto="0 0 0 1.0 0 0"
                  size="0.04" mass="1.0" rgba="0.3 0.5 0.85 1"/>

            <body name="forearm" pos="1.0 0 0">
                <joint name="elbow" type="hinge" axis="0 -1 0"
                       limited="true" range="-2.6 2.6" damping="5.0"/>
                <geom name="forearm_geom" type="capsule" fromto="0 0 0 0.8 0 0"
                      size="0.035" mass="0.5" rgba="0.85 0.25 0.2 1"/>
            </body>
        </body>
    </worldbody>

    <actuator>
        <motor name="shoulder_motor" joint="shoulder" gear="80"
               ctrllimited="true" ctrlrange="-1 1"/>
        <motor name="elbow_motor" joint="elbow" gear="40"
               ctrllimited="true" ctrlrange="-1 1"/>
    </actuator>
</mujoco>
"#;

/// Maximum physics steps per frame (safety cap).
const MAX_STEPS_PER_FRAME: usize = 32;

// Link lengths
const L1: f64 = 1.0;
const L2: f64 = 0.8;

// Shoulder position in physics frame
const SHOULDER_Z: f64 = 1.5;

// Home pose angles (horizontal, straight)
const HOME_THETA1: f64 = 0.0;
const HOME_THETA2: f64 = 0.0;

// Body indices in MJCF tree (worldbody=0, upper_arm=1, forearm=2)
const FOREARM_BODY: usize = 2;

// ============================================================================
// Resources
// ============================================================================

/// End-effector trail for gizmo drawing.
#[derive(Resource)]
struct Trail {
    /// Recent tip positions in Bevy coordinates.
    points: Vec<Vec3>,
}

impl Default for Trail {
    fn default() -> Self {
        Self {
            points: Vec::with_capacity(500),
        }
    }
}

/// Timer for periodic console output.
#[derive(Resource)]
struct HudTimer {
    last_print_time: f64,
}

/// Whether to show the controls hint in the window title.
#[derive(Resource)]
struct ShowControls(bool);

// ============================================================================
// Main
// ============================================================================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(
            SimViewerPlugin::new()
                .without_camera()
                .without_lighting(),
        )
        .add_plugins(ModelDataPlugin::new()) // no auto_step — wall-clock stepping
        .init_resource::<Trail>()
        .insert_resource(ShowControls(true))
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (keyboard_control, restart, step_realtime, record_trail),
        )
        .add_systems(
            PostUpdate,
            (window_hud, console_hud, draw_trail, draw_ee_marker),
        )
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
    let model = load_model(ARM_MJCF).expect("Failed to load MJCF");
    let mut data = model.make_data();

    // Set home pose
    data.qpos[0] = HOME_THETA1;
    data.qpos[1] = HOME_THETA2;
    let _ = data.forward(&model);

    // Auto-spawn visual entities for all geoms (upper arm + forearm capsules)
    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[],
    );

    // Shoulder pivot marker (small gray sphere)
    // Physics (0, 0, 1.5) → Bevy (0, 1.5, 0)
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.06))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.4, 0.4, 0.4),
            metallic: 0.6,
            ..default()
        })),
        Transform::from_xyz(0.0, SHOULDER_Z as f32, 0.0),
    ));

    // Camera — side view (azimuth=π/2 puts camera on Bevy +Z, looking along -Z,
    // so we see the Bevy XY plane = physics XZ plane where the arm moves).
    let orbit = OrbitCamera::new()
        .with_target(Vec3::new(0.0, SHOULDER_Z as f32, 0.0))
        .with_distance(5.5)
        .with_angles(std::f32::consts::FRAC_PI_2, 0.05);
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

    commands.insert_resource(PhysicsModel::new(model));
    commands.insert_resource(PhysicsData::new(data));
    commands.insert_resource(HudTimer {
        last_print_time: -1.0,
    });
}

// ============================================================================
// Systems
// ============================================================================

/// Read keyboard and set motor control inputs.
/// Dismiss controls hint on first keypress.
fn keyboard_control(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut data: ResMut<PhysicsData>,
    mut show: ResMut<ShowControls>,
) {
    // Shoulder: Q = positive (CCW), A = negative (CW)
    let ctrl0 = if keyboard.pressed(KeyCode::KeyQ) {
        1.0
    } else if keyboard.pressed(KeyCode::KeyA) {
        -1.0
    } else {
        0.0
    };
    // Elbow: W = positive (CCW), S = negative (CW)
    let ctrl1 = if keyboard.pressed(KeyCode::KeyW) {
        1.0
    } else if keyboard.pressed(KeyCode::KeyS) {
        -1.0
    } else {
        0.0
    };
    data.ctrl[0] = ctrl0;
    data.ctrl[1] = ctrl1;

    // Dismiss controls hint once the user starts interacting
    if show.0 && (ctrl0 != 0.0 || ctrl1 != 0.0) {
        show.0 = false;
    }
}

/// Reset to home pose on R key.
fn restart(
    keyboard: Res<ButtonInput<KeyCode>>,
    model: Res<PhysicsModel>,
    mut data: ResMut<PhysicsData>,
    mut trail: ResMut<Trail>,
    mut timer: ResMut<HudTimer>,
) {
    if !keyboard.just_pressed(KeyCode::KeyR) {
        return;
    }

    data.qpos.fill(0.0);
    data.qvel.fill(0.0);
    data.ctrl.fill(0.0);
    data.time = 0.0;
    data.qpos[0] = HOME_THETA1;
    data.qpos[1] = HOME_THETA2;
    let _ = data.forward(&model);

    trail.points.clear();
    timer.last_print_time = -1.0;
}

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

/// Compute end-effector tip position in physics frame from simulation data.
///
/// Uses the forearm body's xpos (= elbow world position) and xmat (= forearm
/// orientation) to transform the local tip offset (L₂, 0, 0) into world frame.
fn measured_tip(data: &PhysicsData) -> (f64, f64) {
    let elbow = &data.xpos[FOREARM_BODY];
    let mat = &data.xmat[FOREARM_BODY];
    // tip = elbow + forearm_rotation * (L2, 0, 0)
    // Matrix * (L2, 0, 0) = first column * L2
    let tip_x = elbow.x + mat[(0, 0)] * L2;
    let tip_z = elbow.z + mat[(2, 0)] * L2;
    (tip_x, tip_z)
}

/// Compute FK-predicted end-effector position in physics frame.
fn fk_predicted(theta1: f64, theta2: f64) -> (f64, f64) {
    let x = L1 * theta1.cos() + L2 * (theta1 + theta2).cos();
    let z = SHOULDER_Z + L1 * theta1.sin() + L2 * (theta1 + theta2).sin();
    (x, z)
}

/// Record end-effector trail points.
fn record_trail(data: Res<PhysicsData>, mut trail: ResMut<Trail>) {
    let (tip_x, tip_z) = measured_tip(&data);
    // Physics (x, _, z) → Bevy (x, z, 0) — arm at physics y=0
    let bevy_pos = Vec3::new(tip_x as f32, tip_z as f32, 0.0);

    // Only record if moved enough to be visible
    let should_record = trail
        .points
        .last()
        .is_none_or(|last| last.distance(bevy_pos) > 0.005);
    if should_record {
        trail.points.push(bevy_pos);
        if trail.points.len() > 500 {
            trail.points.drain(0..100);
        }
    }
}

/// Draw end-effector trail as gizmo lines.
fn draw_trail(trail: Res<Trail>, mut gizmos: Gizmos) {
    if trail.points.len() < 2 {
        return;
    }

    let n = trail.points.len();
    for (i, window) in trail.points.windows(2).enumerate() {
        // Fade older segments
        let alpha = (i as f32 + 1.0) / n as f32;
        gizmos.line(
            window[0],
            window[1],
            Color::srgba(1.0, 0.8, 0.0, alpha * 0.7),
        );
    }
}

/// Draw end-effector marker sphere at current tip.
fn draw_ee_marker(data: Res<PhysicsData>, mut gizmos: Gizmos) {
    let (tip_x, tip_z) = measured_tip(&data);
    let bevy_pos = Vec3::new(tip_x as f32, tip_z as f32, 0.0);
    gizmos.sphere(
        Isometry3d::from_translation(bevy_pos),
        0.04,
        Color::srgb(1.0, 0.8, 0.0),
    );
}

/// Window title HUD: shows controls initially, then switches to FK data.
fn window_hud(data: Res<PhysicsData>, show: Res<ShowControls>, mut windows: Query<&mut Window>) {
    let theta1 = data.qpos[0];
    let theta2 = data.qpos[1];

    let (tip_x_meas, tip_z_meas) = measured_tip(&data);
    let (tip_x_fk, tip_z_fk) = fk_predicted(theta1, theta2);

    let error = ((tip_x_meas - tip_x_fk).powi(2) + (tip_z_meas - tip_z_fk).powi(2)).sqrt();

    let dx = tip_x_meas;
    let dz = tip_z_meas - SHOULDER_Z;
    let radius = (dx * dx + dz * dz).sqrt();

    if let Ok(mut window) = windows.single_mut() {
        if show.0 {
            window.title = "2-DOF Arm | Q/A = shoulder | W/S = elbow | R = reset".to_string();
        } else {
            window.title = format!(
                "2-DOF Arm | t1={:+.0} t2={:+.0} deg | EE({:.3},{:.3}) FK({:.3},{:.3}) |err|={:.1e} | r={:.2}",
                theta1.to_degrees(),
                theta2.to_degrees(),
                tip_x_meas,
                tip_z_meas,
                tip_x_fk,
                tip_z_fk,
                error,
                radius,
            );
        }
    }
}

/// Console HUD: periodic detailed output (every 2s sim time).
fn console_hud(data: Res<PhysicsData>, mut timer: ResMut<HudTimer>) {
    if data.time - timer.last_print_time < 2.0 {
        return;
    }
    timer.last_print_time = data.time;

    let theta1 = data.qpos[0];
    let theta2 = data.qpos[1];

    let (tip_x_meas, tip_z_meas) = measured_tip(&data);
    let (tip_x_fk, tip_z_fk) = fk_predicted(theta1, theta2);

    let error = ((tip_x_meas - tip_x_fk).powi(2) + (tip_z_meas - tip_z_fk).powi(2)).sqrt();

    let dx = tip_x_meas;
    let dz = tip_z_meas - SHOULDER_Z;
    let radius = (dx * dx + dz * dz).sqrt();

    println!(
        "t={:.1}s  t1={:+.1}deg  t2={:+.1}deg  EE=({:.4},{:.4})  FK=({:.4},{:.4})  |err|={:.2e}  r={:.3}",
        data.time,
        theta1.to_degrees(),
        theta2.to_degrees(),
        tip_x_meas,
        tip_z_meas,
        tip_x_fk,
        tip_z_fk,
        error,
        radius,
    );
}
