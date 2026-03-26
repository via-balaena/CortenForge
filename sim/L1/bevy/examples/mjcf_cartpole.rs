//! Cart-pole balancing minigame: keep the pole upright as long as you can!
//!
//! Classic inverted-pendulum control benchmark with:
//! - Mixed joint types: slide (cart) + hinge (pole)
//! - Motor actuator with gear ratio and ctrl input
//! - Joint limits (cart ±3.6m)
//! - Keyboard control (Left/Right arrows)
//! - Balance timer + session high score
//! - Restart with `R`
//!
//! Run with: `cargo run -p sim-bevy --example mjcf_cartpole --release`

#![allow(clippy::needless_pass_by_value)]
#![allow(clippy::expect_used)]
#![allow(clippy::cast_precision_loss)]

use std::f64::consts::PI;

use bevy::prelude::*;
use sim_bevy::camera::{OrbitCamera, OrbitCameraPlugin};
use sim_bevy::convert::{quat_from_unit_quaternion, vec3_from_vector};
use sim_bevy::model_data::{ModelDataPlugin, PhysicsData, PhysicsModel};
use sim_bevy::prelude::{load_model, spawn_model_geoms};
use sim_core::{ENABLE_ENERGY, MjJointType};

// ============================================================================
// MJCF Model Definition
// ============================================================================

/// Cart-pole: slide joint (cart) + hinge joint (pole) + motor on slider.
///
/// Classic inverted pendulum on a cart. The cart slides along X on a rail,
/// constrained to ±3.6m. The pole hinges at the cart (axis Y), pointing
/// upward at qpos=0. A motor actuator (gear=40) drives the cart; the user
/// controls it via keyboard (ctrl ∈ [-1, 1]).
///
/// Tuned for human playability: the pole is long (1.0m) and heavy (0.5kg)
/// with moderate hinge damping (1.0) so it falls slowly but swings
/// naturally. High slider damping (8.0) makes the cart stop quickly on
/// direction changes. Motor gear=40 gives firm but controllable pushes.
///
/// Contact is disabled (contype=0) since the cart rides a frictionless rail
/// and the pole never touches the ground. RK4 integrator for accuracy.
const CARTPOLE_MJCF: &str = r#"
<mujoco model="mjcf_cartpole">
    <compiler angle="radian" inertiafromgeom="true"/>
    <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <!-- Visual rail along X showing ±3.6m travel -->
        <geom name="rail" type="capsule" fromto="-3.6 0 0 3.6 0 0"
              size="0.02" rgba="0.5 0.5 0.5 0.8"/>

        <body name="cart" pos="0 0 0">
            <joint name="slider" type="slide" axis="1 0 0"
                   limited="true" range="-3.6 3.6" damping="8.0"/>
            <geom name="cart_geom" type="box" size="0.2 0.15 0.08"
                  mass="1.0" rgba="0.7 0.7 0 1"/>

            <body name="pole" pos="0 0 0">
                <joint name="hinge" type="hinge" axis="0 1 0"
                       damping="0.4"/>
                <geom name="pole_geom" type="capsule"
                      fromto="0 0 0 0 0 1.0" size="0.04"
                      mass="0.5" rgba="0 0.7 0.7 1"/>
            </body>
        </body>
    </worldbody>

    <actuator>
        <motor name="slide_motor" joint="slider" gear="40"
               ctrllimited="true" ctrlrange="-1 1"/>
    </actuator>
</mujoco>
"#;

/// Maximum physics steps per frame (safety cap).
const MAX_STEPS_PER_FRAME: usize = 32;

/// Initial pole tilt in radians (~2°). Barely visible but unstable.
const INITIAL_TILT: f64 = 0.03;

/// Pole angle threshold for "balanced" (±30° from vertical).
const BALANCE_THRESHOLD: f64 = 30.0 * PI / 180.0;

// ============================================================================
// Resources
// ============================================================================

/// Initial total energy for drift tracking.
#[derive(Resource)]
struct InitialEnergy(f64);

/// Whether to show joint axis gizmos.
#[derive(Resource)]
struct ShowJointAxes(bool);

/// Whether to show joint limit gizmos.
#[derive(Resource)]
struct ShowJointLimits(bool);

/// Game state: balance timer + high score.
#[derive(Resource)]
struct GameState {
    /// Sim time when the current balance streak started, or None if fallen.
    balance_start: Option<f64>,
    /// Best balance duration this session.
    high_score: f64,
    /// Last game HUD print time.
    last_print: f64,
    /// Whether we already printed the "fell" message for this streak.
    fell_printed: bool,
}

impl Default for GameState {
    fn default() -> Self {
        Self {
            balance_start: Some(0.0), // pole starts balanced
            high_score: 0.0,
            last_print: -1.0,
            fell_printed: false,
        }
    }
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
        .insert_resource(ShowJointLimits(false))
        .init_resource::<GameState>()
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (keyboard_control, restart, step_realtime, game_hud, toggle_gizmos),
        )
        .add_systems(PostUpdate, (draw_joint_axes, draw_joint_limits))
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
    let mut model = load_model(CARTPOLE_MJCF).expect("Failed to load MJCF");
    model.enableflags |= ENABLE_ENERGY;

    let mut data = model.make_data();

    // Start with pole slightly tilted — unstable, will fall
    data.qpos[1] = INITIAL_TILT;
    let _ = data.forward(&model);

    // Initial energy
    let e0 = data.energy_kinetic + data.energy_potential;

    // Auto-spawn visual entities for all geoms (rail, cart box, pole capsule)
    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[],
    );

    // Camera — side view along Bevy -Z so cart slides left/right on screen.
    // azimuth=π/2 puts camera on +Z axis, elevation=0.15 gives slight top-down.
    let orbit = OrbitCamera::new()
        .with_target(Vec3::new(0.0, 0.5, 0.0))
        .with_distance(7.0)
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
    println!("  CART-POLE BALANCING CHALLENGE");
    println!("=============================================");
    println!("Keep the pole upright as long as you can!");
    println!("---------------------------------------------");
    println!("Left/Right arrows : push cart");
    println!("R                 : restart");
    println!("J                 : toggle joint axis gizmos");
    println!("L                 : toggle joint limit gizmos");
    println!("---------------------------------------------");
    println!("Balanced = pole within 30 deg of vertical");
    println!("=============================================");

    commands.insert_resource(PhysicsModel::new(model));
    commands.insert_resource(PhysicsData::new(data));
    commands.insert_resource(InitialEnergy(e0));
}

// ============================================================================
// Systems
// ============================================================================

/// Read keyboard and set motor control input before physics step.
fn keyboard_control(keyboard: Res<ButtonInput<KeyCode>>, mut data: ResMut<PhysicsData>) {
    let ctrl = if keyboard.pressed(KeyCode::ArrowRight) {
        1.0
    } else if keyboard.pressed(KeyCode::ArrowLeft) {
        -1.0
    } else {
        0.0
    };
    data.ctrl[0] = ctrl;
}

/// Reset physics and game state on R key.
fn restart(
    keyboard: Res<ButtonInput<KeyCode>>,
    model: Res<PhysicsModel>,
    mut data: ResMut<PhysicsData>,
    mut game: ResMut<GameState>,
    mut initial: ResMut<InitialEnergy>,
) {
    if !keyboard.just_pressed(KeyCode::KeyR) {
        return;
    }

    // Reset physics
    data.qpos.fill(0.0);
    data.qvel.fill(0.0);
    data.ctrl.fill(0.0);
    data.time = 0.0;
    data.qpos[1] = INITIAL_TILT;
    let _ = data.forward(&model);

    // Reset energy baseline
    initial.0 = data.energy_kinetic + data.energy_potential;

    // Reset game timer (keep high score)
    game.balance_start = Some(0.0);
    game.last_print = -1.0;
    game.fell_printed = false;

    println!("---------------------------------------------");
    println!("  RESTART — balance the pole!");
    println!("  Best so far: {:.1}s", game.high_score);
    println!("---------------------------------------------");
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

/// Toggle gizmos: J = joint axes, L = joint limits.
fn toggle_gizmos(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut show_axes: ResMut<ShowJointAxes>,
    mut show_limits: ResMut<ShowJointLimits>,
) {
    if keyboard.just_pressed(KeyCode::KeyJ) {
        show_axes.0 = !show_axes.0;
        println!(
            "Joint axis gizmos: {}",
            if show_axes.0 { "ON" } else { "OFF" }
        );
    }
    if keyboard.just_pressed(KeyCode::KeyL) {
        show_limits.0 = !show_limits.0;
        println!(
            "Joint limit gizmos: {}",
            if show_limits.0 { "ON" } else { "OFF" }
        );
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

/// Draw joint limit gizmos for the slider joint.
///
/// Orange vertical lines at ±3.6m, green marker at current cart position.
fn draw_joint_limits(
    show: Res<ShowJointLimits>,
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut gizmos: Gizmos,
) {
    if !show.0 {
        return;
    }

    let limit_color = Color::srgb(1.0, 0.5, 0.0);

    for jnt_id in 0..model.njnt {
        if !model.jnt_limited[jnt_id] {
            continue;
        }

        // Only draw limit gizmos for slide joints
        if model.jnt_type[jnt_id] != MjJointType::Slide {
            continue;
        }

        let body_id = model.jnt_body[jnt_id];
        let (lo, hi) = model.jnt_range[jnt_id];
        let axis_local = &model.jnt_axis[jnt_id];

        let body_quat = quat_from_unit_quaternion(&data.xquat[body_id]);
        let axis_world = body_quat * vec3_from_vector(axis_local);

        // Slide limits are positions along the axis from the joint origin.
        // Parent body position is the reference for the slider.
        let parent_id = model.body_parent[body_id];
        let parent_pos = vec3_from_vector(&data.xpos[parent_id]);

        let lo_pos = parent_pos + axis_world * lo as f32;
        let hi_pos = parent_pos + axis_world * hi as f32;

        // Vertical lines at limit positions (Bevy Y-up)
        let line_half = 0.3;
        let up = Vec3::Y;
        gizmos.line(
            lo_pos - up * line_half,
            lo_pos + up * line_half,
            limit_color,
        );
        gizmos.line(
            hi_pos - up * line_half,
            hi_pos + up * line_half,
            limit_color,
        );

        // Spheres at limit endpoints
        gizmos.sphere(Isometry3d::from_translation(lo_pos), 0.03, limit_color);
        gizmos.sphere(Isometry3d::from_translation(hi_pos), 0.03, limit_color);

        // Current position marker (green)
        let qpos_adr = model.jnt_qpos_adr[jnt_id];
        let current = data.qpos[qpos_adr];
        let cur_pos = parent_pos + axis_world * current as f32;
        gizmos.sphere(
            Isometry3d::from_translation(cur_pos),
            0.02,
            Color::srgb(0.2, 0.9, 0.2),
        );
    }
}

/// Normalize angle to [-π, π].
fn normalize_angle(angle: f64) -> f64 {
    let a = angle % (2.0 * PI);
    if a > PI {
        a - 2.0 * PI
    } else if a < -PI {
        a + 2.0 * PI
    } else {
        a
    }
}

/// Game HUD: updates the window title with balance timer, high score, and status.
fn game_hud(data: Res<PhysicsData>, mut game: ResMut<GameState>, mut windows: Query<&mut Window>) {
    let pole_angle = normalize_angle(data.qpos[1]);
    let is_balanced = pole_angle.abs() < BALANCE_THRESHOLD;

    if is_balanced {
        // Start or continue balance streak
        let start = *game.balance_start.get_or_insert(data.time);
        let duration = data.time - start;
        game.fell_printed = false;

        if duration > game.high_score {
            game.high_score = duration;
        }

        let best_tag = if duration >= game.high_score - 0.01 && duration > 1.0 {
            "  NEW BEST!"
        } else {
            ""
        };

        if let Ok(mut window) = windows.single_mut() {
            window.title = format!(
                "Cart-Pole | Balancing: {:.1}s | Best: {:.1}s | pole={:+.0}deg{}",
                duration,
                game.high_score,
                pole_angle.to_degrees(),
                best_tag,
            );
        }
    } else {
        // Just fell
        if !game.fell_printed {
            let duration = if let Some(start) = game.balance_start.take() {
                data.time - start
            } else {
                0.0
            };
            game.fell_printed = true;

            if let Ok(mut window) = windows.single_mut() {
                window.title = format!(
                    "Cart-Pole | FELL! Lasted {:.1}s | Best: {:.1}s | Press R to restart",
                    duration, game.high_score,
                );
            }
        }
        game.balance_start = None;
    }
}
