//! Crank-slider mechanism: motor-driven crank converts rotary to linear motion.
//!
//! Classic mechanism with analytically exact piston position:
//!   x(θ) = r·cos(θ) + √(L² − r²·sin²(θ))
//! where r = crank radius, L = connecting rod length, θ = crank angle.
//!
//! Parameters: r=0.3 m, L=0.8 m, motor gear=5, crank damping=0.8
//!   → terminal ω ≈ 6.25 rad/s (~60 RPM)
//!
//! Controls: R = reset
//!
//! Run with: `cargo run -p sim-bevy --example crank_slider --release`

#![allow(clippy::needless_pass_by_value)]
#![allow(clippy::expect_used)]
#![allow(clippy::cast_precision_loss)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCamera;
use sim_bevy::model_data::{ModelDataPlugin, PhysicsData, PhysicsModel};
use sim_bevy::prelude::{SimViewerPlugin, load_model};

// ============================================================================
// MJCF Model Definition
// ============================================================================

/// Crank-slider: crank arm (hinge + motor), connecting rod (freejoint with
/// equality connect constraints at both ends), slider (slide joint along X).
///
/// Bodies:
///   - crank: hinge at origin, axis Y, motor-driven, damping for terminal velocity
///   - rod: freejoint, connect-constrained to crank tip and slider
///   - slider: slide joint along X axis
///
/// Connect constraint semantics: anchor point in body1 frame coincides with
/// body2 origin. So:
///   - connect body1="crank" anchor="0.3 0 0" body2="rod" → crank tip = rod origin
///   - connect body1="rod" anchor="0.8 0 0" body2="slider" → rod far end = slider
///
/// No gravity. RK4 integrator, dt=0.001. Contact disabled.
/// Geom alpha=0: visuals drawn entirely with gizmos.
const CRANK_SLIDER_MJCF: &str = r#"
<mujoco model="crank_slider">
    <compiler angle="radian" inertiafromgeom="false"/>
    <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <body name="crank" pos="0 0 0">
            <joint name="crank_hinge" type="hinge" axis="0 1 0" damping="0.8"/>
            <inertial pos="0.15 0 0" mass="0.5"
                      diaginertia="0.001 0.004 0.004"/>
            <geom name="crank_arm" type="capsule" fromto="0 0 0 0.3 0 0"
                  size="0.02" rgba="0.3 0.5 0.85 0"/>
        </body>

        <body name="rod" pos="0.3 0 0">
            <freejoint name="rod_free"/>
            <inertial pos="0.4 0 0" mass="0.3"
                      diaginertia="0.001 0.016 0.016"/>
            <geom name="rod_geom" type="capsule" fromto="0 0 0 0.8 0 0"
                  size="0.015" rgba="0.5 0.5 0.5 0"/>
        </body>

        <body name="slider" pos="1.1 0 0">
            <joint name="slider_joint" type="slide" axis="1 0 0"/>
            <inertial pos="0 0 0" mass="0.5"
                      diaginertia="0.001 0.001 0.001"/>
            <geom name="slider_geom" type="box" size="0.05 0.03 0.05"
                  rgba="0.85 0.25 0.2 0"/>
        </body>
    </worldbody>

    <equality>
        <connect body1="crank" body2="rod" anchor="0.3 0 0"
                 solref="0.002 1.0"/>
        <connect body1="rod" body2="slider" anchor="0.8 0 0"
                 solref="0.002 1.0"/>
    </equality>

    <actuator>
        <motor name="crank_motor" joint="crank_hinge" gear="5"/>
    </actuator>
</mujoco>
"#;

/// Maximum physics steps per frame (safety cap).
const MAX_STEPS_PER_FRAME: usize = 64;

/// Crank radius (m).
const CRANK_R: f64 = 0.3;

/// Connecting rod length (m).
const ROD_L: f64 = 0.8;

/// Body indices (worldbody=0).
const CRANK_BODY: usize = 1;
const ROD_BODY: usize = 2;
const SLIDER_BODY: usize = 3;

// ============================================================================
// Resources
// ============================================================================

/// Timer for periodic console output.
#[derive(Resource)]
struct HudTimer {
    last_print_time: f64,
}

/// Whether to show controls hint in window title.
#[derive(Resource)]
struct ShowControls(bool);

// ============================================================================
// Main
// ============================================================================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(SimViewerPlugin::new().without_camera().without_lighting())
        .add_plugins(ModelDataPlugin::new())
        .insert_resource(ShowControls(true))
        .add_systems(Startup, setup)
        .add_systems(Update, (restart, step_realtime))
        .add_systems(PostUpdate, (window_hud, console_hud, draw_mechanism))
        .run();
}

// ============================================================================
// Setup
// ============================================================================

fn setup(mut commands: Commands) {
    let model = load_model(CRANK_SLIDER_MJCF).expect("Failed to load MJCF");
    let mut data = model.make_data();

    // Initial configuration set by MJCF body positions
    let _ = data.forward(&model);

    // Start motor: constant ctrl=1.0 → torque = gear × ctrl = 5 N·m
    // With damping=0.8, terminal ω ≈ 6.25 rad/s ≈ 60 RPM
    data.ctrl[0] = 1.0;

    // Camera — side view (along -Y physics = -Z Bevy), centered on mechanism
    let cam_target = Vec3::new(0.5, 0.0, 0.0);
    let orbit = OrbitCamera::new()
        .with_target(cam_target)
        .with_distance(3.0)
        .with_angles(std::f32::consts::FRAC_PI_2, 0.0);
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
    println!("  CRANK-SLIDER — Analytical Verification");
    println!("=============================================");
    println!("r={CRANK_R:.2} m, L={ROD_L:.2} m");
    println!("x(θ) = r·cos(θ) + √(L² − r²·sin²(θ))");
    println!("Motor: gear=5, damping=0.8 → terminal ~60 RPM");
    println!("Equality constraints: {}", model.neq);
    println!("---------------------------------------------");
    println!("Press 'R' to reset");
    println!("=============================================");

    commands.insert_resource(PhysicsModel::new(model));
    commands.insert_resource(PhysicsData::new(data));
    commands.insert_resource(HudTimer {
        last_print_time: -1.0,
    });
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Analytical piston position: x(θ) = r·cos(θ) + √(L² − r²·sin²(θ))
fn predicted_x(theta: f64) -> f64 {
    CRANK_R * theta.cos() + (ROD_L * ROD_L - CRANK_R * CRANK_R * theta.sin().powi(2)).sqrt()
}

/// Convert physics (x, y, z) Z-up to Bevy (x, z, y) Y-up.
fn to_bevy(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(x as f32, z as f32, y as f32)
}

/// Compute crank tip position in physics frame from xpos/xmat.
fn crank_tip_world(data: &PhysicsData) -> (f64, f64, f64) {
    let pivot = &data.xpos[CRANK_BODY];
    let mat = &data.xmat[CRANK_BODY];
    (
        pivot.x + mat[(0, 0)] * CRANK_R,
        pivot.y + mat[(1, 0)] * CRANK_R,
        pivot.z + mat[(2, 0)] * CRANK_R,
    )
}

/// Compute rod far-end position in physics frame from xpos/xmat.
fn rod_far_end_world(data: &PhysicsData) -> (f64, f64, f64) {
    let origin = &data.xpos[ROD_BODY];
    let mat = &data.xmat[ROD_BODY];
    (
        origin.x + mat[(0, 0)] * ROD_L,
        origin.y + mat[(1, 0)] * ROD_L,
        origin.z + mat[(2, 0)] * ROD_L,
    )
}

/// Distance between two 3D points.
fn dist3(a: (f64, f64, f64), b: (f64, f64, f64)) -> f64 {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    let dz = a.2 - b.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
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

/// Reset physics on R key.
fn restart(
    keyboard: Res<ButtonInput<KeyCode>>,
    model: Res<PhysicsModel>,
    mut data: ResMut<PhysicsData>,
    mut timer: ResMut<HudTimer>,
    mut show: ResMut<ShowControls>,
) {
    if !keyboard.just_pressed(KeyCode::KeyR) {
        return;
    }

    data.qpos.fill(0.0);
    data.qvel.fill(0.0);
    data.ctrl.fill(0.0);
    data.time = 0.0;

    // Restore freejoint initial state
    // qpos layout: [0]=crank_hinge, [1..4]=rod_pos, [4..8]=rod_quat, [8]=slider
    data.qpos[1] = 0.3; // rod pos.x (at crank tip when θ=0)
    data.qpos[4] = 1.0; // rod quat.w (identity)

    let _ = data.forward(&model);

    // Restart motor
    data.ctrl[0] = 1.0;

    timer.last_print_time = -1.0;
    show.0 = false;

    println!("---------------------------------------------");
    println!("  RESET");
    println!("---------------------------------------------");
}

/// Draw the crank-slider mechanism with gizmos.
fn draw_mechanism(data: Res<PhysicsData>, mut gizmos: Gizmos) {
    // --- Crank arm ---
    let pivot = &data.xpos[CRANK_BODY];
    let pivot_b = to_bevy(pivot.x, pivot.y, pivot.z);
    let tip = crank_tip_world(&data);
    let tip_b = to_bevy(tip.0, tip.1, tip.2);

    let crank_color = Color::srgb(0.3, 0.5, 0.85);
    gizmos.line(pivot_b, tip_b, crank_color);
    // Thicken crank arm with parallel lines
    let crank_mat = &data.xmat[CRANK_BODY];
    let cy = (crank_mat[(0, 1)], crank_mat[(1, 1)], crank_mat[(2, 1)]);
    let cz = (crank_mat[(0, 2)], crank_mat[(1, 2)], crank_mat[(2, 2)]);
    let offset = 0.012_f64;
    for &(dy, dz) in &[(1.0_f64, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
        let off = to_bevy(
            cy.0 * dy * offset + cz.0 * dz * offset,
            cy.1 * dy * offset + cz.1 * dz * offset,
            cy.2 * dy * offset + cz.2 * dz * offset,
        );
        gizmos.line(pivot_b + off, tip_b + off, crank_color);
    }

    // --- Connecting rod ---
    let rod_origin = &data.xpos[ROD_BODY];
    let rod_start_b = to_bevy(rod_origin.x, rod_origin.y, rod_origin.z);
    let rod_end = rod_far_end_world(&data);
    let rod_end_b = to_bevy(rod_end.0, rod_end.1, rod_end.2);

    let rod_color = Color::srgb(0.6, 0.6, 0.6);
    gizmos.line(rod_start_b, rod_end_b, rod_color);
    // Thicken rod
    let rod_mat = &data.xmat[ROD_BODY];
    let ry = (rod_mat[(0, 1)], rod_mat[(1, 1)], rod_mat[(2, 1)]);
    let rz = (rod_mat[(0, 2)], rod_mat[(1, 2)], rod_mat[(2, 2)]);
    let rod_offset = 0.008_f64;
    for &(dy, dz) in &[(1.0_f64, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
        let off = to_bevy(
            ry.0 * dy * rod_offset + rz.0 * dz * rod_offset,
            ry.1 * dy * rod_offset + rz.1 * dz * rod_offset,
            ry.2 * dy * rod_offset + rz.2 * dz * rod_offset,
        );
        gizmos.line(rod_start_b + off, rod_end_b + off, rod_color);
    }

    // --- Slider (box outline in XZ physics plane = XY Bevy plane) ---
    let slider_pos = &data.xpos[SLIDER_BODY];
    let slider_b = to_bevy(slider_pos.x, slider_pos.y, slider_pos.z);
    let bs = 0.06_f32;
    let slider_color = Color::srgb(0.85, 0.25, 0.2);
    let corners = [
        slider_b + Vec3::new(-bs, -bs, 0.0),
        slider_b + Vec3::new(bs, -bs, 0.0),
        slider_b + Vec3::new(bs, bs, 0.0),
        slider_b + Vec3::new(-bs, bs, 0.0),
    ];
    for i in 0..4 {
        gizmos.line(corners[i], corners[(i + 1) % 4], slider_color);
    }

    // --- Joint pins ---
    let pin_color = Color::srgb(0.9, 0.9, 0.9);
    gizmos.sphere(Isometry3d::from_translation(pivot_b), 0.025, pin_color);
    gizmos.sphere(Isometry3d::from_translation(tip_b), 0.02, pin_color);
    gizmos.sphere(Isometry3d::from_translation(rod_end_b), 0.02, pin_color);

    // --- Crank circle (swept path) ---
    let crank_circle_iso =
        Isometry3d::new(pivot_b, Quat::from_rotation_x(std::f32::consts::FRAC_PI_2));
    gizmos.circle(
        crank_circle_iso,
        CRANK_R as f32,
        Color::srgba(0.3, 0.5, 0.85, 0.2),
    );

    // --- Slider rail ---
    let rail_color = Color::srgba(0.4, 0.4, 0.4, 0.4);
    gizmos.line(to_bevy(0.3, 0.0, 0.0), to_bevy(1.3, 0.0, 0.0), rail_color);
    // Rail ticks
    for &x in &[0.5, 0.8, 1.1] {
        let tick = to_bevy(x, 0.0, 0.0);
        gizmos.line(
            tick + Vec3::new(0.0, -0.03, 0.0),
            tick + Vec3::new(0.0, 0.03, 0.0),
            rail_color,
        );
    }
}

/// Window title HUD: controls initially, then live data.
fn window_hud(data: Res<PhysicsData>, show: Res<ShowControls>, mut windows: Query<&mut Window>) {
    let Ok(mut window) = windows.single_mut() else {
        return;
    };

    if show.0 && data.time < 2.0 {
        window.title = "Crank-Slider | R = reset | Orbit camera".to_string();
        return;
    }

    let theta = data.qpos[0];
    let x_meas = data.xpos[SLIDER_BODY].x;
    let x_pred = predicted_x(theta);
    let error = (x_meas - x_pred).abs();
    let omega = data.qvel[0];
    let rpm = omega * 60.0 / (2.0 * std::f64::consts::PI);

    window.title = format!(
        "Crank-Slider | t={:.1}s | θ={:.0}° | x={:.4} pred={:.4} |err|={:.2e} | {:.0} RPM",
        data.time,
        theta.to_degrees(),
        x_meas,
        x_pred,
        error,
        rpm,
    );
}

/// Console HUD: periodic detailed output with constraint violation diagnostics.
fn console_hud(data: Res<PhysicsData>, mut timer: ResMut<HudTimer>) {
    if data.time - timer.last_print_time < 2.0 {
        return;
    }
    timer.last_print_time = data.time;

    let theta = data.qpos[0];
    let x_meas = data.xpos[SLIDER_BODY].x;
    let x_pred = predicted_x(theta);
    let error = (x_meas - x_pred).abs();
    let omega = data.qvel[0];
    let rpm = omega * 60.0 / (2.0 * std::f64::consts::PI);

    // Constraint violation: crank tip vs rod origin, rod far end vs slider
    let tip = crank_tip_world(&data);
    let rod_origin = &data.xpos[ROD_BODY];
    let c1_err = dist3(tip, (rod_origin.x, rod_origin.y, rod_origin.z));

    let rod_end = rod_far_end_world(&data);
    let slider_pos = &data.xpos[SLIDER_BODY];
    let c2_err = dist3(rod_end, (slider_pos.x, slider_pos.y, slider_pos.z));

    println!(
        "t={:.1}s  θ={:+.1}°  ω={:.2} ({:.0} RPM)  x={:.4}  pred={:.4}  |err|={:.2e}  c1={:.2e} c2={:.2e}",
        data.time,
        theta.to_degrees(),
        omega,
        rpm,
        x_meas,
        x_pred,
        error,
        c1_err,
        c2_err,
    );
}
