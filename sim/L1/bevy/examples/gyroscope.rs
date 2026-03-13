//! Gyroscope: spinning disk precession with analytical verification.
//!
//! Spinning disk on a rod with ball joint at pivot. Gravity causes precession.
//! The window title shows measured precession rate vs analytical prediction:
//!   ω_p = mgh / (I_spin · ω_spin)
//!
//! Parameters: m=2.0 kg, h=0.3 m, I_spin=0.04 kg·m², ω_spin=100 rad/s
//!   → ω_p ≈ 1.4715 rad/s (T ≈ 4.27 s)
//!
//! Controls: R = reset with initial spin, Space = kill spin (falls!)
//!
//! Run with: `cargo run -p sim-bevy --example gyroscope --release`

#![allow(clippy::needless_pass_by_value)]
#![allow(clippy::expect_used)]
#![allow(clippy::cast_precision_loss)]
#![allow(clippy::too_many_arguments)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCamera;
use sim_bevy::model_data::{ModelDataPlugin, PhysicsData, PhysicsModel};
use sim_bevy::prelude::{Data, SimViewerPlugin, load_model};

// ============================================================================
// MJCF Model Definition
// ============================================================================

/// Gyroscope: disk on rod with ball joint at pivot.
///
/// Single body with ball joint at (0, 0, 2.0). Explicit inertial element
/// gives precise control over mass distribution:
///   - CoM at (0.3, 0, 0) = disk position along rod
///   - Mass = 2.0 kg (dominated by disk)
///   - I_spin = 0.04 kg·m² (about rod axis X, through CoM)
///   - I_trans = 0.02 kg·m² (about Y and Z, through CoM)
///
/// Geoms are present for collision shape definitions but visuals are drawn
/// entirely with gizmos to avoid mesh rotation conversion issues with
/// fast-spinning ball joints.
const GYRO_MJCF: &str = r#"
<mujoco model="gyroscope">
    <compiler angle="radian" inertiafromgeom="false"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <body name="gyro" pos="0 0 2.0">
            <joint name="ball" type="ball"/>
            <inertial pos="0.3 0 0" mass="2.0"
                      diaginertia="0.04 0.02 0.02"/>
            <geom name="rod" type="capsule" fromto="0 0 0 0.3 0 0"
                  size="0.012" rgba="0.6 0.6 0.6 0"/>
            <geom name="disk" type="cylinder" pos="0.3 0 0"
                  euler="0 1.5708 0" size="0.08 0.015"
                  rgba="0.85 0.2 0.2 0"/>
        </body>
    </worldbody>
</mujoco>
"#;

/// Maximum physics steps per frame (safety cap).
const MAX_STEPS_PER_FRAME: usize = 64;

/// Initial spin rate about rod axis (rad/s).
const OMEGA_SPIN: f64 = 100.0;

/// Gravity acceleration.
const G: f64 = 9.81;

/// Gyro body index (0 = worldbody, 1 = gyro).
const GYRO_BODY: usize = 1;

/// Pivot position in physics frame (Z-up).
const PIVOT_Z: f64 = 2.0;

/// Rod length from pivot to disk center (m).
const ROD_LENGTH: f64 = 0.3;

/// Axle extension past the disk (m).
const AXLE_OVERSHOOT: f64 = 0.06;

/// Disk visual radius (m).
const DISK_RADIUS: f64 = 0.08;

// ============================================================================
// Resources
// ============================================================================

/// Analytical parameters computed from model at startup.
#[derive(Resource)]
struct GyroParams {
    i_pivot: [f64; 3],
    omega_p_pred: f64,
    initial_l_mag: f64,
}

/// Precession angle tracker (unwrapped).
#[derive(Resource)]
struct PrecessionTracker {
    prev_angle: f64,
    total_angle: f64,
    start_time: f64,
    started: bool,
}

impl Default for PrecessionTracker {
    fn default() -> Self {
        Self {
            prev_angle: 0.0,
            total_angle: 0.0,
            start_time: 0.0,
            started: false,
        }
    }
}

/// Trail of recent disk positions for gizmo drawing.
#[derive(Resource)]
struct Trail {
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
        .init_resource::<PrecessionTracker>()
        .init_resource::<Trail>()
        .insert_resource(ShowControls(true))
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                restart,
                kill_spin,
                step_realtime,
                track_precession,
                record_trail,
            ),
        )
        .add_systems(
            PostUpdate,
            (window_hud, console_hud, draw_gyroscope, draw_trail),
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
    let model = load_model(GYRO_MJCF).expect("Failed to load MJCF");
    let mut data = model.make_data();

    // Read model parameters for analytical formula
    let mass = model.body_mass[GYRO_BODY];
    let inertia = &model.body_inertia[GYRO_BODY];
    let ipos = &model.body_ipos[GYRO_BODY];
    let h = ipos.x;
    let i_spin = inertia.x;

    // Inertia about pivot (parallel axis theorem)
    let h_sq = ipos.x * ipos.x + ipos.y * ipos.y + ipos.z * ipos.z;
    let i_pivot = [
        inertia.x + mass * (h_sq - ipos.x * ipos.x),
        inertia.y + mass * (h_sq - ipos.y * ipos.y),
        inertia.z + mass * (h_sq - ipos.z * ipos.z),
    ];

    // ω_p = mgh / (I_spin · ω_spin)
    let omega_p_pred = mass * G * h / (i_spin * OMEGA_SPIN);

    // Initial conditions: identity quaternion (rod along +X), spin + precession
    data.qpos[0] = 1.0; // w
    data.qpos[1] = 0.0; // x
    data.qpos[2] = 0.0; // y
    data.qpos[3] = 0.0; // z

    // qvel in body frame: spin about X, precession about Z
    // At t=0 body frame = world frame, so precession about world Z = body Z
    data.qvel[0] = OMEGA_SPIN;
    data.qvel[1] = 0.0;
    data.qvel[2] = omega_p_pred;

    let _ = data.forward(&model);

    // Compute initial |L|
    let initial_l_mag = compute_l_magnitude(&data, &i_pivot);

    // Pivot marker (small gray sphere) — physics (0,0,2) → Bevy (0,2,0)
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.03))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.5, 0.5, 0.5),
            metallic: 0.6,
            ..default()
        })),
        Transform::from_xyz(0.0, PIVOT_Z as f32, 0.0),
    ));

    // Camera — orbit around pivot
    let orbit = OrbitCamera::new()
        .with_target(Vec3::new(0.0, PIVOT_Z as f32, 0.0))
        .with_distance(1.5)
        .with_angles(0.8, 0.4);
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
    println!("  GYROSCOPE — Precession Verification");
    println!("=============================================");
    println!("m={mass:.2} kg, h={h:.3} m, I_spin={i_spin:.4} kg·m²");
    println!("ω_spin = {OMEGA_SPIN:.1} rad/s");
    println!(
        "ω_p_pred = {omega_p_pred:.4} rad/s (T = {:.3} s)",
        2.0 * std::f64::consts::PI / omega_p_pred
    );
    println!("|L|₀ = {initial_l_mag:.4} kg·m²/s");
    println!(
        "I_pivot = [{:.4}, {:.4}, {:.4}]",
        i_pivot[0], i_pivot[1], i_pivot[2]
    );
    println!("---------------------------------------------");
    println!("Press 'R' to reset");
    println!("=============================================");

    let params = GyroParams {
        i_pivot,
        omega_p_pred,
        initial_l_mag,
    };

    commands.insert_resource(PhysicsModel::new(model));
    commands.insert_resource(PhysicsData::new(data));
    commands.insert_resource(params);
    commands.insert_resource(HudTimer {
        last_print_time: -1.0,
    });
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Compute angular momentum magnitude about pivot.
///
/// L_body = I_pivot_diag * ω_body (both in body frame).
/// |L| = |L_body| (rotation preserves magnitude).
fn compute_l_magnitude(data: &Data, i_pivot: &[f64; 3]) -> f64 {
    let lx = i_pivot[0] * data.qvel[0];
    let ly = i_pivot[1] * data.qvel[1];
    let lz = i_pivot[2] * data.qvel[2];
    (lx * lx + ly * ly + lz * lz).sqrt()
}

/// Get rod direction projected onto horizontal plane (physics XY).
/// Returns (rod_x, rod_y) from xmat first column.
fn rod_direction_xy(data: &PhysicsData) -> (f64, f64) {
    let mat = &data.xmat[GYRO_BODY];
    (mat[(0, 0)], mat[(1, 0)])
}

/// Convert physics (x, y, z) Z-up to Bevy (x, z, y) Y-up.
fn to_bevy(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(x as f32, z as f32, y as f32)
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
    params: Res<GyroParams>,
    mut tracker: ResMut<PrecessionTracker>,
    mut trail: ResMut<Trail>,
    mut timer: ResMut<HudTimer>,
    mut show: ResMut<ShowControls>,
) {
    if !keyboard.just_pressed(KeyCode::KeyR) {
        return;
    }

    data.qpos.fill(0.0);
    data.qvel.fill(0.0);
    data.time = 0.0;

    // Identity quaternion
    data.qpos[0] = 1.0;

    // Spin + precession
    data.qvel[0] = OMEGA_SPIN;
    data.qvel[2] = params.omega_p_pred;

    let _ = data.forward(&model);

    *tracker = PrecessionTracker::default();
    trail.points.clear();
    timer.last_print_time = -1.0;
    show.0 = false;

    println!("---------------------------------------------");
    println!("  RESET — ω_spin = {OMEGA_SPIN:.1} rad/s");
    println!("---------------------------------------------");
}

/// Kill the spin on Space — disk stops spinning and gravity takes over.
/// Without angular momentum, the gyroscopic effect vanishes and it falls.
fn kill_spin(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut data: ResMut<PhysicsData>,
    mut show: ResMut<ShowControls>,
) {
    if !keyboard.just_pressed(KeyCode::Space) {
        return;
    }
    // Zero all angular velocity — no spin, no precession, just gravity
    data.qvel.fill(0.0);
    show.0 = false;
    println!("  SPIN KILLED — gravity takes over");
}

/// Track precession angle (projection of rod onto horizontal plane).
fn track_precession(data: Res<PhysicsData>, mut tracker: ResMut<PrecessionTracker>) {
    let (rod_x, rod_y) = rod_direction_xy(&data);
    let angle = rod_y.atan2(rod_x);

    if !tracker.started {
        tracker.prev_angle = angle;
        tracker.start_time = data.time;
        tracker.total_angle = 0.0;
        tracker.started = true;
        return;
    }

    // Unwrap angle (handle ±π crossings)
    let mut delta = angle - tracker.prev_angle;
    if delta > std::f64::consts::PI {
        delta -= 2.0 * std::f64::consts::PI;
    } else if delta < -std::f64::consts::PI {
        delta += 2.0 * std::f64::consts::PI;
    }
    tracker.total_angle += delta;
    tracker.prev_angle = angle;
}

/// Record disk CoM position trail for gizmo drawing.
fn record_trail(data: Res<PhysicsData>, mut trail: ResMut<Trail>) {
    let com = &data.xipos[GYRO_BODY];
    let bevy_pos = to_bevy(com.x, com.y, com.z);

    let should_record = trail
        .points
        .last()
        .is_none_or(|last| last.distance(bevy_pos) > 0.002);
    if should_record {
        trail.points.push(bevy_pos);
        if trail.points.len() > 500 {
            trail.points.drain(0..100);
        }
    }
}

/// Draw the gyroscope: rod (axle), disk, and vertical axis — all from physics data.
///
/// Uses body xpos (pivot) and xmat (orientation) directly, bypassing geom mesh
/// rendering to avoid coordinate conversion artifacts with fast-spinning ball joints.
fn draw_gyroscope(data: Res<PhysicsData>, mut gizmos: Gizmos) {
    // Body frame axes from xmat columns (physics Z-up world frame)
    let mat = &data.xmat[GYRO_BODY];
    let pivot = &data.xpos[GYRO_BODY]; // = (0, 0, 2) always (ball joint pivot)

    // Rod direction = body X axis = first column of xmat
    let rod_dir = (mat[(0, 0)], mat[(1, 0)], mat[(2, 0)]);
    // Body Y axis = second column (for disk orientation)
    let body_y = (mat[(0, 1)], mat[(1, 1)], mat[(2, 1)]);
    // Body Z axis = third column
    let body_z = (mat[(0, 2)], mat[(1, 2)], mat[(2, 2)]);

    // Key points in physics frame
    let pivot_b = to_bevy(pivot.x, pivot.y, pivot.z);
    let disk_center = to_bevy(
        pivot.x + rod_dir.0 * ROD_LENGTH,
        pivot.y + rod_dir.1 * ROD_LENGTH,
        pivot.z + rod_dir.2 * ROD_LENGTH,
    );
    let axle_tip = to_bevy(
        pivot.x + rod_dir.0 * (ROD_LENGTH + AXLE_OVERSHOOT),
        pivot.y + rod_dir.1 * (ROD_LENGTH + AXLE_OVERSHOOT),
        pivot.z + rod_dir.2 * (ROD_LENGTH + AXLE_OVERSHOOT),
    );

    // Draw rod (axle) from pivot through disk center to tip
    let rod_color = Color::srgb(0.7, 0.7, 0.7);
    gizmos.line(pivot_b, axle_tip, rod_color);
    // Thicken the rod with parallel lines
    let offset = 0.006_f64;
    for &(dy, dz) in &[(1.0_f64, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
        let off = to_bevy(
            body_y.0 * dy * offset + body_z.0 * dz * offset,
            body_y.1 * dy * offset + body_z.1 * dz * offset,
            body_y.2 * dy * offset + body_z.2 * dz * offset,
        );
        gizmos.line(pivot_b + off, axle_tip + off, rod_color);
    }

    // Draw disk as filled circle (multiple concentric rings)
    let disk_normal_bevy = to_bevy(rod_dir.0, rod_dir.1, rod_dir.2).normalize();
    let disk_rot = Quat::from_rotation_arc(Vec3::Z, disk_normal_bevy);
    let disk_iso = Isometry3d::new(disk_center, disk_rot);

    let disk_color = Color::srgb(0.85, 0.2, 0.2);
    let rings = 8;
    for i in 1..=rings {
        let r = DISK_RADIUS as f32 * (i as f32 / rings as f32);
        gizmos.circle(disk_iso, r, disk_color);
    }
    // Draw disk rim thicker
    gizmos.circle(disk_iso, DISK_RADIUS as f32, Color::srgb(0.6, 0.15, 0.15));

    // Spokes along body Y and Z axes — rotate with the spin so you can see it
    let r = DISK_RADIUS;
    let spoke_color = Color::srgb(1.0, 1.0, 1.0);
    for &(ax, ay, az) in &[body_y, body_z] {
        let tip_pos = to_bevy(
            pivot.x + rod_dir.0 * ROD_LENGTH + ax * r,
            pivot.y + rod_dir.1 * ROD_LENGTH + ay * r,
            pivot.z + rod_dir.2 * ROD_LENGTH + az * r,
        );
        let tip_neg = to_bevy(
            pivot.x + rod_dir.0 * ROD_LENGTH - ax * r,
            pivot.y + rod_dir.1 * ROD_LENGTH - ay * r,
            pivot.z + rod_dir.2 * ROD_LENGTH - az * r,
        );
        gizmos.line(tip_neg, tip_pos, spoke_color);
    }

    // Vertical axis through pivot
    gizmos.line(
        pivot_b - Vec3::Y * 0.4,
        pivot_b + Vec3::Y * 0.4,
        Color::srgba(0.0, 0.8, 0.0, 0.4),
    );
}

/// Draw disk position trail as gizmo lines.
fn draw_trail(trail: Res<Trail>, mut gizmos: Gizmos) {
    if trail.points.len() < 2 {
        return;
    }

    let n = trail.points.len();
    for (i, window) in trail.points.windows(2).enumerate() {
        let alpha = (i as f32 + 1.0) / n as f32;
        gizmos.line(
            window[0],
            window[1],
            Color::srgba(1.0, 0.5, 0.0, alpha * 0.8),
        );
    }
}

/// Window title HUD: shows controls initially, then live data.
fn window_hud(
    data: Res<PhysicsData>,
    params: Res<GyroParams>,
    tracker: Res<PrecessionTracker>,
    show: Res<ShowControls>,
    mut windows: Query<&mut Window>,
) {
    let Ok(mut window) = windows.single_mut() else {
        return;
    };

    if show.0 && data.time < 2.0 {
        window.title = "Gyroscope | R = reset | Space = kill spin | Orbit camera".to_string();
        return;
    }

    let spin_rate = data.qvel[0];
    let l_mag = compute_l_magnitude(&data, &params.i_pivot);
    let l_err = (l_mag - params.initial_l_mag) / params.initial_l_mag * 100.0;

    let elapsed = data.time - tracker.start_time;
    let (prec_str, err_str) = if elapsed > 0.5 {
        let omega_p_meas = tracker.total_angle / elapsed;
        let err = (omega_p_meas - params.omega_p_pred) / params.omega_p_pred * 100.0;
        (format!("{omega_p_meas:.3}"), format!("{err:+.2}%"))
    } else {
        ("---".to_string(), "---".to_string())
    };

    window.title = format!(
        "Gyroscope | t={:.1}s | spin={spin_rate:.1} | prec={prec_str} (pred {:.3}, err {err_str}) | |L| err={l_err:+.3}%",
        data.time, params.omega_p_pred,
    );
}

/// Console HUD: periodic detailed output (every 2s sim time).
fn console_hud(
    data: Res<PhysicsData>,
    params: Res<GyroParams>,
    tracker: Res<PrecessionTracker>,
    mut timer: ResMut<HudTimer>,
) {
    if data.time - timer.last_print_time < 2.0 {
        return;
    }
    timer.last_print_time = data.time;

    let spin_rate = data.qvel[0];
    let l_mag = compute_l_magnitude(&data, &params.i_pivot);
    let l_err = (l_mag - params.initial_l_mag) / params.initial_l_mag * 100.0;

    let elapsed = data.time - tracker.start_time;
    let prec_info = if elapsed > 0.5 {
        let omega_p_meas = tracker.total_angle / elapsed;
        let err = (omega_p_meas - params.omega_p_pred) / params.omega_p_pred * 100.0;
        format!(
            "ω_p_meas={omega_p_meas:.4}  ω_p_pred={:.4}  err={err:+.3}%",
            params.omega_p_pred
        )
    } else {
        "ω_p_meas=---  (measuring)".to_string()
    };

    // Rod tilt from horizontal (should stay near 0° for clean precession)
    let mat = &data.xmat[GYRO_BODY];
    let rod_z = mat[(2, 0)]; // Z component of rod direction
    let tilt_deg = rod_z.asin().to_degrees();

    println!(
        "t={:.1}s  spin={spin_rate:.1}  {prec_info}  |L|={l_mag:.4} (err={l_err:+.3}%)  tilt={tilt_deg:+.02}°",
        data.time,
    );
}
