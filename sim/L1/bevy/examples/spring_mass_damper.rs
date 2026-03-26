//! Spring-mass-damper: analytically verifiable damped harmonic oscillator.
//!
//! Mass on a slide joint with spring, damper, and ground contact. The HUD
//! shows measured oscillation frequency vs the analytical prediction
//! ω_d = ω_n√(1−ζ²) and the error %.
//!
//! Analytical values (m=1.0, k=50, c=2.0):
//!   ω_n = √(k/m) ≈ 7.07 rad/s
//!   ζ = c/(2√(km)) ≈ 0.141
//!   ω_d = ω_n√(1−ζ²) ≈ 7.00 rad/s
//!   f_d ≈ 1.11 Hz, T_d ≈ 0.90 s
//!
//! Run with: `cargo run -p sim-bevy --example spring_mass_damper --release`

#![allow(clippy::needless_pass_by_value)]
#![allow(clippy::expect_used)]
#![allow(clippy::cast_precision_loss)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCamera;
use sim_bevy::model_data::{ModelDataPlugin, PhysicsData, PhysicsModel};
use sim_bevy::prelude::{SimViewerPlugin, load_model, spawn_model_geoms};
use sim_core::ENABLE_ENERGY;

// ============================================================================
// MJCF Model Definition
// ============================================================================

/// Mass-spring-damper on a vertical slide joint with ground contact.
///
/// A 1 kg box slides along Z on a spring (k=50 N/m) with viscous damping
/// (c=2.0 Ns/m). A ground plane provides contact so the mass can bounce
/// if dropped from high enough.
///
/// Body pos="0 0 1.0" sets the spring's rest position at Z=1.0 (qpos=0).
/// Under gravity the equilibrium shifts to qpos_eq = −mg/k ≈ −0.196 m
/// (body center at Z ≈ 0.804). Starting at qpos=0.5 (Z=1.5) gives a
/// large initial displacement that briefly contacts the ground on the
/// first downswing before settling into clean free oscillation.
///
/// RK4 integrator with dt=0.001 for high-accuracy frequency measurement.
const SPRING_MASS_MJCF: &str = r#"
<mujoco model="spring_mass_damper">
    <compiler angle="radian" inertiafromgeom="true"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

    <worldbody>
        <geom name="ground" type="plane" size="5 5 0.1" rgba="0.8 0.8 0.8 1"/>

        <body name="mass" pos="0 0 1.0">
            <joint name="slide" type="slide" axis="0 0 1"
                   stiffness="50" damping="2.0"/>
            <geom name="mass_geom" type="box" size="0.15 0.15 0.15"
                  mass="1.0" rgba="0.2 0.6 1.0 1"/>
        </body>
    </worldbody>
</mujoco>
"#;

/// Maximum physics steps per frame (safety cap).
const MAX_STEPS_PER_FRAME: usize = 64;

/// Initial slide joint displacement (Z=1.0 + 0.5 = 1.5 m above ground).
const INITIAL_QPOS: f64 = 0.5;

// Analytical constants (m=1.0, k=50, c=2.0)
const MASS: f64 = 1.0;
const STIFFNESS: f64 = 50.0;
const DAMPING: f64 = 2.0;

/// Natural frequency ω_n = √(k/m).
const OMEGA_N: f64 = 7.071_067_811_865_476; // √50

/// Damping ratio ζ = c / (2√(km)).
const ZETA: f64 = 0.141_421_356_237_310; // 2/(2√50)

/// Damped frequency ω_d = ω_n√(1−ζ²).
const OMEGA_D: f64 = 6.999_999_999_999_999; // ω_n * √(1 − ζ²) ≈ 7.0

/// Predicted frequency in Hz.
const FREQ_PREDICTED: f64 = OMEGA_D / (2.0 * std::f64::consts::PI);

// ============================================================================
// Resources
// ============================================================================

/// Initial total energy for drift tracking.
#[derive(Resource)]
struct InitialEnergy(f64);

/// Equilibrium position (qpos where net force = 0).
#[derive(Resource)]
struct Equilibrium(f64);

/// Oscillation measurement state.
#[derive(Resource)]
struct OscillationTracker {
    /// Previous velocity sign (true = positive).
    prev_vel_positive: bool,
    /// Times of detected peaks (velocity crosses + → −).
    peak_times: Vec<f64>,
    /// Amplitudes at detected peaks (displacement from equilibrium).
    peak_amplitudes: Vec<f64>,
    /// Average measured period over all consecutive peaks.
    measured_period: Option<f64>,
    /// Whether tracking has started (skip initial transient/contact).
    tracking_active: bool,
}

impl Default for OscillationTracker {
    fn default() -> Self {
        Self {
            prev_vel_positive: true,
            peak_times: Vec::new(),
            peak_amplitudes: Vec::new(),
            measured_period: None,
            tracking_active: false,
        }
    }
}

/// Trail of recent positions for gizmo drawing.
#[derive(Resource)]
struct Trail {
    /// (sim_time, qpos) pairs.
    points: Vec<(f64, f64)>,
}

impl Default for Trail {
    fn default() -> Self {
        Self {
            points: Vec::with_capacity(2000),
        }
    }
}

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
        .add_plugins(
            SimViewerPlugin::new()
                .without_camera()
                .without_lighting(),
        )
        .add_plugins(ModelDataPlugin::new()) // no auto_step — wall-clock stepping
        .init_resource::<OscillationTracker>()
        .init_resource::<Trail>()
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (restart, step_realtime, track_oscillation, record_trail),
        )
        .add_systems(PostUpdate, (window_hud, console_hud, draw_trail))
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
    let mut model = load_model(SPRING_MASS_MJCF).expect("Failed to load MJCF");
    model.enableflags |= ENABLE_ENERGY;

    let mut data = model.make_data();

    // Start at Z=1.5 (qpos=0.5 above spring rest at Z=1.0)
    data.qpos[0] = INITIAL_QPOS;
    let _ = data.forward(&model);

    // Initial energy
    let e0 = data.energy_kinetic + data.energy_potential;

    // Equilibrium: qpos where spring force balances gravity
    // F_spring = -k * qpos, F_gravity = -mg → equilibrium at qpos = -mg/k
    let eq = -MASS * 9.81 / STIFFNESS;

    // Auto-spawn visual entities for all geoms (ground plane + mass box)
    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[],
    );

    // Camera — side view from +X axis, centered on oscillation range
    let orbit = OrbitCamera::new()
        .with_target(Vec3::new(0.0, 1.0, 0.0))
        .with_distance(5.0)
        .with_angles(0.0, 0.15);
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
    println!("  SPRING-MASS-DAMPER — Analytical Verification");
    println!("=============================================");
    println!("m={MASS} kg, k={STIFFNESS} N/m, c={DAMPING} Ns/m");
    println!("omega_n = {OMEGA_N:.4} rad/s");
    println!("zeta    = {ZETA:.4}");
    println!("omega_d = {OMEGA_D:.4} rad/s");
    println!(
        "f_pred  = {FREQ_PREDICTED:.4} Hz  (T = {:.4} s)",
        1.0 / FREQ_PREDICTED
    );
    println!("---------------------------------------------");
    println!("Initial: Z=1.5 (qpos=0.5), equilibrium at qpos={eq:.4}");
    println!("Press 'R' to reset");
    println!("=============================================");

    commands.insert_resource(PhysicsModel::new(model));
    commands.insert_resource(PhysicsData::new(data));
    commands.insert_resource(InitialEnergy(e0));
    commands.insert_resource(Equilibrium(eq));
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

/// Reset physics on R key.
fn restart(
    keyboard: Res<ButtonInput<KeyCode>>,
    model: Res<PhysicsModel>,
    mut data: ResMut<PhysicsData>,
    mut initial: ResMut<InitialEnergy>,
    mut tracker: ResMut<OscillationTracker>,
    mut trail: ResMut<Trail>,
    mut timer: ResMut<HudTimer>,
) {
    if !keyboard.just_pressed(KeyCode::KeyR) {
        return;
    }

    // Reset physics
    data.qpos.fill(0.0);
    data.qvel.fill(0.0);
    data.time = 0.0;
    data.qpos[0] = INITIAL_QPOS;
    let _ = data.forward(&model);

    // Reset energy baseline
    initial.0 = data.energy_kinetic + data.energy_potential;

    // Reset oscillation tracker
    *tracker = OscillationTracker::default();

    // Reset trail
    trail.points.clear();

    // Reset HUD timer
    timer.last_print_time = -1.0;

    println!("---------------------------------------------");
    println!("  RESET — dropped from Z=1.5");
    println!("---------------------------------------------");
}

/// Track oscillation peaks to measure frequency and decay envelope.
///
/// Detects peaks by watching for velocity sign changes (positive → negative).
/// Tracking only starts after the mass is above the ground contact zone
/// (clean free oscillation) to avoid measuring the initial bounce transient.
fn track_oscillation(
    data: Res<PhysicsData>,
    eq: Res<Equilibrium>,
    mut tracker: ResMut<OscillationTracker>,
) {
    let qpos = data.qpos[0];
    let qvel = data.qvel[0];
    let vel_positive = qvel >= 0.0;

    // Don't start tracking until after the initial transient: wait for
    // the mass to be oscillating above ground (body center > 0.3m in
    // physics Z, i.e., qpos > -0.7 with body pos at Z=1.0).
    // Also require at least 0.5s of sim time to skip the first bounce.
    if !tracker.tracking_active {
        if data.time > 0.5 && qpos > -0.5 {
            tracker.tracking_active = true;
            tracker.prev_vel_positive = vel_positive;
        }
        return;
    }

    // Detect peak: velocity crosses from positive to negative
    if tracker.prev_vel_positive && !vel_positive {
        let displacement = qpos - eq.0;
        tracker.peak_times.push(data.time);
        tracker.peak_amplitudes.push(displacement.abs());

        // Average period over all consecutive peak pairs for stability
        let n = tracker.peak_times.len();
        if n >= 2 {
            let total_span = tracker.peak_times[n - 1] - tracker.peak_times[0];
            let avg_period = total_span / (n - 1) as f64;
            tracker.measured_period = Some(avg_period);
        }
    }

    tracker.prev_vel_positive = vel_positive;
}

/// Record trail points for gizmo drawing.
fn record_trail(data: Res<PhysicsData>, mut trail: ResMut<Trail>) {
    // Sample every ~10ms of sim time
    let should_record = trail
        .points
        .last()
        .is_none_or(|&(t, _)| data.time - t >= 0.01);
    if should_record {
        trail.points.push((data.time, data.qpos[0]));
        // Keep last 20s of trail
        if trail.points.len() > 2000 {
            trail.points.drain(0..500);
        }
    }
}

/// Draw trail and decay envelope as gizmos.
fn draw_trail(
    trail: Res<Trail>,
    eq: Res<Equilibrium>,
    tracker: Res<OscillationTracker>,
    mut gizmos: Gizmos,
) {
    // Draw position trail (blue line in X-Y plane where X=time, Y=height)
    // Map sim time to X position and qpos to Bevy Y.
    // Use relative time (last 10s) centered on screen.
    if trail.points.len() < 2 {
        return;
    }

    let latest_time = trail.points.last().map_or(0.0, |&(t, _)| t);
    let time_window = 10.0; // show last 10s

    // Trail drawn behind the mass (at X=-1.0 in Bevy space)
    let trail_x: f32 = -1.0;
    let time_scale: f32 = 0.2; // 10s maps to 2.0 Bevy units

    for window in trail.points.windows(2) {
        let (t0, q0) = window[0];
        let (t1, q1) = window[1];

        let age0 = latest_time - t0;
        let age1 = latest_time - t1;

        if age0 > time_window || age1 > time_window {
            continue;
        }

        // Map to Bevy coordinates: Z = relative time offset, Y = body height
        // Body height in Bevy Y-up = body_pos_z = 1.0 + qpos (physics Z → Bevy Y)
        let z0 = -(age0 as f32) * time_scale;
        let z1 = -(age1 as f32) * time_scale;
        let y0 = 1.0 + q0 as f32; // body pos Z=1.0 + qpos → Bevy Y
        let y1 = 1.0 + q1 as f32;

        gizmos.line(
            Vec3::new(trail_x, y0, z0),
            Vec3::new(trail_x, y1, z1),
            Color::srgba(0.2, 0.6, 1.0, 0.5),
        );
    }

    // Draw equilibrium line
    let eq_y = 1.0 + eq.0 as f32;
    gizmos.line(
        Vec3::new(trail_x, eq_y, -(time_window as f32) * time_scale),
        Vec3::new(trail_x, eq_y, 0.0),
        Color::srgba(0.0, 0.8, 0.0, 0.4),
    );

    // Draw decay envelope at peak points
    for (i, (&t, &amp)) in tracker
        .peak_times
        .iter()
        .zip(tracker.peak_amplitudes.iter())
        .enumerate()
    {
        let age = latest_time - t;
        if age > time_window {
            continue;
        }
        let z = -(age as f32) * time_scale;

        // Upper envelope point
        let y_upper = eq_y + amp as f32;
        gizmos.sphere(
            Isometry3d::from_translation(Vec3::new(trail_x, y_upper, z)),
            0.015,
            Color::srgb(1.0, 0.5, 0.0),
        );

        // Connect envelope points
        if i > 0 {
            let prev_age = latest_time - tracker.peak_times[i - 1];
            if prev_age <= time_window {
                let prev_z = -(prev_age as f32) * time_scale;
                let prev_y = eq_y + tracker.peak_amplitudes[i - 1] as f32;
                gizmos.line(
                    Vec3::new(trail_x, prev_y, prev_z),
                    Vec3::new(trail_x, y_upper, z),
                    Color::srgba(1.0, 0.5, 0.0, 0.6),
                );
            }
        }
    }
}

/// Window title HUD: compact live readout.
fn window_hud(
    data: Res<PhysicsData>,
    tracker: Res<OscillationTracker>,
    mut windows: Query<&mut Window>,
) {
    let qpos = data.qpos[0];
    let body_z = 1.0 + qpos; // body pos Z=1.0 + qpos
    let qvel = data.qvel[0];

    let (freq_str, err_str) = if let Some(period) = tracker.measured_period {
        let freq = 1.0 / period;
        let err = (freq - FREQ_PREDICTED) / FREQ_PREDICTED * 100.0;
        (format!("{freq:.3} Hz"), format!("{err:+.2}%"))
    } else {
        ("---".to_string(), "---".to_string())
    };

    if let Ok(mut window) = windows.single_mut() {
        window.title = format!(
            "Spring-Mass-Damper | t={:.1}s | Z={body_z:.3}m | v={qvel:+.2} m/s | f={freq_str} (pred {FREQ_PREDICTED:.3} Hz, err {err_str})",
            data.time,
        );
    }
}

/// Console HUD: periodic detailed output (every 2s sim time).
fn console_hud(
    data: Res<PhysicsData>,
    initial: Res<InitialEnergy>,
    eq: Res<Equilibrium>,
    tracker: Res<OscillationTracker>,
    mut timer: ResMut<HudTimer>,
) {
    if data.time - timer.last_print_time < 2.0 {
        return;
    }
    timer.last_print_time = data.time;

    let qpos = data.qpos[0];
    let qvel = data.qvel[0];
    let body_z = 1.0 + qpos;
    let energy = data.energy_kinetic + data.energy_potential;
    let energy_drift = if initial.0.abs() > 1e-10 {
        (energy - initial.0) / initial.0.abs() * 100.0
    } else {
        0.0
    };

    let freq_info = if let Some(period) = tracker.measured_period {
        let freq = 1.0 / period;
        let err = (freq - FREQ_PREDICTED) / FREQ_PREDICTED * 100.0;
        format!("f_meas={freq:.4} Hz  f_pred={FREQ_PREDICTED:.4} Hz  err={err:+.3}%")
    } else {
        "f_meas=---  (waiting for peaks)".to_string()
    };

    // Envelope comparison: use first detected peak as reference (skips ground bounce)
    // A(t) = A0 * e^(-ζω_n * (t - t0))
    let envelope_info = if tracker.peak_times.len() >= 2 {
        let t0 = tracker.peak_times[0];
        let a0 = tracker.peak_amplitudes[0];
        let latest_amp = *tracker.peak_amplitudes.last().expect("checked len >= 2");
        let latest_t = *tracker.peak_times.last().expect("checked len >= 2");
        let dt = latest_t - t0;
        let analytical_amp = a0 * (-ZETA * OMEGA_N * dt).exp();
        let env_err = if analytical_amp.abs() > 1e-10 {
            (latest_amp - analytical_amp) / analytical_amp * 100.0
        } else {
            0.0
        };
        format!("  amp={latest_amp:.4}  env_pred={analytical_amp:.4}  env_err={env_err:+.2}%")
    } else {
        String::new()
    };

    let disp = qpos - eq.0;
    println!(
        "t={:.1}s  Z={body_z:.3}m  disp={disp:+.4}  v={qvel:+.3}  E={energy:.4}J  dE={energy_drift:+.2}%  {freq_info}{envelope_info}",
        data.time,
    );
}
