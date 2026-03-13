//! Coupled pendulums: beat frequency verification via energy exchange.
//!
//! Two pendulums with different natural frequencies coupled by a torsional
//! spring. Energy transfers between them in a measurable beating pattern.
//!
//! The coupling torque τ = -κ(θ₁ − θ₂) is applied manually via qfrc_applied
//! each physics step. This gives the cleanest analytical prediction since
//! the coupling stiffness κ is explicit.
//!
//! Normal modes are computed exactly from the coupled equations:
//!   I₁θ̈₁ + m₁gL₁θ₁ + κ(θ₁ − θ₂) = 0
//!   I₂θ̈₂ + m₂gL₂θ₂ − κ(θ₁ − θ₂) = 0
//!
//! Beat frequency: f_beat = |f₊ − f₋| / 2 where f₊, f₋ are normal modes.
//!
//! Controls: R = reset
//!
//! Run with: `cargo run -p sim-bevy --example coupled_pendulums --release`

#![allow(clippy::needless_pass_by_value)]
#![allow(clippy::expect_used)]
#![allow(clippy::cast_precision_loss)]
#![allow(clippy::too_many_arguments)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCamera;
use sim_bevy::model_data::{ModelDataPlugin, PhysicsData, PhysicsModel};
use sim_bevy::prelude::{SimViewerPlugin, load_model};

// ============================================================================
// MJCF Model Definition
// ============================================================================

/// Two independent pendulums with hinge joints, hanging from worldbody.
///
/// Pendulum 1: L₁=0.6m, pivot at (-0.75, 0, 2.0)
/// Pendulum 2: L₂=0.8m, pivot at (+0.75, 0, 2.0)
///
/// Both have 1.0 kg point-mass bobs (tiny diaginertia for numerical stability).
/// At qpos=0 both hang straight down (-Z in physics frame).
/// Hinge axis Y: positive θ swings bob toward +X.
///
/// Very light damping (0.02) so beats persist for many cycles.
/// Contact disabled — pure articulated dynamics + manual coupling.
/// Geom alpha=0 because visuals are drawn entirely with gizmos.
const COUPLED_MJCF: &str = r#"
<mujoco model="coupled_pendulums">
    <compiler angle="radian" inertiafromgeom="false"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <body name="pend1" pos="-0.75 0 2.0">
            <joint name="j1" type="hinge" axis="0 1 0" damping="0.02"/>
            <inertial pos="0 0 -0.6" mass="1.0"
                      diaginertia="0.001 0.001 0.001"/>
            <geom name="rod1" type="capsule" fromto="0 0 0 0 0 -0.6"
                  size="0.015" rgba="0.3 0.5 0.85 0"/>
            <geom name="bob1" type="sphere" pos="0 0 -0.6"
                  size="0.05" rgba="0.3 0.5 0.85 0"/>
        </body>

        <body name="pend2" pos="0.75 0 2.0">
            <joint name="j2" type="hinge" axis="0 1 0" damping="0.02"/>
            <inertial pos="0 0 -0.8" mass="1.0"
                      diaginertia="0.001 0.001 0.001"/>
            <geom name="rod2" type="capsule" fromto="0 0 0 0 0 -0.8"
                  size="0.015" rgba="0.85 0.25 0.2 0"/>
            <geom name="bob2" type="sphere" pos="0 0 -0.8"
                  size="0.05" rgba="0.85 0.25 0.2 0"/>
        </body>
    </worldbody>
</mujoco>
"#;

/// Maximum physics steps per frame (safety cap).
const MAX_STEPS_PER_FRAME: usize = 64;

/// Torsional coupling stiffness (N·m/rad).
const KAPPA: f64 = 2.0;

/// Initial displacement of pendulum 1 (rad).
const INITIAL_THETA1: f64 = 0.3;

/// Gravity.
const G: f64 = 9.81;

/// Body indices (worldbody=0, pend1=1, pend2=2).
const PEND1_BODY: usize = 1;
const PEND2_BODY: usize = 2;

// ============================================================================
// Resources
// ============================================================================

/// Analytical parameters computed from model at startup.
#[derive(Resource)]
#[allow(dead_code)]
struct CoupledParams {
    /// Effective moment of inertia of pendulum 1 about hinge.
    i1: f64,
    /// Effective moment of inertia of pendulum 2 about hinge.
    i2: f64,
    /// Distance from pivot to CoM for pendulum 1.
    h1: f64,
    /// Distance from pivot to CoM for pendulum 2.
    h2: f64,
    /// Mass of pendulum 1.
    m1: f64,
    /// Mass of pendulum 2.
    m2: f64,
    /// Normal mode frequency (higher), Hz.
    f_plus: f64,
    /// Normal mode frequency (lower), Hz.
    f_minus: f64,
    /// Predicted beat frequency = |f₊ − f₋| / 2, Hz.
    f_beat_pred: f64,
}

/// Beat frequency measurement via E₁ peak tracking.
#[derive(Resource)]
struct BeatTracker {
    /// Previous E₁ value for peak detection.
    prev_e1: f64,
    /// Whether E₁ was increasing last sample.
    was_increasing: bool,
    /// Times of detected E₁ peaks.
    peak_times: Vec<f64>,
    /// Measured beat frequency (Hz).
    measured_f_beat: Option<f64>,
    /// Whether tracking has started (skip initial transient).
    started: bool,
}

impl Default for BeatTracker {
    fn default() -> Self {
        Self {
            prev_e1: 0.0,
            was_increasing: true,
            peak_times: Vec::new(),
            measured_f_beat: None,
            started: false,
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
        .init_resource::<BeatTracker>()
        .insert_resource(ShowControls(true))
        .add_systems(Startup, setup)
        .add_systems(Update, (restart, step_with_coupling, track_beats))
        .add_systems(
            PostUpdate,
            (
                window_hud,
                console_hud,
                draw_pendulums,
                draw_coupling_spring,
            ),
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
    let model = load_model(COUPLED_MJCF).expect("Failed to load MJCF");
    let mut data = model.make_data();

    // Read model parameters for analytical normal mode computation
    let m1 = model.body_mass[PEND1_BODY];
    let m2 = model.body_mass[PEND2_BODY];
    let ipos1 = &model.body_ipos[PEND1_BODY];
    let ipos2 = &model.body_ipos[PEND2_BODY];
    let inertia1 = &model.body_inertia[PEND1_BODY];
    let inertia2 = &model.body_inertia[PEND2_BODY];

    // Distance from pivot to CoM (magnitude of ipos, but we know it's along -Z)
    let h1 = (ipos1.x * ipos1.x + ipos1.y * ipos1.y + ipos1.z * ipos1.z).sqrt();
    let h2 = (ipos2.x * ipos2.x + ipos2.y * ipos2.y + ipos2.z * ipos2.z).sqrt();

    // Effective moment of inertia about hinge (Y axis through pivot)
    // I_pivot = I_com_y + m * (ipos_x² + ipos_z²)
    let i1 = inertia1.y + m1 * (ipos1.x * ipos1.x + ipos1.z * ipos1.z);
    let i2 = inertia2.y + m2 * (ipos2.x * ipos2.x + ipos2.z * ipos2.z);

    // Normal mode analysis (small angle, undamped):
    //   α₁ = (m₁gL₁ + κ) / I₁
    //   α₂ = (m₂gL₂ + κ) / I₂
    //   β₁ = κ / I₁
    //   β₂ = κ / I₂
    //   ω² = [(α₁+α₂) ± √((α₁−α₂)² + 4β₁β₂)] / 2
    let alpha1 = (m1 * G * h1 + KAPPA) / i1;
    let alpha2 = (m2 * G * h2 + KAPPA) / i2;
    let beta1 = KAPPA / i1;
    let beta2 = KAPPA / i2;

    let sum = alpha1 + alpha2;
    let diff = alpha1 - alpha2;
    let disc = (diff * diff + 4.0 * beta1 * beta2).sqrt();

    let omega_plus = ((sum + disc) / 2.0).sqrt();
    let omega_minus = ((sum - disc) / 2.0).sqrt();
    let f_plus = omega_plus / (2.0 * std::f64::consts::PI);
    let f_minus = omega_minus / (2.0 * std::f64::consts::PI);
    let f_beat_pred = (f_plus - f_minus) / 2.0;

    // Initial conditions: pendulum 1 displaced, pendulum 2 at rest
    data.qpos[0] = INITIAL_THETA1;
    data.qpos[1] = 0.0;
    let _ = data.forward(&model);

    // Pivot markers (small gray spheres)
    let pivot1_physics = &model.body_pos[PEND1_BODY];
    let pivot2_physics = &model.body_pos[PEND2_BODY];
    for pivot in [pivot1_physics, pivot2_physics] {
        commands.spawn((
            Mesh3d(meshes.add(Sphere::new(0.03))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.5, 0.5, 0.5),
                metallic: 0.6,
                ..default()
            })),
            Transform::from_xyz(pivot.x as f32, pivot.z as f32, pivot.y as f32),
        ));
    }

    // Support bar between pivots
    commands.spawn((
        Mesh3d(meshes.add(Capsule3d::new(0.015, 1.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.4, 0.4, 0.4),
            metallic: 0.8,
            ..default()
        })),
        Transform::from_xyz(0.0, pivot1_physics.z as f32, 0.0)
            .with_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)),
    ));

    // Camera — front view (along -Y physics = -Z Bevy), centered between pivots
    let cam_target = Vec3::new(0.0, 1.4, 0.0);
    let orbit = OrbitCamera::new()
        .with_target(cam_target)
        .with_distance(4.0)
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

    let params = CoupledParams {
        i1,
        i2,
        h1,
        h2,
        m1,
        m2,
        f_plus,
        f_minus,
        f_beat_pred,
    };

    println!("=============================================");
    println!("  COUPLED PENDULUMS — Beat Frequency");
    println!("=============================================");
    println!("Pend1: m={m1:.2}kg, L={h1:.3}m, I={i1:.4}");
    println!("Pend2: m={m2:.2}kg, L={h2:.3}m, I={i2:.4}");
    println!("Coupling: κ={KAPPA:.2} N·m/rad");
    println!("Normal modes: f₊={f_plus:.4} Hz, f₋={f_minus:.4} Hz");
    println!(
        "f_beat_pred = {f_beat_pred:.4} Hz (T = {:.2} s)",
        1.0 / f_beat_pred
    );
    println!("---------------------------------------------");
    println!("Press 'R' to reset");
    println!("=============================================");

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

/// Compute per-pendulum energy (kinetic + gravitational potential).
///
/// E_i = ½ I_i θ̇ᵢ² + mᵢgLᵢ(1 − cos(θᵢ))
fn pendulum_energy(theta: f64, omega: f64, i_eff: f64, m: f64, h: f64) -> f64 {
    0.5 * i_eff * omega * omega + m * G * h * (1.0 - theta.cos())
}

/// Coupling energy: ½κ(θ₁ − θ₂)²
fn coupling_energy(theta1: f64, theta2: f64) -> f64 {
    0.5 * KAPPA * (theta1 - theta2).powi(2)
}

/// Convert physics (x, y, z) Z-up to Bevy (x, z, y) Y-up.
fn to_bevy(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(x as f32, z as f32, y as f32)
}

/// Compute bob world position from body xpos, xmat, and CoM offset in body frame.
fn bob_world_pos(data: &PhysicsData, body: usize, h: f64) -> Vec3 {
    let pivot = &data.xpos[body];
    let mat = &data.xmat[body];
    // Bob is at (0, 0, -h) in body frame → world = pivot + xmat * (0, 0, -h)
    // Third column of xmat = body Z-axis in world
    let bx = pivot.x - mat[(0, 2)] * h;
    let by = pivot.y - mat[(1, 2)] * h;
    let bz = pivot.z - mat[(2, 2)] * h;
    to_bevy(bx, by, bz)
}

// ============================================================================
// Systems
// ============================================================================

/// Step physics with coupling torque applied before each substep.
fn step_with_coupling(model: Res<PhysicsModel>, mut data: ResMut<PhysicsData>, time: Res<Time>) {
    let frame_dt = time.delta_secs_f64();
    let steps = ((frame_dt / model.timestep).round() as usize).clamp(1, MAX_STEPS_PER_FRAME);
    for _ in 0..steps {
        // Apply coupling torque: τ₁ = -κ(θ₁ − θ₂), τ₂ = +κ(θ₁ − θ₂)
        let theta1 = data.qpos[0];
        let theta2 = data.qpos[1];
        let coupling = -KAPPA * (theta1 - theta2);
        data.qfrc_applied[0] = coupling;
        data.qfrc_applied[1] = -coupling;

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
    mut tracker: ResMut<BeatTracker>,
    mut timer: ResMut<HudTimer>,
    mut show: ResMut<ShowControls>,
) {
    if !keyboard.just_pressed(KeyCode::KeyR) {
        return;
    }

    data.qpos.fill(0.0);
    data.qvel.fill(0.0);
    data.qfrc_applied.fill(0.0);
    data.time = 0.0;
    data.qpos[0] = INITIAL_THETA1;
    let _ = data.forward(&model);

    *tracker = BeatTracker::default();
    timer.last_print_time = -1.0;
    show.0 = false;

    println!("---------------------------------------------");
    println!("  RESET — θ₁={INITIAL_THETA1:.2} rad, θ₂=0");
    println!("---------------------------------------------");
}

/// Track beat frequency by detecting E₁ peaks.
///
/// The energy of pendulum 1 oscillates at frequency |f₊ − f₋| (twice the
/// beat frequency). The time between consecutive E₁ maxima = T_sloshing.
/// Beat frequency = 1 / (2 × T_sloshing).
fn track_beats(
    data: Res<PhysicsData>,
    params: Res<CoupledParams>,
    mut tracker: ResMut<BeatTracker>,
) {
    let theta1 = data.qpos[0];
    let omega1 = data.qvel[0];
    let e1 = pendulum_energy(theta1, omega1, params.i1, params.m1, params.h1);

    // Skip initial transient (first 0.5s)
    if !tracker.started {
        if data.time > 0.5 {
            tracker.started = true;
            tracker.prev_e1 = e1;
            tracker.was_increasing = true;
        }
        return;
    }

    let increasing = e1 > tracker.prev_e1;

    // Detect E₁ peak: was increasing, now decreasing
    if tracker.was_increasing && !increasing && data.time > 1.0 {
        // Filter out fast-oscillation peaks: only keep peaks separated by at least
        // half the expected sloshing period (use uncoupled frequencies as estimate)
        let min_sep = 0.5 / params.f_beat_pred.max(0.01);
        let dominated_by_recent = tracker
            .peak_times
            .last()
            .is_some_and(|&t| data.time - t < min_sep);

        if !dominated_by_recent {
            tracker.peak_times.push(data.time);

            let n = tracker.peak_times.len();
            if n >= 2 {
                let total_span = tracker.peak_times[n - 1] - tracker.peak_times[0];
                let avg_sloshing_period = total_span / (n - 1) as f64;
                // f_beat = 1 / (2 × T_sloshing)
                tracker.measured_f_beat = Some(1.0 / (2.0 * avg_sloshing_period));
            }
        }
    }

    if increasing != (e1 == tracker.prev_e1 && tracker.was_increasing) {
        tracker.was_increasing = increasing;
    }
    // Only update direction on actual change (avoid noise at plateaus)
    if (e1 - tracker.prev_e1).abs() > 1e-10 {
        tracker.was_increasing = increasing;
    }
    tracker.prev_e1 = e1;
}

/// Draw both pendulums as gizmos: rod + bob + pivot marker.
fn draw_pendulums(data: Res<PhysicsData>, params: Res<CoupledParams>, mut gizmos: Gizmos) {
    let colors = [
        Color::srgb(0.3, 0.5, 0.85),  // pendulum 1: blue
        Color::srgb(0.85, 0.25, 0.2), // pendulum 2: red
    ];
    let bodies = [PEND1_BODY, PEND2_BODY];
    let lengths = [params.h1, params.h2];

    for (idx, (&body, &h)) in bodies.iter().zip(lengths.iter()).enumerate() {
        let pivot = &data.xpos[body];
        let pivot_b = to_bevy(pivot.x, pivot.y, pivot.z);
        let bob_b = bob_world_pos(&data, body, h);

        // Rod
        gizmos.line(pivot_b, bob_b, colors[idx]);

        // Thicken rod with parallel lines
        let mat = &data.xmat[body];
        let offset = 0.008_f64;
        // Offsets along body X and Y axes (perpendicular to rod which is along body Z)
        for &(ax_col, sign) in &[(0, 1.0_f64), (0, -1.0), (1, 1.0), (1, -1.0)] {
            let off = to_bevy(
                mat[(0, ax_col)] * sign * offset,
                mat[(1, ax_col)] * sign * offset,
                mat[(2, ax_col)] * sign * offset,
            );
            gizmos.line(pivot_b + off, bob_b + off, colors[idx]);
        }

        // Bob sphere (single wireframe)
        gizmos.sphere(Isometry3d::from_translation(bob_b), 0.05, colors[idx]);
    }
}

/// Draw coupling spring between bobs as a zigzag gizmo line.
fn draw_coupling_spring(data: Res<PhysicsData>, params: Res<CoupledParams>, mut gizmos: Gizmos) {
    let bob1 = bob_world_pos(&data, PEND1_BODY, params.h1);
    let bob2 = bob_world_pos(&data, PEND2_BODY, params.h2);

    let spring_color = Color::srgba(0.8, 0.8, 0.0, 0.6);
    let segments = 12;
    let amplitude = 0.04_f32;

    let dir = bob2 - bob1;
    let len = dir.length();
    if len < 1e-6 {
        return;
    }
    let along = dir / len;
    // Perpendicular direction (in Bevy Y-up, cross with Z for vertical zigzag)
    let perp = along.cross(Vec3::Z).normalize_or_zero();
    let perp = if perp.length() < 0.5 {
        along.cross(Vec3::X).normalize()
    } else {
        perp
    };

    let mut prev = bob1;
    for i in 1..=segments {
        let t = i as f32 / segments as f32;
        let base = bob1 + dir * t;
        let zigzag = if i == segments {
            0.0
        } else if i % 2 == 0 {
            amplitude
        } else {
            -amplitude
        };
        let pt = base + perp * zigzag;
        gizmos.line(prev, pt, spring_color);
        prev = pt;
    }
}

/// Window title HUD: controls initially, then live data.
fn window_hud(
    data: Res<PhysicsData>,
    params: Res<CoupledParams>,
    tracker: Res<BeatTracker>,
    show: Res<ShowControls>,
    mut windows: Query<&mut Window>,
) {
    let Ok(mut window) = windows.single_mut() else {
        return;
    };

    if show.0 && data.time < 2.0 {
        window.title = "Coupled Pendulums | R = reset | Orbit camera".to_string();
        return;
    }

    let theta1 = data.qpos[0];
    let theta2 = data.qpos[1];
    let omega1 = data.qvel[0];
    let omega2 = data.qvel[1];
    let e1 = pendulum_energy(theta1, omega1, params.i1, params.m1, params.h1);
    let e2 = pendulum_energy(theta2, omega2, params.i2, params.m2, params.h2);

    let (beat_str, err_str) = if let Some(f_meas) = tracker.measured_f_beat {
        let err = (f_meas - params.f_beat_pred) / params.f_beat_pred * 100.0;
        (format!("{f_meas:.4}"), format!("{err:+.2}%"))
    } else {
        ("---".to_string(), "---".to_string())
    };

    window.title = format!(
        "Coupled Pend | t={:.1}s | θ₁={:.1}° θ₂={:.1}° | E₁={e1:.3} E₂={e2:.3} | f_beat={beat_str} (pred {:.4}, err {err_str})",
        data.time,
        theta1.to_degrees(),
        theta2.to_degrees(),
        params.f_beat_pred,
    );
}

/// Console HUD: periodic detailed output.
fn console_hud(
    data: Res<PhysicsData>,
    params: Res<CoupledParams>,
    tracker: Res<BeatTracker>,
    mut timer: ResMut<HudTimer>,
) {
    if data.time - timer.last_print_time < 2.0 {
        return;
    }
    timer.last_print_time = data.time;

    let theta1 = data.qpos[0];
    let theta2 = data.qpos[1];
    let omega1 = data.qvel[0];
    let omega2 = data.qvel[1];
    let e1 = pendulum_energy(theta1, omega1, params.i1, params.m1, params.h1);
    let e2 = pendulum_energy(theta2, omega2, params.i2, params.m2, params.h2);
    let e_coup = coupling_energy(theta1, theta2);
    let e_total = e1 + e2 + e_coup;

    let beat_info = if let Some(f_meas) = tracker.measured_f_beat {
        let err = (f_meas - params.f_beat_pred) / params.f_beat_pred * 100.0;
        format!(
            "f_beat={f_meas:.4} Hz  pred={:.4} Hz  err={err:+.2}%  peaks={}",
            params.f_beat_pred,
            tracker.peak_times.len()
        )
    } else {
        format!(
            "f_beat=---  (measuring, peaks={})",
            tracker.peak_times.len()
        )
    };

    println!(
        "t={:.1}s  θ₁={:+.3} θ₂={:+.3}  E₁={e1:.4} E₂={e2:.4} Ec={e_coup:.4} Et={e_total:.4}  {beat_info}",
        data.time, theta1, theta2,
    );
}
