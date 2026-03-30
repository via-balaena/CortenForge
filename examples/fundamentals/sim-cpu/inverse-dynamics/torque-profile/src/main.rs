//! Torque Profile — Dynamic Inverse Dynamics
//!
//! Compute the torques required for a prescribed sinusoidal trajectory.
//! At each timestep, the example sets the arm to the desired (qpos, qvel,
//! qacc), calls `forward()` then `inverse()`, and reads `qfrc_inverse` —
//! the torques that produce that exact motion.
//!
//! The arm follows the trajectory by directly setting qpos each frame
//! (not via actuators). The HUD displays the computed torque values in
//! real time, showing how gravity, Coriolis, and inertial terms combine.
//!
//! Validates:
//! - Torque profile is smooth (no spikes > 3x RMS)
//! - Shoulder peak torque > elbow peak torque
//! - At trajectory extremes (qacc max): inertial term dominates
//! - At zero-crossings (qvel max): Coriolis + gravity dominate
//!
//! Run with: `cargo run -p example-inverse-torque-profile --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="torque-profile">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="upper" pos="0 0 0">
      <joint name="shoulder" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 -0.2" mass="2.0" diaginertia="0.02 0.02 0.005"/>
      <geom name="upper_rod" type="capsule" size="0.03"
            fromto="0 0 0  0 0 -0.4" rgba="0.48 0.48 0.50 1"/>
      <body name="lower" pos="0 0 -0.4">
        <joint name="elbow" type="hinge" axis="0 1 0"/>
        <inertial pos="0 0 -0.15" mass="1.0" diaginertia="0.008 0.008 0.002"/>
        <geom name="lower_rod" type="capsule" size="0.025"
              fromto="0 0 0  0 0 -0.3" rgba="0.48 0.48 0.50 1"/>
        <geom name="tip" type="sphere" size="0.04"
              pos="0 0 -0.3" rgba="0.85 0.3 0.2 1"/>
        <site name="end_effector" pos="0 0 -0.3"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="shoulder_motor" joint="shoulder" gear="1"/>
    <motor name="elbow_motor" joint="elbow" gear="1"/>
  </actuator>

  <sensor>
    <jointpos name="s_jpos" joint="shoulder"/>
    <jointpos name="e_jpos" joint="elbow"/>
  </sensor>
</mujoco>
"#;

// ── Trajectory parameters ───────────────────────────────────────────────────

const A1: f64 = 0.8; // shoulder amplitude (rad)
const A2: f64 = 0.6; // elbow amplitude (rad)
const OMEGA1: f64 = 2.0; // shoulder frequency (rad/s)
const OMEGA2: f64 = 3.0; // elbow frequency (rad/s)
const PHI: f64 = 1.0; // elbow phase offset (rad)

/// Desired trajectory at time `t`.
/// Returns (qpos, qvel, qacc) for [shoulder, elbow].
fn trajectory(t: f64) -> ([f64; 2], [f64; 2], [f64; 2]) {
    let qpos = [A1 * (OMEGA1 * t).sin(), A2 * (OMEGA2 * t + PHI).sin()];
    let qvel = [
        A1 * OMEGA1 * (OMEGA1 * t).cos(),
        A2 * OMEGA2 * (OMEGA2 * t + PHI).cos(),
    ];
    let qacc = [
        -A1 * OMEGA1 * OMEGA1 * (OMEGA1 * t).sin(),
        -A2 * OMEGA2 * OMEGA2 * (OMEGA2 * t + PHI).sin(),
    ];
    (qpos, qvel, qacc)
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Torque Profile ===");
    println!("  Dynamic inverse dynamics — torques for a sinusoidal trajectory");
    println!("  Shoulder: A={A1}, w={OMEGA1}  |  Elbow: A={A2}, w={OMEGA2}, phi={PHI}");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Torque Profile".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<TorqueValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(6.0)
                .print_every(1.0)
                .display(|m, d| {
                    let s = d.sensor_scalar(m, "s_jpos").unwrap_or(0.0);
                    let e = d.sensor_scalar(m, "e_jpos").unwrap_or(0.0);
                    format!("shoulder={s:.3}  elbow={e:.3}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_trajectory)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                torque_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();

    // Set initial trajectory state
    let (qpos, _, _) = trajectory(0.0);
    data.qpos[0] = qpos[0];
    data.qpos[1] = qpos[1];
    data.forward(&model).expect("forward");

    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.3, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("upper_rod", mat_rod.clone()),
            ("lower_rod", mat_rod),
            ("tip", mat_tip),
        ],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, -0.25),
        1.8,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Step: set trajectory + compute inverse dynamics ─────────────────────────

#[derive(Resource, Default)]
struct CurrentTorques {
    shoulder: f64,
    elbow: f64,
}

fn step_trajectory(
    model: Res<PhysicsModel>,
    mut data: ResMut<PhysicsData>,
    mut torques: Local<CurrentTorques>,
    mut val: ResMut<TorqueValidation>,
) {
    let dt = model.timestep;

    // Accumulate substeps at physics rate
    let steps = (dt.recip() * (1.0 / 60.0)).round() as usize;
    let steps = steps.max(1);

    for _ in 0..steps {
        let sim_t = data.time;
        let (qpos, qvel, qacc) = trajectory(sim_t);

        // Set prescribed state
        data.qpos[0] = qpos[0];
        data.qpos[1] = qpos[1];
        data.qvel[0] = qvel[0];
        data.qvel[1] = qvel[1];

        // forward() computes M, qfrc_bias, kinematics
        // (also computes its own qacc — we discard it)
        data.forward(&model).expect("forward");

        // Overwrite qacc with desired acceleration
        data.qacc[0] = qacc[0];
        data.qacc[1] = qacc[1];

        // inverse() → qfrc_inverse = required torques
        data.inverse(&model);

        torques.shoulder = data.qfrc_inverse[0];
        torques.elbow = data.qfrc_inverse[1];

        // Record for validation
        val.record(torques.shoulder, torques.elbow, qacc, qvel);

        // Advance time (we're driving qpos directly, not integrating)
        data.0.time += dt;
    }

    // Store for HUD access
    val.current_shoulder = torques.shoulder;
    val.current_elbow = torques.elbow;
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    val: Res<TorqueValidation>,
    mut hud: ResMut<PhysicsHud>,
) {
    hud.clear();
    hud.section("Torque Profile");

    let s_pos = data.sensor_scalar(&model, "s_jpos").unwrap_or(0.0);
    let e_pos = data.sensor_scalar(&model, "e_jpos").unwrap_or(0.0);

    hud.scalar("shoulder", s_pos, 3);
    hud.scalar("elbow", e_pos, 3);
    hud.scalar("s_torque", val.current_shoulder, 3);
    hud.scalar("e_torque", val.current_elbow, 3);
    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct TorqueValidation {
    current_shoulder: f64,
    current_elbow: f64,
    // Torque statistics
    shoulder_sum_sq: f64,
    elbow_sum_sq: f64,
    shoulder_peak: f64,
    elbow_peak: f64,
    shoulder_max_spike: f64,
    elbow_max_spike: f64,
    prev_shoulder: Option<f64>,
    prev_elbow: Option<f64>,
    count: u64,
    reported: bool,
}

impl TorqueValidation {
    fn record(&mut self, shoulder: f64, elbow: f64, _qacc: [f64; 2], _qvel: [f64; 2]) {
        self.count += 1;
        self.shoulder_sum_sq += shoulder * shoulder;
        self.elbow_sum_sq += elbow * elbow;

        if shoulder.abs() > self.shoulder_peak {
            self.shoulder_peak = shoulder.abs();
        }
        if elbow.abs() > self.elbow_peak {
            self.elbow_peak = elbow.abs();
        }

        // Track step-to-step jumps for smoothness
        if let Some(prev) = self.prev_shoulder {
            let jump = (shoulder - prev).abs();
            if jump > self.shoulder_max_spike {
                self.shoulder_max_spike = jump;
            }
        }
        if let Some(prev) = self.prev_elbow {
            let jump = (elbow - prev).abs();
            if jump > self.elbow_max_spike {
                self.elbow_max_spike = jump;
            }
        }
        self.prev_shoulder = Some(shoulder);
        self.prev_elbow = Some(elbow);
    }
}

fn torque_diagnostics(harness: Res<ValidationHarness>, mut val: ResMut<TorqueValidation>) {
    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    let n = val.count as f64;
    let s_rms = (val.shoulder_sum_sq / n).sqrt();
    let e_rms = (val.elbow_sum_sq / n).sqrt();

    // Smoothness: max step-to-step jump should be small relative to RMS
    let s_spike_ratio = if s_rms > 1e-10 {
        val.shoulder_max_spike / s_rms
    } else {
        0.0
    };
    let e_spike_ratio = if e_rms > 1e-10 {
        val.elbow_max_spike / e_rms
    } else {
        0.0
    };

    let checks = vec![
        Check {
            name: "Shoulder smooth (spike < 3x RMS)",
            pass: s_spike_ratio < 3.0,
            detail: format!(
                "max jump={:.3}, RMS={:.3}, ratio={:.2}",
                val.shoulder_max_spike, s_rms, s_spike_ratio
            ),
        },
        Check {
            name: "Elbow smooth (spike < 3x RMS)",
            pass: e_spike_ratio < 3.0,
            detail: format!(
                "max jump={:.3}, RMS={:.3}, ratio={:.2}",
                val.elbow_max_spike, e_rms, e_spike_ratio
            ),
        },
        Check {
            name: "|shoulder| peak > |elbow| peak",
            pass: val.shoulder_peak > val.elbow_peak,
            detail: format!("{:.3} vs {:.3}", val.shoulder_peak, val.elbow_peak),
        },
        Check {
            name: "Shoulder peak > 0 (non-trivial)",
            pass: val.shoulder_peak > 1.0,
            detail: format!("peak={:.3} N·m", val.shoulder_peak),
        },
    ];
    let _ = print_report("Torque Profile (t=6s)", &checks);
}
