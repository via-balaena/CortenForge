//! Activation Filter — Position Servo with `FilterExact` Dynamics
//!
//! A position servo with `timeconst=1.0`, which introduces first-order
//! activation dynamics: `act_dot = (ctrl - act) / tau`. The activation state
//! `data.act[0]` lags behind `ctrl` with exponential convergence.
//!
//! The target alternates between +45 and -45 degrees every 3 seconds. The
//! command jumps instantly (square wave), but the activation ramps smoothly
//! (exponential) — the arm follows activation, always lagging behind the
//! command. This is a low-pass filter smoothing a square wave, made physical.
//!
//! Validates:
//! - Activation state exists (model.na == 1)
//! - Filter response at tau: act ~ 0.632 * ctrl (within 5%)
//! - Filter response at 3*tau: act ~ 0.950 * ctrl (within 3%)
//! - act_dot ~ (ctrl - act) / tau at sampled points (within 2%)
//!
//! Run with: `cargo run -p example-actuator-activation-filter --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="activation-filter">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" armature="0.01"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod" type="capsule" size="0.02"
            fromto="0 0 0  0 0 -0.5" rgba="0.48 0.48 0.50 1"/>
      <geom name="tip" type="sphere" size="0.05"
            pos="0 0 -0.5" rgba="0.9 0.6 0.1 1"/>
    </body>
  </worldbody>

  <actuator>
    <position name="filtered_servo" joint="hinge" kp="100" dampratio="1" timeconst="1.0"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="filtered_servo"/>
    <jointpos name="jpos" joint="hinge"/>
  </sensor>
</mujoco>
"#;

const TAU: f64 = 1.0; // filter time constant (seconds)
const TARGET: f64 = std::f64::consts::FRAC_PI_4; // amplitude = 45 deg
const DELAY: f64 = 1.0; // seconds before first command
const HALF_PERIOD: f64 = 3.0; // seconds per half-cycle

/// Compute the current target: 0 during delay, then square wave +/-45 deg.
fn current_target(time: f64) -> f64 {
    if time < DELAY {
        return 0.0;
    }
    let t = time - DELAY;
    let cycle = t % (2.0 * HALF_PERIOD);
    if cycle < HALF_PERIOD { TARGET } else { -TARGET }
}

/// Read `data.act[0]`, returning 0.0 if the vector is empty.
fn read_act(data: &PhysicsData) -> f64 {
    if data.act.is_empty() {
        0.0
    } else {
        data.act[0]
    }
}

/// Read `data.act_dot[0]`, returning 0.0 if the vector is empty.
fn read_act_dot(data: &PhysicsData) -> f64 {
    if data.act_dot.is_empty() {
        0.0
    } else {
        data.act_dot[0]
    }
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    let target_deg = TARGET.to_degrees();
    println!("=== CortenForge: Activation Filter ===");
    println!("  Position servo with FilterExact activation dynamics");
    println!(
        "  target = \u{00b1}{target_deg:.0}\u{00b0} square wave, tau = {TAU} s, period = {}s",
        2.0 * HALF_PERIOD
    );
    println!("  Command jumps instantly, activation ramps exponentially");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge \u{2014} Activation Filter".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<FilterValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let act_deg = (if d.act.is_empty() { 0.0 } else { d.act[0] }).to_degrees();
                    let theta_deg = d.sensor_scalar(m, "jpos").unwrap_or(0.0).to_degrees();
                    let lag_deg = act_deg - theta_deg;
                    format!(
                        "act={act_deg:.1}\u{00b0}  \u{03b8}={theta_deg:.1}\u{00b0}  lag={lag_deg:.1}\u{00b0}"
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_ctrl, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                filter_diagnostics,
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
    let data = model.make_data();

    println!(
        "  Model: {} bodies, {} joints, {} actuators, {} sensors",
        model.nbody, model.njnt, model.nu, model.nsensor
    );
    println!("  na = {} (activation states)\n", model.na);

    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.6, 0.1)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("rod", mat_rod), ("tip", mat_tip)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, -0.25, 0.0),  // arm midpoint in Bevy Y-up coords
        1.8,                         // distance
        std::f32::consts::FRAC_PI_2, // azimuth: 90°
        0.0,                         // elevation: level
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Control ─────────────────────────────────────────────────────────────────

fn apply_ctrl(mut data: ResMut<PhysicsData>) {
    let target = current_target(data.time);
    data.set_ctrl(0, target);
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Activation Filter");

    let theta_deg = data
        .sensor_scalar(&model, "jpos")
        .unwrap_or(0.0)
        .to_degrees();
    let act_deg = read_act(&data).to_degrees();
    let target_deg = current_target(data.time).to_degrees();
    let lag_deg = target_deg - act_deg;

    hud.scalar("tau (filter delay)", TAU, 2);
    hud.scalar("target (command deg)", target_deg, 1);
    hud.scalar("activation (filtered deg)", act_deg, 1);
    hud.scalar("theta (actual deg)", theta_deg, 1);
    hud.scalar("lag (target-act deg)", lag_deg, 1);
    hud.scalar("time (s)", data.time, 2);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct FilterValidation {
    /// Closest sample to t = DELAY + TAU (first ramp, 1 time constant)
    act_at_tau: Option<f64>,
    tau_time: f64,
    /// Closest sample to t = DELAY + 3*TAU (first ramp, 3 time constants)
    act_at_3tau: Option<f64>,
    triple_tau_time: f64,
    /// (expected, actual) act_dot samples
    act_dot_samples: Vec<(f64, f64)>,
    reported: bool,
}

fn filter_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<FilterValidation>,
) {
    let time = data.time;

    // Skip before the step command
    if time < DELAY + 1e-6 {
        return;
    }

    let act = read_act(&data);
    let act_dot = read_act_dot(&data);

    // ── First-ramp sampling (DELAY to DELAY + HALF_PERIOD) ──────────────

    // Capture activation at t closest to DELAY + TAU
    let tau_target_t = DELAY + TAU;
    let tau_dist = (time - tau_target_t).abs();
    if (val.act_at_tau.is_none() || tau_dist < (val.tau_time - tau_target_t).abs())
        && tau_dist < 0.05
    {
        val.act_at_tau = Some(act);
        val.tau_time = time;
    }

    // Capture activation at t closest to DELAY + 3*TAU
    let triple_tau_target_t = DELAY + 3.0 * TAU;
    let triple_tau_dist = (time - triple_tau_target_t).abs();
    if (val.act_at_3tau.is_none()
        || triple_tau_dist < (val.triple_tau_time - triple_tau_target_t).abs())
        && triple_tau_dist < 0.05
    {
        val.act_at_3tau = Some(act);
        val.triple_tau_time = time;
    }

    // ── act_dot sampling (throughout the run) ───────────────────────────

    // Sample at several points during the first ramp
    let sample_times = [DELAY + 0.5, DELAY + 1.0, DELAY + 2.0, DELAY + 2.5];
    for &st in &sample_times {
        if (time - st).abs() < 0.05 && val.act_dot_samples.len() < sample_times.len() {
            let ctrl = current_target(time);
            let expected = (ctrl - act) / TAU;
            val.act_dot_samples.push((expected, act_dot));
        }
    }

    // ── Final report ────────────────────────────────────────────────────

    if harness.reported() && !val.reported {
        val.reported = true;

        let na = model.na;

        // Check 1: Activation exists
        let has_activation = na == 1 && !data.act.is_empty();

        // Check 2: Filter response at tau — act ~ 0.632 * TARGET
        let analytical_1tau = TARGET * (1.0 - (-1.0_f64).exp());
        let tau_err_pct = val.act_at_tau.map_or(100.0, |a| {
            ((a - analytical_1tau) / analytical_1tau).abs() * 100.0
        });

        // Check 3: Filter response at 3*tau — act ~ 0.950 * TARGET
        let analytical_3tau = TARGET * (1.0 - (-3.0_f64).exp());
        let triple_tau_err_pct = val.act_at_3tau.map_or(100.0, |a| {
            ((a - analytical_3tau) / analytical_3tau).abs() * 100.0
        });

        // Check 4: act_dot ~ (ctrl - act) / tau
        let mut max_dot_err_pct = 0.0_f64;
        for &(expected, actual) in &val.act_dot_samples {
            if expected.abs() > 1e-6 {
                let err_pct = ((actual - expected) / expected).abs() * 100.0;
                max_dot_err_pct = max_dot_err_pct.max(err_pct);
            }
        }

        let checks = vec![
            Check {
                name: "Activation exists",
                pass: has_activation,
                detail: format!("na={na}, act.len()={}", data.act.len()),
            },
            Check {
                name: "Filter response at tau",
                pass: tau_err_pct < 5.0,
                detail: format!(
                    "act({:.3})={:.6}, expect={analytical_1tau:.6}, err={tau_err_pct:.2}%",
                    val.tau_time,
                    val.act_at_tau.unwrap_or(0.0),
                ),
            },
            Check {
                name: "Filter response at 3*tau",
                pass: triple_tau_err_pct < 3.0,
                detail: format!(
                    "act({:.3})={:.6}, expect={analytical_3tau:.6}, err={triple_tau_err_pct:.2}%",
                    val.triple_tau_time,
                    val.act_at_3tau.unwrap_or(0.0),
                ),
            },
            Check {
                name: "act_dot correct",
                pass: !val.act_dot_samples.is_empty() && max_dot_err_pct < 2.0,
                detail: format!(
                    "max err={max_dot_err_pct:.2}% ({} samples)",
                    val.act_dot_samples.len(),
                ),
            },
        ];
        let _ = print_report("Activation Filter (t=15s)", &checks);
    }
}
