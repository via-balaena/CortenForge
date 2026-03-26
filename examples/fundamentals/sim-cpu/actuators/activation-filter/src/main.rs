//! Activation Filter — Position Servo with `FilterExact` Dynamics
//!
//! A position servo with `timeconst=0.1`, which introduces first-order
//! activation dynamics: `act_dot = (ctrl - act) / τ`. The activation state
//! `data.act[0]` lags behind `ctrl` with exponential convergence:
//! `act(t) = ctrl * (1 - e^(-t/τ))`.
//!
//! Validates:
//! - Activation state exists (model.na == 1)
//! - Filter response at τ: act ≈ 0.632 * ctrl (within 5%)
//! - Filter response at 3τ: act ≈ 0.950 * ctrl (within 3%)
//! - act_dot ≈ (ctrl - act) / τ at sampled points (within 2%)
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
    <position name="filtered_servo" joint="hinge" kp="100" dampratio="1" timeconst="0.1"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="filtered_servo"/>
    <jointpos name="jpos" joint="hinge"/>
  </sensor>
</mujoco>
"#;

const TAU: f64 = 0.1; // time constant (seconds)
const TARGET: f64 = std::f64::consts::FRAC_PI_4; // ctrl = π/4

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
    println!("=== CortenForge: Activation Filter ===");
    println!("  Position servo with FilterExact activation dynamics");
    println!("  ctrl = {TARGET:.4} rad, τ = {TAU} s");
    println!(
        "  act(τ) ≈ {:.4}, act(3τ) ≈ {:.4}",
        TARGET * (1.0 - (-1.0_f64).exp()),
        TARGET * (1.0 - (-3.0_f64).exp()),
    );
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Activation Filter".into(),
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
                    let force = d.sensor_data(m, 0)[0];
                    let pos = d.sensor_data(m, 1)[0];
                    let act = if d.act.is_empty() { 0.0 } else { d.act[0] };
                    format!("force={force:.3}  θ={pos:.4}  act={act:.4}")
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
        Vec3::new(0.0, -0.2, 0.0),
        1.8,
        std::f32::consts::FRAC_PI_4,
        0.35,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Control ─────────────────────────────────────────────────────────────────

fn apply_ctrl(mut data: ResMut<PhysicsData>) {
    if !data.ctrl.is_empty() {
        data.ctrl[0] = TARGET;
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Activation Filter");

    let force = data.sensor_data(&model, 0)[0];
    let theta = data.sensor_data(&model, 1)[0];
    let act = read_act(&data);
    let act_dot = read_act_dot(&data);

    hud.scalar("ctrl", TARGET, 4);
    hud.scalar("activation", act, 4);
    hud.scalar("act_dot", act_dot, 4);
    hud.scalar("theta", theta, 4);
    hud.scalar("time", data.time, 2);
    hud.scalar("force", force, 4);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct FilterValidation {
    /// Closest sample to t = TAU (0.1s)
    act_at_tau: Option<f64>,
    tau_time: f64,
    /// Closest sample to t = 3*TAU (0.3s)
    act_at_3tau: Option<f64>,
    triple_tau_time: f64,
    /// (ctrl - act) / tau vs data.act_dot at sampled points
    act_dot_samples: Vec<(f64, f64)>, // (expected, actual)
    reported: bool,
}

fn filter_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<FilterValidation>,
) {
    let time = data.time;

    // Skip t=0 frame
    if time < 1e-6 {
        return;
    }

    let act = read_act(&data);
    let act_dot = read_act_dot(&data);

    // Capture activation at t closest to TAU (0.1s)
    let tau_dist = (time - TAU).abs();
    if (val.act_at_tau.is_none() || tau_dist < (val.tau_time - TAU).abs()) && tau_dist < 0.05 {
        val.act_at_tau = Some(act);
        val.tau_time = time;
    }

    // Capture activation at t closest to 3*TAU (0.3s)
    let triple_tau = 3.0 * TAU;
    let triple_tau_dist = (time - triple_tau).abs();
    if (val.act_at_3tau.is_none() || triple_tau_dist < (val.triple_tau_time - triple_tau).abs())
        && triple_tau_dist < 0.005
    {
        val.act_at_3tau = Some(act);
        val.triple_tau_time = time;
    }

    // Sample act_dot at several points spread across the first 2 seconds.
    // Use wide windows (0.05s) because Bevy frame timing is variable.
    let sample_times = [0.2, 0.5, 1.0, 1.5, 2.0];
    for &st in &sample_times {
        if (time - st).abs() < 0.05 && val.act_dot_samples.len() < sample_times.len() {
            let expected = (TARGET - act) / TAU;
            val.act_dot_samples.push((expected, act_dot));
        }
    }

    // Final report
    if harness.reported() && !val.reported {
        val.reported = true;

        let na = model.na;

        // Check 1: Activation exists
        let has_activation = na == 1 && !data.act.is_empty();

        // Check 2: Filter response at τ — act ≈ 0.632 * TARGET
        let analytical_1tau = TARGET * (1.0 - (-1.0_f64).exp()); // 0.6321 * TARGET
        let tau_err_pct = val.act_at_tau.map_or(100.0, |a| {
            ((a - analytical_1tau) / analytical_1tau).abs() * 100.0
        });

        // Check 3: Filter response at 3τ — act ≈ 0.950 * TARGET
        let analytical_3tau = TARGET * (1.0 - (-3.0_f64).exp()); // 0.9502 * TARGET
        let triple_tau_err_pct = val.act_at_3tau.map_or(100.0, |a| {
            ((a - analytical_3tau) / analytical_3tau).abs() * 100.0
        });

        // Check 4: act_dot ≈ (ctrl - act) / τ
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
                name: "Filter response at τ",
                pass: tau_err_pct < 5.0,
                detail: format!(
                    "act({:.3})={:.6}, expect={analytical_1tau:.6}, err={tau_err_pct:.2}%",
                    val.tau_time,
                    val.act_at_tau.unwrap_or(0.0),
                ),
            },
            Check {
                name: "Filter response at 3τ",
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
