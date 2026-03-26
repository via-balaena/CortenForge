//! Cylinder Actuator — Pneumatic Cylinder with Filter Dynamics
//!
//! A pneumatic/hydraulic cylinder on a horizontal rail (no gravity). The
//! cylinder shortcut maps to `GainType::Fixed(area)`,
//! `BiasType::Affine(bias[0], bias[1], bias[2])`,
//! `ActuatorDynamics::Filter(tau)`.  Force = `area * act + bias[0] +
//! bias[1]*length + bias[2]*velocity`.  Filter dynamics is Euler-approximated
//! (not FilterExact), which is the key code-path difference from example 5.
//!
//! Validates:
//! - Activation state exists (data.na == 1)
//! - Filter dynamics: act at t ~= tau is ~0.632 (within 5%)
//! - Force formula: force ~= area*act + bias[0] + bias[1]*pos + bias[2]*vel (within 2%)
//! - Area from diameter: model.actuator_gainprm[0][0] ~= pi/4 * d^2 (within 1e-10)
//!
//! Run with: `cargo run -p example-actuator-cylinder --release`

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
<mujoco model="cylinder">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="cart" pos="0 0 0">
      <joint name="piston" type="slide" axis="1 0 0" armature="0.01"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.001 0.001 0.001"/>
      <geom name="block" type="box" size="0.08 0.05 0.05" rgba="0.5 0.7 0.3 1"/>
    </body>
  </worldbody>

  <actuator>
    <cylinder name="pneumatic" joint="piston" timeconst="0.2" diameter="0.08" bias="0 -5 -2"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="pneumatic"/>
    <jointpos name="jpos" joint="piston"/>
    <jointvel name="jvel" joint="piston"/>
  </sensor>
</mujoco>
"#;

const CTRL_PRESSURE: f64 = 1.0;

// Area = pi/4 * diameter^2
const DIAMETER: f64 = 0.08;
const AREA: f64 = std::f64::consts::PI / 4.0 * DIAMETER * DIAMETER;
const TAU: f64 = 0.2;
const BIAS: [f64; 3] = [0.0, -5.0, -2.0];
// Spring equilibrium: area / |bias[1]| (where force from activation balances spring return)
const X_EQ: f64 = AREA / 5.0;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Cylinder Actuator ===");
    println!("  Pneumatic cylinder with Filter dynamics");
    println!("  ctrl = {CTRL_PRESSURE} (full pressure), tau = {TAU} s");
    println!("  area = {AREA:.6} m^2 (d = {DIAMETER} m)");
    println!("  bias = [{}, {}, {}]", BIAS[0], BIAS[1], BIAS[2]);
    println!("  x_eq = {X_EQ:.6} m (spring equilibrium)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Cylinder Actuator".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<CylinderValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let force = d.sensor_data(m, 0)[0];
                    let pos = d.sensor_data(m, 1)[0];
                    let vel = d.sensor_data(m, 2)[0];
                    format!("force={force:.4}  x={pos:.6}  v={vel:.6}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_ctrl, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                cylinder_diagnostics,
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
        "  Model: {} bodies, {} joints, {} actuators, {} sensors, na={}",
        model.nbody, model.njnt, model.nu, model.nsensor, model.na
    );
    println!("  gainprm[0] = {:.10}\n", model.actuator_gainprm[0][0]);

    let mat_block =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.5, 0.7, 0.3)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("block", mat_block)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.0),
        1.2,
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
        data.ctrl[0] = CTRL_PRESSURE;
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Cylinder Actuator");

    let force = data.sensor_data(&model, 0)[0];
    let pos = data.sensor_data(&model, 1)[0];
    let vel = data.sensor_data(&model, 2)[0];
    let act = if data.act.is_empty() {
        0.0
    } else {
        data.act[0]
    };

    hud.scalar("ctrl", CTRL_PRESSURE, 3);
    hud.scalar("activation", act, 6);
    hud.scalar("force", force, 6);
    hud.scalar("position", pos, 6);
    hud.scalar("velocity", vel, 6);
    hud.scalar("time", data.time, 2);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct CylinderValidation {
    /// Sample closest to t = TAU for filter dynamics check
    tau_sample: Option<(f64, f64)>, // (time, activation)
    /// Samples of (time, force, act, pos, vel) for force formula check
    force_samples: Vec<(f64, f64, f64, f64, f64)>,
    reported: bool,
}

fn cylinder_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<CylinderValidation>,
) {
    let time = data.time;
    let force_sensor = data.sensor_data(&model, 0)[0];
    let pos = data.sensor_data(&model, 1)[0];
    let vel = data.sensor_data(&model, 2)[0];
    let act = if data.act.is_empty() {
        0.0
    } else {
        data.act[0]
    };

    // Skip t=0 frame (actuator_force not yet computed before first step)
    if time < 1e-6 {
        return;
    }

    // Track the sample closest to t = TAU for filter dynamics check
    let dist_to_tau = (time - TAU).abs();
    let current_best = val.tau_sample.map_or(f64::MAX, |(t, _)| (t - TAU).abs());
    if dist_to_tau < current_best {
        val.tau_sample = Some((time, act));
    }

    // Collect force samples every ~10ms for force formula check (first 2s)
    if time <= 2.0 {
        let last_t = val.force_samples.last().map_or(0.0, |s| s.0);
        if time - last_t >= 0.009 {
            val.force_samples.push((time, force_sensor, act, pos, vel));
        }
    }

    // Final report
    if harness.reported() && !val.reported {
        val.reported = true;

        // Check 1: Activation state exists (data.na == 1)
        let na = model.na;
        let na_ok = na == 1;

        // Check 2: Filter dynamics — act at t ~= TAU should be ~0.632 (1 - e^-1)
        let (tau_t, tau_act) = val.tau_sample.unwrap_or((0.0, 0.0));
        let expected_act = 1.0 - (-1.0_f64).exp(); // 1 - e^-1 ≈ 0.6321
        let filter_err_pct = ((tau_act - expected_act) / expected_act).abs() * 100.0;

        // Check 3: Force formula — force ≈ area*act + bias[0] + bias[1]*pos + bias[2]*vel
        // actuator_length = gear * joint_pos (gear=1 default for joint transmission)
        let mut max_force_err_pct = 0.0_f64;
        let mut worst_detail = String::new();
        for &(_t, force, a, p, v) in &val.force_samples {
            let length = p; // gear = 1
            let expected_force = AREA * a + BIAS[0] + BIAS[1] * length + BIAS[2] * v;
            if expected_force.abs() > 1e-10 {
                let err_pct = ((force - expected_force) / expected_force).abs() * 100.0;
                if err_pct > max_force_err_pct {
                    max_force_err_pct = err_pct;
                    worst_detail = format!(
                        "f={force:.6} vs expected={expected_force:.6} (a={a:.4}, p={p:.6}, v={v:.6})"
                    );
                }
            }
        }

        // Check 4: Area from diameter — model.actuator_gainprm[0][0] ≈ AREA
        let gainprm_area = model.actuator_gainprm[0][0];
        let area_err = (gainprm_area - AREA).abs();

        let checks = vec![
            Check {
                name: "Activation exists (na==1)",
                pass: na_ok,
                detail: format!("data.na = {na}"),
            },
            Check {
                name: "Filter dynamics act(tau) ~= 0.632",
                pass: filter_err_pct < 5.0,
                detail: format!(
                    "at t={tau_t:.4}: act={tau_act:.4} (expect {expected_act:.4}), err={filter_err_pct:.2}%"
                ),
            },
            Check {
                name: "Force formula (area*act + bias)",
                pass: max_force_err_pct < 2.0,
                detail: format!(
                    "max err={max_force_err_pct:.4}% ({} samples) worst: {worst_detail}",
                    val.force_samples.len()
                ),
            },
            Check {
                name: "Area from diameter",
                pass: area_err < 1e-10,
                detail: format!(
                    "gainprm[0]={gainprm_area:.12}, expected={AREA:.12}, err={area_err:.2e}"
                ),
            },
        ];
        let _ = print_report("Cylinder Actuator (t=15s)", &checks);
    }
}
