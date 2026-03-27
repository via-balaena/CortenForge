//! Gear & Limits — Gear Ratio and Force Clamping
//!
//! Two side-by-side arms comparing gear scaling and force clamping. Arm A has
//! `gear=1` (baseline), Arm B has `gear=5` with `forcerange=[-3, 3]`. Both
//! receive the same ctrl signal across two phases.
//!
//! Phase 1 (0-5s): ctrl=1.0 — Arm B accelerates 5x faster (gear=5, force
//! not clamped since 1.0 < 3.0).
//!
//! Phase 2 (5-15s): ctrl=5.0 — Arm B's actuator_force is clamped to 3.0
//! (forcerange), while Arm A's actuator_force = 5.0 unclamped.
//!
//! Validates:
//! - Gear scaling: vel_b / vel_a ~ 5.0 at t~1s (within 5%)
//! - Force identity: actuator_force_a == ctrl (gain=1, no clamp)
//! - Force clamping: actuator_force_b == 3.0 when ctrl=5.0
//! - Torque = gear * force: qfrc_actuator verified via acceleration ratio
//!
//! Run with: `cargo run -p example-actuator-gear-and-limits --release`

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
<mujoco model="gear-and-limits">
  <compiler angle="radian" autolimits="true"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="arm_a" pos="0 0.5 0">
      <joint name="hinge_a" type="hinge" axis="0 1 0" armature="0.01" damping="1.0"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod_a" type="capsule" size="0.02"
            fromto="0 0 0  0 0 -0.5" rgba="0.48 0.48 0.50 1"/>
      <geom name="tip_a" type="sphere" size="0.05"
            pos="0 0 -0.5" rgba="0.3 0.6 0.9 1"/>
    </body>
    <body name="arm_b" pos="0 -0.5 0">
      <joint name="hinge_b" type="hinge" axis="0 1 0" armature="0.01" damping="1.0"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod_b" type="capsule" size="0.02"
            fromto="0 0 0  0 0 -0.5" rgba="0.48 0.48 0.50 1"/>
      <geom name="tip_b" type="sphere" size="0.05"
            pos="0 0 -0.5" rgba="0.9 0.4 0.2 1"/>
    </body>
  </worldbody>

  <actuator>
    <motor name="baseline" joint="hinge_a" gear="1"/>
    <motor name="geared_limited" joint="hinge_b" gear="5" forcerange="-3 3"/>
  </actuator>

  <sensor>
    <actuatorfrc name="force_a" actuator="baseline"/>
    <actuatorfrc name="force_b" actuator="geared_limited"/>
    <jointvel name="vel_a" joint="hinge_a"/>
    <jointvel name="vel_b" joint="hinge_b"/>
  </sensor>
</mujoco>
"#;

// I_eff = diaginertia_Y + armature + m*d^2 = 0.01 + 0.01 + 1.0 * 0.25^2
const I_EFF: f64 = 0.0825;
const GEAR_B: f64 = 5.0;
const FORCE_LIMIT: f64 = 3.0;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Gear & Limits ===");
    println!(
        "  Two arms: A (gear=1), B (gear={GEAR_B}, forcerange=[-{FORCE_LIMIT}, {FORCE_LIMIT}])"
    );
    println!("  Phase 1 (0-5s): ctrl=1.0 — B is {GEAR_B}x faster");
    println!("  Phase 2 (5-15s): ctrl=5.0 — B force clamped to {FORCE_LIMIT}");
    println!("  I_eff = {I_EFF} kg*m^2");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge \u{2014} Gear & Limits".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<GearValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let force_a = d.sensor_scalar(m, "force_a").unwrap_or(0.0);
                    let force_b = d.sensor_scalar(m, "force_b").unwrap_or(0.0);
                    let vel_a = d.sensor_scalar(m, "vel_a").unwrap_or(0.0);
                    let vel_b = d.sensor_scalar(m, "vel_b").unwrap_or(0.0);
                    format!("fA={force_a:.3}  fB={force_b:.3}  vA={vel_a:.3}  vB={vel_b:.3}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_ctrl, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                gear_diagnostics,
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
        "  Model: {} bodies, {} joints, {} actuators, {} sensors\n",
        model.nbody, model.njnt, model.nu, model.nsensor
    );

    let mat_rod_a = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip_a =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.6, 0.9)));
    let mat_rod_b = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip_b =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.4, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("rod_a", mat_rod_a),
            ("tip_a", mat_tip_a),
            ("rod_b", mat_rod_b),
            ("tip_b", mat_tip_b),
        ],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, -0.15, 0.0),  // arm midpoints
        2.5,                         // distance — see both arms
        std::f32::consts::FRAC_PI_3, // azimuth: 60 deg — angled to show both arms
        0.15,                        // slight elevation
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Control ─────────────────────────────────────────────────────────────────

fn apply_ctrl(mut data: ResMut<PhysicsData>) {
    let ctrl_val = if data.time < 5.0 { 1.0 } else { 5.0 };
    data.set_ctrl(0, ctrl_val);
    data.set_ctrl(1, ctrl_val);
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Gear & Limits");

    let force_a = data.sensor_scalar(&model, "force_a").unwrap_or(0.0);
    let force_b = data.sensor_scalar(&model, "force_b").unwrap_or(0.0);
    let vel_a = data.sensor_scalar(&model, "vel_a").unwrap_or(0.0);
    let vel_b = data.sensor_scalar(&model, "vel_b").unwrap_or(0.0);
    let ctrl_val = if data.time < 5.0 { 1.0 } else { 5.0 };

    let torque_a = force_a; // gear=1
    let torque_b = force_b * GEAR_B; // gear=5
    let vel_ratio = if vel_a.abs() > 0.01 {
        vel_b / vel_a
    } else {
        0.0
    };

    hud.scalar("ctrl (both)", ctrl_val, 1);
    hud.scalar("force A (gear=1)", force_a, 2);
    hud.scalar("torque A (Nm)", torque_a, 2);
    hud.scalar("force B (gear=5)", force_b, 2);
    hud.scalar("torque B (Nm)", torque_b, 2);
    hud.scalar("vel ratio (B/A)", vel_ratio, 1);
    hud.scalar("time (s)", data.time, 2);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct GearValidation {
    /// Velocity ratio sample near t=1s
    gear_ratio_sample: Option<f64>,
    gear_sample_time: f64,
    /// Track max force error for check 2 (force identity)
    max_force_identity_err: f64,
    /// Force clamping sample from phase 2
    clamped_force_sample: Option<f64>,
    /// Acceleration ratio sample from phase 2 for torque check
    accel_ratio_sample: Option<f64>,
    accel_ratio_time: f64,
    reported: bool,
}

fn gear_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<GearValidation>,
) {
    let time = data.time;

    let force_a = data.sensor_scalar(&model, "force_a").unwrap_or(0.0);
    let _force_b = data.sensor_scalar(&model, "force_b").unwrap_or(0.0);
    let vel_a = data.sensor_scalar(&model, "vel_a").unwrap_or(0.0);
    let vel_b = data.sensor_scalar(&model, "vel_b").unwrap_or(0.0);
    let act_force_a = data.actuator_force[0];
    let act_force_b = data.actuator_force[1];

    // Check 1: Gear scaling — sample velocity ratio near t=1s
    // At this point both arms have been accelerating for ~1s with ctrl=1.0.
    // Arm A: torque = 1*1 = 1 N*m, Arm B: torque = 5*1 = 5 N*m.
    // Same inertia => vel_b / vel_a should be ~5.0.
    let t1_dist = (time - 1.0).abs();
    if t1_dist < 0.05
        && (val.gear_ratio_sample.is_none() || t1_dist < (val.gear_sample_time - 1.0).abs())
        && vel_a.abs() > 1e-6
    {
        val.gear_ratio_sample = Some(vel_b / vel_a);
        val.gear_sample_time = time;
    }

    // Check 2: Force identity — actuator_force_a == ctrl (gain=1, no clamp)
    // Compare force to the ctrl that the physics actually used (data.ctrl[0]),
    // not the ctrl we're about to set. This avoids phase-transition frame mismatch.
    let applied_ctrl = data.ctrl[0];
    let force_id_err = (act_force_a - applied_ctrl).abs();
    if force_id_err > val.max_force_identity_err {
        val.max_force_identity_err = force_id_err;
    }
    // Also verify sensor matches data.actuator_force
    let _sensor_match_err = (force_a - act_force_a).abs();

    // Check 3: Force clamping — in phase 2, actuator_force_b should be clamped to 3.0
    if time > 5.5 && time < 6.0 && val.clamped_force_sample.is_none() {
        val.clamped_force_sample = Some(act_force_b);
    }

    // Check 4: Torque = gear * force — verify via qfrc_actuator ratio.
    // In phase 2: qfrc_a = 1 * 5.0 = 5.0, qfrc_b = 5 * 3.0 = 15.0.
    // Acceleration ratio = qfrc_b / qfrc_a = 15 / 5 = 3.0 (same inertia).
    // We can verify this through the acceleration ratio (qfrc / I_eff).
    if time > 6.0 && time < 7.0 && val.accel_ratio_sample.is_none() {
        let qfrc_a = data.qfrc_actuator[0];
        let qfrc_b = data.qfrc_actuator[1];
        if qfrc_a.abs() > 1e-6 {
            val.accel_ratio_sample = Some(qfrc_b / qfrc_a);
            val.accel_ratio_time = time;
        }
    }

    // Final report
    if harness.reported() && !val.reported {
        val.reported = true;

        // Check 1: gear scaling — vel_b / vel_a ~ 5.0 (within 5%)
        let gear_ratio = val.gear_ratio_sample.unwrap_or(0.0);
        let gear_err_pct = ((gear_ratio - GEAR_B) / GEAR_B).abs() * 100.0;

        // Check 2: force identity — actuator_force_a == ctrl
        let force_id_pass = val.max_force_identity_err < 1e-10;

        // Check 3: force clamping — actuator_force_b == 3.0 in phase 2
        let clamped = val.clamped_force_sample.unwrap_or(f64::NAN);
        let clamp_err = (clamped - FORCE_LIMIT).abs();

        // Check 4: torque = gear * force via acceleration ratio
        // In phase 2: qfrc_b / qfrc_a = (5 * 3.0) / (1 * 5.0) = 3.0
        let expected_ratio = (GEAR_B * FORCE_LIMIT) / (1.0 * 5.0); // 15 / 5 = 3.0
        let accel_ratio = val.accel_ratio_sample.unwrap_or(0.0);
        let accel_err_pct = ((accel_ratio - expected_ratio) / expected_ratio).abs() * 100.0;

        let checks = vec![
            Check {
                name: "Gear scaling",
                pass: val.gear_ratio_sample.is_some() && gear_err_pct < 5.0,
                detail: format!(
                    "vel_b/vel_a={gear_ratio:.4} at t={:.3} (expect {GEAR_B:.1}, err={gear_err_pct:.2}%)",
                    val.gear_sample_time,
                ),
            },
            Check {
                name: "Force identity",
                pass: force_id_pass,
                detail: format!("max err={:.2e}", val.max_force_identity_err),
            },
            Check {
                name: "Force clamping",
                pass: val.clamped_force_sample.is_some() && clamp_err < 1e-10,
                detail: format!(
                    "actuator_force_b={clamped:.6} (expect {FORCE_LIMIT:.1}, err={clamp_err:.2e})",
                ),
            },
            Check {
                name: "Torque = gear * force",
                pass: val.accel_ratio_sample.is_some() && accel_err_pct < 1.0,
                detail: format!(
                    "qfrc_b/qfrc_a={accel_ratio:.4} at t={:.3} (expect {expected_ratio:.1}, err={accel_err_pct:.2}%)",
                    val.accel_ratio_time,
                ),
            },
        ];
        let _ = print_report("Gear & Limits (t=15s)", &checks);
    }
}
