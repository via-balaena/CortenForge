//! Integrator Actuator — General Actuator with Integrator Dynamics
//!
//! A general actuator whose activation integrates the control signal over time:
//! `act = integral(ctrl) dt`. The activation persists even when ctrl returns to
//! zero (integrator memory). Force is computed via affine bias:
//! `force = gain * act + bias_0 + bias_1 * length + bias_2 * velocity`.
//!
//! Three phases demonstrate the integrator behavior:
//! - Phase 1 (0-3s): ctrl=1.0, activation ramps linearly
//! - Phase 2 (3-8s): ctrl=0.0, activation holds (integrator memory)
//! - Phase 3 (8-13s): ctrl=-1.0, activation ramps down
//!
//! Validates:
//! - Linear ramp: act at t~1.0 ~ 1.0 (integral of constant 1)
//! - Hold at zero ctrl: activation stays constant when ctrl=0
//! - Activation clamping: act stays within actrange [-1.57, 1.57]
//! - act_dot == ctrl: integrator dynamics are exact
//!
//! Run with: `cargo run -p example-actuator-integrator --release`

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
<mujoco model="integrator">
  <compiler angle="radian" autolimits="true"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>

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
            pos="0 0 -0.5" rgba="0.7 0.3 0.7 1"/>
    </body>
  </worldbody>

  <actuator>
    <general name="int_actuator" joint="hinge"
             dyntype="integrator" gainprm="50"
             biastype="affine" biasprm="0 0 -5"
             actlimited="true" actrange="-1.57 1.57"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="int_actuator"/>
    <jointpos name="jpos" joint="hinge"/>
  </sensor>
</mujoco>
"#;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Integrator Actuator ===");
    println!("  General actuator with integrator dynamics");
    println!("  act = integral(ctrl) dt");
    println!("  Phase 1 (0-3s): ctrl=1.0 -> activation ramps up");
    println!("  Phase 2 (3-8s): ctrl=0.0 -> activation holds");
    println!("  Phase 3 (8-13s): ctrl=-1.0 -> activation ramps down");
    println!("  force = 50*act + 0 + 0*length + (-5)*velocity");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Integrator Actuator".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<IntegratorValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let force = d.sensor_data(m, 0)[0];
                    let pos = d.sensor_data(m, 1)[0];
                    let act = d.act[0];
                    format!("force={force:.3}  act={act:.4}  theta={pos:.4}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_ctrl, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                integrator_diagnostics,
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
    println!("  actrange: [-1.57, 1.57]\n");

    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.7, 0.3, 0.7)));

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

fn apply_ctrl(data: ResMut<PhysicsData>) {
    if data.ctrl.is_empty() {
        return;
    }
    let time = data.time;
    let ctrl = if time < 3.0 {
        1.0
    } else if time < 8.0 {
        0.0
    } else {
        -1.0
    };
    // Must use interior-mutability-safe approach — data is ResMut
    let data = data.into_inner();
    data.ctrl[0] = ctrl;
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Integrator Actuator");

    let force = data.sensor_data(&model, 0)[0];
    let pos = data.sensor_data(&model, 1)[0];
    let act = data.act[0];
    let act_dot = data.act_dot[0];
    let ctrl = data.ctrl[0];

    hud.scalar("ctrl", ctrl, 3);
    hud.scalar("activation", act, 4);
    hud.scalar("act_dot", act_dot, 6);
    hud.scalar("force", force, 4);
    hud.scalar("theta", pos, 4);
    hud.scalar("time", data.time, 2);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct IntegratorValidation {
    /// Activation sample closest to t=1.0 for linear ramp check
    ramp_sample: Option<(f64, f64)>, // (time, act)
    /// Activation samples at t~4 and t~7 for hold check
    hold_sample_4: Option<(f64, f64)>, // (time, act)
    hold_sample_7: Option<(f64, f64)>, // (time, act)
    /// Track min/max activation for clamping check
    act_min: f64,
    act_max: f64,
    /// Samples of (act_dot, ctrl) for act_dot == ctrl check
    act_dot_samples: Vec<(f64, f64, f64)>, // (time, act_dot, ctrl)
    reported: bool,
}

fn integrator_diagnostics(
    _model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<IntegratorValidation>,
) {
    let time = data.time;
    let act = data.act[0];
    let act_dot = data.act_dot[0];
    let ctrl = data.ctrl[0];

    // Skip t=0 frame
    if time < 1e-6 {
        return;
    }

    // Track min/max activation (Check 3: clamping)
    if act < val.act_min {
        val.act_min = act;
    }
    if act > val.act_max {
        val.act_max = act;
    }

    // Check 1: Sample activation closest to t=1.0
    let dist_to_1 = (time - 1.0).abs();
    let current_best_1 = val.ramp_sample.map_or(f64::MAX, |(t, _)| (t - 1.0).abs());
    if dist_to_1 < current_best_1 {
        val.ramp_sample = Some((time, act));
    }

    // Check 2: Sample activation closest to t=4.0 and t=7.0
    let dist_to_4 = (time - 4.0).abs();
    let current_best_4 = val.hold_sample_4.map_or(f64::MAX, |(t, _)| (t - 4.0).abs());
    if dist_to_4 < current_best_4 {
        val.hold_sample_4 = Some((time, act));
    }

    let dist_to_7 = (time - 7.0).abs();
    let current_best_7 = val.hold_sample_7.map_or(f64::MAX, |(t, _)| (t - 7.0).abs());
    if dist_to_7 < current_best_7 {
        val.hold_sample_7 = Some((time, act));
    }

    // Check 4: Collect act_dot samples at representative points in each phase
    // Phase 1: ~0.5s, ~1.5s, ~2.5s
    // Phase 2: ~4.0s, ~5.5s, ~7.0s
    // Phase 3: ~9.0s, ~10.5s, ~12.0s
    let sample_times = [0.5, 1.5, 2.5, 4.0, 5.5, 7.0, 9.0, 10.5, 12.0];
    for &st in &sample_times {
        let dist = (time - st).abs();
        // Check if we already have a sample near this time
        let already_have = val
            .act_dot_samples
            .iter()
            .any(|&(t, _, _)| (t - st).abs() < dist);
        if !already_have && dist < 0.001 {
            val.act_dot_samples.push((time, act_dot, ctrl));
        }
    }

    // Final report
    if harness.reported() && !val.reported {
        val.reported = true;

        // Check 1: Linear ramp — act at t~1.0 should be ~1.0
        let (ramp_t, ramp_act) = val.ramp_sample.unwrap_or((0.0, 0.0));
        let ramp_err_pct = ((ramp_act - 1.0) / 1.0).abs() * 100.0;

        // Check 2: Hold at zero ctrl — activation constant between t~4 and t~7
        let (t4, act_4) = val.hold_sample_4.unwrap_or((0.0, 0.0));
        let (t7, act_7) = val.hold_sample_7.unwrap_or((0.0, 0.0));
        let hold_diff = (act_4 - act_7).abs();

        // Check 3: Activation clamping — never exceeds [-1.57, 1.57]
        let clamp_ok = val.act_min >= -1.57 - 1e-6 && val.act_max <= 1.57 + 1e-6;

        // Check 4: act_dot == ctrl — should be exact for integrator
        let mut max_act_dot_err = 0.0_f64;
        for &(_t, ad, c) in &val.act_dot_samples {
            let err = (ad - c).abs();
            if err > max_act_dot_err {
                max_act_dot_err = err;
            }
        }

        let checks = vec![
            Check {
                name: "Linear ramp (act@t=1 ~ 1.0)",
                pass: ramp_err_pct < 5.0,
                detail: format!(
                    "at t={ramp_t:.4}: act={ramp_act:.4} (expect 1.0), err={ramp_err_pct:.2}%"
                ),
            },
            Check {
                name: "Hold at zero ctrl",
                pass: hold_diff < 0.01,
                detail: format!(
                    "act@t={t4:.1}={act_4:.4}, act@t={t7:.1}={act_7:.4}, diff={hold_diff:.6}"
                ),
            },
            Check {
                name: "Activation clamping [-1.57, 1.57]",
                pass: clamp_ok,
                detail: format!("min={:.4}, max={:.4}", val.act_min, val.act_max),
            },
            Check {
                name: "act_dot == ctrl (exact)",
                pass: max_act_dot_err < 1e-10,
                detail: format!(
                    "max |act_dot - ctrl| = {max_act_dot_err:.2e} ({} samples)",
                    val.act_dot_samples.len()
                ),
            },
        ];
        let _ = print_report("Integrator Actuator (t=15s)", &checks);
    }
}
