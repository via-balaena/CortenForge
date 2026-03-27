//! Integrator Actuator — Position Control from Velocity Input
//!
//! A general actuator whose activation integrates the control signal:
//! `act_dot = ctrl`. The activation persists when ctrl returns to zero
//! (integrator memory). Combined with a position-servo bias, this gives
//! position control from a velocity-like input: you steer the arm by
//! commanding angular rate, and it holds wherever you stop.
//!
//! Three phases demonstrate the integrator:
//! - Phase 1 (0-3s): ctrl=+1 — activation ramps up, arm swings to +90 deg
//! - Phase 2 (3-8s): ctrl=0 — activation holds (memory), arm stays at 90 deg
//! - Phase 3 (8-13s): ctrl=-1 — activation ramps down, arm swings to -90 deg
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
            pos="0 0 -0.5" rgba="0.7 0.3 0.7 1"/>
    </body>
  </worldbody>

  <actuator>
    <general name="int_actuator" joint="hinge"
             dyntype="integrator" gainprm="50"
             biastype="affine" biasprm="0 -50 -5"
             actlimited="true" actrange="-1.57 1.57"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="int_actuator"/>
    <jointpos name="jpos" joint="hinge"/>
  </sensor>
</mujoco>
"#;

/// Compute ctrl for the current time:
/// Phase 1 (0-3s): +1 (ramp up), Phase 2 (3-8s): 0 (hold), Phase 3 (8-13s): -1 (ramp down)
fn current_ctrl(time: f64) -> f64 {
    if time < 3.0 {
        1.0
    } else if time < 8.0 {
        0.0
    } else if time < 13.0 {
        -1.0
    } else {
        0.0
    }
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Integrator Actuator ===");
    println!("  Position control from velocity input: act_dot = ctrl");
    println!("  Phase 1 (0-3s):  ctrl=+1 -> activation ramps, arm swings to +90 deg");
    println!("  Phase 2 (3-8s):  ctrl= 0 -> activation holds (integrator memory)");
    println!("  Phase 3 (8-13s): ctrl=-1 -> activation ramps down, arm swings back");
    println!("  force = 50*(act - theta) - 5*velocity");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge \u{2014} Integrator Actuator".into(),
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
                    let act_deg = d.act[0].to_degrees();
                    let theta_deg = d.sensor_data(m, 1)[0].to_degrees();
                    let phase = if d.time < 3.0 {
                        "ramp"
                    } else if d.time < 8.0 {
                        "hold"
                    } else {
                        "down"
                    };
                    format!("act={act_deg:.1} theta={theta_deg:.1} [{phase}]")
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
    println!("  actrange: [-1.57, 1.57] rad = [-90, 90] deg\n");

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
        Vec3::new(0.0, -0.25, 0.0),  // arm midpoint in Bevy Y-up coords
        1.8,                         // distance
        std::f32::consts::FRAC_PI_2, // azimuth: 90 deg — face the rotation plane
        0.0,                         // elevation: level
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
    let data = data.into_inner();
    data.ctrl[0] = current_ctrl(time);
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Integrator Actuator");

    let theta_deg = data.sensor_data(&model, 1)[0].to_degrees();
    let act_deg = data.act[0].to_degrees();
    let ctrl = current_ctrl(data.time);

    hud.scalar("ctrl (rate cmd)", ctrl, 1);
    hud.scalar("activation (target deg)", act_deg, 1);
    hud.scalar("theta (actual deg)", theta_deg, 1);
    hud.scalar("time (s)", data.time, 2);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct IntegratorValidation {
    ramp_sample: Option<(f64, f64)>,
    hold_sample_4: Option<(f64, f64)>,
    hold_sample_7: Option<(f64, f64)>,
    act_min: f64,
    act_max: f64,
    act_dot_samples: Vec<(f64, f64, f64)>,
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

    if time < 1e-6 {
        return;
    }

    // Track min/max activation
    if act < val.act_min {
        val.act_min = act;
    }
    if act > val.act_max {
        val.act_max = act;
    }

    // Check 1: Sample activation closest to t=1.0 (linear ramp: act ~ 1.0)
    let dist_to_1 = (time - 1.0).abs();
    let current_best_1 = val.ramp_sample.map_or(f64::MAX, |(t, _)| (t - 1.0).abs());
    if dist_to_1 < current_best_1 {
        val.ramp_sample = Some((time, act));
    }

    // Check 2: Sample activation at t~4 and t~7 (hold: should be equal)
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

    // Check 4: act_dot == ctrl samples
    let sample_times = [0.5, 1.5, 2.5, 4.0, 5.5, 7.0, 9.0, 10.5, 12.0];
    for &st in &sample_times {
        let dist = (time - st).abs();
        let already_have = val
            .act_dot_samples
            .iter()
            .any(|&(t, _, _)| (t - st).abs() < dist);
        if !already_have && dist < 0.05 {
            val.act_dot_samples.push((time, act_dot, ctrl));
        }
    }

    // Final report
    if harness.reported() && !val.reported {
        val.reported = true;

        let (ramp_t, ramp_act) = val.ramp_sample.unwrap_or((0.0, 0.0));
        let ramp_err_pct = ((ramp_act - 1.0) / 1.0).abs() * 100.0;

        let (t4, act_4) = val.hold_sample_4.unwrap_or((0.0, 0.0));
        let (t7, act_7) = val.hold_sample_7.unwrap_or((0.0, 0.0));
        let hold_diff = (act_4 - act_7).abs();

        let clamp_ok = val.act_min >= -1.57 - 1e-6 && val.act_max <= 1.57 + 1e-6;

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
