#![allow(
    missing_docs,
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
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
use sim_ml_bridge::{ActionSpace, Tensor};

#[derive(Resource, Default)]
struct LastAction(f32);

// ── MJCF Model ─────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.02"/>
      <geom name="rod" type="capsule" size="0.05"
            fromto="0 0 0 0 0 -0.5" mass="1"
            rgba="0.48 0.48 0.50 1"/>
      <geom name="tip" type="sphere" size="0.07"
            pos="0 0 -0.5" rgba="0.85 0.3 0.2 1"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge" name="motor" ctrllimited="true" ctrlrange="-2 2"/>
  </actuator>
  <sensor>
    <jointpos joint="hinge" name="angle"/>
    <jointvel joint="hinge" name="angvel"/>
  </sensor>
</mujoco>
"#;

// ── Bevy App ───────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Action Injection ===");
    println!("  Sinusoidal torque via ActionSpace, ctrlrange=[-2, 2]");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Action Injection".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<LastAction>()
        .init_resource::<ActValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let angle = d.sensor_scalar(m, "angle").unwrap_or(0.0);
                    let angvel = d.sensor_scalar(m, "angvel").unwrap_or(0.0);
                    format!(
                        "ctrl={:+.3}  angle={angle:+.4}  angvel={angvel:+.4}",
                        d.ctrl[0]
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_action, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                act_diagnostics,
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

    let act_space = ActionSpace::builder()
        .all_ctrl()
        .build(&model)
        .expect("act space build");

    let spec = act_space.spec(&model);
    println!(
        "  act_space: dim={}, low={:?}, high={:?}\n",
        act_space.dim(),
        spec.low,
        spec.high
    );

    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.3, 0.2)));

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
        Vec3::new(0.0, 0.0, 0.75),
        2.0,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
    commands.insert_resource(act_space);
}

// ── Control ────────────────────────────────────────────────────────────────

fn apply_action(
    model: Res<PhysicsModel>,
    mut data: ResMut<PhysicsData>,
    act_space: Res<ActionSpace>,
    mut last: ResMut<LastAction>,
) {
    // Sinusoidal action: 2.0 * sin(2*t), sweeps full ctrlrange [-2, 2].
    let t = data.time;
    let action_val = (2.0 * t as f32).sin() * 2.0;
    let action = Tensor::from_slice(&[action_val], &[1]);
    act_space.apply(&action, &mut data, &model);
    last.0 = action_val;
}

// ── HUD ────────────────────────────────────────────────────────────────────

fn update_hud(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    act_space: Res<ActionSpace>,
    last: Res<LastAction>,
    mut hud: ResMut<PhysicsHud>,
) {
    hud.clear();
    hud.section("Action Injection");

    hud.raw(format!("action (f32):  {:+.6}", last.0));
    hud.raw(format!("ctrl   (f64):  {:+.6}", data.ctrl[0]));
    hud.raw(format!("act_force:     {:+.6}", data.actuator_force[0]));

    hud.section("Spec Bounds");
    let spec = act_space.spec(&model);
    if let (Some(low), Some(high)) = (&spec.low, &spec.high) {
        hud.raw(format!("low:  {low:?}"));
        hud.raw(format!("high: {high:?}"));
    }

    hud.section("State");
    let angle = data.sensor_scalar(&model, "angle").unwrap_or(0.0);
    let angvel = data.sensor_scalar(&model, "angvel").unwrap_or(0.0);
    hud.scalar("angle", angle, 4);
    hud.scalar("angvel", angvel, 4);
    hud.scalar("time", data.time, 2);
}

// ── Validation ─────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct ActValidation {
    max_ctrl_err: f64,
    sign_changes: u32,
    prev_qvel_sign: i8,
    reported: bool,
}

fn act_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    act_space: Res<ActionSpace>,
    last: Res<LastAction>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<ActValidation>,
) {
    // Track ctrl fidelity: data.ctrl[0] should match f64::from(last applied action).
    let expected_ctrl = f64::from(last.0);
    let ctrl_err = (data.ctrl[0] - expected_ctrl).abs();
    if ctrl_err > val.max_ctrl_err {
        val.max_ctrl_err = ctrl_err;
    }

    // Track oscillation: count qvel sign changes.
    let qvel = data.qvel[0];
    let sign = if qvel > 0.01 {
        1
    } else if qvel < -0.01 {
        -1
    } else {
        0
    };
    if sign != 0 && val.prev_qvel_sign != 0 && sign != val.prev_qvel_sign {
        val.sign_changes += 1;
    }
    if sign != 0 {
        val.prev_qvel_sign = sign;
    }

    // Final report.
    if harness.reported() && !val.reported {
        val.reported = true;

        let spec = act_space.spec(&model);
        let bounds_ok =
            spec.low.as_deref() == Some(&[-2.0_f32]) && spec.high.as_deref() == Some(&[2.0_f32]);

        let checks = vec![
            Check {
                name: "act_space.dim()",
                pass: act_space.dim() == 1,
                detail: format!("{}", act_space.dim()),
            },
            Check {
                name: "ctrl == action",
                pass: val.max_ctrl_err < 1e-6,
                detail: format!("max err={:.2e}", val.max_ctrl_err),
            },
            Check {
                name: "spec bounds",
                pass: bounds_ok,
                detail: format!("low={:?}, high={:?}", spec.low, spec.high),
            },
            Check {
                name: "Oscillation",
                pass: val.sign_changes >= 4,
                detail: format!("{} sign changes", val.sign_changes),
            },
        ];
        let _ = print_report("Action Injection (t=15s)", &checks);
    }
}
