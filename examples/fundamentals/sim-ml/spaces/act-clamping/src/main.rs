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
            pos="0 0 -0.5" rgba="0.2 0.8 0.3 1"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge" name="motor" ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
  <sensor>
    <jointpos joint="hinge" name="angle"/>
    <jointvel joint="hinge" name="angvel"/>
  </sensor>
</mujoco>
"#;

/// Ramp period: action sweeps -3 → +3 over this many seconds, then repeats.
const RAMP_PERIOD: f64 = 10.0;

fn ramp_action(time: f64) -> f32 {
    // Triangle wave: -3 → +3 → -3 over RAMP_PERIOD seconds.
    let phase = (time % RAMP_PERIOD) / RAMP_PERIOD; // 0..1
    let triangle = if phase < 0.5 {
        phase * 2.0 // 0 → 1
    } else {
        2.0 - phase * 2.0 // 1 → 0
    };
    // Map 0..1 → -3..+3
    (triangle * 6.0 - 3.0) as f32
}

// ── Bevy App ───────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Ctrl Clamping ===");
    println!("  Ramp -3→+3, ctrlrange=[-1, 1]");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Ctrl Clamping".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<LastAction>()
        .init_resource::<ClampValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|_m, d| {
                    let requested = ramp_action(d.time);
                    format!("req={requested:+.3}  ctrl={:+.3}", d.ctrl[0])
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_action, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                clamp_diagnostics,
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
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.8, 0.3)));

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
    let action_val = ramp_action(data.time);
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
    let requested = last.0;
    let ctrl = data.ctrl[0];
    let clamped = f64::from(requested).abs() > 1.0;

    hud.clear();
    hud.section("Ctrl Clamping");
    hud.raw(format!("requested: {requested:+.4}"));
    hud.raw(format!("ctrl:      {ctrl:+.4}"));
    hud.raw(format!(
        "status:    {}",
        if clamped { "CLAMPED" } else { "OK" }
    ));

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
struct ClampValidation {
    max_ctrl: f64,
    min_ctrl: f64,
    max_passthrough_err: f64,
    saw_upper_clamp: bool,
    saw_lower_clamp: bool,
    reported: bool,
}

fn clamp_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    act_space: Res<ActionSpace>,
    last: Res<LastAction>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<ClampValidation>,
) {
    let requested = f64::from(last.0);
    let ctrl = data.ctrl[0];

    // Track ctrl range.
    if ctrl > val.max_ctrl {
        val.max_ctrl = ctrl;
    }
    if ctrl < val.min_ctrl || val.min_ctrl == 0.0 {
        val.min_ctrl = ctrl;
    }

    // Check pass-through when within range.
    if requested.abs() <= 1.0 {
        let err = (ctrl - requested).abs();
        if err > val.max_passthrough_err {
            val.max_passthrough_err = err;
        }
    }

    // Check boundary clamping.
    if requested > 1.0 && (ctrl - 1.0).abs() < 1e-15 {
        val.saw_upper_clamp = true;
    }
    if requested < -1.0 && (ctrl + 1.0).abs() < 1e-15 {
        val.saw_lower_clamp = true;
    }

    if harness.reported() && !val.reported {
        val.reported = true;

        let range_ok = val.max_ctrl <= 1.0 && val.min_ctrl >= -1.0;

        let spec = act_space.spec(&model);
        let bounds_ok =
            spec.low.as_deref() == Some(&[-1.0_f32]) && spec.high.as_deref() == Some(&[1.0_f32]);

        let checks = vec![
            Check {
                name: "ctrl within range",
                pass: range_ok,
                detail: format!("[{:.4}, {:.4}]", val.min_ctrl, val.max_ctrl),
            },
            Check {
                name: "Within-range passthrough",
                pass: val.max_passthrough_err < 1e-6,
                detail: format!("max err={:.2e}", val.max_passthrough_err),
            },
            Check {
                name: "Boundary values",
                pass: val.saw_upper_clamp && val.saw_lower_clamp,
                detail: format!(
                    "upper={}, lower={}",
                    val.saw_upper_clamp, val.saw_lower_clamp
                ),
            },
            Check {
                name: "spec bounds",
                pass: bounds_ok,
                detail: format!("low={:?}, high={:?}", spec.low, spec.high),
            },
        ];
        let _ = print_report("Ctrl Clamping (t=15s)", &checks);
    }
}
