#![allow(
    missing_docs,
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::let_underscore_must_use
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
use sim_ml_chassis::ObservationSpace as ChassisObservationSpace;
use sim_ml_chassis_bevy::ObservationSpace;

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
            pos="0 0 -0.5" rgba="0.2 0.5 0.85 1"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge" name="motor"/>
  </actuator>
  <sensor>
    <jointpos joint="hinge" name="angle"/>
    <jointvel joint="hinge" name="angvel"/>
  </sensor>
</mujoco>
"#;

/// Initial angle: 45 degrees in radians.
const INIT_ANGLE: f64 = std::f64::consts::FRAC_PI_4;

// ── Bevy App ───────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Observation Extraction ===");
    println!("  Pendulum from 45deg — tensor vs raw data side-by-side");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Observation Extraction".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<ObsValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let angle = d.sensor_scalar(m, "angle").unwrap_or(0.0);
                    let angvel = d.sensor_scalar(m, "angvel").unwrap_or(0.0);
                    format!("angle={angle:+.4}  angvel={angvel:+.4}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                obs_diagnostics,
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

    // Start at 45 degrees.
    data.qpos[0] = INIT_ANGLE;
    data.forward(&model).expect("forward");

    let obs_space = ChassisObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model)
        .expect("obs space build");

    println!(
        "  obs_space: dim={}, spec shape={:?}\n",
        obs_space.dim(),
        obs_space.spec().shape
    );

    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.5, 0.85)));

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
    commands.insert_resource(ObservationSpace::from(obs_space));
}

// ── HUD ────────────────────────────────────────────────────────────────────

fn update_hud(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    obs_space: Res<ObservationSpace>,
    mut hud: ResMut<PhysicsHud>,
) {
    let obs = obs_space.extract(&data);
    let s = obs.as_slice();

    hud.clear();
    hud.section("Observation Extraction");
    hud.raw(format!(
        "{:<12} {:>12} {:>12} {:>12}",
        "", "raw (f64)", "obs (f32)", "error"
    ));

    // qpos[0] — joint angle.
    let raw_qpos = data.qpos[0];
    let obs_qpos = s[0];
    let err_qpos = (raw_qpos - f64::from(obs_qpos)).abs();
    hud.raw(format!(
        "{:<12} {:>+12.6} {:>+12.6} {:>12.2e}",
        "qpos[0]", raw_qpos, obs_qpos, err_qpos
    ));

    // qvel[0] — joint angular velocity.
    let raw_qvel = data.qvel[0];
    let obs_qvel = s[1];
    let err_qvel = (raw_qvel - f64::from(obs_qvel)).abs();
    hud.raw(format!(
        "{:<12} {:>+12.6} {:>+12.6} {:>12.2e}",
        "qvel[0]", raw_qvel, obs_qvel, err_qvel
    ));

    hud.section("Tensor");
    hud.raw(format!("shape: {:?}", obs.shape()));
    hud.raw(format!("dim:   {}", obs_space.dim()));

    // Sensor comparison.
    hud.section("Sensors");
    let angle = data.sensor_scalar(&model, "angle").unwrap_or(0.0);
    let angvel = data.sensor_scalar(&model, "angvel").unwrap_or(0.0);
    hud.scalar("angle", angle, 4);
    hud.scalar("angvel", angvel, 4);
    hud.scalar("time", data.time, 2);
}

// ── Validation ─────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct ObsValidation {
    max_qpos_err: f64,
    max_qvel_err: f64,
    shape_ok: bool,
    idempotent_ok: bool,
    frames: u32,
    reported: bool,
}

fn obs_diagnostics(
    data: Res<PhysicsData>,
    obs_space: Res<ObservationSpace>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<ObsValidation>,
) {
    let obs = obs_space.extract(&data);
    let s = obs.as_slice();
    val.frames += 1;

    // Check shape.
    if obs.shape() == [2] {
        if val.frames == 1 {
            val.shape_ok = true;
        }
    } else {
        val.shape_ok = false;
    }

    // Track max f64-to-f32 truncation error.
    let qpos_err = (data.qpos[0] - f64::from(s[0])).abs();
    let qvel_err = (data.qvel[0] - f64::from(s[1])).abs();
    if qpos_err > val.max_qpos_err {
        val.max_qpos_err = qpos_err;
    }
    if qvel_err > val.max_qvel_err {
        val.max_qvel_err = qvel_err;
    }

    // Idempotency: two extractions on the same Data must be identical.
    let obs2 = obs_space.extract(&data);
    if val.frames == 1 {
        val.idempotent_ok = obs.as_slice() == obs2.as_slice();
    } else if obs.as_slice() != obs2.as_slice() {
        val.idempotent_ok = false;
    }

    // Final report.
    if harness.reported() && !val.reported {
        val.reported = true;

        // f32 epsilon is ~1.19e-7; truncation from f64 values in the range
        // of this pendulum (~±1 rad, ~±5 rad/s) should be well within 1e-6.
        let checks = vec![
            Check {
                name: "obs shape",
                pass: val.shape_ok,
                detail: format!("{:?}", obs.shape()),
            },
            Check {
                name: "obs[0] vs qpos[0]",
                pass: val.max_qpos_err < 1e-6,
                detail: format!("max err={:.2e}", val.max_qpos_err),
            },
            Check {
                name: "obs[1] vs qvel[0]",
                pass: val.max_qvel_err < 1e-6,
                detail: format!("max err={:.2e}", val.max_qvel_err),
            },
            Check {
                name: "Idempotent extract",
                pass: val.idempotent_ok,
                detail: if val.idempotent_ok {
                    "identical".into()
                } else {
                    "MISMATCH".into()
                },
            },
        ];
        let _ = print_report("Observation Extraction (t=15s)", &checks);
    }
}
