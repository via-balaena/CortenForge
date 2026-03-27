//! ActuatorFrc + JointActuatorFrc Sensors
//!
//! A hinge arm with a motor actuator. A custom Bevy system applies constant
//! `ctrl[0] = 5.0` every frame. ActuatorFrc reads `actuator_force`,
//! JointActuatorFrc reads `qfrc_actuator` — two different code paths that
//! should agree for a single motor with gear=1.
//!
//! Validates:
//! - ActuatorFrc sensor == data.actuator_force[0] (pipeline check)
//! - ActuatorFrc ≈ 5.0 (ctrl * gain * gear = 5 * 1 * 1)
//! - JointActuatorFrc == ActuatorFrc (cross-path check, gear=1 ⟹ moment=1)
//! - |ActuatorFrc| > 1.0 at all times after t=0.01s (non-zero check)
//!
//! Run with: `cargo run -p example-sensor-actuator-force --release`

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

const CTRL_VALUE: f64 = 5.0;

const MJCF: &str = r#"
<mujoco model="actuator_force">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <!-- Support frame -->
        <geom name="beam" type="capsule" size="0.022"
              fromto="-0.25 0 0  0.25 0 0" rgba="0.40 0.40 0.43 1"/>
        <geom name="post_l" type="capsule" size="0.022"
              fromto="-0.25 0 0  -0.25 0 0.30" rgba="0.40 0.40 0.43 1"/>
        <geom name="post_r" type="capsule" size="0.022"
              fromto="0.25 0 0  0.25 0 0.30" rgba="0.40 0.40 0.43 1"/>

        <body name="arm" pos="0 0 0">
            <joint name="hinge" type="hinge" axis="1 0 0" damping="0.1"/>
            <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="rod" type="capsule" size="0.02"
                  fromto="0 0 0  0 0 -0.5" rgba="0.48 0.48 0.50 1"/>
            <geom name="tip" type="sphere" size="0.05"
                  pos="0 0 -0.5" rgba="0.85 0.55 0.15 1"/>
        </body>
    </worldbody>

    <actuator>
        <motor name="motor" joint="hinge" gear="1"/>
    </actuator>

    <sensor>
        <actuatorfrc name="afrc" actuator="motor"/>
        <jointactuatorfrc name="jfrc" joint="hinge"/>
    </sensor>
</mujoco>
"#;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: ActuatorFrc + JointActuatorFrc Sensors ===");
    println!("  Motor with ctrl={CTRL_VALUE}, gear=1, damping=0.1");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Actuator Force".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<SensorValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let afrc = d.sensor_scalar(m, "afrc").unwrap_or(0.0);
                    let jfrc = d.sensor_scalar(m, "jfrc").unwrap_or(0.0);
                    let angle = d.joint_qpos(m, 0)[0].to_degrees();
                    format!(
                        "angle={angle:+6.1}°  afrc={afrc:+6.3}  jfrc={jfrc:+6.3}  diff={:.2e}",
                        (afrc - jfrc).abs()
                    )
                }),
        )
        .add_systems(Startup, setup)
        // set_control MUST run before step_physics_realtime
        .add_systems(Update, (set_control, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                sensor_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

/// Apply constant control input every frame.
fn set_control(mut data: ResMut<PhysicsData>) {
    data.set_ctrl(0, CTRL_VALUE);
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} actuators, {} sensors\n",
        model.nbody, model.njnt, model.nu, model.nsensor
    );

    // ── Materials ────────────────────────────────────────────────────────
    let mat_frame = materials.add(MetalPreset::BrushedMetal.material());
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.55, 0.15)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("beam", mat_frame.clone()),
            ("post_l", mat_frame.clone()),
            ("post_r", mat_frame),
            ("rod", mat_rod),
            ("tip", mat_tip),
        ],
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

// ── HUD ────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Actuator Force");
    let afrc = data.sensor_scalar(&model, "afrc").unwrap_or(0.0);
    let jfrc = data.sensor_scalar(&model, "jfrc").unwrap_or(0.0);
    let angle = data.joint_qpos(&model, 0)[0].to_degrees();
    hud.scalar("angle", angle, 1);
    hud.scalar("afrc", afrc, 3);
    hud.scalar("jfrc", jfrc, 3);
    hud.scalar("|afrc-jfrc|", (afrc - jfrc).abs(), 2);
}

// ── Sensor Validation ───────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SensorValidation {
    afrc_pipeline_max_err: f64,
    afrc_analytical_max_err: f64,
    cross_path_max_err: f64,
    nonzero_violations: u32,
    reported: bool,
}

fn sensor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SensorValidation>,
) {
    let afrc_sensor = data.sensor_scalar(&model, "afrc").unwrap_or(0.0);
    let jfrc_sensor = data.sensor_scalar(&model, "jfrc").unwrap_or(0.0);

    // Pipeline check: sensor == actuator_force[0]
    let afrc_data = data.actuator_force.first().copied().unwrap_or(0.0);
    let err = (afrc_sensor - afrc_data).abs();
    if err > val.afrc_pipeline_max_err {
        val.afrc_pipeline_max_err = err;
    }

    // Analytical check: afrc ≈ CTRL_VALUE (for motor with gain=1, gear=1)
    // Skip first frame — ctrl hasn't been applied yet at t=0
    if data.time > 0.01 {
        let analytical_err = (afrc_sensor - CTRL_VALUE).abs();
        if analytical_err > val.afrc_analytical_max_err {
            val.afrc_analytical_max_err = analytical_err;
        }
    }

    // Cross-path: afrc == jfrc (single motor, gear=1 ⟹ moment=1)
    let cross_err = (afrc_sensor - jfrc_sensor).abs();
    if cross_err > val.cross_path_max_err {
        val.cross_path_max_err = cross_err;
    }

    // Non-zero check (after initial transient)
    if data.time > 0.01 && afrc_sensor.abs() < 1.0 {
        val.nonzero_violations += 1;
    }

    // Report
    if harness.reported() && !val.reported {
        val.reported = true;
        let checks = vec![
            Check {
                name: "AfrcSensor == data",
                pass: val.afrc_pipeline_max_err < 1e-12,
                detail: format!("max error={:.2e}", val.afrc_pipeline_max_err),
            },
            Check {
                name: "Afrc ≈ ctrl",
                pass: val.afrc_analytical_max_err < 1e-6,
                detail: format!(
                    "max |afrc-{}|={:.2e}",
                    CTRL_VALUE, val.afrc_analytical_max_err
                ),
            },
            Check {
                name: "Afrc == Jfrc",
                pass: val.cross_path_max_err < 1e-12,
                detail: format!("max |afrc-jfrc|={:.2e}", val.cross_path_max_err),
            },
            Check {
                name: "|Afrc| > 1.0",
                pass: val.nonzero_violations == 0,
                detail: format!("{} violations", val.nonzero_violations),
            },
        ];
        let _ = print_report("Actuator Force (t=15s)", &checks);
    }
}
