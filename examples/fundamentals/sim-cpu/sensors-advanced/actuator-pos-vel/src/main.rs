//! Actuator Position & Velocity Sensors
//!
//! A pendulum driven by a position servo with gear ratio 2.0. ActuatorPos
//! and ActuatorVel sensors report the actuator's generalized coordinate,
//! which for joint transmission is simply gear × joint value.
//!
//! The HUD shows both actuator and joint readings side by side, with the
//! ratio computed live — it stays locked at exactly 2.000.
//!
//! Validates:
//! - ActuatorPos = gear × JointPos (exact to 1e-10)
//! - ActuatorVel = gear × JointVel (exact to 1e-10)
//! - Gear ratio is constant every frame
//!
//! Run with: `cargo run -p example-sensor-adv-actuator-pos-vel --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::cast_precision_loss
)]

use std::f64::consts::PI;

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
//
// Pendulum on a Y-axis hinge with a position servo (gear=2.0). The servo
// drives a sinusoidal target. Joint damping keeps tracking smooth.
//
const MJCF: &str = r#"
<mujoco model="actuator-pos-vel">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001">
        <flag energy="enable"/>
    </option>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <body name="arm" pos="0 0 1">
            <joint name="hinge" type="hinge" axis="0 1 0" damping="0.5"/>
            <geom name="rod" type="capsule" size="0.025"
                  fromto="0 0 0 0.6 0 0" mass="1.0" rgba="0.48 0.48 0.50 1"/>
            <geom name="tip_ball" type="sphere" size="0.045"
                  pos="0.6 0 0" rgba="0.6 0.2 0.8 1" mass="0.001"/>
        </body>
    </worldbody>

    <actuator>
        <position name="motor" joint="hinge" kp="100" gear="2"/>
    </actuator>

    <sensor>
        <actuatorpos name="act_pos" actuator="motor"/>
        <actuatorvel name="act_vel" actuator="motor"/>
        <jointpos name="jnt_pos" joint="hinge"/>
        <jointvel name="jnt_vel" joint="hinge"/>
    </sensor>
</mujoco>
"#;

const GEAR: f64 = 2.0;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Actuator Pos/Vel Sensors ===");
    println!("  Position servo with gear=2.0 on a pendulum");
    println!("  ActuatorPos = gear * JointPos, ActuatorVel = gear * JointVel");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Actuator Pos/Vel".into(),
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
                    let ap = d.sensor_scalar(m, "act_pos").unwrap_or(0.0);
                    let jp = d.sensor_scalar(m, "jnt_pos").unwrap_or(0.0);
                    let ratio = if jp.abs() > 1e-10 { ap / jp } else { GEAR };
                    format!("act={ap:.4}  jnt={jp:.4}  ratio={ratio:.6}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (drive_servo, step_physics_realtime).chain())
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

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let data = model.make_data();

    println!(
        "  Model: {} bodies, {} joints, {} sensors\n",
        model.nbody, model.njnt, model.nsensor
    );

    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.6, 0.2, 0.8)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("rod", mat_rod), ("tip_ball", mat_tip)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 1.0, 0.0), // Bevy Y-up
        3.0,
        std::f32::consts::FRAC_PI_4,
        0.3,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Servo Drive ─────────────────────────────────────────────────────────────

fn drive_servo(mut data: ResMut<PhysicsData>) {
    // Sinusoidal position target: ±0.5 rad with 3-second period
    let target = 0.5 * (2.0 * PI * data.time / 3.0).sin();
    data.set_ctrl(0, target);
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    let ap = data.sensor_scalar(&model, "act_pos").unwrap_or(0.0);
    let av = data.sensor_scalar(&model, "act_vel").unwrap_or(0.0);
    let jp = data.sensor_scalar(&model, "jnt_pos").unwrap_or(0.0);
    let jv = data.sensor_scalar(&model, "jnt_vel").unwrap_or(0.0);

    let pos_ratio = if jp.abs() > 1e-10 { ap / jp } else { GEAR };
    let vel_ratio = if jv.abs() > 1e-10 { av / jv } else { GEAR };

    hud.clear();
    hud.section("Actuator Pos/Vel (gear=2)");
    hud.scalar("ActuatorPos", ap, 4);
    hud.scalar("JointPos", jp, 4);
    hud.raw(format!("  pos ratio    {pos_ratio:.6}"));
    hud.raw(String::new());
    hud.scalar("ActuatorVel", av, 4);
    hud.scalar("JointVel", jv, 4);
    hud.raw(format!("  vel ratio    {vel_ratio:.6}"));
}

// ── Sensor Validation ───────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SensorValidation {
    max_pos_err: f64,
    max_vel_err: f64,
    sample_count: u64,
    reported: bool,
}

fn sensor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SensorValidation>,
) {
    let ap = data.sensor_scalar(&model, "act_pos").unwrap_or(0.0);
    let av = data.sensor_scalar(&model, "act_vel").unwrap_or(0.0);
    let jp = data.sensor_scalar(&model, "jnt_pos").unwrap_or(0.0);
    let jv = data.sensor_scalar(&model, "jnt_vel").unwrap_or(0.0);

    let pos_err = (ap - GEAR * jp).abs();
    let vel_err = (av - GEAR * jv).abs();
    val.max_pos_err = val.max_pos_err.max(pos_err);
    val.max_vel_err = val.max_vel_err.max(vel_err);
    val.sample_count += 1;

    if harness.reported() && !val.reported {
        val.reported = true;

        let checks = vec![
            Check {
                name: "ActuatorPos = gear * JointPos",
                pass: val.max_pos_err < 1e-10,
                detail: format!("max |err| = {:.2e}", val.max_pos_err),
            },
            Check {
                name: "ActuatorVel = gear * JointVel",
                pass: val.max_vel_err < 1e-10,
                detail: format!("max |err| = {:.2e}", val.max_vel_err),
            },
            Check {
                name: "Gear ratio constant",
                pass: val.max_pos_err < 1e-10 && val.max_vel_err < 1e-10,
                detail: format!(
                    "pos_err={:.2e}, vel_err={:.2e} (gear={})",
                    val.max_pos_err, val.max_vel_err, GEAR
                ),
            },
        ];
        let _ = print_report("Actuator Pos/Vel (t=15s)", &checks);
    }
}
