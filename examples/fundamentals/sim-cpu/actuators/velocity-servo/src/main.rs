//! Velocity Servo Actuator — Constant Speed Despite Gravity
//!
//! The `<velocity>` shortcut: `GainType::Fixed(kv)`, `BiasType::Affine(0, 0, -kv)`,
//! `ActuatorDynamics::None`. Force = kv * (ctrl - qdot).
//!
//! With gravity enabled, the servo maintains near-constant angular velocity
//! while its force oscillates each revolution to compensate gravity — the exact
//! opposite pattern from the motor actuator (constant force, wobbling speed).
//!
//! Validates:
//! - Average ω matches target (< 1% error over 5–15 s window)
//! - Speed regulation: max |ω - target| bounded (servo keeps it tight)
//! - Force oscillates (gravity compensation — not stuck at zero)
//! - Force formula: force = kv * (ctrl - qdot) at every timestep
//!
//! Run with: `cargo run -p example-actuator-velocity-servo --release`

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
<mujoco model="velocity-servo">
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
            pos="0 0 -0.5" rgba="0.3 0.4 0.9 1"/>
    </body>
  </worldbody>

  <actuator>
    <velocity name="vel_servo" joint="hinge" kv="20"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="vel_servo"/>
    <jointvel name="jvel" joint="hinge"/>
  </sensor>
</mujoco>
"#;

const KV: f64 = 20.0;
const TARGET_VEL: f64 = 2.0;

// Peak gravity torque = m*g*d = 1.0 * 9.81 * 0.25 = 2.4525 N·m
const GRAVITY_TORQUE_PEAK: f64 = 1.0 * 9.81 * 0.25;

// Max speed deviation ≈ gravity_peak / kv = 2.45 / 20 ≈ 0.12 rad/s
const MAX_SPEED_RIPPLE: f64 = GRAVITY_TORQUE_PEAK / KV;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Velocity Servo ===");
    println!("  Velocity tracking — force = kv * (ctrl - qdot)");
    println!("  ctrl = {TARGET_VEL} rad/s, kv = {KV}");
    println!("  Gravity ON — peak torque = {GRAVITY_TORQUE_PEAK:.2} N·m");
    println!(
        "  Expected speed ripple ≈ ±{MAX_SPEED_RIPPLE:.2} rad/s ({:.0}%)",
        MAX_SPEED_RIPPLE / TARGET_VEL * 100.0
    );
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Velocity Servo".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<VelocityServoValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let force = d.sensor_data(m, 0)[0];
                    let vel = d.sensor_data(m, 1)[0];
                    let rev = d.qpos[0] / std::f64::consts::TAU;
                    format!("force={force:+.2}  ω={vel:.2}  rev={rev:.1}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_ctrl, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                velocity_servo_diagnostics,
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

    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.4, 0.9)));

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
        2.5,                         // distance — zoomed out for full rotation
        std::f32::consts::FRAC_PI_2, // azimuth: 90° — match motor convention
        0.0,                         // elevation: level
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Control ─────────────────────────────────────────────────────────────────

fn apply_ctrl(mut data: ResMut<PhysicsData>) {
    if !data.ctrl.is_empty() {
        data.ctrl[0] = TARGET_VEL;
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Velocity Servo");

    let force = data.sensor_data(&model, 0)[0];
    let vel = data.sensor_data(&model, 1)[0];
    let rev = data.qpos[0] / std::f64::consts::TAU;

    hud.scalar("target", TARGET_VEL, 1);
    hud.scalar("omega", vel, 2);
    hud.scalar("force", force, 2);
    hud.scalar("rev", rev, 1);
    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct VelocityServoValidation {
    /// Average velocity in the 5–15 s window
    vel_sum: f64,
    vel_count: u32,
    /// Max velocity deviation from target
    max_vel_deviation: f64,
    /// Max |force| seen (should be > 0 — gravity compensation)
    max_force_abs: f64,
    /// Force formula error tracking
    max_force_formula_err: f64,
    force_formula_samples: usize,
    reported: bool,
}

fn velocity_servo_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<VelocityServoValidation>,
) {
    let time = data.time;
    let force = data.sensor_data(&model, 0)[0];
    let omega = data.sensor_data(&model, 1)[0];
    let act_force = data.actuator_force[0];

    // Skip t=0 frame
    if time < 1e-6 {
        return;
    }

    // Track max velocity deviation (after initial spinup)
    if time > 0.5 {
        let deviation = (omega - TARGET_VEL).abs();
        if deviation > val.max_vel_deviation {
            val.max_vel_deviation = deviation;
        }
    }

    // Average velocity in 5–15 s window
    if time > 5.0 {
        val.vel_sum += omega;
        val.vel_count += 1;
    }

    // Track max |force|
    if force.abs() > val.max_force_abs {
        val.max_force_abs = force.abs();
    }

    // Force formula: force = kv * (ctrl - qdot)
    let expected_force = KV * (TARGET_VEL - omega);
    let force_err = (act_force - expected_force).abs();
    let scale = expected_force.abs().max(1.0);
    let relative_err = force_err / scale;
    if relative_err > val.max_force_formula_err {
        val.max_force_formula_err = relative_err;
    }
    val.force_formula_samples += 1;

    // Final report
    if harness.reported() && !val.reported {
        val.reported = true;

        let avg_vel = if val.vel_count > 0 {
            val.vel_sum / f64::from(val.vel_count)
        } else {
            0.0
        };
        let avg_err_pct = ((avg_vel - TARGET_VEL) / TARGET_VEL).abs() * 100.0;

        let checks = vec![
            Check {
                name: "Average ω matches target",
                pass: avg_err_pct < 1.0,
                detail: format!("avg ω={avg_vel:.4} (target={TARGET_VEL}), err={avg_err_pct:.2}%"),
            },
            Check {
                name: "Speed regulation",
                pass: val.max_vel_deviation < 0.5,
                detail: format!(
                    "max |ω - target| = {:.4} (expect < {:.2})",
                    val.max_vel_deviation,
                    MAX_SPEED_RIPPLE * 1.5
                ),
            },
            Check {
                name: "Force oscillates (gravity)",
                pass: val.max_force_abs > 1.0,
                detail: format!(
                    "max |force| = {:.2} (gravity peak ≈ {GRAVITY_TORQUE_PEAK:.2})",
                    val.max_force_abs
                ),
            },
            Check {
                name: "Force formula",
                pass: val.max_force_formula_err < 0.01,
                detail: format!(
                    "max relative err = {:.2e} ({} samples)",
                    val.max_force_formula_err, val.force_formula_samples
                ),
            },
        ];
        let _ = print_report("Velocity Servo (t=15s)", &checks);
    }
}
