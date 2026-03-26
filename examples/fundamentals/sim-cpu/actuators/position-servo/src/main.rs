//! Position Servo Actuator — PD Position Control
//!
//! PD position control without activation dynamics. The `<position>` shortcut
//! maps to `GainType::Fixed(kp)`, `BiasType::Affine(0, -kp, -kv)`,
//! `ActuatorDynamics::None`. Force = kp * (ctrl - q) - kv * qdot.
//!
//! Validates:
//! - Reaches target (|theta - target| < 0.05 rad after 0.5s)
//! - No overshoot (critically damped — theta never exceeds target)
//! - Force formula (force approx kp*(ctrl - q) - kv*qdot at sampled points)
//! - No activation state (data.act is empty, na=0)
//!
//! Run with: `cargo run -p example-actuator-position-servo --release`

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
<mujoco model="position-servo">
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
            pos="0 0 -0.5" rgba="0.2 0.7 0.3 1"/>
    </body>
  </worldbody>

  <actuator>
    <position name="servo" joint="hinge" kp="100" dampratio="1"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="servo"/>
    <jointpos name="jpos" joint="hinge"/>
    <jointvel name="jvel" joint="hinge"/>
  </sensor>
</mujoco>
"#;

const TARGET: f64 = std::f64::consts::FRAC_PI_4; // pi/4 = 45 degrees
const KP: f64 = 100.0;

// I_eff = diaginertia_Y + armature + m*d^2 = 0.01 + 0.01 + 1.0 * 0.25^2
const I_EFF: f64 = 0.0825;

// Expected kv from dampratio=1: kv = 2 * sqrt(kp * I_eff) ~ 5.74
const EXPECTED_KV: f64 = 2.0 * 5.744_562_646_538_03; // 2 * sqrt(100 * 0.0825)

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Position Servo ===");
    println!("  PD position control — kp={KP}, dampratio=1");
    println!(
        "  target = {TARGET:.4} rad ({:.1} deg)",
        TARGET.to_degrees()
    );
    println!("  I_eff = {I_EFF} kg*m^2, expected kv ~ {EXPECTED_KV:.2}");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Position Servo".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<ServoValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let force = d.sensor_data(m, 0)[0];
                    let pos = d.sensor_data(m, 1)[0];
                    let vel = d.sensor_data(m, 2)[0];
                    format!("force={force:.3}  theta={pos:.4}  omega={vel:.4}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_ctrl, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                servo_diagnostics,
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
        "  Model: {} bodies, {} joints, {} actuators, {} sensors",
        model.nbody, model.njnt, model.nu, model.nsensor
    );
    println!("  na = {} (activation states)\n", model.na);

    // Print the converted kv from biasprm
    let kv = -model.actuator_biasprm[0][2];
    println!("  biasprm[0] = {:?}", &model.actuator_biasprm[0][..3]);
    println!("  kv (from -biasprm[2]) = {kv:.6}\n");

    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.7, 0.3)));

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

fn apply_ctrl(mut data: ResMut<PhysicsData>) {
    if !data.ctrl.is_empty() {
        data.ctrl[0] = TARGET;
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Position Servo");

    let force = data.sensor_data(&model, 0)[0];
    let pos = data.sensor_data(&model, 1)[0];
    let vel = data.sensor_data(&model, 2)[0];

    hud.scalar("ctrl", TARGET, 4);
    hud.scalar("force", force, 4);
    hud.scalar("theta", pos, 4);
    hud.scalar("omega", vel, 4);
    hud.scalar("time", data.time, 2);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct ServoValidation {
    max_theta: f64,
    max_force_formula_err: f64,
    force_formula_samples: usize,
    reached_target: bool,
    kv: f64,
    reported: bool,
}

fn servo_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<ServoValidation>,
) {
    let time = data.time;
    let theta = data.sensor_data(&model, 1)[0];
    let omega = data.sensor_data(&model, 2)[0];
    let act_force = data.actuator_force[0];

    // Skip t=0 frame (actuator_force not yet computed before first step)
    if time < 1e-6 {
        return;
    }

    // Read kv once from model (converted from dampratio by compute_actuator_params)
    if val.kv == 0.0 {
        val.kv = -model.actuator_biasprm[0][2];
    }

    // Track max theta for overshoot check
    if theta > val.max_theta {
        val.max_theta = theta;
    }

    // Check 1: Reaches target after 0.5s
    if time >= 0.5 && (theta - TARGET).abs() < 0.05 {
        val.reached_target = true;
    }

    // Check 3: Force formula — force = kp*(ctrl - q) - kv*qdot
    let expected_force = KP * (TARGET - theta) - val.kv * omega;
    let force_err = (act_force - expected_force).abs();
    // Use relative tolerance for large forces, absolute for small
    let scale = expected_force.abs().max(1.0);
    let relative_err = force_err / scale;
    if relative_err > val.max_force_formula_err {
        val.max_force_formula_err = relative_err;
    }
    val.force_formula_samples += 1;

    // Final report
    if harness.reported() && !val.reported {
        val.reported = true;

        let no_activation = data.act.is_empty();
        let na = model.na;

        let checks = vec![
            Check {
                name: "Reaches target",
                pass: val.reached_target,
                detail: format!(
                    "|theta - target| = {:.4} rad (need < 0.05)",
                    (theta - TARGET).abs()
                ),
            },
            Check {
                name: "No overshoot",
                pass: val.max_theta <= TARGET + 0.001,
                detail: format!(
                    "max_theta = {:.6}, target = {:.6}, diff = {:.6}",
                    val.max_theta,
                    TARGET,
                    val.max_theta - TARGET
                ),
            },
            Check {
                name: "Force formula",
                pass: val.max_force_formula_err < 0.01,
                detail: format!(
                    "max relative err = {:.2e} ({} samples, kv={:.4})",
                    val.max_force_formula_err, val.force_formula_samples, val.kv
                ),
            },
            Check {
                name: "No activation state",
                pass: no_activation && na == 0,
                detail: format!("na={na}, act.len()={}", data.act.len()),
            },
        ];
        let _ = print_report("Position Servo (t=15s)", &checks);
    }
}
