//! Forearm Flexion — Muscle Lifting a Load
//!
//! A forearm hanging from gravity, driven by a single MuJoCo `<muscle>`
//! actuator. The muscle activates at t=0.5s, curls the arm upward, then
//! releases at t=3.0s. A 3D tendon mesh shows muscle tension (blue→red).
//!
//! Run with: `cargo run -p example-muscle-forearm-flexion --release`

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
    MuscleAttachment, MuscleMeshIndex, PhysicsAccumulator, PhysicsData, PhysicsModel,
    spawn_model_geoms, spawn_muscle_meshes, step_physics_realtime, sync_geom_transforms,
    update_muscle_meshes,
};
use sim_core::validation::{Check, print_report};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="forearm-flexion">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="upper_arm" pos="0 0 0.8">
      <geom name="upper" type="capsule" size="0.025"
            fromto="0 0 0.15  0 0 0" rgba="0.35 0.35 0.38 1"/>
      <body name="forearm" pos="0 0 0">
        <joint name="elbow" type="hinge" axis="0 1 0"
               range="-1.57 1.57" limited="true" damping="2.0"/>
        <inertial pos="0 0 -0.15" mass="1.0"
                  diaginertia="0.015 0.015 0.001"/>
        <geom name="rod" type="capsule" size="0.02"
              fromto="0 0 0  0 0 -0.3" rgba="0.48 0.48 0.50 1"/>
        <geom name="tip" type="sphere" size="0.04"
              pos="0 0 -0.3" rgba="0.85 0.3 0.2 1"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <muscle name="bicep" joint="elbow"
            scale="200" range="0.75 1.05"
            lmin="0.5" lmax="1.6" vmax="1.0"
            fpmax="0.3" fvmax="1.4"
            timeconst="0.01 0.04"/>
  </actuator>

  <sensor>
    <jointpos name="jpos" joint="elbow"/>
    <jointvel name="jvel" joint="elbow"/>
    <actuatorfrc name="act_force" actuator="bicep"/>
  </sensor>
</mujoco>
"#;

const CTRL_ONSET: f64 = 0.5;
const CTRL_RELEASE: f64 = 3.0;

// ── Bevy App ────────────────────────────────────────────────────────────────

#[derive(Resource)]
struct Attachments(Vec<MuscleAttachment>);

fn main() {
    println!("=== CortenForge: Forearm Flexion ===");
    println!("  Muscle actuator drives a forearm against gravity");
    println!("  Ctrl on at t={CTRL_ONSET:.1}s, off at t={CTRL_RELEASE:.1}s");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Forearm Flexion (Muscle)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<FlexionValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(10.0)
                .print_every(1.0)
                .display(|m, d| {
                    let force = d.sensor_scalar(m, "act_force").unwrap_or(0.0);
                    let angle = d.sensor_scalar(m, "jpos").unwrap_or(0.0);
                    let act = if m.na > 0 { d.act[0] } else { 0.0 };
                    format!("act={act:.2}  force={force:.1}  angle={angle:.2}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_ctrl, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                update_muscles,
                validation_system,
                flexion_diagnostics,
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
    data.forward(&model).expect("initial FK");

    println!(
        "  Model: {} bodies, {} joints, {} actuators",
        model.nbody, model.njnt, model.nu
    );
    println!("  F0 (auto): {:.1} N\n", model.actuator_gainprm[0][2]);

    let mat_upper = materials.add(MetalPreset::CastIron.material());
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.3, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("upper", mat_upper), ("rod", mat_rod), ("tip", mat_tip)],
    );
    spawn_muscle_meshes(&mut commands, &mut meshes, &mut materials, 1);
    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.65, 0.0),
        1.5,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );
    spawn_physics_hud(&mut commands);

    let attachments = vec![MuscleAttachment::from_actuator(&model, 0, 1.0)];
    commands.insert_resource(Attachments(attachments));
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Control ─────────────────────────────────────────────────────────────────

fn apply_ctrl(mut data: ResMut<PhysicsData>) {
    let ctrl = if data.time >= CTRL_ONSET && data.time < CTRL_RELEASE {
        1.0
    } else {
        0.0
    };
    data.set_ctrl(0, ctrl);
}

// ── Muscle mesh update (delegates to building block) ────────────────────────

fn update_muscles(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    attachments: Res<Attachments>,
    mut query: Query<(
        &MuscleMeshIndex,
        &mut Transform,
        &MeshMaterial3d<StandardMaterial>,
    )>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    update_muscle_meshes(&model, &data, &attachments.0, &mut query, &mut materials);
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Forearm Flexion");
    let force = data.sensor_scalar(&model, "act_force").unwrap_or(0.0);
    let angle = data.sensor_scalar(&model, "jpos").unwrap_or(0.0);
    let vel = data.sensor_scalar(&model, "jvel").unwrap_or(0.0);
    let act = if model.na > 0 { data.act[0] } else { 0.0 };
    hud.scalar("activation", act, 3);
    hud.scalar("force (N)", force, 1);
    hud.scalar("angle (rad)", angle, 2);
    hud.scalar("velocity", vel, 2);
    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct FlexionValidation {
    max_force_mag: f64,
    initial_angle: f64,
    initial_set: bool,
    reported: bool,
}

fn flexion_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<FlexionValidation>,
) {
    if !val.initial_set {
        val.initial_angle = data.qpos[0];
        val.initial_set = true;
    }
    val.max_force_mag = val.max_force_mag.max(data.actuator_force[0].abs());

    if harness.reported() && !val.reported {
        val.reported = true;
        let act = if model.na > 0 { data.act[0] } else { 0.0 };
        let angle = data.qpos[0];
        let angle_change = (angle - val.initial_angle).abs();
        let checks = vec![
            Check {
                name: "Activation reached 1.0",
                pass: act > 0.99,
                detail: format!("act={act:.4}"),
            },
            Check {
                name: "Muscle produced force",
                pass: val.max_force_mag > 1.0,
                detail: format!("max |force|={:.2} N", val.max_force_mag),
            },
            Check {
                name: "Arm moved",
                pass: angle_change > 0.05,
                detail: format!(
                    "initial={:.2}, final={angle:.2}, change={angle_change:.2} rad",
                    val.initial_angle
                ),
            },
        ];
        let _ = print_report("Forearm Flexion (t=10s)", &checks);
    }
}
