//! Cocontraction — Agonist-Antagonist Muscle Pair
//!
//! Two opposing muscles on the same elbow joint. When both activate
//! simultaneously, net torque is near zero but the joint becomes stiff.
//! The demo cycles through: rest → cocontraction → agonist-only →
//! antagonist-only → rest, showing how opposing muscles control both
//! position and stiffness independently.
//!
//! Run with: `cargo run -p example-muscle-cocontraction --release`

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

/// Forearm with two opposing muscles: agonist (gear=1) and antagonist (gear=-1).
/// Both are MuJoCo `<muscle>` with identical parameters except gear sign.
const MJCF: &str = r#"
<mujoco model="cocontraction">
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
               range="-1.57 1.57" limited="true" damping="0.5"/>
        <inertial pos="0 0 -0.15" mass="1.0"
                  diaginertia="0.015 0.015 0.001"/>
        <geom name="rod" type="capsule" size="0.02"
              fromto="0 0 0  0 0 -0.3" rgba="0.48 0.48 0.50 1"/>
        <geom name="tip" type="sphere" size="0.04"
              pos="0 0 -0.3" rgba="0.7 0.7 0.72 1"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <muscle name="agonist" joint="elbow" gear="1"
            scale="50" range="0.75 1.05"
            lmin="0.5" lmax="1.6" vmax="1.0"
            fpmax="0.3" fvmax="1.4"
            timeconst="0.01 0.04"/>
    <muscle name="antagonist" joint="elbow" gear="-1"
            scale="50" range="0.75 1.05"
            lmin="0.5" lmax="1.6" vmax="1.0"
            fpmax="0.3" fvmax="1.4"
            timeconst="0.01 0.04"/>
  </actuator>

  <sensor>
    <jointpos name="jpos" joint="elbow"/>
    <jointvel name="jvel" joint="elbow"/>
    <actuatorfrc name="ago_force" actuator="agonist"/>
    <actuatorfrc name="ant_force" actuator="antagonist"/>
  </sensor>
</mujoco>
"#;

// ── Control phases ──────────────────────────────────────────────────────────

/// Returns (agonist_ctrl, antagonist_ctrl) for the current time.
fn ctrl_schedule(time: f64) -> (f64, f64) {
    if time < 1.0 {
        (0.0, 0.0) // Rest
    } else if time < 4.0 {
        (1.0, 1.0) // Cocontraction — both active, joint stiffens
    } else if time < 7.0 {
        (1.0, 0.0) // Agonist only — flexion
    } else if time < 10.0 {
        (0.0, 1.0) // Antagonist only — extension
    } else {
        (0.0, 0.0) // Rest
    }
}

fn phase_label(time: f64) -> &'static str {
    if time < 1.0 {
        "REST"
    } else if time < 4.0 {
        "COCONTRACT"
    } else if time < 7.0 {
        "AGONIST"
    } else if time < 10.0 {
        "ANTAGONIST"
    } else {
        "REST"
    }
}

// ── Muscle attachment (shared building blocks) ───────────────────────────

#[derive(Resource)]
struct Attachments(Vec<MuscleAttachment>);

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

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Cocontraction ===");
    println!("  Agonist + antagonist on same joint");
    println!("  Phases: rest → cocontract → agonist → antagonist → rest");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Cocontraction".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<CocontractionValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(12.0)
                .print_every(1.0)
                .display(|m, d| {
                    let ago = d.sensor_scalar(m, "ago_force").unwrap_or(0.0);
                    let ant = d.sensor_scalar(m, "ant_force").unwrap_or(0.0);
                    let angle = d.sensor_scalar(m, "jpos").unwrap_or(0.0);
                    let phase = phase_label(d.time);
                    format!("[{phase:^11}]  ago={ago:.1}  ant={ant:.1}  angle={angle:.2}")
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
                cocontraction_diagnostics,
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
    for i in 0..model.nu {
        let name = model.actuator_name[i].as_deref().unwrap_or("?");
        let gear = model.actuator_gear[i][0];
        let f0 = model.actuator_gainprm[i][2];
        println!("  actuator[{i}] '{name}': gear={gear:.0}, F0={f0:.1}");
    }
    println!();

    // Agonist on +X side, antagonist on -X side of the arm
    let attachments = vec![
        MuscleAttachment::from_actuator(&model, 0, 1.0),
        MuscleAttachment::from_actuator(&model, 1, -1.0),
    ];

    let mat_upper = materials.add(MetalPreset::CastIron.material());
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.7, 0.7, 0.72)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("upper", mat_upper), ("rod", mat_rod), ("tip", mat_tip)],
    );

    spawn_muscle_meshes(&mut commands, &mut meshes, &mut materials, 2);

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.65, 0.0),
        1.5,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(Attachments(attachments));
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Control ─────────────────────────────────────────────────────────────────

fn apply_ctrl(mut data: ResMut<PhysicsData>) {
    let (ago, ant) = ctrl_schedule(data.time);
    data.set_ctrl(0, ago);
    data.set_ctrl(1, ant);
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Cocontraction");

    let phase = phase_label(data.time);
    let ago_force = data.sensor_scalar(&model, "ago_force").unwrap_or(0.0);
    let ant_force = data.sensor_scalar(&model, "ant_force").unwrap_or(0.0);
    let angle = data.sensor_scalar(&model, "jpos").unwrap_or(0.0);
    let vel = data.sensor_scalar(&model, "jvel").unwrap_or(0.0);
    let ago_act = data.act[model.actuator_act_adr[0]];
    let ant_act = data.act[model.actuator_act_adr[1]];

    hud.raw(format!("  phase: {phase}"));
    hud.scalar("agonist act", ago_act, 2);
    hud.scalar("agonist force", ago_force, 1);
    hud.scalar("antagonist act", ant_act, 2);
    hud.scalar("antagonist force", ant_force, 1);
    hud.scalar("angle (rad)", angle, 2);
    hud.scalar("velocity", vel, 2);
    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
#[allow(clippy::struct_excessive_bools)]
struct CocontractionValidation {
    /// Max |force| seen from each muscle during cocontraction phase
    max_ago_cocontract: f64,
    max_ant_cocontract: f64,
    /// Angle range during cocontraction (should be small)
    cocontract_angle_min: f64,
    cocontract_angle_max: f64,
    cocontract_sampled: bool,
    /// Did agonist-only phase move the joint?
    agonist_angle: f64,
    agonist_sampled: bool,
    /// Did antagonist-only phase move the joint the other way?
    antagonist_angle: f64,
    antagonist_sampled: bool,
    reported: bool,
}

fn cocontraction_diagnostics(
    _model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<CocontractionValidation>,
) {
    let time = data.time;
    let angle = data.qpos[0];

    // Sample during cocontraction (t=2-3s, well into the phase)
    if time > 2.0 && time < 3.0 {
        let ago = data.actuator_force[0].abs();
        let ant = data.actuator_force[1].abs();
        val.max_ago_cocontract = val.max_ago_cocontract.max(ago);
        val.max_ant_cocontract = val.max_ant_cocontract.max(ant);
        if !val.cocontract_sampled {
            val.cocontract_angle_min = angle;
            val.cocontract_angle_max = angle;
            val.cocontract_sampled = true;
        }
        val.cocontract_angle_min = val.cocontract_angle_min.min(angle);
        val.cocontract_angle_max = val.cocontract_angle_max.max(angle);
    }

    // Sample at end of agonist-only phase (t=4-7)
    if time > 6.5 && time < 7.0 && !val.agonist_sampled {
        val.agonist_angle = angle;
        val.agonist_sampled = true;
    }

    // Sample at end of antagonist-only phase (t=7-10)
    if time > 9.5 && time < 10.0 && !val.antagonist_sampled {
        val.antagonist_angle = angle;
        val.antagonist_sampled = true;
    }

    if harness.reported() && !val.reported {
        val.reported = true;

        let cocontract_range = val.cocontract_angle_max - val.cocontract_angle_min;

        let checks = vec![
            Check {
                name: "Both muscles active during cocontraction",
                pass: val.max_ago_cocontract > 1.0 && val.max_ant_cocontract > 1.0,
                detail: format!(
                    "max |ago|={:.2}, max |ant|={:.2}",
                    val.max_ago_cocontract, val.max_ant_cocontract
                ),
            },
            Check {
                name: "Joint stable during cocontraction",
                pass: cocontract_range < 0.3,
                detail: format!("angle range={cocontract_range:.3} rad (need <0.3)"),
            },
            Check {
                name: "Agonist-only moves joint",
                pass: val.agonist_sampled && val.agonist_angle.abs() > 0.1,
                detail: format!("angle={:.2} rad", val.agonist_angle),
            },
            Check {
                name: "Antagonist-only moves joint opposite",
                pass: val.antagonist_sampled
                    && val.agonist_sampled
                    && (val.antagonist_angle - val.agonist_angle).abs() > 0.1,
                detail: format!(
                    "ago_angle={:.2}, ant_angle={:.2}, diff={:.2}",
                    val.agonist_angle,
                    val.antagonist_angle,
                    val.antagonist_angle - val.agonist_angle,
                ),
            },
        ];
        let _ = print_report("Cocontraction (t=12s)", &checks);
    }
}
