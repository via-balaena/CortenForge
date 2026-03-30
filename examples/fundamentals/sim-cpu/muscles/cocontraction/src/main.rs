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
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
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

// ── Muscle attachment ─────────────────────────────────────────────────────

struct SingleAttachment {
    parent_body: usize,
    child_body: usize,
    parent_local: [f64; 3],
    child_local: [f64; 3],
}

impl SingleAttachment {
    fn from_actuator(model: &sim_core::Model, actuator_idx: usize, x_side: f64) -> Self {
        let jnt_id = model.actuator_trnid[actuator_idx][0];
        let child_body = model.jnt_body[jnt_id];
        let parent_body = model.body_parent[child_body];

        let parent_geom_start = model.body_geom_adr[parent_body];
        let parent_radius = if parent_geom_start < model.ngeom {
            model.geom_size[parent_geom_start][0]
        } else {
            0.02
        };
        let child_geom_start = model.body_geom_adr[child_body];
        let child_radius = if child_geom_start < model.ngeom {
            model.geom_size[child_geom_start][0]
        } else {
            0.02
        };

        // x_side: +1 for agonist (front of arm), -1 for antagonist (back)
        Self {
            parent_body,
            child_body,
            parent_local: [parent_radius * x_side, 0.0, 0.10],
            child_local: [child_radius * x_side, 0.0, -0.10],
        }
    }

    fn world_points_bevy(&self, data: &sim_core::Data) -> (Vec3, Vec3) {
        let origin = Self::body_local_to_world(data, self.parent_body, &self.parent_local);
        let insertion = Self::body_local_to_world(data, self.child_body, &self.child_local);
        (origin, insertion)
    }

    fn body_local_to_world(data: &sim_core::Data, body: usize, local: &[f64; 3]) -> Vec3 {
        let pos = &data.xpos[body];
        let mat = &data.xmat[body];
        let wx = pos.x + mat[(0, 0)] * local[0] + mat[(0, 1)] * local[1] + mat[(0, 2)] * local[2];
        let wy = pos.y + mat[(1, 0)] * local[0] + mat[(1, 1)] * local[1] + mat[(1, 2)] * local[2];
        let wz = pos.z + mat[(2, 0)] * local[0] + mat[(2, 1)] * local[1] + mat[(2, 2)] * local[2];
        Vec3::new(wx as f32, wz as f32, wy as f32)
    }
}

#[derive(Resource)]
struct MuscleAttachments(Vec<SingleAttachment>);

// ── Muscle mesh ───────────────────────────────────────────────────────────

#[derive(Component)]
struct MuscleMeshIndex(usize);

fn spawn_muscle_meshes(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    let mesh = meshes.add(Cylinder::new(1.0, 1.0));
    for i in 0..2 {
        let mat = materials.add(StandardMaterial {
            base_color: Color::srgb(0.1, 0.1, 0.9),
            metallic: 0.3,
            perceptual_roughness: 0.6,
            ..default()
        });
        commands.spawn((
            Mesh3d(mesh.clone()),
            MeshMaterial3d(mat),
            Transform::default(),
            MuscleMeshIndex(i),
        ));
    }
}

fn update_muscle_meshes(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    attachments: Res<MuscleAttachments>,
    mut query: Query<(
        &MuscleMeshIndex,
        &mut Transform,
        &MeshMaterial3d<StandardMaterial>,
    )>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for (idx, mut transform, mat_handle) in &mut query {
        let i = idx.0;
        if i >= attachments.0.len() {
            continue;
        }

        let (o, ins) = attachments.0[i].world_points_bevy(&data);
        let midpoint = (o + ins) * 0.5;
        let diff = ins - o;
        let length = diff.length();
        if length < 1e-6 {
            continue;
        }
        let dir = diff / length;
        let rotation = Quat::from_rotation_arc(Vec3::Y, dir);

        let muscle_radius = 0.008;
        transform.translation = midpoint;
        transform.rotation = rotation;
        transform.scale = Vec3::new(muscle_radius, length, muscle_radius);

        // Tension-driven color: blue (rest) → red (peak force)
        let f0 = model.actuator_gainprm[i][2].max(1.0);
        let tension = (data.actuator_force[i].abs() / f0).clamp(0.0, 1.0) as f32;
        if let Some(mat) = materials.get_mut(&mat_handle.0) {
            mat.base_color = Color::srgb(0.8 * tension + 0.1, 0.1, 0.8 * (1.0 - tension) + 0.1);
        }
    }
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
                update_muscle_meshes,
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
        SingleAttachment::from_actuator(&model, 0, 1.0),
        SingleAttachment::from_actuator(&model, 1, -1.0),
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

    spawn_muscle_meshes(&mut commands, &mut meshes, &mut materials);

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.65, 0.0),
        1.5,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(MuscleAttachments(attachments));
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
