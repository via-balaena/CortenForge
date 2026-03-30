//! Forearm Flexion — Muscle Lifting a Load
//!
//! A forearm hanging from gravity, driven by a single MuJoCo `<muscle>`
//! actuator. The muscle activates at t=0.5s and curls the arm upward,
//! demonstrating activation dynamics and force-length-velocity curves.
//! A gizmo line shows the muscle path, derived from the joint's parent/child
//! body positions — no manual site placement needed.
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
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
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

// ── Muscle attachment derived from the model ──────────────────────────────

/// Identifies the two bodies a muscle spans (parent and child of its joint).
/// Local offsets place the attachment points on the body surface near the joint.
#[derive(Resource)]
struct MuscleAttachment {
    parent_body: usize,
    child_body: usize,
    /// Local offset [x, y, z] from parent body origin (body-local frame)
    parent_local: [f64; 3],
    /// Local offset [x, y, z] from child body origin (body-local frame)
    child_local: [f64; 3],
}

impl MuscleAttachment {
    /// Derive attachment from a joint-transmission muscle actuator.
    fn from_model(model: &sim_core::Model, actuator_idx: usize) -> Self {
        let jnt_id = model.actuator_trnid[actuator_idx][0];
        let child_body = model.jnt_body[jnt_id];
        let parent_body = model.body_parent[child_body];

        // Find capsule radius of each body's first geom for surface offset
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

        // Origin: high on the upper arm (near shoulder)
        let parent_local = [parent_radius, 0.0, 0.10];
        // Insertion: well down the forearm (1/3 of the way)
        let child_local = [child_radius, 0.0, -0.10];

        Self {
            parent_body,
            child_body,
            parent_local,
            child_local,
        }
    }

    /// Compute world-space attachment points from current body poses.
    /// Returns Bevy Vec3 (Y-up) ready for gizmo drawing.
    fn world_points_bevy(&self, data: &sim_core::Data) -> (Vec3, Vec3) {
        let origin = Self::body_local_to_world(data, self.parent_body, &self.parent_local);
        let insertion = Self::body_local_to_world(data, self.child_body, &self.child_local);
        (origin, insertion)
    }

    fn body_local_to_world(data: &sim_core::Data, body: usize, local: &[f64; 3]) -> Vec3 {
        let pos = &data.xpos[body];
        let mat = &data.xmat[body];
        // world = body_pos + body_rot * local
        let wx = pos.x + mat[(0, 0)] * local[0] + mat[(0, 1)] * local[1] + mat[(0, 2)] * local[2];
        let wy = pos.y + mat[(1, 0)] * local[0] + mat[(1, 1)] * local[1] + mat[(1, 2)] * local[2];
        let wz = pos.z + mat[(2, 0)] * local[0] + mat[(2, 1)] * local[1] + mat[(2, 2)] * local[2];
        // Z-up → Y-up swap
        Vec3::new(wx as f32, wz as f32, wy as f32)
    }
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Forearm Flexion ===");
    println!("  Muscle actuator drives a forearm against gravity");
    println!("  Ctrl onset at t={CTRL_ONSET:.1}s, MuJoCo <muscle>, scale=200");
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
                update_muscle_mesh,
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

    let f0 = model.actuator_gainprm[0][2];
    println!(
        "  Model: {} bodies, {} joints, {} actuators",
        model.nbody, model.njnt, model.nu
    );
    println!("  F0 (auto): {f0:.1} N\n");

    let attachment = MuscleAttachment::from_model(&model, 0);

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

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.65, 0.0), // center on arm (Y-up Bevy coords)
        1.5,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );

    spawn_muscle_mesh(&mut commands, &mut meshes, &mut materials);
    spawn_physics_hud(&mut commands);

    commands.insert_resource(attachment);
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Control ─────────────────────────────────────────────────────────────────

/// Release muscle at this time (seconds).
const CTRL_RELEASE: f64 = 3.0;

fn apply_ctrl(mut data: ResMut<PhysicsData>) {
    let ctrl = if data.time >= CTRL_ONSET && data.time < CTRL_RELEASE {
        1.0
    } else {
        0.0
    };
    data.set_ctrl(0, ctrl);
}

// ── Muscle Gizmo ────────────────────────────────────────────────────────────

/// Marker for the muscle mesh entity so we can query it.
#[derive(Component)]
struct MuscleMesh;

/// Spawn the muscle as a cylinder mesh between attachment points.
fn spawn_muscle_mesh(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    // Unit cylinder (height 1, radius 1) — we'll scale/position each frame
    let mesh = meshes.add(Cylinder::new(1.0, 1.0));
    let mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.1, 0.1, 0.9),
        metallic: 0.3,
        perceptual_roughness: 0.6,
        ..default()
    });

    commands.spawn((
        Mesh3d(mesh),
        MeshMaterial3d(mat),
        Transform::default(),
        MuscleMesh,
    ));
}

/// Update the muscle cylinder mesh to stretch between attachment points.
/// Color shifts from blue (relaxed) to red (activated).
fn update_muscle_mesh(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    attachment: Res<MuscleAttachment>,
    mut query: Query<(&mut Transform, &MeshMaterial3d<StandardMaterial>), With<MuscleMesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let (o, ins) = attachment.world_points_bevy(&data);
    let midpoint = (o + ins) * 0.5;
    let diff = ins - o;
    let length = diff.length();

    if length < 1e-6 {
        return;
    }

    let dir = diff / length;

    // Bevy Cylinder is Y-aligned by default. Compute rotation from Y-axis to muscle direction.
    let rotation = Quat::from_rotation_arc(Vec3::Y, dir);

    let muscle_radius = 0.008;

    for (mut transform, mat_handle) in &mut query {
        transform.translation = midpoint;
        transform.rotation = rotation;
        transform.scale = Vec3::new(muscle_radius, length, muscle_radius);

        // Color driven by actual force magnitude (tension), not activation.
        // F0 is the peak isometric force — normalize tension against it.
        let f0 = model.actuator_gainprm[0][2].max(1.0);
        let tension = (data.actuator_force[0].abs() / f0).clamp(0.0, 1.0) as f32;
        if let Some(mat) = materials.get_mut(&mat_handle.0) {
            // Blue (no tension) → Red (peak tension)
            mat.base_color = Color::srgb(0.8 * tension + 0.1, 0.1, 0.8 * (1.0 - tension) + 0.1);
        }
    }
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

    let force = data.actuator_force[0];
    val.max_force_mag = val.max_force_mag.max(force.abs());

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
                    val.initial_angle,
                ),
            },
        ];
        let _ = print_report("Forearm Flexion (t=10s)", &checks);
    }
}
