//! Activation Dynamics — Three Arms, Three Time Constants
//!
//! Three identical forearms driven by muscles with different activation
//! time constants. All receive the same ctrl signal simultaneously.
//! The fast muscle snaps, the medium curls, the slow lags behind.
//! Demonstrates asymmetric activation/deactivation dynamics.
//!
//! Run with: `cargo run -p example-muscle-activation --release`

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

/// Three forearms spaced along the Y axis, each with a different activation
/// time constant. All other parameters identical.
///
/// - Left (Y=-0.4):  Fast   — tau_act=0.005, tau_deact=0.02
/// - Center (Y=0):   Medium — tau_act=0.01,  tau_deact=0.04  (MuJoCo default)
/// - Right (Y=0.4):  Slow   — tau_act=0.05,  tau_deact=0.20
const MJCF: &str = r#"
<mujoco model="activation-dynamics">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <!-- Fast (left) -->
    <body name="upper_fast" pos="0 -0.4 0.8">
      <geom name="upper_fast" type="capsule" size="0.025"
            fromto="0 0 0.15  0 0 0" rgba="0.35 0.35 0.38 1"/>
      <body name="forearm_fast" pos="0 0 0">
        <joint name="elbow_fast" type="hinge" axis="0 1 0"
               range="-1.57 1.57" limited="true" damping="0.3"/>
        <inertial pos="0 0 -0.15" mass="1.0"
                  diaginertia="0.015 0.015 0.001"/>
        <geom name="rod_fast" type="capsule" size="0.02"
              fromto="0 0 0  0 0 -0.3" rgba="0.48 0.48 0.50 1"/>
        <geom name="tip_fast" type="sphere" size="0.04"
              pos="0 0 -0.3" rgba="0.2 0.7 0.3 1"/>
      </body>
    </body>

    <!-- Medium (center) -->
    <body name="upper_med" pos="0 0 0.8">
      <geom name="upper_med" type="capsule" size="0.025"
            fromto="0 0 0.15  0 0 0" rgba="0.35 0.35 0.38 1"/>
      <body name="forearm_med" pos="0 0 0">
        <joint name="elbow_med" type="hinge" axis="0 1 0"
               range="-1.57 1.57" limited="true" damping="0.3"/>
        <inertial pos="0 0 -0.15" mass="1.0"
                  diaginertia="0.015 0.015 0.001"/>
        <geom name="rod_med" type="capsule" size="0.02"
              fromto="0 0 0  0 0 -0.3" rgba="0.48 0.48 0.50 1"/>
        <geom name="tip_med" type="sphere" size="0.04"
              pos="0 0 -0.3" rgba="0.85 0.6 0.15 1"/>
      </body>
    </body>

    <!-- Slow (right) -->
    <body name="upper_slow" pos="0 0.4 0.8">
      <geom name="upper_slow" type="capsule" size="0.025"
            fromto="0 0 0.15  0 0 0" rgba="0.35 0.35 0.38 1"/>
      <body name="forearm_slow" pos="0 0 0">
        <joint name="elbow_slow" type="hinge" axis="0 1 0"
               range="-1.57 1.57" limited="true" damping="0.3"/>
        <inertial pos="0 0 -0.15" mass="1.0"
                  diaginertia="0.015 0.015 0.001"/>
        <geom name="rod_slow" type="capsule" size="0.02"
              fromto="0 0 0  0 0 -0.3" rgba="0.48 0.48 0.50 1"/>
        <geom name="tip_slow" type="sphere" size="0.04"
              pos="0 0 -0.3" rgba="0.85 0.3 0.2 1"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <muscle name="fast" joint="elbow_fast"
            scale="200" range="0.75 1.05"
            lmin="0.5" lmax="1.6" vmax="1.0"
            fpmax="0.3" fvmax="1.4"
            timeconst="0.005 0.02"/>
    <muscle name="medium" joint="elbow_med"
            scale="200" range="0.75 1.05"
            lmin="0.5" lmax="1.6" vmax="1.0"
            fpmax="0.3" fvmax="1.4"
            timeconst="0.01 0.04"/>
    <muscle name="slow" joint="elbow_slow"
            scale="200" range="0.75 1.05"
            lmin="0.5" lmax="1.6" vmax="1.0"
            fpmax="0.3" fvmax="1.4"
            timeconst="0.05 0.20"/>
  </actuator>

  <sensor>
    <jointpos name="jpos_fast" joint="elbow_fast"/>
    <jointpos name="jpos_med" joint="elbow_med"/>
    <jointpos name="jpos_slow" joint="elbow_slow"/>
  </sensor>
</mujoco>
"#;

const CTRL_ONSET: f64 = 0.5;
const CTRL_RELEASE: f64 = 4.0;
const NUM_MUSCLES: usize = 3;

// ── Muscle attachment ─────────────────────────────────────────────────────

/// Per-muscle attachment data derived from body transforms.
struct SingleAttachment {
    parent_body: usize,
    child_body: usize,
    parent_local: [f64; 3],
    child_local: [f64; 3],
}

impl SingleAttachment {
    fn from_actuator(model: &sim_core::Model, actuator_idx: usize) -> Self {
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

        Self {
            parent_body,
            child_body,
            parent_local: [parent_radius, 0.0, 0.10],
            child_local: [child_radius, 0.0, -0.10],
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
    for i in 0..NUM_MUSCLES {
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
    println!("=== CortenForge: Activation Dynamics ===");
    println!("  Three muscles, three time constants");
    println!("  Left=fast (5/20ms), Center=medium (10/40ms), Right=slow (50/200ms)");
    println!("  Ctrl on at t={CTRL_ONSET:.1}s, off at t={CTRL_RELEASE:.1}s");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Activation Dynamics".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<ActivationValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(8.0)
                .print_every(1.0)
                .display(|m, d| {
                    let a0 = if m.na > 0 {
                        d.act[m.actuator_act_adr[0]]
                    } else {
                        0.0
                    };
                    let a1 = if m.na > 1 {
                        d.act[m.actuator_act_adr[1]]
                    } else {
                        0.0
                    };
                    let a2 = if m.na > 2 {
                        d.act[m.actuator_act_adr[2]]
                    } else {
                        0.0
                    };
                    format!("act: fast={a0:.2}  med={a1:.2}  slow={a2:.2}")
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
                activation_diagnostics,
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
        let tau_a = model.actuator_dynprm[i][0];
        let tau_d = model.actuator_dynprm[i][1];
        println!("  actuator[{i}] '{name}': tau_act={tau_a:.3}, tau_deact={tau_d:.3}");
    }
    println!();

    // Build attachments for all three muscles
    let attachments: Vec<SingleAttachment> = (0..model.nu)
        .map(|i| SingleAttachment::from_actuator(&model, i))
        .collect();

    let mat_upper = materials.add(MetalPreset::CastIron.material());
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip_fast =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.7, 0.3)));
    let mat_tip_med =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.6, 0.15)));
    let mat_tip_slow =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.3, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("upper_fast", mat_upper.clone()),
            ("upper_med", mat_upper.clone()),
            ("upper_slow", mat_upper),
            ("rod_fast", mat_rod.clone()),
            ("rod_med", mat_rod.clone()),
            ("rod_slow", mat_rod),
            ("tip_fast", mat_tip_fast),
            ("tip_med", mat_tip_med),
            ("tip_slow", mat_tip_slow),
        ],
    );

    spawn_muscle_meshes(&mut commands, &mut meshes, &mut materials);

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.65, 0.0),
        2.5,
        std::f32::consts::FRAC_PI_4, // 45° — see all three arms in perspective
        0.15,                        // slight elevation
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(MuscleAttachments(attachments));
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Control ─────────────────────────────────────────────────────────────────

fn apply_ctrl(mut data: ResMut<PhysicsData>, model: Res<PhysicsModel>) {
    let ctrl = if data.time >= CTRL_ONSET && data.time < CTRL_RELEASE {
        1.0
    } else {
        0.0
    };
    for i in 0..model.nu {
        data.set_ctrl(i, ctrl);
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Activation Dynamics");

    let labels = ["fast", "medium", "slow"];
    for (i, label) in labels.iter().enumerate().take(model.nu.min(NUM_MUSCLES)) {
        let act = data.act[model.actuator_act_adr[i]];
        let force = data.actuator_force[i];
        let angle = data
            .sensor_scalar(&model, &format!("jpos_{label}"))
            .unwrap_or(0.0);
        hud.scalar(&format!("{label} act"), act, 3);
        hud.scalar(&format!("{label} force"), force, 1);
        hud.scalar(&format!("{label} angle"), angle, 2);
    }
    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct ActivationValidation {
    /// Time each muscle first reached act > 0.9
    rise_time: [f64; NUM_MUSCLES],
    rise_found: [bool; NUM_MUSCLES],
    reported: bool,
}

fn activation_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<ActivationValidation>,
) {
    let time = data.time;

    // Track time to reach 90% activation
    for i in 0..model.nu.min(NUM_MUSCLES) {
        let act = data.act[model.actuator_act_adr[i]];
        if act > 0.9 && !val.rise_found[i] {
            val.rise_time[i] = time - CTRL_ONSET;
            val.rise_found[i] = true;
        }
    }

    if harness.reported() && !val.reported {
        val.reported = true;

        let checks = vec![
            Check {
                name: "Fast rises before medium",
                pass: val.rise_found[0] && val.rise_found[1] && val.rise_time[0] < val.rise_time[1],
                detail: format!(
                    "fast={:.3}s, med={:.3}s",
                    val.rise_time[0], val.rise_time[1]
                ),
            },
            Check {
                name: "Medium rises before slow",
                pass: val.rise_found[1] && val.rise_found[2] && val.rise_time[1] < val.rise_time[2],
                detail: format!(
                    "med={:.3}s, slow={:.3}s",
                    val.rise_time[1], val.rise_time[2]
                ),
            },
            Check {
                name: "Slow > 5x fast rise time",
                pass: val.rise_found[0]
                    && val.rise_found[2]
                    && val.rise_time[2] > 5.0 * val.rise_time[0],
                detail: format!(
                    "slow/fast = {:.1}x (need >5x)",
                    val.rise_time[2] / val.rise_time[0].max(1e-6)
                ),
            },
            Check {
                name: "All three reached 90%",
                pass: val.rise_found[0] && val.rise_found[1] && val.rise_found[2],
                detail: format!(
                    "fast={}, med={}, slow={}",
                    val.rise_found[0], val.rise_found[1], val.rise_found[2]
                ),
            },
        ];
        let _ = print_report("Activation Dynamics (t=8s)", &checks);
    }
}
