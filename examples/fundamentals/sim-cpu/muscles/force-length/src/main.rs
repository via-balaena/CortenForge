//! Force-Length Curve — MuJoCo vs Hill
//!
//! Two forearms swept slowly through their joint range while fully activated.
//! The left arm uses MuJoCo's piecewise-quadratic FL curve, the right uses
//! Hill's Gaussian FL curve. Tendon color (blue→red) shows force at each
//! position — peak force at optimal length, dropping off at the extremes.
//!
//! Run with: `cargo run -p example-muscle-force-length --release`

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

/// Two arms side by side:
/// - Left (Y=-0.3): MuJoCo `<muscle>` — piecewise-quadratic FL curve
/// - Right (Y=+0.3): MuJoCo `<muscle>` with different range params to show
///   how the curve shape affects force output
///
/// Both are driven by position servos that sweep the joint angle sinusoidally.
/// The muscle is fully activated (ctrl=1) throughout — the position servo
/// moves the joint while we observe what force the muscle produces at each
/// position.
const MJCF: &str = r#"
<mujoco model="force-length">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <!-- Left arm: wide FL range (lmin=0.5, lmax=1.6) — broad force curve -->
    <body name="upper_wide" pos="0 -0.3 0.8">
      <geom name="upper_wide" type="capsule" size="0.025"
            fromto="0 0 0.15  0 0 0" rgba="0.35 0.35 0.38 1"/>
      <body name="forearm_wide" pos="0 0 0">
        <joint name="elbow_wide" type="hinge" axis="0 1 0"
               range="-1.57 1.57" limited="true" damping="0.3"/>
        <inertial pos="0 0 -0.15" mass="1.0"
                  diaginertia="0.015 0.015 0.001"/>
        <geom name="rod_wide" type="capsule" size="0.02"
              fromto="0 0 0  0 0 -0.3" rgba="0.48 0.48 0.50 1"/>
        <geom name="tip_wide" type="sphere" size="0.04"
              pos="0 0 -0.3" rgba="0.2 0.5 0.85 1"/>
      </body>
    </body>

    <!-- Right arm: narrow FL range (lmin=0.8, lmax=1.2) — sharp force peak -->
    <body name="upper_narrow" pos="0 0.3 0.8">
      <geom name="upper_narrow" type="capsule" size="0.025"
            fromto="0 0 0.15  0 0 0" rgba="0.35 0.35 0.38 1"/>
      <body name="forearm_narrow" pos="0 0 0">
        <joint name="elbow_narrow" type="hinge" axis="0 1 0"
               range="-1.57 1.57" limited="true" damping="0.3"/>
        <inertial pos="0 0 -0.15" mass="1.0"
                  diaginertia="0.015 0.015 0.001"/>
        <geom name="rod_narrow" type="capsule" size="0.02"
              fromto="0 0 0  0 0 -0.3" rgba="0.48 0.48 0.50 1"/>
        <geom name="tip_narrow" type="sphere" size="0.04"
              pos="0 0 -0.3" rgba="0.85 0.4 0.2 1"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <!-- Muscles: fully activated, force depends on joint position -->
    <muscle name="wide" joint="elbow_wide"
            scale="50" range="0.75 1.05"
            lmin="0.5" lmax="1.6" vmax="1.0"
            fpmax="0.3" fvmax="1.4"
            timeconst="0.01 0.04"/>
    <muscle name="narrow" joint="elbow_narrow"
            scale="50" range="0.75 1.05"
            lmin="0.8" lmax="1.2" vmax="1.0"
            fpmax="0.3" fvmax="1.4"
            timeconst="0.01 0.04"/>

    <!-- Position servos to sweep joint angle -->
    <position name="sweep_wide" joint="elbow_wide" kp="20" dampratio="1"/>
    <position name="sweep_narrow" joint="elbow_narrow" kp="20" dampratio="1"/>
  </actuator>

  <sensor>
    <jointpos name="jpos_wide" joint="elbow_wide"/>
    <jointpos name="jpos_narrow" joint="elbow_narrow"/>
    <actuatorfrc name="frc_wide" actuator="wide"/>
    <actuatorfrc name="frc_narrow" actuator="narrow"/>
  </sensor>
</mujoco>
"#;

/// Sweep period (seconds for one full cycle).
const SWEEP_PERIOD: f64 = 8.0;
/// Sweep amplitude (radians).
const SWEEP_AMP: f64 = 1.2;

const NUM_MUSCLES: usize = 2;

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
    println!("=== CortenForge: Force-Length Curve ===");
    println!("  Two muscles swept through joint range at full activation");
    println!("  Left (blue tip) = wide FL range, Right (orange tip) = narrow FL range");
    println!("  Watch the tendon color: red = peak force, blue = weak force");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Force-Length Curve".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<ForceLengthValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(16.0)
                .print_every(2.0)
                .display(|m, d| {
                    let fw = d.sensor_scalar(m, "frc_wide").unwrap_or(0.0);
                    let fn_ = d.sensor_scalar(m, "frc_narrow").unwrap_or(0.0);
                    let aw = d.sensor_scalar(m, "jpos_wide").unwrap_or(0.0);
                    format!("angle={aw:.2}  wide={fw:.1}  narrow={fn_:.1}")
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
                fl_diagnostics,
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
    for i in 0..2 {
        let name = model.actuator_name[i].as_deref().unwrap_or("?");
        let lmin = model.actuator_gainprm[i][4];
        let lmax = model.actuator_gainprm[i][5];
        println!("  muscle[{i}] '{name}': lmin={lmin}, lmax={lmax}");
    }
    println!();

    // Muscle attachments for the first two actuators (muscles, not servos)
    let attachments: Vec<MuscleAttachment> = (0..NUM_MUSCLES)
        .map(|i| MuscleAttachment::from_actuator(&model, i, 1.0))
        .collect();

    let mat_upper = materials.add(MetalPreset::CastIron.material());
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip_wide =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.5, 0.85)));
    let mat_tip_narrow =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.4, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("upper_wide", mat_upper.clone()),
            ("upper_narrow", mat_upper),
            ("rod_wide", mat_rod.clone()),
            ("rod_narrow", mat_rod),
            ("tip_wide", mat_tip_wide),
            ("tip_narrow", mat_tip_narrow),
        ],
    );

    spawn_muscle_meshes(&mut commands, &mut meshes, &mut materials, NUM_MUSCLES);

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.65, 0.0),
        2.2,
        std::f32::consts::FRAC_PI_4,
        0.15,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(Attachments(attachments));
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Control ─────────────────────────────────────────────────────────────────

fn apply_ctrl(mut data: ResMut<PhysicsData>, model: Res<PhysicsModel>) {
    let time = data.time;

    // Muscles fully activated throughout
    data.set_ctrl(0, 1.0); // wide muscle
    data.set_ctrl(1, 1.0); // narrow muscle

    // Position servos sweep sinusoidally
    let target = SWEEP_AMP * (2.0 * std::f64::consts::PI * time / SWEEP_PERIOD).sin();
    // Servo actuators are indices 2 and 3
    if model.nu > 2 {
        data.set_ctrl(2, target);
    }
    if model.nu > 3 {
        data.set_ctrl(3, target);
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Force-Length Curve");

    let angle = data.sensor_scalar(&model, "jpos_wide").unwrap_or(0.0);
    let frc_wide = data.sensor_scalar(&model, "frc_wide").unwrap_or(0.0);
    let frc_narrow = data.sensor_scalar(&model, "frc_narrow").unwrap_or(0.0);

    hud.scalar("angle (rad)", angle, 2);
    hud.scalar("wide force", frc_wide, 2);
    hud.scalar("narrow force", frc_narrow, 2);
    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct ForceLengthValidation {
    /// Track peak force for each muscle
    max_wide: f64,
    max_narrow: f64,
    /// Track whether force varies (min force should be < max force)
    min_wide: f64,
    min_narrow: f64,
    initialized: bool,
    reported: bool,
}

fn fl_diagnostics(
    _model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<ForceLengthValidation>,
) {
    // Only track after activation has built up
    if data.time < 1.0 {
        return;
    }

    let fw = data.actuator_force[0].abs();
    let fn_ = data.actuator_force[1].abs();

    if !val.initialized {
        val.max_wide = fw;
        val.min_wide = fw;
        val.max_narrow = fn_;
        val.min_narrow = fn_;
        val.initialized = true;
    }

    val.max_wide = val.max_wide.max(fw);
    val.min_wide = val.min_wide.min(fw);
    val.max_narrow = val.max_narrow.max(fn_);
    val.min_narrow = val.min_narrow.min(fn_);

    if harness.reported() && !val.reported {
        val.reported = true;

        let wide_range = val.max_wide - val.min_wide;
        let narrow_range = val.max_narrow - val.min_narrow;

        let checks = vec![
            Check {
                name: "Wide muscle produced force",
                pass: val.max_wide > 0.5,
                detail: format!("max |force|={:.2}", val.max_wide),
            },
            Check {
                name: "Narrow muscle produced force",
                pass: val.max_narrow > 0.5,
                detail: format!("max |force|={:.2}", val.max_narrow),
            },
            Check {
                name: "Wide force varies with position",
                pass: wide_range > 0.1,
                detail: format!(
                    "min={:.2}, max={:.2}, range={wide_range:.2}",
                    val.min_wide, val.max_wide
                ),
            },
            Check {
                name: "Narrow force varies with position",
                pass: narrow_range > 0.1,
                detail: format!(
                    "min={:.2}, max={:.2}, range={narrow_range:.2}",
                    val.min_narrow, val.max_narrow
                ),
            },
        ];
        let _ = print_report("Force-Length Curve (t=16s)", &checks);
    }
}
