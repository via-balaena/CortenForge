//! Slide Limits — Motor Force vs Limit Constraint
//!
//! A mass on a horizontal rail with range="-0.3 0.3" pushed by a constant
//! motor force into the positive limit. Shows that the limit holds and the
//! JointLimitFrc sensor reports the balancing reaction force.
//!
//! Run with: `cargo run -p example-joint-limits-slide-limits --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::cast_sign_loss,
    clippy::similar_names
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::convert::physics_pos;
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

// ── MJCF Model ────────────────────────────────────────────────────────────

/// Slider on a rail: slide joint along X, range ±0.3m, motor pushing +X.
/// Zero gravity so the only forces are motor and limit constraint.
/// Rail visual: capsule along X-axis. Slider: box.
const MJCF: &str = r#"
<mujoco model="slide-limits">
  <option gravity="0 0 0" timestep="0.002"/>

  <worldbody>
    <!-- Rail visual (non-colliding) -->
    <geom name="rail" type="capsule" fromto="-0.5 0 0.3 0.5 0 0.3"
          size="0.008" rgba="0.5 0.5 0.5 0.6" contype="0" conaffinity="0"/>

    <!-- Left endstop visual -->
    <geom name="stop_lo" type="sphere" size="0.02"
          pos="-0.3 0 0.3" rgba="0.6 0.6 0.6 0.8" contype="0" conaffinity="0"/>

    <!-- Right endstop visual -->
    <geom name="stop_hi" type="sphere" size="0.02"
          pos="0.3 0 0.3" rgba="0.6 0.6 0.6 0.8" contype="0" conaffinity="0"/>

    <body name="slider" pos="0 0 0.3">
      <joint name="slide" type="slide" axis="1 0 0"
             limited="true" range="-0.3 0.3" damping="2.0"/>
      <geom name="slider_geom" type="box" size="0.04 0.04 0.04" mass="1.0"
            rgba="0.3 0.5 0.85 1"/>
    </body>
  </worldbody>

  <actuator>
    <motor name="push" joint="slide" gear="1"/>
  </actuator>

  <sensor>
    <jointlimitfrc name="slide_lf" joint="slide"/>
  </sensor>
</mujoco>
"#;

const MOTOR_FORCE: f64 = 5.0;

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Slide Limits — Motor vs Limit ===");
    println!("  Blue box on a rail, motor pushes right (force = {MOTOR_FORCE})");
    println!("  Range: -0.3 .. 0.3 m");
    println!("  Limit holds the box at +0.3 m");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Slide Limits (Joint Limits)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(5.0)
                .print_every(0.5)
                .display(|m, d| {
                    let jid = m.joint_id("slide").unwrap();
                    let q = d.qpos[m.jnt_qpos_adr[jid]];
                    let frc = d.jnt_limit_frc[jid];
                    format!("pos={q:.3} limit_frc={frc:.2}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_motor, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

// ── Setup ─────────────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let data = model.make_data();

    let mat_blue =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.5, 0.85)));
    let mat_rail = materials.add(StandardMaterial {
        base_color: Color::srgba(0.5, 0.5, 0.5, 0.6),
        alpha_mode: AlphaMode::Blend,
        metallic: 0.3,
        perceptual_roughness: 0.6,
        ..default()
    });
    let mat_stop = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.6, 0.6, 0.6)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("rail", mat_rail),
            ("stop_lo", mat_stop.clone()),
            ("stop_hi", mat_stop),
            ("slider_geom", mat_blue),
        ],
    );

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 0.3),
        1.5,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Motor ─────────────────────────────────────────────────────────────────

fn apply_motor(mut data: ResMut<PhysicsData>) {
    data.ctrl[0] = MOTOR_FORCE;
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Joint Limits \u{2014} Slide Limit + Motor");

    let jid = model.joint_id("slide").expect("slide");
    let q = data.qpos[model.jnt_qpos_adr[jid]];
    let frc = data.jnt_limit_frc[jid];
    let at_limit = q > 0.29;

    hud.raw(format!("position= {q:7.3}  (range: -0.30 .. 0.30)"));
    hud.raw(format!("motor_force= {MOTOR_FORCE:5.1}"));
    hud.raw(format!("limit_force= {frc:7.2}"));
    hud.raw(format!("at_limit= {}", if at_limit { "YES" } else { "no" }));
    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Default)]
struct DiagState {
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut state: Local<DiagState>,
) {
    if !harness.reported() || state.reported {
        return;
    }
    state.reported = true;

    let jid = model.joint_id("slide").expect("slide");
    let q = data.qpos[model.jnt_qpos_adr[jid]];
    let frc = data.jnt_limit_frc[jid];
    let limit = 0.3;

    let checks = vec![
        Check {
            name: "Position clamped at limit",
            pass: q <= limit + 0.01,
            detail: format!("pos = {q:.4} (limit = {limit})"),
        },
        Check {
            name: "Limit force approx equals motor force",
            pass: (frc - MOTOR_FORCE).abs() < MOTOR_FORCE * 0.3,
            detail: format!("limit_frc = {frc:.2}, motor = {MOTOR_FORCE}"),
        },
        Check {
            name: "Limit force > 0",
            pass: frc > 0.0,
            detail: format!("limit_frc = {frc:.4}"),
        },
    ];
    let _ = print_report("Slide Limits (t=5s)", &checks);
}
