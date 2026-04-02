//! Fixed Coupling — Linear Joint Coupling via Tendon
//!
//! A fixed tendon with coefficients [1.0, −0.5] couples two hinge joints.
//! Motor drives joint A sinusoidally; joint B follows via a soft tendon spring.
//! HUD verifies TendonPos matches 1.0*q_A − 0.5*q_B every frame.
//!
//! Run with: `cargo run -p example-tendon-fixed-coupling --release`

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
use sim_bevy::convert::{physics_pos, vec3_from_vector};
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    tendon_color_ramp, validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};

// ── MJCF Model ────────────────────────────────────────────────────────────

/// Two pendulums coupled by a fixed tendon: L = 1.0*q_A − 0.5*q_B.
/// Stiffness=100 keeps the coupling tight (B clearly follows A).
/// Joint damping=1.0 suppresses transients for a clean, smooth swing.
/// Motor on joint A drives it sinusoidally; joint B follows passively.
const MJCF: &str = r#"
<mujoco model="fixed-coupling">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <compiler angle="radian"/>

  <worldbody>
    <!-- Pendulum A (blue, motor-driven) -->
    <body name="pivot_a" pos="-0.4 0 0.8">
      <geom name="pivot_a_geom" type="sphere" size="0.02"
            rgba="0.4 0.4 0.4 1" contype="0" conaffinity="0"/>
      <body name="arm_a" pos="0 0 0">
        <joint name="j_a" type="hinge" axis="0 1 0" damping="1.0"/>
        <geom name="rod_a" type="capsule" fromto="0 0 0 0 0 -0.4"
              size="0.02" mass="0.5" rgba="0.3 0.5 0.85 1"/>
        <body name="tip_a" pos="0 0 -0.4">
          <geom name="tip_a_geom" type="sphere" size="0.035" mass="0.2"
                rgba="0.3 0.5 0.85 1"/>
          <site name="attach_a" pos="0 0 0" size="0.005"/>
        </body>
      </body>
    </body>

    <!-- Pendulum B (red, passive) -->
    <body name="pivot_b" pos="0.4 0 0.8">
      <geom name="pivot_b_geom" type="sphere" size="0.02"
            rgba="0.4 0.4 0.4 1" contype="0" conaffinity="0"/>
      <body name="arm_b" pos="0 0 0">
        <joint name="j_b" type="hinge" axis="0 1 0" damping="1.0"/>
        <geom name="rod_b" type="capsule" fromto="0 0 0 0 0 -0.4"
              size="0.02" mass="0.5" rgba="0.85 0.25 0.2 1"/>
        <body name="tip_b" pos="0 0 -0.4">
          <geom name="tip_b_geom" type="sphere" size="0.035" mass="0.2"
                rgba="0.85 0.25 0.2 1"/>
          <site name="attach_b" pos="0 0 0" size="0.005"/>
        </body>
      </body>
    </body>
  </worldbody>

  <tendon>
    <fixed name="coupling" stiffness="100" damping="3">
      <joint joint="j_a" coef="1.0"/>
      <joint joint="j_b" coef="-0.5"/>
    </fixed>
  </tendon>

  <actuator>
    <motor name="drive_a" joint="j_a" gear="1"/>
  </actuator>

  <sensor>
    <tendonpos name="ten_pos" tendon="coupling"/>
    <tendonvel name="ten_vel" tendon="coupling"/>
    <jointpos name="angle_a" joint="j_a"/>
    <jointpos name="angle_b" joint="j_b"/>
  </sensor>
</mujoco>
"#;

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Fixed Coupling — Tendon ===");
    println!("  Blue (A): motor-driven, coef = +1.0");
    println!("  Red  (B): passive, coef = −0.5");
    println!("  Tendon: L = 1.0·q_A − 0.5·q_B, stiffness=100, damping=3");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge \u{2014} Fixed Coupling (Tendons)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(10.0)
                .print_every(1.0)
                .display(|m, d| {
                    let ja = m.joint_id("j_a").unwrap();
                    let jb = m.joint_id("j_b").unwrap();
                    let qa = d.qpos[m.jnt_qpos_adr[ja]].to_degrees();
                    let qb = d.qpos[m.jnt_qpos_adr[jb]].to_degrees();
                    format!("A={qa:+6.1}\u{00b0}  B={qb:+6.1}\u{00b0}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_control, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                diagnostics,
                update_hud,
                render_physics_hud,
                draw_tendon_force,
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
    let mat_red =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.25, 0.2)));
    let mat_pivot =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.4, 0.4, 0.4)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("pivot_a_geom", mat_pivot.clone()),
            ("rod_a", mat_blue.clone()),
            ("tip_a_geom", mat_blue),
            ("pivot_b_geom", mat_pivot),
            ("rod_b", mat_red.clone()),
            ("tip_b_geom", mat_red),
        ],
    );

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 0.5),
        2.5,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Tendon Visualization ──────────────────────────────────────────────────

/// Self-calibrating length range for color mapping.
#[derive(Default)]
struct LenRange {
    min: f64,
    max: f64,
    init: bool,
}

/// Draw the tendon as a line between the two attachment sites, colored by length.
/// Green (shortest observed) → yellow → red (longest observed).
fn draw_tendon_force(
    mut gizmos: Gizmos,
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut range: Local<LenRange>,
) {
    let sa = model.site_id("attach_a").expect("attach_a");
    let sb = model.site_id("attach_b").expect("attach_b");
    let start = vec3_from_vector(&data.site_xpos[sa]);
    let end = vec3_from_vector(&data.site_xpos[sb]);

    // Use the visual (Euclidean) distance between sites for color,
    // not ten_length (which is the algebraic coupling sum and stays near 0).
    let length = (data.site_xpos[sa] - data.site_xpos[sb]).norm();
    if !range.init {
        range.min = length;
        range.max = length;
        range.init = true;
    }
    range.min = range.min.min(length);
    range.max = range.max.max(length);

    let span = range.max - range.min;
    let t = if span > 1e-10 {
        ((length - range.min) / span) as f32
    } else {
        0.0
    };
    let color = tendon_color_ramp(t);

    gizmos.line(start, end, color);

    // Small spheres at attachment points
    let radius = 0.008;
    gizmos.sphere(Isometry3d::from_translation(start), radius, color);
    gizmos.sphere(Isometry3d::from_translation(end), radius, color);
}

// ── Control ───────────────────────────────────────────────────────────────

/// Sinusoidal motor on joint A: 0.5 Hz, ±2 N·m.
fn apply_control(model: Res<PhysicsModel>, mut data: ResMut<PhysicsData>) {
    let t = data.time;
    let freq = 0.5;
    data.ctrl[0] = 8.0 * (2.0 * std::f64::consts::PI * freq * t).sin();
    let _ = &model; // used for type inference
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Fixed Coupling \u{2014} L = 1.0\u{00b7}q\u{2090} \u{2212} 0.5\u{00b7}q\u{1d47}");

    let ja = model.joint_id("j_a").expect("j_a");
    let jb = model.joint_id("j_b").expect("j_b");
    let qa = data.qpos[model.jnt_qpos_adr[ja]];
    let qb = data.qpos[model.jnt_qpos_adr[jb]];

    let sensor_l = data.sensor_scalar(&model, "ten_pos").unwrap_or(0.0);
    let manual_l = 1.0 * qa + (-0.5) * qb;
    let match_ok = (sensor_l - manual_l).abs() < 1e-10;

    hud.raw(format!("Joint A (blue):  {:+7.2}\u{00b0}", qa.to_degrees()));
    hud.raw(format!("Joint B (red):   {:+7.2}\u{00b0}", qb.to_degrees()));
    let force = -100.0 * sensor_l - 3.0 * data.ten_velocity[0];
    hud.raw(format!("Tendon L:        {sensor_l:+.6}"));
    hud.raw(format!("Manual L:        {manual_l:+.6}"));
    hud.raw(format!(
        "Match:           {}",
        if match_ok { "\u{2713}" } else { "\u{2717}" }
    ));
    hud.raw(format!("Tendon force:    {force:+.2} N"));
    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Default)]
struct DiagState {
    max_err: f64,
    b_moved: bool,
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut state: Local<DiagState>,
) {
    let ja = model.joint_id("j_a").expect("j_a");
    let jb = model.joint_id("j_b").expect("j_b");
    let qa = data.qpos[model.jnt_qpos_adr[ja]];
    let qb = data.qpos[model.jnt_qpos_adr[jb]];

    let sensor_l = data.sensor_scalar(&model, "ten_pos").unwrap_or(0.0);
    let manual_l = 1.0 * qa + (-0.5) * qb;
    let err = (sensor_l - manual_l).abs();
    state.max_err = state.max_err.max(err);

    if qb.abs() > 0.01 {
        state.b_moved = true;
    }

    if !harness.reported() || state.reported {
        return;
    }
    state.reported = true;

    let checks = vec![
        Check {
            name: "TendonPos = 1.0*q_A - 0.5*q_B (every frame)",
            pass: state.max_err < 1e-10,
            detail: format!("max |error| = {:.2e}", state.max_err),
        },
        Check {
            name: "Joint B moved (coupling active)",
            pass: state.b_moved,
            detail: format!("q_B = {qb:.4} rad"),
        },
    ];
    let _ = print_report("Fixed Coupling (t=10s)", &checks);
}
