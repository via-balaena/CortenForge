//! Wind — Global Wind Field and Body Interaction
//!
//! Two demonstrations of wind:
//! 1. A free sphere drifts sideways as it falls — parabolic trajectory.
//! 2. A pendulum deflects from vertical under steady wind load.
//!
//! Wind subtracts from body velocity before computing drag:
//!   effective_velocity = body_velocity - wind
//! A stationary body in wind experiences the same drag as a body moving
//! at -wind in still air.
//!
//! A 3-second calm period lets you see the initial state before wind + gravity
//! switch on simultaneously.
//!
//! Run: `cargo run -p example-passive-wind --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::needless_range_loop,
    clippy::needless_pass_by_value,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::let_underscore_must_use,
    clippy::unwrap_used
)]

use bevy::prelude::*;
use nalgebra::Vector3;
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

// ── MJCF ──────────────────────────────────────────────────────────────────

/// Starts with zero gravity and zero wind. At t=3s, gravity and wind switch
/// on simultaneously so you can see the initial state first.
///
/// Drifter: free sphere (r=0.15, m=0.5 kg) starting at rest.
///   Wind pushes it sideways while gravity pulls it down → parabolic path.
///
/// Pendulum: hinge joint (Y-axis) with capsule rod + sphere bob.
///   Light mass (rod 0.3 kg, bob 0.5 kg) so wind can deflect it clearly.
const MJCF: &str = r#"
<mujoco model="wind">
  <option gravity="0 0 0" timestep="0.002"
          density="10" viscosity="0.01" wind="0 0 0"/>
  <worldbody>
    <!-- Drifter: free sphere that falls + drifts in wind -->
    <body name="drifter" pos="-2 0 3">
      <freejoint name="drift_free"/>
      <geom name="drift_geom" type="sphere" size="0.15" mass="0.5"
            rgba="0.3 0.6 1.0 1"/>
    </body>

    <!-- Pendulum: hinge at pivot, rod + bob deflects in wind -->
    <body name="arm" pos="2 0 2">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.3"/>
      <geom name="rod_geom" type="capsule" fromto="0 0 0 0 0 -1.2"
            size="0.03" mass="0.3" rgba="0.6 0.6 0.6 1"/>
      <body name="bob" pos="0 0 -1.2">
        <geom name="bob_geom" type="sphere" size="0.12" mass="0.5"
              rgba="0.9 0.5 0.2 1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"#;

const DELAY_GRAVITY: f64 = 3.0;
const DELAY_WIND: f64 = 3.25;
const WIND_OFF: f64 = 6.0;
const REPORT_TIME: f64 = 13.0;

// ── Bevy app ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Wind — Global Wind Field ===");
    println!("  3-second calm, then gravity + wind switch on.");
    println!("  Free sphere drifts sideways. Pendulum deflects from vertical.\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Wind — Global Wind Field".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<WindValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(REPORT_TIME)
                .print_every(1.0)
                .display(|m, d| {
                    let t = d.time;
                    let dof_d = m.body_dof_adr[m.body_id("drifter").unwrap()];
                    let vx = d.qvel[dof_d];
                    let vz = -d.qvel[dof_d + 2];
                    let jid = m.joint_id("hinge").unwrap();
                    let qadr = m.jnt_qpos_adr[jid];
                    let angle_deg = d.qpos[qadr].to_degrees();
                    format!("t={t:.1}s  drift vx={vx:.3} vz={vz:.3}  pend={angle_deg:.1} deg")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (activate_wind, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                wind_diagnostics,
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

    let mat_drift =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.6, 1.0)));
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_bob = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.5, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("drift_geom", mat_drift),
            ("rod_geom", mat_rod),
            ("bob_geom", mat_bob),
        ],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.0),
        14.0,
        std::f32::consts::FRAC_PI_4,
        0.3,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Wind activation after delay ───────────────────────────────────────────

fn activate_wind(mut model: ResMut<PhysicsModel>, data: Res<PhysicsData>) {
    let t = data.0.time;
    if t >= DELAY_GRAVITY && model.0.gravity.norm() == 0.0 {
        model.0.gravity = Vector3::new(0.0, 0.0, -9.81);
    }
    if (DELAY_WIND..WIND_OFF).contains(&t) && model.0.wind.norm() == 0.0 {
        model.0.wind = Vector3::new(5.0, 0.0, 0.0);
    }
    if t >= WIND_OFF && model.0.wind.norm() > 0.0 {
        model.0.wind = Vector3::zeros();
    }
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();

    let m = &model.0;
    let d = &data.0;

    hud.section("Drifter (free sphere)");
    let dof_d = m.body_dof_adr[m.body_id("drifter").expect("drifter")];
    hud.scalar("vx (m/s)", d.qvel[dof_d], 3);
    hud.scalar("vz (m/s)", -d.qvel[dof_d + 2], 3);

    hud.raw(String::new());

    let jid = m.joint_id("hinge").expect("hinge");
    let qadr = m.jnt_qpos_adr[jid];
    let angle_deg = d.qpos[qadr].to_degrees();

    hud.section("Pendulum");
    hud.scalar("angle (deg)", angle_deg, 1);

    hud.raw(String::new());
    hud.section("Wind");
    if d.time < DELAY_WIND {
        hud.raw(format!("  calm ({:.1}s to wind)", DELAY_WIND - d.time));
    } else if d.time < WIND_OFF {
        hud.raw(format!(
            "  [{:.1}, {:.1}, {:.1}] m/s  (off at {WIND_OFF:.0}s)",
            m.wind.x, m.wind.y, m.wind.z
        ));
    } else {
        hud.raw("  off".to_string());
    }
    hud.scalar("density (kg/m3)", m.density, 0);
    hud.scalar("time (s)", d.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct WindValidation {
    /// Peak deflection angle during wind-on period
    peak_angle_deg: f64,
    /// Max drifter x-velocity observed
    max_vx: f64,
    reported: bool,
}

fn wind_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<WindValidation>,
) {
    let m = &model.0;
    let d = &data.0;

    if d.time < DELAY_WIND {
        return;
    }

    // Track peak values during wind-on period
    let dof_d = m.body_dof_adr[m.body_id("drifter").expect("drifter")];
    let vx = d.qvel[dof_d];
    val.max_vx = val.max_vx.max(vx);

    let jid = m.joint_id("hinge").expect("hinge");
    let qadr = m.jnt_qpos_adr[jid];
    let angle_deg = d.qpos[qadr].to_degrees().abs();
    val.peak_angle_deg = val.peak_angle_deg.max(angle_deg);

    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    // After wind off, pendulum swings back — check it returned toward vertical
    let final_angle = d.qpos[qadr].to_degrees().abs();

    let checks = vec![
        Check {
            name: "Drifter gained x-velocity from wind",
            pass: val.max_vx > 0.5,
            detail: format!("max vx={:.3} m/s (need > 0.5)", val.max_vx),
        },
        Check {
            name: "Pendulum deflected during wind",
            pass: val.peak_angle_deg > 10.0,
            detail: format!("peak |angle|={:.1} deg (need > 10)", val.peak_angle_deg),
        },
        Check {
            name: "Pendulum recovers after wind off",
            pass: final_angle < val.peak_angle_deg * 0.5,
            detail: format!(
                "final={final_angle:.1} deg, peak={:.1} deg",
                val.peak_angle_deg
            ),
        },
    ];

    let _ = print_report("Wind (t=13s)", &checks);
}
