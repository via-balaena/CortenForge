//! Tumble — Torque-Free Precession
//!
//! A box with asymmetric inertia (Ixx != Iyy != Izz) in zero gravity with
//! angular velocity only. The angular velocity vector precesses around the
//! angular momentum vector, but |L| is exactly conserved.
//!
//! Demonstrates: The rotational subspace of a free joint — 4-component
//! quaternion in qpos[3..7], 3-component angular velocity in qvel[3..6],
//! quaternion integration via exponential map on SO(3).
//!
//! Validates:
//! - KE drift < 1e-10 over 15s
//! - |L| drift < 1e-8 over 15s
//! - Quaternion norm drift < 1e-10 over 15s
//!
//! Run with: `cargo run -p example-free-joint-tumble --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::struct_excessive_bools
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

// ── MJCF Model ────────────────────────────────────────────────────────────
//
// Asymmetric inertia box in zero gravity. A red marker sphere on one corner
// makes the tumbling rotation easy to track visually.
//
const MJCF: &str = r#"
<mujoco model="tumble">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="box" pos="0 0 0">
      <freejoint name="free"/>
      <inertial pos="0 0 0" mass="2.0" diaginertia="0.10 0.06 0.03"/>
      <geom name="body" type="box" size="0.25 0.18 0.12"
            contype="0" conaffinity="0" rgba="0.30 0.55 0.80 1"/>
      <geom name="marker" type="sphere" size="0.03"
            pos="0.25 0.18 0.12" contype="0" conaffinity="0"
            rgba="0.95 0.25 0.15 1"/>
    </body>
  </worldbody>

  <sensor>
    <subtreeangmom name="angmom" body="box"/>
  </sensor>
</mujoco>
"#;

// Angular velocity: primarily about major axis (X) with small perturbation.
const INIT_ANGVEL: [f64; 3] = [2.0, 0.5, 0.1];

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Tumble — Torque-Free Precession ===");
    println!("  Asymmetric box in zero gravity, angular velocity only");
    println!(
        "  omega_0 = [{}, {}, {}] rad/s",
        INIT_ANGVEL[0], INIT_ANGVEL[1], INIT_ANGVEL[2]
    );
    println!("  Angular momentum |L| is conserved; omega precesses");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Tumble (Free Joint)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<TumbleValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let angmom = d.sensor_data(m, 0);
                    let l_mag = (angmom[0].powi(2) + angmom[1].powi(2) + angmom[2].powi(2)).sqrt();
                    format!(
                        "omega=[{:+.3}, {:+.3}, {:+.3}]  |L|={l_mag:.10}",
                        d.qvel[3], d.qvel[4], d.qvel[5],
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                tumble_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

// ── Initial state snapshot ────────────────────────────────────────────────

#[derive(Resource)]
struct InitialState {
    ke: f64,
    ang_momentum_mag: f64,
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();

    // Set angular velocity only (no linear velocity — body stays at origin)
    data.qvel[3] = INIT_ANGVEL[0];
    data.qvel[4] = INIT_ANGVEL[1];
    data.qvel[5] = INIT_ANGVEL[2];

    data.forward(&model).expect("forward");

    let ke = data.energy_kinetic;
    let angmom = data.sensor_data(&model, 0);
    let ang_momentum_mag = (angmom[0].powi(2) + angmom[1].powi(2) + angmom[2].powi(2)).sqrt();

    println!("  KE_0     = {ke:.10} J");
    println!("  |L|_0    = {ang_momentum_mag:.10} kg*m^2/s");
    println!(
        "  Model: {} bodies, {} joints, {} DOFs\n",
        model.nbody, model.njnt, model.nv
    );

    // ── Materials ──────────────────────────────────────────────────────
    let mat_body =
        materials.add(MetalPreset::BrushedMetal.with_color(Color::srgb(0.30, 0.55, 0.80)));
    let mat_marker =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.95, 0.25, 0.15)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("body", mat_body), ("marker", mat_marker)],
    );

    // Camera: center at origin, distance 1.5, elevated ~90 degrees to look down
    spawn_example_camera(
        &mut commands,
        Vec3::ZERO,
        1.5,
        0.5,
        std::f32::consts::FRAC_PI_2 - 0.1,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(InitialState {
        ke,
        ang_momentum_mag,
    });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    initial: Res<InitialState>,
    mut hud: ResMut<PhysicsHud>,
) {
    hud.clear();
    hud.section("Tumble — Torque-Free Precession");

    hud.scalar("omega_x", data.qvel[3], 3);
    hud.scalar("omega_y", data.qvel[4], 3);
    hud.scalar("omega_z", data.qvel[5], 3);

    let angmom = data.sensor_data(&model, 0);
    let l_mag = (angmom[0].powi(2) + angmom[1].powi(2) + angmom[2].powi(2)).sqrt();
    hud.scalar("|L|", l_mag, 8);
    hud.scalar("|L| drift", (l_mag - initial.ang_momentum_mag).abs(), 2);

    hud.scalar("KE", data.energy_kinetic, 8);
    hud.scalar("KE drift", (data.energy_kinetic - initial.ke).abs(), 2);

    let qnorm =
        (data.qpos[3].powi(2) + data.qpos[4].powi(2) + data.qpos[5].powi(2) + data.qpos[6].powi(2))
            .sqrt();
    hud.scalar("|quat|", qnorm, 12);

    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct TumbleValidation {
    max_ke_drift: f64,
    max_l_drift: f64,
    max_quat_drift: f64,
    reported: bool,
}

fn tumble_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    initial: Res<InitialState>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<TumbleValidation>,
) {
    if data.time < 0.01 {
        return;
    }

    let ke_drift = (data.energy_kinetic - initial.ke).abs();
    val.max_ke_drift = val.max_ke_drift.max(ke_drift);

    let angmom = data.sensor_data(&model, 0);
    let l_mag = (angmom[0].powi(2) + angmom[1].powi(2) + angmom[2].powi(2)).sqrt();
    let l_drift = (l_mag - initial.ang_momentum_mag).abs();
    val.max_l_drift = val.max_l_drift.max(l_drift);

    let qnorm =
        (data.qpos[3].powi(2) + data.qpos[4].powi(2) + data.qpos[5].powi(2) + data.qpos[6].powi(2))
            .sqrt();
    let quat_drift = (qnorm - 1.0).abs();
    val.max_quat_drift = val.max_quat_drift.max(quat_drift);

    if harness.reported() && !val.reported {
        val.reported = true;

        let checks = vec![
            Check {
                name: "KE drift < 1e-10",
                pass: val.max_ke_drift < 1e-10,
                detail: format!("max = {:.2e}", val.max_ke_drift),
            },
            Check {
                name: "|L| drift < 1e-8",
                pass: val.max_l_drift < 1e-8,
                detail: format!("max = {:.2e}", val.max_l_drift),
            },
            Check {
                name: "|quat| drift < 1e-10",
                pass: val.max_quat_drift < 1e-10,
                detail: format!("max = {:.2e}", val.max_quat_drift),
            },
        ];
        let _ = print_report("Tumble (t=15s)", &checks);
    }
}
