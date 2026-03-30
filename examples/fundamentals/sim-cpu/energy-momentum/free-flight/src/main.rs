//! Free Flight — Conservation in Isolation
//!
//! A free-floating rigid body in zero gravity with initial linear and angular
//! velocity. No contacts, no constraints, no forces. All three conservation
//! laws hold to machine precision:
//!
//! - Kinetic energy (KE) is constant
//! - Linear momentum (p = m*v) is constant
//! - Angular momentum magnitude (|L|) is constant
//!
//! The box has asymmetric inertia, so angular velocity precesses in the world
//! frame (torque-free precession / Euler's equations). But angular momentum
//! is conserved — only its direction in body frame changes.
//!
//! Validates:
//! - KE drift < 1e-10 over 10s
//! - Linear momentum drift < 1e-10 over 10s
//! - Angular momentum magnitude drift < 1e-8 over 10s
//!
//! Run with: `cargo run -p example-energy-free-flight --release`

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

const MJCF: &str = r#"
<mujoco model="free-flight">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="box" pos="0 0 0">
      <freejoint name="free"/>
      <inertial pos="0 0 0" mass="2.0" diaginertia="0.05 0.03 0.02"/>
      <geom name="body" type="box" size="0.15 0.1 0.08"
            contype="0" conaffinity="0" rgba="0.35 0.55 0.75 1"/>
    </body>
  </worldbody>

  <sensor>
    <subtreeangmom name="angmom" body="box"/>
  </sensor>
</mujoco>
"#;

// Initial velocities: linear [0.5, 0.3, 0] + angular [0.2, 0.1, 0.4]
const INIT_LINVEL: [f64; 3] = [0.5, 0.3, 0.0];
const INIT_ANGVEL: [f64; 3] = [0.2, 0.1, 0.4];

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Free Flight — Conservation in Isolation ===");
    println!("  Zero gravity, no contacts, no forces");
    println!("  Box with asymmetric inertia (angular velocity precesses)");
    println!("  KE, linear momentum, and angular momentum all conserved");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Free Flight".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<ConservationValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(11.0)
                .print_every(1.0)
                .display(|_m, d| {
                    format!(
                        "t={:.1}  KE={:.10}  |v|={:.10}",
                        d.time,
                        d.energy_kinetic,
                        (d.qvel[0].powi(2) + d.qvel[1].powi(2) + d.qvel[2].powi(2)).sqrt()
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
                conservation_diagnostics,
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
    lin_momentum: [f64; 3],
    ang_momentum_mag: f64,
    mass: f64,
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();

    // Set initial velocities
    for i in 0..3 {
        data.qvel[i] = INIT_LINVEL[i];
        data.qvel[3 + i] = INIT_ANGVEL[i];
    }

    // Forward to compute energy and sensors
    data.forward(&model).expect("forward");

    let mass = model.body_mass[1];
    let ke = data.energy_kinetic;
    let lin_momentum = [
        mass * data.qvel[0],
        mass * data.qvel[1],
        mass * data.qvel[2],
    ];

    // Angular momentum from sensor (triggers mj_subtree_vel)
    let angmom_data = data.sensor_data(&model, 0);
    let ang_momentum_mag =
        (angmom_data[0].powi(2) + angmom_data[1].powi(2) + angmom_data[2].powi(2)).sqrt();

    println!("  Initial state:");
    println!("    mass     = {mass:.3} kg");
    println!("    KE       = {ke:.10} J");
    println!(
        "    p        = [{:.6}, {:.6}, {:.6}] kg*m/s",
        lin_momentum[0], lin_momentum[1], lin_momentum[2]
    );
    println!("    |L|      = {ang_momentum_mag:.10} kg*m^2/s");
    println!();

    let mat_box =
        materials.add(MetalPreset::BrushedMetal.with_color(Color::srgb(0.35, 0.55, 0.75)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("body", mat_box)],
    );

    spawn_example_camera(&mut commands, Vec3::ZERO, 4.0, 0.5, 0.0);
    spawn_physics_hud(&mut commands);

    commands.insert_resource(InitialState {
        ke,
        lin_momentum,
        ang_momentum_mag,
        mass,
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
    hud.section("Free Flight — Conservation");

    // Current KE
    let ke = data.energy_kinetic;
    let ke_drift = (ke - initial.ke).abs();
    hud.scalar("KE", ke, 8);
    hud.scalar("KE_drift", ke_drift, 2);

    // Current linear momentum
    let px = initial.mass * data.qvel[0];
    let py = initial.mass * data.qvel[1];
    let pz = initial.mass * data.qvel[2];
    let p_drift = ((px - initial.lin_momentum[0]).powi(2)
        + (py - initial.lin_momentum[1]).powi(2)
        + (pz - initial.lin_momentum[2]).powi(2))
    .sqrt();
    hud.scalar("|p| drift", p_drift, 2);

    // Current angular momentum (from sensor)
    let angmom_data = data.sensor_data(&model, 0);
    let l_mag = (angmom_data[0].powi(2) + angmom_data[1].powi(2) + angmom_data[2].powi(2)).sqrt();
    let l_drift = (l_mag - initial.ang_momentum_mag).abs();
    hud.scalar("|L|", l_mag, 10);
    hud.scalar("|L| drift", l_drift, 2);

    // Angular velocity (precessing for asymmetric body)
    hud.scalar("omega_x", data.qvel[3], 4);
    hud.scalar("omega_y", data.qvel[4], 4);
    hud.scalar("omega_z", data.qvel[5], 4);

    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct ConservationValidation {
    max_ke_drift: f64,
    max_p_drift: f64,
    max_l_drift: f64,
    reported: bool,
}

fn conservation_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    initial: Res<InitialState>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<ConservationValidation>,
) {
    if data.time < 0.01 {
        return;
    }

    // KE drift
    let ke_drift = (data.energy_kinetic - initial.ke).abs();
    if ke_drift > val.max_ke_drift {
        val.max_ke_drift = ke_drift;
    }

    // Linear momentum drift
    let px = initial.mass * data.qvel[0];
    let py = initial.mass * data.qvel[1];
    let pz = initial.mass * data.qvel[2];
    let p_drift = ((px - initial.lin_momentum[0]).powi(2)
        + (py - initial.lin_momentum[1]).powi(2)
        + (pz - initial.lin_momentum[2]).powi(2))
    .sqrt();
    if p_drift > val.max_p_drift {
        val.max_p_drift = p_drift;
    }

    // Angular momentum magnitude drift (from sensor)
    let angmom_data = data.sensor_data(&model, 0);
    let l_mag = (angmom_data[0].powi(2) + angmom_data[1].powi(2) + angmom_data[2].powi(2)).sqrt();
    let l_drift = (l_mag - initial.ang_momentum_mag).abs();
    if l_drift > val.max_l_drift {
        val.max_l_drift = l_drift;
    }

    // Report once at harness report time
    if harness.reported() && !val.reported {
        val.reported = true;

        let checks = vec![
            Check {
                name: "KE drift < 1e-10",
                pass: val.max_ke_drift < 1e-10,
                detail: format!("max={:.2e}", val.max_ke_drift),
            },
            Check {
                name: "Linear momentum drift < 1e-10",
                pass: val.max_p_drift < 1e-10,
                detail: format!("max={:.2e}", val.max_p_drift),
            },
            Check {
                name: "Angular momentum |L| drift < 1e-8",
                pass: val.max_l_drift < 1e-8,
                detail: format!("max={:.2e}", val.max_l_drift),
            },
        ];
        let _ = print_report("Free Flight — Conservation", &checks);
    }
}
