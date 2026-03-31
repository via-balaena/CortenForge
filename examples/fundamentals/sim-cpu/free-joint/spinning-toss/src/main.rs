//! Spinning Toss — Full 6-DOF Flight
//!
//! A brick with asymmetric inertia tossed upward with both linear and angular
//! velocity. The COM follows a parabolic arc while the body tumbles — both
//! subspaces active simultaneously, demonstrating the full free joint.
//!
//! Validates:
//! - COM trajectory matches point-mass projectile to < 0.1%
//! - |L| conserved to < 1e-8 (rotation decoupled from gravity)
//! - Total energy drift < 0.5%
//!
//! Run with: `cargo run -p example-free-joint-spinning-toss --release`

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

const MJCF: &str = r#"
<mujoco model="spinning-toss">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="brick" pos="0 0 0.5">
      <freejoint name="free"/>
      <inertial pos="0 0 0" mass="1.5" diaginertia="0.06 0.03 0.015"/>
      <geom name="body" type="box" size="0.18 0.12 0.07"
            contype="0" conaffinity="0" rgba="0.40 0.55 0.75 1"/>
      <geom name="marker" type="sphere" size="0.025"
            pos="0.18 0.12 0.07" contype="0" conaffinity="0"
            rgba="0.95 0.25 0.15 1"/>
    </body>
  </worldbody>

  <sensor>
    <subtreeangmom name="angmom" body="brick"/>
  </sensor>
</mujoco>
"#;

// ── Physics constants ─────────────────────────────────────────────────────

const INIT_LINVEL: [f64; 3] = [2.0, 0.0, 4.0];
const INIT_ANGVEL: [f64; 3] = [3.0, 1.0, 0.5];
const Z0: f64 = 0.5;
const G: f64 = 9.81;
const RELAUNCH_PERIOD: f64 = 3.0;

/// Analytical COM position at time t after launch (point-mass projectile).
fn analytical_pos(t: f64) -> (f64, f64) {
    let x = INIT_LINVEL[0] * t;
    let z = Z0 + INIT_LINVEL[2] * t - 0.5 * G * t * t;
    (x, z)
}

fn apex_analytical() -> f64 {
    Z0 + INIT_LINVEL[2] * INIT_LINVEL[2] / (2.0 * G)
}

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Spinning Toss — Full 6-DOF Flight ===");
    println!(
        "  Linear:  [{}, {}, {}] m/s",
        INIT_LINVEL[0], INIT_LINVEL[1], INIT_LINVEL[2]
    );
    println!(
        "  Angular: [{}, {}, {}] rad/s",
        INIT_ANGVEL[0], INIT_ANGVEL[1], INIT_ANGVEL[2]
    );
    println!("  Apex height = {:.4} m", apex_analytical());
    println!("  Relaunches every {RELAUNCH_PERIOD}s");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Spinning Toss (Free Joint)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<TossValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(10.0)
                .print_every(1.0)
                .display(|m, d| {
                    let angmom = d.sensor_data(m, 0);
                    let l_mag = (angmom[0].powi(2) + angmom[1].powi(2) + angmom[2].powi(2)).sqrt();
                    format!(
                        "pos=({:+.2}, {:+.2})  |L|={l_mag:.8}  E={:.4}",
                        d.qpos[0],
                        d.qpos[2],
                        d.total_energy(),
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (relaunch, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                toss_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

// ── Setup ─────────────────────────────────────────────────────────────────

#[derive(Resource)]
struct LaunchState {
    e0: f64,
    l0_mag: f64,
    last_relaunch: f64,
}

fn launch_body(data: &mut sim_core::types::Data, model: &sim_core::types::Model) -> (f64, f64) {
    // Position
    data.qpos[0] = 0.0;
    data.qpos[1] = 0.0;
    data.qpos[2] = Z0;
    // Identity quaternion
    data.qpos[3] = 1.0;
    data.qpos[4] = 0.0;
    data.qpos[5] = 0.0;
    data.qpos[6] = 0.0;

    // Velocity
    for i in 0..3 {
        data.qvel[i] = INIT_LINVEL[i];
        data.qvel[3 + i] = INIT_ANGVEL[i];
    }

    data.forward(model).expect("forward");

    let e0 = data.total_energy();
    let angmom = data.sensor_data(model, 0);
    let l0_mag = (angmom[0].powi(2) + angmom[1].powi(2) + angmom[2].powi(2)).sqrt();
    (e0, l0_mag)
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();

    let (e0, l0_mag) = launch_body(&mut data, &model);

    println!("  E_0  = {e0:.6} J");
    println!("  |L|_0 = {l0_mag:.10} kg*m^2/s");
    println!(
        "  Model: {} bodies, {} joints, {} DOFs\n",
        model.nbody, model.njnt, model.nv
    );

    // ── Materials ──────────────────────────────────────────────────────
    let mat_body =
        materials.add(MetalPreset::BrushedMetal.with_color(Color::srgb(0.40, 0.55, 0.75)));
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

    // Camera: view from the side, centered on the arc midpoint
    spawn_example_camera(
        &mut commands,
        physics_pos(1.0, 0.0, 0.8),
        5.0,
        std::f32::consts::FRAC_PI_2,
        std::f32::consts::FRAC_PI_4,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(LaunchState {
        e0,
        l0_mag,
        last_relaunch: 0.0,
    });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Relaunch ──────────────────────────────────────────────────────────────

fn relaunch(
    model: Res<PhysicsModel>,
    mut data: ResMut<PhysicsData>,
    mut launch: ResMut<LaunchState>,
) {
    if data.time - launch.last_relaunch >= RELAUNCH_PERIOD {
        launch.last_relaunch = data.time;
        let (e0, l0_mag) = launch_body(&mut data, &model);
        launch.e0 = e0;
        launch.l0_mag = l0_mag;
    }
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    launch: Res<LaunchState>,
    mut hud: ResMut<PhysicsHud>,
) {
    hud.clear();
    hud.section("Spinning Toss — Full 6-DOF");

    let t = data.time - launch.last_relaunch;
    let (xa, za) = analytical_pos(t);

    hud.scalar("x", data.qpos[0], 3);
    hud.scalar("z", data.qpos[2], 3);
    hud.scalar("x_err", (data.qpos[0] - xa).abs(), 1);
    hud.scalar("z_err", (data.qpos[2] - za).abs(), 1);

    let angmom = data.sensor_data(&model, 0);
    let l_mag = (angmom[0].powi(2) + angmom[1].powi(2) + angmom[2].powi(2)).sqrt();
    hud.scalar("|L|", l_mag, 8);
    hud.scalar("|L| drift", (l_mag - launch.l0_mag).abs(), 2);

    let e = data.total_energy();
    hud.scalar("E_total", e, 4);
    if launch.e0.abs() > 1e-12 {
        hud.scalar("E_drift%", ((e - launch.e0) / launch.e0).abs() * 100.0, 4);
    }

    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct TossValidation {
    max_x_err_pct: f64,
    max_z_err_pct: f64,
    max_l_drift: f64,
    max_e_drift_pct: f64,
    apex_z: f64,
    apex_recorded: bool,
    prev_vz: f64,
    reported: bool,
}

fn toss_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    launch: Res<LaunchState>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<TossValidation>,
) {
    let t = data.time - launch.last_relaunch;
    if t < 0.01 {
        val.prev_vz = data.qvel[2];
        return;
    }

    // COM vs analytical (only during first flight, t < RELAUNCH_PERIOD for clean data)
    if data.time < RELAUNCH_PERIOD {
        let (xa, za) = analytical_pos(t);
        if xa.abs() > 0.01 {
            let x_err_pct = ((data.qpos[0] - xa) / xa).abs() * 100.0;
            val.max_x_err_pct = val.max_x_err_pct.max(x_err_pct);
        }
        if za.abs() > 0.1 {
            let z_err_pct = ((data.qpos[2] - za) / za).abs() * 100.0;
            val.max_z_err_pct = val.max_z_err_pct.max(z_err_pct);
        }

        // Apex detection
        if !val.apex_recorded && val.prev_vz > 0.0 && data.qvel[2] <= 0.0 {
            val.apex_z = data.qpos[2];
            val.apex_recorded = true;
        }
        val.prev_vz = data.qvel[2];
    }

    // Angular momentum (every frame)
    let angmom = data.sensor_data(&model, 0);
    let l_mag = (angmom[0].powi(2) + angmom[1].powi(2) + angmom[2].powi(2)).sqrt();
    let l_drift = (l_mag - launch.l0_mag).abs();
    val.max_l_drift = val.max_l_drift.max(l_drift);

    // Energy (every frame)
    if launch.e0.abs() > 1e-12 {
        let e_drift_pct = ((data.total_energy() - launch.e0) / launch.e0).abs() * 100.0;
        val.max_e_drift_pct = val.max_e_drift_pct.max(e_drift_pct);
    }

    if harness.reported() && !val.reported {
        val.reported = true;

        let apex_err_pct = if val.apex_recorded {
            ((val.apex_z - apex_analytical()) / apex_analytical()).abs() * 100.0
        } else {
            100.0
        };

        let checks = vec![
            Check {
                name: "Apex height < 0.1%",
                pass: val.apex_recorded && apex_err_pct < 0.1,
                detail: format!(
                    "z = {:.6}, analytical = {:.6}, err = {apex_err_pct:.4}%",
                    val.apex_z,
                    apex_analytical()
                ),
            },
            Check {
                name: "COM x tracks projectile",
                pass: val.max_x_err_pct < 0.1,
                detail: format!("max err = {:.4}%", val.max_x_err_pct),
            },
            Check {
                name: "|L| conserved < 1e-8",
                pass: val.max_l_drift < 1e-8,
                detail: format!("max drift = {:.2e}", val.max_l_drift),
            },
            Check {
                name: "Energy drift < 0.5%",
                pass: val.max_e_drift_pct < 0.5,
                detail: format!("max = {:.4}%", val.max_e_drift_pct),
            },
        ];
        let _ = print_report("Spinning Toss (t=10s)", &checks);
    }
}
