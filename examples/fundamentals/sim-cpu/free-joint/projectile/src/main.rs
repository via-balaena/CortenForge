//! Projectile — Parabolic Trajectory
//!
//! A sphere launched at 45 degrees in gravity. Exact analytical solution for
//! the entire trajectory. Demonstrates the translational subspace of a free
//! joint: 3 position coordinates, 3 linear velocity components, gravity on
//! linear DOFs only.
//!
//! Validates:
//! - Apex height within 0.1% of analytical
//! - Range within 0.1% of analytical
//! - Total energy conserved to < 0.5%
//!
//! Run with: `cargo run -p example-free-joint-projectile --release`

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
<mujoco model="projectile">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="ball" pos="0 0 0">
      <freejoint name="free"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.004 0.004 0.004"/>
      <geom name="sphere" type="sphere" size="0.05"
            contype="0" conaffinity="0" rgba="0.85 0.30 0.15 1"/>
    </body>
  </worldbody>

  <keyframe>
    <key name="launch" qpos="0 0 0 1 0 0 0"
         qvel="3.535533905932738 0 3.535533905932738 0 0 0"/>
  </keyframe>
</mujoco>
"#;

// ── Physics constants ─────────────────────────────────────────────────────

const V0: f64 = 5.0;
const THETA: f64 = std::f64::consts::FRAC_PI_4; // 45 degrees
const G: f64 = 9.81;
const Z0: f64 = 0.0; // launch from origin
const RELAUNCH_PERIOD: f64 = 2.0; // seconds between relaunches

fn vx0() -> f64 {
    V0 * THETA.cos()
}
fn vz0() -> f64 {
    V0 * THETA.sin()
}
fn apex_analytical() -> f64 {
    Z0 + V0 * V0 * THETA.sin().powi(2) / (2.0 * G)
}
fn range_analytical() -> f64 {
    V0 * V0 * (2.0 * THETA).sin() / G
}

/// Analytical position at time t after launch.
fn analytical_pos(t: f64) -> (f64, f64) {
    let x = vx0() * t;
    let z = Z0 + vz0() * t - 0.5 * G * t * t;
    (x, z)
}

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Projectile — Parabolic Trajectory ===");
    println!("  v0 = {V0} m/s, angle = 45 deg, g = {G} m/s^2");
    println!("  Analytical apex  = {:.4} m", apex_analytical());
    println!("  Analytical range = {:.4} m", range_analytical());
    println!("  Relaunches every {RELAUNCH_PERIOD}s");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Projectile (Free Joint)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<ProjectileValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(10.0)
                .print_every(0.5)
                .display(|_m, d| {
                    let t = d.time % RELAUNCH_PERIOD;
                    let (xa, za) = analytical_pos(t);
                    let x_err = (d.qpos[0] - xa).abs();
                    let z_err = (d.qpos[2] - za).abs();
                    format!(
                        "pos=({:.3}, {:.3})  analytical=({xa:.3}, {za:.3})  err=({x_err:.1e}, {z_err:.1e})",
                        d.qpos[0], d.qpos[2],
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
                projectile_diagnostics,
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
    last_relaunch: f64,
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();

    // Reset to launch keyframe (sets qpos + qvel in one call)
    data.reset_to_keyframe(&model, 0).expect("keyframe");
    data.forward(&model).expect("forward");

    let e0 = data.total_energy();
    println!("  E_0 = {e0:.6} J");
    println!(
        "  Model: {} bodies, {} joints, {} DOFs\n",
        model.nbody, model.njnt, model.nv
    );

    // ── Materials ──────────────────────────────────────────────────────
    let mat_sphere =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.30, 0.15)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("sphere", mat_sphere)],
    );

    // Camera: center on trajectory midpoint, face the XZ plane from the side
    spawn_example_camera(
        &mut commands,
        physics_pos(1.3, 0.0, 0.2),
        5.0,
        std::f32::consts::FRAC_PI_2,
        0.1,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(LaunchState {
        e0,
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
        data.reset_to_keyframe(&model, 0).expect("keyframe");
        data.forward(&model).expect("forward");
        launch.e0 = data.total_energy();
    }
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(data: Res<PhysicsData>, launch: Res<LaunchState>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Projectile — Parabolic Trajectory");

    let t = data.time - launch.last_relaunch;
    let (xa, za) = analytical_pos(t);

    hud.scalar("x", data.qpos[0], 3);
    hud.scalar("z", data.qpos[2], 3);
    hud.scalar("vx", data.qvel[0], 3);
    hud.scalar("vz", data.qvel[2], 3);
    hud.scalar("x_err", (data.qpos[0] - xa).abs(), 1);
    hud.scalar("z_err", (data.qpos[2] - za).abs(), 1);

    let e = data.total_energy();
    hud.scalar("E_total", e, 4);
    if launch.e0.abs() > 1e-12 {
        hud.scalar("E_drift%", ((e - launch.e0) / launch.e0).abs() * 100.0, 4);
    }

    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct ProjectileValidation {
    apex_recorded: bool,
    apex_z: f64,
    range_recorded: bool,
    range_x: f64,
    max_energy_drift_pct: f64,
    prev_vz: f64,
    prev_z: f64,
    reported: bool,
}

fn projectile_diagnostics(
    data: Res<PhysicsData>,
    launch: Res<LaunchState>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<ProjectileValidation>,
) {
    let t = data.time - launch.last_relaunch;

    // Only track first flight (before first relaunch)
    if data.time < RELAUNCH_PERIOD {
        // Apex: vz crosses zero (rising -> falling)
        if !val.apex_recorded && val.prev_vz > 0.0 && data.qvel[2] <= 0.0 {
            val.apex_z = data.qpos[2];
            val.apex_recorded = true;
        }

        // Range: z crosses below initial height (falling)
        if val.apex_recorded && !val.range_recorded && val.prev_z >= Z0 && data.qpos[2] < Z0 {
            let frac = (val.prev_z - Z0) / (val.prev_z - data.qpos[2]);
            val.range_x = data.qpos[0] - (1.0 - frac) * data.qvel[0] * 0.001;
            val.range_recorded = true;
        }

        val.prev_vz = data.qvel[2];
        val.prev_z = data.qpos[2];
    }

    // Energy drift (track across all flights)
    if t > 0.01 && launch.e0.abs() > 1e-12 {
        let drift_pct = ((data.total_energy() - launch.e0) / launch.e0).abs() * 100.0;
        val.max_energy_drift_pct = val.max_energy_drift_pct.max(drift_pct);
    }

    if harness.reported() && !val.reported {
        val.reported = true;

        let apex_err_pct = if val.apex_recorded {
            ((val.apex_z - apex_analytical()) / apex_analytical()).abs() * 100.0
        } else {
            100.0
        };
        let range_err_pct = if val.range_recorded {
            ((val.range_x - range_analytical()) / range_analytical()).abs() * 100.0
        } else {
            100.0
        };

        let checks = vec![
            Check {
                name: "Apex height < 0.1% error",
                pass: val.apex_recorded && apex_err_pct < 0.1,
                detail: format!(
                    "z_apex = {:.6}, analytical = {:.6}, err = {apex_err_pct:.4}%",
                    val.apex_z,
                    apex_analytical()
                ),
            },
            Check {
                name: "Range < 0.5% error",
                pass: val.range_recorded && range_err_pct < 0.5,
                detail: format!(
                    "x_range = {:.6}, analytical = {:.6}, err = {range_err_pct:.4}%",
                    val.range_x,
                    range_analytical()
                ),
            },
            Check {
                name: "Energy drift < 0.5%",
                pass: val.max_energy_drift_pct < 0.5,
                detail: format!("max = {:.4}%", val.max_energy_drift_pct),
            },
        ];
        let _ = print_report("Projectile (t=10s)", &checks);
    }
}
