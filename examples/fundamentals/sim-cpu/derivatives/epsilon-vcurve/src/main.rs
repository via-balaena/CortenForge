//! Epsilon V-Curve — Finding the Optimal Perturbation Size
//!
//! Sweeps epsilon from 1e-2 down to 1e-10, comparing centered FD results
//! against the hybrid analytical reference. Error first decreases (truncation-
//! dominated), then increases (roundoff-dominated). The minimum reveals the
//! optimal epsilon for this model.
//!
//! The pendulum ticks like a clock hand, recomputing the V-curve at each
//! position to show that the optimal epsilon is configuration-independent.
//!
//! Run: `cargo run -p example-derivatives-epsilon-vcurve --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::suboptimal_flops,
    clippy::let_underscore_must_use,
    clippy::too_many_lines,
    clippy::needless_range_loop,
    clippy::while_float,
    clippy::cast_precision_loss,
    non_snake_case
)]

use std::f64::consts::PI;

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{PhysicsHud, render_physics_hud, spawn_example_camera, spawn_physics_hud};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{PhysicsData, PhysicsModel, spawn_model_geoms, sync_geom_transforms};
use sim_core::derivatives::max_relative_error;
use sim_core::validation::{Check, print_report};
use sim_core::{DerivativeConfig, mjd_transition_fd, mjd_transition_hybrid};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="epsilon-vcurve">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="link" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.1 0.1 0.001"/>
      <geom name="rod" type="capsule" size="0.02"
            fromto="0 0 0  0 0 -0.5" rgba="0.48 0.48 0.50 1"/>
      <geom name="tip" type="sphere" size="0.05"
            pos="0 0 -0.5" rgba="0.8 0.4 0.7 1"/>
    </body>
  </worldbody>

  <actuator>
    <motor name="torque" joint="hinge" gear="1"/>
  </actuator>
</mujoco>
"#;

// ── V-curve data ────────────────────────────────────────────────────────────

const SWEEP_EPS: [f64; 9] = [1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9, 1e-10];
const FREEZE_SECS: f64 = 3.0;
const NUM_HOURS: usize = 12;
const TICK_SECS: f64 = 2.0;

struct VcurveRow {
    eps: f64,
    error: f64,
}

/// Compute V-curve: error of centered FD at each eps vs hybrid analytical.
fn compute_vcurve(model: &sim_core::Model, data: &sim_core::Data) -> Vec<VcurveRow> {
    // Reference: hybrid analytical (velocity columns are epsilon-independent)
    let ref_config = DerivativeConfig {
        use_analytical: true,
        ..Default::default()
    };
    let reference = mjd_transition_hybrid(model, data, &ref_config).expect("hybrid");

    let mut rows = Vec::new();
    for &eps in &SWEEP_EPS {
        let fd_config = DerivativeConfig {
            eps,
            centered: true,
            use_analytical: false,
            compute_sensor_derivatives: false,
        };
        let fd = mjd_transition_fd(model, data, &fd_config).expect("fd");
        let (err, _) = max_relative_error(&fd.A, &reference.A, 1e-14);
        rows.push(VcurveRow { eps, error: err });
    }
    rows
}

fn find_optimal(rows: &[VcurveRow]) -> (usize, f64, f64) {
    rows.iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| a.error.partial_cmp(&b.error).expect("non-NaN"))
        .map_or((0, 0.0, 0.0), |(i, r)| (i, r.eps, r.error))
}

fn hour_angle(hour: usize) -> f64 {
    (hour as f64) * (2.0 * PI / NUM_HOURS as f64)
}

#[derive(Resource)]
struct VcurveResult {
    rows: Vec<VcurveRow>,
    optimal_idx: usize,
    optimal_eps: f64,
    optimal_err: f64,
    current_hour: usize,
}

#[derive(Resource)]
struct ClockState {
    released: bool,
    wall_at_tick: f64,
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Epsilon V-Curve ===");
    println!("  Finding the optimal FD perturbation size");
    println!("  1-DOF pendulum — clock-hand sweep through 12 positions");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Epsilon V-Curve".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsHud>()
        .insert_resource(ClockState {
            released: false,
            wall_at_tick: 0.0,
        })
        .add_systems(Startup, setup)
        .add_systems(Update, clock_tick)
        .add_systems(
            PostUpdate,
            (sync_geom_transforms, update_hud, render_physics_hud).chain(),
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

    // Start at 1 o'clock (hour 7 internally) — avoids equilibria
    data.qpos[0] = hour_angle(7);
    data.forward(&model).expect("forward");

    println!(
        "  Model: {} bodies, {} joints, nv={}, nu={}",
        model.nbody, model.njnt, model.nv, model.nu
    );

    // Initial V-curve
    let rows = compute_vcurve(&model, &data);
    let (optimal_idx, optimal_eps, optimal_err) = find_optimal(&rows);

    println!("\n  {:>10}  {:>12}", "eps", "error vs hybrid");
    println!("  {:>10}  {:>12}", "---", "--------------");
    for (i, row) in rows.iter().enumerate() {
        let marker = if i == optimal_idx { " <-- best" } else { "" };
        println!("  {:>10.0e}  {:>12.2e}{marker}", row.eps, row.error);
    }
    println!("\n  Optimal eps: {optimal_eps:.0e} (err = {optimal_err:.2e})");

    // Validation checks
    let has_minimum = rows.len() >= 3 && optimal_idx > 0 && optimal_idx < rows.len() - 1;

    let left_decreasing = rows
        .windows(2)
        .take(optimal_idx)
        .all(|w| w[1].error < w[0].error);

    let right_increasing = if optimal_idx + 1 < rows.len() {
        rows[optimal_idx + 1].error > rows[optimal_idx].error
    } else {
        false
    };

    let optimal_in_range = (1e-8..=1e-4).contains(&optimal_eps);

    let checks = vec![
        Check {
            name: "V-curve has interior minimum",
            pass: has_minimum,
            detail: format!("min at eps={optimal_eps:.0e}"),
        },
        Check {
            name: "Left arm: error decreases",
            pass: left_decreasing,
            detail: "truncation-dominated regime".into(),
        },
        Check {
            name: "Right arm: error increases",
            pass: right_increasing,
            detail: "roundoff-dominated regime".into(),
        },
        Check {
            name: "Optimal eps in [1e-8, 1e-4]",
            pass: optimal_in_range,
            detail: format!("eps={optimal_eps:.0e}"),
        },
    ];
    let _ = print_report("Epsilon V-Curve", &checks);

    // Spawn visuals
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.8, 0.4, 0.7)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("rod", mat_rod), ("tip", mat_tip)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, -0.25),
        1.8,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(VcurveResult {
        rows,
        optimal_idx,
        optimal_eps,
        optimal_err,
        current_hour: 7,
    });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Clock tick ──────────────────────────────────────────────────────────────

fn clock_tick(
    time: Res<Time>,
    mut clock: ResMut<ClockState>,
    mut result: ResMut<VcurveResult>,
    model: Option<Res<PhysicsModel>>,
    data: Option<ResMut<PhysicsData>>,
) {
    let wall = time.elapsed_secs_f64();

    if wall < FREEZE_SECS {
        return;
    }

    let (Some(model), Some(mut data)) = (model, data) else {
        return;
    };

    if !clock.released {
        clock.released = true;
        clock.wall_at_tick = wall;
    }

    if wall - clock.wall_at_tick >= TICK_SECS {
        clock.wall_at_tick = wall;

        let next_hour = (result.current_hour + 1) % NUM_HOURS;
        result.current_hour = next_hour;

        data.qpos[0] = hour_angle(next_hour);
        data.qvel[0] = 0.0;
        data.ctrl[0] = 0.0;
        data.forward(&model).expect("forward");

        result.rows = compute_vcurve(&model, &data);
        let (opt_idx, opt_eps, opt_err) = find_optimal(&result.rows);
        result.optimal_idx = opt_idx;
        result.optimal_eps = opt_eps;
        result.optimal_err = opt_err;
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(
    result: Option<Res<VcurveResult>>,
    clock: Res<ClockState>,
    wall: Res<Time>,
    data: Option<Res<PhysicsData>>,
    mut hud: ResMut<PhysicsHud>,
) {
    let Some(r) = result else { return };

    hud.clear();
    hud.section("Epsilon V-Curve");
    hud.raw("Optimal FD perturbation size".into());
    hud.raw(String::new());

    // Clock position
    let clock_hour = (r.current_hour + 6) % 12;
    let clock_label = if clock_hour == 0 { 12 } else { clock_hour };
    hud.raw(format!(
        "Position: {} o'clock ({:.0} deg)",
        clock_label,
        hour_angle(r.current_hour).to_degrees()
    ));
    hud.raw(String::new());

    // V-curve table
    hud.raw(format!("{:>10}  {:>10}", "eps", "error"));
    hud.raw(format!("{:>10}  {:>10}", "---", "-----"));

    for (i, row) in r.rows.iter().enumerate() {
        let marker = if i == r.optimal_idx { " <--" } else { "" };
        hud.raw(format!("{:>10.0e}  {:>10.2e}{marker}", row.eps, row.error));
    }
    hud.raw(String::new());

    // Optimal
    hud.section("Optimal Epsilon");
    hud.raw(format!("  eps = {:.0e}", r.optimal_eps));
    hud.raw(format!("  err = {:.2e}", r.optimal_err));
    hud.raw(String::new());

    hud.raw("Left of min: truncation dominates".into());
    hud.raw("Right of min: roundoff dominates".into());

    // Countdown or live state
    if !clock.released {
        hud.raw(String::new());
        let remaining = (FREEZE_SECS - wall.elapsed_secs_f64()).max(0.0);
        hud.raw(format!("First tick in {remaining:.1}s"));
    } else if let Some(data) = data {
        hud.raw(String::new());
        hud.scalar("qpos", data.qpos[0], 3);
    }
}
