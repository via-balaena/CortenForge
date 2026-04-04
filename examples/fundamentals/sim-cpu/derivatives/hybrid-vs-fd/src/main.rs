//! Hybrid vs FD — Analytical+FD Comparison
//!
//! Compares `mjd_transition_hybrid()` (analytical velocity derivatives + FD
//! position columns) against pure `mjd_transition_fd()`. Same output, lower
//! cost. A 3-link pendulum at non-trivial angles shows the two methods agree
//! to < 1e-4 relative error while hybrid is faster.
//!
//! The HUD stages the comparison as a race: FD computes first, then hybrid,
//! then the winner is declared with speedup and error analysis.
//!
//! Run: `cargo run -p example-derivatives-hybrid-vs-fd --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::suboptimal_flops,
    clippy::let_underscore_must_use,
    clippy::too_many_lines,
    clippy::many_single_char_names,
    non_snake_case
)]

use std::time::Instant;

use bevy::prelude::*;
use nalgebra::DMatrix;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{PhysicsHud, render_physics_hud, spawn_example_camera, spawn_physics_hud};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{PhysicsData, PhysicsModel, spawn_model_geoms};
use sim_core::validation::{Check, print_report};
use sim_core::{DerivativeConfig, max_relative_error, mjd_transition_fd, mjd_transition_hybrid};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="hybrid-vs-fd">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="link1" pos="0 0 0">
      <joint name="j1" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.1 0.1 0.001"/>
      <geom name="rod1" type="capsule" size="0.025"
            fromto="0 0 0  0 0 -1" rgba="0.48 0.48 0.50 1"/>

      <body name="link2" pos="0 0 -1">
        <joint name="j2" type="hinge" axis="0 1 0"/>
        <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.1 0.1 0.001"/>
        <geom name="rod2" type="capsule" size="0.025"
              fromto="0 0 0  0 0 -1" rgba="0.48 0.48 0.50 1"/>

        <body name="link3" pos="0 0 -1">
          <joint name="j3" type="hinge" axis="0 1 0"/>
          <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.1 0.1 0.001"/>
          <geom name="rod3" type="capsule" size="0.025"
                fromto="0 0 0  0 0 -1" rgba="0.48 0.48 0.50 1"/>
          <geom name="tip" type="sphere" size="0.06"
                pos="0 0 -1" rgba="0.2 0.6 0.9 1"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"#;

// ── Race state machine ────────────────────────────────────────────────────

/// Phases of the computation race.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Ready,        // Show pose, countdown
    ComputingFd,  // Flash "Computing FD..."
    FdDone,       // Show FD result, pause
    ComputingHyb, // Flash "Computing Hybrid..."
    Done,         // Show everything + winner
}

#[derive(Resource)]
struct RaceState {
    phase: Phase,
    phase_start: f64,
    // Results (filled as phases complete)
    fd_time_us: f64,
    hyb_time_us: f64,
    a_fd: Option<DMatrix<f64>>,
    a_diff: Option<DMatrix<f64>>,
    err_a: f64,
    err_b: f64,
}

const READY_DURATION: f64 = 3.0;
const COMPUTING_FLASH: f64 = 0.5;
const RESULT_PAUSE: f64 = 2.0;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Hybrid vs FD ===");
    println!("  mjd_transition_hybrid vs mjd_transition_fd");
    println!("  3-link pendulum at non-trivial angles");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Hybrid vs FD".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsHud>()
        .insert_resource(RaceState {
            phase: Phase::Ready,
            phase_start: 0.0,
            fd_time_us: 0.0,
            hyb_time_us: 0.0,
            a_fd: None,
            a_diff: None,
            err_a: 0.0,
            err_b: 0.0,
        })
        .add_systems(Startup, setup)
        .add_systems(Update, race_driver)
        .add_systems(PostUpdate, (update_hud, render_physics_hud).chain())
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.qpos[1] = -0.8;
    data.qpos[2] = 1.2;
    data.forward(&model).expect("forward");

    println!(
        "  Model: {} bodies, {} joints, nv={}",
        model.nbody, model.njnt, model.nv
    );

    // Spawn visuals
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.6, 0.9)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("rod1", mat_rod.clone()),
            ("rod2", mat_rod.clone()),
            ("rod3", mat_rod),
            ("tip", mat_tip),
        ],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(-0.5, -1.25, -1.5),
        5.0,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Race driver ─────────────────────────────────────────────────────────────

fn race_driver(
    wall: Res<Time>,
    model: Option<Res<PhysicsModel>>,
    data: Option<Res<PhysicsData>>,
    mut race: ResMut<RaceState>,
) {
    let t = wall.elapsed_secs_f64();
    let in_phase = t - race.phase_start;

    match race.phase {
        Phase::Ready => {
            if in_phase >= READY_DURATION {
                race.phase = Phase::ComputingFd;
                race.phase_start = t;
            }
        }
        Phase::ComputingFd => {
            if in_phase >= COMPUTING_FLASH {
                // Actually compute FD now
                let (Some(model), Some(data)) = (model, data) else {
                    return;
                };
                let config = DerivativeConfig {
                    use_analytical: false,
                    ..Default::default()
                };
                let t0 = Instant::now();
                let fd = mjd_transition_fd(&model, &data, &config).expect("fd");
                race.fd_time_us = t0.elapsed().as_secs_f64() * 1e6;
                race.a_fd = Some(fd.A);

                println!("  FD time: {:.0} us", race.fd_time_us);

                race.phase = Phase::FdDone;
                race.phase_start = wall.elapsed_secs_f64();
            }
        }
        Phase::FdDone => {
            if in_phase >= RESULT_PAUSE {
                race.phase = Phase::ComputingHyb;
                race.phase_start = t;
            }
        }
        Phase::ComputingHyb => {
            if in_phase >= COMPUTING_FLASH {
                let (Some(model), Some(data)) = (model, data) else {
                    return;
                };
                let config = DerivativeConfig {
                    use_analytical: true,
                    ..Default::default()
                };
                let t0 = Instant::now();
                let hyb = mjd_transition_hybrid(&model, &data, &config).expect("hybrid");
                race.hyb_time_us = t0.elapsed().as_secs_f64() * 1e6;

                // Compare — clone a_fd out to avoid borrow conflict
                let a_fd = race.a_fd.clone().expect("FD should be done");
                let (err_a, loc_a) = max_relative_error(&a_fd, &hyb.A, 1e-10);
                race.err_a = err_a;
                race.err_b = 0.0; // both B are 6×0, trivially equal

                // Difference matrix
                let n = a_fd.nrows();
                let mut diff = DMatrix::zeros(n, n);
                for r in 0..n {
                    for c in 0..n {
                        diff[(r, c)] = (a_fd[(r, c)] - hyb.A[(r, c)]).abs();
                    }
                }
                race.a_diff = Some(diff);

                println!("  Hybrid time: {:.0} us", race.hyb_time_us);
                println!("  Max |A| err: {err_a:.2e} at ({}, {})", loc_a.0, loc_a.1);

                // Validation checks
                let dims_ok = a_fd.nrows() == 6 && hyb.A.nrows() == 6;
                let timing_ok = race.hyb_time_us <= race.fd_time_us * 1.2;
                let checks = vec![
                    Check {
                        name: "A dimensions match (6x6)",
                        pass: dims_ok,
                        detail: format!(
                            "fd={}x{}, hyb={}x{}",
                            a_fd.nrows(),
                            a_fd.ncols(),
                            hyb.A.nrows(),
                            hyb.A.ncols()
                        ),
                    },
                    Check {
                        name: "A agreement < 1e-4",
                        pass: err_a < 1e-4,
                        detail: format!("max rel err = {err_a:.2e}"),
                    },
                    Check {
                        name: "B agreement < 1e-4",
                        pass: true,
                        detail: "both 6x0, trivially equal".into(),
                    },
                    Check {
                        name: "Hybrid not slower (within 20%)",
                        pass: timing_ok,
                        detail: format!(
                            "fd={:.0}us, hyb={:.0}us",
                            race.fd_time_us, race.hyb_time_us
                        ),
                    },
                ];
                let _ = print_report("Hybrid vs FD", &checks);

                race.phase = Phase::Done;
                race.phase_start = wall.elapsed_secs_f64();
            }
        }
        Phase::Done => {}
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(race: Res<RaceState>, wall: Res<Time>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Hybrid vs FD Race");
    hud.raw("3-link pendulum, qpos=[0.5, -0.8, 1.2]".into());
    hud.raw(String::new());

    match race.phase {
        Phase::Ready => {
            let remaining =
                (READY_DURATION - (wall.elapsed_secs_f64() - race.phase_start)).max(0.0);
            hud.raw(format!("Starting race in {remaining:.1}s..."));
        }
        Phase::ComputingFd => {
            hud.raw("  Pure FD:  computing...".into());
            hud.raw("  Hybrid:   waiting".into());
        }
        Phase::FdDone => {
            hud.raw(format!("  Pure FD:  {:>6.0} us", race.fd_time_us));
            hud.raw("  Hybrid:   waiting...".into());
        }
        Phase::ComputingHyb => {
            hud.raw(format!("  Pure FD:  {:>6.0} us", race.fd_time_us));
            hud.raw("  Hybrid:   computing...".into());
        }
        Phase::Done => {
            let speedup = race.fd_time_us / race.hyb_time_us;
            let winner = if race.hyb_time_us < race.fd_time_us {
                "HYBRID WINS"
            } else {
                "FD WINS"
            };

            hud.raw(format!("  Pure FD:  {:>6.0} us", race.fd_time_us));
            hud.raw(format!("  Hybrid:   {:>6.0} us", race.hyb_time_us));
            hud.raw(String::new());
            hud.raw(format!("  >>> {winner} ({speedup:.1}x speedup) <<<"));
            hud.raw(String::new());

            // Agreement
            hud.raw(format!("Max |A| error: {:.2e}", race.err_a));
            hud.raw(format!("Max |B| error: {:.2e}", race.err_b));
            hud.raw(if race.err_a < 1e-4 {
                "Same answer? YES".into()
            } else {
                "Same answer? NO (disagreement!)".into()
            });
            hud.raw(String::new());

            // Difference matrix
            if let Some(ref diff) = race.a_diff {
                hud.raw("|A_fd - A_hyb|:".into());
                for row in 0..diff.nrows() {
                    let vals: Vec<String> = (0..diff.ncols())
                        .map(|c| {
                            let v = diff[(row, c)];
                            if v < 1e-12 {
                                "     0   ".into()
                            } else {
                                format!("{v:>9.1e}")
                            }
                        })
                        .collect();
                    hud.raw(format!(" [{}]", vals.join(",")));
                }
            }
        }
    }
}
