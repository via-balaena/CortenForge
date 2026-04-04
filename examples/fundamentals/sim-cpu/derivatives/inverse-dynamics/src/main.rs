//! Inverse Dynamics Derivatives — DfDq, DfDv, DfDa Jacobians
//!
//! Demonstrates `mjd_inverse_fd()`: finite-difference Jacobians of the inverse
//! dynamics map `qfrc = M*qacc + bias(qpos, qvel)`. The key insight is that
//! DfDa (derivative w.r.t. acceleration) recovers the mass matrix M, since
//! inverse dynamics is linear in qacc.
//!
//! After a 3-second freeze showing the matrices, the double pendulum is
//! released and swings freely — the HUD continuously re-computes DfDa and
//! shows how the mass matrix evolves with configuration.
//!
//! Run: `cargo run -p example-derivatives-inverse-dynamics --release`

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
    non_snake_case
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{PhysicsHud, render_physics_hud, spawn_example_camera, spawn_physics_hud};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{PhysicsData, PhysicsModel, spawn_model_geoms, sync_geom_transforms};
use sim_core::derivatives::max_relative_error;
use sim_core::validation::{Check, print_report};
use sim_core::{DerivativeConfig, mjd_inverse_fd};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="inverse-dynamics">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="link1" pos="0 0 0.5">
      <joint name="j1" type="hinge" axis="0 1 0" damping="0.005"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.02 0.02 0.001"/>
      <geom name="rod1" type="capsule" size="0.02"
            fromto="0 0 0  0 0 -0.5" rgba="0.48 0.48 0.50 1"/>
      <geom name="elbow" type="sphere" size="0.035"
            pos="0 0 -0.5" rgba="0.6 0.6 0.62 1"/>

      <body name="link2" pos="0 0 -0.5">
        <joint name="j2" type="hinge" axis="0 1 0" damping="0.005"/>
        <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.02 0.02 0.001"/>
        <geom name="rod2" type="capsule" size="0.02"
              fromto="0 0 0  0 0 -0.5" rgba="0.48 0.48 0.50 1"/>
        <geom name="tip" type="sphere" size="0.05"
              pos="0 0 -0.5" rgba="0.9 0.3 0.2 1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"#;

// ── Resources ─────────────────────────────────────────────────────────────

const FREEZE_SECS: f64 = 3.0;

#[derive(Resource)]
struct InvDynResult {
    dfdq: nalgebra::DMatrix<f64>,
    dfdv: nalgebra::DMatrix<f64>,
    dfda: nalgebra::DMatrix<f64>,
    mass_matrix: nalgebra::DMatrix<f64>,
    mass_err: f64,
}

#[derive(Resource)]
struct ReleaseState {
    released: bool,
    accumulator: f64,
}

/// Per-swing error stats for DfDa vs M, using joint 1 velocity zero-crossings.
/// A double pendulum's motion is complex, but joint 1 still has a dominant
/// oscillation — 2 zero-crossings of qvel\[0\] = one full swing.
#[derive(Resource)]
struct SwingStats {
    // Accumulators for current swing
    err_sum: f64,
    max_err: f64,
    sample_count: u32,
    // Displayed values (frozen between swings)
    avg_err: f64,
    peak_err: f64,
    swing_count: u32,
    // Zero-crossing detection on joint 1 velocity
    prev_vel_sign: f64,
    crossings: u32,
    // Latest instantaneous error (for HUD)
    instant_err: f64,
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Inverse Dynamics Derivatives ===");
    println!("  mjd_inverse_fd() — DfDq, DfDv, DfDa Jacobians");
    println!("  2-DOF double pendulum, verifying DfDa = M");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Inverse Dynamics Derivatives".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsHud>()
        .insert_resource(ReleaseState {
            released: false,
            accumulator: 0.0,
        })
        .insert_resource(SwingStats {
            err_sum: 0.0,
            max_err: 0.0,
            sample_count: 0,
            avg_err: 0.0,
            peak_err: 0.0,
            swing_count: 0,
            prev_vel_sign: 0.0,
            crossings: 0,
            instant_err: 0.0,
        })
        .add_systems(Startup, setup)
        .add_systems(Update, gated_step)
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

    // Non-trivial state for meaningful derivatives
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.5;
    data.qvel[0] = 1.0;
    data.qvel[1] = -0.5;
    data.forward(&model).expect("forward");

    println!(
        "  Model: {} bodies, {} joints, nv={}, nu={}",
        model.nbody, model.njnt, model.nv, model.nu
    );
    println!("  qpos = [{:.1}, {:.1}]", data.qpos[0], data.qpos[1]);
    println!("  qvel = [{:.1}, {:.1}]", data.qvel[0], data.qvel[1]);

    // Compute inverse dynamics derivatives
    let config = DerivativeConfig::default();
    let derivs = mjd_inverse_fd(&model, &data, &config).expect("mjd_inverse_fd");

    // Print matrices
    let nv = model.nv;
    println!("\n  DfDq ({nv}x{nv}) — dqfrc/dqpos:");
    print_matrix(&derivs.DfDq);
    println!("  DfDv ({nv}x{nv}) — dqfrc/dqvel:");
    print_matrix(&derivs.DfDv);
    println!("  DfDa ({nv}x{nv}) — dqfrc/dqacc:");
    print_matrix(&derivs.DfDa);

    // Mass matrix comparison
    println!("  M (mass matrix, {nv}x{nv}):");
    print_matrix(&data.qM);

    let (mass_err, mass_loc) = max_relative_error(&derivs.DfDa, &data.qM, 1e-10);
    println!(
        "  DfDa vs M: max_rel_error = {mass_err:.2e} at ({}, {})",
        mass_loc.0, mass_loc.1
    );

    // Validation checks
    let dims_ok = derivs.DfDq.nrows() == nv
        && derivs.DfDq.ncols() == nv
        && derivs.DfDv.nrows() == nv
        && derivs.DfDv.ncols() == nv
        && derivs.DfDa.nrows() == nv
        && derivs.DfDa.ncols() == nv;
    let dfda_matches_m = mass_err < 1e-5;
    let dfq_nonzero = derivs.DfDq.iter().any(|v| v.abs() > 1e-15);
    let dfdv_nonzero = derivs.DfDv.iter().any(|v| v.abs() > 1e-15);

    let checks = vec![
        Check {
            name: "DfDq, DfDv, DfDa each 2x2",
            pass: dims_ok,
            detail: format!(
                "DfDq={}x{}, DfDv={}x{}, DfDa={}x{}",
                derivs.DfDq.nrows(),
                derivs.DfDq.ncols(),
                derivs.DfDv.nrows(),
                derivs.DfDv.ncols(),
                derivs.DfDa.nrows(),
                derivs.DfDa.ncols(),
            ),
        },
        Check {
            name: "DfDa matches M within 1e-5",
            pass: dfda_matches_m,
            detail: format!("max_rel_error = {mass_err:.2e}"),
        },
        Check {
            name: "DfDq not all zeros",
            pass: dfq_nonzero,
            detail: "gravity depends on position".into(),
        },
        Check {
            name: "DfDv not all zeros",
            pass: dfdv_nonzero,
            detail: "Coriolis depends on velocity".into(),
        },
    ];
    let _ = print_report("Inverse Dynamics", &checks);

    // Spawn visuals
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_elbow =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.6, 0.6, 0.62)));
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.3, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("rod1", mat_rod.clone()),
            ("rod2", mat_rod),
            ("elbow", mat_elbow),
            ("tip", mat_tip),
        ],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, -0.25),
        2.2,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(InvDynResult {
        dfdq: derivs.DfDq,
        dfdv: derivs.DfDv,
        dfda: derivs.DfDa,
        mass_matrix: data.qM.clone(),
        mass_err,
    });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

fn print_matrix(m: &nalgebra::DMatrix<f64>) {
    for r in 0..m.nrows() {
        let vals: Vec<String> = (0..m.ncols())
            .map(|c| format!("{:>10.6}", m[(r, c)]))
            .collect();
        println!("    [{}]", vals.join(", "));
    }
}

// ── Gated physics ───────────────────────────────────────────────────────────

fn gated_step(
    time: Res<Time>,
    mut release: ResMut<ReleaseState>,
    mut stats: ResMut<SwingStats>,
    model: Option<Res<PhysicsModel>>,
    data: Option<ResMut<PhysicsData>>,
) {
    if time.elapsed_secs_f64() < FREEZE_SECS {
        return;
    }

    let (Some(model), Some(mut data)) = (model, data) else {
        return;
    };

    if !release.released {
        release.released = true;
    }

    release.accumulator += time.delta_secs_f64();
    while release.accumulator >= model.timestep {
        release.accumulator -= model.timestep;
        data.step(&model).expect("step");
    }

    // Refresh derived quantities so qM matches the current state
    data.forward(&model).expect("forward");

    // Compute DfDa vs M error at current state
    let config = DerivativeConfig::default();
    if let Ok(live) = mjd_inverse_fd(&model, &data, &config) {
        let (err, _) = max_relative_error(&live.DfDa, &data.qM, 1e-10);
        stats.instant_err = err;

        // Accumulate
        stats.err_sum += err;
        if err > stats.max_err {
            stats.max_err = err;
        }
        stats.sample_count += 1;

        // Zero-crossing detection on joint 1 velocity
        let vel_sign = data.qvel[0].signum();
        if stats.prev_vel_sign * vel_sign < 0.0 {
            stats.crossings += 1;
            if stats.crossings >= 4 && stats.sample_count > 0 {
                let n = f64::from(stats.sample_count);
                stats.avg_err = stats.err_sum / n;
                stats.peak_err = stats.max_err;
                stats.swing_count += 1;
                stats.err_sum = 0.0;
                stats.max_err = 0.0;
                stats.sample_count = 0;
                stats.crossings = 0;
            }
        }
        if vel_sign.abs() > 0.0 {
            stats.prev_vel_sign = vel_sign;
        }
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn fmt_row(m: &nalgebra::DMatrix<f64>, row: usize) -> String {
    let vals: Vec<String> = (0..m.ncols())
        .map(|c| format!("{:>9.4}", m[(row, c)]))
        .collect();
    format!("[{}]", vals.join(","))
}

fn update_hud(
    result: Option<Res<InvDynResult>>,
    release: Res<ReleaseState>,
    stats: Res<SwingStats>,
    wall: Res<Time>,
    data: Option<Res<PhysicsData>>,
    mut hud: ResMut<PhysicsHud>,
) {
    let Some(r) = result else { return };

    hud.clear();
    hud.section("Inverse Dynamics Derivatives");
    hud.raw("mjd_inverse_fd() — qfrc = M*qacc + bias".into());
    hud.raw(String::new());

    if !release.released {
        // Show initial linearization matrices during freeze
        hud.raw("DfDq (dqfrc/dqpos) — gravity/Coriolis:".into());
        for row in 0..r.dfdq.nrows() {
            hud.raw(format!("  {}", fmt_row(&r.dfdq, row)));
        }
        hud.raw(String::new());

        hud.raw("DfDv (dqfrc/dqvel) — Coriolis/damping:".into());
        for row in 0..r.dfdv.nrows() {
            hud.raw(format!("  {}", fmt_row(&r.dfdv, row)));
        }
        hud.raw(String::new());

        hud.raw("DfDa (dqfrc/dqacc) — should equal M:".into());
        for row in 0..r.dfda.nrows() {
            hud.raw(format!("  {}", fmt_row(&r.dfda, row)));
        }
        hud.raw(String::new());

        hud.raw("M (mass matrix):".into());
        for row in 0..r.mass_matrix.nrows() {
            hud.raw(format!("  {}", fmt_row(&r.mass_matrix, row)));
        }
        hud.raw(String::new());
        hud.raw(format!("DfDa vs M: err = {:.2e}", r.mass_err));
        hud.raw(String::new());

        let remaining = (FREEZE_SECS - wall.elapsed_secs_f64()).max(0.0);
        hud.section("Frozen at q=[0.3, -0.5]");
        hud.raw(format!("Releasing in {remaining:.1}s"));
    } else if let Some(data) = data {
        // Live instant error
        hud.raw(format!("DfDa vs M (instant): {:.2e}", stats.instant_err));
        hud.raw(if stats.instant_err < 1e-3 {
            "DfDa = M confirmed".into()
        } else {
            "DfDa ~ M (FD noise)".into()
        });
        hud.raw(String::new());

        // Per-swing averages
        if stats.swing_count > 0 {
            hud.section(&format!("Swing #{} (j1 zero-crossings)", stats.swing_count));
            hud.raw(format!("  avg err:  {:.2e}", stats.avg_err));
            hud.raw(format!("  peak err: {:.2e}", stats.peak_err));
        } else {
            hud.raw("Avg error: waiting for first j1 swing...".into());
        }
        hud.raw(String::new());

        hud.scalar("sim time", data.time, 2);
        hud.scalar("energy", data.total_energy(), 4);
        hud.scalar("q1", data.qpos[0], 3);
        hud.scalar("q2", data.qpos[1], 3);
    }
}
