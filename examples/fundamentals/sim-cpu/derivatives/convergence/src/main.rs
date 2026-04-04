//! FD Convergence — Epsilon Tuning and Convergence Rates
//!
//! Demonstrates how `DerivativeConfig.eps` and `DerivativeConfig.centered`
//! affect finite-difference accuracy. Centered differences converge as O(eps²)
//! while forward differences converge as O(eps), meaning centered gives
//! quadratically better accuracy for the same epsilon.
//!
//! The HUD shows a convergence table: error at each epsilon for both methods,
//! confirming the theoretical convergence rates.
//!
//! Run: `cargo run -p example-derivatives-convergence --release`

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
use sim_core::derivatives::{fd_convergence_check, max_relative_error};
use sim_core::validation::{Check, print_report};
use sim_core::{DerivativeConfig, mjd_transition_fd};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="convergence">
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
            pos="0 0 -0.5" rgba="0.3 0.8 0.4 1"/>
    </body>
  </worldbody>

  <actuator>
    <motor name="torque" joint="hinge" gear="1"/>
  </actuator>
</mujoco>
"#;

// ── Convergence data ────────────────────────────────────────────────────────

const EPSILONS: [f64; 5] = [1e-3, 1e-4, 1e-5, 1e-6, 1e-7];

struct ConvergenceRow {
    eps: f64,
    centered_err: f64,
    forward_err: f64,
}

#[derive(Resource)]
struct ConvergenceResult {
    rows: Vec<ConvergenceRow>,
    convergence_ok: bool,
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: FD Convergence ===");
    println!("  eps + centered/forward tuning");
    println!("  1-DOF pendulum with motor actuator");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — FD Convergence".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsHud>()
        .add_systems(Startup, setup)
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

    // Non-zero state for nontrivial derivatives
    data.qpos[0] = 0.8;
    data.qvel[0] = 0.5;
    data.ctrl[0] = 0.3;
    data.forward(&model).expect("forward");

    println!(
        "  Model: {} bodies, {} joints, nv={}, nu={}",
        model.nbody, model.njnt, model.nv, model.nu
    );
    println!(
        "  qpos = {:.1}, qvel = {:.1}, ctrl = {:.1}",
        data.qpos[0], data.qvel[0], data.ctrl[0]
    );

    // Ground truth: smallest epsilon, centered
    let truth_config = DerivativeConfig {
        eps: EPSILONS[4], // 1e-7
        centered: true,
        use_analytical: false,
        compute_sensor_derivatives: false,
    };
    let truth = mjd_transition_fd(&model, &data, &truth_config).expect("ground truth");

    // Compute errors at each epsilon for centered and forward
    println!("\n  {:>8}  {:>12}  {:>12}", "eps", "centered", "forward");
    println!("  {:>8}  {:>12}  {:>12}", "---", "--------", "-------");

    let mut rows = Vec::new();
    for &eps in &EPSILONS[..4] {
        // Skip the last one (ground truth)
        let centered_config = DerivativeConfig {
            eps,
            centered: true,
            use_analytical: false,
            compute_sensor_derivatives: false,
        };
        let forward_config = DerivativeConfig {
            eps,
            centered: false,
            use_analytical: false,
            compute_sensor_derivatives: false,
        };

        let a_centered = mjd_transition_fd(&model, &data, &centered_config).expect("centered");
        let a_forward = mjd_transition_fd(&model, &data, &forward_config).expect("forward");

        let (centered_err, _) = max_relative_error(&a_centered.A, &truth.A, 1e-14);
        let (forward_err, _) = max_relative_error(&a_forward.A, &truth.A, 1e-14);

        println!("  {eps:>8.0e}  {centered_err:>12.4e}  {forward_err:>12.4e}");

        rows.push(ConvergenceRow {
            eps,
            centered_err,
            forward_err,
        });
    }

    // fd_convergence_check at default eps
    let convergence_ok =
        fd_convergence_check(&model, &data, 1e-6, 1e-4).expect("fd_convergence_check");
    println!(
        "\n  fd_convergence_check(eps=1e-6, tol=1e-4): {}",
        if convergence_ok { "PASS" } else { "FAIL" }
    );

    // Validation checks
    let monotone_centered = rows
        .windows(2)
        .all(|w| w[1].centered_err < w[0].centered_err);
    let monotone_forward = rows.windows(2).all(|w| w[1].forward_err < w[0].forward_err);
    let centered_beats_forward = rows.iter().all(|r| r.centered_err < r.forward_err);

    let checks = vec![
        Check {
            name: "Centered monotone convergence",
            pass: monotone_centered,
            detail: "err decreases as eps decreases".into(),
        },
        Check {
            name: "Forward monotone convergence",
            pass: monotone_forward,
            detail: "err decreases as eps decreases".into(),
        },
        Check {
            name: "Centered < forward at every eps",
            pass: centered_beats_forward,
            detail: "centered is strictly more accurate".into(),
        },
        Check {
            name: "fd_convergence_check passes",
            pass: convergence_ok,
            detail: format!(
                "eps=1e-6, tol=1e-4: {}",
                if convergence_ok { "true" } else { "false" }
            ),
        },
    ];
    let _ = print_report("FD Convergence", &checks);

    // Spawn visuals — static pendulum at 0.8 rad
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.8, 0.4)));

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

    commands.insert_resource(ConvergenceResult {
        rows,
        convergence_ok,
    });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(result: Option<Res<ConvergenceResult>>, mut hud: ResMut<PhysicsHud>) {
    let Some(r) = result else { return };

    hud.clear();
    hud.section("FD Convergence");
    hud.raw("How eps and centered affect accuracy".into());
    hud.raw(String::new());

    // Convergence table
    hud.raw("Ground truth: centered, eps=1e-7".into());
    hud.raw(String::new());
    hud.raw(format!(
        "{:>8}  {:>10}  {:>10}",
        "eps", "centered", "forward"
    ));
    hud.raw(format!(
        "{:>8}  {:>10}  {:>10}",
        "---", "--------", "-------"
    ));

    for row in &r.rows {
        hud.raw(format!(
            "{:>8.0e}  {:>10.2e}  {:>10.2e}",
            row.eps, row.centered_err, row.forward_err
        ));
    }
    hud.raw(String::new());

    // Convergence rate analysis
    hud.section("Convergence Rates");

    // Show ratio between consecutive rows
    if r.rows.len() >= 2 {
        hud.raw(format!(
            "{:>8}  {:>10}  {:>10}",
            "10x eps", "centered", "forward"
        ));
        hud.raw(format!("{:>8}  {:>10}  {:>10}", "ratio", "ratio", "ratio"));
        for w in r.rows.windows(2) {
            let c_ratio = if w[1].centered_err > 0.0 {
                w[0].centered_err / w[1].centered_err
            } else {
                f64::INFINITY
            };
            let f_ratio = if w[1].forward_err > 0.0 {
                w[0].forward_err / w[1].forward_err
            } else {
                f64::INFINITY
            };
            hud.raw(format!(
                "{:.0e}->{:.0e}  {:>10.1}  {:>10.1}",
                w[0].eps, w[1].eps, c_ratio, f_ratio
            ));
        }
        hud.raw(String::new());
        hud.raw("Expected: centered ~100x (O(eps^2))".into());
        hud.raw("          forward  ~10x  (O(eps))".into());
    }

    hud.raw(String::new());
    hud.raw(format!(
        "fd_convergence_check: {}",
        if r.convergence_ok { "PASS" } else { "FAIL" }
    ));
}
