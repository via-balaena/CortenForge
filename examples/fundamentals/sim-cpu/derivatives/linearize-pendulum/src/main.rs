//! Linearize Pendulum — Pure FD Transition Derivatives
//!
//! The simplest derivative call: `mjd_transition_fd()` on a 1-DOF pendulum at
//! the unstable upright equilibrium. The A matrix reveals instability through
//! its eigenvalues (|λ| > 1 in discrete time).
//!
//! After computing the linearization at exact upright, a tiny perturbation
//! (0.01 rad) is applied and the simulation runs — the pendulum falls,
//! proving the instability that the eigenvalues predict.
//!
//! Run: `cargo run -p example-derivatives-linearize-pendulum --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::suboptimal_flops,
    clippy::let_underscore_must_use,
    clippy::too_many_lines,
    clippy::while_float,
    non_snake_case
)]

use std::f64::consts::PI;

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{PhysicsHud, render_physics_hud, spawn_example_camera, spawn_physics_hud};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{PhysicsData, PhysicsModel, spawn_model_geoms, sync_geom_transforms};
use sim_core::validation::{Check, print_report};
use sim_core::{DerivativeConfig, mjd_transition_fd};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="linearize-pendulum">
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
            pos="0 0 -0.5" rgba="0.2 0.6 0.9 1"/>
    </body>
  </worldbody>
</mujoco>
"#;

// ── Eigenvalue helpers ────────────────────────────────────────────────────

/// Eigenvalues of a 2×2 matrix via the quadratic formula.
/// Returns (λ1, λ2) as complex numbers (real, imag).
fn eigenvalues_2x2(mat: &nalgebra::DMatrix<f64>) -> [(f64, f64); 2] {
    assert_eq!(mat.nrows(), 2);
    assert_eq!(mat.ncols(), 2);
    let m00 = mat[(0, 0)];
    let m01 = mat[(0, 1)];
    let m10 = mat[(1, 0)];
    let m11 = mat[(1, 1)];
    let trace = m00 + m11;
    let det = m00 * m11 - m01 * m10;
    let disc = trace * trace - 4.0 * det;
    if disc >= 0.0 {
        let sqrt_d = disc.sqrt();
        [
            (f64::midpoint(trace, sqrt_d), 0.0),
            (f64::midpoint(trace, -sqrt_d), 0.0),
        ]
    } else {
        let sqrt_d = (-disc).sqrt();
        [
            (f64::midpoint(trace, 0.0), sqrt_d / 2.0),
            (f64::midpoint(trace, 0.0), -sqrt_d / 2.0),
        ]
    }
}

fn complex_mag(c: (f64, f64)) -> f64 {
    c.0.hypot(c.1)
}

fn fmt_complex(c: (f64, f64)) -> String {
    if c.1.abs() < 1e-10 {
        format!("{:.6}", c.0)
    } else if c.1 > 0.0 {
        format!("{:.6} + {:.6}i", c.0, c.1)
    } else {
        format!("{:.6} - {:.6}i", c.0, c.1.abs())
    }
}

// ── Resources ─────────────────────────────────────────────────────────────

const FREEZE_SECS: f64 = 5.0;

#[derive(Resource)]
struct DerivResult {
    a: nalgebra::DMatrix<f64>,
    b: nalgebra::DMatrix<f64>,
    eigs: [(f64, f64); 2],
    has_c: bool,
    has_d: bool,
}

/// Tracks freeze/release state and physics time accumulator.
#[derive(Resource)]
struct ReleaseState {
    released: bool,
    accumulator: f64,
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Linearize Pendulum ===");
    println!("  Pure FD transition derivatives (mjd_transition_fd)");
    println!("  1-DOF pendulum at unstable upright equilibrium (qpos = pi)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Linearize Pendulum".into(),
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
    // Build model and set upright equilibrium
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();
    data.qpos[0] = PI;
    data.forward(&model).expect("forward");

    println!(
        "  Model: {} bodies, {} joints, {} actuators, nv={}",
        model.nbody, model.njnt, model.nu, model.nv
    );

    // Compute pure FD derivatives
    let config = DerivativeConfig {
        use_analytical: false,
        ..Default::default()
    };
    let derivs = mjd_transition_fd(&model, &data, &config).expect("mjd_transition_fd");

    let a = &derivs.A;
    let b = &derivs.B;

    println!("\n  A matrix ({}x{}):", a.nrows(), a.ncols());
    for r in 0..a.nrows() {
        print!("    [");
        for c in 0..a.ncols() {
            if c > 0 {
                print!(", ");
            }
            print!("{:>12.6}", a[(r, c)]);
        }
        println!("]");
    }

    println!("  B matrix ({}x{})", b.nrows(), b.ncols());

    let eigs = eigenvalues_2x2(a);
    println!("\n  Eigenvalues:");
    println!(
        "    λ1 = {}  (|λ1| = {:.6})",
        fmt_complex(eigs[0]),
        complex_mag(eigs[0])
    );
    println!(
        "    λ2 = {}  (|λ2| = {:.6})",
        fmt_complex(eigs[1]),
        complex_mag(eigs[1])
    );

    let max_mag = complex_mag(eigs[0]).max(complex_mag(eigs[1]));
    let stable = max_mag <= 1.0;
    println!(
        "\n  Stability: {} (max |λ| = {:.6})",
        if stable { "STABLE" } else { "UNSTABLE" },
        max_mag
    );

    // Run validation checks
    let checks = vec![
        Check {
            name: "A is 2x2",
            pass: a.nrows() == 2 && a.ncols() == 2,
            detail: format!("{}x{}", a.nrows(), a.ncols()),
        },
        Check {
            name: "B is 2x0 (no actuators)",
            pass: b.nrows() == 2 && b.ncols() == 0,
            detail: format!("{}x{}", b.nrows(), b.ncols()),
        },
        Check {
            name: "Unstable (|lambda| > 1)",
            pass: max_mag > 1.0,
            detail: format!("max |lambda| = {max_mag:.6}"),
        },
        Check {
            name: "C, D are None",
            pass: derivs.C.is_none() && derivs.D.is_none(),
            detail: format!(
                "C={}, D={}",
                if derivs.C.is_some() { "Some" } else { "None" },
                if derivs.D.is_some() { "Some" } else { "None" }
            ),
        },
    ];
    let _ = print_report("Linearize Pendulum", &checks);

    // Spawn visuals (pendulum starts at exact upright — frozen for 5s)
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.6, 0.9)));

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
        Vec3::new(0.0, 0.0, 0.25),
        1.8,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );

    spawn_physics_hud(&mut commands);

    // Store results for HUD
    commands.insert_resource(DerivResult {
        a: a.clone(),
        b: b.clone(),
        eigs,
        has_c: derivs.C.is_some(),
        has_d: derivs.D.is_some(),
    });

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Gated physics ───────────────────────────────────────────────────────────

/// Freeze for 5 seconds, then nudge +0.01 rad and run physics.
fn gated_step(
    time: Res<Time>,
    mut release: ResMut<ReleaseState>,
    model: Option<Res<PhysicsModel>>,
    data: Option<ResMut<PhysicsData>>,
) {
    if time.elapsed_secs_f64() < FREEZE_SECS {
        return;
    }

    let (Some(model), Some(mut data)) = (model, data) else {
        return;
    };

    // On first frame after freeze, apply the nudge
    if !release.released {
        release.released = true;
        data.qpos[0] = PI + 0.01;
        data.forward(&model).expect("forward after nudge");
    }

    // Step physics in real time
    release.accumulator += time.delta_secs_f64();
    while release.accumulator >= model.timestep {
        release.accumulator -= model.timestep;
        data.step(&model).expect("step");
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(
    result: Option<Res<DerivResult>>,
    release: Res<ReleaseState>,
    wall: Res<Time>,
    data: Option<Res<PhysicsData>>,
    mut hud: ResMut<PhysicsHud>,
) {
    let Some(r) = result else { return };

    hud.clear();
    hud.section("Linearize Pendulum");
    hud.raw("mjd_transition_fd() at upright (q=pi)".into());
    hud.raw(String::new());

    // A matrix
    hud.raw(format!("A ({}x{}):", r.a.nrows(), r.a.ncols()));
    for row in 0..r.a.nrows() {
        let vals: Vec<String> = (0..r.a.ncols())
            .map(|c| format!("{:>10.6}", r.a[(row, c)]))
            .collect();
        hud.raw(format!("  [{}]", vals.join(", ")));
    }
    hud.raw(String::new());

    // B matrix
    hud.raw(format!("B ({}x{}): no actuators", r.b.nrows(), r.b.ncols()));
    hud.raw(String::new());

    // Eigenvalues
    hud.raw("Eigenvalues:".into());
    hud.raw(format!("  l1 = {}", fmt_complex(r.eigs[0])));
    hud.raw(format!("       |l1| = {:.6}", complex_mag(r.eigs[0])));
    hud.raw(format!("  l2 = {}", fmt_complex(r.eigs[1])));
    hud.raw(format!("       |l2| = {:.6}", complex_mag(r.eigs[1])));
    hud.raw(String::new());

    // Stability
    let max_mag = complex_mag(r.eigs[0]).max(complex_mag(r.eigs[1]));
    let verdict = if max_mag > 1.0 { "UNSTABLE" } else { "STABLE" };
    hud.raw(format!("Stability: {verdict} (max |l| = {max_mag:.6})"));
    hud.raw(String::new());

    // C, D
    hud.raw(format!(
        "C: {}  D: {}",
        if r.has_c { "Some" } else { "None" },
        if r.has_d { "Some" } else { "None" },
    ));
    hud.raw(String::new());

    // Countdown or live state
    if !release.released {
        let remaining = (FREEZE_SECS - wall.elapsed_secs_f64()).max(0.0);
        hud.section("Frozen at upright");
        hud.raw(format!("Releasing in {remaining:.1}s (+0.01 rad nudge)"));
    } else if let Some(d) = data {
        let theta = d.qpos[0];
        let omega = d.qvel[0];
        let deviation = (theta - PI).abs();
        hud.section("Released (+0.01 rad nudge)");
        hud.scalar("sim time", d.time, 2);
        hud.scalar("theta", theta, 4);
        hud.scalar("deviation", deviation, 4);
        hud.scalar("omega", omega, 4);
        if deviation > 0.1 {
            hud.raw("Fell — eigenvalue was right".into());
        }
    }
}
