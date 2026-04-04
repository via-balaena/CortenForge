//! Control Design — LQR from Linearization
//!
//! Linearizes a cart-pole at the unstable upright equilibrium, solves discrete-
//! time LQR via the Riccati equation, and runs the closed-loop controller.
//! The pole balances — proving that linearization enables control design.
//!
//! Run: `cargo run -p example-derivatives-control-design --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::suboptimal_flops,
    clippy::let_underscore_must_use,
    clippy::too_many_lines,
    clippy::while_float,
    clippy::many_single_char_names,
    non_snake_case
)]

use std::f64::consts::PI;

use bevy::prelude::*;
use nalgebra::DMatrix;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{PhysicsHud, render_physics_hud, spawn_example_camera, spawn_physics_hud};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{PhysicsData, PhysicsModel, spawn_model_geoms, sync_geom_transforms};
use sim_core::validation::{Check, print_report};
use sim_core::{DerivativeConfig, mjd_transition_fd};

// ── MJCF Model ──────────────────────────────────────────────────────────────

/// Cart-pole: slide joint (cart) + hinge joint (pole) + motor on slide.
/// No damping on the hinge — the LQR controller must do all the stabilization.
/// Cart has mild damping for realism.
const MJCF: &str = r#"
<mujoco model="control-design">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <geom name="rail" type="capsule" fromto="-3 0 0 3 0 0"
          size="0.02" rgba="0.5 0.5 0.5 0.6"/>

    <body name="cart" pos="0 0 0">
      <joint name="slider" type="slide" axis="1 0 0" damping="1.0"/>
      <geom name="cart_geom" type="box" size="0.2 0.1 0.08"
            mass="1.0" rgba="0.7 0.7 0.2 1"/>

      <body name="pole" pos="0 0 0">
        <joint name="hinge" type="hinge" axis="0 1 0"/>
        <inertial pos="0 0 0.5" mass="0.5" diaginertia="0.05 0.05 0.001"/>
        <geom name="pole_geom" type="capsule"
              fromto="0 0 0 0 0 1.0" size="0.03"
              rgba="0.2 0.6 0.9 1"/>
        <geom name="tip" type="sphere" size="0.06"
              pos="0 0 1.0" rgba="0.9 0.3 0.2 1"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="cart_force" joint="slider" gear="1"/>
  </actuator>
</mujoco>
"#;

// ── Discrete LQR via DARE iteration ────────────────────────────────────────

/// Solve the discrete algebraic Riccati equation (DARE) and return the
/// optimal LQR gain matrix K.
///
/// DARE iteration:
///   P_{k+1} = Q + A^T P_k A - A^T P_k B (R + B^T P_k B)^{-1} B^T P_k A
///   K = (R + B^T P B)^{-1} B^T P A
fn dlqr(a: &DMatrix<f64>, b: &DMatrix<f64>, q: &DMatrix<f64>, r: &DMatrix<f64>) -> DMatrix<f64> {
    let at = a.transpose();
    let bt = b.transpose();
    let mut p = q.clone();

    for _ in 0..1000 {
        let bp = &bt * &p;
        let bpb_r = &bp * b + r; // nu×nu
        let bpb_r_inv = bpb_r
            .clone()
            .try_inverse()
            .expect("R + B^T P B should be invertible");
        let p_new = q + &at * &p * a - &at * &p * b * &bpb_r_inv * &bp * a;

        let diff = (&p_new - &p).norm();
        p = p_new;
        if diff < 1e-12 {
            break;
        }
    }

    let bp = &bt * &p;
    let bpb_r = &bp * b + r;
    let bpb_r_inv = bpb_r
        .try_inverse()
        .expect("R + B^T P B should be invertible");
    bpb_r_inv * bp * a
}

// ── Resources ─────────────────────────────────────────────────────────────

const FREEZE_SECS: f64 = 3.0;
const INITIAL_TILT: f64 = 0.2; // ~11° from upright

#[derive(Resource)]
struct LqrResult {
    a: DMatrix<f64>,
    b: DMatrix<f64>,
    k: DMatrix<f64>,
}

#[derive(Resource)]
struct ReleaseState {
    released: bool,
    accumulator: f64,
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Control Design ===");
    println!("  LQR from linearization (mjd_transition_fd → DARE → K)");
    println!("  Cart-pole balancing at unstable upright");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Control Design (LQR)".into(),
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
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();

    // Upright equilibrium: qpos[1] = 0 (pole geom points +Z, so 0 = up)
    data.qpos[1] = 0.0;
    data.forward(&model).expect("forward");

    println!(
        "  Model: {} bodies, {} joints, {} actuators, nv={}",
        model.nbody, model.njnt, model.nu, model.nv
    );

    // Compute transition derivatives at upright
    let config = DerivativeConfig {
        use_analytical: false,
        ..Default::default()
    };
    let derivs = mjd_transition_fd(&model, &data, &config).expect("mjd_transition_fd");
    let a = &derivs.A;
    let b = &derivs.B;

    println!(
        "  A: {}x{}, B: {}x{}",
        a.nrows(),
        a.ncols(),
        b.nrows(),
        b.ncols()
    );

    // LQR cost: penalize pole angle heavily, cart position mildly, control cheap
    let nx = a.nrows();
    let nu = b.ncols();
    let mut q = DMatrix::zeros(nx, nx);
    q[(0, 0)] = 1.0; // cart position
    q[(1, 1)] = 100.0; // pole angle (most important)
    q[(2, 2)] = 1.0; // cart velocity
    q[(3, 3)] = 10.0; // pole angular velocity
    let r = DMatrix::from_element(nu, nu, 0.01);

    let k = dlqr(a, b, &q, &r);
    println!(
        "  K gains: [{:.4}, {:.4}, {:.4}, {:.4}]",
        k[0], k[1], k[2], k[3]
    );

    // ── Headless validation checks ──────────────────────────────────────

    // Check 1: dimensions
    let dim_ok = a.nrows() == 4 && a.ncols() == 4 && b.nrows() == 4 && b.ncols() == 1;

    // Check 2: 500 closed-loop steps, pole stays within ±0.1 rad
    let mut d2 = model.make_data();
    d2.qpos[1] = 0.05; // small tilt from upright (0)
    d2.forward(&model).expect("forward");
    let mut balanced = true;
    for _ in 0..500 {
        let x_err = state_error(&d2);
        d2.ctrl[0] = lqr_control(&k, &x_err)[0];
        d2.step(&model).expect("step");
        if d2.qpos[1].abs() > 0.1 {
            balanced = false;
            break;
        }
    }

    // Check 3: K has 4 elements
    let k_size_ok = k.len() == 4;

    // Check 4: without control, pole exceeds π/4 within 100 steps
    let mut d3 = model.make_data();
    d3.qpos[1] = 0.05; // small tilt from upright (0)
    d3.forward(&model).expect("forward");
    let mut fell = false;
    for _ in 0..100 {
        d3.ctrl[0] = 0.0;
        d3.step(&model).expect("step");
        if d3.qpos[1].abs() > PI / 4.0 {
            fell = true;
            break;
        }
    }

    let checks = vec![
        Check {
            name: "A is 4x4, B is 4x1",
            pass: dim_ok,
            detail: format!(
                "A={}x{}, B={}x{}",
                a.nrows(),
                a.ncols(),
                b.nrows(),
                b.ncols()
            ),
        },
        Check {
            name: "Balanced after 500 steps",
            pass: balanced,
            detail: "pole within +/-0.1 rad of upright".into(),
        },
        Check {
            name: "K has 4 elements",
            pass: k_size_ok,
            detail: format!("len={}", k.len()),
        },
        Check {
            name: "Falls without control",
            pass: fell,
            detail: "pole exceeds pi/4 within 100 steps (no ctrl)".into(),
        },
    ];
    let _ = print_report("Control Design", &checks);

    // ── Spawn visuals (start at upright for freeze) ─────────────────────

    data.qpos[0] = 0.0;
    data.qpos[1] = 0.0; // upright
    data.qvel[0] = 0.0;
    data.qvel[1] = 0.0;
    data.forward(&model).expect("forward");

    let mat_cart = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.7, 0.7, 0.2)));
    let mat_pole = materials.add(MetalPreset::BrushedMetal.with_color(Color::srgb(0.2, 0.6, 0.9)));
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.3, 0.2)));
    let mat_rail = materials.add(MetalPreset::BrushedMetal.material());

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("cart_geom", mat_cart),
            ("pole_geom", mat_pole),
            ("tip", mat_tip),
            ("rail", mat_rail),
        ],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.5),
        4.0,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(LqrResult {
        a: a.clone(),
        b: b.clone(),
        k,
    });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── State error ─────────────────────────────────────────────────────────────

/// Compute state error relative to upright equilibrium (qpos\[1\] = 0).
/// x_err = [cart_pos, pole_angle, cart_vel, pole_vel]
fn state_error(data: &sim_core::Data) -> nalgebra::DVector<f64> {
    nalgebra::DVector::from_vec(vec![data.qpos[0], data.qpos[1], data.qvel[0], data.qvel[1]])
}

/// Compute control vector u = -K * x_err.
/// K is nu×nx, x_err is nx×1, result is nu×1.
fn lqr_control(k: &DMatrix<f64>, x_err: &nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
    -(k * x_err)
}

// ── Gated physics with LQR control ─────────────────────────────────────────

fn gated_step(
    time: Res<Time>,
    mut release: ResMut<ReleaseState>,
    lqr: Option<Res<LqrResult>>,
    model: Option<Res<PhysicsModel>>,
    data: Option<ResMut<PhysicsData>>,
) {
    if time.elapsed_secs_f64() < FREEZE_SECS {
        return;
    }

    let (Some(lqr), Some(model), Some(mut data)) = (lqr, model, data) else {
        return;
    };

    // On first frame after freeze, apply initial tilt
    if !release.released {
        release.released = true;
        data.qpos[1] = INITIAL_TILT;
        data.forward(&model).expect("forward after tilt");
    }

    // Step physics with LQR control
    release.accumulator += time.delta_secs_f64();
    while release.accumulator >= model.timestep {
        release.accumulator -= model.timestep;
        let x_err = state_error(&data);
        data.ctrl[0] = lqr_control(&lqr.k, &x_err)[0];
        data.step(&model).expect("step");
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(
    lqr: Option<Res<LqrResult>>,
    release: Res<ReleaseState>,
    wall: Res<Time>,
    data: Option<Res<PhysicsData>>,
    mut hud: ResMut<PhysicsHud>,
) {
    let Some(lqr) = lqr else { return };

    hud.clear();
    hud.section("Control Design (LQR)");
    hud.raw(format!(
        "A: {}x{}  B: {}x{}",
        lqr.a.nrows(),
        lqr.a.ncols(),
        lqr.b.nrows(),
        lqr.b.ncols()
    ));
    hud.raw(String::new());

    hud.raw("K gains (state -> force):".into());
    hud.raw(format!("  cart_pos:  {:>8.4}", lqr.k[0]));
    hud.raw(format!("  pole_ang:  {:>8.4}", lqr.k[1]));
    hud.raw(format!("  cart_vel:  {:>8.4}", lqr.k[2]));
    hud.raw(format!("  pole_vel:  {:>8.4}", lqr.k[3]));
    hud.raw(String::new());

    if !release.released {
        let remaining = (FREEZE_SECS - wall.elapsed_secs_f64()).max(0.0);
        hud.section("Frozen at upright");
        hud.raw(format!(
            "Releasing in {remaining:.1}s (+{INITIAL_TILT} rad tilt)"
        ));
    } else if let Some(d) = data {
        let x_err = state_error(&d);
        let err_norm = x_err.norm();
        let pole_dev = d.qpos[1].abs();
        let ctrl = lqr_control(&lqr.k, &x_err)[0];

        hud.section("LQR active");
        hud.scalar("sim time", d.time, 2);
        hud.scalar("pole dev", pole_dev, 4);
        hud.scalar("|x_err|", err_norm, 4);
        hud.scalar("ctrl force", ctrl, 2);
        hud.scalar("cart pos", d.qpos[0], 3);
    }
}
