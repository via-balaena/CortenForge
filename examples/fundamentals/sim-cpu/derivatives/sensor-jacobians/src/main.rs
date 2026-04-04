//! Sensor Jacobians — C, D Matrices from Transition Derivatives
//!
//! Demonstrates `compute_sensor_derivatives: true` which produces C and D
//! matrices alongside A and B. After a 3-second freeze showing the matrices,
//! a torque pulse is applied and the pendulum swings. The HUD compares
//! predicted sensor changes (C*dx + D*du) against actual sensor readings —
//! they match near the linearization point, then diverge as nonlinearity
//! takes over.
//!
//! Run: `cargo run -p example-derivatives-sensor-jacobians --release`

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
use nalgebra::DVector;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{PhysicsHud, render_physics_hud, spawn_example_camera, spawn_physics_hud};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{PhysicsData, PhysicsModel, spawn_model_geoms, sync_geom_transforms};
use sim_core::validation::{Check, print_report};
use sim_core::{DerivativeConfig, mjd_transition_fd};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="sensor-jacobians">
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
            pos="0 0 -0.5" rgba="0.9 0.5 0.1 1"/>
    </body>
  </worldbody>

  <actuator>
    <motor name="torque" joint="hinge" gear="1"/>
  </actuator>

  <sensor>
    <jointpos name="pos" joint="hinge"/>
    <jointvel name="vel" joint="hinge"/>
  </sensor>
</mujoco>
"#;

// ── Resources ─────────────────────────────────────────────────────────────

const FREEZE_SECS: f64 = 3.0;
const CTRL_PULSE: f64 = 2.0; // N·m — noticeable torque

#[derive(Resource)]
struct SensorResult {
    c: nalgebra::DMatrix<f64>,
    d: nalgebra::DMatrix<f64>,
    // Baseline at linearization point
    x0: DVector<f64>,  // [qpos0, qvel0]
    u0: f64,           // ctrl at linearization
    sensor0: [f64; 2], // [pos0, vel0] sensor readings
}

#[derive(Resource)]
struct ReleaseState {
    released: bool,
    accumulator: f64,
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Sensor Jacobians ===");
    println!("  C, D matrices — predicted vs actual sensor response");
    println!("  1-DOF pendulum with jointpos + jointvel sensors");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Sensor Jacobians".into(),
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
    data.qpos[0] = 0.5;
    data.ctrl[0] = 0.1;
    data.forward(&model).expect("forward");

    println!(
        "  Model: {} bodies, {} joints, {} actuators, {} sensors, nsensordata={}",
        model.nbody, model.njnt, model.nu, model.nsensor, model.nsensordata
    );

    // Compute derivatives WITH sensor derivatives
    let config = DerivativeConfig {
        use_analytical: false,
        compute_sensor_derivatives: true,
        ..Default::default()
    };
    let derivs = mjd_transition_fd(&model, &data, &config).expect("mjd_transition_fd");

    let c = derivs.C.as_ref().expect("C should be Some");
    let d = derivs.D.as_ref().expect("D should be Some");

    println!("\n  C ({}x{}) — dsensor/dstate:", c.nrows(), c.ncols());
    let sensor_names = ["pos", "vel"];
    for r in 0..c.nrows() {
        let vals: Vec<String> = (0..c.ncols())
            .map(|col| format!("{:>10.6}", c[(r, col)]))
            .collect();
        println!("    {:<4} [{}]", sensor_names[r], vals.join(", "));
    }

    println!("\n  D ({}x{}) — dsensor/dctrl:", d.nrows(), d.ncols());
    for r in 0..d.nrows() {
        println!("    {:<4} [{:>10.6}]", sensor_names[r], d[(r, 0)]);
    }

    // Baseline state and sensors at linearization point
    let x0 = DVector::from_vec(vec![data.qpos[0], data.qvel[0]]);
    let u0 = data.ctrl[0];
    let sensor0 = [
        data.sensor_scalar(&model, "pos").unwrap_or(0.0),
        data.sensor_scalar(&model, "vel").unwrap_or(0.0),
    ];

    // Validation checks
    let c_some = derivs.C.is_some();
    let d_some = derivs.D.is_some();
    let c_dims_ok = c.nrows() == 2 && c.ncols() == 2;
    let d_dims_ok = d.nrows() == 2 && d.ncols() == 1;
    let c_nonzero = c.iter().any(|v| v.abs() > 1e-15);
    let d_nonzero = d.iter().any(|v| v.abs() > 1e-15);

    let checks = vec![
        Check {
            name: "C is Some",
            pass: c_some,
            detail: format!("C={}", if c_some { "Some" } else { "None" }),
        },
        Check {
            name: "C is 2x2",
            pass: c_dims_ok,
            detail: format!("{}x{} (nsensordata x 2*nv)", c.nrows(), c.ncols()),
        },
        Check {
            name: "D is 2x1",
            pass: d_some && d_dims_ok,
            detail: format!("{}x{} (nsensordata x nu)", d.nrows(), d.ncols()),
        },
        Check {
            name: "C not all zeros",
            pass: c_nonzero,
            detail: "sensors respond to state".into(),
        },
        Check {
            name: "D not all zeros",
            pass: d_nonzero,
            detail: "sensors respond to control".into(),
        },
    ];
    let _ = print_report("Sensor Jacobians", &checks);

    // Spawn visuals
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.5, 0.1)));

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

    commands.insert_resource(SensorResult {
        c: c.clone(),
        d: d.clone(),
        x0,
        u0,
        sensor0,
    });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Gated physics ───────────────────────────────────────────────────────────

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

    if !release.released {
        release.released = true;
    }

    // Apply torque pulse and step
    data.ctrl[0] = CTRL_PULSE;
    release.accumulator += time.delta_secs_f64();
    while release.accumulator >= model.timestep {
        release.accumulator -= model.timestep;
        data.step(&model).expect("step");
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(
    sr: Option<Res<SensorResult>>,
    release: Res<ReleaseState>,
    wall: Res<Time>,
    model: Option<Res<PhysicsModel>>,
    data: Option<Res<PhysicsData>>,
    mut hud: ResMut<PhysicsHud>,
) {
    let Some(r) = sr else { return };

    hud.clear();
    hud.section("Sensor Jacobians");
    hud.raw(String::new());

    // Always show C and D
    let sensor_names = ["pos", "vel"];

    hud.raw(format!(
        "C ({}x{}) dsensor/dstate:",
        r.c.nrows(),
        r.c.ncols()
    ));
    hud.raw("          dq        qvel".into());
    for row in 0..r.c.nrows() {
        let vals: Vec<String> = (0..r.c.ncols())
            .map(|c| format!("{:>10.6}", r.c[(row, c)]))
            .collect();
        hud.raw(format!("  {:<4} [{}]", sensor_names[row], vals.join(",")));
    }
    hud.raw(String::new());

    hud.raw(format!(
        "D ({}x{}) dsensor/dctrl:",
        r.d.nrows(),
        r.d.ncols()
    ));
    for row in 0..r.d.nrows() {
        hud.raw(format!(
            "  {:<4} [{:>10.6}]",
            sensor_names[row],
            r.d[(row, 0)]
        ));
    }
    hud.raw(String::new());

    // Countdown or live comparison
    if !release.released {
        let remaining = (FREEZE_SECS - wall.elapsed_secs_f64()).max(0.0);
        hud.section("Frozen at q=0.5, ctrl=0.1");
        hud.raw(format!(
            "Releasing in {remaining:.1}s (ctrl={CTRL_PULSE} N·m pulse)"
        ));
    } else if let (Some(model), Some(data)) = (model, data) {
        // Current state deviation from linearization point
        let dx = DVector::from_vec(vec![data.qpos[0] - r.x0[0], data.qvel[0] - r.x0[1]]);
        let du = DVector::from_vec(vec![data.ctrl[0] - r.u0]);

        // Predicted sensor = sensor0 + C*dx + D*du
        let dy_pred = &r.c * &dx + &r.d * &du;
        let pred_pos = r.sensor0[0] + dy_pred[0];
        let pred_vel = r.sensor0[1] + dy_pred[1];

        // Actual sensor readings
        let actual_pos = data.sensor_scalar(&model, "pos").unwrap_or(0.0);
        let actual_vel = data.sensor_scalar(&model, "vel").unwrap_or(0.0);

        let err_pos = (pred_pos - actual_pos).abs();
        let err_vel = (pred_vel - actual_vel).abs();

        hud.section("Predicted vs Actual");
        hud.raw("          predicted    actual      error".into());
        hud.raw(format!(
            "  pos   {pred_pos:>10.4}  {actual_pos:>10.4}  {err_pos:>10.2e}"
        ));
        hud.raw(format!(
            "  vel   {pred_vel:>10.4}  {actual_vel:>10.4}  {err_vel:>10.2e}"
        ));
        hud.raw(String::new());
        hud.scalar("sim time", data.time, 2);
        hud.scalar("|dx|", dx.norm(), 4);

        if dx.norm() < 0.05 {
            hud.raw("Linear regime — prediction accurate".into());
        } else {
            hud.raw("Nonlinear — prediction diverging".into());
        }
    }
}
