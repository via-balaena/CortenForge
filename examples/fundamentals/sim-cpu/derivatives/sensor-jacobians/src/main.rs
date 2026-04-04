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
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.5"/>
      <inertial pos="0 0 -0.5" mass="5.0" diaginertia="0.5 0.5 0.01"/>
      <geom name="rod" type="capsule" size="0.025"
            fromto="0 0 0  0 0 -0.5" rgba="0.48 0.48 0.50 1"/>
      <geom name="tip" type="sphere" size="0.06"
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
const CTRL_PULSE: f64 = 30.0; // N·m — brief impulse, then coast
const PULSE_DURATION: f64 = 0.5; // seconds — short kick then ctrl=0

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

/// Rolling average of prediction error, updated every full swing cycle.
/// A full swing = 2 velocity zero-crossings (out and back).
#[derive(Resource)]
struct ErrorStats {
    // Accumulators for current swing
    pos_err_sum: f64,
    vel_err_sum: f64,
    sample_count: u32,
    // Displayed values (frozen between swings)
    avg_pos_err: f64,
    avg_vel_err: f64,
    swing_count: u32,
    // Zero-crossing detection
    prev_vel_sign: f64,
    crossings: u32,
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
        .insert_resource(ErrorStats {
            pos_err_sum: 0.0,
            vel_err_sum: 0.0,
            sample_count: 0,
            avg_pos_err: 0.0,
            avg_vel_err: 0.0,
            swing_count: 0,
            prev_vel_sign: 0.0,
            crossings: 0,
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

    // Brief torque pulse, then coast under gravity + damping
    if data.time < PULSE_DURATION {
        data.ctrl[0] = CTRL_PULSE;
    } else {
        data.ctrl[0] = 0.0;
    }
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
    mut stats: ResMut<ErrorStats>,
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

        // Accumulate errors and detect swing cycles via velocity zero-crossings
        stats.pos_err_sum += err_pos;
        stats.vel_err_sum += err_vel;
        stats.sample_count += 1;

        let vel_sign = data.qvel[0].signum();
        let sign_changed = stats.prev_vel_sign * vel_sign < 0.0; // opposite signs
        if sign_changed {
            stats.crossings += 1;
            // 2 crossings = one full swing — update displayed averages
            if stats.crossings >= 2 && stats.sample_count > 0 {
                let n = f64::from(stats.sample_count);
                stats.avg_pos_err = stats.pos_err_sum / n;
                stats.avg_vel_err = stats.vel_err_sum / n;
                stats.swing_count += 1;
                stats.pos_err_sum = 0.0;
                stats.vel_err_sum = 0.0;
                stats.sample_count = 0;
                stats.crossings = 0;
            }
        }
        if vel_sign.abs() > 0.0 {
            stats.prev_vel_sign = vel_sign;
        }

        hud.section("Predicted vs Actual");
        hud.raw("          predicted    actual".into());
        hud.raw(format!("  pos   {pred_pos:>10.4}  {actual_pos:>10.4}"));
        hud.raw(format!("  vel   {pred_vel:>10.4}  {actual_vel:>10.4}"));
        hud.raw(String::new());

        if stats.swing_count > 0 {
            hud.raw(format!("Avg error (swing #{}):", stats.swing_count));
            hud.raw(format!("  pos:  {:.2e}", stats.avg_pos_err));
            hud.raw(format!("  vel:  {:.2e}", stats.avg_vel_err));
        } else {
            hud.raw("Avg error: waiting for first full swing...".into());
        }
        hud.raw(String::new());

        hud.scalar("sim time", data.time, 2);
        hud.scalar("|dx|", dx.norm(), 4);

        if stats.swing_count > 0 && stats.avg_pos_err < 0.01 && stats.avg_vel_err < 0.1 {
            hud.raw("Linear regime — prediction accurate".into());
        } else if stats.swing_count > 0 {
            hud.raw("Nonlinear — prediction diverging".into());
        }
    }
}
