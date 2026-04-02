//! Frame Acceleration Sensors
//!
//! FrameLinAcc and FrameAngAcc on a pendulum arm. A strong position servo
//! holds the arm horizontal for 5 seconds, then releases it to swing freely.
//!
//! Key insight: FrameLinAcc includes gravity as pseudo-acceleration — a body
//! at rest reads [0, 0, +9.81], identical to a real accelerometer.
//!
//! Validates:
//! - Rest: FrameLinAcc ≈ [0, 0, +g] (gravity pseudo-acceleration)
//! - Rest: FrameAngAcc ≈ 0 (no angular acceleration)
//! - Swing: AngAcc sign flips with sin(θ)
//! - Swing: |LinAcc| > g at bottom (centripetal adds to gravity)
//!
//! Run with: `cargo run -p example-sensor-adv-frame-acceleration --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::cast_precision_loss
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
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

// ── MJCF Model ──────────────────────────────────────────────────────────────
//
// Pendulum arm on a Y-axis hinge. A pure torque motor provides manual PD
// control during the hold phase, then goes to zero for a clean release.
// Low joint damping (0.05) allows a good free swing.
//
const MJCF: &str = r#"
<mujoco model="frame-acceleration">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001">
        <flag energy="enable"/>
    </option>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <body name="arm" pos="0 0 1.5">
            <joint name="hinge" type="hinge" axis="0 1 0" damping="0.05"/>
            <geom name="rod" type="capsule" size="0.025"
                  fromto="0 0 0 1.0 0 0" rgba="0.48 0.48 0.50 1"/>
            <geom name="tip_ball" type="sphere" size="0.05"
                  pos="1.0 0 0" rgba="0.85 0.3 0.2 1"/>
            <site name="tip" pos="1.0 0 0"/>
        </body>
    </worldbody>

    <actuator>
        <motor name="motor" joint="hinge" gear="1"/>
    </actuator>

    <sensor>
        <framelinacc name="tip_linacc" objtype="site" objname="tip"/>
        <frameangacc name="tip_angacc" objtype="site" objname="tip"/>
        <jointpos name="theta" joint="hinge"/>
    </sensor>
</mujoco>
"#;

const G: f64 = 9.81;
const RELEASE_TIME: f64 = 5.0;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Frame Acceleration Sensors ===");
    println!("  FrameLinAcc + FrameAngAcc on a pendulum arm");
    println!("  Phase 1 (0–5s): Motor holds horizontal — a = [0,0,+g]");
    println!("  Phase 2 (5–15s): Released — swings as pendulum");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Frame Acceleration".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<SensorValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let a = sensor_vec3_or_zero(d, m, "tip_linacc");
                    let a_mag = vec3_norm(&a);
                    let alpha = sensor_vec3_or_zero(d, m, "tip_angacc");
                    let phase = if d.time < RELEASE_TIME {
                        "HOLD"
                    } else {
                        "SWING"
                    };
                    format!("{phase}  |a|={a_mag:.4}  alpha_y={:.4}", alpha[1])
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (drive_motor, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                sensor_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let data = model.make_data();

    println!(
        "  Model: {} bodies, {} joints, {} sensors\n",
        model.nbody, model.njnt, model.nsensor
    );

    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.3, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("rod", mat_rod), ("tip_ball", mat_tip)],
    );

    // Camera from the side, slightly elevated, wide enough to see the full arm
    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 1.5, 0.0), // target at hinge height (Bevy Y-up)
        3.5,
        std::f32::consts::FRAC_PI_4,
        0.3,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Motor Drive ─────────────────────────────────────────────────────────────

fn drive_motor(mut data: ResMut<PhysicsData>, model: Res<PhysicsModel>) {
    if data.time < RELEASE_TIME {
        // Manual PD control: hold at θ = 0 (horizontal)
        let jid = model.joint_id("hinge").unwrap_or(0);
        let theta = data.qpos[model.jnt_qpos_adr[jid]];
        let omega = data.qvel[model.jnt_dof_adr[jid]];
        let kp = 5000.0;
        let kd = 100.0;
        let torque = -kp * theta - kd * omega;
        data.set_ctrl(0, torque);
    } else {
        // Clean release: zero torque, arm swings freely under gravity
        data.set_ctrl(0, 0.0);
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    let a = sensor_vec3_or_zero(&data, &model, "tip_linacc");
    let alpha = sensor_vec3_or_zero(&data, &model, "tip_angacc");
    let a_mag = vec3_norm(&a);
    let theta = data.sensor_scalar(&model, "theta").unwrap_or(0.0);

    let phase = if data.time < RELEASE_TIME {
        "HOLD (motor)"
    } else {
        "SWING (free)"
    };

    hud.clear();
    hud.section("Frame Acceleration");
    hud.raw(format!("  phase      {phase}"));
    hud.raw(String::new());
    hud.vec3("a (linacc)", &a, 3);
    hud.scalar("|a|", a_mag, 3);
    hud.scalar("g", G, 3);
    hud.raw(String::new());
    hud.vec3("alpha (angacc)", &alpha, 3);
    hud.scalar("theta", theta, 3);
}

// ── Sensor Validation ───────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SensorValidation {
    // Rest phase (t=1–4s): accumulate stats
    rest_max_az_err: f64,
    rest_max_alpha_mag: f64,
    rest_samples: u64,
    // Swing phase (t>6s): check centripetal and sign flips
    swing_max_a_mag: f64,
    sign_flip_violations: u32,
    swing_samples: u64,
    reported: bool,
}

fn sensor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SensorValidation>,
) {
    let a = sensor_vec3_or_zero(&data, &model, "tip_linacc");
    let alpha = sensor_vec3_or_zero(&data, &model, "tip_angacc");
    let a_mag = vec3_norm(&a);
    let alpha_mag = vec3_norm(&alpha);
    let theta = data.sensor_scalar(&model, "theta").unwrap_or(0.0);

    // Rest phase: t = 1–4s (skip first second for settling)
    if data.time > 1.0 && data.time < 4.0 {
        let az_err = (a[2] - G).abs();
        val.rest_max_az_err = val.rest_max_az_err.max(az_err);
        val.rest_max_alpha_mag = val.rest_max_alpha_mag.max(alpha_mag);
        val.rest_samples += 1;
    }

    // Swing phase: t > 6s (skip 1s after release for transient)
    if data.time > 6.0 {
        val.swing_max_a_mag = val.swing_max_a_mag.max(a_mag);

        // For a Y-axis hinge with arm at angle θ from horizontal:
        // gravity torque τ_Y = mg*(L/2)*cos(θ), so α_Y ∝ cos(θ).
        // Only check when |cos(θ)| is large enough to be meaningful.
        let cos_theta = theta.cos();
        if cos_theta.abs() > 0.1 && alpha[1].abs() > 0.1 && (alpha[1] > 0.0) != (cos_theta > 0.0) {
            val.sign_flip_violations += 1;
        }
        val.swing_samples += 1;
    }

    if harness.reported() && !val.reported {
        val.reported = true;

        let checks = vec![
            Check {
                name: "Rest: a_z = +g",
                pass: val.rest_max_az_err / G < 0.005,
                detail: format!(
                    "max |a_z - g| = {:.4}, err = {:.3}%",
                    val.rest_max_az_err,
                    val.rest_max_az_err / G * 100.0
                ),
            },
            Check {
                name: "Rest: alpha = 0",
                pass: val.rest_max_alpha_mag < 0.01,
                detail: format!("max |alpha| = {:.2e}", val.rest_max_alpha_mag),
            },
            Check {
                name: "Swing: sign(alpha_Y) = sign(cos theta)",
                pass: val.sign_flip_violations == 0,
                detail: format!("{} violations", val.sign_flip_violations),
            },
            Check {
                name: "Swing: |a| > g at bottom",
                pass: val.swing_max_a_mag > G,
                detail: format!("max |a| = {:.4} > g = {G:.4}", val.swing_max_a_mag),
            },
        ];
        let _ = print_report("Frame Acceleration (t=15s)", &checks);
    }
}

// ── Helpers ─────────────────────────────────────────────────────────────────

fn sensor_vec3_or_zero(
    data: &sim_core::types::Data,
    model: &sim_core::types::Model,
    name: &str,
) -> [f64; 3] {
    if let Some(id) = model.sensor_id(name) {
        let s = data.sensor_data(model, id);
        if s.len() >= 3 {
            return [s[0], s[1], s[2]];
        }
    }
    [0.0, 0.0, 0.0]
}

fn vec3_norm(v: &[f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}
