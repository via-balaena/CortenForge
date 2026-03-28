//! FramePos + FrameQuat Sensors — Conical Pendulum
//!
//! A ball-joint pendulum in steady conical orbit. The tip traces a circle in
//! 3D space, exercising all 3 position components and all 4 quaternion
//! components — a hinge pendulum only moves in a single plane.
//!
//! Validates:
//! - FramePos sensor == data.site_xpos (all 3 axes active)
//! - FrameQuat matches data.site_xquat via rotation-distance metric
//! - FramePos X and Y ranges > 0.4m (circular orbit spans full diameter)
//! - Analytical check: tip position at t=0 matches (0, L·sin(θ), -L·cos(θ))
//! - Energy conservation (undamped ball joint, RK4)
//!
//! Run with: `cargo run -p example-sensor-frame-pos-quat --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops
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

// ── Constants ───────────────────────────────────────────────────────────────

const L: f64 = 0.5; // Rod length (pivot to tip)
const GRAVITY: f64 = 9.81;
const TILT: f64 = std::f64::consts::FRAC_PI_6; // 30° tilt from vertical

/// Angular velocity for steady conical orbit: ω = √(g / (L·cos(θ)))
fn conical_omega() -> f64 {
    (GRAVITY / (L * TILT.cos())).sqrt()
}

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="frame_pos_quat">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
        <flag energy="enable"/>
    </option>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <!-- Support frame -->
        <geom name="beam" type="capsule" size="0.022"
              fromto="-0.25 0 0  0.25 0 0" rgba="0.40 0.40 0.43 1"/>
        <geom name="post_l" type="capsule" size="0.022"
              fromto="-0.25 0 0  -0.25 0 0.30" rgba="0.40 0.40 0.43 1"/>
        <geom name="post_r" type="capsule" size="0.022"
              fromto="0.25 0 0  0.25 0 0.30" rgba="0.40 0.40 0.43 1"/>

        <body name="pendulum" pos="0 0 0">
            <joint name="ball" type="ball" damping="0"/>
            <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.001 0.001 0.001"/>
            <geom name="rod" type="capsule" size="0.02"
                  fromto="0 0 0  0 0 -0.5" rgba="0.48 0.48 0.50 1"/>
            <geom name="tip" type="sphere" size="0.05"
                  pos="0 0 -0.5" rgba="0.82 0.22 0.15 1"/>
            <site name="tip_site" pos="0 0 -0.5"/>
        </body>
    </worldbody>

    <sensor>
        <framepos name="tip_pos" site="tip_site"/>
        <framequat name="tip_quat" site="tip_site"/>
    </sensor>
</mujoco>
"#;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    let omega = conical_omega();
    let radius = L * TILT.sin();
    println!("=== CortenForge: FramePos + FrameQuat Sensors ===");
    println!("  Conical pendulum (ball joint), 30° tilt");
    println!("  ω={omega:.3} rad/s, orbit radius={radius:.3}m");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — FramePos + FrameQuat".into(),
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
                    let pos_id = m.sensor_id("tip_pos").unwrap_or(0);
                    let pos = d.sensor_data(m, pos_id);
                    let r_xy = pos[0].hypot(pos[1]);
                    let energy = d.energy_kinetic + d.energy_potential;
                    format!(
                        "pos=({:+.3},{:+.3},{:+.3})  r={r_xy:.3}m  E={energy:.4}J",
                        pos[0], pos[1], pos[2],
                    )
                })
                .track_energy(0.5),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
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
    let mut data = model.make_data();

    // Initial tilt: 30° about X axis → quaternion [cos(θ/2), sin(θ/2), 0, 0]
    let half = TILT / 2.0;
    let qpos_adr = model.jnt_qpos_adr[0];
    data.qpos[qpos_adr] = half.cos();
    data.qpos[qpos_adr + 1] = half.sin();
    data.qpos[qpos_adr + 2] = 0.0;
    data.qpos[qpos_adr + 3] = 0.0;

    // Initial angular velocity: Ω about world Z, expressed in body frame.
    // Body tilted θ about X → world Z in body frame = [0, sin(θ), cos(θ)]
    let omega = conical_omega();
    let dof_adr = model.jnt_dof_adr[0];
    data.qvel[dof_adr] = 0.0;
    data.qvel[dof_adr + 1] = omega * TILT.sin();
    data.qvel[dof_adr + 2] = omega * TILT.cos();

    let _ = data.forward(&model);

    // Analytical t=0: tilt θ about X maps (0,0,-L) → (0, L·sin(θ), -L·cos(θ))
    let pos_id = model.sensor_id("tip_pos").unwrap_or(0);
    let pos = data.sensor_data(&model, pos_id);
    let expected_y = L * TILT.sin();
    let expected_z = -L * TILT.cos();
    println!(
        "  t=0 tip_pos: ({:.4}, {:.4}, {:.4})",
        pos[0], pos[1], pos[2]
    );
    println!("  t=0 expected: (0.0000, {expected_y:.4}, {expected_z:.4})");
    println!(
        "  Model: {} bodies, {} joints, {} sensors\n",
        model.nbody, model.njnt, model.nsensor
    );

    // ── Materials ────────────────────────────────────────────────────────
    let mat_frame = materials.add(MetalPreset::BrushedMetal.material());
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.82, 0.22, 0.15)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("beam", mat_frame.clone()),
            ("post_l", mat_frame.clone()),
            ("post_r", mat_frame),
            ("rod", mat_rod),
            ("tip", mat_tip),
        ],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, -0.2, 0.0),
        1.8,
        std::f32::consts::FRAC_PI_4,
        0.35,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("FramePos + FrameQuat");
    let pos_id = model.sensor_id("tip_pos").unwrap_or(0);
    let quat_id = model.sensor_id("tip_quat").unwrap_or(0);
    let pos = data.sensor_data(&model, pos_id);
    let quat = data.sensor_data(&model, quat_id);
    let r_xy = pos[0].hypot(pos[1]);
    hud.vec3("pos", pos, 4);
    hud.quat("quat", quat);
    hud.scalar("r_xy", r_xy, 4);
}

// ── Sensor Validation ───────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SensorValidation {
    pos_max_err: f64,
    quat_max_rot_err: f64,
    pos_x_min: f64,
    pos_x_max: f64,
    pos_y_min: f64,
    pos_y_max: f64,
    t0_pos_err: f64,
    t0_checked: bool,
    reported: bool,
}

/// Rotation-distance between two quaternions: `1 - |q1 · q2|`.
/// Zero means identical rotation (handles sign ambiguity).
fn quat_rotation_distance(a: &[f64], b: &[f64]) -> f64 {
    let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    1.0 - dot.abs()
}

fn sensor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SensorValidation>,
) {
    let site_id = model.sensor_objid[0];

    // FramePos pipeline check
    let pos_id = model.sensor_id("tip_pos").unwrap_or(0);
    let pos_sensor = data.sensor_data(&model, pos_id);
    let pos_state = &data.site_xpos[site_id];
    let pos_err = ((pos_sensor[0] - pos_state.x).powi(2)
        + (pos_sensor[1] - pos_state.y).powi(2)
        + (pos_sensor[2] - pos_state.z).powi(2))
    .sqrt();
    if pos_err > val.pos_max_err {
        val.pos_max_err = pos_err;
    }

    // FrameQuat rotation-distance check
    let quat_id = model.sensor_id("tip_quat").unwrap_or(0);
    let quat_sensor = data.sensor_data(&model, quat_id);
    let q_data = &data.site_xquat[site_id];
    let quat_data = [q_data.w, q_data.i, q_data.j, q_data.k];
    let rot_err = quat_rotation_distance(quat_sensor, &quat_data);
    if rot_err > val.quat_max_rot_err {
        val.quat_max_rot_err = rot_err;
    }

    // Range tracking — both X and Y should span the orbit diameter
    if pos_sensor[0] < val.pos_x_min {
        val.pos_x_min = pos_sensor[0];
    }
    if pos_sensor[0] > val.pos_x_max {
        val.pos_x_max = pos_sensor[0];
    }
    if pos_sensor[1] < val.pos_y_min {
        val.pos_y_min = pos_sensor[1];
    }
    if pos_sensor[1] > val.pos_y_max {
        val.pos_y_max = pos_sensor[1];
    }

    // Analytical t=0: tilt θ about X maps (0,0,-L) → (0, L·sin(θ), -L·cos(θ))
    if !val.t0_checked && data.time < 0.01 {
        val.t0_checked = true;
        let expected_y = L * TILT.sin();
        let expected_z = -L * TILT.cos();
        val.t0_pos_err = (pos_sensor[0].powi(2)
            + (pos_sensor[1] - expected_y).powi(2)
            + (pos_sensor[2] - expected_z).powi(2))
        .sqrt();
    }

    // Report
    if harness.reported() && !val.reported {
        val.reported = true;
        let x_range = val.pos_x_max - val.pos_x_min;
        let y_range = val.pos_y_max - val.pos_y_min;
        let checks = vec![
            Check {
                name: "FramePos == site_xpos",
                pass: val.pos_max_err < 1e-12,
                detail: format!("max error={:.2e}", val.pos_max_err),
            },
            Check {
                name: "FrameQuat rotation dist",
                pass: val.quat_max_rot_err < 1e-12,
                detail: format!("max 1-|q·q|={:.2e}", val.quat_max_rot_err),
            },
            Check {
                name: "Pos X range > 0.4m",
                pass: x_range > 0.4,
                detail: format!(
                    "range={x_range:.3}m [{:.3}, {:.3}]",
                    val.pos_x_min, val.pos_x_max
                ),
            },
            Check {
                name: "Pos Y range > 0.4m",
                pass: y_range > 0.4,
                detail: format!(
                    "range={y_range:.3}m [{:.3}, {:.3}]",
                    val.pos_y_min, val.pos_y_max
                ),
            },
            Check {
                name: "t=0 analytical pos",
                pass: val.t0_pos_err < 1e-6,
                detail: format!("error={:.2e}m", val.t0_pos_err),
            },
        ];
        let _ = print_report("FramePos + FrameQuat (t=15s)", &checks);
    }
}
