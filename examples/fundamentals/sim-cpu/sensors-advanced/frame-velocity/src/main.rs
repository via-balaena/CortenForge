//! Frame Velocity Sensors
//!
//! FrameLinVel and FrameAngVel on a spinning rod. A velocity servo drives the
//! hinge at constant ω = 2π rad/s. A site at the rod tip measures both sensors.
//!
//! Validates:
//! - FrameAngVel Z = 2π (constant spin about hinge axis)
//! - FrameAngVel XY ≈ 0 (no off-axis rotation)
//! - |FrameLinVel| = ωR = π (tip traces circle of radius 0.5 m)
//! - FrameLinVel Z ≈ 0 (planar motion)
//!
//! Run with: `cargo run -p example-sensor-adv-frame-velocity --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::cast_precision_loss
)]

use std::f64::consts::PI;

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, sensor_vec3, spawn_example_camera,
    spawn_physics_hud, validation_system, vec3_magnitude,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};

// ── MJCF Model ──────────────────────────────────────────────────────────────
//
// Horizontal rod on a Z-axis hinge, velocity servo at ω = 2π rad/s.
// Site at the tip (0.5, 0, 0) in body frame. The rod spins in the XY plane.
//
const MJCF: &str = r#"
<mujoco model="frame-velocity">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001">
        <flag energy="enable"/>
    </option>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <body name="rod" pos="0 0 0.5">
            <joint name="hinge" type="hinge" axis="0 0 1" damping="0"/>
            <geom name="shaft" type="capsule" size="0.02"
                  fromto="0 0 0 0.5 0 0" rgba="0.48 0.48 0.50 1"/>
            <geom name="tip_ball" type="sphere" size="0.04"
                  pos="0.5 0 0" rgba="0.2 0.5 0.85 1"/>
            <site name="tip" pos="0.5 0 0"/>
        </body>
    </worldbody>

    <actuator>
        <velocity name="motor" joint="hinge" kv="100"/>
    </actuator>

    <sensor>
        <framelinvel name="tip_linvel" objtype="site" objname="tip"/>
        <frameangvel name="tip_angvel" objtype="site" objname="tip"/>
    </sensor>
</mujoco>
"#;

const OMEGA: f64 = 2.0 * PI;
const RADIUS: f64 = 0.5;
const EXPECTED_SPEED: f64 = OMEGA * RADIUS; // π ≈ 3.14159

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Frame Velocity Sensors ===");
    println!("  FrameLinVel + FrameAngVel on a spinning rod");
    println!("  Velocity servo: ω = 2π rad/s, tip radius = 0.5 m");
    println!("  Expected: |v| = π m/s, ω_z = 2π rad/s");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Frame Velocity".into(),
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
                    let v = sensor_vec3(d, m, "tip_linvel");
                    let speed = vec3_magnitude(&v);
                    let w = sensor_vec3(d, m, "tip_angvel");
                    format!("|v|={speed:.4}  ω_z={:.4}", w[2])
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

    let mat_shaft = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.5, 0.85)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("shaft", mat_shaft), ("tip_ball", mat_tip)],
    );

    // Camera looking slightly down at the spinning plane
    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.5, 0.0), // Bevy Y-up: target at rod height
        2.0,
        0.0,
        0.6, // slight elevation
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Motor Drive ─────────────────────────────────────────────────────────────

fn drive_motor(mut data: ResMut<PhysicsData>) {
    data.set_ctrl(0, OMEGA);
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    let v = sensor_vec3(&data, &model, "tip_linvel");
    let w = sensor_vec3(&data, &model, "tip_angvel");
    let speed = vec3_magnitude(&v);

    hud.clear();
    hud.section("Frame Velocity");
    hud.vec3("v (linvel)", &v, 4);
    hud.scalar("|v|", speed, 4);
    hud.scalar("expected |v|", EXPECTED_SPEED, 4);
    hud.raw(String::new());
    hud.vec3("w (angvel)", &w, 4);
    hud.scalar("w_z", w[2], 4);
    hud.scalar("expected w_z", OMEGA, 4);
    hud.raw(String::new());
    let err_v = if EXPECTED_SPEED > 0.0 {
        (speed - EXPECTED_SPEED).abs() / EXPECTED_SPEED * 100.0
    } else {
        0.0
    };
    let err_w = if OMEGA > 0.0 {
        (w[2] - OMEGA).abs() / OMEGA * 100.0
    } else {
        0.0
    };
    hud.raw(format!("  |v| err    {err_v:.3}%"));
    hud.raw(format!("  w_z err    {err_w:.3}%"));
}

// ── Sensor Validation ───────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SensorValidation {
    // Tracked after 2s spin-up
    sum_speed: f64,
    sum_wz: f64,
    max_vz: f64,
    max_wxy: f64,
    sample_count: u64,
    reported: bool,
}

fn sensor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SensorValidation>,
) {
    // Skip first 2 seconds (spin-up transient)
    if data.time < 2.0 {
        return;
    }

    let v = sensor_vec3(&data, &model, "tip_linvel");
    let w = sensor_vec3(&data, &model, "tip_angvel");
    let speed = vec3_magnitude(&v);

    val.sum_speed += speed;
    val.sum_wz += w[2];
    val.max_vz = val.max_vz.max(v[2].abs());
    val.max_wxy = val.max_wxy.max(w[0].abs().max(w[1].abs()));
    val.sample_count += 1;

    if harness.reported() && !val.reported {
        val.reported = true;
        let n = val.sample_count as f64;
        let mean_speed = val.sum_speed / n;
        let mean_wz = val.sum_wz / n;

        let checks = vec![
            Check {
                name: "AngVel Z = 2pi",
                pass: ((mean_wz - OMEGA) / OMEGA).abs() < 0.005,
                detail: format!(
                    "mean w_z={mean_wz:.6}, expected={OMEGA:.6}, err={:.3}%",
                    (mean_wz - OMEGA).abs() / OMEGA * 100.0
                ),
            },
            Check {
                name: "AngVel XY ~ 0",
                pass: val.max_wxy < 0.01,
                detail: format!("max |w_xy|={:.2e}", val.max_wxy),
            },
            Check {
                name: "|LinVel| = pi",
                pass: ((mean_speed - EXPECTED_SPEED) / EXPECTED_SPEED).abs() < 0.005,
                detail: format!(
                    "mean |v|={mean_speed:.6}, expected={EXPECTED_SPEED:.6}, err={:.3}%",
                    (mean_speed - EXPECTED_SPEED).abs() / EXPECTED_SPEED * 100.0
                ),
            },
            Check {
                name: "LinVel Z ~ 0",
                pass: val.max_vz < 0.01,
                detail: format!("max |v_z|={:.2e}", val.max_vz),
            },
        ];
        let _ = print_report("Frame Velocity (t=15s)", &checks);
    }
}
