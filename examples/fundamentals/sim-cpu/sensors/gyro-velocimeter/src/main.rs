//! Gyro + Velocimeter Sensors — Conical Pendulum
//!
//! A ball-joint pendulum in steady conical orbit with sensors at the tip.
//! Both sensors read in the sensor's local frame, which rotates with the body.
//!
//! For a conical pendulum (tilt θ about X, precession Ω around world Z):
//! - **Gyro** reads body-frame angular velocity: `[0, Ω·sin(θ), Ω·cos(θ)]` —
//!   constant, with 2 non-zero components (a hinge would only have 1).
//! - **Velocimeter** reads `[-L·Ω·sin(θ), 0, 0]` — the tangential velocity
//!   in the sensor frame (v = ω × r with site on body Z-axis).
//!
//! Validates:
//! - Gyro == joint_qvel component-wise (3D cross-check, tolerance 1e-10)
//! - |gyro| ≈ Ω analytically (magnitude preserved under frame rotation)
//! - |velocimeter| ≈ Ω·L·sin(θ) analytically (tip orbital speed)
//! - Gyro Y and Z both non-trivial (proves multi-axis rotation)
//! - Energy conservation (undamped ball joint, RK4)
//!
//! Run with: `cargo run -p example-sensor-gyro-velocimeter --release`

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
use sim_core::ENABLE_ENERGY;
use sim_core::validation::{Check, print_report};

// ── Constants ───────────────────────────────────────────────────────────────

const L: f64 = 0.5; // Site distance from pivot
const GRAVITY: f64 = 9.81;
const TILT: f64 = std::f64::consts::FRAC_PI_6; // 30° tilt from vertical

/// Precession rate for steady conical orbit: Ω = √(g / (L·cos(θ)))
fn conical_omega() -> f64 {
    (GRAVITY / (L * TILT.cos())).sqrt()
}

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="gyro_velocimeter">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

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
                  pos="0 0 -0.5" rgba="0.2 0.65 0.35 1"/>
            <site name="imu" pos="0 0 -0.5"/>
        </body>
    </worldbody>

    <sensor>
        <gyro name="gyro" site="imu"/>
        <velocimeter name="vel" site="imu"/>
    </sensor>
</mujoco>
"#;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    let omega = conical_omega();
    let tip_speed = omega * L * TILT.sin();
    println!("=== CortenForge: Gyro + Velocimeter Sensors ===");
    println!("  Conical pendulum (ball joint), 30° tilt");
    println!("  Ω={omega:.3} rad/s, tip speed={tip_speed:.3} m/s");
    println!(
        "  Gyro expects [0, {:.3}, {:.3}] rad/s",
        omega * TILT.sin(),
        omega * TILT.cos()
    );
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Gyro + Velocimeter".into(),
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
                    let gyro = d.sensor_data(m, 0);
                    let vel = d.sensor_data(m, 1);
                    let gyro_mag =
                        (gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]).sqrt();
                    let vel_mag = (vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]).sqrt();
                    format!(
                        "gyro=({:+.3},{:+.3},{:+.3})  |ω|={gyro_mag:.3}  |v|={vel_mag:.3}",
                        gyro[0], gyro[1], gyro[2],
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
    let mut model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    model.enableflags |= ENABLE_ENERGY;
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

    println!(
        "  Model: {} bodies, {} joints, {} sensors\n",
        model.nbody, model.njnt, model.nsensor
    );

    // ── Materials ────────────────────────────────────────────────────────
    let mat_frame = materials.add(MetalPreset::BrushedMetal.material());
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.65, 0.35)));

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
    hud.section("Gyro + Velocimeter");
    let gyro = data.sensor_data(&model, 0);
    let vel = data.sensor_data(&model, 1);
    let gyro_mag = (gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]).sqrt();
    let vel_mag = (vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]).sqrt();
    hud.vec3("gyro", gyro, 3);
    hud.vec3("veloc", vel, 3);
    hud.scalar("|ω|", gyro_mag, 3);
    hud.scalar("|v|", vel_mag, 3);
}

// ── Sensor Validation ───────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SensorValidation {
    gyro_qvel_max_err: f64,
    gyro_mag_max_err: f64,
    vel_mag_max_err: f64,
    gyro_y_max_abs: f64,
    gyro_z_max_abs: f64,
    reported: bool,
}

fn sensor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SensorValidation>,
) {
    let omega = conical_omega();
    let expected_tip_speed = omega * L * TILT.sin();

    // Gyro == joint_qvel: 3D component-wise cross-check.
    // For a ball joint, qvel is angular velocity in body frame — same frame
    // the gyro reads in (site has no rotation offset).
    let gyro = data.sensor_data(&model, 0);
    let qvel = data.joint_qvel(&model, 0);
    let gyro_err =
        ((gyro[0] - qvel[0]).powi(2) + (gyro[1] - qvel[1]).powi(2) + (gyro[2] - qvel[2]).powi(2))
            .sqrt();
    if gyro_err > val.gyro_qvel_max_err {
        val.gyro_qvel_max_err = gyro_err;
    }

    // |gyro| ≈ Ω: analytical magnitude check
    let gyro_mag = (gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]).sqrt();
    let mag_err = (gyro_mag - omega).abs();
    if mag_err > val.gyro_mag_max_err {
        val.gyro_mag_max_err = mag_err;
    }

    // |velocimeter| ≈ Ω·L·sin(θ): analytical tip speed
    let vel = data.sensor_data(&model, 1);
    let vel_mag = (vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]).sqrt();
    let vel_err = (vel_mag - expected_tip_speed).abs();
    if vel_err > val.vel_mag_max_err {
        val.vel_mag_max_err = vel_err;
    }

    // Track max absolute gyro Y and Z (should both be non-trivial)
    if gyro[1].abs() > val.gyro_y_max_abs {
        val.gyro_y_max_abs = gyro[1].abs();
    }
    if gyro[2].abs() > val.gyro_z_max_abs {
        val.gyro_z_max_abs = gyro[2].abs();
    }

    // Report
    if harness.reported() && !val.reported {
        val.reported = true;
        let checks = vec![
            Check {
                name: "Gyro == qvel (3D)",
                pass: val.gyro_qvel_max_err < 1e-10,
                detail: format!("max error={:.2e}", val.gyro_qvel_max_err),
            },
            Check {
                name: "|gyro| == Ω",
                pass: val.gyro_mag_max_err < 1e-4,
                detail: format!("max |err|={:.2e} (Ω={omega:.3})", val.gyro_mag_max_err,),
            },
            Check {
                name: "|veloc| == Ω·L·sin(θ)",
                pass: val.vel_mag_max_err < 1e-4,
                detail: format!(
                    "max |err|={:.2e} (expect={expected_tip_speed:.3})",
                    val.vel_mag_max_err,
                ),
            },
            Check {
                name: "Gyro Y non-trivial",
                pass: val.gyro_y_max_abs > 1.0,
                detail: format!("max |gyro_y|={:.3} rad/s", val.gyro_y_max_abs),
            },
            Check {
                name: "Gyro Z non-trivial",
                pass: val.gyro_z_max_abs > 1.0,
                detail: format!("max |gyro_z|={:.3} rad/s", val.gyro_z_max_abs),
            },
        ];
        let _ = print_report("Gyro + Velocimeter (t=15s)", &checks);
    }
}
