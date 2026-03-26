//! Gyro + Velocimeter Sensors
//!
//! A hinge pendulum with a site at the tip. Both sensors read in the sensor's
//! local frame (not world frame). For a Y-axis hinge:
//!
//! - Gyro reads `[0, ω, 0]` — the rotation axis is invariant under Y-rotation
//! - Velocimeter reads `[-L*ω, 0, 0]` — the tip always moves in the local -X
//!   direction (tangent to the arc)
//!
//! Both relationships are exact for a single-axis hinge.
//!
//! Validates:
//! - Gyro Y == joint_qvel (exact, tolerance 1e-10)
//! - Gyro X, Z == 0 (exact, tolerance 1e-10)
//! - Velocimeter X == -L * joint_qvel (exact, tolerance 1e-10)
//! - Velocimeter Y, Z == 0 (exact, tolerance 1e-10)
//! - Gyro Y range > 1.0 rad/s (pendulum actually swings)
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
use sim_bevy::examples::{ValidationHarness, spawn_example_camera, validation_system};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::ENABLE_ENERGY;
use sim_core::validation::{Check, print_report};

// ── Constants ───────────────────────────────────────────────────────────────

const L: f64 = 0.5; // Site distance from pivot

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

        <body name="arm" pos="0 0 0">
            <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
            <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
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
    println!("=== CortenForge: Gyro + Velocimeter Sensors ===");
    println!("  Sensor-local frame readings on hinge pendulum tip");
    println!("  Gyro: [0, ω, 0]  Velocimeter: [-Lω, 0, 0]");
    println!("  Initial tilt: 30°");
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
        .init_resource::<SensorValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let gyro = d.sensor_data(m, 0);
                    let vel = d.sensor_data(m, 1);
                    let omega = d.joint_qvel(m, 0)[0];
                    format!(
                        "gyro=({:+.3},{:+.3},{:+.3})  vel=({:+.3},{:+.3},{:+.3})  ω={omega:+.3}",
                        gyro[0], gyro[1], gyro[2], vel[0], vel[1], vel[2],
                    )
                })
                .track_energy(0.5),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (sync_geom_transforms, validation_system, sensor_diagnostics).chain(),
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

    // Initial displacement: 30°
    data.qpos[model.jnt_qpos_adr[0]] = std::f64::consts::FRAC_PI_6;
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

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Sensor Validation ───────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SensorValidation {
    gyro_y_max_err: f64,
    gyro_xz_max_err: f64,
    vel_x_max_err: f64,
    vel_yz_max_err: f64,
    gyro_y_min: f64,
    gyro_y_max: f64,
    reported: bool,
}

fn sensor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SensorValidation>,
) {
    let omega = data.joint_qvel(&model, 0)[0];

    // Gyro: should be [0, ω, 0] in sensor frame
    let gyro = data.sensor_data(&model, 0);
    let gyro_y_err = (gyro[1] - omega).abs();
    let gyro_xz_err = gyro[0].abs().max(gyro[2].abs());
    if gyro_y_err > val.gyro_y_max_err {
        val.gyro_y_max_err = gyro_y_err;
    }
    if gyro_xz_err > val.gyro_xz_max_err {
        val.gyro_xz_max_err = gyro_xz_err;
    }

    // Velocimeter: should be [-L*ω, 0, 0] in sensor frame
    let vel = data.sensor_data(&model, 1);
    let expected_vx = -L * omega;
    let vel_x_err = (vel[0] - expected_vx).abs();
    let vel_yz_err = vel[1].abs().max(vel[2].abs());
    if vel_x_err > val.vel_x_max_err {
        val.vel_x_max_err = vel_x_err;
    }
    if vel_yz_err > val.vel_yz_max_err {
        val.vel_yz_max_err = vel_yz_err;
    }

    // Range tracking
    if gyro[1] < val.gyro_y_min {
        val.gyro_y_min = gyro[1];
    }
    if gyro[1] > val.gyro_y_max {
        val.gyro_y_max = gyro[1];
    }

    // Report
    if harness.reported() && !val.reported {
        val.reported = true;
        let gyro_range = val.gyro_y_max - val.gyro_y_min;
        let checks = vec![
            Check {
                name: "Gyro Y == ω",
                pass: val.gyro_y_max_err < 1e-10,
                detail: format!("max error={:.2e}", val.gyro_y_max_err),
            },
            Check {
                name: "Gyro X,Z == 0",
                pass: val.gyro_xz_max_err < 1e-10,
                detail: format!("max error={:.2e}", val.gyro_xz_max_err),
            },
            Check {
                name: "Veloc X == -Lω",
                pass: val.vel_x_max_err < 1e-10,
                detail: format!("max error={:.2e} (L={L})", val.vel_x_max_err),
            },
            Check {
                name: "Veloc Y,Z == 0",
                pass: val.vel_yz_max_err < 1e-10,
                detail: format!("max error={:.2e}", val.vel_yz_max_err),
            },
            Check {
                name: "Gyro Y range",
                pass: gyro_range > 1.0,
                detail: format!("range={gyro_range:.2} rad/s"),
            },
        ];
        let _ = print_report("Gyro + Velocimeter (t=15s)", &checks);
    }
}
