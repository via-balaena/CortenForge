//! Spherical Pendulum — Unlimited Ball Joint
//!
//! A heavy sphere on a rigid rod, hanging from a ball joint attached to a
//! support frame. Displaced ~30° from vertical in both pitch and roll, then
//! released. It traces a precessing elliptical path — a classic 3D spherical
//! pendulum.
//!
//! Demonstrates: `type="ball"`, quaternion representation (4 qpos, 3 qvel),
//! exponential-map integration, `BallQuat` and `BallAngVel` sensors.
//!
//! Validates:
//! - Quaternion norm preserved to machine precision
//! - Energy conservation (undamped system, RK4 integrator)
//! - `BallQuat` sensor matches qpos exactly
//! - `BallAngVel` sensor matches qvel exactly
//!
//! Run with: `cargo run -p example-ball-joint-pendulum --release`

#![allow(
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
use sim_core::validation::{Check, print_report, quat_rotation_angle};

// ── MJCF Model ──────────────────────────────────────────────────────────────
//
// Single ball-joint pendulum hanging from a support frame.
// Zero damping for energy conservation testing.
//
// The socket sphere is ON the pendulum body (not worldbody), so it always
// stays visually connected to the rod regardless of orientation.
//
const MJCF: &str = r#"
<mujoco model="spherical_pendulum">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <!-- Support frame -->
        <geom name="beam" type="capsule" size="0.022"
              fromto="-0.3 0 0  0.3 0 0" rgba="0.40 0.40 0.43 1"/>
        <geom name="post_l" type="capsule" size="0.022"
              fromto="-0.3 0 0  -0.3 0 0.35" rgba="0.40 0.40 0.43 1"/>
        <geom name="post_r" type="capsule" size="0.022"
              fromto="0.3 0 0  0.3 0 0.35" rgba="0.40 0.40 0.43 1"/>

        <!-- Pendulum: unlimited ball joint, zero damping -->
        <body name="pendulum" pos="0 0 0">
            <joint name="ball" type="ball" damping="0"/>
            <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="socket" type="sphere" size="0.035"
                  pos="0 0 0" rgba="0.35 0.33 0.32 1"/>
            <geom name="rod" type="capsule" size="0.02"
                  fromto="0 0 0  0 0 -0.5" rgba="0.48 0.48 0.50 1"/>
            <geom name="tip" type="sphere" size="0.07"
                  pos="0 0 -0.5" rgba="0.2 0.5 0.85 1"/>
        </body>
    </worldbody>

    <sensor>
        <ballquat name="quat" joint="ball"/>
        <ballangvel name="angvel" joint="ball"/>
    </sensor>
</mujoco>
"#;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Spherical Pendulum ===");
    println!("  Unlimited ball joint, zero damping, RK4 integrator");
    println!("  Rod = 0.5 m, mass = 1.0 kg");
    println!("  Initial tilt: 30° about diagonal axis (pitch + roll)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Spherical Pendulum".into(),
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
                    let qpos = d.joint_qpos(m, 0);
                    let qvel = d.joint_qvel(m, 0);
                    let omega_mag =
                        (qvel[0].powi(2) + qvel[1].powi(2) + qvel[2].powi(2)).sqrt();
                    let angle_deg =
                        quat_rotation_angle(qpos[0], qpos[1], qpos[2], qpos[3]).to_degrees();
                    let energy = d.energy_kinetic + d.energy_potential;
                    format!(
                        "q=({:+.3},{:+.3},{:+.3},{:+.3})  |w|={omega_mag:.2}  angle={angle_deg:.1}°  E={energy:.4}J",
                        qpos[0], qpos[1], qpos[2], qpos[3],
                    )
                })
                .track_quat_norm(
                    "Quat norm",
                    |m, d| {
                        let q = d.joint_qpos(m, 0);
                        (q[0], q[1], q[2], q[3])
                    },
                    1e-10,
                )
                .track_energy(0.5),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (sync_geom_transforms, validation_system, sensor_diagnostics),
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

    // ── Initial displacement: 30° tilt about diagonal axis ──────────────
    let angle = 30.0_f64.to_radians();
    let half = angle / 2.0;
    let inv_sqrt2 = std::f64::consts::FRAC_1_SQRT_2;
    let qpos_adr = model.jnt_qpos_adr[0];
    data.qpos[qpos_adr] = half.cos();
    data.qpos[qpos_adr + 1] = half.sin() * inv_sqrt2; // X component
    data.qpos[qpos_adr + 2] = half.sin() * inv_sqrt2; // Y component
    data.qpos[qpos_adr + 3] = 0.0;

    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} DOFs, {} sensors\n",
        model.nbody, model.njnt, model.nv, model.nsensor
    );

    // ── Metallic materials ──────────────────────────────────────────────
    let mat_frame = materials.add(MetalPreset::BrushedMetal.material());
    let mat_socket = materials.add(MetalPreset::CastIron.material());
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.5, 0.85)));

    // ── Spawn MJCF geometry with material overrides ─────────────────────
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
            ("socket", mat_socket),
            ("rod", mat_rod),
            ("tip", mat_tip),
        ],
    );

    // ── Camera + lights ─────────────────────────────────────────────────
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

// ── Sensor Accuracy Validation ──────────────────────────────────────────────
//
// Custom resource + system for BallQuat/BallAngVel sensor accuracy checks.
// These are unique to this example and don't fit the 6 standard tracker types.

#[derive(Resource, Default)]
struct SensorValidation {
    sensor_quat_err: f64,
    sensor_angvel_err: f64,
    reported: bool,
}

fn sensor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SensorValidation>,
) {
    // Accumulate sensor errors every frame
    for sensor_id in 0..model.nsensor {
        let s_adr = model.sensor_adr[sensor_id];
        let s_dim = model.sensor_dim[sensor_id];
        let objid = model.sensor_objid[sensor_id];

        if s_dim == 4 {
            // BallQuat: sensor normalizes, so compare against normalized qpos
            let q_adr = model.jnt_qpos_adr[objid];
            let qnorm = (data.qpos[q_adr].powi(2)
                + data.qpos[q_adr + 1].powi(2)
                + data.qpos[q_adr + 2].powi(2)
                + data.qpos[q_adr + 3].powi(2))
            .sqrt();
            for k in 0..4 {
                let expected = data.qpos[q_adr + k] / qnorm;
                let err = (data.sensordata[s_adr + k] - expected).abs();
                if err > val.sensor_quat_err {
                    val.sensor_quat_err = err;
                }
            }
        } else if s_dim == 3 {
            // BallAngVel: direct pass-through from qvel
            let d_adr = model.jnt_dof_adr[objid];
            for k in 0..3 {
                let err = (data.sensordata[s_adr + k] - data.qvel[d_adr + k]).abs();
                if err > val.sensor_angvel_err {
                    val.sensor_angvel_err = err;
                }
            }
        }
    }

    // Print sensor report when the harness prints its report
    if harness.reported() && !val.reported {
        val.reported = true;

        let checks = vec![
            Check {
                name: "BallQuat sensor",
                pass: val.sensor_quat_err < 1e-10,
                detail: format!("max error={:.2e}", val.sensor_quat_err),
            },
            Check {
                name: "BallAngVel sensor",
                pass: val.sensor_angvel_err < 1e-10,
                detail: format!("max error={:.2e}", val.sensor_angvel_err),
            },
        ];
        let _ = print_report("Sensor Accuracy (t=15s)", &checks);
    }
}
