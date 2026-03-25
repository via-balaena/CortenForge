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
//! - `BallQuat` sensor matches qpos (within one-step lag)
//! - `BallAngVel` sensor matches qvel (within one-step lag)
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
use sim_bevy::camera::{OrbitCamera, OrbitCameraPlugin};
use sim_bevy::materials::{MetalPreset, override_geom_materials_by_name};
use sim_bevy::model_data::{
    ModelGeomIndex, PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms,
    step_physics_realtime, sync_geom_transforms,
};
use sim_core::ENABLE_ENERGY;
use sim_core::validation::{Check, EnergyConservationTracker, QuaternionNormTracker, print_report};

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

// ── Physics constants ───────────────────────────────────────────────────────

const ROD_LENGTH: f64 = 0.5;
const MASS: f64 = 1.0;
const GRAVITY: f64 = 9.81;
const I_DIAG: f64 = 0.01; // isotropic body inertia about COM

/// Compute mechanical energy from qpos and qvel (first-principles).
///
/// For a ball joint pendulum with COM at body-local `(0, 0, -L)`:
/// - `PE = m·g·z_com` where `z_com = -L·(1 - 2·(qx² + qy²))`
/// - `KE = 0.5·m·L²·(ωx² + ωy²) + 0.5·I·|ω|²`
///
/// Conserved when `damping=0`.
fn pendulum_energy(qpos: &[f64], qvel: &[f64]) -> f64 {
    let x = qpos[1];
    let y = qpos[2];
    let (wx, wy, wz) = (qvel[0], qvel[1], qvel[2]);

    // PE: COM height from quaternion rotation of (0, 0, -L)
    let z_com = -ROD_LENGTH * (1.0 - 2.0 * (x * x + y * y));
    let pe = MASS * GRAVITY * z_com;

    // KE: linear (COM velocity) + rotational (body spin)
    let ke_linear = 0.5 * MASS * ROD_LENGTH * ROD_LENGTH * (wx * wx + wy * wy);
    let ke_rotational = 0.5 * I_DIAG * (wx * wx + wy * wy + wz * wz);

    ke_linear + ke_rotational + pe
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Spherical Pendulum ===");
    println!("  Unlimited ball joint, zero damping, RK4 integrator");
    println!("  Rod = {ROD_LENGTH} m, mass = {MASS} kg, I = {I_DIAG}");
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
        .init_resource::<DiagTimer>()
        .init_resource::<Validation>()
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_materials, step_physics_realtime).chain())
        .add_systems(PostUpdate, (sync_geom_transforms, diagnostics))
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

    // ── Spawn MJCF geometry ─────────────────────────────────────────────
    spawn_model_geoms(&mut commands, &mut meshes, &mut materials, &model, &data);

    // ── Metallic materials ──────────────────────────────────────────────
    let mat_frame = materials.add(MetalPreset::BrushedMetal.material());
    let mat_socket = materials.add(MetalPreset::CastIron.material());
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.5, 0.85)));

    // ── Camera + lights ─────────────────────────────────────────────────
    let mut orbit = OrbitCamera::new()
        .with_target(Vec3::new(0.0, -0.2, 0.0))
        .with_angles(std::f32::consts::FRAC_PI_4, 0.35);
    orbit.max_distance = 20.0;
    orbit.distance = 1.8;
    let mut cam_transform = Transform::default();
    orbit.apply_to_transform(&mut cam_transform);
    commands.spawn((Camera3d::default(), orbit, cam_transform));

    commands.insert_resource(GlobalAmbientLight {
        color: Color::WHITE,
        brightness: 800.0,
        ..default()
    });
    commands.spawn((
        DirectionalLight {
            illuminance: 15_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(30.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 5_000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(-20.0, 30.0, -30.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
    commands.insert_resource(MaterialOverrides {
        frame: mat_frame,
        socket: mat_socket,
        rod: mat_rod,
        tip: mat_tip,
    });
}

// ── Material override (runs once) ───────────────────────────────────────────

#[derive(Resource)]
struct MaterialOverrides {
    frame: Handle<StandardMaterial>,
    socket: Handle<StandardMaterial>,
    rod: Handle<StandardMaterial>,
    tip: Handle<StandardMaterial>,
}

fn apply_materials(
    mut commands: Commands,
    model: Option<Res<PhysicsModel>>,
    overrides: Option<Res<MaterialOverrides>>,
    mut query: Query<(&ModelGeomIndex, &mut MeshMaterial3d<StandardMaterial>)>,
) {
    let (Some(model), Some(ovr)) = (model, overrides) else {
        return;
    };

    override_geom_materials_by_name(
        &model,
        &[
            ("beam", ovr.frame.clone()),
            ("post_l", ovr.frame.clone()),
            ("post_r", ovr.frame.clone()),
            ("socket", ovr.socket.clone()),
            ("rod", ovr.rod.clone()),
            ("tip", ovr.tip.clone()),
        ],
        &mut query,
    );

    commands.remove_resource::<MaterialOverrides>();
}

// ── Diagnostics & Validation ────────────────────────────────────────────────

#[derive(Resource, Default)]
struct DiagTimer {
    last: f64,
}

#[derive(Resource)]
struct Validation {
    quat_norm: QuaternionNormTracker,
    energy: EnergyConservationTracker,
    sensor_quat_err: f64,
    sensor_angvel_err: f64,
    reported: bool,
}

impl Default for Validation {
    fn default() -> Self {
        Self {
            quat_norm: QuaternionNormTracker::new(),
            energy: EnergyConservationTracker::new(),
            sensor_quat_err: 0.0,
            sensor_angvel_err: 0.0,
            reported: false,
        }
    }
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut timer: ResMut<DiagTimer>,
    mut val: ResMut<Validation>,
) {
    let time = data.time;

    // ── Quaternion norm ─────────────────────────────────────────────────
    let adr = model.jnt_qpos_adr[0];
    let (w, x, y, z) = (
        data.qpos[adr],
        data.qpos[adr + 1],
        data.qpos[adr + 2],
        data.qpos[adr + 3],
    );
    val.quat_norm.sample(w, x, y, z);

    // ── Energy from qpos/qvel (first-principles, no pipeline lag) ─────
    let qpos_slice = &[
        data.qpos[adr],
        data.qpos[adr + 1],
        data.qpos[adr + 2],
        data.qpos[adr + 3],
    ];
    let dof = model.jnt_dof_adr[0];
    let qvel_slice = &[data.qvel[dof], data.qvel[dof + 1], data.qvel[dof + 2]];
    let energy = pendulum_energy(qpos_slice, qvel_slice);
    val.energy.sample(energy);

    // ── Sensor accuracy ─────────────────────────────────────────────────
    // Sensors are evaluated during forward() inside step(), reflecting
    // the state BEFORE integration. After step(), qpos/qvel are one step
    // ahead. This is standard MuJoCo pipeline behavior.
    for sensor_id in 0..model.nsensor {
        let s_adr = model.sensor_adr[sensor_id];
        let s_dim = model.sensor_dim[sensor_id];
        let objid = model.sensor_objid[sensor_id];

        if s_dim == 4 {
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
            let d_adr = model.jnt_dof_adr[objid];
            for k in 0..3 {
                let err = (data.sensordata[s_adr + k] - data.qvel[d_adr + k]).abs();
                if err > val.sensor_angvel_err {
                    val.sensor_angvel_err = err;
                }
            }
        }
    }

    // ── Per-second diagnostics ──────────────────────────────────────────
    if time - timer.last > 1.0 {
        timer.last = time;

        let dof = model.jnt_dof_adr[0];
        let omega_mag =
            (data.qvel[dof].powi(2) + data.qvel[dof + 1].powi(2) + data.qvel[dof + 2].powi(2))
                .sqrt();

        let energy_str = format!("{energy:.4}");

        let angle = 2.0 * (x * x + y * y + z * z).sqrt().atan2(w.abs());
        let angle_deg = angle.to_degrees();

        println!(
            "t={time:5.1}s  q=({w:+.3},{x:+.3},{y:+.3},{z:+.3})  |w|={omega_mag:.2}  angle={angle_deg:.1}°  E={energy_str}J"
        );
    }

    // ── Validation report at t=15s ──────────────────────────────────────
    if time > 15.0 && !val.reported {
        val.reported = true;

        let sensor_quat_check = Check {
            name: "BallQuat sensor",
            pass: val.sensor_quat_err < 0.005,
            detail: format!(
                "max error={:.2e} (one-step lag, dt=0.001)",
                val.sensor_quat_err
            ),
        };

        let sensor_angvel_check = Check {
            name: "BallAngVel sensor",
            pass: val.sensor_angvel_err < 0.5,
            detail: format!(
                "max error={:.2e} (one-step lag, dt=0.001)",
                val.sensor_angvel_err
            ),
        };

        let checks = vec![
            val.quat_norm.check(1e-10),
            val.energy.check(0.5),
            sensor_quat_check,
            sensor_angvel_check,
        ];
        let _ = print_report("Validation Report (t=15s)", &checks);
    }
}
