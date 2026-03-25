//! Conical Pendulum — Orbiting Ball Joint
//!
//! A pendulum on an unlimited ball joint, given initial angular velocity so the
//! tip mass traces a circular orbit around the vertical axis. A glowing trail
//! makes the 3D trajectory visible — this motion is impossible with a hinge.
//!
//! Demonstrates: `type="ball"` with initial angular velocity, conical orbit,
//! precession, trail visualization.
//!
//! Validates:
//! - Energy conservation (undamped, RK4)
//! - Quaternion norm preserved to machine precision
//!
//! Run with: `cargo run -p example-ball-joint-conical --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::convert::physics_pos;
use sim_bevy::examples::{DiagTimer, spawn_example_camera};
use sim_bevy::gizmos::{TrailGizmo, draw_trails, sample_trails};
use sim_bevy::materials::{MetalPreset, override_geom_materials_by_name_with};
use sim_bevy::model_data::{
    ModelGeomIndex, PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms,
    step_physics_realtime, sync_geom_transforms,
};
use sim_core::ENABLE_ENERGY;
use sim_core::validation::{
    EnergyConservationTracker, QuaternionNormTracker, print_report, quat_rotation_angle,
};

// ── MJCF Model ──────────────────────────────────────────────────────────────
//
// Single ball-joint pendulum. Zero damping, longer rod (0.8m) for visible orbit.
// Initial tilt 25° + angular velocity → steady conical orbit.
//
const MJCF: &str = r#"
<mujoco model="conical_pendulum">
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
            <inertial pos="0 0 -0.8" mass="1.0" diaginertia="0.001 0.001 0.001"/>
            <geom name="socket" type="sphere" size="0.035"
                  pos="0 0 0" rgba="0.35 0.33 0.32 1"/>
            <geom name="rod" type="capsule" size="0.018"
                  fromto="0 0 0  0 0 -0.8" rgba="0.48 0.48 0.50 1"/>
            <geom name="tip" type="sphere" size="0.06"
                  pos="0 0 -0.8" rgba="0.2 0.7 0.4 1"/>
        </body>
    </worldbody>
</mujoco>
"#;

// ── Physics constants ───────────────────────────────────────────────────────

const ROD_LENGTH: f64 = 0.8;
const GRAVITY: f64 = 9.81;
const TILT_ANGLE: f64 = std::f64::consts::FRAC_PI_4; // 45°

/// Angular velocity for steady conical orbit: ω = √(g / (L·cos(θ)))
fn conical_omega() -> f64 {
    (GRAVITY / (ROD_LENGTH * TILT_ANGLE.cos())).sqrt()
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    let omega = conical_omega();
    let radius = ROD_LENGTH * TILT_ANGLE.sin();
    let period = 2.0 * std::f64::consts::PI / omega;
    println!("=== CortenForge: Conical Pendulum (Ball Joint) ===");
    println!("  Circular orbit via ball joint + initial angular velocity");
    println!("  L=0.8m, tilt=45°, ω={omega:.3} rad/s");
    println!("  Orbit radius={radius:.3}m, period={period:.3}s");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Conical Pendulum".into(),
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
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                sample_trails,
                draw_trails,
                diagnostics,
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

    // ── Initial tilt: 25° about X axis ──────────────────────────────────
    let half = TILT_ANGLE / 2.0;
    let qpos_adr = model.jnt_qpos_adr[0];
    data.qpos[qpos_adr] = half.cos();
    data.qpos[qpos_adr + 1] = half.sin();
    data.qpos[qpos_adr + 2] = 0.0;
    data.qpos[qpos_adr + 3] = 0.0;

    // ── Initial angular velocity: ω about world Z, expressed in body frame ─
    // Body is tilted θ about X, so world Z in body frame = [0, sin(θ), cos(θ)]
    let dof_adr = model.jnt_dof_adr[0];
    let omega = conical_omega();
    data.qvel[dof_adr] = 0.0;
    data.qvel[dof_adr + 1] = omega * TILT_ANGLE.sin();
    data.qvel[dof_adr + 2] = omega * TILT_ANGLE.cos();

    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} DOFs\n",
        model.nbody, model.njnt, model.nv
    );

    // ── Spawn MJCF geometry ─────────────────────────────────────────────
    spawn_model_geoms(&mut commands, &mut meshes, &mut materials, &model, &data);

    // ── Metallic materials ──────────────────────────────────────────────
    let mat_frame = materials.add(MetalPreset::BrushedMetal.material());
    let mat_socket = materials.add(MetalPreset::CastIron.material());
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.7, 0.4)));

    // ── Camera + lights ─────────────────────────────────────────────────
    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, -0.4),
        2.5,
        std::f32::consts::FRAC_PI_4,
        0.5,
    );

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
    commands.insert_resource(MaterialOverrides {
        frame: mat_frame,
        socket: mat_socket,
        rod: mat_rod,
        tip: mat_tip,
    });
}

// ── Material override + trail attachment (runs once) ────────────────────────

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
    mut query: Query<(
        Entity,
        &ModelGeomIndex,
        &mut MeshMaterial3d<StandardMaterial>,
    )>,
) {
    let (Some(model), Some(ovr)) = (model, overrides) else {
        return;
    };

    override_geom_materials_by_name_with(
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
        &mut commands,
        |entity, name, cmds| {
            if name == "tip" {
                cmds.entity(entity)
                    .insert(TrailGizmo::new(500, Color::srgb(0.2, 0.7, 0.4), 0.01));
            }
        },
    );

    commands.remove_resource::<MaterialOverrides>();
}

// ── Diagnostics & Validation ────────────────────────────────────────────────

#[derive(Resource)]
struct Validation {
    quat_norm: QuaternionNormTracker,
    energy: EnergyConservationTracker,
    reported: bool,
}

impl Default for Validation {
    fn default() -> Self {
        Self {
            quat_norm: QuaternionNormTracker::new(),
            energy: EnergyConservationTracker::new(),
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

    let adr = model.jnt_qpos_adr[0];
    let (w, x, y, z) = (
        data.qpos[adr],
        data.qpos[adr + 1],
        data.qpos[adr + 2],
        data.qpos[adr + 3],
    );
    val.quat_norm.sample(w, x, y, z);

    let energy = data.energy_kinetic + data.energy_potential;
    val.energy.sample(energy);

    if time - timer.last > 1.0 {
        timer.last = time;
        let angle = quat_rotation_angle(w, x, y, z);
        println!(
            "t={time:5.1}s  tilt={:.1}°  E={energy:.4}J",
            angle.to_degrees(),
        );
    }

    if time > 15.0 && !val.reported {
        val.reported = true;
        let checks = vec![val.quat_norm.check(1e-10), val.energy.check(0.5)];
        let _ = print_report("Validation Report (t=15s)", &checks);
    }
}
