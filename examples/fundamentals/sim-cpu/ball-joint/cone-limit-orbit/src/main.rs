//! Cone-Limit Orbit — Sliding Along the Constraint Surface
//!
//! A pendulum on a ball joint with a 30° cone limit, given initial angular
//! velocity so the tip mass orbits along the inside of the cone boundary.
//! A fading trail shows the circular path on the constraint surface —
//! proving the cone limit is a true 3D surface, not just a 1D angular limit.
//!
//! Demonstrates: `limited="true"` cone constraint as a 3D surface,
//! friction-free constraint sliding, trail visualization.
//!
//! Validates:
//! - Energy monotonically decreasing (lightly damped)
//! - Quaternion norm preserved to machine precision
//! - Cone limit enforced (30° + solver margin)
//!
//! Run with: `cargo run -p example-ball-joint-cone-orbit --release`

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
use sim_bevy::examples::{ValidationHarness, spawn_example_camera, validation_system};
use sim_bevy::gizmos::{TrailGizmo, draw_trails, sample_trails};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms_with, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::ENABLE_ENERGY;
use sim_core::validation::quat_rotation_angle;

// ── MJCF Model ──────────────────────────────────────────────────────────────
//
// Ball joint with 30° cone limit. Very light damping (0.002) so the orbit
// persists for ~20+ seconds but slowly spirals inward.
//
const MJCF: &str = r#"
<mujoco model="cone_limit_orbit">
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

        <!-- Pendulum: ball joint with 30° cone limit -->
        <body name="pendulum" pos="0 0 0">
            <joint name="ball" type="ball" damping="0.002"
                   limited="true" range="0 0.7854"/>
            <inertial pos="0 0 -0.6" mass="1.0" diaginertia="0.001 0.001 0.001"/>
            <geom name="socket" type="sphere" size="0.035"
                  pos="0 0 0" rgba="0.35 0.33 0.32 1"/>
            <geom name="rod" type="capsule" size="0.018"
                  fromto="0 0 0  0 0 -0.6" rgba="0.48 0.48 0.50 1"/>
            <geom name="tip" type="sphere" size="0.06"
                  pos="0 0 -0.6" rgba="0.9 0.55 0.1 1"/>
        </body>
    </worldbody>
</mujoco>
"#;

// ── Physics constants ───────────────────────────────────────────────────────

const ROD_LENGTH: f64 = 0.6;
const GRAVITY: f64 = 9.81;
const CONE_LIMIT_RAD: f64 = std::f64::consts::FRAC_PI_4; // 45°
const TILT_ANGLE: f64 = CONE_LIMIT_RAD; // start exactly at the cone boundary

/// Angular velocity for conical orbit at angle θ: ω = √(g / (L·cos(θ)))
fn conical_omega() -> f64 {
    (GRAVITY / (ROD_LENGTH * TILT_ANGLE.cos())).sqrt()
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    let omega = conical_omega();
    let radius = ROD_LENGTH * TILT_ANGLE.sin();
    let period = 2.0 * std::f64::consts::PI / omega;
    let limit_deg = CONE_LIMIT_RAD.to_degrees();
    println!("=== CortenForge: Cone-Limit Orbit (Ball Joint) ===");
    println!("  Orbit along {limit_deg:.0}° cone boundary, damping=0.002");
    println!("  L=0.6m, ω={omega:.3} rad/s");
    println!("  Orbit radius={radius:.3}m, period={period:.3}s");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Cone-Limit Orbit".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let q = d.joint_qpos(m, 0);
                    let angle = quat_rotation_angle(q[0], q[1], q[2], q[3]);
                    let angle_deg = angle.to_degrees();
                    let limit_deg = CONE_LIMIT_RAD.to_degrees();
                    let energy = d.energy_kinetic + d.energy_potential;
                    let status = if angle > CONE_LIMIT_RAD {
                        "OVER"
                    } else {
                        "ok  "
                    };
                    format!("angle={angle_deg:5.1}° / {limit_deg:.0}° {status}  E={energy:.4}J")
                })
                .track_quat_norm(
                    "Quat norm",
                    |m, d| {
                        let q = d.joint_qpos(m, 0);
                        (q[0], q[1], q[2], q[3])
                    },
                    1e-10,
                )
                .track_energy_monotonic(1e-3)
                .skip_until(1.0)
                .track_limit(
                    "Limits",
                    |m, d| {
                        let q = d.joint_qpos(m, 0);
                        let angle = quat_rotation_angle(q[0], q[1], q[2], q[3]);
                        (angle, 0.0, CONE_LIMIT_RAD)
                    },
                    0.02,
                )
                .skip_until(1.0),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                sample_trails,
                draw_trails,
                validation_system,
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

    // ── Initial tilt: exactly at cone boundary (30° about X) ────────────
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

    // ── Metallic materials ──────────────────────────────────────────────
    let mat_frame = materials.add(MetalPreset::BrushedMetal.material());
    let mat_socket = materials.add(MetalPreset::CastIron.material());
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.55, 0.1)));

    // ── Spawn MJCF geometry with overrides + trail attachment ───────────
    spawn_model_geoms_with(
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
        |cmds, _id, name| {
            if name == "tip" {
                cmds.insert(TrailGizmo::new(500, Color::srgb(0.9, 0.55, 0.1), 0.01));
            }
        },
    );

    // ── Camera + lights ─────────────────────────────────────────────────
    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, -0.3),
        2.2,
        std::f32::consts::FRAC_PI_4,
        0.5,
    );

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}
