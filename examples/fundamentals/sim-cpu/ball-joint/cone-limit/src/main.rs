//! Cone Limit — Limited Ball Joint
//!
//! A pendulum on a ball joint with a 45° cone limit. Displaced 60° from
//! vertical (beyond the limit) and released — the constraint pushes it back
//! inside the cone, where it oscillates.
//!
//! Demonstrates: `limited="true"`, `range="0 0.7854"` (45° cone),
//! constraint solver enforcement, `solref`/`solimp` tuning.
//!
//! Validates:
//! - Cone limit enforced: rotation angle never exceeds 45° + solver margin
//! - Energy monotonically decreasing (damped system, limit dissipates)
//! - Quaternion norm preserved to machine precision
//!
//! Run with: `cargo run -p example-ball-joint-cone-limit --release`

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
use sim_core::validation::quat_rotation_angle;

// ── MJCF Model ──────────────────────────────────────────────────────────────
//
// Single ball-joint pendulum with a 45° cone limit.
// Light damping to stabilize the limit constraint and show energy dissipation.
//
const MJCF: &str = r#"
<mujoco model="cone_limit">
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
              fromto="-0.3 0 0  0.3 0 0" rgba="0.40 0.40 0.43 1"/>
        <geom name="post_l" type="capsule" size="0.022"
              fromto="-0.3 0 0  -0.3 0 0.35" rgba="0.40 0.40 0.43 1"/>
        <geom name="post_r" type="capsule" size="0.022"
              fromto="0.3 0 0  0.3 0 0.35" rgba="0.40 0.40 0.43 1"/>

        <!-- Pendulum: limited ball joint (45° cone), light damping -->
        <body name="pendulum" pos="0 0 0">
            <joint name="ball" type="ball" damping="0.01"
                   limited="true" range="0 0.7854"/>
            <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="socket" type="sphere" size="0.035"
                  pos="0 0 0" rgba="0.35 0.33 0.32 1"/>
            <geom name="rod" type="capsule" size="0.02"
                  fromto="0 0 0  0 0 -0.5" rgba="0.48 0.48 0.50 1"/>
            <geom name="tip" type="sphere" size="0.07"
                  pos="0 0 -0.5" rgba="0.82 0.22 0.15 1"/>
        </body>
    </worldbody>
</mujoco>
"#;

// ── Physics constants ───────────────────────────────────────────────────────

const CONE_LIMIT_RAD: f64 = std::f64::consts::FRAC_PI_4; // 45°

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    let limit_deg = CONE_LIMIT_RAD.to_degrees();
    println!("=== CortenForge: Cone Limit ===");
    println!("  Ball joint with {limit_deg:.0}° cone limit, light damping");
    println!("  Initial tilt: 60° (beyond limit — constraint pushes back)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Cone Limit".into(),
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
                    format!(
                        "angle={angle_deg:5.1}° / {limit_deg:.0}° {status}  E={energy:.4}J"
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
                // Constraint solver can inject tiny energy during limit enforcement.
                // 1e-3 J is well within acceptable solver tolerance.
                .track_energy_monotonic(1e-3)
                .skip_until(1.0)
                // Solver allows slight penetration (controlled by solref/solimp).
                // 0.02 rad ≈ 1.1° is typical for default solver parameters.
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
        .add_systems(PostUpdate, (sync_geom_transforms, validation_system))
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();

    // ── Initial displacement: 60° about X axis (beyond 45° cone limit) ──
    let angle = 60.0_f64.to_radians();
    let half = angle / 2.0;
    let qpos_adr = model.jnt_qpos_adr[0];
    data.qpos[qpos_adr] = half.cos();
    data.qpos[qpos_adr + 1] = half.sin(); // pure X rotation
    data.qpos[qpos_adr + 2] = 0.0;
    data.qpos[qpos_adr + 3] = 0.0;

    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} DOFs\n",
        model.nbody, model.njnt, model.nv
    );

    // ── Metallic materials ──────────────────────────────────────────────
    let mat_frame = materials.add(MetalPreset::BrushedMetal.material());
    let mat_socket = materials.add(MetalPreset::CastIron.material());
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.82, 0.22, 0.15)));

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
