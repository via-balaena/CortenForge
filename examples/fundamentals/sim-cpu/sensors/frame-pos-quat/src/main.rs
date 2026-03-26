//! FramePos + FrameQuat Sensors
//!
//! A hinge pendulum with a site on the tip body. FramePos reads the site's
//! world-frame position, FrameQuat reads the site's world-frame orientation.
//!
//! Validates:
//! - FramePos sensor == data.site_xpos (FK pass-through)
//! - FrameQuat sensor matches data.site_xquat via rotation-distance metric
//!   (handles quaternion sign ambiguity: q and -q represent the same rotation)
//! - FramePos X-component range > 0.2m (tip actually swings)
//! - Analytical check: tip position at t=0 matches (L*sin(30°), 0, -L*cos(30°))
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
use sim_core::ENABLE_ENERGY;
use sim_core::validation::{Check, print_report};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const L: f64 = 0.5; // Rod length

const MJCF: &str = r#"
<mujoco model="frame_pos_quat">
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
    println!("=== CortenForge: FramePos + FrameQuat Sensors ===");
    println!("  Site on hinge pendulum tip, 30° initial tilt");
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
                    let pos = d.sensor_data(m, 0);
                    let quat = d.sensor_data(m, 1);
                    format!(
                        "pos=({:+.3},{:+.3},{:+.3})  q=({:+.3},{:+.3},{:+.3},{:+.3})",
                        pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3],
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

    // Initial displacement: 30°
    let qpos_adr = model.jnt_qpos_adr[0];
    data.qpos[qpos_adr] = std::f64::consts::FRAC_PI_6;
    let _ = data.forward(&model);

    // Analytical check at t=0: tip should be at (L*sin(30°), 0, -L*cos(30°))
    let expected_x = L * std::f64::consts::FRAC_PI_6.sin();
    let expected_z = -L * std::f64::consts::FRAC_PI_6.cos();
    let pos = data.sensor_data(&model, 0);
    println!(
        "  t=0 tip_pos: ({:.4}, {:.4}, {:.4})",
        pos[0], pos[1], pos[2]
    );
    println!("  t=0 expected: ({expected_x:.4}, 0.0000, {expected_z:.4})");
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
    let pos = data.sensor_data(&model, 0);
    let quat = data.sensor_data(&model, 1);
    hud.vec3("pos", pos, 4);
    hud.quat("quat", quat);
}

// ── Sensor Validation ───────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SensorValidation {
    pos_max_err: f64,
    quat_max_rot_err: f64,
    pos_x_min: f64,
    pos_x_max: f64,
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
    let site_id = model.sensor_objid[0]; // site index for framepos

    // FramePos pipeline check
    let pos_sensor = data.sensor_data(&model, 0);
    let pos_state = &data.site_xpos[site_id];
    let pos_err = ((pos_sensor[0] - pos_state.x).powi(2)
        + (pos_sensor[1] - pos_state.y).powi(2)
        + (pos_sensor[2] - pos_state.z).powi(2))
    .sqrt();
    if pos_err > val.pos_max_err {
        val.pos_max_err = pos_err;
    }

    // FrameQuat rotation-distance check
    let quat_sensor = data.sensor_data(&model, 1);
    let q_data = &data.site_xquat[site_id];
    let quat_data = [q_data.w, q_data.i, q_data.j, q_data.k];
    let rot_err = quat_rotation_distance(quat_sensor, &quat_data);
    if rot_err > val.quat_max_rot_err {
        val.quat_max_rot_err = rot_err;
    }

    // Range tracking (X-component in physics frame)
    if pos_sensor[0] < val.pos_x_min {
        val.pos_x_min = pos_sensor[0];
    }
    if pos_sensor[0] > val.pos_x_max {
        val.pos_x_max = pos_sensor[0];
    }

    // Analytical t=0 check (first frame only)
    if !val.t0_checked && data.time < 0.01 {
        val.t0_checked = true;
        let expected_x = L * std::f64::consts::FRAC_PI_6.sin();
        let expected_z = -L * std::f64::consts::FRAC_PI_6.cos();
        val.t0_pos_err = ((pos_sensor[0] - expected_x).powi(2)
            + pos_sensor[1].powi(2)
            + (pos_sensor[2] - expected_z).powi(2))
        .sqrt();
    }

    // Report
    if harness.reported() && !val.reported {
        val.reported = true;
        let pos_x_range = val.pos_x_max - val.pos_x_min;
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
                name: "Pos X range > 0.2m",
                pass: pos_x_range > 0.2,
                detail: format!(
                    "range={pos_x_range:.3}m [{:.3}, {:.3}]",
                    val.pos_x_min, val.pos_x_max
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
