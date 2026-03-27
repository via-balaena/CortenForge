//! SubtreeCom Sensor
//!
//! A two-link arm (shoulder + elbow hinges) with both joints displaced.
//! SubtreeCom reads the center of mass of the entire subtree rooted at the
//! upper body.
//!
//! Validates:
//! - SubtreeCom sensor == `data.subtree_com[body_id]` (FK pass-through)
//! - COM X-component range > 0.1m (chain swings, COM moves)
//! - Analytical cross-check at t=0 against hand-computed COM
//!
//! Run with: `cargo run -p example-sensor-subtree-com --release`

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

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="subtree_com">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <body name="upper" pos="0 0 0">
            <joint name="shoulder" type="hinge" axis="0 1 0" damping="0"/>
            <inertial pos="0 0 -0.2" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="upper_rod" type="capsule" size="0.02"
                  fromto="0 0 0  0 0 -0.4" rgba="0.48 0.48 0.50 1"/>
            <body name="lower" pos="0 0 -0.4">
                <joint name="elbow" type="hinge" axis="0 1 0" damping="0"/>
                <inertial pos="0 0 -0.2" mass="1.0" diaginertia="0.01 0.01 0.01"/>
                <geom name="lower_rod" type="capsule" size="0.02"
                      fromto="0 0 0  0 0 -0.4" rgba="0.48 0.48 0.50 1"/>
                <geom name="lower_tip" type="sphere" size="0.04"
                      pos="0 0 -0.4" rgba="0.2 0.5 0.85 1"/>
            </body>
        </body>
    </worldbody>

    <sensor>
        <subtreecom name="com" body="upper"/>
    </sensor>
</mujoco>
"#;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: SubtreeCom Sensor ===");
    println!("  Two-link arm: shoulder 30°, elbow 45°");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — SubtreeCom".into(),
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
                    let com_id = m.sensor_id("com").unwrap_or(0);
                    let com = d.sensor_data(m, com_id);
                    format!("com=({:+.4},{:+.4},{:+.4})", com[0], com[1], com[2])
                }),
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

    // Initial displacements: shoulder 30°, elbow 45°
    data.qpos[model.jnt_qpos_adr[0]] = std::f64::consts::FRAC_PI_6;
    data.qpos[model.jnt_qpos_adr[1]] = std::f64::consts::FRAC_PI_4;
    let _ = data.forward(&model);

    let com_id = model.sensor_id("com").unwrap_or(0);
    let com = data.sensor_data(&model, com_id);
    println!("  t=0 com: ({:.4}, {:.4}, {:.4})", com[0], com[1], com[2]);
    println!(
        "  Model: {} bodies, {} joints, {} sensors\n",
        model.nbody, model.njnt, model.nsensor
    );

    // ── Materials ────────────────────────────────────────────────────────
    let mat_upper = materials.add(MetalPreset::BrushedMetal.material());
    let mat_lower = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.5, 0.85)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("upper_rod", mat_upper),
            ("lower_rod", mat_lower),
            ("lower_tip", mat_tip),
        ],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, -0.3, 0.0),
        2.5,
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
    hud.section("SubtreeCom");
    let com_id = model.sensor_id("com").unwrap_or(0);
    let com = data.sensor_data(&model, com_id);
    hud.vec3("com", com, 4);
}

// ── Sensor Validation ───────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SensorValidation {
    max_err: f64,
    com_x_min: f64,
    com_x_max: f64,
    t0_com_err: f64,
    t0_checked: bool,
    reported: bool,
}

fn sensor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SensorValidation>,
) {
    let body_id = model.sensor_objid[0];

    // Pipeline check: sensor == subtree_com
    let com_id = model.sensor_id("com").unwrap_or(0);
    let com_sensor = data.sensor_data(&model, com_id);
    let com_state = &data.subtree_com[body_id];
    let err = ((com_sensor[0] - com_state.x).powi(2)
        + (com_sensor[1] - com_state.y).powi(2)
        + (com_sensor[2] - com_state.z).powi(2))
    .sqrt();
    if err > val.max_err {
        val.max_err = err;
    }

    // Range tracking
    if com_sensor[0] < val.com_x_min {
        val.com_x_min = com_sensor[0];
    }
    if com_sensor[0] > val.com_x_max {
        val.com_x_max = com_sensor[0];
    }

    // Analytical t=0 check: COM = (m1*p1 + m2*p2) / (m1+m2)
    //
    // Upper body (mass=1): inertial at (0,0,-0.2) in body frame.
    //   Shoulder θ₁=π/6 around Y: world pos = (-0.2·sin(θ₁), 0, -0.2·cos(θ₁))
    //
    // Lower body (mass=1): inertial at (0,0,-0.2) in body frame.
    //   Body origin in world = R(θ₁)·(0,0,-0.4).
    //   Total rotation = θ₁+θ₂ = π/6+π/4 = 5π/12.
    //   World pos = origin + R(θ₁+θ₂)·(0,0,-0.2).
    if !val.t0_checked && data.time < 0.01 {
        val.t0_checked = true;
        let theta1 = std::f64::consts::FRAC_PI_6;
        let theta2 = std::f64::consts::FRAC_PI_4;
        // Upper body inertial world position
        let p1_x = -0.2 * theta1.sin();
        let p1_z = -0.2 * theta1.cos();
        // Lower body origin in world
        let lo_x = -0.4 * theta1.sin();
        let lo_z = -0.4 * theta1.cos();
        // Lower body inertial world position (total rotation θ₁+θ₂)
        let total = theta1 + theta2;
        let p2_x = lo_x + -0.2 * total.sin();
        let p2_z = lo_z + -0.2 * total.cos();
        // COM = average (equal masses)
        let expected_x = f64::midpoint(p1_x, p2_x);
        let expected_z = f64::midpoint(p1_z, p2_z);
        val.t0_com_err = ((com_sensor[0] - expected_x).powi(2)
            + com_sensor[1].powi(2)
            + (com_sensor[2] - expected_z).powi(2))
        .sqrt();
    }

    // Report
    if harness.reported() && !val.reported {
        val.reported = true;
        let range = val.com_x_max - val.com_x_min;
        let checks = vec![
            Check {
                name: "SubtreeCom == data",
                pass: val.max_err < 1e-12,
                detail: format!("max error={:.2e}", val.max_err),
            },
            Check {
                name: "COM X range > 0.1m",
                pass: range > 0.1,
                detail: format!(
                    "range={range:.3}m [{:.3}, {:.3}]",
                    val.com_x_min, val.com_x_max
                ),
            },
            Check {
                name: "t=0 analytical COM",
                pass: val.t0_com_err < 1e-6,
                detail: format!("error={:.2e}m", val.t0_com_err),
            },
        ];
        let _ = print_report("SubtreeCom (t=15s)", &checks);
    }
}
