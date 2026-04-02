//! Subtree Angular Momentum Sensor
//!
//! A single body on a hinge in zero gravity, spinning at constant angular
//! velocity. SubtreeAngMom measures the angular momentum about the subtree
//! center of mass — for a single rigid body this is I_com × ω, which is
//! exactly conserved when there are no external torques.
//!
//! Validates:
//! - |L| is constant over 15 seconds (drift < 1e-8)
//! - L is nonzero (sensor is actually reading something)
//! - L direction is stable (stays along the hinge axis)
//!
//! Run with: `cargo run -p example-sensor-adv-subtree-angmom --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::cast_precision_loss
)]

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
// Single body on a Y-axis hinge in zero gravity. No damping, no actuators.
// Initial angular velocity set in setup. The body spins forever and L is
// exactly conserved.
//
const MJCF: &str = r#"
<mujoco model="subtree-angmom">
    <compiler angle="radian"/>
    <option gravity="0 0 0" timestep="0.001"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <body name="rod" pos="0 0 0.5">
            <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
            <geom name="shaft" type="capsule" size="0.03"
                  fromto="0 0 0 0 0 -0.6" mass="1.0" rgba="0.48 0.48 0.50 1"/>
            <geom name="tip_a" type="sphere" size="0.05"
                  pos="0 0 0" rgba="0.2 0.5 0.85 1" mass="0.001"/>
            <geom name="tip_b" type="sphere" size="0.05"
                  pos="0 0 -0.6" rgba="0.85 0.3 0.2 1" mass="0.001"/>
        </body>
    </worldbody>

    <sensor>
        <subtreeangmom name="sub_angmom" body="rod"/>
    </sensor>
</mujoco>
"#;

const INITIAL_OMEGA: f64 = 5.0;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Subtree Angular Momentum Sensor ===");
    println!("  Single body spinning in zero gravity");
    println!("  SubtreeAngMom = I_com * omega (exactly conserved)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Subtree Angular Momentum".into(),
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
                    let l = sensor_vec3(d, m, "sub_angmom");
                    let l_mag = vec3_magnitude(&l);
                    format!("|L|={l_mag:.8}  L_y={:.8}", l[1])
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

    // Set initial angular velocity
    let jid = model.joint_id("hinge").expect("hinge");
    data.qvel[model.jnt_dof_adr[jid]] = INITIAL_OMEGA;
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} sensors\n",
        model.nbody, model.njnt, model.nsensor
    );

    let mat_shaft = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip_a =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.5, 0.85)));
    let mat_tip_b =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.3, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("shaft", mat_shaft),
            ("tip_a", mat_tip_a),
            ("tip_b", mat_tip_b),
        ],
    );

    // Camera: side view to see the spinning rod
    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.2, 0.0), // Bevy Y-up
        2.5,
        std::f32::consts::FRAC_PI_4,
        0.3,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    let l = sensor_vec3(&data, &model, "sub_angmom");
    let l_mag = vec3_magnitude(&l);

    hud.clear();
    hud.section("Subtree Angular Momentum");
    hud.vec3("L", &l, 6);
    hud.scalar("|L|", l_mag, 6);
    hud.raw(String::new());
    hud.scalar("omega", INITIAL_OMEGA, 1);
    hud.scalar("t", data.time, 2);
}

// ── Sensor Validation ───────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SensorValidation {
    l0_mag: Option<f64>,
    l0_y: Option<f64>,
    max_drift: f64,
    max_direction_err: f64,
    reported: bool,
}

fn sensor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SensorValidation>,
) {
    let l = sensor_vec3(&data, &model, "sub_angmom");
    let l_mag = vec3_magnitude(&l);

    // Record initial |L| after first step
    if val.l0_mag.is_none() && data.time > 0.001 {
        val.l0_mag = Some(l_mag);
        val.l0_y = Some(l[1]);
    }

    if let Some(l0) = val.l0_mag.filter(|&v| v > 1e-12) {
        let drift = (l_mag - l0).abs() / l0;
        val.max_drift = val.max_drift.max(drift);
    }

    // Check direction stability: L should stay along Y axis
    if l_mag > 1e-12 {
        let off_axis = l[0].hypot(l[2]) / l_mag;
        val.max_direction_err = val.max_direction_err.max(off_axis);
    }

    if harness.reported() && !val.reported {
        val.reported = true;
        let l0 = val.l0_mag.unwrap_or(0.0);

        let checks = vec![
            Check {
                name: "|L| conserved",
                pass: val.max_drift < 1e-8,
                detail: format!("max |dL|/|L0| = {:.2e}, |L0| = {l0:.8}", val.max_drift),
            },
            Check {
                name: "|L| nonzero",
                pass: l0 > 1e-6,
                detail: format!("|L0| = {l0:.8}"),
            },
            Check {
                name: "L along Y axis",
                pass: val.max_direction_err < 1e-6,
                detail: format!("max off-axis ratio = {:.2e}", val.max_direction_err),
            },
        ];
        let _ = print_report("Subtree AngMom (t=15s)", &checks);
    }
}
