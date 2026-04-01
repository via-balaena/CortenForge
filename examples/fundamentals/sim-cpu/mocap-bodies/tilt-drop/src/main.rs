//! Tilt Drop — Orientation-Driven Mocap Interaction
//!
//! A mocap platform tilts via scripted `mocap_quat` updates. A ball sitting
//! on top slides off under gravity as the tilt angle increases.
//!
//! Demonstrates `mocap_quat` driving contact interaction with dynamic bodies
//! through geometry change rather than surface velocity.
//!
//! Run with: `cargo run -p example-mocap-bodies-tilt-drop --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::float_cmp,
    clippy::panic,
    clippy::cast_sign_loss
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::convert::physics_pos;
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

// ── MJCF Model ────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="tilt-drop">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4"/>

  <worldbody>
    <geom type="plane" size="5 5 0.1" rgba="0.3 0.3 0.3 1"/>

    <body name="platform" mocap="true" pos="0 0 1.0">
      <geom name="plat_geom" type="box" size="0.6 0.4 0.03" mass="5.0"
            rgba="0.9 0.55 0.1 1"/>
    </body>

    <body name="ball" pos="0 0 1.12">
      <freejoint name="free"/>
      <geom name="ball_geom" type="sphere" size="0.08" mass="0.3"
            rgba="0.2 0.4 0.85 1"/>
    </body>
  </worldbody>
</mujoco>
"#;

// ── Constants ─────────────────────────────────────────────────────────────

/// Maximum tilt angle (rad). ~35 degrees — enough to slide a ball off.
const MAX_TILT: f64 = 0.6;
/// Time to reach max tilt (seconds).
const TILT_TIME: f64 = 16.0;

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Tilt Drop — Orientation-Driven Mocap ===");
    println!("  Orange platform = mocap body (tilting via mocap_quat)");
    println!("  Blue sphere = dynamic ball (slides off under gravity)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Tilt Drop (Mocap Bodies)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(20.0)
                .print_every(1.0)
                .display(|m, d| {
                    let plid = m.body_id("platform").expect("platform exists");
                    let blid = m.body_id("ball").expect("ball exists");
                    let mocap_idx = m.body_mocapid[plid].expect("platform is mocap");
                    let tilt = d.mocap_quat[mocap_idx].angle();
                    let ball_z = d.xpos[blid].z;
                    format!("tilt={tilt:.2} rad  ball_z={ball_z:.3}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (drive_tilt, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

// ── Setup ─────────────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let data = model.make_data();

    println!(
        "  Model: {} bodies, {} DOFs, {} mocap",
        model.nbody, model.nv, model.nmocap,
    );

    let mat_plat =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.55, 0.1)));
    let mat_ball =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.4, 0.85)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("plat_geom", mat_plat), ("ball_geom", mat_ball)],
    );

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 1.0),
        3.5,
        std::f32::consts::FRAC_PI_2,
        0.2,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Tilt Drive ────────────────────────────────────────────────────────────

/// Tilt the platform about Y axis. Linear ramp then hold.
fn drive_tilt(model: Res<PhysicsModel>, mut data: ResMut<PhysicsData>) {
    let plid = model.body_id("platform").expect("platform exists");
    let mocap_idx = model.body_mocapid[plid].expect("platform is mocap");
    let t = data.time;
    let angle = MAX_TILT * (t / TILT_TIME).min(1.0);
    data.mocap_quat[mocap_idx] =
        nalgebra::UnitQuaternion::from_axis_angle(&nalgebra::Vector3::y_axis(), angle);
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Tilt Drop — Orientation-Driven Mocap");

    let plid = model.body_id("platform").expect("platform exists");
    let blid = model.body_id("ball").expect("ball exists");
    let mocap_idx = model.body_mocapid[plid].expect("platform is mocap");
    let tilt = data.mocap_quat[mocap_idx].angle();
    let ball_pos = data.xpos[blid];

    hud.scalar("tilt (rad)", tilt, 3);
    hud.scalar("tilt (deg)", tilt.to_degrees(), 1);
    hud.raw(format!(
        "ball: [{:.3}, {:.3}, {:.3}]",
        ball_pos.x, ball_pos.y, ball_pos.z
    ));
    hud.scalar("ball z", ball_pos.z, 3);
    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Default)]
struct DiagState {
    ball_fell: bool,
    mocap_drift: f64,
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut state: Local<DiagState>,
) {
    // FK invariant: xquat must match mocap_quat.
    let plid = model.body_id("platform").expect("platform exists");
    let blid = model.body_id("ball").expect("ball exists");
    let mocap_idx = model.body_mocapid[plid].expect("platform is mocap");
    let fk_err = data.xquat[plid].angle_to(&data.mocap_quat[mocap_idx]);
    if fk_err > state.mocap_drift {
        state.mocap_drift = fk_err;
    }

    // Detect if ball has fallen off (below platform height).
    if data.xpos[blid].z < 0.5 {
        state.ball_fell = true;
    }

    if harness.reported() && !state.reported {
        state.reported = true;

        let checks = vec![
            Check {
                name: "Platform orientation matches mocap_quat (FK invariant)",
                pass: state.mocap_drift < 1e-10,
                detail: format!("max angle drift = {:.2e} rad", state.mocap_drift),
            },
            Check {
                name: "Ball slid off tilted platform",
                pass: state.ball_fell,
                detail: format!(
                    "ball_z = {:.3}, fell = {}",
                    data.xpos[blid].z, state.ball_fell
                ),
            },
        ];
        let _ = print_report("Tilt Drop (t=20s)", &checks);
    }
}
