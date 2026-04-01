//! Push Object — Mocap Contact Interaction
//!
//! A mocap paddle (capsule) sweeps across the scene and pushes a dynamic ball
//! through contact. The mocap body has geometry that generates contacts with
//! dynamic bodies but is itself unaffected by physics.
//!
//! Demonstrates one-way contact: mocap bodies participate in collision
//! detection but contact forces only act on the dynamic side.
//!
//! Run with: `cargo run -p example-mocap-bodies-push-object --release`

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
<mujoco model="push-object">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4"/>

  <worldbody>
    <geom type="plane" size="5 5 0.1" rgba="0.3 0.3 0.3 1"/>

    <body name="paddle" mocap="true" pos="-1.5 0 0.15">
      <geom name="paddle_geom" type="capsule" size="0.06" fromto="0 -0.4 0 0 0.4 0"
            mass="10.0" rgba="0.85 0.2 0.15 1"/>
    </body>

    <body name="ball" pos="0 0 0.15">
      <freejoint name="free"/>
      <geom name="ball_geom" type="sphere" size="0.12" mass="0.5"
            rgba="0.2 0.4 0.85 1"/>
    </body>
  </worldbody>
</mujoco>
"#;

// ── Constants ─────────────────────────────────────────────────────────────

/// Paddle sweep speed (m/s).
const SWEEP_SPEED: f64 = 1.0;
/// Paddle start X position.
const START_X: f64 = -1.5;
/// Paddle Z height (center of capsule).
const PADDLE_Z: f64 = 0.15;

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Push Object — Mocap Contact Interaction ===");
    println!("  Red capsule = mocap paddle (linear sweep)");
    println!("  Blue sphere = dynamic ball (pushed by contact)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Push Object (Mocap Bodies)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(8.0)
                .print_every(1.0)
                .display(|m, d| {
                    let pid = m.body_id("paddle").expect("paddle exists");
                    let bid = m.body_id("ball").expect("ball exists");
                    let mocap_idx = m.body_mocapid[pid].expect("paddle is mocap");
                    let paddle_x = d.mocap_pos[mocap_idx].x;
                    let ball_x = d.xpos[bid].x;
                    format!("paddle_x={paddle_x:.2}  ball_x={ball_x:.2}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (drive_paddle, step_physics_realtime).chain())
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

    let mat_paddle =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.2, 0.15)));
    let mat_ball =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.4, 0.85)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("paddle_geom", mat_paddle), ("ball_geom", mat_ball)],
    );

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 0.5),
        4.0,
        std::f32::consts::FRAC_PI_2,
        0.3,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Paddle Drive ──────────────────────────────────────────────────────────

/// Sweep the paddle along +X at constant speed.
fn drive_paddle(model: Res<PhysicsModel>, mut data: ResMut<PhysicsData>) {
    let pid = model.body_id("paddle").expect("paddle exists");
    let mocap_idx = model.body_mocapid[pid].expect("paddle is mocap");
    let t = data.time;
    let x = START_X + SWEEP_SPEED * t;
    data.mocap_pos[mocap_idx] = nalgebra::Vector3::new(x, 0.0, PADDLE_Z);
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Push Object — Mocap Contact");

    let pid = model.body_id("paddle").expect("paddle exists");
    let bid = model.body_id("ball").expect("ball exists");
    let mocap_idx = model.body_mocapid[pid].expect("paddle is mocap");
    let paddle_x = data.mocap_pos[mocap_idx].x;
    let ball_pos = data.xpos[bid];

    hud.scalar("paddle X", paddle_x, 3);
    hud.raw(format!(
        "ball: [{:.3}, {:.3}, {:.3}]",
        ball_pos.x, ball_pos.y, ball_pos.z
    ));
    #[allow(clippy::cast_precision_loss)]
    hud.scalar("contacts", data.ncon as f64, 0);
    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Default)]
struct DiagState {
    ball_initial_x: Option<f64>,
    paddle_drift: f64,
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut state: Local<DiagState>,
) {
    let pid = model.body_id("paddle").expect("paddle exists");
    let bid = model.body_id("ball").expect("ball exists");

    // Record ball's initial X on first frame.
    if state.ball_initial_x.is_none() {
        state.ball_initial_x = Some(data.xpos[bid].x);
    }

    // Track FK invariant: xpos must match mocap_pos exactly.
    let mocap_idx = model.body_mocapid[pid].expect("paddle is mocap");
    let fk_err = (data.xpos[pid] - data.mocap_pos[mocap_idx]).norm();
    if fk_err > state.paddle_drift {
        state.paddle_drift = fk_err;
    }

    if harness.reported() && !state.reported {
        state.reported = true;

        let ball_x = data.xpos[bid].x;
        let initial_x = state.ball_initial_x.unwrap_or(0.0);
        let ball_moved = ball_x - initial_x;

        let checks = vec![
            Check {
                name: "Ball pushed in sweep direction",
                pass: ball_moved > 0.2,
                detail: format!("ball displaced {ball_moved:.3} m along X"),
            },
            Check {
                name: "Paddle xpos matches mocap_pos (FK invariant)",
                pass: state.paddle_drift < 1e-10,
                detail: format!("max drift = {:.2e}", state.paddle_drift),
            },
        ];
        let _ = print_report("Push Object (t=8s)", &checks);
    }
}
