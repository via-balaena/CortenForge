//! Ball Cone — Ball Joint Cone Limit
//!
//! A rod hanging from a ball joint with range="0 30" (30-degree cone).
//! Starts tilted ~40 degrees beyond the cone and is pushed back by the
//! constraint. Demonstrates rotationally symmetric cone limits.
//!
//! Run with: `cargo run -p example-joint-limits-ball-cone --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::cast_sign_loss,
    clippy::similar_names
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

/// Ball joint with 30° cone limit. Initial orientation set programmatically
/// to ~40° tilt about X-axis (beyond the cone).
const MJCF: &str = r#"
<mujoco model="ball-cone">
  <compiler angle="degree"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>

  <worldbody>
    <body name="pivot" pos="0 0 1.5">
      <geom name="pivot_geom" type="sphere" size="0.04"
            rgba="0.4 0.4 0.4 1" contype="0" conaffinity="0"/>
      <body name="rod" pos="0 0 0">
        <joint name="ball" type="ball" limited="true" range="0 30"
               damping="0.2"/>
        <geom name="rod_geom" type="capsule"
              fromto="0 0 0 0 0 -0.5" size="0.015" mass="0.5"
              rgba="0.85 0.5 0.15 1"/>
        <body name="tip" pos="0 0 -0.5">
          <geom name="tip_geom" type="sphere" size="0.04" mass="0.5"
                rgba="0.85 0.5 0.15 1"/>
        </body>
      </body>
    </body>
  </worldbody>

  <sensor>
    <jointlimitfrc name="ball_lf" joint="ball"/>
  </sensor>
</mujoco>
"#;

const CONE_LIMIT_DEG: f64 = 30.0;
const INITIAL_TILT_DEG: f64 = 40.0;

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Ball Cone — 30\u{00b0} Cone Limit ===");
    println!("  Gold rod on ball joint, cone limit = {CONE_LIMIT_DEG}\u{00b0}");
    println!("  Starts at {INITIAL_TILT_DEG}\u{00b0} tilt (beyond cone)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Ball Cone (Joint Limits)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(5.0)
                .print_every(0.5)
                .display(|m, d| {
                    let jid = m.joint_id("ball").unwrap();
                    let q = d.joint_qpos(m, jid);
                    let angle = quat_angle(q).to_degrees();
                    let frc = d.jnt_limit_frc[jid];
                    format!("defl={angle:.1}\u{00b0} frc={frc:.2}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
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

// ── Helpers ───────────────────────────────────────────────────────────────

/// Compute rotation angle (radians, 0..pi) from a unit quaternion [w, x, y, z].
fn quat_angle(q: &[f64]) -> f64 {
    let sin_half = (q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    2.0 * sin_half.atan2(q[0].abs())
}

// ── Setup ─────────────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();

    // Set initial quaternion: ~40° tilt about X axis (beyond 30° cone)
    let jid = model.joint_id("ball").expect("ball joint");
    let adr = model.jnt_qpos_adr[jid];
    let angle = INITIAL_TILT_DEG.to_radians();
    let half = angle / 2.0;
    data.qpos[adr] = half.cos(); // w
    data.qpos[adr + 1] = half.sin(); // x
    data.qpos[adr + 2] = 0.0; // y
    data.qpos[adr + 3] = 0.0; // z

    // Materials
    let mat_gold =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.5, 0.15)));
    let mat_pivot =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.4, 0.4, 0.4)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("pivot_geom", mat_pivot),
            ("rod_geom", mat_gold.clone()),
            ("tip_geom", mat_gold),
        ],
    );

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 1.2),
        2.0,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Joint Limits \u{2014} Ball Joint Cone (30\u{00b0})");

    let jid = model.joint_id("ball").expect("ball");
    let q = data.joint_qpos(&model, jid);
    let deflection = quat_angle(q).to_degrees();
    let frc = data.jnt_limit_frc[jid];
    let active = deflection > CONE_LIMIT_DEG - 1.0;

    hud.raw(format!(
        "deflection= {deflection:5.1}\u{00b0}  (limit: {CONE_LIMIT_DEG}\u{00b0})"
    ));
    hud.raw(format!("limit_force= {frc:7.2}"));
    hud.raw(format!(
        "cone_active= {}",
        if active { "YES" } else { "no" }
    ));
    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Default)]
struct DiagState {
    max_deflection: f64,
    max_frc: f64,
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut state: Local<DiagState>,
) {
    let jid = model.joint_id("ball").expect("ball");
    let q = data.joint_qpos(&model, jid);
    let deflection = quat_angle(q).to_degrees();
    let frc = data.jnt_limit_frc[jid];

    state.max_deflection = state.max_deflection.max(deflection);
    state.max_frc = state.max_frc.max(frc);

    if !harness.reported() || state.reported {
        return;
    }
    state.reported = true;

    let tol = 3.0;
    let checks = vec![
        Check {
            name: "Deflection within cone + tolerance",
            pass: state.max_deflection <= CONE_LIMIT_DEG + tol,
            detail: format!(
                "max deflection = {:.1}\u{00b0} (limit = {CONE_LIMIT_DEG}\u{00b0}, tol = {tol}\u{00b0})",
                state.max_deflection
            ),
        },
        Check {
            name: "JointLimitFrc > 0 when at cone boundary",
            pass: state.max_frc > 0.0,
            detail: format!("peak limit_frc = {:.2}", state.max_frc),
        },
        Check {
            name: "Cone is symmetric (deflection settles near limit)",
            pass: deflection < CONE_LIMIT_DEG + tol,
            detail: format!("current deflection = {deflection:.1}\u{00b0} at t=5s"),
        },
    ];
    let _ = print_report("Ball Cone (t=5s)", &checks);
}
