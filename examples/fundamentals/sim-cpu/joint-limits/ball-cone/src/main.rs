//! Ball Cone — Ball Joint Cone Limit
//!
//! A rod on a ball joint with a 30-degree cone limit, given initial angular
//! velocity to orbit along the cone boundary. Light damping slowly bleeds
//! energy, so the tip spirals inward like a coin-funnel toy — orbiting the
//! cone rim, then spiraling down to rest at the bottom. A fading trail traces
//! the path.
//!
//! Demonstrates: cone limit as a rotationally symmetric 3D surface,
//! friction-free constraint sliding, energy decay, trail visualization.
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
use sim_bevy::gizmos::{TrailGizmo, draw_trails, sample_trails};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms_with, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};

// ── MJCF Model ────────────────────────────────────────────────────────────

/// Ball joint with 30° cone limit. Very light damping (0.01) so the orbit
/// persists for ~15s before spiraling to rest at the bottom.
const MJCF: &str = r#"
<mujoco model="ball-cone">
  <compiler angle="degree"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="pivot" pos="0 0 1.5">
      <geom name="pivot_geom" type="sphere" size="0.04" rgba="0.4 0.4 0.4 1"/>
      <body name="rod" pos="0 0 0">
        <joint name="ball" type="ball" limited="true" range="0 30"
               damping="0.08"/>
        <geom name="rod_geom" type="capsule"
              fromto="0 0 0 0 0 -0.6" size="0.015" mass="0.3"
              rgba="0.85 0.5 0.15 1"/>
        <body name="tip" pos="0 0 -0.6">
          <geom name="tip_geom" type="sphere" size="0.05" mass="1.0"
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

// ── Physics constants ─────────────────────────────────────────────────────

const ROD_LENGTH: f64 = 0.6;
const GRAVITY: f64 = 9.81;
const CONE_LIMIT_DEG: f64 = 30.0;
const CONE_LIMIT_RAD: f64 = CONE_LIMIT_DEG * std::f64::consts::PI / 180.0;

/// Angular velocity for conical pendulum orbit at angle theta:
/// omega = sqrt(g / (L * cos(theta)))
fn conical_omega(theta: f64) -> f64 {
    (GRAVITY / (ROD_LENGTH * theta.cos())).sqrt()
}

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    let omega = conical_omega(CONE_LIMIT_RAD);
    let period = 2.0 * std::f64::consts::PI / omega;
    println!("=== CortenForge: Ball Cone \u{2014} 30\u{00b0} Cone Limit ===");
    println!("  Gold rod orbits along cone boundary, then spirals to rest");
    println!(
        "  L={ROD_LENGTH}m, cone={CONE_LIMIT_DEG}\u{00b0}, \u{03c9}={omega:.2} rad/s, T={period:.2}s"
    );
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge \u{2014} Ball Cone (Joint Limits)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
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
                sample_trails,
                draw_trails,
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

    // Initial tilt: exactly at cone boundary (30° about X)
    let jid = model.joint_id("ball").expect("ball joint");
    let adr = model.jnt_qpos_adr[jid];
    let half = CONE_LIMIT_RAD / 2.0;
    data.qpos[adr] = half.cos(); // w
    data.qpos[adr + 1] = half.sin(); // x
    data.qpos[adr + 2] = 0.0; // y
    data.qpos[adr + 3] = 0.0; // z

    // Initial angular velocity: omega about world Z for conical orbit.
    // Body is tilted theta about X, so world Z in body frame = [0, sin(theta), cos(theta)]
    let dof_adr = model.jnt_dof_adr[jid];
    let omega = conical_omega(CONE_LIMIT_RAD);
    data.qvel[dof_adr] = 0.0;
    data.qvel[dof_adr + 1] = omega * CONE_LIMIT_RAD.sin();
    data.qvel[dof_adr + 2] = omega * CONE_LIMIT_RAD.cos();

    let _ = data.forward(&model);

    // Materials
    let mat_gold =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.5, 0.15)));
    let mat_pivot =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.4, 0.4, 0.4)));

    spawn_model_geoms_with(
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
        |cmds, _id, name| {
            if name == "tip_geom" {
                cmds.insert(TrailGizmo::new(800, Color::srgb(0.85, 0.5, 0.15), 0.01));
            }
        },
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

    // Track max deflection after 1s (skip initial transient)
    if data.time > 1.0 {
        state.max_deflection = state.max_deflection.max(deflection);
    }
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
            name: "JointLimitFrc > 0 (cone constraint activated)",
            pass: state.max_frc > 0.0,
            detail: format!("peak limit_frc = {:.2}", state.max_frc),
        },
        Check {
            name: "Deflection decaying (energy loss via damping)",
            pass: deflection < CONE_LIMIT_DEG,
            detail: format!(
                "deflection = {deflection:.1}\u{00b0} at t=15s (should be < {CONE_LIMIT_DEG}\u{00b0})"
            ),
        },
    ];
    let _ = print_report("Ball Cone (t=15s)", &checks);
}
