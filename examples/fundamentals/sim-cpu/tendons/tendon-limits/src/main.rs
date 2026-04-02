//! Tendon Limits — Range Constraints on Tendon Length
//!
//! A pendulum tethered by a spatial tendon with a max-length limit. The
//! pendulum swings freely until the tether reaches its limit — like a
//! resistance band that goes taut. Gravity pulls the pendulum back, then
//! it swings into the limit again.
//!
//! Run with: `cargo run -p example-tendon-limits --release`

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
    PhysicsHud, ValidationHarness, draw_tendon_segments, render_physics_hud, spawn_example_camera,
    spawn_physics_hud, tendon_color_ramp, validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};

// ── MJCF Model ────────────────────────────────────────────────────────────

/// Pendulum released from a tilted position. A tendon runs from a fixed
/// anchor (above and to the right) to the pendulum tip. At equilibrium
/// (hanging), the tendon is slack (~0.34). When the pendulum swings away
/// from the anchor, the tendon stretches — the limit at 0.40 catches it
/// and bounces it back. Gravity provides the restoring force.
const MJCF: &str = r#"
<mujoco model="tendon-limits">
  <option gravity="0 0 -4.0" timestep="0.002"/>
  <compiler angle="degree"/>

  <worldbody>
    <!-- Fixed anchor: above and right of the pivot. The pendulum swings
         below the pivot so the arm never passes through the anchor. -->
    <body name="anchor_body" pos="0.2 0 1.3">
      <geom name="anchor_geom" type="sphere" size="0.02"
            rgba="0.85 0.25 0.2 1" contype="0" conaffinity="0"/>
      <site name="anchor" pos="0 0 0" size="0.012" rgba="1 1 0 1"/>
    </body>

    <!-- Pendulum: starts tilted toward the anchor (short tendon) -->
    <body name="pivot" pos="0 0 1.2">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.01"
             ref="-90"/>
      <geom name="rod" type="capsule" fromto="0 0 0 0 0 -0.5"
            size="0.02" mass="0.3" rgba="0.3 0.5 0.85 1"/>
      <body name="tip" pos="0 0 -0.5">
        <geom name="ball" type="sphere" size="0.04" mass="0.5"
              rgba="0.3 0.5 0.85 1"/>
        <site name="tip_site" pos="0 0 0" size="0.012" rgba="1 1 0 1"/>
      </body>
    </body>
  </worldbody>

  <tendon>
    <spatial name="tether" limited="true" range="0.0 0.65">
      <site site="anchor"/>
      <site site="tip_site"/>
    </spatial>
  </tendon>

  <sensor>
    <tendonpos name="ten_pos" tendon="tether"/>
    <tendonlimitfrc name="ten_lf" tendon="tether"/>
  </sensor>
</mujoco>
"#;

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Tendon Limits ===");
    println!("  Pendulum tethered to a fixed anchor (max length 0.40)");
    println!("  Swings freely until the tether goes taut — then bounces back");
    println!("  Color: green (slack) → red (at limit)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge \u{2014} Tendon Limits".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(10.0)
                .print_every(0.5)
                .display(|m, d| {
                    let l = d.sensor_scalar(m, "ten_pos").unwrap_or(0.0);
                    let f = d.ten_limit_frc[0];
                    let at = if f > 0.0 { " TAUT" } else { "" };
                    format!("L={l:.3} frc={f:.1}{at}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, delayed_step)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                diagnostics,
                draw_tendon_path,
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
    let mut data = model.make_data();
    // Initialize positions so the scene renders correctly during the delay.
    data.forward(&model).expect("forward");

    let mat_arm = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.5, 0.85)));
    let mat_ball =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.5, 0.85)));
    let mat_anchor =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.25, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("rod", mat_arm),
            ("ball", mat_ball),
            ("anchor_geom", mat_anchor),
        ],
    );

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 0.9),
        2.0,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Physics with startup delay ────────────────────────────────────────────

/// Hold the pendulum in place for 1.5 seconds, then release.
fn delayed_step(
    time: Res<Time>,
    model: Res<PhysicsModel>,
    data: ResMut<PhysicsData>,
    accumulator: ResMut<PhysicsAccumulator>,
) {
    if time.elapsed_secs() < 1.5 {
        return; // hold still — let the viewer see the setup
    }
    step_physics_realtime(model, data, time, accumulator);
}

// ── Tendon Visualization ──────────────────────────────────────────────────

/// Draw the tendon from anchor to tip, colored by proximity to the limit.
/// Green when slack, red when at the limit.
fn draw_tendon_path(mut gizmos: Gizmos, model: Res<PhysicsModel>, data: Res<PhysicsData>) {
    let tid = model.tendon_id("tether").expect("tendon");
    let length = data.ten_length[tid];
    let (_, hi) = model.tendon_range[tid];

    // 0 = fully slack, 1 = at limit
    let color = tendon_color_ramp((length / hi) as f32);

    draw_tendon_segments(&mut gizmos, &data, tid, color, 0.008);
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Tendon Limits \u{2014} tether (max length 0.40)");

    let jid = model.joint_id("hinge").expect("hinge");
    let q = data.qpos[model.jnt_qpos_adr[jid]];

    let tid = model.tendon_id("tether").expect("tendon");
    let length = data.ten_length[tid];
    let (_, hi) = model.tendon_range[tid];
    let frc = data.ten_limit_frc[tid];

    let status = if frc > 0.0 { "TAUT" } else { "slack" };

    hud.raw(format!("Angle:       {:+7.2}\u{00b0}", q.to_degrees()));
    hud.raw(format!("Tendon L:    {length:.4}  (limit: {hi:.2})"));
    hud.raw(format!("Limit frc:   {frc:.2}"));
    hud.raw(format!("Status:      {status}"));
    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Default)]
struct DiagState {
    saw_limit: bool,
    saw_free: bool,
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut state: Local<DiagState>,
) {
    let tid = model.tendon_id("tether").expect("tendon");
    let frc = data.ten_limit_frc[tid];

    if frc > 0.0 {
        state.saw_limit = true;
    } else {
        state.saw_free = true;
    }

    if !harness.reported() || state.reported {
        return;
    }
    state.reported = true;

    let checks = vec![
        Check {
            name: "Limit activated (tether went taut)",
            pass: state.saw_limit,
            detail: format!("saw limit force: {}", state.saw_limit),
        },
        Check {
            name: "Free swing (tendon slack)",
            pass: state.saw_free,
            detail: format!("saw zero force: {}", state.saw_free),
        },
    ];
    let _ = print_report("Tendon Limits (t=10s)", &checks);
}
