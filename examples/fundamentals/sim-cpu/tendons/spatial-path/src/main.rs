//! Spatial Path — 3D Tendon Routed Through Sites
//!
//! A spatial tendon through 3 sites on a 2-link arm. The tendon length equals
//! the sum of Euclidean distances between consecutive sites. No wrapping —
//! pure straight-line segments. Tendon path drawn with stretch-state coloring.
//!
//! Run with: `cargo run -p example-tendon-spatial-path --release`

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

/// 2-link arm with a spatial tendon through 3 sites. Mild stiffness so the
/// tendon influences the arm without locking it. Sites offset from the joint
/// axes so the path length changes visibly with configuration.
const MJCF: &str = r#"
<mujoco model="spatial-path">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <compiler angle="radian"/>

  <worldbody>
    <body name="upper" pos="0 0 1.0">
      <joint name="shoulder" type="hinge" axis="0 1 0" damping="0.3" ref="0.8"/>
      <geom name="upper_geom" type="capsule" fromto="0 0 0 0 0 -0.5"
            size="0.03" mass="1.0" rgba="0.55 0.55 0.65 1"/>
      <site name="s_origin" pos="0.035 0 -0.05" size="0.012"
            rgba="1 1 0 1"/>
      <site name="s_mid" pos="0.035 0 -0.45" size="0.012"
            rgba="1 1 0 1"/>
      <body name="lower" pos="0 0 -0.5">
        <joint name="elbow" type="hinge" axis="0 1 0" damping="0.3" ref="-0.5"/>
        <geom name="lower_geom" type="capsule" fromto="0 0 0 0 0 -0.4"
              size="0.025" mass="0.5" rgba="0.45 0.45 0.55 1"/>
        <site name="s_insert" pos="0.03 0 -0.2" size="0.012"
              rgba="1 1 0 1"/>
      </body>
    </body>
  </worldbody>

  <tendon>
    <spatial name="path" stiffness="30" damping="1">
      <site site="s_origin"/>
      <site site="s_mid"/>
      <site site="s_insert"/>
    </spatial>
  </tendon>

  <sensor>
    <tendonpos name="ten_pos" tendon="path"/>
    <tendonvel name="ten_vel" tendon="path"/>
  </sensor>
</mujoco>
"#;

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Spatial Path — Tendon ===");
    println!("  3 sites on a 2-link arm, no wrapping");
    println!("  Tendon path: colored line (blue/green/red = compressed/relaxed/stretched)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge \u{2014} Spatial Path (Tendons)".into(),
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
                .print_every(1.0)
                .display(|m, d| {
                    let sh = m.joint_id("shoulder").unwrap();
                    let el = m.joint_id("elbow").unwrap();
                    let sa = d.qpos[m.jnt_qpos_adr[sh]].to_degrees();
                    let ea = d.qpos[m.jnt_qpos_adr[el]].to_degrees();
                    format!("shoulder={sa:+6.1}\u{00b0}  elbow={ea:+6.1}\u{00b0}")
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

    // Start with arm horizontal (shoulder at −90°), hold briefly, then drop.
    let sh = model.joint_id("shoulder").expect("shoulder");
    data.qpos[model.jnt_qpos_adr[sh]] = -std::f64::consts::FRAC_PI_2;
    data.forward(&model).expect("forward");

    let mat_upper =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.55, 0.55, 0.65)));
    let mat_lower =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.45, 0.45, 0.55)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("upper_geom", mat_upper), ("lower_geom", mat_lower)],
    );

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 0.6),
        2.5,
        std::f32::consts::FRAC_PI_2,
        0.2,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Delayed Step ─────────────────────────────────────────────────────────

/// Hold the arm still for 1.5 s so you can see the starting pose, then release.
fn delayed_step(
    model: Res<PhysicsModel>,
    data: ResMut<PhysicsData>,
    time: Res<Time>,
    accumulator: ResMut<PhysicsAccumulator>,
) {
    if time.elapsed_secs() < 1.5 {
        return;
    }
    step_physics_realtime(model, data, time, accumulator);
}

// ── Tendon Visualization ──────────────────────────────────────────────────

/// Self-calibrating length range for color mapping.
#[derive(Default)]
struct LenRange {
    min: f64,
    max: f64,
    init: bool,
}

/// Draw the spatial tendon path, colored by length.
/// Green (shortest observed) → yellow → red (longest observed).
fn draw_tendon_path(
    mut gizmos: Gizmos,
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut range: Local<LenRange>,
) {
    let tid = model.tendon_id("path").expect("tendon");
    let length = data.ten_length[tid];

    if !range.init {
        range.min = length;
        range.max = length;
        range.init = true;
    }
    range.min = range.min.min(length);
    range.max = range.max.max(length);

    let span = range.max - range.min;
    let t = if span > 1e-10 {
        ((length - range.min) / span) as f32
    } else {
        0.0
    };
    let color = tendon_color_ramp(t);

    draw_tendon_segments(&mut gizmos, &data, tid, color, 0.008);
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Spatial Path \u{2014} 3-site tendon on 2-link arm");

    let sh = model.joint_id("shoulder").expect("shoulder");
    let el = model.joint_id("elbow").expect("elbow");
    let qa = data.qpos[model.jnt_qpos_adr[sh]];
    let qb = data.qpos[model.jnt_qpos_adr[el]];

    let sensor_l = data.sensor_scalar(&model, "ten_pos").unwrap_or(0.0);

    // Compute manual length from site positions
    let s1 = model.site_id("s_origin").expect("s1");
    let s2 = model.site_id("s_mid").expect("s2");
    let s3 = model.site_id("s_insert").expect("s3");
    let d12 = (data.site_xpos[s1] - data.site_xpos[s2]).norm();
    let d23 = (data.site_xpos[s2] - data.site_xpos[s3]).norm();
    let manual_l = d12 + d23;
    let match_ok = (sensor_l - manual_l).abs() < 1e-8;

    hud.raw(format!("Shoulder:    {:+7.2}\u{00b0}", qa.to_degrees()));
    hud.raw(format!("Elbow:       {:+7.2}\u{00b0}", qb.to_degrees()));
    hud.raw(format!("Tendon L:    {sensor_l:.6}"));
    hud.raw(format!("Manual L:    {manual_l:.6}  (d12+d23)"));
    hud.raw(format!(
        "Match:       {}",
        if match_ok { "\u{2713}" } else { "\u{2717}" }
    ));
    let vel = data.sensor_scalar(&model, "ten_vel").unwrap_or(0.0);
    hud.raw(format!("Tendon V:    {vel:+.6}"));
    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Default)]
struct DiagState {
    max_err: f64,
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut state: Local<DiagState>,
) {
    let sensor_l = data.sensor_scalar(&model, "ten_pos").unwrap_or(0.0);
    let s1 = model.site_id("s_origin").expect("s1");
    let s2 = model.site_id("s_mid").expect("s2");
    let s3 = model.site_id("s_insert").expect("s3");
    let manual_l = (data.site_xpos[s1] - data.site_xpos[s2]).norm()
        + (data.site_xpos[s2] - data.site_xpos[s3]).norm();
    let err = (sensor_l - manual_l).abs();
    state.max_err = state.max_err.max(err);

    if !harness.reported() || state.reported {
        return;
    }
    state.reported = true;

    let checks = vec![Check {
        name: "TendonPos = d(s1,s2) + d(s2,s3) (every frame)",
        pass: state.max_err < 1e-8,
        detail: format!("max |error| = {:.2e}", state.max_err),
    }];
    let _ = print_report("Spatial Path (t=10s)", &checks);
}
