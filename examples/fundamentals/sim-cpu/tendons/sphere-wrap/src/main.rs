//! Sphere Wrap — Tendon Wrapping Around a Sphere
//!
//! A spatial tendon wraps around a sphere when the straight-line path would
//! penetrate it. The tendon hugs the sphere surface as a great-circle arc.
//! A motor oscillates the arm so wrapping engages and disengages.
//!
//! Run with: `cargo run -p example-tendon-sphere-wrap --release`

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
use sim_bevy::convert::{physics_pos, vec3_from_vector};
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

/// Arm rotates around a sphere via a hinge. The tendon origin is on the arm
/// tip, the insertion is fixed on the opposite side. At the initial config
/// (q=0), the sites are on opposite sides of the sphere so the straight-line
/// path crosses through it — wrapping is active immediately. A motor
/// oscillates the arm so the wrap arc grows and shrinks.
const MJCF: &str = r#"
<mujoco model="sphere-wrap">
  <option gravity="0 0 0" timestep="0.002"/>
  <compiler angle="radian"/>

  <worldbody>
    <!-- Wrap sphere (translucent, no collision) -->
    <body name="wrap_body" pos="0 0 0.8">
      <geom name="wrap_sphere" type="sphere" size="0.1"
            rgba="0.9 0.85 0.2 0.3" contype="0" conaffinity="0"/>
      <!-- Sidesite above the sphere — disambiguates which side to wrap on -->
      <site name="side" pos="0 0 0.12" size="0.005"/>
    </body>

    <!-- Rotating arm with origin site on the tip -->
    <body name="arm" pos="0 0 0.8">
      <joint name="j1" type="hinge" axis="0 1 0" damping="5.0"/>
      <geom name="arm_geom" type="capsule" fromto="0 0 0 0.3 0 0"
            size="0.02" mass="0.5" rgba="0.3 0.5 0.85 1"/>
      <site name="s_origin" pos="0.2 0 0" size="0.012" rgba="1 1 0 1"/>
    </body>

    <!-- Fixed anchor on the opposite side -->
    <body name="anchor" pos="-0.2 0 0.8">
      <geom name="anchor_geom" type="sphere" size="0.015"
            rgba="0.85 0.25 0.2 1" contype="0" conaffinity="0"/>
      <site name="s_insert" pos="0 0 0" size="0.012" rgba="1 1 0 1"/>
    </body>
  </worldbody>

  <tendon>
    <spatial name="wrap_tendon">
      <site site="s_origin"/>
      <geom geom="wrap_sphere" sidesite="side"/>
      <site site="s_insert"/>
    </spatial>
  </tendon>

  <actuator>
    <motor name="drive" joint="j1" gear="1"/>
  </actuator>

  <sensor>
    <tendonpos name="ten_pos" tendon="wrap_tendon"/>
    <tendonvel name="ten_vel" tendon="wrap_tendon"/>
  </sensor>
</mujoco>
"#;

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Sphere Wrap — Tendon ===");
    println!("  Arm rotates around a sphere; tendon wraps when path crosses it");
    println!("  Tendon color: blue (shortening) / green (still) / red (lengthening)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge \u{2014} Sphere Wrap (Tendons)".into(),
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
                    let jid = m.joint_id("j1").unwrap();
                    let q = d.qpos[m.jnt_qpos_adr[jid]].to_degrees();
                    let wrap = d.ten_wrapnum[0] > 2;
                    format!(
                        "angle={q:+6.1}\u{00b0}  wrap={}",
                        if wrap { "YES" } else { "no" }
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_control, step_physics_realtime).chain())
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
    let data = model.make_data();

    let mat_arm = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.5, 0.85)));
    let mat_anchor =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.25, 0.2)));
    // Translucent yellow for the wrap sphere
    let mat_sphere = materials.add(StandardMaterial {
        base_color: Color::srgba(0.9, 0.85, 0.2, 0.3),
        alpha_mode: AlphaMode::Blend,
        ..default()
    });

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("wrap_sphere", mat_sphere),
            ("arm_geom", mat_arm),
            ("anchor_geom", mat_anchor),
        ],
    );

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 0.8),
        1.5,
        std::f32::consts::FRAC_PI_2,
        0.1,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Control ───────────────────────────────────────────────────────────────

/// Slow oscillation so the arm sweeps across the sphere.
fn apply_control(model: Res<PhysicsModel>, mut data: ResMut<PhysicsData>) {
    let t = data.time;
    // Gentle sweep around q=0 where wrapping is active. High damping (3.0)
    // keeps the motion smooth; ±3 N·m at 0.15 Hz gives ~±40° oscillation.
    data.ctrl[0] = 3.0 * (2.0 * std::f64::consts::PI * 0.15 * t).sin();
    let _ = &model;
}

// ── Tendon Visualization ──────────────────────────────────────────────────

/// Draw the spatial tendon path from wrap_xpos, colored by velocity.
/// Between tangent points on the sphere, interpolate a smooth arc instead of
/// a straight line (which would cut through the sphere).
fn draw_tendon_path(mut gizmos: Gizmos, model: Res<PhysicsModel>, data: Res<PhysicsData>) {
    let tid = model.tendon_id("wrap_tendon").expect("tendon");
    let adr = data.ten_wrapadr[tid];
    let num = data.ten_wrapnum[tid];
    if num < 2 {
        return;
    }

    let vel = data.ten_velocity[tid];
    let t = (vel / 0.05).clamp(-1.0, 1.0) as f32;

    let color = if t >= 0.0 {
        Color::srgb(t, 0.8 * (1.0 - t), 0.0)
    } else {
        let s = -t;
        Color::srgb(0.0, 0.8 * (1.0 - s), s)
    };

    let arc_segments = 24;

    for i in 0..num - 1 {
        let obj_a = data.wrap_obj[adr + i];
        let obj_b = data.wrap_obj[adr + i + 1];
        let pa = data.wrap_xpos[adr + i];
        let pb = data.wrap_xpos[adr + i + 1];

        // If both points are on the same geom, interpolate a sphere arc
        if obj_a >= 0 && obj_a == obj_b {
            let gid = obj_a as usize;
            let center = data.geom_xpos[gid];
            let radius = model.geom_size[gid].x;

            // Directions from center to each tangent point
            let da = (pa - center).normalize();
            let db = (pb - center).normalize();
            let dot = da.dot(&db).clamp(-1.0, 1.0);
            let angle = dot.acos();

            if angle.abs() < 1e-8 {
                // Degenerate — just draw a line
                gizmos.line(vec3_from_vector(&pa), vec3_from_vector(&pb), color);
            } else {
                // Slerp along the great-circle arc
                let sin_angle = angle.sin();
                let mut prev = vec3_from_vector(&pa);
                for seg in 1..=arc_segments {
                    let frac = f64::from(seg) / f64::from(arc_segments);
                    let sa = ((1.0 - frac) * angle).sin() / sin_angle;
                    let sb = (frac * angle).sin() / sin_angle;
                    let dir = da * sa + db * sb;
                    let point = center + dir * radius;
                    let cur = vec3_from_vector(&point);
                    gizmos.line(prev, cur, color);
                    prev = cur;
                }
            }
        } else {
            // Straight segment (site-to-tangent or site-to-site)
            gizmos.line(vec3_from_vector(&pa), vec3_from_vector(&pb), color);
        }
    }

    // Markers at endpoints and tangent points
    for i in 0..num {
        let pos = vec3_from_vector(&data.wrap_xpos[adr + i]);
        gizmos.sphere(Isometry3d::from_translation(pos), 0.006, color);
    }
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Sphere Wrap \u{2014} tendon wraps around sphere surface");

    let jid = model.joint_id("j1").expect("j1");
    let q = data.qpos[model.jnt_qpos_adr[jid]];
    let tid = model.tendon_id("wrap_tendon").expect("tendon");
    let num = data.ten_wrapnum[tid];
    let wrapping = num > 2;

    let sensor_l = data.sensor_scalar(&model, "ten_pos").unwrap_or(0.0);
    let vel = data.sensor_scalar(&model, "ten_vel").unwrap_or(0.0);

    hud.raw(format!("Joint angle: {:+7.2}\u{00b0}", q.to_degrees()));
    hud.raw(format!("Tendon L:    {sensor_l:.6}"));
    hud.raw(format!("Tendon V:    {vel:+.6}"));
    hud.raw(format!(
        "Wrapping:    {} ({num} path points)",
        if wrapping { "YES" } else { "no" }
    ));
    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Default)]
struct DiagState {
    saw_wrap: bool,
    saw_no_wrap: bool,
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut state: Local<DiagState>,
) {
    let tid = model.tendon_id("wrap_tendon").expect("tendon");
    let num = data.ten_wrapnum[tid];
    if num > 2 {
        state.saw_wrap = true;
    }
    if num <= 2 {
        state.saw_no_wrap = true;
    }

    if !harness.reported() || state.reported {
        return;
    }
    state.reported = true;

    let checks = vec![
        Check {
            name: "Wrapping engaged at some point",
            pass: state.saw_wrap,
            detail: format!("saw wrapnum > 2: {}", state.saw_wrap),
        },
        Check {
            name: "Wrapping disengaged at some point",
            pass: state.saw_no_wrap,
            detail: format!("saw wrapnum <= 2: {}", state.saw_no_wrap),
        },
    ];
    let _ = print_report("Sphere Wrap (t=10s)", &checks);
}
