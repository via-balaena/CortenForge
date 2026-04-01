//! Cylinder Wrap — Tendon Wrapping Around a Cylinder
//!
//! A spatial tendon wraps around a cylinder, following a helical geodesic
//! on the surface. A motor oscillates one arm so the wrap path tightens and
//! loosens visibly.
//!
//! Run with: `cargo run -p example-tendon-cylinder-wrap --release`

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
use nalgebra::Vector3;
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

/// Two arms on opposite sides of a vertical cylinder. The tendon wraps around
/// the cylinder surface when the straight-line path would penetrate. A motor
/// on the upper arm oscillates it so the wrap arc changes.
const MJCF: &str = r#"
<mujoco model="cylinder-wrap">
  <option gravity="0 0 0" timestep="0.002"/>
  <compiler angle="radian"/>

  <worldbody>
    <!-- Wrap cylinder (translucent, vertical, no collision) -->
    <body name="cyl_body" pos="0 0 0.6">
      <geom name="wrap_cyl" type="cylinder" size="0.08 0.2"
            rgba="0.2 0.8 0.3 0.3" contype="0" conaffinity="0"/>
      <site name="side" pos="0 0.1 0" size="0.005"/>
    </body>

    <!-- Upper arm (blue, motor-driven) -->
    <body name="upper" pos="-0.2 0.12 0.75">
      <joint name="j_up" type="hinge" axis="0 1 0" damping="1.0"
             stiffness="3" springref="0"/>
      <geom name="upper_geom" type="capsule" fromto="0 0 0 0.12 0 0"
            size="0.015" mass="0.5" rgba="0.3 0.5 0.85 1"/>
      <site name="s_up" pos="0.1 0 0" size="0.012" rgba="1 1 0 1"/>
    </body>

    <!-- Lower arm (red, passive) -->
    <body name="lower" pos="0.2 -0.08 0.45">
      <joint name="j_down" type="hinge" axis="0 1 0" damping="1.0"
             stiffness="3" springref="0"/>
      <geom name="lower_geom" type="capsule" fromto="0 0 0 -0.12 0 0"
            size="0.015" mass="0.5" rgba="0.85 0.25 0.2 1"/>
      <site name="s_down" pos="-0.1 0 0" size="0.012" rgba="1 1 0 1"/>
    </body>
  </worldbody>

  <tendon>
    <spatial name="cyl_tendon">
      <site site="s_up"/>
      <geom geom="wrap_cyl" sidesite="side"/>
      <site site="s_down"/>
    </spatial>
  </tendon>

  <actuator>
    <motor name="drive" joint="j_up" gear="1"/>
  </actuator>

  <sensor>
    <tendonpos name="ten_pos" tendon="cyl_tendon"/>
    <tendonvel name="ten_vel" tendon="cyl_tendon"/>
  </sensor>
</mujoco>
"#;

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Cylinder Wrap — Tendon ===");
    println!("  Two arms around a cylinder; tendon wraps as a helix");
    println!("  Tendon color: green (relaxed) / yellow / red (stretched)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge \u{2014} Cylinder Wrap (Tendons)".into(),
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
                    let ju = m.joint_id("j_up").unwrap();
                    let jd = m.joint_id("j_down").unwrap();
                    let qu = d.qpos[m.jnt_qpos_adr[ju]].to_degrees();
                    let qd = d.qpos[m.jnt_qpos_adr[jd]].to_degrees();
                    let wrap = d.ten_wrapnum[0] > 2;
                    format!(
                        "up={qu:+6.1}\u{00b0}  down={qd:+6.1}\u{00b0}  wrap={}",
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

/// Tracks the min/max tendon length over time for self-calibrating color.
/// Blue at shortest seen, green at midpoint, red at longest seen.
#[derive(Resource)]
struct TendonRange {
    min: f64,
    max: f64,
}

// ── Setup ─────────────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let data = model.make_data();
    // Don't capture initial length here — forward() hasn't been called yet,
    // so ten_length is zero. Let the range self-initialize on the first frame.

    let mat_upper =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.5, 0.85)));
    let mat_lower =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.25, 0.2)));
    let mat_cyl = materials.add(StandardMaterial {
        base_color: Color::srgba(0.2, 0.8, 0.3, 0.3),
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
            ("wrap_cyl", mat_cyl),
            ("upper_geom", mat_upper),
            ("lower_geom", mat_lower),
        ],
    );

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 0.6),
        1.5,
        std::f32::consts::FRAC_PI_2, // side view
        0.25,                        // slight elevation
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(TendonRange {
        min: f64::MAX,
        max: f64::MIN,
    });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Control ───────────────────────────────────────────────────────────────

/// Oscillate the upper arm to vary the wrap path.
fn apply_control(model: Res<PhysicsModel>, mut data: ResMut<PhysicsData>) {
    let t = data.time;
    data.ctrl[0] = 2.0 * (2.0 * std::f64::consts::PI * 0.15 * t).sin();
    let _ = &model;
}

// ── Tendon Visualization ──────────────────────────────────────────────────

/// Draw the tendon path. Between tangent points on the same geom, interpolate
/// a helix on the cylinder surface (or sphere arc — reuse the slerp approach
/// since both produce correct surface-hugging paths when projected onto the
/// geom's local frame).
fn draw_tendon_path(
    mut gizmos: Gizmos,
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut range: ResMut<TendonRange>,
) {
    let tid = model.tendon_id("cyl_tendon").expect("tendon");
    let adr = data.ten_wrapadr[tid];
    let num = data.ten_wrapnum[tid];
    if num < 2 {
        return;
    }

    let length = data.ten_length[tid];
    range.min = range.min.min(length);
    range.max = range.max.max(length);

    let span = range.max - range.min;
    let t = if span > 1e-10 {
        ((length - range.min) / span).clamp(0.0, 1.0) as f32
    } else {
        0.0
    };

    // Green → Yellow (t=0.5) → Red (t=1.0)
    let color = if t < 0.5 {
        let s = t * 2.0;
        Color::srgb(s, 0.8, 0.0)
    } else {
        let s = (t - 0.5) * 2.0;
        Color::srgb(1.0, 0.8 * (1.0 - s), 0.0)
    };

    let arc_segments: i32 = 32;

    for i in 0..num - 1 {
        let obj_a = data.wrap_obj[adr + i];
        let obj_b = data.wrap_obj[adr + i + 1];
        let pa = data.wrap_xpos[adr + i];
        let pb = data.wrap_xpos[adr + i + 1];

        if obj_a >= 0 && obj_a == obj_b {
            let gid = obj_a as usize;
            let center = data.geom_xpos[gid];
            let mat = data.geom_xmat[gid];
            let radius = model.geom_size[gid].x;

            // Transform tangent points to cylinder-local frame
            let la = mat.transpose() * (pa - center);
            let lb = mat.transpose() * (pb - center);

            // In local frame: XY is the cross-section, Z is the axis.
            // Compute angles in XY plane.
            let angle_a = la.y.atan2(la.x);
            let angle_b = lb.y.atan2(lb.x);

            // Choose the shorter arc direction
            let mut delta = angle_b - angle_a;
            if delta > std::f64::consts::PI {
                delta -= 2.0 * std::f64::consts::PI;
            } else if delta < -std::f64::consts::PI {
                delta += 2.0 * std::f64::consts::PI;
            }

            let mut prev = vec3_from_vector(&pa);
            for seg in 1..=arc_segments {
                let frac = f64::from(seg) / f64::from(arc_segments);
                let angle = angle_a + delta * frac;
                let z = la.z + (lb.z - la.z) * frac;
                let local = Vector3::new(radius * angle.cos(), radius * angle.sin(), z);
                let world = center + mat * local;
                let cur = vec3_from_vector(&world);
                gizmos.line(prev, cur, color);
                prev = cur;
            }
        } else {
            gizmos.line(vec3_from_vector(&pa), vec3_from_vector(&pb), color);
        }
    }

    for i in 0..num {
        let pos = vec3_from_vector(&data.wrap_xpos[adr + i]);
        gizmos.sphere(Isometry3d::from_translation(pos), 0.006, color);
    }
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Cylinder Wrap \u{2014} helical tendon path");

    let ju = model.joint_id("j_up").expect("j_up");
    let jd = model.joint_id("j_down").expect("j_down");
    let qu = data.qpos[model.jnt_qpos_adr[ju]];
    let qd = data.qpos[model.jnt_qpos_adr[jd]];
    let tid = model.tendon_id("cyl_tendon").expect("tendon");
    let num = data.ten_wrapnum[tid];
    let wrapping = num > 2;

    let sensor_l = data.sensor_scalar(&model, "ten_pos").unwrap_or(0.0);
    let vel = data.sensor_scalar(&model, "ten_vel").unwrap_or(0.0);

    hud.raw(format!("Upper angle: {:+7.2}\u{00b0}", qu.to_degrees()));
    hud.raw(format!("Lower angle: {:+7.2}\u{00b0}", qd.to_degrees()));
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
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut state: Local<DiagState>,
) {
    let tid = model.tendon_id("cyl_tendon").expect("tendon");
    if data.ten_wrapnum[tid] > 2 {
        state.saw_wrap = true;
    }

    if !harness.reported() || state.reported {
        return;
    }
    state.reported = true;

    let checks = vec![Check {
        name: "Cylinder wrapping engaged",
        pass: state.saw_wrap,
        detail: format!("saw wrapnum > 2: {}", state.saw_wrap),
    }];
    let _ = print_report("Cylinder Wrap (t=10s)", &checks);
}
