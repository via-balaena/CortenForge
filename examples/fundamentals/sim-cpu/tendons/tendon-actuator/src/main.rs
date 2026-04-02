//! Tendon Actuator — Tendon vs Joint Transmission
//!
//! Two single-link arms side by side. Left: motor drives a spatial tendon
//! (configuration-dependent gear). Right: motor drives the joint directly
//! (constant gear). Same control signal → different response.
//!
//! Run with: `cargo run -p example-tendon-actuator --release`

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
    spawn_physics_hud, tendon_color_bipolar, validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};

// ── MJCF Model ────────────────────────────────────────────────────────────

/// Two single-link pendulums hanging under gravity. Left arm has a spatial
/// tendon from a fixed anchor above to the arm tip — the motor drives this
/// tendon (variable moment arm). Right arm has a direct joint motor
/// (constant moment arm). Higher damping for smooth, non-chaotic motion.
const MJCF: &str = r#"
<mujoco model="tendon-actuator">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <compiler angle="radian"/>

  <worldbody>
    <!-- Left arm: tendon-driven -->
    <!-- Anchor offset to the right of the pivot — creates a moment arm -->
    <site name="L_anchor" pos="-0.2 0 1.1" size="0.012" rgba="1 1 0 1"/>
    <body name="L_arm" pos="-0.4 0 1.0">
      <joint name="L_hinge" type="hinge" axis="0 1 0" damping="1.5"/>
      <geom name="L_rod" type="capsule" fromto="0 0 0 0 0 -0.5"
            size="0.025" mass="1.0" rgba="0.3 0.5 0.85 1"/>
      <body name="L_tip" pos="0 0 -0.5">
        <geom name="L_ball" type="sphere" size="0.04" mass="0.3"
              rgba="0.3 0.5 0.85 1"/>
        <site name="L_tip_site" pos="0 0 0" size="0.012" rgba="1 1 0 1"/>
      </body>
    </body>

    <!-- Right arm: joint-driven (identical geometry) -->
    <body name="R_arm" pos="0.4 0 1.0">
      <joint name="R_hinge" type="hinge" axis="0 1 0" damping="1.5"/>
      <geom name="R_rod" type="capsule" fromto="0 0 0 0 0 -0.5"
            size="0.025" mass="1.0" rgba="0.85 0.25 0.2 1"/>
      <body name="R_tip" pos="0 0 -0.5">
        <geom name="R_ball" type="sphere" size="0.04" mass="0.3"
              rgba="0.85 0.25 0.2 1"/>
      </body>
    </body>
  </worldbody>

  <tendon>
    <spatial name="L_tendon">
      <site site="L_anchor"/>
      <site site="L_tip_site"/>
    </spatial>
  </tendon>

  <actuator>
    <motor name="tendon_motor" tendon="L_tendon" gear="30"/>
    <motor name="joint_motor" joint="R_hinge" gear="3"/>
  </actuator>

  <sensor>
    <tendonpos name="L_ten_pos" tendon="L_tendon"/>
    <jointpos name="L_angle" joint="L_hinge"/>
    <jointpos name="R_angle" joint="R_hinge"/>
  </sensor>
</mujoco>
"#;

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Tendon Actuator ===");
    println!("  Left (blue): tendon transmission — variable gear ratio");
    println!("  Right (red): joint transmission — constant gear ratio");
    println!("  Same control signal \u{2192} different response");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge \u{2014} Tendon Actuator".into(),
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
                    let ls = m.joint_id("L_hinge").unwrap();
                    let rs = m.joint_id("R_hinge").unwrap();
                    let la = d.qpos[m.jnt_qpos_adr[ls]].to_degrees();
                    let ra = d.qpos[m.jnt_qpos_adr[rs]].to_degrees();
                    format!(
                        "L={la:+6.1}\u{00b0}  R={ra:+6.1}\u{00b0}  diff={:+.1}\u{00b0}",
                        la - ra
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

/// Rest length for elongation coloring.
#[derive(Resource)]
struct RestLength(f64);

// ── Setup ─────────────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let rest_l = data.ten_length[0];

    let mat_l = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.5, 0.85)));
    let mat_r = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.25, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("L_rod", mat_l.clone()),
            ("L_ball", mat_l),
            ("R_rod", mat_r.clone()),
            ("R_ball", mat_r),
        ],
    );

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 0.7),
        3.0,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(RestLength(rest_l));
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Control ───────────────────────────────────────────────────────────────

/// Same sinusoidal signal to both actuators.
fn apply_control(model: Res<PhysicsModel>, mut data: ResMut<PhysicsData>) {
    let t = data.time;
    let signal = (2.0 * std::f64::consts::PI * 0.25 * t).sin();
    data.ctrl[0] = signal; // tendon drive
    data.ctrl[1] = signal; // joint drive
    let _ = &model;
}

// ── Tendon Visualization ──────────────────────────────────────────────────

/// Draw the left arm's tendon path colored by elongation.
/// Blue (shorter than rest) → green (rest) → red (stretched).
fn draw_tendon_path(
    mut gizmos: Gizmos,
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    rest: Res<RestLength>,
) {
    let tid = model.tendon_id("L_tendon").expect("tendon");
    let delta = data.ten_length[tid] - rest.0;
    let color = tendon_color_bipolar((delta / 0.15) as f32);

    draw_tendon_segments(&mut gizmos, &data, tid, color, 0.008);
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Tendon vs Joint \u{2014} same input, different output");

    let ls = model.joint_id("L_hinge").expect("L_hinge");
    let rs = model.joint_id("R_hinge").expect("R_hinge");
    let la = data.qpos[model.jnt_qpos_adr[ls]];
    let ra = data.qpos[model.jnt_qpos_adr[rs]];

    hud.raw(format!("L (tendon):  {:+7.2}\u{00b0}", la.to_degrees()));
    hud.raw(format!("R (joint):   {:+7.2}\u{00b0}", ra.to_degrees()));
    hud.raw(format!(
        "Divergence:  {:+.2}\u{00b0}",
        (la - ra).to_degrees()
    ));
    hud.raw(format!("Control:     {:+.3}", data.ctrl[0]));
    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Default)]
struct DiagState {
    max_divergence: f64,
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut state: Local<DiagState>,
) {
    let ls = model.joint_id("L_hinge").expect("L_hinge");
    let rs = model.joint_id("R_hinge").expect("R_hinge");
    let la = data.qpos[model.jnt_qpos_adr[ls]];
    let ra = data.qpos[model.jnt_qpos_adr[rs]];
    state.max_divergence = state.max_divergence.max((la - ra).abs());

    if !harness.reported() || state.reported {
        return;
    }
    state.reported = true;

    let checks = vec![Check {
        name: "Arms diverge (different transmission)",
        pass: state.max_divergence.to_degrees() > 1.0,
        detail: format!(
            "max divergence = {:.2}\u{00b0}",
            state.max_divergence.to_degrees()
        ),
    }];
    let _ = print_report("Tendon Actuator (t=10s)", &checks);
}
