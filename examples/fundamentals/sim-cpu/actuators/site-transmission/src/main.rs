//! Site Transmission — Configuration-Dependent Moment Arms
//!
//! A 2-DOF arm (shoulder + elbow) with a site-based wrench actuator at the
//! hand tip. The actuator applies a force in the tip's local X direction.
//! The effective torque on the shoulder depends on the arm's configuration:
//!
//! - Elbow straight (0 deg): full arm is the lever — large shoulder torque
//! - Elbow bent (90 deg): only the upper arm contributes — smaller shoulder torque
//! - Elbow torque: always constant at L2 * force, regardless of configuration
//!
//! A passive spring on the shoulder converts this varying torque into visible
//! deflection. The shoulder angle "breathes" as the elbow oscillates — same
//! force, different lever, different torque.
//!
//! Validates:
//! - Actuator length == 0 (site transmission has no length)
//! - Shoulder moment arm varies with configuration
//! - Elbow moment arm stays constant (~0.200)
//! - Configuration dependence: shoulder moment arm actually changed
//!
//! Run with: `cargo run -p example-actuator-site-transmission --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
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

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="site-transmission">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="upper_arm" pos="0 0 0">
      <joint name="shoulder" type="hinge" axis="0 1 0"
             stiffness="10" damping="1.5" armature="0.01"/>
      <inertial pos="0 0 -0.15" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="upper_rod" type="capsule" size="0.02"
            fromto="0 0 0  0 0 -0.3" rgba="0.48 0.48 0.50 1"/>
      <body name="forearm" pos="0 0 -0.3">
        <joint name="elbow" type="hinge" axis="0 1 0" armature="0.005"/>
        <inertial pos="0 0 -0.1" mass="0.5" diaginertia="0.002 0.002 0.002"/>
        <geom name="lower_rod" type="capsule" size="0.015"
              fromto="0 0 0  0 0 -0.2" rgba="0.48 0.48 0.50 1"/>
        <geom name="hand" type="sphere" size="0.03"
              pos="0 0 -0.2" rgba="0.9 0.2 0.5 1"/>
        <site name="tip_site" pos="0 0 -0.2"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="elbow_servo" joint="elbow" kp="20" dampratio="1"/>
    <general name="site_force" site="tip_site" gear="1 0 0 0 0 0" gainprm="10"/>
  </actuator>

  <sensor>
    <actuatorfrc name="site_frc" actuator="site_force"/>
    <jointpos name="q_shoulder" joint="shoulder"/>
    <jointpos name="q_elbow" joint="elbow"/>
  </sensor>
</mujoco>
"#;

/// Site force actuator index (elbow servo is 0, site force is 1).
const SITE_ACT_IDX: usize = 1;

/// Elbow oscillation period in seconds.
const PERIOD: f64 = 10.0;

/// Site force magnitude: gain * ctrl = 10 N.
const FORCE_N: f64 = 10.0;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Site Transmission ===");
    println!("  2-DOF arm with site-based wrench actuator at hand tip");
    println!("  Elbow oscillates 0 <-> 90 deg (period {PERIOD}s), site force = {FORCE_N} N");
    println!("  Shoulder deflection changes with configuration — same force, different lever");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge \u{2014} Site Transmission".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<SiteTransmissionValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(17.0)
                .print_every(1.0)
                .display(|m, d| {
                    let q_shoulder = d.sensor_data(m, 1)[0];
                    let q_elbow = d.sensor_data(m, 2)[0];
                    let sh_moment =
                        if d.actuator_moment.len() > 1 && !d.actuator_moment[1].is_empty() {
                            d.actuator_moment[1][0]
                        } else {
                            0.0
                        };
                    format!(
                        "q_sh={:+6.1} deg  q_el={:5.1} deg  arm_sh={sh_moment:.4}",
                        q_shoulder.to_degrees(),
                        q_elbow.to_degrees(),
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_ctrl, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                site_diagnostics,
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
    let data = model.make_data();

    println!(
        "  Model: {} bodies, {} joints, {} actuators, {} sensors\n",
        model.nbody, model.njnt, model.nu, model.nsensor
    );

    let mat_upper = materials.add(MetalPreset::BrushedMetal.material());
    let mat_lower = materials.add(MetalPreset::BrushedMetal.material());
    let mat_hand = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.2, 0.5)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("upper_rod", mat_upper),
            ("lower_rod", mat_lower),
            ("hand", mat_hand),
        ],
    );

    // Multi-body arm: angled view to see both links
    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, -0.2, 0.0),
        1.8,
        std::f32::consts::FRAC_PI_3,
        0.15,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Control ─────────────────────────────────────────────────────────────────

fn apply_ctrl(mut data: ResMut<PhysicsData>) {
    let time = data.time;
    if data.ctrl.len() < 2 {
        return;
    }

    // Actuator 0: elbow servo — oscillate 0 to pi/2 with period PERIOD
    let target_elbow =
        std::f64::consts::FRAC_PI_4 * (1.0 - (2.0 * std::f64::consts::PI * time / PERIOD).cos());
    data.ctrl[0] = target_elbow;

    // Actuator 1: site force — constant ctrl = 1.0
    data.ctrl[1] = 1.0;
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Site Transmission");

    let q_shoulder = data.sensor_data(&model, 1)[0];
    let q_elbow = data.sensor_data(&model, 2)[0];

    let (shoulder_moment, elbow_moment) = if data.actuator_moment.len() > SITE_ACT_IDX
        && data.actuator_moment[SITE_ACT_IDX].len() >= 2
    {
        (
            data.actuator_moment[SITE_ACT_IDX][0],
            data.actuator_moment[SITE_ACT_IDX][1],
        )
    } else {
        (0.0, 0.0)
    };

    hud.scalar("site force (N)", FORCE_N, 1);
    hud.scalar("moment arm -> shoulder (varies)", shoulder_moment, 4);
    hud.scalar("moment arm -> elbow (constant)", elbow_moment, 4);
    hud.scalar("shoulder angle (deg)", q_shoulder.to_degrees(), 1);
    hud.scalar("elbow angle (deg)", q_elbow.to_degrees(), 1);
    hud.scalar("time (s)", data.time, 2);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SiteTransmissionValidation {
    /// Max |actuator_length[SITE_ACT_IDX]| observed.
    max_length: f64,
    /// Shoulder moment arm sampled when elbow is near straight (q2 ~ 0).
    shoulder_moment_straight: Option<f64>,
    /// Elbow moment arm sampled when elbow is near straight.
    elbow_moment_straight: Option<f64>,
    /// Shoulder moment arm sampled when elbow is near 90 deg.
    shoulder_moment_bent: Option<f64>,
    /// Elbow moment arm sampled when elbow is near 90 deg.
    elbow_moment_bent: Option<f64>,
    reported: bool,
}

fn site_diagnostics(
    _model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SiteTransmissionValidation>,
) {
    let time = data.time;

    if time < 1e-6 {
        return;
    }

    // Track actuator length (should always be 0 for site transmission)
    if data.actuator_length.len() > SITE_ACT_IDX {
        let len = data.actuator_length[SITE_ACT_IDX].abs();
        if len > val.max_length {
            val.max_length = len;
        }
    }

    // Read moment arms for the site force actuator
    if data.actuator_moment.len() > SITE_ACT_IDX && data.actuator_moment[SITE_ACT_IDX].len() >= 2 {
        let sh_moment = data.actuator_moment[SITE_ACT_IDX][0];
        let el_moment = data.actuator_moment[SITE_ACT_IDX][1];

        // Sample when elbow is near straight (second cycle, t ~ 10s)
        if val.shoulder_moment_straight.is_none() && (9.5..10.5).contains(&time) {
            val.shoulder_moment_straight = Some(sh_moment);
            val.elbow_moment_straight = Some(el_moment);
        }

        // Sample when elbow is near 90 deg (t ~ 15s)
        if val.shoulder_moment_bent.is_none() && (14.5..15.5).contains(&time) {
            val.shoulder_moment_bent = Some(sh_moment);
            val.elbow_moment_bent = Some(el_moment);
        }
    }

    // Final report
    if harness.reported() && !val.reported {
        val.reported = true;

        let sh_straight = val.shoulder_moment_straight.unwrap_or(f64::NAN);
        let el_straight = val.elbow_moment_straight.unwrap_or(f64::NAN);
        let sh_bent = val.shoulder_moment_bent.unwrap_or(f64::NAN);
        let el_bent = val.elbow_moment_bent.unwrap_or(f64::NAN);

        // Check 1: actuator_length == 0
        let length_ok = val.max_length < 1e-12;

        // Check 2: shoulder moment arm varies with configuration
        // At straight: larger magnitude; at bent: smaller magnitude
        let shoulder_varies_ok = sh_straight.abs() > 0.25 && sh_bent.abs() < sh_straight.abs();

        // Check 3: elbow moment arm constant (stays near 0.200 in magnitude)
        let el_straight_err = (el_straight.abs() - 0.2).abs() / 0.2 * 100.0;
        let el_bent_err = (el_bent.abs() - 0.2).abs() / 0.2 * 100.0;
        let elbow_constant_ok = el_straight_err < 10.0 && el_bent_err < 10.0;

        // Check 4: configuration dependence — moment arm changed significantly
        let moment_delta = (sh_straight - sh_bent).abs();
        let config_dependent_ok = moment_delta > 0.05;

        let checks = vec![
            Check {
                name: "Actuator length = 0",
                pass: length_ok,
                detail: format!("max |length| = {:.2e}", val.max_length),
            },
            Check {
                name: "Shoulder moment arm varies",
                pass: shoulder_varies_ok,
                detail: format!(
                    "straight={sh_straight:.4}, bent={sh_bent:.4} \
                     (|straight| > 0.25 and > |bent|)"
                ),
            },
            Check {
                name: "Elbow moment arm constant",
                pass: elbow_constant_ok,
                detail: format!(
                    "straight={el_straight:.4} (err={el_straight_err:.1}%), \
                     bent={el_bent:.4} (err={el_bent_err:.1}%)"
                ),
            },
            Check {
                name: "Config-dependent",
                pass: config_dependent_ok,
                detail: format!(
                    "|delta| = {moment_delta:.4} (straight={sh_straight:.4}, bent={sh_bent:.4})"
                ),
            },
        ];
        let _ = print_report("Site Transmission (t=17s)", &checks);
    }
}
