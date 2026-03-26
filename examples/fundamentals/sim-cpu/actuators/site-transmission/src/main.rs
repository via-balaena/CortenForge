//! Site Transmission — Configuration-Dependent Moments
//!
//! A 2-DOF arm (shoulder + elbow) with a site-based wrench actuator applied at
//! the hand tip. As the elbow bends, the moment arm of the site force on the
//! shoulder changes, while the elbow moment remains constant.
//!
//! Analytical moments (site force = 10 N in world X, upper arm L1=0.3, forearm L2=0.2):
//! - q2=0:    shoulder moment = -0.500, elbow moment = -0.200
//! - q2=45deg: shoulder moment = -0.412, elbow moment = -0.200
//! - q2=90deg: shoulder moment = -0.200, elbow moment = -0.200
//!
//! Validates:
//! - Actuator length == 0 (site transmission has no length)
//! - Shoulder moment varies with configuration (~-0.5 at q2=0, ~-0.2 at q2=pi/2)
//! - Elbow moment stays constant (~-0.2) regardless of configuration
//! - Configuration dependence: shoulder moment actually changed between samples
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
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="upper_arm" pos="0 0 0">
      <joint name="shoulder" type="hinge" axis="0 1 0" armature="0.01" damping="5"/>
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
    <general name="site_force" site="tip_site" gear="1 0 0 0 0 0" gainprm="1"/>
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

/// Ramp duration: elbow sweeps from 0 to pi/2 over this many seconds, then holds.
const RAMP_SECONDS: f64 = 5.0;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Site Transmission ===");
    println!("  2-DOF arm with site-based wrench actuator at hand tip");
    println!("  Elbow sweeps 0 -> pi/2 over {RAMP_SECONDS}s, site force ctrl=1.0 (gain=10)");
    println!("  Shoulder moment changes with configuration; elbow moment stays constant");
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
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let site_frc = d.sensor_data(m, 0)[0];
                    let q_shoulder = d.sensor_data(m, 1)[0];
                    let q_elbow = d.sensor_data(m, 2)[0];
                    format!("site_frc={site_frc:.3}  q_sh={q_shoulder:.4}  q_el={q_elbow:.4}")
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

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, -0.2, -0.15),
        1.8,
        std::f32::consts::FRAC_PI_4,
        0.35,
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

    // Actuator 0: elbow position servo — ramp from 0 to pi/2 over RAMP_SECONDS, then hold
    let target_elbow = if time < RAMP_SECONDS {
        std::f64::consts::FRAC_PI_2 * (time / RAMP_SECONDS)
    } else {
        std::f64::consts::FRAC_PI_2
    };
    data.ctrl[0] = target_elbow;

    // Actuator 1: site force — constant ctrl = 1.0
    data.ctrl[1] = 1.0;
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Site Transmission");

    let site_frc = data.sensor_data(&model, 0)[0];
    let q_shoulder = data.sensor_data(&model, 1)[0];
    let q_elbow = data.sensor_data(&model, 2)[0];

    // Read moment arms for the site force actuator (index 1)
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

    let target_elbow = if data.time < RAMP_SECONDS {
        std::f64::consts::FRAC_PI_2 * (data.time / RAMP_SECONDS)
    } else {
        std::f64::consts::FRAC_PI_2
    };

    hud.scalar("ctrl_elbow", target_elbow, 4);
    hud.scalar("ctrl_site", 1.0, 1);
    hud.scalar("site_force", site_frc, 4);
    hud.scalar("shoulder_moment", shoulder_moment, 4);
    hud.scalar("elbow_moment", elbow_moment, 4);
    hud.scalar("q_elbow", q_elbow, 4);
    hud.scalar("q_shoulder", q_shoulder, 4);
    hud.scalar("time", data.time, 2);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SiteTransmissionValidation {
    /// Max |actuator_length[SITE_ACT_IDX]| observed
    max_length: f64,
    /// Shoulder moment sampled early (q2 near 0)
    shoulder_moment_early: Option<f64>,
    /// Elbow moment sampled early (q2 near 0)
    elbow_moment_early: Option<f64>,
    /// Shoulder moment sampled late (q2 near pi/2)
    shoulder_moment_late: Option<f64>,
    /// Elbow moment sampled late (q2 near pi/2)
    elbow_moment_late: Option<f64>,
    reported: bool,
}

fn site_diagnostics(
    _model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SiteTransmissionValidation>,
) {
    let time = data.time;

    // Skip t=0 frame
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

        // Sample early: t in [0.5, 1.5] — elbow near 0
        // Use wide window for Bevy frame timing variability
        if val.shoulder_moment_early.is_none() && (0.5..1.5).contains(&time) {
            val.shoulder_moment_early = Some(sh_moment);
            val.elbow_moment_early = Some(el_moment);
        }

        // Sample late: t in [7.0, 9.0] — elbow near pi/2 (ramp finished at t=5)
        if val.shoulder_moment_late.is_none() && (7.0..9.0).contains(&time) {
            val.shoulder_moment_late = Some(sh_moment);
            val.elbow_moment_late = Some(el_moment);
        }
    }

    // Final report
    if harness.reported() && !val.reported {
        val.reported = true;

        let sh_early = val.shoulder_moment_early.unwrap_or(f64::NAN);
        let el_early = val.elbow_moment_early.unwrap_or(f64::NAN);
        let sh_late = val.shoulder_moment_late.unwrap_or(f64::NAN);
        let el_late = val.elbow_moment_late.unwrap_or(f64::NAN);

        // Check 1: actuator_length == 0
        let length_ok = val.max_length < 1e-12;

        // Check 2: shoulder moment varies
        // At q2~0: expect ~-0.500, at q2~pi/2: expect ~-0.200
        let sh_early_err = ((sh_early - (-0.5)) / (-0.5)).abs() * 100.0;
        let sh_late_err = ((sh_late - (-0.2)) / (-0.2)).abs() * 100.0;
        let shoulder_varies_ok = sh_early_err < 10.0 && sh_late_err < 10.0;

        // Check 3: elbow moment constant ~-0.200
        let el_early_err = ((el_early - (-0.2)) / (-0.2)).abs() * 100.0;
        let el_late_err = ((el_late - (-0.2)) / (-0.2)).abs() * 100.0;
        let elbow_constant_ok = el_early_err < 5.0 && el_late_err < 5.0;

        // Check 4: config-dependent — shoulder moment actually changed
        let moment_delta = (sh_early - sh_late).abs();
        let config_dependent_ok = moment_delta > 0.05;

        let checks = vec![
            Check {
                name: "Actuator length = 0",
                pass: length_ok,
                detail: format!("max |length| = {:.2e}", val.max_length),
            },
            Check {
                name: "Shoulder moment varies",
                pass: shoulder_varies_ok,
                detail: format!(
                    "early={sh_early:.4} (expect -0.500, err={sh_early_err:.1}%), \
                     late={sh_late:.4} (expect -0.200, err={sh_late_err:.1}%)"
                ),
            },
            Check {
                name: "Elbow moment constant",
                pass: elbow_constant_ok,
                detail: format!(
                    "early={el_early:.4} (err={el_early_err:.1}%), \
                     late={el_late:.4} (err={el_late_err:.1}%)"
                ),
            },
            Check {
                name: "Config-dependent",
                pass: config_dependent_ok,
                detail: format!(
                    "|delta| = {moment_delta:.4} (early={sh_early:.4}, late={sh_late:.4})"
                ),
            },
        ];
        let _ = print_report("Site Transmission (t=15s)", &checks);
    }
}
