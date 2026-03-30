//! Damped Decay — Energy Dissipation
//!
//! A damped hinge pendulum released from 45 degrees. Joint damping
//! continuously removes energy from the system — the amplitude shrinks
//! with every swing until the pendulum comes to rest at the bottom.
//!
//! Energy is monotonically decreasing: it never increases at any step.
//! This is the fundamental property of dissipative systems.
//!
//! Validates:
//! - Energy monotonically decreasing (zero violations)
//! - Final KE < 20% of peak KE
//!
//! Run with: `cargo run -p example-energy-damped-decay --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::struct_excessive_bools
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

// ── MJCF Model ────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="damped-pendulum">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.05"/>
      <inertial pos="0 0 -0.2" mass="1.0" diaginertia="0.02 0.02 0.002"/>
      <geom name="rod" type="capsule" size="0.02"
            fromto="0 0 0  0 0 -0.4" contype="0" conaffinity="0"
            rgba="0.48 0.48 0.50 1"/>
      <geom name="bob" type="sphere" size="0.04"
            pos="0 0 -0.4" contype="0" conaffinity="0"
            rgba="0.7 0.25 0.25 1"/>
    </body>
  </worldbody>

  <sensor>
    <jointpos name="jpos" joint="hinge"/>
    <jointvel name="jvel" joint="hinge"/>
  </sensor>
</mujoco>
"#;

const INIT_ANGLE: f64 = std::f64::consts::FRAC_PI_4;

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Damped Decay — Energy Dissipation ===");
    println!("  Damped pendulum, released from {INIT_ANGLE:.3} rad (45 deg)");
    println!("  Energy monotonically decreases — amplitude shrinks to zero");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Damped Decay".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<DampedValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(11.0)
                .print_every(1.0)
                .display(|_m, d| {
                    format!(
                        "t={:.1}  KE={:.6}  total={:.6}  angle={:.4}",
                        d.time,
                        d.energy_kinetic,
                        d.total_energy(),
                        d.qpos[0]
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                damped_diagnostics,
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

    data.qpos[0] = INIT_ANGLE;
    data.forward(&model).expect("forward");

    let total = data.total_energy();

    println!("  Initial state:");
    println!(
        "    angle    = {INIT_ANGLE:.4} rad ({:.1} deg)",
        INIT_ANGLE.to_degrees()
    );
    println!("    KE       = {:.6} J", data.energy_kinetic);
    println!("    PE       = {:.6} J", data.energy_potential);
    println!("    total    = {total:.6} J");
    println!("    damping  = 0.05 N*m*s/rad");
    println!();

    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_bob =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.7, 0.25, 0.25)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("rod", mat_rod), ("bob", mat_bob)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, -0.2),
        1.5,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    val: Res<DampedValidation>,
    mut hud: ResMut<PhysicsHud>,
) {
    hud.clear();
    hud.section("Damped Decay — Dissipation");

    let ke = data.energy_kinetic;
    let pe = data.energy_potential;
    let total = data.total_energy();

    hud.scalar("KE", ke, 6);
    hud.scalar("PE", pe, 6);
    hud.scalar("total E", total, 6);

    hud.scalar("KE_max", val.ke_max, 6);
    let ke_pct = if val.ke_max > 1e-12 {
        (ke / val.ke_max) * 100.0
    } else {
        0.0
    };
    hud.scalar("KE % of max", ke_pct, 1);
    hud.scalar("violations", f64::from(val.violations), 0);

    let angle = data.sensor_scalar(&model, "jpos").unwrap_or(0.0);
    let vel = data.sensor_scalar(&model, "jvel").unwrap_or(0.0);
    hud.scalar("angle", angle, 4);
    hud.scalar("velocity", vel, 4);
    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Resource)]
struct DampedValidation {
    prev_energy: f64,
    ke_max: f64,
    violations: u32,
    reported: bool,
}

impl Default for DampedValidation {
    fn default() -> Self {
        Self {
            prev_energy: f64::MAX,
            ke_max: 0.0,
            violations: 0,
            reported: false,
        }
    }
}

fn damped_diagnostics(
    _model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<DampedValidation>,
) {
    if data.time < 0.01 {
        return;
    }

    let total = data.total_energy();
    let ke = data.energy_kinetic;

    // Track peak KE
    if ke > val.ke_max {
        val.ke_max = ke;
    }

    // Check monotonic decrease
    let increase = total - val.prev_energy;
    if increase > 1e-12 {
        val.violations += 1;
    }
    val.prev_energy = total;

    // Report
    if harness.reported() && !val.reported {
        val.reported = true;

        let ke_remaining_pct = if val.ke_max > 1e-12 {
            (ke / val.ke_max) * 100.0
        } else {
            0.0
        };

        let checks = vec![
            Check {
                name: "Energy monotonically decreasing",
                pass: val.violations == 0,
                detail: format!("violations={}", val.violations),
            },
            Check {
                name: "KE_final < 20% of KE_max",
                pass: ke_remaining_pct < 20.0,
                detail: format!(
                    "KE={ke:.8}, KE_max={:.6}, remaining={ke_remaining_pct:.2}%",
                    val.ke_max
                ),
            },
        ];
        let _ = print_report("Damped Decay (t=11s)", &checks);
    }
}
