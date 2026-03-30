//! Pendulum Energy — Potential-Kinetic Exchange
//!
//! An undamped hinge pendulum released from 45 degrees. As it swings,
//! energy flows back and forth between potential (height) and kinetic
//! (speed). Total energy stays flat — a fundamental conservation law.
//!
//! Validates:
//! - Total energy drift < 0.5% over 10s
//! - At bottom swing: KE > 0.9 * PE_drop (nearly all PE converted)
//!
//! Run with: `cargo run -p example-energy-pendulum-energy --release`

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
<mujoco model="pendulum-energy">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
      <inertial pos="0 0 -0.2" mass="1.0" diaginertia="0.02 0.02 0.002"/>
      <geom name="rod" type="capsule" size="0.02"
            fromto="0 0 0  0 0 -0.4" contype="0" conaffinity="0"
            rgba="0.48 0.48 0.50 1"/>
      <geom name="bob" type="sphere" size="0.04"
            pos="0 0 -0.4" contype="0" conaffinity="0"
            rgba="0.85 0.3 0.2 1"/>
    </body>
  </worldbody>

  <sensor>
    <jointpos name="jpos" joint="hinge"/>
    <jointvel name="jvel" joint="hinge"/>
  </sensor>
</mujoco>
"#;

const INIT_ANGLE: f64 = std::f64::consts::FRAC_PI_4; // 45 degrees

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Pendulum Energy — PE/KE Exchange ===");
    println!("  Undamped pendulum, released from {INIT_ANGLE:.3} rad (45 deg)");
    println!("  KE peaks at bottom, PE peaks at top, total stays flat");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Pendulum Energy".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<EnergyValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(11.0)
                .print_every(1.0)
                .display(|_m, d| {
                    format!(
                        "t={:.1}  KE={:.6}  PE={:.6}  total={:.6}",
                        d.time,
                        d.energy_kinetic,
                        d.energy_potential,
                        d.total_energy()
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
                energy_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

// ── Initial state ─────────────────────────────────────────────────────────

#[derive(Resource)]
struct InitialState {
    total_energy: f64,
    pe_initial: f64,
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();

    // Release from 45 degrees
    data.qpos[0] = INIT_ANGLE;
    data.forward(&model).expect("forward");

    let total = data.total_energy();
    let pe = data.energy_potential;

    println!("  Initial state:");
    println!(
        "    angle    = {INIT_ANGLE:.4} rad ({:.1} deg)",
        INIT_ANGLE.to_degrees()
    );
    println!("    KE       = {:.6} J (at rest)", data.energy_kinetic);
    println!("    PE       = {pe:.6} J");
    println!("    total    = {total:.6} J");
    println!();

    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_bob = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.3, 0.2)));

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

    commands.insert_resource(InitialState {
        total_energy: total,
        pe_initial: pe,
    });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    initial: Res<InitialState>,
    mut hud: ResMut<PhysicsHud>,
) {
    hud.clear();
    hud.section("Pendulum — PE/KE Exchange");

    let ke = data.energy_kinetic;
    let pe = data.energy_potential;
    let total = data.total_energy();
    let drift_pct = if initial.total_energy.abs() > 1e-12 {
        ((total - initial.total_energy) / initial.total_energy.abs()) * 100.0
    } else {
        0.0
    };

    // PE drop from initial position
    let pe_drop = initial.pe_initial - pe;
    let ke_ratio = if pe_drop.abs() > 1e-12 {
        ke / pe_drop
    } else {
        0.0
    };

    hud.scalar("KE", ke, 6);
    hud.scalar("PE", pe, 6);
    hud.scalar("total", total, 6);
    hud.scalar("drift %", drift_pct, 4);
    hud.scalar("KE/PE_drop", ke_ratio, 3);

    let angle = data.sensor_scalar(&model, "jpos").unwrap_or(0.0);
    let vel = data.sensor_scalar(&model, "jvel").unwrap_or(0.0);
    hud.scalar("angle", angle, 3);
    hud.scalar("velocity", vel, 3);
    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct EnergyValidation {
    max_drift_pct: f64,
    best_ke_ratio: f64,
    reported: bool,
}

fn energy_diagnostics(
    _model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    initial: Res<InitialState>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<EnergyValidation>,
) {
    if data.time < 0.01 {
        return;
    }

    // Total energy drift
    let total = data.total_energy();
    if initial.total_energy.abs() > 1e-12 {
        let drift_pct = ((total - initial.total_energy) / initial.total_energy.abs()).abs() * 100.0;
        if drift_pct > val.max_drift_pct {
            val.max_drift_pct = drift_pct;
        }
    }

    // Track best KE/PE_drop ratio (should hit ~1.0 at bottom of swing)
    let pe_drop = initial.pe_initial - data.energy_potential;
    if pe_drop > 0.01 {
        let ke_ratio = data.energy_kinetic / pe_drop;
        if ke_ratio > val.best_ke_ratio {
            val.best_ke_ratio = ke_ratio;
        }
    }

    // Report
    if harness.reported() && !val.reported {
        val.reported = true;

        let checks = vec![
            Check {
                name: "Total energy drift < 0.5%",
                pass: val.max_drift_pct < 0.5,
                detail: format!("max={:.4}%", val.max_drift_pct),
            },
            Check {
                name: "KE/PE_drop > 0.9 at bottom",
                pass: val.best_ke_ratio > 0.9,
                detail: format!("best={:.4}", val.best_ke_ratio),
            },
        ];
        let _ = print_report("Pendulum Energy (t=11s)", &checks);
    }
}
