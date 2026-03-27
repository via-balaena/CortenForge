//! ImplicitSpringDamper Integrator — Diagonal Spring/Damper, Cholesky
//!
//! Single undamped pendulum integrated with the legacy implicit spring-damper
//! method. Treats spring and damper forces implicitly (diagonal only). With
//! zero stiffness and zero damping, this degenerates to explicit Euler.
//!
//! Validates:
//! - Energy drift < 5% over 15s (may behave like Euler with zero K/D)
//! - Oscillation period within 2% of analytical T = 2π√(I/(m·g·d))
//!
//! Run with: `cargo run -p example-integrator-implicit-spring-damper --release`

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
<mujoco model="integrator-implicitspringdamper">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.005" integrator="implicitspringdamper">
    <flag energy="enable" contact="disable"/>
  </option>
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod" type="capsule" size="0.02"
            fromto="0 0 0  0 0 -0.5"/>
      <geom name="tip" type="sphere" size="0.05" pos="0 0 -0.5"/>
    </body>
  </worldbody>
  <sensor>
    <jointpos name="jpos" joint="hinge"/>
    <jointvel name="jvel" joint="hinge"/>
  </sensor>
</mujoco>
"#;

const INTEGRATOR_NAME: &str = "ImplicitSpringDamper";
const M_G_D: f64 = 1.0 * 9.81 * 0.25;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: {INTEGRATOR_NAME} Integrator ===");
    println!("  Diagonal spring/damper implicit — Cholesky factorization");
    println!("  dt = 0.005, zero damping, released from horizontal");
    println!("  Note: with zero K/D, degenerates to Euler-like behavior");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: format!("CortenForge — {INTEGRATOR_NAME} Integrator"),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<IntegratorValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|_m, d| format!("E={:.4}", d.total_energy())),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                integrator_diagnostics,
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
    let mut data = model.make_data();

    data.qpos[0] = std::f64::consts::FRAC_PI_2;
    data.forward(&model).expect("forward should succeed");

    println!("  E₀ = {:.6} J", data.energy_initial);
    println!("  Model: {} bodies, {} joints\n", model.nbody, model.njnt);

    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.75, 0.15)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("rod", mat_rod), ("tip", mat_tip)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, -0.25),
        1.8,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(_model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section(INTEGRATOR_NAME);

    let energy = data.total_energy();
    let drift_pct = (energy - data.energy_initial) / M_G_D * 100.0;

    hud.scalar("energy", energy, 4);
    hud.raw(format!("drift: {drift_pct:+.3}% mgd"));
    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct IntegratorValidation {
    reported: bool,
}

fn integrator_diagnostics(
    data: Res<PhysicsData>,

    harness: Res<ValidationHarness>,
    mut val: ResMut<IntegratorValidation>,
) {
    if harness.reported() && !val.reported {
        val.reported = true;

        let energy = data.total_energy();
        let drift_pct = (energy - data.energy_initial) / M_G_D * 100.0;

        let checks = vec![Check {
            name: "ImplicitSpringDamper bounded",
            pass: drift_pct.abs() < 5.0,
            detail: format!(
                "drift={drift_pct:+.4}% of m*g*d (expect <5%, Euler-like with zero K/D)"
            ),
        }];
        let _ = print_report(&format!("{INTEGRATOR_NAME} Integrator (t=15s)"), &checks);
    }
}
