//! Elastic Bounce — Conservation Through Contact
//!
//! A ball dropped from 0.5m onto a ground plane with `solref` tuned for
//! near-perfect restitution. Total energy (KE + PE) is approximately
//! conserved through the bounce — the ball returns to nearly the same
//! height it was dropped from.
//!
//! Note: during contact, energy appears to dip because the contact spring's
//! elastic PE is not tracked by `energy_potential`. Once the ball separates,
//! total energy recovers. The true test is energy at the bounce apex.
//!
//! Validates:
//! - Energy at bounce apex within 5% of initial
//! - Restitution > 0.85 (bounce height / drop height)
//!
//! Run with: `cargo run -p example-energy-elastic-bounce --release`

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
<mujoco model="elastic-bounce">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.0005" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <default>
    <geom solref="-10000 -0.99" solimp="0.99 0.99 0.001 0.5 2"/>
  </default>

  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.01" rgba="0.3 0.3 0.3 1"/>
    <body name="ball" pos="0 0 0.55">
      <freejoint name="free"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.001 0.001 0.001"/>
      <geom name="sphere" type="sphere" size="0.05" condim="1"
            rgba="0.2 0.6 0.9 1"/>
    </body>
  </worldbody>
</mujoco>
"#;

const DROP_HEIGHT: f64 = 0.55;
const SPHERE_R: f64 = 0.05;

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Elastic Bounce — Conservation Through Contact ===");
    println!("  Ball dropped from {DROP_HEIGHT}m, elastic solref for near-perfect bounce");
    println!("  Energy dips during contact (spring PE untracked), recovers at apex");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Elastic Bounce".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<BounceValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(8.0)
                .print_every(1.0)
                .display(|_m, d| {
                    let z = d.qpos[2];
                    let vz = d.qvel[2];
                    format!(
                        "t={:.1}  z={z:.4}  vz={vz:.3}  KE={:.4}  PE={:.4}  total={:.4}",
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
                bounce_diagnostics,
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
    drop_z: f64,
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();

    data.forward(&model).expect("forward");

    let total = data.total_energy();
    let drop_z = data.qpos[2];

    println!("  Initial state:");
    println!("    height   = {drop_z:.4} m (COM)");
    println!("    KE       = {:.6} J", data.energy_kinetic);
    println!("    PE       = {:.6} J", data.energy_potential);
    println!("    total    = {total:.6} J");
    println!();

    let mat_floor = materials.add(MetalPreset::BrushedMetal.with_color(Color::srgb(0.3, 0.3, 0.3)));
    let mat_ball = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.6, 0.9)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("floor", mat_floor), ("sphere", mat_ball)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.3),
        2.0,
        std::f32::consts::FRAC_PI_2,
        0.1,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(InitialState {
        total_energy: total,
        drop_z,
    });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(
    _model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    initial: Res<InitialState>,
    val: Res<BounceValidation>,
    mut hud: ResMut<PhysicsHud>,
) {
    hud.clear();
    hud.section("Elastic Bounce");

    let z = data.qpos[2];
    let vz = data.qvel[2];
    let ke = data.energy_kinetic;
    let pe = data.energy_potential;
    let total = data.total_energy();

    hud.scalar("height", z, 4);
    hud.scalar("velocity_z", vz, 3);
    hud.scalar("KE", ke, 4);
    hud.scalar("PE", pe, 4);
    hud.scalar("total E", total, 4);

    // Energy loss from initial
    let loss_pct = if initial.total_energy.abs() > 1e-12 {
        ((initial.total_energy - total) / initial.total_energy.abs()) * 100.0
    } else {
        0.0
    };
    hud.scalar("E loss %", loss_pct, 2);

    hud.scalar("bounces", f64::from(val.bounce_count), 0);
    if val.bounce_count > 0 {
        hud.scalar("apex_1", val.first_apex_z, 4);
        hud.scalar(
            "restitution",
            restitution(val.first_apex_z, initial.drop_z),
            3,
        );
    }

    hud.scalar("time", data.time, 1);
}

// ── Helpers ───────────────────────────────────────────────────────────────

fn restitution(apex_z: f64, drop_z: f64) -> f64 {
    if drop_z > SPHERE_R {
        (apex_z - SPHERE_R) / (drop_z - SPHERE_R)
    } else {
        0.0
    }
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct BounceValidation {
    hit_ground: bool,
    rising: bool,
    prev_vz: f64,
    bounce_count: u32,
    first_apex_z: f64,
    first_apex_energy: f64,
    reported: bool,
}

fn bounce_diagnostics(
    _model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    initial: Res<InitialState>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<BounceValidation>,
) {
    if data.time < 0.01 {
        return;
    }

    let z = data.qpos[2];
    let vz = data.qvel[2];

    // Detect ground contact
    if z < SPHERE_R + 0.02 && vz < 0.0 {
        val.hit_ground = true;
    }

    // Detect rising after contact
    if val.hit_ground && vz > 0.0 {
        val.rising = true;
    }

    // Detect apex (was rising, now falling)
    if val.rising && val.prev_vz > 0.0 && vz <= 0.0 {
        val.bounce_count += 1;
        if val.bounce_count == 1 {
            val.first_apex_z = z;
            val.first_apex_energy = data.total_energy();
        }
        // Reset for next bounce
        val.hit_ground = false;
        val.rising = false;
    }

    val.prev_vz = vz;

    // Report
    if harness.reported() && !val.reported {
        val.reported = true;

        let r = restitution(val.first_apex_z, initial.drop_z);

        let energy_loss_pct = if initial.total_energy.abs() > 1e-12 {
            ((initial.total_energy - val.first_apex_energy) / initial.total_energy.abs()).abs()
                * 100.0
        } else {
            100.0
        };

        let checks = vec![
            Check {
                name: "Energy at apex < 5% loss",
                pass: val.bounce_count > 0 && energy_loss_pct < 5.0,
                detail: format!(
                    "loss={energy_loss_pct:.2}% (E_init={:.4}, E_apex={:.4})",
                    initial.total_energy, val.first_apex_energy
                ),
            },
            Check {
                name: "Restitution > 0.85",
                pass: val.bounce_count > 0 && r > 0.85,
                detail: format!(
                    "r={r:.4} (drop={:.4}, apex={:.4})",
                    initial.drop_z, val.first_apex_z
                ),
            },
            Check {
                name: "Multiple bounces visible",
                pass: val.bounce_count >= 3,
                detail: format!("count={}", val.bounce_count),
            },
        ];
        let _ = print_report("Elastic Bounce (t=8s)", &checks);
    }
}
