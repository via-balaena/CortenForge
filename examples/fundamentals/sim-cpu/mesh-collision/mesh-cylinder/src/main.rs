//! Mesh-Cylinder — GJK/EPA with MULTICCD
//!
//! An upright cylinder on a mesh ground slab. This is the "money shot" for the
//! GJK/EPA cylinder fix landed on this branch. Before, mesh-cylinder used a
//! capsule approximation (rounded caps). Now it uses exact GJK/EPA, and MULTICCD
//! provides 8 rim contact points on the flat cap for stable flat-on-flat contact.
//!
//! The cylinder drops, settles, and stays perfectly upright — no wobble, no drift.
//!
//! Run with: `cargo run -p example-mesh-collision-cylinder --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
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

// ── Geometry Constants ─────────────────────────────────────────────────────

const CYLINDER_HALF_LENGTH: f64 = 0.2;

/// Expected rest height: z = half_length (upright cylinder).
const REST_HEIGHT: f64 = CYLINDER_HALF_LENGTH;

// ── MJCF Model ─────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="mesh-cylinder">
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
    <flag multiccd="enable"/>
  </option>

  <asset>
    <mesh name="ground"
          vertex="-1 -1 -1  1 -1 -1  1 1 -1  -1 1 -1
                  -1 -1  1  1 -1  1  1 1  1  -1 1  1"
          face="0 2 1  0 3 2  4 5 6  4 6 7
                0 5 4  0 1 5  2 7 6  2 3 7
                0 7 3  0 4 7  1 6 5  1 2 6"/>
  </asset>

  <worldbody>
    <geom name="ground" type="mesh" mesh="ground" pos="0 0 -1"/>
    <body name="cylinder" pos="0 0 0.3">
      <joint type="free"/>
      <geom name="cylinder" type="cylinder" size="0.1 0.2" density="1000"/>
    </body>
  </worldbody>
</mujoco>
"#;

// ── Bevy App ───────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Mesh-Cylinder ===");
    println!("  Cylinder (r=0.1, half_len=0.2) upright on mesh ground slab");
    println!("  GJK/EPA + MULTICCD for flat-cap stability");
    println!("  Expected rest height: z ≈ {REST_HEIGHT:.3}");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Mesh-Cylinder".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<MeshCylinderValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(5.0)
                .print_every(1.0)
                .display(|_m, d| {
                    let z = d.xpos[1].z;
                    let ncon = d.contacts.len();
                    format!("z={z:.4}  ncon={ncon}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                diagnostics,
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

    println!("  Model: {} bodies, {} geoms\n", model.nbody, model.ngeom,);

    let mat_ground = materials.add(MetalPreset::BrushedMetal.material());
    let mat_cylinder = materials.add(MetalPreset::PolishedSteel.material());

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("ground", mat_ground), ("cylinder", mat_cylinder)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.1),
        1.5,
        std::f32::consts::FRAC_PI_4, // 45° azimuth
        0.44,                        // ~25° elevation
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Mesh-Cylinder");

    let z = data.xpos[1].z;
    let vz = data.cvel[1][5];
    let ncon = data.contacts.len();
    let _ = &*model;

    hud.scalar("z", z, 3);
    hud.scalar("vz", vz, 4);
    hud.scalar("ncon", ncon as f64, 0);
    hud.scalar("time", data.time, 1);
}

// ── Validation ─────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct MeshCylinderValidation {
    force_sum: f64,
    force_count: u32,
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<MeshCylinderValidation>,
) {
    let time = data.time;

    // Accumulate contact force in the 3–5s window (after settling)
    if time > 3.0 && time < 5.0 {
        for (i, ct) in data.efc_type.iter().enumerate() {
            if matches!(
                ct,
                sim_core::ConstraintType::ContactFrictionless
                    | sim_core::ConstraintType::ContactPyramidal
                    | sim_core::ConstraintType::ContactElliptic
            ) {
                val.force_sum += data.efc_force[i].abs();
            }
        }
        val.force_count += 1;
    }

    if harness.reported() && !val.reported {
        val.reported = true;

        let z = data.xpos[1].z;
        let vz = data.cvel[1][5];
        let ncon = data.contacts.len();

        // Angular velocity magnitude for rotational stability check
        let wx = data.cvel[1][0];
        let wy = data.cvel[1][1];
        let wz = data.cvel[1][2];
        let omega = (wx * wx + wy * wy + wz * wz).sqrt();

        let body_mass = model.body_mass[1];
        let weight = body_mass * 9.81;
        let avg_force = if val.force_count > 0 {
            val.force_sum / f64::from(val.force_count)
        } else {
            0.0
        };
        let force_ratio = avg_force / weight;

        let checks = vec![
            Check {
                name: "Rest height",
                pass: (z - REST_HEIGHT).abs() < 0.003,
                detail: format!(
                    "z={z:.4} (expect ≈{REST_HEIGHT:.3}, err={:.4})",
                    (z - REST_HEIGHT).abs()
                ),
            },
            Check {
                name: "Settled",
                pass: vz.abs() < 0.01,
                detail: format!("vz={vz:.6}"),
            },
            Check {
                name: "Contact force",
                pass: (0.95..=1.05).contains(&force_ratio),
                detail: format!("force/weight={force_ratio:.4}"),
            },
            Check {
                name: "MULTICCD contacts",
                pass: ncon >= 2,
                detail: format!("ncon={ncon} (need ≥2)"),
            },
            Check {
                name: "Rotational stability",
                pass: omega < 0.1,
                detail: format!("omega={omega:.6} rad/s"),
            },
        ];
        let _ = print_report("Mesh-Cylinder (t=5s)", &checks);
    }
}
