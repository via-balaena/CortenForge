//! Mesh-Box — GJK/EPA Mesh Collision
//!
//! A box (0.15 x 0.1 x 0.05 half-sizes) dropped onto a mesh ground slab.
//! Demonstrates `mesh_box_contact()` via GJK/EPA. The box lands flat and
//! settles at z = half_z = 0.05.
//!
//! Run with: `cargo run -p example-mesh-collision-box --release`

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

const BOX_HALF_Z: f64 = 0.05;

/// Expected rest height: z = half_z.
const REST_HEIGHT: f64 = BOX_HALF_Z;

// ── MJCF Model ─────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="mesh-box">
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
    <body name="box" pos="0 0 0.3">
      <joint type="free"/>
      <geom name="box" type="box" size="0.15 0.1 0.05" density="1000"/>
    </body>
  </worldbody>
</mujoco>
"#;

// ── Bevy App ───────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Mesh-Box ===");
    println!("  Box (0.15×0.1×0.05) on mesh ground slab (GJK/EPA path)");
    println!("  Expected rest height: z ≈ {REST_HEIGHT:.3}");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Mesh-Box".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<MeshBoxValidation>()
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
    let mat_box = materials.add(MetalPreset::PolishedSteel.material());

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("ground", mat_ground), ("box", mat_box)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.025),
        1.5,
        0.52, // ~30° azimuth
        0.35, // ~20° elevation
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Mesh-Box");

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
struct MeshBoxValidation {
    force_sum: f64,
    force_count: u32,
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<MeshBoxValidation>,
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
        ];
        let _ = print_report("Mesh-Box (t=5s)", &checks);
    }
}
