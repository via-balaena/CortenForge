//! Mesh-Capsule — BVH-Accelerated Mesh Collision
//!
//! A capsule lying on its side (axis along Y) on a mesh ground slab.
//! Demonstrates `mesh_capsule_contact()` with BVH triangle acceleration.
//! The curved barrel surface rests against the mesh triangles at z = radius = 0.1.
//!
//! Run with: `cargo run -p example-mesh-collision-capsule --release`

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
    ContactForceAccumulator, PhysicsHud, ValidationHarness, accumulate_contact_force,
    render_physics_hud, spawn_example_camera, spawn_physics_hud, validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};

// ── Geometry Constants ─────────────────────────────────────────────────────

const CAPSULE_RADIUS: f64 = 0.1;

/// Expected rest height: z = radius (lying on side).
const REST_HEIGHT: f64 = CAPSULE_RADIUS;

// ── MJCF Model ─────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="mesh-capsule">
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

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
    <body name="capsule" pos="0 0 0.2" quat="0.707 0.707 0 0">
      <joint type="free"/>
      <geom name="capsule" type="capsule" size="0.1 0.2" density="1000"/>
    </body>
  </worldbody>
</mujoco>
"#;

// ── Bevy App ───────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Mesh-Capsule ===");
    println!("  Capsule (r=0.1, half_len=0.2) lying on side, on mesh ground slab");
    println!("  Expected rest height: z ≈ {REST_HEIGHT:.3}");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Mesh-Capsule".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .insert_resource(ContactForceAccumulator::new(1, 3.0, 5.0))
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
                accumulate_contact_force,
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

    println!("  Model: {} bodies, {} geoms\n", model.nbody, model.ngeom);

    let mat_ground = materials.add(MetalPreset::BrushedMetal.material());
    let mat_capsule =
        materials.add(MetalPreset::Anodized(Color::srgb(0.72, 0.45, 0.20)).material());

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("ground", mat_ground), ("capsule", mat_capsule)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.05),
        1.5,
        std::f32::consts::FRAC_PI_4, // 45° azimuth
        0.17,                        // ~10° elevation
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ────────────────────────────────────────────────────────────────────

fn update_hud(data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Mesh-Capsule");

    let z = data.xpos[1].z;
    let vz = data.cvel[1][5];
    let ncon = data.contacts.len();

    hud.scalar("z", z, 3);
    hud.scalar("vz", vz, 4);
    hud.scalar("ncon", ncon as f64, 0);
    hud.scalar("time", data.time, 1);
}

// ── Validation ─────────────────────────────────────────────────────────────

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    acc: Res<ContactForceAccumulator>,
    mut harness: ResMut<ValidationHarness>,
) {
    if !harness.take_reported() {
        return;
    }

    let z = data.xpos[1].z;
    let vz = data.cvel[1][5];
    let force_ratio = acc.avg_force_ratio(&model);

    let checks = vec![
        Check {
            name: "Rest height",
            pass: (z - REST_HEIGHT).abs() < 0.005,
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
    let _ = print_report("Mesh-Capsule (t=5s)", &checks);
}
