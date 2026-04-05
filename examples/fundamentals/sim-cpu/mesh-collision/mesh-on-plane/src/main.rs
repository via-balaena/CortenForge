//! Mesh-on-Plane — Simplest Mesh Collision
//!
//! A convex tetrahedron defined by 4 vertices and 4 triangular faces, dropped
//! onto an infinite ground plane. Demonstrates `type="mesh"` with inline
//! vertex/face data and the `collide_mesh_plane()` narrow-phase.
//!
//! The tetrahedron is centered at its volumetric centroid so the body position
//! tracks the center of mass. Expected rest height: centroid-to-base distance
//! = sqrt(6)/12 ≈ 0.204 m (for edge length 1).
//!
//! Run with: `cargo run -p example-mesh-collision-plane --release`

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
//
// Regular tetrahedron, edge length 1, centered at volumetric centroid.
//
// Raw vertices (origin at v0):
//   v0 = (0, 0, 0)
//   v1 = (1, 0, 0)
//   v2 = (0.5, sqrt(3)/2, 0)           = (0.5, 0.86603, 0)
//   v3 = (0.5, sqrt(3)/6, sqrt(6)/3)   = (0.5, 0.28868, 0.81650)
//
// Centroid = average of 4 vertices = (0.5, 0.28868, 0.20412)
//
// Centered vertices (subtract centroid):
//   v0 = (-0.5,   -0.28868, -0.20412)
//   v1 = ( 0.5,   -0.28868, -0.20412)
//   v2 = ( 0.0,    0.57735, -0.20412)
//   v3 = ( 0.0,    0.0,      0.61237)
//
// Base face (v0,v1,v2) is at z = -0.20412 in local frame.
// When base rests on ground (world z=0), body center is at z ≈ 0.204.

/// Expected rest height: centroid-to-base distance = sqrt(6)/12.
const REST_HEIGHT: f64 = 0.204_124;

// ── MJCF Model ─────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="mesh-on-plane">
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

  <asset>
    <mesh name="tetra"
          vertex="-0.5 -0.28868 -0.20412
                   0.5 -0.28868 -0.20412
                   0.0  0.57735 -0.20412
                   0.0  0.0      0.61237"
          face="0 2 1  0 1 3  1 2 3  0 3 2"/>
  </asset>

  <worldbody>
    <geom type="plane" size="2 2 0.01"/>
    <body name="tetra" pos="0 0 0.5">
      <joint type="free"/>
      <geom name="tetra" type="mesh" mesh="tetra" density="1000"/>
    </body>
  </worldbody>
</mujoco>
"#;

// ── Bevy App ───────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Mesh-on-Plane ===");
    println!("  Tetrahedron (4 verts, 4 faces) on infinite ground plane");
    println!("  Expected rest height: z ≈ {REST_HEIGHT:.3}");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Mesh-on-Plane".into(),
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
                .display(|m, d| {
                    let z = d.xpos[1].z;
                    let ncon = d.contacts.len();
                    let _ = m;
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

    println!(
        "  Model: {} bodies, {} geoms, mesh has {} vertices\n",
        model.nbody,
        model.ngeom,
        model.mesh_data.first().map_or(0, |m| m.vertices().len())
    );

    let mat_tetra = materials.add(MetalPreset::Anodized(Color::srgb(0.72, 0.45, 0.20)).material());

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("tetra", mat_tetra)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.15),
        2.0,
        std::f32::consts::FRAC_PI_4, // 45° azimuth
        0.35,                        // ~20° elevation
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ────────────────────────────────────────────────────────────────────

fn update_hud(data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Mesh-on-Plane");

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
    let ncon = data.contacts.len();
    let force_ratio = acc.avg_force_ratio(&model);

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
            name: "Contacts exist",
            pass: ncon >= 1,
            detail: format!("ncon={ncon}"),
        },
    ];
    let _ = print_report("Mesh-on-Plane (t=5s)", &checks);
}
