//! Mesh-on-Mesh — Hull-Hull GJK/EPA
//!
//! A mesh wedge (triangular prism) dropped onto a mesh platform (box slab).
//! Both objects are `type="mesh"` — no primitives involved. Demonstrates
//! hull-hull GJK/EPA collision with MULTICCD for face-face stability.
//!
//! The wedge is centered at its volumetric centroid so body position = CoM.
//! Centroid of triangular cross-section: z_c = (0 + 0 + 0.25)/3 ≈ 0.0833.
//! Platform top at z=0.1, so expected rest height ≈ 0.1 + 0.0833 = 0.1833.
//!
//! Run with: `cargo run -p example-mesh-collision-mesh --release`

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

/// Platform top surface z-coordinate (pos.z + half_height = 0.05 + 0.05).
const PLATFORM_TOP: f64 = 0.1;

/// Wedge centroid-to-base distance (z_centroid of triangle = h/3 = 0.25/3).
const WEDGE_CENTROID_TO_BASE: f64 = 0.25 / 3.0;

/// Expected rest height: platform top + centroid-to-base.
const REST_HEIGHT: f64 = PLATFORM_TOP + WEDGE_CENTROID_TO_BASE;

// ── MJCF Model ─────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="mesh-on-mesh">
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
    <flag multiccd="enable"/>
  </option>

  <asset>
    <!-- Platform: box mesh, 1x1x0.1 -->
    <mesh name="platform"
          vertex="-0.5 -0.5 -0.05  0.5 -0.5 -0.05  0.5 0.5 -0.05  -0.5 0.5 -0.05
                  -0.5 -0.5  0.05  0.5 -0.5  0.05  0.5 0.5  0.05  -0.5 0.5  0.05"
          face="0 2 1  0 3 2  4 5 6  4 6 7
                0 5 4  0 1 5  2 7 6  2 3 7
                0 7 3  0 4 7  1 6 5  1 2 6"/>

    <!-- Wedge: triangular prism, centered at volumetric centroid.
         Cross-section triangle (xz): (-0.15,0), (0.15,0), (0,0.25)
         Centroid z = 0.25/3 ≈ 0.0833, subtracted from all vertices. -->
    <mesh name="wedge"
          vertex="-0.15 -0.15 -0.08333   0.15 -0.15 -0.08333   0.0 -0.15  0.16667
                  -0.15  0.15 -0.08333   0.15  0.15 -0.08333   0.0  0.15  0.16667"
          face="0 1 2  3 5 4
                0 3 4  0 4 1  1 4 5  1 5 2  2 5 3  2 3 0"/>
  </asset>

  <worldbody>
    <geom type="plane" size="2 2 0.01"/>
    <geom name="platform" type="mesh" mesh="platform" pos="0 0 0.05"/>
    <body name="wedge" pos="0 0 0.3">
      <joint type="free"/>
      <geom name="wedge" type="mesh" mesh="wedge" density="1000"/>
    </body>
  </worldbody>
</mujoco>
"#;

// ── Bevy App ───────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Mesh-on-Mesh ===");
    println!("  Wedge (triangular prism) on mesh platform — hull-hull GJK/EPA");
    println!("  Expected rest height: z ≈ {REST_HEIGHT:.3}");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Mesh-on-Mesh".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<MeshMeshValidation>()
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

    let mat_platform = materials.add(MetalPreset::BrushedMetal.material());
    let mat_wedge = materials.add(MetalPreset::Anodized(Color::srgb(0.72, 0.45, 0.20)).material());

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("platform", mat_platform), ("wedge", mat_wedge)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.1),
        2.0,
        0.79, // ~45° azimuth
        0.44, // ~25° elevation
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Mesh-on-Mesh");

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
struct MeshMeshValidation {
    force_sum: f64,
    force_count: u32,
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<MeshMeshValidation>,
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

        let min_z = PLATFORM_TOP + WEDGE_CENTROID_TO_BASE - 0.005;

        let checks = vec![
            Check {
                name: "Settled",
                pass: vz.abs() < 0.01,
                detail: format!("vz={vz:.6}"),
            },
            Check {
                name: "Above platform",
                pass: z > PLATFORM_TOP,
                detail: format!("z={z:.4} (platform top={PLATFORM_TOP})"),
            },
            Check {
                name: "No interpenetration",
                pass: z > min_z,
                detail: format!("z={z:.4} (min={min_z:.4})"),
            },
            Check {
                name: "Contact force",
                pass: (0.90..=1.10).contains(&force_ratio),
                detail: format!("force/weight={force_ratio:.4}"),
            },
        ];
        let _ = print_report("Mesh-on-Mesh (t=5s)", &checks);
    }
}
