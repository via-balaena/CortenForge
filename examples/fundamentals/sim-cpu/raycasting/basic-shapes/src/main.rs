//! Basic Shapes — Shape-Level Ray Intersection
//!
//! Six static primitives arranged in a row. A downward ray from z=3 strikes
//! each one. Hit point (green dot) and surface normal (yellow arrow) are drawn
//! as gizmos. HUD shows distance and normal for every shape.
//!
//! Validates:
//! 1. All 6 rays return `Some(hit)`
//! 2. Distances match analytical expectations within 1e-3
//! 3. Hit points lie on the shape surface
//! 4. Normals are unit-length
//! 5. Normals face the ray origin
//!
//! Run: `cargo run -p example-raycasting-basic-shapes --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::option_if_let_else,
    clippy::suboptimal_flops,
    clippy::let_underscore_must_use,
    clippy::single_match_else
)]

use bevy::math::Isometry3d;
use bevy::prelude::*;
use nalgebra::{Point3, UnitVector3, Vector3};
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{PhysicsData, PhysicsModel, spawn_model_geoms, sync_geom_transforms};
use sim_core::validation::{Check, print_report};
use sim_core::{RaycastHit, raycast_scene};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="basic-shapes">
  <option gravity="0 0 0" timestep="0.002"/>
  <worldbody>
    <geom name="sphere"    type="sphere"    pos="-4 0 0.5" size="0.5"
          rgba="0.25 0.55 0.85 1"/>
    <geom name="box"       type="box"       pos="-2 0 0.5" size="0.5 0.5 0.5"
          rgba="0.85 0.35 0.25 1"/>
    <geom name="capsule"   type="capsule"   pos=" 0 0 0.8" size="0.3 0.5"
          rgba="0.30 0.75 0.40 1"/>
    <geom name="cylinder"  type="cylinder"  pos=" 2 0 0.5" size="0.3 0.5"
          rgba="0.80 0.65 0.20 1"/>
    <geom name="ellipsoid" type="ellipsoid"  pos=" 4 0 0.3" size="0.6 0.4 0.3"
          rgba="0.65 0.30 0.75 1"/>
    <geom name="ground"    type="plane"     size="6 2 0.01"
          rgba="0.25 0.25 0.28 1"/>
  </worldbody>
</mujoco>
"#;

// ── Ray targets ─────────────────────────────────────────────────────────────

/// (display_name, ray_x, expected_distance)
const TARGETS: &[(&str, f64, f64)] = &[
    ("Sphere", -4.0, 2.0),   // center z=0.5, top at z=1.0
    ("Box", -2.0, 2.0),      // center z=0.5, top at z=1.0
    ("Capsule", 0.0, 1.4),   // center z=0.8, top at z=1.6
    ("Cylinder", 2.0, 2.0),  // center z=0.5, top at z=1.0
    ("Ellipsoid", 4.0, 2.4), // center z=0.3, top at z=0.6
    ("Ground", 6.0, 3.0),    // plane at z=0
];

const RAY_Z: f64 = 3.0;

/// Cast all 6 rays and return results.
fn cast_rays(
    model: &sim_core::Model,
    data: &sim_core::Data,
) -> Vec<(&'static str, f64, Option<RaycastHit>)> {
    let dir = UnitVector3::new_normalize(-Vector3::z());
    TARGETS
        .iter()
        .map(|&(name, x, expected)| {
            let origin = Point3::new(x, 0.0, RAY_Z);
            let hit = raycast_scene(model, data, origin, dir, 20.0, None, None);
            (name, expected, hit.map(|h| h.hit))
        })
        .collect()
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Raycasting — Basic Shapes ===");
    println!("  6 primitives, each struck by a downward ray from z=3");
    println!("  Green dot = hit point  |  Yellow arrow = surface normal");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Raycasting: Basic Shapes".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsHud>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(2.0)
                .print_every(1.0)
                .display(|_m, _d| "static scene — no physics stepping".into()),
        )
        .add_systems(Startup, setup)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                draw_rays,
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
    let _ = data.forward(&model);

    println!("  Model: {} geoms, {} bodies\n", model.ngeom, model.nbody);

    // Materials — match MJCF rgba via geom-name overrides
    let mat_ground = materials.add(MetalPreset::BrushedMetal.material());

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("ground", mat_ground)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, -3.0, 3.0),
        16.0,
        std::f32::consts::FRAC_PI_4,
        0.2,
    );

    spawn_physics_hud(&mut commands);

    // Run validation once at startup (static scene)
    run_validation(&model, &data);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Gizmo Drawing ───────────────────────────────────────────────────────────

fn draw_rays(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut gizmos: Gizmos) {
    let results = cast_rays(&model, &data);
    let ray_color = Color::srgba(0.2, 0.8, 1.0, 0.4);
    let hit_color = Color::srgb(0.2, 1.0, 0.3);
    let normal_color = Color::srgb(1.0, 0.9, 0.2);
    let dot_radius = 0.06;
    let normal_len = 0.5;

    for (i, &(_, x, _)) in TARGETS.iter().enumerate() {
        let start = Vec3::new(x as f32, 0.0, RAY_Z as f32);

        match results.get(i).and_then(|(_, _, h)| h.as_ref()) {
            Some(hit) => {
                let hp = Vec3::new(hit.point.x as f32, hit.point.y as f32, hit.point.z as f32);
                let n = Vec3::new(
                    hit.normal.x as f32,
                    hit.normal.y as f32,
                    hit.normal.z as f32,
                );

                // Ray line (origin to hit)
                gizmos.line(start, hp, ray_color);
                // Hit point
                gizmos.sphere(Isometry3d::from_translation(hp), dot_radius, hit_color);
                // Normal arrow
                gizmos.arrow(hp, hp + n * normal_len, normal_color);
            }
            None => {
                // Miss — draw faint grey line
                let end = Vec3::new(x as f32, 0.0, -1.0);
                gizmos.line(start, end, Color::srgba(0.5, 0.5, 0.5, 0.3));
            }
        }
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Raycasting — Basic Shapes");
    hud.raw(String::new());

    let results = cast_rays(&model, &data);
    for (name, _expected, hit) in &results {
        match hit {
            Some(h) => {
                let n = h.normal;
                hud.raw(format!(
                    "{name:<10} dist={:.3}  normal=({:.2}, {:.2}, {:.2})",
                    h.distance, n.x, n.y, n.z,
                ));
            }
            None => {
                hud.raw(format!("{name:<10} MISS"));
            }
        }
    }
}

// ── Validation ──────────────────────────────────────────────────────────────

fn run_validation(model: &sim_core::Model, data: &sim_core::Data) {
    let results = cast_rays(model, data);
    let dir = -Vector3::z();

    // Check 1: All 6 rays hit
    let all_hit = results.iter().all(|(_, _, h)| h.is_some());

    // Check 2: Distances match expectations
    let max_dist_err = results
        .iter()
        .filter_map(|(_, exp, h)| h.as_ref().map(|hit| (hit.distance - exp).abs()))
        .fold(0.0_f64, f64::max);

    // Check 3: Hit points on surface (z-coordinate sanity)
    let max_z_err = results
        .iter()
        .filter_map(|(_, exp, h)| h.as_ref().map(|hit| (hit.point.z - (RAY_Z - exp)).abs()))
        .fold(0.0_f64, f64::max);

    // Check 4: Normals unit-length
    let max_norm_err = results
        .iter()
        .filter_map(|(_, _, h)| h.as_ref().map(|hit| (hit.normal.norm() - 1.0).abs()))
        .fold(0.0_f64, f64::max);

    // Check 5: Normals face ray origin (dot < 0)
    let worst_dot = results
        .iter()
        .filter_map(|(_, _, h)| h.as_ref().map(|hit| hit.normal.dot(&dir)))
        .fold(f64::NEG_INFINITY, f64::max);

    let checks = vec![
        Check {
            name: "All 6 rays hit",
            pass: all_hit,
            detail: format!(
                "{}/6 hits",
                results.iter().filter(|(_, _, h)| h.is_some()).count()
            ),
        },
        Check {
            name: "Distance accuracy",
            pass: max_dist_err < 1e-3,
            detail: format!("max error = {max_dist_err:.2e}"),
        },
        Check {
            name: "Hit point on surface",
            pass: max_z_err < 1e-3,
            detail: format!("max z-error = {max_z_err:.2e}"),
        },
        Check {
            name: "Normal unit-length",
            pass: max_norm_err < 1e-3,
            detail: format!("max ‖n‖-1 = {max_norm_err:.2e}"),
        },
        Check {
            name: "Normal faces ray",
            pass: worst_dot < 0.0,
            detail: format!("worst dot(n, dir) = {worst_dot:.6}"),
        },
    ];

    let all_pass = print_report("Raycasting — Basic Shapes (5 checks)", &checks);
    if !all_pass {
        std::process::exit(1);
    }
}
