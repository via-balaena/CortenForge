//! Heightfield — Ray Marching on Terrain
//!
//! A sinusoidal heightfield terrain with an 8×8 grid of downward rays.
//! Hit points form a dot cloud that traces the terrain surface. Ray marching
//! (not analytic intersection) finds the surface.
//!
//! Validates:
//! 1. All 64 rays hit (100% hit rate)
//! 2. Hit z-coordinates match the terrain function within 0.01
//! 3. Surface normals are consistent with the terrain gradient
//!
//! Run: `cargo run -p example-raycasting-heightfield --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::option_if_let_else,
    clippy::suboptimal_flops,
    clippy::let_underscore_must_use,
    clippy::single_match_else,
    clippy::similar_names
)]

use std::sync::Arc;

use bevy::gizmos::config::{DefaultGizmoConfigGroup, GizmoConfigStore};
use bevy::math::Isometry3d;
use bevy::prelude::*;
use nalgebra::{Point3, UnitVector3, Vector3};
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::convert::{vec3_from_point, vec3_from_vector};
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::model_data::{PhysicsData, PhysicsModel, sync_geom_transforms};
use sim_core::validation::{Check, print_report};
use sim_core::{HeightFieldData, raycast_scene};

// ── Terrain function ────────────────────────────────────────────────────────

fn terrain(x: f64, y: f64) -> f64 {
    0.3 * (std::f64::consts::TAU * x / 4.0).sin() * (std::f64::consts::TAU * y / 4.0).cos()
}

/// Analytical terrain normal via gradient: n = normalize(-dz/dx, -dz/dy, 1).
fn terrain_normal(x: f64, y: f64) -> Vector3<f64> {
    let tau = std::f64::consts::TAU;
    let dzdx = 0.3 * (tau / 4.0) * (tau * x / 4.0).cos() * (tau * y / 4.0).cos();
    let dzdy = 0.3 * (tau / 4.0) * (tau * x / 4.0).sin() * (-(tau * y / 4.0).sin());
    Vector3::new(-dzdx, -dzdy, 1.0).normalize()
}

// ── Heightfield parameters ──────────────────────────────────────────────────

const GRID_SIZE: usize = 64;
const CELL_SIZE: f64 = 8.0 / 63.0; // full extent 8m, 64 samples → 63 intervals
const RAY_GRID: usize = 8; // 8×8 grid of rays
const RAY_Z: f64 = 3.0;

// MJCF with a small placeholder hfield (replaced before spawning geoms)
const MJCF: &str = r#"
<mujoco model="heightfield">
  <option gravity="0 0 0" timestep="0.002"/>
  <asset>
    <hfield name="terrain" nrow="4" ncol="4" size="4 4 0.5 0"
            elevation="0 0 0 0  0 0 0 0  0 0 0 0  0 0 0 0"/>
  </asset>
  <worldbody>
    <geom name="terrain" type="hfield" hfield="terrain"/>
  </worldbody>
</mujoco>
"#;

// ── Ray casting ─────────────────────────────────────────────────────────────

struct RayResult {
    x: f64,
    y: f64,
    hit: Option<sim_core::RaycastHit>,
}

fn cast_terrain_rays(model: &sim_core::Model, data: &sim_core::Data) -> Vec<RayResult> {
    let dir = UnitVector3::new_normalize(-Vector3::z());
    let margin = 0.5;
    let extent = (GRID_SIZE as f64) * CELL_SIZE;
    let step = (extent - 2.0 * margin) / (RAY_GRID as f64 - 1.0);

    let mut results = Vec::with_capacity(RAY_GRID * RAY_GRID);
    for row in 0..RAY_GRID {
        for col in 0..RAY_GRID {
            let x = margin + col as f64 * step;
            let y = margin + row as f64 * step;
            let origin = Point3::new(x, y, RAY_Z);
            let hit = raycast_scene(model, data, origin, dir, 20.0, None, None);
            results.push(RayResult {
                x,
                y,
                hit: hit.map(|h| h.hit),
            });
        }
    }
    results
}

// ── Terrain mesh builder ────────────────────────────────────────────────────

/// Build a Bevy `Mesh` from the terrain function, in Y-up coordinates.
fn build_terrain_mesh(grid: usize, cell: f64) -> Mesh {
    let mut positions = Vec::with_capacity(grid * grid);
    let mut normals = Vec::with_capacity(grid * grid);
    let mut indices = Vec::with_capacity((grid - 1) * (grid - 1) * 6);

    // Vertices
    for row in 0..grid {
        for col in 0..grid {
            let x = col as f64 * cell;
            let y = row as f64 * cell;
            let z = terrain(x, y);
            // Physics (x, y, z) → Bevy (x, z, y)
            positions.push([x as f32, z as f32, y as f32]);
            let n = terrain_normal(x, y);
            normals.push([n.x as f32, n.z as f32, n.y as f32]);
        }
    }

    // Triangle indices (two triangles per cell)
    for row in 0..(grid - 1) {
        for col in 0..(grid - 1) {
            let tl = (row * grid + col) as u32;
            let tr = tl + 1;
            let bl = tl + grid as u32;
            let br = bl + 1;
            indices.extend_from_slice(&[tl, bl, tr, tr, bl, br]);
        }
    }

    Mesh::new(
        bevy::mesh::PrimitiveTopology::TriangleList,
        bevy::asset::RenderAssetUsages::default(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_indices(bevy::mesh::Indices::U32(indices))
}

fn configure_gizmos(mut config_store: ResMut<GizmoConfigStore>) {
    let (config, _) = config_store.config_mut::<DefaultGizmoConfigGroup>();
    config.line.width = 2.0;
    config.depth_bias = -0.01;
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Raycasting — Heightfield Terrain ===");
    println!("  8×8 grid of rays rain down on sinusoidal terrain");
    println!("  Green dots = hit points | Yellow arrows = normals");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Raycasting: Heightfield".into(),
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
        .add_systems(Startup, (setup, configure_gizmos))
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                draw_terrain_rays,
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
    let mut model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");

    // Replace placeholder with sinusoidal terrain
    let hfield = HeightFieldData::from_fn(GRID_SIZE, GRID_SIZE, CELL_SIZE, terrain);
    model.hfield_data[0] = Arc::new(hfield);

    let mut data = model.make_data();
    let _ = data.forward(&model);

    println!(
        "  Terrain: {}×{} samples, cell_size={CELL_SIZE:.4}m, extent={:.1}m\n",
        GRID_SIZE,
        GRID_SIZE,
        GRID_SIZE as f64 * CELL_SIZE
    );

    // Build terrain mesh manually (spawn_model_geoms skips hfield geoms)
    let terrain_mesh = build_terrain_mesh(GRID_SIZE, CELL_SIZE);
    let mat_terrain = materials.add(StandardMaterial {
        base_color: Color::srgba(0.35, 0.55, 0.35, 0.6),
        alpha_mode: AlphaMode::Blend,
        perceptual_roughness: 0.9,
        metallic: 0.0,
        double_sided: true,
        cull_mode: None,
        ..default()
    });
    commands.spawn((
        Mesh3d(meshes.add(terrain_mesh)),
        MeshMaterial3d(mat_terrain),
        Transform::IDENTITY,
    ));

    // Camera: above and to the side, looking at terrain center
    let center = (GRID_SIZE as f32 * CELL_SIZE as f32) / 2.0;
    spawn_example_camera(
        &mut commands,
        Vec3::new(center, 0.5, center),
        12.0,
        std::f32::consts::FRAC_PI_4,
        0.6,
    );

    spawn_physics_hud(&mut commands);

    run_validation(&model, &data);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Gizmo Drawing ───────────────────────────────────────────────────────────

fn draw_terrain_rays(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut gizmos: Gizmos) {
    let results = cast_terrain_rays(&model, &data);
    let ray_color = Color::srgba(0.2, 0.8, 1.0, 0.5);
    let hit_color = Color::srgb(0.1, 1.0, 0.2);
    let normal_color = Color::srgb(1.0, 0.85, 0.0);
    let dot_radius = 0.06;
    let normal_len = 0.3;

    for r in &results {
        let start = vec3_from_vector(&Vector3::new(r.x, r.y, RAY_Z));

        match &r.hit {
            Some(hit) => {
                let hp = vec3_from_point(&hit.point);
                let n = vec3_from_vector(&hit.normal);

                gizmos.line(start, hp, ray_color);
                gizmos.sphere(Isometry3d::from_translation(hp), dot_radius, hit_color);
                gizmos.arrow(hp, hp + n * normal_len, normal_color);
            }
            None => {
                let end = vec3_from_vector(&Vector3::new(r.x, r.y, -1.0));
                gizmos.line(start, end, Color::srgba(0.9, 0.3, 0.3, 0.7));
            }
        }
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Heightfield — Ray Marching");
    hud.raw(String::new());

    let results = cast_terrain_rays(&model, &data);
    let total = results.len();
    let hits = results.iter().filter(|r| r.hit.is_some()).count();
    hud.raw(format!("Rays cast: {total}"));
    hud.raw(format!(
        "Hits:      {hits} ({:.0}%)",
        100.0 * hits as f64 / total as f64
    ));

    let max_z_err = results
        .iter()
        .filter_map(|r| {
            r.hit
                .as_ref()
                .map(|h| (h.point.z - terrain(h.point.x, h.point.y)).abs())
        })
        .fold(0.0_f64, f64::max);
    hud.raw(format!("Max z-error vs terrain(x,y): {max_z_err:.4}"));
}

// ── Validation ──────────────────────────────────────────────────────────────

fn run_validation(model: &sim_core::Model, data: &sim_core::Data) {
    let results = cast_terrain_rays(model, data);
    let total = results.len();
    let hits: Vec<_> = results.iter().filter(|r| r.hit.is_some()).collect();

    // Check 1: All 64 rays hit
    let check1 = Check {
        name: "All rays hit (100%)",
        pass: hits.len() == total,
        detail: format!("{}/{total} hits", hits.len()),
    };

    // Check 2: Hit z matches terrain function
    let max_z_err = results
        .iter()
        .filter_map(|r| {
            r.hit
                .as_ref()
                .map(|h| (h.point.z - terrain(h.point.x, h.point.y)).abs())
        })
        .fold(0.0_f64, f64::max);
    let check2 = Check {
        name: "Hit z matches terrain(x,y)",
        pass: max_z_err < 0.01,
        detail: format!("max |dz| = {max_z_err:.4}"),
    };

    // Check 3: Normals consistent with terrain gradient
    let min_dot = results
        .iter()
        .filter_map(|r| {
            r.hit.as_ref().map(|h| {
                let analytical = terrain_normal(h.point.x, h.point.y);
                h.normal.normalize().dot(&analytical)
            })
        })
        .fold(1.0_f64, f64::min);
    let check3 = Check {
        name: "Normal matches gradient",
        pass: min_dot > 0.95,
        detail: format!("min dot(hit_n, analytical_n) = {min_dot:.4}"),
    };

    let checks = vec![check1, check2, check3];
    let all_pass = print_report("Heightfield — Ray Marching (3 checks)", &checks);
    if !all_pass {
        std::process::exit(1);
    }
}
