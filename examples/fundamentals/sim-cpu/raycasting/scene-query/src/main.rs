//! Scene Query — LIDAR-style Ray Fan
//!
//! A fan of 36 rays (10° apart, 360° sweep in the physics XZ plane) fires
//! from a central eye point. `raycast_scene` returns the nearest hit for
//! each ray. Hits are cyan lines with green dots; misses are faint grey.
//!
//! Validates:
//! 1. Ray at 0° (toward sphere) hits at dist ≈ 2.5
//! 2. geom_id matches expected geom for known angles
//! 3. Rays into open space return None
//! 4. Body-exclude filter removes sphere from results
//!
//! Run: `cargo run -p example-raycasting-scene-query --release`

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
use sim_bevy::model_data::{PhysicsData, PhysicsModel, spawn_model_geoms, sync_geom_transforms};
use sim_core::validation::{Check, print_report};
use sim_core::{SceneRayHit, raycast_scene};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="scene-query">
  <option gravity="0 0 0" timestep="0.002"/>
  <worldbody>
    <body name="sphere_body" pos="3 0 0">
      <geom name="sphere" type="sphere" size="0.5"
            rgba="0.25 0.55 0.85 1"/>
    </body>
    <geom name="box"     type="box"     pos="0 0 3"  size="0.8 0.8 0.8"
          rgba="0.85 0.35 0.25 1"/>
    <geom name="capsule" type="capsule" pos="-2 0 1" size="0.3 0.7"
          rgba="0.30 0.75 0.40 1"/>
    <geom name="ground"  type="plane"   pos="0 0 -1" size="5 5 0.01"
          rgba="0.25 0.25 0.28 1"/>
  </worldbody>
</mujoco>
"#;

// ── Ray fan parameters ──────────────────────────────────────────────────────

const NUM_RAYS: usize = 36;
const MAX_DISTANCE: f64 = 10.0;

/// Eye position in physics space (origin).
fn eye_origin() -> Point3<f64> {
    Point3::origin()
}

/// Ray direction for a given index (0..NUM_RAYS) in the physics XZ plane.
fn ray_direction(index: usize) -> UnitVector3<f64> {
    let angle = (index as f64) * std::f64::consts::TAU / (NUM_RAYS as f64);
    UnitVector3::new_normalize(Vector3::new(angle.cos(), 0.0, angle.sin()))
}

/// Cast the full fan and return results.
fn cast_fan(model: &sim_core::Model, data: &sim_core::Data) -> Vec<Option<SceneRayHit>> {
    let origin = eye_origin();
    (0..NUM_RAYS)
        .map(|i| {
            raycast_scene(
                model,
                data,
                origin,
                ray_direction(i),
                MAX_DISTANCE,
                None,
                None,
            )
        })
        .collect()
}

fn configure_gizmos(mut config_store: ResMut<GizmoConfigStore>) {
    let (config, _) = config_store.config_mut::<DefaultGizmoConfigGroup>();
    config.line.width = 3.0;
    config.depth_bias = -0.01;
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Raycasting — Scene Query (LIDAR Fan) ===");
    println!("  36 rays sweep 360° from a central eye point");
    println!("  Cyan = hit | Grey = miss | Green dot = hit point");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Raycasting: Scene Query".into(),
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
                draw_fan,
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

    let mat_ground = materials.add(StandardMaterial {
        base_color: Color::srgba(0.15, 0.15, 0.18, 0.4),
        alpha_mode: AlphaMode::Blend,
        perceptual_roughness: 1.0,
        metallic: 0.0,
        ..default()
    });

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("ground", mat_ground)],
    );

    // Camera: looking at the eye from above-right, seeing the full fan
    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 1.0, 0.0), // eye is at origin, look slightly above
        14.0,
        std::f32::consts::FRAC_PI_4 + 1.0,
        0.5,
    );

    spawn_physics_hud(&mut commands);

    run_validation(&model, &data);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Gizmo Drawing ───────────────────────────────────────────────────────────

fn draw_fan(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut gizmos: Gizmos) {
    let results = cast_fan(&model, &data);
    let eye = vec3_from_point(&eye_origin());
    let hit_line_color = Color::srgba(0.2, 0.8, 1.0, 0.9);
    let miss_line_color = Color::srgba(0.9, 0.3, 0.3, 0.7);
    let hit_dot_color = Color::srgb(0.1, 1.0, 0.2);
    let dot_radius = 0.08;

    // Eye marker
    gizmos.sphere(
        Isometry3d::from_translation(eye),
        0.1,
        Color::srgb(1.0, 1.0, 1.0),
    );

    for (i, result) in results.iter().enumerate() {
        let dir = ray_direction(i).into_inner();
        match result {
            Some(hit) => {
                let hp = vec3_from_point(&hit.hit.point);
                gizmos.line(eye, hp, hit_line_color);
                gizmos.sphere(Isometry3d::from_translation(hp), dot_radius, hit_dot_color);
            }
            None => {
                let end = vec3_from_vector(&(dir * MAX_DISTANCE));
                gizmos.line(eye, eye + end, miss_line_color);
            }
        }
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Scene Query — LIDAR Fan");
    hud.raw(String::new());

    let results = cast_fan(&model, &data);
    let hits = results.iter().filter(|r| r.is_some()).count();
    let misses = NUM_RAYS - hits;
    hud.raw(format!(
        "Rays: {NUM_RAYS}   Hits: {hits}   Misses: {misses}"
    ));
    hud.raw(String::new());

    // Nearest and farthest hits
    let mut nearest: Option<(usize, &SceneRayHit)> = None;
    let mut farthest: Option<(usize, &SceneRayHit)> = None;

    for (i, result) in results.iter().enumerate() {
        if let Some(hit) = result {
            let d = hit.hit.distance;
            if nearest.is_none() || d < nearest.map_or(f64::MAX, |(_, h)| h.hit.distance) {
                nearest = Some((i, hit));
            }
            if farthest.is_none() || d > farthest.map_or(0.0, |(_, h)| h.hit.distance) {
                farthest = Some((i, hit));
            }
        }
    }

    if let Some((i, hit)) = nearest {
        let angle = i * 360 / NUM_RAYS;
        let name = geom_name(&model, hit.geom_id);
        hud.raw(format!(
            "Nearest:  {name} at {:.2} m (angle {angle}°)",
            hit.hit.distance
        ));
    }
    if let Some((i, hit)) = farthest {
        let angle = i * 360 / NUM_RAYS;
        let name = geom_name(&model, hit.geom_id);
        hud.raw(format!(
            "Farthest: {name} at {:.2} m (angle {angle}°)",
            hit.hit.distance
        ));
    }
}

fn geom_name(model: &sim_core::Model, geom_id: usize) -> String {
    model
        .geom_name
        .get(geom_id)
        .and_then(|n| n.as_ref())
        .map_or_else(|| format!("geom_{geom_id}"), |s| format!("\"{s}\""))
}

// ── Validation ──────────────────────────────────────────────────────────────

fn run_validation(model: &sim_core::Model, data: &sim_core::Data) {
    let results = cast_fan(model, data);

    // Check 1: Ray at 0° (toward sphere at x=3) → dist ≈ 2.5
    let check1 = match &results[0] {
        Some(hit) => {
            let err = (hit.hit.distance - 2.5).abs();
            Check {
                name: "Ray 0° hits sphere",
                pass: err < 0.1,
                detail: format!(
                    "dist = {:.3}, expected ≈ 2.5, err = {err:.3}",
                    hit.hit.distance
                ),
            }
        }
        None => Check {
            name: "Ray 0° hits sphere",
            pass: false,
            detail: "got None".into(),
        },
    };

    // Check 2: geom_id at 0° is the sphere (geom 0, first geom in sphere_body)
    let check2 = match &results[0] {
        Some(hit) => {
            let name = geom_name(model, hit.geom_id);
            Check {
                name: "geom_id matches sphere",
                pass: name.contains("sphere"),
                detail: format!("geom_id = {}, name = {name}", hit.geom_id),
            }
        }
        None => Check {
            name: "geom_id matches sphere",
            pass: false,
            detail: "no hit".into(),
        },
    };

    // Check 3: At least one ray misses (open space)
    let miss_count = results.iter().filter(|r| r.is_none()).count();
    let check3 = Check {
        name: "Some rays miss (open space)",
        pass: miss_count > 0,
        detail: format!("{miss_count}/{NUM_RAYS} rays missed"),
    };

    // Check 4: Body-exclude on sphere body → ray 0° misses or hits farther
    let origin = eye_origin();
    let dir0 = ray_direction(0);
    // Find sphere's body id
    let sphere_body = model
        .geom_name
        .iter()
        .position(|n| n.as_deref() == Some("sphere"))
        .map(|gid| model.geom_body[gid]);

    let check4 = if let Some(body_id) = sphere_body {
        let excluded = raycast_scene(model, data, origin, dir0, MAX_DISTANCE, Some(body_id), None);
        let original_dist = results[0].as_ref().map_or(f64::MAX, |h| h.hit.distance);
        let excluded_ok = match &excluded {
            None => true,                                    // miss — sphere was the only target
            Some(h) => h.hit.distance > original_dist + 0.1, // farther object
        };
        Check {
            name: "Body-exclude removes sphere",
            pass: excluded_ok,
            detail: match &excluded {
                None => "ray 0° now misses".into(),
                Some(h) => format!("dist = {:.3} (was {original_dist:.3})", h.hit.distance),
            },
        }
    } else {
        Check {
            name: "Body-exclude removes sphere",
            pass: false,
            detail: "sphere geom not found".into(),
        }
    };

    let checks = vec![check1, check2, check3, check4];
    let all_pass = print_report("Scene Query — LIDAR Fan (4 checks)", &checks);
    if !all_pass {
        std::process::exit(1);
    }
}
