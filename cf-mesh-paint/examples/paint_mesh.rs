//! `paint-mesh` — the generic [`cf_mesh_paint`] showcase.
//!
//! Paints face regions on two procedurally generated UV spheres (asset-free —
//! no external meshes needed), reading the painted-face counts back into an
//! egui status panel. This is the reference integration a consumer app
//! (CortenForge Studio) copies: add [`MeshPaintPlugin`], register each mesh as a
//! [`PaintBody`] built with [`paint_render_mesh`], wire a paint-aware camera,
//! and render your own HUD from the public paint-state resources.
//!
//! Controls: **Shift + left-drag** paints / erases the active sphere; **Tab**
//! switches sphere; **E** paint/erase; **N** normal filter; **`-` / `=`**
//! tolerance; **`[` / `]`** brush size; **Ctrl + Z** undo; **C** clear; the
//! panel mirrors the live state. Left-drag orbits, right-drag pans, scroll
//! zooms. **Esc** quits.
//!
//! Run: `cargo run -p cf-mesh-paint --example paint-mesh`

#![allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.

use std::f64::consts::PI;

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use cf_bevy_common::prelude::update_orbit_camera;
use cf_geometry::IndexedMesh;
use cf_mesh_paint::prelude::*;
use cf_viewer::{UpAxis, setup_camera_and_lighting};
use nalgebra::Point3;

use mesh_types::Aabb;

/// Sphere radius (native units).
const RADIUS: f64 = 1.0;
/// UV tessellation (latitude rings × longitude sectors) — fine enough that the
/// brush footprint reads smoothly.
const RINGS: usize = 24;
const SECTORS: usize = 48;
/// Centre-to-centre separation of the two spheres along x.
const GAP: f32 = 2.6;

fn main() {
    let config = MeshPaintConfig {
        // The spheres are ~unit radius, so a smaller brush than the anatomical
        // (millimetre) default reads better here.
        brush_init: 0.35,
        brush_min: 0.05,
        brush_max: 1.5,
        ..MeshPaintConfig::default()
    };
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin::default())
        .add_plugins(MeshPaintPlugin::new(config))
        .insert_resource(PaintDefaults { config })
        .add_systems(Startup, setup)
        // The paint-aware orbit replaces cf-bevy-common's default orbit input;
        // egui gets first refusal on the pointer so drags over the panel don't
        // orbit the camera or paint the sphere behind it. The arbiter runs
        // before the plugin's `hover_ray` so the paint gate applies same-frame.
        .add_systems(
            Update,
            (
                arbitrate_pointer_over_egui
                    .before(orbit_when_not_painting)
                    .before(cf_mesh_paint::brush::hover_ray),
                orbit_when_not_painting,
                update_orbit_camera.after(orbit_when_not_painting),
                exit_on_esc,
            ),
        )
        .add_systems(EguiPrimaryContextPass, status_panel)
        .run();
}

/// The plugin config, kept so `setup` seeds each render mesh with the same base
/// colour the brush restores on erase.
#[derive(Resource, Clone, Copy)]
struct PaintDefaults {
    config: MeshPaintConfig,
}

fn setup(
    defaults: Res<PaintDefaults>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let base = defaults.config.base_color;
    let spheres = [
        (Vec3::new(-GAP / 2.0, 0.0, 0.0), "sphere A"),
        (Vec3::new(GAP / 2.0, 0.0, 0.0), "sphere B"),
    ];

    let mut entities = Vec::new();
    let mut all_points: Vec<Point3<f64>> = Vec::new();
    for (center, name) in spheres {
        let source = uv_sphere(center, RADIUS, RINGS, SECTORS);
        all_points.extend(source.vertices.iter().copied());
        let handle = meshes.add(paint_render_mesh(&source, UpAxis::PlusZ, base));
        let material = materials.add(StandardMaterial {
            base_color: Color::WHITE, // per-vertex face colours show through
            perceptual_roughness: 0.85,
            double_sided: true,
            cull_mode: None,
            ..default()
        });
        let entity = commands
            .spawn((
                Mesh3d(handle.clone()),
                MeshMaterial3d(material),
                Transform::default(),
                PaintBody::new(name, source, handle),
            ))
            .id();
        entities.push(entity);
    }
    commands.insert_resource(PaintTargets::new(entities));

    let bounds = Aabb::from_points(all_points.iter());
    setup_camera_and_lighting(&mut commands, &bounds, UpAxis::PlusZ);
}

/// The egui status panel — the consumer-side HUD, reading the plugin's public
/// paint-state resources (the plugin itself ships no UI).
fn status_panel(
    mut contexts: EguiContexts,
    brush: Res<Brush>,
    mode: Res<BrushMode>,
    filter: Res<NormalFilter>,
    targets: Res<PaintTargets>,
    q_bodies: Query<&PaintBody>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    egui::SidePanel::right("paint-status")
        .default_width(220.0)
        .show(ctx, |ui| {
            ui.heading("paint-mesh");
            ui.separator();
            let active_name = targets
                .active_entity()
                .and_then(|e| q_bodies.get(e).ok())
                .map_or("?", PaintBody::name);
            let mode = match *mode {
                BrushMode::Paint => "PAINT",
                BrushMode::Erase => "ERASE",
            };
            let filter = if filter.enabled {
                format!("on ({:.0}°)", filter.max_angle_deg)
            } else {
                "off".to_string()
            };
            ui.label(format!("active   {active_name}"));
            ui.label(format!("mode     {mode}"));
            ui.label(format!("brush    {:.2}", brush.radius));
            ui.label(format!("filter   {filter}"));
            ui.separator();
            ui.label("painted faces");
            for &e in targets.entities() {
                if let Ok(body) = q_bodies.get(e) {
                    ui.label(format!("  {:<10}{}", body.name(), body.painted_count()));
                }
            }
            ui.separator();
            ui.label("Shift+drag paint · Tab switch");
            ui.label("E mode · N filter · [ ] size");
            ui.label("Ctrl+Z undo · C clear · Esc quit");
        });
    Ok(())
}

/// Arbitrate the pointer while it is over the egui panel: suppress the brush
/// (via [`PaintingBlocked`], so a `Shift`-click on the panel doesn't paint the
/// sphere behind it) and zero the accumulated camera motion / scroll (so a drag
/// on the panel doesn't orbit).
fn arbitrate_pointer_over_egui(
    mut contexts: EguiContexts,
    mut blocked: ResMut<PaintingBlocked>,
    mut motion: ResMut<bevy::input::mouse::AccumulatedMouseMotion>,
    mut scroll: ResMut<bevy::input::mouse::AccumulatedMouseScroll>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    let over_panel = ctx.wants_pointer_input() || ctx.is_pointer_over_area();
    blocked.0 = over_panel;
    if over_panel {
        motion.delta = Vec2::ZERO;
        scroll.delta = Vec2::ZERO;
    }
    Ok(())
}

/// Esc quits the window (mirrors the workspace viewers).
fn exit_on_esc(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}

/// A watertight UV-sphere `IndexedMesh` centred at `center` (poles as single
/// verts, quad strips between rings), wound outward-CCW.
fn uv_sphere(center: Vec3, radius: f64, rings: usize, sectors: usize) -> IndexedMesh {
    let mut verts: Vec<Point3<f64>> = Vec::new();
    verts.push(Point3::new(
        f64::from(center.x),
        f64::from(center.y),
        f64::from(center.z) + radius,
    )); // north pole = index 0
    for i in 1..rings {
        let theta = PI * (i as f64) / (rings as f64); // 0 (north) → π (south)
        let (z, r) = (radius * theta.cos(), radius * theta.sin());
        for j in 0..sectors {
            let phi = 2.0 * PI * (j as f64) / (sectors as f64);
            verts.push(Point3::new(
                f64::from(center.x) + r * phi.cos(),
                f64::from(center.y) + r * phi.sin(),
                f64::from(center.z) + z,
            ));
        }
    }
    verts.push(Point3::new(
        f64::from(center.x),
        f64::from(center.y),
        f64::from(center.z) - radius,
    )); // south pole
    let south = (verts.len() - 1) as u32;

    let idx = |ring: usize, j: usize| -> u32 { (1 + (ring - 1) * sectors + (j % sectors)) as u32 };
    let mut faces: Vec<[u32; 3]> = Vec::new();
    for j in 0..sectors {
        faces.push([0, idx(1, j), idx(1, j + 1)]); // top fan
    }
    for i in 1..(rings - 1) {
        for j in 0..sectors {
            let (a, b, c, d) = (idx(i, j), idx(i, j + 1), idx(i + 1, j), idx(i + 1, j + 1));
            faces.push([a, c, b]);
            faces.push([b, c, d]);
        }
    }
    for j in 0..sectors {
        faces.push([south, idx(rings - 1, j + 1), idx(rings - 1, j)]); // bottom fan
    }

    let mesh = IndexedMesh::from_parts(verts, faces);
    if mesh.signed_volume() >= 0.0 {
        mesh
    } else {
        let faces = mesh.faces.iter().map(|&[a, b, c]| [a, c, b]).collect();
        IndexedMesh::from_parts(mesh.vertices, faces)
    }
}
