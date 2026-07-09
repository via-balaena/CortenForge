//! Face-painting ladder — **B5.1: brush-paint a region** (`paint-faces`).
//!
//! Loads a real vertebra (`$CF_L4_STL`) and lets you **paint / erase a region
//! of faces** with a round brush that is drawn on the surface. This is the
//! region-selection front-end for the disc loft: a human paints the two
//! endplate patches (the automatic rule can't select real endplates cleanly),
//! which then loft into the disc.
//!
//! Picking contract: `cf_bevy_common::triangle_mesh_flat_shaded` emits three
//! vertices per face **in face order**, so a mesh ray hit's `triangle_index` is
//! the source `IndexedMesh` face id; the brush paints every face whose centroid
//! is within the brush radius of the hit face.
//!
//! Controls: **left-drag** orbits; **scroll** zooms; **right-drag** pans.
//! **Shift + left-drag** paints (or erases) a brush stroke; **`E`** toggles
//! paint / erase; **`[` / `]`** shrink / grow the brush; **`C`** clears all.
//! The brush ring on the surface shows its size (red = paint, cyan = erase).
//!
//! **`N`** toggles a normal-similarity filter (on by default) so the brush only
//! paints faces facing the same way as the one under the cursor — painting a
//! flat endplate won't spill onto the steep lateral walls. **`-` / `=`** widen /
//! tighten the tolerance.
//!
//! The STL is BodyParts3D (CC BY-SA, **not committed**). Point `$CF_L4_STL` at
//! the L4 STL (FMA13075).
//!
//! Run: `CF_L4_STL=… cargo run --release -p sim-bevy-soft --example paint-faces`

#![allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
#![allow(clippy::expect_used)] // an example may expect on its own required asset/env setup.
#![allow(clippy::too_many_arguments)] // Bevy systems legitimately take many resource params.

use std::collections::HashSet;

use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::mesh::VertexAttributeValues;
use bevy::picking::mesh_picking::ray_cast::{MeshRayCast, MeshRayCastSettings};
use bevy::prelude::*;
use cf_bevy_common::mesh::triangle_mesh_flat_shaded;
use cf_bevy_common::prelude::{OrbitCamera, update_orbit_camera};
use cf_fsu_geometry::load_from_env;
use cf_viewer::{UpAxis, setup_camera_and_lighting};
use mesh_types::Aabb;

/// Base (unpainted) face colour — pale bone ivory.
const BASE: [f32; 4] = [0.80, 0.78, 0.72, 1.0];
/// Highlight colour for a painted face.
const HIGHLIGHT: [f32; 4] = [0.90, 0.30, 0.20, 1.0];
/// Initial brush radius, in millimetres (native mesh units).
const BRUSH_INIT: f64 = 4.0;
/// Brush radius bounds (mm).
const BRUSH_MIN: f64 = 0.5;
const BRUSH_MAX: f64 = 30.0;

/// The paintable mesh plus per-face centroids and unit normals (native mm) for
/// the brush-radius and normal-similarity queries.
#[derive(Resource)]
struct PaintTarget {
    mesh: Handle<Mesh>,
    centroids: Vec<[f64; 3]>,
    normals: Vec<[f64; 3]>,
}

/// Restrict the brush to faces whose normal is within `max_angle_deg` of the
/// face under the cursor — so painting a flat endplate doesn't spill onto the
/// steep lateral walls. Toggle with `N`, widen / tighten with `-` / `=`.
#[derive(Resource)]
struct NormalFilter {
    enabled: bool,
    max_angle_deg: f64,
}

/// The set of currently-painted face ids (tracked so `C` can clear them).
#[derive(Resource, Default)]
struct Painted(HashSet<usize>);

/// The surface point under the cursor this frame (world space) and the face
/// there — set by [`hover_ray`], read by the brush ring and the paint stroke.
#[derive(Resource, Default)]
struct Hover {
    point: Option<Vec3>,
    normal: Vec3,
    face: Option<usize>,
}

/// The current brush radius (mm).
#[derive(Resource)]
struct BrushRadius(f64);

/// Whether the brush paints or erases.
#[derive(Resource, Clone, Copy, PartialEq, Eq)]
enum BrushMode {
    Paint,
    Erase,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .init_resource::<Painted>()
        .init_resource::<Hover>()
        .insert_resource(BrushRadius(BRUSH_INIT))
        .insert_resource(BrushMode::Paint)
        .insert_resource(NormalFilter {
            enabled: true,
            max_angle_deg: 35.0,
        })
        .add_systems(Startup, setup)
        .add_systems(Update, (camera_input, update_orbit_camera).chain())
        .add_systems(Update, (hover_ray, apply_brush, draw_brush).chain())
        .add_systems(
            Update,
            (adjust_brush, toggle_mode, toggle_filter, clear_selection),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let l4 = load_from_env("CF_L4_STL").expect("set $CF_L4_STL (vertebra, e.g. FMA13075)");

    // Per-face centroids and unit normals in native coordinates for the
    // brush-radius and normal-similarity queries.
    let mut centroids: Vec<[f64; 3]> = Vec::with_capacity(l4.faces.len());
    let mut normals: Vec<[f64; 3]> = Vec::with_capacity(l4.faces.len());
    for &[a, b, c] in &l4.faces {
        let (a, b, c) = (
            l4.vertices[a as usize],
            l4.vertices[b as usize],
            l4.vertices[c as usize],
        );
        centroids.push([
            (a.x + b.x + c.x) / 3.0,
            (a.y + b.y + c.y) / 3.0,
            (a.z + b.z + c.z) / 3.0,
        ]);
        let n = (b - a).cross(&(c - a));
        let len = n.norm();
        normals.push(if len > 1e-12 {
            [n.x / len, n.y / len, n.z / len]
        } else {
            [0.0, 0.0, 1.0]
        });
    }

    // Per-source-vertex colours seed the ATTRIBUTE_COLOR attribute; the builder
    // emits three per face, which the brush then overwrites per face.
    let seed = vec![BASE; l4.vertices.len()];
    let mesh = triangle_mesh_flat_shaded(&l4, Some(&seed), UpAxis::PlusZ);
    let handle = meshes.add(mesh);

    commands.spawn((
        Mesh3d(handle.clone()),
        MeshMaterial3d(materials.add(StandardMaterial {
            // White base so the per-vertex face colours show through unmodulated.
            base_color: Color::WHITE,
            perceptual_roughness: 0.85,
            ..default()
        })),
        Transform::default(),
    ));
    commands.insert_resource(PaintTarget {
        mesh: handle,
        centroids,
        normals,
    });

    setup_camera_and_lighting(
        &mut commands,
        &Aabb::from_points(l4.vertices.iter()),
        UpAxis::PlusZ,
    );
}

/// Usual controls: orbit on left-drag (unless Shift is held for painting), pan
/// on right-drag, zoom on scroll.
fn camera_input(
    mouse: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    mut cameras: Query<&mut OrbitCamera>,
) {
    for mut camera in &mut cameras {
        if mouse.pressed(MouseButton::Left) && !painting(&keys) {
            camera.orbit(motion.delta);
        }
        if mouse.pressed(MouseButton::Right) {
            camera.pan(motion.delta);
        }
        if scroll.delta.y.abs() > 1e-3 {
            camera.zoom(scroll.delta.y);
        }
    }
}

/// Ray-cast the cursor into the mesh every frame, recording the surface hit for
/// the brush ring and the paint stroke.
fn hover_ray(
    windows: Query<&Window>,
    cameras: Query<(&Camera, &GlobalTransform)>,
    mut ray_cast: MeshRayCast,
    mut hover: ResMut<Hover>,
) {
    hover.point = None;
    hover.face = None;
    let Ok(window) = windows.single() else {
        return;
    };
    let Some(cursor) = window.cursor_position() else {
        return;
    };
    let Ok((camera, camera_transform)) = cameras.single() else {
        return;
    };
    let Ok(ray) = camera.viewport_to_world(camera_transform, cursor) else {
        return;
    };
    if let Some((_entity, hit)) = ray_cast
        .cast_ray(ray, &MeshRayCastSettings::default())
        .first()
    {
        hover.point = Some(hit.point);
        hover.normal = hit.normal;
        hover.face = hit.triangle_index;
    }
}

/// While Shift + left is held, paint (or erase) every face within the brush
/// radius of the hovered face.
fn apply_brush(
    mouse: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    mode: Res<BrushMode>,
    radius: Res<BrushRadius>,
    filter: Res<NormalFilter>,
    hover: Res<Hover>,
    target: Res<PaintTarget>,
    mut painted: ResMut<Painted>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    if !painting(&keys) || !mouse.pressed(MouseButton::Left) {
        return;
    }
    let Some(face) = hover.face else {
        return;
    };
    let (Some(&centre), Some(&reference)) = (target.centroids.get(face), target.normals.get(face))
    else {
        return;
    };
    let Some(mesh) = meshes.get_mut(&target.mesh) else {
        return;
    };
    let Some(VertexAttributeValues::Float32x4(colours)) = mesh.attribute_mut(Mesh::ATTRIBUTE_COLOR)
    else {
        return;
    };

    let r2 = radius.0 * radius.0;
    let cos_min = filter.max_angle_deg.to_radians().cos();
    for (f, (c, n)) in target.centroids.iter().zip(&target.normals).enumerate() {
        let d = [c[0] - centre[0], c[1] - centre[1], c[2] - centre[2]];
        if d[0].mul_add(d[0], d[1].mul_add(d[1], d[2] * d[2])) > r2 {
            continue;
        }
        if filter.enabled {
            let dot = n[0].mul_add(
                reference[0],
                n[1].mul_add(reference[1], n[2] * reference[2]),
            );
            if dot < cos_min {
                continue; // normal too different from the face under the cursor
            }
        }
        match *mode {
            BrushMode::Paint => {
                if painted.0.insert(f) {
                    recolour(colours, f, HIGHLIGHT);
                }
            }
            BrushMode::Erase => {
                if painted.0.remove(&f) {
                    recolour(colours, f, BASE);
                }
            }
        }
    }
}

/// Draw the brush as a ring on the surface, coloured by mode.
fn draw_brush(
    mut gizmos: Gizmos,
    hover: Res<Hover>,
    radius: Res<BrushRadius>,
    mode: Res<BrushMode>,
) {
    let Some(point) = hover.point else {
        return;
    };
    let colour = match *mode {
        BrushMode::Paint => Color::srgb(0.95, 0.35, 0.25),
        BrushMode::Erase => Color::srgb(0.30, 0.80, 0.90),
    };
    let rotation = Quat::from_rotation_arc(Vec3::Z, hover.normal.normalize_or_zero());
    gizmos.circle(Isometry3d::new(point, rotation), radius.0 as f32, colour);
}

/// Shrink / grow the brush with `[` / `]`.
fn adjust_brush(keys: Res<ButtonInput<KeyCode>>, mut radius: ResMut<BrushRadius>) {
    if keys.just_pressed(KeyCode::BracketLeft) {
        radius.0 = (radius.0 * 0.8).max(BRUSH_MIN);
    }
    if keys.just_pressed(KeyCode::BracketRight) {
        radius.0 = (radius.0 * 1.25).min(BRUSH_MAX);
    }
}

/// Toggle paint / erase with `E`.
fn toggle_mode(keys: Res<ButtonInput<KeyCode>>, mut mode: ResMut<BrushMode>) {
    if keys.just_pressed(KeyCode::KeyE) {
        *mode = match *mode {
            BrushMode::Paint => BrushMode::Erase,
            BrushMode::Erase => BrushMode::Paint,
        };
    }
}

/// Toggle the normal-similarity filter with `N`; widen / tighten it with
/// `-` / `=`.
fn toggle_filter(keys: Res<ButtonInput<KeyCode>>, mut filter: ResMut<NormalFilter>) {
    if keys.just_pressed(KeyCode::KeyN) {
        filter.enabled = !filter.enabled;
        println!(
            "normal filter: {}",
            if filter.enabled { "ON" } else { "OFF" }
        );
    }
    if keys.just_pressed(KeyCode::Minus) {
        filter.max_angle_deg = (filter.max_angle_deg + 5.0).min(90.0);
        println!("normal tolerance: {:.0}°", filter.max_angle_deg);
    }
    if keys.just_pressed(KeyCode::Equal) {
        filter.max_angle_deg = (filter.max_angle_deg - 5.0).max(5.0);
        println!("normal tolerance: {:.0}°", filter.max_angle_deg);
    }
}

/// Clear the whole selection with `C`.
fn clear_selection(
    keys: Res<ButtonInput<KeyCode>>,
    target: Res<PaintTarget>,
    mut painted: ResMut<Painted>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    if !keys.just_pressed(KeyCode::KeyC) || painted.0.is_empty() {
        return;
    }
    let Some(mesh) = meshes.get_mut(&target.mesh) else {
        return;
    };
    if let Some(VertexAttributeValues::Float32x4(colours)) =
        mesh.attribute_mut(Mesh::ATTRIBUTE_COLOR)
    {
        for &f in &painted.0 {
            recolour(colours, f, BASE);
        }
    }
    painted.0.clear();
}

/// Whether a Shift key (the paint modifier) is held.
fn painting(keys: &ButtonInput<KeyCode>) -> bool {
    keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight)
}

/// Set the three emitted vertices of face `f` (`3f, 3f+1, 3f+2`) to `colour`.
fn recolour(colours: &mut [[f32; 4]], f: usize, colour: [f32; 4]) {
    let base = f * 3;
    for k in 0..3 {
        if let Some(slot) = colours.get_mut(base + k) {
            *slot = colour;
        }
    }
}
