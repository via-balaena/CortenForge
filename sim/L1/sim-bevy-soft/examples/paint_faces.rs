//! Face-painting ladder — **B5.0: pick + highlight one face** (`paint-faces`).
//!
//! Loads a real vertebra (`$CF_L4_STL`) and lets you **click a face to
//! highlight it**. This is the first rung of the region-painting GUI that will
//! let a human pick the two endplate patches to loft into the disc (superseding
//! the automatic face rule, which can't cleanly select real endplates).
//!
//! It verifies the core picking contract: `cf_bevy_common::triangle_mesh_flat_shaded`
//! emits three unique vertices per face **in face order**, so a mesh ray hit's
//! `triangle_index` is exactly the source `IndexedMesh` face id — and recoloring
//! that face's three vertices highlights precisely the face under the cursor.
//!
//! Controls: **left-drag** orbits (usual); **scroll** zooms; **right-drag** pans.
//! **Shift + left-click** paints the face under the cursor. (Painting is on a
//! modifier so the usual left-drag orbit still works.)
//!
//! The STL is BodyParts3D (CC BY-SA, **not committed**). Point `$CF_L4_STL` at
//! the L4 STL (FMA13075).
//!
//! Run: `CF_L4_STL=… cargo run --release -p sim-bevy-soft --example paint-faces`

#![allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
#![allow(clippy::expect_used)] // an example may expect on its own required asset/env setup.

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
/// Highlight colour for a picked face.
const HIGHLIGHT: [f32; 4] = [0.90, 0.30, 0.20, 1.0];

/// The paintable mesh handle (its per-face vertex colours are mutated in place).
#[derive(Resource)]
struct PaintTarget {
    mesh: Handle<Mesh>,
}

/// A face picked this frame, awaiting recolour. Decouples the ray-cast (which
/// borrows `Assets<Mesh>` immutably) from the recolour (which needs it mutably)
/// — the two cannot share a system (Bevy B0002).
#[derive(Resource, Default)]
struct PickedFace(Option<usize>);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .init_resource::<PickedFace>()
        .add_systems(Startup, setup)
        .add_systems(Update, (camera_input, update_orbit_camera).chain())
        .add_systems(Update, (pick_face, apply_paint).chain())
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let l4 = load_from_env("CF_L4_STL").expect("set $CF_L4_STL (vertebra, e.g. FMA13075)");

    // Per-source-vertex colours seed the ATTRIBUTE_COLOR attribute; the builder
    // emits three per face, which `paint_face` then overwrites per face.
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
    commands.insert_resource(PaintTarget { mesh: handle });

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
    let painting = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
    for mut camera in &mut cameras {
        if mouse.pressed(MouseButton::Left) && !painting {
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

/// On Shift + left-click, ray-cast the cursor into the mesh and record the hit
/// face (Shift keeps painting from fighting the left-drag orbit).
fn pick_face(
    mouse: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    windows: Query<&Window>,
    cameras: Query<(&Camera, &GlobalTransform)>,
    mut ray_cast: MeshRayCast,
    mut picked: ResMut<PickedFace>,
) {
    let painting = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
    if !painting || !mouse.just_pressed(MouseButton::Left) {
        return;
    }
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

    let hits = ray_cast.cast_ray(ray, &MeshRayCastSettings::default());
    if let Some((_entity, hit)) = hits.first() {
        picked.0 = hit.triangle_index;
    }
}

/// Highlight the face picked this frame (if any) by recolouring its three
/// vertices. The flat-shaded mesh emits verts `3·face, 3·face+1, 3·face+2`.
fn apply_paint(
    mut picked: ResMut<PickedFace>,
    target: Res<PaintTarget>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let Some(face) = picked.0.take() else {
        return;
    };
    let Some(mesh) = meshes.get_mut(&target.mesh) else {
        return;
    };
    if let Some(VertexAttributeValues::Float32x4(colours)) =
        mesh.attribute_mut(Mesh::ATTRIBUTE_COLOR)
    {
        let base = face * 3;
        for k in 0..3 {
            if let Some(colour) = colours.get_mut(base + k) {
                *colour = HIGHLIGHT;
            }
        }
    }
}
