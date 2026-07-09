//! Disc-render ladder — **step 5.0: `real-bone-render`** (first real-anatomy rung).
//!
//! The one concept: **load and render a real vertebra — nothing else.** No bond, no contact,
//! no solver. Steps 1–4 proved the disc's physics (bond + curved contact) on a box; before any
//! of that touches real anatomy, this rung de-risks the *asset pipeline* in isolation: read a
//! real BodyParts3D vertebra STL, weld-repair it, and render it in the same frame convention
//! (`UpAxis::PlusZ`, physics Z-up → Bevy Y-up) the deforming disc uses. Real anatomy is where
//! the earlier render work broke, and loading/orienting the mesh is pure render (no physics),
//! so it is the cheapest thing to verify first: does the vertebra load, sit right-side up, and
//! look like a lumbar vertebra (body + posterior arch), at a sane scale?
//!
//! It renders through the SAME `build_soft_mesh` path the soft disc uses (an `IndexedMesh`'s
//! `vertices` + `faces` feed straight in — `faces` is already the outward-CCW `[VertexId; 3]`
//! triangulation), so orientation / winding conventions are shared: whatever looks right here
//! will look right when the disc joins it in step 5.3.
//!
//! The STL is BodyParts3D (CC BY-SA, **not committed**). Point `$CF_L5_STL` at an L5 vertebra
//! surface STL (FMA13076); the same file the parked `cf-spine-viewer` consumes. Orbit with LMB.
//!
//! Run: `CF_L5_STL=/path/to/L5.stl cargo run -p sim-bevy-soft --example real-bone-render`

#![allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
#![allow(clippy::expect_used)] // an example may expect on its own required asset/env setup.

use bevy::prelude::*;
use cf_bevy_common::prelude::OrbitCameraPlugin;
use cf_fsu_geometry::load_from_env;
use cf_viewer::{UpAxis, setup_camera_and_lighting};
use mesh_types::Aabb;
use nalgebra::Point3;
use sim_bevy_soft::prelude::build_soft_mesh;
use sim_soft::{Vec3, VertexId};

/// The loaded vertebra surface (native STL coordinates, mm), split into the two slices
/// `build_soft_mesh` consumes.
#[derive(Resource)]
struct BoneScene {
    verts: Vec<Vec3>,
    faces: Vec<[VertexId; 3]>,
}

/// Load the real L5 vertebra STL named by `$CF_L5_STL` (weld-repaired, native coordinates) and
/// reshape it into the `build_soft_mesh` inputs. Prints a few sanity numbers — vertex / face
/// counts and the signed volume (positive ⇒ outward-CCW, i.e. not inside-out).
fn load_bone() -> BoneScene {
    let mesh = load_from_env("CF_L5_STL")
        .expect("set $CF_L5_STL to a real L5 vertebra STL (BodyParts3D FMA13076)");
    let verts: Vec<Vec3> = mesh
        .vertices
        .iter()
        .map(|p| Vec3::new(p.x, p.y, p.z))
        .collect();
    let faces: Vec<[VertexId; 3]> = mesh.faces.clone();

    let aabb = Aabb::from_points(mesh.vertices.iter());
    let ext = aabb.max - aabb.min;
    println!(
        "real-bone-render: L5 loaded — {} verts, {} faces, signed_volume {:+.3e} (>0 = \
         outward-CCW), extent ({:.1}, {:.1}, {:.1}) native units",
        verts.len(),
        faces.len(),
        mesh.signed_volume(),
        ext.x,
        ext.y,
        ext.z,
    );
    BoneScene { verts, faces }
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(load_bone())
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    scene: Res<BoneScene>,
) {
    // Ivory bone. `double_sided` + `cull_mode: None` so a raw scan surface reads correctly
    // regardless of any residual winding inconsistency (mirrors the cf-spine-viewer bone
    // material); this rung is verifying geometry, not shading.
    commands.spawn((
        Mesh3d(meshes.add(build_soft_mesh(&scene.verts, &scene.faces, UpAxis::PlusZ))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.87, 0.84, 0.78),
            perceptual_roughness: 0.85,
            double_sided: true,
            cull_mode: None,
            ..default()
        })),
        Transform::default(),
    ));

    let pts: Vec<Point3<f64>> = scene.verts.iter().map(|v| Point3::from(*v)).collect();
    setup_camera_and_lighting(&mut commands, &Aabb::from_points(pts.iter()), UpAxis::PlusZ);
}
