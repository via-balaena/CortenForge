//! Disc-render ladder — **the SDF-modelled disc: `modeled-disc`** (superseded checkpoint).
//!
//! ▶ SUPERSEDED by the painted-loft primitive (`paint-faces`): an automatic endplate rule
//! can't cleanly delimit the contact region — it leaves a posterior canal flap where the
//! footprint sits over the pedicle/canal — so the shipping disc is now a human-painted loft.
//! This example is kept as the checkpoint that PROVED a disc can be modelled from the endplate
//! surfaces (68% seated), not the live path.
//!
//! Renders the disc MODELLED from the vertebral endplates (`cf_fsu_geometry::
//! model_disc_between_endplates`) nestled between the real L4 and L5 — so you can SEE it seat.
//! Unlike the scanned disc (whose superior face floats ~2.5 mm below L4), this disc is the solid
//! carved to FILL the endplate gap: `disc = (footprint prism ∩ SI slab) − L4 − L5`. Its top face
//! IS L4's inferior endplate and its bottom face IS L5's superior endplate, by construction — the
//! headless test measures 68% of its surface seated on the real bone.
//!
//! Static scene (no physics) — orbit with LMB to inspect the seating. The two vertebrae render
//! **translucent** (`[` / `]` adjust opacity) so the disc is visible filling the gap between them.
//!
//! The STLs are BodyParts3D (CC BY-SA, **not committed**). Point `$CF_L4_STL` / `$CF_L5_STL` /
//! `$CF_DISC_STL` at the L4 / L5 / disc STLs (FMA13075 / 13076 / 16036 — the disc scan is used only
//! for the lateral footprint).
//!
//! Run: `CF_L4_STL=… CF_L5_STL=… CF_DISC_STL=… cargo run --release -p sim-bevy-soft --example modeled-disc`

#![allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
#![allow(clippy::expect_used)] // an example may expect on its own required asset/env setup.

use bevy::prelude::*;
use cf_bevy_common::prelude::OrbitCameraPlugin;
use cf_fsu_geometry::{DiscModelParams, load_from_env, model_disc_between_endplates};
use cf_geometry::IndexedMesh;
use cf_viewer::{UpAxis, setup_camera_and_lighting};
use mesh_types::Aabb;
use nalgebra::Point3;
use sim_bevy_soft::prelude::build_soft_mesh;
use sim_soft::{Vec3, VertexId};

/// Initial opacity of the (translucent) vertebrae — low enough to see the disc seated inside.
const BONE_ALPHA_INIT: f32 = 0.35;
/// Opacity step per `[` / `]` press.
const BONE_ALPHA_STEP: f32 = 0.05;

/// One renderable surface (native mm) split into the two slices `build_soft_mesh` consumes.
struct Surface {
    verts: Vec<Vec3>,
    faces: Vec<[VertexId; 3]>,
}

fn to_surface(mesh: &IndexedMesh) -> Surface {
    Surface {
        verts: mesh
            .vertices
            .iter()
            .map(|p| Vec3::new(p.x, p.y, p.z))
            .collect(),
        faces: mesh.faces.clone(),
    }
}

/// The captured scene: the modeled disc + the two vertebrae (all native mm).
#[derive(Resource)]
struct Scene {
    disc: Surface,
    l4: Surface,
    l5: Surface,
}

fn build_scene() -> Scene {
    let l4 = load_from_env("CF_L4_STL").expect("set $CF_L4_STL (superior vertebra, FMA13075)");
    let l5 = load_from_env("CF_L5_STL").expect("set $CF_L5_STL (inferior vertebra, FMA13076)");
    let disc_scan =
        load_from_env("CF_DISC_STL").expect("set $CF_DISC_STL (disc footprint, FMA16036)");
    let disc = model_disc_between_endplates(&l4, &l5, &disc_scan, &DiscModelParams::default())
        .expect("model the disc between the endplates");
    println!(
        "modeled-disc: disc {} verts / {} faces carved between L4 ({} v) and L5 ({} v)",
        disc.vertices.len(),
        disc.faces.len(),
        l4.vertices.len(),
        l5.vertices.len(),
    );
    Scene {
        disc: to_surface(&disc),
        l4: to_surface(&l4),
        l5: to_surface(&l5),
    }
}

/// Marks a translucent vertebra (its opacity is `[` / `]`-adjustable).
#[derive(Component)]
struct BoneSurface;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(build_scene())
        .add_systems(Startup, setup)
        .add_systems(Update, adjust_transparency)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    scene: Res<Scene>,
) {
    // Modeled disc — teal, opaque (the star).
    commands.spawn((
        Mesh3d(meshes.add(build_soft_mesh(
            &scene.disc.verts,
            &scene.disc.faces,
            UpAxis::PlusZ,
        ))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.20, 0.62, 0.68),
            perceptual_roughness: 0.6,
            ..default()
        })),
        Transform::default(),
    ));

    // The two vertebrae — translucent ivory so the disc reads through them.
    for surf in [&scene.l4, &scene.l5] {
        commands.spawn((
            Mesh3d(meshes.add(build_soft_mesh(&surf.verts, &surf.faces, UpAxis::PlusZ))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgba(0.87, 0.84, 0.78, BONE_ALPHA_INIT),
                perceptual_roughness: 0.85,
                alpha_mode: AlphaMode::Blend,
                double_sided: true,
                cull_mode: None,
                ..default()
            })),
            BoneSurface,
            Transform::default(),
        ));
    }

    // Frame the camera on the disc (its neighbours extend beyond — orbit/zoom to see them).
    let pts: Vec<Point3<f64>> = scene.disc.verts.iter().map(|v| Point3::from(*v)).collect();
    setup_camera_and_lighting(&mut commands, &Aabb::from_points(pts.iter()), UpAxis::PlusZ);
}

/// Adjust the vertebrae's opacity with `[` (more transparent) / `]` (more opaque) so the seated
/// disc stays visible through the bone.
fn adjust_transparency(
    keys: Res<ButtonInput<KeyCode>>,
    bones: Query<&MeshMaterial3d<StandardMaterial>, With<BoneSurface>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let delta = if keys.just_pressed(KeyCode::BracketRight) {
        BONE_ALPHA_STEP
    } else if keys.just_pressed(KeyCode::BracketLeft) {
        -BONE_ALPHA_STEP
    } else {
        return;
    };
    for m in &bones {
        if let Some(mat) = materials.get_mut(&m.0) {
            let alpha = (mat.base_color.alpha() + delta).clamp(0.05, 1.0);
            mat.base_color.set_alpha(alpha);
        }
    }
}
