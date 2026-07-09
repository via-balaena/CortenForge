//! Disc-render ladder — **step 5.2: `real-disc-bonded`**.
//!
//! The one concept: **the REAL thin-lens intervertebral disc, bonded at its endplate bands and
//! deforming smoothly.** Steps 5.0/5.1 established real anatomy as a render surface and as a grid
//! SDF contact; this rung swaps the box for the actual disc *soft body*. The disc STL is tet-meshed
//! (`SdfMeshedTetMesh` via the validated `cf_fsu_model::build_bonded_disc` recipe: recenter+scale
//! to metres, isosurface-stuff, `largest_component`), its superior/inferior **endplate bands** are
//! bonded between two rigid plates, and the superior plate is flexed about the disc's ML axis while
//! the inferior stays fixed. NO bone contact yet (that is 5.3).
//!
//! **The render's trouble spot, isolated + fixed here: the thin lens can't be drawn from its raw
//! tet boundary.** The BCC isosurface-stuffing mesher fragments the disc's sub-cell-thin tapering
//! rim into slivers, so the tet-boundary surface is a spiky, torn mess. The fix (the FSU viewer's
//! approach): draw the CLEAN STL surface and displace it by the FEM field via **K-nearest skinning**
//! — each STL vertex follows the inverse-distance-squared blend of its `SKIN_NEIGHBOURS` nearest
//! *boundary* tet nodes (blending several, not one, skins the fine surface smoothly over the coarse
//! few-mm tet field). Clean geometry from the STL; real deformation from the physics.
//!
//! Crux banked (step 5.2 confirms it): the disc FEM reaches ROM **incrementally**. A single big
//! from-rest jump stalls the boundary tets' SPD region near ~1°, but a warm-started fine sweep
//! (`capture_flexion` over 0.2° steps) converges cleanly to 6° (conservation residual ≈ 1e-13).
//!
//! Everything is in native millimetres (the vertebra/ligament frame). Captured headlessly
//! (0 → 6° → 0), then looped. Orbit with LMB. The STL is BodyParts3D (CC BY-SA, **not committed**);
//! point `$CF_DISC_STL` at the L4–L5 disc STL (FMA16036).
//!
//! Run (release — 61 real-mesh FEM solves): `CF_DISC_STL=/path/to/disc.stl cargo run --release
//! -p sim-bevy-soft --example real-disc-bonded`

#![allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
#![allow(clippy::expect_used)] // an example may expect on its own required asset/env setup.

use bevy::prelude::*;
use cf_bevy_common::prelude::OrbitCameraPlugin;
use cf_fsu_geometry::load_from_env;
use cf_fsu_model::{DiscParams, build_bonded_disc};
use cf_geometry::IndexedMesh;
use cf_viewer::{UpAxis, setup_camera_and_lighting};
use mesh_types::Aabb;
use nalgebra::{Point3, Unit, UnitQuaternion, Vector3};
use sim_bevy_soft::prelude::{apply_soft_positions, build_soft_mesh};
use sim_soft::{Vec3, VertexId};

/// Tet nodes each surface vertex blends over — enough to interpolate smoothly across the coarse
/// (few-mm) tet field without reaching past the local neighbourhood (mirrors the FSU viewer).
const SKIN_NEIGHBOURS: usize = 6;

/// Peak flexion angle (deg) — within the validated L4–L5 ROM band; reached incrementally.
const PEAK_DEG: f64 = 6.0;
/// Flexion sub-step (deg) — fine enough that each solve warm-starts the previous (crux #1).
const STEP_DEG: f64 = 0.2;
/// Rigid endplate-plate half-thickness (native mm).
const PLATE_H_MM: f64 = 5.0;
/// Plate footprint vs the disc's in-plane extent (slightly larger so the disc reads as sandwiched).
const PLATE_SCALE: f64 = 1.05;
/// Replay seconds per frame (20 fps).
const FRAME_DT: f64 = 0.05;

/// Unit-cube corner sign pattern + outward-wound box triangulation (the two rigid plates).
const CORNERS: [[f64; 3]; 8] = [
    [-1.0, -1.0, -1.0],
    [1.0, -1.0, -1.0],
    [1.0, 1.0, -1.0],
    [-1.0, 1.0, -1.0],
    [-1.0, -1.0, 1.0],
    [1.0, -1.0, 1.0],
    [1.0, 1.0, 1.0],
    [-1.0, 1.0, 1.0],
];
const BOX_FACES: [[VertexId; 3]; 12] = [
    [0, 3, 2],
    [0, 2, 1],
    [4, 5, 6],
    [4, 6, 7],
    [0, 1, 5],
    [0, 5, 4],
    [2, 3, 7],
    [2, 7, 6],
    [0, 4, 7],
    [0, 7, 3],
    [1, 2, 6],
    [1, 6, 5],
];

/// The captured scene: the deforming disc + the two endplate plates (inferior fixed, superior
/// flexing), all in native millimetres.
#[derive(Resource)]
struct Scene {
    disc_rest: Vec<Vec3>,
    disc_boundary: Vec<[VertexId; 3]>,
    disc_frames: Vec<Vec<f64>>,
    lower_plate: Vec<Vec3>,
    upper_plate_frames: Vec<Vec<f64>>,
}

/// A rigid plate's 8 corners as a rigid body: local box `(rest_center ± half)` moved by rotating
/// `theta` about `(pivot, axis)` — flat vertex-major xyz. `rot = identity` ⇒ the plate at rest.
fn plate_corners(
    rest_center: Point3<f64>,
    half: Vector3<f64>,
    pivot: Point3<f64>,
    rot: UnitQuaternion<f64>,
) -> Vec<f64> {
    let mut out = Vec::with_capacity(24);
    for c in CORNERS {
        let rest_corner = rest_center + Vector3::new(c[0] * half.x, c[1] * half.y, c[2] * half.z);
        let posed = pivot + rot * (rest_corner - pivot);
        out.extend_from_slice(&[posed.x, posed.y, posed.z]);
    }
    out
}

/// Flatten native-mm points to vertex-major xyz.
fn flatten(ps: &[Point3<f64>]) -> Vec<f64> {
    let mut v = Vec::with_capacity(3 * ps.len());
    for p in ps {
        v.extend_from_slice(&[p.x, p.y, p.z]);
    }
    v
}

/// K-nearest skinning weights: for each `surface` vertex, the [`SKIN_NEIGHBOURS`] nearest tet nodes
/// **on the disc boundary** (`boundary_faces` — the largest component's surface nodes), with
/// inverse-distance-squared weights summing to 1. Candidates are restricted to boundary nodes so a
/// surface vertex never blends onto a dropped rim-island node (which stays at rest → would tear the
/// disc). Mirrors the FSU viewer's `weighted_tet_nodes` (`tools/cf-spine-viewer`).
fn weighted_tet_nodes(
    surface: &IndexedMesh,
    rest_nodes: &[Point3<f64>],
    boundary_faces: &[[VertexId; 3]],
) -> Vec<Vec<(usize, f64)>> {
    let mut candidates: Vec<usize> = boundary_faces
        .iter()
        .flatten()
        .map(|&v| v as usize)
        .collect();
    candidates.sort_unstable();
    candidates.dedup();
    // Distance below which a surface vertex is taken as coincident with a node (avoids /0).
    const EPS2: f64 = 1e-12;
    surface
        .vertices
        .iter()
        .map(|v| {
            let mut near: Vec<(usize, f64)> = candidates
                .iter()
                .map(|&i| (i, (rest_nodes[i] - v).norm_squared()))
                .collect();
            let k = SKIN_NEIGHBOURS.min(near.len());
            near.select_nth_unstable_by(k - 1, |a, b| a.1.total_cmp(&b.1));
            near.truncate(k);
            let raw: Vec<(usize, f64)> =
                near.iter().map(|&(i, d2)| (i, 1.0 / (d2 + EPS2))).collect();
            let total: f64 = raw.iter().map(|&(_, w)| w).sum();
            raw.into_iter().map(|(i, w)| (i, w / total)).collect()
        })
        .collect()
}

/// Displace the clean `surf_rest` STL vertices by the disc's FEM **displacement** field, skinned:
/// each surface vertex moves by the inverse-distance-squared blend of `deformed[j] − rest[j]` over
/// its `weights` tet nodes. Blending the DISPLACEMENT (not the absolute tet-node positions) keeps
/// the smooth STL geometry — the spiky tet-boundary positions never enter the render (mirrors the
/// FSU viewer's `flexion_update`). At rest (`deformed == rest`) it returns `surf_rest` exactly.
fn skin_displace(
    weights: &[Vec<(usize, f64)>],
    surf_rest: &[Point3<f64>],
    deformed: &[Point3<f64>],
    rest: &[Point3<f64>],
) -> Vec<Point3<f64>> {
    surf_rest
        .iter()
        .zip(weights)
        .map(|(sr, ws)| {
            let mut disp = Vector3::zeros();
            for &(j, w) in ws {
                disp += (deformed[j] - rest[j]) * w;
            }
            sr + disp
        })
        .collect()
}

/// Load + tet-mesh + bond the real disc, sweep flexion 0 → `PEAK_DEG` → 0 incrementally, and
/// reconstruct the two endplate plates (inferior fixed at the disc's inferior face, superior
/// flexing about the disc's ML axis through its centre).
fn capture() -> Scene {
    let disc_mesh = load_from_env("CF_DISC_STL")
        .expect("set $CF_DISC_STL to the L4–L5 disc STL (BodyParts3D FMA16036)");
    // Keep the CLEAN STL surface for rendering (native mm); the tet mesh built below shares this
    // frame (`build_bonded_disc` inverts its recenter+scale back to native mm for the readout).
    let render_surface = disc_mesh.clone();
    let mut bonded =
        build_bonded_disc(disc_mesh, &DiscParams::default()).expect("tet-mesh + bond the disc");

    // Fine monotone chain 0 → PEAK → 0 (each step warm-starts the previous solve — crux #1).
    let n = (PEAK_DEG / STEP_DEG).round() as usize;
    let mut angles = Vec::with_capacity(2 * n + 1);
    for i in 0..=n {
        angles.push((i as f64 * STEP_DEG).to_radians());
    }
    for i in (0..n).rev() {
        angles.push((i as f64 * STEP_DEG).to_radians());
    }
    let traj = bonded.capture_flexion(&angles);

    // Render the CLEAN STL surface, displaced by the FEM field via skinning (NOT the spiky tet
    // boundary). `disc_rest` is the STL geometry itself; each frame adds the smooth FEM displacement.
    let weights = weighted_tet_nodes(
        &render_surface,
        &traj.rest_nodes_native,
        &traj.boundary_faces,
    );
    let disc_boundary: Vec<[VertexId; 3]> = render_surface.faces.clone();
    let disc_rest: Vec<Vec3> = render_surface
        .vertices
        .iter()
        .map(|p| Vec3::new(p.x, p.y, p.z))
        .collect();
    let disc_frames: Vec<Vec<f64>> = traj
        .frames
        .iter()
        .map(|f| {
            flatten(&skin_displace(
                &weights,
                &render_surface.vertices,
                &f.deformed_nodes_native,
                &traj.rest_nodes_native,
            ))
        })
        .collect();

    // Reconstruct the endplate plates from the disc's RENDER-SURFACE AABB (native mm) — the tet-node
    // AABB is padded/larger than the disc, which would float the plates off it. The plates sit flush
    // against the disc's SI (thinnest = z) faces; the superior one is a rigid body flexing about the
    // disc-centre pivot on the ML axis — the same motion the bonded superior band follows.
    let aabb = Aabb::from_points(render_surface.vertices.iter());
    let cx = 0.5 * (aabb.min.x + aabb.max.x);
    let cy = 0.5 * (aabb.min.y + aabb.max.y);
    let half = Vector3::new(
        0.5 * (aabb.max.x - aabb.min.x) * PLATE_SCALE,
        0.5 * (aabb.max.y - aabb.min.y) * PLATE_SCALE,
        PLATE_H_MM,
    );
    let pivot = traj.pivot;
    let axis = Unit::new_normalize(traj.axis);

    let inf_center = Point3::new(cx, cy, aabb.min.z - PLATE_H_MM);
    let lower_plate: Vec<Vec3> = plate_corners(inf_center, half, pivot, UnitQuaternion::identity())
        .chunks_exact(3)
        .map(|c| Vec3::new(c[0], c[1], c[2]))
        .collect();

    let sup_rest = Point3::new(cx, cy, aabb.max.z + PLATE_H_MM);
    let upper_plate_frames: Vec<Vec<f64>> = traj
        .frames
        .iter()
        .map(|f| {
            let rot = UnitQuaternion::from_axis_angle(&axis, f.theta);
            plate_corners(sup_rest, half, pivot, rot)
        })
        .collect();

    println!(
        "real-disc-bonded: captured {} flexion frames (0 → {PEAK_DEG}° → 0); {} tet nodes skinned \
         onto the clean STL surface ({} verts / {} tris); AABB ({:.1}, {:.1}, {:.1}) mm",
        disc_frames.len(),
        traj.rest_nodes_native.len(),
        disc_rest.len(),
        disc_boundary.len(),
        aabb.max.x - aabb.min.x,
        aabb.max.y - aabb.min.y,
        aabb.max.z - aabb.min.z,
    );
    Scene {
        disc_rest,
        disc_boundary,
        disc_frames,
        lower_plate,
        upper_plate_frames,
    }
}

/// Marks the deforming disc.
#[derive(Component)]
struct Disc;
/// Marks the flexing superior plate.
#[derive(Component)]
struct TopPlate;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(capture())
        .add_systems(Startup, setup)
        .add_systems(Update, animate)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    scene: Res<Scene>,
) {
    let disc_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.20, 0.62, 0.68),
        perceptual_roughness: 0.6,
        ..default()
    });
    let plate_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.72, 0.72, 0.76),
        perceptual_roughness: 0.9,
        ..default()
    });

    // Deforming disc.
    commands.spawn((
        Mesh3d(meshes.add(build_soft_mesh(
            &scene.disc_rest,
            &scene.disc_boundary,
            UpAxis::PlusZ,
        ))),
        MeshMaterial3d(disc_mat),
        Disc,
        Transform::default(),
    ));
    // Fixed inferior plate.
    commands.spawn((
        Mesh3d(meshes.add(build_soft_mesh(
            &scene.lower_plate,
            &BOX_FACES,
            UpAxis::PlusZ,
        ))),
        MeshMaterial3d(plate_mat.clone()),
        Transform::default(),
    ));
    // Flexing superior plate (its rest corners = frame 0; animated per frame).
    let upper_rest: Vec<Vec3> = scene.upper_plate_frames[0]
        .chunks_exact(3)
        .map(|c| Vec3::new(c[0], c[1], c[2]))
        .collect();
    commands.spawn((
        Mesh3d(meshes.add(build_soft_mesh(&upper_rest, &BOX_FACES, UpAxis::PlusZ))),
        MeshMaterial3d(plate_mat),
        TopPlate,
        Transform::default(),
    ));

    // Frame the camera on the disc + both plates.
    let mut pts: Vec<Point3<f64>> = scene.disc_rest.iter().map(|v| Point3::from(*v)).collect();
    pts.extend(scene.lower_plate.iter().map(|v| Point3::from(*v)));
    pts.extend(upper_rest.iter().map(|v| Point3::from(*v)));
    setup_camera_and_lighting(&mut commands, &Aabb::from_points(pts.iter()), UpAxis::PlusZ);
}

/// Loop the disc deformation + the superior-plate flexion in lockstep.
fn animate(
    time: Res<Time>,
    scene: Res<Scene>,
    mut meshes: ResMut<Assets<Mesh>>,
    disc: Query<&Mesh3d, (With<Disc>, Without<TopPlate>)>,
    plate: Query<&Mesh3d, (With<TopPlate>, Without<Disc>)>,
) {
    let n = scene.disc_frames.len();
    if n == 0 {
        return;
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let idx = ((time.elapsed_secs_f64() / FRAME_DT) as usize) % n;
    if let Ok(m) = disc.single() {
        if let Some(mesh) = meshes.get_mut(&m.0) {
            apply_soft_positions(mesh, &scene.disc_frames[idx], UpAxis::PlusZ);
        }
    }
    if let Ok(m) = plate.single() {
        if let Some(mesh) = meshes.get_mut(&m.0) {
            apply_soft_positions(mesh, &scene.upper_plate_frames[idx], UpAxis::PlusZ);
        }
    }
}
