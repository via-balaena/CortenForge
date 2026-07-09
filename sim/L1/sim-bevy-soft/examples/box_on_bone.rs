//! Disc-render ladder — **step 5.1b: `box-on-bone`**.
//!
//! The one concept: **penalty contact against the REAL vertebra, via a GRID SDF.** This applies
//! 5.1a's finding (`box-on-mesh-sphere`) to real anatomy: the same box, bonded top+bottom to two
//! plates and pressed so its free +x side bulges (Poisson) into the bone — but the contact
//! surface is the L5 vertebra voxelised into a `CachedGridSdf`, NOT the raw `mesh_sdf::Signed`
//! oracle.
//!
//! Why the grid (5.1a's lesson): a raw mesh oracle's finite-difference gradient chatters over
//! facets and STALLS the Newton solve — and its pseudo-normal sign flips far from the surface
//! (misreading an outside vertex as deeply inside). The grid fixes both: its gradient is a smooth
//! analytic trilinear interpolant, and its sign comes from a robust flood fill. So a PLAIN penalty
//! converges — no interior-cutoff, no smoothing, no normal-averaging (all of which the raw oracle
//! needed and which only *moved* the stall). `tol` is loosened to `5e-4` for the grid's modest
//! interpolation floor. Seating is still by UNSIGNED distance (the box is unambiguously outside
//! the bone on the -x side), verified against the SAME grid the contact uses.
//!
//! rendered === contacts: the bone is scaled to metres, recentred, and translated so its surface
//! seats just tangent to the box's free face; that SAME placed mesh renders while the grid
//! (translated to the same place) is what the box feels.
//!
//! The contact is LOCALIZED — a flat box face against an irregular vertebra touches over a small
//! patch, by geometry; the broad conforming contact is the disc's job (5.3), where the curved
//! annulus matches the bone rim.
//!
//! Captured headlessly (top face pressed 0 → peak → 0, warm-started), then looped. Orbit with
//! LMB; the bone renders **translucent** (`[` / `]` adjust opacity) so the box's contacting
//! side is visible through it.
//!
//! The STL is BodyParts3D (CC BY-SA, **not committed**). Point `$CF_L5_STL` at an L5 vertebra
//! surface STL (FMA13076).
//!
//! Run: `CF_L5_STL=/path/to/L5.stl cargo run -p sim-bevy-soft --example box-on-bone`

#![allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
#![allow(clippy::expect_used)] // an example may expect on its own required asset/env setup.

use std::sync::Arc;

use bevy::prelude::*;
use cf_bevy_common::prelude::OrbitCameraPlugin;
use cf_fsu_geometry::load_from_env;
use cf_viewer::{UpAxis, setup_camera_and_lighting};
use mesh_sdf::{CachedGridSdf, TriMeshDistance, WALL_THRESHOLD_FACTOR_DEFAULT};
use mesh_types::Aabb;
use nalgebra::{Point3, Vector3};
use sim_bevy_soft::prelude::{apply_soft_positions, build_soft_mesh};
use sim_ml_chassis::Tensor;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, HandBuiltTetMesh, MaterialField, Mesh as SoftMesh,
    PenaltyRigidContact, Sdf, Solver, SolverConfig, Tet4, TranslatedSdf, Vec3, VertexId,
    pick_vertices_by_predicate,
};

/// Block edge length (m).
const EDGE: f64 = 0.1;
/// Lattice cells per block edge.
const N_CELLS: usize = 4;
/// Rigid-plate half-thickness (m).
const PLATE_H: f64 = 0.02;
/// Native STL units (mm) → SI metres. The box is defined in metres; the vertebra is ~100 mm,
/// so at 1e-3 it becomes ~0.1 m — comparable to the box, and correct for the penalty params.
const BONE_SCALE: f64 = 1.0e-3;
/// Grid-SDF lattice spacing (m) the bone is voxelised at — the trilinear interpolant's smooth
/// gradient is what makes penalty contact converge (vs the mesh oracle's chattering FD normal).
const BONE_CELL: f64 = 3.0e-3;
/// Rest gap (m) between the box's free face and the bone at seating — a hair outside contact,
/// so the bulge under compression closes it incrementally (no from-rest penetration jump).
const SEAT_GAP: f64 = 1.0e-3;
/// Newton tolerance — a grid SDF has a modest trilinear-interpolation residual floor, so
/// `skeleton()`'s 1e-10 (for analytic SDFs) is unreachable; `5e-4` is visually exact.
const CONTACT_TOL: f64 = 5.0e-4;
/// Distance (m) over which the penetration heatmap fades from the contact edge to "clear" — a
/// box vertex farther than this from the bone renders in the neutral box colour.
const SD_FADE_M: f64 = 1.0e-2;
/// Opacity of the (semi-transparent) soft box — low enough to see the bone through it, high
/// enough to read the penetration heatmap.
const BLOCK_ALPHA: f32 = 0.55;

/// Penetration heatmap: colour a box vertex by its signed distance to the bone. **Red** = inside
/// the bone (penetrating, `sd < 0`) — the failure this rung must NOT show; yellow at the surface,
/// fading to the neutral teal box colour by `SD_FADE_M` outside. Makes non-penetration directly
/// legible where a translucent overlap could not.
fn sd_color(sd: f64) -> [f32; 4] {
    if sd < 0.0 {
        return [0.90, 0.12, 0.12, 1.0]; // RED — penetrating the bone
    }
    #[allow(clippy::cast_possible_truncation)]
    let t = (sd / SD_FADE_M).clamp(0.0, 1.0) as f32; // 0 at surface → 1 at SD_FADE_M outside
    // Lerp contact-edge yellow → neutral teal (the familiar box colour far from the bone).
    let lerp = |a: f32, b: f32| a * (1.0 - t) + b * t;
    [lerp(1.0, 0.20), lerp(0.85, 0.62), lerp(0.20, 0.68), 1.0]
}
/// Peak downward press of the top plate (m) — axial compression that bulges the free side.
const PRESS: f64 = 0.02;
/// Frames per half-cycle (0 → peak); the capture is press-and-release.
const HALF_FRAMES: usize = 24;
/// Neo-Hookean shear modulus μ (Pa) — soft enough to bulge visibly into the surface; λ = 4μ.
const MU: f64 = 5.0e4;
/// Quasi-static timestep (large ⇒ inertia negligible).
const STATIC_DT: f64 = 1.0e3;
/// Replay seconds per frame (20 fps).
const FRAME_DT: f64 = 0.05;
/// Initial opacity of the (translucent) bone — low enough to see the box's contacting side
/// through it, high enough to read the surface.
const CONTACT_ALPHA_INIT: f32 = 0.4;
/// Opacity step per `[` / `]` press.
const CONTACT_ALPHA_STEP: f32 = 0.05;

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

/// The captured scene: the deforming block, the two plates (lower fixed, upper pressing down),
/// and the static real-bone contact surface (placed vertices + faces).
#[derive(Resource)]
struct Scene {
    block_rest: Vec<Vec3>,
    block_boundary: Vec<[VertexId; 3]>,
    block_frames: Vec<Vec<f64>>,
    /// Per-frame, per-vertex penetration-heatmap colours ([`sd_color`]) — same indexing as the
    /// soft vertices, applied to the box mesh's `ATTRIBUTE_COLOR` each frame.
    block_color_frames: Vec<Vec<[f32; 4]>>,
    lower_plate: Vec<Vec3>,
    upper_plate_frames: Vec<Vec<f64>>,
    bone_verts: Vec<Vec3>,
    bone_faces: Vec<[VertexId; 3]>,
}

/// A rigid plate's 8 corners centred at `center` (translation only) — flat vertex-major xyz.
fn plate_corners(center: Point3<f64>) -> Vec<f64> {
    let half = Vector3::new(EDGE * 0.5, EDGE * 0.5, PLATE_H);
    let mut out = Vec::with_capacity(24);
    for c in CORNERS {
        let p = center + Vector3::new(c[0] * half.x, c[1] * half.y, c[2] * half.z);
        out.extend_from_slice(&[p.x, p.y, p.z]);
    }
    out
}

/// The real L5 vertebra, scaled to metres and recentred at the origin: a GRID SDF (voxelised from
/// the recentred mesh) plus the recentred render vertices/faces (placed once seated).
struct Bone {
    grid: CachedGridSdf,
    verts_centered: Vec<Point3<f64>>,
    faces: Vec<[VertexId; 3]>,
    max_x: f64,
}

fn load_bone() -> Bone {
    let mut mesh = load_from_env("CF_L5_STL")
        .expect("set $CF_L5_STL to a real L5 vertebra STL (BodyParts3D FMA13076)");
    // Recentre at the centroid, scale mm → metres, in place.
    let n = mesh.vertices.len() as f64;
    let mut c = Vector3::zeros();
    for v in &mesh.vertices {
        c += v.coords;
    }
    let centroid = Point3::from(c / n);
    for v in &mut mesh.vertices {
        *v = Point3::from((*v - centroid) * BONE_SCALE);
    }
    let max_x = mesh.vertices.iter().map(|p| p.x).fold(f64::MIN, f64::max);
    let faces = mesh.faces.clone();
    let verts_centered = mesh.vertices.clone();

    // Voxelise the recentred bone into a grid SDF (smooth trilinear gradient + flood-fill sign).
    let bbox = Aabb::from_points(mesh.vertices.iter());
    let pad = Vector3::repeat(4.0 * BONE_CELL);
    let bounds = Aabb::from_points([bbox.min - pad, bbox.max + pad].iter());
    let distance = TriMeshDistance::new(mesh).expect("build L5 distance BVH");
    let (grid, _report) =
        CachedGridSdf::build(&distance, bounds, BONE_CELL, WALL_THRESHOLD_FACTOR_DEFAULT)
            .expect("build L5 grid SDF");
    Bone {
        grid,
        verts_centered,
        faces,
        max_x,
    }
}

/// Press the top face down onto the block (bottom face fixed), capturing one warm-started
/// bond+contact solve per frame (0 → `PRESS` → 0). Both faces are Dirichlet bonds; the free
/// +x side penalty-contacts the REAL bone SDF in the SAME solve.
fn capture() -> Scene {
    let field = MaterialField::uniform(MU, 4.0 * MU);
    let mesh = HandBuiltTetMesh::uniform_block(N_CELLS, EDGE, &field);
    let rest: Vec<Vec3> = mesh.positions().to_vec();
    let boundary: Vec<[VertexId; 3]> = mesh.boundary_faces().to_vec();
    let n = rest.len();

    // Both planar faces are bonded (Dirichlet): bottom fixed, top driven down. The four side
    // faces (incl. the +x face that meets the bone) are FREE.
    let bottom = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
    let top = pick_vertices_by_predicate(&mesh, |p| (p.z - EDGE).abs() < 1e-9);
    let xface: Vec<Vec3> = pick_vertices_by_predicate(&mesh, |p| (p.x - EDGE).abs() < 1e-9)
        .iter()
        .map(|&vid| rest[vid as usize])
        .collect();

    // Seat the bone on the +x side: march its centre in from clear of the box until the nearest
    // free-face vertex is `SEAT_GAP` from the bone surface. Seat by UNSIGNED distance — the box is
    // unambiguously outside the bone on the -x side. `sd(p) = grid.eval(p − place_center)` (the
    // recentred grid shifted to `place_center`, exactly what the `TranslatedSdf` below evaluates),
    // so seat and contact agree by construction.
    let bone = load_bone();
    let mut place_x = EDGE + bone.max_x + 0.1; // bone fully clear on +x
    loop {
        let pc = Vec3::new(place_x, EDGE * 0.5, EDGE * 0.5);
        let min_gap = xface
            .iter()
            .map(|&p| bone.grid.eval(Point3::from(p - pc)).abs())
            .fold(f64::INFINITY, f64::min);
        if min_gap <= SEAT_GAP || place_x < EDGE - 0.05 {
            break;
        }
        place_x -= 5.0e-4;
    }
    let place_center = Vec3::new(place_x, EDGE * 0.5, EDGE * 0.5);

    // The placed bone: render vertices (recentred + `place_center`) and the contact SDF (the
    // recentred grid translated by `place_center`) are the SAME surface — rendered === contacts.
    let bone_verts: Vec<Vec3> = bone
        .verts_centered
        .iter()
        .map(|p| Vec3::new(p.x, p.y, p.z) + place_center)
        .collect();
    let bone_faces = bone.faces.clone();
    // PLAIN penalty on the grid SDF — the grid's smooth gradient + flood-fill sign need no
    // interior-cutoff / smoothing / normal-averaging (see the module docs; 5.1a's finding). The
    // placed SDF is shared (`Arc`) so the SAME field drives the contact AND the per-frame
    // penetration heatmap ([`sd_color`]) — the readout is of exactly what the box feels.
    let placed = Arc::new(TranslatedSdf {
        inner: bone.grid,
        offset: place_center,
    });
    let contact = PenaltyRigidContact::new([Arc::clone(&placed)]);

    // One solver hosting BOTH mechanisms: the pinned set is both bonded faces; the contact is
    // the bone. Built once (topology + Dirichlet set constant; only targets change per step).
    let mut pinned = bottom.clone();
    pinned.extend_from_slice(&top);
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = 60;
    cfg.tol = CONTACT_TOL; // mesh-SDF FD-gradient residual floor; skeleton()'s 1e-10 unreachable
    let solver = CpuNewtonSolver::new(
        Tet4,
        mesh,
        contact,
        cfg,
        BoundaryConditions::new(pinned, Vec::new()),
    );

    let flatten = |ps: &[Vec3]| -> Vec<f64> {
        let mut v = Vec::with_capacity(3 * ps.len());
        for p in ps {
            v.extend_from_slice(&[p.x, p.y, p.z]);
        }
        v
    };
    let press_at = |i: usize| (i as f64 / HALF_FRAMES as f64) * PRESS;

    let mut frames = Vec::with_capacity(2 * HALF_FRAMES + 1);
    let mut color_frames = Vec::with_capacity(2 * HALF_FRAMES + 1);
    let mut upper_plate_frames = Vec::with_capacity(2 * HALF_FRAMES + 1);
    let mut warm = flatten(&rest);
    for i in (0..=HALF_FRAMES).chain((0..HALF_FRAMES).rev()) {
        let press = press_at(i);
        let mut x_prev = warm.clone();
        // Bottom bond held at rest; top bond driven down by `press`.
        for &vid in &bottom {
            let idx = vid as usize;
            x_prev[3 * idx] = rest[idx].x;
            x_prev[3 * idx + 1] = rest[idx].y;
            x_prev[3 * idx + 2] = rest[idx].z;
        }
        for &vid in &top {
            let idx = vid as usize;
            x_prev[3 * idx] = rest[idx].x;
            x_prev[3 * idx + 1] = rest[idx].y;
            x_prev[3 * idx + 2] = rest[idx].z - press;
        }
        let x_final = solver
            .replay_step(
                &Tensor::from_slice(&x_prev, &[3 * n]),
                &Tensor::zeros(&[3 * n]),
                &Tensor::zeros(&[0]),
                cfg.dt,
            )
            .x_final;
        warm = x_final.clone();
        // Penetration heatmap: each soft vertex coloured by its signed distance to the placed
        // bone SDF (the SAME field the contact uses) — red iff it is inside the bone.
        let colors: Vec<[f32; 4]> = (0..n)
            .map(|v| {
                let p = Point3::new(x_final[3 * v], x_final[3 * v + 1], x_final[3 * v + 2]);
                sd_color(placed.eval(p))
            })
            .collect();
        color_frames.push(colors);
        frames.push(x_final);
        // Upper plate tracks the driven top face: its bottom sits at z = EDGE − press.
        upper_plate_frames.push(plate_corners(Point3::new(
            EDGE * 0.5,
            EDGE * 0.5,
            EDGE + PLATE_H - press,
        )));
    }

    let lower_plate: Vec<Vec3> = plate_corners(Point3::new(EDGE * 0.5, EDGE * 0.5, -PLATE_H))
        .chunks_exact(3)
        .map(|c| Vec3::new(c[0], c[1], c[2]))
        .collect();

    println!(
        "box-on-bone: captured {} frames; block {} nodes, bonds top+bottom, press {PRESS} m, \
         free +x side contacts real L5 bone (seated at x={place_x:.4} m, {} bone tris)",
        frames.len(),
        rest.len(),
        bone_faces.len(),
    );
    Scene {
        block_rest: rest,
        block_boundary: boundary,
        block_frames: frames,
        block_color_frames: color_frames,
        lower_plate,
        upper_plate_frames,
        bone_verts,
        bone_faces,
    }
}

/// Marks the deforming soft block.
#[derive(Component)]
struct SoftBlock;
/// Marks the pressing top plate.
#[derive(Component)]
struct TopPlate;
/// Marks the translucent rigid bone contact surface (its opacity is `[` / `]`-adjustable).
#[derive(Component)]
struct ContactSurface;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(capture())
        .add_systems(Startup, setup)
        .add_systems(Update, (animate, adjust_transparency))
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    scene: Res<Scene>,
) {
    // White base so the per-vertex penetration heatmap ([`sd_color`]) is the final colour; a
    // `BLOCK_ALPHA` alpha (times the vertices' opaque alpha) makes the box semi-transparent so the
    // bone is visible through it while the heatmap still reads.
    let block_mat = materials.add(StandardMaterial {
        base_color: Color::srgba(1.0, 1.0, 1.0, BLOCK_ALPHA),
        perceptual_roughness: 0.7,
        alpha_mode: AlphaMode::Blend,
        ..default()
    });
    let plate_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.72, 0.72, 0.76),
        perceptual_roughness: 0.9,
        ..default()
    });

    // Deforming block, carrying the frame-0 penetration heatmap as vertex colours.
    let mut block_mesh = build_soft_mesh(&scene.block_rest, &scene.block_boundary, UpAxis::PlusZ);
    block_mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, scene.block_color_frames[0].clone());
    commands.spawn((
        Mesh3d(meshes.add(block_mesh)),
        MeshMaterial3d(block_mat),
        SoftBlock,
        Transform::default(),
    ));
    // Static lower plate (the fixed bond).
    commands.spawn((
        Mesh3d(meshes.add(build_soft_mesh(
            &scene.lower_plate,
            &BOX_FACES,
            UpAxis::PlusZ,
        ))),
        MeshMaterial3d(plate_mat.clone()),
        Transform::default(),
    ));
    // Pressing upper plate (its rest corners = frame 0; animated per frame).
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
    // Real L5 bone contact surface (the SAME placed mesh the contact SDF evaluates). Translucent
    // so the box's contacting side is visible through it — `[` / `]` adjust opacity.
    commands.spawn((
        Mesh3d(meshes.add(build_soft_mesh(
            &scene.bone_verts,
            &scene.bone_faces,
            UpAxis::PlusZ,
        ))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.87, 0.84, 0.78, CONTACT_ALPHA_INIT),
            perceptual_roughness: 0.85,
            alpha_mode: AlphaMode::Blend,
            double_sided: true,
            cull_mode: None,
            ..default()
        })),
        ContactSurface,
        Transform::default(),
    ));

    // Frame the camera on the block + both plates + the bone.
    let mut pts: Vec<Point3<f64>> = scene.block_rest.iter().map(|v| Point3::from(*v)).collect();
    pts.extend(scene.lower_plate.iter().map(|v| Point3::from(*v)));
    pts.extend(upper_rest.iter().map(|v| Point3::from(*v)));
    pts.extend(scene.bone_verts.iter().map(|v| Point3::from(*v)));
    setup_camera_and_lighting(&mut commands, &Aabb::from_points(pts.iter()), UpAxis::PlusZ);
}

/// Loop the block deformation + the top-plate press in lockstep.
fn animate(
    time: Res<Time>,
    scene: Res<Scene>,
    mut meshes: ResMut<Assets<Mesh>>,
    block: Query<&Mesh3d, (With<SoftBlock>, Without<TopPlate>)>,
    plate: Query<&Mesh3d, (With<TopPlate>, Without<SoftBlock>)>,
) {
    let n = scene.block_frames.len();
    if n == 0 {
        return;
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let idx = ((time.elapsed_secs_f64() / FRAME_DT) as usize) % n;
    if let Ok(m) = block.single() {
        if let Some(mesh) = meshes.get_mut(&m.0) {
            apply_soft_positions(mesh, &scene.block_frames[idx], UpAxis::PlusZ);
            // Refresh the penetration heatmap to this frame's signed distances.
            mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, scene.block_color_frames[idx].clone());
        }
    }
    if let Ok(m) = plate.single() {
        if let Some(mesh) = meshes.get_mut(&m.0) {
            apply_soft_positions(mesh, &scene.upper_plate_frames[idx], UpAxis::PlusZ);
        }
    }
}

/// Adjust the bone's opacity with `[` (more transparent) / `]` (more opaque) — a live
/// transparency "slider" so the contact interface stays visible through the surface.
fn adjust_transparency(
    keys: Res<ButtonInput<KeyCode>>,
    surface: Query<&MeshMaterial3d<StandardMaterial>, With<ContactSurface>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let delta = if keys.just_pressed(KeyCode::BracketRight) {
        CONTACT_ALPHA_STEP
    } else if keys.just_pressed(KeyCode::BracketLeft) {
        -CONTACT_ALPHA_STEP
    } else {
        return;
    };
    if let Ok(m) = surface.single() {
        if let Some(mat) = materials.get_mut(&m.0) {
            let alpha = (mat.base_color.alpha() + delta).clamp(0.05, 1.0);
            mat.base_color.set_alpha(alpha);
        }
    }
}
