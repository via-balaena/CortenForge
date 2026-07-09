//! Disc-render ladder — **step 5.1a: `box-on-mesh-sphere`**.
//!
//! The one concept: **penalty contact against a mesh-derived SDF, via a GRID.** This is step 4
//! (`bonded-contact`) with the contact surface changed from the analytic `SphereSdf` to a
//! signed-distance GRID (`CachedGridSdf`) voxelised from a tessellated sphere *mesh* — same
//! smooth geometry, at the same place.
//!
//! Why a grid, not the mesh oracle directly: this rung isolates the SDF *representation* on smooth
//! geometry (the jump straight to the real vertebra, 5.1, conflated a mesh SDF with irregular
//! spiky geometry and the box impaled a process). Isolated, the representation itself bites — a
//! raw `mesh_sdf::Signed` oracle's gradient is a FINITE DIFFERENCE over a faceted surface, so the
//! contact normal jumps between facets and the Newton solve STALLS (measured: the analytic sphere
//! converges to 1e-10; the *mesh oracle* of the same sphere stalls Armijo at ~1e-3, and no amount
//! of tol / smoothing / normal-averaging clears it). The fix — which the FSU's own facet contact
//! already uses — is to voxelise the mesh into a grid: `CachedGridSdf` has a smooth analytic
//! trilinear gradient (no FD step), so penalty contact converges cleanly. `tol` is still loosened
//! (a grid has a modest interpolation residual floor; `skeleton()`'s 1e-10 is for analytic SDFs).
//!
//! rendered === contacts: the SAME tessellated sphere mesh is what renders AND what the grid was
//! built from. Non-penetration is verified over EVERY box vertex (a companion headless check),
//! not a face subset — the mistake that hid the 5.1 impalement.
//!
//! Captured headlessly (top face pressed 0 → peak → 0, warm-started), then looped. Orbit with
//! LMB; the sphere renders translucent (`[` / `]` adjust opacity).
//!
//! Run: `cargo run -p sim-bevy-soft --example box-on-mesh-sphere`

#![allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
#![allow(clippy::expect_used)] // an example may expect on its own fixed scene/SDF construction.

use std::f64::consts::PI;

use bevy::prelude::*;
use cf_bevy_common::prelude::OrbitCameraPlugin;
use cf_geometry::IndexedMesh;
use cf_viewer::{UpAxis, setup_camera_and_lighting};
use mesh_sdf::{CachedGridSdf, TriMeshDistance, WALL_THRESHOLD_FACTOR_DEFAULT};
use mesh_types::Aabb;
use nalgebra::{Point3, Vector3};
use sim_bevy_soft::prelude::{apply_soft_positions, build_soft_mesh};
use sim_ml_chassis::Tensor;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, HandBuiltTetMesh, MaterialField, Mesh as SoftMesh,
    PenaltyRigidContact, Solver, SolverConfig, Tet4, Vec3, VertexId, pick_vertices_by_predicate,
};

/// Block edge length (m).
const EDGE: f64 = 0.1;
/// Lattice cells per block edge.
const N_CELLS: usize = 4;
/// Rigid-plate half-thickness (m).
const PLATE_H: f64 = 0.02;
/// Radius of the rigid contact sphere on the block's free +x side (m).
const DOME_R: f64 = 0.08;
/// Sphere tessellation (latitude rings × longitude sectors) — fine enough that the mesh SDF is a
/// smooth stand-in for the analytic sphere.
const SPHERE_RINGS: usize = 32;
const SPHERE_SECTORS: usize = 48;
/// Grid-SDF lattice spacing (m) — the sphere is voxelised at this resolution; the trilinear
/// interpolant's gradient is what makes penalty contact converge (vs the mesh oracle's FD normal).
const SPHERE_CELL: f64 = 4.0e-3;
/// Peak downward press of the top plate (m) — axial compression that bulges the free side.
const PRESS: f64 = 0.02;
/// Frames per half-cycle (0 → peak); the capture is press-and-release.
const HALF_FRAMES: usize = 24;
/// Neo-Hookean shear modulus μ (Pa) — soft enough to bulge visibly into the curve; λ = 4μ.
const MU: f64 = 5.0e4;
/// Quasi-static timestep (large ⇒ inertia negligible).
const STATIC_DT: f64 = 1.0e3;
/// Newton tolerance — a mesh-SDF's FD gradient floors the residual near ~1e-4, so `skeleton()`'s
/// 1e-10 (fine for the analytic sphere) is unreachable; `5e-4` is visually exact.
const CONTACT_TOL: f64 = 5.0e-4;
/// Replay seconds per frame (20 fps).
const FRAME_DT: f64 = 0.05;
/// Initial opacity of the (translucent) contact sphere.
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

/// A watertight UV-sphere `IndexedMesh` centred at `center` (poles as single verts, quad strips
/// between rings). Winding is corrected to outward-CCW via the signed volume so the pseudo-normal
/// sign oracle reads the inside/outside correctly.
fn uv_sphere(center: Vec3, radius: f64, rings: usize, sectors: usize) -> IndexedMesh {
    let mut verts: Vec<Point3<f64>> = Vec::new();
    verts.push(Point3::new(center.x, center.y, center.z + radius)); // north pole = index 0
    for i in 1..rings {
        let theta = PI * (i as f64) / (rings as f64); // 0 (north) → π (south)
        let (z, r) = (radius * theta.cos(), radius * theta.sin());
        for j in 0..sectors {
            let phi = 2.0 * PI * (j as f64) / (sectors as f64);
            verts.push(Point3::new(
                center.x + r * phi.cos(),
                center.y + r * phi.sin(),
                center.z + z,
            ));
        }
    }
    verts.push(Point3::new(center.x, center.y, center.z - radius)); // south pole
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
        // Flip winding to outward-CCW (positive signed volume) so the sign oracle is correct.
        let faces = mesh.faces.iter().map(|&[a, b, c]| [a, c, b]).collect();
        IndexedMesh::from_parts(mesh.vertices, faces)
    }
}

/// The captured scene: the deforming block, the two plates, and the static sphere (its mesh, the
/// same one the contact oracle was built from).
#[derive(Resource)]
struct Scene {
    block_rest: Vec<Vec3>,
    block_boundary: Vec<[VertexId; 3]>,
    block_frames: Vec<Vec<f64>>,
    lower_plate: Vec<Vec3>,
    upper_plate_frames: Vec<Vec<f64>>,
    sphere_verts: Vec<Vec3>,
    sphere_faces: Vec<[VertexId; 3]>,
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

/// Press the top face down (bottom fixed), capturing one warm-started bond+contact solve per
/// frame (0 → `PRESS` → 0). Both faces are Dirichlet bonds; the free +x side penalty-contacts the
/// sphere's MESH SDF in the same solve.
fn capture() -> Scene {
    let field = MaterialField::uniform(MU, 4.0 * MU);
    let mesh = HandBuiltTetMesh::uniform_block(N_CELLS, EDGE, &field);
    let rest: Vec<Vec3> = mesh.positions().to_vec();
    let boundary: Vec<[VertexId; 3]> = mesh.boundary_faces().to_vec();
    let n = rest.len();

    let bottom = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
    let top = pick_vertices_by_predicate(&mesh, |p| (p.z - EDGE).abs() < 1e-9);

    // Sphere just tangent to the +x face at rest (centre one radius past the face), same as
    // step 4 — but the contact SDF is the mesh oracle, not the analytic primitive.
    let sphere_center = Vec3::new(EDGE + DOME_R, EDGE * 0.5, EDGE * 0.5);
    let sphere_mesh = uv_sphere(sphere_center, DOME_R, SPHERE_RINGS, SPHERE_SECTORS);
    let sphere_verts: Vec<Vec3> = sphere_mesh
        .vertices
        .iter()
        .map(|p| Vec3::new(p.x, p.y, p.z))
        .collect();
    let sphere_faces = sphere_mesh.faces.clone();

    // Voxelise the sphere mesh into a GRID SDF (smooth trilinear gradient), and contact THAT —
    // the mesh oracle's finite-difference normal stalls the solve (see the module docs).
    let bbox = Aabb::from_points(sphere_mesh.vertices.iter());
    let pad = Vector3::repeat(4.0 * SPHERE_CELL);
    let bounds = Aabb::from_points([bbox.min - pad, bbox.max + pad].iter());
    let distance = TriMeshDistance::new(sphere_mesh).expect("sphere BVH");
    let (grid, _report) = CachedGridSdf::build(
        &distance,
        bounds,
        SPHERE_CELL,
        WALL_THRESHOLD_FACTOR_DEFAULT,
    )
    .expect("build sphere grid SDF");
    let contact = PenaltyRigidContact::new([grid]);

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
    let mut upper_plate_frames = Vec::with_capacity(2 * HALF_FRAMES + 1);
    let mut warm = flatten(&rest);
    for i in (0..=HALF_FRAMES).chain((0..HALF_FRAMES).rev()) {
        let press = press_at(i);
        let mut x_prev = warm.clone();
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
        frames.push(x_final);
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
        "box-on-mesh-sphere: captured {} frames; block {} nodes; contact = grid SDF (cell \
         {SPHERE_CELL} m) voxelised from a {}-tri sphere (R={DOME_R} m)",
        frames.len(),
        rest.len(),
        sphere_faces.len(),
    );
    Scene {
        block_rest: rest,
        block_boundary: boundary,
        block_frames: frames,
        lower_plate,
        upper_plate_frames,
        sphere_verts,
        sphere_faces,
    }
}

/// Marks the deforming soft block.
#[derive(Component)]
struct SoftBlock;
/// Marks the pressing top plate.
#[derive(Component)]
struct TopPlate;
/// Marks the translucent rigid contact sphere (its opacity is `[` / `]`-adjustable).
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
    let block_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.20, 0.62, 0.68),
        perceptual_roughness: 0.7,
        ..default()
    });
    let plate_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.72, 0.72, 0.76),
        perceptual_roughness: 0.9,
        ..default()
    });

    commands.spawn((
        Mesh3d(meshes.add(build_soft_mesh(
            &scene.block_rest,
            &scene.block_boundary,
            UpAxis::PlusZ,
        ))),
        MeshMaterial3d(block_mat),
        SoftBlock,
        Transform::default(),
    ));
    commands.spawn((
        Mesh3d(meshes.add(build_soft_mesh(
            &scene.lower_plate,
            &BOX_FACES,
            UpAxis::PlusZ,
        ))),
        MeshMaterial3d(plate_mat.clone()),
        Transform::default(),
    ));
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
    // The SAME sphere mesh the contact oracle was built from — rendered === contacts. Translucent
    // so the box's contacting side is visible through it — `[` / `]` adjust opacity.
    commands.spawn((
        Mesh3d(meshes.add(build_soft_mesh(
            &scene.sphere_verts,
            &scene.sphere_faces,
            UpAxis::PlusZ,
        ))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.72, 0.72, 0.76, CONTACT_ALPHA_INIT),
            perceptual_roughness: 0.9,
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        ContactSurface,
        Transform::default(),
    ));

    let mut pts: Vec<Point3<f64>> = scene.block_rest.iter().map(|v| Point3::from(*v)).collect();
    pts.extend(scene.lower_plate.iter().map(|v| Point3::from(*v)));
    pts.extend(upper_rest.iter().map(|v| Point3::from(*v)));
    pts.extend(scene.sphere_verts.iter().map(|v| Point3::from(*v)));
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
        }
    }
    if let Ok(m) = plate.single() {
        if let Some(mesh) = meshes.get_mut(&m.0) {
            apply_soft_positions(mesh, &scene.upper_plate_frames[idx], UpAxis::PlusZ);
        }
    }
}

/// Adjust the sphere's opacity with `[` / `]` — a live transparency "slider".
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
