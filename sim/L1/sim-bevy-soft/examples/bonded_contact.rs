//! Disc-render ladder — **step 4: `bonded-contact`**.
//!
//! The one concept: a soft body **bonded to two rigid bodies AND conforming to a curved
//! rigid surface by contact — in ONE solve**. This is the disc's full FEM structure: steps
//! 2 (`bonded-sandwich`) and 3 (`curved-contact`) composed on a single box. The block is
//! **glued** top+bottom to two rigid plates (Dirichlet bonds), and its **free side** presses
//! against a rigid **sphere** (`PenaltyRigidContact` on a curved SDF). Both mechanisms must
//! coexist without fighting: the bonded faces track their plates exactly while the free side
//! stops at the curve — no lift off the bonds, no penetration into the sphere.
//!
//! Drive: the top plate is pressed **down** (axial compression, as an endplate loads a disc),
//! bottom plate fixed. The block squishes and its free +x side bulges outward (Poisson) into
//! the sphere, which caps the bulge — the side develops a spherical dimple conforming to the
//! curve, while top and bottom stay flat and glued to their plates.
//!
//! Why one solve, not two passes: the disc's real failure was the FEM bonding to flat boxes
//! (so its face lifts off the curved bone) with the annulus contact bolted on separately. Here
//! the bonds are the solver's Dirichlet set and the contact is the same solver's
//! `PenaltyRigidContact` — one Newton equilibrium carries both. `BondedSandwich` itself can't
//! host a contact (it hardwires `NullContact`), so the bond is realised the primitive way:
//! pin both faces as Dirichlet and drive their targets (see step 2's `BondedSandwich` for the
//! pose-tracking version of the same tie).
//!
//! Captured headlessly (the top face is pressed 0 → peak → 0, warm-started so each step is a
//! small increment from the last — see step 1), then looped. Orbit with LMB; the contact
//! sphere is rendered **translucent** (and its opacity is adjustable with `[` / `]`) so the
//! block's contacting side is visible *through* it — otherwise the opaque surface hides the
//! exact interface being verified.
//!
//! Run: `cargo run -p sim-bevy-soft --example bonded-contact`

#![allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.

use bevy::prelude::*;
use cf_bevy_common::prelude::OrbitCameraPlugin;
use cf_viewer::{UpAxis, setup_camera_and_lighting};
use mesh_types::Aabb;
use nalgebra::{Point3, Vector3};
use sim_bevy_soft::prelude::{apply_soft_positions, build_soft_mesh};
use sim_ml_chassis::Tensor;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, HandBuiltTetMesh, MaterialField, Mesh as SoftMesh,
    PenaltyRigidContact, Solver, SolverConfig, SphereSdf, Tet4, TranslatedSdf, Vec3, VertexId,
    pick_vertices_by_predicate,
};

/// Block edge length (m).
const EDGE: f64 = 0.1;
/// Lattice cells per block edge.
const N_CELLS: usize = 4;
/// Rigid-plate half-thickness (m).
const PLATE_H: f64 = 0.02;
/// Radius of the rigid contact sphere on the block's free +x side (m).
const DOME_R: f64 = 0.08;
/// Peak downward press of the top plate (m) — axial compression that bulges the free side.
const PRESS: f64 = 0.02;
/// Frames per half-cycle (0 → peak); the capture is press-and-release.
const HALF_FRAMES: usize = 24;
/// Neo-Hookean shear modulus μ (Pa) — soft enough to bulge visibly into the curve; λ = 4μ.
const MU: f64 = 5.0e4;
/// Quasi-static timestep (large ⇒ inertia negligible).
const STATIC_DT: f64 = 1.0e3;
/// Replay seconds per frame (20 fps).
const FRAME_DT: f64 = 0.05;
/// Initial opacity of the (translucent) contact sphere — low enough to see the block's
/// contacting side through it, high enough to read the curve.
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
/// and the static contact sphere.
#[derive(Resource)]
struct Scene {
    block_rest: Vec<Vec3>,
    block_boundary: Vec<[VertexId; 3]>,
    block_frames: Vec<Vec<f64>>,
    lower_plate: Vec<Vec3>,
    upper_plate_frames: Vec<Vec<f64>>,
    dome_center: Vec3,
    dome_radius: f64,
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

/// Press the top face down onto the block (bottom face fixed), capturing one warm-started
/// bond+contact solve per frame (0 → `PRESS` → 0). Both faces are Dirichlet bonds; the free
/// +x side penalty-contacts the sphere in the SAME solve.
fn capture() -> Scene {
    let field = MaterialField::uniform(MU, 4.0 * MU);
    let mesh = HandBuiltTetMesh::uniform_block(N_CELLS, EDGE, &field);
    let rest: Vec<Vec3> = mesh.positions().to_vec();
    let boundary: Vec<[VertexId; 3]> = mesh.boundary_faces().to_vec();
    let n = rest.len();

    // Both planar faces are bonded (Dirichlet): bottom fixed, top driven down. The four side
    // faces (incl. the +x face that meets the sphere) are FREE.
    let bottom = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
    let top = pick_vertices_by_predicate(&mesh, |p| (p.z - EDGE).abs() < 1e-9);

    // Rigid sphere just tangent to the block's +x face at rest (centre one radius past the
    // face); as the block compresses, the +x side bulges into it and conforms to the cap.
    let dome_center = Vec3::new(EDGE + DOME_R, EDGE * 0.5, EDGE * 0.5);
    let contact = PenaltyRigidContact::new([TranslatedSdf {
        inner: SphereSdf { radius: DOME_R },
        offset: dome_center,
    }]);

    // One solver hosting BOTH mechanisms: the pinned set is both bonded faces; the contact is
    // the sphere. Built once (topology + Dirichlet set constant; only targets change per step).
    let mut pinned = bottom.clone();
    pinned.extend_from_slice(&top);
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = 60;
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
        "bonded-contact: captured {} frames; block {} nodes, bonds top+bottom, press {PRESS} m, \
         free +x side contacts R={DOME_R} m sphere",
        frames.len(),
        rest.len()
    );
    Scene {
        block_rest: rest,
        block_boundary: boundary,
        block_frames: frames,
        lower_plate,
        upper_plate_frames,
        dome_center,
        dome_radius: DOME_R,
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

    // Deforming block.
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
    // Static contact sphere on the free +x side (mapped through the same UpAxis as the meshes).
    // Translucent so the block's contacting side is visible through it — `[` / `]` adjust opacity.
    let center = UpAxis::PlusZ.to_bevy_point(&Point3::from(scene.dome_center));
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(scene.dome_radius as f32))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.72, 0.72, 0.76, CONTACT_ALPHA_INIT),
            perceptual_roughness: 0.9,
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        ContactSurface,
        Transform::from_translation(center.into()),
    ));

    // Frame the camera on the block + both plates + the sphere extent.
    let mut pts: Vec<Point3<f64>> = scene.block_rest.iter().map(|v| Point3::from(*v)).collect();
    pts.extend(scene.lower_plate.iter().map(|v| Point3::from(*v)));
    pts.extend(upper_rest.iter().map(|v| Point3::from(*v)));
    pts.push(Point3::from(
        scene.dome_center + Vec3::new(scene.dome_radius, 0.0, 0.0),
    ));
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

/// Adjust the contact sphere's opacity with `[` (more transparent) / `]` (more opaque) — a
/// live transparency "slider" so the contact interface stays visible through the sphere.
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
