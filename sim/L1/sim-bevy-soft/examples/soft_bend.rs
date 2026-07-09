//! Disc-render ladder — **step 1: `soft-bend`**.
//!
//! The one concept: **FEM deformation → a smooth rendered surface**, in isolation.
//! No anatomy, no skinning, no rigid bodies, no contact — a single soft block, pinned at
//! its bottom face, sheared by prescribing a smooth lateral displacement on its top face,
//! rendered directly as its deforming tet-boundary surface.
//!
//! This is the baseline the rest of the ladder builds on: it answers "does the raw FEM
//! deformation of a *clean* mesh render as a smooth, coherent surface?" (Yes — a
//! well-conditioned block's tet boundary is watertight, so [`build_soft_mesh`] draws it
//! directly and [`apply_soft_positions`] deforms it per frame. The FSU disc could NOT do
//! this — its thin lens tet-meshes into fragments — which is why later steps introduce the
//! clean-surface-skinning machinery. Here we establish the deformation render is sound
//! before adding any of that.)
//!
//! The block is captured headlessly (a warm-started quasi-static solve per frame, shearing
//! 0 → peak → 0), then looped continuously onto the rendered surface. Orbit with LMB.
//!
//! Run: `cargo run -p sim-bevy-soft --example soft-bend`

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
    NullContact, Solver, SolverConfig, Tet4, Vec3, VertexId, pick_vertices_by_predicate,
};

/// Cube edge length (m).
const EDGE: f64 = 0.1;
/// Lattice cells per edge — enough to shear smoothly, coarse enough to solve instantly.
const N_CELLS: usize = 4;
/// Peak lateral shear of the top face, as a fraction of the edge. A smooth prescribed
/// shear keeps the boundary tets comfortably inside the solver's stretch-validity bound
/// (a large rigid top-face rotation about the far bottom edge over-stretches them).
const MAX_SHEAR_FRAC: f64 = 0.30;
/// Frames per half-cycle (0 → peak); the capture is peak-out-and-back.
const HALF_FRAMES: usize = 20;
/// Neo-Hookean shear modulus μ (Pa); λ = 4μ (the crate's standard near-incompressible ratio).
const MU: f64 = 1.0e5;
/// Replay seconds per frame (20 fps).
const FRAME_DT: f64 = 0.05;

/// The captured scene: the rest surface + its (deformation-invariant) triangulation + the
/// per-frame deformed positions the `animate` loop replays.
#[derive(Resource)]
struct Scene {
    rest: Vec<Vec3>,
    boundary: Vec<[VertexId; 3]>,
    frames: Vec<Vec<f64>>,
}

/// Bend a pinned soft block by prescribing a rotation on its top face, capturing one
/// quasi-static solve per frame (0 → `MAX_BEND_DEG` → 0).
fn capture() -> Scene {
    let field = MaterialField::uniform(MU, 4.0 * MU);
    let mesh = HandBuiltTetMesh::uniform_block(N_CELLS, EDGE, &field);
    let rest: Vec<Vec3> = mesh.positions().to_vec();
    let boundary: Vec<[VertexId; 3]> = mesh.boundary_faces().to_vec();
    let n = rest.len();

    // Pin the bottom face (fixed) and the top face (prescribed) — the interior solves.
    let bottom = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
    let top = pick_vertices_by_predicate(&mesh, |p| (p.z - EDGE).abs() < 1e-9);
    let mut pinned = bottom;
    pinned.extend_from_slice(&top);

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = 1.0e3; // large dt ⇒ inertia negligible ⇒ each solve is quasi-static.
    cfg.max_newton_iter = 50;
    let solver = CpuNewtonSolver::new(
        Tet4,
        mesh,
        NullContact,
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

    // Sweep the top-face shear 0 → peak → 0, warm-starting each solve from the previous
    // frame so every increment is small and the boundary tets stay valid throughout.
    let shear_at = |i: usize| (i as f64 / HALF_FRAMES as f64) * MAX_SHEAR_FRAC * EDGE;
    let schedule = (0..=HALF_FRAMES).chain((0..HALF_FRAMES).rev());
    let mut frames = Vec::with_capacity(2 * HALF_FRAMES + 1);
    let mut warm = flatten(&rest); // carried-forward interior warm-start
    for i in schedule {
        let shear = shear_at(i);
        let mut x_prev = warm.clone();
        // Prescribe the top face: rest position translated laterally (+y) by `shear`.
        for &vid in &top {
            let idx = vid as usize;
            x_prev[3 * idx] = rest[idx].x;
            x_prev[3 * idx + 1] = rest[idx].y + shear;
            x_prev[3 * idx + 2] = rest[idx].z;
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
    }
    // Sanity print (headless): the peak frame's max node displacement from rest.
    let rest_flat = flatten(&rest);
    let peak = &frames[HALF_FRAMES];
    let max_disp = (0..n)
        .map(|i| {
            let d = Vector3::new(
                peak[3 * i] - rest_flat[3 * i],
                peak[3 * i + 1] - rest_flat[3 * i + 1],
                peak[3 * i + 2] - rest_flat[3 * i + 2],
            );
            d.norm()
        })
        .fold(0.0_f64, f64::max);
    println!(
        "soft-bend: captured {} frames; peak node displacement {:.4} m (edge {EDGE} m)",
        frames.len(),
        max_disp
    );
    Scene {
        rest,
        boundary,
        frames,
    }
}

/// Marks the deforming soft-block mesh entity.
#[derive(Component)]
struct SoftBlock;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(capture())
        .add_systems(Startup, setup)
        .add_systems(Update, animate)
        .run();
}

/// Spawn the soft-block entity (at its rest surface) + the orbit camera framed on it.
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    scene: Res<Scene>,
) {
    let mesh = build_soft_mesh(&scene.rest, &scene.boundary, UpAxis::PlusZ);
    let material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.20, 0.62, 0.68),
        perceptual_roughness: 0.7,
        ..default()
    });
    commands.spawn((
        Mesh3d(meshes.add(mesh)),
        MeshMaterial3d(material),
        SoftBlock,
        Transform::default(),
    ));

    let pts: Vec<Point3<f64>> = scene.rest.iter().map(|v| Point3::from(*v)).collect();
    setup_camera_and_lighting(&mut commands, &Aabb::from_points(pts.iter()), UpAxis::PlusZ);
}

/// Continuously loop the captured shear (0 → peak → 0 → …) onto the block's surface, so the
/// deformation is unmistakably live rather than a one-shot that ends back at rest.
fn animate(
    time: Res<Time>,
    scene: Res<Scene>,
    mut meshes: ResMut<Assets<Mesh>>,
    block: Query<&Mesh3d, With<SoftBlock>>,
) {
    let n = scene.frames.len();
    if n == 0 {
        return;
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let idx = ((time.elapsed_secs_f64() / FRAME_DT) as usize) % n;
    for mesh3d in &block {
        if let Some(mesh) = meshes.get_mut(&mesh3d.0) {
            apply_soft_positions(mesh, &scene.frames[idx], UpAxis::PlusZ);
        }
    }
}
