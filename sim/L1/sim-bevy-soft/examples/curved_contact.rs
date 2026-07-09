//! Disc-render ladder — **step 3: `curved-contact`**.
//!
//! The one concept: a soft body's free surface **conforming to a CURVED rigid surface by
//! CONTACT** (not bonding) — no penetration, no lift, it just presses against the curve. This
//! is the physics the FSU disc annulus was missing: the disc FEM only *bonds* to flat boxes
//! and has no knowledge of the curved vertebrae, so its annulus bulges freely into/off the
//! bone. Here a soft block is pressed down onto a rigid **sphere** (a curved SDF) via
//! `PenaltyRigidContact` — and its free bottom face wraps onto the dome, stopping exactly at
//! the surface. On a box + sphere it's obvious whether contact works; on the disc it was not.
//!
//! Key discovery this step banks: soft↔curved-rigid contact needs NO new machinery —
//! `PenaltyRigidContact` accepts any `Sdf` primitive (a sphere here; the real bone SDF in the
//! FSU), so the disc annulus can penalty-contact the actual vertebra surface.
//!
//! Captured headlessly (the top face is pressed down 0 → peak → 0, warm-started), then looped.
//! Orbit with LMB; the rigid dome is rendered **translucent** (opacity adjustable with `[` /
//! `]`) so the block's contacting face is visible *through* it — otherwise the opaque surface
//! hides the exact interface being verified.
//!
//! Run: `cargo run -p sim-bevy-soft --example curved-contact`

#![allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.

use bevy::prelude::*;
use cf_bevy_common::prelude::OrbitCameraPlugin;
use cf_viewer::{UpAxis, setup_camera_and_lighting};
use mesh_types::Aabb;
use nalgebra::Point3;
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
/// Radius of the rigid contact sphere — a pronounced dome under the block (m).
const DOME_R: f64 = 0.08;
/// Peak downward press of the top face (m) — squeezes the block onto the dome.
const PRESS: f64 = 0.03;
/// Frames per half-cycle (0 → peak); the capture is press-and-release.
const HALF_FRAMES: usize = 24;
/// Neo-Hookean shear modulus μ (Pa) — soft enough to wrap the dome visibly; λ = 4μ.
const MU: f64 = 5.0e4;
/// Quasi-static timestep (large ⇒ inertia negligible).
const STATIC_DT: f64 = 1.0e3;
/// Replay seconds per frame (20 fps).
const FRAME_DT: f64 = 0.05;
/// Initial opacity of the (translucent) rigid dome — low enough to see the block's contacting
/// face through it, high enough to read the curve.
const CONTACT_ALPHA_INIT: f32 = 0.4;
/// Opacity step per `[` / `]` press.
const CONTACT_ALPHA_STEP: f32 = 0.05;

/// The captured scene: the deforming block + the static dome (sphere).
#[derive(Resource)]
struct Scene {
    block_rest: Vec<Vec3>,
    block_boundary: Vec<[VertexId; 3]>,
    block_frames: Vec<Vec<f64>>,
    dome_center: Vec3,
    dome_radius: f64,
}

/// Press the block's top face down onto the rigid sphere, capturing one warm-started
/// penalty-contact solve per frame (0 → `PRESS` → 0).
fn capture() -> Scene {
    let field = MaterialField::uniform(MU, 4.0 * MU);
    let mesh = HandBuiltTetMesh::uniform_block(N_CELLS, EDGE, &field);
    let rest: Vec<Vec3> = mesh.positions().to_vec();
    let boundary: Vec<[VertexId; 3]> = mesh.boundary_faces().to_vec();
    let n = rest.len();

    // Pin (and drive) the top face; the bottom face is FREE and contacts the dome.
    let top = pick_vertices_by_predicate(&mesh, |p| (p.z - EDGE).abs() < 1e-9);

    // Rigid dome: a sphere centred below the block so its top just touches the block bottom
    // (z = 0) at the centre and curves away — the block must wrap onto it.
    let dome_center = Vec3::new(EDGE * 0.5, EDGE * 0.5, -DOME_R);
    let contact = PenaltyRigidContact::new([TranslatedSdf {
        inner: SphereSdf { radius: DOME_R },
        offset: dome_center,
    }]);

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = 60;
    let solver = CpuNewtonSolver::new(
        Tet4,
        mesh,
        contact,
        cfg,
        BoundaryConditions::new(top.clone(), Vec::new()),
    );

    let flatten = |ps: &[Vec3]| -> Vec<f64> {
        let mut v = Vec::with_capacity(3 * ps.len());
        for p in ps {
            v.extend_from_slice(&[p.x, p.y, p.z]);
        }
        v
    };
    let depth_at = |i: usize| (i as f64 / HALF_FRAMES as f64) * PRESS;
    let mut frames = Vec::with_capacity(2 * HALF_FRAMES + 1);
    let mut warm = flatten(&rest);
    for i in (0..=HALF_FRAMES).chain((0..HALF_FRAMES).rev()) {
        let depth = depth_at(i);
        let mut x_prev = warm.clone();
        for &vid in &top {
            let idx = vid as usize;
            x_prev[3 * idx] = rest[idx].x;
            x_prev[3 * idx + 1] = rest[idx].y;
            x_prev[3 * idx + 2] = rest[idx].z - depth;
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
    println!(
        "curved-contact: captured {} frames; block {} nodes press {PRESS} m onto R={DOME_R} m dome",
        frames.len(),
        rest.len()
    );
    Scene {
        block_rest: rest,
        block_boundary: boundary,
        block_frames: frames,
        dome_center,
        dome_radius: DOME_R,
    }
}

/// Marks the deforming soft block.
#[derive(Component)]
struct SoftBlock;
/// Marks the translucent rigid dome (its opacity is `[` / `]`-adjustable).
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
    // Deforming block.
    commands.spawn((
        Mesh3d(meshes.add(build_soft_mesh(
            &scene.block_rest,
            &scene.block_boundary,
            UpAxis::PlusZ,
        ))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.20, 0.62, 0.68),
            perceptual_roughness: 0.7,
            ..default()
        })),
        SoftBlock,
        Transform::default(),
    ));
    // Static rigid dome (the sphere the block contacts). Bevy's sphere is centred at the
    // transform; map the SDF centre through the same UpAxis convention as the soft mesh.
    // Translucent so the block's contacting face is visible through it — `[` / `]` adjust opacity.
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

    let pts: Vec<Point3<f64>> = scene.block_rest.iter().map(|v| Point3::from(*v)).collect();
    setup_camera_and_lighting(&mut commands, &Aabb::from_points(pts.iter()), UpAxis::PlusZ);
}

/// Loop the press-and-release deformation onto the block's rendered surface.
fn animate(
    time: Res<Time>,
    scene: Res<Scene>,
    mut meshes: ResMut<Assets<Mesh>>,
    block: Query<&Mesh3d, With<SoftBlock>>,
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
}

/// Adjust the rigid dome's opacity with `[` (more transparent) / `]` (more opaque) — a live
/// transparency "slider" so the contact interface stays visible through the sphere.
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
