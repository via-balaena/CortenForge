//! Disc-render ladder — **step 2: `bonded-sandwich`**.
//!
//! The one concept: a soft body staying **glued to two rigid bodies** as one of them moves.
//! This is the disc's core attachment primitive ([`BondedSandwich`], the exact type the
//! intervertebral disc uses) exercised on a clean box instead of anatomy: a soft block is
//! bonded between two rigid **plates** (bottom fixed, top rotating), and the block must track
//! both — its bottom face flat on the fixed plate, its top face rigidly following the tilting
//! plate, the interior shearing between them. On a box it's obvious whether the block stays
//! attached or peels off; on the thin disc it was invisible.
//!
//! Both plates are drawn (grey) so the attachment is legible, and everything is rendered in
//! ONE frame convention (the soft mesh's [`UpAxis`] mapping) so the block and plates align.
//!
//! Captured headlessly (the top plate rotates 0 → peak → 0, warm-started so the boundary tets
//! stay valid — see step 1), then looped. Orbit with LMB.
//!
//! Run: `cargo run -p sim-bevy-soft --example bonded-sandwich`

#![allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
#![allow(clippy::expect_used)] // an example may expect on its own fixed scene setup.

use bevy::prelude::*;
use cf_bevy_common::prelude::OrbitCameraPlugin;
use cf_viewer::{UpAxis, setup_camera_and_lighting};
use mesh_types::Aabb;
use nalgebra::{Point3, UnitQuaternion, Vector3};
use sim_bevy_soft::prelude::{apply_soft_positions, build_soft_mesh};
use sim_coupling::BondedSandwich;
use sim_mjcf::load_model;
use sim_soft::{Vec3, VertexId};

/// Block edge length (m).
const EDGE: f64 = 0.1;
/// Lattice cells per block edge.
const N_CELLS: usize = 4;
/// Rigid-plate half-thickness (m).
const PLATE_H: f64 = 0.02;
/// Peak rotation of the top plate (deg). Modest + warm-started ⇒ boundary tets stay valid.
const MAX_ROT_DEG: f64 = 12.0;
/// Frames per half-cycle (0 → peak); the capture is peak-out-and-back.
const HALF_FRAMES: usize = 20;
/// Neo-Hookean shear modulus μ (Pa); λ = 4μ.
const MU: f64 = 1.0e5;
/// Quasi-static timestep (large ⇒ inertia negligible).
const STATIC_DT: f64 = 1.0e3;
/// Replay seconds per frame (20 fps).
const FRAME_DT: f64 = 0.05;

/// Body indices in the two-plate MJCF scene (world = 0).
const LOWER: usize = 1;
const UPPER: usize = 2;

/// Unit-cube corner sign pattern + outward-wound box triangulation (shared by both plates).
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

/// The captured scene: the deforming block + the two plates (the lower one static).
#[derive(Resource)]
struct Scene {
    block_rest: Vec<Vec3>,
    block_boundary: Vec<[VertexId; 3]>,
    block_frames: Vec<Vec<f64>>,
    lower_plate: Vec<Vec3>,
    upper_plate_frames: Vec<Vec<f64>>,
}

/// The two rigid plates as free-joint boxes: the lower's top face sits at z = 0 and the
/// upper's bottom at z = `EDGE`, so they coincide with the block's bonded faces.
fn plate_scene_mjcf() -> String {
    let half = EDGE * 0.5; // plate footprint = block footprint
    format!(
        r#"<mujoco><option gravity="0 0 0" timestep="0.001"/><worldbody>
    <body name="lower" pos="{half} {half} {lz}"><freejoint/><geom type="box" size="{half} {half} {h}" mass="0.05"/></body>
    <body name="upper" pos="{half} {half} {uz}"><freejoint/><geom type="box" size="{half} {half} {h}" mass="0.05"/></body>
    </worldbody></mujoco>"#,
        lz = -PLATE_H,
        uz = EDGE + PLATE_H,
        h = PLATE_H,
    )
}

/// A rigid plate's 8 corners posed at `(center, rot)` — flat vertex-major xyz.
fn plate_corners(center: Point3<f64>, rot: UnitQuaternion<f64>) -> Vec<f64> {
    let half = Vector3::new(EDGE * 0.5, EDGE * 0.5, PLATE_H);
    let mut out = Vec::with_capacity(24);
    for c in CORNERS {
        let local = Vector3::new(c[0] * half.x, c[1] * half.y, c[2] * half.z);
        let p = center + rot * local;
        out.extend_from_slice(&[p.x, p.y, p.z]);
    }
    out
}

/// Rotate the top plate 0 → peak → 0, capturing the block's deformation + the plate poses.
fn capture() -> Scene {
    let model = load_model(&plate_scene_mjcf()).expect("plate scene MJCF");
    let mut data = model.make_data();
    data.forward(&model).expect("scene forward");
    let rest_lower = Point3::from(data.xpos[LOWER]);
    let rest_upper = Point3::from(data.xpos[UPPER]);
    let mut sandwich = BondedSandwich::new(model, data, LOWER, UPPER, N_CELLS, EDGE, MU, STATIC_DT);

    // Rotate the top plate about the block centre (about the ML/x axis).
    let pivot = Point3::new(EDGE * 0.5, EDGE * 0.5, EDGE * 0.5);
    let axis = Vector3::x_axis();
    let angle_at = |i: usize| (i as f64 / HALF_FRAMES as f64) * MAX_ROT_DEG.to_radians();

    let mut block_frames = Vec::new();
    let mut upper_plate_frames = Vec::new();
    for i in (0..=HALF_FRAMES).chain((0..HALF_FRAMES).rev()) {
        let rot = UnitQuaternion::from_axis_angle(&axis, angle_at(i));
        let new_upper = pivot + rot * (rest_upper - pivot);
        sandwich.set_body_pose(UPPER, new_upper.coords, rot);
        sandwich.probe();
        block_frames.push(sandwich.soft_positions().to_vec());
        upper_plate_frames.push(plate_corners(new_upper, rot));
    }

    // The block's rest surface = frame 0 (top plate at 0°), reshaped for the mesh build.
    let block_rest: Vec<Vec3> = block_frames[0]
        .chunks_exact(3)
        .map(|c| Vec3::new(c[0], c[1], c[2]))
        .collect();
    let block_boundary = sandwich.soft_boundary_faces().to_vec();
    let lower_plate: Vec<Vec3> = plate_corners(rest_lower, UnitQuaternion::identity())
        .chunks_exact(3)
        .map(|c| Vec3::new(c[0], c[1], c[2]))
        .collect();

    println!(
        "bonded-sandwich: captured {} frames; block {} nodes, plate rotates ±{MAX_ROT_DEG}°",
        block_frames.len(),
        block_rest.len()
    );
    Scene {
        block_rest,
        block_boundary,
        block_frames,
        lower_plate,
        upper_plate_frames,
    }
}

/// Marks the deforming soft block.
#[derive(Component)]
struct SoftBlock;
/// Marks the rotating top plate.
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
    // Static lower plate.
    commands.spawn((
        Mesh3d(meshes.add(build_soft_mesh(
            &scene.lower_plate,
            &BOX_FACES,
            UpAxis::PlusZ,
        ))),
        MeshMaterial3d(plate_mat.clone()),
        Transform::default(),
    ));
    // Rotating upper plate (its rest corners = frame 0; animated per frame).
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

    // Frame the camera on the block + both plates.
    let mut pts: Vec<Point3<f64>> = scene.block_rest.iter().map(|v| Point3::from(*v)).collect();
    pts.extend(scene.lower_plate.iter().map(|v| Point3::from(*v)));
    pts.extend(upper_rest.iter().map(|v| Point3::from(*v)));
    setup_camera_and_lighting(&mut commands, &Aabb::from_points(pts.iter()), UpAxis::PlusZ);
}

/// Loop the block deformation + the top-plate rotation in lockstep.
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
