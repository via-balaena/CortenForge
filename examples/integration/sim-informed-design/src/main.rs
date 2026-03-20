//! Sim-Informed Design — Stress-Graded Lattice from Simulation Forces
//!
//! Demonstrates the feedback loop: simulate → analyze → redesign.
//! 1. Design a bracket
//! 2. Convert to MJCF and simulate under load
//! 3. Extract body forces + contact data → build stress field
//! 4. Generate stress-graded, shape-conforming lattice
//! 5. Visualize: bracket with force indicators + graded lattice side-by-side
//!
//! Run with: `cargo run -p example-sim-informed-design --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use
)]

use std::sync::Arc;

use bevy::prelude::*;
use cf_design::{Material, Mechanism, Part, Solid};
use cf_geometry::IndexedMesh;
use mesh_lattice::{DensityMap, LatticeParams, generate_lattice};
use mesh_measure::dimensions;
use mesh_repair::{RepairParams, repair_mesh};
use nalgebra::{Point3, Vector3};
use sim_bevy::camera::{OrbitCamera, OrbitCameraPlugin};
use sim_bevy::mesh::triangle_mesh_from_indexed;
use sim_core::{Data, Model};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Sim-Informed Design".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .add_systems(Startup, setup)
        .run();
}

/// Build a simple bracket as a one-part mechanism.
fn build_bracket() -> (Mechanism, Solid) {
    let material = Material::new("PLA", 1250.0)
        .with_youngs_modulus(3.5e9)
        .with_color([0.6, 0.7, 0.8, 1.0]);

    let solid = Solid::cuboid(Vector3::new(25.0, 15.0, 10.0))
        .round(2.0)
        .subtract(Solid::cylinder(3.0, 12.0).translate(Vector3::new(8.0, 0.0, 0.0)))
        .subtract(Solid::cylinder(3.0, 12.0).translate(Vector3::new(-8.0, 0.0, 0.0)));

    let part = Part::new("bracket", solid.clone(), material);

    let mechanism = Mechanism::builder("bracket_test").part(part).build();

    (mechanism, solid)
}

/// Build a gaussian-weighted stress field from body-level force data.
///
/// Uses `data.cfrc_ext` (spatial force on each body from contacts + external)
/// and contact positions as stress sources. This is a simplified stress
/// proxy — not FEA — but sufficient for density-grading lattice infill.
fn build_stress_field(
    data: &Data,
    model: &Model,
    sigma: f64,
) -> Arc<dyn Fn(Point3<f64>) -> f64 + Send + Sync> {
    let mut stress_sources: Vec<(Point3<f64>, f64)> = Vec::new();

    // Body-level forces from cfrc_ext [torque(3), force(3)]
    for body_id in 1..model.nbody {
        let force = &data.cfrc_ext[body_id];
        let linear_mag = Vector3::new(force[3], force[4], force[5]).norm();
        if linear_mag > 1e-6 {
            let pos = &data.xpos[body_id];
            stress_sources.push((Point3::new(pos.x, pos.y, pos.z), linear_mag));
        }
    }

    // Contact positions (with penetration depth as magnitude proxy)
    for contact in &data.contacts {
        if contact.depth > 1e-8 {
            let pos = contact.pos;
            stress_sources.push((Point3::new(pos.x, pos.y, pos.z), contact.depth * 1000.0));
        }
    }

    println!(
        "  Stress sources: {} body forces, {} contacts",
        stress_sources.iter().filter(|(_, m)| *m > 1e-6).count(),
        data.contacts.len()
    );

    if !stress_sources.is_empty() {
        let max_stress = stress_sources
            .iter()
            .map(|(_, m)| *m)
            .fold(0.0_f64, f64::max);
        let min_stress = stress_sources
            .iter()
            .map(|(_, m)| *m)
            .fold(f64::MAX, f64::min);
        let count = stress_sources.len();
        let mean_stress: f64 =
            stress_sources.iter().map(|(_, m)| m).sum::<f64>() / f64::from(count as u32);
        println!("  Stress range: min={min_stress:.3}, max={max_stress:.3}, mean={mean_stress:.3}");
    }

    let inv_2sigma2 = 1.0 / (2.0 * sigma * sigma);

    Arc::new(move |p: Point3<f64>| -> f64 {
        stress_sources
            .iter()
            .map(|(src, mag)| {
                let dist2 = (p - src).norm_squared();
                mag * (-dist2 * inv_2sigma2).exp()
            })
            .sum()
    })
}

/// Run the simulation and build the lattice pipeline.
fn run_pipeline() -> (IndexedMesh, IndexedMesh, Vec<(Vec3, f32)>, f32) {
    // ── Design ───────────────────────────────────────────────────────
    println!("Stage 1: Design bracket");
    let (mechanism, solid) = build_bracket();

    let mut bracket_mesh = solid.mesh(0.5);
    let _ = repair_mesh(&mut bracket_mesh, &RepairParams::default());

    let dims = dimensions(&bracket_mesh);
    println!(
        "  Dimensions: {:.1} x {:.1} x {:.1} mm",
        dims.width, dims.depth, dims.height
    );

    // ── Simulate ─────────────────────────────────────────────────────
    println!("\nStage 2: Simulate under load");
    let mjcf_xml = mechanism.to_mjcf(1.5);
    let model = sim_mjcf::load_model(&mjcf_xml).expect("MJCF should load");

    println!("  Model: {} bodies, {} geoms", model.nbody, model.ngeom);

    let mut data = model.make_data();

    // Apply a downward force + torque to the bracket body to simulate loading
    if model.nbody > 1 {
        // Spatial force: [torque_x, torque_y, torque_z, force_x, force_y, force_z]
        data.xfrc_applied[1] = nalgebra::Vector6::new(0.0, 5.0, 0.0, 0.0, 0.0, -20.0);
    }

    // Run forward kinematics + constraint solver to populate cfrc_ext
    let _ = data.forward(&model);

    // Step a few times to let forces propagate
    let n_steps = 50;
    for _ in 0..n_steps {
        if model.nbody > 1 {
            data.xfrc_applied[1] = nalgebra::Vector6::new(0.0, 5.0, 0.0, 0.0, 0.0, -20.0);
        }
        let _ = data.step(&model);
    }
    let _ = data.forward(&model);
    println!("  Ran {n_steps} steps");

    // ── Extract stress field ─────────────────────────────────────────
    println!("\nStage 3: Extract stress field");
    let stress_fn = build_stress_field(&data, &model, 8.0);

    // Sample the stress field to compute statistics for console output
    let n_samples: u32 = 1000;
    let mut stress_min = f64::MAX;
    let mut stress_max = 0.0_f64;
    let mut stress_sum = 0.0_f64;
    for i in 0..n_samples {
        let t = f64::from(i) / f64::from(n_samples);
        let p = Point3::new(
            t.mul_add(dims.width, dims.min.x),
            (t * 7.0).sin().abs().mul_add(dims.depth, dims.min.y),
            (t * 13.0).sin().abs().mul_add(dims.height, dims.min.z),
        );
        let s = stress_fn(p);
        stress_min = stress_min.min(s);
        stress_max = stress_max.max(s);
        stress_sum += s;
    }
    println!(
        "  Sampled stress: min={:.4}, max={:.4}, mean={:.4}",
        stress_min,
        stress_max,
        stress_sum / f64::from(n_samples)
    );

    // ── Stress-graded lattice ────────────────────────────────────────
    println!("\nStage 4: Generate stress-graded lattice");
    let stress_fn_clone = stress_fn.clone();
    let density_map = DensityMap::from_stress_field(move |p| stress_fn_clone(p), 0.15, 0.6);

    let shape_sdf = Arc::new(move |p: Point3<f64>| solid.evaluate(&p));

    let lattice_params = LatticeParams::gyroid(5.0)
        .with_density_map(density_map)
        .with_shape_sdf(shape_sdf);
    let bounds = (dims.min, dims.max);
    let lattice = generate_lattice(&lattice_params, bounds).expect("lattice generation");

    println!(
        "  Lattice: {} vertices, {} faces, density={:.2}",
        lattice.vertex_count(),
        lattice.triangle_count(),
        lattice.actual_density
    );

    // ── Collect force indicators for visualization ───────────────────
    let mut force_indicators = Vec::new();
    for body_id in 1..model.nbody {
        let force = &data.cfrc_ext[body_id];
        let linear_mag = Vector3::new(force[3], force[4], force[5]).norm();
        if linear_mag > 1e-6 {
            let pos = &data.xpos[body_id];
            // Convert Z-up to Y-up for Bevy
            force_indicators.push((
                Vec3::new(pos.x as f32, pos.z as f32, -pos.y as f32),
                linear_mag as f32,
            ));
        }
    }

    let spacing = dims.width as f32 + 15.0;
    (bracket_mesh, lattice.mesh, force_indicators, spacing)
}

/// Spawn a mesh entity at a given X offset with the given color.
fn spawn_mesh(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    mesh_data: &IndexedMesh,
    color: Color,
    x_offset: f32,
    label: &str,
) {
    let indexed = Arc::new(mesh_data.clone());
    let bevy_mesh = triangle_mesh_from_indexed(&indexed);

    println!(
        "  {label}: {} vertices, {} faces",
        indexed.vertices.len(),
        indexed.faces.len()
    );

    commands.spawn((
        Mesh3d(meshes.add(bevy_mesh)),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: color,
            metallic: 0.3,
            perceptual_roughness: 0.5,
            double_sided: true,
            cull_mode: None,
            ..default()
        })),
        Transform::from_xyz(x_offset, 0.0, 0.0),
    ));
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    println!("=== CortenForge: Sim-Informed Design ===\n");

    let (bracket_mesh, lattice_mesh, force_indicators, spacing) = run_pipeline();

    // ── Panel 1: Bracket with force indicators ───────────────────────
    println!("\nSpawning visualization:");

    spawn_mesh(
        &mut commands,
        &mut meshes,
        &mut materials,
        &bracket_mesh,
        Color::srgb(0.6, 0.7, 0.85),
        -spacing / 2.0,
        "Bracket (simulated)",
    );

    // Force indicators as colored spheres
    let max_force = force_indicators
        .iter()
        .map(|(_, m)| *m)
        .fold(0.0_f32, f32::max)
        .max(1.0);

    for (pos, mag) in &force_indicators {
        let intensity = *mag / max_force;
        let radius = 1.0 + intensity * 3.0;
        commands.spawn((
            Mesh3d(meshes.add(Sphere::new(radius))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(1.0, 0.2 * (1.0 - intensity), 0.1),
                emissive: LinearRgba::new(intensity * 2.0, 0.0, 0.0, 1.0),
                ..default()
            })),
            Transform::from_translation(*pos + Vec3::new(-spacing / 2.0, 0.0, 0.0)),
        ));
    }

    // ── Panel 2: Stress-graded lattice ───────────────────────────────
    spawn_mesh(
        &mut commands,
        &mut meshes,
        &mut materials,
        &lattice_mesh,
        Color::srgb(0.3, 0.8, 0.4),
        spacing / 2.0,
        "Stress-graded lattice",
    );

    // ── Camera ───────────────────────────────────────────────────────
    let mut orbit = OrbitCamera::new()
        .with_target(Vec3::ZERO)
        .with_angles(0.5, 0.5);
    orbit.max_distance = 500.0;
    orbit.min_distance = 1.0;
    orbit.orbit_speed = 0.008;
    orbit.pan_speed = 0.015;
    orbit.zoom_speed = 0.15;
    orbit.distance = 120.0;
    let mut cam_transform = Transform::default();
    orbit.apply_to_transform(&mut cam_transform);
    commands.spawn((Camera3d::default(), orbit, cam_transform));

    // ── Lighting ─────────────────────────────────────────────────────
    commands.spawn((
        DirectionalLight {
            illuminance: 12000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(30.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 4000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(-20.0, 30.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Ground
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(80.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.5, 0.5, 0.5, 0.3),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, -20.0, 0.0),
    ));

    println!("\n  Orbit: left-drag | Pan: right-drag | Zoom: scroll");
}
