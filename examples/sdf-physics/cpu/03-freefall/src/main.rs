//! SDF Physics 03 — Freefall
//!
//! First physics step. A cf-design `Solid::sphere(5.0)` converted to a physics
//! body via `to_model()` with a free joint. No ground plane — the body
//! free-falls under gravity.
//!
//! Verifies:
//! - `to_model()` produces correct mass (compared to analytical: ρ × V)
//! - `to_model()` produces correct inertia (compared to analytical: 2/5 mr²)
//! - Gravity is correctly scaled for mm-world (9810 mm/s²)
//! - Position follows analytical freefall: `z(t) = z₀ − ½gt²`
//! - No unexpected rotation (angular velocity stays near zero)
//!
//! New concept: `to_model()` pipeline + mm-scale gravity
//! Depends on: 01-sdf-grid, 02-thin-grid
//!
//! Run with: `cargo run -p example-sdf-cpu-03-freefall --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::let_underscore_must_use,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation
)]

use bevy::prelude::*;
use cf_design::{JointDef, JointKind, Material, Mechanism, Part, Solid};
use nalgebra::{Point3, Vector3};
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::model_data::{
    PhysicsData, PhysicsModel, spawn_model_geoms, step_model_data, sync_geom_transforms,
};
use sim_bevy::scene::ExampleScene;

/// Sphere radius in mm — same shape as steps 01-02.
const RADIUS: f64 = 5.0;

/// PLA density in kg/m³.
const DENSITY: f64 = 1250.0;

/// Starting height of sphere center (mm).
const Z0: f64 = 50.0;

fn main() {
    let solid = Solid::sphere(RADIUS);
    let material = Material::new("PLA", DENSITY);

    let mechanism = Mechanism::builder("freefall")
        .part(Part::new("ball", solid, material))
        .joint(JointDef::new(
            "free",
            "world",
            "ball",
            JointKind::Free,
            Point3::new(0.0, 0.0, Z0),
            Vector3::z(),
        ))
        .build();

    let model = mechanism.to_model(1.0, 0.3);

    // ── Verify mass ─────────────────────────────────────────────────
    let volume_mm3 = (4.0 / 3.0) * std::f64::consts::PI * RADIUS.powi(3);
    let expected_mass = DENSITY * volume_mm3 * 1e-9; // kg/m³ × mm³ × mm³→m³
    let actual_mass = model.body_mass[1];
    let mass_err_pct = ((actual_mass - expected_mass) / expected_mass).abs() * 100.0;

    eprintln!();
    eprintln!("  Freefall Diagnostics");
    eprintln!("  --------------------");
    eprintln!(
        "  Mass:    {actual_mass:.6} kg  (analytical: {expected_mass:.6}, err: {mass_err_pct:.1}%)"
    );

    // ── Verify inertia ──────────────────────────────────────────────
    // I = 2/5 × m × r² for solid sphere. Inertia in kg·mm² (mm-scale positions).
    let expected_i = 0.4 * expected_mass * RADIUS * RADIUS;
    let actual_i = model.body_inertia[1];
    eprintln!(
        "  Inertia: ({:.6}, {:.6}, {:.6}) kg·mm²",
        actual_i.x, actual_i.y, actual_i.z
    );
    eprintln!("           (analytical: {expected_i:.6} for all axes)");

    // ── Verify gravity ──────────────────────────────────────────────
    let g = model.gravity;
    eprintln!("  Gravity: ({:.1}, {:.1}, {:.1}) mm/s²", g.x, g.y, g.z);
    eprintln!("  Timestep: {:.4} s", model.timestep);
    eprintln!();

    let mut pass = true;
    pass &= check(
        &format!("mass within 5% of analytical ({mass_err_pct:.1}%)"),
        mass_err_pct < 5.0,
    );
    pass &= check(
        "inertia diagonal ≈ equal (sphere symmetry)",
        (actual_i.x - actual_i.y).abs() / actual_i.x < 0.05
            && (actual_i.x - actual_i.z).abs() / actual_i.x < 0.05,
    );
    pass &= check(
        "gravity = -9810 mm/s² (mm-scale)",
        (g.z - (-9810.0)).abs() < 1.0,
    );

    eprintln!();
    if pass {
        eprintln!("  Static checks passed. Starting simulation...");
    } else {
        eprintln!("  STATIC CHECKS FAILED. Simulation will still run.");
    }
    eprintln!();

    let mut data = model.make_data();
    data.forward(&model).expect("forward kinematics");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "SDF Physics 03 — Freefall".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(PhysicsModel(model))
        .insert_resource(PhysicsData(data))
        .insert_resource(FreefallTracker::default())
        .add_systems(Startup, setup)
        .add_systems(Update, (step_model_data, track_freefall))
        .add_systems(PostUpdate, sync_geom_transforms)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
) {
    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model.0,
        &data.0,
    );

    ExampleScene::new(500.0, 200.0)
        .with_target(Vec3::new(0.0, -100.0, 0.0))
        .with_max_distance(5000.0)
        .with_angles(0.3, 0.5)
        .spawn(&mut commands, &mut meshes, &mut materials);
}

#[derive(Resource, Default)]
struct FreefallTracker {
    last_print: f64,
    max_error: f64,
    max_omega: f64,
    printed_summary: bool,
}

fn track_freefall(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut tracker: ResMut<FreefallTracker>,
) {
    let t = data.0.time;
    let g = -model.0.gravity.z;
    let h = model.0.timestep;
    let z_actual = data.0.qpos[2];
    // Semi-implicit Euler for constant acceleration: z = z₀ − ½g(t² + ht)
    // The extra −½ght term is the integrator's systematic error.
    let z_euler = (0.5 * g * t).mul_add(-(t + h), Z0);
    let error = (z_actual - z_euler).abs();
    tracker.max_error = tracker.max_error.max(error);

    // Angular velocity magnitude (qvel[3..6])
    let w = &data.0.qvel;
    let omega = w[5].mul_add(w[5], w[3].mul_add(w[3], w[4] * w[4])).sqrt();
    tracker.max_omega = tracker.max_omega.max(omega);

    // Print every 0.25s for first 2s
    if t - tracker.last_print >= 0.25 && t <= 2.0 {
        tracker.last_print = t;
        eprintln!(
            "  t={t:.2}s  z={z_actual:.2}  expected={z_euler:.2}  err={error:.4}  ω={omega:.6}"
        );
    }

    // Print summary once at t≈2s
    if t >= 2.0 && !tracker.printed_summary {
        tracker.printed_summary = true;
        eprintln!();
        check(
            &format!("trajectory max error = {:.4} mm over 2s", tracker.max_error),
            tracker.max_error < 1.0,
        );
        check(
            &format!(
                "max angular velocity = {:.6} rad/s (no rotation)",
                tracker.max_omega
            ),
            tracker.max_omega < 0.001,
        );
        eprintln!();
    }
}

fn check(label: &str, ok: bool) -> bool {
    if ok {
        eprintln!("  PASS: {label}");
    } else {
        eprintln!("  FAIL: {label}");
    }
    ok
}
