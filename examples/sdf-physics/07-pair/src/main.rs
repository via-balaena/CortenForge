//! SDF Physics 07 — Pair
//!
//! Two `Solid::sphere(5.0)` bodies. The lower one rests on the ground, the
//! upper one starts just above it and settles onto it. First test of
//! SDF-vs-SDF collision with surface-tracing multi-contact.
//!
//! Verifies:
//! - Upper body settles on top of the lower body (no tunneling)
//! - Both reach a stable stacked configuration
//! - Lower sphere rests on ground (SDF-plane, proven in step 04)
//! - Upper sphere rests on lower sphere (SDF-SDF surface-tracing)
//!
//! New concept: `sdf_sdf_contact` (surface-tracing with margin)
//! Depends on: 04-rest (SDF-plane works)
//!
//! Run with: `cargo run -p example-sdf-07-pair --release`

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
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_bevy::scene::ExampleScene;

const RADIUS: f64 = 5.0;

fn main() {
    let material = Material::new("PLA", 1250.0);

    let z_lower = RADIUS + 0.5;
    let z_upper = 3.0f64.mul_add(RADIUS, 0.5); // just above lower

    let mechanism = Mechanism::builder("pair")
        .part(Part::new("lower", Solid::sphere(RADIUS), material.clone()))
        .part(Part::new("upper", Solid::sphere(RADIUS), material))
        .joint(JointDef::new(
            "free_lower",
            "world",
            "lower",
            JointKind::Free,
            Point3::new(0.0, 0.0, z_lower),
            Vector3::z(),
        ))
        .joint(JointDef::new(
            "free_upper",
            "world",
            "upper",
            JointKind::Free,
            Point3::new(0.0, 0.0, z_upper),
            Vector3::z(),
        ))
        .build();

    let mut model = mechanism.to_model(1.0, 0.3);
    model.add_ground_plane();

    eprintln!();
    eprintln!("  Pair Diagnostics");
    eprintln!("  ----------------");
    eprintln!("  Bodies: {}, Geoms: {}", model.nbody, model.ngeom);
    eprintln!("  Timestep: {:.4} s", model.timestep);
    eprintln!(
        "  Lower z0: {:.2}, Upper z0: {:.2}",
        model.qpos0[2], model.qpos0[9]
    );
    eprintln!();

    let mut data = model.make_data();
    data.forward(&model).expect("forward kinematics");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "SDF Physics 07 — Pair".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(PhysicsModel(model))
        .insert_resource(PhysicsData(data))
        .insert_resource(PairTracker::default())
        .insert_resource(PhysicsAccumulator::default())
        .add_systems(Startup, setup)
        .add_systems(Update, (step_physics_realtime, track_pair))
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

    ExampleScene::new(50.0, 60.0)
        .with_target(Vec3::new(0.0, 8.0, 0.0))
        .with_angles(0.3, 0.4)
        .with_ground_y(-0.1)
        .spawn(&mut commands, &mut meshes, &mut materials);
}

#[derive(Resource, Default)]
struct PairTracker {
    last_print: f64,
    settled: bool,
}

fn track_pair(data: Res<PhysicsData>, mut tracker: ResMut<PairTracker>) {
    let t = data.0.time;
    // Debug: print full contact details at first timestep
    if tracker.last_print < 0.001 && t > 0.0 {
        for (i, c) in data.0.contacts.iter().take(3).enumerate() {
            eprintln!(
                "  [con {i}] g1={} g2={} depth={:.6} n=({:.3},{:.3},{:.3}) margin={:.4} dim={} solref=[{:.4},{:.4}] mu={:.4}",
                c.geom1,
                c.geom2,
                c.depth,
                c.normal.x,
                c.normal.y,
                c.normal.z,
                c.includemargin,
                c.dim,
                c.solref[0],
                c.solref[1],
                c.friction
            );
        }
    }
    let interval = if t < 3.0 { 0.5 } else { 2.0 };
    if t - tracker.last_print < interval || t > 15.0 {
        return;
    }
    tracker.last_print = t;

    let z_lower = data.0.qpos[2];
    let z_upper = data.0.qpos[9];
    let vz_lower = data.0.qvel[2];
    let vz_upper = data.0.qvel[8];
    let ncon = data.0.ncon;

    eprintln!(
        "  t={t:.1}s  z_lo={z_lower:.3}  z_up={z_upper:.3}  vz_lo={vz_lower:.3}  vz_up={vz_upper:.3}  con={ncon}"
    );

    if t >= 5.0 && !tracker.settled {
        tracker.settled = true;
        let gap = z_upper - z_lower;

        eprintln!();
        check(
            &format!("lower z ≈ {RADIUS:.0} (got {z_lower:.3})"),
            (z_lower - RADIUS).abs() < 1.0,
        );
        check(
            &format!("upper z ≈ {} (got {z_upper:.3})", 3.0 * RADIUS),
            3.0f64.mul_add(-RADIUS, z_upper).abs() < 2.0,
        );
        check(
            &format!("gap ≈ {} mm (got {gap:.3})", 2.0 * RADIUS),
            2.0f64.mul_add(-RADIUS, gap).abs() < 2.0,
        );
        check(
            &format!("|vz_lower| < 1.0 (got {vz_lower:.3})"),
            vz_lower.abs() < 1.0,
        );
        check(
            &format!("|vz_upper| < 1.0 (got {vz_upper:.3})"),
            vz_upper.abs() < 1.0,
        );
        check(&format!("contacts active ({ncon})"), ncon > 0);
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
