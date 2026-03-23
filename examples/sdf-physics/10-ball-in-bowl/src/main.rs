//! SDF Physics 10 — Ball in Bowl
//!
//! A small ball rolling inside a concave spherical bowl (cuboid − sphere).
//! The ball starts high on the inner wall and rolls down to settle at the
//! bottom. Proves curved concave containment — the bridge from flat walls
//! (09) to cylindrical bores (11).
//!
//! New concept: curved concave SDF surface (spherical cavity)
//! Depends on: 09-cube-in-box (concave containment with flat walls proven)
//!
//! Run with: `cargo run -p example-sdf-10-ball-in-bowl --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::let_underscore_must_use,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::similar_names
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

fn main() {
    // ── Bowl: cuboid with spherical cavity ─────────────────────────────
    // Outer block: 30x30x20mm. Spherical cavity R=12mm centered at z=+5
    // in local frame → 3mm floor, open top, 3mm minimum wall at equator.
    let bowl_solid = Solid::cuboid(Vector3::new(15.0, 15.0, 10.0))
        .subtract(Solid::sphere(12.0).translate(Vector3::new(0.0, 0.0, 5.0)));

    let mut mat_bowl = Material::new("steel", 7800.0);
    mat_bowl.color = Some([0.4, 0.4, 0.5, 0.3]); // semi-transparent

    // ── Ball: small convex sphere ──────────────────────────────────────
    let ball_solid = Solid::sphere(3.0); // R=3mm

    let mut mat_ball = Material::new("PLA", 1250.0);
    mat_ball.color = Some([1.0, 0.35, 0.3, 1.0]); // red

    // ── Mechanism ──────────────────────────────────────────────────────
    // Bowl center at z=10 → bottom face at z=0 (ground).
    // Cavity center in world: (0, 0, 15). Cavity bottom: z=3.
    //
    // Ball starts high on the inside wall, ~touching the surface.
    // Cavity R=12, ball R=3 → ball center at distance 9 from cavity center.
    // At ~60deg from vertical: x=8, z=11 → dist from (0,0,15) ≈ 8.9 ≈ 9.
    let bowl_z = 10.0;
    let ball_x = 8.0;
    let ball_z = 11.0;

    let mechanism = Mechanism::builder("ball_in_bowl")
        .part(Part::new("bowl", bowl_solid, mat_bowl))
        .part(Part::new("ball", ball_solid, mat_ball))
        .joint(JointDef::new(
            "bowl_free",
            "world",
            "bowl",
            JointKind::Free,
            Point3::new(0.0, 0.0, bowl_z),
            Vector3::z(),
        ))
        .joint(JointDef::new(
            "ball_free",
            "world",
            "ball",
            JointKind::Free,
            Point3::new(ball_x, 0.0, ball_z),
            Vector3::z(),
        ))
        .build();

    let mut model = mechanism.to_model(1.0, 1.0);
    model.add_ground_plane();

    model.timestep = 0.002;
    model.solver_type = sim_core::SolverType::PGS;

    // Ball settled position: cavity bottom z=3, ball R=3 → center z≈6
    eprintln!();
    eprintln!("  Ball in Bowl — SDF Physics 10");
    eprintln!("  ──────────────────────────────");
    eprintln!("  Bodies: {}, Geoms: {}", model.nbody, model.ngeom);
    eprintln!("  Timestep: {:.4} s", model.timestep);
    eprintln!("  Bowl: 30x30x20mm block, spherical cavity R=12mm");
    eprintln!("  Ball: R=3mm, starts at ({ball_x}, 0, {ball_z}) on inner wall");
    eprintln!("  Should roll down and settle at ~(0, 0, 6)");
    eprintln!();

    let mut data = model.make_data();
    data.forward(&model).expect("forward kinematics");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "SDF Physics 10 — Ball in Bowl".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(PhysicsModel(model))
        .insert_resource(PhysicsData(data))
        .insert_resource(BowlTracker::default())
        .insert_resource(PhysicsAccumulator::default())
        .add_systems(Startup, setup)
        .add_systems(Update, (step_physics_realtime, track_bowl))
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

    ExampleScene::new(80.0, 70.0)
        .with_target(Vec3::new(0.0, 10.0, 0.0))
        .with_angles(0.4, 0.5)
        .with_ground_y(-0.1)
        .spawn(&mut commands, &mut meshes, &mut materials);
}

#[derive(Resource, Default)]
struct BowlTracker {
    last_print: f64,
    checks_done: bool,
}

fn track_bowl(data: Res<PhysicsData>, mut tracker: ResMut<BowlTracker>) {
    let t = data.0.time;
    let bowl_q = 0;
    let ball_q = 7;

    let bowl_z = data.0.qpos[bowl_q + 2];
    let ball_x = data.0.qpos[ball_q];
    let ball_y = data.0.qpos[ball_q + 1];
    let ball_z = data.0.qpos[ball_q + 2];
    let ball_vz = data.0.qvel[8]; // ball linear vel Z

    let interval = if t < 3.0 { 0.5 } else { 2.0 };
    if t - tracker.last_print < interval || t > 15.0 {
        return;
    }
    tracker.last_print = t;

    let radial = ball_x.hypot(ball_y); // lateral distance from bowl center
    eprintln!(
        "  t={t:.1}s  ball=({ball_x:.2},{ball_y:.2},{ball_z:.2})  r={radial:.2}  vz={ball_vz:.3}  ncon={}",
        data.0.ncon
    );

    if t >= 3.0 && !tracker.checks_done {
        tracker.checks_done = true;

        // Ball should settle near the bottom of the cavity.
        // Cavity bottom z=3 (world), ball R=3 → center z≈6.
        let expected_z = 6.0;

        eprintln!();
        eprintln!("  === PASS/FAIL checks at t={t:.1}s ===");

        check(
            "all values finite",
            ball_z.is_finite() && bowl_z.is_finite(),
        );

        check(
            &format!("ball z ≈ {expected_z:.0} (got {ball_z:.2})"),
            (ball_z - expected_z).abs() < 3.0,
        );

        check(
            &format!("ball near center (r={radial:.2} < 5)"),
            radial < 5.0,
        );

        check(
            &format!("ball settled (|vz| < 1.0, got {:.3})", ball_vz.abs()),
            ball_vz.abs() < 1.0,
        );

        check(
            &format!("bowl on ground (z={bowl_z:.2}, expected ~10)"),
            (bowl_z - 10.0).abs() < 2.0,
        );

        check(
            &format!("contacts active (ncon={})", data.0.ncon),
            data.0.ncon > 0,
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
