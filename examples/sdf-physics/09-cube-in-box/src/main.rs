//! SDF Physics 09 — Cube in Box
//!
//! A small convex cube dropped inside a concave open-top box. The box is
//! a cuboid with a smaller cuboid subtracted (first concave SDF shape).
//! The cube rattles around inside and settles at the bottom.
//!
//! New concept: concave SDF containment (walls constrain from the inside)
//! Depends on: 08-stack (multi-body SDF contact proven)
//!
//! Run with: `cargo run -p example-sdf-09-cube-in-box --release`

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
    // ── Box: cuboid with interior subtracted (open top) ────────────────
    // Outer: 30x30x30mm (half-extents 15). Inner void: 20x20mm footprint,
    // shifted up so floor is 5mm thick and top is open.
    let box_solid = Solid::cuboid(Vector3::new(15.0, 15.0, 15.0)).subtract(
        Solid::cuboid(Vector3::new(10.0, 10.0, 13.0)).translate(Vector3::new(0.0, 0.0, 3.0)),
    );

    let mut mat_box = Material::new("steel", 7800.0);
    mat_box.color = Some([0.4, 0.4, 0.5, 0.3]); // semi-transparent

    // ── Cube: small convex body dropped inside ─────────────────────────
    let cube_solid = Solid::cuboid(Vector3::new(4.0, 4.0, 4.0)); // 8x8x8mm

    let mut mat_cube = Material::new("PLA", 1250.0);
    mat_cube.color = Some([1.0, 0.35, 0.3, 1.0]); // red

    // ── Mechanism ──────────────────────────────────────────────────────
    // Box center at z=15 → bottom face at z=0 (ground).
    // Cube center at z=20 → inside box, 15mm above inner floor (z=5).
    let box_z = 15.0;
    let cube_z = 20.0;

    let mechanism = Mechanism::builder("cube_in_box")
        .part(Part::new("box", box_solid, mat_box))
        .part(Part::new("cube", cube_solid, mat_cube))
        .joint(JointDef::new(
            "box_free",
            "world",
            "box",
            JointKind::Free,
            Point3::new(0.0, 0.0, box_z),
            Vector3::z(),
        ))
        .joint(JointDef::new(
            "cube_free",
            "world",
            "cube",
            JointKind::Free,
            Point3::new(0.0, 0.0, cube_z),
            Vector3::z(),
        ))
        .build();

    let mut model = mechanism.to_model(1.0, 1.0);
    model.add_ground_plane();

    model.timestep = 0.002;
    model.solver_type = sim_core::SolverType::PGS;

    eprintln!();
    eprintln!("  Cube in Box — SDF Physics 09");
    eprintln!("  ──────────────────────────────");
    eprintln!("  Bodies: {}, Geoms: {}", model.nbody, model.ngeom);
    eprintln!("  Timestep: {:.4} s", model.timestep);
    eprintln!("  Box: 30x30x30mm outer, 20x20mm inner, 5mm walls, open top");
    eprintln!("  Cube: 8x8x8mm, dropped inside from z={cube_z}");
    eprintln!("  Inner floor at z=5, cube should settle at z≈9");
    eprintln!();

    let mut data = model.make_data();
    data.forward(&model).expect("forward kinematics");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "SDF Physics 09 — Cube in Box".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(PhysicsModel(model))
        .insert_resource(PhysicsData(data))
        .insert_resource(BoxTracker::default())
        .insert_resource(PhysicsAccumulator::default())
        .add_systems(Startup, setup)
        .add_systems(Update, (step_physics_realtime, track_box))
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
        .with_target(Vec3::new(0.0, 15.0, 0.0))
        .with_angles(0.4, 0.5)
        .with_ground_y(-0.1)
        .spawn(&mut commands, &mut meshes, &mut materials);
}

#[derive(Resource, Default)]
struct BoxTracker {
    last_print: f64,
    checks_done: bool,
}

fn track_box(data: Res<PhysicsData>, mut tracker: ResMut<BoxTracker>) {
    let t = data.0.time;
    let box_q = 0; // box qpos [0..7]
    let cube_q = 7; // cube qpos [7..14]

    let box_z = data.0.qpos[box_q + 2];
    let cube_z = data.0.qpos[cube_q + 2];
    let cube_x = data.0.qpos[cube_q];
    let cube_y = data.0.qpos[cube_q + 1];
    let cube_vz = data.0.qvel[8]; // cube linear vel Z (qvel[6+2])

    let interval = if t < 3.0 { 0.5 } else { 2.0 };
    if t - tracker.last_print < interval || t > 15.0 {
        return;
    }
    tracker.last_print = t;

    eprintln!(
        "  t={t:.1}s  box_z={box_z:.2}  cube=({cube_x:.2},{cube_y:.2},{cube_z:.2})  vz={cube_vz:.3}  ncon={}",
        data.0.ncon
    );

    if t >= 3.0 && !tracker.checks_done {
        tracker.checks_done = true;

        // Inner floor at z=5 (world). Cube half-extent=4. Settled cube center ≈ z=9.
        let expected_cube_z = 9.0;
        // Cube should stay inside box walls: |x|, |y| < 10 (inner half-width)
        let inside_x = cube_x.abs() < 10.0;
        let inside_y = cube_y.abs() < 10.0;

        eprintln!();
        eprintln!("  === PASS/FAIL checks at t={t:.1}s ===");

        check("all values finite", cube_z.is_finite() && box_z.is_finite());

        check(
            &format!("cube z ≈ {expected_cube_z:.0} (got {cube_z:.2})"),
            (cube_z - expected_cube_z).abs() < 3.0,
        );

        check(&format!("cube inside box x (|{cube_x:.2}| < 10)"), inside_x);

        check(&format!("cube inside box y (|{cube_y:.2}| < 10)"), inside_y);

        check(
            &format!("cube settled (|vz| < 1.0, got {:.3})", cube_vz.abs()),
            cube_vz.abs() < 1.0,
        );

        check(
            &format!("box on ground (z={box_z:.2}, expected ~15)"),
            (box_z - 15.0).abs() < 2.0,
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
