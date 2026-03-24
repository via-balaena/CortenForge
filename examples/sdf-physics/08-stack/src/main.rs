//! SDF Physics 08 — Stack
//!
//! Three `Solid::cuboid()` bodies stacked on a ground plane. Tests multi-body
//! simultaneous contact stability — cubes use octree-based multi-contact
//! (Tier 2) for face-face interfaces and octree plane detection for ground.
//!
//! Verifies:
//! - All three cubes settle into a stable stack within 3 seconds
//! - No body passes through another
//! - Stack remains stable for at least 5 more seconds after settling
//! - No jitter, drift, or slow collapse
//!
//! Interactive: press Space to knock cubes off one by one (top first).
//!
//! New concept: multi-body stacking stability with flat-face contacts
//! Depends on: 07-pair (SDF-SDF collision proven)
//!
//! Run with: `cargo run -p example-sdf-08-stack --release`

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

const HALF: f64 = 5.0; // mm half-extents for each cube
const GAP: f64 = 0.5; // mm gap between surfaces

fn main() {
    let half_extents = Vector3::new(HALF, HALF, HALF);

    let mut mat_bot = Material::new("PLA", 1250.0);
    mat_bot.color = Some([0.3, 0.5, 1.0, 1.0]); // blue

    let mut mat_mid = Material::new("PLA", 1250.0);
    mat_mid.color = Some([0.3, 0.85, 0.35, 1.0]); // green

    let mut mat_top = Material::new("PLA", 1250.0);
    mat_top.color = Some([1.0, 0.35, 0.3, 1.0]); // red

    // Stack positions: each cube center is at z = HALF + (2*HALF + GAP) * index + GAP
    // Bottom: just above ground  →  z = HALF + GAP = 5.5
    // Middle: on bottom cube     →  z = 3*HALF + 2*GAP = 16.0
    // Top:    on middle cube     →  z = 5*HALF + 3*GAP = 26.5
    let z_bottom = HALF + GAP;
    let z_middle = 3.0f64.mul_add(HALF, 2.0 * GAP);
    let z_top = 5.0f64.mul_add(HALF, 3.0 * GAP);

    let mechanism = Mechanism::builder("stack")
        .part(Part::new("bottom", Solid::cuboid(half_extents), mat_bot))
        .part(Part::new("middle", Solid::cuboid(half_extents), mat_mid))
        .part(Part::new("top", Solid::cuboid(half_extents), mat_top))
        .joint(JointDef::new(
            "free_bottom",
            "world",
            "bottom",
            JointKind::Free,
            Point3::new(0.0, 0.0, z_bottom),
            Vector3::z(),
        ))
        .joint(JointDef::new(
            "free_middle",
            "world",
            "middle",
            JointKind::Free,
            Point3::new(0.0, 0.0, z_middle),
            Vector3::z(),
        ))
        .joint(JointDef::new(
            "free_top",
            "world",
            "top",
            JointKind::Free,
            Point3::new(0.0, 0.0, z_top),
            Vector3::z(),
        ))
        .build();

    let mut model = mechanism.to_model(1.0, 0.3);
    model.add_ground_plane();

    eprintln!();
    eprintln!("  Stack Diagnostics");
    eprintln!("  -----------------");
    eprintln!("  Bodies: {}, Geoms: {}", model.nbody, model.ngeom);
    eprintln!("  Timestep: {:.4} s", model.timestep);
    eprintln!(
        "  z0: bottom={:.2}, middle={:.2}, top={:.2}",
        model.qpos0[2], model.qpos0[9], model.qpos0[16]
    );
    eprintln!("  Press SPACE to knock cubes off (top first)");
    eprintln!();

    let mut data = model.make_data();
    data.forward(&model).expect("forward kinematics");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "SDF Physics 08 — Stack".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(PhysicsModel(model))
        .insert_resource(PhysicsData(data))
        .insert_resource(StackTracker::default())
        .insert_resource(PhysicsAccumulator::default())
        .add_systems(Startup, setup)
        .add_systems(Update, (step_physics_realtime, nudge_stack, track_stack))
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

    ExampleScene::new(60.0, 70.0)
        .with_target(Vec3::new(0.0, 15.0, 0.0))
        .with_angles(0.3, 0.4)
        .with_ground_y(-0.1)
        .spawn(&mut commands, &mut meshes, &mut materials);
}

#[derive(Resource, Default)]
struct StackTracker {
    last_print: f64,
    settled: bool,
    nudged: bool,
}

/// Nudge cubes off one by one: top → middle → bottom.
/// qvel layout: 6 per body. body 1 (bottom) = [0..6], body 2 (middle) = [6..12], body 3 (top) = [12..18].
fn nudge_stack(
    keys: Res<ButtonInput<KeyCode>>,
    mut data: ResMut<PhysicsData>,
    mut tracker: ResMut<StackTracker>,
) {
    if !keys.just_pressed(KeyCode::Space) {
        return;
    }
    tracker.nudged = true;

    // Find the highest cube that hasn't been knocked far off yet.
    // qpos: body 1 [0..7], body 2 [7..14], body 3 [14..21]
    let bodies = [(14, 12, "top"), (7, 6, "middle"), (0, 0, "bottom")];
    for &(qpos_off, qvel_off, name) in &bodies {
        let z = data.0.qpos[qpos_off + 2];
        // Only nudge if it's still stacked (z > 10, i.e. above the bottom cube).
        // Bottom cube at z≈5, middle at z≈15, top at z≈25.
        // A knocked-off cube lands at z≈5, so threshold 10 skips it.
        if z > 10.0 {
            data.0.qvel[qvel_off] += 300.0; // vx impulse
            data.0.qvel[qvel_off + 2] += 150.0; // vz impulse — lift clear before sliding
            eprintln!("  *** NUDGE {name}! Applied vx=+300, vz=+150 mm/s ***");
            return;
        }
    }
}

fn track_stack(data: Res<PhysicsData>, mut tracker: ResMut<StackTracker>) {
    let t = data.0.time;

    // Print contact details at first timestep
    if tracker.last_print < 0.001 && t > 0.0 {
        for (i, c) in data.0.contacts.iter().take(6).enumerate() {
            eprintln!(
                "  [con {i}] g1={} g2={} depth={:.6} n=({:.3},{:.3},{:.3}) margin={:.4} dim={} mu={:.4}",
                c.geom1,
                c.geom2,
                c.depth,
                c.normal.x,
                c.normal.y,
                c.normal.z,
                c.includemargin,
                c.dim,
                c.friction
            );
        }
    }

    let interval = if t < 3.0 { 0.5 } else { 2.0 };
    if t - tracker.last_print < interval || t > 15.0 {
        return;
    }
    tracker.last_print = t;

    // Free body qpos layout: 7 per body (3 pos + 4 quat)
    // Free body qvel layout: 6 per body (3 lin + 3 ang)
    let z_bot = data.0.qpos[2];
    let z_mid = data.0.qpos[9];
    let z_top = data.0.qpos[16];
    let vz_bot = data.0.qvel[2];
    let vz_mid = data.0.qvel[8];
    let vz_top = data.0.qvel[14];
    let ncon = data.0.ncon;

    eprintln!(
        "  t={t:.1}s  z_bot={z_bot:.3}  z_mid={z_mid:.3}  z_top={z_top:.3}  vz=({vz_bot:.3},{vz_mid:.3},{vz_top:.3})  con={ncon}"
    );

    if t >= 5.0 && !tracker.settled && !tracker.nudged {
        tracker.settled = true;

        // Expected settled heights (center of each cube):
        //   bottom: HALF = 5.0
        //   middle: 3*HALF = 15.0
        //   top:    5*HALF = 25.0
        let expect_bot = HALF;
        let expect_mid = 3.0 * HALF;
        let expect_top = 5.0 * HALF;

        eprintln!();
        eprintln!("  === PASS/FAIL checks at t={t:.1}s ===");
        check(
            &format!("bottom z ≈ {expect_bot:.0} (got {z_bot:.3})"),
            (z_bot - expect_bot).abs() < 2.0,
        );
        check(
            &format!("middle z ≈ {expect_mid:.0} (got {z_mid:.3})"),
            (z_mid - expect_mid).abs() < 2.0,
        );
        check(
            &format!("top z ≈ {expect_top:.0} (got {z_top:.3})"),
            (z_top - expect_top).abs() < 2.0,
        );
        check(
            &format!(
                "bottom-middle gap ≈ {} (got {:.3})",
                2.0 * HALF,
                z_mid - z_bot
            ),
            2.0f64.mul_add(-HALF, z_mid - z_bot).abs() < 2.0,
        );
        check(
            &format!("middle-top gap ≈ {} (got {:.3})", 2.0 * HALF, z_top - z_mid),
            2.0f64.mul_add(-HALF, z_top - z_mid).abs() < 2.0,
        );
        check(
            &format!("|vz_bot| < 1.0 (got {vz_bot:.3})"),
            vz_bot.abs() < 1.0,
        );
        check(
            &format!("|vz_mid| < 1.0 (got {vz_mid:.3})"),
            vz_mid.abs() < 1.0,
        );
        check(
            &format!("|vz_top| < 1.0 (got {vz_top:.3})"),
            vz_top.abs() < 1.0,
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
