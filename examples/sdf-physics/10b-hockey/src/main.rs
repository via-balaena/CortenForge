//! SDF Physics 10b — Hockey
//!
//! A hockey stick on a revolute hinge swings to hit a puck across a
//! low-friction ice surface toward a goal. First example of actuated
//! dynamic impact — a user-controlled body strikes a free body,
//! transferring momentum via SDF-SDF contact.
//!
//! New concept: hinged body → free body momentum transfer (actuated impact)
//! Depends on: 10-ball-in-bowl (curved concave SDF contact proven)
//!
//! Press SPACE to swing the stick.
//!
//! Run with: `cargo run -p example-sdf-10b-hockey --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::let_underscore_must_use,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::similar_names,
    clippy::too_many_lines
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
    // ── Puck: flat disc on ice ───────────────────────────────────────
    let puck_solid = Solid::cylinder(12.0, 2.0); // R=12mm, half_h=2mm → 24mm diameter, 4mm thick

    let mut mat_puck = Material::new("PLA", 1250.0);
    mat_puck.color = Some([0.15, 0.15, 0.15, 1.0]); // black

    // ── Stick: angled shaft + blade ──────────────────────────────────
    //
    // Body-local coordinates. Handle at origin (0,0,0) = pivot point.
    // Shaft angles gently down from handle to blade: (0,0,0) → (0, -55, -15).
    // In world: handle at z=18 (grip height), blade at z=18-15=3 (ice level).
    //
    // The revolute joint rotates about Z (vertical), so each point's Z
    // coordinate stays constant. The blade sweeps at z=3 (puck height)
    // while the handle stays at z=18.
    //
    // Angle from horizontal: atan(15/55) ≈ 15° — looks like a hockey stick.
    // Horizontal reach: 55mm (the Y component of the shaft).
    let shaft = Solid::pipe(
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, -55.0, -15.0)],
        2.5, // R=2.5mm
    );

    // Blade: hockey blade perpendicular to shaft.
    // Long along X (30mm, perpendicular to shaft — the face that sweeps).
    // Short along Y (4mm, shaft direction = blade depth).
    // Low Z (5mm, just tall enough to hit the 4mm puck).
    // Bottom of blade at body z=-17.5 → world z=0.5 (just above ice).
    // Offset +13 in X so blade extends to one side of the shaft (L-shape, not T).
    let blade =
        Solid::cuboid(Vector3::new(15.0, 2.0, 2.5)).translate(Vector3::new(13.0, -57.0, -15.5));

    let stick_solid = shaft.smooth_union(blade, 0.5);

    let mut mat_stick = Material::new("PLA", 1250.0);
    mat_stick.color = Some([0.95, 0.95, 0.95, 1.0]); // white

    // ── Goal: open-front frame ───────────────────────────────────────
    // Opening faces +X (toward the puck). Open front, no floor.
    // Outer: 24×50×20mm. Inner: 24×46×20mm shifted +4X.
    // Result: 4mm back wall (-X), 2mm side walls (±Y), 2mm crossbar at top,
    //         open front (+X), open bottom (no floor — sits on ice).
    let goal_solid = Solid::cuboid(Vector3::new(12.0, 25.0, 10.0)).subtract(
        Solid::cuboid(Vector3::new(12.0, 23.0, 10.0)).translate(Vector3::new(4.0, 0.0, -2.0)),
    );

    let mut mat_goal = Material::new("steel", 7800.0);
    mat_goal.color = Some([0.8, 0.1, 0.1, 0.5]); // semi-transparent red

    // ── Layout ───────────────────────────────────────────────────────
    //
    // Coordinate system: Z up, XY is the ice surface.
    // Physics (x,y,z) → Bevy (x,z,y).
    //
    // Pivot at (0, 59, 18): handle at z=18, directly behind puck.
    // Blade center at body (0, -59, -8) → world (0, 0, 10) at θ=0.
    // Blade bottom at world z=0 (ice), top at z=20. Puck at z=[0,4] → overlap.
    //
    // Blade velocity at contact = ω × r, where r = (0, -59).
    // For ω < 0 (clockwise from above): v = (-|ω|·59, 0, 0).
    // Pure -X velocity → straight at the goal!

    let puck_z = 2.0;
    let pivot = Point3::new(0.0, 59.0, 18.0);
    let goal_pos = Point3::new(-50.0, 0.0, 10.0);

    // Layout (top-down, Z up / out of page):
    //
    //              +Y
    //               ^
    //               |
    //               * pivot (0, 55, 18)  [grip height]
    //              /|
    //            /  |  ~15° shaft
    //          /    |
    //    GOAL  (o)  ·  puck (0, 0, 2)   [blade at ice level]
    //  [-50,0]      |
    //               +----------> +X

    let mechanism = Mechanism::builder("hockey")
        .part(Part::new("puck", puck_solid, mat_puck))
        .part(
            Part::new("stick", stick_solid, mat_stick)
                .with_joint_origin(Vector3::new(0.0, 0.0, 0.0)),
        )
        .part(Part::new("goal", goal_solid, mat_goal))
        .joint(JointDef::new(
            "puck_free",
            "world",
            "puck",
            JointKind::Free,
            Point3::new(0.0, 0.0, puck_z),
            Vector3::z(),
        ))
        .joint(JointDef::new(
            "stick_hinge",
            "world",
            "stick",
            JointKind::Revolute,
            pivot,
            Vector3::z(), // vertical rotation axis
        ))
        .joint(JointDef::new(
            "goal_free",
            "world",
            "goal",
            JointKind::Free,
            goal_pos,
            Vector3::z(),
        ))
        .build();

    let mut model = mechanism.to_model(1.0, 0.3);
    model.add_ground_plane();

    model.timestep = 0.002; // 500 Hz
    model.solver_type = sim_core::SolverType::PGS;

    // Enable GPU-accelerated SDF collision (compute shaders).
    // Falls back to CPU transparently if no GPU available.
    match sim_gpu::enable_gpu_collision(&mut model) {
        Ok(()) => eprintln!("  GPU SDF collision: ENABLED"),
        Err(e) => eprintln!("  GPU SDF collision: disabled ({e})"),
    }

    // Low friction everywhere — ice-like surface.
    for i in 0..model.ngeom {
        model.geom_friction[i] = Vector3::new(0.05, 0.001, 0.0001);
    }

    // qpos layout: puck [0..7], stick [7], goal [8..15]
    // qvel layout: puck [0..6], stick [6], goal [7..13]
    let stick_q = 7;
    let stick_dof = 6;

    eprintln!();
    eprintln!("  Hockey — SDF Physics 10b");
    eprintln!("  ────────────────────────");
    eprintln!("  Bodies: {}, Geoms: {}", model.nbody, model.ngeom);
    eprintln!("  Timestep: {:.4} s", model.timestep);
    eprintln!("  Puck: 24mm diameter, 4mm thick, at (0, 0, {puck_z})");
    eprintln!(
        "  Stick: handle at ({}, {}, {}), blade at ice level",
        pivot.x, pivot.y, pivot.z
    );
    eprintln!(
        "  Goal: at ({}, {}, {}), open on +X face",
        goal_pos.x, goal_pos.y, goal_pos.z
    );
    eprintln!("  Press SPACE to swing the stick!");
    eprintln!();

    let mut data = model.make_data();

    // Start stick angled ~46° to the side so blade is away from puck.
    data.qpos[stick_q] = 0.8;

    data.forward(&model).expect("forward kinematics");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "SDF Physics 10b — Hockey".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(PhysicsModel(model))
        .insert_resource(PhysicsData(data))
        .insert_resource(HockeyTracker::new(stick_dof))
        .insert_resource(PhysicsAccumulator::default())
        .add_systems(Startup, setup)
        .add_systems(Update, (step_physics_realtime, swing_stick, track_hockey))
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

    // Camera: physics (x,y,z) → bevy (x,z,y).
    // Target midpoint between puck (bevy 0,2,0) and goal (bevy -45,10,0).
    ExampleScene::new(140.0, 70.0)
        .with_target(Vec3::new(-15.0, 15.0, 15.0))
        .with_angles(-0.4, 0.6)
        .with_ground_y(-0.1)
        .spawn(&mut commands, &mut meshes, &mut materials);
}

// ── Stick swing (spacebar input) ─────────────────────────────────────

fn swing_stick(
    keys: Res<ButtonInput<KeyCode>>,
    mut data: ResMut<PhysicsData>,
    tracker: Res<HockeyTracker>,
) {
    if !keys.just_pressed(KeyCode::Space) {
        return;
    }
    // Negative angular velocity = clockwise from above.
    data.0.qvel[tracker.stick_dof] -= 25.0;
    eprintln!(
        "  *** SWING! ω = {:.1} rad/s ***",
        data.0.qvel[tracker.stick_dof]
    );
}

// ── Tracking & pass/fail checks ──────────────────────────────────────

#[derive(Resource)]
struct HockeyTracker {
    stick_dof: usize,
    last_print: f64,
    auto_fired: bool,
    peak_ncon: usize,
    puck_x_at_fire: f64,
    checks_done: bool,
}

impl HockeyTracker {
    const fn new(stick_dof: usize) -> Self {
        Self {
            stick_dof,
            last_print: 0.0,
            auto_fired: false,
            peak_ncon: 0,
            puck_x_at_fire: 0.0,
            checks_done: false,
        }
    }
}

fn track_hockey(mut data: ResMut<PhysicsData>, mut tracker: ResMut<HockeyTracker>) {
    let t = data.0.time;

    // Track peak contact count.
    if data.0.ncon > tracker.peak_ncon {
        tracker.peak_ncon = data.0.ncon;
    }

    // Auto-fire at t=1s for automated pass/fail testing.
    if t >= 1.0 && !tracker.auto_fired {
        tracker.auto_fired = true;
        tracker.puck_x_at_fire = data.0.qpos[0]; // puck x
        data.0.qvel[tracker.stick_dof] -= 25.0;
        eprintln!("  *** AUTO-SWING at t={t:.2}s ***");
    }

    // Periodic telemetry.
    let interval = if t < 3.0 { 0.5 } else { 2.0 };
    if t - tracker.last_print < interval || t > 15.0 {
        return;
    }
    tracker.last_print = t;

    let puck_x = data.0.qpos[0];
    let puck_y = data.0.qpos[1];
    let puck_z = data.0.qpos[2];
    let stick_angle = data.0.qpos[7];
    let ncon = data.0.ncon;

    eprintln!(
        "  t={t:.1}s  puck=({puck_x:.1},{puck_y:.1},{puck_z:.1})  θ={stick_angle:.2}  ncon={ncon}  peak_ncon={}",
        tracker.peak_ncon
    );

    // Pass/fail checks at t=4s (3s after auto-fire).
    if t >= 4.0 && !tracker.checks_done {
        tracker.checks_done = true;

        let puck_dx = data.0.qpos[0] - tracker.puck_x_at_fire;

        eprintln!();
        eprintln!("  === PASS/FAIL checks at t={t:.1}s ===");

        check(
            "all values finite",
            puck_x.is_finite() && puck_y.is_finite() && puck_z.is_finite(),
        );

        check(
            &format!("puck moved toward goal (Δx={puck_dx:.1}, need < -10)"),
            puck_dx < -10.0,
        );

        check(
            &format!("puck near ground (z={puck_z:.1}, need < 10)"),
            puck_z < 10.0,
        );

        check(
            &format!(
                "contacts detected (peak ncon={}, need > 0)",
                tracker.peak_ncon
            ),
            tracker.peak_ncon > 0,
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

// ═══════════════════════════════════════════════════════════════════════
// GPU + VR hockey — implementation skeleton
// ═══════════════════════════════════════════════════════════════════════
//
// The target: hold a hockey stick via Quest 3 controller and hit the puck.
// Full physics runs on GPU (single command buffer per frame, N substeps).
//
// ## Key change: stick = mocap body
//
// The stick is NOT a hinge joint. It's a mocap body — kinematic, driven
// directly by the VR controller pose. The GPU FK shader already handles
// mocap bodies (reads from mocap_pos/mocap_quat buffers, skips joint FK).
// This gives 1:1 controller tracking with zero lag.
//
// To convert: remove the Revolute joint, set body_mocapid on the stick
// body, and upload controller pose to data.mocap_pos[0]/mocap_quat[0]
// each frame. The constraint solver handles stick↔puck contacts normally
// (mocap body has infinite effective mass → pushes puck, not pushed back).
//
// ## Bevy wiring
//
// ```rust
// #[derive(Resource)]
// struct GpuPhysics(sim_gpu::GpuPhysicsPipeline);
//
// fn step_physics_gpu(
//     model: Res<PhysicsModel>,
//     mut data: ResMut<PhysicsData>,
//     gpu: Res<GpuPhysics>,
// ) {
//     gpu.0.step(&model.0, &mut data.0, 4); // 4 substeps, 1 submit
//     data.0.forward_pos_vel(&model.0, true); // CPU FK for rendering
// }
//
// fn controller_to_mocap(
//     controllers: Query<&Transform, With<RightController>>,
//     mut data: ResMut<PhysicsData>,
// ) {
//     if let Ok(tf) = controllers.single() {
//         data.0.mocap_pos[0] = bevy_to_physics_pos(tf.translation);
//         data.0.mocap_quat[0] = bevy_to_physics_quat(tf.rotation);
//     }
// }
// ```
//
// Note: GpuPhysicsPipeline::step() currently uploads qpos/qvel but not
// mocap. Mocap upload (state_bufs.upload_mocap) needs to be added to
// the orchestrator's step() method.
//
// ## VR deps (behind feature flag)
//
// ```toml
// [features]
// vr = ["dep:bevy_mod_openxr", "dep:bevy_mod_xr"]
// ```
//
// See: EXPECTED_BEHAVIOR.md for full architecture and implementation plan.
