//! SDF Physics 09 — Geometry-Driven Hinge (Pendulum)
//!
//! A flanged pin inside a capped bearing block. SDF collision is the ONLY
//! constraint — no `JointKind::Revolute`. The geometry could be 3D printed
//! in place and would perform similarly.
//!
//! Setup:
//! - Socket: concave bearing block (cuboid − bore − cap holes), steel, Free joint
//! - Pin: convex union (shaft + flanges + crank arm + weight), PLA, Free joint
//! - Both children of world (siblings). Ground plane supports socket.
//! - Bore axis horizontal (Y). Gravity creates pendulum swing.
//! - Pin starts with weight displaced 90° from equilibrium.
//!
//! What SDF collision constrains:
//! - Bore walls → radial (X/Z translation)
//! - Cap + flange → axial (Y translation)
//! - Bore axis rotation (Y) → FREE (the 1 unconstrained DOF)
//!
//! Run with: `cargo run -p example-sdf-09-hinge-free --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::let_underscore_must_use,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation
)]

use bevy::prelude::*;
use cf_design::{JointDef, JointKind, Material, Mechanism, Part, Solid};
use nalgebra::{Point3, UnitQuaternion, Vector3};
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_bevy::scene::ExampleScene;

fn main() {
    // Rotation: bore Z → bore Y (horizontal)
    let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::FRAC_PI_2);

    // ── Socket: bearing block with bore (concave) ────────────────────
    // Pre-rotation (bore along Z): cuboid 20×20×24mm
    // After rotation: 20mm (X) × 24mm (Y, bore) × 20mm (Z, height)
    let socket_solid = Solid::cuboid(Vector3::new(10.0, 10.0, 12.0))
        .subtract(Solid::cylinder(5.0, 10.0)) // bore R=5.0, half_h=10
        .subtract(Solid::cylinder(3.5, 13.0)) // cap openings R=3.5
        .rotate(rot);

    let mut mat_socket = Material::new("steel", 7800.0);
    mat_socket.color = Some([0.4, 0.4, 0.5, 0.3]); // semi-transparent steel

    // ── Pin: shaft + flanges + crank arm + pendulum weight ───────────
    // Pre-rotation (bore along Z)
    let shaft = Solid::cylinder(4.0, 7.0); // R=4.0 (1.0mm bore clearance)
    let top_flange = Solid::cylinder(4.5, 1.0) // R=4.5 (1.0mm cap trap)
        .translate(Vector3::new(0.0, 0.0, 7.0));
    let bot_flange = Solid::cylinder(4.5, 1.0).translate(Vector3::new(0.0, 0.0, -7.0));
    // Arm through cap opening (R=2.0 < cap_opening R=3.5)
    let arm = Solid::cylinder(2.0, 8.0).translate(Vector3::new(0.0, 0.0, 18.0)); // z=10..26, through top cap
    // Pendulum weight: offset from bore axis in -Y (becomes -Z after rotation)
    let weight = Solid::sphere(3.0).translate(Vector3::new(0.0, -6.0, 26.0));

    let pin_solid = shaft
        .union(top_flange)
        .union(bot_flange)
        .union(arm)
        .union(weight)
        .rotate(rot);

    let mut mat_pin = Material::new("PLA", 1250.0);
    mat_pin.color = Some([1.0, 0.35, 0.3, 1.0]); // red

    // ── Mechanism: siblings, both children of world ──────────────────
    let socket_z = 12.0; // housing half_z=10 after rotation
    let mechanism = Mechanism::builder("hinge_free")
        .part(Part::new("socket", socket_solid, mat_socket))
        .part(Part::new("pin", pin_solid, mat_pin))
        .joint(JointDef::new(
            "socket_free",
            "world",
            "socket",
            JointKind::Free,
            Point3::new(0.0, 0.0, socket_z),
            Vector3::y(), // bore axis
        ))
        .joint(JointDef::new(
            "pin_free",
            "world",
            "pin",
            JointKind::Free,
            Point3::new(0.0, 0.0, socket_z),
            Vector3::y(), // bore axis
        ))
        .build();

    // SDF: 2.0mm collision, 0.5mm visual (coarser to keep interactive)
    let mut model = mechanism.to_model(2.0, 0.5);
    model.add_ground_plane();

    // Enable GPU-accelerated SDF collision (falls back to CPU if unavailable)
    match sim_gpu::enable_gpu_collision(&mut model) {
        Ok(()) => eprintln!("  GPU collision enabled"),
        Err(e) => eprintln!("  GPU collision unavailable: {e}"),
    }

    // qpos layout: socket [0..7], pin [7..14]
    // qvel layout: socket [0..6], pin [6..12]
    let pin_q = 7;

    eprintln!();
    eprintln!("  Geometry-Driven Hinge — SDF Physics 09");
    eprintln!("  ────────────────────────────────────────");
    eprintln!("  Bodies: {}, Geoms: {}", model.nbody, model.ngeom);
    eprintln!("  Timestep: {:.4} s", model.timestep);
    eprintln!("  Constraint: SDF collision only (no JointKind::Revolute)");
    eprintln!("  Bore axis: Y (horizontal), pendulum via gravity");
    eprintln!("  Bore: R=5.0mm, Shaft: R=4.0mm (1.0mm clearance)");
    eprintln!("  Flanges: R=4.5mm, Cap opening: R=3.5mm");
    eprintln!();

    let mut data = model.make_data();

    // Start with weight at 90° from equilibrium (pointing sideways)
    // Rotate pin -90° about Y (bore axis) so weight is horizontal
    let angle = -std::f64::consts::FRAC_PI_2;
    data.qpos[pin_q + 3] = (angle / 2.0).cos(); // qw
    data.qpos[pin_q + 5] = (angle / 2.0).sin(); // qy

    data.forward(&model).expect("forward kinematics");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "SDF Physics 09 — Geometry-Driven Hinge".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(PhysicsModel(model))
        .insert_resource(PhysicsData(data))
        .insert_resource(HingeTracker::default())
        .insert_resource(PhysicsAccumulator::default())
        .add_systems(Startup, setup)
        .add_systems(Update, (step_physics_realtime, track_hinge))
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
        .with_target(Vec3::new(0.0, 12.0, 0.0))
        .with_angles(0.8, 0.3)
        .with_ground_y(-0.5)
        .spawn(&mut commands, &mut meshes, &mut materials);
}

#[derive(Resource, Default)]
struct HingeTracker {
    last_print: f64,
    checks_done: bool,
}

fn track_hinge(data: Res<PhysicsData>, mut tracker: ResMut<HingeTracker>) {
    let t = data.0.time;
    let pin_q = 7;
    let socket_q = 0;

    // Relative position: pin - socket
    let rx = data.0.qpos[pin_q] - data.0.qpos[socket_q];
    let ry = data.0.qpos[pin_q + 1] - data.0.qpos[socket_q + 1];
    let rz = data.0.qpos[pin_q + 2] - data.0.qpos[socket_q + 2];
    let omega_y = data.0.qvel[10]; // pin angular velocity about Y (bore axis)

    let interval = if t < 3.0 { 0.5 } else { 2.0 };
    if t - tracker.last_print < interval || t > 15.0 {
        return;
    }
    tracker.last_print = t;

    let radial = rx.hypot(rz); // X-Z plane (perpendicular to bore Y)
    eprintln!(
        "  t={t:.1}s  Δpos=({rx:+.3},{ry:+.3},{rz:+.3})  r_xz={radial:.3}  ωy={omega_y:+.3}  ncon={}",
        data.0.ncon
    );

    if t >= 3.0 && !tracker.checks_done {
        tracker.checks_done = true;

        eprintln!();
        eprintln!("  === PASS/FAIL checks at t={t:.1}s ===");

        // 1. No NaN/Inf
        let finite = rx.is_finite() && ry.is_finite() && rz.is_finite() && omega_y.is_finite();
        check("all values finite", finite);

        // 2. Radial constraint (bore walls constrain X-Z)
        check(
            &format!("radial |Δxz| < 1.5mm (got {radial:.3})"),
            radial < 1.5,
        );

        // 3. Axial constraint (flanges+caps constrain Y)
        check(
            &format!("axial |Δy| < 3mm (got {:.3})", ry.abs()),
            ry.abs() < 3.0,
        );

        // 4. Still swinging (pendulum not fully damped)
        check(
            &format!("|ωy| > 0.01 rad/s (got {:.3})", omega_y.abs()),
            omega_y.abs() > 0.01,
        );

        // 5. Contacts active
        check(
            &format!("contacts active (ncon={})", data.0.ncon),
            data.0.ncon > 0,
        );

        // 6. Socket on ground (not fallen through)
        let socket_z = data.0.qpos[socket_q + 2];
        check(
            &format!("socket on ground (z={socket_z:.1})"),
            socket_z > 5.0,
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
