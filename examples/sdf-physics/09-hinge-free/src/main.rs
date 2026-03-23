//! SDF Physics 09 — Geometry-Driven Hinge
//!
//! A flanged pin inside a capped socket. SDF collision is the ONLY constraint
//! — no `JointKind::Revolute`. The geometry could be 3D printed in place and
//! would perform similarly.
//!
//! Setup:
//! - Socket: concave (cylinder − bore − cap holes), steel density, Free joint
//! - Pin: convex union (shaft + flanges), PLA density, Free joint
//! - Both children of world (siblings). Ground plane supports the socket.
//! - Pin given initial angular velocity about the bore axis (Z)
//!
//! What SDF collision constrains:
//! - Bore walls → radial (X/Y translation)
//! - Cap + flange → axial (Z translation)
//! - Bore axis rotation → FREE (the 1 unconstrained DOF)
//!
//! Pass criteria (at t ≥ 3s):
//! - Pin radially constrained (|Δx|, |Δy| < 1mm)
//! - Pin axially constrained (|Δz| < 2mm)
//! - Pin still spinning (|ωz| > 0)
//! - No NaN/Inf
//! - Contacts active
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
use nalgebra::{Point3, Vector3};
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_bevy::scene::ExampleScene;

fn main() {
    // ── Socket: tube with annular caps (concave) ─────────────────────
    let socket_solid = Solid::cylinder(5.5, 10.0) // outer shell
        .subtract(Solid::cylinder(3.5, 8.0)) // bore
        .subtract(Solid::cylinder(2.5, 11.0)); // cap openings

    let mut mat_socket = Material::new("steel", 7800.0);
    mat_socket.color = Some([0.4, 0.4, 0.5, 1.0]); // dark steel

    // ── Pin: shaft + flanges (convex union) ──────────────────────────
    let shaft = Solid::cylinder(3.0, 6.0);
    let top_flange = Solid::cylinder(3.2, 1.0).translate(Vector3::new(0.0, 0.0, 6.0));
    let bot_flange = Solid::cylinder(3.2, 1.0).translate(Vector3::new(0.0, 0.0, -6.0));
    let pin_solid = shaft.union(top_flange).union(bot_flange);

    let mut mat_pin = Material::new("PLA", 1250.0);
    mat_pin.color = Some([1.0, 0.35, 0.3, 1.0]); // red

    // ── Mechanism: siblings, both children of world ──────────────────
    let socket_z = 15.0; // above ground (socket outer half_h ≈ 12)
    let mechanism = Mechanism::builder("hinge_free")
        .part(Part::new("socket", socket_solid, mat_socket))
        .part(Part::new("pin", pin_solid, mat_pin))
        .joint(JointDef::new(
            "socket_free",
            "world",
            "socket",
            JointKind::Free,
            Point3::new(0.0, 0.0, socket_z),
            Vector3::z(),
        ))
        .joint(JointDef::new(
            "pin_free",
            "world",
            "pin",
            JointKind::Free,
            Point3::new(0.0, 0.0, socket_z),
            Vector3::z(),
        ))
        .build();

    let mut model = mechanism.to_model(1.0, 0.5);
    model.add_ground_plane();

    // qpos layout: socket [0..7], pin [7..14]
    // qvel layout: socket [0..6], pin [6..12]
    let pin_q = 7;

    eprintln!();
    eprintln!("  Geometry-Driven Hinge — SDF Physics 09");
    eprintln!("  ────────────────────────────────────────");
    eprintln!("  Bodies: {}, Geoms: {}", model.nbody, model.ngeom);
    eprintln!("  Timestep: {:.4} s", model.timestep);
    eprintln!("  Constraint: SDF collision only (no JointKind::Revolute)");
    eprintln!("  Bore: R=3.5mm, Pin shaft: R=3.0mm (0.5mm clearance)");
    eprintln!("  Flanges: R=3.2mm, Cap opening: R=2.5mm");
    eprintln!();

    let mut data = model.make_data();

    // Pin starts 0.9mm below socket center (bottom flange near cap)
    data.qpos[pin_q + 2] -= 0.9;
    // Initial spin about bore axis (Z)
    data.qvel[11] = 15.0; // ωz = 15 rad/s (pin angular velocity about Z)

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

    ExampleScene::new(50.0, 70.0)
        .with_target(Vec3::new(0.0, 12.0, 0.0))
        .with_angles(0.5, 0.3)
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
    let omega_z = data.0.qvel[11]; // pin angular velocity Z (qvel[6+5])

    let interval = if t < 3.0 { 0.5 } else { 2.0 };
    if t - tracker.last_print < interval || t > 15.0 {
        return;
    }
    tracker.last_print = t;

    let radial = rx.hypot(ry);
    eprintln!(
        "  t={t:.1}s  Δpos=({rx:+.3},{ry:+.3},{rz:+.3})  r={radial:.3}  ωz={omega_z:+.3}  ncon={}",
        data.0.ncon
    );

    if t >= 3.0 && !tracker.checks_done {
        tracker.checks_done = true;

        eprintln!();
        eprintln!("  === PASS/FAIL checks at t={t:.1}s ===");

        // 1. No NaN/Inf
        let finite = rx.is_finite() && ry.is_finite() && rz.is_finite() && omega_z.is_finite();
        check("all values finite", finite);

        // 2. Radial constraint
        check(
            &format!("radial |Δxy| < 1mm (got {radial:.3})"),
            radial < 1.0,
        );

        // 3. Axial constraint
        check(
            &format!("axial |Δz| < 2mm (got {:.3})", rz.abs()),
            rz.abs() < 2.0,
        );

        // 4. Still spinning (free DOF)
        // Contact friction slows spin — threshold accounts for damping
        check(
            &format!("|ωz| > 0.01 rad/s (got {:.3})", omega_z.abs()),
            omega_z.abs() > 0.01,
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
