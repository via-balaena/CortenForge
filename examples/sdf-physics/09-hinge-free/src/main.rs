//! SDF Physics 09 — Hinge (Free Swing)
//!
//! Two SDF bodies connected by a revolute joint. The arm swings freely
//! under gravity. No collision between them — this only tests that joints
//! work correctly with SDF bodies.
//!
//! Setup:
//! - "base" cuboid: welded to world (no joint) — fixed anchor point
//! - "arm" cuboid: revolute joint child of base — swings about Y axis
//! - Initial angle: 60° from vertical
//! - No ground plane, no inter-body collision
//!
//! Isolates: joint integration with SDF geom types. All previous steps
//! used free bodies. This is the first step with articulation.
//!
//! Pass criteria:
//! - Arm swings smoothly on the hinge
//! - No explosion or divergence
//! - Energy is approximately conserved (small damping OK)
//! - Joint axis is correct (arm swings in the expected plane)
//!
//! New concept: revolute joints with SDF bodies
//! Depends on: 08-stack
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
use sim_core::{ENABLE_ENERGY, Integrator};
use std::f64::consts::PI;

const HALF_BASE: f64 = 5.0; // mm half-extents for base cube
const ARM_HALF_XY: f64 = 3.0; // mm half-extents for arm cross-section
const ARM_HALF_Z: f64 = 12.0; // mm half-extent for arm length
const INITIAL_ANGLE: f64 = PI / 3.0; // 60° from rest

fn main() {
    let mut mat_base = Material::new("PLA", 1250.0);
    mat_base.color = Some([0.3, 0.5, 1.0, 1.0]); // blue

    let mut mat_arm = Material::new("PLA", 1250.0);
    mat_arm.color = Some([1.0, 0.35, 0.3, 1.0]); // red

    // Base: fixed cube (no joint → welded to world at origin)
    // Arm: elongated cuboid hanging from base via revolute joint
    //
    // with_joint_origin places the joint attachment at the top of the arm mesh,
    // so the arm hangs downward from the anchor point.
    let mechanism = Mechanism::builder("hinge_free")
        .part(Part::new(
            "base",
            Solid::cuboid(Vector3::new(HALF_BASE, HALF_BASE, HALF_BASE)),
            mat_base,
        ))
        .part(
            Part::new(
                "arm",
                Solid::cuboid(Vector3::new(ARM_HALF_XY, ARM_HALF_XY, ARM_HALF_Z)),
                mat_arm,
            )
            .with_joint_origin(Vector3::new(0.0, 0.0, ARM_HALF_Z)),
        )
        .joint(JointDef::new(
            "hinge",
            "base",
            "arm",
            JointKind::Revolute,
            Point3::new(0.0, 0.0, -HALF_BASE), // anchor at bottom of base
            Vector3::y(),                      // Y axis → arm swings in XZ plane
        ))
        .build();

    let mut model = mechanism.to_model(1.0, 0.3);
    // No ground plane — pure joint articulation test

    // Enable energy tracking and use RK4 for better conservation
    model.enableflags |= ENABLE_ENERGY;
    model.integrator = Integrator::RungeKutta4;

    // Revolute joint only → nq=1 (angle), nv=1 (angular velocity)
    eprintln!();
    eprintln!("  Hinge Diagnostics");
    eprintln!("  -----------------");
    eprintln!("  Bodies: {}, Joints: {}", model.nbody, model.njnt);
    eprintln!("  nq: {}, nv: {}", model.nq, model.nv);
    eprintln!("  Timestep: {:.4} s", model.timestep);
    eprintln!("  Integrator: RK4");
    eprintln!("  Initial angle: {:.1}°", INITIAL_ANGLE.to_degrees());
    eprintln!();

    let mut data = model.make_data();
    data.qpos[0] = INITIAL_ANGLE; // Start displaced 60° from rest
    data.forward(&model).expect("forward kinematics");

    let initial_energy = data.energy_kinetic + data.energy_potential;
    eprintln!("  E₀ = {initial_energy:.6} (initial energy)");
    eprintln!();

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "SDF Physics 09 — Hinge (Free Swing)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(PhysicsModel(model))
        .insert_resource(PhysicsData(data))
        .insert_resource(HingeTracker {
            initial_energy,
            ..default()
        })
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

    // Camera targets the midpoint of the pendulum swing
    ExampleScene::new(60.0, 70.0)
        .with_target(Vec3::new(0.0, -10.0, 0.0))
        .with_angles(0.3, 0.4)
        .with_ground_y(-35.0)
        .spawn(&mut commands, &mut meshes, &mut materials);
}

#[derive(Resource, Default)]
struct HingeTracker {
    last_print: f64,
    checks_done: bool,
    initial_energy: f64,
    max_angle: f64,
    min_angle: f64,
}

fn track_hinge(data: Res<PhysicsData>, mut tracker: ResMut<HingeTracker>) {
    let t = data.0.time;

    let angle = data.0.qpos[0];
    let omega = data.0.qvel[0];

    // Track angle extremes
    if angle > tracker.max_angle {
        tracker.max_angle = angle;
    }
    if angle < tracker.min_angle {
        tracker.min_angle = angle;
    }

    let interval = if t < 3.0 { 0.5 } else { 2.0 };
    if t - tracker.last_print < interval || t > 15.0 {
        return;
    }
    tracker.last_print = t;

    let energy = data.0.energy_kinetic + data.0.energy_potential;
    let energy_drift = if tracker.initial_energy.abs() > 1e-12 {
        (energy - tracker.initial_energy) / tracker.initial_energy.abs() * 100.0
    } else {
        0.0
    };

    eprintln!(
        "  t={t:.1}s  θ={:+.1}°  ω={omega:+.3} rad/s  E={energy:.6}  ΔE={energy_drift:+.2}%",
        angle.to_degrees()
    );

    if t >= 5.0 && !tracker.checks_done {
        tracker.checks_done = true;

        eprintln!();
        eprintln!("  === PASS/FAIL checks at t={t:.1}s ===");

        // 1. Arm is still swinging (angular velocity nonzero)
        check(
            &format!("|ω| > 0.01 rad/s (got {:.4})", omega.abs()),
            omega.abs() > 0.01,
        );

        // 2. No NaN/Inf in positions
        check(&format!("θ is finite (got {angle:.4})"), angle.is_finite());
        check(&format!("ω is finite (got {omega:.4})"), omega.is_finite());

        // 3. Joint angle stays in expected range (no runaway spinning)
        check(
            &format!("|θ| < 2π (got {:.2}°)", angle.to_degrees().abs()),
            angle.abs() < 2.0 * PI,
        );

        // 4. Angle has oscillated through both sides
        check(
            &format!(
                "oscillated: max={:.1}° min={:.1}°",
                tracker.max_angle.to_degrees(),
                tracker.min_angle.to_degrees()
            ),
            tracker.max_angle > 0.1 && tracker.min_angle < -0.1,
        );

        // 5. Energy approximately conserved (< 5% drift with RK4)
        check(
            &format!("energy drift < 5% (got {energy_drift:+.2}%)"),
            energy_drift.abs() < 5.0,
        );

        // 6. No explosion — angular velocity bounded
        check(
            &format!("|ω| < 100 rad/s (got {:.3})", omega.abs()),
            omega.abs() < 100.0,
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
