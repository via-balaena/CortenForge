//! Integration tests: cf-design → sim-mjcf → sim-core.
//!
//! These tests verify the full pipeline from mechanism definition through
//! MJCF generation to simulation. Requires `sim-mjcf` dev-dependency.

#![allow(clippy::unwrap_used, clippy::panic)]

use nalgebra::{Point3, Vector3};

use crate::{
    ActuatorDef, ActuatorKind, JointDef, JointKind, Material, Mechanism, Part, Solid, TendonDef,
    TendonWaypoint,
};

fn pla() -> Material {
    Material::new("PLA", 1250.0)
}

/// Build a two-part mechanism (palm + finger, revolute joint) for testing.
fn two_part_mechanism() -> Mechanism {
    Mechanism::builder("two_part")
        .part(Part::new(
            "palm",
            Solid::cuboid(Vector3::new(10.0, 10.0, 5.0)),
            pla(),
        ))
        .part(Part::new(
            "finger",
            Solid::cuboid(Vector3::new(3.0, 8.0, 3.0)),
            pla(),
        ))
        .joint(JointDef::new(
            "knuckle",
            "palm",
            "finger",
            JointKind::Revolute,
            Point3::new(5.0, 0.0, 0.0),
            Vector3::x(),
        ))
        .build()
}

/// Build a bio-gripper mechanism for full pipeline testing.
fn bio_gripper_mechanism() -> Mechanism {
    let mat = pla();

    Mechanism::builder("bio_gripper")
        .part(Part::new(
            "palm",
            Solid::cuboid(Vector3::new(18.0, 22.0, 4.0)),
            mat.clone(),
        ))
        .part(Part::new(
            "finger_1",
            Solid::capsule(3.0, 12.0),
            mat.clone(),
        ))
        .part(Part::new("finger_2", Solid::capsule(3.0, 12.0), mat))
        .joint(
            JointDef::new(
                "knuckle_1",
                "palm",
                "finger_1",
                JointKind::Revolute,
                Point3::new(10.0, 20.0, 0.0),
                Vector3::x(),
            )
            .with_range(-0.1, 1.8),
        )
        .joint(
            JointDef::new(
                "knuckle_2",
                "palm",
                "finger_2",
                JointKind::Revolute,
                Point3::new(-10.0, 20.0, 0.0),
                Vector3::x(),
            )
            .with_range(-0.1, 1.8),
        )
        .tendon(
            TendonDef::new(
                "flexor_1",
                vec![
                    TendonWaypoint::new("palm", Point3::new(8.0, -10.0, 0.0)),
                    TendonWaypoint::new("palm", Point3::new(8.0, 18.0, 0.0)),
                    TendonWaypoint::new("finger_1", Point3::new(0.0, 5.0, 0.0)),
                    TendonWaypoint::new("finger_1", Point3::new(0.0, 20.0, 0.0)),
                ],
                1.5,
            )
            .with_stiffness(100.0)
            .with_damping(5.0),
        )
        .actuator(
            ActuatorDef::new("motor_1", "flexor_1", ActuatorKind::Motor, (-50.0, 50.0))
                .with_ctrl_range(-1.0, 1.0),
        )
        .build()
}

// ── Round-trip parse test ───────────────────────────────────────────────

#[test]
fn mjcf_parse_round_trip() {
    let mechanism = two_part_mechanism();
    let xml = mechanism.to_mjcf(2.0);

    // Parse the generated MJCF through sim-mjcf.
    let model = sim_mjcf::load_model(&xml)
        .unwrap_or_else(|e| panic!("sim-mjcf failed to parse cf-design MJCF: {e}"));

    // sim-core counts worldbody as body 0: 1 (world) + 2 (palm, finger) = 3.
    assert_eq!(model.nbody, 3, "expected 3 bodies (world + palm + finger)");
    assert_eq!(model.njnt, 1, "expected 1 joint (knuckle)");
    assert_eq!(model.ngeom, 2, "expected 2 geoms (one per part)");
}

#[test]
fn mjcf_bio_gripper_parse() {
    let mechanism = bio_gripper_mechanism();
    let xml = mechanism.to_mjcf(2.0);

    let model = sim_mjcf::load_model(&xml)
        .unwrap_or_else(|e| panic!("sim-mjcf failed to parse bio-gripper MJCF: {e}"));

    // 1 (world) + 3 (palm + 2 fingers) = 4 bodies, 2 joints.
    assert_eq!(model.nbody, 4, "expected 4 bodies (world + 3 parts)");
    assert_eq!(model.njnt, 2, "expected 2 joints");
    assert_eq!(model.ngeom, 3, "expected 3 geoms (one per part)");
    // 1 tendon, 1 actuator.
    assert_eq!(model.ntendon, 1, "expected 1 tendon");
    assert_eq!(model.nu, 1, "expected 1 actuator (1 ctrl DOF)");
}

// ── Simulation step test ────────────────────────────────────────────────

#[test]
fn mjcf_simulation_step() {
    let mechanism = two_part_mechanism();
    let xml = mechanism.to_mjcf(2.0);

    let model = sim_mjcf::load_model(&xml).unwrap_or_else(|e| panic!("load_model failed: {e}"));

    let mut data = model.make_data();

    // The hinge joint has 1 DOF → qpos should have at least 1 element.
    assert!(
        !data.qpos.is_empty(),
        "expected non-empty qpos for revolute joint"
    );

    // Step 100 times — the primary goal is verifying the full pipeline
    // (cf-design → MJCF → sim-mjcf parse → sim-core step) works without crash.
    for _ in 0..100 {
        data.step(&model)
            .unwrap_or_else(|e| panic!("step failed: {e}"));
    }

    // Verify simulation ran (time advanced).
    assert!(
        data.time > 0.0,
        "simulation time should have advanced, got {}",
        data.time
    );
}
