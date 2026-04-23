//! Integration tests: cf-design → sim-mjcf → sim-core.
//!
//! These tests verify the full pipeline from mechanism definition through
//! MJCF generation to simulation. Requires `sim-mjcf` dev-dependency.

#![cfg(test)]
#![allow(clippy::unwrap_used, clippy::panic)]

use nalgebra::{Point3, Vector3};

use crate::{
    ActuatorDef, ActuatorKind, InfillKind, JointDef, JointKind, Material, Mechanism, Part, Solid,
    TendonDef, TendonWaypoint, templates,
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

// ── Phase 3: Full bio-gripper integration ─────────────────────────────

/// Phase 3 exit criteria integration test.
///
/// Bio-inspired gripper with:
/// - Organic blends (`smooth_union` on palm)
/// - Gyroid-infilled bone
/// - Smooth tendon channels
/// - Compliant finger joints (`FlexZone` auto-splitting with spring-damper)
///
/// Pipeline: define → build → MJCF → sim-mjcf parse → sim-core step → STL export.
#[test]
// Procedural glue code; natural breakpoints are few.
#[allow(clippy::too_many_lines)]
fn phase3_bio_gripper_full_integration() {
    let pla_e = Material::new("PLA", 1250.0).with_youngs_modulus(3.5e9);

    // ── Palm: organic blend of two cuboids with rounded edges ────
    let palm_base = Solid::cuboid(Vector3::new(20.0, 15.0, 5.0));
    let palm_grip =
        Solid::cuboid(Vector3::new(15.0, 8.0, 5.0)).translate(Vector3::new(0.0, 15.0, 0.0));
    let palm_solid = palm_base.smooth_union(palm_grip, 3.0).round(0.5);
    let palm = Part::new("palm", palm_solid, pla_e.clone());

    // ── Finger: 3 phalanges with flex joints (Session 18 template) ──
    let (finger_parts, finger_joints) = templates::finger("finger", 25.0, 3.0, 3, pla_e.clone());

    // ── Bone: gyroid-infilled structural link (Session 18 template) ──
    let bone = templates::link_infilled("bone", 15.0, 4.0, pla_e, InfillKind::Gyroid);

    // ── Assemble mechanism ──────────────────────────────────────────
    let mut builder = Mechanism::builder("bio_gripper_p3").part(palm);

    for p in finger_parts {
        builder = builder.part(p);
    }
    builder = builder.part(bone);

    // Connect palm → finger proximal phalanx.
    builder = builder.joint(
        JointDef::new(
            "knuckle",
            "palm",
            "finger_0",
            JointKind::Revolute,
            Point3::new(10.0, 20.0, 0.0),
            Vector3::x(),
        )
        .with_range(-0.1, 1.8),
    );

    // Connect palm → bone.
    builder = builder.joint(JointDef::new(
        "palm_bone",
        "palm",
        "bone",
        JointKind::Revolute,
        Point3::new(0.0, -15.0, 0.0),
        Vector3::x(),
    ));

    // Add finger flex joints (with spring-damper from FlexZone splitting).
    for j in finger_joints {
        builder = builder.joint(j);
    }

    // Tendon: routes through palm (2 waypoints → channel subtraction).
    builder = builder.tendon(
        TendonDef::new(
            "flexor",
            vec![
                TendonWaypoint::new("palm", Point3::new(8.0, -5.0, 0.0)),
                TendonWaypoint::new("palm", Point3::new(8.0, 18.0, 0.0)),
                TendonWaypoint::new("finger_0", Point3::new(0.0, 0.0, 4.0)),
                TendonWaypoint::new("finger_2", Point3::new(0.0, 0.0, 22.0)),
            ],
            1.0,
        )
        .with_stiffness(100.0)
        .with_damping(5.0),
    );

    // Actuator: motor driving the tendon.
    builder = builder.actuator(
        ActuatorDef::new("motor", "flexor", ActuatorKind::Motor, (-50.0, 50.0))
            .with_ctrl_range(-1.0, 1.0),
    );

    let mechanism = builder.build();

    // ── Verify structure ────────────────────────────────────────────
    // 1 palm + 3 finger phalanges + 1 bone = 5 parts.
    assert_eq!(mechanism.parts().len(), 5, "expected 5 parts");
    // 1 knuckle + 1 palm-bone + 2 flex joints = 4 joints.
    assert_eq!(mechanism.joints().len(), 4, "expected 4 joints");
    assert_eq!(mechanism.tendons().len(), 1, "expected 1 tendon");
    assert_eq!(mechanism.actuators().len(), 1, "expected 1 actuator");

    // Tendon channel should be subtracted from palm.
    let palm_part = &mechanism.parts()[0];
    assert_eq!(palm_part.name(), "palm");
    let channel_pt = Point3::new(8.0, 5.0, 0.0);
    let val = palm_part.solid().evaluate(&channel_pt);
    assert!(
        val > 0.0,
        "tendon channel at (8,5,0) in palm should be void, got {val}"
    );

    // ── MJCF generation ─────────────────────────────────────────────
    let xml = mechanism.to_mjcf(2.0);

    // Verify spring-damper attributes from flex zone splitting.
    assert!(
        xml.contains("stiffness="),
        "MJCF should contain stiffness attribute from flex joints"
    );
    assert!(
        xml.contains("damping="),
        "MJCF should contain damping attribute from flex joints"
    );

    // ── sim-mjcf parse ──────────────────────────────────────────────
    let model = sim_mjcf::load_model(&xml)
        .unwrap_or_else(|e| panic!("sim-mjcf failed to parse Phase 3 gripper: {e}"));

    // 1 (world) + 5 parts = 6 bodies.
    assert_eq!(model.nbody, 6, "expected 6 bodies (world + 5 parts)");
    assert_eq!(model.njnt, 4, "expected 4 joints");
    assert_eq!(model.ngeom, 5, "expected 5 geoms (one per part)");
    assert_eq!(model.ntendon, 1, "expected 1 tendon");
    assert_eq!(model.nu, 1, "expected 1 actuator");

    // ── Simulate ────────────────────────────────────────────────────
    let mut data = model.make_data();
    for _ in 0..100 {
        data.step(&model)
            .unwrap_or_else(|e| panic!("step failed: {e}"));
    }
    assert!(
        data.time > 0.0,
        "simulation time should have advanced, got {}",
        data.time
    );

    // ── STL export ──────────────────────────────────────────────────
    let stl_kit = mechanism.to_stl_kit(1.0);
    assert_eq!(stl_kit.len(), 5, "should export one mesh per part");
    for (name, mesh) in &stl_kit {
        assert!(
            !mesh.vertices.is_empty(),
            "part \"{name}\" mesh should have vertices"
        );
        assert!(
            !mesh.faces.is_empty(),
            "part \"{name}\" mesh should have faces"
        );
    }
}

// ── Collision diagnostic ────────────────────────────────────────────

// ── Phase 5: Design optimization through simulation ─────────────────

/// Phase 5 exit criteria integration test.
///
/// Demonstrates the full design-through-physics optimization pipeline:
///   parameterized geometry → re-mesh → MJCF → parse → simulate → contact
///   force → FD gradient → parameter update → repeat.
///
/// A parameterized sphere (adjustable radius) on a free joint falls onto
/// a ground plane. The optimizer maximizes steady-state contact force
/// by increasing the sphere's radius — larger sphere = more mass = more
/// weight = more contact force. Contact force ∝ mass × g ∝ R³.
///
/// Gradient chain:
///   `∂J/∂θ ≈ [J(θ+ε) − J(θ−ε)] / 2ε`
/// where each `J(θ)` = re-mesh → MJCF → `load_model` → simulate → measure force.
#[test]
fn phase5_parameterized_grasp_optimization() {
    use crate::ParamStore;
    use crate::optim::{OptimConfig, minimize_fd};
    use sim_core::ConstraintType;

    let store = ParamStore::new();
    let _radius = store.add("ball_radius", 15.0);

    let mat = Material::new("PLA", 1250.0);

    let config = OptimConfig {
        max_iters: 3,
        // Contact force = mg = (4/3)πR³ρg. At R=15 (sim-core meters), ρ=1250:
        //   ∂F/∂R = 4πR²ρg ≈ 3.5e7 → lr=1e-9 gives ΔR≈0.035 per step.
        learning_rate: 1e-9,
        fd_eps: 0.5,
        grad_tol: 1e-10,
    };

    let result = minimize_fd(
        &store,
        || {
            // Read current parameter value and rebuild geometry.
            // The mechanism is reconstructed each eval so the joint anchor
            // tracks the ball radius — keeping consistent ground clearance.
            let r = store.get("ball_radius").unwrap_or(15.0);

            let mechanism = Mechanism::builder("grasp_opt")
                .part(Part::new(
                    "frame",
                    Solid::cuboid(Vector3::new(10.0, 10.0, 10.0)),
                    mat.clone(),
                ))
                .part(Part::new("ball", Solid::sphere(r), mat.clone()))
                .joint(JointDef::new(
                    "drop",
                    "frame",
                    "ball",
                    JointKind::Free,
                    Point3::new(0.0, 0.0, r + 0.5), // ball bottom 0.5mm above plane
                    Vector3::z(),
                ))
                .build();

            // Full pipeline: parameterized geometry → mesh → MJCF → parse → simulate.
            let mut xml = mechanism.to_mjcf(2.0);

            // Inject a ground plane for the ball to land on.
            let insert_pos = xml.find("<body").unwrap_or(0);
            xml.insert_str(
                insert_pos,
                "<geom type=\"plane\" size=\"50 50 0.1\"/>\n    ",
            );

            let model =
                sim_mjcf::load_model(&xml).unwrap_or_else(|e| panic!("load_model failed: {e}"));
            let mut data = model.make_data();

            // Settle: let initial impact transients dissipate.
            let settle_steps = 200;
            let measure_steps = 300;
            for _ in 0..settle_steps {
                data.step(&model)
                    .unwrap_or_else(|e| panic!("step failed: {e}"));
            }

            // Measure: average steady-state contact force.
            let mut contact_force = 0.0_f64;
            for _ in 0..measure_steps {
                data.step(&model)
                    .unwrap_or_else(|e| panic!("step failed: {e}"));
                // Sum only contact constraint forces — excludes joint limits,
                // equality constraints, friction loss, etc.
                for (i, ct) in data.efc_type.iter().enumerate() {
                    if matches!(
                        ct,
                        ConstraintType::ContactFrictionless
                            | ConstraintType::ContactPyramidal
                            | ConstraintType::ContactElliptic
                    ) {
                        contact_force += data.efc_force[i].abs();
                    }
                }
            }

            // Objective: maximize contact force (negate for minimization).
            -(contact_force / f64::from(measure_steps))
        },
        &config,
    );

    // Pipeline executed without crash.
    assert!(
        result.iterations >= 2,
        "expected ≥2 iterations, got {}",
        result.iterations
    );

    // Objective improved (contact force increased).
    assert!(
        result.objective < result.history[0],
        "expected improvement: final={:.6} vs initial={:.6}",
        result.objective,
        result.history[0],
    );

    // Radius should have increased (larger = more mass = more contact force).
    let final_radius = store.get("ball_radius").unwrap_or(0.0);
    assert!(
        final_radius > 15.0,
        "expected ball_radius to increase from 15.0, got {final_radius:.4}"
    );
}
