//! Unified PGS/CG solver tests (§29).
//!
//! These tests verify the unified constraint solver implementations:
//! - ne/nf constraint ordering bookkeeping
//! - Correct constraint row ordering (equality → friction → limits+contacts)

use sim_mjcf::load_model;

/// Helper: load model from MJCF string.
fn model_from_mjcf(mjcf: &str) -> (sim_core::Model, sim_core::Data) {
    let model = load_model(mjcf).expect("MJCF should load");
    let data = model.make_data();
    (model, data)
}

// ============================================================================
// ne/nf Bookkeeping Tests
// ============================================================================

#[test]
fn test_ne_nf_zero_for_unconstrained_model() {
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="unconstrained">
            <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"/>
            <worldbody>
                <body name="b" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
    );

    // Step once — no contacts, no constraints
    data.step(&model).expect("step should succeed");
    assert_eq!(data.ne, 0, "no equality constraints");
    assert_eq!(data.nf, 0, "no friction loss constraints");
}

#[test]
fn test_ne_counts_equality_constraints() {
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="weld_eq">
            <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"/>
            <worldbody>
                <body name="a" pos="0 0 0.5">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <body name="b" pos="0 0 1.0">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <weld body1="a" body2="b"/>
            </equality>
        </mujoco>
    "#,
    );

    data.step(&model).expect("step should succeed");
    // Weld = 6 rows (3 translation + 3 rotation)
    assert_eq!(data.ne, 6, "weld constraint should produce 6 equality rows");
}

#[test]
fn test_nf_counts_friction_loss() {
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="friction_loss">
            <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"/>
            <worldbody>
                <body name="arm" pos="0 0 0">
                    <joint type="hinge" axis="0 0 1" frictionloss="1.0"/>
                    <geom type="capsule" size="0.05 0.3" mass="1.0"/>
                    <body name="forearm" pos="0 0.6 0">
                        <joint type="hinge" axis="0 0 1" frictionloss="0.5"/>
                        <geom type="capsule" size="0.04 0.2" mass="0.5"/>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#,
    );

    data.step(&model).expect("step should succeed");
    assert_eq!(
        data.nf, 2,
        "two joints with frictionloss should produce 2 friction rows"
    );
    assert_eq!(data.ne, 0, "no equality constraints");
}

#[test]
fn test_ne_nf_ordering_equality_then_friction() {
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="mixed">
            <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"/>
            <worldbody>
                <body name="a" pos="0 0 0.5">
                    <joint name="j1" type="hinge" axis="0 0 1" frictionloss="1.0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b" pos="0 0.5 0">
                        <joint name="j2" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
            <equality>
                <joint joint1="j1" joint2="j2"/>
            </equality>
        </mujoco>
    "#,
    );

    data.step(&model).expect("step should succeed");
    // 1 joint equality = 1 equality row, 1 friction loss DOF = 1 friction row
    assert_eq!(data.ne, 1, "one joint equality");
    assert_eq!(data.nf, 1, "one friction loss DOF");

    // Verify ordering: equality rows first, then friction
    if !data.efc_type.is_empty() {
        assert!(
            matches!(data.efc_type[0], sim_core::ConstraintType::Equality),
            "first row should be equality, got {:?}",
            data.efc_type[0]
        );
        if data.efc_type.len() > 1 {
            assert!(
                matches!(data.efc_type[1], sim_core::ConstraintType::FrictionLoss),
                "second row should be friction loss, got {:?}",
                data.efc_type[1]
            );
        }
    }
}

#[test]
fn test_friction_loss_force_scales_with_floss() {
    // Verify that different frictionloss values produce different constraint forces.
    // The Huber cost has a quadratic core and linear saturation (force = ±floss).
    // Use small floss values so the solver reaches the linear (saturated) zone,
    // where the force magnitude equals the floss parameter.
    let make_model = |frictionloss: f64| {
        format!(
            r#"
            <mujoco model="floss_scaling">
                <option gravity="0 0 0" timestep="0.001" solver="Newton"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <joint name="hinge" type="hinge" axis="0 1 0"
                               stiffness="0" damping="0" frictionloss="{}"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
            frictionloss
        )
    };

    // Small floss values ensure the solver saturates (enters linear Huber zone):
    // threshold = R * floss ≈ 28 * 0.01 = 0.28, while |jar| ≈ 105 at vel=10.
    let model_low = sim_mjcf::load_model(&make_model(0.01)).expect("load");
    let mut data_low = model_low.make_data();
    data_low.qvel[0] = 10.0;
    data_low.step(&model_low).expect("step");

    let model_high = sim_mjcf::load_model(&make_model(0.1)).expect("load");
    let mut data_high = model_high.make_data();
    data_high.qvel[0] = 10.0;
    data_high.step(&model_high).expect("step");

    // Both should have 1 friction loss row
    assert_eq!(data_low.nf, 1, "low: should have 1 friction row");
    assert_eq!(data_high.nf, 1, "high: should have 1 friction row");

    // Get friction force magnitude
    let force_low = data_low.efc_force[data_low.ne].abs();
    let force_high = data_high.efc_force[data_high.ne].abs();

    // In linear zone: force ≈ floss, so force_high ≈ 10 * force_low
    assert!(
        force_high > force_low,
        "Higher floss should produce higher force: low={force_low}, high={force_high}"
    );
    // The ratio should be approximately 10x (within tolerance for solver convergence)
    let ratio = force_high / force_low;
    assert!(
        ratio > 5.0 && ratio < 15.0,
        "Force ratio should be roughly 10x, got {ratio}"
    );
}

#[test]
fn test_compute_qacc_smooth_no_behavioral_change() {
    // Verify that factoring compute_qacc_smooth() out of newton_solve()
    // didn't break Newton convergence.
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="newton_regression">
            <option gravity="0 0 -9.81" timestep="0.002" solver="Newton"/>
            <worldbody>
                <body name="b" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <geom type="plane" size="5 5 0.1"/>
            </worldbody>
        </mujoco>
    "#,
    );

    // Ball falls and contacts the plane
    for _ in 0..500 {
        data.step(&model).expect("step should succeed");
    }

    // Ball should have come to rest near the plane
    let z = data.qpos[2];
    assert!(z < 0.2, "ball should settle near plane, z = {z}");
    assert!(z > -0.01, "ball should not penetrate plane, z = {z}");

    // qacc should be finite
    for i in 0..model.nv {
        assert!(
            data.qacc[i].is_finite(),
            "qacc[{i}] should be finite after 500 steps"
        );
    }
}
