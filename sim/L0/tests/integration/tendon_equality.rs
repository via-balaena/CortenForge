//! §37 Tendon Equality Constraint Tests (AC1–AC13).
//!
//! Verifies MuJoCo-conformant tendon equality constraint behavior:
//! - MJCF `<equality><tendon>` parsing and model builder wiring
//! - Two-tendon polynomial coupling: `(L1-L1_0) = data[0] + P(L2-L2_0)`
//! - Single-tendon mode: `(L1-L1_0) = data[0]`
//! - Analytical constraint Jacobian and velocity computation
//! - Convergence under default solver parameters
//! - Polynomial coupling (linear, quadratic)
//! - Solver compatibility (PGS, CG, Newton)
//! - solref/solimp sensitivity
//! - Inactive constraint bypass
//! - Regression (no effect on models without tendon equality)

use approx::assert_relative_eq;
use sim_core::EqualityType;
use sim_mjcf::load_model;

// ============================================================================
// Helper: two-tendon model
// ============================================================================

/// Base MJCF for two fixed tendons on separate hinge joints.
///
/// T1 wraps J1 (coef=1.0), T2 wraps J2 (coef=1.0).
/// Reference lengths at qpos0: L1_0 = L2_0 = 0 (both joints at 0).
fn two_tendon_mjcf(polycoef: &str, extra_eq_attrs: &str, option_attrs: &str) -> String {
    format!(
        r#"
        <mujoco model="tendon_eq_test">
            <option gravity="0 0 0" timestep="0.001" {option_attrs}/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="J1" type="hinge" axis="0 1 0" damping="0.5"/>
                    <geom type="capsule" size="0.05 0.3" mass="1.0"/>
                </body>
                <body name="b2" pos="1 0 1">
                    <joint name="J2" type="hinge" axis="0 1 0" damping="0.5"/>
                    <geom type="capsule" size="0.05 0.3" mass="1.0"/>
                </body>
            </worldbody>
            <tendon>
                <fixed name="T1"><joint joint="J1" coef="1.0"/></fixed>
                <fixed name="T2"><joint joint="J2" coef="1.0"/></fixed>
            </tendon>
            <equality>
                <tendon tendon1="T1" tendon2="T2" polycoef="{polycoef}" {extra_eq_attrs}/>
            </equality>
        </mujoco>
        "#
    )
}

/// Single-tendon MJCF: one fixed tendon on a hinge joint.
fn single_tendon_mjcf(polycoef: &str, extra_eq_attrs: &str) -> String {
    format!(
        r#"
        <mujoco model="tendon_eq_single">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="J1" type="hinge" axis="0 1 0" damping="0.5"/>
                    <geom type="capsule" size="0.05 0.3" mass="1.0"/>
                </body>
            </worldbody>
            <tendon>
                <fixed name="T1"><joint joint="J1" coef="1.0"/></fixed>
            </tendon>
            <equality>
                <tendon tendon1="T1" polycoef="{polycoef}" {extra_eq_attrs}/>
            </equality>
        </mujoco>
        "#
    )
}

// ============================================================================
// AC1: Loading
// ============================================================================

/// `<equality><tendon tendon1="T1" tendon2="T2"/>` loads without error.
#[test]
fn ac1_loading() {
    let mjcf = two_tendon_mjcf("0 1 0 0 0", "", "");
    let model = load_model(&mjcf).expect("should load");

    assert_eq!(model.neq, 1, "should have one equality constraint");
    assert_eq!(
        model.eq_type[0],
        EqualityType::Tendon,
        "should be Tendon type"
    );

    // obj1id/obj2id should be valid tendon indices
    assert!(model.eq_obj1id[0] < model.ntendon);
    assert!(model.eq_obj2id[0] < model.ntendon);
    assert_ne!(model.eq_obj1id[0], model.eq_obj2id[0]);
}

// ============================================================================
// AC2: Default polycoef — analytical trace
// ============================================================================

/// Two fixed tendons, default polycoef="0 1 0 0 0".
/// Set q1=0.5, q2=0.2. Verify:
///   dif = L2 - L2_0 = 0.2
///   poly_val = 1.0 * 0.2 = 0.2
///   pos_error = (0.5 - 0) - 0 - 0.2 = 0.3
///   deriv = 1.0
///   Jacobian = J_T1 - 1.0 * J_T2 = [1, 0] - [0, 1] = [1, -1]
///   Velocity (qvel=[1.0, 0.5]) = [1,-1] · [1.0, 0.5] = 0.5
#[test]
fn ac2_default_polycoef_analytical() {
    let mjcf = two_tendon_mjcf("0 1 0 0 0", "", "");
    let model = load_model(&mjcf).expect("should load");
    let mut data = model.make_data();

    // Set joint positions
    data.qpos[0] = 0.5; // J1
    data.qpos[1] = 0.2; // J2
    data.qvel[0] = 1.0;
    data.qvel[1] = 0.5;

    data.forward(&model).expect("forward failed");

    // Verify tendon lengths (fixed tendons: L = coef * q)
    assert_relative_eq!(data.ten_length[0], 0.5, epsilon = 1e-12);
    assert_relative_eq!(data.ten_length[1], 0.2, epsilon = 1e-12);

    // Verify reference lengths are 0 (qpos0 = 0)
    assert_relative_eq!(model.tendon_length0[0], 0.0, epsilon = 1e-12);
    assert_relative_eq!(model.tendon_length0[1], 0.0, epsilon = 1e-12);

    // Should have constraint rows (nefc > 0)
    assert!(
        !data.efc_type.is_empty(),
        "should have constraint rows after forward"
    );

    // Find the equality constraint row
    let eq_row = data
        .efc_type
        .iter()
        .position(|t| *t == sim_core::ConstraintType::Equality)
        .expect("should have an equality constraint row");

    // Verify pos_error = 0.3
    assert_relative_eq!(data.efc_pos[eq_row], 0.3, epsilon = 1e-12);

    // Verify Jacobian: [1, -1]
    let nv = model.nv;
    let j_start = eq_row * nv;
    assert_relative_eq!(data.efc_J[j_start], 1.0, epsilon = 1e-12);
    assert_relative_eq!(data.efc_J[j_start + 1], -1.0, epsilon = 1e-12);

    // Verify velocity: [1, -1] · [1.0, 0.5] = 0.5
    assert_relative_eq!(data.efc_vel[eq_row], 0.5, epsilon = 1e-12);
}

// ============================================================================
// AC3: Default polycoef — convergence
// ============================================================================

/// Same model as AC2. Displace q1=0.5, q2=0. After 1000 steps with default
/// solref/solimp, both tendons converge to equal deviation.
#[test]
fn ac3_default_polycoef_convergence() {
    let mjcf = two_tendon_mjcf("0 1 0 0 0", "", "");
    let model = load_model(&mjcf).expect("should load");
    let mut data = model.make_data();

    data.qpos[0] = 0.5;
    data.qpos[1] = 0.0;

    for _ in 0..1000 {
        data.step(&model).expect("step failed");
    }

    // Tendons should converge to equal deviation from reference
    let l1 = data.ten_length[0] - model.tendon_length0[0];
    let l2 = data.ten_length[1] - model.tendon_length0[1];
    let violation = (l1 - l2).abs();

    assert!(
        violation < 0.05,
        "Tendon equality should converge: |L1_dev - L2_dev| = {violation}"
    );
}

// ============================================================================
// AC4: Constant offset
// ============================================================================

/// polycoef="0.1 1 0 0 0". Set q1=0.5, q2=0.2.
///   pos_error = (0.5 - 0) - 0.1 - 1.0 * (0.2 - 0) = 0.2
#[test]
fn ac4_constant_offset() {
    let mjcf = two_tendon_mjcf("0.1 1 0 0 0", "", "");
    let model = load_model(&mjcf).expect("should load");
    let mut data = model.make_data();

    data.qpos[0] = 0.5;
    data.qpos[1] = 0.2;

    data.forward(&model).expect("forward failed");

    let eq_row = data
        .efc_type
        .iter()
        .position(|t| *t == sim_core::ConstraintType::Equality)
        .expect("should have equality row");

    assert_relative_eq!(data.efc_pos[eq_row], 0.2, epsilon = 1e-12);
}

// ============================================================================
// AC5: Single-tendon mode — analytical
// ============================================================================

/// `<tendon tendon1="T1"/>` (no tendon2), polycoef="0.1 0 0 0 0".
/// Set q1=0.4:
///   pos_error = (0.4 - 0) - 0.1 = 0.3
///   Jacobian = J_T1 = [1] (single DOF)
#[test]
fn ac5_single_tendon_analytical() {
    let mjcf = single_tendon_mjcf("0.1 0 0 0 0", "");
    let model = load_model(&mjcf).expect("should load");
    let mut data = model.make_data();

    assert_eq!(model.neq, 1);
    assert_eq!(model.eq_type[0], EqualityType::Tendon);
    // obj2id should be sentinel for single-tendon mode
    assert_eq!(model.eq_obj2id[0], usize::MAX);

    data.qpos[0] = 0.4;

    data.forward(&model).expect("forward failed");

    let eq_row = data
        .efc_type
        .iter()
        .position(|t| *t == sim_core::ConstraintType::Equality)
        .expect("should have equality row");

    // pos_error = 0.4 - 0.1 = 0.3
    assert_relative_eq!(data.efc_pos[eq_row], 0.3, epsilon = 1e-12);

    // Jacobian = [1] (single DOF, fixed tendon with coef=1)
    let nv = model.nv;
    let j_start = eq_row * nv;
    assert_relative_eq!(data.efc_J[j_start], 1.0, epsilon = 1e-12);
}

// ============================================================================
// AC6: Single-tendon mode — convergence
// ============================================================================

/// Same model as AC5. After 1000 steps from q1=0, verify convergence:
/// |L1 - L1_0 - 0.1| < 0.05.
#[test]
fn ac6_single_tendon_convergence() {
    let mjcf = single_tendon_mjcf("0.1 0 0 0 0", "");
    let model = load_model(&mjcf).expect("should load");
    let mut data = model.make_data();

    data.qpos[0] = 0.0;

    for _ in 0..1000 {
        data.step(&model).expect("step failed");
    }

    let l1_dev = data.ten_length[0] - model.tendon_length0[0];
    let violation = (l1_dev - 0.1).abs();

    assert!(
        violation < 0.05,
        "Single-tendon should converge to offset 0.1: |L1_dev - 0.1| = {violation}"
    );
}

// ============================================================================
// AC7: Polynomial coupling — linear half-ratio
// ============================================================================

/// polycoef="0 0.5 0 0 0". Test 5 configurations (q2 in {-0.4,-0.2,0,0.2,0.4},
/// q1=0). Verify pos_error = (L1-L1_0) - 0.5*(L2-L2_0).
#[test]
fn ac7_half_ratio_polynomial() {
    let mjcf = two_tendon_mjcf("0 0.5 0 0 0", "", "");
    let model = load_model(&mjcf).expect("should load");

    for &q2 in &[-0.4, -0.2, 0.0, 0.2, 0.4] {
        let mut data = model.make_data();
        data.qpos[0] = 0.0; // q1 = 0
        data.qpos[1] = q2;

        data.forward(&model).expect("forward failed");

        let eq_row = data
            .efc_type
            .iter()
            .position(|t| *t == sim_core::ConstraintType::Equality)
            .expect("should have equality row");

        // pos_error = (L1 - L1_0) - 0.5 * (L2 - L2_0)
        // L1 = coef * q1 = 0, L1_0 = 0, L2 = q2, L2_0 = 0
        let expected = 0.0 - 0.5 * q2;
        assert_relative_eq!(data.efc_pos[eq_row], expected, epsilon = 1e-12,);
    }
}

// ============================================================================
// AC8: Quadratic polynomial
// ============================================================================

/// polycoef="0 1 0.1 0 0". Test 5 configurations (q2 in {-0.4,-0.2,0,0.2,0.4},
/// q1=0). Verify pos_error = (L1-L1_0) - (1.0*dif + 0.1*dif^2).
#[test]
fn ac8_quadratic_polynomial() {
    let mjcf = two_tendon_mjcf("0 1 0.1 0 0", "", "");
    let model = load_model(&mjcf).expect("should load");

    for &q2 in &[-0.4, -0.2, 0.0, 0.2, 0.4] {
        let mut data = model.make_data();
        data.qpos[0] = 0.0;
        data.qpos[1] = q2;

        data.forward(&model).expect("forward failed");

        let eq_row = data
            .efc_type
            .iter()
            .position(|t| *t == sim_core::ConstraintType::Equality)
            .expect("should have equality row");

        let dif = q2; // L2 - L2_0 = q2 - 0
        let poly_val = 1.0 * dif + 0.1 * dif * dif;
        let expected = 0.0 - poly_val; // (L1-L1_0) - poly
        assert_relative_eq!(data.efc_pos[eq_row], expected, epsilon = 1e-8,);
    }
}

// ============================================================================
// AC9: Solver compatibility (PGS, CG, Newton)
// ============================================================================

/// Works with PGS, CG, and Newton solvers. After 500 steps, constraint
/// violation < 0.05 for all three. Verify no NaN/Inf in efc_force.
#[test]
fn ac9_solver_compatibility() {
    for solver in &["PGS", "CG", "Newton"] {
        let option_attrs = format!("solver=\"{solver}\"");
        let mjcf = two_tendon_mjcf("0 1 0 0 0", "", &option_attrs);
        let model = load_model(&mjcf).unwrap_or_else(|e| {
            panic!("should load with solver {solver}: {e}");
        });
        let mut data = model.make_data();

        data.qpos[0] = 0.5;
        data.qpos[1] = 0.0;

        for step in 0..500 {
            data.step(&model).unwrap_or_else(|e| {
                panic!("step {step} failed with solver {solver}: {e}");
            });

            // Verify no NaN/Inf in efc_force
            for (i, &f) in data.efc_force.iter().enumerate() {
                assert!(
                    f.is_finite(),
                    "efc_force[{i}] is not finite ({f}) at step {step} with solver {solver}"
                );
            }
        }

        let l1 = data.ten_length[0] - model.tendon_length0[0];
        let l2 = data.ten_length[1] - model.tendon_length0[1];
        let violation = (l1 - l2).abs();

        assert!(
            violation < 0.05,
            "Tendon equality should converge with {solver}: violation = {violation}"
        );
    }
}

// ============================================================================
// AC10: solref/solimp sensitivity
// ============================================================================

/// Two simulations, identical model except:
/// - Stiff: solref="0.02 1.0" (default)
/// - Soft: solref="0.1 1.0"
///
/// After 200 steps from displaced state, soft has larger violation.
#[test]
fn ac10_solref_sensitivity() {
    let mjcf_stiff = two_tendon_mjcf("0 1 0 0 0", "solref=\"0.02 1.0\"", "");
    let mjcf_soft = two_tendon_mjcf("0 1 0 0 0", "solref=\"0.1 1.0\"", "");

    let model_stiff = load_model(&mjcf_stiff).expect("should load stiff");
    let model_soft = load_model(&mjcf_soft).expect("should load soft");

    let mut data_stiff = model_stiff.make_data();
    let mut data_soft = model_soft.make_data();

    // Identical initial displacement
    data_stiff.qpos[0] = 0.5;
    data_stiff.qpos[1] = 0.0;
    data_soft.qpos[0] = 0.5;
    data_soft.qpos[1] = 0.0;

    for _ in 0..200 {
        data_stiff.step(&model_stiff).expect("stiff step failed");
        data_soft.step(&model_soft).expect("soft step failed");
    }

    let violation_stiff = (data_stiff.ten_length[0]
        - model_stiff.tendon_length0[0]
        - (data_stiff.ten_length[1] - model_stiff.tendon_length0[1]))
        .abs();

    let violation_soft = (data_soft.ten_length[0]
        - model_soft.tendon_length0[0]
        - (data_soft.ten_length[1] - model_soft.tendon_length0[1]))
        .abs();

    let ratio = violation_soft / violation_stiff.max(1e-15);

    assert!(
        ratio > 3.0,
        "Soft constraint should have larger violation: stiff={violation_stiff}, soft={violation_soft}, ratio={ratio}"
    );
}

// ============================================================================
// AC11: Inactive constraint
// ============================================================================

/// active="false" produces no constraint rows (nefc unchanged).
#[test]
fn ac11_inactive_constraint() {
    // Model with inactive tendon equality
    let mjcf_inactive = two_tendon_mjcf("0 1 0 0 0", "active=\"false\"", "");
    // Model with no tendon equality at all
    let mjcf_none = r#"
        <mujoco model="no_tendon_eq">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="J1" type="hinge" axis="0 1 0" damping="0.5"/>
                    <geom type="capsule" size="0.05 0.3" mass="1.0"/>
                </body>
                <body name="b2" pos="1 0 1">
                    <joint name="J2" type="hinge" axis="0 1 0" damping="0.5"/>
                    <geom type="capsule" size="0.05 0.3" mass="1.0"/>
                </body>
            </worldbody>
            <tendon>
                <fixed name="T1"><joint joint="J1" coef="1.0"/></fixed>
                <fixed name="T2"><joint joint="J2" coef="1.0"/></fixed>
            </tendon>
        </mujoco>
    "#;

    let model_inactive = load_model(&mjcf_inactive).expect("should load inactive");
    let model_none = load_model(mjcf_none).expect("should load none");

    // Inactive model has the constraint in the model but marked inactive
    assert_eq!(model_inactive.neq, 1);
    assert!(!model_inactive.eq_active[0]);

    let mut data_inactive = model_inactive.make_data();
    let mut data_none = model_none.make_data();

    data_inactive.qpos[0] = 0.5;
    data_inactive.qpos[1] = 0.0;
    data_none.qpos[0] = 0.5;
    data_none.qpos[1] = 0.0;

    data_inactive
        .forward(&model_inactive)
        .expect("forward failed");
    data_none.forward(&model_none).expect("forward failed");

    // Same number of constraint rows (inactive contributes zero rows)
    assert_eq!(
        data_inactive.efc_type.len(),
        data_none.efc_type.len(),
        "inactive constraint should produce no extra rows"
    );

    // efc_force should be identical
    assert_eq!(data_inactive.efc_force.len(), data_none.efc_force.len());
    for i in 0..data_inactive.efc_force.len() {
        assert_relative_eq!(
            data_inactive.efc_force[i],
            data_none.efc_force[i],
            epsilon = 1e-15
        );
    }
}

// ============================================================================
// AC12: Regression — models without tendon equality
// ============================================================================

/// Models without tendon equality produce identical results to baseline.
#[test]
fn ac12_regression_no_tendon_equality() {
    // A model with joint equality but no tendon equality
    let mjcf = r#"
        <mujoco model="regression">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="link1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0" damping="1"/>
                    <geom type="capsule" size="0.05 0.3" mass="1.0"/>
                    <body name="link2" pos="0 0 -0.7">
                        <joint name="j2" type="hinge" axis="0 1 0" damping="1"/>
                        <geom type="capsule" size="0.05 0.3" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
            <equality>
                <joint joint1="j1" joint2="j2" polycoef="0 1" solref="0.1 1.0"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    data.qpos[0] = 0.5;

    // Just verify it runs without errors and the joint equality still works
    for _ in 0..500 {
        data.step(&model).expect("step failed");
    }

    // Joint equality should still function
    let q1 = data.qpos[0];
    let q2 = data.qpos[1];
    let diff = (q2 - q1).abs();

    assert!(
        diff < 0.15,
        "Joint equality regression: q1={q1}, q2={q2}, diff={diff}"
    );
}

// ============================================================================
// AC13: MuJoCo reference — constraint violation trajectory
// ============================================================================

/// Same two-tendon model as AC2 with default polycoef. Displace q1=0.3, q2=0.
/// Run 100 steps. Verify constraint violation decreases monotonically (on average)
/// and efc_force is well-behaved (finite, reasonable magnitude).
#[test]
fn ac13_mujoco_reference_trajectory() {
    let mjcf = two_tendon_mjcf("0 1 0 0 0", "", "");
    let model = load_model(&mjcf).expect("should load");
    let mut data = model.make_data();

    data.qpos[0] = 0.3;
    data.qpos[1] = 0.0;

    let mut violations = Vec::with_capacity(100);

    for step in 0..100 {
        data.step(&model).expect("step failed");

        let l1_dev = data.ten_length[0] - model.tendon_length0[0];
        let l2_dev = data.ten_length[1] - model.tendon_length0[1];
        let violation = (l1_dev - l2_dev).abs();
        violations.push(violation);

        // Verify efc_force is finite at every step
        for (i, &f) in data.efc_force.iter().enumerate() {
            assert!(
                f.is_finite(),
                "efc_force[{i}] not finite at step {step}: {f}"
            );
        }
    }

    // Violation should decrease overall: last 10 avg < first 10 avg
    let first_10_avg: f64 = violations[..10].iter().sum::<f64>() / 10.0;
    let last_10_avg: f64 = violations[90..].iter().sum::<f64>() / 10.0;

    assert!(
        last_10_avg < first_10_avg,
        "Constraint violation should decrease: first_10_avg={first_10_avg}, last_10_avg={last_10_avg}"
    );

    // Final violation should be small
    let final_violation = *violations.last().unwrap();
    assert!(
        final_violation < 0.1,
        "Final violation should be small: {final_violation}"
    );
}

// ============================================================================
// Additional: Jacobian derivative for two-tendon quadratic
// ============================================================================

/// Verify the Jacobian accounts for polynomial derivative correctly.
/// polycoef="0 1 0.5 0 0", q1=0, q2=0.3:
///   dif = 0.3
///   deriv = 1.0 + 2*0.5*0.3 = 1.3
///   J = [1, -1.3]
#[test]
fn ac8b_quadratic_jacobian_derivative() {
    let mjcf = two_tendon_mjcf("0 1 0.5 0 0", "", "");
    let model = load_model(&mjcf).expect("should load");
    let mut data = model.make_data();

    data.qpos[0] = 0.0;
    data.qpos[1] = 0.3;

    data.forward(&model).expect("forward failed");

    let eq_row = data
        .efc_type
        .iter()
        .position(|t| *t == sim_core::ConstraintType::Equality)
        .expect("should have equality row");

    let nv = model.nv;
    let j_start = eq_row * nv;

    // dif = L2 - L2_0 = 0.3 - 0 = 0.3
    // deriv = 1.0 + 2*0.5*0.3 = 1.3
    // J = J_T1 - deriv * J_T2 = [1, 0] - 1.3 * [0, 1] = [1, -1.3]
    assert_relative_eq!(data.efc_J[j_start], 1.0, epsilon = 1e-12);
    assert_relative_eq!(data.efc_J[j_start + 1], -1.3, epsilon = 1e-12);

    // pos_error = (0 - 0) - (1.0*0.3 + 0.5*0.09) = -0.345
    let expected_pos = -(1.0 * 0.3 + 0.5 * 0.3 * 0.3);
    assert_relative_eq!(data.efc_pos[eq_row], expected_pos, epsilon = 1e-12);
}
