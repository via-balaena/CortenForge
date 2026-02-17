//! Unified PGS/CG solver tests (§29).
//!
//! These tests verify the unified constraint solver implementations:
//! - ne/nf constraint ordering bookkeeping
//! - Correct constraint row ordering (equality → friction → limits+contacts)
//! - PGS cost guard monotonicity (dual cost ≤ 0 after convergence)

use nalgebra::DMatrix;
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

// ============================================================================
// PGS Cost Guard Monotonicity (AC4)
// ============================================================================

/// Compute dual cost: 0.5 * f^T * AR * f + f^T * b
/// where AR = J * M_inv * J^T + diag(R)
fn compute_dual_cost(
    efc_j: &DMatrix<f64>,
    q_m: &DMatrix<f64>,
    efc_r: &[f64],
    efc_force: &[f64],
    efc_b: &[f64],
) -> f64 {
    let nefc = efc_force.len();
    let nv = q_m.nrows();
    if nefc == 0 || nv == 0 {
        return 0.0;
    }

    // Compute M_inv (dense inverse for small systems)
    let m_inv = match q_m.clone().try_inverse() {
        Some(inv) => inv,
        None => return 0.0,
    };

    // AR = J * M_inv * J^T + diag(R)
    let j = efc_j;
    let j_minv = j * &m_inv;
    let mut ar = &j_minv * j.transpose();
    for i in 0..nefc {
        ar[(i, i)] += efc_r[i];
    }

    // Dual cost = 0.5 * f^T * AR * f + f^T * b
    let mut cost = 0.0;
    for i in 0..nefc {
        let mut ar_f_i = 0.0;
        for j_idx in 0..nefc {
            ar_f_i += ar[(i, j_idx)] * efc_force[j_idx];
        }
        cost += 0.5 * efc_force[i] * ar_f_i + efc_force[i] * efc_b[i];
    }
    cost
}

#[test]
fn test_pgs_cost_guard_dual_cost_nonpositive() {
    // PGS cost guard ensures dual cost never increases.
    // Since cold start has cost=0, final cost must be <= 0.
    // Use a sphere-on-plane contact with friction (produces elliptic contact rows).
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="pgs_cost_guard">
            <option gravity="0 0 -9.81" timestep="0.002" solver="PGS" iterations="100"/>
            <worldbody>
                <body name="ball" pos="0 0 0.15">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0" friction="1.0 0.005 0.0001"/>
                </body>
                <geom type="plane" size="5 5 0.1" friction="1.0 0.005 0.0001"/>
            </worldbody>
        </mujoco>
    "#,
    );

    // Give the ball some lateral velocity to engage friction cone
    data.qvel[0] = 2.0; // x-velocity

    // Step once to generate contacts
    data.step(&model).expect("step should succeed");

    let nefc = data.efc_type.len();
    if nefc == 0 {
        // No contacts generated (ball not yet in contact) — skip cost check
        return;
    }

    // Compute dual cost from public efc data
    let dual_cost = compute_dual_cost(
        &data.efc_J,
        &data.qM,
        &data.efc_R,
        data.efc_force.as_slice(),
        data.efc_b.as_slice(),
    );

    // The cost guard ensures the dual cost never exceeds 0 (cold start baseline).
    // Allow small tolerance for accumulated per-row guard tolerance (1e-10 per row per iter).
    let tolerance = nefc as f64 * model.solver_iterations as f64 * 1e-10;
    assert!(
        dual_cost <= tolerance,
        "PGS dual cost should be ≤ 0 (cost guard), got {dual_cost:.6e} (tolerance: {tolerance:.6e})"
    );

    // Also verify constraint force feasibility:
    // - Normal forces >= 0 (for contacts and limits)
    // - Friction forces within bounds
    for i in 0..nefc {
        let force = data.efc_force[i];
        match data.efc_type[i] {
            sim_core::ConstraintType::LimitJoint
            | sim_core::ConstraintType::LimitTendon
            | sim_core::ConstraintType::ContactNonElliptic => {
                assert!(
                    force >= -1e-12,
                    "unilateral force[{i}] should be >= 0, got {force}"
                );
            }
            sim_core::ConstraintType::FrictionLoss => {
                let fl = data.efc_floss[i];
                assert!(
                    force >= -fl - 1e-12 && force <= fl + 1e-12,
                    "friction force[{i}] should be in [-{fl}, {fl}], got {force}"
                );
            }
            _ => {}
        }
    }
}

#[test]
fn test_pgs_cost_guard_with_friction_loss_and_contacts() {
    // Combined friction loss + contact constraints exercise both scalar and
    // group cost guards simultaneously.
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="pgs_combined">
            <option gravity="0 0 -9.81" timestep="0.002" solver="PGS" iterations="50"/>
            <worldbody>
                <body name="arm" pos="0 0 0.5">
                    <joint type="hinge" axis="0 1 0" frictionloss="0.5"/>
                    <geom type="capsule" size="0.05 0.3" mass="1.0" friction="0.8 0.003 0.0001"/>
                    <body name="ball" pos="0 0 -0.4">
                        <joint type="hinge" axis="0 1 0" frictionloss="0.3"/>
                        <geom type="sphere" size="0.08" mass="0.5" friction="0.8 0.003 0.0001"/>
                    </body>
                </body>
                <geom type="plane" size="5 5 0.1" friction="0.8 0.003 0.0001"/>
            </worldbody>
        </mujoco>
    "#,
    );

    // Step multiple times to let the arm fall and make contact
    for step in 0..20 {
        data.step(&model).expect("step should succeed");

        let nefc = data.efc_type.len();
        if nefc > 0 {
            let dual_cost = compute_dual_cost(
                &data.efc_J,
                &data.qM,
                &data.efc_R,
                data.efc_force.as_slice(),
                data.efc_b.as_slice(),
            );

            let tolerance = nefc as f64 * model.solver_iterations as f64 * 1e-10;
            assert!(
                dual_cost <= tolerance,
                "Step {step}: PGS dual cost should be ≤ 0, got {dual_cost:.6e}"
            );
        }
    }

    // Friction loss rows should be present
    assert!(
        data.nf >= 2,
        "should have >= 2 friction loss rows, got {}",
        data.nf
    );
}

// ============================================================================
// Per-DOF Friction Loss solref/solimp Tests (#28)
// ============================================================================

#[test]
fn test_per_dof_solreffriction_produces_different_params() {
    // Custom solreffriction should produce different efc_solref than default.
    let (model_default, mut data_default) = model_from_mjcf(
        r#"
        <mujoco model="solref_default">
            <option gravity="0 0 0" timestep="0.001" solver="PGS"/>
            <worldbody>
                <body name="b" pos="0 0 0">
                    <joint type="hinge" axis="0 1 0" frictionloss="1.0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
    );

    let (model_custom, mut data_custom) = model_from_mjcf(
        r#"
        <mujoco model="solref_custom">
            <option gravity="0 0 0" timestep="0.001" solver="PGS"/>
            <worldbody>
                <body name="b" pos="0 0 0">
                    <joint type="hinge" axis="0 1 0" frictionloss="1.0"
                           solreffriction="0.05 0.5"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
    );

    // Verify Model-level per-DOF fields
    assert_eq!(model_default.dof_solref[0], [0.02, 1.0], "default solref");
    assert_eq!(model_custom.dof_solref[0], [0.05, 0.5], "custom solref");

    // Step to generate constraint rows
    data_default.qvel[0] = 5.0;
    data_default.step(&model_default).expect("step");
    data_custom.qvel[0] = 5.0;
    data_custom.step(&model_custom).expect("step");

    // Both should have 1 friction loss row
    assert_eq!(data_default.nf, 1);
    assert_eq!(data_custom.nf, 1);

    // The efc_solref should differ for the friction row (at index ne)
    let fri_idx = data_default.ne;
    assert_eq!(data_default.efc_solref[fri_idx], [0.02, 1.0]);
    assert_eq!(data_custom.efc_solref[fri_idx], [0.05, 0.5]);
}

#[test]
fn test_ball_joint_all_dofs_get_same_solref() {
    // Ball joint: 3 DOFs, all should get identical dof_solref from parent joint.
    let (model, _data) = model_from_mjcf(
        r#"
        <mujoco model="ball_solref">
            <option gravity="0 0 0" timestep="0.001" solver="PGS"/>
            <worldbody>
                <body name="b" pos="0 0 0">
                    <joint type="ball" frictionloss="1.0"
                           solreffriction="0.1 0.8" solimpfriction="0.85 0.9 0.002 0.6 3.0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
    );

    // Ball joint = 3 DOFs, all should have identical params
    assert_eq!(model.nv, 3, "ball joint should have 3 DOFs");
    for i in 0..3 {
        assert_eq!(
            model.dof_solref[i],
            [0.1, 0.8],
            "DOF {i} solref should match joint"
        );
        assert_eq!(
            model.dof_solimp[i],
            [0.85, 0.9, 0.002, 0.6, 3.0],
            "DOF {i} solimp should match joint"
        );
    }
}

#[test]
fn test_solreffriction_default_inheritance() {
    // Default class solreffriction should apply to joints without explicit override.
    let (model, _data) = model_from_mjcf(
        r#"
        <mujoco model="default_inheritance">
            <option gravity="0 0 0" timestep="0.001" solver="PGS"/>
            <default>
                <joint solreffriction="0.04 0.7"/>
            </default>
            <worldbody>
                <body name="a" pos="0 0 0">
                    <joint name="j1" type="hinge" axis="0 1 0" frictionloss="1.0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b" pos="0 0.5 0">
                        <joint name="j2" type="hinge" axis="0 1 0" frictionloss="0.5"
                               solreffriction="0.08 0.3"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#,
    );

    // j1 inherits from default, j2 overrides
    assert_eq!(
        model.dof_solref[0],
        [0.04, 0.7],
        "j1 should inherit from default"
    );
    assert_eq!(
        model.dof_solref[1],
        [0.08, 0.3],
        "j2 should use explicit override"
    );
}

#[test]
fn test_tendon_solreffriction() {
    // Tendon with custom solreffriction should produce different parameters.
    let (model, _data) = model_from_mjcf(
        r#"
        <mujoco model="tendon_solref">
            <option gravity="0 0 0" timestep="0.001" solver="PGS"/>
            <worldbody>
                <body name="a" pos="0 0 0">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b" pos="0 0.5 0">
                        <joint name="j2" type="hinge" axis="0 1 0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t1">
                    <joint joint="j1" coef="1.0"/>
                    <joint joint="j2" coef="-1.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#,
    );

    // Verify default tendon friction solver params
    assert_eq!(
        model.tendon_solref_fri[0],
        [0.02, 1.0],
        "default tendon solref_fri"
    );

    // Now test with custom
    let (model2, mut data2) = model_from_mjcf(
        r#"
        <mujoco model="tendon_solref_custom">
            <option gravity="0 0 0" timestep="0.001" solver="PGS"/>
            <worldbody>
                <body name="a" pos="0 0 0">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b" pos="0 0.5 0">
                        <joint name="j2" type="hinge" axis="0 1 0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t1" frictionloss="2.0"
                       solreffriction="0.06 0.4" solimpfriction="0.8 0.9 0.003 0.4 2.5">
                    <joint joint="j1" coef="1.0"/>
                    <joint joint="j2" coef="-1.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#,
    );

    assert_eq!(
        model2.tendon_solref_fri[0],
        [0.06, 0.4],
        "custom tendon solref_fri"
    );
    assert_eq!(
        model2.tendon_solimp_fri[0],
        [0.8, 0.9, 0.003, 0.4, 2.5],
        "custom tendon solimp_fri"
    );

    // Step and verify efc_solref for tendon friction row
    data2.qvel[0] = 3.0;
    data2.step(&model2).expect("step");
    assert_eq!(data2.nf, 1, "should have 1 tendon friction row");
    let fri_idx = data2.ne;
    assert_eq!(
        data2.efc_solref[fri_idx],
        [0.06, 0.4],
        "efc_solref should match tendon"
    );
}
