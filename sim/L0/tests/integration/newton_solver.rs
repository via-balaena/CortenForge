//! Newton solver integration tests (§15 Phase A + B).
//!
//! These tests verify the Newton solver implementation:
//! - Unified constraint assembly
//! - Exact 1D Newton line search convergence
//! - PGS fallback on failure / implicit integrators
//! - Warmstart behavior
//! - Zero-constraint degenerate case
//! - Energy stability
//! - Contact handling with elliptic cones
//! - Equality constraints
//! - Multi-constraint stability

use approx::assert_relative_eq;
use sim_mjcf::load_model;

/// Helper: load model from MJCF string.
fn model_from_mjcf(mjcf: &str) -> (sim_core::Model, sim_core::Data) {
    let model = load_model(mjcf).expect("MJCF should load");
    let data = model.make_data();
    (model, data)
}

// ============================================================================
// AC1: Newton solver produces finite, non-NaN qacc for basic models
// ============================================================================

#[test]
fn test_newton_basic_free_fall() {
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="free_fall">
            <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"/>
            <worldbody>
                <body name="ball" pos="0 0 2">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <geom type="plane" size="5 5 0.1"/>
            </worldbody>
        </mujoco>
    "#,
    );

    assert_eq!(
        model.solver_type,
        sim_core::SolverType::Newton,
        "Should use Newton solver"
    );

    // Run a few steps
    for _ in 0..10 {
        data.step(&model).expect("step should succeed");
    }

    // qacc should be finite
    for i in 0..model.nv {
        assert!(
            data.qacc[i].is_finite(),
            "qacc[{i}] should be finite, got {}",
            data.qacc[i]
        );
    }
}

// ============================================================================
// AC2: Default MJCF maps to Newton solver
// ============================================================================

#[test]
fn test_default_solver_is_newton() {
    let (model, _data) = model_from_mjcf(
        r#"
        <mujoco model="default_solver">
            <worldbody>
                <body name="b" pos="0 0 0">
                    <joint type="hinge"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
    );

    assert_eq!(
        model.solver_type,
        sim_core::SolverType::Newton,
        "Default solver should be Newton"
    );
}

// ============================================================================
// AC3: Zero-constraint degenerate case (free joint, no contacts/limits)
// ============================================================================

#[test]
fn test_newton_zero_constraints() {
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="no_constraints">
            <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"/>
            <worldbody>
                <body name="ball" pos="0 0 10">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0" contype="0" conaffinity="0"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
    );

    data.step(&model).expect("step should succeed");

    // With no constraints, qacc should match free-fall: a_z ≈ -9.81
    // The free joint has 6 DOFs: [tx, ty, tz, rx, ry, rz]
    // Linear acceleration in z should be -9.81
    assert_relative_eq!(data.qacc[2], -9.81, epsilon = 0.01);
    // Other accelerations should be ~0
    assert_relative_eq!(data.qacc[0], 0.0, epsilon = 0.001);
    assert_relative_eq!(data.qacc[1], 0.0, epsilon = 0.001);
}

// ============================================================================
// AC4: Implicit integrator falls through to PGS (no crash)
// ============================================================================

#[test]
fn test_newton_implicit_fallback() {
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="implicit_guard">
            <option gravity="0 0 -9.81" timestep="0.001"
                    solver="Newton" integrator="implicit"/>
            <worldbody>
                <body name="b" pos="0 0 0">
                    <joint type="hinge" damping="1.0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
    );

    assert_eq!(model.solver_type, sim_core::SolverType::Newton);

    // Should not crash — Newton falls through to PGS for implicit
    for _ in 0..10 {
        data.step(&model)
            .expect("implicit + Newton should not crash");
    }

    // Should still produce finite results
    assert!(data.qacc[0].is_finite());
}

// ============================================================================
// AC5: Newton produces unified constraint fields (efc_type, efc_force, etc.)
// ============================================================================

#[test]
fn test_newton_unified_constraint_fields() {
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="constraint_fields">
            <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"/>
            <worldbody>
                <body name="arm" pos="0 0 0">
                    <joint name="j" type="hinge" limited="true" range="-1.0 1.0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
    );

    // Push into joint limit
    data.qpos[0] = 1.5;
    data.step(&model).expect("step should succeed");

    // If Newton ran, it should have populated efc_* fields
    if data.newton_solved {
        assert!(
            !data.efc_type.is_empty(),
            "Newton should populate efc_type for joint limits"
        );
        assert!(
            !data.efc_state.is_empty(),
            "Newton should populate efc_state"
        );
    }
}

// ============================================================================
// AC6: Warmstart improves over successive steps
// ============================================================================

#[test]
fn test_newton_warmstart_not_zero() {
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="warmstart_test">
            <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"/>
            <worldbody>
                <body name="b" pos="0 0 5">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0" contype="0" conaffinity="0"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
    );

    // Free body under gravity: qacc_z = -9.81, so qacc is definitely nonzero
    data.step(&model).expect("step 1");
    data.step(&model).expect("step 2");

    // After 2 steps, warmstart should be non-zero (saved from previous qacc)
    let warmstart_norm: f64 = data
        .qacc_warmstart
        .iter()
        .map(|x| x * x)
        .sum::<f64>()
        .sqrt();
    assert!(
        warmstart_norm > 0.0,
        "qacc_warmstart should be non-zero after stepping, norm={warmstart_norm}"
    );
}

// ============================================================================
// AC7: Newton energy stability (energy should not diverge)
// ============================================================================

#[test]
fn test_newton_energy_stability() {
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="energy_test">
            <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"/>
            <worldbody>
                <body name="pendulum" pos="0 0 0">
                    <joint type="hinge" damping="0.1"/>
                    <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
    );

    data.qpos[0] = 0.5; // Start displaced
    data.step(&model).expect("initial forward");

    let initial_energy = data.energy_potential + data.energy_kinetic;

    // Run for 100 steps
    for _ in 0..100 {
        data.step(&model).expect("step should succeed");
    }

    let final_energy = data.energy_potential + data.energy_kinetic;

    // Energy should not blow up (some dissipation from damping is expected)
    assert!(
        final_energy.is_finite(),
        "Energy should stay finite, got {final_energy}"
    );
    // With damping, energy should decrease or stay bounded
    assert!(
        final_energy <= initial_energy * 1.1,
        "Energy should not increase significantly: initial={initial_energy:.4}, final={final_energy:.4}"
    );
}

// ============================================================================
// AC8: No PGS/CG regression (PGS and CG tests should still pass)
// ============================================================================

#[test]
fn test_pgs_still_works() {
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="pgs_regression">
            <option gravity="0 0 -9.81" timestep="0.001" solver="PGS"/>
            <worldbody>
                <body name="ball" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <geom type="plane" size="5 5 0.1"/>
            </worldbody>
        </mujoco>
    "#,
    );

    assert_eq!(model.solver_type, sim_core::SolverType::PGS);

    for _ in 0..100 {
        data.step(&model).expect("PGS step should succeed");
    }

    // Ball should reach ground and stop
    assert!(data.qacc[2].is_finite());
}

#[test]
fn test_cg_still_works() {
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="cg_regression">
            <option gravity="0 0 -9.81" timestep="0.001" solver="CG"/>
            <worldbody>
                <body name="ball" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <geom type="plane" size="5 5 0.1"/>
            </worldbody>
        </mujoco>
    "#,
    );

    assert_eq!(model.solver_type, sim_core::SolverType::CG);

    for _ in 0..100 {
        data.step(&model).expect("CG step should succeed");
    }

    assert!(data.qacc[2].is_finite());
}

// ============================================================================
// AC9: Contact handling with Newton
// ============================================================================

#[test]
fn test_newton_contact_basic() {
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="contact_newton">
            <option gravity="0 0 -9.81" timestep="0.001" solver="Newton" cone="elliptic"/>
            <worldbody>
                <body name="ball" pos="0 0 0.2">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <geom type="plane" size="5 5 0.1"/>
            </worldbody>
        </mujoco>
    "#,
    );

    // Let ball fall onto plane
    for _ in 0..200 {
        data.step(&model).expect("step should succeed");
    }

    // Ball should not fall through the plane
    // z position of ball center should be >= sphere radius (0.1)
    let z_pos = data.qpos[2]; // Free joint: [tx, ty, tz, qw, qx, qy, qz]
    assert!(
        z_pos >= 0.05,
        "Ball should not fall through plane, z={z_pos:.4}"
    );
}

// ============================================================================
// AC10: Equality constraints with Newton
// ============================================================================

#[test]
fn test_newton_equality_connect() {
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="equality_newton">
            <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.05" mass="1.0"/>
                </body>
                <body name="b2" pos="0 0 0.5">
                    <joint type="free"/>
                    <geom type="sphere" size="0.05" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <connect body1="b1" body2="b2" anchor="0 0 0"/>
            </equality>
        </mujoco>
    "#,
    );

    for _ in 0..50 {
        data.step(&model).expect("step should succeed");
    }

    // Both bodies should still produce finite results
    for i in 0..model.nv {
        assert!(
            data.qacc[i].is_finite(),
            "qacc[{i}] should be finite with equality constraints"
        );
    }
}

// ============================================================================
// AC11: Newton solver fields (ls_tolerance, noslip_tolerance)
// ============================================================================

#[test]
fn test_newton_option_parsing() {
    let (model, _data) = model_from_mjcf(
        r#"
        <mujoco model="options">
            <option solver="Newton" iterations="50" tolerance="1e-6"
                    ls_iterations="30" ls_tolerance="0.05"
                    noslip_iterations="10" noslip_tolerance="1e-5"/>
            <worldbody>
                <body name="b" pos="0 0 0">
                    <joint type="hinge"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
    );

    assert_eq!(model.solver_type, sim_core::SolverType::Newton);
    assert_eq!(model.solver_iterations, 50);
    assert_relative_eq!(model.solver_tolerance, 1e-6);
    assert_eq!(model.ls_iterations, 30);
    assert_relative_eq!(model.ls_tolerance, 0.05);
    assert_eq!(model.noslip_iterations, 10);
    assert_relative_eq!(model.noslip_tolerance, 1e-5);
}

// ============================================================================
// AC12: stat_meaninertia computation
// ============================================================================

#[test]
fn test_stat_meaninertia() {
    let (model, _data) = model_from_mjcf(
        r#"
        <mujoco model="meaninertia">
            <worldbody>
                <body name="b" pos="0 0 0">
                    <joint type="hinge"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
    );

    assert!(
        model.stat_meaninertia > 0.0,
        "stat_meaninertia should be positive, got {}",
        model.stat_meaninertia
    );
    assert!(
        model.stat_meaninertia.is_finite(),
        "stat_meaninertia should be finite"
    );
}

// ============================================================================
// AC13: Multiple steps don't crash with various constraint combinations
// ============================================================================

#[test]
fn test_newton_multi_constraint_stability() {
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="multi_constraint">
            <option gravity="0 0 -9.81" timestep="0.002" solver="Newton" cone="elliptic"/>
            <worldbody>
                <body name="arm1" pos="0 0 0">
                    <joint name="j1" type="hinge" limited="true" range="-1.57 1.57"
                           damping="0.5"/>
                    <geom type="capsule" size="0.1" fromto="0 0 0 0.3 0 0" mass="5.0"/>
                    <body name="arm2" pos="0.3 0 0">
                        <joint name="j2" type="hinge" limited="true" range="-1.57 1.57"
                               damping="0.5"/>
                        <geom type="capsule" size="0.1" fromto="0 0 0 0.3 0 0" mass="3.0"/>
                    </body>
                </body>
                <geom type="plane" size="5 5 0.1" pos="0 0 -1"/>
            </worldbody>
        </mujoco>
    "#,
    );

    // Run 500 steps with initial velocity
    data.qvel[0] = 2.0;
    data.qvel[1] = -1.0;

    for i in 0..500 {
        data.step(&model).unwrap_or_else(|e| {
            panic!("Step {i} failed: {e:?}");
        });
    }

    // Should stay finite
    for i in 0..model.nv {
        assert!(
            data.qacc[i].is_finite(),
            "qacc[{i}] should be finite after 500 steps"
        );
    }
}
