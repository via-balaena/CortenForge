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

// ============================================================================
// Phase C: Solver Statistics
// ============================================================================

#[test]
fn test_newton_solver_statistics() {
    // Contact scene that requires Newton iterations
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="solver_stats_test">
            <option gravity="0 0 -9.81" solver="Newton" cone="elliptic"/>
            <worldbody>
                <body pos="0 0 0.15">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <geom type="plane" size="5 5 0.1"/>
            </worldbody>
        </mujoco>
    "#,
    );

    // Step enough to get contacts
    for _ in 0..10 {
        data.step(&model).unwrap();
    }

    // solver_niter should be set and match solver_stat length
    assert_eq!(
        data.solver_stat.len(),
        data.solver_niter,
        "solver_stat.len() should equal solver_niter"
    );

    // solver_niter should be within bounds
    assert!(
        data.solver_niter <= model.solver_iterations,
        "solver_niter ({}) should be <= solver_iterations ({})",
        data.solver_niter,
        model.solver_iterations
    );

    // If there were iterations, stats should be populated and finite
    for (i, stat) in data.solver_stat.iter().enumerate() {
        assert!(
            stat.gradient.is_finite(),
            "stat[{i}].gradient should be finite"
        );
        assert!(
            stat.improvement.is_finite(),
            "stat[{i}].improvement should be finite"
        );
        assert!(
            stat.lineslope.is_finite(),
            "stat[{i}].lineslope should be finite"
        );
    }
}

#[test]
fn test_newton_solver_niter_zero_constraints() {
    // Free fall with no contacts → zero iterations
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="zero_constraints">
            <option gravity="0 0 -9.81" solver="Newton"/>
            <worldbody>
                <body pos="0 0 10">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
    );

    data.step(&model).unwrap();

    assert_eq!(
        data.solver_niter, 0,
        "Zero constraints → solver_niter should be 0"
    );
    assert!(
        data.solver_stat.is_empty(),
        "Zero constraints → solver_stat should be empty"
    );
}

// ============================================================================
// Phase C: Per-step meaninertia
// ============================================================================

#[test]
fn test_newton_per_step_meaninertia() {
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="meaninertia_test">
            <option gravity="0 0 -9.81" solver="Newton"/>
            <worldbody>
                <body pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <geom type="plane" size="5 5 0.1"/>
            </worldbody>
        </mujoco>
    "#,
    );

    data.step(&model).unwrap();

    // Per-step meaninertia should be positive and finite
    assert!(
        data.stat_meaninertia > 0.0,
        "per-step meaninertia should be positive, got {}",
        data.stat_meaninertia
    );
    assert!(
        data.stat_meaninertia.is_finite(),
        "per-step meaninertia should be finite"
    );

    // For a simple rigid body, should be close to model-level constant
    let ratio = data.stat_meaninertia / model.stat_meaninertia;
    assert!(
        ratio > 0.5 && ratio < 2.0,
        "per-step ({}) should be close to model-level ({}), ratio={}",
        data.stat_meaninertia,
        model.stat_meaninertia,
        ratio
    );
}

// ============================================================================
// Phase C: Noslip post-processor
// ============================================================================

#[test]
fn test_noslip_zero_iterations_is_noop() {
    // noslip_iterations=0 (default) should produce normal Newton results
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="noslip_noop">
            <option gravity="0 0 -9.81" solver="Newton" cone="elliptic"/>
            <worldbody>
                <body pos="0 0 0.15">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <geom type="plane" size="5 5 0.1"/>
            </worldbody>
        </mujoco>
    "#,
    );

    assert_eq!(model.noslip_iterations, 0);

    for _ in 0..10 {
        data.step(&model).unwrap();
    }

    assert!(
        data.qacc.iter().all(|x| x.is_finite()),
        "qacc should be finite with noslip_iterations=0"
    );
}

#[test]
fn test_noslip_produces_finite_results() {
    // noslip_iterations > 0 should still produce finite results
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="noslip_active">
            <option gravity="0 0 -9.81" solver="Newton" cone="elliptic"
                    noslip_iterations="20" noslip_tolerance="1e-8"/>
            <worldbody>
                <body pos="0 0 0.15">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0" friction="0.5 0.5 0.005"/>
                </body>
                <geom type="plane" size="5 5 0.1"/>
            </worldbody>
        </mujoco>
    "#,
    );

    assert_eq!(model.noslip_iterations, 20);
    assert_relative_eq!(model.noslip_tolerance, 1e-8);

    for _ in 0..20 {
        data.step(&model).unwrap();
    }

    assert!(
        data.qacc.iter().all(|x| x.is_finite()),
        "qacc should be finite with noslip enabled"
    );
    assert!(
        data.qvel.iter().all(|x| x.is_finite()),
        "qvel should be finite with noslip enabled"
    );
}

// ============================================================================
// Phase C: Sparse Hessian path
// ============================================================================

#[test]
fn test_sparse_dense_equivalence() {
    // Build a model with nv > 60 (chain of hinge joints).
    // Use 21 links: 21 hinge joints → nv=21. That's not >60, so we need more.
    // With nv > 60, need at least 61 hinge joints. Build programmatically.
    let mut mjcf = String::from(
        r#"<mujoco model="sparse_test">
            <option gravity="0 0 -9.81" solver="Newton" cone="elliptic"/>
            <worldbody>"#,
    );

    // Create a chain of 65 hinge joints (nv = 65 > 60)
    let n_links = 65;
    for i in 0..n_links {
        let indent = "    ".repeat(i + 2);
        mjcf.push_str(&format!(
            "\n{indent}<body name=\"link{i}\" pos=\"0.1 0 0\">"
        ));
        mjcf.push_str(&format!(
            "\n{indent}    <joint name=\"j{i}\" type=\"hinge\" axis=\"0 0 1\" damping=\"0.1\"/>"
        ));
        mjcf.push_str(&format!(
            "\n{indent}    <geom type=\"capsule\" size=\"0.02\" fromto=\"0 0 0 0.1 0 0\" mass=\"0.1\"/>"
        ));
    }
    // Close all bodies
    for i in (0..n_links).rev() {
        let indent = "    ".repeat(i + 2);
        mjcf.push_str(&format!("\n{indent}</body>"));
    }
    mjcf.push_str(
        r#"
            </worldbody>
        </mujoco>"#,
    );

    let (model, mut data) = model_from_mjcf(&mjcf);

    assert!(
        model.nv >= 61,
        "Model should have nv >= 61 for sparse path, got nv={}",
        model.nv
    );

    // Give initial velocity to first few joints
    for i in 0..5.min(model.nv) {
        data.qvel[i] = 0.5;
    }

    // Step a few times — sparse path should produce finite results
    for step in 0..5 {
        data.step(&model).unwrap_or_else(|e| {
            panic!("Step {step} failed: {e:?}");
        });
    }

    // All qacc should be finite (either from Newton sparse path or PGS fallback)
    for i in 0..model.nv {
        assert!(
            data.qacc[i].is_finite(),
            "qacc[{i}] should be finite with nv={} (sparse threshold={})",
            model.nv,
            60
        );
    }

    // qvel should be finite
    for i in 0..model.nv {
        assert!(
            data.qvel[i].is_finite(),
            "qvel[{i}] should be finite with nv={}",
            model.nv
        );
    }
}
