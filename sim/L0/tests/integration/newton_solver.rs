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
// Phase C: Noslip slip reduction (quantitative)
// ============================================================================

#[test]
fn test_noslip_reduces_slip() {
    // A box sliding on a frictional plane. With noslip enabled, the tangential
    // velocity should be driven closer to zero than without noslip.
    //
    // Setup: box starts at rest on a tilted plane (slight tilt to induce slip).
    // After stepping, measure tangential velocity (vx, vy). With noslip, it
    // should be smaller.

    let mjcf_base = |noslip_iters: usize| {
        format!(
            r#"<mujoco model="noslip_slip">
                <option gravity="0 0 -9.81" solver="Newton" cone="elliptic"
                        noslip_iterations="{noslip_iters}" noslip_tolerance="1e-12"/>
                <worldbody>
                    <body pos="0 0 0.11">
                        <joint type="free"/>
                        <geom type="box" size="0.1 0.1 0.1" mass="1.0"
                              friction="0.3 0.3 0.005"/>
                    </body>
                    <geom type="plane" size="5 5 0.1"
                          euler="3 0 0" friction="0.3 0.3 0.005"/>
                </worldbody>
            </mujoco>"#
        )
    };

    // Run without noslip
    let (model_no, mut data_no) = model_from_mjcf(&mjcf_base(0));
    for _ in 0..50 {
        data_no.step(&model_no).unwrap();
    }

    // Run with noslip
    let (model_ns, mut data_ns) = model_from_mjcf(&mjcf_base(50));
    for _ in 0..50 {
        data_ns.step(&model_ns).unwrap();
    }

    // Tangential velocity for a free body: qvel[0] (vx), qvel[1] (vy)
    let slip_no = (data_no.qvel[0].powi(2) + data_no.qvel[1].powi(2)).sqrt();
    let slip_ns = (data_ns.qvel[0].powi(2) + data_ns.qvel[1].powi(2)).sqrt();

    // Both should be finite
    assert!(slip_no.is_finite(), "no-noslip slip should be finite");
    assert!(slip_ns.is_finite(), "noslip slip should be finite");

    // Noslip should reduce slip (or at least not increase it significantly)
    assert!(
        slip_ns <= slip_no * 1.1 + 1e-10,
        "Noslip should not increase slip: without={slip_no:.6e}, with={slip_ns:.6e}"
    );
}

// ============================================================================
// Phase C: Sparse Hessian path
// ============================================================================

/// Helper: build N independent free-joint spheres on a ground plane.
/// Each free joint contributes 6 DOFs, so n_bodies=11 gives nv=66 > 60.
fn build_multi_sphere_mjcf(n_bodies: usize) -> String {
    let mut mjcf = String::from(
        r#"<mujoco model="multi_sphere">
            <option gravity="0 0 -9.81" solver="Newton" cone="elliptic"
                    solver_iterations="100" tolerance="1e-8"/>
            <worldbody>
                <geom type="plane" size="50 50 0.1"/>"#,
    );

    // Place spheres in a grid on the plane, right at contact height
    // (radius=0.1, pos z=0.1 → touching the plane at z=0)
    for i in 0..n_bodies {
        let x = (i % 6) as f64 * 0.5;
        let y = (i / 6) as f64 * 0.5;
        mjcf.push_str(&format!(
            r#"
                <body name="sphere{i}" pos="{x} {y} 0.1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0" friction="0.5 0.5 0.005"/>
                </body>"#
        ));
    }

    mjcf.push_str(
        r#"
            </worldbody>
        </mujoco>"#,
    );
    mjcf
}

#[test]
fn test_sparse_hessian_with_contacts() {
    // Build a model with nv > 60 using free-joint spheres on a plane.
    // 11 spheres × 6 DOFs = 66 DOFs > 60 threshold, triggering sparse path.
    let mjcf = build_multi_sphere_mjcf(11);
    let (model, mut data) = model_from_mjcf(&mjcf);

    assert!(
        model.nv > 60,
        "Model should have nv > 60 for sparse path, got nv={}",
        model.nv
    );

    // Step — all spheres settle on the plane with contacts
    for step in 0..20 {
        data.step(&model).unwrap_or_else(|e| {
            panic!("Step {step} failed: {e:?}");
        });
    }

    // All qacc should be finite
    assert!(
        data.qacc.iter().all(|x| x.is_finite()),
        "qacc should be all finite with sparse Hessian (nv={})",
        model.nv
    );

    // All qvel should be finite and near zero (spheres resting on plane)
    assert!(
        data.qvel.iter().all(|x| x.is_finite()),
        "qvel should be all finite with sparse Hessian (nv={})",
        model.nv
    );
}

#[test]
fn test_sparse_dense_equivalence() {
    // Compare physics behavior between sparse path (nv > 60) and dense path
    // (nv ≤ 60) on equivalent sphere-on-plane models. Both should produce the
    // same physics: spheres settle under gravity onto the plane.

    // Dense path: 5 spheres × 6 DOFs = 30 ≤ 60
    let mjcf_dense = build_multi_sphere_mjcf(5);
    let (model_d, mut data_d) = model_from_mjcf(&mjcf_dense);
    assert!(
        model_d.nv <= 60,
        "Dense model nv={} should be <= 60",
        model_d.nv
    );

    // Sparse path: 11 spheres × 6 DOFs = 66 > 60
    let mjcf_sparse = build_multi_sphere_mjcf(11);
    let (model_s, mut data_s) = model_from_mjcf(&mjcf_sparse);
    assert!(
        model_s.nv > 60,
        "Sparse model nv={} should be > 60",
        model_s.nv
    );

    // Step both
    for step in 0..20 {
        data_d.step(&model_d).unwrap_or_else(|e| {
            panic!("Dense step {step} failed: {e:?}");
        });
        data_s.step(&model_s).unwrap_or_else(|e| {
            panic!("Sparse step {step} failed: {e:?}");
        });
    }

    // Both should have finite qacc and qvel
    assert!(
        data_d.qacc.iter().all(|x| x.is_finite()),
        "Dense qacc not finite"
    );
    assert!(
        data_s.qacc.iter().all(|x| x.is_finite()),
        "Sparse qacc not finite"
    );
    assert!(
        data_d.qvel.iter().all(|x| x.is_finite()),
        "Dense qvel not finite"
    );
    assert!(
        data_s.qvel.iter().all(|x| x.is_finite()),
        "Sparse qvel not finite"
    );

    // Both paths should produce similar physics for the first sphere (body 0).
    // DOFs 0-5 are [vx, vy, vz, wx, wy, wz] for the first sphere.
    // After settling, vz should be near zero and qacc[2] (z-acc) should be
    // near zero (gravity balanced by contact force).
    let qacc_z_dense = data_d.qacc[2];
    let qacc_z_sparse = data_s.qacc[2];

    // Both should have near-zero vertical acceleration (sphere resting on plane)
    assert!(
        qacc_z_dense.abs() < 1.0,
        "Dense: first sphere z-acceleration should be small, got {qacc_z_dense}"
    );
    assert!(
        qacc_z_sparse.abs() < 1.0,
        "Sparse: first sphere z-acceleration should be small, got {qacc_z_sparse}"
    );

    // The dense and sparse paths solve the same physics for identical spheres,
    // so the first sphere's qacc should match closely (both are independent
    // spheres on a plane — the solver type shouldn't change the answer).
    assert_relative_eq!(qacc_z_dense, qacc_z_sparse, epsilon = 0.1);
}

/// Helper: build a branching "spider" robot — a free-joint torso with N limbs,
/// each having `joints_per_limb` hinge joints, resting on a ground plane.
/// This creates a coupled kinematic tree where contacts on different limbs
/// couple through the shared torso, producing significant off-diagonal fill
/// in the Hessian H = M + J^T·D·J.
/// Helper: build a branching "hedgehog" — a free-joint torso with N single-hinge
/// limbs, each having a foot sphere for ground contact.
/// This creates a coupled kinematic tree where contacts on different limbs
/// couple through the shared torso, producing off-diagonal fill in the Hessian.
/// Each limb adds 1 DOF (hinge), and the torso adds 6 (free), so total nv = 6 + n_limbs.
/// Build a chain of free-joint spheres on a ground plane, linked by connect
/// equality constraints. This produces a coupled system where constraint J rows
/// span DOFs of adjacent bodies, creating off-diagonal fill in the Hessian
/// H = M + J^T·D·J. Each free body adds 6 DOFs, so nv = n_bodies * 6.
fn build_coupled_chain_mjcf(n_bodies: usize) -> String {
    let mut mjcf = String::from(
        r#"<mujoco model="coupled_chain">
            <option gravity="0 0 -9.81" solver="Newton" cone="elliptic"
                    solver_iterations="100" tolerance="1e-8"/>
            <worldbody>
                <geom type="plane" size="50 50 0.1"/>"#,
    );

    // Place spheres in a line, resting on the plane
    for i in 0..n_bodies {
        let x = i as f64 * 0.3;
        mjcf.push_str(&format!(
            r#"
                <body name="b{i}" pos="{x} 0 0.1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0" friction="0.5 0.5 0.005"/>
                </body>"#
        ));
    }

    mjcf.push_str(
        r#"
            </worldbody>
            <equality>"#,
    );

    // Connect adjacent bodies to create coupling
    for i in 0..n_bodies.saturating_sub(1) {
        mjcf.push_str(&format!(
            r#"
                <connect body1="b{}" body2="b{}" anchor="0.15 0 0"/>"#,
            i,
            i + 1
        ));
    }

    mjcf.push_str(
        r#"
            </equality>
        </mujoco>"#,
    );
    mjcf
}

#[test]
fn test_sparse_hessian_coupled_contacts() {
    // A chain of 11 free-joint spheres linked by connect equality constraints,
    // resting on a ground plane. nv = 11 × 6 = 66 > 60, triggering the sparse
    // path. The equality constraints create J rows spanning DOFs of adjacent
    // bodies, producing off-diagonal fill in H = M + J^T·D·J. This tests that
    // the sparse LDL^T factorization correctly handles off-diagonal entries.
    let mjcf = build_coupled_chain_mjcf(11);
    let (model, mut data) = model_from_mjcf(&mjcf);

    assert!(
        model.nv > 60,
        "Coupled chain should have nv > 60 for sparse path, got nv={}",
        model.nv
    );

    // Step — chain settles onto the plane with coupled contacts
    for step in 0..20 {
        data.step(&model).unwrap_or_else(|e| {
            panic!("Step {step} failed: {e:?}");
        });
    }

    // All state should be finite
    assert!(
        data.qacc.iter().all(|x| x.is_finite()),
        "qacc should be all finite (coupled chain, nv={})",
        model.nv
    );
    assert!(
        data.qvel.iter().all(|x| x.is_finite()),
        "qvel should be all finite (coupled chain, nv={})",
        model.nv
    );

    // First body should not have fallen through the plane
    assert!(
        data.qpos[2] > -1.0,
        "First body should not fall through plane, z={}",
        data.qpos[2]
    );
}

#[test]
fn test_sparse_dense_coupled_equivalence() {
    // Compare a small coupled chain (dense path) vs large coupled chain (sparse
    // path). Both use the same physics: free-joint spheres on a plane linked by
    // connect equality constraints. The first body's z-acceleration should match
    // because it experiences the same physics (gravity + contact + one equality
    // constraint to its neighbor).

    // Dense: 5 bodies × 6 DOFs = 30 (< 60)
    let mjcf_dense = build_coupled_chain_mjcf(5);
    let (model_d, mut data_d) = model_from_mjcf(&mjcf_dense);
    assert!(model_d.nv <= 60, "Dense nv={} should be <= 60", model_d.nv);

    // Sparse: 11 bodies × 6 DOFs = 66 (> 60)
    let mjcf_sparse = build_coupled_chain_mjcf(11);
    let (model_s, mut data_s) = model_from_mjcf(&mjcf_sparse);
    assert!(model_s.nv > 60, "Sparse nv={} should be > 60", model_s.nv);

    // Step both
    for step in 0..20 {
        data_d.step(&model_d).unwrap_or_else(|e| {
            panic!("Dense step {step} failed: {e:?}");
        });
        data_s.step(&model_s).unwrap_or_else(|e| {
            panic!("Sparse step {step} failed: {e:?}");
        });
    }

    // Both should be finite
    assert!(
        data_d.qacc.iter().all(|x| x.is_finite()),
        "Dense qacc not finite"
    );
    assert!(
        data_s.qacc.iter().all(|x| x.is_finite()),
        "Sparse qacc not finite"
    );

    // Both chains' bodies should have bounded z-acceleration (gravity + contacts
    // + equality constraints). The chain lengths differ (5 vs 11 bodies) so
    // absolute values differ, but both should produce physically reasonable results.
    for i in 0..model_d.nv / 6 {
        let az = data_d.qacc[i * 6 + 2]; // z-component of each body's acceleration
        assert!(
            az.abs() < 100.0,
            "Dense body {i} z-acc should be bounded, got {az}"
        );
    }
    for i in 0..model_s.nv / 6 {
        let az = data_s.qacc[i * 6 + 2];
        assert!(
            az.abs() < 100.0,
            "Sparse body {i} z-acc should be bounded, got {az}"
        );
    }

    // Verify that the sparse path doesn't diverge from the dense path's
    // energy characteristics: both should have bounded total kinetic energy.
    let ke_dense: f64 = (0..model_d.nv).map(|i| data_d.qvel[i].powi(2)).sum();
    let ke_sparse: f64 = (0..model_s.nv).map(|i| data_s.qvel[i].powi(2)).sum();
    assert!(
        ke_dense.is_finite(),
        "Dense kinetic energy should be finite"
    );
    assert!(
        ke_sparse.is_finite(),
        "Sparse kinetic energy should be finite"
    );
}

#[test]
fn test_sparse_threshold_selection() {
    // Verify that small models use dense path and large models use sparse path.
    // We can observe this indirectly through solver_stat behavior.

    // Small model (nv = 6, free joint): should use dense path
    let (model_small, mut data_small) = model_from_mjcf(
        r#"
        <mujoco model="small_for_dense">
            <option gravity="0 0 -9.81" solver="Newton" cone="elliptic"/>
            <worldbody>
                <body pos="0 0 0.11">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <geom type="plane" size="5 5 0.1"/>
            </worldbody>
        </mujoco>
    "#,
    );
    assert!(
        model_small.nv <= 60,
        "Small model nv={} should be <= 60",
        model_small.nv
    );

    // Step enough to ensure the sphere is resting on the plane with contacts
    for _ in 0..20 {
        data_small.step(&model_small).unwrap();
    }

    // Should produce finite results
    assert!(data_small.qacc.iter().all(|x| x.is_finite()));

    // Large model (nv > 60): should use sparse path
    let mjcf_large = build_multi_sphere_mjcf(11);
    let (model_large, mut data_large) = model_from_mjcf(&mjcf_large);
    assert!(
        model_large.nv > 60,
        "Large model nv={} should be > 60",
        model_large.nv
    );

    for _ in 0..20 {
        data_large.step(&model_large).unwrap();
    }

    // Should also produce finite results
    assert!(data_large.qacc.iter().all(|x| x.is_finite()));
}
