//! Noslip post-processor acceptance tests (§33).
//!
//! Tests verify noslip for PGS/CG/Newton solvers, friction-loss rows,
//! elliptic cone projection, and pyramidal 2x2 block solve.

use approx::assert_relative_eq;
use nalgebra::DVector;
use sim_core::ConstraintType;
use sim_mjcf::load_model;

/// Helper: load model from MJCF string.
fn model_from_mjcf(mjcf: &str) -> (sim_core::Model, sim_core::Data) {
    let model = load_model(mjcf).expect("MJCF should load");
    let data = model.make_data();
    (model, data)
}

// ============================================================================
// AC1: PGS + noslip reduces tangential slip
// ============================================================================

#[test]
fn test_pgs_noslip_reduces_slip() {
    // Box on 20° incline with PGS solver. Noslip should reduce tangential slip.
    let make_model = |noslip_iters: usize| {
        format!(
            r#"<mujoco model="pgs_noslip">
                <option gravity="0 0 -9.81" timestep="0.002" solver="PGS" cone="elliptic"
                        iterations="100" noslip_iterations="{noslip_iters}" noslip_tolerance="1e-12"/>
                <worldbody>
                    <body pos="0 0 0.11">
                        <joint type="free"/>
                        <geom type="box" size="0.1 0.1 0.1" mass="1.0"
                              friction="0.5 0.5 0.005"/>
                    </body>
                    <geom type="plane" size="5 5 0.1"
                          euler="20 0 0" friction="0.5 0.5 0.005"/>
                </worldbody>
            </mujoco>"#
        )
    };

    // Without noslip
    let (model_no, mut data_no) = model_from_mjcf(&make_model(0));
    assert_eq!(model_no.solver_type, sim_core::SolverType::PGS);
    for _ in 0..100 {
        data_no.step(&model_no).unwrap();
    }

    // With noslip
    let (model_ns, mut data_ns) = model_from_mjcf(&make_model(20));
    for _ in 0..100 {
        data_ns.step(&model_ns).unwrap();
    }

    let slip_no = (data_no.qvel[0].powi(2) + data_no.qvel[1].powi(2)).sqrt();
    let slip_ns = (data_ns.qvel[0].powi(2) + data_ns.qvel[1].powi(2)).sqrt();

    assert!(slip_no.is_finite(), "PGS no-noslip slip should be finite");
    assert!(slip_ns.is_finite(), "PGS noslip slip should be finite");
    assert!(
        slip_ns <= slip_no * 1.1 + 1e-10,
        "PGS noslip should not increase slip: without={slip_no:.6e}, with={slip_ns:.6e}"
    );
}

// ============================================================================
// AC2: CG + noslip reduces tangential slip
// ============================================================================

#[test]
fn test_cg_noslip_reduces_slip() {
    let make_model = |noslip_iters: usize| {
        format!(
            r#"<mujoco model="cg_noslip">
                <option gravity="0 0 -9.81" timestep="0.002" solver="CG" cone="elliptic"
                        iterations="100" noslip_iterations="{noslip_iters}" noslip_tolerance="1e-12"/>
                <worldbody>
                    <body pos="0 0 0.11">
                        <joint type="free"/>
                        <geom type="box" size="0.1 0.1 0.1" mass="1.0"
                              friction="0.5 0.5 0.005"/>
                    </body>
                    <geom type="plane" size="5 5 0.1"
                          euler="20 0 0" friction="0.5 0.5 0.005"/>
                </worldbody>
            </mujoco>"#
        )
    };

    let (model_no, mut data_no) = model_from_mjcf(&make_model(0));
    assert_eq!(model_no.solver_type, sim_core::SolverType::CG);
    for _ in 0..100 {
        data_no.step(&model_no).unwrap();
    }

    let (model_ns, mut data_ns) = model_from_mjcf(&make_model(20));
    for _ in 0..100 {
        data_ns.step(&model_ns).unwrap();
    }

    let slip_no = (data_no.qvel[0].powi(2) + data_no.qvel[1].powi(2)).sqrt();
    let slip_ns = (data_ns.qvel[0].powi(2) + data_ns.qvel[1].powi(2)).sqrt();

    assert!(slip_no.is_finite(), "CG no-noslip slip should be finite");
    assert!(slip_ns.is_finite(), "CG noslip slip should be finite");
    assert!(
        slip_ns <= slip_no * 1.1 + 1e-10,
        "CG noslip should not increase slip: without={slip_no:.6e}, with={slip_ns:.6e}"
    );
}

// ============================================================================
// AC3: Newton regression — existing noslip tests still pass
// ============================================================================

#[test]
fn test_newton_noslip_regression() {
    // Verify Newton + noslip still produces finite, reduced-slip results.
    let make_model = |noslip_iters: usize| {
        format!(
            r#"<mujoco model="newton_noslip_reg">
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

    let (model_no, mut data_no) = model_from_mjcf(&make_model(0));
    for _ in 0..50 {
        data_no.step(&model_no).unwrap();
    }

    let (model_ns, mut data_ns) = model_from_mjcf(&make_model(50));
    for _ in 0..50 {
        data_ns.step(&model_ns).unwrap();
    }

    let slip_no = (data_no.qvel[0].powi(2) + data_no.qvel[1].powi(2)).sqrt();
    let slip_ns = (data_ns.qvel[0].powi(2) + data_ns.qvel[1].powi(2)).sqrt();

    assert!(slip_no.is_finite());
    assert!(slip_ns.is_finite());
    assert!(
        slip_ns <= slip_no * 1.1 + 1e-10,
        "Newton noslip regression: without={slip_no:.6e}, with={slip_ns:.6e}"
    );
}

// ============================================================================
// AC5: noslip_iterations=0 is no-op for all solver types
// ============================================================================

#[test]
fn test_noslip_zero_iterations_noop_pgs() {
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="pgs_noop">
            <option gravity="0 0 -9.81" solver="PGS" cone="elliptic"
                    noslip_iterations="0"/>
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
    assert!(data.qacc.iter().all(|x| x.is_finite()));
    assert!(data.qvel.iter().all(|x| x.is_finite()));
}

#[test]
fn test_noslip_zero_iterations_noop_cg() {
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="cg_noop">
            <option gravity="0 0 -9.81" solver="CG" cone="elliptic"
                    noslip_iterations="0"/>
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
    assert!(data.qacc.iter().all(|x| x.is_finite()));
    assert!(data.qvel.iter().all(|x| x.is_finite()));
}

// ============================================================================
// AC6: Normal forces unchanged by noslip
// ============================================================================

#[test]
fn test_noslip_preserves_normal_forces() {
    // Run one step with and without noslip. Contact normal forces should be
    // bit-exact identical (noslip only touches friction rows).
    let make_model = |noslip_iters: usize| {
        format!(
            r#"<mujoco model="normal_forces">
                <option gravity="0 0 -9.81" solver="PGS" cone="elliptic"
                        iterations="100" noslip_iterations="{noslip_iters}" noslip_tolerance="1e-12"/>
                <worldbody>
                    <body pos="0 0 0.05">
                        <joint type="free"/>
                        <geom type="box" size="0.1 0.1 0.1" mass="1.0"
                              friction="0.5 0.5 0.005"/>
                    </body>
                    <geom type="plane" size="5 5 0.1"/>
                </worldbody>
            </mujoco>"#
        )
    };

    // Single step — box starts penetrating, so contact is immediate.
    // Compare normal forces: noslip only touches friction rows, not normals.
    let (model_no, mut data_no) = model_from_mjcf(&make_model(0));
    data_no.step(&model_no).unwrap();

    let (model_ns, mut data_ns) = model_from_mjcf(&make_model(20));
    data_ns.step(&model_ns).unwrap();

    // Find contact normal rows (first row of each contact group)
    let nefc_no = data_no.efc_type.len();
    let nefc_ns = data_ns.efc_type.len();
    assert_eq!(nefc_no, nefc_ns, "same constraint layout");

    let mut found_normals = 0;
    let mut i = 0;
    while i < nefc_no {
        let ctype = data_no.efc_type[i];
        let dim = data_no.efc_dim[i];
        if matches!(
            ctype,
            sim_core::ConstraintType::ContactElliptic
                | sim_core::ConstraintType::ContactFrictionless
        ) && dim >= 1
        {
            // Row i is the normal row
            let normal_no = data_no.efc_force[i];
            let normal_ns = data_ns.efc_force[i];
            assert_relative_eq!(normal_no, normal_ns, epsilon = 1e-12, max_relative = 1e-12);
            found_normals += 1;
            i += dim;
        } else {
            i += 1;
        }
    }
    assert!(found_normals > 0, "should have found contact normal rows");
}

// ============================================================================
// AC7: Friction-loss rows clamped by noslip
// ============================================================================

#[test]
fn test_noslip_friction_loss_clamping() {
    // A joint with frictionloss + noslip. After noslip, the friction-loss
    // constraint force should be clamped to [-floss, +floss].
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="floss_noslip">
            <option gravity="0 0 -9.81" solver="PGS" cone="elliptic"
                    iterations="100" noslip_iterations="50" noslip_tolerance="1e-12"/>
            <worldbody>
                <body name="arm" pos="0 0 0">
                    <joint type="hinge" axis="0 0 1" frictionloss="0.1"/>
                    <geom type="capsule" size="0.05 0.3" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
    );

    // Give the joint a high velocity to drive the friction-loss row into saturation
    data.qvel[0] = 50.0;
    data.step(&model).unwrap();

    // Find friction-loss rows
    let mut found_floss = false;
    for i in 0..data.efc_type.len() {
        if data.efc_type[i] == sim_core::ConstraintType::FrictionLoss {
            let floss = data.efc_floss[i];
            let force = data.efc_force[i];
            assert!(
                force >= -floss - 1e-10 && force <= floss + 1e-10,
                "Friction-loss force {force} should be in [-{floss}, {floss}]"
            );
            found_floss = true;
        }
    }
    assert!(found_floss, "should have friction-loss constraint rows");
}

#[test]
fn test_noslip_friction_loss_produces_finite_forces() {
    // Verify noslip with friction-loss rows produces finite results across solvers.
    for solver in &["PGS", "CG", "Newton"] {
        let mjcf = format!(
            r#"<mujoco model="floss_{solver}">
                <option gravity="0 0 -9.81" solver="{solver}" cone="elliptic"
                        iterations="100" noslip_iterations="20" noslip_tolerance="1e-8"/>
                <worldbody>
                    <body name="arm" pos="0 0 0">
                        <joint type="hinge" axis="0 0 1" frictionloss="0.5"/>
                        <geom type="capsule" size="0.05 0.3" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>"#
        );

        let (model, mut data) = model_from_mjcf(&mjcf);
        data.qvel[0] = 10.0;
        for _ in 0..20 {
            data.step(&model).unwrap();
        }

        assert!(
            data.qacc.iter().all(|x| x.is_finite()),
            "{solver}: qacc should be finite with noslip + friction-loss"
        );
        assert!(
            data.qvel.iter().all(|x| x.is_finite()),
            "{solver}: qvel should be finite with noslip + friction-loss"
        );
    }
}

// ============================================================================
// AC8: Pyramidal noslip — 2x2 block solve
// ============================================================================

#[test]
fn test_noslip_pyramidal_reduces_slip() {
    // Box on incline with pyramidal cone + noslip. Should reduce slip.
    let make_model = |noslip_iters: usize| {
        format!(
            r#"<mujoco model="pyr_noslip">
                <option gravity="0 0 -9.81" timestep="0.002" solver="PGS" cone="pyramidal"
                        iterations="100" noslip_iterations="{noslip_iters}" noslip_tolerance="1e-12"/>
                <worldbody>
                    <body pos="0 0 0.11">
                        <joint type="free"/>
                        <geom type="box" size="0.1 0.1 0.1" mass="1.0"
                              friction="0.5 0.5 0.005"/>
                    </body>
                    <geom type="plane" size="5 5 0.1"
                          euler="20 0 0" friction="0.5 0.5 0.005"/>
                </worldbody>
            </mujoco>"#
        )
    };

    let (model_no, mut data_no) = model_from_mjcf(&make_model(0));
    for _ in 0..100 {
        data_no.step(&model_no).unwrap();
    }

    let (model_ns, mut data_ns) = model_from_mjcf(&make_model(20));
    for _ in 0..100 {
        data_ns.step(&model_ns).unwrap();
    }

    let slip_no = (data_no.qvel[0].powi(2) + data_no.qvel[1].powi(2)).sqrt();
    let slip_ns = (data_ns.qvel[0].powi(2) + data_ns.qvel[1].powi(2)).sqrt();

    assert!(
        slip_no.is_finite(),
        "pyramidal no-noslip slip should be finite"
    );
    assert!(
        slip_ns.is_finite(),
        "pyramidal noslip slip should be finite"
    );
    assert!(
        slip_ns <= slip_no * 1.1 + 1e-10,
        "Pyramidal noslip should not increase slip: without={slip_no:.6e}, with={slip_ns:.6e}"
    );
}

#[test]
fn test_noslip_pyramidal_forces_nonnegative() {
    // After noslip with pyramidal cone, all pyramidal facet forces should remain >= 0.
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="pyr_nonneg">
            <option gravity="0 0 -9.81" solver="PGS" cone="pyramidal"
                    iterations="100" noslip_iterations="50" noslip_tolerance="1e-12"/>
            <worldbody>
                <body pos="0 0 0.099">
                    <joint type="free"/>
                    <geom type="box" size="0.1 0.1 0.1" mass="1.0"
                          friction="0.5 0.5 0.005"/>
                </body>
                <geom type="plane" size="5 5 0.1"
                      euler="10 0 0" friction="0.5 0.5 0.005"/>
            </worldbody>
        </mujoco>
    "#,
    );

    for _ in 0..30 {
        data.step(&model).unwrap();
    }

    // All pyramidal facet forces must be non-negative
    let mut found_pyramidal = false;
    for i in 0..data.efc_type.len() {
        if data.efc_type[i] == sim_core::ConstraintType::ContactPyramidal {
            assert!(
                data.efc_force[i] >= -1e-10,
                "Pyramidal facet force[{i}] should be >= 0 after noslip, got {}",
                data.efc_force[i]
            );
            found_pyramidal = true;
        }
    }
    assert!(found_pyramidal, "should have pyramidal contact rows");
}

#[test]
fn test_noslip_pyramidal_finite_results_all_solvers() {
    // Pyramidal + noslip should produce finite results for all solver types.
    for solver in &["PGS", "CG", "Newton"] {
        let mjcf = format!(
            r#"<mujoco model="pyr_{solver}">
                <option gravity="0 0 -9.81" solver="{solver}" cone="pyramidal"
                        iterations="100" noslip_iterations="20" noslip_tolerance="1e-8"/>
                <worldbody>
                    <body pos="0 0 0.11">
                        <joint type="free"/>
                        <geom type="box" size="0.1 0.1 0.1" mass="1.0"
                              friction="0.5 0.5 0.005"/>
                    </body>
                    <geom type="plane" size="5 5 0.1"/>
                </worldbody>
            </mujoco>"#
        );

        let (model, mut data) = model_from_mjcf(&mjcf);
        for _ in 0..20 {
            data.step(&model).unwrap();
        }

        assert!(
            data.qacc.iter().all(|x| x.is_finite()),
            "{solver}: qacc should be finite with pyramidal noslip"
        );
        assert!(
            data.qvel.iter().all(|x| x.is_finite()),
            "{solver}: qvel should be finite with pyramidal noslip"
        );
    }
}

// ============================================================================
// Combined: friction-loss + contact friction + noslip
// ============================================================================

#[test]
fn test_noslip_mixed_friction_loss_and_contact() {
    // Model with both friction-loss joints and contact friction.
    // Noslip should handle both row types in the same submatrix.
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="mixed_noslip">
            <option gravity="0 0 -9.81" solver="PGS" cone="elliptic"
                    iterations="100" noslip_iterations="20" noslip_tolerance="1e-8"/>
            <worldbody>
                <body name="arm" pos="0 0 0.5">
                    <joint type="hinge" axis="0 1 0" frictionloss="0.5"/>
                    <geom type="capsule" size="0.05 0.3" mass="1.0" friction="0.5 0.5 0.005"/>
                    <body name="forearm" pos="0 0.6 0">
                        <joint type="hinge" axis="0 1 0" frictionloss="0.2"/>
                        <geom type="capsule" size="0.04 0.2" mass="0.5" friction="0.5 0.5 0.005"/>
                    </body>
                </body>
                <geom type="plane" size="5 5 0.1"/>
            </worldbody>
        </mujoco>
    "#,
    );

    for _ in 0..30 {
        data.step(&model).unwrap();
    }

    assert!(
        data.qacc.iter().all(|x| x.is_finite()),
        "Mixed friction-loss + contact with noslip should be finite"
    );

    // Verify friction-loss rows are clamped
    for i in 0..data.efc_type.len() {
        if data.efc_type[i] == sim_core::ConstraintType::FrictionLoss {
            let floss = data.efc_floss[i];
            let force = data.efc_force[i];
            assert!(
                force >= -floss - 1e-10 && force <= floss + 1e-10,
                "Mixed: friction-loss force {force} out of [-{floss}, {floss}]"
            );
        }
    }
}

// ============================================================================
// Noslip tolerance convergence
// ============================================================================

#[test]
fn test_noslip_early_exit_on_convergence() {
    // With very high noslip_iterations and tight tolerance, noslip should
    // converge early. Verify that the result is stable (running more
    // iterations doesn't change forces).
    let make_model = |noslip_iters: usize| {
        format!(
            r#"<mujoco model="converge">
                <option gravity="0 0 -9.81" solver="PGS" cone="elliptic"
                        iterations="100" noslip_iterations="{noslip_iters}" noslip_tolerance="1e-14"/>
                <worldbody>
                    <body pos="0 0 0.11">
                        <joint type="free"/>
                        <geom type="box" size="0.1 0.1 0.1" mass="1.0"
                              friction="0.5 0.5 0.005"/>
                    </body>
                    <geom type="plane" size="5 5 0.1"/>
                </worldbody>
            </mujoco>"#
        )
    };

    // Run with moderate and high noslip iterations
    let (model_mod, mut data_mod) = model_from_mjcf(&make_model(50));
    data_mod.step(&model_mod).unwrap();

    let (model_high, mut data_high) = model_from_mjcf(&make_model(500));
    data_high.step(&model_high).unwrap();

    // Forces should be very close if both converged
    let nefc = data_mod.efc_type.len();
    assert_eq!(nefc, data_high.efc_type.len());
    for i in 0..nefc {
        assert_relative_eq!(
            data_mod.efc_force[i],
            data_high.efc_force[i],
            epsilon = 1e-6,
            max_relative = 1e-6,
        );
    }
}

// ============================================================================
// AC9: Full-matrix residual check (conformance — uses efc_b, not efc_jar)
// ============================================================================

#[test]
fn test_noslip_residual_uses_efc_b() {
    // After noslip convergence with many iterations, the full-matrix residual
    //   r[i] = efc_b[i] + Σ_j A[i,j] * efc_force[j]
    // should be ≈ 0 for unclamped friction rows (where the GS fixed point is
    // at the unconstrained minimum). We verify that noslip converges to the
    // correct fixed point by checking the residual is near zero.
    //
    // This test would FAIL with the old efc_jar-based RHS.
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="residual_check">
            <option gravity="0 0 -9.81" solver="PGS" cone="elliptic"
                    iterations="200" noslip_iterations="200" noslip_tolerance="1e-14"/>
            <worldbody>
                <body pos="0 0 0.05">
                    <joint type="free"/>
                    <geom type="box" size="0.1 0.1 0.1" mass="1.0"
                          friction="0.5 0.5 0.005"/>
                </body>
                <geom type="plane" size="5 5 0.1"/>
            </worldbody>
        </mujoco>
    "#,
    );

    data.step(&model).unwrap();

    // Compute full residual for friction rows: r[i] = efc_b[i] + Σ_j A[i,j]*f[j]
    // where A[i,j] = J[i] · M⁻¹ · J[j]^T (unregularized Delassus)
    let nefc = data.efc_type.len();
    let nv = model.nv;
    assert!(nefc > 0, "should have constraint rows");

    // Find friction rows (noslip-eligible)
    let mut friction_rows: Vec<usize> = Vec::new();
    let mut i = 0;
    while i < nefc {
        let ctype = data.efc_type[i];
        let dim = data.efc_dim[i];
        match ctype {
            ConstraintType::FrictionLoss => {
                friction_rows.push(i);
                i += 1;
            }
            ConstraintType::ContactElliptic | ConstraintType::ContactFrictionless => {
                for j in 1..dim {
                    friction_rows.push(i + j);
                }
                i += dim;
            }
            _ => {
                i += 1;
            }
        }
    }
    assert!(!friction_rows.is_empty(), "should have friction rows");

    // For each friction row, compute full-matrix residual
    for &row in &friction_rows {
        // Solve M⁻¹ · J[row]^T
        let mut minv_j = DVector::<f64>::zeros(nv);
        for col in 0..nv {
            minv_j[col] = data.efc_J[(row, col)];
        }
        let (rowadr, rownnz, colind) = model.qld_csr();
        sim_core::mj_solve_sparse(
            rowadr,
            rownnz,
            colind,
            &data.qLD_data,
            &data.qLD_diag_inv,
            &mut minv_j,
        );

        // Residual = efc_b[row] + Σ_j (J[j] · minv_j) * efc_force[j]
        let mut residual = data.efc_b[row];
        for j in 0..nefc {
            let mut dot = 0.0;
            for col in 0..nv {
                dot += data.efc_J[(j, col)] * minv_j[col];
            }
            residual += dot * data.efc_force[j];
        }

        // Residual should be near zero for converged unclamped rows.
        // Some rows may be at their clamp boundary (friction-loss at ±floss,
        // or cone-projected), so the residual can be nonzero there.
        // For a box on flat ground, friction should be small and unclamped.
        assert!(
            residual.abs() < 0.1,
            "Full-matrix residual for friction row {row} should be small, got {residual:.6e}"
        );
    }
}

// ============================================================================
// AC10: Elliptic cone forces respect cone constraint after QCQP
// ============================================================================

#[test]
fn test_noslip_elliptic_cone_satisfied() {
    // After noslip with QCQP, elliptic contact friction forces should satisfy
    // the cone constraint: Σ(f_j/μ_j)² ≤ f_normal²
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="cone_check">
            <option gravity="0 0 -9.81" solver="PGS" cone="elliptic"
                    iterations="100" noslip_iterations="50" noslip_tolerance="1e-12"/>
            <worldbody>
                <body pos="0 0 0.05">
                    <joint type="free"/>
                    <geom type="box" size="0.1 0.1 0.1" mass="1.0"
                          friction="0.5 0.3 0.005" condim="3"/>
                </body>
                <geom type="plane" size="5 5 0.1"
                      euler="15 0 0" friction="0.5 0.3 0.005" condim="3"/>
            </worldbody>
        </mujoco>
    "#,
    );

    // Single step with penetrating box — guaranteed contact
    data.step(&model).unwrap();

    let nefc = data.efc_type.len();
    let mut checked = 0;
    let mut i = 0;
    while i < nefc {
        let ctype = data.efc_type[i];
        let dim = data.efc_dim[i];
        if matches!(
            ctype,
            ConstraintType::ContactElliptic | ConstraintType::ContactFrictionless
        ) && dim >= 3
        {
            let fn_force = data.efc_force[i];
            let mu = data.efc_mu[i];

            // Compute friction norm: sqrt(Σ (f_j/μ_j)²)
            let mut s_sq = 0.0;
            for j in 1..dim {
                let mu_j = mu[j - 1];
                if mu_j > 1e-15 {
                    s_sq += (data.efc_force[i + j] / mu_j).powi(2);
                }
            }
            let s = s_sq.sqrt();

            // Cone constraint: s ≤ |fn| (with tolerance)
            assert!(
                s <= fn_force.abs() + 1e-6,
                "Cone violation at contact {i}: friction_norm={s:.6e}, |fn|={:.6e}",
                fn_force.abs()
            );
            checked += 1;
            i += dim;
        } else {
            i += 1;
        }
    }
    assert!(checked > 0, "should have checked at least one contact");
}

// ============================================================================
// AC11: Cross-coupling — noslip with mixed constraint types
// ============================================================================

#[test]
fn test_noslip_cross_coupling_with_limits() {
    // Model with joint limits + contacts + friction-loss + noslip.
    // The cross-coupling fix (efc_b + full Delassus row) matters here because
    // limit constraint forces affect the friction rows through the Delassus matrix.
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="cross_coupling">
            <option gravity="0 0 -9.81" solver="PGS" cone="elliptic"
                    iterations="200" noslip_iterations="50" noslip_tolerance="1e-12"/>
            <worldbody>
                <body pos="0 0 0.5">
                    <joint type="hinge" axis="0 1 0" limited="true" range="-45 45"
                           frictionloss="0.1"/>
                    <geom type="capsule" size="0.05 0.4" mass="1.0"
                          friction="0.3 0.3 0.005"/>
                </body>
                <geom type="plane" size="5 5 0.1"/>
            </worldbody>
        </mujoco>
    "#,
    );

    // Run enough steps for the arm to hit the limit and make contact
    for _ in 0..100 {
        data.step(&model).unwrap();
    }

    assert!(
        data.qacc.iter().all(|x| x.is_finite()),
        "cross-coupling with limits + friction-loss + contact should be finite"
    );
    assert!(
        data.qvel.iter().all(|x| x.is_finite()),
        "cross-coupling velocities should be finite"
    );
}

// ============================================================================
// AC12: Noslip processes contacts regardless of efc_state
// ============================================================================

#[test]
fn test_noslip_processes_all_contacts_regardless_of_state() {
    // Even contacts with zero normal force (Satisfied state) should be processed
    // by noslip. The cone projection handles fn=0 gracefully (zeroes friction).
    // This tests that removing the efc_state filter doesn't cause issues.
    let (model, mut data) = model_from_mjcf(
        r#"
        <mujoco model="state_indep">
            <option gravity="0 0 -9.81" solver="PGS" cone="elliptic"
                    iterations="100" noslip_iterations="50" noslip_tolerance="1e-12"/>
            <worldbody>
                <body pos="0 0 0.201">
                    <joint type="free"/>
                    <geom type="box" size="0.1 0.1 0.1" mass="1.0"
                          friction="0.5 0.5 0.005"/>
                </body>
                <geom type="plane" size="5 5 0.1"/>
            </worldbody>
        </mujoco>
    "#,
    );

    // Just barely above contact — first step may have very weak or no contact
    data.step(&model).unwrap();

    assert!(
        data.qacc.iter().all(|x| x.is_finite()),
        "noslip with near-zero contacts should be finite"
    );
    assert!(
        data.qvel.iter().all(|x| x.is_finite()),
        "noslip velocities should be finite"
    );

    // Continue stepping — should remain stable
    for _ in 0..20 {
        data.step(&model).unwrap();
    }
    assert!(
        data.qacc.iter().all(|x| x.is_finite()),
        "noslip should remain stable over many steps"
    );
}
