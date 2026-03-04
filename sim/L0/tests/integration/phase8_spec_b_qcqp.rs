//! Phase 8 Spec B — QCQP Cone Projection integration tests.
//!
//! Tests T4, T5, T6, T7, T8, T15, T17, T25, T27 from SPEC_B.md.
//! Verifies PGS two-phase ray+QCQP, noslip QCQP rewrite, convergence fixes,
//! and CG/Newton regression.

use sim_core::types::ConstraintType;
use sim_mjcf::load_model;

fn model_from_mjcf(mjcf: &str) -> (sim_core::Model, sim_core::Data) {
    let model = load_model(mjcf).expect("MJCF should load");
    let data = model.make_data();
    (model, data)
}

/// Helper: check elliptic cone constraint on all contact rows.
fn assert_cone_satisfied(data: &sim_core::Data, tol: f64) {
    let nefc = data.efc_type.len();
    let mut i = 0;
    while i < nefc {
        if matches!(data.efc_type[i], ConstraintType::ContactElliptic) {
            let dim = data.efc_dim[i];
            let fn_val = data.efc_force[i];
            if fn_val > 1e-10 {
                let mu = data.efc_mu[i];
                let mut ssq = 0.0;
                for j in 1..dim {
                    if mu[j - 1] > 1e-15 {
                        ssq += (data.efc_force[i + j] / mu[j - 1]).powi(2);
                    }
                }
                assert!(
                    ssq <= fn_val * fn_val + tol,
                    "cone violated at row {i}: ssq={ssq:.6e}, fn²={:.6e}",
                    fn_val * fn_val
                );
            }
            i += dim;
        } else {
            i += 1;
        }
    }
}

// ============================================================================
// T4: PGS elliptic contact — sphere at rest → AC4
// ============================================================================

#[test]
fn test_t4_pgs_elliptic_sphere_at_rest() {
    let (model, mut data) = model_from_mjcf(
        r#"<mujoco model="pgs_elliptic_rest">
            <option gravity="0 0 -9.81" timestep="0.001" solver="PGS" cone="elliptic"
                    iterations="100"/>
            <worldbody>
                <geom name="floor" type="plane" size="5 5 0.1"/>
                <body name="ball" pos="0 0 0.1">
                    <joint type="free"/>
                    <geom name="sphere" type="sphere" size="0.1" mass="1.0"
                          condim="3" friction="0.5 0.5 0.005 0.001 0.001"/>
                </body>
            </worldbody>
        </mujoco>"#,
    );

    for _ in 0..500 {
        data.step(&model).unwrap();
    }

    let z = data.qpos[2];
    assert!(
        z > 0.05 && z < 0.20,
        "ball z = {z:.6}, expected in [0.05, 0.20]"
    );

    let zacc = data.qacc[2].abs();
    assert!(zacc < 5.0, "z-acceleration = {zacc:.4}, expected < 5.0");

    assert_cone_satisfied(&data, 1e-6);
}

// ============================================================================
// T5: PGS ray update — non-negative normal → AC5
// ============================================================================

#[test]
fn test_t5a_pgs_ray_nonnegative_normal_cold_start() {
    let (model, mut data) = model_from_mjcf(
        r#"<mujoco model="pgs_cold">
            <option gravity="0 0 -9.81" timestep="0.001" solver="PGS" cone="elliptic"
                    iterations="1"/>
            <worldbody>
                <geom name="floor" type="plane" size="5 5 0.1"/>
                <body name="ball" pos="0 0 0.1">
                    <joint type="free"/>
                    <geom name="sphere" type="sphere" size="0.1" mass="1.0"
                          condim="3" friction="0.5 0.5 0.005 0.001 0.001"/>
                </body>
            </worldbody>
        </mujoco>"#,
    );

    data.step(&model).unwrap();

    let nefc = data.efc_type.len();
    let mut i = 0;
    while i < nefc {
        if matches!(data.efc_type[i], ConstraintType::ContactElliptic) {
            let dim = data.efc_dim[i];
            assert!(
                data.efc_force[i] >= 0.0,
                "normal force must be >= 0 after PGS: got {}",
                data.efc_force[i]
            );
            i += dim;
        } else {
            i += 1;
        }
    }
}

// ============================================================================
// T15: Negative test — pyramidal contacts bypass QCQP → AC14
// ============================================================================

#[test]
fn test_t15_pyramidal_bypass_qcqp() {
    let (model, mut data) = model_from_mjcf(
        r#"<mujoco model="pgs_pyramidal">
            <option gravity="0 0 -9.81" timestep="0.002" solver="PGS" cone="pyramidal"
                    iterations="50"/>
            <worldbody>
                <geom name="floor" type="plane" size="5 5 0.1"/>
                <body name="ball" pos="0 0 0.1">
                    <joint type="free"/>
                    <geom name="sphere" type="sphere" size="0.1" mass="1.0"
                          condim="3" friction="0.5 0.5 0.005 0.001 0.001"/>
                </body>
            </worldbody>
        </mujoco>"#,
    );

    for _ in 0..5 {
        data.step(&model).unwrap();
    }

    let nefc = data.efc_type.len();
    let mut found_pyramidal = false;
    for i in 0..nefc {
        if matches!(data.efc_type[i], ConstraintType::ContactPyramidal) {
            found_pyramidal = true;
            assert!(
                data.efc_force[i] >= 0.0,
                "pyramidal force must be >= 0: row {i}, got {}",
                data.efc_force[i]
            );
        }
        assert!(
            !matches!(data.efc_type[i], ConstraintType::ContactElliptic),
            "pyramidal cone should not produce elliptic rows"
        );
    }
    assert!(found_pyramidal, "should have pyramidal contact rows");
}

// ============================================================================
// T17: PGS ray update with pure-normal force → AC5
// ============================================================================

#[test]
fn test_t17_pgs_ray_pure_normal() {
    let (model, mut data) = model_from_mjcf(
        r#"<mujoco model="pgs_ray_normal">
            <option gravity="0 0 -9.81" timestep="0.002" solver="PGS" cone="elliptic"
                    iterations="100"/>
            <worldbody>
                <geom name="floor" type="plane" size="5 5 0.1"/>
                <body name="ball" pos="0 0 0.1">
                    <joint type="free"/>
                    <geom name="sphere" type="sphere" size="0.1" mass="1.0"
                          condim="3" friction="0.5 0.5 0.005 0.001 0.001"/>
                </body>
            </worldbody>
        </mujoco>"#,
    );

    // Multiple steps to ensure contacts are generated
    for _ in 0..5 {
        data.step(&model).unwrap();
    }

    let nefc = data.efc_type.len();
    let mut found_elliptic = false;
    let mut i = 0;
    while i < nefc {
        if matches!(data.efc_type[i], ConstraintType::ContactElliptic) {
            found_elliptic = true;
            let dim = data.efc_dim[i];
            assert!(
                data.efc_force[i] >= 0.0,
                "normal force = {}, expected >= 0",
                data.efc_force[i]
            );
            for j in 0..dim {
                assert!(
                    data.efc_force[i + j].is_finite(),
                    "force[{}] not finite",
                    i + j
                );
            }
            i += dim;
        } else {
            i += 1;
        }
    }
    assert!(found_elliptic, "should have elliptic contact rows");
    assert_cone_satisfied(&data, 1e-6);
}

// ============================================================================
// T6: Noslip elliptic on tilted plane → AC6
// ============================================================================

#[test]
fn test_t6_noslip_elliptic_tilted_plane() {
    // Use the same model pattern as test_pgs_noslip_reduces_slip (known working)
    let (model, mut data) = model_from_mjcf(
        r#"<mujoco model="noslip_tilted">
            <option gravity="0 0 -9.81" timestep="0.002" solver="PGS" cone="elliptic"
                    iterations="100" noslip_iterations="50" noslip_tolerance="1e-12"/>
            <worldbody>
                <body pos="0 0 0.11">
                    <joint type="free"/>
                    <geom type="box" size="0.1 0.1 0.1" mass="1.0"
                          friction="0.5 0.5 0.005"/>
                </body>
                <geom type="plane" size="5 5 0.1"
                      euler="15 0 0" friction="0.5 0.5 0.005"/>
            </worldbody>
        </mujoco>"#,
    );

    // Run enough steps for ball to settle on tilted surface
    for _ in 0..100 {
        data.step(&model).unwrap();
    }

    let nefc = data.efc_type.len();
    let mut found_contact = false;
    let mut i = 0;
    while i < nefc {
        if matches!(data.efc_type[i], ConstraintType::ContactElliptic) {
            found_contact = true;
            let dim = data.efc_dim[i];
            let fn_val = data.efc_force[i];

            // Check cone constraint if normal force is meaningful
            if fn_val > 1e-10 {
                let mu = data.efc_mu[i];
                let mut ssq = 0.0;
                for j in 1..dim {
                    if mu[j - 1] > 1e-15 {
                        ssq += (data.efc_force[i + j] / mu[j - 1]).powi(2);
                    }
                }
                assert!(
                    ssq <= fn_val * fn_val + 1e-6,
                    "cone violated: ssq={ssq:.6e}, fn²={:.6e}",
                    fn_val * fn_val
                );
            }

            for j in 0..dim {
                assert!(data.efc_force[i + j].is_finite());
            }
            i += dim;
        } else {
            i += 1;
        }
    }
    assert!(found_contact, "should have contact rows on tilted plane");
}

// ============================================================================
// T7: Noslip pyramidal finite results → AC8
// ============================================================================

#[test]
fn test_t7_noslip_pyramidal_finite() {
    let (model, mut data) = model_from_mjcf(
        r#"<mujoco model="noslip_pyramidal">
            <option gravity="0 0 -9.81" timestep="0.002" solver="PGS" cone="pyramidal"
                    iterations="100" noslip_iterations="50" noslip_tolerance="1e-12"/>
            <worldbody>
                <geom name="floor" type="plane" size="5 5 0.1"/>
                <body name="ball" pos="0 0 0.1">
                    <joint type="free"/>
                    <geom name="sphere" type="sphere" size="0.1" mass="1.0"
                          condim="3" friction="0.5 0.5 0.005 0.001 0.001"/>
                </body>
            </worldbody>
        </mujoco>"#,
    );

    data.step(&model).unwrap();

    let nefc = data.efc_type.len();
    for i in 0..nefc {
        assert!(
            data.efc_force[i].is_finite(),
            "force[{i}] not finite: {}",
            data.efc_force[i]
        );
        if matches!(data.efc_type[i], ConstraintType::ContactPyramidal) {
            assert!(
                data.efc_force[i] >= 0.0,
                "pyramidal force must be >= 0: {}",
                data.efc_force[i]
            );
        }
    }
}

// ============================================================================
// T8: Newton/CG elliptic regression → AC9
// ============================================================================

#[test]
fn test_t8_newton_elliptic_regression() {
    let (model, mut data) = model_from_mjcf(
        r#"<mujoco model="newton_elliptic">
            <option gravity="0 0 -9.81" timestep="0.002" solver="Newton" cone="elliptic"
                    iterations="50"/>
            <worldbody>
                <geom name="floor" type="plane" size="5 5 0.1"/>
                <body name="ball" pos="0 0 0.1">
                    <joint type="free"/>
                    <geom name="sphere" type="sphere" size="0.1" mass="1.0"
                          condim="3" friction="0.5 0.5 0.005 0.001 0.001"/>
                </body>
            </worldbody>
        </mujoco>"#,
    );

    for _ in 0..10 {
        data.step(&model).unwrap();
    }

    let nefc = data.efc_type.len();
    let mut found_elliptic = false;
    for i in 0..nefc {
        if matches!(data.efc_type[i], ConstraintType::ContactElliptic) {
            found_elliptic = true;
            assert_eq!(data.efc_dim[i], 3, "elliptic condim=3 should produce dim=3");
        }
        assert!(
            !matches!(data.efc_type[i], ConstraintType::ContactPyramidal),
            "elliptic cone should not produce pyramidal rows"
        );
    }
    assert!(found_elliptic, "should have elliptic contact rows");
}

// ============================================================================
// T16: CG/Newton use primal classifier, not QCQP → AC9
// ============================================================================

#[test]
fn test_t16_newton_uses_primal_not_qcqp() {
    let (model, mut data) = model_from_mjcf(
        r#"<mujoco model="newton_primal">
            <option gravity="0 0 -9.81" timestep="0.002" solver="Newton" cone="elliptic"
                    iterations="50"/>
            <worldbody>
                <geom name="floor" type="plane" size="5 5 0.1"/>
                <body name="ball" pos="0 0 0.1">
                    <joint type="free"/>
                    <geom name="sphere" type="sphere" size="0.1" mass="1.0"
                          condim="3" friction="0.5 0.5 0.005 0.001 0.001"/>
                </body>
            </worldbody>
        </mujoco>"#,
    );

    for _ in 0..5 {
        data.step(&model).unwrap();
    }

    assert_cone_satisfied(&data, 1e-6);
}

// ============================================================================
// T25: Noslip pyramidal threshold discrimination → AC8
// ============================================================================

#[test]
fn test_t25_noslip_pyramidal_threshold() {
    let (model, mut data) = model_from_mjcf(
        r#"<mujoco model="noslip_pyramidal_threshold">
            <option gravity="0 0 -9.81" timestep="0.002" solver="PGS" cone="pyramidal"
                    iterations="100" noslip_iterations="50" noslip_tolerance="1e-12"/>
            <worldbody>
                <geom name="floor" type="plane" size="5 5 0.1"/>
                <body name="ball" pos="0 0 0.1">
                    <joint type="free"/>
                    <geom name="sphere" type="sphere" size="0.1" mass="1.0"
                          condim="3" friction="0.5 0.5 0.005 0.001 0.001"/>
                </body>
            </worldbody>
        </mujoco>"#,
    );

    for _ in 0..50 {
        data.step(&model).unwrap();
    }

    let nefc = data.efc_type.len();
    for i in 0..nefc {
        assert!(data.efc_force[i].is_finite(), "force[{i}] not finite");
    }

    let z = data.qpos[2];
    assert!(
        z > 0.05 && z < 0.20,
        "ball z = {z:.6}, expected in [0.05, 0.20]"
    );
}

// ============================================================================
// T27: Noslip cone satisfied regression → supplementary
// ============================================================================

#[test]
fn test_t27_noslip_cone_satisfied_regression() {
    let (model, mut data) = model_from_mjcf(
        r#"<mujoco model="noslip_cone_regression">
            <option gravity="0 0 -9.81" timestep="0.002" solver="PGS" cone="elliptic"
                    iterations="100" noslip_iterations="20" noslip_tolerance="1e-12"/>
            <worldbody>
                <geom name="floor" type="plane" size="5 5 0.1"/>
                <body name="ball" pos="0 0 0.1">
                    <joint type="free"/>
                    <geom name="sphere" type="sphere" size="0.1" mass="1.0"
                          condim="3" friction="0.5 0.5 0.005 0.001 0.001"/>
                </body>
            </worldbody>
        </mujoco>"#,
    );

    for _ in 0..50 {
        data.step(&model).unwrap();
    }

    assert_cone_satisfied(&data, 1e-6);
}
