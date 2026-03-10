//! Layer C — Trajectory comparison tests.
//!
//! Multi-step trajectory comparison against MuJoCo 3.4.0 reference data.
//! Each test runs N steps of `data.step(&model)` and compares qpos/qvel/qacc
//! at every step against reference `.npy` files with step-aware growing
//! tolerances.
//!
//! 8 trajectory tests — one per canonical conformance model.

use super::common;
use common::{
    TRAJ_BASE_CHAOTIC, TRAJ_BASE_SMOOTH, TRAJ_GROWTH_CHAOTIC, TRAJ_GROWTH_SMOOTH, TRAJ_QACC_FACTOR,
    TrajectoryDivergence,
};

// ═══════════════════════════════════════════════════════════════════════════════
// Trajectory comparison helper
// ═══════════════════════════════════════════════════════════════════════════════

/// Compare CortenForge trajectory against MuJoCo reference.
///
/// Runs `data.step(&model)` for `nsteps` times, comparing qpos/qvel/qacc
/// at each step against reference .npy data.
///
/// Panics with full diagnostic report if any divergence exceeds tolerance.
fn compare_trajectory(
    model_name: &str,
    ctrl_values: &[f64],
    nsteps: usize,
    base_tol: f64,
    growth: f64,
    has_free_joint: bool,
) {
    let (model, mut data) = common::load_conformance_model(model_name);

    // Set ctrl once before stepping (matches gen script behavior)
    for (i, &v) in ctrl_values.iter().enumerate() {
        data.ctrl[i] = v;
    }

    // Load reference trajectories
    let (qpos_shape, ref_qpos) = common::load_reference_f64(model_name, "trajectory", "qpos");
    let (qvel_shape, ref_qvel) = common::load_reference_f64(model_name, "trajectory", "qvel");
    let (_qacc_shape, ref_qacc) = common::load_reference_f64(model_name, "trajectory", "qacc");

    // Validate shapes
    let nq = qpos_shape[1];
    let nv = qvel_shape[1];
    assert_eq!(
        qpos_shape[0], nsteps,
        "[{model_name}] trajectory: qpos has {} steps, expected {nsteps}",
        qpos_shape[0]
    );
    assert_eq!(
        qvel_shape[0], nsteps,
        "[{model_name}] trajectory: qvel has {} steps, expected {nsteps}",
        qvel_shape[0]
    );
    assert_eq!(nq, model.nq, "[{model_name}] trajectory: nq mismatch");
    assert_eq!(nv, model.nv, "[{model_name}] trajectory: nv mismatch");

    let qacc_base = base_tol * TRAJ_QACC_FACTOR;
    let mut divergences: Vec<TrajectoryDivergence> = Vec::new();

    // Step loop: step N times, compare at each step
    for step in 0..nsteps {
        data.step(&model)
            .unwrap_or_else(|e| panic!("[{model_name}] step {step} failed: {e}"));

        let tol_qpos = common::step_tolerance(base_tol, growth, step);
        let tol_qvel = common::step_tolerance(base_tol, growth, step);
        let tol_qacc = common::step_tolerance(qacc_base, growth, step);

        // Compare qpos
        for d in 0..nq {
            let expected = ref_qpos[step * nq + d];
            let actual = data.qpos[d];

            // Free joint quaternion: indices 3..7 need sign-aware comparison
            if has_free_joint && (3..7).contains(&d) {
                continue; // Handled in quaternion block below
            }

            let diff = (expected - actual).abs();
            if diff > tol_qpos {
                divergences.push(TrajectoryDivergence {
                    step,
                    field: "qpos",
                    dof: d,
                    expected,
                    actual,
                    diff,
                    tol: tol_qpos,
                });
            }
        }

        // Free joint quaternion sign-aware comparison for qpos[3..7]
        if has_free_joint && nq >= 7 {
            let ref_q = [
                ref_qpos[step * nq + 3],
                ref_qpos[step * nq + 4],
                ref_qpos[step * nq + 5],
                ref_qpos[step * nq + 6],
            ];
            let cf_q = [data.qpos[3], data.qpos[4], data.qpos[5], data.qpos[6]];
            let dist_pos = (0..4)
                .map(|k| (cf_q[k] - ref_q[k]).abs())
                .fold(0.0f64, f64::max);
            let dist_neg = (0..4)
                .map(|k| (cf_q[k] + ref_q[k]).abs())
                .fold(0.0f64, f64::max);
            let dist = dist_pos.min(dist_neg);
            if dist > tol_qpos {
                divergences.push(TrajectoryDivergence {
                    step,
                    field: "qpos (quat)",
                    dof: 3, // quaternion block starts at index 3
                    expected: ref_q[0],
                    actual: cf_q[0],
                    diff: dist,
                    tol: tol_qpos,
                });
            }
        }

        // Compare qvel
        for d in 0..nv {
            let expected = ref_qvel[step * nv + d];
            let actual = data.qvel[d];
            let diff = (expected - actual).abs();
            if diff > tol_qvel {
                divergences.push(TrajectoryDivergence {
                    step,
                    field: "qvel",
                    dof: d,
                    expected,
                    actual,
                    diff,
                    tol: tol_qvel,
                });
            }
        }

        // Compare qacc
        for d in 0..nv {
            let expected = ref_qacc[step * nv + d];
            let actual = data.qacc[d];
            let diff = (expected - actual).abs();
            if diff > tol_qacc {
                divergences.push(TrajectoryDivergence {
                    step,
                    field: "qacc",
                    dof: d,
                    expected,
                    actual,
                    diff,
                    tol: tol_qacc,
                });
            }
        }
    }

    // Report all divergences
    if !divergences.is_empty() {
        let first = &divergences[0];
        let worst = divergences
            .iter()
            .max_by(|a, b| {
                (a.diff / a.tol)
                    .partial_cmp(&(b.diff / b.tol))
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap();
        let n_steps_clean = first.step;

        let mut msg = format!(
            "\n[{model_name}] TRAJECTORY DIVERGENCE — {n_steps_clean}/{nsteps} \
             steps matched before first divergence.\n\n"
        );
        msg += &format!(
            "  First divergence: step {}, {}.dof[{}]: \
             expected {:.15e}, got {:.15e}, diff {:.3e}, tol {:.3e}\n",
            first.step, first.field, first.dof, first.expected, first.actual, first.diff, first.tol,
        );
        msg += &format!(
            "  Worst divergence: step {}, {}.dof[{}]: \
             expected {:.15e}, got {:.15e}, diff {:.3e}, tol {:.3e} ({:.1}× tol)\n",
            worst.step,
            worst.field,
            worst.dof,
            worst.expected,
            worst.actual,
            worst.diff,
            worst.tol,
            worst.diff / worst.tol,
        );
        msg += &format!(
            "  Total divergences: {} across {} steps\n",
            divergences.len(),
            nsteps,
        );
        panic!("{msg}");
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// S2: Non-contact trajectory tests (T1–T2)
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore] // CONFORMANCE GAP: qacc wrong from step 1 — CRBA/RNE xipos cascade — Phase 1 FK
fn layer_c_trajectory_pendulum() {
    compare_trajectory(
        "pendulum",
        &[], // no ctrl
        100, // steps
        TRAJ_BASE_SMOOTH,
        TRAJ_GROWTH_SMOOTH,
        false, // no free joint
    );
}

#[test]
#[ignore] // CONFORMANCE GAP: qacc wrong from step 1 — CRBA/RNE xipos cascade — Phase 1 FK
fn layer_c_trajectory_double_pendulum() {
    compare_trajectory(
        "double_pendulum",
        &[], // no ctrl
        100, // steps
        TRAJ_BASE_SMOOTH,
        TRAJ_GROWTH_SMOOTH,
        false, // no free joint
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// S3: Contact trajectory test (T3)
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore] // CONFORMANCE GAP: constraint forces wrong — contact.pos convention + efc_J assembly — Phase 3 collision/constraint
fn layer_c_trajectory_contact_scenario() {
    compare_trajectory(
        "contact_scenario",
        &[], // no ctrl
        100, // steps
        TRAJ_BASE_CHAOTIC,
        TRAJ_GROWTH_CHAOTIC,
        true, // has free joint — qpos[3..7] quaternion
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// S4: Actuated trajectory test (T4)
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore] // CONFORMANCE GAP: qacc wrong from step 1 — CRBA/RNE xipos cascade — Phase 1 FK
fn layer_c_trajectory_actuated_system() {
    compare_trajectory(
        "actuated_system",
        &[1.0, 0.5], // motor + position servo
        100,         // steps
        TRAJ_BASE_SMOOTH,
        TRAJ_GROWTH_SMOOTH,
        false, // no free joint
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// S5: Subsystem trajectory tests (T5–T7)
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore] // CONFORMANCE GAP: qacc wrong from step 1 — CRBA/RNE xipos cascade — Phase 1 FK
fn layer_c_trajectory_tendon_model() {
    compare_trajectory(
        "tendon_model",
        &[], // no ctrl
        100, // steps
        TRAJ_BASE_SMOOTH,
        TRAJ_GROWTH_SMOOTH,
        false, // no free joint
    );
}

#[test]
#[ignore] // CONFORMANCE GAP: qacc wrong from step 1 — CRBA/RNE xipos cascade — Phase 1 FK
fn layer_c_trajectory_sensor_model() {
    compare_trajectory(
        "sensor_model",
        &[], // no ctrl
        100, // steps
        TRAJ_BASE_SMOOTH,
        TRAJ_GROWTH_SMOOTH,
        false, // no free joint
    );
}

#[test]
#[ignore] // CONFORMANCE GAP: qacc wrong from step 1 + constraint Jacobian wrong — Phase 1 FK + Phase 3 constraint
fn layer_c_trajectory_equality_model() {
    compare_trajectory(
        "equality_model",
        &[], // no ctrl
        100, // steps
        TRAJ_BASE_SMOOTH,
        TRAJ_GROWTH_SMOOTH,
        false, // no free joint
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// S6: Composite trajectory test (T8)
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore] // CONFORMANCE GAP: qacc wrong from step 1 + constraint wrong — Phase 1 FK + Phase 3 collision/constraint
fn layer_c_trajectory_composite_model() {
    compare_trajectory(
        "composite_model",
        &[1.0], // motor
        200,    // steps — longest trajectory
        TRAJ_BASE_CHAOTIC,
        TRAJ_GROWTH_CHAOTIC,
        false, // no free joint (4 hinge joints)
    );
}
