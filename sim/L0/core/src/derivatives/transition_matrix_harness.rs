//! Differential-testing harness for [`Data::transition_derivatives`].
//!
//! The analytical state-transition Jacobian `A` (the `use_analytical: true`
//! path through [`super::hybrid::mjd_transition_hybrid`], which routes the
//! position columns through `mjd_rne_pos`) must equal a central finite
//! difference of the same one-step map. The single legacy regression
//! ([`super::hybrid::tests::analytical_transition_matches_fd_nonparallel_chain`])
//! only ever exercised an **all-hinge, single-joint-per-body serial chain** —
//! which is exactly why three silent-wrong derivative bugs hid there at once
//! (the parallel-axis chain zeroes the very terms they corrupt).
//!
//! This harness sweeps a MATRIX of joint types `{hinge, slide, ball, free}`
//! across topologies `{single, parallel/spatial chains, multi-joint-per-body,
//! branched}` plus a spatial-tendon-with-length-spring case, with **non-root
//! ball/free** bodies (`hinge → ball`, `hinge → free`) — the gaps only bite
//! once the parent moves and the body's own velocity subspace rotates. It is
//! the permanent guard so future leaves (damped chain, quaternion carry,
//! integrators) cannot silently regress one joint type while another stays
//! green.
//!
//! Each case asserts `max_relative_error(analytic.A, fd.A, floor) < TOL`. The
//! floor ignores the near-zero position↔velocity entries (`~Δt`) whose FD
//! noise would otherwise dominate the relative metric; the velocity↔position
//! block (where the bugs live) is `O(0.1–1)`.
#![allow(clippy::expect_used)]

use super::{DerivativeConfig, max_relative_error, mjd_transition_fd};
use crate::test_fixtures::conformance::{ConformanceCase, dynamics_conformance_matrix};
use crate::types::{Data, Model};

/// The conformance matrix this harness sweeps. Construction lives in the shared
/// [`crate::test_fixtures::conformance`] module so the CPU analytic-vs-FD check
/// here and the GPU-vs-CPU dynamics conformance suite (`sim-gpu`) draw from ONE
/// list and cannot silently drift to different model sets. This wrapper is the
/// only seam — keep it thin.
fn matrix() -> Vec<ConformanceCase> {
    dynamics_conformance_matrix()
}

/// Set the operating point and run forward dynamics so `data` is consistent.
fn forward_at(model: &Model, qpos: &[f64], qvel: &[f64]) -> Data {
    let mut data = model.make_data();
    data.qpos.as_mut_slice().copy_from_slice(qpos);
    data.qvel.as_mut_slice().copy_from_slice(qvel);
    data.forward(model).expect("forward");
    data
}

/// Per-case ceiling on `max_relative_error(analytic.A, fd.A)`. **Every case is
/// now machine-exact** (`< MACHINE_TOL`) — there are no remaining known limits.
/// The parentheticals below record the cases the harness surfaced and the
/// subsystem fixes that closed each (kept as a map from symptom → root cause for
/// the next person who touches this code), rather than silently widening the tol.
///
/// (`multi_joint_body` was a known limit here — a multi-joint body's earlier
/// joint had its motion subspace built from the body's FINAL orientation rather
/// than the PARTIAL frame that precedes the later joints' rotations. It is now
/// machine-exact: forward kinematics stores per-joint partial-frame `xaxis`/
/// `xanchor`, consumed by `joint_motion_subspace`, `velocity.rs`, and the
/// `rne.rs` gravity projection, and the analytic `∂S/∂q` in `mjd_rne_pos`
/// restricts the same-body cross-term to joints applied at-or-before `j`.)
///
/// (`hinge_then_free` was a known limit here — a free child's ancestor-DOF bias
/// derivative. A Free joint sets its body's world pose ABSOLUTELY (`position.rs`
/// overwrites `pos`/`quat` from qpos), so the free body's frame — and hence its
/// velocity, inertia, and motion subspace — is INDEPENDENT of every ancestor DOF
/// (FD confirms `∂cvel/∂q_ancestor = ∂qfrc_bias/∂q_ancestor = 0` exactly). The
/// `mjd_rne_pos` passes treated it as a relative child (applying `axis×r`
/// transport, ancestor `∂S/∂q`, ancestor `∂I/∂q`), which is spurious. It is now
/// machine-exact: for a free body the ancestor `∂S/∂q` and `∂I/∂q` terms are
/// dropped and the offset derivative uses `∂r/∂q = −axis×(xpos[parent]−xanchor)`
/// — only the parent origin moves — across all six transport/subspace/inertia/
/// projection sites in `mjd_rne_pos`.)
///
/// (`hinge_offset_pivot` was a known limit here — a hinge with `jnt_pos ≠ 0`
/// rotates the body about an anchor off the body origin, so the origin moves at
/// `â×r` (r = origin − anchor). The residual was NOT in the derivative: the
/// FORWARD `velocity.rs` built a hinge's `cvel` from the angular axis ONLY,
/// omitting the linear lever `â×r` of the motion subspace `S = [â; â×r]` — so
/// `cvel ≠ S·q̇` for offset pivots, while `mjd_rne_pos` (and CRBA, `mj_jac`) all
/// project with the lever-correct `S`. The analytic derivative differentiated the
/// consistent `S·q̇`, but the operating point did not, leaving the ~5e-3 mismatch.
/// It is now machine-exact: `velocity.rs` adds the `â×r` lever (byte-identical for
/// `jnt_pos = 0`, where `r = 0`), restoring `cvel = S·q̇`. Guarded by
/// `forward_cvel_matches_joint_subspace` extended to a `jnt_pos ≠ 0` body.)
fn known_limit(_name: &str) -> Option<f64> {
    // No known limitations remain — every case in the matrix is machine-exact.
    None
}

/// The harness assertion: analytic transition `A` matches central-FD `A` for
/// every joint type × topology in the matrix. Cases without a [`known_limit`]
/// must be machine-exact; listed cases are regression-guarded at their bound.
#[test]
fn analytical_transition_matches_fd_across_joint_matrix() {
    // Floor matches the legacy regression: ignore the `~Δt` position↔velocity
    // entries whose FD noise dominates the relative metric.
    const FLOOR: f64 = 1e-3;
    const MACHINE_TOL: f64 = 1e-5;

    let mut failures = Vec::new();
    for case in matrix() {
        let data = forward_at(&case.model, &case.qpos, &case.qvel);

        let analytic = data
            .transition_derivatives(
                &case.model,
                &DerivativeConfig {
                    use_analytical: true,
                    ..Default::default()
                },
            )
            .expect("analytical transition");
        let fd = mjd_transition_fd(
            &case.model,
            &data,
            &DerivativeConfig {
                use_analytical: false,
                ..Default::default()
            },
        )
        .expect("FD transition");

        let (err, loc) = max_relative_error(&analytic.A, &fd.A, FLOOR);
        let ceiling = known_limit(case.name).unwrap_or(MACHINE_TOL);
        if !(err < ceiling) {
            let tag = if known_limit(case.name).is_some() {
                "known-limit REGRESSED"
            } else {
                "not machine-exact"
            };
            failures.push(format!(
                "{} → rel {err:.3e} at {loc:?} (ceiling {ceiling:.1e}, {tag})",
                case.name
            ));
        }
    }

    assert!(
        failures.is_empty(),
        "analytic transition A diverged from central-FD for:\n  {}",
        failures.join("\n  ")
    );
}
