//! Keystone closed-loop policy gradient — the feedback-policy substrate.
//!
//! `StaggeredCoupling::coupled_trajectory_policy_gradient` rolls the coupled
//! soft↔rigid system forward under a **closed-loop feedback policy**
//! `u_k = π_θ(state_k)` ([`LinearFeedback`]: `u = w_z·z + w_vz·vz + b`), on ONE
//! chassis tape, then a single `tape.backward(z_N)` gives `∂z_N/∂θ` — the gradient
//! the closed-loop policy half of the co-design optimizer consumes.
//!
//! Unlike the open-loop control gradient (each `u_k` an independent leaf), here θ
//! is shared across every step and the control at step k is a tape NODE built from
//! the loop-carried state vars `z_var`/`vz_var`. Those state vars depend on the
//! policy's own earlier outputs, so the reverse pass accumulates `∂z_N/∂θ` over the
//! whole state→control recurrence (backprop-through-time) automatically.
//!
//! Gate: the one-tape gradient matches a central FD of the FULL real **closed-loop**
//! coupled re-rollout (a fresh coupling rolled at `θ ± ε` via
//! `coupled_trajectory_policy_z`, the policy re-evaluated each step). That oracle
//! re-runs the real Newton solves + sim-core steps + the policy recurrence each
//! perturbation, so it is INDEPENDENT (not an affine identity). The `w_z`/`w_vz`
//! gradients exist *only* through the recurrence, so their FD-exactness is the
//! decisive evidence that closed-loop BPTT is correct.

#![allow(clippy::expect_used)]

use sim_coupling::{LinearFeedback, StaggeredCoupling};
use sim_mjcf::load_model;

// A platen started already in contact (plane at z−clearance = 0.103, the soft
// block's top face at z = 0.1) so the rollout is firmly engaged — the regime the
// gradient is scoped to. Identical to the control trajectory gate's fixture.
const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.108">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const MU0: f64 = 3.0e4;
const DAMPING: f64 = 60.0;

fn build() -> StaggeredCoupling {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, MU0, 1.0e-3, 3.0e4, 1.0e-2, DAMPING,
    )
}

/// Final platen height after the real closed-loop rollout under the policy params.
fn final_z(theta: &[f64], n: usize) -> f64 {
    build().coupled_trajectory_policy_z(&LinearFeedback, theta, n)
}

/// The tape's forward rollout reproduces the real coupled dynamics exactly under a
/// closed-loop policy (the nodes carry real `step` values, and the policy node's
/// value equals `eval` at the real state; only the backward pass is analytic).
#[test]
fn policy_trajectory_forward_matches_real_rollout() {
    // A genuine feedback law: restoring (negative w_z) + velocity damping (negative
    // w_vz) + a positive bias around the platen weight.
    let theta = [-20.0_f64, -5.0, 2.0];
    let n = 12;
    let (z_n, _grad) = build().coupled_trajectory_policy_gradient(&LinearFeedback, &theta, n);
    let z_ref = final_z(&theta, n);
    assert!(
        (z_n - z_ref).abs() < 1e-12,
        "tape forward z_N {z_n} != real rollout {z_ref}"
    );
}

/// One `tape.backward` over the closed-loop rollout gives `∂z_N/∂θ` for EVERY
/// policy parameter, each matching the full-coupled closed-loop FD oracle to
/// machine precision — including the feedback weights `w_z`/`w_vz`, whose gradient
/// flows *only* through the recurrence.
#[test]
fn policy_gradient_matches_full_fd() {
    let theta = [-20.0_f64, -5.0, 2.0];
    let n = 12;

    let (_z, grad) = build().coupled_trajectory_policy_gradient(&LinearFeedback, &theta, n);
    assert_eq!(grad.len(), 3, "one gradient per policy parameter");

    let names = ["w_z", "w_vz", "b"];
    for (i, &g) in grad.iter().enumerate() {
        // Weights multiply state of O(0.1) (z) / O(0.01) (vz); the bias is a direct
        // O(1)-newton force. A modest ε keeps the FD well above its float floor.
        let eps = if i == 2 { 1e-3 } else { 1e-2 };
        let mut up = theta;
        let mut dn = theta;
        up[i] += eps;
        dn[i] -= eps;
        let fd = (final_z(&up, n) - final_z(&dn, n)) / (2.0 * eps);
        let rel = (g - fd).abs() / fd.abs().max(1e-30);
        let abs = (g - fd).abs();
        eprintln!(
            "θ[{}]={}: tape={g:.6e} fd={fd:.6e} rel={rel:.3e} abs={abs:.3e}",
            i, names[i]
        );
        // Machine-exact (~1e-11) since the policy rides the off-by-one-free rigid
        // carry and the chassis autograd carries ∂u/∂θ, ∂u/∂state exactly; gate at
        // 1e-5 (the w_vz FD bottoms near its float floor ~1e-8) so a glue-Jacobian
        // regression — invisible to the real-step forward rollout — is caught.
        assert!(
            rel < 1e-5 || abs < 1e-12,
            "∂z_N/∂{} tape {g} vs closed-loop FD {fd} (rel {rel:e} abs {abs:e})",
            names[i]
        );
    }
}

/// The feedback weights carry nonzero gradient — the recurrence is genuinely
/// exercised (a degenerate "policy" that ignored state would give `∂z_N/∂w_z =
/// ∂z_N/∂w_vz = 0`). This pins that the closed-loop paths are live, not just the
/// bias (which behaves like the open-loop schedule).
#[test]
fn feedback_weights_carry_gradient() {
    let theta = [-20.0_f64, -5.0, 2.0];
    let (_z, grad) = build().coupled_trajectory_policy_gradient(&LinearFeedback, &theta, 12);
    assert!(
        grad[0].abs() > 1e-9 && grad[1].abs() > 1e-9,
        "feedback-weight gradients should be nonzero (recurrence live), got {grad:?}"
    );
}
