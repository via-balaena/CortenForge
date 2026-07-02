//! Closed-loop POLICY gradient THROUGH the friction grip — the invariants the coupling gradient
//! harness can't express.
//!
//! The per-horizon FD cells of `StaggeredCoupling::coupled_trajectory_policy_friction_gradient`
//! (the de-escalation agent that DECIDES how to actuate the grip from the limb's state — the control
//! is a differentiable policy `u_k = π_θ(qpos₀, qvel₀)`, so one `tape.backward` gives `∂tip_x/∂θ`
//! across the state→control recurrence, backprop-through-time under friction) are folded into the
//! `hinge·policy-friction[θ]` and `chain·policy-friction[θ]` rows of `tests/coupling_grad_harness.rs`,
//! which assert the same per-parameter FD match against the same
//! `coupled_trajectory_policy_gripped_x(θ, n).x` oracle (eps 1e-6), plus forward-consistency and a
//! `Comp::Live` non-vacuity floor.
//!
//! What those single-horizon rows CAN'T fold, kept here:
//! - the **single-hinge multi-horizon machine-exactness** — the policy gradient stays FD-limited as
//!   the rollout lengthens (n = 5, 20, 40), with the tip staying engaged near the block top (the
//!   z-band the tip-`x` loss can't express);
//! - the **friction-ON-vs-OFF materiality** — at the same policy the gripped rollout differs from
//!   the frictionless one, so the friction path is genuinely live (a toggle the matrix, which only
//!   ever builds the friction scene, can't express).

#![allow(clippy::expect_used)]

use sim_coupling::{LinearFeedback, StaggeredCoupling};
use sim_mjcf::load_model;

// A single-hinge arm with a motor actuator, pushed sideways by tilted gravity — the policy
// observes the joint state (angle, rate) and drives the gripped limb.
fn scene() -> &'static str {
    r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
  <actuator><motor joint="j" gear="1"/></actuator>
</mujoco>"#
}

const SOFT_MU: f64 = 3.0e3;
const FRIC_MU: f64 = 2.5;
const EPS_V: f64 = 0.1;

fn build(friction: bool) -> StaggeredCoupling {
    let model = load_model(scene()).expect("actuator scene loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).expect("forward");
    let c = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, SOFT_MU, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    );
    if friction {
        c.with_friction(FRIC_MU, EPS_V)
    } else {
        c
    }
}

// A linear joint-feedback policy: u = w_qpos·qpos + w_qvel·qvel + b.
const PARAMS: [f64; 3] = [0.08, -0.02, 0.01];

/// The one-tape policy gradient `∂tip_x/∂θ` (backprop-through-time) vs a per-parameter central FD
/// of the independent policy-driven gripped rollout, plus a forward-match and an engagement check.
/// Single hinge ⇒ analytic friction-loaded carry (machine-exact).
fn gate_single(n: usize) {
    let (tip_x, grad) =
        build(true).coupled_trajectory_policy_friction_gradient(&LinearFeedback, &PARAMS, n);
    let oracle = build(true).coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, n);
    assert_eq!(grad.len(), PARAMS.len());
    assert!(
        (tip_x - oracle.x).abs() < 1e-12,
        "n={n}: tape forward tip_x {tip_x} != policy-gripped rollout {}",
        oracle.x
    );
    assert!(
        (0.10..0.12).contains(&oracle.z),
        "n={n}: arm should stay engaged near the block top, got z={}",
        oracle.z
    );
    let eps = 1e-6;
    let mut max_rel = 0.0_f64;
    for j in 0..PARAMS.len() {
        let mut pp = PARAMS;
        let mut pm = PARAMS;
        pp[j] += eps;
        pm[j] -= eps;
        let fd = (build(true)
            .coupled_trajectory_policy_gripped_x(&LinearFeedback, &pp, n)
            .x
            - build(true)
                .coupled_trajectory_policy_gripped_x(&LinearFeedback, &pm, n)
                .x)
            / (2.0 * eps);
        assert!(
            fd.abs() > 1e-9,
            "n={n}: ∂tip_x/∂θ_{j} should be a nonzero, well-posed target (fd={fd:e})"
        );
        max_rel = max_rel.max((grad[j] - fd).abs() / fd.abs().max(1e-9));
    }
    assert!(
        max_rel < 1e-5,
        "n={n}: policy-friction gradient must match the policy-gripped FD, worst rel {max_rel:.3e}"
    );
}

/// SINGLE-HINGE multi-horizon machine-exactness (the cross-horizon invariant + z-engagement the
/// single-length matrix rows can't express): the backprop-through-time policy gradient stays
/// FD-limited across the make and the sweep as the rollout lengthens.
#[test]
fn policy_friction_gradient_machine_exact_at_all_lengths() {
    for n in [5usize, 20, 40] {
        gate_single(n);
    }
}

/// FRICTION-ON-vs-OFF materiality (the toggle the matrix — which only ever builds the friction
/// scene — can't express): at the same policy the gripped rollout differs from the frictionless one,
/// so the friction path is genuinely live rather than a silent pass-through.
#[test]
fn friction_toggle_changes_the_drag() {
    let with = build(true)
        .coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, 20)
        .x;
    let without = build(false)
        .coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, 20)
        .x;
    assert!(
        (with - without).abs() > 1e-6,
        "friction must change the tangential drag (Δx = {:e}); the grip path looks inert",
        (with - without).abs()
    );
}
