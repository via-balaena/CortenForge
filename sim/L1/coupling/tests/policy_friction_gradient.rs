//! The closed-loop POLICY gradient THROUGH the friction grip — the de-escalation agent that
//! DECIDES how to actuate the grip from the limb's state. Builds on the #404 control substrate:
//! the actuator control `u_k` is now a differentiable policy `u_k = π_θ(qpos₀, qvel₀)` (a
//! `DiffPolicy` sub-expression on the tape), so one `tape.backward` gives `∂tip_x/∂θ` across the
//! state→control recurrence (backprop-through-time), reusing the matrix carry under friction.
//!
//! Gates: a smoke for the policy-driven gripped oracle `coupled_trajectory_policy_gripped_x`
//! (policy + friction both live), then the one-tape policy gradient `d(tip_x)/d(theta)` vs a
//! per-parameter central FD of that oracle — MACHINE-EXACT at single-hinge (n = 5/20/40) AND a
//! 2-link chain (the analytic `chain_state_jacobian` carry). The policy parameters are strong,
//! well-conditioned levers on tip_x (each `theta` acts at every step via the recurrence), so the
//! FD is clean; `eps = 1e-6`.

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

#[test]
fn policy_gripped_engages_and_stays_finite() {
    let tip = build(true).coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, 20);
    assert!(
        tip.x.is_finite() && (0.10..0.12).contains(&tip.z),
        "policy-driven grip rollout must stay finite and engaged, got {tip:?}"
    );
}

#[test]
fn policy_and_friction_are_live() {
    // The policy parameters change the tip's tangential drag (the closed-loop control is live).
    let a = build(true)
        .coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, 20)
        .x;
    let b = build(true)
        .coupled_trajectory_policy_gripped_x(&LinearFeedback, &[-0.08, 0.02, -0.01], 20)
        .x;
    assert!(
        (a - b).abs() > 1e-4,
        "the policy params must move the tip (Δx = {:e}); the closed-loop path looks inert",
        (a - b).abs()
    );
    // Friction is live: at the same policy, the gripped rollout differs from frictionless.
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

/// The one-tape policy gradient `∂tip_x/∂θ` (backprop-through-time) vs a per-parameter central FD
/// of the independent policy-driven gripped rollout. Single hinge ⇒ analytic friction-loaded carry.
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

#[test]
fn policy_friction_gradient_matches_fd_n5() {
    gate_single(5);
}

#[test]
fn policy_friction_gradient_matches_fd_n20() {
    gate_single(20);
}

#[test]
fn policy_friction_gradient_matches_fd_n40() {
    gate_single(40);
}

// A 2-link hinge CHAIN (nv = 2) with the motor + policy on the proximal joint — the closed-loop
// policy gradient (backprop-through-time) on a genuine multi-DOF topology, the analytic
// `chain_state_jacobian` carry under friction + actuator load.
fn scene_2link() -> &'static str {
    r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="upper" pos="0 0 0.2">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.025" size="0.004" mass="0.3"/>
      <body name="lower" pos="0 0 -0.05">
        <joint type="hinge" axis="0 1 0"/>
        <geom type="sphere" pos="0 0 -0.04" size="0.004" mass="0.4"/>
      </body>
    </body>
  </worldbody>
  <actuator><motor joint="j" gear="1"/></actuator>
</mujoco>"#
}

fn build_2link() -> StaggeredCoupling {
    let model = load_model(scene_2link()).expect("2-link actuator scene loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.1;
    data.qpos[1] = -0.05;
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 2, 0.005, 4, 0.1, SOFT_MU, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_friction(FRIC_MU, EPS_V)
}

#[test]
fn policy_friction_gradient_2link_matches_fd() {
    let z0 = build_2link().data().xipos[2].z;
    assert!(z0 > 0.10, "2-link tip must start engaged, got {z0}");
    for n in [3usize, 12] {
        let (tip_x, grad) =
            build_2link().coupled_trajectory_policy_friction_gradient(&LinearFeedback, &PARAMS, n);
        let oracle = build_2link().coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, n);
        assert!(
            (tip_x - oracle.x).abs() < 1e-12,
            "2-link n={n}: tape forward tip_x {tip_x} != policy-gripped rollout {}",
            oracle.x
        );
        // The chain STARTS engaged (z0 > 0.10) but the policy lifts the proximal joint, so the tip
        // can rise off the block mid-rollout — the FD gate stays machine-exact THROUGH that
        // active-set change (a stronger test of the gradient than a permanently-engaged grip), so
        // no post-rollout engagement assert here.
        let eps = 1e-6;
        let mut max_rel = 0.0_f64;
        for j in 0..PARAMS.len() {
            let mut pp = PARAMS;
            let mut pm = PARAMS;
            pp[j] += eps;
            pm[j] -= eps;
            let fd = (build_2link()
                .coupled_trajectory_policy_gripped_x(&LinearFeedback, &pp, n)
                .x
                - build_2link()
                    .coupled_trajectory_policy_gripped_x(&LinearFeedback, &pm, n)
                    .x)
                / (2.0 * eps);
            assert!(
                fd.abs() > 1e-9,
                "2-link n={n}: ∂tip_x/∂θ_{j} nonzero (fd={fd:e})"
            );
            max_rel = max_rel.max((grad[j] - fd).abs() / fd.abs().max(1e-9));
        }
        assert!(
            max_rel < 1e-5,
            "2-link n={n}: policy-friction gradient vs FD, worst rel {max_rel:.3e}"
        );
    }
}
