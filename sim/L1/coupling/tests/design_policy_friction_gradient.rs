//! Design + policy on ONE friction-grip tape — the mission's "one outer loop over BOTH design and
//! policy". The articulated-grip analog of the free-platen `coupled_trajectory_joint_gradient`: a
//! clone of the #405 closed-loop policy gradient with the soft material `μ` lifted to a
//! DIFFERENTIATED leaf (via the combined grip node `trajectory_step_vjp_grip_combined(&[1, 4])`, so
//! the `λ = 4μ` tie rides one tape parent), so one `tape.backward(tip_x)` yields BOTH
//! `(tip_x, ∂tip_x/∂μ_total, [∂tip_x/∂θ])` at once.
//!
//! Gates both channels of `coupled_trajectory_design_policy_friction_gradient` vs central FD of the
//! shared `coupled_trajectory_policy_gripped_x` oracle (μ-driven by construction, θ via the policy):
//! the μ channel via a build-perturbation `soft_mu ± ε` (which moves μ AND λ = 4μ together, so it
//! measures `∂/∂μ_total` — matching the node's `[1, 4]` weights), and the θ channel via `params ± ε`.
//! MACHINE-EXACT at single-hinge (n = 5/20/40) AND a 2-link chain. The μ FD uses a RELATIVE step
//! `ε = μ·1e-4` (μ ≈ 3e3 is large; an absolute step underflows — the #403 conditioning lesson); the
//! well-conditioned policy levers take `ε = 1e-6`.
//!
//! The single-point FD cells of this channel are now the `{hinge, chain, moving-ee, sphere}·design-
//! policy-friction[μ+θ]` rows of `coupling_grad_harness.rs`. What stays HERE is what the single-n
//! matrix rows structurally can't express: the single-hinge MULTI-HORIZON machine-exact sweep
//! (`gate_single` at n = 5 / 20 / 40, spanning make / slide / liftoff) and the 2-link chain's
//! machine-exactness THROUGH a mid-rollout active-set change (the tip lifts off the block), with its
//! start-engagement guard — a stronger gradient test than a permanently-engaged single grip.

#![allow(clippy::expect_used)]

use sim_coupling::{LinearFeedback, StaggeredCoupling};
use sim_mjcf::load_model;

const MU0: f64 = 3.0e3;
const FRIC_MU: f64 = 2.5;
const EPS_V: f64 = 0.1;

// A linear joint-feedback policy: u = w_qpos·qpos + w_qvel·qvel + b.
const PARAMS: [f64; 3] = [0.08, -0.02, 0.01];

// A single-hinge arm with a motor actuator, pushed sideways by tilted gravity — the policy observes
// the joint state (angle, rate) and drives the gripped limb into the soft buffer (μ = soft_mu).
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

fn build(soft_mu: f64) -> StaggeredCoupling {
    let model = load_model(scene()).expect("actuator scene loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, soft_mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_friction(FRIC_MU, EPS_V)
}

/// Both channels of the one-tape design+policy gradient vs central FD of the policy-gripped oracle.
/// Single hinge ⇒ analytic friction-loaded carry; μ moves the soft re-equilibration, θ the control.
fn gate_single(n: usize) {
    let (tip_x, mu_grad, theta_grad) =
        build(MU0).coupled_trajectory_design_policy_friction_gradient(&LinearFeedback, &PARAMS, n);
    let oracle = build(MU0).coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, n);
    assert_eq!(theta_grad.len(), PARAMS.len());
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

    // (1) DESIGN channel: ∂tip_x/∂μ_total via a build-perturbation soft_mu ± ε (μ and λ = 4μ move
    // together, so this measures the tied total — the [1, 4] weights). Relative ε (μ is large).
    let eps_mu = MU0 * 1e-4;
    let fd_mu = (build(MU0 + eps_mu)
        .coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, n)
        .x
        - build(MU0 - eps_mu)
            .coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, n)
            .x)
        / (2.0 * eps_mu);
    assert!(
        fd_mu.abs() > 1e-9,
        "n={n}: ∂tip_x/∂μ_total should be a nonzero, well-posed target (fd={fd_mu:e})"
    );
    let rel_mu = (mu_grad - fd_mu).abs() / fd_mu.abs().max(1e-9);
    assert!(
        rel_mu < 1e-5,
        "n={n}: design (μ) gradient {mu_grad:e} vs FD {fd_mu:e}, rel {rel_mu:.3e}"
    );

    // (2) POLICY channel: ∂tip_x/∂θ via params ± ε (well-conditioned levers, absolute ε).
    let eps = 1e-6;
    let mut max_rel = 0.0_f64;
    for j in 0..PARAMS.len() {
        let mut pp = PARAMS;
        let mut pm = PARAMS;
        pp[j] += eps;
        pm[j] -= eps;
        let fd = (build(MU0)
            .coupled_trajectory_policy_gripped_x(&LinearFeedback, &pp, n)
            .x
            - build(MU0)
                .coupled_trajectory_policy_gripped_x(&LinearFeedback, &pm, n)
                .x)
            / (2.0 * eps);
        assert!(
            fd.abs() > 1e-9,
            "n={n}: ∂tip_x/∂θ_{j} should be a nonzero, well-posed target (fd={fd:e})"
        );
        max_rel = max_rel.max((theta_grad[j] - fd).abs() / fd.abs().max(1e-9));
    }
    assert!(
        max_rel < 1e-5,
        "n={n}: policy gradient must match the policy-gripped FD, worst rel {max_rel:.3e}"
    );
}

#[test]
fn design_policy_gradient_matches_fd_n5() {
    gate_single(5);
}

#[test]
fn design_policy_gradient_matches_fd_n20() {
    gate_single(20);
}

#[test]
fn design_policy_gradient_matches_fd_n40() {
    gate_single(40);
}

// A 2-link hinge CHAIN (nv = 2) with the motor + policy on the proximal joint — design + policy
// (backprop-through-time) on a genuine multi-DOF topology, the analytic `chain_state_jacobian` carry
// under friction + actuator load, with the material design leaf live.
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

fn build_2link(soft_mu: f64) -> StaggeredCoupling {
    let model = load_model(scene_2link()).expect("2-link actuator scene loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.1;
    data.qpos[1] = -0.05;
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 2, 0.005, 4, 0.1, soft_mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_friction(FRIC_MU, EPS_V)
}

#[test]
fn design_policy_gradient_2link_matches_fd() {
    let z0 = build_2link(MU0).data().xipos[2].z;
    assert!(z0 > 0.10, "2-link tip must start engaged, got {z0}");
    for n in [3usize, 12] {
        let (tip_x, mu_grad, theta_grad) = build_2link(MU0)
            .coupled_trajectory_design_policy_friction_gradient(&LinearFeedback, &PARAMS, n);
        let oracle =
            build_2link(MU0).coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, n);
        assert!(
            (tip_x - oracle.x).abs() < 1e-12,
            "2-link n={n}: tape forward tip_x {tip_x} != policy-gripped rollout {}",
            oracle.x
        );
        // The chain STARTS engaged but the policy can lift the proximal joint, so the tip may rise
        // off the block mid-rollout — both FD gates stay machine-exact THROUGH that active-set
        // change (a stronger test of the gradient than a permanently-engaged grip).

        // DESIGN channel (μ_total, relative ε).
        let eps_mu = MU0 * 1e-4;
        let fd_mu = (build_2link(MU0 + eps_mu)
            .coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, n)
            .x
            - build_2link(MU0 - eps_mu)
                .coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, n)
                .x)
            / (2.0 * eps_mu);
        assert!(
            fd_mu.abs() > 1e-9,
            "2-link n={n}: ∂tip_x/∂μ_total nonzero (fd={fd_mu:e})"
        );
        let rel_mu = (mu_grad - fd_mu).abs() / fd_mu.abs().max(1e-9);
        assert!(
            rel_mu < 1e-5,
            "2-link n={n}: design (μ) gradient {mu_grad:e} vs FD {fd_mu:e}, rel {rel_mu:.3e}"
        );

        // POLICY channel.
        let eps = 1e-6;
        let mut max_rel = 0.0_f64;
        for j in 0..PARAMS.len() {
            let mut pp = PARAMS;
            let mut pm = PARAMS;
            pp[j] += eps;
            pm[j] -= eps;
            let fd = (build_2link(MU0)
                .coupled_trajectory_policy_gripped_x(&LinearFeedback, &pp, n)
                .x
                - build_2link(MU0)
                    .coupled_trajectory_policy_gripped_x(&LinearFeedback, &pm, n)
                    .x)
                / (2.0 * eps);
            assert!(
                fd.abs() > 1e-9,
                "2-link n={n}: ∂tip_x/∂θ_{j} nonzero (fd={fd:e})"
            );
            max_rel = max_rel.max((theta_grad[j] - fd).abs() / fd.abs().max(1e-9));
        }
        assert!(
            max_rel < 1e-5,
            "2-link n={n}: policy gradient vs FD, worst rel {max_rel:.3e}"
        );
    }
}
