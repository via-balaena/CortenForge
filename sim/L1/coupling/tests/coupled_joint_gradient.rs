//! Keystone JOINT design+policy gradient — the fusion-soundness invariant.
//!
//! `StaggeredCoupling::coupled_trajectory_joint_gradient` rolls the coupled
//! soft↔rigid system forward under a closed-loop feedback policy `u_k = π_θ(state_k)`
//! with BOTH the soft material design variable `μ` (stiffness scale, λ = 4μ) AND
//! the policy parameters `θ` live on ONE chassis tape, then a single
//! `tape.backward(z_N)` reads `(∂z_N/∂μ_total, ∂z_N/∂θ)` at once — the literal
//! realization of `MISSION.md`'s "one outer loop differentiating w.r.t. both design
//! AND policy parameters".
//!
//! The FD-exactness of both gradient blocks (material along the λ = 4μ line, policy
//! per θ component) is covered by the `joint(μ+θ)` row of `coupling_grad_harness.rs`,
//! the single channel-agnostic FD matrix. What lives HERE is the one invariant that
//! row does NOT subsume: a tape-vs-tape **fusion-soundness** check — the joint
//! method's policy block must equal the standalone `coupled_trajectory_policy_gradient`
//! to near machine zero (rel < 1e-9), far tighter than any FD tolerance, proving
//! that adding the live material design leaf to the tape does not perturb the policy
//! backprop. (There is no passive single-axis equivalent for the material block to
//! compare against — the joint rollout is policy-driven, a *different* trajectory
//! than the control-free `coupled_trajectory_material_gradient` — so the material
//! block is validated by the harness FD oracle.)

#![allow(clippy::expect_used)]

use sim_coupling::{LinearFeedback, StaggeredCoupling};
use sim_mjcf::load_model;

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
const THETA: [f64; 3] = [-20.0, -5.0, 2.0];
const N: usize = 12;

/// A coupling at the baseline soft stiffness (λ = 4μ via the constructor).
fn build() -> StaggeredCoupling {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, MU0, 1.0e-3, 3.0e4, 1.0e-2, DAMPING,
    )
}

/// The joint POLICY block equals the policy-only method
/// `coupled_trajectory_policy_gradient` to machine precision (the same
/// policy-driven rollout): adding the live material design leaf to the tape does
/// not perturb the policy gradient — the fusion is sound. The FD-exactness of both
/// joint blocks is the `joint(μ+θ)` row of `coupling_grad_harness.rs`.
#[test]
fn joint_policy_block_matches_policy_only() {
    let (_z, _dz_dmu, dz_dtheta) =
        build().coupled_trajectory_joint_gradient(&LinearFeedback, &THETA, N);
    let (_zp, g_theta) = build().coupled_trajectory_policy_gradient(&LinearFeedback, &THETA, N);
    for (i, (&gj, &gp)) in dz_dtheta.iter().zip(&g_theta).enumerate() {
        let rel = (gj - gp).abs() / gp.abs().max(1e-30);
        eprintln!("θ[{i}]: joint={gj:.6e} policy-only={gp:.6e} rel={rel:.3e}");
        assert!(
            rel < 1e-9 || (gj - gp).abs() < 1e-14,
            "joint θ[{i}] {gj} != policy-only {gp} (rel {rel:e})"
        );
    }
}
