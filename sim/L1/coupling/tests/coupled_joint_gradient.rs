//! Keystone JOINT design+policy gradient — the mission's "both in one outer loop".
//!
//! `StaggeredCoupling::coupled_trajectory_joint_gradient` rolls the coupled
//! soft↔rigid system forward under a closed-loop feedback policy `u_k = π_θ(state_k)`
//! with BOTH the soft material design variable `μ` (stiffness scale, λ = 4μ) AND
//! the policy parameters `θ` live on ONE chassis tape, then a single
//! `tape.backward(z_N)` reads `(∂z_N/∂μ_total, ∂z_N/∂θ)` at once — the literal
//! realization of `MISSION.md`'s "one outer loop differentiating w.r.t. both design
//! AND policy parameters".
//!
//! Gate: both gradient blocks match a central FD of the FULL real closed-loop
//! coupled re-rollout — the material block FD rebuilds the soft block along the
//! λ = 4μ line (`μ ± ε`), the policy block FD perturbs `θ ± ε`; both re-run the
//! real Newton solves + sim-core steps + the policy recurrence (independent
//! oracles, not affine identities). And the joint POLICY block agrees exactly with
//! the policy-only method `coupled_trajectory_policy_gradient` on the same
//! policy-driven rollout — confirming the material design leaf's presence does not
//! perturb the policy gradient (the fusion is sound). (There is no passive
//! single-axis equivalent for the material block to compare against — the joint
//! rollout is policy-driven, a *different* trajectory than the control-free
//! `coupled_trajectory_material_gradient` — so the material block is validated by
//! the FD oracle above.)

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

/// A coupling at a given soft stiffness `mu` (λ = 4μ via the constructor).
fn build_at(mu: f64) -> StaggeredCoupling {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, DAMPING,
    )
}

fn build() -> StaggeredCoupling {
    build_at(MU0)
}

/// Final platen height after the real closed-loop rollout at stiffness `mu`.
fn final_z_at(mu: f64, theta: &[f64], n: usize) -> f64 {
    build_at(mu).coupled_trajectory_policy_z(&LinearFeedback, theta, n)
}

const THETA: [f64; 3] = [-20.0, -5.0, 2.0];
const N: usize = 12;

/// One `tape.backward` gives BOTH `∂z_N/∂μ_total` and `∂z_N/∂θ`, each matching its
/// independent full-coupled closed-loop FD oracle.
#[test]
fn joint_gradient_matches_full_fd() {
    let (z_n, dz_dmu, dz_dtheta) =
        build().coupled_trajectory_joint_gradient(&LinearFeedback, &THETA, N);
    assert_eq!(dz_dtheta.len(), 3);

    // Forward consistency: the joint tape reproduces the real rollout.
    let z_ref = final_z_at(MU0, &THETA, N);
    assert!(
        (z_n - z_ref).abs() < 1e-12,
        "tape z_N {z_n} != real {z_ref}"
    );

    // Material block: FD along the λ = 4μ line (build rebuilds with λ = 4μ).
    let eps_mu = MU0 * 1e-6;
    let fd_mu = (final_z_at(MU0 + eps_mu, &THETA, N) - final_z_at(MU0 - eps_mu, &THETA, N))
        / (2.0 * eps_mu);
    let rel_mu = (dz_dmu - fd_mu).abs() / fd_mu.abs().max(1e-30);
    eprintln!("∂z/∂μ_total: tape={dz_dmu:.6e} fd(line)={fd_mu:.6e} rel={rel_mu:.3e}");
    assert!(
        rel_mu < 1e-5,
        "joint ∂z_N/∂μ_total {dz_dmu} vs rebuild-line FD {fd_mu} (rel {rel_mu:e})"
    );

    // Policy block: FD per θ component (same coupling, perturb θ).
    let names = ["w_z", "w_vz", "b"];
    for (i, &g) in dz_dtheta.iter().enumerate() {
        let eps = if i == 2 { 1e-3 } else { 1e-2 };
        let mut up = THETA;
        let mut dn = THETA;
        up[i] += eps;
        dn[i] -= eps;
        let fd = (final_z_at(MU0, &up, N) - final_z_at(MU0, &dn, N)) / (2.0 * eps);
        let rel = (g - fd).abs() / fd.abs().max(1e-30);
        let abs = (g - fd).abs();
        eprintln!(
            "θ[{}]={}: tape={g:.6e} fd={fd:.6e} rel={rel:.3e}",
            i, names[i]
        );
        assert!(
            rel < 1e-5 || abs < 1e-12,
            "joint ∂z_N/∂{} tape {g} vs FD {fd} (rel {rel:e} abs {abs:e})",
            names[i]
        );
    }
}

/// The joint POLICY block equals the policy-only method
/// `coupled_trajectory_policy_gradient` to machine precision (the same
/// policy-driven rollout): adding the live material design leaf to the tape does
/// not perturb the policy gradient — the fusion is sound. (The material block has
/// no passive single-axis equivalent — the joint rollout is policy-driven, unlike
/// the control-free `coupled_trajectory_material_gradient` — so it is validated by
/// the FD oracle in `joint_gradient_matches_full_fd`.)
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
