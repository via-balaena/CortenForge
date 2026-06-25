//! The articulated FRICTION-COEFFICIENT (μ_c) gradient — completes the friction co-design set on
//! the articulated matrix carry (material μ/λ shipped in #402; μ_c is the surface-grip sibling).
//!
//! `StaggeredCoupling::coupled_trajectory_tangential_friction_coeff_gradient_articulated` is the
//! μ_c sibling of `..._tangential_material_gradient_articulated` and the articulated successor to
//! the free-platen `coupled_trajectory_tangential_friction_coeff_gradient`. μ_c enters through two
//! channels — the soft `x*` (tiny in deep slip) and the DIRECT friction force `∂∇D_v/∂μ_c =
//! ∇D_v/μ_c` (dominant; threaded through force AND off-COM moment by `FrictionWrenchTrajVjp`'s
//! μ_c parent). One `tape.backward` gives `∂(tip x)_N/∂μ_c` — the tangential drag vs the contact's
//! grip-surface friction coefficient.
//!
//! Gate: the one-tape gradient matches the central FD of the full real grip re-rollout
//! (`coupled_trajectory_gripped_articulated` at μ_c±ε), to `rel < 2e-5`. The μ_c FD here has a
//! V-shaped minimum at `ε = μ_c·1e-3` (rel ~2e-6): smaller steps hit round-off CANCELLATION
//! (∂x/∂μ_c is a tiny change in a large tip coordinate), larger steps the O(ε²) truncation of the
//! near-linear friction response — so the step is the well-conditioned `μ_c·1e-3`. A naive
//! `μ_c·1e-5` step lands in the cancellation branch and reads ~2e-4 (the gradient is exact; the FD
//! is the noisy party). The material gate is differently conditioned — it perturbs the soft `μ`,
//! whose larger lever stays machine-exact at its own step — so its tighter tolerance doesn't carry.
//! Validated at single-hinge (n = 5/20/40) AND a 2-link chain (the off-COM moment cross-term on a
//! genuine `nv > 1` topology). A COMPLIANT block (`μ = 3e3`) keeps the lever well-conditioned.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const SOFT_MU: f64 = 3.0e3; // compliant block
const FRIC_MU: f64 = 2.5; // the Coulomb coefficient we differentiate around (creep grip)
const EPS_V: f64 = 0.1;

fn build(fric_mu: f64) -> StaggeredCoupling {
    let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, SOFT_MU, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_friction(fric_mu, EPS_V)
}

/// `∂(tip x)_N/∂μ_c` from one tape, and a central FD of the full grip re-rollout at μ_c±ε.
fn tape_and_fd(n: usize) -> (f64, f64, f64) {
    let (tip_x, grad) =
        build(FRIC_MU).coupled_trajectory_tangential_friction_coeff_gradient_articulated(n);
    let eps = FRIC_MU * 1e-3; // the FD V-shape minimum (see module note)
    let xp = build(FRIC_MU + eps)
        .coupled_trajectory_gripped_articulated(n)
        .x;
    let xm = build(FRIC_MU - eps)
        .coupled_trajectory_gripped_articulated(n)
        .x;
    let fd = (xp - xm) / (2.0 * eps);
    (tip_x, grad, fd)
}

fn assert_matches(n: usize) {
    let (tip_x, grad, fd) = tape_and_fd(n);
    assert!(tip_x.is_finite() && grad.is_finite() && fd.is_finite());
    assert!(
        grad.abs() > 1e-9 && fd.abs() > 1e-9,
        "n={n}: ∂x/∂μ_c implausibly ~0 (grad={grad:e}, fd={fd:e}) — μ_c channel inert?"
    );
    let rel = (grad - fd).abs() / fd.abs().max(1e-9);
    assert!(
        rel < 2e-5,
        "n={n}: tape ∂x/∂μ_c {grad:e} vs full-grip FD {fd:e} (rel {rel:e}); tip_x={tip_x}"
    );
}

#[test]
fn articulated_friction_coeff_gradient_matches_fd_n5() {
    assert_matches(5);
}

#[test]
fn articulated_friction_coeff_gradient_matches_fd_n20() {
    assert_matches(20);
}

#[test]
fn articulated_friction_coeff_gradient_matches_fd_n40() {
    assert_matches(40);
}

// A 2-link hinge CHAIN (nv = 2): the μ_c moment cross-term + J_lin feedback on a genuine multi-DOF
// topology (the nv = 1 case collapses the arms to one lever — the moment-residual blind spot).
const TWOLINK_MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="upper" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.025" size="0.004" mass="0.3"/>
      <body name="lower" pos="0 0 -0.05">
        <joint type="hinge" axis="0 1 0"/>
        <geom type="sphere" pos="0 0 -0.04" size="0.004" mass="0.4"/>
      </body>
    </body>
  </worldbody>
</mujoco>"#;

fn build_2link(fric_mu: f64) -> StaggeredCoupling {
    let model = load_model(TWOLINK_MJCF).expect("2-link MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.1;
    data.qpos[1] = -0.05;
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 2, 0.005, 4, 0.1, SOFT_MU, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_friction(fric_mu, EPS_V)
}

#[test]
fn articulated_friction_coeff_gradient_2link_matches_fd() {
    let z0 = build_2link(FRIC_MU).data().xipos[2].z;
    assert!(
        z0 > 0.10,
        "2-link tip must start engaged near the block top, got {z0}"
    );
    for n in [3usize, 12] {
        let (_t, grad) = build_2link(FRIC_MU)
            .coupled_trajectory_tangential_friction_coeff_gradient_articulated(n);
        let eps = FRIC_MU * 1e-3; // the FD V-shape minimum (see module note)
        let xp = build_2link(FRIC_MU + eps)
            .coupled_trajectory_gripped_articulated(n)
            .x;
        let xm = build_2link(FRIC_MU - eps)
            .coupled_trajectory_gripped_articulated(n)
            .x;
        let fd = (xp - xm) / (2.0 * eps);
        assert!(
            grad.abs() > 1e-9 && fd.abs() > 1e-9,
            "2-link n={n}: ∂x/∂μ_c implausibly ~0 (grad={grad:e}, fd={fd:e})"
        );
        let rel = (grad - fd).abs() / fd.abs().max(1e-9);
        assert!(
            rel < 2e-5,
            "2-link n={n}: tape ∂x/∂μ_c {grad:e} vs full-grip FD {fd:e} (rel {rel:e})"
        );
    }
}
