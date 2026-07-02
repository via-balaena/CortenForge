//! Articulated FRICTION-COEFFICIENT (μ_c) gradient — the multi-horizon invariant the coupling
//! gradient harness can't express.
//!
//! The per-horizon FD cells of
//! `StaggeredCoupling::coupled_trajectory_tangential_friction_coeff_gradient_articulated` (the
//! tip-`x` slide vs the Coulomb coefficient `μ_c` on a hinge / 2-link grip) are folded into the
//! `hinge·friction-coeff[μ_c]` and `chain·friction-coeff[μ_c]` rows of
//! `tests/coupling_grad_harness.rs`, which assert the same FD match against the same
//! `coupled_trajectory_gripped_articulated(n).x` oracle at the same V-shape step `μ_c·1e-3`, plus
//! forward-consistency and a `Comp::Live` non-vacuity floor.
//!
//! What the single-horizon rows CAN'T fold, kept here: the **hinge multi-horizon FD-match** — the
//! μ_c gradient stays accurate (rel < 2e-5) as the rollout lengthens across the make, a liftoff,
//! and a re-touch (n = 5, 20, 40), a cross-horizon invariant a single-length matrix cell can't
//! hold. (The 2-link start-engagement guard the μ_c gate also carried is NOT re-kept: it runs the
//! byte-identical (μ = 3e3, μ_c = 2.5) baseline the articulated-material slim file's
//! `articulated_friction_material_chain_starts_engaged` already asserts.)
//!
//! The μ_c FD has a V-shaped minimum at `ε = μ_c·1e-3` (rel ~2e-6): smaller steps hit round-off
//! CANCELLATION (∂x/∂μ_c is a tiny change in a large tip coordinate), larger steps the O(ε²)
//! truncation of the near-linear friction response — so the step is `μ_c·1e-3`. See
//! `project-friction-leaf.md`.

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

/// HINGE multi-horizon FD-match (the cross-horizon invariant the single-length matrix row can't
/// express): the μ_c gradient stays accurate as the rollout lengthens across the make, a liftoff,
/// and a re-touch. The non-vacuity floor guards a silently de-coupled μ_c channel from passing on
/// `0 ≈ 0`.
#[test]
fn articulated_friction_coeff_gradient_matches_fd_at_all_lengths() {
    for n in [5usize, 20, 40] {
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
}
