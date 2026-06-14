//! Keystone multi-DOF rigid coupling, PR2 — the coupled ARTICULATED trajectory
//! gradient.
//!
//! `StaggeredCoupling::coupled_trajectory_material_gradient_articulated` is the
//! articulated successor to the free-body-platen `coupled_trajectory_material_gradient`:
//! the rigid body is a Y-hinge (a tilted arm with a point mass at the tip pressing
//! on the soft block), so a contact force at the tip maps to a generalized joint
//! acceleration coupled across the joint (the matrix `Δt·M⁻¹·Jᵀ` ≠ the scalar
//! `dt/m`) and the contact-plane pose tracks the moving tip (`∂(tip height)/∂q =
//! J_z`). One `tape.backward` gives `∂(tip height)_N/∂μ` across the whole rollout.
//!
//! Gate: the one-tape gradient matches a central FD of the FULL real coupled
//! re-rollout (a fresh coupling at μ±ε, the real articulated `step` loop —
//! `coupled_trajectory_articulated_z`), an INDEPENDENT oracle. The state carry is
//! the LOADED single-step Jacobian (finite-differenced; it includes the applied-
//! force geometric stiffness the unloaded `transition_derivatives` drops), so the
//! composed gradient is FD-accurate (~1e-5), adequate for co-design gradient
//! descent (a machine-exact analytic geometric-stiffness term is a follow-on).

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

// A Y-hinge: pivot above the soft block, a point mass on the link tip. Started
// tilted (qpos = 0.3) so the tip arcs (the moment arm makes dt/m wrong) and
// presses into the block top (z = 0.1); gravity swings it inward → sustained
// engagement. No joint damping/stiffness (the bare-M⁻¹ scope).
const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const MU0: f64 = 3.0e4;

fn build(mu: f64) -> StaggeredCoupling {
    let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3; // tilt off vertical
    data.forward(&model).expect("initial forward");
    // body=1, clearance=0.005, n_per_edge=4 (even), edge=0.1, mu, dt=1e-3,
    // kappa=3e4, d_hat=1e-2, rigid_damping=0 (articulated v1 scope).
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
}

#[test]
fn articulated_trajectory_engages_and_is_stable() {
    let mut c = build(MU0);
    let z0 = c.data().xipos[1].z;
    let zn = c.coupled_trajectory_articulated_z(10);
    assert!(zn.is_finite(), "tip height must stay finite");
    // The tilted arm presses into the block and stays engaged (the plane =
    // tip_z − 0.005 sits within d_hat of the block top z = 0.1).
    assert!(
        zn < 0.115 && zn > 0.10,
        "tip should remain engaged near the block top, got z0={z0} zN={zn}"
    );
}

/// The total `∂tip_z_N/∂μ` (along the coupling's λ = 4μ tie) from one tape, and a
/// central FD of the full real coupled re-rollout at μ±ε.
fn tape_and_fd(n: usize) -> (f64, f64, f64) {
    let (tip_z, g_mu) = build(MU0).coupled_trajectory_material_gradient_articulated(n, 0);
    let (_t2, g_la) = build(MU0).coupled_trajectory_material_gradient_articulated(n, 1);
    let total = g_mu + 4.0 * g_la;
    let eps = MU0 * 1e-4;
    let zp = build(MU0 + eps).coupled_trajectory_articulated_z(n);
    let zm = build(MU0 - eps).coupled_trajectory_articulated_z(n);
    let fd = (zp - zm) / (2.0 * eps);
    (tip_z, total, fd)
}

#[test]
fn articulated_gradient_matches_full_coupled_fd() {
    // Forward tape value reproduces the real rollout exactly.
    let n = 10;
    let (tip_z, total, fd) = tape_and_fd(n);
    let z_oracle = build(MU0).coupled_trajectory_articulated_z(n);
    assert!(
        (tip_z - z_oracle).abs() < 1e-12,
        "tape forward tip_z {tip_z} != real rollout {z_oracle}"
    );
    // The composed multi-DOF gradient matches the independent full-coupled FD
    // oracle. The state carry's velocity rows (the geometric-stiffness-laden loaded
    // Jacobian) are finite-differenced, so the gradient is FD-accurate (~6e-6 at
    // n = 10, vs the free-body platen's machine-exact — the articulated body has a
    // real geometric stiffness the analytic factor omits); a machine-exact analytic
    // term is a follow-on. See docs/keystone/multidof_rigid_recon.md §8c.
    let rel = (total - fd).abs() / fd.abs().max(1e-30);
    println!("n={n}: tape total={total:.8e} FD={fd:.8e} rel={rel:.3e}");
    assert!(
        rel < 1e-4,
        "articulated dμ gradient must match full-coupled FD (FD-accurate carry), got rel {rel:.3e}"
    );
    assert!(total.abs() > 1e-9, "expected a nonzero ∂tip_z/∂μ");
}

/// The §8a structure is locked: the pre-update force carry (∂qpos'/∂fz = 0) makes
/// the first step (soft solve from rest, dfz/dμ ≈ 0) carry ZERO gradient, and the
/// short-rollout gradient is machine-exact (the FD-accuracy of the geometric-
/// stiffness carry only accrues over more steps). A regression that re-wires the
/// force into the position carry (the spurious first-step gradient) trips n = 2.
#[test]
fn articulated_gradient_short_rollout_is_machine_exact() {
    let (_t1, total1, fd1) = tape_and_fd(1);
    assert!(
        total1.abs() < 1e-20 && fd1.abs() < 1e-20,
        "first step from rest must carry zero μ-gradient (tape {total1:.3e}, FD {fd1:.3e})"
    );
    let (_t2, total2, fd2) = tape_and_fd(2);
    let rel2 = (total2 - fd2).abs() / fd2.abs().max(1e-30);
    assert!(
        rel2 < 1e-6,
        "two-step articulated gradient must be machine-exact, got rel {rel2:.3e}"
    );
}
