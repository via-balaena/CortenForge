//! Keystone multi-DOF rigid coupling — the coupled ARTICULATED trajectory
//! gradient, WITH the off-COM contact moment.
//!
//! `StaggeredCoupling::coupled_trajectory_material_gradient_articulated` is the
//! articulated successor to the free-body-platen `coupled_trajectory_material_gradient`:
//! the rigid body is a Y-hinge (a tilted arm with a point mass at the tip pressing
//! on the soft block), so a contact force at the tip maps to a generalized joint
//! acceleration coupled across the joint (the matrix `Δt·M⁻¹·Jᵀ` ≠ the scalar
//! `dt/m`), the contact-plane pose tracks the moving tip (`∂(tip height)/∂q = J_z`),
//! and the reaction is routed as the full spatial **wrench** `[τ; f]` about the body
//! COM — including the off-COM contact **moment** `τ = −Σ(rᵢ−c)×gᵢ`. The tip COM
//! (`xipos`, x ≈ −0.028 when tilted) is offset ~0.08 m from the block-top contact
//! centroid (x ≈ 0.05), so the resultant genuinely misses the COM: routing the
//! moment vs a pure force at the COM shifts the rollout ~7% (the forward model is
//! first-order wrong without it). One `tape.backward` gives `∂(tip height)_N/∂μ`
//! across the whole rollout.
//!
//! Gates: the one-tape gradient matches a central FD of the FULL real coupled
//! re-rollout (a fresh coupling at μ±ε, the real articulated `step` loop —
//! `coupled_trajectory_articulated_z`, which routes the same moment), an INDEPENDENT
//! oracle. The wrench node (`∂w/∂x*`, `∂w/∂h`, `∂w/∂s`) is separately FD-validated
//! machine-exact against the real contact readout in `the `contact_wrench_node_matches_readout_fd` lib unit test`. Through
//! contact make/break the composed gradient is machine-exact; over longer rollouts
//! with sustained re-engagement it degrades to ~1e-3 — the moment's config-sensitive
//! rotational Jacobian amplifying the FD'd geometric-stiffness carry residual (the
//! analytic geometric-stiffness term is a follow-on; see
//! docs/keystone/contact_moment_recon.md §6).

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
fn articulated_gradient_matches_full_coupled_fd_through_make_break() {
    // n = 6 spans the initial violent make (25 active, f_z ~ 800 N), the rebound
    // (the active set flips to 0), and re-touch — a genuine make/break. The
    // moment-routing tape gradient is MACHINE-EXACT against the independent
    // full-coupled FD oracle here (the FD oracle is converged to <1e-9 over 4
    // decades of ε; the wrench node is FD-exact vs the real readout in
    // the `contact_wrench_node_matches_readout_fd` lib unit test).
    let n = 6;
    let (tip_z, total, fd) = tape_and_fd(n);
    let z_oracle = build(MU0).coupled_trajectory_articulated_z(n);
    assert!(
        (tip_z - z_oracle).abs() < 1e-12,
        "tape forward tip_z {tip_z} != real rollout {z_oracle}"
    );
    let rel = (total - fd).abs() / fd.abs().max(1e-30);
    println!("n={n}: tape total={total:.8e} FD={fd:.8e} rel={rel:.3e}");
    assert!(
        rel < 1e-5,
        "articulated dμ gradient (with moment) must match full-coupled FD through \
         make/break, got rel {rel:.3e}"
    );
    assert!(total.abs() > 1e-9, "expected a nonzero ∂tip_z/∂μ");
    // The moment is load-bearing: a pure-force-at-COM forward model would shift the
    // rollout materially (routing vs dropping the moment differs ~7% in tip_z_N; see
    // the recon). The dt/m-vs-multi-DOF distinction — J_z ≈ 0.028 ≠ 1 on the tilted
    // contact axis — is asserted at the primitive level in
    // tests/rigid_multidof_response.rs.
}

/// Over longer rollouts with sustained re-engagement the gradient degrades to
/// ~1e-3: the moment's config-sensitive rotational Jacobian amplifies the FD'd
/// geometric-stiffness carry residual the merged force path carries at ~6e-6 (this
/// is NOT a contact-smoothness cap — penalty and IPC degrade identically; the
/// analytic geometric-stiffness term is the follow-on). Still well within
/// co-design-gradient-descent tolerance, and the DIRECTION/magnitude are ~99.9%
/// correct. Gated loosely + documented rather than hidden.
#[test]
fn articulated_gradient_long_rollout_within_geometric_stiffness_cap() {
    let n = 10;
    let (_tip_z, total, fd) = tape_and_fd(n);
    let rel = (total - fd).abs() / fd.abs().max(1e-30);
    println!("n={n}: tape total={total:.8e} FD={fd:.8e} rel={rel:.3e}");
    assert!(
        rel < 3e-3,
        "long-rollout articulated dμ gradient must stay within the documented \
         geometric-stiffness cap, got rel {rel:.3e}"
    );
}

/// The §8a structure is locked: the pre-update force carry (∂qpos'/∂w = 0) makes
/// the first step (soft solve from rest, dw/dμ ≈ 0) carry ZERO gradient, and the
/// two-step gradient is machine-exact (the geometric-stiffness residual only accrues
/// over more steps). A regression that re-wires the wrench into the position carry
/// (the spurious first-step gradient) trips n = 2.
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
