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
//! moment vs a pure force at the COM shifts the rollout materially (~4.5% in tip_z_N
//! by n = 6, growing to ~7% by n = 10 — the forward model is first-order wrong
//! without it). One `tape.backward` gives `∂(tip height)_N/∂μ` across the whole
//! rollout.
//!
//! Gates: the one-tape gradient matches a central FD of the FULL real coupled
//! re-rollout (a fresh coupling at μ±ε, the real articulated `step` loop —
//! `coupled_trajectory_articulated_z`, which routes the same moment), an INDEPENDENT
//! oracle. The wrench node (`∂w/∂x*`, `∂w/∂h`, `∂w/∂s`) is separately FD-validated
//! machine-exact against the real contact readout in the
//! `contact_wrench_node_matches_readout_fd` lib unit test; the rigid state carry
//! `J_state` is the analytic single-hinge geometric stiffness, FD-validated in
//! `analytic_state_jacobian_matches_fd_loaded`.
//!
//! **Accuracy + the long-rollout residual (re-diagnosed 2026-06-15).** Under PENALTY
//! contact the composed gradient is machine-exact through contact make/break (≤~1e-6
//! to n≈6), but over longer rollouts with sustained re-engagement the residual GROWS
//! with n (~1e-3 at n=10, ~5e-3 at n=12). This residual is the OFF-COM MOMENT's
//! gradient over long rollouts — and is NOT what two earlier drafts claimed. It is:
//!   * MOMENT-specific: the SAME articulated path on a free-joint platen (no moment,
//!     J=I) is machine-exact at every n (`articulated_free_platen_exact_all_n`), so the
//!     carry, the soft adjoint, and the §8a structure are all sound;
//!   * NOT the geometric stiffness: making `J_state` analytic (machine-exact vs the FD
//!     loaded Jacobian) leaves n≥10 unchanged — it only tightened n≈6;
//!   * NOT a penalty active-set kink: IPC reaches the same ~1e-3 order;
//!   * resistant to fresh-FK re-forward, lag-attribution, and the true position-row
//!     wrench term (all measured WORSE) — the stale-to-`s_k` calibration is optimal.
//!
//! The precise source is a documented open problem (the off-COM moment's long-rollout
//! gradient); see docs/keystone/geometric_stiffness_recon.md. Still well within
//! co-design tolerance (DIRECTION + ~99.9% magnitude). See also
//! docs/keystone/contact_moment_recon.md §5a/§6.

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
    // n = 6 spans the initial violent make (25 active, f_z ~ 800 N), a genuine
    // LIFTOFF (active set 25 → 0 → 0 → 0 for steps 1–3), and a re-touch (1 active at
    // step 4) — measured active-set sequence 25,0,0,0,1,0. The moment-routing tape
    // gradient is MACHINE-EXACT against the independent full-coupled FD oracle here
    // (the FD oracle is converged to <1e-9 over 4 decades of ε; the wrench node is
    // FD-exact vs the real readout in the `contact_wrench_node_matches_readout_fd`
    // lib unit test). The analytic geometric-stiffness `J_state` (replacing the FD
    // loaded Jacobian) tightens this from ~1.5e-6 to ~7e-7 — the gate is now 3e-6 (a
    // float-noise margin over the measured value, the tightening the analytic enables).
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
        rel < 3e-6,
        "articulated dμ gradient (with moment) must match full-coupled FD through \
         make/break, got rel {rel:.3e}"
    );
    assert!(total.abs() > 1e-9, "expected a nonzero ∂tip_z/∂μ");
    // The moment is load-bearing: a pure-force-at-COM forward model would shift the
    // rollout materially (routing vs dropping the moment differs ~4.5% in tip_z_N at
    // this n = 6 horizon, growing to ~7% by n = 10; see the recon). The
    // dt/m-vs-multi-DOF distinction — J_z ≈ 0.028 ≠ 1 on the tilted contact axis — is
    // asserted at the primitive level in tests/rigid_multidof_response.rs.
}

/// Over longer rollouts with sustained re-engagement the gradient residual GROWS
/// with n (penalty: ~1e-3 at n = 10, ~5e-3 at n = 12). RE-DIAGNOSED 2026-06-15: this
/// is the OFF-COM MOMENT's gradient over long rollouts — NOT the geometric stiffness
/// (the analytic `J_state` here matches the FD loaded Jacobian to machine precision
/// yet leaves n=10 unchanged) and NOT a penalty active-set kink (IPC reaches the same
/// order). It is moment-specific: the free-platen articulated path (no moment) is
/// machine-exact at every n — see `articulated_free_platen_exact_all_n`. The precise
/// source is a documented open problem (docs/keystone/geometric_stiffness_recon.md);
/// fresh-FK, lag-attribution, and the true position-row term were all measured WORSE.
/// Still well within co-design tolerance (DIRECTION + ~99.9% of magnitude). The 3e-3
/// bound is the n ≤ 10 value (exceeded for larger n); gated + documented, not hidden.
#[test]
fn articulated_gradient_long_rollout_grows_with_n() {
    let n = 10;
    let (_tip_z, total, fd) = tape_and_fd(n);
    let rel = (total - fd).abs() / fd.abs().max(1e-30);
    println!("n={n}: tape total={total:.8e} FD={fd:.8e} rel={rel:.3e}");
    assert!(
        rel < 3e-3,
        "n=10 articulated dμ gradient must stay within the documented n≤10 \
         moment-residual bound, got rel {rel:.3e}"
    );
}

/// DISCRIMINATOR (the re-diagnosis evidence): the SAME articulated path on a
/// free-joint platen — no off-COM moment (symmetric contact through the COM), J = I so
/// no geometric stiffness, and the FD loaded Jacobian — is MACHINE-EXACT at every n,
/// including n = 12 where the hinge-with-moment residual is ~5e-3. This isolates the
/// long-rollout residual to the off-COM MOMENT's gradient: the shared multi-DOF carry,
/// the soft adjoint, and the §8a structure are all sound (it is not a carry, soft, or
/// geometric-stiffness defect). See docs/keystone/geometric_stiffness_recon.md.
#[test]
fn articulated_free_platen_exact_all_n() {
    const FREE_PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.108">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
    let build_free = |mu: f64| -> StaggeredCoupling {
        let model = load_model(FREE_PLATEN_MJCF).expect("platen MJCF loads");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        StaggeredCoupling::new(
            model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
        )
    };
    for n in [6usize, 10, 12] {
        let (_t, g_mu) = build_free(MU0).coupled_trajectory_material_gradient_articulated(n, 0);
        let (_t2, g_la) = build_free(MU0).coupled_trajectory_material_gradient_articulated(n, 1);
        let total = g_mu + 4.0 * g_la;
        let eps = MU0 * 1e-4;
        let zp = build_free(MU0 + eps).coupled_trajectory_articulated_z(n);
        let zm = build_free(MU0 - eps).coupled_trajectory_articulated_z(n);
        let fd = (zp - zm) / (2.0 * eps);
        let rel = (total - fd).abs() / fd.abs().max(1e-30);
        println!("free-platen articulated n={n}: tape={total:.6e} FD={fd:.6e} rel={rel:.3e}");
        assert!(
            rel < 1e-6,
            "free-platen articulated gradient (no moment) must be machine-exact at \
             n={n}, got rel {rel:.3e}"
        );
    }
}

/// The §8a structure is locked: the pre-update force carry (∂qpos'/∂w = 0) makes
/// the first step (soft solve from rest, dw/dμ ≈ 0) carry ZERO gradient, and the
/// two-step gradient is machine-exact (the long-rollout moment residual only accrues
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
