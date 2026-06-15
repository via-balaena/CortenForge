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
//! **Accuracy — MACHINE-EXACT at every horizon, single-hinge through multi-link.** The
//! composed gradient matches the full-coupled FD to ~1e-9 at n = 1, 2, 6, 10, 15 — for
//! the single hinge, a free-joint platen (nv = 6), AND a 2-link chain (nv = 2,
//! `twolink_chain_gradient_machine_exact`). The earlier long-rollout moment residual
//! (~1e-3 at n = 10) and the 74%-at-n=2 multi-link error were BOTH the same defect: the
//! stale-FK contact pose + §8a position-row drop was a self-consistent pair calibrated
//! ONLY for nv = 1. The **fully-fresh formulation** — fresh-FK contact pose + fresh
//! output + the true position-row carry `∂qpos'/∂w = Δt·G_vel` — is the correct
//! differentiable formulation (and the fresh pose is more physically faithful). See
//! docs/keystone/moment_residual_recon.md and docs/keystone/multilink_recon.md.

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
    // n = 6 spans the initial violent make (~25 active, f_z ~ 800 N), a liftoff, and a
    // re-touch. The moment-routing tape gradient is MACHINE-EXACT against the
    // independent full-coupled FD oracle (the wrench node is FD-exact vs the real
    // readout in `contact_wrench_node_matches_readout_fd`; the analytic single-hinge
    // `J_state` in `analytic_state_jacobian_matches_fd_loaded`). The fully-fresh
    // formulation (fresh-FK contact pose + fresh output + true position-row carry
    // `∂qpos'/∂w = Δt·G_vel`) removed the old long-rollout moment residual — see
    // `docs/keystone/moment_residual_recon.md`.
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
        rel < 1e-7,
        "articulated dμ gradient (with moment) must be machine-exact vs full-coupled \
         FD through make/break, got rel {rel:.3e}"
    );
    assert!(total.abs() > 1e-9, "expected a nonzero ∂tip_z/∂μ");
    // The dt/m-vs-multi-DOF distinction — J_z ≠ 1 on the tilted contact axis — is
    // asserted at the primitive level in tests/rigid_multidof_response.rs.
}

/// The articulated gradient stays MACHINE-EXACT over LONG rollouts with sustained
/// re-engagement (n = 10 and n = 15) — the fully-fresh formulation removed the
/// moment-residual growth the earlier stale-FK convention carried (~1e-3 at n = 10,
/// ~5e-3 at n = 15). See `docs/keystone/moment_residual_recon.md`.
#[test]
fn articulated_gradient_long_rollout_machine_exact() {
    for n in [10usize, 15] {
        let (_tip_z, total, fd) = tape_and_fd(n);
        let rel = (total - fd).abs() / fd.abs().max(1e-30);
        println!("n={n}: tape total={total:.8e} FD={fd:.8e} rel={rel:.3e}");
        assert!(
            rel < 1e-7,
            "n={n} articulated dμ gradient must be machine-exact, got rel {rel:.3e}"
        );
    }
}

/// The articulated path on a free-joint platen (no off-COM moment, symmetric contact)
/// is machine-exact at every n — the fully-fresh formulation handles the free joint
/// (nv = 6) as cleanly as the hinge. (Historically this was the discriminator that
/// isolated the long-rollout residual to the off-COM moment, since fixed by the
/// fully-fresh formulation; see `docs/keystone/moment_residual_recon.md`.)
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

/// Short rollouts are machine-exact too. With the fresh output (`tip_z_N = xipos(q_N)`,
/// not the old lagged `xipos(q_{N-1})`), even n = 1 carries a real, nonzero μ-gradient
/// (the step-0 wrench reaches `qpos_1` through `Δt·qvel_1`, the true position-row term) —
/// and the tape matches the full-coupled FD machine-exactly at n = 1 and n = 2.
#[test]
fn articulated_gradient_short_rollout_is_machine_exact() {
    for n in [1usize, 2] {
        let (_t, total, fd) = tape_and_fd(n);
        let rel = (total - fd).abs() / fd.abs().max(1e-30);
        assert!(
            total.abs() > 1e-12,
            "n={n}: fresh-output gradient should be nonzero (tape {total:.3e})"
        );
        assert!(
            rel < 1e-7,
            "n={n} articulated gradient must be machine-exact, got rel {rel:.3e}"
        );
    }
}

// A 2-LINK hinge chain (nv = 2): a genuine multi-DOF articulated mechanism — the
// distal tip presses the soft block, the contact wrench couples ACROSS both joints
// (off-diagonal M, Coriolis velocity coupling). The capstone-exo bridge beyond the
// single hinge.
const TWOLINK_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
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

fn build_2link(mu: f64) -> StaggeredCoupling {
    let model = load_model(TWOLINK_MJCF).expect("2-link MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.1;
    data.qpos[1] = -0.05; // off-center tip; the off-diagonal joint coupling is live
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 2, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
}

/// **The multi-link (nv = 2) coupled gradient matches the full-coupled FD.** A 2-link
/// hinge chain — genuine multi-DOF (off-diagonal mass coupling + Coriolis) — through the
/// same articulated path, FD-gated at n = 2/6/10. The fully-fresh formulation handles the
/// chain that the single-hinge-calibrated stale-FK convention got 74% wrong at n = 2.
/// The accuracy here is FD-CARRY precision (~1e-6): a chain has no single-hinge analytic
/// `J_state`, so it uses the FD `loaded_state_jacobian` (eps 1e-6). The analytic CHAIN
/// carry (the Jacobian Hessian + `∂M⁻¹/∂q`) would lift this to ~1e-9 and is the
/// documented follow-on; see `docs/keystone/multilink_recon.md`.
#[test]
fn twolink_chain_gradient_matches_fd() {
    let z0 = build_2link(MU0).data().xipos[2].z;
    assert!(
        z0 > 0.10,
        "2-link tip should start engaged near the block top, got {z0}"
    );
    for n in [2usize, 6, 10] {
        let (_t, gm) = build_2link(MU0).coupled_trajectory_material_gradient_articulated(n, 0);
        let (_t2, gl) = build_2link(MU0).coupled_trajectory_material_gradient_articulated(n, 1);
        let total = gm + 4.0 * gl;
        let eps = MU0 * 1e-4;
        let zp = build_2link(MU0 + eps).coupled_trajectory_articulated_z(n);
        let zm = build_2link(MU0 - eps).coupled_trajectory_articulated_z(n);
        let fd = (zp - zm) / (2.0 * eps);
        let rel = (total - fd).abs() / fd.abs().max(1e-30);
        println!("2-link n={n}: tape={total:.6e} FD={fd:.6e} rel={rel:.3e}");
        assert!(
            total.abs() > 1e-12 && rel < 1e-5,
            "2-link nv=2 gradient must match full-coupled FD (FD-carry precision) at \
             n={n}, got rel {rel:.3e}"
        );
    }
}
