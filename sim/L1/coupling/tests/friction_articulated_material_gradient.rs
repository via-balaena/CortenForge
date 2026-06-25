//! Friction → matrix-carry lift, sub-leaf 3 — the articulated FRICTION material gradient
//! (full grip wrench: force + off-COM moment).
//!
//! `StaggeredCoupling::coupled_trajectory_tangential_material_gradient_articulated` is the
//! articulated successor to the free-platen `coupled_trajectory_tangential_material_gradient`:
//! a hinge arm grips a soft block under a sideways push, and the tangential friction grip
//! maps to a generalized joint acceleration through the FULL matrix carry `Δt·M_impl⁻¹·Jᵀ`
//! (`RigidStateCarryVjp`) instead of the free-platen scalar `dt/m` lanes, with the collider
//! drift read from the articulated state (`Δ_surf,x = (J_lin·qvel)_x·dt`, `DriftFromStateVjp`)
//! and the friction grip routed as the full wrench `[τ_fric; f_fric]` — the per-vertex force
//! `Σ∇D_v` AND its off-COM moment `Σ(r_v−c)×∇D_v` (`FrictionWrenchTrajVjp`, fed by sim-soft's
//! `friction_force_jacobians`). One `tape.backward` gives `∂(tip x)_N/∂p` — the tangential drag
//! vs the soft block's Neo-Hookean material (the constructor ties `λ = 4μ`).
//!
//! Gate: the one-tape gradient matches the central FD of the full real grip re-rollout
//! (`coupled_trajectory_gripped_articulated`, an INDEPENDENT black-box re-rollout at μ±ε) —
//! MACHINE-EXACT (rel ~8e-9, FD-limited) at n = 5, 20, 40. A COMPLIANT block (`μ = 3e3`) keeps
//! `μ` a well-conditioned lever on the tangential drag (a stiff block floors the FD-vs-analytic
//! relative agreement via the friction term's `∇²D ~ 1e4` cancellation, as in the free-platen
//! gate). `n = 1` is exactly 0 (position integrates the pre-step velocity → the drift feedback
//! is load-bearing only at `n ≥ 2`).

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

// A Y-hinge arm pushed sideways by tilted gravity, started tilted so the tip arcs and presses
// on the block top (z = 0.1) — a sustained, tangentially-swept friction grip.
const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const MU0: f64 = 3.0e3; // compliant block — a well-conditioned μ-lever for the FD gate
const FRIC_MU: f64 = 2.5;
const EPS_V: f64 = 0.1;

fn build(soft_mu: f64) -> StaggeredCoupling {
    let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3; // tilt off vertical
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, soft_mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_friction(FRIC_MU, EPS_V)
}

/// The total `∂(tip x)_N/∂μ` (along λ = 4μ) from one tape, and a central FD of the full real
/// grip re-rollout at μ±ε.
fn tape_and_fd(n: usize) -> (f64, f64, f64) {
    let (tip_x, g_mu) =
        build(MU0).coupled_trajectory_tangential_material_gradient_articulated(n, 0);
    let (_t2, g_la) = build(MU0).coupled_trajectory_tangential_material_gradient_articulated(n, 1);
    let total = g_mu + 4.0 * g_la;
    let eps = MU0 * 1e-4;
    let xp = build(MU0 + eps).coupled_trajectory_gripped_articulated(n).x;
    let xm = build(MU0 - eps).coupled_trajectory_gripped_articulated(n).x;
    let fd = (xp - xm) / (2.0 * eps);
    (tip_x, total, fd)
}

fn assert_matches(n: usize) {
    let (tip_x, total, fd) = tape_and_fd(n);
    assert!(tip_x.is_finite() && total.is_finite() && fd.is_finite());
    // Non-vacuity: a friction channel regressed to ~0 would make BOTH total and fd ~0, so a
    // pure relative gate could pass trivially. Require a real lever (mirrors the free-platen
    // `friction_coupled_trajectory_gradient` guard).
    assert!(
        total.abs() > 1e-9 && fd.abs() > 1e-9,
        "n={n}: gradient implausibly ~0 (total={total:e}, fd={fd:e}) — friction channel inert?"
    );
    let rel = (total - fd).abs() / fd.abs().max(1e-9);
    assert!(
        rel < 1e-6,
        "n={n}: tape gradient {total:e} vs full-grip FD {fd:e} (rel {rel:e}); tip_x={tip_x}"
    );
}

#[test]
fn articulated_friction_material_gradient_matches_fd_n5() {
    assert_matches(5);
}

#[test]
fn articulated_friction_material_gradient_matches_fd_n20() {
    assert_matches(20);
}

#[test]
fn articulated_friction_material_gradient_matches_fd_n40() {
    assert_matches(40);
}

// A 2-link hinge CHAIN (nv = 2) — genuine multi-DOF: off-diagonal mass coupling, Coriolis, and
// per-vertex friction moment arms that DON'T collapse to a single lever (the nv = 1 case hides a
// chain-only `J_lin`/arm indexing bug — exactly the moment-residual blind spot, see
// `docs/keystone/moment_residual_recon.md`). The lower link (body 2) is the contacting tip.
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

fn build_2link(soft_mu: f64) -> StaggeredCoupling {
    let model = load_model(TWOLINK_MJCF).expect("2-link MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.1;
    data.qpos[1] = -0.05; // off-center tip → the off-diagonal joint coupling is live
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 2, 0.005, 4, 0.1, soft_mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_friction(FRIC_MU, EPS_V)
}

/// The friction material gradient on a 2-link CHAIN matches the full-grip FD — the moment
/// cross-term and `J_lin` feedback on a genuine `nv > 1` topology (the analytic `chain_state_jacobian`
/// carry). Guards against a chain-only indexing/transpose bug the single-hinge tests cannot see.
#[test]
fn articulated_friction_material_gradient_2link_matches_fd() {
    let z0 = build_2link(MU0).data().xipos[2].z;
    assert!(
        z0 > 0.10,
        "2-link tip must start engaged near the block top, got {z0}"
    );
    for n in [3usize, 12] {
        let (_t, gm) =
            build_2link(MU0).coupled_trajectory_tangential_material_gradient_articulated(n, 0);
        let (_t2, gl) =
            build_2link(MU0).coupled_trajectory_tangential_material_gradient_articulated(n, 1);
        let total = gm + 4.0 * gl;
        let eps = MU0 * 1e-4;
        let xp = build_2link(MU0 + eps)
            .coupled_trajectory_gripped_articulated(n)
            .x;
        let xm = build_2link(MU0 - eps)
            .coupled_trajectory_gripped_articulated(n)
            .x;
        let fd = (xp - xm) / (2.0 * eps);
        assert!(
            total.abs() > 1e-9 && fd.abs() > 1e-9,
            "2-link n={n}: gradient implausibly ~0 (total={total:e}, fd={fd:e})"
        );
        let rel = (total - fd).abs() / fd.abs().max(1e-9);
        assert!(
            rel < 1e-5,
            "2-link n={n}: tape gradient {total:e} vs full-grip FD {fd:e} (rel {rel:e})"
        );
    }
}
