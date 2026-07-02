//! Articulated FRICTION material gradient — the invariants the coupling gradient harness can't
//! express.
//!
//! The per-horizon FD cells of `StaggeredCoupling::coupled_trajectory_tangential_material_gradient_articulated`
//! (the tip-`x` slide vs the soft material on a hinge / 2-link grip, the wrench routed through the
//! joint carry `Δt·M⁻¹·Jᵀ` with its off-COM moment) are folded into the `hinge·friction-material[μ]`
//! and `chain·friction-material[μ]` rows of `tests/coupling_grad_harness.rs`, which assert the same
//! tied-λ = 4μ FD match against the same `coupled_trajectory_gripped_articulated(n).x` oracle plus
//! forward-consistency and a `Comp::Live` non-vacuity floor.
//!
//! What those single-horizon rows CAN'T fold, kept here:
//! - the **hinge multi-horizon machine-exactness** — the composed gradient stays FD-limited
//!   (rel ~8e-9) as the rollout lengthens across the make, a liftoff, and a re-touch (n = 5, 20, 40),
//!   a cross-horizon invariant a single-length matrix cell can't hold;
//! - the **2-link start-engagement** — the chain tip must begin engaged near the block top; the
//!   harness bands only the LOSS (the tip-`x` slide, which swings sign across the horizon), so this
//!   state-level guard on the tip HEIGHT `z` isn't expressible as a loss band.
//!
//! See `docs/keystone/moment_residual_recon.md` and `project-friction-leaf.md`.

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

/// HINGE multi-horizon machine-exactness (the cross-horizon invariant the single-length matrix row
/// can't express): the composed friction-material gradient stays FD-limited as the rollout lengthens
/// across the make, a liftoff, and a re-touch. The `Comp::Live`-style non-vacuity floor guards a
/// silently de-coupled channel from passing on `0 ≈ 0`.
#[test]
fn articulated_friction_material_gradient_machine_exact_at_all_lengths() {
    for n in [5usize, 20, 40] {
        let (tip_x, total, fd) = tape_and_fd(n);
        assert!(tip_x.is_finite() && total.is_finite() && fd.is_finite());
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
}

// A 2-link hinge CHAIN (nv = 2) — genuine multi-DOF: off-diagonal mass coupling, Coriolis, and
// per-vertex friction moment arms that DON'T collapse to a single lever (the nv = 1 case hides a
// chain-only `J_lin`/arm indexing bug — the moment-residual blind spot, see
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

/// The 2-link chain's START-ENGAGEMENT (the state guard the `chain·friction-material[μ]` harness
/// row can't express — the row bands the tip-`x` loss, engagement is the tip HEIGHT `z`): the
/// contacting tip must begin near the block top, so the FD cell the harness gates is taken in the
/// intended engaged regime rather than in free flight.
#[test]
fn articulated_friction_material_chain_starts_engaged() {
    let z0 = build_2link(MU0).data().xipos[2].z;
    assert!(
        z0 > 0.10,
        "2-link tip must start engaged near the block top, got {z0}"
    );
}
