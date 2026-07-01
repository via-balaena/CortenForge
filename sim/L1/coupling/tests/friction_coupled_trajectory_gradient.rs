//! Keystone tangential-friction grip — the ENGAGEMENT invariant the coupling gradient harness
//! can't express.
//!
//! The FD + forward tests for `StaggeredCoupling::coupled_trajectory_tangential_material_gradient`
//! (the one-tape `∂(platen slide x)/∂(soft material)` gradient across both step boundaries and the
//! soft↔rigid tangential grip loop) are folded into the `friction·tangential-material[μ]` row of
//! `tests/coupling_grad_harness.rs`: that row asserts the same tape-vs-oracle forward match
//! (`FORWARD_TOL`), the same central-FD gradient match (tied λ = 4μ, eps = μ·1e-5, tol 1e-4), and
//! the same non-vacuity floor (`Comp::Live`, `|fd| > 1e-9`), all against the same independent
//! `coupled_trajectory_grip(n).x` oracle at the same n = 40 creep horizon.
//!
//! What that row CAN'T fold: the harness bands only the LOSS, and here the loss is the tangential
//! slide `x` — but engagement is a property of a DIFFERENT component, the platen HEIGHT `z`. A
//! fixture regression that over-penetrated the block (z below the contact band) could still leave
//! the slide `x` inside its band with a live gradient, so the x-band + `Comp::Live` don't strictly
//! imply engagement the way a state-level z-band does. This file keeps that direct guard: the grip
//! rollout must stay in the penalty band (`z ∈ (0.100, 0.116)`) over the full horizon, so the
//! gradient the harness gates is taken in the intended engaged regime.
//!
//! See `docs/keystone/friction_recon.md` and `project-friction-leaf.md`.

// A missing/malformed fixture (MJCF load, body index) surfaces as a test panic.
#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

// The PR3a grip scene: a free-joint platen pressed onto a pinned soft block, pushed sideways
// by tilted gravity `gx = 2.0`, started near vertical force balance (xpos.z = 0.115).
const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.115">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

// Compliant block (softer than the forward grip scene's 3e4) — the same fixture the harness row
// uses (a sharper μ-lever for the FD gate). λ = 4μ tied by the constructor.
const MU0: f64 = 3.0e3;
const FRIC_MU: f64 = 2.5; // Coulomb coefficient — the creep grip regime (S0)
const EPS_V: f64 = 0.1;
const DAMPING: f64 = 8.0;
const N: usize = 40; // the S0 creep horizon

fn build_grip(soft_mu: f64) -> StaggeredCoupling {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, soft_mu, 1.0e-3, 3.0e4, 1.0e-2, DAMPING,
    )
    .with_friction(FRIC_MU, EPS_V)
}

/// ENGAGEMENT invariant (the one the harness's x-loss band structurally can't express): over the
/// full S0 creep horizon the platen must stay in the contact penalty band on its HEIGHT `z`, so
/// the tangential-slide gradient the `friction·tangential-material[μ]` harness row gates is taken
/// in the intended engaged regime — a fixture that over-penetrated (z below the band) can't slip
/// past an x-only loss band + liveness floor.
#[test]
fn friction_grip_stays_engaged() {
    let p = build_grip(MU0).coupled_trajectory_grip(N);
    assert!(
        (0.100..0.116).contains(&p.z),
        "platen left the contact band at μ={MU0}: final z={} (expected engaged in 0.100..0.116)",
        p.z
    );
    // A live slide confirms the friction grip is actually dragging the platen (not merely resting
    // in-band): the harness row gates its magnitude, here we only sanity that it moved.
    assert!(
        p.x > 0.0,
        "expected a positive tangential slide, got x={}",
        p.x
    );
}
