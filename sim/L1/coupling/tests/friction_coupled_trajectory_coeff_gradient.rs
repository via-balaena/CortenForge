//! Friction-coefficient leaf — the FULL friction-coupled trajectory gradient w.r.t. the Coulomb
//! COEFFICIENT `μ_c`.
//!
//! `StaggeredCoupling::coupled_trajectory_tangential_friction_coeff_gradient` rolls the grip
//! system forward `n` steps on ONE chassis tape, then a single `tape.backward` gives
//! `∂(platen final x)/∂μ_c`. The reverse pass crosses both step boundaries and the soft↔rigid
//! interface, the same nine-node-per-step tape as the material gradient — but `μ_c` enters
//! through TWO channels the material parameter lacks: (1) via `x*` (the soft equilibrium shift,
//! tiny in deep slip) and (2) DIRECTLY through the friction reaction `fx = (Σ μ_c·λⁿ·…)·react_dir`
//! (`∂fx/∂μ_c = fx/μ_c`, the dominant term in the Coulomb-saturated regime). Both feed the one
//! `μ_c` param leaf; the tape sums them.
//!
//! Gate: the one-tape gradient matches a central FD of the FULL real grip rollout (a fresh
//! coupling at `μ_c ± ε` re-running `coupled_trajectory_grip`) — an independent oracle.
//!
//! **Accuracy / scope.** The composed gradient is machine-exact over the full S0 creep horizon
//! (n ≈ 40): rel ~5e-6. `n = 1` is exactly 0 (position integrates the pre-step velocity → the
//! grip feedback is load-bearing only at n ≥ 2).
//!
//! The gate uses the SAME compliant block (`μ = 3e3`) as the material trajectory gate, and for
//! the same reason: on a STIFF block the FD re-solve suffers catastrophic cancellation of the
//! large friction terms (`∇²D ~ 1e4`), flooring the FD-vs-analytic agreement at ~3e-4 for BOTH
//! `μ_c` and the material parameter (measured: μ_c-stiff 2.8e-4, material-stiff 3.3e-4 — the
//! same conditioning limit, not a `μ_c`-specific defect). `μ_c`'s linear lever DOES buy
//! machine-exactness even at stiff for the soft-only forward sensitivity `∂x*/∂μ_c` (see the
//! sim-soft `friction_coeff_gradient` test, stiff + compliant); the COUPLED FD oracle, routing
//! through the same stiff Woodbury/reaction terms, is conditioning-limited like the material one.
//! See `project-friction-leaf.md`.

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

// Compliant block (same as the material trajectory gate) — the FD oracle's stiff-friction
// cancellation floors the relative agreement at ~3e-4 on a stiff block (for μ_c AND material
// alike), so the coupled gate uses the softer, better-conditioned block.
const MU0: f64 = 3.0e3;
const FRIC_MU: f64 = 2.5; // Coulomb coefficient — the creep grip regime (S0); the gradient target
const EPS_V: f64 = 0.1;
const DAMPING: f64 = 8.0;
const N: usize = 40; // the S0 creep horizon

fn build_grip(fric_mu: f64) -> StaggeredCoupling {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, MU0, 1.0e-3, 3.0e4, 1.0e-2, DAMPING,
    )
    .with_friction(fric_mu, EPS_V)
}

/// Final platen world x after `n` steps of the REAL grip dynamics at friction coeff `fric_mu`.
/// Guards that the platen stayed contact-engaged (z in the penalty band).
fn final_x(fric_mu: f64, n: usize) -> f64 {
    let mut c = build_grip(fric_mu);
    let p = c.coupled_trajectory_grip(n);
    assert!(
        (0.100..0.116).contains(&p.z),
        "platen left the contact band at μ_c={fric_mu}: final z={} (expected engaged)",
        p.z
    );
    p.x
}

/// The tape's forward rollout reproduces the real grip dynamics exactly (the μ_c gradient node
/// adds reverse-mode edges only; the forward values are the real `step` outputs).
#[test]
fn friction_coeff_forward_matches_grip_rollout() {
    let (x_n, _grad) = build_grip(FRIC_MU).coupled_trajectory_tangential_friction_coeff_gradient(N);
    let x_ref = final_x(FRIC_MU, N);
    assert!(
        (x_n - x_ref).abs() < 1e-12,
        "tape forward x_N {x_n} != real grip rollout {x_ref}"
    );
}

/// One `tape.backward` over the grip rollout matches the full-coupled FD oracle — machine-exact
/// over the full S0 creep horizon (n = 40, ~5e-6 on the compliant block).
#[test]
fn friction_coeff_trajectory_gradient_matches_full_fd() {
    let (_x, grad) = build_grip(FRIC_MU).coupled_trajectory_tangential_friction_coeff_gradient(N);
    let eps = FRIC_MU * 1e-5;
    let fd = (final_x(FRIC_MU + eps, N) - final_x(FRIC_MU - eps, N)) / (2.0 * eps);
    let rel = (grad - fd).abs() / fd.abs().max(1e-30);
    eprintln!("μ_c grip gradient (n={N}): tape={grad:.6e} FD={fd:.6e} rel={rel:.3e}");
    assert!(grad.abs() > 1e-9, "gradient implausibly ~0 ({grad:e})");
    // Machine-exact (~5e-6); gate at 1e-4 for cross-platform float-ordering margin. A dropped
    // channel (e.g. the direct reaction term `∂fx/∂μ_c`) blows this past 1.0.
    assert!(
        rel < 1e-4,
        "one-tape ∂x_N/∂μ_c {grad} disagrees with full-coupled FD {fd} (rel {rel:e})"
    );
}
