//! Keystone friction leaf PR3b-2b — the FULL friction-coupled trajectory gradient.
//!
//! `StaggeredCoupling::coupled_trajectory_tangential_material_gradient` rolls the grip
//! system forward `n` steps on ONE chassis tape, then a single `tape.backward` gives
//! `∂(platen final x)/∂p` — the platen's tangential slide vs the soft block's Neo-Hookean
//! material — with the reverse pass crossing BOTH step boundaries AND the soft↔rigid
//! interface, now through the TANGENTIAL friction grip: the moving-collider drift parent
//! (`vx → Δ_surf → x*`), the friction-reaction readout (`x* → fx`), and the tangential
//! rigid carry (`fx → vx' → x'`), closing the two-way grip feedback loop. The friction
//! reference `x_start = x_prev + Δ_surf` further couples the step-start config into both the
//! soft equilibrium and the reaction (the `x_prev` friction terms).
//!
//! Gate: the one-tape gradient matches a central FD of the FULL real grip rollout (a fresh
//! coupling at μ±ε re-running `coupled_trajectory_grip`) — an independent oracle. The block
//! ties `λ = 4μ`, so the constructor-FD measures the total `d/dμ|_{λ=4μ} = ∂/∂μ + 4·∂/∂λ`.
//!
//! **Accuracy / scope.** The composed gradient is MACHINE-EXACT over the full S0 creep
//! horizon (n ≈ 40): rel ~2e-6, holding clean through n = 60. `n = 1` is exactly 0 (position
//! integrates the pre-step velocity → the drift feedback is load-bearing only at n ≥ 2).
//!
//! The gate uses a **compliant block** (`μ = 3e3`, softer than the PR3a forward grip scene's
//! `3e4`): the softer block deforms more under the grip drag, so `μ` is a stronger lever on
//! the tangential slide — `∂x/∂μ` is then a well-conditioned ~5e-8 m rather than the ~4e-9 m
//! residual a stiff block leaves (where catastrophic cancellation of the large stiff friction
//! terms, `∇²D ~ 1e4`, floors the RELATIVE error at ~3e-4 while absolute agreement stays at
//! the f64 floor). Same engaged grip dynamics, just a sharper material lever for the FD gate.
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

// Compliant block (softer than the forward grip scene's 3e4) — a sharper μ-lever for the FD
// gate (see the module note). λ = 4μ tied by the constructor.
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

/// Final platen world x after `n` steps of the REAL grip dynamics at soft `mu`. Guards that
/// the platen stayed contact-engaged (z in the penalty band) so the gradient is meaningful.
fn final_x(soft_mu: f64, n: usize) -> f64 {
    let mut c = build_grip(soft_mu);
    let p = c.coupled_trajectory_grip(n);
    assert!(
        (0.100..0.116).contains(&p.z),
        "platen left the contact band at μ={soft_mu}: final z={} (expected engaged)",
        p.z
    );
    p.x
}

/// The tape's forward rollout reproduces the real grip dynamics exactly (the nodes carry
/// real `step` values; only the backward pass is analytic).
#[test]
fn tangential_forward_matches_grip_rollout() {
    let (x_n, _grad) = build_grip(MU0).coupled_trajectory_tangential_material_gradient(N, 0);
    let x_ref = final_x(MU0, N);
    assert!(
        (x_n - x_ref).abs() < 1e-12,
        "tape forward x_N {x_n} != real grip rollout {x_ref}"
    );
}

/// One `tape.backward` over the grip rollout matches the full-coupled FD oracle —
/// MACHINE-EXACT over the full S0 creep horizon (n = 40).
#[test]
fn tangential_trajectory_gradient_matches_full_fd() {
    let (_x, grad_mu) = build_grip(MU0).coupled_trajectory_tangential_material_gradient(N, 0);
    let (_x2, grad_lambda) = build_grip(MU0).coupled_trajectory_tangential_material_gradient(N, 1);
    // The block ties λ = 4μ, so the constructor-FD measures d/dμ|_{λ=4μ} = ∂/∂μ + 4·∂/∂λ.
    let grad_total = grad_mu + 4.0 * grad_lambda;
    let eps = MU0 * 1e-5;
    let fd = (final_x(MU0 + eps, N) - final_x(MU0 - eps, N)) / (2.0 * eps);
    let rel = (grad_total - fd).abs() / fd.abs().max(1e-30);
    eprintln!(
        "tangential grip gradient (n={N}): ∂/∂μ={grad_mu:.6e} ∂/∂λ={grad_lambda:.6e} \
         total(tape)={grad_total:.6e} FD={fd:.6e} rel={rel:.3e}"
    );
    assert!(
        grad_total.abs() > 1e-9,
        "gradient implausibly ~0 ({grad_total:e})"
    );
    // Machine-exact (~2e-6); gate at 1e-4 for cross-platform float-ordering margin. A
    // structural regression in any friction-coupled VJP edge blows this past 1.0.
    assert!(
        rel < 1e-4,
        "one-tape ∂x_N/∂μ {grad_total} disagrees with full-coupled FD {fd} (rel {rel:e})"
    );
}
