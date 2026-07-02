//! Curved-collider (sphere) FRICTION grip — the composition smoke + engagement invariant the
//! coupling gradient harness can't express, and the canonical note on the INTRINSIC ~2e-3 floor.
//!
//! The μ_c curvature gate (`coupled_trajectory_tangential_friction_coeff_gradient` on a finite
//! `SphereSdf` — the friction force Jacobian's `DN·C` curved-normal term, zero for the plane) is
//! folded into the `sphere·friction-coeff[μ_c]` row of `tests/coupling_grad_harness.rs`, which
//! asserts the same FD match against the same `coupled_trajectory_grip(n).x` oracle at the same
//! compliant `κ = 2e3` scene and n = 8 horizon, to the same ~2e-3 curved-contact floor (tol 5e-3),
//! plus forward-consistency and a `Comp::Live` non-vacuity floor.
//!
//! What that row CAN'T fold, kept here:
//! - the **material composition smoke** — the OTHER lifted method
//!   (`coupled_trajectory_tangential_material_gradient`) runs on the sphere, its forward matches the
//!   real rollout, and it is gross-sane vs FD. It is NOT a curvature gate: the material lever is so
//!   WEAK (the friction reaction barely depends on the block's Neo-Hookean μ in creep slip) that the
//!   curved `DN·C` term sits below the FD noise floor — zeroing the curvature leaves the rel
//!   essentially unchanged, so it can't be a discriminating harness row. Its curvature correctness
//!   is inherited by construction (it shares every curvature-bearing factory with the μ_c gate above
//!   and the single-step suite); this is a wiring/gross-error tripwire only.
//! - the **z-engagement** — the harness bands only the LOSS (the tip-`x` slide, ~5e-5 here), so this
//!   state-level guard on the platen HEIGHT `z` isn't expressible as a loss band.
//!
//! ## Accuracy / scope — the ~2e-3 floor is INTRINSIC (do not chase it; a spike falsified the fix).
//! Each curved-friction term is MACHINE-EXACT, gated per-term by the single-step FD isolation suite
//! `sim/L0/soft/tests/friction_sphere_tangent.rs` (`∂F/∂x*`, `∂F/∂height`, `∂x*/∂μ_c`, …, plus the
//! reverse-mode `A⁻ᵀ == A⁻¹` check) — that suite is the rigorous validation. The end-to-end μ_c row
//! composes them over a rollout and is bounded NOT by the gradient (exact) but by the FORWARD
//! SOLVER: the frozen-lag friction on a curved contact plateaus at a ~1e-10 forward residual it
//! cannot drive lower, and the IFT adjoint (which assumes `r(x*) = 0`) amplifies that residual by
//! the curved-contact conditioning into the floor. The gradient is the EXACT gradient of what the
//! solve produces; it tracks the FD oracle (which rides the same forward solve) to that floor. A
//! well-conditioned (compliant `κ`) scene lands ~2e-3; a stiff scene drifts to ~1e-2. Normal-only
//! curved contact has NO such floor (the `sphere·material[μ]` / `sphere-articulated·material[μ]`
//! rows are machine-exact end-to-end) — it is the frozen-lag friction model, not the curvature
//! conditioning. A 2026-06-27 implement-measure-revert spike falsified all three tangent levers
//! (`lm_regularization` inert; tol 1e-10→1e-13 hits `NewtonIterCap`; threading `DN·C` into the
//! forward Hessian makes it WORSE by desyncing forward `x*` from the frozen-lag adjoint tangent) —
//! the only conceivable fix is a fully consistent asymmetric forward tangent, NOT pursued. See
//! `project-differentiable-finite-contact.md`.

// A missing/malformed fixture (MJCF load, body index) surfaces as a test panic.
#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

// A free-joint platen pressed onto a pinned soft block, pushed sideways by tilted gravity
// `gx = 2.0`. The sphere's south pole sits 0.007 into the d_hat = 0.01 contact band over the rest
// top face (z = 0.1) — a curved contact patch spanning the central top-face vertices under drag.
const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.112">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const MU0: f64 = 3.0e3; // compliant block — a sharper lever; λ = 4μ tied by the constructor
const KAPPA: f64 = 2.0e3; // compliant CONTACT — the best-conditioned curved+friction solve
const FRIC_MU: f64 = 2.5; // Coulomb coefficient — the creep grip regime
const EPS_V: f64 = 0.1;
const DAMPING: f64 = 8.0;
const N: usize = 8; // short engaged horizon: the finite patch stays stable, the curved solve
// stays as converged as it gets (longer rollouts accrue the per-step residual, see module note)
/// Sphere radius (m). Large vs the 0.1 m block so the south-pole patch spans several top-face
/// vertices yet stays curved enough that the geometric term is materially nonzero.
const SPHERE_R: f64 = 0.08;

fn build_grip(fric_mu: f64) -> StaggeredCoupling {
    build(MU0, fric_mu)
}

fn build(soft_mu: f64, fric_mu: f64) -> StaggeredCoupling {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, soft_mu, 1.0e-3, KAPPA, 1.0e-2, DAMPING,
    )
    .with_friction(fric_mu, EPS_V)
    .with_sphere_collider(SPHERE_R)
}

/// Final platen x at soft material `mu` (λ = 4μ tied) — the FD oracle for the MATERIAL lever.
fn final_x_mat(soft_mu: f64, n: usize) -> f64 {
    let mut c = build(soft_mu, FRIC_MU);
    c.coupled_trajectory_grip(n).x
}

/// Z-ENGAGEMENT invariant (the state guard the μ_c harness row's tip-`x` loss band can't express):
/// over the curved-contact creep horizon the platen must stay in the contact penalty band on its
/// HEIGHT `z`, so the `sphere·friction-coeff[μ_c]` gradient the harness gates is taken in the
/// intended engaged regime rather than after the curved patch has slipped off.
#[test]
fn sphere_grip_stays_engaged() {
    let p = build_grip(FRIC_MU).coupled_trajectory_grip(N);
    assert!(
        (0.100..0.120).contains(&p.z),
        "platen left the contact band at μ_c={FRIC_MU}: final z={} (expected engaged)",
        p.z
    );
}

/// COMPOSITION smoke for the *other* lifted method — `coupled_trajectory_tangential_material_gradient`
/// (the soft MATERIAL lever) — on a sphere: it runs (the guard is lifted), its tape forward matches
/// the real rollout, and its gradient is gross-sane vs FD. This is NOT a curvature gate (see the
/// module note): the material lever is so WEAK that the curved `DN·C` term sits below the FD noise
/// floor — zeroing the curvature leaves the rel essentially unchanged, so a curvature regression
/// would NOT trip this and it can't be a discriminating harness row. The material method's curvature
/// CORRECTNESS is inherited by construction: it shares every curvature-bearing factory (the curved
/// tangent `A`, `FrictionReactionTrajVjp`, `active_pair_force_factors`) with the μ_c harness row
/// (which DOES discriminate the term) and the single-step suite; only the collider-agnostic material
/// residual RHS differs. The block ties `λ = 4μ` (`d/dμ|_{λ=4μ} = ∂/∂μ + 4·∂/∂λ`).
#[test]
fn sphere_tangential_material_gradient_composes() {
    let (x_n, grad_mu) = build_grip(FRIC_MU).coupled_trajectory_tangential_material_gradient(N, 0);
    let grad_lambda = build_grip(FRIC_MU)
        .coupled_trajectory_tangential_material_gradient(N, 1)
        .1;
    let grad = grad_mu + 4.0 * grad_lambda;
    // The lifted material method runs on the sphere and its forward reproduces the real rollout.
    assert!(
        (x_n - final_x_mat(MU0, N)).abs() < 1e-12,
        "material tape forward x_N {x_n} != real sphere rollout"
    );
    let mut best = f64::INFINITY;
    let mut best_fd = 0.0;
    for k in 3..=6 {
        let eps = MU0 * 10f64.powi(-k);
        let fd = (final_x_mat(MU0 + eps, N) - final_x_mat(MU0 - eps, N)) / (2.0 * eps);
        let r = (grad - fd).abs() / fd.abs().max(1e-30);
        if r < best {
            best = r;
            best_fd = fd;
        }
    }
    eprintln!(
        "sphere tangential material gradient (n={N}): tape={grad:.6e} FD={best_fd:.6e} rel={best:.3e}"
    );
    assert!(
        grad.abs() > 1e-12,
        "material gradient implausibly ~0 ({grad:e})"
    );
    // Gross-error tripwire only (curvature discrimination lives in the μ_c row + single-step gates):
    // the weak material lever can't resolve the curved term, so this catches a structural blow-up,
    // not a curvature regression.
    assert!(
        best < 5e-2,
        "one-tape sphere ∂x_N/∂μ {grad} disagrees with full-coupled FD {best_fd} (rel {best:e})"
    );
}
