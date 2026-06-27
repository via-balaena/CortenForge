//! L1b free-body TANGENTIAL/friction curvature carry — the friction-coupled trajectory
//! gradient on a FINITE posed sphere collider (end-to-end composition check).
//!
//! The friction successor to `sphere_trajectory_gradient.rs` (the NORMAL crossing on the
//! sphere): the same grip tape machinery (`coupled_trajectory_tangential_*`) against the same
//! full-grip-rollout FD oracle (`coupled_trajectory_grip`) as the plane gate
//! `friction_coupled_trajectory_coeff_gradient.rs`, but with the collider swapped to a finite
//! `TranslatedSdf<SphereSdf>` (`with_sphere_collider`).
//!
//! Why it's a genuinely new check: the friction force `∇D = μ·λⁿ·f₁·T·û` turns its tangent
//! frame `T` as the sphere's contact normal turns — both as a soft vertex slides over it
//! (`∂n̂/∂x = H`) and as the primitive translates with the rigid height (`∂n̂/∂h = −H·ẑ`). That
//! curved-normal term `DN = ∂∇D/∂n̂` is identically zero for the plane's constant normal, so it
//! is dropped in three dual sites (the rigid-reaction readout `friction_reaction_gradients`,
//! the soft adjoint's friction tangent `A`, and the friction pose-residual grad) — invisible to
//! every plane gate.
//!
//! ## Accuracy / scope — the ~2e-3 floor is INTRINSIC (do not chase it; spike falsified the fix).
//! Each curved-friction term is MACHINE-EXACT, gated per-term by the single-step FD isolation
//! suite `sim/L0/soft/tests/friction_sphere_tangent.rs` (`∂F/∂x*`, `∂F/∂height`, `∂x*/∂μ_c`,
//! `∂x*/∂height`, `∂x*/∂drift`, plus the reverse-mode `A⁻ᵀ == A⁻¹` check) — that suite is the
//! rigorous validation. THIS end-to-end gate composes them over a rollout and is bounded NOT by
//! the gradient (which is exact) but by the FORWARD SOLVER: the frozen-lag friction on a curved
//! contact plateaus at a ~1e-10 forward residual it cannot drive lower, and the IFT adjoint (which
//! assumes `r(x*) = 0`) amplifies that residual by the curved-contact conditioning into the floor.
//! The gradient is the EXACT gradient of what the solve produces; it tracks the FD oracle (which
//! rides the same forward solve) to that floor. A well-conditioned (compliant `κ`) scene lands
//! ~2e-3; a stiff scene drifts to ~1e-2. Normal-only curved contact has NO such floor
//! (`sphere_trajectory_gradient.rs` / `sphere_articulated_trajectory_gradient.rs` are machine-exact
//! end-to-end) — it is the frozen-lag friction model, not the curvature conditioning.
//!
//! This floor is INTRINSIC, not a preconditioning gap. A 2026-06-27 implement-measure-revert spike
//! falsified all three tangent levers: (1) `lm_regularization` is INERT — the forward tangent stays
//! positive-definite (Cholesky succeeds), so the `dE·H` NSD term never triggers the `+λI` retry;
//! (2) tightening tol 1e-10→1e-13 with 5× the iter budget hits `NewtonIterCap` (the ~1e-10 plateau
//! is a linear-convergence stall, not a budget problem); (3) threading the curved `DN·C` into the
//! forward friction Hessian (`friction_blocks`) makes it WORSE (~6e-2) by desyncing forward `x*`
//! from the adjoint's frozen-lag tangent. The only conceivable fix is a fully consistent ASYMMETRIC
//! forward tangent (Woodbury in the forward solve) — large change, speculative payoff, NOT pursued.
//! The friction-COEFFICIENT lever is used (linear in `μ_c`, a far cleaner signal than the weak
//! material lever). See `project-differentiable-finite-contact.md`.

// A missing/malformed fixture (MJCF load, body index) surfaces as a test panic.
#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

// A free-joint platen pressed onto a pinned soft block, pushed sideways by tilted gravity
// `gx = 2.0`. The sphere's south pole sits at `height = xpos.z − clearance`, so at z = 0.112
// (clearance 0.005) the pole is 0.007 into the d_hat = 0.01 contact band over the rest top
// face (z = 0.1) — a curved contact patch spanning the central top-face vertices under the
// sideways grip drag.
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
/// vertices yet stays curved enough that the geometric term is materially nonzero (mirrors the
/// normal sphere gate's `SPHERE_R`).
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

/// Final platen world x after `n` steps of the REAL grip dynamics at Coulomb `μ_c`. The grip
/// rollout (`coupled_trajectory_grip`) is SDF-generic, so it drives the actual posed sphere
/// contact. Guards that the platen stayed contact-engaged (z in the penalty band).
fn final_x(fric_mu: f64, n: usize) -> f64 {
    let mut c = build_grip(fric_mu);
    let p = c.coupled_trajectory_grip(n);
    assert!(
        (0.100..0.120).contains(&p.z),
        "platen left the contact band at μ_c={fric_mu}: final z={} (expected engaged)",
        p.z
    );
    p.x
}

/// The tape's forward rollout reproduces the real sphere grip dynamics exactly (the nodes carry
/// real `step` values; only the backward pass is analytic) — confirms the curved-collider
/// forward path is the same one the FD oracle differentiates.
#[test]
fn sphere_tangential_forward_matches_grip_rollout() {
    let (x_n, _grad) = build_grip(FRIC_MU).coupled_trajectory_tangential_friction_coeff_gradient(N);
    let x_ref = final_x(FRIC_MU, N);
    assert!(
        (x_n - x_ref).abs() < 1e-12,
        "tape forward x_N {x_n} != real sphere grip rollout {x_ref}"
    );
}

/// One `tape.backward` over the curved-collider grip rollout tracks the full-coupled FD oracle.
/// The curved-normal `DN` terms (zero for the plane, machine-exact per-term in
/// `friction_sphere_tangent.rs`) are what bring this from the ~1e-2 no-curvature miss down to
/// the forward-solver floor; the residual is curved-contact convergence, NOT a gradient error
/// (see the module note — this ~2e-3 floor is INTRINSIC; a spike falsified the tangent-fix levers).
#[test]
fn sphere_tangential_friction_gradient_matches_fd() {
    let (_x, grad) = build_grip(FRIC_MU).coupled_trajectory_tangential_friction_coeff_gradient(N);
    // FD-step sweep: take the best step (the curved-contact convergence floor sets the bottom).
    let mut best = f64::INFINITY;
    let mut best_fd = 0.0;
    for k in 3..=7 {
        let eps = FRIC_MU * 10f64.powi(-k);
        let fd = (final_x(FRIC_MU + eps, N) - final_x(FRIC_MU - eps, N)) / (2.0 * eps);
        let r = (grad - fd).abs() / fd.abs().max(1e-30);
        if r < best {
            best = r;
            best_fd = fd;
        }
    }
    eprintln!(
        "sphere tangential μ_c gradient (n={N}): tape={grad:.6e} FD={best_fd:.6e} rel={best:.3e}"
    );
    assert!(grad.abs() > 1e-9, "gradient implausibly ~0 ({grad:e})");
    // Honest bound at the curved-contact forward-solver floor (~2e-3 at this compliant κ; the
    // no-curvature drop misses by ~1e-2 — the curved terms close most of that). The rigorous
    // machine-exact validation is the per-term single-step suite, not this composition gate.
    assert!(
        best < 5e-3,
        "one-tape sphere ∂x_N/∂μ_c {grad} disagrees with full-coupled FD {best_fd} (rel {best:e})"
    );
}

/// COMPOSITION smoke test for the *other* lifted method — `coupled_trajectory_tangential_
/// material_gradient` (the soft MATERIAL lever) — on a sphere: it runs (the guard is lifted), its
/// tape forward matches the real rollout, and its gradient is gross-sane vs FD. This is NOT a
/// curvature gate: the material lever is so WEAK (the friction reaction barely depends on the
/// block's Neo-Hookean μ in creep slip) that the curved `DN·C` term sits below the FD noise floor
/// — zeroing the curvature leaves the rel essentially unchanged, so a curvature regression would
/// NOT trip this. The material method's curvature CORRECTNESS is inherited by construction: it
/// shares every curvature-bearing factory (the curved tangent `A`, `FrictionReactionTrajVjp`,
/// `active_pair_force_factors`) with the μ_c gate above (which DOES discriminate the term) and the
/// single-step suite; only the collider-agnostic material residual RHS differs. The block ties
/// `λ = 4μ`, so the constructor-FD measures `d/dμ|_{λ=4μ} = ∂/∂μ + 4·∂/∂λ`.
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
    // Gross-error tripwire only (the curvature discrimination lives in the μ_c + single-step gates):
    // the weak material lever can't resolve the curved term, so this catches a structural blow-up,
    // not a curvature regression.
    assert!(
        best < 5e-2,
        "one-tape sphere ∂x_N/∂μ {grad} disagrees with full-coupled FD {best_fd} (rel {best:e})"
    );
}
