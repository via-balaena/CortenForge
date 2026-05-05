//! neo-hookean-uniaxial — direct-eval NH constitutive surface vs closed form.
//!
//! Direct-evaluation comparison of `NeoHookean::first_piola` /
//! `NeoHookean::energy` against the closed-form Neo-Hookean stress and
//! energy for **traction-free uniaxial stretch**, the canonical NH
//! benchmark per Part 2 Ch 04 [00-energy.md][e] + [01-tangent.md][t].
//! Per inventory Q4 row 5 visualization, JSON-only (no `cf-view`,
//! single-tet has trivial geometry; the curve IS the artifact).
//!
//! # Deformation gradient
//!
//! `F = diag(λ, λ_t, λ_t)` with the transverse stretch `λ_t` chosen so
//! the transverse stress vanishes (`P_22 = 0`). That makes `λ_t` the
//! Newton solution of the scalar transcendental
//!
//! ```text
//!     f(λ_t) = μ λ_t² + Λ ln(λ · λ_t²) − μ = 0
//! ```
//!
//! `f` is strictly monotone (`f'(λ_t) = 2μ λ_t + 2Λ/λ_t > 0`), so Newton
//! from `λ_t = 1` converges quadratically in ≤ 6 iter across the
//! `λ ∈ [0.15, 1.95]` sweep bracket.
//!
//! # Closed forms (compressible NH, μ shear, Λ first-Lamé)
//!
//! ```text
//!     I₁   = λ² + 2 λ_t²
//!     J    = λ · λ_t²
//!     ψ    = (μ/2)(I₁ − 3) + (Λ/2)(ln J)² − μ ln J
//!     P_11 = μ(λ   − 1/λ  ) + Λ ln(J) / λ
//!     P_22 = μ(λ_t − 1/λ_t) + Λ ln(J) / λ_t   (≡ 0 by traction-free construction)
//! ```
//!
//! # Sweep — 12 points across `λ ∈ [0.15, 1.95]`
//!
//! Asymmetric coverage by the in-domain bracket's structure: the
//! compressive boundary at `λ ≈ 0.118` is set by `λ_t → 2` (transverse
//! blow-up), while the tensile boundary at `λ = 2` is set by `λ` directly.
//! Compressive side is more sensitive to small `λ` shifts, so the sweep
//! sits at ~27% margin from the lower boundary vs ~2.5% from the upper.
//!
//! # Anchor groups (all assertions exit-0 on success)
//!
//! - **Validity domain declaration** — `mat.validity()` exact reads.
//! - **Inner Newton convergence** — per-point residual + iter + λ_t bracket.
//! - **Closed-form `P_11` agreement** — analytic vs observed, rel `1e-12`.
//! - **Closed-form `P_22` zero** — observed within Newton precision.
//! - **Closed-form ψ agreement** — analytic vs observed, rel `1e-12`.
//! - **Rest-config zero** — at λ=1: `P_11`, `P_22`, ψ all bit-equal `0u64`.
//! - **Monotonicity and sign** — `P_11` strictly increasing; sign matches `λ−1`.
//! - **Sweep in-domain** — `max |σᵢ − 1| < 1.0` at every point.
//! - **Sweep bit-equal** — observed `P_11`, `P_22`, ψ, λ_t per point match
//!   the captured reference bits (48 pins). IV-1 two-tier contract;
//!   failure-mode protocol verbatim per IV-1.
//!
//! [e]: docs/studies/soft_body_architecture/src/20-materials/04-hyperelastic/00-neo-hookean/00-energy.md
//! [t]: docs/studies/soft_body_architecture/src/20-materials/04-hyperelastic/00-neo-hookean/01-tangent.md

// `doc_markdown` flags Unicode math notation (`λ`, `λ_t`, `μ`, `Λ`, `σᵢ`)
// as if they were unbacktrick-quoted code identifiers. The notation matches
// the closed-form math in this file's docstrings (and in the parent NH
// implementation at `sim/L0/soft/src/material/neo_hookean.rs`); backticking
// every Greek symbol would clutter the prose without adding signal. Code
// identifiers (`mat.first_piola`, `SWEEP_*_BITS`, etc.) ARE backticked.
#![allow(clippy::doc_markdown)]

use std::path::Path;

use anyhow::{Context, Result, bail};
use approx::assert_relative_eq;
use nalgebra::{Matrix3, Vector3};
use serde_json::json;
use sim_soft::{InversionHandling, Material, NeoHookean};

// =============================================================================
// Constants — canonical Ecoflex-class compressible NH (`ν ≈ 0.4`).
// =============================================================================

/// Shear modulus (Pa). Same value as `SoftScene::one_tet_cube`'s
/// `MaterialField::uniform(1.0e5, 4.0e5)` first parameter.
const MU_PA: f64 = 1.0e5;

/// First Lamé parameter (Pa). `Λ = 4μ` ⇒ `ν = 0.4` compressible NH per
/// the Phase H deferral framing in `project_layered_silicone_device`.
const LAMBDA_PA: f64 = 4.0e5;

/// Number of sweep points.
const N_SWEEP: usize = 12;

/// Sweep stretches `λ`, asymmetric density per the in-domain bracket
/// `λ ∈ [≈ 0.118, 2.0]`. Compressive side denser (boundary more
/// sensitive); tensile side sparser. Bracket-margin: compressive 27%,
/// tensile 2.5%.
const SWEEP_LAMBDAS: [f64; N_SWEEP] = [
    0.15, 0.20, 0.30, 0.50, // compressive sub-trace + working compressive
    0.70, 0.85, // approach to rest from below
    1.00, // rest
    1.20, 1.50, 1.75, // working tensile
    1.90, 1.95, // tensile sub-trace
];

// =============================================================================
// Inner Newton — λ_t for traction-free uniaxial.
// =============================================================================

/// Maximum inner-Newton iterations. From `λ_t = 1`, quadratic
/// convergence reaches `<1e-15` absolute step in ≤ 6 iter across the
/// `λ ∈ [0.15, 1.95]` bracket; 30 leaves a wide margin for sweep
/// extension before the abort fires.
const LAMBDA_T_NEWTON_MAX_ITER: usize = 30;

/// Convergence tolerance on the Newton step `|Δλ_t|`. `λ_t` sits in
/// `(0, 2)` across the bracket; `1e-15` is at the f64 precision floor
/// on values of that magnitude — tightest possible without flapping
/// on round-off.
const LAMBDA_T_STEP_TOL: f64 = 1.0e-15;

/// Bound on `|f(λ_t)|` at convergence, scaled to the dominant
/// constitutive parameter. `Λ · 1e-14` ≈ `4e-9` Pa here — 13 orders
/// below typical NH stress magnitudes, catches any genuine convergence
/// regression without flapping on f64 round-off.
const NEWTON_RESIDUAL_REL_TO_LAMBDA: f64 = 1.0e-14;

/// Solve `μ λ_t² + Λ ln(λ · λ_t²) − μ = 0` for `λ_t`. Returns
/// `(λ_t, iter_count, residual_at_solution)` on convergence; `bail`s
/// if the iteration budget is exhausted (which would indicate the
/// monotonicity guarantee `f'(λ_t) > 0` was somehow violated — a real
/// regression, not a tolerance-flap).
fn solve_lambda_t(lambda: f64, mu: f64, lambda_lame: f64) -> Result<(f64, usize, f64)> {
    assert!(
        lambda > 0.0,
        "solve_lambda_t requires λ > 0 (got λ = {lambda}); negative or zero λ \
         is element inversion territory and lies outside the constitutive \
         surface this helper covers"
    );
    let mut lambda_t = 1.0_f64;
    for iter in 0..LAMBDA_T_NEWTON_MAX_ITER {
        let j = lambda * lambda_t * lambda_t;
        let f = mu.mul_add(lambda_t * lambda_t, lambda_lame * j.ln()) - mu;
        let f_prime = 2.0_f64.mul_add(mu * lambda_t, 2.0 * lambda_lame / lambda_t);
        let step = f / f_prime;
        lambda_t -= step;
        if step.abs() < LAMBDA_T_STEP_TOL {
            let j_final = lambda * lambda_t * lambda_t;
            let residual = mu.mul_add(lambda_t * lambda_t, lambda_lame * j_final.ln()) - mu;
            return Ok((lambda_t, iter + 1, residual));
        }
    }
    bail!(
        "inner Newton for λ_t did not converge within {LAMBDA_T_NEWTON_MAX_ITER} iter \
         at λ = {lambda} (μ = {mu}, Λ = {lambda_lame}). The monotonicity \
         guarantee f'(λ_t) > 0 should make divergence impossible — investigate \
         whether the closed form was edited or λ left the in-domain bracket."
    );
}

// =============================================================================
// Closed-form analytic helpers (traction-free uniaxial).
// =============================================================================

/// Closed-form energy density `ψ(λ, λ_t)` for compressible NH under
/// traction-free uniaxial stretch.
fn analytic_psi(lambda: f64, lambda_t: f64, mu: f64, lambda_lame: f64) -> f64 {
    let i_1 = lambda.mul_add(lambda, 2.0 * lambda_t * lambda_t);
    let j = lambda * lambda_t * lambda_t;
    let ln_j = j.ln();
    let half_mu = 0.5 * mu;
    let half_lambda = 0.5 * lambda_lame;
    half_lambda.mul_add(ln_j * ln_j, half_mu.mul_add(i_1 - 3.0, -mu * ln_j))
}

/// Closed-form `P_11(λ, λ_t)` for compressible NH under traction-free
/// uniaxial stretch.
fn analytic_p11(lambda: f64, lambda_t: f64, mu: f64, lambda_lame: f64) -> f64 {
    let j = lambda * lambda_t * lambda_t;
    mu.mul_add(lambda - 1.0 / lambda, lambda_lame * j.ln() / lambda)
}

/// Closed-form `P_22(λ, λ_t)` for compressible NH. Identically zero
/// when `λ_t` is the traction-free Newton solution; left explicit so
/// the `closed_form_P22_zero` anchor compares observed against
/// `analytic_p22 == 0` rather than implicitly assuming it.
fn analytic_p22(lambda: f64, lambda_t: f64, mu: f64, lambda_lame: f64) -> f64 {
    let j = lambda * lambda_t * lambda_t;
    mu.mul_add(lambda_t - 1.0 / lambda_t, lambda_lame * j.ln() / lambda_t)
}

/// Construct `F = diag(λ, λ_t, λ_t)` for the constitutive eval.
fn deformation_gradient(lambda: f64, lambda_t: f64) -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(lambda, lambda_t, lambda_t))
}

/// `max |σᵢ − 1|` over the singular values of `F = diag(λ, λ_t, λ_t)`.
/// Diagonal F has SVs `(|λ|, |λ_t|, |λ_t|)` directly — no SVD needed.
/// Computed analytically rather than via `nalgebra` SVD to keep the
/// example direct-eval through-and-through; matches what the solver's
/// `check_validity_at_step_start` would compute on an identical F.
fn max_stretch_deviation(lambda: f64, lambda_t: f64) -> f64 {
    (lambda - 1.0).abs().max((lambda_t - 1.0).abs())
}

// =============================================================================
// SweepRecord — per-point computed state (verify_* + JSON consumers).
// =============================================================================

#[derive(Debug, Clone)]
struct SweepRecord {
    lambda: f64,
    lambda_t: f64,
    j: f64,
    max_stretch_deviation: f64,
    newton_iter_count: usize,
    newton_residual: f64,
    p_11_observed: f64,
    p_11_analytic: f64,
    p_22_observed: f64,
    p_22_analytic: f64,
    p_33_observed: f64,
    psi_observed: f64,
    psi_analytic: f64,
}

fn run_sweep(mat: &NeoHookean) -> Result<Vec<SweepRecord>> {
    let mut records = Vec::with_capacity(N_SWEEP);
    for &lambda in &SWEEP_LAMBDAS {
        let (lambda_t, iter, residual) = solve_lambda_t(lambda, MU_PA, LAMBDA_PA)
            .with_context(|| format!("inner Newton failed at λ = {lambda}"))?;
        let f = deformation_gradient(lambda, lambda_t);
        let p_observed = mat.first_piola(&f);
        let psi_observed = mat.energy(&f);
        records.push(SweepRecord {
            lambda,
            lambda_t,
            j: lambda * lambda_t * lambda_t,
            max_stretch_deviation: max_stretch_deviation(lambda, lambda_t),
            newton_iter_count: iter,
            newton_residual: residual,
            p_11_observed: p_observed[(0, 0)],
            p_11_analytic: analytic_p11(lambda, lambda_t, MU_PA, LAMBDA_PA),
            p_22_observed: p_observed[(1, 1)],
            p_22_analytic: analytic_p22(lambda, lambda_t, MU_PA, LAMBDA_PA),
            p_33_observed: p_observed[(2, 2)],
            psi_observed,
            psi_analytic: analytic_psi(lambda, lambda_t, MU_PA, LAMBDA_PA),
        });
    }
    Ok(records)
}

// =============================================================================
// Anchor group constants
// =============================================================================

/// Closed-form-vs-observed relative tolerance. Both helpers and `mat`
/// methods compute the same expression in scalar f64; observed gap is
/// at the few-ULP level (`~1e-15`). `1e-12` admits expression-tree
/// reordering noise without flapping.
const REL_TOL: f64 = 1.0e-12;

/// Absolute floor (Pa) for relative comparisons that touch zero
/// (rest config + traction-free `P_22`). `1e-8` Pa is 13 orders below
/// typical NH stress magnitudes.
const EPS_ABS_PA: f64 = 1.0e-8;

/// Validity-domain declaration `max_stretch_deviation` value NH
/// publishes per `sim/L0/soft/src/material/neo_hookean.rs:85`. Pinned
/// exact via `to_bits()` — it's a `const` `1.0_f64`, must be bit-equal.
const NH_VALIDITY_MAX_STRETCH_DEVIATION: f64 = 1.0;

/// Strict in-domain bound on `max |σᵢ − 1|` across the sweep. The
/// solver's `check_validity_at_step_start` panics on `>` the
/// declaration; staying strictly below by the slack here demonstrates
/// the sweep sits inside the boundary, not on it.
const SWEEP_IN_DOMAIN_BOUND: f64 = NH_VALIDITY_MAX_STRETCH_DEVIATION;

// =============================================================================
// verify_validity_domain_declaration
// =============================================================================

/// `mat.validity()` publishes the declared boundary semantics. Two
/// fields exact-pinned: `max_stretch_deviation` is the slot the sweep
/// must stay below (anchor `verify_sweep_in_domain` uses it as the
/// bound); `inversion = RequireOrientation` is the policy enforced
/// upstream by IPC barrier in production scenes (here, by staying
/// `λ > 0` throughout).
fn verify_validity_domain_declaration(mat: &NeoHookean) {
    let validity = mat.validity();
    assert_eq!(
        validity.max_stretch_deviation.to_bits(),
        NH_VALIDITY_MAX_STRETCH_DEVIATION.to_bits(),
        "NH validity declaration drift: max_stretch_deviation = \
         {} (bits {:#018x}), expected {NH_VALIDITY_MAX_STRETCH_DEVIATION} \
         (bits {:#018x}). Sweep margin calculations + the \
         `SWEEP_IN_DOMAIN_BOUND` constant assume `1.0`.",
        validity.max_stretch_deviation,
        validity.max_stretch_deviation.to_bits(),
        NH_VALIDITY_MAX_STRETCH_DEVIATION.to_bits(),
    );
    assert!(
        matches!(validity.inversion, InversionHandling::RequireOrientation),
        "NH inversion handler drift: got {:?}, expected RequireOrientation",
        validity.inversion,
    );
}

// =============================================================================
// verify_inner_newton_convergence
// =============================================================================

/// Per-point convergence: residual within scaled bound, iter count
/// strictly below budget, λ_t inside `(0, 2)`. Plus a sign-consistency
/// gate (compressive λ → λ_t > 1, tensile λ → λ_t < 1, rest → λ_t = 1
/// exact via the IEEE-754 `f(1) = 0` invariant).
fn verify_inner_newton_convergence(records: &[SweepRecord]) {
    let residual_bound = LAMBDA_PA * NEWTON_RESIDUAL_REL_TO_LAMBDA;
    for rec in records {
        assert!(
            rec.newton_residual.abs() <= residual_bound,
            "λ = {}: inner Newton residual {:e} exceeds bound {residual_bound:e} \
             (Λ · {NEWTON_RESIDUAL_REL_TO_LAMBDA:e})",
            rec.lambda,
            rec.newton_residual,
        );
        assert!(
            rec.newton_iter_count < LAMBDA_T_NEWTON_MAX_ITER,
            "λ = {}: Newton iter_count {} exhausted budget {LAMBDA_T_NEWTON_MAX_ITER}",
            rec.lambda,
            rec.newton_iter_count,
        );
        assert!(
            (0.0..2.0).contains(&rec.lambda_t),
            "λ = {}: λ_t = {} outside (0, 2)",
            rec.lambda,
            rec.lambda_t,
        );
        // λ values are sweep constants (no NaN possible); use to_bits
        // comparison for the rest-config branch since `==` on f64 trips
        // `clippy::float_cmp`.
        if rec.lambda.to_bits() == 1.0_f64.to_bits() {
            assert_eq!(
                rec.lambda_t.to_bits(),
                1.0_f64.to_bits(),
                "λ = 1 (rest) requires λ_t = 1 exactly via the f(1) = 0 IEEE-754 \
                 invariant; got λ_t = {} (bits {:#018x})",
                rec.lambda_t,
                rec.lambda_t.to_bits(),
            );
        } else if rec.lambda < 1.0 {
            assert!(
                rec.lambda_t > 1.0,
                "λ = {} < 1 (compressive) requires λ_t > 1 (transverse expansion); \
                 got λ_t = {}",
                rec.lambda,
                rec.lambda_t,
            );
        } else {
            assert!(
                rec.lambda_t < 1.0,
                "λ = {} > 1 (tensile) requires λ_t < 1 (transverse contraction); \
                 got λ_t = {}",
                rec.lambda,
                rec.lambda_t,
            );
        }
    }
}

// =============================================================================
// verify_closed_form_p11
// =============================================================================

/// Per-point `P_11` analytic vs observed at relative `1e-12` (or
/// `1e-8` Pa absolute floor for near-zero rest config).
fn verify_closed_form_p11(records: &[SweepRecord]) {
    for rec in records {
        assert_relative_eq!(
            rec.p_11_observed,
            rec.p_11_analytic,
            max_relative = REL_TOL,
            epsilon = EPS_ABS_PA
        );
    }
}

// =============================================================================
// verify_closed_form_p22_zero
// =============================================================================

/// `P_22` is `≡ 0` when λ_t is the Newton solution. Observed bound is
/// the inner-Newton residual scaled to stress units (`|residual| /
/// λ_t`) plus `EPS_ABS_PA`, since `P_22 = (μ residual_term) / λ_t`
/// in shape — Newton residual sits at convergence-tol level and
/// propagates linearly into observed P_22 magnitude.
fn verify_closed_form_p22_zero(records: &[SweepRecord]) {
    for rec in records {
        let bound = (rec.newton_residual / rec.lambda_t).abs() + EPS_ABS_PA;
        assert!(
            rec.p_22_observed.abs() <= bound,
            "λ = {}: observed P_22 = {:e} exceeds Newton-precision bound {bound:e} \
             (analytic_p22 = {:e}; residual = {:e})",
            rec.lambda,
            rec.p_22_observed,
            rec.p_22_analytic,
            rec.newton_residual,
        );
        // Symmetry: P_33 == P_22 within EPS_ABS_PA.
        assert!(
            (rec.p_33_observed - rec.p_22_observed).abs() <= EPS_ABS_PA,
            "λ = {}: P_33 = {:e} should equal P_22 = {:e} by transverse symmetry",
            rec.lambda,
            rec.p_33_observed,
            rec.p_22_observed,
        );
    }
}

// =============================================================================
// verify_closed_form_psi
// =============================================================================

/// Per-point ψ analytic vs observed at relative `1e-12`.
fn verify_closed_form_psi(records: &[SweepRecord]) {
    for rec in records {
        assert_relative_eq!(
            rec.psi_observed,
            rec.psi_analytic,
            max_relative = REL_TOL,
            epsilon = EPS_ABS_PA
        );
    }
}

// =============================================================================
// verify_rest_config_zero
// =============================================================================

/// At `λ = 1` the Newton solution is exactly `λ_t = 1` (`f(1) = 0`
/// trivially), so `F = I` and every NH constitutive output is `0` in
/// IEEE-754 — `try_inverse(I) = I`, `F − F⁻ᵀ = 0`, `ln(det I) = 0`.
/// Pin the `to_bits() == 0u64` invariant directly; any drift here is
/// either a closed-form algebra change OR a regression in NH's
/// `first_piola` / `energy` code paths.
fn verify_rest_config_zero(records: &[SweepRecord]) {
    // SWEEP_LAMBDAS[6] is the rest config (λ = 1.00) by construction;
    // the assert_eq guards against future reordering of the sweep.
    const REST_INDEX: usize = 6;
    let rest = &records[REST_INDEX];
    assert_eq!(
        rest.lambda.to_bits(),
        1.0_f64.to_bits(),
        "SWEEP_LAMBDAS[{REST_INDEX}] = {} drifted from rest config — \
         re-anchor REST_INDEX to the new index of λ = 1.0",
        rest.lambda,
    );
    assert_eq!(
        rest.p_11_observed.to_bits(),
        0_u64,
        "rest-config P_11 drift: got {} (bits {:#018x}), expected 0u64. \
         F = I should give P = 0 exact in IEEE-754.",
        rest.p_11_observed,
        rest.p_11_observed.to_bits(),
    );
    assert_eq!(
        rest.p_22_observed.to_bits(),
        0_u64,
        "rest-config P_22 drift: got {} (bits {:#018x}), expected 0u64",
        rest.p_22_observed,
        rest.p_22_observed.to_bits(),
    );
    assert_eq!(
        rest.psi_observed.to_bits(),
        0_u64,
        "rest-config ψ drift: got {} (bits {:#018x}), expected 0u64. \
         F = I should give ψ = 0 exact via I₁ − 3 = 0 and ln J = 0.",
        rest.psi_observed,
        rest.psi_observed.to_bits(),
    );
}

// =============================================================================
// verify_monotonicity_and_sign
// =============================================================================

/// `P_11` strictly increasing across the sweep (in λ-ascending order),
/// AND `sign(P_11) == sign(λ − 1)` (negative for compressive,
/// zero at rest, positive for tensile).
fn verify_monotonicity_and_sign(records: &[SweepRecord]) {
    for window in records.windows(2) {
        let prev = &window[0];
        let curr = &window[1];
        assert!(
            curr.p_11_observed > prev.p_11_observed,
            "P_11 monotonicity break between λ = {} (P_11 = {:e}) and \
             λ = {} (P_11 = {:e})",
            prev.lambda,
            prev.p_11_observed,
            curr.lambda,
            curr.p_11_observed,
        );
    }
    for rec in records {
        if rec.lambda.to_bits() == 1.0_f64.to_bits() {
            assert_eq!(
                rec.p_11_observed.to_bits(),
                0_u64,
                "λ = 1 (rest) requires P_11 = 0 exact",
            );
        } else if rec.lambda < 1.0 {
            assert!(
                rec.p_11_observed < 0.0,
                "λ = {} (compressive) requires P_11 < 0; got {:e}",
                rec.lambda,
                rec.p_11_observed,
            );
        } else {
            assert!(
                rec.p_11_observed > 0.0,
                "λ = {} (tensile) requires P_11 > 0; got {:e}",
                rec.lambda,
                rec.p_11_observed,
            );
        }
    }
}

// =============================================================================
// verify_sweep_in_domain
// =============================================================================

/// `max |σᵢ − 1| < 1.0` at every sweep point — the sweep stays
/// strictly inside NH's declared `max_stretch_deviation = 1.0`
/// boundary. Strict (not `≤`) confirms we're inside the boundary,
/// not on it.
fn verify_sweep_in_domain(records: &[SweepRecord]) {
    for rec in records {
        assert!(
            rec.max_stretch_deviation < SWEEP_IN_DOMAIN_BOUND,
            "λ = {}: max_stretch_deviation = {} ≥ bound {SWEEP_IN_DOMAIN_BOUND}",
            rec.lambda,
            rec.max_stretch_deviation,
        );
    }
}

// =============================================================================
// Captured bit patterns — 48 deterministic-state pins
// =============================================================================
//
// Captured on 2026-05-05 at sim-soft `dev` HEAD (preceding commit
// `5bab2e62` from row 4 single-tet-stretch), rustc 1.95.0 (`59807616e`
// 2026-04-14) — the same toolchain IV-1 captured at sim-soft
// `c3729d4a` per `invariant_iv_1_uniform_passthrough.rs:138-151`
// — on macOS arm64.
//
// IEEE-754 determinism: NH `first_piola` and `energy` do pure
// `Matrix3<f64>` arithmetic with one `f64::ln` per call; the inner
// Newton for `λ_t` does standard `f64` arithmetic with `f64::ln` per
// iteration. On a fixed `(rustc, libm)` toolchain every bit is
// reproducible. The IV-1 two-tier contract applies: this dense
// constitutive-eval path is bit-equal across rustc minor versions
// AND across `(macOS arm64, Linux x86_64)` SIMD architectures.
//
// **Failure-mode protocol** (mirrors IV-1's): if any of these asserts
// fails, do NOT re-bake. Diagnose in this order:
//   1. Rule out toolchain drift (rustc / LLVM / libm minor version
//      delta vs the rustc 1.95.0 capture).
//   2. If same toolchain, real regression — identify which sim-soft
//      commit altered the NH constitutive numerics OR the inner-Newton
//      convergence path. The inner Newton is in this file; NH itself
//      is in `sim/L0/soft/src/material/neo_hookean.rs`.
//   3. NEVER re-bake the reference values to make the test green.
//      Spurious re-capture hollows the contract to a tautology.
//
// Layout: 4 parallel `[u64; N_SWEEP]` arrays in λ-ascending order
// matching `SWEEP_LAMBDAS`. Per-row comment names the source λ for
// failure-message clarity.

/// Per-sweep-point `mat.first_piola(F)[(0,0)].to_bits()`.
const SWEEP_P_11_BITS: [u64; N_SWEEP] = [
    0xc141_df2c_250f_fda9, // λ = 0.15
    0xc136_b98f_7f2a_5901, // λ = 0.20
    0xc127_2c02_5d9c_8bba, // λ = 0.30
    0xc111_84dc_d660_669f, // λ = 0.50
    0xc0fc_ea3c_eec7_c9eb, // λ = 0.70
    0xc0e7_cdf8_5ace_7a56, // λ = 0.85
    0x0000_0000_0000_0000, // λ = 1.00 (rest)
    0x40e7_7fd1_ffed_79a3, // λ = 1.20
    0x40f8_f8a7_435c_7d19, // λ = 1.50
    0x4100_fcad_a716_252f, // λ = 1.75
    0x4103_70f9_2cd9_304b, // λ = 1.90
    0x4104_3adb_4fdd_2f2e, // λ = 1.95
];

/// Per-sweep-point `mat.first_piola(F)[(1,1)].to_bits()`. By the
/// traction-free Newton construction, all values are within the
/// `(residual / λ_t).abs() + EPS_ABS_PA` bound of zero — pinning the
/// observed bits gates not just the constitutive eval but also the
/// inner-Newton precision (a tighter Newton tolerance would shift
/// these toward `0u64`; a looser one would walk them away).
const SWEEP_P_22_BITS: [u64; N_SWEEP] = [
    0x3dc0_0000_0000_0000, // λ = 0.15
    0xbdb0_0000_0000_0000, // λ = 0.20
    0x3db0_0000_0000_0000, // λ = 0.30
    0x3dd2_0000_0000_0000, // λ = 0.50
    0xbdb0_0000_0000_0000, // λ = 0.70
    0xbdb4_0000_0000_0000, // λ = 0.85
    0x0000_0000_0000_0000, // λ = 1.00 (rest)
    0xbdc3_0000_0000_0000, // λ = 1.20
    0x3db8_0000_0000_0000, // λ = 1.50
    0xbdc0_0000_0000_0000, // λ = 1.75
    0x3dcc_0000_0000_0000, // λ = 1.90
    0xbde9_0000_0000_0000, // λ = 1.95
];

/// Per-sweep-point `mat.energy(F).to_bits()`.
const SWEEP_PSI_BITS: [u64; N_SWEEP] = [
    0x4115_463f_f717_f2cc, // λ = 0.15
    0x410f_28cd_f410_6d1e, // λ = 0.20
    0x4102_2c5c_4700_8b27, // λ = 0.30
    0x40ea_5f8f_b5dd_4157, // λ = 0.50
    0x40ce_92df_d853_1bb8, // λ = 0.70
    0x40ab_1cf6_c1f4_220f, // λ = 0.85
    0x0000_0000_0000_0000, // λ = 1.00 (rest)
    0x40b3_b68c_8121_e95e, // λ = 1.20
    0x40db_5311_fa3c_3459, // λ = 1.50
    0x40ec_735c_8181_f642, // λ = 1.75
    0x40f3_b173_9545_2d20, // λ = 1.90
    0x40f5_ad45_5826_8264, // λ = 1.95
];

/// Per-sweep-point inner-Newton-converged `λ_t.to_bits()`. Pinning λ_t
/// gates the inner-Newton determinism — a convergence-criterion or
/// initial-guess change that produced a different `λ_t` (even within
/// rel-tol slack on `P_11`) would surface here as a bit drift.
const SWEEP_LAMBDA_T_BITS: [u64; N_SWEEP] = [
    0x3ffe_1679_d63a_5bf7, // λ = 0.15
    0x3ffb_cc6d_ab59_b73e, // λ = 0.20
    0x3ff8_9ee8_040a_da72, // λ = 0.30
    0x3ff4_c529_d6d9_864b, // λ = 0.50
    0x3ff2_6040_cf2d_7bcb, // λ = 0.70
    0x3ff1_0f5c_9b18_af57, // λ = 0.85
    0x3ff0_0000_0000_0000, // λ = 1.00 (rest — λ_t = 1 exact)
    0x3fed_b7e6_4ebc_8555, // λ = 1.20
    0x3feb_12a9_14ee_89d9, // λ = 1.50
    0x3fe9_57f0_c97e_5de4, // λ = 1.75
    0x3fe8_7451_9844_2910, // λ = 1.90
    0x3fe8_2dab_68c8_5a3c, // λ = 1.95
];

// =============================================================================
// verify_sweep_bit_equal
// =============================================================================

/// 48 bit-pins (`P_11`, `P_22`, ψ, `λ_t` × 12) against the captured
/// reference. Strongest regression net on this path — every constitutive
/// output AND the inner-Newton-converged λ_t are bit-stable on the
/// IV-1 two-tier contract toolchain matrix.
fn verify_sweep_bit_equal(records: &[SweepRecord]) {
    for (i, rec) in records.iter().enumerate() {
        check_bits(i, rec.lambda, "P_11", rec.p_11_observed, SWEEP_P_11_BITS[i]);
        check_bits(i, rec.lambda, "P_22", rec.p_22_observed, SWEEP_P_22_BITS[i]);
        check_bits(i, rec.lambda, "psi", rec.psi_observed, SWEEP_PSI_BITS[i]);
        check_bits(
            i,
            rec.lambda,
            "lambda_t",
            rec.lambda_t,
            SWEEP_LAMBDA_T_BITS[i],
        );
    }
}

fn check_bits(idx: usize, lambda: f64, name: &str, value: f64, expected_bits: u64) {
    let got_bits = value.to_bits();
    assert_eq!(
        got_bits,
        expected_bits,
        "sweep[{idx}] (λ = {lambda}) {name} bit drift: got {got_bits:#018x} \
         ({value:e}), expected {expected_bits:#018x} ({:e}). See the \
         SWEEP_*_BITS comment block for the failure-mode protocol — do \
         NOT re-bake without ruling out toolchain delta and a real \
         regression first.",
        f64::from_bits(expected_bits),
    );
}

// =============================================================================
// JSON emit
// =============================================================================

/// Approximate compressive boundary on `λ` — set by `λ_t = 2`. From
/// `μ λ_t² + Λ ln(λ λ_t²) = μ` at `λ_t = 2`:
/// `ln λ = −3μ/Λ − 2 ln 2 ≈ −2.136`, so `λ ≈ 0.1182`. Documented in
/// JSON for cross-reference; sweep starts at `λ = 0.15`, ~27% margin.
const COMPRESSIVE_BRACKET_LAMBDA_APPROX: f64 = 0.1182;

/// Tensile boundary on `λ` — `λ = 2.0` directly (max_stretch_dev =
/// `|λ − 1| = 1.0` exactly). Sweep ends at `λ = 1.95`, 2.5% margin.
const TENSILE_BRACKET_LAMBDA: f64 = 2.0;

fn save_force_stretch_json(records: &[SweepRecord], mat: &NeoHookean, path: &Path) -> Result<()> {
    let validity = mat.validity();
    // `InversionHandling` is `#[non_exhaustive]`; using `{:?}` keeps the JSON
    // schema future-proof against new variants (e.g. `Barrier`, `OptIn` per
    // Phase H roadmap) without a manual match update each time.
    let inversion_str = format!("{:?}", validity.inversion);
    let sweep: Vec<_> = records
        .iter()
        .map(|rec| {
            let p11_rel_err = (rec.p_11_observed - rec.p_11_analytic).abs()
                / rec.p_11_analytic.abs().max(EPS_ABS_PA);
            let psi_rel_err = (rec.psi_observed - rec.psi_analytic).abs()
                / rec.psi_analytic.abs().max(EPS_ABS_PA);
            let p22_newton_bound = (rec.newton_residual / rec.lambda_t).abs() + EPS_ABS_PA;
            json!({
                "lambda":                       rec.lambda,
                "lambda_t":                     rec.lambda_t,
                "J":                            rec.j,
                "max_stretch_deviation":        rec.max_stretch_deviation,
                "in_domain":                    rec.max_stretch_deviation < SWEEP_IN_DOMAIN_BOUND,
                "lambda_t_newton_iter_count":   rec.newton_iter_count,
                "lambda_t_newton_residual":     rec.newton_residual,
                "P_11": {
                    "analytic":  rec.p_11_analytic,
                    "observed":  rec.p_11_observed,
                    "rel_err":   p11_rel_err,
                },
                "P_22": {
                    "analytic_eq_0_by_construction": true,
                    "observed":                      rec.p_22_observed,
                    "P_33_observed":                 rec.p_33_observed,
                    "newton_precision_bound":        p22_newton_bound,
                    "below_bound":                   rec.p_22_observed.abs() <= p22_newton_bound,
                },
                "psi": {
                    "analytic":  rec.psi_analytic,
                    "observed":  rec.psi_observed,
                    "rel_err":   psi_rel_err,
                },
            })
        })
        .collect();

    let record = json!({
        "scene": {
            "material_uniform": {
                "mu_Pa":           MU_PA,
                "lambda_Pa":       LAMBDA_PA,
                "nu_compressible": 0.4,
            },
            "deformation_gradient_form": "traction_free_diag(lambda, lambda_t, lambda_t)",
            "validity_domain_declaration": {
                "max_stretch_deviation": validity.max_stretch_deviation,
                "inversion":             inversion_str,
            },
            "in_domain_bracket_lambda_approx": [
                COMPRESSIVE_BRACKET_LAMBDA_APPROX,
                TENSILE_BRACKET_LAMBDA,
            ],
        },
        "stretch_sweep": sweep,
    });

    let file = std::fs::File::create(path)
        .with_context(|| format!("failed to create {}", path.display()))?;
    serde_json::to_writer_pretty(&file, &record)
        .with_context(|| format!("failed to serialize record to {}", path.display()))?;
    Ok(())
}

// =============================================================================
// print_summary — museum-plaque stdout
// =============================================================================

fn print_summary(records: &[SweepRecord], path: &Path) {
    println!("==== neo-hookean-uniaxial ====");
    println!();
    println!("input  : NeoHookean::from_lame(μ = {MU_PA:e}, Λ = {LAMBDA_PA:e})  (Pa)");
    println!("         compressible NH at ν ≈ 0.4 (Λ = 4μ — Ecoflex-class)");
    println!("         F = diag(λ, λ_t, λ_t)  (traction-free uniaxial)");
    println!("         λ_t per stretch via 1-D Newton on");
    println!("              f(λ_t) = μ λ_t² + Λ ln(λ λ_t²) − μ = 0");
    println!(
        "         sweep   : {N_SWEEP} points, λ ∈ [{:.2}, {:.2}],  asymmetric \
         density",
        SWEEP_LAMBDAS[0],
        SWEEP_LAMBDAS[N_SWEEP - 1],
    );
    println!(
        "         bracket : in-domain λ ≈ [{COMPRESSIVE_BRACKET_LAMBDA_APPROX:.4}, \
         {TENSILE_BRACKET_LAMBDA:.4}]  (compressive: λ_t = 2; tensile: λ = 2)"
    );
    println!();
    println!("Anchor groups (all assertions exit-0 on success):");
    println!(
        "  validity_domain_declaration : max_stretch_deviation == 1.0; inversion == RequireOrientation"
    );
    println!("  inner_newton_convergence    : per-point residual + iter + λ_t bracket + sign");
    println!("  closed_form_P_11_agreement  : per-point analytic vs observed, rel {REL_TOL:e}");
    println!("  closed_form_P_22_zero       : observed |P_22| within Newton-precision bound");
    println!("  closed_form_psi_agreement   : per-point analytic vs observed, rel {REL_TOL:e}");
    println!("  rest_config_zero            : at λ=1, P_11.bits == P_22.bits == ψ.bits == 0u64");
    println!("  monotonicity_and_sign       : P_11 strictly increasing; sign(P_11) == sign(λ-1)");
    println!("  sweep_in_domain             : max|σ-1| < {SWEEP_IN_DOMAIN_BOUND} at every point");
    println!(
        "  sweep_bit_equal             : 48 bit-pins (P_11, P_22, ψ, λ_t × {N_SWEEP}) — IV-1 contract"
    );
    println!();
    println!(
        "{:>5}  {:>14}  {:>10}  {:>10}  {:>4}  {:>13}  {:>13}  {:>13}  {:>13}",
        "λ", "λ_t", "J", "max|σ-1|", "iter", "P_11 [Pa]", "P_22 [Pa]", "ψ [J/m³]", "rel err P_11"
    );
    for rec in records {
        let rel_err =
            (rec.p_11_observed - rec.p_11_analytic).abs() / rec.p_11_analytic.abs().max(EPS_ABS_PA);
        println!(
            "{:>5}  {:>14.10}  {:>10.6}  {:>10.6}  {:>4}  {:>13.6e}  {:>13.6e}  {:>13.6e}  {:>13.3e}",
            rec.lambda,
            rec.lambda_t,
            rec.j,
            rec.max_stretch_deviation,
            rec.newton_iter_count,
            rec.p_11_observed,
            rec.p_22_observed,
            rec.psi_observed,
            rel_err,
        );
    }
    println!();
    println!("JSON   : {}", path.display());
    println!("         12-record force-stretch curve — λ, λ_t, J, max|σ-1|, P_11/P_22/ψ");
    println!("         (analytic vs observed + rel err) per point. Read with:");
    println!(
        "         jq '.stretch_sweep | map({{lambda, P_11: .P_11.observed}})'  {}",
        path.display()
    );
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    let mat = NeoHookean::from_lame(MU_PA, LAMBDA_PA);

    verify_validity_domain_declaration(&mat);
    let records = run_sweep(&mat)?;
    verify_inner_newton_convergence(&records);
    verify_closed_form_p11(&records);
    verify_closed_form_p22_zero(&records);
    verify_closed_form_psi(&records);
    verify_rest_config_zero(&records);
    verify_monotonicity_and_sign(&records);
    verify_sweep_in_domain(&records);
    verify_sweep_bit_equal(&records);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)
        .with_context(|| format!("failed to create {}", out_dir.display()))?;
    let out_path = out_dir.join("force_stretch.json");
    save_force_stretch_json(&records, &mat, &out_path)?;

    print_summary(&records, &out_path);

    Ok(())
}
