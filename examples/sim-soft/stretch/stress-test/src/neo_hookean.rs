//! neo-hookean — traction-free uniaxial force-stretch curve (demonstration).
//!
//! Sweeps the compressible Neo-Hookean law across a 12-point uniaxial
//! stretch and emits the force-stretch curve as JSON — "the curve IS the
//! artifact" (inventory Q4 row 5: JSON-only, no `cf-view`; single-tet has
//! trivial geometry). Companion `plot.py` (PEP 723 `uv run`) renders it.
//! Canonical NH benchmark per Part 2 Ch 04 [00-energy.md][e] + [01-tangent.md][t].
//!
//! **Constitutive correctness lives in the library.** The closed-form
//! agreement of `NeoHookean::first_piola` / `NeoHookean::energy` with the
//! analytic Neo-Hookean stress and energy is owned by `sim-soft`'s own lib
//! tests (`sim/L0/soft/src/material/neo_hookean.rs`): closed-form value
//! checks at `F = diag(s, 1, 1)` (uniaxial) and `F = diag(a, b, b)`
//! (general transverse, `b ≠ 1`) at rel `1e-12`, plus rest-config zero.
//! This module is a DEMONSTRATION: it drives the real material across a
//! physically-meaningful sweep, self-gates on demonstration-integrity and
//! curve-shape properties, and overlays the analytic closed form in the
//! JSON/plot purely for visual comparison — not as a correctness gate.
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
//! # Demonstration gates (all self-gate; exit-101 on mismatch)
//!
//! - **Validity declaration** — `mat.validity()` inversion policy is
//!   `RequireOrientation`; `max_stretch_deviation` is finite and positive
//!   (it IS the in-domain bound the sweep must stay below).
//! - **Inner Newton convergence** — per-point residual + iter + λ_t bracket + sign.
//! - **Traction-free** — observed `P_22`, `P_33` within Newton precision of `0`.
//! - **Monotonicity and sign** — `P_11` strictly increasing; sign matches `λ − 1`.
//! - **Sweep in-domain** — `max |σᵢ − 1|` strictly below the declared bound.
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
            p_33_observed: p_observed[(2, 2)],
            psi_observed,
            psi_analytic: analytic_psi(lambda, lambda_t, MU_PA, LAMBDA_PA),
        });
    }
    Ok(records)
}

// =============================================================================
// Demonstration constants
// =============================================================================

/// Absolute floor (Pa) for comparisons that touch zero (traction-free
/// `P_22` / `P_33`, rel-err denominators). `1e-8` Pa is 13 orders below
/// typical NH stress magnitudes.
const EPS_ABS_PA: f64 = 1.0e-8;

// =============================================================================
// in_domain_bound — the declared boundary the sweep must stay below
// =============================================================================

/// The material's declared `max_stretch_deviation` — the strict bound
/// the solver's `check_validity_at_step_start` enforces on `max |σᵢ − 1|`.
/// Read from the real `mat.validity()` (not hard-coded) and reused as the
/// in-domain bound, and gates the inversion policy the demonstration
/// relies on (`RequireOrientation`, satisfied by staying `λ > 0`).
fn in_domain_bound(mat: &NeoHookean) -> f64 {
    let validity = mat.validity();
    assert!(
        matches!(validity.inversion, InversionHandling::RequireOrientation),
        "NH inversion handler drift: got {:?}, expected RequireOrientation",
        validity.inversion,
    );
    let bound = validity.max_stretch_deviation;
    assert!(
        bound.is_finite() && bound > 0.0,
        "NH validity declaration drift: max_stretch_deviation = {bound} is not \
         a finite positive bound; the in-domain sweep check needs a usable bound",
    );
    bound
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
// verify_traction_free
// =============================================================================

/// The demonstration's defining property, read from the REAL
/// `mat.first_piola`: at the Newton-solved `λ_t` both transverse
/// components `P_22` and `P_33` vanish. Bound = the inner-Newton
/// residual scaled to stress units (`|residual| / λ_t`, since
/// `P_22 = μ·residual_term / λ_t` in shape) plus `EPS_ABS_PA`. This
/// gates the traction-free CONSTRUCTION, not the constitutive law
/// (whose closed-form correctness the sim-soft lib tests own).
fn verify_traction_free(records: &[SweepRecord]) {
    for rec in records {
        let bound = (rec.newton_residual / rec.lambda_t).abs() + EPS_ABS_PA;
        assert!(
            rec.p_22_observed.abs() <= bound,
            "λ = {}: observed P_22 = {:e} exceeds Newton-precision bound {bound:e} \
             (residual = {:e})",
            rec.lambda,
            rec.p_22_observed,
            rec.newton_residual,
        );
        assert!(
            rec.p_33_observed.abs() <= bound,
            "λ = {}: observed P_33 = {:e} exceeds Newton-precision bound {bound:e} \
             (transverse symmetry — should vanish like P_22)",
            rec.lambda,
            rec.p_33_observed,
        );
    }
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
            assert!(
                rec.p_11_observed.abs() <= EPS_ABS_PA,
                "λ = 1 (rest) requires P_11 ≈ 0; got {:e}",
                rec.p_11_observed,
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

/// `max |σᵢ − 1|` strictly below the declared `max_stretch_deviation`
/// bound at every sweep point — the sweep stays strictly inside NH's
/// declared boundary. Strict (not `≤`) confirms we're inside it, not on
/// it. `bound` is read from the real `mat.validity()` (see
/// [`in_domain_bound`]).
fn verify_sweep_in_domain(records: &[SweepRecord], bound: f64) {
    for rec in records {
        assert!(
            rec.max_stretch_deviation < bound,
            "λ = {}: max_stretch_deviation = {} ≥ bound {bound}",
            rec.lambda,
            rec.max_stretch_deviation,
        );
    }
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
    let in_domain_bound = validity.max_stretch_deviation;
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
                "in_domain":                    rec.max_stretch_deviation < in_domain_bound,
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

fn print_summary(records: &[SweepRecord], bound: f64, path: &Path) {
    println!("==== neo_hookean ====");
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
    println!("Constitutive closed-form correctness (first_piola / energy vs analytic)");
    println!("is owned by sim-soft's neo_hookean.rs lib tests. Demonstration gates:");
    println!(
        "  validity_declaration     : inversion == RequireOrientation; bound finite + positive"
    );
    println!("  inner_newton_convergence : per-point residual + iter + λ_t bracket + sign");
    println!("  traction_free            : observed |P_22|, |P_33| within Newton-precision bound");
    println!("  monotonicity_and_sign    : P_11 strictly increasing; sign(P_11) == sign(λ-1)");
    println!("  sweep_in_domain          : max|σ-1| < {bound} (declared bound) at every point");
    println!();
    println!("(rel err P_11 column = observed vs analytic overlay — visual only, not a gate)");
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

pub fn run() -> Result<()> {
    let mat = NeoHookean::from_lame(MU_PA, LAMBDA_PA);

    let bound = in_domain_bound(&mat);
    let records = run_sweep(&mat)?;
    verify_inner_newton_convergence(&records);
    verify_traction_free(&records);
    verify_monotonicity_and_sign(&records);
    verify_sweep_in_domain(&records, bound);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("out")
        .join("neo_hookean");
    std::fs::create_dir_all(&out_dir)
        .with_context(|| format!("failed to create {}", out_dir.display()))?;
    let out_path = out_dir.join("force_stretch.json");
    save_force_stretch_json(&records, &mat, &out_path)?;

    print_summary(&records, bound, &out_path);

    Ok(())
}
