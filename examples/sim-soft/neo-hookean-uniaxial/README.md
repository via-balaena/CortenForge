# neo-hookean-uniaxial

**Direct-evaluation Neo-Hookean constitutive surface vs closed form across a 12-point traction-free uniaxial stretch sweep — the canonical NH stress-strain curve, no FEM, no solver.** `F = diag(λ, λ_t, λ_t)` with `λ_t` chosen per stretch so the transverse Piola stress vanishes (`P_22 = 0`); the resulting `λ_t(λ)` is the Newton solution to the scalar transcendental `μ λ_t² + Λ ln(λ · λ_t²) = μ`. Every claim sits behind an `assert!` / `assert_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m], a clean `cargo run --release` exit-0 IS the correctness signal. Output is `out/force_stretch.json` (12-record force-stretch curve, no spatial artifact, no `cf-view` rendering — single-tet has trivial topology). Optional `out/force_stretch.png` post-hoc visualization via `uv run plot.py` per inventory Q4 ("matplotlib post-hoc, not in viewer scope").

[m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md

## What this example demonstrates

The canonical Neo-Hookean stress-strain benchmark, evaluated directly against `Material::first_piola` and `Material::energy`:

```rust
let mat = NeoHookean::from_lame(1.0e5, 4.0e5);                     // μ, Λ — Ecoflex-class, ν ≈ 0.4
let lambda_t = solve_lambda_t(lambda, MU_PA, LAMBDA_PA)?.0;        // 1-D Newton, traction-free
let f = Matrix3::from_diagonal(&Vector3::new(lambda, lambda_t, lambda_t));
let p = mat.first_piola(&f);                                       // P_11 vs analytic closed form
let psi = mat.energy(&f);                                          // ψ vs analytic closed form
```

For compressible NH with shear modulus `μ` and first Lamé parameter `Λ`, the closed forms are:

```text
I_1   = λ² + 2 λ_t²
J     = λ · λ_t²
ψ     = (μ/2)(I_1 − 3) + (Λ/2)(ln J)² − μ ln J
P_11  = μ(λ   − 1/λ  ) + Λ ln(J) / λ
P_22  = μ(λ_t − 1/λ_t) + Λ ln(J) / λ_t       ≡ 0 by traction-free construction
```

The transverse `λ_t` is determined by `P_22(λ, λ_t) = 0`, which reduces to the scalar transcendental `f(λ_t) = μ λ_t² + Λ ln(λ · λ_t²) − μ = 0`. `f` is strictly monotone in `λ_t > 0` (`f'(λ_t) = 2μ λ_t + 2Λ/λ_t > 0`), so 1-D Newton from `λ_t = 1` converges quadratically in ≤ 6 iter across the full `λ ∈ [0.15, 1.95]` sweep bracket — no nested solver, just an arcsin-shape inversion of a known monotonic function.

This is **row 5 of the sim-soft examples arc** ([`EXAMPLE_INVENTORY.md`][inv]) — the constitutive-surface counterpart to row 4 (`single-tet-stretch`, which is `Solver::step` end-to-end on the same canonical scene). Row 4 exercises the FEM machinery; row 5 exercises the constitutive law directly.

[inv]: ../../../sim/L0/soft/EXAMPLE_INVENTORY.md

**Why traction-free `F = diag(λ, λ_t, λ_t)`** (rather than constrained `F = diag(λ, 1, 1)`): the traction-free curve is the canonical NH stretch benchmark in mechanics literature — every textbook plots `P_11(λ)` under traction-free transverse. The 1-D Newton for `λ_t` is the price of getting the canonical curve; the alternative (constrained transverse) gives a clean closed form but a curve nobody recognizes. The asymmetric in-domain bracket `λ ∈ [≈ 0.118, 2.0]` (compressive limit set by `λ_t → 2`, tensile limit by `λ → 2`) also reveals real physics: the compressive boundary is set by transverse blow-up, not axial collapse.

**Why JSON-only (no cf-view)**: per inventory Q4 row 5, the curve IS the artifact. Single-tet has trivial geometry; rendering one tet adds visual noise. The 12-record force-stretch JSON + the optional matplotlib plot is the inspection signal.

**Why direct-eval (no solver, no Newton iteration on the FEM)**: the row's subject is the constitutive law, not the FEM assembly. Row 4 covers the full `Solver::step` path with bit-equal `x_final` on the same scene. Row 5 strips that down to the constitutive surface, which is what `material_fd.rs` (gradcheck) exercises in test form — this row is the user-facing demo of that surface.

## Numerical anchors

Each anchor is encoded as an `assert!` / `assert_eq!` in `src/main.rs` under `verify_*`. All 9 groups are called from `main()` in dependency order; cargo `--release` exit-0 means every assert passed.

### 1. `validity_domain_declaration`

| Anchor | Bound |
|---|---|
| `mat.validity().max_stretch_deviation` | exact-pin to `1.0_f64` (`to_bits` equality) |
| `mat.validity().inversion` | `matches!(_, InversionHandling::RequireOrientation)` |

NH publishes its `ValidityDomain` declaration at `sim/L0/soft/src/material/neo_hookean.rs:83-92`. The sweep margin calculations + the `SWEEP_IN_DOMAIN_BOUND` constant assume `max_stretch_deviation = 1.0`; pinning the bit catches any decoration / wrapping that would change the published value.

### 2. `inner_newton_convergence`

| Anchor | Bound |
|---|---|
| `\|f(λ_t)\|` at convergence | `≤ Λ · 1e-14` |
| `iter_count` | `< LAMBDA_T_NEWTON_MAX_ITER (= 30)` |
| `λ_t` bracket | `(0, 2)` strict |
| `sign(λ_t − 1)` | matches `sign(1 − λ)` (compressive expands, tensile contracts) |
| `λ_t` at λ=1 | `to_bits == 1.0_f64.to_bits()` exact |

The IEEE-754 invariant `f(1) = 0` forces λ_t = 1 exactly at the rest config (1 iter, residual exactly zero). Other points converge in 5 iter from `λ_t = 1` initial guess.

### 3. `closed_form_P_11_agreement`

| Anchor | Bound |
|---|---|
| Per-point `\|P_11_observed − P_11_analytic\| / max(\|analytic\|, EPS_ABS_PA)` | `< 1e-12` |

`EPS_ABS_PA = 1e-8` Pa absolute floor for the rest-config near-zero comparison. Observed agreement is at ULP level (most points match exactly bit-for-bit; a few sit at 1 ULP).

### 4. `closed_form_P_22_zero`

| Anchor | Bound |
|---|---|
| Per-point `\|P_22_observed\|` | `≤ \|residual / λ_t\| + EPS_ABS_PA` |
| Per-point `\|P_33_observed − P_22_observed\|` | `≤ EPS_ABS_PA` (transverse symmetry) |

`P_22 ≡ 0` by traction-free Newton construction. Observed `P_22` is bounded by the inner-Newton residual scaled to stress units — a tighter Newton tolerance shifts these toward exact zero, a looser one walks them away. Pinning the bound gates the inner-Newton precision indirectly.

### 5. `closed_form_psi_agreement`

| Anchor | Bound |
|---|---|
| Per-point `\|ψ_observed − ψ_analytic\| / max(\|analytic\|, EPS_ABS_PA)` | `< 1e-12` |

ψ has more cancellation noise than P_11 near rest (the `(μ/2)(I_1 − 3)` and `(Λ/2)(ln J)²` terms can have differing signs and similar magnitudes), but observed agreement still sits at `~1e-14` worst-case — well inside the bound.

### 6. `rest_config_zero`

| Anchor | Bound |
|---|---|
| At λ = 1: `P_11.to_bits()` | exact-pin `0u64` |
| At λ = 1: `P_22.to_bits()` | exact-pin `0u64` |
| At λ = 1: `ψ.to_bits()` | exact-pin `0u64` |

At rest, `λ_t = 1` exactly, so `F = I` exactly. NH's `first_piola(I)` evaluates `μ(I − I⁻ᵀ) + Λ ln(1) I⁻ᵀ = 0 · I = 0` in IEEE-754 exact; `energy(I)` evaluates `(μ/2)(3 − 3) + 0 − 0 = 0` exact. The bit pins catch any closed-form algebra change OR a regression in NH's `first_piola` / `energy` paths that would induce non-zero round-off at F = I.

### 7. `monotonicity_and_sign`

| Anchor | Bound |
|---|---|
| Across sweep, `P_11(λ_{i+1}) > P_11(λ_i)` | strict (P_11 strictly increasing in λ) |
| For each λ, `sign(P_11)` | matches `sign(λ − 1)` |

A material whose stress isn't monotone in stretch is unstable. P_11 strictly increasing across the sweep is the constitutive-stability gate.

### 8. `sweep_in_domain`

| Anchor | Bound |
|---|---|
| Per-point `max\|σᵢ − 1\| = max(\|λ−1\|, \|λ_t−1\|)` | `< SWEEP_IN_DOMAIN_BOUND (= 1.0)` |

Strict `<` (not `≤`) confirms the sweep stays inside NH's declared `ValidityDomain` boundary, not on it. The compressive boundary at `λ ≈ 0.118` (where `λ_t → 2`) and the tensile boundary at `λ = 2.0` (where `λ → 2`) sit just outside the sweep bracket — `λ ∈ [0.15, 1.95]` lives at ~27% margin from the compressive edge and ~2.5% from the tensile edge, asymmetrically per the bracket structure.

### 9. `sweep_bit_equal`

| Anchor | Bound |
|---|---|
| Per-point `P_11.to_bits()` | exact-pin to `SWEEP_P_11_BITS[i]` |
| Per-point `P_22.to_bits()` | exact-pin to `SWEEP_P_22_BITS[i]` |
| Per-point `ψ.to_bits()` | exact-pin to `SWEEP_PSI_BITS[i]` |
| Per-point `λ_t.to_bits()` | exact-pin to `SWEEP_LAMBDA_T_BITS[i]` |

48 deterministic-state pins (4 quantities × 12 points). The strongest regression net available on this path. Pinning λ_t alongside the constitutive outputs gates the inner-Newton convergence path itself: a convergence-criterion or initial-guess change that produced a different `λ_t` (even within rel-tol slack on `P_11`) would surface here as a bit drift.

**Capture provenance** (per the comment block on `SWEEP_*_BITS` in `src/main.rs`): captured on 2026-05-05 at sim-soft `dev` HEAD (preceding commit `5bab2e62` from row 4 single-tet-stretch), rustc 1.95.0 (`59807616e` 2026-04-14) — the same toolchain IV-1 captured at sim-soft `c3729d4a` per [`invariant_iv_1_uniform_passthrough.rs:138-151`][iv1] — on macOS arm64.

[iv1]: ../../../sim/L0/soft/tests/invariant_iv_1_uniform_passthrough.rs

**IV-1 two-tier contract applies.** NH `first_piola` / `energy` and the inner Newton for `λ_t` do pure scalar `f64` arithmetic with `f64::ln` (libm); on a fixed `(rustc, libm)` toolchain every bit is reproducible. Bit-equal across rustc minor versions AND across `(macOS arm64, Linux x86_64)` SIMD architectures.

**Failure-mode protocol** (mirrors IV-1's): if any bit-pin fails, do NOT re-bake. Diagnose in this order:

1. Rule out toolchain drift (rustc / LLVM / libm minor version delta vs the rustc 1.95.0 capture).
2. If same toolchain, real regression — identify which sim-soft commit altered the NH constitutive numerics OR the inner-Newton convergence path.
3. NEVER re-bake the reference values to make the test green. Spurious re-capture hollows the contract to a tautology.

## Visuals

`out/force_stretch.json` (12-record schema) is the canonical inspection artifact. Per-record fields:

```text
{ "lambda":                       0.15,
  "lambda_t":                     1.8804872864036477,
  "J":                            0.5304348651488632,
  "max_stretch_deviation":        0.8804872864036477,
  "in_domain":                    true,
  "lambda_t_newton_iter_count":   5,
  "lambda_t_newton_residual":     8.731e-11,
  "P_11": { "analytic": -2342488.29, "observed": -2342488.29, "rel_err": 0.0 },
  "P_22": { "analytic_eq_0_by_construction": true, "observed": 2.91e-11,
            "P_33_observed": 2.91e-11, "newton_precision_bound": 1.00e-8,
            "below_bound": true },
  "psi":  { "analytic": 348559.99,  "observed": 348559.99,  "rel_err": 0.0 } }
```

Read directly or via `jq`:

```text
jq '.scene'                                                                      out/force_stretch.json
jq '.stretch_sweep | map({lambda, P_11: .P_11.observed, psi: .psi.observed})'    out/force_stretch.json
jq '.stretch_sweep[] | select(.lambda == 1.0)'                                   out/force_stretch.json
```

Stdout's museum-plaque summary covers the same numbers in human-readable form (input fixture, all 9 anchor-group names, per-point sweep table).

### Optional post-hoc plot (`plot.py`)

A 2x2 matplotlib panel renders `P_11(λ)`, `ψ(λ)`, `λ_t(λ)`, and `J(λ)`:

```text
uv run plot.py
```

PEP 723 inline `dependencies` block declares matplotlib + numpy; `uv run` installs them on first run and reuses the cached env afterward. Output is `out/force_stretch.png` (also opens an interactive window when a display is available). `J(λ)` is the panel that explains the energy-bowl asymmetry in `ψ(λ)`: J ranges from 0.53 (47% volumetric compression at λ=0.15) to 1.11 (11% expansion at λ=1.95), so `(Λ/2)(ln J)²` blows up far more on the compressive side. The other three panels report stress, energy, and transverse stretch directly. Pink shaded regions on each panel mark the out-of-domain bracket `λ < 0.118` (compressive boundary, `λ_t → 2`) and `λ > 2.0` (tensile boundary, `max|σ−1| → 1`).

This is the first sim-soft example with a `plot.py` — the pattern (PEP 723 + `uv run` + reads `out/*.json`) is reusable for future JSON-curve rows (6 `multi-element-stretch`, 18 `contact-force-readout`).

## Run

```text
cargo run -p example-sim-soft-neo-hookean-uniaxial --release
uv run plot.py    # optional post-hoc visualization
```

Output: `out/force_stretch.json` (12-record schema above) and optionally `out/force_stretch.png`. Stdout prints input fixture summary, all 9 anchor-group names, and the per-point sweep table (λ, λ_t, J, max|σ-1|, iter, P_11, P_22, ψ, rel err P_11).

Per [`feedback_release_mode_heavy_tests`][rel], always `--release` for the example. The IV-1 bit-pin contract was captured under release-mode `cargo test --release -p sim-soft`; matching that invocation shape removes one variable from the bit-equality contract. Debug builds may also fire `debug_assert!`s inside `nalgebra` that release elides, which can change failure modes on edge cases (NaN, near-zero divisors) even when the happy-path arithmetic is identical.

[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md

## Cross-references

- **Sister sim-soft examples**: `single-tet-stretch` (row 4 — same canonical Ecoflex-class scene, but full `Solver::step` end-to-end with backward-Euler Newton; companion to this row's direct-eval form), `sdf/stress-test` modules `sphere_eval` / `hollow_shell` / `sdf_to_tet` (rows 1-3 — SDF + meshing surface, prior to FEM-running rows). See `sim/L0/soft/EXAMPLE_INVENTORY.md` Tier 1 + Tier 2.
- **`NeoHookean` impl**: `sim/L0/soft/src/material/neo_hookean.rs` — the closed forms this example checks (`first_piola`, `energy`) and the `validity()` declaration the `validity_domain_declaration` anchor reads.
- **`Material` trait + `ValidityDomain`**: `sim/L0/soft/src/material/mod.rs:19-91`.
- **Companion gradcheck test**: `sim/L0/soft/tests/material_fd.rs` — FD vs analytic check on `first_piola` and `tangent` at one nontrivial F. This row is the user-facing demo of the same constitutive surface that test exercises in unit form, scaled to a full sweep with closed-form comparison instead of FD.
- **IV-1 bit-equal contract**: `sim/L0/soft/tests/invariant_iv_1_uniform_passthrough.rs` — the two-tier contract this row's `sweep_bit_equal` anchor inherits, captured on the same rustc 1.95.0 toolchain.
- **Validity-domain solver enforcement**: `sim/L0/soft/src/solver/backward_euler.rs:431-523` — `check_validity_at_step_start` panics with structured slot-name messages when sweep stretches violate the boundary at solver-step time. NOT exercised by this row (constitutive surface only); the IV-7 follow-on (`tests/multi_material_validity.rs`) covers the solver-side panic semantics for stretch-deviation; the inversion-via-vertex-permutation case remains a flagged follow-on per IV-7's docstring.
- **Book references**: Part 2 Ch 04 [00-energy.md](../../../docs/studies/soft_body_architecture/src/20-materials/04-hyperelastic/00-neo-hookean/00-energy.md) (closed-form ψ derivation) + [01-tangent.md](../../../docs/studies/soft_body_architecture/src/20-materials/04-hyperelastic/00-neo-hookean/01-tangent.md) (`P` and `C` derivations); Part 2 Ch 00 [02-validity.md](../../../docs/studies/soft_body_architecture/src/20-materials/00-trait-hierarchy/02-validity.md) (`ValidityDomain` semantics).
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
  [`feedback_one_at_a_time_review`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_one_at_a_time_review.md),
  [`feedback_thorough_review_before_commit`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_thorough_review_before_commit.md),
  [`feedback_visual_pass_collapses_for_json_rows`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visual_pass_collapses_for_json_rows.md).
