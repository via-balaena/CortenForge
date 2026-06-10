# M1 — Silicone uniaxial measured-accuracy — RECON

*Active recon, opened 2026-06-09. Head-engineer-owned slicing; user picked the macro
direction (Layer-1 soft-FEM fidelity) and the first target (silicone, uniaxial).*

> **Validate that `sim-soft`'s hyperelastic material response reproduces a *measured*
> uniaxial true-stress–stretch curve for one silicone (Ecoflex 00-30 — switched from
> Dragon Skin 30 in S0 because only Ecoflex 00-10/00-30 carry real data in the Zenodo
> snapshot; the pre-S0 planning sections below still reference Dragon Skin 30), over the
> device-relevant stretch window, within a stated tolerance — and quantify the gap vs the
> current one-point datasheet params.** Oracle = Marechal et al. 2021 (ASTM D412, Zenodo
> `10.5281/zenodo.3611329`).

---

## 1. Empirical baseline (what's true in the code today)

- **Material:** `sim_soft::material::Yeoh` — compressible **2-parameter** Yeoh
  (`yeoh.rs`):
  - `ψ(F) = C₁(I₁−3) + C₂(I₁−3)² − μ ln J + (λ/2)(ln J)²`, with `C₁ = μ/2`.
  - Closed-form `P(F) = μ(F − F⁻ᵀ) + λ(ln J)F⁻ᵀ + 4 C₂(I₁−3)F` and analytic tangent.
  - At `C₂ = 0` reduces bit-exactly to `NeoHookean` (D2 contract,
    `tests/yeoh_contract.rs`).
- **Params:** `material/silicone_table.rs` — every silicone's `(μ, λ, C₂)` derived from a
  **single TDS point**, the 100 % modulus `σ₁₀₀`, via small-strain identities
  (`μ = σ₁₀₀/3`, `λ = 4μ` at ν=0.40, `C₂ = (σ₁₀₀/3.5 − C₁)/4`). Self-flagged: *"~1.7×
  catalog-value uncertainty"*, ν=0.40 locking compromise, *"relative-comparison tool, not
  an absolute predictor."* Dragon Skin 30A row: **μ = 198 kPa, λ = 792 kPa, C₂ = 17.6 kPa,
  λ_break = 4.64** → tensile validity cap `max_principal_stretch = 0.8·λ_break ≈ 3.71`.
- **Solver:** `solver::backward_euler::CpuNewtonSolver` — backward-Euler hyperelastic FEM
  on `Tet4`, `BoundaryConditions { pinned_vertices (Dirichlet), loaded_vertices (traction,
  `LoadAxis`) }`. Validated against **analytical** benchmarks only: Hertz sphere–plane
  (rel-err < 20 %, monotonic convergence), concentric Lamé shells, the uniform-passthrough
  invariant (`tests/invariant_iv_1_uniform_passthrough.rs` — homogeneous-deformation
  reuse candidate).
- **No measured-data validation exists anywhere** in `sim-soft`. Rows 6–24 validate
  composition/machinery, not material accuracy.

## 2. Thesis

The differential-oracle doctrine that worked for the MSK builder (OpenSim 4.6 stood in for
hardware) applies to materials: a **published, peer-reviewed measurement** stands in for our
own tensile rig. Marechal et al. 2021 measured ASTM D412 uniaxial **true-stress–stretch**
curves on exactly our anchor silicones and published per-material Yeoh/Ogden/MR/NH fits +
the raw data on Zenodo. We simulate the *same test* in `sim-soft` and grade against the
measured curve. This turns "relative tool" into "validated to measurement within regime R,"
and produces an actionable number: *how wrong was the one-point datasheet fit, and does the
ν=0.40 compressible model cap achievable accuracy?*

## 3. End-state (M1 done)

A committed, CI-runnable gate: `sim-soft` Ecoflex 00-30 under uniaxial
tension reproduces Marechal's measured true-stress–stretch curve within tolerance
**over the device-relevant window**, with: (a) a vendored curated reference asset
(measured curve + Marechal params + provenance), (b) an analytical compressible-Yeoh
uniaxial solver + a FEM coupon harness that agree with each other (solver-vs-analytical)
and with measurement (model-vs-measured), (c) a `fit_yeoh_to_curve` utility producing
window-optimal params, (d) the one-point-vs-measured gap banked as a number, (e)
`silicone_table.rs` Ecoflex 00-30 gains a Path-3 (measured) const with citation.

## 4. Gap table

| # | Have | Need | Leaf |
|---|---|---|---|
| G1 | Closed-form `P(F)` for compressible Yeoh | Closed-form **uniaxial** response `σ_true(λ)` (1-D root-find for free-transverse `λ₂`) | S0/S1 |
| G2 | Analytical solver benchmarks only | A **measured** oracle asset (Zenodo curve + Table III params), vendored + provenance | S0/S1 |
| G3 | One-point datasheet params | Params **fit to the full measured curve** over the device window (nonlinear LSQ) | S0/S3 |
| G4 | FEM solver, traction BCs | A **displacement-controlled uniaxial coupon** harness; FEM ≡ analytical (solver fidelity) | S2 |
| G5 | "relative tool" disclaimer | A **quantified** measured-accuracy number + regime statement; provenance upgrade | S3 |

## 5. Decisions (head-engineer; revisit if S0 data says otherwise)

- **D1 — First material = Ecoflex 00-30** *(revised in S0: Dragon Skin data is a 1-byte
  LFS placeholder in the Zenodo v0.0 snapshot; only Ecoflex 00-10/00-30 have real curves).*
  Ecoflex 00-30 was the recon's named alternative, is the most-cited Ecoflex grade, and is
  more tissue-relevant (soft, skin-like). One material nailed, then generalize (M2).
  *(MSK discipline: one material + one mode fully validated before generalizing.)*
- **D2 — Mode = uniaxial tension only** for M1. Shear/equibiaxial = M2.
- **D3 — Stress measure = true (Cauchy).** Oracle reports true stress; convert our
  `P₁₁ → σ_true = P₁₁·λ₁/J` explicitly. State it once, never mix nominal/true.
- **D4 — Fit & validate over the DEVICE window, not to failure.** Marechal fit to failure
  (λ up to ~4.6 for DS30); Yeoh's higher terms carry the high-stretch upturn, so a
  to-failure fit trades away low-stretch accuracy. M1 window = operationally relevant
  stretch (S0 fixes the bound; candidate `λ ∈ [1, ~2]`, well under the 3.71 validity cap).
  Report the gap at the window *and* note to-failure behavior.
- **D5 — Reconcile model-form mismatch by re-fitting OUR model, not adopting theirs.**
  Marechal's Yeoh is **incompressible 3-term** `(C₁,C₂,C₃)`; ours is **compressible 2-term
  at ν=0.40**. We do **not** transplant their coefficients. We fit *our* compressible
  2-term Yeoh to *their measured curve* over the window (Fork-B doctrine: absorb
  incompressibility into effective params) and report the residual. If the residual exceeds
  a useful tolerance *because of* ν=0.40 (lateral contraction error), that is the **measured
  case for pulling the deferred Phase-H near-incompressibility (Tet10 + F-bar / mixed-u-p)
  forward** — an M2 trigger, surfaced as a number, not assumed.
- **D6 — Oracle vendored, not fetched at test time.** Curate a small reference JSON under
  `sim/L0/soft/tests/assets/marechal_2021/` (subsampled measured curve + 5-sample handling
  + Table III params + DOI), mirroring the vendored `opensim_gait2392` refs. CI stays
  network-free.

## 6. Sub-leaf ladder

- **S0 — oracle ingest + analytical spike (THROWAWAY, `#[ignore]`, not committed).**
  Download Zenodo `3611329`; parse Dragon Skin 30 uniaxial true-stress–stretch (confirm
  format + stress convention + per-sample scatter); transcribe Table III DS30 Yeoh params.
  Implement the analytical compressible-Yeoh uniaxial `σ_true(λ)` (root-find `λ₂`). Plot/
  compare four curves over candidate windows: (i) measured, (ii) Marechal incompressible
  3-term Yeoh (Table II closed form), (iii) our compressible 2-term Yeoh @ current
  `silicone_table` params, (iv) our model re-fit over the window. **Answers the open
  questions before any production commitment:** size of the one-point-fit gap; whether
  ν=0.40 caps accuracy (the #1 risk); whether C₃ matters in-window; achievable tolerance;
  the window bound. *De-risk first, exactly like the MSK spikes.*
- **S1 — measured-data asset + analytical-uniaxial validator (committed).** Vendor the
  curated reference JSON (D6). Add the analytical uniaxial solver as a tested helper; unit
  test vs Marechal's closed form (Table II); gate: analytical compressible-Yeoh reproduces
  measured DS30 true stress within tol over the window.
- **S2 — FEM uniaxial coupon + solver-vs-analytical gate.** Displacement-controlled
  uniaxial coupon in `sim-soft` (reuse the `invariant_iv_1` passthrough pattern; **free
  lateral faces** so Poisson contraction is unconstrained → no spurious locking). Assert
  FEM homogeneous response ≡ S1 analytical answer. This **separates solver error from model
  error** before the measured gate.
- **S3 — measured-accuracy gate + curve-fit + provenance.** Headline gate: `sim-soft` DS30
  reproduces the measured curve within tol over the window. Ship `fit_yeoh_to_curve`
  (nonlinear LSQ, window-bounded). Re-tag `silicone_table.rs` DS30 as Path-3 (measured)
  with citation. **Bank the one-point-vs-measured gap number** in memory + the gate's prose.

Each leaf: n+1 cold-read + pre-PR local ultra-review ([[feedback-pre-pr-local-ultra-review]]);
no push / PR without user go-ahead.

## 7. Risks

- **R1 (#1) — ν=0.40 caps achievable accuracy.** Real silicones are ν≈0.4999; at ν=0.40 the
  model under-contracts laterally, so for a given axial stretch the stress diverges from the
  incompressible truth. If S0 shows this exceeds a useful tolerance over the window, M1's
  honest outcome is "validated to X %, ν-limited" + a measured trigger for Phase-H
  incompressibility (M2). **MITIGATION: S0 spike measures it directly, before any
  production code.**
- **R2 — model-form mismatch (2-term compressible vs 3-term incompressible).** Handled by
  D5 (fit our model over the window); quantify residual; decide C₃ by data not assumption.
- **R3 — oracle data parsing / convention.** Zenodo format unknown; true vs engineering
  stress; 5-sample scatter. MITIGATION: S0 confirms format + convention; use the sample mean;
  vendor a fixed subsample (D6).
- **R4 — fit-window sensitivity.** To-failure vs device-window changes the effective params
  materially (the DS30 C₂ gap below). MITIGATION: explicit window (D4); report both.
- **R5 — solver-harness BC locking masquerading as model error.** Over-constraining lateral
  faces induces volumetric locking that looks like a fidelity gap. MITIGATION: S2 free
  lateral faces + the validated passthrough invariant; solver-vs-analytical gate isolates it.

## 8. Illustrative gap (to verify precisely in S0 — OCR'd from Table III, MPa)

Marechal DS30 Yeoh ≈ `C₁ 1.00e-1, C₂ 8.8e-2, C₃ 6.0e-6` MPa vs ours `C₁ 0.099, C₂ 0.0176`
MPa. **C₁ matches (~1 %)** — linear modulus right. **C₂ ~5× higher** in Marechal — but that
fit is **to failure**, where C₂/C₃ carry the high-stretch upturn; the in-window effective C₂
is smaller. This is precisely why D4 (window) + S0 (measure, don't compare catalog numbers)
exist. Do not treat the 5× as the answer; treat it as the question.

## Progress · S0 — FINDINGS (DONE 2026-06-09, spike `tests/spike_uniaxial_s0.rs`, throwaway)

Zenodo `3611329` downloaded; Ecoflex 00-30 measured curve (706 pts, λ∈[1.05,17.07], true
stress MPa) parsed; analytical **compressible-Yeoh free-transverse uniaxial** solver
(root-find λ₂ s.t. P[1,1]=0, σ_true = P₀₀·λ₁/J) built on the real `Yeoh::first_piola`.
Every S0 question answered:

- **★ ROOT CAUSE = the datasheet, not the conversion math.** Smooth-On TDS 100 %-modulus
  (10 PSI = 69 kPa) is **2.32× higher** than Marechal's measured engineering stress at λ=2
  (29.7 kPa). The whole table builds `μ = σ₁₀₀/3` off that TDS number → μ inherits the 2.3×.
  Measured small-strain μ ≈ 13 kPa (secant) vs table's 23 kPa.
- **Gap size = LARGE.** Current params over-predict measured σ_true by RMS **63 % (λ≤1.5),
  85 % (λ≤2), 182 % (λ≤3)**.
- **#1 RISK (R1, ν=0.40) RETIRED for uniaxial tension.** Compressible-ν0.40 vs proper
  incompressible (Table II closed form) differ only **~4–6 pp** over λ≤3 (compr slightly
  *closer*). Near-incompressibility (Tet10 + F-bar, deferred Phase H) is **NOT** an M1
  blocker. *(Caveat: ν may bind in compression/contact/confined modes — revisit in M2+.)*
- **C₃ NOT needed in-window.** Best 2-term fits drive C₂ → small/zero; the λ≤2 curve is
  nearly Neo-Hookean. Stay 2-term.
- **Measured accuracy ACHIEVABLE with the existing model.** Re-fit compressible 2-term Yeoh
  (μ, C₂; λ=4μ fixed): **λ≤2 → μ=16.6 kPa (−28 %), C₂≈0.25 kPa, RMS 6 % (max 24 %), abs-RMS
  0.59 kPa**; λ≤1.5 → μ=15.2 kPa, RMS 5 %; λ≤3 → 14 % (J-upturn begins). Refit μ *rises*
  with window (15→17→20 kPa) — exactly why D4 fits the **device** window not to-failure
  (to-failure NH fit gives μ=32 kPa; truth near origin ~13 kPa).

**Locked by S0:** D1 = Ecoflex 00-30; M1 device window = **λ ≤ 2.0** (report λ≤1.5 tighter +
λ≤3 sensitivity); M1 tolerance gate = **≤ 10 % RMS over λ≤2** (S0 achieves 6 %); banked gap =
**~85 % → ~6 % RMS**. R1 retired for tension. C₃ out. ν=0.40 kept (Fork-B: absorb into μ).
The spike `tests/spike_uniaxial_s0.rs` is throwaway/uncommitted — **delete when S1 lands.**

## Progress · S1 — SHIPPED (2026-06-09, branch `feat/soft-fidelity-m1-silicone-uniaxial`, NOT pushed)

Committed leaf. **Vendored asset** `sim/L0/soft/tests/assets/marechal_2021/`
(`ecoflex_00_30_uniaxial.json` = 100-pt curated subsample λ≤3.3 + datasheet-baseline
snapshot; `PROVENANCE.md`). **Production module** `sim/L0/soft/src/material/uniaxial.rs`:
`free_transverse_uniaxial<M: Material>` (root-find λ_t s.t. lateral P₁₁=0, on the real
`first_piola`) + `fit_yeoh_uniaxial` (ν=0.40 two-stage grid). Re-exported at crate root.
**Unit tests** (3): residual-zero across the sweep, small-strain ≡ linear elasticity (on a
C₂=0 material), fitter round-trips a known Yeoh. **Integration gate**
`tests/uniaxial_measured_accuracy.rs` (2): datasheet snapshot **85 % RMS** over λ≤2 (asserts
the gross gap, keyed off the *snapshot* not the live table so the S3 re-fit can't erase the
evidence) → measured-fit **6.5 % RMS** (μ=16.9 kPa, C₂=0.22 kPa; passes the ≤10 % gate).
grade **A** (Documentation/Clippy/Safety/Deps/Layer/WASM all A); full sim-soft suite green;
S0 throwaway spike deleted.

**Scope note:** the fitter (`fit_yeoh_uniaxial`) landed in S1 (it is the natural validated
pair of the analytical solver, and the measured gate needs it) → **S3 narrows to: apply the
fitter to update `silicone_table` Ecoflex 00-30 to Path-3 (measured) + bank + cross-checks.**
The recon's "unit test vs Table II incompressible closed form" was *replaced* by stronger,
apt validators (the solver's defining residual + the linear-elastic limit + fit
identifiability) — Table II is incompressible, different physics from our compressible
ν=0.40 model, so a direct equality would be wrong; it stays a documented reference.

## Progress · S2 — SHIPPED (2026-06-10, same branch, NOT pushed)

`tests/uniaxial_fem_coupon.rs` — a **constant-strain patch test** separating solver error
from model error. Imposes the analytical homogeneous uniaxial deformation `F=diag(λ,λ_t,λ_t)`
(λ_t from `free_transverse_uniaxial`) as boundary Dirichlet on a `cantilever_bilayer_beam`
box of `Tet4`; free interior nodes start **perturbed off** the affine field; one static
`replay_step` (STATIC_DT=1e3 collapses inertia). Asserts, for λ∈{1.1,1.25,1.5}: (1) every
free interior node is driven back onto the homogeneous affine field (max dev < 1e-6 m); (2)
an interior element's recovered `F` ≡ `diag(λ,λ_t,λ_t)` (< 1e-6), its lateral traction ~0,
and its axial Cauchy ≡ the analytical value. ⇒ the solver adds no error to a homogeneous
uniaxial state, so the S1 measured gap is **model/param error, not solver error**. grade A;
suite green.

**KEY DECISIONS / LIMITATIONS (head-engineer, documented in-file):**
- **Runs on `NeoHookean`** — `HandBuiltTetMesh` only implements `Mesh<NeoHookean>` (no Yeoh
  box mesh; the Yeoh path needs `SdfMeshedTetMesh<Yeoh>` + a box SDF the crate lacks). The
  solver's assembly/Dirichlet/Newton path is **generic over `M: Material`**, so this
  validates the machinery the Yeoh material also flows through; Yeoh constitutive fidelity is
  covered by S1 + the `material::uniaxial` unit tests + `yeoh_contract.rs`.
- **Constant-strain patch test, not a free-lateral traction solve.** The recon's "free
  lateral faces" form (independently *re-discovering* λ_t under traction) needs **per-axis
  (roller) Dirichlet BCs** — the Phase-2 BC surface is full-3-DOF pins only. **→ M2 API
  follow-up: add roller/per-axis Dirichlet (also needed by the keystone soft↔rigid coupling).**
- Interior must start *near* affine (perturbed), not from rest: a rest start over-stretches
  the boundary-adjacent elements past the fail-closed validity gate (σ<2).

## Progress · S3 — SHIPPED as Path-3 (2026-06-10, same branch, NOT pushed) — M1 COMPLETE

**Audit-first overwrite → pivoted to Path-3 by the audit (user-approved).** Per the
"audit-first overwrite" instruction, the Ecoflex 00-30 anchor was first overwritten in place
(measured μ/C₂ + `Measured` provenance) and the workspace exercised. **Audit verdict:
overwriting the anchor is unsound** — `build --all-targets` clean (0 compile breaks; no
downstream asserts the old values), but **4 `silicone_table` tests fail from one root cause:
a measured anchor cannot live in a datasheet family.** (i) `from_effective_shore` hits the
`anchor_name` `unreachable!()` (a `Measured` entry in `ECOFLEX_FAMILY`); (ii) Shore-monotonicity
breaks — measured 00-30 (16.9 kPa) is *softer* than datasheet 00-20 (18 kPa); (iii) provenance
test; (iv) C₂-vs-TDS calibration. **Deciding fact: a coherent default-flip needs measured data
for the WHOLE family; Marechal only measured 00-10 & 00-30 → can't re-base.** So the measured
value belongs in **Path-3**, exactly as the table design intends. Overwrite reverted.

**Shipped (Path-3):** `pub const ECOFLEX_00_30_MEASURED` (`silicone_table.rs`) — the curve-fit
`μ=16.918 kPa, λ=4μ, C₂=0.218 kPa`, `ConstructionSource::Measured` (cited), **deliberately NOT
in `ECOFLEX_FAMILY`** (family stays a coherent datasheet basis). Built as a struct literal (not
`from_measured`, which re-applies the σ₁₀₀/3 formula we're escaping). Prominent module-doc
"Measured calibration" note documenting the 2.3× gap + when to prefer the measured const. Gate
`published_measured_table_entry_matches_measurement` (in `uniaxial_measured_accuracy.rs`):
measured const reproduces the curve **6.5 % RMS** over λ≤2, carries `Measured` provenance, and
a `const {}` compile-time invariant pins it < 0.85× the datasheet μ. Datasheet anchor + all
downstream consumers (cf-cast/cf-studio/cf-device) **unchanged** (zero blast radius). grade A;
full sim-soft suite green.

**Banked gap:** datasheet one-point Ecoflex 00-30 over-predicts measured uniaxial true stress
**~85 % RMS over λ≤2**; the measured Path-3 calibration reaches **~6.5 %**. Root cause: the
Smooth-On TDS 100 %-modulus is **2.32× too stiff** vs Marechal's measured value.

**Deferred (named):** flipping the *default* for downstream consumers needs a fully-measured
Ecoflex family (re-measure 00-20/00-50, or adopt a measured family source) — a data-acquisition
task, not a code edit. Plus the M2 roller-BC follow-up from S2.

**★ M1 (silicone uniaxial measured accuracy) COMPLETE — S0→S3 done; branch has 3 commits.**

## 9. M1 validation gate (definition of done)

CI-runnable, network-free: `sim-soft` Ecoflex 00-30 uniaxial true-stress–stretch matches
Marechal's measured mean curve to within **[tol set in S0]** over **[window set in S0]**;
FEM coupon ≡ analytical (solver fidelity) to mesh tolerance; the one-point-vs-measured gap is
banked; provenance upgraded. Honest scope: validated against a *published measurement*, not
our own cast — the hardware tensile gate remains a named later step.

## Key files / pointers

- Material: `sim/L0/soft/src/material/{yeoh.rs, silicone_table.rs, neo_hookean.rs}`
- Solver + BCs: `sim/L0/soft/src/solver/backward_euler.rs`; `readout::{BoundaryConditions,
  LoadAxis}`
- Reuse: `sim/L0/soft/tests/{invariant_iv_1_uniform_passthrough.rs, hertz_sphere_plane.rs,
  yeoh_contract.rs}`
- Vendored-asset precedent: `sim/L0/tests/assets/opensim_gait2392/` (refs + gen scripts)
- Oracle: Marechal et al. 2021, Soft Robotics 8(3):284–297; data Zenodo
  `10.5281/zenodo.3611329`; paper Tables II (uniaxial closed forms) + III (fits).
