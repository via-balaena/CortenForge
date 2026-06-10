# M3 — Deformation modes (equibiaxial + planar) measured accuracy — RECON

*Active recon, opened 2026-06-10. Head-engineer-owned slicing; user picked the macro direction
(deepen substrate fidelity → modes) at the M2-S4 fork. Follows the M1/M2 differential-oracle
doctrine. See `../m2_roller_bc_and_modes/s4_mode_oracle_recon.md` for why modes is its own phase.*

> **Validate that `sim-soft` reproduces the measured *cross-mode* behavior — the
> uniaxial→planar→equibiaxial *relationship* — for Ecoflex, and decide from the residual whether
> the compressible 2-term Yeoh (ν = 0.40) suffices across modes or whether near-incompressibility
> / a richer model is needed.** Oracle = the Hossain group's Ecoflex multi-mode dataset (Liao/Yang/
> Hossain, *Int. J. Mech. Sci.* 206:106624, 2021, Swansea Cronfa `cronfa57203`; uniaxial + planar +
> equibiaxial; digitized).

---

## ★ REFRAME (2026-06-10) — target the cross-mode SHAPE, not absolute modulus

A foundational finding while sourcing the oracle **redirected this phase's target** (user-approved):

**Two peer-reviewed "measured" Ecoflex 00-30 uniaxial datasets disagree by ~1.75×.** The Hossain
2021 curve (digitized from `cronfa57203` Fig 4a) is **~1.70–1.97× stiffer than the Marechal 2021
curve** M1/M2 calibrated to (λ1.5: 1.70× / λ2.0: 1.76× / λ2.5: 1.85×; steady ratio ⇒ not a
digitization artifact — saved at `sim/L0/soft/tests/assets/hossain_2021/ecoflex_00_30_uniaxial_
digitized.csv`). At λ=2 nominal: datasheet TDS 69 kPa, **Hossain ~51 kPa, Marechal ~30 kPa**.
Silicone modulus genuinely varies ~1.5–2× with batch / mix ratio / cure / lab (not rate — Hossain
tested slower, yet stiffer). So:

- **Material variability (~1.75×) is the dominant Layer-1 uncertainty — it dwarfs the 6 % M1 fit,
  the 20 % ν-effect, and the 29 % I₂-spread.** Chasing 6 % absolute-stress fidelity against any
  single curve is below the material-variability floor.
- **It recontextualizes M1:** "the datasheet is 2.3× too stiff vs measured" was vs Marechal's
  specimen specifically; the TDS is only ~1.35× above Hossain.

**Reframed target:** validate the **cross-mode relationship**, which is far more material-invariant
than absolute modulus. Calibrate to a *single source's* uniaxial (absorbing that batch's modulus),
then **predict that same source's planar + equibiaxial** and grade the residual — the model-form
fidelity (the thing ν / I₂ actually move), cleanly isolated from material variability. **Hossain
has all three modes on consistent material → it is the same-batch multi-mode oracle** (Marechal,
uniaxial-only, stays the M1/M2 anchor + a variability data point). The absolute-modulus scatter is
documented as the material-characterization floor — addressable only by measuring the *actual*
device material (the deferred hardware tensile gate). Consequently the near-incompressibility
upgrade is scoped as **solver capability / correctness**, not a high-precision fidelity closer (see
`near_incompressibility_recon.md`).

The sections below predate this reframe; read them through it — "match measured absolute stress to
X %" becomes "match the same-source cross-mode *relationship*; report absolute fidelity only within
the ~1.75× material floor."

---

## 1. Empirical baseline (what's true in the code today)

- **Material calibration is UNIAXIAL-only.** `ECOFLEX_00_30_MEASURED` / `ECOFLEX_00_10_MEASURED`
  (`silicone_table.rs`) are compressible 2-term Yeoh (ν = 0.40, λ = 4μ) fit to Marechal's
  measured **uniaxial** curve via `fit_yeoh_uniaxial` over λ ≤ 2 (~6 % RMS). The model has
  **never been evaluated outside uniaxial tension.**
- **Analytical primitive is uniaxial-only.** `material/uniaxial.rs::free_transverse_uniaxial`
  root-finds `λ_t` for `F = diag(λ, λ_t, λ_t)`. No equibiaxial / planar closed form exists.
- **Roller BCs (M2-S1) make the multi-mode FEM coupons buildable.** The M2 free-lateral
  uniaxial coupon (`tests/uniaxial_roller_coupon.rs`) is the validated template: per-axis
  Dirichlet via `BoundaryConditions::roller_vertices`, drive some faces, free/roller others.
- **No equibiaxial/planar measured validation exists anywhere.**

## 2. Thesis

A hyperelastic model calibrated to one mode mispredicting the others is *the* canonical failure
of this exact workflow — so a uniaxial-fit model is unproven precisely where it is most likely
wrong. The differential-oracle doctrine extends cleanly: a published **multi-mode** silicone
measurement (Hossain Ecoflex: uniaxial + planar + equibiaxial on our anchor family) stands in for
our own biaxial rig. We predict equibiaxial + planar from the existing uniaxial-fit model, grade
against measurement, and let the residual decide the model-form question — surfaced as a number,
not assumed. This is the first real stress test of the *constitutive* choice (2-term Yeoh,
ν = 0.40), and the natural place the deferred near-incompressibility (e) either proves needed or
stays parked.

## 3. End-state (M3 done)

A committed, CI-runnable gate set: `sim-soft` Ecoflex 00-30 reproduces the **measured**
equibiaxial and planar true-stress–stretch curves within tolerance over the device window, with:
(a) closed-form analytical equibiaxial (`F = diag(λ, λ, λ_t)`) and planar (`F = diag(λ, 1, λ_t)`)
responses on the real `first_piola` (root-find `λ_t` per mode); (b) roller-based FEM coupons
(equibiaxial + planar) that agree with the analytical answer (solver fidelity) and measurement
(model fidelity); (c) the **cross-mode gap** of the uniaxial-fit model banked as a number;
(d) a **decision** on whether a joint multi-mode refit / near-incompressibility / a richer model
is needed, with a measured trigger; (e) digitized multi-mode oracle assets, vendored + provenance.

## 4. Gap table

| # | Have | Need | Leaf |
|---|---|---|---|
| G1 | Uniaxial closed form (`free_transverse_uniaxial`) | **Equibiaxial** + **planar** closed forms (1-D root-find `λ_t` per mode) | S0/S1 |
| G2 | Uniaxial oracle (Marechal, machine-readable) | **Digitized** Hossain equibiaxial + planar assets + provenance (figure-data tier) | S0/S1 |
| G3 | Uniaxial-only fit params | The **cross-mode prediction residual**; *if needed*, a **joint multi-mode** Yeoh refit | S0/S3 |
| G4 | Free-lateral uniaxial coupon (roller) | **Equibiaxial** coupon (drive 2 axes, roller the 3rd symmetry plane) + **planar** coupon (drive 1 axis, roller a lateral to zero, free the thickness) | S2 |
| G5 | ν = 0.40 "retired for tension" (M1-S0) | A **measured trigger** for near-incompressibility (Tet10 + F-bar / mixed-u-p) if equibiaxial binds | S0/S3 |

## 5. Decisions (head-engineer; revisit if S0 data says otherwise)

- **D1 — First material = Ecoflex 00-30.** Matches M1; the Hossain set has it; lets us reuse the
  existing `ECOFLEX_00_30_MEASURED` uniaxial fit as the thing under test. (00-10 is a later
  breadth add, as in M2-S3.)
- **D2 — Both modes (equibiaxial + planar) in scope; S0 measures both gaps.** Implementation
  *order* is set by S0 — likely **planar first** (simpler coupon: plane-strain, one driven axis,
  one roller-to-zero lateral, free thickness; gentler volumetric demand), then **equibiaxial**
  (the stringent one — biaxial loads the volumetric response hardest, where ν = 0.40 is most
  likely to bind). The recon scopes both; pick by the S0 numbers.
- **D3 — Stress measure = true (Cauchy).** Oracle reports nominal/Cauchy; convert our `P → σ`
  explicitly per mode (`σ = P·Fᵀ/J` diagonal components). State once; never mix.
- **D4 — Fit & validate over the DEVICE window.** Likely λ ≤ 2 again, but equibiaxial reaches the
  volumetric limit faster, so the window may be tighter for that mode — **S0 fixes each mode's
  window and tol.**
- **D5 — Reconcile model-form by MEASURING, in order, before deciding.** The honest first
  question: does the EXISTING uniaxial-fit `ECOFLEX_00_30_MEASURED` already predict the other
  modes within tol? If yes → the constitutive choice is vindicated cross-mode (a strong result).
  If the gap exceeds tol, evaluate in increasing cost: (a) a **joint multi-mode refit** of the
  *same* 2-term Yeoh (one param set graded across all three modes — does a single (μ, C₂) fit
  everything?); (b) **near-incompressibility** (ν → 0.499 via the deferred Tet10 + F-bar /
  mixed-u-p, Phase-H) if the gap is volumetric; (c) a **richer model** (I₂ dependence / Ogden) if
  the gap is deviatoric-form. Decide by the data; do not assume which. This ordering is the spine
  of S0.
- **D6 — Oracle = digitized Hossain Ecoflex curves, honestly tiered.** Digitize the published
  equibiaxial + planar (and uniaxial, as a self-consistency cross-check vs Marechal) curves with
  WebPlotDigitizer; vendor a curated, cited subsample under `tests/assets/hossain_2020/`
  (mirroring `marechal_2021/`); state plainly this is **figure-digitized data (~1–3 % reading
  error), one provenance tier below M1's machine-readable raw**. Cross-check the *methodology*
  against Meunier 2008 (different silicone, gold-standard 5-mode) — sanity, not a gate.

## 6. Sub-leaf ladder

- **S0 — multi-mode ingest + analytical cross-mode spike (THROWAWAY, `#[ignore]`, uncommitted).**
  The #1-risk de-risk, exactly the M1-S0 pattern. Digitize Hossain Ecoflex 00-30 equibiaxial +
  planar curves. Implement the analytical compressible-Yeoh **equibiaxial** `σ(λ)` (`F =
  diag(λ,λ,λ_t)`, root-find `λ_t` s.t. `P₃₃ = 0`) and **planar** `σ(λ)` (`F = diag(λ,1,λ_t)`,
  root-find `λ_t` s.t. the free-thickness `P₃₃ = 0`) on the real `Yeoh::first_piola`. Then grade
  three predictors against each measured mode: (i) the existing **uniaxial-fit**
  `ECOFLEX_00_30_MEASURED`; (ii) a **joint multi-mode refit** (one (μ, C₂) across all modes);
  (iii) the **incompressible** closed form (to isolate whether ν = 0.40 is the cap). **Answers
  before any production commit:** the cross-mode gap size; whether a single 2-term Yeoh fits all
  modes; whether ν = 0.40 caps equibiaxial accuracy (the e trigger); each mode's window + tol;
  which mode to implement first. *De-risk first.*
- **S1 — digitized multi-mode oracle assets + analytical multi-mode validator (committed).**
  Vendor the curated Hossain assets (D6) + `PROVENANCE.md`. Add equibiaxial + planar analytical
  response as tested helpers (extend `material/uniaxial.rs` → a `multiaxial` surface, or a new
  module); unit tests (residual-zero per mode, small-strain ≡ linear elasticity, the modes agree
  with uniaxial at λ→1). Gate: analytical compressible-Yeoh reproduces measured equibiaxial +
  planar within each mode's tol over its window (using whatever param set S0 selects).
- **S2 — FEM equibiaxial + planar coupons + solver-vs-analytical gate.** Build both coupons on the
  M2 roller-BC template (`uniaxial_roller_coupon.rs`): equibiaxial = drive +x and +y faces,
  roller the three min symmetry planes, free +z (thickness); planar = drive +x, roller a lateral
  (`u_y = 0`) to enforce plane strain, free +z. Assert FEM ≡ analytical per mode (separates
  solver from model error — the volumetric-locking check for equibiaxial lives here).
- **S3 — measured-accuracy gates + (joint refit if needed) + provenance + the (e) decision.**
  Headline gates: `sim-soft` Ecoflex 00-30 reproduces measured equibiaxial + planar within tol.
  Ship the joint multi-mode fit *iff* S0 showed the uniaxial fit insufficient; **bank the
  cross-mode gap**; record the near-incompressibility decision (needed / parked) with its
  measured trigger.

Each leaf: n+1 cold-read + pre-PR local ultra-review ([[feedback-pre-pr-local-ultra-review]]);
no push / PR without user go-ahead. Slicing head-engineer-owned
([[feedback-head-engineer-owns-technical-calls]]).

## Progress · S0 (analytical half) — FINDINGS (2026-06-10, data-free; spike `tests/spike_modes_s0.rs`, throwaway/uncommitted)

The foundational model-form interrogation, run with **zero oracle data** (only the
already-vendored machine-readable Marechal uniaxial curve + the math). Both data-free questions
answered, and they **redirect the phase**:

- **★ (1) ν = 0.40 binds progressively harder across modes.** Compressible ν=0.40 (the real
  production `first_piola`) vs the incompressible closed form, *same* Yeoh params
  (`ECOFLEX_00_30_MEASURED`): max deviation over λ≤2 = **uniaxial 10 % → planar 16 % →
  equibiaxial 20 %**. The volumetric penalty roughly *doubles* from uniaxial to equibiaxial
  (equibiaxial loads the volumetric response hardest). Sanity-checked: compressible σ_uni(λ=2) =
  58.6 kPa (= the M2-S2 coupon value) vs incompressible 65.1 kPa = exactly 10 %. M1 absorbed the
  uniaxial penalty into a refit; a 20 % equibiaxial penalty **cannot** be absorbed that way. ⇒
  **the deferred near-incompressibility (Tet10 + F-bar / mixed-u-p, "Phase-H") is now triggered by
  a number** for multi-mode fidelity (G5/R3 fire).
- **★ (2) Uniaxial calibration does NOT determine equibiaxial (I₂-blindness).** An incompressible
  I₁-only Yeoh (C1=7200, C2=360; uniaxial RMS 3.9 %) and an incompressible I₁+I₂ Mooney-Rivlin
  (C10=7700, C01=0; uniaxial RMS 6.8 %), **both fit to the same measured uniaxial curve**, predict
  equibiaxial stresses that **diverge by up to 29 % at λ=2** (6 %/9 %/19 %/29 % at λ=1.1/1.5/1.75/
  2.0) — with no equibiaxial data used to fit either. (Treloar's classic result, quantified on
  Ecoflex.) The best Mooney-Rivlin fit drove C01→0 because uniaxial is I₂-insensitive — itself the
  point: uniaxial cannot pin the cross-mode response, so **multi-mode data is load-bearing**, not
  optional.

**★ S0 VERDICT — M3 is a SUBSTRATE-UPGRADE phase, not a "validate the current model" phase.** The
shipped 2-term Yeoh at ν=0.40, fit to uniaxial, is structurally untrustworthy in equibiaxial on
*two independent counts* (ν binds ~20 %; uniaxial-fit carries ~29 % model-form uncertainty). So D5
resolves, before any data, to: **(a) bring in near-incompressibility** (pull deferred Phase-H
forward: Tet10 + F-bar or a mixed u-p formulation, ν→~0.499) — a model/solver-side upgrade
needing no oracle to *build*; **and (b) calibrate/validate against MULTI-MODE data** (the Hossain
equibiaxial+planar oracle is now clearly required — uniaxial can't pin I₂/cross-mode). The
acquired-oracle half of S0 (digitizing Hossain) is still needed for S1+ calibration, but the
*model-form decision* is already made. **This is a larger commitment than "grade vs data": it pulls
the banked Phase-H near-incompressibility work into M3's critical path — a scoping decision for the
user.** Spike `tests/spike_modes_s0.rs` is throwaway/uncommitted — delete when S1 lands.

## Progress · S1 — SHIPPED (2026-06-10, branch `feat/soft-fidelity-m3-modes`, NOT pushed)

The reframed cross-mode validation, productionized + committed:
- **Analytical multi-mode response** in `material/uniaxial.rs`: `DeformationMode {Uniaxial, Planar,
  Equibiaxial}` + `free_transverse<M>(material, mode, λ)` (root-find the free thickness stretch s.t.
  `P₃₃=0` on the real `first_piola`); `free_transverse_uniaxial` now delegates (M1/M2 suite
  unchanged — full regression green). Unit tests: thickness-traction-zero per mode, ET>PT>UT stress
  ordering + planar held-width reaction.
- **Vendored Hossain same-source oracle** `tests/assets/hossain_2021/ecoflex_00_30_multimode.json`
  (UT 12 / PT 9 / ET 31 pts, nominal→Cauchy) + raw digitized CSVs + `PROVENANCE.md` (figure-digitized
  tier; the ~1.75× variability finding documented).
- **Gate** `tests/multimode_measured_accuracy.rs` (3 tests, green): (1) calibrate to Hossain
  uniaxial at ν=0.499 → predict **planar 2.3 %, equibiaxial 7.9 %** over λ≤2 (μ=28 kPa, C₂≈0.1 kPa)
  — the 2-term Yeoh form is adequate cross-mode; (2) near-incompressibility **halves** the
  equibiaxial error (ν=0.40 14.6 % → ν=0.499 7.9 %, 1.85×) — the concrete payoff justifying the
  upgrade; (3) inter-source material variability **1.72×** (Hossain/Marechal at λ=1.5) ≫ the model
  error — the floor. grade A; sim-soft suite green (45 binaries). Throwaway spikes deleted.

**Net M3 result (analytical):** at ν→0.499 the substrate reproduces the measured *cross-mode*
behavior to ~2 % (planar) / ~8 % (equibiaxial) — well inside the material floor. The near-
incompressibility upgrade is justified (halves cross-mode error) and right-sized (I₂ residual ~8 %
is below the floor → skip). **Next: the near-incompressibility build (its recon's S1/S2) for the
inhomogeneous/solver side, or call M3's model-form question answered and move on.**

## 7. Risks

- **R1 (#1) — the model fails multi-mode AND neither a joint 2-term refit nor ν fixes it**, so a
  richer model (Ogden / I₂) is needed → genuine scope growth. **MITIGATION: S0 measures all three
  predictors (uniaxial-fit / joint-refit / incompressible) analytically before any production
  code** — if a richer model is required, that is M3's honest finding and a scoped sub-arc, not a
  surprise.
- **R2 — digitization error.** Figure-data carries ~1–3 % reading error that pollutes the gate.
  MITIGATION: careful digitization, vendor + provenance-tier the claim, set tol with margin,
  cross-check methodology vs Meunier. (The hardware biaxial rig remains a later step, as in M1.)
- **R3 — equibiaxial volumetric locking at ν = 0.40 in the FEM coupon** masquerading as model
  error. MITIGATION: S2 solver-vs-analytical isolates it — the analytical primitive uses the same
  ν, so analytical and FEM agree, separating model from solver; if the *analytical* equibiaxial
  itself misses measurement because of ν, that is the (e) trigger (R1/G5), surfaced as a number.
- **R4 — multi-mode coupon BC setup error** (roller misuse for equibiaxial/planar). MITIGATION:
  reuse the validated M2 free-lateral coupon template + the `driven_roller_holds_dof_at_nonzero_
  initial_offset` unit invariant; the planar plane-strain roller (`u_y = 0`) is exactly the
  per-axis Dirichlet M2-S1 shipped.

## 8. M3 validation gate (definition of done)

CI-runnable, network-free: `sim-soft` Ecoflex 00-30 equibiaxial + planar true-stress–stretch
match the digitized Hossain measured curves to within **[tol per mode, set in S0]** over
**[window per mode, set in S0]**; FEM coupons ≡ analytical (solver fidelity); the cross-mode gap
of the uniaxial-fit model is banked; the near-incompressibility / model-form decision is recorded.
Honest scope: validated against a *published, figure-digitized* measurement, not our own biaxial
rig — the hardware multi-mode gate remains a named later step.

## Key files / pointers

- Material: `sim/L0/soft/src/material/{uniaxial.rs (the primitive to extend), yeoh.rs,
  silicone_table.rs (ECOFLEX_00_30_MEASURED = the uniaxial fit under test)}`.
- Coupon template: `sim/L0/soft/tests/uniaxial_roller_coupon.rs` (M2 roller free-lateral) +
  `BoundaryConditions::roller_vertices` (`readout/scene.rs`).
- Oracle: Hossain group — Liao, Hossain, Yao, *Ecoflex polymer of different Shore hardnesses*,
  Mech. Mater. 144:103366 (2020), OA Swansea Cronfa `cronfa53571`; companion stress-recovery
  paper (Glasgow eprint 197199). Cross-check: Meunier et al., Polymer Testing 27:765 (2008).
  Vendored-asset precedent: `tests/assets/marechal_2021/`. Full source list + provenance tiering
  in `../m2_roller_bc_and_modes/s4_mode_oracle_recon.md`.
