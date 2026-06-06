# D4 Layer 2 — physics-grounded de-risking (before hardware)

> Goal: maximize the probability the **first** physical experiment matches the
> simulation, by killing every identified failure mode in software / math / the
> literature first. Nothing here needs hardware. Builds on G1 (`recon.md`).

The G1 software stack predicts a *quartic, moderate-friction* double well. The
real device is a *magnet-shaped, high-Q, shaker-driven* cantilever. The gap
between those is where a first attempt fails. This layer closes the gap.

## Failure-mode map

| # | How it fails | Software/math that retires it |
|---|---|---|
| **R1** | **Wrong Kramers regime.** Spring steel is high-Q/underdamped; the shipped `kramers_rate` is the spatial-diffusion (moderate/high-friction) form. Underdamped escape is *energy-diffusion-limited* (rate ∝ γ — opposite dependence). Comparing the two is invalid. | Add the low-friction rate + the **Meľnikov–Meshkov turnover** (verified vs the literature pass). A **γ-sweep test** showing the Langevin sim reproduces the full turnover curve, with the high-γ branch matching `kramers_rate`. Estimate the real beam's γ from Q → locate it on the curve. |
| **R2** | **Injected noise isn't a thermal bath.** A shaker gives band-limited forcing not FDT-paired with the beam's intrinsic damping. Does the system reach Boltzmann / a well-defined `T_eff`? | Sim with colored injected noise + non-FDT damping → test in-well Gaussianity, Arrhenius escape, `T_eff(noise power)`, and the noise-bandwidth-vs-`ω_a` condition. |
| **R3** | **Potential shape unknown.** Magnet wells aren't quartic. | **Magnetoelastic forward model** (the keystone): `U(x) = ½k_beam·x² + dipole magnet energy` → `ΔV, ω_a, ω_b`, bistability condition. A `MagnetoelasticPotential` component so the sim runs Langevin in the *real* potential, + a design sweep that outputs rig numbers landing `ΔV/kT_eff ~ 2–5` and a countable rate. |
| **R4** | **Sensing contaminated.** Hall reads the tip magnet *and* the two well magnets. | Magnetostatic model of field-at-sensor vs tip position → sensor placement (linearity, sensitivity, gradiometer), count→position calibration. |
| **R5** | **Uncountable rates.** Beam resonance 10s–100s Hz; need switches ~0.1–10 Hz. | Folds into R3: the design sweep finds configs where `ΔV/kT_eff`, rate, and capture time are all measurable. |
| **R6** | **The sim itself is wrong.** It runs ~15% under the formula — physics or Euler-Maruyama error? | Integrator trust suite: **BAOAB** vs EM, dt-convergence, vs analytic equipartition + Kramers + velocity autocorrelation → the sim's own error bars. Required to trust the low-γ regime in R1. |
| **R7** | **Reinventing known failures.** | Deep literature pass (running): magnetoelastic/mechanical SR + Kramers-escape experiments, effective temperature, practical knobs, documented pitfalls. |

## Build order

1. **R3 — magnetoelastic forward model** (unblocked, self-contained; produces the
   actual rig spec). **Keystone, start here.**
2. **R1 — Kramers turnover** (after the literature pass confirms the MM formula;
   the γ-sweep itself is sim-ground-truth and needs no external formula).
3. **R6 — integrator trust** (pairs with R1; needed to trust the underdamped branch).
4. **R2 — effective-bath validation.**
5. **R4 — sensing model.**
- **R7 — literature pass** runs alongside all of it.

Each lands as a new module/test + recon update, fully gated (clippy/fmt/tests),
the same discipline as G1.

## Key physics to implement (verify against R7 before trusting)

- **Spatial-diffusion (have it):** `k_S = (ω_a/2π)(λ_+/ω_b)·exp(−ΔV/kT)`,
  `λ_+ = −γ̃/2 + √((γ̃/2)² + ω_b²)`, `γ̃ = γ/M`.
- **Energy-diffusion (low friction, to add) — VERIFIED ([`literature.md`](literature.md)):**
  `k = γ·β·I(E_b)·exp(−β·E_b)`, valid for `kT/E_b ≪ 1` and `γ·I(E_b) ≪ kT`; `I(E_b)`
  = barrier-orbit action (analytic for the quartic well). **∝ γ** — this is the
  branch the cantilever lives in.
- **Turnover (Meľnikov–Meshkov) — VERIFIED form:** `k = G_TST·κ_SD·Υ(δ)`,
  `Υ(δ) = exp[(1/π)∫₀^∞ ln(1 − exp(−δ(λ²+¼)))/(λ²+¼) dλ]`, `δ = β·⟨ΔE⟩` (energy
  lost per round trip). Good to **~±20%** near the turnover. **DONE + unit-tested**
  in `sim_thermostat::DoubleWellPotential::{barrier_action, depopulation_factor,
  kramers_rate_turnover}` (`S(E_b)=(8/3)x₀√(MΔV)`).
- **Effective temperature (R2) — VERIFIED:** external non-FDT noise gives
  Arrhenius escape at `kT → kT + D·κ₀²`, **but only for short correlation time**;
  band-limited drive breaks Boltzmann (worst at the barrier top). Design rule:
  keep the drive bandwidth wide vs `ω_a`.
- **Magnetoelastic potential:** elastic `½k_beam·x²` (`k_beam = 3EI/L³`) + dipole
  interaction of the tip moment with each fixed magnet; modal mass
  `m ≈ 0.24·m_beam + m_tip`. Numerically locate minima/barrier → `ΔV, ω_a, ω_b`.

## R3 — DONE (forward model + rig spec)

`tools/pbit-analyze/src/magnetoelastic.rs` (model + 6 tests) and the
`pbit-design` binary (`cargo run -p pbit-analyze --bin pbit-design --release`).

**Headline finding — you cannot run a deep magnetoelastic well thermally.** For
the default rig (45×6×0.08 mm spring steel, Ø5×2 mm tip / Ø8×3 mm fixed magnets,
±5 mm spread), the deep-well configs (small gap) have ΔV up to tens of mJ and
attempt frequencies ~1 kHz; sustaining `ΔV/kT_eff = 3` there would need
**thousands of g** of shaker drive — impossible. The binding constraint is drive
acceleration (`ω_a²·rms`), not amplitude.

**The feasible regime is near the bifurcation.** Two wells appear at a critical
magnet gap (~10 mm here); operating just inside it gives a *shallow, tunable*
barrier. Feasible window for the default rig: **gap ≈ 8.0–9.5 mm**, where
`ΔV ≈ 1.8–122 µJ`, attempt frequency `≈ 20–70 Hz` (shaker-friendly), and the
required drive is `≤ 25 g` (0.5–16 g). The **magnet gap is the primary
`ΔV/kT_eff` knob** — exactly the adjustable bracket in the BOM.

**Rig-spec implications carried into S1b:** the design wants a *shallow* well
(operate near the bifurcation), a low-frequency mode (tens of Hz), and a
**finely adjustable magnet gap** with sub-0.5 mm resolution around ~9 mm. The
attempt frequency landing at 20–70 Hz is well inside what a tactile shaker
delivers. Open knobs to still optimize once R1/R2 land: magnet size (sets the
absolute ΔV scale and the gap), beam stiffness (sets `ω_a`), and `Q` (sets the
drive-to-`kT_eff` conversion).

## R1 — analytic DONE; in-sim validation BLOCKED on R6 (now proven)

The turnover physics is implemented and unit-tested in `sim-thermostat`
(`barrier_action`, `depopulation_factor`, `kramers_rate_turnover`, 13 lib tests):
`Υ ≤ 1` always, `→1` at high friction, `→δ` at low friction, and the shipped
`kramers_rate` is an upper bound that overestimates underdamped. The corrected MM
formula (denominator restored) was caught by the `Υ→δ` unit test.

**Finding (γ-sweep, `sim/L0/therm-env/tests/kramers_turnover.rs`):** the
**Euler–Maruyama Langevin sim does NOT reproduce the energy-diffusion regime.** At
γ=0.1 (δ≈0.46) the turnover predicts ~4× suppression (`Υ≈0.26`), but the EM sim
still tracks the spatial-diffusion rate (`sim/k_S ≈ 1.0`, `sim/k_turnover ≈ 3.8`).
A 4× gap is far beyond the formula's ±20%, so this is the **integrator**, not the
formula — EM over-thermalizes and never sees the energy-diffusion bottleneck.
Where EM is valid (overdamped, γ≥3) the sim matches the analytic rate.

**Consequence:** **R6 (BAOAB) is now a hard prerequisite**, not optional — the
real device is high-Q (deeply underdamped), and we cannot predict or validate its
switching rate in-sim until the integrator is fixed. R6 is promoted to next.

## R6 — DONE (BAOAB integrator + the rate-engine finding)

`sim_thermostat::Baoab1D` (`src/baoab.rs`) — a standalone 1-DOF BAOAB Langevin
integrator with the **exact** Ornstein–Uhlenbeck velocity step. Validated:
`⟨v²⟩ = kT/m` to ~1%, **dt-independent** across dt = 0.001–0.02 (BAOAB's hallmark;
EM is dt-biased). Re-running the R1 γ-sweep through BAOAB
(`tests/baoab_turnover.rs`):

- BAOAB shows the **turnover** and matches the analytic rate overdamped (γ≥3,
  BAOAB/k_turnover 0.89–0.99).
- BAOAB tracks the friction dependence EM erases (γ=0.1: BAOAB/k_S ≈ 0.84–1.0
  across seeds vs EM's ≈1.0).

**Scope / ultra-review correction — what this run does NOT establish.** The
γ-sweep is at `ΔV/kT = 3`, where the simulated rate is **γ-independent (the TST
plateau)**, not energy-diffusion-limited (a genuine energy-diffusion regime needs
`rate ∝ γ`, which is absent here). So this run **cannot test the absolute
underdamped rate or the MM factor**: the earlier "MM over-suppresses at δ≈0.5"
reading conflated "sim sits in the TST plateau" with "formula wrong" and is
**withdrawn**. The MM factor itself is correctly implemented (`Υ→δ` as `δ→0`,
`Υ→1` at large `δ`, verified). **Operational stance:** BAOAB is the best-available
underdamped integrator (equipartition validated, dt-independent); the MM closed
form is a ±20% guide untested in our regime — so predict high-Q rates with BAOAB.
**Required before trusting an absolute underdamped rate (pending):** a
deep-well/`δ≪1` run where escape is genuinely energy-diffusion-limited
(`rate ∝ γ`, `Υ→δ` exact).

## R2 — DONE (the injected-noise bath is thermal only when broadband)

`sim_thermostat::ColoredDriveSim` (`src/colored_drive.rs`) models the macroscopic
shaker-driven beam: intrinsic damping + an **external OU colored force** (not
FDT-paired), no room-temperature noise. Thermal-ness is tested by whether the
**kinetic** temperature `m⟨v²⟩` and the **configurational** temperature
`⟨V′²⟩/⟨V″⟩` agree (they must, for any Boltzmann state).

**Result** (`tests/colored_bath.rs`, τ-sweep at fixed white-limit `kT_eff`):

| τ·ω_a | 0.018 | 0.089 | 0.27 | 0.89 | 2.7 | 8.9 |
|---|---|---|---|---|---|---|
| kin/conf | 1.00 | 0.99 | 0.97 | 0.91 | 0.77 | 0.51 |

The in-well stationary state keeps a **Boltzmann shape** (kinetic ≈ configurational
temperature) when broadband (`τ·ω_a ≪ 1`); as the band narrows the two diverge.
**Quantitative rig rule: drive broadband, bandwidth ≳ a few × `ω_a`** (`τ·ω_a ≲ 0.3`
keeps kin/conf within ~5%). Composes with R3 (`ω_a ≈ 20–70 Hz`): a few-hundred-Hz
band suffices — within a tactile shaker's range.

**Scope / ultra-review corrections.** (1) The kin/conf ratio tests Boltzmann
*shape*, not the absolute temperature: even at `τ·ω_a ≈ 0.27` the OU rolloff
under-drives the well, so the realized `kT` is already ~10% below the nominal
`kT_eff` (a ~2–3× Arrhenius-rate shift in a deep well). **The operating
temperature must be calibrated against the *measured* in-well variance, not the
commanded drive.** (2) The test samples only the in-well (harmonic) region of a
deep no-escape well, so it does **not** probe the barrier-top non-Boltzmann
distortion (the literature-flagged colored-noise failure mode that governs
escape), nor Arrhenius escape or `T_eff(noise power)` — those remain to validate.

## R4 — DONE (magnetostatic sensing model)

`pbit_analyze::sensing` — point-dipole field model of the Hall readout (tip magnet
+ the two static well magnets). Key insight: the well magnets are fixed, so they
contribute only a **constant DC offset** — the design problem is the
*x-dependence*. Result (`tests/`):

- **Recommended geometry:** magnetise the tip magnet **along the motion axis** and
  offset the Hall sensor **perpendicular** by `d ≳ 2·x_max` — where `x_max` is the
  *peak* tip excursion (well throw + underdamped overshoot during a transit +
  thermal jitter), **not** merely `x₀`. `B_z ∝ x/(x²+d²)^{5/2}` inflects at
  `x = d/2`, so the whole crossing must stay inside `|x| < d/2` for a monotonic
  readout; practically `d ≳ 4·x₀`.
- **Sensitivity** ≈ `3μ₀·m_tip/(4π·d⁴)` — **~20 mT/mm at the shallow operating
  point** (`x₀ ≈ 2 mm`, `d ≈ 5 mm`); it falls as `1/d⁴`, so the deep end of the R3
  window (`x₀ ≈ 3.5 mm` → `d ≈ 7 mm`) gives **~5 mT/mm**. `d` must be chosen against
  the actual magnet gap / `x₀`, not fixed.
- **Resolution** ≈ **3 µm/ADC-count at the shallow point** (DRV5055A4, Teensy
  12-bit) — far finer than the sub-mm jitter; ~12 µm at the deep end.
- **Well-magnet constraint:** only a DC offset (the tip response is odd, ~zero at
  centre); total DC field is tens of mT, so pick a wide-range DRV5055 variant
  (A3/A4). `49E` works but is noisier.

## Layer 2 — geometry/sensing/bandwidth VALIDATED; absolute rate PENDING

The de-risking that licenses **ordering parts** is done. Per the ultra-review, the
absolute underdamped switch-*rate* prediction is **not yet** certified — so this is
scoped honestly rather than declared "complete".

| Risk | Status |
|---|---|
| R1 friction regime | ✅ turnover physics (analytic); shipped rate was wrong-regime |
| R6 integrator | ✅ BAOAB validated for equipartition (dt-independent) · ⚠ absolute underdamped *rate* not yet validated (TST-plateau sweep; deep-δ run pending) |
| R3 potential / rig spec | ✅ operate near the bifurcation (gap ~9 mm); ω_a 20–70 Hz |
| R2 bath / bandwidth | ✅ Boltzmann *shape* vs bandwidth (drive ≳ few × ω_a) · ⚠ absolute T_eff, Arrhenius escape, barrier-top distortion not yet tested |
| R4 sensing | ✅ tip-along-motion + perpendicular Hall (d ≳ 2·x_max); ~3 µm res at the shallow point |
| R7 literature | ✅ formulas pinned; platform precedent (Spano 1992) |

**Licensed now:** the **rig geometry (R3), sensing (R4), and drive-bandwidth (R2
shape)** are validated, so ordering the printed brackets + Hall sensor + shaker is
justified — the device *will be* a measurable bistable system.

**Still to validate (the two deferred checks) — these gate *quantitative rate
agreement*, not feasibility, and can run while parts ship:**
1. a deep-well / `δ≪1` BAOAB run to anchor the absolute underdamped rate (where
   `Υ→δ` is exact and escape is genuinely energy-diffusion-limited, `rate ∝ γ`);
2. an Arrhenius-escape / `T_eff(noise-power)` test under colored drive (shallow
   barrier, so the trajectory visits the saddle), to confirm escape — not just
   in-well shape — survives the injected bath.

The remaining build work (S1b printed brackets, S4b firmware bring-up, the
physical gates G2–G5) is execution against this plan.

## Success criterion for Layer 2 (the gate to spend money)

Before any part is ordered we should have, in software:
1. The sim predicting escape with the **correct friction-regime physics** for the
   measured/estimated Q of the real beam (R1+R6).
2. A **rig spec derived from physics** — beam dimensions, magnet sizes/gaps, shaker
   drive — that puts `ΔV/kT_eff` and the switch rate in a comfortably measurable
   window (R3+R5), with the bath assumption checked (R2) and the sensor placement
   designed (R4).
3. **Known error bars** on our own predictions, and a literature-grounded list of
   pitfalls we've designed around (R6+R7).

Then the physical build is executing a plan we already trust, not a guess.
