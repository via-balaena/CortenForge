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
- **Energy-diffusion (low friction, to add):** `k_E ≈ (γ̃·S_b/kT)·(ω_a/2π)·exp(−ΔV/kT)`,
  where `S_b` is the barrier-orbit action (analytic for the quartic well).
- **Turnover (Meľnikov–Meshkov, to add):** `k = k_S · Υ(δ)` with the depopulation
  factor `Υ(δ) = exp[ (1/π)∫₀^∞ ln(1−e^{−δ(λ²+¼)})/(λ²+¼) dλ ]`, `δ = γ̃·S_b/kT`.
  **Pin the exact form to Hänggi–Talkner–Borkovec (Rev. Mod. Phys. 1990) once the
  literature pass returns — do not ship from memory.**
- **Magnetoelastic potential:** elastic `½k_beam·x²` (`k_beam = 3EI/L³`) + dipole
  interaction of the tip moment with each fixed magnet; modal mass
  `m ≈ 0.24·m_beam + m_tip`. Numerically locate minima/barrier → `ΔV, ω_a, ω_b`.

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
