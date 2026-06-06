# D4 Layer-2 R7 — literature grounding

Distilled from a fact-checked deep-research pass (adversarially verified, 3-vote).
Everything below is sourced; it pins the formulas R1/R2 implement and the design
constraints the rig must respect. **Net: the underdamped-regime threat (R1) is
real and confirmed, and the effective-temperature approach (R2) works — but only
under a bandwidth condition we must design for.**

## 1. Friction regime — our shipped rate is WRONG for this device (→ R1)

A high-Q spring-steel cantilever (`Q ~ 100s–1000s`, so `γ/ω = 1/Q ≪ 1`) sits
**deep in the energy-diffusion (underdamped) Kramers regime**, not the
spatial-diffusion regime our `kramers_rate` implements. The escape rate vs damping
is a **bell-shaped turnover**: rises with γ at low friction, peaks, falls at high
friction; turnover near `kT/E_b ~ γ/ω_b`. The shipped moderate/strong-friction
prefactor **overestimates** the rate in our regime and "breaks down at low
friction."

- **Energy-diffusion (weak-friction) rate** — use this for the cantilever:
  `k = γ·β·I(E_b)·exp(−β·E_b)`, valid for `kT/E_b ≪ 1` and `γ·I(E_b) ≪ kT`.
  `I(E_b)` = action of the barrier orbit (analytic for the quartic well). Note
  it is **∝ γ** — opposite to the spatial-diffusion form.
- **Turnover bridge (Meľnikov–Meshkov 1986)**: `k = G_TST · κ_SD · Υ(δ)`, with
  depopulation factor `Υ(δ) = exp[(1/2π)∫ ln(1 − exp(−δ(t²+¼)/2)) dt]`,
  `δ = β·⟨ΔE⟩` (reduced energy lost to the bath per barrier→well→barrier round
  trip). **Accurate only to ~±20% in the turnover region**; PGH (1989) extends to
  full friction + memory.
- Sources: Hänggi–Talkner–Borkovec, *Rev. Mod. Phys.* 62, 251 (1990), Eqs.
  4.33/4.49/4.50; Pollak, *ChemPhysChem* (2023) doi:10.1002/cphc.202300272.

**R1 action:** implement the energy-diffusion rate + the MM factor (these exact
forms), validate the Langevin sim reproduces the full γ-turnover, and report rate
predictions with a ~20% band (not tighter) near the turnover.

## 2. Effective temperature from injected noise — works, conditionally (→ R2)

When a bistable system is driven by external Gaussian noise **not** FDT-paired to
its damping, it can still reach a **Boltzmann-like** stationary state with
**Arrhenius** escape at an effective temperature: `kT → kT + D·κ₀²` (reduces to
ordinary Kramers when the external noise `D→0`). The full Kramers turnover
survives even with multiplicative / colored external noise.

**But the clean result is conditional — the failure modes matter:**
- It holds cleanly only for **short noise correlation time τ** (wide bandwidth).
- **Band-limited (colored) drive ⇒ non-Boltzmann** stationary state (UCNA:
  gradient-squared + Hessian-determinant corrections), and the approximation is
  **worst exactly at the barrier top** (`V''<0`) — the saddle that governs escape.
  Colored noise can even produce a counter-intuitive **trimodal** distribution.
- Experiment (Mestres et al., *PRE* 90, 032116, 2014, optically-trapped colloid):
  the external-noise intensity sets a genuine effective temperature and work
  obeys Crooks' theorem at it — **only when the sampling and noise-cutoff
  frequencies are chosen properly**.
- Sources: Ray Chaudhuri/Banik/Ray (arXiv:cond-mat/9911262; *PRE* 63 061111, 82
  041113); Shit et al., *Chem. Phys. Lett.* (2012) doi:10.1016/j.cplett.2012.06.024;
  Maggi et al., *Sci. Rep.* 5:10742 (2015); Mestres et al. (2014).

**R2 action + design constraint:** **keep the shaker drive-noise bandwidth WIDE
relative to the well frequency `ω_a` (small effective τ)** so the
effective-temperature / Arrhenius picture our sim assumes actually holds. Validate
in sim by sweeping τ and watching the in-well distribution depart from Gaussian /
the barrier region misbehave. This is a hard rig requirement, not a nicety.

## 3. The platform is experimentally validated (→ confidence + technique)

Macroscopic magnetoelastic bistables are a proven noise-driven barrier-crossing
platform:
- **Spano, Wun-Fogle & Ditto, *Phys. Rev. A* 46, 5253 (1992)** — a buckled
  magnetoelastic *ribbon*; adding broadband white noise raised the response SNR by
  **10–12 dB** (the hallmark non-monotonic SR gain), **measured via
  residence-time distributions** — exactly the dwell-time analysis `pbit-analyze`
  already does. Strong precedent that our measurement approach is right.
- Also: ferrite-garnet films (Grigorenko 1994), YIG spheres (Reibold 1997);
  Gammaitoni–Hänggi–Jung–Marchesoni SR review, *Rev. Mod. Phys.* 70, 223 (1998).

## Consequences carried forward

1. **R1 is mandatory** before the comparison is valid — and now has verified
   formulas. The 8% "headline slope" we once celebrated was the *wrong-regime*
   rate; the right test is the γ-turnover and the energy-diffusion branch.
2. **R2 has a concrete design rule** (wide drive bandwidth) and known failure
   modes (colored-noise non-Boltzmann, barrier-top breakdown, trimodality).
3. **Measurement = residence-time / dwell-time distributions** — validated by
   Spano 1992; our analyser is aligned.
4. **Expect ~20% prediction error near the turnover** — set tolerances honestly.
