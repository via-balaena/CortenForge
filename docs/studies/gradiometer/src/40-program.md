# Sequence, gates, and open questions

This is the program's control surface. It sequences the build the way every CortenForge arc is
run — **fix the sequence and the GO/NO-GO gates; re-plan each rung's internals on arrival; stay
honest about the unknowns.** It is the same "bulletproof gameplan" species as the routing arc:
we always know the next gate and its decision, never the whole implementation in advance.

Each rung follows the standard covenant: recon → spike-before-trust → **ASK** → build minimal
(A-grade, per-crate tests) → build-and-measure (never assert a proxy) → grade → gating review →
**ASK** to commit → PR → CI → merge.

## The ladder

```text
Rung 0  Feasibility (this book)                                  ✅ DONE — conditional GO
Rung 1  ∇g forward model (geometry+mass → gradient tensor)       ▶ FIRST CODE — in-character, unblocks all
GATE 1  Matching leverage: co-design beats hand-matching?        ⛔ LOAD-BEARING — could NO-GO the positioning
GATE 2  Q / dissipation floor (nominal vs pessimistic?)          ⚠ HARD — decides if the device is useful
GATE 3  Self-gravity cancellation to residual (drift-nulled)     — differentiable, in-character
GATE 4  Platform / CMRR + the co-design objective (payoff)       — the north-star
```

*The gates consume the [Chapter 3](30-primitives.md) rungs: Gate 1 ← Rung 2 (matching, on Rung 1),
Gate 2 ← Rung 4 (Q/noise), Gate 3 ← Rung 3 (self-gravity), Gate 4 ← Rung 5 (the objective, with
Rung 2's CMRR). Rung 1 is the pre-gate forward model everything else stands on.*

## Rung 1 — the ∇g forward model  ▶ *start here*

The first real code, and the smallest in-character rung. Build the differentiable
geometry+mass → gradient-tensor primitive ([Chapter 3, Rung 1](30-primitives.md#rung-1--the-g-forward-model-tier-1-crown-jewel--build-first)) on top of the
existing `mass_properties` voxel integration.

- **Spike first:** compute $\Gamma_{ij}$ for a known analytic source (point mass, uniform sphere)
  and check against the closed form; FD-check the gradient w.r.t. a geometry parameter (the same
  discipline as the routing R0 spike).
- **Measurable claims:** matches the analytic gradient tensor; differentiable w.r.t. geometry with
  a clean FD plateau; self-gravity of a `Solid` housing reproduces the [Chapter 1](10-feasibility.md)
  ~1000 E figure.
- **Home:** likely a new `cf-gravity` (or `gradiometer`) L0 crate consuming `cf-design`, mirroring
  how `cf-routing` was seeded. Decide at the rung's recon.

Passing Rung 1 gives a differentiable forward model — necessary for everything, but it does **not**
yet prove the program is worth pursuing. That is Gate 1.

## Gate 1 — does the matching leverage actually exist?  ⛔ the load-bearing decision

**The question the whole positioning rests on:** does a *co-designed* proof-mass match beat a
*hand-matched* one by enough to matter, under **real FEM behavior and real manufacturing
tolerances**? [Chapter 1](10-feasibility.md) showed a toy optimization makes matching a smooth,
differentiable objective — a *necessary* condition — but a toy cannot answer this.

- **What to build:** the FEM matching primitive (two parametric channels; $\varepsilon$ as a
  differentiable function of geometry via [Rung 2](30-primitives.md#rung-2--the-differential--cmrr-framing)) **plus a tolerance model**
  (perturb the geometry by realistic fab tolerances and measure residual mismatch).
- **The test:** co-design the geometry to minimize $\varepsilon$ under tolerance; compare the
  achievable match against the hand-matched baseline ($\sim10^{-3}$–$10^{-4}$) and the lab-trim
  baseline ($\sim10^{-5}$–$10^{-6}$). [Chapter 1, Gate 3](10-feasibility.md#gate-3--what-actually-makes-it-hard-and-where-cortenforge-earns-its-keep) says a moving platform
  needs sub-ppm — so the co-designed match must credibly reach $\lesssim10^{-6}$ *after* tolerances
  to justify the program.
- **GO / NO-GO:** if co-design + symmetry-by-construction reaches sub-ppm robustly, GO — the bet is
  real. If tolerances wash the advantage out to hand-matched levels, the positioning fails; fall
  back to the gravimeter (single element, no matching claim) or shelve.

This gate comes **before** the hard noise-budget work, so we spend the expensive $Q$ effort only
after the differentiator is proven.

## Gate 2 — the $Q$ / dissipation floor  ⚠ the genuinely hard part

The noise floor ([Chapter 1, Gate 2](10-feasibility.md#gate-2--is-the-signal-above-the-noise)) swings the device between "nominal"
(~0.6–2 E, useful) and "pessimistic" (~30–100 E, marginal). Of the parameters that set it, $Q$ is
the one that is modelled dissipation physics rather than a design lever — so it is where the
uncertainty lives. This is the missing Tier-2 physics ([Chapter 3, Rung 4](30-primitives.md#rung-4--the-thermomechanical-noise-budget-tier-2-the-hard-part)):

- **$f_0$ (modal analysis):** an eigenproblem on the structural FEM's stiffness/mass matrices —
  addable, in-character.
- **$Q$ (dissipation):** the hard middle. Squeeze-film gas damping needs a thin-film fluid model;
  thermoelastic damping is coupled thermo-mechanical; anchor loss and material loss angle each need
  their own treatment. Some are separate solvers. **Do not hand-wave $Q$** — it sets whether the
  instrument is worth building.
- **GO / NO-GO:** if a defensible $Q$ model lands the floor in the nominal band for a realistic
  design, GO. If achievable $Q$ pins the device to pessimistic across the design space, the useful
  niche shrinks — report honestly and decide.

## Gate 3 — self-gravity cancellation to residual

Model the instrument's own ∇g from CAD ([Rung 3](30-primitives.md#rung-3--self-gravity-from-cad)) and co-design the structure to
null not just the ~1000 E DC offset but its **thermal drift** below the signal. Differentiable and
in-character; the risk is the thermal coupling, not the gravity.

## Gate 4 — platform / CMRR + the co-design objective (the payoff)

Assemble the full differentiable objective ([Rung 5](30-primitives.md#rung-5--the-differentiable-objective)) into a
`cf_codesign::CoDesignProblem` and co-design geometry against sensitivity + matching + self-gravity
+ noise at once. Establish the still-and-dwell operating model and how far common-mode rejection
can be pushed. This is [Chapter 5](50-payoff.md).

## Open questions and honest risks

- **★ The matching leverage (Gate 1) is unproven** — the single risk that can sink the positioning.
  Everything upstream of it is worth building only as far as it is needed to *run* that gate.
- **$Q$ modelling (Gate 2)** may require a thin-film fluid solver CortenForge does not have; a real
  scope question, not a formality.
- **Room-temp MEMS sensitivity** is genuinely coarse (Chapter 1's optimistic case is generous);
  the honest product is void/anomaly survey, not a precision instrument. Do not let the book's later
  chapters re-inflate past this.
- **Platform vibration** even when "still" (a walking robot creeps, breathes) — the still-and-dwell
  model needs its own measurement before any mobile deployment claim.
- **Fab reality** — DRIE tolerances, single-crystal-Si anisotropy — sit at the scope boundary. The
  tolerance model in Gate 1 is a *proxy* for fab, not the fab itself.

**Posture:** this is a banked expansion, not committed work. Rung 1 is the smallest honest first
step; Gate 1 is where we learn whether to keep going. Nothing starts without an explicit go-ahead.
