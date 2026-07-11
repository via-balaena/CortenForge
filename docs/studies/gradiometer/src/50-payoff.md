# The co-design north-star

Everything before this chapter earns the right to state the payoff. If the forward model is built
([Rung 1](30-primitives.md#rung-1--the-g-forward-model-tier-1-crown-jewel--build-first)), the matching leverage is proven ([Gate 1](40-program.md#gate-1--does-the-matching-leverage-actually-exist---the-load-bearing-decision)), and the
noise floor is defensible ([Gate 2](40-program.md#gate-2--the-q--dissipation-floor---the-genuinely-hard-part)), then the instrument becomes a single
optimization.

> **The north-star:** co-design the gradiometer's geometry against one differentiable objective
> that combines noise-limited sensitivity, self-gravity nulling, and common-mode rejection — the
> same body↔device co-design loop CortenForge already runs, pointed at a new axis.

## The objective

The four terms from [Chapter 3](30-primitives.md), combined into one scalar loss over the geometry
parameters $\theta$:

$$
\mathcal{L}(\theta) =
\underbrace{-\,w_s\, S(\theta)}_{\text{sensitivity}}
\;+\; \underbrace{w_m\, \varepsilon(\theta)^2}_{\text{channel match}}
\;+\; \underbrace{w_g\, \big[\partial_T \Gamma_\text{self}(\theta)\big]^2}_{\text{self-gravity drift}}
\;+\; \underbrace{w_n\, \sigma_\Gamma(\theta)}_{\text{noise floor}}
$$

- $S$ — signal sensitivity of the differential channel (Rung 1 + Rung 2).
- $\varepsilon$ — channel scale-factor mismatch under tolerance (Rung 2 + Gate 1); the term that
  makes the pair *matched by construction*.
- $\partial_T \Gamma_\text{self}$ — the thermal drift of the instrument's own self-gravity (Rung 3);
  minimized, not just the DC offset.
- $\sigma_\Gamma$ — the thermomechanical noise floor (Rung 4); the term that couples geometry to $Q$.

Implemented as a `cf_codesign::CoDesignProblem`, minimized by the existing `optimize(problem, x0,
cfg)` driver with `Normalized` loss scaling. **The loop is not new; only the loss is.**

## Why this is the reward worth the ladder

The instrument that comes out the far end has properties no budget vendor's does, and each is a
*consequence of the optimization*, not weeks of hand-labor:

- Its proof-mass pair is **matched by construction** — the sub-ppm match that Chapter 1 showed a
  moving platform demands, produced as a gradient rather than a trim bench.
- Its own **self-gravity is subtracted from its CAD model**, and its structure is shaped so the
  residual *drift* stays below the signal across temperature.
- Its geometry is optimized against a **measured noise floor**, so sensitivity is spent where it
  buys the most, not guessed.

That is the whole pitch, restated exactly: *the matching and self-gravity work that normally costs
weeks of hand-calibration comes out of the optimizer, plus a self-gravity model no budget vendor
has.* Not a lower noise floor than a SQUID — a **cheaper path to a matched, self-gravity-corrected,
noise-aware instrument** in the room-temperature MEMS band where the hard part is mechanical.

## What "done" would mean

The program reaches its north-star when a single `optimize(...)` run takes a parametric
proof-mass-pair geometry and returns a design that is (a) sub-ppm matched under tolerance,
(b) self-gravity-drift-nulled below the signal, and (c) noise-floor-limited in the nominal band —
each claim *measured*, not asserted, against the forward model and noise budget the earlier rungs
built. At that point the gradiometer is not a special project; it is one more thing the co-design
loop designs.

Whether the program gets there depends entirely on [Gate 1](40-program.md#gate-1--does-the-matching-leverage-actually-exist---the-load-bearing-decision). Until it is run, this
chapter is the destination, not a promise.
