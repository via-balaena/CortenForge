# Why this study exists

> ## Program status — read this first
>
> This book is the head engineer's working recon for a possible CortenForge expansion:
> the SDK primitives that would enable designing a **room-temperature MEMS gravity gradiometer**.
> It is optimized for fast recall by whoever picks the program up, not for a general audience.
>
> | | |
> |---|---|
> | **Status** | Feasibility established (spike-backed). **Conditional GO** for the large-anomaly / void niche. No code written yet. |
> | **Verdict** | A nominal room-temp MEMS device reaches ~0.6–2 E with dwell; tunnels/chambers/big caches are 1–100 E. Person-borne threats are *out* (physics). |
> | **The bet** | Matching *by construction* (sub-ppm) + self-gravity *from CAD* — both geometry problems, both on CortenForge's exact strength. Not "beat a SQUID on noise." |
> | **Load-bearing unknown** | Does a *co-designed* match beat a *hand-matched* pair by enough to matter under real FEM + tolerances? Unproven. This is [Gate 1](40-program.md#gate-1--does-the-matching-leverage-actually-exist---the-load-bearing-decision). |
> | **First buildable rung** | The ∇g forward model (geometry + mass → gradient tensor + self-gravity), differentiable, built on the existing `mass_properties`. See [Chapter 3](30-primitives.md) + [Chapter 4](40-program.md). |
> | **Out of scope** | Analog readout electronics; silicon micromachining process; anything past the mechanical transducer. |
>
> **Where to go:** feasibility numbers → [Ch 1](10-feasibility.md) · why it's our problem → [Ch 2](20-thesis.md) ·
> what to build on what → [Ch 3](30-primitives.md) · the gated program → [Ch 4](40-program.md) ·
> the north-star → [Ch 5](50-payoff.md) · constants & formulas → [appendix](appendices.md).

CortenForge's transferable core is **differentiable mechanical multiphysics + geometry
+ gradient-based co-design**: rigid dynamics, soft and structural FEM, contact, and
optimization over all of it, with geometry carried differentiably from CAD to physics.
That core transfers wherever a device has a **mechanical transducer whose geometry trades
sensitivity against noise against manufacturability**.

A gravity **gradiometer** — an instrument that measures the *difference* in gravitational
acceleration across a short baseline — is exactly such a device. Its performance is set by
how well two proof masses can be **matched** and how completely the instrument's **own mass**
can be modeled and cancelled. Both are geometry problems. Neither is electromagnetics, circuit
simulation, quantum mechanics, or optics — the solver universes CortenForge deliberately does
not touch.

This book asks a narrow, honest question:

> Can the CortenForge SDK lower the engineering cost of a **room-temperature MEMS gravity
> gradiometer** to the point where the matching and self-gravity work that normally costs
> weeks of hand-calibration falls out of the optimizer instead?

It is **not** a claim to beat a cryogenic SQUID or an atom-interferometer on noise floor. Those
live in cost universes — cryogenics, optical pumping, silicon process — where CortenForge brings
nothing. The bet is confined to the one band where the dominant cost driver is *mechanical*:
matched, symmetric, common-mode-rejecting proof-mass structures, plus a self-gravity model
computed straight from CAD.

## How this book is ordered

The order is deliberate and matches how every CortenForge arc is built — **measure before you
architect.**

1. **Feasibility first.** [Chapter 1](10-feasibility.md) leads with the physics that could kill
   the idea: what signal a real anomaly produces, what noise a room-temperature MEMS proof mass
   floors at, and therefore *what this instrument could ever detect*. Every number there comes
   from a reproducible feasibility spike, not from optimism.
2. **Then the thesis.** [Chapter 2](20-thesis.md) argues — grounded on primitives that already
   exist in the SDK — why a gradiometer is the application where CortenForge's differentiable-
   co-design bet is most load-bearing.
3. **Then the ladder.** [Chapter 3](30-primitives.md) inventories what the SDK already has and
   derives the primitives gradiometer co-design still needs, each one consumer-gated.
4. **Then the program.** [Chapter 4](40-program.md) sequences the build as a gated ladder and
   states the open questions — including the one that could still sink the whole positioning.
5. **Then the payoff.** [Chapter 5](50-payoff.md) describes the north-star: co-designing the
   instrument's geometry against a noise-limited, self-gravity-nulled, common-mode-rejecting
   objective.

Nothing past the mechanical transducer — analog readout electronics, silicon micromachining
process — is in scope. Those stay outside, by design.
