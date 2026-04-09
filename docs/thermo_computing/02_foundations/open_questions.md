# Open Questions / Unknowns

> Extracted from `MASTER_PLAN.md` §5 *Open Questions / Unknowns* during
> the 2026-04-09 doc-tree refactor.

These need answers before the relevant phase can start. Numbered for
referenceability.

- **Q1 — Noise vs. constraint projection.** When stochastic forces are
  written into `qfrc_applied` and projected by the constraint solver, what is
  the effective temperature on constrained DOFs? There is real literature on
  constrained Langevin dynamics (Lelièvre, Stoltz, *Free Energy Computations*)
  — needs a half-day read before Phase 2 can claim a meaningful equipartition
  test on articulated bodies.
- **Q2 — Implicit integrator interaction.** **RESOLVED 2026-04-09 (part 2).**
  Recon of `forward/mod.rs` and `integrate/mod.rs` showed that the canonical
  force-injection point is `qfrc_applied` between `step1()` and `step2()`,
  not a velocity update outside the integrator. Forces written there are
  folded into `qfrc_smooth` and projected by the constraint solver before
  `qacc` is computed; the `integrate()` step then propagates the noise through
  `qvel`. This is integrator-agnostic (Euler, ImplicitFast, Implicit all work;
  RK4 is excluded by the split-step API itself, which is fine for Langevin).
  No BAOAB or integrator bypass needed for Phase 1. See Recon Log entry
  2026-04-09 part 2 for the full trace.
- **Q3 — Does `thrml-rs` exist?** **RESOLVED 2026-04-09 (doc review S1)
  via web search.** Answer is *"yes, but with caveats."*
  - **Original THRML exists**: `extropic-ai/thrml` on GitHub — a JAX
    library, "Thermodynamic Hypergraphical Model Library," focused on
    block Gibbs sampling on sparse heterogeneous graphs and discrete
    EBMs. Confirms the original prompt's premise.
  - **Two community Rust ports exist**, both on GitHub, *neither
    apparently published to crates.io*:
    1. `SashimiSaketoro/thrml-rs` — described as a pure Rust
       implementation of GPU-accelerated sampling for PGMs, with a
       CPU/GPU/hybrid runtime abstraction (`GpuFast`, `CpuPrecise`,
       `Adaptive` routing modes).
    2. `Pingasmaster/thrml-rs` — "1:1 safe Rust rewrite of the JAX
       version. Compatibility is NOT guaranteed. Speed ~2x better on
       CPU, way worse on GPU."
  - **Neither port is officially maintained by Extropic.** Both are
    community ports of unknown maturity. Pingasmaster's explicit
    "compatibility not guaranteed" warning is a red flag for
    correctness-critical use.

  **Implication for Phase 6**: the original "bridge to THRML or
  implement native" framing remains correct, but the trade-off is
  richer than "exists / doesn't exist." Three options for Phase 6:
  - **(A) Depend on one of the community ports** — fastest path to a
    working bridge, but inherits unknown maintenance status and the
    "compatibility not guaranteed" risk. Pingasmaster's port is
    likely more stable based on the description; Sashimi's has the
    GPU story.
  - **(B) Implement a minimal native block-Gibbs sampler** in Rust
    (~few hundred LOC) — most consistent with the sharpen-the-axe
    discipline ("best plans, best materials"). No external dep risk,
    natively integrates with the `sim-thermostat` trait shape, full
    control. Slower to ship.
  - **(C) Vendor / fork one of the ports into a sibling crate**
    (`sim/L0/thrml/` → `sim-thrml`) — middle ground. Inherits the
    initial implementation, owns maintenance going forward, can
    adapt to the chassis (e.g., implement `PassiveComponent` /
    `Stochastic` directly).

  **No commitment yet**, but the leading direction is **(B)** —
  consistent with the same "narrow crates, full control, no external
  silent-failure dependencies" reasoning that drove items 4 and 8 of
  the recon round (`ChaCha8Rng` over `StdRng`, `sim-thermostat`
  sibling crate over sim-core integration). Phase 6 spec will decide.

  **What this changes about the build order**: nothing for Phases
  1-5. Q3 was a Phase 6 question and remains a Phase 6 question; the
  early resolution just sharpens the trade-off the Phase 6 spec will
  face. The "5-minute web search" was the right move because deferring
  it preserved an artificial "exists / doesn't exist" binary that
  hid the real "three options" structure.
- **Q4 — Default `dof_damping` is zero.** **RESOLVED 2026-04-09 (part 2) →
  option (a).** Thermostat carries its own `γ_thermostat[i]` parameter and
  writes both `−γ·qvel` *and* the FDT-paired noise into `qfrc_applied`. Reason:
  `model.implicit_damping` is the canonical per-DOF damping vector populated
  from `jnt_damping` at model init (`model_init.rs:911`) and consumed by
  Eulerdamp, the constraint solver, the implicit-fast `qacc` computation, and
  the derivatives engine. It is model-owned state. Mutating it per step would
  conflate physical damping (joint friction) with thermodynamic damping (FDT
  pairing). Thermostat-owned `γ` keeps the two cleanly separated.
- **Q5 — Is the cf-design → sim-core parameter pipeline already
  differentiable end-to-end?** Phases 5+ depend on it, *and* the
  D3 co-design experiment (the headline Research Direction) does
  too. **Status (updated 2026-04-09 by doc review M3)**: escalated
  from "deferred until Phase 5" to **active foreground recon,
  scheduled in parallel with the Phase 1 spec drafting**. Target
  resolution: before the Phase 1 spec is finalized, so the
  Phase 1+ build order is informed by the answer rather than
  committing to it blind.

  **Why escalated**: the asymmetric risk argument. If Q5 turns
  out "yes, cf-design is fully differentiable end-to-end," the
  cost of the early recon was a half-day and the build order
  is unchanged — no harm done. If Q5 turns out "no" (e.g.,
  non-differentiable booleans, FD-only past a certain layer,
  no autograd hookup at all), the build order *changes
  substantially*. Plausible reactions to a "no":
  1. Build the differentiable layer earlier — potentially
     before D1 (Brownian motor), reordering the priority ladder.
  2. Re-prioritize the Research Directions away from D3 toward
     D1+D2+D4, which need much less differentiability.
  3. Use a surrogate model (e.g., a small neural net trained
     on cf-design output) and explicitly accept the boundary.
  4. Implement the differentiable layer as a sim-thermostat-style
     sibling crate that wraps cf-design with custom autograd.

  Discovering this *after* months of Phases 1-4 commitment is
  the kind of avoidable surprise sharpen-the-axe forbids. The
  cost of doing the recon now is small; the cost of doing it
  late is potentially months.

  **Recon scope** (what to read, ~half-day):
  - `crates/cf-design/src/` — locate the autograd integration
    point (if any). Look for `Tensor`, `Variable`, `grad`,
    `backward`, or interop with `sim-ml-bridge`'s autograd
    engine.
  - The cf-design Phase 5 commit history (per project memory:
    "cf-design Phases 1–5 complete (including differentiable
    design optimization)") — read the spec for the
    differentiable design optimization phase.
  - SDF library composition: are booleans (union, difference,
    intersection) differentiable, or do they introduce
    non-differentiable kinks at the boundary?
  - Mesh extraction: marching cubes or a smooth alternative?
    Marching cubes is non-differentiable at the topology
    boundary.
  - Existing examples or tests that exercise the
    cf-design → sim-core parameter flow with gradients.

  **Recon log entry**: will be opened when the recon starts,
  named "2026-04-XX (part N) — Q5: cf-design end-to-end
  differentiability". This question is the next item on the
  recon queue after the Phase 1 spec is in flight.
- **Q6 — What's the right reward signal for Phase 7?** ESS, integrated
  autocorrelation time, KL divergence to target, wall-clock to convergence?
  Different choices give different agents. Defer the decision to Phase 6
  results.
