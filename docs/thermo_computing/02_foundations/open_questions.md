# Open Questions / Unknowns

> Extracted from `MASTER_PLAN.md` §5 *Open Questions / Unknowns* during
> the 2026-04-09 doc-tree refactor.

These need answers before the relevant phase can start. Numbered for
referenceability.

- **Q1 — Noise vs. constraint projection.** **RESOLVED 2026-04-10 (Phase 2
  spec §3) for joint-only models. Constraint-projection testing still deferred.**
  For Phases 2–4's joint-only models (free body, hinge chain, slide-joint
  bistable elements), constraint projection does not apply — joint DOFs in
  generalized coordinates are already the unconstrained variables. The constraint
  solver only activates for equality/inequality constraints (contacts, connect,
  weld), none of which appear in Phases 2–4. The force-space FDT (`σ² = 2γkT`)
  is mass-independent and samples the canonical distribution regardless of M(q).
  The real subtlety Phase 2 tests is the **generalized equipartition theorem**
  for non-diagonal M: `⟨v_i · (M(q)·v)_i⟩ = kT`.
  **Constraint-projection testing** (noise + active contacts/equality constraints)
  is deferred to the first phase that *naturally* introduces active constraints —
  likely D2 (cf-design buckling beam with contact surfaces) or a dedicated
  integration test. Phases 2–4 use joint-only models where the question doesn't
  apply; forcing constraints into Phase 4 to test Q1 would mix concerns.
  The theoretical expectation (Lelièvre, Stoltz) is that constraint forces do no
  work on average and should not affect the canonical distribution of unconstrained
  DOFs — but this is unvalidated in our stack.
  **Additional finding**: Euler-Maruyama integrator breaks equipartition at large
  joint angles where M(q) changes significantly per step — this is an integrator
  limitation, not a thermostat limitation. See
  [`../03_phases/02_multi_dof_equipartition.md`](../03_phases/02_multi_dof_equipartition.md)
  §3 and §12 Finding 1 for the full resolution and stiffness sweep data.
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
  via web search. COMMITTED 2026-04-10: Option B (native single-site Gibbs
  sampler in Rust). Implemented as `GibbsSampler` in `sim-thermostat/src/gibbs.rs`.
  See [`../03_phases/06_gibbs_sampler.md`](../03_phases/06_gibbs_sampler.md) §4
  for the decision rationale.**
  Answer is *"yes, but with caveats."*
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
  differentiable end-to-end?** **RESOLVED 2026-04-09 (recon log part 13):
  NO with three named breaks.** cf-design is differentiable *up to* the
  SDF field level (analytically) and *no further*.

  **What works**: `cf-design/src/param_gradient.rs` (~273 LOC) is
  hand-rolled forward-mode AD over the `FieldNode` enum. Smooth
  booleans and transforms carry full analytic chain-rule derivatives
  in θ; tested against centered-FD oracle to ε=1e-6.

  **Three breaks downstream**:
  1. **Mesh extraction is topologically discrete.** Marching cubes
     and dual contouring both run on a regular voxel grid; vertex/
     edge/face count changes discretely as θ crosses critical
     values. No differentiable surrogate (no DiffMC, no neural
     implicit extractor) anywhere in the codebase.
  2. **Mass properties are unwired** — the **fixable** break.
     `cf-design/src/mechanism/mass.rs` integrates density over a
     grid and returns plain `f64` mass + COM + inertia, with no
     `param_gradient` propagation. The chain rule could be applied
     here in principle (a few hundred LOC, independent of the
     mesh-extraction question) and would unlock analytic
     ∂(mass, COM, inertia)/∂θ.
  3. **sim-core's forward step is opaque to autograd.** `mj_step`
     is a hand-coded forward integrator with contact, constraints,
     and implicit damping; ml-bridge's autograd never touches
     `Model`/`Data` fields. Same problem MuJoCo MJX solved by
     rewriting the entire forward step in XLA — no shortcut.

  The shipping Phase 5 "differentiable design optimization" is
  actually `minimize_fd` — centered finite differences over the
  full `θ → SDF → mesh → Model → simulate → J` pipeline
  (`cf-design/src/optim.rs:9-19` is the smoking gun). No test,
  anywhere in the workspace, exercises an end-to-end analytical
  θ-gradient from cf-design through sim-core.

  **Build-order implication**: D3 (co-design) drops from headline
  experiment to "blocked on cf-design foundation, revisit when
  D2/D4 force the question." Phases 1-4 are unaffected — the
  Langevin thermostat chassis lives entirely on the sim-core side
  and touches no cf-design code. D1 (Brownian motor), D2 (stochastic
  resonance), and D4 (sim-to-real on a printed device) are all
  unblocked; none require end-to-end gradients through cf-design.
  Catalog ordering now reflects this in
  [`../01_vision/research_directions.md`](../01_vision/research_directions.md)
  and `project_thermo_computing.md`.

  **Canonical record**:
  [`../04_recon_log/2026-04-09_part_13_q5_cf_design.md`](../04_recon_log/2026-04-09_part_13_q5_cf_design.md)
  has the full file/line citations, the recon-scope coverage table,
  and the "plausible reactions" trace.
- **Q6 — What's the right reward signal for Phase 7?** ESS, integrated
  autocorrelation time, KL divergence to target, wall-clock to convergence?
  Different choices give different agents. Defer the decision to Phase 6
  results.
- **Q7 — D5 viability recon (placeholder; opens after D1–D4 close).** D5
  (Brownian computer near the Landauer bound) has not had a
  foundation-level recon round. Stack-fit is plausible at Phase 6+ but
  the specific questions D5 implies — and does not yet name — need
  investigation before commitment:
  - **kT-scale energy accounting** in many-DOF mechanical systems —
    can the Langevin chassis (or its successors) account for heat,
    work, and entropy production at the precision Landauer-bound
    claims require? What are the integrator/discretization error
    floors on kT-scale energy budgets?
  - **Theoretical framing** for non-quasistatic Landauer-bound
    approach — the textbook `kT ln 2` is the quasistatic limit;
    finite-time bit erasure has correction terms (Sagawa, Ueda,
    Jarzynski). Which framework do we calibrate against?
  - **Sim-only vs physical realization** — is D5 achievable in
    simulation alone, or does it inherently require the printed-
    device path (D4) to mean anything? If sim-only, what does
    the validation gate look like?
  - **Designable energy landscapes** — can a mechanical lattice
    actually be parameterized such that under Langevin dynamics it
    embodies a target computation (e.g., a 3-bit logic gate)?
    What is the design lever — cf-design parameters? Phase 6 EBM
    training? Both?

  Recon when D1–D4 have closed (per the "don't attempt unless D1–D4
  are working" guidance in
  [`../01_vision/research_directions.md`](../01_vision/research_directions.md)
  D5). Tracked here so future sessions don't lose the placeholder;
  D5's `Foundation status` field in `research_directions.md` references
  this entry as a forward link.
