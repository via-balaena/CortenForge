# 2026-04-09 (part 12) — Doc review pass: M1, M4, M5, M2, M3, S1

> Extracted from `MASTER_PLAN.md` §7 part 12 during the 2026-04-09 doc-tree refactor.

- **Trigger**: Reviewer pass (Claude Opus 4.6, 1M context) over
  `MASTER_PLAN.md` + `THERMO_CHASSIS_DESIGN.md`. Produced a
  15-item checklist (5 must-fix, 6 should-fix, 4 nits) recorded
  at `docs/thermo_computing/DOC_REVIEW_2026-04-09.md`. Items
  applied one at a time as separate commits on the branch
  `feature/thermo-doc-review`.
- **Cadence**: Same sharpen-the-axe rhythm — one item per
  commit, two-schemes-then-choose for design changes (M5 and
  M2), explicit acceptance criteria, user confirmation for the
  load-bearing chassis-altering items.
- **Items closed in this entry**:
  - **M1 — Phase 1 sampling-error tolerance wording fix**.
    The original `0.22 · (½kT) ≈ ±22%` phrasing was off by
    a factor of 2. Corrected to `(½kT)·√2/√10 ≈ 0.45·(½kT)`,
    i.e. ±45% of the expected mean for one trajectory; option
    (β) with 100 trajectories brings this to ±4.5% of `½kT`.
    Locked in a tolerance convention: tolerances are always
    expressed as a fraction of the expected mean, not as a
    fraction of `kT`. Commit `75d90de`.
  - **M4 — `Welford::reset()` and `merge()` added to chassis
    Decision 5**. Without `reset()`, the streaming-Welford
    burn-in story is broken. Without `merge()`, option (β)
    has to materialize per-trajectory means and re-Welford
    them. Both shipped in Phase 1; `merge` uses the
    Chan/Pébay parallel formula. `WelfordOnline` gains
    `#[derive(Clone)]`. Commit `031f777`.
  - **M5 — Type-enforce `PassiveComponent` write target**.
    Trait revised from `apply(&self, &Model, &mut Data)` to
    `apply(&self, &Model, &Data, &mut DVector<f64>)`. Data
    is now read-only; `qfrc_out` (length nv) is the only legal
    write target. `PassiveStack::install` allocates a scratch
    buffer per step, runs each component into it, and folds
    the result into `data.qfrc_passive` once at the end. The
    breaking change is free *right now* because no code has
    been written; after Phase 1 ships, the same change would
    be ripple-through painful. Same "loud over silent" line
    as items 2, 4, 6. Commit `9bc030b`.
  - **M2 — Decision 7: stochastic gating for FD/autograd**.
    The Phase 5+ caveat about `cb_passive` firing inside
    `forward_skip()` was previously punted. Doc review M2
    escalated it to a chassis-level decision because
    retrofitting trait shape at Phase 5 is breaking-change
    territory. Two-scheme analysis: Scheme A (orthogonal
    `Stochastic` opt-in trait + RAII `disable_stochastic()`
    guard) vs Scheme B (RNG snapshot/restore on
    `PassiveStack`). Scheme A confirmed. Every stochastic
    component on the roadmap has state-independent noise, so
    "disable noise during FD" gives the exactly-correct
    derivative `∂F_det/∂qpos`. Scheme B is documented as the
    additive future direction (`RngSnapshot` orthogonal trait)
    if a state-dependent noise component ever lands. Commit
    `034d9c8`.
  - **M3 — Q5 (cf-design end-to-end differentiability)
    escalated**. Q5 was previously a single-sentence
    deferred-to-Phase 5 entry. Doc review M3 escalated it to
    active foreground recon, scheduled in parallel with the
    Phase 1 spec drafting, on the asymmetric-risk argument.
    Discovering "no" after months of Phase 2-4 commitment is
    exactly the avoidable surprise sharpen-the-axe forbids.
    Recon scope named (cf-design autograd integration, SDF
    boolean differentiability, marching cubes vs smooth
    alternative, the cf-design Phase 5 differentiable-
    optimization spec). Recon log entry name reserved for when
    the recon actually starts. Commit `6ccc0ff`.
  - **S1 — Q3 (`thrml-rs` existence) resolved**. 5-minute web
    search. Answer is *"yes, but with caveats"*: original
    `extropic-ai/thrml` (JAX) exists, two community Rust
    ports exist on GitHub (`SashimiSaketoro/thrml-rs`,
    `Pingasmaster/thrml-rs`), neither officially maintained
    by Extropic, neither apparently on crates.io. Phase 6
    options refined from "exists / doesn't" binary to (A)
    depend on a port, (B) implement minimal native block-Gibbs
    (~few hundred LOC, leading direction, sharpen-the-axe
    consistent), (C) vendor / fork into a `sim-thrml` sibling
    crate. No commitment yet — Phase 6 spec will decide. The
    "5-minute web search" was the right move because deferring
    it preserved an artificial binary that hid the real
    three-option structure. This commit.
- **Aggregate effect on the chassis**:
  - Trait shape (Decision 1) revised twice: M5 added the
    scratch buffer signature, M2 added the `as_stochastic`
    introspection hook. Both additive and consistent.
  - Stack API (Decision 2) revised twice: M5 added the
    scratch-buffer install body, M2 added
    `set_all_stochastic` and `disable_stochastic` RAII guard.
  - Test utilities (Decision 5) gained `Welford::reset()` and
    `merge()` (M4).
  - File inventory: `component.rs` 30→70 LOC (Stochastic
    trait), `stack.rs` 120→160 LOC (gating + guard),
    `test_utils.rs` 150→170 LOC (reset + merge), total Phase
    1 footprint 790→890 LOC.
  - Decision count: 6→7. The chassis design round complete
    table now has 7 rows.
- **Open follow-ons after this entry** (the should-fix /
  nit items still on the doc review checklist):
  - S2 — Rewrite §3 Current State to reflect chassis-design-
    round outcome (or split master plan into two files).
  - S3 — Reconsider `install_per_env` return type
    (`Vec<Model>` vs `EnvBatch`).
  - S4 — Add "passive forces only" framing paragraph to
    chassis §0.
  - S5 — Call out D4's external dependencies (3D printer
    accuracy, measurement infrastructure).
  - S6 — Note thermostat persistence as a future direction.
  - N1-N4 — polish pass.
  - **Then**: draft the Phase 1 spec.
- **Did NOT yet draft**: any code, any Cargo.toml, any new
  directories. Doc review pass is paper-only by design.

