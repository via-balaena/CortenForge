# Current State (the starting point)

> Extracted from `MASTER_PLAN.md` §3 *Current state of the thermo line*
> subsection during the 2026-04-09 doc-tree refactor.

*Snapshot — last updated 2026-04-09 (after Phase 1 implementation +
closing review). The Recon Log at the bottom of this document is
append-only history; **this section is the canonical "where are we
right now"** and is rewritten as the state evolves. Re-verify before
relying on these claims; the codebase moves.*

## Current state of the thermo line (where we are *right now*)

*Thermo-computing initiative on `feature/thermo-computing` (now merged
to main as `143c2fc`) plus `feature/thermo-doc-review` for the doc
review pass.*

- **Recon round COMPLETE.** Eight Phase-1-blocking items resolved
  (parts 2-10 of the Recon Log); the substrate (`cb_passive`,
  `qfrc_passive`, `Model::set_passive_callback`, `ChaCha8Rng`,
  `model.timestep` semantics, test conventions, sibling-crate
  layout) is fully understood.
- **Chassis design round COMPLETE.** Seven decisions resolved in
  [`chassis_design.md`](./chassis_design.md):
  1. `PassiveComponent` trait (broad, callback-shaped, type-enforced
     write target via `&mut DVector<f64>` per M5)
  2. `PassiveStack::builder().with(...).build().install(&mut model)`
  3. `install_per_env(prototype, n, build_one)` + defensive clear
     resolves the `Model::clone()` callback-sharing footgun
  4. `Diagnose` trait with `diagnostic_summary -> String` (minimal)
  5. `WelfordOnline` (with `reset` + `merge` per M4) +
     `assert_within_n_sigma` + `sample_stats` test utilities
  6. Flat ml-bridge-style crate layout at `sim/L0/thermostat/`
  7. `Stochastic` orthogonal trait + `disable_stochastic()` RAII guard
     for FD/autograd contexts (Decision 7 added by doc review M2)
- **Doc review pass COMPLETE for must-fixes**. M1 (sampling-error
  tolerance wording), M2 (Decision 7 stochastic gating), M3 (Q5
  escalated to active recon), M4 (Welford reset + merge), M5 (trait
  write-target contract type-enforced) all landed. Should-fix and
  nit items are in flight on `feature/thermo-doc-review`.
- **Phase 1 spec COMMITTED.**
  [`../03_phases/01_langevin_thermostat.md`](../03_phases/01_langevin_thermostat.md)
  drafted, fresh-eyes reviewed, and tightened post-Crack-4 (see below).
  Inherits chassis Decisions 1–7 verbatim. Q1 named as Phase 2 gate.
- **Phase 1 implementation COMPLETE.** `sim/L0/thermostat/` crate
  shipped. Production code: ~1450 LOC across 8 files. **35 unit tests
  + 5 mandatory integration tests + 1 sweep test (`#[ignore]`'d), all
  green** on three back-to-back runs of the central gate, no flake.
  `cargo clippy -p sim-thermostat --tests --release -- -D warnings`
  clean; `cargo fmt --check` clean. **sim-core stays rand-free in
  production** (verified by inspection: `rand` is in
  `sim/L0/core/Cargo.toml` `[dev-dependencies]`, not `[dependencies]`).
- **Phase 1 — three cracks found across two sessions, all fixed.**
  Each is recorded in
  [`../06_findings/`](../06_findings/) with quantitative diagnosis,
  fix plan, and post-fix verification:
  - **Cracks 1+2** (statistical propagation chain):
    [`2026-04-09_phase1_statistical_propagation_chain.md`](../06_findings/2026-04-09_phase1_statistical_propagation_chain.md)
    — chassis Decision 5 ship-justification framed `WelfordOnline::merge`
    as the §7.3 primitive, but merging per-step accumulators across
    autocorrelated trajectories underestimates the std error by
    `√(1+2·τ_int) ≈ 100`. Fix: two-level Welford (per-trajectory inner
    accumulator + across-trajectories top-level accumulator). Merge
    still ships for Phase 4+ IID parallel-reduce contexts but is not
    used by the §7 gate.
  - **Crack 3** (§10 underdamped time constant):
    [`2026-04-09_phase1_section_10_time_constant.md`](../06_findings/2026-04-09_phase1_section_10_time_constant.md)
    — spec said amplitude time constant is `M/γ = 10`; correct value
    is `2M/γ = 20` (the spec confused free-particle velocity decay /
    oscillator energy decay with oscillator amplitude decay). Fix:
    extend §10 decay window from 50,000 to 200,000 steps; envelope
    ≈ `exp(-10) ≈ 4.5e-5`, ~22× margin under the `< 1e-3` threshold.
  - **Crack 4** (burn-in `τ_int` vs `τ_eq` conflation):
    [`2026-04-09_phase1_burn_in_tau_int_vs_tau_eq.md`](../06_findings/2026-04-09_phase1_burn_in_tau_int_vs_tau_eq.md)
    — spec §7.2 used `5·τ_int` as the burn-in heuristic, but burn-in
    adequacy is governed by `τ_eq = M/γ`, which is 2× longer than
    `τ_int = M/(2γ)`. The conflation was invisible at γ=0.1 (residual
    bias 0.82%, swamped by sampling noise) but catastrophic at γ=0.01
    in the §7.4 sweep (49% bias). Fix: name `τ_eq` and `τ_int` as
    distinct quantities in §7.2; central test scaled to `n_burn_in =
    5·τ_eq = 50_000`, `n_measure = 20·τ_eq = 200_000`; sweep test
    parameterized per combo by `τ_eq(γ)`. Quantitative model matched
    both Probe F's central-test 6-seed average and the sweep failure
    to 1% relative — diagnosis confirmed by independent first-principles
    derivation.
  - **Three foundational time constants now distinguished** in spec
    §7.2 ("Two distinct time scales") and inherited by every future
    phase: amplitude decay `2M/γ`, equilibration `τ_eq = M/γ`,
    `½v²` autocorrelation `τ_int = M/(2γ)`. Phase 2+ specs that run
    equipartition tests must use `τ_eq` for burn-in, `τ_int` for
    `N_eff`. **Don't conflate.**
- **Closing implementation review (§13) DONE.** No chassis breaks
  surfaced. Spec/impl alignment verified end-to-end. Documentation
  drift in `test_utils.rs` (lingering "merge for §7.3" framing from
  before Cracks 1+2 fix) reconciled in commit `a7192bc`. Phase 1
  acceptance criteria §12.4 #1–#4, #6, #7 all satisfied; #5
  (`cargo xtask grade sim-thermostat`) deferred to PR-prep per the
  user's "never run heavy gates during normal work" preference.
- **Recon-to-iteration handoff principle empirically validated TWICE
  on this branch.** First iteration (previous session) caught Cracks
  1+2+3 on the very first central-test run after multiple paper
  passes. Second iteration (this session) caught Crack 4 on the very
  first sweep run after Cracks 1+2+3 were fixed. Pattern is now
  battle-tested, not speculative. See
  [`../06_findings/2026-04-09_phase1_handoff_principle_vindication.md`](../06_findings/2026-04-09_phase1_handoff_principle_vindication.md).
- **Open questions status**:
  - Q1 (constrained Langevin) — UNRESOLVED, gates Phase 2.
    Reference: Lelièvre, Rousset, Stoltz.
  - Q2 (implicit integrator interaction) — RESOLVED part 2.
  - Q3 (`thrml-rs` existence) — RESOLVED 2026-04-09 doc review S1.
    Yes with caveats; two community Rust ports on GitHub, neither
    on crates.io, neither officially maintained. Three Phase 6
    options (depend / native / vendor); leading direction is native.
  - Q4 (zero default `dof_damping`) — RESOLVED part 2.
  - Q5 (cf-design end-to-end differentiability) — RESOLVED 2026-04-09
    ([recon log part 13](../04_recon_log/2026-04-09_part_13_q5_cf_design.md)):
    **NO.** cf-design has analytic ∂f/∂θ at the SDF field level only;
    mesh extraction (marching cubes + dual contouring) is topologically
    discrete, mass properties are unwired to `param_gradient`, and
    sim-core's forward step has no autograd handshake. The shipping
    Phase 5 "differentiable design optimization" is actually centered
    finite differences over the full `θ → SDF → mesh → Model → simulate
    → J` pipeline. D3 (co-design) demoted from headline experiment to
    "blocked on cf-design foundation, revisit when D2/D4 force the
    question." Phases 1-4 unaffected.
  - Q6 (Phase 7 reward signal) — deferred until Phase 6 results.
- **Phase 1 PR PARKED** (2026-04-09 PR prep session). Phase 1
  implementation is complete and validated (35 unit + 5 mandatory
  integration + 1 sweep test all green; clippy + fmt clean; rustdoc
  has the four chassis Decision 6 pieces; sim-core stays rand-free in
  production). **Six of seven §12.4 criteria are satisfied; only #5
  (`cargo xtask grade sim-thermostat` reaches A across 7 criteria) is
  open**, and the openness is due to grade tool gaps, not
  sim-thermostat gaps. PR prep was attempted in a dedicated session
  and surfaced two pre-existing latent bugs in the grade tool itself
  (one fixed in flight at commit `c3f08c5`, one diagnosed but unfixed)
  plus six concrete drifts between the tool and the canonical
  `docs/STANDARDS.md`. The gaps are large enough that the right scope
  of fix is its own initiative — recon → audit → chassis → rebuild
  plan → execute → grade — not in-flight patches. Phase 1 PR ships
  AFTER the grade tool audit closes, OR after Phase 1 spec §12.4 #5
  is amended to acknowledge the gate's known gaps. See
  [`../../grade_tool_audit/audit_findings_2026-04-09.md`](../../grade_tool_audit/audit_findings_2026-04-09.md)
  for the full 23-item inventory and
  [`../../grade_tool_audit/recon_plan.md`](../../grade_tool_audit/recon_plan.md)
  for what the next session does.
- **Next action**: **Grade tool audit recon round.** First session
  of the new `grade_tool_audit` initiative. Read-only investigation
  (D1-D5 + B1-B5 audits + F1-F6 decision escalation), produces a
  recon report and a scope recommendation (one of A/B/C/D). See
  `../../grade_tool_audit/recon_plan.md` for the self-contained brief.
  Phase 1 PR resumption + Phase 2 spec drafting + chassis amendment
  all wait behind the audit closing.
