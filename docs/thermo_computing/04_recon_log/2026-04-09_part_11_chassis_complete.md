# 2026-04-09 (part 11) — Chassis design round complete

> Extracted from `MASTER_PLAN.md` §7 part 11 during the 2026-04-09 doc-tree refactor.

- **Trigger**: After the Phase-1-blocking recon round (items 2-8)
  closed in part 10, the user reframed the next step away from a
  monolithic Phase 1 spec and toward a *bolt-pattern design*
  document defining the swappable chassis of the `sim-thermostat`
  crate. The framing was R34-explicit: "we might decide to redo
  the entire thing, as long as the bolt patterns are the same,
  we can swap it." The reference architecture was named
  explicitly as `sim-ml-bridge`. A new artifact was opened:
  `docs/thermo_computing/THERMO_CHASSIS_DESIGN.md`.
- **Cadence**: Same sharpen-the-axe rhythm as the recon round
  (one decision at a time, two schemes per decision,
  recommendation + reasoning, user confirmation, log + commit),
  applied at the *design layer* instead of the code-reading
  layer.
- **Six chassis decisions resolved**:

  | # | Decision                  | Resolution                                                            |
  |---|---------------------------|-----------------------------------------------------------------------|
  | 1 | Core trait shape          | `PassiveComponent` — broad, callback-shaped, no physics in the trait  |
  | 2 | Composition idiom         | `PassiveStack::builder().with(...).build().install(&mut model)`       |
  | 3 | Clone footgun resolution  | `install_per_env(prototype, n, build_one)` + defensive clear inside loop |
  | 4 | `Diagnose` trait surface  | Minimal: `diagnostic_summary -> String` only                          |
  | 5 | Public test utilities     | `WelfordOnline` (Welford 1962) + `assert_within_n_sigma` + `sample_stats` |
  | 6 | Crate layout              | Flat ml-bridge style: 5 source files + 1 test file at `sim/L0/thermostat/` |

  Full reasoning for each decision lives in
  [`THERMO_CHASSIS_DESIGN.md`](./THERMO_CHASSIS_DESIGN.md). The
  recon log entries here capture the *meta* — that the round
  happened, what it decided, what it found.
- **Side findings surfaced during chassis design**:
  - **Decision 5 — autocorrelation correction**: The recon log
    part 2 claim of "10⁻² sampling tolerance for 10⁵ samples"
    was off by ~10× because samples from the Markov chain are
    correlated. Effective N ≈ 10, not 10⁵. The `§The Gap Phase 1
    Validation` paragraph above now carries the corrected
    calculation and a note that the Phase 1 spec will choose
    between three fixes (α/β/γ).
  - **Decision 3 — clone footgun resolved at chassis level**:
    Item 7 / part 9 flagged the `Model::clone()` callback-
    sharing footgun as needing Phase 1 spec resolution. Now
    resolved at the chassis level (not the spec level) via
    `PassiveStack::install_per_env`. The part 9 entry above
    now carries an "update" note pointing to Decision 3.
  - **Total chassis surface estimate**: ~790 LOC across 8
    files (6 source + 1 integration test + 1 Cargo.toml).
    Comparable to a small ml-bridge algorithm + tests.
    Manageable, mechanical, swappable.
- **Why the chassis design round was a separate phase from
  spec drafting**: The user named the principle directly —
  "the implementation may be wrong, that's OK, AS LONG AS the
  bolt patterns are right." Designing the bolt patterns
  *first*, then implementing one component against them,
  produces a chassis that survives implementation surprises.
  The Phase 1 implementation is now allowed to inform a chassis
  revision if needed; the chassis is small enough (~320 LOC of
  trait + composer + diagnose + test_utils, per Decision 6's
  file inventory: 30 + 120 + 20 + 150) that revising it is a
  focused operation, not a rewrite.
- **Memory entry created during this round**:
  [Genuine agreement, not passive](feedback_genuine_agreement.md)
  — feedback memory established when the user noted that their
  string of confirmed recommendations was genuine agreement,
  not passive acceptance. Keep being decisive; don't water down
  recommendations to extract artificial pushback.
- **Did NOT yet draft**: any code, any Cargo.toml, any new
  directories. Six decisions are designed, none are
  implemented. The chassis is a paper artifact at the close of
  this entry.
- **Next action — IN A NEW SESSION**: draft the Phase 1 spec
  (`PHASE_1_LANGEVIN_THERMOSTAT_SPEC.md`) against the now-
  finalized chassis. The spec is small because the chassis
  answered everything except the validation parameter fix
  (α/β/γ). Estimated size ~300-500 lines, half-day of focused
  drafting. The new session should get a tailored prompt that
  references:
  - This master plan (with the chassis design round complete)
  - `THERMO_CHASSIS_DESIGN.md` for the bolt patterns
  - Decision 5's autocorrelation finding and the three fix
    options for the validation parameter set
  - The remaining sharpen-the-axe discipline for spec design

