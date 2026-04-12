# Review log — Chapter 40: PR 1 Chassis reproducibility

Chapter 40 is the first PR-plan chapter in Part 4 of the study.
It plans PR 1 (Chassis reproducibility) as a 2-PR split per
D14: PR 1a (additive `prf.rs` module) and PR 1b
(`LangevinThermostat` rewrite + `BatchSim::new_per_env`
rewire + regression test + D2 call-site ripple).

PR-plan chapters are a different genre from the argument
chapters that preceded them (Ch 14, 15, 23, 24, 32). An argument
chapter defends a pick against alternatives; a PR-plan chapter
renders a locked pick into an executable plan. The ratio of
defending to rendering flips. Ch 40's factual and thinking
passes were tuned for this genre — the factual pass checked
file:line citations and call-site counts for the rendering
sections, and the thinking pass pressure-tested the four
explicit in-chapter sub-decisions that D1–D15 and Ch 15 left
as "Part 4 concerns." Both passes ran sequentially per the
argument-chapter pattern, because §3.3's `BatchSim` rewire
call is an argument, not a rendering, and the chapter's voice
shifts mode there.

Chapter 40 is a gated chapter per the Ch 01 protocol — it
pauses for user review after the factual + thinking passes
and before commit. This log documents the pre-commit state.

## Recon

Ch 40's recon was focused on three classes of source-tree
verification and one re-read of upstream chapters.

- **Ch 15 (design decisions) — full re-read.** The C-3 pick,
  Route 2 for the PRF, and §4/§5 sketches of the `apply`
  method and the regression test were absorbed in full. Ch 40
  inherits C-3 + Route 2 without re-litigation. One factual
  slip in Ch 15 §5.6 surfaced during recon and is flagged as
  a post-commit patch candidate — see the "Ch 15 §5.6 + Ch 11
  patch candidate" section below.
- **Project memory — D1–D15 table.** The 15 decisions locked
  in session 4 were treated as inputs, not work-products. The
  D-table is reproduced as Ch 40's Table 1 for reader
  convenience; the original source of truth is the session-4
  conversation as recorded in `project_ml_chassis_refactor_study.md`.
- **Source tree — `langevin.rs`, `stack.rs`, `component.rs`,
  `batch.rs`.** Full Read of each file. Cited file:line
  ranges confirmed during drafting and the factual pass.
- **Call-site census for the D2 4-arg ripple.** Grep across
  `sim/L0/thermostat/src/` and `sim/L0/thermostat/tests/` plus
  the one production caller at `ising_learner.rs:198`
  enumerated 56 `LangevinThermostat::new` sites across 18
  files. This census fed Table 2 and the PR 1b description.
- **BatchSim caller census.** Grep across the workspace for
  `BatchSim::new` enumerated ~30 call sites spanning sim-core,
  sim-ml-bridge, sim-integration tests, sim-L1-bevy, benches,
  and examples — none of which currently install a
  `PassiveStack` at all. This is the finding that tipped §3.3
  toward reading (ii) (additive second constructor) over
  reading (i) (destructive rewrite).
- **splitmix64 workspace search.** Grep for
  `splitmix64 | split_mix | SplitMix` returned only documentation
  hits in `docs/studies/`, no production Rust code. This
  confirmed that Ch 32's §4.6 commitment to splitmix64 needs
  a primitive landing somewhere in PR 1 — §2.4's in-chapter
  call (b).

**One targeted recon question** surfaced during the
recon-to-leans conversation and was resolved before drafting:
how literal to read D1's `new(n, factory)` signature. Three
readings of D1 were enumerated — (i) destructive replacement
of `BatchSim::new`, (ii) additive second constructor, (iii)
phased destructive via `#[deprecated]`. The lean toward (ii)
was communicated to the user in the recon-to-leans note; the
user delegated the call under standing autonomous-delegation
authority. §3.3 defends the pick in chapter form.

**Three smaller in-chapter calls** were also surfaced during
the recon-to-leans note and delegated: (a) cross-verification
test location (inline unit tests in `prf.rs`), (b) splitmix64
home (ship in `prf.rs`), (c) visibility (`pub(crate)` for the
four in-crate helpers, `pub` for `splitmix64`, `pub mod prf;`
at the crate root).

## Factual pass

**Status: 37 checked, 33 verified, 4 drift (all corrected in
single round).**

The 4 drifts were minor citation tightenings — none affected
the chapter's structural claims or any in-chapter argument.
Details below.

| # | Claim | Method | Status |
|---|-------|--------|--------|
| 1 | `langevin.rs:167-174` — gating early-return pattern | Read :141-185 | Verified |
| 2 | `langevin.rs:176-183` — stale mutex comment (D13 target) | Read :176-184 | Verified |
| 3 | `langevin.rs:26-42` — module-level mutex doc | Read :26-43 | Verified |
| 4 | `langevin.rs:49` — `use rand_chacha::ChaCha8Rng;` | Read :44-52 | Verified |
| 5 | `langevin.rs:102-111` — current 3-arg constructor | Read :101-112 | Verified |
| 6 | `langevin.rs:134-139` — "one lock acquisition per apply" comment | Read :133-140 | DRIFT → corrected (draft said `:140`, which is the `#[allow]` attribute; comment is at `:134-139`) |
| 7 | `langevin.rs:212-232` — `Diagnose` impl | Read :211-233 | Verified |
| 8 | `langevin.rs:241-266` — stochastic_active roundtrip tests | Read :240-270 | Verified |
| 9 | `langevin.rs:269-276` — `diagnostic_summary_format_matches_spec` test | Read :268-277 | Verified |
| 10 | `langevin.rs:279-308` — `apply_writes_pure_damping_when_stochastic_inactive` test | Read :278-310 | Verified |
| 11 | `langevin.rs:311-363` — FD invariant test | Read :311-365 | Verified |
| 12 | 18 `LangevinThermostat::new` internal test sites in `langevin.rs` | Grep | Verified (count = 18) |
| 13 | `stack.rs:174-199` — `install_per_env` signature and body | Read :164-200 | Verified |
| 14 | `stack.rs:165-172` — "prototype unused inside the function" doc | Read :164-174 | Verified |
| 15 | `stack.rs:231` — `disable_stochastic` location | Read :228-250 | Verified |
| 16 | `stack.rs:282` — `StochasticGuard` type | Read :280-295 | Verified |
| 17 | `stack.rs:558` — lone test caller of `install_per_env` | Grep | Verified (the only caller workspace-wide) |
| 18 | `component.rs:101-108` — `Stochastic` trait | Read :101-108 | Verified |
| 19 | `batch.rs:61-63` — "All environments share the same Arc<Model>" invariant | Read :58-95 | DRIFT → corrected (draft said `:60-63`; tight range is `:61-63` starting at the `/// All environments share` line) |
| 20 | `batch.rs:64-67` — current `BatchSim` struct | Read :58-95 | Verified |
| 21 | `batch.rs:92-94` — `model()` accessor | Read :90-96 | Verified |
| 22 | `batch_sim.rs:55` — `batch_matches_sequential_with_contacts` test location | Read :50-95 | Verified (and noted as correction to Ch 15 §5.6 + Ch 11 inherited citation — see "patch candidate" below) |
| 23 | `batch_sim.rs:59, :98, :131, :165` — four BatchSim::new call sites | Grep | Verified |
| 24 | `vec_env.rs:391` — BatchSim::new call site in VecEnv | Grep | Verified |
| 25 | `task.rs:480, :664, :944, :979, :1003` — 5 BatchSim::new call sites in ml-bridge tests | Grep | Verified |
| 26 | `batch_benchmarks.rs:43` — BatchSim::new in bench | Grep | Verified |
| 27 | `multi_scene.rs:204-215` — `sync_batch_geoms` BatchSim reader | Grep | Verified (function at `:215`; doc preceding) |
| 28 | `ising_learner.rs:198` — production LangevinThermostat::new caller | Grep | Verified |
| 29 | `d2c_cem_training.rs:62` — `SEED_BASE: u64 = 20_260_412` | Read :58-75 | Verified |
| 30 | Ch 32 §4.6 splitmix64 commitment | Read project memory § "Phase 3 Part 3 commits added in session 7" | Verified |
| 31 | Ch 23 §1.2 prose recommendation for splitmix64 | Read 23-competition-api-v2.md §1.2 (prior recon, not re-read) | Verified via Ch 32 review log |
| 32 | Total call-site count for D2 ripple | Grep across all named files | DRIFT → corrected (draft said "~50 across 17 files"; actual = 56 across 18 files). §3.1 summary, §3.5 PR description, §3.6 footer, and §3.4 rewritten to cite the exact count) |
| 33 | `d2c_cem_training.rs:62 through :168` span language | Inspect file | DRIFT → corrected (draft's "line 62 through line 168" read as a continuous range; actual is :62 declaration + :168 first-use-site with further use sites at :289, :325, :350, :384. §1.3 rewritten to name the declaration + use-site structure explicitly) |
| 34 | No existing `prf.rs` or `prf*.rs` in `sim/L0/thermostat/src/` | Glob | Verified (zero matches — net-new file) |
| 35 | No existing `splitmix64` in workspace source code | Grep | Verified (zero production Rust matches; 11 documentation hits in `docs/studies/`) |
| 36 | `Cargo.toml:37-38` — `default = []`, `parallel = ["dep:rayon"]` | Read `sim/L0/core/Cargo.toml` | Verified |
| 37 | Ch 15 §5.6 cites `batch_matches_sequential_with_contacts` at `sim/L0/thermostat/src/stack.rs` | Read 15-design-decisions.md §5.6 | Verified slip — see patch candidate |

**Factual drifts (4), all minor:**

- **#6**: `langevin.rs:140` citation shifted to `:134-139`.
- **#19**: `batch.rs:60-63` tightened to `:61-63`.
- **#32**: "~50 call sites across 17 files" → "56 call sites
  across 18 files" (6 sites, across 5 call locations in
  `langevin.rs` internal tests and one undercounted file,
  accumulated).
- **#33**: `d2c_cem_training.rs:62 through :168` rewritten to
  name the `:62` declaration + `:168` first-use-site with
  further use sites at `:289, :325, :350, :384`.

All four were corrected in a single round. None affected
structural claims, in-chapter arguments, or the D1–D15 input
interpretation.

## Thinking pass

**Status: 2 must-fix, 5 should-fix, 0 minor. All 7 applied
in single round.**

The 10-point brief for argument chapters was applied because
§3.3's `BatchSim` rewire is an argument section, even though
the rest of the chapter is rendering. Findings below.

### Must-fix (2)

**M1. §3.3 reading (ii) did not address "what if a
non-stochastic caller later becomes stochastic?"** The
implicit answer was "migrate that one caller then" but the
argument for reading (ii) depended on this answer without
naming it. A reviewer approaching the reading (ii) defense
fresh would flag this gap. Fix: add a new Fifth observation
to §3.3 that names the caller-becomes-stochastic migration
path explicitly and replaces the old Fifth observation
("reading (ii) is rollback-clean"), which was derivative
rather than independent. This fix also addresses S4 (below)
in the same edit.

**M2. §3.3's `PerEnvStack` trait invention was not
explicitly framed as "rendering force-multiplied by a
cross-crate dependency constraint, not a fresh design
decision."** A later reader seeing the trait could mistake
it for a fifth in-chapter sub-decision that Ch 40 weighed
between alternatives. Fix: add a "This is rendering, not
deciding" paragraph at the end of §3.3's trait-invention
block, explicitly framing the trait as the minimum edit that
makes D1's locked `Arc<PassiveStack>` factory shape
implementable without inverting the sim-core / sim-thermostat
dependency direction. Cross-reference from §4 (below).

### Should-fix (5)

**S1. §4's table should footnote `PerEnvStack` trait invention
as "not a sub-decision."** Without this footnote, a reader
scanning §4's table for a quick index might look for `PerEnvStack`
and not find it, then hunt for it elsewhere. Fix: add a new
bullet to §4's "What Ch 40 does *not* treat as sub-decisions"
list naming the `PerEnvStack` trait and `EnvBatch<S>` generic,
with a cross-reference to §3.3's "This is rendering, not
deciding" paragraph.

**S2. §2.4's splitmix64 argument was missing two counterfactuals.**
The argument-chapter discipline (used by Ch 23, Ch 24, Ch 32)
names the alternatives considered and rejected. §2.4 named one
alternative (Ch 42 owning splitmix64 privately) but not two
others that a stats-aware reader would ask about: a new
`sim-core::util` module, and a workspace-level prng crate. Fix:
add a "Two other homes considered and rejected" paragraph to
§2.4 naming the `sim-core::util` module (rejected: premature
abstraction; one function doesn't justify a module) and Ch 42
owning it privately (rejected: defers the shared-primitive
question and makes the eventual move more expensive). The
workspace-level prng crate was not added as a counterfactual
because it was already rejected implicitly in §2.2's "no new
dependencies" framing and would duplicate §2.2's reasoning.

**S3. §5 should name "restructuring `install_per_env` to take
the prototype as an argument rather than a receiver" as
deferred future work.** The §3.3 usage-pattern sketch shows
the awkwardness of the current receiver pattern, and the
factory-duplication bug Ch 40 caught during drafting stems
from it. Without explicit deferral in §5, a reviewer might
read the sketch as a chassis-level flaw Ch 40 should have
fixed. Fix: add a bullet to §5 naming the deferral and
explaining that PR 1b should not grow to include chassis-API
changes to `install_per_env` itself.

**S4. §3.3's fifth observation was derivative, not
independent.** The original fifth observation argued "reading
(ii) is rollback-clean" — but rollback-cleanness is a
consequence of being additive (the first observation's
point), not an independent argument. A reader counting the
observations would notice this redundancy. Fix: replace the
fifth observation with the caller-becomes-stochastic argument
from M1. Four independent observations become five
independent observations via M1; the double-coverage is
eliminated.

**S5. §3.6 did not name what happens to PR 1a if PR 1b's
reading (ii) is overruled during review.** The large-revision
risk in §3.6 named the scenario (reviewer pushes for reading
(i) or (iii)) but left the question of PR 1a's status open.
The answer is that PR 1a is unaffected because `prf.rs` does
not reference `BatchSim` or any constructor pattern. Fix: add
a sentence to the "Large structural revisions" bullet in §3.6
naming that PR 1a is unaffected and that `PerEnvStack` trait
landing simply shifts PRs under the overrule.

### Minor (0)

The self-review pass on the fixed chapter did not surface
any editing slips at edit-region seams. The M1 fifth
observation rewrite was self-contained, the M2 "This is
rendering, not deciding" paragraph added cleanly after the
existing trait-invention block, and the S1–S5 fixes sat in
their own sections without touching neighboring prose.

### Self-review after fixes

One grep checked for stale references to the old "Fifth,
rollback-clean" language — zero matches. The M1 rewrite
replaced the entire old bullet cleanly.

One grep checked for consistency of `PerEnvStack` references
across §3.3, §3.5, §3.6, §4, and §5 — all 9 references are
consistent in naming and in the "rendering, not deciding"
framing.

One grep checked for `~50 | 17 files` — zero matches, all
five sites converted to `56 | 18 files`.

## Ship verdict

**Ship with must-fix + should-fix applied.** No decision-level
issues. The structural claims, file:line citations, call-site
counts, D1–D15 table values, and Ch 15 / Ch 32 inheritance
references all hold. The four in-chapter sub-decisions are
each defended against their named alternatives, and the
`PerEnvStack` trait invention is explicitly framed as
rendering. The chapter is ready to pause for user review.

Final line count: **1821 lines**. This is slightly longer
than Ch 32 (1460) but in the same order of magnitude as Ch 23
(1181) and Ch 24 (1139). The length is a consequence of PR-plan
chapters carrying more file-level diff tables and longer
PR-description drafts than argument chapters — not a sign of
rhetorical bloat.

## Ch 15 §5.6 + Ch 11 patch candidate

During recon Ch 40 surfaced that Ch 15 §5.6 cites
`batch_matches_sequential_with_contacts` as living at
`sim/L0/thermostat/src/stack.rs`, when it actually lives at
`sim/L0/tests/integration/batch_sim.rs:55` (recon-reported).
Ch 11 — which Ch 15 §5.6 inherited the citation from — also
carries the wrong location. This is a factual slip, not a
decision error; the test exists and covers the non-stochastic
case exactly as both chapters describe.

**Proposed patch sequencing** (following the `3e1ec0ff`
precedent from session 4 where Ch 22's thinking pass
surfaced a Ch 21 framing issue, and the `6b876bc5` precedent
from session 7 where Ch 32's drafting surfaced a Ch 31 §4.4
issue): Ch 40 commits first, then a narrow patch to Ch 15
§5.6 and Ch 11's inherited citation lands as a separate
follow-up commit. Ch 40 does not modify Ch 15 or Ch 11 in
its own commit. The patch is approximately 4 lines of diff
across 2 files, zero semantic change, pure citation fix.

If a reviewer would prefer the patch to land immediately
(either as part of Ch 40's commit or as a separate commit
before Ch 40), say so before the gate is cleared and the
sequencing will adjust accordingly. The §5 "what Ch 40 does
not decide" list already names this patch candidate for the
reader's awareness.

## Gate

Chapter 40 is a gated chapter per the Ch 01 protocol. The
factual and thinking passes are complete and all findings
applied. The chapter waits for explicit user "ok to commit"
before moving to git. Do not commit without the approval.

Upon gate clearance, the commit sequence is:

1. Add `docs/studies/ml_chassis_refactor/src/40-pr-1-chassis-reproducibility.md`
   and `docs/studies/ml_chassis_refactor/review_logs/40-pr-1-chassis-reproducibility.review.md`.
2. Update `docs/studies/ml_chassis_refactor/src/SUMMARY.md` to
   wire the `- [PR 1: Chassis reproducibility]()` placeholder
   to the new file.
3. Commit with message: `docs(ml-chassis-study): Ch 40 PR 1
   Chassis reproducibility plan` (plus the usual co-author
   line per repo convention).
4. Flag the Ch 15 §5.6 + Ch 11 patch candidate as the next
   narrow commit after Ch 40 lands.

No other files are modified in the Ch 40 commit.
