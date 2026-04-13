# PR 1: Chassis reproducibility

Part 4 is the execution layer of the study. Chapters 14 and 15
picked the design (C-3, Route 2, the convention around
`install_per_env`); the 14 decisions locked in session 4 after
Ch 15 committed — D1 through D14, with D15 added later in the
same session — specified the concrete shape of the refactor
at the level of constructor signatures, bit layouts, stored
field names, and test coverage. Ch 40 is the first of three
PR-plan chapters in Part 4, and it owns PR 1: chassis
reproducibility. Its job is to turn the Ch 15 pick plus the
D1–D15 values into an executable plan — what files change,
what the diff looks like, what the test surface becomes, what
order the PRs land, and what the rollback path is if something
breaks mid-land.

PR-plan chapters are a different genre from the argument
chapters that preceded them. An argument chapter picks between
alternatives (Ch 15 over {A-3, B-3, C-1, C-3}; Ch 24 between
three reconciliation shapes for `EpochMetrics::mean_reward`).
A PR-plan chapter renders a locked shape into a landing plan,
makes the small calls that the argument chapter left as "Part 4
concerns," and writes the PR descriptions. The ratio of
defending to rendering flips: Ch 40 spends most of its pages
describing what will happen, and spends a minority on the few
in-chapter sub-decisions that actually require argument.

Ch 40 plans the **2-PR split** committed in D14:

- **PR 1a** — an additive module `sim/L0/thermostat/src/prf.rs`
  that ships the counter-based PRF primitive (`chacha8_block`,
  `expand_master_seed`, `encode_block_counter`, the Box–Muller
  wrapper, and a `splitmix64` helper for downstream seed
  derivation), plus a cross-verification test that asserts
  `chacha8_block` agrees with `rand_chacha 0.9.0` at matching
  block coordinates. PR 1a has zero consumers at merge time —
  it is purely additive chassis material that PR 1b will call
  into — so it can land independently and poses no rollback
  risk.
- **PR 1b** — the `LangevinThermostat` rewrite, the
  `BatchSim::new_per_env` constructor addition, the new
  `parallel_matches_sequential_with_langevin` regression test
  alongside the existing `batch_matches_sequential_with_contacts`
  test, and the `D2` constructor-signature ripple across 56
  call sites. PR 1b is the user-visible change that turns C-3
  from a plan into code.

Ch 40's in-chapter sub-decisions are four in number, and none of
them re-litigate any of the D1–D15 decisions. Section 2 covers
three of them (where `prf.rs`'s tests live, whether `splitmix64`
ships in `prf.rs` or is deferred to Ch 42, and what visibility
the module takes). Section 3 covers the largest one (how
`BatchSim::new` grows the per-env path without breaking the ~20
non-stochastic callers of the existing constructor). Section 4
tables them together. Section 5 draws the scope line.

## Section 1 — Scope and inheritance

### 1.1 What Ch 40 inherits from Ch 15

Ch 15 picked **C-3**: shape 3 (counter-based PRF) hosted on
per-env component instances via `PassiveStack::install_per_env`,
with **Route 2** (manual ChaCha8 block function in the
thermostat crate) as the PRF implementation. The four steps of
Ch 15's argument — shape 3 wins the mechanism axis, hosting A
eliminated on the strict reading of the `Data` invariant,
hosting C wins over B on balance once gating-under-FD is
accounted for, the shortlist collapses to C-3 — are Ch 40's
premise, not its work. Ch 40 does not defend C-3. It plans its
landing.

Ch 15 also specified four smaller design calls that fall out of
C-3: per-env seed derivation dissolves under shape 3 (no
derivation needed — `traj_id` is simply the env index fed as a
constructor argument); gating preservation is unchanged in shape
from the current code's `AtomicBool` + early-return pattern at
`langevin.rs:167-174` (recon-reported); the regression test is a
parallel-matches-sequential assertion using the Ch 13
sequential-fallback oracle; and the PRF implementation is Route
2. Each of these has concrete values locked in session 4 after
Ch 15 committed.

### 1.2 What Ch 40 inherits from the session-4 D-decisions

The 14 decisions locked in session 4 after Ch 15 committed, with
D15 added later in the same session, are the concrete values
that sit between Ch 15's argument and Ch 40's landing plan.
Table 1 names each decision and the value it takes. They are
inputs to Ch 40, not work-products, and Ch 40 does not
re-litigate any of them.

| # | Decision | Value |
|---|----------|-------|
| D1 | `BatchSim::new_per_env` signature | factory closure: `new_per_env(n, factory)` where `factory: impl FnMut(usize) -> (Model, Arc<PassiveStack>)`. (Note: §3.3 renders this as an additive second constructor, not a destructive rename.) |
| D2 | `LangevinThermostat::new` signature | direct 4-arg: `new(gamma, k_b_t, master_seed, traj_id)`. `with_ctrl_temperature` remains the builder method for optional config. |
| D3 | PRF module name | `prf.rs` inside the thermostat crate |
| D4 | `encode_block_counter` bit layout | 32/32 split — `(traj_id as u64) << 32 \| (step_index as u64)`. 4B envs × 4B steps per env, symmetric, explainable in one line. |
| D5 | General-case vs single-block PRF wrapper | general case (loop over DOF groups of 8). Removes the 8-DOF ceiling at the cost of ~10 lines. |
| D6 | Verification test coverage | edge cases (seed=0, counter=0, block boundary, near `u64::MAX`) + 3 random (seed, counter) pairs |
| D7 | Stored field name + expansion | `master_key: [u8; 32]` on the struct. Constructor parameter stays `master_seed: u64` (user-facing); `expand_master_seed(u64) -> [u8; 32]` sits between them. |
| D8 | `diagnostic_summary` output tuple | `(k_b_t, n_dofs, master_seed, traj_id)` — static only. Counter is runtime state and doesn't belong in a configuration identifier. |
| D9 | Regression test strategy | strategy A — feature-gate-per-test. Run the test twice: once under default features (sequential), once under `--features parallel`. Keeps the current feature-gate architecture unchanged. |
| D10 | Seed count for regression test | 3 seeds: `0`, `0xD06F00D_42`, `u64::MAX` |
| D11 | Test fixture | reuse the 1-DOF SHO fixture from the existing Phase 1 integration tests |
| D12 | Second-stochastic-component trait formalization | deferred until a second component exists. `prf.rs` IS the formalization. A `CounterBasedStochastic` trait would only pay for itself if the chassis needed to iterate stochastic components with access to their counters, which isn't on the roadmap. |
| D13 | Stale comment at `langevin.rs:176-183` | remove entirely (the comment references the mutex, which no longer exists under C-3) |
| D14 | Landing strategy | 2-PR split. PR 1a = additive `prf.rs` module + cross-verification test. PR 1b = `LangevinThermostat` rewrite + `BatchSim::new_per_env` addition + new regression test + existing test updates + stale-comment removal. |
| D15 | `batch_matches_sequential_with_contacts` handling | the new Langevin regression test lives **alongside** the existing deterministic-physics contacts test, not as an update to it. Keeps the existing test's scope clean (parallel/sequential agree for deterministic physics) and gives the new test a focused identity (parallel/sequential agree for stochastic components). |

A reader coming to Ch 40 cold should read the Table 1 row for
each decision before reading the section that implements it. The
decisions are named rather than defended — each was a real call
made under broad user delegation during the session-4
conversation — and Ch 40's sections assume them as context.

### 1.3 What Ch 40 inherits from Ch 32

Ch 32 (hyperparameter sensitivity) committed in session 7 with
five decisions plus one named contingency. Only one of those
decisions touches Ch 40: **Decision 4** (initial-batch
composition) committed to `splitmix64(MASTER.wrapping_add(i))`
as the seed-derivation rule for replicates, with
`MASTER = 20_260_412` matching `d2c_cem_training.rs:62`
(recon-reported) and Ch 23 §1.2's example. The splitmix64
primitive does not exist anywhere in the codebase — the current
`d2c_cem_training.rs` declares `SEED_BASE: u64 = 20_260_412` at
`:62` and uses it via raw sequential `SEED_BASE + seed_offset + i
as u64` arithmetic at `:168` (with further additive-offset uses
at `:289`, `:325`, `:350`, `:384`, all recon-reported), which is
exactly the form Ch 23 §1.2 warned against. Ch 42 (PR 3, the
rematch) will be the PR that adopts splitmix64 at the rematch
analysis layer, and it needs a primitive to call.

The question Ch 40 has to answer is whether `prf.rs` — the
chassis-level PRF module that PR 1a lands — ships a
`splitmix64` helper alongside `chacha8_block`, or whether
`splitmix64` lives somewhere else. §2.4 picks it, with the
short answer being **yes, ship it in `prf.rs`**. Naming it
here keeps Ch 42 from inventing a private copy of a primitive
that has no reason to live in two places.

### 1.4 Genre note: Ch 40 renders, Ch 15 defends

A reader of the argument chapters (14, 15, 23, 24, 32) is used
to seeing counterfactuals walked explicitly, rejection grounds
named, and the decision defended on principle grounds. A reader
of Ch 40 will see less of that and more of the other shape:
file-level diff descriptions, call-site counts, PR-description
drafts, landing-order justifications, rollback-path
walkthroughs. The sub-decisions Ch 40 does make (the ones that
D1–D15 and Ch 15 left as "Part 4 concerns") get the full
counterfactual treatment — Section 4's table names each one and
§2 / §3 argue them in place — but most of the chapter is
rendering, not arguing. That is the intended genre for Part 4:
turn the committed design into landed code, plan the landing
carefully, and do not re-open questions that are already
answered. A reader looking for defenses of D1–D15 should read
Ch 15 and the session-4 review log, not Ch 40.

## Section 2 — PR 1a: the `prf.rs` additive module

PR 1a is the additive half of D14's 2-PR split. It ships one
new file (`sim/L0/thermostat/src/prf.rs`), one `mod prf;`
declaration line in the crate root, and no changes to any other
file in the workspace. At merge time it has zero consumers —
`LangevinThermostat` does not yet call into it, and no other
stochastic component exists — so the module is a purely
additive chassis primitive that PR 1b will wire up. This
structure lets PR 1a land independently, with its own review
cycle, its own cross-verification test run, and its own merge.
If something goes wrong in PR 1b, PR 1a is already merged and
remains inert-but-correct until the follow-up PR lands.

### 2.1 File-level diff

PR 1a's diff is contained within the thermostat crate and
touches exactly two files:

- `sim/L0/thermostat/src/prf.rs` — **new file**, ~250 lines
  (core PRF primitive ~100 lines, helpers ~40 lines, inline
  unit tests ~100 lines, module-level documentation ~10
  lines). The exact line count is a drafting detail that Part 4
  implementation refines; the shape is fixed by D3–D7.
- `sim/L0/thermostat/src/lib.rs` — **one-line addition**,
  `mod prf;` (or `pub(crate) mod prf;` per §2.5). No other
  `lib.rs` change.

The thermostat crate's `Cargo.toml` is unchanged. No new
dependencies are taken — `prf.rs` is self-contained,
implementing the ChaCha8 block function over the `u32`/`u64`
primitives that `core` already provides. The cross-verification
test imports `rand_chacha`, which is already a runtime
dependency of the crate at `thermostat/Cargo.toml:19`
(workspace-pinned at `rand_chacha = "0.9"`, used by
`LangevinThermostat` at `langevin.rs:49`, recon-reported).
PR 1b demotes it to a dev-dependency once the mutex-bearing
`LangevinThermostat` shape is replaced and the only remaining
consumer is PR 1a's cross-verification test.

### 2.2 Public surface and the 32/32 bit-layout pickup

The module's public surface is five items, as locked by D3–D7:

```rust
/// Compute one ChaCha8 block keyed by `key` at block counter
/// `block_counter`. Returns 64 bytes (one ChaCha block) of
/// uniform pseudorandom output. Pure function: same inputs
/// always produce the same output.
pub(crate) fn chacha8_block(key: &[u8; 32], block_counter: u64) -> [u8; 64];

/// Expand a `u64` master seed into a 32-byte ChaCha8 key using
/// the same rule `rand_chacha`'s `SeedableRng::seed_from_u64`
/// uses internally. The cross-verification test in this module
/// asserts the result matches
/// `ChaCha8Rng::seed_from_u64(seed).get_seed()`.
pub(crate) fn expand_master_seed(master_seed: u64) -> [u8; 32];

/// Encode a `(traj_id, step_index)` pair into a 64-bit block
/// counter. The 32/32 split (D4) puts `traj_id` in the high
/// 32 bits and `step_index` in the low 32 bits.
pub(crate) fn encode_block_counter(traj_id: u64, step_index: u64) -> u64;

/// Apply Box–Muller to a 64-byte block, producing 8 `f64`
/// Gaussians as a fixed-size array. Paired-sample structure:
/// bytes 0..16 produce Gaussians [0, 1], bytes 16..32 produce
/// [2, 3], and so on.
pub(crate) fn box_muller_from_block(block: &[u8; 64]) -> [f64; 8];

/// SplitMix64 — the canonical counter-to-stream hash for
/// deriving many uncorrelated `u64` seeds from one master.
/// This is the primitive Ch 32 §4.6 committed to for the
/// rematch's replicate-seed derivation; see §2.4 for why it
/// lives in this module.
pub(crate) fn splitmix64(seed: u64) -> u64;
```

**The 32/32 bit-layout pickup.** Ch 15 §2.3 sketched a 40/24
split (40 bits for `traj_id`, 24 bits for `step_index`) as a
defensible starting point that gave "ample room" at the 32
envs × 500K steps configuration of the D2c rematch. D4 locked
the shape differently: a symmetric 32/32 split, with `traj_id`
in the high 32 bits and `step_index` in the low 32 bits. The
two shapes are equally correct for the rematch configuration
(both support far more envs and far more steps per env than
the rematch uses), but D4's symmetric split wins on
explainability — one sentence describes the layout fully, and
a reader can sanity-check a counter value at a glance. The
40/24 sketch was a Part 4 stub; the 32/32 pick is the Part 4
answer, and Ch 40 §2.2 is the site where that upgrade appears
in the book. This is not a reconsideration of Ch 15 — it is
the rendering of a call Ch 15 explicitly deferred to Part 4.

The `chacha8_block` signature uses `&[u8; 32]` (not `u64`) for
the key because ChaCha8 is defined over a 256-bit key, and
passing a `u64` would force an in-function expansion step that
obscures which expansion rule the function uses.
`expand_master_seed` is the one place the expansion rule lives,
and its cross-verification test pins it to match
`rand_chacha 0.9.0`'s `SeedableRng::seed_from_u64` behavior.

The `box_muller_from_block` signature returns a fixed-size `[f64; 8]`
rather than `Vec<f64>` or an iterator because D5's general-case
wrapper loops over DOF groups of 8, and each iteration needs
exactly 8 outputs per block. A fixed-size array is the most
honest shape for "one block produces 8 Gaussians."

`splitmix64`'s presence in the same module as `chacha8_block`
is a §2.4 call — the short answer is that §2.4 argues yes, and
the signature above reflects that.

### 2.3 Cross-verification test

D6 specifies the correctness-verification coverage: edge cases
(seed = 0, counter = 0, counter at a block boundary, counter
near `u64::MAX`) plus three random `(seed, counter)` pairs. The
edge cases are enumerated; the random pairs are chosen at
test-authoring time and hard-coded with their expected outputs,
so the test is not itself randomized at runtime — the "random"
is only in how the authors picked the pairs.

The test file name and location are an in-chapter call:

**In-chapter call (a): where the cross-verification test
lives.** Two options:

- **Inline unit tests in `prf.rs`.** The test sits in a
  `#[cfg(test)] mod tests { ... }` block at the bottom of
  `prf.rs`, imports `rand_chacha::ChaCha8Rng`, and asserts
  `chacha8_block` matches `rand_chacha`'s output at each
  covered `(seed, counter)` pair. This is the standard Rust
  pattern for testing a single-file module's internals and is
  what most of the thermostat crate does today (see
  `langevin.rs:235-619` for the existing internal test pattern,
  recon-reported).
- **Separate integration test in `tests/prf_cross_check.rs`.**
  The test is a top-level `#[test]` in the integration-test
  directory. This pattern is used elsewhere in the crate for
  tests that exercise multiple components or load MJCF fixtures
  (see `tests/langevin_thermostat.rs`, recon-reported, for the
  prior art).

**Pick: inline unit tests.** Three reasons:

1. PR 1a has zero consumers. The test surface is purely
   primitive-correctness — "does `chacha8_block` agree with
   `rand_chacha` at these 7 points" — and that scope is the
   exact thing inline unit tests are for. There is no MJCF
   fixture to load, no `PassiveStack` to build, no `BatchSim`
   to run. A file-level unit-test block is the cleanest fit.
2. Inline tests let the test author close over the module's
   private helpers without making them `pub(crate)` gratuitously.
   `prf.rs` has some internal helpers (the ChaCha8 quarter-round
   function, for instance) that deserve their own unit tests,
   and those tests need access to `mod` internals — possible
   from an inline block, awkward from an integration test.
3. The existing thermostat crate pattern is inline unit tests
   for single-component modules (`langevin.rs`, `double_well.rs`,
   `ratchet.rs`, etc., recon-reported) and integration tests for
   cross-component or fixture-driven tests. `prf.rs` is a
   single-component module, so inline unit tests match the
   surrounding convention.

The test block sits at the bottom of `prf.rs` in a
`#[cfg(test)] mod tests` scope, imports `rand_chacha::ChaCha8Rng`
and `rand::SeedableRng`, and covers:

- `chacha8_block(expand_master_seed(0), 0)` equals
  `rand_chacha`'s output at the same coordinate
- the block-boundary coordinate (counter just before a
  word-position wraparound — pinned at a specific value during
  implementation)
- `chacha8_block(expand_master_seed(u64::MAX), u64::MAX - 1)`
  equals the reference
- three pre-chosen `(seed, counter)` pairs whose expected
  64-byte outputs are hard-coded in the test file
- `encode_block_counter(traj_id, step_index)` recovers
  `(traj_id, step_index)` via the obvious inverse (simple
  round-trip on a handful of pairs)
- `splitmix64(0)`, `splitmix64(1)`, and two other values match
  the canonical SplitMix64 reference output (pinned against
  the reference implementation at test-authoring time)
- `box_muller_from_block(zeros)` produces the expected
  Gaussians for the all-zero block (a smoke test — Box–Muller
  on zeros is well-defined and pins the implementation's
  orientation)

The test count is ~10 `#[test]` functions. Each is small and
independent. No fixture loading, no MJCF, no `PassiveStack` —
pure primitive verification.

### 2.4 In-chapter call (b): splitmix64 in prf.rs or not

Ch 32 §4.6 committed to `splitmix64(MASTER.wrapping_add(i))`
as the rematch's replicate-seed derivation rule. The primitive
does not exist anywhere in the codebase as of the
`6b876bc5` tip (recon-reported: one Grep across the workspace
returns only documentation hits in `docs/studies/`, no
production Rust code). Ch 42 will be the PR that adopts
splitmix64 at the rematch analysis layer, replacing the raw
sequential `SEED_BASE + seed_offset + i as u64` arithmetic at
`d2c_cem_training.rs:62` through `:168` (recon-reported) that
Ch 23 §1.2 warned against.

The question is whether `prf.rs` — Ch 40's new module — ships
`splitmix64` as a public helper, or whether Ch 42 adds it
somewhere else (a private helper in the rematch test file, a
new module under `sim/L0/thermostat/src/rematch_utils.rs`, a
small helper in `sim/L0/core`, etc.).

**Arguments for shipping splitmix64 in prf.rs:**

- The primitive is the same shape as the rest of `prf.rs`:
  a pure function that hashes an integer to a pseudorandom
  output. Same category, same style, same test discipline.
- Landing it in PR 1a (the additive, zero-consumer PR) means
  PR 1a's chassis contribution is complete — when Ch 42 opens
  its PR, the rematch code just calls `prf::splitmix64` and the
  seed-derivation question is a solved problem. If splitmix64
  lives somewhere else, Ch 42's PR grows a primitive-addition
  step on top of its analysis logic, blurring its scope.
- A reader opening `prf.rs` sees all the rematch-adjacent PRFs
  in one place: the ChaCha8 block function for per-apply noise,
  the block-counter encoder for per-env coordinates, and
  splitmix64 for replicate-level seed derivation. The file is
  "the PRF toolbox for this crate," and the toolbox being in
  one place matches how thermostat-crate modules are organized
  today (`diagnose.rs` holds all the diagnostic helpers, not
  scattered).
- The maintenance cost is small. `splitmix64` is ~5 lines of
  well-known code, tested against a canonical reference.
  Adding it to `prf.rs` grows the file by ~15 lines (function
  + test) and adds zero dependencies.

**Arguments against:**

- `splitmix64` is not used by `LangevinThermostat` or any
  other stochastic component. It is used by the rematch
  *analysis code* (Ch 42), which is a different layer. Strict
  scope-discipline would say PR 1a should ship only what PR 1b
  consumes.
- `prf.rs`'s module-level framing is "counter-based PRF for
  stochastic components." `splitmix64` is a seed-derivation
  primitive, not a noise-generation primitive. Grouping them
  together could mislead a reader into thinking splitmix64 is
  part of the stochastic-component call path.

**Two other homes considered and rejected.**

- **A new `sim-core::util` module.** `sim-core` does not have
  a `util` module today, and introducing one for a single
  5-line function would be either premature abstraction (one
  function in a new module) or an invitation for the module
  to accumulate unrelated helpers over time. Neither is
  desirable. If a future PR needs a second sim-core-level
  utility, the `util` module can be introduced then with two
  callers to justify its existence.
- **Ch 42 owns splitmix64 privately.** The rematch code could
  define `splitmix64` as a private helper in whichever file
  implements the replicate-seed derivation loop. This keeps
  PR 1a strictly scoped and defers the shared-primitive
  question to "if a second caller ever needs it." The cost
  is that Ch 42's PR grows a primitive-addition step on top
  of its analysis logic, blurring the "Ch 42 is the rematch"
  framing that Ch 32 committed to, and the primitive's
  location is discoverable only through Ch 42's own file
  tree. If a third caller later needs splitmix64, the
  primitive has to move anyway, and the move is a larger
  PR than landing it in `prf.rs` today. Deferred-to-later is
  the more expensive path whenever the later move is
  foreseeable.

**Pick: ship splitmix64 in `prf.rs`.** The scope-discipline
argument is real but outweighed by the "solved problem at
Ch 42's start line" benefit. Ch 42 is the largest PR in Part 4
(sim-opt split + rematch + analysis + report generation), and
anything Ch 40 can hand it as a ready primitive tightens
Ch 42's scope. The "counter-based PRF" framing of `prf.rs` is
not load-bearing — the module can be framed as "counter-based
hash primitives for deterministic stochastic behavior," which
covers both uses.

The module-level documentation in `prf.rs` names both uses
explicitly, so a reader is not misled:

```rust
//! `prf.rs` — counter-based hash primitives used across the
//! thermostat crate.
//!
//! Two kinds of primitives live here:
//!
//! 1. **Per-step noise generation.** `chacha8_block` +
//!    `expand_master_seed` + `encode_block_counter` + the
//!    Box–Muller wrapper form the Route 2 PRF chain that C-3
//!    stochastic components (starting with
//!    `LangevinThermostat`) call from their `apply` method.
//!    These replace the per-component `Mutex<ChaCha8Rng>`
//!    pattern that pre-C-3 stochastic components used.
//!
//! 2. **Seed derivation.** `splitmix64` is the canonical
//!    counter-to-stream hash used by Ch 32's rematch protocol
//!    (and by any other code that needs many uncorrelated
//!    `u64` seeds from one master seed). It is not called by
//!    `apply` — it lives here because it is the same shape
//!    (pure integer-to-pseudorandom function) and belongs in
//!    the same toolbox.
//!
//! See the ML chassis refactor study Ch 15 for the argument
//! that shape 3 + Route 2 is the right chassis primitive, and
//! Ch 32 §4.6 for the argument that splitmix64 is the right
//! seed-derivation primitive.
```

The module-level doc makes the "two kinds" framing explicit,
so the mislabeling risk is defused by naming it out loud.

### 2.5 In-chapter call (c): `pub` vs `pub(crate)` visibility

D3 says the module lives in the thermostat crate; it does not
specify whether the module and its items are `pub` (visible to
out-of-crate consumers) or `pub(crate)` (visible only inside
the thermostat crate).

The D12 conversation established that plausible future
stochastic components (two-temperature Langevin, merged
stochastic ratchet, stochastic actuator, Ornstein-Uhlenbeck
colored-noise thermostat, RL exploration noise) all live
inside the thermostat crate. No out-of-crate stochastic
component is on the roadmap. The only out-of-crate consumer
that might want `prf.rs` functions is the rematch analysis
code in Ch 42, and Ch 42 is planned to live in a crate that
imports `sim-thermostat` — splitmix64 is the only function it
would call, and it can call it from inside a `sim-thermostat`
integration test or from a dependent crate.

**Pick: `pub(crate)` for the module and all its functions,
with one exception.** The exception is `splitmix64`, which
Ch 42's rematch code (in a separate crate) needs to call.
`splitmix64` is marked `pub`. The other four functions
(`chacha8_block`, `expand_master_seed`, `encode_block_counter`,
`box_muller_from_block`) are `pub(crate)` because their only
consumers are in-crate stochastic components.

The module itself is declared `pub(crate) mod prf;` in the
crate root. This is a slight inconsistency with the
per-function visibilities — a `pub(crate)` module cannot
export a `pub` function to out-of-crate consumers — so the
actual shape is `pub mod prf;` at the crate root, with
per-function `pub(crate)` on the four in-crate-only helpers
and `pub` on `splitmix64`. This is awkward-looking in the
source but correct. The alternative (declaring the module
`pub(crate)` and adding a `pub use prf::splitmix64;` at the
crate root) is slightly less direct and doesn't buy anything
real; the per-function visibilities make the surface
self-documenting.

If a future PR discovers that a fifth function needs
out-of-crate visibility, flipping it from `pub(crate)` to
`pub` is a one-line non-breaking change. The current pick
errs on the side of keeping the public surface minimal until a
caller asks for it.

### 2.6 PR 1a description and landing

PR 1a's description (the text the PR review page shows) is
short — additive-only PRs do not need extensive prose because
the diff speaks for itself:

```
Title: chassis: counter-based PRF module for stochastic components (PR 1a of Part 4)

Summary:
Adds sim/L0/thermostat/src/prf.rs, a new module holding counter-
based hash primitives used by the C-3 chassis refactor. Four of
the five functions are pub(crate) and will be consumed by PR 1b's
LangevinThermostat rewrite; the fifth (splitmix64) is pub for
downstream use by the Ch 42 rematch.

This PR is additive only. No existing file is modified beyond the
one-line `mod prf;` declaration in lib.rs. No tests are broken;
the new inline unit tests in prf.rs cross-verify chacha8_block
against rand_chacha 0.9.0 at 7 test coordinates.

References:
- ML chassis refactor study, Ch 15 §2 (why Route 2)
- ML chassis refactor study, Ch 32 §4.6 (why splitmix64)
- ML chassis refactor study, Ch 40 §2 (this PR's plan)

Test plan:
- cargo test -p sim-thermostat (all existing tests plus the 10
  new inline tests in prf.rs)
- cargo test -p sim-thermostat --features sim-core/parallel
  --lib prf:: (the cross-verification tests must also pass
  with sim-core's parallel feature enabled; propagation uses
  sim-core/parallel because sim-thermostat has no local
  `parallel` feature). The test surface is feature-independent,
  so both runs produce identical results.
```

**Landing notes:**

- PR 1a can merge the moment review approves. There is no
  dependency on any other PR in the study.
- The grade-tool run (`cargo xtask grade sim-thermostat`) must
  pass after PR 1a merges. `prf.rs` is new chassis code and
  should meet the same 7-criterion bar as the rest of the
  crate.
- If the cross-verification test fails on CI but passes
  locally, the most likely cause is a `rand_chacha` version
  pin drift — Part 4 implementation must verify
  `rand_chacha = "0.9.0"` at merge time and add a
  version-guard comment in `prf.rs` naming the pinned version.

PR 1a has **no rollback path** because it has zero consumers.
If a post-merge issue surfaces, the fix lands as a forward
patch in a follow-up PR or as a revert. No `LangevinThermostat`
behavior changes in PR 1a, so there is no correctness surface
to regress.

## Section 3 — PR 1b: LangevinThermostat rewrite + BatchSim rewire

PR 1b is the user-visible change that turns C-3 from a plan
into code. It touches the core of the thermostat crate
(`langevin.rs` rewrite), adds a new constructor path to
`BatchSim` in sim-core, adds a new regression test under the
`parallel` feature, and ripples the D2 constructor signature
change across every call site of `LangevinThermostat::new`
in the workspace. The diff is substantial — roughly three
orders of magnitude larger than PR 1a's — but the shape is
well-constrained by Ch 15 and D1–D15.

This section's job is to make that diff legible: name every
file that changes, count every call site, describe every
test update, and surface the one in-chapter sub-decision
(§3.3's additive-vs-destructive call on the `BatchSim`
rewire). After §3.6, a reviewer should be able to open PR 1b
and see each change on the list.

### 3.1 File-level diff: what changes and how much

PR 1b's diff spans four directories and roughly a dozen
files. Table 2 enumerates them:

| Crate / file | Change type | Call-site count | Notes |
|---|---|---|---|
| `sim/L0/thermostat/src/langevin.rs` | rewrite | — | Struct fields, constructor, `apply`, `Diagnose` impl, module-level doc, ~18 internal test call sites. D2 + D7 + D8 + D13. |
| `sim/L0/thermostat/src/ising_learner.rs` | call-site update | 1 | Production caller at `:198` (recon-reported) adopts the D2 4-arg constructor. |
| `sim/L0/thermostat/src/double_well.rs` | doctest update | 1 | Doctest example at `:43` uses the D2 signature. |
| `sim/L0/thermostat/src/pairwise_coupling.rs` | doctest update | 1 | Doctest example at `:54` uses the D2 signature. |
| `sim/L0/thermostat/src/lib.rs` | doctest update | 1 | Crate-root doctest example at `:45` uses the D2 signature. |
| `sim/L0/thermostat/tests/langevin_thermostat.rs` | call-site update | 6 | D11-reused fixture. One of these tests may grow the new regression test body; see §3.4. |
| `sim/L0/thermostat/tests/multi_dof_equipartition.rs` | call-site update | 4 | Mechanical D2 ripple. |
| `sim/L0/thermostat/tests/kramers_escape_rate.rs` | call-site update | 3 | Mechanical. |
| `sim/L0/thermostat/tests/d1a_ratchet_potential.rs` | call-site update | 3 | Mechanical. |
| `sim/L0/thermostat/tests/d1b_brownian_ratchet_baselines.rs` | call-site update | 2 | Mechanical. |
| `sim/L0/thermostat/tests/d1c_cem_training.rs` | call-site update | 2 | Mechanical. |
| `sim/L0/thermostat/tests/d1d_reinforce_comparison.rs` | call-site update | 2 | Mechanical. |
| `sim/L0/thermostat/tests/d2a_stochastic_resonance_components.rs` | call-site update | 4 | Mechanical. |
| `sim/L0/thermostat/tests/d2b_stochastic_resonance_baselines.rs` | call-site update | 3 | Mechanical. |
| `sim/L0/thermostat/tests/d2c_cem_training.rs` | call-site update | 2 | Mechanical. Ch 42's splitmix64 adoption lands separately. |
| `sim/L0/thermostat/tests/coupled_bistable_array.rs` | call-site update | 1 | Mechanical. |
| `sim/L0/thermostat/tests/boltzmann_learning.rs` | call-site update | 1 | Mechanical. |
| `sim/L0/thermostat/tests/gibbs_sampler.rs` | call-site update | 1 | Mechanical. |
| `sim/L0/core/src/batch.rs` | additive constructor + field growth | — | §3.3's additive `new_per_env` constructor. Existing `new(model, n)` retained. |
| `sim/L0/tests/integration/batch_sim.rs` | new regression test | — | `parallel_matches_sequential_with_langevin` lives here, alongside the existing `batch_matches_sequential_with_contacts` at `:55` (recon-reported). The owning crate is `sim-conformance-tests` (per `sim/L0/tests/Cargo.toml:2`), not the informal name "sim-integration" used elsewhere. D9 + D10 + D11 + D15. |
| `sim/L0/tests/Cargo.toml` | dev-dep + feature addition | — | Adds `sim-thermostat` as a dev-dependency (absent today) and adds a `parallel` feature forwarding to `sim-core/parallel`, both required to build the new regression test. Session-13 recon drift fix — Ch 40 initial draft assumed both were already present. |
| `sim/L0/thermostat/Cargo.toml` | dependency demotion | — | `rand_distr` moves from runtime deps to `[dev-dependencies]` (after the Langevin rewrite its only `src/` consumer is gone; `d1b` / `d1c` test files still use it). `rand_chacha` STAYS in runtime deps because `gibbs.rs` is a second runtime consumer of `ChaCha8Rng` — `GibbsSampler` is a discrete sampler with legal `&mut self`, not a `PassiveComponent`, and is out of C-3's scope. Session-13 recon drift fix — Ch 40 initial draft assumed `langevin.rs` was `rand_chacha`'s only `src/` consumer. |

The call-site column totals **56** `LangevinThermostat::new`
sites across 18 files. Most are mechanical (a 3-arg call
becomes a 4-arg call, with the new argument being either `0`
for single-env tests or `env_idx` for per-env tests). The
non-mechanical sites are the handful in `tests/langevin_thermostat.rs`
and `tests/multi_dof_equipartition.rs` that use
`LangevinThermostat` inside a custom passive wrapper for
coupled simulations — those need a careful read to make sure
the `traj_id` semantics are preserved.

**What does not change in PR 1b:**

- `sim/L0/thermostat/src/stack.rs` — `PassiveStack`,
  `PassiveStackBuilder`, `install`, `install_per_env`,
  `EnvBatch`, `StochasticGuard`. The chassis hooks C-3 was
  designed to sit on top of are already in place; PR 1b does
  not add to them.
- `sim/L0/thermostat/src/component.rs` — the
  `PassiveComponent` and `Stochastic` traits. Both are
  preserved unchanged, as Ch 15 §4 committed.
- `sim/L0/core/src/types/*.rs` — no `Data` invariant changes
  (hosting A was eliminated on exactly this ground), no
  `Model` changes, no callback changes.
- `sim/L0/ml-bridge/src/vec_env.rs` — the existing
  `BatchSim::new(Arc<Model>, usize)` path is retained, so
  `VecEnv` continues to use it at `:391` (recon-reported)
  unchanged. See §3.3.
- `sim/L0/core/benches/batch_benchmarks.rs` and
  `sim/L0/ml-bridge/benches/bridge_benchmarks.rs` — both
  continue to use `BatchSim::new(model, n)` and are
  unmodified by PR 1b.
- All `examples/` targets that use `BatchSim::new(model, n)` —
  unchanged.
- `sim/L1/bevy/src/multi_scene.rs` — the `BatchSim` consumer
  at `:204-215` (recon-reported) is a read-only iterator over
  the batch's envs and does not care about the constructor.
  Unchanged.

The "what does not change" list is part of the PR description
precisely because a reviewer seeing a 20-file diff needs to
know what the non-stochastic surface was deliberately left
alone. §3.3 is where that deliberate-leave-alone is argued.

### 3.2 The `LangevinThermostat` rewrite

The rewrite replaces the mutex-bearing shape with the
counter-bearing shape. The new struct fields are:

```rust
pub struct LangevinThermostat {
    gamma: DVector<f64>,
    k_b_t: f64,
    master_seed: u64,   // D7: u64 for user-facing display + D8 diagnostic
    master_key: [u8; 32],  // D7: 32-byte key, expanded once at construction
    traj_id: u64,
    counter: AtomicU64,
    stochastic_active: AtomicBool,
    k_b_t_ctrl: Option<usize>,  // unchanged from current code
}
```

The `master_seed: u64` field is retained alongside
`master_key: [u8; 32]` because D8 specifies that
`diagnostic_summary` reports the user-facing `master_seed`,
not the expanded key. Holding both is ~40 bytes of overhead
per instance, which at 32 envs is 1280 bytes — negligible.
The alternative (reconstructing the `u64` from the key for
display) is lossy because the key space is larger than the
seed space, so holding both is the only honest choice.

The D2 constructor signature is:

```rust
impl LangevinThermostat {
    /// Construct a thermostat with per-DOF damping coefficients
    /// `gamma`, bath temperature `k_b_t`, master seed
    /// `master_seed`, and trajectory id `traj_id`. The master
    /// seed is expanded once at construction into a 32-byte
    /// ChaCha8 key; the thermostat's noise stream at step `s`
    /// for DOF `d` is a pure function of
    /// `(master_key, traj_id, s, d)`.
    ///
    /// `traj_id` is typically the env index under an
    /// `install_per_env` factory, but it can be any `u64` —
    /// distinct `traj_id` values at the same `master_seed`
    /// produce disjoint noise streams by the PRF's
    /// construction.
    #[must_use]
    pub fn new(
        gamma: DVector<f64>,
        k_b_t: f64,
        master_seed: u64,
        traj_id: u64,
    ) -> Self {
        Self {
            gamma,
            k_b_t,
            master_seed,
            master_key: prf::expand_master_seed(master_seed),
            traj_id,
            counter: AtomicU64::new(0),
            stochastic_active: AtomicBool::new(true),
            k_b_t_ctrl: None,
        }
    }

    /// Enable runtime temperature modulation via a ctrl channel.
    /// (Unchanged from current code; moved here for exposition.)
    #[must_use]
    pub const fn with_ctrl_temperature(mut self, ctrl_idx: usize) -> Self {
        self.k_b_t_ctrl = Some(ctrl_idx);
        self
    }
}
```

The `apply` method transcribes Ch 15 §4's sketch:

```rust
impl PassiveComponent for LangevinThermostat {
    fn apply(&self, model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
        let h = model.timestep;
        let n_dofs = self.gamma.len();
        let k_b_t = self.effective_k_b_t(data);

        // Damping runs unconditionally — deterministic half of
        // the FD pair, written under both stochastic-active and
        // stochastic-inactive states (Ch 15 §4).
        for i in 0..n_dofs {
            qfrc_out[i] += -self.gamma[i] * data.qvel[i];
        }

        // Gating early-return before the counter advance. This
        // is load-bearing for the FD invariant at `apply_does_
        // not_advance_rng_when_stochastic_inactive`: no counter
        // advance under disable_stochastic ⇒ post-re-enable PRF
        // coordinate equals pre-disable PRF coordinate.
        if !self.stochastic_active.load(Ordering::Relaxed) {
            return;
        }

        // Counter advance (one per apply call) + PRF over DOF
        // groups of 8. D5 general case: loop over groups; the
        // single-block special case (n_dofs ≤ 8) is the common
        // path and does one iteration.
        let step_index = self.counter.fetch_add(1, Ordering::Relaxed);
        let base_counter = prf::encode_block_counter(self.traj_id, step_index);

        let n_groups = n_dofs.div_ceil(8);
        for group in 0..n_groups {
            let block = prf::chacha8_block(
                &self.master_key,
                base_counter.wrapping_add(group as u64),
            );
            let gaussians = prf::box_muller_from_block(&block);
            let dof_start = group * 8;
            let dof_end = (dof_start + 8).min(n_dofs);
            for dof in dof_start..dof_end {
                let gamma_i = self.gamma[dof];
                let sigma = (2.0 * gamma_i * k_b_t / h).sqrt();
                qfrc_out[dof] += sigma * gaussians[dof - dof_start];
            }
        }
    }

    fn as_stochastic(&self) -> Option<&dyn Stochastic> {
        Some(self)
    }
}
```

Note the `base_counter.wrapping_add(group as u64)` — this
advances the block counter within one apply call, one block
per DOF group of 8, without touching the stored
`self.counter`. The stored counter advances exactly once per
apply (matching the current mutex-bearing code's "one lock
acquisition per apply" invariant documented at
`langevin.rs:134-139`, recon-reported). The 32/32 layout means each `(traj_id,
step_index)` pair has 4 billion block-counter slots reserved
for within-apply iteration — vastly more than the ceiling for
any realistic DOF count, so the `wrapping_add` is defensive
against `u64::MAX` wraparound at the high end rather than a
live concern.

The `Stochastic` impl is unchanged (Ch 15 §4 point 2):

```rust
impl Stochastic for LangevinThermostat {
    fn set_stochastic_active(&self, active: bool) {
        self.stochastic_active.store(active, Ordering::Relaxed);
    }
    fn is_stochastic_active(&self) -> bool {
        self.stochastic_active.load(Ordering::Relaxed)
    }
}
```

The `Diagnose` impl changes per D8:

```rust
impl Diagnose for LangevinThermostat {
    fn diagnostic_summary(&self) -> String {
        match self.k_b_t_ctrl {
            None => format!(
                "LangevinThermostat(kT={:.6}, n_dofs={}, \
                 master_seed={}, traj_id={})",
                self.k_b_t,
                self.gamma.len(),
                self.master_seed,
                self.traj_id,
            ),
            Some(idx) => format!(
                "LangevinThermostat(kT={:.6}, n_dofs={}, \
                 master_seed={}, traj_id={}, ctrl_temp={})",
                self.k_b_t,
                self.gamma.len(),
                self.master_seed,
                self.traj_id,
                idx,
            ),
        }
    }
}
```

D8 specifies a static-only tuple — the counter is runtime
state and deliberately excluded. A reader using
`diagnostic_summary` for logging or test-failure messages
gets the configuration-identifying fields, not the
runtime-varying counter.

**D13 stale-comment removal.** The current code at
`langevin.rs:176-183` (recon-reported) contains the comment
block that ends with `"unwrap_or_else(PoisonError::into_inner)
is the std idiom for 'recover the inner value if poisoned,
proceed anyway.'"` — this comment discusses the
`Mutex<ChaCha8Rng>` lock pattern that no longer exists under
C-3. D13 removes it entirely. No replacement comment is added;
the new `apply` method's structure speaks for itself.

**Mechanical ripple: module-level documentation.** The current
`langevin.rs:26-42` (recon-reported) contains the "## RNG and
`cb_passive`" section, which discusses the mutex pattern and
the poison-recovery idiom. This section is also stale under
C-3 and must be rewritten as part of PR 1b. D13 names only
the `:176-183` block, but the module-level doc is the same
material at a different file location. The rewrite replaces
the entire module-level doc with a new framing that names the
counter-based PRF pattern and cross-references `prf.rs`:

```rust
//! ## RNG and `cb_passive`
//!
//! Under the C-3 chassis refactor (study Ch 15), the
//! thermostat holds no mutable RNG state. Noise at step `s`
//! for DOF `d` is computed as
//!
//! ```text
//! block = chacha8_block(master_key, encode_block_counter(traj_id, s) + group)
//! z_d   = box_muller_from_block(block)[d - group*8]
//! ```
//!
//! where `master_key` is expanded once at construction from
//! the user-supplied `master_seed: u64`, `traj_id` is set per
//! env by an `install_per_env` factory, `s` is the component's
//! own `AtomicU64` counter (advanced once per apply call,
//! gated by `stochastic_active`), and `group = d / 8` handles
//! DOF counts above 8.
//!
//! Because the PRF is a pure function of integers, the
//! thermostat is structurally immune to the
//! parallel-vs-sequential reproducibility defect Ch 10 named:
//! two runs at the same `(master_seed, traj_id)` at the same
//! step index produce bit-identical noise regardless of
//! thread scheduling. The
//! `parallel_matches_sequential_with_langevin` regression test
//! at `sim/L0/tests/integration/batch_sim.rs` asserts this by
//! construction.
//!
//! See `sim_thermostat::prf` for the primitive module and
//! the study's Ch 15 §2 for the argument that Route 2 (the
//! manual ChaCha8 implementation used by `prf.rs`) is the
//! right PRF-implementation choice.
```

This rewrite is not a D13 item literally, but it is the same
class of change (stale mutex-era prose in `langevin.rs`) and
must land together to keep the file coherent. A reviewer
reading the PR 1b diff will see both the `:26-42` rewrite and
the `:176-183` removal, and should recognize them as the
same cleanup.

### 3.3 The `BatchSim` rewire — in-chapter call (d)

This is the biggest in-chapter call in Ch 40 and deserves
room. D1 says: *`BatchSim::new` signature | factory closure:
`new(n, factory)` where `factory: impl FnMut(usize) -> (Model,
Arc<PassiveStack>)`*. Three readings of D1 are possible, and
the recon surfaced that the literal reading is not the only
defensible rendering.

**Reading (i): destructive replacement.** `BatchSim::new`'s
current `(model: Arc<Model>, n: usize)` signature is literally
replaced with `(n: usize, factory: impl FnMut(usize) -> (Model,
Arc<PassiveStack>))`. Every existing caller of
`BatchSim::new(model, n)` must migrate. Non-stochastic
callers (VecEnv, benches, examples, integration tests, the
Bevy multi-scene consumer) wrap
`(model.clone(), empty_stack.clone())` in their factory
closure. One constructor shape, maximum API consistency.

**Reading (ii): additive second constructor.** The existing
`BatchSim::new(model: Arc<Model>, n: usize)` is retained
unchanged. A new constructor `BatchSim::new_per_env(n,
factory)` is added alongside it, invoking
`PassiveStack::install_per_env` internally and growing the
`BatchSim` struct to hold per-env `Model` instances.
Non-stochastic callers continue to use `new` unchanged.
Stochastic-aware callers (none today, `LangevinThermostat` +
`parallel_matches_sequential_with_langevin` after PR 1b) use
`new_per_env`. Two constructor shapes, zero collateral on
non-stochastic code.

**Reading (iii): phased destructive.** Both shapes coexist in
PR 1b. The original `new(model, n)` is marked `#[deprecated]`
pointing callers to `new_per_env` (or to a "shared-model"
variant). A future PR removes the old signature once every
caller migrates. One-and-a-half constructor shapes during
the transition; eventually converges to (i).

Ch 40 picks **reading (ii): additive second constructor**.
The argument is five observations:

**First, the non-stochastic call surface is large and has no
reason to change.** The recon enumerated the existing
`BatchSim::new(model, n)` callers across the workspace:

- `sim/L0/core/src/batch.rs` — ~20 internal unit-test sites
- `sim/L0/core/benches/batch_benchmarks.rs` — `:43`
  (recon-reported)
- `sim/L0/ml-bridge/src/vec_env.rs:391` (recon-reported)
- `sim/L0/ml-bridge/src/task.rs` — 5 test sites at
  `:480, :664, :944, :979, :1003` (recon-reported)
- `sim/L0/ml-bridge/benches/bridge_benchmarks.rs` — 3 sites
  (recon-reported)
- `sim/L0/tests/integration/batch_sim.rs` — 4 sites at
  `:59, :98, :131, :165` (recon-reported)
- `sim/L1/bevy/src/multi_scene.rs` — 1 reader site at
  `:204-215` (recon-reported)
- `examples/fundamentals/sim-cpu/batch-sim/reset-subset/src/main.rs:217`
- `examples/fundamentals/sim-cpu/batch-sim/parameter-sweep/src/main.rs`

None of these callers need per-env stacks. Every one of them
is either deterministic physics (contact resolution, reset
behavior, control sweeps) or benchmark infrastructure that
deliberately avoids stochastic components for reproducibility
of timing measurements. Forcing them to migrate to a factory
closure whose factory returns an empty `PassiveStack` is
gratuitous churn — it grows every call site by several
lines, introduces a `PassiveStack::builder().build()` pattern
at call sites that have no stack, and couples sim-core's
public API to the thermostat crate (which sim-core does not
depend on today, so the factory type's `PassiveStack` would
need to move or be generic).

Reading (i) hits this pattern hardest. The ripple is ~30
call-site updates across six crates, for zero correctness
benefit to non-stochastic callers. Reading (iii) is marginally
better — callers only need to rename their call, not rewrite
its arguments — but the eventual removal step is still
coming, and the `#[deprecated]` attribute would fire on every
caller that chooses not to migrate, generating warnings
indefinitely.

Reading (ii) sidesteps the whole migration. Non-stochastic
callers continue to use `new(model, n)` and see zero change.
Stochastic-aware callers use `new_per_env(n, factory)` and
get the chassis they need. The two paths coexist cleanly
because they solve different problems.

**Second, D1's literal text does not force reading (i).** D1
specifies the *shape* of the per-env-factory constructor —
its signature, its argument types, its closure return type.
It does not specify that the existing constructor must be
removed. A reader of the D-decision table can read D1 as
"the per-env path takes this shape" without reading it as
"the non-per-env path is eliminated." The session-4
conversation did not explicitly address non-stochastic
callers (the context at the time was the Langevin
refactor, not the `VecEnv` / bench / example surface), so
D1's scope is naturally the per-env path.

**Third, the `BatchSim` invariant stated at
`batch.rs:61-63` (recon-reported) is "All environments share
the same `Arc<Model>`" — a statement about the shared-model
path, not the per-env path.** The shared-model path and the
per-env path have genuinely different invariants: the first
says "one model, N data"; the second says "N models, N
data, each model's cb_passive is its own closure."
Merging them into one constructor would require one of the
two invariants to bend. Keeping them separate honors both.

**Fourth, the R34 chassis-overbuild philosophy prefers
"the chassis tolerates two clean paths" over "the chassis
forces one hybrid path."** The project memory's user
preferences include "readability and organization are the
highest priority" and "breaking changes that fix architecture
over non-breaking hacks." Reading (ii) is not a non-breaking
hack — it is two clean constructors for two genuinely
different use cases. The breaking-change preference does not
apply here because there is no architecture to fix: the
shared-model path is not broken, it is just not what
stochastic components need.

**Fifth, a caller that later becomes stochastic has a clean
migration path under reading (ii).** The legitimate follow-up
worry with additive is "what if VecEnv or one of the benches
later needs stochastic components?" The answer is that the
caller in question migrates its own call site from `new` to
`new_per_env`, which is a local change in one file — the same
change any caller has to make the first time it takes on
stochastic components under any of the three readings. Reading
(ii) does not close that door; it simply does not force every
caller through it on day one. The caller-becomes-stochastic
event is an event that happens to one file at a time, and the
migration cost belongs on that file's PR, not on PR 1b.

**Objection: reading (ii) locks in two shapes forever.** This
is the legitimate counter-argument. Two constructors means
two maintenance burdens, two code paths in `BatchSim::step_all`
(potentially), two documentation entries, two test surfaces.
If at some future point the per-env path becomes the only
path anyone wants, the old shared-model path lingers as dead
weight. The response to this objection is that the two shapes
encode two genuinely different use cases and the "only one
path anyone wants" future is speculative. If it does
materialize, a future PR can consolidate; meanwhile, the
additive shape is the cheapest option that does not create
collateral damage. The one-path-eventually hypothesis does
not justify paying the migration cost today.

**Pick: reading (ii), additive second constructor.**
`BatchSim`'s existing `new(model: Arc<Model>, n: usize)` is
preserved unchanged. A new constructor `new_per_env(n: usize,
factory: impl FnMut(usize) -> (Model, Arc<PassiveStack>))` is
added. The `BatchSim` struct grows to support both paths.

**What the struct looks like.** `BatchSim` today holds
`{ model: Arc<Model>, envs: Vec<Data> }`. Under reading (ii),
it grows to:

```rust
pub struct BatchSim {
    // Shared-model path: one Arc<Model> for all envs.
    // Populated by new(model, n); None under new_per_env.
    shared_model: Option<Arc<Model>>,

    // Per-env path: one Model per env with cb_passive already
    // installed. Populated by new_per_env(n, factory); empty
    // under new(model, n).
    per_env_models: Vec<Model>,

    // Per-env path: one Arc<PassiveStack> per env, retained
    // so callers can invoke disable_stochastic per env.
    // Empty under new(model, n).
    per_env_stacks: Vec<Arc<PassiveStack>>,

    // Always populated: one Data per env, indexed by env i.
    envs: Vec<Data>,
}
```

The two paths are discriminated by `shared_model.is_some()`.
`step_all` branches on this and calls `data.step(&model)`
with the appropriate model reference per env. `env`, `env_mut`,
`envs`, `envs_mut`, `reset`, `reset_where`, `reset_all` are
unchanged in signature — they operate on `self.envs` and do
not care which path populated the batch.

The `model()` accessor (at `batch.rs:92-94`, recon-reported)
returns `&Model` and its semantics shift slightly: under the
shared-model path it returns `&*self.shared_model.unwrap()`
(the one model); under the per-env path it returns
`&self.per_env_models[0]` (the first env's model). The latter
is sound because all per-env models in one `BatchSim` share
the same `Model` shape (same `nq`, `nv`, body tree), and a
reader calling `model()` for its static fields gets correct
values. The doc comment is updated to name this.

**The factory path.** `new_per_env` delegates to
`PassiveStack::install_per_env` by constructing a "prototype"
stack reference:

```rust
impl BatchSim {
    /// Create a batch of `n` environments, each constructed
    /// via a factory closure that returns a `(Model,
    /// Arc<PassiveStack>)` pair. Each env's model has its
    /// cb_passive already installed from its paired stack
    /// via `PassiveStack::install_per_env`.
    ///
    /// Use this constructor when the batch needs per-env
    /// stochastic components (e.g. `LangevinThermostat`
    /// under C-3). For deterministic-physics batches or
    /// any case where all envs share the same callback-free
    /// `Model`, prefer `BatchSim::new(model, n)`.
    #[must_use]
    pub fn new_per_env<F>(n: usize, factory: F) -> Self
    where
        F: FnMut(usize) -> (Model, Arc<PassiveStack>),
    {
        // Grab the first factory result to get a prototype
        // stack handle for install_per_env. install_per_env
        // takes &Arc<Self> as its receiver — we need any
        // Arc<PassiveStack> to call it.
        let (_, prototype) = factory(0);
        let batch = prototype.install_per_env(n, factory);
        let envs = batch
            .models
            .iter()
            .map(Model::make_data)
            .collect();
        Self {
            shared_model: None,
            per_env_models: batch.models,
            per_env_stacks: batch.stacks,
            envs,
        }
    }
}
```

**Wait.** The sketch above has a bug: calling `factory(0)`
once before passing `factory` to `install_per_env` would
build env 0 twice — once to grab the prototype, once inside
`install_per_env`'s loop. That is a correctness bug and must
be fixed in the real implementation, not documented as a
PR 1b TODO.

The clean fix is to expose a `PassiveStack` prototype
externally: the user constructs a prototype stack once and
hands it in, or `install_per_env` is restructured to take a
`&Arc<PassiveStack>` prototype reference separately from the
factory. The current `install_per_env` receiver at
`stack.rs:174` is `self: &Arc<Self>` with the prototype
unused inside the function body (noted at
`stack.rs:165-172`, recon-reported) — so the prototype is
passed in as a separate argument by the stack-builder-caller.

The cleanest rendering of `new_per_env` that matches the
current `install_per_env` shape:

```rust
impl BatchSim {
    #[must_use]
    pub fn new_per_env<F>(
        prototype: &Arc<PassiveStack>,
        n: usize,
        factory: F,
    ) -> Self
    where
        F: FnMut(usize) -> (Model, Arc<PassiveStack>),
    {
        let batch = prototype.install_per_env(n, factory);
        let envs = batch
            .models
            .iter()
            .map(Model::make_data)
            .collect();
        Self {
            shared_model: None,
            per_env_models: batch.models,
            per_env_stacks: batch.stacks,
            envs,
        }
    }
}
```

The caller's usage pattern becomes:

```rust
let prototype: Arc<PassiveStack> = PassiveStack::builder()
    .with(LangevinThermostat::new(...))
    .build();

let batch = BatchSim::new_per_env(&prototype, n_envs, |i| {
    let model = load_model(xml).unwrap();
    let stack = PassiveStack::builder()
        .with(LangevinThermostat::new(
            DVector::from_element(model.nv, gamma),
            k_b_t,
            master_seed,
            i as u64,   // traj_id = env index
        ))
        .build();
    (model, stack)
});
```

The prototype is discarded after the call (it is only used
as the receiver for `install_per_env`'s current shape); the
factory builds the real per-env pairs. This is slightly
awkward because the prototype duplicates information the
factory also produces, but it matches the existing
`install_per_env` signature and avoids restructuring chassis
machinery in PR 1b. A future PR can restructure
`install_per_env` to take the prototype as a plain argument
rather than a receiver, removing the awkwardness — but that
is out of Ch 40's scope.

**Fifth observation note.** The sketch-and-fix above is a
real bug that Ch 40 caught during drafting and reports here
so PR 1b's implementation does not repeat it. This is
exactly the kind of small correctness issue that a
PR-plan chapter exists to surface before the code is
written.

**sim-core depends on thermostat? No.** The `new_per_env`
signature above takes `&Arc<PassiveStack>` as an argument,
where `PassiveStack` is a thermostat-crate type. `sim-core`
does not depend on `sim-thermostat` today (the dependency
direction is the other way: `sim-thermostat` depends on
`sim-core` for `Model`, `Data`, `DVector`). Adding a direct
use of `PassiveStack` in `sim-core::batch` would invert the
dependency and break the build.

The fix is to make `new_per_env` generic over the stack type:

```rust
impl BatchSim {
    #[must_use]
    pub fn new_per_env<S, F>(
        prototype: &Arc<S>,
        n: usize,
        factory: F,
    ) -> Self
    where
        S: PerEnvStack,
        F: FnMut(usize) -> (Model, Arc<S>),
    { ... }
}

/// Trait implemented by `PassiveStack` (in the thermostat
/// crate) exposing the install_per_env surface sim-core
/// needs. Lives in `sim-core::batch` so sim-core owns the
/// trait definition; implemented externally in `sim-thermostat`.
pub trait PerEnvStack: Send + Sync + 'static {
    fn install_per_env<F>(self: &Arc<Self>, n: usize, build_one: F) -> EnvBatch<Self>
    where
        F: FnMut(usize) -> (Model, Arc<Self>);
}

pub struct EnvBatch<S> {
    pub models: Vec<Model>,
    pub stacks: Vec<Arc<S>>,
}
```

The trait lives in `sim-core::batch`, is implemented in
`sim-thermostat::stack` by `PassiveStack`, and keeps the
dependency direction sound. This grows sim-core's public
surface by one trait and one small struct, but the growth
is contained and self-documenting.

**This is a concrete correction that affects PR 1b's
implementation.** A reviewer looking at Ch 40 §3.3 should
expect PR 1b to land the `PerEnvStack` trait definition and
the `EnvBatch<S>` generic, not a direct `PassiveStack`
reference inside `sim-core`. This is additional chassis
work beyond the D1–D15 list, but it is a consequence of
D1's factory-closure-with-`Arc<PassiveStack>` shape colliding
with the `sim-core` / `sim-thermostat` dependency direction.
It was not named in the session-4 conversation because the
conversation was about the per-env path's shape, not about
its crate placement. Ch 40 is the first place the
cross-crate constraint surfaces.

**This is rendering, not deciding.** The `PerEnvStack` trait
and the `EnvBatch<S>` generic are not a fifth in-chapter
sub-decision — they are the minimum edit that makes D1's
locked factory shape implementable without inverting the
sim-core / sim-thermostat dependency direction. A reader
picking up Ch 40 later should not confuse the trait invention
with the four explicit sub-decisions in §4's table; the trait
is rendering force-multiplied by a cross-crate constraint
that D1 did not anticipate, not a fresh design call with
alternatives that Ch 40 weighed. If D1 had specified a factory
closure that did not mention `PassiveStack` — for example, a
closure returning `(Model, Arc<dyn AnyInstallable>)` where
`AnyInstallable` is a sim-core-owned trait — the rendering
would look different, and the trait invention would live at
that earlier layer. Since D1 specified `Arc<PassiveStack>`
directly, the trait invention has to live in the rendering
layer where the constraint first bites. §4's table notes
this explicitly.

**Alternative: put `new_per_env` in sim-thermostat.** Instead
of growing `sim-core::batch`, the new constructor could live
as a free function in `sim-thermostat::stack`:
`fn build_batch_sim_per_env(n, factory) -> BatchSim`. This
keeps `sim-core` untouched and avoids the trait-ownership
question entirely. The cost is that the stochastic batch
construction path is discovered via the thermostat crate's
public API rather than `BatchSim`'s, which is a modest
discoverability loss but not a correctness issue.

**Pick: grow `sim-core::batch` with the `PerEnvStack` trait
and the generic `new_per_env`.** Reasons: (1) the
constructor belongs on `BatchSim` because the result is a
`BatchSim`, and a reader looking at `BatchSim`'s API should
see all the ways to construct one; (2) the trait is small
and self-documenting; (3) the alternative pulls batch
construction out of `BatchSim`'s own file, which fragments
the `BatchSim` API surface. The dependency-inversion cost
(one trait definition in sim-core) is small relative to
the discoverability win.

### 3.4 Test surface updates

Three kinds of test changes land in PR 1b.

**(a) The new regression test.** A new integration test,
`parallel_matches_sequential_with_langevin`, lives at
`sim/L0/tests/integration/batch_sim.rs`, alongside the
existing `batch_matches_sequential_with_contacts` at `:55`
(recon-reported). D15 specifies that the new test lives
*alongside* the existing test, not as an update — so the
existing test's scope (parallel/sequential agree for
deterministic physics with contacts) stays unchanged, and the
new test's scope (parallel/sequential agree for stochastic
components) is a separate function. The two tests together
cover the stochastic / non-stochastic boundary; either one
failing localizes the regression to its respective side.

D9 picks **strategy A (feature-gate-per-test)**. The test
runs in two invocations: once under `cargo test` (default
features, sequential path) and once under `cargo test
--features parallel` (parallel path). Each invocation writes
a trace file containing the post-K-step `(qpos, qvel)` for
every env; a third test (or a companion assertion built into
the same function via a post-run file compare) loads both
trace files and asserts bit-identical.

D10 picks 3 seeds: `0`, `0xD06F00D_42`, `u64::MAX`. D11
picks the 1-DOF SHO fixture from the existing Phase 1
integration tests. The fixture MJCF is already usable in the
integration test surface (see the `apply_writes_pure_damping_
when_stochastic_inactive` test at `langevin.rs:279-308`,
recon-reported, for the existing SHO inline XML — the
regression test reuses this or factors it into a shared
helper).

The test's structure transcribes Ch 15 §5.3's sketch:

```rust
#[test]
fn parallel_matches_sequential_with_langevin() {
    let n_envs = 32;
    let n_steps = 1000;
    let gamma_val = 0.1;
    let k_b_t = 1.0;

    for master_seed in [0u64, 0xD06F00D_42, u64::MAX] {
        // Build two batches identically — same factory, same
        // master_seed, same env count. At the feature gate the
        // step_all path selects sequential or parallel.
        let prototype = PassiveStack::builder()
            .with(LangevinThermostat::new(
                DVector::from_element(1, gamma_val),
                k_b_t,
                master_seed,
                0,
            ))
            .build();

        let factory = |env_idx: usize| {
            let model = sim_mjcf::load_model(SHO_XML).unwrap();
            let stack = PassiveStack::builder()
                .with(LangevinThermostat::new(
                    DVector::from_element(model.nv, gamma_val),
                    k_b_t,
                    master_seed,
                    env_idx as u64,
                ))
                .build();
            (model, stack)
        };

        let mut batch = BatchSim::new_per_env(&prototype, n_envs, factory);
        for _ in 0..n_steps {
            batch.step_all();
        }

        // Dump per-env (qpos, qvel) to the feature-indexed
        // trace file. Under strategy A, the test writes
        // "trace-sequential-{master_seed}.bin" under default
        // features and "trace-parallel-{master_seed}.bin"
        // under --features parallel. The filename naming is
        // part of the test infrastructure and is finalized
        // during PR 1b implementation.
        dump_env_state(&batch, trace_file_path(master_seed));
    }
}

#[test]
#[cfg(feature = "parallel")]
fn parallel_and_sequential_traces_match_langevin() {
    // Runs only under --features parallel. Assumes the
    // default-features run has already written its traces.
    for master_seed in [0u64, 0xD06F00D_42, u64::MAX] {
        let seq = load_trace(&format!("trace-sequential-{master_seed}.bin"));
        let par = load_trace(&format!("trace-parallel-{master_seed}.bin"));
        assert_eq!(seq.len(), par.len());
        for (env_idx, (s, p)) in seq.iter().zip(par.iter()).enumerate() {
            assert_eq!(
                s.qpos, p.qpos,
                "env {env_idx} qpos divergence at master_seed {master_seed}",
            );
            assert_eq!(
                s.qvel, p.qvel,
                "env {env_idx} qvel divergence at master_seed {master_seed}",
            );
        }
    }
}
```

The CI recipe for this test is two cargo invocations in
sequence, with the assertions only running under the second:
first `cargo test -p sim-integration parallel_matches_sequential_with_langevin`,
then `cargo test -p sim-integration --features parallel` (which
runs both the dump-writer and the comparison). The trace files
live in `target/test-artifacts/` or equivalent and are cleaned
up between runs — PR 1b's implementation picks the exact path
and the cleanup strategy.

**(b) The D2 4-arg ripple.** 56 `LangevinThermostat::new`
call sites across 18 files update from 3-arg to 4-arg. The
update is mechanical: each call's third argument (the seed
`u64`) is renamed `master_seed` conceptually and a fourth
argument `traj_id: u64` is added. For single-env tests that
instantiate one thermostat, `traj_id = 0` is the convention
(matching "this is env 0 of a 1-env notional batch"). For
tests that build multiple thermostats (e.g.
`multi_dof_equipartition.rs` at `:188, :263, :346, :357`,
recon-reported), each thermostat gets a distinct `traj_id`
matching its logical index in the test's own loop.

A handful of call sites deserve manual attention rather than
mechanical replacement:

- `tests/multi_dof_equipartition.rs:188, :263` (per-trajectory
  `for i in 0..n_traj` loops that build one fresh thermostat
  per trajectory with `seed_base + i as u64`): these are the
  genuine "distinct `traj_id` matching loop index" case. The
  mechanical rendering splits `seed_base + i` into
  `(master_seed = seed_base, traj_id = i as u64)`, preserving
  per-trajectory decorrelation.
- `tests/multi_dof_equipartition.rs:346, :357` (the
  `assert_multi_dof_reproducibility` helper — two SEPARATE
  simulations built with identical parameters, asserting
  bit-for-bit `qpos` / `qvel` agreement after N steps): both
  thermostats must receive IDENTICAL `(master_seed, traj_id)`
  pairs — `traj_id = 0` for both is fine and correct.
  Distinct `traj_id`s would break the reproducibility
  assertion. (Session-13 drift fix: Ch 40 initial draft
  misread these as "two thermostats in the same test coupled
  to a single simulation." They are two parallel builds of
  the same simulation, not one coupled simulation.)
- `tests/langevin_thermostat.rs:323, :334` (the
  `test_reproducibility_from_seed` test — same structure as
  the `:346/:357` pair above, two parallel builds asserting
  bit-identity): both get identical `(master_seed, traj_id
  = 0)`. This pair was not flagged in the initial draft;
  session-13 recon added it.
- `src/ising_learner.rs:198` (the production caller,
  recon-reported): `ising_learner.rs` is itself under the
  thermostat crate and its caller is whatever code
  instantiates the learner. `traj_id` must be threaded in
  from that outer context. If no caller exists yet,
  `traj_id = 0` is the default during construction and the
  learner gains a setter or builder method for cases where
  it matters.

(`tests/langevin_thermostat.rs:282`, the `CountingWrapper`
custom passive wrapper, was listed as non-mechanical in the
initial draft but is actually a single-thermostat wrapper
in a callback-firing-count test — the mechanical rendering
is `traj_id = 0`. Session-13 drift fix.)

**(c) The FD-invariant test update.** The existing
`apply_does_not_advance_rng_when_stochastic_inactive` at
`langevin.rs:311-363` (recon-reported) currently asserts
"after two inactive applies and one re-enable, the first
active noise sample equals a fresh thermostat's first noise
sample." Under C-3 the assertion shape is the same — first
post-re-enable sample equals a fresh thermostat's first
sample — but the operational construction differs: the
fresh thermostat is now a `LangevinThermostat::new` with the
same `master_seed, traj_id` pair, and the equality check is
a direct `assert_eq!` on the two `f64` outputs. The update
is ~10 lines of change and the test's semantic contract
(the FD-invariant) is unchanged.

Similarly,
`new_initializes_stochastic_active_to_true`,
`set_stochastic_active_roundtrips`, and
`as_stochastic_returns_some_self` at
`langevin.rs:241-266` (recon-reported) all update to the
D2 4-arg constructor (adding `traj_id = 0` to each call)
but their semantic content is unchanged.
`diagnostic_summary_format_matches_spec` at
`langevin.rs:269-276` updates to the D8 tuple format.

### 3.5 PR 1b description and landing order

PR 1b's description is longer than PR 1a's because the
surface is larger. A draft:

```
Title: chassis: LangevinThermostat rewrite + BatchSim per-env constructor (PR 1b of Part 4)

Summary:
Implements the C-3 chassis pick from study Ch 15. Three changes:

1. LangevinThermostat no longer holds Mutex<ChaCha8Rng>. The
   struct fields become (gamma, k_b_t, master_seed, master_key,
   traj_id, counter: AtomicU64, stochastic_active, k_b_t_ctrl).
   The apply method computes noise from prf::chacha8_block
   keyed on master_key at block counter
   encode_block_counter(traj_id, step_index). Gating is
   preserved by an early-return before the counter's fetch_add.

2. BatchSim gains a new constructor:
     BatchSim::new_per_env(prototype, n, factory)
   where factory: FnMut(usize) -> (Model, Arc<S: PerEnvStack>).
   The existing BatchSim::new(model, n) is unchanged — all
   non-stochastic callers (VecEnv, benches, examples,
   multi_scene, deterministic-physics integration tests)
   continue to use it.

3. New integration test
     parallel_matches_sequential_with_langevin
   at sim/L0/tests/integration/batch_sim.rs, exercised under
   strategy A (feature-gate-per-test) across three seeds.
   Lives alongside the existing
   batch_matches_sequential_with_contacts test.

Collateral:
- Constructor signature ripple (D2): 56 call sites across
  18 files updated from new(gamma, k_b_t, seed) to
  new(gamma, k_b_t, master_seed, traj_id). Most are 1-line
  mechanical updates in the thermostat crate's own tests.
- Module-level rewrite of langevin.rs documentation to
  remove mutex-era prose. Stale comment at
  langevin.rs:176-183 removed entirely (D13).
- sim-core::batch gains a small PerEnvStack trait (dependency
  direction preserved: sim-core does not import
  sim-thermostat). PassiveStack implements it in stack.rs.
- langevin.rs's runtime use of rand_distr is dropped
  (replaced by prf::box_muller_from_block). rand_distr
  moves from [dependencies] to [dev-dependencies]; the
  d1b / d1c test files still use it for Uniform action
  sampling. rand_chacha STAYS in [dependencies] because
  gibbs.rs is a separate runtime consumer of ChaCha8Rng
  (out of C-3 scope — GibbsSampler is a discrete sampler
  with legal &mut self, not a PassiveComponent).

References:
- ML chassis refactor study, Ch 15 (C-3 pick + Route 2)
- ML chassis refactor study, Ch 40 §3 (this PR's plan)
- Session-4 D1–D15 decision table (reproduced in Ch 40 §1.2)

Test plan:
- cargo test -p sim-thermostat (all existing tests with D2
  updates applied, plus ~3 updated FD-invariant tests)
- cargo test -p sim-core (BatchSim existing tests unchanged
  + new tests for BatchSim::new_per_env construction)
- cargo test -p sim-conformance-tests --test integration
  batch_sim (the integration-test crate is
  sim-conformance-tests per sim/L0/tests/Cargo.toml, not
  sim-integration — session-13 drift fix)
- cargo test -p sim-conformance-tests --test integration
  --features parallel batch_sim (second pass for strategy
  A trace comparison; the parallel feature is added to
  sim-conformance-tests' Cargo.toml by PR 1b itself)
- cargo xtask grade sim-thermostat (7-criterion bar)
- cargo xtask grade sim-core (7-criterion bar)

Rollback:
- git revert <this PR>. PR 1a stays merged; prf.rs becomes
  inert-but-available for the rewrite attempt. No database
  or file-format state has been changed, so rollback is clean.
```

**Landing order:**

- **PR 1a lands first.** No other ordering is defensible:
  PR 1b imports `prf::chacha8_block`, and the import must
  resolve at merge time.
- **PR 1b lands second.** Between PR 1a merging and PR 1b
  merging, there is no urgent timeline — PR 1a is additive
  and inert, so it can sit in the main branch for days or
  weeks without consequence.
- **Ch 41 and Ch 42 are not blocked by PR 1b.** Ch 41 (PR 2,
  Competition replicates + algorithm surface fixes) is
  independent of the `LangevinThermostat` refactor; its
  work is in `sim-ml-bridge` / `sim-rl` / `sim-opt` and
  does not touch the thermostat crate's noise-generation
  path. Ch 42 (PR 3, sim-opt split + rematch) does depend
  on splitmix64 from PR 1a and on the per-env BatchSim
  constructor from PR 1b, but it inherits from both through
  the normal import path rather than through a sequencing
  constraint.

### 3.6 Rollback path and in-flight revision risk

**Rollback from PR 1b.** If a post-merge issue surfaces in
PR 1b — for example, the regression test passes in CI but
fails on a specific platform, or a performance regression
surfaces in a bench that Ch 40 did not anticipate — the
rollback is `git revert <PR 1b merge commit>`. PR 1a's
`prf.rs` module remains merged but unused; no caller
references it between the revert and the follow-up rewrite,
and the cross-verification test in `prf.rs` continues to
pass on every `cargo test -p sim-thermostat` run.

No persisted state changes during PR 1b — no database
migrations, no file-format changes, no serialized-output
formats. The revert is clean.

**In-flight revision risk for PR 1b.** Because PR 1b is the
larger of the two PRs and touches more file surfaces, there
is non-trivial risk that review surfaces issues requiring
structural revision. Two categories of risk deserve naming:

- **Small-to-medium structural revisions.** A reviewer might
  flag that `new_per_env`'s receiver pattern is awkward
  (the `prototype` argument duplicates what the factory
  produces), or that the `PerEnvStack` trait should be in
  `sim-core::types` rather than `sim-core::batch`, or that
  the trace-file I/O in the regression test is too
  implementation-specific. These are real findings and PR 1b
  should be expected to absorb 1–3 rounds of small
  structural revision.
- **Large structural revisions.** A reviewer might flag
  that reading (ii)'s dual-constructor approach in §3.3 is
  wrong and reading (i) or (iii) is better. This is the
  one-in-three scenario where Ch 40's in-chapter call (d)
  is overruled during PR 1b's review. If it happens, the
  path forward is to pause PR 1b, patch Ch 40 §3.3 with
  the new argument (a third post-commit-patch precedent
  after 3e1ec0ff and 6b876bc5), and rewrite PR 1b's
  `BatchSim` changes to match. This is the largest in-flight
  revision risk and would add a session of work. **PR 1a
  is unaffected** by any reading-(ii) overrule — `prf.rs`
  does not reference `BatchSim` or any constructor pattern
  and its correctness surface is self-contained. If PR 1a
  has already merged when the overrule happens (the expected
  timing), no rollback of PR 1a is required, and the
  `PerEnvStack` trait landing simply shifts from PR 1b to
  whichever rewrite PR 1b becomes.

**Risk mitigation in the PR 1b description.** The PR
description should link to Ch 40 §3.3 explicitly and name
the reading (ii) pick, so a reviewer who disagrees can
surface the disagreement early rather than re-discover it
several rounds in. A PR description that pre-empts the
objection is cheaper than a review-round that re-surfaces it.

## Section 4 — In-chapter sub-decisions

Ch 40 made four in-chapter sub-decisions — the calls D1–D15
did not lock and Ch 15 deferred to Part 4. Table 3 names
them, records the pick, and summarizes the reasoning in one
line. A reader can use this table as a quick index into
Ch 40's sections.

| # | Sub-decision | Pick | One-line rationale |
|---|---|---|---|
| (a) | Cross-verification test location | inline unit tests in `prf.rs` | PR 1a has zero consumers; inline tests match the existing thermostat-crate pattern for single-component modules and give test access to module internals without gratuitous `pub(crate)` spills. |
| (b) | splitmix64 in `prf.rs` | yes | Lands the primitive alongside the other counter-based hash functions so Ch 42's rematch can call it without inventing a private copy; the "counter-based hash toolbox" framing covers both noise generation and seed derivation. |
| (c) | `prf.rs` visibility | `pub mod prf;` at crate root; `pub(crate)` on the four in-crate-only helpers (`chacha8_block`, `expand_master_seed`, `encode_block_counter`, `box_muller_from_block`); `pub` on `splitmix64` | Out-of-crate callers only need `splitmix64` (Ch 42). The four helpers stay minimal-surface until a concrete out-of-crate caller asks for them. |
| (d) | `BatchSim::new` rewire | additive second constructor `new_per_env` alongside unchanged `new` | Non-stochastic callers (VecEnv, benches, examples, deterministic-physics tests) have no reason to change and the migration would be pure churn; the two paths encode genuinely different use cases and coexist cleanly. |

Sub-decision (d) is the one to watch during PR 1b review. If
a reviewer disagrees with reading (ii), §3.3 can be patched
post-commit with the alternative argument; the 3e1ec0ff and
6b876bc5 precedents establish the pattern for narrow
amendments to a committed study chapter when downstream
implementation surfaces a framing issue.

**What Ch 40 does *not* treat as sub-decisions.** The
following are mentioned in Ch 40 but inherited wholesale
from Ch 15 or the D1–D15 table, and a reader should *not*
read them as in-chapter calls:

- The C-3 pick itself (Ch 15 §1).
- Route 2 for the PRF implementation (Ch 15 §2).
- The `prf.rs` module name (D3).
- The 32/32 bit layout (D4). Ch 40 §2.2 describes it
  as a "pickup" from D4, not a fresh call.
- The direct 4-arg constructor signature (D2).
- The `master_key: [u8; 32]` stored field (D7).
- The static `diagnostic_summary` tuple (D8).
- Strategy A for the regression test (D9).
- The 3 seeds (D10).
- The 1-DOF SHO fixture (D11).
- The stale-comment removal (D13).
- The 2-PR split (D14).
- The new-regression-test-lives-alongside-existing pattern
  (D15).
- **The `PerEnvStack` trait and `EnvBatch<S>` generic
  introduced in §3.3.** These are rendering force-multiplied
  by a cross-crate dependency constraint D1 did not
  anticipate, not a fresh design call. The trait is the
  minimum edit that makes D1's `Arc<PassiveStack>` factory
  shape implementable without inverting the sim-core /
  sim-thermostat dependency direction. See §3.3's "This is
  rendering, not deciding" paragraph for the longer framing.

When a PR 1b reviewer challenges any of these, the correct
response is to point at the owning artifact (Ch 15, the
session-4 conversation, the D-table, or §3.3 for the trait),
not to re-argue it in PR 1b's review comments.

## Section 5 — What Ch 40 does not decide

Ch 40 renders PR 1, not PR 2 or PR 3. It also defers a handful
of calls that surfaced during drafting but are out of scope.

**Ch 41 (PR 2) scope.** The Competition replicates + algorithm
surface fixes work is owned by Ch 41. Ch 40 makes no claim
about `Competition::run_replicates`'s signature, no claim
about the `best_reward` / `mean_reward` reconciliation picked
in Ch 24, no claim about the algorithm-surface per-replicate
reduction shape. Any reviewer reading Ch 40 who expects
answers to those questions is reading the wrong chapter.

**Ch 42 (PR 3) scope.** The sim-opt split + rematch is owned
by Ch 42. Ch 42 will use splitmix64 from `prf.rs` (shipped by
Ch 40's PR 1a) and will use `BatchSim::new_per_env` with
`LangevinThermostat` factories (shipped by Ch 40's PR 1b),
but the shape of the rematch's analysis pipeline, the
folded-pilot protocol's implementation, the bimodality
contingency check, the CI reporting format, and all the
other Ch 32 decisions are rendered in Ch 42, not here.

**Trait formalization for a second stochastic component.**
D12 defers this explicitly. Ch 40 does not introduce a
`CounterBasedStochastic` trait, does not formalize the
"stochastic component calls `prf::chacha8_block`" convention
in the type system, and does not grow `prf.rs` with any
trait that second-stochastic-component authors would
implement. The convention is documented prose; the trait is
future work pending a concrete second component.

**The exact filenames for the regression test's trace
files.** §3.4 describes the shape of the feature-gated
trace-file dance but leaves the exact path and cleanup
strategy to the PR 1b implementation. Options include
`target/test-artifacts/langevin-trace-*.bin`,
`$TMPDIR/sim-thermostat-trace-*.bin`, or an in-test
temporary directory via `tempfile::TempDir`. All three are
defensible; the pick is a PR 1b implementation detail that
does not affect the test's semantic contract.

**The exact `Cargo.toml` dep changes for the thermostat
crate.** PR 1b drops `rand_chacha` and `rand_distr` from
`[dependencies]` and retains them in `[dev-dependencies]`
for the cross-verification test. The exact edit is
mechanical but worth flagging so PR 1b's reviewer is not
surprised to see dependency-removal lines in the diff.

**The `PerEnvStack` trait's exact API surface.** §3.3
sketches the trait as a single method that wraps
`install_per_env`, but the final shape — whether the
`EnvBatch<S>` associated type lives on the trait or is a
free-standing generic struct, whether the method is called
`install_per_env` or `install_batched`, whether
`PassiveStack`'s impl uses `self: &Arc<Self>` or a plain
`&self` — is a PR 1b implementation detail. The shape above
is a reasonable starting point, not a specification.

**Restructuring `install_per_env` to take the prototype as an
explicit argument rather than a receiver.** The current
`install_per_env` at `stack.rs:174` (recon-reported) uses the
`self: &Arc<Self>` receiver pattern with the prototype
unused inside the function body (documented at
`stack.rs:165-172`), which forces `BatchSim::new_per_env`
callers to construct a prototype `PassiveStack` before the
real factory builds the per-env stacks (§3.3's usage-pattern
sketch shows the awkwardness). A cleaner signature would be
`install_per_env(n: usize, mut build_one: F) -> EnvBatch`
as a plain free function or a receiver-less method, with no
prototype argument at all. This restructuring is out of PR 1b
scope — PR 1b should not grow to include chassis-API changes
to `install_per_env` itself — but is a natural future-PR
cleanup once PR 1b's per-env path has settled and a second
stochastic-aware caller (or a second call-site pattern)
forces the question. Flagging it here so the awkwardness
is recognized as intentional deferral rather than an
oversight.

**Post-commit patch to Ch 15 §5.6.** During recon Ch 40
surfaced that Ch 15 §5.6 cites
`batch_matches_sequential_with_contacts` as living at
`sim/L0/thermostat/src/stack.rs`, when it actually lives at
`sim/L0/tests/integration/batch_sim.rs:55`. This is a factual
slip, not a decision error, and the fix is a one-line
citation update. Following the `3e1ec0ff` and `6b876bc5`
precedents (downstream chapter commits first, then a narrow
patch to the upstream chapter lands as a separate follow-up
commit), the correct sequencing is: Ch 40 commits, then a
narrow Ch 15 §5.6 patch lands next to update the citation.
Ch 40 does not modify Ch 15 in its own commit. The patch is
a one-line citation update and does not affect any argument
in the chapter. **Note:** the Ch 40 draft initially also
named Ch 11 as needing the same patch, but a second recon
read during the immediate follow-up session found Ch 11 at
`:267-269` correctly cites the test at
`sim/L0/tests/integration/batch_sim.rs` lines 54–91 — Ch 11
is fine. The Ch 15 slip is self-inflicted, not inherited.

**What Ch 40 does decide,** to close the chapter: the shape
of PR 1a (additive `prf.rs` module with five functions, four
`pub(crate)` and one `pub`, inline unit tests against
`rand_chacha`), the shape of PR 1b (`LangevinThermostat`
rewrite + additive `BatchSim::new_per_env` constructor +
new regression test alongside the existing contacts test +
56-call-site ripple), the order in which they land (1a
first, 1b second), and the rollback plan if either has to
back out. Part 4's execution layer begins here.

**Session 12 post-commit patches to §2.1 and §2.6.** During
the spec-vs-shipped audit that followed PR 1a's landing at
`e6ff35e4`, two narrow prose drifts in Ch 40 surfaced. §2.1
claimed `rand_chacha` was "already a dev-dependency of the
crate" — it is actually a runtime dependency at
`thermostat/Cargo.toml:19` (workspace-pinned at
`rand_chacha = "0.9"`), and PR 1b will be the commit that
demotes it. §2.6's second test-plan bullet specified
`--features parallel` as the dual-run invocation, but the
literal command errors with "package does not contain this
feature: parallel" because `parallel` is a feature on
`sim-core`, not on `sim-thermostat`; the correct propagation
syntax is `cargo test -p sim-thermostat --features
sim-core/parallel --lib prf::`, verified during the audit to
run and pass all 10 prf tests. Both fixes land as a single
narrow post-commit patch, matching the `b5cb3f6c`/`3e1ec0ff`
precedent. Both are prose drifts, not argument changes —
Ch 40's decisions stand.

**Session 13 pre-work patches to §3.1, §3.3, §3.4, §3.5, and
Appendix B.** During the session-13 recon pass that followed
PR 1a's landing — the read-Ch 40-§3-end-to-end + recon-the-
current-state step before any PR 1b code was written — six
prose drifts surfaced. All are rendering mistakes from the
session-8 drafting of §3; none change the load-bearing
decisions in §4's in-chapter sub-decision table. They were
patched pre-work (before PR 1b's first code commit) rather
than post-commit so the spec would be accurate for the
session-13 implementation pass. The six:

1. **§3.1 Table 2 — `rand_chacha` cannot be fully demoted.**
   `gibbs.rs` imports `rand_chacha::ChaCha8Rng` as a runtime
   consumer (`GibbsSampler` struct field at
   `thermostat/src/gibbs.rs:34`). Ch 40's initial draft
   assumed `langevin.rs` was the only `src/` user. The
   corrected row: `rand_distr` moves to `[dev-dependencies]`;
   `rand_chacha` stays in `[dependencies]` because gibbs
   still needs it. An alternative — migrating `GibbsSampler`
   to a prf-based Bernoulli — is out of PR 1b's scope
   (`GibbsSampler` is not a `PassiveComponent` and the mutex
   argument does not apply; `&mut self` is legal there).

2. **§3.1 Table 2 — integration-test crate is
   `sim-conformance-tests`, not "sim-integration".** The
   informal name matched no actual crate. The new regression
   test lives in `sim-conformance-tests` (per
   `sim/L0/tests/Cargo.toml:2`), and PR 1b must add
   `sim-thermostat` as a dev-dep plus a `parallel` feature
   forwarding to `sim-core/parallel` to that crate's
   `Cargo.toml` because neither exists today. Table 2 grows
   by one row to name this work.

3. **§3.3 trait return type — `-> EnvBatch` should be
   `-> EnvBatch<Self>`.** The trait definition block and the
   `EnvBatch<S>` struct definition immediately after were
   inconsistent within §3.3 itself (return type was
   non-generic; struct was generic). The rendered code in PR
   1b uses `EnvBatch<Self>` as the return type, threading
   through `S` at the `BatchSim::new_per_env` call site.

4. **§3.4 (b) non-mechanical call-site list was partly
   wrong.** The initial draft listed
   `tests/langevin_thermostat.rs:282` (the `CountingWrapper`)
   and `tests/multi_dof_equipartition.rs:346-357` as
   non-mechanical, and described `:346/:357` as "two
   thermostats coupled to a single simulation" needing
   distinct `traj_id`s. The session-13 recon re-read both
   files and found that `:346/:357` is actually the
   `assert_multi_dof_reproducibility` helper — two SEPARATE
   parallel simulations built with identical seeds,
   asserting bit-identity after N steps. Distinct `traj_id`s
   would break the reproducibility assertion. Both get
   `traj_id = 0`. Similarly, `tests/langevin_thermostat.rs
   :323/:334` (the `test_reproducibility_from_seed` test) is
   the same shape and also needs identical `traj_id = 0` at
   both sites, but was not flagged in the initial draft.
   `:282` is a single-thermostat wrapper — mechanical
   `traj_id = 0`. The genuine "distinct `traj_id` = i" case
   is `multi_dof_equipartition.rs:188, :263` (per-trajectory
   loops with `seed_base + i as u64`), which the initial
   draft placed inside the mechanical bucket but should be
   called out because the `seed_base + i` → `(master_seed,
   traj_id = i)` split is a non-trivial semantic rendering.

5. **§3.5 Collateral dependency bullet — same `rand_chacha`
   drift as §3.1.** Bullet rewritten to match the corrected
   Table 2 row: only `rand_distr` demotes; `rand_chacha`
   stays in runtime deps for `gibbs.rs`.

6. **§3.5 test plan bullets — same `sim-integration` →
   `sim-conformance-tests` rename as §3.1.** Both test-plan
   lines updated to the correct crate name and the correct
   `--test integration batch_sim` selector.

7. **Appendix B (`91-test-inventory.md`) carries the same
   non-mechanical-site misread as §3.4 (b).** Lines 154-160
   of Appendix B duplicate the `:282` / `:346-357` framing
   from the §3.4 initial draft. Appendix B is patched in the
   same pre-work commit to stay consistent with §3.4's
   corrected list.

All seven fixes land as a single pre-work drift patch before
PR 1b's first code commit, matching the `b5cb3f6c` /
`3e1ec0ff` / `eefa7842` narrow-patch precedent at a new
position (pre-work rather than post-commit). None change any
in-chapter sub-decision in §4. Ch 40's decisions stand; the
recon pass just caught the rendering before the code was
written against it.
