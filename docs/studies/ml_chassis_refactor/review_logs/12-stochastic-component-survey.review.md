# Review log — Chapter 12: Stochastic component survey

## Factual pass

**Status:** Run by a general-purpose sub-agent with no prior
context. 22 falsifiable claims extracted from the draft and
verified against the current tree.

| # | Claim | Verdict |
|---|---|---|
| 1 | `LangevinThermostat` struct at `langevin.rs:72` | VERIFIED |
| 2 | `seed: u64` field at line 75 | VERIFIED |
| 3 | `rng: Mutex<ChaCha8Rng>` field at line 76 | VERIFIED |
| 4 | Constructor `new(gamma, k_b_t, seed)` at line 102 | VERIFIED |
| 5 | `ChaCha8Rng::seed_from_u64(seed)` at line 107 | VERIFIED |
| 6 | `PassiveComponent::apply` impl at line 141 | VERIFIED |
| 7 | Mutex lock at line 184 | VERIFIED |
| 8 | Sample loop at 186–193 drawing `N_DOF` Gaussian samples | VERIFIED |
| 9 | `Diagnose` impl at 211–233 formats seed into summary | VERIFIED |
| 10 | `apply_does_not_advance_rng_when_stochastic_inactive` at `langevin.rs:312–363` | VERIFIED |
| 11 | `Stochastic` trait at `component.rs:101–108` with quoted signature | VERIFIED |
| 12 | `install_per_env` exists at `stack.rs:174–199` | VERIFIED |
| 13 | `install_per_env` produces N independent `(Model, Arc<PassiveStack>)` pairs | SEMANTIC DRIFT |
| 14 | Each env gets its own `LangevinThermostat` with its own `Mutex<ChaCha8Rng>` | VERIFIED |
| 15 | Five pure files have zero matches for `rng\|Rng\|ChaCha\|rand::` | VERIFIED |
| 16 | Each of the five implements `PassiveComponent` | VERIFIED |
| 17 | `LangevinThermostat` implements `PassiveComponent` (`langevin.rs:133`) | VERIFIED |
| 18 | `GibbsSampler` at `gibbs.rs:26–147` | VERIFIED |
| 19 | `GibbsSampler` owns `ChaCha8Rng` directly, not in a mutex | VERIFIED |
| 20 | `sweep(&mut self)` method exists | VERIFIED |
| 21 | `GibbsSampler` does NOT implement `PassiveComponent` | VERIFIED |
| 22 | `GibbsSampler` is not installed on any `PassiveStack` | VERIFIED |

**Substantive finding (claim 13):** The chapter's original framing
said `install_per_env` "produces $N$ independent `(Model,
Arc<PassiveStack>)` pairs." Actual signature is
`install_per_env(n, build_one: FnMut(usize) -> (Model, Arc<Self>))`
returning an `EnvBatch { models, stacks }`. The helper *consumes*
caller-supplied pairs from `build_one`, asserts `cb_passive.is_none()`
on each, and orchestrates the `PassiveStack::install` call. This
is a meaningful narrative inversion: the helper does not
construct anything, it orchestrates installation of things the
caller constructs. The refactor-path argument still holds — each
env ends up with its own component instance and its own
`Mutex<ChaCha8Rng>` — but the section's characterization of who
does the work needed to be rewritten.

**Fixes applied from factual pass:**

1. Rewrote the opening paragraph of the `install_per_env` section
   to describe the helper as installation plumbing that wraps a
   caller-supplied factory, explicitly naming `FnMut(usize) ->
   (Model, Arc<PassiveStack>)` and `EnvBatch { models, stacks }`.
2. Deleted the "Unverified detail carried forward" paragraph at
   the end of the section now that verification is done.
3. Updated the chapter's closing summary to describe
   `install_per_env` as "an installation helper that orchestrates
   per-env construction through a caller-supplied factory,"
   removing the "pending factual verification" qualifier.

**Minor cosmetic note (claim 18):** `GibbsSampler` citation
`gibbs.rs:26–147` technically runs through `config_bitmask`. The
struct itself plus `sweep` run 26–132. Within tolerance; not
fixed.

## Thinking pass

**Status:** Cold-reader sub-agent with no prior context. Eight-point
brief covering assumptions, unexamined alternatives, soft words,
logical gaps, scope discipline, over/under-claiming between the
two refactor paths, the `GibbsSampler` section's framing, and
the blast-radius comparison's fairness.

**Verdict:** Ship with minor edits. Structurally sound survey;
several scope-discipline and symmetry issues in the connective
tissue.

**Substantive findings and fixes:**

1. **Purity claim needs a scope caveat.** The original "none
   holds mutable state of any kind beyond its configuration" was
   sourced from a grep plus structural check and should say so.
   *Fix:* rewrote the paragraph to describe purity as a
   "lexical-plus-structural claim" and noted the forward-looking
   caveat that a component could in principle become stochastic
   via a future `Data` field.
2. **"Practical consequence" paragraph self-contradicted.** The
   draft said "one trait signature change" and then immediately
   said "regardless of which path chapters 14 and 15 pick" — the
   first framing assumes the trait-signature path, so the second
   framing cannot hold. *Fix:* rewrote the paragraph to describe
   "one file of substantive logic, with ripple scope that depends
   on which path chapters 14 and 15 pick," and explicitly worked
   through both paths (signature-change ripple vs
   `install_per_env` ripple).
3. **"Load-bearing for the study's cost model" is a promissory
   note.** The cost model lives in chapters 14/15, which do not
   exist yet. *Fix:* downgraded to "relevant to any cost
   estimate later chapters construct."
4. **Blast-radius bullets were asymmetric.** The draft gave
   `install_per_env` a friendlier phrasing ("preserves the
   `Stochastic` gating trait trivially") than the `Data` path
   ("only if chapter 14 resolves ..."), omitted per-env seed
   derivation as a cost on the `install_per_env` side, and did
   not mention test-suite impact for either. *Fix:*
   restructured both bullets along four explicit axes —
   structural change, per-env seed derivation, gating
   reconciliation, memory cost — and added a paragraph after
   the bullets noting that test-suite impact is symmetric
   between the two paths.
5. **`GibbsSampler` section smuggled an argument.** The second
   paragraph made a causal claim about how the D2c rematch
   performance gap would narrow once the Langevin refactor lands.
   That is chapter 30's argument, not chapter 12's. *Fix:* cut
   the causal claim to a one-sentence forward reference to
   chapter 30, noting the implementation asymmetry as an
   experimental-design concern and leaving the causal analysis
   to the rematch chapter.
6. **"Category error" in the `Stochastic` section was picking a
   side.** The draft called one way of preserving gating "a
   category error"; that phrase belongs to chapter 14, not the
   survey. *Fix:* softened to "awkward, because the gate is a
   component-level flag, not a per-env state property."
7. **Missing orthogonal-axes clarification.** The draft presented
   the "move RNG onto `Data`" vs `install_per_env` fork as if
   it were the full design space, but chapter 10 enumerated four
   mechanism shapes (stateful per-trajectory, splittable keys,
   counter-based, pre-generated tape) that are orthogonal to the
   hosting choice. A cold reader who did not read chapter 10
   would see only two options. *Fix:* added a paragraph after
   the blast-radius bullets naming the 2×$k$ grid explicitly and
   giving examples of splittable-key and counter-based
   implementations compatible with either hosting choice.
8. **"Smallness cuts the other way" paragraph was re-arguing
   chapter 10.** The history-of-the-code observation about how a
   one-file defect goes un-fixed because it looks patchable is
   chapter 10's territory. *Fix:* deleted the paragraph. The
   surrounding scope argument carries the survey's contribution
   on its own.

**Lower-severity items not acted on:**

- Cosmetic `GibbsSampler` line range (26–147 vs 26–132). Within
  factual-pass tolerance; left as-is.
- Wording in the "split-borrow as seen from the survey" section
  is a little compressed where it ties the partition choice to
  the license for `install_per_env`. Left intact because
  tightening it would risk losing the causal bridge.

## Second round

**Triggered:** No. All eight thinking-pass edits were targeted
revisions to connective tissue; none changed the structure, the
findings, or the forward references. No round-2 substantive
contradiction surfaced.

## Open questions carried forward

- **Downstream callers of `install_per_env`.** The survey verifies
  the helper exists but does not check how many callers currently
  use it. If no `BatchSim`-adjacent caller uses the helper today,
  chapter 14's "install_per_env vs RNG on Data" comparison needs
  to factor in the additional cost of building the first real
  caller. Flagged for chapter 14 and possibly for recon session
  D if the question deserves direct investigation.
- **The `Stochastic` gating-trait preservation question.** The
  survey names the constraint. Chapter 14 has to propose the
  concrete mechanism. Flagged.
- **Per-env seed derivation primitive.** Both paths need a
  per-env seed derivation. Chapter 15 has to pick the mixer
  (XOR, SplitMix64, counter-based, splittable). Flagged.

## Status

Drafted, factual pass run with 22/22 claims verified and one
narrative inversion fixed, thinking pass run with eight edits
applied, no second round triggered. Ready for commit (pending
user permission).
