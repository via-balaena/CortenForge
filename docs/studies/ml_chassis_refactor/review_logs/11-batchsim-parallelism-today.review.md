# Review log — Chapter 11: BatchSim parallelism today

## Pre-draft consistency check

Before drafting, a sub-agent cross-read recon session A
(`session-a-stochastic-components.md`) and recon session B
(`session-b-batchsim-parallelism.md`) for factual and conceptual
consistency. Verdict: **Consistent.** Both documents agree on every
overlapping fact (`PassiveComponent::apply` signature, `LangevinThermostat`'s
`Mutex<ChaCha8Rng>`, `PassiveStack`'s `mem::replace` split-borrow,
the `batch_matches_sequential_with_contacts` deterministic-physics
scope). They partition the seam cleanly: session B covers
`sim/L0/core` and `sim/L0/ml-bridge`; session A covers
`sim/L0/thermostat`. One drafting instruction came out of the
check: chapter 11 should forward-reference the `install_per_env`
alternative that chapter 12 develops, since session B does not
cover it. This instruction was followed in the split-borrow
section of the draft.

## Factual pass

**Status:** Run by a general-purpose sub-agent with no prior
context. 24 falsifiable claims extracted from the draft and
verified against the current tree.

| # | Claim | Verdict |
|---|---|---|
| 1 | `BatchSim` struct at `batch.rs:64–67` with `model: Arc<Model>, envs: Vec<Data>` | VERIFIED |
| 2 | `BatchSim::new(model: Arc<Model>, n: usize)` at `batch.rs:73–76` | VERIFIED |
| 3 | `step_all` at `batch.rs:148–167` with the quoted rayon/iter_mut body | VERIFIED |
| 4 | `sim/L0/core/Cargo.toml` declares `parallel = ["dep:rayon"]` at line 38 | VERIFIED |
| 5 | `parallel` is default-on in the workspace build | **DISCREPANCY** |
| 6 | `Data::step` at `forward/mod.rs:220` | VERIFIED |
| 7 | `Data::forward` at `forward/mod.rs:279–281` | VERIFIED |
| 8 | `Data::forward_core` at `forward/mod.rs:408–427` | VERIFIED |
| 9 | `Data::forward_acc` at `forward/mod.rs:527–586`, calls `mj_fwd_passive` at line 535 | VERIFIED |
| 10 | `mj_fwd_passive` at `forward/passive.rs:348–736`, `cb_passive` invocation at 719–721 | VERIFIED |
| 11 | `cb_passive` closure is `Fn(&Model, &mut Data)`, installed by `PassiveStack::install` | VERIFIED |
| 12 | `PassiveStack::install` uses `mem::replace` on `qfrc_passive` for split-borrow | VERIFIED |
| 13 | `LangevinThermostat::apply` at `langevin.rs:141` | VERIFIED |
| 14 | `self.rng.lock()` at `langevin.rs:184` | VERIFIED |
| 15 | Lines 186–193 draw `N_DOF` Gaussian samples | VERIFIED |
| 16 | `Data::reset` in `sim/L0/core/src/data.rs` lines 1064–1200 | **DISCREPANCY** (path) |
| 17 | `Data` has no RNG field | VERIFIED |
| 18 | `Data::reset` enumerated fields match the chapter's list | VERIFIED |
| 19 | `batch_matches_sequential_with_contacts` at `integration/batch_sim.rs:54–91` | DRIFT (actual 55–91, within tolerance) |
| 20 | Test uses four-env deterministic `BatchSim`, bit-exact assertions | VERIFIED |
| 21 | `VecEnv::step` at `ml-bridge/src/vec_env.rs:132–248` | VERIFIED |
| 22 | Line 156 calls `self.batch.step_all()` | VERIFIED |
| 23 | Sub-stepping loop at 155–167 | VERIFIED |
| 24 | Auto-reset/obs-extraction at lines 194–226 | **DISCREPANCY** (actual 194–236) |

**Discrepancies resolved:**

1. **Claim 5 — `parallel` feature status.** The chapter's draft had
   flagged this for verification. Resolution: `sim/L0/core/Cargo.toml:37`
   declares `default = []` — the feature is default-off. A grep for
   `features = ["parallel"]` across every `Cargo.toml` under `sim/`
   returned zero matches: no crate in the workspace opts in. Chapter
   section rewritten to state factually that the parallel branch is
   unreachable under a plain `cargo build` / `cargo test`, and that
   the problem the rest of the chapter describes is latent under the
   default build and becomes reachable only under `--features parallel`
   or an external consumer that opts in. The chapter's framing shifted
   from "the parallel path is the live one" to "the parallel path is
   the path later chapters weigh fixes against"; this is reflected in
   the rewritten "step_all and where rayon enters" section.
2. **Claim 16 — `Data::reset` file path.** Path corrected from
   `sim/L0/core/src/data.rs` to `sim/L0/core/src/types/data.rs`. Line
   range 1064–1200 was correct in the right file.
3. **Claim 24 — `VecEnv` auto-reset line range.** Changed from 194–226
   to 194–236. The inner auto-reset branch closes at 226, but the
   observation-extraction block that the chapter is describing runs
   194–236 end-to-end; the wider range matches the sentence.

**Drift noted but not fixed:** claim 19 (test starts at line 55 if
counted from the `fn` signature rather than the `#[test]` attribute).
Within ±5-line tolerance per the factual-pass protocol.

**Ancillary notes from the factual pass (informational):**

- `langevin.rs:176–183` has a forward-looking comment anticipating
  an `install_per_env` world. A reader opening that file with the
  chapter in hand will see language that sounds like it contradicts
  the chapter. Left unaddressed in Ch 11; the chapter's forward-
  reference to `install_per_env` in the split-borrow section gives
  the reader enough context to reconcile.
- `Data::step` at line 220 also validates the timestep and calls
  `mj_check_pos`/`mj_check_vel` and (in the RK4 branch) invokes
  `forward` multiple times per integrator stage. The chapter's
  one-line summary is defensible but compressed. Not fixed.
- `forward_core` fires `cb_control` between `forward_pos_vel` and
  `forward_acc`. Elided in the chapter's description. Not fixed.

## Thinking pass

**Status:** Cold-reader sub-agent with no prior context. Eight-point
brief covering assumptions, unexamined alternatives, soft words,
logical gaps, simpler explanations, falsifiability, scope discipline,
over/under-claiming.

**Verdict:** Ship with minor edits. Descriptive backbone sound; all
problems in connective tissue.

**Substantive findings:**

1. **"The architecture produces it."** Over-claim. Chapter 11 has
   shown that the draw order is thread-schedule-dependent; the
   statistical-independence failure is chapter 10's argument.
   Edit: replaced "produces it" with language about the architecture
   *permitting* the order-variance, and handed off the independence
   conclusion to chapter 10 explicitly.
2. **"The whole reason `BatchSim` holds `Vec<Data>`."** Unsupported
   historical claim. Downgraded to "a central reason," and "designed
   for" downgraded to "compatible with."
3. **"The refactor has to fix" the parallel path.** Prejudges
   chapter 12. Reworded to "later chapters will weigh fixes
   against."
4. **"Earn its name" / "exactly the kind of silence the study's
   title refers to."** Rhetorical flourishes smuggling chapter 13's
   conclusion into chapter 11's description. Dropped both; kept the
   descriptive version.
5. **Design-intent reading of `PassiveStack`'s `&Data` partition.**
   The draft said `PassiveStack` "was written to partition `Data`" —
   asserting intent without a git-blame citation. Reworded to
   "currently effects a partition," and added a sentence explicitly
   declining to decide whether the current shape is deliberate design
   or path-of-least-resistance.
6. **"The narrow change it is" / three-option enumeration for
   reset-with-seed.** Pre-weighing chapter 15's design space.
   Demoted to a single cross-reference sentence: chapter 11 now
   only notes the current absence of RNG state on `Data` and defers
   the design call to chapter 15.
7. **Determinism test "becomes false" the moment a Langevin is
   installed.** Strictly, it becomes *flaky* — bit-exactness depends
   on whether the two runs saw the same thread-scheduling decisions.
   Rewrote to name the flakiness directly, including the
   four-env-four-core vs sixteen-core asymmetry.
8. **"Any fix has to land inside `step_all`."** Self-contradicts
   the chapter's own forward-reference to `install_per_env`, which
   would be installed at `BatchSim::new` time and not touch
   `step_all`. Rewrote to acknowledge both in-scope fix locations
   ("change what runs inside `step_all`" OR "change what the call
   chain sees at all, via `BatchSim::new` installing a per-env
   stack").

**Recon-scope disclosure added** per finding 6: one new sentence
at the end of the "rayon enters" section notes that the recon
covers `sim/` and the `cb_passive` path only; downstream consumers
or alternate install sites are out of scope and would require a
separate pass.

## Second round

**Triggered:** No. All ten edits from the thinking pass were
targeted word-level or paragraph-level changes to an otherwise
sound descriptive backbone. None changed the chapter's structure,
the claims it makes, or the forward references to chapters 10,
12, 13, 14, 15. No substantive contradiction surfaced that would
invalidate the description.

## Open questions carried forward

- **Downstream `parallel`-feature opt-in outside the workspace.**
  The recon covered every `Cargo.toml` under `sim/`. If a
  downstream consumer (e.g., a notebook, a research script, or
  an external crate) enables the feature via `sim-core = { features
  = ["parallel"] }`, the "no workspace crate opts in" statement
  still holds but the "problem is unreachable under the default
  build" framing needs to be qualified for that consumer. Not
  load-bearing for Phase 2; flagged for Phase 3 if relevant.
- **The `langevin.rs:176` anticipatory comment.** Chapter 11 does
  not address the comment directly. If a cold reader later opens
  `langevin.rs` and is confused by the comment vs the chapter, a
  one-sentence parenthetical in the call-chain section could be
  added. Not currently blocking.

## Status

Drafted, factual pass run with three discrepancies resolved,
thinking pass run with ten edits applied, no second round
triggered. Ready for commit (pending user permission).
