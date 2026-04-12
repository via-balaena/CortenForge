# Review log: Ch 14 The new shape

Chapter type: **hybrid (descriptive + light argument)**. Lays out
the 2-axis design grid (mechanism × hosting) for the Langevin RNG
refactor; does not pick a cell. Phase 3 **gated chapter** — paused
for user review before commit per the Ch 01 protocol.

Passes run in parallel (consistent with the Phase 2 pattern for
descriptive chapters, adapted here because the chapter is
primarily descriptive with contained argument moments — the
collapse and elimination claims).

## Factual pass

**Status:** Run via Explore agent. **24/24 verified, 0
corrections needed.** Every file:line citation in the chapter was
grep-verified or read-verified against the current tree.

| # | Claim | Method | Result |
|---|-------|--------|--------|
| 1 | `Stochastic` trait declared at `sim/L0/thermostat/src/component.rs:101-108` | Read lines 101-108 | Verified |
| 2 | Trait includes `set_stochastic_active(&self, bool)` and `is_stochastic_active(&self) -> bool` | Read lines 101-108 | Verified |
| 3 | Test `apply_does_not_advance_rng_when_stochastic_inactive` at `langevin.rs:311-363` | Read lines 311-363 | Verified |
| 4 | Test description (sets inactive, runs two applies, flips active, constructs fresh thermostat, asserts equality) | Read test code | Verified |
| 5 | `LangevinThermostat` field `rng: Mutex<ChaCha8Rng>` at `langevin.rs:76` | Read line 76 | Verified |
| 6 | `LangevinThermostat` field `stochastic_active: AtomicBool` at `langevin.rs:77` | Read line 77 | Verified |
| 7 | Seed consumed at construction via `ChaCha8Rng::seed_from_u64` at `langevin.rs:107` | Read line 107 | Verified |
| 8 | `Data` struct defined at `data.rs:34` | Read line 34 | Verified |
| 9 | "Key Invariant" doc comment quoted text at `data.rs:27-30` | Read lines 27-30 | Verified (exact match) |
| 10 | `time: f64` field at `data.rs:520` | Read line 520 | Verified |
| 11 | `qDeriv_pos` CortenForge extension at `data.rs:606` | Read lines 606-608 | Verified (doc comment matches quote) |
| 12 | `impl Clone for Data` at `data.rs:671` | Read line 671 | Verified |
| 13 | `BatchSim` struct at `batch.rs:64-67` with fields `model: Arc<Model>` and `envs: Vec<Data>` | Read lines 64-67 | Verified |
| 14 | `BatchSim::new(model: Arc<Model>, n: usize)` at `batch.rs:73-76` with no seed parameter | Read lines 73-76 | Verified |
| 15 | `PassiveComponent::apply` signature as quoted | Read component.rs | Verified |
| 16 | Flag read at `langevin.rs:146` via `self.stochastic_active.load(Ordering::Relaxed)` | Read line 146 | Verified |
| 17 | Gating early-return at `langevin.rs:167-174` with `if !active { return; }` | Read lines 167-174 | Verified |
| 18 | Mutex lock at `langevin.rs:184` via `self.rng.lock().unwrap_or_else(...)` | Read line 184 | Verified |
| 19 | Quoted comment at `langevin.rs:176-183` on contention | Read lines 176-183 | Verified (later deleted during thinking-pass fix, see finding 7) |
| 20 | `PassiveStack::install_per_env` at `stack.rs:174` | Read line 174 | Verified |
| 21 | Module doc at `stack.rs:20-25` with quoted parallel-environment-construction text | Read lines 20-25 | Verified (exact match) |
| 22 | `debug_assert!` + `clear_passive_callback` in `install_per_env` at `stack.rs:185-186` | Read lines 185-193 | Verified |
| 23 | "nothing in `BatchSim` today calls `install_per_env`" | Grep `install_per_env` in `batch.rs` | Verified (zero matches) |
| 24 | "no seed parameter today on `BatchSim::new`" | Read signature at lines 73-76 | Verified |

**Verdict:** Factual pass clean. All source-verifiable claims
accurate as written. Every recon-reported file:line citation in
the chapter is correct at commit time.

**Non-verified items (by kind, intentional):**

- Textbook-level physics claims (the SDE form, the ensemble-
  variance argument) — not CortenForge code claims, no source
  verification needed.
- Memory/size calculations for shape 4 — arithmetic, verified
  against the stated assumptions, not against the tree.
- Claims about Rust ecosystem characteristics (what `rand_chacha`,
  `rand_philox`, `rand_threefry` are and are not) — general Rust
  knowledge, not CortenForge-specific.

## Thinking pass

**Status:** Run via general-purpose agent against the 10-point
brief for argument chapters (adapted for the lay-out-the-grid
role). **17 findings surfaced.** Verdict returned: *"ship with
medium edits."* All 17 findings have been addressed in the draft;
the chapter is ready for user review at gated commit.

### Findings applied

1. **Soft-word: "The most obvious mechanism" opened the shape 1
   section** with a frame that pre-loaded the reader against the
   other shapes. Replaced with "The mechanism the current code
   already uses, minus the shared-state defect." Neutral,
   factually tighter.

2. **Shape 2 collapse argument too narrow.** Original draft said
   shape 2 collapses into shape 1 in Rust. The collapse is real
   but happens in two directions: a stateful-child-from-stateful-
   parent reading collapses into shape 1, a pure-key-on-PRF reading
   collapses into shape 3. The conclusion ("three live entries,
   not four") is unchanged; the argument is now honest about both
   collapse directions.

3. **`time`/`step_index` equivalence argument too strong.** The
   original draft said `step_index` is "the same clock as `time`
   in discrete units" and implied the precedent was decisive. It
   isn't — `time` is a *physics* quantity that `OscillatingField`
   and `RatchetPotential` read; `step_index` is a *bookkeeping*
   quantity with no physics use. Tightened the argument to: the
   precedent is suggestive, not dispositive. Strict and loose
   readings are both defensible; the question is genuinely
   contested, and Ch 15 has to adjudicate.

4. **Cell A-1's in-section-2 elimination was asymmetric** with how
   B-1 gets handled (both carry a chassis-wide cost; B-1 is
   carried into the grid, A-1 was dismissed inline). The fix was
   to add one paragraph explaining what makes A-1 different: it
   takes *both* costs (trait change and mutex-in-a-different-
   struct) at once for no additional benefit, vs B-1's one cost
   for the mechanism's benefit. Grid in Section 3 now carries A-1
   through to explicit elimination on that ground, matching B-1's
   treatment in structural shape.

5. **PRF dependency framing missing the research-vs-production
   asymmetry.** "Less battle-tested" was stated as a risk without
   calibrating what the risk actually is for this codebase. Added
   one paragraph explaining that for a research codebase whose PRF
   output is fed through Gaussian transforms and consumed by an
   SDE integrator, "less audited" is a documentation cost not a
   numerical-quality risk, and that the Ch 13 regression oracle
   catches defects before they reach published numbers. The
   framing is now honest about magnitude, not just direction.

6. **Shape 4 cost calculation needed "16M-step budget" reading
   pinned down.** Original draft used 500K steps per env without
   saying why; under the alternate reading "16M per env," the
   numbers would be 32× larger. Pinned the calculation to Ch 22's
   Steps-as-total-env-steps convention (16M aggregate across the
   batch, 500K per env at `n_envs=32`) and added a note that the
   alternate reading would make the elimination even more
   decisive.

7. **Deleted the `langevin.rs:176-183` comment paragraph from
   Section 2 / Hosting C.** The paragraph was doing useful work —
   pointing out that hosting C closes a gap between an existing
   in-source comment and the live code path — but it was doing
   the work partisanly. Section 2 is supposed to walk hosting
   choices neutrally; that observation belongs in Ch 15's
   argument, not Ch 14's walk. Cleaner to delete than to
   rebalance. The observation itself is not lost — it lives in
   the Ch 13 latent-flakiness chapter and the Ch 15 thinking pass
   can resurrect it.

8. **Grid A-3 row "defensible under the loose reading" →
   "live under the loose reading."** "Defensible" prejudged the
   loose reading as winning the argument-quality contest;
   "live" is the neutral word the chapter uses elsewhere for
   cells that are candidates pending decision.

9. **Hosting C / shape 1 missing the non-mutex `Sync` wrapper
   escape hatch.** Under hosting C's per-env isolation, the
   generator type could be something `Sync` natively, or wrapped
   in a non-mutex pattern justified by the isolation. The chapter
   had dismissed these alternatives under shape 1's general
   section but not revisited them under hosting C, which matters
   for the C-1 vs C-3 race section 3 flags as central. Added one
   sentence to surface the question for Ch 15.

10. **"C-1 strictly dominates B-1" softened** to "C-1 dominates
    B-1 for the current single-stochastic-component world."
    Ch 12's survey found one stochastic component today, not a
    promise that it stays one; if a second is added, B-1's
    already-paid trait-signature change accommodates the second
    component without re-opening the `BatchSim::new` rewire
    question. The conclusion (B-1 out of Ch 15 shortlist) is
    unchanged; the framing is now honest about the future-
    component consideration, and the Part 4 PR plan inherits
    "reopens later" as a known consideration.

11. **Shape 3 "two integers" framing undercounted.** Shape 3's
    state is actually two per-trajectory integers *plus* a
    master seed shared across the batch, and the master seed has
    its own hosting question (most naturally on `Model` under
    hostings B and C, open under hosting A). Expanded the
    framing and added the master-seed location question
    explicitly under hosting A's shape 3 discussion.

12. **Closing section** had two small consistency issues. (a)
    Shape 4 verb drift: Section 1 said "Ch 15 is unlikely to
    pick," Section 3 said "eliminated," the closing said
    "dominated." Picked "eliminated" consistently across all
    three. (b) The "does not decide" list did not explicitly
    name the central deferred question (shape 1 vs shape 3,
    hosting A/B/C for whichever shape wins). Added one sentence
    making the central deferral explicit.

13. **Section 2 intro "Three choices are named by the constraints
    in Part 1 and Part 2" corrected** to "Three choices are
    visible in the chassis as it stands today." The original
    framing deferred the three-choice enumeration's authorship
    to earlier chapters, which is not quite honest — the
    enumeration is Ch 14's. The new framing has Ch 14 owning the
    enumeration and therefore being responsible for defending its
    completeness.

14. **Missing hosting-axis dismissal: on `Model`.** Added one
    sentence to Section 2's intro noting that putting per-env
    state on `Model` as a `Vec` indexed by env id collapses into
    hosting B (structurally a sibling vector to `envs`, just
    owned by `Model` instead of `BatchSim`) and is dismissed on
    the additional ground that `Arc<Model>` sharing is designed
    for read-only access during stepping. Plus a brief mention
    of the other candidates (thread-local, static global,
    `Data::plugin_state`) being dismissed in passing through the
    walk.

15. **"B1 is the honest version of hosting B" loaded.** Implied
    B2 was dishonest. Replaced with "B1 is the more direct
    version of hosting B; B2's avoidance of the trait change
    comes at the cost of mechanisms that are each their own kind
    of fragility, and is not obviously better."

16. **Duplicated gating discussion in Hosting C / shape 3.** The
    chapter had walked shape 3 gating once in Section 1 and
    walked it again almost line-for-line in Section 2 / Hosting
    C. The axes-first structure's whole point is to describe
    shared costs once. Fixed by replacing the Section 2 repeat
    with a short reference to Section 1's treatment.

17. **"The race is most naturally between C-1 and the shape-3
    family" was Ch 15's framing arriving early.** Dropped the
    framing in favor of a neutral enumeration: "Ch 15 has to
    pick within and across both shapes." Ch 15 can frame the
    race however its argument needs.

### Process notes

- The thinking pass correctly identified that for a chapter whose
  stated job is neutrality, **soft-word audit** and **pre-emptive
  elimination audit** are the two highest-yield brief points. Six
  of the 17 findings (1, 4, 7, 8, 15, 17) were "Ch 14 sneaking
  in a pick by framing" — more than any other category. This is
  consistent with Phase 2's pattern that thinking passes on
  grid-laying chapters find the pick-related leakage that the
  chapter's author is most likely to miss, since the author is
  closest to the design space and unconsciously weighted toward
  a preferred cell.
- The thinking pass also caught one structural repetition
  (finding 16) that the factual pass cannot catch by construction
  — the two passes are genuinely non-redundant.
- No finding triggered a full re-review of another section. The
  shape-2 collapse argument (finding 2) touched two places and
  was still a localized edit. The closing section's verb drift
  (finding 12a) was a three-place edit but each edit was one
  word. The thinking pass held up under the Phase 2 protocol's
  "localized-edit expected" assumption.
- Both passes ran in parallel (descriptive-chapter pattern). No
  reconciliation conflict: the one citation the thinking pass
  asked to delete (finding 7) was a paragraph the factual pass
  had already verified (item 19), and the deletion is because
  the paragraph was partisan, not because the quote was wrong.
  Post-fix, the quote is no longer in the chapter and item 19 is
  retained in this log as a record that the original quote was
  accurate.

### Verdict

Chapter 14 is **ready for user review before gated commit**. All
17 thinking-pass findings applied. Factual pass clean. No
structural rework required. Per the Ch 01 protocol for gated
chapters, the chapter pauses here for user review; the author
does not commit autonomously.

## Round 2: self-review post-commit patch (commit `7661f3d9`)

A second round of small consistency edits applied after the user
delegated the self-review. Caught two artifacts of the thinking-
pass fixes that hadn't propagated cleanly: a duplicate "three
live entries, not four" phrase in the shape 2 collapse section,
and Section 3's intro using stale framings ("Shape 2 collapses
into shape 1 in Rust, shape 4 is dominated by cost") that
contradicted the corrected framings elsewhere in the chapter.
Both fixed. No factual claims changed.

## Round 3: Ch 15 recon triggered PRF-ecosystem correction (commit pending)

The Ch 15 recon pass surfaced two factual and framing issues in
Ch 14's shape 3 PRF-cost discussion that needed a narrow patch
before Ch 15 could build on Ch 14 honestly. This follows the
Ch 21 post-commit-patch precedent (commit `3e1ec0ff`) where a
downstream chapter's recon revealed that an earlier chapter had
stated something that did not hold up against the code.

**What the Ch 15 recon found:**

1. **Ch 14 asserted `rand_philox` and `rand_threefry` "exist and
   are not hypothetical" as counter-based PRF crate candidates.**
   `cargo search rand_philox` and `cargo search rand_threefry`
   both return empty or abandoned results (`philox = "0.0.0"`
   is a placeholder). The crates named in Ch 14 do not exist as
   maintained entries in the Rust ecosystem. The factual claim
   was wrong.

2. **Ch 14 framed shape 3's cost as "a less-battle-tested PRF
   dependency," which was incorrect in a way that affected the
   shortlist analysis.** The Ch 15 recon verified that
   `rand_chacha 0.9.0` (already in the workspace) exposes
   `ChaCha8Rng::set_word_pos(u128)` as a public seeking API at
   `rand_chacha-0.9.0/src/chacha.rs:213`. This means shape 3 can
   be implemented either (a) on top of the chassis PRNG via
   `set_word_pos` (no new dependency, but keeps the mutex
   ceremony), (b) via a ~50-100 line manual ChaCha8 block
   function as a stateless pure function in the thermostat crate
   (no mutex, but owns a cryptographic primitive), or (c) via a
   small community crate such as `aprender-rand` or `squares`
   (both exist, both are real, both are less widely adopted
   than `rand_chacha`). The honest cost framing is
   "implementation cost, three routes," not "dependency cost."

**What the patch changed:**

- **Section 1 / Shape 3 / PRF-cost paragraph:** rewrote from one
  long paragraph that named non-existent crates into three
  explicit route descriptions. Route 1 is `set_word_pos` on
  `rand_chacha`, Route 2 is a manual ChaCha8 implementation in
  the thermostat crate, Route 3 is a counter-based PRF crate
  (with `aprender-rand` and `squares` named accurately as the
  live options). Each route has its own cost profile. Ch 14 does
  not pick; it names them honestly so Section 3's grid treats
  the cost accurately.
- **Section 1 / Summary of the mechanism axis:** the shape 3
  bullet changed from "a PRF dependency cost" to "a PRF
  implementation cost (one of three routes named above)."
- **Section 3 / grid rows A-3, B-3, C-3:** the "chassis changes"
  column changed from "PRF dependency" to "PRF implementation
  (Section 1, three routes)." The C-3 notes column was also
  updated to clarify that the lock-free `AtomicU64` replacement
  of `Mutex<ChaCha8Rng>` holds specifically under Route 2 (the
  manual ChaCha8 implementation), since Routes 1 and 3 each
  carry their own interior-mutability considerations.

**What the patch did not change:**

- The shortlist is unchanged: A-3, B-3, C-1, C-3.
- No eliminations or collapses changed.
- The shape 1, shape 2, and shape 4 discussions are unchanged.
- The hosting-axis walk is unchanged.
- The `Data` invariant discussion is unchanged.
- All 24 previously-verified file:line citations remain valid
  (none were touched).

**Process notes for Round 3:**

- Following the Ch 21 post-commit-patch pattern, this round did
  not go through a full factual + thinking pass cycle. The
  changes are narrow enough that this Round 3 log entry is the
  review record. The `rand_chacha 0.9.0` `set_word_pos` claim
  was verified against
  `/Users/jonhillesheim/.cargo/registry/src/index.crates.io-*/rand_chacha-0.9.0/src/chacha.rs:213`
  during the Ch 15 recon; the `cargo search` results for
  `rand_philox` and `rand_threefry` were captured during the
  same recon and are the basis for the factual correction.
- The decision to patch Ch 14 rather than handle the drift
  entirely in Ch 15 was taken because Ch 15 will cite Ch 14's
  grid directly, and citing a grid whose "PRF dependency" column
  contradicts Ch 15's argument would either force Ch 15 to
  silently paper over the contradiction or to visibly correct
  Ch 14 mid-argument. Cleaner to fix the foundation before
  building on it.
- The Ch 15 recon's other major finding — that C-3 (not B-3) is
  the right shortlist pick once the gating-under-FD property is
  accounted for — is a Ch 15 argument, not a Ch 14 correction.
  Ch 14 laid out the shortlist neutrally and still does; Ch 15
  is the place to argue the pick. No Ch 14 patch is needed for
  that finding.
