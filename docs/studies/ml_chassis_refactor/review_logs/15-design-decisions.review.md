# Review log: Ch 15 Design decisions

Chapter type: **argument chapter**. Picks the cell (C-3) from Ch
14's shortlist, picks the PRF implementation (Route 2 — manual
ChaCha8 block function in the thermostat crate), and specifies
three smaller design calls (per-env seed derivation, gating
preservation, regression test). Phase 3 **gated chapter** — paused
for user review before commit per the Ch 01 protocol.

Passes run **sequentially** per the Phase 2 process note: "Running
factual + thinking passes in parallel works for descriptive
chapters; run them sequentially for design chapters." Ch 15 is the
first Phase 3 argument chapter where the sequential pattern
applied (Ch 14 was hybrid and ran passes in parallel).

## Factual pass

**Status:** Run via Explore agent. **33/33 claims verified.**
Every file:line citation in the chapter matches the current
tree. One minor line-range overspecification was fixed before
the thinking pass.

| # | Claim | Method | Result |
|---|-------|--------|--------|
| 1 | `component.rs:101-108` Stochastic trait | Read | Verified |
| 2 | `langevin.rs:133` impl PassiveComponent for LangevinThermostat | Read | Verified |
| 3 | `langevin.rs:102` LangevinThermostat::new(gamma, k_b_t, seed) signature | Read | Verified |
| 4 | `langevin.rs:146` flag read | Read | Verified |
| 5 | `langevin.rs:167-174` gating early-return | Read | Verified |
| 6 | `langevin.rs:184` mutex lock | Read | Verified |
| 7 | `langevin.rs:191` StandardNormal.sample call | Read | Verified |
| 8 | `langevin.rs:196-198` as_stochastic override | Read | Verified |
| 9 | `langevin.rs:241-266` three gating tests | Read | Verified |
| 10 | `langevin.rs:311-363` apply_does_not_advance_rng_when_stochastic_inactive | Read | Verified |
| 11 | `stack.rs:174` install_per_env | Read | Verified |
| 12 | `stack.rs:231` disable_stochastic | Read | Verified |
| 13 | `stack.rs:282` StochasticGuard struct | Read | Verified |
| 14 | `stack.rs:307,316,327` three dummy test impls | Read | Verified |
| 15 | `stack.rs:380-480` disable_stochastic guard tests | Read | **Line drift** — actual range is 366-446, chapter said 380-480. Fixed pre-thinking-pass. |
| 16 | `component.rs:124,137` two dummy test impls | Read | Verified |
| 17 | `double_well.rs:142` impl PassiveComponent | Read | Verified |
| 18 | `oscillating_field.rs:139` impl PassiveComponent | Read | Verified |
| 19 | `ratchet.rs:142` impl PassiveComponent | Read | Verified |
| 20 | `pairwise_coupling.rs:167` impl PassiveComponent | Read | Verified |
| 21 | `external_field.rs:61` impl PassiveComponent | Read | Verified |
| 22 | `tests/langevin_thermostat.rs:260` CountingWrapper impl | Read | Verified |
| 23 | `data.rs:27-30` Key Invariant doc | Read | Verified (exact match) |
| 24 | `data.rs:520` time: f64 field | Read | Verified |
| 25 | `batch.rs:64-67` BatchSim struct | Read | Verified |
| 26 | `batch.rs:73-76` BatchSim::new signature | Read | Verified |
| 27 | `rand_chacha-0.9.0/src/chacha.rs:213` set_word_pos public API | Read | Verified |
| 28 | No rand_philox crate | cargo search | Verified (zero matches) |
| 29 | No rand_threefry crate | cargo search | Verified |
| 30 | aprender-rand at 0.29.0 Philox 4x32-10 | cargo search | Verified |
| 31 | squares at 0.1.1 counter-RNG | cargo search | Verified |
| 32 | BatchSim::new has no seed parameter | Read | Verified |
| 33 | BatchSim does not call install_per_env | Grep | Verified (zero matches) |

**Verdict:** Factual pass clean. All source-verifiable claims
accurate. The one line-range drift (claim 15) was fixed in place
before the thinking pass ran.

**Non-verified items (by kind, intentional):**
- Crypto specification claims about ChaCha8 (256-bit key, 64-byte
  block output, etc.) — standard well-known properties, verified
  indirectly via the thinking pass's technical review of the
  `chacha8_block` signature.
- The language-level properties of `AtomicU64`, `AtomicBool`,
  `Sync`, and `Send` — standard Rust language guarantees.
- The Ch 14 grid inheritance — Ch 14's internal structure is
  verified by Ch 14's own factual pass; Ch 15's references to
  Ch 14 are verified by reading Ch 14 (done during the thinking
  pass setup).

## Thinking pass

**Status:** Run via general-purpose agent against the 10-point
brief for argument chapters. **17 findings surfaced.** Verdict
returned: *"ship with medium edits."* All 17 findings have been
addressed in the draft (16 with edits, 1 minor finding
intentionally left as a judgment call, documented below).

### Findings applied

1. **Soft-word: "strictly dominates" in Section 1.1 reader-clarity
   argument.** The chapter had claimed Route 2 "strictly dominates"
   shape 1 on reader clarity, but "strictly dominates" is a
   precise technical term (better on every axis) that does not
   apply here — Route 2 has real costs (the ~50-100 lines of
   owned code and the verification test). Softened to "wins on
   balance" and acknowledged the cost side explicitly in one
   sentence.

2. **Argument gap: foundational-over-patch framing too strong.**
   The chapter had implied future stochastic components under
   C-1 would "relearn the mutex dance" as if the dance were a
   significant obstacle, while under C-3 they would "share" a
   primitive. In reality the mutex dance is a 2-line idiom and
   C-3's sharing is also by-convention-and-documentation (Section
   7 declines to formalize). Rewrote the paragraph to name the
   actual delta: "shape 1's per-component code reads as ceremony,
   shape 3's reads as arithmetic — for the beginners-see-the-
   architecture-clearly criterion, the arithmetic reading is
   preferred." Same conclusion without overclaiming.

3. **Argument gap: `time` / `step_index` asymmetry stated but
   not explained.** The Section 1.2 argument had observed that
   `time` and `step_index` have different consumers but left
   implicit why that mattered. The strengthened version names
   the actual distinction: `time` earns its place on `Data` by
   its *consumer reach* — non-thermostat components like
   `OscillatingField` and `RatchetPotential` read it as physics.
   `step_index` has no such reach; it is thermostat-internal
   plumbing. Hosting a thermostat-internal field on `Data` is
   the opposite of the `PassiveComponent`-as-aftermarket framing.
   This closes the gap between "different consumers" and
   "disqualifies `step_index`" that the prior version had left
   open.

4. **Argument gap: Section 1.3 conflated FD invariant with
   constructor-vs-ApplyCtx.** The original Section 1.3 argument
   walked the FD-breaks-under-B-3 analysis and then pivoted to
   "B-3's remaining benefit is very small" without marking that
   the second move was a new, independent argument. Split the
   section explicitly into two stages: **stage 1** is the
   FD-breaks argument (shows B-3's natural reading is broken),
   **stage 2** is the constructor-vs-ApplyCtx argument (shows
   the only repair leaves B-3 with nothing to offer over C-3).
   The two stages now have explicit subheaders and the reader
   sees B-3 is dismissed by two independent arguments, not one
   argument asked to cover both.

5. **Soft-word: "natural shape" of B-3 unexplained.** The
   Section 1.3 argument rested on calling counter-on-EnvMeta
   the "natural reading" of B-3 without saying why. Added one
   sentence: "It is the natural reading because `EnvMeta`'s
   purpose — hold per-env state that sits outside `Data` and
   outside the component instance — is exactly what a per-env
   step counter is." This makes option (iii)'s later appearance
   read as a rescue rather than as an alternative the chapter
   reverse-engineers toward.

6. **Must-fix, technical error: `chacha8_block` signature wrong.**
   The original Section 2.3 had `chacha8_block(master_seed: u64,
   counter: u128) -> [u8; 64]`. This is technically wrong in two
   ways: (a) ChaCha8 has a 256-bit (32-byte) key, not a 64-bit
   seed, so the function signature should take `key: &[u8; 32]`;
   (b) ChaCha's block counter architecture is a 64-bit block
   counter (plus internal word-in-block addressing that `rand_chacha`
   exposes as a `u128`), not a plain `u128` counter. The
   verification-against-rand_chacha claim in the original paragraph
   would have failed at implementation time because the key
   expansion from `u64` to `[u8; 32]` was elided. **Rewrote Section
   2.3 substantially** to:
   - Take `key: &[u8; 32]` explicitly in `chacha8_block`'s signature.
   - Add a sibling `expand_master_seed(master_seed: u64) -> [u8; 32]`
     helper that matches `rand_chacha`'s `seed_from_u64` expansion
     rule, so the verification test can cross-check the expansion
     as well as the block function.
   - Use `block_counter: u64` for the block counter, with an
     `encode_block_counter(traj_id: u64, step_index: u64) -> u64`
     helper that reserves high bits for `traj_id` (40 bits,
     covering up to 10^12 envs) and low bits for `step_index` (24
     bits, covering 16M steps per env) at the rematch
     configuration.
   - Add explicit ChaCha8 background (256-bit key, 64-bit nonce,
     block counter, 64-byte output per block) so a reader knows
     what primitive Route 2 is implementing.
   - Note that one block holds 8 `u64`s = 4 Box-Muller pairs = 8
     Gaussians, so a 6-DOF thermostat uses one block per apply
     (the first 6 of 8 Gaussians, the remaining 2 discarded).
   - For `n_dofs > 8`, the wrapper iterates the block counter —
     Part 4 decides whether to ship the general case or optimize
     for the single-block case at the D2c configuration.

7. **Realism: Section 2.3 performance claim contradicted Section 4
   sketch.** Section 2.3 had claimed "one block per apply is
   sufficient" but Section 4's code sketch looped
   `gaussian_at(master_seed, traj_id, step_index, dof)` per DOF,
   which would naively compute one block per DOF (6× work).
   Reconciled by updating Section 4's sketch to compute the block
   once outside the DOF loop and call `box_muller_from_block` to
   produce 8 Gaussians, then use the first `n_dofs` Gaussians in
   the accumulation loop. Section 2.3's performance paragraph now
   matches the sketch.

8. **Missing coverage: Section 4 sketch elided constructor
   signature change.** The original Section 4 sketch left
   `k_b_t = /* unchanged */` and did not mention that
   `LangevinThermostat::new`'s signature grows under C-3 to carry
   `master_seed` and `traj_id`, or that the `seed` field is
   renamed to `master_key`, or that `diagnostic_summary` updates
   accordingly. Added an explicit subsection naming each of these
   and marking the exact constructor shape as a Part 4 PR concern
   with two defensible options (direct constructor vs builder
   method). The `effective_k_b_t(data)` helper now encapsulates
   the `k_b_t_ctrl` logic explicitly, pointing a reader at
   `langevin.rs:154-156` for the unchanged behavior.

9. **Must-fix, realism: "bit-identical by construction" claim
   incomplete.** The original Section 5.1 had asserted bit-identical
   across parallel and sequential "by construction" because the
   PRF is a pure function, but this argument only covers the
   noise *inputs*. For `(qpos, qvel)` trajectories to be
   bit-identical, the downstream integrator arithmetic also needs
   to be deterministic in the face of parallel scheduling — which
   requires no cross-env reductions in the step path. Added a
   paragraph naming the four independent properties the
   bit-identical claim rests on: (a) PRF purity (verified in
   Section 2.3), (b) per-env thermostat instances (via
   `install_per_env`), (c) no cross-env reductions in
   `BatchSim::step_all`'s step path (a property of the current
   code that must continue to hold), (d) single-threaded
   `cb_passive` within one env (preserved by `install_per_env`).
   Named the regression test as a detector of future regressions
   against property (c) if a refactor introduces a cross-env
   reduction.

10. **Realism: Section 5.3 test sketch invented API names that
    conflict with the current chassis.** The sketch had referenced
    `BatchSim::new_from_env_batch`, `step_all_sequential`, and
    `step_all_parallel` as if they were the post-refactor API,
    but the current `BatchSim::step_all` is feature-gated by the
    `parallel` Cargo feature and the sketch was implicitly
    proposing an API change Ch 15 had no authority to make.
    Rewrote the Section 5.3 preamble to name two test strategies:
    **strategy A** uses the current feature-gate architecture and
    runs the test twice (once with, once without `--features
    parallel`), dumping traces to files and comparing in a
    separate companion assertion; **strategy B** assumes a post-
    refactor runtime-selectable API. Ch 15 recommends strategy A
    for the initial landing because it does not require deciding
    whether the feature-gate architecture should change. The
    sketch is now consistent with "current API, feature-gate-
    per-test" and names strategy B as a future variant.

11. **Soft-word: "refactor has a bug" framing in Section 5.1.**
    The original framing had said any numerical difference
    between runs meant "the refactor has a bug." Given finding 9's
    four-property list, the softer framing is that a numerical
    difference could indicate either (a) a bug in the PRF purity
    or (b) a new cross-env coupling introduced by an unrelated
    refactor. Updated to name both failure modes.

12. **Structural: Section 5.4 heading contradicted its own
    content.** The heading was "Why a single master seed is
    sufficient" but the content then recommended running at three
    master seeds for robustness coverage. A reader who read only
    the heading would conclude "one seed, done." Renamed the
    section to "Seed coverage: one for correctness, three for
    robustness."

13. **Scope drift: Section 7 second-stochastic-component paragraph
    framed a decision as a deferral.** The original paragraph had
    declined to address how a second stochastic component should
    reuse the PRF primitive, but then went on to specify the
    convention anyway (constructor args, own counter, own active
    flag, `chacha8_block` call). That is not a deferral; it is a
    specification that the chapter declines to formalize as a
    trait. Rewrote to separate decided (the convention) from
    deferred (whether to formalize as a trait), so the reader
    sees both the positive statement and the explicit
    deferral without confusion.

14. **Minor cleanup: Section 7 PR-split deferral was a Part 4
    preview in disguise.** The original paragraph had listed the
    pieces of the refactor ("the PRF primitive, the thermostat
    rewrite, the `BatchSim::new` rewire, the regression test,
    the existing test updates, the stale-comment removal") while
    framing this as a deferral to Part 4. That list is not a
    deferral — it is a preview of Part 4's scope. Shortened the
    paragraph to two sentences naming the landing strategy as a
    Part 4 concern without previewing the piece list.

15. **Structural: Section 1.4 synthesis too terse.** The original
    synthesis was three bullet points and a one-sentence
    conclusion, which for a ~400-line argument chain felt pre-
    ordained. Added a "What would have changed the pick"
    paragraph naming the counterfactuals: Route 2 unavailable
    → C-1 wins; loose reading of the `Data` invariant → A-3
    wins; FD gating invariant not load-bearing → B-3 wins. This
    makes the synthesis read as honest rather than reverse-
    engineered.

16. **Soft-word: "dominates" used loosely in Section 1.3 header
    and chapter intro.** "Dominates" has a precise meaning
    (better on every axis); the argument supports "wins on
    balance" (better overall, with some axes as tradeoffs).
    Updated the Section 1.3 header, the chapter intro, and the
    Section 1 opening to use "wins over ... on balance"
    consistently.

### Findings not applied

**Finding 17: Chapter opening reveals the pick twice.** The
thinking pass noted that the opening three paragraphs preview
the C-3 conclusion in both the second paragraph ("The pick is
C-3") and the third ("The four smaller design calls fall out of
C-3 more or less automatically"), which creates a belt-and-
suspenders effect where the reader knows the answer before the
argument begins. The reviewer offered two fixes: trim the intro
or mark sub-section headers as "argument for" rather than as
conclusions.

**Left as a judgment call.** For a long argument chapter, an
upfront reveal of the conclusion helps the reader orient to
the argument's destination; the "belt-and-suspenders" effect is
mild and arguably serves the reader better than delaying the
reveal would. The sub-section headers that restate the
conclusion (1.1 "Shape 3 wins the mechanism axis", 1.3
"Hosting C wins over hosting B on balance") are load-bearing
navigation — they tell a reader what each sub-section
concludes, which is the point of section headers in a design
chapter. Changing both the intro and the headers would make
the chapter harder to navigate for a marginal gain in
rhetorical restraint. Finding 17 is flagged in this log but
not applied.

### Process notes

- **Sequential passes worked as expected for an argument
  chapter.** The factual pass (33/33 verified, 1 minor line-range
  fix) gave the thinking pass a clean citation base to work
  against, and the thinking pass's findings were
  argument-structural rather than citation-structural — which is
  the division of labor the sequential pattern is designed to
  produce. The parallel pattern used for Ch 14's descriptive
  chapters would have been a worse fit for Ch 15 because
  finding 6 (the `chacha8_block` technical error) and finding 9
  (the bit-identical argument gap) are the kind of findings that
  would have forced a reconciliation with the factual pass's
  citation verification — running them in sequence avoids that
  friction.
- **Two must-fix findings.** Finding 6 (chacha8_block signature)
  and finding 9 (bit-identical argument) were the only findings
  that would have shipped wrong content if left as-is. The
  remaining 15 findings were rhetorical, structural, or framing
  issues that improve the chapter but would not have misled a
  reader about the refactor's correctness. The ratio (2
  must-fixes in a ~1000-line argument chapter) is consistent
  with Phase 2's observation that thinking passes surface 1-2
  substantive issues per chapter along with a handful of
  polish-level findings.
- **Three findings touched the technical specification of
  Route 2.** Findings 6, 7, and 8 all required updates to
  Section 2.3 and Section 4 together, and the three had to be
  applied as a coherent unit (the signature change in 6 informs
  the sketch in 4, which informs the performance claim in 7,
  which informs the constructor deferral in 8). This is
  consistent with "an argument chapter's substantive findings
  cluster in the load-bearing sections, not the scope-discipline
  sections." Future argument-chapter thinking passes should
  expect clusters of this kind.

### Verdict

Chapter 15 is **ready for user review before gated commit**. All
17 thinking-pass findings addressed (16 applied, 1 intentionally
left). Factual pass clean. No structural rework required. Per
the Ch 01 protocol for gated chapters, the chapter pauses here
for user review; the author does not commit autonomously.
