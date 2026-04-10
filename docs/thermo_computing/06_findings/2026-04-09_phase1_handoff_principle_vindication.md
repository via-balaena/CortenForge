# Phase 1 finding — recon-to-iteration handoff principle, empirically vindicated

> **Status**: retrospective finding from the Phase 1 implementation session.
> **Recorded**: 2026-04-09 at the natural session boundary after the crime-scene investigation.
> **Branch**: `feature/thermo-doc-review`
> **Type**: meta — observations about the design discipline itself, not about the Phase 1 code.

## TL;DR

The recon-to-iteration handoff principle (canonical statement: `feedback_recon_to_iteration_handoff.md`) says:

> "The first compile error or test failure that contradicts a chassis decision is the *real* recon round, far more informative than another paper pass."

Phase 1 was the first time this prescription was tested empirically on a substantive piece of work. It worked exactly as advertised. Two paper-locked errors propagated through 4 paper-review passes (chassis grade, doc review, spec self-read, spec fresh-eyes review post-commit) without being caught. The first run of `cargo test` caught both errors within seconds. Diagnosis to root cause took ~30 minutes. Total cost of the "wasted" paper passes: minutes-to-hours each. Total cost of the recovery: hours, not weeks.

**The handoff principle is empirically validated.** This finding records the data so future sessions on this line (and on other initiatives) have a concrete data point to anchor on.

## The setup

The thermo-computing line was deliberately built under the sharpen-the-axe discipline (`feedback_sharpen_the_axe.md`):

> "Always prefer an extra recon round over committing on intuition. Pride, vision, and wisdom over speed."

The line went through:
1. **Recon round** (parts 02-10): 8 Phase-1-blocking items resolved
2. **Chassis design round** (`02_foundations/chassis_design.md`): 7 bolt-pattern decisions
3. **Doc review round** (`05_doc_reviews/2026-04-09_doc_review.md`): 15-item checklist (M1-M5, S1-S6, N1-N4)
4. **Phase 1 spec drafting** (`03_phases/01_langevin_thermostat.md`): ~500 lines, 3 pre-flight passes (inline self-read against rubric, stress-test against codebase, fresh-eyes review post-commit)
5. **Implementation** (this session): ~890 LOC across 8 files

By the start of the implementation session, every paper artifact had been read, graded, stress-tested, and reviewed. The chassis was paper-locked. The spec was paper-locked. The implementation phase was supposed to be a translation exercise: read spec → write Rust → run gate → review.

The recon-to-iteration handoff memory (a feedback memory written during the sharpen-the-axe iterations) anticipated that some chassis decisions might break under code despite all the paper passes:

> "Several chassis decisions — the trait write-target ergonomics (Decision 1), the `install_per_env` clone-footgun resolution (Decision 3), the Stochastic gating under FD perturbation (Decision 7), the Welford tolerance bands at 4.5% (Decision 5) — are *unverified assumptions* until code runs against them. The longer the chassis stays paper-only, the higher the chance one decision has a hidden flaw that propagates into the spec → code chain."

**Decision 5 was explicitly named as a candidate for code-time discovery.** It was the most concrete prediction the handoff memory made. **And Decision 5 is exactly what broke.**

## What was found and when

### Time spent on paper passes (cumulative)

| Pass | What it did | Approx. cost | Caught the bugs? |
|---|---|---|---|
| Recon log part 9 | Identified statistical infra as critical, deferred to chassis | hours | no |
| Chassis Decision 5 reasoning | Derived the std error formula, picked option β | hours | wrote the bugs |
| Chassis grading rubric | Graded Decision 5 against 7 criteria | ~15-30 min | no |
| Doc review M1, M4 | Corrected wording, added merge primitive | ~30-60 min | propagated the bugs |
| Spec §7.2 + §7.3 drafting | Translated chassis to phase spec | hours | propagated the bugs |
| Spec inline self-read against rubric | Pre-commit consistency check | ~5-10 min | no |
| Spec stress-test against codebase | Pre-commit code-citation check | ~15-20 min | no |
| Spec fresh-eyes review post-commit | Post-commit defect catching | ~15-30 min | no (caught 4 unrelated defects) |

**Total paper-pass time: ~5-10 hours of focused work.**

**Total bugs found: 0 (the two §7 + §10 bugs were not caught by any paper pass).**

### Time spent on the first code run + investigation

| Step | What it did | Approx. cost | Caught the bugs? |
|---|---|---|---|
| Implement langevin.rs + tests/langevin_thermostat.rs | Translation work | ~1-2 hours | wrote the impl that exposed the bugs |
| `cargo test -p sim-thermostat --release` | First gate run | ~5 seconds | **YES — both bugs surfaced as test failures** |
| Crime-scene investigation | Theoretical analysis + code archaeology + 6 empirical probes | ~30-45 minutes | confirmed root causes |
| Cold-read confirmation pass | Skeptic re-derivation of every finding | ~15 minutes | no new findings |

**Total code-pass time-to-detection: ~5 seconds (cargo test). Total time-to-root-cause-confirmation: ~45-60 minutes.**

### The accounting

- **Paper passes cost: ~5-10 hours, caught 0 of 2 bugs.**
- **Code passes cost: ~5 seconds + ~45-60 minutes for investigation, caught 2 of 2 bugs.**
- **Detection efficiency improvement from paper to code: at least 2 orders of magnitude on this round.**

The handoff principle predicted this. The empirical result is even stronger than the principle's claim — it didn't just say "code is better than paper at this step," it said paper has DIMINISHING RETURNS at this step. The data agrees: on this round, paper had ZERO returns.

## Why didn't the paper passes catch it?

This is the more interesting question. The 4 paper passes weren't lazy or careless — they were explicitly designed to catch consistency errors and were run with intent. The chassis grading rubric is a structured 7-criterion grade. The doc review was substantive (15 items). The spec self-read used the rubric criteria as mental anchors. The fresh-eyes review caught 4 OTHER real defects. **None of these passes caught the §7 std error formula error or the §10 time constant error.**

### What the paper passes WERE good at

Each paper pass has a domain where it works:

- **Inline self-read against rubric**: catches structural gaps, scope leaks, sequencing bugs, missing cross-references. Caught nothing on this round but typically catches 1-3 items.
- **Stress-test against codebase**: catches stale code citations, broken function names, line-number drift, factual errors about the surrounding code. Caught the chassis citation drifts on this round (now in `06_findings/chassis_citation_drifts.md`).
- **Fresh-eyes review post-commit**: catches author-direction blindness — defects the author missed because they read their own writing in the same direction it was written in. Caught 4 real defects on this round (a `Data::new(&model)` call that should've been `model.make_data()`, a section reference drift, an untestable §8 sub-test, and a missing Arc-clone dance in §10).

These three checks are complementary and load-bearing. The Phase 1 session validated them as effective for what they're designed to catch.

### What the paper passes were NOT good at

**Math/physics correctness vs. external truth.** The 7-criterion rubric grades structural properties of the artifact (coverage, reversibility, scope, sequencing, content fidelity, verifiability, executability). It does not grade math derivations against textbook physics. It does not grade statistical formulas against statistical theory.

**The translation gap between math text and code/API decision.** The math text is correct at every paper layer:
- Recon log part 9: correct, defers the choice
- Chassis Decision 5 reasoning: correctly derives `run_std/√100 ≈ 0.022`
- Doc review M1: correctly derives `0.707/√1000 ≈ 0.022`
- Spec §7.2: correctly derives `±4.5% of ½kT`

But each translation from math text to code/API decision had the same wrong inference:
- Chassis Decision 5 ship-justification: "we need merge to compute this"
- Doc review M4: "merge for option β"
- Spec §7.3 pseudocode: `global.merge(&traj); ... global.std_error_of_mean()`

**The rubric grades the math text. The rubric grades the code. The rubric does not grade the translation between them.**

This is a sharper meta-finding than "the rubric is blind to math." The math is in the text and the math is right. The blind spot is at the math-text-to-code-pseudocode boundary, where the same conceptual error can propagate through multiple layers because each layer reads the text and recopies the inference without re-deriving it.

### The deepest version of the meta-finding

The paper passes are good at **internal consistency** and **citation accuracy** but bad at **external correctness**. External correctness requires comparing the artifact to ground truth — and ground truth for math is "do the derivation from first principles" or "run the code and see what it does." Both are expensive ways for paper passes to verify; both are cheap for code passes to verify (the first via a careful re-derivation, the second via `cargo test`).

The handoff principle is the architectural response to this asymmetry. **Paper passes have diminishing returns once the artifact is internally consistent and citation-accurate; from there, the next learning gain comes from code.** The principle says: don't keep doing paper passes hoping they'll catch external-truth errors; switch to code at the right point.

The "right point" is when:
1. The high-value recon round has produced its findings (✓ done by part 11)
2. Load-bearing API surfaces are designed but not compiled (✓ chassis paper-locked)
3. The cheapest next experiment is small enough that running it costs hours, not weeks (✓ Phase 1 ~890 LOC, ~5 second gate run)

Phase 1 met all three criteria. The handoff fired exactly when it should have. The first code run produced both diagnoses within seconds.

## What this validates and what it doesn't

### Validates

- The recon-to-iteration handoff principle is empirically supported by the most rigorous test possible: a substantive piece of work where 4 paper passes ran clean and the first code run found 2 real bugs.
- The chassis paper-lock principle is good for what it does (lets the chassis remain stable while the spec evolves) but is not a correctness guarantee. It's a stability guarantee. Code is the correctness check.
- The "split spec session from implementation session" pattern from the recon-to-iteration handoff memory's "small enough" qualifier is justified. The Phase 1 spec session ran 3 pre-flight passes and committed the spec. The implementation session caught 2 bugs the spec session missed. The implementation session was the right place to find them.
- The 3 pre-flight checks (inline self-read, stress-test, fresh-eyes review) are still valuable. They catch a different class of bugs than code does. They are NOT a substitute for code, but they are not redundant with code either.

### Does NOT validate

- This is **one data point**. The handoff principle is supported by this round, not proven by it. A future round where paper passes catch the bugs and code is "redundant" would weaken the conclusion. We should keep watching.
- The detection efficiency comparison (paper: 0/2, code: 2/2) is round-specific. It depends on the bugs being math-correctness errors, which paper passes are bad at and code is good at. A round where the bugs are typos or scope leaks would invert the comparison.
- The 5-second time-to-detection is misleading without context. The 5 seconds is the runtime of `cargo test` AFTER the implementation was written (~1-2 hours of work). The implementation work is part of the cost of the code pass. Honest comparison: paper passes ~5-10 hours / 0 bugs vs code passes ~1-2 hours of impl work + 5 seconds of test + 45-60 min of investigation / 2 bugs.

The honest framing is: **once the implementation work is done anyway (which it has to be), the cost of running the gate is essentially zero, and the gate catches a class of errors that paper passes systematically miss.** That's the principle's claim, and that's what we observed.

## Implications for future sessions on the thermo-computing line

### Immediate (Phase 1 closing review)

The fix execution session — the next session — should expect to find more issues during the closing implementation review (§13) than the paper passes predict. The pattern is now established: paper-locked decisions can have flaws that only surface under code. The closing review should be done with a **skeptic's stance**, not a "the spec is right, just verify the impl matches" stance.

### Phase 2 onward

Each subsequent phase should expect a similar pattern. The chassis is paper-locked and serves Phase 1 well, but Phase 2's first code run will likely find another class of issues — issues we can't predict from inside Phase 1 because they require Phase 2's specific physics + code interactions.

**Don't try to fix this by adding more paper passes.** The diminishing-returns curve is real. Instead, structure each phase so the first code run happens early and the closing review is the highest-value step.

### Phase 1 spec rubric amendment (deferred decision)

Should the rubric grow a new criterion for "math-to-code translation correctness"? My weak no: the rubric already has 7 criteria, and adding more makes it heavier without addressing the core issue (paper passes are bad at external correctness, period). The handoff principle is the better response. If the user wants to amend the rubric, the right amendment is not "add a new criterion" but "name explicitly that the rubric does NOT cover external math correctness, and the code pass IS the math-correctness check."

This would be a `feedback_grading_rubric.md` memory amendment in a future session.

### Generalizing beyond the thermo line

The handoff principle was written specifically for the thermo-computing line, but the pattern is more general. Any initiative that follows the sharpen-the-axe → chassis → spec → impl pipeline could face the same issue. The Phase 1 round is concrete evidence that the pipeline can produce paper-locked errors that only surface under code. Other initiatives should plan for this.

## Reproducibility / data preservation

This finding is not directly reproducible — it's a retrospective on the *process* that produced the §7 + §10 bugs and the *process* that caught them. The reproducible artifacts are:

- The two cracks themselves (recorded in `06_findings/2026-04-09_phase1_statistical_propagation_chain.md` and `06_findings/2026-04-09_phase1_section_10_time_constant.md`)
- The empirical evidence (Probes A through F in `tests/_phase1_findings_probes.rs`, committed in this session as a handoff artifact, deleted in the next session)
- The git history of the propagation chain (every chassis edit, doc review entry, spec edit is in git)

What's NOT reproducible is the specific time accounting in this finding — the "~5-10 hours of paper passes" is an estimate based on session lengths, not a measured quantity. Future sessions could try to track this more rigorously if the question matters enough.

## Closing observation

The handoff memory was written 2026-04-09 (the same day as the spec session, before the implementation session). It explicitly named Decision 5 as a candidate for code-time discovery:
> "the Welford tolerance bands at 4.5% (Decision 5) — are *unverified assumptions* until code runs against them"

Decision 5 was the one that broke. The handoff memory predicted the failure mode at the right level of specificity (named decision, named risk pattern). The empirical confirmation came less than 24 hours later.

**The thermo-computing line has now produced its first concrete piece of evidence that the recon-to-iteration handoff principle works as designed.** It wasn't a proof, but it was a strong validation: the right principle, the right prediction, the right outcome. Future sessions should treat the principle as battle-tested, not speculative.
