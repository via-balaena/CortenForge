# Review log — Chapter 13: Latent flakiness

Chapter 13 is a synthesis/argument chapter. It introduces no new
recon and pulls its facts from chapters 10, 11, and 12.

## Factual pass

**Status:** Run by a general-purpose sub-agent. Two verification
targets: (1) file:line citations against current source, and (2)
chapter-attribution claims verified against the feeder chapters'
actual text.

### Source-level citations (6/6 VERIFIED)

| Claim | Verdict |
|---|---|
| `sim/L0/core/src/batch.rs:148–167` — `step_all` body | VERIFIED |
| `sim/L0/thermostat/src/langevin.rs:141` — `apply` impl | VERIFIED |
| `sim/L0/core/src/forward/passive.rs:719–721` — `cb_passive` invocation | VERIFIED |
| `sim/L0/core/Cargo.toml:37` — `default = []` | VERIFIED |
| `sim/L0/tests/integration/batch_sim.rs:54–91` — `batch_matches_sequential_with_contacts` | VERIFIED |
| No workspace `Cargo.toml` enables `features = ["parallel"]` on a `sim-core` dep | VERIFIED (grep under `sim/` returned zero matches at pass time) |

### Feeder-chapter attributions (6/6 VERIFIED)

| Claim | Verdict |
|---|---|
| Ch 10 uses variance formula `(1/N)(σ² + (N-1)ρ̄σ²)` and frames it exactly as Ch 13 paraphrases | VERIFIED (Ch 10 line 70) |
| Ch 10 establishes per-trajectory statistical independence as "the physics requirement" | VERIFIED (Ch 10 lines 13–14, 84–85) |
| Ch 11 walks the `BatchSim::step_all` → `par_iter_mut` → `cb_passive` → `Mutex<ChaCha8Rng>` chain | VERIFIED (Ch 11 §"call chain", lines 152–186) |
| Ch 11 finding: `parallel` default-off and no workspace crate opts in | VERIFIED (Ch 11 lines 105–111) |
| Ch 12 finding: only one component in `sim/L0/thermostat/src/` holds stochastic state (six total) | VERIFIED (Ch 12 lines 34–36) |
| Ch 12 finding: the other five are pure and would not be affected by the refactor | VERIFIED (Ch 12 lines 61–84) |

**Verdict:** PASS. Every file:line citation lands against current
source; every feeder-chapter attribution is faithful to the
feeder's actual text. No fixes applied on the factual pass.

## Thinking pass

**Status:** Cold-reader sub-agent with no prior context. Ten-point
brief covering: the term "latent" itself, the three-legged tripod
metaphor, the worst-time-to-debug narrative, the urgency argument,
the test-shape gap framing, scope discipline, over-claiming,
under-claiming, the "honest note about naming" paragraph, and
whether the chapter's argument earns its conclusion.

**Verdict:** Ship with minor edits. Six substantive findings; all
applied. The verdict was "minor edits" but two of the findings
were structural (drop the tripod, compress the worst-time
vignette).

**Substantive findings and fixes:**

1. **"Latent" definition conflated severity with latency.** The
   original four-condition definition bundled "correctness
   failure symptom" (a severity claim) into what should be a
   pure reachability definition. *Fix:* dropped condition 4 from
   the definition of "latent," and moved the symptom-class
   analysis into a separate "how bad is it when live" section
   and a separate "symptom class routes debugging away from the
   physics layer" section. The word "latent" now does pure
   reachability work, and the severity question gets its own
   space.
2. **The tripod metaphor was forced.** Chapter 11 was being
   cited under two legs (execution and latency), so "three
   independent feeder chapters" wasn't quite true. Chapter 12's
   scope leg was a stakes claim, not a defect claim — not the
   same kind of support. *Fix:* dropped the tripod framing
   entirely. Renamed the section "What the feeder chapters
   establish" and presented four facts explicitly labeled as
   "Defect leg 1 (physics)," "Defect leg 2 (execution),"
   "Latency," "Scope." Four things, honestly named, instead of
   three things wedged into a geometric figure.
3. **The worst-time-to-debug section was speculative and
   rhetorical.** The Tuesday/Wednesday `experiment_042_seed_7`
   vignette was vivid but not earned from the findings, and it
   smuggled in "compounding debugging cost" without a cost
   model. *Fix:* cut the vignette and "under the lamp" flourish
   entirely. Compressed the section to one paragraph making the
   narrower, defensible claim: the symptom class (silent
   non-reproducibility) routes debugging attention away from
   the physics layer by default, because that symptom is the
   shape of a whole class of RL-stack bugs that have nothing
   to do with physics. Renamed the section accordingly.
4. **Missing "how bad is it when live" paragraph (under-claim).**
   A reader will immediately ask how large the statistical bias
   $\bar\rho$ is. Chapter 10 does not bound it, and Chapter 13
   was silent. The honest answer — "we do not know, the
   measurement is machine-dependent, and the refactor is the
   way we stop needing to ask" — is the chapter's to give.
   *Fix:* added a new section "How bad is it when live" that
   makes the honest-unknown argument and names the
   machine-dependence of the correlation structure as part of
   the problem.
5. **Under-claim: the sequential path is a ready-made oracle.**
   The sequential fallback is bit-reproducible today, which
   means the eventual regression test can assert "parallel
   matches sequential" against a known-good reference rather
   than needing to generate a pre-recorded tape. That is a
   non-trivial gift and the chapter was silent on it. *Fix:*
   added a paragraph inside the latency leg naming the
   sequential path as an oracle and noting that chapter 15
   inherits it for free.
6. **The "honest note about naming" paragraph over-defended the
   test author.** Roughly 100 words to say "not the author's
   fault." *Fix:* compressed to two sentences. The first
   acknowledges that the missing coverage is a defect in the
   test suite as a whole (which the previous version was not
   saying cleanly). The second explains how the gap opened
   without blaming anyone.
7. **Urgency argument was leaking into fix territory.** The
   original conclusion said "the target chapters 14 and 15
   design against" — a directive to the downstream chapters
   about how the fix should be designed. It also reintroduced
   the "compounding debugging cost" claim that the
   worst-time-to-debug section never actually established.
   *Fix:* dropped condition (e) from the urgency summary,
   reframed the deadline as a property of the problem ("the
   window closes the first time ...") rather than a directive,
   and softened "expected cost not bounded by a short horizon"
   to the concrete cost-model sentence ("a step function that
   jumps the day the flag flips, plus a tail ...").
8. **Wording fixes from findings 7 and 10.** Changed "real in
   the tree" to "in the tree under the `parallel` feature" at
   several places, to keep the conditional framing honest.
   Rewrote the four enabling events so they are not all framed
   as single-action triggers: events 1 and 2 are single-action
   from inside the project, event 3 is conditional on the
   refactor itself (and is a verification pathway more than a
   discovery pathway), event 4 involves a non-project actor.
9. **Follow-up cleanup.** The opening and closing paragraphs
   still referenced "three feeder chapters" and "three-legged
   tripod" after the body had been restructured to four facts.
   *Fix:* rewrote both paragraphs to use the new language.

**Lower-severity items noted but not acted on:**

- Cold reader suggested considering dropping the word "latent"
  in favor of "dormant." Kept "latent" because the chapter
  title is locked in to SUMMARY.md and the word has a specific
  history in software engineering that matches the chapter's
  usage closely enough. The three-condition definition does
  the work that matters; the word choice itself is secondary.
- Cold reader flagged "compounds with each future researcher"
  as unquantified. Replaced with "recurs" in the urgency
  section; the word "compounds" no longer appears in the
  chapter.

## Second round

**Triggered:** No. All nine edits were targeted revisions to
either restructuring (findings 2 and 3), adding missing
material (findings 4 and 5), or tightening connective tissue
(findings 1, 6, 7, 8, 9). None invalidated the chapter's
structure or its central claim. The chapter's argument — "the
defect is real under the parallel feature, the feature is
default-off but reachable in one step, the research direction
depends on the affected component, and the window closes the
first time any workspace caller flips the switch" — holds after
the edits and is now better-supported and less rhetorical than
the original draft.

## Open questions carried forward

- **Magnitude of $\bar\rho$.** Chapter 13 names this as an
  honest unknown. If later work gives us a way to bound it
  (a measurement, a theoretical argument, or an empirical
  experiment), the chapter should be revisited. Not currently
  blocking.
- **Downstream consumer exposure.** Chapter 13 lists "external
  user enables the feature" as the least-protected enabling
  event. Whether the project should take any protective action
  (release notes, compile-time warning, feature-gate guard) is
  out of scope for the study but is a project-management
  question worth revisiting once chapters 14/15 land.

## Status

Drafted, factual pass run with 12/12 claims verified (six
source-level, six feeder-attribution), thinking pass run with
nine edits applied, no second round triggered. Ready for commit
(pending user permission).
