# Refactor Plan Grading Rubric — 2026-04-09

A grading rubric for `REFACTOR_PLAN_2026-04-09.md`. Inspired by
`cargo xtask grade`'s 7-criterion structure (A-grade or it doesn't
ship). Applied to a paper artifact instead of code.

The rubric is the *standard* the plan must meet before any files
move. Read the plan, then read the rubric, then grade the plan,
then either execute or fix gaps until every criterion is A.

## 0. Why grade a plan?

The rocket-platform principle says foundations before payload. The
refactor *is* the foundation for everything that lands in
`docs/thermo_computing/` from this point forward. A B-grade
refactor plan executed at full speed produces a B-grade foundation,
and every artifact built on top inherits the grade. The cost of
grading the plan is one document; the cost of refactoring the
refactor is unbounded.

## 1. Principles

A good refactor plan has these properties:

- **Closed-loop on inputs and outputs.** Every existing file maps
  to a destination. Every new file has a stated source.
- **Reversible at every step.** Each commit can be backed out
  without breaking subsequent commits.
- **Honest about what it isn't doing.** Explicit non-scope shrinks
  the surface where mistakes can hide.
- **Safe to execute by a fresh session.** A reader who has not
  participated in the conversation can read the plan and execute
  it correctly.
- **Verifiable after execution.** Concrete checks let a third
  party confirm the refactor was correct.

These principles motivate the seven criteria below.

## 2. The seven criteria

### C1 — Coverage

**What it tests**: Does the plan account for every byte of input
and explicitly destine every byte of output? Are there any
"orphans" — content in the source that the plan doesn't say where
it goes, or new files in the target that the plan doesn't say
where they came from?

| Grade | Definition |
|---|---|
| **A** | Every section of every existing file maps to a destination. Every new file declares its source (or is marked NEW with a written purpose). No "deferred to execution time" decisions about where content lands. |
| **B** | One or two small sections have ambiguous destinations or "decide later" notes. Aggregate is < 5% of total content. |
| **C** | Multiple "decide later" decisions, or one major section (> 100 lines) has no clear destination. |
| **D** | Large gaps; multiple sections unmapped; the plan can't be executed without further design rounds. |
| **F** | Plan does not enumerate inputs at all; reader has to infer what's being moved. |

**How to evaluate**: For each existing file in
`docs/thermo_computing/`, find the row(s) in the plan that name
its destination. For each new directory and file in the target
structure, find the row that names its source. Any unfound row is
a coverage gap.

---

### C2 — Reversibility

**What it tests**: Can each commit be reverted independently? Is
there a clean rollback path if execution goes wrong? Is the
branching strategy safe?

| Grade | Definition |
|---|---|
| **A** | Each step in the order-of-operations is its own commit, each commit independently revertable, explicit rollback target named (a commit hash to reset to), no force-push during execution, single PR. |
| **B** | Most steps are independent commits but one or two are bundled; rollback target named but reset path is multi-commit. |
| **C** | Some steps depend on prior steps in ways that make individual revert risky. Rollback path is "just revert all the commits." |
| **D** | Plan describes a single monolithic commit with no rollback strategy. |
| **F** | Force-push or branch-replacement strategy that destroys history. |

**How to evaluate**: Walk the order-of-operations. For each step,
ask: "if I revert this commit and stop, is the tree in a coherent
state?" Then ask: "if I want to back out the entire refactor mid-
stream, what's the path?"

---

### C3 — Scope Discipline

**What it tests**: Does the plan explicitly name what it is *not*
doing? Does it resist the urge to bundle in adjacent work that
"would be easy while we're in there"?

| Grade | Definition |
|---|---|
| **A** | Explicit "non-scope" section enumerates ≥ 5 things the refactor will NOT touch, including the obvious temptations (memory updates, content rewrites, phase spec drafting). Follow-ons that depend on the refactor are listed separately as their own commits. |
| **B** | Non-scope section exists but is shallow or misses one obvious temptation. |
| **C** | Non-scope is implicit (you have to infer what's not being touched from what is). |
| **D** | The plan smuggles in adjacent rewrites without flagging them. |
| **F** | The plan is a refactor + spec + rewrite + memory pass bundled together. |

**How to evaluate**: Look for an explicit non-scope section. Count
the items. Check whether the obvious temptations (drafting the
next spec, updating memory, rewriting the recon log entries) are
named.

---

### C4 — Sequencing Safety

**What it tests**: Is the order of operations chosen such that no
intermediate commit leaves the tree in a broken state? Does each
step's "why this order" make sense?

| Grade | Definition |
|---|---|
| **A** | Each step has a stated reason for its position. No commit leaves the tree in a state where another tool (cargo, hooks, links) breaks. Pure moves first, content edits in the middle, large mechanical splits last among content commits, indexes after content. |
| **B** | Order is sensible but one step's position is unjustified or could be tightened. |
| **C** | Order works but one intermediate commit leaves something inconsistent (e.g., a stale link surfaces between commits 4 and 5 but is fixed by commit 6). |
| **D** | Order is arbitrary; some commits would fail if applied alone. |
| **F** | Plan provides no order; all changes happen at once. |

**How to evaluate**: For each step in the sequence, simulate
applying only the commits up to and including that step. Ask: "is
the tree coherent at this point? Would a `git status` + `mdbook
build` (or equivalent) succeed?"

---

### C5 — Content Fidelity

**What it tests**: Does the plan preserve content where it claims
to? Are historical artifacts (numbering, cross-references,
verbatim sections) honored? Where edits exist, are they minimized
and named explicitly?

| Grade | Definition |
|---|---|
| **A** | Plan classifies every move as `PURE MOVE` / `EXTRACT` / `SPLIT` / `EDIT` and only the explicitly-named EDITs touch content. Verbatim claims are honest (header-level adjustment and "extracted from" notes are documented as the only allowed edits inside an EXTRACT). Cross-reference handling is documented. Historical numbering oddities are preserved. |
| **B** | Mostly verbatim, but one EXTRACT silently rewrites a heading or fixes a typo without flagging it. |
| **C** | EDITs creep into operations claimed as PURE MOVE or SPLIT. Cross-references are rewritten without explicit policy. |
| **D** | Plan claims "split, not rewrite" but actually rewrites significant content. |
| **F** | Plan is a rewrite disguised as a refactor. |

**How to evaluate**: Find the row(s) classified as EDIT. Verify
that they are the *only* rows where content changes. Check the
plan's policy on cross-references and header levels. Check that
historical oddities (e.g., "no part 1 ever existed") are
preserved, not "fixed."

---

### C6 — Verifiability

**What it tests**: After execution, can a third party (or future-
you) verify the refactor was correct? Are the post-execution
checks concrete enough to be runnable?

| Grade | Definition |
|---|---|
| **A** | Plan has a validation section with ≥ 5 concrete, runnable checks. Each check has a clear pass/fail outcome. Line-conservation bounds are tight enough to catch real drift. `git diff -M` is named for rename verification. |
| **B** | Validation section exists with concrete checks but one or two are vague ("looks right") or have loose bounds. |
| **C** | Validation is one-paragraph "make sure nothing is lost." No specific commands. |
| **D** | No validation section. Verification is left to vibes. |
| **F** | Plan claims success criteria that aren't checkable (e.g., "reads better"). |

**How to evaluate**: Read the validation section. For each check,
ask: "could I run this as a command? What's the pass condition?
Would it actually catch the failure mode I'm worried about?"

---

### C7 — Executability

**What it tests**: Could a fresh session — one that has not
participated in the conversation — read the plan and execute it
correctly?

| Grade | Definition |
|---|---|
| **A** | Plan is self-contained. Every decision is locked or has a stated lean. Filenames, commands, sequence, and rationale are all in the document. No "ask user during execution" footnotes. Cross-references between sections of the plan are unambiguous. |
| **B** | Mostly self-contained but one or two decisions need user input mid-execution. |
| **C** | Plan requires reader to consult conversation history or recon log to fill gaps. |
| **D** | Plan is a sketch that requires the original author to interpret. |
| **F** | Plan is unintelligible without the author present. |

**How to evaluate**: Imagine handing the plan to a fresh session
with no context. Walk through the order of operations and ask:
"can I execute this step from the plan alone, or do I need to
look something up?"

---

## 3. Aggregate grade

The plan ships if and only if every criterion is **A**. One B
anywhere is a stop-the-line event — fix the gap, regrade, then
ship. This mirrors `cargo xtask grade`'s "A-grade or it doesn't
ship" rule applied to documentation.

The cost of fixing a B is minutes. The cost of executing on a B
plan is hours, and the cost of un-doing a botched refactor is
days. Always pay the minutes.

---

## 4. Self-assessment of `REFACTOR_PLAN_2026-04-09.md`

Applied 2026-04-09 by Claude (drafting session).

| # | Criterion | Grade | Reasoning |
|---|---|---|---|
| C1 | Coverage | **A−** | Every existing file is mapped, every new file declares its source. Gap: §6 Spec Index destination is "deferred to execution time" between `02_foundations/open_questions.md` and the top-level `README.md`. That's a 5-10 line section with no committed home. **Fix**: lock the destination now. |
| C2 | Reversibility | **A** | 11 independent commits, explicit rollback target (`c9b7a7f` = the part 13 commit), single PR, no force-push, each step is its own logical unit. |
| C3 | Scope Discipline | **A** | §6 enumerates 8 explicit non-scopes including all the obvious temptations (Phase 1 spec, memory updates, content rewrites, code). Follow-ons table separates "after refactor" work cleanly. |
| C4 | Sequencing Safety | **A** | Each of the 11 steps has a stated reason for its position. Pure moves first, smallest extract second (validates the pattern), recon log split last among content commits, index files after content. No intermediate commit leaves the tree broken. |
| C5 | Content Fidelity | **A** | Every operation is classified. Only one EDIT exists (step #4, adding foundation-status fields to research_directions.md). Verbatim claims explicitly allow header-level adjustment + a one-line "extracted from §X" note, which is honest. Cross-reference handling is documented (plain-text "part N" stays as plain text). Historical numbering oddity (no part 1) is preserved. |
| C6 | Verifiability | **A−** | §9 has 7 concrete checks. Gap: the line-conservation criterion is fuzzy ("on the order of +50 to +100 lines, not +1000"). The bound is right but loose. **Fix**: tighten by computing the expected delta from the EXTRACT operations + the foundation-status field additions, then bound it as `expected ± 30 lines` or similar. |
| C7 | Executability | **A−** | Plan is largely self-contained. Gap: §7 follow-ons table contains two rows that say "Already done as part of refactor extract step #4" — those rows describe work that *isn't* a follow-on, they're work that's already inside the refactor. A fresh session reading the table would be confused. **Fix**: remove the two redundant rows; leave only true post-refactor follow-ons. |

**Aggregate**: A− across the board. Three small fixes would close
the gap to clean A. The fixes are listed in the next section and
are minutes of work each.

## 5. Named gaps and fixes (to close to A)

To bring the plan to A on every criterion before execution:

1. **C1 fix** — Lock §6 Spec Index destination. Recommendation:
   absorb into top-level `README.md`, since "what specs exist" is
   a navigation question and the spec index is tiny (~5 lines).
   Update the §3.1 mapping table accordingly. Removes the only
   "deferred to execution time" decision in the plan.

2. **C6 fix** — Tighten the line-conservation validation criterion.
   Compute expected delta: EXTRACT operations preserve content
   verbatim (delta = N header-line additions + N×1 "extracted
   from" notes), the recon log SPLIT is verbatim (delta ≈ 0 plus
   one header per file), the EDIT in step #4 adds ~5 fields × 5
   directions ≈ 25 lines, the new index files (`README.md`,
   `SUMMARY.md`, `04_recon_log/README.md`) add ~95 lines. The
   `MASTER_PLAN.md` reduction shrinks the source by ~2580 lines
   and adds back ~30. Expected net delta: **+150 lines ± 50**.
   Replace the loose "on the order of +50 to +100" with this
   tighter bound.

3. **C7 fix** — Remove the two redundant rows from §7 (the
   follow-ons table) that describe work already inside refactor
   step #4 (D1/D2/D4 foundation-status fields, D5 NOT RECON'D
   annotation). Leave only post-refactor follow-ons. Add a
   one-line note clarifying that the foundation-status work is
   *part of* the refactor, not after it.

After these three fixes, regrade. Expectation: **A across all
seven criteria, plan is shippable**.

## 5a. Post-fix self-assessment

Applied 2026-04-09 by Claude (same drafting session, after the
three fixes from §5 landed in `REFACTOR_PLAN_2026-04-09.md`).

| # | Criterion | Grade | Reasoning |
|---|---|---|---|
| C1 | Coverage | **A** | §6 Spec Index destination locked as top-level `README.md`. Zero "decide later" decisions remaining. |
| C2 | Reversibility | **A** | Unchanged from §4 self-assessment. |
| C3 | Scope Discipline | **A** | Unchanged from §4 self-assessment. |
| C4 | Sequencing Safety | **A** | Unchanged from §4 self-assessment. |
| C5 | Content Fidelity | **A** | Unchanged from §4 self-assessment. |
| C6 | Verifiability | **A** | Line-conservation criterion replaced with computed expected delta: `+150 lines ± 50`, with the computation shown inline. Out-of-band deltas now have a runnable detection threshold. |
| C7 | Executability | **A** | The two redundant rows in §7 follow-ons table are removed. A new clarifying note explicitly distinguishes "part of the refactor" (D1-D5 foundation status) from "after the refactor." Fresh-session reader can no longer be confused about what's during vs. after. The new Q7 entry (D5 viability recon placeholder) is added to the post-refactor follow-ons table. |

**Aggregate**: **A across all seven criteria. Plan is shippable.**

The three fixes were:
- §3.1 row: `02_foundations/open_questions.md (appended) or README.md` → `README.md`
- §9 criterion #1: fuzzy "+50 to +100" → computed `+150 ± 50` with inline derivation
- §7 table: removed two redundant rows, added clarifying note, added Q7 placeholder row

Total time to fix and regrade: ~10 minutes. The cost of grading
the plan was a single document; the cost of executing on a B
plan would have been hours, and the cost of un-doing a botched
refactor would have been days. Always pay the minutes.

## 5b. Pre-execution discovery + re-grade

Applied 2026-04-09 by Claude (executing session, after the §5a
post-fix self-assessment but before any file moves).

When the executing session began reading `MASTER_PLAN.md` to
confirm the §3.1 mapping against the actual file, four gaps were
discovered in the §3.1 table that the §4 self-assessment and the
§5a post-fix self-assessment had both missed. Both prior gradings
had been done from the plan-author's mental model of the source
file, not from a fresh read. The four gaps:

1. **§0 Working Principles was unmapped** (~45 lines, no row in
   §3.1). Would have been silently dropped during the
   `MASTER_PLAN.md → pointer` reduction.
2. **§1 / §4 label mismatch.** §3.1 used "§1 The Gap" and "§4
   Phases" as row labels, but actual `MASTER_PLAN.md` has §1 =
   "Vision (the endpoint)" and §4 = "The Gap (containing the phase
   material)." A fresh executor could not interpret the mapping
   without choosing between two plausible readings.
3. **§3 "What does not yet exist" subsection unmapped** (~24 lines,
   sits between two subsections that *were* mapped to
   `existing_substrate.md`).
4. **§2 sub-numbering references stale** (`§2.0–2.5`, `§2.4`)
   when actual §2 has no `§2.N` numbering.

Strict reading: gaps 1, 2, and 3 dropped **C1 Coverage** below A
(orphaned content + ambiguous-destination content). Gap 2 dropped
**C7 Executability** below A (a fresh executor could not run §3.1
from the plan alone). Gap 4 was cosmetic and did not affect a
grade.

The executing session paused before any file moves, surfaced the
gaps to the user, received green light to fix and proceed,
patched the plan in §3.1, §3.6 (new), §2 (target tree), §4
(order of operations — added a new step #3 for §0 extraction), and
re-graded here. Re-grade after the patches:

| # | Criterion | Grade | Reasoning |
|---|---|---|---|
| C1 | Coverage | **A** | §0, §1, §3 "What does not yet exist", and the §1/§4 ambiguity all have committed destinations in the patched §3.1. Zero orphaned content. Zero "decide later" decisions. New §3.6 explicitly documents the four discoveries and resolutions for future sessions. |
| C2 | Reversibility | **A** | Unchanged. New step count is 12 (was 11); each step is still its own commit with the same independent-revert property. The plan-fix commit (this regrade + the §3.1 patches) is itself a separate revertable commit. |
| C3 | Scope Discipline | **A** | Unchanged. The patches add no new scope — they correct mis-mapped existing content. The §6 non-scope list is untouched. |
| C4 | Sequencing Safety | **A** | Unchanged. New step #3 (§0 extraction) slots cleanly into the source-order extract sequence and validates the extract pattern (smallest extract, in source order). Subsequent step numbers shift by 1 with no ordering change. |
| C5 | Content Fidelity | **A** | Unchanged. The patches rename one destination filename (`the_gap.md` → `vision.md`), add three orphaned source ranges to existing destination files (or new ones), and correct stale section labels. No new EDITs introduced; the only EDIT is still the §2 foundation-status fields in step #5. |
| C6 | Verifiability | **A** | Unchanged. Line-conservation band stays at +150 ± 50 — the four discoveries shift the expected delta by under 30 lines (preserving §0's 45 lines instead of dropping them, preserving §3 "What does not yet exist" 24 lines, etc., counterbalanced by the same lines reappearing in destination files). New step count (12 vs 11) doesn't materially affect the band. |
| C7 | Executability | **A** | Patched §3.1 uses actual section titles and line ranges. New §3.6 documents the discovery process so a *future* fresh session can see how the plan was hardened. The plan can now be executed by a fresh session reading only the plan, without needing to consult `MASTER_PLAN.md` to disambiguate row labels. |

**Aggregate**: **A across all seven criteria. Plan is shippable
post-discovery patches.**

Lesson for future paper-artifact gradings: even a careful self-
assessment can miss what a fresh-eyes execution-time read catches.
A two-pass discipline is worth applying to load-bearing paper
artifacts: (1) author self-grades immediately after drafting, then
(2) execution session does a fresh read and validates the mapping
against the actual file before doing any work. Pass 2 is cheap and
catches what pass 1 systematically misses (the author's mental
model vs. the file).

## 6. Re-grading discipline

Same cadence as the chassis design and the doc review:

- Apply each fix as a single edit to the plan file.
- After all fixes, regrade in this same file (append a new
  self-assessment dated 2026-04-09 (post-fix) below the original
  one, don't rewrite).
- Plan is shippable when the post-fix grade is A across all seven.
- This file (the rubric) is paper-only and lives until the
  refactor is complete. After the refactor lands, the rubric and
  the plan can either be deleted, kept as historical artifacts in
  `05_doc_reviews/`, or moved to `06_findings/` as a worked
  example of "how we graded a refactor." **Decision deferred.**

---

**End of rubric.**
