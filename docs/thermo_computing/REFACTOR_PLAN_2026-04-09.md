# Thermo-Computing Docs Refactor Plan — 2026-04-09

Paper-only artifact. Drafted in response to recon log [part 13](./MASTER_PLAN.md#7-recon-log)
(Q5 resolution) and the rocket-platform reframe of the same session. Dies
after the refactor lands. Read this before any file moves.

## 0. Why this refactor exists

`docs/thermo_computing/` is at the edge of its single-file scale:

- `MASTER_PLAN.md` is now 2610 lines after part 13. The recon log alone
  spans lines 837-end, with 14 entries that all live in one file. Every
  recon round churns the same file; every section edit risks touching
  unrelated content.
- `THERMO_CHASSIS_DESIGN.md` is 2743 lines.
- The Phase 1 spec (`PHASE_1_LANGEVIN_THERMOSTAT_SPEC.md`, not yet
  drafted) will add 300-500 more lines if it lands as another monolith.
- The Q5 finding made the conflation between research-direction
  ranking and build-order ranking concrete: §2.4 was secretly answering
  three different questions with one ladder.

The rocket-platform principle says fix the foundation before it
breaks. The cheapest moment in the project's life to refactor the
doc tree is right now — the master plan is the largest it has ever
been but smaller than it will ever be again.

## 1. Principles

- **Split, not rewrite.** Existing content moves to new homes with
  the *minimum* edits necessary to make the split coherent. Content
  rewrites are deferred to follow-on commits, not bundled into the
  refactor.
- **One commit per logical step.** Pure moves first, then extracts,
  then the recon log split, then index files. Each commit small
  enough to revert individually.
- **Cross-references preserved.** The recon log entries reference
  each other extensively ("see part 4," "per part 9 finding 2,"
  etc.). Historical part numbers are preserved in filenames and in
  text. No find-and-replace across entries.
- **Append-by-adding-files.** The recon log stops being an
  append-to-monster-file and becomes append-by-adding-files. New
  parts get new filenames.
- **`MASTER_PLAN.md` becomes a pointer file**, not deleted, so
  external references (memory files, future links) keep working.
- **Phase 1 spec drafting is out of scope.** Separate session,
  separate plan, lands in the new structure once it exists.

## 2. Target structure

```
docs/thermo_computing/
├── README.md                        ← NEW: one-paragraph "what this is"
├── SUMMARY.md                       ← NEW: mdbook-style index
├── MASTER_PLAN.md                   ← REDUCED to a pointer file
├── 01_vision/
│   ├── vision.md                    ← from MASTER_PLAN §1 Vision (the endpoint)
│   ├── research_directions.md       ← from MASTER_PLAN §2 D1–D5 + Foundational connection (UNRANKED, foundation status added)
│   └── synthesis.md                 ← from MASTER_PLAN §2 Synthesis subsection
├── 02_foundations/
│   ├── working_principles.md        ← from MASTER_PLAN §0 Working Principles — Sharpen the Axe
│   ├── current_state.md             ← from MASTER_PLAN §3 "Current state of the thermo line" (the live snapshot)
│   ├── existing_substrate.md        ← from MASTER_PLAN §3 "What already exists" + "What does not yet exist" + "Adjacent capabilities"
│   ├── chassis_design.md            ← PURE MOVE: THERMO_CHASSIS_DESIGN.md
│   └── open_questions.md            ← from MASTER_PLAN §5
├── 03_phases/
│   ├── overview.md                  ← from MASTER_PLAN §4 The Gap (intro + Phase 1 spec + Phases 2–7)
│   └── (per-phase spec files added later, not in this refactor)
├── 04_recon_log/
│   ├── README.md                    ← NEW: explains append-by-adding-files + numbering
│   ├── 2026-04-09_part_00a_initial_scaffold.md
│   ├── 2026-04-09_part_00b_research_directions_added.md
│   ├── 2026-04-09_part_02_forward_step.md
│   ├── 2026-04-09_part_03_qfrc_applied.md
│   ├── 2026-04-09_part_04_cb_passive.md
│   ├── 2026-04-09_part_05_qfrc_applied_patterns.md
│   ├── 2026-04-09_part_06_prng.md
│   ├── 2026-04-09_part_07_qfrc_passive_pub.md
│   ├── 2026-04-09_part_08_timestep_variability.md
│   ├── 2026-04-09_part_09_test_infrastructure.md
│   ├── 2026-04-09_part_10_module_location.md
│   ├── 2026-04-09_part_11_chassis_complete.md
│   ├── 2026-04-09_part_12_doc_review.md
│   └── 2026-04-09_part_13_q5_cf_design.md
├── 05_doc_reviews/
│   └── 2026-04-09_doc_review.md     ← PURE MOVE: DOC_REVIEW_2026-04-09.md
└── 06_findings/
    └── (empty for now — directory exists for future cross-cutting deep dives)
```

Note: `priority_ladder.md` does **not** exist. This is per the
refined Framing 3 decision — the build-order ladder isn't yet an
operational artifact (only Phase 1 is in flight), so it doesn't
earn a doc. The information lives in `03_phases/overview.md` for
phases and in the per-direction `earliest phase` field of
`research_directions.md` for directions.

## 3. File-by-file mapping

Each existing file's content is classified, with destination paths.
"Classification" types:

| Type | Meaning |
|------|---------|
| `PURE MOVE` | File moves to new path. Zero content edits. |
| `EXTRACT` | A subsection of the source becomes a new file. Content preserved verbatim except for header level adjustment and a one-sentence "extracted from MASTER_PLAN §X" note at the top. |
| `SPLIT` | One file becomes N files. Content preserved verbatim, distributed across the new files. |
| `NEW` | File does not exist; written from scratch in this refactor. |
| `EDIT` | Content edit beyond a pure move/extract. Used sparingly. |
| `DELETE` | File removed; content goes elsewhere or is dropped. |
| `REDUCE` | File is shortened but not deleted (e.g., MASTER_PLAN → pointer). |

### 3.1 `MASTER_PLAN.md` (2610 lines) — REDUCE + EXTRACT + SPLIT

The largest source. Sections distributed as follows. Mapping was
corrected during execution against the actual file structure — see
§3.6 for the four discoveries that motivated the corrections.

| Source range | New location | Type | Notes |
|---|---|---|---|
| §0 Working Principles — Sharpen the Axe | `02_foundations/working_principles.md` | EXTRACT | Operating discipline for the initiative. Slow-changing. (Added §3.6 #1.) |
| §1 Vision (the endpoint) | `01_vision/vision.md` | EXTRACT | Aspirational endpoint + "Why CortenForge is the right substrate" + "Headline research claim." Filename matches content. (Was `the_gap.md` in the original §3.1; renamed §3.6 #2.) |
| §2 Research Directions (intro + D1–D5 + Foundational connection) | `01_vision/research_directions.md` | EXTRACT + EDIT | Drop the Priority ladder subsection. Add `Foundation status:` field per direction. D3 → BLOCKED on Q5. D5 → NOT RECON'D. D1, D2, D4 → viable. **This is the only EDIT in the refactor.** |
| §2 Synthesis subsection (the "CortenForge becomes the only..." quote) | `01_vision/synthesis.md` | EXTRACT | Carries the narrative weight that the rank-1 slot used to carry. |
| §2 Priority ladder subsection | (deleted) | DELETE | Per refined Framing 3. Information preserved via `03_phases/overview.md` + per-direction `earliest phase` fields. |
| §3 "Current state of the thermo line" subsection | `02_foundations/current_state.md` | EXTRACT | The live snapshot. Updated post-refactor in a separate commit (see §7 follow-ons). |
| §3 "What already exists" + "What does not yet exist" + "Adjacent capabilities" subsections | `02_foundations/existing_substrate.md` | EXTRACT | Slow-changing substrate reference, positive AND negative space. ("What does not yet exist" added §3.6 #3.) |
| §4 The Gap (intro narrative + Phase 1 spec + Phases 2–7) | `03_phases/overview.md` | EXTRACT | All §4 content. The intro narrative ("phases ordered by dependency...") becomes the file's preamble. (Was "§4 Phases" in the original §3.1; corrected §3.6 #2.) |
| §5 Open Questions (Q1-Q6) | `02_foundations/open_questions.md` | EXTRACT | Q-numbered register. Q5 status updated post-refactor. |
| §6 Spec Index | `README.md` | EXTRACT | Tiny section (~5 lines). Folds into the top-level README — "what specs exist" is a navigation question. |
| §7 Recon Log | `04_recon_log/*.md` | SPLIT | 14 entries → 14 files. See §3.4 below. |
| Top-level pointer | `MASTER_PLAN.md` | REDUCE | What remains: ~30-line pointer file with links to the new structure. |

### 3.2 `THERMO_CHASSIS_DESIGN.md` (2743 lines) — PURE MOVE

Moves to `02_foundations/chassis_design.md`. Zero content edits.
The file is dense and well-organized; splitting it further is out
of scope (no maintenance pain right now).

### 3.3 `DOC_REVIEW_2026-04-09.md` (688 lines) — PURE MOVE

Moves to `05_doc_reviews/2026-04-09_doc_review.md`. Zero content
edits.

### 3.4 The recon log split (the largest piece)

14 entries become 14 files. Filenames preserve historical part
numbers per the locked decision:

| Source line | Heading | Filename |
|---|---|---|
| 842 | Initial scaffold | `2026-04-09_part_00a_initial_scaffold.md` |
| 875 | Research Directions added | `2026-04-09_part_00b_research_directions_added.md` |
| 901 | (part 2) Forward step + integrator code recon | `2026-04-09_part_02_forward_step.md` |
| 963 | (part 3) Phase-1-blocking item 2: `qfrc_applied` lifecycle | `2026-04-09_part_03_qfrc_applied.md` |
| 1063 | (part 4) Item 3 first finding: `cb_passive` exists | `2026-04-09_part_04_cb_passive.md` |
| 1209 | (part 5) Item 3 closed: existing `qfrc_applied` write patterns | `2026-04-09_part_05_qfrc_applied_patterns.md` |
| 1329 | (part 6) Item 4: PRNG conventions | `2026-04-09_part_06_prng.md` |
| 1552 | (part 7) Item 5: `Data::qfrc_passive` public mutability | `2026-04-09_part_07_qfrc_passive_pub.md` |
| 1620 | (part 8) Item 6: `model.timestep` variability | `2026-04-09_part_08_timestep_variability.md` |
| 1748 | (part 9) Item 7: test infrastructure conventions | `2026-04-09_part_09_test_infrastructure.md` |
| 1968 | (part 10) Item 8: thermostat module location | `2026-04-09_part_10_module_location.md` |
| 2196 | (part 11) Chassis design round complete | `2026-04-09_part_11_chassis_complete.md` |
| 2279 | (part 12) Doc review pass | `2026-04-09_part_12_doc_review.md` |
| 2390 | (part 13) Q5 cf-design end-to-end differentiability | `2026-04-09_part_13_q5_cf_design.md` |

**Numbering note**: parts 0a and 0b are the historical unnumbered
"scaffold" entries; parts start at 2 (no part 1 ever existed). This
oddity is preserved verbatim — documented in
`04_recon_log/README.md` so future readers don't trip on it.

**Cross-reference handling**: every entry's text references other
entries by part number ("see part 4," "per part 9 finding 2"). These
references stay as plain text — they are *not* rewritten as file
links. Reasons:
- Plain text is grep-able and doesn't break if files are renamed.
- The references are historical claims about a record, not
  navigation.
- Adding 100+ link rewrites violates the "split, not rewrite"
  principle.

`04_recon_log/README.md` will document the convention: "References
of the form 'part N' point to `2026-04-09_part_NN_*.md`."

### 3.5 New files

| File | Lines (estimated) | Purpose |
|---|---|---|
| `README.md` | ~30 | One-paragraph "what is this initiative" + top-level navigation. |
| `SUMMARY.md` | ~40 | mdbook-style index with all paths. Optional for `mdbook build`. |
| `01_vision/synthesis.md` | ~30 | Extracted, but standalone enough to count as effectively-new shaping. |
| `04_recon_log/README.md` | ~25 | Numbering convention, append-by-adding-files convention, cross-reference convention. |

### 3.6 Pre-execution discovery + corrections

When the executing session began reading `MASTER_PLAN.md` to confirm
the §3.1 mapping against the actual file, four gaps were found in
the original §3.1 table. The plan was paused, the gaps surfaced, and
the corrections below were applied before any file moves.

1. **§0 Working Principles was unmapped.** A ~45-line section with
   no row in §3.1. It would have been silently dropped during the
   `MASTER_PLAN.md → pointer` reduction. Resolved: extracted to
   `02_foundations/working_principles.md`. The operating discipline
   is foundational to the initiative; the doc tree carries it
   directly rather than relying on operator memory.

2. **§1 / §4 label mismatch.** The original §3.1 labeled §1 as
   "The Gap" (→ `the_gap.md`) and §4 as "Phases (1-7)"
   (→ `overview.md`), but the actual `MASTER_PLAN.md` has §1 =
   "Vision (the endpoint)" and §4 = "The Gap (working both ways
   toward the middle)" containing the phase material. Resolved:
   §1 Vision → `01_vision/vision.md` (filename matches content);
   §4 The Gap → `03_phases/overview.md` (all §4 content including
   intro narrative and the phase status table). No `the_gap.md`
   file is created — the gap framing is captured in `overview.md`'s
   preamble, not as a standalone file.

3. **§3 "What does not yet exist" subsection was unmapped.** The
   original §3.1 mapped "What already exists" + "Adjacent
   capabilities" to `existing_substrate.md` but omitted the
   "What does not yet exist" subsection (~24 lines) that sits
   between them in the source. Resolved: added to
   `existing_substrate.md`. Slow-changing substrate reference
   covers both positive and negative space.

4. **§2 sub-numbering references were stale.** The original §3.1
   used "§2.0–2.5" and "§2.4 priority ladder", but actual §2 has
   no `§2.N` numbering — only `### D1`–`### D5`,
   `### Foundational connection`, `### Synthesis`, and
   `### Priority ladder` subsections. Resolved: row labels now
   reference actual subsection titles. No content impact.

The corrections were applied to §3.1 (mapping table), §2 (target
tree), and §4 (order of operations — added a new step for §0
extraction). The rubric was re-graded in
`REFACTOR_PLAN_RUBRIC_2026-04-09.md` §5b. The corrected plan is
shippable; execution proceeds against the corrected plan.

## 4. Order of operations

Each step is its own commit. Each commit is small enough to revert
independently. The order is chosen so that no commit leaves the
tree in a broken state.

| # | Step | What it does | Why this order |
|---|------|---|---|
| 1 | Create directories | `mkdir 01_vision/ 02_foundations/ 03_phases/ 04_recon_log/ 05_doc_reviews/ 06_findings/`. Empty dirs. | Lets subsequent moves land in valid locations. |
| 2 | Pure moves | `git mv THERMO_CHASSIS_DESIGN.md 02_foundations/chassis_design.md`, `git mv DOC_REVIEW_2026-04-09.md 05_doc_reviews/2026-04-09_doc_review.md`. | Zero content risk. Done first to clear simple work. |
| 3 | Extract §0 Working Principles | Create `02_foundations/working_principles.md`. Remove §0 from `MASTER_PLAN.md`. | Smallest extract, in source order. Validates the extract pattern. (Added §3.6 #1.) |
| 4 | Extract §1 Vision | Create `01_vision/vision.md`. Remove §1 from `MASTER_PLAN.md`. | Mechanical extract. (Was "Extract §1 The Gap" in the original step list; corrected §3.6 #2.) |
| 5 | Extract §2 Research Directions | Create `01_vision/research_directions.md` (unranked, foundation status added) + `01_vision/synthesis.md`. Remove §2 from `MASTER_PLAN.md`. **The only EDIT in the refactor.** | The load-bearing change. Done early so subsequent work doesn't need to re-edit it. |
| 6 | Extract §3 Current State | Create `02_foundations/current_state.md` and `02_foundations/existing_substrate.md`. Remove §3 from `MASTER_PLAN.md`. | Mechanical extract. |
| 7 | Extract §4 The Gap | Create `03_phases/overview.md` (intro narrative + Phase 1 spec + Phases 2–7). Remove §4 from `MASTER_PLAN.md`. | Mechanical extract. (Was "Extract §4 Phases" in the original step list; corrected §3.6 #2.) |
| 8 | Extract §5 Open Questions + §6 Spec Index | Create `02_foundations/open_questions.md`. Remove §5+§6 from `MASTER_PLAN.md`. (§6 content is folded into the top-level `README.md` in step #11.) | Mechanical extract. |
| 9 | Split §7 Recon Log | Create 14 files in `04_recon_log/`. Remove §7 from `MASTER_PLAN.md`. **Largest commit by file count, but each file is a verbatim copy of its source range.** | Done after all other extracts so `MASTER_PLAN.md` is now ~30 lines and easy to convert. |
| 10 | Reduce `MASTER_PLAN.md` to pointer file | Rewrite the remaining file as a pointer with links to the new structure. | Last content commit. |
| 11 | Write index files | `README.md` (with §6 Spec Index folded in), `SUMMARY.md`, `04_recon_log/README.md`, `06_findings/.gitkeep`. | Index after content. |
| 12 | Verify cross-references | Grep for stale "MASTER_PLAN" references inside `docs/thermo_computing/` and update where appropriate (footers, "see also" pointers in chassis design, doc review). External references (memory files) updated separately as a follow-on. | Single pass at the end. |

Estimated: ~12 commits (was 11 in the original plan; +1 for §0
extraction added in §3.6 #1). Plan is to do them all in one
focused session, plus one additional commit (this one) that lands
the §3.1 / §3.6 / §4 corrections themselves before execution
begins.

## 5. Decisions locked (from prior conversation)

- **Filename scheme**: `2026-04-09_part_NN_<slug>.md`, historical
  numbering preserved verbatim, oddity (no part 1) documented in
  `04_recon_log/README.md`.
- **`MASTER_PLAN.md` becomes a pointer file**, not deleted. Three
  external memory references stay live.
- **`SUMMARY.md` is written**, even if mdbook isn't used today.
  Doubles as a top-level index.
- **Same branch** (`feature/thermo-doc-review`). Refactor commits
  land alongside the doc review commits. Branch may be renamed at
  PR time.
- **`research_directions.md` is unranked**. Per-entry metadata
  only. `Foundation status` field added. D3 → BLOCKED on Q5. D5 →
  NOT RECON'D. D1/D2/D4 → viable.
- **`priority_ladder.md` is not created.** Build-order ladder
  isn't operational yet; the information lives in
  `03_phases/overview.md` and per-direction `earliest phase`
  fields.

## 6. Explicit non-scope

The refactor will **NOT**:

- Touch the chassis design content. (Pure move only.)
- Touch the doc review content. (Pure move only.)
- Touch recon log entry content. (Split only — text preserved
  verbatim.)
- Rewrite cross-references in recon log entries. (Plain-text
  "part N" references stay as plain text.)
- Draft the Phase 1 spec.
- Update memory files (`MEMORY.md`, `project_thermo_computing.md`,
  `feedback_sharpen_the_axe.md`). Memory updates are a separate
  follow-on commit on the same branch.
- Rewrite the recon log to reorganize parts by topic. Append-only
  history is preserved.
- Write any code, any Cargo.toml, any new sim/L0/thermostat/
  directory contents.

## 7. Open follow-ons (after refactor)

These are real edits, but they happen *after* the refactor lands,
in their own commits, against the *new* small files (not the old
monolith). Listed here so the refactor doesn't try to absorb them.

Note: foundation-status fields for D1–D5 (including D3 BLOCKED and
D5 NOT RECON'D annotations) are *part of* the refactor itself —
specifically the EDIT in step #4 — not follow-ons. They are not
listed below.

| Follow-on | Target file | Notes |
|---|---|---|
| Update §3 Q5 status line: ESCALATED → RESOLVED (NO) | `02_foundations/current_state.md` | Reflects part 13 finding. |
| Rewrite §5 Q5 entry to reflect resolution | `02_foundations/open_questions.md` | Link to part 13 file. |
| Add Q7 — D5 viability recon as new entry in open questions | `02_foundations/open_questions.md` | Placeholder; recon happens after D1-D4 close. |
| Soften "differentiable design optimization" phrasing | `02_foundations/current_state.md` + `MEMORY.md` + `project_thermo_computing.md` | Honest description per part 13. |
| Update memory file paths | `MEMORY.md`, `project_thermo_computing.md`, `feedback_sharpen_the_axe.md` | "MASTER_PLAN.md" references updated to point at new structure. Pointer file keeps old refs alive in the meantime. |
| Decide whether to surface the cf-design fixable break (analytic mass properties) as a cf-design follow-on | (separate from thermo line) | Out of scope; flag to user when refactor lands. |
| Optional cross-cutting deep dive: `06_findings/q5_cf_design_differentiability.md` | `06_findings/` | Not needed unless future sessions want a more detailed write-up than part 13 already provides. |

## 8. Rollback plan

- Single PR on `feature/thermo-doc-review`.
- Each step in §4 is its own commit, small enough to `git revert`
  individually.
- The recon log split commit (#8) is the largest by file count but
  contains zero content edits — easy to verify by `git diff -M`
  after the commit.
- If the entire refactor needs to be backed out: `git reset --hard
  c9b7a7f` (the part 13 commit) returns the tree to the
  pre-refactor state. Branch never gets force-pushed during the
  refactor.

## 9. Validation criteria

After the refactor lands, the new structure should pass these
checks:

1. **No content lost**: `git log --stat` of the refactor commits
   shows the expected net delta. Computed: EXTRACT operations add
   ~35 lines (header + "extracted from" notes across ~10
   extracts); the foundation-status field additions in step #4
   add ~25 lines (5 fields × 5 directions); the new index files
   (`README.md`, `SUMMARY.md`, `04_recon_log/README.md`) add ~95
   lines; the `MASTER_PLAN.md` reduction shrinks the source by
   ~2580 lines and adds back ~30 as a pointer file. Net:
   `35 + 25 + 95 - 2580 + 30 ≈ -2395` lines in `MASTER_PLAN.md`
   itself, balanced by `+2580` lines distributed across the new
   files, for a workspace-wide net of **+150 lines ± 50**.
   Anything outside this band (e.g., -500 or +500) means content
   was silently dropped or smuggled in — investigate before
   continuing.
2. **No broken cross-references inside `docs/thermo_computing/`**:
   grep for stale "§N" or "MASTER_PLAN.md#" anchors that no longer
   exist. Plain-text "part N" references in the recon log are
   intentionally preserved.
3. **Memory files still point to working content** (via the
   `MASTER_PLAN.md` pointer file).
4. **`research_directions.md` carries no global ranking**, only
   per-entry metadata. D3 is annotated `Foundation status: BLOCKED
   on Q5 (recon log part 13)`. D5 is annotated `NOT RECON'D`.
5. **No `priority_ladder.md` exists.**
6. **Each recon log entry is its own file**, named per the §3.4
   table.
7. **`THERMO_CHASSIS_DESIGN.md` and `DOC_REVIEW_2026-04-09.md`
   moved verbatim** — `git diff -M` against their old locations
   shows zero content change, only the rename.

## 10. After the refactor

The next two artifacts that build on this structure:

1. **Memory updates** (separate commit, same branch). Update
   `MEMORY.md`, `project_thermo_computing.md`, and
   `feedback_sharpen_the_axe.md` to reflect the new file paths and
   the Q5 finding.
2. **Phase 1 spec drafting** (separate session). Lands as
   `03_phases/01_langevin_thermostat.md` against the new
   structure. The chassis design is the input; the spec is the
   output.

Both are out of scope for the refactor itself.

---

**End of plan.** Read and pushed back on, then executed in a
single focused session.
