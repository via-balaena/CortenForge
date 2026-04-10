# Grade Tool Audit — Recon Plan

> **For the next session.** Self-contained brief that bootstraps a fresh
> session with zero conversation memory of how this initiative started.
> Read [`audit_findings_2026-04-09.md`](audit_findings_2026-04-09.md)
> first; this document is the next-session action plan that depends on
> those findings being understood.
>
> **Revised once** alongside the audit findings doc, after the original
> draft's first cold-read pass surfaced 6 new findings (the F section)
> and corrected stale line numbers across all gate functions. The recon
> plan was updated to reflect that D1 (read STANDARDS.md) is now
> partially answered and that the F section needs its own decision
> escalation track separate from the B-section verification track.

## What this initiative is in one paragraph

The xtask grade tool was built and validated against a flat-mesh-style
workspace and never exercised against the layered sim layout. Phase 1
PR prep for `sim-thermostat` (the first sim crate ever put through the
gate) surfaced two latent bugs and an audit of surrounding gate code
revealed substantially more debt — **23 items total** across confirmed
bugs, suspected bugs, architectural debt, open investigation questions,
spec/rubric alignment, and (the F section, surfaced from reading
`docs/STANDARDS.md` end-to-end) six concrete drifts between the tool's
behavior and the canonical standard. The right scope is treating the
grade tool as its own initiative requiring its own recon → audit →
chassis → rebuild plan → execute → grade cycle. Phase 1 PR is parked
pending the audit because Phase 1 spec §12.4 #5 explicitly requires
`cargo xtask grade sim-thermostat` to reach A across 7 criteria, which
is currently impossible under the buggy tool.

The user's operating principle for this work is: **"do it right so we
don't have to go back and do it twice. patience."** Honor it. The
gate-tool layer is *meta* infrastructure — it grades everything else.
Getting it wrong propagates silently into every future crate that
passes through it. Treat this with the same rigor as any code-grade
initiative.

## What this session is for

**The recon round.** Specifically, the read-only investigation that
informs whether the rebuild scope should be A (minimal patch),
B (audit + targeted fix), C (architectural rebuild), or D (split with
spec amendment) — see the options table at the bottom of this document.

You are NOT yet writing a chassis design, NOT yet writing a rebuild
plan, NOT yet touching `xtask/src/grade.rs`. You are gathering enough
information to know which scope of next session to plan.

There are now **three parallel investigation tracks**:

1. **D1-D5 — open questions** (read-only investigation, ~15 min)
2. **B1-B5 — suspected bugs** (targeted experiments, may need test
   crates, ~30-60 min)
3. **F1-F6 — code-vs-standard drift decisions** (read STANDARDS.md
   end-to-end, surface decisions to user, do not unilateral on the
   philosophical ones)

The audit findings doc has captured all three tracks; this recon plan
tells you how to execute each.

## First three actions (mandatory, in order)

1. **Read [`audit_findings_2026-04-09.md`](audit_findings_2026-04-09.md)
   end-to-end.** It is the canonical record of what's known about the
   grade tool's gaps. Every claim is sourced. Read it cold; if anything
   is unclear or contradictory, surface it before doing anything else.
2. **Verify git state.** `git status` should be clean; HEAD should be
   **at or after** `c3f08c5` (the A1 `find_crate_path` fix) on
   `feature/thermo-doc-review`. The audit findings + persistence
   updates from the originating session committed on top of `c3f08c5`,
   so the HEAD by the time you read this will be a few commits later.
   The marker is "the audit findings doc exists in
   `docs/grade_tool_audit/`." If git state has drifted from what the
   audit doc describes, surface and stop.
3. **Decide whether the audit should happen on `feature/thermo-doc-review`
   (where the A1 fix and the audit findings already live) or on a
   fresh branch off main.**
   - Argument for current branch: A1 fix is already there, the audit
     findings doc is already there, the audit is logically part of
     the "ship Phase 1 thermostat" arc.
   - Argument for fresh branch: the audit may diverge significantly
     from Phase 1's needs and may want to ship independently. The F6
     decision (wgpu philosophy) in particular could touch sim-gpu
     and feel out-of-scope for a thermo PR branch.
   - **Surface to user, do not unilateral.**

## Track 1: The recon round (D1-D5)

These are the five open investigation questions from the audit
findings, rewritten here as concrete actions with cheap checks.

Each is read-only — no code changes, no commits.

### D1 — `STANDARDS.md` content audit [PARTIALLY ANSWERED]

The originating session's verification pass already read `docs/STANDARDS.md`
(roughly the first 480 of 583 lines, covering all seven criteria) and
surfaced the six drifts catalogued in the audit doc as F1-F6. **D1 is
not "does it exist" — it's "are there more drifts the verification
pass missed."** The next session's job:

```bash
wc -l docs/STANDARDS.md   # Should be 583 (or near)
cat docs/STANDARDS.md | tail -n +480   # Read the part not yet covered
```

Look for:

- **Additional drifts** between code and standard not yet captured in
  F1-F6. The verification pass focused on the criteria sections
  (lines 21-450 roughly); anything past that may contain rules or
  thresholds the tool also disagrees with.
- **Authoring date / last-update date** — does the standard have
  versioning? When was it last touched? Does the answer to D2 (when
  was grade.rs last touched?) tell us which document came first and
  whether one was updated without the other?
- **Internal tensions in the standard itself** — the verification
  pass already noted one (criterion 4's table-row says "Zero
  unwrap/expect" but the expanded measurement section lists 6
  patterns). Are there other places where the standard contradicts
  itself? These are tensions the rebuild needs to resolve before it
  can implement against the standard.
- **Sections that don't appear in the criteria** — the standard may
  contain process guidelines, code review requirements, performance
  expectations, etc. that the tool doesn't even attempt to enforce.
  Cataloging these helps scope the rebuild's "what should the tool
  cover" question.

Do **not** re-derive F1-F6 from scratch — they're already captured
with verbatim quotes in the audit findings doc. Confirm them by
spot-check, focus your reading on the parts the verification pass
didn't cover.

**Surface findings as part of the recon report.**

### D2 — Git history of `xtask/src/grade.rs` and `xtask/Cargo.toml`

```bash
git log --oneline -- xtask/src/grade.rs
git log --oneline -- xtask/Cargo.toml
```

Look for:

- When grade.rs was last touched (excluding the A1 fix `c3f08c5`)
- How many distinct authors have touched it
- Whether there are any commit messages that hint at known issues
  ("TODO", "fixme", "broken on...", etc.)
- Whether xtask/Cargo.toml deps have been bumped recently or are
  stale

If the file has been untouched for >6 months while sim crates were
being built (memory says cf-design Phases 1-4 are done, sim is at
Phases 1-6 done), that's strong evidence that no one has used the
grade tool on a sim crate and no one has revalidated it.

**Cross-check against D1**: if grade.rs and STANDARDS.md were last
touched at very different times, the document with the older last-touch
is likely the one that was abandoned. This affects which is more
authoritative for the F-section decisions.

### D3 — Tarpaulin version installed vs latest upstream

```bash
cargo tarpaulin --version
```

Then check crates.io for the latest version (you may use WebFetch on
`https://crates.io/crates/cargo-tarpaulin` for this — read-only).

If installed is significantly older than latest, the rebuild may want
to either upgrade or switch tools entirely.

**Sharpened by F2**: per the standard, tarpaulin shouldn't be the
developer-facing local gate on macOS at all. The version question is
now also the question of "should we be running it locally in the first
place." If the answer is "no" (per the standard), the version
becomes less critical.

### D4 — Genealogy of mesh COMPLETION.md files

```bash
git log --oneline -- mesh/mesh-types/COMPLETION.md
git log --oneline -- mesh/mesh-io/COMPLETION.md
# (etc. for each of the 9 mesh COMPLETION files listed in audit findings)
```

Compare the dates to the git history of grade.rs from D2. If the
COMPLETION.md files predate the current grade.rs, "mesh works under
this tool" is unverified. If the COMPLETION.md files were generated
under a substantially older predecessor, the entire mesh COMPLETION
set may need to be re-graded under whatever the rebuild produces.

This finding affects scope: if mesh is unverified, the rebuild's
"keep mesh working" constraint disappears and the rebuild has more
freedom.

**Sharpened by F1, F3, F4**: even if the COMPLETION files were
generated by the current grade.rs, they were generated under
thresholds that drift from the standard (90% vs 75% for coverage,
2 of 6 safety patterns, ≤7 deps invented). So the question isn't
just "was the tool the same version" — it's "were the right things
being measured, against the right thresholds." The honest answer is
probably no.

### D5 — Read `cargo-llvm-cov` documentation

```
WebFetch https://github.com/taiki-e/cargo-llvm-cov
```

Compare to tarpaulin on:

- Runtime cost (tarpaulin took ~17 minutes for sim-thermostat)
- JSON output format and how easy it is to filter per-crate
- macOS / Apple Silicon support
- Workspace handling (does it have the same workspace-wide-denominator
  problem as tarpaulin? this is the load-bearing question)
- Stability / maintenance status
- Whether it requires nightly Rust or works on stable

This is the load-bearing architectural decision for the coverage
gate. **The recon report should make a recommendation** between
"keep tarpaulin (with the A2 fix)" and "switch to cargo-llvm-cov" —
or "we need to prototype both before deciding."

**Sharpened by F2**: the standard already says tarpaulin is
Linux-only. The case for cross-platform coverage tooling is
significantly stronger than just "tarpaulin is slow." If
`cargo-llvm-cov` works well on macOS/Windows as a first-class
platform, F2 alone is a strong argument for the switch.

## Track 2: Audit of B1-B5 (suspected bugs)

After D1-D5 are done, validate or refute each suspected bug with a
targeted experiment. These are NOT read-only — they involve running
clippy/doc/etc on test crates to see what the gate actually does. Ask
permission before each.

### B1 — `grade_clippy` measurement

Test plan:

1. Pick a small crate (probably create a throwaway test crate, or
   use a fixture).
2. Add an intentional clippy warning.
3. Run `cargo clippy -p <crate> --all-targets --all-features -- -D
   warnings 2>&1 | tee /tmp/clippy_output.txt`.
4. Inspect the output: does it contain `"warning:"` substring? Does
   it contain `"error["` substring? What does `error: <lint name>`
   look like?
5. Run `grade_clippy` against the same crate and see what number it
   reports.
6. Verdict: confirmed bug / not a bug / refined understanding.

### B2 — `grade_documentation` measurement

Same shape as B1 but for `cargo doc` with `RUSTDOCFLAGS="-D warnings"`.

### B3 — `grade_safety` test boundary

Audit plan:

1. Walk each sim crate's source files for files containing
   `#[cfg(test)]` followed by more production code (free fns, impl
   blocks, etc.) outside the test mod.
2. For each such file, hand-compute the unwrap/expect count in the
   production-after-test region.
3. Compare against what `grade_safety` reports.
4. Verdict: confirmed bug / not a bug / how widespread.

`sim-core`, `sim-mjcf`, and `cf-design` are the largest crates and
most likely to have inline test mods.

### B4 — `grade_dependencies` accuracy

Compare `grade_dependencies` output for several crates (mesh-types,
sim-thermostat, sim-core, cf-design) against `cargo metadata
--format-version 1` parsed dep counts. If any disagreement, B4 is
confirmed.

**Coordinated with F4 and F5**: B4 is about the line-counting
algorithm; F4 is about the threshold being invented; F5 is about the
measurement command (`--depth 1` vs full tree). All three need to
move together when the rebuild touches `grade_dependencies`. Don't
fix B4 without resolving F4 and F5 at the same time.

### B5 — `grade_bevy_free` substring

Low priority. Defer unless time permits or unless the rebuild design
discussion needs the answer.

**Coordinated with F6**: F6 (wgpu philosophical drift) is the more
substantive issue here. B5 is a fragility note; F6 is a decision
about what the function should be checking at all.

## Track 3: F section decisions (NEW)

The F section captures six concrete drifts between the tool and
`docs/STANDARDS.md`. Unlike the B section (suspected bugs needing
verification) and the D section (open questions needing investigation),
the F items are **already verified by the originating session** —
they don't need to be re-derived, they need **decisions**.

**The next session's job for the F section**:

1. **Confirm each F item by independent reading.** Cross-check the
   audit doc's verbatim quotes against `docs/STANDARDS.md` and
   `xtask/src/grade.rs`. If any quote doesn't match the actual
   source, surface the discrepancy.
2. **Look for additional F items** the originating session missed
   (this is the D1 task).
3. **For each F item, surface the decision to the user.** The audit
   doc lists possible resolutions for each; the user picks. Do not
   unilateral on F1, F2, F4, F6 in particular — these have substantive
   tradeoffs the user knows better than you do.
4. **Some F items can be auto-resolved if the user delegates.** F3
   (safety patterns) is mostly mechanical: extend the patterns array
   from 2 to 6 elements. F5 (deps measurement command) is one flag.
   The user may want to delegate these and reserve their attention
   for F1, F2, F6.

### F1 — Coverage threshold (75% vs 90%)

- **Current gap**: tool enforces 90%, standard says 75% (target 90%).
- **Decision options**: align tool down to 75% / align standard up
  to 90% / maintain a 75% A / 90% A+ split.
- **Default if delegated**: align tool down to 75% (matches standard
  literally; "target" semantics survive as a separate line item).
- **Severity**: Medium. Changes which crates pass.

### F2 — Tarpaulin Linux-only (load-bearing)

- **Current gap**: tool runs tarpaulin on every platform; standard
  says tarpaulin is Linux-only and macOS/Windows should rely on CI.
- **Decision options**:
  1. Gate tarpaulin behind `cfg(target_os = "linux")`, show
     "coverage: skipped" elsewhere
  2. Switch to `cargo-llvm-cov` (sharpens D5)
  3. Update standard to drop the Linux-only restriction
- **Default if delegated**: option 2 (switch tools), pending D5
  confirmation that cargo-llvm-cov is actually viable on macOS.
- **Severity**: High. User-facing every macOS/Windows developer.

### F3 — Safety pattern coverage (6 vs 2)

- **Current gap**: tool checks `unwrap, expect`; standard says
  `unwrap, expect, panic!, unreachable!, todo!, unimplemented!`.
- **Decision options**: extend tool to all 6 / make todo!/unimplemented!
  a hard fail (any occurrence is F) / ignore the difference.
- **Default if delegated**: extend tool to all 6, with
  todo!/unimplemented! as hard-fail per standard's "Zero" wording.
- **Severity**: Medium-high. Mostly mechanical, but may affect crates
  the recon hasn't audited yet.

### F4 — Dependencies threshold invented

- **Current gap**: tool enforces `≤7 deps for A`; standard has no
  numerical threshold, only qualitative "Minimal and Justified."
- **Decision options**:
  1. Codify a numerical threshold in the standard
  2. Replace tool's count check with qualitative gate ("every dep in
     Cargo.toml has a justification comment")
  3. Both — count as soft warning + justification as hard requirement
- **Default if delegated**: option 3 (both), reflecting the actual
  intent of "minimal AND justified."
- **Severity**: High. Tool and standard are measuring different
  things under the same grade name.

### F5 — Dependencies measurement command (`--depth 1`)

- **Current gap**: tool uses `--depth 1` (direct deps only); standard
  uses no depth flag (full transitive tree).
- **Decision options**: align tool with standard / update standard.
- **Default if delegated**: align tool with standard (drop `--depth 1`).
  Should be paired with F4's resolution since the count semantic and
  the threshold should move together.
- **Severity**: Medium. Silent drift.

### F6 — Bevy-free measurement drops `wgpu` (LOAD-BEARING PHILOSOPHICAL)

- **Current gap**: tool checks `bevy, winit`; standard says `bevy,
  wgpu, winit`. Tool author intentionally diverged with a docstring
  justification at `grade.rs:583-587`.
- **Decision options**:
  1. Standard is authoritative — wgpu disallowed in Layer 0,
     sim-gpu becomes structurally non-compliant
  2. Tool is authoritative — update standard to allow wgpu in
     Layer 0 as compute-shader infrastructure
  3. Refined rule — distinguish "wgpu as compute" (allowed) from
     "wgpu as part of windowing/rendering stack" (not allowed)
- **Default if delegated**: **DO NOT auto-resolve.** This is a
  philosophical decision about what "Layer 0" means. The user must
  pick.
- **Severity**: High. Affects every GPU-touching Layer 0 crate
  (sim-gpu primarily; possibly future ones).

## Surfacing the recon report

After D1-D5, the B1-B5 audits, and the F section decisions are all
complete (or have been escalated to the user as needed), write a
recon report as a new file in this directory:

- `docs/grade_tool_audit/recon_report_<date>.md`

The report should contain:

1. **D1-D5 findings** (one section each, with verbatim outputs /
   quotes; D1 should also include any new drifts not yet in F1-F6)
2. **B1-B5 verdicts** (confirmed / not / refined)
3. **F1-F6 decisions** (the user's call on each, or "still
   undecided" with the reason)
4. **Updated punch list** — items that have moved categories (e.g.,
   a confirmed B becomes an A; a refuted B becomes a "not a bug")
   plus any new items the recon surfaced
5. **Recommendation: scope A/B/C/D** with reasoning
6. **Open questions for the user** — anything the recon couldn't
   resolve and needs human judgment

This recon report is the input to the next decision point: pick a
scope, then start a chassis design session.

## Scope options (the decision the recon enables)

| Option | Includes | Phase 1 PR shape | Expected sessions |
|---|---|---|---|
| **A — minimal** | Apply the A2 pseudocode fix (~30 LOC). Keep tarpaulin. File B/C/D/E/F as a follow-on issue. Re-grade sim-thermostat, ship Phase 1. | Phase 1 PR description: "Two pre-existing tooling gaps surfaced and patched (A1 + A2). Audit of remaining gate logic and 6 documented standard-vs-tool drifts deferred." | 1 (this audit session) + 1 (apply A2 + re-grade + Phase 1 PR) = 2 |
| **B — audit + targeted** | Apply A2 + run B1-B5 audits + fix any confirmed B-bugs in flight + apply mechanical F-section fixes (F3, F5) + file C/D/F1/F2/F4/F6 as follow-ons. | Same as A but with an audit-finding doc and a smaller follow-on list. F2 (platform) and F6 (wgpu) explicitly named as deferred decisions. | 1 (recon) + 1 (audit + fixes) + 1 (Phase 1 PR) = 3 |
| **C — rebuild** | Treat the grade tool as a substantive paper artifact. Run D1-D5 + audit B1-B5 + resolve F1-F6 (each one is a chassis-level decision), write a chassis design (the 7 criteria, the architecture, the tool choices, the standard alignment), grade it against the rubric, write a rebuild plan, grade that, execute, then come back to Phase 1. Mesh COMPLETION files may need to be regraded under the rebuild's standards. May need to update STANDARDS.md alongside the rebuild to resolve F-section drifts coherently. | Phase 1 PR is gated on the rebuild. Probably 5-7 sessions before Phase 1 ships. | 1 (recon) + 1 (chassis) + 1 (rebuild plan) + 2-3 (execute) + 1 (Phase 1 PR) = 6-7 |
| **D — split-and-defer** | Apply A2 + ship Phase 1 NOW under "best-effort grade." Treat the tool rebuild as its own initiative entirely, separate from any Phase 1 acceptance criterion. Amend Phase 1 spec §12.4 #5 wording from "grade A" to "grade A on coverage/clippy/safety/deps/bevy-free pending audit of remaining gate logic and standard alignment." | Most honest about the situation; ships Phase 1 fastest after the recon round. The grade tool initiative continues independently. | 1 (recon) + 1 (A2 + spec amendment + Phase 1 PR) + N (independent grade tool work) |

**The recon report's load-bearing recommendation is which of these
four to take.** Each has real tradeoffs and the recon evidence will
inform which is right.

**Note on option scoping**: The F section additions to options A and
B are real. Option A is now "fix one bug, file 6 standards drifts as
follow-on" — that's a more substantial follow-on than the original
draft suggested. Option C now has more chassis-level decisions to
resolve before any code is written. Option D is largely unchanged
because it always treated the rebuild as a separate initiative.

## Asking-permission cadence

- Ask before each `git log` or `cat docs/STANDARDS.md` (these are
  read-only and quick — likely just batch them in a single permission
  ask)
- Ask before WebFetching cargo-llvm-cov docs
- Ask before any B1-B5 experiment (these involve creating test
  fixtures, may modify state)
- Ask before writing the recon report
- **Surface every D1-D5 finding before moving to the next** — don't
  silently aggregate
- **Surface every B1-B5 verdict before moving to the next**
- **Surface F1-F6 decisions to the user one at a time, or as a
  single batch** — your call which is more efficient given how the
  user is engaging in the session, but do NOT unilateral on F1, F2,
  F4, or F6
- **Do NOT touch xtask/src/grade.rs in this session.** The audit
  conclusion may overturn whatever you'd patch. Recon-only.
- **Do NOT touch sim-thermostat in this session.** Phase 1 is parked.
- **Do NOT touch the Phase 1 spec in this session.** The §12.4 #5
  amendment (if needed) is its own decision after the recon.
- **Do NOT touch docs/STANDARDS.md in this session.** Any changes to
  the standard are part of the rebuild's chassis design, not the
  recon round.

## What NOT to do

- Do NOT start writing the chassis design or the rebuild plan in
  this session. That's a future session whose scope depends on the
  recon outcome.
- Do NOT patch any of the gate functions in flight. The whole point
  of the audit is to make architectural decisions deliberately, not
  reactively.
- Do NOT bundle other work onto this session. If you find an unrelated
  issue, file it for a future session.
- Do NOT skip D1-D5 because they "look easy." Each finding may shift
  the recommendation; cumulative they may shift it dramatically.
- Do NOT trust the existing mesh COMPLETION.md files as proof that
  the tool works for mesh. D4 is the test of that claim, and F1/F3/F4
  cast doubt on the underlying thresholds even if the tool version
  was the same.
- Do NOT recommend a scope without the evidence to back it. If the
  recon is incomplete, recommend "more recon needed" rather than
  forcing an answer.
- Do NOT make F6 (wgpu philosophical) a unilateral call. It's the
  most substantive philosophical decision in the inventory.

## What success looks like

- D1-D5 findings captured with verbatim evidence (quotes from
  STANDARDS.md, git log output, version strings, etc.)
- B1-B5 verdicts with concrete test inputs and observed behaviors
- F1-F6 decisions captured (each one is either user-decided or
  explicitly marked "still pending")
- Any additional drifts found by the more thorough D1 read added to
  the F section (or to a new "F-extended" section in the recon
  report)
- A recon report file in this directory naming the recommendation
  (one of A/B/C/D, or "more recon needed")
- The auto-memory `project_grade_tool_audit.md` updated to reflect
  recon-round-complete state with the recommendation captured
- Phase 1 PR is still parked at `c3f08c5`-or-later on
  `feature/thermo-doc-review`, unchanged
- Total session time: 1-2 hours, almost all read-only investigation

## Closing

This is the *first* session of the grade tool audit initiative. Its
scope is recon, not rebuild. The whole point of the audit is to
deserve confidence in whatever rebuild plan emerges. The recon report
is the foundation that the chassis design and rebuild plan stand on.

The user's principle for this work is "do it right so we don't have
to go back and do it twice. patience." That principle has the
strongest claim on this session because the recon is the cheapest
session in the whole arc, and a good recon makes every subsequent
session 2-3× more productive than a rushed one.

Take the time. Don't shortcut. Surface findings as you go.
