# Grade Tool Audit — Recon Report 2026-04-10

> **Recon round complete.** This report captures all findings from the
> D1-D5 read-only investigations, B1-B5 suspected-bug verifications,
> and F1-F6 code-vs-standard drift decisions. The load-bearing output
> is the scope recommendation at the bottom: **C (full rebuild).**

## Metadata

| Field | Value |
|---|---|
| Session date | 2026-04-10 |
| Branch | `feature/thermo-doc-review` (continued from prior session) |
| HEAD at session start | `09fc814` |
| Parent initiative | Thermo-computing Phase 1 PR prep |
| Audit findings doc | `audit_findings_2026-04-09.md` (canonical, unchanged) |
| Recon plan | `recon_plan.md` (canonical, unchanged) |

---

## Track 1: D1-D5 — Read-Only Investigations

### D1 — STANDARDS.md content audit

**Status**: Complete. All 583 lines read end-to-end.

**F1-F6 spot-checks**: All 9 verbatim citations in the audit findings doc
confirmed accurate by independent reading of both `docs/STANDARDS.md` and
`xtask/src/grade.rs`.

**New drifts surfaced (F-extended)**:

| ID | Description | Severity |
|---|---|---|
| **F-ext-1** | Criterion 1 has 4 qualitative requirements (public fn tests, error paths, edge cases, integration tests) the tool doesn't measure. Coverage % alone ≠ criterion 1. | Medium |
| **F-ext-2** | Criterion 2 has 3 qualitative requirements (Examples sections, module-level docs, item coverage) the tool doesn't check. The `missing_docs` lint only fires if the crate enables it — the tool doesn't verify this. | Medium |
| **F-ext-3** | Criterion 3's standard says "No `#[allow(clippy::...)]` without justification comment." Tool doesn't audit allow annotations. | Low |
| **F-ext-4** | Criterion 4 doesn't check `unsafe` at all. Standard treats `unsafe` as part of Safety (lines 285-300): zero unjustified, each has `// SAFETY:` comment, maintainer review, prefer safe alternatives. All four checks unimplemented. | Medium-high |
| **F-ext-5** | Criterion 4's exception list (unwrap in tests, expect in build.rs, panic with justification) is only partially implemented via the buggy `#[cfg(test)]` heuristic (B3). | Medium |
| **F-ext-6** | Criterion 5 has 4 qualitative checks (justification in Cargo.toml, no duplicates, heavy deps feature-gated, reasonable version constraints) + a 5-question review checklist. All unimplemented. Extends F4. | High |
| **F-ext-7** | Criterion 6 has 3 checks unimplemented: "No bevy types in public API," "No bevy traits implemented directly," and **"Can be compiled for `--target wasm32-unknown-unknown` without bevy"** (the standard's actual portability test — not attempted by the tool). | High |
| **F-ext-8** | `cargo xtask complete` generates COMPLETION.md files that claim "Reviewed by automated" for criterion 7, which the standard says requires manual review (line 402). The "complete" subcommand documents automated review for a manual-only criterion. | Medium |
| **F-ext-9** | Tool implements numerical B/C bands for criteria 2, 3, and 4 (e.g., "1-5 warnings = B") that the standard doesn't sanction. Standard treats criteria 2 and 3 as binary (zero warnings or not A). | Low |

**New internal tensions in the standard**:

| # | Tension | Impact |
|---|---|---|
| T1 | (Already known) Criterion 4 table-row says "Zero unwrap/expect"; expanded section lists 6 patterns. | Chassis must choose canonical representation. |
| T2 | Standard's example output (line 528) shows `≥75%` as threshold; tool actually displays `≥90%`. The standard's own example contradicts the tool's behavior. | Strongest evidence that the tool drifted from the standard. |
| T3 | Standard's example output shows `minimal` for deps threshold; tool displays `≤7`. The example shows qualitative; the tool implements quantitative. | Extends F4 into the standard's self-documentation. |
| T4 | Standard's example output shows `0` for safety threshold; tool displays `≤5`. Three of seven rows in the standard's own example output contradict the tool's actual behavior. | The tool has diverged from the document that literally shows what it should look like. |

**New open questions**:

- Does CI actually enforce grades on PRs (standard claims it at lines 555-559)? Or is the grade tool local-only with aspirational CI enforcement? Not verified during this recon.
- Does `cargo xtask complete` have an interactive path for criterion 7, or does it auto-A everything? The mesh COMPLETION.md files show "automated" for the manual-review criterion.

### D2 — Git history of grade.rs and STANDARDS.md

**Status**: Complete.

**grade.rs**: 6 commits total. Born in initial commit `07f07db` (2026-01-18).
Only substantive edit: A1 fix `c3f08c5` (2026-04-09, this initiative). Two
intervening commits (`0f193bb`, `4971461`) are a sim cleanup and a docs
restructure — neither changes gate logic. **Functionally untouched for ~3
months while sim Phases 1-6 were built under it.**

**STANDARDS.md**: 1 commit (`4971461`, 2026-03-19 docs restructure). Born
in the initial commit. **Zero substantive edits since creation.** Both
documents were authored together by a single source and abandoned in
parallel.

**No TODO/fixme/WIP breadcrumbs** in the grade.rs commit history. The
drift is silent, not known-and-deferred.

**Cross-check**: STANDARDS.md and grade.rs were last substantively touched
at the same time (initial commit). Neither is "more authoritative" based
on recency. Both need to be reconciled as part of the chassis design.

### D3 — Tarpaulin version

**Status**: Complete.

- **Installed**: `cargo-tarpaulin 0.35.2`
- **Latest upstream**: `0.35.2` (released 2025-02-19)
- **Version is current.** The runtime/platform problem is structural, not
  a "needs upgrade" problem.
- Recent releases (0.34.0-0.35.2) contain **zero macOS/Apple Silicon
  improvements**. The standard's Linux-only guidance remains accurate.
- The "ancient tarpaulin" hypothesis is refuted; the platform mismatch
  hypothesis (F2) is strengthened.

### D4 — Mesh COMPLETION.md genealogy

**Status**: Complete. **Load-bearing finding.**

**All 9 mesh COMPLETION.md files were committed in `07f07db` (2026-01-18) —
the initial commit of the repository.** They were authored alongside
grade.rs, STANDARDS.md, and `cargo xtask complete` in the same commit,
by the same author, within minutes of each other (timestamps cluster
around 07:49-07:51Z).

**Consequences**:

1. **The mesh COMPLETION files are not independent verification of the
   grade tool.** They are self-certifications co-authored with the tool
   and standard. None of the three artifacts can vouch for the others —
   they're the same artifact at three different abstraction layers.
2. **No crate in the workspace has ever been independently verified
   against the grade tool.** The "mesh works under this tool" baseline
   is refuted. The rebuild has no positive baseline.
3. **All 9 COMPLETION files claim "Reviewer: automated" for criterion 7**
   (manual review per the standard). The `complete` subcommand appears to
   rubber-stamp criterion 7.
4. **All 9 COMPLETION files document `≥90%` for coverage** — matching the
   tool's drifted threshold (F1), not the standard's `≥75%`.
5. **The rebuild may need to re-grade all 9 mesh crates** under the new
   tool. The originals have no grandfather status because they were never
   independently verified.

### D5 — cargo-llvm-cov vs tarpaulin

**Status**: Complete. **cargo-llvm-cov is a credible default for the
rebuilt coverage gate.**

| Dimension | tarpaulin | cargo-llvm-cov |
|---|---|---|
| macOS support | Linux-only per standard; no upstream macOS improvements | **First-class.** Prebuilt binaries for x86_64 + aarch64 (Apple Silicon). Homebrew tap. |
| Windows | Linux-only per standard | First-class (dedicated README section) |
| Stable Rust | Yes | Yes (nightly for optional features only) |
| JSON output | `--out Json` | `--json` |
| Nextest | Limited | Native (`cargo llvm-cov nextest`) |
| Denominator problem | Present (A2). Fix requires post-hoc JSON filtering. | Present (probably). Fix available via `--exclude-from-report` and `--ignore-filename-regex` CLI flags. |
| Maintenance | 0.35.2 (2025-02-19), modest activity | 1,179 commits, 47 open issues, active |

**One empirical check needed**: run `cargo llvm-cov --json -p sim-thermostat`
on this workspace and verify the reported coverage matches 96.2% (not
9.1%). This is a chassis-design-phase activity, not a recon blocker.

**Recommendation**: Switch to cargo-llvm-cov. Resolves F2 (platform), C2
(runtime cost plausibly), and provides better CLI knobs for per-package
scoping than tarpaulin.

---

## Track 2: B1-B5 — Suspected Bug Verification

### B1 — `grade_clippy`: CONFIRMED. Critical.

**Method**: Source-code reading + empirical test (ran `cargo clippy -p
sim-thermostat` splitting stdout/stderr).

**Finding**: `grade_clippy` calls `.ignore_stderr().read()`, which reads
**stdout**. `cargo clippy` emits all diagnostics on **stderr**. Stdout is
literally 0 bytes. The substring matches `"warning:"` and `"error["` count
to 0 against an empty string. **The function returns A unconditionally,
regardless of how many clippy warnings exist.**

**Empirical evidence**: `cargo clippy -p sim-thermostat --all-targets
--all-features -- -D warnings` produced 0 bytes of stdout and 1 line of
stderr (`Finished \`dev\` profile`).

**Severity**: Critical. The clippy gate has been structurally non-functional
since the initial commit (`07f07db`, 2026-01-18). Every crate that has been
"graded" on clippy got a hallucinated A.

### B2 — `grade_documentation`: CONFIRMED (structural analogy). Critical.

**Method**: Source-code reading. Same `.ignore_stderr().read()` pattern
applied to `cargo doc --no-deps`, which also writes diagnostics to stderr.

**Finding**: Identical shape to B1. `cargo doc` outputs nothing on stdout.
The substring matches for `"warning:"` and `"error"` count to 0. **The
function returns A unconditionally.**

**Severity**: Critical. Same non-functional shape as B1.

### B3 — `grade_safety`: CONFIRMED (source reading). High.

**Method**: Source-code reading + targeted grep on sim-thermostat.

**Bug 1** (audit doc's hypothesis): The `entry().or_insert()` logic keeps
only the FIRST `#[cfg(test)]` line per file. The `>= test_start` skip
treats everything from that line onward as test code. Production code after
an inline `#[cfg(test)] mod tests { ... }` is misclassified.

**Bug 2** (NEW): `if content.contains("assert") { continue; }` silently
exempts any line containing the substring `assert` from the safety count.
This is an undocumented whitelist not sanctioned by the standard.

**Bug 3** (NEW, low): Comment skipping handles only `//` line comments,
not `/* */` block comments.

**Bug 4**: Only checks 2 of 6 safety patterns (= F3).

**Bug 5** (NEW, low): `.ignore_stderr()` on grep masks "no such file"
errors, making path failures silently return 0.

**Impact on sim-thermostat**: Correct result by coincidence. All source
files have test-mod-at-EOF (bug 1 doesn't trigger); zero lines have
`assert` + `unwrap` (bug 2 doesn't trigger). For sim-core (16K lines)
and sim-mjcf (9K lines), the bugs likely manifest; empirical sweep deferred
to the rebuild.

### B4 — `grade_dependencies`: NOT CONFIRMED (fragility validated). Low.

**Method**: Empirical comparison of `cargo tree --depth 1` line count
against `cargo metadata` authoritative dep counts.

| Crate | cargo tree (default features) | cargo metadata | Match |
|---|---|---|---|
| sim-thermostat | 5 | 5 | ✓ |
| sim-core | 6 | 8 | ✗ (rayon + serde are feature-gated optional deps) |
| sim-core (--all-features) | 8 | 8 | ✓ |

The line counting works for tested crates under default features. The
`--all-features` inconsistency (C4) is real but B4's fragility edge cases
haven't manifested. Filed as architecture debt / rebuild item.

### B5 — `grade_bevy_free`: DEFERRED. Very low.

No current false positives. The substring matching fragility (`"bevy"` vs
exact-name) is real but not manifesting. Superseded by F6 (philosophical
decision about wgpu) and C1 (rebuild should use cargo metadata).

---

## Track 3: F1-F6 — Code-vs-Standard Drift Decisions

All six decisions resolved by the user during this session.

| F | Decision | Details |
|---|---|---|
| **F1** | **Split: 75% = A, 90% = A+** | Two-tier. ≥75% ships (matches standard literally), ≥90% is gold standard. Adds a second display tier to the grade tool's coverage output. |
| **F2** | **Switch to cargo-llvm-cov** | Drop tarpaulin. First-class macOS/Apple Silicon support. Resolves platform mismatch. One empirical check needed during chassis design. |
| **F3** | **Delegated: extend to all 6 patterns** | `todo!`/`unimplemented!` hard-fail (any occurrence = F). `panic!`/`unreachable!` counted toward violations. |
| **F4** | **Both: count as soft warning + justification as hard gate** | Keep dep-count indicator (warn, don't fail). Actual A/F gate: every dep in Cargo.toml has a justification comment above it. |
| **F5** | **Delegated: drop `--depth 1`** | Align with standard's `cargo tree -p <crate> --edges normal` (full transitive tree) or switch to cargo metadata. Pairs with F4. |
| **F6** | **Tool is authoritative: wgpu allowed in Layer 0** | wgpu as compute-shader infrastructure is fine. Update STANDARDS.md to remove wgpu from the grep list. sim-gpu remains compliant. |

---

## Updated Punch List

### Category A — Confirmed bugs (now 4, was 2)

| ID | Status | Description |
|---|---|---|
| A1 | **FIXED** (`c3f08c5`) | `find_crate_path` hard-coded location heuristic |
| A2 | **UNFIXED** | `grade_coverage` workspace-wide denominator (partially moot: F2 switches to cargo-llvm-cov) |
| ~~B1~~ → A3 | **CONFIRMED** | `grade_clippy` reads stdout; clippy writes to stderr. Returns A unconditionally. |
| ~~B2~~ → A4 | **CONFIRMED** | `grade_documentation` reads stdout; cargo doc writes to stderr. Returns A unconditionally. |

### Category B — Suspected bugs (reclassified)

| ID | Status | Description |
|---|---|---|
| B3 | **CONFIRMED** (5 sub-bugs) | `grade_safety` has first-cfg(test)-wins boundary, blanket `assert` exclusion, block comment miss, 4/6 patterns missing, grep stderr suppressed. Correct for sim-thermostat by luck. |
| B4 | **NOT A BUG** (fragile) | `grade_dependencies` line counting works on tested crates. Fragile by construction, rebuild should use cargo metadata. |
| B5 | **DEFERRED** | `grade_bevy_free` substring matching. No current false positive. |

### Category C — Architectural debt (unchanged, now sharper)

| ID | Description | Sharpened by |
|---|---|---|
| C1 | Shell out + parse text with regex. Root cause of B1, B2, and half of B3. | B1+B2 empirically confirm the architecture doesn't work. |
| C2 | Tarpaulin runtime cost (17 min). | F2 resolves (switch to cargo-llvm-cov). |
| C3 | Magic number thresholds drifting from standard. | F1, F4 resolve directions. |
| C4 | Inconsistent `--all-targets`/`--all-features` across gates. | Unchanged. |

### Category D — Open questions (all answered)

| ID | Finding |
|---|---|
| D1 | STANDARDS.md fully read. 9 F-extended drifts, 4 internal tensions, 2 open questions surfaced. |
| D2 | grade.rs + STANDARDS.md both stale since initial commit. Single author, zero substantive edits in ~3 months. |
| D3 | Tarpaulin installed = latest (0.35.2). Platform problem is structural. |
| D4 | All 9 mesh COMPLETION.md files committed in initial commit alongside the tool. Not independent verification. No crate ever independently verified. |
| D5 | cargo-llvm-cov is credible alternative: first-class macOS, JSON output, nextest support, documented per-package filtering. |

### Category E — Spec/rubric alignment (unchanged)

Phase 1 spec §12.4's 7 criteria don't map 1:1 to the grade tool's 7
criteria. This is downstream of the rebuild: if the rebuild changes the
criteria set, the spec mapping changes naturally. If it doesn't, a spec
amendment may be needed.

### Category F — Code-vs-standard drifts (all resolved)

All six original F items have user decisions (see table above). Nine
F-extended items surfaced by D1 are recorded in the D1 section above and
will be carried into the chassis design as inputs.

---

## Remaining Open Questions for the User

1. **CI enforcement**: Does CI actually run `cargo xtask grade` and block
   PRs? If so, the rebuild needs CI integration. If not, the standard's
   CI claims (lines 555-559) should be updated. *Not verified during this
   recon; verifying requires reading `.github/workflows/`.*

2. **`cargo xtask complete` behavior**: Does it have an interactive
   criterion-7 path or does it auto-A everything? The mesh COMPLETION
   files show "automated" for the manual criterion. *Answering requires
   reading the `complete` subcommand source in grade.rs or a sibling file.*

3. **Mesh COMPLETION.md re-grading**: After the rebuild, should all 9 mesh
   crates be re-graded? The recon evidence says yes (they were never
   independently verified, and the thresholds they were "graded" against
   drifted from the standard). But this adds scope to the rebuild.

4. **STANDARDS.md reconciliation**: The chassis design needs to reconcile
   the standard's internal contradictions (T1-T4) and incorporate the F1-F6
   decisions before the tool can be rebuilt against it. Should this be a
   separate "fix the standard" session or folded into the chassis design
   session?

---

## Scope Recommendation: C (Full Rebuild)

### The evidence

1. **Four of six automated gates are structurally non-functional:**
   - Coverage (A2): workspace-wide denominator → wrong number
   - Clippy (B1/A3): reads stdout; clippy writes to stderr → always returns A
   - Documentation (B2/A4): reads stdout; cargo doc writes to stderr → always returns A
   - Safety (B3): 5 bugs, correct for sim-thermostat by luck

2. **No crate has ever been independently verified** (D4). The mesh
   COMPLETION.md files are self-certifications from the initial commit.

3. **Both STANDARDS.md and grade.rs are stale since 2026-01-18** (D2).
   Zero substantive edits in ~3 months.

4. **The standard has 4 internal contradictions** (T1-T4) and at least
   **15 documented drifts** (F1-F6 + F-ext-1 through F-ext-9).

5. **The architecture is the root cause** (C1). Shell out + ignore stderr
   + substring match stdout is why B1, B2, and half of B3 are broken. The
   rebuild must use structured output (JSON) from every cargo subcommand.

### Why not A, B, or D

| Option | Why not |
|---|---|
| **A (minimal patch)** | Fixes 1 of 4 broken gates (A2). Leaves B1, B2, B3 shipping hallucinated A grades. Phase 1 §12.4 #5 would be satisfied in letter, not in spirit. |
| **B (audit + targeted)** | Fixing A2 + B1 + B2 + B3 in-flight IS a rebuild. You'd rewrite 4 of 6 gate functions from scratch. Scope B is scope C with the chassis design skipped — exactly the recon-to-iteration anti-pattern. |
| **D (split-and-defer)** | Honest about the situation, but requires amending §12.4 #5 to weaken the acceptance criterion. Violates "A-grade or it doesn't ship. No exceptions." Available as escape hatch if C's cost is unacceptable. |

### Why C

The grade tool is meta-infrastructure that grades everything else. Getting
it wrong propagates silently. The evidence shows it's been wrong since
inception. The rebuild is a real initiative:

1. **Reconcile STANDARDS.md** — fix internal contradictions (T1-T4),
   incorporate F-decisions, reconcile example output with actual behavior.
2. **Chassis design** — define the rebuilt tool's architecture (structured
   output everywhere, cargo-llvm-cov for coverage, cargo metadata for deps
   and path lookup, proper stderr capture for diagnostics), the grade
   scale, and the measurement semantics for each criterion.
3. **Rebuild plan** — step-by-step execution, graded against the rubric.
4. **Execute** — rewrite the gate functions, verify empirically.
5. **Re-grade mesh crates** under the new tool.
6. **Phase 1 PR** — re-grade sim-thermostat, ship.

**Estimated cost**: 5-7 sessions before Phase 1 ships. Not cheap. But the
alternative is shipping under a gate that says A while measuring nothing,
and the user's stated principle is "do it right so we don't have to go back
and do it twice."

### What the chassis design session needs from this recon

- All D1-D5 findings (this report)
- All B1-B5 verdicts (this report)
- All F1-F6 decisions (this report)
- The F-extended items as additional requirements (this report)
- The STANDARDS.md tensions (T1-T4) as prerequisites to resolve
- The remaining open questions (CI enforcement, `complete` behavior, mesh re-grading, STANDARDS.md reconciliation)

---

## Next Session

**Chassis design for the grade tool rebuild.** Inputs: this recon report +
the audit findings doc + the user's F-decisions. Outputs: a chassis design
document defining the rebuilt tool's architecture, measurements, thresholds,
and standard alignment, graded against the 7-criterion rubric before
execution begins.

The chassis design session should also answer the four remaining open
questions above (CI enforcement, `complete` behavior, mesh re-grading
scope, STANDARDS.md reconciliation approach).

---

*End of recon report. The recon round is complete. The audit is alive.*
