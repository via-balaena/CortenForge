# Grade Tool Rebuild -- Chassis Design

> **Blueprint for the `cargo xtask grade` full rebuild (scope C).**
> This document defines what each gate function should measure, how,
> and against what thresholds. The rebuild execution session implements
> against this document.

## Metadata

| Field | Value |
|-------|-------|
| Date | 2026-04-10 |
| Branch | `feature/thermo-doc-review` |
| Parent | `recon_report_2026-04-10.md` (recon round output) |
| Scope decision | C (full rebuild), accepted 2026-04-10 |
| F-decisions | F1-F6, all user-decided (see Appendix A) |
| Blocking | Phase 1 PR, sim-thermostat A-grade (ss12.4 #5) |

---

## Open Questions Resolution

These four questions were left open by the recon report. Each is now
answered with evidence.

### Q1: CI Enforcement

**Answer: CI does NOT run `cargo xtask grade`. It has its own
independent quality jobs.**

`.github/workflows/quality-gate.yml` runs on every push to
`main`/`develop` and every PR. Seven parallel jobs (format, clippy,
test, docs, safety, dependencies via cargo-deny, bevy-free) feed a
final `quality-gate` job that blocks merge. CI's implementations are
functional -- the safety job uses awk (not the broken grade.rs
heuristic), the docs job uses env-level `RUSTDOCFLAGS`, clippy runs
with `-D warnings` at the command level.

**Impact on chassis design:**
- The grade tool is a **local development tool** and per-crate
  diagnostic. CI is the **enforcement layer**. They overlap in intent
  but are separate systems.
- The rebuild does NOT need to modify CI. The grade tool and CI should
  converge on definitions (same thresholds, same patterns) but
  maintain separate implementations.
- STANDARDS.md ss "Enforcement" section (lines 544-559) must be
  updated to accurately describe CI as independent quality jobs, not
  as `cargo xtask grade` invocations.
- CI has **no coverage gate**. Criterion 1 is local-only.

### Q2: `cargo xtask complete` Behavior

**Answer: It has an interactive criterion-7 path, but also a
`--skip-review` bypass that rubber-stamps everything.**

`xtask/src/complete.rs` offers two paths:
- **Default** (lines 46-91): runs `cargo xtask grade`, checks
  automated = A, prints API design checklist, prompts `[y/N]`, asks
  for reviewer name. This IS a real interactive criterion-7 flow.
- **`--skip-review`** (lines 97-100): bypasses the prompt, writes
  `"automated"` as reviewer. All 9 mesh COMPLETION.md files used
  this path.

**Additional problems found in `complete.rs`:**
1. `find_crate_path` (line 115) still uses the old hard-coded
   directory heuristic. The A1 cargo-metadata fix in grade.rs was
   never ported.
2. The COMPLETION.md template (line 161) hard-codes `>=90% line
   coverage` -- the drifted threshold, not the standard's >=75%.
3. The automated verification calls the broken grade tool, so it
   accepts hallucinated A grades from the B1/B2 non-functional gates.

**Impact on chassis design:** `complete` needs a redesign alongside
`grade`. See ss5.

### Q3: Mesh COMPLETION.md Re-Grading

**Answer: Yes. Re-grade all 9 mesh crates after the rebuild.**

Evidence:
1. All 9 created in the initial commit (`07f07db`) alongside the
   grade tool -- self-certifications, not independent verification.
2. All 9 used `--skip-review` (reviewer = `"automated"`) --
   criterion 7 was bypassed for all.
3. All 9 were "verified" against 4/6 broken gates -- automated A
   grades are hallucinated.
4. COMPLETION.md template hard-codes the drifted >=90% threshold.
5. No crate has been re-checked since 2026-01-18.

The re-grading is a step in the rebuild's verification plan (ss7.2),
not a new initiative. Cost is manageable: one `cargo xtask grade`
invocation per crate.

### Q4: STANDARDS.md Reconciliation Approach

**Answer: Specify changes in this document (ss4). Apply in the
rebuild execution session.**

Editing STANDARDS.md before the tool is rebuilt creates a period where
the standard says one thing, the tool does another, and neither is
authoritative. Safer to design the changes here, then apply them
atomically with the tool rebuild.

---

## ss1. Architecture

### 1.1 Design Principles

The rebuilt tool follows three architectural principles that resolve
the root causes identified in the recon:

1. **Structured output everywhere.** Every cargo subcommand is
   invoked with structured output (JSON where available, exit-code +
   captured stderr where not). No substring matching on unstructured
   text. This resolves C1 (the root cause of B1, B2, and half of B3).

2. **Correct stream capture.** Each gate function captures the stream
   that cargo actually writes diagnostics to. Clippy with
   `--message-format=json` writes JSON to stdout. `cargo doc` writes
   diagnostics to stderr. Coverage tools write JSON to stdout. The
   gate function's stream choice is documented in the per-criterion
   spec and tested empirically during verification.

3. **Cargo metadata as the foundation.** Crate path resolution,
   dependency enumeration, and workspace structure all come from
   `cargo metadata --format-version 1`, not from directory heuristics
   or `cargo tree` line counting. This resolves A1 (hard-coded paths),
   C4 (inconsistent flags), and the fragility identified in B4.

### 1.2 Internal API

The rebuild refactors grade.rs into an internal API that `complete`
can call directly, eliminating the shell-out pattern.

```
grade::evaluate(sh, crate_name) -> Result<GradeReport>
    Runs all 6 automated criteria. Returns structured report.

grade::display(report: &GradeReport)
    Prints the results table to stdout.

grade::run(crate_name, format) -> Result<()>
    Entry point for `cargo xtask grade`. Calls evaluate + display.
    Current behavior preserved (table output, color, advice).
```

`GradeReport` is the existing struct (already defined in grade.rs
lines 55-64), with two changes:
- Add `A+` variant to the `Grade` enum (see ss3.2).
- Store the actual measured values (coverage %, warning count,
  violation count, dep count) alongside the grade letter, so
  `complete` can write them to COMPLETION.md.

### 1.3 Crate Path Resolution

Keep the A1-fixed `find_crate_path` from grade.rs (lines 292-325):
`cargo metadata --format-version 1 --no-deps`, look up
`packages[].manifest_path` by name, strip to directory, rebase onto
workspace root. Port this to `complete.rs` to replace its broken
hard-coded heuristic.

### 1.4 Error Handling

| Condition | Behavior |
|-----------|----------|
| cargo-llvm-cov not installed | Criterion 1 reports `Grade::Manual` with result `"(llvm-cov n/a)"`. Tool continues to criteria 2-7. |
| Crate not found in metadata | `bail!` with clear message (existing behavior). |
| Cargo subcommand fails (non-zero exit, not a diagnostic failure) | Report the error text, grade criterion as F with the error as the result string. |
| JSON parsing fails | Report raw output to stderr, grade criterion as F. |
| Source directory (`src/`) missing | Criterion 4 reports `Grade::Manual` (existing behavior). |
| Cargo.toml unreadable | Criterion 5 reports F with error message. |

### 1.5 Output Display

Keep the current Unicode-box table format. Update column content to
match the rebuilt thresholds:

```
+------------------+------------------+-------+----------------------+
| Criterion        | Result           | Grade | Threshold            |
+------------------+------------------+-------+----------------------+
| 1. Test Coverage | 96.2%            |  A+   | >=75% (A), >=90% (A+)|
| 2. Documentation | 0 warnings       |   A   | 0 warnings           |
| 3. Clippy        | 0 warnings       |   A   | 0 warnings           |
| 4. Safety        | 0 violations     |   A   | 0 violations         |
| 5. Dependencies  | 5 deps, all just.|   A   | all justified        |
| 6. Bevy-free     | confirmed        |   A   | no bevy/winit        |
| 7. API Design    | (manual review)  |   ?   | checklist            |
+------------------+------------------+-------+----------------------+
| AUTOMATED        |                  |   A   |                      |
| OVERALL          |                  |   ?   | needs review         |
+------------------+------------------+-------+----------------------+
```

The `A+` display is green+bold (same as A). The threshold column now
matches the standard rather than the tool's drifted values.

### 1.6 xtask Dependencies

The rebuilt grade.rs requires these dependencies (most already
present in xtask/Cargo.toml):
- `xshell` -- command execution (existing)
- `serde_json` -- JSON parsing (existing, used by cargo-metadata)
- `owo-colors` -- terminal colors (existing)
- `anyhow` -- error handling (existing)
- `regex` -- pattern matching for safety scan (existing)
- `toml` -- Cargo.toml parsing for criterion 5 justification check
  (NEW -- add to xtask dependencies)

No new external tool installations are required by the grade tool
itself. `cargo-llvm-cov` must be installed by the user; the tool
degrades gracefully if absent (ss1.4).

---

## ss2. Per-Criterion Measurement Specs

Each criterion below is self-contained: a developer can implement
the gate function from this spec alone without consulting other
documents.

### 2.1 Criterion 1: Test Coverage

**What is measured:** Line coverage percentage for the target crate,
scoped to the crate's own source files (not workspace-wide).

**Tool:** `cargo-llvm-cov` (F2 decision: replaces tarpaulin).

**Command:**
```bash
cargo llvm-cov --json -p <crate>
```

**Output capture:** stdout (JSON). The JSON contains a `totals`
object with `lines.percent` (a float, 0-100).

**Parsing:**
1. Run the command, capture stdout.
2. Parse JSON. Navigate to `data[0].totals.lines.percent`.
3. If parsing fails or cargo-llvm-cov is not installed, report as
   `Grade::Manual` with appropriate message (ss1.4).

**Thresholds (F1 decision: two-tier split):**

| Coverage | Grade |
|----------|-------|
| >= 90%   | A+    |
| >= 75%   | A     |
| >= 60%   | B     |
| >= 40%   | C     |
| < 40%    | F     |

**Display:** `"{:.1}%"` (e.g., `"96.2%"`). Threshold column:
`">=75% (A), >=90% (A+)"`.

**A2 resolution (workspace-wide denominator):** cargo-llvm-cov's
`-p <crate>` flag scopes coverage to the named package. If the
reported coverage is still workspace-wide (verify empirically in
ss7.1), use `--ignore-filename-regex` to filter to only the crate's
source files.

**F-ext-1 disposition (qualitative coverage requirements):** The four
qualitative requirements (public fn tests, error paths, edge cases,
integration tests) are not automatable. They are documented as
manual verification items under criterion 7's API/quality review
checklist. The automated gate is the coverage percentage; the
qualitative gate is the human reviewer.

### 2.2 Criterion 2: Documentation

**What is measured:** Whether `cargo doc` produces zero
warnings/errors for the target crate under strict doc flags.

**Command:**
```bash
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps -p <crate>
```

**Output capture:** stderr + exit code. `cargo doc` emits all
diagnostics (missing docs, broken links, etc.) on stderr. With
`-D warnings` in RUSTDOCFLAGS, any warning is promoted to an error,
causing a non-zero exit code. stdout from `cargo doc` is empty (it
produces HTML files, not text output) -- this is why B2 was broken.

**Parsing:**
1. Run the command, capture stderr and exit code.
2. Count occurrences of `"warning:"` on stderr (for display).
3. Grade from exit code: 0 = A, non-zero = F.

**Thresholds:**

| Result | Grade |
|--------|-------|
| Exit 0 (zero warnings) | A |
| Exit non-zero (any warnings) | F |

No B/C bands. The standard says "Zero Warnings, Complete Coverage."
The F-ext-9 unsanctioned B/C bands (1-5 warnings = B, etc.) are
removed.

**Display:** `"{count} warnings"`. Threshold column: `"0 warnings"`.

**F-ext-2 disposition (qualitative documentation requirements):**
Three checks are partially automatable:
- **`missing_docs` lint enabled:** After the main doc check, read
  the crate's `lib.rs` and check for `#![warn(missing_docs)]` or
  `#![deny(missing_docs)]`. If absent, append a note to the result:
  `"0 warnings (missing_docs not enabled)"`. This is informational,
  not a grade factor -- the standard requires zero warnings, and if
  `missing_docs` isn't enabled, you get zero warnings trivially. The
  note flags this for the human reviewer.
- **Examples sections and module-level docs:** Not automatable.
  Documented as criterion 7 review items.

### 2.3 Criterion 3: Clippy

**What is measured:** Whether clippy produces zero warnings/errors
for the target crate.

**Command:**
```bash
cargo clippy -p <crate> --all-targets --all-features --message-format=json
```

Note: `-- -D warnings` is NOT used. The structured JSON output lets
us count warnings directly without promotion to errors.

**Output capture:** stdout (JSON lines). Each line is a JSON object.
Filter for `"reason": "compiler-message"`. Diagnostics have a
`message` object with a `level` field.

**Parsing:**
1. Run the command, capture stdout line by line.
2. Parse each line as JSON.
3. Filter: `reason == "compiler-message"` AND `message.level` in
   `["warning", "error"]` AND `message.spans` is non-empty (excludes
   summary lines like "N warnings emitted").
4. Count matching messages.
5. Grade: 0 = A, >0 = F.

**Thresholds:**

| Result | Grade |
|--------|-------|
| 0 diagnostics | A |
| >0 diagnostics | F |

No B/C bands. The standard says "Zero Warnings." F-ext-9 bands
removed.

**Display:** `"{count} warnings"`. Threshold column: `"0 warnings"`.

**F-ext-3 disposition (unjustified clippy allows):** Secondary check
after the main clippy run:
1. Grep all `.rs` files under `<crate>/src/` for
   `#[allow(clippy::` (excluding test code via the brace-depth
   tracker described in ss2.4).
2. For each occurrence, check if the preceding 1-3 lines contain a
   `//` comment (excluding doc comments `///`, `//!`).
3. Count allows without justification comments.
4. If count > 0, append to result: `"{clippy_count} warnings,
   {allow_count} unjustified allows"`.
5. Unjustified allows contribute to the F grade (any unjustified
   allow = F for criterion 3).

### 2.4 Criterion 4: Safety

**What is measured:** Presence of panic-capable patterns in library
code (non-test, non-example, non-build.rs).

This criterion is the most complex gate and the one with the most
bugs in the current implementation (B3: 5 sub-bugs). The rebuilt
implementation addresses all five.

**Scan scope:** All `.rs` files under `<crate>/src/`. Excluded:
- Lines inside `#[cfg(test)]` blocks (brace-depth tracking, see
  below)
- Comment lines (`//` line comments, `/* */` block comments)
- `build.rs` files (for `.expect(` only)

**Patterns and severity (F3 decision: all 6 patterns):**

| Pattern | Severity | Policy |
|---------|----------|--------|
| `todo!()` | **Hard fail** | Any occurrence in library code = F, regardless of count or justification |
| `unimplemented!()` | **Hard fail** | Any occurrence in library code = F, regardless of count or justification |
| `.unwrap()` | Counted | 0 in library code = A component. Standard: "Zero `unwrap()` in library code" |
| `.expect(` | Counted | 0 in library code = A component (excluded from `build.rs` scan). Standard: "Zero `expect()` in library code" |
| `panic!()` | Counted with justification | 0 unjustified = A component. A `//` comment on the preceding line or same line counts as justification. Standard: "Zero `panic!()` except for genuinely impossible states" |
| `unreachable!()` | Counted with justification | 0 unjustified = A component. A `//` comment on the preceding line or same line counts as justification. Standard: "All `unreachable!()` have comments explaining why" |

**F-ext-4 disposition (unsafe code):** Additional sub-check:
1. Grep for `unsafe` blocks/fns in library code.
2. For each `unsafe` occurrence, check if the preceding 1-3 lines
   contain a `// SAFETY:` comment (case-insensitive match on
   `SAFETY`).
3. Count `unsafe` blocks without `// SAFETY:` comments.
4. Any `unsafe` without safety comment = violation, contributes to F.

**Brace-depth-tracked test exclusion (fixes B3 bug 1):**

The current implementation keeps only the FIRST `#[cfg(test)]` line
per file and treats everything from that line to EOF as test code.
This misclassifies production code after inline test modules.

Rebuilt algorithm (per-file):
1. Scan line by line. Maintain state: `in_test: bool`,
   `test_brace_depth: usize`, `pending_test_attr: bool`,
   `in_block_comment: bool`.
2. Track block comments: set `in_block_comment = true` on `/*`,
   clear on `*/`. Lines inside block comments are excluded from all
   pattern checks.
3. When a line matches `#[cfg(test)]`: set `pending_test_attr = true`.
4. When `pending_test_attr` and a `{` is encountered: set
   `in_test = true`, `test_brace_depth = 1`, clear
   `pending_test_attr`.
5. While `in_test`: increment `test_brace_depth` for each `{`,
   decrement for each `}`. When depth reaches 0: set
   `in_test = false`.
6. Lines where `in_test = true` are excluded from the safety scan.

**Limitations acknowledged:** String literals containing `{` or `}`
can throw off brace counting. For a grade tool (not a compiler), this
is an acceptable edge case -- it would only matter if a string literal
containing braces appears at the exact boundary of a test module,
which is vanishingly rare. If encountered, the false positive is
conservative (flags production code that's actually in tests, not the
reverse).

**Blanket `assert` exclusion removed (fixes B3 bug 2):** The current
`if content.contains("assert") { continue; }` is removed. The
standard says "Zero `unwrap()` in library code" without an assert
exception. Lines like `debug_assert!(x.unwrap())` in library code ARE
violations.

**Block comment handling added (fixes B3 bug 3):** The
`in_block_comment` flag (step 2 above) handles `/* */` comments.

**All 6 patterns checked (fixes B3 bug 4 / F3):** See pattern table
above.

**Stderr not suppressed (fixes B3 bug 5):** The safety gate does not
shell out to grep. It reads source files directly using `std::fs`.
No stderr suppression issue.

**Thresholds:**

| Result | Grade |
|--------|-------|
| Any `todo!`/`unimplemented!` | F (hard fail) |
| 0 counted violations + 0 unjustified unsafe | A |
| Any counted violations or unjustified unsafe | F |

No B/C bands. The standard's requirements are absolute. F-ext-9
bands removed.

**Display:** `"{count} violations"` or `"F: found todo!/unimplemented!"`.
Threshold column: `"0 violations"`.

**F-ext-5 disposition (exception list):** Fully addressed by the
brace-depth tracker (tests excluded), `build.rs` exclusion (for
expect), and the justification check (for panic/unreachable). The
blanket assert exclusion is removed as an unsanctioned exception.

### 2.5 Criterion 5: Dependencies

**What is measured:** Whether every dependency in `Cargo.toml` has a
justification comment, plus informational dependency count.

**F4 decision:** Dep count is a soft warning (informational, does not
affect grade). Justification comment is the hard gate.

**F5 decision:** Drop `--depth 1`. Use cargo metadata for dependency
enumeration.

**Step 1 -- Dependency count (informational):**
1. Run `cargo metadata --format-version 1 --no-deps`.
2. Find the target package in `packages[]`.
3. Count entries in `dependencies[]` where `kind` is `null` (normal
   deps) or `"dev"` (dev deps). Exclude `"build"` deps from the
   count (they're build-time only).
4. Display the count. If count > 10, append `" (heavy)"` as a
   visual flag. This does NOT affect the grade.

**Step 2 -- Justification check (hard gate):**
1. Read the crate's `Cargo.toml` as text.
2. Identify all dependency entries under `[dependencies]`,
   `[dev-dependencies]`, and `[build-dependencies]` sections.
   A dependency entry is a line matching `<name> = ...` within a
   dependency section.
3. For each dependency entry line, check the preceding 1-3 lines for
   a `#` comment. The comment must be on a non-blank line between the
   previous entry (or section header) and this entry. Inline comments
   (`name = "1.0" # justification`) also count.
4. Count dependencies without justification comments.
5. Grade: 0 unjustified = A, >0 unjustified = F.

**Thresholds:**

| Result | Grade |
|--------|-------|
| All deps justified | A |
| Any dep unjustified | F |

**Display:** `"{count} deps, all just."` or `"{count} deps, {n}
unjustified"`. Threshold column: `"all justified"`.

**F-ext-6 disposition (qualitative dependency checks):**
- **No duplicates:** Not automatable reliably (requires semantic
  understanding of what "duplicate functionality" means). Documented
  as a criterion 7 review item.
- **Heavy deps feature-gated:** Partially automatable via cargo
  metadata (check if deps are optional). Deferred to a future
  enhancement -- the justification gate is the primary value.
- **Reasonable version constraints:** Partially automatable (flag
  `"*"` versions). Deferred to a future enhancement.
- **5-question review checklist:** Documented as criterion 7 review
  items.

### 2.6 Criterion 6: Bevy-Free (Layer 0)

**What is measured:** Whether the crate's dependency tree contains
bevy or winit packages.

**F6 decision:** wgpu is allowed in Layer 0. Remove from check.

**Command:**
```bash
cargo tree -p <crate> --prefix none --format "{p}"
```

This outputs one package name per line (no tree formatting), making
parsing trivial.

**Parsing:**
1. Run the command, capture stdout.
2. Check each line for package names starting with `bevy` or matching
   `winit`. Use exact package-name prefix matching, not substring
   matching (avoids false positives on crate names that happen to
   contain "bevy" as a substring).
3. Collect any matching package names as violations.
4. Grade: 0 violations = A, >0 = F.

**Thresholds:**

| Result | Grade |
|--------|-------|
| No bevy/winit in tree | A |
| bevy or winit found | F |

**Display:** `"confirmed"` or `"has: {violations}"`. Threshold
column: `"no bevy/winit"`.

**F-ext-7 disposition (qualitative Layer 0 checks):**
- **No bevy types in public API, no bevy traits implemented:** Not
  automatable without AST analysis. Documented as criterion 7 review
  items.
- **WASM compatibility:** Automatable but requires
  `wasm32-unknown-unknown` target installed. Not included in the
  rebuilt grade tool. CI already checks WASM compatibility for a
  subset of crates (quality-gate.yml, lines 114-150). The grade
  tool's criterion 6 checks the dependency tree; WASM compilation
  is a CI responsibility.

### 2.7 Criterion 7: API Design

**What is measured:** Manual review against the STANDARDS.md API
design checklist.

**Automated component:** None. The grade tool reports `Grade::Manual`
with result `"(manual review)"`.

**The grade tool's role:** Print the checklist items from STANDARDS.md
as a reminder when displaying the results table. The `complete`
subcommand handles the interactive review flow (ss5).

**F-ext-8 disposition:** Addressed in the `complete` redesign (ss5).
The `--skip-review` flag is removed. Criterion 7 requires genuine
human review.

---

## ss3. Grade Scale

### 3.1 Grade Definitions

| Grade | Meaning | Ship? |
|-------|---------|-------|
| **A+** | Exceeds standard (coverage only) | Yes |
| **A** | Meets all requirements | Yes |
| **B** | Below standard, close | No -- fix required |
| **C** | Below standard, significant gap | No -- major work required |
| **F** | Does not meet standard | No -- rewrite or fundamental fix |
| **?** | Requires manual review | Pending human input |

"Only A-grade crates ship. No exceptions." A+ is strictly
informational -- it ships identically to A.

### 3.2 Per-Criterion Grade Rules

| Criterion | Has A+? | Has B/C? | Rationale |
|-----------|---------|----------|-----------|
| 1. Coverage | Yes (>= 90%) | Yes (60-74% = B, 40-59% = C) | Coverage is a continuous metric; intermediate grades have diagnostic value |
| 2. Documentation | No | No (binary A/F) | Standard: "Zero Warnings" -- any warning is a failure |
| 3. Clippy | No | No (binary A/F) | Standard: "Zero Warnings" -- any warning is a failure |
| 4. Safety | No | No (binary A/F) | Standard: "Zero" for each pattern -- any violation is a failure |
| 5. Dependencies | No | No (binary A/F) | F4: justification is a binary gate |
| 6. Bevy-free | No | No (binary A/F) | Bevy is present or absent; no middle ground |
| 7. API Design | No | No (binary A/F) | Manual review: passes or doesn't |

Coverage retains B/C because the 75% -> 90% gap is wide enough that
"you're at 68%" (B) is meaningfully different from "you're at 12%"
(F) as diagnostic guidance. All other criteria are binary because
their standards are absolute (zero-tolerance).

### 3.3 Overall Grade Computation

```
AUTOMATED = worst grade among criteria 1-6
OVERALL   = worst grade among criteria 1-7
```

Worst-grade ordering: F < C < B < A < A+. If any criterion is F,
the automated grade is F. If criterion 7 is `?` (unreviewed),
OVERALL is `?`.

The `Grade` enum adds an `APlus` variant:
```
enum Grade { APlus, A, B, C, F, Manual }
```

`APlus` sorts above `A` for display but is functionally equivalent
for ship decisions.

### 3.4 What Changes from the Current Tool

| Aspect | Current | Rebuilt |
|--------|---------|--------|
| Coverage threshold | >= 90% = A | >= 75% = A, >= 90% = A+ |
| Coverage threshold display | `>=90%` | `>=75% (A), >=90% (A+)` |
| Criteria 2-3 B/C bands | 1-5 = B, 6-20 = C | Removed (binary A/F) |
| Criterion 4 B/C bands | 1-5 = B, 6-15 = C | Removed (binary A/F) |
| Criterion 4 threshold display | `<=5` | `0 violations` |
| Criterion 5 threshold display | `<=7` | `all justified` |
| Criterion 5 grade basis | Dep count | Justification presence |
| Criterion 6 grep list | bevy, wgpu, winit | bevy, winit (wgpu removed) |

---

## ss4. STANDARDS.md Changes

Each change below specifies: the location, what currently exists,
what the new text should say, and which finding it resolves.

### 4.1 T1: Criterion 4 Table-Row vs Expanded Section

**Location:** Line 29 (criteria table, row 4).

**Current:** `| 4 | Safety | Zero unwrap/expect | grep + review |`

**New:** `| 4 | Safety | Zero safety violations | grep (6 patterns) + review |`

**Rationale:** The expanded section (lines 240-300) already lists all
6 patterns. The table row should summarize, not contradict. "Zero
safety violations" encompasses unwrap, expect, panic, unreachable,
todo, unimplemented, and unjustified unsafe.

**Resolves:** T1.

### 4.2 T2: Example Output Coverage Threshold

**Location:** Line 528 (example output table).

**Current:** `| 1. Test Coverage | 94.2%            |   A   | >=75%           |`

**New:** `| 1. Test Coverage | 94.2%            |  A+   | >=75% (A), >=90% (A+) |`

**Rationale:** F1 decision (two-tier split). 94.2% earns A+, and the
threshold column shows both tiers.

**Resolves:** T2, F1.

### 4.3 T3: Example Output Dependencies Threshold

**Location:** Line 532 (example output table).

**Current:** `| 5. Dependencies  | 1 dep (nalgebra) |   A   | minimal        |`

**New:** `| 5. Dependencies  | 1 dep, all just. |   A   | all justified   |`

**Rationale:** F4 decision (justification as hard gate). "Minimal" is
qualitative and unmeasurable; "all justified" matches the automated
gate.

**Resolves:** T3, F4.

### 4.4 T4: Example Output Safety Threshold

**Location:** Line 531 (example output table).

**Current:** `| 4. Safety        | 0 unwrap/expect  |   A   | 0              |`

**New:** `| 4. Safety        | 0 violations     |   A   | 0 violations    |`

**Rationale:** Criterion 4 checks 6 patterns + unsafe, not just
unwrap/expect. "0 violations" covers all.

**Resolves:** T4.

### 4.5 F2: Coverage Tool

**Location:** Lines 40-48 (Criterion 1 measurement section).

**Current:**
```
**Measurement (Linux only):**
cargo tarpaulin -p <crate> --out Html
cargo tarpaulin -p <crate> --fail-under 75
Note: `cargo tarpaulin` only works reliably on Linux.
```

**New:**
```
**Measurement:**
cargo llvm-cov -p <crate> --json
cargo llvm-cov -p <crate> --fail-under-lines 75
Note: Requires `cargo-llvm-cov` installed. Works on Linux,
macOS (including Apple Silicon), and Windows.
```

**Rationale:** F2 decision (switch to cargo-llvm-cov). Remove
Linux-only restriction.

**Resolves:** F2.

### 4.6 F6: Bevy-Free Grep List

**Location:** Lines 360-362 (Criterion 6 measurement section).

**Current:**
```bash
cargo tree -p <crate> | grep -E "(bevy|wgpu|winit)"
# Must return empty
```

**New:**
```bash
cargo tree -p <crate> --prefix none --format "{p}" | grep -E "^(bevy|winit)"
# Must return empty
```

And line 368: remove `wgpu` from the bullet list:

**Current:** `- [ ] cargo tree shows no bevy, wgpu, or winit`

**New:** `- [ ] cargo tree shows no bevy or winit`

**Rationale:** F6 decision (wgpu allowed in Layer 0 as compute-shader
infrastructure).

**Resolves:** F6.

### 4.7 CI Enforcement Section

**Location:** Lines 553-559 (Enforcement > CI section).

**Current:**
```
### CI: Quality Gate
Every push triggers:
- All automated criteria checked
- Coverage computed
- Dependency tree analyzed
- PR blocked if any criterion fails
```

**New:**
```
### CI: Quality Gate
Every push to `main`/`develop` and every PR triggers independent
quality jobs (`.github/workflows/quality-gate.yml`):
- Format: `cargo fmt` strict check
- Clippy: zero warnings, all targets
- Tests: full test suite, 3-OS matrix
- Documentation: zero warnings under `-D warnings`
- Safety: awk-based scan for panic-capable patterns
- Dependencies: `cargo-deny` license and source audit
- Bevy-free: dependency tree check for Layer 0 crates
- WASM: compilation check for select Layer 0 crates

PR is blocked if any job fails. Note: CI does not run
`cargo xtask grade`. The grade tool is a local per-crate
diagnostic; CI enforces workspace-wide quality jobs.
Coverage (criterion 1) is local-only -- CI has no coverage gate.
```

**Rationale:** Accurately describes the actual CI setup discovered
in Q1.

### 4.8 Criterion 1 A+ Tier

**Location:** Line 38 (Criterion 1 header) and line 52 (coverage
requirement).

**Current:** `### A Standard: >=75% Line Coverage (Target: 90%)`

**New:** `### A Standard: >=75% Line Coverage; A+ Standard: >=90%`

And add after line 52:
```
- [ ] Line coverage >=75% (A grade -- ships)
- [ ] Line coverage >=90% (A+ grade -- gold standard)
```

**Rationale:** F1 decision. Makes the two-tier split explicit.

### 4.9 Criterion 5 Measurement

**Location:** Lines 308-311 (Criterion 5 measurement section).

**Current:**
```bash
cargo tree -p <crate> --edges normal
```

**New:**
```bash
# Dep count (informational):
cargo metadata --format-version 1 --no-deps
# Justification check (hard gate):
# Every dependency in Cargo.toml must have a justification comment
```

**Rationale:** F4 + F5 decisions. The measurement is now
justification-based, not count-based.

---

## ss5. `cargo xtask complete` Redesign

### 5.1 Internal API Call

Replace the shell-out to `cargo xtask grade` (complete.rs line 28)
with a direct call to `grade::evaluate(sh, crate_name)`. This
ensures `complete` uses the rebuilt gate functions, not a separate
process that could be a different binary version.

### 5.2 Path Lookup Fix

Replace `complete.rs`'s `find_crate_path` (lines 115-135, hard-coded
directory heuristic) with a call to the shared `find_crate_path` from
grade.rs (the A1-fixed cargo-metadata version). Extract the shared
function to a common module if needed.

### 5.3 COMPLETION.md Template

Replace the current template (complete.rs lines 142-181) with one
that uses **actual measured values** from the `GradeReport`:

```markdown
| # | Criterion | Status | Notes |
|---|-----------|--------|-------|
| 1 | Test Coverage | {grade} | {actual_percent}% line coverage |
| 2 | Documentation | {grade} | {actual_count} warnings |
| 3 | Clippy | {grade} | {actual_count} warnings |
| 4 | Safety | {grade} | {actual_count} violations |
| 5 | Dependencies | {grade} | {dep_count} deps, {status} |
| 6 | Bevy-free | {grade} | {result} |
| 7 | API Design | A | Reviewed by {reviewer} |
```

The current template hard-codes `">=90% line coverage"` for criterion
1 regardless of actual coverage. The rebuilt template writes the
measured value.

### 5.4 `--skip-review` Removal

Remove the `--skip-review` flag entirely. Criterion 7 requires human
review per the standard (line 402: "This criterion requires manual
review"). A flag that bypasses it defeats the purpose.

If a `--force` escape hatch is needed for automation (e.g., CI
scripts that record completion after a separate human review), it
should:
1. Be named `--force` (not `--skip-review`)
2. Print an explicit warning: "Bypassing manual API review. Criterion
   7 will be recorded as 'automated (forced)'."
3. Record the reviewer as `"automated (forced)"` in COMPLETION.md
   to distinguish from genuine reviews.

### 5.5 Interactive Flow

Keep the existing interactive flow (checklist display, y/N prompt,
reviewer name prompt). It works correctly and matches the standard's
intent. The only change is that the automated verification step (step
1) now calls the rebuilt grade tool internally.

---

## ss6. Scope and Non-Scope

### 6.1 In Scope

| Item | Description |
|------|-------------|
| `xtask/src/grade.rs` | Full rewrite of all 6 gate functions + internal API refactor |
| `xtask/src/complete.rs` | Redesign per ss5 |
| `xtask/Cargo.toml` | Add `toml` dependency for criterion 5 |
| `docs/STANDARDS.md` | Apply changes specified in ss4 (ss4.1 through ss4.9) |
| `Grade` enum | Add `APlus` variant |
| Mesh COMPLETION.md re-grading | Run rebuilt tool on all 9 mesh crates (ss7.2) |
| sim-thermostat grading | Final A-grade verification for Phase 1 ss12.4 #5 |

### 6.2 Not in Scope

| Item | Reason |
|------|--------|
| `.github/workflows/quality-gate.yml` | CI works independently and correctly (Q1). No changes needed. |
| Any sim crate source code | The grade tool grades crates; it doesn't modify them. |
| Any mesh crate source code | Same. If mesh crates fail re-grading, that's a separate fix initiative. |
| `sim/L0/thermostat/` source or spec | Phase 1 artifacts are frozen; only the grade tool blocks the PR. |
| cf-design, cf-geometry, cf-spatial | Unrelated crates. |
| sim-bevy or any L1 crate | Unrelated layer. |
| cargo-llvm-cov installation | The tool degrades gracefully (ss1.4). Installation is the user's responsibility. |
| New criteria beyond the existing 7 | The rebuild fixes criteria 1-7; it doesn't add criteria 8+. |
| Publishing to crates.io | Unrelated. |
| `cargo xtask check` | Separate subcommand, out of scope. |
| Coverage of CI gaps | CI has no coverage gate, but adding one is a separate initiative. |

### 6.3 Deferred to Future Enhancements

| Item | Reason |
|------|--------|
| `--json` output flag for grade tool | Useful for CI integration but not required for the rebuild |
| Criterion 5 duplicate-dep detection | Requires semantic analysis; justification gate is sufficient |
| Criterion 5 feature-gate audit | Partially automatable via cargo metadata; deferred |
| Criterion 6 WASM compilation check | CI already does this; adding to grade tool is low priority |
| Parallel gate execution | Cargo lock contention makes this fragile; sequential is fine |

---

## ss7. Verification Plan

Each verification step is a concrete, runnable test with a
pass/fail outcome.

### 7.1 Per-Gate Empirical Tests

**V1 -- Coverage gate (resolves A2, F2):**
1. Install `cargo-llvm-cov` if not present.
2. Run `cargo llvm-cov --json -p sim-thermostat`.
3. Parse the JSON output for `data[0].totals.lines.percent`.
4. **Pass:** reported coverage is approximately 96% (the known value
   from the Phase 1 implementation), not 9.1% (the broken
   workspace-wide number from tarpaulin).
5. **Pass:** the tool reports `A+` (>= 90%).

**V2 -- Clippy gate (resolves B1/A3):**
1. Introduce an intentional clippy warning in a test file (e.g., add
   `let x = vec![1,2,3]; let _ = x.len() > 0;` which triggers
   `clippy::len_zero`).
2. Run the rebuilt `grade_clippy`.
3. **Pass:** the gate reports >= 1 warning and grades F.
4. Revert the intentional warning.

**V3 -- Documentation gate (resolves B2/A4):**
1. Add a `pub fn` without a doc comment to a file in a crate that
   enables `#![warn(missing_docs)]`.
2. Run the rebuilt `grade_documentation`.
3. **Pass:** the gate reports >= 1 warning and grades F.
4. Revert the intentional missing doc.

**V4 -- Safety gate (resolves B3, F3):**
1. **Hard-fail test:** Add `todo!()` to a library source file (not
   in a test module). Run the rebuilt `grade_safety`. **Pass:** gate
   reports F with "found todo!/unimplemented!". Revert.
2. **Unwrap test:** Add `.unwrap()` to a library source file. Run
   the rebuilt gate. **Pass:** gate reports >= 1 violation and
   grades F. Revert.
3. **Test-exclusion test:** Add `.unwrap()` inside a `#[cfg(test)]
   mod tests { ... }` block. Run the rebuilt gate. **Pass:** gate
   reports 0 violations (the unwrap is correctly excluded as test
   code).
4. **Post-test-module test:** Add `.unwrap()` in production code
   AFTER an inline `#[cfg(test)]` module in the same file. Run the
   rebuilt gate. **Pass:** gate reports >= 1 violation (the unwrap
   is correctly identified as production code, not misclassified as
   test code -- this is the B3 bug 1 fix).
5. **Unsafe test:** Add an `unsafe {}` block without a `// SAFETY:`
   comment. Run the rebuilt gate. **Pass:** gate reports a violation.
   Revert.

**V5 -- Dependencies gate (resolves F4, F5):**
1. Remove a justification comment from a dependency in a crate's
   `Cargo.toml`.
2. Run the rebuilt `grade_dependencies`.
3. **Pass:** gate reports >= 1 unjustified dependency and grades F.
4. Restore the comment.

**V6 -- Bevy-free gate (resolves F6):**
1. Run the rebuilt `grade_bevy_free` on `sim-gpu` (which depends on
   wgpu).
2. **Pass:** gate reports A (wgpu is no longer flagged).
3. Run on `sim-bevy` (which depends on bevy).
4. **Pass:** gate reports F (bevy is correctly flagged).

### 7.2 Mesh Crate Re-Grading

After all gate functions are rebuilt and verified (V1-V6):

1. Run `cargo xtask grade <crate>` on all 9 mesh crates:
   `mesh-types`, `mesh-io`, `mesh-repair`, `mesh-sdf`,
   `mesh-offset`, `mesh-shell`, `mesh-measure`,
   `mesh-printability`, `mesh-lattice`.
2. Record results. Expected: some crates may fail criteria that
   were previously rubber-stamped by broken gates.
3. Any failures are documented as findings but do NOT block the
   grade tool rebuild. Fixing mesh crates is a separate initiative
   (ss6.2).
4. **Pass condition:** the rebuilt tool runs to completion on all 9
   crates without crashing, and produces plausible results (no
   hallucinated A grades on criteria with known issues).

### 7.3 sim-thermostat Final Gate

The capstone verification:

1. Run `cargo xtask grade sim-thermostat` with the fully rebuilt
   tool.
2. **Pass:** criteria 1-6 all report A (or A+ for coverage).
3. Run `cargo xtask complete sim-thermostat` (interactive flow).
4. Complete the criterion 7 manual review.
5. **Pass:** COMPLETION.md is written with actual measured values
   and a real reviewer name.
6. This satisfies Phase 1 ss12.4 #5 and unblocks the Phase 1 PR.

### 7.4 Rollback Target

**Pre-rebuild rollback:** `git reset --hard 57ecaea` returns to the
end-of-recon state. The chassis design document is the only new
artifact; it has no code dependencies.

**Mid-rebuild rollback:** Each gate function is rewritten
independently. If gate N's rewrite fails, revert to the commit
before gate N's changes. The other gates are unaffected.

---

## Appendix A: F-Decision Traceability

Every F-decision maps to a specific section of this chassis design.

| F | Decision | Chassis section |
|---|----------|-----------------|
| F1 | 75% = A, 90% = A+ | ss2.1 (coverage thresholds), ss3.2 (A+ tier), ss4.2 + ss4.8 (STANDARDS.md) |
| F2 | Switch to cargo-llvm-cov | ss2.1 (coverage tool), ss4.5 (STANDARDS.md) |
| F3 | Extend to all 6 patterns, todo/unimplemented hard-fail | ss2.4 (safety patterns table), ss4.1 (STANDARDS.md) |
| F4 | Dep count soft warning + justification hard gate | ss2.5 (deps measurement), ss4.3 + ss4.9 (STANDARDS.md) |
| F5 | Drop --depth 1, use cargo metadata | ss2.5 (deps step 1), ss4.9 (STANDARDS.md) |
| F6 | wgpu allowed in Layer 0 | ss2.6 (bevy-free check), ss4.6 (STANDARDS.md) |

## Appendix B: F-Extended Item Disposition

Every F-extended item is either incorporated into a per-criterion
spec or explicitly deferred with reasoning.

| F-ext | Description | Disposition | Section |
|-------|-------------|-------------|---------|
| F-ext-1 | Criterion 1 qualitative coverage requirements | Deferred to criterion 7 manual review. Coverage % is the automated gate. | ss2.1 |
| F-ext-2 | Criterion 2 qualitative doc requirements + missing_docs check | Partially incorporated: missing_docs lint check as informational note. Qualitative checks deferred to criterion 7. | ss2.2 |
| F-ext-3 | Criterion 3 unjustified clippy allows | Incorporated: secondary check in criterion 3 gate. Unjustified allows = F. | ss2.3 |
| F-ext-4 | Criterion 4 unsafe code checks | Incorporated: unsafe-without-SAFETY-comment check in criterion 4 gate. | ss2.4 |
| F-ext-5 | Criterion 4 exception list partially implemented | Incorporated: brace-depth tracking + build.rs exclusion + justification check. Blanket assert exclusion removed. | ss2.4 |
| F-ext-6 | Criterion 5 qualitative dep checks | Partially incorporated: justification check is the hard gate. Duplicate detection, feature-gate audit, version constraints deferred to future enhancements. | ss2.5 |
| F-ext-7 | Criterion 6 qualitative Layer 0 checks | Partially deferred: WASM check handled by CI. Public API and trait checks deferred to criterion 7. | ss2.6 |
| F-ext-8 | `complete` auto-stamps criterion 7 | Incorporated: --skip-review removed in complete redesign. | ss5.4 |
| F-ext-9 | Unsanctioned B/C bands for criteria 2-4 | Removed. Criteria 2-6 are binary A/F. Only criterion 1 retains B/C for diagnostic value. | ss3.2 |

---

*End of chassis design. The rebuild execution session implements
against this document.*
