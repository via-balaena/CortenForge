# Grade Tool Rebuild -- Execution Plan

> **Step-by-step execution blueprint for the `cargo xtask grade` full
> rebuild (scope C).** This document sequences the work defined in the
> chassis design into implementable steps with explicit commit
> boundaries. The execution session implements against this document.

## Metadata

| Field | Value |
|-------|-------|
| Date | 2026-04-10 |
| Branch | `feature/thermo-doc-review` |
| Parent | `chassis_design.md` (chassis design, A-graded) |
| Rollback target | `abc4ce3` (pre-rebuild, end of chassis design) |
| Steps | 11 |
| Estimated size | ~6 S, ~4 M, ~1 L |

---

## 1. Pre-Flight Checklist

The execution session must verify all of these before writing any code.

- [ ] **Branch:** on `feature/thermo-doc-review`
- [ ] **Working tree:** `git status` is clean
- [ ] **HEAD:** at or after `abc4ce3`
- [ ] **cargo-llvm-cov installed:** `cargo llvm-cov --version` succeeds.
  Required for Step 7 verification. If not installed: `cargo install
  cargo-llvm-cov`. The gate function itself can be written without it
  (graceful degradation), but V1 verification requires it.
- [ ] **xtask builds:** `cargo build -p xtask` succeeds
- [ ] **Chassis design accessible:** `docs/grade_tool_audit/chassis_design.md`
  exists and matches this plan's references
- [ ] **toml crate already present:** `xtask/Cargo.toml` already has
  `toml = "1.1"` (line 29). Chassis design ss1.6 says "NEW" but this is
  already satisfied. No Cargo.toml dependency addition needed.

---

## 2. Execution Sequence

Each step is one commit. No step spans multiple commits. No commit
spans multiple steps. Each step names its files, commit message,
verification gate, dependencies, complexity, and rollback target.

### Step 1 -- Infrastructure: Grade enum + internal API refactor

**Complexity:** M
**Dependencies:** none (first step)
**Rollback target:** `abc4ce3`

**Files changed:** `xtask/src/grade.rs`

**Changes:**

1. **Grade enum (chassis ss3.2):** Add `APlus` variant. Update
   `as_str` (`"A+"`) and `colored` (green+bold, same as A). Update
   ordering in `overall_automated` so `APlus` sorts above `A`
   (chassis ss3.3).

2. **CriterionResult measured values:** Add a field for the raw
   measured value (e.g., coverage percentage as `Option<f64>`,
   warning count as `Option<usize>`) so `complete.rs` can write
   actual numbers to COMPLETION.md. The exact field design is left
   to the execution session -- the requirement is that each gate
   function's numeric result is recoverable from the report without
   re-parsing the display string.

3. **Internal API (chassis ss1.2):** Extract `evaluate` function:
   ```
   pub fn evaluate(sh: &Shell, crate_name: &str) -> Result<GradeReport>
   ```
   Move the gate-calling logic from `run` into `evaluate`. Create
   `display(report: &GradeReport)` for the table output. `run`
   becomes: `evaluate` + `display` + advice text.

4. **Shared crate path lookup:** Make `find_crate_path` `pub(crate)`
   so Step 9 (complete.rs) can call it directly.

**Note:** Threshold column strings (e.g., `">=90%"` → `">=75% (A),
>=90% (A+)"`) are NOT updated in this step. Each gate function sets
its own threshold string when creating a `CriterionResult`, so the
strings update naturally in Steps 2-7 as each gate is rebuilt.

**Commit message:**
```
refactor(xtask): extract evaluate API and add APlus grade variant
```

**Verification:**
- `cargo build -p xtask` compiles with zero errors
- `cargo xtask grade sim-thermostat` produces the same table as
  before (gate logic unchanged; only structural refactor)

---

### Step 2 -- Criterion 6: Bevy-free gate rebuild

**Complexity:** S
**Dependencies:** Step 1 (Grade enum exists)
**Rollback target:** Step 1 commit

**Files changed:** `xtask/src/grade.rs` (`grade_bevy_free` fn)

**Changes:** Implement per chassis ss2.6.
- Replace `cargo tree -p <crate>` (full tree) with
  `cargo tree -p <crate> --prefix none --format "{p}"` (one
  package name per line)
- Remove wgpu from the violation check (F6 decision)
- Use exact package-name prefix matching (`line.starts_with("bevy")`
  or `line == "winit"`) instead of substring `contains`
- Binary A/F (no change needed; current implementation is already
  binary)

**Commit message:**
```
fix(xtask): rebuild criterion 6 bevy-free gate -- wgpu allowed, prefix matching
```

**Verification (V6):**
1. Run rebuilt `grade_bevy_free` on `sim-gpu` (depends on wgpu).
   **Pass:** reports A.
2. Run on `sim-bevy` (depends on bevy). **Pass:** reports F.

---

### Step 3 -- Criterion 2: Documentation gate rebuild

**Complexity:** S
**Dependencies:** Step 1 (Grade enum, binary A/F)
**Rollback target:** Step 2 commit

**Files changed:** `xtask/src/grade.rs` (`grade_documentation` fn)

**Changes:** Implement per chassis ss2.2.
- Capture **stderr** and **exit code** (not stdout). `cargo doc`
  writes diagnostics to stderr; stdout is empty. This is the B2
  root cause fix.
- Grade from exit code: 0 = A, non-zero = F. Count `"warning:"`
  on stderr for display only.
- Remove B/C bands (binary A/F per ss3.2).
- Add `missing_docs` informational check: after the main doc check,
  read `<crate>/src/lib.rs` for `#![warn(missing_docs)]` or
  `#![deny(missing_docs)]`. If absent, append note to result string.
  Informational only, does not affect grade.

**Commit message:**
```
fix(xtask): rebuild criterion 2 documentation gate -- stderr capture, binary A/F
```

**Verification (V3):**
1. Add a `pub fn` without doc comment to a file in a crate that
   enables `#![warn(missing_docs)]`.
2. Run rebuilt `grade_documentation`. **Pass:** reports >= 1 warning
   and grades F.
3. Revert the intentional missing doc.

---

### Step 4 -- Criterion 3: Clippy gate rebuild

**Complexity:** M
**Dependencies:** Step 1 (Grade enum, binary A/F)
**Rollback target:** Step 3 commit

**Files changed:** `xtask/src/grade.rs` (`grade_clippy` fn)

**Changes:** Implement per chassis ss2.3.
- Add `--message-format=json` to clippy invocation. Do NOT use
  `-- -D warnings` (JSON output lets us count directly).
- Capture **stdout** (JSON lines). Parse each line as JSON.
- Filter: `reason == "compiler-message"` AND `message.level` in
  `["warning", "error"]` AND `message.spans` is non-empty (excludes
  summary lines).
- Count matching messages. 0 = A, >0 = F. Remove B/C bands.
- Add unjustified `#[allow(clippy::` check per chassis ss2.3
  (F-ext-3): grep `.rs` files under `<crate>/src/` for allows,
  check preceding 1-3 lines for justification comment, unjustified
  allows = F. **Note:** The chassis spec says to exclude test code
  "via the brace-depth tracker described in ss2.4," but that tracker
  is built in Step 5. For this step, use the simpler heuristic
  (skip lines after `#[cfg(test)]` to EOF, same as current grade.rs)
  or skip test exclusion entirely — `#[allow(clippy::` almost never
  appears in test code, so false positives are negligible.

**Commit message:**
```
fix(xtask): rebuild criterion 3 clippy gate -- JSON parsing, unjustified allows
```

**Verification (V2):**
1. Introduce intentional clippy warning (e.g.,
   `let x = vec![1,2,3]; let _ = x.len() > 0;` triggers
   `clippy::len_zero`).
2. Run rebuilt `grade_clippy`. **Pass:** reports >= 1 warning, grades F.
3. Revert the intentional warning.

---

### Step 5 -- Criterion 4: Safety gate rebuild

**Complexity:** L (most complex gate, brace-depth tracker)
**Dependencies:** Step 1 (Grade enum, binary A/F)
**Rollback target:** Step 4 commit

**Files changed:** `xtask/src/grade.rs` (`grade_safety` fn)

**Changes:** Implement per chassis ss2.4. Full rewrite of the safety
gate -- the current implementation has 5 sub-bugs (B3).

1. **Direct file reading:** Replace `grep` shell-outs with
   `std::fs::read_to_string` + line-by-line scan. Eliminates the
   stderr suppression bug (B3 bug 5).

2. **Brace-depth-tracked test exclusion (B3 bug 1):** Implement
   the per-file state machine from chassis ss2.4:
   - Track `in_test`, `test_brace_depth`, `pending_test_attr`,
     `in_block_comment`
   - `#[cfg(test)]` sets `pending_test_attr`
   - Opening `{` after pending attr starts test region
   - Balanced brace tracking; test region ends when depth returns
     to 0
   - Lines where `in_test = true` excluded from scan

3. **Block comment handling (B3 bug 3):** Track `/* */` blocks.
   Lines inside block comments excluded from all pattern checks.

4. **All 6 patterns (B3 bug 4, F3 decision):** Check all patterns
   from chassis ss2.4 table: `todo!()`, `unimplemented!()`,
   `.unwrap()`, `.expect(`, `panic!()`, `unreachable!()`.

5. **Hard-fail for todo/unimplemented:** Any occurrence in library
   code = immediate F, regardless of count.

6. **Justification check for panic/unreachable:** `//` comment on
   preceding line or same line counts as justification. Unjustified
   = violation.

7. **Unsafe-without-SAFETY check (F-ext-4):** Grep for `unsafe`
   blocks/fns, check preceding 1-3 lines for `// SAFETY:` comment
   (case-insensitive on `SAFETY`). Missing comment = violation.

8. **Blanket assert exclusion removed (B3 bug 2):** Delete the
   `if content.contains("assert") { continue; }` line. Lines like
   `debug_assert!(x.unwrap())` in library code ARE violations.

9. **build.rs exclusion:** Skip `.expect(` pattern in `build.rs`
   files (per chassis ss2.4 scan scope).

**Commit message:**
```
fix(xtask): rebuild criterion 4 safety gate -- brace-depth tracking, 6 patterns, unsafe check
```

**Verification (V4 -- 5 sub-tests):**
1. **Hard-fail test:** Add `todo!()` to library source (not in test
   module). **Pass:** gate reports F with "found todo!/unimplemented!".
   Revert.
2. **Unwrap test:** Add `.unwrap()` to library source. **Pass:**
   reports >= 1 violation, grades F. Revert.
3. **Test-exclusion test:** Add `.unwrap()` inside a `#[cfg(test)]
   mod tests { ... }` block. **Pass:** reports 0 violations (correctly
   excluded as test code).
4. **Post-test-module test:** Add `.unwrap()` in production code AFTER
   an inline `#[cfg(test)]` module in the same file. **Pass:** reports
   >= 1 violation (correctly identified as production code -- this is
   the B3 bug 1 fix). Revert.
5. **Unsafe test:** Add `unsafe {}` without `// SAFETY:` comment.
   **Pass:** reports a violation. Revert.

---

### Step 6 -- Criterion 5: Dependencies gate rebuild

**Complexity:** M
**Dependencies:** Step 1 (Grade enum, binary A/F). `toml` crate
already in Cargo.toml (pre-flight verified).
**Rollback target:** Step 5 commit

**Files changed:** `xtask/src/grade.rs` (`grade_dependencies` fn)

**Changes:** Implement per chassis ss2.5.

1. **Dep count (informational):** Run
   `cargo metadata --format-version 1 --no-deps`, find target
   package, count `dependencies[]` where kind is `null` (normal) or
   `"dev"`. Exclude `"build"` deps. Display count. If > 10, append
   `"(heavy)"`. Does not affect grade.

2. **Justification check (hard gate):** Read crate's `Cargo.toml`
   as text. Parse with the `toml` crate or line-by-line text scan.
   Identify dependency entries under `[dependencies]`,
   `[dev-dependencies]`, `[build-dependencies]`. For each entry,
   check preceding 1-3 lines for a `#` comment. Inline comments
   also count. Count unjustified deps. 0 = A, >0 = F.

3. Remove old count-based B/C bands and `cargo tree --depth 1`
   approach entirely.

**Commit message:**
```
fix(xtask): rebuild criterion 5 dependencies gate -- justification check, cargo metadata
```

**Verification (V5):**
1. Remove a justification comment from a dependency in a crate's
   `Cargo.toml`.
2. Run rebuilt `grade_dependencies`. **Pass:** reports >= 1 unjustified,
   grades F.
3. Restore the comment.

---

### Step 7 -- Criterion 1: Coverage gate rebuild

**Complexity:** M
**Dependencies:** Step 1 (APlus variant). Requires cargo-llvm-cov
for verification (pre-flight).
**Rollback target:** Step 6 commit

**Files changed:** `xtask/src/grade.rs` (`grade_coverage` fn)

**Changes:** Implement per chassis ss2.1.

1. **Tool switch (F2):** Replace tarpaulin with
   `cargo llvm-cov --json -p <crate>`.
2. **JSON parsing:** Parse stdout as JSON. Navigate to
   `data[0].totals.lines.percent`.
3. **Two-tier thresholds (F1):** >= 90% = A+, >= 75% = A, >= 60% = B,
   >= 40% = C, < 40% = F. Coverage retains B/C bands for diagnostic
   value (chassis ss3.2).
4. **Graceful degradation (chassis ss1.4):** If cargo-llvm-cov is not
   installed or JSON parsing fails, report `Grade::Manual` with
   appropriate message.
5. **Display:** `"{:.1}%"`. Threshold column:
   `">=75% (A), >=90% (A+)"`.

**Commit message:**
```
fix(xtask): rebuild criterion 1 coverage gate -- cargo-llvm-cov, two-tier A/A+
```

**Verification (V1):**
1. Run `cargo llvm-cov --json -p sim-thermostat` directly to
   establish baseline coverage number.
2. Run rebuilt `grade_coverage` on `sim-thermostat`.
3. **Pass:** reported coverage is approximately 96% (known Phase 1
   value), NOT 9.1% (broken workspace-wide number). Grade is A+.

---

### Step 8 -- STANDARDS.md reconciliation

**Complexity:** S
**Dependencies:** Steps 2-7 (all gates rebuilt; standard should match
tool)
**Rollback target:** Step 7 commit

**Files changed:** `docs/STANDARDS.md`

**Changes:** Apply all 9 changes from chassis ss4:

| Change | Chassis section | What |
|--------|----------------|------|
| ss4.1 (T1) | Criterion 4 table row | "Zero safety violations" + "grep (6 patterns)" |
| ss4.2 (T2, F1) | Example output coverage | A+ grade, two-tier threshold |
| ss4.3 (T3, F4) | Example output deps | "all just." + "all justified" |
| ss4.4 (T4) | Example output safety | "0 violations" |
| ss4.5 (F2) | Criterion 1 measurement | cargo-llvm-cov, cross-platform |
| ss4.6 (F6) | Criterion 6 grep list | Remove wgpu, add `--prefix none` |
| ss4.7 | CI enforcement section | Accurate description of independent CI jobs |
| ss4.8 (F1) | Criterion 1 A+ tier | ">=75% Line Coverage; A+ Standard: >=90%" |
| ss4.9 (F4, F5) | Criterion 5 measurement | Justification-based, not count-based |

Each change is specified in the chassis design with exact location
(line numbers), current text, and new text. Apply verbatim. Line
numbers may have shifted due to prior edits in the same file --
use the content match, not the line number, as the anchor.

**Commit message:**
```
docs(standards): apply 9 reconciliation changes from grade tool chassis design
```

**Verification:**
- Diff the committed file against the chassis ss4 specs. Every change
  accounted for, no unintended changes.
- `cargo build -p xtask` still compiles (STANDARDS.md is not compiled,
  but sanity check).

---

### Step 9 -- complete.rs redesign

**Complexity:** M
**Dependencies:** Step 1 (evaluate API, shared find_crate_path),
Steps 2-7 (all gates rebuilt so evaluate returns correct results)
**Rollback target:** Step 8 commit

**Files changed:** `xtask/src/complete.rs`, `xtask/src/main.rs`

**Changes:** Implement per chassis ss5.

1. **Internal API call (ss5.1):** Replace the shell-out
   (`cmd!(sh, "cargo xtask grade {crate_name}")` at complete.rs
   line 28) with `grade::evaluate(&sh, crate_name)?`. Check
   `report.automated_grade` directly instead of parsing stdout text.

2. **Path lookup fix (ss5.2):** Replace complete.rs `find_crate_path`
   (lines 115-135, hard-coded directory heuristic) with a call to
   `grade::find_crate_path` (the A1-fixed cargo-metadata version,
   made `pub(crate)` in Step 1).

3. **COMPLETION.md template (ss5.3):** Replace the hard-coded template
   (lines 142-181) with one that writes **actual measured values**
   from `GradeReport`. Each criterion row uses the real result string
   (e.g., `"96.2% line coverage"`) instead of static text like
   `">=90% line coverage"`.

4. **--skip-review removal (ss5.4):** Remove the `--skip-review` flag
   from `Commands::Complete` in **main.rs** (line 74) and the
   `skip_review` parameter throughout complete.rs. Add `--force` flag
   with explicit warning per chassis ss5.4: prints bypass warning,
   records reviewer as `"automated (forced)"`.

5. **Interactive flow (ss5.5):** Keep the existing checklist display,
   y/N prompt, reviewer name prompt. Only change is that the automated
   verification step now calls `grade::evaluate` internally.

6. **Delete old `find_crate_path`** from complete.rs (lines 115-135)
   since it's replaced by the shared version.

**Commit message:**
```
refactor(xtask): redesign complete command -- internal API, measured values, remove --skip-review
```

**Verification:**
- `cargo build -p xtask` compiles with zero errors.
- `cargo xtask complete --help` shows `--force` flag, no
  `--skip-review`.
- `cargo xtask grade sim-thermostat` still works (regression check).

---

### Step 10 -- Mesh crate re-grading

**Complexity:** S
**Dependencies:** Steps 2-7 (all gates rebuilt)
**Rollback target:** Step 9 commit

**Files changed:** `docs/grade_tool_audit/mesh_regrading_results.md`
(new file)

**Changes:** Per chassis ss7.2.

1. Run `cargo xtask grade <crate>` on all 9 mesh crates:
   `mesh-types`, `mesh-io`, `mesh-repair`, `mesh-sdf`,
   `mesh-offset`, `mesh-shell`, `mesh-measure`,
   `mesh-printability`, `mesh-lattice`.
2. Record the full results table for each crate in the results file.
3. Note any criteria that fail under the rebuilt tool (previously
   rubber-stamped by broken gates).
4. Failures are documented as findings. Fixing mesh crates is out of
   scope (chassis ss6.2).

**Commit message:**
```
docs(grade-tool-audit): mesh crate re-grading results under rebuilt tool
```

**Verification (ss7.2 pass condition):**
- The rebuilt tool runs to completion on all 9 crates without
  crashing.
- Results are plausible (no hallucinated A grades on criteria with
  known issues).

---

### Step 11 -- sim-thermostat final gate

**Complexity:** S
**Dependencies:** Steps 2-9 (rebuilt tool + redesigned complete)
**Rollback target:** Step 10 commit

**Files changed:** `sim/L0/thermostat/COMPLETION.md` (generated by
`cargo xtask complete`)

**Changes:** Per chassis ss7.3. The capstone verification.

1. Run `cargo xtask grade sim-thermostat`.
2. **Pass:** criteria 1-6 all report A (or A+ for coverage).
3. Run `cargo xtask complete sim-thermostat` (interactive flow, no
   `--force`).
4. Complete the criterion 7 manual review against the API checklist.
5. **Pass:** COMPLETION.md is written with actual measured values and
   a real reviewer name.
6. This satisfies Phase 1 ss12.4 #5 and unblocks the Phase 1 PR.

**Commit message:**
```
feat(sim-thermostat): A-grade certification via rebuilt grade tool
```

**Verification (ss7.3 pass condition):**
- COMPLETION.md exists at `sim/L0/thermostat/COMPLETION.md`.
- Contains actual measured values (not hard-coded template text).
- Reviewer field contains a real name (not "automated").

---

## 3. STANDARDS.md Timing

STANDARDS.md changes land in **Step 8**, between "all gates rebuilt"
(Step 7) and "complete.rs redesign" (Step 9).

**Rationale:**
- **Not before gates:** Landing the standard before the tool is fixed
  creates a window where the standard says the right thing but the
  tool still returns wrong results. Someone running `cargo xtask grade`
  during the rebuild would see a correct standard contradicted by a
  broken tool -- maximally confusing.
- **Not after complete.rs:** The complete.rs redesign (Step 9) should
  operate against a consistent standard+tool pair. If STANDARDS.md
  landed after Step 9, the COMPLETION.md template might reference
  thresholds that don't match the standard.
- **The sweet spot:** After all gates pass their verification tests,
  update the standard in one commit. The divergence window is exactly
  one commit wide (Step 8 itself). From Step 9 onward, standard and
  tool are consistent.

---

## 4. Risk Points

These are the steps most likely to hit unexpected issues during
execution.

### R1: cargo-llvm-cov JSON format (Step 7)

**Risk:** The JSON schema for `cargo llvm-cov --json` may differ from
the chassis design's assumed path (`data[0].totals.lines.percent`).
The chassis references the llvm-cov JSON format, but this was based on
documentation review, not empirical output.

**Mitigation:** The pre-flight checklist requires running
`cargo llvm-cov --json -p sim-thermostat` directly and inspecting the
raw JSON before implementing the parser. If the path differs, adjust
the parser at implementation time -- the chassis design's path is the
target, the actual JSON is the authority.

### R2: Brace-depth tracker edge cases (Step 5)

**Risk:** The brace-depth state machine (chassis ss2.4) has known
limitations around string literals containing `{` or `}`. Additional
edge cases may surface: raw strings (`r#"{"#`), attribute macros that
generate code, nested `#[cfg(...)]` blocks.

**Mitigation:** The chassis acknowledges the string-literal limitation
and accepts it as conservative (false positives, not false negatives).
The V4 verification has 5 sub-tests that exercise the critical paths.
If edge cases surface during execution, fix them in the same commit --
the brace-depth tracker is self-contained within `grade_safety`.

### R3: Cargo.toml justification parsing (Step 6)

**Risk:** Cargo.toml has multiple dependency formats (`name = "version"`,
`name = { version = "...", features = [...] }`, inline tables spanning
multiple lines). The justification check (preceding 1-3 lines for a
`#` comment) may misparse multi-line dependency specifications.

**Mitigation:** Use the `toml` crate (already in Cargo.toml) to parse
the structure, but use line-by-line text scan for the comment check
(TOML parsers strip comments). The two-pass approach -- TOML parse for
dependency names, text scan for comment proximity -- is more robust
than either approach alone. Test against `xtask/Cargo.toml` itself
(which has justification comments for all deps) as the golden-path
case.

### R4: Clippy JSON line parsing (Step 4)

**Risk:** Clippy's `--message-format=json` may emit non-JSON lines
(progress indicators, build status). Parsing every line as JSON
without filtering will produce parse errors.

**Mitigation:** Wrap each line's JSON parse in a `Result` and skip
lines that fail to parse. Only process lines where JSON parsing
succeeds and `reason == "compiler-message"`. This is standard practice
for cargo's JSON output.

### R5: complete.rs internal API integration (Step 9)

**Risk:** The `evaluate` function (Step 1) may not return enough
information for complete.rs to produce the COMPLETION.md template.
The measured-value field design in Step 1 determines what Step 9 can
use.

**Mitigation:** Step 1 explicitly requires that each gate function's
numeric result is recoverable from the report. If Step 9 discovers a
missing field, the fix is a backward-compatible addition to
`CriterionResult` -- add the field in Step 9's commit, not a
retroactive change to Step 1.

---

## 5. Rollback Strategy

Every step names its rollback target. Rollback is always
`git reset --hard <target>`.

| Step | Rollback target | What is lost |
|------|----------------|--------------|
| 1 | `abc4ce3` | Infrastructure refactor |
| 2 | Step 1 commit | Bevy-free gate rebuild |
| 3 | Step 2 commit | Documentation gate rebuild |
| 4 | Step 3 commit | Clippy gate rebuild |
| 5 | Step 4 commit | Safety gate rebuild |
| 6 | Step 5 commit | Dependencies gate rebuild |
| 7 | Step 6 commit | Coverage gate rebuild |
| 8 | Step 7 commit | STANDARDS.md changes |
| 9 | Step 8 commit | complete.rs redesign |
| 10 | Step 9 commit | Mesh re-grading results |
| 11 | Step 10 commit | sim-thermostat certification |

**Full rollback:** `git reset --hard abc4ce3` returns to the
end-of-chassis-design state. No code outside `xtask/` and
`docs/STANDARDS.md` is modified by the rebuild, so rollback has
zero blast radius on sim/mesh/design crates.

**Partial rollback principle:** Gate functions are independent. If
Step N fails and rollback to Step N-1 is needed, all prior gates
remain functional. The only exception is Step 1 (infrastructure) --
rolling back Step 1 rolls back everything because all subsequent
steps depend on the API refactor.

---

## 6. Scope and Non-Scope

Inherited from chassis ss6, restated for the execution session.

### 6.1 In Scope

| Item | Steps |
|------|-------|
| `xtask/src/grade.rs` -- full rewrite of 6 gate functions + API refactor | 1-7 |
| `xtask/src/complete.rs` -- redesign per chassis ss5 | 9 |
| `xtask/src/main.rs` -- update CLI args (--skip-review -> --force) | 9 |
| `docs/STANDARDS.md` -- 9 changes per chassis ss4 | 8 |
| `Grade` enum -- add `APlus` variant | 1 |
| Mesh COMPLETION.md re-grading -- run rebuilt tool on 9 crates | 10 |
| sim-thermostat certification -- final A-grade gate | 11 |

### 6.2 Not in Scope

| Item | Reason |
|------|--------|
| `.github/workflows/quality-gate.yml` | CI works independently (chassis Q1) |
| Any sim/mesh/design crate source code | Grade tool grades crates, doesn't modify them |
| `sim/L0/thermostat/` source or spec | Phase 1 artifacts are frozen |
| `cargo xtask check` | Separate subcommand |
| New criteria beyond the existing 7 | Rebuild fixes 1-7, doesn't add 8+ |
| `--json` output flag for grade tool | Future enhancement (chassis ss6.3) |
| cargo-llvm-cov installation | User's responsibility; tool degrades gracefully |
| Fixing mesh crates that fail re-grading | Separate initiative; failures documented only |
| `xtask/Cargo.toml` dependency changes | `toml` already present; no new deps needed |

---

## 7. Definition of Done

The execution session is complete when ALL of the following are true:

1. **All 6 gate functions rebuilt:** Each implements its chassis ss2
   spec. Each has passed its verification test (V1-V6).

2. **STANDARDS.md reconciled:** All 9 changes from chassis ss4
   applied. Standard and tool are consistent.

3. **complete.rs redesigned:** Internal API call, shared path lookup,
   measured-value template, `--skip-review` removed, `--force` added.

4. **Mesh crates re-graded:** All 9 run to completion under the
   rebuilt tool. Results documented.

5. **sim-thermostat reaches A:** Criteria 1-6 all A/A+ under the
   rebuilt tool. COMPLETION.md written with actual measured values
   and a real reviewer name. Phase 1 ss12.4 #5 satisfied.

6. **11 commits on `feature/thermo-doc-review`:** Each independently
   revertable. No intermediate commit breaks `cargo build -p xtask`.

7. **Phase 1 PR unblocked:** The grade tool is trustworthy. The
   Phase 1 PR can proceed.

---

## 8. Dependency Graph (Visual)

```
Step 1 (infrastructure)
  |
  +---> Step 2 (C6 bevy-free)
  |       |
  +---> Step 3 (C2 docs)
  |       |
  +---> Step 4 (C3 clippy)
  |       |
  +---> Step 5 (C4 safety)
  |       |
  +---> Step 6 (C5 deps)
  |       |
  +---> Step 7 (C1 coverage)
          |
          v
        Step 8 (STANDARDS.md) -- waits for all gates
          |
          v
        Step 9 (complete.rs) -- waits for Step 1 + Step 8
          |
          v
        Step 10 (mesh re-grading) -- waits for all gates
          |
          v
        Step 11 (sim-thermostat) -- waits for Step 9 + Step 10
```

Steps 2-7 each depend on Step 1 but are **independent of each other**.
The linear ordering (2 -> 3 -> 4 -> 5 -> 6 -> 7) is a sequencing
choice (simplest-to-most-complex), not a dependency requirement. If
Step 5 fails, Steps 2-4 and 6-7 are unaffected.

---

*End of rebuild plan. The execution session implements against this
document and the chassis design.*
