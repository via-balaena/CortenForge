# Grade Tool Audit — Findings 2026-04-09

> **Read this cold.** This document is self-contained for a session that
> has zero conversation memory of how it got written. Every claim has a
> file path or commit hash. Every number is reproducible. Every gap is
> named with enough context to act on without back-references.
>
> **Revised once.** First draft was reviewed via fresh-eyes verification
> pass against the actual sources, which surfaced 6 new findings (the F
> section), corrected stale line numbers across all gate functions
> (mechanical drift introduced by commit `c3f08c5`), and tightened
> several claims that had been hedged unnecessarily. The revision
> shifted the inventory count from 17 to 23 items.

## Metadata

| Field | Value |
|---|---|
| Initiative | Grade tool audit (xtask grade rebuild) |
| Originating session | Phase 1 PR prep, 2026-04-09 |
| Originating branch | `feature/thermo-doc-review` (HEAD `c3f08c5` at the time of original capture; will advance after this audit doc and persistence updates commit) |
| Status | RECON pending — findings captured, no rebuild work started |
| Trigger | Phase 1 spec §12.4 acceptance criterion #5 (`cargo xtask grade sim-thermostat` must reach A across 7 criteria) — first time any sim crate had been put through the grade gate |
| Authors | Phase 1 PR prep session 2026-04-09 |

## TL;DR

The xtask grade tool was built and validated against a flat-mesh-style
workspace and never exercised against the layered sim layout (`sim/L0/<dir>`
or `sim/L1/<dir>` with `sim-` prefix on package names). Phase 1 PR prep
for `sim-thermostat` was the first time any sim crate was put through
the gate. Two distinct bugs surfaced (one fixed in flight, one diagnosed
but unfixed), an audit of the surrounding gate functions revealed a
pattern of fragility plus significant architectural debt, and a
fresh-eyes pass against the canonical `docs/STANDARDS.md` revealed that
the tool has drifted from the standard in **six concrete ways** —
including one substantive philosophical disagreement (criterion 6's
treatment of `wgpu`) and one platform mismatch (the standard says
tarpaulin is Linux-only; the tool runs it everywhere).

The total inventory is **23 items**:

- **2 confirmed bugs** (1 fixed, 1 unfixed)
- **5 suspected bugs** (named, not yet investigated)
- **4 architectural debts**
- **5 open investigation questions**
- **1 spec/rubric alignment concern**
- **6 code-vs-standard drifts** (the F section, all surfaced from
  reading `docs/STANDARDS.md` end-to-end during the verification pass)

The right scope of fix is its own initiative — recon → audit → chassis →
rebuild plan → execute → grade — not in-flight patches against a tool
we don't yet trust. **Phase 1 PR is parked pending the audit.**

## Why audit instead of patch

The discipline that built the thermo-computing line is the same
discipline that should govern the grade tool. Two latent bugs surfacing
in one session is a pattern, not a coincidence. Six documented drifts
between code and the canonical standard is an even stronger pattern —
the tool has been silently inconsistent with its own published rubric
for long enough that no one noticed. Patching the second bug (A2) in
flight would risk:

1. **Locking in an architectural decision** (e.g., keep tarpaulin) that
   the audit might overturn (e.g., move to `cargo-llvm-cov` — F2 makes
   this case sharper, since the standard already says tarpaulin is
   Linux-only).
2. **Doing the same work twice** if the rebuild restructures the
   function — sunk cost on a patch that gets thrown away.
3. **Ignoring the broader fragility pattern (B1-B5)** that the same
   architectural fix would address as a class.
4. **Ignoring the code-vs-standard drift pattern (F1-F6)**, which is
   the most damning evidence in the inventory because it's not "the
   tool might be wrong" — it's "the tool's behavior demonstrably
   diverges from the document that's supposed to define correct
   behavior."
5. **Treating substantive paper-grade infrastructure casually** —
   exactly the anti-pattern the rocket-platform principle exists to
   prevent.

The recon-to-iteration handoff principle has been validated twice on
the thermo line (Cracks 1+2+3 on first central-test run, Crack 4 on
first sweep run). It applies recursively here: the grade tool deserves
a real recon round and a chassis decision, not a piecemeal patch. The
gate-tool layer is *meta* infrastructure — it grades everything else.
Getting it wrong propagates silently into every future crate that
passes through it.

The user's operating principle for this work is: **"do it right so we
don't have to go back and do it twice. patience."** This document
exists in service of that principle.

## Session timeline (what happened 2026-04-09)

1. Phase 1 PR prep session began with the persistence layer (Phase 1
   "DONE in branch" state, six of seven §12.4 criteria already
   satisfied, only #5 grade gate open).
2. Verified git state clean at HEAD `9998119`.
3. Asked permission to run `cargo xtask grade sim-thermostat` (the
   load-bearing activity).
4. **First run hit A1**: `Could not find crate 'sim-thermostat'. Looked
   in: ["design", "mesh", "geometry", "sim"]`.
5. Diagnosed A1 as a hard-coded location heuristic in `find_crate_path`.
   Surfaced three fix options, recommended cargo metadata lookup,
   asked permission, fixed, committed as `c3f08c5`.
6. **Re-ran grade gate**. Tarpaulin took ~17 minutes (initially I
   misdiagnosed it as hung; it was actually doing instrumented work).
   Result: F at 9.1% coverage on criterion #1, A on criteria #2-#6,
   manual on #7.
7. **Diagnosed A2** by reading `target/tarpaulin/coverage.json`
   directly: the 9.1% is computed over the entire workspace (30,666
   instrumentable lines, 2,797 hit) instead of just sim-thermostat.
   Real sim-thermostat coverage is **96.2%** (126/131), and the 5
   "uncovered" lines are a multi-line `debug_assert!` panic message
   in `stack.rs:186-190` that fires only on user misuse — effective
   production-path coverage is 100%.
8. **Pass-2 audit** of the surrounding gate functions surfaced B1-B5
   (suspected fragility patterns) and C1-C4 (architectural debt).
9. **Strategic question raised**: should this be a per-symptom patch
   (option A), a targeted audit (option B), or a foundational rebuild
   (option C)? After surfacing the full inventory at the time, the
   user chose to treat the grade tool as its own initiative requiring
   its own recon → audit → chassis → rebuild cycle.
10. **First draft of this document written.**
11. **Fresh-eyes verification pass** against the actual sources caught
    stale line numbers (the A1 fix had shifted every gate function by
    ~18 lines, but the draft cited pre-fix line numbers from memory)
    AND surfaced six new findings by reading `docs/STANDARDS.md` end-to-end
    (which the first draft hadn't done, only hedged about). The
    findings became the new F section. The TL;DR inventory grew from
    17 to 23. This is the revised version.

## The full punch list

### A. Confirmed bugs

#### A1 — `find_crate_path` hard-coded location heuristic [FIXED, commit `c3f08c5`]

- **File**: `xtask/src/grade.rs` lines 292-325 (post-fix; pre-fix the
  function was at lines 286-307).
- **Symptom**: `cargo xtask grade sim-thermostat` failed with
  `Could not find crate 'sim-thermostat'. Looked in: ["design", "mesh",
  "geometry", "sim"]`.
- **Root cause**: The function used a hard-coded list of 4 directory
  names and looked for `<location>/<crate_name>`. This works for mesh
  crates (`mesh/mesh-types/`) where directory name = package name. It
  is broken for any sim crate, where:
  1. Sim crates live one level deeper: `sim/L0/thermostat/`,
     `sim/L1/bevy/`, never `sim/<name>/`.
  2. Sim crate package names have a `sim-` prefix that the directory
     name does not: package `sim-thermostat` lives in directory
     `thermostat`.
- **Fix**: Replace the directory heuristic with a `cargo metadata
  --format-version 1 --no-deps` lookup that finds the package by name
  and derives the directory from its `manifest_path`, rebased onto the
  workspace root.
- **Side benefit**: Foundational. Works for any future workspace
  layout cargo recognizes — flat, layered, or otherwise — without
  further hard-coding.
- **Cost**: ~25 LOC in the function + 1 new dep (`serde_json`).
- **Commit**: `c3f08c5` on `feature/thermo-doc-review`.
- **Files changed**:
  - `xtask/Cargo.toml` — added `serde_json = "1"`
  - `xtask/src/grade.rs` — rewrote `find_crate_path`
  - `Cargo.lock` — auto-updated for the new dep
- **Pre-commit hook results**: fmt clean, clippy clean.
- **Audit notes for the rebuild**:
  - The fix is correct and contained. The rebuild can keep it as-is or
    absorb it into a larger structural rewrite.
  - The new function uses `serde_json::Value` for ergonomic access. A
    more rigorous rebuild might depend on the typed `cargo_metadata`
    crate (which deserializes into typed structs and is the canonical
    Rust way to consume cargo metadata).

#### A2 — `grade_coverage` workspace-wide denominator [DIAGNOSED, UNFIXED]

- **File**: `xtask/src/grade.rs` lines 328-381 (function `grade_coverage`).
- **Symptom**: `cargo xtask grade sim-thermostat` returned
  `1. Coverage  9.1%  F  ≥90%` despite sim-thermostat having 35 unit
  tests + 5 mandatory integration tests + 1 sweep test (`#[ignore]`'d)
  all green and exercising the production code end-to-end.
- **Root cause**: The function runs `cargo tarpaulin -p sim-thermostat
  --out Json --output-dir target/tarpaulin` (lines 347-350) which scopes
  the *test execution* to sim-thermostat tests. But tarpaulin's
  instrumentation includes **every workspace crate** that gets built
  — sim-core (16,572 lines) and sim-mjcf (9,067 lines) are in
  sim-thermostat's dep closure. The function then parses the
  workspace-wide `(\d+\.?\d*)% coverage` line out of tarpaulin's
  stdout via regex (line 358), which produces:
  - Numerator: 2,797 lines (sim-thermostat hits + dep-closure hits)
  - Denominator: 30,666 lines (entire workspace traceable lines)
  - Ratio: 9.1%
- **Real coverage** (computed by hand from
  `target/tarpaulin/coverage.json` filtered to sim-thermostat source
  files):
  - 131 instrumentable lines in sim-thermostat
  - 126 hit
  - **96.2% — comfortably A under either the tool's `≥90%` threshold
    or the standard's `≥75%` threshold (see F1 below)**
  - The 5 "uncovered" lines are at
    `sim/L0/thermostat/src/stack.rs:186-190`, which is a multi-line
    panic message string inside a `debug_assert!` that fires only on
    user misuse. The macro itself spans lines 184-191; tarpaulin
    instruments the message-string lines (186-190) as separate trace
    entries. The macro looks like:
    ```rust
    debug_assert!(
        model.cb_passive.is_none(),
        "install_per_env: build_one returned a Model that already has \
         a cb_passive set. install_per_env will overwrite it (silently \
         dropping the prior callback's captured state, including any \
         Mutex<RNG>). Construct the Model fresh inside build_one and \
         let install_per_env be the only callback installer.",
    );
    ```
    The assertion never fires on the happy path (which is correct —
    we don't want it to fire). **Effective production-path coverage of
    sim-thermostat is 100%.**
- **Per-file breakdown for sim-thermostat**:

  | File | Lines | Hit | % |
  |---|---:|---:|---:|
  | `src/component.rs` | 2 | 2 | 100.0% |
  | `src/langevin.rs` | 26 | 26 | 100.0% |
  | `src/stack.rs` | 50 | 45 | 90.0% |
  | `src/test_utils.rs` | 53 | 53 | 100.0% |
  | `src/diagnose.rs` | 0 | 0 | n/a (no instrumentable lines) |
  | `src/lib.rs` | 0 | 0 | n/a (re-exports only) |
  | **Total** | **131** | **126** | **96.2%** |

- **Per-crate workspace breakdown** (also computed from coverage.json
  — illustrates the artifact):

  | Group | Files | Lines | Hit | % |
  |---|---:|---:|---:|---:|
  | sim/L0/core | 107 | 16,572 | 1,157 | 7.0% |
  | sim/L0/mjcf | 29 | 9,067 | 1,514 | 16.7% |
  | sim-thermostat | 6 | 131 | 126 | 96.2% |
  | examples | 194 | 0 | 0 | 0.0% |
  | sim/L0/tests | 72 | 39 | 0 | 0.0% |
  | design/cf-design | 33 | 128 | 0 | 0.0% |
  | sim/L1/bevy | 33 | 219 | 0 | 0.0% |
  | sim/L0/ml-bridge | 29 | 167 | 0 | 0.0% |
  | design/cf-geometry | 19 | 2,327 | 0 | 0.0% |
  | sim/L0/gpu | 18 | 10 | 0 | 0.0% |
  | mesh/mesh-io | 12 | 648 | 0 | 0.0% |
  | mesh/mesh-lattice | 11 | 89 | 0 | 0.0% |
  | mesh/mesh-repair | 10 | 2 | 0 | 0.0% |
  | mesh/mesh-shell | 8 | 62 | 0 | 0.0% |
  | design/cf-spatial | 7 | 167 | 0 | 0.0% |
  | mesh/mesh-printability | 7 | 2 | 0 | 0.0% |
  | mesh/mesh-measure | 6 | 4 | 0 | 0.0% |
  | sim/L0/simd | 6 | 356 | 0 | 0.0% |
  | sim/L0/urdf | 6 | 516 | 0 | 0.0% |
  | xtask | 6 | 0 | 0 | 0.0% |
  | mesh/mesh-offset | 5 | 0 | 0 | 0.0% |
  | mesh/mesh-sdf | 4 | 0 | 0 | 0.0% |
  | mesh/mesh-types | 4 | 16 | 0 | 0.0% |
  | sim/L0/types | 4 | 144 | 0 | 0.0% |
  | mesh/mesh | 1 | 0 | 0 | 0.0% |
  | **TOTAL** | **637** | **30,666** | **2,797** | **9.1%** |

  The crates with non-zero coverage are exactly sim-thermostat's dep
  closure: sim-thermostat itself, sim-core, sim-mjcf. Everything else
  gets instrumented during workspace compilation but is never executed
  because the test scope is `-p sim-thermostat`.

- **Why this matters**: This bug structurally tanks coverage for any
  crate with a non-trivial dep closure. Mesh crates passed because
  their dep closures are small (mesh-types depends on essentially
  nothing); their `<package_lines>/<workspace_lines>` ratio happened
  to clear 90% by luck. Sim crates all depend on sim-core (heavy) and
  most also depend on sim-mjcf, and will all return F under the
  current code.
- **Proposed fix** (for the audit to validate or override):
  ```
  1. Run tarpaulin (same command as today)
  2. Read target/tarpaulin/coverage.json
  3. Use find_crate_path() to get the target crate's source directory
  4. Filter the JSON traces dict to entries whose absolute path starts
     with <workspace>/<crate_dir>/src/
  5. Sum total + hit lines across that filter
  6. Compute percentage from that filtered set, not the workspace-wide
     total
  ```
  ~30-40 LOC in `grade_coverage`. No new dependencies (`serde_json` is
  already an xtask dep after A1's fix).
- **Why we did not fix this in the surfacing session**: Fixing in
  flight would lock in tarpaulin as the coverage tool. The audit may
  recommend `cargo-llvm-cov` instead (D5 below — and F2 makes the
  case for switching even stronger), in which case the targeted fix
  would be wasted. The audit is the right place to make a coherent
  architectural choice.

### B. Suspected bugs (named, not yet investigated)

These are read-from-the-source suspicions, not verified failures. The
audit's first pass should validate or refute each one with a targeted
experiment.

#### B1 — `grade_clippy` may not measure what it claims

- **File**: `xtask/src/grade.rs` lines 416-447 (function `grade_clippy`).
- **What it does**: Runs `cargo clippy -p {crate_name} --all-targets
  --all-features -- -D warnings` (line 419). The `-D warnings` flag
  converts every warning into an error. Captures stdout via `.read()`.
  Counts (lines 426-427):
  - `output.matches("warning:").count()` as warnings
  - `output.matches("error[").count()` as errors
- **Why this is structurally suspicious**:
  - In the success path (no warnings), both counts are 0 → grade A.
    Correct.
  - In the failure path (some warnings), `-D warnings` upgrades them
    to errors. Cargo emits `error: <lint_name>` (with bracketed
    `error[E....]` format only for compile errors, not lint errors).
    Lint errors emitted as `error: <lint name>` may not match the
    `error[` substring.
  - Net effect: the function might miss lint failures entirely and
    falsely report A on a crate with actual clippy issues.
- **Verification needed**: Test on a crate with intentional warnings;
  see what `output` actually contains; see whether the regex catches
  it; see whether the function returns the right grade.
- **Severity**: Potentially high. If confirmed, the gate has been
  silently passing crates that should have failed. Mesh COMPLETION.md
  files (if generated under this code) would be untrustworthy.

#### B2 — `grade_documentation` loose substring matching

- **File**: `xtask/src/grade.rs` lines 384-413 (function
  `grade_documentation`).
- **What it does**: Runs `cargo doc --no-deps -p {crate_name}` with
  `RUSTDOCFLAGS=-D warnings` (lines 385-386). Counts (lines 392-393):
  - `output.matches("warning:").count()` as warnings
  - `output.matches("error").count()` as errors (loose — not
    `error[` or `error:`)
- **Why suspicious**:
  - The bare `error` substring is too loose. It would match any
    document containing the word "error" anywhere — in doc comments,
    in error message text, in stderr passthrough. False positives are
    likely; the function may be structurally over-counting.
  - `RUSTDOCFLAGS="-D warnings"` upgrades doc warnings to errors.
    Same B1 dynamic may apply: warnings become errors in rustdoc
    output, the substring "warning:" disappears, and the function
    silently undercounts.
- **Verification needed**: Test on a crate with intentional doc
  warnings.
- **Severity**: Probably low (cargo doc rarely emits the literal word
  "error" outside of actual errors) but worth checking.

#### B3 — `grade_safety` test-boundary heuristic is incorrect for inline test mods

- **File**: `xtask/src/grade.rs` lines 450-551 (function `grade_safety`).
- **What it does**: Uses `grep -rn '#\[cfg(test)\]'` to find the
  FIRST `#[cfg(test)]` line per file (lines 463-484), then treats
  everything at or after that line as test code (skipped from the
  unwrap/expect count, lines 511-516).
- **Why this is wrong**: The "first cfg(test) line marks the rest of
  the file as test" heuristic fails for any file with an inline
  `#[cfg(test)] mod tests { ... }` followed by more production code.
  Example:
  ```rust
  // production code
  fn frobnicate() { let x = something().unwrap(); ... }

  #[cfg(test)]
  mod tests { ... }

  // more production code (a free fn or a separate impl block)
  fn baz() { let y = something_else().unwrap(); ... }
  // ↑ this is now incorrectly classified as "test" and its unwraps
  //   are missed by the safety gate.
  ```
- **Also**: The grep output parser uses `splitn(3, ':')` (line 477)
  which is fragile against file paths containing `:` (rare but
  possible).
- **Verification needed**: Audit each crate for files with inline test
  mods followed by more production code. `sim-core`, `sim-mjcf`, and
  `cf-design` are the largest crates and most likely to have this
  pattern.
- **Severity**: Medium. Plausibly affects sim crates that follow
  common Rust patterns.

#### B4 — `grade_dependencies` line counting is fragile

- **File**: `xtask/src/grade.rs` lines 554-581 (function
  `grade_dependencies`).
- **What it does**: Runs `cargo tree --depth 1` (line 555) and counts
  output lines minus 1 (line 561).
- **Why suspicious**: Doesn't handle:
  - Tree formatting characters that span multiple lines
  - `(*)` continuation marks for diamond dependencies
  - Version disambiguation lines (`pkg v1.0.0 (proc-macro)`)
  - Empty lines or trailing whitespace
- **Currently correct by accident**: Returned `5 deps` for
  sim-thermostat. The actual dep count from `cargo tree -p
  sim-thermostat --edges normal --depth 1` is 4 direct + 1 internal
  (sim-core) = 5. Right number, fragile algorithm.
- **Verification needed**: Compare against `cargo metadata
  --format-version 1` actual dep count (parsed from JSON,
  authoritative).
- **Severity**: Low. May produce miscounts on edge cases but unlikely
  to flip a grade.
- **Related**: F4 (the threshold the count is compared against is
  invented relative to the standard) and F5 (the measurement command
  itself differs from the standard).

#### B5 — `grade_bevy_free` substring matching false-positive risk

- **File**: `xtask/src/grade.rs` lines 588-626 (function
  `grade_bevy_free`).
- **What it does**: Substring-searches `"bevy"` and `"winit"` in
  `cargo tree` output (lines 597-598).
- **Why suspicious**: Substring matching, not exact-name matching.
  False-positives if any future dep happens to contain "bevy" or
  "winit" in its name.
- **Currently fine**: No such dep exists in the workspace today.
- **Verification needed**: Replace with exact-name matching or use
  cargo metadata to parse the dep tree structurally.
- **Severity**: Very low (no false positive today).
- **Related**: F6 (the function intentionally drops `wgpu` from the
  list, which is a more substantive disagreement with the standard
  than the substring fragility).

### C. Architectural debt (not specific bugs, but the foundation)

#### C1 — Tool shells out to cargo and parses text with regex

The entire grade tool is structured as: shell out to a cargo
subcommand, capture stdout, regex-match the text, derive a number,
derive a grade. Every gate function except the just-fixed
`find_crate_path` follows this pattern. It is fragile by construction:

- **Cargo's text output format is not a stable API.** Updates to
  cargo, clippy, rustdoc, tarpaulin, or cargo-tree have historically
  changed output formats.
- **Substring matching is loose** and produces false positives /
  negatives (see B1, B2, B5).
- **Regex parsing of human-formatted output is brittle** (see A2
  where the regex matched the wrong number entirely).
- **Each gate is independently fragile**, and the failure modes are
  silent: the function returns a number; whether the number is right
  depends on whether the regex matched the expected pattern.

**The architectural fix**: cargo has structured output for almost
every operation:

- `cargo metadata --format-version 1` → JSON workspace data
- `cargo build --message-format=json` → JSON build diagnostics
- `cargo clippy --message-format=json` → JSON lint diagnostics
- `cargo doc --message-format=json` → JSON doc diagnostics
- Tarpaulin / `cargo-llvm-cov` → JSON coverage reports
- `cargo tree --format-version 1` → JSON dep tree (if a flag exists,
  worth verifying; if not, parse `cargo metadata` for the dep tree
  directly)

A rebuilt grade tool would use structured outputs, parse with serde,
and compute grades from typed data. This kills B1-B5 as a class
because the failure modes change from "regex didn't match" to
"deserialization failed" — far easier to debug, far less likely to
silently undercount. The A1 fix already moved `find_crate_path` in
this direction.

#### C2 — Tarpaulin runtime cost is high; the alternative is plausibly faster; AND the standard says tarpaulin is Linux-only

- Sim-thermostat grade run took **~17 minutes**, almost all of it
  tarpaulin instrumentation + execution.
- Each re-grade is a 17-minute wall-clock commitment.
- `cargo-llvm-cov` is the modern alternative: wraps `llvm-cov`
  directly, generally faster, simpler architecture, also produces
  JSON, well-maintained, designed for source-based coverage on stable
  Rust.
- **AND** per `docs/STANDARDS.md` the standard explicitly says
  tarpaulin is Linux-only — see F2. The tool runs tarpaulin on macOS
  unconditionally, which is out-of-spec.
- Whether tarpaulin is still the right tool is an open question for
  the rebuild. See D5.

#### C3 — Grade thresholds are magic numbers AND drift from the standard

- `≥90%` coverage, `≤7` deps, `≤5` safety violations, `≤5` doc
  warnings, etc. all live as magic numbers in `grade.rs`.
- `grade.rs:3` references "the seven-criterion grading system defined
  in docs/STANDARDS.md" — these thresholds should be driven by that
  standard.
- **The drift between code thresholds and the standard is no longer a
  hypothetical risk; it has been verified.** See F1 (coverage 75% vs
  90%) and F4 (deps threshold invented out of whole cloth — the
  standard has no numerical dep threshold). The F section catalogs
  every drift surfaced this session; C3 is the abstract pattern that
  the F items are concrete instances of.

#### C4 — Inconsistent invocation across gates

- Coverage doesn't pass `--all-targets`
- Documentation doesn't pass `--all-features`
- Safety doesn't even invoke cargo (uses `grep -rn`)
- Clippy passes both `--all-targets --all-features`
- Bevy-free uses `cargo tree` without `--all-features` or `--edges`

Inconsistent invocation = inconsistent semantics across gates. A
crate that passes one gate under one set of features may fail
another gate under a different set, and the inconsistency is
invisible to the user.

### D. Open investigation questions (would inform rebuild scope)

These are read-only investigations the audit session should run
**before** committing to a rebuild architecture. **D1 is partially
answered already** by the verification pass that wrote this revision;
the next session should still verify and extend rather than starting
from scratch.

#### D1 — `STANDARDS.md` content audit [PARTIALLY ANSWERED]

- **Status**: STANDARDS.md exists (`docs/STANDARDS.md`, 583 lines).
  Verification pass surfaced six concrete drifts between code and
  standard (the F section). The next session should:
  1. Read the rest of STANDARDS.md that the verification pass didn't
     cover (criteria 7+, anything past the criteria sections, the
     overall structure of the standard).
  2. Look for additional drifts not yet captured in F1-F6.
  3. Note STANDARDS.md's authoring date / last-update date and
     whether it has versioning that the rebuild needs to respect.
- **Why it matters**: The F section is the load-bearing finding for
  the rebuild's "what should the gates measure" question. Any
  additional drifts found by a more thorough pass extend that
  picture.
- **Cheap check**: `wc -l docs/STANDARDS.md && cat docs/STANDARDS.md`
  (the full file is ~583 lines; verification pass only read the first
  ~480).

#### D2 — When was `xtask/src/grade.rs` last touched?

- **What**: `git log --oneline -- xtask/src/grade.rs` and `git log
  --oneline -- xtask/Cargo.toml`.
- **Why it matters**: If the code has been untouched for 6+ months
  while sim crates were being built, that's strong evidence of "no
  one has used this on a sim crate, no one has revalidated it." Also
  informs how stale the deps are.

#### D3 — What tarpaulin version is installed vs latest upstream?

- **What**: `cargo tarpaulin --version` + check crates.io for latest.
- **Why it matters**: An ancient tarpaulin may have known issues that
  the rebuild can sidestep by upgrading or by switching to llvm-cov.
- **Sharpened by F2**: Per the standard, tarpaulin shouldn't be the
  developer-facing local gate on macOS at all. The version question
  is now also the question of "should we be running it locally in the
  first place."

#### D4 — Were the existing `mesh/**/COMPLETION.md` files generated under the current `grade.rs` or a predecessor?

- **What**: `git log` on the COMPLETION.md files; compare to grade.rs
  commit history. Existing COMPLETION.md files (per glob 2026-04-09):
  - `mesh/mesh-io/COMPLETION.md`
  - `mesh/mesh-lattice/COMPLETION.md`
  - `mesh/mesh-measure/COMPLETION.md`
  - `mesh/mesh-offset/COMPLETION.md`
  - `mesh/mesh-printability/COMPLETION.md`
  - `mesh/mesh-repair/COMPLETION.md`
  - `mesh/mesh-sdf/COMPLETION.md`
  - `mesh/mesh-shell/COMPLETION.md`
  - `mesh/mesh-types/COMPLETION.md`
- **Why it matters**: If COMPLETION.md files predate the current
  `grade.rs`, "mesh works under this tool" is unverified. The rebuild
  may not have any positive baseline at all. The entire mesh COMPLETION
  set may need to be re-graded under whatever the rebuild produces.
- **Sharpened by F1, F3, F4**: Even if the COMPLETION files were
  generated by the current grade.rs, they were generated under
  thresholds that drift from the standard. So the question isn't just
  "was the tool the same version" — it's "were the right things being
  measured, and against the right thresholds." The honest answer is
  probably no.

#### D5 — Is `cargo-llvm-cov` a better fit than tarpaulin?

- **What**: Read its docs (https://github.com/taiki-e/cargo-llvm-cov);
  compare features, runtime, JSON output format, macOS support,
  workspace handling, package-scoped behavior.
- **Why it matters**: This is the load-bearing architectural decision
  for the coverage gate. If llvm-cov wins, the rebuild's coverage
  gate looks substantially different from any tarpaulin patch. If
  tarpaulin wins, the rebuild can use the proposed A2 fix as-is.
- **Sharpened by F2**: The standard already says tarpaulin is
  Linux-only. The case for cross-platform coverage tooling is
  significantly stronger than just "tarpaulin is slow."

### E. Spec/rubric alignment concern

This is a **separate strategic question from the tooling rebuild**
that surfaced during the same session and should not be lost.

Phase 1 spec §12.4's seven acceptance criteria do **not** map 1:1 to
the grade tool's seven criteria:

| Spec §12.4 criterion | Grade tool criterion that covers it |
|---|---|
| #1 build clean | (none — implicit in other gates running) |
| #2 clippy clean | #3 Clippy ✓ |
| #3 fmt clean | (none — separate gate) |
| #4 tests green | (none — separate gate; coverage exercises tests but doesn't gate on green) |
| #5 grade A | the grade itself (recursive) |
| #6 sim-core rand-free | (none — separate concern; bevy-free is the closest analog but tests a different thing) |
| #7 lib.rs rustdoc has the four chassis Decision 6 pieces | #2 Documentation (partially — checks for warnings, does not check for the four specific pieces) |

**The spec's #5 says "grade reaches A across 7 criteria" but the
grade tool's 7 criteria don't fully cover the spec's other 6
criteria.** They're overlapping but distinct rubrics. This isn't a
bug per se — the spec's other criteria are verified separately —
but it means "grade A" is **not** sufficient evidence for "spec
§12.4 fully satisfied."

**Possible resolutions** (for the audit or for a Phase 1 spec
amendment session):

1. **Tighten the Phase 1 spec** to map cleanly onto the grade tool
   (e.g., split §12.4 into "automated grade requirements" and
   "manual checks" with explicit cross-reference).
2. **Tighten the grade tool** to cover the spec (add fmt,
   tests-green, rand-free, rustdoc-pieces gates).
3. **Acknowledge in the PR description** that "grade A" is one piece
   of the §12.4 evidence among several, and table the alignment as
   a follow-on.

This decision can be made independently of the grade tool rebuild —
e.g., even if the rebuild keeps the same 7 criteria, the spec text
may want to be more honest about the mapping. Or, if the rebuild
expands the criteria, the spec mapping becomes cleaner naturally.

### F. Code-vs-standard drift (NEW — surfaced from `docs/STANDARDS.md` during the verification pass)

`docs/STANDARDS.md` (583 lines) is the canonical document referenced
by `grade.rs:3` as defining the seven-criterion grading system. The
verification pass that produced this revision read the standard
end-to-end (roughly the first 480 lines, all seven criteria fully
covered) and found **six concrete ways the tool's behavior diverges
from what the standard says it should be doing**. These are not bugs
in the "the code might have an error" sense — the code does what it
does deliberately. They're drifts in the "the code does one thing,
the canonical document says another" sense, and each one carries an
implicit decision: which document is authoritative going forward?

#### F1 — Coverage threshold: standard says ≥75% (target 90%), tool enforces ≥90%

- **Standard** (`docs/STANDARDS.md` line 26 — row 1 of the
  seven-criterion table at lines 24-32 — expanded at lines 38-67
  in the Criterion 1 section):
  ```
  | 1 | Test Coverage | ≥75% (target: 90%) | `cargo tarpaulin` (Linux only) |
  ```
  Plus: `Line coverage ≥75% (target: 90%)` in the requirements
  checklist at line 52.
- **Tool** (`grade.rs:365`):
  ```rust
  let grade = if coverage >= 90.0 {
      Grade::A
  ```
  Plus: `threshold: "≥90%"` at line 379.
- **Drift**: The 90% in the tool is the *target*, not the *A
  standard*. The actual A standard is 75%. The tool is **stricter**
  than the standard.
- **Consequence**: A crate at 80% coverage is A under the standard
  but B under the tool. Mesh crates that have COMPLETION.md may have
  been graded against the harder bar than the standard required, OR
  may have predated this drift.
- **Decision needed**: Align tool down to 75%, align standard up to
  90%, or maintain a 75% A / 90% A+ split. The right answer depends
  on what was originally intended and which document is authoritative.
- **Severity**: Medium. The drift is in the "tool is stricter than
  standard" direction, which is the safer of the two possible
  drifts. But it still means the tool's A grade and the standard's A
  grade are different things.

#### F2 — Tarpaulin platform mismatch: standard says Linux-only, tool runs unconditionally

- **Standard** (`docs/STANDARDS.md` line 48):
  > Note: `cargo tarpaulin` only works reliably on Linux. Mac/Windows
  > users should rely on CI for coverage reporting.
- **Tool** (`grade.rs:328-381`, function `grade_coverage`): runs
  tarpaulin unconditionally regardless of `cfg!(target_os = ...)`.
  No platform check. The 17-minute tarpaulin run we just observed on
  this macOS machine is the user-facing manifestation of running an
  unsupported configuration.
- **Drift**: The tool ignores the standard's platform guidance.
- **Consequence**:
  - macOS/Windows developers get a 17-minute local run that the
    standard explicitly says they shouldn't be relying on.
  - The grade tool produces a coverage number on a platform where
    the standard says CI should be authoritative.
  - On macOS specifically, tarpaulin's reliability is sketchy enough
    that the standard called it out — meaning even if the tool's
    local run "succeeds," the result may not match what CI would
    produce.
- **Decision needed** (load-bearing for the rebuild):
  1. **Gate tarpaulin behind `cfg(target_os = "linux")`** — preserves
     the tool but honors the standard. Non-Linux developers see a
     "coverage: skipped (platform not supported)" message and rely on
     CI, exactly as the standard prescribes.
  2. **Switch to a coverage tool that supports macOS/Windows as
     first-class platforms** (e.g., `cargo-llvm-cov`, see D5) so the
     local-developer experience is the same on every OS. This also
     resolves C2 (runtime cost) if the alternative is faster.
  3. **Update the standard** to drop the Linux-only restriction
     (only if tarpaulin's macOS support has actually improved since
     the standard was written).
- **Severity**: High. This is the most user-facing drift in the F
  section — every macOS/Windows developer who runs `cargo xtask
  grade` is currently getting an out-of-spec experience.

#### F3 — Safety pattern coverage gap: standard checks 6 patterns, tool checks only 2

- **Standard** (`docs/STANDARDS.md` lines 240-247):
  > **Measurement:**
  > ```bash
  > # In xtask, we grep for these patterns
  > grep -r "\.unwrap()" src/
  > grep -r "\.expect(" src/
  > grep -r "panic!" src/
  > grep -r "unreachable!" src/
  > grep -r "todo!" src/
  > grep -r "unimplemented!" src/
  > ```
- **Tool** (`grade.rs:487`):
  ```rust
  let patterns = [".unwrap()", ".expect("];
  ```
- **Drift**: Tool checks **2 of 6** patterns. Missing:
  `panic!`, `unreachable!`, `todo!`, `unimplemented!`.
- **Consequence**: A crate with `todo!()` or `unimplemented!()` in
  production code passes criterion 4 under the tool. Per the
  standard's requirements list (`docs/STANDARDS.md` lines 250-256):
  > - [ ] Zero `todo!()` or `unimplemented!()` in shipped code
  > - [ ] All `unreachable!()` have comments explaining why
  
  Such a crate should fail criterion 4 entirely. The current tool
  silently passes it.
- **Decision needed**: Extend the tool's grep patterns to all 6.
  This is essentially mechanical — add 4 more patterns to the array
  at line 487 and re-test. The harder question is whether `todo!`
  and `unimplemented!` should be hard-fail (any occurrence is F)
  rather than counted toward the violation count, since the standard
  says "Zero" for these.
- **Severity**: Medium-high. Plausibly there are sim crates with
  `todo!()` markers that have been silently passing the tool.

#### F4 — Dependencies threshold invented out of whole cloth

- **Standard** (`docs/STANDARDS.md` lines 304-352, Criterion 5):
  Defines the standard as **"Minimal and Justified"** with a
  qualitative checklist:
  > - [ ] Each dependency has documented justification in Cargo.toml
  > - [ ] No duplicate functionality (e.g., both `rand` and `fastrand`)
  > - [ ] Heavy dependencies are feature-gated
  > - [ ] Version constraints are reasonable (not overly restrictive)

  **No numerical threshold for dep count anywhere in the standard.**
- **Tool** (`grade.rs:565`):
  ```rust
  let grade = if dep_count <= 7 {
      Grade::A
  ```
  Plus: `threshold: "≤7"` at line 579, and a justification comment
  at lines 563-564:
  ```rust
  // Thresholds: A ≤7 allows reasonable crates (mesh-types + error + logging + domain-specific)
  // GPU crates legitimately need wgpu + bytemuck + pollster + thiserror + tracing
  ```
- **Drift**: The tool **invents** a numerical threshold the standard
  doesn't have. The justification comment is reasonable but is not
  in the canonical document.
- **Consequence**:
  - A crate with 8+ deps that satisfies all the standard's
    qualitative requirements (every dep documented in Cargo.toml,
    no duplicates, heavy deps gated) still fails criterion 5 under
    the tool.
  - Conversely, a crate with 5 deps that has zero of them documented
    passes the tool but fails the standard.
  - The tool measures dep count; the standard measures dep
    *justification*. **Different things being scored as the same
    grade.**
- **Decision needed**:
  1. **Codify a numerical threshold in the standard** (and pick the
     right number — 7? 10? unbounded for some crate categories?).
  2. **Replace the tool's count check with a qualitative gate** —
     e.g., "every dep entry in Cargo.toml has a comment immediately
     above it." This is the option that aligns the tool with the
     standard's intent.
  3. **Both** — count *and* justification, with the count as a soft
     warning and the justification as a hard requirement.
- **Severity**: High. The tool and the standard are measuring
  fundamentally different things under the same grade name.

#### F5 — Dependencies measurement command differs

- **Standard** (`docs/STANDARDS.md` lines 308-311):
  ```bash
  cargo tree -p <crate> --edges normal
  ```
  No `--depth` flag; implies the full transitive tree.
- **Tool** (`grade.rs:555`):
  ```rust
  let output = cmd!(sh, "cargo tree -p {crate_name} --edges normal --depth 1")
  ```
  Adds `--depth 1` — counts only direct deps.
- **Drift**: Tool counts direct deps; standard implies full transitive
  tree. **Different things are being measured.**
- **Consequence**: The "5 deps" reported for sim-thermostat is direct
  deps only (`nalgebra, rand, rand_chacha, rand_distr, sim-core`).
  The transitive closure is much larger — sim-core alone pulls in
  nalgebra, parry3d, and many other transitive deps.
- **Decision needed**: Align tool with standard (drop `--depth 1`),
  or update standard to clarify "direct deps only." This is downstream
  of F4 (the threshold question) — the count semantic and the
  threshold numeric should be settled together.
- **Severity**: Medium. The drift is silent (the user sees "5 deps"
  and assumes that's what the standard tracks; it isn't).

#### F6 — Bevy-free measurement drops `wgpu` (philosophical drift, requires explicit decision)

- **Standard** (`docs/STANDARDS.md` lines 359-362):
  ```bash
  cargo tree -p <crate> | grep -E "(bevy|wgpu|winit)"
  # Must return empty
  ```
  Plus the requirements list at lines 367-370:
  > - [ ] `cargo tree` shows no bevy, wgpu, or winit
  > - [ ] No bevy types in public API
  > - [ ] No bevy traits implemented directly (only in cortenforge crate)
  > - [ ] Can be compiled for `--target wasm32-unknown-unknown` without bevy
  
  **The standard treats `wgpu` as Layer-0-disqualifying.**
- **Tool** (`grade.rs:597-598`):
  ```rust
  let has_bevy = output.contains("bevy");
  let has_winit = output.contains("winit");
  ```
  Checks **only** bevy and winit. Lines 583-587 contain an explicit
  docstring justifying the exclusion of wgpu:
  ```rust
  /// Layer 0 crates should not depend on Bevy or windowing libraries.
  /// Note: `wgpu` is NOT considered a Bevy dependency - it's a standalone
  /// WebGPU implementation that can be used independently for compute shaders.
  ```
- **Drift**: **Substantive philosophical disagreement.** The grade
  tool's author intentionally diverged from the standard with a
  reasoned justification. Whoever wrote the tool decided wgpu was
  acceptable in Layer 0; whoever wrote the standard decided it was
  not. The two documents disagree on what "Layer 0" *means*.
- **Consequence**:
  - Sim-gpu (depends on wgpu) passes the tool's criterion 6. Per the
    standard, it would fail.
  - Any future GPU-using Layer 0 crate inherits the same divergence.
  - Sim-thermostat is unaffected (no wgpu dep), so this drift didn't
    manifest in the Phase 1 PR prep — but it would manifest the
    moment any GPU-touching crate is graded.
- **Decision needed** (load-bearing — affects the meaning of "Layer
  0" going forward):
  1. **Standard is authoritative**: wgpu is disallowed in Layer 0.
     The grade tool is wrong; sim-gpu is structurally non-compliant
     and needs to either move to Layer 1 or shed wgpu (impossible
     for a GPU crate). This is the most disruptive option.
  2. **Tool is authoritative**: wgpu is fine in Layer 0 as a
     compute-shader infrastructure layer. The standard needs to be
     updated. This is the philosophical position the tool's author
     already took.
  3. **Both, with refinement**: distinguish "wgpu as compute shader
     dependency" (allowed in Layer 0) from "wgpu as part of a
     windowing/rendering stack" (not allowed). Requires a more
     nuanced rule that probably can't be expressed via simple
     substring grep.
- **Severity**: High (most substantive of the F items). The
  decision shapes what "Layer 0" means architecturally and affects
  every GPU-touching crate in the workspace.

## Evidence captured

### Grade tool output (verbatim, from this session's failing run)

```
╔══════════════════════════════════════════════════════════════╗
║                   GRADING: sim-thermostat                    ║
╠══════════════════════════════════════════════════════════════╣
║ Criterion        │ Result           │ Grade │ Threshold      ║
╠══════════════════════════════════════════════════════════════╣
║ 1. Coverage      │ 9.1%             │   F   │ ≥90%           ║
║ 2. Documentation │ 0 warnings       │   A   │ 0              ║
║ 3. Clippy        │ 0 warnings       │   A   │ 0              ║
║ 4. Safety        │ 0 violations     │   A   │ ≤5             ║
║ 5. Dependencies  │ 5 deps           │   A   │ ≤7             ║
║ 6. Bevy-free     │ ✓ confirmed      │   A   │ no bevy        ║
║ 7. API Design    │ (manual review)  │   ?   │ checklist      ║
╠══════════════════════════════════════════════════════════════╣
║ AUTOMATED        │                  │   F   │                ║
║ OVERALL          │                  │   ?   │ needs review   ║
╚══════════════════════════════════════════════════════════════╝

✗ Automated grade: F. Refactor required before completion.
```

Note: against the F section, **every "A" in this output should be
read as "passes the tool, may or may not pass the standard."** The
tool's A grades on criteria 4 (Safety) and 5 (Dependencies) in
particular are not equivalent to the standard's A grade for those
criteria — see F3 and F4.

### Coverage data location (ephemeral)

- `target/tarpaulin/coverage.json` — full coverage data, 637 file
  entries. Computed from this session's grade run.
- `target/tarpaulin/tarpaulin-report.json` — alternate format, ~14MB.
- `target/tarpaulin/profraws/` — raw llvm-cov profile data.
- **The data is NOT committed** (`target/` is gitignored). Re-running
  grade overwrites it. Reproducing requires another ~17-minute run.

### A1 fix details

Commit `c3f08c5` on `feature/thermo-doc-review`:

- `xtask/Cargo.toml` — added `serde_json = "1"` dep
- `xtask/src/grade.rs` — rewrote `find_crate_path` (new function at
  lines 292-325) to use `cargo metadata --format-version 1
  --no-deps`, parse JSON with `serde_json::Value`, find the package
  by name, derive directory from `manifest_path`, rebase onto
  `workspace_root`. ~25 LOC of new function code (replacing ~22 LOC
  of old function code).
- `Cargo.lock` — auto-updated for the new dep (and its transitive
  deps: `itoa`, `memchr`, `ryu`, etc.)
- Pre-commit hook passed: fmt clean, clippy clean.
- Verified post-fix: `cargo xtask grade sim-thermostat` got past the
  path lookup and into the coverage gate (where it then hit A2).

### sim-thermostat per-file coverage (computed by hand from coverage.json)

| File | Lines | Hit | % |
|---|---:|---:|---:|
| `sim/L0/thermostat/src/component.rs` | 2 | 2 | 100.0% |
| `sim/L0/thermostat/src/diagnose.rs` | 0 | 0 | n/a |
| `sim/L0/thermostat/src/langevin.rs` | 26 | 26 | 100.0% |
| `sim/L0/thermostat/src/lib.rs` | 0 | 0 | n/a |
| `sim/L0/thermostat/src/stack.rs` | 50 | 45 | 90.0% |
| `sim/L0/thermostat/src/test_utils.rs` | 53 | 53 | 100.0% |

The 5 uncovered lines in `stack.rs` are at lines 186-190 — a
multi-line `debug_assert!` panic message string that fires only on
user misuse of `install_per_env`. Effective production-path coverage
is 100%.

### STANDARDS.md key quotations (sourced for the F section)

The standard's seven-criterion table (`docs/STANDARDS.md` lines
24-32):

```
| # | Criterion | A Standard | Measurement |
|---|-----------|------------|-------------|
| 1 | Test Coverage | ≥75% (target: 90%) | cargo tarpaulin (Linux only) |
| 2 | Documentation | Zero warnings | RUSTDOCFLAGS="-D warnings" cargo doc |
| 3 | Clippy | Zero warnings | cargo clippy -- -D warnings |
| 4 | Safety | Zero unwrap/expect | grep + review |
| 5 | Dependencies | Minimal, justified | cargo tree + review |
| 6 | Bevy-free (Layer 0) | No bevy in tree | cargo tree | grep bevy |
| 7 | API Design | Idiomatic, intuitive | Manual review |
```

Note the table-row summary for criterion 4 says only "Zero
unwrap/expect," while the expanded measurement section at lines
240-247 lists 6 patterns (see F3). Even within STANDARDS.md itself
there's a tension between the summary and the expansion. This is
relevant for the rebuild — the rebuild may need to choose which
representation is canonical.

## What's parked

### Phase 1 PR

- **Status**: Parked pending grade tool audit. Cannot ship until
  Phase 1 spec §12.4 #5 can be honestly satisfied, which requires
  either (a) the audit closing and the grade tool being trustworthy,
  or (b) §12.4 #5 being amended.
- **Branch**: `feature/thermo-doc-review`
- **HEAD at original capture**: `c3f08c5` (the find_crate_path fix;
  previous head was `9998119`, the chassis Option C amendment). HEAD
  will advance after this audit doc + persistence updates commit.
- **Working tree at HEAD**: clean (post-fix, before audit doc commit).
- **Phase 1 implementation status**: complete and validated. 35 unit
  tests + 5 mandatory integration tests + 1 sweep test (`#[ignore]`'d)
  all green; clippy + fmt clean; rustdoc has the four chassis
  Decision 6 pieces; sim-core stays rand-free in production. **Six
  of seven §12.4 criteria are satisfied; only #5 (grade A) is open**,
  and the openness is due to grade tool gaps, not sim-thermostat gaps.

### Decisions deferred

1. **Audit scope (A/B/C/D)** — see audit-options table in
   `recon_plan.md`.
2. **Phase 1 spec §12.4 #5 amendment** — see E above.
3. **Tarpaulin vs cargo-llvm-cov** — see D5 (and F2 for the
   standards-driven case).
4. **Whether COMPLETION.md should be created for sim-thermostat** —
   depends on whether the audit's rebuild produces a trustworthy
   grade.
5. **Whether mesh COMPLETION.md files need to be re-graded** under
   whatever the rebuild produces — depends on D4. Sharpened by F1,
   F3, F4: even if the COMPLETION files were generated by the
   current grade.rs, the underlying thresholds may have drifted
   from the standard.
6. **Code-vs-standard drift resolution** — for each of F1-F6, which
   document is authoritative? F2 (tarpaulin platform) and F6 (wgpu
   philosophical) are the load-bearing decisions; the others are
   more mechanical.
7. **Whether STANDARDS.md itself needs updating** — the audit may
   surface tensions in the standard (e.g., the criterion-4 table-row
   vs expanded-measurement tension noted above) that should be
   resolved before the rebuild grades anything new.

## Cross-references

- **Originating session**: 2026-04-09 Phase 1 PR prep
- **Phase 1 spec**: `docs/thermo_computing/03_phases/01_langevin_thermostat.md` §12.4
- **Phase 1 implementation**: `sim/L0/thermostat/`
- **Grade tool source**: `xtask/src/grade.rs`
- **Standards (canonical rubric, EXISTS)**: `docs/STANDARDS.md` (583
  lines; the F section quotes the criteria and measurement sections;
  the next session should read the rest end-to-end as part of D1)
- **A1 fix commit**: `c3f08c5`
- **Pre-A1 branch HEAD**: `9998119`
- **Recon plan for next session**: `docs/grade_tool_audit/recon_plan.md`
- **Initiative README**: `docs/grade_tool_audit/README.md`
- **Auto-memory entries** (out of repo, in
  `~/.claude/projects/.../memory/`):
  - `project_grade_tool_audit.md` (new)
  - `project_thermo_computing.md` (updated to reflect parked PR)
  - `feedback_recon_to_iteration_handoff.md` (the principle that
    motivates this audit)
  - `feedback_sharpen_the_axe.md`
  - `feedback_grading_rubric.md`
  - `feedback_rocket_platform.md`

---

*End of audit findings. The next session's job is captured in
`recon_plan.md`.*
