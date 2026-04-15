# Platform Infrastructure Audit

**Branch:** `feature/platform-infra-audit`
**Opened:** 2026-04-15
**Status:** Recon-ready; execution begins next session
**Mission:** Sharpen-the-axe pass on the platform layer (xtask grader, sim/L0
discipline, examples, docs, memory) before thermo-RL critical-path work begins.

This is the **live working checklist**. Items get checked off as they ship.
The companion memory file
([`project_platform_infrastructure_audit.md`](../.. memory pointer))
holds the *why* and the durable index entry; this file is the *what* and the
*how-far-along*.

---

## Why this audit, why now

Three reasons, all pointing the same direction:

1. **Fresh tree state.** PR #192 (`2835b4f4`, merged 2026-04-15) just
   touched every `sim_ml_bridge::` consumer, every downstream `Cargo.toml`,
   every example crate. Ground truth across the platform layer is the
   clearest it will ever be. This is the cheapest possible moment to audit;
   miss it and latent issues compound through thermo-RL spec work.

2. **The grader doesn't match the standard it enforces.** "A-grade or it
   doesn't ship" is enforced by `cargo xtask grade`, but PR #192 surfaced
   four concrete issues — two real gaps in `grade_*` functions, one
   test-pattern incompatibility, and one coverage gap where the grader
   hadn't been run against current state:
   - `xtask/src/grade.rs::has_enclosing_allow` (line ~995) uses a loose
     300-line backward substring scan that can false-negative on panic
     justification.
   - `grade_clippy` (line ~660) accepts `//` but not `///` as preceding-line
     justification for `#[allow(clippy::...)]`.
   - Wall-clock assertions (`assert!(m.wall_time_ms < N)`) in test code
     crash under llvm-cov's ~10× instrumentation overhead, breaking the
     grader's coverage pass on any crate that holds them. Not a bug in a
     `grade_*` function, but a test-side anti-pattern the grader can't
     currently tolerate — four were deleted from sim-opt during PR #192
     alone; more may exist elsewhere. (Item 5 sweeps for stragglers.)
   - sim-opt's `Cargo.toml` had unjustified deps that were only caught
     mid-PR. `grade_dependencies` (line ~1021) would have flagged them on
     any run — the gap isn't that the checker is broken, it's that the
     grader hadn't been run against sim-opt's current dep set. Other crates
     may be in the same state for the same reason. (Item 4 sweeps for
     stragglers.)

   The grader should be as tight as the discipline it enforces. Anything
   less is self-contradictory.

3. **`feedback_sharpen_the_axe.md` + `feedback_r34_architecture.md` point
   exactly here.** Starting thermo-line north-star work on a platform with
   known-latent debt would be rushing in disguise. The R34 chassis principle
   — overbuild the chassis so aftermarket can push 3× factory spec — applies
   to the grader and discipline scaffolding too, not just trait boundaries.

---

## Execution philosophy

This is a **persistent checklist**, not a time-boxed sprint. Each item gets
the proper amount of attention/compute/time it needs. Items run in their
natural sequence (Tier A grader items first because they inform Tiers B–D),
but there is no pressure to finish in N sessions.

**Three-phase workflow per item:**

1. **Recon** — typically delegated to a single Explore agent with a tight
   prompt. Returns a punch list of findings.
2. **Triage** — categorize each finding using the rubric below.
3. **Fix** — execute (a)-category items inline as `chore(infra):` or
   `chore(xtask):` commits on this branch. File (b)/(c) items as their own
   tracked work. Skip (d) items with one-line rationale logged here.

**Categorization rubric for findings:**

| Tag | Meaning | Action |
|-----|---------|--------|
| (a) | Fixable inline now | Add to current commit batch on this branch |
| (b) | Needs its own dedicated PR | File a TODO line under that item; ship after audit closes or on a parallel branch |
| (c) | Larger scope; file as separate project | Create a `project_*.md` memory entry, link here, defer |
| (d) | Decline to fix | Log one-line rationale in the findings section below |

**Per-criterion discipline (from `feedback_xtask_grade_opacity.md`):** never
loop on full `cargo xtask grade` runs during iteration. Use targeted checks
(`cargo test -p <crate> --lib --release`, `cargo clippy`, manual greps) and
escalate to the full grader only as a final confirmation. Item 9 below
implements logging that should make item 10's grade sweep tractable.

**Branch hygiene:** all audit commits land on `feature/platform-infra-audit`.
Bundle related items into single commits where it makes sense (e.g., the
panic audit fix and the grader hardening go together). PRs back to main can
be one big PR or several small ones — decide at audit close, not now.

---

## Audit checklist

Status legend: ☐ pending · ◐ in progress · ☑ done · ⊘ skipped (with reason)

### Tier A — grader self-consistency (high-value, tightly coupled)

#### ☑ 1. Grader criterion correctness sweep

Re-read all 7 criteria in `xtask/src/grade.rs` against their docstrings and
`docs/STANDARDS.md`. Goal: catch gaps beyond the four already known.

**Specific checks:**
- Does each `grade_*` function actually verify what its docstring claims?
- Does each criterion's pass/fail logic match the corresponding section of
  `docs/STANDARDS.md`?
- Are there criteria with stale thresholds (coverage %, warning counts)
  that no longer match the standard?
- Are there criteria silently double-counting or under-counting (e.g., a
  warning that flows through both clippy and safety paths)?

**Recon prompt sketch:** "Read `docs/STANDARDS.md` and every `grade_*`
function in `xtask/src/grade.rs`. For each criterion, report (a) what the
standard says, (b) what the function actually checks, (c) any delta. ~1h
read budget. Output as a 7-row table."

**Time:** ~1h recon + ~30min triage. Findings likely feed items 2–4.

**Findings:**

Recon ran 2026-04-15 via an Explore agent reading `docs/STANDARDS.md` and
every `grade_*` function body end-to-end. Delta table:

| # | Criterion | Standard | grade.rs implementation | Delta |
|---|-----------|----------|-------------------------|-------|
| 1 | Test Coverage | ≥75% (A), ≥90% (A+) via llvm-cov | Two-tier 75/90 with two-pass (unit-only for coverage, all for correctness), per-crate path filtering. `grade.rs:461–600` | None |
| 2 | Documentation | Zero warnings under `-D warnings` | Runs `cargo doc -D warnings`, gates on exit code + stderr `warning:`/`error:` count. `grade.rs:606–653` | None |
| 3 | Clippy | Zero warnings; `#[allow(clippy::...)]` must be justified | JSON-parses clippy diagnostics; separate scan for unjustified `#[allow(clippy::…)]`, accepts `//` (not `///`/`//!`) within 1–3 lines. Test-exclusion used a one-way `in_test` flag without brace tracking. `grade.rs:660–758` | **NEW — MED**: test-exclusion latch never reset, silently skipping every `#[allow(clippy::...)]` in library code textually after the first `#[cfg(test)]` module in a file. Plus the known `///` asymmetry. |
| 4 | Safety | Zero unwrap/expect/panic/unreachable/todo/unimplemented in lib; `// SAFETY:` on unsafe | Brace-depth-correct test exclusion (824–877); hard-fail on `todo!`/`unimplemented!`; counts the rest; case-insensitive `// SAFETY:` check; profile-relaxed for Examples/Xtask. `grade.rs:787–968` | Known loose `has_enclosing_allow` (line 995); no new findings |
| 5 | Dependencies | Each dep has `#`-comment justification | Scans `[dependencies]`/`[dev-dependencies]`/`[build-dependencies]`; inline-`#` or 1–3 preceding-line `#`, halts on blank/section header. `grade.rs:1021–1139` | None — reframed: earlier "silently incomplete" claim was wrong, the grader would have flagged sim-opt on any run |
| 6 | Bevy-free (Layer 0) | `cargo tree` shows no bevy/winit | Profile-gated to Layer 0 only; runs `cargo tree --prefix none --format "{p}"`, exact-match `starts_with("bevy")` / `== "winit"`. `grade.rs:1154–1214` | None |
| 7 | API Design | Manual review | Always returns `Grade::Manual`. `grade.rs:225–231` | None (intentional) |

**New finding — clippy test-exclusion latch (MED):** `grade_clippy` set
`in_test = true` on the first `#[cfg(test)]` encountered in a file and
never reset it, so every `#[allow(clippy::...)]` in library code
appearing textually *after* the first test module in the same file was
silently skipped. The inline comment at line 691 explicitly acknowledged
the gap (`"brace-depth tracker in Step 5"`) but Step 5 never landed.
Fixed by mirroring `grade_safety`'s brace-depth state machine
(lines 824–877) into `grade_clippy`. Commit `b7ef1c73`.

**Reframing of earlier premises (doc-only fix):** two of the four bullets
in "Why this audit, why now" §2 were inflated:
- Wall-clock asserts (bullet c) are a test-side pattern incompatible with
  llvm-cov, not a bug in a `grade_*` function. Item 5 still covers the
  sweep; framing corrected.
- `grade_dependencies` (bullet d) is correct — sim-opt's missing
  justifications were a real miss the grader would have flagged on any
  run. The gap is that the grader hadn't been run against sim-opt's
  current dep set, not that the checker is broken. Item 4 still covers
  the sweep; framing corrected.

Only 2 of the 4 originally-listed bullets are real grader bugs:
`has_enclosing_allow` looseness and `//`-vs-`///` asymmetry. Both remain
open (item 2 addresses the former; item 3 addresses the latter). Commit
`ab38945b`.

**Dispositions:**
- Clippy latch fix — **(a)**, shipped as `b7ef1c73`.
- Framing correction — **(a)**, shipped as `ab38945b`.
- State-machine deduplication between `grade_clippy` and `grade_safety`
  (two near-identical ~50-line brace-depth machines) — **(b)**, filed as
  a follow-up. Not shipping inline to keep this item's diff bounded.
- Unit tests for the grader's state machines — **(b)**, filed as a
  follow-up. The grader currently has no `#[cfg(test)]` coverage and
  adding tests just for this fix would be inconsistent.

---

#### ☑ 2. Panic / unreachable span audit

(Absorbed from former `project_grader_safety_hardening.md`.)

Enumerate every `panic!()` and `unreachable!()` call site under
`sim/L0/*/src/` (excluding `#[cfg(test)]` blocks) and pair each with whether
its enclosing scope has a real `#[allow(clippy::panic)]` /
`#[allow(clippy::unreachable)]` attribute. Compute the delta between what
the current loose `has_enclosing_allow` (line ~995) accepts and what a
strict span-aware check would accept.

**Background — why the helper is currently loose:** the substring scan was
chosen deliberately to handle multi-line `#[allow(...)]` attributes without
an AST parser:

```rust
#[allow(
    clippy::panic,
    clippy::unwrap_used
)]
fn foo() { ... panic!("..."); ... }
```

A naïve `lines[j].contains("#[allow") && lines[j].contains(lint)` same-line
check would false-negative here. The 300-line substring scan catches the
multi-line case at the cost of looseness — a regular comment mentioning
`clippy::panic` would also satisfy it.

**Span-aware helper sketch (~40 lines):**

```rust
fn has_enclosing_allow(lines: &[&str], i: usize, lint: &str) -> bool {
    // Scan backward from line i looking for `#[allow(` openings.
    // For each opening, find the matching `)]` closing on the same line
    // or a subsequent line (up to `i`). Check whether the lint name
    // appears inside the opening..closing span.
    let start = i.saturating_sub(300);
    for open_idx in (start..i).rev() {
        let open_line = lines[open_idx];
        let Some(open_col) = open_line.find("#[allow(") else { continue };
        let mut body = String::new();
        body.push_str(&open_line[open_col + "#[allow(".len()..]);
        let mut close_idx = open_idx;
        while !body.contains(")]") && close_idx + 1 <= i {
            close_idx += 1;
            body.push('\n');
            body.push_str(lines[close_idx]);
        }
        if !body.contains(")]") { continue; }
        let span = &body[..body.find(")]").unwrap()];
        if span.contains(lint) {
            return true;
        }
    }
    false
}
```

Still a heuristic (e.g., doesn't distinguish a struct-field allow 250 lines
up from a fn-level one), but dramatically tighter than the current scan.

**Why this isn't a blocker (context for triage):** `grade_clippy` does NOT
backstop this — `clippy::panic` is in `restriction`, not default-enabled,
and no Layer 0 crate opts in via `#![warn(clippy::panic)]`. The safety
criterion is the sole grader-level gate. Worst realistic outcome: one crate
holds an undeserved A for a grade cycle until a test hits the panic or a
reviewer reads the source. Small blast radius — but a known false-negative
window against an "A-grade or it doesn't ship" standard is precisely the
self-contradiction this audit exists to close.

**Recon prompt sketch:** "Enumerate every `panic!()` and `unreachable!()`
under `sim/L0/*/src/` excluding `#[cfg(test)]` blocks. For each, report
whether its enclosing scope has a real `#[allow(clippy::panic)]` /
`#[allow(clippy::unreachable)]` attribute (same-line, same-fn, same-impl,
same-mod, or crate-root `#![allow(...)]`). Use the current
`has_enclosing_allow` at `xtask/src/grade.rs:~995` as the loose baseline.
Report as a punch list grouped by crate."

**Action sequencing:**
1. Run the recon. Get the delta count.
2. **If delta is 0–2 sites:** quick-fix the exposed sites with proper
   `#[allow(clippy::panic)]` + `// <reason>` comments, then tighten the
   helper. One commit batch on this branch.
3. **If delta is larger:** the fix is still bounded (~15–30 min mechanical
   per site) but log the timeline estimate before committing to it.
4. Write the span-aware helper. Unit-test it with multi-line attribute
   cases and false-positive cases (a comment saying `clippy::panic`).
5. Run `cargo xtask grade` on every affected crate (now made tractable by
   item 9 below). Fix any newly-exposed sites.
6. Land as part of this branch's final commit batch.

**Cosmetic non-finding for context:** `sim-ml-chassis/src/stats.rs`'s
`diagonal_independence_sums_per_dim` test uses
`3.0f64.mul_add(-per_dim, lp).abs() < 1e-12` instead of the construction
spec's `(lp - 3.0 * per_dim).abs() < 1e-12`. Mathematically identical;
mul_add satisfies clippy `suboptimal_flops`. Ignore.

**Time:** ~10min recon delegate + ~20min review + ~1h fix (helper +
exposed sites + tests) = ~2–3h total.

**Findings:**

Recon strategy: a first-pass Explore agent returned results the main
thread couldn't trust after spot-checking (the agent applied a "is this
comment semantically related to the panic?" filter while `grade_safety`
applies a looser "any `//` comment within 1–3 preceding lines" filter).
Re-ran with **rustc itself** as the strict-check ground truth: enabling
`clippy::panic` and `clippy::unreachable` (both in `restriction`, off by
default) via `-W` flags on every Layer 0 crate, then parsing the warnings
with a Python script that mirrored `grade_safety`'s exact preceding-
comment / same-line-comment / 300-line loose-scan / brace-depth test
exclusion / `src/`-scope logic. This sidestepped the reimplement-rustc
problem by letting rustc answer "does a strict enclosing `#[allow]` exist
here?" natively.

**Classification of the 27 clippy warning sites** (all `unreachable!`,
zero `panic!`):

| Verdict | Count | Meaning |
|---------|-------|---------|
| TEST — inside `#[cfg(test)]` | 2 | Grader skips (flex_self.rs:1096, 1156) |
| OUT-OF-SCOPE — not under `<crate>/src/` | 1 | `sim/L0/tests/integration/sensors_phase4.rs` not scanned by `grade_safety` |
| CMT — grader accepts via preceding `//` comment | 10 | Grader's comment path is more permissive than clippy itself — not item 2's concern |
| **DELTA** — grader accepts via loose 300-line scan only | **0** | **Item 2's false-negative window is empty** |
| **HARD** — grader rejects today | **14** | Three crates silently fail `grade_safety` criterion 4 |

**Headline 1 — item 2's narrow question: no false-negatives.**
`has_enclosing_allow`'s 300-line loose substring scan is doing zero
false-negative work today. Every site that relies on it has a real
enclosing `#[allow]` that clippy agrees with. The planned span-aware
rewrite is purely preventive — not closing any active hole.

**Headline 2 — bigger finding: 14 HARD sites across sim-core, sim-mjcf,
sim-urdf.** These are naked `unreachable!()` calls in match arms and
`.unwrap_or_else(|| unreachable!(...))` fallbacks, lacking both a
justifying `//` comment and an enclosing `#[allow(clippy::unreachable)]`.
All three crates would currently fail `grade_safety` on criterion 4:

- **sim-core (10):** `collide_with_mesh` ×3 (mesh_collide.rs), `collide_capsule_box` (pair_cylinder.rs), `mjd_transition_hybrid` ×3 (derivatives/hybrid.rs), `mj_sensor_pos` (sensor/position.rs), `mj_fwd_tendon_spatial` ×2 (tendon/spatial.rs)
- **sim-mjcf (3):** `make_cable`, `append_to_chain`, `build_cable_geom` (builder/composite.rs)
- **sim-urdf (1):** `convert_joint` (converter.rs)

Phase-5/6 work (derivatives, mesh_collide SDF dispatch, composite cable
bodies, URDF conversion) accumulated these after the grader last ran on
these crates. Exactly the audit-coverage gap "Why this audit" anticipates.

**Fixes shipped:**

- **14 HARD sites → closed.** Added function-level
  `#[allow(clippy::unreachable)]` attributes to the 9 enclosing functions
  with same-line `//` justifications naming the specific invariant that
  makes each arm unreachable (dispatch ordering, validation upstream,
  variant guarantees from enclosing if-branches). Verified via clippy
  `-W clippy::unreachable` re-run on the three crates: HARD count dropped
  from 14 to 0. Standard `cargo clippy -- -D warnings` still clean.
  Commit `3e799bbd`.

- **Span-aware `has_enclosing_allow` (preventive).** Replaced the 300-line
  substring scan with a span-aware parser that finds each `#[allow(` /
  `#![allow(` opening in the window, walks to the matching `)]` (possibly
  multi-line for formatted attribute blocks), and checks whether the lint
  appears inside that span. A stray mention of the lint name in a
  comment, string literal, or unrelated allow attribute for a different
  lint no longer matches. Added the grader's first unit-test module with
  10 cases covering the interesting variants (same-line, multi-line,
  different-lint-negative, comment-mention-negative, string-literal-
  negative, inner `#![...]` attribute, 300-line window boundary both
  sides, multiple-allows-one-matches). Closes item 1's (b)-follow-up on
  grader unit tests. Commit `9438d8e2`.

**Recon methodology reusable for items 3–5.** The "clippy with `-W` +
Python mirror of grader logic" pipeline is the durable artifact of this
item. Script at `/tmp/panic-audit/` during this session — worth promoting
to an xtask subcommand in the future if items 3–5 exercise the same
pattern.

**Dispositions:**

- Clippy recon + Python classification → **(a)**, session-local tooling.
- 14 HARD fixes → **(a)**, shipped as `3e799bbd`.
- Span-aware helper + 10 unit tests → **(a)**, shipped as `9438d8e2`.
- Promoting the clippy-based audit pipeline to an xtask subcommand
  (`cargo xtask audit-panics` or similar) → **(b)**, file as follow-up if
  items 3–5 want the same methodology. Not shipping inline.
- Systemic coverage gap (crates accumulating criterion-4 violations
  because the grader hasn't been run against their current state) →
  **absorbed by item 10** (per-crate grade verification). Item 10 will
  re-verify the three fixed crates and catch any stragglers across the
  other 8 Layer 0 members.

---

#### ☐ 3. Non-test `#[allow(clippy::...)]` justification audit

Every non-test `#[allow(clippy::...)]` under `sim/L0/*/src/` should have a
`//` (not `///`) preceding-line comment within 1–3 lines. PR #192 added 8
such comments across the chassis/rl split; others may still be missing in
crates the grader hasn't touched recently.

**Recon prompt sketch:** "Grep for `#[allow(clippy::` under `sim/L0/*/src/`
excluding `#[cfg(test)]` blocks and `tests/` directories. For each hit,
report whether the 1–3 preceding lines contain a `//` comment (not `///`,
not blank). Output as a punch list grouped by crate."

**Time:** ~15min recon + fixes are 1 line each.

**Findings:**
_(none yet)_

---

#### ☐ 4. Dep justification coverage

Every `Cargo.toml` under `sim/L0/`, `sim/L1/`, and `examples/fundamentals/`
has each dep prefixed by a `#`-comment justification per `grade_dependencies`
(line ~1021). sim-opt had unjustified deps caught mid-PR #192; other crates
may too because the grader hasn't been run against their current dep sets.

**Recon prompt sketch:** "For each `Cargo.toml` under `sim/L0/`, `sim/L1/`,
and `examples/fundamentals/` (skipping the workspace root), report any dep
line that lacks a `#`-comment justification on the line(s) immediately
above it. Use `xtask/src/grade.rs::grade_dependencies` (line ~1021) as the
spec for what counts as a justification."

**Time:** ~15min recon + 1-line fixes.

**Findings:**
_(none yet)_

---

### Tier B — anti-pattern sweeps (fast, repo-wide grep)

#### ☐ 5. Wall-clock assertion sweep

Grep for `assert!(...)` patterns matching `wall_time_ms`, `elapsed`,
`Duration::as_*`, `Instant::elapsed`, etc., across all workspace test files.
Four were deleted from sim-opt during PR #192:
- `cem.rs:329` (commit 3)
- `algorithm.rs:420` (commit 5)
- `richer_sa.rs:543` (commit 5)
- `parallel_tempering.rs:552` (commit 5)

These crash under llvm-cov's ~10× instrumentation overhead and are a flaky
anti-pattern even outside grading. Survey for stragglers.

**Recon prompt sketch:** "Grep workspace-wide for `assert!` / `assert_*`
calls that compare wall-clock measurements (`wall_time_ms`, `elapsed`,
`Duration::as_*`, `Instant::elapsed`, `start.elapsed()`). Include both
`tests/` directories and `#[cfg(test)]` blocks. Report file:line + the
asserted threshold. Exclude historical-record files under `archive/`."

**Time:** ~10min recon + delete-on-sight fixes.

**Findings:**
_(none yet)_

---

#### ☐ 6. Doc + post-split hygiene

Stale `sim_ml_bridge` / `sim-ml-bridge` references outside historical
git-commit quotes; dead file-path pointers in rustdoc; outdated module-doc
API claims; stale `bevy` feature-flag patterns from before the split.

**Recon prompt sketch:** "Workspace-wide grep for `sim_ml_bridge` and
`sim-ml-bridge` outside `archive/`, `docs/thermo_computing/`, and any file
clearly historical (e.g., commit-quote contexts). Report each hit with
3 lines of context. Also: spot-check `lib.rs` module docs across `sim/L0/`
crates for stale API claims (functions/types that no longer exist
post-split). Also: grep for `bevy` feature-flag patterns that may have been
left over from sim-bevy work and check they still resolve."

**Time:** ~20min recon + 1-line fixes per hit.

**Findings:**
_(none yet)_

---

### Tier C — broader cleanup

#### ☐ 7. Test organization audit

Overlapping coverage between sim-ml-chassis unit tests and sim-rl
integration tests; dead fixtures; misplaced integration tests (e.g.,
chassis-only tests that ended up under `sim-rl/tests/` or vice versa
post-split). This item leans on judgment more than mechanical scanning, so
expect more (b)/(c) findings than (a) findings.

**Recon prompt sketch:** "Read every `tests/*.rs` file under
`sim/L0/ml-chassis/`, `sim/L0/rl/`, and `sim/L0/opt/`. For each test,
report: (1) what crate's symbols it actually exercises, (2) whether it
duplicates coverage of any other test in the trio, (3) whether it appears
to be in the right crate given its dependencies. Also: list any fixture
files or test data that look orphaned (not referenced by any test). Output
as a per-file table."

**Time:** ~30min recon + ~30min triage. Most findings likely (b)/(c).

**Findings:**
_(none yet)_

---

#### ☐ 8. Memory system hygiene

Stale `project_*.md` files in the memory dir; duplicate coverage between
project files; MEMORY.md index entries pointing at closed initiatives;
line-count budget against the 200-line truncation limit (currently ~87).

**Specific checks:**
- Any `project_*.md` for an initiative listed under "Resolved Blockers" or
  "Completed Work" that should be archived/deleted?
- Any `feedback_*.md` whose rule has been internalized to the point where
  the file is no longer load-bearing?
- Any duplicate coverage where two memory files say the same thing about
  the same initiative?
- Is MEMORY.md still under the 200-line budget?
- Any broken links in MEMORY.md (entries pointing at files that no longer
  exist)?

**Time:** ~30min total. Likely all (a) fixes.

**Findings:**
_(none yet)_

---

### Tier D — capstone

#### ☐ 9. xtask progress logging

Implement progress logging on long-running xtask commands — `cargo xtask
grade` especially. Per `feedback_xtask_grade_opacity.md`, the full grade
suite freezes the terminal for 10–20+ minutes per crate with no output,
which is frustrating and obscures progress. Item 10 below (full grade
verification) is impractical without this.

**Minimum bar:**
- Print "running cargo llvm-cov pass 1/2 (~10 min for instrumented test
  runs)" before each long-running subprocess.
- Print "criterion N/7: <name> — running" before each criterion check.
- Print elapsed wall-clock per criterion when it completes.
- Optionally: a heartbeat (one line every 30s) during the longest stages.

**Stretch goals (separate finding if scope balloons):**
- Structured per-criterion timing summary at the end.
- `--quiet` / `--verbose` toggles.
- JSON output mode for CI consumption.

**Recon prompt sketch:** "Read `xtask/src/grade.rs` end-to-end and identify
every `cmd!()` / `Command::new()` call that runs longer than ~5 seconds in
practice. For each, report the surrounding context and what a human-readable
'starting X (estimated Y minutes)' line would look like. Also identify any
existing logging infrastructure (eprintln, log crate, etc.) so the
implementation matches house style."

**Implementation note:** keep the diff bounded. This is enabling work for
item 10, not a full xtask refactor. If an issue surfaces that wants a
deeper refactor, file it as (b) and ship the minimum-viable logging only.

**Time:** ~20min recon + ~1h implementation + ~20min testing = ~1.5–2h.

**Findings:**
_(none yet)_

---

#### ☐ 10. Per-crate grade verification

Run `cargo xtask grade <crate>` on every workspace member that is supposed
to hold an A grade. With item 9's logging in place, this is observable and
tolerable rather than a black-box freeze.

**Discipline (from `feedback_xtask_grade_opacity.md`):** even with logging,
don't loop on full grade runs during iteration. Use targeted per-criterion
checks during any fix-up loops; reserve the full grade for the final
confirmation pass per crate.

**Crates to verify (Layer 0 + Layer 1 minimum):**
- sim-types
- sim-simd
- sim-core
- sim-thermostat
- sim-gpu
- sim-mjcf
- sim-urdf
- sim-tests
- sim-ml-chassis
- sim-rl
- sim-opt
- sim-bevy (Layer 1)

**Approach:**
1. Run all 12 in sequence with logging enabled.
2. For any crate that fails to A, capture the failing criterion, triage as
   (a)/(b)/(c)/(d).
3. (a) fixes go inline on this branch.
4. (b) fixes get filed and either deferred or shipped in a follow-up commit.
5. Final pass: re-grade every crate that was touched, confirm green.

**Time:** ~12 crates × ~15min/crate (with logging) = ~3h sequential. Can be
parallelized partially via background bash if desired, but watch for
llvm-cov disk contention.

**Findings:**
_(none yet)_

---

## Audit close criteria

The audit is **closed** when:
- Every item above is either ☑ done or ⊘ skipped with logged rationale.
- Every (a)-category finding is committed to this branch.
- Every (b)-category finding has a tracking file (in-tree TODO or memory
  `project_*.md`) and a short timeline estimate.
- Every (c)-category finding has a memory `project_*.md` entry.
- This branch has been merged to main as one or more PRs (decide at close,
  not now).
- `project_platform_infrastructure_audit.md` in the memory dir has been
  updated with closing-state summary and moved to Completed Work in
  MEMORY.md.

After close, the next critical-path work is custom thermo-RL per
[`project_thermo_rl_loop_vision.md`](../.. memory pointer).

---

## Findings log (cross-item, for audit-wide patterns)

_(empty until execution begins)_
