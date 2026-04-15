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

2. **The grader is self-contradictory with the standard.** "A-grade or it
   doesn't ship" is enforced by `cargo xtask grade`, but PR #192 surfaced
   four concrete gaps in the grader itself:
   - `xtask/src/grade.rs::has_enclosing_allow` (line ~995) uses a loose
     300-line backward substring scan that can false-negative on panic
     justification.
   - `grade_clippy` (line ~660) accepts `//` but not `///` as preceding-line
     justification for `#[allow(clippy::...)]`.
   - Wall-clock assertions (`assert!(m.wall_time_ms < N)`) crash under
     llvm-cov instrumentation. Four were deleted from sim-opt during PR #192
     alone; more may exist elsewhere.
   - `grade_dependencies` (line ~1021) requires `#`-comment justification on
     each dep line, but sim-opt's Cargo.toml was silently incomplete until
     mid-PR. Other crates may be too because the grader has never graded
     them under their current dep set.

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

#### ☐ 1. Grader criterion correctness sweep

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
_(none yet)_

---

#### ☐ 2. Panic / unreachable span audit

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
_(none yet)_

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
(line ~1021). sim-opt was silently incomplete until mid-PR #192; others may
be too because the grader has never graded them under their current dep
sets.

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
