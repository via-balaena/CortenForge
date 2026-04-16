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

#### ☑ 3. Non-test `#[allow(clippy::...)]` justification audit

Every non-test `#[allow(clippy::...)]` under `sim/L0/*/src/` should have a
`//` (not `///`) preceding-line comment within 1–3 lines, OR a same-line
trailing `//` after the attribute. PR #192 added 8 such comments across
the chassis/rl split; others may still be missing in crates the grader
hasn't touched recently.

**Recon prompt sketch:** "Grep for `#[allow(clippy::` under `sim/L0/*/src/`
excluding `#[cfg(test)]` blocks and `tests/` directories. For each hit,
report whether the 1–3 preceding lines contain a `//` comment (not `///`,
not blank). Output as a punch list grouped by crate."

**Time:** ~15min recon + fixes are 1 line each.

**Findings:**

Recon strategy followed `feedback_ground_truth_via_tool.md`'s second-best
option (clippy/rustc don't enforce comment-justification — that's a
grader-local rule) by writing a Python script that mirrored
`grade_clippy`'s file-scan state machine verbatim. First pass returned
**171 UNJUSTIFIED hits** workspace-wide, but spot-checking revealed two
distinct false positives the grader was contributing to the count:

**Adjacent grader gap A — test-module-attribute stack (58 sites).**
`grade_clippy`'s `pending_test_attr → in_test` transition only fires when
it sees `{`, so an `#[allow(...)]` stacked between `#[cfg(test)]` and the
`mod tests {` opening is scanned as library code:

```rust
#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]   // ← scanned as lib
mod tests { ... }
```

This is the test-module house pattern across the repo. 58 sites across 7
of 8 Layer 0 crates (sim-types, sim-urdf, sim-mjcf, sim-thermostat,
sim-core, etc.).

**Adjacent grader gap B — multi-line attribute form (51 sites).**
`grade_clippy` only matched `#[allow(clippy::` as a single-line substring,
so the formatted form
```rust
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
)]
```
was silently ignored — neither flagged nor justification-checked. 51
occurrences workspace-wide, of which **25 were in library code** in the
just-refactored chassis/rl/opt crates (`sim-rl/cem.rs`, `ppo.rs`,
`reinforce.rs`, `sac.rs`, `td3.rs`; `sim-opt/algorithm.rs`,
`analysis.rs`, `parallel_tempering.rs`, `richer_sa.rs`;
`sim-ml-chassis/competition.rs`; etc.) — exactly the post-PR-#192
staleness this audit exists to catch. The other 26 were in test modules.

**Both gaps closed in commit 1 (grader fix).** `count_unjustified_clippy_allows`
extracted into a tested helper; the state machine now (a) skips attribute
lines when `pending_test_attr` is true, and (b) walks `#[allow(`
openings forward to the matching `)]` (single- or multi-line), checking
whether the joined span contains `clippy::`. 13 new unit tests cover the
interesting cases: same-line, multi-line, doc-comment-doesn't-justify,
test-mod-attr stack, multi-line test-mod-attr stack, multi-line
unjustified, multi-line justified, non-clippy allow ignored, after-test-
module still scanned, block comment skipped. Total grader unit tests:
10 → 23.

**Post-grader-fix true-positive count: 146 sites** (113 visible to the
old grader minus the 58 false-positive test-mod-attrs, plus the 25 newly-
visible chassis/rl/opt library multi-line sites + the 33 single-line
chassis/rl/opt allows the audit had missed pre-fix).

| Crate | Pre-fix REAL-LIB | Post-fix true total |
|---|---|---|
| sim-core | 83 | **104** |
| sim-mjcf | 21 | **22** |
| sim-gpu | 6 | **6** |
| sim-thermostat | 3 | **3** |
| sim-opt | (hidden) | **5** |
| sim-rl | (hidden) | **5** |
| sim-ml-chassis | (hidden) | **1** |
| sim-types, sim-urdf, sim-simd, sim-tests | 0 | **0** |
| **TOTAL** | 113 | **146** |

**Fixes shipped — per-crate `chore` commits:**

- **sim-ml-chassis (1 site).** `competition.rs::now_iso8601`: Hinnant
  civil-date single-letter symbols + epoch-arithmetic cast bounds.
- **sim-thermostat (3 sites).** `prf.rs`: ChaCha8 state-word truncation
  + PCG32-XSH-RR algorithm-defined output width + power-of-2 mantissa
  exactness for the uniform-`(0,1]` map.
- **sim-opt (5 sites).** `algorithm.rs`/`richer_sa.rs`/`parallel_tempering.rs`
  multi-line cast bundles for cooling/swap math; `analysis.rs`
  bootstrap-CI cast bundles (B = 10 000, slice lengths far below 2^52).
- **sim-rl (5 sites).** `cem.rs`/`ppo.rs`/`reinforce.rs`/`sac.rs`/`td3.rs`
  multi-line cast + `too_many_lines` + `panic` bundles on each
  `Algorithm::train`; comments lead with the inlined-pipeline
  readability rationale and the panic-as-internal-invariant contract.
- **sim-gpu (6 sites).** `pub_underscore_fields` on three WGSL-mirroring
  padding fields (must be `pub` for `bytemuck::Pod`/`Zeroable`);
  `too_many_lines` on the dispatch pipeline; `too_many_arguments` on the
  per-dispatch builder.
- **sim-mjcf (22 sites).** Lifted the `cast_possible_truncation,
  cast_possible_wrap` allow stack on `assemble_model`'s seven `nuser_*`
  computations to a function-level allow with a single justification
  (deleting six redundant attribute lines in the process); added
  upstream-table justifications to the three Gauss-Kronrod / Gauss-
  Legendre constant blocks; justified mesh-inertia
  `suspicious_operation_groupings` once with cross-references for the
  three call sites; bounds-check / branch-guard reasons for the parser
  bitmask casts; upstream-schema reasons for the two `struct_excessive_bools`.
- **sim-core (104 sites).** Largest sweep. Categories:
  - Collision-pair dispatchers (`pair_convex`, `pair_cylinder`, `plane`,
    `mesh_collide`, `narrow`, `sdf_collide`, `hfield`, `flex_self`,
    `flex_narrow`): full-per-geom-context reason.
  - Constraint solvers (`assembly`, `contact_assembly`, `equality`,
    `pgs`, `primal`, `qcqp`, `hessian`, `newton`, `cg`, `noslip`,
    `mod`): full-per-iteration-state reason + published-notation
    single-letter-name reason.
  - Dynamics (`crba`, `rne`, `spatial`): Featherstone-notation reason +
    inlined-as-single-function reason for the forward/backward passes.
  - Derivatives (`fd`, `hybrid`, `integration`): MuJoCo analytical-
    derivatives notation reason.
  - Casts (`island/sleep`, `forward/check`, `forward/muscle`, `validation`,
    `gjk_epa`, `model_init`, `constraint/mod`, `constraint/solver/cg`,
    `constraint/solver/pgs`): bounds-check / model-dimension-bounded /
    branch-guard reasons.
  - Indexed loops (`hessian`, `linalg`, `model_init`, multiple in
    `derivatives/hybrid`): parallel-array-update reason.
  - Math/notation (`linalg`, `mesh`, `forward/position`, `joint_visitor`,
    `dynamics/spatial`, `sensor/geom_distance`): published-notation +
    paired-identifier reasons.
  - One-offs: `clippy::panic` on `model_init::make_data` (plugin init
    failures are unrecoverable load errors, contract documented in
    `# Panics`); `island::mj_island` and `mj_flood_fill` inlined-pipeline
    + i32-storage-bounded reasons.

  Bulk-applied via a Python `insert.py` script (in `/tmp/item3-audit/`)
  that takes a `(file, line, comment)` TSV and inserts indentation-matched
  `// <comment>` lines bottom-up per file. The script kept the diff
  mechanical and made it possible to land 104 site-specific comments
  without 104 individual `Edit` round-trips.

**Recon-script reuse note.** The Python audit script
(`/tmp/item3-audit/audit-v2.py`) mirrors the new `count_unjustified_clippy_allows`
helper line-for-line. If items 4–5 want to do the same "is the grader
seeing the current state of crate X" check on dependencies / wall-clock
asserts, this is the second-time-confirmed pattern for those audits too.

**Verification:** `cargo clippy -p {sim-core,sim-mjcf,sim-gpu,sim-thermostat,sim-ml-chassis,sim-rl,sim-opt} --tests --all-features -- -D warnings`
clean. `cargo test -p xtask` 23 grader unit tests pass (was 10 pre-item-3).

**Dispositions:**

- Grader test-mod-attr fix → **(a)**, shipped in commit 1.
- Grader multi-line-form fix → **(a)**, shipped in commit 1 (revised
  from initial (b)-defer instinct after recon showed 25 of 51 multi-line
  sites were library code in chassis/rl/opt — the audit-coverage hole
  this audit exists to catch, not a hypothetical edge case).
- 146 site fixes → **(a)**, shipped in 7 per-crate commits (2-8).
- `count_unjustified_clippy_allows` helper extraction with 13 new unit
  tests → **(a)**, shipped in commit 1. Closes item 1's earlier (b)
  follow-up on adding `#[cfg(test)]` coverage to `grade_clippy`.
- Refactoring `assemble_model`'s nuser_* allow stack to a function-level
  allow with a single justification (sim-mjcf) → **(a)**, shipped inline
  with the per-crate sweep. Touches a load-bearing build pipeline
  function but the change is purely attribute hygiene.
- Trailing `//` comments inside multi-line allow bodies (e.g.,
  `sim-mjcf/builder/mesh.rs:181`'s `clippy::cast_precision_loss,   //
  nrow/ncol are small ints` form) — grader does NOT recognize these as
  justification (the inline-`//` check anchors on the opening line
  only). Could harden the helper to walk trailing `//` comments inside
  the body too → **(b)**, file as follow-up. Not shipping inline; the
  workaround (preceding `// reason:` line) lands the same information
  at the same review distance.
- Promoting the `audit-v2.py` mirror script to an xtask subcommand
  (`cargo xtask audit-clippy-allows`) → **(b)**, file as follow-up.
  Same disposition as item 2's `audit-panics` candidate; group with that
  if either becomes a recurring need.

---

#### ☑ 4. Dep justification coverage

Every `Cargo.toml` under `sim/L0/`, `sim/L1/`, and `examples/fundamentals/`
has each dep prefixed by a `#`-comment justification per
`grade_dependencies` (post-`bc119f08` line `xtask/src/grade.rs:1124–1247`).
sim-opt had unjustified deps caught mid-PR #192; other crates may too
because the grader hasn't been run against their current dep sets.

**Recon prompt sketch:** "For each `Cargo.toml` under `sim/L0/`, `sim/L1/`,
and `examples/fundamentals/` (skipping the workspace root), report any dep
line that lacks a `#`-comment justification on the line(s) immediately
above it. Use `xtask/src/grade.rs::grade_dependencies` (post-`bc119f08`
line 1124) as the spec for what counts as a justification."

**Time:** ~15min recon + 1-line fixes.

**Methodology intel from items 2 + 3** (this is the third item that
benefits from the same shape, so the pattern is now battle-tested):

1. **`grade_dependencies`'s exact helper logic** (mirror this verbatim
   in the recon script, no paraphrasing — see
   `feedback_ground_truth_via_tool.md`):
   - Walk `Cargo.toml` line by line.
   - Track three section headers as "in dep section": `[dependencies]`,
     `[dev-dependencies]`, `[build-dependencies]`. Other `[...]` headers
     exit the dep section.
   - Inside a dep section, skip blank lines, `#`-only lines, and lines
     without `=`.
   - A dep line is **justified** if (a) it contains an inline `#`
     comment, OR (b) one of the preceding 1–3 lines starts with `#`,
     stopping the backward scan at any blank line or `[` section header
     (the chain is "broken" by either).
   - Note: pure-TOML scan, no nesting/multi-line attribute handling.
     Simpler than `grade_clippy` — no state machine needed.

2. **Recon script.** Adapt `/tmp/item3-audit/audit-v2.py` to walk
   `Cargo.toml` files instead of `.rs` files. Mirror the rule above.
   Output a per-crate punch list grouped by `sim/L0/`, `sim/L1/`, and
   `examples/fundamentals/`. Don't reuse audit-v2.py wholesale — write
   `/tmp/item4-audit/audit.py` from scratch with the simpler rule.

3. **Crate scope.** Walk these `Cargo.toml` paths:
   - `sim/L0/{types,simd,core,thermostat,gpu,mjcf,urdf,tests,ml-chassis,rl,opt}/Cargo.toml`
   - `sim/L1/sim-bevy/Cargo.toml`
   - `examples/fundamentals/**/Cargo.toml` (find via glob)
   - **Skip the workspace root `Cargo.toml`** — it's not graded by
     `grade_dependencies` (it has no `[package]` section).

4. **Adjacent grader gaps to watch for.** Items 2 and 3 both surfaced
   grader bugs while running their narrow recon. For item 4, the
   things to check:
   - Does `grade_dependencies` correctly handle multi-line dep
     specifications (`name = { version = "...", features = [...] }`
     spanning lines)? Spot-check by looking at a known multi-line dep
     in any current `Cargo.toml`. If the grader scans the second line
     as a separate "dep" entry without `=`, fine. If it false-positives
     on the second line, that's an adjacent gap to fix inline.
   - Does it handle workspace-inherited deps (`name = { workspace = true }`)?
     The justification rule should still apply — verify the helper
     doesn't special-case these.
   - Are `[target.'cfg(...)'.dependencies]` sections covered? The
     current `in_dep_section` check uses exact-string equality on
     `[dependencies]`/`[dev-dependencies]`/`[build-dependencies]`, so
     target-conditional dep sections are silently skipped. If any Layer
     0 crate has them, that's an adjacent gap.

5. **Bulk-edit script.** Reuse `/tmp/item3-audit/insert.py` — it's
   indentation-aware and works on any text file. The TSV format is
   `<file>\t<line>\t<comment>` and inserts `<indent># <comment>` (TOML
   uses `#` for comments — adjust the script's hardcoded `// ` prefix
   to `# ` for item 4, or write a sibling `insert_toml.py`).

6. **Per-crate commit pattern.** `chore(<crate>): justify dependencies
   in Cargo.toml`. Pre-commit hook will run rustfmt on staged `.rs`
   files (none here) and clippy on changed crates. `Cargo.toml` edits
   don't trigger rustfmt — but **may trigger a `cargo check` rebuild**
   in the hook if it's wired to run on Cargo.toml changes. Verify
   nothing surprising trips the hook before staging.

7. **rustfmt watch-out (does not apply).** Item 3 hit a near-miss where
   rustfmt rewrapped a same-line `// justification` comment after a
   nearby insertion. TOML doesn't have this risk — there's no auto-
   formatter wrapping comment lines. Skip this concern for item 4.

8. **Adjacent grader-bug fix-or-defer rubric** (carried from item 3).
   If recon surfaces a `grade_dependencies` gap, default to **fix
   inline** unless: (a) the fix is genuinely scope creep (>200 LOC),
   (b) it requires adding new state machinery (TOML parser, etc.) that
   warrants its own PR. Item 3 set the precedent that "same recon,
   different gap" → fix inline.

**Findings:**

Recon ran the script-mirror pattern from items 2-3, this time against
`grade_dependencies:1169–1221` (line 1124 post-`bc119f08`, now extracted
as `count_unjustified_deps` post-this-item). Script:
`/tmp/item4-audit/audit.py` walks 188 `Cargo.toml` files (11 `sim/L0`,
1 `sim/L1`, 176 `examples/fundamentals`). A second script
`/tmp/item4-audit/gap_a_check.py` refined GAP-A detection from "any
unclosed `name = {` line" (133 files) to "unclosed lines whose
continuations actually contain `=`" (0 files) — the second pass is the
one that answers whether the grader is currently false-positiving.

**Path drift noted:** the intel block from `7b2c442b` referenced
`sim/L1/sim-bevy/Cargo.toml`; actual path is `sim/L1/bevy/Cargo.toml`.
Cosmetic — the recon script scans directories so no impact.

**Classification of the 790 total unjustified sites:**

| Bucket | Sites | Files | Notes |
|---|---|---|---|
| sim/L0 | 58 | 8 | `core` 12, `mjcf` 13, `urdf` 9, `gpu` 9, `tests` 6, `simd` 5, `types` 4. Clean: `thermostat`, `ml-chassis`, `rl`, `opt` (all justified in prior commits) |
| sim/L1 | 1 | 1 | `sim-bevy` L21 `sim-urdf` — the 4th dep under a `# Layer 0 simulation crates` header, outside the grader's 3-line backward-scan window |
| examples/fundamentals | 731 | 176 | 595 across `sim-cpu/` (151 crates) + 136 across `design/` + `mesh/` + `sim-ml/` (25 crates) |
| **TOTAL** | **790** | **185** | |

**Headline 1 — item 4's narrow question: 790 real sites.**
`grade_dependencies` hadn't been run against 7 of 11 Layer 0 crates or
any example crate since PR #192's chassis/rl split. Sim-opt-style
audit-coverage hole exactly like the one item 4 exists to sweep.

**Headline 2 — adjacent grader gaps: both latent, both closed inline.**
Recon surfaced two `grade_dependencies` holes with zero active impact
on this tree but both are real TOML forms the grader should handle:

- **GAP-A — multi-line inline-table dep specs.** The old line-by-line
  scanner processes each line independently. A legal TOML form like
  `rand = {\n    version = "0.8",\n    features = ["x"],\n}` would have
  its `version` and `features` continuation lines scanned as fresh dep
  entries (both contain `=`), triple-counting the one real dep.
  Zero active sites in tree (133 multi-line openings exist, all in the
  `bevy = { workspace = true, features = [` + quoted-feature-string +
  `] }` form, whose continuation lines have no `=` and were already
  being skipped by the old `if !trimmed.contains('=')` branch).

- **GAP-C — `[target.'cfg(...)'.dependencies]` sections.** The old
  section-header check used exact-string equality on
  `[dependencies]`/`[dev-dependencies]`/`[build-dependencies]`, so
  target-conditional dep sections were silently skipped — the grader
  would never see them, so any unjustified dep inside one would hold a
  false-pass. Zero target-cfg sections in tree today.

Both gaps closed by a brace-depth-aware `count_unjustified_deps` helper
(extracted from the inline `grade_dependencies` loop, matching item 3's
`count_unjustified_clippy_allows` precedent) plus an
`is_dep_section_header` helper that accepts `[target.<spec>.dependencies]`
and its dev-/build- variants, plus 21 new unit tests. Grader test count
23 → 44. Commit `4623d7bd`.

**Sanity checks of other forms:**

- **GAP-B — workspace-inherited deps.** `name = { workspace = true }`
  is the common form (810 occurrences workspace-wide). All single-line,
  so the existing rule applies cleanly. Non-gap.
- Comment-only lines, blank-line chain-breaking, section-header
  chain-breaking, and `[patch.crates-io]`-style non-dep sections all
  behave correctly under both old and new helper. Regression-tested.

**Fixes shipped — per-crate `chore` commits:**

| Crate / Bucket | Sites | Commit |
|---|---|---|
| Grader hardening (GAP-A + GAP-C + 21 unit tests) | — | `4623d7bd` |
| sim-types | 4 | `d86df059` |
| sim-simd | 5 | `94ba4416` |
| sim-core | 12 | `049fdabb` |
| sim-gpu | 9 | `0b5e1bd3` |
| sim-mjcf | 13 | `aca8ec59` |
| sim-urdf | 9 | `989c8080` |
| sim-conformance-tests | 6 | `ae02602e` |
| sim-bevy | 1 (grader-flagged) + restructured all deps for consistency | `97c9dd44` |
| examples/fundamentals/sim-cpu/** (151 crates) | 595 | `ee281546` |
| examples/fundamentals/{design,mesh,sim-ml}/** (25 crates) | 136 | `bd463940` |
| **Total site fixes** | **790** | 10 per-bucket commits |

**Bulk-edit tooling.** sim/L0 and sim/L1 crates edited by hand with
`Edit` — low volume (1-13 sites each), high per-dep nuance (domain-
specific justification). Example-crate sites swept by a
`/tmp/item4-audit/fix_examples.py` script built around a static
dep-name → justification map (20 unique names covered the full 731-site
space). The script reads each `Cargo.toml`, runs `grade_dependencies_mirror`
to get the unjustified line list, and inserts `<indent># <comment>`
lines bottom-up per file so earlier inserts don't shift later line
numbers — the same pattern validated in item 3 but with `# ` comment
prefix for TOML instead of `// ` for Rust.

**Recon-script reuse note.** `/tmp/item4-audit/audit.py` plus the
static justification map in `fix_examples.py` generalize to any
future dep-justification sweep. If item 5 (wall-clock assertions) uses
a different helper-logic mirror, the repository-walking scaffolding can
be cloned.

**sim-bevy restructuring.** sim-bevy's pre-existing section-style
comments (`# Shared geometry kernel`, `# Layer 0 simulation crates`,
`# Math`, `# Bevy 0.18 - ...`) were replaced with per-dep comments so
every dep falls inside the grader's 3-line backward-scan window. Pre-
restructure, only `sim-urdf` (the 4th dep after `# Layer 0 simulation
crates`) was flagged; the other three benefited incidentally from
falling within 1-3 lines of the header. The new form is consistent with
every other L0/L1 crate and doesn't rely on positional proximity to a
section-level comment.

**Verification:**
- `cargo test -p xtask`: 44 passed, 0 failed (was 23 pre-item-4).
- `cargo clippy -p xtask --tests --all-features -- -D warnings`: clean.
- `/tmp/item4-audit/audit.py` post-item-4: **0 unjustified sites**
  across all 188 in-scope Cargo.toml files.
- TOML-comment-only edits cannot break crate compilation; no per-crate
  `cargo check` run.
- Full `cargo xtask grade` sweep deferred to item 10 per `feedback_xtask_grade_opacity.md`.

**Dispositions:**

- GAP-A fix → **(a)**, shipped in commit `4623d7bd`. No active sites
  today, but the form is legal TOML and a known grader hole — fix
  inline per item 3's precedent ("same recon, different gap") and the
  user's explicit direction not to defer.
- GAP-C fix → **(a)**, shipped in commit `4623d7bd`. Same rationale as
  GAP-A.
- `count_unjustified_deps` helper extraction + 21 new unit tests →
  **(a)**, shipped in commit `4623d7bd`. Matches item 3's
  `count_unjustified_clippy_allows` extraction pattern — grader
  continues accumulating `#[cfg(test)]` coverage as state machines are
  touched.
- 790 site fixes → **(a)**, shipped in 10 per-bucket commits.
- Promoting `audit.py` + `fix_examples.py` (and items 2-3's analogous
  scripts) to `cargo xtask audit-*` subcommands → **(d)**, decline.
  The scripts are one-shot recon aids that answer "is the grader
  seeing the current state of crate X?" for a specific audit item.
  Once this audit closes and item 10 runs `cargo xtask grade` on every
  crate, the audit-script layer has no remaining purpose — the grader
  itself is the steady-state check. Promoting them to xtask would
  create a second layer of the same analysis that needs to be
  maintained in lockstep with every `grade_*` helper change. Keeping
  them at `/tmp/` is correct: they're session scratch for audit
  sessions, not durable tooling.
- Intel-block path drift (`sim/L1/sim-bevy/Cargo.toml` →
  `sim/L1/bevy/Cargo.toml`) → **(a)**, doc-only, correct inline on
  close.
- Pre-commit hook pathspec skipping clippy on Cargo.toml-only changes
  (observed on every site-fix commit: "→ No Rust/Cargo files staged —
  skipping clippy") → **(a)**, shipped as `82bcfec3`. The hook's
  `'Cargo.toml'` pathspec without wildcards was anchored to the tree
  root, so every nested `sim/L0/**/Cargo.toml` /
  `examples/**/Cargo.toml` was silently skipped. Fixed in both sources
  of truth (`xtask/build.rs` PRE_COMMIT_HOOK + `xtask/src/setup.rs`
  hook literal) by switching to `'*Cargo.toml'`. Rebuilding xtask
  auto-reinstalls the fixed hook via `build.rs::install_hook_if_needed`.

---

### Tier B — anti-pattern sweeps (fast, repo-wide grep)

#### ☑ 5. Wall-clock assertion sweep

Grep for `assert!(...)` patterns matching `wall_time_ms`, `elapsed`,
`Duration::as_*`, `Instant::elapsed`, etc., across all workspace test files.
Four were deleted from sim-opt during PR #192:
- `cem.rs:329` (commit 3)
- `algorithm.rs:420` (commit 5)
- `richer_sa.rs:543` (commit 5)
- `parallel_tempering.rs:552` (commit 5)

These crash under llvm-cov's ~10× instrumentation overhead and are a flaky
anti-pattern even outside grading. Survey for stragglers.

**Recon methodology — DEPARTURE from items 2-4's script-mirror pattern.**
Item 5 is a test-side anti-pattern sweep, not a grader-discipline audit. There's
no `grade_*` helper to mirror — wall-clock asserts are flaky-under-instrumentation
because they encode an assumption the grader's coverage criterion can't satisfy,
not because the grader is missing a check. Direct grep is the authoritative
tool for this question. Workspace-wide multi-line `assert!` regex over `*.rs`
files covering `elapsed`, `wall_time`, `speedup`, `steps_per_sec(_ond)`, `ratio`,
`secs_f`, `as_millis`, `as_micros`, `as_nanos`. Cross-referenced the 21 files
containing `.elapsed()` to separate test-side gates from production timing
emission and from `eprintln!`-only diagnostics. Multi-line `if elapsed > X
{ panic! }` pattern: zero hits — only the `assert!`-form exists in this tree.

**Classification of timing-derived assertions:**

| Verdict | Count | Meaning |
|---|---|---|
| HARD-CRASH active — pure throughput gate | 8 | `assert!(steps_per_sec >= N)` where N is an absolute threshold; throughput drops ~10× under llvm-cov, crash guaranteed; on default grader path |
| HARD-CRASH but `#[ignore]`'d | 3 | Same anti-pattern, gated behind `#[ignore = "benchmark"]`; not on default grader path but still anti-pattern by the same logic. Deleted under sharpen-the-axe. |
| BORDERLINE — same-test ratio gate, kept and flagged for item 10 | 2 | Both halves run in same test under same instrumentation; ratio bounds are generous: `runtime_flags.rs:2820` AC32 `ratio < 0.5` (BVH < 50% brute), `collision_performance.rs:534` scaling `ratio < 10.0` (5×→10× bodies). If item 10's grade run crashes here, delete then. |
| DIAGNOSTIC ONLY (kept) | many | `eprintln!`/`println!` of elapsed; production `wall_time_ms` field emission in rl/opt training loops; `print_report` Check entries with discarded result in non-test example mains; `profile_steps`/`profile_split_steps` whose returned timings feed only `eprintln!`s; `timed`/`benchmark_steps_per_second` helpers with zero callers (dead code, left as-is). |

**Recon-fidelity note.** Initial classification put `derivatives.rs:test_pos_deriv_performance` in the BORDERLINE bucket; spot-check during verification revealed it's already `#[ignore]`'d (`#[ignore = "benchmark — run with --release --ignored for meaningful timing"]` at line 1965), making it a HARD-CRASH-#[ignore]'d site, not a borderline-active. The deletion decision stands (sharpen-the-axe deletes #[ignore]'d sites of the same shape), but the better-than-expected outcome is that **zero borderline asserts were touched** — the two ratio asserts that survive (AC32 mesh midphase, scaling_collision_bodies) are explicitly kept and flagged for item 10. Methodological fix: future recon should `grep -B 5` for `#[ignore]` immediately above any candidate site, not infer it from line context.

**Headline — 11 deletions across 4 files:**

| File | Function | Line | Form | Status |
|---|---|---|---|---|
| `sim/L0/tests/integration/collision_performance.rs` | `perf_simple_pendulum` | 121 | `steps_per_sec >= threshold` | DELETED |
| same | `perf_double_pendulum` | 161 | `steps_per_sec >= threshold` | DELETED |
| same | `perf_ball_stack` | 211 | `steps_per_sec >= threshold` | DELETED |
| same | `perf_falling_boxes` (`#[ignore]`) | 265 | `steps_per_sec >= threshold` | DELETED |
| same | `perf_capsule_pile` | 310 | `steps_per_sec >= threshold` | DELETED |
| same | `perf_humanoid_simplified` | 392 | `steps_per_sec >= threshold` | DELETED |
| same | `perf_robot_arm` | 451 | `steps_per_sec >= threshold` | DELETED |
| `sim/L0/tests/integration/validation.rs` | `test_performance_humanoid` | 1686 | `steps_per_second > min_threshold` | DELETED |
| same | `test_performance_simple_pendulum` | 1756 | `steps_per_second > 3_000.0` | DELETED |
| `sim/L0/tests/integration/derivatives.rs` | `test_pos_deriv_performance` (`#[ignore]`) | 2011 | `speedup >= 1.5` | DELETED |
| `design/cf-design/src/solid.rs` | `mesh_to_tolerance_perf_50mm_sphere` (`#[ignore]`) | 3564 | `elapsed < 5.0` | DELETED |

**Cleanup of orphaned dead code (collision_performance.rs):** removed
`SIMPLE_DEBUG_THRESHOLD`, `SIMPLE_RELEASE_THRESHOLD`, `CONTACT_DEBUG_THRESHOLD`,
`CONTACT_RELEASE_THRESHOLD`, `COMPLEX_DEBUG_THRESHOLD`, `COMPLEX_RELEASE_THRESHOLD`
constants and the `get_threshold` helper function — orphaned by the assert
deletions. Module-level docstring rewritten from "establishes hard performance
thresholds" to "throughput observation and scaling validation". Net diff:
−136 lines, +9 lines.

**Cleanup of orphaned dead code (validation.rs):** in `test_performance_humanoid`
removed the `#[cfg(debug_assertions)]` / `#[cfg(not(debug_assertions))]`
`min_threshold` + `is_ci` machinery, the multi-line threshold-strategy comment,
and the `else if is_ci { ... }` branch of the spec-tracking output. Same in
`test_performance_simple_pendulum`. Spec-tracking `MEETS SPEC` / `BELOW SPEC`
print preserved (simplified to `if > 10_000 / else`). Throughput `println!`
preserved. Net diff: −49 lines, +2 lines.

**Verification:** `cargo test -p sim-conformance-tests --release` and
`cargo test -p cf-design --release` (per-crate per `feedback_xtask_grade_opacity.md`,
not `cargo xtask grade`). Both pass post-fix. Full `cargo xtask grade` sweep
deferred to item 10.

**Dispositions:**

- 8 hard-crash active deletes → **(a)** fix-inline.
- 3 `#[ignore]`'d HARD sites (collision_performance.rs `perf_falling_boxes`,
  derivatives.rs `test_pos_deriv_performance`, cf-design solid.rs
  `mesh_to_tolerance_perf_50mm_sphere`) → **(a)** fix-inline. Same anti-pattern;
  sharpen-the-axe closes them inline rather than waiting for someone to run
  `--ignored` and hit the same crash. Throughput is still observable via the
  preserved `eprintln!`/`println!` lines in each.
- `runtime_flags.rs:2820` AC32 `ratio < 0.5` → **kept**, flagged for item 10
  verification. Both halves run in the same test under the same instrumentation
  overhead, only 5 iterations per path; mesh-mesh contact code is symmetric
  across both paths; bound is generous. Delete only if item 10's grade run
  crashes here.
- `collision_performance.rs:534` scaling `ratio < 10.0` → **kept**, flagged
  for item 10. Same-test ratio with very generous bound (O(n³) Cholesky already
  eats most of the headroom; instrumentation drift on top is unlikely to push
  past 10×).
- Orphaned `THRESHOLD` constants and `get_threshold` helper cleanup →
  **(a)** fix-inline. Per CLAUDE.md ("avoid backwards-compat hacks like
  renaming unused `_var`"), the right move when assertions become dead is
  to delete the dead code, not silence the warnings.
- `min_threshold` / `is_ci` cleanup in `validation.rs` → **(a)** fix-inline.
  Same rationale.

**Methodology takeaway:** item 5 confirms the items 2-4 script-mirror pattern
is not the only audit shape that fits this tier. When the grader doesn't have
a helper to mirror — when the question is "does any test contain anti-pattern
X?" rather than "does the grader correctly check rule Y?" — direct grep with
careful triage is the right tool. The `assert!`-form-only result (zero
panic-in-if straggers) confirms the audit's narrow scope was correctly drawn.

---

#### ☑ 6. Doc + post-split hygiene

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

Recon used text-and-judgment direct grep — DEPARTURE from items 2-4's
script-mirror pattern and from item 5's direct-grep-for-anti-patterns.
Item 6 is the third methodology shape in this audit: workspace-wide grep
to find candidates, then generous context reads (≥10 lines) to classify
each hit as historical-record vs current-state-claim. No Python mirror —
the triage decision ("is this describing what the split did, or implying
the crate still exists?") requires human/AI judgment, not a state machine.

Three sub-targets scanned in parallel:

**(A) sim_ml_bridge / sim-ml-bridge references** — workspace-wide grep
excluding `archive/`. 24 hits across 10 files. Classification:

| Classification | Count | Files |
|---|---|---|
| Historical — audit doc, migration comment, commit quote, construction-spec chapter | 5 files | `platform_infrastructure_audit.md`, `sim-rl/lib.rs` (migration guide), `sim-opt/tests/d2c_sr_rematch.rs` (commit quote), `docs/studies/ml_chassis_refactor/src/42-pr-3-sim-opt-rematch.md` (frozen chapter with pre-squash tag audit trail — same ruling as `docs/thermo_computing/`), `ML_COMPETITION_SPEC.md` (already annotated as pre-split historical record) |
| **Current-state claim — stale** | **4 files** | `docs/SKELETON.md` (tree map + section), `SIM_ML_EXAMPLES_SPEC.md`, `PHASE_4_SPEC.md`, `COMPETITION_TESTS_SPEC.md` |

**(B) lib.rs module-doc drift** — read module-level docs across all 11
sim/L0 crates + sim/L1/bevy. Every named symbol verified against the
current tree. One stale finding:

- `sim/L0/thermostat/src/lib.rs` line 65: link to
  `docs/thermo_computing/03_phases/01_langevin_thermostat.md` — the
  `03_phases/` directory was deleted in the 2026-04-11 repo audit
  (647→344 md files). The companion link at line 63
  (`02_foundations/chassis_design.md`) is valid. All other 11 crates
  clean.

**(C) bevy feature flags** — grep for `feature = "bevy"`,
`cfg(feature = "bevy")`, `cfg_attr.*feature.*bevy` workspace-wide, plus
`[features]` section scan in all `Cargo.toml` files under `sim/` and
`examples/`. **Zero issues.** `sim-ml-chassis` is the only crate
declaring a `bevy` feature; it's properly used in 3 `cfg_attr` sites
(`env.rs` SimEnv, `space.rs` ObservationSpace + ActionSpace — all
`derive(bevy_ecs::prelude::Resource)`) and correctly consumed by 7
example crates. No broken (used-not-declared) or dead (declared-not-used)
patterns anywhere.

**Fixes shipped:**

- **`docs/SKELETON.md`** — tree map: `ml-bridge/` → `ml-chassis/` +
  `rl/` + `opt/` (3 lines). Domain status section: rewrote the
  `sim/L0/ml-bridge — ML/RL Bridge` block to `sim/L0/ml-chassis + rl +
  opt — ML Stack (3-crate split)` with per-crate descriptions. Deleted
  stale branch reference.

- **`sim/L0/thermostat/src/lib.rs`** — deleted stale `See also` link to
  non-existent `03_phases/01_langevin_thermostat.md`. Verified via
  `cargo doc -p sim-thermostat --no-deps`: zero warnings.

- **Spec file deletions** — deleted 3 completed specs with stale
  `sim-ml-bridge` current-state claims:
  - `examples/fundamentals/sim-ml/SIM_ML_EXAMPLES_SPEC.md` (Phases 1-4
    complete, 5-8 specified but initiative CLOSED and crate name wrong)
  - `examples/fundamentals/sim-ml/PHASE_4_SPEC.md` ("Status: Complete,
    all 6 steps done")
  - `examples/fundamentals/sim-ml/COMPETITION_TESTS_SPEC.md` (Phases 3,
    6, 6b complete; 6c planned but initiative CLOSED)

  Per `feedback_code_speaks.md`: the code is the spec for completed work.
  Memory file already notes "Phase 5-8 example spec still valid as
  followup" as the durable pointer if that work revives — a fresh spec
  reflecting the 3-crate structure would be needed anyway.
  `ML_COMPETITION_SPEC.md` kept — already has pre-split historical-record
  annotation and contains R34 GT-R vision framing (architecture rationale,
  a "keep" category per code-speaks).

**Dispositions:**

- SKELETON.md update → **(a)**, shipped inline.
- Thermostat rustdoc stale link → **(a)**, shipped inline.
- 3 spec deletions → **(a)**, shipped inline (per code-speaks).
- 5 historical-record files → **keep**, no action.
- Sub-target C (bevy feature flags) → **clean**, no action.

---

### Tier C — broader cleanup

#### ☑ 7. Test organization audit

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

Recon methodology DEPARTURE from items 2–6: pure reading audit — read
each integration test file end-to-end, catalog its `use` imports, identify
what crate's symbols it exercises, cross-reference against inline
`#[cfg(test)]` modules in all three crates (22 in ml-chassis, 5 in rl,
4 in opt). No grep sweep, no script mirror. The question is structural
("is this test in the right crate?"), not pattern-based.

**Per-file classification:**

| File | Crate | Cross-crate imports | Placement | Duplication |
|------|-------|---------------------|-----------|-------------|
| `rl/tests/autograd_policy_reinforce_integration.rs` (54 lines, 1 test) | sim-rl | chassis (`AutogradPolicy`) + rl (`Reinforce`) | CORRECT | None — chassis inline tests cover forward/gradient parity via FD, not training convergence |
| `rl/tests/best_tracker_cem_integration.rs` (207 lines, 5 tests) | sim-rl | chassis (`BestTracker`, `TrainingCheckpoint`) + rl (`Cem`) | CORRECT | None — chassis inline tests cover pure `BestTracker` unit logic (8 tests) |
| `rl/tests/custom_task.rs` (110 lines, 1 test) | sim-rl | chassis (`TaskConfig::builder()`) + rl (`Cem`) + `sim_mjcf` | CORRECT | None |
| `rl/tests/competition.rs` (1857 lines, 13 `#[ignore]` tests) | sim-rl | chassis (`Competition`, all policy/value types) + rl (all 5 algorithms) | CORRECT | None — chassis inline uses `MockAlgorithm` for API mechanics (28 unit tests); integration tests run real scientific experiments |
| `opt/tests/d2c_sr_rematch.rs` (342 lines, 1 `#[ignore]` test) | sim-opt | opt (`Sa`, `run_rematch`) + rl (re-exported chassis types + `Cem`) + core + thermostat | CORRECT | None |
| `opt/tests/d2c_sr_rematch_richer_sa.rs` (334 lines, 1 smoke + 1 `#[ignore]`) | sim-opt | opt (`RicherSa`, `run_rematch`) + rl + core + thermostat | CORRECT | Intentional within-crate MJCF/param duplication per Ch 42 §6(f) |
| `opt/tests/d2c_sr_rematch_pt.rs` (313 lines, 1 smoke + 1 `#[ignore]`) | sim-opt | opt (`Pt`, `run_rematch`) + rl + core + thermostat | CORRECT | Same intentional duplication |

**Structural findings:**

1. **ml-chassis/tests/ does not exist** — correct. Chassis can't have
   integration tests using rl algorithms without a `sim-ml-chassis →
   sim-rl` dev-dep cycle. The 22 inline `#[cfg(test)]` modules provide
   complete chassis-level unit coverage.

2. **All 4 sim-rl integration tests cross the chassis/rl boundary** —
   every file imports from both `sim_ml_chassis` and `sim_rl`. The two
   relocated files (`autograd_policy_reinforce_integration.rs` and
   `best_tracker_cem_integration.rs`) have module docstrings documenting
   why they moved during the split.

3. **All 3 sim-opt integration tests correctly placed** — they test
   sim-opt's SA/RicherSa/PT algorithms, using sim-rl as a dev-dep for
   the CEM control arm and chassis re-exports.

4. **No orphaned fixtures, test data, or helper modules** — no
   `testdata/`, `fixtures/`, or loose files in any test directory.

5. **No duplicate coverage** — inline `#[cfg(test)]` modules and
   integration tests operate at different layers. Chassis inline tests
   use mocks/FD checks; integration tests use real algorithms through
   real physics. Zero overlap.

6. **Within-crate MJCF/parameter duplication in sim-opt** (~100 lines
   across three rematch fixtures) — explicitly documented and declined
   per Ch 42 §6(f): frozen task, self-contained fixtures, readability
   preference. **(d) decline.**

**Verdict:** Clean bill of health. No misplaced tests, no duplicate
coverage, no orphaned fixtures, no gaps. The three-crate split landed
tests in the right places. **Zero code changes; item closes on doc
update alone.**

---

#### ☑ 8. Memory system hygiene

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

Recon methodology DEPARTURE from all prior items: this is the one audit
item that modifies the memory directory itself, not the repo tree. Recon
is a cross-reference exercise — read every `.md` file in the memory dir,
cross-reference against MEMORY.md's index, classify each file as
keep/delete per the code-speaks rule and the memory system's own "what
NOT to save" guidelines.

**Inventory:** 65 files on disk (34 `project_*`, 27 `feedback_*`, 2
`user_*`, 1 `reference_*`, 1 `MEMORY.md`). MEMORY.md at 91 lines (200
budget). Every file read end-to-end. Cross-reference against the 34-file
`project_*.md` disk listing identified:

**Check 1 — Broken links in MEMORY.md:** zero. All linked files exist.

**Check 2 — MEMORY.md line count:** 91 → 87 post-cleanup. Healthy.

**Check 3 — Orphaned files** (on disk, no MEMORY.md entry): **21 files**
(15 `project_*`, 5 `feedback_*`, 1 `user_*`). Of the 15 orphaned
project files: 14 were completed initiatives or session-prep docs whose
content is derivable from code/git history — deleted. 1
(`project_thermo_rl_bridge_architecture.md`) carries non-obvious "why"
context about the ctrl-channel architectural decision — kept and indexed.
All 5 orphaned feedback files carry valid, non-redundant guidance — kept
and indexed. The orphaned user file (`user_ml_bridge_inspiration.md`)
captures timeless user motivation — kept and indexed.

**Check 4 — Active Initiatives for completed work:** 9 entries in the
Active Initiatives section pointed at files for shipped/closed
initiatives:
- `project_repo_audit.md` — COMPLETE 2026-04-11
- `project_sim_bevy_api_spec.md` — All 6 changes COMPLETE
- `project_sim_ml_pivot.md` — FULLY SHIPPED
- `project_ml_chassis_refactor_study.md` — COMPLETE at study level
- `project_sim_ml_split.md` — Shipped
- `project_sim_ml_renovation.md` — CLOSED 2026-04-15
- `project_grade_tool_audit.md` — COMPLETE 2026-04-10
- `project_d2_sr_findings.md` — D2 COMPLETE
- `project_sensor_examples.md` — 9 examples COMPLETE

All 9 files deleted (content derivable from code/docs/git history per
code-speaks). Their key conclusions already appear inline in MEMORY.md's
Completed Work section. Active Initiatives trimmed from 13 to 4 entries.

**Check 5 — Resolved Blockers section:** all 3 files
(`project_connect_constraint_bug.md`, `project_sdf_sdf_stacking_blocker.md`,
`project_convex_plane_dispatch.md`) deleted — fixes are in the code,
commit messages have context. Section removed from MEMORY.md.

**Check 6 — Duplicate coverage:** the ML cluster (11 files across
`project_sim_ml_pivot`, `project_sim_ml_split`,
`project_sim_ml_renovation`, `project_ml_chassis_refactor_study`,
`project_autograd_engine`, `project_best_policy_tracking`,
`project_competition_findings`, `project_policy_persistence`,
`project_policy_visualization`, `project_reinforce_findings`,
`project_ml_bridge_bench_revisit`) all described the same completed
initiative from different angles. All 11 deleted. The crate split's
architecture decisions live in code rustdoc; the study's execution
record lives in the mdbook; the key conclusions live inline in
MEMORY.md's Completed Work entries.

**Check 7 — Feedback file review:** all 27 `feedback_*.md` files
reviewed. None redundant with another or with default behavior. All
kept. 5 previously orphaned files added to MEMORY.md's User Preferences
index.

**Net result:**

| Metric | Before | After |
|---|---|---|
| Total files | 65 | 39 |
| `project_*.md` | 34 | 8 |
| `feedback_*.md` | 27 | 27 |
| `user_*.md` | 2 | 2 |
| `reference_*.md` | 1 | 1 |
| MEMORY.md lines | 91 | 87 |
| Orphaned files | 21 | 0 |
| Broken links | 0 | 0 |
| Active Initiatives entries | 13 | 4 |
| Tokens freed (estimate) | — | ~60K+ (`project_ml_chassis_refactor_study.md` alone was 41K) |

The single largest win is deleting `project_ml_chassis_refactor_study.md`
(41K tokens) — a session-to-session handoff document that loaded into
every conversation context but whose content is preserved in the mdbook
and in MEMORY.md's inline Completed Work entries.

**Dispositions:**

- 26 file deletions → **(a)**, fix-inline (memory dir, not repo tree).
- 7 orphaned file index additions → **(a)**, fix-inline (MEMORY.md).
- MEMORY.md structural rewrite (remove Resolved Blockers section, trim
  Active Initiatives from 13 to 4, add 5 feedback + 1 user + 1 project
  entries to their respective sections) → **(a)**, fix-inline.
- Feedback files — all 27 kept, none deleted. Conservative per audit
  methodology note 6.
- Vision/philosophy files — all kept per audit methodology note 7.

---

### Tier D — capstone

#### ☑ 9. xtask progress logging

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

Recon methodology DEPARTURE from items 2–8: this is the first item that
writes new Rust code in xtask. Items 1–4 modified existing `grade_*`
helpers; item 9 adds logging infrastructure around the grade pipeline.
Recon was a code read of `grade.rs` identifying every subprocess
invocation, the criterion sequencing flow, existing output style, and
the two-pass coverage structure. No script mirror, no grep sweep.

**Subprocess inventory (9 calls):** `cargo locate-project` (instant),
`cargo metadata` ×2 (fast, ~2s), `cargo llvm-cov --version` (instant),
**`cargo llvm-cov --json --release --lib`** (~5-10 min, longest stage),
**`cargo test --release`** (~1-5 min, second longest), `cargo doc`
(seconds), `cargo clippy` (seconds), `cargo tree` (instant).

**Existing output:** two bare `println!` lines in `grade_coverage`
(`"    Pass 1: coverage (unit tests only)..."` and
`"    Pass 2: all tests (no instrumentation)..."`). No timing, no
criterion-level progress, no structured output. `owo_colors` for the
final Unicode table. `eprintln!` used only once (test-failure warning).

**Implementation — minimum bar + all three stretch goals shipped:**

All progress logging goes to stderr; the final report (table or JSON)
goes to stdout. This separation means `--json | jq .` works cleanly.

1. **`Verbosity` struct** (`quiet`, `verbose`, `json`) threaded through
   `run()` → `evaluate()` → `grade_coverage()`.

2. **`run_criterion` helper** — wraps each criterion with
   `"  criterion N/7: <name> — running…"` before and
   `"  criterion N/7: <name> — <grade> (<elapsed>s)"` after, both on
   stderr. Suppressed by `--quiet`.

3. **`evaluate()` overall timing** — prints
   `"  grading <crate> (profile: <label>)…"` at start and
   `"  grading complete — <elapsed>s total"` at end, on stderr.

4. **`grade_coverage()` pass messages** — enhanced with time estimates
   (`"pass 1/2: cargo llvm-cov --lib --release (~5-10 min for
   instrumented test runs)"`) and per-pass elapsed timing
   (`"pass 1/2: done (342.1s)"`), both on stderr. Subprocess stdout
   captured via `.output()` in `--json`/`--quiet` modes to keep stdout
   clean.

5. **`Heartbeat` struct** (~30 lines) — background thread printing
   `"    … still running (Ns elapsed)"` every 30s on stderr. Stops
   automatically on `Drop`. Activated by `--verbose` during the two
   coverage passes. Polls the stop flag every 1s so the thread exits
   within 1s of the subprocess completing.

6. **CLI flags** — replaced the unused `--format` (was `_format: &str`)
   with three clap flags: `--quiet` (suppress progress, conflicts with
   `--verbose`), `--verbose` (enable heartbeat, conflicts with
   `--quiet`), `--json` (structured JSON on stdout). `--quiet --verbose`
   gives a clean clap error.

7. **`json_output()`** (~20 lines) — builds JSON via `serde_json::json!`
   (already a dep) and prints to stdout. Called in `run()` instead of
   `display()` when `--json`.

8. **`complete.rs` call site** — updated `evaluate()` call to pass
   default `Verbosity` (non-quiet, non-verbose, non-json).

**Diff shape:** 3 files, +257/−55 lines. All additive output
infrastructure — zero changes to grading logic, zero changes to any
`grade_*` helper, zero changes to pass/fail thresholds.

**Verification:**

- `cargo test -p xtask`: 44 passed, 0 failed (unchanged from pre-item-9).
- `cargo clippy -p xtask --tests --all-features -- -D warnings`: clean.
- `cargo xtask grade sim-types`: progress output readable, grade
  unchanged (A). Sample output:
  ```
    grading sim-types (profile: Layer 0 library)…

    criterion 1/7: Coverage — running…
      pass 1/2: cargo llvm-cov --lib --release (~5-10 min for instrumented test runs)
      pass 1/2: done (6.6s)
      pass 2/2: cargo test --release (~1-5 min depending on integration tests)
      pass 2/2: done (5.4s)
    criterion 1/7: Coverage — A (12.1s)
    criterion 2/7: Documentation — A (0.9s)
    criterion 3/7: Clippy — A (0.4s)
    criterion 4/7: Safety — A (0.0s)
    criterion 5/7: Dependencies — A (0.1s)
    criterion 6/7: Bevy-free — A (0.1s)

    grading complete — 13.8s total

  ╔══════════════════════════════════════════════════════════════╗
  ║                      GRADING: sim-types                      ║
  ...
  ```
- `cargo xtask grade sim-types --json 2>/dev/null | python3 -m json.tool`:
  parses clean.
- `cargo xtask grade sim-types --quiet`: progress suppressed, table only.
- `cargo xtask grade sim-types --quiet --verbose`: clap error
  (`the argument '--quiet' cannot be used with '--verbose'`).

**Dispositions:**

- Minimum bar (criterion progress + pass messages + elapsed timing) →
  **(a)**, shipped in `a9fc7c45`.
- Heartbeat (stretch goal, optional in spec) → **(a)**, shipped inline.
  ~30 lines, self-contained, directly relevant to item 10's 12-crate
  sweep.
- `--quiet` / `--verbose` toggles (stretch goal) → **(a)**, shipped
  inline. Clean three-tier design (quiet/normal/verbose). `--quiet` also
  suppresses subprocess stdout for `cargo test` pass via `.output()`
  capture.
- JSON output mode (stretch goal) → **(a)**, shipped inline. Uses
  existing `serde_json` dep, no new dependencies. stdout/stderr
  separation makes it pipe-friendly.
- The unused `--format` flag (was `_format: &str`, default `"pretty"`,
  never read) → deleted, replaced by `--json`. Breaking change for
  anyone typing `--format pretty` (nobody — the parameter was literally
  unused).

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
