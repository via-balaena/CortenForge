# PR #246 bookmark — pivot to cf-scan-prep v1.0 completion

**Date**: 2026-05-15 EOD
**Branch**: `dev` at `6ebc60db`
**Status**: PR **NOT** opened. Work paused mid-arc per user decision.

## What triggered the pause

The PR #246 plan (slice 9.6 plug-inset + 9.5 Slacker recipe + centerline-trim
in cf-scan-prep + 9.7 progress logging + 9.8 simplify-on-save) was 5 sub-commits
in when the user surfaced the load-bearing problem:

> "the cf-scan prep is not finished, and the downstream consumers of this mesh
>  are all suffering the consequences. we need to pause here, make a bulletproof
>  bookmark of where we are at. then, we need to go back into completing
>  cf-scan-prep v1.0 because it isnt finished yet"

The trigger was discovering that the Simplify panel slider was viewport-only
— users could click Save with the slider at 200k and still get a 3.34M-face
`.cleaned.stl` out. 9.8 fixed that specific instance, but the underlying
diagnosis is broader: **cf-scan-prep v1.0 is not actually v1.0**. The cleaned
scan it emits is not workshop-ready for cf-device-design + cf-cast-cli
consumption. Patching gaps as the downstream finds them is the wrong arc;
cf-scan-prep needs a recon → completion pass.

## What's shipped on `dev` that's NOT on `main`

`dev` is ahead of `main` (`007db543`) by **5 commits**, all green tests +
clippy. Listed innermost-first (oldest first):

| Hash | Subject | Scope |
|---|---|---|
| `6e4d3973` | slice 9.6a+b — inset plug + layer outer surfaces by `cavity.inset_m` | cf-cast-cli geometry |
| `989d7301` | slice 9.6c — surface press-fit reservation in procedure.md | cf-cast-cli procedure post-process |
| `cd466309` | slice 9.5 — surface per-layer Slacker recipe in procedure.md | cf-cast-cli procedure post-process |
| `de567792` | live progress logging for v2 mold export | cf-cast + cf-cast-cli stderr |
| `6ebc60db` | slice 9.8 — make Simplify slider a save-time face budget | cf-scan-prep save path |

### Test counts on dev

- **cf-cast** 136 lib (was ~132)
- **cf-cast-cli** 42 lib + 6 integration (was 33 + 4)
- **cf-scan-prep** 69 lib (unchanged — 9.8 covered by existing `simplify_mesh` tests)
- **cf-device-design** 93 lib + 9 ignored ramps (unchanged from main)

All four crates: `cargo xtask grade` not re-run since dev shipped, but each
sub-commit was clippy-clean + fmt-clean at commit time. Pre-commit hook
fmt+clippy gates passed on all 5.

### What these commits buy

- **9.6 family**: cf-cast-cli now mirrors cf-device-design's cavity-inset
  geometry (plug + layers shifted inward by `cavity.inset_m`); the procedure
  markdown surfaces the press-fit reservation as a `## Press-Fit Reservation`
  section above `## Materials Summary`. Inline-layers casts (no `[design]`
  block) are bit-exact preserved (inset = 0 → no offset, no section).
- **9.5**: cf-cast-cli surfaces per-layer Slacker mass + fraction as a
  `## Slacker Recipe` table. Skipped when no layer opts in.
- **9.7**: cf-cast emits per-piece + per-plug stderr progress with
  compose+MC + F4 timings + face counts. cf-cast-cli emits orchestration
  milestones. Iter-1-scale runs no longer go dark for 10+ min between
  "loaded design" and final write.
- **9.8**: cf-scan-prep's Simplify panel slider now acts as a save-time
  face budget. Default 200k → 200k cleaned.stl unless the user drags up.
  Help-text added below the buttons explaining the new semantics.

## What was about to happen next (now deferred)

Slice **CL.1 centerline-trim** (planned 6 sub-commits in cf-scan-prep) was
next on the PR #246 ladder. **Not started.** Tasks #60–65 in the harness
still pending — left as-is for the upcoming recon to reshape.

The cold-read + PR #246 open (task #66) was the closing step, also not done.

## Why `dev` is staying as-is, NOT being merged

The 5 commits are individually solid (each green, each cold-read-able). But:

1. **They're tangled with cf-scan-prep incompleteness**. 9.5 + 9.6 work
   correctly downstream, but cf-scan-prep produces input meshes that
   make the downstream painful. Shipping these alone leaves the
   "scan → cast" loop still slow + missing centerline-trim.
2. **The PR shape will change** after cf-scan-prep recon. The recon may
   surface cf-scan-prep gaps that need cf-cast-cli adjustments, or it
   may justify shipping these 5 commits separately as a small "polish
   PR" before the bigger cf-scan-prep arc lands. Decision deferred to
   the recon session.

Branch posture: **leave dev pointing at `6ebc60db`. Do not push. Do not
merge. Do not rebase.** Future work branches from dev cleanly.

## The pivot — cf-scan-prep v1.0 completion

### Three-session pattern (per `feedback_bookmark_when_surface_levers_exhaust`)

1. **This file = the bookmark** (current session).
2. **Recon session** (next session): enumerate cf-scan-prep v1.0 gaps. Read
   `docs/SCAN_PREP_DESIGN.md` + the as-built code + the v1.0 promise. List
   every concrete delta. Produce `docs/CF_SCAN_PREP_V1_RECON.md`.
3. **Implementation session(s)**: ship the gaps. Order to be set in recon.

### Known gaps as starting points for recon (NOT a complete list)

- **CL.1 centerline trim** — perpendicular plane clip near each centerline end
  to drop noisy entry/exit segments. User-supplied manual sliders + live
  preview. Was the next slice in PR #246; tasks #60-65 hold the design
  outline. Defer until recon revisits.
- **Simplify-on-save** — addressed by 9.8 (`6ebc60db`). May still need recon
  attention if the recon decides the save-time-budget UX needs further
  treatment.
- **Centerline-adjust** — manual nudging of centerline points (user
  flagged as "less important than trim" — but still on the list). Not
  designed yet.
- **Anything else the design doc says + the code doesn't deliver** — the
  recon's job is to surface these by reading the design doc cold against
  the implementation.

### What "complete" should mean for cf-scan-prep v1.0

A cleaned scan emitted by cf-scan-prep should be **workshop-ready for cf-
device-design + cf-cast-cli consumption**. Concretely, a downstream tool
loading the `.cleaned.stl + .prep.toml` pair should never need to:

- Re-decimate (face count is within workshop budget) ✓ as of 9.8.
- Re-trim noisy ends (clean centerline + clean STL endpoints) — **gap**.
- Re-orient or recenter (canonical scan-frame) ✓ existing.
- Re-cap open boundaries (closed surface for SDF) ✓ existing.
- Re-derive the centerline (provenance + algorithm captured in
  `.prep.toml`) ✓ existing.

The recon should validate each of these + look for hidden assumptions.

### Workflow rules for the recon

- Read `docs/SCAN_PREP_DESIGN.md` first.
- Then read `tools/cf-scan-prep/src/main.rs` cold (no skimming).
- Then re-read the cf-device-design + cf-cast-cli code paths that
  CONSUME a `.cleaned.stl + .prep.toml` pair.
- Produce a flat list of gaps with severity + estimated effort.

## Files of interest

- `tools/cf-scan-prep/src/main.rs` — the entire cf-scan-prep crate (~4900 LOC at this snapshot, single-file).
- `docs/SCAN_PREP_DESIGN.md` — the original v1.0 design spec.
- `tools/cf-cast-cli/src/scan.rs` + `tools/cf-cast-cli/src/prep.rs` — the downstream consumer side.
- `tools/cf-device-design/src/main.rs` — the other downstream consumer.

## Resume checklist (cold-read, next session)

1. `git log --oneline -8` confirms HEAD at `6ebc60db`, 5 ahead of main.
2. `git status` is clean. No uncommitted work.
3. No background processes left running (`pgrep cf-scan-prep` / `pgrep cf-cast-cli` both empty).
4. Read **this file** (`docs/PR_246_BOOKMARK.md`) first.
5. Read **`docs/SCAN_PREP_DESIGN.md`** before opening any code.
6. Begin the recon. Output: `docs/CF_SCAN_PREP_V1_RECON.md` with a flat
   gap list.
7. DO NOT touch `dev`'s 5 shipped commits. DO NOT open PR #246 yet.
