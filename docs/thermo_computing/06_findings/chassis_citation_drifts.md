# Chassis Citation Drifts (catalogued 2026-04-09)

> **Status**: Open. Cleanup deferred — no urgency, substantive claims unaffected.
> **Origin**: Surfaced during the Phase 1 spec stress-test pass (commit `45fa896`).
> **Discipline**: The recon→iteration handoff prohibits touching the chassis or recon log mid-session for cosmetic edits. Catalogued here so the work is not lost; addressed in a future dedicated chassis-drift-audit session when there's a real reason to be in the chassis.

This is the first real file in `06_findings/` (previously only `.gitkeep`), establishing the pattern: cross-cutting findings about the doc tree itself, or about claims in the chassis / recon log that the codebase has slightly outgrown, live here.

## Background

During the stress-test pass for the Phase 1 spec (`03_phases/01_langevin_thermostat.md`), I walked every code citation in the spec against the actual files in `sim/L0/core/`. Two chassis citations were found to have minor line-range drift — the substantive claims (what the code does) are still accurate; only the line numbers shifted, presumably from later edits to those files after the chassis and recon log were originally drafted.

Both findings are non-blocking and zero-urgency. Recorded so a future "chassis drift audit + cleanup" session has a starting point.

---

## Drift #1 — `forward/passive.rs:723-731` (plugin passive forces fire after `cb_passive`)

**Where it appears**:
- `02_foundations/chassis_design.md` — Decision 7 section, multiple references
- `project_thermo_computing.md` (auto-memory mirror)

**Chassis claim**: "Plugin passive forces fire after `cb_passive` (`passive.rs:723-731`)" — i.e., line range `723-731`.

**Actual state in `sim/L0/core/src/forward/passive.rs`**: the plugin block runs lines **723–735**. The chassis range stops at line 731 (the inner `crate::plugin::PluginCapabilityBit::Passive,` argument), missing the four closing-brace / `);` lines that complete the `if model.nplugin > 0 { for i in 0..model.nplugin { ... } }` block.

**Substantive claim**: still accurate. Plugin passive forces do fire after `cb_passive` (verified at `forward/passive.rs:719-720` for the `cb_passive` invocation immediately preceding the plugin block at `:723`).

**Likely cause**: a later edit added a line of indentation or split the `compute(...)` call across more lines after the chassis was written. Cosmetic, no behavioral change.

**Spec-side state**: the Phase 1 spec already propagates the corrected range `:723-735` in §2, §11, §14, per the stress-test on the spec session itself (commit `45fa896`). The chassis is the only remaining doc still using the old range.

**Fix**: update every `chassis_design.md` reference to `passive.rs:723-731` → `:723-735`. By inspection, roughly 2-3 occurrences in the Decision 7 section.

### Status update — Drift #1 RESOLVED (2026-04-09, Phase 1 chassis amendment session)

**The chassis-location claim above was factually wrong at write-time.** During the chassis amendment session that added forward links for Cracks 1+2+4, the chassis was inspected by `grep` for any reference to `passive.rs`, `723`, `731`, `forward/passive`, or `plugin`. **All grep patterns returned zero matches** in `chassis_design.md`. The chassis genuinely has no references to the Drift #1 citation — the original finding's "Decision 7 section, multiple references" claim and "roughly 2-3 occurrences" estimate were both incorrect.

The actual stale citations to `passive.rs:723-731` exist in:

- **`docs/thermo_computing/03_phases/overview.md:165`** — inside the intentional `<details>` block at the bottom of the Phase 1 section that preserves the historical inline sketch. Per the same convention as the recon log, collapsed historical material is **not edited retroactively**: leave alone.
- **`~/.claude/projects/.../memory/project_thermo_computing.md:146`** — auto-memory file, in the now-historical "Phase 5+ caveats flagged for later" list. Out of scope for the chassis amendment commit; can be cleaned up as a separate micro-edit if/when the auto-memory file is otherwise being touched.

The Phase 1 spec (`03_phases/01_langevin_thermostat.md`) had **already adopted the corrected `:723-735` range** in §2, §11, and §14 during the spec stress-test pass — that part of the original Drift #1 finding is correct.

**Why was the original Drift #1 finding wrong about chassis location?** Most likely cause: the finding was written in the same stress-test session that surfaced the spec-side correction. The author inferred "if the spec used to have this and was just corrected, the chassis must too" without actually grep-checking the chassis. This is itself a small instance of the **same paper-vs-code translation gap** the recon-to-iteration handoff principle is about — except here the "code" is the actual file content under `grep`, not a Rust compiler. **Always grep before claiming a citation exists.**

**Disposition**: Drift #1 is **RESOLVED as not-applicable to chassis**. The chassis was already clean. No chassis edit was needed; the chassis amendment commit that added forward links for Cracks 1+2+4 also added this status update but did not touch the chassis text for any Drift #1 reason.

---

## Drift #2 — `constraint/mod.rs:78-87` (qfrc_smooth aggregation)

**Where it appears**:
- `04_recon_log/2026-04-09_part_02_forward_step.md` — the recon entry that established the force-flow trace and resolved Q2 (implicit integrator interaction)

**Recon log claim**: "`qfrc_applied[i] += …` → folded into `qfrc_smooth` in `constraint/mod.rs:78-87` (`qfrc_smooth = qfrc_applied + qfrc_actuator + qfrc_passive − qfrc_bias`, plus `xfrc_applied` projection)"

**Actual state in `sim/L0/core/src/constraint/mod.rs`**:
- Aggregation runs lines **78–84** (init at 80, accumulation loop at 81–84)
- `xfrc_applied` projection comments start at line 86; the actual projection block (the `for body_id in 1..model.nbody` loop) runs from roughly line 89 onward

**Substantive claim**: still accurate. `qfrc_smooth = qfrc_applied + qfrc_actuator + qfrc_passive - qfrc_bias` plus `xfrc_applied` projection happens in this region.

**Drift shape**: combined-range `78-87` is loose at both ends — too generous on the upper bound for the aggregation alone (which ends at 84), and too tight for the projection block (which extends past 87). Most likely the recon log was trying to give a single combined range and the code grew slightly between the aggregation and the projection blocks.

**Caveat** (the reason this one is harder than Drift #1): recon log entries are historical record per the doc-tree refactor plan §3.4 + §6 non-scope. Updating recon log line numbers in retrospect is mildly revisionist — it makes the entry "agree with current code" rather than "capture what we saw when we did the recon." If we touch this, the right move is one of:
- (a) **Leave the recon log alone** and add a tiny "see chassis_design.md / current line ranges" footnote in the relevant chassis section that propagated the citation
- (b) **Accept** that the recon log will gently drift from current code over time and treat the dates as the canonical "valid as of when written" markers — i.e., do nothing and document the convention
- (c) **Update in place** with a strikethrough-style edit (`~~78-87~~ → 78-84 + 89+`) that preserves the original text as historical artifact while showing the corrected range

Decide during the audit. **(a) is the safest option** — it leaves the recon log fully untouched and surfaces the correction in the living-doc layer where corrections belong.

**Fix**: per (a), add a footnote to whichever chassis section propagated this citation; recon log untouched.

---

## Why these live in `06_findings/` rather than the recon log

Recon log entries capture observations made at the time of a recon round. These citation drifts are *findings about the recon log and chassis themselves* — meta-findings — and belong in a different category. `06_findings/` was created during the doc-tree refactor for "cross-cutting deep dives as they accumulate" (per the refactor plan §3.1 mapping); this is the first real file to land there.

The pattern this file establishes: any future cross-cutting finding that doesn't fit cleanly into a single recon log entry, doesn't belong in the chassis (because it's a finding *about* the chassis, not a chassis decision), and isn't a per-phase concern (so doesn't go in `03_phases/`) lands here as a dated `.md` file.

---

## Suggested handling

**Cheapest path**: a single small commit during a future session that already has a reason to touch the chassis. Title: `docs(thermo-computing): chassis citation drift cleanup`. Updates:
- All `passive.rs:723-731` → `:723-735` references in `chassis_design.md` (Drift #1)
- A footnote in the relevant chassis section pointing at the corrected `constraint/mod.rs` ranges, recon log untouched (Drift #2, option (a) above)
- Removes or archives this findings file (or marks it `Status: Closed` with the resolving commit hash)

Estimated time: 15-30 minutes including verification. No urgency.

## Triggers for actually doing the audit

Don't trigger on this file alone. Wait for one of:
- A new contributor stumbles on a drift and asks why the chassis cites a line that doesn't exist
- The count of known drifts in this file grows to 4-5 and an audit becomes worth its own session
- Any session is already touching `chassis_design.md` for another reason and can ride the cleanup along
- A future spec stress-test surfaces a third or fourth drift

If none of those happen, the file sits here indefinitely without cost. The substantive claims in both citations are accurate; no implementer reading the chassis is going to be confused by an off-by-4-on-a-closing-brace.

## Adding new drifts to this file

When a future stress-test surfaces another chassis or recon log citation drift, append it as a new `## Drift #N — <citation>` section following the same template:
- Where it appears (chassis section or recon log part)
- Chassis/recon claim verbatim
- Actual state in the file with line numbers
- Substantive claim status (accurate? broken? in-between?)
- Likely cause
- Fix (option list if there are trade-offs)

Don't audit proactively from this file; only catalog drifts that surface naturally during other work.
