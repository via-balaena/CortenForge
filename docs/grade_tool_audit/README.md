# Grade Tool Audit

> Initiative to audit and (likely) rebuild the `cargo xtask grade`
> tool, surfaced 2026-04-09 during Phase 1 PR prep for the
> thermo-computing line.

## What this is

The `cargo xtask grade <crate>` tool implements the seven-criterion
A-grade-or-it-doesn't-ship rubric described in `docs/STANDARDS.md`.
Phase 1 PR prep for `sim-thermostat` was the first time any sim crate
had been put through this gate. Two latent bugs surfaced (one fixed in
flight, one diagnosed but unfixed), a closer reading of the surrounding
code revealed architectural fragility across the gate functions, and
a fresh-eyes pass against `docs/STANDARDS.md` revealed that the tool's
behavior has drifted from the canonical standard in **six concrete
ways** — including one substantive philosophical disagreement (the
treatment of `wgpu` in Layer 0) and one platform mismatch (the
standard says tarpaulin is Linux-only; the tool runs it everywhere).

Total accumulated debt: **23 items** spanning confirmed bugs,
suspected bugs, architectural fragility, open investigation questions,
spec/rubric alignment, and code-vs-standard drift.

The right scope of fix is **its own initiative** — recon → audit →
chassis → rebuild plan → execute → grade — not in-flight patches
against a tool we don't yet trust. This directory holds the artifacts
for that initiative.

## Status

**CHASSIS DESIGN COMPLETE** as of 2026-04-10. Scope C (full rebuild) accepted.

- ✓ Audit findings captured (2026-04-09)
- ✓ Recon plan written (2026-04-09)
- ✓ Recon round complete (2026-04-10) — D1-D5 + B1-B5 + F1-F6 all resolved
- ✓ Recon report written (2026-04-10) — scope C recommended and accepted
- ✓ Chassis design written (2026-04-10) — A across all 7 rubric criteria
- ✗ Rebuild plan + grading — next session
- ✗ Rebuild execution — future session
- ✗ Rebuild verification + Phase 1 PR resumption — future session

## Navigation

| File | Purpose | Lifecycle |
|---|---|---|
| [`README.md`](README.md) | This file. Top-level orientation. | Updated as the initiative evolves. |
| [`audit_findings_2026-04-09.md`](audit_findings_2026-04-09.md) | The 23-item punch list, evidence, and analysis from the originating session. The canonical record of the initial audit. | Frozen. Recon findings extend this in the recon report. |
| [`recon_plan.md`](recon_plan.md) | Self-contained brief for the recon session — what to investigate, in what order, what to NOT do. | Frozen. |
| [`recon_report_2026-04-10.md`](recon_report_2026-04-10.md) | Output of the recon round: D1-D5 findings, B1-B5 verdicts, F1-F6 decisions, scope recommendation (C). | Frozen. |
| [`chassis_design.md`](chassis_design.md) | Blueprint for the rebuilt grade tool: architecture, per-criterion measurement specs, grade scale, STANDARDS.md changes, complete redesign, scope, verification plan. | Frozen. |

Future files (created by subsequent sessions):
- `rebuild_plan.md` — step-by-step execution plan
- `rebuild_plan_rubric_<date>.md` — paper-artifact rubric grading the
  plan
- `findings/` — directory of any cracks discovered during execution

## Branch / parked work

- **Originating branch**: `feature/thermo-doc-review`
- **Originating HEAD**: `c3f08c5` (the A1 `find_crate_path` fix); HEAD
  will advance once this directory + the persistence updates commit.
  The marker for "audit findings landed" is the existence of
  `docs/grade_tool_audit/audit_findings_2026-04-09.md`.
- **Pre-A1 HEAD**: `9998119` (the chassis Option C amendment)
- **Phase 1 PR is parked** on this branch pending the audit. Phase 1
  spec §12.4 #5 (`cargo xtask grade sim-thermostat` reaches A across
  7 criteria) is currently impossible to satisfy because of grade tool
  gaps, not because of sim-thermostat gaps. Six of seven §12.4
  criteria are satisfied; only #5 is open.
- The next session may want to use a fresh branch off main rather
  than continue on `feature/thermo-doc-review`. **Surface to user
  before deciding.**

## Operating principle

> Do it right so we don't have to go back and do it twice. Patience.

The user's words for this initiative. The grade tool layer is *meta*
infrastructure — it grades everything else. Getting it wrong propagates
silently into every future crate that passes through it. Treat this
with the same rigor as any code-grade initiative on the thermo line:
sharpen the axe, recon depth before commitment, no rushing.

## Cross-references

- **Phase 1 spec acceptance criterion**: `docs/thermo_computing/03_phases/01_langevin_thermostat.md` §12.4
- **Standards (canonical rubric)**: `docs/STANDARDS.md` (existence
  unverified — see recon plan D1)
- **Grade tool source**: `xtask/src/grade.rs`
- **Originating commit**: `c3f08c5`
