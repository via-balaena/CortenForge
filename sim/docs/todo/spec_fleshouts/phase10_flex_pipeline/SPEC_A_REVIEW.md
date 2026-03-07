# Spec A — Sparse Flex Edge Jacobian: Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_A.md`
**Implementation session(s):** Session 5
**Reviewer:** AI agent
**Date:** 2026-03-06

---

## Purpose

This review verifies that the implementation matches the approved spec,
surfaces weakly implemented items that should be fixed now, and ensures
deferred work is properly tracked so nothing falls through the cracks.

**Five questions this review answers:**

1. **Closed?** Are the conformance gaps from the spec's Key Behaviors
   table actually closed?
2. **Faithful?** Does the implementation match the spec — every section,
   every AC, every convention note, every planned test?
3. **Predicted?** Did the blast radius match the spec's predictions, or
   were there surprises?
4. **Solid?** Are there any items that technically "work" but are weakly
   implemented (hacks, TODOs, incomplete edge cases, loose tolerances)?
5. **Tracked?** Is every piece of deferred or out-of-scope work tracked
   in `sim/docs/todo/` or `sim/docs/ROADMAP_V1.md`?

---

## 1. Key Behaviors Gap Closure

The spec's Key Behaviors table has a "CortenForge (current)" column showing
the conformance gap *before* implementation. Verify each gap is now closed.

| Behavior | MuJoCo (from spec) | CortenForge Before | CortenForge After | Gap Closed? |
|----------|-------------------|--------------------|-------------------|-------------|
| Edge Jacobian structure | 1 row per edge, CSR format, sparsity from `mj_jacDifPair`. `engine_setconst.c:makeFlexSparse()` | No sparse Jacobian. Inline `±direction` at each consumer site. | | |
| J computation | `vec^T * jacdif` per edge in `mj_flex()`. `engine_core_smooth.c:~1290–1330` | Not implemented. Direction computed inline at each consumer. | | |
| Edge spring-damper force | `J^T * force` sparse loop. `engine_passive.c:~671–679` | Inline `±direction * force` to `dof0+ax`, `dof1+ax`. `passive.rs:541–564` | | |
| Edge velocity | `J * qvel` sparse mat-vec. `engine_forward.c:mj_fwdVelocity()` | Inline `(vel1-vel0)·direction`. `dynamics/flex.rs:27–57` | | |
| Constraint Jacobian | Scatter `flexedge_J` row into `efc_J`. `engine_core_constraint.c:mj_instantiateEquality()` | Inline `±direction[k]` to `efc_J[(row, dof+k)]`. `assembly.rs:340–349` | | |
| Rigid flex skip | `flex_rigid[f]` skips J computation | `flex_rigid[f]` skips passive force loop (T1). J computation not implemented. | | |
| Zero-length edge | `dist < epsilon → skip J` | `dist < 1e-10 → skip` in passive and velocity | | |

**Unclosed gaps:**

---

## 2. Spec Section Compliance

Walk through every spec section (S1, S2, ...) and grade the implementation
against it. This is the core of the review.

**How to grade each section:**

- Read the spec section (algorithm, file path, MuJoCo equivalent, design
  decision, before/after code).
- Read the actual implementation.
- Compare. Grade honestly.

| Grade | Meaning |
|-------|---------|
| **Pass** | Implementation matches spec. Algorithm, file location, edge cases all correct. |
| **Weak** | Implementation works but deviates from spec, uses shortcuts, has loose tolerances, missing edge-case guards, or TODOs. **Fix before shipping.** |
| **Deviated** | Implementation intentionally diverged from spec (spec gap discovered during implementation). Deviation is documented and justified. Acceptable if the spec was updated. |
| **Missing** | Section not implemented. Must be either fixed or explicitly deferred with tracking. |

### S1. CSR Structure on Model — Builder Allocation

**Grade:**

**Spec says:**
Add 3 CSR fields to Model (`flexedge_J_rownnz`, `flexedge_J_rowadr`,
`flexedge_J_colind`). Compute sparsity pattern at build time from edge
endpoint body DOF counts. Pinned vertices (worldbody, `body_dof_num=0`)
naturally contribute 0 DOFs. Files: `model.rs`, `model_init.rs`,
`builder/flex.rs`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Runtime Jacobian Computation on Data — `mj_flex_edge()` Rewrite

**Grade:**

**Spec says:**
Add `flexedge_J: Vec<f64>` to Data. Rewrite `mj_flex_edge()` to compute
`J_edge = vec^T * jacdif` (for free vertices: `[-vec^T, +vec^T]`) and
`velocity = J * qvel` (sparse dot product). Per-flex skip conditions:
`flex_rigid[f]` skips J+velocity; `skip_jacobian` (no stiffness/damping/
constraints) falls back to inline velocity. Zero-length edges (dist < 1e-10)
skip J computation. Length always computed, even for rigid flex.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. Consumer Migration — Edge Spring-Damper (`passive.rs`)

**Grade:**

**Spec says:**
Replace inline `±direction * force` application (at `dof0+ax`, `dof1+ax`)
with unified `J^T * force` sparse loop reading from `flexedge_J`,
`flexedge_J_rowadr`, `flexedge_J_rownnz`, `flexedge_J_colind`. Remove
`dof0`, `dof1`, `direction` variables from edge spring-damper section.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Consumer Migration — Velocity (`flex.rs`)

**Grade:**

**Spec says:**
Velocity computation is absorbed into the S2 rewrite of `mj_flex_edge()`.
The current inline velocity computation is replaced by
`velocity[e] = J * qvel` using the sparse Jacobian. Documented as separate
section for traceability — not a separate code change from S2.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. Consumer Migration — Constraint Jacobian (`assembly.rs`)

**Grade:**

**Spec says:**
Replace inline `±direction[k]` Jacobian population at `efc_J[(row, dof+k)]`
with scatter from sparse J: `efc_J[(row, colind[j])] = flexedge_J[rowadr+j]`.
Also replace inline velocity error computation with
`data.flexedge_velocity[e]`. Removes `vel0`, `vel1`, `direction` from
constraint assembly flex edge section.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

Walk through every AC from the spec. For runtime ACs, confirm the test
exists and passes. For code-review ACs, perform the review now.

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | CSR structure correctness — 3-vertex cable with pin | T1 | | |
| AC2 | CSR structure — all-free 4-vertex square shell | T2 | | |
| AC3 | Free-vertex Jacobian identity (`[-1,0,0,+1,0,0]`) | T3 | | |
| AC4 | Free-vertex simulation regression — bit-identical | T4 | | |
| AC5 | Velocity via J*qvel matches inline | T5 | | |
| AC6 | Constraint Jacobian migration | T6 | | |
| AC7 | Pinned vertex — zero J contribution | T7 | | |
| AC8 | Rigid flex skip | T8 | | |
| AC9 | Zero-length edge skip | T9 | | |
| AC10 | No orphan inline ±direction sites (code review) | — (code review) | | |
| AC11 | Multi-flex model | T10 | | |

**Missing or failing ACs:**

---

## 4. Test Plan Completeness

Verify every planned test from the spec was actually written. The AC table
above checks that ACs have *some* test — this section checks that the
spec's *full* test plan was executed.

> **Test numbering note:** If the spec's test labels (T1, T2, ...) differ
> from the implementation's test function names (e.g., `t01_*`, `t02_*`),
> note the mapping convention here so readers can cross-reference.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | CSR structure — pinned cable (3-vertex, v0 pinned). Assert `rownnz=[3,6]`, `rowadr=[0,3]`, colind length=9. | | | |
| T2 | CSR structure — all-free shell (4-vertex, 5 edges). Assert every `rownnz=6`, colind length=30. | | | |
| T3 | Free-vertex Jacobian values (2-vertex cable, dir=[1,0,0]). Assert `flexedge_J = [-1,0,0,+1,0,0]` exact. | | | |
| T4 | Simulation regression — bit-identical (10 steps, qfrc_spring/damper/qpos/qvel). | | | |
| T5 | Velocity J*qvel (3-vertex cable, 5 steps, compare to inline `(vel1-vel0)·direction`). | | | |
| T6 | Constraint Jacobian scatter (3-vertex cable, edge constraints `solref=[-100,-10]`, v0 pinned). efc_J row matches flexedge_J at colind positions. | | | |
| T7 | Pinned vertex — zero contribution (3-vertex cable, v0 pinned). Edge 0 rownnz=3, J = `[+vec^T]` only. | | | |
| T8 | Rigid flex skip (all 3 vertices pinned). All flexedge_J = 0, velocity = 0, length IS computed. | | | |
| T9 | Zero-length edge (coincident vertices). J = 0, velocity = 0, length ≈ 0, no panic/NaN. | | | |
| T10 | Multi-flex model (cable + shell, different stiffness). CSR rowadr spans both flexes, independent J values. | | | |

### Supplementary Tests

Tests that were added beyond the spec's planned test list — additional
coverage discovered during implementation or review. These don't map 1:1
to ACs but strengthen the test suite.

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| S1 | nflexedge=0 model — flex with 1 vertex and 0 edges. CSR arrays empty, no OOB in mj_flex_edge(). | | |

### Edge Case Inventory

Cross-reference the spec's Edge Case Inventory. Were all edge cases tested?

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| Pinned vertex (dofadr=usize::MAX) | rownnz must exclude pinned body's DOFs; J must not write to invalid indices | | | |
| Fully rigid flex (all pinned) | J computation must be skipped entirely | | | |
| Zero-length edge (dist < 1e-10) | Must skip J computation without NaN/panic from division by zero | | | |
| Mixed rigid/non-rigid edges | Within one flex, some edges rigid (both endpoints pinned), others not | | | |
| Single-vertex flex (no edges) | nflexedge=0 for this flex; all CSR arrays empty, no OOB in mj_flex_edge() | | | |
| Both vertices on same body | §27D-only; impossible for current free-vertex models. Noted for forward-compat but untestable until §27D. | | | |
| Disabled spring+damper, active constraint | J must still be computed when stiffness=0 and damping=0 but edge constraints are active (solref != [0,0]) | | | |
| Multi-flex CSR offsets | rowadr must span across flex boundaries correctly | | | |

**Missing tests:**

---

## 5. Blast Radius Verification

The spec predicted behavioral changes, files affected, and existing test
impact. Compare predictions against reality. Surprises here reveal spec
gaps worth learning from.

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| Edge spring force application: inline `±direction * force` → sparse `J^T * force` | | |
| Edge velocity computation: inline `(vel1-vel0)·direction` → sparse `J * qvel` | | |
| Constraint Jacobian assembly: inline `±direction[k]` → scatter `flexedge_J` | | |
| Constraint velocity error: inline `(vel1-vel0)·direction` → read `data.flexedge_velocity[e]` | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/core/src/types/model.rs` — Add 3 CSR fields | | |
| `sim/L0/core/src/types/model_init.rs` — Init CSR fields | | |
| `sim/L0/core/src/types/data.rs` — Add `flexedge_J`, Clone, reset | | |
| `sim/L0/core/src/dynamics/flex.rs` — Rewrite `mj_flex_edge()` | | |
| `sim/L0/core/src/forward/passive.rs` — Replace inline force application | | |
| `sim/L0/core/src/constraint/assembly.rs` — Replace inline Jacobian + velocity | | |
| `sim/L0/mjcf/src/builder/flex.rs` — Allocate CSR structure | | |
| `sim/L0/tests/integration/flex_unified.rs` — New tests T1–T10 + S1 | | |

{List any files changed that were NOT in the spec's prediction:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|

### Non-Modification Sites: Predicted vs Actual

The spec explicitly predicted these files would NOT be changed by Spec A.
If any were changed, that's an over-reach signal.

| File:line | What It Does | Why NOT Modified (spec) | Actually Unchanged? | Notes |
|-----------|-------------|------------------------|---------------------|-------|
| `passive.rs:567–699` | Bending force section (dihedral angle spring-damper) | Modified by Spec B, not Spec A. Bending does NOT use `flexedge_J`. | | |
| `dynamics/flex.rs:14–19` | `mj_flex()` — vertex xpos sync from body FK | Input to `mj_flex_edge()`, no change needed. | | |
| `forward/mod.rs:311–312` | Pipeline call sites for `mj_flex()` and `mj_flex_edge()` | Same functions, same call order. Signature unchanged. | | |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| All 43 `flex_unified.rs` tests | Pass (unchanged) — bit-identical results for free vertices | | |
| Flex conformance tests | Pass (unchanged) — bit-identical results | | |
| Non-flex tests | Pass (unchanged) — no code changes outside flex subsystem | | |

**Unexpected regressions:**

---

## 6. Convention Notes Audit

Check every row in the spec's Convention Notes table. Convention mismatches
are the #1 source of silent conformance bugs.

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| Edge vertex indexing | Direct port — `flexedge_vert[e]` gives global vertex indices directly; no base offset needed. | | |
| Vertex body ID | Direct port — `flexvert_bodyid[v]` globally indexed. | | |
| Vertex DOF address | Use `flexvert_dofadr[v]` directly. For `body_dofnum`, look up `model.body_dof_num[model.flexvert_bodyid[v]]`. | | |
| Pinned vertex sentinel | Check `dofadr == usize::MAX` to identify pinned vertices. Equivalent to MuJoCo's `body_dofnum == 0`. | | |
| Differential Jacobian | For free vertices, construct `jacdif` inline: `-I₃` for v0, `+I₃` for v1. Body-attached deferred to §27D. | | |
| J data layout | `data.flexedge_J[rowadr + j]` — same flat CSR data layout as MuJoCo. | | |
| Force target arrays | Index with `[colind]` instead of `[dof+ax]`. | | |
| Body DOF count | `model.body_dof_num[bodyid]` — same semantics as MuJoCo's `body_dofnum[bodyid]`. | | |
| Vector arithmetic | nalgebra `Vector3` supports `[]` indexing with same semantics as MuJoCo `mjtNum vec[3]`. | | |

---

## 7. Weak Implementation Inventory

Items that technically work but aren't solid. These should be fixed now —
"weak" items left unfixed tend to become permanent technical debt.

**What counts as weak:**

- `TODO` / `FIXME` / `HACK` comments in new code
- Hardcoded values that should come from the spec or MuJoCo
- Loose tolerances (e.g., 1e-3 where MuJoCo conformance demands 1e-10)
- Missing edge-case guards the spec calls for
- Placeholder error handling (e.g., `unwrap()` in non-test code, empty
  `Err(_) => {}` that swallows information)
- Functionality that "passes tests" but uses a different algorithm than
  the spec prescribes
- Dead code or commented-out code from debugging

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|

---

## 8. Deferred Work Tracker

Every item that was in the spec's scope but not fully implemented, plus
anything discovered during implementation or review that's out of scope.
**The goal: nothing deferred is untracked.**

For each item, verify it appears in at least one of:
- `sim/docs/todo/` (future_work file or spec_fleshout)
- `sim/docs/ROADMAP_V1.md`
- The umbrella spec's Out of Scope section (if applicable)

If it's not tracked anywhere, it's a review finding — add tracking now.

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Body-attached flex vertices | Out of Scope, bullet 1 | | §27D | |
| Full `mj_jacDifPair()` implementation | Out of Scope, bullet 2 | | §27D dependency | |
| Sparse constraint assembly | Out of Scope, bullet 3 | | post-v1.0 | |
| GPU flex pipeline | Out of Scope, bullet 4 | | DT-67 | |
| `flex_edgeequality` flag | Out of Scope, bullet 5 | | deferred | |

### Discovered During Implementation

Items that came up during implementation that weren't anticipated by the
spec. These are the most likely to be untracked.

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|

### Discovered During Review

Items found during this review that were not surfaced during
implementation. Reviews are a discovery mechanism — gaps in tracking,
missing match arms, untracked deferred items, and spec inaccuracies
often only become visible during side-by-side review.

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|

### Spec Gaps Found During Implementation

Items where the spec was wrong or incomplete and was (or should have been)
updated. Verify the spec was actually updated.

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|

---

## 9. Test Coverage Summary

Quick sanity check on test health after the implementation.

**Domain test results:**
```
sim-core:              N passed, 0 failed, M ignored
sim-mjcf:              N passed, 0 failed, M ignored
sim-conformance-tests: N passed, 0 failed, M ignored
Total:                 N passed, 0 failed, M ignored
```

**New tests added:**
**Tests modified:**
**Pre-existing test regressions:**

**Clippy:**
**Fmt:**

---

## 10. Review Verdict

| Category | Section | Status |
|----------|---------|--------|
| Key behaviors gap closure | 1 | |
| Spec section compliance | 2 | |
| Acceptance criteria | 3 | |
| Test plan completeness | 4 | |
| Blast radius accuracy | 5 | |
| Convention fidelity | 6 | |
| Weak items | 7 | |
| Deferred work tracking | 8 | |
| Test health | 9 | |

**Overall:**

**Items fixed during review:**

**Items to fix before shipping:**

**Items tracked for future work:**
