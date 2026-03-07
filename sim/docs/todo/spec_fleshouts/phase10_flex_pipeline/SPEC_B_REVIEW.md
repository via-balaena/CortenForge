# Spec B — Cotangent Laplacian Bending: Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_B.md`
**Implementation session(s):** 10 (build-time S1–S4), 11 (runtime S6–S8)
**Reviewer:** AI agent
**Date:** 2026-03-07

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
| Bending model | Cotangent Laplacian (Wardetzky/Garg). 4x4 B matrix precomputed per edge from rest geometry. Runtime: linear matrix-vector. | Bridson dihedral angle springs. Nonlinear atan2 each step. Per-vertex stability clamp. | | |
| Precomputed data | `flex_bending[17*e]`: 4x4 matrix + curved ref. `flexedge_flap[2*e]`: opposite vertices. | `flexhinge_vert[4*h]`: [ve0,ve1,va,vb]. `flexhinge_angle0[h]`: rest angle. No B matrix. | | |
| Force sign | Spring **subtracted** from `qfrc_spring`. Damper **subtracted** x `flex_damping[f]` from `qfrc_damper`. | Spring added as `grad * fm` to `qfrc_spring`. Damper added as `grad * fm` to `qfrc_damper`. Sign via angle_error sign. | | |
| Stability clamp | None needed (linear force, constant matrix). | Per-vertex clamp: `fm_max = 1/(dt^2 * \|grad\| * invmass)`. | | |
| Curved reference | `b[16]` encodes rest curvature via Garg correction. | No curved reference. Rest angle stored as scalar. | | |
| Bending stiffness source | Baked into B matrix via `mu * thickness^3 / volume`. | Kirchhoff-Love: `E*t^3 / (12*(1-nu^2))`. Different formula. | | |
| Topology | Per-edge flap stencil (edges, not hinges). | Per-hinge (adjacent triangle pairs, separate from edges). | | |

**Unclosed gaps:**
{List any behaviors where the gap is not fully closed. Each needs an action:
fix now, defer with tracking ID, or justify why partial closure is acceptable.}

---

## 2. Spec Section Compliance

Walk through every spec section (S1, S2, ...) and grade the implementation
against it. This is the core of the review.

### S1. `flexedge_flap` topology computation

**Grade:**

**Spec says:**
Compute per-edge flap vertices (opposite vertices in adjacent triangles) from
element connectivity during `process_flex_bodies()`. Boundary edges get
`flap = [-1, -1]`. Interior edges get valid opposite vertex indices. Uses
existing `edge_elements` HashMap and a new `edge_key_to_global` mapping.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Cotangent weight precomputation

**Grade:**

**Spec says:**
Standalone `compute_bending_coefficients()` function taking rest vertex
positions, mu, thickness. Returns `[f64; 17]` per edge. Implements the full
MuJoCo `ComputeBending()` algorithm: 4 cotangent values, weight vector,
diamond volume, material stiffness, transport vectors, cos_theta, 4x4 outer
product matrix, and Garg curved reference coefficient (b[16]). Helpers:
`cot_angle()`, `triangle_area()`. Degenerate guards: `cross_norm < 1e-30`,
`volume < 1e-30`, `sqr < 1e-30`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. `flex_bending` storage and population

**Grade:**

**Spec says:**
Flat `Vec<f64>` with `17 * nflexedge` entries on Model. Indexed as
`flex_bending[17*e + 4*i + j]` for 4x4 matrix, `flex_bending[17*e + 16]`
for Garg coefficient. Pre-allocated with zeros during edge extraction.
Populated by calling `compute_bending_coefficients()` for each interior edge
of `dim == 2` flex with `young > 0` and `thickness > 0`. Boundary edges
retain zeros.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. `FlexBendingType` enum and MJCF parsing

**Grade:**

**Spec says:**
`FlexBendingType` enum with `Cotangent` (default) and `Bridson` variants.
MJCF `bending_model` attribute parsed from `<elasticity>` child of `<flex>`.
Model field `flex_bending_type: Vec<FlexBendingType>` pushed during
`process_flex_bodies()`. Missing attribute defaults to `Cotangent`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. (Reserved — absorbed into S2/S3)

**Grade:** N/A (absorbed)

**Spec says:**
Section numbering note: session plan's S2–S5 (cotangent weights, material
stiffness, 4x4 matrix, curved reference) were absorbed into spec's S2 (all
computed in one `compute_bending_coefficients()` function) and S3 (storage).

**Implementation does:** N/A

**Gaps (if any):** None — verify S2 and S3 cover all five build-time tasks.

**Action:** None

### S6. Runtime cotangent force application

**Grade:**

**Spec says:**
Replace Bridson bending loop with `FlexBendingType`-dispatched section. For
`Cotangent`: triple-nested loop over diamond stencil — B matrix x position for
spring, B matrix x velocity for damper, plus curved reference cross-product
forces. Spring subtracted (`-=`), damper subtracted and multiplied by
`flex_damping[f]`. Pinned vertices skipped (`dofadr == usize::MAX`). Boundary
edges skipped (`flap[1] == -1`). No stability clamp. Gate: `dim == 2` and
`!flex_rigid[f]`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S7. `FlexBendingModel` trait definition and `CotangentBending`

**Grade:**

**Spec says:**
Per AD-1, use enum dispatch (not dynamic dispatch). No Rust `trait` keyword —
the `match` on `FlexBendingType` in S6 IS the trait architecture. Cotangent
is the inline match arm body. Bridson delegates to `apply_bridson_bending()`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S8. `BridsonBending` — preserve existing dihedral code

**Grade:**

**Spec says:**
Move existing bending code (lines 548–670) into `apply_bridson_bending(model,
data, flex_id, has_spring, has_damper)`. Zero algorithm changes. Function
iterates `flexhinge_*` arrays for the given flex and applies dihedral angle
spring/damper forces with existing per-vertex stability clamp. Filter:
`flexhinge_flexid[h] != flex_id` → continue.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Equilateral diamond precomputation — cotangent values, weight vector, stiffness, B matrix entries, b[16]=0, row sums=0 | T1 | | |
| AC2 | Boundary edge produces zero coefficients — single triangle, all edges boundary | T2 | | |
| AC3 | Cotangent runtime force — flat diamond deflection, z-force magnitudes, force balance sum=0, x/y=0 | T3 | | |
| AC4 | Curved reference — non-flat rest mesh, cos_theta < 1, b[16] != 0, row sums=0 | T4 | | |
| AC5 | dim=1 flex produces zero bending | T5 | | |
| AC6 | flex_rigid skip — all-pinned flex, no bending forces | T6 | | |
| AC7 | Bridson regression — identical results with `bending_model="bridson"` within 1e-14 | T7 | | |
| AC8 | Damper with flex_damping multiplier — forces scaled by damping coefficient | T8 | | |
| AC9 | DISABLE_SPRING gate — qfrc_spring zero, qfrc_damper non-zero | T9 | | |
| AC10 | DISABLE_DAMPER gate — qfrc_damper zero, qfrc_spring non-zero | T10 | | |
| AC11 | No stability clamp needed — very high stiffness, 500 steps, no NaN | T11 | | |
| AC12 | flexedge_flap topology correctness — 4x4 grid, interior/boundary classification, valid diamonds | T12 | | |
| AC13 | Zero thickness produces zero bending — all flex_bending entries zero | T13 | | |
| AC14 | FlexBendingType default is Cotangent (code review) | T14 | | |
| AC15 | Mixed bending models — two flex bodies, cotangent + bridson, no cross-contamination | T15 | | |

**Missing or failing ACs:**
{List any ACs that don't have passing tests or that failed code review.
Each needs an action: fix now, or track with rationale.}

---

## 4. Test Plan Completeness

> **Test numbering note:** Spec uses T1–T15. Implementation test functions
> may use different naming (e.g., `t01_*`, `t02_*`, `ac*_*`). Note the
> mapping convention here so readers can cross-reference.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Equilateral diamond precomputation — stiffness, B matrix, b[16], row sums (AC1) | | | |
| T2 | Boundary edge — single triangle, all flap[1]==-1, all flex_bending zeros (AC2) | | | |
| T3 | Cotangent runtime force — deflected diamond z-forces, force balance (AC3) | | | |
| T4 | Curved reference — asymmetric out-of-plane diamond, cos_theta, b[16] (AC4) | | | |
| T5 | dim=1 cable — no bending forces (AC5) | | | |
| T6 | Rigid flex — bending skipped (AC6) | | | |
| T7 | Bridson regression — 2000 steps, positions within 1e-14 of pre-Spec-B (AC7) | | | |
| T8 | Damper with flex_damping multiplier — forces scaled by 0.5 (AC8) | | | |
| T9 | DISABLE_SPRING gate — spring zero, damper non-zero (AC9) | | | |
| T10 | DISABLE_DAMPER gate — damper zero, spring non-zero (AC10) | | | |
| T11 | Cotangent stability — very high stiffness, 500 steps, no NaN (AC11) | | | |
| T12 | Flap topology — 4x4 grid, interior/boundary, valid diamonds (AC12) | | | |
| T13 | Zero thickness — all flex_bending zeros (AC13) | | | |
| T14 | Default bending type — FlexBendingType::Cotangent without attribute (AC14) | | | |
| T15 | Mixed bending models — two flex bodies, 100 steps, no error (AC15) | | | |

### Supplementary Tests

Tests that were added beyond the spec's planned test list — additional
coverage discovered during implementation or review. These don't map 1:1
to ACs but strengthen the test suite.

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| | | | |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| Boundary edge (`flap[1] == -1`) | Must produce zero force, not crash | | | |
| Single triangle (no interior edges) | All edges boundary — zero bending | | | |
| Flat rest mesh (`cos_theta=1`, `b[16]=0`) | Common case, verifies formula | | | |
| Curved rest mesh (`b[16] != 0`) | Non-flat meshes need Garg correction | | | |
| `dim == 1` skip | Cables have no bending | | | |
| `flex_rigid[f]` skip | All-pinned flex skipped | | | |
| `DISABLE_SPRING` / `DISABLE_DAMPER` | Gate flags must be respected | | | |
| Zero thickness | Produces zero stiffness, zero force | | | |
| Very high stiffness (no clamp) | Cotangent is stable without clamp | | | |
| Pinned vertex in diamond | Force insertion skipped (`dofadr == usize::MAX`) | | | |
| Degenerate triangle (zero area) | Guard against division by zero | | | |

**Missing tests:**
{List any planned tests or edge cases that weren't implemented.
Each needs an action: write now, or track with rationale.}

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| Default bending model changes from Bridson to Cotangent Laplacian | | |
| Bending force values change from nonlinear angle-dependent to linear position-dependent | | |
| Stability clamp removed for cotangent; preserved for Bridson | | |
| New Model fields: `flex_bending`, `flexedge_flap`, `flex_bending_type` | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/core/src/types/model.rs` — flex_bending, flexedge_flap, flex_bending_type fields | | |
| `sim/L0/core/src/forward/passive.rs` — replace Bridson loop with dispatch, add apply_bridson_bending() | | |
| `sim/L0/mjcf/src/types.rs` — FlexBendingType enum, bending_model field on MjcfFlex | | |
| `sim/L0/mjcf/src/parser.rs` — parse bending_model attribute | | |
| `sim/L0/mjcf/src/builder/flex.rs` — flap topology, compute_bending_coefficients, populate flex_bending | | |
| `sim/L0/tests/integration/flex_unified.rs` — T1–T15 new tests | | |

{List any files changed that were NOT in the spec's prediction:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| | |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| `ac6_bending_stiffness` | Pass (likely unchanged) — qualitative test, deflection direction preserved | | |
| `ac19_bending_damping_only` | Pass (unchanged) — young=0 → stiffness=0, same early-exit | | |
| `ac20_bending_stability_clamp` | Pass (value change) — cotangent forces linear, no clamp needed, assertions should pass easier | | |
| `ac21_single_triangle` (if exists) | Pass — all boundary edges, zero bending | | |

**Unexpected regressions:**
{Any test breakage the spec did not predict. Root-cause each one.}

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| Boundary flap vertex: `i32 -1` sentinel. Both `flap[0]` and `flap[1]` set to `-1` for boundary (diverges from MuJoCo which populates `flap[0]`). Harmless — boundary edges skipped. | Use `i32 -1` for boundary; check `flap[1] == -1` to skip. | | |
| Vertex DOF access: `flexvert_dofadr[v[i]]` substituted for MuJoCo's `body_dofadr[bodyid[v[i]]]`. Skip if `usize::MAX` (pinned). | Substitute `flexvert_dofadr[v[i]]` for `body_dofadr[bodyid[v[i]]]`. | | |
| `flex_bending` flat indexing: `17*e + 4*i + j` (same as MuJoCo C array). | Direct port — same indexing. | | |
| `flexedge_flap` indexing: `Vec<[i32; 2]>` indexed as `flexedge_flap[e][k]` (vs MuJoCo flat `flex_edgeflap[2*e + k]`). | Use Rust array indexing instead of flat offset. | | |
| Damping multiplier: `flex_damping[f]` → `model.flex_damping[flex_id]`. | Direct port — same field name. | | |
| Force accumulation: `data.qfrc_spring`, `data.qfrc_damper`. | Direct port — same convention from Phase 5. | | |
| Disable flags: `has_spring` / `has_damper` locals (already in `mj_fwd_passive`). | Use existing locals. | | |
| Rigid skip: `model.flex_rigid[flex_id]` (from T1). | Direct port — use T1's pre-computed flag. | | |
| `flexedge_vert`: `model.flexedge_vert[e][0]`, `model.flexedge_vert[e][1]` (Rust array indexing). | Use Rust array indexing. | | |

---

## 7. Weak Implementation Inventory

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|
| | | | | |

**If no weak items are found,** document the verification methodology:

> {Verification methodology to be filled during review execution.}

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Body-attached flex vertices — cotangent bending would need edge Jacobian for force projection through body DOFs | Out of Scope, bullet 1 | | DT-87 / §27D | |
| FEM bending for `dim=3` — volumetric elements use FEM stiffness, not cotangent | Out of Scope, bullet 2 | | | |
| `elastic2d` keyword — fine-grained membrane/bending/FEM mode control | Out of Scope, bullet 3 | | DT-86 | |
| Per-edge material variation — MuJoCo uses per-flex, same in CortenForge | Out of Scope, bullet 4 | | | |
| Hinge topology optimization — `flex_hingeadr`/`flex_hingenum` for O(1) Bridson iteration | Out of Scope, bullet 5 | | | |
| GPU flex bending | Out of Scope, bullet 6 | | DT-67 | |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| | | | | |

### Discovered During Review

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| | | | | |

### Spec Gaps Found During Implementation

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| | | | |

---

## 9. Test Coverage Summary

**Domain test results:**
```
{To be filled during review execution}
```

**New tests added:** {count}
**Tests modified:** {count}
**Pre-existing test regressions:** {count — should be 0}

**Clippy:** {clean / N warnings}
**Fmt:** {clean / issues}

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
