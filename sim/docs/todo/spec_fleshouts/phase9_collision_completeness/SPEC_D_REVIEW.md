# Convex Collision Solver Completeness — Spec D Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_D.md`
**Implementation session(s):** Session 20
**Reviewer:** AI agent
**Date:** 2026-03-05

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

## 0. Scope Adjustment Verification

The spec contains a major scope adjustment: the original umbrella/session-plan
described "conservative-advancement CCD" and "time-of-impact estimation" for
tunneling prevention. The spec's rubric phase discovered that MuJoCo's "CCD"
stands for "Convex Collision Detection" (from the `libccd` library), NOT
continuous collision detection. The scope was redefined to:

1. GJK distance query (for margin-zone contacts, not conservative advancement)
2. Margin-zone contact generation for GJK/EPA pairs
3. Parse `ccd_tolerance`, wire `ccd_iterations`/`ccd_tolerance` to solver
4. MULTICCD multi-point contact generation
5. `DISABLE_NATIVECCD` conformant no-op
6. Return type migration (`Option<Contact>` → `Vec<Contact>`)

The reviewer must verify the implementation matches the **adjusted** scope
(convex collision solver completeness), not the original umbrella description
(time-of-impact CCD). No conservative advancement or velocity-based filtering
should exist in the implementation.

---

## 1. Key Behaviors Gap Closure

The spec's Key Behaviors table has a "CortenForge (current)" column showing
the conformance gap *before* implementation. Verify each gap is now closed.

| Behavior | MuJoCo (from spec) | CortenForge Before | CortenForge After | Gap Closed? |
|----------|-------------------|--------------------|-------------------|-------------|
| GJK/EPA iteration limit | Configurable via `ccd_iterations`, default 35. Single parameter controls both GJK and EPA. | Hardcoded: `GJK_MAX_ITERATIONS=64`, `EPA_MAX_ITERATIONS=64`. `ccd_iterations` parsed (default 50, WRONG) but not threaded to solver. | | |
| GJK/EPA convergence tolerance | Configurable via `ccd_tolerance`, default 1e-6. | Hardcoded: `EPA_TOLERANCE=1e-6`. `ccd_tolerance` not parsed. | | |
| Margin-zone contacts (GJK/EPA pairs) | Generated: convex solver returns separation distance for non-overlapping shapes, pipeline creates contact when `dist < margin`. | **Missing**: `gjk_epa_contact()` returns `None` for non-overlapping shapes. All GJK/EPA pairs miss margin-zone contacts. | | |
| Multi-point contacts (MULTICCD) | When enabled: runs penetration solver with multiple initial directions, generates up to 50 contacts per pair. | **Not implemented**: `tracing::warn!`, single contact only. | | |
| `DISABLE_NATIVECCD` flag | Disables native solver, falls back to libccd. | `tracing::warn!` no-op. Flag stored on Model but not acted on. | | |
| `ccd_iterations` default | 35 (verified MuJoCo 3.5.0) | 50 in `MjcfOption` (**WRONG**) | | |

**Unclosed gaps:**

---

## 2. Spec Section Compliance

Walk through every spec section (S1, S2, ...) and grade the implementation
against it. This is the core of the review.

### S1. GJK Distance Query

**Grade:**

**Spec says:**
Add standalone `gjk_distance()` function to `gjk_epa.rs` (~120 lines) with
`GjkDistanceResult` return type, `closest_point_on_simplex_to_origin()`,
`closest_point_on_triangle_to_origin()` (full Voronoi region analysis),
`recover_witness_points()`, and `reduce_simplex_to_closest()` helpers.
Returns `None` for overlapping shapes (transition to EPA).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Parameter Threading (Parse + Model + Wire)

**Grade:**

**Spec says:**
S2a: Fix `ccd_iterations` default from 50 to 35 in `MjcfOption`.
S2b: Parse `ccd_tolerance` from `<option>` in `parser.rs`.
S2c: Add `ccd_tolerance: f64` to `MjcfOption` (default 1e-6).
S2d: Add `ccd_iterations: usize` and `ccd_tolerance: f64` to `Model` and `model_init.rs`.
S2e: Thread through builder (`config.rs`, `builder/mod.rs`).
S2f: Update `gjk_query()`, `epa_query()`, `gjk_epa_contact()`, `gjk_intersection()`
signatures to accept `max_iterations`/`tolerance`. Update `epa_with_expanded_simplex()`
internal call. Update constants to 35/35/1e-6.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. Margin-Zone Contact Generation

**Grade:**

**Spec says:**
In `narrow.rs`: when `gjk_epa_contact()` returns `None` and `margin > 0.0`,
call `gjk_distance()` — if `distance < margin`, generate a margin-zone contact
with `depth = -separation_distance` (negative = non-penetrating). Same pattern
in `mesh_collide.rs` for mesh-mesh hull path.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. MULTICCD Multi-Point Contact Generation

**Grade:**

**Spec says:**
When `ENABLE_MULTICCD` is set, run GJK/EPA with 3 perturbed initial search
directions (tangent-plane rotations of primary contact normal) to find up to
4 total contacts. Add `multiccd_contacts()` helper, `gjk_query_with_direction()`,
and `gjk_epa_contact_with_direction()` to `gjk_epa.rs`. Filter duplicate
contacts within tolerance. Apply to both `narrow.rs` GJK/EPA path and
`mesh_collide.rs` hull path.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. Return Type Migration + Flag Cleanup

**Grade:**

**Spec says:**
S5a: Change `collide_geoms()` return from `Option<Contact>` to `Vec<Contact>`.
S5b: Change `collide_with_mesh()` return from `Option<Contact>` to `Vec<Contact>`.
S5c: Update broadphase loops in `mod.rs` (mechanism-1 and mechanism-2) to iterate
over Vec.
S5d: Remove `tracing::warn!` stubs for MULTICCD/NATIVECCD at `builder/mod.rs:933-938`.
S5e: `DISABLE_NATIVECCD` is a conformant no-op — just remove warning.
S5f: Update dead `ccd_enabled()` method in `config.rs:154`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S6. Update `hfield.rs` Call Site

**Grade:**

**Spec says:**
Update `gjk_epa_contact` call in `hfield.rs` (line 163) to pass
`model.ccd_iterations, model.ccd_tolerance`. Also update `sensor/geom_distance.rs`
and `convex_hull.rs` call sites. Pure signature compatibility, no behavioral change.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | GJK distance — separated spheres: `distance = 2.0 ± 1e-6`, witness points correct | T1 | | |
| AC2 | GJK distance — overlapping shapes return `None` | T2 | | |
| AC3 | Margin-zone contact — ellipsoid pair within margin: 1 contact, `depth ≈ -0.05` | T3 | | |
| AC4 | Margin-zone contact — beyond margin: no contact | T4 | | |
| AC5 | `ccd_iterations` wiring — non-default value reaches solver: `model.ccd_iterations == 10` | T5 | | |
| AC6 | `ccd_tolerance` parsing — non-default: `model.ccd_tolerance == 1e-8` | T6 | | |
| AC7 | `ccd_iterations` default fixed to 35 | T7 | | |
| AC8 | MULTICCD — mesh-mesh hull pair produces 4 contacts at corners | T8 | | |
| AC9 | MULTICCD disabled — single contact | T9 | | |
| AC10 | `DISABLE_NATIVECCD` — no crash, solver works | T10 | | |
| AC11 | `tracing::warn!` guards removed (code review) | — (code review) | | |
| AC12 | Existing GJK/EPA tests pass with updated signatures (20 tests) | T11 | | |
| AC13 | Return type migration — no semantic change for single-contact paths (code review) | — (code review) | | |

**Missing or failing ACs:**

---

## 4. Test Plan Completeness

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | GJK distance — separated spheres (distance ≈ 2.0, witness points) | | | |
| T2 | GJK distance — overlapping spheres return `None` | | | |
| T3 | Margin-zone contact — ellipsoid pair within margin (depth ≈ -0.05) | | | |
| T4 | Beyond margin — no contact generated | | | |
| T5 | `ccd_iterations` non-default reaches Model (value 10) | | | |
| T6 | `ccd_tolerance` non-default reaches Model (value 1e-8) | | | |
| T7 | `ccd_iterations` default is 35 | | | |
| T8 | MULTICCD — mesh-mesh hull pair produces 4 contacts | | | |
| T9 | MULTICCD disabled — single contact | | | |
| T10 | `DISABLE_NATIVECCD` — no crash | | | |
| T11 | Existing GJK/EPA tests regression (20 tests pass) | | | |

### Supplementary Tests

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| T12 | Just touching — two spheres at distance 0 ± epsilon | | |
| T13 | Zero margin — verify margin-zone path not entered | | |
| T14 | `ccd_iterations=0` — graceful handling | | |
| T15 | MULTICCD on curved contact (ellipsoids) — single contact expected | | |
| T16 | GJK distance with box shapes — exercises triangle simplex case | | |
| T17 | Sensor `geom_distance` signature compat — updated call sites work | | |
| T18 | `ccd_tolerance=0` — solver runs to max iterations, valid contact | | |
| T19 | Multi-body integration — mixed geom scene, no regression | | |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| Coincident shapes (distance=0) | GJK distance at boundary — return `None` (transition to EPA) | | | |
| Shapes just touching (distance ≈ 0) | Numerical stability at zero-distance boundary | | | |
| Zero margin | Margin-zone path not entered (`margin > 0.0` guard) | | | |
| Large separation (no contact) | GJK distance terminates quickly, returns distance > margin | | | |
| `ccd_iterations=0` | Immediate termination with no result | | | |
| `ccd_tolerance=0` | Runs to max iterations, returns final result | | | |
| MULTICCD on curved contact | Single contact expected (no flat surface) | | | |
| GJK distance with box shapes | Non-sphere shapes exercise triangle simplex case | | | |
| Multi-body mixed geom scene | Integration — no regression with mixed geom types | | | |

**Missing tests:**

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| `gjk_epa_contact()` signature: 4 params → 6 params (+max_iterations, +tolerance) | | |
| `gjk_query()` signature: 4 params → 5 params (+max_iterations) | | |
| `epa_query()` signature: 5 params → 7 params (+max_iterations, +tolerance) | | |
| Default iteration count: `GJK_MAX_ITERATIONS=64` → 35 | | |
| `MjcfOption.ccd_iterations` default: 50 → 35 | | |
| `collide_geoms()` return type: `Option<Contact>` → `Vec<Contact>` | | |
| `collide_with_mesh()` return type: `Option<Contact>` → `Vec<Contact>` | | |
| Margin-zone contacts for GJK/EPA pairs: not generated → generated when within margin | | |
| MULTICCD contacts: not generated → up to 4 contacts for flat surfaces when flag enabled | | |
| `tracing::warn!` removal for MULTICCD/NATIVECCD | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `core/src/gjk_epa.rs` | | |
| `core/src/collision/narrow.rs` | | |
| `core/src/collision/mesh_collide.rs` | | |
| `core/src/collision/mod.rs` | | |
| `core/src/collision/hfield.rs` | | |
| `core/src/sensor/geom_distance.rs` | | |
| `core/src/convex_hull.rs` | | |
| `core/src/types/model.rs` | | |
| `core/src/types/model_init.rs` | | |
| `mjcf/src/types.rs` | | |
| `mjcf/src/parser.rs` | | |
| `mjcf/src/config.rs` | | |
| `mjcf/src/builder/mod.rs` | | |

Unexpected files changed:

| Unexpected File | Why It Was Changed |
|----------------|-------------------|

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| 20 GJK/EPA tests in `gjk_epa.rs` | Signature update, all pass (iterations 64→35 sufficient) | | |
| `test_option_parsing_ccd_iterations` (parser.rs) | Pass unchanged | | |
| Hfield collision tests | `gjk_epa_contact` call site update (add params) | | |
| Mesh collision tests | Return type change: `Option<Contact>` → `Vec<Contact>` | | |
| Sensor geom_distance tests | `gjk_epa_contact` and `gjk_query` signature updates | | |
| `convex_hull.rs` test (line 1115) | `gjk_epa_contact` call signature update (add params) | | |

**Unexpected regressions:**

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `ccd_iterations` | Use `model.ccd_iterations` for both `gjk_query()` and `epa_query()` max iteration parameters | | |
| `ccd_tolerance` | Use `model.ccd_tolerance` for `epa_query()` tolerance and `gjk_distance()` convergence threshold. Keep `EPSILON=1e-8` for geometric direction checks (not configurable) | | |
| Contact depth sign | GJK distance result must be **negated** to produce `Contact.depth`. For margin-zone contacts: `depth = -separation_distance` (negative value) | | |
| Contact normal direction | `GjkContact.normal`: from shape_b toward shape_a (same as MuJoCo). `make_contact_from_geoms` already handles the convention. | | |
| Max contacts per pair | Share existing `MAX_CONTACTS_PER_PAIR = 50` constant from `hfield.rs` for MULTICCD cap | | |
| Model options location | Add `ccd_iterations: usize` and `ccd_tolerance: f64` as direct fields on `Model`, following the `sdf_iterations` / `sdf_initpoints` pattern | | |
| `DISABLE_NATIVECCD` semantics | Conformant no-op: when `DISABLE_NATIVECCD` is set, continue using our GJK/EPA (no libccd fallback) | | |

---

## 7. Weak Implementation Inventory

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Conservative advancement / time-of-impact CCD | Out of Scope, bullet 1 | | | |
| Velocity-based CCD pair filtering | Out of Scope, bullet 2 | | | |
| libccd fallback implementation | Out of Scope, bullet 3 | | | |
| Non-convex GJK distance | Out of Scope, bullet 4 | | | |
| MULTICCD for analytical pairs | Out of Scope, bullet 5 | | | |
| GJK/EPA warm-starting | Out of Scope, bullet 6 | | | |
| `ccd_enabled()` integration into collision pipeline | Out of Scope, bullet 7 | | | |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|

### Discovered During Review

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|

### Spec Gaps Found During Implementation

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|

---

## 9. Test Coverage Summary

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
