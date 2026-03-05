# Spec C — Heightfield Collision Pairs: Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_C.md`
**Implementation session(s):** Session 15
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

## 1. Key Behaviors Gap Closure

The spec's Key Behaviors table has a "CortenForge (current)" column showing
the conformance gap *before* implementation. Verify each gap is now closed.

| Behavior | MuJoCo (from spec) | CortenForge Before | CortenForge After | Gap Closed? |
|----------|-------------------|--------------------|-------------------|-------------|
| Convex-hfield collision algorithm | `mjc_ConvexHField` — prism decomposition + GJK penetration, returns 0..50 contacts | Point-sampling per type, returns 0..1 contact | | |
| Hfield-mesh | Supported via `mjc_ConvexHField` using mesh convex hull | `unreachable!` panic — dispatch sends to mesh handler | | |
| Hfield-cylinder | Exact via `mjc_ConvexHField` | Approximated as capsule | | |
| Hfield-ellipsoid | Exact via `mjc_ConvexHField` | Approximated as sphere (max radius) | | |
| Multi-contact | Up to 50 contacts per hfield pair | At most 1 contact | | |
| Hfield-plane | Not supported (null entry) | Returns `None` (correct) | | |
| Hfield-hfield | Not supported (null entry) | Returns `None` (correct) | | |
| Prism base elevation | `-size[3]` from `hfield_size[3]` | Not applicable (no prisms) | | |
| Margin handling | Added to prism top vertex Z | Not threaded to hfield helpers | | |

**Unclosed gaps:**
{To be filled during review execution.}

---

## 2. Spec Section Compliance

### S1. Dispatch ordering fix in `narrow.rs`

**Grade:**

**Spec says:**
Move the hfield check before the mesh check in `narrow.rs` (lines 86-94) so
hfield-mesh pairs are routed to the hfield handler. Remove `unreachable!`
arms in `mesh_collide.rs` at lines 107 and 140.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Multi-contact dispatch in broadphase loops

**Grade:**

**Spec says:**
Add `is_hfield_pair` type checks in mechanism 1 (~line 440) and mechanism 2
(~line 486) loops in `mod.rs`. Hfield pairs call `collide_hfield_multi`
returning `Vec<Contact>`. SDF-Hfield pairs are excluded from the hfield path.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. Prism-based `collide_hfield_multi` — core algorithm

**Grade:**

**Spec says:**
Replace `collide_with_hfield` with `collide_hfield_multi` implementing
MuJoCo's 6-phase prism algorithm: frame setup, bounding sphere early exit,
AABB via support queries, sub-grid clipping, prism iteration, per-prism
GJK/EPA collision. Returns 0..50 contacts.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3a. `build_prism` helper

**Grade:**

**Spec says:**
Construct 6 prism vertices directly (3 bottom at `-size[3]`, 3 top at
surface height + margin). Two triangle patterns per cell: `i=0` lower-left,
`i=1` upper-right. Heights from `HeightFieldData.get(x, y)` (pre-scaled).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3b. `compute_local_aabb` helper

**Grade:**

**Spec says:**
Query GJK support function along ±X, ±Y, ±Z in hfield-local frame to get
tight AABB of the convex shape. Uses `support()` from `gjk_epa.rs`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Remove old single-contact `collide_with_hfield`

**Grade:**

**Spec says:**
Remove old `collide_with_hfield` from `hfield.rs`. In `collide_geoms`, keep
hfield check (moved before mesh per S1) but return `None` as safety fallback.
Remove `use super::hfield::collide_with_hfield` import.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Hfield-sphere contact via prism approach | T1 | | |
| AC2 | Hfield-mesh contact (previously missing pair) | T2 | | |
| AC3 | Hfield-cylinder exact (no capsule approximation) | T3 | | |
| AC4 | Hfield-ellipsoid exact (no sphere approximation) | T4 | | |
| AC5 | Multi-contact generation (≥4 contacts) | T5 | | |
| AC6 | `mjMAXCONPAIR` = 50 limit | T6 | | |
| AC7 | No contacts when geom outside hfield bounds | T7 | | |
| AC8 | Hfield-mesh with no convex hull → 0 contacts | T8 | | |
| AC9 | Dispatch ordering safety — no panic for mesh-hfield | T9 | | |
| AC10 | Mechanism-2 hfield pair overrides applied to all contacts | T10 | | |
| AC11 | Hfield-plane returns no contacts | T11 | | |
| AC12 | SDF-Hfield routing preserved (not intercepted by hfield path) | T13 | | |
| AC13 | No regression in existing tests | T12 | | |
| AC14 | Dispatch and pipeline structure (code review) | — (code review) | | |

**Missing or failing ACs:**
{To be filled during review execution.}

---

## 4. Test Plan Completeness

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Hfield-sphere prism contact (5×5 flat hfield, sphere radius=0.3) | | | |
| T2 | Hfield-mesh contact — new pair (5×5 flat hfield, unit cube mesh with hull) | | | |
| T3 | Hfield-cylinder exact (5×5 flat hfield, cylinder on its side) | | | |
| T4 | Hfield-ellipsoid exact (5×5 flat hfield, ellipsoid penetrating) | | | |
| T5 | Multi-contact generation (10×10 sinusoidal hfield, large box, ≥4 contacts) | | | |
| T6 | MAX_CONTACTS_PER_PAIR = 50 limit (50×50 hfield, huge box) | | | |
| T7 | Geom outside hfield bounds → 0 contacts (sphere at x=100) | | | |
| T8 | Mesh with no convex hull → 0 contacts | | | |
| T9 | Dispatch ordering — mesh-hfield pair, no panic | | | |
| T10 | Mechanism-2 pair overrides for hfield (custom solref/solimp) | | | |
| T11 | Hfield-plane negative case → 0 contacts | | | |
| T12 | Full regression suite (all 2,489+ existing tests pass) | | | |
| T13 | SDF-Hfield routing preserved | | | |

### Supplementary Tests

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| TS1 | Single-cell hfield (2×2 grid) with sphere contact — minimum valid hfield | | |
| TS2 | Edge-straddling geom — sphere at hfield boundary, sub-grid clipping correctness | | |
| TS3 | Small geom in single cell — tiny sphere, sub-grid converges to 1-2 prisms | | |
| TS4 | Tilted hfield — rotated hfield with sphere contact, frame transform correctness | | |
| TS5 | Flat prism — all vertices at same height, degenerate prism handling in GJK | | |
| TS6 | Hfield-hfield no crash — two hfield geoms, graceful return | | |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| Flat hfield (all same height) | Simplest case — verifies basic prism construction | | | |
| Single-cell hfield (2×2 grid) | Minimum valid hfield — 1 cell = 2 prisms | | | |
| Geom entirely outside bounds | Tests bounding sphere and AABB early exits | | | |
| Geom straddling hfield edge | Partial sub-grid — only some cells overlap | | | |
| Geom smaller than one cell | Only 1-2 prisms tested | | | |
| Geom larger than entire hfield | Full grid iterated, hits MAX_CONTACTS limit | | | |
| Mesh with no convex hull | Edge case for hfield-mesh | | | |
| Tilted hfield (non-identity pose) | Tests hfield-local frame transform correctness | | | |
| Degenerate prism (coplanar top verts) | All 3 heights same — prism is a flat slab | | | |
| Hfield-plane (negative case) | Not in MuJoCo — must return 0 contacts | | | |
| Hfield-hfield (negative case) | Not in MuJoCo — must not be attempted | | | |
| SDF-Hfield (routing preservation) | S2's `is_hfield_pair` must exclude SDF to preserve existing `collide_with_sdf` routing | | | |
| Contact limit (50) | Hard cap per MuJoCo | | | |

**Missing tests:**
{To be filled during review execution.}

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| Hfield-sphere: single contact → 0-50 contacts via prism GJK/EPA | | |
| Hfield-capsule: single contact → 0-50 contacts via prism GJK/EPA | | |
| Hfield-box: single contact → 0-50 contacts via prism GJK/EPA | | |
| Hfield-cylinder: capsule approx → exact cylinder via prism, multi-contact | | |
| Hfield-ellipsoid: sphere approx → exact ellipsoid via prism, multi-contact | | |
| Hfield-mesh: `unreachable!` panic → 0-50 contacts via prism GJK/EPA | | |
| Dispatch ordering: mesh check before hfield → hfield check before mesh | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `core/src/collision/hfield.rs` (major rewrite: ~+200 / ~-80) | | |
| `core/src/collision/narrow.rs` (swap hfield/mesh order, ~10 modified) | | |
| `core/src/collision/mod.rs` (hfield multi-contact path, ~+30) | | |
| `core/src/collision/mesh_collide.rs` (remove `unreachable!`, ~-2) | | |
| `core/src/heightfield.rs` tests (~+300) | | |

{Unexpected files:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| | |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| `test_heightfield_flat` | Pass (unchanged) | | |
| `test_heightfield_from_fn` | Pass (unchanged) | | |
| `test_heightfield_bilinear_interpolation` | Pass (unchanged) | | |
| `test_heightfield_normal` | Pass (unchanged) | | |
| `test_heightfield_sphere_contact` | Pass (unchanged — tests function directly) | | |
| `test_heightfield_sphere_contact_on_slope` | Pass (unchanged) | | |
| `test_heightfield_capsule_contact` | Pass (unchanged) | | |
| `test_heightfield_box_contact` | Pass (unchanged) | | |
| `test_heightfield_cells_in_aabb` | Pass (unchanged) | | |
| `test_heightfield_aabb` | Pass (unchanged) | | |
| `test_heightfield_with_pose` | Pass (unchanged) | | |
| Integration tests using hfield (`sim-conformance-tests`) | May see different contact counts/values | | |

**Unexpected regressions:**
{To be filled during review execution.}

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| Heightfield origin (center-origin vs corner-origin) | Prism construction uses MuJoCo center-origin coords directly (`dx * c - hf_x_half`). Hfield Pose uses `hf_pos` WITHOUT centering offset. Heights accessed from `HeightFieldData` using corner-origin grid indices | | |
| Height scaling | Do NOT multiply by `size[2]` again — heights in `HeightFieldData` are pre-scaled | | |
| Grid indices | MuJoCo `r` = CortenForge `y`, MuJoCo `c` = CortenForge `x` | | |
| Grid dimensions | MuJoCo `nrow` = CortenForge `depth`, MuJoCo `ncol` = CortenForge `width` | | |
| Cell size | Compute `dx = 2*hf_size[0]/(width-1)`, `dy = 2*hf_size[1]/(depth-1)` from `hf_size` and grid dimensions. Do NOT use `cell_size()` for prism math | | |
| Base elevation | Read `hfield_size[3]` from model for prism bottom Z (`-size[3]`) | | |
| Penetration test | `gjk_epa_contact(prism_shape, prism_pose, convex_shape, convex_pose)` as direct substitute for `mjc_penetration()` | | |
| Contact normal | GJK/EPA normal points from shape A to shape B. No fixNormal needed (native mode) | | |
| Pose mutation | Compute relative transform without mutation — new variables for `hf_local_pos`/`hf_local_mat` | | |
| Upper Z extent | Use `max_height()` for bounding sphere/AABB tests (pre-scaled actual maximum), not `hf_size[2]` | | |

---

## 7. Weak Implementation Inventory

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|
| | | | | |

{To be filled during review execution. Verify by grepping for `TODO`, `FIXME`,
`HACK`, `todo!`, `unimplemented!` in all new/modified files. Review all new
production code for `unwrap()`, loose tolerances, and algorithm deviations.}

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Hfield-plane collision | Out of Scope, bullet 1 | | | |
| Hfield-hfield collision | Out of Scope, bullet 2 | | | |
| Hfield-SDF collision (already handled) | Out of Scope, bullet 3 | | | |
| Optimized prism BVH (quadtree acceleration) | Out of Scope, bullet 4 | | | |
| Flex-vs-hfield via prism approach | Out of Scope, bullet 5 | | | |

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
{To be filled during review execution.}
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
