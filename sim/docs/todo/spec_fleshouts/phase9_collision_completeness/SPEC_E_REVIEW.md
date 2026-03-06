# Spec E — Deformable Complex-Geom Narrowphase: Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_E.md`
**Implementation session(s):** 25
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
| Flex vertex vs mesh geom | Sphere-vs-convex-hull via GJK/EPA — produces contact with point, normal, depth | `_ => return None` at `flex_collide.rs:153` — no contact generated | | |
| Flex vertex vs hfield geom | Sphere-vs-hfield via `mjc_ConvexHField()` prism-based — produces contact | `_ => return None` — no contact generated | | |
| Flex vertex vs SDF geom | Sphere-vs-SDF via distance field query — produces contact | `_ => return None` — no contact generated | | |
| Normal direction | From geom surface toward vertex (outward from geom) | N/A (no contact generated) | | |
| Contact parameter combination | Uses flex-specific params (flex friction, condim, solref, solimp) | Already implemented in `make_contact_flex_rigid()` at `flex_collide.rs:169` — works for any geom type | | |
| Missing geom data guard | No crash if geom has no mesh/hfield/SDF data — skip silently | N/A (must implement: `geom_mesh[gi] = None` → no contact) | | |

**Unclosed gaps:**
{To be filled during review execution.}

---

## 2. Spec Section Compliance

Walk through every spec section (S1, S2, ...) and grade the implementation
against it.

### S1. Flex-vs-mesh match arm

**Grade:**

**Spec says:**
Add a `GeomType::Mesh` match arm to `narrowphase_sphere_geom` in
`flex_collide.rs`. Primary path: convex hull GJK via
`CollisionShape::convex_mesh_from_hull(hull)` + `gjk_epa_contact()`.
Fallback: per-triangle BVH via `mesh_sphere_contact()`. Option B early
return. Negate GJK normal (A=sphere, B=hull → raw normal points toward
hull). Thread `model.ccd_iterations` and `model.ccd_tolerance` to GJK.
Guard on `model.geom_mesh[geom_idx]?`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Flex-vs-hfield match arm

**Grade:**

**Spec says:**
Add a `GeomType::Hfield` match arm. Reuse `heightfield_sphere_contact()`
directly with centering offset for corner-origin coords:
`hf_offset = geom_mat * Vector3::new(-hf_size[0], -hf_size[1], 0.0)`.
Option B early return. Guard on `model.geom_hfield[geom_idx]?`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. Flex-vs-SDF match arm

**Grade:**

**Spec says:**
Add a `GeomType::Sdf` match arm. Reuse `sdf_sphere_contact()` directly.
No centering offset needed. Option B early return. Guard on
`model.geom_sdf[geom_idx]?`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Remove catch-all comment and update

**Grade:**

**Spec says:**
Replace the old catch-all comment `// Mesh/Hfield/Sdf: not yet supported`
with `// All GeomType variants handled above. Defensive guard for future
enum extensions.` Keep the `_ => return None` as a defensive catch-all.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Flex-vs-mesh generates contact (sphere at Z=0.45, r=0.1, cube hull at origin → depth ≈ 0.15, normal Z > 0.5) | T1, TS1 | | |
| AC2 | Flex-vs-hfield generates contact (sphere at Z=0.45, r=0.1, flat hfield at Z=0.5 → depth ≈ 0.15, normal ≈ (0,0,1)) | T2, TS2, TS4 | | |
| AC3 | Flex-vs-SDF generates contact (sphere at Z=0.95, r=0.1, sphere SDF r=1.0 → depth ≈ 0.05, normal Z > 0.5) | T3 | | |
| AC4 | Missing mesh data guard — `geom_mesh[idx] = None` → returns None, no panic | T4 | | |
| AC5 | Missing hfield data guard — `geom_hfield[idx] = None` → returns None, no panic | T4 | | |
| AC6 | Missing SDF data guard — `geom_sdf[idx] = None` → returns None, no panic | T4 | | |
| AC7 | Vertex above surface — no contact for mesh/hfield/SDF | T5, TS3 | | |
| AC8 | Normal direction convention — normals point from geom toward vertex | — (code review) | | |
| AC9 | Catch-all comment updated — no longer says "not yet supported" | — (code review) | | |
| AC10 | Flex-vs-mesh per-triangle BVH fallback — mesh without hull generates contact | T6 | | |

**Missing or failing ACs:**
{To be filled during review execution.}

---

## 4. Test Plan Completeness

> **Test numbering note:** Spec uses T1–T6 and TS1–TS4. Implementation test
> function names will be mapped here during review execution.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Flex vertex sphere vs mesh with convex hull → AC1 | | | |
| T2 | Flex vertex sphere vs heightfield → AC2 | | | |
| T3 | Flex vertex sphere vs SDF → AC3 | | | |
| T4 | Missing geom data → no contact, no panic → AC4, AC5, AC6 | | | |
| T5 | Vertex above surface — no contact → AC7 | | | |
| T6 | Flex-vs-mesh per-triangle BVH fallback → AC10 | | | |

### Supplementary Tests

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| TS1 | Zero-radius vertex sphere against mesh — degenerate input, no panic | | |
| TS2 | Hfield vertex on surface — boundary: penetration ≈ 0, no false positive | | |
| TS3 | SDF vertex outside bbox — `sdf_sphere_contact` returns None gracefully | | |
| TS4 | Single-cell hfield (2x2 grid) — minimum grid, centering offset check | | |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| Mesh with no convex hull | Falls back to per-triangle BVH — verify fallback works | | | |
| `geom_mesh[idx] = None` | Missing data guard — `?` operator returns None | | | |
| `geom_hfield[idx] = None` | Missing data guard | | | |
| `geom_sdf[idx] = None` | Missing data guard | | | |
| Vertex above all surfaces | No contact should be generated | | | |
| Zero-radius flex vertex | `sphere_radius = 0` passed through; contact functions handle degenerately small spheres | | | |
| Hfield vertex at grid boundary | Centering offset must be correct; `heightfield_sphere_contact` clamps to bounds | | | |
| Hfield vertex exactly on surface | Boundary between contact/no-contact — `penetration = 0` case | | | |
| SDF vertex outside bounding box | `sdf.distance()` returns `None` for out-of-bounds queries → `sdf_sphere_contact` returns `None` | | | |
| Single-triangle mesh (no hull) | Minimal mesh geometry — per-triangle fallback on simplest mesh | | | |
| Single-cell hfield (2x2 grid) | Minimal heightfield — verifies centering offset works at minimum grid size | | | |

**Missing tests:**
{To be filled during review execution.}

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| `narrowphase_sphere_geom` for Mesh/Hfield/Sdf geom types: returns `Some(...)` when vertex sphere penetrates (was `None`) | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/core/src/collision/flex_collide.rs` (+60 / ~2 modified) | | |

{List any files changed that were NOT in the spec's prediction:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| {to be filled during review execution} | |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| All existing `flex_collide` tests | Pass (unchanged) — no existing tests exercise Mesh/Hfield/Sdf flex paths | | |
| All existing mesh/hfield/SDF collision tests | Pass (unchanged) — new callers of existing functions, doesn't modify them | | |
| All Phase 8 constraint tests | Pass (unchanged) — no constraint code modified | | |

**Unexpected regressions:**
{To be filled during review execution.}

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| Flex vertex sphere | Direct port — `narrowphase_sphere_geom` receives the inflated radius (already inflated in `mj_collision_flex` at `mod.rs:604`) | | |
| `MeshContact.normal` | Direct port — outward from mesh surface (toward sphere), no negation needed | | |
| `HeightFieldContact.normal` | Direct port — upward from terrain (toward object), no negation needed | | |
| `SdfContact.normal` | Direct port — outward from SDF surface (toward object), no negation needed | | |
| Hfield local coords | Apply centering offset: `hf_offset = geom_mat * Vector3::new(-hf_size[0], -hf_size[1], 0.0)` | | |
| `Pose` construction | `Pose::from_position_rotation(Point3::from(pos), UnitQuaternion::from_matrix(&mat))` | | |
| `GjkContact.normal` | **Negate** — `return Some((..., -gjk.normal, ...))` to get outward-from-mesh-toward-vertex convention | | |
| BVH midphase flag | `!disabled(model, DISABLE_MIDPHASE)` → `use_bvh` for per-triangle fallback path | | |
| GJK/EPA solver params | Use `model.ccd_iterations` and `model.ccd_tolerance` — NOT hardcoded defaults | | |

---

## 7. Weak Implementation Inventory

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|
| {to be filled during review execution} | | | | |

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Flex self-collision (Phase 10, §42A-iv) | Out of Scope, bullet 1 | | | |
| Flex-flex cross-body collision filtering (Phase 10, §42A-v) | Out of Scope, bullet 2 | | | |
| SAP for flex broadphase (DT-69) | Out of Scope, bullet 3 | | | |
| Flex adhesion contacts (DT-72) | Out of Scope, bullet 4 | | | |
| GPU flex pipeline (DT-67) | Out of Scope, bullet 5 | | | |
| Prism-based hfield collision for flex vertices | Out of Scope, bullet 6 | | | |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| {to be filled during review execution} | | | | |

### Discovered During Review

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| {to be filled during review execution} | | | | |

### Spec Gaps Found During Implementation

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| {to be filled during review execution} | | | |

---

## 9. Test Coverage Summary

**Domain test results:**
```
{to be filled during review execution}
```

**New tests added:** {to be filled}
**Tests modified:** {to be filled}
**Pre-existing test regressions:** {to be filled}

**Clippy:** {to be filled}
**Fmt:** {to be filled}

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
{To be filled during review execution.}

**Items to fix before shipping:**
{To be filled during review execution.}

**Items tracked for future work:**
{To be filled during review execution.}
