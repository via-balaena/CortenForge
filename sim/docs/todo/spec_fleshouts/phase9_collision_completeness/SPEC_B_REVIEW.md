# Mesh Inertia Modes (Spec B, §43) — Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_B.md`
**Implementation session(s):** Session 10
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
| Default mesh inertia mode | `Convex` (enum 0) — `user_mesh.cc` | Always `Exact` — no mode concept | | |
| Shell mesh inertia | `ComputeInertia()` shell path: area-weighted surface triangles, divisor 6/12 | **Not implemented** | | |
| Legacy mesh inertia | `ComputeInertia()` legacy path: `\|det\|/6` | **Not implemented** | | |
| Convex mesh inertia | Exact on convex hull via `ComputeInertia()` | **Not implemented** (hull exists from Spec A but not used for inertia) | | |
| Primitive shell formulas | `SetGeom()` shell path: 5 geom types | **Not implemented** | | |
| `shellinertia` on `<geom>` | Parsed; rejected on mesh geoms | **Not parsed** | | |
| `inertia` on `<mesh>` | Parsed; 4-value enum | **Not parsed** | | |
| `exactmeshinertia` on `<compiler>` | **Removed** from schema | Parsed and stored (no behavioral effect) | | |
| Ellipsoid volume in mass | `4/3 pi a b c` | **Missing** — falls to `_ => 0.001` in `compute_geom_mass()` | | |

**Unclosed gaps:**
{To be filled during review execution.}

---

## 2. Spec Section Compliance

### S1. MeshInertia enum and type additions

**Grade:**

**Spec says:**
Add `MeshInertia` enum (Convex=0, Exact=1, Legacy=2, Shell=3) to `types.rs`.
Add `inertia: Option<MeshInertia>` to `MjcfMesh` and `MjcfMeshDefaults`.
Add `shellinertia: Option<bool>` to `MjcfGeom` and `MjcfGeomDefaults`.
Default is `Convex`. Update all struct defaults/constructors.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Parser changes

**Grade:**

**Spec says:**
Parse `inertia` as keyword string on `<mesh>` (convex/exact/legacy/shell).
Parse `shellinertia` as boolean on `<geom>`. Parse `inertia` and
`shellinertia` on respective default elements. Add deprecation warning for
`exactmeshinertia`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. Default class inheritance

**Grade:**

**Spec says:**
Add `inertia` cascade in `apply_to_mesh()` and `merge_mesh_defaults()`.
Add `shellinertia` cascade in `apply_to_geom()` and `merge_geom_defaults()`.
Follow existing `.or()` merge pattern.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Shell mesh inertia algorithm

**Grade:**

**Spec says:**
New `compute_mesh_inertia_shell()` in `builder/mesh.rs`. Area-weighted
surface distribution per triangle. Integrals: `A/6` for diagonal,
`A/12` for cross terms. PAT shift using `total_area`. Degenerate fallback
for zero-area meshes. Returns `(total_area, com, inertia_at_com)`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. Legacy mesh inertia algorithm

**Grade:**

**Spec says:**
New `compute_mesh_inertia_legacy()` in `builder/mesh.rs`. Identical to
`compute_mesh_inertia()` (exact mode) but uses `det.abs()` instead of
signed `det`. Overcounts non-convex regions.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S6. Convex mesh inertia dispatch

**Grade:**

**Spec says:**
New `compute_mesh_inertia_on_hull()` that runs exact algorithm on convex
hull geometry. Helper `compute_inertia_from_verts_faces()` to avoid
constructing temporary `TriangleMeshData`. Falls back to
`compute_mesh_inertia()` if hull not available.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S7. Primitive shell inertia formulas

**Grade:**

**Spec says:**
Add `compute_geom_shell_mass()` and `compute_geom_shell_inertia()` in
`builder/geom.rs`. Five primitive types: sphere (I=2/3 mr^2), box (face
decomposition + PAT), cylinder (lateral + 2 caps), capsule (lateral + 2
hemisphere shells with d=h/2+r/2 PAT), ellipsoid (Thomsen SA + GL
quadrature with n_theta=64, n_phi=128). Helper functions:
`shell_box_inertia()`, `shell_cylinder_inertia()`, `shell_capsule_inertia()`,
`shell_ellipsoid_inertia()`, `ellipsoid_surface_area()`,
`gauss_legendre_01()`, `gauss_legendre_standard()`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S8. Dispatch integration

**Grade:**

**Spec says:**
S8a: Fix missing Ellipsoid arm in `compute_geom_mass()` (was `_ => 0.001`).
S8b: Shell mass dispatch when `shellinertia == Some(true)`.
S8c: Shell inertia dispatch in `compute_geom_inertia()`.
S8d: Mode-aware `MeshProps` via `compute_mesh_inertia_by_mode()` dispatch
function. Add `mesh_inertia_modes: &[MeshInertia]` parameter to
`compute_inertia_from_geoms()`. Builder populates `mesh_inertia_modes` in
`process_mesh()`.
S8e: Negative volume validation for exact mode — reject misoriented meshes.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S9. Validation: shellinertia rejection on mesh geoms

**Grade:**

**Spec says:**
Check `shellinertia` on mesh geoms during model building and return error:
"for mesh geoms, inertia should be specified in the mesh asset". Check in
`process_geom()` or `compute_geom_mass()`.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Default mesh inertia mode is Convex — unit cube mass=1000, I=[166.667, 166.667, 166.667] | T1 | | |
| AC2 | Shell mesh inertia — unit cube mass=6000, I=[1666.667, 1666.667, 1666.667] | T2 | | |
| AC3 | Shell mesh inertia — explicit mass override mass=5.0, I=[1.389, 1.389, 1.389] | T3 | | |
| AC4 | Sphere shell inertia — mass=12566.37, I=8377.58 | T4 | | |
| AC5 | Box shell inertia — mass=88000, I=[541333.33, 421333.33, 242666.67] | T5 | | |
| AC6 | Cylinder shell inertia — mass=31415.93, I=[72780.23, 72780.23, 28274.33] | T6 | | |
| AC7 | Capsule shell inertia — mass=37699.11, I=[129852.50, 129852.50, 33510.32] | T7 | | |
| AC8 | Ellipsoid shell inertia — mass=48971.93, I=[180751.03, 140683.07, 81026.34] (1% tol) | T8 | | |
| AC9 | Convex mode on non-convex mesh — L-shape mass=3500 | T9 | | |
| AC10 | Legacy mode — L-shape mass=3000, I=[1833.33, 1500.0, 833.33] | T10 | | |
| AC11 | shellinertia rejected on mesh geom — error "for mesh geoms" | T11 | | |
| AC12 | Default class inheritance for mesh inertia — shell via default, mass=6000 | T12 | | |
| AC13 | Ellipsoid solid volume fix — mass=25132.74 | T13 | | |
| AC14 | exactmeshinertia warning — model loads, warning emitted | T14 | | |
| AC15 | Existing exact inertia tests unchanged — all 10 pass | T15 (regression) | | |
| AC16 | inertia attribute parsed on mesh — same as AC2 | T2 | | |
| AC17 | Exact mode rejects misoriented mesh — "mesh volume is negative" | T18 | | |
| AC18 | Default class inheritance for shellinertia — sphere shell mass=12566.37 | T19 | | |

**Missing or failing ACs:**
{To be filled during review execution.}

---

## 4. Test Plan Completeness

> **Test numbering note:** Spec uses T1–T19. Implementation test function
> names may use `t01_*` through `t19_*` or similar naming. Map in table below.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Default mode is Convex — unit cube mass=1000 | | | AC1 |
| T2 | Shell mesh inertia — unit cube mass=6000, I=[1666.667, ...] | | | AC2, AC16 |
| T3 | Shell mesh inertia — explicit mass override mass=5.0 | | | AC3 |
| T4 | Sphere shell — mass=12566.37, I=8377.58 | | | AC4 |
| T5 | Box shell — mass=88000, I=[541333.33, 421333.33, 242666.67] | | | AC5 |
| T6 | Cylinder shell — mass=31415.93, I=[72780.23, ...] | | | AC6 |
| T7 | Capsule shell — mass=37699.11, I=[129852.50, ...] | | | AC7 |
| T8 | Ellipsoid shell — mass=48971.93 (1% tol), I=[180751, ...] | | | AC8 |
| T9 | Convex mode on non-convex mesh — L-shape mass=3500 | | | AC9 |
| T10 | Legacy mode — L-shape mass=3000, I=[1833.33, ...] | | | AC10 |
| T11 | shellinertia rejected on mesh geom — error | | | AC11 |
| T12 | Default class inheritance — mesh inertia shell, mass=6000 | | | AC12 |
| T13 | Ellipsoid solid volume fix — mass=25132.74 | | | AC13 |
| T14 | exactmeshinertia deprecation — model loads OK | | | AC14 |
| T15 | Existing exact tests regression — all 10 pass | | | AC15 |
| T18 | Exact mode rejects misoriented mesh — error | | | AC17 |
| T19 | Default class inheritance for shellinertia — sphere shell | | | AC18 |

### Supplementary Tests

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| T16 | Zero-area degenerate mesh with shell mode — tests fallback path | | |
| T17 | Mixed shell/solid body — body with one solid + one shell sphere | | |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| Zero-area degenerate mesh + shell mode | Division by zero in COM/inertia — fallback needed | | | T16 |
| Misoriented mesh with exact mode | Must reject with negative volume error | | | T18 |
| Convex mode on already-convex mesh (cube) | Must equal exact mode | | | T1 |
| Explicit mass override with shell density | Scale factor = mass/(density*area) | | | T3 |
| `shellinertia` on mesh geom | Must error (MuJoCo 3.5.0 rejects) | | | T11 |
| `shellinertia` on each of 5 primitive types | All must compute shell inertia | | | T4-T8 |
| Default class inheritance for `<mesh inertia>` | Attribute must cascade | | | T12 |
| Default class inheritance for `<geom shellinertia>` | Attribute must cascade through `apply_to_geom()` | | | T19 |
| Multi-geom body mixing shell and solid geoms | Mass aggregation must be correct | | | T17 |

**Missing tests:**
{To be filled during review execution.}

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| Default mesh inertia mode: Always Exact -> Convex | | |
| Ellipsoid volume: `_ => 0.001` -> `4/3 pi a b c` | | |
| `shellinertia` attribute: not parsed -> parsed and applied | | |
| `inertia` attribute on `<mesh>`: not parsed -> 4 modes | | |
| `exactmeshinertia`: silently parsed -> parsed + warning | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/mjcf/src/types.rs` | | |
| `sim/L0/mjcf/src/parser.rs` | | |
| `sim/L0/mjcf/src/defaults.rs` | | |
| `sim/L0/mjcf/src/builder/mesh.rs` | | |
| `sim/L0/mjcf/src/builder/geom.rs` | | |
| `sim/L0/mjcf/src/builder/mass.rs` | | |
| `sim/L0/mjcf/src/builder/mod.rs` (or body.rs) | | |
| Test file(s) | | |

{List any files changed that were NOT in the spec's prediction:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| | |

### Non-Modification Sites

Verify these files from the spec's Non-Modification Sites were NOT changed:

| File | Why NOT modified (per spec) | Actually unchanged? | Notes |
|------|---------------------------|--------------------|----|
| `sim/L0/core/src/convex_hull.rs` | Read-only consumer; Spec B calls `mesh.convex_hull()` | | |
| `sim/L0/core/src/mesh.rs:285-293` | `compute_convex_hull()` / `convex_hull()` already complete from Spec A | | |
| `sim/L0/core/src/collision_shape.rs` | `CollisionShape::ConvexMesh` — collision shapes, not related to inertia | | |
| `sim/L0/mjcf/src/builder/body.rs` | Unchanged; calls `compute_inertia_from_geoms()` which gets new parameter | | |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| `ac1_parser_default_false` (`exactmeshinertia.rs:32`) | Pass (unchanged) | | |
| `ac2a_parser_true` (`exactmeshinertia.rs:54`) | Pass + warning | | |
| `ac3_unit_cube_mass_inertia` (`exactmeshinertia.rs:88`) | Pass (unchanged) | | |
| `ac4_asymmetric_mesh` (`exactmeshinertia.rs:142`) | Pass (unchanged) | | |
| All other `exactmeshinertia.rs` tests | Pass (unchanged) | | |
| Struct literal tests in `mesh.rs` (`mesh.rs:754,779,795,824`) | **Compile error** — add `inertia: None` | | |
| Struct literal test in `geom.rs` (`geom.rs:535-559`) | **Compile error** — add `shellinertia: None` | | |

**Unexpected regressions:**
{To be filled during review execution.}

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `mjtMeshInertia` enum — C: CONVEX=0, EXACT=1, LEGACY=2, SHELL=3 | `MeshInertia` Rust enum: `Convex`, `Exact`, `Legacy`, `Shell`; match ordinals; default = `Convex` | | |
| `mjtGeomInertia` enum — C: VOLUME=0, SHELL=1 | `shellinertia: Option<bool>` on `MjcfGeom`; None = not set, Some(false) = volume, Some(true) = shell | | |
| `shellinertia` attribute name — XML verbatim, not snake_case | Rust field `shellinertia` (verbatim) | | |
| `size` for box — half-extents (a, b, c) | Direct port, no translation | | |
| `size` for cylinder/capsule — [radius, half_height] | Direct port; h = 2 * size[1] | | |
| Shell mass computation — `mass = density * surface_area` | For shell mode, `compute_geom_mass()` returns `density * SA` | | |
| `MeshProps` tuple — first element is "measure" (volume or area) | `(f64, Vector3, Matrix3)` — mode-unaware; callers use `.abs()` | | |

---

## 7. Weak Implementation Inventory

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|
| | | | | |

{To be filled during review execution. Check for TODO/FIXME/HACK, hardcoded
values, loose tolerances, missing edge-case guards, placeholder error handling,
algorithm deviations, dead code.}

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Deeply concave mesh for legacy vs exact demonstration | Out of Scope, bullet 1 | | | |
| Flex inertia (deformable body) | Out of Scope, bullet 2 | | | |
| GPU-accelerated inertia computation | Out of Scope, bullet 3 | | | |
| `exactmeshinertia` full removal (match MuJoCo 3.5.0 schema rejection) | Out of Scope, bullet 4 | | | |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| | | | | |

{To be filled during review execution.}

### Discovered During Review

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| | | | | |

{To be filled during review execution.}

### Spec Gaps Found During Implementation

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| | | | |

{To be filled during review execution.}

---

## 9. Test Coverage Summary

**Domain test results:**
```
{To be filled during review execution.}
sim-core:              N passed, 0 failed, M ignored
sim-mjcf:              N passed, 0 failed, M ignored
sim-conformance-tests: N passed, 0 failed, M ignored
Total:                 N passed, 0 failed, M ignored
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

**Overall:** {Ship / Ship after fixes / Needs rework}

**Items fixed during review:**
{To be filled during review execution.}

**Items to fix before shipping:**
{To be filled during review execution.}

**Items tracked for future work:**
{To be filled during review execution.}
