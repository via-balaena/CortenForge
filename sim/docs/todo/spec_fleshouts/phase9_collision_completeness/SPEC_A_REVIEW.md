# Spec A — Mesh Convex Hull Auto-Computation: Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_A.md`
**Implementation session(s):** Session 5
**Reviewer:** AI agent
**Date:** 2026-03-04

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
| Hull computation at build time | `MakeGraph()` in `user_mesh.cc` via Qhull library | **Not implemented** — no hull computation | | |
| Adjacency graph for GJK | `mesh_graph[]` flat int array, O(sqrt(n)) hill-climbing | **Not implemented** — O(n) exhaustive scan only | | |
| `maxhullvert` attribute | Parsed from `<mesh>`, validated >= 4 or -1, maps to Qhull `TA(N-4)` | **Not parsed** — `MjcfMesh` has no `maxhullvert` field | | |
| GJK support strategy selection | Hill-climbing for >= 10 vertices, exhaustive for <10, warm-started | **Exhaustive only** — `support_convex_mesh()` does fresh O(n) scan | | |
| Hull trigger | `needhull_` set for collision meshes and convex-inertia meshes | **N/A** — no hull system exists | | |
| Mesh-mesh collision | GJK/EPA on convex hulls | Per-triangle BVH via `mesh_mesh_deepest_contact()` | | |
| ConvexMesh construction | Hull vertices populate collision shape automatically | **Never constructed in production** — only in tests | | |

**Unclosed gaps:**

---

## 2. Spec Section Compliance

### S1. ConvexHull and HullGraph structs + Quickhull algorithm

**Grade:**

**Spec says:**
New `sim/L0/core/src/convex_hull.rs` with `ConvexHull` struct (vertices,
faces, normals, graph), `HullGraph` struct (adjacency lists), and Quickhull
algorithm: initial simplex from extremals, conflict graph, iterative
expansion with BFS horizon detection, face-neighbor tracking (no edge map),
scale-dependent epsilon, degenerate handling (`None` for < 4 verts,
collinear, coplanar). Graph built from face connectivity. Normals computed
as cross products.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Hill-climbing support + warm-start in `gjk_epa.rs`

**Grade:**

**Spec says:**
Extend `CollisionShape::ConvexMesh` with `graph: Option<HullGraph>` and
`warm_start: Cell<usize>`. Add `convex_mesh_from_hull()` factory. Update
`support_convex_mesh()` to accept graph + warm_start, dispatch to
steepest-ascent hill-climbing (>= 10 vertices with graph) or exhaustive
scan (< 10 or no graph). Hill-climbing matches `mjc_hillclimbSupport()`:
scans ALL neighbors, picks best, `max_dot` initialized to `NEG_INFINITY`.
`HILL_CLIMB_MIN = 10`. Update all match sites on ConvexMesh.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. `maxhullvert` parsing in `types.rs` + `builder/mesh.rs`

**Grade:**

**Spec says:**
Add `maxhullvert: Option<usize>` to `MjcfMesh` and `MjcfMeshDefaults`.
Parse from `<mesh>` element as integer: `-1` -> `None`, `>= 4` ->
`Some(n)`, `< 4 && != -1` -> error "maxhullvert must be larger than 3".
Default `None`. Apply defaults inheritance in `process_mesh()`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Mesh-mesh hull integration in `mesh_collide.rs`

**Grade:**

**Spec says:**
In `collide_with_mesh()` mesh-mesh path: check if both meshes have convex
hulls. If yes, construct `CollisionShape::convex_mesh_from_hull()` for each,
call `gjk_epa_contact()`, convert `GjkContact` to `Contact` via
`make_contact_from_geoms()`, early return. If either lacks hull, fall back
to per-triangle BVH (`mesh_mesh_deepest_contact`). AD-1 option (b) — hull
used only for mesh-mesh pairs, dispatch routing unchanged.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. Build pipeline integration in `builder/mesh.rs`

**Grade:**

**Spec says:**
In `process_mesh()`, after `convert_mjcf_mesh()` returns, call
`mesh_data.compute_convex_hull(mjcf_mesh.maxhullvert)` before wrapping in
`Arc::new()`. Add `convex_hull: Option<ConvexHull>` field to
`TriangleMeshData`. Add `compute_convex_hull()` and `convex_hull()` methods.
Hull computed for all meshes unconditionally (AD-3). `ConvexHull` derives
Serialize/Deserialize (not skipped like BVH). `PartialEq` unchanged (new
field excluded automatically).

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Unit cube hull: 8 vertices, 12 faces | T1 | | |
| AC2 | Tetrahedron hull: 4 vertices, 4 faces | T2 | | |
| AC3 | Interior point excluded (8 cube verts + origin -> 8 hull verts) | T3 | | |
| AC4 | `maxhullvert` limits vertex count (icosphere 42v, limit 10) | T4, T22 | | |
| AC5 | `maxhullvert=4` boundary (8 cube verts -> 4 hull verts) | T5 | | |
| AC6 | Fewer than 4 vertices -> None | T6, T21 | | |
| AC7 | Duplicate vertices handled (16 points -> 8 hull verts) | T7 | | |
| AC8 | Hill-climbing equiv exhaustive for all directions (100 random dirs) | T8 | | |
| AC9 | `maxhullvert` parsing (`maxhullvert="10"` -> `Some(10)`) | T9, T18 | | |
| AC10 | `maxhullvert` validation (`maxhullvert="2"` -> error) | T10 | | |
| AC11 | Mesh-mesh collision uses hull (two overlapping cubes) | T11 | | |
| AC12 | Hull available after build (MJCF mesh geom) | T12 | | |
| AC13 | No `unsafe` blocks in new/modified code | — (code review) | | |
| AC14 | Face normals point outward (cube hull normals) | T19 | | |
| AC15 | Graph adjacency structural validity (symmetric, min degree >= 3, no self-loops) | T23 | | |

**Missing or failing ACs:**

---

## 4. Test Plan Completeness

> **Test numbering note:** The spec uses T1..T23. The implementation may
> use `t01_*`..`t23_*` or different function names. Note the mapping when
> filling in during review execution.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Cube hull correctness: 8 verts, 12 faces, normals outward | | | |
| T2 | Tetrahedron hull: 4 verts, 4 faces, normals outward | | | |
| T3 | Interior point excluded: 9 points -> 8 hull verts | | | |
| T4 | maxhullvert limits: icosphere 42v, limit 10, valid polytope | | | |
| T5 | maxhullvert=4 boundary: 8 cube verts -> 4 hull verts | | | |
| T6 | Fewer than 4 vertices -> None | | | |
| T7 | Duplicate vertices: 16 points (8 doubled) -> 8 hull verts, 12 faces | | | |
| T8 | Hill-climbing equiv exhaustive: 100 random dirs on icosphere | | | |
| T9 | maxhullvert parsing: `maxhullvert="10"` -> `Some(10)` | | | |
| T10 | maxhullvert validation: `maxhullvert="2"` -> error | | | |
| T11 | Mesh-mesh collision via hull: two overlapping cubes, contact depth ~0.1 | | | |
| T12 | Hull available after model build | | | |

### Supplementary Tests

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| T13 | Coplanar points: 4 coplanar verts -> None | | |
| T14 | Collinear points: 4 collinear verts -> None | | |
| T15 | Large mesh: 162-vertex icosphere, graph adjacency, hill-climb equiv exhaustive | | |
| T16 | Thin sliver: near-coplanar tetrahedron (z=1e-8), should produce hull | | |
| T17 | Serde round-trip: hull preserved across serialize/deserialize | | |
| T18 | maxhullvert=-1 parse: `maxhullvert="-1"` -> None | | |
| T19 | Face normals outward: cube hull, (centroid-to-face) dot normal > 0 | | |
| T20 | Warm-start correctness: consecutive queries with similar directions | | |
| T21 | All-identical vertices: 10 copies of same point -> None, no panic | | |
| T22 | maxhullvert > vertex count: limit 100 with 8 cube verts -> full hull | | |
| T23 | Graph structural validity: symmetric adjacency, min degree >= 3, no self-loops | | |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| Fewer than 4 vertices | MuJoCo returns no hull; must not panic | | | |
| Exactly 4 vertices (tetrahedron) | Minimum valid hull; no expansion needed | | | |
| All coplanar (flat quad) | Degenerate — no 3D hull possible | | | |
| All collinear (3+ points on a line) | Degenerate — no hull | | | |
| Duplicate vertices | Must not produce invalid hull | | | |
| Already-convex mesh (all on hull) | All vertices should be hull vertices | | | |
| `maxhullvert = 4` (boundary) | Maps to zero expansion iterations | | | |
| `maxhullvert = -1` in MJCF | Stored as `None` (no limit) | | | |
| Large mesh (>100 vertices) | Tests algorithm at scale; hill-climbing perf | | | |
| Near-degenerate (thin sliver) | Thin tetrahedron with nearly-coplanar vertices | | | |
| Face normals outward | All normals point away from hull interior | | | |
| Warm-start correctness | Cache updated between consecutive queries | | | |
| Visual-only mesh | N/A per AD-3; all meshes get hulls unconditionally | | | |
| Graph adjacency structural | Graph must be symmetric, no self-loops, min degree >= 3 | | | |
| All-identical vertices | 10 copies of same point — must not panic, return None | | | |
| `maxhullvert` > vertex count | Limit never reached — should produce full hull | | | |
| Serde round-trip | Hull preserved across serialize/deserialize | | | |

**Missing tests:**

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| `CollisionShape::ConvexMesh` gains `graph` and `warm_start` fields | | |
| Mesh-mesh collision path: per-triangle BVH -> GJK/EPA on hulls (when both have hulls) | | |
| `TriangleMeshData` gains `convex_hull: Option<ConvexHull>` field | | |
| GJK support for ConvexMesh: O(n) exhaustive -> hill-climbing O(sqrt(n)) for >= 10 verts | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/core/src/convex_hull.rs` (new) | | |
| `sim/L0/core/src/lib.rs` | | |
| `sim/L0/core/src/mesh.rs` | | |
| `sim/L0/core/src/collision_shape.rs` | | |
| `sim/L0/core/src/gjk_epa.rs` | | |
| `sim/L0/core/src/raycast.rs` | | |
| `sim/L0/core/src/collision/mesh_collide.rs` | | |
| `sim/L0/mjcf/src/types.rs` | | |
| `sim/L0/mjcf/src/parser.rs` (or equivalent mesh parser) | | |
| `sim/L0/mjcf/src/builder/mesh.rs` | | |
| `sim/L1/bevy/src/mesh.rs` | | |
| `sim/L0/core/tests/` (new test file) | | |

{List any files changed that were NOT in the spec's prediction:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|

### Non-Modification Sites: Verified Untouched

| File:line | What it does | Why NOT modified | Actually untouched? | Notes |
|-----------|-------------|-----------------|---------------------|-------|
| `collision_shape.rs:348-358` (`is_convex`) | Returns true for ConvexMesh | Uses `..` pattern — no field access | | |
| `collision_shape.rs:418-422` (`is_mesh_based`) | Returns true for ConvexMesh | Uses `..` pattern | | |
| `bevy/components.rs:82` | Maps ConvexMesh to enum variant | Uses `..` pattern | | |
| `collision/narrow.rs:87-88` | Early return for Mesh geoms | Dispatch routing unchanged (AD-1 option b) | | |
| `collision/narrow.rs:222` | `geom_to_collision_shape` returns None for Mesh | Unchanged (AD-1 option b) | | |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| `test_convex_mesh_creation` (`collision_shape.rs`) | Pass unchanged — factory defaults graph to None | | |
| `test_bounding_radius_all_types` (ConvexMesh arm) | Pass unchanged — pattern changed to `{ vertices, .. }` | | |
| `test_local_aabb_convex_mesh` | Pass unchanged — pattern changed to `{ vertices, .. }` | | |
| `test_is_mesh_based` | Pass unchanged — `matches!` with `..` pattern | | |
| GJK/EPA tests using ConvexMesh | Pass unchanged — construct via `convex_mesh()`, exhaustive path | | |
| Predicate tests (`is_convex`, `is_mesh_based`) | Pass unchanged — `{ .. }` pattern | | |
| Mesh-mesh collision tests (7 tests in `mesh.rs`) | Pass unchanged — call `mesh_mesh_contact()`, not `collide_with_mesh()` | | |

**Unexpected regressions:**

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| Graph storage: MuJoCo flat `int*` vs CortenForge `HullGraph { adjacency: Vec<Vec<usize>> }` | Use adjacency list per vertex instead of flat array with `edgeadr` offsets | | |
| Vertex storage: `float*` vs `Vec<Point3<f64>>` on ConvexHull | Direct — same data, different container | | |
| Hull presence: `mesh_graphadr[i] != -1` vs `Option<ConvexHull>` | Check `Option::is_some()` instead of `graphadr != -1` | | |
| Mesh-to-geom: `geom_dataid[g]` vs `model.geom_mesh[g]: Option<usize>` | Use `.expect()` or `?` instead of raw index | | |
| Support direction frame: rotation matrix vs UnitQuaternion inverse | Direct port — both achieve world-to-local rotation | | |
| Vertex warm-start: `vertindex`/`meshindex` on `mjCCDObj` vs `Cell<usize>` on `ConvexMesh` | `convex_mesh()` initializes `Cell::new(0)`. MuJoCo initializes to -1 (no cache); both start from vertex 0 on first call. | | |
| Face normals in graph: NOT stored in MuJoCo vs precomputed `normals: Vec<Vector3<f64>>` | CortenForge convenience — needed by Spec B and contact computation | | |
| `maxhullvert` type: `int` default `-1` vs `Option<usize>` None = no limit | Parse XML as `i32`, convert: `-1` -> `None`, `>= 4` -> `Some(n as usize)`, `0..3` -> error | | |

---

## 7. Weak Implementation Inventory

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Mesh-primitive dispatch routing to GJK/EPA (AD-1 option a) | Out of Scope, bullet 1 | | | |
| Collision-only hull trigger matching `needhull_` (AD-3 option A chosen) | Out of Scope, bullet 2 | | | |
| Non-convex mesh-mesh collision (per-triangle BVH fallback) | Out of Scope, bullet 3 | | | |
| Vertex deduplication in `TriangleMeshData` | Out of Scope, bullet 4 | | | |
| GPU convex hull computation | Out of Scope, bullet 5 | | | |

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
