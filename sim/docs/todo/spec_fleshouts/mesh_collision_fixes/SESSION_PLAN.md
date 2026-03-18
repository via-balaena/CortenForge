# Mesh Collision Fixes — Session Plan

Source spec: `SPEC.md` (DT-179 + DT-180)

## Session 1: Profile + Fix DT-179 — Mesh AABB Bloat ✅

**Status:** Complete
**Test:** `sim/L0/tests/integration/mesh_collision_profile.rs`

### Root cause

`aabb_from_geom()` was a runtime type-dispatch function that reconstructed
bounding boxes from `(geom_type, size, pos, mat)`. For meshes, `size` doesn't
encode mesh geometry, so it used `MESH_DEFAULT_EXTENT = 10.0` — a 20m³
bounding box for every mesh geom regardless of actual size.

### Fix

Pre-computed local-frame AABBs for all geoms at build time, stored as
`model.geom_aabb: Vec<[f64; 6]>` (center offset + half-extents). At runtime,
`mj_collision` transforms these to world space via the standard rotated-box
formula — no type dispatch, no mesh vertex iteration.

- `model.geom_aabb` populated in `compute_geom_bounding_radii()` alongside
  `geom_rbound`, using the same mesh/hfield/sdf/primitive dispatch
- `aabb_from_geom_aabb()` replaces `aabb_from_geom()` — uniform for all types
- `MESH_DEFAULT_EXTENT` deleted
- Matches MuJoCo's `mjModel.geom_aabb` architecture

### Results (release, Apple M-series)

Collision phase (step1) with N well-separated mesh geoms + plane:

| N | Before (mesh step1) | After (mesh step1) | Before ratio | After ratio |
|---|---|---|---|---|
| 5 | 28.9µs | 4.3µs | 13.8× prim | 1.5× prim |
| 10 | 77.8µs | 6.9µs | 15.6× prim | 1.3× prim |
| 15 | 124.6µs | 12.5µs | 13.3× prim | 1.2× prim |

False broadphase pairs eliminated. Mesh collision cost now matches primitives.

---

## Session 2: Diagnose DT-180 — Mesh Contact Force Magnitude

**Status:** Not started
**Independent of Session 1.**

---

## Session 3: Fix DT-180

**Status:** Not started (was originally the DT-179 fix session — DT-179 fix
landed in Session 1 instead)

---

## Session 4: cf-design Integration + Spec Completion

**Status:** Not started
