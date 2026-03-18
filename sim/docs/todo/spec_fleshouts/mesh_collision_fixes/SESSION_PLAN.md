# Mesh Collision Fixes — Session Plan

Source spec: `SPEC.md` (DT-179 + DT-180)

## Session 1: Profile DT-179 — Mesh-Plane Collision Performance ✅

**Status:** Complete
**Test:** `sim/L0/tests/integration/mesh_collision_profile.rs`

### Findings

1. **AABB bloat confirmed.** `MESH_DEFAULT_EXTENT = 10.0` in
   `forward/position.rs:343` creates 20m × 20m × 20m bounding boxes for every
   mesh geom, regardless of actual mesh size. `geom_size` for mesh geoms is
   [0.1, 0.1, 0.1] (default from `geom_size_to_vec3` catch-all) and is unused
   by `aabb_from_geom`.

2. **step1 (FK+collision) is the hot path.** For mesh scenes, step1 accounts
   for 51-94% of step time. For primitive scenes, step1 is only 6-19% (step2
   dominates with constraint assembly + solver).

3. **O(N²) broadphase pairs confirmed.** With N mesh geoms spaced 5m apart
   (far beyond any physical overlap), 10m AABBs cause ALL C(N,2) mesh-mesh
   pairs to pass broadphase:

   | N geoms | Tight pairs | Bloat pairs | Mesh step1 | Prim step1 |
   |---------|-------------|-------------|------------|------------|
   | 2       | 2           | 3           | 7.7µs      | 1.6µs      |
   | 5       | 5           | 15          | 34.5µs     | 2.5µs      |
   | 10      | 10          | 55          | 77.8µs     | 5.0µs      |
   | 15      | 15          | 120         | 124.6µs    | 9.4µs      |

   Mesh step1 grows quadratically; primitive step1 grows linearly.

4. **Mesh/prim ratio is ~2-3× per step** (not 40,000× as spec claims). Each
   false mesh-mesh narrowphase call costs ~3-5µs (GJK/EPA on 42-vertex convex
   hull + `UnitQuaternion::from_matrix` conversion). The per-pair cost is
   modest, but it compounds quadratically with geom count.

5. **The 40,000× from the spec is not reproducible** with 42-162 vertex meshes.
   The original measurement likely used much denser meshes from cf-design's
   adaptive DC mesher, or measured a different scenario.

### Fix for Session 3

Replace `MESH_DEFAULT_EXTENT` with tight AABB computed from transformed mesh
vertices. The mesh `TriangleMeshData` already stores a local-space AABB. The
fix needs to transform it by the geom's world-space pose (pos + rotation
matrix). This requires access to `model.mesh_data` inside `aabb_from_geom`,
which currently only takes `(geom_type, size, pos, mat)`. Options:

- **Option A:** Pass `Option<&Aabb>` for mesh local AABB into `aabb_from_geom`
- **Option B:** Pre-compute mesh geom AABBs in `mj_collision` using mesh_data,
  before building the SAP structure
- **Option C:** Store pre-computed mesh AABBs (in geom-local space) on Model
  at build time, accessible via `model.geom_aabb[geom_id]`

Option B is simplest (localized change in `mj_collision`). Option C is cleanest
(separates build-time and runtime concerns). Session 3 should decide.

---

## Session 2: Diagnose DT-180 — Mesh Contact Force Magnitude

**Status:** Not started
**Independent of Session 1.**

---

## Session 3: Fix DT-179

**Status:** Not started
**Entry:** Session 1 findings (above)

---

## Session 4: Fix DT-180

**Status:** Not started

---

## Session 5: cf-design Integration + Spec Completion

**Status:** Not started
