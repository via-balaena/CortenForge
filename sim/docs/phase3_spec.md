# Phase 3 Spec — Extract Collision into Module Tree

> **Branch**: `refactor/structural-decompose`
> **Session**: S11
> **Baseline**: 2,007 passed / 0 failed / 20 ignored (11-crate scope)
> **Estimated extraction**: ~2,700 lines (9 files)

---

## Goal

Extract all collision dispatch, narrow-phase primitives, and contact parameter
mixing from `mujoco_pipeline.rs` into `src/collision/` — 9 new files. Zero
physics changes, zero API changes.

---

## Current State (post-Phase 4/5 line numbers)

All collision code lives in `mujoco_pipeline.rs` lines **1398–3950**.
The low-level algorithms (GJK/EPA, mesh BVH, heightfield, SDF) already live
in standalone crates (`gjk_epa.rs`, `mesh.rs`, `heightfield.rs`, `sdf.rs`).
What remains is the **dispatch layer** — the code that decides *which*
algorithm to call and *how* to combine contact parameters.

---

## Extraction Plan (9 sub-modules)

### Sub-module 1: `collision/mod.rs` (~465 lines)

**One-sentence responsibility**: Top-level collision detection dispatch
(broad-phase → narrow-phase) and contact parameter mixing.

**Functions to move** (in order):

| Function | Current lines | Visibility |
|----------|--------------|------------|
| `check_collision_affinity` | L1398–L1443 | `pub(crate)` |
| `mj_collision` | L1469–L1591 | `pub(crate)` |
| `mj_collision_flex` | L1599–L1653 | `pub(crate)` |
| `contact_param` | L2092–L2145 | `pub(crate)` |
| `contact_param_flex_rigid` | L2152–L2204 | `pub(crate)` |
| `solmix_weight` | L2208–L2219 | private |
| `combine_solref` | L2224–L2233 | private |
| `combine_solimp` | L2236–L2238 | private |

**`//!` doc**: `//! Collision detection pipeline — broad-phase dispatch, affinity
filtering, and contact parameter mixing.`

**Sub-module declarations**:
```rust
pub(crate) mod flex_collide;
pub(crate) mod hfield;
pub(crate) mod mesh_collide;
pub(crate) mod narrow;
pub(crate) mod pair_convex;
pub(crate) mod pair_cylinder;
pub(crate) mod plane;
pub(crate) mod sdf_collide;
```

**Re-exports from mod.rs**:
```rust
pub(crate) use narrow::{collide_geoms, geom_to_collision_shape};
pub(crate) use flex_collide::{narrowphase_sphere_geom, make_contact_flex_rigid};
```
(Other sub-module functions are called only from within collision/ or from the
monolith's forward pipeline via mod.rs dispatch — no re-exports needed.)

**Monolith dependencies** (functions mod.rs will call that are still in the
monolith): The `mj_collision` function calls `mj_fwd_position`-adjacent SAP
broadphase results already stored in `data`. It calls `collide_geoms` (moving
to `narrow.rs` in the same phase). No monolith function imports needed once
all 9 sub-modules are extracted.

**Note**: `mj_collision` and `mj_collision_flex` call functions in `narrow.rs`
and `flex_collide.rs`. Extract `narrow.rs` and `flex_collide.rs` **before**
or **at the same time as** mod.rs to avoid temporary forward references.
Strategy: create the mod.rs shell first (step 0 of the protocol), then extract
the leaf sub-modules first, then populate mod.rs last.

---

### Sub-module 2: `collision/narrow.rs` (~272 lines)

**One-sentence responsibility**: Narrow-phase collision dispatch — selects
analytical or GJK/EPA path for a geom pair.

**Functions to move**:

| Function | Current lines | Visibility |
|----------|--------------|------------|
| `collide_geoms` | L1863–L2002 | `pub(crate)` |
| `geom_to_collision_shape` | L2006–L2027 | `pub(crate)` (imported by sensor/position.rs) |
| `apply_pair_overrides` | L2060–L2084 | `pub(crate)` |
| `make_contact_from_geoms` | L2246–L2283 | `pub(crate)` |
| Constants: `GEOM_EPSILON`, `AXIS_VERTICAL_THRESHOLD`, `AXIS_HORIZONTAL_THRESHOLD`, `CAP_COLLISION_THRESHOLD` | L2294–L2306 | `pub(crate)` |

**`//!` doc**: `//! Narrow-phase collision dispatch — selects analytical or
GJK/EPA path for a geom pair.`

**Cross-module import update**: `sensor/position.rs:8` currently imports
`use crate::mujoco_pipeline::geom_to_collision_shape`. This must be updated
to `use crate::collision::geom_to_collision_shape` (via mod.rs re-export).

**Monolith dependencies**: `collide_geoms` calls functions in `plane.rs`,
`pair_convex.rs`, `pair_cylinder.rs`, `mesh_collide.rs`, `hfield.rs`,
`sdf_collide.rs`. All are extracted in this same phase. During the per-item
loop, use `use crate::mujoco_pipeline::{collide_with_plane, ...}` with
`// monolith: removed later in Phase 3` annotations. After all sub-modules
are extracted, replace with `use super::plane::collide_with_plane` etc.

---

### Sub-module 3: `collision/flex_collide.rs` (~202 lines)

**One-sentence responsibility**: Flex-rigid collision helpers — sphere-geom
narrowphase and contact creation for flex vertices.

**Functions to move**:

| Function | Current lines | Visibility |
|----------|--------------|------------|
| `narrowphase_sphere_geom` | L1659–L1810 | `pub(crate)` |
| `make_contact_flex_rigid` | L1816–L1857 | `pub(crate)` |

**`//!` doc**: `//! Flex-rigid collision helpers — narrowphase for flex vertex
spheres against rigid geoms.`

---

### Sub-module 4: `collision/pair_convex.rs` (~310 lines)

**One-sentence responsibility**: Analytical pairwise collision for convex
primitives (sphere, capsule, box).

**Functions to move**:

| Function | Current lines | Visibility |
|----------|--------------|------------|
| `collide_sphere_sphere` | L3113–L3165 | `pub(crate)` |
| `collide_capsule_capsule` | L3166–L3228 | `pub(crate)` |
| `collide_sphere_capsule` | L3229–L3318 | `pub(crate)` |
| `collide_sphere_box` | L3319–L3423 | `pub(crate)` |

**`//!` doc**: `//! Analytical pairwise collision for convex primitives
(sphere, capsule, box).`

---

### Sub-module 5: `collision/pair_cylinder.rs` (~620 lines)

**One-sentence responsibility**: Analytical pairwise collision for cylinders
and box-box SAT.

**Functions to move**:

| Function | Current lines | Visibility |
|----------|--------------|------------|
| `collide_cylinder_sphere` | L3424–L3553 | `pub(crate)` |
| `collide_cylinder_capsule` | L3554–L3653 | `pub(crate)` |
| `collide_capsule_box` | L3654–L3798 | `pub(crate)` |
| `collide_box_box` | L3799–L3928 | `pub(crate)` |
| `test_sat_axis` | L3932–~L3950 | private |

**`//!` doc**: `//! Analytical pairwise collision for cylinders and box-box
SAT.`

**MARGIN CHECK**: ~620 lines of production code. With `use` imports and `//!`
doc, expect ~640. Well under 800.

---

### Sub-module 6: `collision/plane.rs` (~415 lines)

**One-sentence responsibility**: Collision detection against infinite planes.

**Functions to move**:

| Function | Current lines | Visibility |
|----------|--------------|------------|
| `collide_with_plane` | L2698–L2898 | `pub(crate)` |
| `collide_cylinder_plane_impl` | L2922–L3015 | private |
| `collide_ellipsoid_plane_impl` | L3047–L3106 | private |

**`//!` doc**: `//! Collision detection against infinite planes (sphere, box,
capsule, cylinder, ellipsoid).`

**Note**: `collide_with_plane` uses the constants from `narrow.rs`
(`AXIS_VERTICAL_THRESHOLD`, `AXIS_HORIZONTAL_THRESHOLD`, `CAP_COLLISION_THRESHOLD`).
Import via `use super::narrow::{...}`.

---

### Sub-module 7: `collision/mesh_collide.rs` (~174 lines)

**One-sentence responsibility**: Mesh collision dispatch — routes mesh-geom
pairs to the standalone `crate::mesh` library.

**Functions to move**:

| Function | Current lines | Visibility |
|----------|--------------|------------|
| `collide_with_mesh` | L2524–L2642 | `pub(crate)` |
| `collide_mesh_plane` | L2651–L2689 | `pub(crate)` |

**`//!` doc**: `//! Mesh collision dispatch — routes mesh-geom pairs to the
standalone mesh library.`

---

### Sub-module 8: `collision/hfield.rs` (~87 lines)

**One-sentence responsibility**: Height field collision dispatch.

**Functions to move**:

| Function | Current lines | Visibility |
|----------|--------------|------------|
| `collide_with_hfield` | L2315–L2394 | `pub(crate)` |

**`//!` doc**: `//! Height field collision dispatch — routes geom-hfield pairs
to the standalone heightfield library.`

---

### Sub-module 9: `collision/sdf_collide.rs` (~119 lines)

**One-sentence responsibility**: SDF collision dispatch.

**Functions to move**:

| Function | Current lines | Visibility |
|----------|--------------|------------|
| `collide_with_sdf` | L2401–L2514 | `pub(crate)` |

**`//!` doc**: `//! SDF collision dispatch — routes geom-SDF pairs to the
standalone SDF library.`

---

## Extraction Order

Handles a circular dependency: `narrow.rs` and `flex_collide.rs` call
`contact_param`/`contact_param_flex_rigid` (in mod.rs), while mod.rs's
`mj_collision` calls `collide_geoms` (in narrow.rs). Solution: populate mod.rs
in two passes — standalone functions first, dispatch functions last.

**Step 0**: Create `collision/mod.rs` shell (sub-module declarations + `//!` doc)
  and add `pub mod collision;` to `lib.rs`. `cargo check`.

**Pass 1 — Standalone functions into mod.rs**:
1. `check_collision_affinity` → mod.rs
2. `contact_param`, `contact_param_flex_rigid`, `solmix_weight`,
   `combine_solref`, `combine_solimp` → mod.rs
   (These have zero collision-internal dependencies.)

**Pass 2 — Leaf sub-modules** (no intra-collision callers):
3. `collision/hfield.rs` — smallest, standalone
4. `collision/sdf_collide.rs` — standalone
5. `collision/mesh_collide.rs` — standalone
6. `collision/pair_convex.rs` — standalone
7. `collision/pair_cylinder.rs` — standalone
8. `collision/plane.rs` — uses constants from monolith temporarily
   (`use crate::mujoco_pipeline::GEOM_EPSILON; // monolith: removed later in Phase 3`)

**Pass 3 — Dispatch sub-modules** (call leaf modules + mod.rs functions):
9. `collision/flex_collide.rs` — calls `super::contact_param_flex_rigid`
10. `collision/narrow.rs` — calls all leaf sub-modules + `super::contact_param`
    Constants (GEOM_EPSILON, etc.) extracted here; update plane.rs imports
    from monolith → `super::narrow::GEOM_EPSILON`.

**Pass 4 — Dispatch functions into mod.rs**:
11. `mj_collision`, `mj_collision_flex` → mod.rs
    (Call `narrow::collide_geoms` and `flex_collide::narrowphase_sphere_geom`,
    both now in their final locations.)

**Rationale**: Move callees before callers. The two-pass mod.rs strategy
avoids temporary monolith imports within `collision/` entirely — every
sub-module can import from its final locations on first write.

---

## Inline Test Blocks to Move

| Test module | Current line | Moves to | Helper functions |
|------------|-------------|----------|-----------------|
| `primitive_collision_tests` | L14732 | `collision/pair_convex.rs` (sphere/capsule/box tests) or `collision/pair_cylinder.rs` (cylinder/box-box tests) — split by function tested | `make_collision_test_model` (L14737) |
| `contact_param_tests` | L17983 | `collision/mod.rs` | `make_two_geom_model` (L17988) |

**Decision**: If `primitive_collision_tests` tests functions across both
`pair_convex.rs` and `pair_cylinder.rs`, move the entire block to the file
containing the majority of tested functions, or split it. Inspect the actual
test contents before deciding.

---

## Import Updates (per "every phase includes")

### sensor/position.rs
```
- use crate::mujoco_pipeline::geom_to_collision_shape;
+ use crate::collision::geom_to_collision_shape;
```

### lib.rs
Add module declaration:
```rust
// Collision detection pipeline (dispatch, narrow-phase, contact params)
pub(crate) mod collision;
```

No public API symbols move (none of these functions are in the `pub use
mujoco_pipeline::{...}` re-export block). Use `pub mod collision;` (not
`pub(crate)`) to match the existing convention for extracted modules.

### Monolith re-imports
Add to `mujoco_pipeline.rs` top (after existing re-imports):
```rust
// Re-imports from collision module (Phase 3 extraction, removed in Phase 12).
pub(crate) use crate::collision::check_collision_affinity;
pub(crate) use crate::collision::collide_geoms;
pub(crate) use crate::collision::contact_param;
pub(crate) use crate::collision::contact_param_flex_rigid;
pub(crate) use crate::collision::geom_to_collision_shape;
pub(crate) use crate::collision::make_contact_flex_rigid;
pub(crate) use crate::collision::mj_collision;
pub(crate) use crate::collision::mj_collision_flex;
pub(crate) use crate::collision::narrowphase_sphere_geom;
// Sub-module functions (only needed if monolith still calls them directly):
pub(crate) use crate::collision::narrow::apply_pair_overrides;
pub(crate) use crate::collision::narrow::make_contact_from_geoms;
pub(crate) use crate::collision::narrow::GEOM_EPSILON;
// (plane, pair_convex, pair_cylinder, mesh_collide, hfield, sdf_collide
//  functions are called only from within collision/ — no monolith re-imports needed)
```

### derivatives.rs / batch.rs
Check for any imports of collision-related symbols from the monolith.
(Expected: none — collision functions are internal to the pipeline.)

---

## Lazy Import Check

After all 9 sub-modules are extracted and monolith re-imports are added:

1. Comment out all `pub(crate) use crate::collision::*` re-imports in the monolith
2. `cargo check -p sim-core`
3. Expect errors in `mujoco_pipeline.rs` only
4. Any errors in other files → fix those imports to point to `crate::collision::*`
5. Uncomment re-imports

---

## Verification

```bash
cargo clippy -p sim-core -- -D warnings
cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics \
  -p sim-constraint -p sim-muscle -p sim-tendon -p sim-sensor -p sim-urdf \
  -p sim-types -p sim-simd
```

**Expected**: 2,007 passed / 0 failed / 20 ignored

---

## S1–S8 Pre-Assessment

| # | Criterion | Expected grade | Notes |
|---|-----------|---------------|-------|
| S1 | Module Size | A | Largest: pair_cylinder.rs ~640 lines (under 800) |
| S2 | Naming | A | Names match MuJoCo C file boundaries (engine_collision_*.c) |
| S3 | Single Responsibility | A | Each file = one collision category |
| S4 | Dependency Direction | A | collision/ depends on types/ + standalone libs; nothing depends on collision/ except forward/ |
| S5 | mod.rs Clarity | A | mod.rs = broad-phase dispatch + param mixing; sub-modules = narrow-phase implementations |
| S6 | Reference Completeness | A | sensor/position.rs import updated; lazy import check catches any others |
| S7 | Discoverability | A | `collision/` → `narrow.rs` or `pair_convex.rs` by name alone |
| S8 | Test Colocation | A | primitive_collision_tests + contact_param_tests move with their code |
