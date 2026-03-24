# Convex Multi-Contact for Analytical Shapes

**Status**: COMPLETE
**Motivation**: examples/sdf-physics/08-stack — cubes are unstable

## 1. Problem

Three 10mm cubes (AnalyticalShape, ShapeHint::Convex) stacked on a
ground plane should settle into a stable tower. Instead, cubes slide
and topple. The visual mesh shows sharp edges, but the physics
behaves as if the cubes have no face-face contact stability.

## 2. Root Cause

**Misdiagnosis**: The initial hypothesis was grid rounding in Tier 3
surface tracing (smooth edges → cubes roll). This is wrong.

**Actual cause**: The plane contact dispatch gives non-spherical convex
shapes a single contact point. A cube face on a floor gets 1 contact
instead of 4+ distributed contacts. One contact provides zero torque
resistance — any lateral force tips the cube with no restoring moment.

### 2.1 Dispatch Trace for 08-stack

All three cubes are `AnalyticalShape` with `ShapeHint::Convex`.

**Cuboid-plane** (`compute_shape_plane_contact` in `shape.rs:191`):
1. Line 201: `shape.effective_radius(&local_dir)` → calls `ray_march_solid()`
   → returns `Some(t)` for all convex shapes
2. Tier 1 (single contact) is taken → **1 contact per face**
3. A cube face on a floor gets no distributed contact patch
4. No torque resistance → cube tips over from any perturbation

**Cuboid-cuboid** (`compute_shape_contact` in `shape.rs:118`):
1. Line 135: `prefers_single_contact()` → `false` (not Sphere) → skip Tier 1
2. Line 156: `evaluate_interval()` → `Some(lo, hi)` (cuboid intervals are tight)
   → **Tier 2 (octree) is taken**
3. Octree already uses exact evaluation (`shape.distance()` / `shape.gradient()`)
4. Produces distributed multi-contact across face overlap → **already correct**

**Conclusion**: Cuboid-cuboid is fine (Tier 2, exact). Cuboid-plane is
broken (Tier 1, single contact). The grid is never even queried for
these shapes.

### 2.2 Why the Dispatch is Asymmetric

`compute_shape_contact` gates Tier 1 on `prefers_single_contact()`:
```rust
if a.prefers_single_contact() && b.prefers_single_contact() {
    if let (Some(r_a), Some(r_b)) = (a.effective_radius(...), b.effective_radius(...)) {
        // single contact
    }
}
```

`compute_shape_plane_contact` gates Tier 1 on `effective_radius()` alone:
```rust
if let Some(radius) = shape.effective_radius(&local_dir) {
    // single contact
}
```

The plane dispatcher treats all convex shapes as sphere-like. Cuboids,
cylinders, and other non-spherical convex shapes need multi-contact
against planes for rotational stability — the same reason
`compute_shape_contact` checks `prefers_single_contact()`.

### 2.3 Same Bug Affects MJCF Shapes

`ShapeConvex` also returns `Some` from `effective_radius()` (ray-marches
the grid). So MJCF-imported convex shapes also get single-contact plane
dispatch. This is wrong for any non-spherical MJCF convex shape, but
was masked because most MJCF examples use MuJoCo primitives (sphere,
capsule) which have their own contact functions in `primitives.rs`.

## 3. Design Principle

Fix the dispatch gate first. Then improve Tier 3 as a secondary change.

No new collision algorithms. The existing octree and surface tracing
algorithms are correct. The bug is in the dispatch routing, not the
algorithms themselves.

## 4. What Changes

### 4.1 Primary Fix: Plane Dispatch Gate (`shape.rs`)

**Change**: Gate Tier 1 plane contact on `prefers_single_contact()`,
matching the SDF-SDF dispatch.

```rust
// Before (shape.rs:200–213):
if let Some(radius) = shape.effective_radius(&local_dir) {
    // Tier 1: single contact for ALL convex shapes
    ...
}

// After:
if shape.prefers_single_contact() {
    if let Some(radius) = shape.effective_radius(&local_dir) {
        // Tier 1: single contact ONLY for spheres
        ...
    }
}
```

**Dispatch after fix:**

| Shape | `prefers_single_contact` | `effective_radius` | `evaluate_interval` | Tier |
|-------|--------------------------|--------------------|--------------------|------|
| AnalyticalShape Sphere | true | Some(r) | Some | **1** (single, correct) |
| AnalyticalShape Cuboid | false | Some(t) | Some | **2** (octree, multi-contact) |
| AnalyticalShape Concave | false | None | varies | **2 or 3** (multi-contact) |
| ShapeSphere | true | Some(r) | Some | **1** (single, correct) |
| ShapeConvex | false | Some(t) | None | **3** (grid, multi-contact) |
| ShapeConcave | false | None | None | **3** (grid, multi-contact) |

Every row either takes Tier 1 (sphere — correct as single contact) or
multi-contact (Tier 2/3). Non-spherical convex shapes correctly skip
to distributed contact paths.

### 4.2 Verify Octree Plane Detection Quality

After the dispatch fix, AnalyticalShape cuboids take Tier 2
(`octree_plane_detect`) for plane contacts. This path already exists
and uses exact evaluation. Verify it produces adequate contacts:

- `octree_plane_detect` is called with `max_contacts = 8` (line 218)
- For a cube face on a plane, leaf cells cover the bottom face
- Newton iteration projects seed points onto the cuboid surface
- Dedup + `select_representatives` distributes contacts spatially

**Test**: An AnalyticalShape cuboid at rest on a plane must produce
≥ 3 contacts distributed across the face (at least 3 for planar
stability). Verify the contacts span the face width.

### 4.3 Secondary: Thread `PhysicsShape` into Tier 3 (`operations.rs`)

This is a maintenance improvement, not the primary fix. It benefits:

- **ShapeConvex cuboid-plane**: After §4.1, ShapeConvex cuboids fall
  to Tier 3 (`sdf_plane_contact`). This path currently uses
  `sdf.gradient()` (grid finite differences). Threading PhysicsShape
  would let exact shapes use `shape.gradient()` — but ShapeConvex
  delegates to the grid anyway, so it's a no-op for this case.

- **Mixed AnalyticalShape + ShapeConvex pairs**: One shape has exact
  eval, the other doesn't. Both lack `evaluate_interval()` on one
  side → Tier 2 unavailable → Tier 3. Threading PhysicsShape gives
  exact evaluation on the analytical side.

- **Future-proofing**: If `evaluate_interval()` becomes too loose
  for a specific CSG tree (the span > 10× diagonal guard in
  `AnalyticalShape`), the shape falls to Tier 3. With PhysicsShape
  threading, Tier 3 still gets exact evaluation.

#### 4.3.1 `trace_surface_into_other` — new signature

```rust
fn trace_surface_into_other(
    src_grid: &SdfGrid,            // cell iteration (spatial indexing)
    src_shape: &dyn PhysicsShape,  // distance/gradient queries
    src_pose: &Pose,
    dst_grid: &SdfGrid,            // (unused but symmetric)
    dst_shape: &dyn PhysicsShape,  // distance/gradient queries
    dst_pose: &Pose,
    margin: f64,
    contacts: &mut Vec<SdfContact>,
    flip_normal: bool,
)
```

Query changes inside the function:

| Role | Currently | After |
|------|-----------|-------|
| Surface threshold | `src_sdf.get(x, y, z)` | `src_grid.get(x, y, z)` (same — grid broadphase) |
| Source gradient | `src_sdf.gradient(local_point)` | `src_shape.gradient(&local_point)` |
| Source distance (Newton correction) | `src_value` (raw grid vertex) | `src_shape.distance(&local_point).unwrap_or(src_value)` |
| Destination distance | `dst_sdf.distance(dst_local)` | `dst_shape.distance(&dst_local)` |
| Destination gradient | `dst_sdf.gradient(dst_local)` | `dst_shape.gradient(&dst_local)` |

**Why keep `src_grid.get()` for threshold**: Broadphase filter — skips
deep interior cells. Grid approximation is fine for this. Avoids
calling `shape.distance()` on all N³ cells.

**Newton correction consistency**: Currently line 185 mixes a raw grid
vertex value (`src_value` from `get(x,y,z)`) with an interpolated
gradient (`src_sdf.gradient()`). These are semantically inconsistent:
`get()` returns the uninterpolated value at the grid vertex, while
`gradient()` uses centered finite differences on interpolated values.
After the change, both come from the same source (the shape), restoring
consistency.

#### 4.3.2 Public function signatures

`sdf_sdf_contact_raw_split`, `sdf_sdf_contact_raw`, `sdf_sdf_contact`
all change from `&SdfGrid` to `&dyn PhysicsShape`. Grids extracted
internally via `shape.sdf_grid()`.

#### 4.3.3 `sdf_plane_contact` (`primitives.rs`)

Same pattern: takes `&dyn PhysicsShape`, extracts grid for cell
iteration, delegates distance/gradient to shape.

#### 4.3.4 Caller Migration (27+ callers)

The public `sdf_sdf_contact` function has 19 callers. `sdf_plane_contact`
has 9 callers. All currently pass `&SdfGrid` directly.

| Location | Count | Migration |
|----------|-------|-----------|
| `operations.rs` unit tests | 15 | Wrap in `ShapeConvex::new(Arc::new(grid))` |
| `primitives.rs` unit tests | 8 | Same |
| `sim-gpu/collision.rs` tests | 2 | Same |
| `cf-design/model_builder.rs` tests | 2 | Same |
| `shape.rs` Tier 3 call sites | 2 | Pass shape directly (§4.3 goal) |

All test callers use raw `SdfGrid` without a `PhysicsShape`. The
mechanical fix is wrapping each grid: `ShapeConvex::new(Arc::new(sdf))`.
Since `ShapeConvex::distance/gradient` delegates to the grid, behavior
is identical — these tests don't exercise exact evaluation.

**Alternative**: Keep the `&SdfGrid` functions as internal helpers and
add new public `&dyn PhysicsShape` entry points. This avoids touching
27 call sites. The downside is API surface bloat — two functions that
do the same thing with different signatures.

**Recommendation**: Change the signatures. The migration is mechanical
and tests should pass identically.

### 4.4 Octree Detection (`octree_detect.rs`) — No Changes

Already uses `shape.distance()` / `shape.gradient()` throughout:
- `extract_leaf_contacts` (line 228–229): exact
- `newton_contact` (line 277–282): exact
- `extract_plane_leaf` (line 753–776): exact
- `refine_contact_normal` (line 850–854): exact

Confirmed correct. No changes needed.

### 4.5 AnalyticalShape (`analytical_shape.rs`) — No Changes

Already delegates to exact `Solid`:
- `distance()` → `solid.evaluate()` (line 44)
- `gradient()` → normalized `solid.gradient()` (line 48)
- `evaluate_interval()` → `solid.evaluate_interval()` (line 74)

Confirmed correct. No changes needed.

## 5. What Stays the Same

- Tier 1 SDF-SDF dispatch (gated on `prefers_single_contact()`) — unchanged
- Tier 2 octree detection — unchanged, already exact
- Grid used for spatial indexing in Tier 3 (cell iteration)
- Grid used for MJCF shapes (`ShapeConvex`/`ShapeConcave` delegate to grid)
- Visual mesh — unchanged (already uses exact evaluation)
- `prefers_single_contact()` — spheres still use Tier 1 for both SDF-SDF
  and SDF-plane
- `refine_contact_normal` — still applied to Tier 3 contacts. Harmless
  no-op for analytical shapes (Newton finds point already on surface after
  1 iteration with |d| < 1e-10). Costs ~6 cheap calls per contact.
- GPU path — uses grid directly, no trait dispatch on GPU. Not in scope.

## 6. Expected Results

### Cuboid-Plane Stability
- A cuboid face on a plane gets 4–8 distributed contacts (from octree
  plane detection)
- Contacts span the face → torque resistance → cube doesn't tip from
  small perturbations
- This is the primary observable fix for 08-stack

### Cuboid-Cuboid (Already Working)
- Tier 2 octree already produces multi-contact with exact evaluation
- No change expected here — confirm this remains stable

### ShapeConvex-Plane (MJCF Fix)
- After dispatch fix, MJCF convex shapes get Tier 3 grid multi-contact
  for planes instead of Tier 1 single-contact
- Grid produces 4 contacts (MAX_SDF_PLANE_CONTACTS) distributed across
  the bottom face

### Edge Sharpness (Tier 3 only)
- After §4.3 (Tier 3 threading), shapes with exact SDF get exact
  normals from surface tracing. Cuboid edges: exact 90°, no rounding.
- This only matters for shapes that actually reach Tier 3 (mixed pairs,
  or shapes with loose `evaluate_interval`). Pure AnalyticalShape pairs
  already get exact evaluation from Tier 2 octree.

### Performance
- Tier 1 → Tier 2 for cuboid-plane: slightly more expensive (octree
  traversal vs. one analytical formula). Still trivial for simple shapes.
- Tier 3 with exact eval: `Solid::evaluate()` for primitives is a few
  math ops — likely faster than grid trilinear (8 lookups + 7 lerps).

## 7. Testing

### 7.1 Dispatch Routing Tests (New — `shape.rs`)

#### Cuboid-plane takes Tier 2
Create an `AnalyticalShape` cuboid (5mm half-extents, Convex hint).
Call `compute_shape_plane_contact` with the cuboid on a z=0 plane.
Assert:
- `contacts.len() >= 3` (multi-contact, not single)
- Contacts span the face (max x-distance between contacts > 4mm)

#### Sphere-plane still takes Tier 1
Create an `AnalyticalShape` sphere (5mm, Sphere hint). Call
`compute_shape_plane_contact`. Assert:
- `contacts.len() == 1` (single contact, fast path preserved)

#### ShapeConvex cuboid-plane takes Tier 3
Create a `ShapeConvex` from a box grid. Call
`compute_shape_plane_contact`. Assert:
- `contacts.len() >= 2` (multi-contact, not single)

### 7.2 Octree Plane Detection Quality (New — `octree_detect.rs`)

#### Cuboid face produces distributed contacts
An AnalyticalShape cuboid sitting on a plane with 1mm overlap.
Call `octree_plane_detect` directly. Assert:
- ≥ 4 contacts
- Contacts near the 4 corners of the bottom face (within 2mm)
- All normals equal plane normal

#### Tilted cuboid produces edge contacts
Cuboid rotated 45° about X axis, edge resting on plane. Assert:
- ≥ 2 contacts along the edge
- Contacts separated along the edge axis

### 7.3 Tier 3 Exact Evaluation Tests (New — after §4.3)

#### Box-Box Normal Exactness
Two `AnalyticalShape` cuboids overlapping 1mm along X. Force Tier 3
by calling `sdf_sdf_contact_raw_split` directly. Assert:
- Contact normals within 1° of ±X axis
- Compare with `ShapeConvex` grid result — grid normals may deviate
  by several degrees near edges

### 7.4 Integration — Stack Stability

Run 08-stack example. Verify EXPECTED_BEHAVIOR.md pass criteria at
t ≥ 5s:
- Each cube z within 2mm of expected height
- Inter-cube gaps ≈ 10mm
- All z-velocities < 1 mm/s
- Contacts active (ncon > 0)

### 7.5 Regression

```
cargo test -p sim-core -p sim-conformance-tests -p cf-design
```

MJCF shapes (ShapeConvex, ShapeConcave, ShapeSphere) must produce
identical results. Their `distance()`/`gradient()` delegate to the
grid — the Tier 3 change is a no-op for them.

## 8. Implementation Plan

### Phase 1: Fix Plane Dispatch Gate

1. In `compute_shape_plane_contact`: wrap Tier 1 in
   `if shape.prefers_single_contact()` guard
2. Write dispatch routing tests (§7.1)
3. Write octree plane quality test (§7.2)
4. Verify: `cargo test -p sim-core`

This is the critical fix. After Phase 1, cuboid-plane gets multi-contact
via octree (Tier 2) with exact evaluation. The 08-stack example should
stabilize.

### Phase 2: Thread `PhysicsShape` into Tier 3

1. Change `trace_surface_into_other` signature (§4.3.1)
2. Change `sdf_sdf_contact_raw_split` / `sdf_sdf_contact_raw` /
   `sdf_sdf_contact` signatures (§4.3.2)
3. Change `sdf_plane_contact` signature (§4.3.3)
4. Migrate 27 callers (§4.3.4)
5. Write Tier 3 exact evaluation test (§7.3)
6. Verify: `cargo test -p sim-core -p cf-design`

This is maintenance — improves Tier 3 for mixed pairs and future
fallback cases. It does NOT fix the 08-stack bug (that's Phase 1).

### Phase 3: Integration Validation

1. Run 08-stack example, verify pass criteria
2. Nudge top cube laterally — should tip off edge cleanly
3. Run conformance tests: `cargo test -p sim-conformance-tests`

## 9. Resolved Questions

### 9.1 "Grid rounding causes cubes to roll"

**Wrong.** AnalyticalShape cuboids take Tier 2 (octree) for
cuboid-cuboid contacts, which already uses exact evaluation. The grid
is never queried for distance/gradient in this path. The "rolling"
behavior was caused by single-contact plane dispatch, not edge rounding.

### 9.2 "Tier 3 needs exact evaluation for cuboids"

**Not for the motivating use case.** AnalyticalShape cuboids never
reach Tier 3 — they take Tier 2 (both provide `evaluate_interval()`).
Tier 3 exact evaluation helps mixed pairs and fallback cases, but
isn't the fix for 08-stack.

### 9.3 `refine_contact_normal` redundancy after Tier 3 threading

After Phase 2, Tier 3 contacts from analytical shapes already have
exact normals. `refine_contact_normal` becomes a no-op (Newton
converges in 0 iterations with |d| < 1e-10). Leave it — 6 cheap
calls per contact, not worth a branch to skip.

### 9.4 GPU path

Not in scope. GPU uses the grid directly (no trait dispatch on GPU).
CPU path handles cf-design shapes; GPU handles large MJCF scenes.

### 9.5 Coarse grid optimization

Defer. With exact evaluation in Tier 2/3, grid resolution controls
only spatial density of contact candidates. Coarser grid = fewer
contacts on flat faces. Measure after this spec ships.

## 10. Files to Touch

| File | Phase | Change |
|------|-------|--------|
| `sim/L0/core/src/sdf/shape.rs` | 1 | Add `prefers_single_contact()` guard to plane dispatch |
| `sim/L0/core/src/sdf/shape.rs` | 1 | New dispatch routing tests |
| `sim/L0/core/src/sdf/octree_detect.rs` | 1 | New plane detection quality tests (no code change) |
| `sim/L0/core/src/sdf/operations.rs` | 2 | Signature change + query delegation |
| `sim/L0/core/src/sdf/primitives.rs` | 2 | `sdf_plane_contact` signature + queries |
| `sim/L0/core/src/sdf/mod.rs` | 2 | Re-export update |
| `sim/L0/core/src/sdf/operations.rs` tests | 2 | Wrap grids in `ShapeConvex` (15 tests) |
| `sim/L0/core/src/sdf/primitives.rs` tests | 2 | Wrap grids in `ShapeConvex` (8 tests) |
| `sim/L0/gpu/src/collision.rs` tests | 2 | Wrap grids in `ShapeConvex` (2 tests) |
| `design/cf-design/src/mechanism/model_builder.rs` tests | 2 | Wrap grids in `ShapeConvex` (2 tests) |
| `sim/L0/core/src/sdf/octree_detect.rs` | — | No changes (confirmed exact) |
| `design/cf-design/src/mechanism/analytical_shape.rs` | — | No changes (confirmed exact) |
