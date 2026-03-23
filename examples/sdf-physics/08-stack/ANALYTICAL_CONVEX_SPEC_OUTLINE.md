# Exact SDF Evaluation for Collision — Spec Outline

## Problem

SDF grid collision rounds sharp edges. A 10mm cuboid at 1mm grid
resolution has ~1mm edge rounding. Cubes roll like spheres instead of
tumbling with sharp edge transitions.

Discovered via example 08-stack: nudged cubes roll off each other instead
of sliding to the edge and tipping.

## Root Cause

The collision pipeline queries the SDF **grid** — a sampled approximation.
Trilinear interpolation between grid points smooths sharp features.

cf-design shapes (`AnalyticalShape`) have the **exact** SDF function:
- `Solid::evaluate(point)` → exact signed distance, sharp edges preserved
- `Solid::gradient(point)` → exact surface normal, discontinuous at edges

The grid exists because MJCF shapes (imported from files) have no
analytical function — the grid IS their only representation. cf-design
shapes have both the exact function and the grid. Collision currently
uses the grid for all shapes, ignoring the exact function.

The visual mesh (marching cubes) already has sharp edges because it calls
`evaluate()` to find exact zero-crossings. Collision should do the same.

## Approach: Exact SDF Evaluation in Existing Collision Algorithms

**No new collision algorithms.** The existing surface tracing and octree
algorithms are correct. They just need to call the exact SDF function
instead of interpolating the grid.

- Grid remains for **spatial indexing** (which cells to visit)
- Exact SDF function used for **distance and normal queries**
- Same code path for all shapes — exact where available, grid where not

This is the only approach we're pursuing. SAT, GJK, mesh-as-collider,
and other alternatives were considered and rejected as unnecessary
complexity. The SDF function route is cleaner: one code path, all shapes
benefit, zero new algorithms.

## What Changes

### 1. Surface Tracing (`operations.rs`)

Currently: iterates near-surface grid cells, calls `sdf_grid.distance()`
and `sdf_grid.gradient()` to check penetration and compute normals.

Change: when the shape provides exact evaluation (`distance()` /
`gradient()` return `Some`), call those instead of the grid. The grid
cell iteration stays — it's just spatial indexing.

### 2. Octree Detection (`octree_detect.rs`)

Currently: uses `evaluate_interval()` for cell pruning, Newton iteration
for refinement. Already partially exact.

Change: verify that leaf contact extraction calls `distance()` /
`gradient()` on the shape, not on the grid. The pruning is already
interval-based (exact). Newton refinement should already use the shape's
`distance()` — confirm this.

### 3. SDF-Plane Collision (`shape.rs`)

Currently: uses `effective_radius()` for single contact, or grid surface
tracing for multi-contact.

Change: same as surface tracing — use exact evaluation where available.

### 4. PhysicsShape Trait

Already has the right methods:
```rust
fn distance(&self, local_point: &Point3<f64>) -> Option<f64>;
fn gradient(&self, local_point: &Point3<f64>) -> Option<Vector3<f64>>;
```

`AnalyticalShape` implements both via the exact `Solid`. Grid-only shapes
return values from the grid. No new trait methods strictly required —
the collision code just needs to call `distance()` / `gradient()` on the
shape instead of calling equivalent methods on `sdf_grid()`.

If we want an explicit opt-in flag:
```rust
fn has_exact_sdf(&self) -> bool { false }
```
But this may not be necessary — `distance()` returning `Some` vs `None`
already signals capability.

## What Stays the Same

- Grid used for spatial indexing (cell iteration, broadphase)
- Grid used for MJCF shapes (no exact function available)
- Dispatch tiers unchanged (1/2/3 all benefit from exact eval)
- Visual mesh unchanged (already uses exact evaluation)
- All CSG operations work (their `Solid::evaluate()` is exact)

## Expected Results

- Cuboid edges: exact 90-degree in collision, no rounding
- Face-face contacts: distributed across overlap (surface tracing)
- Edge transitions: discontinuous normals (proper tumbling, not rolling)
- CSG shapes: also benefit (exact subtract/union boundaries)
- Performance: `Solid::evaluate()` for simple primitives is a few math
  ops — likely faster than grid trilinear interpolation
- Grid resolution becomes irrelevant for edge sharpness (only affects
  which cells get visited, not the actual distance values)

## Files to Touch

- `sim/L0/core/src/sdf/operations.rs` — surface tracing distance/normal
  queries → shape.distance() / shape.gradient()
- `sim/L0/core/src/sdf/octree_detect.rs` — verify leaf extraction uses
  exact evaluation
- `sim/L0/core/src/sdf/shape.rs` — possibly add `has_exact_sdf()`,
  adjust surface tracing calls to accept shape reference
- `cf-design/src/mechanism/analytical_shape.rs` — confirm distance() /
  gradient() delegate to exact Solid (already true)
- Tests: box-box tumbling, edge-edge contact, stacking stability

## Open Questions (for full spec)

- Threading the `PhysicsShape` reference into surface tracing: currently
  `sdf_sdf_contact_raw_split` takes two `&SdfGrid`. It needs to also
  accept the shape (or a closure for exact evaluation). What's the
  cleanest interface?
- Does `prefers_single_contact()` (added this session) still make sense?
  Probably yes — sphere-sphere is still faster as analytical single
  contact than running surface tracing with exact eval.
- Performance of deep CSG trees: `Solid::evaluate()` walks the CSG tree
  recursively. For complex designs with 20+ operations, is per-contact
  evaluation fast enough? May need benchmarking. Simple primitives
  (cuboid, sphere, cylinder) are trivially fast.
- Grid resolution for spatial indexing: with exact evaluation, could we
  use a coarser grid (fewer cells to visit) without losing edge
  sharpness? This would be a pure performance win.
