# mesh-measure-cross-section

**Planar slicing of a 32-segment closed cylinder — mid-slice + 10-slice
stack + convenience helpers + out-of-mesh + plane-normal normalization.**
Hand-authors a UV-cylinder fixture (radius 5 mm, height 10 mm; 66 verts +
128 tris) and exercises the full `mesh-measure::cross_section` /
`cross_sections` / `circumference_at_height` / `area_at_height` public
surface. Demonstrates: chord-shrinkage in polygon-shoelace area
(0.64% under analytical π·r² for n=32 segments), z-uniform polygon shape
across all 10 stack slices, helper-function equivalence to the explicit
single-slice call, empty-slice behavior outside the mesh, and internal
`plane_normal.normalize()` regardless of input magnitude. Surfaces a
v0.9 platform gap: the library's `CrossSection::centroid` is the naive
average over chain-closed perimeter points, **not** the polygon centroid.

## What it does

Builds a 66-vertex / 128-triangle closed cylinder:

- **Bottom ring** (verts 0..31) at z=0; vertex i at
  `(5·cos(2π·i/32), 5·sin(2π·i/32), 0)` — sin/cos derived,
  cross-platform stable to `1e-12` (`f64::sin` / `f64::cos` are NOT
  correctly-rounded; tolerance per spec §4.7).
- **Top ring** (verts 32..63) at the same `(x, y)` with z=10.
- **Cap centers**: bottom (vert 64) at `(0, 0, 0)`; top (vert 65) at
  `(0, 0, 10)`.
- **Faces**: 32 bottom-cap fan tris (outward normal -z) + 32 top-cap
  fan tris (outward normal +z) + 64 wall tris (32 quads × 2; outward
  normal radial at chord midpoint).

Then computes:

1. `cross_section(mesh, (0, 0, 5), Vector3::z())` — single mid-height
   slice; perimeter, area, centroid, plane normal, plane origin,
   bounds, contour count, `is_closed`.
2. `cross_sections(mesh, (0, 0, 0.5), Vector3::z(), 10, 1.0)` — 10
   slices at z = 0.5..9.5, all matching the mid-slice area + perimeter
   bit-equivalently (z-uniform cylinder).
3. `circumference_at_height(mesh, 5.0)` and `area_at_height(mesh, 5.0)`
   — convenience-helper wrappers; outputs match the explicit single-
   slice call.
4. `cross_section(mesh, (0, 0, 100), Vector3::z())` — out-of-mesh
   slice; returns the default `CrossSection` with `is_empty() == true`
   and area / perimeter exactly `0.0`.
5. `cross_section(mesh, (0, 0, 5), Vector3::new(0, 0, 2))` — caller
   passes un-normalized plane normal; library normalizes internally
   so output `plane_normal.norm() == 1`.

Then writes `out/cylinder.ply` (the input mesh) and asserts every
numerical anchor below.

## Numerical anchors

Each anchor is encoded as `assert_relative_eq!` (or `assert_eq!` for
counts) in `src/main.rs` under `verify_fixture_geometry`,
`verify_mid_slice`, `verify_slice_stack`, `verify_helpers`,
`verify_out_of_mesh`, and `verify_plane_normal_normalization`. Per
[`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
a clean `cargo run --release` exit-0 == clean visual inspection.

### Fixture geometry (`verify_fixture_geometry`)

- 66 per-vertex coordinate anchors at `1e-12` (32 bottom-ring + 32
  top-ring sin/cos derived, plus 2 FP-exact cap centers). Per spec
  §4.7, `f64::sin` / `f64::cos` are not correctly-rounded; `1e-12`
  covers cross-platform libm drift.
- 128 per-face winding cross-product unit-normal anchors at cosine-
  similarity `> 0.999_999_9` with the analytical outward direction
  (cap fans → ±z; wall tris → radial at chord midpoint angle
  `(2k+1)·π/32`). Spec §7 R6 sets the worst-case floor at `> 0.99`;
  the tighter floor surfaces any winding flip without burning
  cross-platform headroom.

### Mid-slice (`verify_mid_slice`)

The wall-quad split into `[bot_k, bot_(k+1), top_(k+1)]` (tri A) +
`[bot_k, top_(k+1), top_k]` (tri B) means each wall side contributes
**2 plane-edge segments** at every z ∈ (0, 10): one outer ring point
on the right vertical edge + one diagonal-midpoint on the shared
`bot_k → top_(k+1)` edge. The diagonal midpoint at any z lies on the
chord between the two adjacent outer ring intersections at parameter
`t = z/h`, so the polygon traces the same convex 32-gon shape
regardless of z (just with one extra collinear vertex per chord).
Output contour: 64 unique perimeter points + 1 chain-closure
duplicate ⇒ **65 stored points**.

Anchors:

- `area = 400·sin(π/16) ≈ 78.0357 mm²` (polygon shoelace; **NOT**
  analytical π·r² ≈ 78.5398 — 32-segment chord-shrinkage is 0.64%,
  outside the original spec's 0.5% tolerance proposal). Tolerance
  `1e-10`.
- `perimeter = 320·sin(π/32) ≈ 31.3655 mm` (32 chords; analytical
  2π·r = 31.4159, 0.16% chord-shrinkage). Tolerance `1e-10`.
- `centroid ≈ (-0.0762, -0.0075, 5.0)` — **NOT** `(0, 0, 5)`. The
  library returns the naive average of the 65 chain-closed points
  (NOT the polygon centroid). For our symmetric 32-gon perimeter,
  `sum_unique = 0`; `sum = V_dup` where `V_dup` is the duplicated
  chain-closure vertex. Probe shows `V_dup = mid_(16, 17)` at z=5,
  giving `centroid = (-(1+cos(π/16))/26, -sin(π/16)/26, 5)`.
  Captured as a v0.9 candidate gap in `mesh-measure` (spec §10
  item 11). Anchor tolerance `1e-12` (deterministic per FP).
- `contour_count == 1`; `is_closed() == true` — semantically
  `contour_count > 0`, **NOT** geometric closure (last point ==
  first point); per `cross_section.rs:73-77`.
- `plane_normal.norm() == 1` to `1e-12`; `plane_normal == (0, 0, 1)`.
- `plane_origin == (0, 0, 5)` (echoes input `plane_point`).
- Bounds within `±r` in (x, y); z-range `[5, 5]`.

### Slice stack (`verify_slice_stack`)

`cross_sections(mesh, (0, 0, 0.5), Vector3::z(), 10, 1.0)` returns
**10 slices** at `z = 0.5, 1.5, 2.5, …, 9.5`. Cylinder is z-uniform
and the polygon shape is invariant in z (per the chord-parameter
argument above), so every slice's area + perimeter match the
mid-slice values to `1e-10`. Each slice's `plane_origin.z` matches
the analytical `0.5 + i·1.0`.

### Convenience helpers (`verify_helpers`)

`circumference_at_height(mesh, 5.0)` and `area_at_height(mesh, 5.0)`
invoke `cross_section` with the same arguments as the explicit
single-slice call. Outputs are bit-equivalent (FP determinism);
anchor tolerance `1e-12`.

### Out-of-mesh slice (`verify_out_of_mesh`)

At `z = 100.0`, no triangle straddles the plane; `cross_section`
short-circuits to `CrossSection::default()` with `points.is_empty()`,
`contour_count == 0`, `area == 0.0` exact, `perimeter == 0.0` exact,
`is_closed() == false`. `plane_origin` echoes input `(0, 0, 100)`;
`plane_normal` is still normalized.

### Plane-normal normalization (`verify_plane_normal_normalization`)

Caller passes `Vector3::new(0, 0, 2)` (magnitude 2); `cross_section`
calls `plane_normal.normalize()` internally (line 122 of
`mesh-measure/src/cross_section.rs`). Output `plane_normal.norm() == 1`
to `1e-12`; the slice geometry is identical to the explicit-unit call
(same `area`, same `perimeter` to `1e-10`).

## Visuals

`out/cylinder.ply` contains the 66-vertex / 128-triangle closed
cylinder. Open in f3d to see the axis-aligned UV-cylinder:

```text
f3d examples/mesh/mesh-measure-cross-section/out/cylinder.ply
```

The cylinder renders as a 32-segment polygonal tube along the +Z
axis, height 10 mm, radius 5 mm. Cap fans converge to the bottom
and top center vertices (which sit at the geometric cap centers).
Viewed from above or below, the cap fan tris radiate from the
cap center to the ring; viewed from the side, the wall facets
form the 32-segment faceted side surface.

## Run

```text
cargo run -p example-mesh-mesh-measure-cross-section --release
```

Output: `examples/mesh/mesh-measure-cross-section/out/cylinder.ply`
(ASCII PLY, 66 verts + 128 tris). Stdout prints all measurement
summaries (mid-slice, stack, helpers, out-of-mesh, un-normalized
normal demo) with chord-shrinkage diagnostics.

## Cross-references

- **Sister examples** (round out `mesh-measure` public-surface
  coverage): `mesh-measure-bounding-box` (§5.1) shipped at
  `719a85d3`; `mesh-measure-distance-to-mesh` (§5.3) lands at
  §6.2 #7.
- **Mesh book**: `docs/studies/mesh_architecture/src/80-examples.md`
  — Part 8 inventory of v1.0 examples (depth-pass lands in spec
  §6.2 commit 31).
- **Surfaced platform gap** (v0.9 candidate in
  `mesh-measure::cross_section`):
  1. `CrossSection::centroid` is `sum / N` over chain-closed points
     (one duplicate). For symmetric polygons this returns
     `V_dup / N` instead of the true polygon centroid `(0, 0, 5)`.
     Proper fix: shoelace-weighted polygon centroid `c_x =
     (1/(6A))·Σ(x_i + x_(i+1))·(x_i·y_(i+1) − x_(i+1)·y_i)` and
     similar for `c_y` (project to 2D via the same `(u, v)` basis
     as `calculate_cross_section_area`, then back to 3D).
     ~30 LOC. Captured as spec §10 item 11.
