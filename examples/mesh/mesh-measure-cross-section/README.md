# mesh-measure-cross-section

**Planar slicing of a mesh into 2D contours — perimeter, area, centroid,
slice stacks.** Hand-authors a 32-segment closed cylinder (radius 5 mm,
height 10 mm) and exercises the full `mesh-measure::cross_section` /
`cross_sections` / `circumference_at_height` / `area_at_height` public
surface. Demonstrates plane-mesh intersection, multi-slice stacks along
an axis, and convenience-helper equivalence.

> Skeleton commit (`§6.2 #5` per the v1.0 examples-coverage arc spec
> at `mesh/MESH_V1_EXAMPLES_SCOPE.md`). The 32-segment cylinder
> fixture (66 verts + 128 tris) + cross-section anchors land in the
> next commit (`§6.2 #6`). This README is a placeholder; the
> museum-plaque content (numerical anchors, visuals, cross-references)
> fills in alongside the fixture commit.

## What this example demonstrates

A closed cylinder hand-authored from:

- **32 bottom-ring vertices** at `(5·cos(2π·i/32), 5·sin(2π·i/32), 0)`
  for `i` in `0..32` (sin/cos derived; FP-stable to `1e-12`).
- **32 top-ring vertices** at the same `(x, y)` with `z = 10`.
- **2 cap centers** at `(0, 0, 0)` and `(0, 0, 10)`.

Total: **66 verts**, **128 tris** (32 bottom-cap fan + 32 top-cap fan
+ 32 wall-quads × 2 = 64 wall tris).

The example computes:

1. `cross_section(mesh, (0, 0, 5), Vector3::z())` — single mid-height
   slice; perimeter, area, centroid, contour count.
2. `cross_sections(mesh, (0, 0, 0.5), Vector3::z(), 10, 1.0)` — 10-slice
   stack at `z = 0.5..9.5` (z-uniform cylinder ⇒ all slices equal).
3. `circumference_at_height(mesh, 5.0)` and `area_at_height(mesh, 5.0)`
   — convenience-helper equivalence to the explicit single-slice call.
4. Out-of-mesh slice at `z = 100.0` — empty cross-section behavior.
5. `plane_normal` normalization — caller passes `Vector3::new(0, 0, 2)`
   un-normalized; output `plane_normal` has `||p|| = 1`.

## Numerical anchors (TBD — land in `§6.2 #6`)

- 66 hand-authored vertex coordinates within `1e-12` (sin/cos derived).
- 128 face-winding cross-product unit normals (cap fans → ±z; wall
  tris → radial-outward at chord midpoint).
- Mid-slice (`z = 5`) — area `400·sin(π/16) ≈ 78.0357` (polygon
  shoelace, NOT analytical π·25); perimeter `320·sin(π/32) ≈ 31.37`;
  `contour_count == 1`; `is_closed() == true`.
- Slice stack — 10 slices at `z = 0.5..9.5`, all with the same area
  as the mid-slice (z-uniform cylinder).
- Convenience helpers — `circumference_at_height(_, 5.0)` and
  `area_at_height(_, 5.0)` match the single-slice values.
- Out-of-mesh slice — `is_empty() == true`, `contour_count == 0`,
  `area == 0.0`.
- `plane_normal` — magnitude 1 within `1e-12`.

## Visuals

- `out/cylinder.ply` — the 32-segment closed cylinder (66 verts,
  128 tris). Open in f3d, MeshLab, or ParaView to see the
  axis-aligned cylinder.

## Run

```text
cargo run -p example-mesh-mesh-measure-cross-section --release
```

Output written to
`examples/mesh/mesh-measure-cross-section/out/cylinder.ply`.

## Cross-references

- Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.2 (this example) +
  §4.4 (FP-exact rotation via `f64::sqrt(0.5)`) + §4.7 (sin/cos
  tolerance `1e-12` not bit-exact) + §7 R6 (UV-cylinder face
  winding cosine similarity).
- Sister examples: `mesh-measure-bounding-box` (§5.1),
  `mesh-measure-distance-to-mesh` (§5.3) — round out the
  `mesh-measure` public-surface coverage.
- Mesh book: `docs/studies/mesh_architecture/src/80-examples.md`
  Part 8 (depth-pass lands in `§6.2 #31` of the arc).
