# mesh-measure-bounding-box

**AABB and OBB on the same mesh — when do they coincide, when do they
diverge?** Hand-authors a two-cube fixture (one axis-aligned, one
tilted 45° around Z) and exercises the full `mesh-measure::dimensions`
+ `oriented_bounding_box` public surface. Demonstrates that PCA-driven
OBB recovers the original cube volume on tilted geometry while AABB
inflates by 2×.

> Skeleton commit (`§6.2 #2` per the v1.0 examples-coverage arc spec
> at `mesh/MESH_V1_EXAMPLES_SCOPE.md`). Fixture + anchors land in the
> next two commits. This README is a placeholder; the museum-plaque
> content (numerical anchors, visuals, cross-references) fills in
> alongside the fixture commit.

## What this example demonstrates

Two cubes side-by-side as one input mesh:

- **Axis-aligned cube** at `[0, 10]³` (8 verts + 12 tris).
- **Tilted cube** with the same dimensions, rotated 45° around Z and
  translated so its bbox-center sits at `(25, 0, 5)`. The rotation
  uses `let s = f64::sqrt(0.5)` for both `cos` and `sin` coefficients
  — `f64::sqrt` is correctly-rounded per IEEE-754 (`f64::sin(π/4)`
  and `f64::cos(π/4)` are NOT correctly-rounded; using sqrt-derived
  values keeps the rotation matrix FP-stable across libm versions).

The combined fixture demonstrates:

- AABB (`dimensions`) inflates the tilted cube to `(10√2)² × 10 =
  2000` mm³ — twice the original volume.
- OBB (`oriented_bounding_box`) recovers `≈ 1000` mm³ via PCA on the
  16-vertex point cloud (5% tolerance reflects FP noise in the
  symmetric eigendecomposition).

## Numerical anchors (TBD — land in `§6.2 #3` and `§6.2 #4`)

- 16 hand-authored vertex coordinates within `1e-12`.
- 24 face-winding cross-product unit normals.
- `Dimensions` fields: `width`, `depth`, `height`, `diagonal`,
  `bounding_volume`, `center`, `min_extent`, `max_extent`,
  `aspect_ratio`, `is_cubic`.
- `OrientedBoundingBox` fields: `volume` within 5%, `extents`
  ordering (longest, mid, shortest), `surface_area > 0`, `contains`
  on all 16 input vertices.

## Visuals

- `out/mesh.ply` — the two-cube fixture (16 verts, 24 tris). Open in
  f3d, MeshLab, or ParaView to see the axis-aligned + tilted pair.

## Run

```text
cargo run -p example-mesh-mesh-measure-bounding-box --release
```

Output written to
`examples/mesh/mesh-measure-bounding-box/out/mesh.ply`.

## Cross-references

- Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.1 (this example) +
  §4.4 (FP-exact rotation via `f64::sqrt(0.5)`).
- Sister examples: `mesh-measure-cross-section` (§5.2),
  `mesh-measure-distance-to-mesh` (§5.3) — round out the
  `mesh-measure` public-surface coverage.
- Mesh book: `docs/studies/mesh_architecture/src/80-examples.md`
  Part 8 (depth-pass lands in `§6.2 #31` of the arc).
