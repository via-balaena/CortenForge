# mesh-measure-distance-to-mesh

**Point-to-point + point-to-mesh distance + symmetric Hausdorff
between two meshes.** Hand-authors two unit cubes as separate
`IndexedMesh` instances (cube_a at `[0, 1]³`, cube_b at `[2, 3]³`)
and exercises the full `mesh-measure::measure_distance` /
`closest_point_on_mesh` / `distance_to_mesh` public surface.
Demonstrates that `mesh-measure` exposes the primitives;
**symmetric Hausdorff** between two meshes is achievable as
composition over the primitives (NOT a built-in).

> Skeleton commit (`§6.2 #7` per the v1.0 examples-coverage arc spec
> at `mesh/MESH_V1_EXAMPLES_SCOPE.md`). The two-cube fixture (16 verts
> + 24 tris across two separate meshes) + point-to-point +
> point-to-mesh + Hausdorff anchors land in the next commit
> (`§6.2 #8`). This README is a placeholder; the museum-plaque
> content (numerical anchors, visuals, cross-references) fills in
> alongside the fixture commit.

## What this example demonstrates

Two **vertex-disjoint, separate** unit cubes:

- **Cube A**: axis-aligned unit cube at `[0, 1] × [0, 1] × [0, 1]`
  (8 verts, 12 tris).
- **Cube B**: axis-aligned unit cube at `[2, 3] × [2, 3] × [2, 3]`
  (8 verts, 12 tris).

Closest-face distance between the two cubes along `+X` is `1.0`;
farthest-corner-to-farthest-corner pair is `(0, 0, 0)` ↔ `(3, 3, 3)`
at distance `3√3`. The two cubes live in distinct `IndexedMesh`
instances so `distance_to_mesh(&cube_a, _)` and
`distance_to_mesh(&cube_b, _)` query independent geometry. The
combined `out/two_cubes.ply` is for visualization only.

The example computes:

1. `measure_distance((0, 0, 0), (3, 4, 0))` — point-to-point Euclidean
   (3-4-5 triangle); `distance == 5.0`, `dx == 3.0`, `dy == 4.0`,
   `dz == 0.0`. **Note**: `dx`/`dy`/`dz` are **absolute** values per
   `distance.rs:97-99`.
2. `direction_normalized()` returns `Some((0.6, 0.8, 0.0))`;
   `midpoint()` returns `(1.5, 2.0, 0.0)`.
3. `distance_to_mesh(&cube_a, p)` for 6 face-direction queries
   (1.0 each), 8 corner-direction queries (`sqrt(3)` each), and
   1 center query (`0.5`).
4. `closest_point_on_mesh(&cube_a, p)` returns the expected closest
   face / corner point per query.
5. Empty-mesh edge case — `distance_to_mesh(&empty, p) == None`,
   `closest_point_on_mesh(&empty, p) == None`.
6. **Symmetric Hausdorff** — iterate cube_a's 8 verts → max
   `distance_to_mesh(&cube_b, v)` = `sqrt(12) = 2√3 ≈ 3.464`
   (achieved at vert `(0, 0, 0)`, closest point on cube_b is the
   clamped corner `(2, 2, 2)`). Symmetrically, cube_b's farthest
   vert `(3, 3, 3)` clamps to cube_a's `(1, 1, 1)` at the same
   distance. Hausdorff = `max(d_AB, d_BA) = sqrt(12)`.

## Numerical anchors (TBD — land in `§6.2 #8`)

- 16 hand-authored vertices within `1e-12` (8 per cube; FP-exact
  integer corners).
- 24 face-winding cross-product unit normals within `1e-12`
  (12 per cube; analytical ±x / ±y / ±z).
- Point-to-point — `distance == 5.0`, `dx == 3.0`, `dy == 4.0`,
  `dz == 0.0`, `direction_normalized == (0.6, 0.8, 0.0)`,
  `midpoint == (1.5, 2.0, 0.0)`.
- Point-to-mesh on cube_a — 6 face queries at distance `1.0`,
  8 corner queries at distance `sqrt(3)`, 1 center query at `0.5`.
- `closest_point_on_mesh` returns the expected clamped point per
  query (probe-locked).
- Empty mesh — `distance_to_mesh` and `closest_point_on_mesh`
  both return `None`.
- Symmetric Hausdorff — `d_AB == d_BA == sqrt(12)`, Hausdorff =
  `sqrt(12)`.

## Visuals

- `out/two_cubes.ply` — combined mesh (16 verts, 24 tris); two unit
  cubes side-by-side along `+X`. Open in f3d, MeshLab, or ParaView.

## Run

```text
cargo run -p example-mesh-mesh-measure-distance-to-mesh --release
```

Output written to
`examples/mesh/mesh-measure-distance-to-mesh/out/two_cubes.ply`.

## Cross-references

- Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.3 (this example) +
  §8 round 1 (Hausdorff sqrt(12) clamped-corner correction;
  original draft said sqrt(8) — clamped corner of `[2, 3]³`
  from the origin is `(2, 2, 2)`, not the face-interior
  `(2, 0, 0)`).
- Sister examples: `mesh-measure-bounding-box` (§5.1),
  `mesh-measure-cross-section` (§5.2) — round out the
  `mesh-measure` public-surface coverage.
- Mesh book: `docs/studies/mesh_architecture/src/80-examples.md`
  Part 8 (depth-pass lands in `§6.2 #31` of the arc).
