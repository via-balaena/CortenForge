# mesh-sdf-distance-query

**Numerical signed-distance field on a closed mesh — signed +
unsigned distance, closest point, inside/outside classification,
direct geometric primitives, and a bulk-query 1000-point cubic
grid.** Hand-authors a unit octahedron (6 verts + 8 tris) as
`IndexedMesh` and exercises the full `mesh-sdf` public surface:
`SignedDistanceField` cached queries, `signed_distance` /
`unsigned_distance` one-shot free fns, `closest_point_on_triangle`
/ `ray_triangle_intersect` / `point_segment_distance_squared`
direct geometric primitives, and `point_in_mesh` ray-casting
inside-test.

> Skeleton commit (`§6.2 #9` per the v1.0 examples-coverage arc spec
> at `mesh/MESH_V1_EXAMPLES_SCOPE.md`). The 6-vertex / 8-triangle
> octahedron fixture + 14 SDF query points + direct geometric
> primitive anchors + 1000-point bulk-grid PLY land in the next
> commit (`§6.2 #10`). This README is a placeholder; the
> museum-plaque content (numerical anchors, visuals, cross-
> references) fills in alongside the fixture commit.

## What this example demonstrates

A unit octahedron — 6 vertices at `(±1, 0, 0)`, `(0, ±1, 0)`,
`(0, 0, ±1)` and 8 triangle faces (one per `(sx, sy, sz)` octant
with `sx, sy, sz ∈ {±1}`). The octahedron is the L1-unit-ball
`|x| + |y| + |z| ≤ 1`; its signed distance has a closed-form
analytical expression `(|x| + |y| + |z| − 1) / √3` for query
points whose closest face is a face interior (NOT an edge or
vertex region), so anchors lock to `1e-12` against the analytical
formula.

**Why octahedron, not sphere**: a UV-tessellated sphere would
have `1/√3` chord-error at face centers; the octahedron has a
finite triangle count (8) with FP-exact integer vertex
coordinates and analytically-known SDF, so every anchor is
tractable to derive and bit-stable across libms.

The example computes (TBD — land in `§6.2 #10`):

1. **Empty-mesh edge case** — `SignedDistanceField::new(empty)`
   returns `Err(SdfError::EmptyMesh)` per `sdf.rs:48-51`.
2. **14 SDF query points** — 1 generic interior, 6 face-center
   directions (interior, ±0.3 octant variants), 6 vertex-direction
   off-mesh queries (each clamps to a vertex, distance `1.0`),
   1 far exterior `(10, 10, 10)`. Origin `(0, 0, 0)` is **NOT** a
   query — see "Origin corner-degeneracy" below.
3. **Direct geometric primitives** — `closest_point_on_triangle`
   on a known triangle for the 3 sub-cases (face-interior, edge,
   vertex region); `ray_triangle_intersect` for hit / miss /
   parallel; `point_segment_distance_squared` for perpendicular
   drop / beyond-endpoint.
4. **One-shot free fns** match `SignedDistanceField` cached values
   bit-equivalently on the same query points.
5. **Bulk grid** — 1000-point cubic grid in `[-2, 2]³` (10×10×10,
   spacing `4/9`, endpoint-inclusive) saved as
   `out/sdf_grid.ply` via `save_ply_attributed` with
   `extras["signed_distance"]` per-vertex scalar. User colormaps
   externally by signed distance.

## Origin corner-degeneracy (R5 + HE-1)

`SignedDistanceField::is_inside` uses ray-casting in the +X
direction (per `mesh-sdf::query::point_in_mesh`). For the unit
octahedron, the +X ray from the origin hits the vertex `(1, 0, 0)`
which is shared by 4 faces; Möller-Trumbore returns `Some(t = 1)`
for ALL 4 faces, count = 4, even → `is_inside((0, 0, 0))` reports
**false** (the origin is reported as outside).

This is a documented platform truth, not a fixture bug. The
example uses an off-axis interior point `(0.05, 0.07, 0.11)` to
probe `is_inside == true` on a clear interior. v0.9 candidate
upgrade: winding-number-based inside-test (spec §10 item 8).

## Numerical anchors (TBD — land in `§6.2 #10`)

- 6 hand-authored octahedron vertices at `1e-12` (FP-exact integer
  `(±1, 0, 0)`, `(0, ±1, 0)`, `(0, 0, ±1)`).
- 8 face-winding cross-product unit-normal anchors at `1e-12`
  (analytical `(sx, sy, sz) / √3` per octant — outward-pointing;
  CCW from outside requires parity-flipped winding `[v0, v2, v1]`
  for the 4 octants where `sx*sy*sz = -1`).
- 14 SDF query points anchored at `1e-12` against analytical
  `(|x| + |y| + |z| − 1) / √3` (interior face-region) or vertex-
  clamped Euclidean distance (vertex-direction queries).
- 3 `closest_point_on_triangle` sub-cases at `1e-12`.
- 3 `ray_triangle_intersect` sub-cases at `1e-12`.
- 2 `point_segment_distance_squared` sub-cases at `1e-12`.
- `point_in_mesh((0.05, 0.07, 0.11), octahedron) == true`;
  `point_in_mesh((10, 10, 10), octahedron) == false`.
- One-shot `signed_distance` / `unsigned_distance` free fns match
  `SignedDistanceField` cached values bit-equivalently.
- Bulk-grid stats: discrete count + percent inside + max unsigned
  distance.

## Run (TBD — land in `§6.2 #10`)

```text
cargo run -p example-mesh-mesh-sdf-distance-query --release
```

Output: `out/octahedron.ply` (input mesh) +
`out/sdf_grid.ply` (1000-vertex grid with per-vertex
`signed_distance` for external colormap).

## Cross-references

- **Spec**: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.4 (this example).
- **Sister examples** in the v1.0 mesh-measure trio:
  `mesh-measure-bounding-box` (§5.1) at `719a85d3`,
  `mesh-measure-cross-section` (§5.2) at `021a9712`,
  `mesh-measure-distance-to-mesh` (§5.3) at `4650058a`.
- **PLY-attribute pattern**: `examples/mesh/ply-with-custom-attributes/`
  — same `save_ply_attributed` + `extras["<scalar>"]` pattern; this
  example reuses it for the bulk-grid output.
- **Mesh book**: `docs/studies/mesh_architecture/src/80-examples.md`
  — Part 8 inventory (depth-pass updates land in `§6.2 #31` of the
  arc).
- **Inside-test caveat**: corner-degeneracy noted above is the
  trigger for v0.9 candidate `winding-number-based inside-test`
  (spec §10 item 8). `mesh-measure::distance_to_mesh` (§5.3) is
  the **unsigned** sibling — same `closest_point_on_triangle`
  internals (spec §10 item 7 v0.9 dedup candidate), but no
  inside-test.
