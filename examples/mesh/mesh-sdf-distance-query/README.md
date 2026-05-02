# mesh-sdf-distance-query

**Numerical signed-distance field on a closed mesh — signed +
unsigned distance, closest point, inside/outside classification,
direct geometric primitives, and a 1000-point bulk-query grid.**
Hand-authors a unit octahedron (6 verts + 8 tris) as `IndexedMesh`
and exercises the full `mesh-sdf` public surface:
`SignedDistanceField` cached queries (`distance` / `unsigned_distance`
/ `closest_point` / `is_inside` / `mesh`), `signed_distance` /
`unsigned_distance` one-shot free fns, `closest_point_on_triangle`
/ `ray_triangle_intersect` / `point_segment_distance_squared` direct
geometric primitives, `point_in_mesh` ray-casting inside-test, and
`SdfError::EmptyMesh` error path. The 1000-point grid lands as
`out/sdf_grid.ply` with `extras["signed_distance"]` for external
colormap rendering.

## What this example demonstrates

The unit octahedron — 6 vertices at `(±1, 0, 0)`, `(0, ±1, 0)`,
`(0, 0, ±1)` and 8 triangle faces (one per `(sx, sy, sz)` octant)
— is the L1-unit-ball `|x| + |y| + |z| ≤ 1`. Its signed distance
has a closed-form expression `(|x| + |y| + |z| − 1) / √3` for any
query point whose closest octahedron point is in a face interior.
For vertex-clamped queries, distance is the Euclidean offset from
the query to the closest vertex. **Both regimes are exercised in
this example** plus the public-surface coverage outlined above.

**Why octahedron, not sphere**: a UV-tessellated sphere would have
`1 - 1/√3 ≈ 0.423` chord-error at face centers (centroid of any
sphere-inscribed triangle sits at distance `1/√3` from origin, vs
the unit sphere at distance `1`). The octahedron has finite triangle
count (8) with FP-exact integer vertex coordinates and analytically-
known SDF, so every anchor is tractable to derive and bit-stable
across libms.

The example computes:

1. **Empty-mesh edge case** — `SignedDistanceField::new(empty)`
   returns `Err(SdfError::EmptyMesh)` per `sdf.rs:48-51`; the
   one-shot `signed_distance` / `unsigned_distance` free fns return
   the `f64::MAX` sentinel per `sdf.rs:236-239` + `sdf.rs:285-288`.
2. **14 SDF query points** — 1 generic interior, 6 face-center
   directions (interior, 6 of 8 octants), 6 vertex-direction
   off-mesh queries, 1 far exterior. Origin `(0, 0, 0)` is
   **excluded** per HE-1 (see "Origin corner-degeneracy" below).
3. **Direct geometric primitives** on a known reference triangle
   (vertices `(0, 0, 0)`, `(10, 0, 0)`, `(5, 10, 0)`):
   `closest_point_on_triangle` for the 3 region cases;
   `ray_triangle_intersect` for hit / miss / parallel;
   `point_segment_distance_squared` for perpendicular drop /
   beyond-endpoint.
4. **One-shot free fns** match `SignedDistanceField` cached values
   bit-equivalently (`to_bits()` equality) on shared queries.
5. **`point_in_mesh` boolean** — `(0.05, 0.07, 0.11)` → true;
   `(10, 10, 10)` → false.
6. **Bulk grid** — 1000-point cubic grid in `[-2, 2]³` (10 × 10 × 10,
   spacing `4/9`, endpoint-inclusive). Saved as `out/sdf_grid.ply`
   via `save_ply_attributed` with `extras["signed_distance"]` per-
   vertex scalar; user colormaps externally. **Two divergent inside-
   counters** — see "Bulk grid" anchor table below.

## Origin corner-degeneracy (R5 + HE-1)

`SignedDistanceField::is_inside` uses ray-casting in the +X
direction (per `query.rs:158-174`). For the unit octahedron, the
+X ray from the origin hits vertex `(1, 0, 0)` which is shared by
4 faces (`+x+y+z`, `+x+y-z`, `+x-y+z`, `+x-y-z`); Möller-Trumbore
returns `Some(t = 1)` for ALL 4 faces because the vertex `(1, 0, 0)`
lies on each triangle's `(u, v) = (0, 0)` boundary. count = 4,
even → `is_inside((0, 0, 0))` reports **false** (origin reads as
outside).

This is a documented platform truth, NOT a fixture bug. The
example uses an off-axis interior point `(0.05, 0.07, 0.11)` so
the +X ray hits the `+x+y+z` face interior at exactly one point.
v0.9 candidate upgrade: winding-number-based inside-test (spec
§10 item 8 trigger expanded by drift-10 below).

## Bulk-grid drift-10 — face-normal sign-test fails at vertex regions

A second, independent failure of the face-normal sign convention
surfaces in the 1000-point bulk-grid scan. `signed_distance < 0`
reports **14** grid points inside, but **only 8** are
geometrically inside the octahedron. The 6 false-positives are
points like `(-2, -2/3, 2/3)` where the closest octahedron point
is the vertex `(-1, 0, 0)`; 4 -X faces tie on this vertex via
Ericson's vertex-region branch, and the strict-`<` tie-break in
`SignedDistanceField::distance` picks the first emitted face (f2
in this fixture). For some of the 6 false-positives, f2's outward
normal `(-1, 1, -1) / √3` dotted with `to_point = p − v_f2_v0`
flips negative even though the point is geometrically OUTSIDE —
yielding `signed_distance < 0`.

This failure mode is fundamental to the **face-normal-of-closest-
face sign convention** (`sdf.rs:172-185`): when the closest point is
a vertex shared by ≥ 2 faces with diverging outward directions,
the chosen face's normal may give the wrong sign. **The failure
applies to CONVEX geometry** at vertex / edge regions (spec §7 R5
framed it as concave-only; corrected by drift-10 inline).

Compensating fix is the **pseudo-normal sign convention**
(Bærentzen-Aanæs angle-weighted vertex normal) or **winding-number
sign convention**. Both robustly classify inside / outside
regardless of which closest-point region wins on tie-break. v0.9
candidate trigger: spec §10 item 8 (description expanded inline at
this commit from "concave-mesh" to "vertex / edge regions of any
mesh, including CONVEX").

The example surfaces both counts as platform-truth anchors. For
analytical-grid queries, prefer **`point_in_mesh`** (ray-casting)
or a future winding-number variant; **`signed_distance < 0` is
unreliable at vertex / edge regions**.

## Numerical anchors

Each anchor is encoded as `assert_relative_eq!` (or `assert_eq!`
for counts) in `src/main.rs` under `verify_octahedron_geometry`,
`verify_empty_mesh`, `verify_sdf_query_points`,
`verify_one_shot_equivalence`, `verify_closest_point_on_triangle`,
`verify_ray_triangle_intersect`,
`verify_point_segment_distance_squared`, `verify_point_in_mesh`,
and `verify_bulk_query_stats`. Per
[`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
a clean `cargo run --release` exit-0 == clean visual inspection
(plus the optional colored-grid visual below).

### Fixture geometry (`verify_octahedron_geometry`)

- 6 per-vertex coordinate anchors at `1e-12` (FP-exact integer
  axis vertices `(±1, 0, 0)`, `(0, ±1, 0)`, `(0, 0, ±1)`).
- 8 per-face winding cross-product unit-normal anchors at `1e-12`
  (analytical `(sx, sy, sz) / √3` per octant; cross products on
  integer coordinates are bit-exact, the `1 / √3` divisor is
  correctly-rounded by the libm). Face windings (CCW from outside,
  with parity flip per spec R10):

  | Face | Octant | Vertex order | Outward normal     | Parity |
  |------|--------|--------------|--------------------|--------|
  | f0   | `+x+y+z` | `[0, 2, 4]` | `(+1, +1, +1) / √3` | +1 (std) |
  | f1   | `+x-y-z` | `[0, 3, 5]` | `(+1, −1, −1) / √3` | +1 (std) |
  | f2   | `-x+y-z` | `[1, 2, 5]` | `(−1, +1, −1) / √3` | +1 (std) |
  | f3   | `-x-y+z` | `[1, 3, 4]` | `(−1, −1, +1) / √3` | +1 (std) |
  | f4   | `+x+y-z` | `[0, 5, 2]` | `(+1, +1, −1) / √3` | −1 (flip) |
  | f5   | `+x-y+z` | `[0, 4, 3]` | `(+1, −1, +1) / √3` | −1 (flip) |
  | f6   | `-x+y+z` | `[1, 4, 2]` | `(−1, +1, +1) / √3` | −1 (flip) |
  | f7   | `-x-y-z` | `[1, 5, 3]` | `(−1, −1, −1) / √3` | −1 (flip) |

### Empty-mesh edge case (`verify_empty_mesh`)

- `SignedDistanceField::new(empty) == Err(SdfError::EmptyMesh)`.
- `signed_distance(p, &empty) == f64::MAX` exact.
- `unsigned_distance(p, &empty) == f64::MAX` exact.

### SDF query points (`verify_sdf_query_points`)

14 queries; analytical SDF `(|x|+|y|+|z| − 1) / √3` for face-region
queries, vertex-clamped Euclidean for vertex-direction queries.

| Group | Query                           | signed_distance       | closest_point          | is_inside |
|-------|---------------------------------|-----------------------|------------------------|-----------|
| 1     | `(0.05, 0.07, 0.11)` (generic)  | `−0.77 / √3 ≈ −0.4446` | `(0.3067, 0.3267, 0.3667)` = `p + 0.77/3`   | true      |
| 2 ×6  | `±(0.3, 0.3, 0.3)` (6 octants)  | `−0.1 / √3 ≈ −0.0577` | `(±1/3, ±1/3, ±1/3)` (face centroid)        | true      |
| 3 ×6  | `(±2, 0, 0)`, `(0, ±2, 0)`, `(0, 0, ±2)` | `+1.0` exact | `(±1, 0, 0)`, `(0, ±1, 0)`, `(0, 0, ±1)` (vertex clamp) | false |
| 4     | `(10, 10, 10)` (far)            | `+29 / √3 ≈ +16.7432` | `(1/3, 1/3, 1/3)` (`f0` centroid)           | false     |

Anchors at `1e-12` per group.

### Direct geometric primitives

Reference triangle: `v0 = (0, 0, 0)`, `v1 = (10, 0, 0)`, `v2 = (5, 10, 0)`.

`closest_point_on_triangle` — Ericson's algorithm, 3 region cases:

| Query             | Region    | Returned closest_point |
|-------------------|-----------|------------------------|
| `(5, 3, 5)`       | face      | `(5, 3, 0)` (xy projection inside triangle) |
| `(-5, -5, 0)`     | vertex A  | `v0 = (0, 0, 0)`       |
| `(5, -5, 0)`      | edge AB   | `(5, 0, 0)` (perpendicular onto edge) |

`ray_triangle_intersect` — Möller-Trumbore, 3 cases:

| origin           | direction          | result        |
|------------------|--------------------|---------------|
| `(5, 3, 5)`      | `(0, 0, −1)`       | `Some(5.0)`   |
| `(100, 100, 5)`  | `(0, 0, −1)`       | `None` (miss) |
| `(5, 3, 5)`      | `(1, 0, 0)`        | `None` (parallel — `t > 0` cannot reach z=0 plane) |

`point_segment_distance_squared` — segment from `(0, 0, 0)` to `(10, 0, 0)`:

| Query        | Closest segment-point | dist²  |
|--------------|------------------------|--------|
| `(5, 5, 0)`  | `(5, 0, 0)` (perpendicular drop) | `25.0` |
| `(-5, 0, 0)` | `(0, 0, 0)` (clamped to endpoint a) | `25.0` |

Anchors at `1e-12`.

### One-shot equivalence (`verify_one_shot_equivalence`)

`signed_distance(p, mesh)` and `unsigned_distance(p, mesh)` free
fns walk the same `compute_face_normals` + `closest_point_on_triangle`
+ face-normal-dot path as `SignedDistanceField::distance` /
`unsigned_distance`, so they produce **bit-equivalent** outputs
(`f64::to_bits()` equality) on the same `(point, mesh)` input. 4
queries verified.

### `point_in_mesh` boolean (`verify_point_in_mesh`)

- `point_in_mesh((0.05, 0.07, 0.11), octahedron) == true` (off-axis
  interior; +X ray hits `f0` interior at one point → odd count).
- `point_in_mesh((10, 10, 10), octahedron) == false` (far
  exterior; +X ray misses all faces → 0 hits).

### Bulk grid (`verify_bulk_query_stats`)

10 × 10 × 10 cubic grid in `[-2, 2]³`, spacing `4 / 9`,
endpoint-inclusive at `−2` and `2`. Two inside-counters disagree
by design (see "Bulk-grid drift-10" callout above):

| Method                                   | Count   | Percent |
|------------------------------------------|---------|---------|
| `point_in_mesh` (ray-casting)            | **8**   | 0.8%    |
| `signed_distance < 0` (face-normal sign) | **14**  | 1.4%    |

Continuous-volume sanity check: the unit octahedron has volume
`(4/3) · r³` (L1-ball), giving continuous fraction
`(4/3) / 64 = 1/48 ≈ 2.083%` — a denser grid would converge
toward this value (drift-9: spec previously asserted `(8/3) · r³`,
off by factor 2; corrected inline at this commit).

The 8 ray-casting "inside" points are exactly the
`(±2/9, ±2/9, ±2/9)` grid corners with `|x| + |y| + |z| = 6/9 < 1`;
the next-nearest combination `2/9 + 2/9 + 6/9 = 10/9 > 1` is
exterior. The 6 signed-distance false-positives are at
`(±2, ±2/3, ∓2/3)`-permutation grid points where 4 faces tie on
the closest octahedron vertex (see drift-10 above).

Max unsigned distance: `5 / √3 ≈ 2.887`, achieved at all 8 bbox
corners (e.g., `(−2, −2, −2)` projects to `(−1/3, −1/3, −1/3)` on
`f7`'s plane). Anchored at `1e-12`.

## Visuals

`out/octahedron.ply` (ASCII, 6 verts + 8 tris, 312 bytes) — open in
f3d to verify the face windings visually:

```text
f3d examples/mesh/mesh-sdf-distance-query/out/octahedron.ply
```

`out/sdf_grid.ply` (binary LE, 1000 verts + 0 faces, ~16KB) — point
cloud with per-vertex `extras["signed_distance"]` scalar. Open in a
viewer that understands custom PLY attributes (Paraview, Meshlab,
or any field-data colormap renderer) to see the SDF's 8-fold
symmetry: the 8 strictly-interior grid points at
`(±2/9, ±2/9, ±2/9)` read with small-magnitude negative signed
distance (`-1/(3√3) ≈ -0.192`); a smooth positive gradient extends
toward the bbox corners up to `5/√3 ≈ 2.887`; and the 6 drift-10
false-positives at the `(±2, ±2/3, ∓2/3)`-permutation grid points
also register as negative (`-√17/3 ≈ -1.374`) due to the vertex-
region face-normal sign-flip. Color by `signed_distance` for the
canonical visualization; the sign-divergence between the true L1-
ball interior and the 6 bbox-edge false-positives is visible as
isolated negative-color points along the bbox boundary.

## Run

```text
cargo run -p example-mesh-mesh-sdf-distance-query --release
```

Output: `out/octahedron.ply` (input mesh) +
`out/sdf_grid.ply` (1000-vertex grid). Stdout prints input
fixture summary, the empty-mesh `Err` / `f64::MAX` behavior, all
4 SDF query groups, the 3 direct-primitive regions, the
`point_in_mesh` boolean queries, and the dual bulk-grid inside-
counter values + max unsigned distance.

## Cross-references

- **Spec**: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.4 (this example) +
  §7 R5 / R10 (corner / winding risks) + §10 item 8 (v0.9
  pseudo-normal / winding-number trigger; expanded by drift-10
  inline at this commit). Drift-9 (volume formula `(4/3)·r³` not
  `(8/3)·r³`) lands inline at §5.4 line 572.
- **Sister examples** rounding out the v1.0 mesh-arc:
  `mesh-measure-bounding-box` (§5.1) at `719a85d3`,
  `mesh-measure-cross-section` (§5.2) at `021a9712`,
  `mesh-measure-distance-to-mesh` (§5.3) at `4650058a`. The §5.3
  unsigned `distance_to_mesh` uses an internal
  `closest_point_on_triangle` that **duplicates** mesh-sdf's free
  fn (spec §10 item 7 v0.9 dedup candidate).
- **PLY-attribute pattern**: `examples/mesh/ply-with-custom-attributes/`
  — same `save_ply_attributed` + `extras["<scalar>"]` pattern; this
  example reuses it for the bulk-grid output (vertices-only, 0
  faces).
- **Mesh book**: `docs/studies/mesh_architecture/src/80-examples.md`
  — Part 8 inventory (depth-pass updates land at `§6.2 #31` of the
  arc).
- **Inside-test caveats** consolidated:
  - HE-1 origin corner-degeneracy → use off-axis interior queries.
  - drift-10 vertex-region face-normal sign-flip → prefer
    `point_in_mesh` (ray-casting) or future winding-number
    convention (v0.9 item 8).
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
  [`feedback_examples_drive_gap_fixes`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_examples_drive_gap_fixes.md),
  [`feedback_thorough_review_before_commit`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_thorough_review_before_commit.md),
  [`feedback_risk_mitigation_review`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_risk_mitigation_review.md).
