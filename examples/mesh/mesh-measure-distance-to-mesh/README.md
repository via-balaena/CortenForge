# mesh-measure-distance-to-mesh

**Point-to-point + point-to-mesh distance + symmetric Hausdorff
between two meshes — composed from the `mesh-measure` primitives.**
Hand-authors two SEPARATE axis-aligned unit-cube `IndexedMesh`
instances (`cube_a` at `[0, 1]³`, `cube_b` at `[2, 3]³`) and
exercises the full `mesh-measure::measure_distance` /
`closest_point_on_mesh` / `distance_to_mesh` public surface.
Demonstrates that `mesh-measure` exposes the primitives;
**symmetric Hausdorff** between two meshes is achievable as
composition over those primitives (NOT a built-in).

## What it does

Builds two vertex-disjoint axis-aligned unit cubes, each as its
own `IndexedMesh`:

- **cube_a**: `[0, 1] × [0, 1] × [0, 1]` (8 verts, 12 tris).
- **cube_b**: `[2, 3] × [2, 3] × [2, 3]` (8 verts, 12 tris).

Closest-face gap along `+X` is `1.0` (cube_a's `+X` face at `x=1`
to cube_b's `−X` face at `x=2`); farthest-corner pair `(0, 0, 0)`
↔ `(3, 3, 3)` is at distance `3√3 ≈ 5.196`. The two cubes live in
distinct `IndexedMesh` instances so the Hausdorff iteration calls
`distance_to_mesh(&cube_a, _)` and `distance_to_mesh(&cube_b, _)`
on independent geometry. The combined mesh `out/two_cubes.ply`
is for visualization only.

Then computes:

1. `measure_distance((0, 0, 0), (3, 4, 0))` — point-to-point
   Euclidean (3-4-5 Pythagorean triangle); `distance == 5.0`,
   `dx == 3.0`, `dy == 4.0`, `dz == 0.0`.
2. `direction_normalized()` returns `Some((0.6, 0.8, 0.0))`;
   `midpoint()` returns `(1.5, 2.0, 0.0)`; `direction_normalized()`
   on a zero-distance measurement returns `None`.
3. `distance_to_mesh(&cube_a, p)` for 6 face-direction queries
   (each `1.0`), 8 corner-direction queries (each `sqrt(3)`),
   and 1 center query (`0.5`). `closest_point_on_mesh` returns
   the corresponding face-center / corner / bottom-projection
   point per query.
4. `distance_to_mesh(&empty, p) == None` and
   `closest_point_on_mesh(&empty, p) == None` on an empty
   `IndexedMesh::new()`.
5. **Symmetric Hausdorff** between cube_a and cube_b — composed
   from `distance_to_mesh` over each cube's vertex set:
   `H(A, B) = max(d_H(A→B), d_H(B→A)) = sqrt(12) ≈ 3.464`. Both
   one-sided maxima hit `sqrt(12)` at the diagonally-opposite
   corner pair: `cube_a.(0, 0, 0)` clamps to `cube_b.(2, 2, 2)`,
   and `cube_b.(3, 3, 3)` clamps to `cube_a.(1, 1, 1)`.

Then writes `out/two_cubes.ply` (the combined fixture) and
asserts every numerical anchor below.

## `dx` / `dy` / `dz` are absolute values

Per `mesh-measure/src/distance.rs:97-99`,
`DistanceMeasurement::{dx, dy, dz}` are the **absolute** componentwise
distances, not the signed differences:

```text
measure_distance(from=(0, 0, 0), to=(-3, -4, 0))
  → distance = 5.0, dx = 3.0, dy = 4.0, dz = 0.0
```

The point-to-point verifier exercises only the all-positive
direction `(0, 0, 0) → (3, 4, 0)` so the absolute-value behavior
is implicit; this README surfaces the convention explicitly.

## Numerical anchors

Each anchor is encoded as `assert_relative_eq!` (or `assert_eq!`
for counts) in `src/main.rs` under `verify_cube_geometry`,
`verify_point_to_point`, `verify_point_to_mesh_face_directions`,
`verify_point_to_mesh_corner_directions`,
`verify_point_to_mesh_center`, `verify_empty_mesh`, and
`verify_hausdorff`. Per
[`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
a clean `cargo run --release` exit-0 == clean visual inspection.

### Fixture geometry (`verify_cube_geometry` × 2)

- 16 per-vertex coordinate anchors at `1e-12` (8 corners per cube;
  FP-exact integer `(0|1, 0|1, 0|1)` for cube_a and
  `(2|3, 2|3, 2|3)` for cube_b).
- 24 per-face winding cross-product unit-normal anchors at `1e-12`
  (12 per cube; analytical ±x / ±y / ±z directions). Cross
  products on integer coordinates are bit-exact.

### Point-to-point (`verify_point_to_point`)

- `measure_distance((0, 0, 0), (3, 4, 0))` →
  `distance == 5.0` exact; `dx == 3.0`, `dy == 4.0`, `dz == 0.0`
  exact (`3² + 4² = 25 = 5²`, all integer arithmetic).
- `direction_normalized()` returns `Some((0.6, 0.8, 0.0))` to
  `1e-12` (rationals from `3/5` and `4/5`; binary
  representation rounded to 1 ULP).
- `midpoint()` returns `(1.5, 2.0, 0.0)` exact.
- `direction_normalized()` on a zero-distance measurement returns
  `None`.

### Point-to-mesh — face directions (`verify_point_to_mesh_face_directions`)

6 queries placed at the cube-face center, offset by `1.0` along
the outward normal:

| Query              | Expected closest point | Distance |
|--------------------|------------------------|----------|
| `(2.0, 0.5, 0.5)`  | `(1.0, 0.5, 0.5)`      | `1.0`    |
| `(-1.0, 0.5, 0.5)` | `(0.0, 0.5, 0.5)`      | `1.0`    |
| `(0.5, 2.0, 0.5)`  | `(0.5, 1.0, 0.5)`      | `1.0`    |
| `(0.5, -1.0, 0.5)` | `(0.5, 0.0, 0.5)`      | `1.0`    |
| `(0.5, 0.5, 2.0)`  | `(0.5, 0.5, 1.0)`      | `1.0`    |
| `(0.5, 0.5, -1.0)` | `(0.5, 0.5, 0.0)`      | `1.0`    |

Anchors at `1e-12`; FP-exact integer projections.

### Point-to-mesh — corner directions (`verify_point_to_mesh_corner_directions`)

8 queries placed `1.0` outward from cube_a along all three axes
simultaneously, one per cube corner. Each closest point is the
corresponding corner; distance is `sqrt(3)` (Pythagorean of three
unit offsets).

| Query               | Expected closest corner | Distance   |
|---------------------|-------------------------|------------|
| `(2, 2, 2)`         | `(1, 1, 1)`             | `sqrt(3)`  |
| `(-1, -1, -1)`      | `(0, 0, 0)`             | `sqrt(3)`  |
| `(2, 2, -1)`        | `(1, 1, 0)`             | `sqrt(3)`  |
| `(-1, -1, 2)`       | `(0, 0, 1)`             | `sqrt(3)`  |
| `(2, -1, 2)`        | `(1, 0, 1)`             | `sqrt(3)`  |
| `(-1, 2, -1)`       | `(0, 1, 0)`             | `sqrt(3)`  |
| `(-1, 2, 2)`        | `(0, 1, 1)`             | `sqrt(3)`  |
| `(2, -1, -1)`       | `(1, 0, 0)`             | `sqrt(3)`  |

Anchors at `1e-12`.

### Point-to-mesh — center query (`verify_point_to_mesh_center`)

- Query `(0.5, 0.5, 0.5)` (cube centroid) → `distance == 0.5`.
  `distance_to_mesh` is **unsigned**: it returns the closest-face
  distance regardless of inside / outside semantics. Caller must
  use a separate inside-test (e.g., `mesh-sdf::point_in_mesh`)
  for signed queries.
- Closest point is `(0.5, 0.5, 0.0)` on the bottom face — the
  query iteration in `closest_point_on_mesh` (per
  `distance.rs:141-149`) walks `mesh.triangles()` in face-emission
  order; the bottom-face tris emit first, and the strict `<` tie-
  break keeps the first triangle that ties on `dist_sq`. (All
  6 face centers tie at `0.5` from the cube centroid; the bottom
  face wins on iteration order.)

### Empty-mesh edge case (`verify_empty_mesh`)

- `distance_to_mesh(&empty, p) == None` per `distance.rs:134-136`
  (`mesh.faces.is_empty()` short-circuit).
- `closest_point_on_mesh(&empty, p) == None` from the same
  short-circuit.

### Symmetric Hausdorff (`verify_hausdorff`)

The example exposes a `one_sided_hausdorff(from, to)` helper that
iterates every vertex of `from` and computes
`distance_to_mesh(to, v)`, returning the maximum. The symmetric
Hausdorff is `max(d_H(A→B), d_H(B→A))`.

For the two-cube fixture:

- `d_H(cube_a → cube_b) = sqrt(12) ≈ 3.464` — achieved at
  `cube_a.(0, 0, 0)`, the corner of A diagonally-opposite from
  cube_b. Closest point on cube_b is the **clamped corner**
  `(2, 2, 2)`: cube_b's `+X` face at `x = 2` has `(y, z) ∈ [2, 3]²`,
  which excludes the projection `(0, 0)` — the closest point
  clamps to the corner, NOT to the face-interior `(2, 0, 0)`.
  Square distance `4 + 4 + 4 = 12`.
- `d_H(cube_b → cube_a) = sqrt(12)` by symmetry — `cube_b.(3, 3, 3)`
  clamps to `cube_a.(1, 1, 1)`.
- `H(A, B) = max(d_AB, d_BA) = sqrt(12)`.

Anchors at `1e-12` (deterministic Pythagorean of integer offsets).

> The `sqrt(12)` vs naive `sqrt(8)` clamped-corner correction came
> from spec §8 round 1. The `sqrt(8)` answer projects `(0, 0)` onto
> the `+X` face interior at `(2, 0, 0)` — but the face's
> `(y, z) ∈ [2, 3]²` bounds exclude that point, so the projection
> clamps to the corner `(2, 2, 2)` instead.

## Visuals

`out/two_cubes.ply` contains the 16-vertex / 24-triangle combined
mesh (cube_a at `[0, 1]³` + cube_b at `[2, 3]³`). Open in f3d to
see two axis-aligned unit cubes side-by-side along the body
diagonal:

```text
f3d examples/mesh/mesh-measure-distance-to-mesh/out/two_cubes.ply
```

Low visual novelty (two unit cubes); the math-pass-first verifiers
encode every property, so a clean `cargo run --release` exit-0 is
the primary correctness signal.

## Run

```text
cargo run -p example-mesh-mesh-measure-distance-to-mesh --release
```

Output: `examples/mesh/mesh-measure-distance-to-mesh/out/two_cubes.ply`
(ASCII PLY, 16 verts + 24 tris). Stdout prints input fixture
summary, the 3-4-5 point-to-point measurement, sample point-to-mesh
queries (1 face + 1 corner + 1 center), the empty-mesh
`Option::None` behavior, and the symmetric Hausdorff value with
the clamped-corner explanation.

## Cross-references

- **Spec**: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.3 (this example)
  + §8 round 1 (Hausdorff sqrt(12) vs naive sqrt(8) clamped-corner
  correction).
- **Sister examples** (round out `mesh-measure` public-surface
  coverage): `mesh-measure-bounding-box` (§5.1) shipped at
  `719a85d3`; `mesh-measure-cross-section` (§5.2) shipped at
  `021a9712`.
- **Mesh book**: `docs/studies/mesh_architecture/src/80-examples.md`
  Part 8 — depth-pass lands in `§6.2 #31` of the arc.
- **Inside-test caveat**: `distance_to_mesh` is unsigned. For
  signed-distance + inside-test queries, use `mesh-sdf` —
  `mesh-sdf-distance-query` (§5.4) covers
  `SignedDistanceField::is_inside` + `distance` (signed) + bulk
  query patterns. (Note `mesh-sdf::query::closest_point_on_triangle`
  duplicates `mesh-measure::distance::closest_point_on_triangle`
  internally; spec §10 item 7 v0.9 dedup candidate.)
