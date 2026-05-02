# mesh-measure-bounding-box

**AABB and OBB on the same mesh — when do they coincide, when do they
diverge?** Hand-authors a two-shape fixture (an axis-aligned cube + a
rotated brick) and exercises the full `mesh-measure::dimensions` and
`oriented_bounding_box` public surface. Demonstrates two pedagogical
points: (1) on the combined mesh, OBB finds a modest principal-axis
tilt and reduces volume by ~5% over AABB; (2) on the rotated brick
alone, OBB cleanly recovers the original `(20, 10, 10)` extents
because PCA's eigenvalues are non-degenerate when the input shape is
non-cubic. Closes the v1.0 coverage gap for `mesh-measure`'s
bounding-box surface.

## What it does

Builds a 16-vertex / 24-triangle fixture with two vertex-disjoint
shapes:

- **Cube A** — an axis-aligned 10 mm cube at
  `[0, 10] × [-5, 5] × [0, 10]` (y-centered on the origin so cube A's
  y-range sits inside the brick's; combined-mesh AABB then has clean
  `center.y = 0`).
- **Cube B** — a `20 × 10 × 10` mm tilted brick (long axis along
  local +X, then rotated 45° around Z) translated so its bbox-center
  is `(25, 0, 5)`. The 2:1 aspect ratio is load-bearing: a
  `10×10×10` cube has covariance `25·I` (scalar matrix) and PCA
  cannot uniquely identify the rotation. The brick's covariance has
  one large eigenvalue (long axis) and two equal small ones (short
  axes), so the principal axis is unique up to a sign — and OBB
  recovers the box.

The example computes:

1. `dimensions(mesh)` — the AABB-derived `Dimensions` struct on the
   combined mesh.
2. `oriented_bounding_box(mesh)` — the PCA-derived OBB on the
   combined mesh.
3. `oriented_bounding_box(brick_mesh)` — the same OBB call on cube B
   alone (extracted as a no-faces sub-mesh from
   `mesh.vertices[8..16]`).

Then writes `out/mesh.ply` (the input mesh) and asserts every
numerical anchor below.

## Numerical anchors

Each anchor is encoded as `assert_relative_eq!` in `src/main.rs` under
`verify_dimensions` (the AABB anchors), `verify_combined_obb` (the
combined-mesh OBB anchors), and `verify_brick_obb` (the brick-alone
OBB anchors). Per
[`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
a clean `cargo run --release` exit-0 == clean visual inspection.

### Fixture geometry (`verify_fixture_geometry`)

- 16 per-vertex coordinate anchors at `1e-12` (cube A's 8 corners +
  cube B's 8 brick corners). Cube B vertex coords use
  `s = f64::sqrt(0.5)` for both cosine and sine of the 45° rotation
  matrix per spec §4.4 — `f64::sqrt` is correctly-rounded per
  IEEE-754, while `f64::sin(π/4)` and `f64::cos(π/4)` are not, and
  the 1-ULP divergence would defeat the `1e-12` tolerance.
- 24 per-face winding cross-product unit-normal anchors at `1e-12`
  (CCW from outside; outward normals on both shapes verified).

### AABB on combined mesh (`verify_dimensions`)

Combined AABB: `[0, 25 + 7.5√2] × [-7.5√2, 7.5√2] × [0, 10]`.

- `width = 25 + 7.5√2 ≈ 35.61`, `depth = 15√2 ≈ 21.21`,
  `height = 10`.
- `bounding_volume = 3750·√2 + 2250 ≈ 7553.30`.
- `diagonal ≈ 42.64`, `center = ((25 + 7.5√2)/2, 0, 5) ≈ (17.80, 0, 5)`.
- `min_extent = 10`, `max_extent ≈ 35.61`,
  `aspect_ratio ≈ 3.56`, `is_cubic(0.01) == false`.

All anchors hold to `1e-12`.

### OBB on combined mesh (`verify_combined_obb`)

The combined fixture's covariance has off-diagonal `Cov(x, y) ≠ 0`
(brick elongation breaks y-symmetry); PCA yields three distinct
eigenvalues and a principal axis at `atan(3/8)/2 ≈ 10.28°` from
world +X.

- `volume ≈ 7169.48` mm³ (5.08% reduction vs AABB; PCA finds
  compromise tilt) — within 5% per spec §5.1 OBB volume tolerance.
- `axis_x ≈ (0.984, 0.178, 0)` within `1e-9`; principal axis ≈
  `10.28°` from world +X.
- `axis_x ⊥ axis_y ⊥ axis_z` within `1e-9` (rotation orthogonality).
- `surface_area > 0`.
- All 16 input vertices satisfy `|local_offset| ≤ half_extents +
  1e-9` (tolerance-aware — strict `obb.contains(v)` can fail by 1
  ULP for the 4 brick vertices that defined the OBB extremes; this
  is captured as a v0.9 candidate gap in `mesh-measure`).

> **No "8 OBB corners ⊂ inflated AABB" anchor** here. That folk
> intuition (OBB ⊆ AABB) is false for any non-trivial OBB rotation
> — corners extend OUTSIDE the AABB by `half_extent · sin(angle)`.
> The 16-vertex `obb.contains()` anchor is the proper enclosure
> test. Captured as v0.9 doc/example-pattern fix in `mesh-measure`.

### OBB on cube B alone (`verify_brick_obb`)

Cube B sub-mesh = the 8 brick vertices, no faces. PCA on the brick:
covariance has eigenvalues `(33.33, 8.33, 8.33)` — long-axis
distinct, two short axes equal. The principal direction is uniquely
the rotated +X (45° from world +X).

- `volume = 2000` mm³ exact (recovers `BRICK_LONG · BRICK_SHORT ·
  BRICK_SHORT`) within `1e-9`.
- Sorted extents `(20, 10, 10)` within `1e-9`.
- `center = (25, 0, 5)` within `1e-9`.
- `surface_area = 2·(20·10 + 20·10 + 10·10) = 1000` mm².
- All 8 brick vertices contained (tolerance-aware).

The pedagogical contrast: the brick's AABB has volume `(15√2)² · 10
= 4500` mm³ (2.25× the brick's actual `2000`); the OBB recovers the
brick exactly. This is the textbook "AABB inflates a rotated shape;
OBB recovers it" demonstration — and it requires a non-cubic input
to hold (drift caught and resolved in v1.0 spec-authoring; see
spec §10 v0.9 backlog items 9 and 10 for the surfaced gaps).

## Visuals

`out/mesh.ply` contains the 16-vertex / 24-triangle fixture. Open in
f3d to see the contrast:

```text
f3d examples/mesh/mesh-measure-bounding-box/out/mesh.ply
```

Cube A renders as a 10 mm cube near the world origin; cube B renders
as an elongated diamond profile when viewed from above (the rotated
20×10 footprint), positioned at `x ≈ 25`. Both shapes' tops and
bottoms remain at `z = 0` and `z = 10` (the rotation is around Z,
preserving the vertical extent).

## Run

```text
cargo run -p example-mesh-mesh-measure-bounding-box --release
```

Output: `examples/mesh/mesh-measure-bounding-box/out/mesh.ply` (ASCII
PLY, 16 verts + 24 tris). Stdout prints AABB + OBB summaries with
the volume reduction percentage and principal-axis angle.

## Cross-references

- **Spec**: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.1 (this example) +
  §4.4 (FP-exact rotation via `f64::sqrt(0.5)`) + §10 items 9/10
  (v0.9 candidate gaps surfaced by this example).
- **Sister examples** (round out `mesh-measure` public-surface
  coverage): `mesh-measure-cross-section` (§5.2),
  `mesh-measure-distance-to-mesh` (§5.3).
- **Mesh book**: `docs/studies/mesh_architecture/src/80-examples.md`
  — Part 8 inventory of v1.0 examples (depth-pass lands in spec
  §6.2 commit 31).
- **Surfaced platform gaps** (v0.9 candidates in
  `mesh-measure::oriented_bounding_box`):
  1. Strict `OrientedBoundingBox::contains(v)` returns `false` for
     OBB-extreme vertices by 1 ULP — needs tolerance-aware
     `contains_with_tol`.
  2. Folk intuition "OBB ⊆ AABB" is false for non-trivial rotations
     — rustdoc should clarify, and the "corners within AABB" anchor
     pattern should not appear in user-facing examples.
