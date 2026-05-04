# sphere-sdf-eval

**`Sdf` trait contract on `SphereSdf` — analytic signed distance and
unit-length gradient on the entire domain except the documented
`Vec3::z()` fallback at the origin singularity.** Unit sphere
`SphereSdf { radius: 1.0 }` centred at the origin. Asserts
`eval(p) = ‖p‖ − 1` and `grad(p) = p / ‖p‖` at axis-aligned + off-axis
+ Pythagorean-triple anchors, then sweeps an 11³ = 1331-point grid in
`[−2, 2]³` at spacing 0.4 to verify the bit-exact identity at every
sample and emit `out/sdf_grid.ply` with `extras["signed_distance"]`
for external colormap rendering.

## What this example demonstrates

The unit sphere has the simplest closed-form SDF in `sim-soft`:
`eval(p) = ‖p‖ − 1` and `grad(p) = p / ‖p‖`. Both are bit-exact at
axis-aligned points (`(±1, 0, 0)`, etc.) and at Pythagorean-triple
points (`(3, 4, 0)`, `(0, 3, 4)`) where the implied square-roots
are integer, and approximate within `1e-15` everywhere else. The
example exercises the public `Sdf` trait surface — `Sdf::eval` and
`Sdf::grad` only — across all three regions (interior, surface,
exterior) plus the documented origin singularity, then bulk-validates
the same identity at 1331 grid points.

**Why a sphere, not a more elaborate primitive**: the sphere is the
only `Sdf` impl with no compositional moving parts (no boolean
combinations, no transforms, no warp). It IS the trait contract in
its simplest form. Boolean SDF composition lands in the sister
example `hollow-shell-sdf` (row 2 of the inventory) via
`DifferenceSdf`.

## Numerical anchors

Each anchor is encoded as `assert_relative_eq!` in `src/main.rs`
under `verify_surface_eval_axis_aligned`,
`verify_surface_eval_off_axis`, `verify_interior_eval`,
`verify_exterior_eval`, `verify_gradient_unit_length`,
`verify_gradient_direction`, `verify_origin_singularity`, and
`verify_grid_consistency`. Per
[`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
a clean `cargo run --release` exit-0 IS the correctness signal; the
visuals pass below is optional pedagogy.

Two tolerance regimes:

- **`epsilon = 0.0`** (`EXACT_TOL` — bit-exact) where the arithmetic
  admits it: axis-aligned surface points (`√1 = 1`), origin
  (`‖0‖ = 0`), dyadic interior (`√0.25 = 0.5`), Pythagorean-triple
  exterior (`√25 = 5`), Pythagorean gradient (`(3, 4, 0) / 5 =
  (0.6, 0.8, 0.0)`).
- **`epsilon = 1e-15`** (`APPROX_TOL`) where coordinates involve
  non-dyadic divisors (`1/√3`) or compound roots (`√300`).

### Surface (`eval = 0`)

| Query              | Tolerance | Expected |
|--------------------|-----------|----------|
| `(±1, 0, 0)`       | bit-exact | 0        |
| `(0, ±1, 0)`       | bit-exact | 0        |
| `(0, 0, ±1)`       | bit-exact | 0        |
| `(1/√3, 1/√3, 1/√3)` | 1e-15   | 0        |

### Interior (`eval < 0`)

| Query              | Tolerance | Expected      |
|--------------------|-----------|---------------|
| `(0, 0, 0)`        | bit-exact | −1            |
| `(0.5, 0, 0)`      | bit-exact | −0.5          |
| `(0, 0, 0.5)`      | bit-exact | −0.5          |
| `(0.5, 0.5, 0.5)`  | 1e-15     | √0.75 − 1     |

### Exterior (`eval > 0`)

| Query              | Tolerance | Expected      |
|--------------------|-----------|---------------|
| `(2, 0, 0)`        | bit-exact | +1            |
| `(3, 4, 0)`        | bit-exact | +4 (norm = 5) |
| `(0, 3, 4)`        | bit-exact | +4 (norm = 5) |
| `(10, 10, 10)`     | 1e-15     | √300 − 1      |

### Gradient unit-length (`‖grad‖ = 1`)

- 6 axis-aligned anchors at bit-exact tolerance (the gradient is the
  unit axis vector, whose norm is exactly 1).
- 4 off-axis anchors `{(1/√3)·(1,1,1), (0.5,0.5,0.5), (3,4,0),
  (10,10,10)}` at 1e-15 tolerance.

### Gradient direction (`grad(p) = p / ‖p‖`)

- `grad(±e_i) = ±e_i` bit-exact (3 axes × 2 signs).
- `grad(2 e_x) = e_x` bit-exact (direction is invariant under
  positive magnitude; `2 / 2 = 1`).
- `grad((3, 4, 0)) = (0.6, 0.8, 0.0)` bit-exact (Pythagorean —
  `3 / 5 = 0.6` and `4 / 5 = 0.8` are correctly-rounded f64
  dyadics).
- `grad((1, 1, 1)) = (1/√3)·(1, 1, 1)` at 1e-15.

### Origin singularity (documented `Vec3::z()` fallback)

- `grad((0, 0, 0)) = Vec3::z() = (0, 0, 1)` bit-exact.

`SphereSdf::grad`'s impl notes that the centre is an interior
singularity where `p / ‖p‖` is undefined; the fallback choice is
unobservable downstream because the centre lives strictly inside
the SDF, far from the zero set the mesher actually queries (see
`sim/L0/soft/src/sdf_bridge/sdf.rs:55-68`). Pinning the fallback
in this example guards the bulk-grid pipeline against silent
behavioural drift if the documented choice ever changes.

### Bulk-grid identity (`verify_grid_consistency`)

11³ = **1331** grid points in `[−2, +2]³` at spacing 0.4
(endpoint-inclusive). The identity `eval(p) == ‖p‖ − 1` is asserted
**bit-exact** at every grid point: both sides do the same
component-wise FP arithmetic, so the equality holds at `epsilon =
0.0`. Gradient norm asserted at `1e-15` for every non-origin point
and at bit-exact tolerance at the centre (where the `Vec3::z()`
fallback has norm 1 exactly).

The grid spacing 0.4 puts the origin at index `(5, 5, 5)` (the
documented singularity), exercises 7 lattice radii inside the unit
sphere (origin + 6 axis half-shells at radius 0.4), and reaches
out to bbox corners at distance `2√3 ≈ 3.464` from origin (eval
≈ +2.464).

Per-bucket counts printed by `print_summary` (interior / surface /
exterior) are emitted to stdout for the museum-plaque summary; the
counts are derived from grid sampling, not asserted as anchors,
since they are determined entirely by the deterministic grid +
sphere-radius choice.

## Visuals

`out/sdf_grid.ply` (binary LE, 1331 verts + 0 faces, ~16 KB) — point
cloud with per-vertex `extras["signed_distance"]` scalar. Open in a
viewer that understands custom PLY attributes:

```text
# MeshLab — Render → Color → Per-vertex Quality, then
# Filters → Quality Mapper applied to "signed_distance"
meshlab examples/sim-soft/sphere-sdf-eval/out/sdf_grid.ply

# ParaView — open the PLY, then in the Coloring panel pick
# the "signed_distance" array and apply a divergent colormap
paraview examples/sim-soft/sphere-sdf-eval/out/sdf_grid.ply
```

What you should see when colour-mapped by `signed_distance`:

- **Spherically symmetric radial gradient** centred on the origin.
- **Negative interior** (small grid-cluster near origin — 7 inside
  points at the origin + the 6 axis-half-shell points).
- **Zero on the unit sphere surface** — no grid point lands exactly
  on the surface (spacing 0.4 doesn't intersect the sphere at any
  lattice node), so the "surface band" appears as the colormap's
  zero-crossing isosurface inferred from the surrounding gradient.
- **Positive exterior** (the bulk of the grid), maxing out at the
  bbox corners (`(±2, ±2, ±2)`) at value `√12 − 1 = 2√3 − 1 ≈
  +2.464`.

## Run

```text
cargo run -p example-sim-soft-sphere-sdf-eval --release
```

Output: `out/sdf_grid.ply` (1331-vertex grid). Stdout prints input
fixture summary, all 7 anchor-group names, and the bulk-grid
interior / surface / exterior counts.

## Cross-references

- **Sister example** rounding out the SDF / meshing tier:
  `hollow-shell-sdf` (boolean SDF composition via `DifferenceSdf`)
  and `sdf-to-tet-sphere` (BCC + Isosurface Stuffing tet meshing).
  See `sim/L0/soft/EXAMPLE_INVENTORY.md` Tier 1.
- **PLY-attribute pattern** mirrored from
  `examples/mesh/mesh-sdf-distance-query` (1000-point bulk grid +
  `extras["signed_distance"]`) and
  `examples/mesh/ply-with-custom-attributes` (`save_ply_attributed`
  + `extras["<scalar>"]` round-trip).
- **`SphereSdf` impl**:
  `sim/L0/soft/src/sdf_bridge/sdf.rs` — analytic eval / grad +
  documented origin singularity.
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
  [`feedback_one_at_a_time_review`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_one_at_a_time_review.md),
  [`feedback_thorough_review_before_commit`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_thorough_review_before_commit.md).
