# hollow-shell-sdf

**Sharp-CSG difference combinator on `SphereSdf` — `SphereSdf{R_OUTER=1.0}
\ SphereSdf{R_CAVITY=0.5}` composed via `DifferenceSdf` (book Part 7
§00 §01). The result is a thick-walled hollow shell — body interior
between the two operand surfaces, cavity (inside the inner sphere)
and exterior (outside the outer sphere) both *outside* the body.** A
2-D xy-plane slice at z = 0 is emitted as `out/sdf_slice.ply` (49² =
2401 vertices) with two per-vertex scalars — `signed_distance`
(analytic SDF) and `active_branch` (which operand is selected by the
`max(φ_a, -φ_b)` evaluator) — for cf-view colormap rendering.

## What this example demonstrates

`DifferenceSdf::eval(p) = max(φ_a(p), -φ_b(p))` under the `φ < 0
inside` convention: a point lies inside `A \ B` iff it is inside `A`
and outside `B`. The example wraps two unit-style `SphereSdf`s as the
operands and visually validates the donut cross-section, plus encodes
the **two operand-active branches** as numerical anchors:

- **Outer-active branch (a-branch, `φ_a ≥ -φ_b`)** — wherever the
  point is closer to the outer surface than the cavity surface
  (radially: `|p| > 0.75`). Gradient is `+p/|p|` (radially outward).
- **Cavity-active branch (b-branch, `φ_a < -φ_b`)** — wherever the
  point is closer to the cavity surface (radially: `|p| < 0.75`).
  Gradient is `-p/|p|` (radially **inward**) — this matches the
  `φ < 0 inside` convention's outward-normal semantics: at the cavity
  wall, the body's outward normal points INTO the cavity.

The branch flip happens at the equidistant locus `|p| = (R_OUTER +
R_CAVITY)/2 = 0.75` — a sphere sitting between the two operand
surfaces. The `DifferenceSdf::grad` `≥` tie-break selects the
a-branch deterministically on this locus, so the gradient field stays
a pure function (no two-valued ambiguity at the crease).

**Why these radii**: `R_OUTER = 1.0` and `R_CAVITY = 0.5` are dyadic,
so axis-aligned anchor points (`(±1, 0, 0)` on the outer sphere,
`(±0.5, 0, 0)` on the cavity sphere) land exactly on the zero set in
f64. The mid-shell equidistant locus `0.75 = (1.0 + 0.5)/2` is also
dyadic. This lets every axis-aligned anchor below assert at
`epsilon = 0.0` (bit-exact) and the slice-grid identity hold exactly
at every one of 2401 grid points.

**Why a 2-D slice (vs. row 1's 3-D bulk grid)**: the donut topology is
fundamentally 2-D — `signed_distance` colormapped over a `z = 0`
slice through the centre IS the canonical hollow-shell visual.
Sister example `sphere-sdf-eval` (row 1) emits a 3-D bulk grid
because the unit sphere is rotationally symmetric and a slice would
discard no information; here the slice is the right primitive
because both spheres share the origin and the great-circle slice
through the centre captures the full geometry.

## Numerical anchors

Each anchor is encoded as `assert_relative_eq!` in `src/main.rs`
under `verify_outer_surface_eval`, `verify_cavity_surface_eval`,
`verify_shell_interior_eval`, `verify_cavity_interior_eval`,
`verify_exterior_eval`, `verify_outer_active_grad`,
`verify_cavity_active_grad`, `verify_branch_flip_locus`,
`verify_outer_surface_grad`, `verify_cavity_surface_grad`, and
`verify_slice_consistency`. Per
[`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
a clean `cargo run --release` exit-0 IS the correctness signal; the
visuals pass below is optional pedagogy.

All anchors below are bit-exact (`epsilon = 0.0`) — the radii and
probe points are all chosen dyadic so the FP arithmetic admits exact
equality.

### Outer-surface eval (`eval = 0`)

| Query              | Expected |
|--------------------|----------|
| `(±1, 0, 0)`       | 0        |
| `(0, ±1, 0)`       | 0        |
| `(0, 0, ±1)`       | 0        |
| `(0.6, 0.8, 0)`    | 0 (Pythagorean — `0.36 + 0.64 = 1` exactly) |

### Cavity-surface eval (`eval = 0`)

| Query              | Expected |
|--------------------|----------|
| `(±0.5, 0, 0)`     | 0        |
| `(0, ±0.5, 0)`     | 0        |
| `(0, 0, ±0.5)`     | 0        |

### Shell interior (`eval < 0`)

| Query              | Expected | Active branch |
|--------------------|----------|---------------|
| `(0.75, 0, 0)`     | −0.25    | tie → a (`≥`) |
| `(0.875, 0, 0)`    | −0.125   | a (outer)     |
| `(0.625, 0, 0)`    | −0.125   | b (cavity)    |

### Cavity interior (`eval > 0`)

| Query              | Expected     |
|--------------------|--------------|
| `(0, 0, 0)`        | +0.5 (= cavity radius) |
| `(0.25, 0, 0)`     | +0.25        |

### Exterior (`eval > 0`)

| Query              | Expected     |
|--------------------|--------------|
| `(1.5, 0, 0)`      | +0.5         |
| `(3, 4, 0)`        | +4 (norm = 5 Pythagorean) |

### Outer-active gradient (`grad = +p/|p|`)

- `(0.875, 0, 0)` → `(1, 0, 0)` (just inside outer wall).
- `(1.5, 0, 0)` → `(1, 0, 0)` (outside outer).
- `(-1.5, 0, 0)` → `(-1, 0, 0)` (sign symmetry).
- `(0.6, 0.8, 0)` → `(0.6, 0.8, 0)` (Pythagorean off-axis on outer
  surface — `(0.6, 0.8, 0) / 1 = (0.6, 0.8, 0)` bit-exact).

### Cavity-active gradient (`grad = -p/|p|`, **INWARD**)

- `(0.625, 0, 0)` → `(-1, 0, 0)` (just inside cavity wall, body's
  outward normal points INTO cavity → toward origin).
- `(0.25, 0, 0)` → `(-1, 0, 0)` (deep in cavity).
- `(-0.25, 0, 0)` → `(1, 0, 0)` (sign symmetry — at `-x̂` the
  inward direction is `+x̂`).

### Branch-flip locus (`|p| = 0.75`)

- 6 axis-aligned points on the equidistant sphere: `(±0.75, 0, 0)`,
  `(0, ±0.75, 0)`, `(0, 0, ±0.75)`. At each: `eval = -0.25`
  (both branches return -0.25); `grad = ±(unit axis)` (a-branch
  selected by tie-break, gradient is the outer sphere's `p/|p|`).

### Outer-surface gradient (`grad = ±(unit axis)`)

- 6 axis-aligned points at `|p| = R_OUTER = 1.0`: `φ_a = 0` and
  `-φ_b = -0.5`, so `0 ≥ -0.5` selects a-branch. Gradient is the
  unit axis (radially outward — body's outward wall normal).

### Cavity-surface gradient (`grad = ∓(unit axis)`, INWARD)

- 6 axis-aligned points at `|p| = R_CAVITY = 0.5`: `φ_a = -0.5`
  and `-φ_b = 0`, so `-0.5 < 0` selects b-branch. Gradient is the
  **negated** unit axis (radially inward — body's outward cavity-wall
  normal points INTO the cavity). This is the load-bearing visual for
  the difference operator's geometric meaning: a `Difference(A, B)`
  body has B's surface as its inner wall, with normals flipped
  relative to B's own SDF.

### Slice-grid identity (`verify_slice_consistency`)

49² = 2401 grid points in `[−1.5, +1.5]²` × `{0}` at spacing 0.0625
(endpoint-inclusive). The identity `eval(p) == max(|p| − R_OUTER,
−(|p| − R_CAVITY))` is asserted **bit-exact** at every grid point
(both sides do the same FP arithmetic). The active-branch field
captured during slice construction is re-derived from `phi_a >=
-phi_b` and asserted equal at every point.

The slice spacing 0.0625 = 2⁻⁴ is dyadic and lattice-aligned with
both radii: `R_CAVITY` lands at axis index 32 (`-1.5 + 32·0.0625 =
0.5`), `R_OUTER` at axis index 40, and the origin at axis index 24.
The lattice intersects each operand circle at 4 axis-aligned points
each, so `surface = 8` deterministically. The other bucket counts
(shell interior / cavity interior / exterior) are deterministic from
the geometry; printed by `print_summary` for the museum-plaque
summary, not asserted as anchors.

## Visuals

`out/sdf_slice.ply` (binary LE, 2401 verts + 0 faces) — vertices-only
point cloud (all at `z = 0`) with two per-vertex scalars:

- `extras["signed_distance"]` — analytic SDF; auto-detected as
  divergent (any negative → divergent).
- `extras["active_branch"]` — 0.0 = cavity-branch (b) active, 1.0 =
  outer-branch (a) active. Categorical, displays as a sharp
  threshold visualizing the branch-flip circle at `|p| = 0.75`.

Open in `cf-view`, the workspace's unified visual-review viewer:

```text
cargo run -p cf-viewer --release -- examples/sim-soft/hollow-shell-sdf/out/sdf_slice.ply
```

cf-view auto-discovers per-vertex scalars on load and selects the
alphabetical first by default — so the launch view is colour-mapped
by `active_branch` (sharp threshold; visualizes the branch-flip
circle). For the canonical donut visualization, switch to
`signed_distance` via the side-panel scalar dropdown, or pre-select
from the CLI:

```text
cargo run -p cf-viewer --release -- examples/sim-soft/hollow-shell-sdf/out/sdf_slice.ply --scalar=signed_distance
```

What you should see when colour-mapped by `signed_distance`
(divergent bwr — blue interior, white surface band, red exterior):

- **Donut cross-section** centred on the origin: a ring of negative
  values between two zero-circles (radii 0.5 and 1.0) with positive
  values inside the inner circle (the cavity) and outside the outer
  circle (the exterior).
- **Cavity is positive (red)**, NOT negative — the cavity is OUTSIDE
  the body. The signed distance from a point inside the cavity to the
  body is the radial gap to the cavity wall.
- **Most negative point in the shell is at the equidistant locus**
  `|p| = 0.75` with value `-0.25` — the body is thickest here in the
  signed-distance sense.

What you should see when colour-mapped by `active_branch` (categorical):

- **Two concentric regions**: a darker disk inside `|p| < 0.75`
  (cavity-branch active, value 0.0) and a brighter annulus + bulk for
  `|p| > 0.75` (outer-branch active, value 1.0).
- **Sharp threshold circle at `|p| = 0.75`** — the equidistant locus
  between the two operand surfaces. This circle does NOT correspond
  to a feature of the body geometry; it is purely an artefact of the
  evaluator's `max` decision. Visually overlaying the two scalars
  (mental composition) shows: the threshold circle sits in the middle
  of the negative shell band.

## Run

```text
cargo run -p example-sim-soft-hollow-shell-sdf --release
```

Output: `out/sdf_slice.ply` (2401-vertex slice + 2 per-vertex scalars).
Stdout prints input fixture summary, all 11 anchor-group names, and
the slice interior / cavity-interior / exterior / surface counts.

## Cross-references

- **Sister Tier 1 examples**: `sphere-sdf-eval` (row 1 — `Sdf` trait
  contract on the simplest primitive) and `sdf-to-tet-sphere` (row 3
  — BCC + Isosurface Stuffing tet meshing on the sphere SDF). See
  `sim/L0/soft/EXAMPLE_INVENTORY.md` Tier 1.
- **`DifferenceSdf` impl**: `sim/L0/soft/src/sdf_bridge/difference.rs`
  — sharp-CSG difference operator + `≥` tie-break invariant + 7
  internal unit tests on the same hollow-shell shape (different radii:
  `R_outer=0.10, R_cavity=0.04`).
- **Downstream consumer**: `SoftScene::layered_silicone_sphere`
  (`sim/L0/soft/src/readout/scene.rs:175`) — the IV-5 multi-material
  hollow-sphere scene meshes a `DifferenceSdf` of two `SphereSdf`s
  with internal pressure on the cavity surface.
- **Book reference**: Part 7 §00 §01 (sharp-CSG difference operator);
  the operator section of the SDF pipeline chapter.
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
  [`feedback_one_at_a_time_review`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_one_at_a_time_review.md),
  [`feedback_thorough_review_before_commit`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_thorough_review_before_commit.md).
