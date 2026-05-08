# mesh-scan-as-solid

**`mesh_sdf::SignedDistanceField` satisfies [`cf_design::Sdf`] —
shipped at PR3 F2.** A scanned-mesh-derived SDF dispatches through
the same trait surface as parametric [`cf_design::Solid`] bodies.
12-triangle axis-aligned cube fixture (`R = 1.0`), built
programmatically and round-tripped through binary STL on disk to
exercise the literal scan-import workflow without a checked-in
asset. Closed-form L∞-ball anchors at face / edge / vertex / interior
probes; bulk-grid PLY emits two per-vertex scalars
(`signed_distance`, `inside_raycast`) for cf-view.

## What this example demonstrates

The layered-silicone-device pipeline routes `mesh-sdf →
cf-design → sim-soft → BCC stuffing → FEM`. The first link
(scanned mesh as a first-class design primitive) becomes possible
when [`SignedDistanceField`] satisfies the [`cf_design::Sdf`]
contract. The example does exactly that: builds a 12-triangle cube,
saves it to STL, loads it back through `mesh-io`, builds a
`SignedDistanceField` over the loaded mesh, and dispatches every
numerical anchor through `&dyn cf_design::Sdf` rather than the
inherent `SignedDistanceField` API. No newtype wrapper, no per-mesh
glue — the trait is enough.

The cube is the smallest fixture with closed-form L∞-ball SDF
plus enough faces (12) to exercise mesh-sdf's
`closest_point_on_triangle` Ericson region resolution at
face / edge / vertex regions. The "scan" framing is the workflow,
not the geometry — programmatic-cube avoids checked-in scan assets
per the layered-silicone-device sanitization directive.

## Numerical anchors

Each anchor is encoded as `assert_relative_eq!` in `src/main.rs`
under `verify_cube_geometry`, `verify_face_region_exterior`,
`verify_edge_region_exterior`, `verify_vertex_region_exterior`,
`verify_interior`, `verify_grad_finite_and_outward_on_face_band`,
`verify_stl_round_trip`, and `verify_grid_consistency`. Per
[`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
a clean `cargo run --release` exit-0 IS the correctness signal.

Two tolerance regimes:

- **`epsilon = 0.0`** (`EXACT_TOL` — bit-exact) where coords AND
  closed-form distances are dyadic. The integer cube vertices
  `(±1, ±1, ±1)` plus dyadic probe offsets (`±0.5`, `±0.25`)
  produce f64-bit-exact distances.
- **`epsilon = 1e-12`** (`BAND_TOL`) where distances involve
  `√(rational)` (e.g. `√0.5` for edge-region, `√0.75` for
  vertex-region). mesh-sdf's `(point − closest).norm()` introduces
  sub-ULP rounding; `1e-12` is comfortable headroom on the cube.

### Cube geometry contract

| Anchor                      | Tolerance | Expected                      |
|-----------------------------|-----------|-------------------------------|
| 8 vertex coords             | bit-exact | `(±R, ±R, ±R)` enumeration    |
| 12 outward unit normals     | bit-exact | `±x̂`, `±ŷ`, `±ẑ` per face pair |

The face winding and per-triangle cross-product unit normals are
hand-verified in `verify_cube_geometry` and pinned bit-exact —
integer vertex coords make each cross product produce a single
non-zero component of magnitude `4 · R² = 4`, normalizing to a
unit axis vector exactly.

### Face-region exterior (`eval = +0.5` bit-exact)

Six axis-aligned probes at distance 0.5 outside one cube face.
Closed-form SDF: `√(0.5²) + 0 = 0.5`.

| Probe                | SDF |
|----------------------|------|
| `(±1.5, 0, 0)`       | +0.5 |
| `(0, ±1.5, 0)`       | +0.5 |
| `(0, 0, ±1.5)`       | +0.5 |

### Edge-region exterior (`eval = √0.5 ≈ 0.707` within 1e-12)

Three edge-direction probes outside two cube faces simultaneously.
Closed-form: `√(0.5² + 0.5²) = √0.5`.

| Probe              | SDF |
|--------------------|------|
| `(1.5, 1.5, 0)`    | √0.5 |
| `(1.5, 0, 1.5)`    | √0.5 |
| `(0, 1.5, 1.5)`    | √0.5 |

### Vertex-region exterior (`eval = √0.75 ≈ 0.866` within 1e-12)

Eight corner probes at the eight cube-vertex offsets.
Closed-form: `√(0.5² + 0.5² + 0.5²) = √0.75`.

| Probe              | SDF |
|--------------------|------|
| `(±1.5, ±1.5, ±1.5)` (8 sign combinations) | √0.75 |

### Interior (`eval < 0` bit-exact)

Five interior probes covering deep / asymmetric / near-surface
regions. Closed-form for strict interior:
`max(\|p_x\| − R, \|p_y\| − R, \|p_z\| − R)` (negative of the
closest-face inset).

| Probe                | SDF |
|----------------------|------|
| `(0, 0, 0)`          | −1.0 |
| `(0.5, 0, 0)`        | −0.5 |
| `(0, 0.5, 0)`        | −0.5 |
| `(0, 0, 0.5)`        | −0.5 |
| `(0.5, 0.25, −0.75)` | −0.25 (max d_i at \|z\| = 0.75) |
| `(0.75, 0, 0)`       | −0.25 (near +X face)             |

### Gradient face-band (per F2 docstring)

mesh-sdf's `<SignedDistanceField as cf_design::Sdf>::grad` is a
central finite-difference approximation with `eps = 1e-6`
(`design/cf-design/src/sdf.rs:90-110`). On the cube's face contact
band, `‖grad‖ ≈ 1` and `grad · n̂_face > 0.9` (gradient aligned
with outward face normal). Asserted in
`verify_grad_finite_and_outward_on_face_band` at 7 face-band
probes (3 above-face, 1 below-face per axis sign-mirrored).

### STL round-trip

`save_stl(&cube, ...)` → `load_mesh(...)` → new
`SignedDistanceField` produces an SDF that agrees with the
in-memory original at every face-region + interior named probe at
**bit-exact tolerance**. STL stores binary f32 vertex coords, but
integer `±R = ±1.0` round-trip losslessly through f32. Face count
preserved (12); STL's lossy face-indexing inflates the vertex
count from 8 → 36 but mesh-sdf consumes faces directly without
relying on dedup.

### Bulk-grid consistency (17³ = 4913 in `[−2, 2]³` at spacing 0.25)

Two scalars over the full grid, three load-bearing assertions, and
empirical pins on raycast counts:

1. **Closed-form-vs-trait identity** at every grid point —
   `<SignedDistanceField as cf_design::Sdf>::eval(p)` equals
   `analytical_cube_sdf(p)` within `1e-12`. This is the
   trait-dispatch contract over a representative bulk sample.
2. **Heuristic strict-interior coverage** — every grid point with
   `|coord| ≤ 0.75` has `eval < 0`. The F2 face-normal sign
   heuristic is reliable on cube interiors.
3. **Raycast off-diagonal strict-interior coverage** — every
   strict-interior probe with `y ≠ z` (i.e. NOT on the `+X` face's
   `[1, 2, 6]`/`[1, 6, 5]` shared diagonal) reports
   `inside_raycast = true`. The 49 probes ON the diagonal hit
   HE-1 ray-edge-coincidence degeneracy in mesh-sdf's
   Möller-Trumbore filter and may report `inside = false`; we make
   no assertion about those.
4. **F2-caveat-absent identity** — `heuristic_inside ==
   STRICT_INTERIOR_COUNT = 343`. No grid point OUTSIDE strict
   interior reports `eval < 0`; the F2 far-field heuristic
   sign-flip does not surface on this cube fixture.
5. **Empirical raycast pins** — `raycast_inside == 576` and
   `divergence == 331`. These are bit-pinned to detect drift in
   mesh-sdf's parallel- / edge-ray Möller-Trumbore policy.

The heuristic ↔ raycast divergence (331) is dominated by HE-1
(raycast's failure at face-diagonal-incident probes) and boundary
tie-break behavior, NOT by the F2 caveat. For a fixture exhibiting
the F2 sign-flip, see `examples/mesh/mesh-sdf-distance-query`
(octahedron, 6 vertex-region false-positives at the bbox boundary).

## Visuals

`out/cube_scan.stl` (binary, 12 facets, ~700 bytes) — the cube
fixture as an STL "scan" artifact, the input side of the disk
transit demo.

`out/sdf_grid.ply` (binary LE, 4913 verts + 0 faces, ~80 KB) —
point cloud with two per-vertex scalars:

- `extras["signed_distance"]` — analytical SDF; auto-detected as
  divergent (any negative → divergent).
- `extras["inside_raycast"]` — `0 / 1` categorical from
  `SignedDistanceField::is_inside`; auto-detected as sequential.

Open in `cf-view`, the workspace's unified visual-review viewer:

```text
cargo run -p cf-viewer --release -- examples/sim-soft/mesh-scan-as-solid/out/sdf_grid.ply
```

cf-view auto-discovers per-vertex scalars and selects the
alphabetical first by default — so the launch view is colour-mapped
by `inside_raycast` (sequential viridis; binary mask of the cube
interior, with HE-1 holes along the `y = z` diagonal of the +X
face). Switch to `signed_distance` (divergent bwr — blue interior,
white surface band, red exterior) via the side-panel scalar
dropdown; the colormap re-detects per scalar.

For scripted reproducibility, pre-select `signed_distance` from
the CLI:

```text
cargo run -p cf-viewer --release -- examples/sim-soft/mesh-scan-as-solid/out/sdf_grid.ply --scalar=signed_distance
```

What you should see when colour-mapped by `signed_distance`:

- **Cubic radial gradient** — negative interior shrinks to a
  central darkest-blue cube of strict-interior probes, surface
  band at zero, positive exterior expanding to bbox corners
  (`(±2, ±2, ±2)`) at value `√0.75 + 1.0 ≈ +1.866` for the
  vertex-region offsets, and `(±2, 0, 0)` etc. at `+1.0` for
  axis-aligned far-field.
- **Sharp face-band step** — the L∞-ball SDF has piecewise-smooth
  gradient (six face regions, twelve edge wedges, eight vertex
  caps); the colormap shows visible kinks where the closest-face
  identity flips.

What you should see when colour-mapped by `inside_raycast`:

- **Bright cube interior** with a discernible HE-1 thinning along
  the `y = z` diagonal of the +X face (probes on that line report
  `inside = false` from raycast despite being strict-interior).
- **Dark exterior** beyond the cube boundary with sparse
  bright artifacts at boundary points where Möller-Trumbore's
  parallel-ray epsilon flickers.

## Run

```text
cargo run -p example-sim-soft-mesh-scan-as-solid --release
```

Output: `out/cube_scan.stl` + `out/sdf_grid.ply`. Stdout prints the
fixture summary, all 6 closed-form anchor groups, the STL
round-trip OK, and the bulk-grid stats with HE-1 / F2-caveat
provenance.

## Cross-references

- **F1+F2 trait surface**: [`cf_design::Sdf`] at
  `design/cf-design/src/sdf.rs` — the trait + the
  `mesh_sdf::SignedDistanceField` impl with the sign-heuristic
  caveat.
- **Sister example** for a fixture that DOES trigger the F2
  caveat: `examples/mesh/mesh-sdf-distance-query` (octahedron, 6
  vertex-region false-positives at the bbox boundary).
- **PLY-attribute pattern**: mirrored from
  `examples/mesh/ply-with-custom-attributes` and
  `examples/sim-soft/sphere-sdf-eval`.
- **Inventory entry**:
  `sim/L0/soft/EXAMPLE_INVENTORY.md` row 15 (Tier 5 bridges).
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
  [`feedback_thorough_review_before_commit`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_thorough_review_before_commit.md),
  [`feedback_release_mode_heavy_tests`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md).

[`cf_design::Sdf`]: ../../../design/cf-design/src/sdf.rs
[`cf_design::Solid`]: ../../../design/cf-design/src/solid.rs
[`SignedDistanceField`]: ../../../mesh/mesh-sdf/src/sdf.rs
