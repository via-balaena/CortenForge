# mesh-lattice-density-gradient

**Variable-density lattice via `DensityMap` on the octet-truss preset
— direct `DensityMap` evaluation anchors (`Uniform` / `Gradient` /
`Radial` / `Function` + the `evaluate_with_distance` demo on
`SurfaceDistance`), then `generate_lattice` for a 30 mm³ bbox at cell
size 7.5 mm, strut thickness 0.6 mm, with a `Gradient` density map
climbing from 0.1 at z=0 to 0.5 at z=30 and `with_beam_export(true)`
flipping on the per-beam radius readout that demonstrates density
modulation.** The density-modulated counterpart to §5.6
`mesh-lattice-strut-cubic`: same strut path, but octet-truss topology
(richer geometry — 20 struts per cell instead of 3 axis families) and
a non-uniform density map, so per-beam `r1` varies with cell position
rather than being a global constant.

## What this example demonstrates

`mesh-lattice` exposes density modulation via the `DensityMap` enum:
six variants (`Uniform`, `Gradient`, `Radial`, `SurfaceDistance`,
`StressField`, `Function`) feeding `LatticeParams::with_density_map`,
which the strut-path generators consume to scale per-beam radius as
`r1 = strut_thickness/2 × density.sqrt()`. §5.6 covered the
uniform-density strut path (cubic, density 1.0); this example covers
the variable-density strut path (octet-truss, `Gradient` map).

The fixture is a 30 mm × 30 mm × 30 mm bounding box at 7.5 mm cell
size (so 4 × 4 × 4 = **64 cells**, octet-truss topology). Strut
thickness is 0.6 mm (baseline radius 0.3 mm). The `Gradient` density
map climbs linearly along the z axis: `from_density = 0.1` at z=0,
`to_density = 0.5` at z=30. Density is sampled at each **cell center**
(`mesh-lattice/src/generate.rs:290-293`), so the four cell-z strata at
z = 3.75 / 11.25 / 18.75 / 26.25 produce the four discrete per-cell
densities `0.15 / 0.25 / 0.35 / 0.45` and the four discrete per-beam
radii `0.3 × √density ≈ 0.1162 / 0.1500 / 0.1775 / 0.2012`.
`with_beam_export(true)` toggles on the `BeamLatticeData` side-channel
that exposes per-beam `r1` for the load-bearing density-modulation
anchor — a public-surface read that requires the matching octet-truss
beam-data export populated by the immediately preceding commit
(`fix(mesh-lattice): populate BeamLatticeData on octet-truss path`).

The example computes:

1. **Direct `DensityMap` evaluation anchors** — 4 variants
   (`Uniform`, `Gradient`, `Radial`, `Function`) at known sample
   points within `1e-12`, plus `Gradient` clamping beyond either
   endpoint and `Function` clamping the closure output to `[0, 1]`.
2. **`DensityMap::evaluate_with_distance` demo** on `SurfaceDistance`
   — the special method that takes a precomputed distance scalar
   instead of a `Point3` (returns `Option<f64>`; only
   `SurfaceDistance` returns `Some(_)`, others return `None`). Plain
   `evaluate(p)` on `SurfaceDistance` returns the midpoint fallback
   `(surface + core) / 2` per `density.rs:203-212`.
3. **`DensityMap::from_function` and `from_stress_field`
   constructors** — both build the corresponding `Function` /
   `StressField` variant with the closure captured for `evaluate`.
   `StressField::evaluate` exercised at min / max / midpoint.
4. **`DensityMap::from_function` sinusoidal demo** — exact-anchor
   evaluations of `f(p) = 0.5 + 0.1 × sin(p.x / 10.0)` at zeros and
   the first peak (output stays in `[0.4, 0.6]` so no clamping
   fires).
5. **`LatticeParams::octet_truss(7.5)` preset + builder chain** —
   `with_strut_thickness(0.6)`, `with_density_map(_)`,
   `with_beam_export(true)`; `validate() == Ok(())`;
   `params.density_at((0, 0, 0)) == 0.1` and
   `params.density_at((0, 0, 30)) == 0.5` (the accessor delegates to
   the wrapped density map when present).
6. **`generate_lattice` result counts** — `cell_count == 64` (4³)
   BIT-EXACT; `result.beam_data == Some(_)` (post C15a parity fix);
   `actual_density` finite + in `[0, 1]` (F9 heuristic — octet path
   uses the same `estimate_strut_volume` route as cubic, no §5.5
   drift-12 closed-orientable-manifold pathology).
7. **Per-beam `r1` density anchor (HE-4, load-bearing)** — every
   octet-truss cell emits 20 struts × 14 verts/strut = 280 verts
   regardless of density (geometric tessellation is FIXED; density
   does NOT change vertex count). Density modulation manifests as
   `r1 = strut_thickness/2 × density.sqrt()` per beam. Anchors:
   - **Per-beam exact**: every beam's `r1` matches one of the four
     stratum radii `{0.1162, 0.1500, 0.1775, 0.2012}` within `1e-10`
     (correctly-rounded `density.sqrt()`; no other density values can
     occur).
   - **Regional mean ratio**: filtering by `vertices[v1].z` partitions
     beams unevenly across the z=15 cut (12 of 20 per-cell beams have
     `v1` at the cell's low corner, 8 at the high corner, per
     `generate.rs:325-382`). Empirical: bottom-half (v1.z ≤ 15) has
     n=832 beams with mean r1 ≈ **0.1433 mm**; top-half (v1.z > 15)
     has n=448 beams with mean r1 ≈ **0.1945 mm**. Top/bottom mean
     ratio ≈ **1.357** — well inside the spec's ±5% band around 1.41
     (1.357 / 1.41 ≈ 0.962, ~3.8% below; the offset is structural,
     not noise — see "v1-filter asymmetry" below).
   - **Spot-checks**: `min r1 at v1.z < 7.5 ≈ 0.1162 < 0.16`
     (iz=0 stratum); `max r1 at v1.z > 22.5 ≈ 0.2012 > 0.18`
     (iz=3 stratum, exact within `1e-9`).
8. **`BeamLatticeData` BIT-EXACT counts** —
   `beam_count == 1280` (64 cells × 20 beams; gradient density stays
   `≥ 0.1 > 0.05` so no cell drops at the early-out filter);
   `vertex_count == 189` (5³ = 125 deduplicated grid corners + 64
   unique cell centers; `quantize`/`vertex_map` dedup at scale
   `1e6`, `generate.rs:276-284`).

## v1-filter asymmetry

Per-cell, the 20 beams break down as: 8 corner-to-center +
4 bottom-face edges + 4 top-face edges + 4 vertical edges. By
construction (`generate.rs:325-382`) `v1` is the corner argument for
corner-to-center beams and the first-listed corner index for edge
beams. That puts `v1` at the cell's *low* corner for 12 of the 20
beams (4 bottom-face edges, 4 vertical edges, 4 corner-to-center from
the bottom corners) and at the *high* corner for 8 of the 20 (4
top-face edges, 4 corner-to-center from the top corners). With 16
cells per z-stratum and 4 strata across z=0..30, the v1.z ≤ 15
filter sweeps in (a) all 320 beams of stratum iz=0, (b) all 320 of
iz=1, and (c) the 12-per-cell low-corner beams of iz=2 (192 beams) —
totalling 832 beams. The v1.z > 15 filter sweeps in (a) the
8-per-cell high-corner beams of iz=2 (128 beams) and (b) all 320 of
iz=3, totalling 448 beams. The `r1` means are weighted by beam count
within stratum so the top/bottom ratio diverges slightly from the
naïve `√(0.4 / 0.2) = 1.414`. Per-stratum `r1` is exact at the four
stratum values (locked under the per-beam exact anchor), so the
density-modulation observation is quantitatively grounded; the
ratio anchor's tolerance accounts for the v1-filter's structural
asymmetry.

## Density-modulated vs uniform contrast (with §5.6)

| | §5.6 `mesh-lattice-strut-cubic` | §5.7 `mesh-lattice-density-gradient` |
|---|---|---|
| Topology | Cubic (3 axis families per cell) | Octet truss (20 struts per cell) |
| Density map | None (`density = 1.0` global) | `Gradient` (0.1 at z=0 → 0.5 at z=30) |
| Per-beam `r1` | `0.5` everywhere (BIT-EXACT; `density.sqrt() = 1.0`) | One of `{0.1162, 0.1500, 0.1775, 0.2012}` per cell stratum |
| Cell count | `125` (5³, 25 mm bbox) | `64` (4³, 30 mm bbox) |
| Beam count | `540` (3 × 5 × 6 × 6 cubic edges) | `1280` (64 cells × 20) |
| BeamLatticeData vertex count | `216` (6³ deduplicated grid nodes) | `189` (5³ deduplicated corners + 64 cell centers) |
| Load-bearing anchor | `total_strut_length == Some(2700.0)` BIT-EXACT | top/bottom mean `r1` ratio ≈ 1.357 (spec ±5% of 1.41) |
| Visual centerpiece | Cubic strut grid (axis-aligned cylinders) | Octet truss with thin bottom → thick top struts |

## `DensityMap` six variants

The enum exposes (4 demonstrated + 1 via the special method; one
deferred):

- **`Uniform(f64)`** — constant density everywhere; `evaluate`
  ignores the point.
- **`Gradient { from, from_density, to, to_density }`** — line
  interpolation between two `Point3` endpoints (NOT an axis enum —
  `density.rs:34-44`); clamps the parameter to `[0, 1]` beyond either
  endpoint.
- **`Radial { center, inner_radius, outer_radius, inner_density, outer_density }`**
  — radial interpolation from a center point.
- **`SurfaceDistance { surface_density, core_density, transition_depth }`**
  — `evaluate(p)` returns the midpoint fallback per
  `density.rs:203-212`; the *real* method is
  `evaluate_with_distance(distance: f64) -> Option<f64>` which takes
  a precomputed distance scalar.
- **`StressField`** — closure-captured field; constructor
  (`from_stress_field`) demonstrated; `StressField::evaluate` is
  also exercised in `verify_density_map_constructors`.
- **`Function`** — closure-captured `f: Fn(Point3) -> f64`; result
  clamped to `[0, 1]` post-closure (`density.rs:225`).

## 3MF beam-data precursor

The `BeamLatticeData` populated when `with_beam_export(true)` is the
in-memory shape of the 3MF beam-lattice extension's `<beamlattice>` /
`<beams>` / `<beam>` blocks. Each `Beam` carries `v1` / `v2`
(grid-node indices), `r1` / `r2` (per-end radii — *the* density-
modulation target for this example), and `cap1` / `cap2`. The 3MF
*writer* itself is **F11**, a v0.9 candidate (see
[`mesh-io/CHANGELOG.md`](../../../mesh/mesh-io/CHANGELOG.md)) that
`mesh-lattice-strut-cubic` already pre-stages for the uniform case
and that this example pre-stages for the variable-density case.

## Numerical anchors

- **`DensityMap::evaluate`**:
  - `Uniform(0.4).evaluate(any) == 0.4` within `1e-12`.
  - `Gradient(...)` at endpoints `(0, 0, 0) == 0.1`, `(0, 0, 30) == 0.5`
    within `1e-12`; midpoint `(0, 0, 15) == 0.3` within `1e-12`.
  - `Gradient` clamping: `(0, 0, -50) == 0.1`, `(0, 0, 150) == 0.5`
    within `1e-12`.
  - `Radial { inner_radius: 0, ... }.evaluate(center) == inner_density`
    within `1e-12`.
  - `from_function(|_| 1.5).evaluate(origin) == 1.0`,
    `from_function(|_| -0.5).evaluate(origin) == 0.0` within `1e-12`.
- **`DensityMap::evaluate_with_distance`**:
  - `SurfaceDistance::evaluate_with_distance(0.0) == surface_density`,
    `evaluate_with_distance(transition_depth) == core_density` within
    `1e-12`.
  - `SurfaceDistance::evaluate(p) == (surface + core) / 2` within
    `1e-12` (midpoint fallback).
  - Other variants: `evaluate_with_distance(_).is_none()`.
- **`DensityMap::from_stress_field`** — `evaluate` at zero-stress
  point == `min_density`, at unit-stress == `max_density`, at
  midpoint == midpoint within `1e-12`.
- **`DensityMap::Function` sinusoidal demo** —
  `f(0) == 0.5`, `f(10π) == 0.5`, `f(5π) == 0.6` within `1e-12`.
- **`LatticeParams::octet_truss(7.5).with_density_map(_)`**:
  `lattice_type == OctetTruss`, `cell_size == 7.5` within `1e-12`,
  `strut_thickness == 0.6` within `1e-12`, `density_map.is_some()`,
  `preserve_beam_data == true`, `validate() == Ok(())`.
- **`params.density_at`**: `(0, 0, 0) == 0.1`, `(0, 0, 30) == 0.5`
  within `1e-12` (delegates to the Gradient map).
- **`generate_lattice` result**:
  - `cell_count == 64` BIT-EXACT.
  - `beam_data == Some(_)` (post C15a parity fix).
  - `actual_density.is_finite() && in [0, 1]`.
- **`BeamLatticeData`**:
  - `beam_count() == 1280` BIT-EXACT.
  - `vertex_count() == 189` BIT-EXACT.
- **Per-beam `r1` density anchor**:
  - Every beam's `r1` matches one of the four cell-stratum radii
    `0.3 × √0.15`, `0.3 × √0.25`, `0.3 × √0.35`, `0.3 × √0.45` within
    `1e-10`.
  - Bottom-half (v1.z ≤ 15) mean r1 ≈ 0.1433, n=832.
  - Top-half (v1.z > 15) mean r1 ≈ 0.1945, n=448.
  - Top/bottom ratio ∈ (1.30, 1.45) — empirical 1.357.
  - Spot-check: `min r1 at v1.z < 7.5 < 0.16` (iz=0 stratum);
    `max r1 at v1.z > 22.5 > 0.18` (iz=3 stratum, within `1e-9` of
    `0.3 × √0.45 ≈ 0.2012`).

## Visuals

- **`out/density_gradient_lattice.ply`** — 17 920-vertex /
  30 720-triangle binary little-endian PLY of the triangulated
  density-modulated octet-truss lattice. Each octet cell contributes
  20 struts × (14 verts + 24 tris); `combine_struts` does NOT weld
  inter-strut nodes (matches §5.6's no-weld pattern).

```text
f3d examples/mesh/mesh-lattice-density-gradient/out/density_gradient_lattice.ply
```

⏸ Visual review **optional but recommended** — the gradient is
visually striking: the bottom half of the cube has thin struts, the
top half has noticeably thicker struts, with the four cell-z strata
sharply visible as four discrete radius bands. The octet topology
(corners-to-center diagonals + face/vertical edges) creates a denser,
more visually rich lattice than §5.6's pure cubic grid.

## Run

```text
cargo run -p example-mesh-mesh-lattice-density-gradient --release
```

## Cross-references

- **Sister examples**: §5.6 `mesh-lattice-strut-cubic` (uniform-density
  strut counterpart); §5.5 `mesh-lattice-tpms-gyroid` (TPMS
  contrast).
- **Mesh book**: `docs/studies/mesh_architecture/src/80-examples.md`
  — Part 8 inventory (depth-pass updates land at `§6.2 #31` of the
  arc).
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md)
  — clean exit-0 gates the visuals pass;
  [`feedback_examples_drive_gap_fixes`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_examples_drive_gap_fixes.md)
  — the C15a octet-truss beam-data parity fix preceding this commit
  is the in-arc gap-fix this example surfaced.
