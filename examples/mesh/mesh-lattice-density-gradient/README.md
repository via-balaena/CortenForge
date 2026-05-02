# mesh-lattice-density-gradient

**Variable-density lattice via `DensityMap` on the octet-truss preset
— direct `DensityMap` evaluation anchors (`Uniform` / `Gradient` /
`Radial` / `Function` + the `evaluate_with_distance` demo on
`SurfaceDistance`), then `generate_lattice` for a 30 mm³ bbox at
cell size 7.5 mm, strut thickness 0.6 mm, with a `Gradient` density
map climbing from 0.1 at z=0 to 0.5 at z=30 and `with_beam_export(true)`
flipping on the per-beam radius readout that demonstrates density
modulation.** The density-modulated counterpart to §5.6
`mesh-lattice-strut-cubic`: same strut path, but octet-truss
topology (richer geometry — 20 struts per cell instead of 3 axis
families) and a non-uniform density map, so per-beam `r1` varies
with cell position rather than being a global constant.

> Skeleton commit (`§6.2 #17` per the v1.0 examples-coverage arc spec
> at `mesh/MESH_V1_EXAMPLES_SCOPE.md`). The `DensityMap` 4-variant
> evaluation anchors, the `LatticeParams::octet_truss` + builder
> chain + `density_at` accessor anchors, the `generate_lattice` +
> `cell_count == 64` + `beam_data == Some(_)` anchors, and the
> load-bearing per-beam `r1` density-modulation anchor (top-half
> mean ≈ 1.41× bottom-half mean per the
> `r1 = strut_thickness/2 × density.sqrt()` formula) all land in
> `§6.2 #18`. This README is a placeholder; museum-plaque content
> (numerical anchors, visuals, 3MF beam-export callout, cross-
> references) fills in alongside the impl commit.

## What this example will demonstrate

`mesh-lattice` exposes density modulation via the `DensityMap` enum:
six variants (`Uniform`, `Gradient`, `Radial`, `SurfaceDistance`,
`StressField`, `Function`) feeding `LatticeParams::with_density_map`,
which the strut-path generators consume to scale per-beam radius
as `r1 = strut_thickness/2 × density.sqrt()`. §5.6 covered the
uniform-density strut path (cubic, density 1.0); this example covers
the variable-density strut path (octet-truss, `Gradient` map).

The fixture is a 30 mm × 30 mm × 30 mm bounding box at 7.5 mm cell
size (so 4 × 4 × 4 = **64 cells**, octet-truss topology). Strut
thickness is 0.6 mm (baseline radius 0.3 mm). The `Gradient` density
map climbs linearly along the z axis: `from_density = 0.1` at z=0,
`to_density = 0.5` at z=30, so per-beam `r1` follows
`r1 = strut_thickness/2 × density.sqrt()` — beams in the bottom half
report a smaller `r1` than beams in the top half (precise per-half
mean anchor lands at `§6.2 #18`). `with_beam_export(true)` toggles on
the `BeamLatticeData` side-channel that exposes per-beam `r1` for the
load-bearing density-modulation anchor.

The example will compute (TBD — land in `§6.2 #18`):

1. **Direct `DensityMap` evaluation anchors** — 4 variants
   (`Uniform`, `Gradient`, `Radial`, `Function`) at known sample
   points within `1e-12`; plus the `evaluate_with_distance` demo on
   `SurfaceDistance` (the special method that takes a distance
   scalar instead of a `Point3`). `StressField` deferred to a richer
   §5.X example.
2. **`Gradient` clamping beyond endpoints** —
   `evaluate((0,0,-50)) == 0.1` and `evaluate((0,0,150)) == 0.5`;
   **`Function` clamping to [0, 1]** —
   `from_function(|_| 1.5).evaluate(origin) == 1.0`.
3. **`DensityMap::from_function` + `from_stress_field` constructors**
   — both build a `Function` / `StressField` variant with the closure
   captured for evaluation.
4. **`LatticeParams::octet_truss(7.5)` preset + builder chain** —
   `with_strut_thickness(0.6)`, `with_density_map(_)`,
   `with_beam_export(true)`; `params.density_at((0,0,0)) == 0.1` and
   `params.density_at((0,0,30)) == 0.5` (the accessor delegates to
   the wrapped density map when present).
5. **`generate_lattice` result counts** — `cell_count == 64` (4³);
   `result.beam_data == Some(_)` (preserve_beam_data ON for this
   example).
6. **Per-beam `r1` density anchor (HE-4, load-bearing)** — every
   octet-truss cell emits 20 struts × 14 verts/strut = 280 verts
   regardless of density (geometric tessellation is FIXED; density
   does NOT change vertex count). Density modulation manifests as
   `r1 = strut_thickness/2 × density.sqrt()` per beam. Anchor:
   filter `data.beams` by `vertices[v1].z` — bottom-half beams
   (z ≤ 15) have mean `r1 ≈ 0.3 × √0.20 ≈ 0.134`; top-half beams
   (z > 15) have mean `r1 ≈ 0.3 × √0.40 ≈ 0.190`. Top/bottom mean
   ratio ≈ `√(0.4/0.2) = 1.41` ± 5%. Spot-check anchor: at least
   one beam at `z < 7.5` has `r1 < 0.16`; at least one at `z > 22.5`
   has `r1 > 0.18`.
7. **Direct `DensityMap::Function` demo** — build
   `from_function(|p| 0.5 + 0.1 × sin(p.x / 10.0))`, query at known
   points, assert.

## Density-modulated vs uniform contrast (with §5.6)

| | §5.6 `mesh-lattice-strut-cubic` | §5.7 `mesh-lattice-density-gradient` |
|---|---|---|
| Topology | Cubic (3 axis families per cell) | Octet truss (20 struts per cell) |
| Density map | None (`density = 1.0` global) | `Gradient` (0.1 at z=0 → 0.5 at z=30) |
| Per-beam `r1` | `0.5` everywhere (BIT-EXACT; density.sqrt() = 1.0) | Varies with z per `r1 = 0.3 × density.sqrt()` |
| Cell count | `125` (5³, 25 mm bbox) | `64` (4³, 30 mm bbox) |
| Load-bearing anchor | `total_strut_length == Some(2700.0)` BIT-EXACT | top-half mean `r1` ≈ 1.41× bottom-half mean |
| Visual centerpiece | Cubic strut grid (axis-aligned cylinders) | Octet truss with thin bottom → thick top struts |

## `DensityMap` six variants

The enum exposes (TBD — land in `§6.2 #18`):

- **`Uniform(f64)`** — constant density everywhere; `evaluate` ignores
  the point.
- **`Gradient { from, from_density, to, to_density }`** — linear
  interpolation between two points (not an axis enum); clamps beyond.
- **`Radial { center, inner_radius, outer_radius, inner_density, outer_density }`**
  — radial interpolation from a center point.
- **`SurfaceDistance { ... }`** — `evaluate(p)` returns midpoint
  fallback (per `density.rs:203-212`); the *real* method is
  `evaluate_with_distance(distance: f64) -> Option<f64>` which takes
  a precomputed distance scalar.
- **`StressField`** — closure-captured field; deferred from this
  example.
- **`Function`** — closure-captured `f: Fn(Point3) -> f64`; result
  clamped to [0, 1].

This example covers 4 variants directly + 1 via the special method
(`SurfaceDistance::evaluate_with_distance`); `StressField` is
deferred entirely (would need a meaningful stress field function;
defer to a richer §5.X example or v0.9 candidate).

## 3MF beam-data precursor

The `BeamLatticeData` populated when `with_beam_export(true)` is the
in-memory shape of the 3MF beam-lattice extension's `<beamlattice>` /
`<beams>` / `<beam>` blocks. Each `Beam` carries `v1` / `v2` (grid-
node indices), `r1` / `r2` (per-end radii — *the* density-modulation
target for this example), and `cap1` / `cap2`. The 3MF *writer*
itself is **F11** in `mesh/MESH_V1_EXAMPLES_SCOPE.md` §10 — a v0.9
candidate that §5.6 `mesh-lattice-strut-cubic` already pre-stages
for the uniform case and that this example pre-stages for the
variable-density case.

## Numerical anchors (TBD — land in `§6.2 #18`)

- 4 `DensityMap` variant evaluation anchors at known points within
  `1e-12` (`Uniform` / `Gradient` endpoints + midpoint / `Radial`
  at center / `Function` clamp to [0,1]).
- `Gradient` clamp anchors beyond endpoint at `(0,0,-50)` and
  `(0,0,150)`.
- `SurfaceDistance::evaluate_with_distance(0.0) == surface_density`
  and `evaluate_with_distance(transition_depth) == core_density`.
- `from_function` + `from_stress_field` constructor anchors.
- `LatticeParams::octet_truss` + `with_density_map` +
  `with_strut_thickness` + `with_beam_export` builder chain;
  `validate() == Ok(())`.
- `params.density_at((0,0,0)) == 0.1`;
  `params.density_at((0,0,30)) == 0.5` within `1e-12`.
- After `generate_lattice`: `result.cell_count == 64` (BIT-EXACT;
  4³); `result.beam_data == Some(_)`.
- **Per-beam `r1` density anchor** (HE-4, load-bearing): top-half
  mean `r1` / bottom-half mean `r1` ≈ `√(0.4/0.2) ≈ 1.41` ± 5%;
  spot-check `r1 < 0.16` for some beam at z < 7.5 and `r1 > 0.18`
  for some beam at z > 22.5.
- Direct `DensityMap::Function` demo at known points.

## Run (TBD — land in `§6.2 #18`)

```text
cargo run -p example-mesh-mesh-lattice-density-gradient --release
```

Output: `out/density_gradient_lattice.ply` (the triangulated
density-modulated octet-truss lattice). ⏸ visual review **optional
but recommended** — the gradient is visually striking (thin bottom
struts → thick top struts).

## Cross-references

- **Spec**: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.7 (this example) +
  §10 v0.9 backlog (F11 3MF beam writer — consumes the
  `BeamLatticeData` populated here, same v0.9 candidate that §5.6
  pre-stages for the uniform case).
- **Sister examples**: §5.6 `mesh-lattice-strut-cubic` at
  `732f06a2` (uniform-density strut counterpart);
  §5.5 `mesh-lattice-tpms-gyroid` at `947aa8d8` (TPMS contrast).
- **Mesh book**: `docs/studies/mesh_architecture/src/80-examples.md`
  — Part 8 inventory (depth-pass updates land at `§6.2 #31` of the
  arc).
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md)
  — clean exit-0 gates the visuals pass;
  [`feedback_examples_drive_gap_fixes`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_examples_drive_gap_fixes.md)
  — F11 3MF beam writer is the v0.9 candidate this example
  pre-stages.
