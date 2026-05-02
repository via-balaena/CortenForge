# mesh-lattice-strut-cubic

**Cubic strut lattice generation — direct `generate_strut` /
`generate_strut_tapered` / `combine_struts` / `estimate_strut_volume`
anchors, then `generate_lattice` for a 25mm³ bbox at cell size 5mm,
strut thickness 1.0mm, uniform density 1.0, with `with_beam_export(true)`
flipping on the 3MF `BeamLatticeData` precursor (deduplicated grid
nodes + per-edge beam list).** The strut counterpart to §5.5
`mesh-lattice-tpms-gyroid`: cylindrical beams between integer-spaced
grid nodes — NOT a TPMS isosurface — so the per-strut mesh counts are
combinatorial constants, the total strut length is bit-exact, and the
beam-data `vertex_count` / `beam_count` / `total_length` follow
straight from the cubic-cell topology.

> Skeleton commit (`§6.2 #15` per the v1.0 examples-coverage arc spec
> at `mesh/MESH_V1_EXAMPLES_SCOPE.md`). The `generate_strut` family
> anchors, the `LatticeParams::cubic` + `with_*` builder chain, the
> `generate_lattice` + `cell_count` + `total_strut_length` anchors,
> and the `BeamLatticeData` (vertex + beam + length + per-beam
> radius / cap) verification land in `§6.2 #16`. This README is a
> placeholder; museum-plaque content (numerical anchors, visuals,
> 3MF beam-export callout, cross-references) fills in alongside the
> impl commit.

## What this example will demonstrate

`mesh-lattice` exposes two distinct lattice paths: the **TPMS**
path (gyroid / Schwarz-P / diamond / IWP / neovius) extracted via
marching cubes — covered by §5.5 — and the **strut** path
(cylindrical beams between cell-grid nodes) covered here. Cubic is
the simplest strut topology: 3 axis-aligned beam families per cell,
no diagonal struts (octet-truss / kelvin / etc. add diagonals on
top of the same cubic node grid).

The fixture is a 25mm × 25mm × 25mm cubic bounding box at 5mm cell
size (so 5 × 5 × 5 = 125 cells, with `(cells + 1)³ = 6³ = 216`
unique grid nodes and `3 × 5 × 6 × 6 = 540` unique edges). Strut
thickness is 1.0mm (radius 0.5mm). Density is 1.0 (uniform full-
density grid; `density.sqrt() == 1.0`, so every beam reports
`r1 == r2 == 0.5` exactly). `with_beam_export(true)` toggles on the
`BeamLatticeData` side-channel that the v0.9 3MF beam writer (F11)
will consume.

The example will compute (TBD — land in `§6.2 #16`):

1. **Direct `generate_strut` anchors** — `STRUT_SEGMENTS = 6` per
   `mesh-lattice/src/strut.rs:14` ⇒ each strut emits exactly
   14 vertices (2 cap centers + 6 × 2 ring) and 24 triangles
   (6 sides × 2 + 12 caps); zero-length strut → `None`;
   `combine_struts` of N struts ⇒ `N × 14` vertices + `N × 24`
   faces (no inter-strut welding).
2. **`estimate_strut_volume` anchors** — cylinder formula
   `π × length × r²` and cone formula `π/3 × length × r²` at
   `1e-10`.
3. **`LatticeType::Cubic` traits** — `is_strut_based() == true`,
   `is_tpms() == false`, `recommended_resolution() == 10`,
   `name() == "Cubic"`.
4. **`LatticeParams::cubic + with_*` builder chain** —
   `with_strut_thickness`, `with_density`, `with_beam_export`,
   `with_trim_to_bounds`; `validate()` returns `Ok(())`.
5. **`generate_lattice` result counts** — `cell_count == 125`
   (5³); `vertex_count == 540 × 14 == 7560`; `triangle_count ==
   540 × 24 == 12960` (BIT-EXACT; `combine_struts` does not weld
   inter-strut nodes).
6. **`total_strut_length == Some(2700.0)`** BIT-EXACT (`540 × 5.0`;
   integer multiples of 5.0 are exact in f64 below 2⁵³).
7. **`BeamLatticeData`** — `vertex_count() == 216` (deduplicated
   grid nodes via the `quantize` HashMap pattern at scale `1e6`);
   `beam_count() == 540`; `total_length()` exactly `2700.0`;
   per-beam `r1 == r2 == 0.5` within `1e-12`; per-beam
   `cap1 == cap2 == BeamCap::Sphere` (default); `estimate_volume()`
   in the cylinder-sum range (~2120 mm³); per-beam
   `length(&data.vertices) == Some(5.0)` within `1e-9`.

## Strut-vs-TPMS contrast (with §5.5)

| | §5.5 `mesh-lattice-tpms-gyroid` | §5.6 `mesh-lattice-strut-cubic` |
|---|---|---|
| Path | TPMS isosurface (marching cubes) | Strut topology (cylindrical beams) |
| `is_tpms` / `is_strut_based` | `true` / `false` | `false` / `true` |
| Vertex/triangle counts | Cube-case dependent (vertex-soup) | Combinatorial (`N_struts × 14` / `× 24`) |
| Bit-exact total length | n/a | `total_strut_length == 540 × 5.0` |
| Beam-data side channel | n/a | `BeamLatticeData` (216 nodes + 540 beams) |
| Visual centerpiece | iconic gyroid surface | cubic strut grid (axis-aligned cylinders) |

## 3MF beam-data precursor

The `BeamLatticeData` populated when `with_beam_export(true)` is the
in-memory shape of the 3MF beam-lattice extension's `<beamlattice>` /
`<beams>` / `<beam>` blocks. Each `Beam` carries `v1` / `v2`
(grid-node indices), `r1` / `r2` (per-end radii), `length` /
`average_radius` (cached scalars), and `cap1` / `cap2`
(`BeamCap::Sphere` default; `Hemisphere` and `Butt` are the other
3MF-spec options). The 3MF *writer* itself is **F11** in
`mesh/MESH_V1_EXAMPLES_SCOPE.md` §10 — v0.9. v1.0 stops at the
in-memory data structure, with this example as the test that the
build path populates it correctly from a strut-based
`generate_lattice` call.

## Numerical anchors (TBD — land in `§6.2 #16`)

- 4 `generate_strut` direct anchors (vertex/face count, zero-length
  `None`, tapered same counts, `combine_struts` linearity).
- 2 `estimate_strut_volume` anchors (cylinder + cone) at `1e-10`.
- `LatticeType::Cubic` 4-trait coverage.
- `LatticeParams` builder chain + `validate() == Ok(())`.
- `cell_count == 125` (BIT-EXACT).
- `vertex_count == 7560` + `triangle_count == 12960` (BIT-EXACT;
  `540 × 14` and `540 × 24`).
- `total_strut_length == Some(2700.0)` BIT-EXACT.
- `BeamLatticeData::vertex_count() == 216` (BIT-EXACT).
- `BeamLatticeData::beam_count() == 540` (BIT-EXACT).
- `BeamLatticeData::total_length() == 2700.0` within `1e-9`.
- All beams `r1 == r2 == 0.5` within `1e-12`; all caps `Sphere`.
- `estimate_volume()` in the cylinder-sum range (~2120 mm³).
- Per-beam `length(&vertices) == Some(5.0)` within `1e-9`.

## Run (TBD — land in `§6.2 #16`)

```text
cargo run -p example-mesh-mesh-lattice-strut-cubic --release
```

Output: `out/cubic_lattice.ply` (the triangulated strut mesh —
540 cylindrical beams). ⏸ visual review **optional but
recommended** — the cubic strut grid is visually clear and the
axis-aligned regularity is part of the pedagogy.

## Cross-references

- **Spec**: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.6 (this example) +
  §10 v0.9 backlog (F11 3MF beam writer — consumes the
  `BeamLatticeData` populated here).
- **Sister examples**: §5.5 `mesh-lattice-tpms-gyroid` at
  `947aa8d8` (TPMS contrast). The next example —
  `mesh-lattice-density-gradient` (§5.7, variable density via
  `DensityMap` on octet-truss) — lands in `§6.2 #17 / #18`.
- **Mesh book**: `docs/studies/mesh_architecture/src/80-examples.md`
  — Part 8 inventory (depth-pass updates land at `§6.2 #31` of
  the arc).
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md)
  — clean exit-0 gates the visuals pass;
  [`feedback_examples_drive_gap_fixes`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_examples_drive_gap_fixes.md)
  — F11 3MF beam writer is the v0.9 candidate this example
  pre-stages.
