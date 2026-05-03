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

## What this example demonstrates

`mesh-lattice` exposes two distinct lattice paths: the **TPMS** path
(gyroid / Schwarz-P / diamond / IWP / neovius) extracted via marching
cubes — covered by §5.5 — and the **strut** path (cylindrical beams
between cell-grid nodes) covered here. Cubic is the simplest strut
topology: 3 axis-aligned beam families per cell, no diagonal struts
(octet-truss / kelvin / etc. add diagonals on top of the same cubic
node grid).

The fixture is a 25 mm × 25 mm × 25 mm cubic bounding box at 5 mm cell
size (so 5 × 5 × 5 = **125 cells**, with `(cells + 1)³ = 6³ = 216`
unique grid nodes and `3 × 5 × 6 × 6 = 540` unique edges). Strut
thickness is 1.0 mm (radius 0.5 mm). Density is 1.0 (uniform full-
density grid; `density.sqrt() == 1.0`, so every beam reports
`r1 == r2 == 0.5` exactly). `with_beam_export(true)` toggles on the
`BeamLatticeData` side-channel that the v0.9 3MF beam writer (F11)
will consume.

The example computes:

1. **Direct `generate_strut` anchors** — `STRUT_SEGMENTS = 6` per
   `mesh-lattice/src/strut.rs:14` ⇒ each strut emits exactly 14
   vertices (2 cap centers + 6 × 2 ring) and 24 triangles (6 sides
   × 2 + 12 caps); zero-length strut → `None`; `combine_struts` of
   N struts ⇒ `N × 14` vertices + `N × 24` faces (no inter-strut
   welding).
2. **`estimate_strut_volume` anchors** — cylinder formula `π · r² · L`
   and cone formula `(π/3) · L · r²` at `1e-10`.
3. **`LatticeType::Cubic` traits** — `is_strut_based() == true`,
   `is_tpms() == false`, `recommended_resolution() == 10`,
   `name() == "Cubic"`.
4. **`LatticeParams::cubic + with_*` builder chain** —
   `with_strut_thickness`, `with_density`, `with_beam_export`,
   `with_trim_to_bounds`; `validate()` returns `Ok(())`.
5. **`generate_lattice` result counts** — `cell_count == 125` (5³);
   `vertex_count == 540 × 14 == 7560`;
   `triangle_count == 540 × 24 == 12960` (all BIT-EXACT;
   `combine_struts` does not weld inter-strut nodes).
6. **`total_strut_length == Some(2700.0)` BIT-EXACT** (`540 × 5.0`;
   the running sum stays an integer multiple of 5.0, exact in f64
   below 2⁵³).
7. **`actual_density.is_finite() && ∈ [0, 1]`** — F9 heuristic
   (cubic-strut path uses `estimate_strut_volume` on the mesh, NOT
   the §5.5 signed-tet formula; empirical `~0.13`).
8. **`BeamLatticeData`** — `vertex_count() == 216` (deduplicated
   grid nodes via the `quantize` `HashMap` pattern at scale `1e6`);
   `beam_count() == 540`; `total_length() ≈ 2700.0` within `1e-9`;
   per-beam `r1 == r2 == 0.5` within `1e-12`; per-beam
   `cap1 == cap2 == BeamCap::Sphere` (default); `estimate_volume()`
   in the cylinder-sum range (≈ `675π ≈ 2120.575` mm³); per-beam
   `length(&data.vertices) == Some(5.0)` within `1e-9`.

## Strut-vs-TPMS contrast (with §5.5)

| | §5.5 `mesh-lattice-tpms-gyroid` | §5.6 `mesh-lattice-strut-cubic` |
|---|---|---|
| Path | TPMS isosurface (marching cubes) | Strut topology (cylindrical beams) |
| `is_tpms` / `is_strut_based` | `true` / `false` | `false` / `true` |
| Vertex/triangle counts | Cube-case dependent (vertex-soup) | Combinatorial (`N_struts × 14` / `× 24`) |
| Bit-exact total length | n/a (TPMS path returns `None`) | `total_strut_length == Some(2700.0)` BIT-EXACT |
| `actual_density` heuristic | `estimate_mesh_volume` (signed-tet; broken on shells per §5.5 drift-12) | `estimate_strut_volume` (mesh-bbox heuristic; not affected by drift-12) |
| Beam-data side channel | n/a (TPMS path skips `BeamLatticeData`) | populated when `with_beam_export(true)` (216 nodes + 540 beams) |
| Visual centerpiece | iconic gyroid surface | cubic strut grid (axis-aligned cylinders) |

## 3MF beam-data precursor

The `BeamLatticeData` populated when `with_beam_export(true)` is the
in-memory shape of the 3MF beam-lattice extension's `<beamlattice>` /
`<beams>` / `<beam>` blocks. Each `Beam` carries `v1` / `v2` (grid-
node indices), `r1` / `r2` (per-end radii), `length` /
`average_radius` (computed methods), and `cap1` / `cap2`
(`BeamCap::Sphere` default; `Flat` and `Butt` are the other
variants exposed). The 3MF *writer* itself is **F11**, a v0.9
candidate (see [`mesh-io/CHANGELOG.md`](../../../mesh/mesh-io/CHANGELOG.md)).
v1.0 stops at the in-memory data structure, with this example as the test that
the build path populates it correctly from a strut-based
`generate_lattice` call.

The dedup mechanism is the `quantize` `HashMap` pattern at scale `1e6`
(`mesh-lattice/src/generate.rs:111-119`): each strut endpoint is
quantized to an `[i64; 3]` key and looked up before insertion. Since
all 540 strut endpoints are integer multiples of 5 mm, the quantize
produces exact i64 values, collapsing 1080 strut-end references into
the 216 unique grid nodes.

## Why every BIT-EXACT anchor holds

- **`vertex_count == 7560` / `triangle_count == 12960`** — none of
  the 540 grid struts is degenerate (axis-aligned, 5 mm length); none
  is trimmed (max grid-node coord = 25.0, never strictly greater than
  `BBOX_MAX = 25.0`). `combine_struts` (`mesh-lattice/src/strut.rs:191`)
  appends per-strut vertex arrays without any dedup pass.
- **`total_strut_length == Some(2700.0)` BIT-EXACT** —
  `generate_cubic_lattice` (`mesh-lattice/src/generate.rs:122/156/183/209`)
  initializes `total_length = 0.0` and adds `cell_size = 5.0` per
  strut. The running sum is `[5.0, 10.0, 15.0, …, 2700.0]`; every
  partial sum is an integer ≤ 2700, exact in f64 (2700 < 2⁵³).
- **`data.total_length() ≈ 2700.0` within `1e-9`** — every beam's
  `length` is `(v2 − v1).norm() = sqrt(25.0) = 5.0` (perfect square,
  IEEE 754 sqrt is correctly-rounded). The sum of 540 fives is
  2700.0; tolerance is `1e-9` per spec but the empirical value prints
  as `2700.000000`.
- **`data.vertex_count() == 216`** — the `quantize` `HashMap` keys
  are `[(p.x · 1e6) as i64, …]` triples on grid nodes that are
  integer multiples of 5.0; no quantization collisions are possible
  at this scale.
- **`data.beam_count() == 540`** — every grid node `(ix, iy, iz)`
  emits up to 3 outgoing struts (along +X/+Y/+Z if `i_axis < cells`),
  for `5 · 6 · 6 = 180` per axis × 3 axes = 540.

## Numerical anchors

Each anchor is encoded as `assert_relative_eq!` (or `assert_eq!` for
bit-exact / counts) in `src/main.rs` under `verify_strut_free_fns`,
`verify_estimate_strut_volume`, `verify_lattice_type_traits`,
`verify_params_validate`, `verify_lattice_result_geometry`,
`verify_total_strut_length`, `verify_actual_density`, and
`verify_beam_data`. Per
[`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
a clean `cargo run --release` exit-0 == clean visual inspection (plus
the optional cubic-grid visual below).

### Direct strut anchors (`verify_strut_free_fns`)

| Query | Expected | Tolerance | Reason |
|-------|----------|-----------|--------|
| `generate_strut(p0, p1, 0.5).vertex_count()` | `14` | BIT-EXACT (`assert_eq!`) | `STRUT_SEGMENTS = 6`; 2 caps + 6 × 2 ring |
| `generate_strut(p0, p1, 0.5).face_count()` | `24` | BIT-EXACT | 6 sides × 2 + 12 cap-fan tris |
| `generate_strut_tapered(p0, p1, 0.5, 0.25)` counts | `14 / 24` | BIT-EXACT | tessellation is independent of taper |
| `generate_strut(p, p, 0.5)` | `None` | strict | `length < f64::EPSILON` returns `None` |
| `combine_struts(N=3)` counts | `42 / 72` | BIT-EXACT | linear concatenation; no welding |

### `estimate_strut_volume` (`verify_estimate_strut_volume`)

| Query | Expected | Tolerance | Reason |
|-------|----------|-----------|--------|
| `estimate_strut_volume(10, 1, 1)` | `10π ≈ 31.4159` | `1e-10` | cylinder branch (`|r1 − r2| < f64::EPSILON`) |
| `estimate_strut_volume(10, 1, 0)` | `10π/3 ≈ 10.4720` | `1e-10` | cone branch (`(π/3) · L · (r1² + r1·r2 + r2²)`) |

### `LatticeType::Cubic` traits

- `is_strut_based() == true`
- `is_tpms() == false`
- `recommended_resolution() == 10` (matches the strut-path default)
- `name() == "Cubic"`

### Generated-lattice geometry (`verify_lattice_result_geometry`)

| Anchor | Value | Tolerance |
|--------|-------|-----------|
| `cell_count` | `125` | BIT-EXACT (`5 × 5 × 5`) |
| `vertex_count` | `7560` | BIT-EXACT (`540 × 14`) |
| `triangle_count` | `12960` | BIT-EXACT (`540 × 24`) |

### `total_strut_length` (`verify_total_strut_length`)

| Anchor | Value | Tolerance |
|--------|-------|-----------|
| `result.total_strut_length` | `Some(2700.0)` | BIT-EXACT (`to_bits` equality) |

### `actual_density` (`verify_actual_density`)

`actual_density.is_finite() && actual_density ∈ [0.0, 1.0]` —
empirical `≈ 0.1335`. The cubic-strut path uses
`estimate_strut_volume(&mesh, radius)` (`generate.rs:519-549`), a
mesh-bbox-diagonal heuristic; *not* the signed-tet integration that
§5.5 drift-12 broke on un-welded gyroid shells. The cubic estimate
is approximate but stable, and well within `[0, 1]`.

### `BeamLatticeData` (`verify_beam_data`)

| Anchor | Value | Tolerance | Reason |
|--------|-------|-----------|--------|
| `vertex_count()` | `216` | BIT-EXACT | `(cells + 1)³ = 6³` after `quantize` dedup |
| `beam_count()` | `540` | BIT-EXACT | `3 × 5 × 6 × 6` cubic-lattice edges |
| `total_length()` | `2700.0` | `1e-9` | sum of 540 axis-aligned 5.0 norms |
| per-beam `r1` / `r2` | `0.5` | `1e-12` | `STRUT_THICKNESS / 2 · density.sqrt()` (density 1.0) |
| per-beam `cap1` / `cap2` | `BeamCap::Sphere` | strict | `Beam::new` default |
| `estimate_volume()` | `2120.58` mm³ | range `[2100, 2150]` | analytical `675π ≈ 2120.575` mm³ |
| per-beam `length(&vertices)` | `Some(5.0)` | `1e-9` | `sqrt(25)` is exactly 5.0 |

## Visuals

`out/cubic_lattice.ply` (binary little-endian, 7 560 verts + 12 960
tris, ~260 KB) is the visual centerpiece. Open in f3d:

```text
f3d examples/mesh/mesh-lattice-strut-cubic/out/cubic_lattice.ply
```

⏸ **Visual review optional but recommended** — even though math-pass
anchors verify correctness, opening the `.ply` shows the cubic strut
grid: 540 hexagonal-prism cylinders along the X / Y / Z axes between
216 grid nodes. The axis-aligned regularity is part of the pedagogy,
and the hexagonal cross-section (per `STRUT_SEGMENTS = 6`) is
visible at intersections.

## Run

```text
cargo run -p example-mesh-mesh-lattice-strut-cubic --release
```

Output (the museum-plaque summary, abbreviated):

```text
==== mesh-lattice-strut-cubic ====

fixture: 25 mm bbox, cell_size = 5 mm, strut_thickness = 1 mm
         density = 1, with_beam_export(true)

Free-fn anchors:
  generate_strut → 14 verts + 24 tris (BIT-EXACT)
  generate_strut_tapered → same 14 / 24 counts
  generate_strut(p, p, 0.5) = None (zero-length branch)
  combine_struts(N=3) → 42 verts + 72 tris (no inter-strut welding)
  estimate_strut_volume(10, 1, 1) ≈ 10π   = 31.4159 (cylinder, ≤ 1e-10)
  estimate_strut_volume(10, 1, 0) ≈ 10π/3 = 10.4720 (cone,     ≤ 1e-10)

LatticeType::Cubic traits:
  is_strut_based = true, is_tpms = false, recommended_resolution = 10

Generated lattice:
  cell_count       = 125 (BIT-EXACT; 5 × 5 × 5)
  vertex_count     = 7560 (BIT-EXACT; 540 struts × 14)
  triangle_count   = 12960 (BIT-EXACT; 540 struts × 24)
  total_strut_len  = 2700.0 (BIT-EXACT Some(2700.0); 540 × 5.0)
  actual_density   ≈ 0.1335 (F9 heuristic; finite + in [0, 1])

BeamLatticeData (3MF beam-extension precursor; F11 v0.9 consumer):
  vertex_count() = 216 (BIT-EXACT; 6³ unique grid nodes after dedup)
  beam_count()   = 540 (BIT-EXACT; 3 × 5 × 6 × 6)
  total_length() = 2700.000000 mm (≤ 1e-9; sum of 540 axis-aligned 5.0)
  per-beam r1 == r2 == 0.5 mm (≤ 1e-12; density.sqrt() = 1.0)
  per-beam cap1 == cap2 == BeamCap::Sphere (default)
  estimate_volume() = 2120.58 mm³ (in [2100, 2150]; ≈ 675π)
  per-beam length(&vertices) = Some(5.0) for all 540 beams (≤ 1e-9)

artifact: out/cubic_lattice.ply (7560v, 12960f, binary little-endian)

OK — 4 strut + 2 estimate_strut_volume + 4 LatticeType + ...
```

## Cross-references

- **Sister examples** rounding out the v1.0 mesh-arc:
  `mesh-measure-bounding-box` (§5.1) at `719a85d3`,
  `mesh-measure-cross-section` (§5.2) at `021a9712`,
  `mesh-measure-distance-to-mesh` (§5.3) at `4650058a`,
  `mesh-sdf-distance-query` (§5.4) at `ab7335fd`,
  `mesh-lattice-tpms-gyroid` (§5.5) at `947aa8d8`. The next
  example — `mesh-lattice-density-gradient` (§5.7, variable density
  via `DensityMap` on octet-truss) — locks in `§6.2 #17 / #18`.
- **Mesh book**: `docs/studies/mesh_architecture/src/80-examples.md`
  — Part 8 inventory (depth-pass updates land at `§6.2 #31` of the
  arc).
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md)
  — clean exit-0 gates the visuals pass (delivered first-run on
  this example, all anchors green);
  [`feedback_examples_drive_gap_fixes`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_examples_drive_gap_fixes.md)
  — F11 3MF beam writer is the v0.9 candidate this example
  pre-stages.
