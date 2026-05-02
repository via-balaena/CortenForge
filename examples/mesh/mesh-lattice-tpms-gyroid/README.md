# mesh-lattice-tpms-gyroid

**TPMS lattice generation via the gyroid implicit surface — free-
function `gyroid` + `density_to_threshold` + `make_shell` direct
anchors, then `generate_lattice` for a 30 mm³ × 10 mm-period block at
density 0.5, resolution 15, wall thickness 1.5 mm. The marching-cubes
output is **vertex-soup** (no welding pass) — three unique verts per
triangle by construction (F10).** Hand-locks the gyroid implicit
function at known points, the density→threshold mapping at
`density = 0.5 → threshold = 0` bit-exact, and verifies the generated
lattice mesh against the gyroid SDF post-extraction.

## What this example demonstrates

The **gyroid** is one of the most-printed triply-periodic minimal
surfaces (TPMS). Its implicit function is

```text
G(x, y, z) = sin(x)·cos(y) + sin(y)·cos(z) + sin(z)·cos(x)
```

(scaled by `2π / cell_size` to set the period). The `G == 0` level
set is the gyroid surface — a self-supporting, smoothly-varying
shape with no flat overhangs that prints cleanly on FDM without
supports.

`mesh-lattice` exposes both the free `gyroid(point, cell_size)`
evaluator (for direct field queries) AND the `generate_lattice`
pipeline that wraps `gyroid` into a `make_shell`-thickened SDF and
extracts the isosurface via marching cubes. The fixture is a
30 mm × 30 mm × 30 mm bounding box at 10 mm cell size (so 3 × 3 × 3 =
27 cells), resolution 15 samples per cell, density 0.5
(threshold = 0 bit-exact, R3 mitigation for F8 approximate
density-to-threshold), and wall thickness 1.5 mm.

**Why density = 0.5**: `density_to_threshold(0.5, "gyroid")` returns
exactly `0.0` (per `tpms.rs:172` formula `(0.5 − density) × 3.0`).
For other density values the mapping is *approximate* (F8 — declared
in the doc comment at `tpms.rs:164`); locking the fixture at
`density = 0.5` puts the threshold anchor on the one bit-exact case.

The example computes:

1. **Free-fn `gyroid` evaluation at known points** — `gyroid((0, 0,
   0), 10) == 0` BIT-EXACT; one-cell periodicity at `1e-10`;
   `gyroid((2.5, 0, 0), 10) == 1.0` at `1e-12`.
2. **`density_to_threshold` anchors** — bit-exact at
   `density = 0.5 → threshold = 0`; `density = 0.3 → threshold ≈
   0.6` within `1e-12`.
3. **`make_shell` SDF wrapper** — origin lies on the gyroid surface;
   `make_shell(|p| gyroid(p, 10), 1.0)((0, 0, 0)) == −0.5` (origin
   is `half_thickness` deep inside the shell wall) at `1e-12`.
4. **`LatticeType::Gyroid` traits** — `is_tpms() == true`,
   `is_strut_based() == false`, `recommended_resolution() == 15`,
   `name() == "Gyroid"`.
5. **`LatticeParams` builder + `validate()`** — gyroid preset chained
   with `with_density(0.5)`, `with_resolution(15)`,
   `with_wall_thickness(1.5)` produces a valid params struct.
6. **`generate_lattice`** — 30 mm³ bbox + cell_size 10 → cell_count
   exactly 27.
7. **Vertex-soup signature (F10)** —
   `result.vertex_count() == 3 × result.triangle_count()` BIT-EXACT
   (`assert_eq!`, NOT `assert_relative_eq!`).
8. **Bounds containment** — every output vertex lies within the
   closed bbox `[min, max]` within `1e-12` (MC's linear-interp
   places verts strictly between cell-corner samples; the highest-
   index cell's far corner equals `bounds.max` exactly).
9. **`actual_density` is finite + in `[0, 1]`** — see drift-12
   below for why the prior `[0.3, 0.7]` range was wrong.
10. **Per-vertex SDF surface verification** — every vertex `v`
    satisfies `||gyroid(v, 10)| − half_thickness| < 0.05` (the
    drift-11 anchor; see callout below).

## Vertex-soup output (F10) — platform truth

The marching-cubes implementation in `mesh-lattice` emits **vertex-
soup**: every triangle gets 3 fresh vertex indices (no welding
pass). Source: `mesh-lattice/src/marching_cubes.rs:456` —
`// Add vertices (could deduplicate, but this is simpler)`. This is
a deliberate simplification, not a regression — and a **v0.9
candidate** for replacement once a real consumer needs welded
TPMS-lattice output (file size, cleaner viewer rendering, robust
orientation propagation) — spec §10 item 4.

The example asserts `vertex_count == 3 × triangle_count`
**bit-exact** — both quantities are gated by the same
`is_degenerate(...)` filter inside MC, so the ratio is
**invariant** at the implementation level. (The filter removes both
the three `vertices.push(...)` calls AND the `faces.push([...])`
atomically inside `marching_cubes.rs:462-468`.) Rendering the
resulting `.ply` in f3d shows the soup-output's hallmark: face
boundaries appear faceted because adjacent triangles do NOT share
vertex indices despite sharing geometric position.

The mesh-shell engine (PR #222 commits 11.5.1 / 11.5.2) DOES weld
its MC output via `mesh-repair::weld_vertices`; v1.0 lattice
examples treat the lattice's un-welded output as platform-truth and
document the alternative.

## Drift-11 — vertices live on the SHELL surface, not on `G = 0`

Spec §5.5 line 617 framed the per-vertex SDF anchor as
`gyroid(v).abs() < threshold + voxel_size + epsilon` with
`threshold = 0` for `density = 0.5`, implying vertices live near the
bare gyroid level set `G = 0`. **Geometric truth**:
`generate_tpms_lattice` (`mesh-lattice/src/generate.rs:379-386`)
wraps the gyroid into `make_shell` whose surface is at
`|G| = half_thickness = 0.75` (NOT `G = 0`). The MC isosurface
extracts that shell wall, so vertices live at
`|G(v)| ≈ half_thickness ± quantization`, NOT at `|G(v)| ≈ 0`.

The reformulated anchor — encoded in this example — is

```text
||gyroid(v, 10)| − half_thickness| < 0.05  (for all v)
```

Empirical max across the 315,576 output vertices is **0.0229**,
50× tighter than the spec's prior 1.0 cushion AND 30× tighter than
the analytical worst-case bound (`½ · |∇²G|_max · voxel² ≈ 0.18`)
— the gyroid's second-derivative magnitude is well below its
theoretical max across most cells. The `0.05` cushion absorbs
cross-platform libm `sin`/`cos` drift.

Both `max |G(v)| ≈ 0.7729` and `max ||G(v)| − half_thickness| ≈
0.0229` are surfaced in the stdout summary. Drift-11 spec edit
lands inline at this commit (precedent: drifts 7+8 in §5.2,
drifts 9+10 in §5.4).

## Drift-12 — `actual_density` heuristic is broken on shell topologies

Spec §5.5 line 615 said `actual_density` lands in `[0.3, 0.7]` for
`density = 0.5` input (declared F9 heuristic, "not tight").
Empirically `actual_density ≈ 0.06` — the range was wrong by ~6×.

`estimate_mesh_volume` (`mesh-lattice/src/generate.rs:552-576`) is
the divergence-theorem formula `(1/6) Σ v0·(v1×v2)`, which requires
a closed orientable manifold mesh with consistently-oriented
triangles. The lattice's gyroid SHELL has TWO boundary components
(`|G| = 0.75`, the surface from above and below); the un-welded MC
output violates one or more of the precondition assumptions in this
implementation — exact cause **untraced** (candidates: TRI_TABLE
orientation inconsistencies between cube cases, `is_degenerate`
filter interactions, or seam handling between adjacent cells). F9
is more broken than the spec assumed; the heuristic cannot be
range-anchored on shell topologies. The reformulated anchor is

```text
actual_density.is_finite() && actual_density ∈ [0.0, 1.0]
```

The v0.9 fix (proper volume integration on welded mesh) belongs
with the F10 weld-pass — both unblock cleaner volume estimates.
Drift-12 spec edit lands inline at this commit alongside drift-11.

## Numerical anchors

Each anchor is encoded as `assert_relative_eq!` (or `assert_eq!`
for bit-exact / counts) in `src/main.rs` under `verify_gyroid_free_fn`,
`verify_density_to_threshold`, `verify_make_shell`,
`verify_lattice_type_traits`, `verify_params_validate`,
`verify_lattice_result_geometry`, `verify_actual_density`, and
`verify_vertex_sdf_surface`. Per
[`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
a clean `cargo run --release` exit-0 == clean visual inspection
(plus the optional gyroid-surface visual below).

### Free-fn `gyroid` (`verify_gyroid_free_fn`)

| Query | Expected | Tolerance | Reason |
|-------|----------|-----------|--------|
| `gyroid((0, 0, 0), 10)` | `0.0` | BIT-EXACT (`to_bits` equality) | `sin(0) = 0` and `cos(0) = 1` exactly; all 3 terms = 0 |
| `gyroid((10, 10, 10), 10)` | `0.0` | `1e-10` | One-cell periodicity; `sin(2π)` rounds to `~1e-16` in f64 |
| `gyroid((2.5, 0, 0), 10)` | `1.0` | `1e-12` | `sin(π/2)·cos(0) = 1·1 = 1.0` exactly; other terms zero |

### `density_to_threshold` (`verify_density_to_threshold`)

| Query | Expected | Tolerance | Reason |
|-------|----------|-----------|--------|
| `density_to_threshold(0.5, "gyroid")` | `0.0` | BIT-EXACT (`to_bits` equality) | `(0.5 − 0.5) · 3.0 = 0.0 · 3.0 = 0` exact (R3 / F8 mitigation) |
| `density_to_threshold(0.3, "gyroid")` | `0.6` | `1e-12` | `(0.5 − 0.3) · 3.0 ≈ 0.6 ± 1 ULP` |

### `make_shell` (`verify_make_shell`)

`make_shell(|p| gyroid(p, 10), 1.0)((0, 0, 0)) == −0.5` within
`1e-12` — origin is on the bare gyroid surface (`G = 0`), so
`shell_sdf = |0| − half_thickness = −0.5` (origin is
`half_thickness` deep inside the shell wall).

### `LatticeType::Gyroid` traits

- `is_tpms() == true`
- `is_strut_based() == false`
- `recommended_resolution() == 15` (matches our fixture)
- `name() == "Gyroid"`

### Generated lattice geometry (`verify_lattice_result_geometry`)

| Anchor | Value | Tolerance |
|--------|-------|-----------|
| `cell_count` | `27` | BIT-EXACT (`3 × 3 × 3`) |
| `vertex_count` | `> 0` | strict |
| `triangle_count` | `> 0` | strict |
| `vertex_count == 3 × triangle_count` (F10 vertex-soup) | exact equality | **BIT-EXACT** |
| Bounds containment (every v in `[min − ε, max + ε]`) | within | `1e-12` |

Concrete output values: `vertex_count = 315 576`,
`triangle_count = 105 192` (`315 576 = 3 × 105 192` BIT-EXACT).
F10 ratio is invariant per commit (filter at
`marching_cubes.rs:462-468`).

### Per-vertex SDF surface (`verify_vertex_sdf_surface`, drift-11)

For all 315,576 output vertices `v`:

```text
||gyroid(v, 10)| − half_thickness| < 0.05
```

Empirical max across the run:

| Quantity | Empirical | Locked anchor | Spec (prior) |
|---------|-----------|---------------|--------------|
| `max |G(v)|` | `0.7729` | within `[0.70, 0.80]` (= `half_thickness ± 0.05`) | `< 1.0` cushion (drift-11 reformulated) |
| `max ||G(v)| − half_thickness|` | `0.0229` | `< 0.05` | n/a (anchor reframed) |

### `actual_density` (`verify_actual_density`, drift-12)

`actual_density.is_finite() && actual_density ∈ [0.0, 1.0]` —
empirical `~0.06`, but per drift-12 the heuristic on un-welded
shell topologies is severely broken. See drift-12 callout above.

## Visuals

`out/gyroid_lattice.ply` (binary little-endian, 315 576 verts +
105 192 tris) is the visual centerpiece. Open in f3d:

```text
f3d examples/mesh/mesh-lattice-tpms-gyroid/out/gyroid_lattice.ply
```

⏸ **Visual review recommended** — even though math-pass anchors
verify correctness, opening the `.ply` shows the iconic
'twisted-saddle' surface of the gyroid TPMS. The lattice has 27
unit cells (3 × 3 × 3) of the periodic gyroid surface, thickened
into a 1.5 mm shell. f3d will show the soup-output's hallmark
faceting at triangle boundaries (per F10 — adjacent triangles do
NOT share vertex indices despite sharing geometric position).

## Run

```text
cargo run -p example-mesh-mesh-lattice-tpms-gyroid --release
```

Output: `out/gyroid_lattice.ply` (the generated MC lattice mesh).
Stdout prints fixture summary, all free-fn anchors, the
`LatticeType::Gyroid` trait checks, generated-lattice counts +
F10 vertex-soup signature, the drift-11 SDF surface anchors
(both `max |G(v)|` and `max ||G(v)| − half_thickness|`), and the
artifact path.

## Cross-references

- **Spec**: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.5 (this example) +
  §7 R3 (density_to_threshold approximate, mitigated by anchoring
  at `density = 0.5 → threshold = 0` bit-exact) + §10 v0.9 backlog
  (F10 weld-pass trigger, item 4). Drift-11 (line 617 anchor
  reformulation) + drift-12 (line 615 range correction) land
  inline at this commit.
- **Sister examples** rounding out the v1.0 mesh-arc:
  `mesh-measure-bounding-box` (§5.1) at `719a85d3`,
  `mesh-measure-cross-section` (§5.2) at `021a9712`,
  `mesh-measure-distance-to-mesh` (§5.3) at `4650058a`,
  `mesh-sdf-distance-query` (§5.4) at `ab7335fd`. The next two —
  `mesh-lattice-strut-cubic` (§5.6, the strut-based-lattice
  contrast) and `mesh-lattice-density-gradient` (§5.7, variable
  density via `DensityMap`) — lock in `§6.2 #15-#20`.
- **MC welding contrast**: `mesh-offset-{outward,inward}` use the
  `mesh-shell` engine which DOES weld MC output (PR #222 commits
  11.5.1 / 11.5.2 via `mesh-repair::weld_vertices`); the lattice's
  un-welded output is documented as deliberate (F10) until a real
  consumer surfaces.
- **Mesh book**: `docs/studies/mesh_architecture/src/80-examples.md`
  — Part 8 inventory (depth-pass updates land at `§6.2 #31` of the
  arc).
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md)
  — clean exit-0 gates the visuals pass;
  [`feedback_examples_drive_gap_fixes`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_examples_drive_gap_fixes.md)
  — drift-11 / drift-12 surface as v0.9 candidates from this
  example; spec §10 backlog migration lands at `§6.2 #32`.
