# mesh-lattice-tpms-gyroid

**TPMS lattice generation via the gyroid implicit surface — free-
function `gyroid` + `density_to_threshold` + `make_shell` direct
anchors, then `generate_lattice` for a 30mm³ × 10mm-period block at
density 0.5, resolution 15, wall thickness 1.5mm. The marching-
cubes output is **vertex-soup** (no welding pass) — three unique
verts per triangle by construction (F10).** Hand-locks the gyroid
implicit function at known points, the density→threshold mapping
at `density = 0.5 → threshold = 0.0` bit-exact, and verifies the
generated lattice mesh against the same gyroid SDF
post-extraction.

> Skeleton commit (`§6.2 #12` per the v1.0 examples-coverage arc
> spec at `mesh/MESH_V1_EXAMPLES_SCOPE.md`). Free-fn `gyroid` /
> `density_to_threshold` / `make_shell` anchors land in `§6.2 #13`;
> `generate_lattice` + cell_count + vertex-soup signature + bounds
> containment + per-vertex SDF surface verification land in `§6.2
> #14`. This README is a placeholder; museum-plaque content
> (numerical anchors, visuals, F8/F10 callouts, cross-references)
> fills in alongside the impl commit.

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
30mm × 30mm × 30mm bounding box at 10mm cell size (so 3 × 3 × 3 =
27 cells), resolution 15 samples per cell, density 0.5
(threshold = 0 bit-exact, R3 mitigation for F8 approximate
density-to-threshold), and wall thickness 1.5mm.

**Why density = 0.5**: `density_to_threshold(0.5, "gyroid")`
returns exactly `0.0` (per `tpms.rs:172` formula `(0.5 − density)
× 3.0`). For other density values the mapping is *approximate*
(F8 — declared in the doc comment at `tpms.rs:164`); locking the
fixture at `density = 0.5` puts the threshold anchor on the one
bit-exact case.

The example computes (TBD — land in `§6.2 #13` + `§6.2 #14`):

1. **Free-fn `gyroid` evaluation at known points**
   — `gyroid((0, 0, 0), 10) == 0` exactly; periodicity over one
   cell; `(2.5, 0, 0)`-axis sample equals `1.0` exactly.
2. **`density_to_threshold` anchors** — bit-exact at `density =
   0.5 → threshold = 0`; `density = 0.3 → threshold = 0.6` within
   `1e-12`.
3. **`make_shell` SDF wrapper** — origin lies on the gyroid
   surface; `make_shell(|p| gyroid(p, 10), 1)((0,0,0)) == −0.5`
   (half_thickness inside the wall).
4. **`generate_lattice`** — 30mm³ bbox + cell_size 10 → cell_count
   exactly 27.
5. **Vertex-soup signature (F10)** —
   `result.vertex_count() == 3 × result.triangle_count()`
   **bit-exact**. The MC source filters degenerate triangles by
   gating both `vertices.push(...)` calls and the `faces.push([...])`
   inside the same `if !is_degenerate(...)` block
   (`marching_cubes.rs:462-468`), so the ratio is invariant per
   commit.
6. **Bounds containment** — every output vertex lies within the
   closed bbox `[min, max]`.
7. **Per-vertex SDF surface verification** — every vertex `v` of
   the lattice mesh satisfies `|gyroid(v, 10).abs() − 0.75|` within
   one voxel + cushion (the lattice approximates the
   thickened-gyroid surface at `|gyroid| = half_thickness = 0.75`,
   so the implicit MC quantization stays within ~1 voxel = `10/15
   ≈ 0.667` of the analytical level set).

## Vertex-soup output (F10) — platform truth

The marching-cubes implementation in `mesh-lattice` emits **vertex-
soup**: every triangle gets 3 fresh vertex indices (no welding
pass). Source: `mesh-lattice/src/marching_cubes.rs:456` —
`// Add vertices (could deduplicate, but this is simpler)`.
This is a deliberate simplification, not a regression — and a
**v0.9 candidate** for replacement once a real consumer needs
welded TPMS-lattice output (file size, cleaner viewer rendering,
robust orientation propagation).

The example asserts `vertex_count == 3 × triangle_count` bit-
exact — both quantities are gated by the same
`is_degenerate(...)` filter inside MC, so the ratio is
**invariant** at the implementation level. (The filter removes
both `vertices.push(...)` calls AND the `faces.push([...])`
atomically.) Rendering the resulting `.ply` in f3d shows the
soup-output's hallmark: face boundaries appear faceted because
adjacent triangles do NOT share vertex indices despite sharing
geometric position.

The mesh-shell engine (PR #222 commits 11.5.1 / 11.5.2) DOES weld
its MC output via `mesh-repair::weld_vertices`; v1.0 lattice
examples treat the lattice's un-welded output as platform-truth
and document the alternative.

## Numerical anchors (TBD — land in `§6.2 #13` + `§6.2 #14`)

- 3 free-fn `gyroid` direct anchors at `1e-10` (origin = 0;
  periodicity over `cell_size = 10`; `(2.5, 0, 0)` axis sample =
  `1.0`).
- 2 `density_to_threshold` anchors (density = 0.5 → threshold = 0
  bit-exact; density = 0.3 → 0.6 within `1e-12`).
- 1 `make_shell` anchor (origin on gyroid surface → shell =
  `−half_thickness`).
- `LatticeType::Gyroid::is_tpms() == true` and
  `recommended_resolution() == 15`.
- `LatticeParams::gyroid + with_*` builder chain + `validate()` ok.
- `result.cell_count == 27` (3 × 3 × 3 cells in 30mm³ at 10mm).
- `vertex_count == 3 × triangle_count` bit-exact (F10 vertex-soup
  signature).
- Bounds containment within `1e-12`.
- Per-vertex `|gyroid(v, 10)|` distance to half-thickness shell
  surface within one-voxel + cushion tolerance.

## Run (TBD — land in `§6.2 #13` + `§6.2 #14`)

```text
cargo run -p example-mesh-mesh-lattice-tpms-gyroid --release
```

Output: `out/gyroid_lattice.ply` (the generated MC lattice mesh —
the visual centerpiece; iconic 'twisted-saddle' surface of the
gyroid TPMS). ⏸ visual review **recommended** for this example —
even though math-pass anchors verify correctness, opening the
`.ply` in f3d shows the gyroid's distinctive shape.

## Cross-references

- **Spec**: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.5 (this example) +
  §7 R3 (density_to_threshold approximate, mitigated by anchoring
  at `density = 0.5 → threshold = 0` bit-exact) + §10 v0.9 backlog
  (F10 weld-pass trigger).
- **Sister examples**:
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
  — F8 / F10 surfaced as v0.9 candidates from this example; spec
  §10 backlog migration lands at `§6.2 #32`.
