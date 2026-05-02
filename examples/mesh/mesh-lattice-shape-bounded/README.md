# mesh-lattice-shape-bounded

**Boundary-conforming TPMS lattice clipped to an analytical sphere
SDF via `with_shape_sdf` — direct `is_outside_shape` predicate
anchors (interior / exterior + the default `false` when no SDF is
set), then `generate_lattice` for a 30 mm³ bbox centered at origin
(`min = (-15, -15, -15)`, `max = (15, 15, 15)`) at cell size 10 mm
and resolution 15, with a sphere SDF of radius 12 mm at origin
(`Arc::new(|p| p.coords.norm() - 12.0)`) clipping the gyroid
output.** The shape-bounded counterpart to §5.5
`mesh-lattice-tpms-gyroid`: same TPMS path, but the lattice is
trimmed by a mathematical shape (analytical SDF) rather than filling
the whole axis-aligned bounding box. Complementary to §5.9
`mesh-lattice-mesh-bounded-infill`, which exercises the
mesh-bounded composite path via `generate_infill`.

> Skeleton commit (`§6.2 #19` per the v1.0 examples-coverage arc spec
> at `mesh/MESH_V1_EXAMPLES_SCOPE.md`). The `is_outside_shape`
> predicate anchors (4 spatial points + the no-SDF default), the
> with-vs-without trimmed-vertex-count anchor, and the per-vertex
> distance-to-origin bound (every output vertex lies within
> `sphere_radius + voxel_size + cushion ≈ 13.67` of origin) all
> land in `§6.2 #20`. This README is a placeholder; museum-plaque
> content (numerical anchors, visuals, comparison-output callout,
> cross-references) fills in alongside the impl commit.

## What this example will demonstrate

`mesh-lattice` exposes boundary-conforming generation via the
`with_shape_sdf` builder: an arbitrary analytical SDF
(`Arc<dyn Fn(Point3<f64>) -> f64 + Send + Sync>` per
`mesh-lattice/src/params.rs:406`) is composed into the TPMS path's
`shell_sdf` as `tpms_value.max(sdf(p))` (SDF intersection, per
`mesh-lattice/src/generate.rs:441`), so only lattice geometry where
both the gyroid wall is present **and** the shape SDF is non-positive
gets emitted. The `is_outside_shape` accessor
(`params.rs:415-417`) returns `sdf(point) > 0.0` when an SDF is set,
and `false` for any point when no SDF is set — the convention is
"outside means strictly positive; negative = inside; zero = on
surface." §5.5 covered the bbox-filling TPMS path (no shape SDF);
this example covers the shape-clipped TPMS path.

The fixture is a 30 mm × 30 mm × 30 mm bounding box centered at
origin (`min = (-15, -15, -15)`, `max = (15, 15, 15)`) at 10 mm
cell size and resolution 15, mirroring the in-tree
`test_gyroid_lattice_with_shape_sdf` precedent in
`mesh-lattice/src/generate.rs`. The shape SDF is a sphere of radius
12 mm centered at origin (`Arc::new(|p| p.coords.norm() - 12.0)`),
so points strictly closer than 12 mm to origin are "inside" and
points strictly farther are "outside." With cell size 10 mm and
resolution 15, the marching-cubes voxel size is exactly
`10 / 15 = 2/3 ≈ 0.667 mm` (the bbox is an integer multiple of the
cell, so `total_resolution = ceil((30 / 10) × 15) = 45` and voxel
size = `30 / 45 = 2/3` exactly). Every output vertex satisfies
`(v - origin).norm() < 12 + voxel_size + cushion ≈ 13.67` — the
cushion accounts for marching-cubes interpolating across cells
straddling the sphere boundary.

The example will compute (TBD — land in `§6.2 #20`):

1. **Direct `is_outside_shape` predicate anchors** — 4 spatial
   evaluations against the sphere-radius-12 SDF at known points
   (origin + a clearly-interior point at radius 11 + a clearly-
   exterior point at radius 13 + a far-exterior point at radius 20)
   plus the default `is_outside_shape == false` for any point when
   no SDF is set on `LatticeParams::gyroid(10.0)` (per
   `params.rs:415-417`). All exact (boolean equality).
2. **`LatticeParams::gyroid(10.0)` builder chain** —
   `with_density(_)`, `with_resolution(15)`, `with_shape_sdf(_)`;
   `validate() == Ok(())`; `params.shape_sdf.is_some()` (post-
   builder) and `is_some_and(...)` returns the expected `true` /
   `false` per the four spatial probes above.
3. **`generate_lattice` result counts (with-SDF vs without-SDF)** —
   two runs over the same bbox + same gyroid params; the only
   difference is whether `with_shape_sdf` is applied. The
   with-SDF result has strictly fewer vertices than the without-SDF
   result (strict `<` on `vertex_count()`). The trimmed lattice
   retains the full bbox `cell_count` (the count reports total
   bbox cells, not cells inside the SDF — F9-related, documented
   in the README).
4. **Per-vertex distance-to-origin bound** — every vertex `v` of
   the with-SDF result satisfies
   `(v - origin).norm() < sphere_radius + voxel_size + cushion`,
   with `voxel_size = 2/3` (exact integer multiple) and a small
   cushion accounting for marching-cubes interpolation across the
   sphere boundary.
5. **Edge case: tiny-sphere trim** — a sphere SDF with a
   sufficiently small radius produces a result with very few output
   vertices (or zero) compared to the bbox-fill — demonstrates that
   the SDF clip is real and not a no-op.

## Boundary-conforming vs bbox-filling contrast (with §5.5)

| | §5.5 `mesh-lattice-tpms-gyroid` | §5.8 `mesh-lattice-shape-bounded` |
|---|---|---|
| Topology | Gyroid (TPMS shell) | Gyroid (TPMS shell) |
| Bounds | 30 mm³ bbox-filling | 30 mm³ bbox, sphere-clipped at radius 12 |
| `with_shape_sdf` | None (default) | sphere SDF closure of radius 12 at origin |
| `is_outside_shape` | `false` for any point (default) | `sdf(p) > 0.0` |
| `shell_sdf` composition | `tpms_value` only | `tpms_value.max(sphere_sdf(p))` |
| Output extent | Fills the bbox | Trimmed to sphere interior |
| Load-bearing anchor | per-vertex `abs(abs(G(v)) - 0.75) < 0.05` | strict `<` on `vertex_count()` (with-SDF vs without) + per-vertex `norm(v) < 13.67` |
| Visual centerpiece | Bbox-filling gyroid (3³ ≈ 27 cells) | Sphere-shaped gyroid (boundary-conforming) |

## Public-surface coverage (per spec §5.8)

- `LatticeParams::gyroid` — TPMS path preset (where shape-conforming
  is most striking visually).
- `LatticeParams::with_shape_sdf` — accepts the
  `Arc<dyn Fn(Point3<f64>) -> f64 + Send + Sync>` shape-SDF closure
  per `mesh-lattice/src/params.rs:406`.
- `LatticeParams::is_outside_shape` accessor — returns
  `sdf(p) > 0.0` when set, `false` otherwise per `params.rs:415-417`.
- The `shell_sdf` composition pattern internally —
  `tpms_value.max(sdf(p))` (SDF intersection) per
  `mesh-lattice/src/generate.rs:441`.
- `generate_lattice` (TPMS path with shape SDF).
- The `Arc<dyn Fn>` convention for the shape SDF (per the existing
  `test_gyroid_lattice_with_shape_sdf` unit test in
  `mesh-lattice/src/generate.rs`).

## SDF convention recap

The `is_outside_shape(p)` predicate (`params.rs:415-417`) defines:

- `sdf(p) > 0.0` ⇒ **outside** (point is excluded from the lattice).
- `sdf(p) < 0.0` ⇒ **inside** (point is retained).
- `sdf(p) == 0.0` ⇒ **on the boundary surface** (treated as inside,
  since the predicate is strict `> 0.0`).
- No SDF set ⇒ `is_outside_shape(any_point) == false` (everything
  inside).

For the sphere SDF `|p| p.coords.norm() - 12.0`: points with
`norm < 12` return negative (inside); points with `norm > 12` return
positive (outside); points exactly on the sphere surface return zero
(retained).

## Numerical anchors (TBD — land in `§6.2 #20`)

- 4 `is_outside_shape` predicate anchors at known spatial points
  (origin, radius 11 interior, radius 13 exterior, radius 20 far
  exterior) — boolean equality.
- 1 default-no-SDF `is_outside_shape == false` anchor.
- `LatticeParams::gyroid(10.0).with_shape_sdf(_)` builder anchor:
  `validate() == Ok(())`, `shape_sdf.is_some()`.
- With-vs-without `generate_lattice` comparison:
  `result_with.vertex_count() < result_without.vertex_count()`
  (strict inequality).
- Per-vertex distance-to-origin bound: every `v` of the with-SDF
  result satisfies `(v - origin).norm() < 12 + 2/3 + cushion`
  (with the cushion picked to absorb marching-cubes boundary
  interpolation).
- Tiny-sphere edge-case anchor: an even smaller sphere radius yields
  a sharply smaller output vertex count.

## Run (TBD — land in `§6.2 #20`)

```text
cargo run -p example-mesh-mesh-lattice-shape-bounded --release
```

Output: `out/sphere_gyroid.ply` (the trimmed sphere-bounded lattice);
optionally `out/sphere_gyroid_full.ply` (the un-trimmed comparison
under a `WRITE_COMPARISON: bool = true` constant for reviewer
clarity).

## Visuals (TBD — land in `§6.2 #20`)

⏸ Visual review **recommended** — sphere-bounded gyroid is visually
iconic; the with-vs-without comparison tells the boundary-conforming
story. The sphere-clipped output should be visibly contained inside
a 12 mm radius from origin while still showing the gyroid's
characteristic twisted-saddle wall structure.

## Cross-references

- **Spec**: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.8 (this example).
- **Sister examples**: §5.5 `mesh-lattice-tpms-gyroid` at
  `947aa8d8` (TPMS bbox-filling counterpart — this example is the
  shape-clipped variant of the same path); §5.9
  `mesh-lattice-mesh-bounded-infill` (the mesh-bounded composite
  path via `generate_infill`, complementary to this analytical-SDF
  path).
- **Mesh book**: `docs/studies/mesh_architecture/src/80-examples.md`
  — Part 8 inventory (depth-pass updates land at the v1.0 closeout
  commit of the arc).
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md)
  — clean exit-0 gates the visuals pass;
  [`feedback_examples_drive_gap_fixes`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_examples_drive_gap_fixes.md)
  — example impl recon may surface in-arc platform fixes (precedent:
  C15a octet-truss beam-data parity fix between §6.2 #18 skeleton
  and fixture).
