# mesh-lattice-shape-bounded

**Boundary-conforming TPMS lattice clipped to an analytical sphere
SDF via `with_shape_sdf` — direct `is_outside_shape` predicate
anchors (interior / exterior + the default `false` when no SDF is
set), then `generate_lattice` for a 30 mm³ bbox centered at origin
(`min = (-15, -15, -15)`, `max = (15, 15, 15)`) at cell size 10 mm
and resolution 15, with a sphere SDF of radius 12 mm at origin
(`Arc::new(|p| p.coords.norm() - 12.0)`) clipping the gyroid
output.** The shape-bounded counterpart to
`mesh-lattice-tpms-gyroid`: same TPMS path, but the lattice is
trimmed by a mathematical shape (analytical SDF) rather than filling
the whole axis-aligned bounding box. Complementary to
`mesh-lattice-mesh-bounded-infill`, which exercises the
mesh-bounded composite path via `generate_infill`.

## What this example demonstrates

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
surface." The `mesh-lattice-tpms-gyroid` example covers the
bbox-filling TPMS path (no shape SDF); this example covers the
shape-clipped TPMS path.

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
`(v - origin).norm() < 12 + voxel_size + cushion ≈ 13.667` — the
cushion accounts for marching-cubes interpolating across cells
straddling the sphere boundary.

The example computes:

1. **Direct `is_outside_shape` predicate anchors** — 4 spatial
   evaluations against the sphere-radius-12 SDF at known points
   (origin + a clearly-interior point at radius 11 + a clearly-
   exterior point at radius 13 + a far-exterior point at radius 20)
   plus the default `is_outside_shape == false` for any point when
   no SDF is set on `LatticeParams::gyroid(10.0)` (per
   `params.rs:415-417`). All exact (boolean equality).
2. **`LatticeParams::gyroid(10.0)` builder chain** —
   `with_density(0.3)`, `with_resolution(15)`, `with_shape_sdf(_)`;
   `validate() == Ok(())`; `params.shape_sdf.is_some()`; and field
   locks pinning `cell_size`, `density`, and `resolution` to the
   fixture values.
3. **`generate_lattice` result counts (with-SDF vs without-SDF)** —
   two runs over the same bbox + same gyroid params; the only
   difference is whether `with_shape_sdf` is applied. The with-SDF
   result has strictly fewer vertices than the without-SDF result
   (strict `<` on `vertex_count()`). The trimmed lattice still
   retains the full bbox `cell_count` (the count reports total bbox
   cells, not cells inside the SDF — F9-related, documented below).
4. **Per-vertex distance-to-origin bound** — every vertex `v` of the
   with-SDF result satisfies
   `(v - origin).norm() < sphere_radius + voxel_size + cushion`
   `≈ 13.667 mm`, with `voxel_size = 2/3` (exact integer multiple)
   and a `1.0 mm` cushion accounting for marching-cubes interpolation
   across the sphere boundary.
5. **Edge case: tiny-sphere trim** — a sphere SDF of radius `1 mm`
   produces a result with order-of-magnitude fewer output vertices
   than the radius-12 trim, demonstrating that the SDF clip is real
   and not a no-op.

## Boundary-conforming vs bbox-filling contrast

| | `mesh-lattice-tpms-gyroid` | `mesh-lattice-shape-bounded` |
|---|---|---|
| Topology | Gyroid (TPMS shell) | Gyroid (TPMS shell) |
| Bounds | 30 mm³ bbox-filling | 30 mm³ bbox, sphere-clipped at radius 12 |
| `with_shape_sdf` | None (default) | sphere SDF closure of radius 12 at origin |
| `is_outside_shape` | `false` for any point (default) | `sdf(p) > 0.0` |
| `shell_sdf` composition | `tpms_value` only | `tpms_value.max(sphere_sdf(p))` |
| Output extent | Fills the bbox | Trimmed to sphere interior |
| Load-bearing anchor | per-vertex `abs(abs(G(v)) - 0.75) < 0.05` | strict `<` on `vertex_count()` (with-SDF vs without) + per-vertex `norm(v) < 13.667` |
| Visual centerpiece | Bbox-filling gyroid (3³ ≈ 27 cells) | Sphere-shaped gyroid (boundary-conforming) |

## Public-surface coverage

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

## Numerical anchors

| Anchor | Locked value |
|---|---|
| `is_outside_shape((0, 0, 0))` (sdf = -12) | `false` (interior) |
| `is_outside_shape((11, 0, 0))` (sdf = -1) | `false` (interior) |
| `is_outside_shape((13, 0, 0))` (sdf = +1) | `true` (exterior) |
| `is_outside_shape((20, 0, 0))` (sdf = +8) | `true` (far exterior) |
| no-SDF default `is_outside_shape(any_point)` | `false` (per `is_some_and` short-circuit) |
| Builder validate | `Ok(())` |
| `params.shape_sdf.is_some()` (post-builder) | `true` |
| `params.cell_size` / `params.density` / `params.resolution` | `10.0` / `0.3` / `15` |
| `result_with_sdf.vertex_count()` | `87480` (trimmed) |
| `result_without_sdf.vertex_count()` | `321084` (bbox-filling baseline) |
| With-vs-without strict-`<` on vertex_count | trim drops `233604` verts (`72.8%`) |
| Empirical max `norm(v)` over with-SDF result | `11.9999 mm` |
| Per-vertex bound `< 12 + 2/3 + 1.0 ≈ 13.667 mm` | green (cushion comfortably oversized) |
| `result.cell_count` (with-SDF) | `27` (BIT-EXACT; total bbox cells, not trimmed) |
| `tiny.vertex_count()` (radius-1 sphere) | `24` (`< 87480 / 10 = 8748`) |

The empirical max distance-to-origin is `11.9999 mm` — slightly
*inside* the sphere boundary, not beyond it. The `voxel_size + cushion`
allowance is therefore comfortably oversized for this fixture; the
bound `13.667` holds with ~1.7 mm of headroom. This is consistent
with marching-cubes producing edge vertices via linear interpolation
between corners that bracket the `shell_sdf == 0` isovalue: when one
corner is outside the sphere (`shell_sdf == sphere_sdf > 0`) and one
inside, the interpolated vertex lands between them, and at this density
+ resolution the interpolation typically lands just inside the sphere
boundary rather than beyond it. The cushion remains in the bound to
absorb FP wobble at thinner shell densities or other shape SDFs where
the interpolation could land further out.

The `cell_count == 27` anchor is F9-related: the TPMS path computes
`cells_x × cells_y × cells_z` pre-trim at
`mesh-lattice/src/generate.rs:417`, so SDF clipping cannot reduce this
count. The trimmed `vertex_count()` shrinks by 72.8% (`87480` vs
`321084`), but `cell_count` reports the bbox geometry, not the post-
trim mesh.

## Run

```text
cargo run -p example-mesh-mesh-lattice-shape-bounded --release
```

Expected exit `0`; the binary asserts every anchor (boolean / strict
inequality / per-vertex bound / `BIT-EXACT` `cell_count`) and prints a
museum-plaque summary plus an `OK — …` confirmation banner.

Artifacts written to `out/` (gitignored at the repo level):

- `out/sphere_gyroid.ply` — the trimmed sphere-bounded lattice
  (`87480` vertices, `29160` faces, binary little-endian).
- `out/sphere_gyroid_full.ply` — the un-trimmed comparison (`321084`
  vertices, `107028` faces, binary little-endian), written behind a
  `WRITE_COMPARISON: bool = true` constant in `src/main.rs` (toggle
  to `false` to skip; the comparison helps a reviewer see the trim
  effect side-by-side).

## Visuals

⏸ Visual review **recommended** — sphere-bounded gyroid is visually
iconic; the with-vs-without comparison tells the boundary-conforming
story.

```text
f3d out/sphere_gyroid.ply
f3d out/sphere_gyroid_full.ply
```

The sphere-clipped output should be visibly contained inside a `12 mm`
radius from origin while still showing the gyroid's characteristic
twisted-saddle wall structure (the same TPMS topology as the
bbox-filling `mesh-lattice-tpms-gyroid` example, but trimmed to the
analytical sphere boundary). The un-trimmed comparison fills the full
30 mm bbox with bbox-edge-aligned slab faces where marching-cubes
terminates at the bounding box. F10 vertex-soup faceting is visible
on curl crests in both meshes (platform-truth, same as the bbox-filling
example — F10 weld pass is a v0.9 backlog candidate).

## Cross-references

- **Sister examples**: `mesh-lattice-tpms-gyroid` (TPMS bbox-filling
  counterpart — this example is the shape-clipped variant of the
  same path); `mesh-lattice-mesh-bounded-infill` (the mesh-bounded
  composite path via `generate_infill`, complementary to this
  analytical-SDF path).
- **Mesh book**: `docs/studies/mesh_architecture/src/80-examples.md`
  — Part 8 inventory.
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md)
  — clean exit-0 gates the visuals pass;
  [`feedback_examples_drive_gap_fixes`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_examples_drive_gap_fixes.md)
  — example impl recon may surface in-arc platform fixes (precedent:
  the octet-truss beam-data parity fix surfaced during the density-
  gradient example's authoring).
