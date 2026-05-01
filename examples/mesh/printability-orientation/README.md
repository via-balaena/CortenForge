# `mesh-printability-orientation`

Visual demo of Gap L (`PrinterConfig::build_up_direction` parametrization)
of the v0.8 fix arc.

## ⚠ f3d viewer callout — pass `--up +Z` to see the cylinder upright

The fixture's leaning cylinder is authored in a `+Z`-up world (build
plate at `z = 0`, axis tilted 60° from vertical toward `+X`). `f3d`'s
default `+Y`-up world rotates the scene so the build-plate-perpendicular
direction goes *into the screen*, which makes the cylinder appear to
*lie* on its side rather than *lean*. Always pass `f3d --up +Z` for this
example:

```text
f3d --up +Z out/mesh_original.ply                # leaning cylinder
f3d --up +Z out/mesh_rotated.ply                 # post-R_Y(-60°) — perfectly vertical
f3d --up +Z --multi-file-mode=all out/mesh_original.ply out/issues_run1.ply  # overhang centroids overlay
```

To compare original vs rotated side-by-side: open both PLYs in two f3d
windows. The rotated cylinder is OFFSET in `−X` (bbox `x ∈ [-12, -2]`)
because the rotation pivots around the world origin, not around the
mesh centroid; this is geometrically expected — `place_on_build_plate`
re-aligns `z`, but `x` / `y` translation from the rotation is preserved.
Both meshes have identical signed volume (`1170.5 mm³`) and identical
vertex/face counts (`66v / 128f`), confirming the rotation is
orientation-preserving.

## What this fixture is

A single hand-authored watertight cylinder:

- **Radius**: `5 mm`
- **Length**: `15 mm` (axial, end-cap to end-cap)
- **Axis direction**: `(sin(60°), 0, cos(60°)) ≈ (0.866, 0, 0.5)` — the
  cylinder leans 60° from vertical toward `+X`
- **Tessellation**: 32 azimuthal segments (chamfered at default zoom,
  not smooth-curved per `feedback_chamfered_not_rounded`)
- **Topology**: 66 vertices + 128 triangles (2 cap centres + 2 × 32
  rim verts; 32 negative-cap fans + 32 positive-cap fans + 64 lateral
  triangles in 32 segments × 2)

After `place_on_build_plate`, bbox is approximately
`x ∈ [-9, +9], y ∈ [-5, +5], z ∈ [0, 16.16]`.

## The Gap L invariant — the load-bearing claim

Gap L (§5.6 of the v0.8 fix arc) added `PrinterConfig::build_up_direction`,
parametrizing the validator's notion of "up". This example demonstrates
the **Gap L invariant**: rotating the mesh by an exact rotation `R` ≡
setting the build-up direction to `R⁻¹ · Z` (with `R = R_Y(-60°)`,
`R⁻¹ · Z = R_Y(+60°) · Z = (sin(60°), 0, cos(60°)) = axis`).

The example asserts this equivalence directly:

```rust
approx::assert_relative_eq!(run2_overhang_area, run3_overhang_area, epsilon = 1e-6);
```

where Run 2 applies the exact rotation to the mesh and Run 3 keeps the
mesh untouched but sets `with_build_up_direction(axis)`. Both produce
zero overhang area; the assertion is `0 == 0` at the FP level but
semantically the load-bearing equivalence claim.

## Why isn't `find_optimal_orientation` Run 2?

`find_optimal_orientation` is a **search heuristic** over a discrete
sample set, NOT the mathematical equivalence operator. The 12-sample
default is `{identity, ±90°/180° X, ±90° Y, ±45°/±135° X, +45° Y}` —
none of which is `R_Y(-60°)`, the exact compensating rotation needed
for this fixture.

For the `R = 5 mm` / `L = 15 mm` cylinder, the search picks **identity**
(no rotation) — every non-identity sample either preserves or worsens
the overhang area for this aspect ratio. The closest non-trivial
candidate is `R_Y(-90°)` which drives the lateral overhang to 0 but
tilts the bottom cap 30° from horizontal, exposing ~30 of 32 cap-fan
triangles to a comparable post-rotation overhang.

The example calls `find_optimal_orientation(&mesh, &config, 12)` after
the 3 runs and prints the picked rotation + resulting `overhang_area`
to stdout — this is a **diagnostic, NOT a load-bearing assertion**. The
diagnostic illustrates the search's discrete-sample limitation directly:
the heuristic can fail to improve over identity entirely. For an EXACT
mathematical equivalence test, a manually-constructed exact rotation is
required (which is what Run 2 does).

**v0.9 candidate**: enrich `find_optimal_orientation`'s sampler (e.g.,
1° angle bins around primary axes, or a gradient-descent refinement
step) so it can reach arbitrary axis-aligned rotations like
`R_Y(-60°)`.

## Length deviation from spec (30 mm → 15 mm)

V08_FIX_ARC_SPEC.md §7.6 originally specified `LENGTH = 30 mm`. At
that length, `mesh-repair`'s `detect_self_intersections` (called from
`check_self_intersecting`) fires **8 false-positive `SelfIntersecting`
`Critical` pairs** between diametrically-opposite lateral triangles
(empirically reproduced at SEGMENTS ∈ {8, 12, 16, 32}; the threshold
is around L ≈ 18 mm regardless of segment count). The 30:1 lateral-
triangle aspect ratio at R=5 / L=30 / SEG=32 trips an FP edge case in
mesh-repair's BVH triangle-pair test (suspected cause: opposite-side
tangent-plane intersection within FP tolerance bounds).

Reducing `L` to 15 mm drops the aspect ratio to 15:1 (still thin, but
below the false-positive threshold) and yields a clean
`is_printable() == true` Run 1 with 2 Info-only `OverhangRegion`s. The
spec's expected `[100, 250]` mm² area band was sized for L=30; the
example asserts the empirically-observed band `[50, 100]` for L=15.

**v0.9 candidate logged**: investigate mesh-repair
`detect_self_intersections` sensitivity to thin-aspect-ratio cylindrical
lateral triangles; the same fixture shape at L=15 produces zero false-
positives, while L≥18 consistently flags 8 cross-cylinder pairs
regardless of segment count.

## Run-by-run breakdown

| Run | Mesh | Config (`build_up_direction`) | Expected outcome |
|-----|------|-------------------------------|------------------|
| 1 | Original (leaning) | Default `+Z up` | 2 OverhangRegions, total area ~66 mm² (Info), `is_printable() = true` |
| 2 | Manual `R_Y(-60°)` rotation | Default `+Z up` | 0 OverhangRegions, area = 0 |
| 3 | Original (leaning) | `axis = (sin60°, 0, cos60°)` | 0 OverhangRegions, area = 0 |
| Diagnostic | Original (leaning) | Default + `find_optimal_orientation(..., 12)` | Identity rotation picked, area = 66 mm² (no improvement) |

**Run 2 vs Run 3 — why have both?** They're **architecturally different**:
Run 2 modifies the mesh; Run 3 modifies the config. For workflows that
don't want to rotate the mesh (preserves authoring frame, allows
comparing orientations without rebuilding the mesh), Run 3 is the right
tool. For workflows where the rotated mesh IS the artifact (e.g., for
post-rotation slicing), Run 2 is the right tool. The example
demonstrates both equivalence and the use-case difference.

## Numerical anchors

Per V08_FIX_ARC_SPEC.md §7.6 (corrected from the spec's original
`find_optimal_orientation`-based Run 2 to a manual exact rotation; see
"Why isn't `find_optimal_orientation` Run 2?" above):

1. Run 1: `validation.overhangs.len() >= 1`; total overhang area
   ∈ `[50, 100]` mm² (analytical downhill arc minus build-plate-
   filtered cluster ≈ 66 mm² at R=5 / L=15 / SEG=32).
2. Run 1: at least one `ExcessiveOverhang` issue with `Info` severity
   (per §4.3 boundary: `45° < observed ≤ 60°` band; max observed
   59.526° due to 32-segment chord discretization).
3. Run 1: `is_printable()` printed but NOT asserted (cylinder has no
   thin walls / cavity / self-intersection / small features at this
   scale, and ExcessiveOverhang at Info severity does NOT block; the
   value is `true` but not load-bearing).
4. Run 2: `validation.overhangs.len() == 0`; total overhang area = 0
   within `1e-6` mm² (exact rotation makes cylinder perfectly vertical;
   bottom cap flat on plate → all build-plate-filtered).
5. Run 3: `validation.overhangs.len() == 0`; total overhang area = 0
   within `1e-6` mm² (up-vector aligned with axis → all bottom-cap
   rim vertices share `axis-projection = mesh_min_along_up` → all
   build-plate-filtered).
6. **Gap L invariant**: `assert_relative_eq!(run2_area, run3_area,
   epsilon = 1e-6)` — semantically the load-bearing equivalence
   (mesh-rotation ≡ up-vector-rotation when both are exact); trivially
   `0 == 0` at the FP level.
7. Run 2 area STRICTLY LESS than Run 1 area — the exact rotation
   eliminates support need.
8. All three Runs print `overhangs.len()` + total area to stdout.
9. Manual rotation correctness: `debug_assert!` that
   `R_Y(-60°) × axis_pre - (0, 0, 1)` has norm < `1e-12` (locks the
   construction at impl time).
10. Diagnostic `find_optimal_orientation` print — picked rotation +
    resulting `overhang_area`; no value-level assertion (the picked
    sample depends on sample-set ordering, which may shift in v0.9
    enrichment).

## How to run

```text
cargo run -p example-mesh-printability-orientation --release
```

`--release` is recommended (the diagnostic `find_optimal_orientation`
call evaluates each of the 12 samples against the full mesh; debug-mode
runs are roughly 5× slower). Output is written to
`examples/mesh/printability-orientation/out/`:

- `out/mesh_original.ply` — 66-vertex, 128-triangle ASCII PLY of the
  leaning cylinder, post-`place_on_build_plate`.
- `out/mesh_rotated.ply` — 66-vertex, 128-triangle ASCII PLY of the
  cylinder after manual `R_Y(-60°)` + `place_on_build_plate`. Axis
  perfectly vertical; cylinder stands on its bottom end-cap. Bit-exact
  signed volume preservation vs `mesh_original.ply` (1170.54 mm³ both).
- `out/issues_run1.ply` — vertex-only ASCII PLY of 2 `OverhangRegion`
  centroids from Run 1.
- `out/issues_run2.ply`, `out/issues_run3.ply` — empty point-clouds
  (zero overhang regions); demonstrates the "no issues = no points"
  rendering convention shared with `printability-self-intersecting`'s
  single-cylinder regression case.

## v0.8 fix arc cross-references

- §5.6 — Gap L `build_up_direction` parametrization (landed earlier
  in the arc; this example consumes the parametrization).
- §7.6 — this example's spec (corrected from the original
  `find_optimal_orientation`-based Run 2 to a manual exact rotation;
  see V08_FIX_ARC_SPEC.md §7.6 "Why not `find_optimal_orientation`
  for Run 2?" sub-section).
- `mesh_printability::orientation::find_optimal_orientation` /
  `apply_orientation` / `place_on_build_plate` — the orientation
  infrastructure this example exercises.
- `mesh_printability::config::PrinterConfig::with_build_up_direction`
  — the Gap L parametrization's builder API.
