# `mesh-printability-small-feature`

Visual demo of the §6.5 `SmallFeature` detector (Gap J of the v0.8 fix
arc).

## ⚠ f3d viewer callout — pass `--up +Z` and zoom to the burr

`f3d` defaults to a `+Y`-up world, so the cube renders on its side and
the build plate goes into-screen. Pass `f3d --up +Z out/mesh.ply` to
match the FDM print orientation (build plate horizontal). The burr is
at `(35, 15, 0.1)` and is **only 0.2 mm wide** — about 1:150 of the
cube's 30 mm side — so it is invisible at default framing. Zoom to
`(35, 15, 0.1)` at ≥ 100× magnification (in `f3d`, click-drag with the
mouse to orbit, then `+`/`-` to zoom) to see the 6-segment hex prism.

To overlay the centroid markers from `out/issues.ply`, use
`f3d --up +Z --multi-file-mode=all out/mesh.ply out/issues.ply`. There
are 2 markers — the `SmallFeature` and `ThinWall` regions both
localize at `(35, 15, 0.1)`, so the markers stack at the same point.
Press `O` to toggle point sphere rendering on the issues point cloud.

## What this fixture is

Two hand-authored components concatenated **without** any boolean
union, welding, or vertex sharing — the kind of artefact a CAD
boolean cut leaves behind when a tiny island of geometry survives the
operation:

- **Main cube**: a 30 × 30 × 30 mm solid cube on the build plate
  (spans `(0,0,0)` to `(30, 30, 30)`). 8 vertices + 12 triangles,
  watertight and outward-wound.
- **Hex-prism burr**: a 6-segment "cylinder" with circumradius
  0.1 mm (vertex-to-vertex diameter 0.2 mm) and height 0.2 mm,
  sitting on the build plate at horizontal centre `(35, 15)` and
  spanning `z ∈ [0, 0.2]`. 14 vertices (12 rim verts + 2 hub centres)
  + 24 triangles (12 lateral + 6 top fan + 6 bottom fan), watertight
  and outward-wound.

The combined assembly has 22 vertices + 36 triangles total.
Edge-adjacency partition under the §6.5 detector treats the cube and
burr as two distinct components.

## Why on-plate placement matters

A floating burr would expose its bottom hex fan as a downward-facing
face, and Gap M's overhang predicate would flag it Critical (a
0.2 mm dot in mid-air with no support). Placing the burr **on the
build plate** (`z_min = 0`, matching the cube's `z_min = 0` so
`mesh_min_along_up = 0`) means the burr's bottom fan is build-plate-
filtered and never reaches the overhang classifier. This keeps the
example's load-bearing concept clean: only the `SmallFeature` Warning
fires + the `ThinWall` co-flag (see below). No overhang, no trapped
volume, no self-intersection clutter.

## Why a hex prism, not a cylinder

The §6.5 detector needs a **discretely-faceted** geometry with a
locked-in face count for the `region.face_count == 24` numerical
anchor. A smooth cylinder would require choosing a tessellation
resolution; the regular hexagon is the simplest closed polygon that
still produces a credible "round burr" silhouette under MeshLab/f3d
rendering. Six lateral quads (= 12 lateral triangles) + 6 top fan
triangles + 6 bottom fan triangles = exactly 24, regardless of
azimuth-resolution choices.

## What the SmallFeature detector reports

| Metric | Value |
|--------|-------|
| `small_features.len()` | `1` (only the burr; cube max-extent 30 mm ≫ 0.8 mm threshold) |
| `region.center` | `(35, 15, 0.1)` (within `1e-6`) |
| `region.face_count` | `24` (locked by hex-prism construction) |
| `region.max_extent` | `0.2 mm` (vertex-to-vertex hex diameter; FP-stable in `[0.199, 0.201]`) |
| `region.volume` | `5.196e-3 mm³` (divergence-theorem `(3√3/2)·r²·h` within `1e-3 max_relative`) |
| Severity | `Warning` (`0.2 < 0.8/2 = 0.4`) |

The detector's classifier maps `max_extent < min_feature_size / 2`
to `Warning`, else to `Info`. There is no `Critical` band for
`SmallFeature` — small features are **advisory**, not blocking.

## ThinWall co-flag — `is_printable() == false`, but not because of `SmallFeature`

The §7.5 spec line 1746 anticipated `is_printable() == true` for
this fixture, on the reasoning that `SmallFeature` Warning alone
does not block printability. That holds at the `SmallFeature`-
detector level. **However**, the §6.1 `ThinWall` detector (added
in row #10 of this same v0.8 arc, after the §7.5 spec was first
drafted) ALSO fires on the burr:

- The inward ray-cast from each burr face's centroid travels
  `r · √3 ≈ 0.173 mm` (flat-to-flat hex distance) before hitting
  the opposite lateral face.
- The `ThinWall` classifier maps `thickness < min_wall / 2`
  (`0.173 < 1.0 / 2 = 0.5`) to **Critical**.
- One `ThinWall` Critical issue + one `SmallFeature` Warning issue
  → `is_printable() == false`, blocked by `ThinWall`, not by
  `SmallFeature`.

This is a **spec deviation** documented in the commit body and the
example's module doc-comment. The two detectors agree on the same
defect from complementary angles — `SmallFeature` sees "this
component is too small overall" while `ThinWall` sees "every face
is closer to its opposite than `min_wall_thickness` allows" — and
that complementary-diagnostic property is pedagogically useful.
Anchor #8 in `main()` is corrected to `is_printable() == false`;
new anchor #9 locks the `ThinWall` Critical co-flag observation.

## "Warning ≠ Critical" — the load-bearing pedagogy

Even with the `ThinWall` co-flag, the load-bearing teaching of this
example holds: **`SmallFeature` Warning by itself does not block
`is_printable()`**. To verify: comment out anchor #9, rebuild
mentally with only the `SmallFeature` Warning issue (say, on a
fixture where `ThinWall` doesn't apply — e.g. a tiny floating
surface island that's open and fails ThinWall's watertight
precondition), and `is_printable()` returns `true`. The user reading
a `SmallFeature Warning` should treat it as "inspect this CAD region
for a stray fragment", not "this print will fail". Critical issues
are the print-blocking band.

## Numerical anchors

1. `validation.small_features.len() == 1` — exactly one
   `SmallFeature` region (the burr).
2. `region.center` within `1e-6` of `(35, 15, 0.1)`.
3. `region.face_count == 24` — locked-in by `12 lateral + 6 top fan
   + 6 bottom fan` hex-prism construction.
4. `region.max_extent ∈ [0.199, 0.201]` — vertex-to-vertex hex
   diameter, FP roundoff tolerated.
5. `region.volume ≈ 5.196e-3 mm³` within `1e-3 max_relative` —
   divergence-theorem volume of a hex prism with `r = 0.1 mm`,
   `h = 0.2 mm`: `(3√3/2) · r² · h`.
6. `SmallFeature` issue severity is `Warning`.
7. `validation.overhangs.len() == 0` — both component bottoms are
   build-plate-filtered; lateral hex faces have `normal.z = 0` so
   are vertical walls, not overhangs.
8. **CORRECTED**: `validation.is_printable() == false` — `ThinWall`
   Critical co-flag on the burr blocks printability. The §7.5 spec
   anticipated `true`; the spec did not account for the `ThinWall`
   detector's behaviour on solid sub-millimetre components.
9. At least one `ThinWall` Critical issue exists, locking the co-flag
   that drives anchor #8's correction.

All asserted in `main()` — a failed numerical anchor breaks the run.

## How to run

```text
cargo run -p example-mesh-printability-small-feature --release
```

`--release` matches the workspace convention for any non-trivial
validation walk; the SmallFeature detector itself is `O(n_faces)`
so debug mode runs in well under a second on the 36-tri fixture,
but the convention applies to all examples uniformly. Output is
written to `examples/mesh/printability-small-feature/out/`:

- `out/mesh.ply` — 22-vertex, 36-triangle ASCII PLY (cube + burr).
- `out/issues.ply` — vertex-only ASCII PLY of 2 region centroids
  (1 `SmallFeature.center` + 1 `ThinWallRegion.center`). Both
  centroids localize at `(35, 15, 0.1)` since they describe the
  same component; the markers stack visually.

## Pitfalls

### Why doesn't the main cube flag?

The main cube's `max_extent = 30 mm` is well above
`min_feature_size = 0.8 mm`. The detector is bbox-extent-based, not
volume-based; a large solid part with no tiny features passes
unflagged regardless of its absolute volume.

### What if I scale the model 1000× by mistake (mm vs m)?

The main cube becomes 30 m and the burr becomes 0.2 m — neither
flags as `SmallFeature` (both extents are well above 0.8 mm), but
`ExceedsBuildVolume` (Critical) fires instead, since the cube no
longer fits the FDM default `200 × 200 × 200 mm` build volume. A
**unit-detection heuristic** that warns "this mesh's bbox is 1000×
larger / smaller than the printer's build volume; check your units"
is a v0.9 followup candidate — useful for catching the metres-vs-
millimetres CAD-export error before the user assumes any other
detector's diagnosis.

### Why `Warning` and not `Info` for a 0.2 mm burr?

`classify_small_feature_severity` uses a two-band threshold:
`max_extent < min_feature_size / 2` → `Warning` (definitely below
resolution); else → `Info` (borderline; may print). The burr's
0.2 mm extent is well below `0.4 mm`, so it lands in the `Warning`
band. A 0.5 mm burr (between `min_feature_size / 2 = 0.4` and
`min_feature_size = 0.8`) would land in `Info` — borderline; might
print, might not.

## v0.8 fix arc cross-references

- §6.5 — `SmallFeature` detector (landed at row #18, commit
  `b91a7421`)
- §7.5 — this example's spec
- §9.2.7 — small-feature stress fixtures
- `mesh_printability::SmallFeatureRegion` — the typed-region
  surfaced by the detector
- §6.1 `ThinWall` co-flag — landed at row #10, drives anchor #8's
  correction
