# `mesh-printability-self-intersecting`

Visual demo of the §6.4 `SelfIntersecting` detector (Gap I of the v0.8
fix arc).

## ⚠ f3d viewer callout — interpenetrating cylinders render cleanly, but watch the cross zone

`f3d out/mesh.ply` works fine for the overall geometry: a 3-D plus-
sign cross with the two cylinders crossing at the origin. But the
self-intersection zone near `(0, 0, 5)` is where the two faceted
lateral surfaces interpenetrate, so the renderer paints triangles that
visually overlap and z-fight along the four interpenetration "rings".
**That visual mess is exactly what the detector flags.** It is the
load-bearing geometry of the example.

To inspect the four interpenetration rings:

- **`f3d`**: f3d defaults to a `+Y`-up world, so cylinder B (axis
  `+Y`) renders vertically and the build-plate-perpendicular `+Z` goes
  into-screen. Pass `f3d --up +Z out/mesh.ply` to match the FDM print
  orientation (build plate horizontal); the canonical `+`-shape view
  is then looking down `+Z`. Toggle edges (`E`) to see the 16-segment
  lateral facets where the cylinders cross. To overlay the 100
  self-intersection markers + 4 lateral-overhang markers from
  `out/issues.ply`, use `f3d --up +Z --multi-file-mode=all
  out/mesh.ply out/issues.ply` — f3d's default multi-file mode loads
  both as separate file groups (showing one at a time; cycle with
  `→` / `←`), so `--multi-file-mode=all` is what you want for the
  overlay. Press `O` to toggle point sphere rendering on the issues
  point cloud.
- **MeshLab** (`meshlab out/mesh.ply`): enable `Render → Show Edges`
  and `Render → Show Vertices` to see the interpenetration zone
  clearly. The lateral surfaces' z-fighting in the cross region is the
  detector's input signal.
- **ParaView** (`paraview out/mesh.ply`): a `Clip` filter at `z = 5`
  reveals the cross-section through the interpenetration zone — the
  four ring intersections show up as four bow-tie crossings.

## What this fixture is

Two hand-authored cylinders concatenated **without** any boolean
union, welding, or vertex sharing:

- **Cylinder A**: axis along `+X`, length 30 mm, radius 5 mm,
  centred at origin (spans `x ∈ [-15, +15]`).
- **Cylinder B**: axis along `+Y`, length 30 mm, radius 5 mm, also
  centred at origin (spans `y ∈ [-15, +15]`).

Each cylinder has 16 azimuthal segments → 34 vertices + 64
triangles per cylinder (2 cap centres, 32 rim verts; 16 cap-tris × 2
+ 32 lateral-tris). The combined assembly has 68 vertices + 128
triangles. After `place_on_build_plate`, the assembly bounding box is
`x ∈ [-15, +15], y ∈ [-15, +15], z ∈ [0, 10]` and the original origin
maps to `(0, 0, 5)`.

## Why two interpenetrating cylinders, with no boolean cleanup

The `SelfIntersecting` detector flags every pair of triangles that
physically intersect in space, with the exception of pairs that share
a vertex (mesh-repair's `IntersectionParams::skip_adjacent = true`
filter). Two independently-authored cylinders that share **no**
vertices and pass through each other's lateral surfaces produce dozens
of un-skipped pair flags — exactly the "broken-mesh-from-STL-export"
shape a CAD user might encounter after booleaning two primitives
without union cleanup.

A clean boolean union would merge the lateral surfaces, weld at the
intersection rings, and produce a single watertight shell with **zero**
self-intersections. Skipping the union is what makes this fixture
useful as a self-intersection demo.

## Analytical structure of the intersection

Cylinder A's lateral surface satisfies `y² + z² = 25` for `x ∈
[-15, +15]`. Cylinder B's: `x² + z² = 25` for `y ∈ [-15, +15]`.
Subtracting → `y² = x²` → `y = ±x`, and combined with `z² = 25 − x²`
gives **four space-curves** (rings):

| Ring | Equation | x range |
|------|----------|---------|
| 1 | `y = +x, z = +√(25 − x²)` | `[-5, +5]` |
| 2 | `y = +x, z = −√(25 − x²)` | `[-5, +5]` |
| 3 | `y = −x, z = +√(25 − x²)` | `[-5, +5]` |
| 4 | `y = −x, z = −√(25 − x²)` | `[-5, +5]` |

The four rings meet at the four "corner" points `(±5, ±5, 0)`
(`y = ±x`, `x = ±5`, `z = 0`). mesh-repair flags every `(A_face,
B_face)` pair where the discretized A-tri and B-tri both span the
ring; the count grows roughly with `4 rings × segments-per-ring`.

**Cap pairs do not contribute**: cylinder A's caps are at `x = ±15`,
which sits outside cylinder B's lateral surface region (`|x| ≤ 5`).
Symmetrically for B's caps. So every flagged pair is
`(A_lateral, B_lateral)` — directly readable from the `face_a` /
`face_b` indices in the diagnostic output (lateral faces of cylinder A
land at face indices 32–63 within A; lateral B faces are at 96–127
of the combined assembly post-concatenation).

## What the detector reports

| Metric | Value |
|--------|-------|
| `self_intersecting.len()` | `100` (mesh-repair `max_reported` cap; stable across runs) |
| Description text (typical) | `"~101–104 self-intersecting triangle pair(s) (search truncated; total may be higher)"` |
| Severity | `Critical` (slicer behaviour on self-intersection is undefined) |
| `is_printable()` | `false` |
| `overhangs.len()` | `4` (lateral underside co-flag, not load-bearing) |

The 100-vs-101+ difference reflects mesh-repair's contract:
`result.intersection_count` (used in the description) tracks the
actual total, while `result.intersecting_pairs.len()` (which becomes
`validation.self_intersecting.len()`) is capped at `max_reported = 100`.
The total is reported by per-thread counters and varies slightly
across runs (Rayon parallelism non-determinism: 101 → 104 in our
samples — each thread stops at its local cap hit, so the joined total
exceeds 100 by however many threads were mid-pair when the cap was
reached). The truncation suffix flags the cap hit unambiguously, and
the example's anchor #2 conditional asserts the correct
`(len == cap) ⟺ (suffix present)` relationship either way.

## `ExcessiveOverhang` co-flag — documented, not load-bearing

Cylinders placed with horizontal axes have lateral arcs whose normals
span every direction perpendicular to the axis. Bottom-most lateral
faces touching `mesh_min_along_up = 0` are build-plate-filtered per
Gap M; the next-up ring of faces is not filtered, so each cylinder
produces ≥ 1 `OverhangRegion`.

For the 16-segment fixture the lateral faces just above the bottom-
most segment have radial outward normals at azimuth offset `±22.5°`
from `−Z`, giving an `overhang_angle = 56.25°` — above FDM's `45°`
threshold but below the `45° + 30° = 75°` Critical band, so the issues
are classified as `Info` (not Critical or Warning). They do not block
`is_printable()`; the `Critical SelfIntersecting` issue is the sole
driver of `is_printable() == false` on this fixture.

The cylinder splits the lateral surface into 4 distinct regions (one
per quadrant cluster of faces just above the bottom-most). Each
region has 2 faces; total: 4 regions, 8 faces flagged. Centres at
`(0, ±2.7245, 0.9225)` and `(±2.7245, 0, 0.9225)` — the two-axis
symmetry of the +-cross fixture in plain sight.

## Single-cylinder regression — the convex-mesh anchor

After validating the `+`-cross assembly, the example re-runs validation
on cylinder A alone (still horizontal, still `place_on_build_plate`-
translated). A single cylinder is convex, its lateral cannot cross
itself, and mesh-repair's `skip_adjacent` filter rules out the
vertex-shared neighbours. Result: `self_intersecting.len() == 0`. The
lateral-overhang co-flag still fires (the cylinder is still
horizontal), but the load-bearing anchor — no false positives on a
convex single-mesh — is locked.

## Numerical anchors

1. `validation.self_intersecting.len() >= 4` — interpenetrating
   cylinders must produce at least 4 pair flags (one-per-ring lower
   bound; the actual count for this fixture is 100, the cap).
2. `validation.self_intersecting.len() <= 100` — mesh-repair's
   `max_reported` cap. Truncation suffix
   (`"(search truncated; total may be higher)"`) is asserted iff
   `len() == 100`.
3. All entries' `face_a < face_b` (canonical ordering per §6.4).
4. All entries' `approximate_location` is within `±5 mm` component-
   wise of the post-placement origin `(0, 0, 5)`.
5. Every `SelfIntersecting` `PrintIssue` is `Critical`;
   `is_printable() == false`.
6. `validation.overhangs.len() >= 1` (lateral underside co-flag).
7. **Single-cylinder regression**: cylinder A alone produces
   `self_intersecting.len() == 0` (no false positives on a convex
   single-mesh).

## How to run

```text
cargo run -p example-mesh-printability-self-intersecting --release
```

`--release` is recommended (mesh-repair's parallel SAT routine is
heavy in debug). Output is written to
`examples/mesh/printability-self-intersecting/out/`:

- `out/mesh.ply` — 68-vertex, 128-triangle ASCII PLY (the
  un-cleaned-up `+`-cross assembly).
- `out/issues.ply` — vertex-only ASCII PLY of 104 region centroids
  (100 self-intersection `approximate_location` points + 4
  `OverhangRegion.center` points). Open alongside `mesh.ply` to see
  the spatial distribution of flagged regions: 100 dense points
  clustered around the four interpenetration rings near `(0, 0, 5)`,
  plus 4 sparse points marking the lateral-overhang region centres
  at `z ≈ 0.9` mm.

## "Couldn't I just call `mesh_repair::detect_self_intersections`?"

Yes — and `validate_for_printing` does internally. The §6.4 wrapper
exposes self-intersection through the same higher-level API as every
other detector: same `PrintValidation` shape, same severity classifier,
same `is_printable()` gate. You get the typed `SelfIntersectingRegion`
list (with canonical face ordering + midpoint approximate location)
and the `PrintIssue` summary with truncation suffix — without
re-implementing any of that bookkeeping per call site.

For power users who need to tune `IntersectionParams` (a different
`max_reported`, `epsilon`, or `skip_adjacent`), call
`mesh_repair::intersect::detect_self_intersections` directly with the
desired params — `mesh-repair` is a workspace crate; add it as a
direct dep to access `IntersectionParams::exhaustive()`. The §3 spec
calls for re-exporting these types from `mesh-printability` itself for
ergonomics, but the row #16 detector landed without that re-export
(open as a v0.9 candidate). Until the re-export ships, the
default-params path through `validate_for_printing` is the
print-validation default and the direct `mesh-repair` call is the
power-user escape hatch.

## v0.8 fix arc cross-references

- §6.4 — `SelfIntersecting` detector (landed at row #16, commit
  `63838fb4`)
- §7.4 — this example's spec
- §9.2.6 — self-intersection stress fixtures
- `mesh-repair::intersect::detect_self_intersections` — the upstream
  routine the §6.4 wrapper re-uses
