# printability-thin-wall

**Visual demo of the §6.1 ThinWall detector (Gap C of the v0.8 fix arc).**
Hand-authors a 24-triangle, double-walled hollow box whose top wall has
been deliberately thinned to 0.4 mm. The detector flags two clusters
(outer top + inner top) as Critical ThinWall, the cavity ceiling
co-flags as a Critical Overhang, and `is_printable()` returns false.

> **Heads-up for `f3d` users — winding-pair artifact:** the inner
> cavity surface and the outer surface are wound in **opposite
> directions** (the inner shell's normals point INTO the cavity,
> away from the surrounding solid; the outer shell's normals point
> outward). f3d's default two-sided lighting renders both shells as
> if their normals always faced the camera, so the file looks like
> a regular box. To *see* both shells with their distinct
> orientations, open the file in **MeshLab** or **ParaView** (which
> render with single-sided lighting + visible backfaces by default,
> or expose the toggle in the menu) and enable a clipping plane to
> peek inside. This is **not** a bug in the example or the artifact
> — it is the load-bearing topology the detector relies on. See
> [Why two disjoint shells](#why-two-disjoint-shells-with-reversed-inner-winding)
> below.

## What this fixture is

A box `[0, 30] × [0, 20] × [0, 15]` mm with an internal cavity
`[1.5, 28.5] × [1.5, 18.5] × [1.5, 14.6]` mm — side and bottom walls
1.5 mm thick, **top wall thinned to 0.4 mm**. Two vertex-disjoint
triangle shells (8 outer corners + 8 inner corners; 12 outer triangles
+ 12 inner triangles = 24 total). Watertight + consistently wound by
construction.

## What this teaches

ThinWall detection requires **watertight + consistently-wound** input
(§6.1). The inward ray-cast from each face's centroid needs a real
opposite face to hit; on an open mesh, no opposite face exists and the
detector emits `DetectorSkipped` Info instead of running. The
double-walled construction is what actually exercises the detector.

The 0.4 mm wall is a roughly half-millimetre thin face — well below
the 1.0 mm FDM `min_wall_thickness` default and below the
`min_wall_thickness / 2 = 0.5 mm` Critical-severity threshold. So both
flagged clusters surface as **Critical** ThinWall issues, not warnings.

A simpler "single thin slab" fixture would be open (5-of-6 face), and
ThinWall would silently skip — defeating the example's purpose.

## Why two disjoint shells (with reversed inner winding)

The cluster contract (§6.1) groups flagged faces by **edge-adjacency**
via `build_edge_to_faces` — two faces are in the same cluster iff they
share a manifold edge. The two shells share **no vertices**, so they
share no edges; the outer top face and the inner top face land in
**separate clusters** even though they sit at nearly the same z. That
two-cluster outcome is the load-bearing topological prediction the
example anchors.

The inner shell's winding is **reversed** relative to a standalone
outward-wound box because each face's normal must point AWAY from the
solid material it's bounding:

- Outer cube faces: normals point OUTWARD (away from the cube's
  interior solid).
- Inner cavity faces: normals point INTO the cavity (away from the
  surrounding solid material between the cavity and the outer shell).

Both shells are independently watertight, and the union is too — every
edge appears in exactly two faces (always intra-outer or intra-inner;
never cross-shell). The two shells are topologically two disconnected
components in the same `IndexedMesh`; `validate_for_printing`'s
manifold + winding-consistency checks treat them correctly.

## Numerical anchors (asserted in `main`)

| Anchor | Expected | Tolerance |
|---|---|---|
| `thin_walls.len()` | 2 | exact |
| Outer cluster centroid | `(15, 10, 15)` | 1e-9 |
| Outer cluster area | 600 mm² | 1e-9 |
| Inner cluster centroid | `(15, 10, 14.6)` | 1e-9 |
| Inner cluster area | 459 mm² | 1e-9 |
| Both clusters' `thickness` | 0.4 mm | 1e-5 |
| Critical ThinWall issues | 2 | exact |
| `is_printable()` | `false` | exact |
| Overhang regions (cavity ceiling) | ≥ 1 | exact |
| `DetectorSkipped` issues | 0 | exact |

Cluster centroids are the **unweighted mean of per-face centroids**
(§6.1, `emit_thin_wall_component`): each cluster has 2 triangles whose
centroids average to `(15, 10, z)`. Areas are exact for axis-aligned
right triangles (no chord error). Reported `thickness` is
`min_dist + EPS_RAY_OFFSET`, the geometric wall thickness corrected
for the ray's 1 µm starting offset; the 1e-5 tolerance leaves an order
of magnitude of headroom for cross-platform IEEE-754 add ordering.

## What you'll see — `out/mesh.ply`

The hollow box, 16 vertices and 24 triangles. Open in MeshLab or
ParaView with a clipping plane (or in any viewer with single-sided
rendering) and you can see the cavity through the thin top wall — the
top wall reads as a much thinner lip than the 1.5 mm side and bottom
walls. The "side cross-section" view (looking along ±y at the x-z
plane) is the cleanest visual for confirming the wall-thickness
distinction.

The two shells share no vertices — useful for confirming the
disjoint-shell construction in a vertex inspector if anything looks
wrong.

## What you'll see — `out/issues.ply`

A vertex-only PLY (4 points, 0 faces) marking the centroids of every
populated region the detector emitted:

- `(15, 10, 15)` — **outer top** ThinWall cluster (z = 15.0)
- `(15, 10, 14.6)` — **inner top** ThinWall cluster (z = 14.6)
- `(15, 10, 14.6)` — **cavity ceiling** Overhang region (overlaps the
  inner ThinWall cluster centroid by construction; the cavity ceiling
  IS the inner-top face)
- `(15, 10, 14.6)` — **support region** for the cavity ceiling
  (`check_overhangs` emits one `SupportRegion` per `OverhangRegion`)

MeshLab's "Show Vertices" rendering with a large point size makes the
cluster geometry obvious; ParaView users can apply a `Glyph` filter
with a sphere source to render each centroid as a marker.

A future commit (row #14 of the v0.8 fix arc, with the centroid
assertion landing at row #14b) will add a TrappedVolume detector and
the sealed cavity at `(15, 10, 8.05)` will appear as an additional
centroid in this file.

## Known co-flags + deferred assertions

The fixture flags **three** Critical issues, not just one:

1. Outer top **ThinWall** Critical (the load-bearing pedagogical
   concept).
2. Inner top **ThinWall** Critical (the load-bearing pedagogical
   concept, second cluster).
3. Cavity ceiling **ExcessiveOverhang** Critical — the inner top
   face's normal points DOWN into the cavity (overhang angle = 90°,
   well past the Critical band threshold 45° + 30° = 75°). This is a
   structural-detector co-flag the user learns to read alongside the
   ThinWall cluster: a ceiling over an enclosed cavity is, mechanically,
   a 90° bridge with no support material option.

Either of these issues independently drives `is_printable() == false`;
the example demonstrates ThinWall as the load-bearing concept while
documenting the Overhang co-flag.

**Three TrappedVolume assertions deferred to row #14b**:
§7.1 of the v0.8 fix arc spec lists three additional anchors —
`trapped_volumes.len() == 1`, volume ≈ 6012.9 mm³, centroid
`(15, 10, 8.05)`. The TrappedVolume detector first ships at row #14
of the arc; row #14b (a tiny follow-up commit immediately after)
backfills these three assertions into this example's `verify`. See
§12.1 of the v0.8 fix arc spec for the row #11 / row #14b split
rationale (kept off the high-tier row #14 detector commit so that
focus stays on detector + cross-os CI + voxel-grid memory cap; loop
closes by name in `git log`). The cavity is sealed by construction
(the bottom wall is 1.5 mm of solid below z = 1.5), so when the
detector ships it WILL fire.

## Run

```text
cargo run -p example-mesh-printability-thin-wall --release
```

Output written to `examples/mesh/printability-thin-wall/out/`.

## See also

- [`mesh-printability` `[Unreleased]/Added`](../../../mesh/mesh-printability/CHANGELOG.md)
  — the §6.1 ThinWall detector entry that this example demonstrates.
- [`mesh-offset-inward`](../mesh-offset-inward/) — for the analogous
  "platform truth + viewer-friendly" winding-pair narrative on a
  different artifact.
