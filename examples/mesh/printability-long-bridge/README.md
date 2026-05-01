# printability-long-bridge

**Visual demo of the §6.2 LongBridge detector (Gap G of the v0.8 fix arc).**
Hand-authors a 24-vertex / 44-triangle H-shape — two pillars joined by
a horizontal lintel — as a single watertight, consistently-wound
boolean-union solid. The lintel's 20 mm middle bridge fires Critical
LongBridge under FDM defaults; the two 5 mm cantilevers cluster
independently and fall below the bridge threshold. All three downward
slab-bottom regions also co-flag Critical ExcessiveOverhang (90° tilt),
and `is_printable()` returns false.

> **Heads-up for `f3d` users — back-face-culling artifact:** the slab's
> bottom is the load-bearing geometry the detector flags, but it's a
> *downward-facing* surface. Viewers that cull back-faces by default
> (or render the model from above) will hide the very face this
> example is about. Open the file in **MeshLab** or **ParaView**
> (which render with two-sided lighting + visible back-faces by
> default, or expose the toggle in the menu) and tilt the camera
> below the slab to see the bridge + cantilever bottoms. This is
> **not** a bug in the example or the artifact — the slab bottom is a
> genuine outward-facing-with-normal-`-z` surface, exactly as the
> printability detector sees it.

## What this fixture is

Three axis-aligned boxes unioned into a single H-shape:

- **Pillar 1**: `[0, 5] × [0, 5] × [0, 18]` mm.
- **Pillar 2**: `[25, 30] × [0, 5] × [0, 18]` mm.
- **Slab (lintel)**: `[-2.5, 32.5] × [0, 5] × [18, 20]` mm.

The pillar tops at `z = 18` are **interior** to the boolean union
(covered by the slab) — they are NOT face surfaces. The slab's bottom
at `z = 18` is exposed in **three edge-disjoint regions** separated
by the pillar-attachment cutouts:

- **Left cantilever**: `x ∈ [-2.5, 0]`, 2.5 × 5 = 12.5 mm². Span 5 mm
  (longer perpendicular axis is `y`, extent 5).
- **Middle bridge**: `x ∈ [5, 25]`, 20 × 5 = 100 mm². Span **20 mm**.
  ← bridge target.
- **Right cantilever**: `x ∈ [30, 32.5]`, 2.5 × 5 = 12.5 mm². Span 5 mm.

24 unique vertices (8 pillar bases + 8 pillar tops + 4 slab outer at
`z = 18` + 4 slab top at `z = 20`); 44 triangles. Watertight +
consistently wound by construction (every undirected edge appears in
exactly two faces; every directed edge is matched by its reverse).

## What this teaches

`LongBridge` detection (`check_long_bridges`, §6.2) flags faces whose
outward normal sits within 30° of `-up`, then clusters them by
edge-adjacency (manifold-edge-share), then projects each cluster onto
the plane perpendicular to `up` and computes the **axis-aligned
bounding box** in that plane. A cluster fires if and only if
`max(extent_e1, extent_e2) > config.max_bridge_span`.

The fixture exercises BOTH detector branches on a single mesh:

- **Emit branch**: middle bridge — perpendicular-plane bbox `(20, 5)`,
  span = 20 mm. 20 > 10 (FDM `max_bridge_span`) → emitted. Severity
  band: `span > max_bridge_span × 1.5 = 15` → **Critical**.
- **Silent-drop branch**: each cantilever — perpendicular-plane bbox
  `(2.5, 5)`, span = 5 mm. 5 ≤ 10 → `emit_long_bridge_component`
  early-returns; no region, no issue. The cantilevers exist as
  candidate clusters for the detector but never surface as user-facing
  bridges.

## Why a single watertight H (not three boxes)

`validate_for_printing` is layered: `check_basic_manifold` runs
**before** `check_long_bridges` and pushes a Critical `NotWatertight`
issue if any edge has incidence count other than 2. Three independent
triangle shells (one per box) would have:

- Two pairs of overlapping faces at each pillar-slab interface (`5 × 5`
  squares at `z = 18` with opposed normals) — non-manifold.
- Coincident y=0 and y=5 walls between pillar fronts/backs and slab
  fronts/backs — depending on how they're triangulated, either
  duplicate edges or t-junctions.

A boolean-union surface dissolves all of this: pillar tops become
interior, the slab bottom becomes a polygon-with-2-rectangular-holes
(triangulated as 3 disjoint rectangles via the pillar-attachment
cutouts), and the slab front/back become 8-gons (the bottom edge of
each is broken at the four `x` values where pillars meet the slab,
but the rest of the H y-faces are continuous).

The H-shape construction in `build_h_shape()` enumerates the 44
triangles directly, with shared vertex indices at every pillar-slab
junction so `check_basic_manifold` sees a single connected, watertight,
consistently-wound mesh.

## The detector pipeline (applied to this fixture)

1. **Pre-check** `requires_supports()`: FDM is true → run; SLS/MJF
   silent-skip (no `DetectorSkipped` issue per §6.2 line 996).
2. **Per-face filter**: `acos(N · -up) < 30°` — the slab's 6 bottom
   triangles (3 regions × 2 tris each) have `N = (0, 0, -1)`,
   `N · -up = 1`, `acos(1) = 0 < 30°`. All 6 are candidates. The pillar
   bases (also `N = (0, 0, -1)`) get rejected by the build-plate filter
   (their `min(face.z) = 0 = mesh-min.z`, within `EPS_GEOMETRIC`). All
   other faces (sides + slab top) have outward normals not within 30°
   of `-up` and are rejected at the angle filter.
3. **Cluster** by manifold-edge adjacency — the three slab-bottom
   rectangles share no edges with each other (their adjacent edges all
   sit at the pillar-attachment border, which is interior to the
   union).
4. **Per-cluster span** — left/right cantilever `bbox = (2.5, 5)`,
   middle bridge `bbox = (20, 5)`. Apply the threshold gate; only the
   middle bridge survives.
5. **Emit** `LongBridgeRegion(start = (5, 2.5, 18), end = (25, 2.5, 18),
   span = 20)` and a Critical `LongBridge` `PrintIssue` with location
   at the midpoint `(15, 2.5, 18)`.

## Numerical anchors (asserted in `main`)

| Anchor | Expected | Tolerance |
|---|---|---|
| `long_bridges.len()` | 1 | exact |
| Middle bridge `span` | 20.0 mm | 1e-6 |
| Middle bridge `start.x` / `end.x` | 5.0 / 25.0 | 1e-6 |
| Middle bridge `start.y` / `end.y` | 2.5 / 2.5 | 1e-6 |
| Middle bridge `start.z` / `end.z` | 18.0 / 18.0 | 1e-6 |
| Critical LongBridge issues | 1 | exact |
| `overhangs.len()` (Gap-D split) | 3 | exact |
| Critical ExcessiveOverhang issues | 3 | exact |
| Overhang centroids (sorted by x) | -1.25 / 15.0 / 31.25 | 1e-9 |
| Overhang centroid `y` | 2.5 | 1e-9 |
| Overhang centroid `z` | 18.0 | 1e-9 |
| LongBridge midpoint = middle-overhang centroid | (15, 2.5, 18) | 1e-9 |
| `TrappedVolume` issues (issue-filter) | 0 | exact |
| `is_printable()` | `false` | exact |
| `DetectorSkipped` issues (FDM) | 0 | exact |
| SLS `long_bridges.len()` | 0 | exact |
| SLS `overhangs.len()` | 0 | exact |
| SLS `DetectorSkipped` issues | 0 | exact |

The middle-bridge centroid `(15, 2.5, 18)` is computed two
independent ways: as the LongBridge `(start + end) / 2` midpoint, and
as the unweighted mean of the middle-bridge OverhangRegion's per-face
centroids. Both arithmetic paths land at exact-representable f64 — the
`1e-9` tolerance is for IEEE-754 add ordering on the running cluster
sum.

The TrappedVolume anchor uses an `issues.iter().filter(…)` rather than
`validation.trapped_volumes.len()` — the latter field doesn't exist
on `PrintValidation` until row #14 of the v0.8 fix arc ships the §6.3
detector. The two checks are semantically equivalent (the H-shape has
no sealed cavity, so the count is 0 today AND after row #14 lands).
No row #14b backfill is needed for this example.

## What you'll see — `out/mesh.ply`

The H-shape: 24 vertices, 44 triangles, ASCII PLY. Best camera angles:

- **Side-on (looking along `-y`)**: the H profile reads cleanly — two
  vertical pillars + horizontal lintel + 2.5 mm cantilevers past each
  outer pillar edge.
- **Slightly below + side-on**: the slab bottom comes into view; the
  20 mm middle bridge sits between the two 5×5 pillar attachments,
  with the cantilevers visible as much smaller bottom regions to the
  outside.

The slab's downward face IS the artifact — viewers that hide
back-faces by default will need a clipping plane or a tilted-up camera
to expose it (see f3d callout above).

## What you'll see — `out/issues.ply`

A vertex-only PLY with **4 points** (zero faces). The 4 centroids:

- `(15, 2.5, 18)` — **middle bridge** LongBridge midpoint.
- `(-1.25, 2.5, 18)` — **left cantilever** OverhangRegion centroid.
- `(15, 2.5, 18)` — **middle bridge** OverhangRegion centroid (this
  point coincides with the LongBridge midpoint above; that's the
  load-bearing co-flag this example demonstrates).
- `(31.25, 2.5, 18)` — **right cantilever** OverhangRegion centroid.

MeshLab's "Show Vertices" with a large point size makes all 4
positions obvious; ParaView users can apply a `Glyph` filter with a
sphere source to render each centroid as a marker. The two coincident
points at `(15, 2.5, 18)` will look like a single point at default
zoom; zoom in to confirm both LongBridge midpoint and middle-bridge
OverhangRegion are present as distinct PLY vertex records.

## Known co-flags

The fixture flags **four** Critical issues, not just one:

1. Middle bridge **LongBridge** Critical (the load-bearing pedagogical
   concept — span 20 mm > 10 mm × 1.5 = 15 mm Critical band).
2. Left cantilever **ExcessiveOverhang** Critical (90° tilt > 75°).
3. Middle bridge **ExcessiveOverhang** Critical — same cluster footprint
   as the LongBridge above; the two detectors see the same surface
   geometry and emit complementary signals (LongBridge cares about
   span, ExcessiveOverhang cares about tilt).
4. Right cantilever **ExcessiveOverhang** Critical.

Any one of these independently drives `is_printable() == false`. The
example demonstrates LongBridge as the load-bearing concept and
documents the three ExcessiveOverhang co-flags, mirroring the §7.1
ThinWall example's "ThinWall + cavity-ceiling Overhang" co-flag pattern.

## SLS tech-skip cross-check

The same H-shape revalidated under `PrinterConfig::sls_default()`
produces **zero** flagged regions. SLS has `requires_supports() ==
false`, so BOTH `check_overhangs` (`validation.rs:270`) AND
`check_long_bridges` (`validation.rs:1303`) early-return at the start
of the function without classifying any face:

- `long_bridges.len() == 0` — `check_long_bridges` silent-skip. **No
  `DetectorSkipped` issue** is pushed (per §6.2 line 996; that variant
  is reserved for precondition skips like `ThinWall` on an open mesh,
  not for technology-policy skips where the detector is simply
  inapplicable).
- `overhangs.len() == 0` — `check_overhangs` silent-skip on the same
  gate.

As a backup invariant, even if the detectors DID run on SLS the
predicates would not fire on this fixture: SLS's `max_overhang_angle
= 90°` is not strictly less than the slab's 90° face tilt
(`overhang_angle > 90°` is false for `overhang_angle = 90°`), and
`max_bridge_span = ∞` rejects any finite span. So the result is
robust to a future change in the technology gate's mechanism.

This is the §7.2 spec assertion #8 cross-check: **the same physical
geometry, validated at a different printer technology, produces a
clean print verdict** because powder-bed support semantics differ from
extrusion semantics.

## Run

```text
cargo run -p example-mesh-printability-long-bridge --release
```

Output written to `examples/mesh/printability-long-bridge/out/`.

## See also

- [`mesh-printability` `[Unreleased]/Added`](../../../mesh/mesh-printability/CHANGELOG.md)
  — the §6.2 LongBridge detector entry that this example demonstrates,
  plus the v0.8 limitations (cantilever-as-bridge + diagonal-underflag)
  that this fixture's two cantilevers preview.
- [`printability-thin-wall`](../printability-thin-wall/) — sibling
  example for the §6.1 ThinWall detector with a similar two-pass
  numbers + visuals workflow and an analogous co-flag (cavity ceiling
  ExcessiveOverhang).
