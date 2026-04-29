# mesh-offset-inward

**Erosion via SDF + marching cubes.** Given a closed triangle mesh and
a negative offset distance, `mesh-offset::offset_mesh` produces a new
triangle mesh whose surface is everywhere at perpendicular distance
`|d|` *inward* from the original — an erosion. The algorithm samples
the input's signed-distance field on a 3D grid, subtracts `d` from
every sample (translating the zero-isosurface inward by `|d|` along
the SDF gradient), and extracts the new isosurface via marching cubes.
Companion to [`mesh-offset-outward`](../mesh-offset-outward/) — same
function, opposite sign, distinct geometry.

## What it does

Builds `mesh_types::unit_cube()` (8 verts, 12 triangles, AABB
`[0,1]³`) and offsets it inward by `0.1` mesh units using
`OffsetConfig::default().with_resolution(0.025)`. Saves THREE PLY
artifacts and round-trips all three via `load_ply`:

| File | Description |
|---|---|
| `out/before.ply` | Input unit cube. |
| `out/after.ply` | Raw `offset_mesh` output. Approximates a smaller cube `[0.1, 0.9]³` (the level set is the exact polytope; the MC render shows visible cell-scale chamfering at corners + edges — see [Polytope preservation](#polytope-preservation-under-inward-offset)), inside-out and topologically vertex-soup. **The platform-truth artifact** — anchors what `mesh-offset` actually returns; drift catcher for any future change to MC vertex-sharing or winding convention. |
| `out/after_flipped.ply` | Same geometry as `after` with winding reversed per face. Renders correctly across all viewer pipelines (whereas `after.ply` only renders correctly in viewers without backface culling and with two-sided lighting); **what you'll usually want to look at**. |

Three platform behaviors are anchored — see
[Polytope preservation under inward offset](#polytope-preservation-under-inward-offset),
[The vertex-soup output](#the-vertex-soup-output), and
[Inside-out winding (mesh-offset 0.7.x quirk)](#inside-out-winding-mesh-offset-07x-quirk).
A fourth section,
[The per-face winding flip pattern](#the-per-face-winding-flip-pattern),
documents the inline remediation the example uses to produce the
`after_flipped.ply` artifact.

> **Heads-up for `f3d` users:** `after.ply` and `after_flipped.ply`
> look identical in `f3d` (and any viewer with two-sided lighting
> and no backface culling). The inside-out winding only becomes
> visible in viewers with single-sided rendering OR backface
> culling — MeshLab solid mode, Blender solid mode with cull
> backfaces enabled, glTF/PBR materials with `cullFace = back`.
> This is **not** a bug in the example or the artifacts; it is a
> property of the renderer's lighting model. See
> [Visuals](#visuals) for the full pipeline-by-pipeline breakdown.

## Polytope preservation under inward offset

The headline contrast with [`mesh-offset-outward`](../mesh-offset-outward/):
**inward offset of a convex polytope preserves polytope structure.**

For the unit cube + offset `d`, the level set decomposes very
differently in the two directions:

| Direction | Decomposition | Volume formula |
|---|---|---|
| **Outward** (d > 0) | cube + 6 face slabs + 12 edge quarter-cylinders + 8 corner sphere octants | `V_d = 1 + 6d + 3πd² + (4π/3)d³` (Steiner-Minkowski) |
| **Inward** (d < 0) | smaller scaled cube with sharp 90° corners and edges | `V_d = (1 + 2d)³` (exact polytope) |

Why the asymmetry? At any convex corner of the body (the unit cube has
eight), the *inside* of the corner has body-interior space "behind"
the wedge. The inward level set retreats into that space and stays
sharp — the corner moves inward by `|d|` along each axis but its 90°
geometry is preserved. The *outside* of the corner is empty space "in
front of" the wedge, and the outward level set fills it: the locus of
points at perpendicular distance `d` from a single sharp vertex is the
sphere octant the Steiner formula's `(4π/3)d³` term accounts for.
Convex corners *can't* round under inward offset because there's
nothing to round into; they *must* round under outward offset because
sphere-octant filling is the only consistent way to maintain the
constant-perpendicular-distance condition at a vertex.

**The level set vs. the rendered MC mesh.** The polytope-preservation
claim is about the *level set* — the locus of points at signed
distance `d` from the input. The rendered MC mesh approximates that
level set with planar triangles, and at each of the eight polytope
corners and twelve edges the MC kernel emits cell-scale **chamfers**
(angular planar cuts) rather than reproducing the sharp creases —
each cell can only emit planar facets, so a sharp 90° edge becomes
a stair-step of small planar chamfers, one per cell along the edge.
The result is **chamfered, not filleted**: the surface has flat
angular cuts at the cell scale, not smooth curvature blending. So
at `GRID_RESOLUTION = 0.025` the rendered surface looks visibly
chamfered at corners and edges even though the mathematical level
set IS exactly the polytope. The chamfering shrinks proportionally
with resolution:
`.with_resolution(0.0125)` halves it, with ~4× triangle count. The
volume anchor stays tight at any resolution because volume is
dominated by the polytope interior, not the corner/edge cells —
0.57% in the run on the author's machine versus the 2% asserted
tolerance.

For `d = -0.1` (this example):

```
V = (1 + 2 × (-0.1))³ = 0.8³ = 0.512   exact
A = 6 × (1 + 2 × (-0.1))²  = 6 × 0.64  = 3.84   exact
```

The example's volume anchor uses ±2% tolerance, tighter than
`mesh-offset-outward`'s ±5%. The 5% headroom in the outward case
absorbs the Steiner-vs-MC approximation gap (rounded corners
discretized by a finite grid); inward has NO such gap, only MC
discretization. The level set faces fall exactly on grid lines at
`GRID_RESOLUTION = 0.025` (`0.1 = 4 × 0.025`, `0.9 = 36 × 0.025`,
both reachable from the grid's `-0.225` start by integer multiples of
`0.025`), so MC places the face vertices bit-exactly modulo IEEE-754
add ordering and the only geometric error comes from MC's
triangulation of the eight corner cells.

**Why this matters for design + manufacturing:** inward offset is the
load-bearing operation for **mold-cavity inversion**. Given a part
with sharp design features, the cavity that produces it must mirror
those features sharply. Outward offset is the wrong tool for that job
— it rounds. Inward offset is, for convex polytope subsets, the
lossless geometric inverse.

The asymmetry breaks for non-convex polytopes (concave corners *do*
round under inward offset because the concavity *is* a corner from the
field's perspective), but the unit cube has no concave corners.

## API surface — `OffsetConfig` resolution dial

The crate exposes three named presets and a builder method:

| Construction | `resolution` | Cells per `0.1` offset | Use case |
|---|---|---|---|
| `OffsetConfig::preview()` | 0.5 | 0.2 | Way too coarse for unit-scale geometry; interactive UIs only |
| `OffsetConfig::default()` | 0.1 | 1 | Marginal — corners look chamfered, not sharp |
| `OffsetConfig::high_quality()` | 0.05 | 2 | Visible corner sharpness, slightly faceted faces |
| `.with_resolution(0.025)` | 0.025 | 4 | This example — near-sharp corners with visible cell-scale chamfering, sub-second |
| `.with_resolution(0.0125)` | 0.0125 | 8 | Half the chamfering, roughly 4× more triangles |

The "knee" is around 4 cells per offset distance — past that, returns
diminish quickly. Below 2 cells per offset, the marching-cubes corner
patches become visibly faceted. `with_resolution` is the right knob
when none of the named presets fit the input's scale.

The same `offset_mesh` function with a POSITIVE `distance` performs
dilation (expansion) — see
[`mesh-offset-outward`](../mesh-offset-outward/) for the pair
companion. For the unit cube, the safe range for inward offset is
`0 < |d| < 0.5`; at `|d| ≥ 0.5` the inner level set has zero or
negative volume and `offset_mesh` returns
`MarchingCubesFailed { reason: "no triangles generated" }`. See
architecture book chapter 30 for the full SDF↔mesh bridge story.

## Numerical anchors

**Before (`unit_cube`):**

- `report.vertex_count == 8`, `report.face_count == 12`
- `report.is_watertight && report.is_manifold && !report.is_inside_out`
- `signed_volume == 1.0` exactly
- `aabb.min == (0,0,0)`, `aabb.max == (1,1,1)`

**After (offset surface, geometric anchors):**

- `aabb.min ≈ (0.1, 0.1, 0.1)`, `aabb.max ≈ (0.9, 0.9, 0.9)` —
  the inner level set retreats from each face by exactly `|d|`,
  and the unit cube's faces at `x, y, z ∈ {0, 1}` map directly to
  inner faces at `x, y, z ∈ {0.1, 0.9}`. Tolerance: `½ × cell_size + ε`
  ≈ `0.0125` (kept consistent with `mesh-offset-outward`; in practice
  the inner faces fall exactly on grid lines and the AABB is tighter
  than this — the slack accommodates whatever residual MC artifacts
  appear at the corner cells).
- `|signed_volume|` matches the exact polytope formula within ±2%:
  ```
  V_d = (1 + 2d)³
      = 0.8³
      = 0.512    (for d = -0.1)
  ```
  In the run on the author's machine: `|signed_volume| = 0.5091`,
  0.57% off the analytical value — the level set is geometrically
  very close to exact at this resolution. The 2% asserted tolerance
  leaves ~3.5× headroom for cross-platform FP drift. The asserted
  `abs()` is needed because `signed_volume` itself is NEGATIVE (see
  [Inside-out winding](#inside-out-winding-mesh-offset-07x-quirk) below).
  See [Polytope preservation](#polytope-preservation-under-inward-offset)
  above for why this formula is exact (no Steiner-Minkowski rounded
  contributions) and why the tolerance is tighter than the outward
  example's ±5%.
- Surface area (printed, not asserted) follows `A_d = 6 (1 + 2d)² ≈ 3.84`
  and should be in that neighborhood. In the run on the author's
  machine: `surface_area = 3.6975`, ~3.7% under the analytical value.
  The undershoot is larger than the volume error because each MC
  corner cell triangulates the sharp polytope vertex with planar
  facets that "cut" the very tip of the corner, removing a small
  patch of area without removing a comparable patch of volume.

**After (offset surface, topological anchors):**

- `vertex_count == 3 × face_count` exactly — marching cubes does not
  share vertices across triangles; every triangle gets 3 fresh
  vertex entries. Identical to `mesh-offset-outward` — soup-mesh
  output is sign-agnostic.
- `face_count > 12` — the level set is denser than the input cube;
  exact count is grid-resolution-dependent and intentionally NOT
  anchored.
- `boundary_edge_count == 3 × face_count` — every edge of every
  triangle is unique (no two triangles share any edge by index),
  so every edge is a boundary edge.
- `!report.is_watertight` — direct consequence of the boundary
  count above; this is the discriminating signal that distinguishes
  the soup from a real surface.
- `report.is_manifold == true` — and this is misleading.
  `validate_mesh::is_manifold` only checks "no edge has 3+ incident
  faces"; the soup mesh trivially satisfies that condition because
  every edge has exactly 1 face. Watertightness is the right check.
- `report.is_inside_out == true` — see
  [Inside-out winding](#inside-out-winding-mesh-offset-07x-quirk).

These anchors are drift catchers: if the marching cubes
implementation ever adds an edge-vertex dedup pass, the
`vertex_count == 3 × face_count` and `boundary_edge_count == 3 ×
face_count` equalities break and this example fails loudly so the
topological narrative can be updated.

**After flipped (`out/after_flipped.ply`):**

Identical to `after` in every way except orientation:

- `vertex_count`, `face_count`, and AABB are bit-equal to `after` —
  `flip_winding` is a per-face index swap (`[a, b, c] → [a, c, b]`);
  the vertex array is untouched.
- `signed_volume` is the **exact arithmetic negation** of
  `after.signed_volume()`, asserted within `1e-9` (modulo IEEE-754
  add ordering).
- `report.is_inside_out == false` — the win for visuals.
- All topological flags **unchanged**: still vertex-soup, still
  `!is_watertight`, still trivially `is_manifold`,
  `boundary_edge_count == 3 × face_count` still. The flip is a
  presentation change, not a topological repair.

**PLY round-trip:**

- `out/before.ply` reloads as 8 verts × 12 faces (unit cube preserved).
- `out/after.ply` reloads with vertex/face counts matching the
  in-memory offset mesh (no welding on save, no dedup on load).
- `out/after_flipped.ply` round-trips identically to `after.ply`
  (only face index order differs in the file body).

## The vertex-soup output

`marching_cubes` in `mesh-offset` walks each grid cell independently
and emits triangles by writing 3 fresh vertex entries plus one face
record per triangle (see `mesh-offset/src/marching_cubes.rs::process_cell`
lines 131-137). It does NOT deduplicate vertices that lie on shared
edges between cells. The result: `vertex_count == 3 × face_count`
exactly, every edge appears in exactly one face, and `validate_mesh`
reports `!is_watertight` with `boundary_edge_count == 3 × face_count`.

A subtle point: `validate_mesh::is_manifold` checks "no edge has
3+ incident faces." The soup mesh trivially satisfies that — every
edge has exactly 1 face. So `is_manifold` reports `true` even
though the mesh is not topologically a manifold in the usual sense
(every triangle is a disconnected island). For marching-cubes
output, **`is_watertight` is the right discriminator**, not
`is_manifold`.

The geometry is fine — the surface is closed in space — but the
topology is "soup." Two consequences:

1. Geometric queries that depend only on per-face math (`signed_volume`,
   `surface_area`, AABB) work correctly on the soup mesh.
2. Adjacency-based queries (`MeshAdjacency::build`, `detect_holes`,
   `count_inconsistent_faces`, BFS-based winding propagation) treat
   every triangle as disconnected. They run, but their answers
   describe the soup, not the surface.

To convert the soup mesh into a true manifold mesh, run
`mesh_repair::weld_vertices(&mut mesh, params)` afterward. With a
sensible weld tolerance (e.g., `0.5 × cell_size`), shared edge
vertices fuse, adjacency becomes meaningful, and `validate_mesh`
reports the surface as both manifold AND watertight. See
[`mesh-repair-walkthrough`](../mesh-repair-walkthrough/) for the
welding pattern in context. This example deliberately does NOT
perform the weld step — the un-welded MC output is what
`offset_mesh` returns, and seeing the soup is the lesson.

The soup-mesh behavior is sign-agnostic — it applies identically to
inward and outward offset because both call the same MC kernel.

## Inside-out winding (mesh-offset 0.7.x quirk)

The marching-cubes triangle tables in `mesh-offset` produce
inward-pointing triangles for the iso/inside convention used here
(`val < iso ⇒ inside`, with iso=0 and inside-corresponds-to-negative
SDF after the `sdf - distance` shift). The orientation depends on the
table-driven iso/inside relationship, **not on the sign of the offset
distance** — so this quirk applies identically to inward and outward
offset, and `mesh-offset-inward` exhibits the same behavior as
`mesh-offset-outward`: `signed_volume < 0` and
`report.is_inside_out == true`. The level set is geometrically correct
(matches the exact polytope volume to ~0.6% in this run — see
[Numerical anchors](#numerical-anchors)); only the orientation of
every triangle is flipped relative to the outward-facing convention.

The example anchors this as the current behavior in `verify_after`,
and produces a flipped companion artifact (`out/after_flipped.ply`)
via the per-face flip pattern documented in the next section.
Implications for downstream code:

1. **Don't trust `signed_volume` directly** on raw `mesh-offset`
   output — use `signed_volume.abs()`, or apply the per-face flip
   below.
2. **`mesh_repair::fix_winding_order` won't help on raw MC output** —
   it BFS-traverses face adjacency, and the soup mesh has no
   adjacency to traverse (every triangle is disconnected), so the
   pass is effectively a no-op. The per-face flip is the right tool
   for soup meshes.

This is a candidate cleanup for a future `mesh-offset` revision —
either fix the table convention, or add an explicit winding-flip
post-processing step inside `offset_mesh`. Tracked as a
platform-improvement followup per `feedback_improve_mesh_crate`.

## The per-face winding flip pattern

```rust
fn flip_winding(mesh: &mut IndexedMesh) {
    for face in &mut mesh.faces {
        face.swap(1, 2);   // [a, b, c] → [a, c, b]
    }
}
```

This is the right remediation for the inside-out MC output. Three
properties make it the right choice over `mesh_repair::fix_winding_order`:

- **Adjacency-free.** Operates on each face independently. Works on
  vertex-soup meshes where `fix_winding_order`'s BFS has nothing to
  traverse.
- **Position-preserving.** Doesn't touch `mesh.vertices`, so AABB,
  surface area, and `|signed_volume|` are bit-equal before/after.
  Only `signed_volume`'s sign and `is_inside_out`'s flag flip.
- **Idempotent under double application.** Two flips are a no-op
  (swap → swap returns the original ordering), so it composes
  predictably with other transforms.

The example anchors all three properties in `verify_after_flipped`:
counts equal, `vertices` array byte-identical, AABB bit-equal,
`signed_volume` exactly negated, `is_inside_out` toggled false.

When to reach for `mesh_repair::fix_winding_order` instead: after
welding the soup mesh into a manifold (see
[`mesh-repair-walkthrough`](../mesh-repair-walkthrough/)), the BFS
becomes meaningful — it can detect and fix per-component winding
inconsistencies. For a uniformly-inside-out MC output (every
triangle flipped the same way), the per-face flip is simpler and
preserves the "soup" character of the mesh, which is what
`offset_mesh`'s output convention currently is.

## Visuals

Open the artifacts in MeshLab, ParaView, Blender, or `f3d`:

- **`out/before.ply`** — clean unit cube. Sharp 90° corners and
  edges, six flat quad-pairs, eight corners.
- **`out/after_flipped.ply`** — *the artifact you want for
  visuals*. A SMALLER cube of side `0.8` occupying `[0.1, 0.9]³`.
  The level set is mathematically the exact polytope (see
  [Polytope preservation](#polytope-preservation-under-inward-offset)),
  but the MC kernel's discretization at `GRID_RESOLUTION = 0.025`
  emits cell-scale **chamfers** (angular planar cuts, NOT smooth
  fillets) at the eight corners and twelve edges, so the rendered
  surface looks **noticeably chamfered at corners and edges** — the
  polytope is the math; the render is the polytope with a stair-step
  of small planar chamfers wherever the level set has a sharp
  crease. What you'll see:
  - The eight corners are chamfered at the cell scale (~`0.025`):
    a flat planar cut where the math has a sharp vertex. Much
    smaller cuts than `mesh-offset-outward`'s sphere-octant fillets
    at the offset scale (~`0.1`), and **angular not curved**, but
    obvious next to the input cube.
  - The twelve edges are chamfered at the cell scale: a stair-step
    of small planar chamfers along each edge, one per cell — not a
    smooth fillet, and not a perfect crease.
  - The six faces are flat patches translated inward by `0.1`,
    showing the marching-cubes grid imprint (grid-aligned
    tessellation, not surface-aligned). The grid imprint on the
    faces is identical to `mesh-offset-outward`'s — same MC kernel,
    same artifacts.

  Headline visual contrast vs `mesh-offset-outward`: **inward** is
  chamfered (angular) at the **cell scale** (~`0.025` at this
  resolution), **outward** is filleted (curved) at the **offset
  scale** (~`0.1`). Two axes of difference, not one — both the
  *scale* and the *shape* of the deviation from sharp polytope
  differ: inward chamfering is angular and shrinks proportionally
  with finer resolution; outward filleting is curved and is
  fundamental to the Steiner-Minkowski geometry (sphere octants
  persist at any resolution). For sharper inward visuals, use
  `.with_resolution(0.0125)` — corner chamfers halve to ~`0.0125`
  with roughly 4× triangle count.
- **`out/after.ply`** — *the platform-truth artifact*, geometrically
  identical to `after_flipped.ply` but with inside-out winding.
  Whether the file looks visibly different from `after_flipped.ply`
  depends entirely on your viewer's rendering pipeline:

  - **Two-sided lighting, no backface culling** (`f3d` default):
    **no visible difference.** When a face's normal points away
    from the light, the lighting calculation uses the flipped
    normal so the face appears lit anyway. Winding direction
    becomes invisible because every triangle gets shaded as if
    its normal pointed toward the camera regardless of which
    direction the index ordering implies.
  - **Backface culling enabled** (MeshLab "solid" mode default,
    Blender solid mode with cull-backface on, glTF/PBR materials
    with `cullFace = back`): **visible difference.** The inside-out
    smaller cube renders only its interior-facing triangles, so the
    cube either appears hollow or shows the far side through the
    camera-facing surfaces.
  - **Single-sided lighting** (some Phong/Gouraud configurations
    without two-sided lighting enabled): **visible difference,
    subtler.** Even without backface culling, the inside-out
    cube's exterior surfaces have flipped normals, so they appear
    darker or unlit and the interior surfaces appear bright.

  Quick rule (same as `mesh-offset-outward`): viewers configured for
  single-sided rendering OR backface culling show the difference;
  viewers configured for two-sided lighting without culling (`f3d`'s
  defaults) treat both files identically.

You typically only need `after.ply` if you're verifying that
`offset_mesh`'s raw output convention hasn't changed (the example
already verifies this in `verify_after`; the file is for human
inspection of that fact). For everyday use across mixed-viewer
workflows, **`after_flipped.ply` is the safe artifact** — it
renders the same as `after.ply` in `f3d` and renders correctly
in viewers where `after.ply` would not.

## Run

```
cargo run -p example-mesh-mesh-offset-inward --release
```

Output written to `examples/mesh/mesh-offset-inward/out/`.
