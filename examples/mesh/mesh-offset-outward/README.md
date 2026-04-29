# mesh-offset-outward

**Expansion via SDF + marching cubes.** Given a closed triangle mesh
and a positive offset distance, `mesh-offset::offset_mesh` produces a
new triangle mesh whose surface is everywhere at perpendicular
distance `d` from the original — a dilation. The algorithm samples the
input's signed-distance field on a 3D grid, subtracts `d` from every
sample (translating the zero-isosurface outward by `d` along the SDF
gradient), and extracts the new isosurface via marching cubes.

## What it does

Builds `mesh_types::unit_cube()` (8 verts, 12 triangles, AABB
`[0,1]³`) and offsets it outward by `0.1` mesh units using
`OffsetConfig::default().with_resolution(0.025)`. Saves THREE PLY
artifacts and round-trips all three via `load_ply`:

| File | Description |
|---|---|
| `out/before.ply` | Input unit cube. |
| `out/after.ply` | Raw `offset_mesh` output. Geometrically a closed rounded cube, but inside-out and topologically vertex-soup. **The platform-truth artifact** — anchors what `mesh-offset` actually returns; drift catcher for any future change to MC vertex-sharing or winding convention. |
| `out/after_flipped.ply` | Same geometry as `after` with winding reversed per face. Renders correctly across all viewer pipelines (whereas `after.ply` only renders correctly in viewers without backface culling and with two-sided lighting); **what you'll usually want to look at**. |

Two platform behaviors are anchored — see
[The vertex-soup output](#the-vertex-soup-output) and
[Inside-out winding (mesh-offset 0.7.x quirk)](#inside-out-winding-mesh-offset-07x-quirk).
A third section,
[The per-face winding flip pattern](#the-per-face-winding-flip-pattern),
documents the inline remediation the example uses to produce the
`after_flipped.ply` artifact.

## API surface — `OffsetConfig` resolution dial

The crate exposes three named presets and a builder method:

| Construction | `resolution` | Cells per `0.1` offset | Use case |
|---|---|---|---|
| `OffsetConfig::preview()` | 0.5 | 0.2 | Way too coarse for unit-scale geometry; interactive UIs only |
| `OffsetConfig::default()` | 0.1 | 1 | Marginal — corners look chamfered, not rounded |
| `OffsetConfig::high_quality()` | 0.05 | 2 | Visible corner rounding, slightly faceted |
| `.with_resolution(0.025)` | 0.025 | 4 | This example — crisp rounding, sub-second |
| `.with_resolution(0.0125)` | 0.0125 | 8 | Smooth corners, roughly 4× more triangles |

The "knee" is around 4 cells per offset distance — past that, returns
diminish quickly. Below 2 cells per offset, the marching-cubes corner
patches become visibly faceted. `with_resolution` is the right knob
when none of the named presets fit the input's scale.

The same `offset_mesh` function with a NEGATIVE `distance` performs
erosion (cavity prep). See architecture book chapter 30 for the full
SDF↔mesh bridge story.

## Numerical anchors

**Before (`unit_cube`):**

- `report.vertex_count == 8`, `report.face_count == 12`
- `report.is_watertight && report.is_manifold && !report.is_inside_out`
- `signed_volume == 1.0` exactly
- `aabb.min == (0,0,0)`, `aabb.max == (1,1,1)`

**After (offset surface, geometric anchors):**

- `aabb.min ≈ (-0.1, -0.1, -0.1)`, `aabb.max ≈ (1.1, 1.1, 1.1)` —
  the offset surface includes the corner sphere octants of radius
  `d` around each cube corner, extending exactly `d` past the cube
  on each axis. Tolerance: `½ × cell_size + ε ≈ 0.0125` (the level
  set is bilinearly interpolated between SDF samples and can land
  anywhere within half a grid cell of the analytical position).
- `|signed_volume|` matches the Steiner-Minkowski formula for a unit
  cube within ±5%:
  ```
  V_d = 1 + 6d + 3πd² + (4π/3)d³
      = 1 + 0.600 + 0.0942 + 0.00419
      ≈ 1.6984     (for d = 0.1)
  ```
  - `1` — original cube
  - `6d` — six face slabs of thickness d
  - `3πd²` — twelve quarter-cylinders along the cube edges
  - `(4π/3)d³` — eight corner sphere octants summing to a full sphere

  In the run on the author's machine: `|signed_volume| = 1.6971`,
  0.08% off the analytical value — the level set is geometrically
  almost exact at this resolution. The asserted `abs()` is needed
  because `signed_volume` itself is NEGATIVE (see
  [Inside-out winding](#inside-out-winding-mesh-offset-07x-quirk) below).
- Surface area (printed, not asserted) follows the analogous formula
  `A_d = 6 + 6πd + 4πd² ≈ 8.01` and should be in that neighborhood.

**After (offset surface, topological anchors):**

- `vertex_count == 3 × face_count` exactly — marching cubes does not
  share vertices across triangles; every triangle gets 3 fresh
  vertex entries.
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
  every edge has exactly 1 face. The flag is a poor discriminator
  for marching-cubes output. Watertightness is the right check.
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
  `after.signed_volume()`: in this run `+1.6971 ≈ -(-1.6971)`,
  asserted within `1e-9` (modulo IEEE-754 add ordering).
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

## Inside-out winding (mesh-offset 0.7.x quirk)

The marching-cubes triangle tables in `mesh-offset` produce
inward-pointing triangles for the iso/inside convention used here
(`val < iso ⇒ inside`, with iso=0 and inside-corresponds-to-negative
SDF). The result: `signed_volume < 0` and `report.is_inside_out ==
true`. The level set is geometrically correct (matches the
Steiner-Minkowski volume to <0.1% in this run); only the orientation
of every triangle is flipped relative to the outward-facing
convention.

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
  visuals*. Visibly larger than the cube, with all edges and
  corners rounded:
  - The eight corners are sphere octants of radius ≈ 0.1.
  - The twelve edges are quarter-cylinders of radius ≈ 0.1
    sweeping along the cube edges.
  - The six faces are flat patches translated outward by 0.1.

  In flat-shaded views, look for: smooth shading bands at the
  corners (the spherical patches); smooth shading bands along the
  edges (the cylindrical patches); uniform flat shading on the six
  face panels. If your viewer renders triangle edges, you'll see
  the marching-cubes grid imprint on the curved patches — the patch
  tessellation is grid-aligned, not surface-aligned. This is a
  marching-cubes signature, not an offset-operation artifact.
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
    cube renders only its interior-facing triangles, so the
    rounded cube either appears hollow or shows the far side of
    the cube through the camera-facing surfaces.
  - **Single-sided lighting** (some Phong/Gouraud configurations
    without two-sided lighting enabled): **visible difference,
    subtler.** Even without backface culling, the inside-out
    cube's exterior surfaces have flipped normals, so they appear
    darker or unlit and the interior surfaces appear bright.

  Quick rule: viewers configured for single-sided rendering OR
  backface culling show the difference; viewers configured for
  two-sided lighting without culling (`f3d`'s defaults) treat
  both files identically.

You typically only need `after.ply` if you're verifying that
`offset_mesh`'s raw output convention hasn't changed (the example
already verifies this in `verify_after`; the file is for human
inspection of that fact). For everyday use across mixed-viewer
workflows, **`after_flipped.ply` is the safe artifact** — it
renders the same as `after.ply` in `f3d` and renders correctly
in viewers where `after.ply` would not.

## Run

```
cargo run -p example-mesh-mesh-offset-outward --release
```

Output written to `examples/mesh/mesh-offset-outward/out/`.
