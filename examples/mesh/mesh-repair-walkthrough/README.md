# mesh-repair-walkthrough

**The repair pipeline pattern from broken to clean.** The `mesh-repair`
crate exposes operations to fix six classes of common mesh defects.
This example deliberately constructs a unit cube with all six defects,
runs the repair pipeline in three explicit stages, and verifies the
result is watertight, manifold, and outward-wound.

`repair_mesh(&RepairParams)` is the **basic** pipeline — it runs four
of the six fix operations: degenerate-triangle removal, vertex welding,
duplicate-face removal, and unreferenced-vertex cleanup. The other
two — `fill_holes` and `fix_winding_order` — are separately-exposed
APIs because they're heavier (boundary trace + ear-clipping
triangulation; BFS flood-fill across components) and not always
desired (some workflows preserve boundaries; some preserve mixed
winding for two-sided rendering). This example calls all three to span
the full repair surface.

## What it does

Builds `mesh_types::unit_cube()` (8 verts, 12 triangles) and corrupts
it with six defects, one per repair operation:

1. **Duplicate vertex** — appends a position-equal copy of vertex 0
   (new index `8`), then re-routes `face[1] = [0, 3, 2]` to reference
   the duplicate instead. `weld_vertices` will fuse `8 → 0` and remap
   the face back.
2. **Degenerate triangle** — appends a vertex at the midpoint of edge
   `v0→v1` and a face `[0, mid, 1]`. The three vertices are collinear,
   so the triangle has zero area. `remove_degenerate_triangles_enhanced`
   strips it.
3. **Duplicate face** — appends an exact copy of original `face[0] =
   [0, 2, 1]`. `remove_duplicate_faces` strips the second occurrence
   (the appended copy, since duplicate-face detection normalizes
   vertex order and marks the higher-index occurrence).
4. **Unreferenced vertex** — appends a vertex at `(99, 99, 99)` that
   no face references. `remove_unreferenced_vertices` cleans it up.
5. **Reversed winding** — swaps indices 1 and 2 of `face[5] = [0, 5, 4]`
   so it points inward. `fix_winding_order` BFS from `face[0]`
   propagates outward winding across the connected mesh and flips it
   back.
6. **Small hole** — deletes `face[11] = [1, 6, 5]` (the right-face
   top triangle), exposing a 3-edge boundary loop with vertices
   `{1, 5, 6}`. `fill_holes` traces the loop and triangulates it via
   ear clipping.

Snapshots the broken mesh as `out/before.ply`, runs the pipeline,
snapshots the repaired mesh as `out/after.ply`, and reloads both via
`load_ply` to verify round-trip integrity.

## The pipeline diagram

```
broken mesh             repair_mesh             fill_holes           fix_winding_order
(11v 13f)               (basic pipeline)        (close hole)         (consistent winding)
  ├ dup vert ─────────► welded                    │                    │
  ├ degen tri ────────► removed                   │                    │
  ├ dup face ─────────► removed                   │                    │
  ├ unref vert ──────► removed                    │                    │
  ├ reversed face ────────────────────────────────┼─────────────────► flipped
  └ hole (3 edges) ─────────────────────────────► filled               │
                                                                       │
                                                  ┌────────────────────┘
                                                  ▼
                                              clean mesh (8v 12f, watertight, outward)
```

The pipeline order matters. **`fix_winding_order` runs LAST** so its
BFS reaches all faces through the now-closed surface — including the
fill triangle, whose ear-clipping winding is determined by the hole's
boundary normal and may or may not match the rest of the mesh. One
final winding pass fixes both the original reversed face AND any
fill-tri inconsistency.

## Numerical anchors

**Pre-repair (broken mesh):**

- `report.vertex_count == 11`, `report.face_count == 13`
- `report.boundary_edge_count >= 3` (the 3 edges of the deleted-face
  hole, plus more from defect side-effects — see "Defect side-effects
  on adjacency" below)
- `report.degenerate_face_count >= 1`, `report.duplicate_face_count >= 1`
- `!report.is_watertight`, `!report.is_manifold`

The pre-repair `non_manifold_edge_count`, `count_inconsistent_faces`,
and `detect_holes(...)` values are PRINTED but NOT asserted: the
duplicate face and degenerate triangle share edges with cube
neighbors, so multiple edges have 3+ incident faces (non-manifold).
The BFS-based winding check and the boundary-loop tracer both behave
unpredictably on non-manifold adjacency, and `hashbrown`'s iteration
order varies across runs. These properties become predictable after
`repair_mesh` cleans the adjacency.

### Defect side-effects on adjacency

Several defects introduce extra boundary or non-manifold edges that
aren't part of the "intended" defect class. This is realistic — repair
is rarely as surgical as a defect inventory suggests — and `repair_mesh`
sweeps them all away.

- The re-routed `face[1] = [dup_idx, 3, 2]` introduces edges
  `(dup_idx, 3)` and `(dup_idx, 2)`, neither shared with any other
  face → both boundary. It also leaves edge `(0, 3)` with one fewer
  incident face → boundary.
- The degenerate triangle `[0, mid_idx, 1]` introduces edges
  `(0, mid_idx)` and `(mid_idx, 1)`, both unique → both boundary.
  Its edge `(0, 1)` is shared with two existing cube faces, taking
  that edge to 3 incident faces → non-manifold.
- The duplicate face `[0, 2, 1]` shares all 3 edges with original
  `face[0]`. Two of those edges (e.g., `(2, 1)` and `(0, 1)`) are
  also in other cube faces, taking them to 3+ incident faces →
  non-manifold.

In the run on the author's machine: 8 boundary edges and 2
non-manifold edges pre-repair, dropping to 3 boundary edges and 0
non-manifold edges after `repair_mesh`. The exact pre-repair counts
may vary across machines/runs (hashbrown ordering); the post-repair
counts are deterministic and asserted by the example.

**After `repair_mesh(&RepairParams::default())`:**

- `summary.vertices_welded == 1` (vert 8 → vert 0)
- `summary.degenerates_removed == 1` (the collinear triangle)
- `summary.duplicates_removed == 1` (the appended face[0] copy)
- `summary.unreferenced_removed == 3` — see the cascade explanation
  below
- `summary.final_vertices == 8`, `summary.final_faces == 11`
- `report.boundary_edge_count == 3` (just the hole now; defect-induced
  boundaries are gone)
- `report.non_manifold_edge_count == 0`, `is_manifold == true`
- `count_inconsistent_faces(&mesh) == 1` (the hand-reversed face[5],
  now in clean adjacency where the BFS check is deterministic)

**After `fill_holes(&mut mesh, max_edges=10)`:**

- Returns `Ok(1)` (one hole filled)
- `report.face_count == 12` (11 + 1 fill triangle)
- `report.boundary_edge_count == 0`, `is_watertight == true`
- `count_inconsistent_faces(&mesh)` may be 1 or 2 (the original
  reversed face plus possibly the fill triangle; ear-clipping winding
  is determined by the hole's boundary normal and may not match the
  surrounding faces). Not asserted — the next stage handles it.

**After `fix_winding_order` (final state):**

- `report.vertex_count == 8`, `report.face_count == 12`
- `report.boundary_edge_count == 0`
- `report.is_watertight && report.is_manifold && !report.is_inside_out`
- `report.degenerate_face_count == 0 && report.duplicate_face_count == 0`
- `count_inconsistent_faces(&mesh) == 0`

**PLY round-trip:**

- `out/before.ply` reloads as 11 verts × 13 faces (matches in-memory
  pre-repair state)
- `out/after.ply` reloads as 8 verts × 12 faces and re-validates as
  watertight + manifold

### Why `unreferenced_removed == 3`, not 1

The example hand-strands one vertex (`vert 9 = (99, 99, 99)`), but
`remove_unreferenced_vertices` runs **last** in `repair_mesh` and
sweeps everything orphaned by **all** prior stages:

- `vert 8` is referenced pre-repair (by the re-routed `face[1]`),
  but `weld_vertices` remaps that reference to vert 0, leaving vert 8
  with no remaining users.
- `vert 10` is referenced pre-repair (by the degenerate triangle),
  but `remove_degenerate_triangles_enhanced` strips the only face
  that uses it.
- `vert 9` was never referenced.

After all four stages: 3 orphaned vertices, all cleaned up in the
final pass. This is the cascade pattern — earlier repair stages create
later cleanup work, which is why running `remove_unreferenced_vertices`
last is correct. The number `3` is asserted exactly so this property
becomes a drift-catching anchor: if a future change to `repair_mesh`
re-orders stages or stops orphaning vertices, the example will fail
loudly rather than silently diverging from the documented behavior.

## Visuals

Open both PLY files in MeshLab, ParaView, or Blender:

- **`out/before.ply`** — the broken cube. Look for: a triangular gap
  in the upper-right corner (the deleted face's hole); one face that
  flips its outward normal (the reversed face on the front-right edge,
  visible as a darkened patch in flat-shaded views since its normal
  points inward); possibly a sliver near the bottom-front edge (the
  degenerate collinear triangle, which most viewers render as a thin
  line rather than a surface); a stray distant vertex at (99, 99, 99)
  that may shift the camera framing significantly.
- **`out/after.ply`** — the clean closed cube. Uniform outward
  shading, no gaps, no slivers, no stray vertices. Geometrically
  identical to the original `unit_cube` (modulo where the fill-tri
  ear-clipping placed the seam — the cube has 12 triangles either
  way, but the diagonal in the right face's reconstructed triangulation
  may differ from the original).

## Run

```
cargo run -p example-mesh-mesh-repair-walkthrough --release
```

Output written to `examples/mesh/mesh-repair-walkthrough/out/`.
