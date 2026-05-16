# cf-device-design SDF-Based Layer Surfaces — Bookmark

**Status**: OPEN. Recon session is NEXT. Implementation arc is the session
after.

## Three-session pattern in flight

1. **Bookmark session** (2026-05-16, this one). Centerline arc shipped on
   `dev` (PR #248), 6 commits. Architectural decision: cf-device-design's
   per-layer surfaces should be UNIFORM-OFFSET ISOSURFACES of the scan
   SDF, replacing the current per-vertex displacement approach.
2. **Recon session** (NEXT). Read cf-cast-cli's SDF + marching-cubes
   pattern, spec the cf-device-design integration, measure performance
   budget, decide caching strategy. **NO IMPLEMENTATION.**
3. **Implementation session** (AFTER). Build against the recon's chosen
   approach. Estimated 4-8 commits.

## TL;DR — why the per-vertex approach is wrong

cf-device-design currently builds each layer's outer surface via per-vertex
displacement:

```
vertex_layer_i = vertex_proxy + radial_dir(vertex_proxy) × offset_i
where radial_dir(v) = (v − nearest_centerline_pt(v)).normalize()
```

This is a centerline-driven RADIAL displacement, not a SURFACE-NORMAL
OFFSET. The result has visible artifacts:

- **Layer 1 nipple on iter-1 dome apex** (user-verified 2026-05-16). Apex
  vertices' `nearest_centerline_pt` is `polyline[0]` (~2 mm below the
  apex inside the body), so the radial points UP-and-slightly-OUT from
  there. At Layer 1's cumulative 15.3 mm displacement, apex vertices race
  past their neighbors → pointy nipple. Layer 0 (8 mm displacement) has
  the same mechanism but the elongation is small enough that the dome
  stays rounded.
- **Lateral lumps** on prior iterations when the centerline polyline was
  off-axis. The centerline arc fixed the axis problem but the per-vertex
  approach's structural bias remains.

The PROPER fix: build the cleaned scan's SDF; for each layer with
cumulative thickness `T`, extract the isosurface at `SDF = −T` (or `+T`
for the inward cavity) via marching cubes. Every point on the resulting
surface lies exactly `T` meters perpendicular to the scan surface —
uniform offset, every point, by construction.

User direction 2026-05-16: "each layer should essentially just be a
perfectly scaled version of the base scan geometry." (Read as "uniform
offset" — additive, not multiplicative.)

## Why this matters now

We're entering the FEM insertion sim phase (slice 7+). The sim consumes
the layer geometry. If the layer surfaces are wrong (lumps, nipples,
chunkiness from per-vertex artifacts), the sim sits on wrong inputs.
This isn't a visual-polish issue — it's a load-bearing correctness issue
for the simulation work.

cf-cast-cli is unaffected — it samples the cleaned STL's SDF directly,
not the per-vertex displaced preview meshes. Mold geometry is already
correct; the design-tool preview is what's wrong.

## Where the existing per-vertex code lives

In `tools/cf-device-design/src/main.rs`:

- `EnvelopeProxyMesh` (struct, ~line 346) — decimated proxy +
  `vertex_radial_directions: Vec<Vector3<f64>>` field.
- `compute_envelope_proxy_mesh` — builds the proxy, computes per-vertex
  radials as `(vertex − nearest_centerline_pt).normalize()` (split by
  Z-region: below `centerline_min_z` the Z-component is zeroed to pin
  the bottom rim).
- `VERTEX_RADIAL_SMOOTH_ITERATIONS = 5` — Laplacian smoothing pass on
  the radial field (irrelevant under SDF approach; surface-normal offset
  is inherently smooth).
- Per-layer mesh rendering: `vertex_proxy + radial * offset_m` per
  vertex; same `proxy` connectivity reused for every layer.
- Cavity mesh: same pattern with negative offset.
- `EnvelopeProxyMesh::min_radial_distance_m` — cavity-self-intersection
  threshold (smallest `||v − nearest_centerline_pt||`). Will need an SDF
  analog for the Validations panel.

## Where the SDF + isosurface infrastructure lives

Already in the workspace:

- `mesh/mesh-sdf/src/sdf.rs`:
  - `SignedDistanceField::new(mesh: IndexedMesh) -> SdfResult<Self>` —
    builds the SDF from a triangle mesh (caches face normals).
  - `sdf.distance(point: Point3<f64>) -> f64` — query signed distance.
    Positive outside the mesh, negative inside.
- `mesh/mesh-offset` crate (re-exported by cf-cast):
  - `ScalarGrid::from_bounds(min, max, cell_size, padding_cells)` — 3D
    grid for marching cubes input.
  - `grid.set(ix, iy, iz, value)` — set per-cell scalar value.
  - `grid.position(ix, iy, iz) -> Point3<f64>` — world position of a cell.
  - `grid.dimensions() -> (usize, usize, usize)`.
  - `marching_cubes(&grid, &MarchingCubesConfig::default()) -> IndexedMesh`
    — extract isosurface at value 0 of the grid.

## Reference: cf-cast-cli's pattern

`design/cf-cast/src/mesher.rs::solid_to_mm_mesh`:

```rust
let bounds = solid.bounds().ok_or(...)?;
let mut grid = ScalarGrid::from_bounds(
    bounds.min, bounds.max, cell_size_m, GRID_PADDING_CELLS,
);
let (nx, ny, nz) = grid.dimensions();
for iz in 0..nz {
    for iy in 0..ny {
        for ix in 0..nx {
            let p = grid.position(ix, iy, iz);
            grid.set(ix, iy, iz, solid.evaluate(&p));
        }
    }
}
let mesh = marching_cubes(&grid, &MarchingCubesConfig::default());
```

For cf-device-design's case the cell evaluation becomes:
`sdf.distance(p) + offset_m` (positive offset for outward layers,
negative for the inward cavity). The shifted SDF's zero-isosurface is
the uniform offset surface.

`GRID_PADDING_CELLS = 2` in cf-cast — gives marching cubes a clean view
of the surface without inflating grid size meaningfully.

## Key open questions for the recon session

1. **Performance budget — live slider vs save-time.** Per-slider-tick
   budget is ~16 ms (60 FPS). Marching cubes at 2 mm cell size on a
   70 × 70 × 130 mm body ≈ 150k cells, plus an SDF query per cell. cf-cast
   runs this at save-time (slow path acceptable). cf-device-design needs
   responsive preview on slider drag. Options:
   - Coarser grid for live preview (e.g., 5 mm cell size ≈ 25k cells),
     full resolution on Save.
   - Cache per-cell SDF values (depend only on scan, not on layer
     thickness). Per slider tick only re-runs marching cubes with a
     shifted threshold (cheap-ish).
   - Incremental updates near the changed surface.
   - Async extraction on a background thread with last-good mesh
     rendered until ready.
2. **SDF cell size vs scan feature resolution.** Cleaned scan ≈ 169k
   faces with mm-scale features. Cell size choice affects faithfulness.
   2 mm = cf-cast default. Preview might tolerate coarser; design-time
   validations might need finer.
3. **Per-layer rendering pipeline.** Each layer is currently a separate
   Bevy Mesh asset built from `proxy.vertices` + per-vertex displacement.
   Under SDF: marching cubes produces a new `IndexedMesh` per layer
   (different topology per layer because each cumulative thickness has
   its own isosurface). Need to wire the new mesh into the Bevy asset
   pipeline and into the existing mesh-update systems.
4. **Cavity self-intersection check.** Currently
   `EnvelopeProxyMesh::min_radial_distance_m` — smallest
   `||v − nearest_centerline_pt||` over proxy vertices. SDF analog
   options: detect when the inward-offset isosurface degenerates (no
   triangles produced past some inset), or compute max-inward-distance
   via `sdf.distance` minima over the body interior. Validations panel
   (slice 6) reads this.
5. **Pour-volume calculation.** Slice 6 ships divergence-theorem signed
   mesh volume on the per-layer displaced meshes. The formula works on
   any closed triangle mesh, so it ports to SDF-extracted meshes
   unchanged. Confirm no edge cases at the cell-boundary topology.
6. **Test fixtures.** Sphere SDF → offset isosurface = bigger sphere
   (radius + offset). Cylinder → cylinder + offset radius + half-sphere
   caps at the ends. Box → rounded box (corners spherified). Tests
   should pin these analytical cases.
7. **Insertion sim coupling.** Slice 7+ FEM consumes layer geometry as
   `IndexedMesh`. SDF-extracted meshes have different topology from
   per-vertex displaced meshes (no shared-topology guarantee across
   layers). Verify the sim path tolerates this — likely benefits, since
   each layer can now have its own optimal tessellation.
8. **On-disk format.** `.design.toml` should be unchanged — stores layer
   thicknesses, not displaced geometry. Verify the recon.
9. **Validations panel + min-wall check.** Slice 6 reads per-layer
   thicknesses and tessellation density. SDF approach gives uniform
   thickness everywhere by construction — the min-wall check might
   simplify or become trivial. Confirm.

## Recon session agenda

1. Read `design/cf-cast/src/mesher.rs` IN FULL + `cf-design::Solid::evaluate`
   (the offset abstraction cf-cast uses to combine scan SDF + layer
   thickness).
2. Read crate-level docs: `mesh-sdf/src/lib.rs`, `mesh-offset/src/lib.rs`.
3. **Spike**: build the SDF for the iter-1 cleaned scan, fill a grid at
   2 mm and 5 mm cell sizes, extract the zero-isosurface, time each step.
   Quantifies the per-slider-tick feasibility.
4. **Triage** the perf options (coarser-for-preview / cache-SDF /
   async-extraction / mixed approach).
5. **Decide**: cell-size strategy, cache strategy, UI integration shape,
   how to handle the validations panel.
6. **Write spec** at `docs/CF_DEVICE_DESIGN_SDF_LAYERS_SPEC.md` with:
   chosen architecture, performance plan, test plan, migration plan,
   implementation arc sub-leaves, open risks.
7. **NO IMPLEMENTATION.** That's the next-next session.

## Implementation arc shape (after the recon ships a spec)

Estimated 4-8 commits. Likely sub-leaves (subject to recon's spec):

1. Subroutine: `extract_offset_isosurface(sdf, offset_m, cell_size_m)
   -> IndexedMesh` + unit tests against analytical cases (sphere,
   cylinder, box).
2. Replace cavity per-vertex mesh with SDF-isosurface cavity. Visual
   regression test on iter-1.
3. Replace per-layer outer surfaces with SDF-isosurface offsets. Visual
   regression: Layer 1 nipple should be gone on iter-1.
4. Cache the SDF on scan load (built once); per-slider-tick re-runs only
   marching cubes (cheap-ish).
5. Optional: coarser live-preview cell size + full-resolution on Save.
6. Retire `EnvelopeProxyMesh` per-vertex code paths
   (`vertex_radial_directions`, `smooth_radial_field`,
   `VERTEX_RADIAL_SMOOTH_ITERATIONS`, `min_radial_distance_m`).
7. Update Validations panel hooks (cavity self-intersection signal +
   pour volume on the new meshes).
8. Update tests + cold-read pass + grade clean + ship.

## PR #248 status

Currently has the centerline arc (6 commits, all green). Per user
direction 2026-05-16: bundle the SDF arc into the same PR. The dev
branch stays as-is across sessions; new commits get added; the eventual
squash-merge will be titled around the combined arc when both ship.

If recon + implementation take multiple sessions before merging, PR #248
stays open as the working PR. No need to close it during the recon
session.

## Reproduction fixture

`~/scans/sock_over_capsule.stl` (raw 3.34M faces, repo-excluded) →
through cf-scan-prep → `~/scans/sock_over_capsule.cleaned.stl` +
`.prep.toml`.

Open in cf-device-design to see the current Layer 1 nipple at the dome
apex. After the SDF arc lands, that nipple should be gone; Layer 1 should
look like a smooth uniform offset of the scan dome.

## Related

- `docs/CF_DEVICE_DESIGN_LAYER_PREVIEW_BOOKMARK.md` — predecessor (sister
  bookmark from 2026-05-16). Phases 1-3 are now moot / addressed by the
  centerline arc + full-mesh proxy. Phase 4 ("replace proxy with per-layer
  SDF resampling") IS this arc.
- `docs/CENTERLINE_SPEC.md` + `docs/CENTERLINE_RECON_BOOKMARK.md` — the
  centerline arc that just shipped. Sphere-cut bias insight documented;
  arc complete; the residual artifacts in cf-device-design are not the
  centerline algorithm's fault — they're the per-vertex displacement
  approach's fault, which is what this arc addresses.
