# cf-device-design SDF-Based Layer Surfaces — Spec

**Status**: SPEC (recon session output). Implementation = next session.
Bundles into PR #248.

**Predecessors**:
- `docs/CF_DEVICE_DESIGN_SDF_LAYERS_BOOKMARK.md` — problem statement +
  agenda for this recon.
- `docs/CF_DEVICE_DESIGN_LAYER_PREVIEW_BOOKMARK.md` — earlier
  layer-preview bookmark; this arc is its Phase 4.
- `docs/CENTERLINE_SPEC.md` + the just-merged centerline arc on
  `dev` (PR #248).

## Goal

Replace cf-device-design's per-vertex radial layer-surface displacement
with **uniform-offset isosurfaces of the cleaned scan's SDF**. Two
properties this delivers that the per-vertex approach cannot:

1. **No dome-apex nipple on Layer 1.** Apex vertices currently race past
   their neighbors because their "radial direction" points up-and-out
   from a centerline point inside the body. Uniform-offset isosurfaces
   are exactly perpendicular to the source surface by construction.
2. **Correct geometry inputs for the FEM insertion sim (slice 7+).**

## The chosen architecture

### Components

1. **Decimated SDF source** — `decimate_for_sdf(scan, 2500)` (reuse the
   existing helper at `tools/cf-device-design/src/insertion_sim.rs`).
2. **`SignedDistanceField`** wrapped in an `Arc`-style cheap-clone
   handle (`SharedScanSdf` analog already exists at
   `tools/cf-cast-cli/src/scan.rs`).
3. **Cached `ScalarGrid` filled with raw SDF values** — one fill per
   scan-load. NOT per layer. NOT per slider tick.
4. **Per-layer extraction** — `MarchingCubesConfig::at_iso_value(offset)`
   over the cached grid. Each layer has its own MC topology.

### Pipeline

**On scan load** (and ONLY on scan load):

```
decimated = decimate_for_sdf(scan, SDF_SOURCE_TARGET_FACES)
sdf       = Arc::new(SignedDistanceField::new(decimated)?)
bounds    = scan_aabb + LAYER_GRID_MARGIN_M
grid      = ScalarGrid::from_bounds(bounds, LAYER_PREVIEW_CELL_SIZE_M, 0)
for (ix, iy, iz) in grid.cells():
    grid.set(ix, iy, iz, sdf.distance(grid.position(ix, iy, iz)))
cache = CachedScanSdf { sdf, grid, bounds }
```

**On slider tick** (cavity inset, layer count, layer thickness):

```
for affected_layer in changed_layers:
    iso  = cumulative_offset_m_at(affected_layer)   // negative for cavity
    cfg  = MarchingCubesConfig::at_iso_value(iso)
    mesh = marching_cubes(&cache.grid, &cfg)
    bevy_mesh = build_bevy_mesh(mesh, up, render_scale)
    // GPU upload via Bevy Mesh asset replace
```

**On Save** (open question — see Open risks #3): same pipeline at
finer cell size if higher fidelity needed.

### Sign convention (pinned by the spike)

`sdf.distance(p)` is POSITIVE outside the scan, NEGATIVE inside.
`MarchingCubesConfig::at_iso_value(iso)` extracts the surface where
`grid_value == iso`:

- `iso = +T` (T > 0) → outward offset by T meters (the layer outer
  surfaces).
- `iso = -T` (T > 0) → inward offset by T meters (the cavity surface).
- `iso = 0` → original scan surface.

This is why we fill with RAW SDF values (no iso shift) and let the
extraction config pick the offset — every layer reuses the same grid
fill. Contrast with `mesh-offset::offset_mesh`, which subtracts the
offset inside `sample_sdf_to_grid` and always extracts at iso 0 (one
fill PER offset). Our path is functionally equivalent but cache-
friendly.

### Proposed constants

- `SDF_SOURCE_TARGET_FACES: usize = 2_500` — matches the insertion-sim
  spike's production value.
- `LAYER_PREVIEW_CELL_SIZE_M: f64 = 0.005` — 5 mm preview grid; chosen
  from spike data (see Performance plan).
- `LAYER_GRID_MARGIN_M: f64 = 0.040` — 40 mm margin around scan AABB.
  Covers the worst case of 6 layers × 5 mm cumulative offset + headroom.

## Performance plan

### Spike measurements (2026-05-16, M-series, release build, iter-1
cleaned scan: 167,912 faces, 503,736 vertices)

| SDF source     | Cell | Grid cells | One-time fill | MC per layer |
|----------------|------|-----------:|--------------:|-------------:|
| **RAW (167k)** | 5 mm |     20,160 |   **39411 ms** ❌ |        < 1 ms |
| dec-5000       | 5 mm |     20,160 |        707 ms |        < 1 ms |
| **dec-2500**   | 5 mm |     20,160 |    **324 ms** |    **< 1 ms** |
| dec-1500       | 5 mm |     20,160 |        170 ms |        < 1 ms |
| dec-5000       | 2 mm |    276,165 |       7641 ms |        3–4 ms |
| dec-2500       | 2 mm |    276,165 |       3167 ms |        2–3 ms |
| dec-1500       | 2 mm |    276,165 |       1604 ms |        2–3 ms |

(SDF build: 0–3 ms across all configurations — not the bottleneck.)

### Conclusions

1. **Raw scan SDF is non-viable.** 39 s grid fill at 5 mm is
   catastrophic. MUST decimate.
2. **Grid fill is the dominant cost.** Scales O(faces × cells) since
   mesh-sdf has no spatial acceleration (`unsigned_distance_with_face`
   iterates all faces per query).
3. **MC per-layer is essentially free.** < 1 ms at 5 mm, 2–3 ms at 2 mm.
   The fill-once-extract-many architecture works.
4. **324 ms one-time fill at scan load** is acceptable as a startup
   cost (cf-scan-prep's STL load already takes ~5 ms and the decimation
   takes 30 ms; the SDF fill is the dominant load-time step but still
   well under a second).

### Per-slider-tick budget walk-through

| Scenario                  | MC time | Bevy mesh build | Total tick | Headroom |
|---------------------------|--------:|----------------:|-----------:|---------:|
| Single layer thickness    | < 1 ms  |          ~3 ms |     ~4 ms |  60 FPS ✓ |
| Cavity inset (6 layers)   |  ~3 ms  |          ~20 ms |    ~25 ms |  40 FPS  |

The cavity-inset case is the worst because each layer's outer offset =
`cumulative_thickness − cavity.inset_m`, so changing inset shifts every
layer's iso.

**Decision**: ship the synchronous path first. Async-extract on a
background thread is in scope if 25 ms-per-tick proves laggy in
practice (cf-device-design already uses `bevy_tasks` for the insertion
sim — pattern in hand). Mitigation is a sub-commit, not a design
re-think.

## Test plan

### Unit tests in the new `sdf_layers` module

1. **Sphere isosurface**: build SDF from a small icosphere mesh (radius
   R). Extract at iso +T. Assert all vertices at radius `R + T ± 0.5 ×
   cell_size`. Pins the uniform-offset property.
2. **Box isosurface**: SDF of a unit cube. Extract at iso +T. Corners
   should be rounded with radius ≈ T (signature of SDF offset, NOT of
   the per-vertex displacement). Inspect via max-distance-from-flat-
   face metric.
3. **Cylinder isosurface**: capped cylinder. Caps should grow into
   hemispherical end-caps (signature of SDF offset). Verify by sampling
   end-cap vertices and asserting they lie on a hemisphere of radius
   `cylinder_radius + offset`.
4. **Inward offset = cavity**: same cube at iso −T. Output should be a
   smaller cube (still rounded). Volume should decrease monotonically
   with T.
5. **Cavity collapse**: cube at iso −0.6 (60% of half-extent). Should
   produce empty or near-empty mesh — pins the cavity-self-intersection
   detection path.
6. **`build_cached_scan_sdf` deterministic**: same input → same fill.
7. **Iso-value re-extraction is independent**: extract at iso +T₁ then
   iso +T₂ from the same cache, assert the latter is unaffected by the
   former (no grid mutation between calls).

### Visual regression (manual, user-side)

- Open `~/scans/sock_over_capsule.cleaned.stl` in cf-device-design
  after the swap. Layer 1 dome apex must be SMOOTH (no nipple). Compare
  screenshots before / after the arc on the same fixture. This is the
  load-bearing user-facing gate.

## Migration plan (implementation arc sub-leaves)

Estimated 5–7 commits. Each ends green per workspace conventions:
`cargo test -p cf-device-design --lib`, `cargo clippy -p
cf-device-design`, `cargo run -p xtask -- grade cf-device-design`. PR
#248 stays open across the arc.

### 1. Add the `sdf_layers` module + cache + tests

New `tools/cf-device-design/src/sdf_layers.rs`:

- `pub struct CachedScanSdf { sdf: Arc<SignedDistanceField>, grid:
  ScalarGrid, bounds: (Point3<f64>, Point3<f64>) }`
- `pub fn build_cached_scan_sdf(scan: &IndexedMesh, cell_size_m: f64,
  margin_m: f64) -> Result<CachedScanSdf>` — decimates via the existing
  `decimate_for_sdf` (lift it to `pub(crate)` in `insertion_sim`), then
  fills the grid.
- `pub fn extract_layer_surface(cache: &CachedScanSdf, offset_m: f64)
  -> IndexedMesh` — `MarchingCubesConfig::at_iso_value(offset_m)` plus
  `marching_cubes(...)`.
- Unit tests per the Test plan §"Unit tests".

### 2. Wire `CachedScanSdf` as a Bevy resource

Bevy `Resource` parallel to `EnvelopeProxyMesh`. Built at scan load
(same insertion point as `EnvelopeProxyMesh` construction). Add to the
Bevy app's startup pipeline; gate the SDF cache build behind the same
"valid scan loaded" precondition.

### 3. Replace cavity preview

`update_cavity_mesh` swaps from
`build_displaced_proxy_mesh(&proxy, -cavity.inset_m, ...)` to a new
`build_bevy_mesh_from_indexed(extract_layer_surface(&cache,
-cavity.inset_m), ...)`. Visual gate: cavity surface smooth on iter-1.

### 4. Replace per-layer outer surfaces

Same swap for `update_layer_meshes`. Per-layer extraction at
`iso = cumulative_thickness − cavity.inset_m`. **Visual gate**: Layer 1
nipple GONE on iter-1 dome apex. This is the load-bearing user-visible
slice.

### 5. Validations panel SDF analogs

- `signed_volume_m3`: drop the `(proxy, offset_m)` signature; take an
  `&IndexedMesh` (the layer's MC mesh) directly. Divergence-theorem on
  the mesh's own vertices, no displacement step.
- `compute_validations`: pass each layer's already-extracted MC mesh
  rather than computing volumes from a shared proxy. The MC meshes are
  already in scope at the render call sites — pass them through.
- Cavity self-intersection: replace `inset >= proxy.min_radial_distance_m`
  with the cavity MC mesh's empty / near-empty face count. Or: maintain
  the cheap pre-computed scalar `max_inward_distance_m = -sdf.min()`
  over interior cells of the cached grid. Pick the latter if `is_empty`
  bounces noisily near the collapse threshold.

### 6. Retire per-vertex code paths

Delete after the new path is plumbed end-to-end:

- `EnvelopeProxyMesh::vertex_radial_directions`
- `EnvelopeProxyMesh::min_radial_distance_m`
- `smooth_radial_field`
- `VERTEX_RADIAL_SMOOTH_ITERATIONS`
- The radial-computation block in `compute_envelope_proxy_mesh` (steps
  5–7).
- `build_displaced_proxy_mesh` / `_with_colors` (replaced by
  `build_bevy_mesh_from_indexed`).

If nothing else needs `EnvelopeProxyMesh.vertices/faces` after the
swap, retire the struct entirely; the SDF cache subsumes its role.

### 7. Heat-map re-projection

The Insertion Sim panel's Option-C per-vertex coloring (Ψ, ‖P‖)
currently keys on `proxy.vertices` length. Under SDF each layer has its
own MC vertex set. Re-project per-tet scalar fields onto each layer's
MC mesh via closest-point lookup at heat-map-update time. Likely a
separate small commit because the projection helper crosses module
boundaries; may benefit from a per-layer-vertex → nearest-tet cache to
avoid re-walking per frame.

### 8. Cold-read pass + grade clean + ship

Standard end-of-arc pass. Update the bookmark to RESOLVED. Bank patterns
in a per-arc memo (`project_cf_device_design_sdf_layers.md`).

## What stays unchanged

- `.design.toml` schema (stores thicknesses, not displaced geometry).
- Per-layer thickness sliders + UI.
- `compute_validations` overall shape — just swaps the volume helper.
- Layer materials, colors, palette (`LAYER_SURFACE_PALETTE`,
  `CAVITY_COLOR`).
- cf-cast-cli (uses cleaned STL SDF directly, doesn't touch the design
  preview meshes).
- The insertion sim (already on its own SDF path via
  `run_sdf_bridge_spike` / `build_insertion_geometry`).
- The centerline overlay (centerline arc just shipped; orthogonal to
  layer-surface rendering).

## Open risks

### 1. Heat-map re-projection cost

Per-tet → per-layer-vertex projection currently rides on the shared
`proxy.vertices`. Under SDF, projection happens per-layer-mesh. With
~10k tets × ~3k vertices per layer × 6 layers ≈ 180M closest-point ops
per heat-map update. Probably needs a precomputed nearest-tet map per
layer mesh. Defer the bench to the implementation arc; if the naive
path is too slow, add the cache as a sub-commit.

### 2. Decimation fidelity for non-iter-1 scans

`SDF_SOURCE_TARGET_FACES = 2500` is generous for the smooth iter-1 sock
fixture. Body parts with finer features (knuckles, fingertips, nostrils)
may need 5000+ faces. The current spec hard-codes 2500; revisit when
iter-2 / iter-3 scans surface a visible quality regression. Pin as a
named constant so the change is one-line. Per the
[cf-scan-prep target use case memo][use-case], body-part scans ARE the
primary use case, so this risk is real — but iter-1 is the immediate
deadline and 2500 is correct for that.

[use-case]: project_cf_scan_prep_target_use_case.md

### 3. Save cell size

Spec proposes preview cell size = save cell size = 5 mm. **Open**:
should Save bump to 2 mm for higher-fidelity exports? Cost = 3 s on
Save. Probably yes once round-trip Save → re-open becomes a workflow
(today the only consumer is cf-cast-cli, which re-samples the cleaned
STL's SDF anyway, NOT the per-layer cf-device-design meshes). Defer
to a sub-commit decision after iter-1 ships.

### 4. Margin for max-layer offset

Spec uses 40 mm. 6 layers × max ~5 mm = 30 mm cumulative + safety
margin. If layers ever support > 40 mm offsets (unlikely — the device
is small), the grid bounds need to extend. Pin as
`LAYER_GRID_MARGIN_M`; `extract_layer_surface` should
`debug_assert!(offset_m.abs() < LAYER_GRID_MARGIN_M)` to catch any
future regression at test time.

### 5. SDF sign anomalies on the cleaned scan

`mesh-sdf`'s `compute_sign` uses face-normal sign of the closest face.
For meshes with consistent CCW outward winding (cf-scan-prep's output)
this is correct, BUT the [cf-scan-prep cap-winding concern memo][cap]
notes that `auto_cap_open_boundaries` flips winding to INWARD on caps.
On the iter-1 fixture this is workshop-quiet (cf-cast tolerates it,
the layer surfaces likely tolerate it too if MC sees consistent
sign per region). Flag for the implementation arc: if Layer 1 still
shows artifacts after the SDF swap, investigate cap-winding first.

[cap]: project_cf_scan_prep_cap_winding_concern.md

### 6. `is_changed()` footgun

The current `update_layer_meshes` uses a `LayerMeshKey` snapshot to
dodge Bevy's `is_changed()` deref-mut footgun
([memo][is-changed-memo]). The new path MUST preserve the snapshot
pattern — extending `LayerMeshKey` with whatever new inputs the
`CachedScanSdf`-based extraction reads. The cache itself is built once
at scan load, so it doesn't need to enter the snapshot; layer thickness
+ cavity inset + layer count already do.

[is-changed-memo]: project_bevy_is_changed_footgun.md

## Out of scope (defer)

- Centerline UI / wire-handles (orthogonal — those address scan
  orientation, not surface geometry).
- Async background extraction (in scope only if 25 ms cavity-tick is
  laggy in practice — measure first).
- Per-layer SDF source decimation (every layer uses the same shared
  SDF — no per-layer decimation knob).
- Caching the per-layer MC meshes across slider ticks (extraction is
  cheap enough that re-MC per tick beats invalidation logic).
- Generalizing `mesh-offset::offset_mesh` to accept a cached SDF +
  iso-value pair. Could be a follow-up if a second caller needs it; for
  now the cache lives in cf-device-design only.
