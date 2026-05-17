# cf-device-design Cavity Pinned-Floor — Spec

**Status**: RECON COMPLETE 2026-05-16 EVENING (fresh-context session
following [`CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_BOOKMARK.md`](CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_BOOKMARK.md)).
Implementation NOT yet started; user reviews this spec before any
code lands. dev at `3cb6c8a0`, tree clean, 146 tests / 9 ignored on
cf-device-design.

**Parent**: [`CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_BOOKMARK.md`](CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_BOOKMARK.md)
— carries the verbatim user geometric model + 8 recon questions. Read
its §1 BEFORE this spec; the user's model is the load-bearing context.

**Supersedes**: [`CF_DEVICE_DESIGN_CAVITY_MOUTH_SPEC.md`](CF_DEVICE_DESIGN_CAVITY_MOUTH_SPEC.md)
— shipped but solving wrong problem (open cavity vs closed-with-pinned-
floor). Most of its scaffolding survives unchanged (see §1 Q5).

**Blocks**: penetration sim, mold generation, fit-viz rungs 2-6 — all
LOCKED until this lands.

---

## Headline finding (read this first if you read nothing else)

**The cavity-mouth arc's two-SDF cached-grid construction is exactly
the primitive the pinned-floor formulation needs.** The shipped code
stripped cap polygons from the SDF source for an open cavity; the
pinned-floor version uses the SAME two-SDF construction and INTERSECTS
the iso surface with the cap-plane half-space at extract time. Net
implementation change: ~80 LOC in `extract_layer_surface` + ~50 LOC
deletion in `signed_volume_m3` callers. The expensive scaffolding
(`CapPlane` parser, `dome_wall_only_mesh`, `build_cached_scan_sdf`'s
two-SDF path, `CachedScanSdf` struct, `report_cap_face_classification`
diagnostic) stays VERBATIM.

The downstream-consumer plumbing (insertion_sim + cf-cast-cli
switching from `scan_sdf.offset(...)` to a pinned-floor variant) is
banked as followups — those consumers compose at the
`cf-design::Solid` level today, never see a triangle mesh, and depend
on a primitive that's a separate (later) arc to extract.

---

## 1. Recon answers

### Q1. API shape — primitive's signature

**Answer**: keep the current
`extract_layer_surface(cache, cap_planes, offset_m) -> IndexedMesh`
signature. Behavior changes (now returns a CLOSED manifold with a
flat floor at every included cap plane) but every call site stays
unchanged.

**Rationale**:
- All four production call sites (`spawn_cavity_mesh`,
  `update_cavity_mesh`, `update_layer_meshes`, `compute_validations`)
  already pass `(cache, &cap_planes.planes, offset_m)`. No threading
  changes.
- The `(cavity, layer 0, layer 1, ..., layer N-1)` shells share one
  uniform primitive — every shell is "scan offset by some
  `offset_m`, floor pinned at the cap plane(s)". The cavity is
  just `offset_m = -inset`; outer layers are `offset_m = +T`.
  Splitting the primitive into wall + floor halves would force every
  caller to compose them, with no upside.
- Multi-cap support is automatic (the slice already accepts `&[CapPlane]`;
  the new composition folds over caps with `max`).

**Downstream-consumer primitive** (the cf-design version for
insertion_sim + cf-cast-cli): banked as Followup F1. That primitive
returns a `Solid` (not a mesh) and composes
`Solid::from_sdf(dome_wall_signed_sdf, bounds).offset(offset_m).intersect(Solid::plane(...))`
per cap. Out of scope for v1 — see §5.

### Q2. Implementation strategy — pick one of (a)-(c) from the bookmark

**Picked**: **(d) per-extract SDF composition** (a new alternative
the bookmark didn't list).

**Construction**: leave the cached grid storing the two-SDF modulated
distance `sign(sd_closed) * |sd_open|` (= the dome-wall-signed-distance
field). At extract time, clone the grid and per-cell compose with
the cap-plane half-spaces:

```rust
fn extract_layer_surface(cache, cap_planes, offset_m) -> IndexedMesh {
    if cap_planes.is_empty() {
        // No-caps fast path = pre-pinned-floor behavior, byte-identical.
        let config = MarchingCubesConfig::at_iso_value(offset_m);
        return marching_cubes(&cache.grid, &config);
    }
    let mut composed = cache.grid.clone();
    for (ix, iy, iz) in grid_cells {
        let p = composed.position(ix, iy, iz);
        let shifted_scan_sd = composed.get(ix, iy, iz) - offset_m;
        // Cap half-space SDF: > 0 outside body (above cap plane).
        // Intersection-of-half-spaces => fold with max.
        let cap_sd = cap_planes.iter()
            .map(|c| (p.coords - c.centroid.coords).dot(&c.normal))
            .fold(f64::NEG_INFINITY, f64::max);
        // Intersection of (shifted dome-wall-signed body) and (body-
        // interior side of every cap plane).
        composed.set(ix, iy, iz, shifted_scan_sd.max(cap_sd));
    }
    marching_cubes(&composed, &MarchingCubesConfig::at_iso_value(0.0))
}
```

**Why (d) over the bookmark's three options**:

- **(a) anisotropic SDF offset (skip cap-normal direction)** — REJECTED.
  `mesh-sdf` has no such API (Q6 below); building one is an
  arc-sized addition to `mesh-sdf` / `mesh-offset` that would have
  to support per-vertex direction masking. The cf-design composition
  achieves the same geometric result without modifying either crate.
- **(b) post-MC delete-floor + stitch-down + flat-cap triangulation** —
  REJECTED. Post-MC stitching is fragile: boundary loops may not be
  clean after MC interpolation jitter, multi-cap requires per-cap loop
  detection + classification, and the flat-cap triangulation has to
  match the boundary-loop polygon shape per shell. (d) sidesteps all
  of this by letting MC handle the closure naturally — the iso 0 of
  the composed SDF includes the cap-plane intersection as a flat
  polygon.
- **(c) use the cleaned-scan's actual cap polygon as the floor
  triangulation directly** — REJECTED. The cleaned-scan cap polygon
  is at one fixed shape (matching cf-scan-prep's emitted polygon).
  Per-shell pinned-floor polygons have DIFFERENT shapes per
  `offset_m` (the inset cavity's floor is the inset polygon; the
  outer layer's floor is the outward polygon). Reusing cf-scan-prep's
  polygon would require per-shell shrink/grow logic on the polygon
  itself — a 2D polygon offset problem that's harder than the 3D
  SDF composition.

**Subtle benefit of (d)**: every shell's floor is a triangulated
polygon at `iso 0 ∩ cap_plane`, produced by MC's own triangulator.
Each shell gets its own naturally-stitched floor matched to its own
iso shape, with NO custom triangulation code anywhere in the pipeline.

**Cost**: per-extract grid clone is `O(cells)`. iter-1's cached grid
is ≈20 k cells, ≈ 0.5 ms per extract. Per-cell composition is
`O(cells × n_caps)`; n_caps = 1 on iter-1 → another ≈ 0.5 ms. MC stays
sub-millisecond. Total per-extract cost roughly doubles from ~0.5 ms
→ ~1.5 ms; absolutely negligible against frame rate. Could optimize
further by composing in-place (no clone) if profiling surfaces a
real cost — banked as Followup F6.

### Q3. Where it lives — cf-device-design vs cf-design

**Picked**: v1 stays in cf-device-design's `sdf_layers` module. The
cf-design extraction is banked as Followup F1.

**Rationale**:
- The cf-device-design preview is the ONLY consumer today (insertion_sim
  + cf-cast-cli operate on `Solid::from_sdf(scan_sdf, bounds).offset(...)`,
  the BARE closed-scan SDF, and don't know about cap planes at all).
- The v1 change is narrowly scoped to `extract_layer_surface` +
  `signed_volume_m3` callers — both inside cf-device-design.
- Extracting to cf-design requires pulling `CapPlane` (currently
  `pub(crate)` in cf-device-design) + the two-SDF builder
  (`build_cached_scan_sdf` + `dome_wall_only_mesh`) + cap-face
  classification constants into a shared library. That's its own arc
  with its own design questions (does cf-design own `.prep.toml`
  schema awareness? if not, where does CapPlane parsing live? does
  insertion_sim's tet-mesher tolerate the composed SDF's piecewise-
  smooth gradient near the cap plane?).
- v1 unblocks cf-device-design's penetration-sim preview + fit-viz
  rungs 2-6. F1 separately unblocks insertion_sim + cf-cast-cli to
  use pinned-floor geometry (today they both use uniform offset of
  the closed scan, which has its own well-documented "the scan's
  cap polygon is part of the SDF body, so offset shrinks/grows past
  the cap plane" behavior — works for v1 of cf-cast-cli + the
  insertion sim because both consume the resulting mesh / tet mesh
  without caring about the cap-plane side; F1 makes them caring is
  the workshop-iter-2 ergonomics step).

**Implication**: cf-device-design's preview will show CLOSED shells
with pinned floors; cf-cast-cli's mold STLs + insertion_sim's tet
mesh will STILL be uniform-offset of closed scan (= today's behavior)
until F1+F2+F3 land. The visual divergence (preview shows pinned-
floor; downstream tools don't) is acceptable for v1 — preview is the
load-bearing user-facing tool; the downstream consumers' geometry has
been workshop-iter-1-good enough.

### Q4. Shells vs scan cap polygon — shared or per-shell triangulation?

**Answer**: per-shell, naturally-triangulated by marching cubes at
the iso ∩ cap-plane intersection.

**Rationale** (covered in Q2): every shell's pinned-floor polygon is
a different shape (the inset cavity's floor is the inset polygon; the
outer T-shell's floor is the outward polygon). MC on the composed
SDF triangulates each polygon at extract time. No shared cap
polygon, no per-shell shrink/grow logic on a polygon, no relationship
between cf-scan-prep's emitted cap polygon and the per-shell floors.

The cf-scan-prep emitted cap polygon is consumed ONLY for the
`CapPlane` parameters (centroid + outward normal + vertex_count for
the primary-cap tiebreak). The polygon ITSELF is not consumed
geometrically by the pinned-floor primitive.

Winding-consistency: MC on a well-formed SDF produces consistent
inward-winding triangles (per `mesh-offset`'s convention; the existing
`signed_volume_m3` already strips the sign for floating-point-noise
reasons). The pinned-floor extraction preserves this convention.

### Q5. Two-SDF code disposition — keep vs rip out

**Answer**: KEEP. Almost the entire cavity-mouth arc's scaffolding
is reusable for the pinned-floor primitive.

**Stays** (~1011 LOC across sdf_layers.rs + main.rs):
- `CapPlane` / `CapPlanes` struct (75 LOC). Data layer is unchanged.
- `parse_cap_planes` (≈40 LOC). Cap-plane parser stays.
- `dome_wall_only_mesh` (≈50 LOC). Two-SDF construction needs the
  open mesh.
- `report_cap_face_classification` (≈100 LOC diagnostic). Permanent
  regression sentinel for cap-face detection.
- `build_cached_scan_sdf` two-SDF path (≈80 LOC). Cached grid still
  stores modulated distance.
- `CachedScanSdf` struct (≈30 LOC). All fields still used (`sdf_closed`,
  `sdf_open`, `grid`, `bounds`, `min_sdf_value`).
- `CAP_FACE_NORMAL_DOT_MIN` + `CAP_FACE_CENTROID_DIST_M` constants
  (face-normal cap-face detection survives).
- `clip_mesh_against_cap_plane` (≈100 LOC). KEPT as generic utility
  (matches the bookmark's pattern-bank note), but loses its only
  caller. Could delete now if no future use predicted; recommendation
  is KEEP for one revision cycle in case Followup F1's tetra-mesher
  needs it, then prune in F6 cleanup.

**Deletes** (~50 LOC across main.rs):
- `primary_cap_origin` (≈15 LOC) — closed shells are divergence-
  origin-invariant, so the cap-centroid-origin trick is no longer
  needed; revert to `Point3::origin()`. Tests
  `primary_cap_origin_no_caps_is_world_origin`,
  `primary_cap_origin_picks_largest_by_vertex_count`,
  `primary_cap_origin_ties_break_on_lowest_loop_index` (≈40 LOC)
  go with the function.
- `signed_volume_m3`'s second `origin` parameter (≈5 LOC). Revert
  to single-arg `signed_volume_m3(mesh) -> f64`. The 6 test cases
  that exercised origin-shifting (`signed_volume_m3` returns 1.0
  for origin + (1,1,1) shifts) still pass — origin-invariance was
  the property being tested, just for the closed case.

**Changes** (~80 LOC in sdf_layers.rs):
- `extract_layer_surface` body: replace `at_iso_value(offset_m) +
  post-MC clip loop` with `clone grid + per-cell compose + MC at iso 0`.
  See §2 sub-leaf 1 for the diff.

### Q6. Anisotropic offset feasibility

**Answer**: not needed; mesh-sdf / mesh-offset don't support it
(confirmed by surface-scan); the cf-design composition (= the per-
extract SDF composition picked in Q2) achieves the same geometric
result without modifying either crate.

**API survey** (mesh-sdf + mesh-offset):

| Crate | Function | Behavior |
| --- | --- | --- |
| mesh-sdf | `SignedDistanceField::{new, distance, unsigned_distance, closest_point, is_inside}` | Uniform SDF queries only. No directional masking. |
| mesh-offset | `offset_mesh(mesh, config)` | Uniform offset via SDF + MC. One-shot. No per-vertex direction. |
| mesh-offset | `ScalarGrid::{from_bounds, get, set, dimensions, position}` | Generic scalar grid; consumers compose SDF values into cells. |
| mesh-offset | `MarchingCubesConfig::{at_iso_zero, at_iso_value}` | Extract iso surface at a stored value. |

To implement (a) anisotropic offset: would need either (a1)
per-face directional offset distances in mesh-offset's SDF→grid fill
loop, or (a2) a new "skip-axis" parameter that's applied per-cell
based on the cell's spatial relationship to a reference plane. Either
addition spans both mesh-sdf (the SDF derivation) and mesh-offset
(the fill loop). 100+ LOC, separate arc, new public API surface.

The cf-design composition does the same job by INTERSECTING the
uniform-offset SDF with a cap-plane half-space at composition time
— uses ONLY existing primitives (`max` operation in SDF algebra,
already implicit in `Solid::intersect`).

**Verdict**: (a) is over-engineering for this primitive. The
composition approach is correct, cheap, and uses existing crate
surfaces.

### Q7. Multi-cap handling

**Answer**: automatic via the fold-over-caps with `max`. iter-1
single-cap and iter-N two-cap (proximal + distal) use the same code
path.

**Construction**: the per-cell composition `fold(NEG_INFINITY, max)`
over caps composes the intersection-of-half-spaces SDF. For:

- 0 caps: `cap_sd = NEG_INFINITY`; `max(scan_sd - offset, -inf) =
  scan_sd - offset`. Identical to today's `at_iso_value(offset_m)`.
  No-caps fast path is byte-identical.
- 1 cap: one half-space intersection. Shell has 1 flat floor.
- 2 caps: two half-space intersections. Shell has 2 flat floors
  (one at each cap plane).
- N caps: N flat floors.

**iter-2 scope**: body-part scans (e.g. forearm: wrist + elbow caps)
will exercise the 2-cap path. The primitive needs no special code
for it — the fold handles N caps uniformly. iter-2 visual gate is
just confirmation, not a separate code change.

**The primary-cap concept becomes obsolete**: today
`primary_cap_origin(cap_planes)` picks ONE cap centroid as the
divergence-volume integration origin (largest by vertex_count,
tiebroken by lowest loop_index). With closed shells, the divergence
volume is origin-invariant, so no primary-cap pick is needed. Delete
the helper. (See Q5.)

### Q8. What does the insertion sim CSG actually want? (LOAD-BEARING)

**Answer**: it doesn't want a triangle mesh at all. The insertion
sim composes at the SDF/`Solid` level via `cf-design::Solid::subtract`,
never sees a cavity mesh, doesn't care about manifold/winding/water-
tightness of any triangle output of `extract_layer_surface`.

**Evidence** (`tools/cf-device-design/src/insertion_sim.rs:660-813`):

```rust
let (scan_sdf, _) = build_grid_sdf(&decimated, bounds, grid_cell_m, ...)?;
// ...
let cavity = Solid::from_sdf(scan_sdf.clone(), bounds).offset(cavity_offset_m);
let outer = Solid::from_sdf(scan_sdf.clone(), bounds).offset(outer_offset_m);
let body = outer.subtract(cavity);
// ...
let mesh = SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh(&body, &hints)?;
```

The chain: scan_sdf → `Solid::from_sdf` → `.offset()` → `.subtract()`
→ `from_sdf_yeoh` (BCC-lattice tet-mesher resamples the SDF
independently — doesn't see any caller's triangle mesh).

cf-cast-cli takes the same path
(`tools/cf-cast-cli/src/derive.rs:196-208`):

```rust
let plug = Solid::from_sdf(scan_sdf.clone(), sdf_bounds).offset(-cavity_inset_m);
// ...
let body = Solid::from_sdf(scan_sdf.clone(), sdf_bounds)
    .offset(cumulative_so_far - cavity_inset_m);
```

**Implication**:
- **For the FEM sim**, the cavity mesh's manifoldness/winding/water-
  tightness is irrelevant — the sim's `from_sdf_yeoh` consumes
  `body` as a Solid, samples its SDF over a BCC lattice, and builds
  its own tet mesh. The pinned-floor primitive's job is to deliver
  the CORRECT GEOMETRIC SDF (= "scan inset cavity AND floor at cap
  plane"), not a particular mesh representation.
- **For cf-device-design's PREVIEW** (the egui+Bevy visualization),
  the cavity mesh IS what the user sees — closed/manifold/water-tight
  matters here. MC on the composed SDF produces a closed manifold
  naturally when the SDF is well-formed. The pinned-floor formulation
  in Q2 is well-formed.
- **For `signed_volume_m3`** (the Validations panel's per-layer pour-
  volume + mass-budget readout), CLOSED shells make the divergence
  integral origin-invariant, recovering the pre-cavity-mouth-arc
  bit-exact contract. Cap-centroid-origin trick (added for OPEN
  surfaces) is no longer needed.
- **For cf-cast-cli's mold STLs** (the workshop-physical output),
  the per-layer body mesh is what gets sliced + printed. Today
  cf-cast-cli uses `Solid::from_sdf(scan_sdf).offset(...)` directly,
  producing meshes that aren't pinned-floor. That's a separate plumb-
  ing arc (Followup F3). v1 of the pinned-floor primitive does NOT
  change cf-cast-cli's output.

**Cross-checking against the user's geometric model (bookmark §1)**:
> "All shells are closed manifolds — the cavity is a 'hole' that
> gets subtracted (CSG) from the layer 0 material for the
> penetration simulation."

The user said "CSG subtract". The insertion sim's `outer.subtract(cavity)`
IS CSG subtract — at the SDF level (`Solid::subtract` = SDF max of
shell vs `-other`). The user's words and the sim's mechanism agree;
the user just didn't necessarily realize the CSG happens at the SDF
level (no triangle CSG library is involved). No user-spec collision.
The pinned-floor primitive delivers the CORRECT geometric `Solid`
for `subtract` to operate on; the rest already works.

---

## 2. Implementation ladder

**Total: 2 sub-leaves of code change + 1 user-side visual gate.**
Per-sub-leaf rationale + test list below.

### Sub-leaf 1 — Rewrite `extract_layer_surface` to compose pinned-floor SDF at extract time

**File**: `tools/cf-device-design/src/sdf_layers.rs`

**Change**: replace the body of `extract_layer_surface` with the
construction from §1 Q2. Drop the post-MC clip loop.

**Diff sketch**:

```rust
// BEFORE (lines 846-864):
pub(crate) fn extract_layer_surface(
    cache: &CachedScanSdf,
    cap_planes: &[CapPlane],
    offset_m: f64,
) -> IndexedMesh {
    debug_assert!(offset_m.abs() < LAYER_GRID_MARGIN_M, ...);
    let config = MarchingCubesConfig::at_iso_value(offset_m);
    let mut mesh = marching_cubes(&cache.grid, &config);
    for plane in cap_planes {
        mesh = clip_mesh_against_cap_plane(&mesh, plane);  // ← REMOVED
    }
    mesh
}

// AFTER:
pub(crate) fn extract_layer_surface(
    cache: &CachedScanSdf,
    cap_planes: &[CapPlane],
    offset_m: f64,
) -> IndexedMesh {
    debug_assert!(offset_m.abs() < LAYER_GRID_MARGIN_M, ...);
    if cap_planes.is_empty() {
        // No-caps fast path: byte-identical to pre-pinned-floor behavior.
        let config = MarchingCubesConfig::at_iso_value(offset_m);
        return marching_cubes(&cache.grid, &config);
    }
    // Composed pinned-floor SDF (per §1 Q2):
    //   shell_sd(p) = max( cache.grid(p) - offset_m,
    //                      max over caps of (p - cap.centroid) · cap.normal )
    // The first term is the uniform-offset two-SDF body; the second
    // is the union-of-cap-half-spaces SDF (positive outside body
    // interior, intersected via SDF max). iso 0 of the composed
    // field is the pinned-floor shell boundary.
    let mut composed = cache.grid.clone();
    let (nx, ny, nz) = composed.dimensions();
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let p = composed.position(ix, iy, iz);
                let shifted_scan_sd = composed.get(ix, iy, iz) - offset_m;
                let cap_sd = cap_planes
                    .iter()
                    .map(|c| (p.coords - c.centroid.coords).dot(&c.normal))
                    .fold(f64::NEG_INFINITY, f64::max);
                composed.set(ix, iy, iz, shifted_scan_sd.max(cap_sd));
            }
        }
    }
    marching_cubes(&composed, &MarchingCubesConfig::at_iso_value(0.0))
}
```

**Tests** (add to `sdf_layers::tests`):

- `extract_pinned_floor_cube_cavity_has_flat_floor` — unit cube
  scan, single cap on bottom face, `offset_m = -0.005`. Extract,
  assert (1) mesh is closed (Euler characteristic χ = 2 for a
  closed sphere/box topology), (2) bottom vertices all sit at the
  cap plane within numerical tolerance, (3) signed volume is within
  ~1 % of analytical `(0.99)² × 0.495` (depth = 1.0 - 0.005 inset
  on top + 0.005 on cap-plane side closure).
- `extract_pinned_floor_cube_outer_grown_has_flat_floor` — same
  setup but `offset_m = +0.005`. Assert closed mesh, flat floor at
  cap plane, signed volume within ~1 % of `1.01² × 1.005`.
- `extract_pinned_floor_no_caps_fast_path_byte_identical` — unit
  cube scan, empty `cap_planes`. Build cache, extract at
  `offset_m = ±0.005`. Assert mesh vertices + faces match what the
  pre-pinned-floor code path would produce (regression sentinel for
  the no-caps fast path's byte-identical behavior). Pre-pinned-floor
  expected mesh can be checked in as a small golden file or computed
  via a separate `marching_cubes(&cache.grid, &MarchingCubesConfig::at_iso_value(...))`
  call.
- `extract_pinned_floor_multi_cap_closes_both_floors` — unit cube
  scan, 2 caps (bottom face + top face). Extract at `offset_m = -0.003`.
  Assert mesh has TWO flat floors (one at each cap plane), is
  closed, volume ≈ analytical `0.994² × 0.994`.
- `extract_pinned_floor_cube_taubin_drift_robustness` — same cube
  test but slight Taubin-smoothed cap vertices (drifts ~1 mm off
  plane). Reuse the existing `dome_wall_only_mesh` face-normal
  classification — the test exercises that the cap classification
  STAYS robust to the same Taubin drift the cavity-mouth arc fixed.

**Acceptance criteria**:
- Existing no-caps tests pass byte-identical (no-caps fast path
  short-circuits to pre-pinned-floor code).
- New caps tests pass.
- `cargo test -p cf-device-design --lib` clean.
- `cargo clippy -p cf-device-design --all-targets` clean.

**LOC delta**: ~80 net (removes ~5 LOC of post-MC clip, adds ~30
LOC of per-cell composition, plus ~50 LOC of new tests).

### Sub-leaf 2 — Revert `signed_volume_m3` callers to world origin; delete `primary_cap_origin`

**File**: `tools/cf-device-design/src/main.rs`

**Change**:
- Drop `signed_volume_m3`'s `origin: Point3<f64>` parameter — restore
  the pre-cavity-mouth signature `signed_volume_m3(mesh: &IndexedMesh) -> f64`
  with internal origin = world origin. (Closed shells are
  divergence-origin-invariant — proven by the existing test
  `signed_volume_m3_origin_invariant`.)
- Delete `primary_cap_origin` (≈15 LOC) and its three tests
  (`primary_cap_origin_no_caps_is_world_origin`,
  `primary_cap_origin_picks_largest_by_vertex_count`,
  `primary_cap_origin_ties_break_on_lowest_loop_index`).
- Update `compute_validations`: drop the `volume_origin =
  primary_cap_origin(cap_planes)` line; pass nothing to
  `signed_volume_m3`.

**Diff sketch**:

```rust
// BEFORE (main.rs:829-862):
fn signed_volume_m3(mesh: &IndexedMesh, origin: Point3<f64>) -> f64 {
    let o = origin.coords;
    let mut six_volume = 0.0_f64;
    for face in &mesh.faces { ... a.dot(&b.cross(&c)) ... }
    (six_volume / 6.0).abs()
}

fn primary_cap_origin(cap_planes: &sdf_layers::CapPlanes) -> Point3<f64> {
    cap_planes.planes.iter().max_by(...).map_or(Point3::origin(), |p| p.centroid)
}

// AFTER:
fn signed_volume_m3(mesh: &IndexedMesh) -> f64 {
    let mut six_volume = 0.0_f64;
    for face in &mesh.faces {
        let a = mesh.vertices[face[0] as usize].coords;
        let b = mesh.vertices[face[1] as usize].coords;
        let c = mesh.vertices[face[2] as usize].coords;
        six_volume += a.dot(&b.cross(&c));
    }
    (six_volume / 6.0).abs()
}
// primary_cap_origin gone.
```

**`compute_validations` update**:

```rust
// BEFORE (main.rs:900-922):
let volume_origin = primary_cap_origin(cap_planes);
let cavity_mesh = sdf_layers::extract_layer_surface(...);
let mut prev_inner_volume = signed_volume_m3(&cavity_mesh, volume_origin);
// ...
let outer_volume = signed_volume_m3(&outer_mesh, volume_origin);

// AFTER:
let cavity_mesh = sdf_layers::extract_layer_surface(...);
let mut prev_inner_volume = signed_volume_m3(&cavity_mesh);
// ...
let outer_volume = signed_volume_m3(&outer_mesh);
```

**Tests**:
- Update existing volume tests that called `signed_volume_m3(&mesh, Point3::origin())`
  to drop the second arg.
- Update `signed_volume_m3_origin_invariant` test → becomes
  redundant (the function no longer takes origin); convert to a
  CLOSED-shell test that verifies the integral is bit-identical
  whether the mesh is translated by `(1, 1, 1)` or not. Keep this
  as the origin-invariance regression sentinel.
- Delete the three `primary_cap_origin_*` tests.

**Acceptance criteria**:
- `cargo test -p cf-device-design --lib` clean.
- `cargo clippy -p cf-device-design --all-targets` clean.
- `compute_validations` cube + cylinder analytical-fixture tests
  pass with bit-identical pre-cavity-mouth-arc numbers (the
  divergence integral was origin-invariant for closed cube + cylinder
  fixtures; the cap-centroid trick added a tiny numerical drift on
  open meshes that's now gone).

**LOC delta**: ~50 net deletion (≈15 LOC primary_cap_origin + ≈40
LOC its tests + ≈5 LOC origin-parameter plumbing − ≈10 LOC test
adjustments).

### Visual gate (NOT a sub-leaf — the ship point)

User opens `~/scans/sock_over_capsule.cleaned.stl` in cf-device-design
post-sub-leaf-2. Confirms by eye:

1. **Cavity** has flat floor pinned at cap plane (the load-bearing
   visual gate — the original cavity-mouth visual revealed the
   wrong-fit; this gate proves the new code matches the user model).
2. **Outer layers** are closed shells with flat floors at cap plane.
3. **Layer-stack preview** looks reasonable (no layer "skirt"
   sticking out past the cap plane, no missing floors, no broken
   topology).
4. **Validations panel** mass numbers are sane (cube + cylinder
   analytical fixtures match pre-cavity-mouth-arc bit-exact numbers).

If iter-1 reveals the model is wrong somewhere, this is the
falsification gate — work resumes from a new bookmark, NOT this
spec.

If iter-1 passes the gate, the cf-device-design preview is unblocked
and Followups F2-F6 become eligible for separate arcs.

---

## 3. Test plan

### Unit (per sub-leaf, listed above)

- 5 new tests in `sdf_layers::tests` (sub-leaf 1).
- 1 updated test, 3 deleted tests in `main::tests` (sub-leaf 2).

### Integration

Confirm cf-device-design end-to-end test (the analytical-fixture
cube preview test) passes bit-exactly with pre-cavity-mouth-arc
numbers. No new integration tests.

### Visual gate (user-driven, post-sub-leaf-2)

Iter-1 sock fixture, per §2 "Visual gate" section above. User-side;
Claude cannot verify (no GUI visibility).

### Regression

- `cargo test -p cf-device-design --lib` — 146 tests baseline → ~148
  expected (drop 3 primary_cap_origin tests + 1 test arg-update; add 5
  new pinned-floor tests).
- `cargo clippy -p cf-device-design --all-targets` clean.
- `cargo run -p xtask -- grade cf-device-design` ≥ A.
- `cargo build -p cf-device-design --release` clean (release path is
  what the user runs).

---

## 4. Acceptance criteria

- [ ] `extract_layer_surface` produces CLOSED manifold output with
      flat floor(s) at every included cap plane for `offset_m` in
      `[-LAYER_GRID_MARGIN_M, +LAYER_GRID_MARGIN_M]`.
- [ ] No-caps fast path is byte-identical to pre-pinned-floor code.
- [ ] `signed_volume_m3` reverts to single-argument form; closed-
      shell `compute_validations` numbers match pre-cavity-mouth-arc
      bit-exact on cube + cylinder analytical fixtures.
- [ ] All existing tests pass unchanged (except the 3 deleted
      `primary_cap_origin_*` and the 1 updated origin-invariance
      test).
- [ ] New unit tests for closed cavity, closed outer-grown, no-caps
      fast path, multi-cap, Taubin-drift robustness pass.
- [ ] `cargo test -p cf-device-design --lib` + `cargo clippy -p
      cf-device-design --all-targets` + `cargo build -p cf-device-design
      --release` all clean.
- [ ] **User visual gate** on iter-1 sock fixture: cavity has flat
      floor pinned at cap plane; outer layers are closed shells with
      flat floors at cap plane.

---

## 5. Banked followups (out of scope for v1)

- **F1. Extract `pinned_floor_shell` primitive to cf-design** as a
  generic `pub fn pinned_floor_shell(dome_wall_signed_sdf: impl Sdf,
  cap_planes: &[(Point3, Vector3)], offset_m, bounds: Aabb) -> Solid`.
  Consumers (insertion_sim, cf-cast-cli) pass a
  `DomeWallSignedSdf { closed: Arc<SDF>, open: Arc<SDF> }` adapter
  implementing `Sdf`. Requires pulling `CapPlane` parser +
  `dome_wall_only_mesh` from cf-device-design into cf-design (or a
  new `cf-scan-prep-shared` crate). Estimated ~200 LOC + tests +
  call-site plumbing. Unblocks F2 + F3.
- **F2. insertion_sim consumes pinned-floor geometry via F1**.
  Replace `Solid::from_sdf(scan_sdf, bounds).offset(cavity_offset_m)`
  with `pinned_floor_shell(...)`. Re-run the row-23 + cf-device-design
  ramp regressions to confirm pinned-floor cavity changes the
  intruder-cavity contact behavior in the expected direction.
  Required for slice-7 penetration sim correctness on real body-part
  scans with cap plane(s).
- **F3. cf-cast-cli consumes pinned-floor geometry via F1**. Plug
  + per-layer body construction switches to pinned-floor. Workshop
  iter-2+ mold STLs will have flat-floor cavity (vs today's
  uniform-shrink dome-poking-through). Requires updating
  `tools/cf-cast-cli/src/derive.rs` + threading cap-planes through
  the config schema (new `.cast.toml` field or implicit from
  `.design.toml` + `.prep.toml`).
- **F4. Fit-viz rungs 2-6 re-enabled**: per-step playback, scan-as-
  intruder, pressure scoring, retraction + over-cycle score,
  auto-search. These consume the cf-device-design preview's CLOSED
  cavity mesh from sub-leaf 1; unblocked once v1 lands.
- **F5. Multi-cap iter-N visual gate**: when a 2-cap body-part scan
  is in hand (forearm: wrist + elbow), confirm the dual-pinned-floor
  shells render correctly. Code-wise already supported by the fold-
  over-caps composition; this is a confirmation pass, not a code
  change.
- **F6. Cleanup pass**: delete `clip_mesh_against_cap_plane` if no
  consumer surfaces in F1-F5. Optimize `extract_layer_surface`'s
  per-cell composition in-place (no grid clone) if profiling
  surfaces a real cost. Delete `dome_wall_only_mesh`'s
  `report_cap_face_classification` diagnostic if the regression
  sentinel turns out unused after 2 iters.

---

## 6. Patterns predicted to bank from implementation

- **SDF-level CSG via per-extract grid composition**: the pattern
  of "cache the expensive SDF source; compose cheap per-extract
  overlays at extract time" lets a single fill serve many extraction
  variants. Reusable wherever an SDF needs to be intersected /
  united with a cheap analytical SDF (half-spaces, spheres, etc).
- **Fold-over-half-spaces with `max`** as a multi-cap composition
  primitive. Order-independent, naturally handles N caps with no
  special-casing for the multi-cap case.
- **MC closure of an SDF intersection produces naturally-stitched
  flat polygons** at the intersection planes — no custom
  triangulation needed, no boundary-loop detection, no fan
  triangulation logic. The triangulator inherits its quality from
  marching cubes.
- **The two-SDF construction (sign-from-closed × magnitude-from-open)
  is the natural way to represent a "body inset INWARD from a SUBSET
  of the surface" SDF**. The cavity-mouth arc shipped it for OPEN
  shells; the pinned-floor arc retains it as the dome-wall-signed-
  distance field and composes the cap-plane intersection on top. The
  two-SDF generalization (sign from one source, magnitude from
  another) is a reusable idiom for any "treat parts of the surface
  differently" SDF need.

---

## 7. Open risks

- **MC iso-0 sign-tie behavior at the cap plane**. mesh-sdf's
  `distance` heuristic has a `>= 0.0` branch at sign assignment
  (see `sdf_layers.rs::MARGIN_OFFSET_M` doc comment for the prior
  workaround). The composed `max(scan_sd - offset, cap_sd)` field
  produces an EXACTLY-zero cell-corner value on the cap plane
  whenever `scan_sd >= offset`. MC's behavior on exact-zero
  corners can produce phantom-needle fragments. The existing
  fixture-offset technique (3 × 1.7 mm fixture translation, 43 mm
  grid margin) was the workaround for closed-body fixtures with
  axis-aligned faces. The pinned-floor extraction may surface a
  NEW phantom-needle case at the cap plane on the iter-1 sock
  fixture (whose cap plane isn't axis-aligned, so it MAY avoid the
  exact-zero tie, but the body-interior corners adjacent to the
  cap plane are subject to the modulated-distance sign branch).
  **Mitigation**: if visual gate surfaces phantom needles at the
  cap plane, replace `cap_sd` with `(p - centroid).dot(normal) +
  CAP_SD_TIE_BREAK_M` (~1e-9) to nudge the cap-plane half-space
  inward by a sub-numerical-tie amount. Banked the resolution
  rather than implementing eagerly; tackle if the visual gate
  reveals it.

- **`mesh-sdf`'s far-field sign heuristic on the dome-wall-only mesh**
  (per cavity-mouth spec §1 Q2 + insertion_sim's `build_grid_sdf`
  precedent — sign was ~12% wrong on the sloppy-decimated open
  scan). For the cf-device-design preview, the cached grid only
  evaluates SDF inside the body AABB + 40 mm margin, where mesh-sdf's
  heuristic was reliable enough on iter-1 (the cavity-mouth visual
  gate passed). The pinned-floor extraction adds a per-cell cap_sd
  composition but doesn't re-query mesh-sdf, so this risk is
  unchanged from cavity-mouth ship. **Mitigation**: not needed at
  v1 of pinned-floor; if iter-2+ surfaces it, switch the cf-device-
  design preview's SDF source from `mesh-sdf::SignedDistanceField`
  to insertion_sim's `build_grid_sdf` (flood-fill, robust sign).
  Bigger arc; bank for iter-2.

- **`extract_layer_surface`'s per-extract `cache.grid.clone()`** —
  iter-1's grid is ~20 k cells (~160 kB f64); cloning every frame
  at 60 Hz is ~10 MB/s allocator churn. Bevy's frame-time budget
  has ~16 ms per frame; per-extract cost is ~1.5 ms including the
  clone (per Q2 cost estimate). Acceptable for v1. If profiling
  surfaces a real cost (e.g. iter-2 grids with finer pitch), switch
  to in-place composition with a per-cell `read → write` scan (no
  clone). Banked as F6.

---

## 8. Decision: spec-only this session

This session produces ONLY this spec doc + a `git commit
docs(cf-device-design): recon → spec` commit. NO code, NO experiments,
NO one-off scripts. Per [[feedback-autonomous-architecture]] +
[[feedback-bookmark-when-surface-levers-exhaust]] three-session
pattern.

NEXT SESSION = implementation per the §2 ladder. Cold-read entry
point: this spec, then `sdf_layers.rs::extract_layer_surface` +
`main.rs::signed_volume_m3` + `main.rs::compute_validations`. The
existing two-SDF cached-grid path stays UNCHANGED.

---

## 9. Decisions log (load-bearing recon outputs)

1. **Pinned-floor primitive is per-extract SDF composition**, NOT
   anisotropic offset or post-MC stitching. (§1 Q2)
2. **Two-SDF cached grid stays** as the dome-wall-signed-distance
   field source. (§1 Q5)
3. **`primary_cap_origin` + cap-centroid-origin volume integral
   trick gets deleted** — closed shells are origin-invariant. (§1 Q5)
4. **v1 stays inside cf-device-design**; cf-design extraction is
   Followup F1. (§1 Q3)
5. **Insertion sim + cf-cast-cli don't change in v1** — they
   compose at SDF/`Solid` level, never consume the
   `extract_layer_surface` triangle mesh. F2 + F3 plumb them to
   pinned-floor geometry separately. (§1 Q8)
6. **Multi-cap is automatic** via fold-over-caps with `max`. iter-1
   single-cap + iter-N two-cap use the same code path. (§1 Q7)
