# cf-device-design Cavity Mouth Opening — Spec

**Status**: SHIPPED 2026-05-16 (5 commits on dev, `03bb695f` →
`a403a4f3`; 144 tests / 9 ignored; clippy clean; release-build clean).
User visual gate on iter-1 sock fixture pending.
**Parent**: [`CF_DEVICE_DESIGN_CAVITY_MOUTH_BOOKMARK.md`](CF_DEVICE_DESIGN_CAVITY_MOUTH_BOOKMARK.md).
**Followup** (not blocking fit-viz): [`CF_DEVICE_DESIGN_INSERTION_SIM_OPEN_CAVITY_BOOKMARK.md`](CF_DEVICE_DESIGN_INSERTION_SIM_OPEN_CAVITY_BOOKMARK.md).

## As-built (post-2026-05-16) — read this first

Shipped 2026-05-16, then the mesh-sdf oracle-decomposition **D-arc**
(which post-dates this spec) plus the D.2 flood-fill grid moved the API
and the grid-fill mechanism out from under the Sub-leaf 3 sketch below.
The current code is the source of truth; this records the divergences
so the spec stays a faithful shipped record. The two-SDF *idea* (§1 Q2)
is unchanged — closed body for sign, cap-stripped open body for
unsigned magnitude — the drift is in types and the sign source.

- **Location + SDF type.** The `sdf_layers` module moved
  `tools/cf-device-design` → `design/cf-device-geometry`
  (`src/sdf_layers.rs`). The `mesh_sdf::SignedDistanceField` named below
  no longer exists — the D-arc replaced it with the composed
  `Signed<TriMeshDistance, PseudoNormalSign>`.
- **As-built `CachedScanSdf`.** The Sub-leaf 3 struct
  `{ sdf_closed, sdf_open: Arc<SignedDistanceField>, grid, bounds,
  min_sdf_value }` shipped as `{ sdf_closed, sdf_open:
  Arc<Signed<TriMeshDistance, PseudoNormalSign>>, closed_grid,
  open_grid: Option<ScalarGrid>, bounds, margin_m, min_sdf_value }`.
- **Grid sign source (the real change).** The Sub-leaf 3 fill computes
  `sd_closed.signum() * open_unsigned` from the pseudo-normal
  `sdf_closed.distance(p)`. The shipped code fills `closed_grid` via
  mesh-sdf's `CachedGridSdf::build` (3-region **flood-fill** sign),
  because pseudo-normal sign flips far-field on cleaned scans; the
  `sdf_closed` field is retained only for sign-independent closest-point
  projection (heat-map re-projection), NOT for the grid sign.

## Implementation summary

5 sub-leaves shipped one commit at a time:

| Sub-leaf | Commit       | What                                                                                  |
| -------- | ------------ | ------------------------------------------------------------------------------------- |
| 1        | `03bb695f`   | `[caps]` parser + `CapPlane` / `CapPlanes` Bevy resource; `[transform]` bake (+5 tests). |
| 2        | `cb03b144`   | `dome_wall_only_mesh` helper (`CAP_FACE_PLANARITY_EPS_M=1e-6`) (+5 tests).            |
| 3        | `544a8905`   | Two-SDF `build_cached_scan_sdf`; `Arc::clone` no-caps fast path (+4 tests).           |
| 4        | `5ad09176`   | Post-MC Sutherland-Hodgman half-space clip; thread `cap_planes` into every caller (+6 tests). |
| 5        | `a403a4f3`   | Cap-centroid-origin `signed_volume_m3` + `primary_cap_origin` (+6 tests).             |

Total: 26 new tests; no regressions on the no-caps path.

## Acceptance criteria

- [x] cf-device-design parses `[caps]` and bakes `[transform]`.
- [x] `build_cached_scan_sdf` builds two SDFs when caps present;
      one when not.
- [x] `extract_layer_surface` clips against every `included` cap
      plane.
- [x] `signed_volume_m3` accepts an integration origin; default
      callers pass the primary cap centroid (or world origin).
- [x] All existing tests pass unchanged on the no-caps path.
- [x] New unit tests for dome_wall_only_mesh, two-SDF construction,
      half-space clip, and cap-origin volume integral pass.
- [ ] **User visual gate** on iter-1 sock fixture pending.
- [ ] Grid-fill cost on iter-1 < 1 s — predicted ~648 ms, will
      observe at visual gate.
- [x] No regression in `compute_validations` numbers vs current on
      the closed-body analytical fixtures.

## Patterns banked from this arc

- **Two-SDF construction pattern.** Closed-body SDF for sign, open-
  body SDF for magnitude. Reusable wherever a "subtract a virtual
  face from the SDF" operation is needed — e.g., the same trick can
  open arbitrary holes on top of body parts (deferred multi-hole
  rim feature).
- **Cap-centroid divergence-volume integral.** Origin-translated
  divergence theorem for open surfaces. Reusable in any future shell-
  volume computation where the shell is bounded by a polygon.
- **`Arc::clone` short-circuit for absent-feature fast path.** Sharing
  the Arc between `sdf_closed` and `sdf_open` when no cap planes are
  present avoids paying the second SDF build cost. Pattern: feature-
  gated arc clone + per-cell `if cap_planes.is_empty()` short-circuit.
- **Sutherland-Hodgman triangle clip.** Standard 4-case
  (k=0/1/2/3 vertices inside) Sutherland-Hodgman implementation with
  cyclic edge walk + fan triangulation from index 0. Reusable
  whenever a triangle mesh needs to be clipped against a half-space.
- **Pre-bake → post-bake transform recovery from `.prep.toml`.**
  cf-scan-prep emits the user's `[transform]` (rotation quaternion
  + translation) and records geometric features in the pre-bake
  frame. Downstream consumers can recover post-bake coordinates by
  applying the same rotation + translation; pivot = origin is an
  acceptable approximation when the working scan AABB centroid sits
  at origin (which auto-center delivers to sub-cm).

## Lessons during implementation

(Items that diverged from the spec or surfaced edge cases.)

- **Spec called for `pub(crate) const CAP_FACE_PLANARITY_EPS_M`,
  implementation went private.** Only `sdf_layers.rs` uses it; no
  cross-module exposure needed.
- **Spec's `extract_layer_surface(cache, cap_planes, offset_m)`
  signature touched 17 call sites** (11 in sdf_layers tests, 2
  production + 4 test in main.rs). The `&[]` empty-slice pattern
  for the no-caps path kept the test sweep simple (sed swap).
- **`signed_volume_m3` got a sibling helper.** Spec sketched
  `compute_validations` inlining the origin pick; pulling it into
  `primary_cap_origin(cap_planes)` made the multi-cap tiebreaker
  (vertex_count → loop_index) testable independently.
- **Pivot caveat documented in code.** The bake formula uses
  pivot = origin, not the working-scan AABB centroid (which
  `.prep.toml` does not persist). Recorded in `parse_cap_planes`'s
  docstring; behaviour is correct for identity transforms (the
  common case) and within sub-mm for typical manual reorients
  after auto-center.

---



The user-pinned approach from the bookmark is preserved verbatim:

1. cf-scan-prep stays untouched.
2. cf-device-design holds each iso surface at the cap plane.
3. Knife-edge brim acceptable for v1.

Recon turned up one structural surprise that drove the implementation
shape: **post-MC half-space clip alone does not open the cavity**.
The cavity iso surface for `iso = -T` sits entirely in the body-
interior half-space and never crosses the cap plane, so clipping
against the cap plane discards nothing. The cavity floor disappears
only if the cap polygon is removed from the SDF source. The spec is
built around a two-SDF construction that delivers this without
modifying cf-scan-prep.

---

## 1. Recon answers

### Q1: Multi-cap-loop behavior

**Resolution**: clip independently against every `included = true`
cap plane. A triangle is kept iff it survives every per-plane test.

- Iter-1 (sock-over-capsule) has 1 cap loop.
- Body-part scans may have 2 (residual limb proximal + distal), rarely
  more.
- Pathological collision (two half-spaces eating the whole device) is
  geometrically possible only when cap planes face each other across
  the body — handled by an empty-mesh early-out with a `warn!` line
  rather than an explicit pre-check. Cheap to detect; we already
  early-out on empty extractions for the cavity-collapse case.

The `[caps]` `included` flag is the only gate. Excluded loops do not
clip; their region of the iso surface stays closed (consistent with
"the user said leave this hole as-is").

### Q2: Pre-MC vs post-MC clip — **two-SDF construction required**

The bookmark sketch listed pre-MC zero-out, post-MC half-space cull,
and a hybrid. None of those open the cavity by themselves because:

> The cavity iso surface for `iso = -T` (T > 0) sits entirely in the
> body-interior half-space. Its "floor" — the inward-offset of the
> cap polygon — is at axial position +T meters from the cap plane in
> the body-interior direction (outward-normal-NEGATIVE direction).
> Clipping triangles whose centroid satisfies `dot(p − cap_centroid,
> outward_normal) > 0` discards the OUTER iso's floor (which lives
> at axial position −T_N, cap-extension side) but leaves the cavity
> floor intact.

What actually works: **remove the cap polygon from the SDF source so
the cavity iso has no floor to begin with.**

But mesh-sdf computes signed distance from a closed-manifold winding;
stripping the cap leaves an open mesh whose sign is undefined. The
escape: build two SDFs and combine.

**Two-SDF construction**

```
SDF_closed = SignedDistanceField::new(cleaned_mesh)        // full body, cap included
SDF_open   = SignedDistanceField::new(dome_wall_only_mesh) // cap faces stripped

sd_mod(p) = sign(SDF_closed.distance(p)) * SDF_open.unsigned_distance(p)
```

Why this works:

- **Sign** comes from the closed body, so points outside the body
  laterally (e.g., far above the dome, or beside the wall) get the
  correct positive sign. The half-space-from-cap-plane sign convention
  fails here — it would mark all body-side points negative even when
  they are outside the body shape.
- **Magnitude** comes from the open body's UNSIGNED distance. The cap
  polygon does not contribute, so the cavity iso surface terminates
  naturally where the dome+wall reaches the cap plane — there's no
  cavity floor because there's no cap face to be inward-offset from.
- Continuous everywhere, including across the cap plane (sd_mod = 0
  at the cap polygon centroid because both SDFs agree there).

**Cost**: 2× the grid fill (one query per SDF per cell). On iter-1
(324 ms → ~648 ms one-time at scan load). Acceptable.

**Outer-iso behavior**: the outer iso at `iso = +T_N` still extends
PAST the cap plane on the cap-extension side. The dome+wall-only
distance field is large in that half-space (distance to the cap-edge
ring of the open mesh), so the outer iso surface DOES eat into that
region — it forms a "skirt" going some distance past the cap plane.
We clip this off with a post-MC half-space clip against each cap
plane. With sd_mod's continuity, the clip boundary lands cleanly on
the cap plane.

So the unified pipeline is:

```
fill grid with sd_mod
per iso:
    mc_mesh = marching_cubes(grid, iso)
    for each cap plane:
        mc_mesh = clip_against_half_space(mc_mesh, cap_plane)
    return mc_mesh
```

For the cavity (iso < 0) the half-space clip is usually a no-op
(cavity wall already terminates at the cap plane via the open-SDF
construction). For the outer (iso > 0) the clip cuts off the skirt.
Uniform handling — no per-iso branching.

**Identifying cap faces** (for `dome_wall_only_mesh`):

cf-scan-prep's `auto_cap_open_boundaries` (`tools/cf-scan-prep/src/
main.rs:1228`) projects boundary vertices onto the fit plane *before*
ear-clipping, so cap face vertices sit EXACTLY on the cap plane
(modulo float roundoff from the projection). The detection rule:

```
A face is a cap face iff, for ANY `included` cap plane,
    all three vertices satisfy
        |dot(v - cap_centroid, cap_normal)| < CAP_FACE_PLANARITY_EPS_M
```

`CAP_FACE_PLANARITY_EPS_M = 1e-6` matches the existing weld epsilon
(`SDF_WELD_EPSILON_M`). The projection in cf-scan-prep produces
vertices that are bit-exactly on the plane modulo a few float ULPs,
so 1 µm is generous. We can re-tune if iter-2 / iter-3 scans surface
false negatives (a dome face that happens to lie ε-near a cap plane).

### Q3: Divergence-theorem on open surfaces

The current `signed_volume_m3` (`tools/cf-device-design/src/main.rs:
677`) uses

```
V = (1/6) Σ_triangles  a · (b × c)
```

For a CLOSED mesh this is the signed volume. For an OPEN mesh, it
computes the volume of the closed shape formed by attaching the
boundary to the ORIGIN — NOT to the geometric cap polygon.

Math: each triangle's `a · (b × c)` is the signed volume of the
tetrahedron `(origin, a, b, c)`. Summing over a closed surface
cancels internal contributions and leaves the enclosed volume. For
an open surface, the "missing" piece is the contribution from the
cone connecting the boundary to the origin.

**Fix**: integrate around the cap centroid instead of the world
origin. For a single cap plane:

```
V = (1/6) Σ_triangles  (a - cap_centroid) · ((b - cap_centroid) × (c - cap_centroid))
```

When all three vertices of a triangle lie on the cap plane through
the cap centroid, `a · (b × c)` collapses to zero — so the implicit
"cap-polygon contribution" vanishes and V equals the volume bounded
by the open surface plus the cap polygon at the cap plane.

For MULTIPLE cap planes: pick the primary cap (largest by vertex
count, or `included` loop with lowest `loop_index`) as the integration
origin. The other cap polygons contribute small errors proportional to
their distance from the chosen origin × their area. For typical
body-part scans (proximal + distal cuts separated by ~10–50 cm) this
is meaningful — the validations panel readouts would diverge a few
percent from truth on multi-mouth scans.

A more precise variant: explicitly add each excluded cap polygon's
divergence contribution. Implementation deferred to v2 (post iter-2
multi-mouth scan when we have a real fixture). v1 takes the primary-
cap-origin shortcut; flag in code comment.

### Q4: Cap-edge mesh quality

The post-MC half-space clip produces triangle edges lying on the cap
plane. Each clipped triangle is split into 0/1/2 child triangles
using barycentric interpolation along the cut edges. Standard half-
space mesh clipping — no jaggedness if the algorithm is correct.

The boundary ring is NOT re-triangulated against a "perfect" cap
polygon — it's just the set of cut-edge segments, in whatever order
the clip emits them. For v1 this is acceptable (the boundary is
visible only at the knife-edge brim, which the user has explicitly
called a v2 polish target).

Deferred to v2: boundary-ring re-projection + re-stitching to a
perfect circle / polygon. Not blocking.

### Q5: Slice-7 insertion sim dependency check

**Found a real but bounded issue.** The sim has its own SDF pipeline
(`build_grid_sdf` → `Solid::offset/subtract`, in
`tools/cf-device-design/src/insertion_sim.rs`) and does NOT consume
the cf-device-design preview's `CachedScanSdf`. So shipping the
cavity-mouth feature in the preview does not break the sim build.

But the sim is currently calibrated for a CLOSED-cavity, full-
surface-engagement scenario:

- `INSERTION_CONTACT_KAPPA = 1.0e3` (one-tenth the default `1e4`).
  Docstring at `insertion_sim.rs:857-865`: "the *whole* cavity wall
  engages at once, unlike the rows' localized probe". Explicit pin
  of the closed-cavity assumption.
- The rigid intruder is a copy of the scan SDF offset by
  `interference_m + cavity_offset_m`. With a closed cavity, the
  intruder presses into a sealed dome and contact is full-surface
  from `interference_m = 0`. With an open cavity, contact is
  progressive (intruder enters mouth, contacts dome last).

Implications:

- Current sim results are physically meaningless against the v1.0
  device geometry. Were always — predates this spec.
- Fixing requires updating `build_insertion_geometry` to use the
  open-cavity body. `Solid::subtract` doesn't directly support
  half-space clip; would need a custom `body = outer.subtract(cavity)
  .intersect(below_cap_plane)` or equivalent.
- Re-tuning `INSERTION_CONTACT_KAPPA` (progressive contact may
  tolerate the default `1e4`) and the trajectory anchoring.

**Decision**: tracked as a separate followup arc, NOT part of this
spec. The cavity-mouth feature ships, restoring the preview's
physical truth. The sim's broken state is pre-existing; opening the
cavity makes it more visible (panel readouts will look "off" against
the now-open preview) but doesn't make it worse. Bookmark a new
followup at session end: `docs/CF_DEVICE_DESIGN_INSERTION_SIM_OPEN_
CAVITY_BOOKMARK.md`. Not blocking the fit-viz program rungs 2-6.

---

## 2. Implementation ladder

**Estimate**: 5 sub-leaves, ~700 LOC + tests. Three-session pattern
calls for spec-only this turn, implementation next.

### Sub-leaf 1 — `[caps]` schema parser + `CapPlanes` resource

**Files**: `tools/cf-device-design/src/main.rs` (parser + resource +
`build_scan_info` wiring).

- Extend `PrepTomlSubset` to deserialize a `caps` block matching
  `PrepCapsBlock`. Reuse the existing `centerline_only` posture —
  treat missing block as "no caps".
- Add a `CapPlanes` resource (`Vec<CapPlane>`) holding post-bake cap
  planes. `CapPlane { centroid: Point3<f64>, normal: Vector3<f64>,
  vertex_count: usize, loop_index: usize }`. Only `included == true`
  loops are kept.
- Bake `[transform]` (the rotation quaternion + translation in meters)
  onto each cap plane at load time. cf-scan-prep already does this
  for the cleaned STL but emits cap metadata in the PRE-bake frame
  (`PrepCapLoop.plane_normal/plane_centroid_m` docstring at
  `cf-scan-prep/src/main.rs:3426-3429`).
- Insert `CapPlanes` into the Bevy app via `app.insert_resource`
  alongside `CachedScanSdf`.

LOC: ~120 (parser + resource + test).

**Tests**: round-trip a small `.prep.toml` with one cap loop + a 90°
rotation; verify the baked plane normal/centroid are correctly
rotated.

### Sub-leaf 2 — `dome_wall_only_mesh` helper

**Files**: `tools/cf-device-design/src/sdf_layers.rs`.

```rust
const CAP_FACE_PLANARITY_EPS_M: f64 = 1e-6;

/// Strip cap-polygon faces from `cleaned_mesh`. A face is a cap face
/// iff its three vertices all lie within CAP_FACE_PLANARITY_EPS_M of
/// any cap plane. Leaves the vertex array as-is (no compaction —
/// SDF construction tolerates unreferenced vertices).
fn dome_wall_only_mesh(cleaned_mesh: &IndexedMesh, cap_planes: &[CapPlane]) -> IndexedMesh {
    if cap_planes.is_empty() {
        return cleaned_mesh.clone();
    }
    let kept_faces = cleaned_mesh.faces.iter().filter(|face| {
        let v: [Point3<f64>; 3] = [
            cleaned_mesh.vertices[face[0] as usize],
            cleaned_mesh.vertices[face[1] as usize],
            cleaned_mesh.vertices[face[2] as usize],
        ];
        !cap_planes.iter().any(|plane| {
            v.iter().all(|vi| {
                (vi.coords - plane.centroid.coords).dot(&plane.normal).abs()
                    < CAP_FACE_PLANARITY_EPS_M
            })
        })
    }).copied().collect();
    IndexedMesh { vertices: cleaned_mesh.vertices.clone(), faces: kept_faces }
}
```

LOC: ~50.

**Tests**:

- Closed box with a known cap face flagged → that face removed, rest
  intact.
- Two co-planar cap loops on opposite sides of a cylinder → both sets
  of cap faces removed.
- Empty `cap_planes` → mesh unchanged (legacy path).
- Off-plane face that grazes the cap plane (single vertex within ε)
  → face KEPT (only all-3-vertices triggers removal).

### Sub-leaf 3 — Two-SDF `build_cached_scan_sdf`

**Files**: `tools/cf-device-design/src/sdf_layers.rs` +
`tools/cf-device-design/src/main.rs` (call-site).

```rust
pub(crate) struct CachedScanSdf {
    pub(crate) sdf_closed: Arc<SignedDistanceField>,
    pub(crate) sdf_open:   Arc<SignedDistanceField>,  // NEW; equals sdf_closed when no caps
    pub(crate) grid: ScalarGrid,
    pub(crate) bounds: (Point3<f64>, Point3<f64>),
    pub(crate) min_sdf_value: f64,
}

pub(crate) fn build_cached_scan_sdf(
    scan: &IndexedMesh,
    cap_planes: &[CapPlane],     // NEW
    cell_size_m: f64,
    margin_m: f64,
) -> Result<CachedScanSdf> {
    let decimated = decimate_scan_for_sdf(scan, SDF_SOURCE_TARGET_FACES);
    let sdf_closed = SignedDistanceField::new(decimated.clone()).context("...")?;
    let open_mesh = dome_wall_only_mesh(&decimated, cap_planes);
    let sdf_open = if cap_planes.is_empty() {
        sdf_closed.clone()   // shared Arc — fill loop's `if` short-circuits, no second query
    } else {
        SignedDistanceField::new(open_mesh).context("...")?
    };

    // ... allocate grid as today ...

    for iz, iy, ix {
        let p = grid.position(...);
        let sd_closed = sdf_closed.distance(p);
        let sd = if cap_planes.is_empty() {
            sd_closed
        } else {
            let sd_open_unsigned = sdf_open.unsigned_distance(p);
            sd_closed.signum() * sd_open_unsigned
        };
        grid.set(ix, iy, iz, sd);
    }
}
```

**Backwards-compat**: empty `cap_planes` → single-SDF fast path (no
second query, sd_mod = sd_closed). All existing tests pass unchanged.

`Arc<SignedDistanceField>` is `Clone`; sharing the same Arc avoids
allocating a second SDF when no caps are present.

LOC: ~80 (impl) + ~150 (tests).

**Tests**:

- **Analytical sphere** with a hemispherical cap (faces below z = 0
  flagged as cap) → cavity iso at `-T` extracts an OPEN bowl (no
  floor at z = T), wall vertices terminate at z ≈ 0. The existing
  `sphere_isosurface_inward_offset_lies_on_radius_one_minus_t` test
  becomes a regression check for the no-caps fast path.
- **Cube with one face removed** (top +z face flagged) → cavity iso
  opens at the top, dome+wall body offset inward.
- **Determinism** — two builds with identical inputs produce identical
  grids (existing `build_cached_scan_sdf_is_deterministic` test
  extended to cover the cap path).
- **Grid fill cost** — measure on iter-1 sock fixture, confirm < 1 s
  (target ≤ 700 ms).

### Sub-leaf 4 — Post-MC half-space clip in `extract_layer_surface`

**Files**: `tools/cf-device-design/src/sdf_layers.rs`.

```rust
pub(crate) fn extract_layer_surface(
    cache: &CachedScanSdf,
    cap_planes: &[CapPlane],   // NEW
    offset_m: f64,
) -> IndexedMesh {
    debug_assert!(...);
    let mut mesh = marching_cubes(&cache.grid, &MarchingCubesConfig::at_iso_value(offset_m));
    for plane in cap_planes {
        clip_against_half_space(&mut mesh, plane);
    }
    mesh
}
```

Half-space clip: for each triangle, compute signed distance of each
vertex to the cap plane (`dot(v − centroid, normal)`); classify as
inside (signed ≤ 0, body side) or outside (signed > 0, cap-extension
side). All-inside triangle → keep. All-outside → discard. Mixed →
split along the plane using barycentric interpolation on the crossed
edges. Standard algorithm; output is a clean polyline boundary at
the cap plane.

Call-site updates: `compute_validations` (`main.rs:716, 727`) and
`update_layer_meshes` (TBD line — slice-9 sub-leaf 4) thread
`cap_planes` through to every `extract_layer_surface` call.

LOC: ~100 (clip impl) + ~80 (caller updates) + ~120 (tests).

**Tests**:

- Triangle entirely above plane → kept verbatim.
- Triangle entirely below → empty.
- Triangle straddling plane (1 vertex above, 2 below) → 2 triangles
  forming a quad on the body side.
- Triangle straddling plane (2 above, 1 below) → 1 triangle on body
  side.
- Two cap planes producing intersecting clips on a sphere — clipped
  mesh has both boundary rings, no spurious cut-edge artifacts.

### Sub-leaf 5 — Cap-centroid-origin volume integral

**Files**: `tools/cf-device-design/src/main.rs`.

```rust
fn signed_volume_m3(mesh: &IndexedMesh, origin: Point3<f64>) -> f64 {
    let mut six_volume = 0.0_f64;
    for face in &mesh.faces {
        let a = mesh.vertices[face[0] as usize].coords - origin.coords;
        let b = mesh.vertices[face[1] as usize].coords - origin.coords;
        let c = mesh.vertices[face[2] as usize].coords - origin.coords;
        six_volume += a.dot(&b.cross(&c));
    }
    (six_volume / 6.0).abs()
}
```

`compute_validations` picks the primary cap centroid (or world origin
when no caps) as the integration origin:

```rust
let volume_origin = cap_planes.first().map(|p| p.centroid).unwrap_or(Point3::origin());
let cavity_volume = signed_volume_m3(&cavity_mesh, volume_origin);
```

LOC: ~30 (impl) + ~80 (test updates).

**Tests**:

- Unit cube centered at origin, integrated about origin → 1 m³
  (regression of existing test).
- Unit cube centered at origin, integrated about (1, 1, 1) → still
  1 m³ (translation-invariance for closed surfaces).
- Open bowl (hemisphere with cap loop at z = 0): integrate about
  (0, 0, 0) → ~(2π/3) (volume bounded by hemisphere + flat cap at
  z = 0).
- Same open bowl, integrate about (1, 1, 1) → DIFFERENT result
  (open-surface integral IS origin-dependent; confirms we must use
  the cap-centroid origin for correctness).
- Two open hemispheres sharing a cap polygon at z = 0 → V(outer) −
  V(inner) = shell volume between them, bounded at z = 0. Both
  integrated about the shared cap centroid.

**Cross-check**: re-run the `compute_validations_*` test suite with
the no-caps path (origin = world origin) — results must be bit-
identical to today.

---

## 3. Test plan

### Unit (per sub-leaf, listed above)

### Integration

- New test `cavity_opens_on_sock_capsule_fixture` (release-only,
  user-supplied fixture under `tests/fixtures/` — gated `#[ignore]`
  if the fixture file isn't committed):
  - Load `sock_over_capsule.cleaned.stl` + `.prep.toml`.
  - Build two-SDF cache with the parsed cap planes.
  - Extract cavity at inset 5 mm, layer 1 outer at 10 mm.
  - Assert: no triangle in either mesh has all 3 vertices within
    `CAP_FACE_PLANARITY_EPS_M × 100` of the cap plane on the body-
    interior side (no floor triangles survive).
  - Assert: at least N triangles have one vertex within
    `CAP_FACE_PLANARITY_EPS_M × 100` of the cap plane (the open ring
    is present).

### Visual gate (user-driven)

- `cf-device-design` opens the cleaned sock fixture.
- Click "Show cavity" + "Show layer 1 outer".
- Use the clip-plane slider (rung 1) to look INSIDE the cavity from
  below (cut from the cap-end direction).
- Confirm: the cavity is open at the bottom; you can see UP into the
  cavity dome from below the cap plane.
- Confirm: the outer skin terminates cleanly at the cap plane, no
  skirt extending past.
- Confirm: at the cap plane, the inner-cavity-wall ring and the
  outer-skin-wall ring are concentric, separated by the layer
  thickness (the knife-edge brim).

### Regression

- Run the existing `compute_validations_*` tests with no cap planes:
  bit-identical results to pre-spec (the no-caps path is the legacy
  path).
- Run `sphere_isosurface_outward_offset_*` and `cube_outward_offset_*`
  tests: unchanged (the analytical fixtures are closed bodies with
  no cap planes).
- Per-crate clippy + release test: `cargo test -p cf-device-design
  --bin cf-device-design --release` clean ([[feedback-cf-cast-tests-
  use-release]]).

---

## 4. Acceptance criteria

- [ ] cf-device-design parses `[caps]` and bakes `[transform]` onto
      cap planes at load.
- [ ] `build_cached_scan_sdf` builds two SDFs when cap planes are
      present; one SDF when not.
- [ ] `extract_layer_surface` clips against every `included` cap
      plane.
- [ ] `signed_volume_m3` accepts an integration origin; default
      callers pass the primary cap centroid (or world origin when no
      caps).
- [ ] All existing tests in `cf-device-design` pass unchanged on the
      no-caps path.
- [ ] New unit tests for `dome_wall_only_mesh`, two-SDF construction,
      half-space clip, and cap-origin volume integral pass.
- [ ] User visual gate on iter-1 sock fixture: cavity opens, outer
      skin terminates, knife-edge brim visible.
- [ ] Grid-fill cost on iter-1 < 1 s (target ≤ 700 ms).
- [ ] No regression in `compute_validations` numbers vs current on
      the closed-body analytical fixtures.

## 5. Banked followups (out of scope for v1)

- **Knife-edge brim smoothing.** Round the cross-section near the cap
  plane so wall thickness tapers smoothly. User said v2 polish.
- **Rim / lip slider.** Per-layer slider to extrude past the cap
  plane for grip/rigidity. Spec only when a workshop case needs it.
- **Slice-7 insertion sim open-cavity rework.** Separate bookmark at
  session end. Re-anchor intruder trajectory, re-tune
  `INSERTION_CONTACT_KAPPA`. Not blocking fit-viz rungs.
- **Boundary-ring re-projection.** Snap clip-boundary vertices onto a
  perfect circle / polygon at the cap plane. v2 quality polish.
- **Multi-cap divergence correction.** v1's volume integral picks one
  cap centroid; multi-mouth scans have small (~few %) volume error.
  Add per-cap divergence corrections when iter-2 surfaces this.
- **Persist parsed cap planes in `<scan>.design.toml`.** Today the
  cap planes are re-derived from `.prep.toml` each load; if the
  `.prep.toml` is missing, the device falls back to closed-cavity.
  Worth a separate scan_ref echo in `design.toml` for audit. Low
  priority.

## 6. Patterns predicted to bank from implementation

(To be confirmed at ship time. Listing here so the implementation
session can call them out.)

- **Two-SDF construction pattern.** Closed-body SDF for sign, open-
  body SDF for magnitude. Reusable wherever a "subtract a virtual
  face from the SDF" operation is needed — e.g., the same trick can
  open arbitrary holes on top of body parts (deferred multi-hole
  rim feature).
- **Cap-centroid divergence-volume integral.** Origin-translated
  divergence theorem for open surfaces. Reusable in any future shell-
  volume computation where the shell is bounded by a polygon.
- **`Arc::clone` short-circuit for absent-feature fast path.** Sharing
  the Arc between `sdf_closed` and `sdf_open` when no cap planes are
  present avoids paying the second SDF build cost. Pattern: feature-
  gated arc clone + per-cell `if cap_planes.is_empty()` short-circuit.

---

## 7. Decision: spec-only this session

5 sub-leaves, ~700 LOC + ~500 LOC of tests. Sub-leaf 3 (two-SDF
construction) is the structural pivot — it changes `CachedScanSdf`'s
shape, which ripples into every existing `extract_layer_surface` /
`compute_validations` call-site. Cap-face identification (sub-leaf 2)
and the volume integral fix (sub-leaf 5) both have edge cases worth
their own ship cycles.

Implementation lands NEXT session. This session: spec committed,
bookmark updated, MEMORY resume note rewritten for the
implementation phase.
