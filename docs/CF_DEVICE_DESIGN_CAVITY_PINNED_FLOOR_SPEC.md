# cf-device-design Cavity Pinned-Floor — Spec (Scope C)

**Status**: RECON COMPLETE 2026-05-16 EVENING (fresh-context session
following [`CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_BOOKMARK.md`](CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_BOOKMARK.md)).
Implementation NOT yet started. dev at `80716523` (recon-COMPLETE
commit; awaits scope-C update), tree clean. cf-device-design baseline
146 tests / 9 ignored.

**Scope**: **C — all three consumers updated together** (cf-device-design
preview + insertion_sim + cf-cast-cli). User-confirmed 2026-05-16
EVENING after recon laid out scope (A) narrow / (B) preview+sim /
(C) all-three trade-offs.

**Parent**: [`CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_BOOKMARK.md`](CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_BOOKMARK.md)
— carries the verbatim user geometric model + 8 recon questions. Read
its §1 BEFORE this spec; the user's model is the load-bearing context.

**Supersedes**: [`CF_DEVICE_DESIGN_CAVITY_MOUTH_SPEC.md`](CF_DEVICE_DESIGN_CAVITY_MOUTH_SPEC.md)
— shipped but solving wrong problem (open cavity vs closed-with-pinned-
floor). Most of its scaffolding survives unchanged (see §1 Q5).

**Blocks**: cf-device-design preview, insertion sim correctness, cf-cast-cli
mold geometry — all targeted in this arc. Workshop iter-1 cast happens
on the new pinned-floor mold geometry (single-commit rollback path
documented in §7 if print/cast surfaces a failure mode).

---

## Headline finding (read this first if you read nothing else)

Three load-bearing recon outputs:

1. **The cavity-mouth arc's two-SDF cached-grid construction is
   exactly the dome-wall-signed-distance field the pinned-floor
   primitive needs.** The shipped code stripped cap polygons from the
   SDF source for an OPEN cavity; the pinned-floor version uses the
   SAME two-SDF construction and INTERSECTS the iso surface with the
   cap-plane half-space.

2. **Insertion sim + cf-cast-cli compose at the `cf-design::Solid`
   level** (`Solid::from_sdf(scan_sdf, bounds).offset(...).subtract(...)`),
   never consume a triangle mesh from `extract_layer_surface`. The
   pinned-floor primitive for them is a `Solid`-returning composition:
   `Solid::from_sdf(dome_wall_signed_sdf).offset(offset_m).intersect(Solid::plane(...))`
   per cap. Triangle-mesh closed/manifold properties are load-bearing
   for cf-device-design's preview + Validations panel volume integral
   ONLY.

3. **Scope C requires a shared library home** for `CapPlane` +
   parser + `dome_wall_only_mesh` + the two-SDF adapter so all three
   consumers can use the same code. **Decision: new `cf-cap-planes`
   workspace crate** (NOT cf-design — would couple it to cf-scan-prep's
   schema; NOT cf-scan-prep-as-lib — makes a binary tool a load-bearing
   library dep with awkward workspace topology). cf-design's
   `pinned_floor_shell` takes generic `&[(Point3, Vector3)]` cap
   tuples and has ZERO cf-cap-planes dep — keeps cf-design schema-
   agnostic.

Implementation = **6 sub-leaves + visual gate + iter-1 physical cast**.
Right at the ~6-sub-leaf scope-creep trigger but defensible: sub-leaf 1
is enabling scaffolding (new crate creation) with independent reuse
value; sub-leaves 2-6 are the actual pinned-floor behavior change.

---

## 1. Recon answers

### Q1. API shape — primitive signatures

**Answer**: TWO primitives — one for cached-grid extraction (cf-device-
design preview), one for ad-hoc `Solid` composition (insertion_sim +
cf-cast-cli).

**Primitive A** — cf-device-design preview path (signature unchanged
from today; behavior changes to return CLOSED manifold):

```rust
// tools/cf-device-design/src/sdf_layers.rs
pub(crate) fn extract_layer_surface(
    cache: &CachedScanSdf,
    cap_planes: &[CapPlane],
    offset_m: f64,
) -> IndexedMesh
```

Every existing call site (`spawn_cavity_mesh`, `update_cavity_mesh`,
`update_layer_meshes`, `compute_validations`) stays unchanged.

**Primitive B** — cf-design Solid composition (new):

```rust
// design/cf-design/src/lib.rs (or new file solid_layered.rs)
pub fn pinned_floor_shell<S: Sdf + 'static>(
    sdf: S,
    bounds: Aabb,
    cap_planes: &[(Point3<f64>, Vector3<f64>)],
    offset_m: f64,
) -> Solid {
    let mut shell = Solid::from_sdf(sdf, bounds).offset(offset_m);
    for (centroid, normal) in cap_planes {
        // Solid::plane SDF f(p) = p·normal - offset. With offset =
        // centroid·normal, the plane passes through centroid. The
        // cap-normal-side (= outside body) is "positive" → intersect
        // (= max) discards it, keeping body-interior half-space.
        let d = centroid.coords.dot(normal);
        shell = shell.intersect(Solid::plane(*normal, d));
    }
    shell
}
```

**Why two primitives** (not one shared):
- Preview path needs cached-grid acceleration (~20 k cells filled
  once at scan load, extracted many times per frame on slider edits).
  Pointwise `Solid::evaluate` is too slow for per-extract per-cell
  use (no spatial accel).
- Sim/cast path operates on Solid composition that downstream
  meshers (`SdfMeshedTetMesh::from_sdf_yeoh`, mesh-offset's MC) consume.
  They DO accept a `Solid` because the meshers do their own grid fills
  at their own preferred cell sizes.
- Both primitives share the same SEMANTIC composition:
  `dome_wall_signed_sdf.offset(offset_m) ∩ body-interior-side of each cap`.
  Just implemented in two ways for the two performance postures.

### Q2. Implementation strategy — composition mechanism

**Picked**: **per-extract SDF composition (preview) + cf-design::Solid
composition (sim/cast)**. Rejected anisotropic SDF offset (Q6 below),
post-MC delete-floor + stitch-down, scan-cap-polygon reuse.

**Preview construction** (per-cell composition at extract time):

```rust
fn extract_layer_surface(cache, cap_planes, offset_m) -> IndexedMesh {
    if cap_planes.is_empty() {
        // No-caps fast path: byte-identical pre-pinned-floor behavior.
        let config = MarchingCubesConfig::at_iso_value(offset_m);
        return marching_cubes(&cache.grid, &config);
    }
    let mut composed = cache.grid.clone();
    let (nx, ny, nz) = composed.dimensions();
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let p = composed.position(ix, iy, iz);
                let shifted_scan_sd = composed.get(ix, iy, iz) - offset_m;
                // Outward cap normals: (p - c)·n > 0 outside body.
                // Intersection (max) of body-interior half-spaces:
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

**Sim/cast construction** uses Primitive B per Q1.

**Why per-extract composition over alternatives**:

- **Anisotropic SDF offset (skip cap-normal direction)** — REJECTED.
  mesh-sdf / mesh-offset have no such API (Q6); adding one is an
  arc-sized addition spanning two crates. cf-design composition
  achieves the same result with existing primitives.
- **Post-MC delete-floor + stitch-down + flat-cap triangulation** —
  REJECTED. Fragile: boundary loops post-MC may have interpolation
  jitter; multi-cap requires per-cap loop detection + classification;
  flat-cap triangulation per shell must match its boundary polygon
  shape. MC on the composed SDF handles closure naturally.
- **Use cleaned-scan's actual cap polygon as the floor directly** —
  REJECTED. cf-scan-prep's polygon is at the SCAN surface (offset=0);
  inset/outset shells have different floor polygon shapes per offset_m.
  Reuse would need per-shell 2D polygon offset.

**Subtle benefit of per-extract composition**: every shell's floor is
triangulated by MC at the iso 0 ∩ cap_plane intersection. Each shell
gets its own naturally-stitched floor matched to its iso shape, with
NO custom triangulation code anywhere.

**Cost**: per-extract grid clone is `O(cells)` (~160 kB f64 alloc on
iter-1's 20 k-cell grid); per-cell composition is `O(cells × n_caps)`.
Total ~1.5 ms per extract on iter-1, doubles current ~0.5 ms. Negligible
against frame budget. In-place composition (no clone) is a Followup F6
optimization if profiling surfaces a real cost.

### Q3. Where the shared code lives (THE SCOPE-C DECISION)

**Picked**: **new `cf-cap-planes` workspace crate** (option (ii) of the
three architecture options surfaced post-recon).

**Layout**:

```
design/cf-cap-planes/
├── Cargo.toml          # deps: mesh-types, mesh-sdf, mesh-repair,
│                       #       nalgebra, serde, toml, anyhow
└── src/
    └── lib.rs          # ~250 LOC total
```

**Public surface**:

```rust
// design/cf-cap-planes/src/lib.rs
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct CapPlane {
    pub centroid: Point3<f64>,
    pub normal: Vector3<f64>,
    pub vertex_count: usize,
    pub loop_index: usize,
}

impl CapPlane {
    /// Helper for cf-design::pinned_floor_shell call sites that take
    /// generic (centroid, normal) tuples.
    pub fn as_tuple(&self) -> (Point3<f64>, Vector3<f64>) { (self.centroid, self.normal) }
}

/// Parse + transform-bake `[caps]` block from a `.prep.toml` text.
/// Empty `Vec` when block absent, no loops, or all `included = false`.
pub fn parse_cap_planes(prep_toml_text: &str) -> anyhow::Result<Vec<CapPlane>>;

/// Strip cap-polygon faces from a cleaned-scan mesh.
pub fn dome_wall_only_mesh(scan: &IndexedMesh, planes: &[CapPlane]) -> IndexedMesh;

/// Constants for cap-face classification (face-normal + centroid-distance).
pub const CAP_FACE_NORMAL_DOT_MIN: f64 = 0.95;
pub const CAP_FACE_CENTROID_DIST_M: f64 = 0.025;

/// Permanent regression-sentinel diagnostic. Logs per-face cap
/// classification table at startup.
pub fn report_cap_face_classification(mesh: &IndexedMesh, planes: &[CapPlane]);

/// Sdf adapter combining sign-from-closed-body × magnitude-from-open-body.
/// Composes two underlying SDFs into a "signed distance to dome+wall
/// surface" field that has the right sign at every point AND zero
/// magnitude at the cap polygon's natural opening.
pub struct DomeWallSignedSdf {
    pub closed: Arc<dyn Sdf>,
    pub open: Arc<dyn Sdf>,
}

impl Sdf for DomeWallSignedSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.closed.eval(p).signum() * self.open.eval(p).abs()
    }
    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        // Central finite-difference; piecewise-smooth at cap-plane / dome-
        // wall boundaries (DomeWallSignedSdf inherits the piecewise-smooth
        // gradient of mesh-sdf, which cf-design's `Sdf for SignedDistanceField`
        // already documents).
        // ... (eps=1e-6 central diff, matches Sdf for SignedDistanceField precedent)
    }
}
```

**Why (ii) — new crate** vs the alternatives:
- **(i) cf-design owns it**: REJECTED. cf-design today is geometry-
  agnostic — knows nothing about `.prep.toml` schema. Adding parser
  couples it to scan-prep's emit format + adds toml/serde deps it
  doesn't otherwise need. `dome_wall_only_mesh` is also a mesh op
  (walks IndexedMesh faces); cf-design deals in SDF/Solid composition,
  not mesh manipulation. Two abstraction-layer violations.
- **(iii) cf-scan-prep becomes lib+bin**: REJECTED. Makes a Bevy-based
  GUI binary a load-bearing library dep across the workspace. Awkward
  topology; the lib could acquire scope creep over time. cf-scan-prep
  is a tool, not a foundation crate.
- **(ii) new cf-cap-planes**: PICKED. Focused crate (~250 LOC),
  owns the cap-plane abstraction across emit (cf-scan-prep) + consume
  (cf-device-design, cf-design call sites, insertion_sim, cf-cast-cli).
  Bonus benefit: shared serde CapPlane type makes the cf-scan-prep
  emit ↔ consumer parse round-trip type-safe (today both sides hand-
  roll TOML strings).

**Where pinned_floor_shell lives**: cf-design owns the SDF
composition primitive (it's a Solid composition); it takes
generic `&[(Point3, Vector3)]` cap tuples (NOT `&[CapPlane]`) so
cf-design has NO cf-cap-planes dep. Consumers convert at call sites:
`cap_planes.iter().map(|c| c.as_tuple()).collect::<Vec<_>>()`.

**Dependency graph (post-scope-C)**:

```
cf-cap-planes ──── mesh-types, mesh-sdf, mesh-repair, nalgebra, serde, toml
cf-design ──────── mesh-sdf, mesh-offset, nalgebra
                   (NO dep on cf-cap-planes — schema-agnostic)
cf-device-design ─ cf-cap-planes, cf-design, mesh-* (and Bevy/egui)
cf-cast-cli ────── cf-cap-planes, cf-design, mesh-*
insertion_sim ──── cf-cap-planes, cf-design, sim/L0/soft, mesh-*
                   (insertion_sim is in cf-device-design's tree; its
                    deps go through that crate's Cargo.toml)
cf-scan-prep ───── cf-cap-planes (for serde CapPlane round-trip),
                   mesh-* (and Bevy/egui)
```

### Q4. Per-shell triangulation — shared or per-shell?

**Answer**: per-shell, naturally-triangulated by marching cubes at
the iso ∩ cap-plane intersection.

Every shell's pinned-floor polygon is a different shape (inset
cavity's floor = inset polygon; outer T-shell's floor = outward
polygon). MC on the composed SDF triangulates each polygon at extract
time. No shared cap polygon, no per-shell shrink/grow logic, no
relationship between cf-scan-prep's emitted cap polygon and the per-
shell floors.

cf-scan-prep's emitted cap polygon is consumed ONLY for the CapPlane
parameters (centroid + outward normal + vertex_count for tiebreak —
NOTE: tiebreak becomes obsolete with scope-C closed shells; see Q5).
The polygon ITSELF is not consumed geometrically by the pinned-floor
primitive.

### Q5. Two-SDF code disposition — keep or rip out

**Answer**: KEEP, LIFT to cf-cap-planes. Almost the entire cavity-
mouth arc's scaffolding is reusable.

**Lifts to cf-cap-planes** (~700 LOC moving cross-crate):
- `CapPlane` struct (~75 LOC) — now public + serde-derived for emit ↔
  parse round-trip safety.
- `parse_cap_planes` (~50 LOC including PrepTomlSubset schema) —
  moves from cf-device-design's main.rs.
- `dome_wall_only_mesh` (~50 LOC) — moves from sdf_layers.rs.
- `report_cap_face_classification` (~100 LOC) — moves; permanent
  regression sentinel for any face-vs-plane classification.
- Constants (`CAP_FACE_NORMAL_DOT_MIN`, `CAP_FACE_CENTROID_DIST_M`,
  `CAP_FACE_PLANARITY_EPS_M`) — move with their consumers.
- `DomeWallSignedSdf` struct + Sdf impl (~40 LOC NEW; lifts the
  inline two-SDF formula from `build_cached_scan_sdf`'s fill loop
  into a reusable adapter that insertion_sim + cf-cast-cli need
  for pointwise eval via cf-design::Solid::from_sdf).

**Stays in cf-device-design** (cached-grid acceleration is
preview-specific):
- `CachedScanSdf` struct + `build_cached_scan_sdf` (~150 LOC). The
  fill-once / extract-many caching is preview-tied; sim + cast use
  their own SDF construction at their own preferred resolutions.
- `extract_layer_surface` (~80 LOC, rewritten per Q2).
- Cache constants (`LAYER_PREVIEW_CELL_SIZE_M`, `LAYER_GRID_MARGIN_M`,
  `SDF_SOURCE_TARGET_FACES`).

**Deletes** (~50 LOC across cf-device-design's main.rs):
- `primary_cap_origin` (~15 LOC) — closed shells are origin-invariant;
  cap-centroid trick no longer needed.
- `signed_volume_m3`'s second `origin` parameter (~5 LOC). Revert
  to pre-cavity-mouth single-arg form.
- Three associated tests (~40 LOC).

**Stays in cf-device-design as utility, no caller** (~100 LOC; bookmark
recommendation = keep one revision cycle, prune in F6 if no new caller
emerges):
- `clip_mesh_against_cap_plane` — generic Sutherland-Hodgman half-
  space clipper. Loses its caller in sub-leaf 2.

### Q6. Anisotropic offset feasibility

**Answer**: not needed; mesh-sdf / mesh-offset don't support it; the
cf-design composition (uniform offset ∩ cap half-space) achieves the
same geometric result.

**API survey** (confirmed against `mesh/mesh-sdf/src/sdf.rs` +
`mesh/mesh-offset/src/{offset.rs,grid.rs,marching_cubes.rs}`):

| Crate | Function | Behavior |
| --- | --- | --- |
| mesh-sdf | `SignedDistanceField::{new, distance, unsigned_distance, closest_point, is_inside}` | Uniform SDF queries; no directional masking. |
| mesh-offset | `offset_mesh(mesh, config)` | Uniform SDF→MC offset; no per-axis directional config. |
| mesh-offset | `ScalarGrid::{from_bounds, get, set, position, dimensions}` | Generic scalar storage; consumer composes per cell. |
| mesh-offset | `MarchingCubesConfig::{at_iso_zero, at_iso_value}` | Iso surface extraction at a stored value. |

Adding anisotropic offset would span both crates (per-face directional
config in mesh-sdf + per-cell axis-masked fill in mesh-offset). 100+ LOC,
separate arc, new public API. cf-design composition gets the same
result with existing primitives + zero crate changes.

### Q7. Multi-cap handling

**Answer**: automatic via fold-over-caps with `max`. iter-1 single-cap,
iter-N two-cap (proximal + distal), N-cap all use the same code path.

- 0 caps: `cap_sd = NEG_INFINITY`; intersection no-op; identical to
  today's `at_iso_value(offset_m)` byte-for-byte.
- 1 cap: one flat floor.
- 2 caps: two flat floors.
- N caps: N flat floors.

The primary-cap concept (used by today's `primary_cap_origin` for the
divergence-volume integral on open surfaces) becomes obsolete — closed
shells are origin-invariant. See Q5.

### Q8. What does the insertion sim CSG actually want? (LOAD-BEARING)

**Answer**: not a triangle mesh at all. The insertion sim composes at
SDF/`Solid` level via `cf-design::Solid::subtract`, never sees a
triangle mesh.

**Evidence** (`tools/cf-device-design/src/insertion_sim.rs:660-813`):

```rust
let (scan_sdf, _) = build_grid_sdf(&decimated, bounds, grid_cell_m, ...)?;
let cavity = Solid::from_sdf(scan_sdf.clone(), bounds).offset(cavity_offset_m);
let outer = Solid::from_sdf(scan_sdf.clone(), bounds).offset(outer_offset_m);
let body = outer.subtract(cavity);
let mesh = SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh(&body, &hints)?;
```

cf-cast-cli takes the same path (`tools/cf-cast-cli/src/derive.rs:196-208`).

**Scope-C implication**: both consumers swap their
`Solid::from_sdf(scan_sdf, bounds).offset(offset_m)` calls for
`pinned_floor_shell(dome_wall_signed_sdf, bounds, &cap_tuples, offset_m)`.
The FEM sim's `from_sdf_yeoh` and cf-cast-cli's mesher consume the
resulting `Solid` exactly as they consume today's offset-Solid — no
upstream API changes needed.

**Cross-checked against user's geometric model** (bookmark §1):
> "All shells are closed manifolds — the cavity is a 'hole' that
> gets subtracted (CSG) from the layer 0 material for the
> penetration simulation."

`outer.subtract(cavity)` IS that CSG subtract, just at the SDF level
(Solid::subtract = SDF max of shell vs `-other`). User-spec aligned;
the user didn't need to know that the CSG happens at SDF level
mechanically.

---

## 2. Implementation ladder

**Total: 6 sub-leaves + visual gate + iter-1 physical cast.**

Per system-prompt PAUSE-and-flag trigger ("spec ladder grows beyond
~6 sub-leaves — primitive is probably the wrong scope"): at the
threshold but defensible. Sub-leaf 1 is enabling scaffolding (new
workspace crate creation + cross-crate refactor); sub-leaves 2-6 are
the actual pinned-floor primitive work. Each sub-leaf is one commit;
sub-leaf 6 is the LAST so the rollback path (§7) is a single-commit
revert.

### Sub-leaf 1 — Create `cf-cap-planes` crate; lift CapPlane + parser + dome_wall_only_mesh + DomeWallSignedSdf + diagnostics

**Files**:
- NEW: `design/cf-cap-planes/Cargo.toml`
- NEW: `design/cf-cap-planes/src/lib.rs` (~250 LOC)
- `Cargo.toml` (workspace): add `design/cf-cap-planes` member.
- `tools/cf-device-design/Cargo.toml`: add `cf-cap-planes = { path = "../../design/cf-cap-planes" }` dep.
- `tools/cf-device-design/src/sdf_layers.rs`: replace local `CapPlane`,
  `dome_wall_only_mesh`, `report_cap_face_classification`,
  `CAP_FACE_*` constants with `use cf_cap_planes::*;`.
- `tools/cf-device-design/src/main.rs`: replace local `parse_cap_planes`
  with `cf_cap_planes::parse_cap_planes`.

**Behavior**: PURE REFACTOR — no functional changes. All 146 existing
cf-device-design tests pass unchanged. New cf-cap-planes crate has
its own unit tests (lifted from sdf_layers.rs's tests for the moved
functions).

**NEW (didn't exist before)**: `DomeWallSignedSdf` adapter. ~40 LOC +
tests. Implements `Sdf` (eval = `closed.eval().signum() * open.eval().abs()`;
grad = central finite-difference per cf-design's `Sdf for SignedDistanceField`
precedent).

**Tests** (in cf-cap-planes' own tests module):
- All lifted unit tests for the moved functions (parse_cap_planes,
  dome_wall_only_mesh, report_cap_face_classification).
- `dome_wall_signed_sdf_evaluates_to_zero_at_cap_polygon` — point on
  cap polygon's natural boundary returns 0 (open SDF has 0 unsigned
  there; sign × 0 = 0).
- `dome_wall_signed_sdf_negative_inside_body` — point deep inside
  body returns negative (sign(closed_sdf) = -1, magnitude > 0).
- `dome_wall_signed_sdf_positive_outside_body` — point well outside
  body returns positive.
- `dome_wall_signed_sdf_grad_central_diff` — gradient approximates
  the unit normal of the dome-wall surface at a sample point.

**Acceptance**:
- `cargo test -p cf-cap-planes --lib` clean.
- `cargo test -p cf-device-design --lib` clean (regression — 146
  tests pass unchanged).
- `cargo clippy -p cf-cap-planes -p cf-device-design --all-targets` clean.

**LOC delta**: ~+250 in cf-cap-planes, ~-200 in cf-device-design (the
lift) + ~-30 in main.rs (parse_cap_planes lift) + ~+5 in
Cargo.toml/use lines.

### Sub-leaf 2 — Rewrite `extract_layer_surface` to compose pinned-floor SDF at extract time

**File**: `tools/cf-device-design/src/sdf_layers.rs`

**Change**: replace `at_iso_value(offset_m) + post-MC clip loop` with
`clone grid + per-cell compose + MC at iso 0`. Delete the post-MC
`for plane in cap_planes { mesh = clip_mesh_against_cap_plane(...); }`
block. Two-SDF cached grid stays unchanged (built in sub-leaf 1's
refactor target).

**Diff sketch** as in §1 Q2 above.

**Tests** (add to `sdf_layers::tests`):
- `extract_pinned_floor_cube_cavity_has_flat_floor` — unit cube + 1
  cap, `offset_m = -0.005`. Assert closed mesh (χ = 2), bottom
  vertices at cap plane within tol, signed volume ≈ analytical.
- `extract_pinned_floor_cube_outer_grown_has_flat_floor` — same setup,
  `offset_m = +0.005`. Assert closed mesh, flat floor at cap plane,
  signed volume ≈ analytical.
- `extract_pinned_floor_no_caps_fast_path_byte_identical` — empty
  cap_planes; mesh vertices + faces match pre-pinned-floor output
  byte-for-byte.
- `extract_pinned_floor_multi_cap_closes_both_floors` — unit cube +
  2 opposed caps. Assert mesh has TWO flat floors, is closed.
- `extract_pinned_floor_cube_taubin_drift_robustness` — same cube,
  Taubin-smoothed cap vertices (~1 mm drift). Face-normal
  classification stays robust.

**Acceptance**:
- New caps tests pass.
- No-caps fast path byte-identical to pre-pinned-floor.
- `cargo test -p cf-device-design --lib` + `cargo clippy -p
  cf-device-design --all-targets` clean.

**LOC delta**: ~+80 net (delete ~5 LOC post-MC clip; add ~30 LOC
per-cell composition; ~+50 LOC new tests).

### Sub-leaf 3 — Revert `signed_volume_m3` to single-arg; delete `primary_cap_origin`

**File**: `tools/cf-device-design/src/main.rs`

**Change**:
- Drop `signed_volume_m3`'s `origin: Point3<f64>` parameter. Restore
  pre-cavity-mouth single-arg signature; closed shells are origin-
  invariant (proven by existing test).
- Delete `primary_cap_origin` (~15 LOC) and its 3 tests (~40 LOC).
- Update `compute_validations`: drop `volume_origin =
  primary_cap_origin(cap_planes)`; pass nothing to `signed_volume_m3`.

**Tests**:
- Update existing volume tests that called `signed_volume_m3(&mesh,
  Point3::origin())` to drop the second arg.
- Convert `signed_volume_m3_origin_invariant` to a CLOSED-shell
  translation test (translate by `(1, 1, 1)`, assert bit-identical
  volume). Keep as regression sentinel.
- Delete the three `primary_cap_origin_*` tests.

**Acceptance**:
- `cargo test -p cf-device-design --lib` + `cargo clippy` clean.
- `compute_validations` cube + cylinder analytical-fixture tests pass
  with bit-identical pre-cavity-mouth-arc numbers (cap-centroid trick
  added a tiny numerical drift on open meshes that's now gone).

**LOC delta**: ~-50 net deletion.

### Sub-leaf 4 — `cf-design::pinned_floor_shell` primitive + tests

**File**:
- `design/cf-design/src/solid_layered.rs` (NEW, ~50 LOC) OR add to
  `design/cf-design/src/solid.rs` near `Solid::offset` (lighter).
  Recommendation: NEW file `solid_layered.rs` — keeps composite-
  primitive functions separate from primitive constructors.
- `design/cf-design/src/lib.rs`: re-export `pub use solid_layered::pinned_floor_shell;`.

**Signature**:

```rust
pub fn pinned_floor_shell<S: Sdf + 'static>(
    sdf: S,
    bounds: Aabb,
    cap_planes: &[(Point3<f64>, Vector3<f64>)],
    offset_m: f64,
) -> Solid {
    let mut shell = Solid::from_sdf(sdf, bounds).offset(offset_m);
    for (centroid, normal) in cap_planes {
        let d = centroid.coords.dot(normal);
        shell = shell.intersect(Solid::plane(*normal, d));
    }
    shell
}
```

**Tests** (in `solid_layered::tests`):
- `pinned_floor_shell_empty_caps_byte_identical_to_offset` —
  `pinned_floor_shell(sdf, bounds, &[], offset_m).evaluate(p)` ≡
  `Solid::from_sdf(sdf, bounds).offset(offset_m).evaluate(p)` at
  10 sample points. Empty-caps fast path is degenerate-correct.
- `pinned_floor_shell_cube_inset_pins_floor` — unit cube + 1 cap
  on -Z face, `offset_m = -0.005`. Sample SDF at (0, 0, 0) (cap
  plane center, on body side) → returns ~0 (on boundary); at
  (0, 0, +0.01) (above cap plane on cap-normal side) → returns
  positive (outside).
- `pinned_floor_shell_cube_outer_pins_floor` — `offset_m = +0.005`.
  Sample SDF at (0, 0, 0) → returns ~0 (on boundary); same outside
  test.
- `pinned_floor_shell_multi_cap_intersects_all` — unit cube + 2
  opposed caps; at the cube center returns negative; at +Z above
  the top cap returns positive (cap intersection clips).
- `pinned_floor_shell_grad_outside_cap_plane_is_cap_normal` —
  gradient at a point just outside the cap plane points in the cap
  normal direction (the cap-plane Solid's plane SDF dominates).

**Acceptance**:
- `cargo test -p cf-design --lib` clean (existing + new tests).
- `cargo clippy -p cf-design --all-targets` clean.

**LOC delta**: ~+50 (primitive + tests in cf-design).

### Sub-leaf 5 — insertion_sim plumbing: cavity + outer become pinned-floor

**File**: `tools/cf-device-design/src/insertion_sim.rs`

**Change**:
- `build_insertion_geometry` accepts new `cap_planes: &[CapPlane]`
  parameter (or threads via the existing CapPlanes resource —
  caller's choice; recommend explicit param for explicit data flow).
- Build the open-mesh GridSdf alongside the existing closed-mesh
  GridSdf:
  ```rust
  let decimated_open = cf_cap_planes::dome_wall_only_mesh(&decimated, cap_planes);
  let (scan_sdf_open, _) = build_grid_sdf(&decimated_open, bounds, grid_cell_m, ...)?;
  let dome_wall_signed = cf_cap_planes::DomeWallSignedSdf {
      closed: scan_sdf.clone(),  // Arc<GridSdf>
      open: scan_sdf_open,       // Arc<GridSdf>
  };
  ```
  (Note: cf-cap-planes' `DomeWallSignedSdf` holds `Arc<dyn Sdf>`,
  so `GridSdf` must implement `Sdf` — it does today per
  `insertion_sim::GridSdf::Sdf` impl.)
- Replace cavity + outer constructions:
  ```rust
  // BEFORE:
  let cavity = Solid::from_sdf(scan_sdf.clone(), bounds).offset(cavity_offset_m);
  let outer  = Solid::from_sdf(scan_sdf.clone(), bounds).offset(outer_offset_m);
  // AFTER:
  let cap_tuples: Vec<_> = cap_planes.iter().map(|c| c.as_tuple()).collect();
  let cavity = pinned_floor_shell(dome_wall_signed.clone(), bounds, &cap_tuples, cavity_offset_m);
  let outer  = pinned_floor_shell(dome_wall_signed.clone(), bounds, &cap_tuples, outer_offset_m);
  ```
  (`dome_wall_signed.clone()` is `Arc`-cheap.)
- `InsertionGeometry` gains a `cap_planes: Vec<CapPlane>` field for
  downstream consumers (the Bevy panel's per-layer projection may
  want the cap geometry for visualization).
- The per-tet layer assignment (`per_tet_layer` field) — currently
  buckets via `scan_sdf.eval(centroid)` against `layer_boundary_thresholds`.
  With pinned-floor shells, the right scan-distance for bucketing is
  `dome_wall_signed.eval(centroid)` — the new field. Update the
  centroid sampling site to use `dome_wall_signed` instead of
  `scan_sdf` for the layer-membership decision.

**Tests** (in insertion_sim::tests):
- `build_insertion_geometry_no_caps_byte_identical_to_pre_pinned_floor` —
  empty cap_planes; the cavity + outer solids should evaluate to the
  same SDF values as the pre-pinned-floor `scan_sdf.offset()` at 10
  sample points.
- `build_insertion_geometry_with_caps_produces_pinned_floor_body` —
  iter-1-like fixture (cube + 1 cap). Cavity SDF samples: at cap-
  plane center returns ~0; at cube center returns negative; at +Z
  beyond cap plane returns positive.
- Existing row-23 + cf-device-design ramp regression tests pass with
  iter-1's empty-caps path (no caps fixture). The "with caps" ramp
  regression is new behavior; expected to change. NEW reference
  values to be captured at first run (the user reviews + accepts;
  this is the F2 calibration step — bookmark observation that
  pinned-floor changes sim force readouts in a physically-meaningful
  way).

**Acceptance**:
- New tests pass.
- Pre-existing tests on empty-caps path pass bit-identical (regression
  protection on no-caps fast path).
- `cargo test -p cf-device-design --lib --release` clean.
- `cargo clippy -p cf-device-design --all-targets` clean.
- The Bevy Insertion Sim panel's per-layer heat-map projection runs
  cleanly when caps are present (visual gate, post-sub-leaf-5).

**LOC delta**: ~+80 net (new open-mesh GridSdf build + DomeWallSignedSdf
construction + pinned_floor_shell calls + per_tet_layer source switch).

### Sub-leaf 6 — cf-cast-cli plumbing: plug + per-layer bodies become pinned-floor (LAST sub-leaf; rollback target)

**File**: `tools/cf-cast-cli/src/derive.rs` + `tools/cf-cast-cli/src/lib.rs`

**Change**:
- Add `.prep.toml` discovery + parsing to cf-cast-cli (currently
  reads only scan + design):
  ```rust
  // lib.rs or scan.rs:
  fn resolve_prep_toml_path(scan_path: &Path) -> Option<PathBuf> {
      // Mirror cf-device-design's resolve_prep_toml_path logic
      // (strip ".cleaned" suffix; sibling .prep.toml).
  }
  let cap_planes = if let Some(p) = resolve_prep_toml_path(...) {
      cf_cap_planes::parse_cap_planes(&std::fs::read_to_string(p)?)?
  } else {
      Vec::new()  // raw scan, no caps — pinned_floor_shell empty-caps
                  // fast path is byte-identical to today's behavior.
  };
  ```
- Build open-mesh SDF + DomeWallSignedSdf same as sub-leaf 5.
- Replace plug + per-layer body constructions in derive.rs:
  ```rust
  // BEFORE:
  let plug = Solid::from_sdf(scan_sdf.clone(), sdf_bounds).offset(-cavity_inset_m);
  let body = Solid::from_sdf(scan_sdf.clone(), sdf_bounds)
      .offset(cumulative_so_far - cavity_inset_m);
  // AFTER:
  let cap_tuples: Vec<_> = cap_planes.iter().map(|c| c.as_tuple()).collect();
  let plug = pinned_floor_shell(dome_wall_signed.clone(), sdf_bounds, &cap_tuples, -cavity_inset_m);
  let body = pinned_floor_shell(dome_wall_signed.clone(), sdf_bounds, &cap_tuples, cumulative_so_far - cavity_inset_m);
  ```
- Update `procedure_post.rs` if it references cavity geometry in
  markdown (probably not — it just emits press-fit instructions
  driven by `cavity_inset_m`).
- Workshop iter-1 cast geometry CHANGES vs prior iters. See §7 Open
  Risks for the rollback path.

**Tests** (in cf-cast-cli's tests):
- `derive_no_prep_toml_path_byte_identical` — config without
  `.prep.toml` next door yields byte-identical Solid composition to
  pre-pinned-floor (empty-caps fast path).
- `derive_with_caps_uses_pinned_floor_geometry` — synthetic test
  fixture with `.prep.toml` available; plug + bodies are pinned-
  floor (the Solid evaluates ~0 at cap-plane center sample).
- Pre-existing cf-cast-cli tests pass bit-identical on the no-prep-
  toml path.

**Acceptance**:
- New tests pass; old tests pass bit-identical on no-caps path.
- `cargo test -p cf-cast-cli --lib --release` + `cargo clippy` clean.
- `cargo run -p xtask -- grade cf-cast-cli` ≥ A.
- Workshop iter-1 example produces 9 mold STLs + procedure.md
  (matches v2.1 arc's contract; mold geometry changes to pinned-floor
  is the EXPECTED behavior change, NOT a test failure).

**LOC delta**: ~+80 net (.prep.toml discovery + DomeWallSignedSdf
construction + pinned_floor_shell calls).

### Visual gate (post-sub-leaf-6, user-driven)

User runs:
1. **cf-device-design preview** on
   `~/scans/sock_over_capsule.cleaned.stl`. Confirms cavity has flat
   floor pinned at cap plane; outer layers are closed shells with
   coplanar floors; Validations panel mass numbers sane.
2. **Insertion sim** with `[insertion-sim]` enabled. Confirms ramp
   completes; force readouts shift in a physically-meaningful
   direction vs pre-pinned-floor (the cavity floor pin produces a
   harder press-fit toward the cap plane — expected).
3. **cf-cast-cli mold gen** on the same fixture. Confirms 9 mold
   STLs + procedure.md emit; plug visualization in your preferred
   STL viewer shows flat floor at cap plane.

If iter-1 preview or sim looks wrong → that's a falsification of the
spec; work resumes from a new bookmark (NOT continuation).

If iter-1 LOOKS RIGHT but workshop iter-1 print/cast fails on the
new mold geometry → §7 rollback path: `git revert <sub-leaf-6-commit>`
falls back to today's uniform-offset mold while preview + sim retain
pinned-floor.

If all three pass → v1 ships. Followups F4 (fit-viz rungs 2-6), F5
(iter-N multi-cap), F6 (cleanup pass) become eligible for separate
arcs.

---

## 3. Test plan

### Unit (per sub-leaf above)

- Sub-leaf 1: ~10 lifted tests + 4 new `DomeWallSignedSdf` tests in
  cf-cap-planes.
- Sub-leaf 2: 5 new pinned-floor extraction tests in cf-device-design.
- Sub-leaf 3: 1 updated origin-invariance test; 3 deleted tests.
- Sub-leaf 4: 5 new pinned_floor_shell tests in cf-design.
- Sub-leaf 5: 2 new sim-side tests; 1 reference-value capture.
- Sub-leaf 6: 2 new cf-cast-cli tests.

### Integration

- `cargo test --workspace --release` on the empty-caps path produces
  bit-identical behavior to pre-pinned-floor across all touched
  consumers.
- `cargo run -p cf-cast-cli --release -- iter-1-config` produces the
  expected 9-STL output set.

### Visual gate (user-driven, post-sub-leaf-6)

iter-1 sock fixture, per §2. User-side; Claude cannot verify
(no GUI visibility).

### Regression

- `cargo test --workspace --release` clean.
- `cargo clippy --workspace --all-targets` clean.
- `cargo run -p xtask -- grade cf-cap-planes cf-design cf-device-design cf-cast-cli`
  each ≥ A.
- cf-device-design test count: 146 baseline + ~5 new (pinned-floor
  tests) − ~4 (deleted primary_cap_origin tests) = ~147 expected.
- cf-cap-planes: NEW crate; tests target ~15.
- cf-design: existing + ~5 new pinned_floor_shell tests.
- cf-cast-cli: existing + ~2 new pinned-floor tests.
- insertion_sim: existing + ~2 new pinned-floor tests.

---

## 4. Acceptance criteria

- [ ] `cf-cap-planes` workspace crate exists; CapPlane / parser /
      dome_wall_only_mesh / DomeWallSignedSdf / report diagnostic /
      constants all live there with public API.
- [ ] cf-device-design depends on cf-cap-planes; local copies of
      lifted code DELETED.
- [ ] cf-design::pinned_floor_shell primitive exists; ZERO cf-cap-planes
      dep (takes generic `&[(Point3, Vector3)]` tuples).
- [ ] `extract_layer_surface` produces CLOSED manifold output with
      flat floor(s) at every included cap plane for `offset_m` in
      `[-LAYER_GRID_MARGIN_M, +LAYER_GRID_MARGIN_M]`.
- [ ] No-caps fast path byte-identical to pre-pinned-floor on ALL
      consumers (preview + sim + cast).
- [ ] `signed_volume_m3` reverted to single-arg; closed-shell
      `compute_validations` numbers match pre-cavity-mouth-arc bit-
      exact on cube + cylinder fixtures.
- [ ] insertion_sim's `build_insertion_geometry` accepts cap_planes;
      cavity + outer become pinned-floor `Solid`s.
- [ ] cf-cast-cli's `derive` reads sibling `.prep.toml` for caps; plug
      + bodies become pinned-floor `Solid`s.
- [ ] All pre-existing tests pass unchanged (except the 3 deleted
      `primary_cap_origin_*` and the 1 updated origin-invariance test).
- [ ] New unit tests across all 4 crates pass.
- [ ] `cargo test --workspace --release` + `cargo clippy --workspace
      --all-targets` + `cargo run -p xtask -- grade <each>` all clean.
- [ ] **User visual gate** on iter-1 sock fixture (preview + sim +
      mold): all three show pinned-floor geometry correctly.
- [ ] Workshop iter-1 physical print + cast on the new mold geometry
      ships OR rollback path (§7) is exercised cleanly.

---

## 5. Banked followups (out of scope for v1)

(F1/F2/F3 from the scope-B spec are FOLDED INTO v1 as sub-leaves 4/5/6.
The followup list shrinks to:)

- **F4. Fit-viz rungs 2-6 re-enabled**: per-step playback, scan-as-
  intruder, pressure scoring, retraction + over-cycle score,
  auto-search. These consume cf-device-design preview's CLOSED
  cavity mesh from sub-leaf 2; unblocked once v1 lands.
- **F5. iter-N multi-cap visual gate**: when a 2-cap body-part scan
  is in hand (e.g. forearm: wrist + elbow), exercise the fold-over-
  caps path in all three consumers. Code-wise already supported
  by the fold-with-max composition; this is a confirmation pass,
  not a code change.
- **F6. Cleanup pass**: delete `clip_mesh_against_cap_plane` if no
  consumer surfaces in F4/F5. Optimize `extract_layer_surface`'s
  per-cell composition in-place (no grid clone) if profiling
  surfaces a real cost. Delete `report_cap_face_classification`
  diagnostic if regression sentinel turns out unused after 2 iters.
- **F7. Sim-side reference-value re-baseline** (within v1's sub-
  leaf 5; called out separately so the followup ledger captures
  it). The pre-pinned-floor force readouts on the row-23 + cf-device-
  design ramp regressions need user-blessed new reference values
  the first time the with-caps path runs. Worth a session to walk
  through the new numbers + commit them as the new baseline.

---

## 6. Patterns predicted to bank from implementation

- **Two-primitive composition pattern**: cached-grid-acceleration
  variant for the preview path + pointwise-Sdf variant for ad-hoc
  Solid composition. Same SEMANTIC composition; two implementations
  for two performance postures. Reusable wherever a primitive needs
  both fill-once preview + pointwise downstream use.
- **`DomeWallSignedSdf` two-source SDF adapter** as a reusable Sdf
  impl idiom. Generalizes to any "sign from one SDF, magnitude from
  another" composition need.
- **SDF-level CSG via per-extract grid composition**: cache the
  expensive SDF source; compose cheap per-extract overlays at
  extract time. Reusable wherever an SDF needs intersection / union
  with cheap analytical SDFs (half-spaces, spheres, etc).
- **Fold-over-half-spaces with `max`** as a multi-cap composition
  primitive. Order-independent, naturally handles N caps with no
  special-casing.
- **MC closure of an SDF intersection produces naturally-stitched
  flat polygons** at the intersection planes — no custom
  triangulation needed.
- **New focused workspace crate for cross-binary-tool shared schema
  + types** (vs putting it in a foundation crate). Pattern: when 3+
  binary tools need the same schema-aware code, create a focused
  crate rather than coupling foundation crates to the schema.

---

## 7. Open risks

- **MC iso-0 sign-tie behavior at the cap plane**. mesh-sdf's
  `distance` heuristic has a `>= 0.0` branch at sign assignment.
  Composed `max(scan_sd - offset, cap_sd)` can produce exactly-zero
  cell-corner values on the cap plane. MC's exact-zero corner
  handling can produce phantom-needle fragments. **Mitigation**: if
  visual gate (sub-leaf 2) surfaces phantom needles at the cap
  plane, add `CAP_SD_TIE_BREAK_M ≈ 1e-9` to nudge the cap-plane
  half-space inward by a sub-numerical-tie amount. Banked the
  resolution rather than implementing eagerly.

- **mesh-sdf's far-field sign heuristic on dome_wall_only mesh**
  (per cavity-mouth spec §1 Q2 + insertion_sim's build_grid_sdf
  precedent — sign was ~12% wrong on sloppy-decimated open scan).
  For cf-device-design preview, the cached grid only evaluates SDF
  inside body AABB + 40 mm margin where mesh-sdf's heuristic was
  reliable on iter-1. **For insertion_sim's open-mesh GridSdf**
  (NEW in sub-leaf 5): `build_grid_sdf` is the flood-fill SDF that
  was specifically built to handle non-manifold open meshes
  (insertion_sim::build_grid_sdf docstring §7.3a). The
  DomeWallSignedSdf adapter consumes this flood-fill SDF for the
  open side — should sidestep the heuristic-sign issue entirely.
  **For cf-cast-cli** (NEW in sub-leaf 6): cf-cast-cli builds via
  `mesh_sdf::SignedDistanceField` today, not flood-fill. The
  open-mesh SDF build in cf-cast-cli could surface the ~12%-wrong
  sign at points far from the open surface. **Mitigation**:
  cf-cast-cli's sub-leaf 6 should mirror insertion_sim's path and
  use `build_grid_sdf` for the open SDF — this requires lifting
  `build_grid_sdf` from insertion_sim.rs into cf-cap-planes (or
  cf-design). Bundle this into sub-leaf 6's design pass or call it
  out as a sub-leaf-6 dependency. **Recommend**: lift `build_grid_sdf`
  into cf-cap-planes' v1.0 surface alongside DomeWallSignedSdf —
  it's the canonical SDF construction for the open mesh and all
  consumers need it.

- **Per-extract grid clone churn** (cf-device-design preview): iter-1's
  ~20 k cells × ~8 bytes = ~160 kB/clone; per-frame at 60 Hz is
  ~10 MB/s allocator. Acceptable for v1; in-place composition is F6.

- **iter-1 mold geometry changes** vs prior iters (sub-leaf 6). New
  potential failure modes at print/cast:
  - Print: flat cap-plane floor adds a large flat face; orientation
    + supports may need adjustment.
  - Cast: plug now has a flat cap-plane-side floor instead of a
    uniformly-shrunk dome; plug orientation in mold may need
    adjustment.
  **Rollback path**: `git revert <sub-leaf-6-commit>` removes ONLY
  the cf-cast-cli plumbing, leaving preview + sim on pinned-floor
  while cf-cast-cli falls back to today's uniform-offset mold.
  Single-commit revert; clean rollback. Sub-leaf 6 is the LAST
  sub-leaf precisely so this revert is uncomplicated.

- **insertion_sim reference-value re-baseline** (sub-leaf 5). The
  row-23 + cf-device-design ramp regression tests today have force
  readouts captured against uniform-offset geometry. With pinned-
  floor cavity, the cavity floor pin shifts force balance in
  physically-meaningful ways — the regression tests' empty-caps
  path stays bit-identical, but with-caps fixtures need new
  reference values. F7 (banked in §5) captures this; user walks
  through the new numbers + accepts them as the new baseline.

- **cf-scan-prep ↔ cf-cap-planes round-trip schema** (sub-leaf 1).
  cf-cap-planes' serde-derived CapPlane should make the
  cf-scan-prep emit ↔ consumer parse round-trip type-safe. If
  cf-scan-prep doesn't already use cf-cap-planes (today it emits
  TOML by hand), sub-leaf 1 should ALSO update cf-scan-prep's
  emit path to use the shared serde CapPlane. Bundle this into
  sub-leaf 1 to lock in the round-trip safety from day 1, OR call
  out as a sub-leaf-1.5 (cf-scan-prep emit refactor). **Recommend**:
  bundle into sub-leaf 1 — it's a small additional change (~20 LOC
  in cf-scan-prep) and gets the round-trip safety in place before
  any consumer relies on it.

---

## 8. Decision: spec-only this session

This session produces ONLY this spec doc + a `git commit
docs(cf-device-design): pinned-floor scope-C spec rewrite` commit.
NO code, NO experiments, NO one-off scripts. Per
[[feedback-autonomous-architecture]] +
[[feedback-bookmark-when-surface-levers-exhaust]] three-session
pattern.

NEXT SESSION = implementation per the §2 ladder. Cold-read entry
point: this spec, then the parent bookmark §1 (user model), then the
file surfaces listed per sub-leaf. The existing two-SDF cached-grid
path stays as the dome-wall-signed-distance source.

---

## 9. Decisions log (load-bearing recon outputs)

1. **Scope C — all three consumers updated in v1** (preview + sim +
   mold). User-confirmed after recon laid out trade-offs. (§ Headline)
2. **New `cf-cap-planes` workspace crate** as the shared library
   home for CapPlane + parser + dome_wall_only_mesh + DomeWallSignedSdf
   + diagnostics. (§1 Q3)
3. **Two primitives, one semantic composition**: cached-grid extract
   for preview; cf-design Solid composition for sim/cast. Same
   `dome_wall_signed_sdf.offset(offset_m) ∩ cap half-spaces`
   formula. (§1 Q1 + Q2)
4. **cf-design stays schema-agnostic** — `pinned_floor_shell` takes
   generic `&[(Point3, Vector3)]`, NO cf-cap-planes dep. (§1 Q3)
5. **Two-SDF cached grid stays** as the dome-wall-signed-distance
   field source. The cavity-mouth arc's scaffolding survives almost
   entirely. (§1 Q5)
6. **`primary_cap_origin` + cap-centroid-origin volume integral
   trick gets deleted** — closed shells are origin-invariant. (§1 Q5)
7. **Six sub-leaves; cf-cast-cli plumbing is LAST**; single-commit
   revert path for iter-1 print/cast rollback. (§2 + §7)
8. **Insertion sim + cf-cast-cli compose at `Solid` level** — never
   consume a triangle mesh; pinned-floor primitive returns `Solid`.
   (§1 Q8)
9. **Multi-cap is automatic** via fold-over-caps with `max`. iter-1
   single-cap + iter-N two-cap use the same code path. (§1 Q7)
10. **`build_grid_sdf` may need to lift into cf-cap-planes** for
    cf-cast-cli's open-mesh SDF construction (sub-leaf 6 risk
    surfaced in §7). Recommend bundle into sub-leaf 1.
11. **cf-scan-prep emit path** should be updated to use serde CapPlane
    in sub-leaf 1 for round-trip schema safety from day 1.
