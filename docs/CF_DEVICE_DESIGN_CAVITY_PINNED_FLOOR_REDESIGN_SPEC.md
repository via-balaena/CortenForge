# cf-device-design Cavity Pinned-Floor — REDESIGN Spec (recon iteration 2)

**Status**: RECON ITERATION 2 COMPLETE 2026-05-16 LATE-EVENING (fresh-
context session following scope-C falsification at visual gate #1).
Implementation NOT yet started — user reviews spec + decides rollback
strategy before any code lands. Three-session pattern, **second
iteration's recon** (bookmark → recon-this-doc → implementation).

**Scope**: SAME as scope-C — all 3 consumers (cf-device-design preview
+ insertion_sim + cf-cast-cli). The PRIMITIVE CONSTRUCTION changes
(sign × |open| → set-theoretic anisotropic offset); consumer-side
plumbing keeps similar shape (a two-SDF call instead of a sign-blend
adapter call).

**Parent docs** (cold-read order for implementation session):
1. This spec.
2. [`CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_FALSIFICATION_BOOKMARK.md`](CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_FALSIFICATION_BOOKMARK.md)
   — full math of why scope-C failed + 5 candidate alternatives.
3. [`CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_BOOKMARK.md`](CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_BOOKMARK.md)
   §"The actual geometric model the user needs" — LOAD-BEARING user
   verbatim. The anisotropic-offset requirement comes from here.

**Supersedes**:
[`CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_SPEC.md`](CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_SPEC.md)
— shipped (`ccf9c7af` → `fdfeee1d`) but FALSIFIED at visual gate #1.
Sub-leaf 1 (cf-cap-planes scaffolding) and sub-leaf 3 (signed_volume_m3
revert) survive; sub-leaves 2/4/5/6 get reverted per §7 below.

**Blocks (same set as scope-C)**: cf-device-design preview correctness,
insertion sim force-readout correctness, cf-cast-cli mold geometry,
fit-viz rungs 2-6.

---

## Headline finding (read this first if you read nothing else)

**Candidate A wins: set-theoretic anisotropic offset via cf-design
`Solid` boolean ops.** Construction (same semantic for all three
consumers):

```
rind  = Solid::from_sdf({ |open_sdf(p)| - T }, bounds)   // T-thick rind around dome+walls
cavity = body.subtract(rind)                              // body shrunk inward, anisotropically
outer  = body.union(rind)                                 // body grown outward, anisotropically
then intersect each shell with body-interior half-space of every cap.
```

`body` is the closed scan's SDF; `open` is the dome-wall-only mesh's
SDF (cap polygons stripped, same as scope-C's `dome_wall_only_mesh`).
The rind only thickens around dome+walls (cap polygon isn't in the
open mesh) — so subtracting/unioning the rind DOESN'T move the cap
polygon. The cap polygon STAYS at the body's cap polygon position
regardless of `T`. **This is anisotropic offset by construction.**

Floor pinning = floor at body's cap polygon. Body cap polygon sits
≈ at recorded cap plane modulo Taubin drift (~6 mm worst case on iter-
1's cap-fan center vertex per the cap-planes diagnostic). Workshop-
acceptable for v1. Candidate B (pre-project Taubin-drifted cap-fan
vertices to recorded cap plane) is a banked hardening followup if the
v1 visual gate surfaces unacceptable floor drift.

**Structural verification (the math scope-C didn't do)** — iter-1 sock
numbers, body center on cap plane, T = 3 mm cavity inset:

| Quantity            | Falsified scope-C            | Candidate A             |
| ------------------- | ---------------------------- | ----------------------- |
| `closed_sdf(p)`     | ≈ 0 (on cap polygon)         | ≈ 0                     |
| `open_sdf(p)`       | ≈ 31 mm (to dome wall)       | ≈ 31 mm                 |
| `grid_value` / rind | `+1 × 31 = +31 mm`           | `rind = 31 - 3 = +28 mm`|
| Cavity composed SD  | `max(+31-(-3), 0) = +34 mm`  | `max(0, -28) = 0`       |
| Floor at this point | **38 mm OUTSIDE — no floor** | **ON BOUNDARY (✓)**     |

Verified analytically against iter-1's diagnostic (44k cells, body
radius 31 mm, 31 cap-fan faces stripped, cap-fan rim drift p25=6.5 mm).

Implementation = **revert + 5 sub-leaves + visual gate + iter-1 cast**.
Smaller than scope-C's 6 sub-leaves: sub-leaf 1 (cf-cap-planes) and
sub-leaf 3 (signed_volume_m3 revert) survive untouched.

---

## §1. Recon answers

### Q1. Which candidate construction wins?

**A — set-theoretic anisotropic offset via cf-design `Solid` boolean
ops.** The five candidates from the falsification bookmark, scored:

| # | Candidate                              | Status   | Reason                                                                                          |
| - | -------------------------------------- | -------- | ----------------------------------------------------------------------------------------------- |
| A | Set-theoretic via Solid ops            | **PICK** | Anisotropic emerges naturally from `body − rind` where rind is over open mesh. Existing primitives. Structurally verified above. |
| B | A + project Taubin-drifted cap-fan     | Defer    | Tighter floor (no body-polygon drift). Worth only if A's visual gate surfaces drift issues.     |
| C | Pre-process + standard isotropic offset| Reject   | Floor moves with T; doesn't deliver "floor fixed to the plane."                                  |
| D | True anisotropic in mesh-offset        | Reject   | 100+ LOC arc spanning two foundation crates; A delivers same geometry with existing ops.        |
| E | Post-MC boundary-loop stitching        | Reject   | Fragile (interpolation jitter; per-cap loop classification). A's MC-on-composed-SDF stitches naturally. |

The full math walk-through above proves A produces a floor at the cap
plane interior. Layer outer (T > 0) follows the same construction with
`union` instead of `subtract`; floor outline extends T outward from
dome wall instead of inward.

**Multi-cap**: fold `Solid::intersect(body_interior_side)` over caps —
identical pattern to scope-C's `pinned_floor_shell`. N caps gives N
floors at N cap planes.

**`offset_m == 0`**: short-circuit to `body.intersect_all(caps)` (no
rind operation; just the body clipped). Preserves the existing v1
slider-at-zero semantics.

**Empty caps**: short-circuit to `body.offset(offset_m)` — byte-
identical to pre-pinned-floor uniform offset (open SDF never touched).

### Q2. Cap polygon position handling

**Accept Taubin-drifted body cap polygon for v1.** Candidate B (pre-
project) is a banked followup, NOT v1 scope.

Iter-1 diagnostic measurements (this session's stderr capture):

| Metric                                            | Value                |
| ------------------------------------------------- | -------------------- |
| Decimated mesh faces                              | 2,500                |
| Cap-fan faces stripped (face-normal classifier)   | 31                   |
| Cap-fan centroid distance distribution            | p0=0.2 mm, p25=6.5 mm|
| Worst documented vertex drift (per cf-cap-planes) | ~6 mm                |

Candidate A produces floor at body's cap polygon. Floor coplanarity is
"approximately cap plane modulo ~6 mm worst-case drift." For workshop
iter-1 sock_over_capsule, this is workshop-acceptable. If the visual
gate surfaces unacceptable drift on iter-1 print/cast:

**Candidate B as followup arc** (cheap; project cap-fan center +
optional rim vertices to recorded cap plane during cf-cap-planes'
`dome_wall_only_mesh` complement). Doesn't change cf-design's primitive
or any consumer's call site — only the input mesh changes.

### Q3. Preview path performance

**Cache TWO grids** in `CachedScanSdf`: `closed_grid` (always) +
`open_grid: Option<ScalarGrid>` (Some only when caps present).

Memory: 2× v1 (~320 KB on iter-1 vs 160 KB). Negligible against the
frame budget.

Per-extract cost: clone closed_grid; per-cell candidate A composition;
MC at iso 0. Per cell:

```rust
let body_sd = closed_grid.get(cell);
let composed_sd = if cap_planes.is_empty() {
    body_sd - offset_m            // no-caps fast path
} else if offset_m < 0.0 {
    // cavity = body.subtract(rind): max(body_sd, -(|open_sd| - T)) = max(body_sd, T - |open_sd|)
    let rind_sd_neg = T - open_grid.get(cell).abs();
    body_sd.max(rind_sd_neg)
} else if offset_m > 0.0 {
    // outer = body.union(rind): min(body_sd, |open_sd| - T)
    let rind_sd = open_grid.get(cell).abs() - T;
    body_sd.min(rind_sd)
} else {
    body_sd                       // offset_m == 0: just body
};
let composed_with_caps = cap_planes.iter().fold(composed_sd, |acc, cap| {
    let cap_sd = (p - cap.centroid).dot(&cap.normal);
    acc.max(cap_sd)               // intersect with body-interior half-space
});
```

Expected ~2-3 ms total per extract on iter-1 (was ~1.5 ms scope-C; was
~0.5 ms pre-pinned-floor). Within frame budget.

**No-caps fast path**: skip open_grid lookup entirely; behavior byte-
identical to pre-pinned-floor.

### Q4. Sim/cast path: does Solid composition still mesh cleanly?

**Yes.** Same Solid-AST shape as scope-C — nested
`union/subtract/intersect/from_sdf/offset/plane` ops; mesher
(`SdfMeshedTetMesh::from_sdf_yeoh`, mesh-offset's MC) consumes any
`Solid`. The new construction adds one `Solid::from_sdf(UnsignedRindSdf)`
node in the AST — same machinery cf-design's existing primitives use.

Spot-check (folded into sub-leaf A2 test plan): unit-cube fixture with
1 cap, run through `SdfMeshedTetMesh::from_sdf_yeoh(&body, &hints)`,
assert mesh non-empty + closed (Euler characteristic = 2).

### Q5. Rollback strategy

**Revert sub-leaves 2/4/5/6** (commits `fdfeee1d`, `c1994571`,
`2077c25f`, `27851305`). Keep sub-leaf 1 (`ccf9c7af`, cf-cap-planes
crate) + sub-leaf 3 (`af7348fa`, `signed_volume_m3` single-arg revert
+ `primary_cap_origin` delete) — both unconditionally good regardless
of the new construction.

Why revert (not forward-fix):
- Sub-leaves 2/4/5/6 contain test scaffolding tied to the falsified
  `sign × |open|` construction; the new tests (per Q8) probe different
  properties (floor-at-cap-plane structural verification) — refactoring
  the existing tests is more churn than rewriting on a clean base.
- The cf-design::pinned_floor_shell signature changes (gains `open_sdf`
  parameter); forward-fix would still need to rewrite consumer call
  sites; revert just removes them so we re-add cleanly.
- Sub-leaf 1's DomeWallSignedSdf adapter is dead code under candidate A
  (Q6) — removing it cleanly is a single delete on the post-revert tree.

**USER GATE**: flag rollback decision to user BEFORE executing. The
revert is a separate commit, NOT bundled with the spec doc commit.

### Q6. cf-cap-planes survivors

| Item                              | Status     | Reason                                                                              |
| --------------------------------- | ---------- | ----------------------------------------------------------------------------------- |
| `CapPlane` runtime type           | KEEP       | Used by any candidate; the cap-plane parameter shape.                               |
| `parse_cap_planes`                | KEEP       | Parses `[caps]` + `[transform]` from `.prep.toml`; load-bearing scaffolding.        |
| `dome_wall_only_mesh`             | KEEP       | Builds the open mesh that drives the open SDF. Face-normal classifier survives.     |
| `report_cap_face_classification`  | KEEP       | Permanent regression sentinel. Today's iter-1 stderr confirms it works.             |
| Constants (`CAP_FACE_*`)          | KEEP       | Move with classifier.                                                                |
| `CapPlane::as_tuple`              | KEEP       | cf-design stays schema-agnostic via tuple conversion at call sites.                 |
| **`DomeWallSignedSdf` adapter**   | **REMOVE** | The `sign × |open|` construction is what failed at gate #1. No consumer needs it.   |

cf-design's pinned_floor_shell takes a private `UnsignedRindSdf<O: Sdf>`
adapter internally (computes `open.eval(p).abs() - half_thickness`).
Lives in `cf-design/src/solid_layered.rs` as a private struct, NOT
exposed publicly — its only use is inside `pinned_floor_shell`.

cf-cap-planes ends up ~120 LOC smaller (DomeWallSignedSdf + its tests +
its eps constant removed).

### Q7. per_tet_layer + material_field bucketing source

**Keep on `scan_sdf`** — sub-leaf 5's existing deviation (which kept
scan_sdf for per_tet_layer) stays correct under candidate A.

Under candidate A, the wall region (= `outer.subtract(cavity)`) is a
sleeve along the dome wall:
- Inside body, within T_cavity of dome wall (cavity-side of sleeve).
- Outside body, within T_outer of dome wall (outer-side of sleeve).

Wall tets:
- All near the dome wall (not in the cap-plane interior — that's in the
  cavity).
- `scan_sdf` at sleeve tets ≈ ± lateral distance to dome wall (positive
  outside body, negative inside).
- Bucketing by `scan_sdf` against `layer_boundary_thresholds` puts
  innermost tets (closest to cavity = innermost layer) at most-negative
  scan_sdf; outermost layer at most-positive scan_sdf. Matches the user's
  "layer N is wall around layer N-1" radial nesting.

Sub-leaf 5's commit message captures this reasoning verbatim — survives
the construction change because the construction's wall region is
qualitatively similar (the sleeve around the dome wall).

### Q8. Structural validation strategy

**Unit test that samples composed SDF at cap-plane interior points and
asserts SDF ≈ 0, BEFORE running the visual gate.**

The scope-C spec didn't have this. The visual gate falsification could
have been caught at recon time by a 10-line test:

```rust
#[test]
fn pinned_floor_shell_cavity_iso_zero_on_cap_plane_at_body_center() {
    // Synthetic: closed hemisphere (dome above z=0 + flat cap polygon at z=0).
    let closed_mesh = closed_hemisphere_with_cap_polygon();
    let open_mesh   = dome_wall_only_mesh(&closed_mesh, &[cap]);  // strips cap polygon

    let closed_sdf = SignedDistanceField::new(closed_mesh).unwrap();
    let open_sdf   = SignedDistanceField::new(open_mesh).unwrap();
    let cap = (Point3::new(0,0,0), Vector3::new(0,0,1));
    let bounds = Aabb::new(Point3::new(-2,-2,-2), Point3::new(2,2,2));

    let cavity = pinned_floor_shell(closed_sdf, open_sdf, bounds, &[cap], -0.1);

    // BODY CENTER ON CAP PLANE: should be on cavity boundary (SDF ≈ 0).
    let sd = cavity.evaluate(&Point3::new(0,0,0));
    assert!(sd.abs() < 0.01, "expected ~0, got {sd}");

    // ABOVE CAP PLANE: should be outside cavity (clipped).
    let sd_above = cavity.evaluate(&Point3::new(0,0,0.1));
    assert!(sd_above > 0.0, "above cap plane should be outside");

    // DEEP INSIDE BODY (cavity interior): should be negative.
    let sd_deep = cavity.evaluate(&Point3::new(0,0,-0.5));
    assert!(sd_deep < 0.0, "deep interior should be inside cavity");
}
```

This test on the falsified scope-C construction returns `~+0.95` at
body center (sphere_radius - T = 1.0 - 0.1 on the synthetic), failing
loudly. On candidate A it returns ~0. **This is the test that should
have ridden alongside sub-leaf 4 in scope-C.**

Mandate: every sub-leaf in §2 that introduces a new construction step
gets at least one body-center-on-cap-plane SDF-sample-against-tol
assertion in its test plan.

---

## §2. Implementation ladder

**Total: 1 revert commit + 5 sub-leaves + visual gate + iter-1 cast.**

The pre-revert state on `dev` is `25ae9297` (current HEAD; tree clean
after the falsification bookmark commit). The revert backs out 4
sub-leaves and lands at a new commit; ladder builds forward from
there.

### Pre-step (separate commit, USER GATE) — Revert sub-leaves 2/4/5/6

```
git revert fdfeee1d c1994571 2077c25f 27851305
```

Four reverts, four commits (per memory `feedback-post-fmt-stage-all-
changed` + sub-leaf separation). Each revert is mechanical; cargo fmt
may touch sibling files in cf-design/cf-device-design/cf-cast-cli —
stage all-changed per the memory's posture.

**Post-revert state**:
- cf-cap-planes: unchanged from sub-leaf 1 (still has DomeWallSignedSdf
  as dead code).
- cf-design: pinned_floor_shell + solid_layered.rs deleted (rolled back
  to pre-sub-leaf-4 state).
- cf-device-design extract_layer_surface: returns to per-MC-call iso
  value semantics (pre-sub-leaf-2).
- cf-device-design insertion_sim: cavity + outer = `Solid::from_sdf(scan_sdf).offset(...)`
  (pre-sub-leaf-5).
- cf-cast-cli derive: plug + bodies = uniform offset (pre-sub-leaf-6).
- signed_volume_m3 single-arg + primary_cap_origin gone (sub-leaf 3
  survives, NOT reverted).
- All consumers compile + tests pass (since these crates were
  bit-identical pre-sub-leaf-2 except for the deletions in sub-leaf 3).

Test counts on post-revert dev (predicted):
- cf-cap-planes: 19 tests (unchanged from sub-leaf 1).
- cf-design: 771 tests (back to pre-sub-leaf-4 baseline).
- cf-device-design: 134 (sub-leaf-3 deletes survive; sub-leaf-2 adds
  reverted) — same as scope-C dev with 9 ignored.
- cf-cast-cli: 12 derive + 6 integration (back to pre-sub-leaf-6).

### Sub-leaf A1 — cf-cap-planes: remove DomeWallSignedSdf

**Files**:
- `design/cf-cap-planes/src/lib.rs` — delete `DomeWallSignedSdf` struct
  + `Sdf` impl + `DOME_WALL_SIGNED_SDF_GRAD_EPS` const + all
  `dome_wall_signed_sdf_*` tests (~120 LOC removal).
- Update module doc comment to drop the DomeWallSignedSdf mention.

**Behavior**: PURE REMOVAL — no consumer references it after the
revert.

**Acceptance**:
- `cargo test -p cf-cap-planes` clean.
- `cargo clippy -p cf-cap-planes --all-targets` clean.
- Test count: 19 → ~12 (7 DomeWallSignedSdf tests removed).

**LOC delta**: ~-120 net.

### Sub-leaf A2 — cf-design: pinned_floor_shell takes (closed, open) and uses candidate A

**Files**:
- `design/cf-design/src/solid_layered.rs` — NEW file (the post-revert
  state has solid_layered.rs deleted by the revert; we recreate it).
- `design/cf-design/src/lib.rs` — re-export `pub use
  solid_layered::pinned_floor_shell;`.

**Signature**:

```rust
pub fn pinned_floor_shell<C, O>(
    closed_sdf: C,
    open_sdf: O,
    bounds: Aabb,
    cap_planes: &[(Point3<f64>, Vector3<f64>)],
    offset_m: f64,
) -> Solid
where
    C: Sdf + 'static,
    O: Sdf + 'static,
{
    if cap_planes.is_empty() {
        // No-caps fast path: standard isotropic offset (byte-identical
        // to pre-pinned-floor). open_sdf is unused.
        return Solid::from_sdf(closed_sdf, bounds).offset(offset_m);
    }
    let body = Solid::from_sdf(closed_sdf, bounds);
    let mut shell = if offset_m < 0.0 {
        let rind = Solid::from_sdf(
            UnsignedRindSdf::new(open_sdf, -offset_m),
            bounds,
        );
        body.subtract(rind)
    } else if offset_m > 0.0 {
        let rind = Solid::from_sdf(
            UnsignedRindSdf::new(open_sdf, offset_m),
            bounds,
        );
        body.union(rind)
    } else {
        body  // offset_m == 0: just the body, then cap intersects below
    };
    for (centroid, normal) in cap_planes {
        let d = centroid.coords.dot(normal);
        shell = shell.intersect(Solid::plane(*normal, d));
    }
    shell
}

struct UnsignedRindSdf<O: Sdf> {
    open: O,
    half_thickness: f64,
}

impl<O: Sdf> UnsignedRindSdf<O> {
    fn new(open: O, half_thickness: f64) -> Self { Self { open, half_thickness } }
}

impl<O: Sdf> Sdf for UnsignedRindSdf<O> {
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.open.eval(p).abs() - self.half_thickness
    }
    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        // Central finite difference: eval is piecewise smooth (kinks at
        // open.eval == 0). Matches cf-design's Sdf adapter precedent.
        const EPS: f64 = 1e-6;
        let inv_2eps = 0.5 / EPS;
        Vector3::new(
            (self.eval(p + Vector3::x() * EPS) - self.eval(p - Vector3::x() * EPS)) * inv_2eps,
            (self.eval(p + Vector3::y() * EPS) - self.eval(p - Vector3::y() * EPS)) * inv_2eps,
            (self.eval(p + Vector3::z() * EPS) - self.eval(p - Vector3::z() * EPS)) * inv_2eps,
        )
    }
}
```

**Tests** (in `solid_layered::tests`):

1. `pinned_floor_shell_empty_caps_byte_identical_to_offset` — no caps;
   open_sdf unused; result equals `Solid::from_sdf(closed).offset(T)`
   at 10 sample points. Empty-caps fast path regression sentinel.
2. **`pinned_floor_shell_cavity_iso_zero_on_cap_plane_at_body_center`**
   — Q8's structural-verification test. Closed hemisphere + flat cap
   polygon; assert cavity SDF ≈ 0 at body center on cap plane, > 0
   above, < 0 deep below. **This is the test scope-C should have had.**
3. `pinned_floor_shell_outer_iso_zero_on_cap_plane_at_body_center` —
   same shape, T > 0 (layer outer); assert outer SDF ≈ 0 at body
   center on cap plane, > 0 above, < 0 deep below.
4. `pinned_floor_shell_cavity_floor_outline_shrunk_by_T` — point at
   lateral distance T-ε inside from dome wall on cap plane → outside
   cavity (SDF > 0). Point at T+ε inside → inside cavity (SDF < 0).
   Verifies anisotropic floor outline.
5. `pinned_floor_shell_outer_floor_outline_extended_by_T` — point at
   lateral distance T-ε outside dome wall on cap plane → inside outer.
   Point at T+ε outside → outside.
6. `pinned_floor_shell_multi_cap_clips_all_caps` — synthetic two-cap
   sphere; cap-extension sides on both caps return positive SDF.
7. `pinned_floor_shell_offset_zero_returns_body_clipped_by_caps` — `T
   == 0`; result evaluates as the body intersected with cap half-
   spaces (no rind operation).
8. `pinned_floor_shell_meshes_cleanly_via_sdf_meshed_tet_mesh` —
   spot-check (Q4): build cavity Solid, run `SdfMeshedTetMesh::from_sdf_yeoh`
   on unit-cube + 1-cap fixture, assert mesh non-empty.
9. `unsigned_rind_sdf_eval_returns_unsigned_distance_minus_half_thickness`
   — adapter unit test on a known SDF.

**Acceptance**:
- `cargo test -p cf-design --lib` clean (existing 771 + ~9 new).
- `cargo clippy -p cf-design --all-targets` clean.
- All sample-point assertions in test 2 must pass. **If test 2 fails,
  STOP — recon iteration 3 needed; do NOT propagate to consumers.**

**LOC delta**: ~+60 (primitive + adapter + tests).

### Sub-leaf A3 — cf-device-design extract_layer_surface: candidate A per-cell composition

**Files**:
- `tools/cf-device-design/src/sdf_layers.rs` —
  - `CachedScanSdf` struct gains `open_grid: Option<ScalarGrid>` field
    (None on no-caps).
  - `build_cached_scan_sdf` populates `open_grid` only when
    `!cap_planes.is_empty()` (one extra grid fill loop over
    `sdf_open.unsigned_distance(p)`).
  - `extract_layer_surface` rewritten per §1 Q3's per-cell composition
    sketch.
  - Delete the v1's per-cell `signum × |open|` baked-grid pattern from
    `build_cached_scan_sdf` (replaced by raw `closed_grid` containing
    `sdf_closed.distance(p)` and `open_grid` containing
    `sdf_open.unsigned_distance(p)`).

**Tests** (additions in `sdf_layers::tests`):

1. `extract_layer_surface_no_caps_byte_identical_to_pre_pinned_floor`
   — empty cap_planes; result matches `marching_cubes(closed_grid,
   at_iso_value(offset_m))` byte-for-byte.
2. `extract_cube_cavity_has_flat_floor_at_cap_plane` — unit cube + 1
   cap, T = 5 mm. Assert mesh closed (Euler χ = 2), all bottom vertices
   within ε of cap plane.
3. `extract_cube_outer_has_flat_floor_at_cap_plane` — same fixture, T
   = +5 mm.
4. `extract_multi_cap_cube_closes_both_floors` — unit cube + 2 opposed
   caps.
5. `extract_cube_cavity_floor_outline_shrunk_by_T` — measure floor
   polygon's inradius; expect cap polygon inradius minus T.

**Acceptance**:
- New tests pass; existing tests pass unchanged.
- `cargo test -p cf-device-design --lib --release` clean.
- `cargo clippy -p cf-device-design --all-targets` clean.

**LOC delta**: ~+50 net (`open_grid` field + grid-fill loop + per-cell
composition rewrite + 5 new tests).

### Sub-leaf A4 — cf-device-design insertion_sim: cavity + outer via candidate A

**File**: `tools/cf-device-design/src/insertion_sim.rs`

**Change**: thread `open_sdf` into `pinned_floor_shell` calls. The
existing `dome_wall_only_mesh` + `build_grid_sdf` for the open mesh
already exists in sub-leaf 5 (which gets reverted, then re-added here)
— port it over.

```rust
// post-A4:
let cap_tuples: Vec<_> = cap_planes.iter().map(CapPlane::as_tuple).collect();
let (closed_sdf_arc, open_sdf_arc) = if cap_planes.is_empty() {
    let arc: Arc<dyn Sdf> = Arc::new(scan_sdf.clone());
    (arc.clone(), arc)  // open arc unused on no-caps path
} else {
    let decimated_open = dome_wall_only_mesh(&decimated, cap_planes);
    let (open_grid, _) = build_grid_sdf(&decimated_open, bounds, grid_cell_m, 0.75 * grid_cell_m)
        .context("build flood-fill GridSdf from cap-stripped decimated scan")?;
    (
        Arc::new(scan_sdf.clone()) as Arc<dyn Sdf>,
        Arc::new(open_grid) as Arc<dyn Sdf>,
    )
};
let cavity = pinned_floor_shell(closed_sdf_arc.clone(), open_sdf_arc.clone(), bounds, &cap_tuples, cavity_offset_m);
let outer  = pinned_floor_shell(closed_sdf_arc,         open_sdf_arc,         bounds, &cap_tuples, outer_offset_m);
let body   = outer.subtract(cavity);
```

`per_tet_layer` source stays on `scan_sdf` (per Q7; sub-leaf 5's
deviation re-applied verbatim).

**Tests**:
- `build_insertion_geometry_no_caps_byte_identical_to_pre_pinned_floor`
  — empty caps; sampled SDF values match pre-pinned-floor.
- `build_insertion_geometry_with_caps_cavity_iso_zero_at_cap_plane_body_center`
  — synthetic cube + 1 cap. Sample cavity SDF on cap plane at body
  center, assert ≈ 0.

**Acceptance**:
- New tests pass; old tests on no-caps path bit-identical.
- The Bevy Insertion Sim panel's per-layer heat-map projection runs
  cleanly when caps present (visual gate, post-A5).
- `cargo test -p cf-device-design --lib --release` + `cargo clippy
  --all-targets` clean.

**LOC delta**: ~+40 net (re-port the sub-leaf-5 dome_wall_only_mesh +
build_grid_sdf threading; signature update for the two-SDF call).

### Sub-leaf A5 — cf-cast-cli derive_spec_and_ribbon: candidate A (LAST + rollback target)

**Files**: `tools/cf-cast-cli/src/derive.rs` + `lib.rs`.

**Change**: thread `open_sdf` into `pinned_floor_shell` calls. Re-port
the sub-leaf-6 `.prep.toml` discovery + open-mesh SDF build that the
revert removed.

cf-cast-cli uses `mesh_sdf::SignedDistanceField` for the closed scan
today (NOT flood-fill `build_grid_sdf` like insertion_sim). Per scope-C
sub-leaf 6's autonomous-architecture decision: cf-cast-cli's open-mesh
SDF also uses `mesh_sdf::SignedDistanceField` — the sign heuristic on
non-manifold open meshes is irrelevant because `UnsignedRindSdf` only
consumes `open.eval(p).abs()` (the sign source is `closed_sdf`).

This call-site shape replaces scope-C's `DomeWallSignedSdf { closed,
open }` adapter construction:

```rust
let (closed_sdf_arc, open_sdf_arc): (Arc<dyn Sdf>, Arc<dyn Sdf>) =
    if cap_planes.is_empty() {
        let arc: Arc<dyn Sdf> = scan_sdf.inner_arc();
        (arc.clone(), arc)
    } else {
        let open_mesh = dome_wall_only_mesh(scan_mesh, cap_planes);
        let open_sdf = SignedDistanceField::new(open_mesh)
            .context("build SignedDistanceField from cap-stripped scan mesh")?;
        (scan_sdf.inner_arc(), Arc::new(open_sdf))
    };
let cap_tuples: Vec<_> = cap_planes.iter().map(CapPlane::as_tuple).collect();

let plug = pinned_floor_shell(closed_sdf_arc.clone(), open_sdf_arc.clone(), sdf_bounds, &cap_tuples, -cavity_inset_m);

let mut layers = Vec::with_capacity(config.layers.len());
let mut cumulative_so_far = 0.0;
for layer_cfg in &config.layers {
    cumulative_so_far += layer_cfg.thickness_m;
    let body = pinned_floor_shell(
        closed_sdf_arc.clone(),
        open_sdf_arc.clone(),
        sdf_bounds,
        &cap_tuples,
        cumulative_so_far - cavity_inset_m,
    );
    // ...material + push CastLayer
}
```

**Tests** (in cf-cast-cli's tests):
- `derive_no_prep_toml_byte_identical` — config without `.prep.toml`
  yields byte-identical plug + bodies to pre-pinned-floor.
- `derive_with_caps_plug_iso_zero_on_cap_plane_body_center` — synthetic
  fixture with caps; plug SDF ≈ 0 at body center on cap plane.

**Acceptance**:
- New tests pass; no-prep-toml path bit-identical.
- `cargo test -p cf-cast-cli --lib --release` + `cargo clippy
  --all-targets` clean.
- `cargo run -p xtask -- grade cf-cast-cli` ≥ A.

**LOC delta**: ~+40 net (re-port sub-leaf-6 prep.toml discovery + open-
mesh SDF build; two-SDF signature update).

### Visual gate (post-A5, user-driven)

Iter-1 sock fixture (`~/scans/sock_over_capsule.cleaned.stl` +
`~/scans/sock_over_capsule.prep.toml`):

1. **cf-device-design preview**: hide scan + both layers, look at
   cavity alone. CRITERIA: cavity is a CLOSED shell with a flat-ish
   floor at cap plane (drift up to ~6 mm tolerable for v1; absolute
   no-floor = falsification).
2. **Insertion sim**: with `[insertion-sim]` enabled. Ramp completes;
   force readouts shift in a physically-meaningful direction vs
   uniform-offset (the floor pin changes contact mechanics; F7
   reference-value re-baseline is the captured deliverable).
3. **cf-cast-cli mold gen**: same fixture; emits 9 mold STLs +
   procedure.md; plug visualization shows flat-ish floor at cap plane.

**If iter-1 visual gate FAILS** (cavity still has no floor; MC
artifacts; etc.) → falsification of THIS spec; new bookmark + recon
iteration 3.

**If iter-1 visual gate PASSES but workshop iter-1 cast surfaces drift
issues** (~6 mm floor wave makes the cast unusable) → candidate B
hardening followup (project cap-fan vertices to recorded cap plane in
pre-processing; small addition to cf-cap-planes).

**If all three pass** → v2 ships. F4 (fit-viz rungs 2-6), F5 (iter-N
multi-cap), F6 (cleanup) become eligible.

---

## §3. Test plan summary

| Crate              | New tests | Changed tests | Deleted tests |
| ------------------ | --------- | ------------- | ------------- |
| cf-cap-planes      | 0         | 0             | 7 (DomeWallSignedSdf) |
| cf-design          | ~9        | 0             | 0             |
| cf-device-design   | ~7        | 0             | 0             |
| cf-cast-cli        | ~2        | 0             | 0             |

**Bit-identical regression sentinels** (the no-caps fast path):
- `cargo test --workspace --release` on any pre-existing no-caps test
  produces bit-identical output to pre-pinned-floor.
- The only behavior change is the with-caps path; the no-caps path is
  protected by short-circuits in every consumer.

**Pre-visual-gate structural validation**:
- Sub-leaf A2 test #2 (cavity SDF ≈ 0 at body center on cap plane) IS
  the gate that scope-C lacked. If it fails, the construction is wrong
  — stop, don't propagate.

---

## §4. Acceptance criteria

- [ ] Pre-step: 4 revert commits land cleanly; tree clean; existing
      tests pass.
- [ ] Sub-leaf A1: cf-cap-planes loses DomeWallSignedSdf + tests +
      eps constant; ~12 tests remain; clippy clean.
- [ ] Sub-leaf A2: cf-design `pinned_floor_shell` rewritten with new
      signature; Q8's structural test passes (`abs(cavity_sd_at_body_center_on_cap) < 0.01`).
- [ ] Sub-leaf A3: `extract_layer_surface` produces CLOSED manifold
      with flat-ish floor at every cap plane for any `offset_m` in
      `[-LAYER_GRID_MARGIN_M, +LAYER_GRID_MARGIN_M]`. No-caps byte-
      identical.
- [ ] Sub-leaf A4: `build_insertion_geometry` accepts cap_planes; cavity
      + outer compose via candidate A pinned_floor_shell. No-caps bit-
      identical.
- [ ] Sub-leaf A5: cf-cast-cli reads `.prep.toml`; plug + bodies are
      candidate A pinned-floor shells. No-prep-toml bit-identical.
- [ ] All sub-leaves: `cargo test --workspace --release` + `cargo
      clippy --workspace --all-targets` + `cargo run -p xtask -- grade
      <each>` ≥ A.
- [ ] User visual gate on iter-1 sock fixture: preview + sim + mold all
      show pinned-floor cavity with floor at (or near) cap plane.
- [ ] Workshop iter-1 physical print + cast ships OR drift hardening
      followup (candidate B) is exercised cleanly.

---

## §5. Banked followups (out of scope for v2)

- **B. Cap-fan pre-projection** — if iter-1 visual gate or cast
  surfaces unacceptable Taubin-drift on the floor: add a
  `project_cap_fan_to_cap_plane(mesh, planes) -> mesh` helper to cf-cap-
  planes; call from consumers BEFORE building the closed SDF. Cheap
  (~50 LOC); doesn't change cf-design's primitive or any consumer's call
  site shape.
- **F4. Fit-viz rungs 2-6 re-enabled** — per-step playback, scan-as-
  intruder, pressure scoring, retraction + over-cycle score, auto-
  search. Consumes cf-device-design's CLOSED cavity mesh from A3;
  unblocked once v2 lands.
- **F5. Iter-N multi-cap visual gate** — when a 2-cap body-part scan
  (e.g. forearm: wrist + elbow) is in hand. Code-wise already
  supported by the fold-over-caps composition.
- **F6. Cleanup pass** — delete `clip_mesh_against_cap_plane` if no
  consumer surfaces in F4/F5 (it's already orphaned post-revert).
  Optimize `extract_layer_surface`'s per-cell composition in-place
  (no grid clone) if profiling surfaces a real cost.
- **F7. Sim-side reference-value re-baseline** — pre-pinned-floor force
  readouts on row-23 + cf-device-design ramp regressions need user-
  blessed new reference values the first time the with-caps path runs.
  Folded into sub-leaf A4's test plan; called out here for ledger
  visibility.

---

## §6. Patterns predicted to bank from implementation

- **Set-theoretic anisotropic offset via boolean ops over an open-mesh
  rind** — the load-bearing recon contribution. Pattern: when you need
  to offset a closed body anisotropically (skip some surface region),
  build a rind over a CO-DEFINED open-mesh subset of the body, then
  `body.subtract(rind)` for inset or `body.union(rind)` for outset.
  The omitted region (cap polygon here) acts as the anisotropic mask.
- **Structural-verification unit test BEFORE visual gate** — sample
  composed SDF at problem-relevant points (here: body center on cap
  plane) and assert against analytical expectation. Cheap; would have
  caught scope-C's falsification at recon time.
- **`f64::signum(0) = +1` is a load-bearing footgun** — any
  construction that flips sign at a body boundary needs to handle
  `signum(0)` explicitly OR avoid sign-based composition altogether
  (candidate A avoids it by only consuming `.abs()` of the open SDF).
- **Two-SDF sign × magnitude constructions have inherent discontinuities**
  where the two SDFs disagree on "is this on the body surface" (cap
  polygon in one mesh, not in the other). Counter-pattern; document
  in cf-design's Sdf adapter docs.
- **Cached two-grid composition pattern** — for a preview path that
  needs to compose two SDFs at extract time, cache both grids; per-
  cell composition is O(cells) and stays in the per-frame budget.
- **Pre-revert posture as a clean base** — when a multi-sub-leaf arc
  ships then falsifies the core construction, prefer revert (clean
  base) over forward-fix (fighting the wrong abstraction). Demonstrated
  here (the scope-C revert is 4 commits cleanly; forward-fix would
  modify 4 files in 4 crates).

---

## §7. Open risks

- **Body cap polygon Taubin drift** — `~6 mm` worst case on iter-1.
  Floor sits at body's drifted cap polygon (NOT recorded cap plane);
  visual gate may surface this as "wavy floor." Mitigation: candidate
  B hardening followup (project cap-fan vertices to cap plane).
- **mesh-sdf far-field sign on dome-wall-only mesh** — sign-heuristic
  on non-manifold open meshes is ~12% wrong per insertion_sim's prior
  diagnostic. Sidestepped by construction: `UnsignedRindSdf` consumes
  `open.eval(p).abs()`, so sign is discarded. For cf-cast-cli's open
  SDF (which uses `mesh_sdf::SignedDistanceField`, not flood-fill
  `build_grid_sdf`), the unsigned-magnitude reliability is what matters
  and that's robust.
- **MC iso-0 stitching at cap rim** — the cavity floor outline =
  cap polygon shrunk inward by T; MC stitches this naturally at the
  iso=0 intersection of body and rind. Possible MC sampling artifacts
  if the cap-plane half-space cell-corners land exactly at sd=0.
  Mitigation: same `CAP_SD_TIE_BREAK_M ≈ 1e-9` posture scope-C
  banked. Don't add eagerly; only if sub-leaf A3 visual gate surfaces
  phantom needles.
- **Per-extract grid clone churn** — iter-1's ~44k cells × 8 bytes =
  ~350 KB/clone; per-frame at 60 Hz = ~21 MB/s allocator. Acceptable
  for v2; in-place composition is F6.
- **iter-1 mold geometry changes** vs prior iter — same as scope-C's
  risk. Rollback path: `git revert <sub-leaf-A5-commit>` removes ONLY
  cf-cast-cli; preview + sim retain pinned-floor; cf-cast-cli falls
  back to uniform-offset mold. Sub-leaf A5 is LAST precisely so this
  revert is uncomplicated.
- **insertion_sim reference-value re-baseline** — pre-pinned-floor
  force readouts on row-23 + cf-device-design ramp regressions need
  new reference values the first time with-caps path runs. F7 captures
  this; user walks through + accepts as new baseline.

---

## §8. Decision: spec-only this session

This session produces:
- This spec doc.
- Memory updates (annotate falsification + arc + bookmark memos with
  recon-iter-2 outcome; new spec memo; MEMORY.md Resume note).
- (CONDITIONAL ON USER OK) revert commits.

NO code lands in this session beyond the revert (if user OKs).

Per [[feedback-autonomous-architecture]] +
[[feedback-bookmark-when-surface-levers-exhaust]] three-session pattern,
second iteration. NEXT SESSION = implementation per §2 ladder.

Cold-read entry point for implementation: this spec, then the
falsification bookmark (math), then the parent bookmark §1 (user's
verbatim geometric model), then the post-revert file surfaces.

---

## §9. Decisions log (load-bearing recon-iter-2 outputs)

1. **Candidate A wins** — set-theoretic anisotropic offset via cf-design
   Solid composition. Five candidates evaluated; A is correct,
   structurally verified, and uses existing primitives. (§ Headline + Q1)
2. **Revert sub-leaves 2/4/5/6**; keep sub-leaf 1 (cf-cap-planes) +
   sub-leaf 3 (signed_volume_m3). Clean base for new construction. (Q5)
3. **DomeWallSignedSdf adapter gets deleted** in sub-leaf A1 —
   candidate A doesn't need a sign-blend adapter; consumes only
   `.abs()` of open SDF via private `UnsignedRindSdf` in cf-design. (Q6)
4. **`pinned_floor_shell` signature changes** to take TWO Sdfs (closed
   + open). Empty caps → no-caps fast path (open unused). (§ Headline + A2)
5. **CachedScanSdf gains `open_grid: Option<ScalarGrid>`** for preview-
   path candidate A composition. (Q3 + A3)
6. **Body cap polygon Taubin drift accepted for v2** (~6 mm worst case);
   candidate B (pre-projection) is a banked hardening followup, NOT v2
   scope. (Q2)
7. **`per_tet_layer` stays on `scan_sdf`** under candidate A — sub-leaf
   5's existing deviation re-applied verbatim. (Q7)
8. **Structural-verification test at sub-leaf A2** asserting cavity SDF
   ≈ 0 at body center on cap plane — the test scope-C lacked, would
   have caught the falsification at recon time. (Q8)
9. **5 sub-leaves + 1 revert + visual gate + iter-1 cast** — smaller
   than scope-C (sub-leaves 1 + 3 survive untouched). Sub-leaf A5
   (cf-cast-cli) is LAST for single-commit rollback path. (§2)
10. **F7 sim reference-value re-baseline folded into A4** test plan.
    Pre-pinned-floor force readouts need user-blessed new values; bank
    as new baseline at first with-caps run. (§5)
