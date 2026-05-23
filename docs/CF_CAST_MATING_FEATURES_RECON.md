> **PARTIAL REVERT (2026-05-23):** S4 (cup-piece seam SDF → post-MC `MatingTransform::SeamTrim`) was reverted by recon-4 (P) per [`docs/CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md`](./CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md) §F-2 + impl `24bdc221`. S7's funnel-nipple sub-migration was reverted by the funnel fix `2bf0bd17`. S5/S6/S7 mating-features mesh-CSG (registration pins, T-bar/T-slot/plug-shaft, cup pour-gate) STAYS. §F-4 audit falsified S4's flatness justification (pre-S4 SDF seam was already bit-precise flat via MC linear-SDF interpolation). The plug-shaft `MatingTransform::UnionCylinder` got a near-end overlap-bias (`PLUG_SHAFT_NEAR_END_OVERLAP_M`) per the recon-4 paradigm-boundary pattern (`6fcdeb0b`). Retained as audit trail.

# cf-cast mating-features — S2 recon

**Arc:** [`docs/CF_CAST_MATING_FEATURES_PLAN.md`](./CF_CAST_MATING_FEATURES_PLAN.md)
**Predecessors:** [`docs/CF_CAST_MATING_FEATURES_BOOKMARK.md`](./CF_CAST_MATING_FEATURES_BOOKMARK.md)
(S0 inventory + S1 ADR).
**Session:** S2 (Phase 1, recon — no production code edits).
**Library decision (S1):** `manifold3d` 0.1.8 wins; csgrs falsified.

## What S2 ships

A single decision document that S3 (plumbing) and S4/S5/S6 (feature
migrations) execute against. Decisions only — no production code edits,
no spec field renames, no dependency additions to any workspace
`Cargo.toml`. S3 is the first session that touches production code.

The recon answers plan §S2's nine question hooks (`§1`–`§9` below) plus
the three S1-kicked questions (`§10 S2-A/B/C`). Each section closes
with a single boxed **Decision** or **Confirmation** line so S3 can
read this document linearly without re-deriving the rationale.

## §1 Pipeline boundary placement

### Current control flow (one piece, end to end)

`CastSpec::export_molds_v2` (`design/cf-cast/src/spec.rs:803`) drives
the v2 pipeline. The per-piece path is:

```text
                       compose_piece_solid()  ← cf-design Solid algebra
                       design/cf-cast/src/piece.rs:94
                                  │
                                  ▼  cf_design::Solid (SDF expression tree)
                       solid_to_mm_mesh()    ← SDF → MC → mm rescale
                       design/cf-cast/src/mesher.rs:28
                                  │
                                  ▼  IndexedMesh (mm, world-coord verts)
                  ┌──────  HERE  ───────┐    ← post-MC mesh-CSG stage
                  │   apply mating-     │      (introduced in S3)
                  │   feature mesh-CSG  │
                  └─────────┬───────────┘
                            ▼  IndexedMesh (mm, world-coord verts)
                       run_printability_gate()
                       design/cf-cast/src/spec.rs:1244 (validate_for_printing wrapper)
                                  │
                                  ▼  PrintValidation (F4 grade + issues)
                       PendingPiece { mesh, validation, path }
                                  │
                                  ▼  (later, in write_v2_artifacts)
                       save_stl()
                       mesh-io::save_stl
```

The same shape repeats for plugs (`spec.rs:952`), the platform
(`spec.rs:1006`), and the funnel (`spec.rs:1067`) — four `solid_to_mm_mesh
→ run_printability_gate` sites total. Composition uses different
helpers (`compose_piece_solid` for pieces, `add_plug_pins` for plugs,
`platform::build_platform_solid` / `funnel::build_funnel_solid` for
the two single-artifact STLs) but every site funnels through
`solid_to_mm_mesh` for the SDF→mesh boundary.

### Where the new stage goes

The mesh-CSG stage lives **between `solid_to_mm_mesh` and
`run_printability_gate`**. F4 receives the post-CSG mesh so its
grading reflects the final printed geometry — matching plan §G5
(F4 informs but does not gate; physical print at S8 is
authoritative). STL save consumes the same post-CSG mesh; there is
no separate "save the pre-CSG mesh for debugging" path.

### Data structures crossing the boundary

The new stage's function signature:

```rust
// crate: cf-cast (likely new module: design/cf-cast/src/mesh_csg.rs)
pub fn apply_mating_transforms(
    mesh: IndexedMesh,                          // mm, world coords
    transforms: &[MatingTransform],
    target: CastTarget,
) -> Result<IndexedMesh, CastError>;
```

- **`IndexedMesh`** (`mesh-types::IndexedMesh`) is the canonical
  shared-index mesh format already used by `solid_to_mm_mesh`,
  `validate_for_printing`, `save_stl`, and cf-view loaders. Vertices
  are `Vec<Point3<f64>>` in **mm world coordinates** (G9 below);
  faces are `Vec<[u32; 3]>` referencing vertex indices. Shared-index
  by construction.
- **`MatingTransform`** is a new enum (S3 ships) carrying one of
  three variants — the union of operations S4/S5/S6 need:
  ```rust
  pub enum MatingTransform {
      /// S4 seam-plane trim against an exact plane.
      /// Plane normal points toward the half-space to KEEP.
      SeamTrim { normal: Vector3<f64>, offset_m: f64 },
      /// S5/S6 mesh-union of an exact axis-aligned cylinder primitive.
      UnionCylinder { params: CylinderParams },
      /// S5/S6 mesh-subtract of an exact axis-aligned cylinder primitive.
      SubtractCylinder { params: CylinderParams },
  }
  pub struct CylinderParams {
      pub center_m: Point3<f64>,
      pub axis: UnitVector3<f64>,
      pub radius_m: f64,
      pub half_length_m: f64,
      pub segments: u32,        // determinism contract (§2)
  }
  ```
  Compose-time (`compose_piece_solid`, `add_plug_pins`, etc.) emits
  a `Vec<MatingTransform>` *alongside* the `Solid` it returns; the
  mesher site applies them in order. **Transforms are applied in
  declared order** (e.g., S6 T-bar carve ⊳ S5 registration pins ⊳
  S4 seam trim, per §5 G1 bisection).
  (`CylinderParams` here is the per-op payload — `CylinderParent`
  from §2 + per-side `radius_m` + segments. S3 picks the exact
  decomposition; the recon names them separately because the
  *shared* unit across pieces is the `CylinderParent` triple, not
  the per-op struct.)

### Compose-site changes

Each of the four `mesh_and_gate_v2_*` helpers in `spec.rs` gets a
two-line insert between `solid_to_mm_mesh` and
`run_printability_gate`:

```rust
let mesh = solid_to_mm_mesh(&piece_solid, spec.mesh_cell_size_m, target)?;
let mesh = apply_mating_transforms(mesh, &mating_transforms, target)?;  // NEW
let validation = run_printability_gate(&mesh, &spec.printer_config, &path)?;
```

`mating_transforms` is produced by the composition helper alongside
the `Solid`. S3 plumbs the empty case (every composer returns
`(solid, vec![])`) — pure pass-through.

> **Decision §1.** New stage `apply_mating_transforms(IndexedMesh,
> &[MatingTransform], CastTarget) -> Result<IndexedMesh, CastError>`
> lives between `solid_to_mm_mesh` and `run_printability_gate` at
> every compose-and-mesh site (4 sites). Composition helpers emit
> `(Solid, Vec<MatingTransform>)`; S3 ships the empty case.

## §2 Shared-primitive invariant + determinism contract

### What "shared primitive" means in practice

The architectural fix's load-bearing claim is that two pieces'
cylinder meshes are matched **by construction**, not by tolerance.
The unit of sharing is *not* the cylinder mesh itself — pin and
socket are deliberately different radii (pin = `r`, socket =
`r + diametral_clearance_m / 2`). The unit is the **parent
geometry triple**:

```rust
pub struct CylinderParent {
    pub center_m: Point3<f64>,
    pub axis: UnitVector3<f64>,
    pub half_length_m: f64,
}
```

A `CylinderParent` is computed once per pin location at compose
time (e.g., for an M2 registration pin: derived from the ribbon's
split-normal + arc fraction + body-relative offset, as today's
`registration::build_registration_solid` already does). Both
`Negative.union(pin)` and `Positive.subtract(socket)` consume the
same `CylinderParent`; only the `radius_m` differs.

### Deterministic builder

S3 ships:

```rust
// crate: cf-cast (in the new mesh_csg.rs module)
/// Build a manifold3d cylinder primitive aligned to `parent.axis`,
/// centred at `parent.center_m`, with `radius_m` and `parent.half_length_m`.
/// `segments` controls the polygonal facet count and is part of the
/// determinism contract — bit-equal builds require bit-equal segments.
pub fn build_cylinder_along_axis(
    parent: &CylinderParent,
    radius_m: f64,
    segments: u32,
) -> manifold3d::Manifold;
```

Internally this:
1. Constructs `Manifold::cylinder(2.0 * parent.half_length_m, radius_m,
   radius_m, segments as i32, /*center=*/true)` — manifold3d's
   primitive is Z-axis-aligned centred at origin.
2. Builds a 4×3 affine `[f64; 12]` that rotates `+Z` to `parent.axis`
   and translates by `parent.center_m`, then applies
   `Manifold::transform(&affine)` (§10 S2-B; affine over euler).
3. Returns the `Manifold`.

Same triple + same radius + same segments → bit-equal output, by
construction. Same triple + different radius + same segments →
two cylinder meshes whose vertex layout matches face-for-face
modulo the radial scale; that's exactly what the cross-piece
fit invariant requires.

### Determinism contract that manifold3d must honor

From the S1 spike, manifold3d output is bit-deterministic across
runs **after a 1 µm weld pass on the welded form of the output**.
The S3 plumbing layer:

- **Does NOT re-weld manifold3d's output** — `Manifold::to_mesh_f64`
  already returns shared-index data (§10 S2-C). The spike's
  output-side welding was needed to canonicalize csgrs's BSP
  reorderings, not manifold3d's.
- **DOES weld input** when bridging external STLs — but the live
  pipeline's `solid_to_mm_mesh` already emits shared-index
  `IndexedMesh`, so the `IndexedMesh → Manifold` conversion path
  bypasses welding for live use. Welding helpers exist only for
  spike/test fixtures and the S3 geometric-equivalence test
  helper (plan §G2).
- **Pins manifold3d** in the workspace `Cargo.toml` at exact patch
  `=0.1.8` (S1 ADR §"Findings worth banking" item 4). Float-version
  ranges defer until the crate hits 1.0.

### Cross-piece bit-equal test

```rust
// crate: cf-cast, module: mesh_csg::tests
#[test]
fn pin_cylinder_mesh_is_bit_equal_across_pieces() {
    // Build a registration parent triple from a synthetic ribbon
    // (identical to the M2 path used in production).
    let parent = CylinderParent {
        center_m: Point3::new(0.030, 0.010, 0.000),
        axis: UnitVector3::new_normalize(Vector3::new(0.0, 1.0, 0.0)),
        half_length_m: 0.005,
    };
    let r_pin = 0.0015;            // PinSpec::iter1
    let segments = 32_u32;          // determinism: pinned by contract

    // Build the SAME radius twice — two independent calls should be
    // bit-identical mesh-output.
    let m_a = build_cylinder_along_axis(&parent, r_pin, segments);
    let m_b = build_cylinder_along_axis(&parent, r_pin, segments);

    let (verts_a, _, tris_a) = m_a.to_mesh_f64();
    let (verts_b, _, tris_b) = m_b.to_mesh_f64();
    assert_eq!(verts_a, verts_b, "two builds of same parent+radius must produce bit-equal vertex array");
    assert_eq!(tris_a, tris_b, "…and bit-equal triangle index array");

    // The cross-piece fit invariant rests on the SAME-radius bit-equal
    // assertion above (pin and socket of identical radius produce
    // identical mesh, so a Negative-pin call and a Positive-pin call
    // for the SAME registration pin produce matched halves). The
    // pin-vs-socket pair use DIFFERENT radii; their geometric match
    // comes from the affine-12 pose being identical (§10 S2-B) and
    // both meshes deriving from the same `CylinderParent` triple, not
    // from radius-invariant triangulation in manifold3d's primitive.
}
```

The same shape generalizes for the T-bar (M3) and plug-pin shaft
(M4), wrapped behind module-level helpers in S5/S6.

> **Decision §2.** Shared unit is `CylinderParent { center_m, axis,
> half_length_m }`, not the cylinder mesh. Builder
> `build_cylinder_along_axis(parent, radius, segments)` is
> deterministic and lives in cf-cast's new `mesh_csg.rs`. Determinism
> contract: manifold3d 0.1.8 output is bit-equal across runs without
> output welding (§10 S2-C confirms); pin `manifold3d = "=0.1.8"`.
> Test `pin_cylinder_mesh_is_bit_equal_across_pieces` ships in S5
> alongside the first pin migration.

## §3 World-coordinate assumption (G9)

### Trace

`solid_to_mm_mesh` (`design/cf-cast/src/mesher.rs:28`):

```rust
let bounds = solid.bounds().ok_or(CastError::InfiniteBounds(target))?;
let mut grid = ScalarGrid::from_bounds(bounds.min, bounds.max, cell_size_m, GRID_PADDING_CELLS);
for (ix,iy,iz) in 0..nx × 0..ny × 0..nz {
    let p = grid.position(ix, iy, iz);          // ← world coords
    grid.set(ix, iy, iz, solid.evaluate(&p));
}
let mut mesh = marching_cubes(&grid, &MarchingCubesConfig::default());
scale_in_place(&mut mesh, METERS_TO_MM);        // ← unit conversion only
Ok(mesh)
```

`ScalarGrid::from_bounds` (`mesh/mesh-offset/src/grid.rs:65`):

```rust
let origin = Point3::new(min.x - padding_f, min.y - padding_f, min.z - padding_f);
// dimensions chosen so the grid spans the AABB + 2 cells of padding
```

`ScalarGrid::position` (`mesh/mesh-offset/src/grid.rs:120`) is
explicitly documented as **"Get the world-space position of a grid
point"** and returns `origin + (ix, iy, iz) * cell_size`. Marching
cubes interpolates MC vertices linearly between grid corner
positions, so MC vertices live at world-coordinate positions in
meters.

`scale_in_place(mesh, METERS_TO_MM)` then multiplies every vertex
by 1000 — a **pure unit conversion**, not a re-origining. Origin
stays at world `(0, 0, 0)` mm.

### Implication for shared-primitive geometry

The `CylinderParent` triple is in **meters world coordinates** at
compose time. The mesh-CSG stage receives an `IndexedMesh` in
**millimeters world coordinates**. Two options for resolving the
unit gap:

(A) Convert the `IndexedMesh` from mm → m before constructing the
    manifold3d `Manifold`, run CSG in meters, convert back to mm
    on output. Requires two `scale_in_place(0.001)` /
    `scale_in_place(1000)` round-trips per piece. Lossless on f64.

(B) Convert the `CylinderParent` from m → mm at the mesh-CSG-stage
    boundary, run CSG in mm. One scalar multiply per parent at the
    plumbing layer; zero mesh-traversal cost.

Approach (B) is the right choice. The mesh-CSG stage operates on
millimeter `IndexedMesh` already; the parent triples are
compose-time descriptors not yet realized as meshes, so unit
conversion costs O(1) per primitive vs O(verts) per mesh under (A).
The conversion happens once, inside
`build_cylinder_along_axis`, which takes `center_m` /
`half_length_m` and emits a manifold whose coordinates are
already in mm.

> **Confirmation §3.** `solid_to_mm_mesh` emits world-coordinate
> mm vertices. No re-origining. The new mesh-CSG stage receives mm
> world-coord meshes; cylinder builders convert parent triples
> from m → mm at primitive-build time (Approach B). Affine-12
> pose applied in mm-world frame.

## §4 Phase-3 sub-leaf ordering (Q1)

**Decision: S4 → S5 → S6 (plan default).**

### Why

1. **Bisection dependency.** §5 picks G1 default (a): "run the
   cylinder CSG *before* the seam trim from S4 so the trim
   bisects the meshed cylinder cleanly." Approach (a) **requires
   the seam-trim primitive to exist** when S6's T-bar CSG composes
   `[UnionCylinder(t_bar), SeamTrim(ribbon_plane)]`. S6-first means
   building either approach (b)'s two-half-cylinder primitive
   (deviating from G1 default) or a throwaway seam-trim that S4
   later replaces. Either path adds rework that S4-first avoids.

2. **First-integration risk-kill (plan §"Bail-out branches" S4).**
   S4 is the smallest feature with no inter-piece coupling — one
   mesh-trim per piece, no cross-piece test, no spec-field plumbing.
   If manifold3d's mesh-trim against the real-fixture cup geometry
   blows up under any failure mode the S1 spike didn't surface, S4
   surfaces it cheap. S6-first would entangle that risk with
   three-piece coupling failures + new spec fields, making
   diagnosis harder.

3. **Surface area gradient.** S4 ships zero new spec fields. S5
   ships two (`PinSpec::diametral_clearance_m`,
   `PinSpec::axial_clearance_m`). S6 ships four
   (`PlugPinSpec::t_bar_diametral_clearance_m`,
   `PlugPinSpec::t_bar_axial_clearance_m`,
   `PlugPinSpec::shaft_diametral_clearance_m`,
   `PlugPinSpec::shaft_axial_clearance_m`) **and** deletes two
   (`t_slot_radial_slack_m`, `socket_radial_slack_m`). Stepwise
   surface-area growth lets each session's diff stay readable and
   each cold-read pass stay tractable.

The S6-first alternative (plan §S2 Q1) is sound in principle —
most-coupled-first surfaces architectural flaws when less is
invested. It's rejected here on (1)'s dependency cost: the S6-first
path that doesn't rework would be approach (b) (precomputed
half-cylinders), and §5 picks approach (a) for its own reasons.

> **Decision §4.** S4 (seam) → S5 (registration pins) → S6
> (T-bar + plug pin shaft). Plan default.

## §5 T-bar bisection mechanism (G1)

**Decision: approach (a) — run the cylinder CSG *before* the seam
trim** (plan default).

The transform sequence S6 emits for the Negative cup piece (cups
CARVE the T-slot and shaft socket, plugs ADD the T-bar and shaft;
`SeamTrim` only applies to cup pieces because the plug is unsplit):

```text
[
    SubtractCylinder   { t_bar parent + r_t_bar + diametral_clearance/2 },  // M3 T-slot carve
    SubtractCylinder   { shaft parent + r_shaft + diametral_clearance/2 },  // M4 plug shaft socket
    UnionCylinder      { pin_0 parent + r_pin },                            // M2 registration pins (Negative gains protrusion)
    UnionCylinder      { pin_1 parent + r_pin },                            //   Positive piece's sequence swaps these
    UnionCylinder      { pin_2 parent + r_pin },                            //   four to SubtractCylinder + adds
    UnionCylinder      { pin_3 parent + r_pin },                            //   diametral_clearance/2 to each radius.
    SeamTrim           { normal = ribbon_binormal, offset = +bias },        // bisects everything above
]
```

### Why approach (a) over (b)

Approach (b) — two precomputed half-cylinder primitives with a
shared parent + a side enum — buys nothing here. Both halves still
need pose-matched vertex layout, which requires *bisecting one
cylinder mesh*, which is what approach (a) already does. (b)'s only
advantage is decoupling S6 from S4's plumbing; S4 ships first by
§4 ordering, so the decoupling has no value.

Approach (a) inherits a stronger guarantee:

- Both halves come from one input mesh whose vertex layout is
  fixed by `build_cylinder_along_axis(parent, r_t_slot, segments)`.
- The trim operates on the cup body **with the T-slot already
  subtracted**, so the seam plane cuts cup body + T-slot
  cylindrical hole simultaneously. Each half of the T-slot is
  geometrically defined by the same trim plane intersection with
  the same parent cylinder; by symmetry of the bias, both halves
  match to manifold3d's numerical precision.

### Failure mode to monitor

If approach (a) ever produces a T-slot half whose mating face is
not co-planar with the cup-piece mating face (e.g., manifold3d's
trim leaves a sliver where the cylinder hole intersects the
trim plane), recon-2 would either (i) move to approach (b), or
(ii) introduce a separate **plane-intersect-cylinder** CSG check
(intersect the cylinder mesh with the trim plane *before* the cup
trim; assert the resulting half-disk is co-planar to the cup half).
Not pre-emptively designed because the S1 spike's `plane-trim`
op produced zero open edges on real-fixture input.

> **Decision §5.** Approach (a): cylinder CSG before seam trim.
> Single `MatingTransform` sequence per cup piece, ordered to apply
> all UnionCylinder/SubtractCylinder ops first, then SeamTrim. S6
> ships the test
> `t_bar_halves_share_coplanar_seam_face` to gate this.

## §6 Perf budget (G4)

### Back-of-envelope

S1 spike, real-fixture, release build:

| Op                  | manifold3d 0.1.8 |
|---------------------|------------------|
| plane-trim          | 7 ms             |
| cylinder-subtract   | 6 ms             |
| cylinder-union      | 5 ms             |

Per-piece ops (iter-1, 4 layers × 2 cup pieces = 8 cup pieces;
plug is single-piece per layer = 4 plugs; platform + funnel
are out of Phase-3 scope per S0 bookmark's M5/M6 entries):

| Site         | Ops per piece                  | Per-piece cost (ms) |
|--------------|--------------------------------|---------------------|
| Cup Negative | 1 trim + 1 subtract (T-slot) + 1 subtract (shaft socket) + 4 unions (M2 pins) = **7** | 7 + 6 + 6 + 4×5 = **39** |
| Cup Positive | 1 trim + 1 subtract + 1 subtract + 4 subtracts                                       | 7 + 6 + 6 + 4×6 = **43** |
| Plug         | 1 union (T-bar) + 1 union (shaft)                                                    | 5 + 5 = **10**          |

Total per layer = 39 + 43 + 10 = **92 ms** of CSG kernel time.
Times 4 layers = **368 ms iter-1 total**.

Add ~50% overhead for `IndexedMesh ↔ MeshGL64` conversion (vertex
buffer widen u32→u64 + copy; on a ~50 k-vert mesh ~10 ms per
conversion, two conversions per op): ~250 ms additional.

**Iter-1 total estimated mesh-CSG cost: ≤700 ms** (~0.7 s).

### Sanity vs the dominant cost

`project_cf_cast_f4_split_asymmetry` recorded F4 validation taking
~50 s per Positive cup piece on iter-1 — the dominant cost of the
~6 min cf-cast-cli run. Mesh-CSG adds <1 s to a 6 min run, well
below the noise floor of F4's split-asymmetry variance.

### Budget

Plan §G4's ≤15 min ceiling for iter-1 stands; mesh-CSG consumes
under 1% of that. The recon does NOT escalate to parallel-per-piece
CSG or lazy STL export. If iter-2 surfaces a mesh-CSG perf
regression, those are the bail-out levers (plan §S2 G4).

> **Decision §6.** iter-1 mesh-CSG perf budget: ≤2 s additional
> total (≥3× the back-of-envelope, generous). Existing ≤15 min
> ceiling held. No perf mitigation planned for S4-S6; revisit
> only if iter-2 shows >2 s spent in mesh-CSG.

## §7 F4 policy (G5)

Re-affirm: **F4 informs but does not gate; the physical print at
S8 is authoritative.**

### Expected-to-flake under manifold3d output

S1 spike real-fixture F4 grades came back F across all three
operations, dominated by these issue types — all **intrinsic to
the input cup-piece geometry**, not introduced by manifold3d's
CSG:

- **`ThinWall`** — iter-1 cup wall is `wall_thickness_m = 5 mm` near
  the pour-leg / funnel area, below FDM defaults' wall threshold.
  Already flakes pre-CSG; manifold3d preserves the geometry.
- **`ExcessiveOverhang`** — curve-following cup body has steep
  slopes along the dome. Intrinsic to the body cavity surface;
  unrelated to mating features.
- **`LongBridge`** — cap-end concavity. Intrinsic to the
  cup-cavity geometry.
- **`SmallFeature`** — may appear on the M2 pin (3 mm Ø,
  near the FDM default minimum-feature threshold). Pre-CSG also
  flaked here; the spec already documents this as workshop-trusted
  ergonomics.

### Should NOT appear under manifold3d (sentinels for S4-S6)

- **`NotWatertight`** — manifold3d guarantees closed, manifold
  output (S1 spike: zero open edges across all 6 ops).
- **`NonManifold`** — same guarantee. (S1 spike: zero edges with
  3+ incident faces.)
- **`SelfIntersecting`** — manifold3d's boolean engine produces
  intersection-free output by construction; this issue is what
  csgrs surfaced and what failed the library evaluation.
- **`TrappedVolume`** — debatable. The mold cup cavity *is* a
  trapped volume when the cup-piece STL is considered alone (the
  silicone-pour space). F4 will flag this pre- and post-CSG; the
  flag is semantic, not a CSG regression. Treat as expected-flake.

If any of `NotWatertight` / `NonManifold` / `SelfIntersecting`
**appears post-CSG when pre-CSG was clean**, S4/S5/S6 should
halt and treat it as a manifold3d regression — write a recon-2
bookmark.

### Operationalization

S3 does not change `run_printability_gate`. F4 still emits its
existing severity-graded `PrintIssue` set; `blocking_critical_count`
(`spec.rs:1311`) still gates `Critical` issues. The S1 spike's
F-grade real-fixture meshes contained zero Critical issues for
manifold3d output, so the gate threshold doesn't change.

> **Decision §7.** F4 policy holds as plan §G5 stated. Expected-flake:
> `ThinWall`, `ExcessiveOverhang`, `LongBridge`, `SmallFeature`,
> `TrappedVolume` (semantic). Sentinel-flake: `NotWatertight`,
> `NonManifold`, `SelfIntersecting` — if these go from absent to
> present across the CSG stage, halt + write recon-2.

## §8 Test-semantics churn (G8)

Walked all 15 test modules in `design/cf-cast/src/`. Total tests
affected by SDF→mesh-CSG migration:

| File:line                          | Test                                                                                  | Action  | Owning session |
|------------------------------------|---------------------------------------------------------------------------------------|---------|----------------|
| `piece.rs:290`                     | `both_pieces_overlap_at_ribbon_seam`                                                  | DELETE  | S4             |
| `piece.rs:422`                     | `negative_piece_has_single_connected_component_with_iter1_pins_on_wide_body`          | REWRITE | S5             |
| `piece.rs:492`                     | `plug_socket_carves_at_cap_plane_centroid_past_trimmed_centerline_tip`                | REWRITE | S6             |
| `registration.rs:391`              | `pin_cylinders_are_inside_only_at_pin_positions`                                      | REWRITE | S5             |
| `plug.rs:706`                      | `plug_pin_solid_is_inside_only_at_pour_end_pin_by_default`                            | REWRITE | S6             |
| `plug.rs:743`                      | `socket_solid_is_larger_radius_than_pin_solid`                                        | DELETE  | S6             |
| `plug.rs:769`                      | `add_plug_pins_extends_plug_into_pin_region`                                          | REWRITE | S6             |
| `plug.rs:803`                      | `plug_pin_anchors_at_cap_plane_centroid_past_trimmed_centerline_tip`                  | REWRITE | S6             |
| `spec.rs:2745`                     | `compose_piece_solid_with_pins_negative_side_gains_protrusion`                        | DELETE  | S5             |
| `spec.rs:2789`                     | `compose_piece_solid_with_pins_positive_side_gains_hole`                              | DELETE  | S5             |
| `spec.rs:2428`                     | `export_molds_v2_per_piece_aabb_within_bounding_region`                               | MONITOR | S5             |

11 affected tests, split as 5 DELETE / 6 REWRITE (one of which is
MONITOR-only — may not need a code edit if the assertion margin
holds).

### Rationale per row

- **DELETE rows** assert behavior that no longer exists post-migration:
  - `both_pieces_overlap_at_ribbon_seam` asserts the 0.5 mm SDF
    bias. Post-S4 the seam is a post-MC trim and no bias exists.
  - `socket_solid_is_larger_radius_than_pin_solid` asserts the
    SDF-time `t_slot_radial_slack_m` adds to pin radius. Post-S6
    `t_slot_radial_slack_m` is deleted in favour of an explicit
    `t_bar_diametral_clearance_m` field; the socket > pin
    relationship becomes a spec-field arithmetic identity, not a
    behavioral claim.
  - `compose_piece_solid_with_pins_*_gains_{protrusion,hole}` assert
    that `compose_piece_solid` modifies the *Solid* with pin
    geometry. Post-S5 the Solid is unchanged; pins are applied as
    `MatingTransform`s. The post-CSG mesh assertions live in new
    tests S5 writes.

- **REWRITE rows** assert real invariants whose surface moves from
  SDF-evaluation to post-CSG mesh-inspection:
  - The two `*_carves_at_cap_plane_centroid_past_trimmed_centerline_tip`
    tests survive as MESH tests: assert the post-CSG mesh contains
    no triangles inside the socket interior at the relevant query
    point (use parry3d's TriMesh point-in-mesh query, or a
    bounding-AABB intersection).
  - `negative_piece_has_single_connected_component…` keeps its
    connected-component invariant; the geometry it asserts moves
    from SDF-meshed pin union to post-CSG pin union, same outcome.
  - `pin_cylinders_are_inside_only_at_pin_positions` becomes a
    post-CSG point-in-mesh query at pin centers + a few off-pin
    sample points.
  - `plug_pin_solid_is_inside_only_at_pour_end_pin_by_default`,
    `add_plug_pins_extends_plug_into_pin_region` — same pattern.

- **MONITOR row** — `export_molds_v2_per_piece_aabb_within_bounding_region`:
  pre-S5 the pin SDF union grows the piece AABB; post-S5 the pin
  mesh union grows the post-CSG AABB the same way. The test should
  continue to pass with no edit; flagged in case the assertion's
  margin tolerance is tighter than the post-CSG AABB allows.

### Out-of-scope tests (KEEP, untouched)

All other tests across cf-cast — including all 26 ribbon tests, all
6 funnel tests, all 4 platform tests, all 6 procedure tests, all
material/cure/pour_volume tests, and the rest of `spec.rs`'s 58
tests — are untouched by S4-S6 because:

- Ribbon / centerline / split-normal / arc-fraction logic feeds
  compose-time geometry derivation, independent of SDF vs mesh-CSG.
- Funnel + platform are S7 sweep candidates per §M5/M6; their tests
  are out of Phase 3 scope.
- Material / cure / pour_volume + procedure / mass-budget tests
  live above the SDF/mesh-CSG boundary entirely.
- Spec error paths (`export_molds_v2_errors_when_*`) test
  high-level pipeline failure modes that don't touch the
  CSG-stage internals.

### G8 split-trigger check (plan)

Plan §"Bail-out branches" cites "more than ~8 tests needing rewrite"
as the S3 split-trigger. This walk surfaces 11 affected tests — but
**6 are REWRITE, 5 are DELETE, and 1 of the 6 is MONITOR-only**.
The substantive code churn is 5-6 rewrites, distributed across S4
(1), S5 (4), and S6 (3-4). **No session lands more than 4 test
churns of its own**, so the plumbing-only S3 stays a single session
and the test rewrites land in S4/S5/S6 alongside the migrations
that need them.

> **Decision §8.** 11 tests affected: 5 DELETE, 6 REWRITE (one
> MONITOR-only). Distribution: S4 owns 1, S5 owns 5, S6 owns 5.
> Per-session count under 5 each; **S3 stays single-session**, no
> S3a/S3b split.

## §9 Clearance spec fields (G6)

### Final naming form: SPLIT everywhere

S0 §"Open questions kicked to S2" item 5 surfaced drift between
the bookmark (split: `_diametral_clearance_m` + `_axial_clearance_m`)
and plan §G6 (singular: `shaft_clearance_m`, `NIPPLE_CLEARANCE_M`).
The asymmetry was always within plan §G6 itself — M3 used split
(`t_bar_diametral_clearance_m` + `t_bar_axial_clearance_m`) while
M4/M5 used singular.

Decision: **split form across all six clearance fields**. Rationale:

1. Diametral and axial clearances are physically independent (M3
   captive-insertion needs ~0 mm diametral + 1.0 mm axial; M5
   resting-contact needs 0.50 mm diametral + 0.50 mm axial; the
   single-number form forces both to be the same). Spec-field
   surface should reflect that physical independence.
2. The S0 clearance-budget table already presents every entry as
   "diametral × axial" — split naming aligns the spec with the
   doc that drives clearance decisions.
3. Symmetry across M2/M3/M4/M5 makes future readers' grep patterns
   predictable.

### Final spec-field set (S5/S6/S7 ships these)

| ID | Owning session | Spec field                                                | Default (mm) |
|----|----------------|-----------------------------------------------------------|--------------|
| M2 | S5             | `PinSpec::diametral_clearance_m`                          | 0.00020 (0.20 mm)  |
| M2 | S5             | `PinSpec::axial_clearance_m`                              | 0.00050 (0.50 mm)  |
| M3 | S6             | `PlugPinSpec::t_bar_diametral_clearance_m`                | 0.00030 (0.30 mm)  |
| M3 | S6             | `PlugPinSpec::t_bar_axial_clearance_m`                    | 0.00100 (1.00 mm)  |
| M4 | S6             | `PlugPinSpec::shaft_diametral_clearance_m`                | 0.00030 (0.30 mm) ¹ |
| M4 | S6             | `PlugPinSpec::shaft_axial_clearance_m`                    | 0.00100 (1.00 mm)  |
| M5 | S7             | `funnel.rs::NIPPLE_DIAMETRAL_CLEARANCE_M` (const)         | 0.00050 (0.50 mm)  |
| M5 | S7             | `funnel.rs::FLANGE_AXIAL_STANDOFF_M` (const)              | 0.00050 (0.50 mm)  |

¹ See M4 fit-class resolution below; this default may escalate to
0.50 mm after S5's print trial.

S6 **deletes**: `plug.rs::PlugPinSpec::t_slot_radial_slack_m`
(`plug.rs:207`), `plug.rs::PlugPinSpec::socket_radial_slack_m`
(`plug.rs:145`).

S7 **deletes**: `funnel.rs::FUNNEL_NIPPLE_SLACK_M` (`funnel.rs:65`).

### M4 fit-class resolution (S0 open question 1)

S0 flagged: current `socket_radial_slack_m = 0.5 mm` (1.0 mm
diametral) is a slide-fit; plan baseline 0.30 mm is positional;
recon decides whether to keep slide-fit semantics or tighten.

Decision: **tighten to 0.30 mm × 1.00 mm (positional-fit baseline)**
in S6's initial spec defaults. Print-trial in S5 (using the
M2 pin/socket at 0.20 mm diametral as the same-printer same-filament
calibration probe) informs S6: if the M2 trial binds at 0.20 mm,
S6 ships M4 at 0.50 mm diametral (slide-fit retained) instead of
0.30 mm. The split between M2 (0.20 mm) and M4 (0.30 mm) preserves
the positional vs sliding distinction from the budget table.

Why not keep slide-fit unconditionally: the 1.0 mm slack was a
Mechanism-B compensation, not a slide-fit choice — `plug.rs:145`'s
comment cites "MC grid offset between plug and cup pieces" as the
reason for 0.5 mm/side. With the architectural fix erasing
Mechanism B, the slide-fit can shrink to the FDM-precision floor.
Workshop ergonomics is preserved by axial 1.00 mm pocket-bottom
relief (already in the budget).

Why not pre-emptively widen to 0.50 mm: the S1 spike confirmed
manifold3d produces bit-deterministic, zero-open-edge geometry on
real-fixture input. The cylinder pair is bit-equal by construction.
0.30 mm is the FDM-precision midpoint per plan §S0 baseline
rationale; the only failure mode forcing escalation is FDM bead
bulge worse than the spec assumes, which the M2 print trial
empirically tests.

> **Decision §9.** Split form across all six clearance fields:
> `*_diametral_clearance_m` + `*_axial_clearance_m`. Defaults per
> table above. S5/S6/S7 ship the fields per the Owning Session
> column. M4 baseline = 0.30 mm × 1.00 mm; escalates to 0.50 mm
> only if M2's 0.20 mm trial binds.

## §10 S1-kicked questions (S2-A/B/C)

### S2-A — manifold-csg-sys internal precision

**Confirmation: f64 end-to-end.**

manifold-csg-sys 3.4.107's `build.rs` clones upstream
`elalish/manifold` at commit
`5f95a3ac0e906f596bb2d27a52d005ef60de58f3` and configures cmake with:

```text
-DMANIFOLD_TEST=OFF -DMANIFOLD_PYBIND=OFF -DMANIFOLD_JSBIND=OFF
-DMANIFOLD_CBIND=ON -DMANIFOLD_CROSS_SECTION=ON
-DMANIFOLD_USE_BUILTIN_CLIPPER2=ON -DBUILD_SHARED_LIBS=OFF
-DCMAKE_POSITION_INDEPENDENT_CODE=ON
[+ -DMANIFOLD_PAR=ON/OFF per the "parallel" cargo feature]
```

No `-DMANIFOLD_USE_FLOAT` flag is set. Upstream manifold's default
is f64 (the codebase compiles in dual-precision and exposes both
MeshGL (f32) and MeshGL64 (f64) C-API surfaces; the f64 surface
is what manifold-csg's `from_mesh_f64`/`to_mesh_f64` calls into).

The Rust FFI confirms this end-to-end:

- `manifold-csg-sys/src/lib.rs:579-580`
  `manifold_meshgl64_vert_properties(mem: *mut f64, m: *const ManifoldMeshGL64) -> *mut f64`
- `manifold-csg-sys/src/lib.rs:612`
  `manifold_meshgl64_tolerance(m: *const ManifoldMeshGL64) -> f64`
- `manifold-csg-sys/src/lib.rs:504,514`
  `manifold_meshgl64(... vert_props: *const f64, ...)`
- `manifold-csg/src/manifold.rs:85` (`from_mesh_f64` doc):
  "This uses MeshGL64 internally for full f64 precision — no f32
  round-trip that would destroy sub-mm features at large coordinates."

The S1 ADR's "S2-A line item" is closed: f64 across the FFI, f64
inside the C++ kernel.

### S2-B — affine-12 vs euler-XYZ-degrees for shared-primitive

**Decision: affine-12.**

Use `manifold-csg-0.1.8::Manifold::transform(&[f64; 12])`
(`manifold-csg-0.1.8/src/manifold.rs:571`). The 12 values are a 4×3
column-major affine. Two reasons:

1. **Precision.** Euler-XYZ rotates by composing three single-axis
   rotation matrices internally (`manifold.rs:550` doc: "applied in
   z-y'-x'' order"). Each accumulates round-off. Affine-12 is one
   matmul on the manifold side, identical bit-pattern across calls
   with identical inputs.
2. **Determinism by construction.** The shared-primitive cookbook
   needs same-pose meshes across pieces. With euler, the caller
   must derive the same three angles from the same axis (a
   non-trivial computation involving `atan2` + branch-cuts), and
   pin/socket builders sharing one parent must both produce
   bit-equal angles. With affine-12, the caller builds *one*
   `nalgebra::Isometry3<f64>` from the parent triple, converts to
   `[f64; 12]` once, and both pin and socket builders consume that
   exact array.

`build_cylinder_along_axis` (§2 builder) takes
`(CylinderParent, radius_m, segments)`, computes
`Rotation3::rotation_between(z_axis, parent.axis)` (handling the
antipodal `Some/None` case explicitly — `axis ≈ -z` yields a 180°
rotation around any perpendicular axis; the helper picks one
deterministically), packs `(rotation, parent.center_m)` into
`[f64; 12]` via a small helper
`affine_12_from_isometry(iso: &Isometry3<f64>) -> [f64; 12]` (S3
ships in `mesh_csg.rs`), and calls `m.transform(&affine)`.

### S2-C — output re-weld necessity for to_mesh_f64

**Confirmation: no re-weld needed.**

`Manifold::to_mesh_f64` (`manifold-csg-0.1.8/src/manifold.rs:329`):

```rust
pub fn to_mesh_f64(&self) -> (Vec<f64>, usize, Vec<u64>) {
    let meshgl = unsafe { manifold_alloc_meshgl64() };
    unsafe { manifold_get_meshgl64(meshgl, self.ptr) };
    // …copies meshgl's vert_properties + tri_verts into Vec buffers…
    (vp_buf, n_props, tri_buf)
}
```

The implementation calls into `manifold_get_meshgl64` (the C-API
shim that copies manifold's internal `MeshGL64` representation into
the wrapper) and returns it verbatim. Manifold's internal canonical
form is **index-shared by construction** — each vertex appears once
in `vert_properties`, and `tri_verts` (the u64 triangle index array)
references those shared vertices. There is no per-triangle vertex
duplication that re-welding would collapse.

S1 spike empirical evidence supports the impl-read: bit-deterministic
output (`dv0` across re-runs) and zero open edges on the welded form
of the output mean the output was already manifold-clean *without*
the weld step — the weld pass was canonicalizing csgrs's BSP
reorderings, not fixing topology.

The S3 plumbing path:

- `IndexedMesh → Manifold`: convert in place, no welding (the
  upstream `solid_to_mm_mesh` already emits shared-index meshes).
- `Manifold → IndexedMesh`: also no welding (`to_mesh_f64` returns
  index-shared data). Narrow `Vec<u64>` to `Vec<[u32; 3]>` with a
  `debug_assert!(verts < u32::MAX as usize)` (iter-1 cup mesh is
  ~50 k verts; comfortably within u32).

Spike/test helpers that round-trip through STL (`mesh-io::save_stl`
→ `mesh-io::load_stl`) MUST weld before constructing a `Manifold`
(STL is non-indexed; manifold rejects non-shared-vertex meshes
with `CsgError::ManifoldStatus(NotManifold)`, per S1 ADR finding 3).
This is a test-helper concern, not a live-pipeline concern.

> **Decision §10.** S2-A confirmed (f64 end-to-end). S2-B affine-12
> picked. S2-C no output re-weld; input-weld only at STL round-trip
> boundaries (test helpers).

## §11 Spike-surfaced failure modes + mitigations (S3 reads this)

S1 ADR §"Findings worth banking" + this recon's investigations
surfaced the following risks that S3 plumbing must handle in its
first commit:

1. **manifold3d 0.1.8 is pre-1.0.** Pin exact version
   `manifold3d = "=0.1.8"` in the workspace `Cargo.toml`. Same
   for `manifold-csg = "=0.1.8"` if S3 depends on it directly
   (it shouldn't — manifold3d re-exports the safe surface).
2. **Z-axis-only primitives.** `Manifold::cylinder` is Z-aligned;
   `Manifold::cube` is corner-or-center-at-origin. The
   `build_cylinder_along_axis(parent, radius, segments)` helper
   (§2) is the only callable surface S4-S6 should touch. Same
   pattern for a `build_half_space_slab(plane_point, normal,
   slab_thickness)` helper that backs `SeamTrim` — S4 ships this
   alongside the trim impl.
3. **manifold rejects unwelded input.** Live pipeline is safe
   (shared-index by construction); test/spike round-trips through
   STL must weld at 1 µm. S3 ships a `weld_in_place(mesh: &mut
   IndexedMesh, tolerance_m: f64)` helper for the
   geometric-equivalence test helper (plan §G2) + future test
   fixtures.
4. **Conversion cost is not free.** The `IndexedMesh ↔ MeshGL64`
   round-trip copies vertex + index buffers. At ~50 k verts /
   ~100 k tris (iter-1 cup piece) this is ~10 ms per conversion,
   ~20 ms per CSG op including round-trip. The §6 perf budget
   absorbs this with margin.
5. **C++ build is slow first time.** manifold-csg-sys's build.rs
   clones + cmake-configures + builds the upstream C++ kernel.
   Cold build measured ~3 min 14 s in the S1 spike. Document this
   in the workspace `Cargo.toml`'s comment near the dependency so
   future contributors don't think their machine is broken.

> Mitigation §11. S3 owns: pin exact version; ship
> `build_cylinder_along_axis` + `build_half_space_slab` +
> `weld_in_place` helpers; document the cold-build cost.

## Decision summary

| § | Topic                              | Decision                                                                                           |
|----|-----------------------------------|----------------------------------------------------------------------------------------------------|
| 1  | Pipeline boundary                 | New stage `apply_mating_transforms` between MC and F4 at all 4 compose-and-mesh sites.             |
| 2  | Shared-primitive invariant        | `CylinderParent` triple + `build_cylinder_along_axis` deterministic builder. Pin to `=0.1.8`.      |
| 3  | World-coord assumption (G9)       | Confirmed: mm world coords throughout the new stage. Convert parent triples m→mm at builder time.  |
| 4  | Phase-3 ordering (Q1)             | S4 → S5 → S6 (plan default).                                                                       |
| 5  | T-bar bisection (G1)              | Approach (a): cylinder CSG before seam trim.                                                       |
| 6  | Perf budget (G4)                  | ≤2 s additional iter-1 total; ≤15 min ceiling held.                                                |
| 7  | F4 policy (G5)                    | Informs not gates. Expected-flake vs sentinel-flake enumerated.                                    |
| 8  | Test-churn (G8)                   | 11 tests: 5 DELETE / 6 REWRITE (1 MONITOR). Distributed across S4/S5/S6. S3 stays single-session.  |
| 9  | Clearance spec fields (G6)        | Split form everywhere. 8 new fields + 3 deletes. M4 = 0.30×1.00 mm, escalate-on-bind.              |
| 10 | S2-A/B/C                          | f64 confirmed; affine-12 picked; no output re-weld.                                                |
| 11 | Mitigations carryover             | Pinned dep; 3 plumbing helpers; cold-build doc note.                                               |

## Cross-references

- `docs/CF_CAST_MATING_FEATURES_PLAN.md` — arc cadence (dev `ba74b1a8`).
- `docs/CF_CAST_MATING_FEATURES_BOOKMARK.md` — S0 inventory + S1 ADR.
- Source: `design/cf-cast/src/{spec,mesher,piece,registration,plug,pour,funnel,platform}.rs`,
  `mesh/mesh-offset/src/grid.rs`, `mesh/mesh-printability/src/issues.rs`.
- External: `manifold3d` 0.1.8, `manifold-csg` 0.1.8, `manifold-csg-sys`
  3.4.107 (vendors upstream `elalish/manifold` `@5f95a3ac`).
- Memory: `project_cf_cast_mating_features_plan`,
  `project_cf_cast_mating_features_s0_bookmark`,
  `project_cf_cast_mating_features_s1_adr`,
  `project_cf_cast_f4_split_asymmetry`,
  `feedback_cold_read_two_passes_for_non_trivial_diffs`,
  `feedback_default_numeric_vs_derived_geometry`.

## Cold-read findings (applied in-place)

Per `feedback_cold_read_two_passes_for_non_trivial_diffs`, two passes
applied same session. Hooks below for the future audit trail.

**Pass-1 (sentence-level):**

- **C1-A** — §5 transform-sequence Vec contained a wrong-direction
  `UnionCylinder` line for the cup-side T-slot, parenthetically
  "corrected" in prose underneath. Rewrote the Vec with the actual
  `SubtractCylinder` ops; folded the per-piece explanation into the
  lead-in.
- **C1-B** — §2 test asserted `tris_a == tris_socket` (pin and
  socket of *different* radii share triangulation indices) — a
  strong claim about manifold3d's cylinder primitive's
  radius-invariance that the spike didn't measure. Removed the
  third assertion; added a comment clarifying that the cross-piece
  fit invariant rests on same-radius bit-equality + shared
  affine-12 pose, not radius-invariant triangulation.
- **C1-C** — §5 test name `t_bar_halves_share_a_planar_co_seam_face`
  was clumsy. Renamed to `t_bar_halves_share_coplanar_seam_face`.
- **C1-D** — §6 "per §M5/M6" referenced this doc's nonexistent
  M-sections; meant S0 bookmark's M5/M6 entries. Fixed.
- **C1-E** — §11 header was unnumbered; Decision Summary table
  referenced §11. Added §11 prefix to the header.
- **C1-F** — §10 S2-B's prose described
  `Rotation3::rotation_between(z, parent.axis)` without
  acknowledging the `Option<None>` antipodal edge case. Added a
  parenthetical noting the builder picks a deterministic
  perpendicular for the 180° case.

**Pass-2 (structural / contradictions):**

- **C2-A** — §1's `MatingTransform` enum example uses `CylinderParams`,
  but §2 introduces `CylinderParent` with overlapping but smaller
  shape. Pass 1 didn't catch the under-explanation. Added a closing
  parenthetical to §1's enum bullet tying `CylinderParams` to
  `CylinderParent`-plus-per-op-fields and noting S3 picks the exact
  decomposition.
- **C2-B** — §1's "Transforms are applied in declared order" example
  cited the WRONG order (`T-bar union ⊳ seam trim ⊳ pins`) — §5's
  bisection mechanism requires `cylinder CSG ⊳ then seam trim`, so
  pins must precede seam-trim too. Updated to
  `T-bar carve ⊳ pins ⊳ seam trim`, matching §5's transform Vec.
- (Pass-2 verified `IndexedMesh.faces: Vec<[u32; 3]>` in
  `design/cf-geometry/src/mesh.rs:34`, supporting the §10 S2-C
  `u64 → u32` narrowing claim — no edit needed.)

## Status

- **2026-05-22 — S2 recon drafted + cold-read pass 1 applied.**
  C1-A/B/C/D/E/F resolved.
- **2026-05-22 — Cold-read pass 2 applied.** C2-A/B resolved.
  Recon ready for S3 to execute against.
