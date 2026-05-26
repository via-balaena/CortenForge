# cf-cast §Q-4 planar-trim primitives recon (2026-05-26)

**Status:** scaffold. Triggered by workshop user's cf-view smoke on
the post-§Q-1-fix 2 mm full-config STLs
(`~/scans/cast_iter1_q1_phase1_2mm_full/`,
[[project-cf-cast-geometry-crispness-q1-fix]]):

> *"Mating face doesn't look perfectly flat, flange isn't all the
> way around, but pocket for plug stem is looking much better.
> Resolution is looking really solid too. Steps in the right
> direction. I dont think i will do a pr until we get the flange all
> the way around, interior mold of floor shell flat, bottom of plug
> flat, and flat mating faces for each mold half."*

This recon addresses 3 of 4 PR-blockers via post-MC mesh-CSG
planar-trim primitives applied at the cap-plane + seam plane:

1. **Interior mold floor flat** (cup-piece cavity floor at Z=cap_z)
2. **Bottom of plug flat** (plug's bottom face at Z=cap_z)
3. **Flat mating faces** (cup-piece seam-plane face at Y=±RIBBON_PIECE_OVERLAP_M)

The 4th PR-blocker (flange-perimeter continuity) is a separate arc;
see [[project-cf-cast-flange-perimeter-continuity-bookmark]].

**Picked architectural direction:** OFFSET-trim variants of the
existing `build_plug_cap_trim_transform` + `build_cup_cap_trim_transform`
defensive primitives (`plug.rs:407` + `plug.rs:445`), trimmed by a
small ε into the body's pre-trim material so the resulting face is
CONTAINED inside existing geometry (paradigm-safe per the framework
in [[project-cf-cast-sdf-meshcsg-paradigm-boundary]]) rather than
face-coincident with the SDF cap-plane (the failure mode that
disabled the primitives 2026-05-24).

## Cross-arc decision staleness

The [[project-cf-cast-cap-plane-flatness-bookmark]] (`06286520`
morning + same-day update) shipped a (4') accept+document decision
at 3 mm MC cells. The argument was that the cap-plane edge chamfer
is ≤100 µm (below Bambu A1's ~200 µm print resolution), so the
chamfer wouldn't translate to print. **The (4') decision DOES NOT
hold at 2 mm cells** (post-§Q-1-fix scale):

- Chamfer band scales roughly with cell size — at 2 mm cells the
  chamfer is ~2 mm, **above** Bambu A1 print resolution and
  workshop-visible per cf-view smoke.
- Workshop user explicitly flagged it as PR-blocking.
- The bookmark's "doubly contraindicated" framing was specifically
  about *slab subtract face-coincident with SDF cap-plane*. The
  `trim_by_plane` approach (halfspace cut, NOT slab subtract)
  produces ONE new face, not two coincident faces. The
  paradigm-boundary risk is lower-bound, manageable via offset
  bias.

The (4') decision is correctly REOPENED at 2 mm scale. This recon
is the corrective surface, picked up by the §Q-1 fix's unblock of
finer cells.

**Out of scope:**

- §Q-4 candidate (1) "pre-MC SDF halfspace intersect at cap-plane"
  — already implemented in `pinned_floor_shell` per the bookmark's
  (5') verification. Adding another intersect is a no-op.
- §Q-4 candidate (2) "adaptive MC near cap-plane" — substantially
  larger scope (mesh-offset changes). Bookmarked but not pursued
  here.
- Flange-perimeter continuity — see
  [[project-cf-cast-flange-perimeter-continuity-bookmark]];
  separate §F arc, independent of this recon.
- §Q-6 cup-piece flange asymmetry — needs workshop 360° cf-view
  rotation; orthogonal.
- Modifying body's SDF (e.g., changing `pinned_floor_shell` to
  give body a Z-distance SDF below cap-plane) — invasive,
  potentially breaks scan-mesh-SDF consumers.

## §Q4-0 Why this recon (workshop empirical signal)

Cf-view smoke on `~/scans/cast_iter1_q1_phase1_2mm_full/`:

| Workshop observation | STL examined | Geometric feature | Root cause |
|----------------------|--------------|-------------------|------------|
| Patches faded ✅ | cup-piece `mold_layer_*_piece_*.stl` | cup-wall outer surface | §Q-1 fix shipped |
| Plug stem pocket cleaner ✅ | cup-piece interior | plug-floor-lock socket cavity | §Q-1 fix shipped |
| Interior mold floor not flat ❌ | cup-piece cavity floor | Z=cap_z plane (body's bottom face) | MC quantization at cap-plane × body-lateral kink in SDF gradient → **§Q-4 cap-slab union FULLY ADDRESSES** |
| Bottom of plug not flat ❌ | `plug_layer_*.stl` bottom | Z=cap_z plane (plug body's bottom) | Same root cause as cup cavity floor; per-plug instance → **§Q-4 plug cap-plane trim FULLY ADDRESSES** |
| Mating face not flat ❌ | cup-piece seam-plane face | Y=±RIBBON_PIECE_OVERLAP_M plane | TWO root causes: (a) potential halfspace SDF overshoot post-§Q-1 union with FlangeSdf — **§Q-4 SeamTrim DEFENSIVE PRIMITIVE**; (b) cup-wall outer MC edge stair-stepping driving visible jaggedness — **§Q-4 DOES NOT ADDRESS**, follow-up arc gated on workshop S4 cf-view smoke |
| Flange not all the way around ❌ | cup-piece flange ring | flange's outer-edge perimeter | `FlangeSdf::eval`'s 3D-projected body_dist fails in body concavities — separate arc |

## §Q4-1 SDF root cause analysis

### §Q4-1-a Cap-plane × body-lateral SDF kink

`pinned_floor_shell` (`design/cf-design/src/solid_layered.rs:122-130`)
composes the body as `body_open.intersect(halfspace_above_cap)`.
For points NEAR the cap-plane (Z = cap_z), `body.evaluate` returns
the SIGNED Euclidean distance to the nearest body surface — which
near cap_z is either the cap-plane (Z-distance) or the body's
lateral surface (lateral distance), whichever is closer.

This creates a non-Lipschitz SDF at the cap-plane × lateral-surface
intersection (the body's bottom edge): the gradient flips between
±Z direction (near cap-plane) and ±lateral direction (near lateral
surface). MC linear interpolation across grid edges that span this
kink produces vertex placements that don't land exactly on cap_z.

Empirical illustration — for an MC edge from sample at Z = cap_z +
1 mm (sdf = -25 mm, taking the LATERAL distance because the
lateral surface is far away but I'm AT the corner so cap-plane is
ALSO 1 mm) to sample at Z = cap_z - 1 mm (sdf = +1 mm, below
cap-plane), the interpolated vertex lands at:

```
t        = sdf_a / (sdf_a - sdf_b) = -25 / -26 = 0.962
Z_vertex = Z_a + t * (Z_b - Z_a) = (cap_z + 1) + 0.962 * (-2)
         = cap_z - 0.923 mm
```

The vertex lands ~1 mm BELOW cap_z (undershoot). For samples
where lateral distance < Z-distance, the SDF magnitude flips and
MC can also OVERSHOOT above cap_z. The direction depends on grid
alignment with cap_z AND on which surface is closest at each
sample (Z vs lateral).

**Empirical observation from `~/scans/cast_iter1_q1_phase1_2mm_full/`
cf-view smoke**: workshop user reports cavity floor + plug bottom
are visibly stair-stepped at 2 mm MC cells. The direction (under
or overshoot) at any specific lateral position is grid-alignment-
dependent; a robust fix must handle both directions.

For deep-inside-body samples where Z-distance dominates (i.e.,
lateral surface is FAR compared to cap-plane), MC interpolation
across cap_z is approximately linear and produces near-exact
cap-plane vertices. The stair-stepping concentrates near body's
bottom edge (where the kink is sharpest).

### §Q4-1-b Mating face overshoot

`ribbon::halfspace_solid` produces a linear halfspace SDF; MC
interpolation across the halfspace cut is f64-precise per recon-4
(P) §F-4. The mating face *should* be mathematically flat. Why
does cf-view show stair-stepping?

Hypothesis: the stair-stepping is at the EDGE between mating face
(flat at Y=±0.5 mm) and cup-wall outer (curved, MC-quantized).
At 2 mm cells, the cup-wall outer's MC vertices have ±0.5 mm
deviation from the ideal smooth surface. These deviations propagate
into the mating face's EDGE where it meets the cup-wall outer,
producing visible stepping along the perimeter. The face itself is
flat; the edge is jagged.

`trim_by_plane(Y=±0.5 mm)` cleanups the edge by truncating any MC
overshoot past the halfspace cut (small but possible due to
non-Lipschitz SDF compositions involving the union with
`CupWallShellSdf`).

### §Q4-1-c Why §Q-1's shell SDF didn't help

§Q-1's `CupWallShellSdf` fixed the cuboid-face × flange-interior
MC ambiguity. It did NOT change the body's SDF (still uses
`pinned_floor_shell` with cap-plane intersect). The cap-plane ×
body-lateral kink is independent of cup-wall composition. §Q-4 is
orthogonal to §Q-1's scope.

## §Q4-2 Paradigm-boundary analysis

The [[project-cf-cast-sdf-meshcsg-paradigm-boundary]] framework
identifies three patterns for mesh-CSG operations against SDF→MC
hosts:

- **CONTAINED**: primitive lives entirely inside host with no
  surface coincidence. Safe.
- **PROTRUDING**: primitive lateral surface intersects host at a
  circle/edge (not a face). Safe.
- **WELDED-TO-BULK**: primitive face coincident with host face.
  **Dangerous** — manifold3d produces topologically-ambiguous
  output.

### Why the 2026-05-24 disabling fired BRANCH C

`build_plug_cap_trim_transform` emits a `SeamTrim` (halfspace cut)
with normal = -cap_normal and offset = `cap_normal · cap_centroid`.
The kept half-space is `+kept_normal` side = above cap-plane (for
plug). The trim plane EQUALS the body's cap-plane (`pinned_floor_shell`
intersect at the same Z). `trim_by_plane` produces a face at this
plane. The face IS the body's pre-MC cap-plane surface.

Per the recon-4 (P) §F-2 framework: this is WELDED-TO-BULK. The
new face shares the same geometric position as the existing pre-MC
cap-plane intersect's MC-output face. manifold3d's boolean has
ambiguous handling here → F4 Critical issue.

### The offset-bias workaround (per the framework)

The §G-7 plug-shaft fix (`extend_near_end` 1 mm overlap-bias) and
the §F-1 seam-flange grid-bias (1 µm) both pattern: **shift the
boundary slightly into the host's interior** so the resulting face
is CONTAINED, not WELDED.

For cap-plane trim with offset ε > 0 (plug only):
- `trim_by_plane(normal=+cap_normal, offset = (cap_normal · cap_centroid) + ε, keep_above)`
  → keeps Z > cap_z + ε. Plug's new bottom face is at Z = cap_z + ε.
  Plug-side trim cleanly removes the MC-undershoot region below
  cap_z and the MC-overshoot region between cap_z and cap_z + ε.

For the cup-side cavity floor, `trim_by_plane` doesn't work — it
cuts globally and would remove either the cup-walls or the cup-
piece's bottom slab. The cup-side fix uses a different primitive
(slab union); see §Q4-3-b.

ε magnitude trade-off:
- ε = 0: face-coincident → BRANCH C failure (the disabled state).
- ε too small (< MC numerical noise ~ 1 nm): may still trigger
  manifold3d ambiguity.
- ε too large: workshop-visible (cf-view shows plug bottom
  shifted up by ε).
- **Pick: ε = 0.001 mm (1 µm)**. Below FDM print resolution by
  200×; below mesh-CSG numerical noise of manifold3d's f64
  internal precision by 6+ orders of magnitude. Same magnitude as
  the seam-flange S1's pre-§Q-1 grid-bias attempt; that attempt
  failed for §Q-1 but for a different reason (the bug wasn't
  grid-coincident at all). For paradigm-boundary purposes 1 µm
  is the empirically-proven-safe overlap-bias scale.

## §Q4-3 Per-sub-problem architectural decisions

### §Q4-3-a Plug bottom flat

**Picked**: re-enable `build_plug_cap_trim_transform` with offset
ε = +1 µm into the plug body (above cap-plane in the kept
half-space sense).

Implementation:
- Modify `build_plug_cap_trim_transform` to accept an optional
  offset parameter (or hardcode the 1 µm bias inside).
- In `add_plug_pins`, re-add the trim to the `transforms` Vec
  (currently disabled per `plug.rs:482-498`).
- Trim is applied POST-MC via `apply_mating_transforms` (existing
  pipeline; uses `manifold3d::trim_by_plane_nalgebra`).

Workshop-visible effect: plug's bottom face becomes mathematically
flat at Z = cap_z + 1 µm. The 1 µm offset is below FDM resolution
→ the workshop user sees a flat plug bottom matching the cap-plane.

### §Q4-3-b Cup-piece cavity floor flat

**This is the hardest of the three sub-problems.** The cavity floor
is an INTERIOR cup-piece surface (the upper side of the cup-piece
material, where the cavity opens). `trim_by_plane` is a halfspace
cut — applied globally to the cup-piece mesh, it removes EITHER
the cup-walls (kept_below) OR the cup-piece's bottom slab
(kept_above). Neither alone flattens just the cavity floor; both
break the cup-piece geometry.

**Three candidate approaches:**

#### Candidate A: Cap-slab union (PICKED)

Construct a flat slab manifold:
- Lateral extent: covers body's AABB at cap_z + a small margin
  (e.g., +0.5 mm).
- Z extent: [cap_z - cell_size, cap_z + 1 µm].
- Mesh-CSG `union` with cup-piece manifold POST-MC.

The slab's top face at Z = cap_z + 1 µm dominates the cup-piece's
stair-stepped material in body's lateral footprint. The slab
thickness must exceed the MC vertex-undershoot magnitude (~cell_size
at the body-bottom-edge corner) so the slab covers the cavity-floor
MC region completely.

**Why union, not subtract**: MC produces cup-piece cavity-floor
vertices in BOTH directions around cap_z (under- and overshoot,
grid-alignment-dependent per §Q4-1-a). A subtract above
cap_z + 1 µm would only address overshoot (cutting off bumps);
undershoot dips would remain. A union slab at Z ∈
[cap_z - cell_size, cap_z + 1 µm] handles both: where MC produced
overshoot bumps, union's `min(slab, piece)` keeps the slab interior
up to its top at cap_z + 1 µm and cuts the bumps off implicitly
(the bumps become INTERIOR to the union, no longer a boundary);
where MC produced undershoot dips, the slab fills them up to
cap_z + 1 µm. Result: cavity floor flat at cap_z + 1 µm in both
cases.

**Paradigm-boundary analysis**:
- Slab's TOP face at Z = cap_z + 1 µm, lateral extent
  (body_AABB + margin): this is the new flat cavity floor.
  Coincident with NOTHING (the body's SDF cap-plane is at cap_z;
  slab top is 1 µm above; the original MC-output cup-piece cavity
  floor is at varying Z around cap_z; slab top is everywhere
  ABOVE these). CONTAINED, paradigm-safe.
- Slab's LATERAL faces (±X, ±Y at body_AABB + margin): for points
  in the lateral-margin region (between body's lateral outer and
  slab's lateral extent), cup-wall material exists for Z > cap_z
  (cup-wall extends up from body's outer surface for all Z).
  Slab's lateral face is INSIDE cup-wall material → CONTAINED.
- Slab's BOTTOM face at Z = cap_z - cell_size: overlaps cup-piece's
  existing bottom slab material (which extends from cap_z down to
  cap_z - wall_thickness). Union keeps both interior; no new
  surface created at slab bottom (it's interior to cup-piece
  material there).

**Implementation:**
- New `MatingTransform::UnionCuboid { params: CuboidParams }`
  variant in `mesh_csg.rs`.
- `CuboidParams` = half-extents (Vector3) + center (Point3) +
  optional rotation (UnitQuaternion).
- `apply_one` arm uses `Manifold::cuboid` primitive + transform.
- New `build_cavity_floor_slab_transform(ribbon, layer_body)` in
  `piece.rs` emits the union for each cup-piece (uses
  `ribbon.pour_end_hint` for cap-plane Z + `layer_body.bounds()` for
  body's lateral AABB at cap-plane × wall_thickness for slab
  thickness × MC_BOUNDS_PAD_M-scale margin).

#### Candidate B: Above-cap-plane subtract (rejected)

A subtract of a cuboid above Z = cap_z + 1 µm × body's lateral
footprint would address ONLY the overshoot direction (cutting off
bumps). Per §Q4-1-a, MC's vertex displacement near cap_z is grid-
alignment-dependent and frequently produces undershoot — subtract
would be a no-op in undershoot regions, leaving them stair-stepped.

#### Candidate C: Modify body's SDF (rejected)

Change `pinned_floor_shell` (or wrap it) so the body's SDF below
cap_z is `+(cap_z - P.z)` (pure Z-distance, ignoring lateral
position). This would eliminate the SDF kink at cap_z × body-
lateral intersection.

**Pro:** cleanest mathematical fix; addresses cavity floor + plug
bottom in one change (plug uses the same body SDF).

**Con:** invasive change to `pinned_floor_shell`; may break other
consumers (scan-mesh-SDF tests; cf-mesh-shell; sim-soft); the body
geometry "below cap-plane" semantics changes. Out of §Q-4 scope;
bookmarked for a future arc if §Q-4's mesh-CSG approach surfaces
limitations.

### §Q4-3-c Mating face flat — partial fix (defensive primitive)

**Honest scope statement**: workshop user's "mating face not flat"
complaint may have TWO independent root causes, and §Q-4 only
addresses one of them:

1. **Halfspace SDF overshoot** (addressed by §Q-4): the §F-4 audit
   (recon-4 (P)) established seam-cap vertices ARE bit-precise on
   the halfspace plane FOR the bare cup-piece SDF
   (`bounding ∖ body ∩ halfspace`). Post-§Q-1 the SDF is
   `shell ∩ halfspace [∪ flange]`; the union with `FlangeSdf`
   could introduce sub-ulp overshoot past the halfspace plane via
   floating-point arithmetic interactions. A defensive
   `MatingTransform::SeamTrim` clipping at Y = ±(overlap − 1 µm)
   guarantees no overshoot regardless. **Near-no-op in the
   common case; paradigm-safe defensive primitive.**

2. **Cup-wall outer MC edge stair-stepping** (NOT addressed by
   §Q-4): the visible jaggedness at the mating-face perimeter in
   the cf-view smoke is driven by the cup-wall outer's MC
   quantization at 2 mm cells — the cup-wall's curved outer surface
   has ±0.5 mm deviations that propagate into the OUTLINE of the
   mating face. The mating-face PLANE is mathematically flat per
   §F-4; the visible jaggedness is its EDGE where the plane
   intersects the stair-stepped cup-wall outer. A planar trim of
   the mating face doesn't smooth the cup-wall outer's MC vertices.

**Addressing root cause #2 requires a separate arc** — either
finer MC cells (current wall-clock prohibitive even after §Q-1
fix), adaptive MC near the seam plane, an offset-bias on the
cup-wall outer to make it locally planar near the seam plane, or
a different meshing algorithm (dual contouring). Bookmark this as
a follow-up arc when the workshop user has prioritized.

**§Q-4 ships the SeamTrim defensive primitive** for root cause #1
because it's a near-zero-cost paradigm-safe primitive that
guarantees no halfspace overshoot regardless of upstream SDF
arithmetic changes. Workshop user should expect §Q-4 to leave the
mating-face EDGE jaggedness mostly unchanged (the §Q-1 fix at 2 mm
cells is the current best-attainable smoothness without a finer
MC cell size or a separate arc).

**Implementation:**
- New `build_mating_face_trim_transform(ribbon, side)` in `piece.rs`
  emits the SeamTrim per side.
- Append to `compose_piece_solid`'s `transforms` Vec.
- Per-side semantics: Negative side trims at Y = -RIBBON_PIECE_OVERLAP_M + 1 µm
  (kept side: Y >= -overlap + 1µm); Positive side trims at
  Y = +RIBBON_PIECE_OVERLAP_M - 1 µm (kept side: Y <= +overlap - 1µm).

## §Q4-4 API surface

### New `MatingTransform` variant

```rust
pub enum MatingTransform {
    // ... existing variants ...
    /// §Q-4 cap-plane slab: unions a flat-top cuboid into the
    /// cup-piece mesh covering the body's lateral footprint at
    /// cap_z, flattening the cup-piece's cavity floor at the
    /// cap-plane regardless of MC undershoot OR overshoot
    /// direction.
    UnionCuboid { params: CuboidParams },
}

pub struct CuboidParams {
    pub center_m: Point3<f64>,
    pub half_extents_m: Vector3<f64>,
    pub rotation: Option<UnitQuaternion<f64>>,
}
```

### Re-enabled emitters

- `plug.rs::add_plug_pins`: re-adds `build_plug_cap_trim_transform`
  result to the `transforms` Vec.
- `plug.rs::build_plug_cap_trim_transform`: gain a 1 µm offset bias
  (toward the kept side) in the emitted `SeamTrim::offset_m`.
- `piece.rs::compose_piece_solid`: adds two new transform
  emissions (cap-plane cavity-floor slab via new
  `build_cavity_floor_slab_transform`; mating face trim per side
  via new `build_mating_face_trim_transform`).

### Determinism contract

- `CuboidParams` mirrors the existing `CylinderParent` /
  `PrismaticPinParams` patterns: bit-identical params → bit-
  identical mesh-CSG output across calls.
- Offset biases (1 µm) are named constants, not magic numbers, in
  the relevant modules' `pub(crate) const` block.

## §Q4-5 cf-cast-cli interaction

Transparent — no new config knobs. The cap-plane source is
`ribbon.pour_end_hint` (already plumbed from `cast.toml`'s
`[scan].prep_toml` → `pinned_floor_shell`'s cap_planes). Mating
face uses the existing `RIBBON_PIECE_OVERLAP_M` constant.

If a future iter surfaces a need to disable individual primitives
(e.g., debugging), a `[planar_trim]` block with per-primitive
`enabled` toggles could be added; not in scope for S1.

## §Q4-6 Test coverage

### Per-primitive emitter tests

- `build_plug_cap_trim_transform` returns `Some(SeamTrim)` with
  offset bias when `ribbon.pour_end_hint` is set; returns `None`
  when unset.
- `build_cavity_floor_slab_transform` returns `Some(UnionCuboid)`
  with center at `(body_lateral_aabb_center.x, body_lateral_aabb_center.y,
  cap_z - 0.5 * slab_thickness + 1 µm)`; half-extents = body lateral
  half-extents + margin (X, Y) × 0.5 × slab_thickness (Z). Slab
  thickness >= cell_size to cover MC undershoot region.
- `build_mating_face_trim_transform` returns `Some(SeamTrim)` per
  side with the halfspace's normal + biased offset.

### Determinism tests

- Same fixture → bit-identical transform params across calls.
- Same body bounds + ribbon → bit-identical CuboidParams.

### Integration tests (post-MC mesh validation)

- After `apply_mating_transforms` with new variants: cup-piece mesh
  is still watertight + manifold (via `mesh-repair::validate_mesh`).
- Plug mesh's bottom face Z-coordinate is within ε of cap_z + 1 µm
  (post-trim flatness check).
- Cup-piece's cavity-floor surface (selected by Z-bounded face
  filter) has all vertices within ε of cap_z + 1 µm.

### Production-regen gates

- 2 mm full config: workshop user cf-view smoke gates cavity-floor
  + plug-bottom + mating-face visual flatness.
- §R1 connectivity inspector: cup-piece + plug remain 1-component
  each.
- F4 print-validation: no Critical issues introduced by the new
  transforms.

## §Q4-7 Open questions

1. **Should `CuboidParams` include rotation, or assume axis-aligned?**
   - Axis-aligned simpler; cap-plane is Z-aligned by convention.
   - Recon §F-1 simplification (seam plane Y-hardcoded) sets
     precedent for axis-aligned simplification.
   - **Default pick**: rotation `Option<UnitQuaternion>` with
     `None` meaning axis-aligned; cap-plane subtract uses `None`.

2. **Mating face trim — keep the existing `RIBBON_PIECE_OVERLAP_M`
   convention or shift it?**
   - Pre-§Q-4: cup-piece overlaps 1 mm at seam (each side biased
     `0.5 mm` past Y=0). Post-§Q-4 with 1 µm trim bias: each side
     trimmed at Y = ± (0.5mm - 1µm), effective overlap = 1mm - 2µm
     (workshop-invisible change).
   - **Default pick**: 1 µm bias toward kept side; overlap
     effectively unchanged.

3. **Per-piece vs per-layer cap-plane Z?**
   - In production, all layers share the same cap-plane (from
     `sock_over_capsule.prep.toml`). Plug-per-layer means each
     plug has the same Z=cap_z bottom.
   - But layer body bounds DIFFER per layer (cumulative offset).
     Cavity-floor subtract cuboid's lateral extent must match
     EACH layer's body lateral AABB.
   - **Default pick**: `build_cavity_floor_slab_transform`
     takes the per-layer body Solid; reads body's lateral AABB at
     cap_z.

4. **Should cap-plane trim apply to gasket molds + platform +
   funnel?**
   - Gasket molds are flat trays — no cap-plane issue.
   - Platform is a flat slab — already flat.
   - Funnel has its own cap-plane (top opening); existing
     `build_plug_cap_trim_transform` doesn't apply.
   - **Default pick**: scope §Q-4 to cup-pieces + plugs only.

5. **F4 gate behavior with cuboid union on cup-piece?**
   - Union operation post-MC adds a (small) cuboid to the mesh-CSG
     pipeline. F4 self-intersect check is O(face²) pre-BVH; per
     [[project-cf-cast-self-intersect-bvh-s1]] now O(n log n).
     Should handle the additional faces without blowup.
   - **Empirical S2 gate**: measure F4 wall-clock pre/post §Q-4
     emission; expect <10 % regression.

6. **Behavior change vs. backward-compat?**
   - The (4') accept+document procedure.rs prose added in
     `06286520` describes the cap-plane edge chamfer as workshop-
     acceptable. After §Q-4 ships, this prose is obsolete +
     should be REMOVED in S3 (procedure.rs cleanup).
   - **Default pick**: include the procedure.rs prose removal as
     S3 scope; the (4') decision is officially superseded by §Q-4.

7. **Mating-face edge stair-stepping: does §Q-4 unblock the
   workshop user's PR-merge gate, or do we need a separate arc for
   the cup-wall outer MC quantization?**
   - Per §Q4-3-c, §Q-4 ships the `SeamTrim` defensive primitive
     for halfspace overshoot but does NOT smooth the cup-wall
     outer's MC vertices that drive the visible edge jaggedness.
   - Workshop user might accept this if (a) the edge jaggedness is
     bounded by ~cell_size at 2 mm and (b) FDM printing further
     smooths it via the 0.4 mm extrusion width.
   - Empirical S4 cf-view smoke gate will surface whether §Q-4's
     scope is sufficient. If not, follow-up arc options: finer MC
     cells globally (currently wall-clock-prohibitive), adaptive
     MC near the seam plane, a cup-wall-outer flange-like primitive
     that's locally planar near the seam, or a different meshing
     algorithm (dual contouring).
   - **Default pick**: ship §Q-4 with current scope; defer
     cup-wall outer smoothing to a follow-up arc gated on
     workshop user's S4 cf-view smoke feedback.

## §Q4-8 Implementation arc

Estimated 4-5 phases, ~3-5 sessions:

### S1: Probe spike — paradigm-boundary safety verification

**Scope**: ~50 LOC. Add ONLY the plug-bottom trim (re-enable
`build_plug_cap_trim_transform` with 1 µm offset bias). Run 2 mm
+ 3 mm full-config regens. Workshop user cf-view smoke confirms
plug bottom flat + no new F4 Critical issues.

**Outcome:** if S1 produces clean output, the 1 µm offset bias
hypothesis is validated → S2 can apply the same pattern to cavity
floor + mating face. If S1 fails (F4 Critical OR cf-view shows
new artifacts), revisit the offset magnitude OR fall back to
Candidate C (modify body SDF).

**Falsification gate**: F4 print-validation passes on both 2 mm +
3 mm regens; workshop cf-view confirms plug bottom is flat.

### S2: Cavity-floor slab union primitive

**Scope**: ~150 LOC. New `MatingTransform::UnionCuboid` variant
+ `CuboidParams` + `apply_one` arm in `mesh_csg.rs`;
`build_cavity_floor_slab_transform` in `piece.rs`; emit per
cup-piece in `compose_piece_solid`. Tests + production regen.

**Falsification gate**: workshop cf-view confirms cavity floor
flat at cap_z; cup-piece §R1 connectivity intact (1 component);
F4 clean.

### S3: Mating-face trim primitive

**Scope**: ~50 LOC. New `build_mating_face_trim_transform` in
`piece.rs`; emit per-side SeamTrim in `compose_piece_solid`. Tests
+ production regen.

**Falsification gate**: workshop cf-view confirms mating face edge
cleanup; cup-piece §R1 connectivity intact; F4 clean.

### S4: Production iter-1 regen + cf-view smoke + procedure.rs cleanup

**Scope**: production regen on `~/scans/cast.toml` at 2 mm + 3 mm
with all §Q-4 primitives enabled; workshop user cf-view smoke
gates all three sub-problems; procedure.rs §Cap-Plane Edge Chamfer
prose REMOVED (the (4') decision is superseded).

### S5: Cold-read pass + commit

**Scope**: cold-read pass on all §Q-4 commits; final commit + this
recon doc's status update to FIXED.

## §Q4-9 Prior-arc memory checklist

Per [[feedback-read-prior-arc-memory-before-architectural-decisions]]
rules 1-6, the following memories MUST be read before any §Q-4
phase touches the primitive in question:

- [[project-cf-cast-cap-plane-flatness-bookmark]] — the (4')
  accept+document decision this recon supersedes; rule #6 applies:
  the (5') BRANCH A bisect outcome must NOT be silently reverted.
- [[project-cf-cast-sdf-meshcsg-paradigm-boundary]] — paradigm-
  boundary framework + the 2026-05-24 disabling rationale for
  `build_plug_cap_trim_transform`. Rule #6: must NOT silently
  revert the disabling without offset-bias safeguard.
- [[project-cf-cast-geometry-crispness-q1-fix]] — §Q-1 sibling fix;
  shell SDF + mc_bounds plumbing. §Q-4 emissions must work with
  the new shell-based cup-piece SDF.
- [[project-cf-cast-seam-flange-s1]] — flange composition; §Q-4
  cavity-floor subtract must not interact with the flange's union
  in `compose_piece_solid` (flange lives at Y=±4 mm thickness, way
  above any cap_plane Z range — should be paradigm-safe by
  construction).
- [[project-cf-cast-self-intersect-bvh-s1]] — F4 self-intersect
  check is now O(n log n) post-BVH; additional cuboid subtract
  faces shouldn't blowup F4 wall-clock.

## Successor

S1 probe spike picks up next. Workshop user picks empirical-vs-
analytical first-step (probe spike empirically falsifies the 1 µm
offset hypothesis; analytical would require working through the
paradigm-boundary math more carefully). Per the §Q-5 + §Q-1
pattern, **empirical falsification of the smallest hypothesis is
the cheapest first step** — S1 probe is the right scope.
