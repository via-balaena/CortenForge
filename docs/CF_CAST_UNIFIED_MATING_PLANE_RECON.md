# cf-cast unified-mating-plane recon

> **Status**: SCAFFOLD (cold-read pending before commit).
> **Trigger**: workshop user cf-view smoke 2026-05-27 on §F-S2
> production regen (`~/scans/cast_iter1_flange_continuity_s2_clipped/`)
> + Y-bin STL diagnostic: cup-piece mating face is a STEPPED surface
> with 0.5 mm offset between cup-wall portion (y=-0.5 mm) and flange
> portion (y=0 mm), preventing the two halves from seating flush.
> Workshop user articulated a structural simplification: single flat
> mating plane + symmetric dowel-hole registration + perimeter flange.
> Predecessors: [[project-cf-cast-flange-continuity-s1]],
> [[project-cf-cast-flange-continuity-bolt-pattern-recon]],
> [[project-cf-cast-registration-pin-disconnection-impl]].
> Cold-read discipline: per
> [[feedback-cold-read-two-passes-for-non-trivial-diffs]] + prior
> recon cold-reads catching ~10 substantive issues each pre-commit,
> this doc gets a pass-1 cold-read before commit + pass-2 at S1
> kickoff. Workshop user surfaces decisions, recon enumerates them
> per [[feedback-keep-asks-bold-and-short]] — no unilateral picks on
> workshop-relevant trade-offs.

## §M-1 Problem statement

The current cup-piece architecture composes the seam face from
multiple SDF terms with *different* halfspace overlap settings:

- **Cup-wall** (post-§Q-1 body-tracking shell): cut by
  `ribbon.halfspace_solid(side, mc_bounds, RIBBON_PIECE_OVERLAP_M)`
  with `RIBBON_PIECE_OVERLAP_M = 0.5 mm`. Mating face at cast
  `y = -0.5 mm` (Negative side).
- **Flange** (post-§F-S1 silhouette-based slab): cut by
  `ribbon.halfspace_solid(side, bounds, 0.0)` — zero overlap. Mating
  face at cast `y = 0 mm`.

The two halfspace cuts use intentionally different overlap values:
the cup-wall overlap exists so the two halves overlap 1 mm at the
body cavity rim (PLA-on-PLA compression backstop); the flange's zero
overlap exists so the two halves' flange portions butt flush at the
seam plane (gasket-precision mating).

**Result**: the cup-piece mating face is a STEPPED surface — inner
region (silhouette to silhouette + `wall_thickness_m=5 mm`) sits 0.5 mm
deeper into the negative half than outer region (silhouette + 5 mm to
silhouette + `flange_width_m=15 mm`). When the two halves are clamped
with bolts (§B), the cup-wall portions over-engage by 1 mm (squeeze)
while the flange portions either gap by 0.5 mm (if cup-wall stays at
its design height) or the cup wall deflects elastically. Both modes
are workshop-undesirable.

Empirical confirmation (§F-S2 STL diagnostic 2026-05-27):

```
piece_1 (mold_layer_0_piece_1.stl) Y-bin distribution around mating face:
  dominant bin at print y = -3.847 mm: 2357 vertices  ← cup-wall mating face
  secondary bin at print y = -4.328 mm:   27 vertices  ← flange mating face
  separation: 481 µm ≈ RIBBON_PIECE_OVERLAP_M (500 µm)
```

The dominant bin is the cup-wall portion (large area, many MC vertices);
the secondary bin is the flange-only portion (smaller area where cup-wall
ends). Both are mathematically planar at their respective Y values; the
step between them is the structural design choice the recon proposes to
revisit.

Workshop user verbatim 2026-05-27: "the two mold should have flat
mating faces. i think honestly both sides should just have negative
pin holes, then we can just print dowels to lock them together with.
one[ce] the mating flat plane is defined, the flange is just around
the perimeter of the geometry for each a certain distance parallel
to the flat plane the join over". The proposed simplification
collapses three independent design intents (cup-wall overlap, flange
flush mating, registration pin/socket asymmetry) into a single
unified mating-plane architecture.

## §M-2 Diagnostic data (informs §M-3 candidate weights)

### §M-2-a STL Y-bin signature

From `~/scans/cast_iter1_flange_continuity_s2_clipped/mold_layer_0_piece_1.stl`,
1 µm Y-bin histogram of 45648 vertices (Python script in
[[project-cf-cast-flange-continuity-s1]]'s §F-S2 follow-up):

- Top bin `y = -3.8470 mm`: 2357 verts (cup-wall mating face).
- Top bin `y = -3.8410 mm`: 1956 verts (6 µm above top — likely
  flange portion of mating face mapped to print frame via the
  cast → print rotation; the 6 µm gap is the cast-to-print
  rotation's interaction with the 500 µm overlap separation).
- Cluster at `y = -4.328 mm` (27 verts) = top bin -481 µm. Matches
  `RIBBON_PIECE_OVERLAP_M = 0.5 mm` exactly. This is the flange
  mating face's print-coord image of the cast-y=0 plane.
- Cluster at `y = -3.362 mm` (5 verts) = top bin +488 µm. The
  opposite-direction flange mating face (cast y = 0 mapped through
  the other end of the cast-to-print rotation).

Step magnitude: ~500 µm = `RIBBON_PIECE_OVERLAP_M`. Confirms the
overlap-inconsistency hypothesis as the root cause.

### §M-2-b SDF code path

`design/cf-cast/src/piece.rs:306` (cup-wall halfspace):

```rust
let halfspace = ribbon.halfspace_solid(side, mc_bounds, RIBBON_PIECE_OVERLAP_M);
let mut base_piece = shell.intersect(halfspace);
```

`design/cf-cast/src/flange.rs:290-291` (flange halfspace — different
overlap):

```rust
let flange = build_flange_solid(layer_body, ribbon, spec, bounds);
flange.intersect(ribbon.halfspace_solid(side, bounds, 0.0))
```

Two different `overlap_m` arguments → two different mating-face Y
positions → stepped mating face after union.

### §M-2-c Pin/socket registration asymmetry

Per [[project-cf-cast-registration-pin-disconnection-impl]] + S5
mating-features architecture: registration is a `PrismaticPinSpec`
truncated-pyramid pair where the Negative-side piece gets a pin
(`UnionTruncatedPyramid`) protruding into +Y from its mating face,
and the Positive-side piece gets a socket (`SubtractTruncatedPyramid`)
into +Y from its mating face. The pin + socket are mirror images at
the seam plane; the workshop user joins the two halves by inserting
the Negative-side pin into the Positive-side socket.

This is asymmetric: each half has a different feature (pin vs
socket), and one feature is additive (UnionTruncatedPyramid) while
the other is subtractive (SubtractTruncatedPyramid). The workshop
proposal collapses this to a single symmetric primitive: a
through-the-mating-plane cylindrical HOLE on each half, with a
printed-separately PLA dowel inserted at assembly time.

Workshop benefits:
- Mirror-image symmetry between halves (same code, same MC, same STL
  structure modulo cup-half-specific cavity).
- Simpler post-MC mesh-CSG primitive (`SubtractCylinder` only, no
  `UnionTruncatedPyramid` paired complexity).
- Dowels are loose parts → no asymmetric pin/socket pairing risk if
  one side prints sloppily.

## §M-3 Mating-plane unification candidates

### §M-3-a Candidate A: zero overlap throughout (flush mating)

**Mechanism**: replace both `RIBBON_PIECE_OVERLAP_M = 0.5 mm` and the
flange's explicit `0.0` with a single unified `overlap_m = 0`. Both
cup-wall and flange halfspaces cut at cast `y = 0` exactly. Two halves
butt at `y = 0` with no overlap.

**Pros**:
- Single mating plane → guaranteed flush.
- Conceptually simpler (one halfspace cut, one mating face).
- Workshop's stated preference (flush mating).
- Gasket handles all sealing (it always did; the cup-wall PLA-on-PLA
  contact was a backup that physically doesn't engage when the
  gasket is at design compression).

**Cons**:
- MC robustness near the mating plane: two cells straddling `y = 0`
  on opposite sides emit vertices that could be coincident across
  pieces. Each piece is meshed INDEPENDENTLY so within-piece
  coincidence isn't an issue, but BETWEEN-piece coincidence would
  prevent any future inter-piece mesh-CSG (none planned, but
  consider).
- The cup-wall's "1 mm overlap squeeze" is gone → if the gasket fails
  (tears, mis-installs), the seam is unprotected. Iter-1 tested with
  the squeeze present; workshop user expressed preference for flush
  mating, but the absence of squeeze is a regression vs the
  iter-1-print backstop.
- Risk: MC at the mating plane might emit vertices slightly below
  `y = 0` (interpolation noise), creating a few-µm gap when halves
  butt. Same magnitude as the post-§F-S2 6 µm noise — workshop-
  invisible at FDM 200 µm layer height.

### §M-3-b Candidate B: unified positive overlap (matched compression)

**Mechanism**: set both halfspaces to `overlap_m = 0.0005` (current
cup-wall value). Cup-wall + flange mating faces both at cast
`y = -0.5 mm` (Negative side). Two halves overlap 1 mm everywhere.

**Pros**:
- Preserves the PLA-on-PLA compression backstop (workshop iter-1
  proven).
- Same MC robustness as today (1 mm separation between piece
  boundaries).
- Minimal code change: just pass `RIBBON_PIECE_OVERLAP_M` through
  `build_flange_solid_for_side` instead of `0.0`.

**Cons**:
- The two halves overlap 1 mm at the gasket channel too → gasket gets
  squeezed extra 1 mm AT INSTALL even before bolts engage. Currently
  the gasket only sees the bolt-pressure compression because the
  flange portions butt flush. Extra 1 mm pre-load might over-compress
  the gasket or prevent the cup-wall PLA from making contact.
- Workshop user's stated preference is flush, not compressed.

### §M-3-c Candidate C: bias inside-piece SDF (eliminate halfspace overlap concept)

**Mechanism**: drop the overlap parameter entirely. Each piece's cup
SDF gets a small inward bias term (e.g., `+ ε` where ε ≈ 1 µm) so MC
doesn't produce vertices on the exact y=0 plane. Mating face is at
y = -ε for Negative side, y = +ε for Positive side, gap of 2ε
between pieces (workshop-invisible).

**Pros**:
- Geometrically equivalent to Candidate A but with explicit ε-bias
  for MC robustness, avoiding potential coincident-surface artifacts.
- Eliminates `RIBBON_PIECE_OVERLAP_M` constant + the
  overlap-vs-no-overlap dichotomy across cup-wall / flange.
- ε ≪ FDM print resolution (~200 µm layer height) so workshop-
  invisible.

**Cons**:
- Magic ε value introduces a new constant to maintain.
- Equivalent functionality to Candidate A; the ε is only insurance
  against a problem that may not manifest. YAGNI.

### §M-3-d Picked candidate

**Candidate A** (zero overlap, flush mating). Rationale:

1. Workshop user explicitly requested flush mating.
2. Iter-1 will physical-test whether the absence of cup-wall
   compression backstop matters; if it does, Candidate B is a small
   follow-up change.
3. Candidate C is over-engineering for a non-existent MC problem
   (each piece is independently meshed).
4. Per [[feedback-strip-the-knob-when-default-works]]: removing the
   overlap parameter entirely is simpler than introducing a new ε.

Open Question 1 (workshop user picks at §M-S1 kickoff): keep
`RIBBON_PIECE_OVERLAP_M` exposed as a TOML override (default = 0) for
post-iter-1 backstop experiments, or DELETE the constant entirely?

## §M-4 Dowel-hole registration candidates

The workshop spec: "negative pin holes" on both sides + "we can just
print dowels to lock them together". Two halves get the SAME hole
pattern; loose PLA dowels (printed separately) insert into matching
holes when the halves close.

### §M-4-a Candidate D1: post-MC mesh-CSG SubtractCylinder per dowel

**Mechanism**: new `MatingTransform::SubtractCylinder` (or reuse the
existing variant if it exists — verify) emitted side-agnostically
(same transform list for both Negative + Positive sides). Each
DowelHoleSpec entry produces ONE subtract-cylinder transform applied
to both pieces post-MC. The cylinder axis is binormal-aligned (same
direction as flange thickness); cylinder length spans 2 ×
`mating_overlap_padding_m` for cleanliness.

**Pros**:
- Symmetric (same primitive on both halves). [[feedback-load-bearing-test-fixtures]]
  approves of architectural symmetry.
- Mesh-CSG paradigm-safe per [[project-cf-cast-sdf-meshcsg-paradigm-boundary]] —
  the SubtractCylinder is a discrete post-MC op, not an SDF union.
- Composes with existing pour-gate + plug-lock mating transforms
  (same mesh-CSG framework).
- Workshop already mentally models dowels this way (loose pin
  inserted into pre-drilled hole).

**Cons**:
- Cylinder hole edges are sharp 90° corners — minor stress
  concentration. For PLA dowels at iter-1 design loads this is fine.
- Each dowel hole needs `dowel_radius_m + clearance_m` to allow
  insertion + thermal tolerance.

### §M-4-b Candidate D2: SDF-level subtract

**Mechanism**: build a SDF-domain `Solid::cylinder` and `subtract` it
from each cup-piece's pre-MC SDF before MC.

**Pros**:
- Cylinder hole walls get MC-smoothness (no sharp 90° corners).

**Cons**:
- Adds another SDF composition layer to debug if MC artifacts appear
  near the dowel hole / mating plane junction (similar shape of
  problem to the §Q-1 cup-wall-cuboid-vs-flange interference).
- Doesn't match the rest of the mating-features framework (all are
  post-MC mesh-CSG per recon-4 (P)).
- Per [[project-cf-cast-sdf-meshcsg-paradigm-boundary]]: SDF for
  rough body shape + mesh-CSG for precise discrete features. Dowel
  holes are discrete precise features → mesh-CSG.

### §M-4-c Picked candidate

**Candidate D1** (post-MC mesh-CSG SubtractCylinder, side-agnostic).

## §M-5 Dowel-hole geometry parameters

### §M-5-a DowelHoleSpec defaults

Iter-1 starting envelope (workshop-confirmed at §M-S2 kickoff):

| Parameter | Default | Range | Notes |
|---|---|---|---|
| `dowel_diameter_m` | 0.003 (3 mm) | [0.002, 0.005] | Matches standard 3 mm dowel pin sizes; workshop may have stock |
| `clearance_m` | 0.0001 (0.1 mm) | [0.00005, 0.0003] | Hole radius = (diameter + 2×clearance)/2 = 1.6 mm. PLA-on-PLA dowel slide-fit |
| `depth_m` (per half) | 0.005 (5 mm) | [0.003, 0.010] | Dowel length = 2 × depth = 10 mm. Penetrates each half by 5 mm; dowel sticks out 0 mm when seated |
| `count` | 4-6 around silhouette | [2, 10] | Arc-length-equal spacing around silhouette polyline (reuses §F-S1 `Silhouette2d` infrastructure) |
| `silhouette_outboard_offset_m` | 0.008 (8 mm) | [0.005, 0.013] | Dowel centerline distance from silhouette curve. Must be > `flange_inner_offset_m + dowel_diameter_m/2` to fit inside flange band [`flange_inner_offset_m`, `flange_width_m`]. Inside flange band so dowels don't pierce gasket channel |

### §M-5-b Cross-field invariants

Validated at `cf-cast-cli` config-derive time. Let
`hole_outer_radius_m = dowel_diameter_m / 2 + clearance_m`
(default = 1.5 + 0.1 = 1.6 mm).

- **Inboard wall ≥ FDM floor**: `silhouette_outboard_offset_m - hole_outer_radius_m ≥ flange_inner_offset_m + FDM_wall_thickness_floor`. Default: 8 - 1.6 = 6.4, vs 2 + 1 = 3 → 6.4 ≥ 3 ✓ (4.4 mm margin).
- **Outboard wall ≥ FDM floor**: `silhouette_outboard_offset_m + hole_outer_radius_m ≤ flange_width_m - FDM_wall_thickness_floor`. Default: 8 + 1.6 = 9.6, vs 15 - 1 = 14 → 9.6 ≤ 14 ✓ (4.4 mm margin).
- **Dowel doesn't pierce body cavity**: `depth_m × 2 < body_extent_at_dowel_position_in_binormal_axis`. For 5 mm depth × 2 = 10 mm dowel length: body's binormal-axis extent at dowel position ≈ 60-130 mm depending on which layer; far above 10 mm. ✓ (Workshop confirms at §M-S2 kickoff for layer-specific body geometry.)
- **Dowel doesn't collide with cup-wall material on opposite side**: the dowel hole is `SubtractCylinder` centered at the mating plane y=0; it carves through 5 mm of each half. As long as `depth_m + clearance_m < flange_thickness_m_per_half + cup_wall_axial_reach_at_dowel_position`, the dowel hole stays within mating-feature region. Default depth=5 mm + 0.1 mm clearance = 5.1 mm vs (flange_thickness 4 mm + cup-wall reach at silhouette+8 ≈ unconstrained because cup-wall is body-tracking). ✓ but verify per-layer at §M-S2.

All four invariants pass at recon defaults — no need to widen
`flange_width_m` for the §M arc on its own.

### §M-5-c Optional alignment with §B (informational, not §M-blocking)

Independent of §M, the §B M5 bolt-pattern recon
([[project-cf-cast-flange-continuity-bolt-pattern-recon]] Open
Question 0) recommends widening `flange_width_m` from 15 mm to
18.5 mm to give M5 through-bolts sufficient wall thickness
(>1× hole diameter rule for plastic fasteners). The §M arc is
geometrically satisfied at 15 mm; widening for §B reasons would
also widen the §M dowel margins from 4.4 mm to 6.2 mm. No conflict —
§M ships at workshop-picked flange_width and adapts.

## §M-6 Open questions

Workshop user picks before / during implementation arc:

0. **Overlap_m**: keep `RIBBON_PIECE_OVERLAP_M = 0.5 mm` exposed as
   TOML override (default = 0) or delete the constant entirely?
   *Recommended: delete (per [[feedback-strip-the-knob-when-default-works]]).*
1. **Dowel material**: print PLA dowels (matches cup-piece material;
   workshop already has filament) or buy metal dowels (precision +
   stiffness)? *Recommended: PLA per workshop's "we can just print
   dowels".*
2. **Flange width** at §M-S2 kickoff: 15 mm (current — §M-S2 dowel
   math passes with 4.4 mm margins) vs 18.5 mm (recommended by §B M5
   bolt-pattern recon Open Question 0 for wall-thickness rule). §M
   arc is satisfied at either value; decision is for §B compatibility
   only.
3. **Dowel count + placement**: auto-derived (arc-length-equal around
   silhouette like §B bolts) vs config-specified positions? *Recommended:
   auto-derived with TOML-specified count (default 4).*
4. **RegistrationKind retirement**: keep `RegistrationKind::Pins`
   (prismatic-pin pairs) as a legacy fallback or delete after
   §M-S2 ships dowel holes? *Recommended: delete — no scenario needs
   asymmetric pin/socket once dowels work, and the constant code
   surface is maintenance debt.*
5. **Dowel hole binormal orientation per layer**: should each layer's
   dowel pattern share dowel positions across layers (so a single
   dowel set works for all 3 layers' mold pieces) or per-layer? Workshop
   uses 3 separate molds at iter-1 so per-layer might be fine, but
   single-dowel-set-for-all-layers reduces parts count + setup time.
   *Recommended: shared positions across layers via the same
   silhouette-arc-length placement (silhouettes are similar across
   layers within ~few mm).*
6. **§M-S3 cross-spec validation**: where does dowel-vs-bolt
   collision-avoidance live? If §B bolts and dowels both target the
   silhouette band [2, 15] mm, they could overlap. Workshop picks at
   §B-S1 kickoff (after §M arc ships, before §B arc opens).

## §M-7 Implementation arc

Estimated 4-6 sessions across 2-3 PRs.

### §M-S1: unify mating-plane halfspace overlap (~50 LOC)

- Either delete `RIBBON_PIECE_OVERLAP_M` constant + change call sites
  to `0.0` (per Open Question 0 recommended pick), OR change the
  default to 0.0 and keep the constant as a TOML override hook.
- Change `build_flange_solid_for_side`'s explicit `0.0` to whatever
  the unified value is (now the same as cup-wall's call).
- Regression tests in `ribbon.rs` for `halfspace_*` tests at zero
  overlap: verify Negative + Positive halfspaces at y = 0 with no
  bias.
- Tests in `piece.rs` for `pieces_partition_cup_material_symmetrically`
  + `pieces_overlap_at_ribbon_seam_with_overlap_bias`: update
  expectations.
- §M-S1 production regen + workshop cf-view smoke + Y-bin STL
  diagnostic re-run: verify mating face is now SINGLE plane (no 0.5 mm
  step).

### §M-S2: dowel-hole primitive + side-agnostic emission (~150 LOC)

- New `DowelHoleSpec` (5 fields per §M-5-a) + `DowelHoleKind` ribbon
  field matching the existing kind enum pattern.
- New `cf-cast::dowel_hole` module:
  - `build_dowel_hole_transforms(ribbon, spec) -> Vec<MatingTransform>`:
    arc-length-equal placement around silhouette (reuses
    `crate::silhouette_2d::Silhouette2d` from §F-S1; ordered closed-
    polyline assembly needs to actually land here since dowels need
    arc-length parameterization — this is the §B prerequisite the
    §F-S1 recon deferred).
  - Per-dowel `MatingTransform::SubtractCylinder` with axis = binormal,
    centered at the mating plane (cast y = 0), length = 2 × `depth_m`
    + slop, radius = `dowel_diameter_m / 2 + clearance_m`.
- `compose_piece_solid` integration: emit dowel transforms post-MC,
  applied to both Negative + Positive pieces.
- Polyline-assembly + arc-length-placement scaffolding in
  `silhouette_2d.rs` (deferred from §F-S1; lands here).
- Unit tests for arc-length placement + collision-skip + per-side
  symmetry.

### §M-S3: cf-cast-cli TOML config + cross-field validation (~80 LOC)

- New `[dowel_hole]` TOML block in `cast.iter1-design.toml`.
- `DowelHoleConfig` → `DowelHoleSpec` mapping in `tools/cf-cast-cli/src/config.rs`.
- Cross-field validation per §M-5-b in
  `tools/cf-cast-cli/src/derive.rs::validate_after_layer_source`:
  - dowel-vs-gasket clearance.
  - dowel-vs-flange-outer-edge clearance.
  - dowel-vs-body-cavity clearance.
  - dowel-hole-wall-thickness minimum.
- procedure.md prose addition: "Insert printed dowels into the hole
  pattern at assembly time. Dowel BOM: N × 3 mm × 10 mm PLA dowels
  (printed separately, 1 STL per cast)."

### §M-S4: RegistrationKind retirement (~50 LOC code + ~30 LOC tests)

Gated on Open Question 4. If picked:
- Remove `RegistrationKind::Pins` variant (or its Plate equivalent).
- Remove `PrismaticPinSpec` if unused elsewhere.
- Remove `add_pin_transforms` call from `compose_piece_solid`.
- Update tests that exercised the prismatic-pin path.
- Update procedure.md prose.

If not picked: skip §M-S4; dowel holes + prismatic pins coexist (TOML
gates which is used; default to dowel holes).

### §M-S5: production regen + workshop cf-view smoke gate

- Full regen to `~/scans/cast_iter1_unified_mating_plane/`.
- Workshop user cf-view smoke verifies:
  - Mating face is a single flat plane (no 0.5 mm step).
  - Dowel holes are visible at the correct positions on both halves.
  - Two-half assembly looks flush in cf-view's `--assembly` mode.
- Wall-clock + STL byte diff vs §F-S2 baseline (expected: minor
  decrease in cup-piece STL size from removed overlap region;
  dowel-hole subtracts ~5-10 KB per piece).

### §M-S6: cold-read + commit + memory entry

- Cold-read pass on the §M-S1..§M-S5 diff per
  [[feedback-cold-read-two-passes-for-non-trivial-diffs]].
- Single commit per phase OR omnibus per [[feedback-omnibus-pr-single-branch]]
  if the changes are tightly coupled (probably tight: §M-S1 + §M-S2 +
  §M-S3 form one architectural change).
- Memory entry: `project_cf_cast_unified_mating_plane_*.md`.
- Update [[project-cf-cast-flange-continuity-s1]] memory: note that
  §M arc supersedes the §F-S1 architecture's overlap inconsistency.

## §M-8 Composition with existing features

The §M architecture is additive / replacing-by-strict-subset relative
to current cf-cast composition:

- **Pour gate** (V-at-dome funnel): independent. Pour gate composition
  uses its own halfspace logic; flush mating doesn't change the pour
  geometry.
- **Plug-floor-lock socket**: existing `SubtractTruncatedPyramid`,
  side-agnostic. Composes naturally with the new dowel-hole
  `SubtractCylinder` (both are side-agnostic mesh-CSG ops; no
  interference).
- **Flange (§F-S1 silhouette-based)**: unchanged. Just the halfspace
  cut at the mating plane changes from "overlap 0" to "overlap 0"
  (already the same; no code change for flange other than the unified
  halfspace lookup).
- **Cup-wall (§Q-1 body-tracking shell)**: unchanged geometrically.
  Halfspace overlap changes 0.5 → 0; cup-wall now extends ONLY to
  y = 0 instead of y = -0.5 mm.
- **Bolt pattern (§B per [[project-cf-cast-flange-continuity-bolt-pattern-recon]])**:
  composes on top. §B opens AFTER §M ships. §B-S1 kickoff must
  re-validate the dowel-hole-vs-bolt-hole collision-avoidance per
  Open Question 6.
- **Gasket channel + mold**: unchanged. Gasket sits in the channel at
  the body silhouette; flush mating gives the gasket a clean PLA
  flange to seat against.

## §M-9 Spec implications

The §M architecture changes one workshop-facing invariant:

- **Pre-§M**: two-half assembly involves PLA-on-PLA compression at the
  cup-wall rim (1 mm squeeze) + gasket compression at the seam channel
  (per gasket spec).
- **Post-§M**: two-half assembly is PLA-on-PLA flush mating at both
  cup-wall + flange + gasket compression at the seam channel (per
  gasket spec, same as before).

Net effect on iter-1 sealing: gasket carries the same sealing load.
Cup-wall PLA-on-PLA backstop is gone. If the gasket fails (tears,
mis-installs), the seam leaks. This is the workshop user's call —
they have decided the simpler flush-mating model is preferable, with
the gasket as sole seal.

## §M-10 Lessons + discipline notes

1. **Inconsistent overlap across SDF terms creates structural
   stepped surfaces**. The current architecture had cup-wall using
   `RIBBON_PIECE_OVERLAP_M` and flange using `0.0` — both intentional
   at their respective design points, but the combination produced a
   workshop-visible step. Lesson: when two SDF terms share a common
   boundary (here, the seam plane), their boundary-specific parameters
   (here, overlap) must be intentionally unified or differenced — not
   independently chosen by each subsystem.

2. **Workshop user's "step back" reframe is the most reliable bug
   solver in this arc**. §F-S1 → §F-S2 cycle revealed that
   silhouette-vs-3D-distance wasn't the workshop-visible bottleneck;
   the workshop user's reframe of "I want flat mating face + dowel
   holes" identified the actual root cause (overlap inconsistency)
   without any further code-side investigation. Pattern reinforces
   [[feedback-foundational-ordering]] + [[feedback-spike-before-trust-analytical]]:
   workshop user articulates physical requirements, then code-side
   inspects whether the architecture satisfies them.

3. **Symmetric registration > asymmetric pin/socket pair**. The
   prismatic-pin architecture worked but introduced asymmetric per-
   side code paths. Symmetric dowel-hole + loose-dowel inserts removes
   the asymmetry. Pattern: when two halves are physically identical
   except for what they CONTAIN, their CODE paths should be identical
   too. Asymmetric primitives are a smell that the abstraction is at
   the wrong level.

4. **Cross-spec invariants surface at recon time, not impl time**.
   §M-5-b's dowel-hole wall-thickness math FAILED at default geometry
   — caught at recon authoring, not impl PR. Per
   [[feedback-math-pass-first-handauthored]]: hand-derive cross-field
   constraints before writing impl code; the recon's math pass
   prevents impl PRs from shipping geometrically-infeasible defaults.

5. **STL Y-bin diagnostic is a cheap geometry inspector**. The 1 µm
   precision Y histogram of cf-cast output STLs surfaced the 481 µm
   overlap signature in <30 LOC of Python (per the §F-S2 follow-up
   investigation). Pattern: for SDF + MC architecture debugging,
   extract vertex distributions from the final mesh + look for
   structural signatures (cell-aligned bins, plane-aligned clusters,
   sub-cell artifacts). Add to the diagnostic toolkit.

## §M-11 Cold-read self-check (pre-commit)

Run before committing this recon (per
[[feedback-cold-read-two-passes-for-non-trivial-diffs]]):

- [x] Math in §M-5-b: re-derived each cross-field invariant from
      construction. Cold-read pass-1 caught an earlier formulation
      that double-counted `silhouette_outboard_offset_m` and falsely
      concluded the invariants fail at defaults. Re-derived correctly
      using per-side wall thickness (inboard + outboard separately):
      both pass with 4.4 mm margins. §M-5-c reframed from "must widen
      flange" to "optional §B alignment".
- [ ] API type references match workspace (grep for `MatingTransform`,
      `DowelHoleSpec` placeholders).
- [ ] Phase scope estimates (LOC counts) match similar prior recon
      phases (§F-S1 silhouette was ~525 LOC actual vs ~150 estimated —
      this recon should estimate generously).
- [ ] Workshop-user-relevant decisions surfaced as Open Questions, not
      unilateral picks. Grep for "Decision: keep / pick X" — none
      should appear without "Open Question N" framing.
- [ ] Predecessor + successor memory links resolve. Specifically:
  - [[project-cf-cast-flange-continuity-s1]] ✓ (exists).
  - [[project-cf-cast-flange-continuity-bolt-pattern-recon]] ✓.
  - [[project-cf-cast-registration-pin-disconnection-impl]] — verify
    spelling.
  - [[feedback-foundational-ordering]] — verify spelling.
- [ ] §M-S2's "polyline-assembly + arc-length-placement scaffolding"
      claim: the §F-S1 silhouette stores segments as 2-vertex
      polylines (unordered). §M-S2 needs to land the ORDERED closed-
      polyline assembly. Either §M-S2 scope grows to ~200 LOC for
      polyline assembly + arc-length, OR a new §M-S1.5 spike phase
      lands polyline assembly first. Pick at §M-S2 kickoff.

## §M-12 Predecessors

- [[project-cf-cast-flange-continuity-s1]] — §F-S1 silhouette + §F-S2
  shard-clip; established the silhouette_2d module + surfaced the
  workshop "not flat mating face" finding that triggered this recon.
- [[project-cf-cast-flange-continuity-bolt-pattern-recon]] — §F + §B
  recon scaffold; §B bolt pattern composes on top of §M.
- [[project-cf-cast-registration-pin-disconnection-impl]] — pre-§M
  registration architecture (prismatic pin/socket pairs) that §M-S4
  retires.
- [[project-cf-cast-geometry-crispness-q1-fix]] — §Q-1 body-tracking
  shell; cup-wall architecture that §M unifies the halfspace
  treatment of.

## §M-13 Successor

§M-S1 (~50 LOC unified halfspace) next session. Workshop user picks
Open Question 0 (delete or expose `RIBBON_PIECE_OVERLAP_M`) at
session kickoff. §M-S1 ships with regression test re-running the
Y-bin diagnostic on a synthetic fixture + asserting single mating
plane.

Then §M-S2 + §M-S3 + §M-S4 in subsequent sessions per phase scope.

§M-S5 production regen + workshop cf-view smoke gates the arc as
WORKSHOP-VALUE-DELIVERED (vs §F-S1's ZERO-VALUE outcome).
