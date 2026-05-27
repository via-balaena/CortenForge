# cf-cast seam-flange recon (2026-05-25)

**Status:** scaffold. Triggered by workshop user's iter-1 cf-view
observation that the current contour-following cup exterior is
hard to clamp evenly — load-bearing for the seam-gasket-mold arc's
predicted compression pressure (~20 kPa per
[[project-cf-cast-seam-gasket-mold-s2]]
`GasketSpec.workshop_clamp_pressure_pa`).

**Picked architectural fix:** add a flat rectangular flange at the
seam plane around the body cavity perimeter on each cup half.
Cup body retains its contour-following rind (preserves the ~10×
PLA savings from
[`docs/archive/CF_CAST_MOLD_WALL_RECON.md`](../docs/archive/CF_CAST_MOLD_WALL_RECON.md)
Option A); the flange supplies a flat clamp-grip surface ONLY at
the seam-plane perimeter where the gasket actually seals.
**Hybrid contoured-cup + flat-flange** — restores workshop clamp
ergonomics WITHOUT reverting the prior recon's full PLA savings.

## Cross-arc decision staleness

The prior mold-wall recon
([`docs/archive/CF_CAST_MOLD_WALL_RECON.md`](../docs/archive/CF_CAST_MOLD_WALL_RECON.md))
shipped "Option A contour-following rind" with §3.3 explicitly
punting on clamp ergonomics:

> *Pour gate / vent / pin clearance (Q5)... workshop pour-fixture
> is gravity-driven, not flat-clamp-driven (Q5 verdict: aesthetic,
> not blocking).*

That verdict was correct at the time (gravity-pour workflow).
**The seam-gasket-mold arc changed the workflow.**
[[project-cf-cast-seam-gasket-mold-s2]] introduced
`GasketSpec.workshop_clamp_pressure_pa = 20 kPa` as the
design-time anchor for `predicted_compression_m()` — which
*must* exceed the §G-0 FDM-tolerance target (~200 µm) for the
silicone seal to function. If the cup halves can't be clamped
evenly to that pressure, the gasket's predicted compression isn't
delivered, and the seam leaks.

Per [[feedback-read-prior-arc-memory-before-architectural-decisions]]
rule #1: "Before any cf-cast architectural decision: grep memory
for prior arcs touching the primitive." This recon is the
corrective surface for the mold-wall recon's Q5 verdict, picked up
by the gasket arc's workshop-clamp requirement.

**Out of scope:**

- Reverting to the pre-mold-wall-recon full cuboid bounding region
  (~10× more PLA per cup half). The seam flange achieves clamp
  ergonomics at a fraction of the cost.
- Bolt-hole flanges (drilled holes through the flange for through-
  bolts). Possible workshop-iter-N escalation if C-clamp pressure
  proves insufficient; not iter-3 path.
- Per-piece flange retention features (toggle latches, snap fits,
  alignment lugs). The cup halves already have inter-piece
  registration pins per [`crate::registration`]; flange contact
  is passive (C-clamp force only).
- Funnel + platform + plug STL flange geometry. Funnel is gravity-
  pour (doesn't clamp); platform is a flat support slab (already
  clamp-friendly); plugs are interior to the cup cavity (no
  clamping pressure path).

## §F-0 Why this recon

Empirical from S2 of the seam-gasket-mold arc
([[project-cf-cast-seam-gasket-mold-s2]]):

| Field | Value | Source |
|-------|-------|--------|
| Target workshop clamp pressure | 20 kPa | recon §G-1, S2 `GasketSpec.workshop_clamp_pressure_pa` |
| Predicted Ecoflex 00-30 compression at 20 kPa × 0.8 mm | 232 µm | S2 `predicted_compression_m()` |
| §G-0 FDM-tolerance target (~200 µm) | 200 µm | gasket recon §G-0 typed-range |
| Cup outer surface (current; contour-following) | curved everywhere | derive.rs:297 `pinned_floor_shell(...).offset(wall_thickness_m)` |
| C-clamp jaw geometry | flat parallel jaws, ~50 mm reach | typical workshop hardware |

A C-clamp on a curved cup outer applies pressure unevenly: the jaw
contacts the body's local tangent plane (a point or line, not a
face), and the load concentrates where the curve is shallowest.
On a sock-shaped scan with a tapered cross-section, the clamp
slips during tightening. Workshop user reports difficulty closing
the cup halves square to the seam plane.

A flat flange at the seam plane provides a parallel-jaw-friendly
contact surface, transferring clamp force directly to the seam
where the gasket needs it.

## §F-1 Flange geometry concept

**Flange = flat rectangular plate at the seam plane (Y = 0 in
production cast.toml's split-normal frame) extending OUTWARD from
the cup body cavity perimeter on each half.** Each cup half gets a
mirror-image flange in its own half-space (Negative half: flange
in -Y half-space; Positive half: flange in +Y half-space). Closed
cup: two flange faces meet at the seam plane, with the gasket
between them.

- **Cross-section (seam plane view):** flat rectangular ring
  surrounding the body cavity perimeter. Inner edge follows the
  body perimeter; outer edge is straight (rectangular bounding box
  + flange_extent).
- **Profile (perpendicular to seam plane):** rectangular slab of
  thickness `flange_thickness_m` (~3-5 mm), extending into each
  half's exterior side.
- **Material:** PLA, same FDM stack as the cup body.

The flange does NOT touch the cup cavity interior — it sits
strictly outside the body perimeter, in the same Y-plane as the
seam face. The cup body retains its contour-following rind for
the cavity-adjacent walls; the flange is a peripheral skirt.

## §F-2 Flange dimensions

Three numeric parameters with workshop-iter-3 defaults:

- **`flange_width_m`**: lateral extent of the flange perpendicular
  to the body perimeter, in the seam plane. Default **15 mm**.
  Rationale: ~10 mm for a standard C-clamp jaw reach + ~5 mm
  perimeter clearance.
- **`flange_thickness_m`**: **per-half** thickness perpendicular
  to the seam plane (along the Y-axis in production frame).
  Default **4 mm per cup half** → total closed flange-zone
  thickness ≈ 8 mm + gasket compression. Above the FDM minimum-
  wall floor (1.0 mm). Bending check: a 1 mm-wide flange strip
  cantilevered along its 15 mm width direction (force applied
  perpendicular to the seam plane by the C-clamp), thickness
  4 mm, PLA flexural modulus ≈ 3.5 GPa, distributed load of
  ~1 N/mm of perimeter (50 N spread over ~50 mm of clamp
  contact): δ ≈ F·L³ / (3·E·I) ≈ 1 · 15³ / (3 · 3500 · 5.33) ≈
  **30 µm** — well below the gasket's ~200 µm compression
  budget.
- **`flange_inner_offset_m`**: gap between the body perimeter
  (where the gasket sits) and the flange's INNER edge (where the
  flange material begins). Default **2 mm** = ~2.7 × the gasket-
  channel half-width at iter1 (0.75 mm). Keeps the flange's PLA
  material LATERALLY DISJOINT from the gasket channel so the
  silicone seal isn't pinched by the flange.

Total PLA cost per cup half:
- Flange volume ≈ seam_perimeter × flange_width_m × flange_thickness_m
  (per cup half).
- Production sock-over-capsule cross-section at the seam plane
  ≈ 200 × 30 mm (long along centerline, short radial); perimeter
  ≈ 2 × (200 + 30) = **460 mm**.
- Volume per cup half ≈ 460 × 15 × 4 = 27,600 mm³ ≈
  **27.6 cm³**.
- Per cast (3 layers × 2 halves = 6 cup halves): ~**165 cm³**.
- Comparison: full-cuboid bounding region per the mold-wall recon
  was ~570 cm³ for the entire cup; contour + flange ≈ 165 cm³
  extra vs current contour-only.
- **Net cost**: **~3-4× less PLA than the full cuboid** the
  prior mold-wall recon rejected. Clamp ergonomics restored at a
  fraction of the material cost.

## §F-3 SDF composition options

Three composition strategies; pick at S1 implementation:

**(a) Per-perimeter-offset flange** (geometrically clean).
```text
flange_sdf(P) = max(
    body.evaluate(P_at_seam_plane) - flange_width_m,
    flange_inner_offset_m - body.evaluate(P_at_seam_plane),
    |P.y - seam_plane_y| - flange_thickness_m
)
```
The flange exists where ALL THREE inequalities are ≤ 0:
- `body_dist ≤ flange_width_m` (within outward reach)
- `body_dist ≥ flange_inner_offset_m` (clears the gasket channel
  laterally per §F-4 disjoint invariant)
- `|P.y - seam_plane_y| ≤ flange_thickness_m` — symmetric
  ±thickness around the seam plane. After the per-half halfspace
  cut (next paragraph), this gives `flange_thickness_m` of
  material in EACH half-space (consistent with §F-2's per-half
  thickness definition).

**Mirrors the `GasketChannelSdf` pattern** from
[[project-cf-cast-seam-gasket-mold-s1]] — projection-to-seam-plane
+ max() composition. The seam-plane-projection trick is already
proven on the gasket channel.

Per-half halfspace cut splits the symmetric flange into Negative-
half and Positive-half flanges via
`ribbon.halfspace_solid(side, 0.0)` (the existing seam halfspace).

**(b) Rectangular-bounding-box flange** (simplest).
```text
flange_sdf(P) = max(
    bounding_rect_sdf(P_at_seam_plane),
    |P.y - seam_plane_y| - flange_thickness_m,
    flange_inner_offset_m - body.evaluate(P_at_seam_plane)
)
```
Flange is a flat rectangle at the seam plane (sized to body XZ
bbox + `flange_width_m`), minus the body cavity inflated by
`flange_inner_offset_m`. Simpler but wastes material at narrow-
end-of-body regions. PLA cost ~30-50% higher than option (a).

**(c) Constant-Y offset flange** (intermediate).
Same as (a) but with a fixed-thickness rectangular flange shape
in XZ (no body-perimeter-following), positioned at the seam
plane Y. Compromise between (a) and (b).

**Recommended: option (a)** — perimeter-following geometry mirrors
the gasket-mold composition pattern (so the gasket channel + the
flange share the same projection-to-seam-plane infrastructure),
saves the most PLA, and stays consistent with the contour-
following cup body. Mating: union with the existing cup-piece
SDF in `compose_piece_solid`.

## §F-4 Interaction with the gasket-mold channel

The flange and the gasket live in the **same Y-range** (centered
on the seam plane) but at **different lateral positions** relative
to the body perimeter:

- **Gasket strip**: at the body perimeter itself
  (`|body_dist| ≤ gasket_half_width ≈ 0.75 mm` at iter1).
  Compressed between the two cup halves' seam faces.
- **Flange**: outside the body perimeter, starting beyond the
  gasket's lateral reach (`body_dist ≥ flange_inner_offset_m ≈
  2 mm`) and extending outward to `body_dist ≤ flange_width_m ≈
  15 mm`.

**Y-cross-section schematic** (looking along the body perimeter
tangent at any seam-perimeter point):

```
                                  outward (+body_dist) →

           Y = +flange_thickness_m ─┬─────────────────────────────
                                    │                             │
                                    │     +half flange material   │
                                    │     (cup half + Y)          │
           Y = +ε (seam face, +) ───┤                             │
           Y = 0  (seam plane)    ──┼─ gasket strip here (body_dist ≈ 0)
           Y = −ε (seam face, −) ───┤                             │
                                    │                             │
                                    │     -half flange material   │
                                    │     (cup half - Y)          │
           Y = -flange_thickness_m ─┴─────────────────────────────

           body_dist:          0    2 mm                       15 mm
                          (gasket   (flange inner edge)   (flange outer edge)
                           lives
                           here)
```

The flange's PLA-on-PLA contact at the seam plane (Y = 0) is
**outside** the gasket strip. C-clamp force on the flange (in the
±Y direction) translates to compressive force on the gasket strip
through the cup body — the flange provides the grip surface; the
gasket provides the seal.

**Critical invariant:** flange and gasket are **LATERALLY
DISJOINT** in the seam plane. The SDF in §F-3 option (a)
enforces this via `body_dist ≥ flange_inner_offset_m`. Default
`flange_inner_offset_m = 2 mm` ≈ 2.7× the gasket half-width
(0.75 mm) — comfortable lateral margin so MC quantization at the
flange's inner edge can't accidentally touch the gasket channel
at the §G-13 0.5 mm gasket cell size.

## §F-5 Interaction with mating features

The cup-piece SDF currently has these mating features at the
seam plane:

- **Cup-pin truncated-pyramid protrusions**
  ([`crate::registration`]). Pins are along the binormal direction
  (perpendicular to seam plane), positioned at the cup wall mid-
  thickness. Pin half-extent perpendicular to seam plane is
  typically 3-5 mm. **Conflict potential**: if the flange covers
  the seam-plane region where pins protrude, the pins clip into
  the flange material.
  - **Resolution at S1**: pins sit at the body wall mid-thickness
    (inside body region, NOT outside). Flange is OUTSIDE body. No
    spatial overlap — orthogonal features.
- **Plug-floor lock truncated-pyramid**
  ([`crate::plug::build_plug_lock_sdf`]). Lives entirely inside
  the plug body, opens through the cap-plane face. Cup-side socket
  is in the cap-plane region (not at the seam plane). No flange
  conflict.
- **Pour gate cylinder** ([`crate::pour`]). Runs along the
  binormal from the centerline midpoint. The pour gate exits
  through the cup wall on ONE side of the cup. At the seam plane,
  the pour-gate cylinder may pass through the flange. **Conflict
  potential**: gate cylinder ID = ~6 mm; flange thickness = 4 mm;
  the cylinder threading through the flange creates a partial
  cylinder through the flange material.
  - **Resolution at S1**: gate cylinder is subtractive (already
    cuts through the cup wall). Same subtract through the flange
    is geometrically valid; the pour-gate just becomes a longer
    tube. Aesthetic-only; not load-bearing.

No paradigm-boundary conflicts per
[[project-cf-cast-sdf-meshcsg-paradigm-boundary]]: the flange is
bulk-welded geometry (SDF-side), like the cup wall itself. No mesh-
CSG mating-features pose changes needed.

## §F-6 cf-cast-cli integration

New `[flange]` config block in `cast.toml` schema, matching the
existing `[gasket]` block pattern:

```toml
[flange]
enabled = true                      # default true; set false to skip flanges
width_m = 0.015                     # 15 mm lateral extent
thickness_m = 0.004                 # 4 mm thickness perpendicular to seam
inner_offset_m = 0.002              # 2 mm gap from body perimeter (preserves gasket clearance)
```

All four fields optional; absent block → `FlangeConfig::default()`
with the iter-3 defaults above. Cross-field validation: at
`validate_after_layer_source` time, check
`inner_offset_m > half_gasket_channel_width` (~0.75 mm at iter1)
to prevent gasket-flange lateral overlap.

cf-cast surface:

- `FlangeSpec` struct (3 f64 fields + `iter1()` constructor).
- `FlangeKind { None, Plate(FlangeSpec) }` enum mirroring
  [[project-cf-cast-seam-gasket-mold-s3]]'s `GasketKind` pattern.
- `Ribbon::flange: FlangeKind` field + `with_flange(...)` builder.
- `compose_piece_solid` gains a `FlangeKind` parameter (threaded
  from the ribbon); when `Plate(spec)`, unions the flange SDF
  into the cup piece geometry.

## §F-7 Procedure.rs implications

Workshop clamp protocol needs a new section
`## Cup-Half Clamping with Gasket Installation`:

1. Pour silicone into gasket molds + cure (per existing gasket
   protocol).
2. Peel gasket out + position on Negative cup half seam face,
   inside the flange perimeter (gasket strips trace the body
   cavity edge).
3. Close Positive cup half over Negative + gasket; align via
   registration pins.
4. Apply C-clamps to the **flange** at 4 positions (one per
   90° quadrant around the seam plane perimeter). Tighten each
   to ~hand-tight + 1/8 turn.
5. Pour main layer silicone through pour gate.
6. Cure per layer material's TDS.
7. Release clamps + open cup halves.

Clamp count + tightness is workshop-empirical at S6 iter-3; the
recon documents the iter-3 starting recipe.

## §F-8 Per-half symmetry

Both Negative and Positive cup halves carry their own flange in
their respective Y half-space. The two flanges meet at the seam
plane (Y = 0) when the cup closes; their interface IS the seam.
The gasket strip is sandwiched between the two flange faces along
the body perimeter; outside the gasket strip, the flange faces
contact each other directly (PLA-on-PLA, no seal — that's OK,
the gasket's curve is what seals the cavity).

**Critical alignment invariant:** the two flanges MUST align
exactly at the seam plane. The existing registration-pin features
([`crate::registration`]) provide this alignment — same mechanism
that aligns the cup walls. No new alignment infrastructure needed.

## §F-9 Cap-plane / pour-end interaction (out of scope)

The cap plane (where the plug exits the cup cavity) is
perpendicular to the centerline at the pour end. The flange lies
in the seam plane (perpendicular to the split-normal). These two
planes are mutually perpendicular for the production split-normal
[1, 0, 0]. The cap-plane face is NOT clampable (it's where the
plug + pour gate live); workshop user clamps only the seam-plane
flange. Cap-plane behavior unchanged by this recon.

## §F-10 Open questions

1. **Flange edge chamfering for handling**. PLA-printed sharp
   edges are skin-grippy. Add a 0.5-1 mm chamfer to the flange
   outer edges + corners for workshop ergonomics? Empirical at
   S6 iter-3; defer.
2. **Flange thickness vs PLA infill density**. At workshop default
   infill (~20% gyroid), 4 mm flange should be stiff enough. If
   workshop iter-3 reveals flex, options are: (a) increase
   flange_thickness, (b) increase infill density at slicer level
   (no geometry change), (c) add internal ribbing geometry to the
   flange. Pick at S6 if needed.
3. **Multiple gaskets / multiple seal lines**. Currently one
   gasket per seam. A second outer seal (e.g., O-ring groove in
   the flange face) could provide redundancy if iter-3 reveals
   single-gasket leakage. Defer until empirical fail mode.
4. **Reusing the same flange geometry for the funnel mating**.
   Funnel currently lives at the pour-gate end, separate STL.
   Funnel-cup mating is gravity-driven (per mold-wall recon §3.3
   Q5), not clamped. Defer.

## §F-11 Workshop iter-3 unblock criteria

Quad-gate post-S1 ship (modelled on the gasket arc's §G-11):

1. **§R1 connectivity** on all 6 cup-half STLs: 1 component each.
2. **F4 gate** passes on all cup halves (Critical = 0). Validates
   flange + cup-body composition is FDM-printable.
3. **Workshop cf-view smoke**: flange visibly present + flat +
   parallel to seam plane on each cup half. Both halves' flanges
   align when assembled in cf-view's pair view.
4. **Workshop physical clamp test** (iter-3): C-clamps grip the
   flange flat surface securely; closing the cup over a gasket
   produces a visible compression along the entire body
   perimeter. Empirical only; deferred to S6 workshop session.

## §F-12 Bail-out priority

If S1 reveals the flange breaks something:

1. **Disable via TOML**: `[flange] enabled = false`. Defaults are
   permissive; failures degrade to current contour-only behavior
   with no flange. Easy A/B test.
2. **Revert the S1 commit**: each phase is a single commit on dev;
   `git revert` restores serial behavior. Tested at S2 / S4.
3. **Switch to SDF composition option (c)** (constant-Y offset
   flange) — simpler geometry, less per-perimeter SDF complexity.
   May reduce composition-time cost if option (a) is slow.
4. **Switch to SDF composition option (b)** (rectangular bounding
   flange) — last resort, more PLA but trivially correct.
5. **Revert to mold-wall-recon Option B** (per-layer cuboid
   bounding region) — drops contour savings entirely. Worst case
   fallback.

## §F-13 Implementation arc

5 phases, ~4-6 wall-clock sessions:

- **S1: FlangeSpec + compose_piece_solid integration**
  (~200 LOC). New `FlangeSpec` struct + `FlangeKind` enum + Ribbon
  field + builder. Modify `compose_piece_solid` to union the
  flange SDF when `FlangeKind::Plate(spec)`. SDF composition
  option (a) (per-perimeter-offset). Tests: paired bare-baseline
  per [[feedback-load-bearing-test-fixtures]]: cup-piece with
  flange vs cup-piece without; flange-vs-gasket lateral non-
  overlap probe; flange-thickness probe at seam plane vs Y =
  thickness boundary.
- **S2: cf-cast-cli `[flange]` config + derive**
  (~80 LOC). `FlangeConfig` TOML block + `resolve_flange_kind`
  derive helper + cross-field validation
  (`inner_offset_m > half_gasket_channel_width`). Per the gasket
  arc's S3 pattern.
- **S3: procedure.rs workshop clamp protocol prose**
  (~100 LOC + tests). New `## Cup-Half Clamping with Gasket
  Installation` section. Doc-anchoring tests per the
  cap-plane (4')-pattern precedent.
- **S4: cf-cast-cli iter-1 regen on production cast.toml**.
  Verify 6 cup-half STLs include flange geometry; F4 gate
  passes; §R1 inspector clean. Workshop user cf-view smoke gate.
- **S5: Cold-read pass-1 + omnibus PR** per
  [[feedback-omnibus-pr-single-branch]] alongside the gasket arc
  + parallel-meshing arc.

Total scope: ~380 LOC across S1-S3; S4-S5 are gate + ship.

## §F-14 Prior-arc memory checklist

Per [[feedback-read-prior-arc-memory-before-architectural-decisions]]
rules 1-6, the following memories MUST be read by the S1
implementation session BEFORE any architectural decision:

- [[project-cf-cast-seam-gasket-mold-s2]] — gasket spec +
  `workshop_clamp_pressure_pa` invariant. Sets the load-bearing
  reason for this recon.
- [[project-cf-cast-seam-gasket-mold-s3]] — gasket geometry on
  the seam plane. Defines the lateral region the flange must NOT
  overlap (gasket channel half-width).
- [[project-cf-cast-fdm-friendly-geometry-arc]] — cup-pin +
  plug-lock mating feature pose conventions. Confirms flange ⊥
  these features (no spatial overlap).
- [[project-cf-cast-sdf-meshcsg-paradigm-boundary]] — paradigm-
  boundary framework. Confirms flange is bulk-welded SDF
  geometry (correct paradigm placement).
- [`docs/archive/CF_CAST_MOLD_WALL_RECON.md`](../docs/archive/CF_CAST_MOLD_WALL_RECON.md)
  — predecessor recon whose Q5 verdict this recon updates.
  Quote: *"Q5 verdict: aesthetic, not blocking"* — context-stale
  per the gasket arc's workshop_clamp_pressure_pa requirement.
- [[feedback-load-bearing-test-fixtures]] — paired with-feature
  + bare-baseline test pattern (flange-with vs flange-without).

## §F-15 Cross-refs

- [`docs/archive/CF_CAST_MOLD_WALL_RECON.md`](../docs/archive/CF_CAST_MOLD_WALL_RECON.md)
  — prior recon establishing the contour-following rind
  (Option A) decision. This recon AMENDS Q5 (clamp ergonomics
  flipped from aesthetic to load-bearing).
- [`docs/CF_CAST_SEAM_GASKET_MOLD_RECON.md`](CF_CAST_SEAM_GASKET_MOLD_RECON.md)
  — gasket arc that introduced the `workshop_clamp_pressure_pa`
  invariant.
- `design/cf-cast/src/piece.rs` `compose_piece_solid` — the
  function S1 modifies to add the flange union.
- `tools/cf-cast-cli/src/config.rs` — `[flange]` block landing
  site at S2.
- `design/cf-cast/src/procedure.rs` — workshop clamp protocol
  prose at S3.

## Status (open)

- Scaffold shipped 2026-05-25 (this commit).
- Awaiting workshop user approval to proceed to S1.
- No code change yet; this is design doc only.
- Workshop iter-3 print remains BLOCKED on cup-half clampability
  pending S1-S5 ship.
- Branch: `dev`, no push, no PR until arc close (per
  [[feedback-omnibus-pr-single-branch]]).
- Parallel arcs in flight on same branch:
  - Seam-gasket-mold arc through S3 ([[project-cf-cast-seam-gasket-mold-s3]],
    commit `fc9f30e7`).
  - Parallel-meshing arc through S2
    ([[project-cf-cast-parallel-meshing-s2]], commit `d3ff8063`).
