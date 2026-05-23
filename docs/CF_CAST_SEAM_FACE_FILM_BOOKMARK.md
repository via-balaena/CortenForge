# cf-cast seam-face film — bookmark

**Status:** bookmark, 2026-05-23 — workshop iter-2 print BLOCKED.
**Branch state:** dev (clean, pushed to `origin/dev` at `f83dd62b`).
**Predecessors:**

- [`docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN_V2.md`](./CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN_V2.md)
  (recon-3 (α) protrusion arc, just shipped at `d1a16074` + `f83dd62b`).
  Resolved the §R1 connectivity invariant (post-S5 disconnected
  pin shells); workshop iter-2 STILL blocked on this orthogonal
  bug.

## TL;DR for the cold reader

Workshop iter-2 print is blocked by a thin sheet of cup-wall
material ("film") that covers the body-cavity opening on each
cup piece's seam face. The film has the 4 registration-pin features
visible as small holes/bumps embedded in it. Expected geometry: the
seam face has a horseshoe-shaped open hole where the body cavity
crosses the seam plane (the silicone cast lives in this cavity
region; the assembled mold's two pieces meet at the seam plane with
the body cavity continuous through both).

The bug is independent of recon-1/recon-2/recon-3's
registration-pin-disconnection arc — those addressed the §R1
inspector's connectivity-invariant failure (pins surviving as
separate connected components in the mesh topology). The §R1 inspector
PASSES post-(α) (`d1a16074`) but the cup pieces are STILL
workshop-broken because of this film.

## Diagnostic evidence (cf-view smoke + bisect, 2026-05-23)

Workshop user inspected `mold_layer_0_piece_*.stl` in
`cf-viewer --assembly ~/scans/cast_iter1/` (post-(α) STLs) and
identified the failure mode: looking from inside the cavity through
the seam plane, the body-cavity opening is closed by a thin layer of
cup material with the 4 pin features visible through it. The user
also noted the pin features "face out instead of toward each other"
— but the pin-orientation question is separate (recon-3 §R3-2 (α)
intentionally points pins radially through the cup wall; whether the
workshop-ergonomic pin should be along binormal instead is a related
open question, see §"Open coupled question" below).

Provenance bisect (this session, 2026-05-23): regenerated iter-1
STLs at each S-ship commit (S4 `93aaa0c2`, S5 `0ddef82b`, S6
`8e4f6a2d`, S7 `4751a416`) into `~/scans/cast_iter1_bisect_s{4..7}`.
Inspected each in cf-view:

- **`~/scans/cast_iter1_design.OLD/`** (pre-S4 baseline, ~PR #254
  era) — **no film**. Body cavity opens cleanly on the seam plane.
- **`~/scans/cast_iter1_bisect_s4/`** (S4 `93aaa0c2`) — **film
  present** identically to post-(α). The seam-plane mesh-CSG
  migration is the introducing commit.
- S5/S6/S7 — not visually re-inspected separately (film persists by
  construction; the SeamTrim mechanism that landed in S4 is the
  inherited root cause).

S3 plumbing `7c9c772d` is pure pass-through (zero behavior change
per the S3 ship memory entry), so behavior matches the OLD baseline
without separate verification.

## Root-cause hypothesis (preliminary; recon-4 must confirm)

S4's seam-plane migration (`piece.rs::compose_piece_solid`):

**Pre-S4** — SDF half-space intersection BEFORE marching cubes:

```rust
let halfspace = ribbon.halfspace_solid(side, bounds, RIBBON_PIECE_OVERLAP_M);
let base_piece = bounding_region
    .clone()
    .subtract(layer_body.clone())
    .intersect(halfspace);
```

MC samples a single continuous SDF that already encodes the seam
half-space. At the seam plane, the SDF is exactly zero in cup-wall
material (the halfspace's `dot(p - midpoint, n) = 0` plane) and
positive in body-cavity material (because `-body_sdf > 0` there).
MC generates no isosurface face in the body-cavity region at the
seam plane → seam face naturally has a horseshoe-shaped open hole.

**Post-S4** — SDF subtract only, full hollow envelope, then post-MC
`Manifold::trim_by_plane`:

```rust
let base_piece = bounding_region.clone().subtract(layer_body.clone());
// ...later, in apply_one() for MatingTransform::SeamTrim:
let offset_mm = offset_m * METERS_TO_MM;
m.trim_by_plane_nalgebra(&normal.into_inner(), offset_mm)
```

The Solid is now the **full** hollow cup envelope (around the body
on both halves of the seam). MC produces a complete watertight
shell. manifold3d's `trim_by_plane` then bisects this shell and
**closes the cut surface with a cap** to keep the manifold
watertight (per manifold3d's contract — every Manifold is a closed
volume).

The hypothesized cap shape: manifold3d caps the volume's
cross-section at the trim plane. For a hollow shell (cup wall
annulus volume between body cavity inner surface and bounding outer
surface), the cross-section at the seam plane is a 2D ANNULAR region
(outer bounds outline minus inner body-cavity outline). The
**expected** cap shape is this annular ring (cup-wall material
filled, body-cavity hole open). The **observed** cap shape includes
material across the body-cavity opening too — producing the "film".

This needs confirmation in recon-4 — likely candidates for why the
observed cap doesn't match the expected annular ring:

1. **manifold3d's `trim_by_plane` cap fills the convex hull of the
   cross-section** rather than the topologically-correct shape.
   Library-level behavior; would affect every hollow-shell mesh.
2. **MC quantization at 3 mm cell size leaves the inner body-cavity
   surface not perfectly closed near the seam plane**, so manifold3d
   sees an "almost closed" mesh whose body-cavity opening at the
   seam plane is interpreted as wall material to cap. The S4 ship
   would have surfaced this in a synthetic finer-cell test but the
   workshop-fixture iter-1 cell is 3 mm.
3. **The `RIBBON_PIECE_OVERLAP_M = 0.5 mm` bias** offsets the trim
   plane 0.5 mm into the opposite half. The cap is on a plane
   slightly offset from the actual mesh's "natural" seam. The cap
   plane might intersect cells where the inner body-cavity surface
   hasn't been fully resolved by MC, leaving a thin film artifact.
4. **The body subtract SDF at the trim plane produces a
   sub-cell-resolution gap** between the MC-discretized inner
   surface and the trim plane, and manifold3d's robust booleans
   fill the gap with cap material.

Hypotheses (3) and (4) are the most likely. The S4 ship test
`seam_trim_caps_at_offset_and_removes_opposite_half` (in
`mesh_csg::tests`) verified the trim's cap-on-plane property at a
single point but didn't check the cap's 2D shape against the
expected annular cross-section.

## What recon-4 must answer

### §F-1 — Confirm the manifold3d trim-cap shape on a synthetic hollow-shell host

Build a throwaway test mirroring the cf-cast pipeline at small
scale: a hollow cube (20 mm outer minus 10 mm inner) meshed via MC
(or via direct manifold3d ops to skip the MC layer), then
`trim_by_plane`d at the center. Inspect the cap shape:

- If the cap is the expected annular ring (outer square outline
  minus inner square outline) → manifold3d's trim is correct;
  hypothesis (2)/(3)/(4) is in play (MC quantization or overlap
  bias is the issue).
- If the cap covers the full square (filling the inner hole) →
  manifold3d's trim has a behavioral bug on multi-surface
  manifolds; hypothesis (1) is in play.

This test is the analog of the recon-3 §R3-3 shell-host probe — a
small synthetic reproducer that pins the failure mode at the right
layer of the stack.

### §F-2 — Candidate fixes

Pre-listed by failure-mode hypothesis:

- **(a) Restore the pre-S4 SDF half-space intersect for the seam
  plane.** Drops the S4 architectural choice; pre-S4 ergonomically
  worked (workshop iter-1 print on `cast_iter1_design.OLD` had no
  film). The trade-off lost: post-MC seam trim was load-bearing
  for the S4-S7 mating-features mesh-CSG migration because it
  produced bit-precise (1 µm) flat seam faces vs the MC-quantized
  ~0.5 mm-stepped SDF seam. Workshop relevance of bit-precise flat
  seam is unclear — sub-mm stair-stepping on the seam plane is
  below FDM print resolution (0.4 mm bead). Investigate whether
  the bit-precise flat seam matters for the assembly fit.

- **(b) Use `Manifold::difference` against a half-space slab
  instead of `trim_by_plane`.** The existing
  `mesh_csg::build_half_space_slab` helper builds a finite slab; a
  `difference(slab)` operation removes the half-space material with
  STANDARD boolean semantics (no auto-cap). manifold3d's
  documented quirks around `trim_by_plane` and coincident faces
  (recon §10 S2-B issue #1516) may be the smoking gun; switching
  to slab-difference sidesteps it.

- **(c) Inset the body subtract by a small ε past the seam plane
  before MC.** Force the body cavity to extend 1-2 mm past the
  seam plane on both sides; MC then samples a fully-closed inner
  surface near the trim plane, eliminating any sub-cell gap that
  could cause the cap to spill over. Cost: body cavity is slightly
  enlarged (~ε); silicone shells become slightly thinner near the
  seam. ε of 0.5-1 mm should be acceptable workshop-fit.

- **(d) Post-CSG mesh-repair to detect + remove cap material
  inside the body cavity at the seam plane.** Adds a new mesh-CSG
  stage that re-opens the cap. Complexity is high (need to identify
  cap faces vs cup-wall annulus faces); false-positive risk on
  removing legitimate cap material.

- **(e) Decrease MC cell size at the cup-piece compose stage.**
  3 mm cells → 1 mm cells. ~27× cost in MC compute (cubic). Total
  iter-1 runtime would balloon from 300 s to ~2-3 hours. Not
  workshop-acceptable.

Recon-4 picks one of these (or a recon-4-introduced variant) and
runs the implementation session. The §F-1 reproducer's branch
result determines which is the right pick:

- §F-1 shows manifold3d trim cap covers convex hull → (b) is
  the right pick (sidestep `trim_by_plane`).
- §F-1 shows manifold3d trim cap correct on synthetic but wrong on
  cf-cast pipeline → (c) is the right pick (MC + overlap-bias
  interaction).
- §F-1 surfaces a different root cause → recon-4 picks accordingly.

### §F-3 — Open coupled question: pin orientation

Separately from the film, the workshop user has flagged that the
recon-3 §R3-2 (α) pin axis (split_normal — radial through cup wall)
produces lateral 0.5 mm bumps on the cup outer face but no
workshop-meaningful pin-into-socket registration at the seam plane.
The user's mental model is binormal-axis pins (perpendicular to seam
plane, sticking up from one piece and into a matching socket on the
other piece).

The recon-3 split_normal pick was load-bearing for §R1 connectivity
(the (α) protrusion through the bounding outer face forces
manifold3d's union onto its most-robust surface-vs-surface
intersection path). Binormal-axis pins don't have a natural
"protrusion through a host surface" — they live entirely within
the cup-wall annulus volume and tripped the §R1 disconnection in
production iter-1.

If the film fix (recon-4) restores the pre-S4 SDF half-space
intersect (candidate (a) above), the pin disconnection failure
mode may NOT re-surface — because pre-S4 the seam was an SDF
operation that ALSO subsumed any pin SDF additions into one
continuous SDF, which MC would mesh as a single connected
component. Worth confirming this in recon-4: if reverting to pre-S4
seam handling unblocks the workshop AND restores the original
binormal-axis pin design, the (α) work may be supersedeable.

This is the recon-4 design space's biggest open question.

## Suggested recon-4 session structure

Single-session, ~3-4 hr active:

1. **Spike** the §F-1 synthetic reproducer (~30 min). Read result.
2. Pick fix candidate per §F-2 based on §F-1 result.
3. Spike the picked candidate in a throwaway worktree (~1 hr).
   Verify the film disappears on iter-1 STLs.
4. Verify the §R1 connectivity invariant still PASSES on the fixed
   iter-1 STLs (the inspector at
   `tests/iter_connectivity_inspector.rs`). If (α) preserves: ship
   the film fix on top of (α). If (α) becomes superseded: write
   the supersession into the recon-4 decision doc + plan
   implementation as a partial-revert.
5. Document decisions in
   `docs/CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md` per the standard
   bookmark → recon → implementation pattern (extending the
   canonical arc count to ~3 sessions for this specific failure
   mode).

## Why this is bookmarked + not directly implemented

Per [[feedback-bookmark-when-surface-levers-exhaust]]: shipping a
seam-face fix on top of (α) without recon-grade analysis carries the
recon-2 lesson (the "topology mathematically guaranteed" claim that
got falsified empirically). The manifold3d `trim_by_plane` behavior
on hollow-shell hosts is a library-level question that deserves an
empirical reproducer before code lands. Per
[[feedback-implement-measure-revert-pattern]], the §F-1 synthetic
reproducer is the cheap empirical falsification gate that pins which
of §F-2's candidates is the right pick.

## Read-first for the recon-4 session

1. This bookmark end-to-end.
2. `git show 93aaa0c2 -- design/cf-cast/src/piece.rs` (the S4
   compose_piece_solid changes — pre-S4 SDF intersect vs post-S4
   side-agnostic + post-MC SeamTrim).
3. `design/cf-cast/src/mesh_csg.rs::apply_one` (the trim_by_plane
   call site).
4. `design/cf-cast/src/mesh_csg.rs::build_half_space_slab` (the
   existing helper for the candidate (b) slab-difference approach,
   currently exercised only by S3 unit tests).
5. The 4 bisect STL sets at
   `~/scans/cast_iter1_bisect_s{4..7}/` + the post-(α) STLs at
   `~/scans/cast_iter1/` and the OLD baseline at
   `~/scans/cast_iter1_design.OLD/`.
6. Memory entries:
   `[[project-cf-cast-mating-features-s4-seam-plane]]` (S4 ship +
   the bit-precise flat-seam claim);
   `[[project-cf-cast-registration-pin-disconnection-impl]]` (the
   recon-3 (α) implementation that just landed).

## Status log

- **2026-05-23 — Bookmark written + (α) implementation closed.**
  The recon-3 §R3-2 (α) implementation (commits `d1a16074` +
  `f83dd62b`) resolved the §R1 pin-disconnection bug on the
  regenerated iter-1 STL set (2 components per piece, baseline
  topology restored). cf-view smoke on the same STL set surfaced
  this orthogonal seam-face film bug. Bisect on S4/S5/S6/S7 commits
  pinned the film's provenance to S4 `93aaa0c2`. Workshop iter-2
  print remains BLOCKED. Recon-4 session is next.
