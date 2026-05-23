> **SUPERSEDED (2026-05-23):** Resolved by recon-4 (P) — see [`docs/CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md`](./CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md) §F-2/§F-3. The (α) split-normal protrusion pin design was reverted to the pre-recon-2 binormal-axis annulus-midpoint design per recon-4 §F-3 + impl `24bdc221`. Workshop iter-1 plug STL connectivity (orthogonal to the registration arc) fixed by the recon-4-pattern plug-shaft overlap-bias `6fcdeb0b`. Retained as audit trail.

# cf-cast registration-pin disconnection — recon-3 bookmark

**Status:** bookmark, 2026-05-22 — recon-2 implementation FALSIFIED;
recon-3 has not yet run.
**Branch state:** dev — implementation work staged but NOT
committed; tests + clippy clean but the §R1 falsification gate fails
on regenerated iter-1 STLs.
**Predecessors:**
[`docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_BOOKMARK.md`](./CF_CAST_REGISTRATION_PIN_DISCONNECTION_BOOKMARK.md)
(bookmark) →
[`docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN.md`](./CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN.md)
(recon-2 decision doc) → this bookmark (recon-2 implementation
session's falsification).

## TL;DR for the cold reader

Recon-2 picked candidate (a) **pin axis = split_normal** + drop
`pin_half_length_m` 5 → 2 mm. Implementation session executed the
work: ~430 LOC across `cf-cast::registration` (axis change +
docstring), `cf-cast-cli::CastConfig::validate` (cross-field gate),
`cf-cast::procedure` (half-cylinder ridge prose), `cf-cast::mesh_csg`
(generic §R1 gate), and a promoted workshop-fixture
`tests/iter_connectivity_inspector.rs`. **178 cf-cast tests + 51 cf-
cast-cli tests + clippy `-D warnings` all green.** Workshop iter-1
re-run produced 11 STLs in 305 s.

**Inspector verification on the regenerated iter-1 STLs FAILED §R1:**
every cup-piece STL still has 5-6 connected components — same
topology as the pre-recon-2 iter-2 set. Pin shells survive as
separate ~188-face components even though:
- Pin position is at the cup-wall annulus midpoint along `+X`
  (verified mathematically: `body.evaluate > 0` AND
  `bounds.evaluate < 0` at pin's `(X, Y, Z)`).
- Pin's split_normal axis keeps the pin's volume entirely INSIDE the
  cup-wall annulus volume along its full length.
- The pin's surface is cut by the seam-trim plane at the same Y
  offset (`+0.5 mm` overlap) as the cup-wall mesh's seam cap,
  giving a co-planar half-disk that SHOULD share boundary geometry
  with the cup-wall trim cap.

But manifold3d's `Manifold::union(cup_wall_mesh, pin_cylinder)`
returns a 2-component result for this specific topology even with
`pin_volume ⊂ cup_wall_volume + co-planar trim faces`. The
implementation-session unit test
`apply_mating_transforms_absorbs_contained_cylinder_into_host`
empirically confirms manifold3d DOES absorb a fully-contained
cylinder into a simple cube host (1 component); it documents that
the multi-surface shell host produced by `bounds.subtract(body)` →
MC has a different behavior the recon-2 plan didn't anticipate.

## Diagnostic evidence (this session's iter-1 re-run)

Regenerated cup-piece STLs at `~/scans/cast_iter1/`:

| Cup piece | Components | Largest | Other |
|---|---|---|---|
| layer_0_piece_0 (Negative) | 6 | 16224 faces | 4 × ~248-face pin shells + 1 × 24-face sliver |
| layer_0_piece_1 (Positive) | 6 | 16094 faces | 4 × ~188-face socket shells + 1 × 24-face sliver |
| layer_1_piece_0 (Negative) | 6 | 16994 faces | 4 × ~168-face pin shells (smaller in layer 1) + sliver |
| layer_1_piece_1 (Positive) | 6 | 16918 faces | 4 × ~188-face socket shells + sliver |
| layer_2_piece_0 (Negative) | 6 | 17906 faces | 4 × ~168-face pin shells + sliver |
| layer_2_piece_1 (Positive) | 5 | 18038 faces | 4 × ~188-face socket shells + sliver (Positive layer 2 missing one shell) |

**Pin shell signature changed per the axis fix.** Pre-recon-2 iter-2
Negative shells were `3.00 × 5.65 × 3.31 mm` (Y axis = binormal,
~5.65 mm = `2 × pin_half_length=5 mm` halved by trim). Post-recon-2
iter-1 Negative shells are `4.00 × ~2.08 × 3.00 mm` (X axis =
split_normal, 4 mm = `2 × pin_half_length=2 mm`; Y = lateral
half-cylinder cross-section after seam trim). So the axis change
DID land at the geometry layer — pin orientation is now radial
through the annulus per recon-2 §R2. **The fix did not fail at the
SDF or transform-emission layer; it failed at manifold3d's boolean
union step.**

## Why recon-2 §R2's "topology mathematically guaranteed" claim was
## wrong

The recon-2 §R2 argument: "By construction of `pin_offset =
midpoint(body_dist, bounding_dist) along +split_normal`, the pin's
axis line lies entirely inside the cup wall AT THE PIN'S `(Y_c,
Z_c)` SLICE as long as `pin_half_length ≤ annulus_half_thickness`.
Topological connectivity is mathematically guaranteed, not
geometry-dependent on body cross-section shape."

The argument confused two different topology notions:

1. **`pin_volume ⊂ cup_wall_volume`** (SDF-level set inclusion).
   This IS guaranteed by the geometry — verified mathematically:
   at every point in the pin's volume, `body.evaluate > 0` AND
   `bounds.evaluate < 0`, so the point is in cup-wall material.

2. **`pin_mesh ∪ cup_wall_mesh = 1 connected mesh component`**
   (boolean mesh union output). This is what `Manifold::union`
   ACTUALLY does, and it depends on whether the two manifolds'
   surfaces intersect.

For a **single-surface host** (e.g., a cuboid), manifold3d's union
absorbs a fully-contained cylinder into the host — empirically
confirmed in `mesh_csg::tests::apply_mating_transforms_absorbs_contained_cylinder_into_host`
(the implementation session's positive-control test passes with 1
component).

For a **multi-surface shell host** (e.g., `bounds.subtract(body)`
MC output with separate outer + inner surfaces), manifold3d's union
of a contained cylinder DOES NOT absorb the cylinder — the cylinder
survives as a separate component. The post-CSG mesh has the host's
surfaces + the cylinder's surface as 3+ disjoint components.

Adding the SeamTrim AFTER the union doesn't help because each
manifold's trim cap is computed independently — the pin's cap and
the cup-wall's cap are co-planar but not shared geometry.

The recon-2 §R3 #2 generic gate
(`apply_mating_transforms_keeps_cup_mesh_in_one_component`) only
passes because its fixture is a single-surface cube + a
**protruding** cylinder (whose surface PHYSICALLY intersects the
cube's outer surface). For a non-protruding fully-contained
cylinder it would have failed similarly.

## Implications

**Workshop iter-2 print is still BLOCKED.** The post-recon-2 STL
set has the same `slicer-eats-pin-shells-as-air-voids-or-ignores-
them` failure mode as pre-recon-2:

- Best case: FDM slicer treats the pin shells as solid inclusions
  inside the cup-wall material and prints solid cup wall (no
  registration ridges). Workshop user has no piece-to-piece
  registration — same outcome as `RegistrationKind::None`.
- Worst case: FDM slicer treats the pin shells as VOIDS (even-odd
  parity ray-casting from infinity gives air at the pin's interior),
  printing cup wall with internal voids. Workshop user has cup wall
  with hollow pin-shaped holes inside the material — UNUSABLE.

**Either way, no functional registration mechanism is printed.**

## Recon-3 must answer

### §R3-1 — What does the FDM slicer actually do with the post-fix
### pin shells?

Load `~/scans/cast_iter1/mold_layer_0_piece_0.stl` into PrusaSlicer
or Cura. Inspect the sliced layers near the pin shell locations
(`(±46, ?, +49) mm`). Does the slicer:
- Print solid material throughout (ignoring the pin shell as a
  redundant interior surface)?
- Print voids (treating the pin shell as an air boundary)?
- Print something else (e.g., infill that's different from
  surrounding cup wall material)?

This dispositively answers whether the post-recon-2 fix is
workshop-bad or workshop-noop. If it's workshop-noop (no
registration but no voids), the workshop user could still print +
clamp by hand, and the fix has effectively reverted to
`RegistrationKind::None` behavior.

### §R3-2 — What's the actual fix for "make the pin manifold-merge
### with the cup-wall manifold"?

Options:

- **(α) Move pin position so its surface PROTRUDES through the
  cup-wall's outer surface.** Pin extends past the bounds face by
  `protrusion_m` mm so its lateral surface intersects the bounds
  boundary. The bounds boundary cuts the pin, manifold3d's union
  merges them. **Risk**: bound-overhang must fit FDM print posture
  + may interfere with the workshop platform / other layers.
- **(β) Manually weld the pin's seam-trim cap to the cup-wall's
  seam-trim cap.** Post-CSG mesh-repair: detect coplanar trim caps
  and weld coincident vertices. **Risk**: adds a new mesh-repair
  pass to the cf-cast pipeline; needs careful implementation to
  avoid false-positive merges.
- **(γ) Use a pre-MC SDF approach for registration pins only**
  (belt-and-suspenders option (d) from recon-2 §R2). Add the pin
  SDF to the cup-wall SDF BEFORE marching cubes, so MC produces a
  single connected mesh. Pin and socket of a pair still share the
  mesh-CSG `CylinderParent` for the bit-precise cross-piece fit
  (the S5 architectural goal), but the workshop-visible pin
  geometry lives in the SDF, not in mesh-CSG. **Risk**: loses S5's
  "bit-precise OD" benefit for the pin shape (MC quantizes the
  SDF-side cylinder at the cell size). Workshop fit comes from the
  mesh-CSG socket, which still gets bit-precise from the
  CylinderParent.
- **(δ) Pivot the registration mechanism away from cylindrical
  pins.** Use a different geometry that's naturally
  surface-intersecting (e.g., a half-spherical dimple on each piece
  carved INTO the seam face from outside; or a rectangular tab
  that extends past the cup-wall's outer face). **Risk**: bigger
  redesign; workshop user's assembly experience changes.
- **(ε) Use manifold3d's `compose` or some other manifold3d API
  variant** that might handle the multi-shell-host + contained-
  cylinder case better. Library spike required.

Recon-3 picks one of (α)-(ε) (or a recon-introduced variant) and
runs the implementation session.

### §R3-3 — Is the workshop iter-1 sock fixture too complex for
### diagnostic synthesis?

The synthetic-fixture §R1 test `cup_pieces_stay_connected_with_
registration_pins_on_non_convex_body` (added during this session,
then renamed/downgraded to a transform-parameter audit) hit similar
component-survival issues on a simple non-convex body (oblate
ellipsoid; sphere ∪ sphere; cuboid). Either the bug is more general
than "sock-over-capsule specific" (every cup-wall ≠ shell host
manifests it), or my synthetic fixtures were too aggressive.

Recon-3 should converge on a SYNTHETIC unit-test fixture that
reproducibly demonstrates:
- Pre-fix: §R1 fails (the issue the bookmark first surfaced).
- Post-fix: §R1 passes (the issue is fixed).

If no synthetic fixture can reproduce both states, the §R1 test
must permanently live as a workshop-fixture `#[ignore]` integration
test, and the implementation session's confidence in the fix relies
on the workshop-fixture run, not a unit test.

## Status of this session's work

**Code changes are staged but uncommitted.** The implementation
session's work is:
- `design/cf-cast/src/registration.rs`: axis change (binormal →
  split_normal) + `pin_half_length_m` default 5 → 2 mm + docstring
  updates.
- `tools/cf-cast-cli/src/config.rs`: cross-field validator
  `2 × pin_half_length_m ≤ wall_thickness_m - 1 mm`.
- `design/cf-cast/src/procedure.rs`: workshop assembly prose
  rewritten for half-cylinder ridge/groove keyways (4 keyways, 4
  mm long, 1.5 mm engagement depth).
- `design/cf-cast/src/mesh_csg.rs`: 3 new tests (positive case,
  negative-overlap case, contained-cylinder absorption
  documentation).
- `design/cf-cast/src/piece.rs`: 1 new
  axis-correctness test for the post-recon-2 transforms (the §R1
  synthetic-fixture test was dropped — fixtures can't reproduce
  the bug reliably).
- `design/cf-cast/tests/iter_connectivity_inspector.rs`: new
  `#[ignore]` workshop-fixture integration test (promoted from
  the recon-2 throwaway `mesh/mesh/tests/stl_shells_inspector.rs`,
  which is deleted).
- `docs/CURVE_FOLLOWING_DESIGN.md` §Risks: design-doc mirror of
  the registration-pin axis discussion.

**Recommended disposition.** Recon-3 can either:
1. **Discard this session's changes** entirely + restart from the
   pre-implementation state. Procedure.md prose + docstrings would
   roll back; ~430 LOC of work is thrown away. Cost is the
   ~3-4 hr of implementation + cold-read time invested.
2. **Keep the workshop-fixture inspector** (recon-2 §R3 #3 —
   the kept-around diagnostic that has independent value beyond
   this arc) and the generic `mesh_csg::tests` gates (recon-2 §R3
   #2 — covers a real architectural invariant) + revert the
   live-code changes (axis, half-length default, validator,
   procedure prose). The diagnostic surfaces are independent of
   the pin geometry change and are useful for recon-3 + future
   mating-feature work.
3. **Stage this session's work on a feature branch**
   `feat/registration-pin-disconnection-recon2-falsified` and let
   recon-3 reuse it as a baseline. Recon-3 can pick from (α)-(ε)
   and either build on top or revert.

Default for recon-3 entry: **option (2)** — keep the diagnostic
infrastructure but revert the (axis + validator + half-length)
live-code changes. Recon-3's implementation can re-add them on top
of whatever new direction it picks.

## What to read first (recon-3 cold-start)

In order:
1. This bookmark end-to-end.
2. `docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN.md`
   §R2 (picked candidate analysis — note the "mathematically
   guaranteed" claim that this bookmark falsifies).
3. `docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_BOOKMARK.md`
   (the original diagnostic — still the load-bearing description
   of the workshop failure mode).
4. The current iter-1 STL set at `~/scans/cast_iter1/` for
   inspection in PrusaSlicer/Cura.
5. The implementation-session diff (uncommitted on dev):
   `git diff` to see what was staged.
6. Memory entries (Cmd-F in `MEMORY.md`):
   `[[project-cf-cast-registration-pin-disconnection-recon2]]`
   (recon-2 decisions + this implementation's work-item list);
   `[[project-cf-cast-mating-features-s5-registration-pins]]`
   (S5 ship + the IGNORED-test-rewrite gap that originally hid
   this bug).

## Status log

- **2026-05-22 — Recon-3 bookmark written + recon-2 implementation
  session FALSIFIED.** 11/11 STLs regenerate; every cup-piece STL
  fails §R1 with 5-6 components (same topology as pre-recon-2 iter-2
  set). Workshop iter-2 print remains BLOCKED. Recon-3 session is
  next.
- **2026-05-22 — Recon-3 session COMPLETE.** Decisions written to
  `docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN_V2.md`
  §R3-1 – §R3-7. Picked candidate (α) protrusion through the
  bounding outer surface with (γ) belt-and-suspenders SDF as named
  bail-out. §R3-3 surfaces a load-bearing empirical reproducer
  (direct-manifold3d shell-host union probe) the implementation
  session must run FIRST to confirm or refute this bookmark's
  "multi-surface shell host is the failure mode" framing. §R3-1
  slicer-side test is workshop-user-physical, side-tracked.
  §R3-7 implementation estimate ~315 LOC; workshop iter-2 print
  remains BLOCKED until implementation lands.
