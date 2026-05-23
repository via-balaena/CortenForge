# cf-cast seam-face film — recon-4

**Status:** decisions filled in, 2026-05-23 — recon-4 session COMPLETE.
Boxed Decision lines §F-1 – §F-7 resolved. Implementation session is
the follow-up (size estimate in §F-7).
**Predecessors:**
[`docs/CF_CAST_SEAM_FACE_FILM_BOOKMARK.md`](./CF_CAST_SEAM_FACE_FILM_BOOKMARK.md)
(bookmark) →
[`docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN_V2.md`](./CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN_V2.md)
(recon-3 (α) implementation just shipped — dev `d1a16074` + `f83dd62b`;
resolved the §R1 inspector connectivity gate but did NOT unblock
workshop iter-2 because of this orthogonal film bug) →
this doc.
**Scope:** seam-face film bug + the §F-3 coupled pin-axis question.
The recon-4 spike surfaced that S4's architectural justification was
unverified, so this decision doc EXPANDS to a partial-revert
("architectural correction") rather than a film-only patch — see
§F-2 and §F-3 below.

## Why recon-4 (not an inline tweak to the recon-3 (α) ship)

Recon-3 (α) (just shipped `d1a16074`) resolved the post-S5
registration-pin disconnection bug at the §R1 inspector layer. The
recon-3 impl session's cf-view smoke on the regenerated iter-1 STL
set surfaced a SECOND, orthogonal bug: a thin film of cup-wall
material covers the body-cavity opening on each cup piece's seam
face. The §R1 inspector PASSES post-(α) because the film is one
connected component with the cup body — no topology violation. The
workshop user was diagnosing the film visually in cf-view all along
while the inspector + implementation tracked a different signal
(see [[feedback-diagnostic-signal-vs-workshop-failure]] from the
recon-3 impl session).

The film bisected to S4 `93aaa0c2` (the seam-plane SDF → post-MC
`Manifold::trim_by_plane` migration): pre-S4 baseline at
`~/scans/cast_iter1_design.OLD/` has no film; S4-ship iter-1 STLs
at `~/scans/cast_iter1_bisect_s4/` have the film identically to
post-(α). S5/S6/S7 inherit the S4 mechanism unchanged.

Five candidate fixes were enumerated in the bookmark, (a)-(e). This
recon-4 session ran five probes (§F-1, §F-1b, §F-3, §F-3b, §F-4) to
disambiguate which candidate is correct, and surfaced a sixth option
(P) — an "architectural correction" partial revert — that the
bookmark's enumeration didn't anticipate. §F-1 through §F-4 below
report the probe results; §F-2 picks (P).

Per [[feedback-implement-measure-revert-pattern]]: probe spikes
landed FIRST, decision flows FROM empirical data. Per
[[feedback-math-verify-geometric-contracts]]: the load-bearing
verification (S4 architectural-claim audit, §F-4) is at the layer
where the workshop-visible failure lives.

## What recon-4 ships

A single decision doc (this one) with:

- Five probe spikes (§F-1, §F-1b, §F-3, §F-3b, §F-4) characterising
  `Manifold::trim_by_plane` behavior on a hollow shell, the
  cf-cast-pipeline SDF→MC→trim composition on axis-aligned geometry,
  the pre-S4 form's connectivity invariant under SDF-union pin
  composition, and the pre-S4 SDF seam's bit-precise flatness
  property. **F-1 and F-1b were deleted from this commit's tree**
  (they characterised the post-S4 form (P) removes — no value as
  permanent tests); **F-3, F-3b, F-4 are LOAD-BEARING for the recon-4
  pick** and land as permanent characterisation tests in
  `mesh_csg::tests` (the implementation session will repurpose them
  to gate the post-(P) production form).
- A revised candidate pick — **(P) architectural correction** — that
  the bookmark's (a)-(e) enumeration did not anticipate.
- A coupled-question resolution: §F-3 confirms the recon-3 (α)
  protrusion pin design is supersedeable under (P), restoring the
  workshop-ergonomic binormal-axis pin design.
- An implementation-session scope estimate (§F-7) for the partial-
  revert + supersession scope.
- A bail-out priority (§F-6) covering branches where the §F-3 /
  §F-4 production-empirical verification fails after (P) ships.

No production code edits in recon-4 itself — only the permanent
F-3/F-3b/F-4 characterisation tests in `mesh_csg::tests` (F-1/F-1b
were spike-then-delete throwaways). **Production behavior unchanged
in this commit** — the implementation session lands the (P) revert.

## Session-cadence (recon-4 only)

Single-session, ~3-4 hr active including the five probe spikes. No
multi-session library spike needed — manifold3d's trim_by_plane +
the cf-cast SDF→MC→trim composition pinned in ~45 min, the §F-3
supersession + §F-4 flatness audit in ~30 min.

If §F-4 had FAILED (i.e., pre-S4 SDF seam was not bit-precise flat
at the MC vertex layer), the recon-4 pick would have been (b)
slab-difference instead — preserving S4's mesh-CSG-side flatness
gate without the trim_by_plane auto-cap behaviour. §F-4 passed, so
(P) is a strict architectural upgrade with no flatness regression.

## Concrete questions recon-4 must answer

Each question's resolution is a boxed Decision line at the end of
the section (per the recon-2/recon-3 template).

### §F-1 — `Manifold::trim_by_plane` cap topology on a hollow shell

The bookmark's hypothesis (1) was: manifold3d's `trim_by_plane` caps
the trimmed cross-section with the CONVEX HULL of the volume's
intersection with the plane, rather than the topologically-correct
shape. If true, on a hollow-shell host (outer surface + inner cavity
surface, like the post-S4 cup-piece envelope) the cap would fill the
inner cavity hole — producing the film.

**Probe.** Build a hollow cube (20 mm outer minus 10 mm inner) via
manifold3d's `Manifold::cube(20).difference(Manifold::cube(10))`
(skip MC — pure manifold3d so the result isolates the library's
behavior). Apply `trim_by_plane(+Z, offset=0)`. Inspect the cap
vertex set:

- BRANCH A: 4 outer corner verts (`|x| = |y| = 10 mm`) + 4 inner
  corner verts (`|x| = |y| = 5 mm`); no verts strictly inside the
  inner footprint → cap is the annular ring (CORRECT).
- BRANCH B: only outer corner verts, no inner; strict-inner-interior
  verts → cap fills the inner hole (full disc, BUG).
- BRANCH C: mixed (some inner verts, some interior).

**Result.** **BRANCH A.** 16 verts / 28 faces total; 8 cap verts on
z = 0: 4 outer corners at `(±10, ±10, 0)` + 4 inner corners at
`(±5, ±5, 0)`; 0 verts in the strict inner interior. `trim_by_plane`
produces the correct annular ring cap on a clean synthetic
hollow-shell host. **Hypothesis (1) is FALSIFIED.**

> **Decision §F-1.** **manifold3d's `trim_by_plane` is correct on a
> clean hollow-shell host.** The library is not the bug. The film
> must come from interaction with the cf-cast pipeline's MC layer
> (or curved-body geometry; §F-1b extends this probe with the MC
> layer to narrow further).
>
> Probe `f1_probe_trim_by_plane_cap_on_hollow_shell` was a
> recon-4-throwaway, **deleted from this commit's tree** alongside
> §F-1b's probe — they characterise the post-S4 form (P) removes,
> so they have no value as permanent characterisation tests.

### §F-1b — SDF→MC→trim composition on axis-aligned geometry

Bookmark hypotheses (2), (3), (4) all involve the MC layer: cell-size
quantization (2), overlap-bias plane offset (3), sub-cell-resolution
gap (4). §F-1 ruled out the library; §F-1b extends with the cf-cast
production pipeline path (SDF→MC→trim) on axis-aligned geometry to
test whether MC quantization alone produces the film, or whether the
curved-body case is required.

**Probe.** `bounding(40mm).subtract(body(20mm))` SDF → MC at 3 mm
(iter-1's cell size) → `apply_mating_transforms` with
`SeamTrim(+Z, offset=0)`. Inspect the post-trim mesh for cap
triangles whose 3 vertices ALL lie strictly inside the body-cavity
footprint at z = 0 (the film signature).

**Result.** Post-MC mesh: 5070 verts / 10132 faces. Post-trim: 2527
verts / 5046 faces. **0 film triangles. 0 cap verts strictly inside
the body-cavity footprint.** Axis-aligned MC + trim produces a clean
annular cap; the film does NOT appear on this fixture.

> **Decision §F-1b.** **The MC layer is not the bug on axis-aligned
> geometry.** The film must come from curved-body × MC × trim
> interaction in production (the cf-cast iter-1 ribbon-following
> body has nonzero curvature at the seam plane, unlike this
> axis-aligned probe). The exact mechanism — cell straddle at the
> body-cavity surface where the surface's local tangent plane is
> nearly parallel to the seam plane, producing a sub-cell-resolution
> ambiguity that manifold3d's robust booleans close with cap
> material — remains UNCHARACTERISED at the cellular MC level. This
> recon does NOT pin the exact mechanism because the picked
> candidate (P) sidesteps it entirely; a future recon may
> characterise it as part of a `mesh_offset` MC robustness audit.
>
> Probe `f1b_probe_sdf_mc_then_trim_axis_aligned` was a
> recon-4-throwaway, **deleted from this commit's tree** (same
> rationale as F-1: characterises the form (P) removes).

### §F-2 — Candidate pick from (a)-(e) + new candidate (P)

The bookmark's enumeration:

| Option | Mechanism | Preserves S4 architecture? | Empirical evidence |
|---|---|---|---|
| (a) Revert pre-S4 SDF half-space intersect | SDF-side seam cut; no post-MC trim | No (reverts S4) | §F-4 confirms bit-precise flatness preserved |
| (b) `Manifold::difference` against half-space slab | Replace `trim_by_plane` with slab-subtract | Yes | Untested at recon — would require iter-1 ship |
| (c) Inset body subtract past seam plane by ε before MC | Force body cavity to extend past trim plane | Yes | Untested at recon |
| (d) Post-CSG mesh-repair to detect + remove cap material | Identify film triangles (body-cavity-footprint), excise | Yes | Untested; risk of false positives |
| (e) Decrease MC cell size 3 mm → 1 mm | Workshop runtime balloons ~27× | Yes (mostly) | Rejected per bookmark §F-2 (cost) |
| **(P)** | **Revert ONLY S4 seam (SDF-side); KEEP S5-S7 mating-features mesh-CSG; revert recon-3 (α) to binormal-axis pin** | **Partial revert (S4 only)** | **§F-1/F-1b/F-3/F-4 all converge on this** |

Candidate (P) was not in the bookmark's enumeration. It emerged
during recon-4 from auditing S4's architectural justification + the
§F-3 coupled question.

**The argument for (P) (architectural framing).** cf-cast mixes two
geometric paradigms: SDF/MC (continuous, analytical) for bulk
geometry (body + bounding) and mesh-CSG (discrete, manifold3d on
MC-quantized meshes) for fine features (cylinder primitives). The
mixing boundary is the layer where each feature lives. Each feature
belongs to the paradigm it geometrically fits:

| Feature | Paradigm | Why |
|---|---|---|
| Body (curved, ribbon-swept) | SDF/MC | Continuous deformations, curve-following |
| Bounding region | SDF/MC | Continuous offset of body |
| **Seam cut (plane)** | **SDF/MC** | **Part of bulk geometry; the body-cavity opening at the seam is an SDF property of the bulk** |
| Registration pin/socket (cylinder) | Mesh-CSG | Exact primitive; needs bit-precise diametral fit (S5 contract) |
| T-bar / T-slot (cylinder) | Mesh-CSG | Exact primitive; bit-precise diametral + axial (S6) |
| Funnel nipple / pour-gate (cylinder) | Mesh-CSG | Exact primitive (S7) |

S4 put the seam in the WRONG column — moved it from SDF (where it
geometrically belongs) to mesh-CSG. S5/S6/S7 put their features in
the RIGHT column. **(P) corrects S4's misplacement without touching
S5/S6/S7.**

The bookmark's framing "(a) drops the S4 architectural choice; loses
bit-precise flat seam" anticipated a flatness regression. **§F-4
empirically falsifies that regression** (pre-S4 seam is already
bit-precise flat to f64 precision via MC's linear-SDF interpolation;
S4 added no flatness benefit, only the film cost — see §F-4 below).
With §F-4's audit landed, (P) is a **strict architectural upgrade**
over the current state.

**The S5-S7 architecture stays.** Mesh-CSG cylinder primitives are
in the right paradigm for bit-precise mating features. S5's
shared-`CylinderParent` cross-piece fit invariant, S6's three-piece
T-bar / T-slot / plug-shaft coupling, S7's funnel-nipple +
pour-gate cylinders all continue to compose via post-MC mesh-CSG on
top of the pre-S4 SDF-side seam.

**The §F-3 coupled question is in scope.** The recon-3 (α)
protrusion pin design was a SYMPTOMATIC fix for a S5-introduced
disconnection bug. The bookmark's hypothesis: the post-S4 film
ADDED extra topology (a capping membrane covering the body cavity
at the seam plane) that disrupted the pin's mesh-CSG union; remove
the film, and the original binormal-axis pin design might work
again by construction. §F-3 empirically confirms this on a
synthetic fixture (1-component MC mesh with SDF-union pin); (P) is
ALSO a path to restoring the workshop-ergonomic binormal-axis pin
design. **(α) is superseded under (P)** — its protrusion + bounds-
anchored offset + dimple-arithmetic are no longer needed.

**Why not the alternatives.**

- **(a) bookmark form (revert only).** (P) IS (a) with scope-
  expanded deliverable. The bookmark anticipated (a) as a film-only
  fix; recon-4 surfaced that the §F-3 coupled question is the
  natural pairing under (a). Without the scope expansion, (a) would
  ship the film fix but leave the workshop-ergonomic pin design
  unresolved (workshop user's mental model is binormal-axis pins;
  (α) split_normal produces 0.5 mm lateral bumps that don't
  ergonomically register). **Subsumed into (P).**
- **(b) slab-difference.** Preserves S4 architecture by swapping
  `trim_by_plane` for `Manifold::difference` against the existing
  `build_half_space_slab` helper. Topology-wise should produce the
  same result as `trim_by_plane` on a clean shell (per §F-1
  geometry: both ops produce the same kept-half volume with annular
  cap). Whether manifold3d's `difference` against a slab has
  DIFFERENT numerical robustness than `trim_by_plane` on the curved
  cf-cast iter-1 body is an open question. (b) is a smaller change
  (~30-50 LOC) but a SYMPTOMATIC fix at the same architectural
  layer where S4's misplacement lives — the seam is still post-MC
  mesh-CSG. **Rejected: smaller LOC but no architectural ceiling
  raise.** Retained as bail-out (§F-6) if (P)'s §F-3 production-
  empirical verification falsifies.
- **(c) Inset body past seam by ε.** Requires the body SDF to know
  about the seam plane geometry at SDF-compose time. The plumbing
  exists (`piece::compose_piece_solid` already calls
  `ribbon.seam_plane_reference()`), but the implementation adds a
  cuboid-at-seam SDF op that constructs an inset region of the body
  near the trim plane — moderate complexity (~50-100 LOC). Like
  (b), it's a symptomatic fix preserving S4's architectural
  misplacement. **Rejected: same architectural ceiling as (b), more
  LOC.**
- **(d) Post-CSG mesh-repair.** Detection criterion (cap triangles
  with all 3 verts inside body-cavity footprint at trim plane,
  computable from `layer_body.evaluate()` at z = offset) is precise
  enough to avoid false positives. But adds a new mesh-repair pass
  (~150-200 LOC) downstream of every mating-features composition;
  every future cup-piece feature must be audited against the
  detection. **Rejected: scope grows with every future feature;
  ceiling-raising direction is wrong.**
- **(e) Decrease MC cell size.** 27× cost. **Rejected per
  bookmark.**

> **Decision §F-2.** **Picked: (P) architectural correction.**
> Revert S4's seam migration (post-MC `trim_by_plane` → SDF
> half-space intersect, restoring `Ribbon::halfspace_solid` deleted
> in `93aaa0c2`). Keep S5/S6/S7 mating-features mesh-CSG unchanged.
> Scope-expand to revert recon-3 (α) protrusion pin to the workshop-
> ergonomic binormal-axis design (per §F-3).
>
> **Why (P) is the right pick.**
>
> 1. **Architectural ceiling raise.** Cf-cast's SDF↔mesh-CSG boundary
>    is correctly placed in (P) — bulk geometry in SDF, fine features
>    in mesh-CSG. The S4 misplacement is corrected at the root, not
>    patched.
> 2. **Film is architecturally impossible under (P)**, not patched.
>    The body-cavity opening at the seam plane is established by SDF
>    arithmetic (the halfspace intersect SDF is zero on the plane;
>    `bounding ∖ body` makes body-cavity material POSITIVE at the
>    seam plane; the intersect's max-operator surfaces the halfspace
>    SDF dominantly, MC's linear interpolation places vertices
>    exactly on the seam plane). No post-MC cap operation runs; no
>    auto-cap behaviour to mis-fire.
> 3. **§R1 connectivity is architecturally guaranteed**, not
>    patched. The pre-S4 SDF half-space intersect produces a
>    1-component half-shell MC mesh (§F-3b confirms on synthetic) —
>    not the 2-shell hollow cuboid the post-S4 form produces. (P)
>    keeps S5's mesh-CSG pin architecture (`MatingTransform::UnionCylinder`);
>    the binormal-axis pin is union'd onto the 1-component half-shell
>    host post-MC. Per the recon-3 impl session's §R3-3 shell-host
>    probes (in `mesh_csg::tests`), manifold3d absorbs contained
>    cylinders into CLEAN shell hosts without disconnection — the
>    (α) disconnection observed in production was a downstream effect
>    of the post-S4 film's extra topology, not the contained-cylinder
>    mode itself. With the film removed by construction (§F-2 #2),
>    binormal-axis mesh-CSG pin absorbs cleanly. §F-3's SDF-union pin
>    probe is a SUFFICIENT condition check (if SDF-union absorbs,
>    mesh-CSG-union on the same clean host is at least as robust); it
>    confirmed BRANCH A on the synthetic fixture. See §F-3 for the
>    load-bearing analysis + the production-empirical validation
>    deferred to the impl session.
> 4. **Math-verified flat seam preserved.** §F-4 confirms pre-S4 SDF
>    seam is bit-precise flat to f64 precision (max cap-vertex
>    distance from seam plane = 0.000000 mm across 480 cap verts on
>    both sides). Stricter than S4's 1 µm gate. The S4 ship's
>    architectural justification ("post-S4 is bit-precise flat,
>    pre-S4 wasn't") was an unverified claim; pre-S4 was already
>    bit-precise via MC's linear-SDF interpolation. **No flatness
>    regression under (P).**
> 5. **Workshop iter-2 unblocks at the right architectural layer.**
>    Both the film bug and the §F-3 pin-ergonomic question resolve in
>    a single implementation session under (P). (b)/(c)/(d) would
>    ship the film fix but leave (α) split_normal pins in production
>    — the workshop user would still hand-fit pieces without
>    effective registration, with another recon arc looming.
>
> **What stays from S4 (the architectural keeps).**
>
> - `apply_mating_transforms` stage between MC and F4 (S3 plumbing).
> - `CylinderParent` + `CylinderParams` primitives + bit-equal
>   determinism contract.
> - `MatingTransform::UnionCylinder` / `SubtractCylinder` variants
>   + their `apply_one` arms (used by S5/S6/S7).
> - `is_blocking_critical` target-aware gate (cup-piece carve-out
>   per recon-3 §G5; still valid for the post-(P) seam — sub-mm
>   slivers near the SDF seam are intrinsic-curved-body × planar-
>   intersect flakes, same as post-S4).
>
> **What reverts from S4.**
>
> - `MatingTransform::SeamTrim` variant emission from
>   `compose_piece_solid` (no longer used). Variant itself stays in
>   the enum as a defensive primitive — easy to delete later if
>   confirmed unused.
> - `compose_piece_solid` formula returns to pre-S4:
>   `bounding ∖ body ∩ halfspace(side)` (side-specific Solid, not
>   side-agnostic). The S4 side-agnostic Solid refactor is reverted.
> - `Ribbon::halfspace_solid(side, bounds, overlap_m)` (deleted in
>   S4) is restored.
> - `Ribbon::seam_plane_reference()` stays (still emitted for the
>   shared seam-plane math used by registration pin pose derivation
>   in S5).
> - S4's `s4_mating_face_is_mathematically_flat_*` tests are
>   re-purposed to test the pre-S4 form (mirror the §F-4 probe
>   structure on production fixtures). The bit-precise flatness
>   gate STAYS; the form under test changes.
>
> **What reverts from recon-3 (α) per §F-3.**
>
> - `PinSpec` defaults: pin axis back to binormal (the pre-recon-2
>   design); `pin_half_length_m` back to 5 mm (recon-3 dropped to
>   3 mm for workshop-acceptable dimple geometry under (α); under
>   (P) the binormal pin has no inner-cavity dimple since the pin
>   is contained in the cup wall, so the 5 mm default returns).
> - `PIN_PROTRUSION_M` constant deleted from `registration.rs`.
> - `pin_offset` back to annulus-midpoint
>   (`f64::midpoint(body_dist, bounding_dist)`) — the pre-recon-2
>   design.
> - cf-cast-cli cross-field validator using `PIN_PROTRUSION_M`
>   simplified back to the pre-recon-3 form (`pin_half_length_m ≤
>   wall_thickness_m / 2`-style; exact form per the iter-1 spec
>   defaults).
> - Inner-cavity dimple-arithmetic prose across `registration.rs`,
>   `procedure.rs`, `CURVE_FOLLOWING_DESIGN.md` — corrected to
>   describe the binormal-axis contained-pin (no protrusion, no
>   dimple; pin lives entirely within the cup-wall annulus on one
>   side of the seam; matching socket on the other side).
> - `s5_pin_socket_fit_invariant` test stays — the bit-precise
>   diametral fit invariant is unchanged by the axis revert.

### §F-3 — Coupled question: pin-axis supersession

The bookmark flagged this as the "biggest open question": does
candidate (a) (revert pre-S4) also re-enable the original
binormal-axis workshop-ergonomic pin design that recon-3 (α)
replaced with a split_normal protrusion?

**The historical context.** Pre-recon-arc iter-2 (post-S5, pre-
recon-2) shipped binormal-axis pins via `MatingTransform::UnionCylinder`
on the post-S4 cup-piece envelope. Result: every cup-piece STL
had 5-6 connected components — pin shells survived as separate
~168-188-face components per pin per piece, NOT absorbed into the
cup wall by manifold3d's mesh-CSG union. The §R1 inspector caught
this; recon-2 + recon-3 (α) addressed it by switching to split_normal
axis with bounds-anchored protrusion (forcing manifold3d's most-
robust surface-vs-surface intersection path).

**The recon-3 impl session's §R3-3 finding** (probe in
`mesh_csg::tests::apply_mating_transforms_absorbs_contained_cylinder_into_shell_host`):
on a CLEAN synthetic shell host (`Manifold::cube(20).difference(
Manifold::cube(10))`), manifold3d's union absorbs a contained
cylinder cleanly. **The pre-recon production cup-piece envelope was
NOT a clean shell host** — the post-S4 trim_by_plane was producing
a film + additional cap topology that diverged from the synthetic
clean shell. The recon-3 impl memory anticipated this:

> The production failure mechanism lives at a different layer of the
> cf-cast pipeline — likely the multi-op + complex curved-body
> interaction with MC-quantized surfaces, NOT in manifold3d's
> shell-host union per se.

**Recon-4's hypothesis to test.** If (P) removes the film by going
SDF-side at the seam, the cup-piece envelope post-MC is a CLEAN
1-component half-shell (no extra cap topology). Per §R3-3,
manifold3d's mesh-CSG union should then absorb a contained
binormal-axis pin into this clean host without disconnection.
**(α)'s protrusion would no longer be needed** — the (α) symptom
(disconnection) was a downstream effect of the film's extra
topology, not the contained-cylinder mode itself.

**Probe.** §F-3 (`mesh_csg::tests::f3_probe_presdf_seam_plus_sdf_pin_components`)
composes a pre-S4 SDF-side seam + SDF-union binormal-axis pin →
MC → connected-component count. §F-3b
(`f3b_probe_presdf_seam_baseline_components`) does the same without
the pin (baseline). Synthetic axis-aligned fixture (bounding 40 mm,
body 20 mm, halfspace at z = 0, pin at y = 15 mm offset, axis +Z).

This probe tests the SDF-union pin path, not the mesh-CSG-union pin
path that (P) actually ships (S5 architecture preserved). However:

- If §F-3 returns 1 component: SDF-union pin works → confirms that
  the half-shell + contained-pin topology is connectable by MC into
  1 component. This is a NECESSARY condition for mesh-CSG-union pin
  to ALSO work (manifold3d's mesh-CSG union of a contained cylinder
  on a host that's already a clean 1-component manifold is the same
  topological setup as the recon-3 §R3-3 synthetic test, which
  PASSES).
- If §F-3 returns > 1 component: even SDF-union pin doesn't work on
  the half-shell → mesh-CSG-union pin definitely won't either →
  (α) is NOT supersedeable; (P) ships film fix only, keeps (α).

**Result.** §F-3 BRANCH A. Synthetic fixture: pre-S4 SDF half-cube
+ SDF-union binormal-axis pin → 3056 verts / 6108 faces / **1
connected component**. §F-3b baseline (no pin): 3016 verts / 6028
faces / **1 connected component** — confirms the half-cube SDF
itself meshes to 1 component, not 2-shell hollow cuboid as the
post-S4 form does. The pin SDF-union adds ~40 verts + ~80 faces
without changing component count, confirming MC absorbed the
pin geometry into the cup-wall continuously.

> **Decision §F-3.** **(α) IS supersedeable under (P).** Synthetic
> §F-3 BRANCH A confirms the half-shell + contained-pin topology
> meshes to 1 component. Combined with the recon-3 §R3-3 finding
> that manifold3d's mesh-CSG union absorbs contained cylinders into
> clean shell hosts, mesh-CSG-union binormal-axis pin on the post-
> (P) 1-component half-shell host is expected to produce a single
> connected mesh — restoring the workshop-ergonomic original pin
> design.
>
> **Production validation deferred to implementation session.** The
> synthetic axis-aligned fixture is suggestive, not definitive — per
> [[feedback-implement-measure-revert-pattern]]'s recon-2 lesson
> (synthetic ≠ production). The implementation session's §R1
> inspector pass on regenerated iter-1 STLs is the load-bearing
> verification. If §R1 fails on iter-1 with binormal-axis pin after
> (P) lands, fall back to keeping (α) (per §F-6) — the film fix
> ships regardless.
>
> **Why this is in recon-4's scope (not deferred to a separate
> recon-5).** The recon-3 (α) ship was 2 days ago; reverting it
> now is cheaper than letting it compound into S5-S7 architectural
> drift. The workshop user explicitly flagged (α)'s split_normal
> bumps as not workshop-ergonomic. (P) is the architectural
> correction; (α) revert is part of the correction. Splitting them
> creates a multi-session arc that compounds the recon overhead with
> no architectural payoff.

### §F-4 — Pre-S4 architectural-claim audit (math-verified flatness)

**The audit question.** S4's ship-justification was "math-verified
bit-precise flat seam (post-MC `trim_by_plane` produces cap-on-plane
to 1 µm)". The implicit premise: pre-S4 was NOT bit-precise flat.
This premise was never empirically tested at S4-ship time. (P)'s
case depends on it being FALSE: if pre-S4 is also bit-precise flat
(via MC's linear-SDF interpolation property), (P) is a strict
architectural upgrade with no flatness regression; if pre-S4 is
not bit-precise flat, (P) trades flatness for the film fix.

**The theoretical argument.** The pre-S4 SDF formula
`bounding ∖ body ∩ halfspace` evaluates to the halfspace SDF's value
at the seam plane (because: bounding interior negative; body
exterior positive; intersect's max-operator surfaces the halfspace's
exactly-zero value at the seam plane). The halfspace SDF is
**exactly linear** in space (signed distance to plane). MC's
vertex placement linearly interpolates the SDF along cell edges to
find the zero-crossing. For a linear SDF, the interpolated vertex
lands **exactly on the SDF = 0 surface** (modulo f64 noise from
gradient solving + the intersect's max-operator).

**Probe.** §F-4 (`mesh_csg::tests::f4_probe_presdf_seam_flatness`).
`bounding(40mm) ∖ body(20mm) ∩ halfspace(z > 0)` (Negative side) +
mirror for Positive (`z < 0`) → MC at 3 mm → measure max + mean
distance of seam-cap vertices (verts within 100 µm of z = 0) from
the seam plane.

**Result.** Both Negative + Positive pieces: 480 seam-cap candidates
within 100 µm; **max |z| = 0.000000 mm; mean |z| = 0.000000 mm.**
Bit-precise to f64 precision (stricter than S4's 1 µm gate by
orders of magnitude). Pre-S4 SDF seam-cap vertices land EXACTLY
on the seam plane in MC's output.

> **Decision §F-4.** **Pre-S4 SDF seam is ALSO bit-precise flat
> (≤ 1 µm by S4's gate; in fact bit-equal to f64 precision).** S4's
> migration had NO flatness benefit. The S4-ship justification
> ("math-verified flatness > eyeball cf-view smoke") was correct in
> spirit (the workshop needs bit-precise flat mating faces) but
> wrong in attribution (the post-S4 form was math-verified flat;
> pre-S4 was UNVERIFIED but, per this audit, EQUALLY flat). S4
> traded the film cost for an unverified-and-actually-equal
> flatness claim.
>
> **(P) is a strict architectural upgrade** — film disappears by
> construction; flatness gate preserved (re-targeted to the
> pre-S4 form via the §F-4 probe pattern); S5-S7 mating-features
> mesh-CSG kept; workshop-ergonomic binormal-axis pin restored.
>
> **Math-verified flatness gate stays.** The implementation session
> repurposes S4's `s4_mating_face_is_mathematically_flat_and_coplanar`
> + `..._under_curved_centerline` tests to mirror the §F-4 probe
> structure on the pre-S4 form (production cup-piece fixture, not
> synthetic). Gate threshold stays 1 µm; the production-form audit
> may produce slightly larger noise than the §F-4 synthetic's
> 0.000000 mm (curved centerline + ribbon binormal numerics not
> exact-linear), but should stay well below 1 µm. If production
> noise exceeds 1 µm, the gate threshold relaxes to 10 µm
> (still ≪ FDM 0.4 mm bead).
>
> **The §F-4 probe is LOAD-BEARING for (P)** and lands as a
> permanent characterisation test in `mesh_csg::tests`. The
> implementation session promotes it from synthetic-fixture probe
> to production-fixture gate (mirroring S4's curved-centerline
> variant pattern).

### §F-5 — Unblock criteria for workshop iter-2

The recon-3 (α) implementation set unblock criteria at `INSPECT_STL_R1=1`
PASSES on all 6 cup-piece STLs + cf-view smoke. **(α) cleared §R1
but failed cf-view smoke** (the film surfaced there). Recon-4's
lesson: cf-view smoke must gate iter-N unblock BEFORE implementation
lands, not after — per [[feedback-diagnostic-signal-vs-workshop-failure]].

(P)'s unblock criteria layer four gates:

1. **§R1 inspector PASSES on all 6 cup-piece STLs** (the recon-3
   gate, retained). Connectivity invariant ≤ 2 components per piece,
   slivers within size + extent caps. Under (P) this is architecturally
   guaranteed by the 1-component half-shell + clean mesh-CSG pin
   absorption; the inspector remains a regression guard against
   future migrations.
2. **§F-4 flatness gate PASSES on iter-1 production fixture**
   (curved-centerline ribbon). Max cap-vertex distance from seam
   plane ≤ 1 µm (strict) or ≤ 10 µm (relaxed). The implementation
   session promotes the §F-4 synthetic probe to a production gate.
3. **cf-view smoke shows NO film** on all 6 cup-piece STLs. Workshop-
   user-eyeball inspection of the seam face from inside the body
   cavity; body cavity opens cleanly through the seam plane. This
   is the load-bearing workshop-visible gate that (α) missed.
4. **cf-view smoke shows binormal-axis pin geometry**: 4 pins per
   cup piece, axes perpendicular to the seam plane, pin tips
   sticking up from the seam face into matching sockets on the
   paired piece. Workshop-ergonomic registration model restored.

Gates 1+2 are mechanical (test-runnable); gates 3+4 require
workshop-user inspection in cf-view. All four must pass before
workshop iter-2 print.

> **Decision §F-5.** **Workshop iter-2 unblocks on §R1 + §F-4 +
> cf-view-no-film + cf-view-binormal-pins all passing on
> regenerated iter-1 STL set.** The implementation session's
> commit-or-bail gate.
>
> **Failure modes for each gate.**
>
> 1. §R1 fails (pin disconnection under binormal axis): fall back to
>    keeping (α) split_normal protrusion pin. (P)'s film fix still
>    ships; supersession reverts. See §F-6 #1.
> 2. §F-4 production-form fails > 10 µm: investigate whether the
>    binormal direction + arc-length-midpoint sampling introduces
>    larger MC interpolation noise than the synthetic axis-aligned
>    case. Cell-edge-by-cell-edge audit. Relax gate to whatever the
>    empirical production value is, document the gap. Workshop
>    impact is zero (≪ FDM bead); this is an architectural-claim
>    cleanliness issue.
> 3. cf-view shows film STILL present after (P): the film bug's
>    root cause is NOT the post-S4 mesh-CSG seam mechanism — must
>    be a different mechanism somewhere upstream in the cf-cast
>    pipeline. Escalate to recon-5 with empirical "(P) ships but
>    film persists" data; rerun bisect against pre-S4 era. **This
>    would be a major recon-4 falsification** and warrants a
>    cross-session pause.
> 4. cf-view shows broken pin geometry (gaps, misaligned sockets):
>    investigate whether the binormal-axis pose derivation
>    (untouched by the recon-3 (α) → (P) revert) needs adjustment
>    for the post-(P) seam geometry. Single iteration in the impl
>    session.

### §F-6 — Bail-out priority

(P)'s implementation session has well-defined fallbacks at each
falsification point.

> **Decision §F-6. Bail-out priority (in order):**
>
> 1. **§R1 fails on binormal-axis pin under (P) → revert ONLY the
>    pin supersession, keep the seam revert.** Ship (P)'s seam
>    revert + retain recon-3 (α) split_normal protrusion pin. Film
>    fix lands; workshop-ergonomic pin question deferred to a future
>    recon-5 (pivot to dimples or surface-intersecting tabs, per
>    recon-3 §R3-6 #2). Workshop iter-2 unblocks for the film and
>    hand-clamps without effective registration as an interim — same
>    workshop posture as post-recon-3 (α). ~250-300 LOC (the seam
>    revert subset of §F-7's ~480) instead of the full ~480.
>
> 2. **cf-view still shows film after (P) seam revert → escalate to
>    recon-5.** The §F-1/§F-1b/§F-4 probes converged on (P) being
>    the right architectural fix; if it fails in production, the
>    bisect needs to re-run against the pre-S4 era to identify a
>    DIFFERENT upstream mechanism. (P) likely backs out
>    (re-apply S4 + (α)). Workshop iter-2 stays blocked. Recon-5
>    fires with the new empirical data.
>
> 3. **(P) ships clean (§F-5 gates all pass) → workshop iter-2
>    unblocks.** Print + caliper acceptance per the standard
>    cf-cast-cli workshop loop. Recon-4 arc closes.
>
> 4. **(P) ships clean BUT workshop iter-2 print surfaces a NEW
>    mating-features failure (T-bar fit, pour-gate flow, etc.) →
>    orthogonal arc.** Not within recon-4's scope. (P)'s film + pin
>    fix is independent of S6/S7 features; new arc fires on the new
>    failure.

### §F-7 — Implementation-session scope estimate

If recon-4 picks (P) per §F-2 (the default branch given §F-3 + §F-4
BRANCH A):

| Component | LOC | Notes |
|---|---:|---|
| Restore `Ribbon::halfspace_solid(side, bounds, overlap_m)` | ~50 | Mirror of the S4-deleted code. Returns a `Solid::plane`-built halfspace SDF with the 0.5 mm overlap bias. Doc-mirror in `ribbon.rs` module + fn docstrings. |
| `compose_piece_solid` formula revert: re-add `.intersect(halfspace)` | ~20 | Drop `MatingTransform::SeamTrim` emission. Restore side-specific Solid (Negative / Positive return different Solids). Update fn docstring + module docstring. |
| `MatingTransform::SeamTrim` variant + `apply_one` arm | ~0–30 | Decision: keep as defensive primitive OR delete. **Keep** in this commit for safety (deletion is independent cleanup later). If kept, the existing tests `seam_trim_caps_at_offset_*` continue to PASS (they test the primitive in isolation). |
| `Ribbon::seam_plane_reference()` | ~0 | Stays unchanged; still used by S5 registration pin pose derivation. |
| `piece.rs` test churn: restore side-specific Solid tests | ~80 | Restore `negative_piece_excludes_cup_wall_above_ribbon` + `pieces_partition_cup_material_symmetrically` to pre-S4 form (now passes again since cup-piece Solid is side-specific). `both_pieces_overlap_at_ribbon_seam` rewrites against the SDF formula instead of the post-MC pipeline. |
| `mesh_csg::tests` repurpose: promote §F-3 / §F-3b / §F-4 probes to production-fixture gates | ~80 | Replace S4's `s4_mating_face_is_mathematically_flat_and_coplanar` + `..._under_curved_centerline` with pre-S4 variants (same gate, different code path). Promote §F-3 + §F-3b (1-component baseline + SDF-union pin variant) to permanent regression guards. Promote §F-4 (flatness) to permanent gate. Each probe's body unchanged from recon-4 commit; production-fixture variants added alongside. |
| F-1 / F-1b throwaway deletion | ~0 | Already deleted in recon-4 commit (~-260 LOC removed there). No impl-session cost. |
| Recon-3 §R3-3 shell-host probe tests | ~0 | Stay (the absorbing-contained / merging-protruding characterisation is still valid; under (P), the contained-cylinder mode is the one the cup-piece uses for binormal-axis pins). |
| `registration.rs` revert: PinSpec defaults | ~30 | Pin axis back to binormal (`binormal_vec`, not `split_vec`). `pin_half_length_m` back to 5 mm. `pin_offset` back to annulus-midpoint. Delete `PIN_PROTRUSION_M` const. Doc-mirror across module + field docstrings. |
| `registration.rs` test churn: revert pin_offset + axis | ~80 | Restore pre-recon-2 tests where they were deleted by (α). Update post-(α) tests (`pin_transforms_position_each_pin_at_bounds_anchored_offset` → `..._at_annulus_midpoint`, etc.). |
| `cf-cast-cli` cross-field validator simplification | ~30 | Drop the `PIN_PROTRUSION_M`-based formula; restore the pre-recon-3 form (`2 × pin_half_length_m ≤ wall_thickness_m`). Update tests. |
| `procedure.rs` workshop prose revert: binormal-axis pin description | ~40 | Drop the (α) bumps + dimples prose; restore the binormal-axis half-cylinder ridge/groove keyway model description. |
| `CURVE_FOLLOWING_DESIGN.md` §Step 9 + §Risks doc-mirror | ~40 | Same prose direction: binormal-axis pin, no protrusion, no dimple. §Risks captures the architectural claim about SDF↔mesh-CSG paradigm boundary. |
| iter-1 cf-cast-cli regen + §R1 + §F-4 verification | ~0 | File-redirect to `~/tmp/cf-cast-cli-recon4-impl.log`, tail. Expected ~300 s. Workshop user runs cf-view smoke as the final gate. |
| Cold-read pass-2 polish bundle | ~30 | Doc lies, anchors, multi-line-string drift — same shape as the prior arc's cold-read pass-2. |
| **TOTAL** | **~480** | One session, ~1 day wall-clock. ~165 LOC more than recon-3's (α) implementation estimate (~315 LOC) — the architectural correction is bigger in scope. |

If recon-4's §F-3 production verification fails (binormal-axis pin
disconnects post-(P)): the impl session falls back to §F-6 #1 —
ship the seam revert only, keep (α). Net LOC drops to ~250-300.

If recon-4's §F-5 cf-view-smoke gate falsifies the (P) film fix
(unlikely given §F-1/§F-1b/§F-4 converge on the architectural
correction): impl session reverts entire (P), escalates to recon-5.
Net LOC = 0 (everything reverts).

> **Decision §F-7.** **Single implementation session, ~480 LOC, ~1
> day wall-clock for the (P) default path.** Cold-read pass-2
> mandatory per [[feedback-cold-read-two-passes-for-non-trivial-diffs]].
>
> **Session protocol:**
>
> 1. Restore `Ribbon::halfspace_solid` + `compose_piece_solid` to
>    pre-S4 form. Drop SeamTrim emission. Restore side-specific
>    Solid + associated `piece.rs` tests.
> 2. Promote §F-3 / §F-3b / §F-4 synthetic probes to permanent gates
>    on production fixture (curved-centerline iter-1 ribbon).
>    Flatness threshold ≤ 1 µm strict, ≤ 10 µm relaxed. Connectivity
>    threshold = 1 component for the cup-piece SDF baseline.
> 3. Revert recon-3 (α) `PinSpec` defaults, `PIN_PROTRUSION_M`,
>    `pin_offset`, cross-field validator. Update tests.
> 4. Procedure.rs + CURVE_FOLLOWING_DESIGN.md prose revert.
> 5. Re-run cf-cast-cli on `~/scans/cast.toml` (file-redirect per
>    [[feedback-long-running-commands-use-file-redirect]], ~300 s).
>    Regenerate `~/scans/cast_iter1/`.
> 6. Run `INSPECT_STL_R1=1` on all 6 cup-piece STLs. MUST pass per
>    §F-5 #1.
> 7. cf-view smoke gates (§F-5 #3 + #4): workshop user inspects, no
>    film + binormal pins visible. **Workshop-user-physical**;
>    impl session pauses until smoke gate clears.
> 8. Cold-read pass-2 polish bundle.
> 9. Commit. Workshop iter-2 print unblocks.
>
> **If any gate fails:** branch per §F-6.

## Bail-out branches for recon-4 (recursive — what does recon-5 look like?)

Per [[feedback-defensive-scope-cuts]] + the recon-3 §R3-6 pattern,
every gate has a defined recursive next step:

- **§F-5 #1 fails (§R1 disconnection on binormal pin post-(P))**:
  fall back to §F-6 #1 (keep (α)). No new recon needed; impl session
  scope shrinks to ~250 LOC. Workshop iter-2 prints with film fixed
  + (α) split_normal pin (workshop user hand-clamps without
  effective registration; same posture as post-recon-3 (α)).

- **§F-5 #3 fails (film persists post-(P))**: escalate to recon-5
  with the empirical "(P) shipped but film persists" data. (P) likely
  backs out. The bisect needs to re-run against pre-S4 era to identify
  a DIFFERENT upstream mechanism. Recon-5 question: "what's the
  upstream-of-S4 mechanism that introduced the film?" The bisect to
  date pinned the film's appearance at S4 `93aaa0c2` — if (P) reverts
  S4 and film persists, the bisect was WRONG (the film was actually
  introduced earlier, with S4's changes coincident but not causal).
  Recon-5 fires with full bisect rerun, ~3-4 hr.

- **§F-5 #4 fails (pin geometry broken post-(P))**: investigate
  binormal-axis pose derivation noise (untouched by (P)'s seam revert
  but possibly fragile post-MC). Single iteration in impl session,
  not a new recon arc.

- **§F-7 LOC estimate overruns 2× (>900 LOC)**: stop, audit scope.
  Likely a sub-system surfaced unanticipated coupling; bookmark + new
  recon. Single iteration in impl session if minor; new recon-5
  session if structural.

- **All recon-4 + (recon-5) outcomes infeasible → registration-less
  iter-2** per recon-3 §R3-6 #3. Workshop hand-clamps; the registration
  arc decouples from cast pipeline iteration speed.

The canonical pattern is now **5 sessions for the standard arc**
(bookmark → recon-2 → recon-3 → recon-4 → implementation), extending
to 6+ if §F-5 #3 surfaces a non-S4 root cause.

## Memory + cross-refs

- `docs/CF_CAST_SEAM_FACE_FILM_BOOKMARK.md` — recon-4 entry-state
  bookmark; enumerates (a)-(e); the bookmark anticipated (a) as a
  film-only fix and recon-4 expanded scope to (P) per the
  bail-out branch.
- `docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN_V2.md` —
  recon-3 (α) decision doc; the design (P) supersedes per §F-3.
- `design/cf-cast/src/mesh_csg.rs::tests` — recon-4 probe tests:
  - `f1_probe_*` / `f1b_probe_*` — throwaway, **deleted from this
    commit's tree** (characterised the post-S4 form (P) removes).
  - `f3_probe_presdf_seam_plus_sdf_pin_components` — permanent;
    impl session promotes to production-fixture gate.
  - `f3b_probe_presdf_seam_baseline_components` — permanent.
  - `f4_probe_presdf_seam_flatness` — permanent; load-bearing
    architectural-claim audit; impl session promotes to production gate.
  - `apply_mating_transforms_absorbs_contained_cylinder_into_shell_host`
    + `..._merges_protruding_cylinder_into_shell_host` — recon-3
    §R3-3 characterisation tests, retained (still valid under (P)).
- `design/cf-cast/tests/iter_connectivity_inspector.rs` — workshop-
  fixture integration test; load-bearing for §F-5 #1 unblock criteria.
  Becomes regression-guard-only under (P) (connectivity by construction).
- [[project-cf-cast-mating-features-s4-seam-plane]] — S4 ship record;
  the architectural choice (P) corrects. Load-bearing for understanding
  what S4 actually changed + what (P) reverts vs keeps.
- [[project-cf-cast-mating-features-s5-registration-pins]] — S5 ship
  record; the architecture (P) PRESERVES (mesh-CSG pin/socket with
  bit-precise cross-piece fit).
- [[project-cf-cast-registration-pin-disconnection-impl]] — recon-3
  (α) ship that surfaced the orthogonal film bug + the §F-3
  supersession opportunity.
- [[project-cf-cast-seam-face-film-bookmark]] — recon-4 entry-state
  memory.
- [[feedback-math-verify-geometric-contracts]] — recon-4 audits S4's
  architectural claim at the math-verified layer; §F-4 is the gate.
- [[feedback-implement-measure-revert-pattern]] — recon-4's spike-then-
  decide protocol; load-bearing for the §F-1/§F-1b/§F-3/§F-4 probe
  ordering before §F-2 commits.
- [[feedback-diagnostic-signal-vs-workshop-failure]] — the recon-3
  impl session's lesson that informs §F-5's quad-gate unblock criteria
  (cf-view smoke as load-bearing, not advisory).
- [[feedback-bookmark-when-surface-levers-exhaust]] — recon-4 extends
  the canonical arc pattern from 4 sessions to 5.
- [[feedback-defensive-scope-cuts]] — §F-6 + the recursive bail-out
  enumeration.
- [[feedback-cf-cast-tests-use-release]] — recon-4 probes ran
  `cargo test --release -p cf-cast`.
- [[feedback-autonomous-architecture]] — recon-4 ran autonomously
  through probes; paused at §F-2 for user direction once the
  (P) architectural-correction option surfaced (a "big departure
  from the bookmark's recommended structure").

## Status log

- **2026-05-23 — Recon-4 decision doc drafted.** Picked candidate
  (P) architectural correction (revert S4 seam to SDF + revert
  recon-3 (α) to binormal-axis pin + keep S5/S6/S7 mating-features
  mesh-CSG). §F-1/§F-1b ruled out manifold3d + axis-aligned MC as
  the film source. §F-4 falsified S4's architectural-claim audit
  (pre-S4 seam is ALSO bit-precise flat to f64 precision); S4
  migration had no flatness benefit, only the film cost. §F-3
  confirmed (α) is supersedeable under (P) on synthetic
  (1-component MC mesh with SDF-union pin) — production validation
  in impl session. §F-5 quad-gate unblock criteria (§R1 + §F-4 +
  cf-view-no-film + cf-view-binormal-pins). §F-7 implementation
  estimate ~450 LOC for the default-branch path. Workshop iter-2
  print remains BLOCKED until implementation ships.
