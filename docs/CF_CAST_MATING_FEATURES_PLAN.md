> **PARTIAL REVERT (2026-05-23):** S4 (cup-piece seam SDF → post-MC `MatingTransform::SeamTrim`) was reverted by recon-4 (P) per [`docs/CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md`](./CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md) §F-2 + impl `24bdc221`. S7's funnel-nipple sub-migration was reverted by the funnel fix `2bf0bd17`. S5/S6/S7 mating-features mesh-CSG (registration pins, T-bar/T-slot/plug-shaft, cup pour-gate) STAYS. §F-4 audit falsified S4's flatness justification (pre-S4 SDF seam was already bit-precise flat via MC linear-SDF interpolation). The plug-shaft `MatingTransform::UnionCylinder` got a near-end overlap-bias (`PLUG_SHAFT_NEAR_END_OVERLAP_M`) per the recon-4 paradigm-boundary pattern (`6fcdeb0b`). Retained as audit trail.

# cf-cast mating-features arc — risk-mitigating cadence

**Status:** draft, 2026-05-22 — session-cadence plan only.
**Trigger:** workshop iter-1 print (PR #254 / `358d682c`) physically falsified the
mold's mating surfaces:

1. Mating faces between the two cup halves are not flat enough to seal silicone
   liquid-tight.
2. The interlocking registration-pin protrusion on the Positive side is longer
   than the matching socket is deep on the Negative side.
3. The plug T-bar does not fit into the T-slot carved in the cup pieces.

User framing (verbatim, 2026-05-22):

> the actuaul interior mold can be geometrically detailed, but all of the
> mechanical parts/areas of the mold should be simple, and cad like so
> everything fits together via simple geometry, in a way that even fdm
> printers can handle. still tight tolerances

## Root-cause diagnosis

The three defects do not share a *single* root cause — they share an
architectural pattern (mating geometry expressed as SDF + MC instead of
exact-CAD primitives) that lets three distinct mechanisms each contribute
unfit-for-FDM error at the mating surfaces. Naming them separately matters
because they require different mitigations and the original-draft "MC cell
size is the culprit" framing was wrong: MC's linear interpolation reproduces
a *linear* SDF (a true plane, a true cylinder) exactly. A flat seam is not
lost to cell quantization; it is lost to the mechanisms below.

**Mechanism A — CSG composition is not C¹ at feature junctions.**
The per-piece SDF is `bounding_region ∖ layer_body ∩ ribbon_side` plus
unions/subtractions for pins, sockets, pour gate, T-bar, T-slot
(`piece.rs::compose_piece_solid`). Each CSG `min`/`max` glues two locally
linear SDFs at a non-smooth seam. Near where the ribbon plane meets the
body's organic surface or any pin/T-bar cylinder, the combined SDF has
cell-scale derivative kinks, and the MC triangulation around the junction
sprouts irregular triangles. This is what bumps the *mating face near the
body cavity*; the open-air portion of the seam *should* print flat.

**Mechanism B — Independent MC grids per piece.**
The Negative and Positive cup pieces are meshed in **independent**
`solid_to_mm_mesh` calls (`spec.rs:895`) with bounding boxes computed per
piece, so their grid origins do not align. A 10 mm pin (5 mm half-length
each side of the seam) is unioned into Negative on one grid and subtracted
from Positive on a different grid. The cylinder SDF is exact and linear at
its sides + caps, but the grid offset shifts where the cap-disk
triangulation lands; protrusion length and socket depth drift up to one
cell (~3 mm at iter-1) in independent directions. **This is the dominant
mechanism for the pin-vs-socket defect.**

**Mechanism C — FDM bead-line tolerance is a co-defendant.**
A 3 mm Ø pin printed with a 0.4 mm nozzle is *geometrically* oversized
by ~0.15 mm because of bead-extrusion bulge on the outer perimeter; the
matching socket hole is *geometrically* undersized by ~0.15 mm on the
inner perimeter. Net interference ~0.3 mm even with perfect upstream
geometry. The same applies to the T-bar/T-slot at larger absolute
clearance. No mesh-CSG fix recovers this; **the spec needs explicit
per-feature clearance fields** (see the [Clearance-budget table](#clearance-budget-table-g6)
under S0 and the spec-field enumeration under S2).

### Field readings (iter-1 production)

| Knob | Value | File |
| --- | --- | --- |
| `mesh_cell_size_m` | 0.003 m | `~/scans/cast.toml` |
| `RIBBON_PIECE_OVERLAP_M` | 0.0005 m | `piece.rs:64` |
| `PinSpec::iter1.pin_radius_m` | 0.0015 m | `registration.rs:111` |
| `PinSpec::iter1.pin_half_length_m` | 0.005 m | `registration.rs:112` |
| `PlugPinSpec::iter1.t_bar_radius_m` | 0.003 m | `plug.rs` (per memory) |
| `PlugPinSpec::iter1.t_bar_half_length_m` | 0.012 m | `plug.rs` (per memory) |
| `t_slot_radial_slack_m` | 0.0001 m | `plug.rs` (per memory) |

FDM tolerance is ~0.1 mm; the upstream geometry's combined-mechanism error
budget exceeds 1 mm on multiple features. Architectural fix + explicit
clearance budget are both required.

## Architectural fix (the target)

**Interior cavity stays SDF/MC** — that is what gives the cup its curve-
following organic surface fidelity. **Mating features become CAD-exact**: after
MC produces the cup body mesh, mating geometry is applied as mesh-boolean
operations against exact-primitive meshes (true plane, true cylinder).

The seam plane becomes a half-space mesh-trim against the exact ribbon plane,
producing a snapped-flat mating face. Registration pins become an exact
cylinder mesh that is mesh-unioned into the Negative piece **and bit-equal to**
the cylinder mesh subtracted from the Positive piece. T-bar/T-slot becomes the
same shared-primitive pattern across plug + both cup halves. Cross-piece fit
becomes correct *by construction*, not by tolerance.

## De-risked cadence

Risk-mitigation principles:

1. **Document before code.** Bookmark → recon → implementation; the established
   three-session pattern (`feedback_bookmark_when_surface_levers_exhaust`).
2. **Spike before commit.** Mesh-CSG correctness is famously fragile; bound the
   library-choice risk with a throwaway ~200-LOC spike against a real
   iter-1 fixture (not a 500-LOC committed integration that can't be
   reverted). The hand-rolled fallback, if the spike forces it, is its
   own session — never a side-task.
3. **Plumbing-only before behavior change.** A pure-pass-through refactor
   isolates the boundary-design risk from any feature-migration risk.
4. **One feature per session, one falsification gate per session.** A failure
   tells you *exactly* which thing broke.
5. **Matched primitives via shared code.** Codify the architectural invariant
   the bug came from as a cross-piece test
   (`pin_cylinder_mesh_is_bit_equal_across_pieces`).
6. **Cold-read after every non-trivial diff.** Per
   `feedback_cold_read_review_post_ship` and
   `feedback_cold_read_two_passes_for_non_trivial_diffs`.
7. **Physical falsification gate.** The only acceptance that matters is the
   re-printed mold sealing liquid-tight.

### Phase 1 — Plan (2 sessions, no production code)

**S0 — Bookmark.**
File: `docs/CF_CAST_MATING_FEATURES_BOOKMARK.md`.
Inventory *all* mating features in cf-cast.

**Inventory method (G7).** Don't list from memory. Walk every STL pair the
cf-cast-cli example emits — 14 STLs in iter-1 — and document every
face-to-face contact under physical assembly. Likely items, but the walk
must confirm and may add:

- Cup seam plane (Negative ↔ Positive).
- Registration pins (Negative protrusion ↔ Positive socket).
- T-bar/T-slot (plug T-bar ↔ Positive cup half-slot ↔ Negative cup
  half-slot — three pieces share one primitive).
- Plug pin shaft ↔ cup-wall socket at pour-end.
- Platform T-bar pocket ↔ plug T-bar protrusion under assembled mold.
- Pour-gate hole ↔ funnel nipple.
- Layer-N cup ↔ Layer-(N+1) cup contact face (if multi-layer pours stack).
- Assembled mold ↔ platform top surface (workshop rests cup on slab).

For each contact: current SDF encoding (file:line), observed-or-latent
defect, target CAD-exact representation, shared-primitive partners, **and
current clearance value vs target clearance value (G6)**.

**Clearance-budget table (G6).** S0 records the current `*_slack_m` /
`*_clearance_m` fields per mating pair and proposes a target. Baseline
proposals (to be confirmed during S0 by a print-trial spike if needed):
sliding press-fit (pin in socket) = 0.20 mm diametral, 0.50 mm axial;
positional registration (T-bar in T-slot, plug pin in cup socket) = 0.30
mm diametral, 1.00 mm axial; resting contact (funnel nipple, platform
top) = 0.50 mm. These are *targets the spec must surface as named fields*,
not magic numbers in compose-time code.

**Acceptance gate:** re-printed iter-1 mold halves seal flush, all pins
and the T-bar drop into their matching pockets, calipers-verified
≤0.2 mm length/depth mismatch on each mating feature *after* spec
clearance is subtracted.

**S1 — Library spike.**
~200 LOC throwaway crate (kept out of the workspace; deleted at end of
session). Implement three operations on **two** fixtures:
(i) a toy MC-meshed cube fixture for correctness/edge-case isolation, and
(ii) a real iter-1 Negative-piece STL pulled from
`~/scans/cast_iter1_design/`, so library performance is measured on the
actual MC mesh quality the production pipeline emits, not on adversarially-
clean toy input (S1-spike-scope fix).

Operations:
(a) mesh-trim against an exact plane,
(b) mesh-subtract an exact cylinder,
(c) mesh-union an exact cylinder.

Try in series: `manifold-rs` (Google's manifold kernel — exact, well-
tested, mesh-quality guarantees), `csgrs` (BSP-based, pure Rust, simpler
deps), then a hand-rolled half-space-only trim (for the plane case as a
fallback).

Score each on: correctness on edge cases (triangle exactly on plane,
near-tangent cylinder, slivers); output mesh manifoldness; F4 validation
pass-through against the real-fixture output; build cost; license;
**determinism of output vertex order across runs (G3)** — both libraries
have historical reports of hash-set iteration affecting output, and the
shared-primitive bit-equal invariant requires deterministic CSG. If
output is non-deterministic, the spike also evaluates a canonicalization
pass (sort vertices by world-coordinate hash, reindex faces).

**Output:** ADR-style decision memo appended to the bookmark. Spike code
thrown away. The two finalists' deltas — correctness, perf on real
fixture, determinism — go in the memo as a comparison table the recon
references.

**Bail-out branches:**
- If both libraries produce F4-failing meshes on the real fixture →
  hand-rolled fallback. **The hand-rolled fallback is its own session,
  not part of S1.** A correct half-space mesh-trim needs triangle
  classification, edge clipping (three cases per crossing triangle),
  polygon cap retriangulation, manifold-preservation tests — realistic
  budget is 400–500 LOC + tests, not the 150 LOC the draft promised (F2
  fix).
- If hand-rolled is also out of scope → pivot to **Route B** —
  the short-term unblock alternative (inflate `*_clearance_m`
  spec fields, drop `mesh_cell_size_m` to 1.0–1.5 mm globally or
  add a seam-region cell-size override; ships iter-2 print in
  ~1–2 hours of work but leaves the SDF-only architecture
  intact). Closes the architectural arc; iter-3+ will hit the
  same wall.

**S2 — Recon.**
File: `docs/CF_CAST_MATING_FEATURES_RECON.md`.

Concrete questions the recon must answer (each cold-read finding listed
here is a hook):

- **Pipeline boundary placement.** Where the SDF → mesh-CSG boundary
  lives (likely between `solid_to_mm_mesh` and STL export, before F4
  validation, but confirm by tracing one call path end-to-end). Include
  an **architectural diagram** with the data structures that cross the
  boundary.
- **Shared-primitive invariant.** The cylinder mesh added to Negative is
  bit-equal to the one subtracted from Positive. Define the data
  structure (one `IndexedMesh` built once per feature, passed by `Arc`,
  consumed unchanged by every side). Specify the determinism contract
  the CSG library must honor (G3) and the canonicalization pass that
  enforces it if not.
- **World-coordinate assumption (G9).** Confirm `solid_to_mm_mesh` emits
  vertices in world coordinates, not re-origined to local AABB. If it
  re-origins, the shared-primitive pattern needs an inverse transform at
  CSG-application time. Trace via a grep + a debug-print spike call —
  ≤30 min of recon work.
- **Sub-leaf ladder for Phase 3.** Order: S4 (seam) → S5 (pins) → S6
  (T-bar + plug pin/socket). The ordering is an explicit recon decision
  (Q1 below), not a default. **Q1 alternative:** lead with S6 (T-bar)
  because it is the most-coupled three-piece feature; an architecture
  flaw surfaces there first, while less is invested. Counter: revert
  cost is higher. Recon picks an order and documents *why*.
- **T-bar bisection mechanism (G1).** The T-bar cylinder currently
  works *because* it lies in the seam plane and the ribbon-side SDF
  subtraction bisects it. Once the seam becomes a post-MC trim, S6
  must either (a) run the cylinder CSG *before* the seam trim so the
  trim bisects the meshed result, or (b) precompute two half-cylinder
  primitives, one per piece, sharing one parent geometry hash. Recon
  picks one approach.
- **Perf budget (G4).** cf-cast-cli iter-1 currently takes ~6 min
  (per `project_cf_cast_f4_split_asymmetry`). Recon sets a target
  ceiling (proposed ≤15 min for iter-1, ≤2× baseline). Includes a
  spike-measurement back-of-envelope for the chosen library on
  representative mesh sizes. Bail-out strategies: parallel per-piece
  CSG, mating-feature input decimation, lazy STL export.
- **F4 policy (G5).** F4 informs but does not gate. Recon declares F4
  *grade B is acceptable* on mating-feature-modified outputs *provided*
  the physical print at S8 seals. Document which F4 sub-checks
  (TrappedVolume, NotWatertight, ThinWall) are expected to flake on
  CSG output, and the recon's expected failure modes for each.
- **Failure modes from the spike + mitigations.** Whatever S1 surfaced.
- **Test-semantics churn (G8).** Enumerate the tests that need
  *deletion vs rewrite*:
  - `both_pieces_overlap_at_ribbon_seam` — directly tests the 0.5 mm
    SDF bias. Post-S4 this bias *does not exist*; the test is
    meaningless, not broken. **Delete.**
  - `negative_piece_has_single_connected_component_with_iter1_pins_on_wide_body`
    — tests that the registration pin is a connected component of the
    Negative piece mesh. Post-S5 the pin geometry comes from a CSG
    operation, not the SDF; the test still tests a real invariant
    (mesh manifoldness) but the geometry it asserts moves. **Rewrite.**
  - `plug_socket_carves_at_cap_plane_centroid_past_trimmed_centerline_tip`
    — tests SDF-level socket carving. Post-S6 the carve is a mesh CSG.
    **Rewrite at mesh-level.**
  - `pieces_partition_cup_material_symmetrically` and similar
    SDF-evaluate-at-point tests — still valid as SDF-level tests for
    the *pre-CSG* pipeline output, since the SDF is consumed by MC
    before the CSG stage. **Keep.**
  - Recon walks the cf-cast test suite and adds any others missed
    here.
- **Clearance spec fields (G6).** Recon names the new fields and where
  they go: `PinSpec::diametral_clearance_m`, `PinSpec::axial_clearance_m`,
  `PlugPinSpec::t_bar_diametral_clearance_m`,
  `PlugPinSpec::t_bar_axial_clearance_m`, `PlugPinSpec::shaft_clearance_m`,
  `funnel.rs::NIPPLE_CLEARANCE_M`. Defaults from the S0 clearance-budget
  table. Spec deletes the existing `t_slot_radial_slack_m` (replaced by
  the explicit diametral clearance) — see test-churn entry above.

### Phase 2 — Plumbing refactor (1 session, zero behavior change)

**S3 — Post-MC mesh-transform stage.**
Introduce a transform pipeline between `solid_to_mm_mesh` and STL export.
Pure pass-through — no transforms registered yet. New `MeshTransform`
trait/fn signature threaded through `compose_piece_solid` consumers.

**Falsification gate (G2-relaxed).** *Byte-identical* STLs are too strict
— adding a pipeline stage can shuffle `Vec<Triangle>` ordering even with
semantically-identical output. Relax to **geometric equivalence**:
- Same vertex set, compared after sorting by `(x, y, z)` hash.
- Same face set, compared after canonical edge-cycle rotation +
  same-sort hash.
- F4 grade unchanged from `~/scans/cast_iter1_design/` reference.
- cf-view visual diff zero (smoke per
  `feedback_smoke_as_user_facing_cold_read`).

The geometric-equivalence check ships in S3 as a test helper, since it is
reused by S4/S5/S6 to confirm that pre-CSG mesh state is preserved across
unrelated changes.

### Phase 3 — Migrate features, one at a time (3 sessions)

Each session follows the same shape:
(i) remove the feature from the SDF composition;
(ii) add it as an exact-CAD mesh primitive via mesh-CSG;
(iii) write the cross-piece-matching invariant test (shared-primitive hash);
(iv) re-mesh iter-1 with the new pipeline;
(v) cold-read the diff same session.

**S4 — Seam plane.**
Smallest feature, no inter-piece coupling, biggest single visual win.
Replace `RIBBON_PIECE_OVERLAP_M` half-space subtraction with a post-MC
mesh-trim against the exact ribbon plane (with the same 0.5 mm bias as a
2D offset).
**Falsification gate:** cf-view inspection on iter-1, mating face flat to
≤0.1 mm RMS (instrumented metric); F4 result acceptable per the G5
policy (grade A preferred; grade B documented and accepted if S8
physical print still seals).

**S5 — Registration pins/sockets.**
Build ONE exact cylinder mesh per pin, **diametral-clearance-inflated for
the socket variant** (G6): the Negative pin uses radius `r_pin`; the
Positive socket uses the same axis + length but radius `r_pin +
diametral_clearance_m / 2`, and depth `pin_half_length + axial_clearance_m`.
The shared parent is the *axis-and-pose* triple `(center, axis,
half_length)`, not the cylinder mesh itself, since pin and socket are
deliberately different sizes. The invariant test asserts both meshes
derive from the same parent triple via deterministic builders. Also
introduces the new spec fields `PinSpec::diametral_clearance_m` +
`PinSpec::axial_clearance_m` (defaults from S0 budget).
**Falsification gate:** print + caliper. Pin OD vs socket ID matches
diametral clearance ±0.05 mm; pin protrusion vs socket depth matches
axial clearance ±0.10 mm.

**S6 — T-bar/T-slot + plug pin/socket.**
Three-piece coupling: plug T-bar + Positive cup half-slot + Negative cup
half-slot all derive from one parent axis/pose triple. Plug pin shaft +
cup-wall socket: same pattern. Hardest matched-primitive bookkeeping.

**Bisection choice (G1).** S2 picks one of:
(a) run the cylinder CSG *before* the seam trim from S4 so the trim
    bisects the meshed cylinder cleanly into two halves; OR
(b) build two half-cylinder primitives explicitly, one per piece,
    both deriving from the same parent triple + a side enum.

Approach (a) reuses S4's plumbing and produces a guaranteed-matching
bisection by construction. Approach (b) decouples S6 from S4 but doubles
the primitive count and requires a separate half-cylinder mesher. **S2
default recommendation: (a).** S6 implementation can deviate if the
recon argues otherwise.

Also introduces clearance spec fields `t_bar_diametral_clearance_m`,
`t_bar_axial_clearance_m`, `shaft_clearance_m` (G6), and *deletes*
`t_slot_radial_slack_m` (replaced by the explicit diametral field).

**Falsification gate:** print + drop test. T-bar slides into assembled
cup-T-slot without binding; plug pin slides into cup socket without
binding. Both matched to spec clearance ±0.10 mm.

### Phase 4 — Sweep & physical gate (2 sessions)

**S7 — Mating-feature sweep.**
Address residual items from the S0 inventory not covered by S4-S6:
platform T-bar pocket, funnel nipple vs pour-gate, anything iter-2 print
surfaces.

**S8 — Workshop print + cold-read pass.**
Print every STL. Physical falsification gate against the S0 acceptance
criteria. Cold-read the full Phase 3+4 diff (two passes per
`feedback_cold_read_two_passes_for_non_trivial_diffs` — likely warranted
given Phase 3 adds parallel surfaces across three features). Bundle
polish.

**Companion updates (G10).** Phase 3 changed mating-feature shapes and
spec field names; S8 confirms-and-updates (if Phase 3 missed any) the
following downstream consumers:
- `procedure.md` prose mentioning slack values, fits, drop directions.
- cf-view smoke on the assembled mold (all 14 STLs loaded together).
- Example crate(s) referencing renamed/deleted spec fields.
- Any external recipes or docs in `docs/` that reference
  `t_slot_radial_slack_m` or pin half-length fields directly.

**Failure path (G11).** S8 outcome branches:
- **Pass.** Proceed to S9 patterns memo. Bank insights, close arc.
- **Partial pass** (some features seal, others don't). Diagnose per
  failing feature: was it clearance, was it CSG output quality, was it
  print orientation. Add a *single* recon-2 session scoped to the
  failing features only; do not re-open the architectural arc.
- **Full fail** (faces still don't seal, fundamental defect). Open
  recon-2 with a fresh diagnosis pass — the architectural fix may have
  been correct but mis-applied, *or* the real root cause may be
  Mechanism C (FDM bead tolerance) at a magnitude the clearance budget
  underestimated. Recon-2 chooses between (i) increase clearance
  budget + reprint, (ii) post-process printed parts (lapping, sanding,
  resin-coat), or (iii) accept that some seams need a gasket / O-ring
  / hot-glue seal at workshop time. Document in a new bookmark.

### Phase 5 — Bank insights (1 session, optional)

**S9 — Patterns memo.**
What carries forward: the shared-primitive invariant pattern, the
mesh-CSG library cookbook, F4-after-CSG gotchas, the SDF/mesh-CSG hybrid
architecture as a reusable template for any future "organic body + CAD
mating features" component.

## Total scope

**9 mandatory sessions + 1 optional + 1 contingent.**
- S0/S1/S2 — Phase 1 (3 sessions): de-risk + document, no behavior
  change.
- S3 — Phase 2 (1 session): plumbing refactor, zero behavior change.
- S4/S5/S6 — Phase 3 (3 sessions): feature migrations.
- S7/S8 — Phase 4 (2 sessions): sweep + physical gate.
- S9 — Phase 5 (1 session, *optional*): patterns memo.
- **+1 contingent** — hand-rolled CSG fallback if S1 finds no acceptable
  library (own session, *not* a side-task within S1; F2 fix).

## Bail-out branches

Three commits matter most for risk control:

- **S1** kills library-choice risk. If no candidate works on the real
  iter-1 fixture, branch to the hand-rolled fallback session (+1 session
  budget) or pivot to Route B (tolerance bump + finer MC) and end the
  architectural arc.
- **S3** kills plumbing risk. If the boundary refactor is too invasive,
  revert it and apply mating-feature transforms inline at each MC call
  site instead (less clean, still correct).
- **S4** kills first-feature-integration risk. If the seam-plane CSG
  can't produce F4-acceptable meshes (under the G5 grade-B policy) even
  with the spike-vetted library, revert that one feature and consider
  tighter MC for the seam-region only.

After S4, S5/S6/S7 are mechanical applications of the same pattern; risk
drops sharply.

## Cross-refs

- `docs/CURVE_FOLLOWING_DESIGN.md` — v2 multi-piece mold architectural
  source-of-truth. Phase 3 modifies its §"Per-piece SDF construction" and
  §"Risks at the ribbon ∩ mold-cup intersection" sections.
- `docs/WORKSHOP_ITER1_MOLD_RECON.md` — predecessor recon for the four
  iter-1 mold defects that shipped in PR #252; superseded by PR #254.
  This plan addresses what PR #254 *didn't* fix: a different *class* of
  defect (mating-surface unfit-for-FDM error) whose three mechanisms
  (A/B/C above) all bypass the per-defect geometric tightening PR #254
  did.
- Memory: `project_workshop_iter1_cast_pipeline_pr254` (parent arc),
  `feedback_default_numeric_vs_derived_geometry` (related root-cause
  pattern), `feedback_bookmark_when_surface_levers_exhaust`
  (three-session pattern), `project_mesh_sdf_parry_accel_spec`
  (prior mesh-acceleration arc — different problem, similar
  "post-process the SDF pipeline" architectural move).

## Cold-read findings index

The plan's first draft was cold-read same session 2026-05-22. Findings
applied in-place; the labels below let future readers trace which
sections answer which finding.

- **F1** — root-cause diagnosis rewritten: three mechanisms (A CSG-not-
  C¹, B independent MC grids, C FDM bead tolerance), not a single
  MC-cell-size story. Applied: Root-cause diagnosis.
- **F2** — hand-rolled CSG fallback is its own session (~400-500 LOC),
  not a side-task within S1. Applied: S1 bail-outs + Total scope.
- **G1** — T-bar bisection mechanism: S2 picks
  CSG-before-trim vs precomputed-half-cylinders. Applied: S2 + S6.
- **G2** — S3 falsification gate relaxed from byte-identical STL to
  geometric equivalence. Applied: S3.
- **G3** — CSG output determinism + canonicalization pass evaluated in
  S1. Applied: S1 + S2 shared-primitive invariant.
- **G4** — perf budget set in S2 (≤15 min iter-1 ceiling). Applied: S2.
- **G5** — F4 grade-B acceptance policy: F4 informs but does not gate;
  physical print at S8 is authoritative. Applied: S2 + S4 bail-out.
- **G6** — clearance-budget thread: S0 table, S5/S6 spec fields,
  S8 caliper-against-clearance gates. Applied: S0 + S5 + S6.
- **G7** — S0 inventory method: walk every STL pair, not a list from
  memory. Applied: S0.
- **G8** — test-semantics churn enumerated: delete vs rewrite vs keep
  per existing cf-cast test. Applied: S2.
- **G9** — `solid_to_mm_mesh` world-coordinate assumption confirmed
  in S2 by grep + debug-print spike. Applied: S2.
- **G10** — procedure.md / cf-view smoke / example crate / `docs/`
  cross-refs as S8 line items. Applied: S8.
- **G11** — S8 failure path: pass / partial-pass / full-fail branches
  with specific next steps. Applied: S8.
- **Q1** — Phase 3 ordering S4→S5→S6 vs S6-first is an *explicit S2
  decision*, not a default. Applied: S2.
- **Q2** — S1 spike includes a real iter-1 Negative-piece STL
  fixture, not toy-cube only. Applied: S1.

### Pass-2 findings (structural / contradiction)

A second cold-read on the post-pass-1 document caught drift the first
pass missed (per `feedback_cold_read_two_passes_for_non_trivial_diffs`):

- **P2-A** — broken `#g6` anchor link from the Mechanism C paragraph.
  Replaced with anchored link to the S0 clearance-budget table and a
  prose pointer to the S2 spec-field enumeration.
- **P2-B** — risk-mitigation principle #2 still said "150-LOC spike"
  after F2 had raised it to ~200 LOC + own-session fallback. Updated
  to match S1's actual budget.
- **P2-C** — orphan label `(G — explicit boundary spec)` left behind
  in S2's pipeline-boundary bullet. Removed.
- **P2-E** — Cross-refs paragraph for `WORKSHOP_ITER1_MOLD_RECON.md`
  claimed "the failure mode is at the MC-resolution level," directly
  contradicting the rewritten root-cause diagnosis. Rewrote to point
  at the three-mechanism story.
- **P2-F** — S4 falsification gate said "F4 validation still grade A,"
  contradicting G5 policy. Reworded to defer to G5.
- **P2-L** — S8 "confirms downstream consumers updated" → "confirms-
  and-updates (if Phase 3 missed any)" — Phase 3 sessions are
  responsible for the updates; S8 is the safety net.
- **P2-N** — Route B was referenced as a bail-out but never defined in
  the plan body (only the chat). Added a one-paragraph definition
  inline at S1's bail-out.
- **P2-P** — Total scope said "~9 sessions, +1 contingent" but the
  actual count is 9 mandatory + 1 optional + 1 contingent. Itemized.

## Deferred items (locked by decision, not omission)

S0–S7 shipped the architectural fix across M1 (seam) / M2 (registration
pins) / M3 (T-bar) / M4 (plug pin shaft) / M5 (funnel-nipple + cup
pour-gate). Two items from the arc are explicitly NOT shipped and are
locked here so a future session reader sees the decision rationale
inline rather than reconstructing it from session memories.

- **`funnel::FLANGE_AXIAL_STANDOFF_M`** (recon §9 M5). Recon proposed
  a 0.5 mm physical lip on the funnel flange underside so the flange
  doesn't wobble on the cup's curved + MC-jittery outer surface. S7
  attempted to ship the lip as a post-MC contact-ring mesh-CSG
  primitive; the resulting <1 mm-thick ring fails F4 ThinWall
  (`FUNNEL_MAX_CELL_SIZE_M = 1 mm` blocks below-cell-size mesh
  features). **Disposition: deferred.** Reopen only if iter-2
  workshop physical print physically surfaces a flange-wobble defect
  during pour. The two implementation paths remain (a) drop
  `FUNNEL_MAX_CELL_SIZE_M` below 0.5 mm and ship the lip as an SDF
  primitive, or (b) lap a flat patch on the cup outer face under the
  flange footprint. Don't reopen pre-emptively — S0 bookmark M5 noted
  the standoff was "likely needed" but the FDM-bead bulge on a 20 mm
  flange may already provide sufficient bearing.

- **M6 platform T-bar pocket**. Stays as the pre-S0 SDF cuboid carve;
  not migrated to mesh-CSG. **Disposition: locked unchanged.**
  Rationale: (a) workshop iter-1 print physically validated the
  current 4 mm lateral × 5 mm axial cuboid slack without a binding
  complaint (S0 bookmark M6 "Likely no spec change"); (b) the cuboid
  pocket has no shared-primitive cross-piece coupling (one platform
  STL, one pocket — Mechanism B from the §"Mechanism A/B/C"
  diagnosis doesn't apply); (c) workshop ergonomics favor the
  generous slack over a tight CAD-precise fit. Reopen only if a
  future print physically surfaces a fit issue.

If either deferral fires in iter-2, scope it as a recon-2 (single
session, no architectural-arc reopen) per §G11 partial-pass branch.

## Status log

- **2026-05-22 — Plan drafted.** Initial cadence + diagnosis.
- **2026-05-22 — Cold-read pass 1 applied.** F1/F2 + G1-G11 + Q1/Q2
  integrated.
- **2026-05-22 — Cold-read pass 2 applied.** P2-A/B/C/E/F/L/N/P
  resolved structural drift introduced by pass 1. Plan ready for S0
  (bookmark) next session. No production code touched yet.
- **2026-05-22 — S0/S1/S2/S3/S4/S5/S6/S7 shipped on dev.** Eleven
  commits `ba74b1a8` (plan) through `ae53ee93` (S7 polish); each
  session's memory has the per-ship detail. Phases 1+2+3+4 all
  complete; F4 grade preserved or improved across every migrated
  feature; cf-cast-cli iter-1 wall-clock came in ~10 s *under* the
  S5 baseline (mesh-CSG is faster than the SDF subtract it replaced —
  recon §6 predicted ≤2 s additional; actual is negative).
- **2026-05-22 — S8 Phase A shipped on dev.** Cross-session cold-read
  pass across the eleven-commit arc; companion-doc polish bundle
  (procedure.rs prose surfaces S5/S6 clearance values + post-S4 seam
  semantics; CURVE_FOLLOWING_DESIGN.md §Step 3 + §Risks updated to
  acknowledge the SDF → mesh-CSG migration); deferred items locked
  here (§"Deferred items" above). cf-view smoke + S8 Phase B physical
  print are workshop-user steps and don't ship from this session.
  Per §G11, S8 Phase B is the next inflection — workshop user prints
  the iter-2 STLs at `~/scans/cast_iter1/`, caliper-verifies the
  acceptance gate, and the outcome branches (pass / partial-pass /
  full-fail) decide whether S9 (patterns memo) or recon-2 follows.
- **2026-05-22 — S8 Phase B BLOCKED on registration-pin disconnection
  regression.** cf-view smoke during the S8 Phase A close surfaced
  a triangular "film" inside the cup cavity; chasing the diagnostic
  via env-var-gated shell inspector revealed that every iter-2 cup
  piece contains 6 connected components (1 main cup + 4 floating
  registration pin/socket shells + 1 sub-mm sliver), where the
  pre-S5 baseline had 2 (cup + sliver). Root cause: post-S5
  mesh-CSG pin cylinders don't topologically merge with the cup
  wall because the pin's binormal axis sweeps Y values where the
  cup wall doesn't exist at the pin's X. Pre-S5 SDF-side union
  didn't have this problem (SDF arithmetic is contiguous; mesh
  topology isn't). Test gate didn't catch it because the S5 rewrite
  of `negative_piece_has_single_connected_component_*` to a
  transform-parameter audit dropped the connected-component check
  per `[[feedback-workaround-removal-verification]]`. Documented
  in [`docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_BOOKMARK.md`](./CF_CAST_REGISTRATION_PIN_DISCONNECTION_BOOKMARK.md);
  recon-2 plan opened at [`docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN.md`](./CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN.md).
  Workshop iter-2 print HALTED pending recon-2 + implementation.
