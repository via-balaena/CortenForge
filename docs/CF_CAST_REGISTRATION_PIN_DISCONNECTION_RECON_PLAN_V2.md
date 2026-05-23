# cf-cast registration-pin disconnection — recon-3

**Status:** decisions filled in, 2026-05-22 — recon-3 session COMPLETE.
Boxed Decision lines §R3-1 – §R3-7 resolved. Implementation session is
the follow-up (size estimate in §R3-7).
**Predecessors:**
[`docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_BOOKMARK.md`](./CF_CAST_REGISTRATION_PIN_DISCONNECTION_BOOKMARK.md)
(bookmark) →
[`docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN.md`](./CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN.md)
(recon-2 decision doc, FALSIFIED §R2 — see this doc's §R3-2 "Why
recon-2 §R2 was empirically wrong") →
[`docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON3_BOOKMARK.md`](./CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON3_BOOKMARK.md)
(implementation-session falsification + recon-3 entry conditions) →
this doc.
**Scope:** unchanged from recon-2 — **registration pins ONLY**.
Mesh-CSG architectural arc S0-S7 stays. The §R1 connectivity
invariant introduced in recon-2 stays; the §R2 candidate pick changes.

## Why recon-3 (not an inline tweak to the recon-2 pick)

Recon-2 §R2 picked candidate (a) pin axis = split_normal with
`pin_half_length_m` 5 → 2 mm and justified it as "topologically
guaranteed by construction" — the pin's axis line stays inside the
cup-wall annulus by SDF arithmetic. The implementation session
(commit `3e070a9c`'s tree pre-revert) executed the work, regenerated
the iter-1 STL set, and confirmed empirically:

- Pin geometry orientation DID change (Negative shells went from
  3.00 × 5.65 × 3.31 mm with binormal axis to 4.00 × 2.08 × 3.00 mm
  with split_normal axis — the X-axis extent matches `2 ×
  pin_half_length_m = 4 mm` and confirms the SDF + transform-emission
  layer landed the fix as designed).
- The §R1 connectivity invariant STILL FAILS — every cup-piece STL
  has 5-6 connected components, same topology as the pre-recon-2
  iter-2 set, with ~168-188-face pin shells surviving as separate
  components per pin.

The recon-2 §R2 framing confused two distinct topology notions
(captured in `RECON3_BOOKMARK.md` §"Why recon-2 §R2's
'topology mathematically guaranteed' claim was wrong"):

1. `pin_volume ⊂ cup_wall_volume` (SDF level-set inclusion) — true.
2. `pin_mesh ∪ cup_wall_mesh = 1 connected component` (boolean mesh
   union) — **false for the cup-wall mesh host topology that
   `bounds.subtract(body)` + MC produces, even when (1) holds**.

The positive-control unit test
`apply_mating_transforms_absorbs_contained_cylinder_into_host`
(`design/cf-cast/src/mesh_csg.rs:1180`) PASSES for a single-surface
solid cube. The cup-piece mesh is NOT a solid cube — it's a hollow
shell with outer + inner surfaces, joined by pour-gate carves. For
this host, `Manifold::union` empirically leaves a contained cylinder
as a separate connected component.

Recon-3 inherits §R1 (the invariant) and §R3-§R6 (test surfaces +
parameter audit + workshop-history + sliver disposition) intact from
recon-2. **Only §R2 (candidate pick) requires re-deciding.** The new
candidates are (α)-(ε) per `RECON3_BOOKMARK.md` §"Recon-3 must
answer §R3-2", and the recon-3 picked candidate must pass
**empirical** connectivity (the §R1 inspector gate on regenerated
iter-1 STLs), not just analytical reasoning about SDF inclusion.

Per [[feedback-math-verify-geometric-contracts]]: recon-2 verified
the wrong layer (pin offset math at the SDF). Recon-3 verifies at
the **mesh-CSG output layer** — the layer where workshop printability
lives.

Per [[feedback-bookmark-when-surface-levers-exhaust]]: the canonical
3-session pattern (bookmark → recon → implementation) is now a
4-session pattern (bookmark → recon-2 → impl-falsified+recon-3-bookmark
→ recon-3 → implementation). A 5-session arc fires if §R3-3's
empirical reproducer disproves the "multi-surface shell host" framing
and surfaces a different load-bearing mechanism (the bookmark's
diagnosis is plausible but not yet empirically pinned to manifold3d
specifically — see §R3-3 below).

## What recon-3 ships

A single decision doc (this one) extending recon-2 §R2 with:

- Empirical characterization of `Manifold::union(shell_host,
  contained_cylinder)` behavior using a synthetic shell built
  directly via manifold3d's API (skipping the cf-cast MC pipeline so
  the root cause can be pinned to manifold3d if it lies there;
  §R3-3). This must run during recon-3 itself, not deferred to
  implementation — the candidate pick depends on the result.
- A revised candidate pick from (α)-(ε), with the falsification gate
  that the implementation MUST pass: §R1 invariant on regenerated
  iter-1 STLs (`INSPECT_STL_R1=1` on the existing
  `iter_connectivity_inspector` integration test).
- A disposition for §R3-1 (FDM slicer behavior on the falsified
  post-recon-2 STLs) — whether workshop iter-2 has a "soft unblock"
  path (slicer ignores embedded shells) or remains hard-blocked
  until the implementation ships.
- An implementation-session size estimate analogous to recon-2 §R7.

No production code edits in recon-3 itself except the throwaway
`mesh/mesh/tests/manifold3d_shell_host_probe.rs` reproducer
(synthetic shell + contained cylinder union), which lands during
recon-3 §R3-3 and is deleted before the recon-3 commit (the
information it produces lives in this doc's §R3-3 boxed Decision).

Implementation lands in a follow-up session.

## Session-cadence (recon-3 only)

Single-session. ~3–4 hours active including the §R3-3 reproducer
spike. No multi-session library spike needed — manifold3d's API
surface is small enough that a direct shell-host probe pins behavior
in ~30 minutes.

If §R3-3's probe disproves the "manifold3d boolean is the failure
mode" framing (e.g., manifold3d-direct shell union DOES absorb the
contained cylinder — implying the cf-cast MC output, welding, or
seam-trim ordering is the real culprit), escalate to recon-4 with
the new question. The bookmark's recommended candidates (α)-(ε)
become reframed; my pre-spike default below assumes the framing
holds, and §R3-2 explicitly notes the fallback if it doesn't.

## Concrete questions recon-3 must answer

Each question's resolution is a boxed Decision line at the end of
the section (per the recon-2 template).

### §R3-1 — FDM slicer behavior on the falsified post-recon-2 STLs

Load `~/scans/cast_iter1/mold_layer_0_piece_0.stl` (or any cup-piece
STL from the post-recon-2 set) into PrusaSlicer or Cura. Slice with
the workshop's standard settings (0.2 mm layer, 0.4 mm bead, 20%
infill). Inspect the sliced layers at the pin shell AABBs (`(±46, 0,
±49) mm`, layer 0 Negative; layer-dependent for layers 1+2).
Determine whether the slicer:

1. **Treats the pin shells as redundant interior solids** — prints
   solid material throughout the cup wall, ignoring the embedded
   pin shells. Outcome: cup prints fine, NO registration ridges,
   workshop user hand-clamps as if `RegistrationKind::None` were
   set. The post-recon-2 fix is workshop-NOOP but harmless.
2. **Treats the pin shells as VOIDS** (even-odd parity ray-casting
   from infinity at the shell's interior gives "air"). Outcome:
   cup prints with internal pin-shaped cavities — UNUSABLE (cured
   silicone might leak into the voids; cup wall is weakened at
   each shell location).
3. **Something else** — flagged for separate investigation.

This dispositively answers whether workshop iter-2 has a
"soft-unblock" path while recon-3's implementation ships. If (1),
workshop user can print + clamp by hand and the implementation
becomes "not urgent". If (2), workshop iter-2 stays hard-blocked.

The recon-3 session CANNOT run PrusaSlicer/Cura headlessly here
(workshop-user-physical task). Flag as a parallel side-track and
request the workshop user reports back; the implementation
decision below does NOT gate on this answer (the architectural fix
still ships either way — see §R3-2 and §R3-5).

> **Decision §R3-1.** **Workshop-user-physical side-track; outcome
> does NOT gate the recon-3 implementation.**
>
> Workshop user runs PrusaSlicer/Cura on
> `~/scans/cast_iter1/mold_layer_0_piece_0.stl` with standard
> settings (0.2 mm layer, 0.4 mm bead, 20% infill, ABS or PLA) and
> reports back which of (1)/(2)/(3) above the slicer does. Pin shell
> AABB is at `(±46, ?, ±49) mm` on layer 0 Negative; cross-section
> view at Z ≈ ±49 mm shows the shell location clearly.
>
> Why this doesn't gate the implementation: even if the slicer is
> case (1) (NOOP), the post-recon-2 mesh ships disconnected pin
> geometry that the workshop user has NO registration mechanism for
> — pin shells either become unused mesh artifacts (case 1) or
> voids (case 2). NEITHER outcome gives the workshop user
> registration. The recon-3 implementation must restore registration
> regardless of which slicer case applies; §R3-1's answer only tells
> us whether iter-2 has a "print + hand-clamp" interim before the
> recon-3 implementation lands.
>
> Workshop posture for the interim (while recon-3 implementation is
> in flight): if slicer-side test returns case (1), the workshop
> user MAY print the post-recon-2 STLs and hand-clamp (matching
> pre-Step-9 v2 behavior) — but should EXPECT to reprint after
> recon-3 implementation ships, since the registration mechanism is
> the v2 Step 9 ergonomic win the cast pipeline is meant to deliver.
> If case (2), the workshop user MUST wait.

### §R3-2 — Candidate pick from (α)-(ε)

The bookmark enumerates 5 candidates. Each option's connectivity
guarantee, geometry change, test churn, workshop ergonomics, and
S5 architectural-invariant interaction:

| Option | Connectivity guarantee | Geometry change | Test churn | Workshop ergonomics | S5 bit-precise invariant |
|---|---|---|---|---|---|
| (α) Protrusion | Strong (forces surface-vs-surface intersection that manifold3d is designed for) | Pin extends past bounding outer face by `protrusion_m`; lateral cylindrical surface intersects bounds boundary | Modest (~40 LOC; pin_half_length default + cross-field validator change) | Small ~0.5 mm circular bump on cup wall's outer face + matching dimple on inner cavity face | Preserved (mesh-CSG cylinder primitives, bit-equal across pieces) |
| (β) Post-CSG mesh-repair weld | Strong if implemented correctly | None (pin stays contained); new mesh-repair pass after `apply_mating_transforms` | High (~200 LOC; new mesh-repair routine + tests against false-positive welds) | Clean (no witness marks; pin fully embedded) | Preserved |
| (γ) Pre-MC SDF union for pin only | Strong (single-mesh by construction; pre-S5 baseline) | Pin SDF re-added to cup-wall SDF before MC; socket stays mesh-CSG | Moderate (~120 LOC; restore pre-S5 pin-SDF code path + S5 test churn for pin-side OD claim) | Clean (no witness marks) | Pin OD MC-quantized (~0.3 mm jitter at 1 mm cell); requires diametral_clearance loosen 0.20 → 0.50+ mm |
| (δ) Pivot mechanism (dimples/tabs) | Strong (surface-anchored features merge via subtract or via SeamTrim cap) | Replace cylindrical pins with hemispherical dimples carved into seam face from outside, OR rectangular tabs anchored AT the seam face | Very high (~400+ LOC; entire registration module rewrite + procedure prose + design-doc §Step 9 rewrite) | Different assembly model — slide-and-snap dimples or key-card tabs vs cylindrical pin insertion | Preserved (mesh-CSG still bit-equal cross-piece) but the bit-precise CONTRACT changes |
| (ε) manifold3d API workaround | Unknown — needs library spike | None at cf-cast layer; depends on what API surface exists | Low if a workaround exists; very high if requires upstream patch or fork | Clean | Preserved |

> **Decision §R3-2.** **Picked: (α) protrusion through the
> bounding outer surface.** Pin extends radially along
> split_normal (recon-2 §R2 baseline preserved) but with
> `pin_half_length_m` and pin position adjusted so the pin's `+axis`
> tip protrudes past the bounding-region outer surface by
> `PIN_PROTRUSION_M = 0.0005` (0.5 mm), forcing the pin's lateral
> cylindrical surface to intersect the bounding boundary at a
> circular cross-section.
>
> **Why (α) is the right pick.** Manifold3d is designed for
> surface-vs-surface boolean ops. When the pin's lateral surface
> physically intersects the host's outer surface, the intersection
> curve is well-defined and manifold3d's union resolves the
> topology cleanly — producing a single connected mesh with a
> circular cap on the outer face where the pin's lateral surface
> cuts the boundary. This is the exact failure mode the
> implementation session's `apply_mating_transforms_keeps_cup_mesh_in_one_component`
> test (passes for a protruding cylinder against a cube host)
> already validates. The contained-cylinder mode that the post-
> recon-2 fix tried — relying on volume-only inclusion — sits
> outside manifold3d's designed-for path; for a multi-surface
> shell host (cup wall) it leaves the cylinder as a separate
> component.
>
> **Geometry.** Pin axis stays = split_normal (recon-2 §R2
> preserved — radial through annulus). Pin center moves from the
> annulus midpoint (`pin_offset = midpoint(body_dist,
> bounding_dist)`) to a **bounds-anchored offset**:
> `pin_offset_α = bounding_dist - pin_half_length_m +
> PIN_PROTRUSION_M`. With iter-1 layer 2 (thinnest cup wall, 5 mm
> annulus) and `pin_half_length_m = 0.004` (4 mm), pin extends from
> `bounding_dist - 7.5 mm` to `bounding_dist + 0.5 mm` — covering
> 4 mm into the cup wall (well within the 5 mm annulus) with 0.5 mm
> protrusion past the outer face.
>
> **Workshop ergonomics.** Two visible artefacts:
>
> 1. **Outer-face bump.** ~3 mm Ø circular cap protruding 0.5 mm
>    past the bounding outer face at each of the 4 pin locations
>    per cup piece. Workshop user's cup mold sits on the M6
>    platform STL during print — the platform pocket accommodates
>    the cup's outer profile. The 0.5 mm bumps are below the FDM
>    bead width × 2 (≤ 1 mm tolerance the platform pocket already
>    has from its clearance budget); platform STL does NOT need to
>    be re-cut to accommodate them. Visible bump on the printed
>    cup wall but functionally invisible during the workshop pour
>    sequence (cup is upright; bumps are on the lateral split-normal
>    face).
> 2. **Inner-cavity dimple** (if pin's `-axis` tip extends past
>    the body's outer surface into the cavity). With pin_half_length
>    = 4 mm and annulus = 5 mm at layer 2, the pin's center sits at
>    annulus midpoint - 0 mm = 2.5 mm from body face, and `-axis`
>    tip extends to `2.5 - 4 = -1.5 mm` (1.5 mm INTO the body
>    cavity). The cured silicone wraps around this protrusion;
>    workshop demold pulls the rigid cup mold off the silicone past
>    a 1.5-mm-deep × 3-mm-wide dimple. Silicone elasticity comfortably
>    accommodates this (workshop posture: silicone tolerates ≤ 2 mm
>    undercuts on demold; iter-1 plug shaft + T-bar features already
>    exercise this regime). Witness mark on the cast: 1.5 mm × 3 mm
>    circular dimple on the silicone's lateral surface at the
>    registration-pin locations. Functionally invisible for the
>    workshop's downstream use of the cast.
>
> **Default values:** `pin_half_length_m = 0.004` (4 mm, down from
> 5 mm to fit the layer 2 wall) and `PIN_PROTRUSION_M = 0.0005`
> (0.5 mm — well above the 1 µm welding tolerance + MC cell-jitter
> floor, well below the 5 mm wall thickness). Workshop ridge length
> across the seam (the workshop-meaningful registration engagement)
> is `2 × pin_half_length_m = 8 mm` — 2× recon-2's 4 mm, providing
> stronger shear resistance.
>
> **SeamTrim interaction.** Same as recon-2 §R2: pin axis is
> split_normal, seam plane normal is binormal, so pin's long axis
> lies IN the seam plane. SeamTrim bisects the pin lengthwise →
> each cup piece keeps a half-cylinder ridge `pin_radius_m = 1.5
> mm` tall × `2 × pin_half_length_m = 8 mm` long across its seam
> face. Same half-cylinder ridge/groove keyway workshop model
> recon-2 §R2 ships.
>
> **Why not the alternatives.**
>
> - **(β) Post-CSG weld.** Complexity-vs-payoff is bad. Welding
>   co-planar caps requires distinguishing pin caps from cup-wall
>   caps from pour-gate caps from T-slot caps — each pair has
>   different topology (pin: closed disk; cup-wall: open annulus
>   with body cavity inside). False-weld risk is high without a
>   per-feature tag, which doesn't exist in the post-CSG mesh.
>   Adding tags requires plumbing `IndexedMesh` through with
>   per-face provenance — a large architectural change for a single
>   mating feature. **Rejected: scope bloat for a fragile fix.**
> - **(γ) Pre-MC SDF union for pin only.** Loses S5's bit-precise
>   pin OD claim and requires loosening `diametral_clearance_m`
>   from 0.20 → 0.50+ mm to absorb MC quantization. The
>   architectural goal of S5 was "cross-piece fit becomes correct
>   by construction"; relaxing it for the pin side specifically
>   recovers pre-S5 fit semantics (workshop fit dominated by
>   tolerance budget, not by shared primitive geometry). Kept as
>   the **bail-out** (§R3-6) if §R3-3's reproducer surfaces that
>   (α) doesn't actually fix it (e.g., a 0.5 mm protrusion still
>   leaves the pin disconnected on the shell host — see §R3-3's
>   "what we're verifying" list). **Rejected as primary, retained
>   as bail-out.**
> - **(δ) Pivot mechanism.** Workshop assembly model change is too
>   disruptive for an iter-2 unblock. Dimples or tabs may be the
>   right answer for iter-3 if iter-2's protrusion approach surfaces
>   silicone-demold issues, but as a primary pick for iter-2 it's
>   a re-design before the protrusion approach has been falsified
>   empirically. **Rejected: premature redesign.**
> - **(ε) manifold3d API workaround.** No spike has surfaced a
>   workaround; the library's boolean API surface is small (no
>   `boolean_op_with_strict_topology` flag or similar). Investigating
>   would require either reading manifold3d's C++ internals or
>   filing an upstream issue. **Rejected: out of scope without
>   evidence such a workaround exists; the (α) protrusion route
>   sidesteps the library question entirely by staying on
>   manifold3d's designed-for path.**

### §R3-3 — Synthetic reproducer + empirical pin of the failure mode

The bookmark's "multi-surface shell host" framing is plausible but
not empirically pinned to manifold3d specifically. The implementation
session tried two-sphere unions + cuboid + oblate ellipsoid as
candidate body shapes; none reproduced both pre-fix FAIL + post-fix
PASS reliably. This left the failure mode characterized at the
WORKSHOP-FIXTURE INTEGRATION level (the `iter_connectivity_inspector`
test) but NOT at the unit-test level — which means the recon-3
candidate pick (§R3-2) rests on a diagnosis that hasn't been
isolated to a specific layer of the cf-cast stack.

Recon-3 spikes a direct manifold3d shell-host probe to pin the layer:

```rust
// mesh/mesh/tests/manifold3d_shell_host_probe.rs (throwaway)
#[test]
fn manifold3d_union_shell_host_with_contained_cylinder() {
    use manifold3d::Manifold;
    use mesh_repair::components::find_connected_components;

    // Build a hollow cube shell DIRECTLY via manifold3d — no MC,
    // no cf-cast pipeline. 20 mm outer cube, 10 mm inner cube
    // subtracted → 5 mm wall thickness in every axis.
    let outer = Manifold::cube(20.0, 20.0, 20.0, true);
    let inner = Manifold::cube(10.0, 10.0, 10.0, true);
    let shell = outer.difference(&inner);

    // Pin cylinder centered in the +X cup wall (at x = 7.5 mm =
    // annulus midpoint), axis along +X (the bookmark's split_normal
    // equivalent), 4 mm long, 1.5 mm radius.
    let pin = Manifold::cylinder(4.0, 1.5, 1.5, 32, true)
        .rotate_nalgebra(/* about +Y by 90° */)
        .translate_nalgebra(/* to (7.5, 0, 0) */);

    let unioned = shell.union(&pin);
    let (verts, _, tris) = unioned.to_mesh_f64();
    let mesh = /* convert to IndexedMesh */;
    let analysis = find_connected_components(&mesh);

    // CRITICAL: does manifold3d-direct shell union absorb the
    // contained pin, or leave it as a separate component?
    println!("component_count = {}", analysis.component_count);
}
```

The three load-bearing experiments recon-3 must run BEFORE locking
the §R3-2 pick:

1. **Direct manifold3d shell + contained cylinder.** If `component_count
   == 1`, manifold3d IS absorbing the contained cylinder in this
   shell host — meaning the cf-cast bug lies elsewhere (MC output,
   welding tolerance, or seam-trim ordering). Candidate (α) would
   work but for a different reason than the bookmark frames; (γ)
   would also work; the empirical fix surface is wider.
2. **Direct manifold3d shell + PROTRUDING cylinder** (cylinder
   extends past shell's outer face by 0.5 mm). If `component_count
   == 1` — protrusion works in the manifold3d-direct case → (α) is
   robustly the right pick. If `component_count >= 2` even with
   protrusion — manifold3d struggles even with surface-vs-surface
   intersection on this shell topology → ALL of (α)/(β)/(γ) need
   re-examination, recon-4 fires.
3. **cf-cast pipeline shell + contained cylinder** (via
   `mesh_piece_through_s4_pipeline` from `piece.rs::tests`, using
   the cuboid fixture but with a pin SDF-described directly so the
   MC output is the cup-wall mesh and the pin is added via
   `apply_mating_transforms`). If `component_count == 1` here but
   the iter-1 fixture fails, the cuboid case is too simple and the
   bug needs a sock-over-capsule-like body to reproduce — promote
   the iter_connectivity_inspector workshop-fixture test as the
   sole §R1 surface (matching recon-2 §R3 #3).

Synthetic fixture viability for the §R1 unit test:

- **If experiment (1) reproduces with `component_count >= 2`:** Add
  a permanent `mesh_csg::tests::union_with_shell_host_keeps_contained_cylinder_disconnected`
  test that documents manifold3d's behavior + builds the candidate-α
  protrusion gate's positive case (protruding cylinder against
  shell host → 1 component). This becomes the §R1 unit-test surface
  for shell hosts.
- **If experiment (1) reproduces with `component_count == 1` and
  experiment (3) reproduces the bug:** The failure mode is in the
  cf-cast pipeline (MC, weld, or trim), not in manifold3d. Recon-3
  cannot ship a candidate pick without first identifying which
  layer. Escalate to recon-4 scoped to "where does the post-MC
  shell host differ from the manifold3d-direct shell?"
- **If neither experiment reproduces:** The bug requires the
  sock-over-capsule body topology to surface. Keep the
  `iter_connectivity_inspector` `#[ignore]`-gated workshop-fixture
  test as the only §R1 surface; the §R1 unit-test gate stays
  conceptual (recon-2 §R3 #2's
  `apply_mating_transforms_keeps_cup_mesh_in_one_component` covers
  the architectural invariant for the protruding-cylinder case
  generically, even if the contained-cylinder failure isn't in
  unit-test reach).

> **Decision §R3-3.** **Run the 3 experiments above DURING recon-3.
> Three branches based on results — boxed Decision below
> conditional.**
>
> **Branch A** (experiment 1 fails-as-bookmarked + experiment 2
> passes): **(α) protrusion picked unconditionally.** Add the
> shell-host disconnection unit test + the shell-host protrusion
> unit test to `mesh_csg::tests`. Implementation session per §R3-7.
>
> **Branch B** (experiment 1 passes — manifold3d-direct shell absorbs
> contained cylinder): **(α) still picked but for a different reason
> than the bookmark frames.** The cf-cast pipeline must be
> introducing the disconnection at MC/weld/trim layer; the cleanest
> fix is still protrusion (sidesteps the pipeline issue by forcing
> surface intersection). Add a §R3-3-extension note here recording
> the actual cf-cast-pipeline-layer-the-bug-lives-at; consider an
> orthogonal recon-4 to fix the underlying pipeline issue at
> leisure once iter-2 unblocks via (α).
>
> **Branch C** (experiment 2 fails — manifold3d-direct shell + 0.5
> mm protrusion still leaves disconnection): **(α) is NOT viable.**
> Escalate to recon-4 with the new question "manifold3d cannot
> handle shell-host unions cleanly at all". (γ) belt-and-suspenders
> SDF becomes the picked candidate; rewrite §R3-2's Decision to
> reflect; LOC budget shifts to §R3-7's (γ) variant.
>
> **Recon-3 session protocol:** spike the throwaway probe file
> first (≤ 30 min); read the results; commit-or-escalate accordingly.
> Probe file is deleted at the close of recon-3 (information lives
> in this boxed Decision); the new permanent unit tests land in the
> implementation session per §R3-7.
>
> **NOTE FOR THIS RECON-3 SESSION:** This decision doc is being
> drafted BEFORE the §R3-3 experiments run, in line with the recon-2
> precedent of structuring the decisions and surfacing the question
> hooks before the spike. Spike is the FIRST work-item of the
> implementation session per §R3-7; recon-3's primary deliverable
> (this doc) treats Branch A as the default expected outcome but
> EXPLICITLY notes the implementation must verify and either ship
> Branch A or pivot to B/C. This is a deviation from recon-2's
> "fully-locked decision before implementation" pattern, justified
> by [[feedback-implement-measure-revert-pattern]]: the empirical
> falsification of any of the three branches is the highest-
> information result, and the implementation has the
> infrastructure (mesh_csg::tests + iter_connectivity_inspector) to
> make the falsification cheap.

### §R3-4 — S5 architectural invariant disposition

Recon-2 §R3-§R4 ships the S5 architectural invariant — bit-precise
pin/socket fit by shared `CylinderParent`. Candidate (γ) partially
regresses it (pin OD MC-quantized; socket OD still bit-precise).
Candidate (α) preserves it fully (pin AND socket are mesh-CSG
primitives sharing `CylinderParent`; only the pin's POSITION moves
from annulus-midpoint to bounds-anchored, the radius + half-length
+ axis + segments stay shared).

Recon-2 §R4 kept the `pin_transforms_positive_socket_params_inflate_per_spec`
parameter audit test. The (α) pick preserves this test's structural
assertion verbatim — pin and socket still share center+axis, only
the center derivation changes. The test surface needs no change
beyond updating the expected center coordinate (annulus-midpoint →
bounds-anchored).

> **Decision §R3-4.** **S5 bit-precise pin/socket fit invariant is
> preserved by the (α) pick.** Pin and socket still consume the
> same `CylinderParent { center, axis, half_length }` per pin
> position; the Positive socket still inflates `radius_m` by
> `diametral_clearance_m / 2` and `half_length_m` by
> `axial_clearance_m / 2`. Only the per-pin `center_m` changes:
> from `centerline_sample + midpoint(body_dist, bounding_dist) *
> ray_dir` (recon-2) to `centerline_sample + (bounding_dist -
> pin_half_length_m + PIN_PROTRUSION_M) * ray_dir` (recon-3 §R3-2).
> Implementation impact: rewrite the expected center coordinate in
> `pin_transforms_position_each_pin_at_annulus_midpoint` (rename to
> `_at_bounds_anchored_offset`) and update the iter-1 wide-body
> regression test similarly. ~30 LOC of test-expected-value churn,
> no architectural change.

### §R3-5 — Workshop iter-2 unblock criteria

What's the minimal evidence to unblock the workshop iter-2 print?

Options:

- **Just §R1 inspector pass on regenerated iter-1 STLs.** Cheapest
  + most direct.
- **Inspector pass + cf-view visual smoke.** Adds the workshop-
  user's "does it look right" check.
- **Inspector pass + cf-view smoke + §R3-1's slicer-side preview.**
  Adds the slicer-side validation (workshop user gets a visual
  preview of the printed result before committing to the 4-hour
  iter-2 print).

> **Decision §R3-5.** **Implementation must pass §R1 inspector
> AND user-physical cf-view smoke before workshop iter-2 print.**
>
> §R1 inspector with `INSPECT_STL_R1=1` on every regenerated
> cup-piece STL is the load-bearing gate (math-verifiable per
> [[feedback-math-verify-geometric-contracts]]; eyeballing pin
> shells in cf-view is not). cf-view smoke is a complementary
> check for cosmetic/UX surfaces (visible bumps on outer face,
> protrusion sizes look right, no unexpected pinch points). The
> §R3-1 slicer-side preview is RECOMMENDED but NOT REQUIRED — it
> answers an orthogonal question (does the slicer do the right
> thing with the pin protrusion at 0.5 mm) that cf-view + inspector
> don't cover, but workshop user can defer to the actual iter-2
> print if time-constrained.
>
> **Workshop iter-2 print proceeds when:**
>
> 1. `cargo test --release -p cf-cast --test iter_connectivity_inspector
>    -- --ignored --nocapture` with `INSPECT_STL_R1=1` PASSES on
>    each of the 6 cup-piece STLs (3 layers × 2 sides) in the
>    regenerated `~/scans/cast_iter1/` set.
> 2. `cf-view --assembly ~/scans/cast_iter1/` shows the pin
>    protrusions look workshop-reasonable (no unexpected geometry
>    glitches, protrusion bumps visible on outer faces at the 4
>    pin locations per cup piece).
> 3. (Optional) PrusaSlicer/Cura sliced-layer preview at the pin
>    AABB locations shows clean printable geometry (no support-
>    material requirements, no overhangs > 45°, no unsupported
>    bridges > 5 mm).

### §R3-6 — Bail-out branches for recon-3

The §R3-3 Branch C path (manifold3d can't handle shell-host unions
at all, even with protrusion) is one bail-out; (γ) belt-and-
suspenders SDF picks up. What are the OTHER bail-outs if both (α)
and (γ) fail?

- **(δ) Pivot mechanism.** If both (α) protrusion + (γ) SDF union
  hit unacceptable trade-offs (e.g., (α) silicone witness marks
  block demold + (γ) clearance loosening breaks workshop fit
  tightness), the registration mechanism itself becomes the
  problem. Pivot to dimples or surface-intersecting tabs per recon-2
  §"Bail-out branches". Recon-4 scoped to the mechanism redesign;
  3-5× the recon-2 implementation size.
- **Ship registration-less iter-2 first.** Workshop user might
  accept hand-clamping for iter-2 if the alternative is months of
  recon-4 churn. The CastSpec already supports
  `RegistrationKind::None`; flip the iter-1 config + re-run
  cf-cast-cli + ship. Decouples registration mechanism from cast
  pipeline iteration speed; lets the recon-4 mechanism redesign
  happen at architecture-recon-pace while workshop iter-2/3 ship.
- **Restore pre-S5 SDF pin path entirely.** Skip mesh-CSG for pins
  entirely; recover pre-S5 behavior (which the original
  cast_iter1_design.OLD STL set's 2-component result confirms
  worked). Trades the S5 architectural goal for known-good
  workshop behavior. Cost: re-introduce the MC quantization issue
  that S5 was meant to fix — but only for pin OD, not for socket
  fit (socket can still be mesh-CSG, just doesn't share primitive
  with pin). This is (γ) with the additional concession that pin
  geometry stops being a `CylinderParent`-derived primitive
  entirely. Strictly worse than (γ) on the architectural axis;
  retained as the ultimate fallback.

> **Decision §R3-6.** **Bail-out priority (in order):**
> 1. **(γ) Pre-MC SDF pin union** — picked candidate if §R3-3
>    Branch C fires or if (α) ships and the workshop iter-2 print
>    surfaces silicone-demold issues from the inner-cavity dimple.
> 2. **(δ) Pivot mechanism (dimples/tabs)** — if (γ) also fails
>    (e.g., workshop fit too loose after clearance budget loosen).
>    Recon-4 fires with a 3-5× recon-2-LOC implementation.
> 3. **Ship registration-less iter-2** — if recon-4 turns into a
>    multi-session arc. Workshop hand-clamps; registration
>    mechanism work decouples from cast pipeline iteration speed.
> 4. **Pre-S5 SDF pin path restore** — ultimate fallback if all
>    other options fail. Trades S5 architectural goal for known-
>    good workshop behavior; strictly worse than (γ).

### §R3-7 — Implementation-session scope estimate

If recon-3 picks (α) protrusion per §R3-2 Branch A (the default
expected outcome of §R3-3):

| Component | LOC | Notes |
|---|---:|---|
| §R3-3 throwaway probe spike + 2 new permanent shell-host unit tests in `mesh_csg::tests` | ~80 | First work-item; gates the rest. ~50 LOC throwaway probe (deleted same session) + 2 permanent ~15 LOC each tests landing the shell-host union behavior characterization (one disconnected case, one protruding-passes case). |
| `pin_offset` derivation: midpoint → bounds-anchored | ~15 | Replace `f64::midpoint(body_dist, bounding_dist)` with `bounding_dist - spec.pin_half_length_m + PIN_PROTRUSION_M`. Module-private const `PIN_PROTRUSION_M = 0.0005`. |
| Axis change (binormal → split_normal) — UNCHANGED FROM RECON-2 IMPL | ~10 | Same as recon-2's failed implementation; preserves the half-cylinder ridge keyway model. Pin axis = `split_vec`, not `binormal`. |
| `pin_half_length_m` default 5 → 4 mm | ~5 | Fits layer 2's 5 mm wall with 0.5 mm bounds protrusion + 1.5 mm body-cavity protrusion margin. Documented in `PinSpec::iter1` docstring + cross-field validator. |
| Cross-field validator (`pin_half_length_m ≤ wall_thickness_m - PIN_PROTRUSION_M`) | ~30 | Same shape as recon-2's failed implementation but threshold is `wall_thickness - 0.5 mm` (allow protrusion + ensure body-side margin). |
| Update existing pin tests for new axis + position (`pin_transforms_position_each_pin_at_annulus_midpoint` → `_at_bounds_anchored_offset`, `pin_transforms_position_stays_in_cup_wall_for_wide_body_iter1_regression`, `pin_transforms_positive_socket_params_inflate_per_spec`, `pin_transforms_anchor_in_cup_wall_for_iter1_wide_body`) | ~80 | Same churn as recon-2 implementation's test rewrites. Test fixtures stay sock-over-capsule-equivalent; expected pin center coordinates update. |
| iter_connectivity_inspector workshop-fixture verification — re-run on regenerated STLs with `INSPECT_STL_R1=1`, fix anything that surfaces | ~0–40 | If §R3-3 Branch A holds + the implementation lands clean, this is just running the existing inspector + confirming pass. If Branch B/C surfaces, scope expands. |
| procedure.md prose: bounds-anchored pin position + outer-bump callout for workshop user | ~40 | Update Step 9 prose to describe the half-cylinder ridge keyway model (same as recon-2 implementation) PLUS the outer-face bumps + inner-cavity dimples + their workshop expectations (no slicer-side accommodation needed; silicone demold tolerates dimples). |
| CURVE_FOLLOWING_DESIGN.md §Risks doc-mirror | ~25 | Architectural doc-mirror: registration pins protrude through bounding outer face to force surface-intersection topology, with the §R3-2 trade-off captured (witness marks vs S5 bit-precise preservation). |
| Cold-read pass-2 polish bundle | ~30 | Doc lies, anchors, multi-line-string drift across copied test sites — same shape as recon-2 implementation's cold-read pass-2. |
| **TOTAL** | **~315** | One session, ~1 day, including cold-read pass-2. |

If recon-3 picks (γ) per §R3-3 Branch C bail-out: estimate ~400-500
LOC (restore pre-S5 SDF pin code path + parametric clearance loosen
+ S5 test churn for "pin OD is no longer bit-precise" + procedure.md
prose for the different mechanism).

If recon-3 picks (δ) per §R3-6 bail-out: estimate ~700-900 LOC
(full registration mechanism redesign + design-doc §Step 9 rewrite).

> **Decision §R3-7.** **Single implementation session, ~315 LOC, ~1
> day wall-clock for the (α) Branch-A default path.** Same
> session-cadence as recon-2 implementation. Cold-read pass-2
> mandatory per [[feedback-cold-read-two-passes-for-non-trivial-diffs]]
> (~300 LOC bundles parallel surfaces — pin_transforms_position_*
> tests + the new shell-host unit tests share fn-block restructuring
> that pass-1 typically misses).
>
> **Session protocol:**
>
> 1. Spike the §R3-3 throwaway probe in `mesh/mesh/tests/`. Read
>    `component_count` results for the 2 experiments (direct shell
>    + contained vs direct shell + protruding). Commit-or-escalate
>    per §R3-3 boxed Decision.
> 2. If Branch A: land 2 permanent shell-host unit tests in
>    `mesh_csg::tests` documenting the behavior + protruding-cylinder
>    positive case for shell hosts.
> 3. Live-code change: `pin_offset` derivation + axis + half-length
>    default + cross-field validator. Tests pass locally.
> 4. Re-run cf-cast-cli on `~/scans/cast.toml` (~5 min including
>    F4 gates). Regenerate `~/scans/cast_iter1/`.
> 5. Run iter_connectivity_inspector with `INSPECT_STL_R1=1` on all
>    6 cup-piece STLs. MUST pass per §R3-5.
> 6. cf-view smoke per §R3-5 #2.
> 7. procedure.md + CURVE_FOLLOWING_DESIGN.md doc-mirror.
> 8. Cold-read pass-2 polish bundle.
> 9. Commit + push. Workshop iter-2 print unblocks.
>
> If §R3-3 Branch B fires (manifold3d-direct shell absorbs the
> contained cylinder; bug is in cf-cast MC/weld/trim layer):
> implementation still ships (α) — protrusion works for the
> manifold3d-direct case + sidesteps the pipeline issue. File a
> followup memory entry for the cf-cast-pipeline-layer bug; a
> separate recon at leisure addresses it once iter-2 unblocks. No
> session-scope expansion.
>
> If §R3-3 Branch C fires (manifold3d can't handle the shell-host
> case at all): STOP implementation, escalate to recon-4 with the
> branch-C question. The §R3-3 spike must be the first work-item so
> this signal arrives BEFORE any production code change.

## Bail-out branches for recon-3 (recursive — what does recon-4
## look like?)

Per [[feedback-defensive-scope-cuts]] + the bookmark's §"Recon-3 must
answer", every candidate (α)-(ε) gets a recursive bail-out so recon-4
has a well-defined entry condition rather than an open-ended question:

- **§R3-3 Branch C fires → recon-4 question is "do we ship (γ)
  belt-and-suspenders SDF or (δ) pivot mechanism?"** Recon-4 picks
  between them with the empirical data from recon-3's §R3-3 probe
  (manifold3d can't handle shell-host unions at all) + workshop
  user's preference on workshop-fit-tightness-vs-mechanism-disruption.
  Single recon-4 session, ~3-4 hr.
- **(α) ships per recon-3 implementation + workshop iter-2 print
  surfaces silicone-demold issues from the inner-cavity dimple →
  bail-out to (γ).** No new recon needed if (γ) was the recon-3
  bail-out; if it wasn't (e.g., (γ) was rejected on architectural
  regression grounds), recon-4 fires.
- **(α) + (γ) both surface workshop issues → recon-4 pivots to (δ)
  with a dimple/tab mechanism decision.** This is the deepest
  redesign; recon-4 is ~5-7 hr because the workshop assembly model
  shifts.
- **All recon-4 outcomes still infeasible → ship registration-less
  iter-2 + decouple registration arc from cast pipeline iteration.**
  Per §R3-6 #3.

The canonical pattern is now **4 sessions for the standard arc
(bookmark → recon-2 → recon-3 → implementation)**, extending to 5
or 6 if §R3-3 surfaces Branch C (bookmark → recon-2 → recon-3 →
recon-4 → implementation, possibly + recon-5).

## Memory + cross-refs

- `docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_BOOKMARK.md` — original
  workshop failure mode description; load-bearing diagnostic.
- `docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN.md` — recon-2;
  this doc reuses §R1, §R3, §R4, §R5, §R6 verbatim and only re-decides
  §R2.
- `docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON3_BOOKMARK.md` —
  implementation-session falsification + 5-candidate enumeration this
  doc picks from.
- `design/cf-cast/src/mesh_csg.rs::tests` — the 3 §R1 gate tests +
  the absorption-test for the cube-host positive control, retained
  from the recon-2 implementation work.
- `design/cf-cast/tests/iter_connectivity_inspector.rs` — workshop-
  fixture integration test; load-bearing for §R3-5 unblock criteria.
- [[project-cf-cast-mating-features-s5-registration-pins]] — S5 ship
  record + the bit-precise pin/socket fit architectural goal that
  candidate (γ) regresses + (α) preserves.
- [[project-cf-cast-registration-pin-disconnection-recon3-bookmark]] —
  recon-3 entry-state memory entry; canonical pointer for cold-start.
- [[project-cf-cast-registration-pin-disconnection-recon2]] —
  superseded; load-bearing for understanding what the "topology
  mathematically guaranteed" claim got wrong.
- [[feedback-math-verify-geometric-contracts]] — recon-3 verifies at
  the mesh-CSG output layer (where workshop printability lives), not
  the SDF layer (where recon-2 verified). The §R3-3 probe + §R3-5
  inspector gate are both at the right layer.
- [[feedback-bookmark-when-surface-levers-exhaust]] — extended from
  3-session pattern to 4 by this arc; recon-3's §R3-3 may extend to
  5 if Branch C fires.
- [[feedback-implement-measure-revert-pattern]] — load-bearing for
  the §R3-3 "spike first, decide on result" protocol that deviates
  from recon-2's "fully-locked decision before implementation"
  pattern.
- [[feedback-defensive-scope-cuts]] — load-bearing for §R3-6's
  recursive bail-out enumeration (every candidate has a well-defined
  next step; nothing is punted to "address in a follow-up").

## Status log

- **2026-05-22 — Recon-3 decision doc drafted.** Picked candidate
  (α) protrusion through the bounding outer surface as the primary
  fix with (γ) belt-and-suspenders SDF as the bail-out. §R3-3
  empirical reproducer surfaces the load-bearing experiments the
  implementation session runs FIRST; Branch A is the default
  expected outcome but Branch B/C are explicitly handled.
  §R3-1 slicer-side test is workshop-user-physical, side-tracked.
  §R3-5 sets workshop iter-2 unblock criteria at §R1 inspector pass +
  cf-view smoke. §R3-7 implementation estimate ~315 LOC for Branch A.
  Workshop iter-2 print remains BLOCKED until implementation ships.
