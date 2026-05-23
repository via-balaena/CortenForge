> **SUPERSEDED (2026-05-23):** Resolved by recon-4 (P) — see [`docs/CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md`](./CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md) §F-2/§F-3. The (α) split-normal protrusion pin design was reverted to the pre-recon-2 binormal-axis annulus-midpoint design per recon-4 §F-3 + impl `24bdc221`. Workshop iter-1 plug STL connectivity (orthogonal to the registration arc) fixed by the recon-4-pattern plug-shaft overlap-bias `6fcdeb0b`. Retained as audit trail.

# cf-cast registration-pin disconnection — bookmark

**Status:** bookmark, 2026-05-22 — diagnostic captured, recon-2 not yet
run.
**Branch state:** dev `14f57b1e` / `98e0a317` (S8 Phase A polish);
**S8 Phase B physical print BLOCKED** until this regression is
resolved.
**Predecessors:** S0 → S8 Phase A of the mating-features arc
(`docs/CF_CAST_MATING_FEATURES_PLAN.md`); the architectural fix's
[S5 sub-leaf](./CF_CAST_MATING_FEATURES_PLAN.md#phase-3--migrate-features-one-at-a-time-3-sessions)
introduced this defect.
**Successor:** [`docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN.md`](./CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN.md).

## TL;DR for the cold reader

Every iter-2 cup-piece STL contains the 4 registration pins as
**topologically disconnected closed shells**, not integrated into
the main cup mesh. Pre-S5 SDF-meshed pins were one connected
component with the cup; post-S5 mesh-CSG pins float. FDM slicers
would treat each pin as a separate island: workshop user prints the
cup + 4 separate ~5 mm pin columns sitting on the build plate next
to the cup, requiring **manual glue-up**. That defeats the entire
mating-features-arc value proposition. The defect was caught by
S8 Phase A's cf-view smoke + an env-var-gated shell inspector;
falsification-of-S5-correctness diagnostic done same session.

## How this surfaced

S8 Phase A close: user kicked off cf-view `--assembly` on
`~/scans/cast_iter1/` (the iter-2 STL set). Visible "residual film"
inside the cup cavity in screenshots
(`2026-05-22 at 9.09.57 PM.png` + 3 more). My first read attributed
the film to the body-cavity inner surface visible through
transparency — which is correct for that specific film. But running
a connected-component check via `mesh-repair::find_connected_components`
on `mold_layer_0_piece_0.stl` to chase the diagnosis surfaced an
unrelated regression: every cup piece has 6 components instead of 1.

### Diagnostic evidence

| Cup piece | Components | Largest | Other |
|---|---|---|---|
| layer_0_piece_0 (Negative) | 6 | 16224 faces | 4 × ~248-face pin shells + 1 × 24-face sliver |
| layer_0_piece_1 (Positive) | 6 | 16094 faces | 4 × ~188-face socket shells + 1 × 24-face sliver |
| layer_1_piece_0 (Negative) | 6 | 16994 faces | same 4-pin pattern + sliver |
| layer_2_piece_0 (Negative) | 6 | 17906 faces | same 4-pin pattern + sliver |
| `cast_iter1_design.OLD/mold_layer_0_piece_0.stl` (pre-S5) | **2** | 15100 faces | **0 pin shells** + 24-face sliver |

The pin-shell AABBs match `PinSpec::iter1` exactly:
- Negative: 3.00 × 5.65 × 3.31 mm (pin radius 1.5 mm + post-trim binormal extent)
- Positive: 3.20 × 5.93 × 3.52 mm (socket radius 1.6 mm + axial clearance / 2)

Pin centers at `[±46, 1.25, ±49]` mm (Negative-side) and
`[±46, -3.37, ±49]` mm (Positive-side) — radially at the
midpoint of the cup-wall annulus along `±split_normal` (+X), at
arc-fractions 0.25 and 0.75 along the centerline (Z ≈ ±49 mm).

## Why this is bad for the workshop

When the workshop user prints a Negative cup with the seam face
down on the FDM build plate:

- The cup wall material's seam-face footprint sits on the build
  plate.
- Each pin's footprint at `(X = ±46, Z = ±49)` falls **outside**
  the cup wall footprint (the body's cross-section at that X is
  wider than the cup wall annulus reaches; cup material at that
  X exists only at Y values away from the centerline, but the
  pin sits near Y = 0).
- Slicer treats each pin as a separate island. The pin's bottom
  face is at the seam plane (Y = 0 of the trimmed cylinder),
  which IS on the build plate — so the pin's first layer adheres.
- Pin grows as an isolated ~5 mm column adjacent to the cup wall.
  No topological attachment.
- Workshop user pulls the print off the bed: cup + 4 detached
  pin columns. Must glue the pins onto the cup wall by hand at
  the matching footprint locations.

The Positive piece has matching socket shells (also disconnected) —
those carve nothing from the cup wall (no overlap), and would
print as small **cylindrical voids** in mid-air next to the cup,
which the slicer would skip entirely. **Workshop user gets a
Positive cup with NO sockets at all.**

The architectural fix's goal — "cross-piece fit becomes correct by
construction, not by tolerance" (plan §"Architectural fix") —
**reduces to glue-ups + manual socket drilling**. S5 + S6 + S7's
bit-equal CylinderParent invariant is correct in isolation but
operationally useless when the cylinders never overlap with the
host mesh.

## Hypothesized root cause

`registration.rs::build_registration_transforms` (`registration.rs:249-297`)
computes pin position as:

```
pin_center = centerline_sample(t) + pin_offset * split_normal_vec
pin_offset = midpoint(body_dist, bounding_dist)
```

where `body_dist` and `bounding_dist` are distances along the
`+split_normal` ray from the centerline sample to the body surface
and bounding-region surface, respectively. The pin sits at the
**radial midpoint of the cup-wall annulus** — measured at one
single Y slice (the centerline's Y position).

The pin axis is `binormal` (= `tangent × split_normal`), which
extends perpendicular to both the centerline tangent and the
split_normal. For a sock-over-capsule scan with centerline along
`-Z` and `split_normal = +X`, binormal = `-Y`. **Pin extends
±5 mm in Y.**

The cup wall material at the pin's `(X, Z)` position exists ONLY
at Y values OUTSIDE the body's cross-section at that `(X, Z)`. For
an oval body, the cup wall at `X = 46 mm` exists at `|Y| > ~12 mm`
(outside the body's lateral extent at that X). The pin's Y extent
`[-3.75, +6.25]` is entirely INSIDE the body's lateral range — so
the pin never overlaps cup material along its full 10 mm cylinder
length.

UnionCylinder (Negative side) adds a floating cylinder with no
merge; SubtractCylinder (Positive side) carves nothing because
there's no material to remove at the cylinder's location.

The pre-S5 SDF formulation didn't have this problem because
`Solid::union` works on SDF values, not topological adjacency —
the pin's SDF was added everywhere in space (including where there
was no cup material), and the resulting unioned SDF was meshed by
marching cubes through a grid that contained both the pin AND the
cup. Even if the pin extended "outside" the cup wall in some
regions, MC produced a single connected mesh because the SDF level
set is contiguous: the pin's "phantom" surface in empty space was
just not present (SDF < 0 outside the union), and MC produced one
shell per connected SDF region.

Post-S5, mesh-CSG works on actual mesh topology. Two meshes that
don't physically overlap stay as two meshes — manifold3d's
`Manifold::boolean_union` of two disjoint solids returns a
**Manifold with two components**. Same kernel-correct behavior;
just exposes the pin-placement bug that SDF arithmetic was hiding.

## Why the test gate didn't catch this

S4 ignored
`negative_piece_has_single_connected_component_with_iter1_pins_on_wide_body`
with the note (per
[[project-cf-cast-mating-features-s4-seam-plane]]) "post-S4
hollow-cuboid is 2 shells; needs parry3d probe." S5's rewrite
(per [[project-cf-cast-mating-features-s5-registration-pins]]
"Connected-component IGNORED test rewritten as transform-parameter
audit") replaced the connected-component check with a check that
the UnionCylinder transform parameters match the spec
(`pin_transforms_positive_socket_params_inflate_per_spec`).

The transform-parameter audit asserts that S5 emits the right
cylinder parameters. It does not assert that the resulting mesh is
one connected piece. **The original failure class — pin floats free
of cup mesh — was not preserved in the new test.**

This matches [[feedback-workaround-removal-verification]] exactly:
"when replacing a workaround ... with a 'cleaner' Result API,
enumerate the failure classes it caught + verify each is covered
by the new surface; if not, keep belt-and-suspenders."

## Diagnostic env-var-gated tool

A throwaway `mesh/mesh/tests/stl_shells_inspector.rs` was used and
deleted same session. Inspector loads an external STL via
`INSPECT_STL`, welds at 1 µm via `mesh_repair::weld_vertices`, runs
`find_connected_components`, and reports per-component face count +
AABB. Reproducer (re-add the file if needed):

```rust
// mesh/mesh/tests/stl_shells_inspector.rs
#[test]
fn inspect_external_stl() {
    let Some(raw) = std::env::var_os("INSPECT_STL") else { return; };
    let path = PathBuf::from(raw);
    let mut mesh = mesh_io::load_stl(&path).unwrap();
    mesh_repair::weld_vertices(&mut mesh, 1e-6);
    let analysis = mesh_repair::components::find_connected_components(&mesh);
    // print component count + per-component AABB
}
```

Run as `INSPECT_STL=<path> cargo test --release -p mesh --test
stl_shells_inspector -- --nocapture`. Either re-add this file
during recon-2 OR upgrade it into a kept-around inspector tool
(see Open question Q5 below).

## What this doesn't block

- **Companion-doc polish from S8 Phase A** (`14f57b1e` + `98e0a317`)
  is correct independent of this defect — clearance values surfaced
  in procedure.md are accurate even if pin attachment is broken.
- **Deferred items locked in plan §"Deferred items"** (FLANGE_AXIAL_STANDOFF_M
  + M6 platform-pocket) are orthogonal. Don't reopen those decisions.
- **S4 seam-plane migration** — main cup mesh is correctly bisected
  (component 0 is one piece, F4-clean except for known intrinsics).
- **S6 plug-shaft socket + T-slot + S7 pour-gate + funnel-nipple**
  migrations — those cylinders cut through the cup wall material
  (they originate at the cap-plane centroid and extend outward),
  so they always overlap and always merge. Empirically confirmed:
  no S6/S7 sites show as separate components in the shell analysis.
  Bug is registration-pin-specific.

## Open questions for recon-2

These are the load-bearing questions; the recon doc structures the
exploration.

**Q1.** Which candidate fix minimizes the chance of regressing other
mating features? Candidates:
- **(a) Pin axis along split_normal** (radial, perpendicular to
  centerline tangent + binormal): the pin would extend RADIALLY
  through the cup wall, always overlapping. But changes the
  seam-bisection geometry — the pin axis is now perpendicular
  to the seam plane, not parallel; SeamTrim would clip the pin
  through-axis rather than along-axis.
- **(b) Pin axis along centerline tangent** (along the body's
  long axis): pin extends along the centerline at the pin's
  radial location. Cup material exists along the centerline (it
  IS the cup wall). Pin would overlap consistently. Workshop
  insertion direction: along centerline tangent at that arc
  fraction — geometrically reasonable.
- **(c) Keep binormal axis; widen pin's radial extent** (raise
  `pin_radius_m` enough that the pin's radial sweep covers
  most of the cup-wall annulus, guaranteeing overlap at
  most Y values). Trades workshop-ergonomic pin diameter for
  topological guarantee. Probably awkward.
- **(d) Restore SDF-side pin union** alongside mesh-CSG, belt-
  and-suspenders. Pin SDF contributes via `Solid::union` AND
  mesh-CSG emits the cylinder. Loses the S5 bit-precise OD
  benefit (MC quantizes the SDF-side pin) but always-connected.
- **(e) Place pin closer to centerline** (radially inward) so the
  pin's binormal extent stays inside the cup wall material.
  Requires understanding how far inward is "in cup wall" at all
  Y values within the binormal sweep — geometry-dependent.

**Q2.** Does the fix need to be `RegistrationKind::Pins`-specific
or should the connected-component invariant be a generic mesh-CSG
output check that ALL mating features must pass? If generic, S6/S7
are off the hook empirically but the test gate should still apply
defensively.

**Q3.** Should the IGNORED-since-S4 connected-component test be
**restored** (not just supplemented) as the regression sentinel?
The "transform-parameter audit" rewrite was wrong for the failure
class it was meant to replace. Restoring the original test —
asserted against the iter-1 fixture — would have caught this in S5.

**Q4.** Iter-1 was physically printed (PR #254 / `358d682c`) and
the print "worked" enough to produce the mating-surface defects
that triggered this arc. How did the iter-1 print have attached
pins? Two hypotheses:
- (i) Pre-S5 SDF pins were genuinely attached (per the
  cast_iter1_design.OLD shell-inspection result), so the iter-1
  print used integrated pins. The mating-defect arc fixed
  protrusion/socket-depth precision but introduced this
  disconnection regression.
- (ii) iter-1 was printed with separate pin columns and the
  workshop user manually glued them — and the "longer than the
  socket is deep" defect was the WORKSHOP GLUE-UP mismatch,
  not an MC quantization issue.

Hypothesis (i) is supported by the OLD STL analysis (2 components
pre-S5). Hypothesis (ii) would require workshop-user testimony.
Recon-2 should pick which to assume.

**Q5.** Should the `stl_shells_inspector` get promoted from
throwaway scratch test to a kept-around diagnostic? It cost ~50 LOC
to write; running it on every STL set takes <1 s. Could live as a
`#[ignore]` integration test in cf-cast similar to `iter1_gate`.

**Q6.** Does the cup's 24-face sub-mm sliver near the pour-gate
end (component 5 in every cup piece, AABB ~0.04 × 1.0 × 0.34 mm,
center near `[-4.37, 4.93, -57.34]`) matter? Sub-mm closed shell
likely a SeamTrim sliver where the seam plane intersects a cylinder
at a near-tangent angle. Could be left alone (workshop-invisible at
0.04 mm), but recon-2 should explicitly disposition.

## What to read first (recon-2 cold-start)

In order:
1. This bookmark end-to-end.
2. `docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN.md`
   (the recon plan; reads what the recon must answer).
3. `design/cf-cast/src/registration.rs:249-297` —
   `build_registration_transforms` body.
4. `design/cf-cast/src/piece.rs` — how the registration transforms
   are appended to the cup-piece Vec.
5. `design/cf-cast/src/mesh_csg.rs` —
   `apply_mating_transforms` + `MatingTransform::{Union,Subtract}Cylinder`
   builder calls.
6. Memory entries (Cmd-F in `MEMORY.md` for these slugs):
   `[[project-cf-cast-mating-features-s5-registration-pins]]`
   (gives the IGNORED→rewritten test history);
   `[[feedback-workaround-removal-verification]]` (gives the
   load-bearing pattern this recon falsifies); `[[project-cf-cast-mating-features-s8-phase-a]]`
   (S8 close + finds).

## Status log

- **2026-05-22 — Bookmark written + recon-2 plan opened.**
  S8 Phase B physical print on hold; iter-2 STLs at
  `~/scans/cast_iter1/` are sound for everything *except* pin
  attachment.
