# Workshop iter-1 mold visual-gate — RECON

**Status:** recon complete 2026-05-21. Bookmark (prior session):
[[project-workshop-iter1-mold-unsatisfactory-bookmark]]. Implementation
next session.

**Scope:** defects (4) pour gate missing, (5a) registration pins
missing, (5b) free-floating positive sliver in `mold_layer_0_piece_0`.
Defect (1) plug base flare is a separate cf-scan-prep concern, out of
scope.

## TL;DR

**Single root cause** for (5a) + (5b): `PinSpec::iter1().offset_from_centerline_m
= 0.025` (25 mm) — fixed offset hardcoded since v2 Step 9 (2026-05-13),
calibrated against the OLD cuboid bounding region. After the mold-wall
arc (`54df41cb`, 2026-05-19), the bounding region is
`outermost_layer_body.offset(wall_thickness_m) = body + cumulative + wall`,
so the pin's "is the pin in the cup wall?" geometry now depends on
**body width at the pin's centerline z**, which the fixed 25 mm offset
ignores.

For sock-over-capsule (scan X half-extent ~36 mm) the pin at x=25 mm
lies INSIDE the layer-0 body at most centerline z values. Result:

- (5a) Pin protrusion lives in the body-cavity region (not on the
  cup-wall seam face). No useful registration.
- (5b) Pin geometry is geographically DISCONNECTED from the cup-wall
  mesh — MC produces TWO disjoint connected components in the
  Negative-piece STL: the cup wall AND a free-floating pin cylinder.
  Matches the user's "free-floating positive sliver" verbatim.

**(4) pour gate** is most likely a **piece-side inspection oversight**,
not a code defect. The v2.1 sub-leaf 2 design intentionally places the
gate cylinder ENTIRELY on the Positive piece's half-space (centerline
midpoint along binormal, length 90 mm out, doesn't straddle the seam).
Piece 0 (Negative) gets no gate by design. Piece 1 (Positive) should
have a 6 mm Ø × 20 mm radial hole through the -Y face. **Verification
gate**: cf-view `mold_layer_0_piece_1.stl` before any code change.

## Evidence trail

### Construction pipeline (confirmed correct upstream of compose_piece_solid)

`tools/cf-cast-cli/src/derive.rs:336-351` translates `[registration_pins]
enabled = true` / `[pour_gate] enabled = true` / `[plug_pins] enabled = true`
into `Ribbon::with_registration(Pins(iter1()))` /
`with_pour_gate(Default(iter1()))` / `with_plug_pins(Axial(iter1()_with_override))`.
The Ribbon construction is byte-for-byte the same as the v2.1 arc
integration tests that pass — H1 (stale-clone) **FALSIFIED**.

`design/cf-cast/src/piece.rs:114-148` composes per-piece geometry:
- Registration: Negative side `base.union(pins)` (protrusion),
  Positive side `base.subtract(pins)` (hole).
- Pour gate: BOTH sides `piece.subtract(channels)`.
- Plug socket: BOTH sides `piece.subtract(sockets)`.

CSG signs match the design-doc convention — H2 (sign flip) **FALSIFIED**.

### Bounding region — the changed assumption

`derive.rs:297-304`:
```rust
let bounding_region = pinned_floor_shell(
    closed_sdf_arc, open_sdf_arc, sdf_bounds, &cap_tuples,
    cumulative_thickness - cavity_inset_m,
).offset(config.cast.wall_thickness_m);
```

For iter-1 sock: `cumulative_thickness = 4 × 5 mm = 20 mm`, `wall_thickness
= 5 mm`. So the bounding region is the **scan surface offset outward by
25 mm**. Scan X half-extent is ~36 mm (per `sock_over_capsule.prep.toml
[output.aabb_m]`), so bounding extends to ~61 mm in +X.

Compare to layer 0's body (scan + 5 mm): X half-extent ~41 mm in +X.
Cup-wall annulus at layer 0 in +X direction: x ∈ [41 mm, 61 mm].

### The pin position — the broken assumption

`design/cf-cast/src/registration.rs:126-152`:
```rust
let split_vec = ribbon.split_normal.as_vector();     // = +X
for &t in &spec.arc_fractions {                      // t ∈ {0.25, 0.75}
    let (center, _tangent, binormal) = ribbon.sample_at_arc_fraction(t)?;
    let pin_center = center + spec.offset_from_centerline_m * split_vec;
    //              = (≈0, 0, z_at_t) + 0.025 * (1, 0, 0)
    //              = (0.025, 0, z_at_t)
```

For iter-1 sock-over-capsule, the centerline x/y values are ~1-4 mm
(near origin in XY). So pin center is at **x ≈ 25 mm** in world frame.

Compare to cup-wall annulus in +X: [41 mm, 61 mm].

**Pin at x=25 mm lies INSIDE the layer-0 body** (x < 41 mm). It's not
in the cup wall.

### The CSG → STL chain that produces the floating sliver

For the Negative piece at `(25 mm, y, z_at_t)` near the pin axis:
- `bounding_sdf` < 0 (inside the +61 mm-wide bounding region)
- `layer_body_sdf` < 0 (inside the +41 mm-wide layer 0 body)
- `-layer_body_sdf` > 0 → dominates → `base_piece_sdf > 0` → OUTSIDE the
  mold piece.
- `pin_sdf` < 0 (inside the pin cylinder).
- Union: `min(base_piece_sdf, pin_sdf) = pin_sdf < 0` → INSIDE the
  unioned solid.

So the union creates a positive-geometry region at x=25 mm where the
cup-wall mesh isn't (the cup wall is at x ∈ [41, 61]). MC on the
Negative piece walks a grid bounded by `base_piece.bounds() ∪
pin_solid.bounds()` (cf-design `union` extends AABB to cover both
operands), so the grid DOES reach to x=25 mm and meshes the pin.

The pin cylinder is small (3 mm Ø × 10 mm long, half-length 5 mm along
binormal -Y), free-floating ~16 mm inside the cup wall's inner edge.

User describes "a small isolated positive-geometry shard sitting inside
the mold cavity" — matches exactly. The "looks like an orphaned
plug-pin socket" suspicion is shape-based (a small cylinder), but it's
NOT a socket — it's a registration pin in the wrong place.

### Pin at t=0.75 — why the user saw only ONE sliver

Per the iter-1 prep.toml centerline, z values:
- t=0.25 → z ≈ +0.039 (closer to dome, body narrower there)
- t=0.75 → z ≈ -0.022 (closer to base, body still ~25-30 mm in +X)

**Both** pin positions likely produce a sliver — the user just saw one
(or saw both as a single visual cluster). Implementation regression
test must check connected-components count, not just "any sliver
exists".

## Working hypothesis (single root cause confirmed for 5a/5b)

`PinSpec::iter1().offset_from_centerline_m = 0.025` is a fixed numeric
default that worked when the bounding region was a cuboid envelope
~80 mm half-extent (so a pin at 25 mm was reliably inside the cup wall
regardless of body size). After the mold-wall arc, the bounding region
is body-relative (`body + cumulative + wall`), so the "is the pin in
the cup wall" check depends on body radius along split-normal at the
pin's centerline z — **a quantity the spec never reads**.

H3 (transformed-frame mismatch) **PARTIALLY CONFIRMED** in a related
sense: the spec uses world-frame `+X * 25 mm` for placement, but the
correct frame would be **layer-body-surface-relative** along split-normal.

## (4) Pour gate — verification gate first, code dive only if needed

`pour.rs:189-216` builds the gate cylinder at `midpoint + binormal *
half_length`, axis `binormal`, half_length `0.045 m`. The cylinder
spans from the centerline midpoint outward along binormal by 90 mm —
**entirely on one side of the seam** (does not straddle the cutting
plane).

For iter-1 (split_normal +X, tangent -Z → binormal -Y), the cylinder
lives entirely at y ≤ 0. The cup-wall in -Y at z=midpoint: y ∈
[-57 mm, -37 mm] (20 mm thick wall, layer 0 inner at -37, bounding at
-57). The gate cylinder carves a 6 mm Ø × 20 mm-long hole through that
wall on the Positive piece. **Negative piece by design has no gate.**

The module docstring at `pour.rs:31-36` says "each piece carves a
half-cylinder cross-section that combines into the full channel when
the pieces close" — that's STALE DOC from the pre-v2.1 placement.
v2.1 sub-leaf 2 moved the gate off-seam (inner tip at centerline,
extends outward) so it's intentionally one-sided. The stale paragraph
should be fixed during implementation; not a code bug.

**Verification step**: open `mold_layer_0_piece_1.stl` in cf-view, look
at the -Y outer face for a 6 mm Ø hole. If present, (4) is doc-rot
only. If absent, dig into `compose_piece_solid` for piece_1 path.

## Sub-leaf ladder for implementation session

### A. Recon-iter-2 synthetic spike (~15 min, cheap)

Build a minimal cf-cast lib test that reproduces the floating-sliver
phenomenon with synthetic geometry:
- 60 mm cube body (X half-extent 30 mm) — wide enough that the iter1
  pin at 25 mm sits inside it.
- 1 layer × 5 mm thickness, 5 mm wall.
- Registration pins enabled with `PinSpec::iter1()`.
- Compose the Negative piece, mesh it, count connected components.
- Assert: `components == 1` (currently fails: produces 2 — cup wall +
  floating pin). Use the existing mesh-printability or
  `mesh_simplification` utilities for component-counting; otherwise
  flood-fill triangles via shared-edge BFS.

This pins the bug at the cf-cast lib level. Also verifies the fix
later.

### B. Verify pour-gate piece 1 (~5 min)

Before any code change, open `mold_layer_0_piece_1.stl` in cf-view and
confirm the -Y face has a 6 mm Ø hole. Two outcomes:

- **Hole present**: (4) is a documentation issue. Fix the stale
  "half-cylinder" paragraph in `pour.rs:31-36` to match v2.1 sub-leaf 2
  reality, and add a procedure.md update telling workshop users to look
  at the Positive piece for the gate.
- **Hole absent**: dig into `compose_piece_solid` for piece_1 path.
  Likely candidates: `bounding_region.bounds()` doesn't extend far
  enough in -Y; or MC grid is misaligned with cylinder; or some
  per-layer issue. Spike a 60mm cube + pour gate similar to step A.

### C. Fix `offset_from_centerline_m` (~1-2 hr)

Two design candidates. **Recommend C1**.

**C1 — Body-relative placement (default-only)**: Change
`build_registration_solid` to derive the pin center from the layer
body's surface along the split-normal direction at the pin's
centerline z. Pseudocode:

```rust
let (center, _tangent, binormal) = ribbon.sample_at_arc_fraction(t)?;
let pin_offset = body_half_extent_along_split_at_z(layer_body, center, split_vec)
    + wall_thickness_m / 2.0;
let pin_center = center + pin_offset * split_vec;
```

Requires plumbing `layer_body` + `wall_thickness_m` into
`build_registration_solid` (currently only takes `&Ribbon`). Cleanest
implementation: pass `&CastLayer` and the bounding-region instead of
relying on Ribbon-only state. Alternatively, evaluate the layer body's
SDF along the split-normal ray to find its surface and use that as the
anchor — pure SDF math, no AABB query needed.

**Surface area:** new parameter on `build_registration_solid`. Internal
function, no public API churn.

**C2 — Surface a per-cast knob (slower escape hatch)**: Make
`pin_offset_from_centerline_m` a per-cast TOML knob in
`config.registration_pins` with a cross-field validator analogous to
the existing `wall_thickness_m + 1 mm ≥ pin_length_m` gate. Burdens
the workshop user; per [[feedback-strip-the-knob-when-default-works]]
this is the wrong default — pick C1.

### D. (Conditional on B failure) Fix pour-gate placement

If verification step B shows the gate is genuinely missing in piece 1,
candidate fixes:
- Move the gate cylinder to STRADDLE the seam (center at centerline
  midpoint, axis binormal, length 2 * half_length straddling y=0). This
  reverts v2.1 sub-leaf 2; need to re-justify the off-seam design.
- Increase `gate_half_length` or shift the cylinder further outward so
  MC reliably resolves a clean through-hole.

### E. Regression tests (~30 min)

Inside cf-cast lib:
- `mold_piece_negative_has_single_connected_component_with_pins_iter1()`:
  the spike from step A, generalized to assert N=1 component for the
  fixed iter1 PinSpec on the synthetic 60 mm cube.
- `pin_position_stays_in_cup_wall_for_wide_body()`: probes
  `build_registration_solid` directly on the 60 mm cube fixture and
  asserts the returned solid's AABB lies inside the cup-wall annulus
  (`x ∈ [body_half + wall_thickness, bounding_half - pin_radius]`).
- Optional `pour_gate_visible_in_positive_piece()`: assert at least one
  vertex in mold_layer_0_piece_1's mesh has y > -bounding_y + 1 mm
  AND lies inside the gate cylinder's cross-section. Documents the
  intentional one-sided placement.

Inside cf-cast-cli:
- Add a `wide_body_fixture_has_clean_negative_piece_with_pins()`
  integration test using the existing fixture pattern.

## Cold-read polish opportunities (out of scope, surface during implementation)

- `pour.rs:31-36` "each piece carves a half-cylinder cross-section"
  paragraph is stale post-v2.1 sub-leaf 2 (gate is now one-sided).
  Update to match the current placement OR add a "v2.1 sub-leaf 2:
  side-mounted" annotation similar to lines 7-23.
- `plug.rs:31-39` "pour-end pin is centered at centerline[0]" — for
  the iter-1 sock the cf-scan-prep centerline runs **tip → base**
  (centerline[0] is the closed dome at z=+0.073, centerline.last() is
  the cap-closed base at z=-0.054). The "pour-end" / "dome-end"
  vocabulary doesn't survive the cf-scan-prep centerline orientation
  convention. Either re-derive the pour-end from cap-plane data in
  `prep.toml`, or rename to "start-end" / "end-end" and let the workshop
  user pick `include_dome_pin` to mean "pin at the OTHER end".
  Functional implication: for iter-1 the socket subtraction cylinder is
  mostly INSIDE the layer-0 body (z < tip), so the carve is mostly a
  no-op — only the ~1 mm of cylinder extending past the body tip
  actually carves cup-wall material, and 3 mm MC cells can't resolve a
  1 mm-deep feature. **Plug-anchor pin is functionally broken for
  tip-first centerlines.** Bank as a sibling bookmark — it's distinct
  from the bookmark's three defects but a load-bearing v2.1 feature
  for workshop iter-1 assembly.

## Coverage gap

The v2.1 arc's integration tests at `design/cf-cast/src/registration.rs:155-246`
+ `pour.rs:255-400` use a **straight 100 mm centerline along +X with
+Y split-normal**, fixture body 20 mm half-extent in +X. With body
half-extent 20 mm and pin at offset 25 mm, the pin is at x = 25 mm
which is 5 mm OUTSIDE the body — in cup-wall territory for the
fixture's 30 mm half-extent bounding region. Test passes.

For iter-1's 36 mm body half-extent + bounding 61 mm, the pin at 25 mm
is INSIDE the body. The test geometry didn't surface this because the
relationship between fixture body size and pin offset wasn't ever
exercised in a "body bigger than offset" regime.

**Test plan addition for implementation session**: include the
"wide-body fixture" (≥ 60 mm cube body) in cf-cast lib tests, not just
the 20 mm narrow fixture. Same pattern as the mold-wall arc's added
`cube_fixture_mold_pieces_follow_contour_not_cuboid` regression.

## Banked observations

- The v2.1 arc's integration tests pass on iter-1 because the test
  fixtures use a 20-mm-half-extent body — narrower than the 25 mm pin
  offset. Iter-1's sock-over-capsule (36 mm) crosses the threshold.
  Pattern: **fixture sizing relative to parameter defaults matters**.
- This is the SECOND v2.1 default-vs-real-geometry failure surfaced in
  the mold-wall arc's wake (first was the 20 mm plug-pin vs 5 mm wall,
  caught by the cross-field validator at config-parse time). Both have
  the same shape: an iter-1 numeric default calibrated against an
  older bounding-region regime, not adapted to the new contour-follow
  geometry. Pattern worth banking:
  [[feedback-default-numeric-vs-derived-geometry]] (to write).
- The plug-pin centerline-orientation bug (cold-read item) is a THIRD
  iter-1-default-vs-real-geometry case. Bookmark it as a sibling
  defect — same root cause class, separate implementation arc.
