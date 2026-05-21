# Workshop iter-1 mold visual-gate — RECON

**Status:** recon complete 2026-05-21. Bookmark (prior session):
[[project-workshop-iter1-mold-unsatisfactory-bookmark]]. Implementation
next session.

**Scope:** defects (4) pour gate missing, (5a) registration pins
missing, (5b) free-floating positive sliver in `mold_layer_0_piece_0`,
**plus (6) plug-pin centerline-orientation bug** — surfaced during this
recon's cold-read pass, folded into scope because it's the same
root-cause class as (5a)+(5b) (iter-1 hardcoded default vs. derived
geometry) and an unfixed (6) means the workshop user gets a printed
plug with no usable mold anchor at the dome end. Defect (1) plug base
flare is a separate cf-scan-prep concern, out of scope.

## TL;DR

Two iter-1 hardcoded-default-vs-derived-geometry bugs in v2.1 cf-cast,
both folded into the same arc:

**Root cause for (5a) + (5b)**: `PinSpec::iter1().offset_from_centerline_m
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

**Root cause for (6) plug-pin centerline-orientation**:
`plug.rs:198-211` hardcodes `pin_at_start = anchor_cylinder(first.start,
-first.tangent, …)` with docstrings declaring `centerline[0]` to be
the pour-end. cf-scan-prep's `compute_centerline_polyline` emits
points in **min-projection → max-projection** order along the chord-
direction principal axis — for the iter-1 sock that's tip → base
(`centerline[0]` is the closed dome at z=+0.073, `centerline.last()`
is the cap-closed base at z=-0.054). So the "pour-end pin" is placed
at the dome, and the socket subtraction cylinder sits mostly INSIDE
the layer-0 body where the carve is a no-op. Only the ~1 mm of
cylinder extending past the body tip touches cup-wall material, and
3 mm MC cells can't resolve a 1 mm-deep feature. **Plug-anchor
feature is functionally broken for tip-first centerlines** — the
printed plug has no usable socket in any of the mold pieces.

Same root-cause class as (5a)+(5b): an iter-1 numeric/geometric
default that worked under a narrower assumption (test fixture: pour-
end at centerline[0], straight along +X) and silently breaks under
real cf-scan-prep output.

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

Order is **A → B → C → E → F → G**, with D firing only if B fails.
A is cheap and pins the bug; B is a 5-minute cf-view check that can
collapse D out of scope entirely; C and E are the two real fixes; F
and G land the regression coverage.

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
- **Hole absent**: D fires (see below).

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

### E. Fix plug-pin endpoint selection (~1-2 hr)

`plug.rs:193-244` (`build_plug_pin_solid` + `build_plug_socket_solid`)
currently treat `centerline[0]` as the pour-end unconditionally.
Replace with a **cap-plane-derived endpoint selection**: cf-scan-prep
already records the open-end plane in `prep.toml [caps].loops[i]`
(centroid + normal). Plumb the cap-plane centroid through
`derive.rs` → `Ribbon::with_plug_pins` or as a sibling argument, and
pick the centerline endpoint nearest the cap-plane centroid as the
"pour-end" anchor for `pin_at_start`.

Two candidates considered:

**E1 — cap-plane-derived endpoint (recommended)**: Per the sibling
bookmark §"Hypotheses for the fix" (1). Leverages data we already
record. Adapts automatically to any centerline orientation (PCA,
reversed, etc.).

**E2 — `cast.toml [plug_pins].pour_end = "start" | "end"` knob**:
Cheap escape hatch but pushes the orientation question to the
workshop user, who shouldn't have to reason about cf-scan-prep
internal axis conventions. Skipped per
[[feedback-strip-the-knob-when-default-works]].

**Surface area:** New optional `cap_plane_centroid: Option<Point3<f64>>`
parameter on `build_plug_pin_solid` + `build_plug_socket_solid`, OR a
new `Ribbon::with_pour_end_hint(Point3<f64>)` builder method. Prefer
the latter — keeps the registration / pour-gate / plug-pin builder
signatures uniform (`&Ribbon` only) and surfaces the pour-end hint as
explicit ribbon state for the workshop user to inspect.

The cap-plane centroid is already in `cf-scan-prep`'s `prep.toml`;
`cf-cast-cli` reads it (the pinned-floor arc already consumes it).
Plumbing should be minimal — `derive.rs:336-351` is the right place
to wire `Ribbon::with_pour_end_hint(cap_plane.centroid)` between the
ribbon construction and the `with_plug_pins` builder call.

**Fallback for no caps**: When `cap_planes.is_empty()` (e.g.,
hand-built closed scan), default to `centerline.last()` as the
pour-end. Rationale: cf-scan-prep's PCA-aligned auto-orientation puts
the body's natural "open end" at the END of the principal axis when
the scan was loaded with normals correctly. Bank the assumption;
revisit if a no-caps fixture surfaces a different convention.

### F. Regression tests — wide body + reversed centerline (~45 min)

Inside cf-cast lib:
- `mold_piece_negative_has_single_connected_component_with_pins_iter1()`:
  the spike from step A, generalized to assert N=1 component for the
  fixed iter1 PinSpec on the synthetic 60 mm cube.
- `pin_position_stays_in_cup_wall_for_wide_body()`: probes
  `build_registration_solid` directly on the 60 mm cube fixture and
  asserts the returned solid's AABB lies inside the cup-wall annulus
  (`x ∈ [body_half + wall_thickness, bounding_half - pin_radius]`).
- `plug_pin_anchors_to_pour_end_for_tip_first_centerline()`: synthetic
  fixture where `centerline[0]` is the dome (tip-first orientation,
  matching cf-scan-prep's iter-1 output). Pass a `cap_plane_centroid`
  that's nearer `centerline.last()`. Assert
  `build_plug_pin_solid(ribbon).bounds()` is positioned at
  `centerline.last()`, NOT `centerline[0]`.
- `plug_socket_carves_visible_mold_material_for_tip_first_centerline()`:
  the same fixture, but compose the layer-0 mold piece and assert the
  socket cylinder's intersection with the mold piece SDF is at least
  `cell_size * 2` deep (i.e., MC will resolve it).
- Optional `pour_gate_visible_in_positive_piece()`: assert at least one
  vertex in mold_layer_0_piece_1's mesh has y > -bounding_y + 1 mm
  AND lies inside the gate cylinder's cross-section. Documents the
  intentional one-sided placement.

Inside cf-cast-cli:
- `wide_body_fixture_has_clean_negative_piece_with_pins()` integration
  test using the existing fixture pattern.
- `reverse_centerline_fixture_plug_pin_anchors_at_cap_end()`:
  feed an integration fixture where the centerline runs tip → base
  AND `prep.toml [caps]` records the cap plane at the base end.
  Verify the cf-cast-cli output's plug STL has the pin extending from
  the base-end face, AND the mold pieces have a visible socket carve
  at the base-end face.

### G. Update workshop iter-1 cast.toml + procedure.md (~15 min)

After C + E land, re-run cf-cast-cli on
`~/scans/cast.iter1-design.toml`. Inspect all 12 STLs in cf-view
(assembly mode). Verify:

- (5a) Registration pins are visible protrusions on the +Y seam face
  of piece_0, matching holes on the -Y seam face of piece_1.
- (5b) No free-floating mesh components in piece_0 or piece_1.
- (4) Pour gate is visible as a 6 mm Ø hole on the -Y outer face of
  piece_1 (per the v2.1 sub-leaf 2 one-sided design).
- (6) Plug STLs have a single pin at the BASE end (z=-0.054). Mold
  piece_0 + piece_1 have matching socket carves at the base end's +Z
  face (assuming the cap plane is at the base).

This is the workshop iter-1 visual gate, re-fired post-fix.

If anything fails, that's the next session's recon — the fixes
themselves don't ship until G passes.

## Cold-read polish (in-arc, surface during implementation)

These survived the scope-fold from the sibling-bookmark into this arc.
Land alongside C and E in the same implementation session.

- `pour.rs:31-36` "each piece carves a half-cylinder cross-section"
  paragraph is stale post-v2.1 sub-leaf 2 (gate is now one-sided).
  Update to match the current placement OR add a "v2.1 sub-leaf 2:
  side-mounted" annotation similar to lines 7-23.
- `plug.rs:31-39` "pour-end pin is centered at centerline[0]" — the
  "pour-end" / "dome-end" vocabulary doesn't survive the cf-scan-prep
  centerline orientation convention; rewrite to "pour-end is the
  cap-plane-nearest centerline endpoint" and reference the E1 fix.
- `registration.rs:16-17` "pin sits in the cup wall (outside the body
  cavity, inside the bounding region)" — true under C1's new
  body-relative placement; reaffirm as the invariant.

## Coverage gap (drives F + G test plan)

### Registration pin

The v2.1 arc's integration tests at
`design/cf-cast/src/registration.rs:155-246` + `pour.rs:255-400` use a
**straight 100 mm centerline along +X with +Y split-normal**, fixture
body 20 mm half-extent in +X. With body half-extent 20 mm and pin at
offset 25 mm, the pin is at x = 25 mm which is 5 mm OUTSIDE the body —
in cup-wall territory for the fixture's 30 mm half-extent bounding
region. Test passes.

For iter-1's 36 mm body half-extent + bounding 61 mm, the pin at 25 mm
is INSIDE the body. The test geometry didn't surface this because the
relationship between fixture body size and pin offset wasn't ever
exercised in a "body bigger than offset" regime.

### Plug-pin centerline orientation

The cf-cast lib tests at `plug.rs:296-490` use a synthetic centerline
running `-50 mm → +50 mm along +X`. `centerline[0] = (-0.05, 0, 0)` is
"clearly the pour end" by intent of the test fixture, and
`first.tangent = +X`, `-first.tangent = -X` reaches at x=-60 mm into
empty space (no body geometry there in the test fixture). Tests pass.

Real cf-scan-prep centerlines don't have this convenient pour-end-at-zero
convention — they follow the scan's principal-axis projection
direction, which is set by the scan's PCA-detected long axis, not by
any pour-end vs. tip semantics.

### Shared pattern

Both gaps share a shape: a fixture-vs-real-output mismatch where the
fixture's numerical convenience happened to satisfy the implicit
invariant, hiding the load-bearing dependency on a property cf-scan-prep
doesn't guarantee. F + G's test fixtures must include geometry where
the convenience does NOT hold (wide-body for registration, tip-first
for plug-pin).

## Banked observations (patterns)

- The v2.1 arc's integration tests pass because the fixtures' numerical
  conveniences happened to satisfy load-bearing invariants the spec
  never wrote down. Pattern: **fixture sizing AND fixture axis
  conventions relative to parameter defaults matters**.
- This recon's two folded defects join the mold-wall arc's plug-pin
  cross-field failure (20 mm pin vs 5 mm wall, caught by config-parse
  validator) as THREE iter-1-numeric-default-vs-derived-geometry
  failures surfaced in the mold-wall arc's wake. Common root-cause
  class worth a feedback memo:
  [[feedback-default-numeric-vs-derived-geometry]] (to write
  post-implementation).
