# cf-cast registration-pin disconnection — recon-2

**Status:** decisions filled in, 2026-05-22 — recon-2 session COMPLETE.
Boxed Decision lines §R1–§R7 resolved. Implementation session is the
follow-up (size estimate in §R7).
**Predecessor:** [`docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_BOOKMARK.md`](./CF_CAST_REGISTRATION_PIN_DISCONNECTION_BOOKMARK.md).
**Scope:** per plan §G11 partial-pass branch — **registration pins
ONLY**. Do NOT reopen the mating-features architectural arc; S4 +
S6 + S7 are confirmed-clean by the shell-inspection. Mesh-CSG +
shared `CylinderParent` pattern stays.

## Why recon-2 (not an inline patch)

The bug surfaces a real architectural question that wasn't faced in
recon §2 / §5: **what does the mating-features pipeline guarantee
about topological connectivity** between the mesh-CSG primitive
mesh and the cup-piece mesh? Recon §2's load-bearing claim is
bit-equal cross-piece primitives. Connectivity-to-host-mesh was
implicit (the SDF formulation had it for free); the architectural
fix lost it. Picking a candidate fix without that question
answered risks shipping a fix that breaks again under iter-3 scan
geometry.

Plan §G11 partial-pass: "diagnose per failing feature ... single
recon-2 scoped to failing features only; do not re-open the
architectural arc." This is that recon-2.

Bookmark + recon + implementation = three sessions per
[[feedback-bookmark-when-surface-levers-exhaust]].

## What recon-2 ships

A single decision document (this doc, after the session) extending
recon §2 + §9 with:
- A new **connectivity invariant** that mesh-CSG mating features
  must satisfy.
- A picked candidate fix (one of (a)-(e) below or a recon-introduced
  variant), with a falsification gate.
- A connected-component sentinel test that the implementation
  session must ship + that gates every future mating-feature
  migration.
- A bounded list of follow-up cleanups (e.g., remove the failed
  `pin_transforms_*` transform-parameter audit if recon decides
  the connected-component test obsoletes it).

No production code edits in recon-2 itself. Implementation lands
in a follow-up session.

## Session-cadence (recon-2 only)

Single-session. ~3–4 hours active. No spike needed — manifold3d's
boolean behavior is already characterized; the question is
mathematical (pin geometry) not library-empirical.

If recon-2 surfaces a deeper architectural problem (e.g., binormal
axis is fundamentally incompatible with mesh-CSG attachment), it
escalates to its own recon-3 — but the bookmark's analysis suggests
the fix is local to `build_registration_transforms`.

## Concrete questions recon-2 must answer

Each question's resolution is a boxed Decision line at the end of
the recon-2 doc (per the recon §2 template).

### §R1 — Connectivity invariant

What invariant does every mesh-CSG mating feature need to satisfy?
Proposed: **"After `apply_mating_transforms`, the cup-piece mesh
has exactly one connected component plus any explicitly-allowed
sub-mm slivers (per Q6 in the bookmark)."**

Tighten to a concrete check the test gate can assert at iter-1 size:
- Run `find_connected_components` post-CSG.
- `analysis.component_count == 1` OR
- `analysis.component_count > 1` AND all extra components are
  below some `max_sliver_faces` threshold AND below a
  `max_sliver_aabb_mm` threshold.

Pick concrete thresholds (e.g., ≤ 50 faces / ≤ 1 mm³ AABB) that
the iter-1 fixture's known sliver (24 faces / sub-mm AABB) passes
under but a registration pin shell (~248 faces / 60 mm³ AABB)
fails.

> **Decision §R1.** **Connectivity invariant:** "After
> `apply_mating_transforms` runs on a cup-piece mesh, the result
> must have exactly one *workshop-meaningful* component — i.e., one
> component carries ≥ 99% of the total face count, and every other
> component is a SLIVER below both face-count and AABB-extent
> thresholds." Concrete thresholds picked off the empirical iter-1
> measurements:
>
> - `max_extra_components: 2` — per-cup-piece-mesh threshold.
>   Iter-1 has exactly 1 known sub-mm sliver per cup piece (24
>   faces, max-extent 1.012 mm); the +1 margin slot covers future
>   near-tangent SeamTrim artefacts at other ribbon/body
>   geometries. Workshop-print posture: any growth past 2 is a
>   regression to investigate, not a free-ride.
> - `max_sliver_faces: 50` — sliver has 24, pin shell has 248
>   (10× ratio). 50 is comfortably above the largest known sliver
>   and 5× below the smallest known disconnection-shell.
> - `max_sliver_max_extent_mm: 1.5` — sliver max extent 1.012 mm
>   (the Y extent); pin shell max extent 5.65 mm. 1.5 mm threshold
>   has 0.5 mm of margin above the known sliver and 4 mm of clear
>   air below the smallest known disconnection-shell, sitting well
>   inside the "workshop-invisible at 0.4 mm FDM bead width"
>   threshold the iter-1 prep posture treats as below-resolution.
>
> Math-verifiability (per
> [[feedback-math-verify-geometric-contracts]]): the invariant is a
> single boolean per cup-piece STL with thresholds picked off engine-
> precision measurements (face counts are integers; AABB extents are
> bit-precise at f64 precision). Test gate asserts at engine
> precision, not eyeball-and-judgment.
>
> The invariant is **per cup-piece mesh** (one assertion per
> Negative + Positive × per-layer output, so 6 assertions for iter-1
> + 8 for the eventual iter-3 4-layer set). The plug + funnel +
> platform STLs are out of scope — they're single-primitive
> objects (no Boolean union ever happens against them) and an
> ALL-mating-feature gate would over-constrain them.

### §R1 worked analysis (load-bearing for §R2, §R3)

The shell inspector run on iter-2 STLs (re-added during this recon
as `mesh/mesh/tests/stl_shells_inspector.rs`; see §R3) confirms the
bookmark's empirical claim verbatim:

| STL | components | largest | other components |
|---|---|---|---|
| iter-2 layer_0_piece_0 (Negative) | 6 | 16224 faces | 4 × 248-face pin shells + 1 × 24-face sliver |
| iter-2 layer_0_piece_1 (Positive) | 6 | 16094 faces | 4 × 188-face socket shells + 1 × 24-face sliver |
| iter-2 layer_1/2 piece_0/1 | 6 | 16994/17906 faces | same 4-pin pattern + sliver |
| OLD/mold_layer_0_piece_0 (pre-S5) | 2 | 15100 faces | 1 × 24-face sliver |

A targeted geometry probe (Y-direction vertex bucket scan inside a
5 mm × 5 mm cube around the pin's `(X=46.05, Z=49.27) mm` location
on layer_0_piece_0) shows cup-mesh vertices only at `Y ∈ [-1.6, +4.0]
mm` (the pin shell itself) and at `Y ∈ [+19, +28] mm` (the cup wall
material on the +Y side of the pin). **No cup vertices between
Y = +4 and Y = +19 mm** — the pin's post-trim binormal extent sits in
a region with no cup wall to merge against. (The probe was a
throwaway `iter1_body_probe.rs` test deleted at the close of this
recon; the inspector remains under §R3.)

This contradicts a "simple capsule" mental model of sock-over-capsule
(where one would expect cup material at all Y between `Y_body` and
`Y_bounding` regardless of X). The actual scan's body cross-section
at `(X = 46 mm, Z = 49 mm)` has Y extent reaching past ±19 mm — i.e.
the body is wider in Y than the pin's binormal sweep. The pin's
center IS at the cup-wall annulus midpoint along `+split_normal`
(by construction of `pin_offset = midpoint(body_dist, bounding_dist)`),
but the **binormal sweep is in body interior**, not the cup wall.
manifold3d's mesh-CSG union of two non-overlapping volumes returns
two disjoint components — same kernel-correct behavior the pre-S5
SDF/MC pipeline glossed over (a free-floating SDF surface inside an
empty MC region produces an isolated MC shell, not "smeared into the
cup mesh" as the bookmark's narrative implies; see §R5).

### §R2 — Pin axis choice (bookmark Q1)

Pick one of (a) split_normal axis, (b) tangent axis, (c) wider
radial, (d) belt-and-suspenders SDF, (e) closer-to-centerline,
or a recon-introduced variant. Each option's tradeoffs:

| Option | Connectivity guarantee | Geometry change | Test churn | Workshop ergonomics |
|---|---|---|---|---|
| (a) split_normal axis | Strong (pin radial, always in cup wall) | Pin perpendicular to seam; SeamTrim bisects through-axis (different cap geometry) | Pin extends only RADIALLY (1.5 mm radius × 10 mm long → workshop user inserts the pin perpendicular to seam) | Insertion direction same as workshop intent (perpendicular to seam) ✓ |
| (b) tangent axis | Strong (pin along centerline, always in cup wall) | Pin parallel to body's long axis; SeamTrim bisects radially | Pin extends ALONG centerline (10 mm) at radial midpoint — long protrusion along the centerline direction | Workshop user inserts in centerline-tangent direction — unusual |
| (c) wider radial | Weak (depends on body cross-section variation) | Inflate `pin_radius_m` enough to sweep the cup wall annulus at all binormal Y values | None (params change only) | Larger pin Ø may interfere with body cavity at narrow cross-sections |
| (d) belt-and-suspenders SDF | Strong (SDF always merges) | Keep S5 mesh-CSG primitive + restore pre-S5 SDF pin union | Recurse to MC quantization on OD (loses S5 main benefit) | Workshop fit no better than pre-S5 |
| (e) closer-to-centerline | Weak (depends on body shape) | Compute pin offset using a "shrunk" body distance + a "shrunk" bounding distance so the radial midpoint stays inside cup wall at all Y in pin sweep | High (offset derivation more complex) | Pin lands closer to body surface — may be too close for FDM bead clearance |

Plan default: **(a) split_normal axis** — strongest guarantee, no
tolerance-budget loss, no awkward workshop direction. Pin extends
1.5 mm × 10 mm radially through the cup wall, which is 5 mm thick;
the pin spans the FULL wall thickness with 5 mm of extra length
that protrudes out one side. Workshop user inserts each pin
perpendicular to the seam (matching `procedure.md`'s current "the
hole axis is perpendicular to the seam at the pin position" prose).

Recon-2 can deviate. The picked option's geometric implications:
- SeamTrim's interaction with the pin (does the pin axis stay
  inside or out of the seam plane).
- `RIBBON_PIECE_OVERLAP_M` (0.5 mm) effect on pin protrusion past
  the seam.
- Test churn: which existing tests measure
  pin-axis-specific properties.

> **Decision §R2.** **Picked: (a) pin axis = split_normal.** Pin
> extends radially through the cup-wall annulus along the same
> direction `pin_offset` is measured. By construction of
> `pin_offset = midpoint(body_dist, bounding_dist) along
> +split_normal`, the pin's axis line lies entirely inside the cup
> wall AT THE PIN'S `(Y_c, Z_c)` SLICE as long as `pin_half_length
> ≤ annulus_half_thickness`. Topological connectivity is
> mathematically guaranteed, not geometry-dependent on body
> cross-section shape — the load-bearing property the bookmark's
> empirical falsification surfaces.
>
> **SeamTrim interaction.** Seam plane normal is the ribbon's
> binormal (per `Ribbon::seam_plane_reference`). Pin axis is
> `split_normal`, which is perpendicular to the binormal. So the
> pin's long axis lies IN the seam plane (parallel to it), and
> SeamTrim bisects the pin LENGTHWISE (along the long axis) — each
> cup-piece keeps a HALF-CYLINDER half-disk profile (D-shape cross-
> section) protruding from / cut into its seam face. Engagement
> depth across the seam = `pin_radius_m` (1.5 mm at iter-1 default),
> not `pin_half_length_m`. This is the SAME bisection mode S6's
> T-bar already ships (see [[project-cf-cast-mating-features-s6-t-bar]]
> cold-read pass-2 finding #1) — recon §5's transform ordering
> (`UnionCylinder` then `SeamTrim`) survives unchanged.
>
> **Workshop ergonomic shift.** Current (binormal axis): pin
> half-length protrudes 5 mm out of the seam face axially, mating
> with a 5-mm-deep hole on the other piece — a long axial pin into
> a deep socket. Post-fix (split_normal axis): pin presents a half-
> cylinder ridge 1.5 mm tall × `2 × pin_half_length` mm long on the
> seam face, mating with a corresponding half-cylinder groove. The
> ridge constrains lateral shear in two axes (split_normal +
> tangent) but only protrudes 1.5 mm into the other piece — a
> shallow keyway, not a deep pin. Workshop assembly is identical
> (pieces slide together along binormal/seam-normal), but the
> engagement is a keyway, not a pin. procedure.md prose needs
> updating to match (§R7); the current procedure.md prose ("the
> hole axis is perpendicular to the seam at the pin position"
> from OLD's procedure quoted in §R5) was accurate for the
> binormal-axis baseline — it becomes geometrically wrong post-fix
> (axis is now parallel-to-seam, not perpendicular) but the
> user-facing assembly direction (slide pieces along seam-normal)
> is unchanged.
>
> **Thin-wall caveat.** Annulus half-thickness varies per layer:
> iter-1 layer 0 has 13 mm cup wall (6.5 mm half-thickness), layer 2
> has 5 mm (2.5 mm half-thickness). The iter-1 default
> `pin_half_length_m = 0.005` (5 mm) exceeds layer 2's half-
> thickness by 2× — the pin would protrude 2.5 mm into the body
> cavity on the inner side and 2.5 mm past the outer surface on the
> outer side. Implementation MUST reduce the default to fit the
> thinnest layer's wall (≤ 2 mm half-length leaves 0.5 mm margin
> against cavity protrusion). Workshop ridge length drops from 10
> mm to 4 mm — still ~3× the seam-overlap budget, ample for shear
> resistance.
>
> **Why not the alternatives.**
>
> - **(b) tangent axis.** Pin extends along centerline tangent;
>   topological connectivity is geometry-dependent (body shape
>   along tangent at the pin's lateral location). Bookmark's
>   empirical probe showed the iter-1 body has irregular cross-
>   section that violates simple-capsule assumptions — no robust
>   guarantee. Workshop direction (insertion along tangent) is also
>   unusual for printed molds. **Rejected: not mathematically
>   robust.**
> - **(c) wider radial.** To span the body's empirical Y extent
>   (±19 mm at iter-1's pin location), pin radius would need to be
>   ~25 mm — that's a slab, not a pin. **Rejected: functionally not
>   a pin.**
> - **(d) belt-and-suspenders SDF.** Restoring the pre-S5 SDF pin
>   union alongside mesh-CSG would lose S5's bit-precise OD benefit
>   (the entire architectural goal of S5: pin and socket of a pair
>   share a bit-equal `CylinderParent` for fit-by-construction).
>   **Rejected: architecturally regressive.**
> - **(e) closer to centerline.** Plan framing was backwards —
>   moving the pin CLOSER to centerline puts it CLOSER to the body
>   surface, where the body's lateral extent is typically WIDER,
>   not narrower. Moving the pin FARTHER (closer to bounding) would
>   help, but the engagement is now in the bounding-surface region
>   which is thinly-walled and FDM-printability-marginal. **Rejected:
>   geometry-dependent without a clean guarantee + bad FDM
>   posture.**
>
> No recon-introduced variant survived: (g) "lateral offset along
> binormal + axis = split_normal" and (h) "lateral offset along
> binormal + axis = binormal" both place the pin so far from the
> seam plane that it doesn't cross the seam — pin sits entirely on
> one piece's side. Workshop function breaks. **Rejected: no seam
> crossing → no piece engagement.**

### §R3 — Test gate restoration (bookmark Q3 + Q5)

Should the IGNORED-since-S4 `negative_piece_has_single_connected_component_with_iter1_pins_on_wide_body`
test be **restored verbatim**, **replaced with a stronger generic
connectivity check** (per §R1), or **both**? Counterpart Positive
piece — `positive_piece_has_single_connected_component_with_iter1_pins`
— probably should ship too.

Tooling: should the throwaway `mesh/mesh/tests/stl_shells_inspector.rs`
become a kept-around `#[ignore]`-by-default integration test in
cf-cast (parallel to `iter1_gate`)? Workshop-iter physical-print
sessions could run it before workshop print to catch regressions.

> **Decision §R3.** **Three test surfaces:**
>
> 1. **Restore the IGNORED-since-S4 connected-component test** as a
>    NON-`#[ignore]` cf-cast unit test, promoted to assert the §R1
>    invariant (not just `component_count == 1`). One assertion for
>    Negative-piece, one for Positive-piece. Wire it against the
>    iter-1 wide-body fixture
>    (`pin_transforms_position_stays_in_cup_wall_for_wide_body_iter1_regression`'s
>    cuboid fixture is too generous — the bug is sock-over-capsule-
>    specific; the test needs a body fixture whose Y-extent at the
>    pin location exceeds the pin's binormal sweep so the
>    pre-fix code fails the assertion and the post-fix code
>    passes). Failure-class enumeration per
>    [[feedback-workaround-removal-verification]]: this single test
>    covers (i) pin floats free of cup mesh (the empirical bug);
>    (ii) socket disappears from cup mesh (the Positive-piece mirror
>    of the same bug); (iii) any future mating-feature transform
>    that fails to overlap with cup material.
>
> 2. **Generic mesh-CSG connectivity gate** as an
>    `apply_mating_transforms` post-condition test: synthesize a
>    cup-piece fixture, populate it with one each of every
>    `MatingTransform` variant (Union/Subtract Cylinder + SeamTrim),
>    assert the §R1 invariant on the post-CSG mesh. Lives in
>    `mesh_csg.rs::tests`. Gates ALL future mating features against
>    the connectivity invariant, not just registration pins.
>
> 3. **Promote the inspector to a kept cf-cast integration test.**
>    Move `mesh/mesh/tests/stl_shells_inspector.rs` to
>    `design/cf-cast/tests/stl_shells_inspector.rs` (cf-cast tier),
>    rename to `iter_connectivity_inspector`, gate behind
>    `#[ignore]` so it doesn't run in CI by default but the workshop
>    user can fire it on any STL output set before printing. Same
>    INSPECT_STL env-var contract; same shell count + AABB report.
>    Sits alongside `iter1_gate` (the existing `#[ignore]` workshop-
>    fixture integration test). Cheap to keep around (~80 LOC); the
>    disconnection bug would not have been DIAGNOSED without it (S8
>    Phase A close + this recon both relied on it), so the
>    cost-vs-value ratio is high — workshop iter-3 will use the same
>    diagnostic surface to falsification-gate the print BEFORE
>    physically printing.
>
> The S5-shipped
> `pin_transforms_positive_socket_params_inflate_per_spec`
> parameter-audit test stays as-is — see §R4.
>
> **Failure-class coverage matrix** (per
> [[feedback-workaround-removal-verification]] — enumerate failures
> the workaround / IGNORED-test caught, verify the new surface
> covers each):
>
> | Failure class | Pre-S4 SDF inside-only | S5 transform-param audit | NEW §R1 connectivity gate |
> |---|---|---|---|
> | pin SDF outside body interior | ✓ (caught by `< 0` assertion) | ✗ | ✓ (caught: pin mesh disconnected from cup mesh) |
> | socket SDF inflated per spec | ✗ | ✓ | ✗ (covers only the mesh-output side; param math invisible) |
> | pin's mesh-CSG output disconnects from cup mesh | ✗ (didn't exist pre-S5) | ✗ (this is exactly the gap S5 introduced) | ✓ (the load-bearing new check) |
> | Sub-mm SeamTrim sliver appears (Q6) | ✗ | ✗ | ✓ (caught — sliver exceeds threshold) |
> | Future mating feature emits a transform that doesn't merge | ✗ | ✗ | ✓ (generic gate) |
>
> The S5 transform-param audit covers a DIFFERENT failure class
> (clearance arithmetic) than the §R1 gate (topological
> connectivity). Both must stay; neither obsoletes the other.

### §R4 — S5 transform-parameter audit disposition

The S5-shipped `pin_transforms_positive_socket_params_inflate_per_spec`
test asserts UnionCylinder + SubtractCylinder params match the spec
(diametral / axial clearance arithmetic). This test passes the
spec's clearance budget through to the transform parameters but is
**orthogonal** to the topological connectivity bug. Two options:
- **Keep** it as a complementary check (spec → transform params)
  and ADD the connectivity check (transforms → mesh) — gates
  different failure classes.
- **Remove** it as a now-obsolete check (the connectivity check
  is the real failure class; the parameter check was a
  band-aid).

Recon default: **keep** + add. Parameter audits are cheap and
provide parameter regression coverage if a future refactor breaks
the clearance arithmetic.

> **Decision §R4.** **Keep.** The `pin_transforms_positive_socket_params_inflate_per_spec`
> test covers a failure class (clearance arithmetic in
> `pin_transform_for_side`) that's orthogonal to the §R1
> connectivity gate (mesh topology). Both must survive; neither
> obsoletes the other (see §R3's failure-class coverage matrix).
> Implementation will need to rewrite the test's expected `axis`
> value (from binormal to split_normal per §R2) but the structural
> assertion — "shared center+axis, Positive radius inflated by
> diametral/2, Positive half_length inflated by axial/2" — survives
> the axis change unchanged.

### §R5 — Iter-1 workshop history (bookmark Q4)

Confirm whether pre-S5 iter-1 print had attached pins (hypothesis
(i) — supported by `cast_iter1_design.OLD/` having 2 components)
or floating glued-on pins (hypothesis (ii) — would require workshop
testimony). Decision: pick which hypothesis to assume so the recon
text doesn't equivocate.

Default: hypothesis (i) — pre-S5 SDF union produced integral pins;
the iter-1 mating-defect arc was about MC quantization, not
glue-up. The shell-inspection evidence is dispositive here.

> **Decision §R5.** **Confirmed hypothesis (i), but the mechanism
> is more nuanced than the bookmark's narrative suggests.** The
> OLD shell-inspection result (2 components, no pin shells) was
> the dispositive evidence; what the bookmark glossed over is
> *why* the SDF formulation produced an integral mesh given the
> same geometric mechanism that fails for S5 mesh-CSG.
>
> The OLD STLs are dated 2026-05-21 01:33 — BEFORE commit `4003b921`
> (workshop iter-1 mold visual-gate fix, May 21 10:04) and BEFORE
> `b336f351` (bilateral pin mirroring, May 21 12:58). At OLD's
> timestamp the registration code was at the PR #251 / cf-scan-prep
> arc baseline — pin offset was the LEGACY FIXED 25 mm (not the
> derived annulus-midpoint) AND only 2 pins (not 4) per piece-pair.
> The OLD procedure.md text confirms: "2 cylindrical pins (3.0 mm Ø
> × 10.0 mm long, printed integrally with `_piece_0` and matched by
> cylindrical holes in `_piece_1`)".
>
> So OLD's "2 components" doesn't prove pre-S5 SDF was robust —
> rather, OLD's geometry was DIFFERENT enough that the same
> geometric mechanism didn't trigger. With a fixed 25 mm offset
> (placing pins inside the body's X-extent ≈ 41 mm), the
> mold-recon-arc later showed those pins were "free-floating
> slivers" (see [[project-workshop-iter1-mold-recon]]) — the
> falsification that triggered the offset-derivation rewrite. The
> "no pin shells in OLD" empirical observation has at least two
> equally-plausible explanations (small-component filtering by MC
> + repair; or the (then-current) export pipeline dropping
> sub-threshold isolated meshes). Recon-2 does NOT need to decide
> between them to act on the current bug.
>
> The recon-2 decision treats hypothesis (i) as CONFIRMED for the
> immediate question — "did the iter-1 workshop user receive a
> mold with attached pins?" The OLD STLs they printed had no
> visible disconnection-shells (2 components, both workshop-fine).
> The pin-attachment mechanism in OLD is not load-bearing for
> recon-2's decision (since OLD's geometry differs from current
> in two non-trivial ways) and is **deferred as an open question
> for iter-3** if the workshop ever needs to revive the legacy
> offset path (unlikely; mold-recon-arc made the derivation the
> only supported mode).
>
> No glue-up was needed for iter-1 (hypothesis ii rejected).
> Workshop testimony from the iter-1 print would be helpful
> confirmation but is not blocking for recon-2's implementation
> decision.

### §R6 — Sub-mm sliver disposition (bookmark Q6)

The 24-face sub-mm sliver at `[-4.37, 4.93, -57.34]` (near the
pour-gate end) appears in every cup piece (both Negative and
Positive, pre- and post-S5). It's been around the whole arc;
nobody investigated. Likely a SeamTrim sliver where the seam plane
intersects a cylinder at near-tangent angle, or an MC artifact at
the bounding-region offset boundary.

Recon-2 either:
- **Leaves it alone** + adds a `max_sliver_faces` exception to
  the §R1 invariant.
- **Investigates root cause** with a targeted geometry probe at
  that AABB.

Default: **leave alone** + exception. Sub-mm + workshop-invisible
at FDM resolution.

> **Decision §R6.** **Leave alone + add explicit §R1 exception.**
> The sliver appears on every cup piece in both pre-S5 AND
> post-S5 STL sets at the same location (`[-4.37, 4.93, -57.34] mm`
> ± sub-mm wobble) with the same face count (24) — it's a
> long-standing artefact, not an S5 regression, and didn't cause
> any workshop iter-1 print issue.
>
> AABB extent `[0.041, 1.012, 0.335] mm` puts the dominant Y axis
> at 1 mm — exactly 2.5× the FDM bead width (0.4 mm), so it's
> roughly 2-3 layers tall on a printer; below the workshop's
> functional resolution (printed silicone mold features under 0.5
> mm don't survive demold per the workshop posture). The X and Z
> axes are 0.04 mm and 0.34 mm — sub-bead-width on X, ~1 bead Z.
>
> §R1 covers it via the `max_sliver_max_extent_mm: 1.5` threshold
> (sliver's 1.01 mm Y fits with 0.5 mm margin). Iter-3 could
> investigate root cause as a SEPARATE bookmark if a slicer-side
> issue surfaces; per [[feedback-bookmark-when-surface-levers-exhaust]]
> recon-2 doesn't expand scope to chase non-blocking artefacts.

### §R7 — Implementation-session scope estimate

Recon-2 closes with an estimate of the implementation-session
size: line counts, test churn, expected wall-clock. If the picked
candidate fix is (a) split_normal axis, implementation is local
to `build_registration_transforms` + test rewrite + procedure.md
verification (procedure.md prose currently says "hole axis is
perpendicular to the seam" which matches (a) exactly — no doc
update needed). Estimated **~100–150 LOC + 1 day** if (a). Other
options larger.

> **Decision §R7.** **Single implementation session, ~345 LOC, ~1
> day wall-clock.** Breakdown:
>
> | Component | LOC | Notes |
> |---|---:|---|
> | `pin_transform_for_side` axis: binormal → split_normal | ~10 | `build_registration_transforms` body change: pass `split_vec` not `binormal` as the axis (and rename the local accordingly). |
> | `PinSpec::iter1` default `pin_half_length_m`: 0.005 → 0.002 | ~5 | 2 mm half-length fits layer 2's 5 mm wall with 0.5 mm margin each side. Workshop ridge length drops from 10 mm to 4 mm. |
> | Cross-field validator (`pin_half_length_m ≤ wall_thickness_m / 2 - 0.0005`) | ~30 | Add to `CastSpec::validate` matching the existing `pin_length_m ≤ wall_thickness_m - 0.001` plug-pin pattern; fails fast if user's cast.toml has an over-long pin. |
> | Update existing pin tests for new axis (`pin_transforms_position_*`, `pin_transforms_positive_socket_params_inflate_per_spec`) | ~40 | Replace `axis: +Z` (binormal of the +X straight ribbon) with `axis: +Y` (split_normal). Replace `pin_half_length_m: 0.005` references where the spec default is asserted. |
> | NEW §R1 connectivity test on a body-shape that actually triggers the bug (Negative + Positive) | ~80 | Build cup-piece mesh via existing `mesh_piece_through_s4_pipeline`, then run §R1 invariant assertion. `pin_transforms_position_stays_in_cup_wall_for_wide_body_iter1_regression`'s cuboid fixture is INSUFFICIENT — its uniform cross-section means the pin-axis-line condition holds at every Y, so the disconnection doesn't trigger even with binormal axis. Use an **oblate ellipsoid** body (longer Y axis than X axis) where the body's Y-extent at the pin's `(X = annulus-midpoint, Z = ribbon-midpoint)` exceeds the pin's binormal sweep — this is the geometric signature that triggers the iter-1 bug. Pre-fix run on this fixture: assertion FAILS (6 components). Post-fix: PASSES (1 component + sliver-tolerance). |
> | NEW generic mesh-CSG connectivity gate (`mesh_csg.rs::tests`) | ~50 | Synthesize a cup-piece + apply one each of UnionCylinder + SubtractCylinder + SeamTrim transforms, assert §R1. |
> | Promote inspector from `mesh/mesh/tests/` to `design/cf-cast/tests/iter_connectivity_inspector.rs`, `#[ignore]`-gated | ~80 | Drop mesh-crate stub, add cf-cast `#[ignore]` integration test with same INSPECT_STL env-var contract. |
> | procedure.md prose: pin assembly direction + ridge-vs-pin semantics | ~30 | Cf [[project-cf-cast-mating-features-s8-phase-a]] / [[feedback-defensive-scope-cuts]]: the user-facing prose explains the new engagement model. (`procedure.rs::write_procedure_v2` registration section.) |
> | CURVE_FOLLOWING_DESIGN.md §Step 9 + §Risks doc-mirror | ~20 | Architectural change to the registration mechanism's geometry needs to land in the design doc per the S8 Phase A pattern. |
> | **TOTAL** | **~345** | One session, ~1 day, including cold-read pass-2. |
>
> **Session-cadence recommendation.** SINGLE implementation session.
> Decision-doc-then-implement cadence (recon-2 here, implementation
> next) — total ~3-session arc per [[feedback-bookmark-when-surface-levers-exhaust]]:
> 1. Bookmark (`90d1aaac`, 2026-05-22)
> 2. Recon-2 (this doc)
> 3. Implementation (next session)
>
> If the implementation session's cold-read pass-2 surfaces a fourth
> session's worth of follow-up (likely candidates: the connectivity
> gate flagging an S6/S7 surface I missed in §R3's "S6/S7 empirically
> clean" claim, or procedure.md prose drift across `procedure.rs`'s
> ~6 mating-feature sections all needing matching updates), bundle
> those as a separate polish commit on dev rather than expanding the
> implementation session.
>
> Workshop iter-2 print can resume after the implementation session
> ships. The implementation MUST close with a re-run of cf-cast-cli
> on `~/scans/cast.toml` and an inspector re-run on the regenerated
> STLs confirming `component_count ≤ 1 + max_extra_components` per
> §R1.

## Bail-out branches for recon-2

- **All five candidate fixes have unacceptable tradeoffs.**
  Escalate to recon-3 with the question "is mesh-CSG fundamentally
  incompatible with thin-cup-wall pin geometry?" Possible
  outcomes: pivot the registration mechanism to dovetails or
  magnets (per the v2 design-doc Open Question #1 alternatives,
  abandoned at v2 step 9 in favor of pins).
- **Connectivity invariant has too many slivers under iter-1.**
  Tighten the §R1 thresholds without crossing into "registration
  pins look like slivers" territory. If the workshop-invisible
  slivers are NOT cleanly separable from the workshop-visible
  pin shells by face count + AABB, the test gate becomes a
  judgment call rather than a math gate.

## Memory + cross-refs

- `docs/CF_CAST_MATING_FEATURES_PLAN.md` §G11 partial-pass branch —
  recon-2 is the canonical follow-up form.
- `docs/CF_CAST_MATING_FEATURES_RECON.md` §2 (shared-primitive
  invariant) + §5 (T-bar bisection) — recon-2 extends these with
  the new connectivity invariant.
- [[project-cf-cast-mating-features-s5-registration-pins]] — S5
  ship record + the IGNORED-test-rewrite that masked the bug.
- [[project-cf-cast-mating-features-s8-phase-a]] — diagnostic
  surface; cf-view smoke + shell inspector that caught the bug.
- [[feedback-workaround-removal-verification]] — the load-bearing
  pattern; recon-2 enforces it as a future-mating-feature gate
  rather than just a session-level reminder.
- [[feedback-math-verify-geometric-contracts]] — the connectivity
  invariant per §R1 IS a math-verifiable geometric contract;
  recon-2 promotes "topological connectedness" to that bar.
- [[project-cf-cast-registration-pin-disconnection-bookmark]] —
  predecessor bookmark with the diagnostic evidence + open
  questions that recon-2 resolves.

## Status log

- **2026-05-22 — Recon-2 plan drafted.** Session-cadence + question
  hooks + candidate-fix tradeoff table + bail-outs. Ready for the
  recon-2 session next.
- **2026-05-22 — Recon-2 session COMPLETE.** Decisions §R1–§R7 filled
  in. Picked candidate: **(a) pin axis = split_normal** with a
  shorter default `pin_half_length_m = 0.002` to fit the thinnest
  iter-1 layer wall. §R1 connectivity invariant ships as
  per-cup-piece check (≤ 2 extra components, ≤ 50 faces, ≤ 1.5 mm
  max extent for slivers). §R3 ships three test surfaces: restored
  connected-component test against a sock-over-capsule-like
  fixture, generic mesh-CSG gate in `mesh_csg.rs::tests`, and
  inspector promotion to `design/cf-cast/tests/` as a kept
  `#[ignore]` integration test. §R4 keeps the S5 parameter audit
  (different failure class). §R5 confirms hypothesis (i) for the
  immediate question with caveats around OLD's pre-mold-wall-arc
  configuration. §R6 leaves the sub-mm sliver alone. §R7
  implementation session estimated at ~345 LOC / 1 day; bookmark
  → recon-2 → implementation is the canonical three-session
  pattern.
  Diagnostic side-effect: `mesh/mesh/tests/stl_shells_inspector.rs`
  re-added during recon for empirical verification (and to keep
  around for the implementation session's pre/post comparison);
  §R3 promotes it to a kept cf-cast test. Implementation deletes
  the mesh-tier copy as part of the promotion.
