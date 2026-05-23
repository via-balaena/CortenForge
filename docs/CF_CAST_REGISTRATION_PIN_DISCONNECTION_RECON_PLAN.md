# cf-cast registration-pin disconnection — recon-2 plan

**Status:** draft, 2026-05-22 — recon-2 session not yet run.
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

> **Decision §R1.** Pick invariant. Pick sliver thresholds.

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

> **Decision §R2.** Pick axis. Document SeamTrim interaction.

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

> **Decision §R3.** Pick test restoration scope. Pick inspector
> promotion or not.

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

> **Decision §R4.** Keep / remove / merge the existing
> parameter-audit test.

### §R5 — Iter-1 workshop history (bookmark Q4)

Confirm whether pre-S5 iter-1 print had attached pins (hypothesis
(i) — supported by `cast_iter1_design.OLD/` having 2 components)
or floating glued-on pins (hypothesis (ii) — would require workshop
testimony). Decision: pick which hypothesis to assume so the recon
text doesn't equivocate.

Default: hypothesis (i) — pre-S5 SDF union produced integral pins;
the iter-1 mating-defect arc was about MC quantization, not
glue-up. The shell-inspection evidence is dispositive here.

> **Decision §R5.** Confirm hypothesis (i).

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

> **Decision §R6.** Sliver disposition.

### §R7 — Implementation-session scope estimate

Recon-2 closes with an estimate of the implementation-session
size: line counts, test churn, expected wall-clock. If the picked
candidate fix is (a) split_normal axis, implementation is local
to `build_registration_transforms` + test rewrite + procedure.md
verification (procedure.md prose currently says "hole axis is
perpendicular to the seam" which matches (a) exactly — no doc
update needed). Estimated **~100–150 LOC + 1 day** if (a). Other
options larger.

> **Decision §R7.** Implementation-session size estimate +
> session-cadence recommendation.

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

## Status log

- **2026-05-22 — Recon-2 plan drafted.** Session-cadence + question
  hooks + candidate-fix tradeoff table + bail-outs. Ready for the
  recon-2 session next.
