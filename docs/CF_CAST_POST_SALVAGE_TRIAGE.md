# cf-cast post-salvage triage (2026-05-24 night)

**Status:** workshop iter-3 print UNBLOCKED on pin/lock GEOMETRY
per the 2026-05-24 night salvage (commits `b2ff45d5` + `fed4b0c6`
+ `a8e3e056` on dev). Workshop user has cf-view-verified the
cup-pin trapezoidal ridges + base-down captive plug-lock pyramid.

This doc triages the 4 follow-up findings the workshop user
flagged in the same cf-view session, scoped per finding, with a
recon-prep sketch for the cap-plane flatness fix that the next
implementation session needs.

The intent is: read this doc fresh in the morning, decide which
findings are workshop blockers vs ergonomic, scope a recon arc
for blockers. **No implementation tonight** — slow careful
sequencing per the
`[[feedback-read-prior-arc-memory-before-architectural-decisions]]`
lesson saved this session.

## Methodology

Per finding:
- **Symptom**: what the workshop user observed in cf-view (with
  screenshot reference where applicable).
- **Root cause**: traced through the cf-cast pipeline.
- **Blocker vs ergonomic**: workshop-user judgment call. Triaged
  per the §G-11 quad-gate: would this prevent the print from
  WORKING (functional blocker) or just make it less PLEASANT to
  use (workshop ergonomic).
- **Recon scope estimate**: rough LOC + session count if the
  finding turns out to be a blocker that needs fixing before
  workshop iter-3 print.

## Finding A — Cup-piece floor face faceted

**Symptom**: the cup-piece's interior wall at the cap-plane (the
"cup-floor" where the plug-lock pyramid seats) is not perfectly
flat. Visible MC quantization facets at the 3 mm grid scale.
Workshop user's read: "we actually had this down and solved in
the main branch" — PR #255 era had this flat.

**Root cause**: the plug body comes from a scan mesh
(`sock_over_capsule.cleaned.stl`) which has a flat cap-plane face
per cf-scan-prep preprocessing. The cf-cast pipeline converts the
scan mesh → SDF → samples the SDF at 3 mm MC grid points →
re-meshes via marching-cubes. Even though the SDF is
mathematically flat at the cap-plane, MC's triangulation produces
visible facets because the 3 mm grid samples don't land exactly
on the cap-plane plane (no MC sample point is guaranteed to be
ON the SDF zero-crossing). The body cavity floor in the cup-
piece IS the plug body's cap-plane face (by construction:
`cup-piece = bounding ∖ body ∩ halfspace`), so the faceting
propagates to the cup-floor.

**Why PR #255 didn't have this**: investigation needed.
Hypothesis: pre-arc the plug body was meshed at a different
resolution or via a different pipeline that preserved the flat
face. Need to git-bisect or read pre-arc cf-cast mesher code to
confirm.

**Blocker vs ergonomic**: probably between. Sub-mm facets on the
cup-floor mean the plug doesn't seat perfectly flat against it
→ small wobble (~0.5-1 mm) during silicone pour. With the
captive plug-lock pyramid trapped by the closed cup halves, the
plug stays positionally locked even with floor wobble — but the
wobble could allow silicone to leak between cup-floor and plug
bottom face during pour. Workshop user has the better intuition
on whether this is acceptable for the first iter-3 cast.

**Recon scope estimate**: 1 recon session + 1 impl session for
cap-plane flatness. Likely shared with finding B (same root
cause). See the recon-prep section below.

## Finding B — Plug bottom face faceted

**Symptom**: same as A but on the OPPOSITE side of the cap-plane.
The plug's exterior bottom face (the cap-plane face of the plug
body) is faceted at the 3 mm grid scale. Workshop user
screenshot at session timestamp 22:17.

**Root cause**: SAME as A — plug body SDF sampled at 3 mm MC
cells. The plug body's bottom face is the plug-side view of the
same cap-plane plane that the cup-floor sees from the cup side.

**Blocker vs ergonomic**: same as A. The plug bottom face seats
against the cup-floor; combined facets from both = larger
mating-gap wobble.

**Recon scope**: SHARED with A. One fix addresses both.

## Finding C — Cup-side socket cavity mouth has partial obstruction

**Symptom**: the plug-lock socket cavity carved into the cup-
floor has irregular geometry at its mouth (where the cavity opens
at the cap-plane plane). Workshop user screenshot at session
timestamp 22:12: "matter obstructing the opening" on piece 0.

**Root cause**: hypothesis-1 (most likely): combination of (a)
MC quantization on the cup-floor face per A, and (b) mesh-CSG
boolean subtract of the inflated-pyramid SubtractTruncatedPyramid
leaves boundary artifacts where the pyramid's cap-plane cross-
section intersects the (already faceted) cup-floor face. The
boolean junction at the cavity mouth inherits the cap-plane
faceting + adds its own artifacts.

Hypothesis-2 (less likely): the cavity carve is geometrically
correct but appears obstructed in cf-view due to viewing-angle /
rendering artifacts. Would need a different viewing angle or
mesh-inspector probe to confirm. Less likely because the
hypothesis-1 root cause is well-understood.

**Blocker vs ergonomic**: if the obstruction is small (< 0.5 mm
of mesh artifacts at the cavity mouth), the plug pyramid can
still seat (the cavity widens past the mouth into the wider
interior). If larger, workshop user would need to manually file
or drill the socket mouth before assembly to allow plug
insertion. Borderline.

**Recon scope**: probably resolves with the same cap-plane
flatness fix as A+B (cleaning up the cap-plane face removes the
boundary artifacts at the cavity mouth). If it doesn't, separate
sub-fix for the mesh-CSG boolean junction (possibly via a
controlled epsilon-offset on the subtract pyramid extents to
avoid the coincident-face issue at the cap-plane).

## Finding D — Cup-piece seam face non-flatness at dome + cap regions

**Symptom**: workshop user screenshot at session timestamp 22:21
shows the cup-piece viewed edge-on with the seam face along the
right edge. Mid-section of the seam face looks reasonably flat
(per recon-4 (P) §F-4 bit-precise SDF→MC interpolation). Top
edge (where seam plane meets the dome curve of the cup body) and
bottom edge (where seam plane meets the cap-plane) show visible
non-flatness / triangulation. Workshop user: "we actually had
this down and solved in the main branch" — PR #255 era had this
fully flat.

**Root cause**: TWO sub-parts with separate root causes:

  - **D1 — cap-plane edge**: SAME root cause as A+B. The seam
    plane's intersection with the cap-plane is a straight line
    on the seam face's bottom edge. MC quantization on the cap-
    plane face propagates to this edge.
  - **D2 — dome edge**: the seam plane's intersection with the
    cup body's curved dome surface is a CURVE on the seam face's
    top edge. MC quantization at 3 mm cells approximates this
    curve as a faceted polyline. Different root cause from A+B
    — this is curved-body × seam-plane MC quantization, not
    flat-face MC quantization.

**Why PR #255 had this fully flat**: recon-4 (P) §F-4 proved the
seam face was bit-precisely flat for the MAIN seam-face area via
SDF halfspace intersect + MC linear interpolation. The EDGES of
the seam face (cap-plane edge + dome edge) were also reasonably
clean because the pre-arc mesh-CSG cylinder pins had circumferential
curved surfaces that meshed cleanly. **Post-salvage** the cup-pin
pyramids have sharper lateral corners which produce sharper
(worse) boolean-junction artifacts where they intersect the seam
plane — possibly contributing to the dome-edge visible
quantization. Hypothesis-2 for D2: it's the cup-pin pyramid
boolean junctions, not the curved-body × seam-plane intersection
per se. Would need to test by regen-with-no-pins to isolate.

**Blocker vs ergonomic**: slight non-flatness on the seam face
means cup halves don't seat perfectly flush → small seam gaps
where silicone might leak during pour. Workshop ergonomic at
small magnitudes (< 0.3 mm), blocker at larger (rubber-band
clamping plus mold-release-putty fillet on outside of seam can
absorb small gaps).

**Recon scope**:
  - D1 SHARES with the cap-plane recon (A+B).
  - D2 needs ITS OWN recon if it turns out to be a blocker. Likely
    options: (a) finer MC cell size in the dome region only
    (adaptive mesh — NOT supported by current pipeline); (b)
    revert cup-pin to mesh-CSG cylinder (loses §G-2 dovetail
    shape per recon-1; trade-off); (c) accept the dome-edge non-
    flatness as ergonomic. **Triage D2 separately after the
    cap-plane fix lands — possibly the cap-plane fix is enough
    workshop ergonomic improvement to render D2 acceptable.**

## Cap-plane flatness — recon-prep sketch

The shared root cause for findings A + B + C + D1: plug body SDF
sampled at 3 mm MC cells produces visible facets on the cap-plane
face. The fix needs to enforce a mathematically-flat cap-plane
face on the plug body BEFORE that body propagates through to the
cup-floor (via the body subtract in the cup-piece composition)
and through to the plug STL (via the plug solid_to_mm_mesh path).

The 2026-05-24 night attempt was post-MC `MatingTransform::SeamTrim`
applied at the cap-plane — F4-blocked by the recon-4 (P) §F-2
paradigm-boundary failure mode (trim plane coincides with body
cap-plane SDF surface → film/intersection artifacts). That attempt
is bookmarked at `a8e3e056` with the trim builders retained as
defensive primitives + tests.

Four candidate approaches for the next recon to consider, ranked
by my current estimate of viability:

### Candidate (1) — SDF halfspace intersect at the plug body (recommended)

**Approach**: compose the plug body Solid with an SDF halfspace
intersect that enforces the cap-plane plane as a clean halfspace
boundary, BEFORE the body is sampled by MC.

```rust
// Pseudocode in the body-derivation path (cf-cast-cli derive or
// cf-cast spec.rs):
let body = original_scan_body;
let cap_plane_halfspace = Solid::plane(cap_normal, dot(cap_normal, cap_centroid));
let body_with_cap = body.intersect(cap_plane_halfspace);
// `body_with_cap` is then used wherever `body` was used.
```

**Paradigm**: SDF-side composition (same column as recon-4 (P)'s
seam-flat fix). The halfspace SDF is exactly linear; the
intersect with the body SDF preserves linearity AT the cap-plane
boundary; MC interpolation places cap-plane vertices bit-precisely
to f64 precision per recon-4 (P) §F-4.

**Risk**: the body SDF post-intersect must NOT extend past the
cap-plane on the cup-floor side. cf-scan-prep's `trim_floor_mm`
processing should ensure the scan body extends only on the plug-
body side of the cap-plane; need to verify on iter-1 by querying
the body SDF at points BELOW the cap-plane (should return
exterior). If verified, the intersect is a no-op everywhere
except the cap-plane boundary itself.

**Recon-prep verification**: load iter-1 scan body Solid, query
`body.evaluate(cap_centroid + ε × cap_normal)` for ε in
[+0.5, +1, +5, +20] mm (= on the cup-floor side of cap-plane).
Expected: all exterior (> 0). If yes, the intersect is safe.

**Effort estimate**: small if the verification passes. ~50 LOC
across cf-cast-cli derive path + 1 test. 1 impl session.

**Why this is recommended**: same paradigm as the known-good
recon-4 (P) seam-flat fix. Bit-precise flatness by construction.
No paradigm-boundary risk (the halfspace intersect is composed
INTO the body SDF, not applied as a post-MC trim).

### Candidate (2) — Mesh-CSG slab subtract with offset

**Approach**: use the existing `build_half_space_slab` primitive
in `mesh_csg.rs`. Build a slab whose +cap_normal-facing face is
at `cap_centroid + ε × cap_normal` (offset by ε to AVOID
coincident-face with the body cap-plane SDF). Append a
`MatingTransform::SubtractCylinder`-style subtract (slab variant)
to the plug transforms Vec.

**Paradigm**: mesh-CSG (post-MC). The mesh_csg.rs docstring
explicitly identifies build_half_space_slab as "retained for
future paradigm-boundary primitives that may need an explicit-
slab boolean as a numerical fallback" — this is exactly that
use case.

**Risk**: ε too small → slab boundary still close enough to body
cap-plane SDF to trigger paradigm-boundary artifacts. ε too
large → lose visible amount of plug material (epsilon worth of
plug height). For 3 mm MC cells, ε ≈ 0.5 mm (~1/6 of cell size)
likely safe per the recon-4 (P) experience with `extend_near_end`
overlap-bias. Need probe to verify.

**Recon-prep verification**: same g7c-style synthetic probe but
on a flat-face fixture (cube or half-shell with flat cap),
union/subtract a slab with various ε offsets; measure post-CSG
component count + F4 critical types per ε.

**Effort estimate**: ~80 LOC + new MatingTransform variant
(SubtractSlab or reuse SeamTrim with slab implementation). 1
recon session + 1 impl session.

**Pros**: works regardless of body SDF extent (no need to verify
body doesn't extend past cap-plane). More general than (1).

**Cons**: paradigm-boundary risk requires careful probe. Loses
ε of plug material.

### Candidate (3) — Different MC cell size for body region

**Approach**: locally finer MC for the plug body's cap-plane
region (e.g., 0.5 mm cells in a ~10 mm band around the cap-plane,
3 mm cells elsewhere).

**Paradigm**: pre-MC pipeline change. Adaptive mesh.

**Risk**: cf-cast's current `mesher::solid_to_mm_mesh` API takes
a SINGLE `cell_size_m` — no adaptive support. Would require new
MC infrastructure.

**Effort estimate**: large — new mesher API. Multi-week. NOT
viable for this follow-up arc unless we want to scope-creep into
mesher infrastructure work.

**Defer**: not recommended unless (1) and (2) both fail.

### Candidate (4) — Accept the MC quantization (no fix)

**Approach**: document in procedure.rs that workshop user should
manually sand the cup-floor + plug-bottom faces flat after print
(or accept the wobble).

**Paradigm**: documentation only.

**Risk**: workshop user judges acceptable or not.

**Effort estimate**: ~20 LOC procedure.rs prose addition.

**When this is the right pick**: if (1) verification fails AND (2)
probe shows paradigm-boundary artifacts at any reasonable ε. The
workshop user effectively bears the cost of post-print sanding.

## Recommended next session

1. **Triage decision**: workshop user reviews findings A-D in
   this doc + decides which are blockers vs ergonomic. Open
   questions for the workshop user:
   - Is sub-mm cup-floor / plug-bottom wobble acceptable for
     iter-3 print? (informs whether A+B+C are blockers)
   - Is the dome-edge non-flatness visible in cf-view a workshop
     concern or just visual noise? (informs whether D2 is a
     blocker)
2. **If A+B+C are blockers**: open the cap-plane recon arc.
   First task: verify Candidate (1)'s body-extent assumption on
   iter-1 (query body SDF at points past cap-plane on cup-floor
   side). If safe, ship (1) — single impl session, bit-precise
   flat by construction, no paradigm-boundary risk.
3. **If (1) fails verification**: probe Candidate (2) on a
   synthetic flat-face fixture (g7c-style); pick the safe ε
   offset; ship.
4. **If D2 is a blocker after cap-plane fix lands**: open
   separate D2 recon arc (likely needs paradigm-careful approach
   for the dome-edge sub-mm quantization).
5. **Once geometry blockers resolved**: workshop iter-3 print
   on Bambu A1 + default + Jayo. Workshop-user-physical PAUSE
   point. Caliper data → S7 calibration of clearances + chamfers
   in `PrismaticPinSpec` defaults.
6. **Cold-read close + PR to main**: final session after iter-3
   workshop print success. Includes procedure.rs prose updates
   for any geometry changes, dead-code cleanup (`MatingTransform::UnionCylinder`
   confirmed unused, deletion deferred per recon-1 §G-5), and
   omnibus PR open.

## Lessons referenced

- `[[feedback-read-prior-arc-memory-before-architectural-decisions]]`
  — saved 2026-05-24 night post-mortem. The cap-plane recon MUST
  read recon-4 (P) impl memory + mating-features S4 seam-plane
  memory + paradigm-boundary framework memory BEFORE picking a
  candidate. Same failure mode that the trim attempt hit at
  `a8e3e056`.
- `[[project-cf-cast-sdf-meshcsg-paradigm-boundary]]` — recon-4
  (P) framework, applies directly to Candidate (2)'s slab-subtract
  paradigm-boundary risk.
- `[[project-cf-cast-seam-face-film-recon4-impl]]` — recon-4 (P)
  implementation details for the SDF halfspace intersect approach
  (Candidate 1 is the same approach for cap-plane that recon-4
  (P) used for seam plane).
- `g7c_naive_union_paradigm_boundary_probe.rs` — permanent
  regression guard from the 2026-05-24 night post-mortem; the
  cap-plane recon's synthetic probe (if Candidate 2) should
  follow the same pattern: include a cross-primitive control on
  the same fixture to avoid the §G-7 "evidence was not shape-
  specific" mistake.

## Status

- Salvage commits: `b2ff45d5` + `fed4b0c6` + `a8e3e056` on dev.
- Workshop user cf-view-verified: cup-pin features + base-down
  captive plug-lock pyramid. Cup-pin/lock geometry unblocked.
- This doc: workshop user reviews + decides triage in next
  session. No production code change tonight.
- 201 cf-cast lib tests + 8 g7_g9 probe + 1 g7c probe + 1
  single-layer-smoke / clippy / fmt clean.
- cf-cast-cli iter-1 regen ~5 min produces 11 STLs at
  `~/scans/cast_iter1/` with current captive-lock geometry.
