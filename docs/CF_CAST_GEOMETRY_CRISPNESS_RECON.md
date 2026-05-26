# cf-cast geometry-crispness recon (2026-05-25)

**Status:** scaffold. Triggered by workshop user's cf-view smoke
gate inspection of `~/scans/cast_iter1_post_flange_smoke/` STLs
(the [[project-cf-cast-seam-flange-s4]] production output).
Six distinct failure modes catalogued during the live cf-view
walkthrough; this recon scaffolds the triage table + per-failure-
mode fix candidates + the implementation arc.

**Scope:** geometry-quality / visual-fidelity improvements
**within** the shipped curve-following + seam-flange + gasket
architecture. Strictly additive (and possibly refinement of
existing primitives) — does NOT retire any shipped arc. The
condo-floor weight constraint
([[project-workshop-condo-weight-constraint]]) rules out
concrete-backed fixes; the cuboid pivot was explored + reverted
in the same session. This recon is the response to "I want to
solve this problem within the current architecture, without
giving up on what we have."

**Six failure modes (cf-view diagnostic 2026-05-25):**

1. **§Q-1 Seam-face perimeter saw-tooth** — triangular spikes
   along the flange's outer edge; flange's defining geometric
   features blur into a peripheral lip.
2. **§Q-2 Plug-floor-lock polygonal socket** — the truncated-
   pyramid cup-piece socket cavity has visible coarse mesh-CSG
   triangulation (~6-8 facets per face).
3. **§Q-3 Cavity walls degraded scan resolution** — re-MC at 3 mm
   cells coarsens the scan mesh's native sub-mm contour.
4. **§Q-4 Cap-plane × cavity floor not flat** — the plug-mating
   plane in the cup cavity is a saw-tooth mess; the plug's
   matching face is also stair-stepped. The two surfaces don't
   seat flush.
5. **§Q-5 Normals are weird** — visible bright/dark triangular
   patches on the plug surface suggesting inconsistent winding
   order or normal-export bug.
6. **§Q-6 Flange asymmetric — possibly missing on some sides** —
   visible flange tail on one cup-half side but not all the way
   around the perimeter. **OPEN: real bug vs viewing-angle vs
   genuine geometric asymmetry; disambiguate via 360° camera
   rotation before scoping fix.**

**Lessons carrying forward** (already-banked patterns this recon
must respect):

- The SDF/MC ↔ mesh-CSG paradigm-boundary framework
  ([[project-cf-cast-sdf-meshcsg-paradigm-boundary]]) — each
  failure mode lives in a specific paradigm; fixes that
  cross paradigms cost extra complexity.
- The (4') accept-not-fix pattern
  ([[project-cf-cast-cap-plane-flatness-bookmark]]) — some
  quantization is below print resolution + workshop-acceptable;
  fixing it via mesh-CSG can be doubly contraindicated.
- The "read prior-arc memory before architectural decisions"
  rule ([[feedback-read-prior-arc-memory-before-architectural-decisions]])
  — specifically rule #6: bail-out picks MUST NOT silently
  re-retire a load-bearing prior-arc fix. The FDM-friendly
  geometry arc's S4 retired the SDF-side mating-feature
  mechanism after a workshop cf-view smoke gate; we must NOT
  bail back to it.
- LESSON 4 from [[project-cf-cast-seam-flange-s4]]'s cold-read:
  empirical baselines must be measured or softened, never
  asserted from imagined values. Compute-cost claims in §Q-7
  fix candidates need empirical anchoring or explicit "TBD,
  verify at S1" markers.

**Out of scope:**

- Workshop-fabrication changes (concrete fill, vacuum-bag clamp,
  air-bladder press) — separate arc if needed; weight-constraint
  filters those.
- Pivoting away from curve-following bounding region. Workshop
  user's directive: stay within current architecture.
- Retiring the seam-flange / seam-gasket-mold / FDM-friendly-
  geometry / mating-features arcs.

## §Q-0 Why this recon

cf-view smoke gate on `~/scans/cast_iter1_post_flange_smoke/`
2026-05-25 produced 6 distinct visual quality complaints from the
workshop user. The complaints span **THREE identified paradigm
strata** in the cast pipeline + **one TBD stratum** pending §Q-6
disambiguation:

| Paradigm | Affected by failure modes |
|---|---|
| SDF/MC composition at 3 mm cells | §Q-1, §Q-3, §Q-4 |
| Mesh-CSG mating-feature primitives | §Q-2 |
| Mesh export (STL writer + normal computation) | §Q-5 |
| TBD pending §Q-6 diagnostic (could be per-side halfspace cut OR scan-driven geometric asymmetry OR viewing-angle illusion) | §Q-6 |

Each paradigm has its own quality knobs + cost surfaces. A single
"fix the geometry" patch doesn't exist — the fixes are paradigm-
specific. This recon enumerates the candidates per failure mode +
identifies cross-failure-mode synergies (single fixes that address
multiple failure modes).

## §Q-1 Seam-face perimeter saw-tooth

**Symptom:** triangular spikes 1-3 mm in extent all along the
flange's outer edge perimeter. The flange's 15 mm × 4 mm × 2 mm
inner-offset profile is visible as a rounded peripheral lip
rather than a clean rectangular skirt.

**Paradigm:** SDF/MC composition. The flange SDF
(`flange::FlangeSdf::eval`) uses `max(outer, inner, vertical)`
compositions — sharp ridges in SDF that MC interpolation at 3 mm
cells rounds into ~1-cell-wide chamfers + saw-tooth at oblique
SDF crossings.

**Root cause specifics:**

- 4 mm flange thickness is barely above 1 MC cell width (3 mm).
- 2 mm inner-offset gap (between body cavity perimeter and
  flange's inner edge) is sub-cell — entirely unresolved.
- The flange's OUTER edge follows the body perimeter contour at
  `body_dist = inner_offset + width = 17 mm`; the SDF zero-
  crossing crosses MC cells at oblique angles → saw-tooth.

**Fix candidates:**

1. **Finer MC cells globally**: drop `mesh_cell_size_m` from 3 mm
   to ~1 mm. Resolves features cleanly. **Cost: TBD — verify
   empirically at §Q-11 S1.** Quantitative bound only: MC sample
   count scales as `(3/1)³ = 27×` per piece (lower bound on
   compute), but **wall-clock scaling is NOT purely cubic** —
   F4 + post-MC mesh-CSG + STL I/O scale separately (F4 thin-wall
   BVH at O(n log n), trapped-volumes at O(voxel-grid), I/O at
   O(triangle count)). Parallel-meshing-S2's 2.6× amortizes
   compute across cores; F4-S1 + F4-S3 + self-intersect-BVH may
   ALSO need re-calibration at the new face-count regime (see
   §Q-15 cross-arc interactions). **The pre-S1 wall-clock
   projection range is uncertain enough that any specific number
   would be hand-waving; S1 empirical regen is the load-bearing
   data point.**
2. **Adaptive MC cell sizing**: coarse cells (3 mm) over body
   bulk, fine cells (~0.5-1 mm) near flange + cap-plane +
   thin-feature regions. Compute-efficient version of (1).
   **Cost: significant infrastructure work (new MC pipeline
   wrapping the existing single-cell-size flow). LOC estimate
   ~600-1000 across cf-design + cf-cast — TBD pending S2
   scaffold (this LOC range is a rough order-of-magnitude
   guess, NOT empirically anchored). Pre-existing bookmark from
   [[project-cf-cast-self-intersect-bvh-s1]] §S-12.**
3. **Mesh-CSG flange primitive (additive)** (paradigm extension,
   NOT replacement): add a mesh-CSG flange variant ALONGSIDE the
   shipped SDF `FlangeSdf` (workshop user toggles via a new
   `FlangeKind::MeshPlate(spec)` variant or similar). Build the
   flange as a separate analytical mesh primitive (clean
   rectangular box; sharp edges), post-MC `.union()` it onto
   the cup-piece mesh. Same paradigm placement as the existing
   `MatingTransform::Union/SubtractCylinder` pins.
   **Cost: ~200-300 LOC (TBD anchor; needs scaffold). Additive
   to FlangeKind; the existing `Plate(FlangeSpec)` variant
   stays.** Per-perimeter offset is geometrically more complex
   than a rectangular box; mesh-CSG flange might need
   per-perimeter sweep approximation OR accept a rectangular-
   bounding-box approximation. The `Plate` (SDF) variant remains
   default until S1 empirical comparison.
4. **Smoother flange SDF**: replace `max(...)` with `smin`-style
   smooth-max blends. **Cost: ~20 LOC. Accepts slightly rounded
   flange profile (no sharp 90° edges); doesn't fix the
   thin-feature resolution problem (4 mm × 3 mm cells stays
   bad).** Workaround tier.

**Recommendation pending §Q-12 S1 measurement:** (1) is the
simplest single-knob fix; (2) is the compute-efficient version
if (1) busts wall-clock budget; (3) is paradigm-clean but adds
mesh-CSG complexity; (4) is a partial workaround.

## §Q-2 Plug-floor-lock polygonal socket

**Symptom:** the cup-piece's cap-plane interior socket cavity
(where the plug-floor-lock truncated pyramid mates) is a coarse
6-8 facet polygonal hole, not a clean truncated-pyramid shape.

**Paradigm:** mesh-CSG mating-feature. The plug-lock socket comes
from `MatingTransform::SubtractTruncatedPyramid` shipped at
[[project-cf-cast-fdm-friendly-geometry-arc]] S4. The pyramid is
built from a **convex hull over a small point set** via
`mesh_csg::build_truncated_pyramid_via_hull_pts`
(`design/cf-cast/src/mesh_csg.rs:356`); boolean-subtract preserves
the hull's native triangulation.

**Root cause specifics (verified against `mesh_csg.rs`):**

- The pyramid is **4-sided rectangular** (NOT 6-sided): rectangular
  base × rectangular top × 4 trapezoidal lateral faces.
- Point counts feeding the convex hull (`Manifold::hull_pts`):
  - **Without chamfer:** 4 base + 4 tip = 8 points → ~12 triangles.
  - **With chamfer (production default, `base_chamfer_m > 0`):**
    8 base (4 inner + 4 outer) + 4 tip = 12 points → ~16-20
    triangles (exact count depends on convex-hull triangulation).
- Workshop user's "6-8 facet" cf-view observation matches the
  chamfered-pyramid edition viewed from one camera angle (visible
  lateral facets + chamfer-band triangles from that side).
- MC cells are not involved at this surface — increasing
  `mesh_cell_size_m` doesn't help.

**Fix candidates:**

1. **Finer-tessellated truncated pyramid primitive**: replace the
   `Manifold::hull_pts` call with a **parametric mesh constructor**
   that subdivides each pyramid face (lateral + chamfer-band)
   into N×N triangles. Doesn't apply regular `12 × N²` scaling
   because the hull's native triangulation isn't a regular
   subdivision; a parametric constructor would emit roughly
   `(4 lateral faces + base + top) × 2 triangles × N²` =
   `12 × N²` ≈ N=4 → ~192 triangles, N=8 → ~768. **Cost: ~80-150
   LOC in `mesh_csg::build_truncated_pyramid_via_hull_pts` (or a
   sibling `_subdivided` constructor); requires switching from
   convex-hull to explicit parametric face mesh — not a trivial
   knob.** Workshop-visible improvement scales with N; budget-
   bound by F4 wall-clock impact (more triangles per piece → more
   F4 work).
2. **Replace truncated pyramid with cylindrical post primitive
   ⚠ VIOLATES PRIOR-ARC-MEMORY RULE #6**: cylinders mesh with ~32
   segments around → 64 triangles for lateral surface alone.
   Visually MUCH crisper than truncated pyramid at workshop scale.
   **Cost: ~150 LOC (new `PrismaticPinSpec` variant).** But:
   **this candidate silently reverts the [[project-cf-cast-fdm-friendly-geometry-arc]]
   S4 decision** which explicitly picked truncated-pyramid OVER
   cylindrical for the self-centering wedge action workshop
   ergonomics. Per [[feedback-read-prior-arc-memory-before-architectural-decisions]]
   rule #6, **DO NOT pick this candidate without an explicit
   workshop-user re-decision** that re-opens the FDM-friendly-S4
   call. Listed for completeness only.
3. **Higher mesh-CSG resolution applied uniformly**: not just
   the plug-lock; cup-pin sockets too. Same fix as (1) applied
   across the prismatic-pin module. **Cost: same per-call-site
   delta as (1).**

**Recommendation:** (1) is the lowest-risk workshop-visible fix
(no rule #6 concern; just primitive-resolution increase). (2) is
listed for completeness but **explicitly contraindicated** unless
the workshop user re-opens the FDM-friendly-S4 decision via a
separate re-decision pass.

## §Q-3 Cavity walls degraded scan resolution

**Symptom:** the cup-piece's interior cavity walls show visible
faceting where the body cavity follows the device contour. The
scan mesh from `~/scans/sock_over_capsule.cleaned.stl` is
triangulated at sub-mm native resolution; cf-cast re-MC's at 3 mm
cells and coarsens it.

**Paradigm:** SDF re-MC of already-meshed scan geometry. The
body cavity is `pinned_floor_shell(...)` per the cf-design
pipeline → a Solid whose SDF is evaluated by sampling the scan
mesh's TriMeshDistance; MC at 3 mm cells then re-meshes the
combined cuboid - cavity SDF.

**Root cause specifics (S0-empirical per
[[project-cf-cast-geometry-crispness-s0]]):**

- Scan mesh resolution: **167,898 faces, AABB ~71 × 71 × 127 mm,
  ~0.66 mm avg edge length** (measured 2026-05-25 via the §R1
  inspector + binary-STL header math).
- MC cell size: 3 mm (production cast.toml).
- **Coarsening factor: ~4.5×** (3 mm cells / 0.66 mm scan edges).
- Re-meshing loses any scan detail finer than ~6 mm (2× cell
  width for MC's typical feature-resolution limit).

**Cost-sharing note (Tier 2 cold-read pass-2):** §Q-3 fix
candidate (1) "Finer MC cells" is THE SAME compute spend as
§Q-1's (1) + §Q-4's (1) — same global cell-size change applies to
all three failure modes simultaneously; the compute cost
amortizes across all three, not adds per-failure-mode.

**Fix candidates:**

1. **Finer MC cells**: same path as §Q-1's (1). Single-knob
   global fix; addresses §Q-1 + §Q-3 + §Q-4 simultaneously.
2. **Adaptive MC**: same path as §Q-1's (2). Targeted fine cells
   around the cavity-wall + cap-plane regions.
3. **Direct scan-mesh use (bypass SDF re-MC for the cavity)**:
   keep the scan mesh as a mesh-CSG SUBTRACT primitive applied
   to the SDF-meshed bounding region. **Cost: ~300-500 LOC for
   the pipeline change; substantial mesh-CSG plumbing.** Most
   paradigm-pure fix but largest scope change.

**Workshop iter-3 assessment:** how visible is this failure mode
in practice? For sock-over-capsule the scan curves are broad +
gentle (the cavity floor + dome sections in cf-view's middle
region look reasonably smooth). The problem shows up at tight
features (cap-plane × cavity floor junction = §Q-4 territory;
dome end's tighter curvature). **May be subordinate to §Q-1 +
§Q-4 in workshop visual-quality priority; defer until empirical.**

## §Q-4 Cap-plane × cavity floor not flat

**Symptom:** the cap-plane region inside the cup cavity (where
the plug's cap-plane face mates) is stair-stepped. The plug's
matching cap-plane face has the same artifact. The two surfaces
should seat plane-on-plane as a clean kinematic constraint; they
instead rock against quantized stair-steps.

**Paradigm:** SDF/MC composition. The cap-plane is a derivative-
discontinuity surface (body cavity SDF × cap-plane half-space SDF)
re-MC'd at 3 mm cells.

**Workshop consequence:** the plug rocks during pour → cured
device has off-axis tilt at the cap end. Affects the cured
device's pour-end cosmetic finish + may break the v2.1
detachable-shell post-cure alignment if the plug-floor sleeve fit
is sensitive.

**Fix candidates:**

1. **Finer MC cells** (same as §Q-1 (1) + §Q-3 (1)). The cap-
   plane orientation interacts with MC's cubic grid the same way
   the seam plane does; finer cells resolve both.
2. **Adaptive MC at cap-plane region** (same as §Q-1 (2)).
3. **Mesh-CSG cap-plane primitive**: post-MC, intersect the cup
   piece with an analytical flat plane (mesh-CSG plane-cut). The
   cap-plane SDF gets retired; the cap-plane face is built as a
   clean infinite-plane mesh + booleaned. **Cost: ~150 LOC; paradigm-
   shift on the cap-plane geometry alone.** Same approach as the
   seam-face halfspace cut (which IS bit-precisely flat per
   recon-4 §F-2) but applied to the cap-plane.
4. **Accept + workshop sand** (the (4') pattern from
   [[project-cf-cast-cap-plane-flatness-bookmark]]): document the
   stair-step as expected MC quantization; workshop user sands
   the plug + cavity cap-plane faces flush before assembly.
   **Cost: ~50 LOC procedure.rs prose; the (4') doc-anchor test
   pattern.** Doesn't fix the geometry; sets expectations.

**Recommendation:** candidate (3) is the cleanest paradigm fix —
the seam-face halfspace cut already proves the technique works
for one orthogonal plane; extending to the cap-plane is a parallel
application. Candidate (4) is the bail-out if iter-3 surfaces a
workshop-blocking issue and (3) isn't ready.

## §Q-5 Normals are weird — ✅ FIXED 2026-05-26 dev `efdff6b8`

**Status: RESOLVED at the upstream foundational layer.** See
[[project-cf-cast-geometry-crispness-q5-fix]] for the full fix
record. The S0 diagnostic + investigation chain identified the
root cause at `mesh-offset/src/marching_cubes.rs:160-162` (face
emission missing the e1/e2 swap that cf-design's `mesher.rs` uses
for CCW outward winding). 1-effective-LOC fix + downstream
compensation removal in `mesh-shell/src/shell/generate.rs:300`.
1353 tests across 7 crates pass; production regen confirms
platform.stl 100% → 0% inverted-by-heuristic. Workshop visual
gate (cf-view inspection of `~/scans/cast_iter1_post_winding_fix/`)
pending.

The investigation history below is preserved for context.

### Original §Q-5 framing

**Symptom:** visible bright/dark triangular patches on the plug
surface in cf-view; lighting doesn't match the underlying smooth
dome curvature.

**Paradigm:** mesh export (STL writer) + upstream mesh winding
convention. cf-view's StandardMaterial with `double_sided: true,
cull_mode: None` renders both sides of every triangle, so visible
patches = lighting discontinuities at feature boundaries where
adjacent inverted-winding triangles produce inconsistent shading.

**S0 diagnostic result (per [[project-cf-cast-geometry-crispness-s0]]):**

**100% of cf-cast STL output has systematically inverted triangle
winding.** Smoking gun: `platform.stl` (a flat slab with no
concavity → centroid heuristic is unambiguous) reports 100% of
its 33,340 triangles' winding-derived normals point INWARD toward
the volume centroid. Spot-checked individual triangles confirm:
top-face triangles have normals pointing -Z (into the slab from
above); bottom-face triangles have normals pointing +Z (into the
slab from below). Plug + cup-piece + gasket-mold STLs show
55-99% inverted-by-heuristic (lower rates explained by the
centroid heuristic's false-negative band on shapes with
cavities). Scan input mesh
(`sock_over_capsule.cleaned.stl`) reports 1.25% inverted —
within noise; the scan winding is CORRECT.

**The inversion happens INSIDE cf-cast's pipeline**, not in the
scan input. Three candidate layers:

- (a) **STL writer (`mesh_io::stl::save_stl_binary`)**: uses
  `(v1 - v0).cross(v2 - v0)`. If the upstream mesh's winding is
  CW outward (some Manifold3d configurations), this cross-
  product gives an INWARD normal. **2-LOC fix possible** —
  reverse the cross-product order OR swap `v1`/`v2` when
  writing the face. **MOST LIKELY ROOT CAUSE** given the
  uniformity of inversion across MC + mesh-CSG outputs (all
  produce STLs through this writer).
- (b) **MC implementation** in cf-design / manifold3d: if MC
  emits CW winding, all bulk geometry is inverted. Mesh-CSG
  primitives (truncated pyramids, cylinders) would also need
  to match the same convention.
- (c) **Mesh-CSG operations** in mesh_csg / manifold3d: boolean
  union/subtract at the manifold boundary may invert winding.

**Workshop visual impact:** cf-view's StandardMaterial with
double-sided + cull-none rendering shows inverted-normal
geometry, but the shading is INCONSISTENT at feature transitions
(plug dome × cylinder junction; chamfer-band edges). These are
the "bright/dark patches" the workshop user saw. Slicers
typically recompute normals from winding on STL import + handle
single-sided rendering robustly, so the inverted output may
print fine in slicer preview AND in physical printing (slicers
re-derive outward direction); but for cf-view inspection +
workshop user expectations + tools that DO use stored STL
normals (some 3D viewers), the inversion is workshop-visible.

**Fix candidates (revised post-S0):**

1. **Reverse winding at STL write time** (most likely correct
   layer per candidate-source analysis above). Swap `v1` and `v2`
   in `save_stl_binary` (or equivalently negate the cross-product).
   **Cost: ~2 LOC + a regression test ensuring scan-input
   ROUND-TRIP produces correct winding.** Workshop-visible result
   confirmed via re-run of the §Q-5 S0 diagnostic on the new
   output: platform.stl should report ~0% inverted-by-heuristic
   post-fix.
2. **Fix upstream MC / mesh-CSG winding convention** (if (1)'s
   reverse-at-write hides a real upstream bug instead of fixing
   it). **Cost: depends on which layer; ~10-100 LOC.** More
   thorough; protects against future consumers that bypass
   `save_stl_binary` (e.g. PLY writer if cf-cast ever emits PLY).
3. **mesh-repair `enforce_consistent_winding(mesh)` pass**: walks
   the mesh's face-adjacency graph, flips any triangle whose
   normal disagrees with its connected-component majority.
   **Cost: ~150 LOC mesh-repair lib + tests.** **Overkill given
   S0 finding** — there's nothing to disambiguate; ALL output is
   inverted in the same direction. Listed for completeness but
   no longer recommended.
4. **cf-view recompute normals**: ignore stored STL normals; have
   cf-view recompute per-vertex normals from winding. **Cost:
   ~30 LOC cf-view. Doesn't fix the STL ITSELF** (workshop
   tools that respect stored STL normals still see bad data).
   Listed as a partial workaround.

**Recommendation:** (1) the 2-LOC winding reversal at write time
is the smallest workshop-visible fix path. If post-fix the §Q-5
S0 diagnostic shows correct winding (platform → ~0% inverted),
ship it. If it surfaces another upstream issue (e.g. some
triangles still inverted from a different convention), escalate
to (2). The S0 diagnostic script (`/tmp/normal_check.py`) is
the verification gate.

## §Q-6 Flange asymmetric — possibly missing on some sides

**Symptom:** in cf-view's assembly mode with both cup halves
visible (Image 7 of the cf-view smoke gate), the flange visibly
extends to one side of the cup body but NOT all the way around
the perimeter.

**Paradigm:** TBD — either a real geometric bug in
`build_flange_solid_for_side` per-side halfspace logic, OR a
viewing-angle illusion (flange sections parallel to camera appear
edge-on and invisible), OR genuine geometric asymmetry (body
cavity narrows on certain sides → flange's outer-edge contour
narrows similarly).

**Status: PARTIALLY ANSWERED post-S0 (per [[project-cf-cast-geometry-crispness-s0]]); workshop 360° cf-view visual disambiguation still pending.**

**S0 analytical-pass result:** `build_flange_solid_for_side` in
`design/cf-cast/src/flange.rs:283-292` is **structurally
symmetric** — `build_flange_solid` produces the full perimeter
ring SDF with no per-side bias; the per-side cut is just
`ribbon.halfspace_solid(side, bounds, 0.0)` (same mechanism that
produces the bit-precise-flat seam face, verified in
[[project-cf-cast-seam-flange-s1]] LESSON 3). **Branch C (real
bug) is geometrically unlikely** given the code's symmetry.
Branch A (viewing-angle illusion) or branch B (scan-driven
asymmetric body perimeter narrowing → flange's outer-edge
contour narrows correspondingly + appears edge-on / invisible
from certain camera angles) are the candidates. Visual A-vs-B
disambiguation still pending workshop input.

**Diagnostic (§Q-12 S0):**

- Rotate the cf-view camera 360° around the assembled cup pair.
- Capture views from at least 4 camera positions (front, back,
  left, right of the body).
- For each view, confirm whether the flange ring is visible all
  the way around OR genuinely missing on certain sides.

**Outcome branches:**

- **A) Viewing-angle illusion**: nothing to fix. Document in
  procedure.rs + workshop user knows the flange is fully present
  but invisible from certain camera angles.
- **B) Genuine geometric asymmetry** (scan-driven; body perimeter
  narrows): document expected behavior; flange's outer-edge
  contour inherits the body-perimeter narrowing. Workshop user
  should know this is expected for sock-like scans.
- **C) Real bug**: trace `build_flange_solid_for_side`'s halfspace
  cut + the FlangeSdf's projection-to-seam-plane logic. The S1
  LESSON 2 memory notes the PieceSide ↔ ±Y mapping was empirically
  corrected from an initial-wrong assumption; a regression in
  that area could be the cause. **Cost: TBD — depends on whether
  the bug is a 1-line sign flip or a more structural issue with
  the per-perimeter SDF.**

## §Q-7 Cross-failure-mode synergies

Some fixes address multiple failure modes at once. Tabulating
the high-value synergies:

| Fix candidate | Addresses |
|---|---|
| Finer MC cells globally | §Q-1, §Q-3, §Q-4 |
| Adaptive MC cell sizing | §Q-1, §Q-3, §Q-4 |
| Mesh-CSG flange primitive | §Q-1 only |
| Mesh-CSG cap-plane primitive | §Q-4 only |
| Finer-tessellated truncated pyramid | §Q-2 only |
| `enforce_consistent_winding` mesh-repair pass | §Q-5 only |

**Highest-value single fix: adaptive MC cell sizing.** Addresses
3 of 6 failure modes simultaneously. But: largest implementation
cost (~600-1000 LOC infrastructure work) + bookmarked as a future
arc post-self-intersect-BVH-S1 §S-12, not yet scaffolded in detail.

**Cheapest single fix: finer MC cells globally.** Addresses same 3
failure modes; trades implementation simplicity for compute cost.
**Empirical verification at §Q-12 S1 needed — if 1 mm cells busts
the wall-clock budget, fall back to adaptive MC OR accept the
quantization.**

## §Q-8 Open questions — status post-S0 diagnostics

| # | Question | Status |
|---|---|---|
| 1 | §Q-6 disambiguation (bug vs viewing-angle vs scan-asymmetry) | **PARTIALLY ANSWERED** — S0 analytical pass rules out branch C (real bug) as geometrically unlikely; A-vs-B still pending workshop 360° cf-view rotation |
| 2 | §Q-1 + §Q-3 + §Q-4 fix path (finer MC vs adaptive MC vs paradigm shift) | OPEN — driven by §Q-11 S1 empirical regen |
| 3 | §Q-2 truncated-pyramid vs cylindrical post | OPEN; recommendation per pass-1 cold-read: stay with pyramid + subdivide (candidate (1)); reverting to cylindrical violates rule #6 |
| 4 | §Q-5 normal-correctness scope | **ANSWERED post-S0**: 100% systematic (NOT rare-artifact); simple writer-or-upstream fix path; 2-LOC candidate (1) identified |
| 5 | Production scan mesh resolution | **ANSWERED post-S0**: 167,898 faces / AABB ~71×71×127 mm / ~0.66 mm avg edge length / 4.5× coarsening at 3 mm MC cells |
| 6 | MC cell-size scaling at 1 mm — empirical wall-clock | OPEN — load-bearing for §Q-11 S1 (the headroom estimate "~20× headroom" was hand-waved pre-S0; needs the actual S1 measurement) |
| 7 | GASKET geometry crispness — hidden §Q-7 entry? | **ANSWERED post-S0**: NO. Gasket molds already use 0.5 mm cell override (6× finer than cup pieces); subordinate priority as transient artifacts |

**S0 closed 4 of 7 (#1 partial; #4 + #5 + #7 fully).** Open
questions for S1: #2 (fix-path selection driven by S1
measurement), #3 (workshop-user re-decision territory; not
S1-blocking), #6 (THE load-bearing S1 measurement).

**Workshop-user gate (out-of-S0)**: 360° cf-view rotation to
close #1 (branch A vs B). Quick: open assembly mode in cf-view +
rotate camera around the cup pair while both halves are at α=1.0;
confirm whether the flange ring is visible from at least one
angle on every side.

## §Q-9 Workshop iter-3 unblock criteria

Quad-gate post-arc-ship:

1. **§R1 connectivity** holds on all 6 cup-half STLs after fixes
   ship: 1 component each. (Already passing pre-arc; verify no
   regression.)
2. **F4 gate passes (Critical = 0)** on all cup halves post-fix.
   (Already passing pre-arc; verify no regression.)
3. **Workshop cf-view smoke**: each of the 6 failure modes is
   either visibly improved (workshop-user judgment call) or
   explicitly documented as accepted (the (4') pattern).
4. **Workshop physical print + pour test (iter-3)**: plug seats
   flush in cup cavity (§Q-4 fix); plug-floor-lock socket
   accepts plug without hand-fitting (§Q-2 fix); flange clamps
   evenly all around (§Q-6 not-a-bug confirmed OR fixed).

## §Q-10 Bail-out priority

If §Q-12 S1 measurements reveal that the projected fix costs
bust the workshop wall-clock budget:

1. **Cell-size compromise**: drop from 3 mm to 2 mm globally
   instead of 1 mm. Should preserve ~2× more feature resolution
   at ~8× compute cost (vs 27× at 1 mm). Workshop-visible
   improvement smaller but real.
2. **Targeted-region fix only**: skip §Q-3 (cavity walls degraded
   scan resolution) — workshop-acceptable per S4 smoke gate; the
   cavity middle DOES look smooth. Focus on §Q-1 + §Q-4 only.
3. **Accept § Q-1 + §Q-4 via the (4') pattern** (procedure.rs
   prose; document expected quantization; workshop user sands or
   accepts the cosmetic finish).
4. **Drop §Q-5 normals** to cf-view-only fix (recompute in
   viewer, don't fix the STL). Slicers handle STL normals
   robustly in practice (recompute from winding on import).
5. **Defer §Q-6 entirely** to post-iter-3 physical pour. If the
   flange asymmetry doesn't workshop-block, skip the fix.

## §Q-11 Implementation arc

**Estimated 5-7 phases, ~5-8 wall-clock sessions:**

- **S0: Diagnostics (no code change). ✅ SHIPPED 2026-05-25**
  (per [[project-cf-cast-geometry-crispness-s0]]).
  - **§Q-6 disambiguation** (PARTIAL): code-side analytical
    pass — branch C unlikely; A-vs-B still pending workshop
    360° cf-view rotation.
  - **§Q-5 normal-correctness diagnostic** (FULL): 100%
    systematic inverted winding across cf-cast STL output;
    scan input clean.
  - **§Q-3 scan mesh resolution measurement** (FULL): 167,898
    faces / ~0.66 mm avg edge length / 4.5× coarsening at 3 mm
    MC cells.
  - **§Q-7 gasket crispness check** (FULL via code-side):
    subordinate priority; 0.5 mm cell override already
    optimized.
- **S0.5 (NEW post-S0): §Q-5 winding-reversal fix expedited**
  (proposed; awaiting workshop user pick). Reverse triangle
  winding at write time in `mesh_io::stl::save_stl_binary`
  (~2 LOC) + a regression test asserting platform.stl reports
  ~0% inverted-by-heuristic post-fix (re-run S0's diagnostic
  script `/tmp/normal_check.py` as the gate). Decoupled from
  §Q-1 + §Q-3 + §Q-4 fix path; ships independently. **Time
  cost: ~5 min implementation + ~5 min verification + 1 brief
  cf-view check.** Workshop user can ship this BEFORE §Q-11 S1
  to confirm the visual fix without mixing it with the
  finer-MC-cells regen.
- **S1: §Q-1 + §Q-3 + §Q-4 finer-MC-cells experiment** (~5 LOC
  config change + production iter-1 regen + cf-view smoke gate).
  Change `mesh_cell_size_m` from 0.003 to 0.001 in
  `~/scans/cast.toml`; measure wall-clock impact + cf-view
  visual improvement. **Empirical-only ship phase (like the
  F4-S2 / seam-flange-S4 pattern).** **MUST run with
  `MESH_PRINTABILITY_TIMING=1` per §Q-15 cross-arc table to
  catch any perf-arc regression at the larger face-count
  regime.** Time budget: ~10-50 min per regen attempt
  depending on cell-size scaling; budget 2-3 attempts.
  Decision branches:
  - Wall-clock acceptable + visual improvement satisfies §Q-1
    + §Q-3 + §Q-4 → proceed to S8 (omnibus PR rollup).
  - Wall-clock too slow + visual improvement worth pursuing
    via different path → proceed to S2 (adaptive MC scaffold).
  - Visual improvement insufficient at 1 mm → proceed to S2
    + S3 (paradigm-shift to mesh-CSG flange + cap-plane).
- **S2: Adaptive MC scaffold + implementation** (~600-1000
  LOC). Only if S1 measurement justifies. New MC pipeline
  wrapping existing flow with per-region cell-size override.
- **S3: Mesh-CSG cap-plane primitive** (~150 LOC, **rough
  estimate; no existing primitive scaffolded — verify
  pre-implementation against MatingTransform variant catalogue
  + manifold3d integration cost**). Only if finer MC alone
  doesn't resolve §Q-4 satisfactorily. Post-MC intersect with
  analytical flat-plane mesh.
- **S4: §Q-2 truncated-pyramid subdivision** (~80-150 LOC per
  pass-1 cold-read; requires switching from convex-hull to
  parametric face mesh — not a trivial knob). Add N×N
  subdivision to `mesh_csg::build_truncated_pyramid_via_hull_pts`
  (or a sibling parametric constructor). Default N=4;
  workshop-iter-3 calibration may tune.
- **S5: §Q-5 normal-consistency** (already shipped at S0.5 if
  workshop user takes the expedite path; otherwise scoped here).
- **S6: §Q-6 fix** (only if workshop 360° cf-view inspection
  reveals branch C — geometrically unlikely per S0 analytical
  pass). Scope TBD per the specific bug.
- **S7: Production iter-1 regen + cf-view smoke gate** —
  empirical-only ship verifying all 6 failure modes resolved
  or accepted per §Q-9.
- **S8: Cold-read pass-1 + omnibus PR rollup** (joint with
  any in-flight arcs).

Total scope: **~50 LOC (S1) + 0-1200 LOC (S2-S6 depending on
branches) + ~2 LOC (S0.5 if taken) + procedure.rs prose**.

## §Q-12 Prior-arc memory checklist

Per [[feedback-read-prior-arc-memory-before-architectural-decisions]]
rules 1-6, the following memories MUST be read before any §Q-11
phase touches the primitive in question:

- [[project-cf-cast-fdm-friendly-geometry-arc]] (S1-S7): the
  prior cup-pin + plug-floor-lock arc. §Q-2 touches the
  truncated-pyramid primitive shipped here. **Rule #6: must
  NOT silently revert this arc's S4 (SDF-side → mesh-CSG)
  load-bearing fix.**
- [[project-cf-cast-seam-flange-s1]] through s4 (+ cold-reads):
  the FlangeSdf + FlangeKind machinery. §Q-1 touches the
  flange SDF; §Q-6 may touch the per-side halfspace logic
  (S1 LESSON 2 is directly relevant).
- [[project-cf-cast-seam-gasket-mold-s1]] through s3: the
  gasket-mold geometry. §Q-7 if gasket crispness enters scope.
- [[project-cf-cast-sdf-meshcsg-paradigm-boundary]]: the
  paradigm-boundary framework. §Q-1 (3) and §Q-4 (3) and §Q-2
  candidates all interact with this.
- [[project-cf-cast-cap-plane-flatness-bookmark]]: the (4')
  accept-not-fix precedent. §Q-1 (4) + §Q-4 (4) bail-outs use
  this pattern.
- [[project-cf-cast-self-intersect-bvh-s1]]: §S-12 bookmarks
  adaptive MC cell sizing as a future arc. §Q-1 (2) + §Q-3 (2)
  + §Q-4 (2) are that arc's first scaffold point.
- [[project-cf-cast-f4-spatial-index-s2]]: empirical-only
  measurement-phase precedent. §Q-11 S1 follows this pattern.
- [[project-workshop-condo-weight-constraint]]: rules out
  any concrete / heavy-mass fix paths.

## §Q-13 Retirement scope (NOTHING)

**This arc retires no shipped code.** All fixes are additive
(new mesh-CSG primitives ALONGSIDE existing SDF ones; new mesh-
repair passes; new MC configuration options; new procedure.rs
accept-prose) or in-place refinements (cell-size config change;
pyramid subdivision parameter). The shipped seam-flange + seam-
gasket-mold + FDM-friendly-geometry + parallel-meshing + F4-index +
self-intersect-BVH arcs all carry forward as-is.

**Note for §Q-1 candidate (3) + §Q-4 candidate (3) mesh-CSG
paradigm-extensions:** these are **additive** (new
`FlangeKind::MeshPlate` / new `MatingTransform::IntersectFlatPlane`
variants) — they do NOT retire the existing SDF FlangeSdf or
SDF cap-plane composition. Workshop user picks via the new variant
explicitly; default stays on the SDF path until S1 empirical
comparison.

## §Q-15 Cross-arc interactions (regression-risk table)

The shipped perf arcs (parallel-meshing, F4-spatial-index, self-
intersect-BVH) were each calibrated at the CURRENT 3 mm MC cell
size / ~16-19k faces-per-cup-piece regime. §Q-1 + §Q-3 + §Q-4
candidate (1) (finer MC cells globally) shifts that regime
significantly. Per-arc regression risk analysis:

| Active arc | At 3 mm cells (current) | At 1 mm cells (§Q-1 (1)) | Regression risk |
|---|---|---|---|
| Parallel-meshing-S2 | Wall 1:44; rayon par_iter over pieces+plugs+gaskets phases | Compute per-task grows ~27× (cubic on MC samples). Rayon scaling holds; per-task absolute time grows. | LOW — parallelism amortizes per-task work across cores; the 2.6× speedup factor preserved. |
| F4-spatial-index-S1 (thin-wall BVH) | parry3d-f64 BVH crossover at ~30-50k faces (per [[project-cf-cast-self-intersect-bvh-s1]] LESSON a). Current cup pieces 15-19k faces → BVH amortizes marginally. | Face count grows 9-27× → cup pieces 150-500k faces → BVH amortizes strongly. | LOW — moves face count INTO the BVH's design regime, doesn't break it. |
| F4-spatial-index-S3 (voxel-axis cap) | `MAX_VOXELS_PER_AXIS = 500` cap on `check_trapped_volumes`; production 200 mm part at 3 mm cells = ~67 cells/axis (no cap fires). | At 1 mm cells, 200 mm part = 200 cells/axis. Still under 500 cap → trapped_volumes still bounded. | LOW — cap was sized for 200-300 mm parts; 1 mm cells stays under cap. |
| Self-intersect-BVH-S1 | parry3d-f64 Qbvh crossover at ~30-50k faces (LESSON a). Current gaskets 400k faces strongly amortized; cup pieces marginally. | Cup-piece face counts move into BVH's design regime. | LOW — same crossover reasoning as F4-S1. |
| **Seam-flange-S1 SDF** | FlangeSdf adds 1 max() composition per MC sample. | Same SDF eval per sample but **27× more samples** → compose-time grows ~27×. Cup-piece compose phase was non-dominant at 3 mm cells; at 1 mm it may compete with gasket-MC for wall-clock dominance. | **MEDIUM** — re-shuffles wall-clock-bottleneck ordering; doesn't break correctness. |
| **Seam-gasket-mold-S1 SDF** | Gasket MC at 0.5 mm cells (per `GASKET_MAX_CELL_SIZE_M`); independent of `cast.mesh_cell_size_m`. | UNCHANGED — gasket cell size is per-gasket-spec, not global. | NONE — gasket arc not affected by §Q-1 (1). |

**Load-bearing regression check:** §Q-11 S1 production regen MUST
re-run with `MESH_PRINTABILITY_TIMING=1` enabled (the env-var
introduced by [[project-cf-cast-f4-spatial-index-s3]]) so per-check
F4 timings + per-phase wall-clock are captured. If any check
regresses significantly (e.g. trapped_volumes growing
super-linearly with face count), the perf arcs need recalibration
BEFORE the §Q-1 fix ships.

**MEDIUM-risk seam-flange-S1 SDF compose-time amplification** is
the most likely surprise. Mitigation: §Q-1 candidate (3) — the
mesh-CSG flange paradigm-extension — bypasses the SDF compose-time
amplification by building the flange post-MC. If S1 regen reveals
seam-flange compose time dominating, escalate to candidate (3).

## §Q-14 Successor

After scaffold cold-read pass-1 (per
[[project-cf-cast-seam-flange-s4]] LESSON 4 — same-session
recon cold-read catches drift), the arc enters §Q-11 S0
diagnostics (no code change) to disambiguate the open
questions in §Q-8. Empirical findings from S0 drive S1's
decision branches.
