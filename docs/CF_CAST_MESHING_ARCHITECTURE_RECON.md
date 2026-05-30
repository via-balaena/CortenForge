# cf-cast meshing-architecture recon (§MA, 2026-05-30)

**Status:** scaffold (cold-read-corrected — see §MA-12). This is the
long-deferred **§Q-11** ("finer cells / dual-contouring") question that the
geometry-crispness (§Q-1/§Q-5), §Q-4 planar-trim, and §SMD scan-mesh-direct
arcs all bookmarked and pushed past. It is now the main event.

**Trigger.** Workshop cf-view smoke on the `base_mold` canal cast
(`~/scans/cast_base_mold_canal/`, config `cast.base_mold.canal.toml`)
flagged the cavity-floor → seam/mating-face corner as jagged: "chunks
missing from what should be a flat floor, to a sharp corner, to the
mating face." After an empirical drill-down (this session) the workshop
diagnosed it directly as **resolution**, and raised the real question:

> *"is there somehow a way to generate very fine detail STLs but for it
> to not take forever?"*

…then chose to **slow down and reconsider the whole meshing architecture**
rather than take the fastest patch. This recon is that reconsideration.

The workshop's framing (which this recon adopts): the parts need **both**
faithful organic shape **and** crisp manufactured features, and those two
qualities **interpenetrate on the same surfaces, meeting at the same
edges** — so the lever is not "pick a resolution," it is *how organic and
exact geometry are composed*, and *which isosurface extractor* turns the
field into triangles.

---

## §MA-0 Empirical findings to date (this session)

All measured on `mold_layer_0_piece_0.stl` (layer-0 Negative cup half),
via throwaway Python STL probes (`/tmp/probe_*.py`; may not persist).
Diagnostic regens at `~/scans/cast_base_mold_q4diag_*` (1.5mm, no-flange,
no-lock, flange-only) from `cast.base_mold.q4diag-*.toml`.

| Finding | Evidence |
|---|---|
| The cavity **floor is flat**, not tilted/lumpy. | Central-floor plane fit RMS **0.12mm**; the "tilt" I first reported was an artifact of including the rising rim fillet in the fit. |
| The visible "slope before the corner" is **two superimposed things**: (a) the **plug-lock socket** (a real 4mm-deep `SubtractTruncatedPyramid` at the cap centroid, ≈2mm from the fitted seam so it straddles/hugs the mating face), and (b) the **jagged MC junction** around it. | Socket trough z=−64.23 is **bit-identical at 3mm and 1.5mm** (a real CSG feature, resolution-independent) and **vanishes when `plug_pins` is disabled** (floor → flat to ~1mm). The remaining jaggedness is MC faceting. |
| The jaggedness **is resolution** (marching cubes), and it is **localized to the feature-dense junction**. | Same junction region: **255 faces @ 3mm → 935 @ 1.5mm** (3.7×), corners/chunks tighten — but the **flat-floor RMS is unchanged (125µm @ both)**. The flat floor is already below print resolution; only the *junction* improves with finer cells. |
| The **flange has zero effect on the cavity floor**. | Floor profile **bit-identical** with flange on vs off. (Rules the flange out of Defect 1.) |
| Orthogonal finding (NOT this recon, tracked): the scan floor is **mildly un-level (~2–4°)**. cf-scan-prep applied a partial level (`[transform.rotation]` roll 2.73° / pitch 3.33°) but the recorded cap normal is **still 4.3° off vertical** and the larger PCA auto-level (`auto_pca_quaternion`) is `applied = false`. | `base_mold.prep.toml`. A **cf-scan-prep leveling** issue (`CF_CAST_FLAT_FLOOR_RECON.md`), independent of the meshing architecture; flagged so it isn't lost. |

**Takeaway:** the floor is flat and the socket is a legitimate feature.
The defect is purely **how the mesher resolves the place where the
organic surface, the cap-plane floor cut, the seam cut, and the socket
boolean all crowd together** — and brute cell-size is the wrong knob
because it pays cubically across the whole part to fix one region.

---

## §MA-1 Root cause: blending + marching cubes

`compose_piece_solid` (`design/cf-cast/src/piece.rs`) builds the cup half
as **one implicit SDF field**:

```
base = CupWallShellSdf(body, wall_thickness)        // organic shell
         .intersect( ribbon.halfspace_solid(side) )  // SEAM cut  (exact plane)
         .union( flange.subtract(body) )             // FLANGE   (exact slab ∩ organic silhouette)
// body itself = pinned_floor_shell = organic ∩ cap-plane halfspace  → FLOOR cut (exact plane)
```

Then `solid_to_mm_mesh` (`design/cf-cast/src/mesher.rs`) turns that field
into triangles by:

1. building a dense **`ScalarGrid`** over the bounds at `cell_size_m`,
2. **sampling `solid.evaluate(p)` at every grid point** (this *bakes the
   composite SDF into a grid*), and
3. running **classic marching cubes** (`marching_cubes`, fixed per-cube
   topology + linear edge interpolation) on that grid.

The plug-lock socket, dowels, bolt holes, and pour gate are applied
**after** MC as exact **mesh-CSG** booleans (`apply_mating_transforms`).

So today the **manufactured cap-plane floor cut and the seam cut live
inside the blended field that gets baked and MC'd**, while the
socket/pins/bolts/pour are exact post-MC booleans.

**Why that produces the jaggedness — two compounding causes:**

1. **Blending.** At an organic∩exact junction the field has to be *smooth*
   along the organic surface and *razor-sharp* perpendicular to it, in the
   same region. Crammed into one field and sampled on a grid, the two
   demands fight.
2. **Marching cubes cannot represent a sharp feature inside a cube.** MC
   uses a fixed topology table and linear interpolation along cube edges;
   a corner that falls *inside* a cube is rounded off — by construction.
   Finer cells shrink the rounded band but never make it sharp. This is
   the same physics the geometry-crispness (§Q-1/§Q-5) and cap-plane
   (§Q4-1-a) arcs documented.

**Key architectural fact (corrected in cold-read):** cf-cast meshes with
**marching cubes**, even though cf-design **already ships dual contouring**
(`dual_contouring::mesh_field_dc`, QEF vertex per cell — which *can* place
a vertex at a sharp corner) and adaptive DC. cf-cast simply doesn't call
them; it has its own MC path. **That unused DC mesher is a first-class
lever** (§MA-6 B).

---

## §MA-2 The surface taxonomy — corrected by the workshop

An earlier draft split surfaces into "manufactured-exact" vs
"organic-smooth" classes to mesh separately. **The workshop rejected that**
— the classes are not separable. Almost every cup-piece surface is *both at
once*:

| Surface | Organic part | Exact part |
|---|---|---|
| cavity floor | the part bottom (scan) | clipped flat by the **cap plane** |
| seam / mating face | outline = body cross-section (scan) | the flat **seam-plane** cut |
| flange | inner edge follows the **silhouette** (scan) | flat **slab** ± `flange_thickness` |
| cavity wall | organic (scan) | ends at sharp edges where it meets floor + seam |
| mold outer | organic body **offset** (scan) | — |

The organic and the exact **interpenetrate**, and they **meet at the edges
and corners** — exactly where quality matters and where the jaggedness
shows. The requirement is organic fidelity **and** manufactured sharpness
**on the same feature, at the same edge.**

So the architectural lever is **not** "classify each surface." It is two
coupled choices:

- **How to compose** organic + exact: *blending* (one implicit field,
  meshed once — today) vs *composition* (mesh the organic shape, then apply
  exact cuts as **booleans**).
- **Which extractor** turns the field into triangles: **MC** (rounds
  corners — today) vs **DC** (QEF preserves sharp features — available,
  unused).

Under composition + a sharp-feature extractor:
- **exact ∩ exact** edges (floor-plane meets seam-plane — *the* sharp 90°
  corner) come out **mathematically exact**, sharp at any resolution;
- **exact ∩ organic** edges (floor meets cavity wall) come out as a sharp
  **crease that follows the organic surface**;
- the **organic surface itself** keeps whatever fidelity its mesher gave.

cf-cast already composes this way for the socket/pins/bolts (post-MC
booleans — *why* the socket faces are crisp while the floor around it is
jagged). The cap-plane floor and the seam are the cuts still trapped in the
blended-then-MC'd field.

---

## §MA-3 What the fix delivers — and what it relocates (honesty)

Neither composition nor DC is a free resolution win; each **relocates** the
cost:

- **exact ∩ exact corners** (floor↔seam, the workshop's complaint): solved
  by exact CSG (A), resolution-independent. ✓
- **exact ∩ organic edges** (floor↔cavity-wall): sharp crease, but the
  crease *line* follows the organic mesh — coarse organic mesh → a
  sharp-but-coarse polyline edge. Improved, not eliminated.
- **the organic surface's own fidelity** (cavity walls, canal texture):
  unchanged by CSG; this is where DC (sharper at equal cells) and/or finer
  sampling / mesh-direct earn their keep.

So "both" resolves to: **organic mesher for fidelity + exact CSG for the
manufactured cuts, composed** — and the *only* place that still spends
resolution is the genuinely-organic surface.

---

## §MA-4 Hard constraints any solution must satisfy

1. **Paradigm boundary** ([[project-cf-cast-sdf-meshcsg-paradigm-boundary]]).
   A post-MC boolean face **coincident** with the SDF surface is
   WELDED-TO-BULK → manifold3d topological ambiguity (the 2026-05-24
   failure). Cuts must be biased ~1µm into the interior to be CONTAINED
   (the shipped `PLUG_CAP_TRIM_BIAS_M` pattern, §Q-4 S1).
2. **The cup straddles the cap plane.** A *global* cap-plane halfspace cut
   **bisects** the cup (organic-arc S2, 2026-05-28: a 172mm half collapsed
   to a 23mm stub). The floor cut must be **bounded** to the cavity-floor
   region (a box/region op), not a halfspace. The **seam** cut, by
   contrast, *is* a clean halfspace (the cup is entirely one side) — a seam
   trim is safe.
3. **Determinism.** cf-cast guarantees bit-identical regen. Any extractor or
   composition change must be deterministic.
4. **Watertight + manifold + F4-clean.** Hard print requirement. **DC's
   manifoldness is the key risk** if we switch extractors — MC here is
   manifold by construction; a DC swap must prove watertight/manifold/F4 on
   the production parts. F4 is O(n log n) post-BVH
   ([[project-cf-cast-self-intersect-bvh-s1]]).
5. **Below-print-resolution = accept** precedent ((4'),
   [[project-cf-cast-cap-plane-flatness-bookmark]]): the Bambu A1 ~0.2mm
   slicer/print quantization bounds how far organic fidelity must go. The
   flat floor (125µm) is already there.

---

## §MA-5 Current architecture audit (corrected)

- **Extractor: classic marching cubes on a baked dense grid.**
  `solid_to_mm_mesh` (`cf-cast/src/mesher.rs`): build a `ScalarGrid` at
  `cell_size_m`, sample `solid.evaluate` at every point, `marching_cubes`,
  scale to mm. **Not** dual contouring. MC rounds sharp features by
  construction.
- **Per-grid-point cost = scalar SDF eval, no gradients.** MC needs only
  the scalar field at grid corners (linear edge interpolation). The
  dominant term is the body distance via **`SharedScanSdf`**
  (`Arc<Signed<TriMeshDistance, FloodFillSign>>` — **parry BVH** TriMesh
  closest-point + flood-fill sign), plus the SDF composition (shell
  max/min, halfspace, flange silhouette distance).
- **Grid bake is O(nx·ny·nz)** — cubic in `1/cell_size`. Halving the cell ⇒
  ~8× grid points ⇒ ~8× BVH queries. This is the most likely dominant
  regen cost (S0 must confirm); the MC pass itself is cheap.
- **cf-design has DC + adaptive DC, UNUSED by cf-cast.**
  `dual_contouring::mesh_field_dc` (QEF, sharp-feature-preserving) and
  `adaptive_dc::mesh_field_adaptive` (octree). Note: cf-design's DC is
  **field-based** — it re-evaluates the `FieldNode` (with finite-difference
  gradients for Hermite normals), so used naively it pays the SDF-eval cost
  *again* and more (the cost cliff a prior probe hit). A **grid-based DC**
  (read the already-baked `ScalarGrid`, derive normals from cheap grid
  central-differences) would get DC sharpness without that cliff — but does
  not exist yet.
- **`adaptive_dc` only prunes:** the octree prunes whole interior/exterior
  subtrees, but surface leaves sit at the uniform finest `cell_size`. It
  speeds same-resolution meshing; it does **not** refine *finer near
  features*.
- **Mesh-direct exists:** `scan_mesh_direct` pulls the layer-0 plug straight
  from the (already-fine) scan mesh. §SMD-4 S3 (cup-wall outer via
  `mesh_offset`) is bookmarked but unbuilt.
- **Post-MC mesh-CSG already in use:** socket (`SubtractTruncatedPyramid`),
  dowels + bolts (`SubtractCylinder`), pour gate. The §Q-4 S1 plug-bottom
  cap trim (`build_plug_cap_trim_transform`, +1µm bias) shipped in PR #258.

---

## §MA-6 Option space (evaluated against §MA-4; none chosen)

**A. Exact CSG for the manufactured cuts (floor + seam).** Move the
cap-plane floor cut (bounded box per constraint 2, 1µm-biased per
constraint 1) and re-evaluate the seam (halfspace `SeamTrim`) **out of the
blended field into post-MC booleans**. Floor↔seam corner → exact∩exact =
sharp at *any* cell size; floor↔wall → sharp crease on the organic mesh.
This is the §Q-4 planar-trim plan, **now correctly motivated** — not
"flatten the floor" (it's flat) but "make the cuts exact so their edges
stay sharp." *Pro:* directly fixes the workshop complaint;
resolution-independent; reuses the shipped paradigm-safe bias pattern;
small, contained. *Con:* only the manufactured cuts; organic fidelity
untouched; must solve the bounded-floor-cut geometry carefully.

**B. Switch the extractor MC → DC.** DC's QEF *can* place a sharp vertex at
a corner inside a cell, so it preserves the manufactured corners MC rounds
— at the *same* grid resolution. Two flavors: **B1** call cf-design's
field-based `mesh_field_dc` directly (re-evaluates the SDF + finite-diff
gradients — expensive, the cost cliff); **B2** a *grid-based* DC that reads
the already-baked `ScalarGrid` and derives normals from grid
central-differences (DC sharpness at ~MC cost — but it's new code). *Pro:*
helps every junction *and* organic curvature at once; B2 is cheap. *Con:*
manifoldness/watertightness/F4 risk (constraint 4); determinism; DC still
only resolves features to cell scale (a true 90° corner is best from A);
B2 must be written.

**C. Adaptive grid refinement.** The dense grid is the cubic cost; refine
the `ScalarGrid` (octree) *deeper near features* (high curvature /
sign-complex cells), coarse elsewhere — flat floor coarse, junction fine.
Pairs naturally with B (adaptive DC). *Pro:* "fine detail only where
needed" — the literal answer to the workshop's question. *Con:* most code;
manifold stitching across resolution boundaries; determinism; cf-design's
`adaptive_dc` today prunes but doesn't feature-refine, so this is an
extension.

**D. Mesh-direct for organic surfaces.** Pull the organic cavity/cup-wall
surface straight from the scan mesh (as the plug already does); apply the
manufactured cuts as booleans on top. *Pro:* full organic fidelity at ~zero
meshing cost; sidesteps resolution for organic parts; §SMD-4 S3 scoped.
*Con:* scan-mesh quality becomes the floor; offsetting/booleaning raw scan
meshes has robustness issues; interacts with cavity-inset layering.

**E. Brute-force finer uniform `cell_size`.** *Pro:* trivial. *Con:* cubic
grid-bake cost everywhere for a localized fix; MC never truly sharp; the
thing the workshop wants to avoid. **Baseline / fallback only.**

**F. Hybrid (likely the answer).** A for the manufactured cuts (sharp,
cheap) **+** B/C/D for organic fidelity where it is still implicit. The
composition principle lands here: stop MC-ing things that are exact (→ CSG)
or already fine meshes (→ mesh-direct); reserve the volumetric extractor
for the genuinely-implicit organic surface, and there use DC (sharper) on
a possibly-adaptive grid.

**G. Optimize the flange-silhouette extraction (NEW — the S0 #1 lever).**
The S0 profile (§MA-7) showed `Silhouette2d::from_body_in_plane` is ~70% of
production cup-piece time, as **fixed overhead independent of mesh
sharpness**. Three cheap, orthogonal wins: (i) **compute the silhouette once
per layer and share it across both `PieceSide`s** (it's the same body
cross-section — currently built twice); (ii) **decouple
`SILHOUETTE_GRID_STEP_M` from the over-fine 0.5mm** / scale it toward the
mold cell size (6× oversampled at 3mm); (iii) **derive the silhouette by
slicing the scan mesh at the seam plane** (an exact polyline from triangle
intersections — no SDF grid sampling at all). *Pro:* roughly halves the
regen with **zero geometry change**, and removes the fixed tax that makes
finer cells unaffordable — i.e. it directly funds "fine detail without
forever." Determinism-preserving. *Con:* (iii) needs robust mesh-plane
slicing; the fitted (tilted) seam basis must be threaded through.
**This is the highest value-per-effort item and is independent of the
sharpness fix (A) — they compose.**

---

## §MA-7 S0 PROFILE RESULTS (2026-05-30 — done) — where the time actually goes

Matched full-feature, canal-off regens at 3mm vs 1.5mm, plus a
throwaway-instrumented split of the per-piece `compose+MC` bucket into
{`compose_piece_solid`, grid-bake, MC, post-MC booleans}. Instrumentation
reverted (uncommitted, `[PROBE]` eprintlns in `mesher.rs`/`spec.rs`).

**Per cup piece (3mm):**

| phase | L0 | L1/L2 | scales with | notes |
|---|---|---|---|---|
| `compose_piece_solid` | **~14.1s** | **~37s** | body size + **fixed 0.5mm** | the silhouette extraction — see below |
| grid bake | 5.6s | 14.3s | **cubic in 1/cell** | `solid.evaluate` × grid points |
| marching cubes | ~0s | ~0s | — | negligible |
| post-MC booleans | 0.1s | 0.1s | — | socket/dowel/bolt/pour — negligible |
| F4 validation | 2.9s | 5.6s | ~faces (quadratic) | — |

**The dominant cost is `compose_piece_solid` (~70% at 3mm) — and it is the
`FlangeSdf` `Silhouette2d::from_body_in_plane` extraction.** It samples the
body SDF (`SharedScanSdf` parry-BVH) on a 2D seam-plane grid at
**`SILHOUETTE_GRID_STEP_M = 0.5mm`, FIXED** (independent of
`mesh_cell_size_m`), then marching-squares. Three damning properties:

1. **6× oversampled vs the mold** at 3mm cells — it samples the seam plane
   at 0.5mm to build a curve the 3mm mold mesh can't even resolve.
2. **Computed redundantly per side** — `build_flange_solid_for_side` calls
   `build_flange_solid` for *both* Negative and Positive, but the silhouette
   is the body cross-section, **identical for both halves**.
3. **It does not affect mesh sharpness at all** — it just builds the
   flange's lateral curve. It is pure fixed overhead.

**Grid bake is the *secondary* cost** (~30% at 3mm) and the one that scales
**cubically** with `1/cell_size` (5.6s → ~37s inferred from the 1.5mm
compose+MC of 51s minus the ~constant ~14s silhouette). It is what makes
*finer* cells expensive. MC itself and the post-MC booleans are free.

**Other:** funnel **F4 = 11s** is a serial outlier (validation, not
meshing). Peak RSS 527MB (3mm) → **2.6GB (1.5mm)** — memory becomes a real
constraint at fine cells. Wall: 86s (3mm) / 360s (1.5mm), ~2.7–3.2× rayon
parallelism (per-layer + per-side).

**What this means for the option map:**
- At **production (3mm) resolution**, the meshing-architecture options
  (A/B/C/D) address the *secondary* cost. The **#1 lever is the silhouette
  extraction** (new option **G** below): sharing it across sides + not
  over-sampling it (or deriving it from the scan mesh) roughly **halves the
  regen with ZERO quality change** — and it is the fixed tax that makes fine
  cells unaffordable today.
- For **fine cells specifically**, the **cubic grid bake** is the wall —
  so C (adaptive grid), D (mesh-direct), B2 (grid-DC) attack the right
  thing, *once the silhouette tax is removed*.
- **E (brute finer uniform) is the worst**: pays the cubic bake everywhere
  *and* the memory blows up (2.6GB at 1.5mm).

---

## §MA-8 Defect 2 belongs here too

The flange-perimeter **slivers / "vertical cracks"** (Defect 2, the other
PR-blocker) are stacked sub-1e-4mm² triangles + a few non-manifold edges at
the silhouette's high-curvature corners where `FlangeSdf` unions with
`CupWallShellSdf` in the blended field (`piece.rs:373`), then MC'd. **Same
root** as Defect 1 — MC of a feature-dense organic∩exact junction in the
blended field — at a convex perimeter instead of a concave corner. The
composition fix (take the flange union out of the blended field / bias it
CONTAINED) and/or a sharp-feature extractor should resolve it by
construction. **Solve both defects under one architecture, not separately.**

---

## §MA-9 Open questions

1. **Profile first (S0):** where does regen time actually go? (Gates the
   whole option choice.)
2. **How sharp does the workshop actually need it** — exact (CSG, A) vs
   "fine enough" (DC / finer MC)? Print is Bambu A1 ~0.2mm — does a
   sharp-crease-on-coarse-organic-edge print acceptably, or must the organic
   edge also be fine?
3. **Bounded floor-cut geometry:** what bounds the cap-plane floor box — the
   body lateral AABB at cap_z, the cavity silhouette, or the tilted-cap
   basis (the ~4° cap tilt)?
4. **Does the seam stay an SDF halfspace or become a post-MC `SeamTrim`?**
   (§Q4-3-c shipped the defensive-SeamTrim reasoning; revisit under
   composition.)
5. **DC swap feasibility:** can a (grid-based, B2) DC stay watertight +
   manifold + F4-clean + deterministic on the production parts, and compose
   with the existing post-MC mesh-CSG transforms?
6. **Mesh-direct vs adaptive** for organic fidelity, given scan-mesh
   quality?
7. **The orthogonal scan-leveling residual (~4°):** separate cf-scan-prep
   item (recommend) — does not block this arc.

---

## §MA-10 Proposed implementation arc

- **S0 — Profile. ✅ DONE (§MA-7).** Found: ~70% of production cup-piece
  time is the fixed flange-silhouette extraction (option G), grid bake is
  the secondary cubic cost, MC/booleans free, funnel-F4 an 11s outlier,
  memory blows to 2.6GB at 1.5mm. Reframed the priorities below. (Sharpness
  ceiling vs cell size: still worth a quick characterization in S1.)
- **S1 — Two parallel, independent tracks (both gate on workshop cf-view):**
  - **S1a — Silhouette optimization (G).** The highest value-per-effort:
    share the silhouette across sides + stop over-sampling / derive from the
    scan-mesh slice. Halves the regen with zero geometry change — verify
    bit-identical output + the wall-time win. Funds everything downstream.
  - **S1b — Composition spike (A).** Cap-plane floor cut (bounded box, 1µm
    bias) + seam as exact post-MC CSG; verify the floor↔socket↔seam junction
    goes crisp at the *current* 3mm cell size. Confirms composition fixes the
    manufactured sharpness without finer cells. (Carry the quick sharpness-vs-
    cell-size characterization here.)
- **S2 — Organic fidelity path (only if S1 leaves it needed).** With the
  silhouette tax gone (S1a), finer cells / DC become affordable: DC swap
  (B2 grid-DC) and/or mesh-direct (D) and/or adaptive grid (C). Scope set by
  Q2 (how fine the organic surface must actually print).
- **S3 — Defect 2** falls out of S1b/S2 (same junction class); verify
  slivers gone.
- **S4 — Production regen + workshop cf-view + print gate**; cold-read;
  commit; status → FIXED.

Each phase gates on the workshop's cf-view (ultimately a physical
print/cast). Per [[feedback-correctness-over-speed]] — recon→cold-read→
spike→validate, no rush.

---

## §MA-11 Prior-arc memory checklist (read before any phase)

- [[project-cf-cast-sdf-meshcsg-paradigm-boundary]] — CONTAINED /
  PROTRUDING / WELDED-TO-BULK + the 1µm-bias rule. Governs every CSG cut
  (A). Rule #6: do not revert the 2026-05-24 disabling without the bias.
- [[project-cf-cast-cap-plane-flatness-bookmark]] — the (4')
  below-print-resolution accept precedent; the cap-plane × organic-edge MC
  corner physics.
- [[project-cf-cast-geometry-crispness-q1-fix]] / -q5 / -s0 — the
  `CupWallShellSdf` refactor, MC-winding fix, and §Q-11's origin.
- `docs/CF_CAST_Q4_PLANAR_TRIM_RECON.md` + [[project-cf-cast-q4-mating-face-quality-active]]
  — option A *is* the §Q-4 plan, re-motivated; S1 (plug cap trim) shipped.
- [[project-cf-cast-canal-interior-recon]] / `CF_CAST_ORGANIC_PARTS_RECON.md`
  — straddle-the-cap-plane (constraint 2), fitted seam, apex pour, canal
  0.5mm pass.
- [[project-cf-cast-self-intersect-bvh-s1]] — F4 cost for heavier meshes.
- `CF_CAST_FLAT_FLOOR_RECON.md` — the orthogonal scan-leveling residual
  (§MA-0), tracked, out of scope.

---

## §MA-12 Cold-read corrections (2026-05-30)

The first draft asserted cf-cast meshes via **dual contouring** (reasoning
from `Solid::from_sdf` → `Solid::mesh_dc`). **Wrong.** cf-cast meshes via
its own `solid_to_mm_mesh` = **bake `ScalarGrid` + classic marching
cubes**; `mesh_dc`/`adaptive_dc` exist in cf-design but are **unused** by
cf-cast. Consequences fixed throughout:
- jaggedness = **MC** corner-rounding (not a DC QEF limit);
- "baked SDF grid" is **already done** (the lever is *adaptive* grid +
  *which extractor*, not "bake a grid");
- MC needs **no gradients** — the per-point cost is one scalar
  `SharedScanSdf` (parry-BVH) eval, so the earlier "finite-diff gradient
  cost" was wrong; the real cost is the **cubic grid bake**;
- a **new option B** surfaced: switch the extractor MC→DC (cf-design DC
  exists, unused) — preserve sharp features at the same resolution.

This is exactly the class of error a cold-read catches: an architecture
claim inferred from a plausible call path that the production code doesn't
actually take. S0's profile will further ground the cost model.

---

## Successor

S0 (profile + sharpness-ceiling characterization) picks up next — the
cheapest, most decisive first step; it converts "which option" from
argument into data and ships no production code, matching the workshop's
deliberate, correctness-first pacing.
