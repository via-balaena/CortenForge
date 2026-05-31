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
  - **S1a — Silhouette optimization (G). ✅ PARTLY DONE (§MA-13).** Sharing
    across sides SHIPPED (`06c0c8d7`), bit-identical, −27% CPU but **0% wall**
    (the redundant silhouette was on the *parallel* path, not the critical
    path). The over-sampling cut was SPIKED and **rejected** — it does move
    wall (−27…−35%) but spends ~2.4–2.7 mm of organic fidelity at the
    seam/floor high-curvature region (Defect-1 zone). See §MA-13.
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

## §MA-13 S1a results + silhouette-cost spike (2026-05-30)

Two measured results landed this session. Validation rig: ORIG vs NEW
release binaries (NEW = the S1a commit), production canal config +
`cast.base_mold.canaloff.toml` (production-identical except canal off, so
the cup-piece silhouette path is fully exercised but the serial 0.5 mm
canal-plug MC doesn't mask it). Machine: 8 physical / 12 logical cores.
Byte-diff = `cmp` per STL + aggregate `md5`; geometry delta = numpy
point-to-surface (dense baseline sampling, not vertex-to-vertex — the
latter overstates deviation on the flat flange via re-triangulation).

### S1a — share the silhouette across sides (SHIPPED `06c0c8d7`)

`compose_piece_solid` split into `compose_piece_shared` (side-invariant:
MC bounds + un-split flange `Solid` w/ silhouette + side-agnostic post-MC
transforms) + `compose_piece_with_shared` (per-side shell ∩ halfspace ∪
flange). `mesh_and_gate_v2_pieces` builds `shared` ONCE per layer above
the `rayon::join`. Note the redundancy was broader than §MA-7 framed:
**flange + dowel + bolt** each rebuilt their silhouette *per side* (6 per
layer where 3 suffice); all take side-independent inputs.

| metric | ORIG | NEW (S1a) | Δ |
|---|---|---|---|
| STL bytes (canal + canal-off) | — | — | **0 — bit-identical (12/12 ×2, md5 match)** |
| lib tests | 304/0 | 304/0 | — |
| CPU `user` (canal-off, 3-layer, 3 mm) | 321.6 s | **236.2 s** | **−27%** |
| wall `real` | 101.4 s | 102.7 s | **≈ 0 (+1.3%, noise)** |
| realized parallelism | 3.17× | 2.30× | — |

**Why CPU drops but wall doesn't (the load-bearing finding):** a single
cup half's chain is `silhouette → bake → MC`, run serially within the
side; the two sides ran *in parallel*, each redundantly building the
silhouette. With 6 cup-piece tasks on 8 cores the duplicate ran on an
otherwise-idle core — costing CPU but **free in wall-clock**. S1a moves
the silhouette to a serial-once step before the join, so the per-layer
critical path is now `silhouette(~36 s, L1/L2) + bake+MC(~14.5 s) ≈ 51 s`
— **identical to ORIG's single-side ~51 s**. The redundant silhouette was
on the *parallel* path, not the critical path. (The §MA-7 "share across
sides → ~halve the regen" framing implicitly assumed core saturation;
with spare cores it's a CPU win only.) Still worth shipping: bit-identical,
−27% CPU (helps the serial-ish grade-all/CI backstop), and the
"compute-once" structure every downstream wall lever builds on.

### Silhouette over-sampling cut (G-ii) — SPIKED, rejected for now

Swept `SILHOUETTE_GRID_STEP_M` (0.5 → 1 → 2 → 3 mm) on canal-off, post-S1a.
This *does* shorten the critical path (it cuts the silhouette cost itself):

| step | wall `real` | vs 0.5 mm | F4 criticals |
|---|---|---|---|
| 0.5 mm (today) | 102.7 s | — | 0 |
| 1 mm | 74.8 s | **−27%** | 0 |
| 2 mm | 68.5 s | **−33%** | 0 |
| 3 mm | 67.0 s | **−35%** | 0 |

Knee at ~2 mm; F4-clean throughout (Defect-2 not pushed to failure).

**But it is NOT geometrically free.** Point-to-surface deviation vs the
0.5 mm baseline (cup pieces): typical rms **0.1–0.3 mm**, p95 **0.2–0.6 mm**
(near the Bambu ~0.2 mm print floor) — *but* localized maxima of
**~2.4–2.7 mm even at the 1 mm step**. Localized: the worst vertices
cluster at **z ≈ −73 mm, x≈0, y≈0 — the part's floor end at the
seam/centerline** (+ some flange-perimeter extremes), i.e. the
high-curvature end of the body cross-section, **overlapping the Defect-1
junction** S1b targets. The "below mold resolution anyway" escape **does
not hold**: silhouette rounding is a *curvature-scale* change the 3 mm
mold MC faithfully preserves, not something the mold quantizes away.

This is the §MA-3 "neither composition nor DC is free — each *relocates*
cost" prediction in the data: the over-sampling cut buys ~30% wall by
spending organic flange/seam fidelity at exactly the corner we want crisp.

**Decision:** ship S1a; **do not ship the over-sampling cut** (it degrades
the seam/floor region above print resolution, conflicting with S1b's
crispness goal). The cheap spike earned its keep by telling us *not* to
spend this lever. Speed should instead come from the **bake/extractor**
half of the critical path (S2: adaptive grid / grid-DC), where the
trade-off doesn't touch organic fidelity. A conservative
silhouette-step-tied-to-cell-size (~1 mm, −27% wall, ~2.4 mm worst-case)
remains available behind a workshop cf-view gate if a quick win is wanted.

---

## §MA-14 Defect 2 — flange-perimeter slivers ("vertical cracks") (2026-05-30 #4)

Workshop cf-view flagged vertical slits/cracks running the full height of
the flange. This section is the focused recon for the fix (S1b is shipped;
Defect 2 is the remaining PR-blocker). Per [[feedback-correctness-over-speed]]:
recon → cold-read → diagnostic spike → Option-1 implementation, each gated
on workshop cf-view.

### Confirmed (re-probed on the production canal cup pieces, `72496d3e`)

Binary current, tree clean, `flat_cavity_floor=true` (S1b) in the config.
`/tmp/sliver_probe2.py` + `/tmp/sliver_geom.py`:

| piece | slivers <1e-3 mm² | <1e-4 | span | r |
|---|---|---|---|---|
| L0 p0 | 20 | 11 | z[−61, 92] | [2.6, 38.3] |
| L2 p0 | 37 | 10 | z[−64, 101] | [6.2, **54.9**] |
| L2 p1 | 31 | 10 | z[−79, 100] | [5.0, **54.9**] |

Geometry of the defect (L2 p0):
- **24/37 slivers are vertical walls** (|normal·z| < 0.2, normals point
  radially in XY) → a thin **vertical groove**, not a hole.
- Edge lengths: short **0.023 mm**, mid/long ~0.06 mm — sub-0.1 mm knife
  slivers; several are **fully degenerate** (all three verts identical).
- They ring the **whole silhouette perimeter** at large radius (r out to
  54.9 mm = the flange outer reach `flange_width_m`=20 mm), full height.
- **MANIFOLD** (0 non-manifold edges → F4 passes). Structural defect — a
  weak point + bolt-clamp leak path — not a topological hole.

### Diagnosis (same class as Defect 1, convex perimeter not concave corner)

`piece.rs` composes the flange **in the blended SDF field**:
`base_piece = (CupWallShellSdf ∩ halfspace).union(flange_unsplit ∩ halfspace ∖ body)`
then marches-cubes the whole thing. `FlangeSdf::eval = outer.max(inner).max(vertical)`
is a flat slab in the seam plane: inner edge at `body_dist = flange_inner_offset_m`
(**default 2 mm — a deliberate gasket-clearance gap outboard of the body
silhouette**), outer edge at `flange_width_m`=20 mm, half-thickness
`flange_thickness_m`=4 mm. The `min()` union of the flange's flat faces
against the **rising organic cup-wall shell surface** near the silhouette
curve is where two surfaces graze near-tangentially around the whole
perimeter → MC emits a string of degenerate slivers along the crease (a
"crack"). Identical root to Defect 1 (MC of a feature-dense organic∩exact
junction in the blended field); here at a convex perimeter line rather than
the concave floor corner.

### Spike options (cheapest → heaviest)

1. **CONTAINED bias (recommended first).** Keep the flange in SDF/MC
   (paradigm-correct — the flange genuinely *welds to bulk* with the
   cup-wall, so it belongs in SDF per
   [[project-cf-cast-sdf-meshcsg-paradigm-boundary]]), but **bury the
   junction** so the flange overlaps the cup-wall volumetrically instead of
   grazing its surface. Likely lever: extend the flange inner edge *inward*
   into the cup-wall bulk (reduce/negate `flange_inner_offset_m`, or add a
   dedicated inward overlap-bias) so the flange's faces emerge from inside
   solid material, not tangent to the rising shell surface. Direct analog of
   S1b's 1 µm floor biases. **Flag-gated, default-off, byte-identical off.**
2. **Post-MC mesh boolean union.** The **coincident-face landmine**: per the
   paradigm-boundary framework this is precisely what produced the
   funnel/seam/plug-shaft disconnections (union on coincident faces → two
   components). The flange↔cup-wall seam would be face-coincident. Only if
   Option 1 can't bury the tangency, and then only with an overlap-bias that
   re-converts it to CONTAINED — at which point it offers little over 1.
3. **DC extractor.** S2-class, heavy; deferred.

### Open questions for the diagnostic spike (do FIRST, before any fix)

- **Which two surfaces kiss, and at what angle?** Sample `FlangeSdf` +
  `CupWallShellSdf` at a known sliver location (e.g. one of the degenerate
  triangles above) and walk the field to confirm the flange-face ↔
  shell-surface grazing and measure the dihedral. Determines the *minimal*
  bias direction (inward lateral overlap vs seam-normal thickness vs both).
- **Does the `flange_inner_offset_m`=2 mm gasket gap matter?** The gasket is
  disabled in iter-1 — confirm, so reducing/overlapping the inner edge
  inward is safe.
- **Is the crack at the inner edge, the faces, or both?** The vertical-wall
  normals (radial) suggest the lateral inner-edge/face crease; confirm.

### Cold-read confirmations (2026-05-30 #4)

- **Flange is on by default**: `FlangeConfig::default` = `enabled=true` with
  `FlangeSpec::iter1()` (width 20, thickness 4, inner_offset 2 mm). base_mold
  has no `[flange]` block → these defaults (consistent with r→54.9 mm slivers).
- **Gasket is OFF** for base_mold (`[gasket] enabled=false`). So the
  inner_offset's gasket-clearance purpose is moot for this part →
  **extending the inner edge inward is safe here**. The fix must still be
  gasket-aware in general (there's a config cross-field invariant that the
  flange clears the gasket channel when both are on — recon §F-4); flag-gated
  + the bias only active for this part keeps that clean. Validation runs
  gasket-off, so no channel-pinch risk.
- **The flange is `∖ body` clipped** (`piece.rs:540`
  `flange.subtract(layer_body)`). So a negative `flange_inner_offset_m` does
  **not** push flange material into the body cavity — the body-subtract caps
  it at the body surface. Caveat the spike must check: that body-surface cap
  could relocate the graze to the body surface rather than eliminate it; the
  shell is *also* body-subtracted there, so they'd share a clean cut, but
  confirm empirically. The minimal safe bias may be a small inward overlap
  (inner edge a hair inside the shell's lateral reach, not all the way to the
  body) — the diagnostic spike sizes it.

### S3b diagnostic spike RESULTS (2026-05-30 #4) — Option 1 OVERTURNED

Throwaway `#[ignore]` test `piece::tests::ma14_sliver_bias_sweep` (delete
after Defect 2). Synthetic **sphere** body (r 20 mm — no flat caps, so the
*only* slivers are the flange↔cup-wall interaction; the earlier cylinder
fixture's 36 end-cap artifact slivers swamped the signal), wall 5 mm, cell
3 mm, count = mesh faces with area < 1e-3 mm². Findings:

1. **Flange is the cause.** Flange OFF → 0 slivers; flange ON → many. ✓
2. **`flange_inner_offset_m` (lateral inward overlap) has ZERO effect** —
   swept +2 → −4 mm, sliver count unchanged. **Option 1 (the approved
   inward-overlap CONTAINED bias) is dead** — the graze is not at the inner
   edge.
3. **`flange_thickness_m` is non-monotonic + cell-size-periodic** — at an
   axis-aligned seam, the flat flange FACE (a plane at ±thickness) lands
   on/off the 3 mm MC sample grid: th 4 mm → 128 *exactly-degenerate*
   slivers (area ~1e-29), th 3/4.5/5/6 mm → 0, th 7 mm (= 4+cell) → 208.
   A grid-coincidence resonance, not a usable lever.
4. **Tilt is decisive.** Adding a fitted-seam tilt (production uses an
   apex-anchored *tilted* seam, not axis-aligned) collapses the catastrophic
   axis-aligned resonance to a **handful (1–6) of tiny near-degenerate
   slivers, min area 6e-6…6e-4 mm², INSENSITIVE to both thickness and
   inner_offset** — matching production's *nonzero* character (min 4.7e-7).
   The sphere's short equator gives ~6; the production body's long,
   high-curvature silhouette (z[−64,100]) scales that to ~37.

**Production sliver character (re-probed, `/tmp/degen_probe.py`):** the 37
are **thin** triangles (3 distinct verts, **median min-edge ~20 µm**, max
edge up to mm) — NOT collapsed. (The earlier "3 identical verts" read was
2-decimal print rounding.) They render as cracks in cf-view (a thin sliver
has an ill-defined normal → shading discontinuity) but are **manifold,
F4-clean, ~20 µm = well below the 200 µm Bambu print floor**.

**Revised mechanism:** the slivers are an intrinsic MC artifact of the
**tilted flat flange faces grazing the axis-aligned MC grid at a shallow
angle** — same MC-junction family as Defect 1, but neither inner-offset nor
thickness biases it away (a tilted face crosses every grid plane; you can't
align it out). The recon's original Option-1/Option-2 framing is superseded.

### Revised fix options (post-spike)

- **R1 — Post-MC sliver cleanup (NEW, cheapest).** Run an in-tree
  `mesh-repair` pass (`remove_degenerate_triangles` /
  `degenerate_aspect_ratio` + `degenerate_min_edge_length` + `weld_vertices`)
  on the cup-piece mesh to collapse the ~20 µm slivers on the flat flange
  face. Manifold-preserving, mesh-level (no SDF-composition change →
  paradigm-safe), directly removes the cf-view cracks, changes no
  above-print-resolution geometry. Risk: must NOT weld/damage the
  bit-precise mating features (pin/socket/seam) — needs a conservative
  threshold scoped to true slivers (high aspect + sub-print min-edge), and
  re-validate F4 + mating-face flatness after. Flag-gated default-off.
- **R2 — DC / sharp-feature extractor (S2-class, root cause, heavy).**
  Meshes the flange faces without grid-aligned slivers. Broad change;
  belongs with the S2 cup-wall-resolution work.
- **R3 — Accept as below-print-resolution ((4') precedent).** They're 20 µm
  ≪ 200 µm; the cast cannot show them. But the workshop is *actively*
  objecting via cf-view, and (4') requires workshop-acceptance precedent, so
  this is weak unless paired with a cf-view-side don't-render-degenerate fix.

### R2/DC feasibility spike RESULTS (2026-05-30 #4) — R2 does NOT deliver

Throwaway `#[ignore]` test `piece::tests::ma14_dc_feasibility` (delete after
the decision). Meshes the SAME tilted-sphere cup field with MC (today) vs
cf-design's `Solid::mesh_dc` (the only DC available; cf-cast uses neither
today), measuring the three make-or-break questions:

| mesher | verts | faces | slivers<1e-3 | watertight (boundary edges) | near-seam vtx scatter (rms) |
|---|---|---|---|---|---|
| MC 3 mm | 1954 | 3904 | **2** | **0 ✓** | **3.6 µm** |
| DC tol 6 mm (0.15 s) | 353 | 796 | 0 | **40 ✗ (holes)** | 9.0 µm |
| DC tol 3 mm (0.90 s) | 1955 | 3908 | **3** | 0 ✓ | 13.4 µm |
| DC tol 1.5 mm (5.0 s) | 7978 | 15952 | **10** | 0 ✓ | 8.4 µm |

Three strikes, all against R2:
1. **DC does NOT eliminate the slivers.** At matched resolution (tol 3 mm,
   ~same face count) DC yields **3** slivers vs MC's 2 — *more*, not fewer;
   finer DC yields 10. DC's QEF vertex placement + quad→triangle emission on
   the tilted flat flange faces sheds its own thin triangles. The premise
   ("sharp-feature DC → no grid-alignment slivers") does **not hold** for
   this field — DC would itself need the R1 weld to clean up, so it buys
   nothing for Defect 2.
2. **DC regresses the seam mating face.** Near-seam vertices scatter ~3–4×
   more than MC (rms 13.4 µm vs 3.6 µm at matched resolution) — exactly the
   §F-4 risk: MC lands seam vertices flat because it linearly interpolates
   the linear halfspace SDF; DC's QEF minimization does not, so the 1 µm-flat
   mating face (the surface that *seals the mold*) would degrade.
3. **DC is watertight-fragile.** Coarse DC (6 mm) emits 40 boundary edges
   (holes) → would fail F4; only finer tolerances close up.

Caveat (honest): this is cf-design's *uniform* `mesh_dc` out of the box, not
a bespoke grid-DC with seam-plane snapping (recon's "B2"). A purpose-built
extractor *could* in principle snap seam verts + special-case the flange —
but that's a research project (recon MA-9 #5 open questions), not an
available lever, and it still wouldn't auto-kill the flange-face slivers
(strike 1 is intrinsic to dual vertex placement on tilted planes). The spike
tested what actually exists, and it does not solve Defect 2.

**Decision: proceed with R1 (weld-only sliver cleanup).** The DC "root fix"
intrigue is refuted by data — DC doesn't fix the slivers, regresses the
mating face, and is watertight-fragile; it would need R1's weld anyway. R1 is
the correct, validated, reversible fix (when/if a future bespoke extractor
lands as S2, the weld becomes a no-op and is deleted). Per
[[feedback-correctness-over-speed]] — the spike converted "is the foundational
fix better?" from intuition into a measured *no* before betting the
mating-face precision on it.

### R1 implementation attempts — ALL FAILED (2026-05-30 #4, §MA-14 S3d)

Validated on the real production part (`cast.q4weld_on.toml`, canal-off A/B vs
`cast.q4weld_off.toml`). The weld-OFF baseline confirms the slivers are
present (39–78 per cup piece) **and the STLs pass F4** (manifold, watertight —
written successfully). Then every mutation tried to remove them failed:

| attempt | result |
|---|---|
| **weld_vertices 30µm, POST-CSG** | F4 FAIL — 5 non-manifold edges + 10 winding-inconsistent + self-intersections. The blind spatial weld merges vertices across the tight mating-feature walls (socket / pour bore / bolt+dowel). |
| **weld_vertices 30µm, PRE-CSG** | manifold3d REJECTS the welded raw mesh (`NotManifold`) — the weld creates an edge shared by >2 faces on the complex production flange, so the CSG stage can't even ingest it. |
| **`Manifold::simplify`** (manifold-safe) | Stays watertight, but does **NOT clear the slivers**: 2→1 on the sphere, never 0, even at 200µm tolerance (the print floor). simplify optimizes vertex count under an error bound; it does not target high-aspect slivers, and leaves the stubborn ones at the flange-edge corner. |

**Root insight:** `weld_vertices` is topology-unaware (clears slivers, breaks
manifold); `simplify` is manifold-safe (keeps manifold, doesn't clear slivers).
Neither is a manifold-preserving *targeted sliver collapse*. mesh-repair has no
such primitive.

### The reframe — what these slivers actually are

The weld-off STLs **pass F4**: manifold, watertight, every edge shared by
exactly two faces. So the slivers are **not a structural defect / leak path**
(a watertight surface has no gap) — they are **~20 µm thin triangles on a
sound, sub-print-resolution surface** that *render* as "cracks" in cf-view
because a thin triangle has an ill-defined normal (shading artifact). The open
empirical question that should drive the decision: **does the slicer actually
choke on them when printing, or is it only a cf-view display artifact?** The
workshop flagged them in cf-view, not (yet) from a failed slice.

### Remaining real options (none is the "cheap fix" first imagined)

- **R3 — Accept + document** (the (4') below-print-resolution precedent): the
  STLs are F4-clean + 20 µm ≪ 200 µm print floor. Pair with a cf-view-side
  "don't shade degenerate triangles" fix so the workshop stops seeing
  "cracks." Cheapest + safest; contingent on the slicer not choking.
- **R-collapse — purpose-built manifold-preserving sliver edge-collapse**: a
  new routine that collapses only the short edge of high-aspect triangles
  *with link-condition checks* (unlike blind weld) so it stays manifold. The
  technically-correct removal, but it's new engineering (mesh-repair lacks it).
- **R-flange-CSG — compose the flange as a post-MC exact mesh-CSG union** of a
  clean extruded-prism flange (CONTAINED-biased per the paradigm-boundary
  framework) instead of blending it into the MC field — removes the slivers
  *at the source* (the flange faces are never MC'd). Real surgery + paradigm
  risk.
- **R2 / bespoke extractor** — refuted for this defect (above); S2/future
  (`docs/CF_CAST_SHARP_FEATURE_EXTRACTOR_RECON.md`).

**Status: BLOCKED on a decision.** The naive R1 weld is dead; the proportionate
path depends on whether the slivers are a print problem or only a display
problem — an empirical (slice/print) question for the workshop.

### THE ROOT — it's a geometric feather-edge, NOT a meshing artifact (2026-05-30 #4)

Workshop loaded the cup into Orca and **confirmed the flange slit goes ALL
THE WAY THROUGH** — a real structural gap that prints, not a cf-view shading
artifact. (A watertight 2-manifold *can* pinch to a near-zero-thickness web:
F4 passes, but the web is sub-printable, so it prints as a gap. The earlier
"watertight ⇒ cosmetic" reasoning was WRONG.)

The decisive test — MC cell-size sweep on the tilted-sphere cup (if it were a
meshing artifact, finer cells would *fix* it):

| MC cell | slivers | thinnest web |
|---|---|---|
| 3.0 mm | 2 | 2.57 µm |
| 1.5 mm | 50 | 2.43 µm |
| 0.8 mm | 328 | 1.46 µm |
| 0.5 mm | 2056 | **0.00 µm** |

Finer cells make it **worse**, and the web thickness stays ~0 at every
resolution. **Conclusion: the SDF solid itself necks to zero thickness** — a
flat flange slab unioned with the curved cup-wall **inherently feathers to a
zero-thickness knife-edge** where the flat face grazes the curve (geometric,
resolution-independent; the flange-thickness/inner-offset biases of §MA-14 S3b
can't fix it in the tilted case). MC at any resolution faithfully reproduces a
~zero web; finer cells just lay more degenerate triangles along it.

**This invalidates the entire meshing-side approach** (R1 weld/simplify, R2
DC, finer cells) — none can add thickness the geometry doesn't have. **The fix
must be GEOMETRIC**: the flange↔cup-wall junction must be composed so it never
feathers — guaranteed finite (≥ printable) thickness. Candidates for a fresh
recon→spike:

- **Filleted/gusseted root** — thicken the cup-wall→flange transition (a
  finite-radius fillet or a soft-min union) so the junction is never thinner
  than ~1 mm, instead of a sharp flat-on-curve tangent.
- **Clip the flange to its full-thickness region** — emit flange material only
  where the flat slab is finite-thickness-outside the cup-wall; let the
  cup-wall itself carry the seam inboard of that. Risk: a notch where they meet.
- **Flange as a clean post-MC CSG solid** welded to the cup-wall with a
  guaranteed-overlap (CONTAINED) join — no flat-on-curve tangent in the field.

**R1 weld code/flag to be reverted** (it's a dead end for a geometric defect).
The §MA-14 arc pivots from "meshing/weld" to "flange-junction geometry."

### Plan (revised)

- **S3a — §MA-14 recon + cold-read. ✅ DONE.**
- **S3b — diagnostic spike. ✅ DONE — overturned Option 1 (see above).**
- **S3c — re-decide the fix with the workshop** (R1 / R2 / R3). ← CHECK-IN.
- **S3d — implement chosen fix, flag-gated default-off**; validate flag-off
  byte-identical no-op (md5 across all STLs), flag-on slivers gone
  (`sliver_probe2.py` → ~0) + bbox/mating-face unchanged + F4-clean. Gate
  before/after STL on workshop cf-view.
- **S3e — production regen + cf-view + commit.**
- **S3 OUTCOME (2026-05-30 #4): all meshing-side fixes DEAD; R1 reverted.**
  Root is a geometric feather-edge (above). Arc pivots to §MA-15.

---

## §MA-15 Flange-junction feather fix (geometric) — ACTIVE

The §MA-14 root: a **flat flange slab unioned (hard `min`) with the curved
cup-wall shell feathers to a zero-thickness knife-edge** where the flange's
flat face grazes the curved shell tangentially — a real sub-printable web that
prints as a through-slit. Resolution-independent; no meshing fix can add the
missing thickness. The fix must make the junction **finite-thickness by
construction** (target ≥ ~1 mm, the FDM min-wall floor).

### Candidates

1. **Smooth-union the flange into the cup-wall (LEAD).** Replace the hard
   `shell.union(flange)` with `shell.smooth_union(flange, k)` (cf-design has
   it; max material correction `k/4`, blends only in the junction region,
   matches sharp far away). The smooth-min **fills the feather with a fillet
   of finite radius** → the junction can't taper below the blend thickness. A
   filleted clamp-flange root is also *stronger*, not just printable.
   - **Apply the blend in 3D BEFORE the seam halfspace cut** (`smooth_union`
     then `∩ halfspace ∖ body`), so the seam mating face stays a clean flat
     cut of the blended solid — protects §F-4 1 µm flatness (a blend applied
     *after* the cut would round the mating-face edge).
   - Risk: the fillet adds a little material at the flange↔wall transition
     (k/4) and could round the seam edge if applied wrong; both measured in
     the spike. Pick the smallest `k` that lifts the thinnest web ≥ 1 mm.
2. **Clip the flange to its full-thickness region** — emit flange only where
   the slab is finite-thickness-outside the shell; let the cup-wall carry the
   seam inboard. Risk: a notch/discontinuity where flange meets wall.
3. **Flange as a clean post-MC CSG solid** welded with guaranteed (CONTAINED)
   overlap — no flat-on-curve tangent in the field. Heaviest; paradigm risk.

### Spike plan (tilted-sphere fixture, the §MA-14 reproduction)

Sweep `smooth_union` `k`; per `k` measure on the composed cup mesh:
- **thinnest web** (min sliver altitude) — target **≥ 1 mm** (the slit is gone)
- **slivers** <1e-3 mm² → ~0
- **watertight** (boundary edges = 0) + manifold
- **seam mating-face flatness** (near-plane vertex rms) — must stay ~MC's
  baseline (≤ ~10 µm; §F-4)
- **bbox / added material** — bounded by `k/4`

Decision: smallest `k` clearing the web while holding the mating face. If
smooth-union can't hold the mating face → candidate 2/3.

### LOCALIZATION CORRECTION (2026-05-30 #4) — it's the SEAM cut, not the flange

The smooth_union candidate is DEAD (web stays 2.6 µm at all `k` — a feather is
a thin *convex fin*, and smooth-min only fills *concave* creases). The
decomposition put the fixture's feather in `flange ∩ halfspace`, but the
**sphere fixture mis-localized it** (no pour/floor, trivial wall).

Classifying the REAL production slits (`q4_weld_off` L2 p0) against the fitted
seam plane (normal `[0.838,0.546,-0.001]`) is decisive: **17 of 19 visible
through-slits sit at |dist-to-seam| = 0.2 mm — i.e. ON the seam/mating-face
plane**, spanning the whole piece (r 7→55 mm, z −60→+100). The through-slit is
the **flat SEAM cut meeting the organic cup-wall/flange surfaces tangentially**
— the *same class as the cavity-floor feather (Defect 1)* that §MA-S1b fixed
with exact post-MC CSG, just at the seam instead of the floor.

Today the seam is cut **in the SDF/MC field** (`ribbon.halfspace_solid` ∩,
pre-MC). Where that flat halfspace grazes the rising organic walls, MC feathers
the boundary to a sub-µm web → the printable through-slit. **This is recon
§MA-9 open question #4** ("does the seam stay an SDF halfspace or become a
post-MC trim?").

**Lead fix (mirrors S1b): exact post-MC seam trim.** Mesh the full organic
piece (shell ∪ flange, *no* SDF seam cut), then trim at the seam plane with
manifold3d `trim_by_plane` (already used in-tree, `MatingTransform::SeamTrim`)
→ a clean planar mating face with a clean boundary polygon, no feather.
Paradigm caveat ([[project-cf-cast-sdf-meshcsg-paradigm-boundary]] failure #1):
a post-MC seam *slab* once coincided with the body cavity → thin film; a
half-space `trim_by_plane` (not a slab) keeping one side avoids that, and S1b's
floor CSG is the working precedent. Spike must confirm the trimmed mating face
stays §F-4-flat + watertight + composes with the existing post-MC features.

### PINNED ROOT (2026-05-30 #4, real-geometry probe) — flange↔cup-wall disconnection

Probed the REAL composed cup SDF (`CF_MA15_PROBE` env hook in `cli::run`,
canaloff layer 2 Negative, `evaluate` sampled as ASCII slices) — the sphere
fixture mis-localized this. Findings:

- **Seam-normal transects are clean everywhere** (one solid run → empty); the
  slit is NOT along the seam normal, and the inboard cup-wall slivers (r21)
  are benign MC artifacts on a clean-solid wall.
- **The real through-slit is at the FLANGE, and it's a void between two
  sub-solids.** XY overlay slices (`#`=cup, `o`=body cavity, `·`=void) at
  z=57.8 AND z=20 both show the cup-wall shell (hugging the body `o`) and the
  flange plate **separated by a widening empty `·` gap** — full-height.

**Cause:** the `FlangeSdf` is built from the **2D seam-plane silhouette**
(`Silhouette2d`, + `flange_inner_offset_m`=2 mm), while `CupWallShellSdf`
tracks the **3D body surface**. The two coincide only *at* the seam plane, so
flange and cup-wall overlap in just a thin band there; where the organic body
curves away from the seam plane, that overlap **pinches to zero and the flange
detaches from the cup-wall** → a vertical through-slot. (A sphere is convex/
smooth everywhere, so its flange↔wall overlap never pinches — why the proxy
showed only mild flange-edge slivers, not this disconnection.)

This is a **connection failure between two unioned sub-solids**, not a feather
of a single surface — which is why smooth_union (concave-crease filler),
post-MC seam-trim, weld/simplify/DC all missed it.

**Fix LEVER CONFIRMED on real geometry — flange THICKNESS, not inner-offset.**
Real-geometry fix-tests (re-derive ribbon + recompose, re-map the z=20 slot):
- `flange_inner_offset_m` −5 mm (deep lateral inward overlap): **gap UNCHANGED**
  → it is not a lateral-reach problem.
- `flange_thickness_m` 4 → 12 mm (widen the seam-normal band): **gap CLOSES** —
  flange and cup-wall merge into one continuous solid.

So the through-slit is the flange detaching from the cup-wall where the organic
body curves *out of the flange's thin ±`flange_thickness_m` (4 mm) band*;
widening the band reconnects them. (This reconciles §MA-14 S3b, where thickness
moved sliver count in the axis-aligned case — thickness was always the lever;
the sphere just lacked the disconnection to show it.)

**Open design choice for the fix (workshop input needed):** a uniform 12 mm
half-thickness (= 24 mm closed flange zone) is a lot of PLA and changes the
clamp plate the workshop tuned to 4 mm. Options: (a) find the *minimum* uniform
thickness that reconnects everywhere (sweep real geometry — maybe 6–8 mm), or
(b) a TARGETED fix — keep the 4 mm outboard clamp plate but give the flange a
thicker **inner root** that spans to the cup-wall (a gusset/skirt that follows
the body), so only the junction thickens, not the whole plate. (b) is the
material-frugal answer but more work; (a) is simpler. Decide with the workshop
(clamp/bending/print intent).

### Thickness sweep on the REAL part — INCONCLUSIVE (metric problem) (2026-05-30 #4)

Regenned canaloff with `[flange] thickness_m` = 4/5/6/8 mm, counted long
slivers (span > 1 mm) across all 6 cup pieces:

| flange half-thickness | long slivers (all cups) |
|---|---|
| 4 mm (today) | 73 |
| 5 mm | 57 |
| 6 mm | 51 |
| 8 mm | 51 (plateau) |

Thickness **reduces but does not eliminate** them, plateauing at 51 — a caveat
to "12 mm closed it" (that was *one* location, z=20). **The metric is the
problem:** "span > 1 mm sliver" conflates (a) the real flange↔cup-wall VOIDS
(thickness fixes) with (b) benign MC-triangulation slivers on clean-solid,
watertight surfaces (confirmed inboard; thickness can't/needn't touch them).
So the plateau is ambiguous: either the real voids are gone and 51 = benign
slivers, OR uniform thickness is insufficient (different perimeter azimuths
need different band widths) → argues for a body-following gusset over a uniform
bump. **Sliver-count is the WRONG metric** — it can't isolate through-voids.

**NEXT SESSION — disambiguate with a void-specific SDF measure** (re-add the
`CF_MA15_PROBE` eval-based slot check, sweep thickness, count remaining SDF
*voids* not slivers). That decides uniform-bump vs gusset. Validation rigs left
in `~/scans/`: `cast.q4thick_{5,6,8}.toml` + `q4thick_*` output dirs,
`cast.q4weld_off.toml` + `q4_weld_off` (4 mm baseline, 78-sliver reference),
probes `/tmp/slit_classify.py` (slit→seam classification), `/tmp/count_longslivers.py`.
The `CF_MA15_PROBE` hook (ASCII SDF cross-sections in `cli::run`) was reverted;
re-add from git history of this session if needed.

### LEAD FIX DIRECTION (next session) — upstream hybrid flange (#1)

Don't band-aid the symptom (thickness/gusset are downstream of the root). The
root is that the flange and cup-wall are built from **two different source
geometries** that agree only at the seam plane. **Fix it upstream: build the
flange's INNER CONNECTION from the 3D cup-wall, not the 2D silhouette**, so
flange and wall are connected *by construction* at any body curvature — zero
void, no thickness tuning. This subsumes the gusset and removes the root.

- **Hybrid (the design to work out):** inner connection/weld from the **3D
  cup-wall** (`CupWallShellSdf` / body offset — guarantees the merge), outboard
  reach from the **2D silhouette** (`Silhouette2d` — keeps the §F-1
  concavity-following the flange needs around the organic perimeter). The 2D
  source was a deliberate §F-1 choice (3D body distance "falsely tripped the
  inner check" at silhouette concavities); the hybrid keeps that win while
  fixing the 3D connection it broke.
- Thickness sweep becomes **moot** under this fix.

### ⚠️ HARD CONSTRAINT — mating faces MUST be perfectly flat

Workshop (2026-05-30): **the cup-half mating faces have to be perfectly flat —
a major constraint.** (Workshop also prints the halves *mating-face-down* so
the bed backstops flatness — but the geometry must not fight it.) **Any flange
redesign MUST preserve the flat seam mating face** (the §F-4 bit-precise flat
halfspace cut, 1 µm). The seam cut stays a clean planar cut of the (now
robustly-connected) piece — do NOT round/blend the seam edge, do NOT introduce
a fillet/gusset that bulges onto the mating face. The flange connection fix is
*inboard* of the mating surface; the mating face itself is untouched. Validate
the mating-face flatness gate on every fix candidate.

### S15b VOID-MEASURE PROBE + HYBRID SPIKE (2026-05-30 #5) — SPIKED, then REJECTED for the resolution fix (§S15e)

**OUTCOME (read first):** the void-measure probe below pinned the slit as a
real but **~2 mm-thick SDF web** that the 3 mm production MC cell can't resolve
— i.e. a **mesh-resolution artifact, not a geometry defect**. The hybrid flange
was built + fully validated (it welds, mating face stays flat, bolts re-lengthened)
**but then REJECTED** because finer cells fix the same slit with the *original
flat flange* — no gusset, no bolt hack (see §S15e). The hybrid code was reverted;
its lasting value is the diagnosis. The diagnostic detail is kept below.

Re-added the `CF_MA15_PROBE` env hook (`design/cf-cast/src/ma15_probe.rs`,
throwaway) with a **void-specific** measure that finally disambiguates the
plateau: **connected solid components per XY slice at the production 3 mm MC
cell** (a flange arm in a separate component = a sub-cell web = the
through-slit). On the real `canaloff` layer-2 Negative piece:

- **Current production: 38 of 51 z-slices have the flange detached** from the
  cup-wall at the 3 mm cell — the full-height through-slit, reproduced on real
  geometry with a clean metric (NOT sliver count). The web is ~2–3 mm thick
  (1 component up to a 2.5 mm flood-fill cell, splits at 3 mm) — right at the
  production cell, so 3 mm MC under-resolves it → the printed slit. (This is
  why finer cells made the *sliver* count worse but the geometry is a genuine
  thin web — both true.)
- ASCII overlays confirm the mechanism: the seam plane exits the body at two
  points; the flange arm at one exit **welds** to the cup-wall, the other
  **floats** (detached by a void band) wherever the leaning body curves out of
  the flat clamp band.

**Hybrid construction (chosen + validated):** replace the flange's flat
seam-normal thickness term with a `min` of (flat clamp) and (3D-body
connection):

```text
flange exists where  d2 ∈ [inner_offset, flange_width]   (2D silhouette — unchanged)
                AND  min( |s| − flange_thickness,         (flat ±thickness clamp plate)
                          max( d3 − reach, |s| − cap ) )  (3D body connection, capped)
                     < 0
```

where `d2` = 2D seam-plane silhouette distance (outboard reach + gasket
clearance, **unchanged**), `s` = seam-normal coord, `d3` = the **3D body
distance** (the SAME body `CupWallShellSdf` tracks — this is the "inner
connection from the 3D cup-wall"). The body term emits flange material within
`reach` of the body, **capped to ±`cap` seam-normal** so it's a thin
body-following root, not a full body dilation. Derived params, **no new knobs**:
`reach = flange_width_m` (20 mm), `cap = 1.5 × flange_thickness_m` (6 mm).

Probe sweep (real geometry, web @ production cell, material vs current):

| construction | welds all z? | material @dome / mid |
|---|---|---|
| current (flat ±4 mm) | NO — 38/51 detached, web ~2 mm | baseline |
| `d3<reach`, no inner-clip | yes (reach 16) | **+211 %** (skin bulge) |
| + inner-clip `d2>inner` | yes (reach 16) | +88 % |
| + seam-normal cap `\|s\|<cap` | yes (reach 16, cap 6) | **+47 % dome, +6–7 % mid** |
| **chosen: reach=width, cap=1.5×thick** | **yes — 0/51 detached** | **+49 % dome, +8–9 % mid** |

The `|s|` cap is the key frugality lever (kills the over-fill far from the
seam where the broad body is). **Mating face stays flat** by construction (the
connection is `∩ halfspace` — a planar cut; the flat §F-4 seam is untouched)
and the **gasket channel stays clear** (the `d2 > inner_offset` clip is
retained). The smooth_union / thickness-sweep / body-dilation candidates are
all superseded by this.

**Hybrid implementation (this session, since REVERTED):** `Ribbon::hybrid_flange`
flag; `FlangeSdf` optional `HybridConnection`; `bolt_pattern` half-length sized
to the hybrid cap (`HYBRID_FLANGE_CAP_FACTOR`) so through-bolts still pass the
thicker flange. Workshop Orca gate: slits GONE, 9/9 bolts through, mating face
flat (24.6 µm). **But meaty (+49 % material at the dome) — and the workshop's
production target is ~0.5 mm, which exposed §S15e.**

### S15e — the slit is a 3 mm-MESH-RESOLUTION artifact; the fix is finer cells, NOT the hybrid (2026-05-30 #5)

The workshop noted production will run at a **high resolution (~0.5 mm)**. That
reframed everything. Probe measured the **flat (non-hybrid) flange** detachment
vs production cell (full z, real `canaloff` L2 N):

| MC cell | flat-flange z-slices detached |
|---|---|
| 0.5 mm | **0 / 51** |
| 1.0 mm | **0 / 51** |
| 1.5 mm | **0 / 51** |
| 2.0 mm | **0 / 51** |
| 3.0 mm | 38 / 51 |

The ~2 mm web resolves at **any cell ≤ 2 mm** → the flat flange connects with
**no gusset, no bolt-length hack, no meatiness**. The hybrid was a 3 mm-MC
workaround for a problem finer cells design away. **Decision: revert the hybrid;
set the production cup cell to 1.5 mm** (verified feasible: full regen 357 s,
peak RSS 2.6 GB; the flat-flange mesh is one connected contour at every z — slit
gone). True **0.5 mm** for *surface finish* is a separate, harder goal (the
dense-grid MC is cubic in RAM/time → 0.5 mm on the full cup likely OOMs) = the
S2 affordable-fine-meshing work; not a slit blocker. The hybrid's value was
purely diagnostic (it proved the web is real + 2 mm, hence resolution is the
lever). Validation rigs: `~/scans/q4flat15/` (1.5 mm flat-flange, slit-free),
`/tmp/loop_scan.py` (slicer-contour test), `/tmp/cf-cast-cli-NEW`.

**Also surfaced (separate pre-existing issue): apex pour-flanking bolts are
DROPPED for this part.** The apex-axial pour brackets the bore with a flanking
bolt each side to clamp the split flange. On base_mold's tight, LEANING dome
apex those flanking bolts can't be placed cleanly — workshop flagged repeatedly:
the washer overlaps the open sprue, or the bolt lands at the apex tip, or it
crowds a dowel. Four geometric criteria were tried (cavity-3D distance,
centerline distance, normal-vs-pour-axis, washer-on-flange) and NONE cleanly
isolates the stray: relative to the bore the stray and the good flank bolt are
geometric twins (same clearances) — they differ only in absolute position, and
auto-suppression either misfired or relocated the bolt into a dowel. Conclusion:
a leaning apex can't host a clean flanking bolt; **drop them** via a new
`[pour_gate].flank_bolts` config (default `true`; set `false` for base_mold).
The apex split is then clamped by the arc-bolt ring + hand pressure (verify at
the physical pour; add a hand clamp if it weeps). Result: min hole-to-hole
distance 22 mm (was a 4.4 mm bolt-dowel pair), clean even ring. The flanking-fix
spikes (washer-clear-bore, apex-tip heuristic) were reverted — `flank_bolts=false`
sidesteps the whole problem for this part.

### §MA-16 — floor "regression" at fine cells = the S1b SLAB itself

After 1.5 mm shipped the slit fix, the workshop saw a new floor defect: an
**octagonal groove ringing the plug-lock socket** (cf-view + Orca). Long
mis-chase (recorded so the next session doesn't repeat it): it is NOT a deep
organic bowl — the cleaned scan is **capped flat at the cap plane** (z = −60,
scan z-min = −60), so the layer body does not extend below the cap and the
cavity floor is *naturally flat at z = cap* at a fine cell. ROOT: the
`flat_cavity_floor` SLAB (S1b) itself. Its fill+cut clamp pulls a **bimodal
floor** (most faces to z = −61.15, islands left at z = −60); that 1.15 mm step,
traced as the ray-sampled footprint polygon, **is the groove**. A/B at 1.5 mm,
cavity floor around the socket (r 6–26 mm, off-seam): **slab OFF → flat, std
0.13 mm; slab ON → bimodal, std 0.47 mm + groove.** FIX: set
`[cast].flat_cavity_floor = false` for flat-capped bodies meshed at fine cells —
the slab's value was **cell-dependent** (clamped genuine bumpiness at the 3 mm
coarse MC the workshop praised; pure downside at 1.5 mm where the floor is
already clean). NO code change — the flag exists; default stays `true` for
coarse cells / non-flat-capped bodies. Confirmed in `cast_base_mold_canaloff`.

### §MA-17 — remaining: seam EDGE sharpness = fine-cell MC (OOM-bound)

The last workshop item: the **edge where the cavity wall meets the flat mating
face** is a jagged MC zigzag, not a crisp line. Characterised: the mating FACE
is flat (planarity 0.108 mm — §F-4 holds); the edge is the MC faceting of the
**organic scan wall** cut by the seam plane at 1.5 mm (same root as the cup-wall
terracing). NOT a fixable CSG artifact like the groove. Smoothing is out
(would round the flat mating face / §F-4); the only constraint-safe lever is a
**finer cell**, which sharpens the edge, smooths the walls, AND gives the
workshop-wanted 0.5 mm surface finish — all at once. Blocker: 0.5 mm OOMs the
baked-grid MC. So **S2 = fine-cell MC without OOM** is the real next task (it
unlocks edge + walls + finish together). Per [[feedback-correctness-over-speed]].

### Plan

- **S15a — §MA-15 recon. ✅**
- **S15b — void-measure probe + hybrid spike. ✅** (diagnosis solid; hybrid built.)
- **S15c — flag-gated hybrid impl + Orca gate. ✅** (welds, but meaty.)
- **S15e — REVERT hybrid; fix via resolution (1.5 mm production cup cell). ✅
  workshop-chosen** (above). Flat flange connects at ≤ 2 mm, no meatiness.
- **S15f — drop apex flanking bolts (`flank_bolts=false`); set 1.5 mm prod
  config. ✅**
- **S16 — floor groove = the S1b slab at fine cells; `flat_cavity_floor=false`
  for flat-capped bodies. ✅** (config; cf-view ✓.) ← committing.
- **S2 — fine-cell MC without OOM** (edge sharpness + wall terracing + 0.5 mm
  finish). ← next real task.

---

## Successor

**S2 — fine-cell MC without OOM picks up next.** It is the root unlock for the
0.5 mm production surface finish the workshop wants, and it simultaneously
sharpens the seam EDGE (§MA-17) and removes the cup-wall terracing — all three
are the same organic-wall-at-1.5 mm faceting. The baked-grid MC OOMs at 0.5 mm;
investigate sparse/per-piece-bounds/streaming MC. The physical print follows.
Floor (S16) + slit (S15) + apex flanking (S15f) are resolved. Per
[[feedback-correctness-over-speed]].

### Historical: S1b succession note

S1b (composition spike — exact-CSG floor + seam cuts) was the prior
successor and **SHIPPED** as `72496d3e` (cf-view ✓). S1a is banked
(§MA-13); the silhouette-cost speed lever was spiked-and-rejected, so the
speed question defers to S2 (bake/extractor) where it won't fight S1b.
