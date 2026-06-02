# cf-cast narrow-band marching cubes — design & bit-identity argument

**Goal:** make `solid_to_mm_mesh` (cf-cast's SDF→mesh path) skip the ~99% of grid cells far
from the surface, **bit-identically** (the output mesh must be *exactly* the full-grid result —
zero quality loss). At 0.5 mm over a ~200 mm cup the full grid is ~400³ evals, ~all wasted on
deep-inside / deep-outside cells; the surface is a thin 2D band. The cost being cut is **SDF
evaluation time** (dominated by the BVH body-distance), not memory — the `ScalarGrid` stays dense
(allocated `n³`); we just fill it sparsely. Expected ~`n/K`-ish speedup (tens of ×, less the
~2× the 1-point halo adds in the refine band). This is the bit-identical speedup deferred from §MA-18.

Why cf-cast can't reuse cf-design's mesher pruning: that uses *interval* bounds over a cell AABB,
and the body SDF is a BVH mesh-distance with no interval bound; cf-cast also needs manifold MC, not
dual contouring. The technique below uses the *point value + Lipschitz* property instead — no
intervals needed.

## The Lipschitz skip

Most cf-cast SDFs are ≤1-Lipschitz (|∇| ≤ 1): cf-design primitives are exact distance fields, and the
compositors are plain `min`/`max`/intersect/subtract (the **flat-mating-face constraint forbids
`smooth_union`**, which is the thing that would break Lipschitz). So for any point `p`,
`|SDF(q) − SDF(p)| ≤ |q − p|`. Therefore if `|SDF(p)| > R`, no point within distance `R` of `p`
has `SDF = 0` — the surface is `> R` away.

### The non-1-Lipschitz case (canal feature field) — `field_skin_m`

The canal plug is the exception: its field is `f = base + tex` where `base` is the ≤1-Lipschitz scan
plug distance and `tex` is the **additive** grip-ring / D-section / texture / suction displacement.
`tex` is *magnitude*-bounded (`|tex| ≤ A`, `A = max_inward_depth + suction_bulge`) but **not**
1-Lipschitz: the texture's axial slope `amp·π/pitch ≈ 0.59` at the iter-1 defaults pushes the composite
gradient to ≈1.59. A naive 1-Lipschitz skip would therefore silently drop texture geometry — and the
0.5 mm production plug uses exactly this field. The fix exploits that `tex` is magnitude-bounded: a
zero of `f` within radius `ρ` of `p` forces `|base(p)| ≤ A + ρ` (`base` 1-Lipschitz), and
`|base(p)| ≥ |f(p)| − A`, so **`|f(p)| > ρ + 2A` still proves no zero within `ρ`**. The skip threshold
gains `2A`: `skip_radius = circumradius + 2·cell + 2·field_skin_m`. Callers pass `field_skin_m = A` via
[`solid_to_mm_mesh_with_skin`] (0 for plain CSG ⇒ unchanged). This is provably correct for *any*
bounded additive perturbation on a 1-Lipschitz base, not just the texture term — no per-feature
gradient bookkeeping needed.

## Coarse-to-fine fill (replaces the full triple-loop)

1. **Coarse pass:** for each block of `K³` cells (K = `NARROW_BAND_BLOCK_CELLS`), evaluate the SDF at
   the block center. If `|SDF(center)| > skip_radius` (= `circumradius + 2·cell + 2·field_skin_m`,
   center→farthest-corner distance + the MC-halo margin + the non-1-Lipschitz skin) **and the eval is
   finite** → **skip** (record `sign(center)`); else → **refine** (a non-finite center always refines,
   so the classification is total).
2. **Skip-fill:** set every grid point in a skip block to `sign · FAR_SENTINEL` (large value, correct
   sign). MC reads only the SIGN of far cells (all-same-sign → no triangle), so the magnitude is
   immaterial.
3. **Refine + halo:** for each refine block, evaluate the exact SDF at every grid point in the block
   **plus a 1-point halo** into its neighbors, overwriting any sentinels. (Order: skip-fill all, then
   refine — so the halo's exact values win at shared boundaries.)
4. `marching_cubes(&grid)` unchanged.

## Why it's bit-identical

`marching_cubes` emits a triangle for a cell only if its 8 corners change sign, interpolating each
crossing edge between its two corner values. So the output is identical to the full grid iff:

- **Every surface-crossing cell's 8 corners hold the exact SDF.** A cell containing the surface lies
  within `circumradius` of the center of at least one block touching it → that block is refined
  (`|SDF(center)| ≤ circumradius`). The **1-point halo** then guarantees all 8 corners of that cell
  (which may straddle into a neighbor block) are exactly evaluated — so no crossing edge interpolates
  against a sentinel. *(This halo is the load-bearing detail: without it, a crossing edge from a
  refine corner to a skip-block corner would interpolate a wrong vertex.)*
- **Skipped regions are all-same-sign.** A skip block has no internal sign change (`|f| > skip_radius`
  via the `ρ + 2A` argument above), so it emits no triangles — exactly as the full grid (which holds
  large same-sign values there) would. Two *adjacent* skip blocks can't have opposite signs: across the
  ~`K·cell` center-to-center distance `f` changes by at most `K·cell + 2A` (1-Lipschitz base + bounded
  `tex`), which is `< 2·skip_radius`, so sentinels never create a spurious crossing at a skip/skip seam.

**The assumptions are (i) the base field is ≤1-Lipschitz and (ii) any non-1-Lipschitz part is an
*additive* term of magnitude `≤ field_skin_m` — and they are GATED, not trusted:** the unit test
`narrow_band_fill_is_bit_identical_to_full` asserts `marching_cubes(full) == marching_cubes(narrow)`
across sphere / cuboid / union / **subtract (hollow shell) / intersect** fixtures (and checks the skip
path actually fires), and `narrow_band_field_skin_handles_non_lipschitz_field` asserts byte-identity on
a **>1-Lipschitz** ribbed field at `skin = amp` plus that the skin is load-bearing (a larger skin
strictly widens the refine band). A 0.5 mm full-vs-narrow A/B on the real production cast compares STL
hashes end-to-end. Any divergence fails a test → widen `field_skin_m` or fall back to
`CF_CAST_FULL_GRID`.

## Gates

cf-cast lib byte-identity unit test · real-body regen A/B (hashes) · grade-all 295/295 · clippy
(pedantic+nursery) + fmt + rustdoc · the 0.5 mm production regen timed full-vs-narrow.
