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

cf-cast SDFs are ~1-Lipschitz (|∇| ≤ 1): cf-design primitives are exact distance fields, and the
compositors are plain `min`/`max`/intersect/subtract (the **flat-mating-face constraint forbids
`smooth_union`**, which is the thing that would break Lipschitz). So for any point `p`,
`|SDF(q) − SDF(p)| ≤ |q − p|`. Therefore if `|SDF(p)| > R`, no point within distance `R` of `p`
has `SDF = 0` — the surface is `> R` away.

## Coarse-to-fine fill (replaces the full triple-loop)

1. **Coarse pass:** for each block of `K³` cells (K = `NARROW_BAND_BLOCK_CELLS`), evaluate the SDF at
   the block center. If `|SDF(center)| > circumradius` (center→farthest-corner distance + safety
   margin) → **skip** (record `sign(center)`); else → **refine**.
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
- **Skipped regions are all-same-sign.** A skip block has no internal sign change (`|SDF| > R`), so it
  emits no triangles — exactly as the full grid (which holds large same-sign values there) would.
  Two *adjacent* skip blocks can't have opposite signs (1-Lipschitz: the cross-block difference
  `≤ K·cell` can't span `> 2R ≈ 1.7·K·cell`), so sentinels never create a spurious crossing at a
  skip/skip seam.

**The 1-Lipschitz property is the only assumption, and it is GATED, not trusted:** a unit test asserts
`marching_cubes(full) == marching_cubes(narrow)` on a real solid, and a real-body regen A/B compares
STL hashes. Any divergence (a field marginally over 1-Lipschitz) fails the test → widen the margin or
fall back. The `circumradius` carries a conservative `+2·cell` margin for robustness.

## Gates

cf-cast lib byte-identity unit test · real-body regen A/B (hashes) · grade-all 295/295 · clippy
(pedantic+nursery) + fmt + rustdoc · the 0.5 mm production regen timed full-vs-narrow.
