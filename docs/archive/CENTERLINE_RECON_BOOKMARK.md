# Centerline Computation — Recon Bookmark

**Status**: OPEN, blocks workshop iter-1 cast. Three-session pattern: this is
the bookmark; next session does dedicated recon (research + spec); third
session implements the chosen approach.

**Where we are on dev (`df79452d`)**: shipped Taubin smoothing pipeline + cf-
device-design radial smoothing. Centerline algorithm is at iteration #5 (AABB
center + spine_hint), produces a straight line but doesn't align with the
body's perceived visual center on asymmetric scans. Workshop iter-1 cast
quality depends on getting this right.

---

## Problem statement

We need to compute a robust centerline (polyline of 30 points) through a
roughly-axisymmetric body-part scan such that:

1. **cf-device-design's per-vertex radial direction computation produces
   smooth, consistent radials.** Each scan vertex picks its nearest centerline
   point via `nearest_centerline_index` and computes radial = (vertex -
   nearest). If the centerline is wiggly or off-axis, adjacent vertices on the
   dome map to different centerline points → discontinuous radial directions
   → lumpy / asymmetric displaced layer surfaces.
2. **cf-cast's curve-following mold paths follow the body's true axis.** Used
   by v2/v2.1 mold generation; an off-axis centerline produces off-axis mold
   pieces.
3. **The centerline overlay visually aligns with the body's perceived center.**
   Design-time user confidence — if the overlay sits off-center from what the
   user sees, they can't trust the design.

Specifically: the **dome end** of the centerline (`polyline[0]`) is the part
that fails. It tends to wiggle laterally because dome cross-sections shrink
toward a point and the statistics of small cross-sections are noise-dominated.
The wiggle cascades into asymmetric layer surfaces visible in cf-device-design.

---

## What we tried (5 iterations, all failed) and root-cause analysis

### Iteration 1 — Per-slab Kasa best-fit circle

Slabs perpendicular to spine direction; Kasa algebraic least-squares fits a
circle to each slab's vertex projections; circle center = centerline point.

**Failed because**: non-uniform vertex density around cross-sections biased
the per-slab fit center toward the dense side. Scanner data is asymmetric
around the dome (scanner couldn't capture both sides perfectly evenly), so
the fitted circle's center is pulled toward the data side.

### Iteration 2 — Per-slab Kasa + centroid-prior blend

Same as #1 but blended Kasa with body centroid prior, weighted by vertex
count: `alpha = min(1, vertex_count / 200)`, `center = alpha × Kasa + (1 -
alpha) × centroid_prior`.

**Failed because**: dense dome slabs (~1500-2000 verts on the iter-1
sock-over-capsule fixture, way above the 200-vert threshold) had alpha
clamped to 1.0 → blend never engaged for the cases where it was needed →
centerline curve persisted.

### Iteration 3 — PCA-only

Centerline = N points along body's PCA principal axis through body centroid.
Compute PCA via 3×3 covariance eigendecomposition.

**Failed because**: for short-wide bodies (lateral variance > axial variance),
PCA's principal eigenvector lies LATERAL, not along the intended body axis.
Caught by a regression test using a tapered cylinder fixture where the
cylinder was wider than tall.

### Iteration 4 — Centroid + spine_hint (no per-slab fit)

Centerline = line through body vertex centroid in spine_hint direction.
Trusts the user's spine direction (cap normal); anchors at the mean of
vertex positions.

**Failed because**: vertex centroid is biased by:
- Non-uniform vertex density along the body (more verts on densely-tessellated
  regions, fewer on sparsely-tessellated ones).
- The reconstruct extension's added vertices at the floor end (which add
  geometry weighted toward one end and possibly slightly off-axis).

Centerline came out straight but laterally offset from the body's perceived
geometric center.

### Iteration 5 — AABB center + spine_hint (current shipped state)

Centerline = line through mesh AABB center in spine_hint direction. After
auto-center at load, AABB center is at origin → centerline runs along the
world Z axis (assuming spine_hint = ±Z).

**Failed because**: the body's perceived geometric center doesn't coincide
with AABB center for asymmetric scans. The iter-1 fixture has subtle
asymmetry — one-side bulges, plus the reconstruct extension's geometry from
prior iterations of this same algorithm — that makes AABB center diverge
from the visual center.

User verbatim: "now the line is straight but not centered."

---

## The deeper insight

We've been trying to **compute** the centerline from body data via various
statistics. None worked because the data has biases that no simple statistic
escapes:

- **Vertex density bias**: surface samples aren't uniformly distributed.
- **Scanner view bias**: more data on the side facing the scanner.
- **Reconstruction asymmetry**: the floor-end reconstruct adds vertices
  weighted by previous-iteration centerline choices.
- **AABB asymmetry**: extreme points include bumps + noise, not just the
  body's main mass.

**The architectural question for the next session**: should we keep trying
to compute a centerline that ALIGNS with the body, OR should we DECOUPLE
the centerline from the body — let the user / a global axis define the
centerline, and then **force the reconstruction (and possibly other design
artifacts) to be symmetric around that centerline**?

This is the difference between:
- **Descriptive centerline**: follows the body's actual axis (including
  asymmetries). What we've been trying.
- **Prescriptive centerline**: defines what the design's axis SHOULD be.
  The body shape doesn't dictate it; the centerline dictates the design.

For silicone casting of body parts where the user wants the cast to be
**symmetric** (because silicone surface tension smooths asymmetries), a
prescriptive centerline + symmetric reconstruction might be the right
architecture. The descriptive-centerline approach is what fights us.

---

## Research directions for the recon session

### Direction A — Mean Curvature Flow (MCF) skeleton extraction

Tagliasacchi, Alhashim, Olson, Zhang, "Mean Curvature Skeletons" (SGP 2012).
Iteratively contract the mesh inward along the curvature direction; the
surface eventually collapses to a 1D skeleton.

- **Pros**: geometrically rigorous, handles arbitrary shapes including
  branching topology, well-known + cited algorithm.
- **Cons**: iterative + slow (could be 10-30s for 169k-face mesh), complex
  to implement from scratch (~500+ LOC), requires careful step-size tuning.
- **Library**: not sure if there's a Rust implementation; CGAL has one in
  C++.

### Direction B — Voronoi-based Medial Axis Transform (MAT)

Compute the Voronoi diagram of mesh vertices; the medial axis is the set
of Voronoi vertices "inside" the mesh. Prune medial axis to keep only the
trunk (no branches).

- **Pros**: classical algorithm, well-studied, exact (not iterative).
- **Cons**: Voronoi computation is O(N log N) but with high constant;
  pruning is a heuristic + ad-hoc; medial axis is sensitive to surface
  noise (every bump produces a medial axis branch).
- **Library**: nalgebra alone won't give us Voronoi; we'd need an external
  crate (no good options in pure Rust ecosystem).

### Direction C — Iterative re-orientation per-slab fit

Compute initial centerline (any algorithm — even AABB+spine_hint). Re-orient
slabs to be perpendicular to the **local** centerline direction (not the
global spine). Re-fit per-slab centers. Iterate until convergence.

- **Pros**: simpler than MCF/MAT, handles curved bodies, builds on what we
  already have.
- **Cons**: may not converge cleanly; doesn't address the root cause (per-
  slab fit bias from vertex density).

### Direction D — Voxel skeletonization

Voxelize the mesh, apply 3D skeleton thinning (Lee-Kashyap-Chu 1994 or
similar). Skeleton is a 1-voxel-thick representation of the body's medial
structure.

- **Pros**: robust to surface noise (voxelization absorbs sub-voxel detail),
  well-known thinning algorithms.
- **Cons**: voxelization adds discretization error (depends on voxel size);
  thinning is its own algorithmic complexity; output is a voxel skeleton
  that needs to be re-traced to a polyline.

### Direction E — L1-medial skeleton

Huang, Wu, Cohen-Or, Gong, Zhang, Chen, "L1-Medial Skeleton of Point
Clouds" (SIGGRAPH 2013). Iteratively project sample points to their local
L1-medians.

- **Pros**: works on point clouds (no mesh topology needed), handles noise
  well, robust + general.
- **Cons**: complex to implement; convergence parameters need tuning;
  output is a sparse skeleton that needs polyline extraction.

### Direction F — Hybrid "force-symmetric-reconstruction" architecture

Decouple centerline from body. Define centerline as **PRESCRIPTIVE** (AABB
center + spine_hint OR user-clicked endpoints OR PCA major axis with sign
hint). Force the **reconstruction algorithm** to build axisymmetric geometry
around this prescriptive centerline (rather than following the body's
asymmetric shape).

- **Pros**: simplest to implement; reuses existing AABB+spine_hint
  centerline; only changes the reconstruct algorithm. Aligns with the
  user's actual goal (symmetric silicone casts).
- **Cons**: descriptive-centerline use cases (e.g., curve-following cast
  for a bent limb) wouldn't work. But our target use case is straight
  body-part scans, so this is acceptable.

### Direction G — User-defined centerline (two-click axis pick)

Add two click-points in the cf-scan-prep UI: "centerline tip" and
"centerline floor". User clicks both. Centerline = line between them.
Reconstruction operates around this user-defined axis.

- **Pros**: zero algorithmic risk; user controls the axis explicitly;
  guaranteed correct from the user's perspective.
- **Cons**: UI friction (one more click per scan); user needs to know
  where the "true" axis is.

---

## Suggested agenda for the next session

The session should be a **dedicated recon arc** — research + spec, not
implementation. The actual implementation arc comes AFTER the recon's
output (a spec doc + a chosen approach).

### Phase 1 — Research (1-2 hours)

For each of Directions A-G above:
- Read the relevant papers / docs.
- Find existing implementations (Rust crates, C++ libraries, reference
  implementations).
- Estimate implementation cost (LOC + complexity).
- Estimate robustness (does it handle the iter-1 fixture's asymmetries?
  the dome-tip noise? non-uniform sampling?).
- Note dependencies (external crates? linear algebra requirements?
  voxelization libraries?).

### Phase 2 — Triage (30 min)

Score each direction on:
- **Robustness**: handles iter-1 fixture's asymmetries + dome noise.
- **Simplicity**: implementation cost + dependency surface.
- **Correctness**: produces a geometrically meaningful centerline.
- **Fit for use case**: aligns with cortenforge's body-part-scan target.

Pick the top 1-2 directions to advance.

### Phase 3 — Spec the chosen approach (1 hour)

Write a spec doc at `docs/CENTERLINE_SPEC.md`:
- Chosen algorithm + alternative considered + why this one.
- Detailed pseudocode.
- Data structures (input mesh + output polyline format).
- Performance budget (latency for 169k-face scan).
- Test plan (synthetic fixtures + iter-1 verification).
- Open questions / risks.

### Phase 4 — Implementation arc (separate next-next session)

Once the spec is locked, the implementation arc follows. Single PR /
single coherent commit ladder.

---

## Reproduction fixture

`~/scans/sock_over_capsule.stl` (raw, 3.34M faces, repo-excluded).

Symptom reproducer:
1. `cargo run -p cf-scan-prep --release -- ~/scans/sock_over_capsule.stl --stl-units mm`
2. Apply Simplify (200k) → Cap → Scan → Centerline trim 40mm + Apply →
   Reconstruct Extrapolate + Apply → smoothing 8 → Save.
3. `cargo run -p cf-device-design --release -- ~/scans/sock_over_capsule.cleaned.stl`
4. Toggle Layer 0 visible (default 10mm thick from cavity).
5. Observe: dome on the layer surface is asymmetric / lumpy despite
   underlying scan dome being smooth and round.

---

## Related memos

- `[[project_cf_scan_prep_target_use_case]]` — body-part scan focus.
- `[[project_cf_device_design_layer_preview_bookmark]]` — sibling bookmark
  for the cf-device-design layer-preview quality coupling.
- `[[feedback_strip_the_knob_when_default_works]]` — applies here too: if
  the algorithm is right, no user-tunable centerline parameters needed.
- `[[feedback_bookmark_when_surface_levers_exhaust]]` — three-session
  pattern we're using here.
