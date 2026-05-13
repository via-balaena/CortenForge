# Curve-Following Mold Design — v2 cf-cast architecture

**Status:** design phase (opened 2026-05-12)
**Predecessor docs:** [`CASTING_ROADMAP.md`](CASTING_ROADMAP.md), [`SCAN_PREP_DESIGN.md`](SCAN_PREP_DESIGN.md)
**Supersedes:** v1 single-cup straight-pull cf-cast (Stages 1 + 2 in main) for production use

---

## Strategic context

v1 cf-cast (shipped, Stages 1 + 2) generates single-cup straight-pull molds that demold along `+Z`. The user's verdict 2026-05-12: **straight-pull is a proven failure for the appendage shapes that cortenforge targets.** Real scans of body-part-class geometry are:

- **Curved** — bananas, fingers, limbs, prosthetic shells. The chord-from-base-to-tip isn't the natural demold path; the curve is.
- **Imperfect** — rotating-table scans produce jagged open boundaries, sub-mm textile noise, occasional non-watertight regions. Forcing them to align with a rigid `+Z` axis fights the data.

The architectural answer: **the mold demolds along the scan's own centerline curve, in multiple pieces that separate along a surface derived from that curve.** No "perfect alignment" required; the scan's own geometry tells the mold what shape to be.

This is v2 cf-cast. v1 stays in main as a building block (the underlying SDF + marching-cubes + multi-layer infrastructure all carry over); v2 adds the curve-following composition on top.

---

## Inputs

v2 cf-cast consumes the output of `cf-scan-prep` commit #12 (Save):

1. **`<stem>.cleaned.stl`** — the watertight, simplified, optionally clipped + capped scan mesh in cast-frame meters. cf-scan-prep's Cap step closes any open boundaries so `mesh_sdf::SignedDistanceField::from_mesh` gives valid inside/outside.

2. **`<stem>.prep.toml`** — provenance + the **centerline polyline** computed by cf-scan-prep's cross-section-centroid algorithm (commit #9). Polyline format:

   ```toml
   [centerline]
   # Sampled along a "spine axis" at scan-prep time; vertices in
   # physics-frame meters. ~10-30 points typical.
   points = [
     [0.000,  0.000, -0.060],   # base (cup mouth end)
     [0.002,  0.001, -0.050],
     [0.005,  0.003, -0.040],
     # ... (curving)
     [0.012,  0.018,  0.060],   # tip
   ]
   ```

   Plus per-layer material spec carried through from cf-cast's existing `CastLayer` definition (unchanged from v1):

   ```toml
   [[layers]]  # innermost-first
   display_name = "Ecoflex 00-30 (inner)"
   density = 1070.0
   anchor_key = "ecoflex_00_30"
   thickness_m = 0.006
   # ... etc per layer
   ```

3. **`PrinterConfig`** (same as v1) — build volume, technology, materials. v2 adds: per-piece-must-fit check (each mold piece individually fits the build volume, not just the aggregate AABB).

---

## Output artifacts

For an N-piece mold (default N=2; see "Piece count selection"):

- **`mold_layer_<i>_piece_<j>.stl`** — N × L STLs (L = layer count), one per piece per layer
- **`plug.stl`** — innermost cavity plug, single piece (same as v1; the plug doesn't split)
- **`procedure.md`** — per-piece assembly order, layer cast sequence, demold sequence

Total file count: `N * L + 1 + 1` (e.g., 2-piece × 3-layer = 8 STLs + procedure.md = 9 files vs v1's 5).

---

## Algorithm

### Step 1 — Centerline as input

Already computed in cf-scan-prep (cross-section centroids; commit #9). v2 cf-cast reads the polyline from `.prep.toml`. **No re-computation in cf-cast.** This separation matters: if the centerline algorithm needs upgrading (e.g., to mean-curvature-flow for branching geometry), cf-scan-prep changes; v2 cf-cast doesn't.

### Step 2 — Split-surface generation (the ribbon)

For each consecutive pair of centerline points `(P_i, P_{i+1})`:

1. **Tangent** `T_i = normalize(P_{i+1} - P_i)` — direction of the curve at that segment
2. **Split-normal `N_split`** — user-chosen world-frame "which way does the mold open?" direction. Default: world `+X`. The chosen direction is rotated into each segment's local frame so it stays perpendicular to the tangent.
3. **Local ribbon normal** `B_i = normalize(T_i × N_split)` — the binormal that defines the cutting plane at each segment. The ribbon at segment `i` is the half-plane through `P_i` with normal `B_i`.

The **ribbon SDF** is then: for any world-frame point `Q`,

1. Find the closest centerline segment to `Q` (linear scan over the ~10-30-point polyline; cheap)
2. Project `Q` onto that segment to get `P*` and the local tangent `T*` + binormal `B*`
3. Return `dot(Q - P*, B*)` — signed distance to the local cutting plane

This produces a continuous signed-distance field whose zero set is the ribbon surface.

**Why a ribbon, not a flat plane:** for a curved centerline, the cutting surface MUST curve along with it. A flat plane would intersect the mesh non-trivially on curved bodies and produce ugly piece geometry.

### Step 3 — Per-piece SDF construction

For each layer `i` (innermost-first, same as v1) and each piece `j ∈ {0, 1}` (2-piece default):

```
piece_sdf = mold_cup_sdf
            ∖ layer_body_sdf
            ∩ (j == 0 ? ribbon_negative : ribbon_positive)
```

Where:
- `mold_cup_sdf` — outer mold envelope (same construction as v1: bounding region minus the layer body's outward-offset SDF). Closes around the entire scan.
- `layer_body_sdf` — the scan + cumulative outer-shell-thickness offset (same as v1's layered scheme)
- `ribbon_negative / ribbon_positive` — half-space SDFs: `ribbon_negative = -ribbon_sdf`, `ribbon_positive = +ribbon_sdf`. Marching cubes on these gives one piece on each side of the ribbon.

### Step 4 — Marching cubes per piece per layer

Same `mesh-offset::marching_cubes` infrastructure as v1, called once per `piece_sdf`. Each call → one IndexedMesh → `mesh-io::save_stl`. Atomic write at the meshing boundary (same pattern as v1's `cf-cast` Stage 2 F1).

### Step 5 — Printability + per-piece volume check

Each piece STL goes through v1's printability gate (`mesh-printability::validate_for_printing`). **New per-piece check**: each piece's AABB must fit the printer's build volume independently. v1's aggregate check passes if the *combined* AABB fits; v2 enforces the stricter per-piece bound (you can't print a piece larger than your printer no matter how the combined fits).

### Step 6 — Per-piece procedure.md

Generated by an extended version of v1's `cf-cast::procedure` module:

- Pre-pour: how to align + clamp the N pieces (registration pins/dovetails — TBD, see "Open questions")
- Per-layer cast sequence (unchanged from v1 conceptually; just references "the assembled N-piece mold")
- Demold: explicit piece-removal order along the centerline direction. For a curved scan, removing piece 0 then piece 1 in order means the part slides out along the curve.

---

## Piece count selection

**v2 ships with N = 2.** Algorithm picks based on max tangent rotation along the centerline polyline:

| Max tangent rotation | Piece count |
|---|---|
| < 60° | 2 |
| 60–120° | 2 (with warning: tight curves may bind) |
| > 120° | refuse with error; user must split scan upstream |

**3+-piece molds are v3.** They need a more complex split-surface algorithm (multiple ribbons, intersection of cutting surfaces) and the procedure.md becomes much more involved. Out of scope until v2 surfaces real use cases that need it.

---

## Open questions

These need pinning before implementation:

1. **Registration mechanism between pieces** — how do the N pieces lock together during pour? Options: cylindrical pins through tabs (printed integrally with the pieces), dovetail interlocks, magnets, hand-clamping with rubber bands. Affects the mold-piece geometry — pins/dovetails are additive features that need to be CSG'd in before export.

2. **Pour gate location** — silicone has to enter the mold somewhere. v1 used the `+Z` mouth from the original cup. v2: gate placement TBD; probably at one end of the centerline (the base end), forming a small channel that's CSG'd off the mold envelope.

3. **Air-release vents** — curved molds trap air at the curve's apex. Vent placement also TBD.

4. **`N_split` direction selection** — user-pickable in cf-scan-prep, or auto-derived from minimizing piece overhang? Auto would be nicer but requires per-direction optimization (search over `N_split` candidate angles, pick the one that produces pieces with minimum overhang). v2: user-pickable via a `--split-normal X,Y,Z` CLI flag on the cast example; auto-search is v3 polish.

5. **Centerline upgrade path** — cross-section centroids work for tubular shapes; branching geometry (multi-finger hands) needs mean curvature flow or Voronoi-of-surface-samples. v2 stays with cross-section centroids since the primary use case (single-limb appendages) is well-served. Upgrade trigger: iter-1 or later surfaces a scan where the cyan polyline visibly drifts off the scan's actual spine.

6. **Multi-piece interaction with multi-layer** — for v1's nested-cup multi-layer scheme to work in v2's multi-piece geometry, each layer's mold pieces have to register against the previous layer's CURED silicone (which is itself an N-piece-cast object now). Does that actually work in practice? Workshop validation needed; iter-1 single-layer-curved cast should test this before iter-2 multi-layer-curved.

---

## Implementation arc

**Estimate: 3-6 weeks active across multiple sessions.** Risk tail if centerline algorithm or split-surface SDF surfaces convergence/numerical issues.

| # | Scope | Est |
|---|---|---:|
| 1 | This design doc + slice-log opens v2 track in `CASTING_ROADMAP.md` | done |
| 2 | `cf-scan-prep` commit #12 (Save) — write cleaned STL + `.prep.toml` with `[centerline]` block. Existing scan-prep panels (Simplify, Reorient, Recenter, Clip, Cap) all stay. Reorient/Recenter become "rough orient so the scan isn't upside down", not "perfect cast-frame alignment". | ~3 hr |
| 3 | New `cf-cast::SplitNormal` + `Ribbon` types | ~2 hr |
| 4 | Ribbon SDF implementation + unit tests against analytic curves | ~4-6 hr |
| 5 | Per-piece SDF composition (`piece_sdf = mold_cup ∖ layer_body ∩ ribbon_side`) | ~3 hr |
| 6 | Marching cubes per piece + STL export | ~3 hr |
| 7 | Per-piece printability + per-piece AABB checks | ~2 hr |
| 8 | Procedure.md generator for N-piece assembly + demold | ~3-4 hr |
| 9 | Registration features (pins or dovetails) — pick one approach, integrate | ~5-8 hr |
| 10 | Pour-gate + vent geometry | ~3-4 hr |
| 11 | `examples/cast/layered-silicone-device-v1-scan` (renamed to `-v2-scan-curve-following` or similar) — end-to-end exercise on the iter-1 fixture, produces N × L STLs + procedure.md | ~3 hr |
| 12 | iter-1 physical cast (workshop activity, not code) | external |

**~30-40 hours active** for the code-side work, plus the workshop iter-1.

cf-scan-prep commits #10 (Mouth ext.) and #11 (Cleaned AABB) are **cancelled** — they were v1-specific concepts. cf-scan-prep #13 (mesh-repair diagnostics + keyboard shortcuts) survives unchanged. cf-scan-prep #14 (example crate) becomes the v2 example.

---

## Things v1 contributes that survive into v2

Not all of v1 is "proven failure"; the SDF + composition infrastructure is sound, only the demolding strategy was wrong. Reused unchanged:

- `cf-cast::CastSpec` data carrier (layer ordering, material density, anchor keys)
- `mold_cup` SDF construction (outer envelope around the scan)
- Layered shell construction (innermost → outer cumulative thickness offsets)
- Pour-volume Riemann-sum integration
- Mass-budget gate (`DEFAULT_MASS_BUDGET_KG`)
- Procedure.md base template + cure-table lookup (extended for per-piece, not rewritten)
- `mesh-offset::marching_cubes` + `mesh-printability::validate_for_printing` plumbing
- Atomic write at the meshing boundary
- F4 printability gate (blocking-critical filter)

v1 cf-cast stays in main for backward compatibility; users who actually want single-cup straight-pull (some specific use case may exist?) can still call it. v2 is the new default for everything cf-scan-prep produces.

---

## Risks

- **Ribbon SDF discontinuities at polyline segment boundaries** — the closest-point computation switches segments abruptly. Without smoothing, the ribbon surface has creases at each segment junction. Mitigation: project to the polyline using parameterized arc length + linear interpolation of tangent/binormal between adjacent samples. Bumps the LOC count for the ribbon SDF but produces a C¹-continuous surface.

- **Centerline doesn't pass through scan interior on highly curved/irregular scans** — for some pathological shapes (sharply concave, near-self-intersecting), the chord-direction cross-section centroids drift outside the scan's surface. The ribbon would then cut through air for parts of its length. Mitigation: validate centerline points are inside the mesh's SDF (i.e., SDF(centerline_point) < 0); flag/error if not. Recovery TBD — maybe a `--centerline-fallback` flag that uses the chord direction if cross-section centroids fail.

- **Per-piece marching-cubes runtime** — for fine cell sizes (1mm), each piece's SDF gets sampled on a 3D grid bounded by its AABB. 2-piece × 3-layer = 6 marching-cubes runs. v1's single-piece × 3-layer = 3 runs. Roughly 2× the CPU. Acceptable; not a blocker.

- **CSG numerical issues at the ribbon ∩ mold_cup intersection** — SDF intersection is min(a, b), which is correct geometrically but produces marching-cubes artifacts when the surfaces are coincident or near-tangent. Mitigation: add a small bias (~0.5 cell-size) to the ribbon SDF so the intersection is clean. v1 already uses this trick for the clip-body overlap (`CLIP_BODY_OVERLAP_M = 0.5 mm`).

---

## Slice ship log

- **2026-05-12** — Design doc opens (this commit). Pivots cf-cast architecture from straight-pull (v1, shipped in main) to curve-following multi-piece (v2). cf-scan-prep MVP cuts commits #10 + #11 (Mouth ext., Cleaned AABB — v1-specific concepts) and reshapes #12 (Save) to write the `.prep.toml` `[centerline]` block alongside the cleaned STL.
- **2026-05-13** — Step 3 (`cf-cast::SplitNormal` + `Ribbon` types) ships. New `cf-cast::ribbon` module (~470 LOC including docs + 12 tests). Types: `SplitNormal(Unit<Vector3<f64>>)` (default `+X`; `new` normalizes input + rejects zero/non-finite vectors via `Option`); `RibbonSegment { start, end, tangent, binormal }` (per-segment cached frame, all vectors unit-length); `Ribbon { points, segments, split_normal }` (owns the centerline polyline + cached frames); `RibbonError` (`InsufficientPoints` / `ZeroLengthSegment` / `TangentParallelToSplitNormal` — all three are user-facing geometric pathologies surfaced at construction time so SDF evaluation in Step 4 can assume well-formed frames). `Ribbon::new` validates: ≥2 points, segment length ≥ 1µm, `|tangent × N_split| ≥ 1µm`. Helper methods `arc_length()` (sum of segment lengths) + `max_tangent_rotation_rad()` (for piece-count selection at Step 5; `<60°` -> 2 pieces, `60-120°` -> 2 pieces + warning, `>120°` -> error). 12 tests pin: default `+X` split normal; input normalization + zero/NaN rejection; per-segment binormal computation (+X polyline × +Y split-normal -> +Z binormal); arc length sum; tangent rotation `0.0` for straight line + `π/2` for right-angle polyline; all three RibbonError variants. cf-cast tests now 55 (43 + 12 new ribbon); xtask grade cf-cast `coverage 95.5% A+ / docs A / clippy A / safety A / deps A / layer integrity A`. ~2 hr active. Next: Step 4 (Ribbon SDF + unit tests against analytic curves, ~4-6 hr).
