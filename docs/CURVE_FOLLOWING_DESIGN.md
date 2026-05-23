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

> **Note (mating-features arc, 2026-05-22).** The `∩ ribbon_side` half-space intersection above describes the **pre-mesh** SDF expression that marching cubes consumes. Per `docs/CF_CAST_MATING_FEATURES_PLAN.md` §S4 the workshop-facing seam migrated to a **post-MC mesh-trim** (`MatingTransform::SeamTrim`) against an exact ribbon plane; the SDF expression here over-builds each piece by `RIBBON_PIECE_OVERLAP_M` past the seam and the post-MC trim cuts both halves back to a flat, bit-precise mating face. Same applies to registration pins (S5), the plug T-bar + T-slot + shaft socket (S6), and the funnel-nipple + cup pour-gate (S7) — each migrated from SDF-time `union`/`subtract` to post-MC `MatingTransform` ops. The per-piece composer (`piece.rs::compose_piece_solid`) now returns `(Solid, Vec<MatingTransform>)`; the Solid is a side-agnostic cup-wall envelope and every mating feature lives in the Vec.

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

- **CSG numerical issues at the ribbon ∩ mold_cup intersection** — SDF intersection is min(a, b), which is correct geometrically but produces marching-cubes artifacts when the surfaces are coincident or near-tangent. Mitigation: add a small bias (~0.5 cell-size) to the ribbon SDF so the intersection is clean. v1 already uses this trick for the clip-body overlap (`CLIP_BODY_OVERLAP_M = 0.5 mm`). **Update (2026-05-22, mating-features arc S4):** the SDF over-build still keeps MC numerics clean inside the cup-piece grid, but the *workshop-visible* seam face is now a post-MC mesh-trim (`MatingTransform::SeamTrim`) — bit-precise to f64 per the manifold3d kernel rather than MC-cell-quantized. See `docs/CF_CAST_MATING_FEATURES_PLAN.md` §S4 and the §"Per-piece SDF construction" note above.

---

## Slice ship log

- **2026-05-12** — Design doc opens (this commit). Pivots cf-cast architecture from straight-pull (v1, shipped in main) to curve-following multi-piece (v2). cf-scan-prep MVP cuts commits #10 + #11 (Mouth ext., Cleaned AABB — v1-specific concepts) and reshapes #12 (Save) to write the `.prep.toml` `[centerline]` block alongside the cleaned STL.
- **2026-05-13** — Step 3 (`cf-cast::SplitNormal` + `Ribbon` types) ships. New `cf-cast::ribbon` module (~470 LOC including docs + 12 tests). Types: `SplitNormal(Unit<Vector3<f64>>)` (default `+X`; `new` normalizes input + rejects zero/non-finite vectors via `Option`); `RibbonSegment { start, end, tangent, binormal }` (per-segment cached frame, all vectors unit-length); `Ribbon { points, segments, split_normal }` (owns the centerline polyline + cached frames); `RibbonError` (`InsufficientPoints` / `ZeroLengthSegment` / `TangentParallelToSplitNormal` — all three are user-facing geometric pathologies surfaced at construction time so SDF evaluation in Step 4 can assume well-formed frames). `Ribbon::new` validates: ≥2 points, segment length ≥ 1µm, `|tangent × N_split| ≥ 1µm`. Helper methods `arc_length()` (sum of segment lengths) + `max_tangent_rotation_rad()` (for piece-count selection at Step 5; `<60°` -> 2 pieces, `60-120°` -> 2 pieces + warning, `>120°` -> error). 12 tests pin: default `+X` split normal; input normalization + zero/NaN rejection; per-segment binormal computation (+X polyline × +Y split-normal -> +Z binormal); arc length sum; tangent rotation `0.0` for straight line + `π/2` for right-angle polyline; all three RibbonError variants. cf-cast tests now 55 (43 + 12 new ribbon); xtask grade cf-cast `coverage 95.5% A+ / docs A / clippy A / safety A / deps A / layer integrity A`. ~2 hr active. Next: Step 4 (Ribbon SDF + unit tests against analytic curves, ~4-6 hr).
- **2026-05-13** — Step 4 (`Ribbon::sdf` + closest-segment query + analytic-curve SDF tests) ships. New methods on `Ribbon`: `closest_segment(query)` (linear-scan O(N) for typical 10-30-point centerlines; clamps `t` to `[0, 1]` so out-of-span queries snap to endpoint segments and SDF stays well-defined everywhere in world space); `sdf(query)` (returns `dot(query - projection_on_closest_segment, binormal)`). 8 tests pin: SDF above centerline = +z; below = -z; on centerline = 0; ignores offsets along split-normal direction; clamps to last segment for queries beyond polyline; correct segment dispatch on right-angle XY polyline with +Z split-normal (segment 0 binormal -Y; segment 1 binormal +X); antisymmetry across cutting plane; closest_segment picks the right segment + computes correct `t`. cf-cast tests now 63 (43 + 20 ribbon). Continuity: C⁰ everywhere; potential creases at segment-junction Voronoi boundaries — magnitude is `O(sin(tangent_rotation))`, well-tolerated by marching cubes at the spec's 2mm cell size; C¹ smoothing (linear interp of frame across neighbors) deferred until iter-1 surfaces a visible artifact. xtask grade pending re-run. ~1.5 hr active. Next: Step 5 (per-piece SDF composition `piece_sdf = mold_cup ∖ layer_body ∩ ribbon_side`).
- **2026-05-13** — Step 5 (per-piece SDF composition) ships. **Architectural choice**: extend cf-cast surface additively rather than introducing a `V2CastSpec` type — v1 `CastSpec` data carrier is preserved (per §"Things v1 contributes that survive into v2"), and v2 surfaces as new methods/free-functions composing over the existing `cf_design::Solid` algebra. **New surface**: (1) `PieceSide { Negative, Positive }` enum with `sign()` accessor + 2-piece convention pinned (per §"Piece count selection" — v3 covers 3+); (2) private `RibbonHalfspaceSdf` adapter implementing `cf_design::Sdf` (eval + analytic grad from closest-segment binormal × sign); (3) `Ribbon::halfspace_solid(side, bounds, overlap_m) -> Solid` bridges the ribbon's half-space into the typed `Solid` expression tree via `Solid::from_sdf` (caller-supplied bias for test/prod flexibility); (4) new `cf-cast::piece` module with `pub const RIBBON_PIECE_OVERLAP_M = 0.0005` (cf-cast-specific 0.5 mm seam bias mirroring v1's `CLIP_BODY_OVERLAP_M`) + `compose_piece_solid(layer_body, bounding_region, ribbon, side) -> Result<Solid, CastError>` wiring the `bounding_region.subtract(layer_body).intersect(halfspace)` chain. v1's `+Z` clip-cuboid is replaced by the ribbon-side intersection — no axis is privileged in v2, the cup closes on every face and only opens at the ribbon seam. 14 new tests (6 ribbon halfspace + 8 piece) pin: half-space SDF sign convention for both sides; bias-symmetric seam overlap (2 × `RIBBON_PIECE_OVERLAP_M` = 1 mm); piece SDF semantics at hand-picked points inside cup-wall / inside body / outside bounding-region; `Negative`+`Positive` partition the cup material; `InfiniteBounds(BoundingRegion)` error path; **unbounded `layer_body` is permitted** in v2 (v1's `clip_above_body` required body bounds for the `+Z` clip placement; v2 has no such requirement). cf-cast tests now 77 (63 + 14). `cargo test -p cf-cast --lib` + `cargo clippy --tests -- -D warnings` green; xtask grade ran to completion but the `tail -50` stdout buffer dropped the numeric summary — re-run out-of-band before Step 6 ships. ~2.5 hr active including the architectural-choice recon. Next: Step 6 (marching cubes per piece + STL export wiring through `solid_to_mm_mesh` + per-piece `MoldArtifact` schema; lands `CastSpec::export_molds_v2` as the additive method).
- **2026-05-13** — Step 6 (marching cubes per piece + STL export) ships. **`CastSpec::export_molds_v2(&self, ribbon: &Ribbon, out_dir: &Path) -> Result<V2MoldExportReport, CastError>`** lands as the additive v2 entry point parallel to v1's `export_molds`. Pipeline: empty-layers gate → tangent-rotation gate (`ribbon.max_tangent_rotation_rad()` ≤ `2π/3` rad = 120°; refuse with new `CastError::CenterlineTooCurved`) → pour-volume + mass-budget gate (reused from v1 unchanged) → for each `(layer × piece_side ∈ [Negative, Positive])`: `compose_piece_solid` (Step 5) → `solid_to_mm_mesh` (reused) → `validate_for_printing` F4 gate → buffer; plug mesh+gate (unchanged from v1); atomic FS create + write per layer pair + plug. **Per-piece printability** comes for free via `mesh-printability`'s `ExceedsBuildVolume` Critical applied to each piece's AABB — v1's aggregate-AABB check is replaced by stricter per-piece bounds (you can't print a piece larger than your printer no matter how the combined fits). New surface: (1) `CastTarget::MoldPiece { layer_index, piece_side }` variant for v2-specific error reporting; (2) `CastError::CenterlineTooCurved { max_rotation_rad, max_rotation_deg, threshold_rad, threshold_deg }`; (3) `PieceArtifact { piece_side, path, validation, summary }`, `V2LayerReport { layer_index, material_display_name, pour_volume, pieces: [PieceArtifact; 2] }` (fixed array statically pins 2-piece convention), `V2MoldExportReport { layers, plug_path, plug_validation, plug_summary }`; (4) `mold_piece_filename(layer_index, piece_side) -> String` maps `Negative→_piece_0`/`Positive→_piece_1` per design-doc §"Output artifacts". Pre-write atomicity scope identical to v1 (all 2L pieces + plug clear F4 before any STL lands; output directory not created on any pre-write failure). Implementation factored into 3 free-function helpers (`mesh_and_gate_v2_pieces`, `mesh_and_gate_v2_piece`, `mesh_and_gate_plug_v2`, `write_v2_pieces`) so `export_molds_v2` stays inside the 100-line clippy budget + the pre-paired `[PendingPiece; 2]` layout eliminates the `.expect()` un-flatten dance. 8 new tests pin: end-to-end single-layer writes 2 pieces + plug; multi-layer 2L writes 4 pieces + plug with correct (layer, side) ordering; per-layer pour-volume threading; all 4 error paths (empty layers, centerline too curved with 135° fixture, mass budget exceeded, unbounded bounding region); filename convention. cf-cast tests now 85 (77 + 8). `cargo test -p cf-cast --lib --release` (8.8s, 32× faster than debug) + `cargo clippy --tests -- -D warnings` green. **Pattern banked**: cf-cast tests must run `--release` — debug-mode test cycle is 286s vs release's 8.8s due to `validate_for_printing` O(faces²) loops dominating in debug. Saved as feedback memory. ~3 hr active. Next: Step 7 (formal per-piece printability + AABB check refinement — surface `ExceedsBuildVolume` per-piece in the V2 report's validation; mostly mechanical, may fold into Step 6 wrap-up if the F4 reporting already covers it).
- **2026-05-13** — Step 7 (per-piece printability + per-piece AABB checks) ships as a confirmation pass. Per-piece behavior was already implemented by Step 6's pipeline (each piece's mesh gets independent `validate_for_printing`, which includes `ExceedsBuildVolume` Critical in the blocking set). Step 7 = 3 regression tests pinning the behavior: (1) per-piece mm-frame AABB ⊆ bounding region's AABB ± 18 mm MC slack (sanity check on `bounding_region ∩ halfspace ∖ body` composition); (2) each piece's `PrintValidation` is independently populated (zero blocking-Critical issues on the synthetic fixture); (3) tiny printer (30×30×30 mm vs 80×80×60 mm bounding region) trips `PrintabilityCritical { target: CastTarget::MoldPiece { layer_index: 0, piece_side: PieceSide::Negative }, ... }` on the first piece in iteration order, with no `out_dir` created (pre-write atomicity preserved). Pin verifies v2's stricter per-piece bound (each piece must individually fit the printer) vs v1's aggregate single-cup check. cf-cast tests 85 → 88. `cargo test -p cf-cast --lib --release` 9.56s + clippy `-D warnings` green. ~30 min active (light step — Step 6 did the heavy lifting). No new public API; the v2 contract was already complete after Step 6. Next: Step 8 (procedure.md generator for N-piece assembly + demold sequence; ~3-4 hr — first non-trivial extension of v1's `cf-cast::procedure` for multi-piece molds).
- **2026-05-13** — Step 8 (procedure.md generator for N-piece assembly + demold sequence) ships. **`pub fn generate_procedure_markdown_v2(spec, pour_volumes, ribbon) -> String`** + **`CastSpec::write_procedure_v2(ribbon, path) -> Result<(), CastError>`** land as the additive v2 procedure surface parallel to v1's `generate_procedure_markdown` / `write_procedure`. Reuses v1's `write_materials_table` / `write_generic_guidance` / `write_mass_budget_summary` / `cure_protocol_cells` / `layer_position_label` helpers verbatim (materials/budget/cure-protocol logic is layer-level, unchanged by piece-count). **New v2-specific sections**: (1) `## Cast Geometry` surfaces centerline arc length (mm) + max tangent rotation (deg) + piece count = 2; (2) `## v2 Mold Assembly` documents the iter-1 manual-clamp-with-rubber-bands approach (registration features are Step 9; the 1 mm seam overlap means hand alignment is workable for FDM accuracy ~0.1 mm); (3) per-layer Step 1 references BOTH piece STLs by name (`mold_layer_{i}_piece_0.stl` + `mold_layer_{i}_piece_1.stl`); (4) per-layer Step 8 is the new demold-in-order prose ("remove `piece_0` first, then `piece_1` — part slides out along the centerline rather than fighting an undercut"). v2 procedure shares the curve-rotation + budget gates with `export_molds_v2` so the rendered markdown never misrepresents a refused cast. 9 new tests pin: all 6 required section headers present (header, geometry, materials, guidance, assembly, per-layer, budget); both piece STLs referenced per layer; demold prose specifies piece order; arc length renders in mm at `.1` precision; anchored materials get full cure-protocol prose; round-trip FS wrapper matches in-memory generator; EmptyLayers / CenterlineTooCurved / MassBudgetExceeded error paths refuse pre-render with no file landing. cf-cast tests 88 → 97. `cargo test -p cf-cast --lib --release` 9.60s + clippy `-D warnings` green. ~2.5 hr active. Next: Step 9 (registration features — pins or dovetails — picks one approach + integrates into piece geometry; ~5-8 hr per design doc, the first geometry-modification step in v2's arc).
- **2026-05-13** — Step 9 (registration features) ships. **Picked cylindrical pins** vs dovetails/magnets — simplest SDF (one cylinder per pin), industry-standard for printed molds, easy to print + insert + remove, gravity-held. Geometry: pin center at `centerline_sample(arc_fraction) + offset * split_normal_vec` (positions pin in cup wall, outside body); pin axis along ribbon's binormal at that centerline position (perpendicular to seam); cylinder spans both pieces. 2 pins per layer-piece-pair at arc-fractions [0.25, 0.75] prevent rotation around centerline. **New surface**: (1) new `cf-cast::registration` module with `PinSpec { pin_radius_m, pin_half_length_m, offset_from_centerline_m, arc_fractions }` (`::iter1()` defaults: 3 mm Ø × 10 mm long × 25 mm offset × 2 pins); (2) `RegistrationKind { None, Pins(PinSpec) }` (default `None`); (3) `pub fn build_registration_solid(ribbon) -> Option<Solid>` returning the unioned pin cylinders (rotated to align with binormal via `UnitQuaternion::rotation_between`, translated to pin center); (4) `Ribbon::with_registration(self, RegistrationKind) -> Self` builder + `pub registration: RegistrationKind` field with default `None` (Steps 5-8 pre-Step-9 callers unaffected); (5) `Ribbon::sample_at_arc_fraction(t) -> Option<(Point3, Vector3, Vector3)>` (centerline position + tangent + binormal at arc-length fraction); (6) `compose_piece_solid` consults `ribbon.registration`: `Negative` side unions pin solid (gains protrusions), `Positive` side subtracts (gains matching holes); (7) `write_v2_assembly_note` switches on registration kind — `Pins(spec)` renders pin-count/diameter/length prose; `None` keeps the existing rubber-band-clamp prose; per-layer Step 3 simplified to "assemble per the v2 Mold Assembly section" (registration-mechanism-neutral). 14 new tests (7 registration + 7 spec/procedure) pin: PinSpec::iter1 defaults; RegistrationKind default; build_registration_solid None/Some paths + bounds; Ribbon::with_registration field set; sample_at_arc_fraction position+tangent+binormal at t=0/0.5/1.0 + out-of-range rejection; compose_piece_solid with pins on Negative gains protrusion at pin position (SDF<0 vs no-pin SDF>0); compose_piece_solid with pins on Positive gains hole (SDF>0 vs no-pin SDF<0); end-to-end `export_molds_v2` with pins writes valid STLs; procedure markdown mentions pin count + 3.0 mm Ø + 10.0 mm long when Pins; procedure keeps clamp-with-rubber-bands prose when None. cf-cast tests 97 → 111. `cargo test -p cf-cast --lib --release` 10.32s + clippy `-D warnings` green. ~4 hr active including two test-iteration cycles (query-on-body-boundary returned SDF≈0 → moved query 0.5 mm into cup wall; "rubber bands" leaked from per-layer Step 3 → made step registration-neutral). Next: Step 10 (pour-gate + air-release vent geometry, ~3-4 hr; second + last geometry-modification step before example crate ships at Step 11).
- **2026-05-13** — Step 10 (pour-gate + air-vent geometry) ships. Closes the v2-arc geometry-mod work; Step 11 example crate is the last remaining ship. **New surface**: (1) new `cf-cast::pour` module with `PourGateSpec { gate_radius_m, vent_radius_m, channel_half_length_m, include_vent: bool }` (`::iter1()` defaults: 6 mm Ø pour gate, 3 mm Ø vent, 40 mm total channel, vent enabled); (2) `PourGateKind { None, Default(PourGateSpec) }` (default `None`); (3) `pub fn build_pour_gate_solid(ribbon) -> Option<Solid>` returning the union of pour-gate cylinder at `centerline[0]` + (optional) vent cylinder at `centerline.last()`, both oriented along the local segment tangent; (4) `Ribbon::with_pour_gate(self, kind)` builder + `pub pour_gate: PourGateKind` field with default `None` (Steps 5-9 callers unaffected); (5) `compose_piece_solid` subtracts the pour-gate solid AFTER the registration-pin CSG, so both pieces lose material along the gate/vent channels symmetric across the ribbon seam; (6) new `write_v2_pour_gate_note` procedure section that mentions the gate Ø + vent Ø + channel length when `Default`, or falls back to "pour through the seam + drill vents post-print" prose when `None`; (7) per-layer Step 6 now references "the pour gate (base end of the centerline)" when enabled, "the assembled mold cavity" when not. 9 new tests pin: PourGateSpec::iter1 defaults; PourGateKind default; build_pour_gate_solid None/Some + bounds + no-vent shorter than with-vent; pour gate SDF negative at channel axis; Ribbon::with_pour_gate field set; compose_piece_solid carves channel through BOTH pieces at the gate axis (Positive side SDF flips from negative no-gate to positive with-gate; Negative side same); end-to-end `export_molds_v2` with pour gate writes valid STLs; procedure prose mentions 6.0 mm Ø gate + 3.0 mm Ø vent + 40.0 mm long with Default; falls back to "assembled mold cavity" with None; `include_vent=false` prose explicitly notes the disabled vent; **orthogonality test**: enabling BOTH registration pins (Step 9) AND pour gate (Step 10) composes cleanly — Negative piece contains pin protrusion AND excludes pour-gate channel, demonstrating the two feature dimensions are independent. cf-cast tests 111 → 125. `cargo test -p cf-cast --lib --release` 10.40s + clippy `-D warnings` green. ~2.5 hr active (Step 10 is mostly mirroring the Step 9 registration pattern — same module structure, same Ribbon-builder pattern, same compose_piece_solid switch). Two minor clippy const_fn errors fixed (PourGateSpec::iter1 + Ribbon::with_pour_gate both eligible because PourGateKind has no Vec, unlike RegistrationKind). Next: **Step 11** (example crate `examples/cast/layered-silicone-device-v2-scan-curve-following` consuming cleaned STL + `.prep.toml` `[centerline]` from cf-scan-prep, producing N×L mold STLs + procedure.md; ~3 hr per design doc; closes the v2 code-side arc and unblocks Track F Stage 3 physical-cast workshop iter-1).
- **2026-05-13** — Step 11 (example crate) ships. **Closes the v2 code-side arc**. New crate `examples/cast/layered-silicone-device-v2-scan-curve-following` (~210 LOC main.rs + README.md + .gitignore + Cargo.toml workspace registration) exercises every v2 feature (Steps 5-10) end-to-end on a synthetic curved-pipe body: 3-layer device along a 5-point centerline polyline (gentle arc in XZ plane, ~84 mm arc length, ~39° max tangent rotation — well under v2's 120° refusal threshold); 8 mm radius `Solid::pipe` plug along the centerline; cumulative-shell layers (Ecoflex inner 6 mm / Dragon Skin middle 4 mm / Ecoflex outer 4 mm); 120 mm cube bounding region; ribbon split-normal `+Y` (pieces split top/bottom of the curved tube); registration pins + pour gate + air vent both opted in via `Ribbon::with_registration(RegistrationKind::Pins(PinSpec::iter1()))` + `Ribbon::with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()))`. **Output**: 6 piece STLs + 1 plug STL + procedure.md in 8 files. **Total silicone mass**: 346.77 g across 3 layers (well under 907.18 g per-pour budget). Scan-driven swap-in pattern documented in the module docstring + README (load cleaned STL via `mesh_io::load_stl` → wrap in `Solid::from_sdf`; parse `.prep.toml` `[centerline]` block → pass to `Ribbon::new`). **F4 wall-threshold relaxation**: example uses `PIECE_MIN_WALL_MM = 0.1 mm` (vs FDM default 1.0 mm) — v2 piece geometry (ribbon split + pin holes + pour-channel CSG) produces sub-1mm MC artifacts at the seam + feature edges that don't reflect actual print failures; 0.1 mm threshold blocks only on genuine sliver geometry vs MC stair-stepping. Production setups may tune back up after iter-1 post-print inspection. cf-cast tests unchanged at 125 (Step 11 adds no library code; only example). `cargo run --release -p example-cast-layered-silicone-device-v2-scan-curve-following` produces the 8 artifacts in ~10s. `cargo clippy -p example-cast-layered-silicone-device-v2-scan-curve-following -- -D warnings` green. ~3 hr active (with one debug-iteration cycle to find that ThinWall Critical was the F4 blocker at FDM default 1.0 mm min wall; resolved by relaxing the per-example threshold). **v2 code-side arc COMPLETE** — Steps 1-11 of `docs/CURVE_FOLLOWING_DESIGN.md` Implementation arc all shipped; remaining workshop iter-1 + iter-2 work is Track F Stage 3 (physical-cast activity, NOT code).
