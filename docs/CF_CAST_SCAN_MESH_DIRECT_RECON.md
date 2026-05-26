# cf-cast scan-mesh-direct recon (2026-05-26)

**Status:** scaffold. Triggered by workshop user's cf-view + Orca
Slicer smoke on the post-§Q-4-S1 2 mm full-config STLs
(`~/scans/cast_iter1_post_q4_s1_plug_trim_2mm/`):

> *"the actual reference processed scan its using to make the plug
> should be completely flat. it was trimmed in the cf scan processing
> with the cap plane/trim floor/regenerate floor workflow."* — comparing
> the smooth processed scan mesh (`~/scans/sock_over_capsule.cleaned.stl`,
> ~0.66 mm avg edge length per [[project-cf-cast-geometry-crispness-s0]])
> against cf-cast's plug + cup-piece body output, which shows visible
> MC stair-step + radial striping at 2 mm cell scale.

§Q-4 S1 (dev `adb8e8ef`) shipped the cap-plane trim's paradigm-safe
re-enable + production gates: `trim_by_plane` produces a mathematically
flat plug bottom FACE at the cap-plane. But cf-view + Orca smoke
confirms the workshop visual gate — *"plug matches reference scan
visually"* — is **NOT** closed by S1 alone:

- The flat trim face's PERIMETER traces the cup-wall's MC-quantized
  lateral polygon, producing stair-step edge.
- The plug body's curved lateral surface carries radial MC-quantization
  "striping" + isolated facet patches.
- Both pre-exist §Q-4 (verified via byte-near-identical pre-§Q-4 2 mm
  full baseline at `~/scans/cast_iter1_q1_phase1_2mm_full/`, byte
  delta −1.6 KB / −0.16% on `plug_layer_0.stl`).

§Q-4 S2 (cup-piece cavity floor slab union) and S3 (mating-face
defensive trim) would have the **same fundamental ceiling**: planar
trims flatten faces but cannot smooth the surrounding MC-quantized
lateral curves. The recon-acknowledged "honest scope" gap
(`docs/CF_CAST_Q4_PLANAR_TRIM_RECON.md` §Q4-3-c) is now empirically
load-bearing.

This recon proposes an **architectural pivot**: for scan-derived body
surfaces (plug body, per-layer body, cup-wall outer), bypass cf-cast's
current SDF→MC re-meshing where possible by routing through scan-mesh-
direct paths.

**Important up-front honest scope** (caught by cold-read): the
existing `mesh-offset` crate's `offset_mesh` IS itself an SDF→MC
pipeline internally (`mesh/mesh-offset/src/offset.rs:195-218` — builds
SDF, samples on a grid, runs marching cubes). So the pivot only fully
eliminates MC for the **zero-offset case** (plug body = scan mesh
AS-IS). Per-layer body + cup-wall outer use `mesh-offset` and remain
MC-quantized at `OffsetConfig::resolution`. The pivot's value for the
non-zero-offset surfaces is the ability to tune mesh-offset's cell
size INDEPENDENTLY of cf-cast's main `mesh_cell_size_m` (e.g., 0.5 mm
resolution for layer offsets while bulk cf-cast pieces stay at 2 mm).

This means workshop PR-merge gate #3 (plug bottom flat) closes
cleanly; gates #2 (interior floor flat) and #4 (mating face flat)
have a more nuanced trade-off (mesh-offset MC resolution × wall-clock).
See §SMD-8 for the updated PR-gate mapping.

## Why this recon (workshop empirical signal)

Workshop user comparison:

| Reference scan | cf-cast output (2 mm cells) | Visual gap |
|---|---|---|
| `~/scans/sock_over_capsule.cleaned.stl` — smooth body, clean flat cap-plane (cf-scan-prep trimmed it) | `cast_iter1_post_q4_s1_plug_trim_2mm/plug_layer_2.stl` — radial MC stripes on body lateral + stair-step perimeter on cap-plane edge | Visible at 2 mm; would still be visible at 1 mm (~3× coarser than scan's 0.66 mm avg edge); only matches reference at scan-mesh-native resolution |

**Workshop user PR-merge gate** (recapped from
[[project-cf-cast-q4-planar-trim-recon]]):

1. Flange all the way around — separate §F arc; bookmarked.
2. **Interior mold floor flat** — §Q-4 S2 candidate; planar-trim ceiling.
3. **Bottom of plug flat** — §Q-4 S1 shipped; planar-trim ceiling
   (face flat, edge jagged).
4. **Flat mating faces** — §Q-4 S3 candidate; planar-trim ceiling.

The scan-mesh-direct pivot addresses the **2 mm MC quantization root
cause** for the zero-offset case (plug body, #3 PR-blocker) and
provides an independently-tunable finer cell size for the offset
cases (per-layer body + cup-wall outer, #2 + #4 PR-blockers). The
planar-trim ceiling is removed for #3, narrowed but not eliminated
for #2 + #4 (depending on offset MC resolution chosen).

## Out of scope

- **Gasket molds**, **funnel**, **platform** — these are SDF-side
  cuboid/extrude geometry, NOT scan-derived. SDF→MC is the right tool
  for them; their workshop appearance is acceptable at 2 mm cells
  (gasket trays are bulk geometry; channels at 0.5 mm cell override
  per [[project-cf-cast-geometry-crispness-s0]]).
- **Pinned-floor / cap-plane truncation** — the cf-scan-prep cap-plane
  truncation already cuts the scan mesh at the cap-plane; the pivot
  uses that pre-trimmed mesh as-is. No new cap-plane SDF logic.
- **Plug-floor-lock pyramid** — already mesh-CSG `UnionTruncatedPyramid`
  post-MC; pivot KEEPS this primitive (its post-MC mesh-CSG path
  works with scan-mesh-resolution input too).
- **Registration pins / pour-gate carve / cup-pin sockets** — all
  existing mesh-CSG primitives stay; they consume any input mesh
  resolution.
- **§Q-1 cup-wall body-tracking shell SDF** — REPLACED by mesh-side
  offset (the cup-wall outer becomes a mesh-offset of the body
  surface). The §Q-1 fix's MC-topology safety is no longer needed
  because we skip MC for the cup-wall outer entirely.
- **§Q-4 S2/S3 (planar-trim primitives for cavity floor + mating face)**
  — superseded by this pivot's scan-mesh-direct flow. The planar-trim
  primitives' problem (face flat, edge jagged) is solved by preserving
  scan resolution at the edges.

## §SMD-1 Root cause analysis

### §SMD-1-a The MC re-meshing step destroys scan resolution

cf-cast's current pipeline for any scan-derived surface:

```
scan.stl (~0.66 mm avg edge, ~168k faces)
  → TriMeshDistance (mesh_sdf — exact-distance SDF)
  → pinned_floor_shell (SDF composition: scan_sdf offset + cap-plane intersect)
  → marching_cubes (sample SDF on 2 mm grid, ~80k cells in scan AABB ≈ 71×71×127 mm)
  → IndexedMesh (~10-20k faces — 8-17× COARSER than scan input)
  → mesh-CSG ops (cap-plane trim, pyramid union)
  → STL
```

The MC step at 2 mm cells coarsens the mesh by ~3× in linear edge
length (0.66 mm → 2 mm) and ~10× in face count. The visible workshop
artifacts (radial striping, stair-step perimeters, "fish-scale" facet
patches on body lateral) are all MC quantization at this scale.

Empirical confirmation (§Q-11 S0,
[[project-cf-cast-geometry-crispness-s0]]):
- Scan mesh = 167,898 faces / AABB ~71×71×127 mm
- ~0.66 mm avg edge length (sub-mm resolution)
- 4.5× linear coarsening at 3 mm MC cells; 3× at 2 mm

### §SMD-1-b The MC step isn't doing useful work for scan-derived surfaces

cf-cast originally re-meshes through SDF→MC because OTHER pieces (cup
wall outer, gasket molds, funnel, platform) ARE SDF-composed
primitives that NEED MC to produce a mesh. The scan mesh is treated
uniformly through the same pipeline FOR CONSISTENCY, not because it
needs the round-trip.

For the plug body and per-layer body cavity surfaces, MC is a pure
RESOLUTION-LOSS step:
- Input: scan mesh at native resolution.
- SDF interpretation: exact-distance field (`TriMeshDistance` is
  lossless — it knows about every triangle).
- MC sampling: discards the exact-distance information, keeps only
  zero-crossings on a coarse grid.
- Output: a coarser mesh that approximates the scan surface.

If the downstream consumer (mesh-CSG ops) can accept the scan mesh
directly, the MC step is pure loss.

### §SMD-1-c What about per-layer offset?

The current pipeline uses `pinned_floor_shell(... cumulative_thickness)`
to produce each layer's body. The SDF offset is the load-bearing
operation: `body_offset(d).evaluate(p) = body_sdf.evaluate(p) - d`.
This shifts the body's zero-isosurface outward by `d`.

The pivot replaces the SDF offset with a mesh-side offset via the
existing **`mesh-offset` crate** (`mesh/mesh-offset/src/offset.rs:155`,
`offset_mesh(mesh, distance)`). Mesh-offset produces a new
triangulation by:
- Computing per-vertex outward normals.
- Moving each vertex along its normal by `distance`.
- Optionally rebuilding via SDF-snap to handle self-intersections
  in highly-curved regions (depends on the offset distance vs local
  curvature radius).

For sock-shaped scan bodies with the workshop's iter-1 thicknesses
(6 + 4 + 4 = 14 mm max cumulative), curvature radius is dominated by
the body's overall shape (~30 mm scale) which is much greater than
14 mm — offset should be self-intersection-free. To be confirmed
empirically in S0 (see §SMD-7).

## §SMD-2 Candidate architectures

### Candidate A: scan-mesh-direct + mesh-offset (PICKED)

Use scan mesh directly as the Manifold3D starting point for the
plug body + per-layer body. Apply mesh-CSG ops on top:

```
scan.stl (~0.66 mm avg edge, ~168k faces)
  → IndexedMesh (load directly)
  → mesh-offset(d_layer_N)  [for layer N body; identity for plug body]
  → cap-plane truncation (already in scan via cf-scan-prep; OR mesh-side trim_by_plane)
  → mesh-CSG ops (pyramid union, pin sockets, etc.)
  → STL
```

**Wins:**
- Plug body (zero-offset case): scan resolution preserved fully; no
  MC step at all → workshop PR-gate #3 closes cleanly.
- Per-layer body + cup-wall outer: routed through `mesh-offset` which
  has a **configurable internal MC resolution** (`OffsetConfig::resolution`).
  Setting this finer than cf-cast's `mesh_cell_size_m` narrows the
  workshop visual gap on cavity floor + mating face perimeter (PR-gates
  #2 + #4); cost scales as the resolution is dropped.
- All existing mesh-CSG primitives already work mesh-side — no new
  composition machinery.

**Costs:**
- `mesh-offset` IS an SDF→MC pipeline internally
  (`mesh/mesh-offset/src/offset.rs:195-218` — SDF build via
  `TriMeshDistance` or `flood_filled_sdf`, then `marching_cubes`).
  So offset operations still carry MC quantization at
  `OffsetConfig::resolution`. To close PR-gates #2 + #4 visually, the
  offset MC resolution likely needs to be in the 0.5-1.0 mm range
  (3-2× finer than cf-cast's main 2 mm).
- Offset MC at 0.5 mm in the scan AABB ≈ 71×71×127 mm has ~4.6M cells
  (cubic growth) — wall-clock cost per offset operation is non-trivial.
  Mitigated by mesh-offset's `OffsetConfig::preview` vs production
  presets (TBD: characterize empirically in S0).
- Mesh-offset has self-intersection edge cases at high curvature ×
  large offset — needs empirical S0 verification on workshop's
  iter-1 scan at 4-14-19 mm offsets.
- Output STLs become larger (the offset-output mesh face count
  depends on offset MC resolution; finer = more faces). Slicer cost
  may grow.
- The cup-wall outer (currently §Q-1 `CupWallShellSdf` body-tracking
  shell) becomes a mesh-offset of the outermost body by
  `wall_thickness_m` — needs careful integration with §Q-1's safety
  rationale.
- `mesh-offset`'s offset direction depends on triangle winding; needs
  winding-consistent input (post-§Q-5 winding fix this should be
  fine).

### Candidate A2: scan-mesh-direct + vertex-displacement offset (variant of A)

Same as Candidate A for the zero-offset plug body. For per-layer body
+ cup-wall outer, replace `mesh-offset` with a **pure vertex-displacement
offset**: compute per-vertex outward normals on the scan mesh, move
each vertex along its normal by the offset distance. No SDF, no MC,
no resolution loss.

**Wins (additional vs A):**
- Truly preserves scan resolution for offset operations too — no
  internal MC.
- All workshop visual PR-gates (#2 + #3 + #4) close at scan
  resolution.
- Wall-clock minimal — `O(n)` in vertex count per offset.

**Costs (additional vs A):**
- Self-intersection risk grows with offset distance × local curvature.
  Sock geometry's curvature radius (~30 mm) vs cumulative offset
  (14-19 mm) sits in a borderline regime — likely OK for most of the
  body, may fail at the toe end where local curvature radius
  approaches the offset distance.
- Self-intersection repair requires either a fallback to `mesh-offset`
  (back to internal MC) for failing regions OR a new repair pass.
- No existing crate provides pure vertex-displacement offset —
  ~50-100 LOC new helper.
- Requires post-offset manifoldness verification (a vertex-displaced
  mesh isn't guaranteed manifold).

**Decision tree** (resolved empirically in S0):
- If pure vertex-displacement is self-intersection-free at the
  workshop's iter-1 offset scales → use Candidate A2.
- If pure vertex-displacement fails at some offset scale → use
  Candidate A with mesh-offset's `OffsetConfig::resolution` set fine
  enough to close the workshop visual gap.

### Candidate B: Finer MC cells (~1 mm or sub-mm)

Drop MC cell size from 2 mm to ~1 mm; same architecture otherwise.

**Wins:**
- No new code beyond cell-size config.
- Architecturally minimal.

**Costs:**
- Wall-clock blow-up: 2 mm → 1 mm is 8× more MC cells per layer. Post-
  §Q-1 + §Q-4 S1 baseline = 3:45 wall at 2 mm full; 1 mm would be
  ~30 min. Workshop's design-iteration cadence breaks.
- Still 1.5× COARSER than scan resolution (1 mm vs 0.66 mm) — visual
  gap narrows but doesn't close.
- Doesn't address the perimeter stair-stepping problem; just shrinks
  the steps.

### Candidate C: Adaptive MC near scan-derived surfaces

Use coarse MC (2 mm) for bulk geometry + fine MC (~0.5 mm) near
scan-derived body surfaces only.

**Wins:**
- Scan resolution mostly preserved.
- Wall-clock smaller than Candidate B (fine cells only near surfaces).

**Costs:**
- Requires adaptive-MC machinery in `mesh-offset` or a new crate.
  Substantial implementation cost (~1000+ LOC of new MC infrastructure).
- Still quantizes onto a grid — adaptive MC at 0.5 mm cells matches
  scan's ~0.66 mm avg edge length on average but produces stair-step
  edges at cell-boundary alignment regardless. Would need sub-mm
  cells (e.g., 0.3 mm) to dominate scan resolution; cost grows
  cubically with cell-size reduction even in the locally-fine region.
- Doesn't eliminate the boundary stair-stepping problem fundamentally;
  just shrinks the steps.

### Candidate D: Dual contouring or alternative mesher

Replace MC with dual contouring (or similar) which can produce
sharp-feature-aware output.

**Wins:**
- Better edge / sharp-feature preservation than MC.

**Costs:**
- Massive implementation cost. New mesher in `mesh-offset`.
- Doesn't fundamentally change the resolution-loss problem — still
  samples on a grid.

### Why Candidate A (with A2 as upgrade path) wins

Workshop user's empirical signal is clear: the **scan mesh ALREADY
looks like the workshop wants the plug to look**. The cheapest way
to get the workshop-desired result for the plug body is to use the
scan mesh AS-IS — that part of Candidate A is unambiguous.

For per-layer body + cup-wall offset operations:
- Candidate A (mesh-offset) carries an MC quantization step but at a
  configurable resolution; can be tuned to narrow the visual gap.
- Candidate A2 (pure vertex displacement) would close the gap fully
  but carries a self-intersection risk that needs empirical validation.
- Candidates B / C / D all keep MC in the main pipeline with various
  trade-offs.

Picked direction: **start with Candidate A** (lowest implementation
risk; mesh-offset crate already exists + battle-tested at smaller
scales). **Promote to A2 in S0/S1 if mesh-offset's quantization
proves workshop-visible** at the largest workshop-acceptable
`OffsetConfig::resolution`. The S0 spike's empirical output decides
which path to pursue beyond the plug body.

Worst case: if mesh-offset's internal MC produces workshop-visible
artifacts AND A2's vertex-displacement self-intersects, fall back to
Candidate A with a thinner SDF→MC fallback OR scope the pivot to the
plug body only (still closes PR-gate #3 — a real workshop win).

## §SMD-3 Paradigm-boundary consistency

Per [[project-cf-cast-sdf-meshcsg-paradigm-boundary]]: bulk-welded
geometry → SDF/MC; fine mating-feature primitives → mesh-CSG.

The pivot moves scan-derived bulk geometry from SDF/MC to **scan-mesh-
direct**. This is a third paradigm box:

| Geometry kind | Pre-pivot | Post-pivot |
|---|---|---|
| Scan-derived bulk (plug body, per-layer body cavity, cup-wall outer) | SDF/MC | **scan-mesh-direct** |
| SDF-composed bulk (gasket molds, funnel, platform, plug-floor-lock socket SDF) | SDF/MC | SDF/MC (unchanged) |
| Fine mating-feature primitives (pyramid, pins, pour-gate carve, cap-plane trim) | mesh-CSG | mesh-CSG (unchanged) |

The pivot does NOT add a new paradigm-boundary mode — it just routes
scan-derived surfaces along a shorter path. The mesh-CSG ops
(post-pivot) compose scan-mesh-resolution input with primitive
mesh-CSG primitives; manifold3d already handles arbitrary mesh
resolution.

**No new WELDED-TO-BULK exposure**: the mesh-CSG primitives'
paradigm-boundary safety (recon-4 (P) §F-2) is independent of input
mesh resolution. The §Q-4 S1 plug-cap-trim with 1 µm offset bias
ports cleanly to scan-mesh input — the trim plane is still 1 µm OFF
the body's cap-plane (whether the cap-plane lives in SDF or mesh
form). The §G-7 `extend_near_end` pattern for the plug-shaft +
T-bar (recon-4 (P) §F-2 worked examples) similarly ports unchanged.

## §SMD-4 Implementation arc

### S0 — Empirical offset feasibility + resolution spike (no commit)

Verify two load-bearing assumptions for the per-layer + cup-wall
offset operations: (1) mesh-offset's quality at workshop-acceptable
resolutions, (2) pure vertex-displacement viability.

**S0a — mesh-offset characterization (Candidate A)**:
- Load `~/scans/sock_over_capsule.cleaned.stl` via mesh-io.
- Run `offset_mesh(scan, 0.006, OffsetConfig { resolution: r, .. })`
  for r ∈ {0.002, 0.001, 0.0005} (2 mm, 1 mm, 0.5 mm).
- Run same sweep at offsets 0.014, 0.019 (cumulative + cup-wall).
- Measure: wall-clock per offset operation; output face count;
  workshop visual smoothness (cf-view smoke).
- Empirical question: what's the largest `r` that produces
  workshop-acceptable smoothness for #2 + #4?
- Verify manifoldness via existing `mesh-repair::detect_self_intersections`
  post-§S-9 BVH at O(n log n).

**S0b — vertex-displacement viability spike (Candidate A2)**:
- Write a probe binary or unit test:
  - Compute per-vertex outward normals on scan mesh (area-weighted
    face-normal average).
  - Displace each vertex by offset distance along its normal.
  - Run self-intersection check on the displaced mesh.
- Repeat for offsets 0.006, 0.014, 0.019.
- Empirical question: at what offset distance (if any) does pure
  vertex-displacement first produce a self-intersection on workshop's
  iter-1 sock geometry?

**Decision matrix from S0 output**:
- A2 self-intersection-free at all offsets → use A2 (closes all PR-gates).
- A2 self-intersection-free at small offsets only → use A2 for those,
  fall back to A (with finest workshop-acceptable resolution from
  S0a) for the rest.
- A2 unusable at any production offset → use A; pick `r` from S0a's
  smoothness sweep.
- A's wall-clock at workshop-acceptable `r` exceeds design-iteration
  cadence → fall back to plug-body-only pivot (PR-gate #3 only).

**No code change in S0** — purely diagnostic + decision-locking.

### S1 — plug_layer_0 scan-mesh-direct (probe spike, zero-offset case)

**Important architectural note** (caught by cold-read): cf-cast's
per-layer plug derivation in `derive.rs:255-262` builds `spec.plug`
with offset `-cavity_inset_m` (≈ 0 for inline-layers), used for
layer 0's plug. For layers N > 0, the plug is `layers[N-1].body`,
which is `scan + cumulative_thickness_through_N-1`. Concretely with
iter-1 thicknesses (6+4+4 mm):
- plug_layer_0 = scan body, zero offset
- plug_layer_1 = scan body + 6 mm offset
- plug_layer_2 = scan body + 10 mm offset

S1 scopes to **plug_layer_0 only** — the only true zero-offset case.
That's the smallest possible commitment that demonstrates "scan
mesh AS-IS through mesh-CSG → STL" works. ~100 LOC.

- New `cf-cast::scan_mesh_direct::build_plug_body_mesh(scan_mesh, cap_planes)`
  function: loads scan mesh, trims at cap-planes (using
  `manifold3d::trim_by_plane` directly on the mesh, OR relying on
  cf-scan-prep's pre-trimmed cap), returns `IndexedMesh`.
- `CastSpec::export_molds_v2` plug-emission branch: if a feature flag
  / config knob is set + AND the layer is layer 0, use the scan-mesh-
  direct path; otherwise fall back to current SDF→MC path. Layers
  1+ continue using SDF→MC in S1.
- Mesh-CSG ops (cap-plane trim with 1 µm bias from §Q-4 S1 +
  pyramid union) apply directly to the scan-mesh `IndexedMesh`.
- Production gates: 2 mm regen with plug_layer_0 using scan-mesh-
  direct (layers 1+2 unchanged). Compare plug_layer_0.stl to reference
  scan visually. Workshop cf-view smoke: workshop user confirms
  "plug_layer_0 looks like reference scan" or doesn't.
- Falsification: if plug_layer_0 still doesn't match reference scan
  visually (e.g., mesh-CSG ops corrupt the scan resolution somehow),
  diagnose; consider Candidate B fallback (finer MC at wall-clock
  cost).

This is the smallest-scope plunge. plug_layer_0 is the only truly
simple scan-derived surface. If this works, S2 extends to the
offset cases (which are the harder problem — Candidate A vs A2
decision lands there).

### S2 — Per-layer body + plug_layer_1+2 scan-mesh-direct (the offset case)

Extend S1's pattern to all scan-derived offset surfaces using the
S0-picked Candidate A or A2. ~250 LOC.

- New `build_layer_body_mesh(scan_mesh, cap_planes, offset_distance,
  offset_config)` function: applies the picked offset method
  (mesh-offset OR vertex-displacement) to scan mesh, trims at
  cap-planes, returns `IndexedMesh`.
- `CastSpec::export_molds_v2` per-layer body emission: use scan-mesh-
  direct for the body for all layers. plug_layer_1+2 use the same
  helper.
- The cup-wall outer (§Q-1 territory) STILL uses SDF/MC for now
  (S3 handles that).
- Production gates: 2 mm regen with per-layer bodies + plug_layer_1+2
  using scan-mesh-direct. Workshop cf-view smoke on cup-piece STLs'
  BODY CAVITY + plug_layer_2.stl.
- Falsification (Candidate A): if mesh-offset's MC at the picked `r`
  produces workshop-visible artifacts on cf-view, escalate `r`
  (finer) and re-measure wall-clock; if wall-clock breaks the
  design-iteration cadence, pivot to A2 if S0b cleared it.
- Falsification (Candidate A2): if vertex-displacement produces
  self-intersections at one or more layer thicknesses, fall back to
  Candidate A for that layer ONLY (scoped fallback).

### S3 — Cup-wall outer scan-mesh-direct

Replace §Q-1's `CupWallShellSdf` body-tracking shell with mesh-offset
of the outermost body by `wall_thickness_m`. ~150 LOC.

- The cup-wall outer becomes `mesh_offset(outermost_body_mesh,
  wall_thickness_m)`.
- `CupWallShellSdf` is REMOVED (§Q-1 fix superseded).
- The cup-piece body cavity SUBTRACTS from the cup-wall outer mesh
  to form the cup-piece (manifold3d::difference).
- Production gates: 2 mm regen. Verify §R1 inspector still passes
  (cup-piece is 1 connected component, no slivers).
- Workshop cf-view smoke: cup-wall outer should look smooth (matches
  body shape offset by 5 mm; the §Q-1 fix's body-tracking concept
  is preserved, just realized via mesh-side offset instead of SDF
  composition).

**Note**: §Q-1 fix's load-bearing test (`compose_piece_solid` doesn't
emit a cuboid lateral) is preserved in spirit — the new cup-wall
outer is also body-tracking, just at scan resolution.

### S4 — Mating-feature integration + procedure.rs cleanup

Verify all mating-feature mesh-CSG ops compose cleanly with scan-
mesh-direct input. ~80 LOC + procedure.rs adjustments.

- Registration pins, pour-gate carve, cup-pin sockets all consume
  the post-S3 cup-piece mesh; they should work with any input
  resolution.
- §Q-4 S1 plug-cap-trim with 1 µm bias is preserved (paradigm-safe;
  works on scan-mesh input).
- procedure.rs: remove obsolete §Cap-Plane Edge Chamfer prose
  (already scoped into §Q-4 S4 but moved here under the pivot's
  scope).
- F4 gate: verify printability checks pass.

### S5 — Cold-read + omnibus PR rollup

Cold-read pass on the full arc (S1-S4 commits + this recon). Per
the §Q-5 + §Q-1 + §Q-4 S1 pattern:
- Walk every empirical claim against measured data.
- Verify all paradigm-boundary checks ported correctly.
- Verify no silent regression of prior arc fixes (§Q-1 cup-wall,
  §Q-4 S1 trim bias, §Q-5 winding).
- Omnibus PR rollup or per-arc PRs per workshop user's preference.

## §SMD-5 Open questions

1. **Mesh-offset self-intersection failures**: at what offset
   distance does the workshop's iter-1 scan first fail
   self-intersection? S0 answers empirically. If 19 mm (cumulative
   + cup-wall) fails, the cup-wall could fall back to SDF/MC while
   the inner layers stay scan-mesh-direct.
2. **STL size growth**: scan-mesh-direct plug body output is ~10×
   the current MC output face count (168k scan faces vs ~15k MC
   output). Per-layer body output under A depends on mesh-offset's
   resolution; under A2 it matches scan resolution. Workshop's print
   bed + slicer comfortable with multi-MB STLs? (Bambu Studio + Orca
   handle them fine — informal confirmation needed.)
3. **Wall-clock impact**: for the plug body (zero offset), MC step
   is eliminated entirely; saves ~1-3 s per plug per §F4 timing infra.
   For per-layer body + cup-wall (mesh-offset path), mesh-offset's
   own internal MC at `OffsetConfig::resolution` adds wall-clock that
   scales cubically with `1/r`. At r = 2 mm matching current cf-cast
   cell size, wall-clock is comparable. At r = 0.5 mm (likely needed
   for workshop visual on #2 + #4 under A), wall-clock per offset
   operation could be ~64× the 2 mm cost — possibly minute-scale per
   layer. To be measured at S0a. If A2 viable, this risk evaporates.
4. **§Q-1 fix supersession**: the §Q-1 `CupWallShellSdf` was a real
   safety fix (cuboid-face × flange-interior MC ambiguity). Its
   replacement via mesh-offset preserves the body-tracking concept
   — does it preserve the safety property? S3 verifies via §R1
   inspector + new connectivity tests.
5. **Per-layer cap-plane truncation**: the cf-scan-prep cap-plane
   truncation produced ONE mesh (the open scan mesh). For per-layer
   bodies offset OUTWARD by N mm, the cap-plane shifts UP by N mm
   too (each layer's "floor" is at the original cap_z, but the layer
   body extends from cap_z upward + N mm sideways). Should each
   layer body's mesh-side cap-plane trim use the original
   `cap_centroid` or `cap_centroid + offset × cap_normal`? Geometric
   reasoning: cap-plane is the seam plane between body interior
   and the air below; for each layer, the boundary should stay at
   the original cap_z (where the device's bottom face sits when
   inserted). To be confirmed in S2.
6. **mesh-offset crate maturity at workshop scale**: is
   `offset_mesh` battle-tested at the 14-19 mm offset scale, and at
   sub-mm `OffsetConfig::resolution`? Cold-read confirmed it does
   SDF→MC internally (`offset.rs:195-218`). Empirical S0a measures
   wall-clock + face count + workshop visual smoothness. **The
   answer to this question + S0b's vertex-displacement viability
   together decide whether the pivot goes A or A2 or shrinks to
   plug-body-only.**
7. **Fallback path interaction with mating features**: if a layer
   falls back to SDF/MC (per question #1), its mating-feature
   mesh-CSG ops still need to apply. The fallback path produces an
   `IndexedMesh` at MC resolution; mesh-CSG ops compose either way.
   No special handling needed.

## §SMD-6 Cross-arc decision implications

### Supersedes §Q-4 S2/S3

§Q-4's cavity-floor slab union (S2) and mating-face defensive trim
(S3) are PAUSED. The pivot addresses the same workshop-visible
artifacts (cavity floor not flat, mating face not flat) by a
different mechanism (preserve scan resolution at the edges instead
of flattening faces). If the pivot fully closes the workshop visual
gate, §Q-4 S2/S3 are unnecessary. If the pivot only partially
closes the gate (e.g., cavity floor still has visible edge issues
after scan-mesh-direct), §Q-4 S2/S3 can be re-pursued as cleanup.

### Supersedes §Q-1 cup-wall body-tracking shell

§Q-1's `CupWallShellSdf` was a fix for the cuboid-face × flange-
interior MC ambiguity (recon-4 (P) §F-2 paradigm-boundary). Under
the pivot:
- **Under Candidate A**: cup-wall is `mesh_offset(outermost_body)` —
  still has an internal MC at `OffsetConfig::resolution`, but no
  cuboid + no flange interior intersection in the MC sampling region
  (the body-tracking shape is implicit in the offset op). The
  cuboid-face × flange-interior MC ambiguity that §Q-1 fixed cannot
  recur because there's no cuboid bounding region in the mesh-offset
  flow.
- **Under Candidate A2**: cup-wall is pure vertex-displaced — no MC
  at all in cf-cast's pipeline for this surface; ambiguity cannot
  recur by construction.

The §Q-1 fix is SUPERSEDED in implementation in either case, but its
concept (body-tracking outer boundary, not cuboid) is PRESERVED.

The §Q-1 fix's regression gate test (`compose_piece_solid` doesn't
emit a cuboid lateral) needs a parallel test under the new
architecture: the cup-wall outer mesh tracks the body's lateral
shape, with bounded distance.

### Does NOT supersede §Q-5 winding fix

§Q-5's `mesh-offset/marching_cubes::compose_face_triangles` swap of
e1/e2 (for CCW outward winding) is upstream of the MC step that the
pivot eliminates for scan-derived surfaces. But the SDF-composed
pieces (gasket molds, funnel, platform) STILL go through MC, so
§Q-5's fix stays load-bearing for them. The scan-mesh-direct path
just bypasses the MC layer where §Q-5 applies.

### Does NOT supersede §Q-4 S1 plug-cap-trim

§Q-4 S1's 1 µm offset bias on the cap-plane trim ports cleanly to
scan-mesh-direct input. The trim's purpose (flatten the cap-plane
face) and its paradigm-boundary safety (offset bias makes the trim
face CONTAINED inside body pre-trim material) are independent of
whether the body mesh came from MC or from the scan directly. S1's
regression gate test (`build_plug_cap_trim_transform_shifts_plane_one_micron_inward`)
stays load-bearing.

### Does NOT supersede flange / gasket / pour-gate / pin arcs

All other recon arcs ship features layered on top of the body mesh.
The pivot changes the body mesh's source; the layered features
consume any input resolution.

## §SMD-7 Empirical verification plan

### S0a mesh-offset characterization (no code change)

```bash
cargo test -p mesh-offset --test scan_mesh_offset_probe -- --nocapture

# Probe:
# 1. Load ~/scans/sock_over_capsule.cleaned.stl
# 2. For each (distance, resolution) ∈
#    {0.006, 0.014, 0.019} × {0.002, 0.001, 0.0005}:
#      offset_mesh(mesh, distance, OffsetConfig { resolution, .. })
# 3. For each output: measure wall-clock; face count; output bytes;
#    manifoldness via manifold3d; self-intersections via
#    mesh-repair::detect_self_intersections
# 4. Save outputs for cf-view + Orca smoke
```

Empirical question to resolve: largest workshop-acceptable `r` per
(distance, surface kind) — answers the wall-clock-vs-visual trade-off
for Candidate A.

### S0b pure vertex-displacement viability (no code change)

```bash
# Probe binary or unit test:
# 1. Load ~/scans/sock_over_capsule.cleaned.stl
# 2. Compute per-vertex outward normals (area-weighted face-normal
#    average; mesh-types or mesh-repair likely has helpers).
# 3. For each distance ∈ {0.006, 0.014, 0.019}:
#      displaced = mesh.vertices.map(|v| v + normal[v] * distance)
#      verify manifoldness + self-intersections
# 4. Save outputs for cf-view smoke
```

Empirical question: largest offset distance where pure vertex-
displacement remains self-intersection-free on workshop's iter-1
sock geometry — answers Candidate A2's feasibility.

### S1 production regen comparison (after S0 picks A vs A2)

### S1 production regen comparison

After S1 plug body scan-mesh-direct:
- Regen `~/scans/cast.toml` at 2 mm cells (still 2 mm for non-plug
  SDF-composed pieces).
- Output `~/scans/cast_iter1_post_smd_s1_plug/`.
- Compare `plug_layer_2.stl` byte size + visual:
  - Current (post-§Q-4 S1): 1,345,384 B, radial striping + edge
    stair-step.
  - Scan-mesh-direct expected: face count grows from ~15k to ~168k
    (scan's native count, ~11×); byte size scales with face count
    plus mesh-CSG overhead. Workshop visual expected to match
    reference scan within mesh-CSG's contribution at the cap-plane
    trim + pyramid union boundaries.

### S2 production regen full

After S2 per-layer bodies scan-mesh-direct:
- Output `~/scans/cast_iter1_post_smd_s2_bodies/`.
- Compare cup-piece body cavity visual against current.

### S3 production regen with cup-wall offset

After S3 cup-wall scan-mesh-direct:
- Output `~/scans/cast_iter1_post_smd_s3_cupwall/`.
- §R1 inspector gate: cup-piece STILL 1 connected component / no
  slivers / no §R1 violations.
- Workshop cf-view smoke: cup-wall outer matches body shape +
  smooth.

## §SMD-8 Workshop PR-merge gate mapping

| Blocker | Pre-pivot status | Candidate A (with mesh-offset) | Candidate A2 (with vertex displacement) |
|---|---|---|---|
| Flange all the way around | Separate §F arc (bookmark) | Separate §F arc (unchanged; flange is SDF-side, post-MC, mesh-CSG-compatible with scan-mesh-direct body) | Same — orthogonal |
| Interior mold floor flat | §Q-4 S2 candidate (planar-trim ceiling: face flat, edge jagged) | **Partial** — narrowed to mesh-offset's `OffsetConfig::resolution` ceiling (e.g., 0.5-1.0 mm cells); residual MC stair-step at that scale | **Closed** if A2 self-intersection-free; **partial fallback to A** otherwise |
| Bottom of plug flat | §Q-4 S1 shipped (face flat, edge jagged) | **Closed for plug_layer_0** (zero offset → scan mesh AS-IS). **Partial for plug_layer_1+2** (mesh-offset MC quantization). §Q-4 S1 trim still flattens the cap-plane FACE on all three plugs. | **Closed for plug_layer_0** (same as A). **Closed for plug_layer_1+2** if A2 viable; same fallback otherwise. |
| Flat mating faces | §Q-4 S3 candidate (planar-trim ceiling) | **Partial** — narrowed similarly to #2; cup-wall outer's perimeter at the seam edge inherits mesh-offset's resolution | **Closed** if A2 viable; same fallback |

PR-gate #3 (plug bottom) closes cleanly under either A or A2. #2 + #4
close fully only under A2 (vertex displacement); under A they're
narrowed to mesh-offset's MC resolution. Flange PR-blocker is
orthogonal.

The **workshop visual gate fully closes only if A2 ships viably**.
Under A alone, the gap narrows but a residual MC stair-step remains
at `OffsetConfig::resolution` (configurable, with wall-clock cost).

## §SMD-9 Risks + mitigations

### Risk 1: mesh-offset internal MC resolution × workshop visual

**Likelihood**: high (mesh-offset's internal MC is confirmed in
`offset.rs:195-218`; the workshop visual gap at 2 mm MC is exactly
what triggered this recon).
**Impact**: medium (PR-gates #2 + #4 close partially under A; A2
needed for full close).
**Mitigation**: S0a empirically measures the `r` resolution at which
mesh-offset produces workshop-acceptable smoothness for #2 + #4. If
S0a's best workshop-acceptable `r` carries an unacceptable wall-clock,
pursue Candidate A2 in S0b.

### Risk 1a: vertex-displacement self-intersections (A2)

**Likelihood**: medium (specific to sock geometry's local curvature
× large cumulative offsets).
**Impact**: high under A2 (gates S2+ on the affected layer).
**Mitigation**: S0b spike measures empirically. Scoped fallback to
Candidate A for the affected offsets only. Worst case (A2 unusable
at any production offset): pivot scope shrinks to plug body only +
falls back to A for offsets — still closes PR-gate #3; #2 + #4
narrowed partially under A.

### Risk 2: STL size growth breaks the slicer

**Likelihood**: low (modern slicers handle 10+ MB STLs fine).
**Impact**: medium (workshop print workflow disruption).
**Mitigation**: S0 + S1 produce sample STLs for workshop to load in
Orca / Bambu Studio. If slicer chokes, downstream `mesh-decimate`
crate (if exists) or `manifold3d::simplify_mesh` could decimate the
output to a workshop-acceptable size while keeping resolution at
visible features.

### Risk 3: Mesh-CSG ops on high-resolution input slow down regen

**Likelihood**: medium (manifold3d's mesh-CSG complexity is
~O(n log n) in face count; 10× faces → ~13× wall-clock at worst).
**Impact**: medium (workshop's design-iteration cadence target is
sub-5-min regen; current 3:45 + 13× = 49 min would break).
**Mitigation**: Mesh-CSG ops are NOT all O(n log n); pyramid union
is `O(small_primitive × log(host))` which barely grows. The
expensive op (cup-piece halfspace cut + cup-pin socket carves) is
O(host log host); for ~168k host face count this is still in the
second-scale, NOT minute-scale. To be measured at S1.

### Risk 4: §Q-1 fix supersession leaves a regression hole

**Likelihood**: low (the regression gate test can be parallel-ported
to the new architecture).
**Impact**: high (§Q-1's bug was a real MC topology ambiguity that
broke 2 mm regen).
**Mitigation**: S3 ports the §Q-1 regression test to the new
architecture; existing fixtures + §R1 inspector cover the safety
property.

### Risk 5: Cap-plane truncation discrepancy

**Likelihood**: low (cf-scan-prep already does the truncation; the
pivot uses its output as-is).
**Impact**: medium (could leave a thin "skirt" at the cap-plane if
mesh-side trim isn't applied consistently).
**Mitigation**: S1 verifies the cap-plane geometry matches the
processed scan; uses `manifold3d::trim_by_plane` defensively if
needed.

## §SMD-10 Prior-arc memory checklist

Per [[feedback-read-prior-arc-memory-before-architectural-decisions]]:

- [[project-cf-cast-q4-planar-trim-recon]] — §Q-4 S1 shipped is the
  predecessor; S2/S3 PAUSED pending the pivot's outcome.
- [[project-cf-cast-sdf-meshcsg-paradigm-boundary]] — framework
  unchanged; pivot adds scan-mesh-direct as a third routing for
  scan-derived bulk, doesn't introduce new boundary modes.
- [[project-cf-cast-geometry-crispness-q1-fix]] — §Q-1 fix's
  `CupWallShellSdf` is SUPERSEDED in S3; concept (body-tracking
  outer) preserved.
- [[project-cf-cast-geometry-crispness-q5-fix]] — §Q-5 winding fix
  unchanged; still load-bearing for SDF-composed pieces.
- [[project-cf-cast-geometry-crispness-s0]] — scan-mesh dimensional
  + edge-length characterization; this recon builds on the
  ~0.66 mm avg edge length empirical baseline.
- [[project-cf-cast-flange-perimeter-continuity-bookmark]] — sibling
  PR-blocker; orthogonal arc.
- [[feedback-keep-asks-bold-and-short]] — workshop UX preference;
  applied throughout this recon's framing.

## §SMD-11 Successor

S0a + S0b feasibility spikes. Workshop user picks:
- (a) **S0a only**: characterize mesh-offset's resolution-vs-quality
  trade-off; commit Candidate A in S1 with the picked `r`. Smaller
  scope; closes PR-gate #3 cleanly, #2 + #4 partially.
- (b) **S0a + S0b**: characterize both Candidate A's mesh-offset AND
  Candidate A2's pure vertex-displacement. Decide A vs A2 from
  empirical data; commit the picked path in S1. Larger up-front
  diagnostic; potentially closes all PR-gates if A2 viable.
- (c) **Skip S0, plunge into S1 plug-body-only**: ship the certain
  win (PR-gate #3) without empirical pre-validation. Defers #2 + #4
  questions to S2+.

**Recommendation: (b) S0a + S0b.** The pivot's value depends on
whether A2 is viable; deciding that empirically up-front saves a
potential mid-arc reroute. ~1 hour of diagnostic; no code commit.
Follows the workshop user's "diagnose before fixing" discipline +
the §Q-5 / §Q-1 pattern of empirical falsification at the smallest
scope before committing implementation code.

S1 code lands in the next session after S0a + S0b conclude.
