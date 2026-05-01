# mesh-shell and mesh-printability — manufacturing-aware operations

What this part covers (`mesh-printability` is depth-passed at v0.8.0; `mesh-shell` is skeleton-only pending its own arc):

This part covers two crates that share a thesis: **manufacturing constraints belong in the geometry layer, not bolted on as an afterthought**. `mesh-shell` produces printable shells (offset-based geometry that respects wall-thickness constraints); `mesh-printability` validates printability against printer-specific configurations. Both inform — and are foundational to — the casting domain's manufacturability story (see [Casting book Part 5](../../casting_architecture/src/50-constraints.md)).

## `mesh-shell` — printable shells

```rust
let shell = ShellBuilder::new(&mesh)
    .wall_thickness(2.0)        // 2mm walls
    .voxel_size(0.5)            // SDF grid resolution
    .high_quality()             // SDF-based vs normal-based
    .build()?;
```

A `ShellBuilder` consumes a closed triangle mesh and produces a watertight shell — inner surface (offset inward by `wall_thickness`), outer surface (the original), and a rim closing the open boundary. Output is a single watertight `IndexedMesh` ready for printing.

Two wall-generation methods:

| Method | Speed | Wall thickness uniformity |
|---|---|---|
| **Normal-based** (`.fast()`) | fast | varies at corners (each vertex offset along its normal; corners get geometric distortion) |
| **SDF-based** (`.high_quality()`) | slower | uniform everywhere (offset inward via `mesh-offset`'s SDF + marching-cubes path) |

Optional pre-pass `ShellBuilder::offset(distance)` applies an outward offset before shell generation — useful for creating molds where the cavity matches an inflated version of the part (accounting for shrinkage).

The depth pass covers: the shell topology guarantees (always watertight under successful generation; rim closes any open boundary), the cases where shell generation fails (non-watertight input, geometry below the wall-thickness limit), the interaction with `mesh-printability` (shells should round-trip through the printability validator after generation), and the mold-relevance — see below.

## `mesh-shell` and the casting domain

The casting book's [Part 2 Mold Geometry](../../casting_architecture/src/20-mold-geometry.md) discusses mold-cavity generation. The same `ShellBuilder` machinery is the natural foundation:

- **Mold cavity = inverse shell.** A mold is essentially "the negative space around a part, contained within an enclosure." `ShellBuilder` already produces that geometry — wall_thickness becomes mold wall, the inner surface becomes the cavity, the rim becomes the enclosure boundary.
- **Multi-shot sequencing = sequential shells.** Each shot's mold has the previous shot's cured cast as a feature of its cavity — `ShellBuilder` combined with the design SDF produces this directly when the mold cavity SDF is `DifferenceSdf(enclosure, current_cast_state)`.
- **Sim-cast::MoldBuilder** is the casting-domain wrapper that bolts onto `ShellBuilder` with mold-specific concerns (parting surfaces, pour ports, vents, demoldability validation). Same builder pattern, additional manufacturing-concerns layer on top.

The depth pass covers the design pattern transfer in detail and names which `ShellBuilder` features map onto which `MoldBuilder` features.

## `mesh-printability` — print validation

```rust
let config = PrinterConfig::fdm_default();
let result = validate_for_printing(&mesh, &config)?;

if result.is_printable() {
    println!("Pass — {}", result.summary());
} else {
    for issue in &result.issues { /* ... */ }
}
```

Validates a mesh against a `PrinterConfig` describing printer capabilities:

- **Print technology** — FDM, SLA, SLS, MJF, each with characteristic constraints. The `*_default()` constructors pre-load tier-aware thresholds; per-call tuning via `with_min_wall_thickness`, `with_max_overhang_angle`, `with_build_volume`.
- **Minimum wall thickness** — technology-dependent (FDM ~1 mm, SLA ~0.4 mm, SLS ~0.7 mm, MJF ~0.5 mm).
- **Maximum overhang angle** — FDM ~45°; SLA more conservative (~30°); SLS/MJF unrestricted (powder support).
- **Bridge span limit** — FDM/SLA only; how far the printer can bridge unsupported (SLS/MJF skip silently — powder bed acts as support).
- **Build-up direction** — parametrized via `with_build_up_direction(up)` (default `+Z`); enables non-`+Z` print orientations without re-rotating the mesh first.

Returns a `PrintValidation` enumerating `PrintIssue`s — each with severity, location, and a description — plus per-detector region vectors carrying centroids, areas / spans / volumes, and affected face indices for downstream rendering or repair.

### v0.8 detector inventory

Eight detectors run on every `validate_for_printing` call, in this order. The first three pre-date v0.8 (`check_build_volume`, `check_basic_manifold`, `check_overhangs` — corrected and refined in the v0.8 arc); the remaining five (`check_thin_walls`, `check_long_bridges`, `check_trapped_volumes`, `check_self_intersecting`, `check_small_features`) are new in v0.8.

1. `check_build_volume` — emits `ExceedsBuildVolume` Critical if any vertex falls outside `config.build_volume`.
2. `check_basic_manifold` — emits `NotWatertight` / `NonManifold` Critical via undirected-edge incidence + directed-edge winding-orientation tests. Gap F's directed-edge check catches inside-out shells that v0.7 silently accepted.
3. `check_overhangs` — emits `ExcessiveOverhang` for faces with `overhang_angle > config.max_overhang_angle_rad` (where `overhang_angle = acos(face_normal · up) - π/2`); Gap M corrected the predicate to the FDM industry convention. Gap D splits flagged faces into edge-adjacency components (each becoming an `OverhangRegion`); Gap E grades severity by tilt-past-threshold (Critical at `> threshold + 30°`, Warning at `> threshold + 15°`, Info above threshold).
4. `check_thin_walls` (Gap C) — for each face, casts an inward Möller-Trumbore ray from the centroid (offset 1 µm along `-N_F`) and reports the nearest opposite-face hit as the local wall thickness; clusters flagged faces by edge-adjacency. Severity: Critical if `thickness < min_wall_thickness / 2`, Warning otherwise.
5. `check_long_bridges` (Gap G) — flags faces with `arccos(N · -up) < 30°` (near-horizontal downward), filters faces touching the build plate, clusters by edge-adjacency, projects each cluster onto the perpendicular plane, and reports the bbox span as `LongBridgeRegion.span`. Severity: Critical at `span > max_bridge_span * 1.5`, Warning above `max_bridge_span`. Skipped silently on SLS / MJF.
6. `check_trapped_volumes` (Gap H) — voxelizes the mesh's bbox with asymmetric per-axis row jitter, scanline-parities along `+X` to mark voxels inside the mesh, exterior-flood-fills from a known-outside seed, and emits a `TrappedVolumeRegion` for each remaining unreached cavity. Memory cap: 1 GB voxel grid → `DetectorSkipped` fallback.
7. `check_self_intersecting` (Gap I) — re-uses `mesh_repair::detect_self_intersections` with `IntersectionParams::default()` (`max_reported = 100`, `epsilon = 1e-10`); emits one `SelfIntersectingRegion` per BVH-detected pair.
8. `check_small_features` (Gap J) — runs a connected-component analysis on the face graph; for each component, computes the bbox extent and emits a `SmallFeatureRegion` if extent on any axis is below a tier-aware threshold (currently `min_wall_thickness * 5`). Two severity bands: `Info` at borderline, `Warning` below threshold.

`is_printable() == false` iff any issue is `Critical`. The `Info`-severity `DetectorSkipped` issue (added in v0.8 as a typed slot for "this detector did not run because its preconditions weren't met") never blocks `is_printable`; it carries the detector name + missing precondition in its description string.

### Severity policy

| Detector | `Info` band | `Warning` band | `Critical` band |
|----------|-------------|-----------------|-------------------|
| `ExcessiveOverhang` (FDM 45°) | `> 45°, ≤ 60°` | `> 60°, ≤ 75°` | `> 75°` |
| `ThinWall` | (none) | `min/2 ≤ thickness < min` | `thickness < min/2` |
| `LongBridge` | (none) | `> max_bridge_span` | `> max_bridge_span * 1.5` |
| `TrappedVolume` | FDM only (drainage-tolerant) | (none) | SLA / SLS / MJF |
| `SelfIntersecting` | (none) | (none) | always |
| `SmallFeature` | borderline | sub-threshold | (none — per-tier `min_wall_thickness` reused) |
| `ExceedsBuildVolume` / `NotWatertight` / `NonManifold` | (none) | (none) | always |

`DetectorSkipped` is always `Info` (severity contract); `Other` is a caller-extension hook never emitted by `mesh-printability` itself.

### Why this detector ordering

The order `check_build_volume → check_basic_manifold → check_overhangs → check_thin_walls → check_long_bridges → check_trapped_volumes → check_self_intersecting → check_small_features` is not arbitrary. Two design constraints set it:

1. **Cheap topology checks first.** `check_build_volume` runs in `O(V)` (one bbox pass) and `check_basic_manifold` runs in `O(F)` (one edge-incidence pass). They are the gating checks that downstream detectors implicitly depend on — `check_thin_walls` and `check_trapped_volumes` skip with `DetectorSkipped` if the mesh isn't watertight + consistently wound, since their algorithms (inward ray-cast, voxel parity) require a closed-surface contract. Running the topology checks first means later detectors emit informative skip diagnostics rather than crashing on malformed input.
2. **Most-likely-to-block-the-print earlier.** A mesh that exceeds the build volume can't print at any orientation; flagging it first gives `summary()` callers an early-exit signal. `ExcessiveOverhang` is similarly tier-1 (every print technology has an overhang concern). Detectors with bounded false-positive rates (`SelfIntersecting` BVH false-positives on thin-aspect-ratio prismatic geometry, `SmallFeature` extent-without-volume) run last so callers iterating `validation.issues` see the high-confidence flags up-front.

The order is fixed by `validation::validate_for_printing` and is not currently caller-tunable. A global severity-descending sort on the final `validation.issues` Vec was part of the v0.8 design but did not land — issues append in detector run order; v0.9 candidate (see CHANGELOG `[Unreleased] / v0.9 candidates`).

### Validate as architectural seam

`mesh-printability` is the **boundary** between the geometric-correctness layer (`mesh-types` + `mesh-repair` + `mesh-shell`) and the manufacturing-action layer (slicers, support-generation passes, design loops that re-tune SDFs to stay within constraints). Its job is to stay opinion-free about *what to do* with an issue and very precise about *what the issue is* — every `PrintIssue` carries a typed `issue_type`, a graded `severity`, a `location` (centroid), `affected_elements` (face indices), and a typed region in the per-detector `Vec<*Region>` slot. Downstream tooling pattern-matches on `issue_type` and either (a) annotates the mesh for the user, (b) routes to a repair pass (`mesh-repair`'s `fill_holes` for `NotWatertight`, `flip_winding` for `NonManifold`-via-winding, `repair_self_intersections` for `SelfIntersecting`), or (c) feeds the geometry into an orientation search to find a more-printable rotation.

The pattern that emerges is **typed reports + typed regions + typed dispatch**: `PrintValidation` is the typed report, region structs are typed dispatch targets, and `IssueSeverity` orders priority. No string-keyed dictionaries; no `HashMap<String, serde_json::Value>` extension hooks. The contract is: if your downstream tool wants to react to an issue, it gets a typed handle to do so — and if v0.8's surface doesn't expose the handle you want (e.g., tunable `IntersectionParams`), that's a v0.9 backlog item, not a workaround channel.

This positions `mesh-printability` as the cousin of `mesh-repair`: both are L0 pure-Rust analysis crates that consume `IndexedMesh` + a config and return a typed report; both run a fixed pipeline of detectors / passes; both leave the action up to the caller. Where `mesh-repair` *modifies* mesh topology, `mesh-printability` *describes* what's wrong without modifying — leaving the repair-vs-redesign choice to the human or autonomous loop.

### Auxiliary tools

- **`find_optimal_orientation`** — searches a discrete sample set of axis-aligned + Fibonacci-sphere rotations for minimum overhang area. The 12-sample default cannot reach arbitrary axis-aligned rotations like `R_Y(-60°)`; for exact-axis recovery, hand-construct a `Matrix3` and use `apply_orientation`. v0.9 candidate (sampler enrichment).
- **`apply_orientation`** — applies a stored `OrientationResult.rotation` to a mesh.
- **`place_on_build_plate`** — translates the mesh so its bbox `min_z == 0`.
- **Per-detector region types** — `OverhangRegion`, `ThinWallRegion`, `LongBridgeRegion`, `TrappedVolumeRegion`, `SelfIntersectingRegion`, `SmallFeatureRegion`, `SupportRegion`. Each subsets the mesh into manufacturability-relevant geometry; downstream tooling (renderers, slicers, support-generation passes) consumes these directly.

### Worked example: a hollow box with one thin wall

The `printability-thin-wall` example crate (`examples/mesh/printability-thin-wall`) hand-authors a `30 × 20 × 15 mm` box with an internal cavity (`[1.5, 28.5] × [1.5, 18.5] × [1.5, 14.6]`) — side and bottom walls 1.5 mm thick, **top wall thinned to 0.4 mm**. Two vertex-disjoint shells (8 outer + 8 inner; outer CCW-from-outside, inner REVERSED so each face's outward normal points away from the surrounding solid). The mesh is watertight + consistently wound by construction; 24 triangles total.

Running `validate_for_printing` under `PrinterConfig::fdm_default()` produces:

- `thin_walls.len() == 2` — two clusters: outer top (12 tris) + inner top (12 tris). Edge-adjacency clustering keeps the shells topologically disjoint because they share no vertex.
- Both clusters: `severity = Critical` (`thickness ≈ 0.4 mm < min_wall_thickness/2 = 0.5 mm`).
- `overhangs.len() ≥ 1` — the inner top's downward-facing normal is a 90° overhang under the corrected Gap M predicate; emits `ExcessiveOverhang` Critical (the cavity-ceiling co-flag — documented v0.9 candidate for cavity-aware severity).
- `trapped_volumes.len() == 1` — the sealed cavity (`≈ 6012.9 mm³`); FDM tolerates closed cavities so severity is `Info`.
- `is_printable() == false` (any Critical blocks it).
- `summary()` — `"Found 4 issue(s): 3 critical, 0 warning, 1 info."`

The fixture is the canonical demonstration of two patterns: **closed-shell topology produces 2-cluster ThinWall outcomes** (top + bottom share no edge through side walls — generalizes to any thin slab/box), and **the cavity-ceiling co-flag** is a load-bearing v0.9 backlog item, not a bug — under the corrected predicate any sealed downward-facing surface is a 90° overhang regardless of whether the cavity is fully enclosed.

### Known limitations

The CHANGELOG's `[Unreleased] / v0.9 candidates` block carries the verbose form (each item with named re-open trigger + resolution path). The headline items, in narrative form:

- **No BVH acceleration.** `ThinWall` (brute-force O(n²) Möller-Trumbore) and `SelfIntersecting` (mesh-repair's BVH path is per-call, not amortized) dominate validation runtime above ~10k triangles. The platform ships with the perf cliff documented but not gated; v0.9 will add BVH when a real mesh exceeds the 5 s budget.
- **`IntersectionParams` not re-exported.** Callers who want to tune intersection sensitivity must depend on `mesh-repair` directly. Same family as the wider question of whether `mesh-printability` should expose every per-detector tuning surface or stay opinionated; v0.9 candidate.
- **Heuristic estimates for support / material volume.** `support_volume = overhang_area * 5.0` and the `material_volume` proxy (`bbox_volume * 0.3`) are adequate for relative comparisons across orientations; not a substitute for slicer-grade volume computation.
- **Cavity-ceiling co-flag.** Sealed cavities flag overhang under Gap M's corrected predicate (the ceiling's normal points down). Documented behavior; v0.9 candidate for cavity-aware overhang severity, requiring `OverhangRegion` to gain an `is_interior` bit cross-referenced against `TrappedVolume`-detected cavities.
- **`find_optimal_orientation` discrete sampling.** The 12-sample default cannot reach arbitrary axis-aligned rotations; the Fibonacci sphere branch produces 90°-only rotations around sphere-distributed axes. For exact-axis recovery, `apply_orientation` with a hand-constructed `Matrix3` is the workaround; v0.9 candidate (1° angle bins around primary axes + gradient-descent refinement).
- **`mesh-repair detect_self_intersections` false-positives** on thin-aspect-ratio prismatic geometry (cylinders L≥18 mm at R=5 mm; leaning-prism wings with lateral-triangle aspect ratio above ~4:1). Empirically reproduced and worked around in the orientation example (cylinder shortened to L=15 mm); v0.9 candidate (carried up to mesh-repair).
- **Build-plate filter discrimination** (Gap M.2 over-aggressiveness). The filter `face_min_along_up == mesh_min_along_up` excludes any face whose lowest VERTEX touches the plate, even when the centroid + remaining vertices represent real overhang concern (the showcase wing's 60°-tilted lateral face). v0.9 candidate (face-supported-by-plate classifier instead of point-touches-plate threshold).
- **Leaning-prism `ThinWall` co-flag.** A solid prismatic feature tilted past ~50° presents narrow inward distances when the ThinWall ray-cast exits through a tilted lateral face. The showcase wing (`5 × 5 × 15 mm`, 60° tilt) sees inward rays exit at `(WING_X_SPAN/2)/tan(60°) ≈ 0.962 mm`, below the FDM `min_wall_thickness = 1 mm` Warning band — even though the cross-section is 5 mm. A real-CAD pitfall surfaced as a load-bearing pedagogical observation; v0.9 candidate (project ray along shell-perpendicular and require both directions to confirm).
- **Global severity-descending sort of `validation.issues` not implemented.** Issues append in detector run order; the cross-detector `Critical → Warning → Info` sort that callers iterating `validation.issues` may expect did not land in v0.8. Per-detector sort policies (e.g., `long_bridges` by `(start.x, start.y, start.z)` via `f64::total_cmp`) are honored. v0.9 candidate.

### Runnable demos — eight example crates

The v0.8 fix arc shipped with eight `examples/mesh/printability-*` crates, each a museum-plaque demonstration of one detector or composition pattern. Each crate's `main()` encodes math-pass numerical anchors as Rust `assert!`s alongside its `out/*.ply` artifact emission, so `cargo run --release -p example-mesh-printability-<name>` exits 0 iff the detector behavior matches the spec. The READMEs walk through "what to look for" prose for the visuals pass.

| Crate | Detector(s) exercised | Pedagogical claim |
|-------|------------------------|-------------------|
| `printability-thin-wall` | `ThinWall` (Gap C) | Hand-authored 24-triangle hollow box; 2-cluster topology + cavity-ceiling co-flag |
| `printability-long-bridge` | `LongBridge` (Gap G) | 44-triangle H-shape boolean-union; middle bridge fires Critical, cantilevers below threshold |
| `printability-trapped-volume` | `TrappedVolume` (Gap H) | Solid cube + sealed sphere cavity (REVERSED winding); per-tech severity matrix (FDM Info, SLA/SLS/MJF Critical) |
| `printability-self-intersecting` | `SelfIntersecting` (Gap I) | Two interpenetrating cylinders; mesh-repair re-use through the validate pipeline |
| `printability-small-feature` | `SmallFeature` (Gap J) | Cube + 0.2 mm hex-prism burr on the build plate; non-Critical Warning, `is_printable() == true` |
| `printability-orientation` | `build_up_direction` (Gap L) | Leaning cylinder; 3-run invariant (rotating mesh ≡ rotating up-vector) |
| `printability-technology-sweep` | (all detectors) | Single hollow box validated under FDM/SLA/SLS/MJF; cross-tech severity divergence |
| `printability-showcase` | (all eight detectors) | Five vertex-disjoint shells (528 v / 1032 f); capstone surfacing five v0.8 spec deviations + a sixth observation (§4.4 sort partial implementation) as load-bearing pedagogy |

Each example ships ASCII PLY artifacts for `f3d` / MeshLab / ParaView review. Where `f3d`'s default rendering hides load-bearing geometry (REVERSED inner shells, downward-facing slab bottoms, `+Y`-up vs `+Z`-up world), the README's top-of-file callout names the workaround.

## Pattern transfer to casting

The casting domain's [Part 5 Manufacturing Constraints](../../casting_architecture/src/50-constraints.md) mirrors `mesh-printability`'s shape:

```rust
// Casting analogue (proposed; not yet implemented)
let config = CastingProcessConfig::ecoflex_open_pour();
let result = validate_for_casting(&mesh, &cast_sequence, &config)?;
// Returns CastingValidation with CastingIssue list
```

Same structural pattern: `Config` → validation → typed `Issue` list + per-issue typed regions. Different concerns (draft angles, demoldability, multi-shot adhesion vs. overhang, wall thickness, bridge spans) but identical API shape. The reuse story has three layers:

1. **Pure geometric primitives** — manifold checks, edge-incidence parity, connected-component analysis, bbox extent — live in `mesh-types` + `mesh-repair` and are consumed by both `mesh-printability` and the future `sim-cast::ManufacturabilityCheck`. These don't get duplicated; they're library calls from both validators.
2. **Detector pattern** — the precondition-skip-with-`DetectorSkipped` policy, the per-detector typed-region output, the per-tier severity bands. The casting validator should adopt the same pattern: `CastingIssueType::DetectorSkipped` for "this analysis didn't run because preconditions weren't met" + `Info` / `Warning` / `Critical` bands graded against material-and-process tunables.
3. **Domain-specific algorithms** — draft-angle analysis (per-face dot product against parting direction with material-dependent threshold), demoldability classification (every cavity must have a free path to one half of the mold), multi-shot adhesion (each shot's bond surface must overlap the previous shot's cured surface). These have no analogue in `mesh-printability` and need fresh detectors.

The takeaway: `mesh-printability` defines the API shape (`Config → validation → typed report`) and the precondition-skip protocol (`DetectorSkipped`); the casting domain inherits the architecture decisions but writes its own detectors against the same `IndexedMesh` substrate. Where the platform invests once (typed-report contract) it pays back twice (manufacturing validation in both print and cast domains).

## What's NOT in these crates

- **Mold-fabrication-specific machinery.** Parting surfaces, pour-port placement, multi-shot sequencing — these are casting-domain concerns. `mesh-shell` produces the geometric primitive (a shell); `sim-cast` will apply mold-specific operations on top.
- **Slicing for layered manufacturing.** Cross-section extraction for FDM/SLA layer-by-layer slicing; out of scope. (Adjacent: `mesh-measure::cross_section` produces single slices, but the slicing-pipeline machinery isn't in mesh-printability.)
- **Print-time prediction / cost estimation.** Estimating how long a part takes to print, or material cost — out of scope; downstream tooling concern.
- **Support structure generation.** `mesh-printability` *detects* regions needing support but doesn't generate the support geometry. Support generation is a separate research-and-engineering problem (and arguably a `mesh-support` crate, if needed).
