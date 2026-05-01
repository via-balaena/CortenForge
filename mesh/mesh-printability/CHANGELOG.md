# Changelog

All notable changes to mesh-printability will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **`PrinterConfig::build_up_direction` parametrization (Gap L).** v0.7's
  `check_overhangs` and `evaluate_orientation` hardcoded
  `Vector3::new(0.0, 0.0, 1.0)` as the build-up direction, blocking
  callers using non-`+Z` build orientations (5-axis printers, oriented
  castings, mesh-frame rotated coordinate systems) from validating
  without first rotating their meshes. v0.8 adds a public
  `build_up_direction: Vector3<f64>` field to `PrinterConfig` (defaults
  to `(0, 0, 1)` in all four `*_default()` constructors, preserving
  v0.7 behaviour) and a `with_build_up_direction(up: Vector3<f64>)`
  builder that normalizes the input internally and `debug_assert!`s on
  zero-vector input. Pure addition; existing callers see no behavioural
  change. The four `*_default()` constructors stay `pub const fn`
  (`Vector3::new` is const-callable in nalgebra ≥ 0.34); the new
  builder is non-const because it calls `.normalize()`. Release-mode
  zero-vector input produces `NaN` components which propagate through
  the downstream `acos(face_normal · build_up_direction)`
  computations — explicit caller responsibility per the builder's
  `# Panics` doc section. **Future-detector commitment**: detectors
  authored later in v0.8 (`TrappedVolume`, `LongBridge`, and any
  successors) consume `config.build_up_direction` from day one — never
  a hardcoded `(0, 0, 1)`.
- **`PrintIssueType::DetectorSkipped` variant added (§5.8).**
  Detectors with precondition skip behaviour (`ThinWall` §6.1,
  `TrappedVolume` §6.2, and any successors per §4.1) need a typed
  issue to push when they early-return because their preconditions
  (e.g. watertight mesh, consistent winding) are not met. v0.7 had no
  typed slot for "this detector did not run because its preconditions
  weren't met"; the existing `Other` variant is a caller-extension
  hook that mesh-printability itself never emits, so re-using `Other`
  would muddle that contract. The new variant gives detectors a
  single, type-safe way to signal a precondition skip with severity
  contract `IssueSeverity::Info` (`as_str() == "Detector Skipped"`);
  the issue's description string names the detector + the missing
  precondition. **SEMVER note**: pure addition at the variant level,
  but `PrintIssueType` is not `#[non_exhaustive]`, so downstream
  callers performing exhaustive `match` on the enum without a
  wildcard arm will need to add a `DetectorSkipped => …` arm. All
  workspace consumers (`mesh-printability` internal sites,
  `mesh`, `mesh-shell`, examples) compare via `==` equality and are
  unaffected. **Future-detector commitment**: detectors authored
  later in v0.8 (`ThinWall` §6.1, `TrappedVolume` §6.2, and any
  successors) emit `DetectorSkipped` on precondition failure rather
  than silently returning.
- **`check_thin_walls` detector + populated `ThinWallRegion` (Gap C, §6.1).**
  v0.7 exposed `PrintIssueType::ThinWall` and `PrintValidation.thin_walls`
  but `validate_for_printing` never populated the field — a critical
  silent-failure for FDM/SLA print validation, since under-min-wall
  geometry is the dominant cause of print failure for sub-1 mm features.
  v0.8 wires a private `check_thin_walls(mesh, config, issues) ->
  Vec<ThinWallRegion>` per §6.1: brute-force O(n²) inward ray-cast from
  each face's centroid (offset 1 µm by `EPS_RAY_OFFSET` to avoid self-hit)
  via a private `moller_trumbore` helper (~30 LOC textbook implementation;
  workspace had no prior ray-tri primitive — `mesh-repair::intersect`
  exposes triangle-triangle, not ray-triangle). Preconditions: watertight
  + consistent winding; on failure, emits the §5.8 `DetectorSkipped`
  variant with description `"ThinWall detection requires watertight mesh
  with consistent winding (skipped)"` and returns empty regions
  (first consumer of the §5.8 variant in production). Per-cluster
  severity via `classify_thin_wall_severity`: `Critical` if `thickness <
  min_wall_thickness / 2`, `Warning` otherwise. Clusters emerge in
  min-face-idx component order via `partition_flagged_into_components`
  (the §5.3 helper, generalized in this commit from `HashMap<u32,
  FlaggedFaceMeta>` to `HashMap<u32, T>` so the same DFS serves both
  detectors); faces within each cluster are sorted ascending. Reported
  thickness is `min_dist + EPS_RAY_OFFSET` (the geometric wall
  thickness, accounting for the ray's 1 µm starting offset). **§7.1
  fixture topology audit (HIGH-tier per §8.4)**: hand-traced the 24-
  triangle hollow-box construction; confirmed watertight + consistent-
  winding + 2-cluster theoretical outcome (outer-top + inner-top
  topologically disjoint via edge-adjacency, since the construction
  uses two disjoint cube-component vertex sets). The 2-cluster topology
  applies generically to closed-shell thin-section fixtures (slab top
  + slab bottom share no edge); unit tests assert it explicitly. **v0.9
  followups**: BVH acceleration for the >10k-tri perf cliff (documented,
  not gated); shell-based thickness via mesh-offset SDF for highly
  curved geometry; anisotropic thresholds per build direction. **SEMVER
  note**: pure addition (new private fns + `partition_flagged_into_components`
  signature generalization that's invisible to existing callers); v0.7's
  unpopulated `validation.thin_walls` was always empty so existing
  callers see no behavioural break beyond now having actual data.
- **`tests-release` CI job covers mesh-printability (§10.4.2).**
  `stress_c_5k_tri_perf_budget` (5000-triangle thin-walled UV-sphere
  shell pair, asserts O(n²) ThinWall detector completes in <2 s release
  mode) is the first release-only stress fixture for mesh-printability;
  subsequent §6 detectors will land more (e.g., `stress_h_voxel_grid_perf_cliff`
  in commit #14, `stress_i_truncation_at_100` in commit #16). Without
  `cargo test --release` in CI, perf regressions on the >5k-tri budget
  go unnoticed. Single-line append to `quality-gate.yml::tests-release`
  step (adds `-p mesh-printability` to the existing
  `cargo test --release` command); cost ~3-5 min cold-cache, ~1-2 min
  warm-cache (Swatinem/rust-cache active); runs in parallel with
  `tests-debug` so no critical-path impact.
- **`examples/mesh/printability-thin-wall` visual demo (Gap C, §7.1).**
  First production consumer of the §6.1 ThinWall detector — a
  hand-authored 24-triangle hollow box (outer 30×20×15 mm; inner
  cavity x ∈ [1.5, 28.5], y ∈ [1.5, 18.5], z ∈ [1.5, 14.6] with the
  top wall thinned to 0.4 mm; side and bottom walls 1.5 mm). The two
  vertex-disjoint shells (8 outer corners + 8 inner corners; outer
  CCW-from-outside, inner REVERSED so each face's normal points away
  from the surrounding solid) are watertight + consistently wound by
  construction, partitioning under edge-adjacency into the predicted
  two clusters (outer top + inner top) per §7.1. `main()` asserts
  eight of the §7.1 numerical anchors: cluster count = 2, both
  centroids `(15, 10, 15)` + `(15, 10, 14.6)` within 1e-9, both areas
  600 + 459 mm² within 1e-9, both `thickness ≈ 0.4 mm` within 1e-5,
  two Critical ThinWall issues, `!is_printable()`, ≥1 Overhang region
  for the cavity-ceiling co-flag (overhang_angle = 90°, Critical
  under FDM 45° + 30° = 75° band), and zero `DetectorSkipped` issues
  (preconditions hold). Produces `out/mesh.ply` (16v, 24f, ASCII)
  + `out/issues.ply` (vertex-only point-cloud of region centroids,
  ASCII) for the visuals pass. **Three TrappedVolume assertions from
  §7.1 (sealed-cavity volume ≈ 6012.9 mm³, centroid `(15, 10, 8.05)`,
  count == 1) are deferred** to row #14b — a tiny follow-up commit
  immediately after row #14 (when the §6.3 TrappedVolume detector
  first ships); the current `PrintValidation` struct has no
  `trapped_volumes` field. The
  README's f3d-winding callout sits **near the top of the file** per
  `feedback_f3d_winding_callout` — the inner cavity's reversed
  winding is the load-bearing topology the detector relies on, not a
  bug, and `f3d`'s default two-sided lighting hides it; MeshLab and
  ParaView render both shells with their distinct orientations
  visible. Crate name `example-mesh-printability-thin-wall` per §7.0
  + §12.3's example-commit naming convention. Pure addition; first
  ⏸︎ pause-for-visuals commit per §12.1 row #11.
- **`check_long_bridges` detector + populated `LongBridgeRegion` (Gap G, §6.2).**
  v0.7 exposed `PrintIssueType::LongBridge` and `PrinterConfig::max_bridge_span`
  but `validate_for_printing` never populated a bridge field on
  `PrintValidation` — a silent miss for FDM/SLA bridge-span constraints.
  v0.8 wires a private `check_long_bridges(mesh, config, validation)`
  per §6.2: per-face `arccos(N · -up) < 30°` classification (the
  "near-horizontal downward" predicate), build-plate filter
  (mesh-min-relative, same pattern as `check_overhangs`'s M.2),
  edge-adjacency clustering (reuses the §5.3 generic
  `partition_flagged_into_components<T>`), and per-cluster bbox-extent
  computation in the plane perpendicular to `up` via a new
  `perpendicular_plane_basis` helper (orthonormal `(e1, e2)` derived
  by Gram-Schmidt from a non-collinear reference vector). New
  `LongBridgeRegion` type added to `regions::*` (re-exported from the
  crate root) carries `start`, `end`, `span`, `edges` (boundary edges
  shared with non-candidate faces, `(min, max)`-normalized), and
  `faces`. New `PrintValidation.long_bridges: Vec<LongBridgeRegion>`
  field populated alongside `thin_walls` and `overhangs`. Per-cluster
  severity via a new `classify_long_bridge_severity`: `Critical` if
  `span > max_bridge_span * 1.5`, `Warning` otherwise — **no Info
  band** (the bridge decision is binary at the printer-config level
  per §6.2 line 1014). SLS/MJF skip **silently** (no
  `DetectorSkipped` issue, distinct from `ThinWall`'s skip semantics)
  per §6.2 line 996, since bridges are not applicable to powder-bed
  processes. **v0.8 documented limitations** (locked by §9.2.4 stress
  fixtures `stress_g_cantilever_currently_flagged` +
  `stress_g_diagonal_bridge_underflagged`): cantilever-as-bridge (no
  support-end analysis; v0.9 followup) and diagonal-underflag
  (axis-aligned-in-perp-plane bbox is conservative for diagonals;
  v0.9 OBB followup). Cluster overlapping with overhang regions
  produces independent flags from both detectors (documented
  behaviour, not a duplicate-flag bug). **§4.4 ordering**:
  `long_bridges` regions sort by `(start.x, start.y, start.z)` via
  `f64::total_cmp` (deterministic across `HashMap` iteration
  permutations and FP signed-zero / NaN edge cases); the sort is
  scoped per-detector to a `regions_before` slice so a future hoist
  to `validate_for_printing` can move it cleanly. **§7.2 visual demo
  deferred** to row #13 (`feat(examples):
  mesh-printability-long-bridge visual demo (Gap G)`). **SEMVER
  note**: pure addition of a new field on `PrintValidation` and a
  new public `LongBridgeRegion` type;
  `PrintIssueType::LongBridge` was already declared in v0.7 so
  exhaustive `match` callers are unaffected.
- **`examples/mesh/printability-long-bridge` visual demo (Gap G, §7.2).**
  Second production consumer of the §6.2 `LongBridge` detector (after
  the 13 unit tests + 5 §9.2.4 stress fixtures from the detector
  commit). Hand-authored 24-vertex / 44-triangle H-shape — two
  5×5×18 mm pillars (`x ∈ [0, 5]` and `[25, 30]`, `y ∈ [0, 5]`) joined
  by a 35×5×2 mm horizontal slab (`x ∈ [-2.5, 32.5]`, `z ∈ [18, 20]`)
  — built as a single watertight, consistently-wound boolean-union
  solid. Pillar tops at `z = 18` are interior to the union; the slab
  bottom is a polygon-with-2-rectangular-holes triangulated as three
  edge-disjoint regions: a 20×5 middle bridge (between pillars) plus
  two 2.5×5 cantilevers (outside the pillars). Slab front + back are
  8-gons (bottom edges broken at the 4 pillar-attachment x-values),
  triangulated as 6-triangle fans from the top-corner vertex; left +
  right are 2 each; bottom regions are 2 each × 3 = 6. Hand-traced
  manifold proof (every undirected edge appears in exactly two faces;
  every directed edge is matched by its reverse) confirms the §6.2
  preconditions. `main()` asserts the §7.2 numerical anchors under
  `PrinterConfig::fdm_default()`: `long_bridges.len() == 1` (only the
  middle bridge clears `max_bridge_span = 10`; cantilevers at span 5
  silently dropped by `emit_long_bridge_component`'s
  `span <= max_bridge_span` early-return), middle bridge `span ≈ 20`
  and `start.x / end.x ≈ 5 / 25` within 1e-6, 1 Critical
  `LongBridge` issue (20 > 10 × 1.5 = 15), `overhangs.len() == 3` with
  centroids `(-1.25, 2.5, 18)` / `(15, 2.5, 18)` / `(31.25, 2.5, 18)`
  within 1e-9, 3 Critical `ExcessiveOverhang` issues (90° tilt > 75°),
  middle-bridge LongBridge midpoint coincides with middle-bridge
  OverhangRegion centroid at `(15, 2.5, 18)` (the load-bearing
  multi-detector co-flag), and `!is_printable()`. The `TrappedVolume`
  spec assertion #7 (`trapped_volumes.len() == 0`) is implemented as
  a `PrintIssueType::TrappedVolume` issue-count filter rather than a
  literal field check — semantically equivalent and stays correct
  after row #14 lands the `trapped_volumes` field; no row #14b
  backfill is needed for this example. SLS tech-skip cross-check
  re-validates the same fixture under `PrinterConfig::sls_default()`
  and asserts `long_bridges.len() == 0` AND `overhangs.len() == 0`
  AND zero `DetectorSkipped` issues — SLS has
  `requires_supports() == false`, so both `check_overhangs`
  (`validation.rs:270`) and `check_long_bridges`
  (`validation.rs:1303`) early-return before classifying any face;
  the `DetectorSkipped` variant is reserved for precondition skips
  (per §6.2 line 996), not technology-policy skips. Produces
  `out/mesh.ply` (24v, 44f, ASCII) +
  `out/issues.ply` (4 centroid points, 2 coincident at `(15, 2.5, 18)`,
  ASCII) for the visuals pass. The README's f3d back-face-culling
  callout sits **near the top of the file** per
  `feedback_f3d_winding_callout` — the slab's bottom is a genuine
  outward-facing-with-normal-`-z` surface, exactly as the printability
  detector sees it; viewers that cull back-faces by default will hide
  the load-bearing geometry. Crate name
  `example-mesh-printability-long-bridge` per §7.0 + §12.3's
  example-commit naming convention. Pure addition; second ⏸︎
  pause-for-visuals commit per §12.1 row #13.
- **`check_trapped_volumes` detector + populated `TrappedVolumeRegion`
  (Gap H, §6.3).** v0.7 exposed `PrintIssueType::TrappedVolume` but
  `validate_for_printing` never populated a `trapped_volumes` field on
  `PrintValidation` — a silent miss for SLA / SLS / MJF print
  validation, since trapped uncured / unsintered material in a sealed
  cavity is one of the dominant hard-failure modes for those processes.
  v0.8 wires a private `check_trapped_volumes(mesh, config, validation)`
  per §6.3: voxel-based exterior flood-fill. Algorithm: `voxel_size =
  min(min_feature_size, layer_height) / 2`; mesh AABB padded by 2 voxel
  widths; integer dims via `ceil(extent / voxel_size)`. Inside-mark
  every voxel via per-row +X scanline + Möller-Trumbore parity; flood-
  fill exterior from grid corner `(0, 0, 0)` over `VOXEL_UNKNOWN`
  voxels (6-connected BFS via `VecDeque`); remaining `VOXEL_UNKNOWN`
  voxels become `VOXEL_TRAPPED`; connected-component label trapped
  voxels via 6-connected BFS; per component emit one
  `TrappedVolumeRegion { center, volume, bounding_box, voxel_count }`
  + one `PrintIssue` keyed on technology-aware severity classifier.
  New public `TrappedVolumeRegion` type added to `regions::*`
  (re-exported from the crate root); new
  `PrintValidation.trapped_volumes: Vec<TrappedVolumeRegion>` field
  populated alongside `thin_walls`, `overhangs`, `long_bridges`.
  Per-cluster severity via a new `classify_trapped_volume_severity`:
  `Info` if `volume < min_feature_size³` (below printer resolution;
  not actionable), else `Critical` for SLA / SLS / MJF, `Info` for
  FDM / Other (sealed cavities print fine on extrusion). Precondition:
  `is_watertight(mesh)` (open-edge-count only — a new helper distinct
  from the §6.1 `is_watertight_and_consistent_winding`, since
  TrappedVolume tolerates inconsistent winding per §9.1 row 11). On
  failure: emit `DetectorSkipped` Info + return without populating
  `trapped_volumes`. **§6.3 step 4.5 amendment (§9.2.5-surfaced):**
  pre-allocation memory check rejects voxel grids whose total byte
  count would exceed 1 GB (1 byte per voxel; `u8` voxel state) —
  emits `DetectorSkipped` Info with description naming the grid
  dimensions + the voxel size, returns BEFORE allocation. Mitigates
  §8.4 grid-memory blowup risk: a 250 × 200 × 220 mm full FDM build
  volume at voxel 0.1 mm would request 11 GB; the cap catches this
  cleanly. Verified pre-flight by direct arithmetic for FDM defaults
  (20 mm part = 8.5 MB OK; 200 mm cube = 8 GB caught; full build
  volume = 11 GB caught). **§6.3 spec refinement (v0.8 implementation
  vs spec):** the §6.3 line 1096 text "a single uniform jitter
  suffices" produces a parity-flip-by-2-cancel bug on axis-aligned
  cube fixtures whose face diagonals run along `y = z`; v0.8
  implementation uses **asymmetric per-axis jitter** (`ROW_JITTER_Y =
  1.0e-5`, `ROW_JITTER_Z = 1.7e-5`) so the ray's `(y, z)` coords never
  lie on a triangle's symmetry line. Both magnitudes still sit an
  order above `EPS_RAY_OFFSET` (1e-6 mm). v0.9 spec edit will document
  the asymmetric requirement. **§4.4 ordering**: `trapped_volumes`
  regions sort by `(center.x, center.y, center.z)` via
  `f64::total_cmp` (mirrors `long_bridges`' pattern at
  `validation.rs:1335`); sort scoped per-detector to a `regions_before`
  slice so a future hoist to `validate_for_printing` can move it
  cleanly. **§7.3 visual demo deferred** to row #15. **§7.1 backfill
  (3 deferred TrappedVolume assertions: `trapped_volumes.len() == 1`,
  volume ≈ 6012.9 mm³, centroid `(15, 10, 8.05)`)** lands at row #14b
  (a tiny follow-up commit immediately after row #14, per §12.1).
  **SEMVER note**: pure addition of a new field on `PrintValidation`
  and a new public `TrappedVolumeRegion` type;
  `PrintIssueType::TrappedVolume` was already declared in v0.7 so
  exhaustive `match` callers are unaffected.
- **§9.2.5 stress fixtures + §9.2.1 deferred fixtures**
  (`tests/stress_inputs.rs`): 9 new integration tests landing alongside
  the §6.3 detector. §9.2.1 deferred from row #2:
  `stress_existing_single_triangle_open_mesh` +
  `stress_existing_two_faces_vertex_only_shared` — both assert that
  the new TrappedVolume detector skips with `DetectorSkipped` on
  non-watertight inputs, alongside ThinWall's matching skip. §9.2.5:
  `stress_h_solid_cube_no_cavity`, `stress_h_open_mesh_skipped`,
  `stress_h_sphere_inside_cube_volume_within_10pct` (cube cavity sized
  to give analytical sphere volume 523.6 mm³; 10 % tolerance per §9.6
  + §8.4 cross-platform FP-drift mitigation),
  `stress_h_two_disjoint_cavities`,
  `stress_h_disconnected_dual_cavity` (verifies grid-corner-seeded
  flood-fill reaches both cubes' exteriors per §9.1 row 13),
  `stress_h_voxel_grid_perf_cliff` (`#[cfg_attr(debug_assertions,
  ignore)]`; release-only; 100 × 100 × 30 mm part with cavity, voxel
  0.4 mm; <30 s release-mode runtime budget),
  `stress_h_voxel_grid_oom_safety` (200 mm cube at FDM voxel 0.1 mm =
  8 GB grid → `DetectorSkipped` emitted before allocation; runs in
  <100 ms, locks the §6.3 step 4.5 amendment). **Deferred:**
  `stress_h_subvoxel_opening_not_flagged` (§9.2.5 line 2382) is
  explicitly deferred — the faithful fixture requires a watertight
  cube + watertight inner cavity + sub-voxel tube fusing them via
  hole-triangulation (≈ 50 LOC of cube-with-hole authoring + tube
  topology), and the v0.8 documented limitation per §6.3 line 1118 is
  already covered indirectly by `test_trapped_volume_info_below_min_feature`
  in `validation.rs::tests` (resolution-threshold path). Geometry-
  authoring effort is better paired with v0.9's drainage-path
  simulation, when the behavior becomes actionable rather than
  purely diagnostic.
- **`cross-os` CI matrix covers mesh-printability (§10.4.1).** Single-
  line append to `quality-gate.yml::cross-os::Run tests` step (adds
  `-p mesh-printability` to the existing `cargo test` command). Cost:
  ~2 min per OS (mesh-printability compiles fast in `cargo test` debug
  mode); fits comfortably in the existing job runtime. Mitigates §8.4
  Gap H FP-drift risk: macOS and Windows runners catch platform-
  divergent hard-fails in the voxel inside-test (scanline ray-tri
  parity + flood-fill) that single-OS coverage would miss. Per §9.4's
  Gap H FP-drift mitigation table, this CI extension is load-bearing
  for closing the §8.4 row 3 risk; the
  `stress_h_sphere_inside_cube_volume_within_10pct` fixture's 10 %
  volume tolerance + the cross-os matrix together absorb the
  documented FP-drift surface.
- **Backfilled `TrappedVolume` assertions in `printability-thin-wall`
  (deferred from row #11).** §7.1 of the v0.8 fix arc spec specified
  three TrappedVolume anchors for the hollow-box example —
  `trapped_volumes.len() == 1`, voxel-discretized cavity volume
  ≈ 6012.9 mm³ (= 27 × 17 × 13.1, the cavity interior), centroid
  `(15, 10, 8.05)` within `voxel_size` (= 0.1 mm at FDM defaults) —
  but row #11 deferred them because the §6.3 `TrappedVolume` detector
  did not yet ship. Row #14 landed the detector; row #14b backfills
  the three assertions in `examples/mesh/printability-thin-wall/src/main.rs`,
  removes the row #11 deferral comment block from `verify`'s
  doc-comment + the module doc-comment, and updates the bullet-list
  numerical anchors. Three new constants
  (`ANALYTICAL_CAVITY_VOLUME = 6012.9`, `TRAPPED_VOLUME_REL_TOL = 0.10`,
  `TRAPPED_CENTROID_TOL = 0.1`) capture the spec's tolerance bands.
  `save_issue_centroids` + `issue_centroid_count` extend to include
  `trapped_volumes` (the cavity centroid lands as the 5th point in
  `out/issues.ply`); `LongBridge` is intentionally omitted from the
  PLY-writer per its differing centroid semantic (cluster-bbox
  midpoint vs per-region issue location). For this fixture the
  cavity boundaries align with voxel edges so the discretization is
  bit-exact: 270 × 170 × 131 = 6 012 900 voxels × 0.001 mm³ =
  6012.90 mm³, voxel-center mean = `(15, 10, 8.05)` exactly. The
  spec's 10 %-volume / 0.1 mm-centroid tolerance bands give
  cross-platform ULP headroom per §9.6 + §8.4 row 3.
- **`examples/mesh/printability-trapped-volume` visual demo (Gap H, §7.3).**
  First production consumer of the §6.3 `TrappedVolume` detector beyond
  the row #14 unit tests + stress fixtures and the row #14b
  `printability-thin-wall` backfill. The fixture is a hand-authored
  20 × 20 × 20 mm solid cube with a sealed sphere cavity (radius 5 mm,
  centred at `(10, 10, 10)`): 12 outer cube triangles plus a UV-
  tessellated sphere (32 segments × 16 stacks = 960 triangles) with
  REVERSED winding so sphere normals point INTO the cavity. Both shells
  are individually watertight + consistently wound and they share no
  vertices. **Multi-technology severity sweep**: validates under all
  four `PrinterConfig::*_default` technologies and asserts the §7.3
  table — TrappedVolume `Info` for FDM (sealed cavities print fine on
  extrusion) vs `Critical` for SLA / SLS / MJF (uncured-resin /
  unsintered-powder trap); cavity-ceiling overhang `Critical` for FDM
  + SLA, silent-skip for SLS / MJF (`requires_supports() == false`).
  Per-tech `voxel_size = min(min_feature_size, layer_height) / 2` is
  recomputed in the centroid-tolerance assertion (`0.1 mm` FDM,
  `0.025 mm` SLA, `0.05 mm` SLS, `0.04 mm` MJF). Volume tolerance is
  the §9.6 `± 10 %` band against analytical `(4/3) π r³ ≈ 523.6 mm³`,
  absorbing UV-tessellation chord shrinkage (~ 1.5 %), voxel
  discretization, and cross-platform FP drift. Each tech fails
  `is_printable()` for a different reason — the pedagogical point of
  the example. Saves `out/mesh.ply` (490 v, 972 f, ASCII) + `out/issues.ply`
  (FDM iteration centroids, ASCII vertex-only) for the visuals pass;
  README front-matter callout per `feedback_f3d_winding_callout` flags
  the inner sphere's REVERSED winding as f3d-hidden by design (use
  MeshLab + slice plane or ParaView + clip filter to see the cavity).
  Crate name `example-mesh-printability-trapped-volume` per §7.0 +
  §12.3's example-commit acceptance gates. **§7.3 spec deviation**:
  `out/voxels.ply` (point-cloud of trapped voxel centres) is deferred
  to v0.9 — `TrappedVolumeRegion` (regions.rs:153) exposes only
  `center / volume / bounding_box / voxel_count` and not the individual
  voxel centres; surfacing them requires a public-API extension that
  is out of scope for the row #15 example-only commit. The cavity
  centroid + bounding box already in `issues.ply` cover the
  pedagogical need; the per-voxel point-cloud is a v0.9 visualization
  enhancement once a clear use case drives it.
- **`SelfIntersecting` detector via mesh-repair re-use (Gap I, §6.4).**
  Re-uses `mesh_repair::detect_self_intersections` with
  `IntersectionParams::default()` (`max_reported = 100`, `epsilon = 1e-10`,
  `skip_adjacent = true`). New `SelfIntersectingRegion` typed-region
  (face_a, face_b, approximate_location) and new
  `PrintValidation.self_intersecting` field populated by
  `check_self_intersecting`. Severity is always Critical (slicer
  behavior on self-intersection is undefined). Per-region
  `(face_a, face_b)` is canonicalized to `face_a < face_b` and §4.4-
  sorted ascending for cross-run determinism. One summary `PrintIssue`
  per call: description format
  `"{N} self-intersecting triangle pair(s)"` + `" (search truncated;
  total may be higher)"` suffix when mesh-repair's
  `SelfIntersectionResult.truncated` is true. mesh-repair workspace
  dep added; HIGH-tier pre-flight per §8.4 row 5 verified Layer
  Integrity Criterion 6 stays A (release graph dep count comfortably
  under L0's 80 cap) and `cargo build --target wasm32-unknown-unknown
  -p mesh-printability` stays clean. Eight unit tests in
  `validation.rs::tests` cover §6.4's 8 adversarial cases (overlapping
  pair, clean cube, edge-adjacent skip, truncation-suffix structure,
  approximate-location midpoint, Critical-blocks-is_printable, sort
  stability, canonical face_a < face_b). Five §9.2.6 stress fixtures
  in `tests/stress_inputs.rs` (clean cube, release-only 21-triangle
  fan that hits the 100-cap with truncation, canonical ordering,
  vertex-only contact, near-coplanar intersection at ~1 mrad).
  **§6.4 spec deviations surfaced**: (a) mesh-repair's `skip_adjacent`
  is *vertex*-shared (per `build_face_adjacency` at
  `mesh-repair/src/intersect.rs:260`), not edge-shared as the spec
  text implied — broader than necessary but conservatively correct
  (vertex-only contact is also "not flagged" per §6.4 line 1189);
  (b) the §9.2.6 line 2408 fixture
  `stress_i_vertex_only_contact_not_flagged` was reframed to use a
  shared vertex *index* rather than coord-only-shared corners, because
  mesh-repair's SAT separating-axis test is loose at coord-only vertex
  contact (`max1 + epsilon < min2` fails for touching projections —
  `mesh-repair/src/intersect.rs:379`) and produces 36 false-positive
  pairs on coord-shared cubes; the v0.8 mechanism for "vertex-only
  contact = not flagged" is the adjacency skip path, faithfully
  exercised by the shared-INDEX fixture, and the SAT looseness on
  duplicate-coord meshes is filed as a v0.9 mesh-repair followup.
- **`examples/mesh/printability-self-intersecting` visual demo (Gap I, §7.4).**
  First production consumer of the §6.4 `SelfIntersecting` detector
  beyond the row #16 unit + stress fixtures. The fixture is two hand-
  authored cylinders concatenated **without** boolean union, welding,
  or vertex sharing: cylinder A axis along `+X` (length 30 mm × radius
  5 mm), cylinder B axis along `+Y` (same dimensions), each with 16
  azimuthal segments → 34 vertices + 64 triangles per cylinder. The
  combined assembly is 68 verts + 128 tris; post-`place_on_build_plate`
  the bbox is `[-15, +15] × [-15, +15] × [0, 10]` and the original
  origin maps to `(0, 0, 5)`. Each cylinder is independently watertight
  and consistently wound; the union is NOT manifold (lateral surfaces
  interpenetrate), but `SelfIntersecting` runs unconditionally — no
  watertightness precondition for §6.4. **Anchor outcomes**: the four
  analytical interpenetration rings (`y = ±x`, `z = ±√(25 − x²)`,
  `x ∈ [-5, +5]`) tessellate to ~ 101–104 actual `(face_a, face_b)`
  pair candidates in mesh-repair (count varies 101 → 104 across runs
  due to Rayon parallel-loop non-determinism — per-thread counters
  trip the cap independently), capped at `max_reported = 100` in the
  returned vec; description carries the truncation suffix. Severity
  Critical → `is_printable() == false`. `overhangs.len() == 4` lateral
  underside co-flag (each cluster's `overhang_angle = 56.25°` from
  azimuth offset `±22.5°` off `−Z`; classified `Info` since `45° <
  56.25° < 75°` — not Critical, not Warning, doesn't block
  `is_printable()`). **Single-cylinder regression**: re-runs validation
  on cylinder A alone (still horizontal, still placed on build plate)
  and asserts `self_intersecting.len() == 0` — locks the no-false-
  positives-on-convex-single-mesh invariant. Saves `out/mesh.ply`
  (68v / 128f, ASCII) + `out/issues.ply` (104 vertex-only centroids:
  100 `approximate_location` points clustered around the four rings
  near `(0, 0, 5)` + 4 `OverhangRegion.center` points at the lateral
  underside region centroids). README front-matter callout per
  `feedback_f3d_winding_callout` documents that the four
  interpenetration rings produce z-fighting in `f3d`'s default
  rendering — that visual mess IS the detector's input signal. Crate
  name `example-mesh-printability-self-intersecting` per §7.0 + §12.3's
  example-commit naming convention. **§6.4 re-export gap surfaced**:
  the §3 spec at line 222 calls for `pub use
  mesh_repair::intersect::{IntersectionParams, SelfIntersectionResult};`
  to give power users an ergonomic tuned-params escape hatch from
  within `mesh-printability`. Row #16 landed the detector but did not
  ship that re-export; the example documents the gap inline (README
  pitfall section + module doc-comment) and flags it as a v0.9
  candidate. Pure addition; fourth ⏸ pause-for-visuals commit per
  §12.6 row 4.
- **`SmallFeature` detector via connected-component bbox extent (Gap J, §6.5).**
  Final v0.8 detector. v0.7 exposed `PrintIssueType::SmallFeature`
  but `validate_for_printing` never populated any field — silent skip
  for floating debris, isolated tiny protrusions, and unit-conversion
  errors (a mesh authored in metres looks like a single sub-millimetre
  fragment under FDM's 0.8 mm resolution). v0.8 wires a private
  `check_small_features(mesh, config, validation)` that partitions
  every face of the mesh into edge-connected components (DFS over
  shared-edge adjacency, regardless of orientation or non-manifold
  edge incidence — broader than `partition_flagged_into_components`'s
  manifold-only adjacency, since §6.5's small-feature definition is
  purely topological), computes per-component AABB max-extent, and
  flags components whose `max_extent < config.min_feature_size`.
  New `SmallFeatureRegion` typed-region (`center`, `max_extent`,
  `volume`, `face_count`, `faces`) and new
  `PrintValidation.small_features` field. Volume comes from the
  divergence-theorem sum `Σ (v0 · (v1 × v2)) / 6` over component
  faces, then `abs(...)` so open / non-manifold components produce
  finite, non-negative values (non-physical but no-panic; documented
  as approximate for non-watertight inputs). Severity classifier:
  `max_extent < min_feature_size / 2` → Warning (definitely below
  resolution); else → Info (borderline; may print). No Critical band
  — small features are advisory, not blocking, so even a unit-
  conversion-mistaken mesh entirely below resolution does not flip
  `is_printable()` to false. Per-region `(face_indices)` is sorted
  ascending by `partition_all_faces_into_components`; the per-detector
  §4.4 sort orders regions by `min(faces)` ascending. New private
  helpers: `partition_all_faces_into_components` (DFS, every face is
  a seed including isolated islands; re-uses `build_edge_to_faces`
  from §5.3), `signed_volume` (verbatim spec form, no `mul_add`
  substitution to preserve cross-platform bit semantics on exact-
  representable inputs), `classify_small_feature_severity`.
  Tolerant of any input — no preconditions, no `DetectorSkipped`
  path: empty/single-face/open/non-manifold/NaN-vertex meshes all
  flow through without panic. Ten unit tests in `validation.rs::tests`
  cover §6.5's adversarial cases (floating triangle Warning,
  borderline `0.5 ≥ 0.4` no-issue, below-half-threshold Warning,
  just-below-threshold Info, clean main body, two floating fragments,
  vertex-only adjacency = 2 components, divergence-theorem volume
  ≈ 1.0 mm³ within 1e-6 on a unit cube, open-component finite volume
  no-panic, sort stability across runs). Eight §9.2.7 stress fixtures
  in `tests/stress_inputs.rs` (clean 30 mm cube no-flag, 0.2 mm
  hex-prism burr Warning, 30-µm cube unit-conversion diagnostic,
  below-threshold Warning vs just-below-threshold Info severity bands,
  open 5-of-6-face cube no-panic, vertex-only adjacency = 2 components,
  unit-cube divergence-theorem 1.0 mm³ within 1e-6). All eight are
  Light (no `#[cfg_attr(debug_assertions, ignore)]`); the algorithm
  is `O(n_faces)` so no perf-budget concern. **v0.9 followups (§6.5)**:
  curvature-based detection (small bumps on a larger body), volume-
  based threshold (long thin spikes that pass extent but fail volume).

### Changed

- Inherited workspace lints (`[lints] workspace = true`); 6 per-statement
  `#[allow]`s preserve FP semantics on overhang-predicate sites where FMA
  (`mul_add`) and midpoint forms would shift FP bits at the threshold
  boundary. Bit-exactness deferral is tracked under
  [v0.9 candidates](#v09-candidates).
- Extracted `build_edge_to_faces` private helper from
  `check_basic_manifold` so the same edge→face-list map can be shared
  with `check_overhangs`' region-split logic (Gap D, next commit). Pure
  refactor; the existing manifold/watertight regression tests
  (`test_not_watertight_detection`, `test_watertight_mesh`,
  `test_validation_summary`, `test_issue_counts`,
  `test_sls_no_overhang_check`) pass without modification.

### Fixed

- **Overhang flagging predicate corrected to FDM-slicer convention
  (Gap M).** The v0.7 predicate
  (`if dot < 0 { angle = π - acos(dot); flag if angle > max }`) flagged
  faces whose normals tilted between vertical and ~135° from up but
  silently *missed* pure roofs (`dot = -1`, `angle = 0`). The corrected
  form is `overhang_angle = acos(dot) - π/2; flag if > max`, matching
  the convention used by PrusaSlicer's "support overhang threshold" and
  Cura's "support angle" — faces tilted more than `max_overhang_angle`
  from vertical now correctly flag, including roofs. **SEMVER-significant
  behavioural change** for callers depending on v0.7 `is_printable()`
  outputs on overhang-near-roof meshes.
- **Build-plate filter added to `check_overhangs` (Gap M.2).** Under
  the corrected predicate above, a solid object's bottom face has
  `overhang_angle = 90°` and would otherwise flag as Critical. A face
  whose minimum projection along build-up is within `EPS_GEOMETRIC`
  (1e-9 mm) of the mesh-minimum is excluded — it is supported by the
  build plate itself. Mesh-min-relative, so it works whether or not
  callers have called `place_on_build_plate`. Mirrors the pattern
  already used by the long-bridge detector.
- Symmetric edits in `evaluate_orientation` (`orientation.rs`) so
  `find_optimal_orientation`'s scoring matches `validate_for_printing`'s
  predicate under candidate rotations.
- **`OverhangRegion.angle` now reports the actual maximum observed
  overhang angle (Gap B).** v0.7's `OverhangRegion.angle =
  config.max_overhang_angle + 10.0` was a hardcoded approximate that did
  not reflect the geometry of the flagged faces. `check_overhangs` now
  tracks the running max of the per-face `overhang_angle` and reports
  the steepest face's tilt-from-vertical (in degrees) at region
  creation. Composes with Gap M (predicate fix above) so post-v0.8 the
  reported angle ranges in `[0°, 90°]` for downward-facing flagged
  faces, matching the FDM-slicer convention. Sets up the Gap E
  severity classifier (commit #6) which will consume `angle` to assign
  Critical / Warning / Info severity bands.
- **Overhangs now split into connected regions (Gap D).** v0.7's
  `check_overhangs` lumped ALL flagged faces into a single
  `OverhangRegion` (centroid taken from `overhang_faces[0]`'s position;
  reported `angle` was the global max; `area` was the global sum).
  v0.8 partitions flagged faces into connected components by
  shared-manifold-edge adjacency (using the `build_edge_to_faces`
  helper extracted in the previous commit). Adjacency contract: two
  flagged faces share a component iff they share a manifold edge
  (incident on exactly 2 faces); non-manifold edges (>2 incident
  faces) and open edges (1 incident face) do not contribute adjacency,
  and faces sharing only a vertex are not adjacent. Each component
  emits one `OverhangRegion` (centroid = mean of per-face centroids;
  `angle` = per-component max; `area` = per-component sum) and one
  matching `SupportRegion`, preserving the
  `support_regions.len() == overhangs.len()` 1:1 invariant at
  component granularity. One `ExcessiveOverhang` `PrintIssue` is
  emitted per region (was: one summary issue across all flagged
  faces) — sets up Gap E (commit #6) which will replace the
  area-based per-region severity with the §4.3 angle-based classifier.
  Components emerge in min-face-idx order and faces within each
  region are sorted ascending for deterministic output across
  HashMap iteration permutations.
- **`ExcessiveOverhang` severity is now angle-based per §4.3
  (Gap E).** v0.7's severity heuristic capped at Warning even for
  near-90° roofs (`if total_area > 1000.0 { Warning } else { Info }`)
  — a small-area "scary" overhang couldn't flip `is_printable()` to
  false. v0.8 classifies per-region severity from the per-region max
  overhang angle (which the Gap D partition produces): observed
  `angle > threshold + 30°` → Critical (e.g., 80° on a 45° threshold);
  `threshold + 15° < observed ≤ threshold + 30°` → Warning (e.g., 65°);
  `threshold < observed ≤ threshold + 15°` → Info (e.g., 50°).
  Critical severities now cause `is_printable()` to return `false`,
  matching FDM-slicer convention (PrusaSlicer's "support overhang
  threshold" + Cura's "support angle" categorize roof faces as
  block-the-print-equivalent). **SEMVER-significant behavioural
  change** for callers depending on v0.7 `is_printable()` outputs on
  meshes with > 75° flagged faces. Helper: new private
  `classify_overhang_severity(observed_deg, threshold_deg)` is the
  single source of policy.
- **Manifold check now detects inconsistent winding orientation
  (Gap F).** v0.7's `check_basic_manifold` treated edges as undirected
  `(min, max)`-normalized pairs and only counted sharing — silently
  passing meshes where two faces share an edge but traverse it in the
  *same* direction (one face is "inside out" relative to the other).
  FDM slicers reject such meshes in practice. v0.8 adds a directed-edge
  pass alongside the existing undirected pass: for each face, the three
  traversals `(face[0], face[1])`, `(face[1], face[2])`,
  `(face[2], face[0])` are recorded; any directed pair appearing more
  than once is a winding-orientation defect. Both detectors push under
  `PrintIssueType::NonManifold`; the new issue's description contains
  "winding inconsistency" so callers can discriminate from open-edge
  ("open edge(s)") and non-manifold-edge ("non-manifold edge(s)") cases
  by string-matching. **SEMVER-significant behavioural change** for
  callers depending on v0.7 `is_printable()` outputs on meshes with
  inconsistent face orientations — they now see a Critical issue and
  `is_printable()` returns `false`.

### v0.9 candidates

These deferrals are tracked here so per-site `#[allow]` justification
comments can reference a stable anchor that survives v0.8 spec deletion.

- **FP bit-exactness of overhang predicate (§5.1 deferral).** v0.8 keeps
  the non-FMA forms `(a * b) + c` and `(a + b) / 2.0` to preserve current
  overhang-detection FP bits across platforms. Re-open trigger: a
  cross-platform CI run on Gap H or Gap M fixtures shows divergence at
  the bit level (e.g., affected-face count differs by 1 between
  macOS/Linux/Windows on a fixture authored to land *exactly* on the
  threshold). Resolution path: either replace the per-site `#[allow]`s
  with `f64::mul_add` / `f64::midpoint` after a tolerance-based diff
  confirms semantic equivalence, or reframe the predicate to be
  FMA-stable across platforms.
- **Cavity-ceiling co-flag (§7.9 / §11.5).** Under the corrected Gap M
  predicate, the *ceiling* of a sealed interior cavity inherently flags
  as a 90° overhang because its outward normal points down into the
  void. This is technically correct given the predicate's definition —
  but a ceiling supported by surrounding solid material has no
  user-actionable overhang concern. v0.8 emits the flag; downstream
  callers can mask it by inspecting `overhangs[i].faces` against an
  enclosed-cavity classifier. Re-open trigger: a user requests
  cavity-aware overhang severity (e.g., "demote interior overhangs to
  Info" or "skip overhangs interior to detected cavities"). Resolution
  path: extend `OverhangRegion` with an `is_interior` field populated by
  cross-referencing flagged faces against `TrappedVolume`-detected
  cavities (which lands in v0.8 as the trapped-volume detector,
  commit #14).

## [0.7.0] - 2025-XX-XX

- Initial release with build-volume, overhang, and basic-manifold detectors.
