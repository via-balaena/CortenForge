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
