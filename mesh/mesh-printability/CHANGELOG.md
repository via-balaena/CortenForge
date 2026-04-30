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
