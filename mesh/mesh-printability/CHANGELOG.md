# Changelog

All notable changes to mesh-printability will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

### Changed

- Inherited workspace lints (`[lints] workspace = true`); 6 per-statement
  `#[allow]`s preserve FP semantics on overhang-predicate sites where FMA
  (`mul_add`) and midpoint forms would shift FP bits at the threshold
  boundary. Bit-exactness deferral is tracked under
  [v0.9 candidates](#v09-candidates).

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
