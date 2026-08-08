# Changelog

All notable changes to mesh-repair will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [2.0.0] - 2026-08-07

### Removed

- **`Default` on `MeshReport` and on `WindingCensus`.** Both are measurements;
  a default-constructed one reports `is_inside_out: false` and an all-zero
  census that nothing computed. Obtain them from `validate_mesh` and
  `winding_census`.

### Changed

- **`MeshReport` and `WindingCensus` are `#[non_exhaustive]`.** Struct
  expressions and exhaustive patterns from other crates no longer compile.

- **`ValidationOptions::check_winding` is renamed `winding_census`, and gates
  only the census.** `MeshReport::is_inside_out` is now **always measured**.

  In 1.0.0, `check_winding: false` made `is_inside_out` a hardcoded `false`,
  indistinguishable from a measured result. **If you set that flag, you will
  now see a real value where 1.0.0 gave you `false`.**

  `ValidationOptions` itself keeps `Default` and is not `#[non_exhaustive]`, so
  `ValidationOptions { winding_census: false, ..Default::default() }` still
  works.

- **`MeshReport`'s `Display` replaces its `Winding:` line with two lines.**
  Both former values — `Winding: Correct` and `Winding: Inside-out` — are gone.
  The `Status:` block now ends with:

  ```text
      Signed volume: non-negative (global, origin-apex)
      Local winding: 0 of 6 interior edges inconsistent
  ```

  `Signed volume:` is `negative` or `non-negative`. `Local winding:` is
  `N of M interior edges inconsistent`, `no judgeable interior edge`, or
  `not measured`.

- **`WindingCensus`'s `Display` suffix** is now
  `" | INCONCLUSIVE: no edge was judged"`, previously
  `" | INCONCLUSIVE: no edge has two incident faces"` — which named a cause the
  type cannot determine.

### Added

Everything here is new to a 1.0.0 consumer, including items that landed on the
development branch between the two releases.

- **`winding_census` and `WindingCensus`** — per-edge orientation consistency,
  with `has_inconsistent_winding()` and `has_judgeable_edges()`. This is the
  *local* instrument; `MeshReport::is_inside_out` is a global signed-volume
  test, and neither subsumes the other. See `MeshReport`'s docs for which
  answers what.
- **`MeshReport::winding: Option<WindingCensus>`** — `validate_mesh` now runs
  the census and reports it. `None` when `winding_census` is off, which keeps
  "never ran" distinct from "ran and had nothing to judge".
- **`taubin_smooth_vertices`**, with `TAUBIN_DEFAULT_LAMBDA` and
  `TAUBIN_DEFAULT_MU`.
- Several inherent methods became `const fn`, including
  `MeshReport::is_printable` / `has_issues` / `issue_count`,
  `RepairSummary::had_changes`, `BoundaryLoop::edge_count` / `is_valid`,
  `ComponentAnalysis::is_connected`, `SelfIntersectionResult::is_clean`, and
  the `RepairParams::with_*` builders.

### Migration

- **Fixtures must be measured.** With no `Default` and `#[non_exhaustive]`
  there is no way to construct either type from nothing: a struct expression
  fails with **E0639** (plus **E0277** from the `Default::default()` term if
  you used functional-update syntax), and an exhaustive pattern with **E0638**.
  Build a mesh with the property under test and validate it — `validate_mesh`'s
  own doc example does this for `boundary_edge_count == 3`.

  Fields stay `pub`, so a report can still be modified after measuring.

- **`check_winding: false`** becomes `winding_census: false`, and no longer
  suppresses `is_inside_out`.

- **Reading the census.** `report.winding` is an `Option`, and a verdict means
  nothing where the census had no edges to judge:

  ```rust
  let locally_clean = report
      .winding
      .is_some_and(|c| c.has_judgeable_edges() && !c.has_inconsistent_winding());
  ```

- **`is_printable` is unchanged**, but its 1.0.0 doc claimed it required
  "correct winding", which it never checked. If you relied on that, add the
  census term above.

- **If you re-export `MeshReport`**, re-export `WindingCensus` too, or your
  consumers can read the field without being able to name its type.

### Performance

- ⚠ **`validate_mesh` is ~28 % slower** when the census runs, which it does by
  default. Measured on `mesh-repair-benches`' `Validation` group against
  1.0.0: +29.5 % / +26.8 % / +28.4 % / +26.5 % / +27.7 % at 12 / 80 / 320 /
  1280 / 5120 faces. Absolute: +0.4 µs at 12 faces, +204 µs at 5120;
  extrapolating linearly past the top of that range gives roughly +0.6 ms on a
  15k-face mesh.

  Set `winding_census: false` to opt out. `is_inside_out` is unaffected.

### Fixed

- **Documentation only; no behaviour changed.** `is_printable`'s "correct
  winding" claim (above), and the description of `is_inside_out`, which
  previously said the signed-volume sum is translation-invariant whenever
  winding is consistent and that the flag tracks distance from the origin.
  Invariance requires `Σ A_f n_f = 0`, which needs the surface to be *closed*
  as well as consistently wound, and the dependence is a half-space in the
  translation rather than a distance.

## [1.0.0] - 2026-05-03

### Added

- First stable release. No functional changes from 0.7.0; the semver-major
  bump indicates API stability commitment per the workspace v1.0 milestone.

## [0.7.0]

Initial release.
