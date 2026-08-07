# Changelog

All notable changes to mesh-repair will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.0.0]

### Breaking

- **`MeshReport` gains a `winding: WindingCensus` field, and is now
  `#[non_exhaustive]`.** Both are major changes for external consumers that
  construct a `MeshReport` with a struct literal. No in-workspace caller does
  — the only construction sites are inside this crate — so the break is
  invisible in-tree and visible only to crates.io consumers.

  `#[non_exhaustive]` is added *with* this break rather than after it: the
  field addition already costs a major version, so making every future field
  addition a minor change is free here and would not be free later.

  **Migration:** build reports with `validate_mesh` / `validate_mesh_with_options`
  rather than by literal. If you were matching exhaustively on the struct, add
  a `..` rest pattern.

### Added

- **`validate_mesh` now reports per-edge winding consistency.** `MeshReport`
  carries a `WindingCensus` alongside the pre-existing `is_inside_out`. The
  two answer different questions and neither subsumes the other — see the
  `MeshReport` type docs.

### Fixed

- **`MeshReport::is_printable` no longer documents a guarantee it cannot
  make.** Its doc claimed the mesh must "have correct winding"; the
  implementation tests `!is_inside_out`, an origin-apex signed-volume sum
  that is not a winding check. The predicate's *behaviour* is unchanged —
  only the claim is corrected.
- **`Display` no longer prints `Winding: Correct`.** That line was derived
  from `is_inside_out` alone. It is replaced by two separately-labelled
  readings: the global signed-volume sign, and the local per-edge census.
- **`ValidationOptions::check_winding` documents its trap.** Setting it
  `false` makes `is_inside_out` a hardcoded `false`, indistinguishable from a
  measured result.

## [1.0.0]

Initial published release.
