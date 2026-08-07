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

### Performance

- ⚠ **`validate_mesh` is ~27 % slower.** Measured, not estimated, on
  `mesh-repair-benches`' `Validation` group against the 1.0.0 baseline:
  +29.5 % / +26.8 % / +28.4 % / +26.5 % / +27.7 % across 12 → 5120 faces
  (p = 0.00). In absolute terms +0.4 µs on a 12-face cube and +204 µs on a
  5120-face sphere, so roughly +0.6 ms on a 15k-face anatomical mesh.

  **Cause:** `winding_census` builds its own edge map and cannot reuse the
  `MeshAdjacency` that `validate_mesh` has already built three lines above —
  the census needs per-edge traversal *direction*, which `MeshAdjacency` does
  not record, and there is no all-edges iterator on it.

  **Known offset, deliberately not taken here:** `MeshAdjacency::build`
  allocates a `vertex_to_faces` map that `validate_mesh` never reads. A
  `build_edges_only` constructor would cut roughly half the adjacency cost of
  *every* `validate_mesh` caller and could pay for the census outright. It is
  a separate change with its own measurement, and is not bundled with this
  release.

  Callers who only want topology can set `ValidationOptions::check_winding`
  to `false` — but read that field's docs first, because it also disables
  `is_inside_out`.

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
