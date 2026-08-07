# Changelog

All notable changes to mesh-repair will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [2.0.0]

### Breaking

- **`MeshReport` gains a `winding: WindingCensus` field, and is now
  `#[non_exhaustive]`.** No in-workspace caller constructs a `MeshReport`, so
  the break is invisible in-tree and visible only to crates.io consumers.

  `#[non_exhaustive]` is added *with* the field rather than after it: the field
  already costs a major version, so making future fields additive is free here
  and would not be free later.

  **Migration.** Build reports with `validate_mesh` /
  `validate_mesh_with_options` rather than by literal, and add a `..` rest
  pattern to any exhaustive match.

  ⚠ **Functional-update syntax stops compiling** (E0639), including
  `MeshReport { boundary_edge_count: 3, ..Default::default() }` — the natural
  fixture idiom, and legal despite the `Default` derive only *inside* this
  crate. This is the part with no in-language substitute: the field addition
  alone would have left such code compiling; `#[non_exhaustive]` is what ends
  it. External fixtures must go through `validate_mesh` on a real mesh.

- **`MeshReport`'s `Display` no longer prints `Winding: Correct`.** That line
  was derived from `is_inside_out` alone, which cannot support it. Replaced by
  two separately-labelled readings: the global signed-volume sign, and the
  local per-edge census. Filed as breaking rather than fixed because anything
  parsing or snapshotting the output will see different text.

- **`WindingCensus`'s `Display` suffix changed** from
  `" | INCONCLUSIVE: no edge has two incident faces"` to
  `" | INCONCLUSIVE: no edge was judged"`. The old wording named a cause that
  is false for the all-zero census a `MeshReport` now carries when
  `check_winding` is off. User-visible: `cf-fsu-geometry`'s `SurfaceReport`
  prints it verbatim.

### Added

- **`validate_mesh` now reports per-edge winding consistency.** `MeshReport`
  carries a `WindingCensus` alongside the pre-existing `is_inside_out`. The two
  answer different questions and neither subsumes the other — see the
  `MeshReport` type docs.

### Performance

- ⚠ **`validate_mesh` is ~28 % slower.** Measured on `mesh-repair-benches`'
  `Validation` group against the 1.0.0 baseline: +29.5 % / +26.8 % / +28.4 % /
  +26.5 % / +27.7 % across 12 → 5120 faces (p = 0.00; mean 27.8 %). In absolute
  terms +0.4 µs on a 12-face cube and +204 µs on a 5120-face sphere, so roughly
  +0.6 ms on a 15k-face anatomical mesh.

  **Cause:** `winding_census` builds its own edge map and cannot reuse the
  `MeshAdjacency` that `validate_mesh` has already built — the census needs
  per-edge traversal *direction*, which `MeshAdjacency` does not record, and
  there is no all-edges iterator on it.

  **Known offset, deliberately not taken here:** `MeshAdjacency::build`
  allocates a `vertex_to_faces` map that `validate_mesh` never reads. A
  `build_edges_only` constructor would cut roughly half the adjacency cost of
  *every* `validate_mesh` caller and could pay for the census outright. It is a
  separate change with its own measurement.

  Callers who only want topology can set `ValidationOptions::check_winding` to
  `false` — but read that field's docs first, because it also disables
  `is_inside_out`.

### Fixed

- **`MeshReport::is_printable` no longer documents a guarantee it cannot
  make.** Its doc claimed the mesh must "have correct winding"; the
  implementation tests `!is_inside_out`, an origin-apex signed-volume sum that
  is not a winding check. The predicate's *behaviour* is unchanged — only the
  claim is corrected.
- **The signed-volume mechanism is stated correctly.** Earlier docs said the
  sum is translation-invariant while the surface is consistently oriented, and
  that the flag tracks distance from the origin. Both are wrong: invariance
  requires `Σ A_f n_f = 0`, which needs closure as well as consistency, and the
  dependence is a half-space in the translation, not a distance. Corrected in
  `MeshReport` and in `WindingCensus`, with producer tests for both
  counter-examples.
- **`ValidationOptions::check_winding` documents its trap.** Setting it `false`
  makes `is_inside_out` a hardcoded `false`, indistinguishable from a measured
  result.

## [1.0.0]

### Added

- First stable release. No functional changes from 0.7.0; the semver-major
  bump indicates API stability commitment per the workspace v1.0 milestone.

## [0.7.0]

Initial release.
