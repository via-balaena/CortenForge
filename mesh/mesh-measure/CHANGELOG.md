# Changelog

All notable changes to mesh-measure will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **`OrientedBoundingBox::contains_with_tol(point, eps)`** — tolerance-aware
  containment. A vertex that defines the OBB (an extreme along a principal
  axis) projects to the local-frame boundary, where PCA's `SymmetricEigen`
  leaves ~1 ULP of roundoff, so strict `contains` could reject it by
  `~1.78e-15`. `contains` is now `contains_with_tol(p, 0.0)`. Surfaced by the
  `measure` stress-test, which previously hand-rolled a tolerant
  reimplementation; it now calls the library method.
- **Rotated-box OBB extent-recovery test** — a non-degenerate 20×12×8 box
  rotated 45° whose OBB recovers the true `(20, 12, 8)` extents (the defining
  OBB-vs-AABB behavior, previously only exercised on axis-aligned cubes).

### Changed

- **`CrossSection`/`Contour` centroid is now the true polygon centroid**
  (area-weighted shoelace moments in the plane frame), replacing the naive
  `sum(points) / points.len()` average that a chain-closure-duplicate vertex
  biased away from the geometric centroid (~0.077 mm on the 32-gon cylinder
  mid-slice; now `(0, 0, 5)` to ~3e-17). The 2D projection basis is shared
  with the area computation via the new `plane_basis` helper. Surfaced by the
  `measure` stress-test. *Consumer note:* callers reading `.centroid` now get
  the geometric centroid, not the point-mean — a value change, not an API one.

### v0.9 candidates

These backlog candidates are gated on a real consumer driving them per
the platform's "examples drive gap-fixes" discipline. Each entry names
the consumer-arrival shape that re-opens the work. The platform-wide
list (sixteen candidates spanning four crates) lives in
[Part 10 of the mesh architecture study](../../docs/studies/mesh_architecture/src/100-roadmap.md);
the entries below are the mesh-measure-specific subset.

- **`MeasureError` / `MeasureResult` adoption.** Currently `Option<T>`
  carries the same information for callers; tightening to
  `Result<T, MeasureError>` is API-shape-breaking. *Trigger*: a
  function tightens validation to fail-fast (e.g., empty mesh becomes
  a hard error), OR a power user reports surprise that
  `dimensions()` returns `Default` rather than an error on degenerate
  input. Effort: ~80 LOC + breaking-API rename + downstream call-site
  updates.

- **Document "OBB ⊄ AABB in general" + remove the corners-within-AABB
  anchor pattern from API docs and examples.** Surfaced during
  `mesh-measure-bounding-box` spec authoring: the folk-intuition
  "OBB is tighter than AABB so OBB ⊆ AABB" is false for any
  non-trivial OBB rotation; OBB corners extend OUTSIDE the AABB
  envelope by `half_extent · sin(rotation_angle)`. *Trigger*: write
  the docstring + audit pass before another consumer encodes the
  wrong sanity check. Effort: ~10 LOC.


- **`closest_point_on_triangle` duplication consolidation (cross-crate).**
  See the corresponding `mesh-sdf` v0.9 candidate; the proposed home
  is `mesh-sdf` with `mesh-measure` re-exporting the consolidated
  function. Listed here so the dedup is discoverable from this
  crate's backlog.

## [1.0.0] - 2026-05-03

### Added

- First stable release. No functional changes from 0.7.0; the
  semver-major bump indicates API stability commitment per the
  workspace v1.0 milestone (the mesh ecosystem ships its full
  examples-coverage arc at v1.0.0).

## [0.7.0]

Initial release.
