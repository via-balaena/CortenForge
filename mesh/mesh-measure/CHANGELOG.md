# Changelog

All notable changes to mesh-measure will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

- **Tolerance-aware `OrientedBoundingBox::contains`.** Surfaced by
  `mesh-measure-bounding-box` (mesh book Part 8 Band 4): PCA's
  iterative `SymmetricEigen` produces `half_extents` and the
  inverse-rotation mapping with ~1 ULP roundoff; the four input
  vertices that defined the OBB extremes can fail strict `<=`
  containment by `~1.78e-15`. *Trigger*: a user reports a vertex
  they passed in failing `obb.contains(v)`. Effort: ~30 LOC: add
  `contains_with_tol(point, eps)` method, keep strict `contains` as
  alias for `contains_with_tol(p, 0.0)`.

- **Document "OBB ⊄ AABB in general" + remove the corners-within-AABB
  anchor pattern from API docs and examples.** Surfaced during
  `mesh-measure-bounding-box` spec authoring: the folk-intuition
  "OBB is tighter than AABB so OBB ⊆ AABB" is false for any
  non-trivial OBB rotation; OBB corners extend OUTSIDE the AABB
  envelope by `half_extent · sin(rotation_angle)`. *Trigger*: write
  the docstring + audit pass before another consumer encodes the
  wrong sanity check. Effort: ~10 LOC.

- **Proper polygon centroid in `CrossSection` (shoelace-weighted, not
  naive average).** Surfaced by `mesh-measure-cross-section` (mesh
  book Part 8 Band 4): the current centroid is
  `sum(points) / points.len()`, but `chain_segments` produces a
  contour with one chain-closure-duplicate point, biasing a symmetric
  polygon's centroid by `V_dup / (N + 1) ≠ (0, 0, 0)`. On the
  32-segment cylinder mid-slice the bias is ~0.077 mm in `(x, y)`.
  *Trigger*: a consumer needs ≤ 1e-10 centroid accuracy. Effort:
  ~30 LOC: replace the naive average with
  `c_x = (1 / (6A)) · Σ (x_i + x_(i+1)) · (x_i · y_(i+1) − x_(i+1) · y_i)`,
  projecting to 2D via the existing `(u, v)` basis lifted from
  `calculate_cross_section_area`.

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
