# Changelog

All notable changes to mesh-repair will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [2.0.0] - 2026-08-07

### Removed

- **`Default` on `MeshReport`.** A report is a measurement, and a
  default-constructed one claims `is_inside_out: false` without having computed
  anything. Obtain reports from `validate_mesh`.

### Changed

- **`MeshReport` is `#[non_exhaustive]`.** Struct expressions and exhaustive
  patterns from other crates no longer compile.

- **`ValidationOptions::check_winding` is renamed `winding_census`, and gates
  only the census.** `MeshReport::is_inside_out` is now **always measured**.

  In 1.0.0, `check_winding: false` made `is_inside_out` a hardcoded `false`,
  indistinguishable from a measured result. **If you set that flag, you will
  now see a real value where 1.0.0 gave you `false`** — and because
  `is_printable` reads that field, its result can change with it. On a
  watertight, manifold, inward-wound mesh with the flag off, 1.0.0 reported
  printable and 2.0.0 does not.

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

- **`detect_self_intersections` now finds candidate pairs with a BVH** instead
  of an exhaustive scan. `SelfIntersectionResult::intersecting_pairs` is **no
  longer ordered by face index**, and with `max_reported` set you may get a
  *different subset* of pairs than 1.0.0 returned. Sort the result if you
  depend on order, and do not assume truncation keeps the lowest-indexed pairs.

- **`is_inside_out` and `fill_holes` can differ on near-degenerate input.**
  Two internal predicates were rewritten to use fused multiply-add between the
  releases, changing floating-point association. On meshes whose signed volume
  sits within rounding error of zero (open surfaces, balanced shells),
  `is_inside_out` — and therefore `is_printable` — can differ from 1.0.0; on
  holes with near-collinear boundary vertices, `fill_holes` can emit a
  different, still-valid triangulation. Both regimes are ones where the result
  was never well-determined, but they are not identical to 1.0.0.

- **`SelfIntersectionResult::intersection_count` is a lower bound when
  `truncated`.** It is incremented before the `max_reported` cap is applied, so
  it can exceed both the cap and `intersecting_pairs.len()`, and is not
  reproducible run to run.

- **New dependency: `parry3d-f64`** (for the BVH above). It pulls in `nalgebra`
  0.33 alongside this crate's 0.34, so a duplicate-dependency policy that
  passed on 1.0.0 may fail on 2.0.0.

- **License is now `Apache-2.0`**, previously `MIT OR Apache-2.0`. If you
  relied on the MIT option, this removes it.

### Added

- **`winding_census` and `WindingCensus`** — per-edge orientation consistency,
  with `has_inconsistent_winding()` and `has_judgeable_edges()`. It has no
  `Default`, is `#[non_exhaustive]`, and its `Display` appends
  `" | INCONCLUSIVE: no edge was judged"` when nothing was judgeable.

  This is the *local* instrument. `MeshReport::is_inside_out` is a global
  signed-volume test, and **neither, nor both together, verifies orientation on
  a multi-component mesh**. The census never compares faces across shells, and
  the volume sum reports whichever way the total lands — so an inverted shell
  may leave the flag clear, or set it while the surface is mostly correct. See
  `WindingCensus`'s "What this does NOT answer", and `split_into_components`
  to check each shell on its own — noting that a legitimate enclosed cavity is
  correctly wound *inward*, so a per-shell check alone will flag hollow parts.
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
  there is no way to construct a `MeshReport` from nothing: a struct expression
  fails with **E0639** (plus **E0277** from the `Default::default()` term if
  you used functional-update syntax), and an exhaustive pattern with **E0638**.
  The nearest replacement for `MeshReport::default()` is a measurement of an
  empty mesh, and fields stay `pub` so you can assign from there — but ⚠ **it
  is not the all-false value `Default` gave you.** An empty mesh has no edges,
  so `is_watertight` and `is_manifold` are vacuously `true` and
  `is_printable()` returns `true`; `winding` is `Some` of an all-zero census,
  not `None`. **Set every field your fixture depends on, explicitly:**

  ```rust
  use mesh_types::IndexedMesh;

  let mut report = validate_mesh(&IndexedMesh::new());
  report.is_watertight = false; // ⚠ the empty-mesh value is `true`
  report.is_manifold = false;   // ⚠ likewise
  report.is_inside_out = true;
  ```

  Functional update from a measured report — `MeshReport { is_watertight:
  true, ..measured }` — is **E0639** too; `#[non_exhaustive]` blocks the
  struct expression regardless of where the base came from.

  Where the mesh itself matters, build one with the property under test and
  validate it — `validate_mesh`'s own doc example does this for
  `boundary_edge_count == 3`.

- **`check_winding: false`** becomes `winding_census: false`, and no longer
  suppresses `is_inside_out`.

- **Reading the census.** `report.winding` is an `Option`, and a verdict means
  nothing where the census had no edges to judge:

  ```rust
  let locally_clean = report
      .winding
      .is_some_and(|c| c.has_judgeable_edges() && !c.has_inconsistent_winding());
  ```

- **`is_printable`'s predicate is unchanged**, but see the `winding_census`
  entry above — its *input* can change if you used the old flag. Its 1.0.0 doc
  also claimed it required "correct winding", which it never checked; add the
  census term above if you relied on that.

- **`detect_self_intersections` assertions.** Pairs are canonical (`a < b`)
  but no longer face-index ordered, so compare as a set or sort first. If you
  assert on `intersection_count`, note it is a lower bound once `truncated` is
  set — and `truncated` is also set when the count lands exactly on
  `max_reported` with nothing dropped.

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

- **Documentation corrections** (numerical changes are under *Changed*). 1.0.0
  documented `is_inside_out` as "whether the mesh appears to be inside-out
  (majority of volume is negative)". It is not a majority test — it is the
  sign of a single
  origin-apex signed-volume sum — and the result is frame-dependent unless
  `Σ A_f n_f = 0`, which holds for a closed, consistently-wound surface. The
  field now documents both.
- `is_printable`'s "correct winding" claim, as above.

## [1.0.0] - 2026-05-03

### Added

- First stable release. No functional changes from 0.7.0; the semver-major
  bump indicates API stability commitment per the workspace v1.0 milestone.

## [0.7.0]

Initial release.
