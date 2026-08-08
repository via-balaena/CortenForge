# Changelog

All notable changes to mesh-repair will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [2.0.0] - 2026-08-07

### Added

- **`validate_mesh` now reports per-edge winding consistency.** `MeshReport`
  carries `winding: Option<WindingCensus>` alongside `is_inside_out`.

  **`winding` answers whether the winding is correct. `is_inside_out` does
  not** — it is the sign of a frame-dependent volume integral, which both a
  local flip and a correctly-wound *open* mesh can set.

  ⚠ **The census runs by default.** `validate_mesh` now performs it, so every
  existing call pays for it — see Performance.

### Breaking

- **`MeshReport` gains `winding`; both it and `WindingCensus` lose their
  `Default` derive and become `#[non_exhaustive]`.**

  Together these mean **a report can only be obtained by measuring**. A
  hand-built one could assert `is_inside_out` without ever computing it — the
  same defect the `Option` on `winding` exists to prevent, reachable from the
  construct side.

- **`ValidationOptions::check_winding` is renamed `winding_census`, and now
  gates only the census.** `MeshReport::is_inside_out` is **always measured**.

  In 1.0.0, `check_winding: false` made `is_inside_out` a hardcoded `false` —
  indistinguishable from a measured result, so any gate asserting it had not
  fired was silently satisfied. It is no longer gated: it is a single
  allocation-free pass (measured at ~0.5 % of `validate_mesh`), and gating it
  bought nothing but a field that could not report having been skipped.
  **Callers who set `check_winding: false` will now see a measured value where
  1.0.0 gave them `false`.**

- **`MeshReport`'s `Display` no longer prints `Winding: Correct`.** That line
  was derived from `is_inside_out` alone, which cannot support it. It is
  Anything parsing this output will see different text. The `  Status:` block
  now ends with two lines instead of one:

  ```text
      Signed volume: non-negative (global, origin-apex)
      Local winding: 0 of 6 interior edges inconsistent
  ```

  `Signed volume:` is `negative` or `non-negative`. `Local winding:` is
  `N of M interior edges inconsistent`, `no judgeable interior edge`, or
  `not measured`.

- **`WindingCensus`'s `Display` suffix** changed from
  `" | INCONCLUSIVE: no edge has two incident faces"` to
  `" | INCONCLUSIVE: no edge was judged"` — the old wording named a cause it
  could not know. User-visible: `cf-fsu-geometry`'s `SurfaceReport` prints it
  verbatim.

### Migration

- **Fixtures must be measured, not built.** `MeshReport` and `WindingCensus`
  have no `Default` and are `#[non_exhaustive]`, so there is no
  hand-construction route at all. A struct expression fails with **E0639**; a
  functional update fails with **E0277** (no `Default`) as well. Replace a literal fixture with
  `validate_mesh` over a mesh that actually has the property under test:

  ```rust
  // was: MeshReport { boundary_edge_count: 3, ..Default::default() }
  let mut mesh = IndexedMesh::new();
  // ...three vertices and one face: a lone triangle has three boundary edges
  let report = validate_mesh(&mesh);
  ```

  This is deliberate rather than incidental: a fabricated report can assert
  `is_inside_out` without ever having computed it.

- **Exhaustive matches** on either type need a `..` rest pattern (**E0638**).

- **`check_winding: false`** becomes `winding_census: false`. If you relied on
  it to suppress `is_inside_out`, note that it no longer does.

- **Reading the census.** `report.winding` is an `Option`. A verdict is
  meaningful only where the census had edges to judge:

  ```rust
  let locally_clean = report
      .winding
      .is_some_and(|c| c.has_judgeable_edges() && !c.has_inconsistent_winding());
  ```

- **If you re-export `MeshReport`**, re-export `WindingCensus` too — otherwise
  your consumers can read the field but cannot name its type.

### Performance

- ⚠ **`validate_mesh` is ~28 % slower** when the census runs. Measured on
  `mesh-repair-benches`' `Validation` group against the 1.0.0 baseline:
  +29.5 % / +26.8 % / +28.4 % / +26.5 % / +27.7 % across 12 → 5120 faces
  (p = 0.00; mean 27.8 %). Absolute: +0.4 µs on a 12-face cube, +204 µs on a
  5120-face sphere, so roughly +0.6 ms on a 15k-face anatomical mesh — the
  range the committed bench covers.

  **Cause:** `winding_census` builds its own edge map and cannot reuse the
  `MeshAdjacency` `validate_mesh` already built — it needs per-edge traversal
  *direction*, which that type does not record and exposes no all-edges
  iterator for.

  **Known offset, not taken here:** `MeshAdjacency::build` allocates a
  `vertex_to_faces` map `validate_mesh` never reads. A `build_edges_only`
  constructor would cut roughly half the adjacency cost of *every* caller and
  could repay the census outright. Separate change, separate measurement.

  Set `winding_census: false` to opt out; `is_inside_out` is unaffected.

### Fixed

- **`MeshReport::is_printable` no longer documents a guarantee it cannot
  make.** Its doc claimed the mesh must "have correct winding"; the
  implementation tests `!is_inside_out`, which is not a winding check. The
  predicate's behaviour is unchanged; if you wanted winding included, write it
  explicitly:

  ```rust
  report.is_printable()
      && report.winding.is_some_and(|c| {
          c.has_judgeable_edges() && !c.has_inconsistent_winding()
      })
  ```
- **The signed-volume mechanism is stated correctly, in one place.** Earlier
  docs said the sum is translation-invariant while the surface is consistently
  oriented, and that the flag tracks distance from the origin. Both are wrong:
  invariance requires `Σ A_f n_f = 0`, which needs closure as well as
  consistency, and the dependence is a half-space in the translation, not a
  distance. `MeshReport::is_inside_out` is now the single home for it; other
  docs link rather than restate.
- **Documented divergences that were previously silent:** the census's edge and
  face counters are not the report's own and can disagree (one index-repeating
  face is enough), and on a non-orientable surface `inconsistent_edges` is a
  topological obstruction that `fix_winding_order` cannot clear.

## [1.0.0] - 2026-05-03

### Added

- First stable release. No functional changes from 0.7.0; the semver-major
  bump indicates API stability commitment per the workspace v1.0 milestone.

## [0.7.0]

Initial release.
