# Changelog

All notable changes to mesh-repair will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [2.0.0]

### Added

- **`validate_mesh` now reports per-edge winding consistency.** `MeshReport`
  carries `winding: Option<WindingCensus>` alongside `is_inside_out`. The two
  answer different questions: a *global* flip leaves the census clean and sets
  `is_inside_out`; a *local* flip does the reverse.

### Breaking

- **`MeshReport` gains `winding`, and both it and `WindingCensus` are now
  `#[non_exhaustive]`.** No caller outside this crate constructs either type,
  so the break is invisible in-tree.

  `#[non_exhaustive]` lands on both *with* the field rather than after it: the
  field already costs a major version, and `WindingCensus` is the type most
  likely to grow counters, so paying once here makes later counters additive.

- **`ValidationOptions::check_winding` is renamed `winding_census`, and now
  gates only the census.** `MeshReport::is_inside_out` is **always measured**.

  Previously one flag gated both instruments, and a disabled `is_inside_out`
  became a hardcoded `false` — indistinguishable from a measured result, so any
  gate asserting it had not fired was silently satisfied. It is no longer
  gated: it is a single allocation-free pass, and gating it bought nothing but
  a field that could not report having been skipped. Callers who set the old
  flag to `false` will now see a real value where they previously saw `false`.

- **`MeshReport::winding` is an `Option`, not a bare census.** `None` means the
  census did not run. An all-zero `WindingCensus` is indistinguishable from a
  clean one on five of its six counters, so "not measured" needed to be
  representable rather than documented.

- **`MeshReport`'s `Display` no longer prints `Winding: Correct`.** That line
  was derived from `is_inside_out` alone, which cannot support it. It is
  replaced by a signed-volume line plus one of three winding lines —
  `N of M interior edges inconsistent`, `no judgeable interior edge`, or
  `not measured`. Anything parsing this output will see different text.

- **`WindingCensus`'s `Display` suffix** changed from
  `" | INCONCLUSIVE: no edge has two incident faces"` to
  `" | INCONCLUSIVE: no edge was judged"` — the old wording named a cause it
  could not know. User-visible: `cf-fsu-geometry`'s `SurfaceReport` prints it
  verbatim.

### Migration

- **Struct literals.** `#[non_exhaustive]` blocks the struct *expression* —
  both `MeshReport { .. }` and functional-update syntax
  `MeshReport { boundary_edge_count: 3, ..Default::default() }` (E0639).
  It does **not** block `Default` or field assignment, and the fields stay
  public, so a hand-built fixture becomes:

  ```rust
  let mut report = MeshReport::default();
  report.boundary_edge_count = 3;
  ```

  Use `validate_mesh` on a real mesh for integration fixtures; the snippet
  above is the direct replacement for a unit-test literal.

- **Exhaustive matches** need a `..` rest pattern.

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
  5120-face sphere, so roughly +0.6 ms on a 15k-face anatomical mesh. The
  overhead is linear and holds to at least 200k faces.

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
  predicate's behaviour is unchanged.
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

## [1.0.0]

### Added

- First stable release. No functional changes from 0.7.0; the semver-major
  bump indicates API stability commitment per the workspace v1.0 milestone.

## [0.7.0]

Initial release.
