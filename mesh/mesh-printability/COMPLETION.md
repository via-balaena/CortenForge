# mesh-printability Completion

## Status: Production-ready for FDM/SLA/SLS/MJF print validation (v0.8.0)

`mesh-printability` is the print-validation crate: it consumes an
`IndexedMesh` + a `PrinterConfig` and returns a `PrintValidation`
enumerating `PrintIssue`s with severity, location, and per-detector
region descriptions. v0.8.0 closes the v0.7→v0.8 fix arc — five
new detectors are populated, the overhang predicate is corrected to
the FDM industry convention, and the build-up direction is
parametrized for non-`+Z` orientations.

## Detector inventory

Each `PrintIssueType` variant is populated by exactly one detector
(except `DetectorSkipped`, which any detector emits on precondition
failure, and `Other`, which is a caller-extension hook that
`mesh-printability` itself never emits).

| `PrintIssueType`    | Populated by                | Severity range         | Tier-aware? |
|---------------------|-----------------------------|------------------------|-------------|
| `ExceedsBuildVolume`| `check_build_volume`        | `Critical`             | No          |
| `ExcessiveOverhang` | `check_overhangs`           | `Info` / `Warning` / `Critical` | Yes (per-tech `max_overhang_angle`) |
| `NotWatertight`     | `check_basic_manifold`      | `Critical`             | No          |
| `NonManifold`       | `check_basic_manifold`      | `Critical`             | No          |
| `ThinWall`          | `check_thin_walls`          | `Warning` / `Critical` | Yes (per-tech `min_wall_thickness`) |
| `LongBridge`        | `check_long_bridges`        | `Warning` / `Critical` | Yes (FDM/SLA only; SLS/MJF skip silently) |
| `TrappedVolume`     | `check_trapped_volumes`     | `Info` / `Critical`    | Yes (`Info` on FDM; `Critical` on SLA/SLS/MJF) |
| `SelfIntersecting`  | `check_self_intersecting`   | `Critical`             | No          |
| `SmallFeature`      | `check_small_features`      | `Info` / `Warning`     | Yes (per-tech `min_wall_thickness`) |
| `DetectorSkipped`   | (any detector on precondition failure) | `Info`     | n/a         |
| `Other`             | caller-extension hook (never emitted by `mesh-printability`) | (caller-defined) | n/a |

Severity policy: `is_printable() == false` iff any issue is
`Critical`. Multi-tier bands are graded; the tier-aware detectors
read their threshold from `PrinterConfig` (constructed via the
`*_default()` helpers per technology, then optionally tuned via
`with_*` builders). Per-detector region types carry the geometry
needed to render or repair the flagged area.

## Public API

### Top-level entry points
- `validate_for_printing(mesh: &IndexedMesh, config: &PrinterConfig) -> PrintabilityResult<PrintValidation>` — runs every detector in order; returns the aggregated `PrintValidation`.
- `find_optimal_orientation(mesh: &IndexedMesh, config: &PrinterConfig, samples: usize) -> OrientationResult` — searches a discrete sample set of axis-aligned + Fibonacci-sphere rotations for minimum overhang area.
- `apply_orientation(mesh: &IndexedMesh, orientation: &OrientationResult) -> IndexedMesh` — applies a stored rotation to a mesh.
- `place_on_build_plate(mesh: &IndexedMesh) -> IndexedMesh` — translates the mesh so its bbox `min_z == 0`.

### Configuration
- `PrintTechnology` — enum: `FDM`, `SLA`, `SLS`, `MJF`, `Other`. `as_str()` + `requires_supports()` const methods.
- `PrinterConfig` — struct with `build_volume`, `min_wall_thickness`, `max_overhang_angle`, `max_bridge_span`, `min_feature_size`, `layer_height`, `build_up_direction` (Gap L), and `technology`.
- Per-technology constructors: `PrinterConfig::fdm_default()`, `sla_default()`, `sls_default()`, `mjf_default()` — all `pub const fn`.
- Builders: `with_build_volume`, `with_min_wall_thickness`, `with_max_overhang_angle`, `with_build_up_direction` (Gap L; the only non-`const` builder, since `Vector3::normalize` is non-const).

### Validation outputs
- `PrintValidation` — struct: `issues: Vec<PrintIssue>`, plus per-detector region vectors `overhangs`, `thin_walls`, `long_bridges`, `trapped_volumes`, `self_intersections`, `small_features`, `support_regions`. Methods: `is_printable()`, `critical_count()`, `warning_count()`, `total_support_volume()`, `summary()`.
- `PrintIssue` — struct: `issue_type: PrintIssueType`, `severity: IssueSeverity`, `description: String`, `location: Point3<f64>`, `affected_elements: Vec<u32>`. `new(...)` + `with_affected_elements(...)` builders.
- `IssueSeverity` — enum: `Info` (0), `Warning` (1), `Critical` (2).
- Region types (per `PrintIssueType`): `OverhangRegion`, `ThinWallRegion`, `LongBridgeRegion`, `TrappedVolumeRegion`, `SelfIntersectingRegion`, `SmallFeatureRegion`, `SupportRegion`. Each carries the centroid, area / span / volume, affected face indices, and (where relevant) classification metadata.
- `OrientationResult` — `rotation: Matrix3<f64>`, `score: f64`, plus diagnostic counters.

### Errors
- `PrintabilityError` — enum: `EmptyMesh`, `InvalidConfig(String)`, etc.
- `PrintabilityResult<T> = Result<T, PrintabilityError>`.

## Test coverage

| Source           | Count | Notes                                                  |
|------------------|------:|--------------------------------------------------------|
| Unit (`src/`)    |   124 | per-detector + per-region + per-builder + helper paths |
| Integration (`tests/stress_inputs.rs`) | 42 | per-detector stress fixtures + cross-detector cascades |
| Release-only ignored stress | 3 | `cfg_attr(debug_assertions, ignore)` perf fixtures (5k-tri ThinWall, voxel-grid memory cap, max-reported truncation) |
| Doc tests        |     4 | `lib.rs` README example + `validate_for_printing` + `PrinterConfig` + `find_optimal_orientation` |
| **Debug total**  |   173 | (124 lib + 42 integ + 3 ignored + 4 doc)               |
| Release total    |   172 | (123 lib + 45 integ + 0 ignored + 4 doc) — 1 lib test is `cfg(debug_assertions)`-gated; 3 ignored debug run unflagged in release |

Per-detector test density: each new v0.8 detector lands with ≥ 8
unit tests covering the watertight-precondition skip path, severity
bands, edge-case clusters, and per-config technology variations.
The release-only stress fixtures gate runtime-budget regressions
(e.g., 5k-tri brute-force ThinWall completes in < 2 s) that would
otherwise be invisible in CI's debug-only test matrix.

## Quality

- **Workspace lints inherited** since Gap A (commit 1c, `[lints] workspace = true`); 6 statement-scoped `#[allow(clippy::suboptimal_flops)]` / `#[allow(clippy::manual_midpoint)]` annotations suppress 9 underlying clippy warnings on overhang-predicate + orientation-score sites where FMA / midpoint forms would shift the final FP bit (deferred as v0.9 candidate per `CHANGELOG.md` `[Unreleased] / v0.9 candidates`).
- **Zero `clippy` warnings** on `cargo clippy -p mesh-printability --tests --all-targets -- -D warnings`.
- **Zero `rustdoc` warnings** on `RUSTDOCFLAGS=-D warnings cargo doc --no-deps -p mesh-printability`.
- **No `unwrap` / `expect` in library code** — `#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]` at the crate root; test sites use per-site `#[allow]` where the panic is local to the test.
- **8 grading criteria all A** (`cargo xtask grade mesh-printability`): Coverage 97.6% A+ (vs §11.2 v0.7 baseline 90.4%; +7.2% lift), Documentation A, Clippy A, Safety A, Dependencies A, Layer Integrity A, WASM Compat A, API Design (manual review) A.
- **Cross-platform CI** covers mesh-printability on macOS / Linux / Windows runners (Gap H FP-drift coverage in the `cross-os` job); release-mode CI covers the 3 release-only stress fixtures (`tests-release` job).

## Dependencies

| Dep            | Source     | Purpose                                                   |
|----------------|------------|-----------------------------------------------------------|
| `mesh-types`   | workspace  | `IndexedMesh`, `Point3`, `AttributedMesh` (vertex attrs)  |
| `mesh-repair`  | workspace  | `detect_self_intersections` re-use for Gap I (added v0.8) |
| `nalgebra`     | workspace  | `Vector3`, `Matrix3`, rotations                           |
| `thiserror`    | workspace  | derive `PrintabilityError`                                |
| `hashbrown`    | workspace  | swiss-table `HashMap` for edge / face adjacency           |
| `approx` (dev) | workspace  | `assert_relative_eq!` for numeric tests                   |

Tier classification (post-arc): **L0** (pure validation analysis on
indexed-mesh primitives; no I/O, no GPU, no async runtime). Release
graph dep count stays under L0's 80-cap (`cargo xtask grade`'s
Layer Integrity criterion). Re-open trigger for an L0 → L0-io tier
bump: a downstream consumer requests `tracing` instrumentation on
`mesh-printability` (currently tracing-free by design — keeps the
release graph wasm-compatible without per-feature gating).

## Known limitations (tracked for v0.9+)

The CHANGELOG.md `[Unreleased] / v0.9 candidates` block carries the
verbose form (each item with named re-open trigger + resolution
path); the project memo `project_mesh_printability_gaps.md` carries
the cross-session-AI form (concise list with triggers); the mesh
book `docs/studies/mesh_architecture/src/50-shell-and-print.md`
carries the human-narrative form. Triple-tracked: the same fact
lives in all three with the same wording — drift between any of
the three is a regression any one of the three's edits can catch.
Headline items:

- **No BVH acceleration** for `ThinWall` (brute-force O(n²) Möller-Trumbore) and `SelfIntersecting` (mesh-repair's BVH path is per-call, not amortized across the validate pipeline). Validation runtime is dominated by these two detectors at > 10k triangles. v0.9 candidate.
- **`IntersectionParams` not re-exported** from `mesh-printability::*` — callers who want to tune `epsilon`, `max_reported`, `allow_touching` must depend on `mesh-repair` directly. v0.9 candidate.
- **Heuristic estimates** for `support_volume` (`overhang_area * 5.0`) and the `material_volume` proxy (`bbox * 0.3`). Adequate for relative comparisons across orientations; not a substitute for slicer-grade volume computation.
- **Overhang-predicate FP bits not bit-exact** across FMA-vs-non-FMA platforms (the 9 per-site `#[allow(clippy::suboptimal_flops)]` deferral). Tolerance bands + cross-os CI mitigate.
- **Cavity-ceiling co-flag** under the corrected Gap M predicate: a sealed cavity's ceiling inherently flags as 90° overhang. Documented behavior; v0.9 candidate for cavity-aware overhang severity.
- **`find_optimal_orientation` discrete sample set** cannot reach arbitrary axis-aligned rotations (e.g., `R_Y(-60°)`). For exact-axis recovery, use `apply_orientation` with a hand-constructed `Matrix3`. v0.9 candidate.
- **`mesh-repair detect_self_intersections` false-positives** on thin-aspect-ratio prismatic geometry (cylinders L≥18mm at R=5mm, leaning-prism wing in `printability-showcase`). v0.9 candidate (carried up to mesh-repair).
- **Build-plate filter discrimination** (Gap M.2 over-aggressiveness): filter excludes any face whose lowest VERTEX touches the plate, even when the centroid + remaining vertices represent real overhang concern. Surfaced by `printability-showcase`. v0.9 candidate.
- **`§4.4` global severity-descending sort** of `validation.issues` not implemented; issues append in detector run order (overhangs → thin_walls → long_bridges → trapped_volumes → self_intersections → small_features). v0.9 candidate.

## Cross-references

- **CHANGELOG.md** — per-entry detail for every Gap A–M shipped + verbose v0.9 candidates anchor.
- **mesh book §50** (`docs/studies/mesh_architecture/src/50-shell-and-print.md`) — depth-pass narrative on the architecture + worked example + Known limitations.
- **Pre-squash audit trail** (lands in row #25): `git checkout feature/mesh-printability-v0-8-pre-squash` recovers the per-commit history.
- **Example crates** (`examples/mesh/printability-*`): 8 visual demos, one per detector + orientation parametrization + cross-tech sweep + capstone showcase. Each is documented as a museum-plaque README per `feedback_museum_plaque_readmes`.

## Example

```rust
use mesh_printability::{PrinterConfig, validate_for_printing};

// Validate a mesh for FDM printing with a non-+Z build orientation
let config = PrinterConfig::fdm_default()
    .with_build_up_direction(nalgebra::Vector3::new(0.0, 1.0, 0.0));
let result = validate_for_printing(&mesh, &config)?;

if result.is_printable() {
    println!("Pass — {}", result.summary());
} else {
    for issue in &result.issues {
        println!("{:?} ({:?}): {}", issue.severity, issue.issue_type, issue.description);
    }
    // Per-detector regions carry centroids + extents + face indices for
    // downstream rendering / repair workflows
    for thin in &result.thin_walls { /* ... */ }
    for over in &result.overhangs  { /* ... */ }
    for tv   in &result.trapped_volumes { /* ... */ }
}
```
