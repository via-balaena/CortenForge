# mesh-printability v0.8 fix arc — specification

**Status**: drafting (Sections 1–5 authored; Sections 6–13 pending)
**Working doc**: deleted in the final commit of the arc per `feedback_code_speaks`. Durable narrative migrates to `docs/studies/mesh_architecture/src/50-shell-and-print.md`.

---

## §1. Overview & gap inventory

### What this arc does

mesh-printability v0.7.0 self-reports "Complete" in `COMPLETION.md` but is materially incomplete: 5 of 10 `PrintIssueType` variants are exposed publicly but never populated by `validate_for_printing`, the `OverhangRegion.angle` field stores `threshold + 10.0` (a hardcoded approximation rather than the observed maximum), all overhang faces collapse into a single region (losing spatial information), ExcessiveOverhang severity is capped at Warning so 80° overhangs on FDM never block `is_printable()`, the basic-manifold check ignores winding orientation, and the build-up direction is hardcoded `+Z` rather than configurable. This arc closes those gaps, ships v0.8.0, and re-aligns `COMPLETION.md` with reality.

### Gap inventory

| Gap | Type | Defect | Source |
|-----|------|--------|--------|
| **A** | Hygiene | `Cargo.toml` lacks `[lints] workspace = true` | Memo, confirmed in recon |
| **B** | Bug | `OverhangRegion.angle = max_overhang_angle + 10.0` (hardcoded approximate) | `validation.rs:256` |
| **C** | Stub | `ThinWall` — variant exposed, never populated | `validation.rs` (no `check_thin_walls`) |
| **D** | Bug | All overhang faces merged into ONE region; spatial information lost | `validation.rs:255–260` (recon-surfaced) |
| **E** | Bug | ExcessiveOverhang severity is Info/Warning only; never Critical regardless of angle | `validation.rs:268–272` (recon-surfaced) |
| **F** | Bug | `check_basic_manifold` ignores winding orientation; two faces traversing a shared edge in same direction are not flagged | `validation.rs:296–311` (recon-surfaced) |
| **G** | Stub | `LongBridge` — variant exposed, never populated | `validation.rs` (no `check_long_bridges`) |
| **H** | Stub | `TrappedVolume` — variant exposed, never populated | `validation.rs` (no `check_trapped_volumes`) |
| **I** | Stub | `SelfIntersecting` — variant exposed, never populated | `validation.rs` (no `check_self_intersecting`); mesh-repair has prior art |
| **J** | Stub | `SmallFeature` — variant exposed, never populated | `validation.rs` (no `check_small_features`) |
| **K** | Doc | `COMPLETION.md` claims "Complete" while every other gap in this table exists | `COMPLETION.md` |
| **L** | Architecture | Build-up direction hardcoded `+Z` across `check_overhangs` + `evaluate_orientation`; new orientation-dependent detectors (TrappedVolume, LongBridge) would inherit the hidden global assumption | `validation.rs:201` + `orientation.rs:191` (recon-surfaced) |

(The `Other` variant is intentional caller-extension catch-all per §3, not a gap.)

### Out of scope (declined-with-rationale)

Each item below is **triple-tracked** at PR-close: (1) user-memory `project_mesh_printability_gaps.md` rewritten as "v0.8 closed + v0.9 backlog with triggers"; (2) mesh book `docs/studies/mesh_architecture/src/50-shell-and-print.md` "Known limitations" subsection; (3) `CHANGELOG.md` unreleased / v0.9-candidates block. Migration happens in the second-to-last commit of the arc, before spec deletion.

- **`PrintIssueType::Other`**: Intentional catch-all for callers building their own validation extensions. Documented as such in §3 + crate-level rustdoc, not implemented (it's not a gap). _Tracking: docstring on the variant._
- **`estimated_material_volume = bbox * 0.3` and `support_volume = overhang_area * 5.0`**: rough estimates with inline comments noting the approximation. Not bugs — documented heuristics. Real fix is research scope (volumetric integration / SDF-based) and a separate arc theme. v0.8 tightens doc-strings to spell out the heuristic nature explicitly. _Tracking: triple-tracked. Trigger for v0.9 work: a downstream consumer reports validation results within 5% of truth as a requirement._
- **BVH acceleration for ThinWall + SelfIntersecting**: typical FDM meshes <10k tris run in <1s. _Tracking: triple-tracked. Trigger for v0.9 work: a real mesh exceeds 10k tris and validation runtime exceeds 5s on the project's reference machine._
- **Tunable `IntersectionParams` from within `validate_for_printing`**: power users can call `mesh_repair::detect_self_intersections` directly via the re-exported types. _Tracking: triple-tracked. Trigger for v0.9 work: a caller asks for `validate_for_printing(mesh, config, intersection_params)` overload._

### Scope cap

If Gap A (workspace lints) surfaces ≥10 fallout sites in src, stop and raise — bundling a major src-cleanup with the lints flip violates `feedback_pr_size_ci_economics`'s commit-segmentation principle. The cleanup gets its own session.

---

## §2. Pre-flight audit

### Audit scope

Read all six src files (`config.rs`, `error.rs`, `issues.rs`, `orientation.rs`, `regions.rs`, `validation.rs`) plus `Cargo.toml`, `COMPLETION.md`, `lib.rs`. TODO/FIXME sweep across src. Workspace-wide consumer enumeration. mesh-repair cycle check. Existing-detector correctness review.

### Findings — summary

| ID | Finding | In/Out of arc | Action |
|----|---------|---------------|--------|
| F1 | No TODO/FIXME/XXX/HACK markers in src | clean | document baseline |
| F2 | mesh-repair does NOT depend on mesh-printability | safe | add mesh-repair as workspace dep |
| F3 | `COMPLETION.md` self-grade is misleading | in (Gap K) | rewrite to reflect v0.8 truth |
| F4 | Overhang multi-region collapse | in (Gap D) | new commit |
| F5 | Overhang severity policy too lenient | in (Gap E) | new commit + cross-cutting policy in §4 |
| F6 | `IssueSeverity::Info` exists but is rarely used | inform §4 policy | clarify when Info is correct |
| F7 | `PrintIssueType` test only asserts `as_str()` for 3 of 10 variants | minor coverage gap | add tests in detector commits |
| F8 | `check_overhangs` uses `+Z up` hardcode (vector `(0,0,1)`); `evaluate_orientation` does likewise | in (Gap L) | parametrize via `PrinterConfig::build_up_direction` (default `+Z`) and propagate |
| F9 | `find_optimal_orientation` uses `support_volume = overhang_area * 5.0` heuristic | document | doc-comment; not in arc |
| F10 | `find_optimal_orientation::generate_sample_orientations` returns up to `samples` items via Fibonacci-sphere fill | inform examples | use for `printability-orientation` example |
| F11 | Inconsistent winding orientation (Gap F) | in (Gap F) | new commit |
| F12 | `IntersectionParams` has `default()/exhaustive()/quick_check()` constructors | re-use | wire `quick_check()` into `validate_for_printing`; expose param choice |
| F13 | Adding mesh-repair pulls in transitive deps `rayon`, `tracing`, `cf-geometry` | acceptable | compile-only, no API leak; document in §3 |
| F14 | All `PrintValidation` fields are `pub` (struct-init) | confirms 0.8.0 minor | note in §3 API diff |

### Workspace-wide consumer enumeration

mesh-printability is consumed by:

- **Workspace root** `Cargo.toml` (workspace member declaration)
- **`mesh/`** umbrella crate — re-exports via `lib.rs`
- **`mesh/tests/api_regression.rs`** — API surface regression test
- **`examples/fundamentals/mesh/mesh-pipeline`** — first consumer
- **`examples/integration/design-to-print`** — second consumer
- **`examples/integration/full-pipeline`** — third consumer

All three examples + the umbrella + the regression test must continue to compile after the v0.8 changes. We will:
1. Audit each consumer's usage during commit 2 (the lints flip) since clippy on `--workspace` will surface any breakage.
2. If new fields on `PrintValidation` cause struct-update-syntax breakage, fix at the call site (no examples appear to use struct-update based on a quick scan; will verify before commit 9 — the version bump).

### COMPLETION.md re-alignment

Current claims that are wrong:
- ❌ "Status: Complete" — was always misleading; arc fixes this
- ❌ "Public API: ThinWallRegion ... OverhangRegion" without noting ThinWall is never populated — deceptive
- ❌ "Test Coverage: All public items have examples" in doc tests — unverified; v0.8 will verify with `cargo test --doc -p mesh-printability`
- ❌ "Quality: Zero clippy warnings" — true only against own lints, not workspace lints

v0.8 `COMPLETION.md` rewrite:
- Status: Production-ready for FDM/SLA/SLS/MJF print validation
- Detector inventory table (which `PrintIssueType` variants are populated, with severity ranges)
- Workspace lints: inherited (post-Gap A)
- Test counts: actual numbers post-v0.8
- Known limitations: BVH absent for ThinWall/SelfIntersecting (>10k tri perf cliff); tunable `IntersectionParams` from within `validate_for_printing` deferred to v0.9

### A-grade baseline

Per `feedback_grading_rubric` ("A-across-the-board or fix-and-regrade before execution"), I will:
1. Run `cargo xtask grade mesh-printability --skip-coverage` BEFORE commit 1 to capture v0.7.0 baseline.
2. Run it AFTER each commit to confirm no regression.
3. Run with coverage at the end of the arc to confirm A-grade lift (target: maintain or improve current coverage %).

If any commit drops a criterion below A, stop and fix before proceeding.

---

## §3. API surface diff — v0.7.0 → v0.8.0

### New public types in `regions.rs`

```rust
/// Information about a long bridge region (FDM-specific).
pub struct LongBridgeRegion {
    pub start: Point3<f64>,    // Bridge endpoint A
    pub end: Point3<f64>,      // Bridge endpoint B
    pub span: f64,             // mm; distance from start to end
    pub edges: Vec<(u32, u32)>, // boundary edge vertex-index pairs forming the bridge
}

/// Information about a trapped interior volume (SLA/SLS uncured-material trap).
pub struct TrappedVolumeRegion {
    pub center: Point3<f64>,        // Centroid of trapped void
    pub volume: f64,                // mm³ (estimated from voxel count × voxel volume)
    pub bounding_box: (Point3<f64>, Point3<f64>),
    pub voxel_count: u32,           // number of unreached voxels in the flood-fill grid
}

/// Information about a small isolated feature.
pub struct SmallFeatureRegion {
    pub center: Point3<f64>,        // Component centroid
    pub max_extent: f64,            // mm; longest bounding-box side
    pub volume: f64,                // mm³ (signed, via divergence theorem on closed component)
    pub face_count: u32,
    pub faces: Vec<u32>,
}

/// Information about a self-intersection between two faces.
pub struct SelfIntersectingRegion {
    pub face_a: u32,
    pub face_b: u32,
    pub approximate_location: Point3<f64>, // Midpoint of the two face centroids
}
```

Each gets `new()` + builder methods consistent with the existing `ThinWallRegion`/`OverhangRegion`/`SupportRegion` pattern.

### New fields on `PrintValidation` (validation.rs)

```rust
pub struct PrintValidation {
    // existing
    pub config: PrinterConfig,
    pub issues: Vec<PrintIssue>,
    pub thin_walls: Vec<ThinWallRegion>,
    pub overhangs: Vec<OverhangRegion>,
    pub support_regions: Vec<SupportRegion>,
    pub estimated_print_time: Option<f64>,
    pub estimated_material_volume: Option<f64>,

    // NEW in v0.8
    pub long_bridges: Vec<LongBridgeRegion>,
    pub trapped_volumes: Vec<TrappedVolumeRegion>,
    pub small_features: Vec<SmallFeatureRegion>,
    pub self_intersecting: Vec<SelfIntersectingRegion>,
}
```

`PrintValidation::new()` initializes all new fields to `Vec::new()` — non-breaking for callers using the constructor.

### New field + builder on `PrinterConfig` (config.rs)

```rust
pub struct PrinterConfig {
    // existing fields ...

    /// Build orientation: which direction is "up" relative to the build plate.
    /// Defaults to +Z (`Vector3::new(0.0, 0.0, 1.0)`) for all `*_default()` constructors.
    /// Used by overhang detection, trapped-volume detection, long-bridge detection,
    /// and orientation analysis. The vector is normalized internally.
    pub build_up_direction: Vector3<f64>,
}

// New builder method
impl PrinterConfig {
    #[must_use]
    pub fn with_build_up_direction(mut self, up: Vector3<f64>) -> Self {
        self.build_up_direction = up.normalize();
        self
    }
}
```

All four technology constructors (`fdm_default`, `sla_default`, `sls_default`, `mjf_default`) populate the field with `Vector3::new(0.0, 0.0, 1.0)`.

### New dependency

```toml
[dependencies]
mesh-repair = { path = "../mesh-repair" }   # for SelfIntersection detection re-use
```

Transitive deps pulled in: `rayon` (parallel iterators in `detect_self_intersections`), `tracing` (debug logs), `cf-geometry` (`Aabb` for bounding-box culling). All compile-only — no API leak from mesh-printability's public surface.

### New re-exports in `lib.rs`

```rust
pub use regions::{
    LongBridgeRegion, OverhangRegion, SelfIntersectingRegion, SmallFeatureRegion,
    SupportRegion, ThinWallRegion, TrappedVolumeRegion,
};

// NEW: re-export mesh-repair's intersection types so callers don't need a direct dep
pub use mesh_repair::intersect::{IntersectionParams, SelfIntersectionResult};
```

The intersection-types re-export keeps mesh-printability self-contained as the printability API. Callers wanting to interpret the `SelfIntersectionResult` from internal use don't need to add mesh-repair as their own dep.

### Internal detector behavior change — IntersectionParams

`validate_for_printing` calls `detect_self_intersections` internally with `IntersectionParams::quick_check()` (`max_reported = 1, epsilon = 1e-10, skip_adjacent = true`). Power users who need exhaustive enumeration can call `detect_self_intersections` directly via the re-exported types. v0.9 followup if a real use case for tunable params from within `validate_for_printing` emerges.

### New `PrintIssueType` variant

```rust
pub enum PrintIssueType {
    // existing 10 variants ...

    /// Emitted by mesh-printability when a detector's preconditions
    /// (e.g. watertight, consistent winding) are not met.
    /// Severity is always Info; description names the detector + missing precondition.
    /// Callers can filter validation.issues for this variant to enumerate which detectors ran.
    DetectorSkipped,
}
```

`as_str()` → `"Detector Skipped"`. Distinct from `Other` (caller-extension hook that mesh-printability never emits).

### Removed / deprecated

Nothing. v0.8 is purely additive at the type level + behaviorally augmented.

### Behavioral change — semver-significant

ExcessiveOverhang severity policy (Gap E) tightens: a face-normal exceeding `max_overhang_angle + 30°` is now Critical. Callers asserting `result.is_printable()` against meshes with severe-but-previously-uncritical overhangs will see `false` post-v0.8. **This is a correctness fix** — the v0.7 behavior was wrong (printer would fail to print the overhang while the validator said "fine"). Documented in `CHANGELOG.md` (added in this arc) and `COMPLETION.md` rewrite.

### Version bump rationale

| Change | semver-strict | practical |
|--------|---------------|-----------|
| Pub field additions on `PrintValidation` | breaking (struct-update syntax) | non-breaking (no caller uses struct-update; verified across all 6 consumers) |
| Pub field addition on `PrinterConfig` | breaking | non-breaking (same; no struct-update) |
| New types added | additive | additive |
| New re-exports | additive | additive |
| Behavioral severity change (Gap E) | behavioral | breaking for callers asserting `is_printable()` on severe overhangs — but a correctness fix |
| New dep (mesh-repair) | additive | additive |

**Verdict: 0.8.0 (minor in 0.x).** Strict-semver-breaking via field additions; pragmatically aligned via constructor convention. The behavioral correctness fix on overhang severity is exactly what minor bumps in 0.x are for.

### Mesh umbrella version

`mesh/Cargo.toml` uses `version.workspace = true` (workspace-package version `0.7.0`). The umbrella re-exports mesh-printability as `mesh::printability`, so its surface technically expanded. Two options:

1. **Keep umbrella at workspace 0.7.0**: Pragmatic since path-deps mean no consumer of the umbrella sees a version mismatch; only matters if mesh ships to crates.io.
2. **Switch umbrella to its own version, bump to 0.8.0**: Strict semver hygiene; signals to external consumers that the umbrella's API surface grew.

Flagged for §11 (Open questions). My recommendation: **option 1** for v0.8 (umbrella isn't published yet per repo state), revisit when umbrella-publishing happens.

### Doc-test impact

Existing doc-tests use `PrinterConfig::fdm_default()` and `validate_for_printing(&mesh, &config)`. New fields with defaults preserve all existing doc-tests. Each new detector commit adds a doc-test on the public `check_*` function (or the populated region type) demonstrating typical use.

### Pre/post-arc public surface diff (count)

| Surface element | v0.7.0 | v0.8.0 | delta |
|-----------------|--------|--------|-------|
| Public types | 13 | 17 + 2 re-exports | +6 |
| `PrintValidation` fields | 7 | 11 | +4 |
| `PrinterConfig` fields | 8 | 9 | +1 |
| `PrinterConfig` builder methods | 3 | 4 | +1 |
| Detectors invoked by `validate_for_printing` | 3 | 9 | +6 |
| `PrintIssueType` variants total | 10 | 11 | +1 (`DetectorSkipped`) |
| `PrintIssueType` variants emitted by platform | 4 | 10 (`Other` remains caller-only) | +6 |

---

## §4. Cross-cutting policies

These policies apply to all detectors uniformly. They live in this section (above the per-detector specs in §6) to enforce consistency and prevent drift.

### §4.1 Input mesh contract — gracefully-skip when not met

Each detector documents its required input properties. When required properties are missing, the detector **does not run and emits one Info-level diagnostic** rather than running and producing junk results.

| Detector | Requires | Behavior when missing |
|----------|----------|------------------------|
| `check_build_volume` | non-empty vertices | tolerant |
| `check_overhangs` | face normals (computed inline) | tolerant |
| `check_basic_manifold` | faces | tolerant |
| `check_thin_walls` | watertight + consistent winding | skip + DetectorSkipped diagnostic ("ThinWall detection requires watertight mesh with consistent winding") |
| `check_long_bridges` | technology requires supports (FDM/SLA/Other) | skip silently for SLS/MJF; tolerant of non-watertight |
| `check_trapped_volumes` | watertight | skip + DetectorSkipped diagnostic ("TrappedVolume detection requires watertight mesh") |
| `check_self_intersecting` | none | tolerant |
| `check_small_features` | none | tolerant |

The skip diagnostic uses a **new `PrintIssueType::DetectorSkipped` variant** — explicit, type-level differentiation from caller-extension `Other`. This keeps `Other`'s contract as "platform never emits; callers may use freely."

**Implementation pattern**: detectors first check preconditions; on failure they push a single Info-level `PrintIssue` with `issue_type = DetectorSkipped` and a description naming the detector + the missing precondition, then return early. They do not push to the typed-region field. Callers can filter via `result.issues.iter().filter(|i| matches!(i.issue_type, PrintIssueType::DetectorSkipped))` to enumerate which detectors ran.

### §4.2 Numerical tolerance policy

| Tolerance | Value | Where used |
|-----------|-------|------------|
| `EPS_DEGENERATE_NORMAL` | `1e-10` | filter zero-area triangles; existing code carry-over |
| `EPS_RAY_OFFSET` | `1e-6 mm` | starting offset for ThinWall ray-cast (avoids self-hit) |
| `EPS_GEOMETRIC` | `1e-9 mm` | general geometric tie-breaking (point-on-plane, ray-tri co-planarity) |
| `EPS_INTERSECTION` | `1e-10` | mesh-repair's `IntersectionParams::epsilon`; reused as-is |
| `ANGLE_TOL_VERTICAL` | `30°` | LongBridge "near-vertical support wall" classification (wall normal within 30° of horizontal) |
| `ANGLE_TOL_HORIZONTAL` | `30°` | LongBridge "near-horizontal bridge face" classification (face normal within 30° of −up) |

All FP comparisons are **absolute in mm units** (not relative). Tolerances are declared as `const` at the top of the relevant source file with the unit in their doc-comment, not as magic numbers inline.

Voxel sizing for `check_trapped_volumes` is **derived, not hardcoded**: `voxel_size = (config.min_feature_size.min(config.layer_height)) / 2.0`. Documented in the function's doc-comment.

### §4.3 Severity assignment policy

Three-level scale. Definitions:

- **Info**: cosmetic or borderline; print will succeed without user action.
- **Warning**: print may succeed but with significant defects; user should review.
- **Critical**: print will fail or have unacceptable defects; `is_printable()` returns `false`.

Per-detector mapping:

| Detector | Condition | Severity |
|----------|-----------|----------|
| ExceedsBuildVolume | always | Critical |
| NotWatertight | always | Critical |
| NonManifold (open edge) | always | Critical |
| NonManifold (>2 face share / inconsistent winding) | always | Critical |
| ExcessiveOverhang | angle > threshold + 30° | **Critical** _(Gap E fix)_ |
| ExcessiveOverhang | threshold + 15° < angle ≤ threshold + 30° | Warning |
| ExcessiveOverhang | threshold < angle ≤ threshold + 15° | Info |
| ThinWall | thickness < min_wall_thickness / 2 | Critical |
| ThinWall | min_wall_thickness / 2 ≤ thickness < min_wall_thickness | Warning |
| LongBridge | span > max_bridge_span × 1.5 | Critical |
| LongBridge | max_bridge_span < span ≤ max_bridge_span × 1.5 | Warning |
| TrappedVolume | technology ∈ {SLA, SLS, MJF} | Critical (uncured-material trap) |
| TrappedVolume | technology = FDM | Info (sealed cavity prints fine; surfaced for awareness in case it's unintentional) |
| TrappedVolume | trapped volume < min_feature_size³ | Info (likely intentional interior; below resolution) |
| SelfIntersecting | always | Critical |
| SmallFeature | extent < min_feature_size / 2 | Warning |
| SmallFeature | min_feature_size / 2 ≤ extent < min_feature_size | Info |
| DetectorSkipped | always | Info |

The mapping is implemented as a single helper `fn classify_severity(detector: DetectorKind, ...) -> IssueSeverity` in `validation.rs` so the policy lives in one place rather than scattered across each `check_*` function.

### §4.4 Determinism and output ordering

mesh-repair's `detect_self_intersections` uses rayon; pair order in `intersecting_pairs` is non-deterministic across runs. We sort all detector outputs deterministically before storing in `PrintValidation`:

| Field | Sort key |
|-------|----------|
| `issues` | by `(severity descending, issue_type, location.x, location.y, location.z, affected_elements[0])` |
| `thin_walls` | by face indices (smallest face index first) |
| `overhangs` | by face indices |
| `support_regions` | by face indices |
| `long_bridges` | by `(start.x, start.y, start.z)` |
| `trapped_volumes` | by `(center.x, center.y, center.z)` |
| `small_features` | by face indices |
| `self_intersecting` | by `(face_a, face_b)` |

Sort happens at the end of `validate_for_printing`, not per-detector. Stable ordering is asserted by tests.

### §4.5 Cross-platform FP stability

The Phase 4 multi-material work surfaced cross-platform FP drift in faer's block-diagonal LDLᵀ — bit-equal assertions on linear-system outputs broke between macOS and Linux. mesh-printability does not invoke faer, but ray-cast distances, voxel-fill counts, and signed-volume integrals still carry FP drift risk on cross-platform.

Mitigations:

1. **Tolerance-based assertions in tests** — use `approx::assert_relative_eq!` with `epsilon = 1e-6` and `max_relative = 1e-9`, never `f64::EPSILON` for derived geometric quantities.
2. **Exact-representable test fixtures** — geometric inputs use values like `1.0`, `2.0`, `0.5` (FP-exact) when possible; avoid `0.1` and similar non-exact decimals.
3. **Platform smoke tests** — covered by existing CI matrix (macOS + Linux + Windows). Detector tests included; failures on one platform fail the gate.
4. **Document fragility** — any detector whose output can shift by FP drift gets a doc-comment noting "results may vary by ULP across platforms; use absolute-tolerance comparison".

### §4.6 Logging — no tracing in v0.8

mesh-printability is not currently instrumented with tracing; mesh-repair is. v0.8 keeps mesh-printability tracing-free. Detector implementations do not add log statements; users get diagnostics through `PrintValidation.issues` and the `summary()` string.

Tracing from `mesh_repair::detect_self_intersections` flows through transparently when a caller initializes a tracing subscriber, but mesh-printability does not introduce its own log instrumentation. v0.9 followup if structured-diagnostics demand emerges (triple-tracked when this becomes load-bearing).

---

## §5. Per-fix specs (non-detector commits)

This section covers in-place fixes to existing code paths and the infrastructure changes (lints, docs). New detectors are in §6.

### §5.1 Gap A — Workspace lints inheritance

**Change**: Add `[lints] workspace = true` block to `mesh/mesh-printability/Cargo.toml` after `[package.metadata.cortenforge]`.

**Procedure**:
1. Edit Cargo.toml.
2. Run `cargo clippy -p mesh-printability --tests --all-targets -- -D warnings`.
3. Triage fallout. Classify each warning:
   - **In-arc fix**: simple unwrap/expect/panic that has an obvious replacement (e.g. `.expect()` in a `_default()` constructor).
   - **Stop-and-raise**: ≥10 fallout sites, or any single site requiring >20 LOC of refactor.
4. If fallout is light, fix in the same commit as the lints flip.
5. If fallout is heavy, **stop, revert the lints flip, and raise** — bundling a major src-cleanup with the lints flip violates the commit-segmentation principle.

**Acceptance criteria**:
- `cargo clippy -p mesh-printability --tests --all-targets -- -D warnings` passes.
- `cargo xtask grade mesh-printability --skip-coverage` baseline preserved.
- Commit message names the count of fallout sites fixed (e.g. "fix(mesh-printability): inherit workspace lints (3 unwrap fixes in tests)").

**Tests**: no new unit tests; the gate is the clippy run.

### §5.2 Gap B — Track actual maximum overhang angle

**Change**: `validation.rs::check_overhangs` currently sets `OverhangRegion.angle = config.max_overhang_angle + 10.0` (line 256, hardcoded approximate). Replace with the actual maximum overhang angle observed across flagged faces.

**Code shape**:
```rust
let mut max_overhang_angle_rad: f64 = 0.0;
for i in 0..num_triangles {
    // ... existing face-iteration ...
    if dot < 0.0 {
        let overhang_angle = std::f64::consts::PI - angle;
        if overhang_angle > max_angle_rad {
            overhang_faces.push(i as u32);
            max_overhang_angle_rad = max_overhang_angle_rad.max(overhang_angle);
            total_overhang_area += area;
        }
    }
}
// ...
let region = OverhangRegion::new(center, max_overhang_angle_rad.to_degrees(), total_overhang_area);
```

**Tests**:
- `test_overhang_angle_tracks_max`: construct an inverted pyramid (steep face angle computable analytically); assert `OverhangRegion.angle` is within `1e-6` of the analytical value.
- `test_overhang_angle_uses_steepest_face`: mesh with two overhanging faces at 50° and 70° (both > 45° threshold); assert reported angle is ~70°.
- `test_overhang_no_overhang_no_region`: mesh with no overhangs (vertical walls); assert no overhang region.

**Acceptance criteria**:
- Three tests pass with `approx::assert_relative_eq!(angle, expected, epsilon = 1e-6)`.
- No change to the count or face-membership of overhang regions (split into connected regions is §5.3).

### §5.3 Gap D — Split overhangs into connected regions

**Change**: `check_overhangs` currently lumps ALL overhanging faces into ONE `OverhangRegion` (only `overhang_faces[0]`'s centroid is used as the region center). Split into one region per connected overhang patch using face-adjacency.

**Algorithm**:
1. Collect all overhanging face indices into `overhang_faces` (existing logic, retained).
2. Build face-adjacency: two overhanging faces are adjacent if they share an edge.
3. Run union-find (or BFS/DFS) to partition `overhang_faces` into connected components.
4. For each component, compute centroid (average of face centroids), max overhang angle (from §5.2), summed area; emit one `OverhangRegion`.

**Implementation note**: face-adjacency is computed from the edge map already built by `check_basic_manifold`. To avoid re-computing, we extract edge-map building into a shared helper:

```rust
fn build_edge_to_faces(mesh: &IndexedMesh) -> HashMap<(u32, u32), Vec<u32>>
```

Keys are undirected edges (smaller vertex index first); values are the list of face indices incident on the edge. Used by both `check_basic_manifold` and `check_overhangs` (for region splitting). This is a refactor — happens as a **separate commit before Gap D** so the diff is reviewable. No new dep (uses existing `hashbrown::HashMap` + `Vec`).

**Refactor regression gate** (in the refactor commit, not Gap D's commit): the existing `test_not_watertight_detection`, `test_watertight_mesh`, `test_validation_summary`, `test_issue_counts`, and `test_sls_no_overhang_check` continue to pass without modification. These functionally cover `check_basic_manifold`'s pre-refactor outputs (open-edge counts, non-manifold counts, the resulting summary string). If any test changes behavior, the refactor changed semantics and must be fixed before merging the commit.

**Tests**:
- `test_overhang_single_connected_region`: ramp mesh with one overhanging surface; assert exactly 1 `OverhangRegion`.
- `test_overhang_two_disjoint_regions`: mesh with two overhang patches separated by vertical walls; assert exactly 2 `OverhangRegion`s with face-disjoint membership.
- `test_overhang_region_centroid_is_component_centroid`: place a single triangular overhang at known position; assert centroid is within `1e-6` of triangle centroid.
- `test_overhang_no_overhangs`: cube on build plate; assert no regions.
- `test_overhang_face_adjacency_via_shared_edge`: two faces sharing an edge are in same component; two faces sharing only a vertex are in different components (matches manifold-edge contract).

**Acceptance criteria**:
- Five tests pass.
- `validation.support_regions` length matches `validation.overhangs` length (one support region per overhang patch — preserves existing 1:1 invariant, just at component granularity now).
- `total_support_volume()` value preserved on test fixtures (sum of per-region volumes equals total).

### §5.4 Gap E — Tighten ExcessiveOverhang severity policy

**Change**: replace the area-based severity (>1000mm² → Warning, else → Info) with the §4.3 angle-based mapping:
- `angle > threshold + 30°` → Critical
- `threshold + 15° < angle ≤ threshold + 30°` → Warning
- `threshold < angle ≤ threshold + 15°` → Info

**Code shape**: introduce the central helper `classify_overhang_severity(observed_angle_deg, threshold_deg) -> IssueSeverity` in `validation.rs`. Each region uses this helper to set its issue severity. The helper is the §4.3 policy in code.

**Tests**:
- `test_overhang_severity_critical_at_steep`: 80° face on 45° threshold → Critical.
- `test_overhang_severity_warning_at_medium`: 65° face on 45° threshold → Warning.
- `test_overhang_severity_info_at_borderline`: 50° face on 45° threshold → Info.
- `test_is_printable_blocks_critical_overhang`: 80° face on 45° threshold → `validation.is_printable() == false` (Gap E primary regression).
- `test_is_printable_allows_borderline_overhang`: 50° face on 45° threshold → `validation.is_printable() == true`.

**Acceptance criteria**:
- Five tests pass.
- Existing test `test_sls_no_overhang_check` still passes (overhangs not checked for technologies that don't require supports — gating is at the function level, not severity level).

### §5.5 Gap F — Detect inconsistent winding orientation in manifold check

**Change**: `check_basic_manifold` currently treats edges as unordered `(min, max)` pairs and only counts sharing. Add directed-edge tracking: for each face, record both `(from, to)` traversals; if any directed pair appears more than once, two faces traverse the edge in the same direction (orientation defect).

**Code shape**:
```rust
let mut directed_edges: HashMap<(u32, u32), u32> = HashMap::new();
for face in &mesh.faces {
    let edges = [
        (face[0], face[1]),
        (face[1], face[2]),
        (face[2], face[0]),
    ];
    for edge in &edges {
        *directed_edges.entry(*edge).or_insert(0) += 1;
    }
}
let inconsistent_count = directed_edges.values().filter(|&&c| c > 1).count();
if inconsistent_count > 0 {
    let issue = PrintIssue::new(
        PrintIssueType::NonManifold,
        IssueSeverity::Critical,
        format!("{inconsistent_count} edge(s) traversed in same direction by multiple faces (winding inconsistency)"),
    );
    validation.issues.push(issue);
}
```

The undirected edge-count (existing logic) and the directed edge-count run side-by-side; both push under `PrintIssueType::NonManifold` with distinct descriptions so callers can tell them apart.

**Tests**:
- `test_winding_inconsistent_two_same_direction_faces`: two triangles sharing an edge, both traversing the same direction → `NonManifold` Critical issue with description containing "winding inconsistency".
- `test_winding_consistent_watertight_cube`: existing `create_watertight_cube` fixture → no winding-inconsistency issue (this is the regression test that the existing winding is correct).
- `test_winding_consistent_disjoint_faces`: two triangles sharing only a vertex → no winding-inconsistency issue.
- `test_winding_inconsistent_does_not_break_open_edge_check`: incomplete cube with consistent winding → reports `NotWatertight` but no winding issue.

**Acceptance criteria**:
- Four tests pass.
- Existing `test_not_watertight_detection` and `test_watertight_mesh` still pass (no regression).

### §5.6 Gap L — `PrinterConfig::build_up_direction` parametrization

**Change**: add `build_up_direction: Vector3<f64>` field + `with_build_up_direction(up: Vector3<f64>) -> Self` builder. Default to `Vector3::new(0.0, 0.0, 1.0)` in all four technology `_default()` constructors. Propagate through `check_overhangs` and `evaluate_orientation` (both currently hardcode `(0,0,1)`).

**Code shape** (config.rs):
```rust
pub struct PrinterConfig {
    // existing fields ...
    pub build_up_direction: Vector3<f64>,
}

impl PrinterConfig {
    #[must_use]
    pub fn fdm_default() -> Self {
        Self {
            // ... existing fields ...
            build_up_direction: Vector3::new(0.0, 0.0, 1.0),
        }
    }
    // sla_default, sls_default, mjf_default similar

    /// Set the build-up direction. The vector is normalized internally.
    ///
    /// # Panics
    ///
    /// Panics in debug builds if `up` is the zero vector. In release builds,
    /// NaN values propagate per nalgebra's normalization contract; callers
    /// are responsible for passing a non-zero vector.
    #[must_use]
    pub fn with_build_up_direction(mut self, up: Vector3<f64>) -> Self {
        debug_assert!(up.norm() > EPS_DEGENERATE_NORMAL, "build_up_direction must be non-zero");
        self.build_up_direction = up.normalize();
        self
    }
}
```

**Propagation**:
- `check_overhangs`: replace `let up = Vector3::new(0.0, 0.0, 1.0);` with `let up = config.build_up_direction;`.
- `evaluate_orientation`: replace `let up = Vector3::new(0.0, 0.0, 1.0);` with `let up = config.build_up_direction;`.
- `find_optimal_orientation`: passes config through; rotation is applied in mesh-frame, so the rotated normals are dotted against `config.build_up_direction` (whatever frame the caller chose).

**Tests**:
- `test_build_up_direction_default_is_z`: `PrinterConfig::fdm_default().build_up_direction == (0,0,1)`.
- `test_build_up_direction_normalized_in_builder`: `with_build_up_direction(Vector3::new(0.0, 0.0, 5.0))` results in `(0,0,1)`.
- `test_build_up_direction_zero_panics_in_debug`: `with_build_up_direction(Vector3::zeros())` panics in debug builds (use `#[should_panic]` gated on `cfg(debug_assertions)`).
- `test_overhang_with_y_up_orientation`: same overhanging mesh validates correctly with `+Z up` and (after 90° rotation around X) with `+Y up`. Symmetric overhang counts.
- `test_overhang_with_oblique_up`: a 45°-from-vertical up vector is normalized and used; overhanging-face count is consistent with manual rotation.
- `test_find_optimal_orientation_respects_build_up_direction`: same mesh, `+Z up` config and `+Y up` config (with appropriately rotated mesh), search returns matching `support_volume` and `overhang_area` values within FP tolerance.

**Acceptance criteria**:
- Four tests pass.
- All existing tests pass without modification (defaults preserve `+Z` behavior).
- Detectors authored in §6 (TrappedVolume, LongBridge) consume `config.build_up_direction` from day one — never the hardcoded `(0,0,1)`.

### §5.7 Gap K — `COMPLETION.md` rewrite

**Change**: rewrite `mesh/mesh-printability/COMPLETION.md` to reflect v0.8 truth.

**New content** (sketch):
```markdown
# mesh-printability Completion

## Status: Production-ready for FDM/SLA/SLS/MJF print validation (v0.8.0)

## Detector inventory

| PrintIssueType | Populated by | Severity range |
|----------------|--------------|----------------|
| ExceedsBuildVolume | check_build_volume | Critical |
| ExcessiveOverhang | check_overhangs | Info / Warning / Critical |
| NotWatertight | check_basic_manifold | Critical |
| NonManifold | check_basic_manifold | Critical |
| ThinWall | check_thin_walls | Warning / Critical |
| LongBridge | check_long_bridges | Warning / Critical |
| TrappedVolume | check_trapped_volumes | Info / Critical |
| SelfIntersecting | check_self_intersecting | Critical |
| SmallFeature | check_small_features | Info / Warning |
| DetectorSkipped | (any detector when preconditions unmet) | Info |
| Other | caller-extension; mesh-printability never emits | (caller-defined) |

## Public API
... (regenerated from current state) ...

## Test Coverage
- Unit tests: <count> in src/
- Doc tests: all public items have examples; verified via `cargo test --doc -p mesh-printability`

## Quality
- Workspace lints inherited (post Gap A)
- Zero clippy warnings on `--workspace -- -D warnings`
- Zero rustdoc warnings on `RUSTDOCFLAGS=-D warnings cargo doc --no-deps -p mesh-printability`
- No unwrap/expect in library code

## Known limitations (tracked for v0.9+)
- BVH absent for ThinWall + SelfIntersecting (<10k tris OK; >10k tris perf cliff)
- Tunable IntersectionParams from within validate_for_printing (use mesh-repair directly)
- Heuristic estimates for support_volume + estimated_material_volume

## Dependencies
- mesh-types (workspace)
- mesh-repair (workspace) — for SelfIntersection re-use
- nalgebra (workspace)
- thiserror (workspace)
- hashbrown (workspace)
```

**Acceptance criteria**:
- `COMPLETION.md` accurately reflects post-§5/§6 state.
- All claims verifiable: every public-API listing matches `pub use` in lib.rs; every detector listed populates its claimed type/severity in tests.

### §5.8 Add `PrintIssueType::DetectorSkipped` variant

**Change**: add the `DetectorSkipped` variant to `PrintIssueType` (issues.rs) + extend `as_str()` mapping. Prerequisite commit for any detector with precondition-skip behavior (ThinWall, TrappedVolume per §4.1). Lands as its own commit before the first detector commit.

**Code shape** (issues.rs):
```rust
pub enum PrintIssueType {
    // existing 10 variants, unchanged ...

    /// Emitted when a detector's preconditions (e.g. watertight mesh,
    /// consistent winding) are not met. Severity is always Info.
    /// The issue's description names the detector + missing precondition.
    /// Distinct from `Other`, which is a caller-extension hook that
    /// mesh-printability never emits.
    DetectorSkipped,
}

impl PrintIssueType {
    pub fn as_str(&self) -> &'static str {
        match self {
            // existing arms ...
            Self::DetectorSkipped => "Detector Skipped",
        }
    }
}
```

**Tests**:
- `test_detector_skipped_as_str`: `PrintIssueType::DetectorSkipped.as_str() == "Detector Skipped"`.
- `test_detector_skipped_is_info_severity_when_emitted`: construct a `PrintIssue::new(DetectorSkipped, Info, "...")`; assert `is_warning() == false && is_critical() == false`.

**Acceptance criteria**:
- Two tests pass.
- All existing `issues.rs` tests continue to pass.
- Match arm exhaustiveness: any external code matching on `PrintIssueType` (within the workspace) gets a compile error if it lacks a `DetectorSkipped` arm — verified via `cargo build --workspace` after the variant adds.

### §5.9 CHANGELOG.md creation + per-commit updates

**Change**: create `mesh/mesh-printability/CHANGELOG.md` in the same commit as Gap A (workspace lints). Each subsequent commit appends to the `[Unreleased]` section. The final pre-spec-deletion commit closes `[Unreleased]` → `[0.8.0] - YYYY-MM-DD`.

**Initial structure**:
```markdown
# Changelog

All notable changes to mesh-printability will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- ...

### Changed
- ...

### Fixed
- ...

## [0.7.0] - 2026-XX-XX
- Initial release with build-volume, overhang, and manifold detectors.
```

**Acceptance criteria**:
- File exists at `mesh/mesh-printability/CHANGELOG.md` after commit 1.
- Each subsequent commit's diff includes a CHANGELOG.md update entry.
- Final commit pins the release date and version under the `[0.8.0]` section.

---

## §6 onwards — pending

(Sections 6–13: per-detector specs, example design, risk inventory, stress-test gauntlet, grading & CI impact, open questions, implementation order, spec lifecycle.)
