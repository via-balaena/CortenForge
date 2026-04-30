# mesh-printability v0.8 fix arc — specification

**Status**: drafting (Sections 1–7 authored; Sections 8–13 pending)
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
| **M** | Bug | `check_overhangs` flagging predicate algebraically reduces to flag-when-`dot ∈ (-cos(max_overhang_angle), 0)` — moderately-tilted only; pure roofs (`dot = -1`) NOT flagged; convention is inverse of standard FDM "tilt from vertical exceeds threshold". M.2 sub-fix: ALSO add a build-plate filter (matching §6.2 LongBridge pattern) since the corrected predicate would otherwise flag every solid-on-plate's bottom face as Critical overhang. No existing test covers either issue. | `validation.rs:234–251` + `orientation.rs:228–238` (§7-recon-surfaced 2026-04-30) |

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
| F12 | `IntersectionParams` has `default()/exhaustive()/quick_check()` constructors | re-use | wire `default()` (max_reported = 100) into `validate_for_printing`; re-export types for power users |
| F13 | Adding mesh-repair pulls in transitive deps `rayon`, `tracing`, `cf-geometry` | acceptable | compile-only, no API leak; document in §3 |
| F14 | All `PrintValidation` fields are `pub` (struct-init) | confirms 0.8.0 minor | note in §3 API diff |
| F15 | Overhang flagging predicate inverted from FDM convention (surfaced during §7 example design) | in (Gap M) | new commit; §5.9 |

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

`validate_for_printing` calls `detect_self_intersections` internally with `IntersectionParams::default()` (`max_reported = 100, epsilon = 1e-10, skip_adjacent = true`). Reports up to 100 intersection pairs — sufficient for almost all real meshes (a mesh with >100 self-intersections is fundamentally broken and the count is informational). Power users who need exhaustive enumeration can call `detect_self_intersections` directly via the re-exported types with `IntersectionParams::exhaustive()`. v0.9 followup if a real use case for tunable params from within `validate_for_printing` emerges (already triple-tracked).

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

**Change**: Add `[lints] workspace = true` block to `mesh/mesh-printability/Cargo.toml` after `[package.metadata.cortenforge]`. Triage the 38-site fallout per the §13-locked pre-flight battery via a 3-commit split (1a/1b/1c) — each sub-commit is FP-semantics-preserving.

**Stop-and-raise gate**: the original gate was "≥10 fallout sites or any single site needing >20 LOC of refactor → revert the lints flip and raise". Pre-flight measured **38 sites — 3.8× the gate**. The arc raises this exception with a documented resolution (3-commit split below); the >10 gate stays in effect for any future Gap-A-shaped lint flip in another crate.

**Pre-flight fallout breakdown** (measured 2026-04-30 night per §13):

| Category | Count | Treatment |
|----------|-------|-----------|
| `clippy::missing_const_for_fn` | 19 | Mechanical — add `const` keyword |
| `clippy::cast_precision_loss` | 3 | Mechanical — add `#[allow]` + one-line justification |
| `clippy::uninlined_format_args` | 1 | Mechanical — inline the format arg |
| `clippy::match_same_arms` | 1 | Mechanical — collapse arms |
| `clippy::expect_used` (in tests) | 5 | Test-site — `#[allow]` or restructure to non-panicking |
| `clippy::suboptimal_flops` (mul_add) | 8 | **FP-semantics** — `#[allow]` per-site with justification |
| `clippy::manual_midpoint` | 1 | **FP-semantics** — `#[allow]` per-site with justification |
| **Total** | **38** | |

**FP-semantics rationale**: 8 `mul_add` sites (validation.rs:227, 235; orientation.rs:387; etc.) and 1 `manual_midpoint` site change overhang-detection floating-point bits. Applying these "fixes" mechanically would change which faces flag as overhang at the threshold boundary — invisible numerically but visible in any test asserting on flag-count or affected-faces lists. Bit-exactness of the post-Gap-M predicate is a separate decision; conflating it with a hygiene commit violates `feedback_baby_steps`. Per-site `#[allow]` keeps the lint local + auditable so future code added to those files still gets the warning.

**Why per-site `#[allow]` (not crate-level `#![allow]`)**: a crate-level `#![allow(clippy::suboptimal_flops)]` in `lib.rs` would silence the lint forever, including on new code added after v0.8 where FMA might be a clean win. Per-site annotation tells the next maintainer which lines deliberately preserve the non-FMA form and why. Trade-off: clippy is **red** between commits 1a and 1c — only green after 1c lands. CI doesn't run between feature-branch commits, so no CI cost; only an ergonomic cost during the 3-commit window.

**3-commit split** (locked; full per-commit detail in §12.4):

| Commit | Theme | Sites | Compiler-checkable | Clippy state |
|--------|-------|-------|---------------------|--------------|
| **1a** | Mechanical-only, no annotations | 19 const + 3 cast + 1 format + 1 match = 24 | Yes (`cargo build`) | Red (lints not yet inherited) |
| **1b** | Test-site cleanup | 5 expect_used (allow or restructure) | Yes (`cargo test`) | Red (lints not yet inherited) |
| **1c** | `[lints] workspace = true` flip + 8 mul_add allows + 1 midpoint allow | Cargo.toml + 9 `#[allow]` annotations | Yes (`cargo clippy`) | **Green** |

**Acceptance criteria** (per sub-commit):

- **1a**: `cargo build -p mesh-printability --tests --all-targets` clean; `cargo test -p mesh-printability` 35/35 pass; commit body names the 24 sites changed by category. No clippy gate (lints not yet inherited).
- **1b**: `cargo test -p mesh-printability` 35/35 pass; commit body names the 5 test sites and which got `#[allow]` vs restructure.
- **1c**: `cargo clippy -p mesh-printability --tests --all-targets -- -D warnings` passes; `cargo xtask grade mesh-printability --skip-coverage` A across 7 criteria; CHANGELOG.md created (per §5.10) with "Inherited workspace lints; 9 per-site allows for FP-semantics-preserving sites" in `[Unreleased] / Changed`. v0.7 coverage baseline (per §11.2) captured in this commit's body since it's the first commit that completes the Gap A scope.

**Justification comment template** (for each FP-semantics `#[allow]`):

```rust
// Preserved as `(a * b) + c` rather than `a.mul_add(b, c)`: FMA changes the
// final FP bit of overhang predicate evaluation, which would shift which
// faces flag at the 45° boundary on cross-platform runs. Bit-exactness of
// the overhang predicate is tracked as a v0.9 candidate; see CHANGELOG.md
// `[Unreleased]` for the open issue.
#[allow(clippy::suboptimal_flops)]
let dot = (n_x * up_x) + (n_y * up_y) + (n_z * up_z);
```

The comment references the CHANGELOG (which survives spec deletion at commit #25 per §13), not this spec section. Wording adapted per site — `manual_midpoint` site uses an analogous comment naming `(a + b) / 2.0` vs `a.midpoint(b)`. The v0.9-candidate entry must exist in §11.5's re-open trigger table at the time of commit 1c so the comment's CHANGELOG reference is honored end-to-end.

**Tests**: no new unit tests; the gate is `cargo clippy` (in 1c) + the existing 35-test suite (gates 1a + 1b + 1c).

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

### §5.9 Gap M — Fix overhang flagging predicate to FDM convention

**Change**: The v0.7.0 `check_overhangs` predicate (`validation.rs:234–251`) and its mirror in `evaluate_orientation` (`orientation.rs:228–238`) flag faces with `dot ∈ (-cos(max_overhang_angle), 0)` — moderately-tilted faces between vertical and ~135° from up. Pure-roof faces (`dot = -1`, fully downward) are NOT flagged. This is inverse from the standard FDM convention where `max_overhang_angle = 45°` means "faces tilted more than 45° from vertical (i.e., normal-from-up exceeds 90° + 45° = 135°) need support". Replace the predicate so roofs are correctly flagged.

**Convention lock-in (FDM standard)**:

We define `overhang_angle` as the **signed deviation of the face normal from the build-plane plane** (= 90° minus the face normal's angle from build-up direction):

```
overhang_angle = acos(dot) - π/2     // dot = normal · up ∈ [-1, +1]
              ∈ [-π/2, +π/2]
```

Sign convention: negative for upward-facing faces (no overhang concern), zero for vertical walls (horizontal normals), positive for downward-tilted faces (overhang). Specifically:

- Top face (normal aligned with up, `dot = +1`): `overhang_angle = 0 - π/2 = -π/2 = -90°`. Face's normal points away from "down"; not an overhang.
- Vertical wall (normal horizontal, `dot = 0`): `overhang_angle = π/2 - π/2 = 0`. Borderline — by convention not an overhang.
- 45° downward-tilted face (normal at 135° from up, `dot = -sin(45°) = -0.7071`): `overhang_angle = 3π/4 - π/2 = π/4 = 45°`.
- Roof (normal fully down, `dot = -1`): `overhang_angle = π - π/2 = π/2 = 90°`. Worst-case overhang.

**New flagging predicate**: flag if `overhang_angle > max_overhang_angle_rad`. Strict greater-than (a vertical wall has `overhang_angle = 0`, which is not greater than any positive threshold; matches the "vertical walls are fine" intent).

**Build-plate filter** (M.2): Gap M ALSO adds a build-plate filter to `check_overhangs`, matching the pattern §6.2 (LongBridge) uses. **Why**: under the new predicate, a solid object's bottom face has `dot = -1`, `overhang_angle = 90°` — flagged as Critical overhang. But a face TOUCHING the build plate is supported by the plate itself; flagging it as overhang is physically wrong and would cause every solid-on-plate fixture to fail `is_printable()`. The v0.7.0 buggy predicate accidentally avoided this case (it didn't flag `dot ≤ -0.707`); removing the bug without adding the build-plate filter would convert a correctness fix into a regression in user-visible behavior.

The filter pattern: a face is "on the build plate" iff `(face_min_along_up - mesh_min_along_up) < EPS_GEOMETRIC`. Applied per-face inside the overhang loop (after computing `overhang_angle`, before flagging). Same pattern as `check_long_bridges` (§6.2).

**Code shape** (`validation.rs::check_overhangs`):

```rust
// Compute mesh_min_along_up once before the face loop:
let mesh_min_along_up = mesh.vertices.iter()
    .map(|v| v.coords.dot(&up))
    .fold(f64::INFINITY, f64::min);

// In the face loop:
let dot = normal.x * up.x + normal.y * up.y + normal.z * up.z;
let angle_from_up = dot.acos();  // 0 to π
let overhang_angle = angle_from_up - std::f64::consts::FRAC_PI_2;  // -π/2 to +π/2

if overhang_angle > max_angle_rad {
    // Build-plate filter: skip faces resting on the build plate (supported by
    // the plate itself; not an overhang concern).
    let face_min_along_up = [v0, v1, v2].iter()
        .map(|v| v.coords.dot(&up))
        .fold(f64::INFINITY, f64::min);
    if (face_min_along_up - mesh_min_along_up) < EPS_GEOMETRIC {
        continue;  // on build plate; not an overhang
    }

    overhang_faces.push(i as u32);
    max_overhang_angle_rad = max_overhang_angle_rad.max(overhang_angle);
    let area = len / 2.0;
    total_overhang_area += area;
}
```

`EPS_GEOMETRIC = 1e-9 mm` per §4.2.

Symmetric change in `orientation.rs::evaluate_orientation` — same predicate replacement + build-plate filter (~10 LOC).

**Removal of the `dot < 0.0` outer gate**: the new predicate `overhang_angle > max_angle_rad` already excludes upward-facing faces (their `overhang_angle ≤ 0`), so the explicit `if dot < 0.0` early-out is no longer needed. Removing it simplifies the structure; performance is comparable (one fewer branch).

**Interaction with Gap B (§5.2)**: Gap B's "track actual maximum overhang angle" (`max_overhang_angle_rad.max(overhang_angle)`) reads the same `overhang_angle` that Gap M defines. After both fixes:
- `overhang_angle = 0` for vertical walls.
- `overhang_angle = 45° (in radians)` for 45°-tilted face.
- `overhang_angle = 90° (in radians)` for pure roof.
- `OverhangRegion.angle = max(observed)` in degrees.
- This matches FDM-slicer conventions.

**Interaction with Gap E (§5.4)**: Gap E's `classify_overhang_severity(observed_angle_deg, threshold_deg)` consumes `observed_angle_deg` (the post-Gap-B value). With Gap M:
- `observed_angle_deg ∈ [0, 90]`.
- Critical: `observed_angle > threshold_deg + 30` → e.g., for FDM threshold 45°, Critical when observed > 75°. Sensible (75°-tilted face is severely overhanging).
- Warning: `threshold + 15 < observed ≤ threshold + 30` → 60° < observed ≤ 75°.
- Info: `threshold < observed ≤ threshold + 15` → 45° < observed ≤ 60°.

This makes physical sense: severity scales with how far past threshold the face leans. Pre-Gap-M severity-band boundaries were physically meaningless because the predicate flagged the wrong faces.

**Interaction with Gap L (§5.6)**: parametrized `build_up_direction` flows through unchanged — `dot = normal.dot(&config.build_up_direction)` and the rest of the predicate is identical. Both fixes commute.

**Tests** (in `validation.rs::tests`):
- `test_overhang_roof_flagged`: face with normal=`(0,0,-1)` on default `+Z up`, `max_overhang_angle = 45°` → flagged with `overhang_angle ≈ 90°`.
- `test_overhang_vertical_wall_not_flagged`: face with normal=`(1,0,0)` (horizontal) → not flagged (`overhang_angle = 0`).
- `test_overhang_top_face_not_flagged`: face with normal=`(0,0,1)` (top) → not flagged (`overhang_angle = -90°`).
- `test_overhang_45deg_tilt_borderline_not_flagged`: face with `dot = -sin(45°) = -0.7071`, `max=45°` → `overhang_angle = 45°` exactly; `45 > 45` is false, NOT flagged. Document the strict-greater-than convention.
- `test_overhang_60deg_tilt_flagged`: face with `dot = -sin(60°) = -0.866`, `max=45°` → `overhang_angle ≈ 60°` > 45° → flagged.
- `test_overhang_30deg_tilt_not_flagged`: face with `dot = -sin(30°) = -0.5`, `max=45°` → `overhang_angle ≈ 30°` < 45° → not flagged.
- `test_overhang_borderline_via_y_up_orientation`: same predicate with `+Y up` config (cross-check that Gap L parametrization composes correctly with Gap M).
- `test_overhang_build_plate_face_not_flagged`: solid cube on build plate (z=0); cube bottom face has `overhang_angle = 90° > 45°` but is filtered by the build-plate check; assert `validation.overhangs.len() == 0`.
- `test_overhang_suspended_roof_flagged`: same cube but lifted to z ∈ [5, 25] (away from build plate); now bottom face is NOT on the build plate (mesh_min_along_up = 5; bottom face min = 5; equal → still filtered, since `0 < EPS_GEOMETRIC` is true). Lift to z ∈ [10, 30] WITH a separate small base touching z=0; the cube's bottom at z=10 is `(10 - 0) > EPS_GEOMETRIC` → NOT filtered → flagged. Validates that the filter is mesh-min-relative, not face-z-absolute.

(Gap-B + Gap-M composition — `OverhangRegion.angle = max observed` in degrees — is exercised by §5.2's `test_overhang_angle_uses_steepest_face`; with Gap M landing first, that §5.2 test runs against Gap-M predicate semantics. No duplication needed here.)

**Acceptance criteria**:
- Nine tests pass with `approx::assert_relative_eq!(observed_angle, expected, epsilon = 1e-6)` where applicable.
- `cargo clippy -p mesh-printability --tests --all-targets -- -D warnings` clean.
- **Test-numerical-anchor regression sweep**: existing tests `test_support_volume_calculation`, `test_validation_summary`, `test_issue_counts`, and any other test that invokes `validate_for_printing` and indirectly depends on `total_overhang_area` (via support volume estimate) may need numerical-anchor updates after Gap M lands — under the new predicate, more (and different) faces flag as overhangs, so `support_volume = overhang_area * 5.0` shifts. Sweep all `validation.rs::tests` for tests asserting overhang-derived numerics; update post-Gap-M values inline in the same commit. Verify at impl time via `cargo test -p mesh-printability` post-edit; any failing test gets either (a) numerical-anchor refresh if the test fixture exercises the new predicate correctly, or (b) re-fixture if the test depended on the buggy convention's specific output. None of the existing tests assert specific overhang flagging behavior directly (verified by inspection of `validation.rs::tests`), so no test logically conflicts with Gap M; only derived numerics shift.

**Pre-flight verification**: before authoring the commit, confirm the convention by reading the final `check_overhangs` and `evaluate_orientation` once more in current code; compute the predicate's true semantics; verify the convention lock-in matches FDM industry convention (e.g., compare against PrusaSlicer's "support overhang threshold" or Cura's "support angle" semantics — both flag faces tilted > threshold from vertical, which is what Gap M produces).

**Commit slot**: lands as commit immediately AFTER §5.1 (Gap A workspace lints) and BEFORE §5.2 (Gap B max-angle tracking). Sequence: A → M → B → D → E → F → L → K → DetectorSkipped → CHANGELOG. Detector sequence (§6): C → G → H → I → J. Then §7 examples in detector-order.

**Cross-references at PR-close**:
- Gap M's fix is documented in the `CHANGELOG.md` "Fixed" section + `COMPLETION.md` rewrite (Gap K).
- The mesh book `docs/studies/mesh_architecture/src/50-shell-and-print.md` Part 5 depth-pass calls out "v0.8 corrected the overhang flagging convention to standard FDM tilt-from-vertical semantics; pre-v0.8 callers may see different `is_printable()` results on previously-near-roof meshes" — semver behavioral note for downstream consumers.

---

### §5.10 CHANGELOG.md creation + per-commit updates

**Change**: create `mesh/mesh-printability/CHANGELOG.md` in **commit 1c** (the sub-commit that flips `[lints] workspace = true` and the only Gap A sub-commit that touches `Cargo.toml`). Per §5.1's 3-commit split, sub-commits 1a + 1b are pure src changes that don't touch Cargo.toml or CHANGELOG. Each subsequent commit (#2 onward) appends to the `[Unreleased]` section. The final pre-spec-deletion commit closes `[Unreleased]` → `[0.8.0] - YYYY-MM-DD`.

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
- File exists at `mesh/mesh-printability/CHANGELOG.md` after commit 1c.
- Each subsequent commit's diff (from commit #2 onward) includes a CHANGELOG.md update entry. Sub-commits 1a + 1b do not (they predate file creation).
- Final commit pins the release date and version under the `[0.8.0]` section.

---

## §6. Per-detector specs (5 new detectors)

Each subsection covers: definition, algorithm, preconditions/skip behavior, tolerances and config, output population, edge cases, adversarial test fixtures, performance and limits, v0.9 followups.

### §6.1 Gap C — `check_thin_walls` via ray-cast inward

**Definition**: For each face F of a watertight mesh, the local wall thickness is the distance from F's surface inward (along −N_F) to the nearest opposite face. If thickness < `config.min_wall_thickness`, F is flagged thin-walled.

**Algorithm**:
```
1. Pre-check: if !watertight or !consistent_winding, push DetectorSkipped issue, return.
2. For each face F (index i):
     C_F = centroid(F)
     N_F = outward unit normal (from existing computation)
     ray_origin = C_F - EPS_RAY_OFFSET * N_F
     ray_direction = -N_F
     min_dist = ∞
     For each face G (index j != i):
         d = möller_trumbore_intersect(ray_origin, ray_direction, G)
         if d is Some(t) where t > 0 and t < min_dist:
             min_dist = t
     If min_dist < config.min_wall_thickness:
         flagged_faces.push((i, min_dist))
3. Cluster flagged_faces by edge-adjacency (using build_edge_to_faces helper from §5.3).
4. For each cluster:
     min_thickness = min(distances in cluster)
     center = mean(centroids in cluster)
     area = sum(face_areas in cluster)
     ThinWallRegion { center, thickness: min_thickness, area, faces: cluster_indices }
     severity = classify_thin_wall_severity(min_thickness, config.min_wall_thickness)
     PrintIssue { issue_type: ThinWall, severity, description: "...", location: center, affected_elements: cluster_indices }
5. Sort regions per §4.4.
```

**Preconditions and skip behavior**: requires watertight + consistent winding. On failure: emit one Info-level `PrintIssue` with `issue_type = DetectorSkipped`, description `"ThinWall detection requires watertight mesh with consistent winding (skipped)"`, return early. No `ThinWallRegion`s populated.

**Tolerances and config**:
- `EPS_RAY_OFFSET = 1e-6 mm` for ray origin offset (avoids self-hit at ray start).
- `EPS_GEOMETRIC = 1e-9 mm` for Möller-Trumbore co-planarity edge cases.
- Source threshold: `config.min_wall_thickness` (existing field).

**Implementation note**: The Möller-Trumbore ray-triangle intersection routine does not currently exist in mesh-printability. It is added as a private helper `fn moller_trumbore(origin, direction, tri_v0, tri_v1, tri_v2) -> Option<f64>` in this commit (~30 LOC). Returns `Some(t)` for ray parameter t > 0 if hit, `None` otherwise. We do NOT pull from mesh-repair (mesh-repair has triangle-triangle intersection, not ray-triangle, in `intersect.rs`).

**Output population**:
- Per cluster: one `ThinWallRegion` with center, thickness (min observed in cluster), area (summed), faces (cluster face indices).
- Per cluster: one `PrintIssue` of type `ThinWall`, severity per `classify_thin_wall_severity`, location at cluster center, affected_elements = cluster face indices.

**Severity helper**:
```rust
fn classify_thin_wall_severity(thickness: f64, min_wall: f64) -> IssueSeverity {
    if thickness < min_wall / 2.0 {
        IssueSeverity::Critical
    } else {
        IssueSeverity::Warning  // min_wall/2 ≤ thickness < min_wall
    }
}
```

**Edge cases**:
- Co-planar faces facing opposite directions: ray hits the opposite face at distance ~ wall_thickness. Correct.
- Ray-tri intersection at vertex: Möller-Trumbore returns either both or neither incident face; both give same `t`, so doubled count doesn't affect min_dist. Acceptable.
- Concave geometry with multiple inward layers: first hit (smallest `t`) is the nearest opposite face. Correct interpretation.
- Ray exits mesh entirely (no hit): treated as min_dist = ∞; face not flagged. Possible for non-watertight geometry — handled by precondition skip.
- Two adjacent thin walls at different thicknesses: clustered into one region; reported thickness is the minimum observed.
- Highly tessellated thin shell (many small triangles forming a thin region): cluster grows to cover the region; correct.

**Adversarial test fixtures**:
- `test_thin_wall_detected_on_thin_box`: hollow watertight box with 0.4 mm walls, `min_wall_thickness = 1.0`. Expect: one region with thickness ≈ 0.4, severity = Critical (0.4 < 1.0 / 2 = 0.5).
- `test_thin_wall_no_issue_on_thick_box`: 5 mm walls, `min_wall = 1.0`. Expect: no `ThinWallRegion`s.
- `test_thin_wall_severity_critical_at_quarter_min`: 0.25 mm walls, `min_wall = 1.0`. Expect: 1 region, Critical.
- `test_thin_wall_severity_warning_at_three_quarter_min`: 0.75 mm walls, `min_wall = 1.0`. Expect: 1 region, Warning.
- `test_thin_wall_borderline_no_issue`: 1.0 mm walls, `min_wall = 1.0`. Expect: no region.
- `test_thin_wall_concave_z_shape`: Z-shaped solid with 0.5 mm thin section sandwiched between thick sections. Expect: 1 region covering the thin section only.
- `test_thin_wall_skipped_on_open_mesh`: open box (5-of-6 faces). Expect: DetectorSkipped issue, no `ThinWallRegion`s.
- `test_thin_wall_skipped_on_inconsistent_winding`: watertight cube with one face flipped. Expect: DetectorSkipped issue (precondition fails on winding).
- `test_thin_wall_two_disjoint_clusters`: two separate thin-walled cylinders + thick connector. Expect: 2 regions.
- `test_thin_wall_sort_stable_across_runs`: same input on two runs returns the same `validation.thin_walls` order (asserts §4.4 sort).

**Performance and limits**:
- Per face: O(n) ray-tri intersections.
- Total: O(n²) brute-force.
- Empirical target: <1 s for 5k tris on the project's reference machine; <5 s for 10k tris.
- Beyond 10k tris: documented perf cliff. v0.9 BVH triggered.

**v0.9 followups**:
- BVH acceleration (triple-tracked).
- Shell-based thickness via mesh-offset SDF for highly curved geometry. Trigger: a real-world mesh shows ray-cast giving misleading results due to oblique ray angles.
- Anisotropic thickness (different thresholds per direction for FDM layer-stratified printing). Trigger: a user requests separate horizontal/vertical wall thresholds.

---

### §6.2 Gap G — `check_long_bridges` via boundary-edge span analysis

**Definition**: A "long bridge" is a connected component of near-horizontal downward-facing faces whose horizontal-plane bounding-box maximum extent exceeds `config.max_bridge_span`. FDM-specific concern (and SLA, conservatively); SLS/MJF skip silently.

**Algorithm**:
```
1. Pre-check: if !config.technology.requires_supports(), return early (no diagnostic — bridges are not a concern for this tech).
2. up = config.build_up_direction (normalized).
3. Identify bridge candidates:
   mesh_min_along_up = min over all mesh vertices v of (v · up)  // computed once
   For each face F:
       N_F = outward unit normal
       angle_to_neg_up = arccos(N_F · (-up))
       if angle_to_neg_up < ANGLE_TOL_HORIZONTAL:
           Filter out build-plate-touching faces (relative to mesh-min, not absolute):
               face_min_along_up = min over F's vertices of (v · up)
               if (face_min_along_up - mesh_min_along_up) < EPS_GEOMETRIC: skip
           bridge_candidates.push(F)
4. Cluster bridge_candidates by edge-adjacency (build_edge_to_faces helper).
5. For each cluster:
   Project cluster vertices onto plane perpendicular to up.
   Compute axis-aligned bounding box of projections.
   span = max(bbox_extent_x, bbox_extent_y) of the 2D projection.
   If span > config.max_bridge_span:
       start = (center - span/2 along longest axis), back-projected to 3D
       end   = (center + span/2 along longest axis), back-projected to 3D
       boundary_edges = edges of cluster shared with non-candidate faces
       LongBridgeRegion { start, end, span, edges: boundary_edges }
       severity = classify_long_bridge_severity(span, config.max_bridge_span)
       PrintIssue { issue_type: LongBridge, severity, description: "Bridge spans X mm (limit Y mm)", location: midpoint(start, end), affected_elements: cluster face indices }
6. Sort regions per §4.4 (by `start.x, start.y, start.z`).
```

**Preconditions and skip behavior**: requires `config.technology.requires_supports()` to be true. SLS/MJF skip **silently** — no DetectorSkipped issue (bridges aren't applicable to powder-bed-supported processes; emitting a skip diagnostic would be noise).

**Tolerances and config**:
- `ANGLE_TOL_HORIZONTAL = 30°` for "near-horizontal bridge face" classification.
- `EPS_GEOMETRIC = 1e-9 mm` for "on build plate" filter (face is on build plate if min vertex `z_along_up` < EPS).
- Source threshold: `config.max_bridge_span`.

**Output population**:
- Per cluster exceeding span: one `LongBridgeRegion` with start, end, span, boundary edges.
- Per cluster: one `PrintIssue`, severity per `classify_long_bridge_severity`, location at midpoint.

**Severity helper**:
```rust
fn classify_long_bridge_severity(span: f64, max_span: f64) -> IssueSeverity {
    if span > max_span * 1.5 {
        IssueSeverity::Critical
    } else {
        IssueSeverity::Warning  // max_span < span ≤ max_span * 1.5
    }
}
```

**Edge cases**:
- Cluster overlapping with overhang region: both detectors flag independently. User gets two signals (bridge + overhang); not a duplicate-flag bug. Documented behavior.
- Cantilever (one-end-anchored horizontal face): currently flagged as bridge (algorithm doesn't distinguish). Documented as v0.9 followup (cantilever detection requires support-end analysis).
- Build-plate filter: faces touching the build plate are excluded. Implementation uses `(face_min_along_up - mesh_min_along_up) < EPS_GEOMETRIC`. Relative-to-mesh-min, so the filter works whether or not `place_on_build_plate` was called. FP-stable since `up` is normalized.
- Curved bridges: bbox approach gives axis-aligned span, which may underestimate true span. Documented as v0.9 followup.
- Bridge with mixed-tilt faces (some 5° from horizontal, some 25°): all included in cluster if within `ANGLE_TOL_HORIZONTAL`. Reported span is over the cluster as a whole. Correct.
- Diagonal bridge (long axis not aligned to X or Y): bbox_extent_x and bbox_extent_y are conservative for diagonal spans. The true span (Euclidean diagonal of bbox) might be longer. v0.8 uses max of the two extents; v0.9 could use OBB. Documented.

**Adversarial test fixtures**:
- `test_long_bridge_horizontal_slab_exceeds_span`: 20 mm × 5 mm horizontal slab at z=10, `max_bridge_span = 10`. Expect: 1 region, span ≈ 20, severity = Critical.
- `test_long_bridge_short_span_no_issue`: 5 mm × 5 mm slab, `max_bridge_span = 10`. Expect: no region.
- `test_long_bridge_borderline_warning`: 12 mm × 5 mm, `max_bridge_span = 10`. Expect: 1 region, Warning.
- `test_long_bridge_well_above_critical`: 20 mm × 5 mm, `max_bridge_span = 10`. 20 > 15 → Critical.
- `test_long_bridge_skipped_for_sls`: SLS config, 20 mm slab. Expect: no regions, no DetectorSkipped issue (silent skip).
- `test_long_bridge_bottom_face_not_flagged`: slab at z=0 (build plate). Expect: no region (build-plate filter).
- `test_long_bridge_two_disjoint_bridges`: two parallel 20 mm slabs separated by gap. Expect: 2 regions.
- `test_long_bridge_with_y_up_orientation`: 20 mm bridge oriented along +X with `+Y up` config. Expect: same flag count as `+Z up` after appropriate rotation (symmetry test).
- `test_long_bridge_cantilever_currently_flagged`: cantilever face anchored on one end. Expect: 1 region (documented v0.8 behavior — cantilever distinction is v0.9).
- `test_long_bridge_diagonal_underflagged_documented`: a 14 mm × 14 mm horizontal patch (diagonal ≈ 19.8 mm), `max_bridge_span = 15`. Expect: **0 regions** (bbox max-extent = 14 < 15, so no flag despite true diagonal exceeding threshold). Documents v0.8 bbox-conservative behavior; v0.9 OBB followup catches this case.

**Performance and limits**:
- Per face: O(1) classification.
- Cluster building: O(n × α(n)) union-find.
- Bbox projection: O(n_vertices_in_cluster).
- Total: O(n) — fast.
- No tri-count ceiling concern.

**v0.9 followups**:
- Cantilever detection (one-end-anchored faces should not flag as bridges). Trigger: a user reports false positives on cantilevered geometry.
- OBB-based span (non-axis-aligned). Trigger: a user reports diagonal bridges underflagged.
- Bridge-span via support-wall distance (anchor-to-anchor distance instead of bbox extent). Trigger: a user requests true-span semantics.

---

### §6.3 Gap H — `check_trapped_volumes` via exterior flood-fill

**Definition**: A "trapped volume" is a connected region of interior empty space inside a watertight mesh that has no path to the exterior. Critical for SLA/SLS/MJF (trapped uncured/unsintered material); Info for FDM (sealed cavity prints fine).

**Algorithm** (voxel-based flood-fill):
```
1. Pre-check: if !watertight (open_edge_count > 0), push DetectorSkipped, return.
2. up = config.build_up_direction (informational; v0.8 doesn't simulate drainage paths).
3. voxel_size = (config.min_feature_size.min(config.layer_height)) / 2.0
4. Compute mesh AABB, padded by 2 voxel_sizes on each side.
5. Allocate voxel grid: dims = (extent / voxel_size).ceil(), each voxel = u8 (0=outside-or-unknown, 1=inside, 2=exterior, 3=trapped).
6. Mark inside voxels via scanline + ray-tri intersection:
   For each (y_idx, z_idx) in the y-z grid plane:
       row_origin = (grid.x_start, y_idx * voxel_size + grid.y_start, z_idx * voxel_size + grid.z_start)
                  + (0, ROW_JITTER, ROW_JITTER)  // small offset to avoid passing exactly through vertices
       Cast ray from row_origin in direction +X across the grid.
       Find all (face, t) intersections with mesh faces using moller_trumbore.
       Sort intersections by t.
       inside = false
       For each voxel x along +X (left-to-right):
           t_voxel_start, t_voxel_end = voxel x-bounds
           Walk through intersection list: for each crossing whose t is within the voxel,
               flip `inside`.
           If `inside` at voxel midpoint: mark voxel as 1 (inside).
7. Flood-fill exterior:
   Seed at voxel grid corner (always outside since grid is padded).
   BFS over 6-connected neighbors that are 0 (outside-or-unknown).
   Mark visited voxels as 2 (exterior).
8. Identify trapped: voxels still marked 0 (not 1 [inside], not 2 [exterior]) → mark 3 (trapped).
9. Connected-component label trapped voxels via 6-connected BFS.
10. For each trapped component:
   centroid = mean of voxel centers
   volume = component_voxel_count * voxel_size^3
   bounding_box = (min voxel center - voxel_size/2, max voxel center + voxel_size/2)
   voxel_count = component voxel count (u32)
   TrappedVolumeRegion { center: centroid, volume, bounding_box, voxel_count }
   severity = classify_trapped_volume_severity(volume, technology, config)
   PrintIssue { issue_type: TrappedVolume, severity, description, location: centroid }
11. Sort regions per §4.4 (by center.x, center.y, center.z).
```

**Preconditions and skip behavior**: requires watertight mesh. On failure: DetectorSkipped issue with description `"TrappedVolume detection requires watertight mesh (skipped)"`.

**Tolerances and config**:
- `voxel_size = (min_feature_size.min(layer_height)) / 2.0` (derived per §4.2).
- `ROW_JITTER = 1e-5 mm`: applied to scanline ray origin in y/z to avoid pathological vertex/edge intersections. Each row is offset slightly differently if needed to break ties; in practice a single uniform jitter suffices for non-degenerate meshes.
- Source threshold for severity: `min_feature_size³` for "below resolution → Info".

**Output population**:
- Per trapped component: one `TrappedVolumeRegion` with center, volume, bounding_box, voxel_count.
- Per component: one `PrintIssue`, severity per `classify_trapped_volume_severity` (technology-aware).

**Severity helper**:
```rust
fn classify_trapped_volume_severity(volume: f64, tech: PrintTechnology, config: &PrinterConfig) -> IssueSeverity {
    let res_volume = config.min_feature_size.powi(3);
    if volume < res_volume {
        return IssueSeverity::Info;  // below resolution; likely intentional or noise
    }
    match tech {
        PrintTechnology::Sla | PrintTechnology::Sls | PrintTechnology::Mjf => IssueSeverity::Critical,
        PrintTechnology::Fdm | PrintTechnology::Other => IssueSeverity::Info,
    }
}
```

**Edge cases**:
- Pinhole leak (≤ voxel_size diameter): flood-fill leaks through, trapped not detected. Intentional — features below printer resolution shouldn't false-flag.
- Multiple disjoint cavities: each gets its own component → separate `TrappedVolumeRegion`.
- Cavity touching external boundary at a single voxel: flood-fill connects → not trapped.
- Mesh entirely engulfed in another mesh (nested closed surfaces): the gap between is a trapped volume. Detected correctly.
- Ray-tri intersection at vertex/edge: scanline `ROW_JITTER` mitigates exact-coincidence cases. If a vertex still sits on a row by accident, FP-stability is preserved by exclusive parity (bottom-edge inclusive, top-edge exclusive — standard CG technique).
- Very large grid (mesh > 100mm in some direction): memory grows as O(extent³ / voxel_size³). Document the perf characteristics.
- `up` direction: v0.8 doesn't use it for drainage simulation; trapped is purely topological. v0.9 could simulate "would this drain through orientation" using `up`. Documented.

**Adversarial test fixtures**:
- `test_trapped_volume_sphere_inside_cube`: solid cube with internal spherical cavity (radius 5 mm, cube 20mm). Expect: 1 region, volume ≈ (4/3)π·5³ ≈ 524 mm³ (within voxel-discretization tolerance), bbox ≈ 10×10×10.
- `test_trapped_volume_no_cavity`: solid cube, no internal void. Expect: 0 regions.
- `test_trapped_volume_critical_for_sla`: same fixture with SLA config. Expect: severity = Critical.
- `test_trapped_volume_critical_for_sls`: same with SLS. Expect: Critical.
- `test_trapped_volume_critical_for_mjf`: same with MJF. Expect: Critical.
- `test_trapped_volume_info_for_fdm`: same with FDM. Expect: Info.
- `test_trapped_volume_info_below_min_feature`: tiny void (volume < min_feature_size³) in any tech. Expect: Info.
- `test_trapped_volume_skipped_on_open_mesh`: cube with one face removed. Expect: DetectorSkipped issue, no regions.
- `test_trapped_volume_two_disjoint_cavities`: cube with 2 separate spherical voids. Expect: 2 regions.
- `test_trapped_volume_volume_within_10pct_of_analytical`: spherical cavity fixture (analytical volume = (4/3)π·5³ ≈ 523.6 mm³); assert `region.volume` within ±10% of analytical (voxel discretization noise band; `approx::assert_relative_eq!` with `max_relative = 0.10`).
- `test_trapped_volume_sort_stable`: same input → same region order across runs (asserts §4.4 sort).

**Performance and limits**:
- Memory: O((part_size / voxel_size)³) bytes. For 100 mm part, voxel_size = 0.4 mm: 250³ ≈ 15.6M bytes ≈ 16 MB.
- Inside-test scanline: O((part_size / voxel_size)² × n_faces). For 250² grid × 10k faces: 6.25e8 ops; ~1s.
- Flood-fill: O(grid_size). Fast.
- Empirical target: <2 s for 100mm-extent meshes with 0.4mm voxel; document the ceiling.
- Beyond ~250³ grid (≈100mm part at 0.4mm voxel): consider adaptive voxel sizing.

**v0.9 followups**:
- Drainage-path simulation along `up`-direction (orientation-aware trapped volumes). Trigger: a real mesh has a cavity that would drain in one orientation but not another, and the user wants the validator to know.
- Adaptive voxel sizing for parts much larger than `min_feature_size`. Trigger: validation runtime > 5 s on a real mesh.
- mesh-sdf integration if scanline FP issues surface in cross-platform tests. Trigger: a CI test fails on macOS/Linux/Windows divergence in voxel inside-test.

---

### §6.4 Gap I — `check_self_intersecting` via mesh-repair re-use

**Definition**: Two non-adjacent triangles intersect each other (their interiors share a point not on a shared edge).

**Algorithm**:
```
1. Call mesh_repair::detect_self_intersections(mesh, &IntersectionParams::default()).
2. If result.has_intersections == false: return early.
3. For each (a, b) in result.intersecting_pairs:
   // Canonicalize ordering: face_a < face_b. mesh-repair does not guarantee canonical order in its output.
   let (face_a, face_b) = if a < b { (a, b) } else { (b, a) };
   centroid_a = compute_face_centroid(mesh, face_a as usize)
   centroid_b = compute_face_centroid(mesh, face_b as usize)
   approximate_location = midpoint(centroid_a, centroid_b)
   SelfIntersectingRegion { face_a, face_b, approximate_location }
4. Push one PrintIssue:
   description = format!("{N} self-intersecting triangle pair(s){}", if result.truncated { " (search truncated; total may be higher)" } else { "" })
   severity = Critical
5. Sort regions per §4.4 (by face_a, face_b).
```

**Preconditions and skip behavior**: none. mesh-repair handles all input gracefully (single-face / empty / large meshes).

**Tolerances and config**:
- `IntersectionParams::default()`: `max_reported = 100`, `epsilon = 1e-10`, `skip_adjacent = true`.

**Output population**:
- One `SelfIntersectingRegion` per `(face_a, face_b)` pair from mesh-repair (capped at 100 by default params).
- One `PrintIssue` summarizing total count + truncation status; severity = Critical.

**Severity**: always Critical — slicer behavior on self-intersecting meshes is undefined; print quality unpredictable.

**Edge cases**:
- Adjacent triangles sharing an edge: skipped via `skip_adjacent = true`. Not flagged as self-intersection.
- Co-planar overlapping triangles: mesh-repair flags via `epsilon`. Handled.
- Truncation at 100 pairs: description mentions "(search truncated; total may be higher)". User can call `mesh_repair::detect_self_intersections` directly with `IntersectionParams::exhaustive()` for full count.
- Vertex-only contact: not an intersection (interiors don't share points). Not flagged.
- Edge-on-face contact (edge of triangle A lies on face of triangle B without crossing): borderline; mesh-repair's epsilon decides. Document.

**Adversarial test fixtures**:
- `test_self_intersecting_two_overlapping_triangles`: two triangles passing through each other at a shared interior point. Expect: ≥1 region, severity = Critical.
- `test_self_intersecting_clean_cube_no_issue`: clean watertight cube. Expect: 0 regions.
- `test_self_intersecting_adjacent_triangles_skipped`: two triangles sharing an edge but not overlapping. Expect: 0 regions.
- `test_self_intersecting_truncation_at_100`: mesh with 200+ intersections (e.g. self-folded thin sheet). Expect: 100 regions, description contains "truncated".
- `test_self_intersecting_approximate_location_in_midpoint`: known-position triangle pair; assert `approximate_location` within `EPS_GEOMETRIC` of expected midpoint.
- `test_self_intersecting_critical_blocks_is_printable`: any self-intersection → `is_printable() == false`.
- `test_self_intersecting_sort_stable`: same input two runs → same region order (asserts §4.4 sort).
- `test_self_intersecting_face_indices_unique_per_pair`: assert `face_a < face_b` for all reported pairs (canonical ordering).

**Performance and limits**:
- mesh-repair uses bounding-box culling + rayon parallelism.
- Empirical target: <500 ms for 5k tris; <2 s for 10k tris on the project's reference machine.
- Beyond 10k tris with many intersections: may exceed 5 s. v0.9 BVH followup applies (already triple-tracked).

**v0.9 followups**:
- Tunable IntersectionParams from `validate_for_printing` (already triple-tracked).
- Self-intersection auto-fix integration with mesh-repair (validator detects → repair fixes). Trigger: a workflow asks for "validate-and-fix" semantics.

---

### §6.5 Gap J — `check_small_features` via connected-component bbox extent

**Definition**: A connected component of the mesh whose bounding-box maximum extent is less than `config.min_feature_size`. Catches floating debris and tiny isolated protrusions that the printer cannot physically resolve.

**Algorithm**:
```
1. Build edge_to_faces map (using build_edge_to_faces helper from §5.3).
2. Build face-adjacency graph: two faces are adjacent if they share an edge (regardless of orientation).
3. Run union-find to identify connected components. Each face → component ID.
4. For each component:
   vertex_indices = unique vertex indices across all faces in component
   bbox = AABB of those vertices
   max_extent = max(bbox.x.size, bbox.y.size, bbox.z.size)
   if max_extent < config.min_feature_size:
       centroid = mean(vertex positions)
       volume = abs(signed_volume(component))  // divergence theorem on closed components; absolute for open
       face_count = component face count
       SmallFeatureRegion { center: centroid, max_extent, volume, face_count, faces: component_face_indices }
       severity = classify_small_feature_severity(max_extent, config.min_feature_size)
       PrintIssue { issue_type: SmallFeature, severity, description, location: centroid }
5. Sort regions per §4.4 (by face indices, smallest first).
```

**Signed volume helper** (divergence theorem):
```rust
fn signed_volume(mesh: &IndexedMesh, face_indices: &[u32]) -> f64 {
    let mut vol = 0.0;
    for &fi in face_indices {
        let face = mesh.faces[fi as usize];
        let v0 = mesh.vertices[face[0] as usize];
        let v1 = mesh.vertices[face[1] as usize];
        let v2 = mesh.vertices[face[2] as usize];
        // (v0 · (v1 × v2)) / 6.0  — correct sign for outward winding
        vol += (v0.x * (v1.y * v2.z - v1.z * v2.y)
              + v0.y * (v1.z * v2.x - v1.x * v2.z)
              + v0.z * (v1.x * v2.y - v1.y * v2.x)) / 6.0;
    }
    vol
}
```

**Preconditions and skip behavior**: none. Tolerant of any input (empty, open, non-manifold). Edge cases noted below.

**Tolerances and config**:
- Source threshold: `config.min_feature_size`.
- No FP epsilon needed for bbox extent comparison (direct subtraction).

**Output population**:
- One `SmallFeatureRegion` per qualifying component.
- One `PrintIssue` per component, severity per `classify_small_feature_severity`.

**Severity helper**:
```rust
fn classify_small_feature_severity(max_extent: f64, min_feature: f64) -> IssueSeverity {
    if max_extent < min_feature / 2.0 {
        IssueSeverity::Warning  // definitely below resolution
    } else {
        IssueSeverity::Info     // borderline; may print
    }
}
```

**Edge cases**:
- Single-component mesh (entire mesh is one piece) below threshold: flagged. User likely has a unit-conversion error (e.g. mesh in meters not mm). Surface as a signal; user decides.
- Many tiny floating fragments: many regions emitted; output grows linearly. No cap.
- Open / non-watertight component: signed_volume returns junk (not a closed surface) but no panic. We use `abs(signed_volume)` which is at least non-negative; users reading the field should treat it as approximate for non-watertight inputs. Documented.
- Degenerate (zero-area) faces: included in component face_count; don't affect bbox or volume meaningfully. Acceptable.
- Component with disconnected vertex islands (two faces sharing a vertex but not an edge): treated as separate components per the edge-adjacency definition. Matches `feedback_no_reflexive_defer` — vertex-only adjacency is not face-adjacency.
- Deep-stacking of extent and volume: a long thin spike has large `max_extent` but small `volume`. Spike won't flag (max_extent > threshold). v0.9 could add volume-based criterion.

**Adversarial test fixtures**:
- `test_small_feature_floating_triangle_detected`: large mesh + isolated triangle 0.1 mm × 0.1 mm, `min_feature_size = 0.4`. Expect: 1 region for the fragment, max_extent ≈ 0.1, severity = Warning (0.1 < 0.4 / 2 = 0.2).
- `test_small_feature_borderline_no_issue`: feature with max_extent = 0.5 mm, `min_feature_size = 0.4`. Expect: 0 regions.
- `test_small_feature_below_half_threshold_warning`: feature with max_extent = 0.1, `min_feature_size = 0.4`. Expect: severity = Warning.
- `test_small_feature_just_below_threshold_info`: max_extent = 0.3, `min_feature_size = 0.4`. Expect: severity = Info.
- `test_small_feature_clean_main_body_not_flagged`: 100 mm cube. Expect: 0 regions.
- `test_small_feature_two_floating_fragments`: 2 disconnected small components + main body. Expect: 2 regions for fragments; main body not flagged.
- `test_small_feature_face_adjacency_via_edge_only`: two faces sharing only a vertex are in different components.
- `test_small_feature_volume_via_divergence_theorem`: unit-cube fragment (1 mm side, volume = 1 mm³) with `min_feature_size = 2.0` mm so the fragment is flagged (max_extent = 1.0 < 2.0). Assert `region.volume ≈ 1.0` mm³ within `1e-6` tolerance (FP-stable for closed-component divergence theorem on exact-representable inputs).
- `test_small_feature_open_component_no_panic`: 5-of-6-face open box (small extent). Expect: 1 region; volume is approximate, no panic.
- `test_small_feature_sort_stable`: same input two runs → same region order (asserts §4.4 sort).

**Performance and limits**:
- edge_to_faces + union-find: O(n_faces × α(n_faces)).
- Per-component bbox: O(n_vertices_in_component).
- Total: O(n_faces) practically.
- No tri-count ceiling concern.

**v0.9 followups**:
- Curvature-based small-feature detection (small protrusions on a larger body). Trigger: a user reports a small bump on a larger part not flagged by extent-based detection.
- Volume-based threshold (in addition to extent). Trigger: a long thin spike (large extent, tiny volume) prints poorly and the user wants it flagged.

---

## §7. Example design

Eight examples ship with the v0.8 arc, one per detector plus orientation, technology-sweep, and a capstone showcase. Each lives at `examples/mesh/printability-<name>/` next to the existing `mesh-repair-walkthrough` / `shell-generation-*` / `mesh-offset-*` examples — same directory pattern, same `Cargo.toml` shape, same `out/*.ply` artifact layout. Each example demonstrates **one concept**, has a **museum-plaque README** (`feedback_museum_plaque_readmes`), is **self-contained** with no shared fixture helpers (`feedback_simplify_examples` ethos extends to per-example duplication), and is reviewed **one at a time** with two passes — numbers (Claude) + visuals (user) — before moving to the next (`feedback_one_at_a_time` + `feedback_one_at_a_time_review`).

### §7.0 Cross-cutting pattern

#### Directory shape

```
examples/mesh/printability-<name>/
├── Cargo.toml
├── README.md
├── src/main.rs
└── out/                  ← gitignored; regenerated each run
    ├── mesh.ply          ← input mesh as authored (or post-construction)
    ├── issues.ply        ← point-cloud of region centroids
    └── ... (per-example extras, e.g. annotated.ply, voxels.ply)
```

The `out/` directory is created at runtime (`std::fs::create_dir_all`); produced PLYs are git-ignored via `examples/mesh/.gitignore` (line 5: `*/out/`), inherited by every example under `examples/mesh/`. The example owns its fixture: no shared `fixture_lib`, no `tests/common`. If two examples need similar geometry, each constructs its own — pedagogical clarity beats DRY for examples.

#### Cargo.toml shape

Each example is a workspace member with `publish = false`, named `example-mesh-printability-<name>`. Dependencies are minimal: `mesh-types`, `mesh-io`, `mesh-printability`, `anyhow` (for `main() -> Result<()>`), and per-example extras. Common per-example extras: `mesh-repair` (for the §7.4 self-intersection fixture's verification + the optional self-intersection auto-fix demo); `mesh-offset` (for §7.1 + §7.3's optional MC fallback fixture-construction path — `mesh-offset::marching_cubes(&ScalarGrid, &MarchingCubesConfig) -> IndexedMesh` is the public, L0, workspace-internal MC entry point); `mesh-shell` (only if a fixture genuinely needs an offset shell — none of the v0.8 examples do, but the door is open for v0.9). §7.8 (showcase) hand-authors all components; no MC fallback applies there. **Note**: mesh-sdf is NOT in this list — it exposes only SDF queries (no public MC API), so it's not the right tool for fixture construction. The `[lints] workspace = true` block is required.

#### `main()` shape — numbers-pass via assertions

Each example's `main()` is structured:

```rust
fn main() -> anyhow::Result<()> {
    // 1. Construct fixture mesh deterministically.
    let mesh = build_fixture();

    // 2. Configure printer.
    let config = PrinterConfig::fdm_default();

    // 3. Validate.
    let validation = validate_for_printing(&mesh, &config);

    // 4. Print summary to stdout (visible to the user reading terminal output).
    println!("{}", validation.summary());

    // 5. Numerical anchors: assert region/issue counts, severities, centroids
    //    within FP tolerance. Failure → non-zero exit, breaking the
    //    numbers-pass gate. Each assertion's failure message names the
    //    invariant being checked.
    assert_eq!(validation.<field>.len(), N, "expected exactly N regions");
    // ... severity / centroid / volume assertions ...

    // 6. Save mesh + issue artifacts to out/.
    fs::create_dir_all("out")?;
    save_ply(&mesh, "out/mesh.ply", false)?;        // ASCII PLY for human-readable diff
    save_issue_centroids(&validation, "out/issues.ply")?;

    Ok(())
}
```

The numbers-pass gate is **the example exits 0 on a clean run**. Per `feedback_risk_mitigation_review` ("pass-by-correctness vs pass-by-coincidence"), each assertion's message names the bug it catches — `assert_eq!(severity, Critical, "0.4mm wall under min_wall=1.0 must be Critical, not Warning")` — so a regression that flips Warning→Critical can't pass under a loose `assert!(severity != Info)`.

#### Issue-centroid point-cloud helper (per-example, copy-paste)

Each example needs a small helper to write region centroids as a point-cloud PLY. Helper is duplicated per-example (not factored out — see `feedback_simplify_examples`):

```rust
fn save_issue_centroids(v: &PrintValidation, path: &str) -> anyhow::Result<()> {
    let mut centroids: Vec<Point3<f64>> = Vec::new();
    centroids.extend(v.thin_walls.iter().map(|r| r.center));
    centroids.extend(v.long_bridges.iter().map(|r| {
        Point3::new(
            (r.start.x + r.end.x) * 0.5,
            (r.start.y + r.end.y) * 0.5,
            (r.start.z + r.end.z) * 0.5,
        )
    }));
    centroids.extend(v.trapped_volumes.iter().map(|r| r.center));
    centroids.extend(v.self_intersecting.iter().map(|r| r.approximate_location));
    centroids.extend(v.small_features.iter().map(|r| r.center));
    let mesh = IndexedMesh::from_parts(centroids, vec![]);  // points only
    save_ply(&mesh, path, false)?;  // ASCII for inspectability
    Ok(())
}
```

`IndexedMesh::from_parts(verts, vec![])` produces a vertex-only mesh (verified in `cf-geometry/src/mesh.rs:61`); `save_ply(&mesh, path, false)` (signature verified in `mesh-io/src/ply.rs:179`) writes it as ASCII PLY with element `vertex` and zero `face`s, which MeshLab/ParaView/Blender open as a point-cloud. **Verification at impl time**: write a 2-vertex empty-face PLY in the first example's commit; if mesh-io rejects the empty-face case, fix it as a §5-prerequisite per `feedback_adjacent_crate_flags`.

#### Annotated coloring — out of v0.8 scope

Per-face severity coloring (red/orange/gray) on the annotated mesh would require either (a) face-color attributes (AttributedMesh currently only supports per-vertex attributes) or (b) vertex-duplication-per-face with vertex-colored PLY. v0.8 ships **point-cloud `issues.ply`** instead of face-colored mesh PLYs. Per-region locality is communicated through the centroid point-cloud + the README's "look for ..." prose. Triple-tracked as v0.9 followup: trigger for v0.9 work is "user reports the centroid point-cloud is not localizing enough for visual review" or "AttributedMesh gains face-attribute support and the change becomes ~30 LOC."

#### README structure (museum plaque)

```
# printability-<name>

**One-sentence summary** of what the example demonstrates.

## What it does

Paragraph: fixture geometry, the detector being demonstrated, why this
fixture exposes the detector cleanly. End with the printer-config
choice and rationale (FDM/SLA/SLS/MJF; threshold values that make the
detector fire vs not).

## The detector pipeline

ASCII diagram or 3–5 bullets walking through the algorithm steps as
applied to this fixture (e.g., "ray cast from face F's centroid;
hits opposite face at t=0.4mm; flagged because 0.4 < min_wall=1.0").

## Numerical anchors

- Region count: N
- Severity: Critical/Warning/Info
- Centroid coordinates (within tolerance)
- Volume / span / extent values
- `is_printable() == false/true`

All asserted in main() — a failed numerical anchor breaks the run.

## Visuals

- `out/mesh.ply` — the part. Open in MeshLab/ParaView/Blender.
  (What to look for, e.g., "the upper face is 0.4mm thinner than the
  others — visually a thin lip near the top.")
- `out/issues.ply` — point-cloud of region centroids. Each point marks
  the center of a flagged region.
- (per-example extras, e.g., `out/voxels.ply` for TrappedVolume.)

Camera notes: which axis is up; which face to look at first; what
artifacts to expect (e.g., "f3d may render the trapped cavity sphere
as a winding-pair artifact since SDF is two-sided" — apply the
`feedback_f3d_winding_callout` callout near the top of the README
when the artifact is load-bearing for visual interpretation).

## Run

cargo run -p example-mesh-printability-<name> --release

## Notes

Anything else load-bearing — e.g., why a particular FP tolerance was
used; why this fixture doesn't trigger a different detector that
might naively seem to fire.
```

#### FP-stability convention

All numerical-anchor assertions use `approx::assert_relative_eq!(..., epsilon = 1e-6)` for derived geometric quantities; never `f64::EPSILON`. Per `feedback_release_mode_heavy_tests`, the README's run command always uses `--release`. Cross-platform smoke (macOS/Linux/Windows) — verify at impl time that the workspace's `cargo xtask grade-all` (current CI gate) actually compiles the new example crates; if not, examples are local-only smoke and v0.8 ships unchanged but visual regressions caught only by hand. Verification command: `cargo build --workspace --release` on the branch covers the example crate compilation; an example failing to compile blocks the corresponding §6 detector commit.

#### Realistic-looking parts vs test fixtures

Per `feedback_simplify_examples`, fixtures should look like real parts. Trade-offs:

- **Hollow box with thinning side** (ThinWall) — looks like a real container with a tapered wall. Good.
- **H-shape with horizontal lintel** (LongBridge) — looks like a doorway / chair frame. Good.
- **Solid cube with internal sphere cavity** (TrappedVolume) — looks like a sealed sample chamber. Good.
- **Two interpenetrating cylinders** (SelfIntersecting) — looks like a bad boolean union. Realistic failure mode. Good.
- **Cube + small cylindrical burr** (SmallFeature) — looks like a CAD leftover from a boolean-cut operation. Good. Avoid "isolated triangle floating in space" — too synthetic.
- **Leaning column** (Orientation) — looks like a part that prints way better lying down. Good.
- **Hollow box with thin wall + sealed cavity** (Tech-sweep) — same fixture, different validators. Good.
- **Small mounting bracket with overhang + thin lip + tiny hole** (Showcase) — looks like a real CAD part with multiple printability concerns. Good.

#### Implementation cadence within the arc

Examples land **after** the corresponding detector commits (§5, §6) so each example exercises the actually-shipped detector. Per `feedback_one_at_a_time` + `feedback_one_at_a_time_review`:

1. Land the detector commit (§5/§6); confirm tests green via `cargo xtask grade mesh-printability --skip-coverage`.
2. Author the example (one example per commit).
3. Run `cargo run -p example-mesh-printability-<name> --release`; confirm assertions pass (numbers-pass).
4. Commit example with message `feat(examples): mesh-printability-<name> visual demo for <gap>`.
5. Pause for user visuals-pass. User opens PLYs in their viewer of choice; reports verdict.
6. Address feedback if any; otherwise proceed to next example.

This gives each example a clean numbers + visuals confirmation before the next one starts. Eight examples × (~30 min author + ~5 min user review) ≈ 4–5 hours of cadence; spread across the arc so a slip on one detector doesn't backlog the rest.

---

### §7.1 `printability-thin-wall` — Gap C demonstration

**Detector**: ThinWall (`check_thin_walls`) via inward ray-cast.

**Concept**: A hollow box with one face deliberately thinned exposes ThinWall detection — the thin face's inward ray hits the opposite face at less than `min_wall_thickness`, flagging the cluster.

**Fixture geometry**:

A double-walled hollow box. Outer cube 30×20×15 mm; inner cavity 27×17×13.1 mm with the inner cavity's top raised so the top wall thins to 0.4 mm. Watertight + consistently wound — the input precondition for ThinWall holds.

**Construction approach** (impl-time choice; spec lists options in preference order):
1. **Hand-author** 12 outer triangles + 12 inner-cavity triangles (24 triangles total, 16 unique vertices: 8 outer cube corners + 8 inner-cavity corners), vertex-list-driven. Each cube face is split along a diagonal into 2 triangles. Outer faces wound CCW-from-outside (normals point outward, away from solid material); inner-cavity faces wound CCW-from-cavity-side (normals point into the cavity, away from solid material — i.e., REVERSED from outer winding so each face's normal points away from the solid that surrounds the face). Concatenate with vertex-index offset; result is watertight (each edge shared by exactly 2 faces — both within outer, or both within inner) but topologically two disconnected components in the same `IndexedMesh`. Verification at impl time: confirm that `validate_for_printing` treats this as watertight (each edge appears exactly twice; component count is incidental to manifold check).
2. **`mesh-offset::marching_cubes(grid, config)`** on a closed-form SDF (`max(box_sdf, -inner_box_sdf)` representing solid = outer ∩ ¬inner) sampled to a `ScalarGrid`. mesh-offset's MC is the public, L0, workspace-internal MC entry point (mesh-sdf has no public MC API — it exposes SDF queries only; mesh-lattice's MC is private). Adds `mesh-offset = { workspace = true }` to the example crate's `[dependencies]` per §7.0. Pre-flight verified 2026-04-30 that mesh-offset is L0 (same tier as mesh-printability) so the example's dep graph stays layer-clean.
3. **NOT mesh-shell directly** — mesh-shell is offset-based, not CSG-based; not the right tool for box-minus-box.

Hand-author (option 1) is the recommended baseline; the example's deterministic 24-triangle construction is the kind of thing the spec can lock in. Option 2 (mesh-offset MC) is the documented fallback if hand-authoring proves error-prone at impl time, but pre-flight verified the 24-triangle hand-author is tractable.

**FDM config**:
- `PrinterConfig::fdm_default()` — `min_wall_thickness = 1.0 mm`, `max_overhang_angle = 45°`, `min_feature_size = 0.8 mm`.

**Geometry math (lock-in)**:

Outer cube: x ∈ [0, 30], y ∈ [0, 20], z ∈ [0, 15].
Inner cavity (the void): x ∈ [1.5, 28.5], y ∈ [1.5, 18.5], z ∈ [1.5, 14.6].
Side wall thicknesses: x walls = 1.5 mm, y walls = 1.5 mm, bottom wall = 1.5 mm.
Top wall thickness: 15 − 14.6 = 0.4 mm.
Inner cavity dimensions: 27 × 17 × 13.1 mm.
Inner cavity volume: 6011.7 mm³.
Inner cavity centroid: ((1.5+28.5)/2, (1.5+18.5)/2, (1.5+14.6)/2) = **(15, 10, 8.05)**.

**ThinWall clustering analysis** (load-bearing for `thin_walls.len()` prediction):

`check_thin_walls` (§6.1) clusters flagged faces by **edge-adjacency** using `build_edge_to_faces`. Two faces are in the same cluster iff they share an edge. In this fixture:

- The outer top face's 2 triangles share their diagonal edge (and share their 4 other edges with the unflagged outer side faces).
- The inner top face's 2 triangles likewise share their diagonal edge among themselves.
- The outer top face and the inner top face are **topologically disjoint** — they share no vertices or edges (separate components per the construction approach).

Therefore: **2 clusters** — one outer-top cluster (2 triangles, 600 mm² area, centroid at (15, 10, 15)), one inner-top cluster (2 triangles, 459 mm² area, centroid at (15, 10, 14.6)). Both clusters report `thickness = 0.4 mm` (their respective ray-cast distances to the opposing top face).

**Expected output**:
- `validation.thin_walls.len() == 2` — outer top cluster + inner top cluster.
- Both clusters: `thickness ≈ 0.4 mm` within `1e-6`.
- Outer cluster centroid: `(15, 10, 15)` within `1e-6`. Inner cluster centroid: `(15, 10, 14.6)` within `1e-6`.
- Outer cluster area: 600 mm² (within triangulation tolerance). Inner cluster area: 459 mm².
- Both clusters emit `PrintIssue` with `issue_type == ThinWall`, severity `Critical` (0.4 < 1.0/2 = 0.5). So `validation.issues` has at least 2 ThinWall issues + 1 TrappedVolume issue.
- `validation.is_printable() == false` (driven by the Critical ThinWall issues).
- Co-flag: the cavity is watertight + sealed, so TrappedVolume also fires on FDM as Info severity. ThinWall is the load-bearing concept, TrappedVolume is the documented co-flag.

**Assertion list** (numbers-pass, in main()):
1. `validation.thin_walls.len() == 2` — "expected outer-top and inner-top clusters; topologically disjoint via edge-adjacency"
2. For each `region` in `validation.thin_walls`: `approx::assert_relative_eq!(region.thickness, 0.4, epsilon = 1e-6)` — "both clusters must report 0.4mm thickness"
3. **Cluster identification**: locate the outer cluster (centroid z = 15) and inner cluster (centroid z = 14.6) by z-coordinate; assert one of each. Specifically: `(walls[0].center.z, walls[1].center.z)` sorted ascending must be `(14.6, 15.0)` within `1e-6`.
4. Outer cluster area ∈ `[599, 601]`; inner cluster area ∈ `[458, 460]` (FP-stable on exact-representable inputs).
5. `validation.issues.iter().filter(|i| matches!(i.issue_type, ThinWall) && matches!(i.severity, Critical)).count() == 2` — "two Critical ThinWall issues, one per cluster"
6. `!validation.is_printable()` — "0.4mm wall under min_wall=1.0 blocks printability"
7. `validation.trapped_volumes.len() == 1` — "sealed inner cavity must flag as TrappedVolume on FDM (Info)"
8. `approx::assert_relative_eq!(validation.trapped_volumes[0].volume, 6011.7, max_relative = 0.10)` — analytical interior volume; voxel-discretization tolerance per §6.3.
9. `validation.trapped_volumes[0].center` within `voxel_size` (= 0.1 mm at FDM defaults) of `(15, 10, 8.05)`.
10. `validation.overhangs.len() >= 1` — "cavity-ceiling co-flag: the inner top face's normal points down into cavity, `overhang_angle = 90°`, flagged Critical under FDM `max=45°`". The example documents this co-flag in the README.
11. The pedagogically-load-bearing flag is the ThinWall Critical (which drives `is_printable()` independently). The cavity-ceiling overhang is a documented structural-detector co-flag that the user learns to read alongside ThinWall.

**Visuals**:
- `out/mesh.ply` — the hollow box. Camera: from the +Y side, looking along −Y at the X-Z plane; the thin top face should be visually distinguishable as a much thinner lip than the side walls (cross-section view in MeshLab is the cleanest rendering). f3d-callout near the top of the README: the inner-cavity surface and outer surface are wound in opposite directions, so f3d may render only one (winding-pair artifact); MeshLab's two-sided rendering shows both.
- `out/issues.ply` — three points: outer ThinWall cluster center at (15, 10, 15), inner ThinWall cluster center at (15, 10, 14.6), TrappedVolume centroid at (15, 10, 8.05).

**README pitfalls**:
- "Wait, the cavity is sealed — won't FDM TrappedVolume Info also fire?" Yes; document it in README so the user isn't surprised by two issues. The example demonstrates ThinWall as **the load-bearing concept**, with TrappedVolume as a documented co-flag.
- "Why not use a single-walled thin box?" A single-walled thin box is open (5-of-6 face), so ThinWall is **skipped** via the watertight-or-skip precondition (§4.1 + §6.1). The double-walled construction is necessary for ThinWall to actually run.

**Implementation slot**: lands as commit immediately after the §6.1 ThinWall detector commit.

---

### §7.2 `printability-long-bridge` — Gap G demonstration

**Detector**: LongBridge (`check_long_bridges`) via boundary-edge span analysis.

**Concept**: An H-shape — two pillars + a horizontal lintel — with a lintel longer than `max_bridge_span`. The lintel's downward-facing face is flagged as a bridge.

**Fixture geometry (H-shape, watertight manifold)**:

- **Pillar 1**: x ∈ [0, 5], y ∈ [0, 5], z ∈ [0, 18]. (5×5×18 mm.)
- **Pillar 2**: x ∈ [25, 30], y ∈ [0, 5], z ∈ [0, 18]. (5×5×18 mm.)
- **Slab (lintel)**: x ∈ [-2.5, 32.5], y ∈ [0, 5], z ∈ [18, 20]. (35×5×2 mm.)

Pillar tops at z=18 are **covered** by the slab and become interior (not part of the boundary surface). The slab's bottom face is exposed in three regions, separated by the pillar-attachment cutouts:
- **Left cantilever**: x ∈ [-2.5, 0], y ∈ [0, 5]. 2.5 × 5 = 12.5 mm². Span: 2.5 mm.
- **Middle bridge**: x ∈ [5, 25], y ∈ [0, 5]. 20 × 5 = 100 mm². Span: 20 mm. ← bridge target.
- **Right cantilever**: x ∈ [30, 32.5], y ∈ [0, 5]. 2.5 × 5 = 12.5 mm². Span: 2.5 mm.

These three regions are topologically disconnected (separated by the pillar-attachment holes in the slab bottom), so they form three independent edge-adjacency clusters per §6.2.

**Construction approach**: hand-authored as a single watertight manifold solid (the boolean union of the 3 boxes) using a deterministic vertex list with shared vertices at the pillar-slab junctions. Approximately 16 unique vertices and 28 triangles total. The pillar top faces (5×5 at z=18) are NOT part of the boundary; the slab bottom is a 35×5 rectangle with two 5×5 rectangular holes (over the pillars), triangulated as a polygon-with-holes.

Verification at impl time: confirm `check_basic_manifold` reports zero open edges + zero non-manifold edges + zero winding inconsistencies (post-Gap-F detector); confirm `mesh_repair::detect_self_intersections` returns empty pairs.

**FDM config**:
- `PrinterConfig::fdm_default()` — `max_bridge_span = 10 mm`. Middle bridge span 20 mm > 1.5 × 10 = 15 mm → Critical.

**Expected output**:
- `validation.long_bridges.len() == 1` — only the middle bridge (20 mm span) exceeds threshold; the two cantilevers (2.5 mm each) are below threshold and not flagged.
- Middle bridge `region.span ≈ 20.0 mm` within `1e-6` (bbox extent along x of the middle bridge cluster).
- `region.start.x ≈ 5.0, region.end.x ≈ 25.0` — endpoints back-projected from the cluster's bbox.
- One `PrintIssue` of type `LongBridge` with severity `Critical` (20 > 15).
- `validation.is_printable() == false` (LongBridge Critical drives this, plus ExcessiveOverhang per Gap M).
- ExcessiveOverhang **also fires** for the slab's downward-facing bottom regions (and any cantilevers/middle bridge — all 100% downward-facing). Under post-Gap-M convention, all 3 bottom regions have `overhang_angle = 90°`, well past the 45° threshold + 30° Critical band → severity Critical. The Gap-D region-splitting (§5.3) means each connected downward-region is a separate `OverhangRegion`. So `validation.overhangs.len() == 3` — left cantilever, middle bridge, right cantilever — each as a separate Critical cluster.
- `validation.trapped_volumes.len() == 0` — H-shape has no sealed cavity.

**Assertion list**:
1. `validation.long_bridges.len() == 1` — "only the middle bridge exceeds 10mm threshold; cantilevers do not"
2. `approx::assert_relative_eq!(region.span, 20.0, epsilon = 1e-6)` — "middle bridge span must match the inter-pillar gap"
3. LongBridge issue severity `Critical` (20 > 1.5 × 10 = 15)
4. `region.start.x` within `1e-6` of `5.0` and `region.end.x` within `1e-6` of `25.0` (bbox-derived endpoints; FP-stable on exact-representable fixture)
5. `validation.overhangs.len() == 3` — "Gap-D-split overhang clusters: left cantilever + middle bridge + right cantilever"
6. All three ExcessiveOverhang issues severity `Critical` (90° tilt > 45° + 30°)
7. `validation.trapped_volumes.len() == 0` — no sealed cavity in H-shape
8. **Tech-skip regression**: re-run with `PrinterConfig::sls_default()`, assert `validation.long_bridges.len() == 0` AND no `DetectorSkipped` issue from LongBridge (silent skip per §6.2). SLS has `max_overhang_angle = 90°`, so the slab's downward faces still have `overhang_angle = 90°` which is NOT > 90° → no overhang flag under SLS either. Asserting `validation.overhangs.len() == 0` under SLS is a clean tech-skip cross-check.
9. `!validation.is_printable()` under FDM (driven by LongBridge Critical + ExcessiveOverhang Critical).

**Visuals**:
- `out/mesh.ply` — the H-shape, hand-authored boolean-union solid. Camera: from the +Y side looking at the X-Z plane. Visual: two pillars at the bottom, slab on top, slab cantilevers slightly past each pillar's outside edge.
- `out/issues.ply` — 4 points: middle bridge centroid at (15, 2.5, 18), three overhang centroids — left cantilever (-1.25, 2.5, 18), middle bridge (15, 2.5, 18), right cantilever (31.25, 2.5, 18). The middle-bridge centroid coincides with the LongBridge midpoint (so two centroids overlap; that's expected — a multi-detector co-flag).

**README pitfalls**:
- "Why does ExcessiveOverhang also fire?" The lintel's bottom face is 100% overhang — both detectors see it. They're complementary signals, not duplicate.
- f3d-equivalence callout: the lintel's bottom face is visible from below; if a viewer's culling discards back-faces, it may appear missing. Note prominently in README.

**Implementation slot**: lands after the §6.2 LongBridge detector commit.

---

### §7.3 `printability-trapped-volume` — Gap H demonstration

**Detector**: TrappedVolume (`check_trapped_volumes`) via exterior flood-fill.

**Concept**: A solid cube with a sealed spherical cavity inside. The example demonstrates technology-aware **TrappedVolume severity divergence**: SLA/SLS/MJF flag Critical (uncured-material trap), FDM flags Info (sealed cavity prints fine for FDM). A separate co-flag also surfaces — the cavity ceiling fires ExcessiveOverhang Critical under all techs (any sealed cavity inherently has a face whose normal tilts downward; Gap M correctly flags it). The README documents the co-flag as a design-pattern: "validators see surface geometry, not interior intent — sealed-cavity ceilings flag as overhang regardless of designer intent."

**Fixture geometry**:

A solid 20×20×20 mm cube containing a sealed sphere cavity of radius 5 mm centered at (10, 10, 10). Watertight + consistently wound.

**Construction approach** (impl-time choice, in preference order):
1. **Hand-author** 12 outer cube triangles + a tessellated sphere (32 segments × 16 stacks ≈ 1024 triangles) with REVERSED winding so sphere normals point INTO the cavity. Concatenate. Total ~1036 triangles. Tractable; deterministic; bounded ray-tri budget for §6.3 scanline.
2. **`mesh-offset::marching_cubes(grid, config)`** on a closed-form `max(box_sdf, -sphere_sdf)` sampled to a `ScalarGrid`. mesh-offset's MC is the public, L0, workspace-internal MC entry point (mesh-sdf has no public MC API; mesh-lattice's MC is private). Adds `mesh-offset = { workspace = true }` to the example crate's `[dependencies]` per §7.0. mesh-offset is L0 (same tier as mesh-printability) so the example's dep graph stays layer-clean. Trade-off vs option 1: MC produces a chamfered (planar-faceted) sphere — see `feedback_chamfered_not_rounded` — which is faithful to MC semantics but visually distinct from a UV-tessellated sphere. The §7.3 README's Visuals section already calls out the chamfered look downstream, so this is consistent.

Hand-author (option 1) recommended; cap sphere tessellation at ≤32 segments to keep §6.3's ray-tri scanline budget under 2 s on the reference machine (200³ voxel grid × ~1000 triangles → ~ 4×10⁷ ray-tri tests; well within target). Option 2 (mesh-offset MC) is the documented fallback if hand-authoring the 1024-triangle tessellated sphere proves error-prone at impl time.

**Multi-config**: validate at FDM, SLA, SLS, MJF and assert each technology's severity policy.

**Expected output (all four techs)**:
- `validation.trapped_volumes.len() == 1`. Cavity centroid ≈ (10, 10, 10) within `voxel_size`. Volume ≈ 523.6 mm³ ± 10%.
- TrappedVolume severity: **Info** for FDM (sealed cavity prints fine for FDM); **Critical** for SLA/SLS/MJF (uncured-material trap).
- ExcessiveOverhang Critical (cavity ceiling co-flag) — fires for all 4 techs since the sphere has a face whose normal points downward (`dot ≈ -1`, `overhang_angle ≈ 90°`, well past 75° Critical band for any tech with `max_overhang_angle ≤ 60°`). For SLS (`max=90°`) and MJF (`max=90°`), the predicate `overhang_angle > 90°` is false (90 NOT strictly > 90) → NOT flagged.

| Tech | `max_overhang` | Cavity ceiling overhang | TrappedVolume severity | `is_printable()` |
|------|----------------|--------------------------|-------------------------|------------------|
| FDM | 45° | Critical (90 > 75) | Info | `false` (overhang Critical) |
| SLA | 30° | Critical (90 > 60) | Critical | `false` (both Critical) |
| SLS | 90° | NOT flagged (90 NOT > 90) | Critical | `false` (TrappedVolume Critical) |
| MJF | 90° | NOT flagged (90 NOT > 90) | Critical | `false` (TrappedVolume Critical) |

**Pedagogical takeaway**: each tech fails the validation, but for different reasons. FDM fails on overhang (cavity ceiling unsupported during print); SLA/SLS/MJF fail primarily on trapped uncured material; SLS+MJF have lenient overhang policy so the cavity ceiling escapes overhang-flagging there.

**Assertion list** (4 sub-runs):
1. For each tech in `[FDM, SLA, SLS, MJF]`:
   - `validation.trapped_volumes.len() == 1`
   - Region center within `voxel_size` of `(10, 10, 10)`
   - `approx::assert_relative_eq!(volume, 523.6, max_relative = 0.10)`
2. **FDM**: TrappedVolume severity `Info`; ExcessiveOverhang severity `Critical`; `is_printable() == false` (driven by overhang).
3. **SLA**: TrappedVolume severity `Critical`; ExcessiveOverhang severity `Critical`; `is_printable() == false` (both Critical).
4. **SLS**: TrappedVolume severity `Critical`; `validation.overhangs.len() == 0` (lenient `max=90°`); `is_printable() == false` (TrappedVolume).
5. **MJF**: TrappedVolume severity `Critical`; `validation.overhangs.len() == 0`; `is_printable() == false` (TrappedVolume).

Voxel-size note (documentation, not an assertion): `voxel_size = min(min_feature_size, layer_height) / 2` per §6.3. For FDM defaults that's `min(0.8, 0.2)/2 = 0.1 mm` → 200³ grid (~8 MB) for the 20mm cube; <2 s on reference machine.

**Visuals**:
- `out/mesh.ply` — solid cube with internal cavity. Outer surface: cube. Inner cavity surface: sphere (only visible in cross-section). f3d may render only the outer cube (the inner sphere's normals point inward, so are back-facing from the camera). MeshLab's "show edges" + slice plane reveals the internal sphere clearly. **f3d-callout near top of README** per `feedback_f3d_winding_callout`.
- `out/issues.ply` — single point at (10, 10, 10) (cavity centroid).
- `out/voxels.ply` — point-cloud of trapped voxel centers (one point per trapped voxel; ~4200 voxels for the sphere at FDM voxel size). Useful for visualizing the discretized cavity shape; demonstrates the voxel-fill algorithm.

**README pitfalls**:
- The voxel grid in `out/voxels.ply` is tens of thousands of points. Render as point-cloud (small point size); too many for face-coloring.
- The chamfered-vs-filleted distinction (per `feedback_chamfered_not_rounded`): the SDF-derived sphere surface, when MC-tessellated coarsely, looks **chamfered** (planar facets), not **smooth/filleted**. Use that wording in the README.

**Implementation slot**: lands after the §6.3 TrappedVolume detector commit.

---

### §7.4 `printability-self-intersecting` — Gap I demonstration

**Detector**: SelfIntersecting (`check_self_intersecting`) via mesh-repair re-use.

**Concept**: Two cylinders intersecting like a plus sign, **without** running boolean union — they overlap geometrically without sharing topology. mesh-repair flags every face pair where one cylinder's surface punches through the other's. Demonstrates `validate_for_printing` re-using mesh-repair's intersection routine.

**Fixture geometry**:

Cylinder A: axis along +X, length 30 mm, radius 5 mm, centered at origin (oriented from x=-15 to x=15).
Cylinder B: axis along +Y, length 30 mm, radius 5 mm, centered at origin.
The two cylinders interpenetrate at the origin — every face pair where A's lateral surface crosses B's lateral surface is a self-intersection. **Each cylinder is independently watertight + consistently wound**, but the union (without boolean cleanup) has dozens of intersecting face pairs.

Constructed via two independent `mesh_types::cylinder(radius, length, segments)`-style helpers concatenated with vertex-index offset (no welding, no boolean union — that's the point of the fixture).

**FDM config**:
- `PrinterConfig::fdm_default()` — params don't affect SelfIntersecting; severity is always Critical.

**Expected output**:
- `validation.self_intersecting.len() >= 4` (lower bound; exact count depends on cylinder tessellation; with 16-segment cylinders, expect 8–24 intersection pairs near the four interpenetration "rings" where A and B's surfaces cross).
- `validation.self_intersecting.len() <= 100` (mesh-repair's `max_reported = 100` cap; if the cylinder tessellation is fine enough to exceed 100, the description says "(search truncated; total may be higher)" and the cap is hit — assert the description string).
- All SelfIntersecting issues are `Critical`.
- `validation.is_printable() == false`.
- **ExcessiveOverhang co-flag**: each cylinder's lateral surface has a "downhill arc" of faces that flag under Gap M (cylinders are placed with axes horizontal = perpendicular to up; lateral normals span all directions perpendicular to axis, so the bottommost lateral arc has `overhang_angle > 45°`). The bottom-most ring of lateral faces (touching `mesh_min_along_up`) is build-plate-filtered; faces just above it are not. Result: `validation.overhangs.len() >= 1` per cylinder. The example documents this as a co-flag (cylinders printed flat have unavoidable overhang on the underside).

**Assertion list**:
1. `validation.self_intersecting.len() >= 4` — "interpenetrating cylinders must produce at least 4 face-pair intersections"
2. All entries' `face_a < face_b` (canonical ordering per §6.4)
3. All entries' `approximate_location` is within ±5 mm of origin (the actual intersection region is a 3D cross at origin, post-`place_on_build_plate` shifted up so origin maps to (0, 0, 5))
4. All SelfIntersecting issues are `Critical`; `validation.is_printable() == false`
5. `validation.overhangs.len() >= 1` (cylinder lateral overhang co-flag, post-Gap-M); not the load-bearing assertion but documented to keep the example's expected output complete.
6. **Cleaned-up regression**: re-run after removing cylinder B entirely (single cylinder = no self-intersections); assert `validation.self_intersecting.len() == 0`. This shows the validator's role as an upstream gate for boolean repair, and demonstrates that the lateral-overhang co-flag is a separate concern (still present on a single cylinder).

**Visuals**:
- `out/mesh.ply` — two interpenetrating cylinders. Camera: from above (+Z); two crossed cylinders form a plus sign. Render with edges visible to see the interpenetration.
- `out/issues.ply` — ≥4 points clustered around the four interpenetration "rings" near the origin (where each cylinder's lateral surface crosses the other's).

**README pitfalls**:
- "Couldn't I just call `mesh_repair::detect_self_intersections` directly?" Yes — and the validator does internally. The example shows it's available through the higher-level `validate_for_printing` API. Cite the §3 re-export of `IntersectionParams` for power users.

**Implementation slot**: lands after the §6.4 SelfIntersecting detector commit.

---

### §7.5 `printability-small-feature` — Gap J demonstration

**Detector**: SmallFeature (`check_small_features`) via connected-component bbox extent.

**Concept**: A main body (a 30 mm cube) plus a tiny isolated cylindrical burr (0.2 mm diameter × 0.2 mm height) floating 1 mm away — a CAD leftover from an imperfect boolean cut. The burr is a separate connected component below `min_feature_size`; the main body passes. Realistic CAD failure mode.

**Fixture geometry**:

Component 1: a 30×30×30 mm solid cube at origin, z ∈ [0, 30] (sits on build plate). 12 triangles. Watertight, consistent winding.
Component 2: a tiny **hexagonal prism** (6-segment "cylinder") representing a CAD burr left from a cut at z=0, with bottom hexagon vertices on a circle of radius 0.1 mm at z=0, height 0.2 mm so top hex is at z=0.2. **Burr sits on the build plate** alongside the cube but topologically disjoint. Burr center at (35, 15, 0.1). Authored via 6 lateral quads (= 12 triangles) + 2 hex caps fan-triangulated from each cap centroid (6 + 6 = 12 triangles). Total: **24 triangles, 14 vertices** (12 outer hex vertices + 2 cap centroids). Watertight, consistent winding.

The two components share **no vertices, no edges, no faces** — they're geometrically and topologically disjoint. Concatenated with vertex-index offset; the resulting mesh has 2 connected components (per the §6.5 union-find).

**Why on-plate placement matters**: under post-Gap-M, a floating burr's bottom face would flag as ExcessiveOverhang Critical (downward-facing, not on build plate). Placing the burr ON the build plate (`z_min = 0`, matching the cube's `z_min = 0` so `mesh_min_along_up = 0`) ensures the burr's bottom face is build-plate-filtered → no overhang. The example's load-bearing concept stays clean (only SmallFeature Warning fires; `is_printable() == true`).

**Hexagonal-prism volume (lock-in)**: a regular hexagon with circumradius `r = 0.1 mm` has area `A = (3√3/2)·r² ≈ 2.5981 × 10⁻² mm²`. Volume = `A × h = 2.5981e-2 × 0.2 ≈ 5.196 × 10⁻³ mm³`. The cylinder formula `π r² h ≈ 6.283 × 10⁻³ mm³` is the smooth-cylinder limit; for a 6-segment polygon it's an over-estimate. Use the **prism volume** for the assertion.

**FDM config**:
- `PrinterConfig::fdm_default()` — `min_feature_size = 0.8 mm`. Burr's `max_extent = 0.2 mm < 0.8/2 = 0.4 mm` → severity Warning. (`max_extent` is the longest bbox side; for the prism that's `max(2r, h) = max(0.2, 0.2) = 0.2 mm`.)

**Expected output**:
- `validation.small_features.len() == 1` — main cube (30 mm extent) is too big to flag; only the burr.
- `region.center ≈ (35, 15, 0.1)` within `1e-6` (FP-exact: vertex positions are exact-representable).
- `region.max_extent` ∈ `[0.199, 0.201]` (exact-representable in principle; range tolerates triangulation artifacts).
- `region.face_count == 24` (locked-in by the construction approach above).
- `approx::assert_relative_eq!(region.volume, 5.196e-3, max_relative = 1e-3)` — divergence-theorem volume of the hex prism. (Tolerance reflects FP roundoff in the cross-product sum, not the cylinder-formula approximation.)
- One `PrintIssue` of type `SmallFeature` with severity `Warning` (0.2 < 0.8/2 = 0.4).
- `validation.overhangs.len() == 0` — both cube bottom and burr bottom are build-plate-filtered; no other downward faces in the fixture.
- `validation.is_printable() == true` — SmallFeature Warning ≠ Critical, so `is_printable()` is **not** blocked. Document this in README (helps users distinguish "warning to inspect" from "blocking error").

**Assertion list**:
1. `validation.small_features.len() == 1` — "exactly one small-feature region (the burr; main cube must NOT flag)"
2. Centroid within `1e-6` of `(35, 15, 0.1)`
3. `region.face_count == 24` — "hex prism face count locked in by construction"
4. `region.max_extent` within tolerance of 0.2
5. `region.volume` within `1e-3 max_relative` of `5.196e-3`
6. Severity `Warning`
7. `validation.overhangs.len() == 0` — "build-plate filter excludes both component bottoms"
8. `validation.is_printable() == true` — "Warning severity does not block is_printable"

**Visuals**:
- `out/mesh.ply` — large cube + tiny burr sitting next to it on the build plate. Camera: from above and slightly to the side; the cube fills most of the view; the burr is barely visible at default zoom (it's only 0.2 mm tall vs the cube's 30 mm). Zoom to (35, 15, 0.1) at 100× to see it. README camera notes call out this scale ratio.
- `out/issues.ply` — one point at (35, 15, 0.1).

**README pitfalls**:
- "Why doesn't the main cube flag?" Its `max_extent` = 30 mm >> `min_feature_size` = 0.8 mm. The detector is bbox-extent-based.
- "What if I scale the model 1000× by mistake (mm vs m)?" The main cube becomes 30 m and the burr becomes 0.2 m — neither flags as small. Different detector concern (build-volume-exceeded would fire instead). Note this as a v0.9 followup for a unit-detection heuristic.

**Implementation slot**: lands after the §6.5 SmallFeature detector commit.

---

### §7.6 `printability-orientation` — Gap L demonstration

**Detector**: orientation infrastructure (`find_optimal_orientation` + `evaluate_orientation`) parametrized by `PrinterConfig::build_up_direction`.

**Concept**: A leaning column — a tilted cylinder oriented at 60° from vertical. Validating with default `+Z up` flags massive overhang (the column's "leaning" direction is 60° from vertical, well past 45° threshold). Validating after running `find_optimal_orientation` rotates the column to lay flat → minimal overhang. Then validating with `with_build_up_direction(Vector3::new(0,1,0))` (+Y up) on the original mesh reproduces the same rotated result without rotating the mesh — demonstrating that build-up direction parametrization is equivalent to mesh rotation.

**Fixture geometry**:

A cylinder of radius 5 mm, length 30 mm, axis aligned along the direction `(sin(60°), 0, cos(60°))` ≈ `(0.866, 0, 0.5)`, base centered at origin so the column "leans" 60° from vertical in the +X direction. Watertight, consistent winding.

**Three runs** (in main()):

1. **Default `+Z up`** with original mesh — large overhang area along the lateral cylinder surface (60° tilt > 45° threshold).
2. **`find_optimal_orientation`** + apply rotation — cylinder lies flat; overhang area drops to near-zero (laterally none; only the two circular end caps which are 90° from the new "up" — but those are wholly above the build plate after `place_on_build_plate`, so they don't flag as overhang either).
3. **`with_build_up_direction(Vector3::new(0.866, 0, 0.5))`** with original mesh — equivalent to "the +(0.866, 0, 0.5) direction is 'up'" — overhang area drops to near-zero (the cylinder's lateral surface is now parallel to the build-up direction, no overhang).

Cross-check: runs 2 and 3 should produce **the same overhang area within 1e-6** — same physical setup, two equivalent ways to express it (rotate the mesh OR rotate the up-vector).

**FDM config**:
- `PrinterConfig::fdm_default()` for run 1.
- `PrinterConfig::fdm_default()` (default `+Z up`) for run 2 after rotation.
- `PrinterConfig::fdm_default().with_build_up_direction(...)` for run 3.

**Expected output (post-Gap-M convention with build-plate filter)**:

Each Run applies `place_on_build_plate` to the validated mesh so `mesh_min_along_up = 0`; this is the canonical evaluation frame.

**Run 1 (default `+Z up`, no rotation)**: Lateral cylinder normals span `dot ∈ [-0.866, 0.866]` (per the convention derivation; max-downward dot = -sin(60°) = -0.866). Under Gap M predicate `overhang_angle > 45°`:
- Lateral normal at `dot = -0.866`: `overhang_angle = 60°` → flagged.
- Lateral normal at `dot = -0.707` (= -sin(45°)): `overhang_angle = 45°` → NOT flagged (strict-greater-than).
- Lateral normals with `dot > -0.707` (the upward-leaning half of the lateral surface): NOT flagged.
- Bottom end cap (normal = -axis ≈ (-0.866, 0, -0.5), `dot = -0.5`): `overhang_angle = 30°` → NOT flagged. (Also irrelevant: any face on the build plate would be filtered.)
- Top end cap (normal = +axis = (0.866, 0, 0.5), `dot = +0.5`): `overhang_angle = -30°` → NOT flagged.

For 32-segment cylinder lateral tessellation (~64 lateral triangles): roughly 12 of them flag (the strip on the downhill side). Each lateral triangle area ≈ (2π·r·length / 64) = (2π·5·30 / 64) ≈ 14.7 mm². Flagged area ≈ 12 × 14.7 ≈ 176 mm². Build-plate filter: lateral faces' min vertex z > 0 in general (lateral faces span the full cylinder; their lowest vertex is at z=0 only for faces immediately at the base ring). Conservatively, ~10 lateral faces touch z=0 (bottom ring); these get filtered. Net flagged: ~10 × 14.7 ≈ 150 mm² (post-filter estimate; exact count depends on tessellation).

**Run 2** (`find_optimal_orientation` + `apply_orientation` + `place_on_build_plate`): the orientation search includes a 45°-rotation around Y (in `generate_sample_orientations`'s 12-sample default), which maps the cylinder's leaning axis (0.866, 0, 0.5) close to +Z (post-rotation axis ≈ (0.259, 0, 0.966)). The exact -60°-around-Y is NOT in the sample set, but the closest sample (45°-around-Y) gets axis 15°-from-up. Under Gap M with FDM `max=45°`:
- Lateral normals (after 45° rotation) span `dot ∈ [-sin(15°), +sin(15°)] = [-0.259, +0.259]`. Max `overhang_angle ≈ 15°`. NOT flagged.
- Top end cap (post-rotation): dot ≈ +0.966, NOT flagged.
- Bottom end cap (post-rotation): dot ≈ -0.966. `overhang_angle = 75°` → 75 > 45 → flagged → BUT on build plate after `place_on_build_plate` → filtered.
- Run 2 overhang area = **0**, `validation.overhangs.len() == 0`.

(Caveat: which rotation `find_optimal_orientation` picks depends on the score function `support_volume + overhang_area*0.1`. Multiple rotations give score 0 if they reduce overhang to 0; the implementation picks the FIRST in the sample list with min score. The 45°-around-Y rotation is one such; identity is not. Verified at impl time by inspecting `result.rotation`.)

**Run 3** (`PrinterConfig::fdm_default().with_build_up_direction(Vector3::new(0.866, 0, 0.5).normalize())` on original mesh, `place_on_build_plate` first): the up-vector aligns with the cylinder's leaning axis. Lateral normals are perpendicular to axis = perpendicular to new up: `dot = 0`, `overhang_angle = 0`, NOT flagged. Top cap normal aligned with new up: `dot = +1`, NOT flagged. Bottom cap normal anti-aligned with new up: `dot = -1`, `overhang_angle = 90°`, flagged → BUT bottom cap is at minimum-along-new-up (axis-pos 0), so `face_min_along_up - mesh_min_along_up = 0` → filtered.
- Run 3 overhang area = **0**, `validation.overhangs.len() == 0`.

**Severity reasoning** (Run 1, downhill faces): the maximum observed `overhang_angle` is ~60° (faces with `dot ≈ -0.866`). Per §4.3:
- Critical: angle > 75°. 60 > 75? No.
- Warning: 60° < angle ≤ 75°. 60 > 60? Strict-greater-than. No.
- Info: 45° < angle ≤ 60°. 60 ≤ 60 → yes. **Info severity** for Run 1's lateral-arc cluster.

So Run 1's overhang issue is Info, not Critical. `is_printable()` is NOT blocked by Run 1's overhang alone. The example documents this — `is_printable()` is **printed but not asserted** for Run 1.

**Assertion list (lock-in)**:
1. Run 1: `validation.overhangs.len() >= 1`; total overhang area ∈ `[100, 250]` mm² (analytical "downhill arc" minus build-plate-filtered ring ≈ 150 mm²).
2. Run 1: ExcessiveOverhang severity `Info` (per §4.3 boundary calculation above).
3. Run 1: print `is_printable()` to stdout but do NOT assert (depends on whether other detectors fire; a cylinder has no thin walls, no cavity, no self-intersections, no small features, so `is_printable() == true` is expected — confirm by print, not by assertion).
4. Run 2: `validation.overhangs.len() == 0`; total overhang area = 0 within `1e-6` mm².
5. Run 3: `validation.overhangs.len() == 0`; total overhang area = 0 within `1e-6` mm².
6. **The Gap L invariant**: `assert_relative_eq!(run2_overhang_area, run3_overhang_area, epsilon = 1e-6)` — trivially 0 = 0; semantically Gap-L's load-bearing claim.
7. Run 2 overhang area STRICTLY LESS than Run 1 overhang area (`assert!(run2_area < run1_area)`; orientation search reduces support need; 0 < ~150 holds).
8. All three Runs print `overhangs.len()` and total overhang area to stdout.

**Visuals**:
- `out/mesh_original.ply` — leaning cylinder pre-validation (post-`place_on_build_plate`; min_z = 0).
- `out/mesh_rotated.ply` — cylinder after `find_optimal_orientation.rotation` applied + `place_on_build_plate`. Axis-up (or near-axis-up post-search); cylinder stands on its bottom end-cap.
- `out/issues_run1.ply` — point cloud of overhang-region centers from Run 1 (likely 1–2 points on the downhill strip).
- `out/issues_run2.ply`, `out/issues_run3.ply` — empty point-clouds (zero overhang); demonstrates the "no issues = no points" rendering.

**README pitfalls**:
- "Run 2 and Run 3 give the same answer — why have both?" Because they're **architecturally different**: Run 2 modifies the mesh; Run 3 modifies the config. For workflows that don't want to rotate the mesh (preserves authoring frame, allows comparing orientations without rebuilding the mesh), Run 3 is the right tool. The example demonstrates both equivalence and the use-case difference.
- The chamfered note (`feedback_chamfered_not_rounded`): the cylinder is tessellated with 32 segments; lateral surface looks **chamfered** at coarse zoom, not smooth-curved. Use that wording in README.

**Implementation slot**: lands after the §5.6 Gap L parametrization commit (and after `find_optimal_orientation` is verified to consume `config.build_up_direction` — F10 in §2 of the spec).

---

### §7.7 `printability-technology-sweep` — cross-tech behavior on same mesh

**Detector**: all detectors, exercised with four different `PrinterConfig`s on the same mesh.

**Concept**: A part with both a **thin wall** AND a **sealed cavity** is validated under FDM, SLA, SLS, MJF. The output diverges — same geometry, four different printability verdicts — demonstrating technology-aware severity (TrappedVolume tech mapping in §4.3) and threshold differences (`min_wall_thickness` is 1.0 for FDM vs 0.4 for SLA vs 0.7 for SLS vs 0.5 for MJF).

**Fixture geometry (simplified — single box-cavity serves both ThinWall and TrappedVolume)**:

A double-walled hollow box at outer dimensions 25×20×15 mm; inner cavity 22×17×13.1 mm with the inner cavity's top raised so the top wall thins to **0.4 mm** (same construction as §7.1, scaled). Side wall thicknesses 1.5 mm; bottom wall 1.5 mm; top wall 0.4 mm. The inner box-cavity is itself the trapped volume — one cavity serves both detector demos. Watertight, consistent winding.

Construction approach: hand-author 12 outer triangles + 12 inner triangles (24 total, 16 unique vertices), per §7.1 pattern.

**Wall-thickness lock-in: 0.4 mm** — chosen so each technology's threshold lands on a different severity band per §4.3.

**Expected output (per technology)** — the inner top face IS the cavity ceiling, so the ThinWall cluster on inner top and the cavity-ceiling overhang share that face. Each is a SEPARATE detector flag (different `validation.<field>`):

| Tech | `min_wall` | `max_overhang` | ThinWall (wall=0.4) | TrappedVolume | Cavity-ceiling overhang | `is_printable()` |
|------|-----------|----------------|---------------------|---------------|--------------------------|------------------|
| FDM | 1.0 mm | 45° | 2× **Critical** (0.4 < 0.5) | Info | **Critical** (90 > 75) | `false` |
| SLA | 0.4 mm | 30° | 2× NOT flagged (0.4 ≥ 0.4) | **Critical** | **Critical** (90 > 60) | `false` |
| SLS | 0.7 mm | 90° | 2× **Warning** (0.35 ≤ 0.4 < 0.7) | **Critical** | NOT flagged (90 NOT > 90) | `false` |
| MJF | 0.5 mm | 90° | 2× **Warning** (0.25 ≤ 0.4 < 0.5) | **Critical** | NOT flagged (90 NOT > 90) | `false` |

(The "2×" notation reflects the §7.1-analyzed two-cluster outcome from edge-adjacency: outer-top and inner-top clusters. Cavity-ceiling overhang is a single cluster on the inner-top face.)

**Each tech flags differently — none give a green check, but each blocks for a different reason.** The example's load-bearing surface is the per-tech severity matrix, not a single "FDM-vs-SLA" binary outcome.

**Assertion list (4 sub-runs)**:
1. **FDM**: `thin_walls.len() == 2` with severity `Critical` each; `trapped_volumes.len() == 1` with severity `Info`; `validation.overhangs.len() == 1` (inner-top cluster as cavity ceiling) with severity `Critical`; `is_printable() == false`.
2. **SLA**: `thin_walls.len() == 0` (strict-less-than boundary); `trapped_volumes.len() == 1` with severity `Critical`; `validation.overhangs.len() == 1` (inner-top cavity ceiling) with severity `Critical`; `is_printable() == false`.
3. **SLS**: `thin_walls.len() == 2` with severity `Warning` each; `trapped_volumes.len() == 1` with severity `Critical`; `validation.overhangs.len() == 0` (lenient `max=90°`); `is_printable() == false` (TrappedVolume).
4. **MJF**: `thin_walls.len() == 2` with severity `Warning` each; `trapped_volumes.len() == 1` with severity `Critical`; `validation.overhangs.len() == 0`; `is_printable() == false` (TrappedVolume).

**Visuals**:
- `out/mesh.ply` — the part.
- `out/issues_fdm.ply`, `out/issues_sla.ply`, `out/issues_sls.ply`, `out/issues_mjf.ply` — region centroids per tech. The visual story: same mesh, different point-cloud overlays per tech. (Aspirational v0.9: render them as a single multi-layered PLY with per-point tech-tag attribute. Out of v0.8 scope.)

**README pitfalls**:
- The boundary cases (e.g., wall=0.4 with SLA min_wall=0.4 → not flagged because the comparison is strict-less-than): document the strict-less-than convention; the README's table reproduces the lock-in.
- "Why does every tech fail this part?" Because the cavity (Critical for SLA/SLS/MJF) and the thin wall (Critical for FDM) make this part unprintable on every common technology. Note that this is a **deliberate fixture choice** to demonstrate divergence — a real CAD pipeline would design around these constraints.

**Implementation slot**: lands after **all** detectors are landed (§7.7 exercises ThinWall + TrappedVolume together; both must be shipped first).

---

### §7.8 `printability-showcase` — capstone with multi-issue mesh

**Detector**: all six detectors (incl. existing overhang/manifold), in one fixture.

**Concept**: A small mounting bracket — a realistic CAD part — with deliberately-introduced printability concerns: a 60° overhang on a side wing, a 0.4 mm thin lip near the mounting hole, a small isolated burr from a boolean cut, and a sealed internal cavity. Demonstrates the "summary report" capability — shows all the detectors firing at once on a realistic part, and how the user reads the multi-detector output (severity-sorted issues list).

**Fixture geometry**:

A simplified bracket — kept tractable to hand-author + minimize fixture complexity, while still feeling like a real CAD part:

- **Body**: 50×30×10 mm rectangular base.
- **Wing**: an attached L-extension 30 mm tall, leaning at 60° from vertical (overhang concern).
- **Thin lip**: a 0.4 mm thin top wall on the base (same construction technique as §7.1; scaled to bracket dimensions).
- **Burr**: a tiny isolated hexagonal-prism component (≈ 0.2 mm extent) placed 5 mm from the bracket — same as §7.5's burr fixture.
- **Sealed cavity**: a 4 mm-radius sphere cavity inside the base — acknowledged as not physically realistic for a bracket; document as a fixture contrivance to exercise TrappedVolume in the showcase.

**Construction approach**: hand-author each component (~120 triangles total: 12 base + 12 wing + 24 thin-lip extras + 24 burr + ~1024 sphere), concatenate. NOT via mesh-shell (which is offset-based, not CSG). Total fixture build is ~1100 triangles; deterministic and reproducible.

**FDM config**:
- `PrinterConfig::fdm_default()`.

**Expected output**: multiple issues, mixed severities. The example's main() walks `validation.issues`, prints them sorted by severity (per §4.4), and asserts:

**Assertion list**:
1. `validation.thin_walls.len() >= 1` — "thin top wall must flag"
2. `validation.small_features.len() >= 1` — "burr must flag"
3. `validation.trapped_volumes.len() >= 1` — "sealed cavity must flag"
4. `validation.overhangs.len() >= 2` — "expect at least: (a) leaning wing at 60° tilt (Info severity, 60° NOT > 60° strict so Info-band per §4.3), (b) cavity ceiling at 90° tilt (Critical severity, 90° > 75° Critical band)".
5. `validation.issues.len() >= 5` — sum of typed-issue records (≥5 covers 1 ThinWall + 1 SmallFeature + 1 TrappedVolume + 2 ExcessiveOverhang).
6. At least 2 `Critical` issues → `is_printable() == false`. Sources of Critical: (i) ThinWall on the 0.4mm lip (0.4 < 1.0/2 = 0.5); (ii) ExcessiveOverhang on cavity ceiling (90° > 45°+30° Critical band).
7. Issues sorted: validate first issue's severity ≥ second's, etc. (per §4.4 sort policy).
8. `println!("{}", validation.summary())` to stdout — visible to the user reading the terminal.
9. The bracket's main body itself is **not flagged** as a SmallFeature (50mm extent >> min_feature_size).

**Visuals**:
- `out/mesh.ply` — the bracket. Camera: 3/4 view from above-front. Look for the leaning wing, the thin lip near mounting holes, the burr.
- `out/issues.ply` — point-cloud of all region centroids (one point per region, all detectors). Density of points ≈ 4–6.

**README pitfalls**:
- "Why is the cavity in a bracket?" Acknowledged as fixture-contrivance. The example demonstrates multi-detector reporting; a real bracket would not have an internal cavity. Documented.
- The chamfered note: the SDF-composed bracket has chamfered features at every boolean edge (per `feedback_chamfered_not_rounded`); call out in README.

**Implementation slot**: lands LAST in the v0.8 example sequence, after all detectors and the orientation example. Acts as the "everything works together" smoke test.

---

### §7.9 Resolved decisions

User confirmed master-architect call on Q1–Q7 (2026-04-30). Resolutions captured below; the body of §7 reflects each resolution.

#### Q1 (resolved): "Cross-section-sweep" → renamed to "technology-sweep"

**Resolution**: §7.7 named **`printability-technology-sweep`** (interpretation c — same mesh, four technologies, demonstrates tech-aware severity divergence). The "cross-section-sweep" naming is dropped. Geometry-sweep and orientation-sweep alternatives are deferred (orientation-sweep is partially covered by §7.6; geometry-sweep is v0.9 candidate if a user wants severity-band visualization).

#### Q2 (resolved): AttributedMesh face-attributes deferred to v0.9

`AttributedMesh` currently supports per-vertex attributes only. v0.8 ships with point-cloud `issues.ply` (region centroids only); README prose carries the visual story. Triple-tracked at PR-close. Trigger for v0.9 work: (i) user reports the centroid point-cloud is insufficient for visual review, or (ii) a non-printability use case for face-attributes lands and the same plumbing serves both.

#### Q3 (resolved): showcase included in v0.8

§7.8 (`printability-showcase`) ships in v0.8. Realistic-part fixture is the strongest counter-evidence to "the detector examples are all toy fixtures". Cost is ~1 hour of authoring; if v0.8 scope ever needs to slip, drop the showcase first — per-detector examples are non-negotiable.

#### Q4 (resolved): eight examples is the right scope

Five detectors → five examples (one-concept-per-example per `feedback_simplify_examples` + `feedback_museum_plaque_readmes`). Gap L → orientation example. Tech-sweep → cross-tech severity divergence. Showcase → integration semantics. Cutting any would either skip a detector or hide a load-bearing v0.8 surface. Pause-for-visuals-pass cadence at ~10 min/example × 8 = ~80 min of user attention; sustainable per `feedback_patience_track_record`.

#### Q5 (resolved): f3d-callout placement

Top-of-README callout in §7.1, §7.3, §7.6, §7.8 (load-bearing for visual interpretation). Inline note in Visuals section for §7.2, §7.4. None for §7.5, §7.7 (no load-bearing winding-pair concern).

#### Q6 (resolved): per-commit workspace-members updates

Each example's commit adds its `examples/mesh/printability-<name>` to the workspace `[workspace.members]` list AND lands the example as one atomic commit. `cargo build --workspace` post-commit confirms.

#### Q7 (resolved): Gap M added to v0.8 scope (with M.2 build-plate filter sub-fix)

The v0.7.0 overhang flagging predicate is inverted from FDM convention; fix in v0.8 as **Gap M** (specced in §5.9). Convention lock-in: `overhang_angle = acos(dot) - π/2` (signed deviation of normal from build-plane); flag if `overhang_angle > max_overhang_angle_rad`. Pure roofs (`dot=-1`) → `overhang_angle = 90°` → flagged. Vertical walls (`dot=0`) → `overhang_angle = 0` → not flagged.

**M.2 build-plate filter**: §7 review surfaced a follow-on consequence — under the corrected predicate, every solid object's bottom face (`dot=-1`) would flag as Critical overhang, including the cube-on-plate of §7.3/§7.5/§7.7/§7.8. The v0.7.0 buggy predicate accidentally avoided this by not flagging fully-downward faces. Gap M adds a build-plate filter (matching §6.2 LongBridge pattern: `(face_min_along_up - mesh_min_along_up) < EPS_GEOMETRIC`) so faces resting on the build plate are excluded — they're supported by the plate, not overhangs. Without M.2, every example in §7 would fail with spurious Critical overhangs.

**Cavity-ceiling co-flag**: a separate behavioral consequence — any sealed internal cavity inherently has a "ceiling" face whose normal points downward (`dot ≈ -1`, `overhang_angle ≈ 90°`). Under Gap M this fires Critical overhang for techs with `max_overhang_angle ≤ 60°` (FDM, SLA), regardless of designer intent. §7.1, §7.3, §7.7, §7.8 each document this as a multi-detector co-flag in the README rather than try to suppress it. v0.9 followup (triple-tracked at PR-close): "interior face detection — distinguish cavity-internal faces from exterior faces". Trigger: a user requests cavity-aware overhang severity.

§7.2, §7.6, §7.8 overhang predictions are specific (post-Gap-M with build-plate filter) rather than soft-landed. The Run-2 ≡ Run-3 invariant in §7.6 is the load-bearing claim regardless.

§1 gap inventory updated with Gap M (incl. M.2 sub-fix in description); §2 added F15 finding; §5.9 holds the per-fix spec including build-plate filter logic; §5.10 holds CHANGELOG creation. Concept-level commit-order is A → M → B → D → E → F → L → K → DetectorSkipped → CHANGELOG → C → G → H → I → J → §7 examples in detector order. **Three reconciliations supersede this letter-level sequence**: (a) Gap A is split into three sub-commits 1a/1b/1c per §5.1 — the row-numbered canonical order is §12.1, where 1a/1b/1c occupy the first three slots; (b) Gap K is bundled into the v0.8.0 release commit (§12.1 row #24 per §12.0 row #4), not the pre-DetectorSkipped slot the letter-sequence reads; (c) CHANGELOG is created inside Gap A's commit 1c (per §12.0 row #2), not a separate slot. §12.1 is canonical for any commit-row-number reference; the A→...→J letter sequence above is preserved for the gap-dependency-graph reading only.

---

## §8. Risk inventory

This section applies `feedback_risk_mitigation_review`'s "what could go wrong" lens systematically. Three layers:

- **§8.1 Per-gap implementation risks** — one entry per gap A–M plus DetectorSkipped + CHANGELOG, naming the failure mode + its mitigation already in §4–§7.
- **§8.2 Cross-cutting risks** — risks that span multiple gaps: FP drift, ABI surface, perf cliffs, test-anchor fragility.
- **§8.3 Per-commit blast radius** — for each commit in the locked order (§7.9, with cadence from §7.0), lists the files modified + downstream consumers affected + test-anchor risk.

Risk-tier classification (used in the right-hand column of each table):

- **High**: a defect would either silently produce wrong validation results or block the arc from progressing; pre-flight verification is mandatory.
- **Medium**: a defect would surface as test failure or compile error; recoverable with a targeted fix in the same commit.
- **Low**: a defect would be cosmetic or trivially diagnosed; rolling forward is fine.

The §7-review-pass methodology (`project_mesh_printability_gaps.md`) — read each section, check math, verify cross-references, look for code-vs-prose inconsistencies, audit assertions for pass-by-coincidence — applies at every implementation commit, not just at spec-authoring time. This section pre-loads the lens.

### §8.1 Per-gap implementation risks

#### Gap A — Workspace lints

| Risk | Mitigation | Tier |
|------|-----------|------|
| Clippy fallout exceeds the 10-site cap, forcing a bundled src-cleanup that violates `feedback_pr_size_ci_economics` | §5.1 stop-and-raise gate; if heavy, revert lints flip and surface to user | Medium |
| Workspace lint surface drifts after Gap A but before later commits (e.g. a new clippy lint promoted at a Rust-toolchain bump mid-arc) | Each post-Gap-A commit re-runs `cargo clippy -p mesh-printability --tests --all-targets -- -D warnings` per §10's per-commit gate; toolchain pinning via `rust-toolchain.toml` (already in repo) prevents surprise upgrades | Low |
| Fixing a clippy fallout site (e.g. unwrap → expect) silently changes panic message, breaking a test that asserts on panic strings | Pre-flight before commit 1a (the first Gap A sub-commit): `grep -rn 'should_panic.*expected' mesh/mesh-printability/src/ mesh/mesh-printability/tests/` should return nothing; if not, the affected test gets its expected-string updated alongside the fallout fix in the appropriate sub-commit (1a for src-side panics; 1b for test-site panics) | Low |

#### Gap M — Overhang predicate fix + build-plate filter

| Risk | Mitigation | Tier |
|------|-----------|------|
| New predicate flags faces the v0.7-using downstream consumers (umbrella, examples, regression test) didn't previously see, breaking their `is_printable()` assertions | §3 documents semver-significant behavioral change; CHANGELOG entry; §5.9's regression sweep checks every existing `validation.rs::tests` assertion that touches overhang-derived numerics | High |
| Build-plate filter mis-fires due to FP drift in `face_min_along_up - mesh_min_along_up` calculation | EPS_GEOMETRIC = 1e-9 mm chosen well above f64 ULP for mm-scale geometry; `dot(&up)` with `up.normalize()` upstream guarantees normalized projection; cross-platform smoke via §10's CI matrix | Medium |
| `mesh_min_along_up` accidentally placed inside the face loop instead of computed once before it → O(n × n_vertices) instead of O(n) | §5.9 code shape places the computation explicitly before the face loop; commit-time read-aloud check; §6.2 LongBridge has the same pattern, cross-reference verifies | Low |
| `validation.rs` and `orientation.rs` predicates drift over time (one fixed, one missed) | §5.9 explicitly calls for symmetric edits in both files; commit message names both files; risk-mitigation review pass at commit time greps both for `dot < 0.0` to confirm both removed | Medium |
| Cavity-ceiling co-flag confuses downstream users post-v0.8 (they didn't see overhang flags on sealed cavities pre-v0.8; now they do) | README docs in §7.1, §7.3, §7.7, §7.8 explicitly call this out; CHANGELOG `[0.8.0] / Fixed` section names the convention change; mesh-book §50 depth-pass migration | Medium |
| Strict-greater-than vs greater-or-equal boundary disagreement (§5.9 specifies strict; an implementer flips it) | §5.9 test `test_overhang_45deg_tilt_borderline_not_flagged` locks in strict semantics with explicit failure message | Low |

#### Gap B — Track actual maximum overhang angle

| Risk | Mitigation | Tier |
|------|-----------|------|
| `max_overhang_angle_rad` reads the pre-Gap-M predicate in tests run before Gap M lands | Locked commit order: M lands before B (§7.9); §5.2 acceptance gates use post-Gap-M semantics by definition since they're tested after both land | Low |
| Test fixture's "expected steepest face angle" computed by hand differs from FP result | All §5.2 tests use exact-representable trig values (e.g. 30°/45°/60°/90°) via `std::f64::consts`; tolerance via `approx::assert_relative_eq!(.., epsilon = 1e-6)` per §4.5 | Low |
| Empty-overhang case (no flagged faces): `max_overhang_angle_rad` reads zero-init; downstream Gap E severity classifier sees angle=0 and emits Info | §5.2 spec: `OverhangRegion` is only created when `overhang_faces.is_empty() == false`; helper short-circuits before reading the max; §5.2 test `test_overhang_no_overhang_no_region` asserts no region in this case | Low |

#### Gap D — Split overhangs into connected regions

| Risk | Mitigation | Tier |
|------|-----------|------|
| Refactor of edge-map building (extract `build_edge_to_faces`) silently changes `check_basic_manifold` semantics — open-edge or non-manifold counts shift | §5.3 refactor regression gate: existing `test_not_watertight_detection` + `test_watertight_mesh` + `test_validation_summary` + `test_issue_counts` + `test_sls_no_overhang_check` continue to pass without modification; refactor commit lands BEFORE Gap D so the diff is reviewable in isolation | Medium |
| Union-find off-by-one → faces in the wrong cluster | Adversarial fixtures: `test_overhang_two_disjoint_regions` (asserts 2 regions, face-disjoint), `test_overhang_face_adjacency_via_shared_edge` (asserts vertex-only-shared faces are NOT adjacent) | Low |
| Component centroid definition (mean-of-face-centroids vs mean-of-vertex-positions vs area-weighted) drifts from spec | §5.3 algorithm step 4: "centroid (average of face centroids)"; lock-in via `test_overhang_region_centroid_is_component_centroid` with a single-face fixture where mean-of-face-centroids equals mean-of-vertex-positions trivially; multi-face case verified by adversarial fixture | Low |
| `support_regions.len()` 1:1 invariant vs `overhangs.len()` (existing test_support_volume_calculation) breaks once overhangs split into components | §5.3 acceptance criterion explicitly: "support_regions length matches overhangs length (one support region per overhang patch)"; existing test re-uses the cube-on-plate fixture which produces zero overhangs in either form | Medium |
| `total_support_volume()` numerical anchor shifts because overhang_area sums differently across split components vs single region | Per §5.3 acceptance: "sum of per-region volumes equals total" — invariant preserved by construction (sum of areas across split components equals the unsplit total) | Low |

#### Gap E — Tighten ExcessiveOverhang severity policy

| Risk | Mitigation | Tier |
|------|-----------|------|
| Existing test asserts `is_printable()` on a fixture with a moderate overhang and now flips to false | §5.4 spec: `test_sls_no_overhang_check` continues to pass (gating at function level, not severity); commit-time sweep of all `is_printable()` assertions in existing tests; any flip means a fixture's overhang is severe enough to be a real issue and the fixture should be updated to a milder slope | Medium |
| Boundary cases at exactly threshold + 15° / threshold + 30° (= comparison) | §5.4 spec: strict-less-than semantics on the upper bound (`<= threshold + 15`), strict-greater on the lower (`> threshold`); §4.3 documents; tests `test_overhang_severity_*_at_*` cover each band including borderline | Low |
| `classify_severity` central helper drift between `check_overhangs` and post-Gap-D region-emit code | §5.4 spec: helper is the single source of policy; both call sites consume it | Low |

#### Gap F — Winding orientation in manifold check

| Risk | Mitigation | Tier |
|------|-----------|------|
| Existing `create_watertight_cube` fixture's winding is silently inconsistent — Gap F reveals it and breaks `test_watertight_mesh` | §5.5 explicit regression test: `test_winding_consistent_watertight_cube` asserts the fixture has consistent winding; pre-flight verification: read `create_watertight_cube` source pre-commit, confirm CCW-from-outside | Medium |
| Two distinct issue descriptions ("N open edge(s)" + "N edge(s) traversed in same direction") collide on a single mesh, doubling the issue count and breaking existing `test_issue_counts` | §5.5 explicit: undirected edge-count + directed edge-count run side-by-side; existing fixtures with ≥1 open edge but consistent winding emit ONLY the open-edge issue, not both | Low |
| HashMap iteration order non-determinism in directed-edge counting changes the descriptive count's tie-breaking on cross-platform | Counts are scalar (just `.values().filter(...).count()`); descriptions don't enumerate which edges; deterministic | Low |

#### Gap L — `PrinterConfig::build_up_direction` parametrization

| Risk | Mitigation | Tier |
|------|-----------|------|
| `with_build_up_direction(Vector3::zeros())` panics in release builds (no debug_assert in release) → caller crashes | §5.6 doc-comment on `with_build_up_direction`: "panics in debug builds; NaN propagates in release"; explicit responsibility on caller; covered by `test_build_up_direction_zero_panics_in_debug` | Medium |
| `find_optimal_orientation` rotates mesh in mesh-frame; `config.build_up_direction` is dotted against rotated normals — sign conventions confusable | §5.6 spec: "rotation is applied in mesh-frame, so the rotated normals are dotted against `config.build_up_direction` (whatever frame the caller chose)"; `test_find_optimal_orientation_respects_build_up_direction` covers the cross-frame symmetry | Medium |
| All four `*_default()` constructors update; one is missed | §5.6 spec lists all four; commit-time grep verifies; `test_build_up_direction_default_is_z` per-tech | Low |
| Build-up direction not normalized → dot products yield non-unit projections, severities and angles inconsistent | `with_build_up_direction` calls `.normalize()` internally; `*_default()` constructors set `(0,0,1)` (already unit); `test_build_up_direction_normalized_in_builder` covers | Low |

#### Gap K — COMPLETION.md rewrite

| Risk | Mitigation | Tier |
|------|-----------|------|
| COMPLETION.md drifts from code (claims a detector is populated when it isn't, or vice versa) | §5.7 acceptance: "every public-API listing matches `pub use` in lib.rs; every detector listed populates its claimed type/severity in tests"; lands LAST so truth is settled | Low |
| Test counts in COMPLETION.md become stale at next test add | Counts are pulled from `cargo test -p mesh-printability` output at the commit; a programmatic grep-and-assert against test count would over-couple the doc to test discovery — manual update is cheap; `cargo doc` lint catches malformed markdown but not stale numerics | Low |

#### DetectorSkipped variant

| Risk | Mitigation | Tier |
|------|-----------|------|
| Workspace consumer matches exhaustively on `PrintIssueType` and breaks compile when the variant adds | §5.8 acceptance: `cargo build --workspace` post-commit verifies; only known consumers of the enum are within `mesh-printability` itself + the umbrella's regression test (no exhaustive match in either) | Medium |
| Caller conflates DetectorSkipped with Other | §3 doc-comment + §5.8 doc-comment make the distinction explicit; `as_str()` returns "Detector Skipped" not "Other" | Low |
| DetectorSkipped emitted twice for the same detector when it has multiple precondition fails (e.g. ThinWall: not watertight + winding inconsistent) | §6.1 spec: ThinWall checks "watertight AND consistent_winding" as a single combined precondition; emits one DetectorSkipped with both reasons in the description, not two issues | Low |

#### CHANGELOG.md

| Risk | Mitigation | Tier |
|------|-----------|------|
| CHANGELOG drifts from commits — features added without entries, or entries describe behavior that didn't ship | §5.10 per-commit append requirement; commit-time review pass includes "did the CHANGELOG line up with the diff?" | Low |
| `[0.8.0]` date pin in final commit gets the wrong date if the arc spans multiple days | Final commit pins via `date -u +%Y-%m-%d` at commit-author time; user-visible commit message includes the date | Low |

#### Gap C — `check_thin_walls`

| Risk | Mitigation | Tier |
|------|-----------|------|
| Möller-Trumbore self-hit on co-planar adjacent faces (ray exits face F's edge into face G that shares the edge) | EPS_RAY_OFFSET = 1e-6 mm offsets ray origin inward by 1µm; `tri-tri co-planarity` epsilon in M-T routine handles edge cases; `test_thin_wall_concave_z_shape` exercises | Medium |
| O(n²) ray-cast is too slow on the 5k-tri budget — perf-cliff regression in CI | §6.1 documents <1s for 5k tris; CI tests-debug uses small fixtures (the §7.1 hollow-box has 24 triangles); 10k+ tri fixture would only appear if a §9 stress test introduces it (decided in §9) | Medium |
| Ray-tri returns `Some(t)` for negative `t` (ray hit behind origin) → mis-flagged as thin wall | M-T returns `Some(t)` only when `t > 0` per §6.1; private helper signature: `Option<f64>` where `Some(t)` is `t > 0`; algorithm step 2 explicitly guards `t > 0 and t < min_dist` | Low |
| Cluster split — adjacent thin-wall faces in different "shell layers" should be one cluster, get split because edge-adjacency is intra-layer only | §6.1 spec uses edge-adjacency on flagged faces only; §7.1 lock-in expects 2 clusters for the hollow-box (outer-top + inner-top) for exactly this topological reason; pre-flight verification in §7.1 main() asserts the 2-cluster outcome explicitly | High |
| Watertight-precondition skip masks a genuine ThinWall in an open mesh | §4.1 + §6.1 documented: open meshes get DetectorSkipped, no `ThinWallRegion`s; user filters issues for DetectorSkipped to enumerate which detectors ran | Low |
| Concave geometry: first hit is the nearest opposite face, but for thick → thin → thick layouts the wrong face is "opposite" (ray skips through the cavity) | §6.1 edge case + `test_thin_wall_concave_z_shape` covers the Z-shape geometry; pathological 3-shell case deferred to v0.9 | Medium |

#### Gap G — `check_long_bridges`

| Risk | Mitigation | Tier |
|------|-----------|------|
| `requires_supports()` semantic for `PrintTechnology::Other` — caller-extension hook; we return `true` (conservative) but this could surprise | Existing semantic (re-confirmed at commit time); §6.2 silently skips for SLS/MJF only; FDM/SLA/Other run | Low |
| Diagonal-bridge underflagging (axis-aligned bbox vs true diagonal) | §6.2 documented as v0.9 OBB followup; `test_long_bridge_diagonal_underflagged_documented` locks in the v0.8 behavior | Low |
| Cantilever flagged as bridge | §6.2 documented as v0.9 cantilever-detection followup; `test_long_bridge_cantilever_currently_flagged` locks in v0.8 behavior | Low |
| Build-plate filter false-positive (face barely above plate excluded incorrectly) | EPS_GEOMETRIC = 1e-9 mm vs minimum bridge height of ~layer_height (0.2mm for FDM) — no overlap | Low |
| ANGLE_TOL_HORIZONTAL = 30° classifies a 25° tilt as bridge candidate; user expected only ~horizontal | Documented tolerance; v0.9 candidate is "tighter classification per technology"; for v0.8, 30° tolerance is consistent with FDM-slicer convention (PrusaSlicer "support overhang" classifies bridges at ~80° from vertical = within 10° of horizontal, but our 30° tolerance is conservative for "near-horizontal") | Low |
| Cluster contains both true-horizontal faces + ~30° tilted faces; bbox conservatively over-projects → false-positive bridge | §6.2 edge case documented; cluster comprises faces within tolerance, but bbox is over horizontal-plane-only of the projected vertices; in practice all near-horizontal faces project onto horizontal plane similarly | Low |

#### Gap H — `check_trapped_volumes`

| Risk | Mitigation | Tier |
|------|-----------|------|
| Voxel inside-test cross-platform divergence — same input produces different `inside` flags on macOS/Linux/Windows due to FP drift in scanline ray-tri intersections (Phase 4 lesson on faer block-diagonal LDLᵀ) | §4.5 + §6.3: tolerance-based volume assertion `max_relative = 0.10` (10% voxel-discretization noise band); ROW_JITTER = 1e-5 mm offsets ray origin to break ties; CI matrix coverage (cross-os tests-debug) catches divergence | High |
| Voxel grid memory blowup on > 100mm parts at FDM voxel size (~250³ ≈ 15.6 MB) → OOM in tight CI runners | §6.3 documents perf ceiling; §7.3 + §7.7 fixtures are 20-25 mm cubes (~200³ ≈ 8 MB, well below ceiling); §9 stress-test gauntlet decides whether to add a 100mm fixture | High |
| Pinhole leak smaller than `voxel_size` voids trapped detection silently | §6.3 documented as intentional (features below printer resolution shouldn't false-flag); README of §7.3 calls this out | Low |
| Scanline parity flips on a vertex hit (ray enters and exits at the same voxel boundary) | §6.3 edge-case: parity uses bottom-edge inclusive / top-edge exclusive standard convention; ROW_JITTER mitigates exact coincidences | Medium |
| `voxel_size = (min_feature.min(layer_height)) / 2` produces a non-FP-exact value (e.g. min_feature=0.4, layer=0.2 → voxel=0.1) → `0.1` is FP-irrational, accumulating drift across grid steps | All grid coordinates derived as `i * voxel_size + grid.x_start`; `i * 0.1` for i up to 250 carries ULP drift but stays within 1e-13 cumulative — well below EPS_GEOMETRIC; verified by `test_trapped_volume_volume_within_10pct_of_analytical` tolerance | Medium |
| Sphere fixture in §7.3 with reversed winding sphere accidentally watertight-but-orientation-inconsistent → ThinWall (post-Gap-F) flags winding inconsistency, blocking ThinWall but not affecting TrappedVolume | §6.3 precondition is watertight only (not winding); §7.3 verification at impl time checks `validation.issues` for absence of NonManifold/winding-inconsistency issues; if winding is wrong, fix the fixture not the detector | Medium |
| Volume estimate via voxel-count × voxel_size³ has 10% noise band; assertion that test fixture's volume ≈ analytical 523.6 mm³ within 10% leaves room for a 50 mm³ off-by-one bug | 10% is the documented voxel-discretization noise band; tighter tolerance would over-constrain; an off-by-100% bug would still fail (`max_relative = 0.10` catches anything outside ±10%) | Low |

#### Gap I — `check_self_intersecting`

| Risk | Mitigation | Tier |
|------|-----------|------|
| `mesh-repair::detect_self_intersections` API surface drift mid-arc (mesh-repair has its own followup work in §10's CI matrix) | Pin via workspace dep `mesh-repair = { path = "../mesh-repair" }` (workspace-local, not crates.io); same workspace = same revision | Low |
| `IntersectionParams::default()` semantics differ from what mesh-printability assumes (e.g., default changes to `max_reported = 1000` upstream) | §3 acceptance: `IntersectionParams::default()` documented inline as `max_reported = 100, epsilon = 1e-10, skip_adjacent = true`; if mesh-repair changes these defaults, the change surfaces in the v0.8 test count or behavior — §6.4 tests catch | Medium |
| `mesh-repair` returns intersection pairs in non-canonical (a > b) order; we canonicalize but might miss a case | §6.4 explicit canonicalization; `test_self_intersecting_face_indices_unique_per_pair` asserts `face_a < face_b` for all reported | Low |
| Spec assumes mesh-repair returns a `result.truncated` boolean field for max_reported overflow; mesh-repair's actual field name may differ (`is_truncated`, `was_truncated`, etc.) | Pre-flight at §6.4 commit: read `mesh-repair::intersect::SelfIntersectionResult` field list; update §6.4 spec + Gap I commit's truncation-description code inline if name differs | Medium |
| `IntersectionResult::intersecting_pairs` could grow to 100 entries even on a clean mesh with adjacent triangles slightly miscoded | §6.4 spec uses `IntersectionParams::default()` which has `skip_adjacent = true`; clean watertight cube produces empty pairs (covered by `test_self_intersecting_clean_cube_no_issue`) | Low |
| Adding mesh-repair as dep introduces a transitive dep that violates L0 dep-count cap (Tier::L0 release_max = 80, test_max = 100; checked via grader Criterion 6 Layer Integrity, see `xtask/src/grade.rs:2118`) | Pre-flight at Gap I commit: run `cargo xtask grade mesh-printability --skip-coverage` and confirm "6. Layer Integrity" criterion remains A. mesh-repair already grades A as L0 (its release count < 80); mesh-printability currently has 4 direct deps; combined unique-dep count post-merge stays comfortably under 80 (mesh-repair + mesh-printability share `nalgebra`/`hashbrown`/`mesh-types` transitively, so dedup keeps it tight). If grader fails Layer Integrity, this would block Gap I — stop and raise | High |

#### Gap J — `check_small_features`

| Risk | Mitigation | Tier |
|------|-----------|------|
| Single-component mesh entirely below threshold (e.g. user has unit-conversion error) → main body flagged as small feature, not a useful diagnostic | §6.5 documented edge case; surfaces as a signal, user decides; `feedback_no_reflexive_defer` — don't suppress the diagnostic | Low |
| Vertex-only "adjacency" (two faces sharing only a vertex) treated as same component — drift from edge-adjacency definition | §6.5 spec: "edge-adjacency only"; `test_small_feature_face_adjacency_via_edge_only` asserts vertex-only-shared faces produce separate components | Low |
| `signed_volume` returns negative on outward-winding (correct sign) but the abs() in §6.5 takes absolute value — a flipped-winding component reports same volume as correctly-wound | abs() is the documented choice for tolerance; "approximate for non-watertight inputs" doc-comment | Low |
| Face indices in `region.faces` reference shared edge-map → if union-find IDs are reordered between runs, region face order changes | §4.4 sort policy: `small_features` sort by `face indices, smallest face index first`; deterministic | Low |
| Spike (long thin protrusion) escapes detection because max_extent > threshold | §6.5 documented as v0.9 followup; volume-based criterion candidate | Low |
| Open / non-watertight sub-component's signed_volume returns junk; user reads `region.volume` and treats it as authoritative | §6.5 documentation: "approximate for non-watertight inputs"; `region.face_count` + `region.max_extent` are the authoritative fields | Low |

### §8.2 Cross-cutting risks

These risks span multiple gaps. Each has at least one mitigation declared in §4 or earlier; this section consolidates the audit trail for risk-mitigation review.

#### §8.2.1 Cross-platform FP drift (Phase 4 / faer-LDLᵀ lesson generalized)

The Phase 4 work surfaced bit-exact assertion fragility in faer block-diagonal LDLᵀ: same input on macOS/Linux/Windows produced ULP-level different outputs, breaking frozen-baseline regression tests. mesh-printability does not invoke faer, but **ray-cast distances (Gap C), voxel scanline inside-tests (Gap H), and signed-volume integrals (Gap J)** all carry FP-drift potential.

| Detector | Operation at risk | Mitigation |
|----------|-------------------|------------|
| Gap C (ThinWall) | Möller-Trumbore ray-tri intersection: `t` parameter sensitivity to vertex coordinates | Tolerance assertion `epsilon = 1e-6` per §4.5; exact-representable test vertex coords (1.0, 2.0, 0.5 — not 0.1) where possible |
| Gap H (TrappedVolume) | Scanline ray-tri parity in voxel inside-test | ROW_JITTER = 1e-5 mm; tolerance-based volume assertion `max_relative = 0.10`; CI cross-os matrix runs (`cross-os` job in `quality-gate.yml`) catches divergence |
| Gap J (SmallFeature) | `signed_volume` divergence-theorem sum across faces: cross-product accumulation | Tolerance `max_relative = 1e-3`; exact-representable fixture vertex coords for the unit-cube fragment in `test_small_feature_volume_via_divergence_theorem` |
| Gap M (Overhang) | `dot(&up)` + `acos()` chain | Tolerance `epsilon = 1e-6`; `up.normalize()` upstream; cross-os matrix catches |

**Recovery if a CI cross-os divergence does surface**: increase tolerance band, document the precision floor in the affected detector's doc-comment, file v0.9 followup. **Do NOT** add platform-specific code paths in v0.8.

#### §8.2.2 ABI stability — `PrintValidation` field additions

Strict semver: adding `pub` fields to `PrintValidation` is a breaking change because struct-update syntax (`PrintValidation { issues, ..existing }`) breaks. Practical: §2 audit (finding F14) confirmed no consumer uses struct-update — only `PrintValidation::new()` + field-mutation patterns.

| Risk | Mitigation | Tier |
|------|-----------|------|
| A consumer added struct-update mid-arc (e.g. another team adds a new test using struct-update) | Pre-flight at the commit immediately after `PrintValidation` field additions: `cargo build --workspace`; `grep -r "PrintValidation {" src/ examples/ tests/` | Medium |
| External (non-workspace) consumer breaks because we ship 0.8.0 to crates.io | mesh-printability not yet published per repo state; bump is workspace-local; revisit at first crates.io publish | Low |

#### §8.2.3 Test-numerical-anchor cascade fragility

Gap M alters the overhang-flagging predicate; Gap D splits regions; Gap E re-classifies severities; Gap B re-defines the reported angle. Cumulatively, **every test that asserts on `validation.overhangs[*].angle`, `validation.overhangs.len()`, `total_support_volume()`, `validation.issues.len()`, or `validation.is_printable()` may shift**.

| Existing test name (validation.rs::tests) | Field touched | Risk |
|--------------------------------------------|---------------|------|
| `test_empty_mesh_error` | error path on empty input | Low (error path unchanged) |
| `test_no_faces_error` | error path on faceless input | Low (error path unchanged) |
| `test_build_volume_check` | `ExceedsBuildVolume` issue | Low (build-volume detector untouched) |
| `test_build_volume_ok` | absence of `ExceedsBuildVolume` | Low (build-volume detector untouched) |
| `test_not_watertight_detection` | NotWatertight issue | Low (open-edge logic unchanged) |
| `test_watertight_mesh` | NonManifold absence + winding (Gap F) | Medium — Gap F may newly flag winding if fixture is wrong |
| `test_validation_summary` | `validation.summary()` string | Medium — depends on overhang count |
| `test_issue_counts` | `severity_counts` | Medium — Gap E severity tightening + Gap D splitting may change counts |
| `test_sls_no_overhang_check` | overhang gate at function level | Low — gate at function level preserved |
| `test_support_volume_calculation` | `total_support_volume()` | Medium — Gap D's split + Gap M's predicate change shift area sum, but `area * 5.0` heuristic is invariant under partition |

**Mitigation cadence**: each commit's acceptance criteria runs `cargo test -p mesh-printability` post-edit. A failing existing test gets either (a) numerical-anchor refresh if the test exercises the new convention correctly, or (b) re-fixture if the test depended on the buggy convention's specific output. Gap M's commit (per §5.9) is the load-bearing sweep — most cascading anchor shifts happen there.

**Strategy for `test_watertight_mesh`** specifically (Gap F): pre-flight read `create_watertight_cube` source; verify CCW-from-outside winding by inspection (cross product of edge vectors against face normal); if fixture is wrong, fix in Gap F's commit (one-line vertex-list reorder).

#### §8.2.4 Performance cliffs

| Detector | Cliff threshold | Symptom | Mitigation |
|----------|-----------------|---------|-----------|
| Gap C (ThinWall) | >10k tris | Validation runtime > 5 s on reference machine | §6.1 doc; v0.9 BVH followup; largest v0.8 example fixture is §7.8 showcase (~1100 tris) — well below cliff |
| Gap H (TrappedVolume) | >100mm part extent at FDM voxel size | Memory > 64 MB; runtime > 5 s | §6.3 doc; v0.9 adaptive voxel; v0.8 examples ≤25mm |
| Gap I (SelfIntersecting) | >10k tris with many intersections | Runtime > 5 s | §6.4 doc; mesh-repair already uses bounding-box culling + rayon |

CI matrix (per `quality-gate.yml`): `tests-debug` runs on default features, debug build, 23 workspace crates. For mesh-printability, debug runtime can be 2-5x slower than release. The §7 examples use `--release` per `feedback_release_mode_heavy_tests`; v0.8 unit tests must complete in debug mode under typical 30-min CI matrix budget.

#### §8.2.5 Fixture topology assumptions (transferred from §7-review-pass findings)

The §7 review surfaced that hand-authored fixtures can have non-obvious topological consequences:

| Fixture / Example | Risk | Mitigation |
|-------------------|------|-----------|
| §7.1 hollow box: outer + inner are topologically disjoint | ThinWall produces 2 clusters, not 1, due to edge-adjacency operating per-component | §7.1 lock-in: explicit 2-cluster assertion + cluster-z disambiguation |
| §7.2 H-shape boolean union: shared vertices at pillar-slab junctions | Faces sharing only a vertex aren't edge-adjacent → split into 3 cantilever/bridge clusters | §7.2 lock-in: 1 LongBridge (only middle exceeds threshold) + 3 OverhangRegion (Gap-D split) |
| §7.3 sphere-in-cube: reversed sphere winding for inward-facing normals | If winding is wrong, Gap F flags it post-arc; sphere is INSIDE solid material → cavity-ceiling fires under Gap M | §7.3 lock-in: pre-flight winding verification + cavity-ceiling explicit prediction |
| §7.5 burr on plate: bottom-of-burr vs build-plate filter | If burr is off-plate, Gap M flags burr's bottom face → spurious Critical breaking the SmallFeature-only narrative | §7.5 burr at z=0.1 with bottom at z=0; build-plate filter applies |
| §7.6 leaning cylinder: orientation-search sample set discrete (12 samples) | `find_optimal_orientation` may NOT pick the exact-perpendicular rotation; expected output computed against the closest-sample rotation | §7.6 lock-in: 45°-around-Y closest sample analysis |

**Pattern**: hand-authored fixtures get a per-fixture topology audit before the example commit. This is the §7-review-pass methodology applied at impl time.

### §8.3 Per-commit blast radius

Each row corresponds to one commit. Order reconciles §7.9's gap-order with §7.0's cadence ("examples land after the corresponding detector commits") — detectors and their examples are interleaved, not batched. Blast radius = (files modified, downstream consumers affected, test-anchor risk).

**Three §-internal inconsistencies that §12 must reconcile** (flagged for explicit commit-order spec in §12, not silently absorbed here):

1. §5.3 says `build_edge_to_faces` extraction "happens as a separate commit before Gap D" but §7.9's locked-order summary doesn't list it. Row #4 below reflects §5.3's reading.
2. §1 + §5.10 specify CHANGELOG creation in the Gap A slot; §7.9 lists CHANGELOG as a separate slot after DetectorSkipped. Row #1c below reflects §5.10's reading (CHANGELOG.md is created in commit 1c — the only Gap A sub-commit that touches Cargo.toml; subsequent commits #2 onward append; no separate creation slot).
3. §7.0 specifies interleaved detector → example → next-detector cadence; §7.9's "C → G → H → I → J → §7 examples in detector order" reads as batched-after. Rows #11–#22 below reflect §7.0's interleaved reading per `feedback_one_at_a_time`.

**Gap A 3-commit split**: per §5.1's pre-flight measurement (38 fallout sites, 3.8× the >10 stop-and-raise gate), Gap A is split into three sub-commits (1a/1b/1c) in §12.1. §8.3 mirrors that split below — rows #1a/#1b/#1c replace the original row #1. Rows #2 onward keep their existing numbers (§8.3 uses sub-suffix sub-numbering, not cascade renumbering).

§12 will produce the canonical commit-order list and resolve all three readings.

| # | Commit | Files modified | Downstream affected | Test-anchor risk |
|---|--------|----------------|---------------------|------------------|
| 1a | Gap A sub-1 — mechanical-only fallout (19 const_fn + 3 cast + 1 format + 1 match = 24 sites) | per-site src edits in `validation.rs`, `orientation.rs`, `regions.rs`, `config.rs` (no Cargo.toml change) | None | None — `cargo build` gate (clippy not yet inherited) |
| 1b | Gap A sub-2 — test-site cleanup (5 expect_used) | per-site src edits in `validation.rs::tests` (or restructure) | None | None — `cargo test` gate |
| 1c | Gap A sub-3 — `[lints] workspace = true` flip + 9 FP-semantics `#[allow]`s + CHANGELOG.md creation + Coverage baseline capture | `Cargo.toml`, `CHANGELOG.md` (new), 8× `#[allow(clippy::suboptimal_flops)]` in `validation.rs` + `orientation.rs`, 1× `#[allow(clippy::manual_midpoint)]` | None (lints only; FP semantics preserved by per-site allows) | None — clippy gate (now active) |
| 2 | Gap M (overhang predicate + build-plate filter) | `validation.rs`, `orientation.rs`, `validation.rs::tests` | Umbrella regression test, 3 examples (mesh-pipeline, design-to-print, full-pipeline) — verify no overhang-derived numerical anchors | **High** — anchor sweep per §8.2.3 |
| 3 | Gap B (max-angle tracking) | `validation.rs::check_overhangs` | None (region.angle field semantics) | Medium — `OverhangRegion.angle` value shifts |
| 4 | Refactor: `build_edge_to_faces` helper | `validation.rs` (extract helper) | None | None per §5.3 regression gate |
| 5 | Gap D (overhang region split) | `validation.rs::check_overhangs` (consume helper) | None at API level | Medium — `validation.overhangs.len()` shifts |
| 6 | Gap E (severity tightening) | `validation.rs::check_overhangs`, `classify_overhang_severity` helper | None | Medium — `is_printable()` shifts on severe overhangs |
| 7 | Gap F (winding orientation) | `validation.rs::check_basic_manifold` | None | Medium — `test_watertight_mesh` regression risk |
| 8 | Gap L (build_up_direction) | `config.rs`, `validation.rs::check_overhangs`, `orientation.rs` | Umbrella + 3 examples — verify they still compile (PrinterConfig field add) | Low — defaults preserve existing behavior |
| 9 | Gap K (COMPLETION.md rewrite) | `COMPLETION.md` | None | None — doc only |
| 10 | DetectorSkipped variant | `issues.rs` | Umbrella regression test (cargo build --workspace) | Low — variant addition; non-exhaustive matches unchanged |
| 11 | Gap C (ThinWall detector) | `validation.rs::check_thin_walls` (new), `regions.rs`, `lib.rs`, mesh-printability `Cargo.toml` (changelog entry) | Umbrella regression test (new types) | Low — new functionality |
| 12 | §7.1 thin-wall example | `examples/mesh/printability-thin-wall/` (new), workspace `Cargo.toml` (new member) | None | Low — example assertions self-contained |
| 13 | Gap G (LongBridge detector) | `validation.rs::check_long_bridges` (new), `regions.rs`, `lib.rs` | Umbrella regression test | Low — new functionality |
| 14 | §7.2 long-bridge example | `examples/mesh/printability-long-bridge/` (new), workspace `Cargo.toml` | None | Low — example assertions self-contained |
| 15 | Gap H (TrappedVolume detector) | `validation.rs::check_trapped_volumes` (new), `regions.rs`, `lib.rs` | Umbrella regression test | Low — new functionality |
| 16 | §7.3 trapped-volume example | `examples/mesh/printability-trapped-volume/` (new), workspace `Cargo.toml` | None | Low — example assertions self-contained |
| 17 | Gap I (SelfIntersecting detector) | `validation.rs::check_self_intersecting` (new), `regions.rs`, `lib.rs`, mesh-printability `Cargo.toml` (mesh-repair dep) | Workspace dep graph (rayon, tracing, cf-geometry transitive); umbrella regression test | **High (Layer Integrity)** — pre-flight `cargo xtask grade` per §8.4 |
| 18 | §7.4 self-intersecting example | `examples/mesh/printability-self-intersecting/` (new), workspace `Cargo.toml` | None | Low — example assertions self-contained |
| 19 | Gap J (SmallFeature detector) | `validation.rs::check_small_features` (new), `regions.rs`, `lib.rs` | Umbrella regression test | Low — new functionality |
| 20 | §7.5 small-feature example | `examples/mesh/printability-small-feature/` (new), workspace `Cargo.toml` | None | Low — example assertions self-contained |
| 21 | §7.6 orientation example | `examples/mesh/printability-orientation/` (new), workspace `Cargo.toml` | None | Low — orientation infra already in Gap L commit |
| 22 | §7.7 technology-sweep example | `examples/mesh/printability-technology-sweep/` (new), workspace `Cargo.toml` | None | Low — exercise of existing detectors |
| 23 | §7.8 showcase example | `examples/mesh/printability-showcase/` (new), workspace `Cargo.toml` | None | Low — exercise of existing detectors |
| 24 | v0.8.0 release commit | mesh-printability `Cargo.toml` (`version = "0.7.0"` → `"0.8.0"`), `CHANGELOG.md` (close `[Unreleased]` → `[0.8.0]`) | Umbrella version (per §3 option 1: workspace umbrella stays at 0.7.0) | None — version pin only |
| 25 | Spec deletion (final commit, `feedback_code_speaks`) | `mesh/mesh-printability/V08_FIX_ARC_SPEC.md` (deleted) | Memo updates: `project_mesh_printability_gaps.md` rewritten as "v0.8 closed + v0.9 backlog with triggers"; mesh book §50 depth-pass migration | None — doc only |

**High-risk commits** (#2 Gap M, #17 Gap I): pre-flight verification mandated per §8.4. **Medium-risk commits** (#3, #5, #6, #7): test-anchor sweep at commit-time per §8.2.3. **All commits**: thorough re-read pass + risk-mitigation review pass per the cadence (`feedback_thorough_review_before_commit` + `feedback_risk_mitigation_review`).

Per `feedback_one_at_a_time_review`, each example commit (#12, #14, #16, #18, #20, #21, #22, #23) pauses for the user's visuals-pass before the next commit. Eight pause-points across the arc.

### §8.4 Risk-tier response policy

Across §8.1's per-gap tables (15 sub-sections) plus §8.2.2's ABI table, ~67 individual risks are tier-tagged. Distribution:

- **High (5)**: Gap M v0.7 anchor cascade; Gap C cluster-split topology; Gap H cross-platform voxel inside-test; Gap H grid memory blowup; Gap I Layer Integrity dep-count cap.
- **Medium (~20)**: ABI updates, regression gates, manifold-precondition skip, severity boundary cases, refactor regressions, semver hygiene, M-T self-hit, fixture-topology audits.
- **Low (~42)**: Cosmetic, documented v0.9 candidates, panic-message stability, FP-drift on exact-representable inputs, deterministic counts, minor variant handling.

**Response by tier**:

- **High**: pre-flight verification mandated before the commit lands; additional test fixtures if any uncertainty; if a high-risk issue surfaces during impl, stop-and-raise to user before proceeding.
- **Medium**: test-anchor sweep at commit-time; recover via targeted fix in same commit; user not blocked.
- **Low**: roll forward; address only if it surfaces.

Pre-flight verifications consolidated for high-risk gaps:

| Gap | Pre-flight verification |
|-----|--------------------------|
| Gap M | Read final `check_overhangs` + `evaluate_orientation` once more; compute predicate's true semantics; verify FDM-convention match (PrusaSlicer or Cura "support overhang threshold" semantic) |
| Gap C | Pre-flight: §7.1 fixture topology audit; verify `validation.thin_walls.len() == 2` is the correct theoretical outcome before authoring main() assertion |
| Gap H | Pre-flight: cross-os CI dry-run on the §7.3 fixture (manual `cargo test` on macOS + Linux + Windows local VMs if accessible; CI matrix coverage in worst case) |
| Gap I | Pre-flight: run `cargo xtask grade mesh-printability --skip-coverage` after adding mesh-repair dep; verify "6. Layer Integrity" criterion remains A (Tier::L0 release_max=80, test_max=100 per `xtask/src/grade.rs:2118`) |

---

## §9. Stress-test gauntlet

This section enumerates pathological inputs each detector must handle, maps them to **flag / skip / tolerate** outcomes, and locks in stress fixtures that prove the §8.4 High-tier mitigations in code (not in the spec). The §7 examples cover the **happy path + load-bearing concept**; §9 covers **adversarial inputs** that surface bugs the happy-path examples cannot.

Stress fixtures live in **`mesh/mesh-printability/tests/stress_inputs.rs`** (new file, lands incrementally with each detector commit). Fixtures whose inputs require >50 LOC of vertex authoring use a small per-test helper in the same file; no shared `tests/common/` module (per `feedback_simplify_examples`).

### §9.0 Methodology

**Three outcomes** (mapped per detector input):

- **flag** — the detector emits a typed-region entry + `PrintIssue` with the documented severity. The fixture asserts on counts + severity.
- **skip** — the detector emits exactly one `Info`-severity `DetectorSkipped` issue and returns early; no typed-region entries. The fixture asserts on the skip diagnostic + the empty typed-region field.
- **tolerate** — the detector runs to completion without panicking, returns a non-fatal result; the fixture asserts no panic + bounded runtime + sensible (possibly empty) output.

**No "undefined behavior" or "best effort" outcomes**: every fixture commits to exactly one of the three. Per `feedback_no_reflexive_defer`, deferring with "the algorithm just handles it" is not a documented outcome.

**Risk-coverage rule** (§8.4 enforcement): every High-tier risk in §8.4 has at least one stress fixture proving its mitigation works. Tracked at §9.4.

**Performance budget**: all stress fixtures must complete in **debug mode under 30 seconds total** (the `tests-debug` CI job's per-crate share of its 20-min budget). Heavy fixtures (Gap C 5k-tris, Gap H 100mm part, Gap I 100+ intersections) are **`#[cfg_attr(debug_assertions, ignore)]`** per `feedback_release_mode_heavy_tests`, run in `tests-release` and locally. Light stress fixtures (empty mesh, single triangle, degenerate triangles) stay debug-only.

**Determinism**: every stress fixture asserts deterministic output across two consecutive runs of `validate_for_printing` (the `_sort_stable` test pattern from §6). Cross-platform determinism is the §10 cross-os concern; the per-fixture stable-order check guards within-platform regression.

### §9.1 Pathological-input → detector matrix

Each row is a pathological input class. Columns name the expected behavior of each detector under v0.8. Multiple non-tolerate cells per row are intentional — e.g., row 11 (Gap F same-direction faces) flags Manifold AND skips ThinWall AND tolerates TrappedVolume.

**Cell legend**: `flag <Type>` = emits typed-region + issue; `skip` = emits one DetectorSkipped Info; `tolerate` = runs without panic, no flag; `n/a` = upstream check (BuildVolume error path) returns before this detector runs.

| # | Pathological input | BuildVolume | Overhang (Gap M) | Manifold (Gap F) | ThinWall (Gap C) | LongBridge (Gap G) | TrappedVolume (Gap H) | SelfIntersect (Gap I) | SmallFeature (Gap J) |
|---|---------------------|-------------|------------------|-------------------|-------------------|--------------------|------------------------|------------------------|----------------------|
| 1 | Empty mesh (0 verts, 0 faces) | error: `Err(EmptyMesh)` | n/a | n/a | n/a | n/a | n/a | n/a | n/a |
| 2 | Single triangle (3 verts, 1 face) | tolerate | tolerate (1 normal; build-plate filter applies if downward at z=0) | flag NotWatertight (3 open edges) | skip (not watertight) | tolerate (no horizontal cluster, or cluster span < threshold) | skip (not watertight) | tolerate (1 face has no pairs) | tolerate or flag SmallFeature (1 component; flag iff max_extent < min_feature_size) |
| 3 | Degenerate near-zero-area face (3 collinear verts) | tolerate | tolerate (degenerate normal filtered via EPS_DEGENERATE_NORMAL) | flag NotWatertight (degenerate face contributes 3 open edges to count) | skip (not watertight) | tolerate (degenerate face's normal classify-skipped) | skip (not watertight) | tolerate (degenerate triangle's M-T returns None per epsilon) | tolerate (face_count > 0; bbox from verts) |
| 4 | Two triangles near-coplanar, intersecting at narrow angle | tolerate | tolerate | flag NotWatertight (6 open edges, 2 disjoint triangles) | skip (not watertight) | tolerate | skip (not watertight) | flag SelfIntersecting (Möller-Trumbore via mesh-repair within EPS_INTERSECTION = 1e-10) | tolerate (2 components, each tiny — possibly flagged SmallFeature) |
| 5 | 100k-triangle thin shell (watertight, consistently wound) | tolerate | flag if downward normals exceed threshold | tolerate (closed; consistent winding) | flag (perf cliff documented; v0.8 stress fixture uses 5k tri proxy) | tolerate (geometry-dependent) | flag if cavity sealed | tolerate (clean watertight shell typically has no self-intersections) | tolerate (1 component) |
| 6 | Voxel grid at perf cliff (100 mm cube at FDM voxel 0.1 mm = 10⁹ voxels = 1 GB grid) | tolerate (within typical FDM build volume) | tolerate | tolerate | tolerate (tri-count budget separate concern) | tolerate | flag w/ runtime budget; **release-only fixture** at 100×100×30 mm sub-cap variant (~300M voxels) | tolerate | tolerate |
| 7 | Unit-conversion error (30 mm part authored in meters → 0.030-mm coords) | tolerate (0.030 mm fits any build volume trivially) | tolerate (overhang predicate is scale-invariant) | tolerate | (depends on watertightness — solid 0.030-mm cube IS watertight; ray-cast distance ≈ 0.030 mm < 1.0/2 → flag Critical ThinWall) | tolerate (no spans exceed `max_bridge_span` thresholds at this scale) | tolerate (no cavity) | tolerate | flag SmallFeature (entire-mesh component max_extent = 0.030 < 0.8/2 = 0.4 → Warning; user-diagnostic per §6.5) |
| 8 | Mesh entirely below `min_feature_size` (e.g. solid 0.1 mm cube with `min_feature_size = 0.8 mm`) | tolerate (fits) | tolerate (cube on plate; bottom filtered; sides vertical) | tolerate | flag Critical ThinWall (cube IS watertight; ray-cast hits opposite face at 0.1 mm < 1.0/2 = 0.5) | tolerate (no horizontal downward faces above plate) | tolerate (no cavity) | tolerate | flag Warning SmallFeature (max_extent = 0.1 < 0.8/2 = 0.4) |
| 9 | Sub-voxel-resolution opening from cavity to exterior (cavity with channel ≤ voxel_size diameter) | tolerate | flag if cavity-ceiling fires Gap M | tolerate | tolerate (walls thick by construction — see fixture) | tolerate | tolerate; cavity NOT flagged (flood-fill leaks through opening; documented intentional v0.8 behavior per §6.3 — drainage simulation is v0.9) | tolerate (no self-intersection) | tolerate (1 component) |
| 10 | Tied vertex via UV-sphere tessellation (≥3 faces share a single vertex at a pole) | tolerate | flag iff downward-tilted faces present | tolerate (each edge shared by exactly 2 faces; vertex-fan does NOT make an edge non-manifold) | flag (hollow shell ray-cast finds opposite-pole face at distance ≈ 2r — i.e., diameter, NOT thickness; documented limitation, fixture asserts ray-cast returns finite distance, no panic) | tolerate | flag iff interior is sealed | tolerate (clean sphere has no self-intersections per `skip_adjacent`) | tolerate (1 component) |
| 11 | Faces traversed in same direction (Gap F target: watertight cube with one face's vertex order flipped) | tolerate | tolerate | flag NonManifold Critical (post-Gap-F: directed-edge collision → "winding inconsistency" description) | skip (not consistent winding) | tolerate (algorithm is winding-agnostic) | tolerate (watertight by edge-count check; flood-fill runs — TrappedVolume requires watertight only, NOT consistent winding per §6.3) | tolerate | tolerate |
| 12 | Two triangles sharing only a vertex (no shared edge) | tolerate | tolerate | flag NotWatertight (no shared edges → 6 open edges) | skip (not watertight) | tolerate | skip (not watertight) | tolerate (vertex-only contact is NOT an intersection per §6.4 documented edge case; mesh-repair returns empty pairs since neither face penetrates the other) | tolerate (2 components per edge-adjacency definition; each potentially flagged if extent < threshold) |
| 13 | Disconnected duplicate of valid mesh (2 watertight components, each with internal cavity) | tolerate | tolerate (each component contributes individually) | tolerate (both components watertight; no open edges or non-manifold edges in either) | flag (each component independently if walls thin; lock-in via fixture geometry) | tolerate or flag (per geometry) | flag (both cavities; flood-fill exterior reaches both component exteriors via grid-corner seed) | tolerate (cleanly offset duplicates don't intersect) | tolerate (or flag if either component below threshold) |
| 14 | NaN / Inf in vertex coordinates | tolerate (bbox computation propagates NaN; threshold comparison with NaN is false → no flag; v0.7 has no `is_finite` gate) | tolerate (NaN propagates through arccos; threshold comparison is false → no flag; centroid is NaN-poisoned but no panic) | tolerate (edge-count uses integer indexes; immune to coord NaN) | skip (precondition runs first; if precondition pass, M-T returns NaN-poisoned distance; threshold comparison is false → no flag) | tolerate (NaN-poisoned span comparison is false → no flag) | tolerate (voxel inside-test parity is undefined-but-bounded; no panic) | tolerate (mesh-repair returns empty pairs on NaN-poisoned input) | tolerate (NaN-poisoned bbox extent comparison is false → no flag) |

#### Notes on edge-cases that need spec lock-in (not just guidance)

These rows above carry footnote-worthy detail that the matrix can only hint at:

**Row 1 (empty mesh)**: `validate_for_printing` returns `Err(PrintabilityError::EmptyMesh)` per existing v0.7 behavior (`test_empty_mesh_error` already covers); detectors are never invoked. Stress fixture verifies the error path is preserved post-arc.

**Row 7 (unit-conversion error)**: a "mesh in meters" produces a 30 mm × 30 mm × 30 mm part if treated as mm, 30000 m × 30000 m × ... if treated as meters by other downstream tools. mesh-printability has no unit metadata; treats coords as mm by convention. The fixture authors a 0.030-mm cube (= 30 µm = a 30 mm part scaled 1000×); it fits the build volume, has a single component below feature size, and the SmallFeature detector flags it. The example demonstrates the validator surfacing a unit-conversion error, not silently approving a 30 µm part.

**Row 13 (disconnected duplicate)**: TrappedVolume's flood-fill exterior is seeded at the grid corner (always-outside per padded grid). When two disjoint watertight components both have interior cavities, the exterior flood-fill reaches exterior of both; both cavities flag. Stress fixture has 2 separate cubes-with-cavities offset by 30 mm; expects 2 TrappedVolume regions.

**Row 14 (NaN/Inf)**: by §4.5's "tolerance-based assertions, exact-representable fixtures" policy, mesh-printability does not panic on NaN inputs but does NOT promise correct output. Stress fixture asserts **no panic** (the detector returns whatever it returns); the fixture's job is regression-protecting against future code that might unwrap a NaN-bearing `Option` or trigger a debug assert. **Caveat**: if a future detector implementation introduces a `debug_assert!(x.is_finite())`, the stress fixture would need to be updated to skip in debug mode (`#[cfg_attr(debug_assertions, ignore)]`); document this in the fixture's doc-comment.

### §9.2 Per-detector stress fixtures

Each subsection lists fixture names, what they prove, and where they live. Fixture name format: `stress_<gap-letter>_<scenario>` for traceability with the gap inventory.

#### §9.2.1 Build volume + manifold (existing detectors)

These are not part of the v0.8 in-scope gaps but interact with the new detectors via the precondition skip pathway. Stress coverage:

- `stress_existing_empty_mesh_error_preserved`: empty `IndexedMesh`; assert `validate_for_printing` returns `Err(EmptyMesh)`; detectors never invoked. Light. (Row 1.)
- `stress_existing_single_triangle_open_mesh`: 1 face, 3 verts; assert `NotWatertight` issue, ThinWall + TrappedVolume both emit `DetectorSkipped`. Light. (Row 2.)
- `stress_existing_two_faces_vertex_only_shared`: 2 disjoint triangles sharing a vertex but no edge; assert 6 open edges flagged, ThinWall + TrappedVolume skip, no winding-inconsistency flag (no shared edge means no directed-edge collision). Light. (Row 12.)

#### §9.2.2 Gap M (overhang predicate + build-plate filter)

- `stress_m_pure_roof_flagged`: single horizontal face at z=10 with normal `(0,0,-1)` and a supporting box below it; under FDM `max_overhang_angle = 45°`, `overhang_angle = 90°` → flagged Critical (90° > 45° + 30°). Light. **(Mitigates High-tier risk: §8.4 anchor cascade — proves the corrected predicate flags roofs.)**
- `stress_m_solid_on_plate_bottom_filtered`: solid 10 mm cube on build plate (z ∈ [0, 10]); the cube's bottom face has `dot=-1`, `overhang_angle = 90°` but `face_min_along_up = 0 = mesh_min_along_up`; assert `validation.overhangs.len() == 0` (build-plate filter applies). Light. **(Mitigates the M.2 sub-fix risk — proves the filter saves cube-on-plate.)**
- `stress_m_floating_box_bottom_flagged`: same cube lifted to z ∈ [10, 20] with no support; assert `validation.overhangs.len() >= 1` (bottom face flagged Critical). Light. (Companion to the previous fixture.)
- `stress_m_layered_bottoms_filter_correctly`: a tall stack — base box at z ∈ [0, 5] + tower at z ∈ [5, 25] sharing the z=5 interface; `mesh_min_along_up = 0`; tower's bottom face at z=5 has `face_min_along_up - mesh_min_along_up = 5 > EPS_GEOMETRIC` → NOT filtered; flagged Critical. Verifies the filter is mesh-min-relative, not geometric-z-relative. Light.
- `stress_m_y_up_orientation_symmetric`: same fixture as `stress_m_pure_roof_flagged` but rotated 90° around X with `with_build_up_direction(Vector3::new(0, 1, 0))`; assert same flag count + severity (Gap L cross-check). Light.

#### §9.2.3 Gap C (ThinWall)

- `stress_c_clean_thick_box_no_flag`: 30 mm cube with 5 mm walls (effectively solid); `min_wall = 1.0`; assert 0 ThinWall regions. Light.
- `stress_c_open_mesh_skipped`: 5-of-6-face open box; assert one `DetectorSkipped` issue with description containing "ThinWall" + "watertight"; assert `thin_walls.len() == 0`. Light.
- `stress_c_inconsistent_winding_skipped`: watertight cube with one face's vertex order flipped (Gap-F target); assert `DetectorSkipped` issue (ThinWall precondition includes consistent winding); independently assert NonManifold Critical also fires (Gap F detector runs first). Light.
- `stress_c_concave_z_shape`: Z-shaped solid with a 0.5 mm thin section sandwiched between thick sections; expect 1 region for the thin section with thickness ≈ 0.5 mm. Light.
- `stress_c_pole_tied_vertex_sphere`: hollow tessellated UV-sphere (outer radius 5 mm, wall 0.4 mm = inner radius 4.6 mm; 16-segment × 8-stack tessellation, ~256 outer + 256 inner triangles) where each pole has 16 faces fanning from one vertex; concatenate outer+inner with REVERSED inner winding for watertight+consistent topology. Assert: ThinWall flags ≥1 cluster spanning equatorial faces; **all flagged regions report `thickness ∈ [0.39, 0.41]` mm** (within shell-thickness tolerance, NOT diameter). Pole faces have ray-cast distance ≈ 2 × outer_radius = 10 mm (ray exits one pole, hits the opposite pole's far face) → > min_wall_thickness, NOT flagged at poles. Verifies the algorithm doesn't degenerate at fan apexes. Light. (Row 10.)
- `stress_c_two_disjoint_thin_clusters`: two disjoint thin-walled hollow boxes (each 10×10×10 mm with 0.4 mm walls), positioned 20 mm apart along +X. Each box independently watertight + consistently wound. Assert `thin_walls.len()` is 2 × (clusters per box per §7.1 analysis) — i.e., ≥ 4 clusters total (each box's outer-top + inner-top ⇒ 2 × 2 = 4 minimum, possibly more if other walls cluster). Critical severity on each. **(Mitigates §8.4 Gap C cluster-split topology risk via explicit multi-cluster fixture.)** Light.
- `stress_c_5k_tri_perf_budget` (`#[cfg_attr(debug_assertions, ignore)]`): tessellated thin-walled cylinder with ~5000 triangles; assert detector completes in <1 s (release mode); assert non-zero ThinWall count. Heavy. **(Mitigates §8.2.4 Gap C >10k-tri cliff documentation; verifies the 5k baseline.)**

#### §9.2.4 Gap G (LongBridge)

- `stress_g_clean_solid_cube_no_flag`: solid cube on plate; no horizontal downward faces above plate; assert 0 LongBridge regions. Light.
- `stress_g_sls_silent_skip`: same H-shape from §7.2 with SLS config; assert `long_bridges.len() == 0` AND no `DetectorSkipped` issue (silent skip, not announced). Light.
- `stress_g_diagonal_bridge_underflagged`: a 14 × 14 mm horizontal patch at z=10 (axis-aligned bbox extent = 14, true diagonal ≈ 19.8) with `max_bridge_span = 15`; assert 0 regions (documented v0.8 behavior; v0.9 OBB followup catches). Light. (Locks in v0.8 bbox-conservative semantics.)
- `stress_g_cantilever_currently_flagged`: 20 mm cantilever face anchored on one end at z=10; assert 1 region (currently can't distinguish from bridge; v0.9 followup). Light. (Locks in v0.8 limitation.)
- `stress_g_diagonal_underflag_with_y_up`: same diagonal patch with `+Y up` config (rotated); assert same 0 regions. Light.

#### §9.2.5 Gap H (TrappedVolume)

- `stress_h_solid_cube_no_cavity`: 20 mm solid cube; assert 0 TrappedVolume regions; verifies flood-fill correctly classifies entire interior as inside. Light.
- `stress_h_open_mesh_skipped`: 20 mm cube with one face removed; assert `DetectorSkipped` issue, 0 regions. Light.
- `stress_h_subvoxel_opening_not_flagged`: 20 mm cube with a 5 mm-radius spherical cavity, plus a sub-voxel-resolution channel from the cavity to the exterior (channel cross-section ≤ FDM voxel size = 0.1 mm). Assert 0 trapped regions (flood-fill leaks through opening; this is correct topological behavior — the cavity has an opening so it's not trapped — but documents that the printer's drainage capability is a separate concern v0.9 will address). Cube walls remain 7.5 mm thick → ThinWall does NOT flag. Light. **(Documents intentional v0.8 behavior; corresponds to row 9.)**
- `stress_h_sphere_inside_cube_volume_within_10pct`: spherical cavity, analytical volume 523.6 mm³; assert region volume within ±10%. Light. **(Mitigates §8.4 cross-platform voxel FP-drift risk: tolerance band is wide enough to absorb cross-platform ULP variance.)**
- `stress_h_two_disjoint_cavities`: cube with 2 separate spherical cavities; assert 2 regions. Light.
- `stress_h_disconnected_dual_cavity`: 2 separate cubes-with-cavity offset by 30 mm; assert 2 regions (verifies flood-fill exterior reaches both component exteriors via grid-corner seed). Light. (Row 13.)
- `stress_h_voxel_grid_perf_cliff` (`#[cfg_attr(debug_assertions, ignore)]`): 80 mm part with a 5 mm cavity at FDM voxel size 0.1 mm (~ 800³ ≈ 512M voxels would OOM); use 100 mm × 100 mm × 30 mm part instead (~250³ × 750 = 47M voxels ≈ 47 MB) and assert detector completes in <2 s release-mode + memory < 64 MB (documented ceiling). Heavy. **(Mitigates §8.4 grid memory blowup risk — proves the documented 100mm ceiling holds within budget.)**
- `stress_h_voxel_grid_oom_safety`: 200 mm cube at FDM voxel size 0.1 mm would request 2000³ × 1 byte = 8 × 10⁹ bytes ≈ 8 GB. v0.8 must NOT panic / OOM. Mitigation: detector emits `DetectorSkipped` Info with description containing `"voxel grid would exceed 1 GB"` when the dim product × 1 byte > 1 GB. **Spec consequence: §6.3 algorithm needs a memory pre-flight check** — see §6.3 amendment below. Light fixture (assertion only; the cap check returns BEFORE the grid is allocated, so the test runs in <1 ms). **(Mitigates §8.4 grid memory blowup in the worst case — full-build-volume parts at fine voxels.)**

**§6.3 spec amendment** (surfaced by §9.2.5 risk-mitigation review): §6.3 algorithm gets a new step prepended to step 5 (voxel grid allocation):

```
4.5. Memory pre-flight: let total_voxels = grid_dims.0 * grid_dims.1 * grid_dims.2;
     if total_voxels > 1_000_000_000 (1 GB equivalent at 1 byte per voxel):
         emit one DetectorSkipped Info issue with description containing
         "TrappedVolume detection skipped: voxel grid <a>×<b>×<c> at <voxel_size> mm would exceed 1 GB memory budget";
         return early.
```

The 1 GB cap is one byte per voxel (the `u8` voxel state), is well above the §6.3 documented ceiling for typical FDM parts (~16 MB at 100mm-extent FDM defaults), and protects against pathological inputs (>100 mm parts at fine voxel; 250 mm × 200 mm × 220 mm full FDM build volume at 0.1 mm voxel = 11 GB grid) without affecting realistic <100 mm parts. v0.9 followup: adaptive voxel sizing for over-cap parts (already triple-tracked).

**Implementation slot**: §12 commit-order spec for Gap H must reference this amendment in its acceptance criteria so the memory check ships in the Gap H detector commit, not as a follow-up.

#### §9.2.6 Gap I (SelfIntersecting)

- `stress_i_clean_cube_no_flag`: clean watertight cube; assert 0 SelfIntersecting regions; verifies `skip_adjacent = true` doesn't flag adjacent triangles. Light.
- `stress_i_truncation_at_100`: heavily self-folded mesh with 200+ intersection pairs (a rolled-up thin sheet wound back through itself); assert `self_intersecting.len() == 100` AND issue description contains "(search truncated; total may be higher)". Heavy. **`#[cfg_attr(debug_assertions, ignore)]`** — heavy mesh-repair work.
- `stress_i_canonical_face_a_lt_face_b`: any fixture with self-intersections; assert all entries' `face_a < face_b`. Light.
- `stress_i_vertex_only_contact_not_flagged`: two cubes touching at a single shared vertex (carefully constructed: shared vertex only, no shared edge or interior overlap); assert 0 self-intersections (vertex-only contact is not an intersection per §6.4). Light. (Row 12.)
- `stress_i_near_coplanar_intersection`: two near-coplanar triangles intersecting at a narrow angle (1 mrad off coplanar) at exactly EPS_INTERSECTION × 100 inside their shared region; assert mesh-repair flags this with `epsilon = 1e-10` per `IntersectionParams::default()`. Light.

#### §9.2.7 Gap J (SmallFeature)

- `stress_j_clean_main_body_not_flagged`: 30 mm cube; max_extent 30 ≫ min_feature_size 0.8; assert 0 regions. Light.
- `stress_j_floating_burr_warning`: 30 mm cube on plate + 0.2 mm hex-prism burr next to it (per §7.5 fixture); assert 1 region, severity `Warning` (0.2 < 0.8/2). Light.
- `stress_j_unit_conversion_diagnostic`: 30-µm cube (entire mesh, single component, max_extent 0.030 mm) with FDM `min_feature_size = 0.8`; assert 1 SmallFeature region for the entire mesh (documents the user-diagnostic role per §6.5). Light. (Row 7.)
- `stress_j_below_threshold_warning_not_info`: floating fragment with max_extent = 0.1 mm, `min_feature_size = 0.4`; assert severity Warning (0.1 < 0.4/2 = 0.2). Light.
- `stress_j_just_below_threshold_info`: floating fragment with max_extent = 0.3 mm, `min_feature_size = 0.4`; assert severity Info (0.3 ≥ 0.2). Light.
- `stress_j_open_component_no_panic`: open 5-of-6 face cube (small extent, non-watertight component); assert 1 region; volume from `signed_volume::abs()` is finite (no panic, no NaN). Light. **(Mitigates §6.5 doc — open-component volume is approximate but no-panic.)**
- `stress_j_face_adjacency_via_edge_only`: two faces sharing only a vertex (no edge); assert 2 separate components (one per face) — verifies edge-adjacency definition. Light.
- `stress_j_signed_volume_unit_cube`: unit-cube fragment (1 mm side, exact-representable verts), `min_feature_size = 2.0`; assert volume within `1e-6` of 1.0 mm³ (FP-stable on exact-representable inputs). Light. **(Mitigates §8.2.1 cross-platform FP-drift via exact-representable inputs.)**

### §9.3 Cross-detector stress fixtures

Stress fixtures that exercise multiple detectors at once. Lands once all detectors are in (§7.7 / §7.8 territory).

- `stress_cross_empty_mesh_full_pipeline`: confirms that error path returns before any detector emits issues. Light. (Row 1; subsumes §9.2.1's coverage but as a single end-to-end gate.)
- `stress_cross_inconsistent_winding_blocks_thin_wall_only`: watertight cube with one face flipped; assert exactly: 1 NonManifold Critical (Gap F), 1 DetectorSkipped from ThinWall, 1 DetectorSkipped from TrappedVolume, 0 ThinWall + 0 TrappedVolume entries. Verifies that a single root-cause issue (winding) cleanly cascades through preconditions. Light. (Row 11.)
- `stress_cross_unit_conversion_full_pipeline`: 30-µm cube; assert SmallFeature flag (entire mesh = 1 region, Warning); assert no other detector flags falsely. Light. (Row 7.)
- `stress_cross_disconnected_full_pipeline`: 2 disjoint watertight 20-mm thin-walled cubes-with-cavity (each with 0.4 mm walls and a 5 mm-radius cavity) offset by 30 mm along +X; assert 2 TrappedVolume regions; assert ≥ 4 ThinWall regions (each cube produces ≥ 2 clusters per §7.1 outer/inner-top topology); assert SmallFeature unaffected (each cube max_extent = 20 mm ≫ 0.8 mm). Light. (Row 13.)
- `stress_cross_face_count_at_zero_after_skips`: open mesh that triggers 2 DetectorSkipped issues + 1 NotWatertight Critical; assert `validation.thin_walls.len() == 0 && validation.trapped_volumes.len() == 0` (skip-precondition pathway leaves the typed-region fields empty). Light.
- `stress_cross_nan_input_no_panic`: authors a mesh with one vertex containing `f64::NAN` in `x`; asserts the call to `validate_for_printing` returns without panicking and produces a `PrintValidation` (any contents, including possibly NaN-poisoned numerics, are acceptable as long as no detector panics). Doc-comment caveat: if a future detector adds `debug_assert!(x.is_finite())`, this fixture must be updated to `#[cfg_attr(debug_assertions, ignore)]`. Light. (Row 14.)

### §9.4 §8.4 High-tier risk → stress-fixture coverage

Each High-tier risk in §8.4 maps to ≥ 1 stress fixture proving the mitigation. Pre-flight per §8.4 is the spec-time check; the stress fixture is the regression-time check.

| §8.4 High-tier risk | Pre-flight (§8.4) | Stress fixture(s) (§9.2) | Expected outcome |
|----------------------|--------------------|----------------------------|--------------------|
| **Gap M v0.7 anchor cascade** (per §8.2.3 `validation.rs::tests` sweep + `total_support_volume` shifts) | Read `check_overhangs` predicate in current code; verify FDM-convention semantics match | `stress_m_pure_roof_flagged`, `stress_m_solid_on_plate_bottom_filtered`, `stress_m_floating_box_bottom_flagged`, `stress_m_layered_bottoms_filter_correctly` | Roofs flagged, build-plate-touching faces filtered, both flag and filter survive `place_on_build_plate` semantics |
| **Gap C cluster-split topology** (§7.1 hollow box: edge-adjacency operates per topological component → 2 clusters) | §7.1 fixture topology audit; verify `validation.thin_walls.len() == 2` is theoretical outcome | `stress_c_two_disjoint_thin_clusters` (explicit multi-cluster fixture, ≥ 4 clusters across 2 boxes) + `stress_c_concave_z_shape` (single-cluster baseline) + §7.1 example assertions (2-cluster lock-in on load-bearing fixture) | Cluster count matches edge-adjacency theory across single, double-component, and concave-Z topologies |
| **Gap H cross-platform voxel inside-test FP drift** | Cross-os CI dry-run on §7.3 fixture (manual or via CI matrix) | `stress_h_sphere_inside_cube_volume_within_10pct` + §10's recommendation to **add mesh-printability to the cross-os CI matrix** (next subsection) | Volume assertion's 10% tolerance band absorbs cross-platform ULP variance; cross-os CI catches platform-divergent hard-fails |
| **Gap H grid memory blowup** (>100mm parts at fine voxel) | (no pre-flight; documented ceiling) | `stress_h_voxel_grid_perf_cliff` (proves 100mm ceiling holds in budget) + `stress_h_voxel_grid_oom_safety` (proves >1GB grid emits `DetectorSkipped` instead of OOM) — **§9.2.5 amends §6.3 with a pre-allocation memory check** | Detector either runs within budget or skips with diagnostic; no OOM panic |
| **Gap I Layer Integrity dep-count cap** (mesh-repair add bumps L0 dep count; cap is release_max=80, test_max=100) | Run `cargo xtask grade mesh-printability --skip-coverage` after dep add; verify Criterion 6 stays A | (no runtime stress fixture — this is a build-time concern; Gap I commit's `xtask grade` run IS the verification) | Layer Integrity criterion remains A post-merge |

The "no runtime fixture for Gap I" entry is appropriate — Layer Integrity is a static dep-graph check, not a runtime invariant. The pre-flight `cargo xtask grade` IS the proof.

### §9.5 Where the fixtures live

| Detector | File | Lands with commit |
|----------|------|---------------------|
| Existing detectors | `tests/stress_inputs.rs` (new) | Lands with first stress-fixture-bearing detector commit (Gap M, since Gap M is the first detector-touching commit per §8.3) |
| Gap M | same | Gap M commit |
| Gap C | same | Gap C commit |
| Gap G | same | Gap G commit |
| Gap H | same | Gap H commit |
| Gap I | same | Gap I commit |
| Gap J | same | Gap J commit |
| Cross-detector | same | After all detectors land (between §7.5 small-feature example and §7.6 orientation example, per §12 commit-order) |

`tests/stress_inputs.rs` is a single integration test file (integration-tests-style, not unit-tests-style — exercises the public `validate_for_printing` entry point with adversarial inputs). Each commit appends; no commit modifies a previous commit's fixtures.

**Hand-authoring note**: stress fixtures that need >50 LOC of vertex authoring (e.g., `stress_h_sphere_inside_cube_volume_within_10pct`) factor a small per-test helper in the same file (e.g., `fn build_sphere_in_cube_fixture() -> IndexedMesh`). No shared `tests/common/` module per `feedback_simplify_examples` — even at the cost of 6× duplicated cube-builder boilerplate across stress fixtures.

### §9.6 Numerical-anchor stability across stress fixtures

Per §8.2.1 cross-platform FP risk, each stress fixture's assertions follow the §4.5 convention:

- Assertions on **counts** (region counts, issue counts) use exact equality (`assert_eq!`).
- Assertions on **derived geometric values** (areas, volumes, thicknesses, angles) use `approx::assert_relative_eq!` with tolerances from §4.5 (`epsilon = 1e-6` for distances/angles; `max_relative = 0.10` for voxel-discretization volume; `max_relative = 1e-3` for divergence-theorem volume).
- Assertions on **severity classifications** use exact match (severity is enum-typed; no FP).
- Assertions on **descriptions** use `description.contains("...")` substring match (not full-string equality), so future doc-improvement edits don't break stress fixtures.

### §9.7 Stress-test gauntlet — total fixture count

| Category | Count |
|----------|-------|
| §9.2.1 Existing detectors | 3 |
| §9.2.2 Gap M | 5 |
| §9.2.3 Gap C | 7 (1 release-only) |
| §9.2.4 Gap G | 5 |
| §9.2.5 Gap H | 8 (1 release-only) |
| §9.2.6 Gap I | 5 (1 release-only) |
| §9.2.7 Gap J | 8 |
| §9.3 Cross-detector | 6 |
| **Total** | **47** (3 release-only) |

Stress fixtures push the post-arc test total to **~186 + 47 = ~233** (vs §-revised "post-arc total ~186" in the gap memo). Updated test-count budget table in §13's PR-close memo migration.

---

## §10. Grading & CI impact

This section names the v0.7→v0.8 grade delta per criterion, the per-commit grading cadence, and the CI matrix changes (additions and recommendations).

### §10.0 v0.7.0 grade baseline (captured 2026-04-30)

`cargo xtask grade mesh-printability --skip-coverage` (run on `feature/mesh-printability-v0-8` cut-point, before any v0.8 commit lands):

| # | Criterion | Result | Grade |
|---|-----------|--------|-------|
| 1 | Coverage | (skipped per `--skip-coverage`) | — |
| 2 | Documentation | 0 warnings | A |
| 3 | Clippy | 0 warnings | A |
| 4 | Safety | 0 violations | A |
| 5 | Dependencies | 5 deps, all justified | A |
| 6 | Layer Integrity | ✓ confirmed | A |
| 7 | WASM Compat | ✓ builds | A |
| 8 | API Design | (manual review) | ? |

**Automated grade: A across 7/7 criteria.** Coverage runs out of band; v0.8 will lift it during end-of-arc grading checkpoint (§10.6). The baseline is what every commit's grading gate compares against (per `feedback_grading_rubric` "A-across-the-board or fix-and-regrade before execution").

**Direct dep count baseline**:
- `cargo tree -p mesh-printability -e normal --prefix none | sort -u | wc -l` = **34** unique entries (release graph; cap at L0 release_max = 80)
- `cargo tree -p mesh-printability -e normal,dev --prefix none | sort -u | wc -l` = **34** (no `dev`-only deps in v0.7 graph beyond `approx`, which is workspace-shared)
- Headroom for Gap I's mesh-repair add: 80 − 34 = 46 deps. mesh-repair's own release graph is 46 unique deps; estimated overlap (mesh-types, nalgebra, hashbrown, rayon-shared, cf-geometry-shared) brings combined unique count to ~50–55 unique entries — **well under 80**. Pre-flight sanity check; commit-time `cargo xtask grade` is the authoritative verification per §8.4.

### §10.1 Per-criterion impact across v0.8

For each criterion, what changes during v0.8 and what's the target post-arc state.

#### §10.1.1 Coverage (Criterion 1)

v0.7 baseline: not captured during arc-grading runs (they use `--skip-coverage`). v0.7 coverage value runs in scheduled CI nightly + locally via `cargo xtask grade mesh-printability` (without `--skip-coverage`).

v0.8 impact:
- **Test count**: 32 (existing) + ~34 (§5 per-fix) + ~50 (§6 per-detector) + ~70 (§7 example-driven main() assertions) + 47 (§9 stress) ≈ **~233 tests** (4 release-only). Test surface roughly 7× the v0.7 baseline.
- **Source LOC**: +5 new detectors (~150–250 LOC each) + 1 architectural change (Gap L) + 5 in-place fixes ≈ +1500 LOC src in `validation.rs` + `regions.rs` + 1 new private helper file (`tests/stress_inputs.rs` is integration-tests, not src).
- **Coverage target**: maintain or improve the v0.7 absolute % (baseline value captured at end-of-arc grading; arc deletion of the working spec doesn't change Coverage).

If Coverage drops below v0.7 baseline at end-of-arc, **stop and add tests** before the v0.8.0 release commit (§12). Per `feedback_grading_rubric` no-exceptions A-grade.

#### §10.1.2 Documentation (Criterion 2)

v0.7 baseline: A (0 rustdoc warnings under `RUSTDOCFLAGS=-D warnings`).

v0.8 impact:
- New public types in `regions.rs` (4 region types per §3) + new `PrinterConfig` field + new variant `PrintIssueType::DetectorSkipped` + 5 new public functions in `validation.rs` — each requires complete rustdoc.
- New doc-tests on each new public function + region type — adds ≥5 doc-tests (per §3 acceptance: "Each new detector commit adds a doc-test on the public `check_*` function or the populated region type").
- **Pre-commit hook is fmt+clippy only** per `feedback_cargo_doc_pre_commit`; rustdoc only surfaces at xtask grade gate. **Each commit's local validation must include**: `RUSTDOCFLAGS=-D warnings cargo doc --no-deps -p mesh-printability` (per the working memory recommendation).

Target: maintain A. Risk: rustdoc warnings (un-backticked brackets like `face[N]` parsed as anchor links) on new doc comments. Mitigation: per-commit local rustdoc check.

#### §10.1.3 Clippy (Criterion 3)

v0.7 baseline: A (0 warnings against own lints; not workspace lints — Gap A target).

v0.8 impact:
- **Gap A**: workspace lints inheritance (`[lints] workspace = true`) tightens the ruleset. §5.1 stop-and-raise gate enforces ≤10 fallout sites per the v0.8-as-arc commitment.
- **All new code post-Gap A**: must satisfy workspace lints from authoring time, not retrofit.
- **Per-commit gate**: `cargo clippy -p mesh-printability --tests --all-targets -- -D warnings` clean.

Target: maintain A under workspace lints (which is a strictly tighter standard than v0.7's own-lints).

#### §10.1.4 Safety (Criterion 4)

v0.7 baseline: A (0 unsafe violations).

v0.8 impact:
- Möller-Trumbore ray-tri intersection helper (Gap C, §6.1): pure-safe nalgebra arithmetic; no `unsafe`.
- Voxel scanline + flood-fill (Gap H, §6.3): pure-safe; uses `Vec<u8>` for the grid + standard BFS over indexed cells.
- mesh-repair re-use (Gap I): mesh-repair's own grade includes safety. Since the v0.8 dep add is path-only (workspace-local), our Safety criterion doesn't transitively check mesh-repair's safety; but the workspace's grade-all does cover it.

Target: maintain A — no `unsafe` blocks introduced.

#### §10.1.5 Dependencies (Criterion 5)

v0.7 baseline: A — grader reports "5 deps, all justified" = 4 normal (`mesh-types`, `nalgebra`, `thiserror`, `hashbrown`) + 1 dev (`approx`).

v0.8 impact:
- **Gap I**: `mesh-repair = { path = "../mesh-repair" }` added. Justification: `// for SelfIntersection re-use; avoids re-implementing tri-tri intersection`. Inline comment in Cargo.toml per the workspace's justification convention (matches existing deps' pattern).
- **No transitive concerns at the Dependencies criterion level** — Criterion 5 audits direct deps, not transitive. Post-arc direct-dep count: 5 normal + 1 dev = 6 per grader.

Target: maintain A — 1 new direct dep, justified inline.

#### §10.1.6 Layer Integrity (Criterion 6)

v0.7 baseline: A (`Tier::L0` confirmed; release graph 34 unique entries < 80 cap; banned-pattern check passes).

v0.8 impact (the **High-tier risk** per §8.4):
- **Gap L (build_up_direction)**: parametrize-via-config — no dep change, no tier change. ✓
- **Gap I (mesh-repair add)**: mesh-printability stays L0; mesh-repair is L0. L0→L0 dep — no Layer Integrity tier-shift. Combined unique-dep count estimated at ~50–55, under the 80 cap.
- **Cross-platform note**: Layer Integrity also enforces banned-pattern checks. mesh-repair's transitive deps (`rayon`, `tracing`, `cf-geometry`) are all on L0's allowed list (none of them in `L0_BANNED` per `xtask/src/grade.rs:2079–2110`).

**Pre-flight at Gap I commit (per §8.4)**: run `cargo xtask grade mesh-printability --skip-coverage` after the Cargo.toml dep add but before any new src code lands; verify Criterion 6 stays A. If it fails (unique-dep count exceeds 80, or a banned pattern surfaces), STOP and surface to user — Gap I cannot ship without dep-graph headroom.

Target: maintain A. Risk: if a future workspace lint pattern is added to `L0_BANNED` mid-arc and one of mesh-repair's transitive deps matches, Gap I breaks. Mitigation: per-commit `cargo xtask grade` re-runs the check.

#### §10.1.7 WASM Compat (Criterion 7)

v0.7 baseline: A (L0-tier WASM build via `cargo build --target wasm32-unknown-unknown -p mesh-printability`).

v0.8 impact:
- **Gap I (mesh-repair add)**: empirically verified — `cargo build --target wasm32-unknown-unknown -p mesh-repair` builds cleanly (~10 s on warm cache, 2026-04-30 local check on `feature/mesh-printability-v0-8` branch HEAD). mesh-repair's transitive deps (rayon, tracing, cf-geometry) all compile to wasm32 without friction. mesh-printability inheriting them poses no WASM blocker.
- **Pre-flight at Gap I commit**: `cargo build --target wasm32-unknown-unknown -p mesh-printability` post-dep-add to confirm.

Target: maintain A. Risk: low — empirical baseline already passes for the dep being added.

#### §10.1.8 API Design (Criterion 8 — manual review)

v0.7 baseline: not graded automatically. v0.8 doesn't change the manual-review process; the spec itself + `COMPLETION.md` rewrite (Gap K) is the v0.8 contribution to API Design hygiene.

v0.8 impact:
- New public types follow existing pattern (`new()` + builder methods consistent with `ThinWallRegion` / `OverhangRegion` / `SupportRegion`).
- New `PrintIssueType::DetectorSkipped` variant + behavior documented (§5.8).
- v0.7→v0.8 semver-strict-breaking fields documented in `CHANGELOG.md` (§5.10) + version bump rationale (§3).
- Public surface diff table in §3 supports the manual API review.

Target: pass manual review. Pre-merge gate: `cargo xtask complete mesh-printability` per the grader's "next step" message.

### §10.2 Per-commit grading cadence

Per §8.4 risk-tier response policy + `feedback_grading_rubric`, every commit runs the grading gate before being committed:

```
cargo xtask grade mesh-printability --skip-coverage
```

Acceptance: All 7 criteria report A. Failure = stop-and-fix before commit lands.

**Per-commit additional gates** (beyond the xtask grade):
- `cargo test -p mesh-printability` — unit + integration tests pass.
- `cargo clippy -p mesh-printability --tests --all-targets -- -D warnings` — workspace lints clean (Gap A onwards).
- `RUSTDOCFLAGS=-D warnings cargo doc --no-deps -p mesh-printability` — rustdoc clean per `feedback_cargo_doc_pre_commit`.
- For example commits (§7): `cargo run -p example-mesh-printability-<name> --release` exits 0 (numbers-pass).
- For workspace-member additions (each example commit): `cargo build --workspace` to confirm workspace-members list integrity.

**xtask grade vs per-criterion checks**: per `feedback_xtask_grade_opacity`, the full grade run is opaque. During iteration, prefer per-criterion checks above; the full `cargo xtask grade` is the final confirmation gate at commit time.

**High-tier-commit additional gates** (per §8.4):
- **Commit #2 (Gap M)**: `cargo test -p mesh-printability` — sweep `validation.rs::tests` for any test asserting on overhang-derived numerics; update inline. Per §5.9 "test-numerical-anchor regression sweep".
- **Commit #17 (Gap I, mesh-repair add)**: `cargo xtask grade mesh-printability --skip-coverage` BEFORE adding any src consuming mesh-repair; verify Criterion 6 stays A. Per §8.4 pre-flight.

### §10.3 CI matrix — current state and v0.8 deltas

Reference: `.github/workflows/quality-gate.yml`. The matrix has 8 jobs that gate `feature/*` and `develop`/`main` branches:

| Job | What it runs | mesh-printability included? |
|-----|--------------|------------------------------|
| `format` | `cargo fmt --all -- --check` | yes (workspace-wide) |
| `grade` | `cargo run -p xtask --release -- grade-all --skip-coverage` | yes (graded crate-by-crate; mesh-printability is in the workspace) |
| `tests-debug` | `cargo test -p mesh-types -p mesh-io ... -p mesh-printability ...` (ubuntu) | **yes** (line 173) |
| `tests-release` | `cargo test --release -p sim-ml-chassis ...` (ubuntu, 5 heavy crates + sim-soft) | **no** — not currently included |
| `cross-os` | `cargo test -p sim-core -p sim-mjcf -p mesh-io` (macos + windows) | **no** — not currently included |
| `feature-combos` | `cargo test --features gpu-probe -p sim-soft` | not applicable (no feature gates) |
| `dependencies` | `cargo-deny check licenses sources` | yes (workspace-wide) |
| `semver` | `cargo-semver-checks-action` (mesh-repair only) | not applicable (mesh-printability not on crates.io yet) |
| `sbom` | `cargo cyclonedx` (workspace-wide) | yes (workspace-wide) |

Final `quality-gate` job aggregates: `[format, grade, tests-debug, tests-release, cross-os, feature-combos, dependencies]`.

**v0.8 deltas (additive)**:

1. **`tests-debug` automatically picks up new tests** — no CI plumbing change. Existing 32 + ~152 new tests (per §10.1.1) = ~184 debug-only tests; well under the 20-min `tests-debug` budget for mesh-printability's per-crate share.
2. **Examples are tested via `cargo build --workspace`** in `tests-debug`; CI compiles all workspace members. The 8 new example crates (§7) compile and the resulting binaries are valid; their `main()` numerical-anchor assertions run as **smoke tests** when manually invoked (`cargo run -p ...`) but are NOT in the CI test matrix unless promoted (deferred to v0.9 if needed).
3. **`tests-release` not added** — mesh-printability has no heavy stochastic-physics tests. The 3 release-only stress fixtures from §9 (`stress_c_5k_tri_perf_budget`, `stress_h_voxel_grid_perf_cliff`, `stress_i_truncation_at_100`) use `#[cfg_attr(debug_assertions, ignore)]` per `feedback_release_mode_heavy_tests`. For these to actually run in CI, they need `cargo test --release` invocation. **Recommendation**: extend `tests-release` job to include `mesh-printability` for these 3 fixtures (~2 min added compile cost; runs in parallel with the existing 5 heavy crates). See §10.4 for the full recommendation summary.
4. **`cross-os` extension** — see §10.4.

### §10.4 CI additions resolved for v0.8

Two CI matrix changes ship as part of the arc, surfaced from §8.4 High-tier risk coverage. Resolved 2026-04-30 (master-architect call): both ship; combined CI cost ~3–4 min in the existing parallel jobs; matches the §8.4 response policy ("High-tier: pre-flight verification mandated; additional test fixtures if any uncertainty"). Implementation commit slot: each YAML edit lands as part of the **corresponding detector commit** (§10.4.1 lands with Gap H detector commit; §10.4.2 lands with the first release-only stress fixture, i.e., Gap C detector commit) — keeps the CI plumbing change beside the test it protects.

#### §10.4.1 Add mesh-printability to `cross-os` matrix (Gap H FP-drift coverage)

**Why**: Gap H's voxel inside-test (scanline ray-tri parity + flood-fill) is FP-drift-sensitive on cross-platform per §8.2.1's Phase-4-faer-LDLᵀ lesson. macOS/Linux/Windows can diverge by 1+ voxel on the boundary of trapped components, breaking `validation.trapped_volumes.len()` assertions or volume tolerances.

**Mitigation in v0.8**: §6.3's `max_relative = 0.10` volume tolerance + `assert!(volume_in_tolerance)` should absorb single-voxel discretization noise. **But platform-divergent hard-fails** (e.g., mesh-repair returns a different `pairs_count` on Windows, or scanline parity flips on macOS) only surface in a multi-OS test run.

**Proposed change** — `quality-gate.yml::cross-os::matrix`:

```yaml
cross-os:
  ...
  - name: Run tests (default features)
    run: cargo test -p sim-core -p sim-mjcf -p mesh-io -p mesh-printability
    shell: bash
```

Single-line addition. Cost: ~2 min added per OS (mesh-printability has small src + ~233 tests; compiles fast in `cargo test` debug mode). Fits comfortably in the existing `cross-os` job runtime.

**Alternative considered**: only add the §9 stress fixtures to cross-os (via test selection); rejected as more brittle than running all of mesh-printability's tests.

**§9.4 Gap H FP-drift mitigation** depends on this CI extension being in place before the arc closes.

**Resolved: ships in v0.8.** Lands inside the Gap H detector commit (§12 commit-order) so the YAML edit is reviewable alongside the detector that depends on it.

#### §10.4.2 Add mesh-printability to `tests-release` matrix (release-only stress fixtures)

**Why**: §9's 3 release-only fixtures (`stress_c_5k_tri_perf_budget`, `stress_h_voxel_grid_perf_cliff`, `stress_i_truncation_at_100`) test perf-cliff regression. Without `cargo test --release` in CI, they only run locally — perf regressions go unnoticed.

**Proposed change** — `quality-gate.yml::tests-release` step:

```yaml
- name: Run tests (--release)
  run: |
    cargo test --release \
      -p sim-ml-chassis -p sim-thermostat -p sim-rl \
      -p sim-conformance-tests -p sim-opt -p mesh-printability
    cargo test --release -p sim-soft \
      --test hertz_sphere_plane \
      --test contact_drop_rest \
      --test non_interpenetration
  shell: bash
```

(Single `-p mesh-printability` addition.)

Cost: mesh-printability `cargo test --release` is roughly 3–5 min cold-cache, 1–2 min warm-cache (Swatinem/rust-cache active). Adds to `tests-release` job runtime; runs in parallel with `tests-debug` so no critical-path impact.

**Alternative considered**: leave perf-cliff fixtures local-only and document that v0.8 doesn't gate on perf in CI. Rejected because perf regressions silently accumulate.

**Resolved: ships in v0.8.** Lands inside the Gap C detector commit (the first commit that introduces a release-only stress fixture), so the YAML edit is reviewable alongside the test it gates.

### §10.5 Other CI implications

- **`dependencies` job (cargo-deny)**: Gap I adds `mesh-repair` as a workspace-internal dep (path-based, not crates.io). cargo-deny's licenses + sources audit operates on the dep graph, transitively covering mesh-repair's transitive deps (rayon, tracing, cf-geometry). All those crates already pass cargo-deny in the workspace baseline (they're consumed by other workspace members). **No change expected.**
- **`semver` job**: only runs against `mesh-repair` per `quality-gate.yml:338` — not mesh-printability. v0.8 publishes neither to crates.io. **No change.** Future v0.9 work could add `package: mesh-printability` to the semver-checks step once a crates.io baseline exists.
- **`sbom` job**: workspace-wide cyclonedx; mesh-printability already covered. **No change.**
- **`format` job**: `cargo fmt --all -- --check` workspace-wide. New code in `validation.rs`, `regions.rs`, `tests/stress_inputs.rs`, and 8 new example crates must be `cargo fmt --all`-clean per the existing pre-commit hook. **No matrix change.**

### §10.6 End-of-arc grading checkpoint

Per §13's spec-lifecycle plan, the **second-to-last commit** of the arc (the v0.8.0 release commit, §8.3 row #24) runs:

```
cargo xtask grade mesh-printability                      # full grade with coverage (5–10 min)
cargo xtask grade-all                                    # workspace-wide grade (5–10 min)
cargo build --workspace --release                        # examples compile (~3–5 min cold)
cargo test -p mesh-printability                          # all 230+ debug tests pass
cargo test --release -p mesh-printability                # 3 release-only stress fixtures pass
RUSTDOCFLAGS=-D warnings cargo doc --no-deps -p mesh-printability   # rustdoc clean
```

(Workspace clippy is enforced by `cargo xtask grade-all` Criterion 3 — no separate `cargo clippy --workspace` call needed.)

**Acceptance**: all 7 automated criteria A; workspace `grade-all` no regression; coverage maintained or improved vs v0.7.0 baseline. Per `feedback_grading_rubric` "A-across-the-board or fix-and-regrade before execution" — any criterion below A blocks the v0.8.0 release commit.

If a criterion drops to B or lower, **stop and fix** before the release commit lands. Specific failure-mode playbook:

| Failure | Likely cause | Recovery |
|---------|--------------|----------|
| Coverage drops | New uncovered branch in §6 detectors | Add unit tests; re-run grade |
| Documentation B | Missing rustdoc on new public type | Add doc-comment; re-run grade |
| Clippy B | New warning post-Gap-A workspace lints | Triage warning; either fix or justify with `#[allow(..)]` (per `feedback_grading_rubric`'s grader lint-justification scan) |
| Safety B | Accidental `unsafe` in new code | Refactor to safe pure-Rust |
| Deps B | Unjustified new dep | Add justification comment to Cargo.toml |
| Layer Integrity B | Banned-pattern dep snuck in via transitive | Investigate `cargo tree -e normal -p mesh-printability`; remove offending transitive |
| WASM B | mesh-repair's transitive breaks WASM build | Add `cfg(target_arch = "wasm32")` fallback in mesh-printability or escalate to mesh-repair |

### §10.7 Summary — v0.8 grading impact

| Criterion | v0.7 | Target v0.8 | Risk source |
|-----------|------|--------------|--------------|
| 1. Coverage | (skipped) | match v0.7 % or better | new src LOC (+5 detectors); offset by ~152 new tests |
| 2. Documentation | A | A | new public types need rustdoc + doc-tests |
| 3. Clippy | A (own lints) | A (workspace lints) | Gap A's stop-and-raise gate caps fallout at 10 sites |
| 4. Safety | A | A | no `unsafe` introduced |
| 5. Dependencies | A (5 = 4 normal + 1 dev) | A (6 = 5 normal + 1 dev) | mesh-repair add justified |
| 6. Layer Integrity | A | A | mesh-repair add stays under 80-dep L0 cap (estimate ~50–55) |
| 7. WASM Compat | A | A | mesh-repair already WASM-compat |
| 8. API Design | manual review | passes manual review | spec authors the rationale |

**v0.8 ships A across all 7 automated criteria. Both §10.4 CI matrix changes ship as part of the arc per master-architect call 2026-04-30** (resolved §10.4.1 + §10.4.2 — folded into the corresponding detector commits in §12).

---

## §11. Open questions

Most open questions surfaced during spec authoring were resolved inline (§7.9 Q1–Q7, §9.2.5 §6.3 OOM amendment, §10.4 CI extensions). This section captures the residual: items that were genuinely undecided after §1–§10 and the master-architect call resolving each.

### §11.1 Mesh umbrella version (§3 flag) — resolved

**The question** (§3, line 275): `mesh/Cargo.toml` uses `version.workspace = true` (workspace-package version `0.7.0`). The umbrella re-exports mesh-printability as `mesh::printability`, so its surface technically expanded with v0.8. Two options:

1. **Keep umbrella at workspace 0.7.0**: pragmatic; path-deps mean no consumer of the umbrella sees a version mismatch; only matters if mesh ships to crates.io.
2. **Switch umbrella to its own version, bump to 0.8.0**: strict semver hygiene; signals to external consumers that the umbrella's API surface grew.

**Resolved (master-architect call 2026-04-30): option 1.** Rationale:

- The umbrella `mesh` crate is **not published to crates.io** (verified in §10.5: the `semver` job in `quality-gate.yml` checks only `mesh-repair`, the sole published workspace crate).
- All 6 in-workspace consumers of mesh-printability (umbrella + 3 examples + regression test + workspace root) use path-deps, which resolve the latest version regardless of pin.
- The umbrella's *internal* code didn't change — only the re-exported surface area expanded by virtue of mesh-printability's own changes. Bumping the umbrella to 0.8.0 in this arc sends a misleading signal that the umbrella itself shipped a behavioral change.
- Decoupling one crate from `version.workspace = true` (option 2) is a one-manifest edit (just `mesh/mesh/Cargo.toml`), not workspace-wide. But it creates a precedent — the next umbrella-publishing crate would need the same decoupling, and the workspace-version vs per-crate-version policy is best decided once at first-publish, not piecemeal per-arc.
- Trigger for re-opening this decision: the moment a workspace crate is first published to crates.io (currently mesh-repair is the sole candidate), the workspace-version vs per-crate-version question gets a workspace-wide ergonomic answer, not a per-arc one.

**Implementation consequence**: §8.3 commit #24 (v0.8.0 release commit) bumps **only** `mesh/mesh-printability/Cargo.toml` from `0.7.0` to `0.8.0`. The umbrella `mesh/mesh/Cargo.toml` stays at `version.workspace = true` (workspace version unchanged at `0.7.0`). No other Cargo.toml edits in commit #24.

### §11.2 Coverage baseline % — resolved (capture timing)

**The question**: §10.0's grade baseline runs with `--skip-coverage`, so v0.7 coverage % is not pinned in the spec. v0.8's coverage target (§10.1.1) is "match v0.7 % or better" — but without an explicit number, "or better" is ambiguous if the v0.7 value drifts during the arc due to scheduled-CI runs against `main`.

**Resolved (master-architect call 2026-04-30)**: the **v0.7 coverage baseline is captured at the start of the arc** (Gap A — specifically commit 1c per §5.1's 3-commit split, since 1c is where `[lints] workspace = true` lands and is the first sub-commit producing a clippy-clean state suitable for grading), not at end-of-arc. Sequence:

1. Commit 1c's local validation includes a one-time `cargo xtask grade mesh-printability` (without `--skip-coverage`) on the branch HEAD post-1c. This run takes 5–10 min on the project's reference machine but runs once. (Pre-1a baseline is functionally equivalent for Coverage since 1a/1b only touch test bodies + add `const` keywords; neither shifts coverage. 1c is chosen for the capture so the recorded baseline is post-lints-flip, matching the workspace-lints-inherited state of every subsequent commit.)
2. The Coverage % output is recorded in commit 1c's message body (a single line: `v0.7 coverage baseline: <X>%`).
3. The end-of-arc grading checkpoint (§10.6) compares end-of-arc Coverage % against the recorded baseline.
4. Acceptance: end-of-arc Coverage ≥ baseline (per `feedback_grading_rubric` "A-across-the-board"). If end-of-arc drops, add tests until it lifts.

This pins the comparison point pre-arc, prevents drift, and avoids the 5–10-min run becoming a per-commit gate (which it cannot be at xtask grade's `--skip-coverage` performance).

**Alternative considered**: capture at end-of-arc and compare against itself ("Coverage A-grade threshold ≥75% per `xtask/src/grade.rs`"). Rejected because the relative comparison ("did v0.8 lift Coverage?") is more informative than the absolute threshold check.

### §11.3 Tier-classification continuity — resolved (mesh-printability stays L0)

**The question**: Gap I adds `mesh-repair = { path = "../mesh-repair" }`. mesh-repair is L0; mesh-printability is L0. Adding a same-tier dep does not auto-promote tier, but should the `[package.metadata.cortenforge] tier = "L0"` declaration in `mesh/mesh-printability/Cargo.toml` change in v0.8?

**Resolved (master-architect call 2026-04-30): no change to tier declaration.** Rationale:

- mesh-printability stays L0 functionally and architecturally — pure-Rust validation analysis on indexed-mesh primitives, no I/O, no GPU.
- The Layer Integrity criterion (§10.1.6) verifies mesh-printability's release-graph dep count stays under L0's 80-cap (estimated ~50–55 post-arc).
- v0.9 candidate trigger: a downstream consumer requests mesh-printability to expose a streaming/progress-callback interface that requires `tracing` instrumentation, in which case L0→L0-io tier-up is on the table. v0.8 does not introduce such an interface (§4.6 explicitly keeps mesh-printability tracing-free).

**Implementation consequence**: no edit to `[package.metadata.cortenforge]` in mesh-printability's `Cargo.toml`.

### §11.4 What's NOT open (consolidated audit trail)

The following items were considered as candidates for §11 during the §8/§9/§10 reviews but resolved inline; recorded here so the audit trail is explicit and the reviewer doesn't re-discover them as open:

- **§6.3 OOM safety amendment** — surfaced during §9.2.5 risk-mitigation review; resolved inline (1 GB voxel-grid cap with DetectorSkipped fallback). Master-architect call 2026-04-30.
- **§10.4.1 cross-os matrix extension for mesh-printability** — surfaced during §10.4 authoring; resolved inline (ships in v0.8 inside Gap H detector commit). Master-architect call 2026-04-30.
- **§10.4.2 tests-release matrix extension for mesh-printability** — surfaced during §10.4 authoring; resolved inline (ships in v0.8 inside Gap C detector commit). Master-architect call 2026-04-30.
- **§7.1, §7.3 fixture construction approach** — §7 specifies hand-author option 1 as recommended baseline, with `mesh-offset::marching_cubes(grid, config)` option 2 as documented fallback. Pre-flight battery 2026-04-30 verified that mesh-sdf has NO public MC API (it exposes SDF queries only); mesh-offset is the workspace's public MC entry point and is L0 (same tier as mesh-printability), so option 2 is layer-clean. This is an impl-time choice, not a spec-time decision; defaults to hand-author (verified deterministic + tractable). §7.8 (showcase) explicitly hand-authors all components per §7.8 — MC fallback not applicable.
- **`feature-combos` job extension** — v0.8 introduces no feature gates on mesh-printability; the existing `feature-combos` job (sim-soft `gpu-probe`-only) is unaffected. Re-evaluate if v0.9 adds a feature flag (e.g., `bvh` for the optional BVH acceleration triple-tracked in §1).
- **Example main() promotion to CI test gate** — §10.3 note 2 documents that example `main()` numerical-anchor assertions run as smoke tests (manual `cargo run`) but are NOT in the CI test matrix. Trigger for v0.9 promotion: a regression on an example that wasn't caught locally lands on `main`. v0.8 covers `cargo build --workspace` (compile-only) for example crates inside `tests-debug`; per-example main() exit-code testing in CI is deferred.
- **Umbrella `mesh/mesh/tests/api_regression.rs` updates** — §3 audit (F14) verified no struct-update syntax on `PrintValidation`; existing tests use `let _ = result.is_printable();` discardable patterns. v0.8's API expansion is purely additive at the umbrella surface. Verified at commit-time per §10.2's `cargo build --workspace` per-commit gate.
- **`PrintTechnology::Other` `requires_supports()` semantic** — pre-existing v0.7 behavior (returns true conservatively). Not a v0.8 decision.

### §11.5 Re-open triggers (explicit)

Each closed-but-trackable item carries a v0.9 re-open trigger; consolidated for audit:

| Item | v0.9 re-open trigger |
|------|------------------------|
| Mesh umbrella version (§11.1) | First workspace crate publishes to crates.io |
| Coverage % baseline pinning (§11.2) | A user asks for a per-detector coverage breakdown |
| Tier-classification (§11.3) | A consumer requests mesh-printability `tracing` instrumentation |
| §6.3 1 GB voxel cap (§11.4) | A user reports a >100 mm part being silently DetectorSkipped |
| Example main() in CI (§11.4) | A regression on an example lands on main without local catch |
| Cavity-ceiling overhang co-flag (§7.9 / §11.4) | A user asks for cavity-aware overhang severity |
| AttributedMesh face-attributes (Q2 / §11.4) | (i) user reports centroid point-cloud is insufficient for visual review, OR (ii) AttributedMesh gains face-attribute support and the integration is ~30 LOC |
| BVH for ThinWall + SelfIntersecting (§1 out-of-scope) | A real mesh exceeds 10k tris and validation runtime exceeds 5 s on the project's reference machine |
| FP bit-exactness of overhang predicate (§5.1 deferral) | Cross-platform CI run on Gap H or Gap M fixtures shows divergence within tolerance but at the bit level (e.g., affected-face count differs by 1 between macOS/Linux/Windows on a fixture authored to land *exactly* on the threshold). Trigger applies to either: (i) replacing the 9 per-site `#[allow]`s with `mul_add` / `midpoint` after a tolerance-based diff confirms semantic equivalence, OR (ii) reframing the predicate to be FMA-stable across platforms. |

Each trigger is verifiable from outside the spec (memo backlog + CHANGELOG `[Unreleased]` block) so v0.9-arc planning isn't dependent on the deleted v0.8 spec.

---

## §12. Implementation order + commit boundaries

This section is the **canonical commit-order spec**. Three §-internal inconsistencies surfaced in §8.3 are reconciled here; §12 supersedes §8.3's working-state best-guess.

### §12.0 Reconciliation of §-internal inconsistencies

| # | Conflict | Reconciled to | Rationale |
|---|----------|---------------|-----------|
| 1 | `build_edge_to_faces` refactor placement: §5.3 says "separate commit before Gap D" vs §7.9's locked-order doesn't list it | **Separate commit between Gap B and Gap D** (slot #4 in §12.1) | §5.3's reading. Extracting the helper from `check_basic_manifold` is a refactor that benefits from isolated diff review + the §5.3 regression-gate sweep. Bundling with Gap D would conflate "refactor" with "behavior change". |
| 2 | CHANGELOG creation slot: §1+§5.10 say "at Gap A" vs §7.9 lists it post-DetectorSkipped | **Created in Gap A's commit 1c (the only Gap A sub-commit that touches `Cargo.toml`); every subsequent commit (#2 onward) appends entries; sub-commits 1a + 1b predate file creation and have no CHANGELOG diff** | §5.10's reading. There's no separate "CHANGELOG creation" commit. §7.9's slot was a stale artifact of an earlier ordering. The CHANGELOG-as-living-document pattern is what's standard for `keepachangelog.com` workflows. |
| 3 | Detector↔example interleave: §7.0 says "interleaved" vs §7.9 reads as "batched-after" | **Interleaved** (detector → its example → next detector) | §7.0's reading. Per `feedback_one_at_a_time` + `feedback_one_at_a_time_review`: each detector commit is followed by its example commit + user visuals-pass before the next detector lands. This is the slow-cadence pattern the user's `feedback_patience_track_record` calls out as historically successful. |
| 4 | Gap K (COMPLETION.md rewrite) slot: §1 says "late (rewrite when truth is settled)" vs §7.9 places it at slot 8 (after L, before DetectorSkipped) | **Bundled into the v0.8.0 release commit** (§12.1 row #24) | §1's reading. "Truth is settled" only after all detectors + examples land. Authoring COMPLETION.md early forces partial-then-fill across many commits, drifting the document at every detector add. Bundling into release ships one clean rewrite with the full v0.8 inventory. Surfaced during §12 risk-mitigation review pass. |

### §12.1 Canonical commit order

**27 commits total** (Gap A 3-commit split per §5.1 lifts the count from 25 to 27; subsequent commit numbers preserved via letter-suffix sub-numbering on the Gap A slot, so existing #2..#25 references throughout the spec stay valid). Each row names the commit, its scope, the affected files, and the v0.8.0 baseline target it advances. **Pause-for-visuals-pass** rows (per `feedback_one_at_a_time_review`) are flagged with ⏸︎ — implementation pauses there for user review before the next commit starts.

| # | Commit | Scope | Files | High-tier? | Pause? |
|---|--------|-------|-------|------------|--------|
| 1a | `chore(mesh-printability): mechanical clippy fallout — const fns, casts, format args, match arms (Gap A sub-1)` | §5.1 mechanical-only sites (19 const_fn + 3 cast + 1 format + 1 match = 24) | per-site src edits in `validation.rs`, `orientation.rs`, `regions.rs`, `config.rs` (no Cargo.toml or CHANGELOG yet) | — | — |
| 1b | `chore(mesh-printability): test-site clippy cleanup — expect_used (Gap A sub-2)` | §5.1 test-site sites (5 expect_used, allow or restructure) | `validation.rs::tests` per-site edits | — | — |
| 1c | `chore(mesh-printability): inherit workspace lints + per-site FP-semantics allows + create CHANGELOG (Gap A sub-3)` | §5.1 lints flip + 9 FP-semantics `#[allow]`s + §5.10 CHANGELOG creation + §11.2 v0.7 coverage baseline capture | `Cargo.toml` (`[lints] workspace = true`), `CHANGELOG.md` (new), 8× `#[allow(clippy::suboptimal_flops)]` + 1× `#[allow(clippy::manual_midpoint)]` per-site annotations in `validation.rs` + `orientation.rs` | — | — |
| 2 | `fix(mesh-printability): correct overhang predicate to FDM convention + build-plate filter (Gap M)` | §5.9 (M) + §9.2.1 + §9.2.2 stress fixtures + `tests/stress_inputs.rs` (new file) | `validation.rs`, `orientation.rs`, `validation.rs::tests`, `tests/stress_inputs.rs` (new), `CHANGELOG.md` | **High** (§8.4 anchor cascade) | — |
| 3 | `fix(mesh-printability): track actual maximum overhang angle (Gap B)` | §5.2 | `validation.rs::check_overhangs`, `validation.rs::tests`, `CHANGELOG.md` | — | — |
| 4 | `refactor(mesh-printability): extract build_edge_to_faces helper` | §5.3 sub-step (helper extraction) + §5.3 regression gate | `validation.rs` (new helper module / fn), `CHANGELOG.md` | — | — |
| 5 | `feat(mesh-printability): split overhangs into connected regions (Gap D)` | §5.3 (Gap D consumes helper from #4) | `validation.rs::check_overhangs`, `validation.rs::tests`, `CHANGELOG.md` | — | — |
| 6 | `feat(mesh-printability): tighten ExcessiveOverhang severity policy (Gap E)` | §5.4 | `validation.rs::check_overhangs`, `validation.rs::tests`, `CHANGELOG.md` | — | — |
| 7 | `feat(mesh-printability): detect inconsistent winding orientation (Gap F)` | §5.5 | `validation.rs::check_basic_manifold`, `validation.rs::tests`, `CHANGELOG.md` | — | — |
| 8 | `feat(mesh-printability): parametrize build_up_direction (Gap L)` | §5.6 | `config.rs`, `validation.rs::check_overhangs`, `orientation.rs`, `config.rs::tests`, `CHANGELOG.md` | — | — |
| 9 | `feat(mesh-printability): add DetectorSkipped issue variant (§5.8)` | §5.8 | `issues.rs`, `issues.rs::tests`, `lib.rs` (re-export), `CHANGELOG.md` | — | — |
| 10 | `feat(mesh-printability): ThinWall detector via inward ray-cast (Gap C) + tests-release CI for stress fixtures` | §6.1 + §9.2.3 + §10.4.2 CI extension | `validation.rs::check_thin_walls` (new), `regions.rs`, `lib.rs`, `tests/stress_inputs.rs` (append), `.github/workflows/quality-gate.yml` (append `-p mesh-printability` to tests-release), `CHANGELOG.md` | **High** (§8.4 cluster-split topology) | — |
| 11 | `feat(examples): mesh-printability-thin-wall visual demo (Gap C)` | §7.1 | `examples/mesh/printability-thin-wall/{Cargo.toml, README.md, src/main.rs}` (new), workspace `Cargo.toml` (member add), `CHANGELOG.md` | — | ⏸︎ |
| 12 | `feat(mesh-printability): LongBridge detector via boundary-edge span analysis (Gap G)` | §6.2 + §9.2.4 | `validation.rs::check_long_bridges` (new), `regions.rs`, `lib.rs`, `tests/stress_inputs.rs` (append), `CHANGELOG.md` | — | — |
| 13 | `feat(examples): mesh-printability-long-bridge visual demo (Gap G)` | §7.2 | `examples/mesh/printability-long-bridge/...` (new), workspace `Cargo.toml`, `CHANGELOG.md` | — | ⏸︎ |
| 14 | `feat(mesh-printability): TrappedVolume detector via exterior flood-fill (Gap H) + cross-os CI + voxel-grid memory cap` | §6.3 (incl. OOM amendment from §9.2.5) + §9.2.5 + §10.4.1 CI extension | `validation.rs::check_trapped_volumes` (new), `regions.rs`, `lib.rs`, `tests/stress_inputs.rs` (append), `.github/workflows/quality-gate.yml` (append `-p mesh-printability` to cross-os), `CHANGELOG.md` | **High** (§8.4 FP drift + grid memory) | — |
| 15 | `feat(examples): mesh-printability-trapped-volume visual demo (Gap H)` | §7.3 | `examples/mesh/printability-trapped-volume/...` (new), workspace `Cargo.toml`, `CHANGELOG.md` | — | ⏸︎ |
| 16 | `feat(mesh-printability): SelfIntersecting detector via mesh-repair re-use (Gap I)` | §6.4 + §9.2.6 + dep add | `validation.rs::check_self_intersecting` (new), `regions.rs`, `lib.rs`, `Cargo.toml` (mesh-repair dep), `tests/stress_inputs.rs` (append), `CHANGELOG.md` | **High** (§8.4 Layer Integrity dep cap) | — |
| 17 | `feat(examples): mesh-printability-self-intersecting visual demo (Gap I)` | §7.4 | `examples/mesh/printability-self-intersecting/...` (new), workspace `Cargo.toml`, `CHANGELOG.md` | — | ⏸︎ |
| 18 | `feat(mesh-printability): SmallFeature detector via connected-component bbox extent (Gap J)` | §6.5 + §9.2.7 | `validation.rs::check_small_features` (new), `regions.rs`, `lib.rs`, `tests/stress_inputs.rs` (append), `CHANGELOG.md` | — | — |
| 19 | `feat(examples): mesh-printability-small-feature visual demo (Gap J)` | §7.5 | `examples/mesh/printability-small-feature/...` (new), workspace `Cargo.toml`, `CHANGELOG.md` | — | ⏸︎ |
| 20 | `test(mesh-printability): cross-detector stress fixtures (§9.3)` | §9.3 (6 cross-detector fixtures) | `tests/stress_inputs.rs` (append), `CHANGELOG.md` | — | — |
| 21 | `feat(examples): mesh-printability-orientation visual demo (Gap L)` | §7.6 | `examples/mesh/printability-orientation/...` (new), workspace `Cargo.toml`, `CHANGELOG.md` | — | ⏸︎ |
| 22 | `feat(examples): mesh-printability-technology-sweep visual demo (cross-tech)` | §7.7 | `examples/mesh/printability-technology-sweep/...` (new), workspace `Cargo.toml`, `CHANGELOG.md` | — | ⏸︎ |
| 23 | `feat(examples): mesh-printability-showcase visual demo (capstone)` | §7.8 | `examples/mesh/printability-showcase/...` (new), workspace `Cargo.toml`, `CHANGELOG.md` | — | ⏸︎ |
| 24 | `release(mesh-printability): v0.8.0 + COMPLETION.md rewrite (Gap K)` | version bump + Gap K rewrite + CHANGELOG release pin + end-of-arc grading checkpoint (§10.6) + memo + book migration per §13 | `Cargo.toml` (`0.7.0` → `0.8.0`), `COMPLETION.md` (Gap K rewrite per §5.7), `CHANGELOG.md` (close `[Unreleased]` → `[0.8.0]`), `project_mesh_printability_gaps.md` (memo rewrite), `docs/studies/mesh_architecture/src/50-shell-and-print.md` (depth-pass section) | — | — |
| 25 | `chore(mesh-printability): delete v0.8 fix arc spec (feedback_code_speaks)` | spec deletion (final commit) | `mesh/mesh-printability/V08_FIX_ARC_SPEC.md` (deleted) | — | — |

**Pause-for-visuals count: 8.** Per `feedback_one_at_a_time_review`'s two-pass rule (numbers-pass + visuals-pass), each ⏸︎ row blocks until the user has opened the produced PLY artifacts in their viewer of choice and reported the verdict.

**High-tier commits: 4** (#2 Gap M, #10 Gap C, #14 Gap H, #16 Gap I). Each runs §8.4 pre-flight verification before authoring src + a thorough re-read pass + risk-mitigation pass before commit lands.

### §12.2 Per-commit message template

All commits use the workspace-standard `type(scope): subject` format per `feedback_commit_message_format` (the pre-commit hook rejects formats it can't parse).

```
<type>(<scope>): <subject>

<body — 1–3 paragraphs explaining the WHY + the load-bearing mechanics>

<acceptance summary — bullet list of what gates passed>

[Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>]
```

`<type>` ∈ {`feat`, `fix`, `refactor`, `chore`, `docs`, `test`, `release`}. Scope is `mesh-printability` for src/test changes, `examples` for example crates.

**Examples** (per §12.1):

- `fix(mesh-printability): correct overhang predicate to FDM convention + build-plate filter (Gap M)`
- `feat(mesh-printability): ThinWall detector via inward ray-cast (Gap C) + tests-release CI for stress fixtures`
- `feat(examples): mesh-printability-thin-wall visual demo (Gap C)`
- `release(mesh-printability): v0.8.0`

**Multi-fix commits** name primary in the subject and itemize secondaries in the body. E.g., commit #15's body opens with the TrappedVolume detector mechanics, then itemizes the cross-os CI extension + the §6.3 OOM amendment.

**No `!` breaking-change marker** (the pre-commit hook rejects it per `feedback_commit_message_format`); semver-significant changes go into the commit body's `BREAKING:` paragraph + the `CHANGELOG.md` entry.

### §12.3 Per-commit acceptance criteria template

Every commit, regardless of type, runs **all** of these gates locally before committing:

```
□ cargo fmt --all (auto-applied; pre-commit hook enforces)
□ cargo clippy -p mesh-printability --tests --all-targets -- -D warnings (post Gap A)
□ cargo test -p mesh-printability (debug)
□ RUSTDOCFLAGS=-D warnings cargo doc --no-deps -p mesh-printability
□ cargo xtask grade mesh-printability --skip-coverage  →  A across 7 criteria
□ cargo build --workspace  (catches workspace-member regressions)
```

**Detector commits (§6) additionally run**:

```
□ At least one stress fixture from §9.2.<gap> is exercised
□ All edge cases from §6.<gap>'s "Edge cases" section have a covering test
```

**Example commits (§7) additionally run**:

```
□ cargo run -p example-mesh-printability-<name> --release  →  exit 0
□ out/*.ply files written; ASCII PLY readable by MeshLab/ParaView/Blender
□ ⏸︎ pause until user signals visuals-pass complete
```

**High-tier commits additionally run pre-flight gates**:

| Commit | Pre-flight (per §8.4 + §10.1) |
|--------|--------------------------------|
| #2 Gap M | Read `check_overhangs` + `evaluate_orientation` predicates in current code; verify FDM convention match (PrusaSlicer/Cura semantic comparison) |
| #10 Gap C | §7.1 fixture topology audit; verify 2-cluster outcome theoretical |
| #14 Gap H | (post-amendment) memory-cap arithmetic verified for FDM defaults; cross-os CI matrix change reviewed |
| #16 Gap I | `cargo xtask grade mesh-printability --skip-coverage` AFTER mesh-repair dep add but BEFORE any src change consuming it; verify Criterion 6 stays A AND `cargo build --target wasm32-unknown-unknown -p mesh-printability` passes |

**v0.8.0 release commit (#24) additionally runs the full §10.6 end-of-arc grading checkpoint** (Coverage % vs §11.2 baseline; workspace `grade-all`; `cargo build --workspace --release`; release-stress fixtures pass; rustdoc clean) AND finalizes Gap K's `COMPLETION.md` rewrite per §5.7 with the full v0.8 detector inventory.

**Spec deletion commit (#25) requires no gate beyond CI green** — the spec file is removed; no src or test changes.

### §12.4 Per-commit details (cross-references to upstream sections)

Each commit's authoritative content is the upstream spec section. Per-commit notes here cover only what's specific to the commit boundary.

#### Commit #1a — Gap A sub-1: mechanical-only fallout (24 sites, no semantics change)

Per §5.1 (mechanical-only category).

**Specific to this commit**:
- 19 `clippy::missing_const_for_fn` — add `const` keyword.
- 3 `clippy::cast_precision_loss` — add `#[allow]` + one-line justification.
- 1 `clippy::uninlined_format_args` — inline the format arg.
- 1 `clippy::match_same_arms` — collapse arms.
- No Cargo.toml change (lints not yet inherited; clippy still red).
- No CHANGELOG.md change (not yet created).
- Body names the 24 sites changed by category (4-line itemization).

**Acceptance**:
- `cargo build -p mesh-printability --tests --all-targets` clean
- `cargo test -p mesh-printability` 35/35 pass
- (Clippy intentionally NOT run — lints not yet inherited; clippy will still report the remaining 14 sites)

#### Commit #1b — Gap A sub-2: test-site cleanup (5 expect_used sites)

Per §5.1 (test-site category).

**Specific to this commit**:
- 5 `clippy::expect_used` sites in `validation.rs::tests` — per-site decision: either `#[allow(clippy::expect_used)]` with one-line justification, or restructure to non-panicking (e.g., `assert!(matches!(...))` instead of `.expect()`).
- No Cargo.toml change. No CHANGELOG.md change.
- Body names the 5 test sites and which got `#[allow]` vs restructure.

**Acceptance**:
- `cargo test -p mesh-printability` 35/35 pass
- (Clippy intentionally NOT run — lints not yet inherited)

#### Commit #1c — Gap A sub-3: workspace lints flip + per-site FP allows + CHANGELOG creation + Coverage baseline

Per §5.1 + §5.10 + §11.2.

**Specific to this commit**:
- Adds `[lints] workspace = true` to `mesh/mesh-printability/Cargo.toml` after `[package.metadata.cortenforge]`.
- 8× `#[allow(clippy::suboptimal_flops)]` per-site annotations on the FMA-equivalent sites in `validation.rs` (lines 227, 235, etc.) and `orientation.rs` (line 387, etc.) — each with the §5.1 justification-comment template adapted per site.
- 1× `#[allow(clippy::manual_midpoint)]` per-site annotation with the analogous comment.
- Creates `mesh/mesh-printability/CHANGELOG.md` per §5.10 initial structure.
- §11.2 one-time Coverage baseline run (5–10 min): `cargo xtask grade mesh-printability` (without `--skip-coverage`) on the post-1c HEAD. Record absolute % in commit body. (Pre-1a baseline is functionally equivalent for Coverage since 1a/1b only touch test bodies + add `const` keywords; 1c is chosen for the capture so the recorded baseline is post-lints-flip, matching every subsequent commit's grading state.)
- First `[Unreleased] / Changed` CHANGELOG entry: "Inherited workspace lints; 9 per-site `#[allow]`s preserve FP semantics for overhang-predicate sites (FMA + midpoint forms changed FP bits; deferred as v0.9 candidate per §11.5)."
- §5.1 stop-and-raise gate documented exception: 38 sites is 3.8× the >10 gate; the 3-commit split (commits 1a/1b/1c) is the resolution; the gate stays in effect for any future Gap-A-shaped lint flip in another crate.

**Body template**:

```
chore(mesh-printability): inherit workspace lints + per-site FP-semantics allows + create CHANGELOG (Gap A sub-3)

Adds [lints] workspace = true to mesh-printability/Cargo.toml. Per-site
allows preserve FP semantics on 8 mul_add + 1 midpoint sites where FMA
would shift overhang-predicate FP bits at the threshold boundary;
bit-exactness deferred as v0.9 candidate per §11.5.

Creates CHANGELOG.md with the keepachangelog.com structure; subsequent
commits append entries.

v0.7 coverage baseline: <X>%  (cargo xtask grade mesh-printability,
without --skip-coverage, on feature/mesh-printability-v0-8 HEAD post-1c)

Acceptance:
- cargo clippy -p mesh-printability --tests --all-targets -- -D warnings: clean
- cargo xtask grade mesh-printability --skip-coverage: A across 7
- cargo test -p mesh-printability: 35/35 pass
- mesh/mesh-printability/CHANGELOG.md exists with [Unreleased] block
```

#### Commit #2 — Gap M overhang predicate + build-plate filter + stress_inputs.rs creation

Per §5.9 + §9.2.1 + §9.2.2.

**Specific to this commit**:
- Creates `tests/stress_inputs.rs` (new file) with §9.2.1 existing-detector stress fixtures + §9.2.2 Gap M fixtures.
- §5.9 test-anchor regression sweep: every existing `validation.rs::tests` test asserting on overhang-derived numerics gets refreshed inline.
- Symmetric edits in `validation.rs` AND `orientation.rs`; commit-time grep verifies both `dot < 0.0` early-out lines removed.
- Pre-flight per §8.4: read both predicates in current code; document the FDM-convention semantic match in the commit body.

#### Commit #3 — Gap B max-angle tracking

Per §5.2.

**Specific to this commit**: small in-place change; the `OverhangRegion.angle` field semantic now reads "actual maximum observed". Existing tests adapted as needed (§5.2 acceptance).

#### Commit #4 — `build_edge_to_faces` helper extraction (separate refactor)

Per §5.3 sub-step.

**Specific to this commit**:
- New private helper `fn build_edge_to_faces(mesh: &IndexedMesh) -> HashMap<(u32, u32), Vec<u32>>` factored from `check_basic_manifold`.
- §5.3 regression gate: `test_not_watertight_detection`, `test_watertight_mesh`, `test_validation_summary`, `test_issue_counts`, `test_sls_no_overhang_check` all pass without modification.
- Pure refactor; zero behavior change. The diff IS the review surface.

#### Commit #5 — Gap D overhang region split (consumes helper)

Per §5.3 (Gap D portion).

**Specific to this commit**: Gap D consumes the helper from commit #4. Region-split logic + 5 §5.3 tests + `validation.support_regions.len()` == `validation.overhangs.len()` invariant.

#### Commit #6 — Gap E severity tightening

Per §5.4. `classify_overhang_severity` central helper added.

#### Commit #7 — Gap F winding orientation in manifold check

Per §5.5. Directed-edge tracking added to `check_basic_manifold`. `test_watertight_mesh` regression risk per §8.1 Gap F High-tier table — pre-flight: read `create_watertight_cube` source pre-commit and confirm CCW-from-outside winding.

#### Commit #8 — Gap L `build_up_direction` parametrization

Per §5.6. New field + builder + propagation through `check_overhangs` and `evaluate_orientation`. All 4 `*_default()` constructors updated.

#### Commit #9 — DetectorSkipped variant

Per §5.8. New `PrintIssueType::DetectorSkipped` variant + `as_str()` arm. Lands BEFORE the first detector with skip behavior (Gap C, commit #11).

#### Commit #10 — Gap C ThinWall + tests-release CI extension

Per §6.1 + §9.2.3 + §10.4.2.

**Specific to this commit**:
- New private helper: Möller-Trumbore ray-tri intersection (~30 LOC).
- Pre-flight per §8.4: §7.1 fixture topology audit before §6.1's algorithm authoring.
- `.github/workflows/quality-gate.yml` `tests-release` job step gets `-p mesh-printability` appended (one-line edit).
- The first stress fixture marked `#[cfg_attr(debug_assertions, ignore)]` (`stress_c_5k_tri_perf_budget`) lands here; verify it runs in `cargo test --release -p mesh-printability` before commit.

#### Commit #11 — §7.1 thin-wall example ⏸︎

Per §7.1 + §7.0 cadence.

**Specific to this commit**:
- New example crate at `examples/mesh/printability-thin-wall/`.
- Workspace `Cargo.toml` `[workspace.members]` updated.
- `out/` written at runtime, gitignored via `examples/mesh/.gitignore`.
- ⏸︎ Pause for user visuals-pass.

#### Commits #12–#19 — Detector + example pairs

Pattern same as #10+#11. Each detector commit appends its §9.2.x stress fixtures to `tests/stress_inputs.rs`. Each example commit adds a workspace member + an `out/` artifact set.

**Special: Commit #14 (Gap H)**:
- Implements §6.3 with the §9.2.5 OOM amendment (1 GB voxel-grid cap → DetectorSkipped) baked in from commit-time, NOT as a follow-up.
- `.github/workflows/quality-gate.yml` `cross-os` job step gets `-p mesh-printability` appended.
- ROW_JITTER constant declared with cross-platform-stability doc-comment.

**Special: Commit #16 (Gap I)**:
- Pre-flight gate per §8.4: split into two stages.
  - Stage 1: `Cargo.toml` mesh-repair dep add + run `cargo xtask grade mesh-printability --skip-coverage` to verify Layer Integrity stays A. If it fails, **stop and raise**.
  - Stage 2: src changes consuming mesh-repair land only after Stage 1 confirms.
- Both stages bundled in commit #16 as a single squash unit; the staging is logical, not commit-boundary.

#### Commit #20 — §9.3 cross-detector stress fixtures

Per §9.3 + §9.5.

**Specific to this commit**: 6 cross-detector fixtures appended to `tests/stress_inputs.rs`. No detector or example changes; pure test additions exercising multi-detector behavior. Lands AFTER all detectors (commit #18) so all fixtures' assertions are valid; lands BEFORE the orientation example (commit #21) per §9.5.

#### Commits #21–#23 — Orientation, technology-sweep, showcase examples ⏸︎

Per §7.6, §7.7, §7.8. All ⏸︎ pause-for-visuals.

#### Commit #24 — v0.8.0 release commit + Gap K COMPLETION.md rewrite

Per §10.6 + §5.7 (Gap K) + §13.

**Specific to this commit**:
- `mesh-printability/Cargo.toml`: `version = "0.7.0"` → `"0.8.0"`. Umbrella `mesh/mesh/Cargo.toml` unchanged per §11.1.
- **Gap K rewrite**: `COMPLETION.md` rewritten per §5.7 with full v0.8 detector inventory, accurate test counts (~233 + 47 stress = ~280), workspace-lints status, dependency list including mesh-repair, known limitations + v0.9 followups. Lands here (not earlier) per §12.0 row #4 reconciliation: truth is settled only after all detectors + examples + cross-detector stress fixtures land.
- `CHANGELOG.md`: close `[Unreleased]` → `[0.8.0] - YYYY-MM-DD` (`date -u +%Y-%m-%d` at commit-author time).
- §10.6 end-of-arc grading checkpoint runs (Coverage compared against §11.2 baseline; workspace `grade-all`; release-stress fixtures pass).
- Memo update: `project_mesh_printability_gaps.md` rewritten as "v0.8 closed + v0.9 backlog with triggers" per §13.
- Mesh book §50 depth-pass section authored per §13.

**Note**: this commit bundles 5 distinct concerns (version bump, Gap K, CHANGELOG pin, memo rewrite, book section). Per `feedback_pr_size_ci_economics`, internal commit segmentation matters but a release commit's job IS to bundle all release-correlated edits. Each concern is small (≤200 LOC) and they are inherently co-changed (you don't ship v0.8.0 with stale COMPLETION.md or stale memo). Keeping them in one commit makes the release atom reviewable.

#### Commit #25 — Spec deletion (final commit)

Per `feedback_code_speaks` + §13.

**Specific to this commit**:
- `mesh/mesh-printability/V08_FIX_ARC_SPEC.md` deleted.
- No other changes.
- Pre-squash tag per `feedback_pre_squash_tag` ships at this point; see §12.5.

### §12.5 Git operations

**Branch**: `feature/mesh-printability-v0-8` (already cut from main 2026-04-30; 9 spec-authoring commits + this 5-spec-section reconciliation already committed; remaining 27 implementation commits land on this branch — 25 base slots + 2 added by Gap A's 3-commit split per §5.1).

**Commit cadence**: ask-before-commit per `feedback_master_architect_delegation` standard counterweights. After each commit lands, surface a one-line confirmation to the user; for ⏸︎ commits, explicitly wait for visuals-pass signal before next.

**Push cadence**: push after each detector + example pair (so commits #10+#11 → push; #12+#13 → push; etc.). Pre-PR commit lands as the pre-squash tag. Avoids long branches without remote backups.

**Pre-squash tag** per `feedback_pre_squash_tag`:

```
git tag -a feature/mesh-printability-v0-8-pre-squash -m "v0.8 fix arc — pre-squash 27-commit audit trail (Gap A 3-commit split per §5.1)"
git push origin feature/mesh-printability-v0-8-pre-squash
```

Tag lands after commit #25 (spec deletion), before opening the PR. Preserves the per-commit audit trail when the merge squashes.

**PR + squash-merge**:

```
gh pr create --title "feat(mesh-printability): v0.8 fix arc — 13 gap fixes + 5 detectors + 8 examples" \
  --body "..."
# Wait for CI green (per reference_ci_timing: ~5 min fast tier; ~25 min total)
# Squash-merge via web UI or `gh pr merge --squash --delete-branch`
```

PR body cross-references the pre-squash tag for the audit trail.

**Working-state restoration if the arc is ever re-opened**: `git checkout feature/mesh-printability-v0-8-pre-squash` recovers the 27-commit history. The deleted spec is recoverable from any pre-#25 commit.

### §12.6 Pause cadence + user-visible coordination

The 8 ⏸︎ pause-points are where the cadence requires user attention:

| Pause | After commit | Concept exercised | What user reviews |
|-------|--------------|--------------------|----------------------|
| 1 | #11 (§7.1 thin-wall) | Edge-adjacency clustering on outer/inner topology | `out/mesh.ply`, `out/issues.ply` |
| 2 | #13 (§7.2 long-bridge) | Bbox-based span detection on H-shape | `out/mesh.ply`, `out/issues.ply` |
| 3 | #15 (§7.3 trapped-volume) | Voxel flood-fill + tech-aware severity | `out/mesh.ply`, `out/issues.ply`, `out/voxels.ply` |
| 4 | #17 (§7.4 self-intersecting) | mesh-repair re-use through validate-pipeline | `out/mesh.ply`, `out/issues.ply` |
| 5 | #19 (§7.5 small-feature) | Connected-component bbox + on-plate burr placement | `out/mesh.ply`, `out/issues.ply` |
| 6 | #21 (§7.6 orientation) | `find_optimal_orientation` + `build_up_direction` parametrization equivalence | `out/mesh_original.ply`, `out/mesh_rotated.ply`, `out/issues_run*.ply` |
| 7 | #22 (§7.7 technology-sweep) | Same mesh, four-tech severity divergence | `out/mesh.ply`, `out/issues_<tech>.ply` × 4 |
| 8 | #23 (§7.8 showcase) | Multi-detector report on a realistic CAD bracket | `out/mesh.ply`, `out/issues.ply` |

Per `feedback_one_at_a_time_review`'s two-pass protocol: numbers-pass (Claude verifies `cargo run` exits 0 + assertions pass) is the precondition for the pause; visuals-pass (user opens PLY in viewer) is the resume gate.

**Each pause's expected user action**: open the artifact set, verify the README's "what to look for" prose matches the rendered geometry, signal "visuals-pass complete" or report regression. Implementation does not advance without this signal.

**Estimated pause time per pass**: 5–10 min for user viewer interaction. 8 pauses × 7.5 min ≈ 1 hour of user attention spread across the arc (per `feedback_patience_track_record`).

### §12.7 What §12 does NOT cover

For audit clarity, items that look like they should be in §12 but aren't:

- **Per-commit content** (algorithms, test names, tolerances): authoritative source is §5 / §6 / §7 / §9. §12 references but doesn't duplicate.
- **CI workflow body** for §10.4.1 / §10.4.2 yaml edits: authoritative source is §10.4. §12 says "where" the edit lands; §10.4 says "what" the edit is.
- **`COMPLETION.md` content**: authoritative source is §5.7. §12 says "when" it's authored (commit #24, bundled into the release per §12.0 row #4 reconciliation).
- **v0.8.0 version bump rationale**: authoritative source is §3 + §11.1. §12 says commit #24 makes the edit.
- **Memo + book migration content**: authoritative source is §13 (next).

---

## §13. Spec lifecycle

This spec is a **working document**. It exists for the duration of the v0.8 fix arc, captures the design + reconciles in-flight decisions, and then **gets deleted in the final commit** of the arc per `feedback_code_speaks` ("delete completed specs; the code IS the documentation"). Durable narrative migrates to three locations at PR-close.

### §13.0 Why delete?

`feedback_code_speaks` captures the principle: completed specs decay into stale liabilities. They duplicate information that lives in code, tests, READMEs, CHANGELOG, and project memos — and they drift when those authoritative sources update. Keeping the v0.8 spec around after the arc closes would produce three failure modes:

1. **Stale references**: a future contributor reads the spec, finds it referencing "the §6.3 1 GB voxel cap" — but maybe v1.1 raised it to 2 GB. Spec is wrong; code is right; reader gets confused.
2. **Authority confusion**: someone debugging a Gap M edge case opens the spec for context. The spec describes the design intent; the code describes current behavior. Which wins? Code always wins, but the spec invites the question.
3. **Decay incentive**: nobody is incentivized to update a spec for a closed arc. It gets stale fast.

The mitigation is **delete + migrate**. Spec content that's still load-bearing post-arc moves into authoritative homes (code, tests, project memo, mesh book, CHANGELOG); spec content that was scaffolding (review-pass methodology notes, intermediate reasoning) gets dropped.

### §13.1 Migration targets — three locations

| # | Target | What it absorbs | Lifetime |
|---|--------|-----------------|----------|
| 1 | `project_mesh_printability_gaps.md` (auto-memory) | Rewritten as "v0.8 closed + v0.9 backlog with triggers". Re-open triggers per §11.5; lessons learned; cross-session continuity for v0.9 planning. | Persists across Claude conversations until v0.9 arc opens. |
| 2 | `docs/studies/mesh_architecture/src/50-shell-and-print.md` (mesh book) | Depth-pass section on what's now implemented + what's deferred (v0.9 candidates with triggers). Rendered as part of the mesh architecture book. | Workspace doc; persists indefinitely. |
| 3 | `mesh/mesh-printability/CHANGELOG.md` `[Unreleased]` block (after `[0.8.0]` is closed in commit #24) | Items already triple-tracked in §1 + §11.5 — BVH acceleration, tunable IntersectionParams, AttributedMesh face-attributes, cavity-aware overhang severity, drainage simulation, etc. | Code-adjacent; updates with each release. |

Triple-tracking these v0.9 candidates in three places (memo + book + CHANGELOG) seems redundant but isn't — each location serves a different reader:

- **Memo**: cross-session AI context for future Claude conversations on the v0.9 arc.
- **Book**: human contributors browsing the mesh architecture for "what's missing".
- **CHANGELOG**: downstream consumers reading the crate's own changelog file for "what's planned".

The same fact (e.g., "BVH for ThinWall is v0.9 candidate; trigger: real mesh exceeds 10k tris") lives in all three with the same wording — drift between them is a regression that any of the three's edits can catch.

### §13.2 Per-section migration map

For each spec section, what migrates where (or doesn't migrate at all). "Drop" = scaffolding; not load-bearing post-arc.

| Section | Topic | Migrates to |
|---------|-------|-------------|
| §1 Overview & gap inventory | Gap A–M definitions | Memo (one-line summary of "v0.8 closed gaps A–M"); CHANGELOG `[0.8.0]` Fixed/Added entries (already populated commit-by-commit). Detail dropped. |
| §1 Out-of-scope (BVH, tunable params, etc.) | v0.9 candidates with triggers | Memo (re-open triggers table per §11.5); book (Known limitations subsection); CHANGELOG `[Unreleased]` block. **Triple-tracked**. |
| §2 Pre-flight audit | Audit findings F1–F15 | Drop (scaffolding for arc planning; not consumed post-arc). |
| §3 API surface diff | v0.7→v0.8 API changes | CHANGELOG `[0.8.0]` Added/Changed entries (already populated commit-by-commit); `COMPLETION.md` (Gap K rewrite) detector inventory + dependencies. |
| §4 Cross-cutting policies | Tolerance constants, severity, determinism, FP-stability | Code (constants in `validation.rs` doc-comments); `COMPLETION.md` "Quality" section. |
| §5 Per-fix specs (A, B, D, E, F, K, L, M + DetectorSkipped + CHANGELOG) | Implementation specs for in-place fixes | Code (the fixes themselves, with doc-comments); rustdoc on new types/methods. Detail dropped. |
| §6 Per-detector specs (C, G, H, I, J) | Algorithm + edge-case specs for new detectors | Code (detector source); rustdoc on `check_*` fns + region types; `tests/` for the test suites. Detail dropped. |
| §7 Example design | 8 example crates with main() assertions + READMEs | Each example's own `examples/mesh/printability-<name>/README.md` (museum-plaque per `feedback_museum_plaque_readmes`); the example crates themselves. Detail dropped. |
| §8 Risk inventory | Per-gap + cross-cutting risk audit | Memo (cross-platform FP-drift lesson generalized; Phase-4-faer-LDLᵀ pattern continuity); book (Known limitations + edge cases). High-tier risks: pre-flight verifications already executed at commit-time. Detail dropped. |
| §9 Stress-test gauntlet | 47 stress fixtures + §6.3 OOM amendment | Code (`tests/stress_inputs.rs` itself); §6.3 amendment baked into `validation.rs::check_trapped_volumes`. Detail dropped. |
| §10 Grading & CI impact | Per-criterion grade impact + CI matrix changes | `.github/workflows/quality-gate.yml` edits; `COMPLETION.md` "Quality" section. End-of-arc grading checkpoint output recorded in commit #24's body. Detail dropped. |
| §11 Open questions | Resolved decisions (umbrella, coverage baseline, tier) | Memo (rationale for v0.9 re-open triggers); for the umbrella+coverage+tier resolutions specifically, the rationale stays in commit #24's message body. Detail dropped. |
| §12 Implementation order | 27-commit canonical order (Gap A 3-commit split per §5.1) | Audit trail preserved via the 27-commit history at the **pre-squash tag** (§13.4). Detail dropped. |
| §13 Spec lifecycle | Migration plan | This section — drops itself in the deletion commit. |

The migration map ensures **every spec fact that's load-bearing post-arc has a durable home**. Scaffolding (review methodology, intermediate reasoning, planning artifacts) gets dropped — because it's already served its purpose.

### §13.3 Memo migration content

`project_mesh_printability_gaps.md` is rewritten in commit #24 to its post-arc form. Sketch:

```markdown
---
name: mesh-printability v0.8 closed + v0.9 backlog
description: v0.8 fix arc CLOSED 2026-XX-XX. 13 gaps A–M shipped + 5 detectors + 8 examples + 47 stress fixtures. v0.9 backlog with explicit re-open triggers.
type: project
---

## v0.8 closed (2026-XX-XX)
- Gaps A–M all shipped; release commit `<hash>`; PR #<num>
- 5 new detectors populated (ThinWall + LongBridge + TrappedVolume + SelfIntersecting + SmallFeature)
- 8 examples shipped under `examples/mesh/printability-*`
- 47 stress fixtures in `tests/stress_inputs.rs`
- Cross-os CI extended (Gap H FP-drift coverage); tests-release extended (release-only stress fixtures)
- Pre-squash audit trail: tag `feature/mesh-printability-v0-8-pre-squash`

## v0.9 backlog (with re-open triggers per §11.5)
1. BVH for ThinWall + SelfIntersecting — trigger: real mesh exceeds 10k tris with >5 s validation runtime
2. Tunable IntersectionParams from validate_for_printing — trigger: a caller asks for the overload
3. AttributedMesh face-attributes for issue annotations — trigger: user reports centroid point-cloud insufficient
4. Cavity-aware overhang severity (interior face detection) — trigger: user requests separating cavity ceilings from exterior overhangs
5. Drainage simulation along build_up_direction — trigger: user reports cavity that drains in one orientation
6. Adaptive voxel sizing for >100mm parts — trigger: user reports >100mm part being silently DetectorSkipped
7. Cantilever distinction in LongBridge — trigger: user reports false positives on cantilevered geometry
8. OBB-based bridge span — trigger: user reports diagonal bridges underflagged
9. Volume-based threshold in SmallFeature — trigger: long thin spike (large extent, tiny volume) user-reported
10. Curvature-based small-feature detection — trigger: user reports small bump on larger body not flagged
11. Self-intersection auto-fix integration with mesh-repair — trigger: workflow asks for "validate-and-fix" semantics
12. Mesh umbrella version decoupling from workspace.package — trigger: first workspace crate publishes to crates.io
13. Per-detector coverage breakdown — trigger: user asks for it
14. Tracing instrumentation in mesh-printability — trigger: structured-diagnostics demand
15. Example main() in CI test gate — trigger: a regression on an example lands without local catch

## Lessons learned (cross-session continuity)
- Master-architect delegation worked. 5 high-tier risks pre-flight-verified; 4 inline scope expansions surfaced via risk-mitigation review (Gap M.2 build-plate filter, §6.3 OOM amendment, §10.4.1 cross-os, §10.4.2 tests-release).
- §-internal inconsistency reconciliation: §12.0 absorbed 4 conflicts (build_edge_to_faces refactor placement; CHANGELOG creation slot; detector↔example interleave; Gap K placement). Pattern: when in doubt, prefer the more pragmatic of two readings + bake the rationale.
- Cross-platform FP-drift lesson generalized from Phase 4 faer-LDLᵀ: tolerance bands + exact-representable inputs + cross-os CI matrix is the durable mitigation for any ray-casting/voxel-fill/divergence-theorem operation.
- Commit cadence: 27 commits across the arc (Gap A 3-commit split per §5.1 lifts the count from 25 to 27; sub-numbering preserves downstream commit-# references); ⏸ pause-for-visuals at 8 example commits per `feedback_one_at_a_time_review`. Internal commit segmentation (vs splitting PRs) aligns with `feedback_pr_size_ci_economics`.

## Cross-references
- Mesh architecture book Part 5: `docs/studies/mesh_architecture/src/50-shell-and-print.md`
- CHANGELOG: `mesh/mesh-printability/CHANGELOG.md` (`[0.8.0]` + `[Unreleased]` block)
- Pre-squash tag: `git checkout feature/mesh-printability-v0-8-pre-squash`
```

The memo's pre-arc form (`project_mesh_printability_gaps.md` as it stands 2026-04-30) is **deleted** as part of the rewrite — there's exactly one memo file at this location, before and after.

### §13.4 Mesh book §50 depth-pass content

`docs/studies/mesh_architecture/src/50-shell-and-print.md` is currently a skeleton (per the mesh-architecture-book memo). The depth-pass section on shell-and-print landing in commit #24 covers:

- v0.8 inventory: 5 detectors populated; severity policy; technology mapping (FDM/SLA/SLS/MJF).
- Worked example: a hollow box with thin walls — read through the validator's report (`validate_for_printing`).
- Known limitations (mirroring v0.9 backlog above): BVH absent; pinhole leaks not detected; cavity-ceiling co-flag; etc.
- Cross-references to the 8 example crates as runnable demos.

The book section is **prose, not spec** — explains the architecture for a human reader. Distinct from the memo (AI cross-session context) and the CHANGELOG (versioned release log).

### §13.5 CHANGELOG migration content

By commit #24, `CHANGELOG.md` has accumulated 24 commit-by-commit `[Unreleased]` entries: one per arc commit (commits #2 through #24 = 23 entries), plus the initial entry that landed when commit 1c created the file (= 1 entry, "Inherited workspace lints; 9 per-site allows for FP-semantics-preserving sites"). Sub-commits 1a + 1b predate CHANGELOG.md creation (no entry); commit #25 deletes the spec (no entry). Total = 1 + 23 = 24 entries — unchanged from pre-split baseline since the split adds 2 commits but those commits are exactly the two pre-CHANGELOG-creation slots. Commit #24 closes the section. Per `keepachangelog.com` convention, `[Unreleased]` stays at the top + new released sections nest below in descending date order:

```markdown
# Changelog

All notable changes to mesh-printability will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### v0.9 candidates (with triggers)
- BVH for ThinWall + SelfIntersecting — trigger: real mesh exceeds 10k tris with >5 s validation
- Tunable IntersectionParams in validate_for_printing — trigger: caller-overload requested
- AttributedMesh face-attributes for issue annotations — trigger: centroid PLY insufficient
- Cavity-aware overhang severity (interior face detection) — trigger: user separates cavity ceilings
- Drainage simulation along build_up_direction — trigger: orientation-dependent cavity drain
- Adaptive voxel sizing — trigger: >100mm part silently DetectorSkipped
- Cantilever distinction in LongBridge — trigger: false-positive on cantilevered geometry
- OBB-based bridge span — trigger: diagonal bridges underflagged
- Volume-based SmallFeature criterion — trigger: long thin spike not flagged
- Curvature-based SmallFeature detection — trigger: small bump on larger body
- Self-intersection auto-fix integration — trigger: validate-and-fix workflow request
- Per-detector coverage breakdown — trigger: user asks
- Tracing instrumentation — trigger: structured-diagnostics demand
- Example main() in CI test gate — trigger: regression lands without local catch
- Mesh umbrella version decoupling — trigger: first crates.io publication

## [0.8.0] - YYYY-MM-DD

### Added
- ThinWall detector (Gap C) via inward ray-cast
- LongBridge detector (Gap G) via boundary-edge span analysis
- TrappedVolume detector (Gap H) via exterior flood-fill (1 GB voxel-grid memory cap; >cap → DetectorSkipped)
- SelfIntersecting detector (Gap I) via mesh-repair re-use
- SmallFeature detector (Gap J) via connected-component bbox extent
- PrinterConfig.build_up_direction parametrization (Gap L)
- PrintIssueType::DetectorSkipped variant
- 4 new region types (LongBridgeRegion, TrappedVolumeRegion, SelfIntersectingRegion, SmallFeatureRegion)
- 8 example crates (`examples/mesh/printability-*`)
- 47 stress fixtures (`tests/stress_inputs.rs`)
- mesh-repair as a workspace dep
- Cross-OS CI coverage; release-mode CI for stress fixtures

### Changed
- Inherited workspace lints (Gap A)
- ExcessiveOverhang severity policy: angle-graded with Critical for tilt > 75° on FDM (Gap E)
- OverhangRegion.angle reports actual maximum observed (Gap B), not threshold + 10°
- Overhang regions split by edge-adjacency component (Gap D), not collapsed to one
- Build-up direction parametrized via PrinterConfig (default +Z preserved) (Gap L)
- COMPLETION.md rewritten to reflect v0.8 truth (Gap K)

### Fixed
- check_overhangs predicate corrected to FDM convention (Gap M); pure roofs now flagged
- Build-plate filter added to check_overhangs (Gap M.2); solid-on-plate bottoms no longer falsely flagged
- check_basic_manifold detects winding-orientation inconsistency (Gap F)

### Notes (semver-significant behavioral changes)
- Callers asserting `is_printable()` on meshes with severe-but-pre-Gap-E-uncritical overhangs now see `false`. Correctness fix.
- Callers asserting on `validation.overhangs.len()` may see a different count per Gap D split + Gap M predicate change.
- Cavity-ceiling co-flag (sealed cavities flag overhang under Gap M) is documented behavior; v0.9 candidate for cavity-aware severity.

## [0.7.0] - 2026-XX-XX
- Initial release with build-volume, overhang, and basic manifold detectors.
```

The release date `YYYY-MM-DD` resolves at commit-author-time via `date -u +%Y-%m-%d`.

### §13.6 Spec deletion procedure (commit #25)

The arc's final commit:

```
git rm mesh/mesh-printability/V08_FIX_ARC_SPEC.md
git commit -m "chore(mesh-printability): delete v0.8 fix arc spec (feedback_code_speaks)

The v0.8 fix arc spec served as the implementation reference for 25
commits. Per feedback_code_speaks, completed specs decay into stale
liabilities; durable narrative migrated to:
- project_mesh_printability_gaps.md (rewritten in commit #24 as v0.8
  closed + v0.9 backlog)
- docs/studies/mesh_architecture/src/50-shell-and-print.md (depth-
  pass section authored in commit #24)
- mesh/mesh-printability/CHANGELOG.md (per-commit + [0.8.0] release
  pin in commit #24)

The 27-commit audit trail is preserved at the pre-squash tag:
  git checkout feature/mesh-printability-v0-8-pre-squash

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

Acceptance: spec file no longer exists in the working tree post-commit; `git log --follow V08_FIX_ARC_SPEC.md` recovers full authoring history.

### §13.7 Pre-squash tag procedure

Per `feedback_pre_squash_tag`, before opening the PR:

```
# After commit #25 lands locally
git tag -a feature/mesh-printability-v0-8-pre-squash \
    -m "v0.8 fix arc — pre-squash 27-commit audit trail (Gap A 3-commit split per §5.1)

13 gaps A–M shipped; 5 detectors populated; 8 examples; 47 stress
fixtures. §-internal inconsistencies reconciled in §12 (4 absorbed).
Master-architect delegation cadence with 4 high-tier pre-flight
verifications + 4 inline scope expansions surfaced via risk-mitigation
review. Gap A scope landed as 3 sub-commits (1a/1b/1c) per §5.1's
pre-flight 38-site fallout measurement vs the >10 stop-and-raise gate.

Squash-merge will collapse this 27-commit history; this tag preserves
the per-commit audit trail."

git push origin feature/mesh-printability-v0-8-pre-squash
```

Then open the PR + squash-merge:

```
gh pr create --title "feat(mesh-printability): v0.8 fix arc — 13 gap fixes + 5 detectors + 8 examples" \
    --body "$(cat <<'EOF'
## Summary
- Closes 13 v0.7→v0.8 gaps (A–M) including 5 new detectors (ThinWall, LongBridge, TrappedVolume, SelfIntersecting, SmallFeature)
- 8 new example crates under examples/mesh/printability-*
- 47 stress-test fixtures (3 release-only)
- v0.8.0 release commit + COMPLETION.md rewrite (Gap K)

## Test plan
- [x] cargo xtask grade-all --skip-coverage: A across 7 criteria
- [x] cargo test -p mesh-printability: ~233/~233 pass
- [x] cargo test --release -p mesh-printability: 3/3 release-only stress pass
- [x] cargo build --workspace --release: examples compile
- [x] RUSTDOCFLAGS=-D warnings cargo doc --no-deps -p mesh-printability: 0 warnings
- [x] CI: format / grade / tests-debug / tests-release / cross-os / feature-combos / dependencies all green
- [x] Coverage: <X>% (>= v0.7 baseline of <Y>%)

## Notes
- Audit trail preserved at tag `feature/mesh-printability-v0-8-pre-squash`
- Semver-significant: ExcessiveOverhang severity tightening (Gap E) + overhang predicate fix (Gap M); CHANGELOG documents

🤖 Generated with [Claude Code](https://claude.com/claude-code)
EOF
)"
```

Wait for CI green per `reference_ci_timing` (~25 min total). Squash-merge via `gh pr merge --squash --delete-branch` or web UI.

### §13.8 Post-merge audit checklist

After squash-merge lands on `main`, verify the migration succeeded:

- [ ] `mesh/mesh-printability/V08_FIX_ARC_SPEC.md` no longer exists on `main`
- [ ] `project_mesh_printability_gaps.md` exists in user-memory and reads as "v0.8 closed + v0.9 backlog"
- [ ] `docs/studies/mesh_architecture/src/50-shell-and-print.md` has the v0.8 depth-pass section
- [ ] `mesh/mesh-printability/CHANGELOG.md` has `[0.8.0] - YYYY-MM-DD` (dated) + `[Unreleased]` block populated
- [ ] `mesh/mesh-printability/COMPLETION.md` reflects v0.8.0 truth (5 detectors populated, mesh-repair listed as dep)
- [ ] Pre-squash tag `feature/mesh-printability-v0-8-pre-squash` resolves to a commit with all 27 commits visible via `git log` (Gap A 3-commit split per §5.1: 1a + 1b + 1c + 24 base slots = 27)
- [ ] `examples/mesh/printability-*` directories exist (8 examples) and each `cargo run -p example-mesh-printability-<name> --release` exits 0

If any item fails, cherry-pick a fix on top of `main` (post-merge) — the migration is part of the arc's contract.

### §13.9 Why §13 itself migrates / drops cleanly

§13 is the only section where the migration plan IS the section content. Self-referential: §13 documents that §13 (along with the rest of the spec) is deleted in commit #25.

The "drops cleanly" property is the test: after the spec deletion, every fact §13 referenced has a durable home. This section's existence ends with the arc; the action it describes (migration) leaves nothing behind that needs §13 to make sense.

---


