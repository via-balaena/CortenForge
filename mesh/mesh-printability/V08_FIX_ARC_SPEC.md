# mesh-printability v0.8 fix arc — specification

**Status**: drafting (Sections 1–6 authored; Sections 7–13 pending)
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
| F12 | `IntersectionParams` has `default()/exhaustive()/quick_check()` constructors | re-use | wire `default()` (max_reported = 100) into `validate_for_printing`; re-export types for power users |
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

## §7 onwards — pending

(Sections 7–13: example design, risk inventory, stress-test gauntlet, grading & CI impact, open questions, implementation order, spec lifecycle.)
