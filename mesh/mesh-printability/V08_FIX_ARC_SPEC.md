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

Each example is a workspace member with `publish = false`, named `example-mesh-printability-<name>`. Dependencies are minimal: `mesh-types`, `mesh-io`, `mesh-printability`, `anyhow` (for `main() -> Result<()>`), and per-example extras (e.g., `mesh-repair` for the self-intersection fixture, `mesh-shell` or `mesh-sdf` if a fixture needs a non-trivial implicit surface). The `[lints] workspace = true` block is required.

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
2. **`mesh-sdf` + marching cubes** if mesh-sdf exposes a public MC entry point on a closed-form SDF (`max(box_sdf, -inner_box_sdf)` representing solid = outer ∩ ¬inner). Currently mesh-shell uses MC internally; if that's not exposed publicly, option 1 is canonical.
3. **NOT mesh-shell directly** — mesh-shell is offset-based, not CSG-based; not the right tool for box-minus-box.

Hand-author (option 1) is the recommended baseline; the example's deterministic 24-triangle construction is the kind of thing the spec can lock in.

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
2. **mesh-sdf + MC** if mesh-sdf exposes a public MC entry point on a closed-form `max(box_sdf, -sphere_sdf)`. Currently unverified.

Hand-author (option 1) recommended; cap sphere tessellation at ≤32 segments to keep §6.3's ray-tri scanline budget under 2 s on the reference machine (200³ voxel grid × ~1000 triangles → ~ 4×10⁷ ray-tri tests; well within target).

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

§1 gap inventory updated with Gap M (incl. M.2 sub-fix in description); §2 added F15 finding; §5.9 holds the per-fix spec including build-plate filter logic; §5.10 holds CHANGELOG creation; commit-order is A → M → B → D → E → F → L → K → DetectorSkipped → CHANGELOG → C → G → H → I → J → §7 examples in detector order.

---

## §8 onwards — pending

(Sections 8–13: risk inventory, stress-test gauntlet, grading & CI impact, open questions, implementation order, spec lifecycle.)
