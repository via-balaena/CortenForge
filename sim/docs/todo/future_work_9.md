# Future Work 9 — Phase 3D: Performance + Crate Hygiene (Items #33–34)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

Performance optimization and codebase health. These items don't add physics features
but improve throughput and reduce maintenance burden.

---

### 33. SIMD Utilization Audit + Wiring
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
`sim-simd` crate (2,015 lines) provides batch SIMD operations: `Vec3x4`, `Vec3x8`,
`batch_aabb_overlap_4`, `batch_normal_force_4`, `batch_integrate_*`, `find_max_dot`.
Only `find_max_dot()` has production callers (in GJK/EPA support function search).
All other batch ops have zero callers outside benchmarks.

#### Objective
Audit hot paths in the pipeline for SIMD opportunities and wire in existing batch
operations where they provide measurable speedup.

#### Specification

1. **Audit targets** (profile first, optimize second):
   - Broad-phase AABB overlap testing — candidate for `batch_aabb_overlap_4`
   - Contact force accumulation — candidate for `batch_normal_force_4`
   - Position/velocity integration — candidate for `batch_integrate_*`
   - Jacobian-transpose force mapping — candidate for `Vec3x4` batch dot products
2. **Wiring**: For each candidate, benchmark SIMD vs scalar on realistic workloads
   (≥100 contacts, ≥30 bodies). Only wire SIMD if speedup > 1.3×.
3. **Cleanup**: Remove unused batch ops from `sim-simd` if no production use case
   materializes after audit.

#### Acceptance Criteria
1. Profile data for top-5 hot paths in pipeline step.
2. SIMD wired for ≥2 hot paths with measured speedup > 1.3×.
3. Unused batch ops removed or documented as "benchmark-only".
4. No correctness regressions (bit-exact where possible, tolerance where not).

#### Files
- `sim/L0/simd/src/` — existing batch ops
- `sim/L0/core/src/mujoco_pipeline.rs` — hot path integration points

---

### 34. Standalone Crate Consolidation
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
Several standalone crates are fully superseded by pipeline-native implementations:
- **sim-tendon** (3,919 lines): All functionality reimplemented in
  `mj_fwd_tendon_fixed()` / `mj_fwd_tendon_spatial()`
- **sim-muscle** (2,550 lines): MuJoCo FLV model reimplemented in
  `mj_fwd_actuation()`
- **sim-constraint** partial: PGS, Newton, islands all deleted or reimplemented.
  Remaining: `PlanarJoint`, `CylindricalJoint`, `CGSolver`, `PneumaticCylinder`,
  equality constraint types

These crates add maintenance burden, confuse new contributors ("which PGS?"), and
inflate compile times.

#### Objective
Deprecate or consolidate superseded standalone crates.

#### Specification

1. **sim-tendon**: Mark as `#[deprecated]`. Add top-level doc comment directing
   users to pipeline `mj_fwd_tendon_*()`. Consider removing from default workspace
   members.
2. **sim-muscle**: Mark as `#[deprecated]`. Note the standalone Hill model is richer
   than pipeline FLV but not MuJoCo-compatible. Keep available for biomechanics
   users who don't need MuJoCo conformance.
3. **sim-constraint**: Keep crate but audit public API. Remove re-exports of deleted
   types (PGSSolver, NewtonSolver, etc.). Document which types are standalone-only
   vs pipeline-compatible.
4. **sim-sensor**: Keep — provides standalone hardware sensor API independent of
   pipeline.

#### Acceptance Criteria
1. Deprecated crates emit compiler warnings on use.
2. `cargo doc` shows clear deprecation notices with migration guidance.
3. No change to pipeline behavior (regression).
4. Workspace compiles cleanly (no dead-code warnings in deprecated crates).

#### Files
- `sim/L0/tendon/src/lib.rs` — deprecation attributes
- `sim/L0/muscle/src/lib.rs` — deprecation attributes
- `sim/L0/constraint/src/lib.rs` — API audit
- `Cargo.toml` (workspace) — optional default-members adjustment
