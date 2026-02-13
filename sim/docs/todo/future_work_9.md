# Future Work 9 — Phase 3D: Performance (Item #33)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

Performance optimization. ~~#34 (Standalone crate consolidation)~~ has been moved
to [future_work_6_precursor_1.md](./future_work_6_precursor_1.md) #19e as a
prerequisite to the MuJoCo conformance test suite.

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

### ~~34. Standalone Crate Consolidation~~
**Status:** Moved to [future_work_6_precursor_1.md](./future_work_6_precursor_1.md) #19e
