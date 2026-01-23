# Feature Implementation Checklist

Tracking progress on remaining MuJoCo feature gaps and infrastructure improvements.

---

## Build & Infrastructure

- [x] **Fix the build** - Add `parallel: ParallelConfig::default()` to `sim-mjcf/src/config.rs:34`
  - Also exported `ParallelConfig` from `sim-types/src/lib.rs`
  - Completed: 2026-01-22

---

## Feature Verification & Implementation

### Collision Systems

- [x] **Verify Non-convex mesh collision status** (TriangleMesh variant)
  - **Status:** ✅ COMPLETE
  - Working: TriangleMesh ↔ Sphere, Capsule, Box + BVH acceleration + MJCF parsing
  - Working: TriangleMesh ↔ TriangleMesh (mesh-mesh collision) - **Implemented!**
  - See: [MESH_MESH_COLLISION_PLAN.md](./MESH_MESH_COLLISION_PLAN.md) for full details
  - Performance: 172 µs for 36k triangle pairs (well under 5ms target)

- [ ] **Verify SDF collision status** (Sdf variant)
  - Status: Pending verification

### MJCF Parser

- [ ] **Implement MJCF `<default>` element support**
  - [ ] Actuator defaults
  - [ ] Tendon defaults
  - [ ] Sensor defaults

---

## Documentation & Testing

- [ ] **Sync gap analysis document** - Update `MUJOCO_GAP_ANALYSIS.md` to reflect:
  - TriangleMesh implementation status (substantially complete)
  - SDF implementation status (pending verification)
  - Mesh-mesh collision plan

- [x] **Add performance benchmarks using criterion**
  - Hot paths benchmarked:
    - [x] Mesh-mesh collision (`sim-core/benches/collision_benchmarks.rs`)
    - [ ] Constraint solving
    - [ ] BVH queries
    - [ ] Triangle-primitive collision

---

## Related Documents

- [MESH_MESH_COLLISION_PLAN.md](./MESH_MESH_COLLISION_PLAN.md) - Detailed plan for implementing mesh-mesh collision
- [MUJOCO_GAP_ANALYSIS.md](./MUJOCO_GAP_ANALYSIS.md) - Overall MuJoCo feature parity tracking

---

## Progress Log

| Date | Item | Status |
|------|------|--------|
| 2026-01-22 | Fix build (ParallelConfig) | ✅ Complete |
| 2026-01-22 | Verify TriangleMesh collision | ✅ Verified (mostly complete) |
| 2026-01-22 | Create mesh-mesh collision plan | ✅ Complete |
| 2026-01-23 | Implement mesh-mesh collision (all 4 milestones) | ✅ Complete |
| 2026-01-23 | Add criterion benchmarks (mesh-mesh) | ✅ Complete |
| - | Verify SDF collision | ⏳ Pending |
| - | MJCF `<default>` element | ⏳ Pending |
| - | Sync gap analysis | ⏳ Pending |
