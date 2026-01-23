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
  - **Status:** Substantially implemented
  - Working: TriangleMesh ↔ Sphere, Capsule, Box + BVH acceleration + MJCF parsing
  - Missing: TriangleMesh ↔ TriangleMesh (mesh-mesh collision)
  - **Action:** Created implementation plan → [MESH_MESH_COLLISION_PLAN.md](./MESH_MESH_COLLISION_PLAN.md)

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

- [ ] **Add performance benchmarks using criterion**
  - Hot paths to benchmark:
    - Constraint solving
    - BVH queries
    - Triangle-primitive collision
    - (Future) Mesh-mesh collision

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
| - | Verify SDF collision | ⏳ Pending |
| - | MJCF `<default>` element | ⏳ Pending |
| - | Sync gap analysis | ⏳ Pending |
| - | Add criterion benchmarks | ⏳ Pending |
