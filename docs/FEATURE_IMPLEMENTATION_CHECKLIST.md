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

- [x] **Verify SDF collision status** (Sdf variant)
  - **Status:** ✅ VERIFIED - Partial implementation (3/10 combinations)
  - **Working combinations:**
    - Sdf ↔ Sphere (trilinear interpolation + gradient normals)
    - Sdf ↔ Capsule (5-point axis sampling)
    - Sdf ↔ Box (8-corner sampling)
  - **Location:** `sim-core/src/sdf.rs`, `sim-core/src/world.rs:1750-1835`

- [ ] **Complete SDF collision support** (7 missing combinations)
  - See: [SDF_COLLISION_PLAN.md](./SDF_COLLISION_PLAN.md) for detailed implementation plan
  - [ ] Milestone 1: Sdf ↔ Cylinder, Ellipsoid (point sampling)
  - [ ] Milestone 2: Sdf ↔ ConvexMesh (vertex sampling)
  - [ ] Milestone 3: Sdf ↔ Plane (grid sampling)
  - [ ] Milestone 4: Sdf ↔ TriangleMesh (vertex + BVH)
  - [ ] Milestone 5: Sdf ↔ HeightField (grid overlap)
  - [ ] Milestone 6: Sdf ↔ Sdf (dual implicit surface)

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
- [SDF_COLLISION_PLAN.md](./SDF_COLLISION_PLAN.md) - Detailed plan for completing SDF collision support
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
| 2026-01-23 | Verify SDF collision | ✅ Verified (partial - Sphere/Capsule/Box working) |
| 2026-01-23 | Create SDF collision plan | ✅ Complete |
| - | Complete SDF collision (7 combinations) | ⏳ Pending |
| - | MJCF `<default>` element | ⏳ Pending |
| - | Sync gap analysis | ⏳ Pending |
