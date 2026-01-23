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

- [x] **Complete SDF collision support** (all combinations implemented!)
  - See: [SDF_COLLISION_PLAN.md](./SDF_COLLISION_PLAN.md) for detailed implementation plan
  - [x] Milestone 1: Sdf ↔ Cylinder, Ellipsoid (point sampling) ✅
  - [x] Milestone 2: Sdf ↔ ConvexMesh (vertex sampling) ✅
  - [x] Milestone 3: Sdf ↔ Plane (grid sampling) ✅
  - [x] Milestone 4: Sdf ↔ TriangleMesh (vertex + edge sampling) ✅
  - [x] Milestone 5: Sdf ↔ HeightField (grid point sampling) ✅
  - [x] Milestone 6: Sdf ↔ Sdf (dual implicit surface sampling) ✅

### MJCF Parser

- [x] **Implement MJCF `<default>` element support**
  - [x] Actuator defaults - Applied in loader via `DefaultResolver::apply_to_actuator()`
  - [x] Tendon defaults - Added `MjcfTendon` struct, parser, and `apply_to_tendon()` method
  - [x] Sensor defaults - Added `MjcfSensor` struct, parser, and `apply_to_sensor()` method
  - Implementation details:
    - `sim-mjcf/src/types.rs`: Added `MjcfTendon`, `MjcfSensor`, `MjcfTendonType`, `MjcfSensorType`
    - `sim-mjcf/src/parser.rs`: Added `parse_tendons()`, `parse_sensors()` functions
    - `sim-mjcf/src/defaults.rs`: Added `apply_to_tendon()`, `apply_to_sensor()` methods
    - `sim-mjcf/src/loader.rs`: Updated `convert_actuators()` to apply defaults

---

## Documentation & Testing

- [x] **Sync gap analysis document** - Updated `MUJOCO_GAP_ANALYSIS.md` to reflect:
  - TriangleMesh implementation: ✅ Complete
  - SDF implementation: ✅ Complete (all 10 combinations)
  - MJCF defaults (actuator, tendon, sensor): ✅ Complete
  - Added LLM guidance note at top of file for section-by-section editing

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
| 2026-01-23 | SDF Milestone 1: Cylinder & Ellipsoid | ✅ Complete |
| 2026-01-23 | SDF Milestone 2: ConvexMesh | ✅ Complete |
| 2026-01-23 | SDF Milestone 3: Plane | ✅ Complete |
| 2026-01-23 | SDF Milestone 4: TriangleMesh | ✅ Complete |
| 2026-01-23 | SDF Milestone 5: HeightField | ✅ Complete |
| 2026-01-23 | SDF Milestone 6: Sdf ↔ Sdf | ✅ Complete |
| 2026-01-23 | MJCF `<default>` element (Actuator, Tendon, Sensor) | ✅ Complete |
| 2026-01-23 | Sync gap analysis document | ✅ Complete |
