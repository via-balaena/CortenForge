# sim-physics Consolidation Plan

## Current State

### L0 Crates (13 total)

| Crate | In sim-physics? | Status | Description |
|-------|-----------------|--------|-------------|
| sim-types | ✅ | Complete | Core data types (bodies, joints, poses, actions) |
| sim-core | ✅ | Complete | Simulation engine (world, stepper, integrators) |
| sim-constraint | ✅ | Complete | Joint constraints (revolute, prismatic, fixed, etc.) |
| sim-contact | ✅ | Complete | Contact dynamics (collision response, friction) |
| sim-urdf | ✅ | Complete | URDF robot description parser |
| sim-muscle | ❌ | Complete | Hill-type muscle actuators (standalone math, integrates via sim-constraint) |
| sim-tendon | ❌ | Complete | Tendon/cable simulation (standalone math, integrates via sim-constraint) |
| sim-sensor | ❌ | Complete | Sensor simulation — all sensors work, RayCaster trait implemented |
| sim-deformable | ❌ | Unknown | Deformable bodies (soft bodies, cloth, ropes) |
| sim-mjcf | ❌ | **Near Complete** | MJCF loader — tendons ✅, equality constraints ✅, muscles ✅, missing proper SDF |
| sim-simd | ❌ | Complete | SIMD-optimized math (internal use only, don't re-export) |
| sim-tests | N/A | — | Integration test harness (not a library) |

### L1 Crates (1 total)

| Crate | Description |
|-------|-------------|
| sim-bevy | Bevy visualization plugin |

---

## Phase 0: Foundational Work

Before wiring crates into sim-physics, complete the incomplete implementations.

### 0.1 Complete sim-mjcf

**Current gaps:**

| Feature | Status | Work Required |
|---------|--------|---------------|
| Tendons | ✅ Complete | Fixed + spatial tendons with wrapping geometry support |
| Equality constraints | ✅ Complete | `connect`, `weld`, `joint`, `distance` constraint types |
| SDF collision | Sphere approximation | Implement proper mesh-to-SDF conversion |
| Muscle actuators | ✅ Complete | MJCF muscle actuators → HillMuscle with activation dynamics |
| Height fields | Not supported | Low priority — skip for now |

**Tasks:**

#### 0.1.1 Tendon Integration ✅
- [x] In `sim-mjcf/src/loader.rs`: Convert parsed `MjcfTendon` to `sim_constraint::TendonConstraint` (fixed tendons)
- [x] In `sim-mjcf/src/loader.rs`: Convert spatial tendons to `sim_tendon::SpatialTendon`
- [x] Add wrapping geometry support for spatial tendons (sphere/cylinder)
- [x] Add tendons to `LoadedModel` spawn process
- [x] Add tendon lookup to `SpawnedModel` (name → tendon)
- [x] Test: MJCF with fixed tendon → verify TendonConstraint created
- [x] Test: MJCF with spatial tendon → verify SpatialTendon with path and wrapping

#### 0.1.2 Equality Constraints ✅
- [x] In `sim-constraint`: Add `WeldConstraint` (6 DOF lock between bodies)
- [x] In `sim-constraint`: Add `JointPositionConstraint` (lock joint to specific position)
- [x] In `sim-constraint`: Add `DistanceConstraint` (fixed distance between points)
- [x] In `sim-mjcf/src/types.rs`: Add `MjcfWeld`, `MjcfJointEquality`, `MjcfDistance` types
- [x] In `sim-mjcf/src/parser.rs`: Parse `<weld>`, `<joint>`, `<distance>` elements
- [x] In `sim-mjcf/src/loader.rs`: Convert parsed equality elements to constraints
- [x] Test: MJCF with weld constraint → verify bodies welded
- [x] Test: MJCF with joint equality → verify joint position locked
- [x] Test: MJCF with distance constraint → verify distance maintained

#### 0.1.3 Muscle Actuator Integration ✅
- [x] In `sim-mjcf/src/loader.rs`: Convert `MjcfActuator::Muscle` to `sim_muscle::HillMuscle`
- [x] Add `LoadedMuscle` struct with muscle metadata (name, joint, ranges, gear)
- [x] Add muscle lookup to `LoadedModel` (by name, by joint)
- [x] Test: MJCF with muscle actuator → verify HillMuscle created with correct parameters

#### 0.1.4 SDF Collision (Optional — can defer)
- [ ] Implement mesh-to-SDF conversion using distance field computation
- [ ] Replace sphere approximation in `loader.rs`
- [ ] Test: MJCF with SDF geom → verify collision works

**Acceptance criteria:** MJCF files with tendons, muscles, and equality constraints load and simulate correctly.

**Progress:**
- ✅ 0.1.1 Tendon Integration — Fixed tendons convert to `TendonConstraint`, spatial tendons convert to `SpatialTendon` with wrapping geometry support (sphere/cylinder). 162 tests passing.
- ✅ 0.1.2 Equality Constraints — Added `WeldConstraint`, `JointPositionConstraint`, `DistanceConstraint` to sim-constraint. MJCF types, parser, and loader support for `<weld>`, `<joint>`, `<distance>`. 18 new tests. 180 tests passing.
- ✅ 0.1.3 Muscle Actuator Integration — Added `LoadedMuscle` struct, muscle lookup methods. MJCF muscle actuators convert to `HillMuscle` with activation dynamics. 7 new tests. 187 tests passing.
- ✅ 0.2.1 Rangefinder Ray-Casting — Added `raycast.rs` module to sim-core. Implemented ray-shape intersection for all primitive types (sphere, box, capsule, cylinder, ellipsoid, plane, heightfield, SDF, triangle mesh). Added `sensor` feature with `RayCaster` trait implementation for `World`. 18 new raycast tests + 6 World::cast_ray tests (272 total sim-core tests).

---

### 0.2 Complete sim-sensor ✅

**Current gaps:**

| Feature | Status | Work Required |
|---------|--------|---------------|
| Rangefinder | ✅ Complete | RayCaster trait + World::cast_ray implementation |
| Other sensors | ✅ Complete | IMU, F/T, touch, magnetometer all work |

**Tasks:**

#### 0.2.1 Rangefinder Ray-Casting ✅
- [x] Define `RayCaster` trait in sim-sensor (already exists)
- [x] Create `raycast.rs` module in sim-core with ray-shape intersection functions
- [x] Add `sensor` feature to sim-core with optional sim-sensor dependency
- [x] Implement `RayCaster` trait for `World` (behind `sensor` feature)
- [x] Add `World::cast_ray()` method for direct ray casting
- [x] Test: Ray casting against spheres, boxes, capsules, cylinders, ellipsoids

#### 0.2.2 Sensor Integration Helper (Optional)
- [ ] Consider adding `SensorBank` struct to manage multiple sensors
- [ ] Add convenience method: `sensor_bank.read_all(&world_state) -> Vec<SensorObservation>`
- [ ] This makes RL observation collection easier

**Acceptance criteria:** All sensor types produce valid readings from simulation state. ✅

---

## Phase 1: Wire L0 Crates into sim-physics

After Phase 0, wire the completed crates into the umbrella.

### 1.1 Add sim-mjcf

Dependencies: sim-types, sim-core, sim-constraint, mesh-io, mesh-types

Changes:
- [ ] Add `sim-mjcf = { workspace = true }` to sim-physics/Cargo.toml
- [ ] Add `"sim-mjcf/serde"` to serde feature
- [ ] Re-export `pub use sim_mjcf;` in lib.rs
- [ ] Add MJCF types to prelude (LoadedModel, load_mjcf_file, load_mjcf_str, etc.)
- [ ] Add integration test for MJCF → World pipeline

### 1.2 Add sim-muscle

Dependencies: sim-types

Changes:
- [ ] Add `sim-muscle = { workspace = true }` to sim-physics/Cargo.toml
- [ ] Add `"sim-muscle/serde"` to serde feature
- [ ] Re-export `pub use sim_muscle;` in lib.rs
- [ ] Add muscle types to prelude (HillMuscle, HillMuscleConfig, ActivationDynamics, etc.)
- [ ] Update architecture diagram in lib.rs docs

### 1.3 Add sim-tendon

Dependencies: sim-types

Changes:
- [ ] Add `sim-tendon = { workspace = true }` to sim-physics/Cargo.toml
- [ ] Add `"sim-tendon/serde"` to serde feature
- [ ] Re-export `pub use sim_tendon;` in lib.rs
- [ ] Add tendon types to prelude (FixedTendon, SpatialTendon, CableProperties, etc.)

### 1.4 Add sim-sensor

Dependencies: sim-types

Changes:
- [ ] Add `sim-sensor = { workspace = true }` to sim-physics/Cargo.toml
- [ ] Add `"sim-sensor/serde"` to serde feature
- [ ] Re-export `pub use sim_sensor;` in lib.rs
- [ ] Add sensor types to prelude (Imu, ForceTorqueSensor, TouchSensor, Rangefinder, etc.)

### 1.5 Add sim-deformable (Optional, behind feature flag)

Dependencies: sim-types, hashbrown, smallvec, bitflags

Changes:
- [ ] Add `sim-deformable = { workspace = true, optional = true }` to sim-physics/Cargo.toml
- [ ] Add `deformable` feature that enables sim-deformable
- [ ] Conditionally re-export: `#[cfg(feature = "deformable")] pub use sim_deformable;`
- [ ] Add deformable types to prelude behind feature gate

### 1.6 sim-simd Decision

**Decision: Don't re-export.**

sim-simd is internal optimization. Users shouldn't need to import it directly.

---

## Phase 2: Integration Testing

### 2.1 Full Pipeline Test: URDF → Physics

Create `sim/L0/tests/integration/urdf_pipeline.rs`:

Test cases:
- [ ] Simple single-link URDF
- [ ] Multi-link articulated robot
- [ ] Robot with revolute + prismatic joints
- [ ] Robot with joint limits and motors

### 2.2 Full Pipeline Test: MJCF → Physics

Create `sim/L0/tests/integration/mjcf_pipeline.rs`:

Test cases:
- [ ] Simple MJCF model (bodies + joints)
- [ ] Model with muscles → verify activation produces motion
- [ ] Model with tendons → verify coupling works
- [ ] Model with sensors → verify observations
- [ ] Model with equality constraints → verify constraints enforced

### 2.3 Muscle-Tendon Integration Test

Create `sim/L0/tests/integration/musculoskeletal.rs`:

Test cases:
- [ ] Single muscle pulling on revolute joint
- [ ] Antagonist muscle pair (co-contraction)
- [ ] Tendon routing through multiple joints
- [ ] Muscle + tendon combination (muscle-tendon unit)

### 2.4 Sensor Integration Test

Create `sim/L0/tests/integration/sensors.rs`:

Test cases:
- [ ] IMU on falling body → verify acceleration includes gravity
- [ ] Force/torque sensor at joint → verify reaction forces
- [ ] Touch sensor on collision → verify contact detection
- [ ] Rangefinder → verify distance measurement

---

## Phase 3: L1 Expansion

### 3.1 sim-bevy Enhancements

Current state: Visualization works for rigid bodies, contacts, velocities, joints.

Missing:
- [ ] Muscle visualization (activation heat map, force vectors)
- [ ] Tendon visualization (cable paths)
- [ ] Sensor visualization (IMU axes, force arrows, touch highlights)
- [ ] Deformable body rendering (mesh deformation)

### 3.2 sim-egui (Future)

A debug UI panel for runtime inspection/control:
- World state inspector
- Body/joint property editors
- Simulation controls (pause, step, speed)
- Muscle activation sliders
- Sensor readout displays

Not started yet. Depends on Phase 1 + 2 completion.

---

## Phase 4: Documentation

### 4.1 sim-physics README Expansion

After all crates are wired:
- [ ] Update architecture diagram to show all crates
- [ ] Add usage examples for each major feature
- [ ] Add comparison table (features vs MuJoCo/PyBullet)

### 4.2 API Documentation

- [ ] Ensure all public types have doc comments
- [ ] Add module-level documentation with examples
- [ ] Cross-reference related types between crates

### 4.3 User Guide (Optional)

Longer-form documentation:
- Getting started tutorial
- URDF/MJCF loading guide
- RL environment setup guide
- Visualization customization guide

---

## Execution Order

```
Phase 0.1 (sim-mjcf completion)
         │
         ▼
Phase 0.2 (sim-sensor completion)
         │
         ▼
Phase 1.1-1.5 (wire into sim-physics) ──→ Phase 2.1-2.4 (integration tests)
                                                  │
                                                  ▼
                                          Phase 3 (L1 expansion)
                                                  │
                                                  ▼
                                          Phase 4 (documentation)
```

Phase 0 is sequential (foundational work).
Phase 1 items can be done in parallel after Phase 0.
Phase 2 requires Phase 1 completion.
Phase 3 requires Phase 2 for confidence.
Phase 4 documents what's tested and working.

---

## Estimated Scope

| Phase | Effort | PRs | Status |
|-------|--------|-----|--------|
| 0.1.1 | Tendon integration | 1 PR | ✅ Complete |
| 0.1.2 | Equality constraints | 1 PR | ✅ Complete |
| 0.1.3 | Muscle actuators | 1 PR | ✅ Complete |
| 0.1.4 | SDF collision | 1 PR | Optional/Defer |
| 0.2 | sim-sensor completion | 1 PR (rangefinder ray-casting) | ✅ Complete |
| 1.1-1.5 | Wire into sim-physics | 5 PRs (one per crate) | Pending |
| 2.1-2.4 | Integration tests | 1-2 PRs | Pending |
| 3.x | L1 expansion | Varies | Pending |
| 4.x | Documentation | 1-2 PRs | Pending |

---

## Resolved Questions

1. **sim-mjcf completeness**: Near complete. ✅ Tendons (fixed + spatial with wrapping). ✅ Equality constraints (`weld`, `joint`, `distance`). ✅ Muscle actuators (HillMuscle integration). Only SDF collision deferred. → Phase 0.1 Done

2. **Muscle/tendon integration with World**: Complete. Both are standalone math libraries by design. Integration layer exists in sim-constraint. → Ready for Phase 1.

3. **Sensor integration with World**: Complete. `RayCaster` trait implemented for `World`. Decoupled design preserved. → Phase 0.2 Done

4. **Feature flags**: Yes. sim-deformable should be optional (heavier deps). Other crates are lightweight. → Phase 1.5

---

## Changelog

| Date | Phase | Description |
|------|-------|-------------|
| 2026-01-25 | 0.1.1 | ✅ Tendon integration complete. Fixed tendons → `TendonConstraint`, spatial tendons → `SpatialTendon` with wrapping geometry (sphere/cylinder). Added `SiteInfo`, `GeomInfo`, `LoadedTendon` enum. 162 tests passing. |
| 2026-01-25 | 0.1.2 | ✅ Equality constraints complete. Added `WeldConstraint`, `JointPositionConstraint`, `DistanceConstraint` to sim-constraint. MJCF types (`MjcfWeld`, `MjcfJointEquality`, `MjcfDistance`), parser, and loader support. 18 new tests (180 total). |
| 2026-01-25 | 0.1.3 | ✅ Muscle actuator integration complete. Added `LoadedMuscle` struct with HillMuscle + metadata. Muscle lookup by name and joint. 7 new tests (187 total). |
| 2026-01-25 | 0.2.1 | ✅ Rangefinder ray-casting complete. Added `raycast.rs` to sim-core with ray-shape intersection for all primitives. Added `sensor` feature with `RayCaster` trait impl for `World`. 18 new raycast + 6 World::cast_ray tests. |
