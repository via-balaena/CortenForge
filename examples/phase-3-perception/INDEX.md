# Phase 3: Perception + Control Loop

> **Sense -> Decide -> Act (same code in sim and on hardware)**

Phase 3 closes the loop. Sensors feed data into control logic that drives
actuators. The defining constraint: the same control code must run
identically in simulation and on real hardware. If the abstraction leaks,
we fix the abstraction.

---

## New Domains Introduced

| Domain | Purpose |
|--------|---------|
| sensor-types | Sensor data types (IMU, range, force, encoder) |
| sensor-fusion | Kalman filtering, complementary filters, state estimation |
| route-types | Spatial path and trajectory representations |
| route-pathfind | Obstacle-aware path planning (A*, RRT, potential fields) |
| sim-sensor | Simulated sensor models with configurable noise and latency |
| cf-spatial | Spatial data structures (occupancy grids, distance fields, KD-trees) |

**Prerequisites:** Phase 2 complete. Rigid body simulation and mesh
pipeline crates are assumed stable and available.

---

## Products

| Product | Description | Hardware | Spec |
|---------|-------------|----------|------|
| Leveling Platform | IMU-fused self-leveling platform with two servos | IMU + 2 servos | [SPEC.md](./leveling-platform/SPEC.md) |
| Obstacle-Avoiding Robot | Wheeled robot with range sensors and path replanning | Range sensors + motors | [SPEC.md](./obstacle-avoiding-robot/SPEC.md) |
| Gesture-Controlled Arm | IMU glove teleoperating a 3-DOF robot arm | IMU glove + 3-DOF arm | [SPEC.md](./gesture-controlled-arm/SPEC.md) |

**Goal:** Close the sense-decide-act loop. Same control code runs in
simulation and on hardware. Simulated sensors must produce statistically
equivalent noise profiles to real hardware so that controllers tuned in
sim transfer without re-tuning.

---

## Crate Coverage

| Crate | Platform | Robot | Arm |
|-------|----------|-------|-----|
| sensor-types | x | x | x |
| sensor-fusion | x | x | x |
| sim-sensor | x | x | x |
| route-types | | x | x |
| route-pathfind | | x | x |
| cf-spatial | | x | |
| sim-core | x | x | x |
| sim-constraint | x | | x |

---

## Running

```bash
# Leveling platform: simulate IMU + servo control loop
cargo run -p example-leveling-platform

# Obstacle-avoiding robot: simulate range sensors + path planning
cargo run -p example-obstacle-avoiding-robot

# Gesture-controlled arm: simulate IMU glove -> arm teleoperation
cargo run -p example-gesture-controlled-arm
```

---

## Shared Learnings

Things we expect to discover and codify during Phase 3:

- Sensor noise models that produce sim-to-real transfer without controller re-tuning
- Fusion filter parameters (Kalman Q/R matrices) that generalize across IMU hardware
- Path planning performance under real-time constraints (replanning within one control cycle)
- Hardware abstraction boundaries -- where the sim/real split belongs in the API
- Latency modeling: how much simulated delay is needed to match real sensor-to-actuator lag
- Occupancy grid resolution tradeoffs for indoor navigation at different scales

---

## Completion Criteria

Phase 3 is complete when all of the following hold:

- [ ] All three products run their control loop identically in sim and on hardware (same binary, different sensor backend)
- [ ] The leveling platform maintains level within 1 degree under step disturbances in both sim and hardware
- [ ] The obstacle-avoiding robot navigates a cluttered environment without collision, replanning in real time
- [ ] The gesture-controlled arm tracks glove orientation with less than 100ms end-to-end latency
- [ ] Simulated sensor noise profiles are statistically validated against real hardware measurements
- [ ] Sensor fusion filters converge from arbitrary initial state within 500ms
- [ ] At least one product has been demonstrated on real hardware with the same control code used in simulation
- [ ] All crates from Phase 1 and Phase 2 continue to pass on Phase 3 geometries and models

---

*See also: [Product Roadmap](../PRODUCT_ROADMAP.md) | [Phase 2: Mechanism Design](../phase-2-mechanism/INDEX.md) | [Phase 4: Deformable](../phase-4-deformable/INDEX.md)*
