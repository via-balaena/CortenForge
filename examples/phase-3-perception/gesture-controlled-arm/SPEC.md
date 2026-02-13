# Gesture-Controlled Arm — Phase 3 Product Spec

IMU-instrumented glove that teloperates a 3-DOF robot arm. Sensor-fusion maps
wrist orientation to joint commands, route-pathfind plans collision-free joint
trajectories. Arm runs in sim first, then on hardware with identical control code.

## Pipeline

```
IMU glove ─► Orientation estimation (sensor-fusion) ─► Joint mapping
    ─► Trajectory planning (route-pathfind) ─► Servo commands
```

1. **Sense** — Read accelerometer + gyroscope from IMU on glove (real or simulated via sim-sensor).
2. **Fuse** — Sensor-fusion estimates wrist orientation from noisy IMU data.
3. **Map** — Convert wrist orientation (roll/pitch/yaw) to target joint angles for 3-DOF arm.
4. **Plan** — route-pathfind generates a collision-free trajectory in joint space from current to target.
5. **Act** — Send joint angle commands to three servos (shoulder, elbow, wrist).

## CortenForge Crates Used

| Crate | Purpose in this product |
|---|---|
| `sensor-types` | IMU data types (accelerometer, gyroscope readings) |
| `sensor-fusion` | Orientation estimation from glove IMU |
| `sim-sensor` | Simulated IMU with configurable noise and latency |
| `sim-core` | Physics simulation stepping for robot arm |
| `sim-constraint` | Joint constraints for 3-DOF arm (revolute joints, limits) |
| `sim-types` | Simulation data structures |
| `route-types` | Joint-space trajectory representations |
| `route-pathfind` | Collision-free trajectory planning in joint space |
| `mesh-types` | Arm link geometry representation |
| `mesh-io` | Load arm and environment meshes |

## Modes

- **Sim mode:** sim-sensor provides simulated glove IMU, sim-core steps arm physics.
- **Hardware mode:** Real IMU glove + real servos, same control code.

## Input

- Wrist orientation from IMU glove (continuous stream).

## Output

- Robot arm tracks glove orientation in real-time.

## Acceptance Criteria

1. Arm tracks glove orientation within 5 degrees across all three axes.
2. End-to-end latency from glove motion to arm motion is less than 50ms.
3. Joint trajectories avoid self-collision and workspace limits.
4. Same control code binary runs in both sim and hardware mode.
5. Sensor-fusion converges from arbitrary initial state within 500ms.

## Status

**Spec** — not yet implemented.
