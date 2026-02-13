# Self-Leveling Platform — Phase 3 Product Spec

IMU-fused self-leveling platform. Two servos, one IMU, complementary filter.
Control code runs identically in sim and on hardware. Proves the
sense->decide->act pipeline with sim<->real parity.

## Pipeline

```
IMU reading ─► Complementary filter (sensor-fusion) ─► PID control ─► Servo commands
```

1. **Sense** — Read accelerometer + gyroscope from IMU (real or simulated via sim-sensor).
2. **Fuse** — Complementary filter combines accel and gyro into a stable orientation estimate.
3. **Decide** — PID controller computes servo corrections to drive tilt error toward zero.
4. **Act** — Send angular position commands to two servos (pitch and roll axes).

## CortenForge Crates Used

| Crate | Purpose in this product |
|---|---|
| `sensor-types` | IMU data types (accelerometer, gyroscope readings) |
| `sensor-fusion` | Complementary filter for orientation estimation |
| `sim-sensor` | Simulated IMU with configurable noise and bias |
| `sim-core` | Physics simulation stepping |
| `sim-constraint` | Joint constraints for servo-driven platform |
| `sim-types` | Simulation data structures |
| `mesh-types` | Platform geometry representation |
| `mesh-io` | Load/export platform mesh |

## Modes

- **Sim mode:** sim-sensor provides simulated IMU, sim-core steps physics.
- **Hardware mode:** Real IMU over serial, same control code.

## Input

- Target level orientation (roll = 0, pitch = 0).

## Output

- Platform maintains level within 2 degrees under disturbance.

## Acceptance Criteria

1. Same control code binary runs in both sim and hardware mode.
2. Platform recovers to level within 500ms after a step disturbance.
3. Steady-state error is less than 2 degrees in both axes.
4. Complementary filter converges from arbitrary initial state within 500ms.
5. Control loop runs at a consistent 100 Hz in both modes.

## Status

**Spec** — not yet implemented.
