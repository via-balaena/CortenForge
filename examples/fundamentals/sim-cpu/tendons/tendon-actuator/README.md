# Tendon Actuator — Tendon vs Joint Transmission

Two identical arms side by side receiving the same control signal. The left
arm is driven through a **tendon transmission** (spatial tendon), the right
through a direct **joint transmission**. Same input, different output —
the tendon-driven arm responds differently because its effective gear ratio
is configuration-dependent.

## Concept

A joint-transmission actuator has a **constant gear ratio**: 1 N·m of control
always produces 1 N·m at the joint. A tendon-transmission actuator maps force
through the tendon's Jacobian, which changes with configuration. The effective
torque at the joint varies as the arm moves — the moment arm is not constant.

## What you see

- **Left arm (blue):** tendon-driven. A spatial tendon from shoulder to
  forearm tip transmits the actuator force.
- **Right arm (red):** joint-driven. Direct motor on the shoulder joint.
- Both receive the same sinusoidal control signal.
- The left arm's response differs from the right — same input, different
  motion — proving the configuration-dependent gear ratio.
- The tendon path is drawn on the left arm.
- The HUD shows both shoulder angles and the actuator forces.

## Run

```
cargo run -p example-tendon-actuator --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
