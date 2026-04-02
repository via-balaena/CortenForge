# Actuator Position & Velocity Sensors

Gear ratio demonstration with ActuatorPos and ActuatorVel sensors.

## What it demonstrates

- `<actuatorpos>` reports the actuator's generalized position
- `<actuatorvel>` reports the actuator's generalized velocity
- For joint transmission with gear=2: ActuatorPos = 2 * JointPos exactly
- The gear ratio is an algebraic identity, not a physical approximation

## Expected visual behavior

A pendulum arm with a purple tip swings back and forth, driven by a
sinusoidal position servo. The motion is smooth and periodic (~3-second
period). The HUD shows actuator and joint readings side by side, with
the ratio locked at exactly 2.000000.

## Expected console output

```
t=  1.0s  act=0.5124  jnt=0.2562  ratio=2.000000
t=  2.0s  act=0.8736  jnt=0.4368  ratio=2.000000
t=  3.0s  act=0.0124  jnt=0.0062  ratio=2.000000
```

The ratio never deviates from 2.0 — it's an exact algebraic relationship.

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| ActuatorPos = gear * JointPos | max error over 15s | < 1e-10 |
| ActuatorVel = gear * JointVel | max error over 15s | < 1e-10 |
| Gear ratio constant | both errors < 1e-10 | exact |

## Run

```bash
cargo run -p example-sensor-adv-actuator-pos-vel --release
```
