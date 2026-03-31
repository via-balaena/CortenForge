# Continuous Joint Wheel

A spinning disc with a marker sphere on its edge. The URDF `continuous`
joint converts to an MJCF `hinge` with `limited="false"`. A constant
torque accelerates the wheel — the angle passes well beyond 2*pi without
clamping.

## What you see

A flat disc spinning faster and faster. The marker sphere makes the
rotation visible. The HUD shows angle, velocity, and revolution count.

## What it tests

The key difference between `revolute` and `continuous` in URDF is that
continuous joints have no position limits. The angle grows without bound.

## Physics

```
alpha = tau / I = 0.1 / 0.02 = 5.0 rad/s^2
```

## Validation

| Check | Source |
|-------|--------|
| Continuous → unlimited hinge | `print_report` |
| Velocity matches alpha*t | `print_report` (5% tolerance) |
| Angle past 2*pi (no clamping) | `print_report` |

## Run

```
cargo run -p example-urdf-continuous --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
