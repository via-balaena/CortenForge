# Continuous Joint Wheel

Spinning wheel with no joint limits. The URDF `continuous` joint type is
an unlimited revolute — it converts to an MJCF `hinge` with
`limited="false"`. A constant torque is applied and the angular
acceleration is verified against alpha = tau / I.

## What it tests

The key difference between `revolute` and `continuous` in URDF is that
continuous joints have no position limits. The angle can grow past 2*pi
without clamping, which is essential for wheels, motors, and other
continuously rotating mechanisms.

## Physics

The wheel has moment of inertia I_zz = 0.02 kg*m^2 about the spin axis.
Applying torque tau = 1.0 N*m:

```
alpha = tau / I = 1.0 / 0.02 = 50 rad/s^2
```

After 500 timesteps at dt = 0.002s, the expected angular velocity is
exactly 50 rad/s. The match is exact (0.000% error) because there is
no damping or friction.

## Checks

| # | Check | Tolerance |
|---|-------|-----------|
| 1 | Joint type is hinge | exact |
| 2 | Joint is unlimited (not limited) | exact |
| 3 | alpha = tau / I after one step | 1% |
| 4 | Angle passes 2*pi (no clamping) | qualitative |
| 5 | Constant torque → linear velocity ramp | 1% |

## Run

```
cargo run -p example-urdf-continuous --release
```
