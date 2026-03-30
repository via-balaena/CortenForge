# Forearm Flexion — Hill Muscle Lifting a Load

A forearm hanging from gravity, driven by a single Hill-type muscle actuator.
The muscle activates over ~50 ms and curls the arm upward against gravity,
demonstrating the complete HillMuscle pipeline: activation dynamics,
force-length curve, force-velocity curve, and rigid tendon geometry.

## What you see

- **Upper arm** (dark metal capsule) — fixed to the world, serves as the
  anchor point
- **Forearm** (lighter metal capsule with red tip) — hangs from a hinge joint
  at the elbow, free to swing in the X-Z plane
- At t=0.5s, the muscle activates (ctrl ramps 0 to 1)
- The forearm curls upward as muscle force overcomes gravity
- Joint damping slows the motion — the arm settles at a steady angle where
  muscle torque balances the gravity load

## Physics

The HillMuscle actuator computes force from three curves:

```
active_force  = -F0 x FL(norm_len) x FV(norm_vel) x cos(alpha) x activation
passive_force = -F0 x FP(norm_len) x cos(alpha)
total_force   = active_force + passive_force
```

Where:
- **FL** = Gaussian force-length curve (peak at optimal fiber length)
- **FV** = Hill hyperbolic force-velocity curve (reduces force during shortening)
- **FP** = Exponential passive force (resists over-stretch)
- **alpha** = pennation angle (0 in this example)

As the muscle shortens (arm curls up), `norm_len` decreases from 1.0
toward 0.5, reducing FL. Simultaneously, the shortening velocity makes
FV < 1.0. Both effects reduce force as the arm approaches full flexion.

| Parameter | Value |
|-----------|-------|
| F0 (peak isometric force) | 50 N |
| Optimal fiber length | 1.0 (actuator-length units) |
| Tendon slack length | 0.0 |
| Max contraction velocity | 10 L_opt/s |
| Pennation angle | 0 |
| Activation time constant | 10 ms |
| Deactivation time constant | 40 ms |
| Joint damping | 0.5 N-m-s/rad |
| Body mass | 1.0 kg |
| Forearm length | 0.3 m (CoM at 0.15 m) |
| Integrator | RK4, dt = 1 ms |

## Expected console output

```
t=  0.5s  ctrl ramp starts
t=  1.0s  act=0.97  force=-42.3  angle=0.92
t=  2.0s  act=1.00  force=-18.7  angle=0.54
t=  5.0s  act=1.00  force=-12.1  angle=0.31
```

Activation reaches ~1.0 within 50 ms of ctrl onset. Force is initially
large (near optimal length) then decreases as the muscle shortens. The
arm settles where gravity torque equals muscle torque.

## Validation

Three automated checks at t=10s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Activation reached 1.0** | act > 0.99 after ctrl=1 for 9.5s | > 0.99 |
| **Muscle produced force** | max \|force\| > 5 N during simulation | > 5 N |
| **Arm moved** | angle differs from initial by > 0.05 rad | > 0.05 rad |

## Run

```
cargo run -p example-muscle-forearm-flexion --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
