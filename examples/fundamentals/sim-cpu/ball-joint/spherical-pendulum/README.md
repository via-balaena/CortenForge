# Spherical Pendulum — Unlimited Ball Joint

A heavy sphere on a rigid rod, hanging from a support frame via a ball joint.
Displaced 30° from vertical in both pitch and roll, then released — it traces
a precessing elliptical path, the classic **3D spherical pendulum**.

See also: [Conical Pendulum](../conical-pendulum/) — circular orbit with trail,
[Cone Limit](../cone-limit/) — constraint enforcement,
[Cone-Limit Orbit](../cone-limit-orbit/) — orbit along constraint surface.

## What you see

- **Blue sphere** — the 1 kg tip mass on a 0.5 m rod
- **Dark socket** — the ball-and-socket joint at the pivot point
- **Steel frame** — horizontal beam with vertical posts (visual only)

## Physics

The ball joint allows **free rotation about all three axes** (3 DOF).
It is specified in MJCF as `type="ball"`.

Unlike a hinge (1 DOF, scalar angle), the ball joint uses a **quaternion**
for orientation (4 qpos values: w, x, y, z) and angular velocity for
generalized velocity (3 qvel values). Integration uses the **exponential
map** on SO(3) to update the quaternion while preserving its unit norm.

The diagonal initial tilt gives the pendulum both pitch and roll, creating
a Lissajous-like precessing trajectory rather than a planar swing.

| Parameter | Value |
|-----------|-------|
| Mass | 1.0 kg |
| Rod length | 0.5 m |
| Body inertia | 0.01 (isotropic) |
| Damping | 0 (undamped) |
| Integrator | RK4 |
| Initial tilt | 30° about (1, 1, 0) axis |

## Validation

The example runs four automated checks at t=15s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Quat norm** | Unit quaternion preserved | < 1e-10 deviation |
| **Energy** | Conserved (undamped, no contacts) | < 0.5% drift |
| **BallQuat sensor** | Matches qpos exactly | < 1e-10 error |
| **BallAngVel sensor** | Matches qvel exactly | < 1e-10 error |

## Run

```
cargo run -p example-ball-joint-pendulum --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
