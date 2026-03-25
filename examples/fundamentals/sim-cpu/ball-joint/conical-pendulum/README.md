# Conical Pendulum — Orbiting Ball Joint

A pendulum on an unlimited ball joint, given initial angular velocity so the
tip mass traces a **circular orbit** around the vertical axis. A glowing trail
makes the 3D trajectory visible — this motion is impossible with a hinge joint.

See also: [Spherical Pendulum](../spherical-pendulum/) — planar swing,
[Cone-Limit Orbit](../cone-limit-orbit/) — orbit along a constraint surface.

## What you see

- **Green tip sphere** — the 1 kg mass orbiting in a circle
- **Fading green trail** — 5 seconds of trajectory history, showing the
  circular orbit and any precession
- **Steel rod** — rigid link from pivot to mass
- **Dark socket** — the ball joint at the pivot
- **Support frame** — horizontal beam with vertical posts

## Physics

A conical pendulum is a mass on a rod that orbits in a horizontal circle.
The required angular velocity for a steady orbit at angle θ from vertical is:

```
ω = √(g / (L·cos(θ)))
```

At θ=45°, L=0.8m: ω = 4.164 rad/s, orbit period = 1.509 s.

This requires setting **initial angular velocity** on the ball joint — the
qvel is a 3-component angular velocity vector (ωx, ωy, ωz) in the body frame,
unlike a hinge which has a single scalar velocity. To orbit about the world
vertical, the angular velocity must be transformed from world frame to body
frame accounting for the body's tilt.

| Parameter | Value |
|-----------|-------|
| Mass | 1.0 kg |
| Rod length | 0.8 m |
| Damping | 0 (undamped) |
| Integrator | RK4 |
| Initial tilt | 45° |
| Initial ω | 4.164 rad/s about vertical |

## Validation

The example runs two automated checks at t=15s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Quat norm** | Unit quaternion preserved | < 1e-10 deviation |
| **Energy** | Conserved (undamped, no contacts) | < 0.5% drift |

## Run

```
cargo run -p example-ball-joint-conical --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
