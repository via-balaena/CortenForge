# Projectile — Parabolic Trajectory

A sphere launched at 45 degrees under gravity. Pure translational motion.

## What you're seeing

An orange-red sphere arcs through empty space. It launches from the origin
at 5 m/s at a 45-degree angle, rises to an apex, then falls back down.
There is no spin — only translation. Every 2 seconds the sphere resets and
launches again.

## What to look for

- **Parabolic arc.** The trajectory is a textbook parabola. The horizontal
  velocity stays constant while the vertical velocity changes linearly due to
  gravity.
- **The HUD tracks error.** `x_err` and `z_err` show the deviation between
  the simulated position and the exact analytical solution. Both should stay
  near zero (< 0.1% of the analytical value).
- **Energy is conserved.** `E_total` (kinetic + potential) stays constant
  throughout the flight. At the apex, all kinetic energy from the vertical
  component has converted to potential energy.
- **The ball relaunches.** Every 2 seconds the sphere resets to the launch
  position and fires again, so you can watch the arc repeatedly.

## The physics

A free joint gives a body 6 degrees of freedom. This example isolates the
translational subspace — the 3 position coordinates in `qpos[0..3]` and 3
linear velocity components in `qvel[0..3]`. The rotational DOFs (quaternion
in `qpos[3..7]`, angular velocity in `qvel[3..6]`) are unused.

Gravity applies a constant downward acceleration of 9.81 m/s^2 to the linear
DOFs only. The exact solution is:

```
x(t) = v0 * cos(theta) * t
z(t) = v0 * sin(theta) * t  -  0.5 * g * t^2
```

With v0=5.0 and theta=45 degrees:
- Apex height: 0.637 m (at t = 0.360 s)
- Range: 2.548 m (at t = 0.721 s)

## Run

```
cargo run -p example-free-joint-projectile --release
```
