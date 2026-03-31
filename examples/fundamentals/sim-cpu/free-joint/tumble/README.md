# Tumble — Torque-Free Precession

A box with asymmetric inertia floating in zero gravity, spinning freely.

## What you're seeing

A steel-blue rectangular box tumbles in place at the center of the scene. A
small red sphere on one corner acts as a visual marker so you can track the
rotation. There is no gravity and no friction — nothing slows the box down.

The box spins primarily about its longest axis (X), but because its three
moments of inertia are all different (Ixx=0.10, Iyy=0.06, Izz=0.03), the
spin axis itself slowly wobbles. This wobble is called **torque-free
precession** — the angular velocity vector traces a cone around the angular
momentum vector.

## What to look for

- **The box never stops.** There are no forces acting on it. It will tumble
  forever at exactly the same energy.
- **The spin axis wobbles.** Watch the red marker — it doesn't trace a simple
  circle. The box's rotation axis drifts in a periodic pattern (~5 second
  cycle). This is Euler's equations at work.
- **The HUD confirms conservation.** The `|L|` (angular momentum magnitude)
  and `KE` (kinetic energy) values stay constant to machine precision. The
  `omega_x/y/z` components oscillate, but `|L|` does not.
- **`|quat|` stays exactly 1.0.** The quaternion representing the box's
  orientation is renormalized every step to prevent numerical drift.

## The physics

A free joint gives a body 6 degrees of freedom: 3 translational + 3
rotational. This example isolates the rotational subspace — the body has
angular velocity but no linear velocity, so it spins in place.

The orientation is stored as a unit quaternion (4 numbers: w, x, y, z) in
`qpos[3..7]`. Each timestep, the integrator applies the exponential map:
`q_new = q_old * exp(omega * dt)`, which rotates the quaternion along the
angular velocity vector on the SO(3) manifold.

For a symmetric body (like a sphere), the spin axis would stay fixed. But
this box has three different moments of inertia, so Euler's equations of
rigid body motion produce precession — the angular velocity rotates in body
frame even though no external torque is applied.

## Run

```
cargo run -p example-free-joint-tumble --release
```
