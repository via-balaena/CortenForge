# Spinning Toss — Full 6-DOF Flight

A brick tossed upward at an angle while spinning. Both translation and
rotation active simultaneously.

## What you're seeing

A steel-blue rectangular brick with a red corner marker is tossed upward and
to the right. It follows a parabolic arc while tumbling — just like tossing a
book in the air. Every 3 seconds it resets and launches again.

## What to look for

- **Parabolic arc + tumble.** The center of mass follows the exact same
  parabola as a point-mass projectile. Meanwhile the brick tumbles freely
  around it. These two motions are completely independent.
- **The HUD tracks both.** `x_err` and `z_err` show deviation of the center
  of mass from the analytical point-mass trajectory. `|L|` shows the angular
  momentum magnitude — it stays constant because gravity produces zero
  torque about the center of mass.
- **Energy is conserved.** `E_total` (translational KE + rotational KE +
  gravitational PE) stays constant throughout the flight.
- **The red marker traces a complicated path.** The marker is fixed to a
  corner of the brick. Its world-space path is the sum of the smooth
  parabola (translation) and the tumbling rotation — a cycloidal spiral.
  But the center of mass path is a clean parabola.

## The physics

This is the full free joint in action: 3 translational DOFs governed by
`F = mg` and 3 rotational DOFs governed by Euler's equations of rigid body
motion. The key insight is that these two subspaces are **decoupled** —
gravity acts on the center of mass but produces no torque about it, so the
rotation evolves exactly as it would in zero gravity.

The position is stored as 7 numbers in `qpos`: 3 for the center of mass
position and 4 for the orientation quaternion. The velocity is stored as 6
numbers in `qvel`: 3 linear and 3 angular. Gravity enters only through the
first 3 (linear) acceleration components.

## Run

```
cargo run -p example-free-joint-spinning-toss --release
```
