# Free Joint — 6 Degrees of Freedom

A free joint gives a body full translational and rotational freedom — no
constraints, no axes, no limits. It is the joint type used for every
untethered object in the engine: thrown objects, floating robots, spacecraft.

The state is stored as 7 position coordinates (3 position + 4 quaternion)
and 6 velocity coordinates (3 linear + 3 angular). The mismatch (7 vs 6)
comes from the quaternion: it takes 4 numbers to represent an orientation,
but only 3 to represent an angular velocity.

## Examples

| Example | Concept | What you see |
|---------|---------|-------------|
| [tumble](tumble/) | Rotation only | Asymmetric box spinning in zero gravity. Angular velocity precesses but angular momentum is conserved. |
| [projectile](projectile/) | Translation only | Sphere launched at 45 degrees. Parabolic arc matches analytical solution exactly. |
| [spinning-toss](spinning-toss/) | Full 6-DOF | Brick tossed with both linear and angular velocity. COM follows a parabola while the body tumbles — the two motions are decoupled. |
| [stress-test](stress-test/) | Headless validation | 12 checks: quaternion norm, momentum conservation, projectile analytics, dimensions, exp map, energy, and more. |

## Key ideas

- **qpos layout:** `[x, y, z, qw, qx, qy, qz]` — position then unit quaternion.
- **qvel layout:** `[vx, vy, vz, wx, wy, wz]` — linear then angular velocity.
- **Quaternion integration:** Each timestep applies the exponential map
  `q_new = q_old * exp(omega * dt)` on the SO(3) manifold.
- **No limits:** Free joints cannot have limits. The `limited` attribute is
  silently ignored.
- **Decoupling:** Gravity acts on the linear DOFs only. Rotation evolves
  independently — a spinning toss has the same COM trajectory as a
  non-spinning projectile.
