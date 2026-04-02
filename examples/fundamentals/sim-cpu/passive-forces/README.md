# Passive Forces — Fluid Drag, Spring-Damper Tuning, Wind

Passive forces are configuration/velocity-dependent forces the engine computes
automatically — no actuator needed. The pipeline produces separate component
arrays (`qfrc_spring`, `qfrc_damper`, `qfrc_fluid`, `qfrc_gravcomp`) that
aggregate into `qfrc_passive`.

## Examples

| Example | Concept | What you see |
|---------|---------|-------------|
| [stress-test](stress-test/) | All subsystems | Headless: 18 checks against analytical predictions |
| [fluid-drag](fluid-drag/) | Inertia-box drag | 3 spheres (different mass) reach different terminal velocities |
| [ellipsoid-drag](ellipsoid-drag/) | Per-geom 5-component drag | 3 shapes (capsule/sphere/cylinder) — shape determines drag |
| [wind](wind/) | Global wind field | Sphere drifts sideways, pendulum deflects then recovers |
| [spring-damper-tuning](spring-damper-tuning/) | Second-order response | Underdamped/critical/overdamped — 3 classical damping regimes |

## Key ideas

- **Two fluid models**: inertia-box (body-level, automatic) and ellipsoid
  (per-geom, 5 force components: added mass, Magnus lift, Kutta lift,
  linear drag, angular drag). Activated by `fluidshape="ellipsoid"`.
- **Terminal velocity**: drag force = weight, net acceleration = 0. Heavier
  objects or streamlined shapes reach higher terminal velocities.
- **Wind**: `<option wind="vx vy vz"/>` subtracts from body velocity before
  computing drag. A stationary body in wind feels the same force as a body
  moving at -wind in still air.
- **Spring-damper**: `stiffness` and `damping` on joints produce second-order
  response. Damping ratio zeta = c / (2*sqrt(k*I)) determines character:
  underdamped (oscillates), critical (fastest, no overshoot), overdamped
  (sluggish return).
