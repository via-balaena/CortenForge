# Tendons — Routing, Wrapping, and Limits

Tendons are scalar-valued elements that transmit force between joints and
actuators. Two flavors: **fixed tendons** (linear combination of joint
positions) and **spatial tendons** (3D paths routed through sites with
optional wrapping around geometry). Tendons can have limits, spring-damper
dynamics, pulleys, and serve as actuator transmission.

## Examples

| Example | Concept | What you see |
|---------|---------|-------------|
| [stress-test](stress-test/) | Headless validation | 16 checks: fixed/spatial length and velocity, sensor readback, wrapping geometry, limits, pulley scaling, actuator forces, spring/damper, Jacobian properties, multi-tendon composition. |
| [fixed-coupling](fixed-coupling/) | Fixed tendon coupling | Two pendulums coupled by coefficients [1.0, -0.5] — joint A moves +1 rad, joint B follows -0.5 rad via tendon spring. |
| [spatial-path](spatial-path/) | 3D site-to-site routing | 2-link arm with tendon through 3 sites — length tracks sum of Euclidean segment distances. |
| [sphere-wrap](sphere-wrap/) | Sphere wrapping | Tendon wraps around a sphere via great-circle arc — configuration-dependent moment arm. |
| [cylinder-wrap](cylinder-wrap/) | Cylinder wrapping | Tendon follows helical geodesic around a cylinder — different wrap geometry than sphere. |
| [tendon-limits](tendon-limits/) | Range constraints | Pendulum tethered by a limited tendon — bounces back when tether goes taut. |
| [pulley](pulley/) | Mechanical advantage | 2:1 pulley — bottom mass pulls twice as far as top mass moves. Ruler ticks visualize the ratio. |
| [tendon-actuator](tendon-actuator/) | Tendon vs joint transmission | Same control signal, different response — tendon transmission has configuration-dependent gear ratio. |

## Key ideas

- **`<tendon><fixed>`:** Length = `Σ coef_i × q_i`. Constant Jacobian.
  Couples joints algebraically without a physical path.
- **`<tendon><spatial>`:** Length = sum of 3D segment distances through
  sites. Jacobian varies with configuration. Foundation for wrapping,
  pulleys, and anatomical routing.
- **Wrapping (`<geom>` in spatial path):** Sphere wrap follows a
  great-circle arc; cylinder wrap follows a helical geodesic. Both create
  configuration-dependent moment arms.
- **`<pulley divisor="N"/>`:** Splits the tendon into branches. The branch
  after the pulley contributes `length / N`, creating mechanical advantage.
- **`limited="true"` + `range="lo hi"`:** One-sided constraint on tendon
  length. `TendonLimitFrc` reports zero when slack, positive when taut.
- **Spring-damper:** `stiffness` and `damping` on a tendon create passive
  forces: `F = -k(L - L0) - bV`.
- **Tendon-driven actuator (`<general tendon="...">`):** Maps actuator
  force through the tendon Jacobian. Effective torque at the joint varies
  with configuration (unlike direct joint transmission).
- **`TendonPos` / `TendonVel` / `TendonLimitFrc`:** Sensor types for
  tendon length, velocity, and limit constraint force.
