# Slider-Crank — Mechanical Linkage Transmission

A slider-crank mechanism that converts rotation into reciprocating linear
motion. A motor spins the crank, and the slider-crank transmission computes
the configuration-dependent moment arm that a physical connecting rod would
produce. The piston reciprocates inside a cylinder housing, driven by a servo
tracking the kinematic curve.

## What you see

- **Crank arm** (silver) — spinning continuously around a pivot, driven by a
  constant-torque motor
- **Connecting rod** (silver) — angled link from the crank pin to the wrist pin,
  changing angle each revolution
- **Piston rod** (silver) — horizontal bar sliding through the cylinder housing,
  connecting the wrist pin to the piston head
- **Piston head** (gold cylinder) — reciprocating inside the translucent cylinder
  housing, reaching the back wall at top dead center
- **Cylinder housing** (dark translucent) — stationary barrel the piston slides
  through

Watch the crank speed: it wobbles slightly each revolution. The transmission
torque assists the motor in one half-revolution and resists it in the other.
At dead center (crank aligned with the piston axis) the moment arm drops to
zero — no torque transmitted. At 90 deg the moment arm peaks.

## Physics

A `<general cranksite="..." slidersite="..." cranklength="...">` actuator
defines a slider-crank transmission. The transmission computes actuator length
and moment arms from the kinematic geometry of the linkage:

```
length = gear * (av - sqrt(det))
where:
  av  = (crank_pos - slider_pos) . slider_axis
  det = av^2 + L^2 - |crank_pos - slider_pos|^2
```

The moment arm (dl/d_theta) varies with crank angle:

```
Dead center (theta = 0, 180 deg):  moment arm ~ 0    (no torque)
Max advantage (theta ~ 90 deg):   moment arm ~ 0.11  (peak torque)
```

A motor provides constant torque (2 Nm) to keep the crank spinning through
dead centers. The slider-crank actuator (3 N) adds configuration-dependent
torque on the crank, creating the visible speed wobble. A position servo
drives the piston along the kinematic curve (simulating the rod constraint
that MuJoCo's transmission does not physically enforce).

| Parameter | Value |
|-----------|-------|
| Crank radius | 0.1 m |
| Rod length (cranklength) | 0.2 m |
| Motor torque | 2.0 Nm (constant) |
| Linkage force | 3.0 N (gain=3, ctrl=1) |
| Crank damping | 0.3 Nm·s/rad |
| Crank armature | 0.05 kg·m² (flywheel) |
| Piston servo | kp=200, critically damped |
| Integrator | RK4, dt = 1 ms |

**Key distinction from site transmission:** site transmission applies a
Cartesian force at a site and maps it through the Jacobian. Slider-crank
models a specific linkage geometry — the force distribution depends on
crank angle, rod length, and slider position, not just the Jacobian.

## Expected console output

```
t=  1.0s  crank=300.4 deg  slider=-0.1182 m  arm=0.0620
t=  5.0s  crank= 36.2 deg  slider=+0.0021 m  arm=-0.0341
t= 10.0s  crank=139.3 deg  slider=-0.0854 m  arm=-0.0913
t= 15.0s  crank=288.1 deg  slider=-0.1291 m  arm=0.0706
```

The moment arm oscillates between 0 (dead center) and ~0.11 (peak). The
slider reciprocates over ~0.15 m each revolution.

## Validation

Four automated checks at t=17s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Length varies** | Actuator length range > 0.01 | structural |
| **Variable moment arm** | Moment arm range > 0.01 | structural |
| **Dead center reached** | min \|moment\| < 0.02 | < 0.02 |
| **Transmission active** | max \|moment\| > 0.05 | structural |

## Run

```
cargo run -p example-actuator-slider-crank --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
