# Spring-Damper Tuning — Second-Order Response

Three identical hinge arms in zero gravity with the same torsional spring but
different damping. Damping values are computed from the mass matrix to hit
exact damping ratios, producing the three classical second-order responses.

## What it demonstrates

- **Underdamped (blue, zeta=0.1)**: oscillates with decaying amplitude.
  23 zero-crossings. Period matches analytical within 0.5%.
- **Critically damped (green, zeta=1.0)**: fastest return to equilibrium
  without overshoot. Settles in ~1.3s after release.
- **Overdamped (red, zeta=3.0)**: sluggish exponential decay, no oscillation.
  Settles in ~5.3s after release (~4x slower than critical).

## What you see

A 2-second hold at the starting angle, then all three release. The blue arm
oscillates back and forth with decreasing swings. The green arm returns
smoothly in one motion. The red arm creeps back slowly. The HUD shows angle,
angular velocity, and system parameters.

## Run

```
cargo run -p example-passive-spring-damper-tuning --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
