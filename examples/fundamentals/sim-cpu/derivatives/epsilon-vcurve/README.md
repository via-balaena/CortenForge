# Epsilon V-Curve — Finding the Optimal Perturbation Size

Finite-difference derivatives have a sweet spot. Make the perturbation `eps` too
large and truncation error dominates — the linear approximation is poor. Make it
too small and roundoff error dominates — the numerator `f(x+e) - f(x-e)` loses
significant digits to floating-point cancellation. The optimal epsilon balances
these two forces.

This example sweeps eps from 1e-2 down to 1e-10 and compares each centered FD
result against the hybrid analytical reference. The error traces out a V-shape
on a log-log scale, with the minimum at the optimal epsilon.

## What you see

- **Single pendulum** (1 hinge, Y-axis) with motor actuator, ticking like a
  clock hand through 12 positions
- At each position the V-curve is recomputed — the HUD shows error at each
  epsilon with an arrow marking the minimum
- At most positions the V-curve has a clear minimum at **eps ~ 1e-4**, with
  error rising on both sides
- At **6 o'clock (0 deg) and 12 o'clock (180 deg)** the V-curve degenerates:
  these are equilibrium positions where the derivatives are symmetric/trivial,
  so FD and hybrid agree down to machine epsilon (~1e-16). The V becomes an
  L-shape — error decreases monotonically and flatlines at the precision floor
  without a visible right arm. The optimal epsilon shifts to 1e-8 or smaller.
  This is correct behavior: simpler dynamics allow smaller perturbations.

## Physics

The reference is computed via `mjd_transition_hybrid()`, which uses analytical
formulas for the velocity columns of the A matrix. This gives an
epsilon-independent baseline for comparison. The position columns still use FD
internally, but the velocity column comparison drives the V-curve shape.

For centered differences, the theoretical error is:

```
total_error = C * eps² + roundoff / eps
```

where `C` depends on the third derivative of the dynamics. Minimizing gives
optimal eps ~ (roundoff / C)^(1/3). For f64 (roundoff ~ 1e-16) this lands
around eps ~ 1e-5 to 1e-4, which matches what the example shows.

| Parameter | Value |
|-----------|-------|
| DOF | 1 (hinge) |
| Actuators | 1 (motor) |
| Epsilon sweep | 1e-2, 1e-3, ..., 1e-10 (9 values) |
| Reference | hybrid analytical (velocity columns) |
| Clock positions | 12, starting at 1 o'clock |
| Tick interval | 2s wall time |

## Validation

Four automated checks at startup:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **V-curve has interior minimum** | min not at endpoints | exact |
| **Left arm: error decreases** | truncation-dominated | strict |
| **Right arm: error increases** | roundoff-dominated | strict |
| **Optimal eps in [1e-8, 1e-4]** | reasonable sweet spot | range |

Note: checks are validated at the starting position (1 o'clock, 210 deg). At
equilibrium positions (6 and 12 o'clock), the interior minimum check would fail
because the curve degenerates — this is expected, not a bug.

## Run

```
cargo run -p example-derivatives-epsilon-vcurve --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
