# Hybrid vs FD — Analytical+FD Comparison

Side-by-side comparison of the two linearization methods: pure finite
differences (`mjd_transition_fd`) vs hybrid analytical+FD
(`mjd_transition_hybrid`). Both produce the same A matrix, but hybrid
replaces FD perturbation of velocity columns with analytical derivatives
from `qDeriv`, cutting wall-clock time.

## What you see

- **3-link pendulum** frozen at non-trivial angles (0.5, -0.8, 1.2 rad)
- HUD stages a **race** between the two methods:
  1. 3-second countdown while you read the setup
  2. "Computing FD..." flashes, then FD time appears
  3. Brief pause, then "Computing Hybrid..." flashes, hybrid time appears
  4. **Winner declared** with speedup ratio
  5. Error analysis and |A_fd - A_hyb| difference matrix

The pendulum doesn't move. This example is about computational comparison,
not simulation.

## Physics

The hybrid method exploits the structure of the dynamics:

```
x_{t+1} = f(x_t, u_t)
```

Position columns of A (how next-state depends on current position) require
FD because position perturbations change contact geometry. But velocity
columns (how next-state depends on current velocity) flow through smooth
force computation, which has analytical derivatives stored in `qDeriv`.
Hybrid uses analytical for velocity, FD only for position — roughly halving
the number of simulation steps needed.

| Parameter | Value |
|-----------|-------|
| DOF | 3 (hinge chain) |
| Actuators | 0 |
| State dim | 6 (3 dq + 3 qvel) |
| A matrix | 6x6 |
| Integrator | Euler (supports hybrid) |
| FD config | centered, eps = 1e-6 |
| qpos | [0.5, -0.8, 1.2] rad |

## Validation

Four automated checks at startup:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **A dimensions match** | both 6x6 | exact |
| **A agreement** | max relative error < 1e-4 | 1e-4 |
| **B agreement** | max relative error < 1e-4 (both 6x0) | 1e-4 |
| **Hybrid not slower** | hybrid time <= FD time * 1.2 | 20% margin |

## Run

```
cargo run -p example-derivatives-hybrid-vs-fd --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
