# FD Convergence — Epsilon Tuning

Finite-difference derivatives are approximations. Their accuracy depends on the
perturbation size `eps` and the differencing scheme. This example shows the
fundamental tradeoff:

- **Centered differences** `(f(x+e) - f(x-e)) / 2e` have O(eps^2) error —
  reducing eps by 10x reduces error by 100x
- **Forward differences** `(f(x+e) - f(x)) / e` have O(eps) error —
  reducing eps by 10x only reduces error by 10x

## What you see

- **Single pendulum** (1 hinge, Y-axis) with motor actuator, ticking like a
  clock hand through 12 positions around the circle
- Frozen for 3 seconds at 1 o'clock (210 deg), then ticks to the next hour
  every 2 seconds
- At each position the convergence table is recomputed — the ratio row shows
  centered ~100x and forward ~10x regardless of configuration
- Centered error is always smaller than forward error at every epsilon

## Physics

The model is deliberately simple — one DOF, one actuator — so the A matrix is
just 2x2 and B is 2x1. The focus is on numerical analysis, not the dynamics.
The clock-hand sweep demonstrates that convergence rates are a property of the
differencing scheme, not the particular state.

Ground truth is centered differences at eps=1e-8. Test values (1e-2, 1e-3) are
well above the model's precision floor (~1e-7), keeping ratios clean. At
smaller epsilons, centered convergence stalls as roundoff overtakes truncation
error — that tradeoff is a separate concept (the "FD V-curve").

| Parameter | Value |
|-----------|-------|
| DOF | 1 (hinge, damping=0) |
| Actuators | 1 (motor) |
| Test epsilons | 1e-2, 1e-3 |
| Ground truth | centered, eps=1e-8 |
| Clock positions | 12, starting at 1 o'clock (210 deg) |
| Tick interval | 2s wall time |

## Validation

Four automated checks at startup:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Centered monotone convergence** | error decreases with eps | strict |
| **Forward monotone convergence** | error decreases with eps | strict |
| **Centered < forward at every eps** | centered strictly more accurate | strict |
| **fd_convergence_check passes** | default eps converges | tol=1e-4 |

## Run

```
cargo run -p example-derivatives-convergence --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
