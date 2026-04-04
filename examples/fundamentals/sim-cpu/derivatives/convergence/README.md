# FD Convergence — Epsilon Tuning

Finite-difference derivatives are approximations. Their accuracy depends on the
perturbation size `eps` and the differencing scheme. This example shows the
fundamental tradeoff:

- **Centered differences** `(f(x+e) - f(x-e)) / 2e` have O(eps^2) error —
  halving eps reduces error by 4x
- **Forward differences** `(f(x+e) - f(x)) / e` have O(eps) error —
  halving eps only halves the error

The HUD displays a convergence table at five epsilon values (1e-3 through 1e-7),
with error ratios between consecutive rows confirming the theoretical rates:
~100x reduction per decade for centered, ~10x for forward.

## What you see

- **Single pendulum** (1 hinge, Y-axis) with motor actuator, frozen at
  q=0.8 rad — this is a static snapshot, not a simulation
- The HUD shows the convergence table: `eps | centered error | forward error`
- Below the table, error ratios between consecutive epsilon values show the
  convergence rate
- Centered error is always smaller than forward error at every epsilon

## Physics

The model is deliberately simple — one DOF, one actuator — so the A matrix is
just 2x2 and B is 2x1. The focus is on numerical analysis, not the dynamics.
A non-zero state (q=0.8, qvel=0.5, ctrl=0.3) ensures nontrivial derivative
values that exercise the convergence behavior.

Ground truth is defined as centered differences at eps=1e-7 (the smallest
tested value). All other results are compared against this reference.

| Parameter | Value |
|-----------|-------|
| DOF | 1 (hinge) |
| Actuators | 1 (motor) |
| State | qpos=0.8, qvel=0.5, ctrl=0.3 |
| Epsilon range | 1e-3, 1e-4, 1e-5, 1e-6 |
| Ground truth | centered, eps=1e-7 |

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
