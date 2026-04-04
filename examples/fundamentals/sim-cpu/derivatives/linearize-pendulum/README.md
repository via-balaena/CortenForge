# Linearize Pendulum — Pure FD Transition Derivatives

A single-link pendulum held at the unstable upright equilibrium. One call to
`mjd_transition_fd()` produces the 2x2 state-transition matrix A, which
encodes how small perturbations grow or shrink over one timestep. Eigenvalue
analysis of A reveals that the upright position is unstable — any nudge
diverges exponentially.

## What you see

- **Metal rod** with a **blue tip sphere** — a single-link arm starting
  nearly upright from the pivot (qpos = pi + 0.01)
- The HUD shows the 2x2 A matrix, eigenvalues, and stability verdict
  (computed at exact upright before the nudge)
- After a brief pause the pendulum **falls** — a tiny 0.01 rad perturbation
  diverges exponentially, exactly as the eigenvalue predicts
- The live section of the HUD tracks angle, deviation from upright, and
  angular velocity as it falls

## Physics

The discrete-time transition linearization:

```
x_{t+1} = f(x_t, u_t)   -->   dx_{t+1} ~ A * dx_t + B * du_t
```

For a 1-DOF hinge with no actuator, the state is `x = [dq, qvel]` (2D) and
there is no control input, so A is 2x2 and B is 2x0.

At the upright equilibrium (theta = pi), gravity is an *inverted* restoring
force — it pushes perturbations away rather than pulling them back. This shows
up as an eigenvalue with |lambda| > 1 in discrete time. The continuous-time
equivalent is a positive real eigenvalue ~ sqrt(g/L).

| Parameter | Value |
|-----------|-------|
| DOF | 1 (hinge) |
| Actuators | 0 |
| State dim | 2 (dq, qvel) |
| Equilibrium | upright (qpos = pi) |
| Timestep | 2 ms |
| FD method | centered, eps = 1e-6 |

## Expected HUD output

```
A (2x2):
  [  1.000000,   0.002000]
  [  0.019620,   1.000000]

B (2x0): no actuators

Eigenvalues:
  l1 = 1.006261    |l1| = 1.006261
  l2 = 0.993778    |l2| = 0.993778

Stability: UNSTABLE (max |l| = 1.006261)

C: None  D: None
```

One eigenvalue exceeds 1.0 — the upright pendulum is unstable. The other is
below 1.0 — perturbations in the stable direction shrink. This is the classic
saddle-point structure of an inverted pendulum.

## Validation

Four automated checks at startup:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **A is 2x2** | 2*nv x 2*nv, nv=1 | exact |
| **B is 2x0** | no actuators (nu=0) | exact |
| **Unstable eigenvalue** | max \|lambda\| > 1 at upright | > 1.0 |
| **C, D are None** | sensor derivatives not requested | exact |

## Run

```
cargo run -p example-derivatives-linearize-pendulum --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
