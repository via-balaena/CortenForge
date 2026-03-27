# RK4 — 4th-Order Runge-Kutta Integration

The gold standard for accuracy on smooth problems. Four evaluations of the
dynamics per step, combined with carefully chosen weights from the Butcher
tableau. Fourth-order accurate — energy drift is essentially zero at
dt = 0.005.

## What you see

A single pendulum released from horizontal, swinging under gravity with
zero damping. Visually indistinguishable from the Euler example — the
pendulum swings identically. The difference is entirely in the energy
conservation.

## What to watch

The **HUD drift value** stays at +0.000% for the entire run. After 15
seconds (3000 steps), the accumulated drift is -0.000001% of m*g*d. After
90 seconds it's still -0.0000. The pendulum will swing with the same
amplitude forever.

Compare: Euler drifts to +0.24% over the same interval — 4500x worse.

## How it works

Classic 4-stage RK4 with Butcher tableau weights [1/6, 1/3, 1/3, 1/6]:

```
k1 = f(t_n, y_n)
k2 = f(t_n + h/2, y_n + h*k1/2)
k3 = f(t_n + h/2, y_n + h*k2/2)
k4 = f(t_n + h, y_n + h*k3)
y_{n+1} = y_n + h/6 * (k1 + 2*k2 + 2*k3 + k4)
```

Each stage evaluates the full forward dynamics at a trial state. The
weighted combination cancels error terms through 4th order, giving local
truncation error O(h^5) and global error O(h^4). Halving the timestep
reduces energy drift by 16x.

The cost is 4x that of Euler per step (four `forward()` evaluations
vs one). For smooth problems without stiff springs, the accuracy gain
far outweighs the cost.

## Parameters

| Parameter | Value |
|-----------|-------|
| Integrator | 4-stage Runge-Kutta |
| Timestep | 0.005 s |
| Body mass | 1.0 kg |
| Arm length | 0.5 m (CoM at 0.25 m) |
| Damping | 0 |
| Initial angle | 90 deg (horizontal) |

## Validation

| Check | Criterion |
|-------|-----------|
| RK4 near-perfect | \|drift\| < 0.001% of m*g*d |

## Run

```
cargo run -p example-integrator-rk4 --release
```
