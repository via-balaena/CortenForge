# Euler — Semi-Implicit Euler Integration

The simplest integrator: update velocity, then use that new velocity to
update position. First-order accurate. Fast, but accumulates energy error
at coarse timesteps.

## What you see

A single pendulum released from horizontal, swinging under gravity with
zero damping. Visually identical to every other integrator example — the
difference is in the numbers.

## What to watch

The **HUD drift value** climbs from 0% toward +0.3% over 15 seconds.
Positive drift means energy gain — the pendulum swings fractionally higher
each cycle. At dt = 0.005 the visual amplitude creep is subtle, but the
numbers don't lie.

Compare: RK4 stays at +0.000% on the same scene.

## How it works

```
v_{n+1} = v_n + h * a(q_n)       velocity updated first
q_{n+1} = q_n + h * v_{n+1}      position uses NEW velocity
```

This "semi-implicit" (symplectic) variant is better than naive forward
Euler — it preserves phase space volume and keeps bounded energy error.
But the error still grows linearly with time, and at coarse timesteps
that growth is measurable.

## Parameters

| Parameter | Value |
|-----------|-------|
| Integrator | Semi-implicit Euler |
| Timestep | 0.005 s (deliberately coarse) |
| Body mass | 1.0 kg |
| Arm length | 0.5 m (CoM at 0.25 m) |
| Damping | 0 |
| Initial angle | 90 deg (horizontal) |

## Validation

| Check | Criterion |
|-------|-----------|
| Euler drifts visibly | \|drift\| > 0.1% of m*g*d |

Inverted check — we want drift. This example exists to show first-order
error, not to demonstrate accuracy.

## Run

```
cargo run -p example-integrator-euler --release
```
