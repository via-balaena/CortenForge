# Implicit — Full Implicit with Coriolis

The most accurate implicit integrator. Assembles the full velocity
derivative matrix D (including Coriolis terms from `mjd_rne_vel`), then
solves `(M - h*D) * qacc = rhs` via LU factorization. Unconditionally
stable for stiff systems.

## What you see

A single pendulum released from horizontal, swinging under gravity with
zero damping. Same scene as every other integrator example.

## What to watch

The **HUD drift value** reads about +0.24% after 15 seconds — similar to
Euler. This is expected: on a zero-damping, zero-stiffness system there
are no springs or dampers to treat implicitly. The implicit solver
reduces to explicit Euler behavior because D = 0 (no velocity-dependent
forces to linearize).

The implicit integrators earn their keep on **stiff systems** — high joint
stiffness, strong damping, tendon coupling. On a free pendulum they have
nothing to implicitly integrate.

## How it works

```
(M - h*D) * qacc = qfrc_smooth + qfrc_constraint
```

Where D = dF/dv is the full Jacobian of smooth forces with respect to
velocity, including:
- DOF damping
- Tendon damping (J^T * B * J)
- Actuator velocity derivatives
- Coriolis velocity derivatives (mjd_rne_vel)

D is asymmetric (Coriolis breaks symmetry), so LU factorization is used
instead of Cholesky. This is the most expensive implicit variant but also
the most accurate for systems with significant Coriolis coupling.

## Parameters

| Parameter | Value |
|-----------|-------|
| Integrator | Full implicit (Coriolis + LU) |
| Timestep | 0.005 s |
| Body mass | 1.0 kg |
| Arm length | 0.5 m (CoM at 0.25 m) |
| Damping | 0 |
| Initial angle | 90 deg (horizontal) |

## Validation

| Check | Criterion |
|-------|-----------|
| Implicit stable | \|drift\| < 0.5% of m*g*d |

## Run

```
cargo run -p example-integrator-implicit --release
```
