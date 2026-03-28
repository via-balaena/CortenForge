# ImplicitSpringDamper — Diagonal Spring/Damper Implicit

The legacy implicit integrator, designed specifically for stiff springs and
dampers. Treats per-DOF spring and damper forces implicitly by adding
`h*D + h^2*K` to the mass matrix diagonal, then solving via Cholesky. Only
handles diagonal (per-DOF) coupling — tendon cross-coupling is added
separately.

## What you see

A single pendulum released from horizontal, swinging under gravity with
zero damping. Same scene as every other integrator example.

## What to watch

The **HUD drift value** reads about +0.36% after 15 seconds — slightly
worse than Euler's +0.24%. With zero stiffness and zero damping, K = 0
and D = 0, so the modified mass matrix M + h*D + h^2*K = M (unchanged).
The solver reduces to `M * v_new = M*v_old + h*f_ext`, which is explicit
Euler. The slightly higher drift comes from a different code path through
the Newton solver interaction.

This integrator's strength is unconditional stability on stiff springs:
a spring with stiffness k = 10000 N/m that would blow up under Euler
remains perfectly stable under ImplicitSpringDamper, because the `h^2*K`
term in the mass matrix absorbs the stiffness implicitly.

## How it works

```
(M + h*D + h^2*K) * v_new = M*v_old + h*f_ext - h*K*(q - q_eq)
```

Where K and D are diagonal matrices of per-DOF spring stiffness and
damping. The modified mass matrix is still SPD (M is SPD, D >= 0, K >= 0),
so Cholesky factorization is used.

After solving for v_new: `qacc = (v_new - v_old) / h`, then position
integration proceeds normally.

## Parameters

| Parameter | Value |
|-----------|-------|
| Integrator | ImplicitSpringDamper (diagonal, Cholesky) |
| Timestep | 0.005 s |
| Body mass | 1.0 kg |
| Arm length | 0.5 m (CoM at 0.25 m) |
| Damping | 0 |
| Stiffness | 0 |
| Initial angle | 90 deg (horizontal) |

## Validation

| Check | Criterion |
|-------|-----------|
| ImplicitSpringDamper bounded | \|drift\| < 5% of m*g*d |

Threshold is the most generous of all integrators because with zero K/D
this integrator degenerates to Euler-like behavior via a different code
path (the non-Newton implicit acceleration solver updates velocity
directly). The energy oscillates more than other methods and the drift
at any given snapshot can be higher.

## Run

```
cargo run -p example-integrator-implicit-spring-damper --release
```
