# ImplicitFast — Symmetric D, Cholesky Factorization

A faster variant of the full implicit integrator. Assembles the same
velocity derivative matrix D but **skips Coriolis terms** and
**symmetrizes** D before solving. This allows Cholesky factorization
(cheaper than LU) while keeping the stability benefits of implicit
integration.

## What you see

A single pendulum released from horizontal, swinging under gravity with
zero damping. Same scene as every other integrator example.

## What to watch

The **HUD drift value** reads about +0.24% after 15 seconds — identical to
Euler and full Implicit on this scene. Again, with zero damping and zero
stiffness, D = 0 and the implicit solve degenerates to explicit behavior.

The difference between ImplicitFast and full Implicit shows up on systems
with significant Coriolis coupling (multi-link chains, gyroscopic bodies).
On a single hinge pendulum, Coriolis is a scalar and both methods agree.

## How it works

```
(M - h*D_sym) * qacc = qfrc_smooth + qfrc_constraint
```

Where D_sym = (D + D^T) / 2 is the symmetrized Jacobian, excluding
Coriolis terms. The symmetry guarantee means M - h*D_sym is SPD
(assuming D doesn't dominate M), so Cholesky factorization can be used.

Cholesky is roughly 2x faster than LU for dense matrices and avoids
pivoting. The tradeoff: skipping Coriolis makes this slightly less
accurate on spinning multi-body systems.

## Parameters

| Parameter | Value |
|-----------|-------|
| Integrator | ImplicitFast (symmetric D, Cholesky) |
| Timestep | 0.005 s |
| Body mass | 1.0 kg |
| Arm length | 0.5 m (CoM at 0.25 m) |
| Damping | 0 |
| Initial angle | 90 deg (horizontal) |

## Validation

| Check | Criterion |
|-------|-----------|
| ImplicitFast stable | \|drift\| < 0.5% of m*g*d |

## Run

```
cargo run -p example-integrator-implicit-fast --release
```
