# ImplicitFast — Implicit Without Coriolis

A variant of the full implicit integrator. Assembles the same velocity
derivative matrix D but **skips Coriolis terms**, and takes `M - h*D` on
the mass matrix's sparsity pattern.

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
(M - h*D) * a = qfrc_smooth + qfrc_constraint
qvel <- qvel + h * a
```

`a` is the acceleration the step advances `qvel` with; `data.qacc` keeps the
explicit acceleration, as in MuJoCo.

Where D excludes Coriolis terms and `M - h*D` is taken on the mass
matrix's pattern (its lower triangle; a simple dof's row its diagonal
only), factored as `L'*D*L` with no pivot check, as MuJoCo's
`mj_factorI`.

## Parameters

| Parameter | Value |
|-----------|-------|
| Integrator | ImplicitFast |
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
