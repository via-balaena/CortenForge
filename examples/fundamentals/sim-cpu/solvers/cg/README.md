# CG Solver

Two boxes stacked on a ground plane, solved with Conjugate Gradient.

## What you see

A red box (1.0 kg) on the ground with a blue box (0.5 kg) on top.
The HUD shows iteration count (1-25 per step), contact count (8),
max penetration depth (0.028 mm once settled), and energy.

## Expected behavior

The stack is perfectly stable. CG converges faster than PGS, averaging
~1.2 iterations (vs PGS's ~49). CG uses the inverse mass matrix as a
preconditioner, giving it superlinear convergence on well-conditioned
contact problems.

## Run

```
cargo run -p example-solver-cg --release
```
