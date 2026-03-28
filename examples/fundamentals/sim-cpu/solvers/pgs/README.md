# PGS Solver

Two boxes stacked on a ground plane, solved with Projected Gauss-Seidel.

## What you see

A red box (1.0 kg) on the ground with a blue box (0.5 kg) on top.
The HUD shows iteration count (~40-55 per step), contact count (8),
max penetration depth (0.026 mm), and energy.

## Expected behavior

The stack is perfectly stable. PGS converges within the 100-iteration
budget on every step, averaging ~47 iterations. This is the most
iterations of the three solvers — PGS uses coordinate-wise descent
with linear convergence.

## Run

```
cargo run -p example-solver-pgs --release
```
