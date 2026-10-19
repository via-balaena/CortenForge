# PGS Solver

Two boxes stacked on a ground plane, solved with Projected Gauss-Seidel.

## What you see

A red box (1.0 kg) on the ground with a blue box (0.5 kg) on top.
The HUD shows iteration count (1-100 per step), contact count (8),
max penetration depth (0.028 mm once settled), and energy.

## Expected behavior

The stack is perfectly stable. PGS averages ~49 iterations and reaches
its 100-iteration budget on 97 of 2500 steps, keeping its last iterate
there, as MuJoCo's does. This is the most
iterations of the three solvers — PGS uses coordinate-wise descent
with linear convergence.

## Run

```
cargo run -p example-solver-pgs --release
```
