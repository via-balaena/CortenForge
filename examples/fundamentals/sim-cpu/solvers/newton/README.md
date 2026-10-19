# Newton Solver

Two boxes stacked on a ground plane, solved with the full Newton method.

## What you see

A red box (1.0 kg) on the ground with a blue box (0.5 kg) on top.
The HUD shows iteration count (1-2 per step), contact count (8),
how Newton stopped ("tolerance" = converged, "AT CAP" = stopped at its
iteration cap with its last iterate), max penetration depth (0.028 mm
once settled),
and energy.

## Expected behavior

The stack is perfectly stable. Newton converges in 1-2 iterations on
this simple scene — quadratic convergence near the solution means it
barely needs to iterate. The "stopped: tolerance" indicator should stay
on every step. Newton never falls back to another solver: at its
iteration cap it keeps its last iterate, as MuJoCo's does.

## Run

```
cargo run -p example-solver-newton --release
```
