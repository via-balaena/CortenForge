# Newton Solver

Two boxes stacked on a ground plane, solved with the full Newton method.

## What you see

A red box (1.0 kg) on the ground with a blue box (0.5 kg) on top.
The HUD shows iteration count (0-2 per step), contact count (8),
fallback status ("yes" = Newton converged, "FALLBACK" = fell back to
PGS), max penetration depth (0.026 mm), and energy.

## Expected behavior

The stack is perfectly stable. Newton converges in 0-2 iterations on
this simple scene — quadratic convergence near the solution means it
barely needs to iterate. The "solved: yes" indicator should stay green
on nearly every step. On the rare step where Newton doesn't converge,
it falls back to PGS automatically.

## Run

```
cargo run -p example-solver-newton --release
```
