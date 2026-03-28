# Solvers — Visual Comparison

Three identical two-box stacks side by side, each using a different
constraint solver. The stacks look identical — the difference is in
the HUD iteration counts.

## What you see

Three stacks spaced horizontally:
- **Left** — PGS (red box_a)
- **Center** — CG (gold box_a)
- **Right** — Newton (green box_a)

The HUD shows per-step iteration count for each solver. Newton sits at
0 most of the time (quadratic convergence). CG fluctuates around 30.
PGS hovers around 47.

## Expected behavior

All three stacks remain perfectly stable indefinitely. No drift, no
wobble, no collapse. The visual output is identical — the solvers
differ only in how many iterations they need to reach the same answer.

## Run

```
cargo run -p example-solver-comparison-visual --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
