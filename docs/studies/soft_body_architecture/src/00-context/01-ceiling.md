# What "ceiling" means in 2026

The book uses the word *ceiling* deliberately. Every system design has an implicit ceiling — the best result reachable before the tools themselves become the bottleneck. In soft-body simulation, the ceiling has been set for decades by a forced choice: pick real-time *or* pick physical correctness, because you cannot have both. This chapter names what the ceiling looks like in 2026 (when the choice no longer holds), why four previously-separate goals collapse to one target, and which pieces of that target nobody has yet integrated.

| Section | What it covers |
|---|---|
| [Physically correct, visually great, real-time, differentiable](01-ceiling/00-definitions.md) | The four axes the ceiling is measured on; precise, testable definitions for each |
| [The convergence thesis](01-ceiling/01-convergence.md) | Why the four axes have collapsed to one target in 2026 — GPU throughput, IPC made real-time, adjoint-as-byproduct |
| [What nobody has shipped yet — the gap](01-ceiling/02-the-gap.md) | The integrated Rust + Bevy + GPU + differentiable + SDF + hyperelastic + IPC solver does not exist; this book's whole motivation |

Two claims Ch 01 rests on:

1. **The ceiling is measurable, not aspirational.** Each of the four axes has a quantitative definition (sub-chapter 00) that a candidate solver either satisfies or does not. The [SOTA survey](02-sota.md) grades nine existing solvers against these four.
2. **The ceiling is reachable in 2026.** No axis requires undiscovered technique; each has at least one reference implementation that hits it in isolation. The gap is integration, not research — which is what makes this a design study rather than a PhD.
