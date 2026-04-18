# Why this study exists

|  |  |
|---|---|
| **What** | A study in building a world-class soft-body simulation stack in Rust. |
| **Why** | Nobody has shipped a clean Rust + Bevy + GPU + differentiable + SDF-driven hyperelastic + IPC soft-body solver. Pieces exist separately (PolyFEM, Warp, Genesis, IPC Toolkit); none integrated. |
| **Goal** | Build the integrated version. No timeline. Ceiling-level quality. |
| **Canonical test** | Compliant cavity–probe conformity optimization — a soft-robotics problem that exercises every subsystem. |
| **Core thesis** | Physically correct and visually great are not separate goals. Modern solvers (neo-Hookean + IPC, differentiable, GPU) give both for free. |
| **Where to start** | [Reading guide](04-reading-guide.md) |
| **Context pages** | [What "ceiling" means in 2026](01-ceiling.md) · [SOTA survey](02-sota.md) |
