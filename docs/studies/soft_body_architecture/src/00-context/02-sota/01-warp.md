# NVIDIA Warp — GPU neo-Hookean, differentiable

Repo: [`github.com/NVIDIA/warp`](https://github.com/NVIDIA/warp) · Apache-2.0 · v1.12.1 (2026-04-06) · Primary reference: Macklin 2022 GTC presentation + NVIDIA tech reports (no single peer-reviewed canonical paper); `warp.fem` implements Smith, Goes, Kim 2018 "Stable Neo-Hookean Flesh Simulation" · Accessed: 2026-04-20

Warp is NVIDIA's Python-first kernel DSL that JIT-compiles to CUDA, with full autograd support. Two soft-body-relevant modules: `warp.fem` (FEM with sparse linear solvers, implementing stable neo-Hookean) and `warp.sim` (XPBD-style particle / constraint dynamics). The DSL is the framing: write Python, compile to CUDA, autograd through the compiled kernels.

- **[Physically correct](../01-ceiling/00-definitions.md#physically-correct):** `partial`. Stable neo-Hookean hyperelastic in `warp.fem` (per Smith et al. 2018), implicit integration via sparse assembly + linear solvers in `warp.fem`, but **no IPC-grade contact** — `warp.fem` uses non-conforming constraint penalty for contact, and `warp.sim`'s soft-body path is XPBD. Missing the IPC component of axis 1.
- **[Visually great](../01-ceiling/00-definitions.md#visually-great):** `—`. Warp is not a renderer; visual quality depends on the host application's rendering of Warp outputs. No FEM-mesh-as-render-mesh commitment.
- **[Real-time](../01-ceiling/00-definitions.md#real-time):** `✓`. Published neo-Hookean benchmarks reach tens-of-thousands-of-tets scenes at 60+ FPS on consumer NVIDIA GPUs per [Part 1 Ch 03](../../10-physical/03-thesis.md) and [Part 11 Ch 03 Phase E](../../110-crate/03-build-order.md#the-committed-order), which derives `sim-soft`'s target envelope from Warp's published numbers. NVIDIA-only (CUDA lock-in).
- **[Differentiable](../01-ceiling/00-definitions.md#differentiable):** `✓`. Full autograd via Warp's tape; kernels are differentiable, gradients propagate through JIT-compiled CUDA, usable in ML training pipelines.

## What `sim-soft` inherits or learns

Warp is the reference implementation for **axes 3 and 4 (real-time + differentiable)**. Its kernel-DSL shape — write Python, compile to CUDA, autograd for free — is the architecture `sim-soft` approximates in Rust + wgpu + [`sim-ml-chassis`](../../110-crate/02-coupling/03-ml-chassis.md)'s GPU-autograd tape ([Part 8 Ch 04](../../80-gpu/04-chassis-extension.md)). Phase E's tet-count/FPS targets are Warp-derived rather than measurements of `sim-soft`. Where `sim-soft` diverges: IPC-grade contact inside the same energy minimization as the elastic solve (Warp's contact is constraint-penalty in `warp.fem` or XPBD in `warp.sim`, neither of which is differentiable in the IPC-barrier sense); and wgpu over CUDA for cross-vendor execution.

## Citation status

- Repo + license + release cadence confirmed from README (v1.12.1 on 2026-04-06).
- Primary reference: no peer-reviewed canonical paper for Warp itself. Cite repo + Macklin 2022 GTC presentation + Smith, Goes, Kim 2018 "Stable Neo-Hookean Flesh Simulation" (implemented in `warp.fem` per `example_mixed_elasticity.py`). Macklin GTC reference URL soft-deferred to Pass 2 (talk/slide reference not independently fetched).
- IPC integration status — absence confirmed weakly from 2026-04-20 fetch (no `warp.contact`-IPC module surfaced in README/examples). Soft-defer to Pass 2 for a repo-structure sweep to rule out hidden IPC integration.
- Published benchmark numbers — Part 1 Ch 03's "tens-of-thousands-of-tets at 60+ FPS" phrasing is load-bearing for Phase E targets; specific benchmark URL soft-deferred to Pass 2.
