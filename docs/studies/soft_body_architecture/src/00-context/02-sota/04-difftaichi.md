# DiffTaichi — differentiable DSL

Primary paper: [Hu, Anderson, Li, Sun, Carr, Ragan-Kelley, Durand 2020 "DiffTaichi: Differentiable Programming for Physical Simulation," ICLR 2020 (arXiv:1910.00935)](https://arxiv.org/abs/1910.00935) · Examples repo: [`github.com/taichi-dev/difftaichi`](https://github.com/taichi-dev/difftaichi) (stale since 2021; framework absorbed into Taichi core at [`github.com/taichi-dev/taichi`](https://github.com/taichi-dev/taichi)) · Accessed: 2026-04-20

DiffTaichi is a differentiable programming DSL — **not a framework on top of PyTorch**, and not a shipped soft-body solver. It is a source-to-source compilation from a Python-embedded DSL to differentiable CUDA kernels, with autograd via a lightweight tape. The paper's contribution is the DSL + compilation path; the example simulators (elastic, MPM, fluid) are illustrative, not production-grade solvers. The scoring below is for DSL-enabled simulators built on DiffTaichi, not for any one shipped library.

- **[Physically correct](../01-ceiling/00-definitions.md#physically-correct):** `partial`. Constitutive models are user-written in the DSL; example simulators include simple neo-Hookean-style elastic and MPM. No IPC-grade contact in the examples; implementing IPC in the DSL is user work and not demonstrated. Partial axis-1 coverage by example.
- **[Visually great](../01-ceiling/00-definitions.md#visually-great):** `—`. DSL-level; no rendering. Visual output is the host application's responsibility.
- **[Real-time](../01-ceiling/00-definitions.md#real-time):** `✓`. Paper claim: "4.2× shorter than hand-engineered CUDA yet runs as fast" — the compiled kernels execute at CUDA-parity speed. Specific scene/tet-count/FPS for soft-body benchmarks — soft-deferred to Pass 2 (paper body details not fetched).
- **[Differentiable](../01-ceiling/00-definitions.md#differentiable):** `✓`. Full source-to-source AD via lightweight tape; gradients propagate through the compiled kernels. The foundational contribution of the framework.

## What `sim-soft` inherits or learns

DiffTaichi is the reference for **axis 4 (differentiable) as a DSL-level contribution** — the idea that a differentiable physics simulator should be written once in a high-level language and compiled to fast kernels with autograd for free. `sim-soft` occupies an architecturally similar position in Rust: the [`autograd/` module](../../110-crate/00-module-layout/07-autograd.md) + [Part 6 Ch 00](../../60-differentiability/00-what-autograd-needs.md)'s "own every line" commitment + wgpu kernels compiled from Rust forward code with a tape that records kernel calls. DiffTaichi serves as **conceptual precedent and design-study anchor**, not as a dependency or regression baseline.

Current status: the DiffTaichi-labeled examples repo has been stale since 2021; the DSL is now part of the actively-maintained Taichi language at `github.com/taichi-dev/taichi`. Cite DiffTaichi-the-paper (arXiv:1910.00935) for the architectural idea; cite Taichi-the-project for the shipping DSL today.

## Citation status

- Primary paper: Hu et al. 2020 ICLR, arXiv:1910.00935 — confirmed from arXiv landing page (title + authors + venue all verified).
- DSL status: confirmed DSL (source-to-source compiled), NOT a framework on top of PyTorch.
- Examples repo license — unable to verify from 2026-04-20 fetch (LICENSE file present but content not shown); soft-deferred to Pass 2.
- Per-scene benchmark details (tet counts, FPS for soft-body specifically) — soft-deferred to Pass 2 (paper body not fetched).
- Current practical usage: DiffTaichi framework absorbed into Taichi; examples repo effectively archived-by-absorption.
