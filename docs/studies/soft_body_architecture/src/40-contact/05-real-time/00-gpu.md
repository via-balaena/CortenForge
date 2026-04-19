# GPU parallelization

IPC's hot path is embarrassingly parallel in its per-pair work. Broadphase, narrow-phase distance queries, per-pair barrier evaluation, Jacobian and Hessian scatter, and CCD inclusion tests are all per-primitive or per-pair operations with bounded per-pair work and no inter-pair data dependencies at assembly time — the canonical scatter-and-add shape GPUs are built for. [Li et al. 2020](../../appendices/00-references/00-ipc.md#li-2020)'s reference implementation is Eigen-on-CPU and offline in practice; the [IPC Toolkit](../../appendices/00-references/00-ipc.md#ipc-toolkit) that followed preserves that CPU-first design across most components. The 2022–2024 follow-up literature closes the gap from offline CPU to real-time GPU paper by paper.

## What parallelizes

Five stages of the IPC hot path are per-primitive or per-pair workloads:

- **Broadphase.** [LBVH Morton-sort + tree build + AABB propagation](../03-self-contact/00-bvh.md) is a sequence of data-parallel primitives (sort, scan, reduction). The double-tree traversal that emits candidate pairs runs independently per subtree-pair; each subtree-pair's descent is a bounded-depth task that dispatches cleanly onto a GPU compute grid.
- **Narrow-phase.** Per-pair exact distance queries are independent of each other; each pair computes its own distance and tests the $d < \hat d_k$ condition in isolation, with no read-before-write dependency on neighbouring pairs.
- **Barrier evaluation.** The per-pair barrier value $b(d_k, \hat d_k)$, first derivative $b'(d_k)$, and second derivative $b''(d_k)$ are [closed-form expressions](../01-ipc-internals/00-barrier.md) in the pair's own gap. No inter-pair coupling; the evaluation is a per-pair kernel.
- **Jacobian / Hessian scatter.** Each active pair contributes a sparse block to the global gradient and Hessian. The scatter uses atomic-add into the per-vertex row; no ordering dependency between pairs.
- **CCD inclusion tests.** Per-pair interval-Newton and inclusion tests from [Ch 01 §02](../01-ipc-internals/02-ccd.md) are independent. The IPC Toolkit's `SweepAndTiniestQueue` GPU variant from [Belgrod et al.](../../appendices/00-references/00-ipc.md#scalable-ccd) is designed for this parallel structure — sweep-based broad-phase on coordinate axes composed with interval-based narrow-phase.

Across these five stages, per-pair work is bounded and structurally identical across pairs — the same barrier function, the same distance computation, the same scatter pattern. Warp-level load-imbalance is low, which is what makes the measured GPU speedups hold in practice rather than only in theoretical FLOP counts.

## What stays CPU or scalar

Three control-flow components of the IPC outer loop remain sequential or scalar by design:

- **Newton outer loop.** Each iterate depends on the previous iterate's converged position. The inner work — barrier assembly, linear solve, trial-step evaluation — ports to GPU; the iterate-to-iterate sequencing is CPU control flow.
- **Line-search decision logic.** The [filtered line search from Ch 01 §02](../01-ipc-internals/02-ccd.md) caps the step by the CCD-safe $\alpha_\text{CCD}$ and then selects a scalar step size within $[0, \alpha_\text{CCD}]$. The *evaluation* of a trial step (barrier energy, elastic energy, CCD predicate) parallelizes; the *decision* to accept or backtrack is a scalar control transition on CPU.
- **Adaptive-$\kappa$ growth rule.** The [growth trigger from Ch 01 §01](../01-ipc-internals/01-adaptive-kappa.md) aggregates a scene-wide gap statistic and makes a scalar doubling decision. The aggregate reduction is GPU-parallel; the decision is CPU-sequential.

Each of these is $O(1)$ per iterate in the scalar sense — not where the rate-limiting work lives, so leaving them on CPU is an architectural simplification rather than a performance concession.

## The follow-up literature

Three lines of work demonstrate GPU-IPC pipelines in the 2022–2024 window.

**Lan et al. 2022 — *Penetration-free Projective Dynamics on the GPU*** ([ACM ToG, SIGGRAPH 2022](../../appendices/00-references/00-ipc.md#lan-2022)). Hybrid projective-dynamics + IPC on GPU. Redesigns the A-Jacobi solver for GPU memory access patterns, reworks CCD with a minimum-gradient Newton pass for parallel-hardware efficiency, and ships an end-to-end GPU pipeline — projective-dynamics elastic update composed with IPC barrier filtering. Demonstrates that GPU-IPC is feasible at interactive rates for the PD subset of elastic formulations; the design choices are PD-specific but the broadphase + CCD machinery is reusable for full-Newton IPC.

**Huang et al. 2024 — *GIPC: Fast and Stable Gauss-Newton Optimization of IPC Barrier Energy*** ([ACM ToG, SIGGRAPH 2024](../../appendices/00-references/00-ipc.md#gipc-2024)). First fully GPU-optimized general IPC framework. Derives analytic eigensystems from simplicial geometric contact measures so the per-pair barrier Hessian is a closed-form eigendecomposition rather than a numerical eigensolve — removing the one per-pair operation that would have serialized the assembly kernel. Reports benchmarked GPU throughput on general elastodynamics at scene sizes beyond the 2020 CPU-IPC interactive range.

**Huang et al. 2025 — *StiffGIPC: Advancing GPU IPC for Stiff Affine-Deformable Simulation*** ([ACM ToG, SIGGRAPH 2025](../../appendices/00-references/00-ipc.md#stiffgipc-2025)). Extends GIPC with a connectivity-improved multilevel additive Schwarz preconditioner on GPU and a $C^2$-continuous cubic barrier variant tuned for stiff materials. Reports roughly $10\times$ over the GIPC baseline in the stiff regime. Anchors the current published ceiling for GPU-IPC throughput on general elastodynamics.

Three papers, one trend: GPU-IPC is demonstrated to work, each follow-up improves per-iterate cost or solver conditioning, and the aggregate speedup range over CPU-IPC reported across this literature spans roughly $10\times$ to $100\times$ depending on scene and baseline. Lan 2022 composes IPC with projective dynamics rather than Newton; GIPC and StiffGIPC ship a Newton-IPC inner loop — the three are not a single common algorithm, but they share the GPU-IPC pipeline shape (per-pair parallel barrier work, atomic-add scatter, GPU linear solve). `sim-soft`'s Phase E commitment is the GIPC/StiffGIPC Newton-IPC branch; Lan 2022 is cited as the progenitor that demonstrated the pipeline shape was viable.

## What `sim-soft` commits to from the literature

- **`sim-soft` ports the Newton-IPC inner loop from GIPC/StiffGIPC, not the projective-dynamics variant from Lan 2022.** The GIPC analytic-eigensystem trick and the StiffGIPC preconditioner are additive refinements on top of the Newton-IPC inner loop; they compose cleanly with `sim-soft`'s [Part 5 Ch 00 backward-Euler Newton solve](../../50-time-integration/00-backward-euler.md). Lan 2022's projective-dynamics variant is cited as the progenitor, not the port target.
- **LBVH + per-primitive adaptive $\hat d$ are the broadphase and pair-generation choices.** The LBVH commitment comes from [Ch 03 §00 BVH](../03-self-contact/00-bvh.md); per-primitive adaptive $\hat d$ comes from [§01 adaptive barrier width](01-barrier-width.md). [IPC Toolkit's `SweepAndTiniestQueue`](../03-self-contact/00-bvh.md) traces to [Belgrod et al.](../../appendices/00-references/00-ipc.md#scalable-ccd) and is the CCD port reference.
- **Speedup targets are planning, not measurement.** The $10$–$100\times$ range is the published GPU-IPC literature range; [Part 11 Ch 03 Phase E](../../110-crate/03-build-order.md)'s 30 FPS / 5 Hz numbers are derived by scaling this range against [Warp's neo-Hookean benchmarks](../../00-context/02-sota/01-warp.md). `sim-soft`'s actual per-scene speedup is measured against the Phase D CPU oracle during Phase E, not derived from the paper range.

## Cost scaling: stack depth, interface area, Newton-iteration count

[Ch 04 §01 no-penetration](../04-multi-layer/01-no-penetration.md) referenced the GPU lever for stack-depth amortization, and [Part 3 Ch 03 §01 sliding](../../30-discretization/03-interfaces/01-sliding.md) referenced it for interface-area growth. Both enter the GPU-IPC rate model through pair count:

- **Stack depth** scales the number of inter-mesh primitive pairs roughly linearly with the number of layers in contact. Per-pair work is bounded; pair-count growth adds parallel work, not serial work.
- **Interface area** scales the persistent-pair count linearly with interface size. Same amortization — each persistent pair is its own independent per-pair workload.

Neither growth mode imposes serial dependencies on the GPU hot path; both are absorbed at the per-pair-throughput level. The second-order cost channel — stiffness-matrix fill growth in the linear solve — is owned by [Part 8 Ch 02 sparse solvers](../../80-gpu/02-sparse-solvers.md); preconditioned CG with block-Jacobi or AMG keeps the fill-growth cost in a single-digit multiplier rather than quadratic.

[Ch 04 §02 thin-material](../04-multi-layer/02-thin-material.md)'s $2$–$5\times$ Newton-iteration-count multiplier is *not* absorbed by the GPU lever. Thin-material scenes take more Newton iterations per timestep; each iteration runs the same GPU-parallel workload at the same per-iteration cost, so wall-clock cost is multiplied by the iteration factor. The GPU lever scales the per-iteration budget; it does not reduce iteration count. That is why [Ch 04 §02](../04-multi-layer/02-thin-material.md) named the multiplier as a structural overhead — a cost regime, not a correctness regime, but also not a regime the GPU lever by itself shrinks.

## What this sub-leaf commits the book to

- **Hot-path GPU ports land at Phase E.** Broadphase (LBVH), narrow-phase distance, barrier evaluation, Jacobian/Hessian scatter, and CCD all target wgpu compute kernels. The same algorithms are used on CPU and GPU with different backends — no per-backend algorithmic divergence.
- **Newton outer loop, line-search decision, adaptive-$\kappa$ growth stay CPU / scalar.** Each is $O(1)$ per iterate; not the rate-limiting stage. CPU control over these pieces keeps the Phase D regression oracle simple.
- **Linear solve is [Part 8 Ch 02](../../80-gpu/02-sparse-solvers.md) scope.** This sub-leaf commits the broadphase-through-Jacobian path; the linear solve's GPU port and preconditioner strategy are owned by Part 8.
- **$10$–$100\times$ is a published-literature estimate, not a measurement.** The paper range grounds [Part 11 Ch 03 Phase E](../../110-crate/03-build-order.md)'s planning targets; `sim-soft`'s actual speedup lands with Phase E benchmarks.
- **GIPC + StiffGIPC + Lan 2022 + Belgrod et al. are the follow-up-literature ancestry.** The four citations in this sub-leaf are the load-bearing sources; `sim-soft`'s GPU-IPC design is not an independent derivation, it is a port of these four bodies of work composed with the [IPC Toolkit](../../appendices/00-references/00-ipc.md#ipc-toolkit) reference implementation.
