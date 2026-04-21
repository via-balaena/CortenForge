# Genesis — Tsinghua's differentiable sim

Repo: [`github.com/Genesis-Embodied-AI/Genesis`](https://github.com/Genesis-Embodied-AI/Genesis) · Apache-2.0 · v0.4.6 (2026-04-11) · Attribution: "Genesis Authors" (collective), with individual credit including Zhou Xian, Yiling Qiao, and collaborators at Tsinghua, CMU, and partner institutions · Accessed: 2026-04-20

Genesis is a multi-physics embodied-AI simulation platform covering rigid body, soft-body (FEM + MPM + SPH), cloth, fluids, and articulated systems within one framework. Differentiability is platform-level rather than uniformly shipped: MPM and "Tool" solvers are currently differentiable; rigid and articulated solvers are roadmapped for differentiability but not yet landed. Python-first; ships a built-in photorealistic renderer alongside the simulator.

- **[Physically correct](../01-ceiling/00-definitions.md#physically-correct):** `partial`. FEM, MPM, and SPH modules ship; hyperelastic constitutive-model coverage, contact formulation, and implicit-vs-explicit integration scheme for the FEM soft-body path are not fully verified from fetched sources. Soft-defer below for per-component status.
- **[Visually great](../01-ceiling/00-definitions.md#visually-great):** `partial`. Genesis ships a photorealistic ray-tracing renderer; the degree of physics-render fusion (per-vertex physics attributes feeding shader directly vs. post-hoc rendering of simulator outputs) is not verified from the fetched README. Soft-defer to Pass 2.
- **[Real-time](../01-ceiling/00-definitions.md#real-time):** `partial`. Interactive-rate performance is claimed in the repo README; specific benchmark numbers (tet counts, FPS on consumer GPUs) not fetched in the 2026-04-20 pass. Soft-defer to Pass 2 for a verified benchmark citation.
- **[Differentiable](../01-ceiling/00-definitions.md#differentiable):** `partial`. MPM and Tool solvers currently differentiable per README; rigid, articulated, and other solvers roadmapped for differentiability. Partial axis-4 coverage — a subset of solvers support it today.

## What `sim-soft` inherits or learns

Genesis is the reference for **multi-physics coverage at the platform level**: the ambition to unify multiple simulation paradigms (FEM + MPM + particle + cloth + articulated) under one simulator with differentiability intended across all of them is structurally close to `sim-soft`'s integration bet. Where `sim-soft` narrows: a focus on volumetric hyperelastic FEM + [IPC-grade contact](../../40-contact/00-why-ipc.md) as the core (not MPM + SPH + cloth); a Rust + [wgpu](../../80-gpu/00-wgpu-layout.md) substrate (not Python + CUDA); and the commitment that differentiability holds uniformly across the integrated solver rather than per-subsolver. Genesis-the-platform is a useful mirror — what a platform-scale multi-physics simulator looks like with Python-first ergonomics and partial differentiability — rather than a direct dependency or regression baseline.

## Citation status

- Repo + license + release cadence (v0.4.6 on 2026-04-11) confirmed from README.
- Primary paper: no single canonical paper for Genesis-the-platform. Attribution is collective ("Genesis Authors"). Adjacent prior work from the same research group: Qiao et al. NeurIPS 2021 "Differentiable simulation of soft multi-body systems"; Xian et al. 2023 "FluidLab" (arXiv:2303.02346); Wang et al. ICLR 2024 "Thin-Shell Object Manipulations." Canonical citation for Genesis-the-platform — soft-deferred to Pass 2.
- FEM constitutive models, contact method, implicit integration scheme — soft-deferred to Pass 2 (not in fetched README content).
- Physics-render fusion vs post-hoc render integration — soft-deferred to Pass 2.
- Real-time benchmark numbers — soft-deferred to Pass 2.
