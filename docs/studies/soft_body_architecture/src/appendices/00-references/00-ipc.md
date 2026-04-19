# IPC papers

Incremental Potential Contact (IPC) is the contact formulation [Part 4](../../40-contact/00-why-ipc.md) commits to. This leaf indexes the 2020 source paper and the follow-up work that makes IPC tractable at real-time rates.

Pass 1 populates only anchors referenced inline by the book. Pass 3 will expand each entry to full bibliographic form (venue, DOI, extracted-result summary) and add adjacent references the book does not currently inline-cite.

## Li et al. 2020 {#li-2020}

*Incremental Potential Contact: Intersection- and Inversion-free, Large-Deformation Dynamics.* ACM Transactions on Graphics (SIGGRAPH 2020).

The source paper for IPC. Introduces the smooth barrier $b(d) = -(d - \hat d)^2 \ln(d / \hat d)$ for gap $d$ less than tolerance $\hat d$, the adaptive barrier-stiffness schedule, the filtered line search that preserves the intersection-free invariant across Newton iterations, and the smoothed Coulomb friction term. Cited inline from [Part 6 Ch 01 (custom VJPs)](../../60-differentiability/01-custom-vjps.md) for the closed-form first and second derivatives $b'(d), b''(d)$ that the hand-stabilized barrier VJP evaluates directly (following the same reference-implementation discipline the [IPC Toolkit](#ipc-toolkit) ships). Also referenced in prose from [Part 4 Ch 00](../../40-contact/00-why-ipc.md), [Part 4 Ch 05](../../40-contact/05-real-time.md), and [Part 1 Ch 03 (thesis)](../../10-physical/03-thesis.md) as "the 2020 offline paper" the real-time commitment extends.

## IPC Toolkit {#ipc-toolkit}

*IPC Toolkit* — open-source C++ library (with Python bindings) implementing the Incremental Potential Contact formulation. Maintained by the IPC research community (Ferguson and collaborators) and released on GitHub as `ipc-sim/ipc-toolkit`. Ships the barrier function, adaptive-$\kappa$ schedule, CCD-filtered line search, and primitive-pair proximity detection as reusable components. Serves as the reference implementation of Li et al. 2020 and subsequent IPC follow-ups; tracks the literature across releases (the original 2020 conservative step-capping was replaced by tighter inclusion-based CCD in later releases). Ships six broadphase variants (BruteForce, HashGrid, SpatialHash, LBVH, SweepAndPrune, SweepAndTiniestQueue) behind a common `BroadPhase` interface and a `SmoothCollisions` variant exposing per-primitive adaptive $\hat d$ via `vert/edge/face_adaptive_dhat` `Eigen::VectorXd` arrays (computed as `SmoothContactParameters::adaptive_dhat_ratio() · sqrt(contact_distance)` in `src/ipc/smooth_contact/smooth_collisions.{hpp,cpp}`). Cited inline from [Part 4 Ch 01 adaptive-kappa](../../40-contact/01-ipc-internals/01-adaptive-kappa.md) for the `initial_barrier_stiffness` helper whose computation `sim-soft` will port, from [Part 4 Ch 01 CCD](../../40-contact/01-ipc-internals/02-ccd.md) for the CCD-algorithm-as-porting-target framing, from [Part 4 Ch 03 §00 BVH](../../40-contact/03-self-contact/00-bvh.md) for the LBVH variant's `src/ipc/broad_phase/lbvh.{cpp,hpp}` reference, from [Part 4 Ch 04 §02 thin-material](../../40-contact/04-multi-layer/02-thin-material.md) and [Part 4 Ch 05 §01 adaptive barrier width](../../40-contact/05-real-time/01-barrier-width.md) for the `SmoothCollisions` adaptive-$\hat d$ port, and from [Part 4 Ch 05 §00 GPU](../../40-contact/05-real-time/00-gpu.md) for the `SweepAndTiniestQueue` GPU CCD variant's role.

## Belgrod et al. — Scalable CCD {#scalable-ccd}

*Time of Impact Dataset for Continuous Collision Detection and a Scalable Conservative Algorithm.* arXiv preprint. David Belgrod, Bolun Wang, Zachary Ferguson, Xin Zhao, Marco Attene, Daniele Panozzo, Teseo Schneider. [arXiv:2112.06300](https://arxiv.org/abs/2112.06300) (submitted December 2021; revised through 2025).

Introduces the SweepAndTiniestQueue GPU CCD algorithm: sweep-based broad-phase on coordinate axes composed with interval-based narrow-phase on CUDA, designed for parallel-hardware efficiency and proven correct under floating-point arithmetic. Benchmarked against more than a dozen prior CCD methods with analytic ground truth. Integrated into [IPC Toolkit](#ipc-toolkit) as the GPU CCD broadphase option (the `SweepAndTiniestQueue` variant referenced from [Part 4 Ch 03 §00 BVH](../../40-contact/03-self-contact/00-bvh.md) traces to this paper, though the Ch 03 sub-leaf does not link this anchor directly). Remains an arXiv preprint as of the latest revision — no major venue proceeding. Cited inline from [Part 4 Ch 05 §00 GPU](../../40-contact/05-real-time/00-gpu.md) as the CCD port reference.

## Lan et al. 2022 — Penetration-free Projective Dynamics on the GPU {#lan-2022}

*Penetration-free Projective Dynamics on the GPU.* ACM Transactions on Graphics (SIGGRAPH 2022). Lei Lan, Guanqun Ma, Yin Yang, Changxi Zheng, Minchen Li, Chenfanfu Jiang. DOI [10.1145/3528223.3530069](https://doi.org/10.1145/3528223.3530069).

Hybrid projective-dynamics + IPC on GPU. Redesigns the A-Jacobi solver for GPU memory access patterns, reworks CCD with a minimum-gradient Newton pass tuned for parallel-hardware efficiency, and ships an end-to-end GPU pipeline composing projective-dynamics elastic update with IPC barrier filtering. Demonstrates feasibility of GPU-IPC at interactive rates for the PD subset of elastic formulations; the broadphase and CCD machinery transfer to full-Newton IPC. Cited inline from [Part 4 Ch 05 §00 GPU](../../40-contact/05-real-time/00-gpu.md) as the 2022 progenitor of GPU-IPC pipelines.

## Huang et al. 2024 — GIPC {#gipc-2024}

*GIPC: Fast and Stable Gauss-Newton Optimization of IPC Barrier Energy.* ACM Transactions on Graphics (SIGGRAPH 2024). Kemeng Huang, Floyd M. Chitalu, Huancheng Lin, Taku Komura. [arXiv:2308.09400](https://arxiv.org/abs/2308.09400).

First fully GPU-optimized general IPC framework. Derives analytic eigensystems from simplicial geometric contact measures so the per-pair barrier Hessian is a closed-form eigendecomposition rather than a numerical eigensolve — the one operation that would otherwise serialize per-pair GPU assembly becomes a per-pair closed-form computation. Benchmarks general elastodynamics at scene sizes where 2020-era CPU IPC was offline-only. Cited inline from [Part 4 Ch 05 §00 GPU](../../40-contact/05-real-time/00-gpu.md) as the primary 2024 GPU-IPC reference, and from [Part 6 Ch 01 §02 contact-barrier VJP](../../60-differentiability/01-custom-vjps/02-contact-barrier.md) plus [Part 6 Ch 01 spine](../../60-differentiability/01-custom-vjps.md) for the Hessian-ill-conditioning-near-$d\to 0^+$ documentation motivating `sim-soft`'s Phase-E-GPU-port option.

## Huang et al. 2025 — StiffGIPC {#stiffgipc-2025}

*StiffGIPC: Advancing GPU IPC for Stiff Affine-Deformable Simulation.* ACM Transactions on Graphics (SIGGRAPH 2025). Kemeng Huang, Xinyu Lu, Huancheng Lin, Taku Komura, Minchen Li. [arXiv:2411.06224](https://arxiv.org/abs/2411.06224).

Extends [GIPC](#gipc-2024) with a connectivity-improved multilevel additive Schwarz preconditioner on GPU and a $C^2$-continuous cubic barrier variant tuned for stiff materials. Reports roughly $10\times$ speedup over the GIPC baseline in the stiff-material regime. Anchors the current published ceiling for GPU-IPC throughput on general elastodynamics. Cited inline from [Part 4 Ch 05 §00 GPU](../../40-contact/05-real-time/00-gpu.md) as the 2025 follow-up.

## Pass 3 anchors (not yet inline-cited)

Reserved slot for works a Pass 3 bibliography expansion may populate: IPC Toolkit SmoothCollisions origin-paper (if one exists beyond the `sim-soft`-native practitioner-heuristic attribution in [Part 4 Ch 04 §02](../../40-contact/04-multi-layer/02-thin-material.md) and [Part 4 Ch 05 §01](../../40-contact/05-real-time/01-barrier-width.md)), and cross-cutting 2024+ GPU-IPC follow-ups beyond GIPC/StiffGIPC. Adding them is Pass 3 judgment.
