# SOTA tool references

This leaf indexes the nine tools surveyed in [Part 0 Ch 02](../../00-context/02-sota.md). Each entry anchors the tool's repository (or canonical documentation link), its license, and the primary paper or presentation reference where one exists. Where a tool's underlying paper is indexed elsewhere in this appendix (e.g., [Li et al. 2020 for IPC](00-ipc.md#li-2020)), the entry cross-links rather than duplicating. All tools accessed 2026-04-20.

Pass 1 populates the entries with the information surfaced by the 2026-04-20 source-fetch verification pass. Pass 3 will tighten any remaining soft-defer items (specific benchmark URLs, DOI numbers for older papers, release-date cross-checks) and add adjacent references that post-Phase-I follow-up work may reveal.

## PolyFEM {#polyfem}

*PolyFEM.* Open-source C++ finite element framework. Repo: [`github.com/polyfem/polyfem`](https://github.com/polyfem/polyfem). License: MIT. Primary paper: Schneider, Dumas, Hu, Bouaziz, Kaufmann, Jiang, Gao, Zhou, Jacobson, Panozzo 2019 "Decoupling Simulation Accuracy from Mesh Quality," SIGGRAPH Asia 2019 (DOI soft-deferred to Pass 2). Cited inline from [Part 0 Ch 02 §00 PolyFEM](../../00-context/02-sota/00-polyfem.md) as the reference implementation for axis-1 physical correctness — hyperelastic FEM + IPC + implicit integration composed and shipped.

## NVIDIA Warp {#warp}

*NVIDIA Warp.* Python-first kernel DSL compiling to CUDA with autograd support. Repo: [`github.com/NVIDIA/warp`](https://github.com/NVIDIA/warp). License: Apache-2.0. v1.12.1 released 2026-04-06. No single canonical peer-reviewed paper; primary reference is Macklin 2022 GTC presentation (URL soft-deferred to Pass 2) plus NVIDIA tech reports. `warp.fem` implements [Smith, Goes, Kim 2018 stable neo-Hookean](01-hyperelastic.md) in its FEM examples. Cited inline from [Part 0 Ch 02 §01 Warp](../../00-context/02-sota/01-warp.md), [Part 1 Ch 03 thesis](../../10-physical/03-thesis.md), and [Part 11 Ch 03 Phase E](../../110-crate/03-build-order.md#the-committed-order) as the reference implementation for axes 3 + 4 (real-time + differentiable) and as the source of `sim-soft`'s target-envelope derivation.

## Genesis {#genesis}

*Genesis.* Multi-physics embodied-AI simulation platform (rigid + soft-body FEM + MPM + SPH + cloth + fluids + articulated). Repo: [`github.com/Genesis-Embodied-AI/Genesis`](https://github.com/Genesis-Embodied-AI/Genesis). License: Apache-2.0. v0.4.6 released 2026-04-11. Attribution: "Genesis Authors" (collective), with individual credit including Zhou Xian, Yiling Qiao, and collaborators at Tsinghua, CMU, and partner institutions. No single canonical paper for Genesis-the-platform (soft-deferred to Pass 2). Adjacent prior work from the group indexed separately: Qiao et al. 2021 NeurIPS "Differentiable simulation of soft multi-body systems" (see [Differentiable sim papers](03-diff-sim.md)); Xian et al. 2023 "FluidLab" (arXiv:2303.02346); Wang et al. 2024 ICLR "Thin-Shell Object Manipulations." Cited inline from [Part 0 Ch 02 §02 Genesis](../../00-context/02-sota/02-genesis.md) as the multi-physics-platform reference at the platform level.

## IPC Toolkit {#ipc-toolkit-sota}

*IPC Toolkit.* Reference C++/Eigen implementation of Incremental Potential Contact with Python bindings via pybind11. Repo: [`github.com/ipc-sim/ipc-toolkit`](https://github.com/ipc-sim/ipc-toolkit). License: MIT. v1.5.0 released 2026-02-06. Primary paper: [Li et al. 2020](00-ipc.md#li-2020) (cross-linked to IPC papers leaf). Cited inline from [Part 0 Ch 02 §03 IPC Toolkit](../../00-context/02-sota/03-ipc-toolkit.md) as the algorithmic reference for axis-1's IPC-grade-contact component.

## DiffTaichi {#difftaichi}

*DiffTaichi.* Differentiable programming DSL — source-to-source compilation from Python-embedded DSL to CUDA with lightweight-tape autograd. Examples repo: [`github.com/taichi-dev/difftaichi`](https://github.com/taichi-dev/difftaichi) (stale since 2021; framework absorbed into [Taichi core](https://github.com/taichi-dev/taichi)). Primary paper: Hu, Anderson, Li, Sun, Carr, Ragan-Kelley, Durand 2020 "DiffTaichi: Differentiable Programming for Physical Simulation," ICLR 2020 ([arXiv:1910.00935](https://arxiv.org/abs/1910.00935)). Cited inline from [Part 0 Ch 02 §04 DiffTaichi](../../00-context/02-sota/04-difftaichi.md) as the DSL-level reference for axis-4 differentiability.

## NVIDIA Flex {#flex}

*NVIDIA Flex.* Unified particle-based physics simulator (PBD/XPBD lineage). Repo: [`github.com/NVIDIAGameWorks/FleX`](https://github.com/NVIDIAGameWorks/FleX). License: LICENSE.txt at repo root, specific name soft-deferred to Pass 2 (likely NVIDIA proprietary / source-available per GameWorks pattern). Last release: 1.1.0 on 2017-04-10 — superseded-in-practice by [Warp](#warp) for NVIDIA's current GPU-physics roadmap (repo not formally archived, so "officially deprecated" is not claimed). Primary paper: Macklin, Müller, Chentanez, Kim 2014 "Unified Particle Physics for Real-Time Applications," SIGGRAPH 2014 (DOI soft-deferred to Pass 2). Distributed as Unity and Unreal Engine 4 GameWorks integrations. Cited inline from [Part 0 Ch 02 §05 NVIDIA Flex](../../00-context/02-sota/05-nvidia-flex.md) as the production games-branch negative example on axes 1 + 4.

## Houdini FEM Solver {#houdini-fem}

*Houdini FEM Solver* (Solid Object DOP / Finite Element Solver). Commercial closed-source — part of SideFX Houdini. Reference: [SideFX Houdini documentation](https://www.sidefx.com/docs/houdini/). Related: [Vellum Solver](https://www.sidefx.com/docs/houdini/nodes/sop/vellumsolver.html) — XPBD-based constraint solver used for cloth, hair, and grains in Houdini's heterogeneous production pipeline. No single primary research paper — the FEM Solver is in-house SideFX engineering. Cited inline from [Part 0 Ch 02 §06 Houdini FEM Solver](../../00-context/02-sota/06-houdini-jelly.md) as the reference for axis-2 visual quality at the production ceiling.

## FEBio {#febio}

*FEBio.* Biomechanics-focused nonlinear FEM solver. Repo: [`github.com/febiosoftware/FEBio`](https://github.com/febiosoftware/FEBio). License: MIT. v4.12 released 2026-02-25. Primary paper: Maas, Ellis, Ateshian, Weiss 2012 "FEBio: Finite Elements for Biomechanics," *Journal of Biomechanical Engineering* 134(1):011005 (venue per common citation; DOI and exact page numbers soft-deferred to Pass 2). Extensive hyperelastic catalog (Arruda-Boyce, Mooney-Rivlin, Neo-Hookean, Ogden, Holzapfel-Gasser-Ogden, Gent, Holmes-Mow, Veronda-Westmann, Porous Neo-Hookean, transversely isotropic, orthotropic) plus viscoelastic, biphasic, biphasic-solute, and triphasic/multiphasic formulations. Cited inline from [Part 0 Ch 02 §07 FEBio](../../00-context/02-sota/07-febio.md) as the biomechanics reference for axis-1's material-model breadth.

## MuJoCo flex {#mujoco-flex}

*MuJoCo flex.* Deformable-object extension to DeepMind MuJoCo (simplicial-complex elements of dimensions 1, 2, or 3 for stretchable lines, triangles, or tetrahedra). Source: MuJoCo 3.0+ release notes. Repo: [`github.com/google-deepmind/mujoco`](https://github.com/google-deepmind/mujoco). License: Apache-2.0 (MuJoCo core; specific SPDX string soft-deferred to Pass 2). Flex introduced in **MuJoCo 3.0.0, released 2023-10-18**. Release-note qualifier: "This feature is still under development and subject to change." Solver per [Part 12 Ch 01 Track 1B](../../120-roadmap/01-track-1b.md) authoritative framing: Newton + constraint-projection + penalty-contact. Cited inline from [Part 0 Ch 02 §08 MuJoCo flex](../../00-context/02-sota/08-mujoco-flex.md) as the in-engine regression baseline + Track 1B platform coverage baseline.

## Pass 3 anchors (not yet inline-cited)

Reserved slot for follow-up references a Pass 3 bibliography expansion may populate: Macklin 2022 GTC presentation URL or slide deck; Macklin et al. 2014 SIGGRAPH DOI; Schneider et al. 2019 SIGGRAPH Asia DOI; Maas et al. 2012 *J. Biomech. Eng.* DOI and exact page numbers; Flex LICENSE.txt content for specific license-name confirmation; possible MJX-flex differentiability status on MuJoCo updates post-3.0.0. Each of these is a soft-defer flagged in a specific Ch 02 sub-leaf's citation-status section.
