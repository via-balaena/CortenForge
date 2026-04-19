# Differentiable sim papers

[Part 6 Ch 05 (differentiability through meshing — the open problem)](../../60-differentiability/05-diff-meshing.md) is the book's densest single chapter for anchor-linked citations. This leaf indexes the differentiable-meshing works that chapter names and the two named differentiable-XPBD implementations (DiffXPBD, NVIDIA Warp) cited from [Part 5 Ch 01](../../50-time-integration/01-projective.md). Pass 3 will add the broader differentiable-physics-framework literature (DiffTaichi, Genesis, JAX-MD) as anchors once prose references them.

## Liao, Donné & Geiger 2018 {#liao-2018}

*Deep Marching Cubes: Learning Explicit Surface Representations.* Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR) 2018, pp. 2916–2925. Authors: Yiyi Liao, Simon Donné, Andreas Geiger. [Open-access PDF (CVF)](https://openaccess.thecvf.com/content_cvpr_2018/papers/Liao_Deep_Marching_Cubes_CVPR_2018_paper.pdf).

Replaces the discrete topology choice in the marching-cubes algorithm with a categorical distribution: per-corner occupancy probabilities (sigmoid-activated) induce a distribution over the 15 rotational equivalence classes of the classical lookup table, and the mesh loss is computed as an expectation under that distribution. Training dynamics sharpen the occupancies toward binary values; during the soft phase the mesh has fractional connectivity across multiple topologies simultaneously. Cited inline from [Part 6 Ch 05](../../60-differentiability/05-diff-meshing.md) as the canonical soft-topology-MC reference; the book's assessment is that the approach is end-to-end differentiable for losses that tolerate a distribution over meshes (differentiable rendering, point-cloud Chamfer), but produces phantom vertices and fractional connectivity that an FEM solver cannot consume during the soft phase. **Common citation-correction note:** the "DiffMC" name refers to a separate later work (Wei et al. NeuManifold 2023, arXiv:2305.17134) that keeps topology discrete and differentiates only vertex positions — not to the soft-topology expectation-loss construction, which is Liao 2018.

## Chen & Zhang 2021 {#chen-zhang-2021}

*Neural Marching Cubes.* ACM Transactions on Graphics Vol. 40, No. 6, Article 251 (SIGGRAPH Asia 2021), December 2021. DOI [10.1145/3478513.3480518](https://doi.org/10.1145/3478513.3480518). arXiv [2106.11272](https://arxiv.org/abs/2106.11272). Authors: Zhiqin Chen, Hao Zhang.

Keeps the topology choice discrete but learns a replacement for Lorensen & Cline's fixed lookup table: the network emits per-corner binary sign predictions and continuous vertex-position offsets, and the binary signs index a learned lookup table with 37 unique topology cases (symmetry-reduced from the full space of $2^{15}$ corner sign patterns). Topology selection at inference is a deterministic lookup on the binary output; differentiability is bought entirely in the vertex-position offsets. Cited inline from [Part 6 Ch 05](../../60-differentiability/05-diff-meshing.md); the book's assessment is that the method produces a fully-specified mesh at every forward (no phantom vertices) but buys no gradient flow through topology changes — the binary sign prediction is itself discrete, so sweeping an SDF parameter across a sign flip reproduces the step discontinuity of [§00](../../60-differentiability/05-diff-meshing/00-why-not-mc.md).

## Shen et al. 2021 {#shen-2021}

*Deep Marching Tetrahedra: a Hybrid Representation for High-Resolution 3D Shape Synthesis.* Advances in Neural Information Processing Systems (NeurIPS) 34, 2021. arXiv [2111.04276](https://arxiv.org/abs/2111.04276). [NVIDIA Toronto AI Lab project page](https://research.nvidia.com/labs/toronto-ai/DMTet/). Authors: Tianchang Shen, Jun Gao, Kangxue Yin, Ming-Yu Liu, Sanja Fidler.

DMTet — deformable tet grid whose SDF values live on grid vertices; iso-surface extracted by a differentiable marching-tetrahedra variant on a six-tet partition of the cube, with three canonical iso-surface topologies per tet up to rotational symmetry. The tet grid is internal scaffolding; the output is a surface triangle mesh, not a volumetric tet mesh. Directly adopted by NVIDIA GET3D (Gao et al. 2022, arXiv [2209.11163](https://arxiv.org/abs/2209.11163), NeurIPS 2022 §3.1.1 "we utilize DMTet to extract a 3D surface mesh from the SDF") and Magic3D (Lin et al. 2023, arXiv [2211.10440](https://arxiv.org/abs/2211.10440), CVPR 2023 §4.2 "we extract a surface mesh from the SDF using a differentiable marching tetrahedra algorithm"). Cited inline from [Part 6 Ch 05](../../60-differentiability/05-diff-meshing.md) and referenced by [Part 12 Ch 07's open-problem Option A](../../120-roadmap/07-open-questions.md) as the basis for a future volumetric extension. Still a surface mesher, not volumetric — the extension to a full tet-mesh FEM-ready output is open research.

## Rakotosaona et al. 2021 {#rakotosaona-2021}

*Differentiable Surface Triangulation.* ACM Transactions on Graphics (SIGGRAPH Asia 2021). arXiv [2109.10695](https://arxiv.org/abs/2109.10695). Authors: Marie-Julie Rakotosaona, Noam Aigerman, Niloy J. Mitra, Maks Ovsjanikov, Paul Guerrero.

Formulates weighted-Delaunay surface triangulation as a soft relaxation: per-vertex weights plus a temperature parameter control the continuous probability that each face is included in the tessellation, and gradients flow smoothly through both weights and vertex positions. Cited inline from [Part 6 Ch 05](../../60-differentiability/05-diff-meshing.md) as the soft-relaxation Delaunay baseline; the book's assessment is that the weighted-Delaunay configuration is a global function of all vertex positions and weights, requiring full re-tessellation on every geometry update — an affordability problem at the tet counts `sim-soft` targets.

## Son et al. 2024 {#son-2024}

*DMesh: A Differentiable Representation for General Meshes.* Advances in Neural Information Processing Systems (NeurIPS) 2024. arXiv [2404.13445](https://arxiv.org/abs/2404.13445). Authors: Sanghyun Son, Matheus Gadelha, Yang Zhou, Zexiang Xu, Ming C. Lin, Yi Zhou.

Refines the Rakotosaona soft-relaxation direction using the geometric power-diagram dual: face inclusion is a sigmoid of the power-diagram predicate, which has closed-form derivatives, so the relaxation temperature plays a smaller role and the gradient signal is cleaner. Cited inline from [Part 6 Ch 05](../../60-differentiability/05-diff-meshing.md); the book's assessment is that the closed-form predicate cleans up the gradient but does not beat the global-retessellation scaling limit — production deployment in volumetric FEM at 30k+ tets has not been demonstrated.

## Hu et al. 2020 (fTetWild) {#hu-2020}

*Fast Tetrahedral Meshing in the Wild.* ACM Transactions on Graphics 39(4), Article 117 (2020). Authors: Yixin Hu, Teseo Schneider, Bolun Wang, Denis Zorin, Daniele Panozzo.

The robust tet-meshing-from-mesh pipeline with envelope-based validity guarantees — handles arbitrary input surfaces including self-intersecting and non-manifold meshes, with mesh-improvement passes that drive aspect ratio and dihedral angle into a usable band. Cited inline from [Part 7 Ch 01 — tetrahedralization strategies](../../70-sdf-pipeline/01-tet-strategies.md) as the default design-mode tet-generation pipeline in `sim-soft` (adapted to SDF input via iso-surface extraction); the SDF-pathology robustness is the property that makes it the right fit for `sim-soft`'s `SdfField`-driven design flow where the iso-surface of an arbitrary cf-design SDF can exhibit any surface pathology. The predecessor paper (Hu, Zhou, Gao, Jacobson, Zorin, Panozzo 2018 — "Tetrahedral Meshing in the Wild") is not anchored here because no inline citation currently references it; the `sim-soft` pipeline consumes the 2020 fast-variant.

## Stuyck & Chen 2023 (DiffXPBD) {#stuyck-2023}

*DiffXPBD: Differentiable Position-Based Simulation of Compliant Constraint Dynamics.* ACM Transactions on Graphics, 2023. DOI [10.1145/3606923](https://doi.org/10.1145/3606923). arXiv [2301.01396](https://arxiv.org/abs/2301.01396). Authors: Tuur Stuyck (Facebook Reality Labs), Hsiao-yu Chen.

Derives analytical gradients through the XPBD Lagrange-multiplier updates, making XPBD amenable to gradient-based optimization over its own compliance parameters ($\alpha_j$, damping $\beta_j$). Distinct from DiffTaichi — a separate system with different authors and scope. Cited inline from [Part 5 Ch 01 §02 convergence](../../50-time-integration/01-projective/02-convergence.md) as one of two named differentiable-XPBD implementations; the book's assessment is that the analytical gradients are well-defined for compliant-constraint parameter optimization but are gradients of the XPBD-objective surface, not of the true hyperelastic potential the solver is approximating.

## NVIDIA Warp {#warp}

NVIDIA's high-performance Python-JIT simulation library. Documentation: [github.com/NVIDIA/warp](https://github.com/NVIDIA/warp); developer overview: [developer.nvidia.com/warp-python](https://developer.nvidia.com/warp-python). No peer-reviewed paper for Warp itself as of this writing.

`warp.sim` ships an XPBD integrator (`IntegratorXPBD`) with autograd support. Cited inline from [Part 5 Ch 01 §02 convergence](../../50-time-integration/01-projective/02-convergence.md) as the second named differentiable-XPBD implementation. The framework's own discussion boards acknowledge practical gradient limitations — [discussion #146](https://github.com/NVIDIA/warp/discussions/146) notes joint-target gradients are not yet differentiable; [issue #759](https://github.com/NVIDIA/warp/issues/759) documents zero-gradient issues in cloth examples. The book's assessment parallels the DiffXPBD assessment: gradients are of the XPBD-objective surface, not of the true hyperelastic potential.

## Pass 3 anchors (not yet inline-cited)

Reserved slots the Pass 3 bibliography will populate: DiffTaichi (Hu et al. 2020, *DiffTaichi: Differentiable Programming for Physical Simulation*, ICLR 2020 — [arXiv 1910.00935](https://arxiv.org/abs/1910.00935)), Genesis (Tsinghua), and the 2018 predecessor to fTetWild if later chapters reference it. None is anchored here yet because no inline citation currently references them by anchor.
