# Differentiable sim papers

[Part 6 Ch 05 (differentiability through meshing — the open problem)](../../60-differentiability/05-diff-meshing.md) is the book's densest single chapter for anchor-linked citations. This leaf indexes the four differentiable-meshing works that chapter names; Pass 3 will add the broader differentiable-physics-framework literature (DiffTaichi, Warp, Genesis, JAX-MD) as anchors once prose references them.

## Wei et al. 2023 {#wei-2023}

DiffMC — differentiable marching cubes. Pass 3 will fill in the full title and venue after the source fetch.

Replaces the discrete topology choice in the marching-cubes algorithm with a soft classification: each cell's topology is a convex combination of the 15 canonical patterns weighted by a softmax over the SDF corner values. Vertices exist everywhere with variable mass; the topology emerges as the softmax sharpens. Cited inline from [Part 6 Ch 05](../../60-differentiability/05-diff-meshing.md) as the canonical end-to-end-differentiable-meshing reference; the book's assessment is that the approach works for inverse-rendering but produces phantom vertices and spurious stiffness contributions unacceptable for a production FEM.

## Chen & Zhang 2021 {#chen-zhang-2021}

*Neural Marching Cubes.* ACM Transactions on Graphics (SIGGRAPH Asia 2021).

Trains a network to place vertices and choose topologies that a downstream loss likes, amortizing the discrete choice into network weights. The topology choice happens inside the network forward pass as a Gumbel-softmax or a hard argmax with straight-through estimator. Cited inline from [Part 6 Ch 05](../../60-differentiability/05-diff-meshing.md); the book's assessment is that this works for reconstruction tasks with a known ground-truth mesh but does not transfer to FEM, where "the right mesh" is defined by downstream solver convergence rather than by reconstruction error.

## Shen et al. 2021 {#shen-2021}

*Deep Marching Tetrahedra — a Hybrid Representation for High-Resolution 3D Shape Synthesis.* NeurIPS 2021.

DMTet — deformable tet grid whose SDF values live on grid vertices; iso-surface extracted by a differentiable marching-tetrahedra variant on a six-tet partition of the cube with a small fixed topology set. Adopted by NVIDIA GET3D and Magic3D as the most production-ready differentiable-meshing approach. Cited inline from [Part 6 Ch 05](../../60-differentiability/05-diff-meshing.md) and referenced by [Part 12 Ch 07's open-problem Option A](../../120-roadmap/07-open-questions.md) as the basis for a future volumetric extension. Still a surface mesher, not volumetric — the extension is open research.

## Wang et al. 2023 {#wang-2023}

Differentiable Delaunay triangulation. Pass 3 will fill in the full title and venue after the source fetch.

Formulates Delaunay triangulation as the limit of a regularized linear-programming problem; gradients flow through the LP's dual variables. Mathematically clean, scales poorly — the LP is re-solved on every geometry update, and constants are prohibitive at the tet counts `sim-soft` targets. Cited inline from [Part 6 Ch 05](../../60-differentiability/05-diff-meshing.md) as the third published angle of attack on differentiable meshing; the book's assessment is that production deployment in FEM has not been demonstrated.

## Hu et al. 2020 (fTetWild) {#hu-2020}

*Fast Tetrahedral Meshing in the Wild.* ACM Transactions on Graphics 39(4), Article 117 (2020). Authors: Yixin Hu, Teseo Schneider, Bolun Wang, Denis Zorin, Daniele Panozzo.

The robust tet-meshing-from-mesh pipeline with envelope-based validity guarantees — handles arbitrary input surfaces including self-intersecting and non-manifold meshes, with mesh-improvement passes that drive aspect ratio and dihedral angle into a usable band. Cited inline from [Part 7 Ch 01 — tetrahedralization strategies](../../70-sdf-pipeline/01-tet-strategies.md) as the default design-mode tet-generation pipeline in `sim-soft` (adapted to SDF input via iso-surface extraction); the SDF-pathology robustness is the property that makes it the right fit for `sim-soft`'s `SdfField`-driven design flow where the iso-surface of an arbitrary cf-design SDF can exhibit any surface pathology. The predecessor paper (Hu, Zhou, Gao, Jacobson, Zorin, Panozzo 2018 — "Tetrahedral Meshing in the Wild") is not anchored here because no inline citation currently references it; the `sim-soft` pipeline consumes the 2020 fast-variant.

## Pass 3 anchors (not yet inline-cited)

Reserved slots the Pass 3 bibliography will populate: DiffTaichi (Hu et al.), NVIDIA Warp (Warp paper), Genesis (Tsinghua), and the 2018 predecessor to fTetWild if later chapters reference it. None is anchored here yet because no inline citation currently references them by anchor.
