# Rendering papers

[Part 9 (visual layer)](../../90-visual/00-sss.md)'s commitments — multi-layer subsurface scattering, anisotropic GGX specular, contact-driven micro-wrinkles — trace back to the rendering literature. This leaf indexes the works Pass 1 cites by anchor. Pass 3 will expand the entries and add the broader SSS and microfacet-BRDF references (Walter, Burley, Jimenez) that Part 9's prose mentions without anchor.

## Li et al. 2018 {#li-2018}

*Differentiable Monte Carlo Ray Tracing through Edge Sampling.* ACM Transactions on Graphics Vol. 37, No. 6, Article 222 (SIGGRAPH Asia 2018), pp. 222:1–222:11. DOI [10.1145/3272127.3275109](https://doi.org/10.1145/3272127.3275109). [Author project page](https://people.csail.mit.edu/tzumao/diffrt/). Authors: Tzu-Mao Li, Miika Aittala, Frédo Durand, Jaakko Lehtinen.

The foundational paper on differentiable rendering through visibility discontinuities. Formulates the pixel-colour derivative as an interior smooth integral plus an explicit boundary integral along silhouette edges, with the boundary integral absorbing the Dirac contribution from the discontinuous visibility function, and a Monte-Carlo estimator that samples along edges via spatial-hierarchy importance sampling. Cited inline from [Part 6 Ch 05 (research frontier — Option B)](../../60-differentiability/05-diff-meshing.md) as the closest published parallel for a topology-aware FEM gradient: if the Dirac contribution at a mesh-topology-change boundary can be integrated analytically in the same style, the FD wrapper at topology boundaries can be replaced by an unbiased estimator. The extension to FEM topology changes is unpublished for volumetric FEM to the book's knowledge; [Part 12 Ch 07's Option B](../../120-roadmap/07-open-questions.md) names it as research frontier.

## Bangaru, Li & Durand 2020 {#bangaru-2020}

*Unbiased Warped-Area Sampling for Differentiable Rendering.* ACM Transactions on Graphics Vol. 39, No. 6, Article 245 (SIGGRAPH Asia 2020), pp. 245:1–245:18. DOI [10.1145/3414685.3417833](https://doi.org/10.1145/3414685.3417833). [Author project page](https://people.csail.mit.edu/sbangaru/projects/was-2020/). Authors: Sai Bangaru, Tzu-Mao Li, Frédo Durand.

Applies the divergence theorem to convert [Li et al. 2018](#li-2018)'s edge-boundary integral into an area integral via a warped-area reparameterization, giving an unbiased estimator without explicit silhouette detection. Cited inline from [Part 6 Ch 05 (research frontier — Option B)](../../60-differentiability/05-diff-meshing.md) as the canonical unbiased generalization of the visibility-boundary treatment — the book's Option B invokes unbiased estimation explicitly, and Bangaru 2020 is the rendering-side reference point for what a working unbiased boundary-integral estimator looks like. The FEM topology-change analog would need its own warped-area formulation; the rendering-side precedent shows the mathematical structure is tractable.

## Pass 3 anchors (not yet inline-cited)

Reserved slots Pass 3 will populate:

- Jimenez et al. 2015 — *Separable Subsurface Scattering*. Referenced by name in [Part 9 Ch 00](../../90-visual/00-sss.md) as the production-grade diffuse-profile screen-space SSS technique `sim-soft` ships; Pass 3 will promote the prose mention to an anchor-linked citation.
- Walter et al. 2007 — GGX microfacet distribution. Referenced in prose in [Part 9 Ch 01](../../90-visual/01-anisotropic-reflection.md).
- Burley 2012 — anisotropic GGX extension. Referenced in the same chapter.
- Jensen 2001 (BSSRDF) — the dipole-approximation reference for subsurface scattering theory.

None is anchored here yet because no inline citation currently references them by anchor.
