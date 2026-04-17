# Rendering papers

[Part 9 (visual layer)](../../90-visual/00-sss.md)'s commitments — multi-layer subsurface scattering, anisotropic GGX specular, contact-driven micro-wrinkles — trace back to the rendering literature. This leaf indexes the works Pass 1 cites by anchor. Pass 3 will expand the entries and add the broader SSS and microfacet-BRDF references (Walter, Burley, Jimenez) that Part 9's prose mentions without anchor.

## Li et al. 2018 {#li-2018}

*Differentiable Monte Carlo Ray Tracing through Edge Sampling.* ACM Transactions on Graphics (SIGGRAPH Asia 2018).

The foundational paper on differentiable rendering through visibility discontinuities. Treats the Dirac contribution at silhouette edges — where the pixel colour jumps as a scene parameter crosses a visibility boundary — as an analytically-integrated boundary integral plus a Monte-Carlo estimator that samples along edges. Cited inline from [Part 6 Ch 05 (research frontier — Option B)](../../60-differentiability/05-diff-meshing.md) as the closest published parallel for a topology-aware FEM gradient: if the Dirac contribution at a mesh-topology-change boundary can be integrated analytically in the same style, the FD wrapper at topology boundaries can be replaced by an unbiased estimator. The extension to FEM is unpublished; [Part 12 Ch 07's Option B](../../120-roadmap/07-open-questions.md) names it as research frontier.

## Pass 3 anchors (not yet inline-cited)

Reserved slots Pass 3 will populate:

- Jimenez et al. 2015 — *Separable Subsurface Scattering*. Referenced by name in [Part 9 Ch 00](../../90-visual/00-sss.md) as the production-grade diffuse-profile screen-space SSS technique `sim-soft` ships; Pass 3 will promote the prose mention to an anchor-linked citation.
- Walter et al. 2007 — GGX microfacet distribution. Referenced in prose in [Part 9 Ch 01](../../90-visual/01-anisotropic-reflection.md).
- Burley 2012 — anisotropic GGX extension. Referenced in the same chapter.
- Jensen 2001 (BSSRDF) — the dipole-approximation reference for subsurface scattering theory.

None is anchored here yet because no inline citation currently references them by anchor.
