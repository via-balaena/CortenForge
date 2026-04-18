# Hyperelasticity and rheology papers

Hyperelastic constitutive laws are `sim-soft`'s default material model per the [Part 1 Ch 03 thesis](../../10-physical/03-thesis.md). This leaf indexes the foundational constitutive-law references (neo-Hookean, Mooney-Rivlin, Ogden, Holzapfel-Gasser-Ogden) plus the saddle-point-stability pair ([Part 2 Ch 05 mixed u-p](../../20-materials/05-incompressibility/01-mixed-up.md) depends on) and the large-deformation viscoelastic / rheological model family ([Part 2 Ch 07 Oldroyd-B sub-chapter](../../20-materials/07-viscoelastic/01-oldroyd.md) depends on). The silicone curve-fit studies the [material database](../02-material-db.md) will cite at Pass 3 are reserved in the Pass-3 anchor list below.

Pass 1 populates only anchors referenced inline by the book; the Phase-α-close reference-anchor sweep (Pass 3, Parts 1 + 2) lands the inline soft-deferrals from α.1 / α.2 / α.Part-2. Pass 3 will expand each entry with full bibliographic form (exact pagination, DOI) and adjacent references the book does not currently inline-cite.

## Mooney 1940 {#mooney-1940}

*A Theory of Large Elastic Deformation.* Journal of Applied Physics 11 (1940).

The original two-term invariant polynomial $\psi = C_{10}(I_1 - 3) + C_{01}(I_2 - 3)$ for incompressible rubber elasticity. Cited inline from [Part 2 Ch 04 — Mooney-Rivlin parent](../../20-materials/04-hyperelastic/01-mooney-rivlin.md) as the first half of the Mooney-Rivlin attribution pair.

## Rivlin 1948 {#rivlin-1948}

*Large Elastic Deformations of Isotropic Materials. IV. Further Developments of the General Theory.* Philosophical Transactions of the Royal Society A 241 (1948).

The generalization of Mooney's two-term form to the full invariant series $\psi = \sum_{i,j} C_{ij}\,(I_1 - 3)^i (I_2 - 3)^j$ plus the first systematic treatment of rotation-invariance and objectivity for nonlinear elastic solids. Cited inline from [Part 2 Ch 04 — Mooney-Rivlin parent](../../20-materials/04-hyperelastic/01-mooney-rivlin.md) as the second half of the Mooney-Rivlin attribution pair.

## Ogden 1972 {#ogden-1972}

*Large Deformation Isotropic Elasticity — On the Correlation of Theory and Experiment for Incompressible Rubberlike Solids.* Proceedings of the Royal Society A 326 (1972).

The principal-stretch form $\psi = \sum_i (\mu_i / \alpha_i)(\lambda_1^{\alpha_i} + \lambda_2^{\alpha_i} + \lambda_3^{\alpha_i} - 3)$ with non-integer exponents $\alpha_i$. Cited inline from [Part 2 Ch 04 — Ogden parent](../../20-materials/04-hyperelastic/02-ogden.md) as the source of the $N$-term form and from the [why-Ogden sub-leaf](../../20-materials/04-hyperelastic/02-ogden/01-why-ogden.md) as the reference for the function-shape-gap argument (non-integer $\alpha_i$ reaches energy shapes outside the invariant-polynomial family). The book's default silicone fit uses $N = 2$ or $N = 3$ per the [Ecoflex hyperelastic-fits leaf](../../10-physical/04-material-data/00-ecoflex/01-hyperelastic-fits.md).

## Holzapfel, Gasser & Ogden 2000 {#holzapfel-gasser-ogden-2000}

*A New Constitutive Framework for Arterial Wall Mechanics and a Comparative Study of Material Models.* Journal of Elasticity 61 (2000).

The HGO fiber-reinforced anisotropic hyperelastic law — exponential fiber-strain energy $\psi_\text{fiber} = (k_1 / 2 k_2)(e^{k_2 (I_4 - 1)^2} - 1)$ stacked on an isotropic base, with an optional two-fiber-family extension for materials with orthogonal fiber networks. Cited inline from [Part 2 Ch 06 — anisotropic parent](../../20-materials/06-anisotropic.md) and [HGO sub-leaf](../../20-materials/06-anisotropic/01-hgo.md) as the default `Anisotropic<M, R>` specialization in `sim-soft`, and from [Part 1 Ch 02 — material-poverty](../../10-physical/02-what-goes-wrong/05-material-poverty.md) as the anisotropic law the book points at for directional soft parts.

## Babuška 1973 and Brezzi 1974 {#babuska-brezzi}

Babuška, I. 1973 — *The Finite Element Method with Lagrangian Multipliers*, Numerische Mathematik 20, 179–192.

Brezzi, F. 1974 — *On the Existence, Uniqueness and Approximation of Saddle-Point Problems Arising from Lagrangian Multipliers*, RAIRO Analyse Numérique 8 (R-2), 129–151.

The independent derivations that together state the inf-sup (Ladyzhenskaya-Babuška-Brezzi) condition — the compatibility requirement between displacement and Lagrange-multiplier (pressure) spaces that governs stability of saddle-point finite-element pairs. Cited inline from [Part 2 Ch 05 — mixed u-p sub-leaf](../../20-materials/05-incompressibility/01-mixed-up.md) as the source of the stability-concern argument for `sim-soft`'s $P_1$–$P_0$ Tet4-plus-constant-pressure pair under strict incompressibility, with the $1/\kappa$ regularization at finite bulk modulus acting as the Tikhonov-style stabilizer that damps spurious pressure modes out of the near-incompressibility regime the book ships. The anchor covers both papers because the book cites the inf-sup condition as a pair-attributed result.

## Viscoelastic and rheological models

The [Part 2 Ch 07 viscoelastic parent](../../20-materials/07-viscoelastic.md) commits to Prony series as the default small-strain viscoelastic parametrization and to [Oldroyd-B](../../20-materials/07-viscoelastic/01-oldroyd.md) as the frame-indifferent large-deformation specialization. Prony is a classical-rheology construction with many near-equivalent citations and is not anchored here; Oldroyd-B and its UCD-family cousins are named in prose and anchored below.

### Oldroyd 1950 {#oldroyd-1950}

*On the Formulation of Rheological Equations of State.* Proceedings of the Royal Society A 200 (1950).

The foundational upper-convected-derivative / lower-convected-derivative objectivity treatment. The Oldroyd-B model (upper-convected) and its Oldroyd-A counterpart (lower-convected) both trace to this paper's tensor-evolution framework. Cited inline from [Part 2 Ch 07 — Oldroyd-B sub-leaf](../../20-materials/07-viscoelastic/01-oldroyd.md) as the source of the frame-indifferent stress-evolution equation $\overset{\nabla}{Q}_i + Q_i/\tau_i = 2\,G_i\,D$ that `sim-soft`'s `OldroydB<M>` decorator ships, and as the reference the "alternative UCD-family choices (Oldroyd-A, …)" aside points at.

### Johnson & Segalman 1977 {#johnson-segalman-1977}

*A Model for Viscoelastic Fluid Behavior which Allows Non-Affine Deformation.* Journal of Non-Newtonian Fluid Mechanics 2 (1977).

Introduces a Gordon-Schowalter-style interpolation between the upper-convected and lower-convected derivatives, parameterized by a scalar slip parameter. The non-affine deformation accounts for network slip relative to the continuum velocity gradient. Cited inline from the [Oldroyd-B sub-leaf](../../20-materials/07-viscoelastic/01-oldroyd.md) as a named alternative UCD-family member not on the Phase A–I roadmap.

### Phan-Thien & Tanner 1977 {#phan-thien-tanner-1977}

*A New Constitutive Equation Derived from Network Theory.* Journal of Non-Newtonian Fluid Mechanics 2 (1977).

The PTT model — adds a linear (or exponential, in the later Phan-Thien 1978 variant) function of the trace of the polymer stress tensor to the relaxation term, producing shear-thinning and bounded extensional stresses that Oldroyd-B lacks. Cited inline from the [Oldroyd-B sub-leaf](../../20-materials/07-viscoelastic/01-oldroyd.md) as a named alternative UCD-family member not on the Phase A–I roadmap.

## Pass 3 anchors (not yet inline-cited)

Reserved slots the Pass 3 bibliography will populate when source-fetch confirms the specific paper:

- Neo-Hookean — variously attributed to Rivlin 1948 (as a special case of the general invariant series above) and to Treloar's *The Physics of Rubber Elasticity* textbook formulations. `sim-soft`'s neo-Hookean form follows the standard compressible generalization in the textbook lineage rather than any single paper, so no primary anchor is reserved.
- Bonet & Wood (textbook) — the standard reference for nonlinear-continuum-mechanics derivations Part 2 cites implicitly when deriving first-Piola expressions and tangent-stiffness forms. Retracted from prose during session 2 α.Part-2 pending Pass-3 source-fetch of the specific edition the book's derivations match.
- Marechal et al. — an Ecoflex 00-30 hyperelastic curve-fit study referenced by [Part 1 Ch 04](../../10-physical/04-material-data/00-ecoflex/01-hyperelastic-fits.md) for the 00-30 fit. Pass-3 source-fetch will confirm the specific paper before anchoring.
- Liao et al. — an Ecoflex 00-50 hyperelastic curve-fit study referenced in the same chapter. Same Pass-3-source-fetch status.
- A Dragon Skin curve-fit study — [Part 1 Ch 04 Dragon Skin hyperelastic-fits leaf](../../10-physical/04-material-data/01-dragonskin/01-hyperelastic-fits.md)'s Mooney-Rivlin data trace. Pass-3 measurement deliverable, no single published fit the book commits to yet.
- A Shore-to-modulus correlation source — referenced from both the [Ecoflex](../../10-physical/04-material-data/00-ecoflex/00-mechanical.md) and [Dragon Skin](../../10-physical/04-material-data/01-dragonskin/00-mechanical.md) mechanical leaves. The specific published correlation the book relies on is a Pass-3 source-fetch deliverable; multiple near-equivalent functional forms exist in the soft-robotics and industrial-elastomer literature.

Anchoring any of these now, before the Pass-3 source fetch, would violate the [Part 0 Ch 03 "no hallucinated citations" commitment](../../00-context/03-how-produced.md) that binds this appendix.
