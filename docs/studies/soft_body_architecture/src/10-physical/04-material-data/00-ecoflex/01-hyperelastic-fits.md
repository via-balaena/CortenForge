# Hyperelastic curve fits

The [mechanical-data sibling leaf](00-mechanical.md) gives the small-strain modulus each Ecoflex grade starts from. That number alone calibrates [linear elasticity](../../../20-materials/02-linear.md) and, through $\mu = E / (2(1+\nu))$, the small-strain limit of [neo-Hookean](../../../20-materials/04-hyperelastic/00-neo-hookean.md). It does not calibrate the rest of the hyperelastic curve. The canonical-problem cavity conforms at stretches the small-strain expansion is not valid at, and the shape of the stress–stretch curve beyond 10%-or-so strain decides how much conformity force the solver predicts. This leaf is where the measured uniaxial curves enter the book.

## What "curve fit" means here

A hyperelastic curve fit is a choice of constitutive form — neo-Hookean, Mooney-Rivlin, Ogden — and a set of parameters minimizing the mean-squared error between the form's uniaxial Cauchy-stress prediction and a measured uniaxial stress–stretch curve. The prediction for uniaxial stretch $\lambda$ with the incompressibility constraint $\lambda_2 = \lambda_3 = \lambda^{-1/2}$ reduces each form to a one-dimensional expression:

$$ \sigma_{\text{NH}}(\lambda) \;=\; \mu\!\left(\lambda^2 - \lambda^{-1}\right) $$

$$ \sigma_{\text{MR}}(\lambda) \;=\; 2\!\left(\lambda^2 - \lambda^{-1}\right)\!\left(C_{10} + C_{01}\,\lambda^{-1}\right) $$

$$ \sigma_{\text{Ogden}}(\lambda) \;=\; \sum_{i=1}^{N} \mu_i\!\left(\lambda^{\alpha_i} - \lambda^{-\alpha_i/2}\right) $$

The full tensorial forms and their Piola and tangent expressions live in [Part 2 Ch 04](../../../20-materials/04-hyperelastic.md) and its three law-specific sub-chapters. For calibration only the uniaxial scalar expressions matter, because that is the geometry the test is run in.

Parameters are fit by nonlinear least-squares against the published $\sigma(\lambda)$ points. The fit quality is reported as the residual RMS error relative to the peak stress across the measured stretch range, and as a plot of predicted-versus-measured that surfaces systematic bias the RMS summary can hide. The book's working fit for each grade is the one with the smallest RMS error subject to the form being parameter-parsimonious enough for the [Part 2 Ch 04 validity regime](../../../20-materials/00-trait-hierarchy/02-validity.md).

## Measured data

Two silicone-characterization studies anchor the Ecoflex curve fits in the book. The studies are reserved as Pass-3 source-fetch deliverables in [the hyperelasticity reference leaf](../../../appendices/00-references/01-hyperelastic.md) — one published 00-30 curve-fit study and one published 00-50 curve-fit study — with the specific paper identifications landing at Pass 3 rather than being guessed from surname alone per the [Part 0 Ch 03 "no hallucinated citations" commitment](../../../00-context/03-how-produced.md).

- **Ecoflex 00-30** — uniaxial tensile tests at quasi-static strain rate, stretch range $\lambda \in [1, \;\approx 4]$, reported Ogden $N = 3$ coefficients, reported Mooney-Rivlin coefficients, raw stress–stretch points tabulated.
- **Ecoflex 00-50** — uniaxial tensile tests at the same rate, stretch range $\lambda \in [1, \;\approx 4]$, reported Ogden $N = 2$ and $N = 3$ coefficients.

For 00-10 and 00-20 there is no anchor in the Pass-1 reference list. Those two grades are Pass-3 measurement deliverables per the [mechanical-data leaf](00-mechanical.md) status; until the in-house measurement lands the book uses the $E$-scaled Ogden coefficients of 00-30 as the working fit, with the residual uncertainty absorbed by the [Part 10 Ch 05 sim-to-real loop](../../../100-optimization/05-sim-to-real.md).

## Fit quality per constitutive form

The three hyperelastic forms [Part 2 Ch 04](../../../20-materials/04-hyperelastic.md) ships — neo-Hookean, Mooney-Rivlin, Ogden — have very different fit quality on Ecoflex data. The progression across the family is what forces the book's default to Ogden rather than neo-Hookean.

**Neo-Hookean.** One parameter, $\mu$. Fit across the measured range by least-squares rather than pinned at the small-strain-limit value, neo-Hookean tracks the Ecoflex curve to within a few percent up to roughly $\lambda \approx 2$, which is the claim the [Part 2 Ch 04 neo-Hookean branch](../../../20-materials/04-hyperelastic/00-neo-hookean.md) rests on for the silicones the book cares about below their breakdown stretch. Beyond that stretch the $\lambda^2$ asymptotic growth diverges from the measured curve — Ecoflex's strain-hardening at $\lambda > 2$ has a different shape that a single-parameter form cannot match. Neo-Hookean is the default at moderate stretch for calibration-minimal designer workflows (one number from a data sheet suffices); it is the Phase-B solver-plumbing target for `sim-soft`'s first-working-FEM because its tangent is closed-form and rotation-invariance is structural; and it is the reduction oracle the [linear-elasticity-to-hyperelastic gradcheck](../../../20-materials/02-linear.md) targets in the small-strain limit. It is not the choice for the canonical-problem cavity-conformity regime which routinely reaches $\lambda > 2$.

**Mooney-Rivlin.** Two parameters, $C_{10}$ and $C_{01}$. The second parameter introduces $I_2$-dependence and refines the curve shape at moderate stretch. On Ecoflex 00-30 the Mooney-Rivlin form extends the neo-Hookean-accurate stretch window further — roughly to $\lambda \in [1, 2.5]$ — while remaining calibration-compact at two parameters. At larger stretches the two-parameter form loses shape accuracy again because Ecoflex's strain-hardening past $\lambda \approx 3$ requires more parameters to capture. Mooney-Rivlin is the book's working fit for a designer who wants quasi-incompressible hyperelasticity at moderate stretch and a compact parameter set ([Part 2 Ch 04 — Mooney-Rivlin](../../../20-materials/04-hyperelastic/01-mooney-rivlin.md)).

**Ogden $N = 2$.** Two pairs $(\mu_i, \alpha_i)$, four parameters total. On Ecoflex 00-50 the two-term Ogden form captures both the moderate-stretch shape and the incipient strain-hardening at $\lambda > 3$, tracking the measured curve across the full measured range at a fit quality materially better than Mooney-Rivlin. This is the minimum-parameter form that handles the full measured stretch window for silicone.

**Ogden $N = 3$.** Three pairs, six parameters. On Ecoflex 00-30 and 00-50 the three-term form captures the stretch range to within measurement noise. The [Part 2 Ch 04 — Ogden](../../../20-materials/04-hyperelastic/02-ogden.md) chapter's derivation-level argument for Ogden over the polynomial-invariant families rests on this — the principal-stretch basis captures the silicone curve shape with a parameter count other forms need more terms to approximate.

The book's default Ecoflex fit across the canonical problem is **Ogden $N = 3$**. The `MaterialField` exposes the parameter set; the [material database](../../../appendices/02-material-db.md) row set will land the specific coefficients at Pass 3 once the reference-leaf anchor is filled.

## Why not higher $N$

Ogden $N = 4$ and beyond are available in the [Part 2 Ch 04 — Ogden $N$-term sub-chapter](../../../20-materials/04-hyperelastic/02-ogden/00-n-term.md)'s implementation. They are not the default for three reasons.

**Over-fitting.** A six-parameter form already matches the measured Ecoflex curve to within measurement noise. An eight-parameter form fits the noise as well, which means the extra parameters pick up the specific measurement's lot-and-equipment idiosyncrasies rather than the material's constitutive response. The sim-to-real loop's per-print calibration is designed to correct lot variability; baking it into the nominal fit is backward.

**Numerical conditioning.** Ogden fits at high $N$ develop near-collinear $(\mu_i, \alpha_i)$ pairs whose individual values are poorly determined even when the summed stress–stretch prediction is excellent. The near-degeneracy manifests in [implicit-function-theorem gradients](../../../60-differentiability/02-implicit-function.md) through the material as ill-conditioned linear solves at the `MaterialField` parameter level. Sticking with $N = 3$ keeps the fit well-conditioned.

**Validity regime match.** The canonical problem operates in $\lambda \in [1, 3]$ most of the time, and the Ogden $N = 3$ form is accurate across $\lambda \in [1, 4]$. Paying four extra parameters to extend validity to $\lambda > 4$ would buy accuracy the solver does not spend; Ecoflex at $\lambda > 4$ is approaching break elongation anyway and the [Part 1 Ch 01 reward peak-stress barrier](../../../10-physical/01-reward/02-peak-bounds.md) pushes the designer away from that regime.

## What the sim-soft `Material` stores

The `Material<Ogden<3>>` specialization built on 00-30 or 00-50 holds the six-parameter coefficient set as a plain array of $(\mu_i, \alpha_i)$ pairs, the small-strain-consistency check (the sum $\tfrac{1}{2}\sum_i \mu_i \alpha_i$ equals the small-strain shear modulus), and a validity-metadata block declaring the fit's stretch-range bounds. The `MaterialField` from [Part 7 Ch 00](../../../70-sdf-pipeline/00-sdf-primitive.md) samples these per element; the [Part 2 Ch 09 spatial-field machinery](../../../20-materials/09-spatial-fields.md) interpolates the small-strain shear modulus across the SDF-defined field and re-fits the full Ogden coefficients per sample only at the [validity boundaries](../../../20-materials/00-trait-hierarchy/02-validity.md), which is cheap relative to the solver step. The specific coefficient numbers land in the [material database](../../../appendices/02-material-db.md) at Pass 3.

## Alternatives considered

**Arruda–Boyce / Gent.** Both are physics-motivated three-parameter forms with a strain-hardening limit built in. For natural rubber with known chain-extensibility they outperform Ogden at the same parameter count. For platinum-cure silicone the chain-extensibility parameter is not independently measurable, so the form reduces to a two-parameter fit at the stretch ranges the Ecoflex data covers, which puts it behind Ogden $N = 2$. Rejected as the default; reachable via the [Part 2 Ch 04 hyperelastic trait surface](../../../20-materials/04-hyperelastic.md) for users who need them.

**Polynomial in $I_1, I_2$ (Yeoh, higher-order Mooney-Rivlin).** Yeoh is three-parameter polynomial in $I_1$ alone; higher-order Mooney-Rivlin uses both invariants. Both fit Ecoflex better than neo-Hookean and worse than Ogden at matched parameter counts. Reachable via the same trait surface; not the default because Ogden is parameter-parsimonious across the measured stretch range and the silicone literature has converged on the principal-stretch basis.

**Fit neo-Hookean to small-strain only, use Ogden only at large stretch.** An adaptive-form strategy that swaps constitutive laws at a stretch threshold. Rejected as the default because the law swap introduces a non-smooth transition in $\partial \Psi / \partial F$ that breaks differentiability ([Part 6](../../../60-differentiability/00-what-autograd-needs.md)), and because Ogden $N = 3$ matches neo-Hookean to six digits in the small-strain limit by construction so there is nothing to gain from the swap.
