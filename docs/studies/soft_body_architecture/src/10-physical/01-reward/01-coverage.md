# Contact coverage

The [uniformity sibling](00-pressure-uniformity.md) penalized non-uniform pressure over the actively-contacted region. Coverage asks a complementary question: what fraction of the intended contact surface is actively contacted in the first place. A perfectly uniform pressure over 40% of the intended surface is not conformity; it is a narrow band of good contact with the rest of the sleeve floating. This leaf pins down the coverage term.

## What it measures

The area-weighted fraction of the intended contact surface $\Gamma$ where the local contact pressure exceeds a small positive threshold. Bounded in $[0, 1]$; 1 is full intended-surface coverage, 0 is no contact anywhere on $\Gamma$.

## Functional form

Using the same $\Gamma$ and $p(x)$ as [uniformity](00-pressure-uniformity.md), define a smooth active-contact indicator:

$$ c(x) \;=\; \operatorname{logistic}\!\big(\beta_c (p(x) - p_\text{th})\big) $$

with the same $\operatorname{logistic}$ named by word in [uniformity](00-pressure-uniformity.md) (the unsubscripted $\sigma$ is reserved for Cauchy stress per [`appendices/03-notation.md`](../../appendices/03-notation.md)), the same pressure threshold $p_\text{th}$ as uniformity's weight (so the two terms agree on what counts as actively contacted), and sharpness $\beta_c$. The coverage reward is the area-weighted mean of $c$ over $\Gamma$:

$$ J_\text{cov}(\theta) \;=\; \frac{1}{|\Gamma|} \int_\Gamma c(x) \, dA $$

Higher is better; bounded at $J_\text{cov} = 1$. The reward contribution is $R_\text{cov} = J_\text{cov}$, passed into [composition](04-composition.md).

## Why this form

Three choices matter.

**Smooth sigmoidal indicator, not binary.** A binary indicator $\mathbf{1}[p > p_\text{th}]$ produces a step function at the contact boundary and a discontinuous gradient: an element flipping from non-contact to contact adds or removes a whole $dA/|\Gamma|$ from the coverage reward, so the derivative with respect to any geometric parameter that moves the contact boundary is a delta function. The logistic sigmoid replaces the step with a smooth transition whose width is controlled by $\beta_c$, which [Part 1 Ch 03's thesis](../03-thesis.md) commits this book to. Optimization proceeds without brittle local minima at contact-boundary flips.

**Area-weighted mean over the full intended surface, not over active-contact only.** Coverage must sense the non-contacted region; computing the mean only where $p > p_\text{th}$ would make the denominator depend on the numerator and coverage would trivially score 1 regardless of extent. The denominator is the full area $|\Gamma|$.

**Shared threshold $p_\text{th}$ with the uniformity term.** Keeps the uniformity-vs-coverage decomposition coherent: the regions uniformity integrates over are exactly the regions coverage counts as contacted. The sharpness $\beta_c$ does not have to equal $\beta_w$; typically $\beta_c$ is somewhat larger (steeper) because coverage cares more about the specific transition location, whereas uniformity's weight acts as a mean-filter mask.

## What the solver exports

Same two fields as [uniformity](00-pressure-uniformity.md#what-the-solver-exports): per-surface-element contact pressure and the intended-contact-surface tag. Integration is done in the reward-evaluation callback as a reduction over surface elements.

## Gradient path

Smooth in $\theta$ through the IPC barrier VJP and the IFT adjoint, as for uniformity. One additional RHS to the backward solve on the shared factor-on-tape. Computationally cheap — the integrand is lower-order than uniformity's (no $\bar p$ reduction, no subtraction, no division).

## Alternatives considered

**Binary indicator (hard cutoff at $p_\text{th}$).** Zero compute cost, unambiguous. Rejected — non-differentiable in $\theta$, so the gradient-augmented optimizer in [Part 10 Ch 02](../../100-optimization/02-bayesopt.md) can't see the coverage gradient and is reduced to finite-difference estimates at every sample.

**L⁰-style active-element count rather than area-weighted integral.** Counts contacted elements, treats all elements as equal contributors. Rejected because the surface mesh has non-uniform element areas (rim vs. bulk), so counting elements weights the rim disproportionately. Area weighting is the unbiased measure.

**Volume of contact band (how deep contact extends into the cavity wall).** A richer measure, more informative for subsurface seal problems. Rejected as the default because it requires a per-tet pressure estimate rather than a per-surface-element estimate, which couples coverage to the bulk stress field and complicates the gradient. Left as a domain-user extension.

**Hausdorff distance between the actively-contacted region and the intended region.** Captures the geometric structure of missing-contact patches, not just their area. Rejected because Hausdorff distance is non-smooth (max over the set) and because the peak-of-mismatch information is adequately captured by uniformity's variance term.
