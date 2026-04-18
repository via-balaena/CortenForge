# Calibration to silicone

The [two-parameter sibling](00-two-param.md) wrote down the Mooney-Rivlin energy; this leaf walks the fit — why uniaxial data alone leaves $(C_{10}, C_{01})$ ill-conditioned, what protocol `sim-soft` accepts as calibrated, and where the biaxial-source data comes from in practice. The [tradeoffs sibling](02-tradeoffs.md) takes the calibrated coefficients and decides whether Mooney-Rivlin is worth running at all for the test case at hand.

## The uniaxial reduced stress

Under uniaxial incompressible stretch $\lambda$, the principal stretches are $(\lambda, \lambda^{-1/2}, \lambda^{-1/2})$ and the invariants reduce to $I_1 = \lambda^2 + 2\lambda^{-1}$, $I_2 = 2\lambda + \lambda^{-2}$. The incompressible Mooney-Rivlin energy $\psi = C_{10}(I_1 - 3) + C_{01}(I_2 - 3)$ has engineering stress $\sigma_\text{eng} = \mathrm{d}\psi/\mathrm{d}\lambda$:

$$ \sigma_\text{eng}(\lambda) \;=\; 2C_{10}\,(\lambda - \lambda^{-2}) \;+\; 2C_{01}\,(1 - \lambda^{-3}) \;=\; 2(\lambda - \lambda^{-2})\,\left(C_{10} + C_{01}\,\lambda^{-1}\right) $$

Dividing out the $2(\lambda - \lambda^{-2})$ factor gives the *reduced stress*

$$ y(\lambda) \;\equiv\; \frac{\sigma_\text{eng}(\lambda)}{2(\lambda - \lambda^{-2})} \;=\; C_{10} \;+\; C_{01}\,\lambda^{-1} $$

which is linear in $(C_{10}, C_{01})$ against the basis functions $\{1, \lambda^{-1}\}$. Plotting $y$ against $\lambda^{-1}$ — the classical Mooney plot — turns the fit into a straight-line regression: intercept is $C_{10}$, slope is $C_{01}$.

## Why uniaxial alone is ill-conditioned

The Mooney plot is a two-parameter linear regression, which looks well-posed in the abstract, but the conditioning depends on how the data populates the $\lambda^{-1}$ axis. Uniaxial stretch data on silicone typically covers $\lambda \in [1.05, 3]$ (small strains are lost to measurement noise, large strains saturate or fail the coupon), which maps to $\lambda^{-1} \in [0.33, 0.95]$ — less than one decade on the regressor. The two columns of the design matrix $\left[\mathbf{1},\, \boldsymbol{\lambda}^{-1}\right]$ are therefore close to each other on the fit's support, the normal matrix is poorly conditioned, and measurement noise on $y$ amplifies into a substantially noisier $C_{01}$ than $C_{10}$ with the two estimates compensating each other.

Worse, the fit is near-indistinguishable from the neo-Hookean one-parameter fit over the uniaxial range: any reasonable uniaxial-only dataset can be fit nearly as well by $(C_{10}, 0)$ as by any nominal $(C_{10}', C_{01}')$ pair, with residual differences smaller than measurement noise. The information that the second parameter is supposed to capture — the $I_2$ sensitivity — is simply absent from uniaxial data because $I_1$ and $I_2$ are close to collinear functions of $\lambda$ in the uniaxial regime. A uniaxial-only Mooney-Rivlin fit therefore reports two parameters but carries only one parameter's worth of information.

This is not a numerical failing `sim-soft` can fix with a better solver; it is an identifiability failing of the data itself. The remedy is more data, not a better regression.

## The biaxial complement

Under equibiaxial incompressible stretch $\lambda_1 = \lambda_2 = \lambda$, $\lambda_3 = \lambda^{-2}$, the invariants become $I_1 = 2\lambda^2 + \lambda^{-4}$ and $I_2 = \lambda^4 + 2\lambda^{-2}$. The engineering stress in either in-plane direction has the same reduced form,

$$ y_\text{biax}(\lambda) \;=\; \frac{\sigma_\text{eng}^\text{biax}(\lambda)}{2(\lambda - \lambda^{-5})} \;=\; C_{10} \;+\; C_{01}\,\lambda^{2} $$

Crucially, the $C_{01}$ coefficient is multiplied by $\lambda^2$ rather than by $\lambda^{-1}$. At $\lambda = 1.5$, the uniaxial fit sees $C_{01}\,\lambda^{-1} = 0.67\,C_{01}$; the biaxial fit sees $C_{01}\,\lambda^2 = 2.25\,C_{01}$ — a more than three-fold leverage. Biaxial data at moderate stretch is therefore $C_{01}$-dominated, whereas uniaxial data is $C_{10}$-dominated, and the joint fit has a well-conditioned normal matrix with orthogonal-ish columns instead of nearly collinear ones.

Planar shear (pure shear in the incompressible limit: $\lambda_1 = \lambda$, $\lambda_2 = 1$, $\lambda_3 = \lambda^{-1}$) is the alternative complement. It satisfies $I_1 = I_2 = \lambda^2 + 1 + \lambda^{-2}$ identically, so planar-shear data imposes a single linear constraint on the sum $C_{10} + C_{01}$ — complementary to uniaxial's $C_{10}$-dominated constraint and usable as a fit regularizer rather than a third parameter. Planar shear is easier to run than equibiaxial on a standard uniaxial rig (it only requires wide grips and a thin coupon), so `sim-soft`'s calibration accepts either uniaxial-plus-biaxial or uniaxial-plus-planar-shear as the minimum protocol.

## Rig sources

`sim-soft` accepts calibration from three rig configurations. The [Part 2 Ch 07 DMA protocol](../../07-viscoelastic/03-dma.md) is the canonical source at low frequency.

- **Uniaxial tensile.** Standard ASTM D412 or ISO 37 dumbbell coupon on a pull-test rig. Covers $\lambda \in [1.05, 3]$ or so. Ecoflex 00-30 and Dragon Skin 20A data from manufacturer sheets and published silicone-characterization papers are all in this form; [Part 1 Ch 04](../../../10-physical/04-material-data.md) catalogs the specific sources.
- **Planar shear.** Wide grips (specimen aspect ratio $\geq 4{:}1$) on the same uniaxial rig. A thin silicone sheet stretched along its length with width held fixed approximates planar shear in the central region. Cheap to run; more sensitive to grip slip than uniaxial.
- **Equibiaxial.** Bulge test (inflating a clamped silicone sheet with regulated pressure, measuring the cap height) or a dedicated biaxial pull rig with orthogonal grip pairs. Less common in published silicone data, more common in bespoke lab work. [Part 2 Ch 07's DMA protocol](../../07-viscoelastic/03-dma.md) doubles as the biaxial source when run at a frequency low enough that viscoelastic response sits at its equilibrium limit — the DMA leaf names the rate regime and the fixture specialization required.

The fitted coefficients are reported in `appendices/02-material-db.md` alongside each silicone's entry. `sim-soft` refuses to accept a `MooneyRivlin` instance whose `from_measured` constructor was called with uniaxial-only data and surfaces the protocol-incompleteness at construction, not at solve time.

## Fitting residual and validity reporting

The fit itself is a linear least-squares regression on $(C_{10}, C_{01})$ against the stacked uniaxial and biaxial (or planar-shear) reduced stresses, weighted by the inverse variance of each rig's measurement noise. `sim-soft` reports three artifacts per fit:

- The fitted coefficients $(C_{10}, C_{01})$ and their covariance from the regression's normal matrix.
- The RMS residual in engineering stress across the calibration stretch range, expressed as a percentage of the maximum measured stress.
- The stretch range $[\lambda_\min, \lambda_\max]$ over which the RMS residual stays below a declared tolerance — the validity range reported on the [`ValidityDomain`](../../00-trait-hierarchy/02-validity.md) struct's stretch slot.

The third is what the [gradcheck suite's](../../../110-crate/04-testing/03-gradcheck.md) out-of-domain asymptotic test operates on: `MooneyRivlin` is claimed valid up to $\lambda_\max$, and the error's quadratic divergence above $\lambda_\max$ (against a reference Ogden fit on the same data) is what the test asserts.

## What this sub-leaf commits the book to

- **`sim-soft` rejects uniaxial-only calibration for Mooney-Rivlin.** The `from_measured` constructor takes stacked dataset records with protocol tags, and the two-protocol requirement is enforced at construction rather than reported as a warning. A user who has only uniaxial data gets a neo-Hookean fit, not an ill-conditioned Mooney-Rivlin fit.
- **Calibration reports covariance, not just point estimates.** $(C_{10}, C_{01})$ without their covariance is incomplete because the covariance is what the ill-conditioning shows up in. Downstream [sim-to-real calibration](../../../100-optimization/05-sim-to-real.md) consumes the covariance when it propagates uncertainty through the reward.
- **The validity stretch range is a fit artifact, not a compile-time constant.** `MooneyRivlin`'s `ValidityDomain::max_stretch_deviation` is populated from the fit's residual-below-tolerance bound at `from_measured` time, not hard-coded. A calibration that covers less stretch range returns a tighter validity domain, and the runtime warning surfaces when a Gauss point exceeds the fit's actual support rather than a nominal constant.
- **Published silicone characterization is usually uniaxial-only.** Manufacturer sheets almost never include biaxial or planar-shear data. [Part 1 Ch 04](../../../10-physical/04-material-data.md)'s material database entries for Ecoflex and Dragon Skin are uniaxial-plus-published-biaxial where available, and the entries without biaxial data land neo-Hookean coefficients rather than Mooney-Rivlin.
