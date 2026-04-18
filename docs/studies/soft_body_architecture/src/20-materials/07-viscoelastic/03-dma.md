# DMA measurement protocol

The [Ch 07 parent's Claim 2](../07-viscoelastic.md) commits to the Prony series as the default viscoelastic parametrization, calibrated from DMA data. This leaf names the experimental protocol — sinusoidal small-strain excitation across a frequency band — derives the storage-and-loss-modulus form the [Prony series](00-prony.md) takes as a function of frequency, and writes the nonlinear least-squares fit that produces the $(\tau_i, g_i)$ tuples the [`Viscoelastic<M>` decorator](../00-trait-hierarchy/01-composition.md) consumes at material setup time.

## The DMA setup

Dynamic mechanical analysis (DMA) drives a small-strain sinusoidal deformation at angular frequency $\omega$ — typically shear or uniaxial tension on a thin sample — and records the resulting stress amplitude and phase. For a small-strain input $\varepsilon(t) = \varepsilon_0 \sin(\omega t)$, the linear viscoelastic response decomposes as:

$$ \sigma(t) = \varepsilon_0 \left[ G'(\omega) \sin(\omega t) + G''(\omega) \cos(\omega t) \right] $$

with the storage modulus $G'(\omega)$ — the in-phase elastic response — and the loss modulus $G''(\omega)$ — the out-of-phase dissipative response. Sweeping $\omega$ across the operating band produces $(G'(\omega_k), G''(\omega_k))$ measurements at each tested frequency; the goal of calibration is to fit a Prony series whose modulus curves match the measurements.

For silicones in the canonical-problem regime, the operating band typically spans several decades — capturing the slow load relaxations relevant to the [wobble failure mode](../../10-physical/02-what-goes-wrong/01-wobble.md) at the low end and the fast contact-impact transients at the high end. Measurement protocols are governed by ASTM and ISO standards for elastomeric materials; per `sim-soft`'s convention, the standards are named (not URL-linked) here and the calibration tool consumes whatever frequency-vs-modulus tabulation the user provides.

## Prony's storage and loss modulus

For the Prony series in the equilibrium-base convention $g(t) = 1 + \sum_i g_i\, e^{-t/\tau_i}$, the complex modulus as a function of angular frequency is:

$$ G^*(\omega) = G_\infty \left(1 + \sum_{i=1}^{N} g_i\, \frac{i\omega \tau_i}{1 + i\omega \tau_i}\right) $$

Separating real and imaginary parts gives the storage and loss modulus:

$$ G'(\omega) = G_\infty \left(1 + \sum_{i=1}^{N} g_i\, \frac{(\omega \tau_i)^2}{1 + (\omega \tau_i)^2}\right) $$

$$ G''(\omega) = G_\infty \sum_{i=1}^{N} g_i\, \frac{\omega \tau_i}{1 + (\omega \tau_i)^2} $$

with $G_\infty$ the equilibrium (long-time) shear modulus — equivalent to the [Lamé parameter $\mu$](../02-linear.md) the wrapped base material specifies in its small-strain limit. Two limits:

- **Low frequency ($\omega \to 0$):** $G' \to G_\infty$, $G'' \to 0$ — the fully-relaxed equilibrium response.
- **High frequency ($\omega \to \infty$):** $G' \to G_\infty (1 + \sum_i g_i)$, $G'' \to 0$ — the glassy (instantaneous) response with no dissipation because the deformation is too fast for any mode to relax during a cycle.

The loss modulus peaks at intermediate frequencies: each mode contributes a Lorentzian-like peak in $G''$ centred at $\omega = 1/\tau_i$ with peak height $g_i G_\infty / 2$. Reading the locations and heights of the loss peaks off a measured $G''(\omega)$ curve gives a first guess for the $(\tau_i, g_i)$ pairs.

## The fit

With $\{(\omega_k, G'_\text{meas}(\omega_k), G''_\text{meas}(\omega_k))\}_{k=1}^{K}$ from the DMA sweep, the Prony fit minimizes a weighted squared-residual loss across both moduli:

$$ L(\{(\tau_i, g_i)\}) = \sum_{k=1}^{K} w_k \left[\left( G'_\text{fit}(\omega_k) - G'_\text{meas}(\omega_k) \right)^2 + \left( G''_\text{fit}(\omega_k) - G''_\text{meas}(\omega_k) \right)^2\right] $$

with weights $w_k$ typically chosen to balance contributions across the (logarithmically-spaced) frequency band. The fit is nonlinear in the $\tau_i$ and quadratic in the $g_i$; standard nonlinear-least-squares (Levenberg-Marquardt) handles both jointly. With $N = 3$ or $4$ modes and a few-dozen DMA points, the fit converges in tens of iterations and produces residuals at or below the measurement repeatability of the DMA instrument.

The number of modes $N$ is a calibration choice: too few modes underfit the loss-peak shape; too many modes overfit measurement noise. The fit residual against an out-of-fit-band validation point (a DMA measurement at a frequency held out from the fit) is the standard discriminator.

## Validity-domain handoff

The fitted $(\tau_i, g_i)$ tuples are valid in the small-strain regime — the linear-viscoelastic operating window the DMA itself measures. The fit's strain-amplitude validity range is reported into the [`ValidityDomain`](../00-trait-hierarchy/02-validity.md)'s `strain_rate_range` slot when the Prony parameters are loaded into a `Viscoelastic<M>` decorator. Above the small-strain boundary, the [Oldroyd-B variant](01-oldroyd.md) is the recommended large-deformation extension of the same parameter set.

The DMA fit happens once per material at calibration time; it is an external calibration tool, not part of the runtime solver. The output is a `Vec<(tau_i, g_i)>` ready to drop into the decorator's `prony` field at material construction.

## What this sub-leaf commits the book to

- **DMA produces $(G'(\omega_k), G''(\omega_k))$ pairs across the operating band.** The standard small-strain sinusoidal protocol; specific protocols are governed by ASTM/ISO standards, named without URL.
- **Prony's storage and loss modulus have closed forms in $(\tau_i, g_i)$.** Lorentzian-like peaks in $G''$ at $\omega = 1/\tau_i$ with heights $g_i G_\infty / 2$ give visual first guesses for the fit.
- **The Prony fit is nonlinear least-squares with Levenberg-Marquardt.** Few-dozen DMA points + $N = 3$ or $4$ modes converges in tens of iterations.
- **The fit is small-strain-validity-bounded.** The DMA's strain-amplitude range becomes the fitted decorator's validity ceiling; large deformation is the [Oldroyd-B](01-oldroyd.md) regime.
- **DMA fitting is an external calibration tool, not a runtime component.** Output `Vec<(tau_i, g_i)>` drops into the decorator's `prony` field at material construction.
