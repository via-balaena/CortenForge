# Viscoelastic spectrum

Ecoflex dissipates energy under cyclic load. The dissipation is modest by rubber standards — below an order of magnitude of what natural rubber loses per cycle — but it is not zero, and the [Part 1 Ch 02 wobble failure](../../02-what-goes-wrong/01-wobble.md) is what happens in the solver when a viscoelastic term is missing from an otherwise-correct hyperelastic fit. The quasi-static canonical problem runs fine without viscoelasticity for a handful of steps; the moment the solver sees a transient, a release, or a cyclic load, the missing damping turns into un-physical mesh oscillations at the natural frequencies of the FEM basis. This leaf names the measured spectrum and the Prony parameters the [Part 2 Ch 07 viscoelastic trait](../../../20-materials/07-viscoelastic.md) consumes to close that failure mode.

## What "spectrum" means here

The viscoelastic response of an isotropic silicone under small-amplitude oscillatory shear at frequency $\omega$ is characterized by two frequency-dependent scalars: the storage modulus $G'(\omega)$ (in-phase, elastic-like) and the loss modulus $G''(\omega)$ (out-of-phase, dissipative). Dynamic mechanical analysis — the [Part 2 Ch 07 DMA protocol leaf](../../../20-materials/07-viscoelastic/03-dma.md) — sweeps $\omega$ across the frequency range the application cares about and measures both moduli.

A Prony series approximates the measured spectrum with $N$ Maxwell branches in parallel with an elastic spring. Each branch has a relaxation time $\tau_i$ and a weight $g_i$ (dimensionless, sums to the total relaxation fraction). The complex shear modulus the series predicts is:

$$ G^{\ast}(\omega) \;=\; G_{\infty} + \sum_{i=1}^{N} \frac{G_i\,(i\omega\tau_i)}{1 + i\omega\tau_i} $$

where $G_{\infty}$ is the equilibrium (long-time) modulus and $G_i = G_{\infty}\,g_i/(1 - \sum_j g_j)$. Fitting the Prony parameters means nonlinear least-squares against the measured $G'(\omega)$ and $G''(\omega)$ simultaneously, with $N$ and the initial $\tau_i$ spacing chosen logarithmically across the frequency window the application spans.

## Measured spectrum

The measured DMA response for Ecoflex across the grade ladder, in the frequency window $10^{-2}$ Hz to $10^2$ Hz that covers the canonical problem's quasi-static to human-hand-cyclic regime, has four features worth naming.

- The storage modulus $G'(\omega)$ is nearly flat — roughly a factor of 1.5 rise across four decades of frequency. This is why the small-strain Young's modulus values in the [mechanical-data leaf](00-mechanical.md) are useful as a quasi-static anchor across the whole window; the grade's stiffness does not shift by an order of magnitude across relevant loading rates.
- The loss modulus $G''(\omega)$ is an order of magnitude smaller than $G'$ at each frequency — the loss factor $\tan \delta = G''/G'$ sits in the 0.05–0.1 band across the window.
- The spectrum does not vary dramatically across the four Ecoflex grades. 00-10 and 00-50 differ by a 4× multiplicative factor in $G'(\omega)$ at fixed frequency, consistent with their small-strain modulus ratio; the relaxation-time spectrum (the *shape* of the frequency dependence, once $G'$ is normalized to its equilibrium value) transfers within measurement noise. This is the single-fit-transfers-across-ladder claim the [Ecoflex branch](../00-ecoflex.md) rests on.
- There is no single dominant relaxation time. The Prony weights $g_i$ spread across two-to-three decades, which is what forces $N \geq 3$ in the fit.

## The Prony fit

The `Material<Viscoelastic<NeoHookean>>` and `Material<Viscoelastic<Ogden<3>>>` wrappers the book ships use an $N = 3$ Prony series on Ecoflex 00-30 as the canonical working fit. The three relaxation times span roughly $\tau_1 \approx 10^{-1}$ s, $\tau_2 \approx 10^{0}$ s, $\tau_3 \approx 10^{1}$ s, with weights in the 0.05–0.15 band each, summing to a total relaxation fraction on the order of 0.3 of $G(0)$. The specific coefficient set lands in the [material database](../../../appendices/02-material-db.md) at Pass 3 once the DMA-protocol-driven fit is anchored to a measurement pass.

Two things about the fit that the downstream solver depends on.

**The equilibrium modulus is the small-strain $\mu$ from the [mechanical-data leaf](00-mechanical.md).** The DMA-measured $G'(\omega \to 0)$ matches $\mu$ from the Shore-correlation or the measured quasi-static uniaxial curve to within data-sheet precision. The Prony fit fixes $G_{\infty}$ at this value and fits only the relaxation weights and times. This is what makes the viscoelastic wrapper a *decorator* on the elastic core ([Part 2 Ch 07 Claim 1](../../../20-materials/07-viscoelastic.md)) rather than a fresh constitutive form — the elastic fit is inherited, not refit.

**The $N = 3$ fit is the floor, not the ceiling.** $N = 4$ is a reachable Pass-3 upgrade if the canonical problem's loading spectrum concentrates in a narrow decade where $N = 3$ leaves structured residuals. The validity-metadata block on the `Material<Viscoelastic<...>>` carries the fit's residual RMS relative to peak $G''$ across the measured window, and the wrapper declares its validity domain accordingly.

## Per-grade transfer

The branch-level claim that a single Prony fit calibrated on 00-30 transfers to the other three grades is the working assumption, not the end state. Two qualifications matter.

First, it transfers at the relaxation-time-spectrum level, not at the absolute-modulus level. The Prony weights $g_i$ and relaxation times $\tau_i$ transfer from 00-30 to 00-10/00-20/00-50 within DMA noise, but the equilibrium modulus $G_{\infty}$ is grade-specific — that is the small-strain modulus from the [mechanical-data leaf](00-mechanical.md), and differs across the ladder by the 4× factor the grade ladder is built on.

Second, the transfer is Pass-1 accuracy, not per-grade calibration. Per-grade DMA measurement is a Pass-3 deliverable alongside the 00-10 and 00-20 uniaxial curves, and the expected refinement is at the relaxation-weight level — the $g_i$ values may drift by ten-to-twenty percent across the ladder in a way that a single-fit-on-00-30 does not capture. The [Part 10 Ch 05 sim-to-real loop](../../../100-optimization/05-sim-to-real.md) absorbs this drift per-print until per-grade DMA lands, which is the pattern every data-constrained calibration in the book follows.

## How this closes the wobble failure

The [Part 1 Ch 02 wobble failure](../../02-what-goes-wrong/01-wobble.md) in the failure catalogue is what an undamped solver does when a load is applied, held, and released on the canonical cavity: the cavity rings at its first-mode natural frequency for a time that depends on the integrator's numerical damping alone. Real silicone rings for a handful of cycles at most before the viscoelastic relaxation drags the amplitude to zero. The Prony fit gives the solver that amplitude-versus-time shape for free through the `Viscoelastic` wrapper's history variables, which the [Part 5 Ch 00 backward-Euler integrator](../../../50-time-integration/00-backward-euler.md) evolves alongside the position variables.

The mechanism is that each Prony branch contributes a per-element internal-force term proportional to the branch's stored history strain, which tracks the strain's exponentially-weighted moving average over the branch's relaxation time. When the cavity is released from a load, the branches carry the recent-strain memory and subtract it from the instantaneous elastic force, damping the release oscillation on a timescale set by $\tau_i$. With $N = 3$ branches spanning $10^{-1}$ to $10^{1}$ seconds, the damping covers the natural-frequency band of the mesh the canonical problem discretizes the cavity at.

The wobble-failure regression test ([Part 11 Ch 04 regression-tests leaf](../../../110-crate/04-testing/01-regression.md)) sets the amplitude-decay envelope as a pass criterion, which fixes the baseline against which future spectrum refinements are measured.

## Autograd and re-mesh interaction

The Prony history variables live in the per-element data block alongside $F$ and the viscoelastic strain is recomputed each step from the previous step's history plus the current displacement. The [Part 7 Ch 04 warm-start machinery](../../../70-sdf-pipeline/04-live-remesh/01-warm-start.md) transfers these history variables across a re-mesh via the same per-element state-transfer interpolation that moves $F$ — history state is carried, not reset, so a topology edit does not reintroduce the wobble by clearing the relaxation memory.

On the autograd side the Prony update is smooth in the history state, so the gradient through a viscoelastic step is a straight application of the [Part 6 Ch 01 custom-VJP surface](../../../60-differentiability/01-custom-vjps.md). The [stochastic-adjoint deferral to Phase H](../../../60-differentiability/03-time-adjoint/02-stochastic.md) is about topology-change and adaptive-timestep-shrink events, not Prony evolution — the viscoelastic wrapper adds no FD-wrapper entry to the gradient pipeline.

## Alternatives considered

**Fractional-derivative viscoelasticity.** A single-parameter non-integer-order time derivative replaces the finite Prony sum. Silicone literature has used it to fit broad relaxation spectra with fewer parameters than an $N = 3$ Prony. Rejected as the default because the history-state representation is not finite — evaluating the fractional derivative requires retaining the full strain history, which conflicts with the per-element finite-memory data model the `Viscoelastic` wrapper uses. Reachable as a specialized `Material` for researchers willing to pay the memory cost.

**Elastic-only plus numerical damping.** Set the integrator's numerical damping high enough to match the measured decay envelope, omit the viscoelastic wrapper entirely. Rejected because numerical damping is frequency-indiscriminate — it damps the high-frequency numerical noise and the physical low-frequency ringing at the same rate, which means the quasi-static equilibrium shifts from the true equilibrium by an amount that scales with the damping. The Prony wrapper damps at the physical relaxation rate and converges to the measured quasi-static equilibrium; numerical damping does not.

**Per-grade independent DMA fits from day one.** Reject by budget, not by principle. A Pass-1 full-per-grade DMA dataset does not exist in the literature for Ecoflex; an in-house measurement is the Pass-3 deliverable. Shipping a single 00-30 fit with explicit transfer caveats is more honest than shipping four grade-labeled fits that are all the 00-30 numbers with the equilibrium modulus rescaled.
