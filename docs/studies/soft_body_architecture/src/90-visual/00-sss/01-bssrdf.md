# BSSRDF basics

The bidirectional scattering-surface reflectance distribution function (BSSRDF) is the mathematical object every real-time SSS technique approximates. A standard BRDF assumes light that enters a surface exits at the same point; the BSSRDF removes that assumption, letting the entry point $x_i$ and the exit point $x_o$ be distinct. This sub-leaf sketches the formalism, the radiative-transport setup it inherits, the diffusion approximation that reduces it to a tractable 1D quantity, and the two specific parameterizations `sim-soft` links against: the [Jensen et al. 2001](../../appendices/00-references/04-rendering.md#jensen-2001) dipole model and the [Christensen & Burley 2015](../../appendices/00-references/04-rendering.md#christensen-burley-2015) sum-of-exponentials empirical form. Nothing here writes an offline path-tracer; the formalism is present so that the three real-time tiers in [§2](02-realtime.md) are honest approximations of a well-defined ground truth rather than folklore.

## Formal definition

The BSSRDF is an 8-dimensional scalar function

$$S(x_i, \omega_i;\, x_o, \omega_o)$$

specifying the fraction of radiance incident at surface point $x_i$ from direction $\omega_i$ that emerges at surface point $x_o$ in direction $\omega_o$. The BRDF is the special case $S(x_i, \omega_i;\, x_o, \omega_o) = f_r(\omega_i, \omega_o)\, \delta(x_i - x_o)$: the exit point is pinned to the entry point, and the distributional factor kills the spatial dimensions. For translucent materials — silicone, skin, marble, wax — the delta is the wrong model, and the full 8D form matters.

The outgoing radiance at a surface point is the surface integral of the BSSRDF against the incident radiance field:

$$L_o(x_o, \omega_o) = \int_A \int_{\Omega} S(x_i, \omega_i;\, x_o, \omega_o)\, L_i(x_i, \omega_i)\, (\omega_i \cdot n_i)\, d\omega_i\, dA_i$$

where $A$ is the material's bounding surface, $\Omega$ is the upper hemisphere at each entry point, and $n_i$ is the surface normal at $x_i$. The integrand vanishes for pairs $(x_i, x_o)$ too far apart for scattering to carry light between them, so the effective support of the integral is a small neighbourhood around $x_o$ — the mean free path of the material, typically 0.1 to a few millimeters for soft silicones.

Evaluating this integral directly, per pixel, per frame, is intractable: the 8D function is not known in closed form for real materials and the quadrature over $A$ has no cheap structure. Every real-time SSS technique reduces it, and the reduction is the content of the diffusion approximation below.

## The diffusion approximation

For a material with scattering coefficient $\sigma_s$, absorption coefficient $\sigma_a$, and Henyey-Greenstein anisotropy parameter $g \in [-1, 1]$, define the reduced scattering coefficient $\sigma_s' = \sigma_s (1 - g)$ and the transport coefficient $\sigma_t' = \sigma_s' + \sigma_a$. When $\sigma_s' \gg \sigma_a$ — the "highly scattering" regime that describes silicone in the visible spectrum — the multiple-scattered component of the BSSRDF is well-approximated by the diffusion equation

$$D\, \nabla^2 \phi(x) - \sigma_a\, \phi(x) = -S(x)$$

for the interior fluence $\phi$, with $D = 1/(3\sigma_t')$ the diffusion constant and $S$ the point-source forcing at the entry point. Solutions to this equation for a half-space geometry with a point source give the [Jensen et al. 2001](../../appendices/00-references/04-rendering.md#jensen-2001) dipole model: a real source at depth $z_r = 1/\sigma_t'$ below the entry point plus a virtual image source above the surface positioned and signed to satisfy the diffusion boundary condition, with the surface-exit radial distribution $R_d(r)$ expressed as a dipole sum of two contributions (real + signed image) each of the form $z(1 + \sigma_{tr} d) e^{-\sigma_{tr} d}/d^3$, where $\sigma_{tr} = \sqrt{3 \sigma_a \sigma_t'}$ is the effective transport coefficient and $d$ is the distance from source to surface-point. The closed-form expression is published in Jensen 2001; the book does not reproduce the specific coefficients since they are not load-bearing for the rest of Part 9.

The key structural takeaway for Part 9: the 8D BSSRDF reduces, under the diffusion approximation on a locally-planar surface, to a **1D radially-symmetric function $R_d(r)$** of the entry-to-exit surface distance. The full multi-bounce light transport is absorbed into that single 1D curve, and every tier-2 real-time technique in [§2](02-realtime.md) is a screen-space convolution of the lit-surface radiance by $R_d(r)$.

## Why the dipole is not the end of the story

The dipole is a closed-form solution to the diffusion equation on a half-space — two assumptions that silicone samples at canonical scale (1–20 mm thick) violate mildly but visibly. The geometric half-space assumption fails near rims, thin features, and layer interfaces; the diffusion-equation assumption fails for the first one to two mean free paths where single-scattering dominates and multiple-scattering has not had time to isotropize. In practice the dipole predicts a profile that is too flat near $r = 0$ and drops off too quickly at intermediate $r$.

The [Christensen & Burley 2015](../../appendices/00-references/04-rendering.md#christensen-burley-2015) improved form fits $R_d(r)$ empirically to Monte Carlo ground-truth as a sum of two exponentials per color channel, parameterized by a single mean-free-path shape factor and a normalization. The two-exponential form captures the steeper near-$r=0$ peak from single scattering plus the broader diffusion tail, without a material-specific fit per new silicone. `sim-soft`'s `DiffusionProfile` stores the Christensen-Burley shape coefficients by default; [§0 stacked](00-stacked.md)'s multi-layer composition refits to a sum-of-Gaussians approximation at scene-ingest time, since Gaussian profiles compound via variance addition while the exponential form does not.

## Measurement protocol

The $R_d(r)$ curve is a physical quantity, measurable on flat material coupons. The standard protocol:

1. Cure a flat coupon of the target material, thickness at least 5× the expected mean free path, area at least 10× the expected profile width. Soft silicones like the [Ecoflex family](../../10-physical/04-material-data/00-ecoflex.md) typically need a centimeter-scale coupon several millimeters thick.
2. Illuminate a small region (<0.5 mm diameter) at the center with a narrow-band source per color channel (typical visible-spectrum wavelengths; collimated lasers are the standard choice but calibrated LEDs work).
3. Image the backscattered surface radiance with a calibrated camera; integrate radially to extract the 1D $R_d(r)$ curve per channel.
4. Fit either the dipole form or the sum-of-exponentials form; store the fit coefficients in the [appendix material database](../../appendices/02-material-db.md).

The protocol is the visual-side measurement companion to the [Part 1 Ch 04 material-data](../../10-physical/04-material-data.md) DMA and uniaxial-tension protocols; coupons from the same print batch carry all three measurements. Practical deployment is on the [Part 10 Ch 05 sim-to-real loop](../../100-optimization/05-sim-to-real.md)'s per-batch calibration sequence; the per-channel `diffusion_gp` residual GP trained from this protocol is what closes the visual half of the [Part 1 Ch 03 thesis](../../10-physical/03-thesis.md).

## What this sub-leaf commits the book to

- **BSSRDF is the formal ground truth; real-time techniques approximate it.** The 8D function reduces under diffusion on a planar surface to a 1D radial profile $R_d(r)$, and that is what every [§2 real-time tier](02-realtime.md) convolves against.
- **`sim-soft`'s stored profile shape is [Christensen & Burley 2015](../../appendices/00-references/04-rendering.md#christensen-burley-2015) sum-of-exponentials by default**, with optional sum-of-Gaussians for measured materials (composes with [§0 stacked](00-stacked.md)'s variance-addition rule).
- **Measurement is via backscatter imaging of laser-illuminated coupons per color channel.** Fit coefficients land in the [appendix material database](../../appendices/02-material-db.md); per-batch recalibration drives the [Part 10 Ch 05](../../100-optimization/05-sim-to-real.md) visual-side residual GP.
- **The diffusion approximation is mildly wrong at rims, thin features, and the first mean free path.** `sim-soft` does not correct it at the physics level; the [§2 Phase I](02-realtime.md) shader approximation inherits the same caveat.
