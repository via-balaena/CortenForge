# Notation reference

Symbol table for the notation used across Parts 1–12. One line per symbol — definition plus the chapter that introduces or most fully treats it. Where a symbol is inherited from the continuum-mechanics or adjoint-method literature without remark, the "introduced in" pointer names the first `sim-soft` chapter that uses it.

## Kinematics and strain measures

| Symbol | Meaning | Introduced |
|---|---|---|
| $F$ | Deformation gradient — $3 \times 3$ matrix, $F = \partial x / \partial X$ | [Part 2 Ch 01](../20-materials/01-strain-measures/00-F.md) |
| $J$ | Volume ratio, $J = \det F$ | [Part 2 Ch 01](../20-materials/01-strain-measures.md) |
| $C$ | Right Cauchy-Green tensor, $C = F^T F$ | [Part 2 Ch 01](../20-materials/01-strain-measures/01-C.md) |
| $E$ (tensor) | Green strain tensor, $E = \tfrac{1}{2}(C - I)$ | [Part 2 Ch 01](../20-materials/01-strain-measures/02-E.md) |
| $\varepsilon$ | Small-strain tensor | [Part 2 Ch 02](../20-materials/02-linear.md) |
| $I_1, I_2, I_3$ | Strain invariants of $C$ | [Part 2 Ch 01](../20-materials/01-strain-measures/03-invariants.md) |
| $I_4$ | Anisotropic invariant, $I_4 = a \cdot C a$ | [Part 2 Ch 06](../20-materials/06-anisotropic.md) |
| $a$ | Fiber-direction unit vector | [Part 2 Ch 06](../20-materials/06-anisotropic.md) |

## Material parameters

| Symbol | Meaning | Introduced |
|---|---|---|
| $E$ (scalar) | Young's modulus. Distinguished from the Green strain tensor by context | [Part 1 Ch 04](../10-physical/04-material-data.md) |
| $\nu$ | Poisson ratio | [Part 1 Ch 04](../10-physical/04-material-data.md) |
| $\mu, \lambda$ | Lamé parameters (neo-Hookean) | [Part 2 Ch 02](../20-materials/02-linear.md) |
| $C_{10}, C_{01}$ | Mooney-Rivlin coefficients | [Part 2 Ch 04 — Mooney-Rivlin](../20-materials/04-hyperelastic/01-mooney-rivlin.md) |
| $\{(\mu_i, \alpha_i)\}_{i=1}^{N}$ | Ogden parameters — $N$-term principal-stretch form | [Part 2 Ch 04 — Ogden](../20-materials/04-hyperelastic/02-ogden.md) |
| $k_1, k_2$ | Holzapfel-Gasser-Ogden fiber parameters | [Part 2 Ch 06](../20-materials/06-anisotropic/01-hgo.md) |
| $(\tau_i, g_i)$ | Prony series — relaxation time and weight for mode $i$ | [Part 2 Ch 07](../20-materials/07-viscoelastic.md) |
| $\alpha$ (thermal) | Thermal expansion coefficient. Distinguished from Ogden $\alpha_i$ by subscript context | [Part 2 Ch 08](../20-materials/08-thermal-coupling/01-expansion.md) |
| $T, T_0$ | Temperature, reference temperature | [Part 2 Ch 08](../20-materials/08-thermal-coupling.md) |
| $\rho$ | Mass density | [Part 1 Ch 04](../10-physical/04-material-data.md) |

## Stress, energy, and contact

| Symbol | Meaning | Introduced |
|---|---|---|
| $\Psi(F)$ | Hyperelastic energy density | [Part 1 Ch 03](../10-physical/03-thesis.md) |
| $\psi$ | Per-element energy density (lower-case variant used in Ogden and HGO sections) | [Part 2 Ch 04](../20-materials/04-hyperelastic.md) |
| $P$ | First Piola-Kirchhoff stress, $P = \partial \Psi / \partial F$ | [Part 2 Ch 00](../20-materials/00-trait-hierarchy.md) |
| $\sigma$ | Cauchy stress | [Part 2 Ch 08](../20-materials/08-thermal-coupling/02-dissipation.md) |
| $U(x; \theta)$ | Total potential energy | [Part 5 Ch 00](../50-time-integration/00-backward-euler.md) |
| $b(d)$ | IPC barrier energy, $b(d) = -(d - \hat d)^2 \ln(d/\hat d)$ | [Part 4 Ch 01](../40-contact/01-ipc-internals/00-barrier.md) |
| $\hat d$ | IPC barrier-width tolerance (per-pair, adaptive) | [Part 4 Ch 05](../40-contact/05-real-time/01-barrier-width.md) |
| $d$ | Proximity gap between primitives | [Part 4 Ch 01](../40-contact/01-ipc-internals.md) |
| $\kappa$ | IPC adaptive barrier stiffness | [Part 4 Ch 01](../40-contact/01-ipc-internals/01-adaptive-kappa.md) |
| $\mu_c$ | Coulomb friction coefficient | [Part 4 Ch 02](../40-contact/02-friction.md) |

## Solver, time integration, and optimization

| Symbol | Meaning | Introduced |
|---|---|---|
| $x$ | State vector (per-vertex positions) | [Part 5 Ch 00](../50-time-integration/00-backward-euler.md) |
| $x^\ast$ | Converged equilibrium state | [Part 5 Ch 00](../50-time-integration/00-backward-euler.md) |
| $r(x, x_{t-1}; \theta)$ | Residual of the step-$t$ energy-minimization problem | [Part 6 Ch 03](../60-differentiability/03-time-adjoint.md) |
| $K$ | Global sparse stiffness matrix | [Part 5 Ch 00](../50-time-integration/00-backward-euler.md) |
| $B$ | Strain-displacement matrix (per element) | [Part 6 Ch 01](../60-differentiability/01-custom-vjps.md) |
| $\Delta t$ | Timestep | [Part 5 Ch 02](../50-time-integration/02-adaptive-dt.md) |
| $\lambda(t)$ | Time-adjoint state variable | [Part 6 Ch 03](../60-differentiability/03-time-adjoint/01-adjoint-state.md) |
| $\theta$ | Design-parameter vector | [Part 10 Ch 00](../100-optimization/00-forward.md) |
| $d$ (dim) | Dimension of $\theta$. Distinguished from proximity-gap $d$ by context | [Part 10 Ch 00](../100-optimization/00-forward.md) |
| $n_\theta$ | Number of design parameters; synonym for $d$ when parameter counts are discussed alongside gap-distance $d$ | [Part 6 Ch 03](../60-differentiability/03-time-adjoint.md) |
| $R(\theta)$ | Scalar reward — the forward-map output | [Part 1 Ch 01](../10-physical/01-reward.md), formalized in [Part 10 Ch 00](../100-optimization/00-forward.md) |
| $L$ | Loss used in autograd-facing sections; equal to $-R$ when minimizing | [Part 6 Ch 00](../60-differentiability/00-what-autograd-needs.md) |
| $w_i$ | Reward-composition weights | [Part 1 Ch 01 — composition](../10-physical/01-reward/04-composition.md) |
| $\Phi$ | Terminal cost in trajectory-adjoint discussions | [Part 6 Ch 03](../60-differentiability/03-time-adjoint.md) |

## Rendering and visualization

| Symbol | Meaning | Introduced |
|---|---|---|
| $\sigma_s$ | Volumetric scattering coefficient | [Part 9 Ch 00](../90-visual/00-sss.md) |
| $\sigma_a$ | Volumetric absorption coefficient | [Part 9 Ch 00](../90-visual/00-sss.md) |
| $g$ | Henyey-Greenstein scattering anisotropy parameter | [Part 9 Ch 00](../90-visual/00-sss.md) |
| $\alpha_x, \alpha_y$ | Anisotropic GGX roughness along tangent axes | [Part 9 Ch 01](../90-visual/01-anisotropic-reflection.md) |

## Notes on collisions and context

Several letters carry multiple meanings across Parts — the book disambiguates by context:

- $E$ is Young's modulus in Part 1 Ch 04 and the material-property tables; Green strain in Part 2 Ch 01's strain-measure discussion.
- $\alpha$ is Ogden's principal-stretch exponent $\alpha_i$, the thermal expansion coefficient in Part 2 Ch 08, the anisotropic-GGX roughness in Part 9 Ch 01, the XPBD compliance in the Part 5 Ch 01 rejection discussion, and the BayesOpt acquisition-function name in Part 10 Ch 02. Subscripts and surrounding context disambiguate.
- $d$ is the design-space dimension in Part 10 and the contact-proximity gap in Part 4. Disambiguated by neighbourhood — `$d \in \mathbb{R}^d$` is avoided; where both appear near each other, the former is written as $n_\theta$.
- $\mu$ is the neo-Hookean Lamé parameter in Part 2 and the Coulomb friction coefficient in Part 4. Friction always carries the subscript $\mu_c$ in `sim-soft`'s prose to avoid collision.
- $\lambda$ is the neo-Hookean Lamé parameter, the time-adjoint variable $\lambda(t)$, and occasionally a Lagrange multiplier in XPBD-rejection contexts. Context and function notation disambiguate.

## Pass 3 scope

The Pass 3 pass will add symbols introduced in the leaf chapters Pass 1 has not yet written — e.g., the per-pair IPC active-set indicator, the SSS screen-space convolution kernel, the GP kernel hyperparameters — and will standardize subscripting conventions where Pass 1 was informal.
