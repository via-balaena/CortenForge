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
| $I_4$ | Anisotropic invariant, $I_4 = a \cdot C a$, with $a$ the reference-frame fiber direction | [Part 2 Ch 06](../20-materials/06-anisotropic.md) |
| $a$ | Fiber-direction unit vector (reference frame) | [Part 2 Ch 06](../20-materials/06-anisotropic.md) |
| $F_e, F_\theta$ | Mechanical / thermal parts of the multiplicative deformation split $F = F_e\, F_\theta$; isotropic $F_\theta = \beta(T)\, I$ | [Part 2 Ch 08](../20-materials/08-thermal-coupling/01-expansion.md) |
| $\beta(T)$ | Thermal stretch factor, $\beta(T) = 1 + \alpha\,(T - T_0)$ | [Part 2 Ch 08](../20-materials/08-thermal-coupling/01-expansion.md) |

## Canonical-problem geometry and reward machinery

| Symbol | Meaning | Introduced |
|---|---|---|
| $r_c$ | Cavity inner radius | [Part 1 Ch 00](../10-physical/00-canonical/00-formulation.md) |
| $r_p$ | Probe outer radius | [Part 1 Ch 00](../10-physical/00-canonical/00-formulation.md) |
| $t_c$ | Cavity wall thickness | [Part 1 Ch 00](../10-physical/00-canonical/00-formulation.md) |
| $L_c$ | Cavity axial length | [Part 1 Ch 00](../10-physical/00-canonical/00-formulation.md) |
| $\delta$ | Prescribed axial engagement depth (probe into cavity) | [Part 1 Ch 00](../10-physical/00-canonical/00-formulation.md) |
| $p_\text{th}$ | Pressure threshold for the uniformity reward's engagement window | [Part 1 Ch 01](../10-physical/01-reward/00-pressure-uniformity.md) |
| $\beta_w, \beta_c$ | Logistic-function sharpness parameters for the uniformity weight $w(p)$ and coverage indicator $c(x)$ respectively; both gate on the shared threshold $p_\text{th}$ | [Part 1 Ch 01](../10-physical/01-reward/00-pressure-uniformity.md) |
| $p_\text{max}$ | Peak-pressure ceiling (per-material, read from tensile strength) | [Part 1 Ch 01](../10-physical/01-reward/02-peak-bounds.md) |
| $\hat m$ | Peak-pressure barrier tolerance (margin at which the barrier engages) | [Part 1 Ch 01](../10-physical/01-reward/02-peak-bounds.md) |
| $\hat m_k$ | Stiffness-bound barrier tolerance | [Part 1 Ch 01](../10-physical/01-reward/03-stiffness-bounds.md) |
| $k_\text{eff}$ | Effective cavity stiffness, $k_\text{eff} = F_\text{ax}/\delta$ | [Part 1 Ch 01](../10-physical/01-reward/03-stiffness-bounds.md) |
| $k_\text{min}$ | Stiffness floor below which the stiffness barrier engages | [Part 1 Ch 01](../10-physical/01-reward/03-stiffness-bounds.md) |

## Material parameters

| Symbol | Meaning | Introduced |
|---|---|---|
| $E$ (scalar) | Young's modulus. Distinguished from the Green strain tensor by context | [Part 1 Ch 04](../10-physical/04-material-data.md) |
| $\nu$ | Poisson ratio | [Part 1 Ch 04](../10-physical/04-material-data.md) |
| $\mu, \lambda$ | Lamé parameters (neo-Hookean) | [Part 2 Ch 02](../20-materials/02-linear.md) |
| $C_{10}, C_{01}$ | Mooney-Rivlin coefficients | [Part 2 Ch 04 — Mooney-Rivlin](../20-materials/04-hyperelastic/01-mooney-rivlin.md) |
| $\{(\mu_i, \alpha_i)\}_{i=1}^{N}$ | Ogden parameters — $N$-term principal-stretch form | [Part 2 Ch 04 — Ogden](../20-materials/04-hyperelastic/02-ogden.md) |
| $k_1, k_2$ | Holzapfel-Gasser-Ogden fiber parameters | [Part 2 Ch 06](../20-materials/06-anisotropic/01-hgo.md) |
| $(\tau_i, g_i)$ | Prony series — relaxation time and over-stress weight for mode $i$ | [Part 2 Ch 07](../20-materials/07-viscoelastic.md) |
| $G_i$ | Oldroyd-B per-mode over-stress shear modulus, $G_i = g_i\,\mu_\text{base}$ | [Part 2 Ch 07 — Oldroyd-B](../20-materials/07-viscoelastic/01-oldroyd.md) |
| $G^\ast(\omega) = G'(\omega) + i\,G''(\omega)$ | DMA complex shear modulus (storage + loss) as a function of angular frequency | [Part 2 Ch 07 — DMA](../20-materials/07-viscoelastic/03-dma.md) |
| $G_\infty$ | DMA equilibrium (long-time) shear modulus; equivalent to the wrapped base material's Lamé $\mu$ at the small-strain equilibrium limit | [Part 2 Ch 07 — DMA](../20-materials/07-viscoelastic/03-dma.md) |
| $\alpha$ (thermal) | Thermal expansion coefficient. Distinguished from Ogden $\alpha_i$ by subscript context | [Part 2 Ch 08](../20-materials/08-thermal-coupling/01-expansion.md) |
| $T, T_0$ | Temperature, reference temperature | [Part 2 Ch 08](../20-materials/08-thermal-coupling.md) |
| $c_\mu, c_\lambda$ | Modulus temperature coefficients — $\mu(T) = \mu_0\,(1 - c_\mu\,(T - T_0))$, same form for $\lambda$ | [Part 2 Ch 08](../20-materials/08-thermal-coupling/00-modulus-T.md) |
| $\rho$ | Mass density | [Part 1 Ch 04](../10-physical/04-material-data.md) |
| $\sigma_e$ | Electrical conductivity, S/m. Subscripted to avoid collision with Cauchy stress $\sigma$ | [Part 1 Ch 04 — carbon-black conductivity](../10-physical/04-material-data/02-carbon-black/01-conductivity.md) |

## Stress, energy, and contact

| Symbol | Meaning | Introduced |
|---|---|---|
| $\Psi(F)$ | Hyperelastic energy density | [Part 1 Ch 03](../10-physical/03-thesis.md) |
| $\psi$ | Per-element energy density (lower-case variant used in Ogden and HGO sections) | [Part 2 Ch 04](../20-materials/04-hyperelastic.md) |
| $P$ | First Piola-Kirchhoff stress, $P = \partial \Psi / \partial F$ | [Part 2 Ch 00](../20-materials/00-trait-hierarchy.md) |
| $\sigma$ | Cauchy stress | [Part 2 Ch 08](../20-materials/08-thermal-coupling/02-dissipation.md) |
| $Q_i$ | Per-mode hereditary over-stress tensor (Prony / Oldroyd-B). Per-element $3 \times 3$ history state | [Part 2 Ch 07 — Prony](../20-materials/07-viscoelastic/00-prony.md) |
| $\alpha_\text{visc}(\Delta t)$ | Viscoelastic tangent multiplier; scales the base tangent in Prony-variant Newton assembly, $\alpha_\text{visc} = 1 + \sum_i g_i (1 - e^{-\Delta t / \tau_i})/(\Delta t / \tau_i)$ | [Part 2 Ch 07 — tangent](../20-materials/07-viscoelastic/02-tangent.md) |
| $\dot q_i$ | Per-mode viscoelastic dissipation rate, $\dot q_i = (Q_i : Q_i)/(G_i \tau_i)$ | [Part 2 Ch 08 — dissipation](../20-materials/08-thermal-coupling/02-dissipation.md) |
| $\text{Wi}$ | Weissenberg number, $\text{Wi} = \tau\,\dot\gamma$; ratio of relaxation time to shear rate | [Part 2 Ch 07 — Oldroyd-B](../20-materials/07-viscoelastic/01-oldroyd.md) |
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
| $\mathbf{p}_e$ | Per-element material-parameter vector (the scalar-valued sampling of `MaterialField` at element $e$'s Gauss point) | [Part 2 Ch 09](../20-materials/09-spatial-fields.md) |
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

- $E$ is Young's modulus in Part 1 Ch 04 and the material-property tables; Green strain in Part 2 Ch 01's strain-measure discussion. Where both appear in close neighbourhood — notably [Part 2 Ch 02](../20-materials/02-linear.md) — the Green strain tensor is written bolded $\mathbf{E}$ to disambiguate.
- $\alpha$ is Ogden's principal-stretch exponent $\alpha_i$, the thermal expansion coefficient in Part 2 Ch 08, the anisotropic-GGX roughness in Part 9 Ch 01, the XPBD compliance in the Part 5 Ch 01 rejection discussion, the BayesOpt acquisition-function name in Part 10 Ch 02, and the viscoelastic tangent multiplier $\alpha_\text{visc}(\Delta t)$ in Part 2 Ch 07. Subscripts (`_i` for Ogden mode, `_\text{visc}` for the viscoelastic multiplier, `_x / _y` for GGX tangent axes) plus surrounding context disambiguate.
- $\beta$ is the thermal stretch factor $\beta(T) = 1 + \alpha(T - T_0)$ in Part 2 Ch 08 expansion, the logistic-sharpness parameters $\beta_w, \beta_c$ in the Part 1 Ch 01 uniformity and coverage reward weights, and occasionally the XPBD damping coefficient in the Part 5 Ch 01 rejection discussion. Subscripts and function-argument form disambiguate ($\beta(T)$ with temperature argument is the thermal factor; subscripted $\beta_w / \beta_c$ are logistic-sharpness parameters).
- $d$ is the design-space dimension in Part 10 and the contact-proximity gap in Part 4. Disambiguated by neighbourhood — `$d \in \mathbb{R}^d$` is avoided; where both appear near each other, the former is written as $n_\theta$.
- $\mu$ is the neo-Hookean Lamé parameter in Part 2 and the Coulomb friction coefficient in Part 4. Friction always carries the subscript $\mu_c$ in `sim-soft`'s prose to avoid collision. In DMA at the small-strain equilibrium limit, the DMA equilibrium modulus $G_\infty$ equals the wrapped base material's Lamé $\mu$ — same value, different symbols by convention (elastic-continuum vs. rheology).
- $\lambda$ is the neo-Hookean Lamé parameter, the time-adjoint variable $\lambda(t)$, and occasionally a Lagrange multiplier in XPBD-rejection contexts. Context and function notation disambiguate.
- $\sigma$ is Cauchy stress everywhere in Parts 2–6 ($\sigma = \mathbb{C}:\varepsilon$ in [Part 2 Ch 02](../20-materials/02-linear.md), $\sigma = P F^T / J$ in the hyperelastic family). Electrical conductivity in the Part 1 Ch 04 carbon-black family and any downstream Part 2 Ch 09 spatial-field that assigns it always carries the subscript $\sigma_e$. The Part 9 rendering coefficients $\sigma_s$ (scattering) and $\sigma_a$ (absorption) are already subscripted per the SSS literature and do not collide with stress.
- $c_\mu$ and $c_\lambda$ are the Part 2 Ch 08 modulus-temperature coefficients (dimensionless, per kelvin) — distinct from $\beta(T)$ (the multiplicative thermal stretch factor, also in Ch 08), which carries temperature as a function argument rather than as a coefficient. The letter $c$ here is local to the modulus-curve formula and does not collide with any other $c$ in the book (no free $c$ introduced elsewhere).
- $G$ appears as $G^\ast, G', G'', G_\infty, G_i$ in the DMA / Oldroyd-B machinery of Part 2 Ch 07. $G$ without subscript is not used in the book; all $G$-symbols carry a subscript or superscript. The equilibrium relationship $G_\infty = \mu_\text{base}$ at the small-strain limit is the cross-notation bridge between the rheology and elastic-Lamé conventions.

## Pass 3 scope

The Pass 3 pass will add symbols introduced in the leaf chapters Pass 1 has not yet written — e.g., the per-pair IPC active-set indicator, the SSS screen-space convolution kernel, the GP kernel hyperparameters — and will standardize subscripting conventions where Pass 1 was informal.
