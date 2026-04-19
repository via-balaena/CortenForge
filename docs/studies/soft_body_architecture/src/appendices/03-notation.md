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
| $\sigma^e$ | Per-tet (per-element) stress tensor; the $\sigma$ value evaluated at element $e$. Superscript $e$ paralleling $K^e, V^e, f^e_\text{int}$. Distinguished from electrical conductivity $\sigma_e$ by superscript-vs-subscript | [Part 7 Ch 03 — adaptive refine](../70-sdf-pipeline/03-adaptive-refine.md) |
| $Q_i$ | Per-mode hereditary over-stress tensor (Prony / Oldroyd-B). Per-element $3 \times 3$ history state | [Part 2 Ch 07 — Prony](../20-materials/07-viscoelastic/00-prony.md) |
| $\alpha_\text{visc}(\Delta t)$ | Viscoelastic tangent multiplier; scales the base tangent in Prony-variant Newton assembly, $\alpha_\text{visc} = 1 + \sum_i g_i (1 - e^{-\Delta t / \tau_i})/(\Delta t / \tau_i)$ | [Part 2 Ch 07 — tangent](../20-materials/07-viscoelastic/02-tangent.md) |
| $\dot q_i$ | Per-mode viscoelastic dissipation rate, $\dot q_i = (Q_i : Q_i)/(G_i \tau_i)$ | [Part 2 Ch 08 — dissipation](../20-materials/08-thermal-coupling/02-dissipation.md) |
| $\text{Wi}$ | Weissenberg number, $\text{Wi} = \tau\,\dot\gamma$; ratio of relaxation time to shear rate | [Part 2 Ch 07 — Oldroyd-B](../20-materials/07-viscoelastic/01-oldroyd.md) |
| $U(x; \theta)$ | Total potential energy — the step-minimization objective in Part 5 Ch 00, including elastic + contact + backward-Euler inertial quadratic contributions per [Ch 00 §01 newton](../50-time-integration/00-backward-euler/01-newton.md) | [Part 5 Ch 00](../50-time-integration/00-backward-euler.md) |
| $U^\text{phys}$ | Physical potential energy (elastic strain + contact-barrier only) — distinct from $U$ in that it excludes the backward-Euler inertial quadratic. Load-bearing for [Ch 02 §00 energy](../50-time-integration/02-adaptive-dt/00-energy.md)'s conservation-ledger invariant | [Part 5 Ch 02 §00](../50-time-integration/02-adaptive-dt/00-energy.md) |
| $b(d)$ | IPC barrier energy, $b(d) = -(d - \hat d)^2 \ln(d/\hat d)$ | [Part 4 Ch 01](../40-contact/01-ipc-internals/00-barrier.md) |
| $\hat d$ | IPC barrier-width tolerance. Appears as $\hat d_k$ (per-pair), $\hat d_v$ (per-primitive — vertex, edge, face), or $\hat d_\text{global}$ (scene-level default upper bound); `sim-soft` aggregates $\hat d_k = \min(\hat d_{v_a}, \hat d_{v_b})$. Adaptive schedule in [Ch 05 §01](../40-contact/05-real-time/01-barrier-width.md) | [Part 4 Ch 01 barrier](../40-contact/01-ipc-internals/00-barrier.md) |
| $d$ | Proximity gap between primitives | [Part 4 Ch 01](../40-contact/01-ipc-internals.md) |
| $\kappa$ | IPC adaptive barrier stiffness | [Part 4 Ch 01](../40-contact/01-ipc-internals/01-adaptive-kappa.md) |
| $\mu_c$ | Coulomb friction coefficient | [Part 4 Ch 02](../40-contact/02-friction.md) |
| $\ell$ | Layer thickness in multi-layer contact configurations; per-layer subscripted as $\ell_k$ | [Part 4 Ch 04 — thin-material](../40-contact/04-multi-layer/02-thin-material.md) |

## Discretization and mesh quality (Part 3)

| Symbol | Meaning | Introduced |
|---|---|---|
| $N_i$ | FEM shape function for node $i$ | [Part 3 Ch 00 — Tet4](../30-discretization/00-element-choice/00-tet4.md) |
| $\xi, \eta, \zeta$ | Barycentric / reference-element coordinates on the unit tetrahedron | [Part 3 Ch 00 — Tet4](../30-discretization/00-element-choice/00-tet4.md) |
| $V^e$ | Element reference volume (signed; positive-orientation convention) | [Part 3 Ch 00 — Tet4](../30-discretization/00-element-choice/00-tet4.md) |
| $K^e$ | Per-element stiffness matrix; $4 \times 4$ block of $3 \times 3$ blocks for Tet4, $10 \times 10$ for Tet10 | [Part 3 Ch 00 — Tet4](../30-discretization/00-element-choice/00-tet4.md) |
| $\mathbb{C}$ | Material tangent stiffness — 4th-order tensor, $\mathbb{C} = \partial P / \partial F$ | [Part 3 Ch 00 — Tet4](../30-discretization/00-element-choice/00-tet4.md) |
| $f^e_\text{int}$ | Per-element internal force vector contribution to the global Newton residual | [Part 3 Ch 00 — Tet4](../30-discretization/00-element-choice/00-tet4.md) |
| $w_q$ | Gauss-quadrature weight at quadrature point $q$ | [Part 3 Ch 00 — Tet10](../30-discretization/00-element-choice/01-tet10.md) |
| $\rho$ (radius ratio) | Per-tet shape-quality metric, $\rho = r_\text{ins}/r_\text{circ}$. Distinguished from mass density $\rho$ by context — mesh-quality discussions never mention mass density | [Part 3 Ch 01 — aspect ratio](../30-discretization/01-mesh-quality/00-aspect-ratio.md) |
| $r_\text{ins}, r_\text{circ}$ | Inscribed-sphere and circumscribed-sphere radii of a tet | [Part 3 Ch 01 — aspect ratio](../30-discretization/01-mesh-quality/00-aspect-ratio.md) |
| $\theta_e$ | Dihedral angle on tetrahedron edge $e$; $\theta_\text{min}, \theta_\text{max}$ are the per-tet min/max across the 6 edges | [Part 3 Ch 01 — dihedral](../30-discretization/01-mesh-quality/01-dihedral.md) |
| $V_\text{min}$ | Volume-consistency lower bound at mesh ingest; expressed as a fraction of median tet volume | [Part 3 Ch 01 — volume](../30-discretization/01-mesh-quality/02-volume.md) |

## SDF pipeline (Part 7)

| Symbol | Meaning | Introduced |
|---|---|---|
| $\phi(p)$ | Signed distance field — scalar function with $\phi < 0$ inside the shape, $\phi > 0$ outside. Subscripted $\phi_a, \phi_b$ for operand SDFs; $\phi_\cup^k, \phi_\cap^k$ for smoothed-union/intersection with blend radius $k$; $\phi_T, \phi_\text{disp}$ for transformed and displacement-perturbed variants | [Part 7 Ch 00 §01 operations](../70-sdf-pipeline/00-sdf-primitive/01-operations.md) |
| $k$ (blend radius) | Smoothed-CSG blend radius in Part 7 Ch 00 §01. Disambiguated from HGO $k_1, k_2$ and stiffness-bound $k_\text{eff}, k_\text{min}$ by unit (length) and surrounding context (SDF composition) | [Part 7 Ch 00 §01 operations](../70-sdf-pipeline/00-sdf-primitive/01-operations.md) |

## Solver, time integration, and optimization

| Symbol | Meaning | Introduced |
|---|---|---|
| $x$ | State vector (per-vertex positions) | [Part 5 Ch 00](../50-time-integration/00-backward-euler.md) |
| $x^\ast$ | Converged equilibrium state | [Part 5 Ch 00](../50-time-integration/00-backward-euler.md) |
| $r(x, x_{t-1}; \theta)$ | Residual of the step-$t$ energy-minimization problem | [Part 6 Ch 03](../60-differentiability/03-time-adjoint.md) |
| $K$ | Global sparse stiffness matrix | [Part 5 Ch 00](../50-time-integration/00-backward-euler.md) |
| $H$ | Newton Hessian of the step objective, $H = K + H_\text{contact} + M/\Delta t^2$ — the object the forward Newton factors, the SPD object the IFT adjoint inverts | [Part 5 Ch 00 §01 Newton](../50-time-integration/00-backward-euler/01-newton.md) |
| $A$ | Residual Jacobian $A \equiv \partial r/\partial x$ in the implicit-function-theorem derivation. For the backward-Euler step, $A = H$ identically; the symbol choice is local to IFT exposition | [Part 6 Ch 02 §00 derivation](../60-differentiability/02-implicit-function/00-derivation.md) |
| $B$ | Strain-displacement matrix (per element) | [Part 6 Ch 01](../60-differentiability/01-custom-vjps.md) |
| $\Delta t$ | Timestep | [Part 5 Ch 02](../50-time-integration/02-adaptive-dt.md) |
| $\lambda(t)$ | Time-adjoint state variable | [Part 6 Ch 03](../60-differentiability/03-time-adjoint/01-adjoint-state.md) |
| $\theta$ | Design-parameter vector | [Part 10 Ch 00](../100-optimization/00-forward.md) |
| $d$ (dim) | Dimension of $\theta$. Distinguished from proximity-gap $d$ by context | [Part 10 Ch 00](../100-optimization/00-forward.md) |
| $n_\theta$ | Number of design parameters; synonym for $d$ when parameter counts are discussed alongside gap-distance $d$ | [Part 6 Ch 03](../60-differentiability/03-time-adjoint.md) |
| $\mathbf{p}_e$ | Per-element material-parameter vector (the scalar-valued sampling of `MaterialField` at element $e$'s Gauss point) | [Part 2 Ch 09](../20-materials/09-spatial-fields.md) |
| $R(\theta)$ | Scalar reward — the forward-map output | [Part 1 Ch 01](../10-physical/01-reward.md), formalized in [Part 10 Ch 00](../100-optimization/00-forward.md) |
| $L$ | Loss used in autograd-facing sections; equal to $-R$ when minimizing | [Part 6 Ch 00](../60-differentiability/00-what-autograd-needs.md) |
| $\bar x$ | Adjoint / upstream gradient in reverse-mode autograd — $\bar x = \partial L / \partial x$; the quantity a tape node's backward propagates backward through the graph | [Part 6 Ch 00](../60-differentiability/00-what-autograd-needs/00-generic.md) |
| $w_i$ | Reward-composition weights | [Part 1 Ch 01 — composition](../10-physical/01-reward/04-composition.md) |
| $\Phi$ | Terminal cost in trajectory-adjoint discussions | [Part 6 Ch 03](../60-differentiability/03-time-adjoint.md) |
| $M$ (preconditioner) | Preconditioner matrix, $M \approx H^{-1}$; applied per CG/MINRES iteration. Distinguished from the mass matrix $M$ in [Part 5 Ch 00 §01 Newton](../50-time-integration/00-backward-euler/01-newton.md) by context — preconditioner discussions are inside the iterative-solver inner loop, mass-matrix discussions are in the Newton-step objective | [Part 8 Ch 02 §00](../80-gpu/02-sparse-solvers/00-cg.md) |
| $\kappa$ (cond. #) | Matrix condition number, $\kappa = \lambda_\text{max}/\lambda_\text{min}$; CG iteration count is $O(\sqrt{\kappa})$. Distinguished from IPC adaptive barrier stiffness $\kappa$ (in Part 4 Ch 01) by context — solver-side discussions use κ for the matrix conditioning, contact-side for barrier stiffness | [Part 8 Ch 02 §00](../80-gpu/02-sparse-solvers/00-cg.md) |
| $\alpha_k, \beta_k$ (CG) | Hestenes-Stiefel CG scalar coefficients at iteration $k$; $\alpha_k$ is the step length, $\beta_k$ is the conjugation coefficient. Distinguished from the other $\alpha/\beta$ uses in the book (Ogden, thermal, logistic, XPBD) by iteration subscript and local-to-the-inner-loop context | [Part 8 Ch 02 §00](../80-gpu/02-sparse-solvers/00-cg.md) |
| $\theta$ (AMG) | Ruge-Stüben strong-connection threshold, $\theta \in [0.25, 0.5]$ typical; DOF $i$ is strongly connected to $j$ if $|A_{ij}| \geq \theta \max_{k \neq i} |A_{ik}|$. Distinguished from the design-parameter $\theta$ (Part 10) and the dihedral angle $\theta_e$ (Part 3) by local context and the accompanying absolute-value formulation | [Part 8 Ch 02 §02](../80-gpu/02-sparse-solvers/02-amg.md) |
| $\nu_\text{pre}, \nu_\text{post}$ | Pre- and post-smoothing iteration counts per AMG level; typically 2 for soft-body Hessians | [Part 8 Ch 02 §02](../80-gpu/02-sparse-solvers/02-amg.md) |

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
- $\lambda$ is the neo-Hookean Lamé parameter, the time-adjoint variable $\lambda(t)$, occasionally a Lagrange multiplier in XPBD-rejection contexts, and the normal-force magnitude $\lambda_k$ (or lagged $\lambda_k^n$) at contact pair $k$ in [Part 4 Ch 02 friction](../40-contact/02-friction.md) and Ch 02's integration sub-leaf. Context (pair subscript, lagged superscript, function argument) disambiguates.
- $\sigma$ is Cauchy stress everywhere in Parts 2–6 ($\sigma = \mathbb{C}:\varepsilon$ in [Part 2 Ch 02](../20-materials/02-linear.md), $\sigma = P F^T / J$ in the hyperelastic family). Electrical conductivity in the Part 1 Ch 04 carbon-black family and any downstream Part 2 Ch 09 spatial-field that assigns it always carries the subscript $\sigma_e$ (with $e$ for "electrical"). The per-tet (per-element) stress evaluated for the [Part 7 Ch 03 refinement criterion](../70-sdf-pipeline/03-adaptive-refine.md) carries the superscript form $\sigma^e$ (with $e$ for element index, paralleling $K^e, V^e, f^e_\text{int}$); superscript-vs-subscript on the same Greek letter is the disambiguation between per-element stress and electrical conductivity. The Part 9 rendering coefficients $\sigma_s$ (scattering) and $\sigma_a$ (absorption) are already subscripted per the SSS literature and do not collide with either.
- $c_\mu$ and $c_\lambda$ are the Part 2 Ch 08 modulus-temperature coefficients (dimensionless, per kelvin) — distinct from $\beta(T)$ (the multiplicative thermal stretch factor, also in Ch 08), which carries temperature as a function argument rather than as a coefficient. The letter $c$ here is local to the modulus-curve formula and does not collide with any other $c$ in the book (no free $c$ introduced elsewhere).
- $G$ appears as $G^\ast, G', G'', G_\infty, G_i$ in the DMA / Oldroyd-B machinery of Part 2 Ch 07. $G$ without subscript is not used in the book; all $G$-symbols carry a subscript or superscript. The equilibrium relationship $G_\infty = \mu_\text{base}$ at the small-strain limit is the cross-notation bridge between the rheology and elastic-Lamé conventions.
- $\rho$ is mass density everywhere except [Part 3 Ch 01 mesh-quality](../30-discretization/01-mesh-quality.md), where it is the radius ratio $r_\text{ins}/r_\text{circ}$. The two meanings do not co-occur in any chapter; mass-density discussions are about material properties (Part 1 Ch 04 and downstream) and radius-ratio discussions are about discretization-mesh topology (Part 3 Ch 01). Where they co-occur in a future cross-reference, the radius ratio gets the local subscript $\rho_\text{aspect}$.
- $\theta$ is the design-parameter vector (Part 10) without subscripts; $\theta_e$ is a dihedral angle on tet edge $e$ in [Part 3 Ch 01 — dihedral](../30-discretization/01-mesh-quality/01-dihedral.md). The subscripted edge-index disambiguates from the design-vector usage. Per-tet aggregates $\theta_\text{min}, \theta_\text{max}$ are clearly local to the mesh-quality chapter.
- $K$ is the global sparse stiffness matrix; $K^e$ is the per-element stiffness matrix from [Part 3 Ch 00 — Tet4](../30-discretization/00-element-choice/00-tet4.md). The superscript $e$ marks the per-element scope; assembly is the standard $K = \sum_e A^{eT} K^e A^e$ scatter-and-add.
- $k$ without subscript is the SDF smoothed-CSG blend radius from [Part 7 Ch 00 §01 operations](../70-sdf-pipeline/00-sdf-primitive/01-operations.md) (units of length). Subscripted $k$-family symbols are reserved for distinct concepts: $k_1, k_2$ are HGO fiber parameters, $k_\text{eff}$ is effective cavity stiffness, $k_\text{min}$ is the stiffness-bound floor. The blend radius and HGO parameters appear in different Parts (7 vs. 2); where they would co-occur in a cross-reference, the blend radius is local to an SDF-composition expression and the HGO parameters local to an anisotropic-material expression.
- $\mathbb{C}$ (blackboard-bold C) is the 4th-order material tangent stiffness $\partial P / \partial F$; $C$ (plain) is the right Cauchy-Green tensor $F^T F$. The two are visually distinct in rendered math and never substituted; LaTeX `\mathbb{C}` for the tangent and `C` for the strain measure.
- $M$ is the mass matrix in [Part 5 Ch 00's Newton-step Hessian decomposition](../50-time-integration/00-backward-euler.md) ($H = K + H_\text{contact} + M/\Delta t^2$) and the preconditioner $M \approx H^{-1}$ in [Part 8 Ch 02's iterative solver inner loop](../80-gpu/02-sparse-solvers/00-cg.md). The two usages do not co-occur in any single passage — the mass matrix appears in the Newton-step-objective derivation, the preconditioner appears inside the CG/MINRES inner loop that runs on that Hessian. Where they would need to co-occur in a future cross-reference, the preconditioner gets the local subscript $M_P$.
- $\kappa$ is the IPC adaptive barrier stiffness in [Part 4 Ch 01](../40-contact/01-ipc-internals/01-adaptive-kappa.md) and the matrix condition number in [Part 8 Ch 02](../80-gpu/02-sparse-solvers.md). Disambiguation is fully by context: barrier stiffness appears in contact/friction derivations, condition number appears in iterative-solver convergence analysis. Where they co-occur in a future Newton-step-cost discussion, the condition number gets the local name $\text{cond}(H)$ written out.

## Pass 3 scope

The Pass 3 pass will add symbols introduced in the leaf chapters Pass 1 has not yet written — e.g., the per-pair IPC active-set indicator, the SSS screen-space convolution kernel, the GP kernel hyperparameters — and will standardize subscripting conventions where Pass 1 was informal.
