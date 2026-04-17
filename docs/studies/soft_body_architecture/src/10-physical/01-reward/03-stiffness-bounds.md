# Effective stiffness bounds

The [peak-pressure barrier](02-peak-bounds.md) penalized pressures approaching a damaging ceiling. This leaf pins down the complementary barrier at the other end of the transmitted-force axis: a smooth penalty growing as the cavity's effective stiffness falls below a minimum. An infinitely compliant cavity achieves perfect pressure uniformity trivially (zero pressure everywhere is uniform) and does nothing useful; the stiffness barrier rules that out. The machinery mirrors the peak-pressure barrier, reflected across the stiffness axis.

## What it measures

The cavity's effective stiffness under the canonical load mode — how much axial force the cavity transmits to the probe per unit of prescribed engagement. Penalized as it falls below a material- and application-dependent minimum $k_\text{min}$.

Two load modes from [Part 1 Ch 00](../00-canonical/00-formulation.md) define the measure slightly differently:

- **Prescribed axial engagement.** Effective stiffness is the ratio of measured axial reaction force $F_\text{ax}$ to the held engagement depth $\delta$: $k_\text{eff} = F_\text{ax}/\delta$.
- **Prescribed axial force.** Effective stiffness is $k_\text{eff} = F_\text{ax}/\delta_\text{obs}$, with $\delta_\text{obs}$ the observed engagement at the prescribed load.

Both expose the same scalar — how stiff the cavity is as a mechanical transducer — and both feed the same barrier machinery below.

## Functional form

The barrier is an inverted IPC form applied to the margin $m = k_\text{eff} - k_\text{min}$, with tolerance $\hat m_k$:

$$ B_\text{stiff}(k_\text{eff}) \;=\; \begin{cases} -(m - \hat m_k)^2 \,\log(m / \hat m_k) & \text{if } 0 < m < \hat m_k \\ 0 & \text{if } m \ge \hat m_k \\ +\infty & \text{if } m \le 0 \end{cases} $$

Same $C^2$ IPC shape as [peak-pressure barrier](02-peak-bounds.md), reflected onto the stiffness axis. The barrier vanishes smoothly when the cavity transmits well above $k_\text{min}$ and diverges as $k_\text{eff} \to k_\text{min}^+$. Below $k_\text{min}$ the configuration is infeasible.

The reward contribution is $R_\text{stiff} = -B_\text{stiff}$, passed into [composition](04-composition.md) with its weight.

## Why this form

Three choices matter.

**Effective stiffness as a scalar, not as a curve.** A cavity under a varying engagement sweep has a force–displacement curve, not a single stiffness number. The barrier reads the scalar at a single representative operating point — either the held steady state (prescribed-displacement mode) or the equilibrium under the prescribed force (prescribed-force mode). A curve-based reward would have to handle the whole trajectory, couple to viscoelastic relaxation ([Part 2 Ch 07](../../20-materials/07-viscoelastic.md)), and multiply the per-evaluation cost; this book defers curve-matching to the [sim-to-real correction](../../100-optimization/05-sim-to-real.md) where it belongs, not to the reward itself.

**Same IPC-shape barrier as peak-pressure.** The argument is identical to [Claim 2 of the parent](../01-reward.md): hard constraints encoded as smooth barriers, diverging at the constraint, vanishing away from it, $C^2$ everywhere the optimizer visits. Symmetry with the peak-pressure barrier is not cosmetic; it means the composition can be Newton-minimized with a single inner solver that handles both barriers identically.

**Minimum $k_\text{min}$ from application-level requirement, not material tensile/yield limits.** The peak-pressure ceiling is a material-limited quantity (tensile strength). The stiffness floor is application-limited — a seal that cannot hold line pressure is useless, a gripper that cannot retain its object is useless. The minimum enters the reward as a design specification, per-application; the canonical problem's baseline value is documented in the [composition sub-chapter](04-composition.md) and can be overridden in the domain user's `SoftScene`.

## What the solver exports

The term reads one scalar at a converged step: the net axial force on the rigid probe. `sim-soft` computes this at the probe contact partner as the sum of the IPC barrier's per-pair traction contributions in the axial direction; the rigid-body coupling in [Part 5 Ch 03](../../50-time-integration/03-coupling.md) makes it available to the reward-evaluation callback without additional solver passes.

Engagement depth $\delta$ is a scene-level scalar read directly from the prescribed-motion trajectory (displacement-control mode) or computed from the probe's converged pose (force-control mode).

## Gradient path

$B_\text{stiff}$ is $C^2$ in $k_\text{eff}$; $k_\text{eff}$ is smooth in the converged state $x^\ast$ because $F_\text{ax}$ is the sum of smooth IPC traction contributions and $\delta$ is either fixed or smoothly determined by the force-control root-find. Standard VJP chain:

$$ \frac{\partial B_\text{stiff}}{\partial \theta} \;=\; \frac{\partial B_\text{stiff}}{\partial k_\text{eff}}\,\frac{\partial k_\text{eff}}{\partial F_\text{ax}}\,\frac{\partial F_\text{ax}}{\partial x^\ast}\,\frac{\partial x^\ast}{\partial \theta} $$

The factor-on-tape pattern amortizes the backward solve across all four reward terms. One RHS.

## Alternatives considered

**Penalty on low transmitted force directly.** $B = \max(0, F_\text{min} - F_\text{ax})^2$. Rejected because it conflates the cavity's stiffness with the prescribed engagement depth — the same cavity transmits a different force at different engagements, so the threshold becomes engagement-dependent. Using $k_\text{eff} = F_\text{ax}/\delta$ normalizes out the engagement.

**Quadratic penalty in a target-stiffness match.** $B = (k_\text{eff} - k_\text{target})^2$. Symmetric around the target, penalizes both too-stiff and too-compliant cavities. Rejected because the peak-pressure barrier already bounds too-stiff cavities from above; adding a symmetric quadratic here would over-specify and double-penalize. This is a lower-bound barrier only.

**Use a Pareto front instead of a barrier.** Have the optimizer produce a front over transmitted-force vs. uniformity, let the domain user pick. Considered as an optimizer-level strategy in [composition](04-composition.md); deferred there, not here. At the reward-term level, the barrier gives a concrete scalar for composition's fixed-weight mode.

**Compute stiffness from the tangent of the force–displacement curve near the operating point.** $k_\text{eff} = \partial F_\text{ax}/\partial \delta$ at the held engagement. More mathematically natural but requires a secondary differentiation pass or a small perturbation sweep; doubles the per-evaluation cost. Rejected because the secant stiffness $F_\text{ax}/\delta$ is adequate for barrier-style bounding, and the tangent version is available when a domain user needs it as a post-hoc diagnostic.
