# Time-dependent tangent stiffness

The [Ch 07 parent's role for Section 2](../07-viscoelastic.md) — "the Newton-solver tangent changes shape accordingly" with the timestep — is what this leaf names. Prony viscoelasticity makes the per-element tangent stiffness an explicit function of $\Delta t$: at small step the tangent recovers the instantaneous (glassy) stiffness; at large step it recovers the equilibrium stiffness; in between it interpolates analytically. This leaf derives the multiplier, names the block-size and sparse-pattern consequences, and notes the parallel for the Oldroyd-B variant.

## Tangent for the Prony variant

The [implicit-exponential update](00-prony.md) gave the per-mode hereditary stress at the new step:

$$ Q_i^{n+1} = e^{-\Delta t / \tau_i}\, Q_i^{n} + g_i\, \frac{1 - e^{-\Delta t / \tau_i}}{\Delta t / \tau_i}\, \left( P_\text{base}(F^{n+1}) - P_\text{base}(F^{n}) \right) $$

For the [Newton tangent](../../50-time-integration/00-backward-euler.md), the relevant derivative is $\partial Q_i^{n+1} / \partial F^{n+1}$. The stored $Q_i^n$ and $P_\text{base}(F^n)$ are constants at the new step, so the only $F^{n+1}$ dependence is through the $P_\text{base}(F^{n+1})$ factor:

$$ \frac{\partial Q_i^{n+1}}{\partial F^{n+1}} = g_i\, \frac{1 - e^{-\Delta t / \tau_i}}{\Delta t / \tau_i}\, \frac{\partial P_\text{base}(F^{n+1})}{\partial F^{n+1}} $$

Summing across modes and adding the base contribution, the total tangent is the base tangent scaled by an effective viscoelastic stiffness multiplier:

$$ \frac{\partial P_\text{total}^{n+1}}{\partial F^{n+1}} = \alpha_\text{visc}(\Delta t)\, \frac{\partial P_\text{base}(F^{n+1})}{\partial F^{n+1}} $$

with:

$$ \alpha_\text{visc}(\Delta t) = 1 + \sum_{i=1}^{N} g_i\, \frac{1 - e^{-\Delta t / \tau_i}}{\Delta t / \tau_i} $$

The multiplier is a scalar (not a tensor): it scales the base tangent uniformly across all components, so the sparse pattern of the Newton matrix is identical to the rate-independent case, and the Cholesky factorization that [Part 5 Ch 00](../../50-time-integration/00-backward-euler.md) commits to applies without modification.

## Limits in $\Delta t$

The multiplier interpolates two physically meaningful limits:

- **Small step ($\Delta t \to 0$):** $(1 - e^{-\Delta t/\tau_i})/(\Delta t/\tau_i) \to 1$, so $\alpha_\text{visc} \to 1 + \sum_i g_i$ — the glassy (instantaneous) stiffness multiplier. The Newton tangent in this limit is the base tangent times the full instantaneous response factor; the solver "sees" the stiff material it would resist a fast loading event with.
- **Large step ($\Delta t \to \infty$):** $(1 - e^{-\Delta t/\tau_i})/(\Delta t/\tau_i) \to 0$, so $\alpha_\text{visc} \to 1$ — the equilibrium stiffness. The Newton tangent is just the base tangent; the solver "sees" only the long-time response because all modes have fully relaxed.

In between, $\alpha_\text{visc}(\Delta t)$ smoothly interpolates between the two limits. A Newton solver running at a viscoelastic-relevant timestep ($\Delta t$ comparable to the smallest $\tau_i$) sees an effective stiffness that lies between glassy and equilibrium — exactly the right physical stiffness for its operating regime.

## Block size and sparse pattern

Per-element block sizes are unchanged from the rate-independent case — $12 \times 12$ for [Tet4](../../30-discretization/00-element-choice/00-tet4.md), $30 \times 30$ for [Tet10](../../30-discretization/00-element-choice/01-tet10.md). The hereditary state $\{Q_i^n\}$ does not introduce extra unknowns in the linear system; it is stored data the per-step assembly consumes, not a coupled field the solver iterates on. The sparse CSR pattern is determined by element connectivity alone, identical to the elastic case.

The autograd tape carries the converged $\{Q_i^{n+1, *}\}$ as inputs to the next step's assembly. The [factor-on-tape Cholesky path](../../50-time-integration/00-backward-euler.md) caches the factorization of $\alpha_\text{visc}(\Delta t) \cdot K_\text{base}$ at step $n+1$ and re-uses it for the IFT adjoint solve at the same step — the multiplier-scaling preserves the SPD structure that makes Cholesky applicable.

## Tangent for the Oldroyd-B variant

The [Oldroyd-B variant](01-oldroyd.md) couples differently. The UCD term $-L Q - Q L^T$ involves the velocity gradient $L = \dot F\, F^{-1}$, so $\partial Q_i^{n+1} / \partial F^{n+1}$ picks up contributions from both $L^{n+1}$ (which depends on $F^{n+1}$ via finite differences with $F^n$) and from the $D$-driving term (also depending on $L^{n+1}$).

The result is no longer a clean scalar multiplier on the base tangent: $\partial Q_i^{n+1} / \partial F^{n+1}$ has tensor structure that couples differently across components. Per-element block sizes remain $12 \times 12$ / $30 \times 30$ (the new couplings are within-element, not cross-element), and the sparse pattern of the global matrix is unchanged for the same reason. But the per-block fill is denser than in the Prony case, and the block is no longer a simple scaling of the base tangent.

The factor-on-tape Cholesky path still applies — Oldroyd-B's tangent is SPD in well-conditioned regimes, and the IFT adjoint re-uses the factorization the same way. The cost difference is in the per-block constitutive evaluation, not in the linear-solver topology.

## What this sub-leaf commits the book to

- **Prony viscoelastic tangent is the base tangent scaled by $\alpha_\text{visc}(\Delta t)$.** Scalar multiplier, smooth in $\Delta t$, interpolating from $1 + \sum_i g_i$ (glassy, small-step) to $1$ (equilibrium, large-step). The Newton matrix's sparse pattern and the Cholesky factorization carry over from the rate-independent case unchanged.
- **Per-element block sizes are unchanged.** Hereditary state $\{Q_i^n\}$ is stored data, not coupled unknowns. $12 \times 12$ Tet4 and $30 \times 30$ Tet10 blocks both inherit; sparse CSR pattern is element-connectivity-determined alone.
- **The autograd tape sees the timestep-scaled tangent.** Factor-on-tape caches $\alpha_\text{visc}(\Delta t) \cdot K_\text{base}$ and re-uses it for the IFT adjoint; the multiplier preserves SPD structure.
- **Oldroyd-B tangent is denser per-block but topologically identical.** Per-element block sizes match Prony; sparse pattern matches Prony; only the per-block constitutive evaluation cost increases.
- **Newton convergence is not degraded by viscoelasticity.** The PSD scaling factor preserves the Newton convergence rate; gradcheck at [Phase B](../../110-crate/03-build-order.md) sees the same effective tangent the line search uses.
