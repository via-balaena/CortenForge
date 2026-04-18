# Prony series — discrete relaxation spectrum

The [Ch 07 parent's Claim 2](../07-viscoelastic.md) commits to the Prony series as `sim-soft`'s default viscoelastic parametrization — a discrete approximation to the continuous relaxation spectrum, with a small number of Maxwell branches summing to fit the storage and loss modulus curves DMA produces. This leaf writes the convolutional stress form, the per-element hereditary-state ODE, and the implicit-exponential time-step rule the [`Viscoelastic<M>` decorator](../00-trait-hierarchy/01-composition.md) hands the [backward-Euler Newton loop](../../50-time-integration/00-backward-euler.md) at assembly time.

## Convolutional stress form

For a deformation history $F(\cdot)$ over a single Newton step, the total first Piola stress at time $t$ is the equilibrium base stress plus a sum of convolutions of the base-stress rate against exponential relaxation kernels:

$$ P_\text{total}(t) = P_\text{base}(F(t)) + \sum_{i=1}^{N} g_i \int_0^t e^{-(t-s)/\tau_i}\, \dot P_\text{base}(F(s))\, ds $$

The first term is the equilibrium response — the long-time stress the wrapped base material settles to under fixed strain — and is what `M: Material`'s `first_piola(F)` returns. The summed convolutions add the rate-dependent over-stress: each mode integrates the recent rate of base-stress change with exponential memory of time constant $\tau_i$, weighted by the dimensionless over-stress weight $g_i$. The convention treats the user-supplied base law as the equilibrium response, so quasi-static stress-strain calibration data anchors $P_\text{base}$ directly with no separate equilibrium-vs-glassy bookkeeping.

For silicones in the canonical-problem regime, $N = 3$ or $4$ typically suffices to fit the DMA storage and loss curves across the operating frequency band, with relaxation times spanning two-to-three decades around the application's characteristic timescale.

## Decorator state versus per-element state

Per the [Ch 00 composition leaf](../00-trait-hierarchy/01-composition.md), the `Viscoelastic<M>` decorator stores only the Prony parameters — uniform across every element assigned to this material:

```rust
pub struct Viscoelastic<M: Material> {
    base: M,
    prony: Vec<(f64 /* tau_i */, f64 /* g_i */)>,
}
```

The hereditary state — one $3 \times 3$ tensor $Q_i$ per Prony mode per tet — lives in the per-element data block in [`element/`](../../110-crate/00-module-layout/01-element.md), alongside the element's deformation gradient. This separation keeps the decorator stateless and `Send + Sync`, and keeps the per-element data block sized predictably ($N$ extra $3 \times 3$ tensors per element).

## Hereditary-state ODE

Defining $Q_i(t) = g_i \int_0^t e^{-(t-s)/\tau_i}\, \dot P_\text{base}(F(s))\, ds$ as the per-mode hereditary stress, differentiating gives the first-order ODE for each mode:

$$ \dot Q_i + \frac{Q_i}{\tau_i} = g_i\, \dot{P}_\text{base}(F(t)) $$

The total per-element first Piola stress is then:

$$ P_\text{total}(F, \{Q_i\}) = P_\text{base}(F) + \sum_{i=1}^{N} Q_i $$

Each $Q_i$ has its own time constant; the modes do not couple to each other through the ODE, only through the shared driving rate $\dot P_\text{base}$. This independence is what makes the per-mode update a single decoupled exponential step rather than a coupled solve.

## Implicit-exponential time-step rule

Backward-Euler is the [Part 5 Ch 00 commitment](../../50-time-integration/00-backward-euler.md), but the hereditary ODE has an analytic solution under constant base-stress rate over the step that is more accurate than first-order Euler and unconditionally stable. Taking $\dot P_\text{base} \approx (P_\text{base}^{n+1} - P_\text{base}^{n}) / \Delta t$ as constant over $[t_n, t_{n+1}]$, the per-mode update is:

$$ Q_i^{n+1} = e^{-\Delta t / \tau_i}\, Q_i^{n} + g_i\, \frac{1 - e^{-\Delta t / \tau_i}}{\Delta t / \tau_i}\, \left( P_\text{base}(F^{n+1}) - P_\text{base}(F^{n}) \right) $$

Two limits:

- **Small step ($\Delta t \ll \tau_i$):** $e^{-\Delta t/\tau_i} \to 1 - \Delta t/\tau_i$ and $(1 - e^{-\Delta t/\tau_i})/(\Delta t/\tau_i) \to 1$, recovering the first-order semi-discrete update $Q_i^{n+1} \approx Q_i^{n} + g_i (P_\text{base}^{n+1} - P_\text{base}^{n}) - Q_i^{n} \Delta t/\tau_i$.
- **Large step ($\Delta t \gg \tau_i$):** $e^{-\Delta t/\tau_i} \to 0$ (the previous state is forgotten) and $(1 - e^{-\Delta t/\tau_i})/(\Delta t/\tau_i) \to \tau_i/\Delta t$, so $Q_i^{n+1} \to g_i\, \tau_i\, \dot P_\text{base}$ — the steady-state response under the assumed constant rate. The mode has fully relaxed during the step and tracks only the current loading rate. The update is unconditionally stable: no amplification, no oscillation, even as $\tau_i$ becomes much smaller than $\Delta t$.

The implicit-exponential form is what makes Prony viscoelasticity practical at the spread of $\tau_i$ values DMA fits produce: relaxation times across two-to-three decades coexist in a single material, and a uniform timestep that resolves the smallest $\tau_i$ would be unworkable for the largest. The exponential update handles all modes in one decoupled pass without timestep-shrinking.

## Assembly contribution

At every Newton iteration of step $n+1$, the [Part 5 Ch 00 backward-Euler loop](../../50-time-integration/00-backward-euler.md) calls `Viscoelastic<M>`'s history-advance method with the current iterate $F^{n+1, k}$ and the stored $\{Q_i^n\}$ from the previous timestep. The method:

1. Evaluates $P_\text{base}(F^{n+1, k})$ from the wrapped `M: Material`.
2. Computes the would-be $\{Q_i^{n+1, k}\}$ from the implicit-exponential rule.
3. Returns $P_\text{total}^{n+1, k} = P_\text{base} + \sum_i Q_i^{n+1, k}$ for the residual assembly.

Only after the Newton iteration converges (the converged $F^{n+1, *}$ is found) are the $\{Q_i^{n+1, *}\}$ committed to the per-element history block; intermediate Newton iterates do not corrupt the stored state. The autograd tape carries the committed $\{Q_i^{n}\}$ as inputs to the next step's evaluation, making the per-step gradient flow through the history naturally.

## What this sub-leaf commits the book to

- **Prony series with $(\tau_i, g_i)$ pairs is the default viscoelastic parametrization.** Over-stress-weight convention: each $g_i$ is the dimensionless rate-dependent overshoot for mode $i$; equilibrium is the wrapped base material's response. Small number of modes ($N = 3$ or $4$ typical for silicones) suffices to fit DMA data.
- **The hereditary state lives in `element/`, the parameters live on `Viscoelastic<M>`.** Per the [Ch 00 composition leaf](../00-trait-hierarchy/01-composition.md): one $3 \times 3$ $Q_i$ tensor per mode per tet, $9N$ extra `f64` fields per element.
- **Implicit-exponential update handles wide-spread $\tau_i$ in one decoupled pass.** Each mode's update is the analytic exponential decay over $\Delta t$ with constant rate; no timestep shrinking required as $\tau_i$ ranges across decades, and the update is unconditionally stable.
- **History commits at Newton convergence, not per iterate.** Intermediate $\{Q_i\}$ during Newton iteration are recomputed from the stored previous-step $\{Q_i^n\}$; the autograd tape stores the converged history at each step boundary.
- **Total stress is base equilibrium plus the hereditary sum.** $P_\text{total} = P_\text{base}(F) + \sum_i Q_i$. The base provides the long-time response; the Prony modes add the rate-dependent over-stress.
