# Forward vs reverse through time

[Ch 02's IFT](../02-implicit-function.md) gives the gradient of a converged backward-Euler step — one implicit solve, one linear system against the factored Hessian, one sparse-matrix contraction against the residual-$\theta$-Jacobian. A trajectory-level gradient composes many such per-step gradients across a rollout $\{x_0, x_1, \ldots, x_T\}$. Two constructions compose them: *forward sensitivity* propagates $\partial x / \partial \theta$ forward alongside $x$, and *reverse adjoint* propagates $\lambda$ backward from the terminal state. This sub-leaf writes the cost model of each, names the regime where one beats the other, and explains why `sim-soft` commits to reverse even for problem sizes where the crossover could go either way.

## Forward sensitivity

Differentiate the forward recursion $r(x_t, x_{t-1}; \theta) = 0$ with respect to $\theta$ at every step:

$$ \frac{\partial r}{\partial x_t}\, \frac{\partial x_t}{\partial \theta} + \frac{\partial r}{\partial x_{t-1}}\, \frac{\partial x_{t-1}}{\partial \theta} + \frac{\partial r}{\partial \theta} = 0. $$

Define the sensitivity $S_t \equiv \partial x_t / \partial \theta$ — a matrix with $n$ rows (state dimension) and $n_\theta$ columns (parameter count). The recursion for $S_t$ reads

$$ S_t = -A_t^{-1}\left(\frac{\partial r_t}{\partial x_{t-1}}\, S_{t-1} + \frac{\partial r_t}{\partial \theta}\right), $$

where $A_t = \partial r_t/\partial x_t = H_t$ is the same forward-Euler Hessian of [Ch 02 §00](../02-implicit-function/00-derivation.md). Each step requires one forward solve for $x_t$ plus $n_\theta$ back-substitutions against the factor $A_t$ — one column of $S_t$ per parameter. The sensitivity matrix lives alongside $x_t$ as the integration marches forward; no trajectory storage is needed.

Per step, forward sensitivity pays $n_\theta$ back-substitutions on top of the forward solve; a trajectory of $T$ steps pays $T \cdot n_\theta$ back-substitutions total. Implementation-wise, forward sensitivity composes cleanly with forward-mode AD — each parameter column is propagated as a tangent alongside the primal — which is why small-$n_\theta$ problems often reach for it first.

## Reverse adjoint

Reverse integrates a backward recursion on a vector $\lambda_t$ whose terminal value encodes the objective's sensitivity to the final state. For an objective $L = \sum_{t=0}^{T} g(x_t; \theta) + \Phi(x_T)$, [§01's derivation](01-adjoint-state.md) gives the backward recursion

$$ \lambda_{t} = A_t^{-1}\left[\left(\frac{\partial g_t}{\partial x_t}\right)^T - \left(\frac{\partial r_{t+1}}{\partial x_t}\right)^T \lambda_{t+1}\right], $$

with the terminal $\bar x_T = (\partial g_T/\partial x_T)^T + (\partial \Phi/\partial x_T)^T$ feeding $\lambda_T = A_T^{-1} \bar x_T$. Each backward step applies the transposed residual-Jacobian coupling from Ch 02 §02 ($(\partial r_{t+1}/\partial x_t)^T$ is the closed-form inertial block) and solves one linear system against $A_t$ — the same factor the forward computed, reused by SPD symmetry. The trajectory gradient is assembled as

$$ \frac{\mathrm dL}{\mathrm d\theta} = \left(\frac{\partial \Phi}{\partial \theta}\right)^T + \sum_{t=0}^{T} \left(\frac{\partial g_t}{\partial \theta}\right)^T - \sum_{t=1}^{T} \left(\frac{\partial r_t}{\partial \theta}\right)^T \lambda_t, $$

using Ch 02 §00's sign convention — the per-step $\lambda_t = A_t^{-1} \bar x_t$ is a positive back-substitution, and each trajectory contribution to $\bar\theta$ reproduces the per-step $-(\partial r/\partial \theta)^T \lambda$ form.

Cost per step: one back-substitution plus one sparse-matrix-vector contraction, both independent of $n_\theta$. Total: $T$ back-substitutions regardless of parameter count. The $n_\theta$ factor that made forward sensitivity expensive is gone from the per-step work; it reappears only in the final contraction of $\lambda_t$ against $\partial r_t/\partial \theta$, which is a single sparse-matrix-vector product per step.

The price is memory: every backward step needs access to the forward step's factored $A_t$ and its residual-Jacobian $\partial r_t/\partial \theta$. Naively, the forward pass stores one such pair per step; [Ch 04 checkpointing](../04-checkpointing.md) replaces that $O(T)$ storage with an $O(\log T)$ checkpoint schedule and $O(T \log T)$ extra forward re-solves.

## When each wins

The break-even between forward sensitivity and reverse adjoint depends on the ratio of per-step sensitivity cost ($n_\theta$ back-substitutions plus one $\partial r/\partial \theta$ evaluation) to per-step adjoint cost (one back-substitution plus one contraction) — a ratio that turns on the factor's back-substitution cost, the sparsity of $\partial r/\partial \theta$, and the memory cost of forward-trajectory storage. No single crossover number holds across scenes; the regime, not a specific count, is what the design choice turns on.

Two regimes bracket the question:

- **A small handful of parameters** — the canonical-problem geometry $\{r_c, r_p, t_c, L_c, \delta\}$ of [Part 1 Ch 00](../../10-physical/00-canonical/00-formulation.md) gives $n_\theta = 5$; a single-material ablation over $\{E, \nu, \rho\}$ gives $n_\theta = 3$. Forward sensitivity is the textbook fit in this regime: its per-step overhead is a small constant multiple of the forward solve, it needs no trajectory storage, and it maps directly onto forward-mode AD. `sim-soft`'s choice here is still reverse — the commitment below explains why even small sweeps route through the reverse-adjoint path rather than through a parallel forward-mode implementation.

- **Hundreds to millions of parameters** — a material field with thousands of per-element stiffness coefficients from [Part 2 Ch 09 spatial fields](../../20-materials/09-spatial-fields.md), or a policy network with $10^5$ weights feeding into the [full design-print-rate loop](../../100-optimization/06-full-loop.md). Forward sensitivity's $n_\theta$-fold per-step cost makes it quadratically slower than the forward solve; reverse adjoint's $O(1)$ per-step cost is the only affordable path. Memory for the forward trajectory (or its checkpoint schedule) is the trade the scene pays to get there.

## Why `sim-soft` commits to reverse

The primary consumer is the design-print-rate loop, whose parameter count is dominated by whichever component is largest — a policy network with $10^5$ weights, or a material field whose spatial resolution matches the mesh's element count. Both are firmly in the regime where reverse adjoint is the only option. The tape infrastructure landing with [Ch 01's registration API](../01-custom-vjps/00-registration.md) is built for reverse; adding a forward-sensitivity path on top would double the adjoint machinery without buying anything for the workloads the crate prioritizes.

Two consequences follow. First, `sim-soft` does not ship a dedicated forward-sensitivity mode: small-$n_\theta$ sweeps use the same reverse-adjoint path, paying the modest memory overhead of trajectory checkpointing for the win of a single uniform gradient construction. Second, forward-mode AD through `sim-ml-chassis`'s tape is not a feature — the chassis tape is reverse-mode only (per [Ch 00 §00](../00-what-autograd-needs/00-generic.md)'s surface description), and `sim-soft`'s custom VJPs are reverse-only closures. Validation of the reverse-adjoint gradient is done by finite differences (central-difference gradcheck against the re-solved forward at perturbed $\theta$, per [Part 11 Ch 04 §03](../../110-crate/04-testing/03-gradcheck.md)) — not by forward-mode AD, since no forward-mode AD path exists on the chassis to compare against.

## What this sub-leaf commits the book to

- **Reverse adjoint is the only gradient construction `sim-soft` ships for trajectory-level objectives.** Forward sensitivity is named as the textbook alternative and is acknowledged as the simpler choice for $n_\theta$ in the small-handful regime; it is not implemented in-crate because the loop that matters is never in that regime.
- **The break-even between forward and reverse is a regime, not a count.** Small-handful versus hundreds-to-millions — both ends sit clearly on one side of any plausible crossover, so the design choice is robust to the ratio uncertainty. Specific-number break-even claims are a Phase-B benchmarking deliverable, not a Pass-3 claim.
- **Forward-trajectory memory is the cost reverse pays; [Ch 04](../04-checkpointing.md) is the mitigation.** A naive O(T) storage of forward factors and residual-Jacobians along the trajectory is unaffordable at long-rollout sizes; Revolve's $O(\log T)$ checkpoint schedule with $O(T \log T)$ extra forward compute is the trade that recovers a bounded memory footprint.
- **Forward-mode AD does not exist on `sim-ml-chassis`'s tape.** The chassis commitment is reverse-mode scalar autograd; `sim-soft`'s custom VJPs extend that surface with reverse-only backward closures. A forward-mode path would require a separate piece of machinery — rejected as unnecessary given the regime analysis above.
