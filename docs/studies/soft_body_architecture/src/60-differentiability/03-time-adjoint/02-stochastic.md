# Stochastic adjoints

When the forward trajectory contains stochastic forcing — Brownian motion injected by `sim-thermostat`'s microscale thermal fluctuation model, or any other noise term crossing into `sim-soft`'s state — the deterministic adjoint of [§01](01-adjoint-state.md) no longer gives the gradient of the expected objective. The trajectory is now a sample from a probability distribution over paths; the gradient of the expectation requires either a backward-in-time SDE for the adjoint ($\lambda$ is itself stochastic, driven by a time-reversed Brownian motion) or an alternative estimator (score-function / REINFORCE-style gradients that treat the noise's density as the differentiable object). This sub-leaf names the constructions, writes the choice `sim-soft` commits to when the stochastic path opens, and clarifies which noise crosses the crate boundary today and which does not.

## When stochasticity enters

Two mechanisms inject randomness into `sim-soft`'s forward; the book treats them as categorically different.

**(1) Brownian forcing inside the solver.** [`sim-thermostat`](../../110-crate/02-coupling/01-thermostat.md)'s thermal-fluctuation coupling (a [Phase H fidelity upgrade](../../110-crate/03-build-order.md#the-committed-order), not a Phase D deliverable) augments the equation of motion with a Brownian-forcing term of the form $\sigma(x; T, \theta)\, \mathrm dW_t$, with magnitude set by temperature. This noise is *inside* the physics state vector — the simulator's $x(t)$ is itself stochastic, and any gradient of a reward computed from $x$ has to propagate through the noise. This is the case Phase H's stochastic adjoint infrastructure targets.

**(2) Policy-injected action noise outside the solver.** `sim-rl`'s on-policy algorithms (REINFORCE, PPO, SAC) sample actions from a stochastic policy $a \sim \pi_\theta(a \mid s)$ and inject those actions through the boundary that `sim-soft` exposes as a reward-and-gradient surface. The noise lives in the policy's action distribution, not in `sim-soft`'s forward. Gradient estimators for $\nabla_\theta\, E_\pi[R]$ — score-function, pathwise via reparameterization, doubly-robust — are the algorithm's concern and are already part of `sim-ml-chassis`'s infrastructure. `sim-soft` sees a deterministic forward conditioned on the action; no stochastic-adjoint machinery is required at the coupling surface.

The distinction is the noise boundary. Brownian forcing crosses the `sim-soft` boundary and requires Phase H's SDE adjoint. Policy-action noise does not cross the boundary; the deterministic adjoint shipping in Phase D handles every objective in that class.

## Forward SDE: Itô vs Stratonovich

The forward dynamics with thermal forcing are an SDE; the interpretation of that SDE is a choice. For a state-dependent noise coefficient $\sigma(x; \theta)$,

$$ \mathrm dx = f(x; \theta)\, \mathrm dt + \sigma(x; \theta)\, \mathrm dW_t $$

is ambiguous until the stochastic integral is defined. Two conventions dominate:

- **Itô.** The stochastic integral is the left-endpoint Riemann sum $\int \sigma\, \mathrm dW = \lim \sum \sigma(x(t_i))\,(W(t_{i+1}) - W(t_i))$; the integrand is non-anticipating. Itô SDEs do not satisfy the ordinary chain rule; differentiating a function $\phi(x)$ of an Itô process requires Itô's lemma, $\mathrm d\phi = \phi'(x)\,\mathrm dx + \tfrac{1}{2}\phi''(x)\,\sigma^2\,\mathrm dt$, whose second term — the Itô correction — is the price of the non-anticipating convention. Numerical schemes like Euler-Maruyama are Itô by construction.
- **Stratonovich.** The stochastic integral uses a midpoint rule $\int \sigma \circ \mathrm dW = \lim \sum \sigma(x(\tfrac{t_i + t_{i+1}}{2}))\,(W(t_{i+1}) - W(t_i))$; integrands anticipate half a timestep into the noise. Stratonovich SDEs *do* satisfy the ordinary chain rule, which makes adjoint derivations one-line applications of the chain rule rather than Itô-calculus bookkeeping.

The two conventions define the same continuous process up to a drift correction: converting an Itô SDE with drift $f$ to Stratonovich form replaces the drift with $f - \tfrac{1}{2}\, \sigma\, \partial\sigma/\partial x$ (scalar case; the matrix analogue generalizes). Phase H's forward discretization is Euler-Maruyama (Itô); the adjoint derivation is cleaner in Stratonovich form and converts at the math layer rather than at the integration layer.

## Backward SDE adjoint

The adjoint of an SDE is a *backward* SDE — an SDE integrated from $t = T$ down to $t = 0$, driven by a time-reversed Brownian motion. The construction is the Stratonovich-form analogue of the deterministic [§01 adjoint ODE](01-adjoint-state.md), with an additional noise-coupling term carrying the gradient information through the stochastic part of the forward.

[Li et al. 2020 (SDE adjoint)](../../appendices/00-references/02-adjoint.md#li-2020-sde) gives the canonical Stratonovich-form backward SDE for a continuous-time forward with state-dependent noise. The adjoint satisfies

$$ \mathrm d\lambda = -\left(\frac{\partial f}{\partial x}\right)^T \lambda\, \mathrm dt - \left(\frac{\partial \sigma}{\partial x}\right)^T \lambda \circ \mathrm dW_t - \left(\frac{\partial g}{\partial x}\right)^T\, \mathrm dt, $$

with terminal condition $\lambda(T) = (\partial \Phi / \partial x)(x(T))$ matching the deterministic case. The second term — the $\sigma$-Jacobian contracted against $\lambda$ and integrated against the same $\mathrm dW_t$ path the forward drew — is what separates the stochastic adjoint from the deterministic one. [Kidger et al. 2021](../../appendices/00-references/02-adjoint.md#kidger-2021) extends this to both Itô and Stratonovich interpretations for neural SDEs and documents the resulting estimator-variance tradeoffs.

The estimator the backward SDE produces is *pathwise*: one forward-backward pass along a single Brownian path gives one sample of $\nabla_\theta L$ whose expectation over the noise distribution is the true gradient $\nabla_\theta\, E[L]$. The estimator is unbiased. What it is not is low-variance: the variance scales with the noise magnitude and the integration length, and a single-path estimate can be noisy enough to require several-to-many sampled paths per gradient update. Score-function estimators trade variance for bias differently; `sim-soft`'s Phase H commitment is to the pathwise Stratonovich-form backward SDE because it composes with `sim-ml-chassis`'s reverse-mode tape with one extra closure per step and because the Li/Kidger machinery is the best-anchored published construction.

## Variance and sample-path reuse

The variance of the pathwise estimator has two mitigations the book names without specifying:

1. **Sample-path reuse.** For multiple objectives evaluated on the same trajectory (e.g., the reward components of [Part 1 Ch 01](../../10-physical/01-reward.md) all computed from the same forward roll-out), one backward SDE sweep produces gradients for every objective at once — the noise variance is shared across gradient components rather than multiplied.
2. **Common random numbers across optimizer iterations.** Drawing the same Brownian path at consecutive optimizer iterations reduces the variance of the finite-difference-like comparison between iterations' gradients, a standard stochastic-optimization technique.

Neither is a free lunch, and both interact with the [Ch 04 checkpointing](../04-checkpointing.md) layer — the checkpoint schedule has to preserve the Brownian path samples alongside the state checkpoints so the backward recomputation hits the same noise realization the forward consumed. Phase H's implementation plan (not this chapter's problem) names the specific sample-storage contract.

## `sim-soft`'s commitment

Phase D ships the deterministic adjoint of [§01](01-adjoint-state.md). Phase H adds the stochastic adjoint alongside the Prony viscoelastic and thermal-fluctuation machinery, because those three upgrades all need each other — microscale thermal fluctuations enter through the same Brownian-forcing term `sim-thermostat`'s Phase H coupling provides, and the hereditary-state viscoelastic terms are what the thermal forcing dissipates into.

The test for which adjoint a given objective needs is the noise-boundary check: does any stochastic term enter `sim-soft`'s forward state vector $x(t)$ between the parameters $\theta$ being differentiated and the objective $L$ being evaluated? If yes, Phase H's stochastic adjoint is required; if no, Phase D's deterministic adjoint gives the exact gradient. Policy-action noise from `sim-rl` does not cross the boundary (the action is consumed by `sim-soft` as a deterministic input, with noise handled by `sim-rl`'s policy-gradient estimator); `sim-thermostat`'s deterministic thermal coupling (Phase F, not Phase H) does not cross the boundary either (no random forcing, just PDE heat transport). Only Phase H's microscale stochastic thermal modeling opens the boundary.

The [custom thermo-RL project](../../10-physical/00-canonical.md) uses two sources of stochasticity at different build phases. Phase D's first-working thermo-RL loop uses policy-side action noise — handled by `sim-rl`'s policy-gradient estimators, boundary clean, deterministic adjoint sufficient. Phase H opens the second source: microscale Brownian thermal forcing injected by `sim-thermostat` becomes part of the forward state and part of the project's exploration strategy. That source crosses the boundary and requires the Stratonovich-form backward SDE. Phase D is "every objective that does not cross a noise boundary"; Phase H adds the objectives that do.

## What this sub-leaf commits the book to

- **The noise-boundary check determines which adjoint applies.** If stochastic terms enter `sim-soft`'s state vector between $\theta$ and $L$, Phase H's stochastic adjoint is required; otherwise Phase D's deterministic adjoint is exact. Policy-action noise from `sim-rl` does not cross the boundary; `sim-thermostat`'s deterministic thermal coupling does not cross it; Phase H's microscale Brownian thermal fluctuations do.
- **Phase H's stochastic adjoint uses the Stratonovich-form backward SDE.** The Li et al. 2020 construction is the anchor; the Stratonovich form admits ordinary chain rule for the adjoint derivation even though Euler-Maruyama forward discretization is natively Itô. Drift conversion happens at the math layer, not the integration layer.
- **The pathwise estimator is unbiased, not low-variance.** Single-path estimates can require multiple-path averaging; the book names sample-path reuse across multi-objective evaluations and common-random-numbers across optimizer iterations as the two canonical variance-reduction handles, with specifics deferred to Phase H's implementation plan.
- **Phase D covers the thermo-RL subset that does not cross the noise boundary.** The first-working thermo-RL loop uses policy-side action noise in `sim-rl`, which does not cross into `sim-soft`'s forward state. Phase H adds the Brownian thermal forcing the [full project vision](../../10-physical/00-canonical.md) uses as part of its exploration strategy; the stochastic adjoint lands alongside `sim-thermostat`'s Phase H microscale-fluctuation upgrade. The Phase H deferral partitions the thermo-RL roadmap into a deterministic first-working subset and a stochastic full-fidelity extension — it does not gate all thermo-RL work on a single stochastic-machinery dependency.
