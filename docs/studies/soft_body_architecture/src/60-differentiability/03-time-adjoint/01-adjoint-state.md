# Adjoint state equation

The continuous-time adjoint $\lambda(t)$ is the auxiliary variable whose backward-in-time integration produces trajectory gradients at a per-step cost independent of parameter dimension. This sub-leaf writes the continuous adjoint derivation, specializes it to backward-Euler's implicit step structure, names the discrete-adjoint choice that threads the backward recursion through the same factorizations the forward Newton produced, and connects the trajectory-level $\lambda(t)$ to [Ch 02 §00](../02-implicit-function/00-derivation.md)'s per-step $\lambda = A^{-1}\, \bar x^\ast$.

## Continuous form

Consider a deterministic forward dynamics $\dot x = f(x;\theta)$ on $[0, T]$ with initial condition $x(0) = x_0$. The objective combines a running term and a terminal cost:

$$ L(\theta) = \int_0^T g(x(t);\theta)\, \mathrm dt + \Phi(x(T)). $$

Adjoining the dynamics constraint with a Lagrange multiplier $\lambda(t)$ produces the augmented functional

$$ \tilde L = L + \int_0^T \lambda(t)^T\, [f(x;\theta) - \dot x]\, \mathrm dt. $$

Integration by parts on the $\lambda^T \dot x$ term gives $\int_0^T \lambda^T \dot x\, \mathrm dt = \lambda(T)^T x(T) - \lambda(0)^T x(0) - \int_0^T \dot\lambda^T x\, \mathrm dt$. Stationarity of $\tilde L$ with respect to $x(t)$ at every interior point yields

$$ \dot\lambda = -\left(\frac{\partial f}{\partial x}\right)^T \lambda - \left(\frac{\partial g}{\partial x}\right)^T, $$

and stationarity at $t = T$ yields the terminal condition

$$ \lambda(T) = \left(\frac{\partial \Phi}{\partial x}\right)(x(T)). $$

With those two conditions enforced, the trajectory gradient collapses to

$$ \frac{\mathrm dL}{\mathrm d\theta} = \frac{\partial \Phi}{\partial \theta} + \int_0^T \left[\frac{\partial g}{\partial \theta} + \lambda^T\, \frac{\partial f}{\partial \theta}\right]\mathrm dt. $$

Every $\theta$-dependent term in the forward dynamics or in the running cost contributes to the gradient through its local $\theta$-Jacobian contracted against $\lambda$; nothing from the forward trajectory enters the gradient except through the coefficients of the adjoint ODE and the $\theta$-branch Jacobians at each time. Integrating $\lambda$ from $t = T$ down to $t = 0$ is one backward sweep whose per-step cost does not scale with $n_\theta$.

## Discrete form

`sim-soft`'s forward is not a continuous flow — it is a backward-Euler Newton solve at each step. The trajectory is $\{x_0, x_1, \ldots, x_T\}$; each $x_t$ satisfies the implicit residual $r_t(x_t, x_{t-1}; \theta) \equiv \nabla_{x_t} U_t(x_t; x_{t-1}, \theta) = 0$ from [Ch 00 §01 Newton](../../50-time-integration/00-backward-euler/01-newton.md), and the objective is

$$ L(\theta) = \sum_{t=0}^{T} g(x_t;\theta) + \Phi(x_T). $$

The discrete adjoint falls out of chaining the per-step IFT of [Ch 02 §00](../02-implicit-function/00-derivation.md) across time, with the Ch 02 $\lambda = A^{-1} \bar x^\ast$ convention (positive back-substitution, negative contraction on gradient contributions) preserved. At each step $t$ the upstream adjoint $\bar x_t$ arrives from downstream; Ch 02 §00 gives the per-step adjoint and its contributions back to step-$t$ inputs:

$$ \lambda_t = A_t^{-1}\, \bar x_t, \qquad \bar x_{t-1}^{(t)} = -\left(\frac{\partial r_t}{\partial x_{t-1}}\right)^T \lambda_t, \qquad \bar\theta^{(t)} = -\left(\frac{\partial r_t}{\partial \theta}\right)^T \lambda_t. $$

The total upstream at step $t$ sums the local-reward contribution with the propagation from step $t+1$:

$$ \bar x_t = \left(\frac{\partial g_t}{\partial x_t}\right)^T - \left(\frac{\partial r_{t+1}}{\partial x_t}\right)^T \lambda_{t+1}, $$

with the terminal case $\bar x_T = (\partial g_T/\partial x_T)^T + (\partial \Phi/\partial x_T)^T$ (no step $T+1$; the terminal cost $\Phi$ contributes directly). Substituting into the first relation gives the backward recursion

$$ \lambda_t = A_t^{-1}\left[\left(\frac{\partial g_t}{\partial x_t}\right)^T - \left(\frac{\partial r_{t+1}}{\partial x_t}\right)^T \lambda_{t+1}\right], $$

with $A_t \equiv \partial r_t/\partial x_t = H_t$ the step-$t$ Newton Hessian (SPD at convergence, so $A_t^{-T} = A_t^{-1}$). Integrating backward from $t = T$ to $t = 1$ produces the full adjoint trajectory; the gradient assembles as

$$ \frac{\mathrm dL}{\mathrm d\theta} = \left(\frac{\partial \Phi}{\partial \theta}\right)^T + \sum_{t=0}^{T} \left(\frac{\partial g_t}{\partial \theta}\right)^T - \sum_{t=1}^{T} \left(\frac{\partial r_t}{\partial \theta}\right)^T \lambda_t. $$

Each per-step term reproduces the Ch 02 §00 single-step form $\bar\theta = -(\partial r/\partial \theta)^T \lambda$ with the per-step $\lambda$ coming from the chained upstream. The symbol $\lambda_t$ means exactly what Ch 02 §00's $\lambda$ meant, now indexed by the trajectory step.

## Per-step structure

Every step's backward update involves exactly one linear solve against $A_t = H_t$. The forward Newton loop computed and factored $H_t$ at convergence; the factor was captured in the tape node's `State` per [Ch 02 §01](../02-implicit-function/01-linear-solve.md)'s `BackwardEulerState`; the backward closure re-applies that factor by back-substitution — exactly the primitive the IFT adjoint uses at step level, now chained across time.

The $(\partial r_{t+1}/\partial x_t)^T$ coupling that carries gradient information from step $t+1$ into step $t$ decomposes into a closed-form inertial block and, where applicable, a velocity-coupling block. Per [Ch 02 §02 memory](../02-implicit-function/02-memory.md), $\partial r/\partial x_{n-1} = -M/\Delta t^2$ and $\partial r/\partial v_{n-1} = -M/\Delta t$ are never stored as matrices — $M$ lives at the engine scope, each step's $\Delta t$ is held alongside the tape node, and the backward closure applies these blocks by multiplying $\lambda$ against the mass-matrix operator rather than against a cached sparse Jacobian. No per-step storage beyond the factor, $\partial r/\partial \theta$, and the scalar $\Delta t$.

One back-substitution plus one sparse $(\partial r_t/\partial \theta)^T \lambda_t$ contraction plus one inertial-block apply per step: this is the per-step work the trajectory adjoint pays, and it is independent of $n_\theta$. The total backward pass cost is $T$ times that per-step work, plus the cost of re-solving the forward trajectory at whatever fraction [Ch 04 checkpointing](../04-checkpointing.md) demands.

## The adjoint-then-discretize alternative

The construction above takes the adjoint of `sim-soft`'s *actual* backward-Euler iteration — the "discretize-then-adjoint" choice. A different discipline derives the continuous adjoint ODE first and then discretizes it with whatever scheme is convenient, producing an algorithm whose per-step structure may not match the forward's. The two approaches give different numerical answers at finite $\Delta t$; they converge to the same continuous gradient only in the limit ([Sirkes & Tziperman 1997](../../appendices/00-references/02-adjoint.md#sirkes-tziperman-1997)).

`sim-soft`'s choice is discretize-then-adjoint, for a specific reason: that is the only way to reuse the forward's factored $A_t$ at every backward step. Adjoint-then-discretize would integrate the continuous $\dot\lambda = -(\partial f/\partial x)^T \lambda - (\partial g/\partial x)^T$ with a scheme of its own choosing — explicit-Euler, RK4, Crank-Nicolson — and the Jacobian samples the adjoint scheme asks for would not in general be the same factored matrices the forward Newton step produced. The factorization-reuse architectural commitment forces the discrete-adjoint choice.

The downside is that the discrete adjoint gives the *true* gradient of the *discretized* objective. For objectives that care about the infinite-resolution trajectory rather than `sim-soft`'s $\Delta t$-discretized trajectory, there is a bias on the order of the forward scheme's truncation error. For RL and design-optimization objectives at the canonical-problem sizes the book targets, the $\Delta t$-discretized reward is already what the simulator evaluates, so "gradient of what we compute" is the gradient the optimizer needs.

## Relationship to Ch 02's $\lambda$

The symbol $\lambda$ is shared across Ch 02 and Ch 03 by design, not by collision. In Ch 02 §00 it is the per-step adjoint $\lambda = A^{-1}\, \bar x^\ast$ — the back-substitution output at a single step given an upstream adjoint $\bar x^\ast$. In Ch 03 the discrete $\lambda_t$ is defined by the same formula $\lambda_t = A_t^{-1}\, \bar x_t$, with the upstream $\bar x_t$ built up across the trajectory as the derivation above showed.

The discrete $\lambda_t$ is therefore literally Ch 02 §00's $\lambda$ evaluated at each trajectory step, with the upstream assembled from downstream contributions rather than supplied from outside. The continuous $\lambda(t)$ is the $\Delta t \to 0$ limit of the same object: substituting the backward-Euler $A_t = I - \Delta t\,(\partial f/\partial x_t)$ and $\partial r_{t+1}/\partial x_t = -I$ into the discrete recursion and expanding to leading order recovers $\dot\lambda = -(\partial f/\partial x)^T \lambda - (\partial g/\partial x)^T$. No sign-convention mismatch, no relabeling — one symbol, one meaning, two scales.

The notation appendix's [collision note on $\lambda$](../../appendices/03-notation.md#notes-on-collisions-and-context) lists several other uses ($\lambda$ the Lamé parameter, $\lambda_k$ the normal-force magnitude at contact pair $k$) that are disambiguated by subscript or context; Ch 02's single-step $\lambda$ and Ch 03's trajectory $\lambda(t)$ or $\lambda_t$ are the same object at different scopes and are not part of that collision set.

## What this sub-leaf commits the book to

- **The trajectory gradient is a backward sweep over a single vector $\lambda_t$.** The continuous form is $\dot\lambda = -(\partial f/\partial x)^T \lambda - (\partial g/\partial x)^T$ with terminal $\lambda(T) = (\partial \Phi/\partial x)(x(T))$; the discrete form is the Ch 02-consistent recursion above with $A_t = H_t$. Both integrate backward at $O(n)$ per step, independent of $n_\theta$.
- **`sim-soft` commits to discretize-then-adjoint.** The adjoint of the backward-Euler Newton step reuses the forward's factored $H_t$ at every backward step; the alternative (adjoint-then-discretize) would break the factorization-reuse architecture by integrating the continuous adjoint with a scheme whose Jacobian samples do not match.
- **Per-step backward work is one back-substitution plus one inertial-block apply plus one $(\partial r_t/\partial \theta)^T \lambda_t$ contraction.** No per-step quantity scales with $n_\theta$; the total backward cost is $T$ times that work, plus whatever [Ch 04 checkpointing](../04-checkpointing.md) adds as forward-recomputation overhead.
- **The discrete $\lambda_t$ is literally Ch 02 §00's $\lambda$ chained across time.** The trajectory $\lambda_t$ is $A_t^{-1} \bar x_t$ at every step, with $\bar x_t$ assembled from local-reward plus downstream-propagated contributions. The continuous $\lambda(t)$ is the $\Delta t \to 0$ limit of that discrete object; no sign-convention flip separates the two.
