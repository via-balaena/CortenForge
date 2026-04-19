# Derivation ∂x/∂θ = -A⁻¹ ∂r/∂θ

The converged state of a backward-Euler step, $x^\ast$, satisfies the first-order condition $\nabla_x U_n(x^\ast; \theta) = 0$ of the step minimization in [Ch 00 §01 Newton](../../50-time-integration/00-backward-euler/01-newton.md). Writing the residual $r(x; \theta) \equiv \nabla_x U_n(x; \theta)$ makes the equilibrium condition into a nonlinear root $r(x^\ast; \theta) = 0$. The implicit function theorem differentiates that root condition with respect to the design parameters $\theta$ and delivers a closed-form expression for $\partial x^\ast / \partial \theta$ as the solution of a single linear system against the Newton Hessian — the same object the forward solve just factored. This sub-leaf writes the two-line derivation, states the smoothness conditions the theorem requires, and names how the reverse-mode adjoint form ($\bar\theta$ given upstream $\bar x^\ast$) drops out by transposition.

## Two lines

Total-derivative $r(x^\ast(\theta); \theta) = 0$ with respect to $\theta$:

$$ \frac{\partial r}{\partial x}\, \frac{\partial x^\ast}{\partial \theta} + \frac{\partial r}{\partial \theta} = 0. $$

With $A \equiv \partial r/\partial x$ nonsingular, rearrange:

$$ \frac{\partial x^\ast}{\partial \theta} = -A^{-1}\, \frac{\partial r}{\partial \theta}. $$

Because $r = \nabla_x U_n$, its $x$-Jacobian is the Newton Hessian $A = \partial r/\partial x = \nabla^2_{xx} U_n = H = K + H_\text{contact} + M/\Delta t^2$ from [Ch 00 §01 Newton](../../50-time-integration/00-backward-euler/01-newton.md). No second Hessian, no second assembly — the same $H$ the last Newton iterate just factored.

## Reverse-mode form drops out by transposition

The forward expression $\partial x^\ast / \partial \theta = -A^{-1}\, \partial r/\partial \theta$ is a Jacobian: one column per component of $\theta$. Reverse-mode autograd does not need the Jacobian explicitly — it consumes an upstream adjoint $\bar x^\ast = \partial L / \partial x^\ast$ (one vector) and produces a gradient $\bar\theta = \partial L / \partial \theta$ (another vector). Chain rule gives

$$ \bar\theta = \left(\frac{\partial x^\ast}{\partial \theta}\right)^T \bar x^\ast = -\left(\frac{\partial r}{\partial \theta}\right)^T A^{-T}\, \bar x^\ast. $$

For the backward-Euler step, $A = H$ is SPD by construction (per-element PSD-projected elastic block + always-PD inertial block + rank-1-per-pair positive contact contributions, per [Ch 00 §01 Newton](../../50-time-integration/00-backward-euler/01-newton.md)), so $A^{-T} = A^{-1}$. Define $\lambda = A^{-1}\, \bar x^\ast$ as the intermediate adjoint solution; then

$$ \bar\theta = -\left(\frac{\partial r}{\partial \theta}\right)^T \lambda. $$

The full backward pass is two steps: (1) solve $A\, \lambda = \bar x^\ast$ — one back-substitution against the forward's factor, covered in [§01 linear solve](01-linear-solve.md); (2) contract $\lambda$ with the residual-$\theta$-Jacobian $\partial r/\partial \theta$ — the FEM-assembly and contact-barrier VJPs of [Ch 01 §01](../01-custom-vjps/01-fem-assembly.md) and [§02](../01-custom-vjps/02-contact-barrier.md) are what compute that contraction. A parallel composition against $\partial r/\partial x_{n-1}$ and $\partial r/\partial v_{n-1}$ produces the gradients flowing back into the previous step's state; those are the inputs the [time adjoint](../03-time-adjoint.md) chains across the trajectory.

## What the theorem requires

The IFT is not unconditional. Three smoothness-and-regularity conditions have to hold at $x^\ast$:

1. **$A$ is nonsingular.** The step Hessian $H$ is SPD at the converged iterate for the reasons in [Ch 00 §01 Newton](../../50-time-integration/00-backward-euler/01-newton.md); SPD implies nonsingular. The SPD property is itself load-bearing for the forward Newton's descent guarantee, so "forward solve converged" implies "IFT's nonsingularity condition holds." The two conditions collapse into one check.

2. **$r$ is $C^1$ in a neighborhood of $(x^\ast, \theta)$.** The elastic and inertial terms are $C^\infty$ in $x$ for the neo-Hookean family [Part 2 Ch 04](../../20-materials/04-hyperelastic.md) ships; the $\theta$ dependence through material parameters $\mathbf{p}_e$ is smooth by the [material trait surface](../../20-materials/00-trait-hierarchy/00-trait-surface.md)'s construction. The one non-trivial case is the IPC contact term: the composed contact energy $\Psi_\text{contact}(x) = \sum_k \kappa\, b(d_k(x), \hat d_k)$ is $C^2$-smooth on the interior of the active-feasible domain $\{x : d_k(x) > 0 \text{ for every active pair } k\}$ by the barrier's construction ([Li et al. 2020](../../appendices/00-references/00-ipc.md#li-2020); the formula $b(d) = -(d - \hat d)^2 \ln(d/\hat d)$ is $C^2$ on $(0, \hat d)$ with $b(\hat d) = b'(\hat d) = b''(\hat d) = 0$, so extending by zero outside yields a globally $C^2$ composed energy). At a converged equilibrium no active pair has $d_k = 0$ — that would be a strict-infeasibility violation IPC's barrier construction forbids — so $x^\ast$ is an interior point of the smooth domain and $r$ is $C^1$ there.

3. **The active set is locally fixed.** The composed barrier sums over the set of pairs with $0 < d_k < \hat d_k$ — the primitive pairs whose proximity gap has entered the barrier's stencil but not yet reached contact. Both bounds are strict inequalities, so small perturbations of $\theta$ that perturb $x^\ast$ (and hence every $d_k(x^\ast; \theta)$) continuously cannot move any pair across either bound under a sufficiently-small perturbation — the active set is locally constant in $\theta$. The IFT runs against the residual whose composition is fixed by the active set resolved during the forward's last Newton iterate; that active set is part of what the VJP node's captured state holds (per [Ch 01 §00 registration](../01-custom-vjps/00-registration.md)'s "per-pair $\hat d$" and broader `State` surface).

The third condition is where finite-distance active-set changes between forward and backward could otherwise slip in. The mitigation is [Part 11 Ch 04 gradcheck](../../110-crate/04-testing/03-gradcheck.md): gradcheck boundary tests exercise parameter perturbations that span active-set transitions; the central-finite-difference reference evaluates a fully re-solved $x^\ast$ at each perturbed $\theta$ (so it *does* see the transition), while the IFT gradient holds the active set fixed. Agreement to 5–6 digits on those boundary tests is what confirms that the active-set-fixed assumption is practically harmless at the tolerances downstream machinery operates at.

## What this sub-leaf commits the book to

- **The IFT is the gradient definition for every backward-Euler step.** Reverse-mode autograd does not unroll the Newton loop; the step's tape node is opaque (per [Ch 01 §00 registration](../01-custom-vjps/00-registration.md)), and its backward closure evaluates the IFT formula. The forward Newton loop's only job is producing $x^\ast$ and the factored $A = H$; it is not a recorded graph.
- **The SPD property that makes forward Newton converge is also what makes the IFT adjoint run.** Nonsingular $A$ at $x^\ast$ is a precondition for both; [Ch 00 §01 Newton](../../50-time-integration/00-backward-euler/01-newton.md)'s per-element PSD projection is the mechanism that ensures it. The forward solve and the IFT adjoint share their regularity guarantee.
- **Smoothness assumptions are documented, not assumed silently.** The composed-barrier $C^2$ property ([Li 2020](../../appendices/00-references/00-ipc.md#li-2020)) is the one non-textbook assumption; the locally-fixed active set is the one that gradcheck specifically validates. Violations outside those two are the topic of [Ch 05 differentiable meshing](../05-diff-meshing.md), not this chapter.
