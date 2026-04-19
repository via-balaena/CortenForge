# What physics-specific autograd needs beyond that

The five failure modes closing [the previous sub-chapter](00-generic.md) — iteration-count-dependent convergence, trajectory-length tape blowup, per-element FEM assembly, divergent-barrier numerics, topology discontinuities — are not weaknesses of reverse-mode AD. Reverse-mode is doing exactly what it promises: chain-rule composition over a recorded graph of elementary ops. The problem is that the physics side of `sim-soft` is not a chain of elementary ops. It is an algorithm whose *gradient is a theorem*, a kernel whose *adjoint collapses to a short closed form*, or a transformation whose *derivative does not exist in the classical sense*. This sub-chapter names the five places that gap appears and points at the sub-chapter of Part 6 that closes each one.

## Gradient through an implicit solve — IFT, not unrolling

A backward-Euler step solves $r(x^\ast; \theta) = 0$ for $x^\ast$ via Newton iteration on the residual $r$. The iteration terminates when $\|r(x)\| < \varepsilon$; that termination condition makes the iteration count data-dependent, and the converged state $x^\ast$ is a function of $\theta$ *defined implicitly* by the root condition, not *computed explicitly* as a chain of ops. Unrolling the Newton loop into the autograd tape does compute a gradient that matches the implicit one at full convergence, but the tape stores the full iterate history whose only purpose was getting $x$ to the root — memory scales linearly with iteration count, and any early termination makes the stored gradient wrong.

The implicit function theorem gives the right gradient as a theorem. Differentiating $r(x^\ast(\theta); \theta) = 0$ w.r.t. $\theta$ and rearranging yields

$$ \frac{\partial x^\ast}{\partial \theta} = -\left(\frac{\partial r}{\partial x}\right)^{-1}\, \frac{\partial r}{\partial \theta}, $$

and the Jacobian $\partial r / \partial x$ is the *same matrix* the forward Newton's last iteration already factored. The gradient costs one back-substitution per downstream adjoint vector, no iteration history needed. Similar constructions anchor deep equilibrium models ([Bai et al. 2019](../../appendices/00-references/02-adjoint.md#bai-2019)) and the [JAXopt](https://jaxopt.github.io) implicit-differentiation layer ([Blondel et al. 2022](../../appendices/00-references/02-adjoint.md#blondel-2022)); the mathematical kernel is much older. [Ch 02](../02-implicit-function.md) derives the construction, names faer's reusable factorization as the pattern that makes it cheap, and notes the assumptions (smooth residual, fixed active set) the theorem requires.

## Backward-in-time trajectories — adjoint ODE, not tape

A trajectory objective $L = \int_0^T g(x(t); \theta)\, \mathrm dt$ (integrated reward over a squeeze-and-release cycle, say) depends on every intermediate $x(t)$ in a 1000-step rollout. Reverse-mode AD over the rollout requires either storing every intermediate state — $O(T)$ memory, prohibitive at `sim-soft` sizes — or recomputing forwards between reverse steps, which is the right idea but needs dedicated bookkeeping (the topic of [Ch 04](../04-checkpointing.md)). The right construction is the *adjoint ODE*: a backward-in-time integration of an auxiliary variable $\lambda(t)$ that carries the objective's sensitivity from the terminal state back to the parameters. The continuous form reads $\dot\lambda = -(\partial f / \partial x)^T \lambda - (\partial g / \partial x)^T$, terminal condition $\lambda(T) = (\partial \Phi / \partial x)(x(T))$; integrating it backwards one step per forward step delivers the full trajectory gradient at a per-step cost that is independent of parameter dimension.

The adjoint ODE couples to the per-step IFT — every backward step of the adjoint solves against the Jacobian at the forward step's converged state, the same Jacobian the forward already factored. [Ch 03](../03-time-adjoint.md) writes this out, notes the break-even where reverse beats forward sensitivity (roughly $n_\theta \gtrsim 20$ for `sim-soft`), and names the stochastic-adjoint extension that `sim-thermostat`'s Brownian forcing requires.

## Bounded-memory backward — checkpointing, not full-state storage

Even with the adjoint ODE, the backward pass needs access to the forward state $x_t$ at every step to evaluate the Jacobian coefficients — so the naive implementation still stores every $x_t$ and every factorization along the forward trajectory. At 30k DOFs the per-step footprint (positions, velocities, assembled Hessian, Cholesky factor, contact active-set metadata) runs to hundreds of megabytes, and a 1000-step rollout allocates tens of gigabytes before backward starts. That exhausts a workstation's VRAM budget and pushes the CPU path toward the edge of host RAM.

Checkpointing is the classical answer. [Griewank & Walther 2000](../../appendices/00-references/02-adjoint.md#griewank-walther-2000)'s Revolve algorithm places $O(\log T)$ checkpoints along the forward trajectory and re-runs the forward between them during backward, achieving $O(\log T)$ memory and $O(T \log T)$ compute with provably optimal constants for fixed checkpoint budgets. The same construction underlies [adolC](https://github.com/coin-or/ADOL-C), [Dolfin-adjoint](https://www.dolfin-adjoint.org/), and JAX's `jax.checkpoint`. [Ch 04](../04-checkpointing.md) names the `sim-soft` build-order policy: uniform every-$k$-step checkpointing ships in Phase D, Revolve replaces it in Phase E when the GPU-VRAM constraint forces the issue.

## Hand-written VJPs for fused kernels — adjoint, not graph

FEM stiffness assembly is $K = \sum_e B_e^T\, C_e\, B_e\, V_e$ — a sum over thousands of per-element contributions, each a few dozen scalar multiplies. A scalar-level autograd tape records one node per multiply, producing a tape of size $O(\text{elements} \times \text{scalars-per-element})$ whose backward dispatch overhead can dominate the actual arithmetic; a tensor-level tape still pays per-element dispatch. Yet the analytical adjoint of the whole assembly is a single pass over the connectivity list with a gradient accumulator — a handful of lines of code whose reverse pass costs the same as the forward.

The IPC barrier $b(d) = -(d - \hat d)^2 \ln(d/\hat d)$ has a different pathology. The expression is smooth on $(0, \hat d)$, with $b(\hat d) = 0$ at the smooth junction and $b(d) \to +\infty$ as $d \to 0^+$. Double-precision evaluation is fragile in two distinct regimes: catastrophic cancellation at the smooth junction $d = \hat d$ where the factors of $b'(d)$ vanish simultaneously, and Hessian ill-conditioning near $d \to 0^+$ where $b''(d) \propto 1/d^2$ diverges by design. The [Part 6 Ch 01 §02 barrier VJP](../01-custom-vjps/02-contact-barrier.md) evaluates the closed-form derivatives the same way the [IPC Toolkit](../../appendices/00-references/00-ipc.md#ipc-toolkit) does — direct algebra, no log1p recentering, no Taylor expansion — and the barrier-variant choice (clamped log, normalized, quadratic log, cubic, two-stage) is a Phase B decision based on condition-number measurements on the canonical problem.

Both kernels need *hand-written* VJPs: derivatives worked out on paper, implemented directly beside the forward, and registered with the tape as an atomic node that fires the backward as a single closure. [Ch 01](../01-custom-vjps.md) specifies the registration API — the `CustomVjp` trait extension to `sim-ml-chassis`'s tape — and works through the FEM-assembly and IPC-barrier VJPs as the two concrete cases `sim-soft` owns.

## Topology discontinuities — not differentiable at all

The fifth failure mode is categorically different. A marching-cubes-style SDF-to-mesh pipeline chooses a mesh-topology pattern for each grid cell based on the sign pattern of the SDF at the cell corners. When an SDF value crosses zero, the cell's topology jumps — a vertex appears or disappears, an element subdivision pattern changes. The mesh-as-a-function-of-SDF-parameters is literally discontinuous at those crossings; its classical derivative is a Dirac delta at the crossing and zero elsewhere. No amount of chain rule rescues this.

Autograd cannot be asked to differentiate what is not differentiable. The honest answer is to name the gap, bound its practical impact, and choose an architecture that avoids dense gradient flow through it. [Ch 05](../05-diff-meshing.md) is that chapter: it names five published approaches to soft-differentiable-meshing, commits `sim-soft` to a three-part compromise (topology-fixed-per-episode, between-episode re-meshing with warm-started state transfer, finite-difference wrappers at topology boundaries), and flags the clean solution as multi-year research.

## Autograd's job is the boundary

The pattern across the five is consistent: each has a non-autograd construction — IFT, adjoint ODE, Revolve, hand-derived VJP, topology-fixed wrapper — that delivers the gradient as *structure*, not as a recorded graph. Autograd is not replaced; it is used *at the boundary*. The physics solver's forward pass produces a scalar (or a small tensor) that feeds into the policy network or the objective function. The autograd tape records that boundary quantity, and the solver's hand-derived VJP — registered as a single opaque tape node — fires during backward with the same gradient the theorem would have produced.

Concretely, the tape sees something like this for a single backward-Euler step:

```text
tape: [ ... | backward_euler_step(x_prev, θ) → x_next | ... ]
              ↑ one opaque node, forward calls the Newton loop,
                backward applies the IFT construction with the
                factorization captured from the forward.
```

The chassis tape does not know that this one node hides several Newton iterates, a sparse Cholesky factorization, and an IPC barrier evaluation. It knows that the node has a forward that produces $x_{\text{next}}$ and a backward that takes an upstream adjoint $\bar x_{\text{next}}$ and returns $\bar x_{\text{prev}}$ and $\bar\theta$. The VJP is a closure; the closure captures the factorization handle; the handle holds the factored Jacobian alive as long as the tape owns the node. That is the shape [Ch 01's registration API](../01-custom-vjps.md) specifies.

Generic reverse-mode autograd is necessary — it is how the chain rule composes solver-level gradients with policy-network gradients at the RL loop's outer boundary. It is not sufficient. The remaining chapters of Part 6 supply the missing five constructions.
