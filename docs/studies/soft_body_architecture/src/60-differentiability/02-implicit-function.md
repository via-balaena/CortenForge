# Implicit function theorem for solver gradients

The implicit function theorem (IFT) is the single load-bearing theorem of Part 6. It is why [Part 1 Ch 03's thesis](../10-physical/03-thesis.md) can claim that "differentiability came for free": once the solver is energy-based, the gradient of the equilibrium configuration with respect to any design parameter costs *one extra sparse solve* with the same matrix the forward Newton already factored. No reverse-mode tape over the Newton iterations themselves; no unrolling; no per-iteration graph. This chapter names what the IFT delivers at the top level and points at the three sub-chapters that derive, implement, and bound the memory cost of the construction.

| Section | What it covers |
|---|---|
| [Derivation $\partial x/\partial \theta = -A^{-1}\, \partial r/\partial \theta$](02-implicit-function/00-derivation.md) | The two-line theorem: at equilibrium $r(x^\ast; \theta) = 0$, implicit differentiation gives $\partial_\theta r + A\, \partial_\theta x^\ast = 0$, where $A = \partial_x r$ is the Jacobian of the residual; solve for $\partial_\theta x^\ast$ directly |
| [Linear solve for gradients](02-implicit-function/01-linear-solve.md) | How the forward [Newton iteration's factored matrix](../50-time-integration/00-backward-euler/01-newton.md) — faer's `SparseLU` or `SparseCholesky` — is re-applied to backward RHSes, one per downstream adjoint vector; cost is back-substitution, not factorization |
| [Memory cost](02-implicit-function/02-memory.md) | What lives in memory during backward: the factorization itself (computed once per converged Newton step), the residual Jacobian w.r.t. $\theta$ (sparse, computed lazily per downstream adjoint); what does *not* live in memory: the Newton iterate history |

Three claims Ch 02 rests on:

1. **The gradient is one solve, not a graph.** At a converged equilibrium, the residual $r(x^\ast; \theta) = 0$ holds exactly. Differentiating w.r.t. $\theta$ gives

   $$ \frac{\partial r}{\partial x}\, \frac{\partial x^\ast}{\partial \theta} + \frac{\partial r}{\partial \theta} = 0 \quad \Longrightarrow \quad \frac{\partial x^\ast}{\partial \theta} = -A^{-1}\, \frac{\partial r}{\partial \theta} $$

   where $A = \partial r / \partial x$ is *the same stiffness-and-contact Jacobian the forward Newton already assembled and factored to compute the last Newton step.* The gradient is therefore a back-substitution on an already-paid-for factorization, not a rebuild. For the canonical-problem-sized scene (~30k DOFs), that is a 10–30× speedup over re-assembling the tangent from scratch in backward.

2. **faer's re-usable factorization is load-bearing.** [Phase B](../110-crate/03-build-order.md#the-committed-order) committed to [faer](https://github.com/sarah-quinones/faer-rs) for the CPU sparse path specifically because its factorizations are first-class objects that survive the forward solve and can be applied to arbitrary RHSes in backward. The concrete pattern is:

   ```rust
   // forward: factor once, re-apply many times
   let factor: SparseLU<f64> = stiffness.factor_lu()?;
   for _ in 0..newton_iters {
       factor.solve_in_place(&mut rhs);
       // ... line search, convergence check ...
   }

   // backward: same factor held in the VJP closure, different RHS per upstream
   for upstream in downstream_adjoints {
       factor.solve_in_place(&mut minus_dr_dtheta.apply_t(&upstream));
   }
   ```

   The chassis-level [VJP registration API](01-custom-vjps/00-registration.md) stores the factorization handle as a captured resource in the backward closure; it is dropped when the tape is dropped. This is exactly the shape `sim-ml-chassis`'s tape extension makes cheap; it is exactly the shape Burn's tensor-first abstraction makes expensive.

3. **What the IFT does not handle is why [Ch 03](03-time-adjoint.md) and [Ch 05](05-diff-meshing.md) exist.** The IFT is the gradient at a *steady state*, for a *fixed mesh topology*, through a *smooth residual*. Transient trajectories require the [time adjoint](03-time-adjoint.md), not IFT. Design changes that alter mesh topology require [differentiable meshing](05-diff-meshing.md) — the book's open-problem chapter. Contact configurations where the [IPC active set](../40-contact/01-ipc-internals.md) changes mid-iteration violate the smoothness assumption; `sim-soft` addresses this by treating the active set as fixed across the IFT solve (valid at convergence, since the barrier smooths the transition) and by regression-testing gradients against finite differences on every active-set boundary the [gradcheck suite](../110-crate/04-testing/03-gradcheck.md) exercises.
