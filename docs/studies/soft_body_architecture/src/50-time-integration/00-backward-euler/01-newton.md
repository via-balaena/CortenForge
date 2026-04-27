# Newton iteration on total potential energy

The [implicit commitment](00-why-implicit.md) picks backward Euler but does not yet say how the nonlinear per-step equation is solved. This leaf picks that solver: classical Newton iteration on the gradient-zero equation of a scalar total-potential objective, with the factorization of the Hessian captured as a first-class, tape-persistent object so [Part 6 Ch 02's IFT adjoint](../../60-differentiability/02-implicit-function.md) can re-use it in the backward pass.

## Three things get introduced here

Three symbols are load-bearing downstream and land in this leaf:

- $U(x; \theta)$ — the total potential energy as a function of nodal positions $x$ and design parameters $\theta$. Elastic + contact + inertial contributions all live inside $U$; see the [spine page's Claim 1](../00-backward-euler.md) for the minimization framing.
- $x^\ast$ — the converged equilibrium position, defined by $\nabla_x U(x^\ast; \theta) = 0$.
- $K$ — the global sparse stiffness matrix, $K = \nabla^2_{xx} \Psi_\text{elastic}(x)$. This is the FEM stiffness in the conventional sense — elastic block only, no contact, no inertia. Its per-element contributions $K^e$ come from the [`Material::tangent()` trait method](../../20-materials/00-trait-hierarchy/00-trait-surface.md) and compose via the standard FEM assembly over the element connectivity.

The full per-step Newton Hessian is distinct from $K$ and gets its own symbol:

$$ H = K + H_\text{contact} + M/\Delta t^2 $$

with $H_\text{contact}$ the [barrier second derivative](../../40-contact/01-ipc-internals/00-barrier.md) (sparse in the active-contact pattern) and $M$ the lumped mass matrix. $H$ is the object Newton factors and inverts; $K$ is one of three blocks inside $H$. When later chapters refer to "the stiffness matrix" they mean $K$; when they refer to "the Hessian" they mean $H$.

## Minimization framing unpacks the backward-Euler residual

The semi-discrete backward-Euler residual is

$$ r_n(x) = M (x - \widehat x_n)/\Delta t^2 + \nabla \Psi_\text{elastic}(x) + \nabla \Psi_\text{contact}(x), \qquad \widehat x_n = x_{n-1} + \Delta t\, v_{n-1} $$

Setting $r_n(x) = 0$ recovers the implicit update. The spine page's Claim 1 is that this residual is itself the gradient of a scalar, specifically

$$ U_n(x) = \Psi_\text{elastic}(x) + \Psi_\text{contact}(x) + \frac{1}{2 \Delta t^2} (x - \widehat x_n)^T M (x - \widehat x_n) $$

and that solving $\nabla_x U_n(x^\ast) = 0$ is equivalent to solving $r_n(x^\ast) = 0$. The inertial quadratic is the kinetic-energy-plus-momentum contribution whose gradient *is* $M(x - \widehat x_n)/\Delta t^2$; its Hessian is the constant block $M/\Delta t^2$. No approximation has been made — the minimization and residual formulations solve the same nonlinear equation.

The benefit the minimization framing buys is that Armijo backtracking on a scalar objective is well-defined ([02-line-search.md](02-line-search.md)), that IPC lives inside $U_n$ as an added energy term rather than as an LCP constraint ([spine page Claim 1](../00-backward-euler.md)), and that the converged equilibrium condition $\nabla_x U_n(x^\ast; \theta) = 0$ is the implicit-function-theorem starting point [Part 6 Ch 02](../../60-differentiability/02-implicit-function.md) differentiates. None of these three properties survive cleanly under a purely residual-based framing.

## The Newton loop

At iterate $x^{(k)}$:

1. **Evaluate $g^{(k)} = \nabla_x U_n(x^{(k)}; \theta)$.** Elastic + contact + inertial contributions sum into the gradient vector.
2. **Assemble $H^{(k)} = \nabla^2_{xx} U_n(x^{(k)}; \theta)$.** Sum the three blocks: $K^{(k)}$ (per-element assembly), $H^{(k)}_\text{contact}$ (per-active-pair sparsity from the [broadphase](../../40-contact/03-self-contact/00-bvh.md)), and $M/\Delta t^2$ (constant).
3. **Project to SPD.** Apply Li 2020's per-element PSD projection (see the [Part 4 Ch 01 barrier anchor](../../40-contact/01-ipc-internals/00-barrier.md) for the Li 2020 cite) — eigendecompose each element's elastic tangent $K^e$ and clamp its negative eigenvalues to zero before assembly — so the assembled $H^{(k)}$ is SPD even when individual elastic tangents are locally indefinite at large compression. The [IPC adaptive $\kappa$](../../40-contact/01-ipc-internals/01-adaptive-kappa.md) handles the separate concern of keeping the barrier block well-conditioned under narrow gaps.
4. **Factor and solve.** Sparse Cholesky factor $H^{(k)} = L L^T$ using [faer](https://github.com/sarah-quinones/faer-rs); the factorization object is an owned, reusable handle (see the [code snippet](#the-per-step-forward-backward-pattern) below). Solve $L L^T \Delta x^{(k)} = -g^{(k)}$ via back-substitution against the factor.
5. **Line search.** [Armijo backtracking](02-line-search.md) along $\Delta x^{(k)}$, with [CCD step-length clip](../../40-contact/01-ipc-internals/02-ccd.md) for contact safety.
6. **Update and check convergence.** $x^{(k+1)} = x^{(k)} + \alpha\, \Delta x^{(k)}$; if $\|g^{(k+1)}\| < \text{tol}$, stop.

Quasi-Newton variants (L-BFGS, Broyden) are rejected; see the [spine page Claim 2](../00-backward-euler.md) for the full argument. The short version is that the true Hessian factorization is not a forward-only cost — Part 6 Ch 02 re-uses it in backward, so trading it away buys lower forward cost at the cost of losing the cheap-adjoint property.

## The per-step forward-backward pattern

Per Newton step, the Hessian is assembled once, factored once, re-applied across the line-search retries within the step, and persisted to the autograd tape for the backward pass to consume. The factor is a first-class `faer::sparse::linalg::solvers::Llt<usize, f64>` handle — owned, reusable, and pushed onto the tape via the [chassis `Tape::push_custom`](../../110-crate/01-traits/00-core.md) API (which binds the factor to the tape node at forward-pass time, not via a separate post-hoc registration):

```rust
use faer::linalg::solvers::SolveCore;
use faer::sparse::{SparseColMat, linalg::solvers::{Llt, SymbolicLlt}};
use faer::{Conj, MatMut, Side};
use sim_ml_chassis::{Tape, Tensor, Var};

pub struct NewtonStep {
    pub x_n: Tensor<f64>,    // converged position x^*
    pub x_n_var: Var,        // tape Var of x_n — parent of downstream cotangents
}

pub fn step(
    tape: &mut Tape,
    x_prev: &Tensor<f64>,
    v_prev: &Tensor<f64>,
    theta_var: Var,          // tape handle, not bare Tensor — the IFT VJP
    dt: f64,                 // pushes itself with theta_var as parent (post-PR-#213
                             // chassis API: Tape::push_custom binds parents at
                             // forward-pass time; see Part 11 Ch 01)
) -> NewtonStep {
    let theta = tape.value_tensor(theta_var).clone();   // primal value off the tape
    let mut x = predictor(x_prev, v_prev, dt);
    let factor = loop {
        let g = grad_U_n(&x, x_prev, v_prev, &theta, dt);
        let h: SparseColMat<usize, f64> = hessian_U_n(&x, &theta);   // PSD-projected
        // Symbolic factorization is faer 0.24's two-step pattern:
        // SymbolicLlt::try_new(symbolic_pattern, side) once per assembly-
        // pattern lifetime; Llt::try_new_with_symbolic per numeric refactor.
        // Production code caches the SymbolicLlt on the solver and reuses
        // across Newton iters (the pattern doesn't change with x).
        let symbolic = SymbolicLlt::<usize>::try_new(h.symbolic(), Side::Lower)?;
        let factor: Llt<usize, f64> =
            Llt::try_new_with_symbolic(symbolic, h.as_ref(), Side::Lower)?;
        let mut dx = -g.clone();
        let dx_mat: MatMut<'_, f64> = dx.as_mat_mut();
        factor.solve_in_place_with_conj(Conj::No, dx_mat);
        let alpha = line_search(&x, &dx, x_prev, v_prev, &theta, dt);
        x += alpha * dx;
        if g.norm() < CONVERGE_TOL { break factor; }
    };

    // Push the IFT VJP onto the tape with theta_var as parent. The VJP
    // owns the factor; tape.backward feeds the cotangent of x_n into
    // the VJP, which solves the adjoint A · λ = g_free in place via the
    // owned factor and contracts against (∂r/∂θ)_free in closed form.
    // See `NewtonStepVjp` in Part 11 Ch 01 for the production type.
    let x_n = Tensor::from_slice(&x, &[x.len()]);
    let vjp = NewtonStepVjp::new(factor /* + scene metadata: free-DOF
                                          indices, loaded-vertex map, etc. */);
    let x_n_var = tape.push_custom(&[theta_var], x_n.clone(), Box::new(vjp));
    NewtonStep { x_n, x_n_var }
}
```

This is the forward-side code for the pattern [Part 6 Ch 02](../../60-differentiability/02-implicit-function.md) shows on the backward side. Both sides read from the same `NewtonStepVjp` object: forward pushes it onto the tape via `Tape::push_custom` (the VJP owns the factor); backward (when `tape.backward` walks back to this node) invokes the VJP, which solves `A · λ = g_free` in place using the owned factor and contracts against `(∂r/∂θ)_free` for the gradient with respect to $\theta$. No second factorization, no re-assembly, no re-registration call — the `push_custom` API binds the factor to the tape node at forward time. The `SolveCore` trait from faer 0.24 exposes `solve_in_place_with_conj(Conj::No, rhs)` for in-place back-substitution; the same factor object can be re-applied to multiple right-hand-sides through this method (within Newton iter for the primal step, and once per upstream cotangent at backward time).

That reusability is the reason [Part 11 Ch 03 Phase B commits to faer specifically](../../110-crate/03-build-order.md#the-committed-order): a solver whose factorizations were private-by-API, discarded after the primal solve, or hidden behind a trait that only exposes `solve(rhs) -> x` would fail this pattern. Time integration is where the factor is *produced*; Part 6 Ch 02 is where it is *re-used*.

## Convergence regime in one line

Newton's local convergence on a smooth scalar objective is quadratic within the basin of attraction. The per-element PSD projection of the elastic block, the rank-1-per-pair positive contributions of the contact block, and the always-PD inertial block $M/\Delta t^2$ together keep $H^{(k)}$ SPD; the [Part 4 Ch 01 barrier smoothness](../../40-contact/00-why-ipc/02-ipc-guarantees.md) keeps $\nabla^2 U_n$ Lipschitz on the active set, so the local quadratic rate applies in principle. Whether a given step's initial iterate (the inertial predictor $\widehat x_n$) sits inside the basin is scene-dependent and is not assumed — [02-line-search.md](02-line-search.md) is what globalizes the method so that iterates outside the local basin still make monotone progress, and [Ch 02 adaptive-dt](../02-adaptive-dt.md) is the fallback when even line search cannot recover convergence within the current step.

## What this commits downstream

- **[Ch 02 adaptive-dt](../02-adaptive-dt.md)** triggers on Newton failure, not on an independent a-priori criterion. Halving retry after convergence failure is the primary adaptive path.
- **[Part 6 Ch 02 IFT](../../60-differentiability/02-implicit-function.md)** re-uses the `Llt<usize, f64>` factor owned by the `NewtonStepVjp` pushed onto the tape. The adjoint cost is one back-substitution per tape-step per upstream gradient, which is the property [Part 1 Ch 03 thesis](../../10-physical/03-thesis.md) calls "differentiability came for free."
- **[Part 8 Ch 02 sparse solvers](../../80-gpu/02-sparse-solvers.md)** extends the CPU factor-on-tape pattern to GPU preconditioned-CG; the tape contract is the same (store a reusable solver handle), the solver internals shift.
- **[Part 11 Ch 04 gradcheck](../../110-crate/04-testing/03-gradcheck.md)** validates the factor-on-tape wiring by comparing IFT-derived gradients against finite-difference gradients on a small static-equilibrium problem. The test verifies the wiring, not the IFT derivation itself.
