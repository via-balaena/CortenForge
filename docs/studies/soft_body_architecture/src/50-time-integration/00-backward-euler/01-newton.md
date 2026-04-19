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

Per Newton step, the Hessian is assembled once, factored once, re-applied across the line-search retries within the step, and stored on the autograd tape for the backward pass to consume. The factor is a first-class `faer::sparse::linalg::solvers::Llt` handle — owned, reusable, and explicitly persisted:

```rust
use faer::sparse::{SparseColMat, linalg::solvers::Llt};
use faer::linalg::solvers::Solve;
use sim_ml_chassis::{Tape, Tensor};

pub struct NewtonStep {
    pub x_n: Tensor<f64>,               // converged position x^*
    pub factor: Llt<f64>,               // captured Hessian factor at x^*
    pub dr_dtheta: SparseColMat<f64>,   // residual Jacobian w.r.t. design params
}

pub fn step(
    tape: &mut Tape,
    x_prev: &Tensor<f64>,
    v_prev: &Tensor<f64>,
    theta: &Tensor<f64>,
    dt: f64,
) -> NewtonStep {
    let mut x = predictor(x_prev, v_prev, dt);
    let factor = loop {
        let g = grad_U_n(&x, x_prev, v_prev, theta, dt);
        let h = hessian_U_n(&x, theta);              // SparseColMat<f64>, PSD-projected
        let factor: Llt<f64> = Llt::try_new_with_symbolic(&h)?;
        let mut dx = -g.clone();
        factor.solve_in_place(&mut dx);
        let alpha = line_search(&x, &dx, x_prev, v_prev, theta, dt);
        x += alpha * dx;
        if g.norm() < CONVERGE_TOL { break factor; }
    };

    let dr_dtheta = residual_jacobian_theta(&x, theta);   // sparse
    let step = NewtonStep { x_n: x, factor, dr_dtheta };
    tape.record_ift_step(&step);   // Part 6 Ch 02's adjoint consumes (factor, dr_dtheta)
    step
}
```

This is the forward-side code for the pattern [Part 6 Ch 02](../../60-differentiability/02-implicit-function.md) shows on the backward side. Forward stores `factor` and `dr_dtheta` on the tape; backward calls `factor.solve_in_place(&mut (-dr_dtheta.transpose() * upstream))` to get the gradient with respect to $\theta$. No second factorization, no re-assembly. The `Solve<T>` trait from faer exposes both `solve(&self, rhs)` (owned output) and `solve_in_place(&self, rhs)` (in-place back-substitution on the RHS), both of which can be called arbitrarily many times on the same factor object against different right-hand-sides.

That reusability is the reason [Part 11 Ch 03 Phase B commits to faer specifically](../../110-crate/03-build-order.md#the-committed-order): a solver whose factorizations were private-by-API, discarded after the primal solve, or hidden behind a trait that only exposes `solve(rhs) -> x` would fail this pattern. Time integration is where the factor is *produced*; Part 6 Ch 02 is where it is *re-used*.

## Convergence regime in one line

Newton's local convergence on a smooth scalar objective is quadratic within the basin of attraction. The per-element PSD projection of the elastic block, the rank-1-per-pair positive contributions of the contact block, and the always-PD inertial block $M/\Delta t^2$ together keep $H^{(k)}$ SPD; the [Part 4 Ch 01 barrier smoothness](../../40-contact/00-why-ipc/02-ipc-guarantees.md) keeps $\nabla^2 U_n$ Lipschitz on the active set, so the local quadratic rate applies in principle. Whether a given step's initial iterate (the inertial predictor $\widehat x_n$) sits inside the basin is scene-dependent and is not assumed — [02-line-search.md](02-line-search.md) is what globalizes the method so that iterates outside the local basin still make monotone progress, and [Ch 02 adaptive-dt](../02-adaptive-dt.md) is the fallback when even line search cannot recover convergence within the current step.

## What this commits downstream

- **[Ch 02 adaptive-dt](../02-adaptive-dt.md)** triggers on Newton failure, not on an independent a-priori criterion. Halving retry after convergence failure is the primary adaptive path.
- **[Part 6 Ch 02 IFT](../../60-differentiability/02-implicit-function.md)** re-uses the `Llt<f64>` factor stored on the tape. The adjoint cost is one back-substitution per tape-step per upstream gradient, which is the property [Part 1 Ch 03 thesis](../../10-physical/03-thesis.md) calls "differentiability came for free."
- **[Part 8 Ch 02 sparse solvers](../../80-gpu/02-sparse-solvers.md)** extends the CPU factor-on-tape pattern to GPU preconditioned-CG; the tape contract is the same (store a reusable solver handle), the solver internals shift.
- **[Part 11 Ch 04 gradcheck](../../110-crate/04-testing/03-gradcheck.md)** validates the factor-on-tape wiring by comparing IFT-derived gradients against finite-difference gradients on a small static-equilibrium problem. The test verifies the wiring, not the IFT derivation itself.
