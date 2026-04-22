# Linear solve for gradients

The [§00 derivation](00-derivation.md) ends with a closed-form adjoint: given upstream $\bar x^\ast$, solve $A\, \lambda = \bar x^\ast$ (one linear system against the SPD Newton Hessian $A = H$), then contract $\lambda$ with the $\theta$-branch residual Jacobian $\partial r/\partial \theta$ to get $\bar\theta$. This sub-leaf writes out how the linear solve actually happens: which faer object carries the factorization from forward into backward, how back-substitution replaces factorization in the cost model, and why the choice of solver with a first-class reusable factor is what makes the adjoint cheap at `sim-soft`'s scene sizes rather than the same cost as a second forward solve.

## The factor that the forward produced

[Ch 00 §01 Newton](../../50-time-integration/00-backward-euler/01-newton.md) ends each converged step with a `NewtonStep` struct holding the converged position $x^\ast$, the factored Hessian, and the sparse residual-$\theta$-Jacobian:

```rust
pub struct NewtonStep {
    pub x_n: Tensor<f64>,               // converged position x^*
    pub factor: Llt<f64>,               // captured Hessian factor at x^*
    pub dr_dtheta: SparseColMat<f64>,   // residual Jacobian w.r.t. design params
}
```

The `factor` field is the output of `Llt::try_new_with_symbolic(&h)?` applied to the PSD-projected Hessian at convergence. It carries the sparse Cholesky decomposition $H = L L^T$ as an owned object — not a boolean "was factored," not a borrow of the matrix, but the $L$ factor itself with its supernodal structure, permutation, and back-substitution kernels. The faer type `faer::sparse::linalg::solvers::Llt<f64>` implements the `Solve` trait from `faer::linalg::solvers`, which exposes both a `solve` method (returning an owned solution) and a `solve_in_place` method (writing back-substitution output over the RHS buffer); both can be called any number of times on the same factor against different right-hand sides.

The step's tape node owns the `factor` and `dr_dtheta` as struct fields on the [Ch 01 §00 registration API](../01-custom-vjps/00-registration.md)'s `VjpOp` impl — concretely:

```rust
struct BackwardEulerStepVjp {
    parents:   Vec<u32>,                // parent Var indices (x_prev, v_prev, theta)
    factor:    Llt<f64>,                // forward's last factorization
    dr_dtheta: SparseColMat<f64>,       // residual Jacobian w.r.t. theta
    // ... enough state for dr/dx_{n-1}, dr/dv_{n-1} composition ...
}
```

captured at the time the forward step finishes and released when the tape is dropped. Between forward and backward, the factor sits in memory; no re-factorization happens in backward.

## Backward: one solve per upstream adjoint

The `vjp` method fires once per tape backward pass per step. It receives an upstream cotangent $\bar x^\ast$ on the step's output and writes gradients into the parent cotangent slots — a back-substitution followed by a sparse-matrix contraction per parent:

```rust
impl VjpOp for BackwardEulerStepVjp {
    fn op_id(&self) -> &'static str { "backward_euler_step" }
    fn parents(&self) -> &[u32] { &self.parents }

    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        // cotangent = upstream bar_x_star on the post-step state, shape [n_dof].
        // parent_cotans = [bar_x_prev, bar_v_prev, bar_theta] — slots to accumulate into.

        // (1) back-substitution: lambda = A^{-1} bar_x_star.
        let mut lambda = cotangent.clone();
        self.factor.solve_in_place(&mut lambda);

        // (2) accumulate -(dr/dtheta)^T * lambda into bar_theta slot.
        parent_cotans[2].sub_assign(&(self.dr_dtheta.transpose() * &lambda));

        // (3) accumulate bar_x_prev and bar_v_prev from lambda against
        //     the inertial-block Jacobians into parent_cotans[0], parent_cotans[1]
        //     (omitted).
    }
}
```

Line 1 is the back-substitution — `solve_in_place` applies $L^{-T} L^{-1}$ to the RHS in place against the already-factored matrix. Line 2 is a sparse-matrix-transpose-vector product: the forward stored `dr_dtheta` directly at convergence via the `residual_jacobian_theta` assembly call in [Ch 00 §01](../../50-time-integration/00-backward-euler/01-newton.md)'s `NewtonStep` pattern, so the backward closure consumes the pre-assembled matrix without re-running any adjoint machinery. The per-element material-parameter columns come out of the same closed-form per-element $\partial f^e_\text{int}/\partial \mathbf{p}_e$ expressions that the [FEM assembly VJP](../01-custom-vjps/01-fem-assembly.md) uses for its own backward; the contact-parameter columns (if any $\theta$ component adjusts contact parameters like per-pair $\hat d_k$) come out of the barrier-derivative formulas the [contact-barrier VJP](../01-custom-vjps/02-contact-barrier.md) uses. The matrix is sparse because the residual's dependence on each $\theta$ component is local — one material-parameter entry per element touches only that element's rows; one contact-pair $\hat d_k$ touches only the rows of that pair's endpoints.

If the same tape node's output is consumed by more than one downstream path (a rollout objective summed across timesteps is the canonical case), the chassis backward traversal accumulates every upstream into a single $\bar x^\ast$ before firing the `vjp` method, per the [§00 registration API](../01-custom-vjps/00-registration.md)'s cotangent-accumulation semantics. One `vjp` invocation per tape backward, one back-substitution, regardless of how many downstream paths contributed.

## Cost: backward pays for back-substitution, not factorization

Sparse Cholesky factorization of an SPD matrix of $n$ DOFs has cost dominated by the fill-in of the $L$ factor — the number of nonzeros $\text{nnz}(L)$ after a fill-reducing ordering is applied. Back-substitution against the already-computed factor is a pair of triangular solves whose per-RHS cost is linear in $\text{nnz}(L)$. For 3D FEM stencils the factorization involves substantially more work per nonzero than the triangular solve does — factorization is asymptotically more expensive than back-substitution as $n$ grows, and the practitioner experience across differentiable-FEM toolchains is that the ratio is large enough to justify the whole "factor once, solve many times" architectural pattern.

The concrete ratio on the canonical problem is a [Phase-B benchmarking deliverable](../../110-crate/03-build-order.md#the-committed-order), not a claim this chapter anchors on a specific multiplier. The previous generation of the Ch 02 spine carried a "10–30× speedup" line; that number was not sourced against any published measurement in the IPC or adjoint-FEM literature and has been dropped in favor of the qualitative framing. The same discipline applies here as to the tape-dispatch claims in [Ch 01 §02 barrier VJP](../01-custom-vjps/02-contact-barrier.md) and [Ch 00 §02 why-not-Burn](../00-what-autograd-needs/02-why-not-burn.md): qualitative and large, with a Phase-B measurement as the anchor.

The alternative — a solver that computes $H$, calls an opaque `solve(rhs) -> x` method, and throws away the internal factorization — would force the backward pass to re-factor. That flips the cost model from "cheap adjoint" to "second forward solve," which is the scaling [Part 1 Ch 03 thesis](../../10-physical/03-thesis.md) explicitly rejects. The commitment to faer is specifically a commitment to a solver that exposes the factor as a first-class object whose lifetime is controlled by the caller, not by the solver's internal API.

## Why `solve_in_place` rather than `solve`

Both methods exist on faer's `Solve` trait; they deliver the same mathematical result. `solve` allocates a fresh output vector per call; `solve_in_place` writes the result over the RHS buffer. For the forward Newton loop (multiple primal solves per step), `solve_in_place` on a reusable scratch buffer avoids per-iteration allocation — that is the pattern Ch 00 §01's code snippet shows. For the backward closure (one primal solve per upstream), either is correct; `solve_in_place` is the same pattern for consistency, and the `grad_outputs` slice the closure received is the RHS buffer the back-substitution writes $\lambda$ into.

The method choice is an optimization detail, not an architectural commitment. What the book commits to is the reuse contract: forward factors once, backward back-substitutes arbitrarily-many-times against the same factor. Which exact faer method carries the back-substitution is a mechanical choice inside that contract.

## What this sub-leaf commits the book to

- **The forward's `Llt<f64>` factor is tape-persisted, not discarded.** The factor survives the forward Newton loop's completion, travels into the tape node's captured `State`, and is released only when the tape is dropped. Re-factoring in backward is forbidden by the cost model.
- **Backward cost is one back-substitution per upstream adjoint.** The specific wall-clock ratio of back-substitution to factorization is deferred to Phase-B benchmarking; the scaling is "significantly cheaper than forward," not a specific multiplier.
- **The `dr/dtheta` sparse matrix is forward-stored, backward-contracted.** The residual-$\theta$-Jacobian is assembled during the forward step (sparse pattern determined by mesh connectivity + active contact pairs), stored in the step's tape node via the [§00 registration API's `State`](../01-custom-vjps/00-registration.md), and contracted against $\lambda$ in backward as a single sparse-matrix-transpose-vector product. No per-backward re-evaluation of the residual's $\theta$-dependence.
- **Per-element VJPs in [Ch 01 §01](../01-custom-vjps/01-fem-assembly.md) and [§02](../01-custom-vjps/02-contact-barrier.md) are utilities, not tape-visible nodes.** They are called from inside this VJP's backward closure whenever the residual Jacobian needs (re-)evaluation or contraction — not as standalone tape ops. The IFT wrapper is the opaque tape node; assembly and barrier adjoints are its internal helpers.
