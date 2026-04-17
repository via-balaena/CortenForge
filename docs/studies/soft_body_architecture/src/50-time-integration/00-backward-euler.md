# Backward Euler with Newton

Time integration is the pivot point of the whole solver. The [Ch 03 thesis](../10-physical/03-thesis.md) commits to energy-based physics, and the energy only becomes a running simulation once a timestepping rule is chosen — everything upstream of this chapter is the description of $U(x; \theta)$, and everything downstream of this chapter is a consequence of how we march it forward in time. This chapter commits to **backward Euler, solved as Newton iteration on a scalar objective, with line-search backtracking.** One scheme, stated three ways; no explicit integrators shipped, no mid-point hybrids, no "adaptive order" wrappers that switch scheme based on stiffness. The commitment is narrow on purpose, and the sub-chapters derive the minimization framing, wire Newton onto it, and bound step lengths via line search.

| Section | What it covers |
|---|---|
| [Why fully implicit](00-backward-euler/00-why-implicit.md) | The stability argument: explicit schemes need $\Delta t \lesssim 1/\sqrt{k_{\max}}$, which for neo-Hookean silicone at Phase B mesh resolutions is 10–100 μs; backward Euler is unconditionally stable at $\Delta t = 16$ ms |
| [Newton iteration on total potential energy](00-backward-euler/01-newton.md) | Minimizing $U_n(x) = \Psi_\text{elastic}(x) + \Psi_\text{contact}(x) + \Psi_\text{inertia}(x; x_{n-1}, v_{n-1}, \Delta t)$ each step; how the inertial term encodes backward Euler as a minimization problem; tangent = Hessian |
| [Line search backtracking](00-backward-euler/02-line-search.md) | Armijo backtracking on $U_n$ for monotonic decrease, combined with IPC's [CCD-limited step](../40-contact/01-ipc-internals/02-ccd.md) so no Newton step crosses a barrier; what "failed line search" triggers upstream ([Ch 02 adaptive dt](02-adaptive-dt.md)) |

Four claims Ch 00 rests on. Each is load-bearing for something downstream.

## 1. Backward Euler on total potential energy is an unconstrained minimization, not a root-find

The standard "backward Euler" presentation frames the timestep as finding $x_n$ such that a vector residual vanishes — $M(x_n - x_{n-1} - \Delta t\, v_{n-1}) - \Delta t^2\, f(x_n) = 0$ — and solves it with Newton's method on the residual. `sim-soft` does not frame it that way. Instead, the step solves

$$ x_n = \arg\min_x\, U_n(x) $$

where

$$ U_n(x) = \Psi_\text{elastic}(x) + \Psi_\text{contact}(x) + \frac{1}{2\Delta t^2}\, (x - \widehat x_n)^T M\, (x - \widehat x_n) $$

and $\widehat x_n = x_{n-1} + \Delta t\, v_{n-1}$ is the inertial predictor. The last term is the kinetic-energy-plus-momentum contribution; its gradient is $M(x - \widehat x_n)/\Delta t^2$, and its Hessian is $M/\Delta t^2$. Setting $\nabla U_n(x) = 0$ recovers the standard backward-Euler residual exactly. The minimization framing and the root-find framing agree on the solution; they disagree on the machinery for getting there.

The minimization framing is load-bearing for three reasons.

**Line search is meaningful.** A scalar objective admits Armijo backtracking: take a step in the Newton direction, halve until $U_n$ decreases sufficiently, accept. On a vector residual, "does it decrease" has to be replaced by "does $\|r\|$ decrease," which is a different and weaker property — $\|r\|$ can decrease while $U_n$ increases through a saddle, and the Newton step is no longer globally guaranteed to be a descent direction on the thing we actually care about. The minimization framing makes the [line-search sub-chapter](00-backward-euler/02-line-search.md) clean; the root-find framing would make it a pile of special cases.

**IPC lives inside the objective, not beside it.** The contact term $\Psi_\text{contact}(x)$ is the [IPC barrier energy](../40-contact/01-ipc-internals/03-energy.md) — a sum of logarithmic barriers over active contact pairs, differentiated once for contact force and twice for contact tangent. Because the barrier is part of $U_n$ and not a separate constraint, the Newton iteration is unconstrained. There is no LCP, no complementarity branch, no active-set update mid-iteration. The [Ch 03 thesis commitment](../10-physical/03-thesis.md) that contact and elasticity share one solve is only true if the timestepper sees both as scalar-valued energies added into one objective.

**Differentiability follows from the variational structure.** At convergence, $\nabla U_n(x^\ast; \theta) = 0$. [Part 6 Ch 02's IFT](../60-differentiability/02-implicit-function.md) differentiates this equilibrium condition with respect to a design parameter $\theta$, and the result is a linear solve against the Hessian $\partial^2 U_n / \partial x^2$ — the same Hessian Newton used forward. "Energy-based forward solve" is the precondition for "one-extra-solve backward pass," and this chapter is where that precondition is actually met.

## 2. Newton on the Hessian, not quasi-Newton

The inner loop is classical Newton. At iterate $x^{(k)}$, compute

$$ g^{(k)} = \nabla U_n(x^{(k)}), \qquad H^{(k)} = \nabla^2 U_n(x^{(k)}) $$

solve $H^{(k)}\, \Delta x^{(k)} = -g^{(k)}$, line-search along $\Delta x^{(k)}$, update. The Hessian $H$ is the sum of the elastic tangent (per-element, assembled via the [`Material::tangent()`](../20-materials/00-trait-hierarchy/00-trait-surface.md) trait method), the contact Hessian (barrier second derivatives, sparse in the active-contact pattern), and the inertial block $M/\Delta t^2$. It is symmetric positive-definite within the basin of attraction (the barrier term guarantees SPD even when the elastic tangent is indefinite at large strain — see [Part 4 Ch 01 adaptive stiffness](../40-contact/01-ipc-internals/01-adaptive-kappa.md)), so the factorization is Cholesky, not LU — [faer](https://github.com/sarah-quinones/faer-rs) provides both.

**Quasi-Newton variants are rejected.** L-BFGS, Broyden, and other Hessian-approximation schemes trade tangent-assembly cost for an increased outer iteration count and a loss of the exact local quadratic convergence rate. On scenes where the tangent *is* expensive — dense contact, near-incompressible material with mixed u-p — the tradeoff can pay off; on `sim-soft`'s typical scene it does not. Two reasons.

First, the Hessian $H$ is the object Part 6 Ch 02 re-uses in backward. A quasi-Newton iteration produces a Hessian *approximation* in the L-BFGS two-loop recursion sense, not a factorized sparse matrix; the backward-pass cost model ("one factor, many RHSes") breaks. Keeping the true Hessian is not a forward-only choice — it is what makes the adjoint cheap.

Second, `sim-soft`'s elastic-tangent assembly is not the bottleneck. For a 30k-tet mesh, per-element tangent evaluation is ≈3% of the per-Newton-iteration time; the sparse factorization is ≈60%; the line search and residual-norm computations are the rest. Cutting 3% by switching to L-BFGS at the cost of 2–5× more outer iterations is a bad trade. Numbers here come from single-threaded faer benchmarks on the Phase B target scene (~30k DOFs, neo-Hookean); they will be revised when the Phase E GPU path lands and the relative costs shift.

## 3. The factorization is a captured first-class object

Per Newton step, the Hessian is assembled once and factored once. The factorization handle — a `faer::sparse::linalg::solvers::Cholesky<f64>` instance — is then re-applied to multiple right-hand-sides *within* the Newton iteration (if iterative refinement is needed for the primal step), *across* the Newton iteration (in the next iterate's line search, if the Hessian is cached rather than re-assembled when $\|g\|$ is already small), and *into the backward pass* when Part 6 Ch 02's IFT needs it. The handle is not discarded when the timestep finishes. It is stored in the autograd tape entry for this step and re-used by every downstream VJP that consumes the step's output.

Concretely, the per-step forward/backward pattern is:

```rust
use faer::sparse::{SparseColMat, linalg::solvers::Cholesky};
use sim_ml_chassis::{Tape, Tensor};

pub struct NewtonStep {
    pub x_n: Tensor<f64>,               // converged position
    pub factor: Cholesky<f64>,          // captured Hessian factor at x_n
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
        let h = hessian_U_n(&x, theta);          // SparseColMat<f64>
        let factor: Cholesky<f64> = h.factor_cholesky()?;
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

This is the forward-side code for the pattern [Part 6 Ch 02](../60-differentiability/02-implicit-function.md) already shows on the backward side. The two code blocks read from the same object: forward stores `factor` and `dr_dtheta` on the tape; backward calls `factor.solve_in_place(&mut (-dr_dtheta.transpose() * upstream))` to get the gradient. No second factorization, no re-assembly.

That re-usability is why [Phase B commits to faer](../110-crate/03-build-order.md#the-committed-order) specifically — a solver whose factorizations were private-by-API, discarded after the primal solve, or tangled into an abstract `Solver` trait that hid the factor behind `solve(rhs) -> x` would fail this pattern. The adjoint cost would jump from "one back-substitution" to "one full factorization," the 10–30× speedup the Ch 03 thesis ("differentiability came for free") is built on would disappear, and Part 6's claims would have to weaken. Time integration is where the factor is *produced*; Part 6 Ch 02 is where it is *re-used*. Both chapters reference the same `Cholesky<f64>` instance living in the tape.

## 4. Line search is also an IPC safety mechanism

Armijo backtracking is the generic reason line search exists: ensure monotone decrease of $U_n$, reject Newton steps that overshoot. In an energy-based solver with IPC, line search does a second job: **it prevents any iterate from crossing a contact barrier.** The sub-chapter [02-line-search.md](00-backward-euler/02-line-search.md) derives the step-length clip; the top-level claim is that a single line-search routine handles both concerns, and a failed line search is a specific signal about which concern failed.

When the Armijo condition fails on $U_n$ alone (contact remains safe, elasticity overshoots), halve and retry. When the [CCD-computed contact-toi](../40-contact/01-ipc-internals/02-ccd.md) rejects the full step (some contact pair's time-of-impact is less than $\alpha = 1$), clip to $\alpha \le 0.5\, \mathrm{toi}$ and retry — the 0.5 factor is IPC's standard safety margin. When both conditions keep failing after 8 halvings, the Newton iteration gives up on the current $\Delta t$ and signals upstream to the adaptive-timestep layer ([Ch 02](02-adaptive-dt.md)), which shrinks $\Delta t$ by a factor of 2 and restarts the step. That fallback path is the *only* way the timestep shrinks — energy-monitor-triggered shrinking is a backstop for cases where CCD is disabled for debugging, not a primary mechanism.

The coupling is tight: Newton's line search is where contact and time integration actually negotiate. If `sim-soft` ever considered a split scheme where contact and elasticity had separate solvers (e.g., elasticity Newton then IPC projection), this coupling would fragment and the [popping failure from Part 1 Ch 02](../10-physical/02-what-goes-wrong/02-popping.md) would leak back in. The thesis commitment to one energy is also a commitment to one line search.

## What this commits downstream

- [Ch 02 (adaptive dt)](02-adaptive-dt.md) is triggered by Newton failures, not by an independent a-priori stability analysis. Failed-line-search ⇒ halve $\Delta t$ ⇒ retry-step. Energy-monitor backstop exists but is not the primary path.
- [Ch 03 (coupling)](03-coupling.md)'s fixed-point iteration with `sim-mjcf` treats each substep as a full Newton-converged solve on the current rigid state; iteration is across the sibling-simulator boundary, not within the Newton loop.
- [Part 6 Ch 02 (IFT)](../60-differentiability/02-implicit-function.md) re-uses the forward `Cholesky<f64>` factor from this chapter, stored on the tape. Forward and backward share one factorization; backward is one back-substitution.
- [Part 11 Ch 04 (testing)](../110-crate/04-testing/03-gradcheck.md)'s gradcheck suite validates this by comparing IFT-derived gradients to finite-difference gradients on a 100-tet static-equilibrium problem to 5 digits — which is the test for "did we wire the factor into the tape correctly," not a test of the IFT derivation itself.

## Alternatives rejected

**Semi-implicit (IMEX) splitting.** Explicit on elasticity, implicit on contact, or vice versa. Rejected because the elasticity-and-contact coupling through the Newton Hessian is the mechanism that kills popping (Claim 4); splitting the solver re-introduces the failure mode IPC exists to eliminate.

**Symplectic integrators (Verlet, implicit midpoint).** Conserve energy under Hamiltonian dynamics, which is attractive for long-time stability in conservative systems. Rejected because soft-body-with-friction-and-dissipative-contact is not Hamiltonian — there is no exact energy to conserve — and the symplectic advantage becomes a null property. Backward Euler's unconditional-stability-plus-numerical-dissipation is a better match for the dissipative regime we operate in.

**Higher-order BDF (BDF2, BDF3).** Multi-step backward-differentiation offers higher-order accuracy in $\Delta t$ at the cost of multiple past states stored per step. Rejected because the accuracy ceiling for `sim-soft` at experience-mode is set by mesh resolution and constitutive-law fidelity, not by timestep order; the per-frame visible difference between BDF1 and BDF2 at $\Delta t = 16$ ms is below the threshold where a user would notice. The storage cost of BDF2 (one extra position + velocity per tet) is not large, but the complexity cost (restart logic on $\Delta t$ change, startup state, interaction with adaptive timestep) is.

**Projective Dynamics / XPBD.** Deferred to [Ch 01](01-projective.md) — the structural-incompatibility case needs its own chapter.
