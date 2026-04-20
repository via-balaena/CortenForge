# solver/

The `solver/` module owns the Newton loop that drives the whole simulation: per timestep, it assembles the total potential energy $U_n(x; \theta) = \Psi_\text{elastic} + \Psi_\text{contact} + \Psi_\text{inertia}$ from [`material/`](00-material.md) / [`element/`](01-element.md) / [`contact/`](03-contact.md), factors the resulting SPD Hessian, line-searches along the Newton direction, and stores the factorization as a first-class object on [`autograd/`](07-autograd.md)'s tape so the backward pass reuses it. The [Part 5 Ch 00 backward-Euler chapter](../../50-time-integration/00-backward-euler.md) carries the time-integration scheme; this module is that chapter translated into running code.

## What the module owns

| Sub-area | Responsibility | Where it is specified |
|---|---|---|
| Timestep loop | Inertial predictor, Newton iteration on $U_n(x)$, line-search backtracking with IPC safety clip | [Part 5 Ch 00](../../50-time-integration/00-backward-euler.md) |
| Global assembly | CSR / BSR sparse-Hessian construction from [`element/`](01-element.md)'s per-tet blocks plus [`contact/`](03-contact.md)'s per-pair blocks plus the constant inertial term | [Part 8 Ch 01](../../80-gpu/01-sparse-matrix.md) |
| Sparse linear solve | `faer::sparse::linalg::solvers::Llt<f64>` on CPU, wgpu-native preconditioned CG on GPU; both expose the factor-as-first-class-object contract | [Part 5 Ch 00 Claim 3](../../50-time-integration/00-backward-euler.md), [Appendix reference](../../appendices/00-references/07-solvers.md) |
| Adaptive Δt | Armijo failure ⇒ step halves; CCD rejection ⇒ step clips. The only two mechanisms that shrink $\Delta t$ | [Part 5 Ch 02](../../50-time-integration/02-adaptive-dt.md) |
| Projective Dynamics path | Projective / XPBD alternative as an opt-in second solver; structural incompatibilities flagged | [Part 5 Ch 01](../../50-time-integration/01-projective.md) |
| Factor-on-tape | The `Llt<f64>` handle and $\partial r / \partial \theta$ sparse Jacobian recorded per step, consumed backward by IFT | [Part 6 Ch 02](../../60-differentiability/02-implicit-function.md) |

## Four claims

**1. Unconstrained minimization, not root-finding.** [Part 5 Ch 00 Claim 1](../../50-time-integration/00-backward-euler.md) is the argument; this module is where that argument is executed. The Newton loop's stopping test is $\|\nabla U_n\| < \text{tol}$ on a scalar objective, not $\|r\| < \text{tol}$ on a vector residual; line search is Armijo on the scalar, not norm-decrease on the residual; contact lives inside the objective as $\Psi_\text{contact}(x)$, not as a constraint solved alongside it. The module's public surface has no residual-form API; the entry point is `step(tape, x_prev, v_prev, theta, dt) -> NewtonStep` and the `NewtonStep` struct carries the converged position, the factor, and the per-step residual Jacobian w.r.t. design parameters.

**2. faer on CPU for the factor-as-object contract.** [Part 5 Ch 00 Claim 3](../../50-time-integration/00-backward-euler.md) names faer's SPD factorization (`Llt<f64>`) as the specific type `sim-soft` ships against. The reason is not vendor preference — it is the public API shape. faer's factors are first-class objects that can be re-applied to arbitrary RHSes, which is exactly what [Part 6 Ch 02 IFT](../../60-differentiability/02-implicit-function.md)'s adjoint needs: forward produces the factor, backward applies it once to the upstream gradient. A solver whose factors were private or discarded would force the adjoint to factor again — the "one extra solve per backward" promise collapses, and [Part 1 Ch 03's "differentiability came for free" thesis](../../10-physical/03-thesis.md) weakens.

**3. GPU CG is an iterative solve with the same factor-on-tape contract.** Phase E's GPU port ([`gpu/`](06-gpu.md), [Part 8 Ch 02](../../80-gpu/02-sparse-solvers.md)) replaces the sparse direct factorization with preconditioned conjugate gradient — an iterative solver, not a factoring one. The factor-on-tape contract is satisfied differently: the "factor" object is a preconditioner handle plus a warm-start iterate, which the backward pass uses as the starting point for the adjoint's own CG solve. This is not cost-free — the adjoint's CG may take multiple iterations where the CPU back-substitution was $O(\text{nnz})$ — but CG on GPU at Phase E throughput is still faster than CPU Cholesky back-substitution on 30k-tet scenes. The contract shape survives; the specific cost model shifts.

**4. Only two things shrink Δt.** [Part 5 Ch 00 Claim 4](../../50-time-integration/00-backward-euler.md) gives the full argument: Armijo line-search failure on $U_n$ triggers $\Delta t \to \Delta t / 2$; CCD-rejected steps clip the Newton $\alpha$ but do not shrink $\Delta t$ unless line search also fails after the clip. An independent energy-conservation monitor exists ([Part 5 Ch 02 §00](../../50-time-integration/02-adaptive-dt/00-energy.md)) as a backstop when CCD is disabled for debugging, but is never the primary mechanism. This keeps the adaptive-Δt path a single-cause failure signal, which is what makes debug sessions tractable: a shrunk $\Delta t$ means the Newton step failed, and the next question is "why did line search fail" — not "which of three independent timestep controllers voted for a shrink."

## What the module does not carry

- **No per-material branches.** The solver is generic over what [`material/`](00-material.md) and [`element/`](01-element.md) produced; it sees per-element stiffness blocks and global DOF indices, not neo-Hookean-vs-Ogden. Per-material differences vanish at the assembly layer.
- **No contact-pair generation.** [`contact/`](03-contact.md) emits the active pairs and their per-pair energy/force/Hessian; the solver assembles them. Broadphase and CCD are not solver concerns.
- **No reward evaluation.** Once a step converges, the position $x^\ast$ goes to [`readout/`](09-readout.md). The solver does not know what the reward is.
- **No re-meshing.** A mesh change between timesteps is an external event ([`sdf_bridge/`](08-sdf-bridge.md) triggers it); the solver receives a new mesh plus warm-start data and proceeds. Re-meshing policy lives elsewhere.

## What this commits downstream

- **[`autograd/`](07-autograd.md)'s IFT pass re-uses the `Llt<f64>` this module produces.** The factor lives on the tape; the backward pass does not construct its own.
- **[Part 11 Ch 04 gradcheck](../04-testing/03-gradcheck.md) regresses against this module's forward-plus-IFT pipeline** — static-equilibrium FD gradients vs IFT gradients to 5 digits is the test of "did we wire the factor into the tape correctly," [Part 5 Ch 00's](../../50-time-integration/00-backward-euler.md) committed acceptance bar.
- **[Part 11 Ch 03 Phase B deliverable](../03-build-order.md#the-committed-order)** is this module minus contact minus GPU — Tet4 + neo-Hookean + faer CPU Newton on a cube, passing the gradcheck suite against FD to 5 digits on a 100-tet mesh. Every subsequent phase extends the solver without changing its Newton-iteration shape.
