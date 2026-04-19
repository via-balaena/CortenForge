# MINRES

MINRES (Minimum Residual) is the iterative solver for symmetric *indefinite* systems — matrices where CG's descent-direction guarantee fails because the quadratic form is no longer positive. The algorithm is [Paige & Saunders 1975](../../appendices/00-references/07-solvers.md#paige-saunders-1975), still the canonical reference fifty years on. `sim-soft` uses it as a fallback when CG's negative-curvature check trips ([§00 CG](00-cg.md)), not as a primary solver — the Hessian is SPD inside the IPC barrier's basin ([Part 5 Ch 00 Claim 2](../../50-time-integration/00-backward-euler.md)), and MINRES triggers on <1% of Newton iterations on the canonical problem. This leaf specifies the algorithm, when the trigger fires, and what changes at the CG→MINRES boundary.

## When the trigger fires

The Hessian is SPD when the elastic tangent is SPD and when the barrier's adaptive-$\kappa$ ([Part 4 Ch 01](../../40-contact/01-ipc-internals/01-adaptive-kappa.md)) has had a chance to re-stabilize the contact Hessian after a step. Two regimes can violate SPD temporarily:

**Large compression strain during the first 1–2 Newton iterations after a re-mesh.** [Part 7 Ch 04's re-mesh](../../70-sdf-pipeline/04-live-remesh.md) transfers state from the old mesh to the new; the transfer can introduce per-tet strain spikes that push the elastic tangent indefinite before the per-element PSD projection has been re-applied ([Part 5 Ch 00 Claim 2](../../50-time-integration/00-backward-euler.md)). CG diverges on the first iterate, MINRES handles it, and by the second Newton iteration the projection has kicked in and CG resumes.

**Snap-through buckling under large elastic compression.** Rare in the canonical problem; occurs when a thin-walled geometry passes through an instability point. Same failure shape (transient indefiniteness), same fallback.

Empirically on the [canonical problem](../../10-physical/00-canonical.md) across Phase A–D regressions, MINRES triggers on <1% of Newton iterations — hundreds-to-thousands of steps pass between triggers — and the re-mesh-induced case dominates the count.

## The algorithm

MINRES minimizes the residual norm $\|b - H x_k\|_2$ over the Krylov subspace, rather than minimizing the $H$-norm of the error as CG does. For symmetric indefinite $H$, this produces a well-defined direction where CG's conjugate-gradient step has none (negative curvature breaks the step-length formula).

The short-recurrence Paige-Saunders form — three-term orthogonalization against the last two iterates via a Lanczos-style tridiagonalization, then a QR update to recover the minimum-residual coefficient — is the canonical implementation:

```text
Initialize: v_1 = b - H x_0 ;  β_1 = ‖v_1‖ ;  v_1 ← v_1 / β_1
            r_1 = β_1 ;  ρ_0 = 0 ;  σ_0 = 0
for k = 1, 2, ...
  u      = H v_k                              # SpMV
  α_k    = ⟨u, v_k⟩                           # dot
  u     ← u - α_k v_k - β_k v_{k-1}           # three-term orthogonalize
  β_k+1  = ‖u‖                                # norm
  v_k+1  = u / β_k+1                          # normalize
  [Givens rotation to update r, ρ, σ, etc.]
  x_k+1  = x_k + ... (coefficient × v_k)
  if r_k+1 ≤ tol : return x_k+1
```

Per iteration: one SpMV, two dot-product reductions (4 dispatches via the 2-dispatch tree reduction from [Ch 00 §00](../00-wgpu-layout/00-kernel-types.md)), three AXPYs (one more than preconditioned CG because of the explicit $-β_k v_{k-1}$ term), one normalize, one preconditioner apply, plus the scalar Givens rotation update on CPU. Total ≈10–11 dispatches per iteration versus preconditioned CG's ≈8–10, so per-iteration cost is ≈10–15% higher at the same preconditioner.

The Lanczos three-term recurrence is how short-recurrence is achieved for symmetric indefinite matrices. General non-symmetric GMRES requires full orthogonalization against every previous iterate and has a memory cost that grows with iteration count; MINRES exploits the matrix's symmetry to collapse that orthogonalization to the last two iterates. MINRES's memory cost is therefore O(DOFs) per iteration regardless of total iteration count, which is why it is the right choice for the fallback role.

## Preconditioning MINRES

MINRES accepts a preconditioner $M$, but $M$ must be SPD (not just positive on the specific Krylov directions). A preconditioner that is SPD for the elastic-tangent-projected Hessian is usually SPD for the unprojected one too, so `sim-soft`'s [§03 preconditioner hierarchy](03-preconditioning.md) (Jacobi / IC0 / AMG) transfers directly — no separate MINRES-preconditioner machinery.

Iteration count with the same preconditioner is typically 1.5–2× higher for MINRES than for CG. This is a property of the residual-minimization objective, not a `sim-soft`-specific inefficiency. In the rare MINRES-trigger regime, the extra iterations cost ≈2× the CG wall-clock — acceptable for a <1% case and better than the alternative (giving up on the Newton step and retrying with halved $\Delta t$).

## Cost relative to CG on the canonical scene

Per-iteration: ≈10–15% more dispatches (10–11 vs. 8–10).
Iteration count: ≈1.5–2× CG's at the same preconditioner.
Per-MINRES-triggered-Newton-iteration: ≈1.7–2.3× CG's wall-clock.
Fraction of Newton iterations that trigger MINRES: <1% on the canonical problem.

Amortized over the full canonical run, MINRES adds <2% to total solver wall-clock — below the noise floor of Phase E regression benchmarks. The design is explicit: MINRES is worth its per-trigger cost because the alternative (adaptive-$\Delta t$ halving on every transient indefiniteness) would cost ≥10× more.

## The CG→MINRES handoff

The failover is a single function call in the solver dispatch:

```rust
pub enum SolverStatus { Converged, Stagnated, NegativeCurvature }

pub fn solve_symmetric(op: &dyn SpMvOp, rhs: &GpuTensor<f32>,
                       out: &mut GpuTensor<f32>) -> Result<SolverStatus, SolverError> {
    match cg_solver.solve(op, rhs, out) {
        Ok(status) => Ok(status),
        Err(SolverError::NegativeCurvature) => {
            // CG detected indefinite direction; restart on MINRES
            minres_solver.solve(op, rhs, out)
                .map_err(SolverError::from)
        }
        Err(e) => Err(e),
    }
}
```

The restart re-uses the same RHS and the same matrix; the MINRES solver's workspace is pre-allocated at chassis init so no per-iteration allocation happens. The tape records whether the solve went CG or MINRES mainly for telemetry and debugging; the preconditioner is the same in either case, and [Ch 03 autograd](../03-gpu-autograd.md)'s backward pass uses the same preconditioner-on-tape regardless of which algorithm produced the forward iterate.

## What this sub-leaf commits the book to

- **MINRES is the indefinite-Hessian fallback, not the primary solver.** Triggers on <1% of Newton iterations on the canonical problem, dominated by post-re-mesh transient strain spikes ([Part 7 Ch 04](../../70-sdf-pipeline/04-live-remesh.md)). Paige-Saunders 1975 short-recurrence form.
- **Per-iteration cost is ≈10–15% more than CG**, iteration count is ≈1.5–2× CG's with the same preconditioner. Amortized-to-total impact is <2% on the canonical run.
- **Same preconditioner as CG.** The [§03 Jacobi/IC0/AMG hierarchy](03-preconditioning.md) transfers unchanged; no MINRES-specific preconditioner machinery.
- **The CG→MINRES handoff is a single fallback branch in the solver dispatch.** Workspace is pre-allocated; the tape records which algorithm converged so the [Ch 03 backward pass](../03-gpu-autograd.md) sees the right state.
