# Parallel sparse solvers

[Phase B and Phase D's CPU path](../110-crate/03-build-order.md#the-committed-order) use [faer's sparse Cholesky](../50-time-integration/00-backward-euler.md) as the direct solver for Newton's linear system. Phase E is the GPU port, and **on GPU the direct-factorization path does not exist in a form we would use.** Sparse Cholesky on GPU is an active research area — recent work from CUDA-native teams and some academic projects has shown it can be made to work — but no cross-vendor wgpu-based implementation is available, and rolling one from scratch is a multi-year effort well outside `sim-soft`'s scope. The GPU solver is therefore **iterative**: preconditioned conjugate gradient (PCG) for SPD systems, MINRES for the rare indefinite cases, algebraic multigrid as a preconditioner for larger scenes. This chapter commits to that path and explains how the [Part 5 Ch 00 factor-on-tape pattern](../50-time-integration/00-backward-euler.md) and [Part 6 Ch 02 IFT cost model](../60-differentiability/02-implicit-function.md) survive the switch — which, up front, is the non-obvious part.

| Section | What it covers |
|---|---|
| [Conjugate Gradient](02-sparse-solvers/00-cg.md) | The primary solver. Preconditioned, warm-started from the previous iterate, converges in $O(\sqrt{\kappa})$ iterations where $\kappa$ is the condition number. Tolerance matched to Newton's outer tolerance |
| [MINRES](02-sparse-solvers/01-minres.md) | Fallback for indefinite systems. Rare in the IPC-stabilized Hessian regime, but the failure mode exists (very large strain before the barrier guarantee kicks in); MINRES lets Newton proceed on the iterate rather than giving up |
| [Algebraic multigrid](02-sparse-solvers/02-amg.md) | Preconditioner for large scenes (≥100k DOFs) where PCG's $O(\sqrt{\kappa})$ iteration count becomes the dominant cost. Setup-amortized across Newton steps when the mesh is unchanged |
| [Preconditioning](02-sparse-solvers/03-preconditioning.md) | Jacobi (diagonal inverse) as the baseline, incomplete Cholesky as the mid-tier option, AMG as the scene-scale upgrade. Preconditioner construction is the cacheable cost |

Five claims, three of them load-bearing reconciliations with earlier Parts.

## 1. CG is the primary GPU solver, direct factorization is not on the roadmap

Why not GPU Cholesky. The short answer: no production-grade, cross-vendor, maintained library exists. The longer answer is that sparse Cholesky on GPU requires (a) symbolic factorization (nested-dissection ordering, elimination tree), which is inherently serial at the task level and scales poorly to thousands of threads; (b) numerical factorization with dynamic fill-in patterns, which requires atomic updates on GPU sparse data structures that vendors do not uniformly support; (c) sparse-triangular solves on GPU, whose dependency chains through the factor limit parallelism and push performance into a regime where the direct path no longer clearly beats a well-preconditioned iterative solver. Individual research groups have shown working GPU sparse Cholesky for specific problems (nested-dissection-friendly meshes, specific CUDA hardware); the combination of all three across vendors is an open engineering problem, and owning it is not in `sim-soft`'s charter.

CG, in contrast, is mechanically simple on GPU. Every inner iteration is one SpMV ([Ch 01 BSR](01-sparse-matrix.md)), three element-wise vector ops, and two dot-product reductions. All four operations thread-parallel trivially and have well-understood GPU implementations in every major library. The algorithmic cost is higher — $O(\sqrt{\kappa})$ iterations versus direct factorization's $O(1)$ — but the per-iteration cost is far lower on GPU, and the net wall-clock is competitive with (and often better than) what a direct solver would deliver if one existed.

**The algorithmic choice is forced by the hardware, not a preference.** On CPU, direct is the right answer and faer provides it. On GPU, iterative is the right answer and CG/MINRES provide it. `sim-soft`'s solver module accepts both backends behind a common trait; the choice is determined by which device owns the matrix, not by a user flag.

## 2. The factor-on-tape pattern generalizes to preconditioner-on-tape

[Part 5 Ch 00 Claim 3](../50-time-integration/00-backward-euler.md) stores the Cholesky factorization on the autograd tape and re-uses it in [Part 6 Ch 02's IFT adjoint](../60-differentiability/02-implicit-function.md) — one factor, many RHSes, 10–30× speedup over re-factoring. The iterative-solver version of this pattern is not "one factor" but "one preconditioner plus one warm-start vector":

```rust
use sim_ml_chassis::gpu::{GpuTensor, GpuSparseMatrix, CgSolver, Preconditioner};

pub struct GpuNewtonStep {
    pub x_n: GpuTensor<f64>,                 // converged position
    pub precond: Box<dyn Preconditioner>,    // cached, re-applied in backward
    pub prev_solution: GpuTensor<f64>,       // warm-start for next backward solve
    pub dr_dtheta: GpuSparseMatrix<f64>,     // residual Jacobian w.r.t. theta
    pub hessian: GpuSparseMatrix<f64>,       // the operator, matrix-free-capable
}

// Forward: CG with preconditioner, warm-started from previous iterate
let mut cg = CgSolver::new();
cg.set_preconditioner(&precond);
for upstream in downstream_adjoints {
    let rhs = &(-dr_dtheta.transpose()) * upstream;
    let grad = cg.solve(&hessian, &rhs, /* warm_start */ prev_solution.clone())?;
    // ...
}
```

Three properties of this pattern that matter.

**Preconditioner re-use is the GPU analog of factor re-use.** Computing an AMG preconditioner is expensive (tens of milliseconds on a canonical-scale scene); applying it in one CG iteration is cheap (sub-millisecond). For the IFT backward pass, the same preconditioner applies to every downstream adjoint because the matrix $H$ is the same — and it stays cached on the tape the same way the Cholesky factor does on CPU. The cost model "one preconditioner setup, many backward solves" recovers the 10–30× Part 6 Ch 02 speedup claim, with the numerical constant shifted but the shape unchanged.

**Warm-starting matters more than on CPU.** Direct solvers don't care about initial guess; they factor once, solve in $O(\mathrm{nnz})$ regardless. CG does care — a good initial guess can drop iteration count by 2–5×. For the IFT loop, each backward solve's RHS differs, but the structure of the matrix is the same, and the previous backward iterate is often a decent warm start for the next. The tape stores the previous solution as well as the preconditioner.

**The failure mode is convergence stagnation, not exception.** A direct solver either produces an answer or fails hard (non-SPD matrix, singular column). CG can stagnate — iterate count hits a cap without reaching tolerance. The tape records the iteration count achieved in each backward pass and a flag if the cap was hit; downstream gradient consumers can inspect the flag and act (e.g., [Part 10's optimizer](../100-optimization/00-forward.md) weights stagnated-gradient samples lower the same way it treats topology-crossing samples from [Part 7 Ch 04](../70-sdf-pipeline/04-live-remesh.md)).

## 3. Precision: single for the solve, double for accumulation

CPU runs at `f64` end-to-end; faer's factorizations are stable at `f64` on sparse matrices up to reasonable condition numbers. GPU hardware's peak FLOPS are at `f32`, with `f64` typically 1/32 to 1/2 of peak depending on vendor. Running the GPU CG purely in `f32` would lose 6–7 digits of precision that the gradcheck suite ([Part 11 Ch 04](../110-crate/04-testing/03-gradcheck.md)) expects at 5.

The commitment: **SpMV and preconditioner apply in `f32`; dot-product accumulators and the solution vector in `f64`.** This is mixed-precision CG, well-studied in the iterative-solvers literature (Carson and Higham 2018 and successors). Gradient error on the canonical scene holds to 4–5 digits versus the CPU path, and a single iterative-refinement step at `f64` SpMV re-rounds the final iterate to clear the [5-digit gradcheck threshold](../110-crate/04-testing/03-gradcheck.md). The refinement step costs ≈10% extra wall-clock; the payoff is full gradcheck compatibility between CPU and GPU paths.

Pure `f64` is available as a debug-only flag and is what the gradient-consistency regression test uses. Production runs mixed-precision.

## 4. MINRES handles the rare indefinite cases

The Hessian is SPD *inside the IPC barrier's basin of attraction* ([Part 5 Ch 00 Claim 2](../50-time-integration/00-backward-euler.md)). Outside that basin — very large strain before the barrier's adaptive-$\kappa$ term kicks in ([Part 4 Ch 01](../40-contact/01-ipc-internals/01-adaptive-kappa.md)) — the elastic tangent can be indefinite and the Hessian loses positive-definiteness. CG on an indefinite matrix can diverge; MINRES handles indefinite symmetric matrices at the cost of a slightly more complex inner loop (short recurrence plus orthogonalization against the last two iterates rather than one).

In the canonical problem, MINRES triggers on <1% of Newton iterations, specifically during the first 2–3 iterations of a topology-changing re-mesh ([Part 7 Ch 04](../70-sdf-pipeline/04-live-remesh.md)) where state-transfer-induced strain spikes are largest. Once the solver has pulled the iterate back into the barrier's basin, CG resumes. The fallback is automatic — a single CG breakdown (detected via negative curvature in the conjugation step) triggers a MINRES retry with the same RHS. The flag is recorded on the tape the same way MINRES-vs-CG is recorded for the backward pass.

## 5. AMG is the scene-scale preconditioner, Jacobi is the baseline

CG's iteration count grows as $\sqrt{\kappa}$ where $\kappa$ is the condition number. For soft-body Hessians, $\kappa$ grows roughly with $1/h^2$ where $h$ is the mesh size — finer meshes have worse conditioning. The preconditioner choice scales accordingly.

- **Jacobi (diagonal inverse).** Cheapest, works for small problems (≤10k DOFs). Canonical scene at Phase E resolution (≈30k DOFs) runs in the ≤100-CG-iteration band with Jacobi — within budget at Phase E's 30 FPS target but not the fastest path; IC0 cuts the iteration count to ≈40, AMG to <15.
- **Incomplete Cholesky (IC0 or IC(k)).** Mid-tier. Construction is sequential on GPU (triangular-structure dependency), so it is not a default; used when Jacobi's iteration count exceeds a threshold.
- **Algebraic multigrid (AMG).** Scene-scale upgrade for ≥100k DOFs where PCG with Jacobi would take hundreds of iterations. Setup is expensive (one-time per unchanged-topology mesh), apply is cheap (sub-millisecond per V-cycle). AMG setup is cached across Newton steps within the same timestep — the mesh is unchanged, so the coarse-grid hierarchy is too.

The preconditioner choice is adaptive: scenes below 30k DOFs use Jacobi, scenes above 100k use AMG, scenes in between use IC0 as a bridge. The thresholds are tunable and benchmarked in [Part 11 Ch 04's gradcheck suite](../110-crate/04-testing/03-gradcheck.md) as part of the regression baseline.

## What this commits downstream

- [Part 6 Ch 02 (IFT)](../60-differentiability/02-implicit-function.md) remains the canonical cost model, but the "factorization" it cites is the CPU path; the GPU equivalent is "preconditioner + warm-start vector," with the 10–30× speedup argument preserved via preconditioner re-use rather than factor re-use.
- [Part 6 Ch 01 (custom VJPs)](../60-differentiability/01-custom-vjps.md) registers solver VJPs that accept either a Cholesky-factor-based or a preconditioner-based backward pass; the registration is backend-polymorphic.
- [Ch 03 (GPU autograd tape)](03-gpu-autograd.md) records the preconditioner alongside the solution on every forward solve; tape memory cost accordingly includes the preconditioner size (≈0.5–2× the assembled Hessian for AMG).
- [Ch 04 (chassis extension)](04-chassis-extension.md) exposes the `CgSolver` / `MinresSolver` types as part of the `sim-ml-chassis::gpu` surface; the cg-vs-minres selection and the precision-mix are sim-ml-chassis configuration, not per-user-per-call.
- [Phase E's GPU port](../110-crate/03-build-order.md#the-committed-order) ships with Jacobi and IC0 as the initial baseline; AMG lands later in Phase E once the baseline is regression-tested against the CPU path to 5-digit gradcheck.

## Alternatives considered

**Sparse Cholesky on GPU, rolled ourselves.** Build-vs-buy: reaching parity with faer's CPU quality on GPU, cross-vendor and maintained, is a multi-engineer-year commitment. The payoff is the $O(1)$ iteration count advantage and factor-re-use that matches the CPU pattern exactly. Rejected as not-on-the-roadmap — the engineering cost far exceeds the win from keeping the pattern identical. The iterative-solver reconciliation in Claim 2 is strictly cheaper than the direct-solver rebuild.

**Vendor-native CUDA solvers (cuSPARSE, cuSolverSp).** Cross-vendor break; ties `sim-soft` to NVIDIA for the GPU path. Rejected per [Ch 00](00-wgpu-layout.md)'s portability commitment.

**BDDC / FETI domain decomposition.** A hybrid approach — domain-decompose the mesh, solve each subdomain with a local direct factorization, couple via a small global problem. Works well on CPU clusters; on a single-GPU soft-body problem the subdomain-size / coupling-overhead trade does not pay. Rejected.

**Eager Krylov methods beyond CG (GMRES, BiCGSTAB).** For SPD matrices, CG is optimal; GMRES is for non-symmetric, BiCGSTAB for more aggressive non-symmetric cases. Neither is needed in `sim-soft`'s symmetric regime. Rejected.
