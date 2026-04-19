# Algebraic multigrid

Multigrid methods exploit the observation that iterative solvers like CG damp high-frequency error modes quickly but leave low-frequency modes nearly unchanged — so a single-grid solve spends most of its iterations grinding through the smooth errors. Multigrid fixes this by solving the smooth-error residual on a *coarser* grid (fewer DOFs, fewer iterations), then interpolating the correction back. Algebraic Multigrid (AMG), as opposed to geometric multigrid, constructs the coarse grids directly from the matrix rather than from the spatial mesh — which matters for `sim-soft` because the tet mesh's geometric structure is irregular and the matrix's algebraic structure is what the solver actually sees. [Trottenberg, Oosterlee & Schüller 2001](../../appendices/00-references/07-solvers.md#trottenberg-oosterlee-schuller-2001) is the canonical textbook; [Stüben 1999](../../appendices/00-references/07-solvers.md#stuben-1999) is the practitioner-facing survey `sim-soft`'s implementation follows. The [Ch 02 parent's Claim 5](../02-sparse-solvers.md) names AMG as the scene-scale preconditioner (≥100k DOFs); this leaf specifies the coarsening strategy, the V-cycle shape, and the setup amortization that makes AMG cost-competitive across Newton iterations.

## The coarsening strategy

Classical AMG (Ruge-Stüben coarsening) selects coarse-grid "C-points" as a subset of the fine-grid DOFs such that every fine-grid "F-point" is strongly connected to at least one C-point. Strong connection is a threshold on the scaled off-diagonal entry magnitude: DOF $i$ is strongly connected to DOF $j$ if

$$ |A_{ij}| \geq \theta \cdot \max_{k \ne i} |A_{ik}| $$

with $\theta \in [0.25, 0.5]$ typical. A greedy max-independent-set sweep over the fine grid then picks C-points subject to the strong-connection covering constraint. For soft-body Hessians the resulting coarsening ratio is ≈1:4 to 1:8 per level — a 300k-DOF fine grid produces a ≈60k-DOF first coarse level, a ≈10k-DOF second level, and so on until the coarsest level is small enough (≈100 DOFs) to direct-solve.

Interpolation weights between levels follow the classical Ruge-Stüben construction from [Stüben 1999](../../appendices/00-references/07-solvers.md#stuben-1999): an F-point's value is expressed as a weighted combination of its strongly-connected C-points, with weights derived from the matrix's off-diagonal entries along both direct F→C connections and the "standard" F-F lumping that captures indirect paths through other F-points. The construction requires inspecting matrix entries — which is why AMG is incompatible with matrix-free SpMV ([Ch 01 §02 custom](../01-sparse-matrix/02-custom.md)) and falls back to Jacobi when matrix-free is in use. Pass 1 references the construction by pointer rather than duplicating the formula; the Stüben survey's §4.2 gives the exact form.

## The V-cycle

Each AMG preconditioner application is one V-cycle: down-sweep through the levels, direct-solve at the coarsest, up-sweep back:

```text
function V-cycle(x, b, level):
    if level == coarsest:
        return direct_solve(A_level, b)                # ≈100-DOF dense solve
    x ← smooth(A_level, x, b, ν_pre iterations)        # Gauss-Seidel / Jacobi smoother
    r ← b - A_level · x                                # residual
    r_coarse ← R_level · r                             # restrict to coarser
    e_coarse ← V-cycle(0, r_coarse, level+1)           # recurse
    x ← x + P_level · e_coarse                         # interpolate back + correct
    x ← smooth(A_level, x, b, ν_post iterations)       # post-smoothing
    return x
```

Pre- and post-smoothing iteration counts $\nu_{\text{pre}} = \nu_{\text{post}} = 2$ are typical for soft-body Hessians. Smoothing uses Jacobi on GPU (embarrassingly parallel, fast per pass) rather than Gauss-Seidel (inherently serial in the classical ordering; only a red-black partitioning gets parallelism, at the cost of more smoothing iterations). The CPU-side multigrid literature typically finds Gauss-Seidel smoothers converge in modestly fewer outer iterations than Jacobi, but the per-pass GPU parallelism win from Jacobi's embarrassingly-parallel structure more than compensates. Phase E benchmarks will re-measure the trade on `sim-soft`'s specific Hessian pattern.

Cost accounting:

- **Fine-level work per V-cycle**: ≈2 pre-smooth + 1 residual + 1 restrict + 1 interpolate + 2 post-smooth ≈ 7 SpMV-equivalent operations on the 300k-DOF level.
- **Coarser levels**: each at ≈1/5 the fine work, geometrically summing. Total V-cycle cost ≈ 1.5 × fine-level work.
- **CG iteration count reduction vs. Jacobi**: from ≈200–400 iters to ≈10–15 iters on the 300k-DOF scene (per the [§03 preconditioner table](03-preconditioning.md)).
- **Net: AMG is ≈5–8× faster than Jacobi** per CG solve on scene-scale problems, despite the ≈1.5× per-CG-iteration overhead.

## Setup cost and amortization across Newton steps

AMG setup — coarsening, interpolation-operator construction, coarsest-level factorization — is expensive. Per-level cost is O(nnz) for the coarsening sweep; across the level hierarchy, total setup is ≈10–50× a single fine-level SpMV. On the canonical 300k-DOF scene, setup is ≈20–50 ms per run.

The setup is amortized across all Newton iterations within a timestep, because the matrix *pattern* (the sparsity structure) is invariant across Newton iterations — only the *values* change. The coarsening is pattern-driven; re-running the classical Ruge-Stüben sweep on the same-pattern matrix produces the same C-point set and the same interpolation operators. Only the coarser-level matrices' numerical values change, which is a cheap O(nnz) rebuild.

```rust
pub struct AmgPreconditioner {
    pattern_hierarchy:  AmgPatternHierarchy,       // cached across Newton iters
    numerical_hierarchy: AmgNumericalHierarchy,    // rebuilt per Newton iter
    smoother_config:     SmootherConfig,
}

pub fn setup_per_newton(&mut self, A_fine: &GpuSparseMatrix<f32>) {
    // Pattern-level setup is reused
    // Numerical setup rebuilds A_coarse matrices via Galerkin projection
    self.numerical_hierarchy.recompute(A_fine, &self.pattern_hierarchy);
}

pub fn setup_per_remesh(&mut self, A_fine: &GpuSparseMatrix<f32>) {
    // Pattern has changed; rebuild from scratch
    self.pattern_hierarchy = AmgPatternHierarchy::build(A_fine);
    self.numerical_hierarchy = AmgNumericalHierarchy::build(A_fine, &self.pattern_hierarchy);
}
```

Per Newton iteration, the cost is the Galerkin projection $A_\text{coarse} = R \, A_\text{fine}\, P$ at each level, ≈5× SpMV-equivalent, plus the coarsest-level refactor (small). Per re-mesh, the full pattern rebuild re-runs. [Part 7 Ch 04's re-mesh pattern](../../70-sdf-pipeline/04-live-remesh.md) is the boundary: within a timestep (and between timesteps with stable topology), the cached pattern hierarchy re-uses; across a re-mesh, it rebuilds.

## Integration with the factor-on-tape pattern

[Ch 02 parent Claim 2](../02-sparse-solvers.md)'s "preconditioner-on-tape" generalization of the CPU factor-on-tape pattern places AMG's pattern hierarchy on the autograd tape alongside the forward solve's converged state. For the IFT backward pass, the same hierarchy applies to every downstream adjoint — one pattern-construction cost amortizes across every backward solve. At Phase E AMG setup cost of ≈20–50 ms, amortizing across ≈5 backward RHSes per forward saves ≈100–250 ms per parameter-gradient evaluation. This is the GPU analog of the CPU "one factor, many RHSes" cost model from [Part 6 Ch 02 (IFT)](../../60-differentiability/02-implicit-function.md), with the numerical constant shifted.

## AMG is scene-scale, not canonical-scale

On the 30k-DOF canonical scene, CG with Jacobi reaches convergence in ≈60–100 iterations. AMG would cut that to ≈5–10, but AMG's setup cost (≈10 ms even at 30k DOFs) and V-cycle overhead push its per-Newton-step cost above Jacobi's. The crossover is at ≈100k DOFs, per [Ch 02 parent Claim 5](../02-sparse-solvers.md); below that, Jacobi (or IC0 at the mid-tier) wins; above it, AMG wins decisively.

`sim-soft`'s preconditioner selector ([§03](03-preconditioning.md)) picks AMG automatically when DOF count exceeds the threshold. The threshold is tunable and gets benchmarked as part of Phase E's gradcheck regression suite ([Part 11 Ch 04](../../110-crate/04-testing/03-gradcheck.md)).

## What this sub-leaf commits the book to

- **AMG is the scene-scale preconditioner (≥100k DOFs).** Below that, Jacobi and IC0 win on setup + per-iteration cost. Crossover is tunable and measured, not pinned.
- **Ruge-Stüben classical coarsening.** Strong-connection threshold $\theta \in [0.25, 0.5]$; greedy max-independent-set C-point selection; classical interpolation weights from [Stüben 1999](../../appendices/00-references/07-solvers.md#stuben-1999).
- **V-cycle with Jacobi smoothing.** 2 pre + 2 post smoothing iterations per level. Gauss-Seidel rejected because its serial dependency chain loses to Jacobi's parallelism on GPU.
- **Pattern hierarchy cached across Newton iters, rebuilt on re-mesh.** Galerkin projection at each level rebuilds numerical matrices per Newton iter; full coarsening reruns per re-mesh ([Part 7 Ch 04](../../70-sdf-pipeline/04-live-remesh.md)).
- **AMG setup is on the autograd tape.** Preconditioner-on-tape generalization of CPU factor-on-tape from [Part 6 Ch 02](../../60-differentiability/02-implicit-function.md); amortizes across backward RHSes.
- **Matrix-free-incompatible.** AMG coarsening inspects matrix entries to pick interpolation weights; [Ch 01 §02 matrix-free](../01-sparse-matrix/02-custom.md) runs fall back to Jacobi.
