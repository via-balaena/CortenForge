# Preconditioning

CG's iteration count grows as $\sqrt{\kappa}$ in the condition number, and soft-body Hessians condition roughly as $1/h^2$ where $h$ is the mesh size — finer meshes have worse conditioning, CG's iteration count grows, and by 300k DOFs an un-preconditioned CG takes hundreds of iterations. Preconditioning trades setup cost for per-iteration iteration-count reduction. `sim-soft` ships three preconditioner tiers, matching the three regimes the [Ch 02 parent's Claim 5](../02-sparse-solvers.md) names: Jacobi at the cheap end, Incomplete Cholesky in the middle, AMG at the scene-scale end. [Saad 2003 Chapters 9–10](../../appendices/00-references/07-solvers.md#saad-2003) is the textbook background the implementation follows. This leaf specifies the tier boundaries, the setup and apply costs per tier, and the runtime selection policy.

## The three tiers

| Preconditioner | Setup cost | Apply cost | CG iter count on 30k-DOF | CG iter count on 300k-DOF | Selected when |
|---|---|---|---|---|---|
| Jacobi ($M = \text{diag}(H)^{-1}$) | O(n): one scan | O(n): one element-wise divide | ≈60–100 | ≈200–400 | DOF count < 30k, or matrix-free |
| IC0 (incomplete Cholesky, no fill) | O(nnz): one factorization | O(nnz): triangular solves | ≈20–40 | ≈40–80 | 30k ≤ DOF count < 100k |
| AMG ([§02](02-amg.md)) | O(nnz) × 10–50 | O(nnz) × 1.5 (V-cycle) | ≈5–10 | ≈10–15 | DOF count ≥ 100k |

"Selected when" is the default; the chassis exposes a runtime override for benchmarking, and Phase E regression ([Part 11 Ch 04](../../110-crate/04-testing/03-gradcheck.md)) re-measures the crossovers as mesh generators and hardware evolve.

## Jacobi: the diagonal-inverse baseline

Jacobi preconditioning is $M = \text{diag}(H)^{-1}$ — the diagonal of the Hessian inverted element-wise. Apply is a per-DOF multiply; setup is a one-shot scan to extract the diagonal.

```wgsl
@compute @workgroup_size(256)
fn jacobi_apply(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= n_dofs) { return; }
    z[i] = r[i] * inv_diag[i];    // inv_diag = 1/diag(H), precomputed
}
```

Three properties that keep Jacobi in the build:

**Trivially parallel.** One thread per DOF, no inter-thread dependencies, no atomics. Runs on the [Ch 00 §00 per-DOF shape](../00-wgpu-layout/00-kernel-types.md) without modification.

**Matrix-free compatible.** The diagonal of $H$ can be extracted element-by-element from the per-tet contributions without ever forming the full Hessian; [Ch 01 §02 matrix-free](../01-sparse-matrix/02-custom.md) runs use Jacobi as the only working preconditioner at scale.

**Weak at scene-scale.** Jacobi's CG iteration count grows linearly with $\sqrt{\kappa}$ and $\kappa \sim 1/h^2$, so doubling DOF count (halving $h$) doubles the iteration count. At 300k DOFs this is ≈200–400 CG iterations per Newton step — dominating wall-clock even though each iteration is cheap. This is why the tier progression exists.

## Incomplete Cholesky (IC0): mid-tier bridge

IC0 factors $H \approx L L^T$ where $L$ is a lower-triangular matrix constrained to have *the same sparsity pattern as the lower triangle of $H$* — no fill-in allowed. The factorization is inexact (the off-pattern entries that would appear in exact Cholesky are dropped), but it captures enough of $H^{-1}$ to improve CG convergence substantially.

Construction is the sparse direct Cholesky algorithm with the fill-drop constraint:

```text
for j = 0, 1, ..., n-1:              # column sweep, lower-triangular
    for i such that A_ij ≠ 0:         # only existing nonzero pattern
        L_ij = (A_ij - Σ_{k<j, (i,k)∈pattern} L_ik · L_jk) / L_jj
    L_jj = sqrt(A_jj - Σ_{k<j, (j,k)∈pattern} L_jk²)
```

The dependency chain — $L_{ij}$ depends on $L_{i,k<j}$ and $L_{j,k<j}$ — is serial in the column direction. This is why IC0 setup does not parallelize cleanly on GPU in the classical ordering; it either runs serially (hundreds of milliseconds on 100k-DOF scenes) or requires a multi-coloring variant (block-based parallel IC0, Anzt et al. family of approaches) that trades some convergence quality for parallelism. Pass 1 commits to the simpler serial IC0 setup, accepting the setup cost as a per-Newton-iteration overhead — the per-iteration improvement in CG count (≈40 vs. ≈100) still wins at the 30–100k DOF band.

Apply is two triangular solves (forward substitution then backward substitution). On GPU, triangular solves are partially parallel — levels in the dependency DAG can be processed together, with synchronization barriers between levels. Typical "level-set" triangular solve parallelism recovers ≈5–10× speedup over fully serial, which is enough to make IC0 apply cost-competitive with Jacobi apply on mid-tier scenes.

IC0's numerical stability depends on the matrix being diagonally dominant or close to it — the `L_jj = sqrt(...)` can produce a non-positive argument on indefinite or near-singular matrices. `sim-soft`'s post-PSD-projection Hessian satisfies this for the canonical problem; if it fails, the setup falls back to Jacobi with a logged warning.

## AMG: scene-scale

Specified in [§02](02-amg.md). This leaf references rather than duplicates. Summary: AMG cuts CG iteration count to single digits on 100k+ DOF scenes, at setup cost that amortizes across Newton iterations within a timestep (pattern cached, numerical rebuild cheap).

## The selector

Preconditioner selection happens at Newton-step setup time, after the matrix pattern is known:

```rust
pub fn select_preconditioner(
    op:        &dyn SpMvOp,
    dof_count: usize,
    prefers_matrix_free: bool,
) -> Box<dyn Preconditioner> {
    if prefers_matrix_free {
        return Box::new(JacobiPrecond::new(op));        // only matrix-free-compatible tier
    }
    match dof_count {
        0..=30_000      => Box::new(JacobiPrecond::new(op)),
        30_001..=99_999 => Box::new(Ic0Precond::new(op)),
        _               => Box::new(AmgPrecond::new(op)),
    }
}
```

Two subtleties:

**Matrix-free scenes always use Jacobi.** IC0 and AMG require matrix-entry inspection; matrix-free has no explicit matrix. This is the [Ch 01 §02 custom](../01-sparse-matrix/02-custom.md) constraint, restated at the selector.

**The breakpoints are defaults, not hard thresholds.** `sim-soft` exposes a `PreconditionerOverride::Jacobi | Ic0 | Amg | Auto` flag on the solver config. Phase E benchmarking will measure the actual crossover points on Phase E's target hardware; the 30k / 100k defaults are from CPU-side soft-body literature (similar matrix structure) and re-tuned as GPU benchmarks land.

## The preconditioner contract

All three preconditioners implement the same trait:

```rust
pub trait Preconditioner {
    fn apply(&self, residual: &GpuTensor<f32>, output: &mut GpuTensor<f32>);
    fn setup(&mut self, op: &dyn SpMvOp);                   // rebuild for new matrix
    fn setup_cost_ms(&self) -> f64;                          // for telemetry
    fn memory_bytes(&self) -> usize;                         // for budget checks
}
```

CG ([§00](00-cg.md)) and MINRES ([§01](01-minres.md)) both consume `&dyn Preconditioner`; the solver doesn't care which tier is in use. Adding a new preconditioner tier (block Jacobi, polynomial, domain-decomposition, etc.) is implementing the trait plus adding a selector branch — the existing solver and solver-caller code is unchanged.

## What this sub-leaf commits the book to

- **Three-tier preconditioner hierarchy.** Jacobi (< 30k DOFs or matrix-free), IC0 (30k–100k DOFs), AMG (≥ 100k DOFs). Breakpoints are tunable defaults, re-measured in Phase E benchmarks.
- **Jacobi is the matrix-free-compatible tier.** Only diagonal-entry-dependent preconditioner; the [§02 matrix-free path](../01-sparse-matrix/02-custom.md) uses it regardless of DOF count.
- **IC0 is serial-setup on GPU in Phase E.** Parallel IC0 variants (block-coloring, Anzt-style) exist and are considered for Phase H if the setup cost proves a bottleneck; Pass 1 ships the simpler serial setup.
- **AMG is pattern-cached across Newton iterations**, rebuilt on re-mesh. Setup amortization pays via [Ch 02 parent Claim 2](../02-sparse-solvers.md)'s preconditioner-on-tape pattern.
- **One `Preconditioner` trait covers all tiers.** Adding a new preconditioner is implementing the trait plus a selector branch; the solver code is unchanged.
