# Custom mesh-derived formats

Explicit storage of the assembled Hessian scales linearly with mesh nnz. Using the upper-triangular BSR default from [§01](01-bsr.md), the Hessian itself takes ≈3.5 MB on the 30k-DOF canonical scene and ≈35 MB on a 300k-DOF scene. The AMG preconditioner's coarse-grid hierarchy ([Ch 02 §02](../02-sparse-solvers/02-amg.md)) adds ≈1–2× the fine-grid matrix size on top, the autograd tape ([Ch 03](../03-gpu-autograd.md)) adds a per-step record, CG needs ≈5 vectors of scratch, and the line-search plus rendering layer compete for the rest. On a consumer GPU with 8–12 GB of VRAM, the 300k-DOF combined footprint approaches several hundred MB — tight but not yet over the cliff. At mesh scales beyond 300k DOFs (which the [cf-design live-authoring loop](../../70-sdf-pipeline/04-live-remesh.md) can generate during coarse-to-fine zoom workflows) the explicit-Hessian path runs out of headroom first. Matrix-free SpMV trades explicit storage for recomputation, and for those larger-than-canonical scenes the trade pays. [Ch 01 parent Claim 2](../01-sparse-matrix.md) names this as a first-class option; this leaf specifies what "matrix-free" concretely means in `sim-soft` and where the break-even lies.

## The matrix-free kernel

Matrix-free SpMV for soft-body FEM computes $y = H\, v$ by walking the element list and evaluating each tet's local contribution on the fly:

```wgsl
@compute @workgroup_size(64)
fn spmv_matrix_free(@builtin(global_invocation_id) gid: vec3<u32>) {
    let tet = gid.x;
    if (tet >= n_tets) { return; }
    let v0 = tet_connectivity[tet].x;
    let v1 = tet_connectivity[tet].y;
    let v2 = tet_connectivity[tet].z;
    let v3 = tet_connectivity[tet].w;
    // Load per-tet reference geometry B_e (3x4) and material params
    let B = reference_geometry[tet];
    let mat = material_params[tet];
    // Load current iterate vertices
    let v_in = load_tet_v(tet, v_input);  // 12-vec
    // Evaluate K_e * v_in via closed-form neo-Hookean tangent
    let K_e_v = eval_tangent_times_vec(B, mat, v_in);
    // Scatter 4x3 result into global y (atomic-CAS)
    scatter_tet_output(tet, K_e_v, y);
}
```

Two properties distinguish this from the [§01 BSR](01-bsr.md) path:

**No assembled Hessian buffer.** `block_values` doesn't exist; its ≈3.5 MB on the canonical scene (≈35 MB on a 300k-DOF scene) of Phase E storage is freed. The per-tet $B_e$ (4 columns × 16 bytes = 64 bytes, scene-static per [§01 bind groups](../00-wgpu-layout/01-bind-groups.md)) and per-tet material scalars (≈12 bytes) are the only tet-local data, both of which BSR already needs for re-assembly on Newton iterate change. The *incremental* memory saving is the `block_values` buffer itself.

**Recomputation cost is 2–3× per SpMV.** BSR's SpMV reads 9 `f32`s per block and does 9 multiply-adds; matrix-free evaluates the neo-Hookean tangent per tet (≈200–500 FLOPs per tet for the full $\partial P/\partial F$ contraction into the 4×3 output — larger than the stress-only FLOP count from [§00 kernel-types](../00-wgpu-layout/00-kernel-types.md) because the tangent is a 9×9 fourth-order derivative) and then scatters. BSR is memory-bound on the canonical scene (the per-Hessian-entry read is the rate-limiter); matrix-free is compute-bound (the tangent evaluation is the rate-limiter). Net per-SpMV wall-clock is ≈2–3× BSR's, tracking the parent chapter's figure. In round numbers: an explicit-path Newton step that hits the Phase E frame budget becomes a matrix-free step that takes ≈2–3× as long. The slowdown is the price for a problem that fits; the combined Hessian + AMG + tape + CG workspace at beyond-canonical mesh scales is what pushes the budget over the cliff on consumer VRAM, not the canonical scene.

## Break-even point and the runtime flag

The `SpMvOp` trait from [Ch 02 §00 CG](../02-sparse-solvers/00-cg.md) abstracts the choice:

```rust
pub trait SpMvOp {
    fn apply(&self, input: &GpuTensor<f32>, output: &mut GpuTensor<f32>);
    fn estimated_gflops(&self) -> f64;  // for perf monitoring
    fn memory_bytes(&self) -> usize;    // BSR = block_values + metadata; matrix-free = 0 per-Hessian
}

pub struct BsrSpMvOp { /* wraps block_values, block_col_idx, block_row_ptr buffers */ }
pub struct MatrixFreeSpMvOp { /* wraps tet_connectivity, reference_geometry, material refs */ }
```

The chassis picks between them at Newton-step setup time based on a memory-budget heuristic:

```rust
pub fn select_spmv(dof_count: usize, available_vram: usize) -> Box<dyn SpMvOp> {
    let bsr_bytes = estimate_bsr_footprint(dof_count);  // upper-tri BSR ≈ 0.12 * dof_count KB on canonical meshes
    let budget_headroom = 0.3;  // keep 30% for tape + preconditioner + workspace
    if bsr_bytes < (available_vram as f64 * (1.0 - budget_headroom)) as usize {
        Box::new(BsrSpMvOp::new(...))
    } else {
        Box::new(MatrixFreeSpMvOp::new(...))
    }
}
```

The threshold is not a hard mesh-size cutoff — it is a memory-fit check. A 50k-DOF scene on a 4 GB integrated GPU uses matrix-free; the same scene on a 24 GB workstation GPU uses BSR. The chassis exposes an override for benchmarking (forcing one or the other regardless of budget) but the default is budget-driven.

## What matrix-free does *not* save

The matrix-free path does not eliminate every Hessian-sized buffer; two costs remain.

**The preconditioner still needs explicit storage.** AMG's coarse-grid hierarchy ([Ch 02 §02](../02-sparse-solvers/02-amg.md)) is explicitly constructed from the fine-grid matrix and cannot be computed matrix-free — coarsening requires inspecting matrix entries to pick interpolation weights. For very-large-scene matrix-free runs, the preconditioner falls back to Jacobi (which *is* matrix-free — just a per-DOF scalar divisor) or to IC0 on a coarsened representation. CG iteration count goes up accordingly per [Ch 02 Claim 5](../02-sparse-solvers.md); matrix-free is a memory-tradeoff, not a CG-iter reduction.

**The autograd tape still records the Newton step.** [Ch 03's tape](../03-gpu-autograd.md) records kernel-handle entries per forward op; matrix-free SpMV is one kernel handle the same way BSR SpMV is. The tape's memory cost is dominated by per-tensor entries, not by the matrix itself, so matrix-free does not meaningfully shrink the tape.

**The per-Newton-iterate scratch persists.** CG needs ≈5 vectors of DOF-length scratch (residual, conjugate direction, Ap, preconditioner workspace, line-search step); at 300k DOFs × 4 bytes per `f32` × 5 vectors that is ≈6 MB total, independent of matrix storage format. Modest in absolute terms, but matrix-free does not help here — it only removes the assembled-Hessian term.

## Future extensions not committed here

Two variants are plausible under the `SpMvOp` trait but not committed in Phase E:

**Hybrid BSR + matrix-free.** Assemble the diagonal-adjacent blocks (high-locality, high reuse within CG) explicitly, leave the off-diagonal "outer band" matrix-free. The split point is mesh-dependent and would require a Phase E benchmarking pass to justify. Not shipped in Pass 1.

**Fused Hessian-vector product with autograd-replayed tangent.** Instead of a dedicated neo-Hookean tangent kernel, evaluate $H v$ by replaying the forward pass's autograd tape with the tangent seed equal to $v$ — a Pearlmutter-style trick. Requires [Ch 03's tape](../03-gpu-autograd.md) to support arbitrary tangent replays, which Pass 1 does not commit to. Latent option, called out as an avenue for Phase-H-or-later scaling work.

## What this sub-leaf commits the book to

- **Matrix-free SpMV is a first-class fallback.** Selected at Newton-step setup time by memory-budget heuristic, not a hard mesh-size cutoff. Lets `sim-soft` run on consumer GPUs where the combined Hessian + AMG + tape + workspace footprint approaches the VRAM ceiling — typically beyond the canonical 30k-DOF scene at consumer-GPU budgets.
- **The `SpMvOp` trait is the abstraction boundary.** CG and every other solver consumes `&dyn SpMvOp`; BSR vs matrix-free is invisible to the solver. [Ch 02 §00 CG](../02-sparse-solvers/00-cg.md) is coded against the trait.
- **Per-SpMV wall-clock is ≈2–3× BSR.** Matrix-free is not free; when memory fits, BSR wins. When it doesn't, matrix-free is the only path that runs at all.
- **Preconditioner compatibility is limited.** AMG requires explicit matrix access; matrix-free runs fall back to Jacobi or coarse IC0. CG iteration count rises accordingly.
- **Pearlmutter-style tape-replayed Hv and hybrid BSR+matrix-free are latent.** Both are valid extensions under `SpMvOp`; neither is on the Phase E roadmap. Named so that a future deep-dive has a scope boundary to push against.
