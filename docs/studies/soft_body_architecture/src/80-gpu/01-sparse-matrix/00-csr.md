# CSR on GPU

Compressed Sparse Row is the lingua franca of sparse linear algebra — three arrays, documented in every textbook, supported by every GPU sparse library that exists. `sim-soft`'s GPU path does not use CSR as its primary format ([§01 BSR](01-bsr.md) is the default for assembled Hessians), but CSR is supported as a fallback for three specific workflows and as the interop format at crate boundaries where a non-`sim-soft` consumer expects the standard format. This leaf specifies what `sim-soft`'s CSR looks like on GPU, what uses it, and what the performance penalty is versus BSR.

## The three-array layout

CSR stores an $n \times n$ sparse matrix as three arrays:

```wgsl
@group(0) @binding(0) var<storage, read> row_ptr: array<u32>;   // length n+1
@group(0) @binding(1) var<storage, read> col_idx: array<u32>;   // length nnz
@group(1) @binding(0) var<storage, read> values:  array<f32>;   // length nnz
```

`row_ptr[i]` is the offset into `col_idx` and `values` where row $i$'s nonzeros begin; `row_ptr[i+1] - row_ptr[i]` is the row's nonzero count. `col_idx[k]` is the column index of the $k$-th stored nonzero; `values[k]` is its numerical value. Symmetric matrices can store only the upper (or lower) triangle with a factor-of-2 memory saving; the SpMV kernel then symmetrizes on the fly.

For `sim-soft`'s 30k-DOF canonical scene, nnz ≈ 1.5M scalar entries (≈170k BSR blocks × 9 entries each). CSR full-matrix storage: 1.5M × 4 bytes for `values` + 1.5M × 4 bytes for `col_idx` + 30k × 4 bytes for `row_ptr` = ≈12 MB. BSR full-matrix storage: ≈170k × 36 bytes (9-float block) + ≈170k × 4 bytes for `block_col_idx` + 10k × 4 bytes for `block_row_ptr` = ≈6.8 MB. BSR wins the full-matrix memory comparison on this scene by ≈43%. (Phase E's actual BSR default is upper-triangular per [§01](01-bsr.md), which halves `block_values` to ≈3.5 MB total; the CSR fallback stays full-matrix for the reason the symmetric-storage paragraph below gives — atomic-CAS cost on scalar-entry scatter exceeds the memory saving.)

## SpMV on GPU, row-per-thread

The standard GPU CSR-SpMV dispatches one thread per row:

```wgsl
@compute @workgroup_size(64)
fn spmv_csr(@builtin(global_invocation_id) gid: vec3<u32>) {
    let row = gid.x;
    if (row >= arrayLength(&row_ptr) - 1u) { return; }
    let start = row_ptr[row];
    let end = row_ptr[row + 1u];
    var sum = 0.0;
    for (var k: u32 = start; k < end; k = k + 1u) {
        sum = sum + values[k] * x[col_idx[k]];
    }
    y[row] = sum;
}
```

Three performance characteristics worth naming:

**Load imbalance across rows.** Rows with ≈50 nonzeros (interior vertices in a well-shaped mesh) and rows with ≈10 nonzeros (boundary vertices) are scheduled into the same warp/subgroup, so the long-row threads idle their short-row neighbors. The idle-fraction is mesh-dependent; on uniform Delaunay meshes it is ≈10%, on high-aspect tet meshes it is ≈25–40%. BSR's block structure mitigates this by making the per-block-row work count more uniform (every block-row has the same per-block cost regardless of overall block count), which is part of the 40% SpMV speedup [Ch 01 parent Claim 1](../01-sparse-matrix.md) quotes.

**Gather from `x` is unpredictable.** The access `x[col_idx[k]]` is a scatter-read into the input vector — no coalescing guarantee, because `col_idx` can point anywhere. Modern GPUs' L1/L2 caches absorb most of this cost for mesh-local connectivity (neighbor vertices have nearby indices if the mesh has good numbering), but cache-miss rates are noticeably higher than for dense kernels. A Cuthill-McKee or METIS reordering pass at mesh-load time cuts miss rates by ≈30%; `sim-soft`'s mesh loader runs one.

**Symmetric storage halves memory but doubles code paths.** Upper-triangular CSR stores each off-diagonal entry once; the SpMV kernel adds both `A[i,j] * x[j]` to row $i$'s output and `A[i,j] * x[i]` to row $j$'s output, requiring an atomic write on the latter (to avoid the race between different threads both updating the same off-diagonal row). On CPU, atomic-add on `f32` is a single instruction; on GPU, it is the `atomic<u32>` + compare-and-swap loop from [§00 kernel-types](../00-wgpu-layout/00-kernel-types.md). `sim-soft` skips the symmetric optimization for CSR fallback because the atomic-CAS cost exceeds the memory saving at typical scene sizes; BSR's upper-triangular storage is simpler because atomic-adds there are on 9-float blocks anyway, and the memory saving pays.

## Where CSR earns its place

Three workflows keep CSR in the build:

**Interop with external sparse libraries.** If a user wants to swap in a `cuSPARSE` or `rocSPARSE` solve for benchmarking, or pipe the matrix into Python via `scipy.sparse.csr_matrix`, CSR is the expected format. `sim-soft::gpu` exposes a `to_csr()` conversion on the global Hessian buffer; the conversion is a one-shot scan over the BSR-block layout, costing ≈5 ms on the canonical scene and not in any hot path.

**Diagnostic tooling.** Gradient-check regression tests ([Part 11 Ch 04](../../110-crate/04-testing/03-gradcheck.md)) dump the Hessian to disk and compare against a CPU reference; CSR is the on-disk format because it round-trips cleanly through every sparse library every reviewer has installed. BSR on disk would require `sim-soft`-specific tooling to inspect.

**Non-uniform-block scenes.** If a future scene mixes Tet4 (3-DOF-per-vertex) and a rigid-body DOF (6-DOF-per-body), the per-vertex block size is no longer uniformly 3×3, and BSR's fixed-block-size assumption breaks. CSR handles this without special-casing — every entry is just a scalar. `sim-soft`'s Phase A–E scenes are Tet4-only, so this workflow is latent; it is kept in the build so that Phase-H-or-later rigid-body-coupled soft-body extensions don't need a format rewrite.

## What CSR does *not* do

- **Not the primary assembly target.** The per-element assembly from [§00 kernel-types](../00-wgpu-layout/00-kernel-types.md) scatters into BSR blocks, not CSR scalar entries. A CSR assembly kernel would have nnz-scale atomic-add contention rather than block-scale contention — 9× more atomic traffic — and is avoided.
- **Not the CG SpMV target.** CG's inner SpMV runs against BSR ([§01](01-bsr.md)) on the hot path. The CSR-SpMV above is used for Hessian inspection, gradient checks, and interop; not the Newton inner loop.
- **Not a vendor-library bridge.** `sim-soft` does not call into `cuSPARSE` or equivalent for CSR ops even when CSR is in use — the portability argument from [Ch 00](../00-wgpu-layout.md) applies. The WGSL CSR-SpMV kernel in this leaf is what runs, fallback or not.

## What this sub-leaf commits the book to

- **CSR is a supported fallback and interop format, not the primary path.** Three workflows keep it in the build (external-library interop, diagnostic dumps, non-uniform-block future scenes); none is on the Phase E hot path.
- **Row-per-thread SpMV is the canonical kernel shape.** Load imbalance and gather-scatter are the two known inefficiencies; mesh-reordering cuts the second by ≈30%, but CSR-SpMV still loses ≈40% throughput to BSR on the canonical scene.
- **Symmetric CSR storage is not used.** The atomic-CAS cost of off-diagonal updates exceeds the memory saving at `sim-soft` scales; BSR's upper-triangular storage is where symmetry pays.
- **CSR round-trips cleanly through external tooling.** The `to_csr()` conversion is one-shot and ≈5 ms on the canonical scene; round-tripping through `scipy.sparse` or `cuSPARSE` for gradient-check regression is a supported Phase E workflow.
