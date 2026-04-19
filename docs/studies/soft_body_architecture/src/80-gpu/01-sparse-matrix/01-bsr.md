# BSR for block structure

Block Sparse Row groups the sparse matrix into fixed-size dense blocks, indexed by block-row and block-column. For soft-body FEM, the 3×3 block size falls out of the physics: each vertex has 3 displacement DOFs, so every stiffness entry $\partial^2 U / \partial x_i \partial x_j$ between vertex $i$ and vertex $j$ is a 3×3 matrix. Storing the matrix block-by-block rather than scalar-by-scalar is not a micro-optimization — it is the data structure that matches the physics. The two gains [Ch 01 parent Claim 1](../01-sparse-matrix.md) quotes (≈40% SpMV throughput, ≈30% memory saving) compose with a third this sub-leaf derives from the [§00 kernel-types assembly pattern](../00-wgpu-layout/00-kernel-types.md) (≈9× lower atomic contention on per-element scatter); all three come from the block-level match, not from tuning.

## The block-indexed layout

BSR stores the matrix as four arrays, with one additional level of indirection compared to CSR:

```wgsl
@group(0) @binding(0) var<storage, read> block_row_ptr: array<u32>;   // length n_block_rows + 1
@group(0) @binding(1) var<storage, read> block_col_idx: array<u32>;   // length n_blocks
@group(1) @binding(0) var<storage, read> block_values: array<f32>;    // length n_blocks * 9
```

`block_row_ptr[i]` offsets into `block_col_idx` and `block_values` for block-row $i$ (corresponding to vertex $i$). Each `block_col_idx[k]` is a *vertex index*, not a DOF index — the block at `block_values[9*k .. 9*k+9]` is the 3×3 stiffness between vertices $i$ and `block_col_idx[k]`. The block is stored row-major (9 consecutive `f32`s: $M_{00}, M_{01}, M_{02}, M_{10}, \dots, M_{22}$).

Storage count on the 30k-DOF canonical scene (10k vertices, ≈170k nonzero blocks): `block_values` = 170k × 9 × 4 B = 6.1 MB. `block_col_idx` = 170k × 4 B = 0.7 MB. `block_row_ptr` = 10k × 4 B = 40 kB. Total ≈ 6.8 MB. CSR on the same matrix: ≈12 MB. [The parent chapter's 30% memory saving claim](../01-sparse-matrix.md) assumes WGSL `mat3x3<f32>` storage (48-byte padded blocks); with the packed `array<f32>` layout this sub-leaf commits to, the saving is closer to 43%. Either choice is under the same trait, so the solver code is unaffected; the packed version is what Phase E ships because the memory-saving dominates the marginal cost of explicit 9-element indexing.

## SpMV: one thread per block-row, cooperative column walk

```wgsl
@compute @workgroup_size(64)
fn spmv_bsr(@builtin(global_invocation_id) gid: vec3<u32>) {
    let vi = gid.x;                                          // vertex index
    if (vi >= n_vertices) { return; }
    let start = block_row_ptr[vi];
    let end = block_row_ptr[vi + 1u];
    var y_block = vec3<f32>(0.0, 0.0, 0.0);
    for (var k: u32 = start; k < end; k = k + 1u) {
        let vj = block_col_idx[k];
        let b_off = 9u * k;
        let x_j = vec3<f32>(x[3u*vj], x[3u*vj + 1u], x[3u*vj + 2u]);
        // row-major 3x3 block multiply
        y_block.x = y_block.x + block_values[b_off + 0u] * x_j.x
                              + block_values[b_off + 1u] * x_j.y
                              + block_values[b_off + 2u] * x_j.z;
        y_block.y = y_block.y + block_values[b_off + 3u] * x_j.x
                              + block_values[b_off + 4u] * x_j.y
                              + block_values[b_off + 5u] * x_j.z;
        y_block.z = y_block.z + block_values[b_off + 6u] * x_j.x
                              + block_values[b_off + 7u] * x_j.y
                              + block_values[b_off + 8u] * x_j.z;
    }
    y[3u*vi]       = y_block.x;
    y[3u*vi + 1u]  = y_block.y;
    y[3u*vi + 2u]  = y_block.z;
}
```

Three properties that give BSR its throughput advantage:

**Block-row work is nearly uniform.** Every block-row's inner loop processes the same amount of work per nonzero block (9 mul-adds), so the per-thread FLOP count depends only on the number of neighboring vertices. For well-shaped tet meshes this count is bounded — ≈8–15 neighbors per vertex regardless of overall mesh size. Warps stay balanced, idle-fraction drops to ≈5–10% versus CSR's 10–40%. This is the primary source of the SpMV throughput gain.

**Indirection pressure is amortized across the block.** CSR indexes 9 scalars via 9 separate `col_idx` reads and 9 separate `values` reads; BSR reads `block_col_idx` once and then streams 9 `block_values` reads contiguously. The L1 cache sees the 9-float block as one cache line (36 bytes, fits comfortably in a 64-byte line); CSR sees 9 non-contiguous scalar reads. The effective memory-bandwidth demand is ≈65% of CSR's for the same matrix, which composes with the uniform-work effect to produce the 40% throughput win.

**`x`-vector reads are fan-out-3 per block.** A single `block_col_idx[k]` lookup feeds three `x` reads (one per DOF of vertex $vj$). Fetching `x[3*vj]`, `x[3*vj+1]`, `x[3*vj+2]` together is coalesced within the vec3 if the input vector is laid out in `vec3<f32>` form with WGSL's 16-byte stride ([§01 bind groups](../00-wgpu-layout/01-bind-groups.md)), so the 3-element load takes one cache line. CSR's per-scalar access requires three separate lookups that may or may not coalesce depending on driver.

## Symmetry: upper-triangular BSR with cheap off-diagonal writeback

Soft-body Hessians are SPD ([Part 5 Ch 00 Claim 2](../../50-time-integration/00-backward-euler.md)) and therefore symmetric. Storing only the upper triangle (vertices $i < j$ only) plus the diagonal halves `block_values` to ≈85k blocks on the canonical scene, trimming `block_values` from 6.1 MB to 3.1 MB. The memory saving pays because the off-diagonal writeback inside SpMV is a 3-DOF block, not a scalar: one atomic-CAS wrapper call performs 9 `f32` additions into the lower-triangle row's output. Per-block atomic-CAS at 9-float payload costs ≈3× a native atomic `f32` add (consistent with [§01 bind groups](../00-wgpu-layout/01-bind-groups.md)'s atomic pattern), but the saved memory-bandwidth cycles for not having to read the lower triangle absorbs the CAS cost at typical nonzero densities.

The CPU-side faer Cholesky ([Part 5 Ch 00 Claim 2](../../50-time-integration/00-backward-euler.md)) already uses upper-triangular sparse layout, so the BSR upper-triangular convention matches the CPU path's storage exactly — the GPU's memory layout is a straightforward mirror of the CPU matrix, which simplifies the CPU→GPU upload path and keeps the "GPU port is a mechanical translation" property from [Part 11 Ch 03's build order](../../110-crate/03-build-order.md).

## Assembly: per-tet scatter into BSR-block atomic targets

The per-element assembly kernel from [§00 kernel-types](../00-wgpu-layout/00-kernel-types.md) walks tets in parallel, computes each tet's 12×12 local stiffness (4 vertices × 3 DOFs squared), and scatters the 4×4 = 16 block contributions into the global `block_values`. Each block write is one atomic-CAS over 9 `f32`s (a 9-entry compare-and-swap loop per block, not 9 independent scalar CASes — the WGSL pattern uses a block-sized u32-array view). Contention is bounded by the mesh's per-vertex-pair valence — a shared vertex-pair is written by ≈2–3 tets per Newton iteration, so the CAS retry rate is ≈2–3% under uniform contention.

This is the per-element-plus-atomic-scatter pattern [Ch 00 §00's Assembly](../00-wgpu-layout/00-kernel-types.md) names explicitly. The 9-float atomic payload is what makes the pattern work — if assembly scattered into CSR scalar entries, each tet would issue 4 × 4 × 9 = 144 independent atomic ops per tet, versus 16 block atomics. Atomic-bus pressure drops by 9× at the BSR target, and the CAS retry rate stays below 5% across the canonical contention range.

## What this sub-leaf commits the book to

- **BSR is the primary GPU storage format for the assembled Hessian.** 9-float blocks match the physics; packed-block storage is ≈6.8 MB on the canonical scene (≈43% less than CSR; improves on the parent chapter's 30% mat3x3-padded figure), SpMV throughput is ≈40% higher, assembly atomic pressure is ≈9× lower.
- **Row-major block layout inside each block.** 9 consecutive `f32`s per block, streamed into L1 as one cache line. Column-major inside the block would cost a transpose on SpMV and is not used.
- **Upper-triangular storage is the default.** The symmetric Hessian stores only $i \leq j$ blocks; SpMV writes the off-diagonal contribution to the lower-triangle row via a 9-float atomic-CAS. Cuts `block_values` memory by ≈50% at the cost of a ≈3× atomic-vs-native-add per off-diagonal.
- **Assembly is per-element-plus-9-float-atomic-scatter.** The scatter atomic payload is a whole 3×3 block, not scalars; contention is ≈9× lower than a CSR-target scatter would produce. Consistent with [Ch 00 §00's per-element assembly pattern](../00-wgpu-layout/00-kernel-types.md).
