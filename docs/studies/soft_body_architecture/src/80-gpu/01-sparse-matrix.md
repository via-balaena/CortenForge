# Sparse matrix structures

`sim-soft`'s Hessian is sparse, symmetric, and has a block structure determined by the mesh: each tet contributes a 12×12 dense block (3 DOFs × 4 vertices) to the global matrix, and each active contact pair contributes a small block on top. On CPU, [faer's `SparseColMat`](../50-time-integration/00-backward-euler.md) handles this cleanly. On GPU, there is no single portable crate that does the same job; this chapter names the three formats `sim-soft` uses and which kernel uses which.

| Section | What it covers |
|---|---|
| [CSR on GPU](01-sparse-matrix/00-csr.md) | Compressed sparse row: row-pointer + column-index + value arrays. Standard SpMV format, well-suited to general sparse ops; used for the assembled global Hessian when passed to the CG solver |
| [BSR for block structure](01-sparse-matrix/01-bsr.md) | Block sparse row: groups DOFs into 3×3 blocks matching the per-vertex structure. 30% fewer indirection levels than CSR for soft-body Hessians; the default format for assembled operators in the CG iteration |
| [Custom mesh-derived formats](01-sparse-matrix/02-custom.md) | Matrix-free operators that never explicitly form the Hessian — SpMV is computed element-by-element from mesh topology + per-tet stiffness. Trades memory for recomputation; enables larger problems than would fit explicit Hessian storage |

Three claims.

**BSR is the default for assembled Hessians.** The 3×3 block structure from per-vertex DOF groupings matches exactly, eliminating one level of column-index indirection compared to CSR on the same matrix. Memory footprint is ≈70% of CSR on the canonical scene (30k DOFs, ≈1.5M nonzeros after dropping zero blocks); SpMV throughput is ≈40% higher on modern hardware due to better vectorization within the 3×3 blocks. The gain is structural, not a micro-optimization.

**Matrix-free is a first-class option for large scenes.** Explicit Hessian storage on a 300k-DOF scene is ≈200 MB at single-precision — fits on-device but eats budget that could go to the RHS, the preconditioner, or the line-search workspace. Matrix-free computes $H\, v$ on the fly by walking the element list, evaluating each tet's local 12×12 contribution, and accumulating into the output. The per-SpMV cost is 2–3× higher than BSR, but the memory saved is the difference between "fits on a mid-tier GPU" and "doesn't." [Ch 02's CG](02-sparse-solvers/00-cg.md) supports both via a `SpMvOp` trait; the choice is a runtime flag keyed on DOF count.

**Symmetry is exploited, not just asserted.** The Hessian is SPD ([Part 5 Ch 00 Claim 2](../50-time-integration/00-backward-euler.md)). Upper-triangular storage halves the memory; the CG inner loop is coded against the symmetric variant directly, not via a "symmetrize the full matrix" wrapper. This is one of the places where rolling the GPU path ourselves pays off — a general-purpose GPU sparse library would store the full matrix for flexibility, and we would lose the 2× memory benefit.
