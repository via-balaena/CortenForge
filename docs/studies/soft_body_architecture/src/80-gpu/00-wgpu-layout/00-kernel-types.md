# Kernel types

The [Ch 00 parent's Claim 2](../00-wgpu-layout.md) commits to *per-element* as the default workload shape, with three named exceptions. This leaf expands the argument: every compute kernel `sim-soft` ships on GPU falls into one of four workload shapes, each with a different thread-count rule, a different memory-access pattern, and a different occupancy profile. Shoehorning any one of them into another's shape is what causes the "my GPU rewrite is only 3× faster than CPU" outcome this book is explicitly avoiding — the wins come from matching each kernel to the shape its work actually has.

## The four shapes

| Shape | Thread count | Access pattern | Example kernels |
|---|---|---|---|
| Per-element (per-tet) | `n_tets` | Indexed via connectivity; coalesced within a tet | Neo-Hookean stress, per-element stiffness $K_e$, PSD projection, per-tet strain |
| Per-contact-pair | `n_active_pairs` | Indexed via active-set list; two-vertex scatter | IPC barrier eval, barrier Hessian, per-pair $\hat d$ aggregation |
| Per-DOF | `3 * n_vertices` | Dense-vector SpMV row-per-thread; reductions | SpMV in CG, CG inner products, line-search candidate eval, residual norm |
| Per-tet (state) | `n_tets` | Gather across mesh; per-element serial in inner loop | State transfer on re-mesh, stress-tensor readout, Gauss-point quadrature for material-field sampling |

Per-element and per-tet share a thread count (`n_tets`) but not an access pattern — per-element walks element-local data (the 4 vertex positions plus the reference-geometry $B_e$), while per-tet-state walks element-to-neighbor or element-to-quadrature-point gather patterns that touch more memory per thread. They are listed separately because fusing them into a single shape would force one access pattern onto the other, and the loser is whichever kernel ends up with strided reads. Keeping them distinct lets `sim-soft`'s dispatch logic pick the right `workgroup_size` per kernel class.

## Why the per-element shape is the GPU-friendly one

The three architectural facts that make per-element the default:

**Thread count scales with mesh size, not problem dimension.** A 30k-DOF scene has ≈10k vertices and ≈50k tets (typical tet/vertex ratio 5–6 for well-shaped meshes); every tet gets one thread, every thread does ≈200 FLOPs of neo-Hookean stress evaluation. This lands in the 30k–60k-thread-count band that modern GPUs schedule efficiently — one dispatch, one workgroup-count-rounded-up, no multi-pass decomposition. Problems with fewer DOFs still saturate the GPU via the per-element count; problems with more DOFs scale linearly.

**Memory per thread is bounded by the element's reference geometry, not the global mesh.** Each thread reads exactly 4 vertex positions (each `vec3<f32>` is 16 bytes in storage layout — [§01 bind groups](01-bind-groups.md) explains WGSL's padding rules), plus a 3×4 reference-geometry matrix $B_e$ stored as WGSL `mat4x3<f32>` (64 bytes), and writes 6 unique stress components (stress tensor is symmetric). ≈128 bytes read, ≈24 bytes written, per thread, per kernel dispatch. On a mid-tier GPU at 500 GB/s memory bandwidth, 50k tets × 152 bytes ≈ 7.6 MB per pass — ~15 μs of bandwidth time, fully compute-bound.

**No inter-thread dependencies within the dispatch.** Per-element kernels are embarrassingly parallel within one dispatch: thread $i$ writes tet $i$'s stress tensor, thread $j$ writes tet $j$'s, no shared-memory coordination, no atomic writes (each thread owns its output slot exclusively). The only inter-tet coordination happens at the *assembly* stage, where per-tet contributions accumulate into a global stiffness matrix — that is where atomics live, and the next section names it as a separate per-element-plus-scatter shape.

## Assembly is per-element with scatter, not a separate shape

Per-element kernels come in two flavors — pure (no scatter) and assembly (scatter into global state). Pure kernels (stress evaluation, PSD projection) write to per-tet output buffers; assembly kernels (global stiffness $K = \sum_e K_e$, global force $f = \sum_e f_e$) scatter per-tet contributions into shared global buffers. The scatter uses atomic adds on the BSR block indices (see [Ch 01 §01 BSR](../01-sparse-matrix/01-bsr.md)), and atomic contention is bounded by the mesh's valence — each global row is written by ≈8–15 tets for a well-shaped tet mesh.

This is the place where a naive "one kernel per thread per output" decomposition would fail: scattering 50k tets × 12 DOFs into 30k global DOFs with no atomics would require a multi-pass histogram, which on GPU is slower than the atomic-add approach by a factor of 2–3×. The assembly shape is per-element-plus-scatter, and the scatter is atomic. Both halves of that statement are load-bearing.

The assembly step is where the [Ch 01 §00 CSR](../01-sparse-matrix/00-csr.md) vs [§01 BSR](../01-sparse-matrix/01-bsr.md) choice from Ch 01 is felt on GPU — BSR's 3×3-block indirection means each atomic target is a 9-float block, not a single scalar, which cuts the contention-spread factor by roughly 9× and makes the scatter kernel faster despite the larger atomic payload. The combination of per-element shape + BSR target + atomic-add scatter is the canonical assembly pattern, and it is not a generic sparse-library operation — it is a `sim-soft`-specific kernel family.

## Per-contact-pair is the contact-force shape, dispatched against a dynamic list

The IPC barrier evaluation and its derivatives are dispatched *per active contact pair*, not per contact primitive and not per mesh element. The active pair list is maintained by [Part 4 Ch 01's adaptive-$\kappa$ machinery](../../40-contact/01-ipc-internals/01-adaptive-kappa.md) and changes between Newton iterations — typically by ≤10% per iteration, but occasionally by 2–3× when a new contact region activates.

Two properties of this shape that differ from per-element:

**Thread count is dynamic.** The dispatch reads `n_active_pairs` at the start of each compute pass and dispatches `ceil(n_active_pairs / workgroup_size)` workgroups. The pair list itself lives in a GPU buffer that is rebuilt each iteration by a separate broadphase kernel (spatial-hash-backed; not deep-dived in Pass 1). Dynamic dispatch costs the command-encoder one extra roundtrip of list-length readout if done naively; `sim-soft` avoids that by dispatching an upper-bound workgroup count and writing a dispatch-epilogue guard that makes over-dispatched threads no-op. The epilogue guard pattern is ≈5% overhead at typical contact densities and avoids the roundtrip entirely.

**Scatter is two-vertex, not one-vertex.** Each contact pair touches two primitives (vertex-face, edge-edge, vertex-vertex), which means each pair's VJP writes to *two* vertex's gradient slots. The atomic-add scatter pattern is the same as per-element assembly but with fan-out 2 instead of fan-out 4 (for tet-element). Contention is lower than per-element assembly in the uniform-mesh regime and higher than per-element assembly in the localized-contact regime (where the same few vertices are active-pair endpoints repeatedly). `sim-soft` accepts this; Phase E's benchmarks will measure it and decide whether a reorder pass is needed.

## Per-DOF is the iterative-solver shape

The CG inner loop's operations — SpMV, vector-vector dot products, element-wise AXPYs — are per-DOF: one thread per degree of freedom, 3 DOFs per vertex. At 30k DOFs, 30k threads is the dispatch size; on mid-tier GPUs this is comfortably in the occupancy sweet spot.

The non-trivial piece is the *reductions* — dot products and residual-norm computations reduce across all 30k threads to a single scalar. GPU reductions are standard: tree reduction within a workgroup using shared memory, partial-sum scatter across workgroups, final reduction on a small buffer. The wgpu-specific detail is that WGSL does not provide hardware reduction intrinsics across workgroups, so the final reduction runs as a second dispatch on a small buffer — typically 32–64 partial sums, fits in a single workgroup. Two dispatches per reduction (tree within + final across) is the committed pattern.

The only alternative worth naming: running the reduction synchronously on CPU, reading the partial-sum buffer back per iteration. Rejected — the per-iteration readback cost (≈30 μs per submit+readback) dominates the ≈5 μs the GPU reduction takes, and stacking hundreds of CG iterations per Newton step turns a ≈3 ms-per-step inner loop into a ≈6 ms one. The partial-sum buffer stays on GPU; only the final scalar crosses back, and only at Newton-step boundaries ([§02 async](02-async.md)).

## What this sub-leaf commits the book to

- **Four workload shapes, each named.** `sim-soft`'s GPU dispatch layer labels every kernel with one of per-element, per-contact-pair, per-DOF, per-tet-state. The label determines the `workgroup_size`, the scatter pattern, and the bind-group layout. Uniform dispatch across shapes is not on the menu.
- **Per-element is the default, the rest are exceptions with stated reasons.** The claim from the parent's Claim 2 is defended here with per-shape access-pattern and thread-count arguments; shape mismatches produce the "only 3× faster" outcome the book is avoiding.
- **Assembly is per-element-plus-atomic-scatter.** The scatter uses BSR-block atomic adds; the global-matrix writes are the only place inter-thread coordination lives in the per-element family, and the atomic payload is a 9-float block.
- **Per-DOF reductions are two-dispatch tree reductions.** CG inner products and residual norms use the standard GPU reduction pattern; the final scalar crosses to CPU only at Newton-step boundaries, not per-CG-iteration.
