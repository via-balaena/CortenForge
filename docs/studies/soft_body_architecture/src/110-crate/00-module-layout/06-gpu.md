# gpu/

The `gpu/` module ports `sim-soft`'s hot paths — per-element neo-Hookean assembly, IPC barrier evaluation, sparse-matrix-vector product, preconditioned conjugate gradient, stress extraction — to wgpu compute kernels. It is the Phase E deliverable from the [build order](../03-build-order.md#the-committed-order): a GPU path whose correctness is regression-tested against [`solver/`](04-solver.md)'s CPU reference to 5 digits on every PR, hitting the per-frame throughput that makes the Part 1 Ch 03 integrated-solver thesis practical ([Part 8 Ch 00](../../80-gpu/00-wgpu-layout.md) carries the full GPU architecture, including [Part 8 Ch 04](../../80-gpu/04-chassis-extension.md)'s crucial commitment that the GPU autograd tape lives in `sim-ml-chassis`, not here).

## What the module owns

| Sub-area | Responsibility | Where it is specified |
|---|---|---|
| wgpu kernel inventory | Per-element, per-contact, per-DOF, per-tet thread patterns; one kernel per workload shape | [Part 8 Ch 00 §00](../../80-gpu/00-wgpu-layout/00-kernel-types.md) |
| Bind-group design | Scene-static (connectivity, material fields, rest-pose) bound once per frame; per-step (positions, velocities, Hessian workspace) bound per pass | [Part 8 Ch 00 §01](../../80-gpu/00-wgpu-layout/01-bind-groups.md) |
| Sparse layouts | BSR 3×3 for the Hessian, CSR for adjacency, mesh-derived custom formats where the payoff is large | [Part 8 Ch 01](../../80-gpu/01-sparse-matrix.md) |
| GPU sparse solvers | Preconditioned CG, MINRES, preconditioner inventory (block-Jacobi, incomplete Cholesky, AMG opt-in) | [Part 8 Ch 02](../../80-gpu/02-sparse-solvers.md) |
| GPU VJP registration | Solver-specific VJPs registered against the [`sim-ml-chassis` GPU tape](../../80-gpu/04-chassis-extension.md); `sim-soft` supplies the VJPs, not the tape | [Part 8 Ch 04 §02](../../80-gpu/04-chassis-extension/02-vjp-api.md) |
| Async scheduling | Command-buffer batching across a Newton step; one submit per step plus one readback for convergence check | [Part 8 Ch 00 §02](../../80-gpu/00-wgpu-layout/02-async.md) |

## Four claims

**1. wgpu is the only shipped backend.** Vendor-native paths (CUDA on NVIDIA, Metal/MPS on Apple) are not on the Phase A–I roadmap. [Part 8 Ch 00](../../80-gpu/00-wgpu-layout.md) names the ≈20–30% single-vendor performance ceiling as the tolerated cost of cross-vendor reach, and that cost sits well inside the design-tool-class simulator budget once the wall-time targets from [Part 10 Ch 00](../../100-optimization/00-forward.md) are satisfied. A post-Phase-I vendor-native replacement for a specific hot kernel (if a benchmark after Phase E shows it losing >50% against a vendor-optimized implementation) is considered an opt-in targeted replacement, not a second backend. The module is singular in its backend, not plural.

**2. The GPU autograd tape lives in `sim-ml-chassis`, not here.** [Part 6 Ch 00 Claim 3](../../60-differentiability/00-what-autograd-needs.md) and [Part 8 Ch 04](../../80-gpu/04-chassis-extension.md) both commit to this: the tape's recording infrastructure — kernel-call capture, graph construction, reverse-mode playback — is a chassis concern because it serves every crate that uses the chassis's autograd, not just `sim-soft`. `sim-soft::gpu/` supplies its own VJPs (FEM assembly gradient, IPC barrier gradient, contact broadphase pass-through) registered against the chassis's VJP API; it does not own the tape itself. This is the "own every line" commitment applied at the right level of the crate stack — `sim-ml-chassis` owns the generic tape, each consumer crate owns its fused VJPs, and no third-party framework owns either.

**3. Convergence check is the CPU-readback boundary, not an independent kernel.** [Part 8 Ch 00 §02](../../80-gpu/00-wgpu-layout/02-async.md) makes the call: a single `f64` per Newton iteration comes back to the CPU to decide $\|\nabla U_n\| < \text{tol}$, the full Newton-step command buffer batches into one submit, readback cost measures at ≤2% of wall time. The alternative (convergence check as a GPU kernel with GPU-side control flow) complicates the adaptive-$\Delta t$ failure path ([`solver/`](04-solver.md) shrinks $\Delta t$ on line-search failure, and the decision needs CPU-side branch), and the 2% cost is not a budget bottleneck. Control flow on CPU, bulk compute on GPU, readback is narrow and deliberate.

**4. Per-element is the workload shape the bulk of the module is tuned for.** Assembly, constitutive-law evaluation, IPC barrier per-pair, stress extraction — all thread-parallel over elements (tets or contact pairs) with per-thread register pressure (≈30–60 registers for neo-Hookean) well inside modern-GPU occupancy targets. Workloads that don't fit this shape — global reductions for CG inner products, sparse-triangular solves for preconditioning — are handled by specific kernel-shape alternatives ([Part 8 Ch 02](../../80-gpu/02-sparse-solvers.md)'s CG primitives). The module does not try to force every workload into the per-element mould; it names the exceptions explicitly and uses the right kernel pattern for each.

## What the module does not carry

- **No CUDA, no Metal-native, no ROCm.** The cross-vendor commitment is firm through Phase I; vendor-native replacements are post-Phase-I targeted work, not a parallel backend.
- **No tape recording.** The tape belongs to `sim-ml-chassis`'s GPU extension, keyed off [Part 6 Ch 00 Claim 3](../../60-differentiability/00-what-autograd-needs.md)'s "own every line" commitment. `gpu/` records VJPs *against* the tape, not a tape of its own.
- **No non-Newton alternative on GPU.** Projective Dynamics / XPBD on GPU is a `sim-soft` future concern, not a Phase E deliverable. The Phase E kernel inventory is the Newton-on-$U_n$ path exclusively.

## What this commits downstream

- **[`solver/`](04-solver.md)'s Phase E GPU backend is this module.** The same Newton-iteration shape, the same factor-on-tape contract (with a CG preconditioner handle replacing a Cholesky factor), regression-tested against the CPU reference.
- **[Part 8 Ch 04 §01](../../80-gpu/04-chassis-extension/01-gpu-backend.md)** is the matching chassis-side commitment — `sim-ml-chassis` grows a wgpu tensor backend and a VJP registration API in the same phase, sized to what `sim-soft::gpu/` needs.
- **[Part 11 Ch 04 gradcheck sub-chapter](../04-testing/03-gradcheck.md)** gains a CPU-vs-GPU comparison regressed to 5 digits at Phase E landing; the gradcheck against finite-difference is the CPU reference's ground truth, and the GPU path regresses against the CPU.
- **[Part 10 Ch 00 cost-table Phase E budgets](../../100-optimization/00-forward.md)** — ≤50 ms for the `ParameterOnly` hot path on a 30k-tet scene — bind on this module's throughput, not on the CPU path.
