# Recording kernel calls

The [Ch 03 parent's Claim 1](../03-gpu-autograd.md) commits the GPU tape to record *kernel handles*, not tensor copies — the ≈50-kernel-per-Newton memory cost of a copy-based tape would blow the Phase E VRAM budget before the solve finished. Claim 4 commits the tape to the *converged* Newton iterate via a checkpoint-and-replay pattern, not the full Newton path. This leaf specifies the recording mechanics: what goes in each `GpuTapeEntry`, how the tensor pool manages intermediate lifetime, and how the checkpoint-replay boundary composes with [Part 5 Ch 00's Newton loop](../../50-time-integration/00-backward-euler.md).

## The `GpuTapeEntry` record

Each forward kernel dispatch that participates in autograd pushes one entry:

```rust
pub struct GpuTapeEntry {
    pub kernel:       KernelHandle,                // forward kernel id
    pub vjp:          VjpKernelHandle,             // backward kernel id
    pub inputs:       SmallVec<[TensorId; 4]>,     // input tensors (max 4 typical)
    pub outputs:      SmallVec<[TensorId; 2]>,     // output tensors
    pub bindings:     BindGroupLayoutHandle,       // which of the four layouts from Ch 00 §01
    pub scalars:      SmallVec<[f64; 3]>,          // captured scalar params (dt, k_barrier, etc.)
    pub dispatch_size: (u32, u32, u32),            // workgroup count at forward dispatch
}

pub struct GpuTape {
    entries:      Vec<GpuTapeEntry>,
    tensor_pool:  GpuTensorPool,
    output_root:  Option<TensorId>,                // the loss scalar's tensor id
}
```

Five design properties the layout enforces:

**Tensor references are by id, not by buffer pointer.** `TensorId` is a 32-bit handle into the `GpuTensorPool`; the pool owns the `wgpu::Buffer` allocations. The tape never holds a `wgpu::Buffer` directly, so dropping a tape entry does not leak or double-free GPU memory — pool reference-counting handles that. Recording cost per entry is ≈100 ns on CPU (SmallVec allocations, no GPU sync), negligible versus the ≈5–50 μs the forward kernel dispatch takes.

**Captured scalars are bit-exact.** Scalar parameters not bound through `TensorId` (timestep $\Delta t$, the IPC barrier radius $\hat d$, the preconditioner tolerance) are copied into the tape as `f64` values. The backward VJP reads the same scalar the forward did — no reconstruction, no risk of a scalar drifting between forward and backward due to interleaved Newton iterations.

**Dispatch size is recorded at forward time.** The backward VJP dispatches at the same workgroup count as forward. For per-element or per-tet kernels this is deterministic (== `n_tets`), but for per-contact-pair kernels the count depends on the active-pair list size at forward time ([Ch 00 §00 per-contact-pair shape](../00-wgpu-layout/00-kernel-types.md)), which may differ between Newton iterations. Recording the count pins what the backward kernel operates on.

**Bindings reference the shared layout registry.** `BindGroupLayoutHandle` is a lookup into [Ch 00 §01](../00-wgpu-layout/01-bind-groups.md)'s shared layout table — per-element / per-contact-pair / per-DOF / per-tet-state. The tape does not duplicate the layout specification; it references it by handle.

**Output tensor count is bounded.** Most ops have a single output; a few (SVD, eigendecomposition) have multiple. `SmallVec<[TensorId; 2]>` is inline-stored for up to 2 outputs, heap-spilled past that. The common case pays zero heap allocations per tape entry.

## Tensor lifetime — the pool and the reference-count pattern

Intermediate tensors live in a `GpuTensorPool` that ref-counts based on tape entries:

```rust
pub struct GpuTensorPool {
    tensors:      SlotMap<TensorId, PooledTensor>,
    free_buffers: HashMap<TensorShape, Vec<wgpu::Buffer>>,    // shape-keyed free list
}

struct PooledTensor {
    buffer:  Arc<wgpu::Buffer>,
    shape:   TensorShape,
    refs:    u32,                                              // live tape entry references
}
```

When the tape is recording a forward kernel, each input tensor's refcount is incremented if it is also held by a live tape entry; when a tape entry is retired (either at end-of-backward or because its output tensor has no consumer), its input refcounts decrement. Tensors with refcount zero are returned to `free_buffers` for re-use by the next allocation that matches the shape.

The common case — intermediate tensors in the Newton iteration are consumed by exactly one successor kernel and dropped after that successor runs — keeps the resident set at ≈3× steady-state on the canonical scene rather than ≈50× naive recording. Concrete: Phase E's Newton-loop intermediate tensors for the 30k-DOF scene sum to ≈5 MB resident at any one time versus ≈200+ MB if every intermediate were kept alive for the whole solve.

The pool implementation borrows from PyTorch's caching allocator — specifically the shape-keyed free list that avoids re-allocation on the steady-state hot path — but the tensor-lifecycle semantics are simpler because `sim-soft` does not need PyTorch's broadcast-and-strides machinery. Every `sim-soft` tensor is contiguous, its shape is known at kernel-registration time, and the backward-pass reference graph is a topologically-sorted DAG with no dynamic consumers.

## Acquire-use contract

The pool exposes two acquire methods: one safe-by-default, one opt-out for kernels that write every element they read.

```rust
impl GpuTensorPool {
    pub fn acquire<T: GpuScalar>(&mut self, shape: TensorShape) -> GpuTensor<T>;         // zeroed, default
    pub fn acquire_uninit<T: GpuScalar>(&mut self, shape: TensorShape) -> GpuTensor<T>;  // undefined content, write-before-read contract
}
```

**`acquire` is the default path and zeros the buffer.** Kernels that scatter-add, accumulate partial contributions, or read slots they don't write (IPC barrier active-pair gradient accumulation, per-vertex residual assembly, CG inner-product reductions via atomic-add) use this path. Output buffer contents are bit-identical (all-zeros) regardless of pool history; determinism-preserving by construction. Cost is one zero-fill pass over the buffer at acquire time, paid on every reuse from the free list.

**`acquire_uninit` is the opt-out for write-every-element kernels.** Kernels whose output indexing covers every slot in the buffer with a pure write — `copy` (`y = x`), fresh-output elementwise transforms (`output[i] = f(a[i], b[i])`), `zero` initialization itself — can use `acquire_uninit` to skip the zero-fill. Write-before-read is the kernel-author contract: every slot the kernel reads must first have been written by the kernel (or by a predecessor in the same command-buffer submit). Accumulator-style kernels that read-modify-write (in-place `axpy` `y += αx`, BSR SpMV output with all-zero rows, any atomic-add reduction) are NOT safe for `acquire_uninit` and use the default `acquire` path instead. Violations produce non-deterministic content leakage from the previous call's buffer tenant — a gradcheck-catchable silent-determinism hazard when the stale content is numerically non-zero. `acquire_uninit` call sites are grep-able in code review; that is the audit surface for the write-before-read contract.

The kernel-author contract for `acquire_uninit` lives alongside [Ch 04 §02's `VjpOp` author contract](../04-chassis-extension/02-vjp-api.md); cross-call determinism contracts unify at one location.

## Dynamic-size tensors — the contact-pair case

The IPC active-pair list ([Part 4 Ch 01](../../40-contact/01-ipc-internals/01-adaptive-kappa.md)) has a size that changes across Newton iterations — a new contact region activating can double the pair count within one step. The tape has to handle this without re-allocating the bind-group layout per iteration:

- The layout declares `array<ContactPair>` without a pinned size; wgpu's `var<storage, read>` with an unsized runtime-length array satisfies this.
- The bind group's buffer-binding size is set from the pair-list length each time the tape records the contact-pair kernel; the `BindGroup` itself is re-created (not the layout).
- The backward VJP kernel receives the same-size buffer binding on replay; the tape records the size alongside the kernel dispatch.

The pattern is a general one — "layout is stable, binding size is per-dispatch" — that extends to any kernel whose operand dimensions can change between iterations. Recording cost for a dynamic binding is one extra `BindGroup::new` per dispatch (≈2 μs driver time), cheap relative to the forward kernel wall-clock.

## The checkpoint-and-replay pattern

[Ch 03 parent Claim 4](../03-gpu-autograd.md) commits the forward Newton loop to "no-tape" mode for the inner iterations, then re-plays the converged iterate once to produce a clean tape. The pattern in code:

```rust
pub fn newton_step_with_checkpoint<'a>(
    solver: &mut NewtonSolver,
    tape:   &mut GpuTape,
    design_params: &GpuTensor<f32>,
) -> (GpuTensor<f32>, NewtonCheckpoint) {
    // Phase A: run Newton with tape recording disabled
    solver.set_tape_mode(TapeMode::Off);
    let x_converged = solver.solve(design_params);      // no tape entries pushed
    let checkpoint = solver.capture_state();             // save mesh, material, x_converged

    // Phase B: one-shot replay to produce a tape of just the equilibrium eval
    solver.set_tape_mode(TapeMode::Recording(tape));
    let _ = solver.evaluate_equilibrium(&checkpoint);    // pushes ≈50 entries to tape
    solver.set_tape_mode(TapeMode::Off);

    (x_converged, checkpoint)
}
```

Two properties that matter:

**The tape records the equilibrium condition, not the path.** [Part 6 Ch 02's IFT](../../60-differentiability/02-implicit-function.md) differentiates at $\nabla U_n(x^\ast) = 0$, not at the Newton iterates that led there. The replay records exactly one evaluation of the equilibrium residual and its Jacobian; the backward pass sees a single linear system to solve, not a Newton sequence to unroll. This is the IFT's whole point.

**Replay is free of CPU↔GPU synchronization.** The replay dispatches run in one command buffer submit per the [Ch 00 §02 async pattern](../00-wgpu-layout/02-async.md). Total replay cost on the canonical scene is ≈1–2 ms wall-clock — negligible relative to the Newton solve's ≈10–200 ms per step — because the replay does a single forward pass with no iterative refinement, no line search, no convergence check.

## Time-adjoint exception

[Part 6 Ch 03's time adjoint](../../60-differentiability/03-time-adjoint.md) differentiates across multiple timesteps, not just within one. For that path, the tape records one checkpoint per timestep (not per Newton iterate) and the [Part 6 Ch 04 checkpointing](../../60-differentiability/04-checkpointing.md) machinery decides which checkpoints to keep versus recompute.

`sim-soft`'s tape supports time-adjoint recording by chaining `GpuTape` instances — one per timestep — and linking them via cross-tape `TensorId`s. The chaining is a Phase-F-or-later implementation detail; Pass 1 names it and scopes it. The single-timestep tape this sub-leaf specifies is the primitive; the multi-timestep chain builds on top.

## Graph mutation during forward — not supported in Pass 1

Some differentiable frameworks (JAX's `jit` with dynamic shape, PyTorch's dynamic graphs) support forward passes that add or remove nodes based on data values. `sim-soft`'s Pass 1 tape does not — the forward pass's structure is a fixed sequence of kernel dispatches, and control-flow decisions (Newton accept/reject, line-search backtrack) happen *outside* the tape on CPU. The CPU-side control flow picks which sequence to record, and the tape records that sequence without branching.

A future extension could support branching by adding a `GpuTapeEntry::Branch { condition: TensorId, then_branch: Vec<GpuTapeEntry>, else_branch: Vec<GpuTapeEntry> }` variant, but this would require the backward pass to evaluate the same branch as forward — and carrying the boolean along the tape adds complexity Pass 1 does not need for the canonical differentiable-design workflow. Scoped out.

## What this sub-leaf commits the book to

- **`GpuTapeEntry` records handles + captured scalars + dispatch size, not tensor contents.** ≈100 ns CPU recording cost per entry; zero GPU synchronization during recording.
- **The `GpuTensorPool` ref-counts intermediate tensors and reuses buffers.** Steady-state resident set is ≈3× naive recording, matching the "≈3× steady-state not ≈50×" figure from [Ch 03 parent Claim 1](../03-gpu-autograd.md).
- **Acquire-use contract: `acquire` (zeroed, default-safe) + `acquire_uninit` (write-before-read kernel-author contract).** Zero-fill by default closes cross-call content leakage; write-every-element kernels opt into the uninit path for a zero-fill-pass saving. Contract for `acquire_uninit` lives alongside [Ch 04 §02's `VjpOp` author contract](../04-chassis-extension/02-vjp-api.md).
- **Dynamic-size tensors (contact pairs) are handled via per-dispatch bind-group re-creation.** The layout is stable; the binding size updates per dispatch. ≈2 μs extra driver cost per dynamic binding.
- **Checkpoint-and-replay is the Newton-integration pattern.** Forward Newton runs in tape-off mode; one replay after convergence produces a tape of ≈50 entries representing the equilibrium evaluation (not the Newton path). Replay cost is ≈1–2 ms, negligible versus the Newton solve itself.
- **Time-adjoint recording chains per-timestep tapes; Pass 1 scopes the chain as Phase-F implementation detail.** The primitive this sub-leaf specifies is the single-timestep tape.
- **Graph mutation during forward is not supported in Pass 1.** Control flow lives on CPU; the tape records a fixed sequence.
