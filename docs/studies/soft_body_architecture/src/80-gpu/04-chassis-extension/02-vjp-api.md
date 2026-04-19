# VJP registration at the chassis level

[Ch 04 parent Claim 2](../04-chassis-extension.md) commits the chassis to *backend-polymorphic VJP registration* — one conceptual API that supports both CPU per-tape-instance closure registration ([Part 6 Ch 01 §00](../../60-differentiability/01-custom-vjps/00-registration.md)) and GPU global pre-compiled-kernel registration ([Ch 03 §00](../03-gpu-autograd/00-recording.md)). The two models are genuinely different under the hood — CPU uses Rust closures with captured state, GPU uses WGSL compute pipelines registered at chassis init — and the unification is conceptual-not-literal: both flow into a shared `BackwardOp::Custom` (CPU) or `GpuTapeEntry::vjp` (GPU) dispatch that the chassis's backward traversal reads from. This leaf specifies the two flows, how they compose, and what the "one API" commitment actually delivers from a user's perspective.

## Two distinct registration models, one consumption pattern

The underlying mechanism is different per backend:

**CPU: per-tape-instance, closure-owning, State-carrying.** Per [Part 6 Ch 01 §00 (registration API)](../../60-differentiability/01-custom-vjps/00-registration.md):

```rust
// User implements CustomVjp
pub trait CustomVjp {
    type State;
    fn forward (&self, state: &Self::State, inputs: &[f64]) -> Vec<f64>;
    fn backward(&self, state: &Self::State, inputs: &[f64], outputs: &[f64],
                grad_outputs: &[f64]) -> Vec<f64>;
}

// User calls Tape::register_custom_vjp at forward time
let factor = faer::sparse::Llt::new(&A)?;              // build per-forward-pass state
let outputs: Vec<Var> = tape.register_custom_vjp(
    NewtonStepVjp,
    NewtonState { factor, jacobian_pattern },           // State owned by this invocation
    inputs,
);
// `outputs` feed downstream tape ops; tape.backward() fires the closure later
```

The `State` is owned by this forward-pass invocation, held alive by the closure in `BackwardOp::Custom`, dropped when the tape drops. The same `CustomVjp` type can be instantiated multiple times with different `State`s in the same tape.

**GPU: chassis-init-global, pre-compiled-kernel, stateless-at-registry.** Per [Ch 03 §00 (recording)](../03-gpu-autograd/00-recording.md):

```rust
// WGSL author ships a forward and a VJP kernel
// (Both kernels are compiled at chassis init into the PipelineCache)

// Chassis-init registration (runs once per GpuDevice)
VjpRegistry::register(
    KernelId::NeoHookeanStressFwd,
    VjpKernelHandle::NeoHookeanStressVjp,
);

// User code at forward dispatch time
let output: GpuTensor<f32> = dispatch_kernel(
    &device, KernelId::NeoHookeanStressFwd, &[F, material_params], /* bindings */
);
// The GpuTape automatically appends a GpuTapeEntry with vjp: VjpKernelHandle::NeoHookeanStressVjp
// GpuTape::backward() later dispatches the VJP kernel via VjpRegistry::lookup
```

The registry is global and append-only — `VjpRegistry::register` runs at chassis init with the pipeline handles from the PipelineCache and cannot be called per-tape. The VJP's "state" is the bind-group contents at dispatch time (forward kernel's inputs/outputs plus gradient buffers), not Rust state held in a closure.

## What's the same across the two

Despite the different underlying mechanisms, four behaviors unify:

**One tape entry per forward kernel that participates in autograd.** CPU emits a `BackwardOp::Custom { parents, closure }`; GPU emits a `GpuTapeEntry { kernel, vjp, bindings, ... }`. Both are pushed to the current tape. Backward traversal iterates in reverse on either.

**One backward dispatch per tape entry.** CPU calls the closure; GPU dispatches the VJP pipeline. Both receive upstream adjoints and emit pushed-back gradients to input positions.

**Gradient accumulation via atomic-style "plus-equal" into input slots.** CPU increments a `&mut [f64]` grad vector; GPU uses the `atomic<u32>` + bit-reinterpret CAS pattern from [Ch 00 §01](../00-wgpu-layout/01-bind-groups.md) on GPU buffers. The semantic is identical — multiple consumers' contributions sum in.

**Audit rule: every forward op has a registered VJP.** CPU: if a user defines a `CustomVjp` and calls `register_custom_vjp`, the backward is guaranteed to fire (by construction). GPU: the chassis VJP-coverage CI test asserts that for every registered `KernelId` in the PipelineCache, there is a matching `VjpKernelHandle` registered in the VjpRegistry. No forward op ships without a backward.

## What's different — and why

The differences are not accidents; they reflect the cost structure of the two backends:

**WGSL doesn't support Rust closures.** The CPU path can capture arbitrary `State` in a closure (a faer factorization, a Rust `HashMap`, a boxed trait object). WGSL compute kernels cannot — their only inputs are bind-group buffers and push-constants. So GPU VJPs cannot capture Rust state; they must receive everything they need through bindings. The registry is correspondingly global (one kernel pair per registered forward op) rather than per-instance.

**WGSL doesn't have runtime compilation in wgpu.** The CPU path can dynamically create new `CustomVjp` implementations at runtime (a user crate implements the trait, the chassis calls it). GPU WGSL kernels must be compiled at chassis init — wgpu does not support runtime WGSL compilation the way some shader frameworks do. So the GPU's "registration" really means "ship a WGSL source file with the crate" — not runtime-user-extensible in the same way as CPU.

**Forward and backward kernel storage converges differently.** CPU's `BackwardOp::Custom` node stores: parents (Vec<u32>), closure (Box<dyn Fn>), State (owned by closure). GPU's `GpuTapeEntry` stores: kernel handles (KernelId, VjpKernelHandle), tensor ids (Vec<TensorId>), dispatch size (u32×3), captured scalars (SmallVec<[f64; 3]>). The two representations handle kernel invocation state differently because the invocation itself is different — Rust function call vs. wgpu dispatch.

## The "one API" user experience

From a user's perspective writing a new physics op, the workflow is:

**CPU-side author** (typical for Phase D):
1. Implement `CustomVjp` trait on a user type `MyPhysicsOp`.
2. Build per-invocation `State` at forward time.
3. Call `tape.register_custom_vjp(MyPhysicsOp, state, &inputs)`.
4. The backward closure fires when downstream gradients reach the op.

**GPU-side author** (typical for Phase E):
1. Write the forward WGSL kernel (e.g., `neo_hookean_stress_fwd`).
2. Write the corresponding VJP WGSL kernel (`neo_hookean_stress_vjp`).
3. Add both to the chassis's `wgsl/` source tree.
4. Add the `(KernelId, VjpKernelHandle)` pair to the chassis init's `register_builtin_vjps()` function.
5. At dispatch, calling `dispatch_kernel(KernelId::NeoHookeanStressFwd, ...)` automatically appends the matching tape entry; nothing else is needed at the call site.

The two workflows feel different because they are — CPU op authoring is dynamic, GPU op authoring is source-tree-level — but both terminate in "the forward-pass call site knows the input tensors and the output tensor; the chassis handles the rest of the backward path." That's what "one API" means here: one consumption pattern (the forward call site), two authoring patterns (closure for CPU, WGSL kernel pair for GPU).

## Cross-backend registration exists, but is phase-deferred

A CPU `CustomVjp` backing a GPU forward kernel is technically possible — the kernel's VJP runs on CPU, receiving a readback of the GPU forward's outputs and pushing gradients back via a CPU→GPU upload. This is useful for kernels where the forward is worth accelerating but the backward is small-operand-count enough that a CPU closure would be acceptable (some IPC active-pair updates, some sparse-indexing ops).

Pass 1 does not specify this cross-direction. The existing surface supports:
- CPU forward → CPU backward (via `Tape::register_custom_vjp`)
- GPU forward → GPU backward (via `VjpRegistry::register` at chassis init)

What's out of scope for Pass 1:
- GPU forward → CPU backward (readback-per-VJP pattern)
- CPU forward → GPU backward (upload-per-VJP pattern)

Both are future extensions if specific kernels motivate them. Pass 1 scopes; Phase F implementation may revisit.

## CI enforcement

`sim-ml-chassis::gpu` ships a `vjp_coverage_test` that iterates every `KernelId` in `KernelId::ALL` and asserts that `VjpRegistry::lookup` returns a registered `VjpKernelHandle` for it. A PR that adds a forward kernel without its VJP fails this test and is blocked.

A complementary test on the CPU side (not part of this sub-leaf but specified in [Part 6 Ch 01 §00](../../60-differentiability/01-custom-vjps/00-registration.md)) asserts that every `CustomVjp` type in `sim-soft` has a corresponding gradcheck regression in [Part 11 Ch 04](../../110-crate/04-testing/03-gradcheck.md).

These two CI tests together form the "every forward has a registered and regression-tested backward" invariant the book relies on for Phase E gradient correctness.

## What this sub-leaf commits the book to

- **Two registration mechanisms, one consumption pattern.** CPU uses `Tape::register_custom_vjp` with per-tape-instance `CustomVjp` + `State`; GPU uses `VjpRegistry::register` at chassis init with pre-compiled WGSL kernel pairs. Both produce tape entries the backward traversal reads.
- **The two mechanisms unify at "one backward per tape entry" and "atomic-style accumulation into input slots," not at a shared registration function.** Users picking backend see different authoring patterns; the consuming solver/optimizer code sees the same tape-backward semantics.
- **GPU registration is append-only at chassis init, not runtime-user-extensible.** WGSL doesn't support runtime compilation in wgpu; new GPU ops land via source-tree additions and a chassis-init registration call, not dynamic trait impls.
- **Cross-direction registration (CPU-fwd/GPU-bwd or GPU-fwd/CPU-bwd) is scoped to future phases.** Pass 1 supports within-backend registration only.
- **CI enforces VJP coverage on both sides.** GPU: every `KernelId` must have a registered `VjpKernelHandle`. CPU: every `CustomVjp` must have a [gradcheck regression](../../110-crate/04-testing/03-gradcheck.md). No forward ships without a backward that passes regression.
