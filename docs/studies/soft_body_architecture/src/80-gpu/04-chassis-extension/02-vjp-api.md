# VJP registration at the chassis level

[Ch 04 parent Claim 2](../04-chassis-extension.md) commits the chassis to *backend-polymorphic VJP registration* — one conceptual API that supports both CPU per-tape-instance closure registration ([Part 6 Ch 01 §00](../../60-differentiability/01-custom-vjps/00-registration.md)) and GPU global pre-compiled-kernel registration ([Ch 03 §00](../03-gpu-autograd/00-recording.md)). The two models are genuinely different under the hood — CPU uses Rust closures with captured state, GPU uses WGSL compute pipelines registered at chassis init — and the unification is conceptual-not-literal: both flow into a shared `BackwardOp::Custom` (CPU) or `GpuTapeEntry::vjp` (GPU) dispatch that the chassis's backward traversal reads from. This leaf specifies the two flows, how they compose, and what the "one API" commitment actually delivers from a user's perspective.

## Two distinct registration models, one consumption pattern

The underlying mechanism is different per backend:

**CPU: per-tape-instance, closure-owning, State-carrying.** Per [Part 6 Ch 01 §00 (registration API)](../../60-differentiability/01-custom-vjps/00-registration.md):

```rust
// User implements VjpOp (chassis trait)
pub trait VjpOp: Send + Sync {
    fn op_id(&self) -> &'static str;  // stable identity, futureproofs serialization
    fn parents(&self) -> &[u32];
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]);
}

// User struct holds per-invocation state as fields
struct NewtonStepVjp {
    parents: Vec<u32>,
    factor:  faer::sparse::Llt<f64>,     // held by the struct, released with the tape
    jacobian_pattern: JacobianPattern,
}

impl VjpOp for NewtonStepVjp {
    fn op_id(&self) -> &'static str { "newton_step" }
    fn parents(&self) -> &[u32] { &self.parents }
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        // back-substitute through self.factor into parent_cotans slots
    }
}

// User calls Tape::push_custom at forward time
let factor = faer::sparse::Llt::new(&A)?;              // build per-forward-pass state
let output: Var = tape.push_custom(
    newton_step_output,                                 // primal tensor output
    Box::new(NewtonStepVjp {
        parents: vec![x_prev.id(), theta.id()],
        factor,
        jacobian_pattern,
    }),
);
// `output` feeds downstream tape ops; tape.backward() calls VjpOp::vjp later
```

Per-invocation state lives as struct fields on the `VjpOp` impl (the `faer::Llt<f64>` factor above), held alive by the `Box<dyn VjpOp>` in `BackwardOp::Custom`, dropped when the tape drops and releases the box. The same user type can be re-instantiated with different state at each forward call.

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

**Audit rule: every forward op has a registered VJP.** CPU: if a user defines a `VjpOp` and calls `push_custom`, the backward is guaranteed to fire (by construction). GPU: the chassis VJP-coverage CI test asserts that for every registered `KernelId` in the PipelineCache, there is a matching `VjpKernelHandle` registered in the VjpRegistry. No forward op ships without a backward.

## What's different — and why

The differences are not accidents; they reflect the cost structure of the two backends:

**WGSL doesn't support arbitrary Rust state.** The CPU path's `VjpOp` impl can hold arbitrary state as struct fields (a faer factorization, a Rust `HashMap`, a boxed trait object). WGSL compute kernels cannot — their only inputs are bind-group buffers and push-constants. So GPU VJPs cannot carry Rust state; they must receive everything they need through bindings. The registry is correspondingly global (one kernel pair per registered forward op) rather than per-instance.

**WGSL doesn't have runtime compilation in wgpu.** The CPU path can dynamically create new `VjpOp` implementations at runtime (a user crate implements the trait, the chassis boxes and dispatches it). GPU WGSL kernels must be compiled at chassis init — wgpu does not support runtime WGSL compilation the way some shader frameworks do. So the GPU's "registration" really means "ship a WGSL source file with the crate" — not runtime-user-extensible in the same way as CPU.

**Forward and backward kernel storage converges differently.** CPU's `BackwardOp::Custom` node stores one `Box<dyn VjpOp>` — the impl carries its own parents slice and per-invocation state as struct fields. GPU's `GpuTapeEntry` stores: kernel handles (KernelId, VjpKernelHandle), tensor ids (Vec<TensorId>), dispatch size (u32×3), captured scalars (SmallVec<[f64; 3]>). The two representations handle kernel invocation state differently because the invocation itself is different — Rust function call vs. wgpu dispatch.

## The "one API" user experience

From a user's perspective writing a new physics op, the workflow is:

**CPU-side author** (typical for Phase D):
1. Implement `VjpOp` trait on a user type `MyPhysicsOp` whose fields carry per-invocation state.
2. Construct the struct at forward time with per-forward state populated.
3. Call `tape.push_custom(forward_output, Box::new(MyPhysicsOp { ... }))`.
4. The boxed `VjpOp::vjp` fires when downstream gradients reach the op.

**GPU-side author** (typical for Phase E):
1. Write the forward WGSL kernel (e.g., `neo_hookean_stress_fwd`).
2. Write the corresponding VJP WGSL kernel (`neo_hookean_stress_vjp`).
3. Add both to the chassis's `wgsl/` source tree.
4. Add the `(KernelId, VjpKernelHandle)` pair to the chassis init's `register_builtin_vjps()` function.
5. At dispatch, calling `dispatch_kernel(KernelId::NeoHookeanStressFwd, ...)` automatically appends the matching tape entry; nothing else is needed at the call site.

The two workflows feel different because they are — CPU op authoring is dynamic, GPU op authoring is source-tree-level — but both terminate in "the forward-pass call site knows the input tensors and the output tensor; the chassis handles the rest of the backward path." That's what "one API" means here: one consumption pattern (the forward call site), two authoring patterns (closure for CPU, WGSL kernel pair for GPU).

## Cross-backend registration exists, but is phase-deferred

A CPU `VjpOp` backing a GPU forward kernel is technically possible — the kernel's VJP runs on CPU, receiving a readback of the GPU forward's outputs and pushing gradients back via a CPU→GPU upload. This is useful for kernels where the forward is worth accelerating but the backward is small-operand-count enough that a CPU impl would be acceptable (some IPC active-pair updates, some sparse-indexing ops).

Pass 1 does not specify this cross-direction. The existing surface supports:
- CPU forward → CPU backward (via `Tape::push_custom`)
- GPU forward → GPU backward (via `VjpRegistry::register` at chassis init)

What's out of scope for Pass 1:
- GPU forward → CPU backward (readback-per-VJP pattern)
- CPU forward → GPU backward (upload-per-VJP pattern)

Both are future extensions if specific kernels motivate them. Pass 1 scopes; Phase F implementation may revisit.

## CI enforcement

`sim-ml-chassis::gpu` ships a `vjp_coverage_test` that iterates every `KernelId` in `KernelId::ALL` and asserts that `VjpRegistry::lookup` returns a registered `VjpKernelHandle` for it. A PR that adds a forward kernel without its VJP fails this test and is blocked.

A complementary test on the CPU side (not part of this sub-leaf but specified in [Part 6 Ch 01 §00](../../60-differentiability/01-custom-vjps/00-registration.md)) asserts that every `VjpOp` impl in `sim-soft` has a corresponding gradcheck regression in [Part 11 Ch 04](../../110-crate/04-testing/03-gradcheck.md).

These two CI tests together form the "every forward has a registered and regression-tested backward" invariant the book relies on for Phase E gradient correctness.

## The `VjpOp` author contract — determinism across calls

Both CPU `VjpOp` impls and GPU VJP kernels must satisfy the `ForwardMap` determinism-in-θ contract from [Part 10 Ch 00](../../100-optimization/00-forward.md) within A.4 §4's 5-digit gradcheck tolerance band (see [Part 11 Ch 04 §03 gradcheck](../../110-crate/04-testing/03-gradcheck.md)) — the chassis cannot enforce this at the type level, so the discipline is trait-contract-and-gradcheck, not runtime-checked. The chassis `VjpOp` trait docstring is the canonical statement of the contract; what follows is the architecture-book-reader's view citing the same source.

**Forbidden modes, both backends:**
- Stateful RNG without an explicitly-passed seed (an RNG seeded deterministically by `theta` is fine).
- Wall-clock reads, env-var reads, file-content reads, mutable global state.
- HashMap iteration order as a semantically-meaningful input (use `BTreeMap` or sorted iteration).

**CPU-specific** (`Box<dyn VjpOp>` impls):
- Cross-call state: state held in the impl's struct fields that persists across forward-pass invocations and affects output. Per-invocation state (factor on tape, cached Jacobian pattern) is *expected* — that's the whole point of impl-fields-over-closures. Cross-call state is what breaks purity.
- Captured `Rc<RefCell<...>>` or similar shared-mutable state across multiple `vjp` invocations in the same backward pass.

**GPU-specific** (registered WGSL VJP kernels):
- Non-deterministic subgroup operations whose shuffle lanes depend on physical lane assignment (varies per GPU vendor and driver version).
- Readback of vendor-state or driver-internal state via extension intrinsics.
- Uninitialized workgroup-shared-memory reads.
- Non-IEEE-deterministic intrinsics beyond the A.4 §4 tolerance band (transcendentals at ≤2 ULP are inside the band; operations not covered by the band are not admitted).

The book-mandated [gradcheck discipline](../../110-crate/04-testing/03-gradcheck.md) catches high-magnitude failures; lower-magnitude drift within the 5-digit band is accepted per A.4 §4's parallel-path tolerance clause and does not require runtime assertion. See [§110 Ch 02 ml-chassis coupling](../../110-crate/02-coupling/03-ml-chassis.md) for the determinism-in-θ contract that grounds this.

## What this sub-leaf commits the book to

- **Two registration mechanisms, one consumption pattern.** CPU uses `Tape::push_custom` with `Box<dyn VjpOp>` (per-invocation state held as impl fields); GPU uses `VjpRegistry::register` at chassis init with pre-compiled WGSL kernel pairs. Both produce tape entries the backward traversal reads.
- **The two mechanisms unify at "one backward per tape entry" and "atomic-style accumulation into input slots," not at a shared registration function.** Users picking backend see different authoring patterns; the consuming solver/optimizer code sees the same tape-backward semantics.
- **GPU registration is append-only at chassis init, not runtime-user-extensible.** WGSL doesn't support runtime compilation in wgpu; new GPU ops land via source-tree additions and a chassis-init registration call, not dynamic trait impls.
- **Cross-direction registration (CPU-fwd/GPU-bwd or GPU-fwd/CPU-bwd) is scoped to future phases.** Pass 1 supports within-backend registration only.
- **CI enforces VJP coverage on both sides.** GPU: every `KernelId` must have a registered `VjpKernelHandle`. CPU: every `VjpOp` impl must have a [gradcheck regression](../../110-crate/04-testing/03-gradcheck.md). No forward ships without a backward that passes regression.
