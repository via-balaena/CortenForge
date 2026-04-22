# GPU autograd tape

`sim-ml-chassis`'s CPU autograd is a reverse-mode tape: forward operations push `TapeEntry` records, backward walks the records in reverse and calls each entry's registered VJP. On GPU, the same tape shape works, but the entries record *kernel launches* rather than CPU function calls, and playback has to respect wgpu's asynchronous execution model. This chapter names the design.

| Section | What it covers |
|---|---|
| [Recording kernel calls](03-gpu-autograd/00-recording.md) | Each forward `wgpu::ComputePass` pushes a `TapeEntry` containing the kernel handle, bind-group inputs, and a reference to the VJP kernel for this op. Recording is cheap — no CPU↔GPU synchronization, just handle manipulation |
| [Reverse-mode playback](03-gpu-autograd/01-playback.md) | Backward walks the tape in reverse, dispatching each entry's VJP kernel with appropriate grad-input/grad-output bind groups. Grad accumulation is a per-tensor atomic-add on GPU, matched by a per-tensor `Rc<RefCell<Tensor>>` on CPU |

Four claims.

## 1. Tape records handles, not tensors

A naive GPU autograd tape would store copies of intermediate tensors — the output of each forward kernel, kept alive for the backward pass. For `sim-soft`'s canonical Newton step, that is gigabytes of GPU memory (30k-DOF state × ≈50 kernel invocations per Newton × 5–15 iterations). Infeasible.

Instead, each tape entry stores:

```rust
pub struct GpuTapeEntry {
    pub kernel: KernelHandle,              // forward kernel that produced this entry
    pub vjp: VjpKernelHandle,              // backward kernel (registered alongside forward)
    pub inputs: Vec<TensorId>,             // input tensor handles
    pub outputs: Vec<TensorId>,            // output tensor handles
    pub bindings: BindGroupLayoutHandle,   // layout for backward-pass binding
    pub scalars: Vec<f64>,                 // any captured scalar parameters (dt, k, etc.)
}
```

Tensors themselves live in a separate `GpuTensorPool` indexed by `TensorId`. A tensor is kept alive *if* some live tape entry references it; entries not referenced by any live tensor can be dropped, releasing the tensor memory. The common case — most intermediate tensors in the Newton iteration are consumed by exactly one successor and can be dropped mid-iteration — keeps peak GPU memory bounded at ≈3× steady-state, not ≈50× naive.

The implementation borrows ideas from JAX's linearization-plus-playback model and from PyTorch's gradient-checkpointing, adapted for wgpu's explicit-binding model. The `KernelHandle` / `VjpKernelHandle` split lets the VJP be registered as a separate compiled WGSL kernel rather than being derived from the forward via source transformation — which wgpu does not support (no equivalent of TorchScript).

## 2. VJP kernels are hand-written WGSL, registered alongside forward kernels

[Part 6 Ch 01](../60-differentiability/01-custom-vjps.md) commits to custom VJPs as the core differentiability mechanism — no source-to-source autodiff over simulation code. On GPU that commitment is even more binding: WGSL has no differentiation mode, and runtime differentiation across compute kernels is not supported by any wgpu-native tooling. Every GPU op in `sim-soft` ships with a hand-written VJP kernel in WGSL, registered at crate load time into a global `VjpRegistry`:

```wgsl
// forward: neo-Hookean stress per tet
@compute @workgroup_size(64)
fn neo_hookean_stress_fwd(/* ... */) { /* ... */ }

// backward: grad_F given grad_stress
@compute @workgroup_size(64)
fn neo_hookean_stress_vjp(/* ... */) { /* ... */ }
```

```rust
// registration at chassis initialization
VjpRegistry::register(
    KernelId::NeoHookeanStressFwd,
    VjpId::NeoHookeanStressVjp,
);
```

The registration is part of `sim-soft`'s library init. New physics ops — added viscoelastic relaxation, new material model — come with their VJP kernel as a mandatory co-delivery. CI enforces pairing: a forward kernel without a registered VJP fails the `sim-ml-chassis` VJP-coverage test. The cost is that every new op is 2× the lines of code to add; the payoff is exact gradients through every GPU operation with no implicit assumptions about what is differentiable.

## 3. Backward playback is one compute pass per tape entry, in reverse

The backward pass dispatches kernels via:

```rust
pub fn backward(tape: &GpuTape, loss_seed: GpuTensor<f32>) -> GradientMap {
    let mut grads = GradientMap::new();
    grads.insert(tape.output_root().unwrap(), loss_seed);  // seed ∂L/∂L = 1
    // VJP kernels atomicAdd their contributions into the grad buffers
    // referenced by `grads`; no separate CPU-side accumulation.
    for entry in tape.entries().iter().rev() {
        let vjp = VjpRegistry::lookup(entry.vjp);
        let bind_group = build_backward_bind_group(entry, &grads);
        let mut encoder = gpu_device.create_command_encoder();
        let mut pass = encoder.begin_compute_pass();
        pass.set_pipeline(&vjp.pipeline);
        pass.set_bind_group(0, &bind_group);
        pass.dispatch_workgroups(entry.dispatch_size());
        drop(pass);
        gpu_queue.submit([encoder.finish()]);
    }
    grads
}
```

Three practical details.

**One submit per tape entry, not one per pass.** The submit boundary forces GPU-side synchronization (the next entry's bind group depends on the previous entry's output). Batching multiple entries into one submit is possible if they don't depend on each other's outputs, but for a reverse-mode walk through a sequential Newton iteration they almost always do. The exception is parallel subtrees (e.g., an optimizer evaluating several design samples in parallel) where batching pays.

**Grad accumulation uses atomic adds.** When a tensor has multiple downstream consumers, its gradient is the sum of contributions from each consumer's VJP. On GPU this is a `atomicAdd(&grad_buffer[i], contribution)` call inside the VJP kernel. WGSL supports `atomicAdd` on `i32` and `u32` storage buffers natively; for `f32` and `f64` atomics, the implementation uses compare-and-swap loops for the f64 case (no hardware f64 atomic support is universal).

**Grad-of-grad is not supported.** Second-order derivatives through the GPU tape would require recording the backward pass as its own tape and re-differentiating. `sim-soft` does not need this — gradients are consumed by the optimizer, not by a meta-optimizer. If a future use case demands it (e.g., natural-gradient descent on a manifold of designs), the infrastructure can be extended, but Pass 1 does not plan for it.

## 4. Integration with Newton: tape records per-iteration, not per-timestep

[Part 5 Ch 00's Newton loop](../50-time-integration/00-backward-euler.md) iterates ≈5–15 times per timestep. A naive recording would push tape entries for every iteration, producing a tape with O(Newton-iters × kernels-per-iteration) entries per timestep — wasteful if only the converged iterate matters for backward.

The design: **only the converged Newton iterate is recorded**, via a checkpoint-and-replay pattern. During the forward pass, the Newton loop runs in a "no-tape-recording" mode — kernels dispatch normally, tensors are produced, but no tape entries are pushed. On convergence, the final iterate is checkpointed. If backward is later requested, the checkpoint is re-played with tape recording enabled, producing a tape of ≈O(kernels-per-iteration) entries — one pass through the converged state, recorded cleanly.

This pattern matches [Part 6 Ch 02's IFT](../60-differentiability/02-implicit-function.md): the backward pass differentiates the equilibrium condition, not the path to equilibrium. The Newton iteration is the path; the IFT operates on the converged state. The checkpoint-and-replay pattern is the GPU concrete form of that claim — the tape represents the equilibrium, not the solve.

The exception is time-integrated trajectories, where [Part 6 Ch 03's time adjoint](../60-differentiability/03-time-adjoint.md) differentiates across multiple timesteps. That path records one checkpoint per timestep (not per Newton iterate) and replays them in reverse; [Part 6 Ch 04 — checkpointing sub-chapter](../60-differentiability/04-checkpointing.md) specifies the memory/compute tradeoff. Pass 1 does not deep-dive the time adjoint; it is named and scoped.

## What this commits downstream

- [Part 6 Ch 01 (custom VJPs)](../60-differentiability/01-custom-vjps.md) gains a `VjpKernelHandle` variant for GPU VJPs; the `VjpRegistry` is backend-polymorphic.
- [Part 11 Ch 04 (testing)](../110-crate/04-testing/03-gradcheck.md)'s gradcheck suite adds a "forward kernel has a registered VJP" coverage test.
- [Ch 04 (chassis extension)](04-chassis-extension.md) exposes `GpuTape` as the dual of the CPU `Tape` type.
- [Phase E](../110-crate/03-build-order.md#the-committed-order) ships the tape machinery; Phase E's regression test is gradient agreement between CPU and GPU paths to 5 digits on the canonical scene.

## What this does NOT cover

- **Performance tuning of the tape replay.** Kernel batching, asynchronous submits where safe, memory-pool management: all Phase E implementation work, not architectural commitments the book pins down.
- **Mixed CPU↔GPU graphs.** If part of the forward lives on CPU (e.g., change-detection from [Part 7 Ch 04](../70-sdf-pipeline/04-live-remesh.md)) and part on GPU, the tape entries mix backend-tagged records. The dispatch logic handles cross-backend by inserting readback points where required; the general shape is straightforward but the specifics are Phase E implementation work.
- **Graph mutation during forward.** Dynamically-sized intermediate tensors (e.g., variable-length contact-pair lists from [IPC's active set](../40-contact/01-ipc-internals/01-adaptive-kappa.md)) require the tape to handle size changes between iterations. Supported via a "rebuild bind group layout on size change" path; details in the [recording sub-chapter](03-gpu-autograd/00-recording.md).
