# Reverse-mode playback

[Ch 03 parent Claim 3](../03-gpu-autograd.md) commits the backward pass to "one compute pass per tape entry, in reverse." This leaf specifies the traversal, the VJP-kernel-dispatch mechanics, how gradient accumulation works on GPU (atomic adds with `atomic<u32>` + bit-reinterpret, matching [Ch 00 §01](../00-wgpu-layout/01-bind-groups.md)), the loss-seed-to-parameter-gradient flow, and the grad-of-grad scope boundary the parent's discussion of Claim 3 names.

## The backward traversal

```rust
pub fn backward(tape: &GpuTape, loss_seed: GpuTensor<f32>) -> GradientMap {
    let mut grads = GradientMap::new();
    grads.insert(tape.output_root().unwrap(), loss_seed);    // seed ∂L/∂L = 1

    for entry in tape.entries().iter().rev() {
        let vjp      = VjpRegistry::lookup(entry.vjp);
        let bind_grp = build_backward_bind_group(entry, &grads, &tape.tensor_pool);
        let mut enc  = device.create_command_encoder(&Default::default());
        {
            let mut pass = enc.begin_compute_pass(&Default::default());
            pass.set_pipeline(&vjp.pipeline);
            pass.set_bind_group(0, &bind_grp, &[]);
            pass.dispatch_workgroups(entry.dispatch_size.0,
                                     entry.dispatch_size.1,
                                     entry.dispatch_size.2);
        }
        queue.submit([enc.finish()]);
    }
    grads
}
```

Five mechanical details this hides from the API surface:

**Dispatch size mirrors forward.** The backward VJP kernel runs at the same workgroup count as the forward kernel — the `dispatch_size` captured in [§00 recording](00-recording.md)'s `GpuTapeEntry`. This is load-bearing for the IPC active-pair case where forward dispatch count is data-dependent; replaying at the wrong count would either skip pairs or touch invalid memory.

**The VJP is looked up by `VjpKernelHandle`, not derived from the forward kernel.** `sim-soft` does not do source-to-source differentiation on WGSL ([Ch 03 parent Claim 2](../03-gpu-autograd.md)); every forward kernel ships with a hand-written VJP kernel registered alongside it. Dispatch is a handle lookup in the `VjpRegistry` — O(1), no runtime compilation.

**Bind groups include both the forward's inputs/outputs *and* the relevant gradient buffers.** `build_backward_bind_group` assembles a bind group containing:
- The forward kernel's input tensors (read-only, via their `TensorId`).
- The forward kernel's output tensors (read-only — the VJP may need forward outputs, e.g., $\nabla_F$ of neo-Hookean stress needs $F$ which is an input, and $\nabla$ of SVD's $\sigma$ needs the output singular values).
- The gradient buffer for each output tensor (read-only — the incoming grad).
- The gradient buffer for each input tensor (read-write, atomic — the VJP writes the push-back).

The bind-group layout here is a per-VJP variant of the [Ch 00 §01 layouts](../00-wgpu-layout/01-bind-groups.md); the `VjpRegistry` stores the layout handle alongside the pipeline.

**Submit boundaries per entry are a submit-performance constraint, not a correctness one.** The submit between tape entries is there because the next entry's bind group depends on the previous entry's output gradient buffer — read-after-write requires a fence. Batching across independent subtrees (two parallel forward branches with separate loss-root contributions) is possible and costed in the "parallel subtree" extension below; the common-case reverse-mode backward through sequential Newton residual evaluation has no parallel subtrees and runs one submit per entry.

**The `GradientMap` is a `HashMap<TensorId, GpuTensor<f32>>` keyed on input tensors.** Downstream consumers (the optimizer, the IFT harness) read specific tensors out of the map; they don't iterate the whole map. At Phase E scale, the map has ≈10–30 entries — one per design parameter and one per trained material property. The read-out cost is a single GPU buffer read per queried parameter.

## Grad accumulation: atomic adds when a tensor has multiple consumers

A forward tensor consumed by $k$ downstream kernels receives $k$ independent gradient contributions on backward, each written by a different VJP kernel. These contributions must sum atomically into the tensor's gradient buffer:

```wgsl
// inside a VJP kernel
let contrib = compute_vjp_contribution(...);    // f32 push-back from this kernel
atomic_add_f32(&grad_buffer[index], contrib);
```

Since WGSL doesn't provide `atomic<f32>` natively, the `atomic_add_f32` helper uses the [Ch 00 §01 bit-reinterpret compare-and-swap pattern](../00-wgpu-layout/01-bind-groups.md) on an `atomic<u32>`-declared buffer:

```wgsl
fn atomic_add_f32(buf: ptr<storage, atomic<u32>, read_write>, value: f32) {
    var old_bits = atomicLoad(buf);
    loop {
        let old_val = bitcast<f32>(old_bits);
        let new_val = old_val + value;
        let new_bits = bitcast<u32>(new_val);
        let ex = atomicCompareExchangeWeak(buf, old_bits, new_bits);
        if (ex.exchanged) { break; }
        old_bits = ex.old_value;
    }
}
```

Contention-and-cost profile:

- **Non-contended writes** (tensor consumed by exactly one downstream — the common case in Newton-inner-loop intermediate tensors): first iteration of the CAS loop succeeds, cost is ≈3× a native `f32` add on mid-tier hardware. No retries.
- **Contended writes** (tensor consumed by many downstream kernels on forward — per-vertex gradient tensors that receive push-back from many per-tet VJPs on backward, or widely-shared intermediate tensors like the global residual): retry rate scales with concurrent-writer count. For `sim-soft`'s per-vertex gradient scatter the fan-in is bounded by the mesh's vertex-valence (≈8–15 tets per vertex), so retry rate is ≈2–5%; cost rises to ≈3.5–4× native. Still better than the alternatives (histogram-then-reduce over the mesh, ≈2–3× higher end-to-end per [Ch 00 §00 assembly](../00-wgpu-layout/00-kernel-types.md)).

The `atomic_add_f64` emulation is structurally similar but uses two `atomic<u32>` slots and either a double-word compare-exchange (where supported) or a double-single Kahan-style split (where it is not). Phase E will benchmark both on target hardware; Pass 1 defers the specific choice to the implementation.

## Seed flow: from loss to design parameters

The backward pass starts at the loss scalar — a one-element `GpuTensor<f32>` at the root of the tape, seeded with $\partial L / \partial L = 1$. The traversal propagates backward through the tape, and the `GradientMap` accumulates contributions at every tensor visited. At the end, the map contains $\partial L / \partial x$ for every design parameter $x$ the caller registered at the start of recording.

```rust
pub fn parameter_gradient<T: Parameter>(
    tape:  &GpuTape,
    param: &T,
) -> Result<GpuTensor<f32>, GradientError> {
    let seed = GpuTensor::scalar_seed(tape.device(), 1.0);
    let grads = backward(tape, seed);
    grads.get(&param.tensor_id())
         .cloned()
         .ok_or(GradientError::ParameterNotTapeRoot)
}
```

The IFT harness from [Part 6 Ch 02](../../60-differentiability/02-implicit-function.md) wraps this call with the "one preconditioner, many RHSes" pattern: each downstream adjoint is a distinct `loss_seed`, and the preconditioner recorded on the tape (from [Ch 02 §03](../02-sparse-solvers/03-preconditioning.md)) is re-used across the solves. No per-adjoint re-build.

## Grad-of-grad is not supported

Second-order derivatives — the gradient of the backward pass itself — would require recording the backward pass as its own tape and re-differentiating through it. `sim-soft` Pass 1 does not do this:

- The forward-pass tape records kernel *calls*, not the arithmetic inside VJP kernels. Re-differentiating through a VJP kernel would require either (a) a VJP-of-VJP kernel shipped alongside, or (b) source-to-source differentiation through the WGSL. Neither is on the Phase E roadmap.
- The canonical optimizer path — [Part 10's forward path](../../100-optimization/00-forward.md) — needs only first-order gradients. L-BFGS-style quasi-Newton consumers approximate the Hessian from first-order gradient history rather than asking for an explicit Hessian; Adam-style consumers use only moment estimates of gradients.
- True second-order methods (trust-region with exact Hessian, natural-gradient descent on the design manifold) are not on `sim-soft`'s Phase-E critical path; if a future use case demands them, the grad-of-grad extension below is what it wires into.

If a future use case demands it, the extension is well-scoped: ship a VJP-of-VJP kernel for every existing op, and add a `GpuTape::record_backward` entry point that emits a second tape during the first tape's replay. Pass 1 does not ship this; the scope boundary is named.

## Parallel-subtree batching (called out, not shipped in Pass 1)

The one-submit-per-entry traversal above is sequential. Some forward graphs have parallel subtrees — an optimizer evaluating design samples in parallel via [sim-opt](../../110-crate/03-build-order.md), or a multi-head loss with independent contributions from vision vs. contact. Both their backward passes could batch: dispatch multiple tape-entries' VJP kernels into one command buffer before submit, pipelined across non-dependent subtrees.

Pass 1 does not implement subtree batching; the common-case canonical-design-loop backward pass has no parallel subtrees to batch across. Future Phase-F use cases that drive batching benefit (sim-opt's batched design evaluation, for instance) will extend the traversal; the extension point is in `backward()`'s tape-iteration logic and doesn't touch any other part of the machinery.

## What this sub-leaf commits the book to

- **Backward traversal is tape-reversed, one submit per entry.** Each entry's VJP kernel dispatches at the forward-recorded workgroup count, using a bind group that includes forward inputs/outputs and input/output gradient buffers.
- **VJP lookup is by registered handle** in the `VjpRegistry`; no source-to-source differentiation, no runtime compilation, ≈O(1) dispatch cost per entry beyond the kernel itself.
- **Gradient accumulation is atomic-add via `atomic<u32>` + bit-reinterpret CAS loops.** Non-contended write is ≈3× native f32-add; contended at mesh valence (≈8–15) is ≈3.5–4× native with ≈2–5% retry rate.
- **Loss-to-parameter-gradient via `GradientMap`.** Caller queries by `TensorId`; IFT harness wraps the call with multi-RHS preconditioner reuse from [Ch 02 §03](../02-sparse-solvers/03-preconditioning.md).
- **Grad-of-grad is not supported in Pass 1.** The extension to VJP-of-VJP kernels is scoped and named for future use cases that require second-order information; the canonical design loop does not need it.
- **Parallel-subtree batching is a future extension.** Pass 1's sequential backward handles the canonical single-loss single-design-branch case. Extensions land at the `backward()` iteration boundary without touching the tape or registry.
