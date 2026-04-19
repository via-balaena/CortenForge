# Async scheduling

GPU compute in wgpu is fundamentally asynchronous: `encoder.finish()` produces a command buffer, `queue.submit([...])` hands it to the driver, and the GPU runs it concurrently with CPU work. The only place the CPU blocks on the GPU is readback — `Buffer::map_async` plus an explicit `device.poll(wgpu::Maintain::Wait)` (or the async-fn equivalent that drives the driver's event queue). Scheduling a Newton step with minimum idle time means being explicit about where readback lands, because every readback point is a submit-boundary that serializes CPU and GPU work across it. The [Ch 00 parent's Claim 3](../00-wgpu-layout.md) names the convergence check as the canonical readback point; this leaf names the full set and justifies the boundary count.

## The Newton-step submit timeline

A single Newton iteration on GPU looks like this, start to finish:

```text
[CPU] encode: assembly kernel (per-element gradient + Hessian scatter)
[CPU] encode: preconditioner apply (per-DOF or scene-scale — see Ch 02 §03)
[CPU] encode: CG inner loop, N_cg iterations; each iteration is
              SpMV + two AXPYs + two dot-product reductions
              (each reduction is 2 dispatches per §00, so ~7 dispatches/iter)
[CPU] encode: line-search direction kernel
[CPU] encode: readback staging — copy residual-norm scalar into mapped buffer
[CPU] queue.submit([encoder.finish()])    ← ONE submit per Newton iteration
[GPU] runs the whole command buffer asynchronously
[CPU] await residual-norm readback (map_async + poll)
[CPU] decide: converged? line-search accept? shrink dt? → next iter or step done
```

One submit per Newton iteration, not per kernel. This is load-bearing — the CPU encodes the entire step's command buffer without any `submit` call between kernels, so the GPU sees one contiguous stream of work and the CPU driver overhead is ≈30–100 μs of encoding spread across the step, not per kernel. At 80–700 kernels × ≈30 μs per submit, the per-kernel-submit alternative would cost ≈2–20 ms of CPU driver time per Newton iteration — multiples of the actual compute wall-clock on a canonical scene.

The single submit rule breaks *only* at readback points, because readback semantics require a submit boundary before the mapped buffer becomes readable. The residual-norm readback at the end of each iteration is the canonical (and typically only) readback per Newton iteration.

## The readback cost model

`Buffer::map_async` is the wgpu API for CPU-side buffer reads. It takes a callback and returns when the driver's copy queue has completed the staging-to-CPU transfer. Cost breakdown on typical mid-tier hardware:

- **Submit latency**: ≈20–40 μs — the time from `queue.submit` to the driver beginning GPU execution. Dominated by driver-side command-buffer translation.
- **GPU execution**: varies — ≈100 μs to 5 ms per Newton iteration depending on scene size and CG iteration count.
- **Map-async latency**: ≈30–80 μs — the time from GPU completion to the mapped buffer being CPU-readable. Dominated by the staging-to-mapped copy plus the roundtrip through the driver's event queue.

The total per-iteration roundtrip is ≈50–120 μs of CPU-GPU pipelining cost on top of the compute itself. On a canonical Phase E scene (≈30k DOFs, ≈10 Newton iterations per step), that is ≈10 × 100 μs ≈ 1 ms of synchronization per timestep — ≈2% of a 60 FPS frame budget. Per-CG-iteration readbacks are explicitly avoided, so the synchronization cost depends only on Newton count, not on the preconditioner-dependent CG iteration count. The parent's Claim 3 "≤2% of wall-time on readback" comes from this arithmetic.

Attempting to eliminate the readback entirely — running the convergence check as a GPU kernel that atomically sets a device-side flag, and dispatching the next iteration conditionally — is possible in principle but requires a mechanism wgpu does not yet expose in stable form (conditional dispatch or work-graph primitives). The Phase E commitment is CPU-side control flow; the Phase-H-or-later upgrade path revisits this if wgpu ships the primitives by then.

## What crosses the boundary: scalar, not vector

The readback payload is a single `f64` scalar — the residual norm $\|g^{(k)}\|$ — packed into an 8-byte mapped buffer. Vector-valued readbacks (e.g., returning the full gradient to CPU) are explicitly avoided inside the Newton loop because an $n$-vector readback scales with $n$ and dominates the synchronization cost at non-trivial mesh sizes.

At end-of-timestep (after Newton has converged), a vector-valued readback may happen: the converged position $x_n$ is copied back to CPU if the caller (e.g., the rendering layer, the experiment harness, [Part 6 Ch 02's IFT adjoint](../../60-differentiability/02-implicit-function.md)) needs it. That readback is one-per-timestep, not one-per-iteration, and the cost is amortized across the ≈10 Newton iterations plus the step's physics work. On Phase E's canonical scene, end-of-step readback is ≈200–500 μs for the 30k-DOF position buffer — a one-time cost per timestep and not the per-iteration hot path.

The IFT backward pass has its own readback pattern: one scalar (the convergence check inside the backward CG solve) per upstream-adjoint, and one vector (the gradient w.r.t. theta) at the end. [Part 6 Ch 02](../../60-differentiability/02-implicit-function.md)'s cost model already folds this in — the "one factor (or preconditioner), many RHSes" pattern means the backward pass has the same submit-per-RHS shape as forward Newton, with identical per-roundtrip costs.

## Integration with the adaptive-dt failure path

[Part 5 Ch 02's adaptive-$\Delta t$](../../50-time-integration/02-adaptive-dt.md) halves the timestep when Newton fails to converge or when IPC's CCD clipping forces a step shorter than the elastic tangent can absorb. The failure decision is CPU-side: after the final Newton-iteration readback, if the residual norm is still above tolerance *and* the iteration count cap has been hit, the CPU rolls back the integration state and retries with $\Delta t / 2$.

This is why the convergence check has to cross to CPU — the failure response is a control-flow decision the solver wants to keep on the CPU, where the rollback-and-retry logic is cleanly expressible as a fallible `step()` call in Rust. A GPU-resident convergence check could emit a "converged" flag, but the "didn't converge, retry with smaller dt" path would require re-dispatching an entire timestep's worth of kernels with a new parameter — either from a conditional-dispatch primitive that wgpu does not expose, or from CPU-side re-submission with the new $\Delta t$. The second is what `sim-soft` does today, at the committed cost of the scalar readback.

The honest trade from the parent's Claim 3 is restated here: ≤2% wall-time on readback, keep control flow on CPU, inherit Part 5 Ch 02's rollback path without a GPU-side rewrite of the state machine.

## Command-buffer batching across CG iterations

A single Newton iteration contains anywhere from ≈10 CG inner iterations (AMG-preconditioned) to ≈100 (Jacobi-preconditioned) on the canonical scene, per [Ch 02 Claim 5](../02-sparse-solvers.md). Each CG iteration is ≈7 dispatches (SpMV + two AXPYs + two dot products, each dot product expanding to a 2-dispatch tree reduction per [§00](00-kernel-types.md)), plus ≈3–5 assembly kernels at the start and a line-search kernel at the end. That is ≈80 kernels per Newton iteration on the AMG path — consistent with the "≈50 kernels per Newton" figure the [Ch 00 parent](../00-wgpu-layout.md) and [Ch 03](../03-gpu-autograd.md) spine quote to one significant figure — and ≈700+ on the Jacobi path. Either way, the whole iteration encodes into one command buffer.

Three implementation details that matter at this scale:

**Command-encoder pooling.** Constructing a fresh `wgpu::CommandEncoder` per Newton iteration is cheap (≈5 μs) but not free. `sim-soft`'s GPU context keeps a pool of ≈3 encoders rotating across Newton iterations — one in use, one being consumed by the driver, one free. The pool size is set by the typical driver-side queue depth on mid-tier hardware; Phase E benchmarking will re-tune this.

**Pipeline-state caching.** Each distinct GPU kernel has a `wgpu::ComputePipeline` state that includes the compiled WGSL, the bind-group layout handle, and the entry-point name. `sim-soft`'s Phase E kernel count is O(30) distinct WGSL entry points (per-shape-per-op), and each one's pipeline costs ≈200–500 μs to build the first time, negligible thereafter if cached. `sim-soft` builds and caches pipelines at chassis init; the per-dispatch kernel lookup is a handle lookup, not a rebuild. The cache key is `(KernelId, BindGroupLayoutId)` from [§01](01-bind-groups.md)'s layout registry.

**Query-set instrumentation is off by default.** wgpu supports timestamp queries (`wgpu::QuerySet` with `QueryType::Timestamp`) for per-pass GPU-side timing. They cost ≈0.5–1 μs per pass and produce per-iteration timing buffers useful for benchmarking but not needed at runtime. `sim-soft` gates timestamp queries behind a `gpu_profile` feature flag — on for Phase E bring-up, off for production.

## What this sub-leaf commits the book to

- **One submit per Newton iteration, not per kernel.** The entire iteration's command buffer is encoded on CPU, submitted once, and awaited via a single residual-norm readback. Per-kernel submits are explicitly avoided; the multi-millisecond CPU driver overhead is unaffordable at 80–700 kernels per iteration.
- **Scalar residual-norm readback is the only per-iteration GPU↔CPU roundtrip.** ≈50–120 μs per iteration, ≈10 iterations per timestep, ≈2% of wall-time at Phase E frame budgets. Vector readbacks land at timestep boundaries only.
- **Adaptive-$\Delta t$ rollback stays on CPU.** The convergence-failure response from [Part 5 Ch 02](../../50-time-integration/02-adaptive-dt.md) is CPU-side control flow; the GPU-resident alternative would require wgpu conditional-dispatch primitives that are not in stable. Phase H revisits; Phase E commits to CPU control flow.
- **Command-encoder pool + pipeline cache + gated timestamp queries** are the three implementation practices that keep encoding overhead bounded at 50–700-kernel-per-iteration densities. Named at Pass 1; Phase E implementation tunes concrete constants.
