# sim-ml-chassis GPU extension

[Part 6 Ch 00 Claim 2](../60-differentiability/00-what-autograd-needs.md) commits: *extend `sim-ml-chassis` with a GPU tensor backend and a VJP registration API, do not import Burn*. This chapter names the concrete shape of that extension — what the chassis looks like before, what it looks like after, and what the three sub-chapters specify.

| Section | What it covers |
|---|---|
| [Current chassis CPU autograd](04-chassis-extension/00-current-cpu.md) | The existing `Tape`, `Tensor<f64>`, `VjpRegistry`, and their ≈3k LOC implementation on nalgebra + ndarray; what works, what the GPU extension needs to preserve |
| [Proposed GPU backend](04-chassis-extension/01-gpu-backend.md) | `GpuTensor<f64>` + `GpuTape` parallel to the CPU types, sharing the `VjpRegistry` for cross-backend registration. wgpu as the execution substrate; sim-soft as the primary consumer, but the backend is sim-ml-chassis-level |
| [VJP registration at the chassis level](04-chassis-extension/02-vjp-api.md) | The unified API: `VjpRegistry::register(KernelId, VjpId, Backend)`. A forward op's VJP is registered per-backend; the chassis picks the registered VJP matching the forward's backend at dispatch time |

Three claims.

**Extension, not replacement.** The existing CPU `Tape` and `Tensor<f64>` types are not deprecated, renamed, or reshaped. They continue to work for CPU-resident workloads ([sim-rl](../110-crate/03-build-order.md) baselines, small sensor post-processing in existing sim-soft consumers, tests). The GPU backend is a new module — `sim_ml_chassis::gpu` — that parallels the CPU surface with `GpuTensor<f64>` and `GpuTape`. Interop at the tape level (a computation graph with both CPU and GPU entries) is supported; the cross-backend dispatch inserts readback points automatically where required. The existing chassis's unit tests continue to pass unchanged; new GPU-specific tests are added alongside.

**Backend-polymorphic VJP registration.** A single `VjpRegistry` holds registrations keyed on `(KernelId, Backend)`. Forward ops tag themselves with a backend at dispatch time; the registry lookup picks the matching VJP (CPU or GPU). This is how a chassis-level API supports two execution backends without duplicating the tape-entry type. From the op author's perspective, adding a new GPU op means (a) writing the WGSL forward kernel, (b) writing the WGSL VJP kernel, (c) registering both in the chassis init. The CPU-op experience is unchanged; the GPU-op experience is the same shape with a WGSL compilation step.

**sim-soft is the primary consumer, but not the only one.** The GPU extension lives in `sim-ml-chassis`, not in `sim-soft`, because the chassis is the shared autograd substrate across `sim-rl`, `sim-opt`, and `sim-soft`. If a future `sim-rl` algorithm wants GPU batch training (SAC, TD3 at scale), it gets the GPU backend for free. The commitment is that GPU-autograd infrastructure is a chassis-level capability, not a sim-soft-local hack, and the sub-chapters specify the API at a level where any crate building on the chassis can consume it.

## What this commits downstream

- [Part 6 Ch 00 (what autograd needs)](../60-differentiability/00-what-autograd-needs.md)'s "Extend sim-ml-chassis with GPU backend and VJP registration" is the capability this chapter specifies concretely.
- [Part 6 Ch 01 (custom VJPs)](../60-differentiability/01-custom-vjps.md) is generalized from CPU-only to backend-polymorphic via the `VjpRegistry` extension.
- [Ch 02 sparse solvers](02-sparse-solvers.md) and [Ch 03 GPU autograd tape](03-gpu-autograd.md) ship as part of `sim_ml_chassis::gpu`; sim-soft consumes them without importing wgpu directly for the common case.
- [Phase E build-order](../110-crate/03-build-order.md#the-committed-order) lists the chassis-GPU extension as a blocking prerequisite; the chassis ships the extension first, then sim-soft ports Newton and CG onto it.
