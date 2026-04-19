# Proposed GPU backend

[Ch 04 parent Claim 2](../04-chassis-extension.md) commits the Phase E GPU surface to live at `sim-ml-chassis::gpu` — a new module inside the existing chassis crate, parallel to the CPU types, sharing the crate's overall trait boundaries. This leaf specifies the module's surface: the types it exposes, how it composes with the CPU surface when a workflow spans both devices, and the registries it owns that [Ch 00 §01](../00-wgpu-layout/01-bind-groups.md) and [Ch 03 §00](../03-gpu-autograd/00-recording.md) reference by name.

## The module surface

`sim_ml_chassis::gpu` exports ten public types and three chassis-init registries:

```rust
// Device and tensor lifecycle
pub struct GpuDevice { /* wraps wgpu::Device + wgpu::Queue */ }
pub struct GpuTensor<T: GpuScalar> { /* wraps wgpu::Buffer + shape + device handle */ }
pub struct GpuTensorPool { /* per Ch 03 §00 — shape-keyed free list */ }

// Autograd machinery (Ch 03)
pub struct GpuTape { /* wraps Vec<GpuTapeEntry> + GpuTensorPool */ }
pub struct GpuTapeEntry { /* per Ch 03 §00 — handles not tensors */ }
pub struct GradientMap { /* per Ch 03 §01 — HashMap<TensorId, GpuTensor<f32>> */ }

// Global-at-chassis-init registries
pub struct BindGroupLayoutRegistry { /* four canonical layouts from Ch 00 §01 */ }
pub struct PipelineCache { /* WGSL → wgpu::ComputePipeline, built at init */ }
pub struct VjpRegistry { /* forward-kernel-id → VJP-kernel-id, per Ch 03 §00 */ }

// Solver surface (Ch 02)
pub trait SpMvOp { /* per Ch 01 §02 — BSR or matrix-free implementations */ }
pub struct CgSolver { /* per Ch 02 §00 */ }
pub struct MinresSolver { /* per Ch 02 §01 */ }
pub trait Preconditioner { /* per Ch 02 §03 */ }
```

Three design choices that structure this surface:

**GPU types parallel CPU types.** `GpuTensor<f32>` is the Phase-E-default parallel of CPU `Tensor` (the CPU side is f32-hardcoded per [§00](00-current-cpu.md), the GPU side is generic over a `GpuScalar` bound so future f16/f64 paths fit the same type); `GpuTape` parallels CPU `Tape`; the `Preconditioner` trait mirrors what sim-soft's CPU solver already has. Every public GPU type has a CPU counterpart with a similar name and overlapping methods. A consumer that writes `let tape: Tape = ...` on CPU writes `let tape: GpuTape = ...` on GPU; the API shape transfers.

**Generic-parameter signature expansion is avoided.** The chassis does NOT introduce a `Tape<Device>` generic where `Device` is CPU or GPU. The CPU `Tape` and GPU `GpuTape` are different types with different internal representations, related by a convention rather than a type parameter. This is a deliberate choice — the audit surface stays readable (one module's worth of GPU code is all that changes) and the type-inference failures that generic backend parameters produce on complex call sites are avoided. Rust's trait resolution handles `dyn SpMvOp` / `dyn Preconditioner` cleanly for per-op backend polymorphism, which is where backend-polymorphism actually lives in the code — not at the tape boundary.

**Wgpu is the substrate, not a wrapper.** `GpuDevice` is a thin wrapper around `wgpu::Device` + `wgpu::Queue` — not an abstraction layer that could swap in CUDA or Metal-native backends. Phase E commits to wgpu, [Ch 00 parent Claim 1](../00-wgpu-layout.md) names the reasons, and the chassis surface reflects that commitment. If a future phase ships a vendor-native path, it ships as a new module (`sim_ml_chassis::cuda`) with its own types, not as a backend-swap on the existing GpuTensor.

## The three global registries

Three chassis-init-constructed, shared-read singletons:

**BindGroupLayoutRegistry** — the four canonical layouts from [Ch 00 §01](../00-wgpu-layout/01-bind-groups.md). Per-element, per-contact-pair, per-DOF, per-tet-state. Each layout is a `wgpu::BindGroupLayout` built at chassis init. Kernels reference the registry by the enum `BindGroupLayoutId::{PerElement, PerContactPair, PerDof, PerTetState}` and pick up the matching layout handle at dispatch time. Adding a new layout type is an audit-level change — a new variant + new init code + a justification for why existing layouts don't fit.

**PipelineCache** — the compiled WGSL kernels from [Ch 00 §02](../00-wgpu-layout/02-async.md). Built at chassis init, one `wgpu::ComputePipeline` per distinct WGSL entry point. Lookup at dispatch is an O(1) handle-to-pipeline map — no runtime compilation, no cache miss. Phase E's kernel count is O(30) distinct entry points (per-shape-per-op); all are built up front.

**VjpRegistry** — the forward-kernel-id to VJP-kernel-id map from [Ch 03 §00](../03-gpu-autograd/00-recording.md). For every registered forward kernel (`KernelId`) there is a registered VJP (`VjpKernelHandle`) that the backward traversal dispatches. CI enforces pairing: a forward kernel without a registered VJP fails the `sim-ml-chassis::gpu` VJP-coverage test, blocking the PR.

All three registries are owned by the `GpuDevice` and borrowed by every `GpuTape` the device produces (via a `&'device` lifetime tied to the device). Constructed once at `GpuDevice::new()`, shared read-only across every tape's lifetime, no per-tape construction cost. The registries are where the GPU surface's "one-time cost, many-times benefit" pattern lives.

## Cross-backend interop

Some workflows run part of the forward pass on CPU and part on GPU. The primary example is the `cf-design` change-detection step: [Part 7 Ch 04](../../70-sdf-pipeline/04-live-remesh.md) runs SDF-hash-change detection on CPU (fast, serial, touching a few scalars), hands off to `sim-soft::gpu` for the FEM solve, then receives the converged state back on CPU for the authoring-layer render. The tape has to handle both sides.

The chassis supports this through a *cross-backend tape* where the entries mix CPU and GPU records:

```rust
pub enum AnyTapeEntry {
    Cpu(CpuTapeEntry),            // references a BackwardOp::Custom, CPU closure, CPU State
    Gpu(GpuTapeEntry),            // references a VjpKernelHandle, GPU bind group, GpuTensorPool
    CpuToGpu { cpu: CpuTensorRef, gpu: GpuTensorRef, /* readback at backward */ },
    GpuToCpu { gpu: GpuTensorRef, cpu: CpuTensorRef, /* readback at forward */ },
}
```

The backward traversal runs entries in reverse; cross-backend entries insert readback points where required (`GpuToCpu` during forward becomes `CpuToGpu` during backward to propagate the gradient, and vice versa). The Phase E common case has very few cross-backend hops — the inner Newton loop is GPU-only, the outer design-loop control flow is CPU-only, and the hop happens once per design edit. Readback cost (≈100–500 μs per vector hop per [Ch 00 §02](../00-wgpu-layout/02-async.md)) is amortized across the ≈5–15 Newton iterations of the step.

Pass 1 names the cross-backend tape as a Phase-E-implementation-detail rather than specifying the exact layout; the type-sketch above is the intended direction, and Phase E benchmarks will validate the readback-insertion policy. What Pass 1 *commits* is that the chassis supports the interop, not that this specific enum is the shipping shape.

## Phase E build-order interaction

[Part 11 Ch 03's build order](../../110-crate/03-build-order.md) requires the chassis GPU extension to ship before `sim-soft`'s `gpu/` module consumes it. Concrete build sequence:

1. `sim-ml-chassis::gpu` module lands first with `GpuDevice`, `GpuTensor<f32>`, `GpuTape`, the three registries, the four bind-group layouts, and a minimal set of WGSL kernels (`zero`, `copy`, `axpy`, `dot`, `elementwise_mul`) for baseline testing.
2. `sim-rl` adopts the GPU backend for policy network forward passes (no physics, just MLPs) — this is the interoperability test: one chassis, two consumers (sim-rl and sim-soft), the GPU tape must work for both.
3. `sim-soft::gpu` consumes the chassis's surface and registers its physics-specific WGSL kernels (FEM assembly, IPC barrier, neo-Hookean tangent, CG inner loop, preconditioner) into the shared `PipelineCache` and `VjpRegistry`.
4. `sim-opt` may adopt the GPU backend for batched design evaluation ([Ch 03 §01 parallel subtree](../03-gpu-autograd/01-playback.md) is what this would drive); Pass 1 scopes this to Phase F.

The chassis ships step 1 complete; step 2 is the cross-consumer interop test; step 3 is where `sim-soft` gains its GPU forward solve and backward IFT. Each step lands as its own PR with regression tests against the Phase D CPU path.

## Where GPU and CPU differ

Two real capability gaps that Phase E ships with, and one scope boundary they share:

**No f64 native arithmetic on GPU (CPU has it).** GPU-side computation is f32 unless the driver provides an f64 extension. Mixed precision (f32 SpMV + f64 accumulators via compensated summation) per [Ch 02 §00](../02-sparse-solvers/00-cg.md) recovers accuracy for the CG inner products, but the forward-pass arithmetic in per-element kernels runs at f32. This is a real precision difference — the f32 forward may produce results that differ from CPU f64 by ≈1e-6 in the third-to-fourth digit. The gradcheck suite ([Part 11 Ch 04](../../110-crate/04-testing/03-gradcheck.md)) uses 5-digit tolerance on forward, 5-digit on backward gradient agreement, which the mixed-precision path meets.

**No per-tape closure registration on GPU (CPU has it).** CPU's `register_custom_vjp` holds an owned `State` for the duration of the tape's lifetime — a user can embed arbitrary Rust state in the closure. GPU's equivalent is the global `VjpRegistry` that registers pre-compiled WGSL kernels — no per-tape state, no runtime closure construction. Chassis state that a kernel needs (scalars, buffer handles) is passed via the bind group, not captured in a closure. This is [Ch 03 §00](../03-gpu-autograd/00-recording.md)'s recording pattern.

**Neither backend ships grad-of-grad in Pass 1.** Both CPU and GPU support first-order gradients only per [Ch 03 §01](../03-gpu-autograd/01-playback.md) / [Part 6 Ch 01 §00](../../60-differentiability/01-custom-vjps/00-registration.md); the canonical optimizer path doesn't need second-order info. The extension cost is asymmetric when it becomes needed — CPU extends via VJP-of-VJP closure registration (incremental), GPU requires shipping a VJP-of-VJP WGSL kernel alongside every existing VJP kernel (duplicative) — so the CPU path would likely pick up grad-of-grad first if a future use case motivates it.

## What this sub-leaf commits the book to

- **`sim_ml_chassis::gpu` is a new module, not a generic backend parameter on existing types.** GpuTensor / GpuTape / GradientMap parallel their CPU counterparts by naming and convention, not via `Tape<Device>` generics. Backend polymorphism lives at the `SpMvOp` / `Preconditioner` trait level, not at the tape type.
- **Three global chassis-init registries**: BindGroupLayoutRegistry, PipelineCache, VjpRegistry. All built at `GpuDevice::new()`, shared across every `GpuTape` on that device.
- **Cross-backend tapes are supported.** Mixed CPU/GPU computation graphs work through cross-backend entries that insert readback points at the device boundary. Exact enum layout is Phase E implementation detail; Pass 1 commits that the interop works, not the specific shape.
- **Phase E build sequence: chassis GPU first, then sim-rl adoption, then sim-soft consumption, then (Phase F) sim-opt batched evaluation.** Each step lands as a regression-tested PR against Phase D CPU.
- **GPU lacks two CPU features in Pass 1**: native f64 arithmetic (mixed-precision workaround for the inner loop) and per-tape-state closure registration (global-kernel-registry instead). Grad-of-grad is absent on both CPU and GPU in Pass 1; the CPU extension path is cheaper if it becomes needed.
