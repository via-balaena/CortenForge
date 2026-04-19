# Current chassis CPU autograd

The [Ch 04 parent's Claim 1](../04-chassis-extension.md) commits the Phase E GPU extension to *extension, not replacement* — the existing `sim-ml-chassis` CPU types keep working unchanged while a new `sim_ml_chassis::gpu` module lands alongside. This leaf describes what the existing CPU autograd actually looks like today, so the GPU-extension shape of [§01](01-gpu-backend.md) is grounded in the real API surface rather than an idealized one, and [§02](02-vjp-api.md)'s backend-polymorphic VJP API is a defensible widening rather than a redesign.

## The tape is f64-scalar, tape-per-forward-pass, discarded after backward

`sim-ml-chassis::autograd` implements a minimal reverse-mode tape:

```rust
pub struct Tape {
    nodes: Vec<Node>,                  // topologically sorted by construction
}

pub struct Var(u32);                   // handle into Tape::nodes, Copy, 4 bytes

struct Node {
    value:    f64,                      // forward value at this node
    backward: BackwardOp,               // how gradients propagate to parents
}

enum BackwardOp {
    Leaf,                              // parameter or constant
    Unary  { parent: u32, local: f64 },
    Binary { lhs: u32, rhs: u32, dl: f64, dr: f64 },
    Tanh   { parent: u32, out: f64 },  // local = 1 - out² (computed during backward)
}
```

Four design properties that matter for the GPU extension:

**Scalar-only `f64` values.** Every `Node::value` is one `f64`. A tensor (a policy network's weight matrix, a rollout's reward) is represented as many `Node`s, one per scalar element. This sounds inefficient versus a tensor-valued tape but pays off via simplicity: the backward rule per op is a scalar formula, not a matrix-calculus dispatch, and the whole module audits in under an hour. The scalar-per-node choice is load-bearing for the ≈3k-LOC audit surface that [Part 1 Ch 03's thesis](../../10-physical/03-thesis.md) protects.

**Topologically sorted by construction.** Every `Node` only references parents already pushed to the tape. No sort pass, no cycle-detection — `backward()` is a single reverse-iterator sweep that unconditionally propagates $\bar y_\text{parent} \mathrel{+}= \text{local} \cdot \bar y_\text{self}$ via scalar add. Simpler than PyTorch's dynamic-DAG / JAX's `jit`-traced staged graph, which is the point.

**Tape-per-forward-pass.** A `Tape` is constructed at the start of a forward pass, populated via method calls on `&mut Tape`, consumed by `tape.backward()`, dropped. No amortization across forward passes, no tape reuse, no pooling. Every episode's rollout builds a fresh tape.

**Twelve public operations.** Nine scalar primitives (`add`, `sub`, `mul`, `neg`, `tanh`, `relu`, `square`, `ln`, `exp`) plus three fused (`sum`, `mean`, `affine`, the last being matrix-vector-plus-bias for policy MLP forward passes). The `push` method that creates a `Node` is private — users cannot extend the tape without going through one of the twelve public ops, and `sim-ml-chassis`'s CI rejects any patch that adds a new public op without the corresponding backward rule tested.

## `Tensor` and `Var` are different types

An important subtlety: the chassis exposes both a `Tensor` struct (a flat `f32` buffer with shape metadata, no autograd) and the `Var`/`Tape` autograd types. They are separate types for separate purposes:

- **`Tensor`** (defined in the chassis's `tensor` module) is the observation / action / buffer type — what flows across the `Environment` trait boundary, what a `Policy` consumes and emits. It has no autograd machinery. Shape metadata is `Vec<usize>`.
- **`Tape` and `Var`** are the autograd machinery — what a `DifferentiablePolicy`'s backward pass uses, what a loss's gradient computation owns. Every `Var` corresponds to one `f64` scalar in the tape.

The two types compose via policy layers ([chassis `autograd` module](../../110-crate/00-module-layout/07-autograd.md)): a policy's forward receives `Tensor` observations, converts their `f32` entries to `f64` on entry to the tape, runs tape ops, reads out the `f64` scalars into an `f32` action tensor on exit. The precision change is intentional — observations and actions live at the framework-interop boundary where `f32` matches convention, while autograd wants the higher precision for gradient stability.

For the GPU extension, this separation survives: [§01 GPU backend](01-gpu-backend.md) ships a `GpuTensor<f32>` (mirror of CPU `Tensor`) and `GpuTape` (mirror of CPU `Tape`), keeping the two-type split.

## The `CustomVjp` extension from Part 6 Ch 01

[Part 6 Ch 01 §00 registration](../../60-differentiability/01-custom-vjps/00-registration.md) commits to three additions on the CPU tape: a `BackwardOp::Custom` variant, a `CustomVjp` trait, and a `Tape::register_custom_vjp` method. These land before Phase E's GPU extension because the physics kernels ([FEM assembly](../../60-differentiability/01-custom-vjps/01-fem-assembly.md), [IPC barrier](../../60-differentiability/01-custom-vjps/02-contact-barrier.md), [IFT backward-Euler wrapper](../../60-differentiability/02-implicit-function.md), [time-adjoint wrapper](../../60-differentiability/03-time-adjoint.md)) need them on the CPU path at Phase D:

```rust
pub trait CustomVjp {
    type State;
    fn forward (&self, state: &Self::State, inputs: &[f64]) -> Vec<f64>;
    fn backward(&self, state: &Self::State, inputs: &[f64], outputs: &[f64],
                grad_outputs: &[f64]) -> Vec<f64>;
}

impl Tape {
    pub fn register_custom_vjp<V: CustomVjp>(
        &mut self, vjp: V, state: V::State, inputs: &[Var]
    ) -> Vec<Var>;
}
```

The key property for Phase E: this API is per-tape-instance (a `CustomVjp` instance owns its `State`, gets registered into one specific `Tape` at forward time, fires its backward closure on that tape's `backward()`). The GPU analog in [Ch 03 §00 recording](../03-gpu-autograd/00-recording.md) is different — it registers pre-compiled `wgpu::ComputePipeline` handles into a *global* `VjpRegistry` at chassis init, and the tape entries reference those handles. Two different models, unified at the conceptual level by [§02](02-vjp-api.md).

## Dependencies: nalgebra only, no ndarray

The chassis `Cargo.toml` depends on:

- `nalgebra` (for `DVector`, `DMatrix` used in policy forward passes and MuJoCo state interop)
- `rand` (for Gaussian noise in stochastic policies, Xavier init)
- `serde` + `serde_json` (for `PolicyArtifact` persistence)
- `sim-core` + `sim-mjcf` (for task fixtures)
- `thiserror` (for error enums)
- `bevy_ecs` (gated behind the `bevy` feature)

Not in the dependency graph: `ndarray`, `tch` (PyTorch bindings), `candle`, `burn`, `dfdx`. The tape is implemented using plain `Vec<Node>` and `Vec<f64>` for gradients; no tensor-framework imports. This is what the "own every line" thesis from [Part 6 Ch 00 Claim 3](../../60-differentiability/00-what-autograd-needs.md) is defending — and what the Phase E GPU extension must preserve.

## What the GPU extension needs to preserve

Four properties that the [§01 GPU backend](01-gpu-backend.md) spec carries forward:

**Transparent backward rules.** Every op's gradient is a short closed-form formula the reader can inspect. The GPU side's hand-written WGSL VJPs ([Ch 03 §00 recording](../03-gpu-autograd/00-recording.md)) satisfy this — each forward kernel has a sibling VJP kernel whose closed-form gradient is ≈20 lines of WGSL.

**No framework dependency.** The CPU autograd is ≈3k LOC of self-contained code; the GPU autograd similarly lives inside `sim-ml-chassis::gpu` as self-contained `wgpu` code, not as a wrapper over Burn or candle. Adding a framework dependency would violate Claim 3's audit-surface constraint.

**Per-tape lifetime ownership.** A `Tape` is built, used, dropped — the backward closures and their captured state live for the tape's lifetime, not longer. The GPU `GpuTape` follows the same pattern: a `GpuTape` owns its `GpuTensorPool`, and both drop together.

**Thread-safety at the tape level, not the node level.** A `Tape` is `!Sync` — one forward pass runs on one thread, one backward pass runs on one thread, multiple rollouts run their own tapes in parallel via `rayon` at the rollout level. The GPU `GpuTape` follows the same pattern: one tape per GPU device, concurrent tapes run as parallel compute-queues, not as parallel writers to the same tape.

## What this sub-leaf commits the book to

- **Scalar-f64 tape with 12 public ops, plus the `CustomVjp` extension from Part 6 Ch 01.** The CPU autograd implementation is ≈3k LOC on nalgebra (no ndarray); every op's backward rule is a closed-form scalar formula.
- **`Tensor` and `Var`/`Tape` are different types.** `Tensor` is the framework-interop data type with no autograd; `Var`/`Tape` is the autograd machinery for `DifferentiablePolicy`. The GPU extension preserves this split as `GpuTensor<f32>` + `GpuTape`.
- **Tape-per-forward-pass, dropped after backward.** No amortization, no pooling. Simple lifetime that composes with `rayon` parallel rollouts at the rollout level, not the node level.
- **No framework dependency.** `sim-ml-chassis` imports nalgebra, rand, serde, thiserror, sim-core, sim-mjcf, optionally bevy_ecs — nothing tensor-framework-shaped. The GPU extension keeps the same discipline (wgpu yes; Burn/candle/dfdx no).
- **The GPU extension preserves four properties**: transparent closed-form backward rules, no framework dependency, per-tape lifetime ownership, thread-safety at the tape level.
