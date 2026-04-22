# Current chassis CPU autograd

The [Ch 04 parent's Claim 1](../04-chassis-extension.md) commits the Phase E GPU extension to *extension, not replacement* — the existing `sim-ml-chassis` CPU types keep working unchanged while a new `sim_ml_chassis::gpu` module lands alongside. This leaf describes what the existing CPU autograd actually looks like today, so the GPU-extension shape of [§01](01-gpu-backend.md) is grounded in the real API surface rather than an idealized one, and [§02](02-vjp-api.md)'s backend-polymorphic VJP API is a defensible widening rather than a redesign.

## The tape is `Tensor<f64>`-valued, tape-per-forward-pass, discarded after backward

`sim-ml-chassis::autograd` implements a minimal reverse-mode tape:

```rust
pub struct Tape {
    nodes: Vec<Node>,                  // topologically sorted by construction
}

pub struct Var(u32);                   // handle into Tape::nodes, Copy, 4 bytes

struct Node {
    value:    Tensor<f64>,             // forward value at this node (shape [] = scalar)
    backward: BackwardOp,              // how gradients propagate to parents
}

enum BackwardOp {
    Leaf,                              // parameter or constant
    Unary  { parent: u32, rule: UnaryRule },    // Neg, Tanh, Relu, Square, Ln, Exp
    Binary { lhs: u32, rhs: u32, rule: BinaryRule }, // Add, Sub, Mul
    Custom(Box<dyn VjpOp>),            // user-registered per Part 6 Ch 01
}
```

Four design properties that matter for the GPU extension:

**`Tensor<f64>` values per node.** Every `Node::value` is a `Tensor<f64>` whose shape can be `[]` (scalar — how RL policy gradients stay) or arbitrary (vector / matrix — how sim-soft represents per-DOF residuals and per-tet stiffness blocks). Built-in primitives are element-wise on shape-matched tensors; no broadcasting or reduction inside primitives. The audit surface stays tight: each built-in op's backward rule is a closed-form element-wise formula; non-element-wise operations (Hessian factorization, SVD) route through `Custom(Box<dyn VjpOp>)` where the impl's per-invocation state carries the complexity. The ≈3k-LOC audit-surface constraint from [Part 1 Ch 03's thesis](../../10-physical/03-thesis.md) holds — tensor-valued nodes are simpler than per-element flattening for tape-heavy sim-soft workloads, not more complex.

**Topologically sorted by construction.** Every `Node` only references parents already pushed to the tape. No sort pass, no cycle-detection — `backward()` is a single reverse-iterator sweep that unconditionally propagates $\bar y_\text{parent} \mathrel{+}= \text{local} \cdot \bar y_\text{self}$ via scalar add. Simpler than PyTorch's dynamic-DAG / JAX's `jit`-traced staged graph, which is the point.

**Tape-per-forward-pass.** A `Tape` is constructed at the start of a forward pass, populated via method calls on `&mut Tape`, consumed by `tape.backward()`, dropped. No amortization across forward passes, no tape reuse, no pooling. Every episode's rollout builds a fresh tape.

**Twelve built-in operations plus `push_custom` for user extension.** Nine element-wise primitives (`add`, `sub`, `mul`, `neg`, `tanh`, `relu`, `square`, `ln`, `exp`) plus three fused (`sum`, `mean`, `affine`, the last being matrix-vector-plus-bias for policy MLP forward passes). User-defined ops land via `Tape::push_custom(value, Box::new(impl_of_VjpOp))` per [§02](02-vjp-api.md). The `push_basic` method that creates a primitive `Node` is private; the built-in ops are the only way to add a primitive entry, and `sim-ml-chassis`'s CI rejects any patch that adds a new built-in without the corresponding backward rule tested. `push_custom` is public because that's where sim-soft's Newton-step / IFT / IPC-barrier VJPs land.

## `Tensor` and `Var` are different types

An important subtlety: the chassis exposes both a `Tensor` struct (a flat `f32` buffer with shape metadata, no autograd) and the `Var`/`Tape` autograd types. They are separate types for separate purposes:

- **`Tensor`** (defined in the chassis's `tensor` module) is the observation / action / buffer type — what flows across the `Environment` trait boundary, what a `Policy` consumes and emits. It has no autograd machinery. Shape metadata is `Vec<usize>`.
- **`Tape` and `Var`** are the autograd machinery — what a `DifferentiablePolicy`'s backward pass uses, what a loss's gradient computation owns. Every `Var` corresponds to one `f64` scalar in the tape.

The two types compose via policy layers ([chassis `autograd` module](../../110-crate/00-module-layout/07-autograd.md)): a policy's forward receives `Tensor` observations, converts their `f32` entries to `f64` on entry to the tape, runs tape ops, reads out the `f64` scalars into an `f32` action tensor on exit. The precision change is intentional — observations and actions live at the framework-interop boundary where `f32` matches convention, while autograd wants the higher precision for gradient stability.

For the GPU extension, this separation survives: [§01 GPU backend](01-gpu-backend.md) ships a `GpuTensor<f32>` (mirror of CPU `Tensor`) and `GpuTape` (mirror of CPU `Tape`), keeping the two-type split.

## The `VjpOp` + `push_custom` extension from Part 6 Ch 01

[Part 6 Ch 01 §00 registration](../../60-differentiability/01-custom-vjps/00-registration.md) commits to two additions on the CPU tape: a `BackwardOp::Custom(Box<dyn VjpOp>)` variant and a `Tape::push_custom` method. The `VjpOp` trait itself is chassis-owned (defined alongside `Tape`), with hybrid-representation per [§02](02-vjp-api.md): built-in primitives use the sum-type variants of `BackwardOp`; user ops use `Custom(Box<dyn VjpOp>)`. These land before Phase E's GPU extension because the physics kernels ([FEM assembly](../../60-differentiability/01-custom-vjps/01-fem-assembly.md), [IPC barrier](../../60-differentiability/01-custom-vjps/02-contact-barrier.md), [IFT backward-Euler wrapper](../../60-differentiability/02-implicit-function.md), [time-adjoint wrapper](../../60-differentiability/03-time-adjoint.md)) need them on the CPU path at Phase D:

```rust
pub trait VjpOp: Send + Sync {
    fn op_id(&self) -> &'static str;
    fn parents(&self) -> &[u32];
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]);
}

impl Tape {
    pub fn push_custom(&mut self, value: Tensor<f64>, op: Box<dyn VjpOp>) -> Var;
}
```

The key property for Phase E: this API is per-tape-instance — a `VjpOp` impl holds its per-invocation state as struct fields (a `faer::Llt<f64>` factor, a cached Jacobian pattern, etc.), gets boxed and pushed into one specific `Tape` at forward time, fires its `vjp` method on that tape's `backward()`, and drops with the tape. The GPU analog in [Ch 03 §00 recording](../03-gpu-autograd/00-recording.md) is different — it registers pre-compiled `wgpu::ComputePipeline` handles into a *global* `VjpRegistry` at chassis init, and the tape entries reference those handles. Two different models, unified at the conceptual level by [§02](02-vjp-api.md).

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

- **`Tensor<f64>`-valued tape with 12 built-in ops plus `Custom(Box<dyn VjpOp>)` extension from Part 6 Ch 01.** The CPU autograd implementation is ≈3k LOC on nalgebra (no ndarray); every built-in op's backward rule is a closed-form element-wise formula, and user-extension VJPs land via `Tape::push_custom` with per-invocation state held as `VjpOp` impl fields.
- **`Tensor` and `Var`/`Tape` are different types.** `Tensor` is the framework-interop data type with no autograd; `Var`/`Tape` is the autograd machinery for `DifferentiablePolicy`. The GPU extension preserves this split as `GpuTensor<f32>` + `GpuTape`.
- **Tape-per-forward-pass, dropped after backward.** No amortization, no pooling. Simple lifetime that composes with `rayon` parallel rollouts at the rollout level, not the node level.
- **No framework dependency.** `sim-ml-chassis` imports nalgebra, rand, serde, thiserror, sim-core, sim-mjcf, optionally bevy_ecs — nothing tensor-framework-shaped. The GPU extension keeps the same discipline (wgpu yes; Burn/candle/dfdx no).
- **The GPU extension preserves four properties**: transparent closed-form backward rules, no framework dependency, per-tape lifetime ownership, thread-safety at the tape level.
