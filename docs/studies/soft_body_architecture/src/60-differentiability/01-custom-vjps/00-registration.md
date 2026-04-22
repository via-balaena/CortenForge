# VJP registration API

A physics kernel's VJP — the FEM assembly adjoint of [§01](01-fem-assembly.md), the IPC barrier backward of [§02](02-contact-barrier.md), the backward-Euler-step IFT wrapper of [Ch 02](../02-implicit-function.md), the time-adjoint wrapper of [Ch 03](../03-time-adjoint.md) — needs to attach to the chassis tape as an *opaque node*: one forward tensor value the downstream graph can read, one backward routine that fires when upstream gradients arrive, and a backing store for whatever per-invocation state the backward needs (a factorization handle, a cached strain-displacement matrix, a checkpoint pointer). The chassis's current tape surface does not expose a public extension point for that shape. This sub-leaf specifies the registration API `sim-soft` adds, what it costs [`sim-ml-chassis`](../../110-crate/00-module-layout/07-autograd.md) to land, and how the kernel-side trait surface is shaped for the four consumers that will use it.

## What the chassis tape currently exposes

The [chassis tape](../../110-crate/00-module-layout/07-autograd.md) is documented in [Ch 00 §00](../00-what-autograd-needs/00-generic.md): a `Vec<Node>` with `Tensor<f64>`-valued nodes and twelve built-in operations — nine element-wise primitives (`add`, `sub`, `mul`, `neg`, `tanh`, `relu`, `square`, `ln`, `exp`) and three fused (`sum`, `mean`, `affine`). The `BackwardOp` enum covers those ops plus a `Custom(Box<dyn VjpOp>)` variant that routes to user-registered VJPs. Built-in node construction goes through a private `push_basic` method; every built-in op is one method that calls it. A user downstream of `sim-ml-chassis` cannot add a new built-in primitive, but can push a custom VJP node via the public `push_custom` entry point.

That is a deliberate design choice — the built-in surface is small enough to audit in an afternoon, which is the property [Part 1 Ch 03's thesis](../../10-physical/03-thesis.md) is protecting. The `push_custom` escape hatch is the shape `sim-soft` needs: a physics kernel that computes `K_e = B_e^T C_e B_e V_e` using non-autograd arithmetic, packages the result as one tape node carrying a `Tensor<f64>` value, and registers a hand-derived backward that runs when the downstream loss's gradient reaches that node. The extension lands as three additions that do not touch anything the chassis already owns.

## The three-part extension

**(1) A `BackwardOp::Custom(Box<dyn VjpOp>)` variant.** The existing `BackwardOp` enum gains one variant carrying a boxed `VjpOp` trait object. The impl holds its own parent list (arbitrary fan-in, unlike `Unary`/`Binary` which are fixed at one and two) as a struct field, along with any per-invocation state the backward needs. The exact fields are impl-defined; what the book commits to is the *semantics*: one `VjpOp::vjp` call fires per custom node per backward pass, consuming the upstream cotangent (one `Tensor<f64>` on the node's output) and accumulating into each parent's cotangent slot in-place.

**(2) A `VjpOp` trait.** Kernel authors implement this chassis-owned trait rather than touching `BackwardOp` directly:

```rust
pub trait VjpOp: Send + Sync {
    fn op_id(&self) -> &'static str;     // stable identity, futureproofs serialization
    fn parents(&self) -> &[u32];         // parent Var indices on tape
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]);
}
```

Per-invocation state lives as struct fields on the impl: a sparse `Llt<f64>` factorization for the [IFT wrapper](../02-implicit-function.md), a cached `B_e` matrix for [§01](01-fem-assembly.md), a barrier-variant tag and captured forward distance for [§02](02-contact-barrier.md). The forward arithmetic runs in the user's own code (producing a `Tensor<f64>` value externally); `push_custom` then takes that value and the boxed impl. `vjp` receives the upstream cotangent and a slice of mutable parent-cotangent slots, computing and accumulating gradients using whatever closed-form adjoint machinery the kernel owns.

The `op_id` method returns a stable string identifier, unused in Pass 1 but reserved for future serialization paths. `Send + Sync` is trait-bound to permit multi-threaded rollouts from `rayon` (each thread owns its own tape and boxed `VjpOp`), which is also the shape [Part 8 Ch 03 GPU autograd](../../80-gpu/03-gpu-autograd.md) eventually needs.

**(3) A `Tape::push_custom` method.** The chassis gains one new public method:

```rust
impl Tape {
    pub fn push_custom(
        &mut self,
        value: Tensor<f64>,
        op: Box<dyn VjpOp>,
    ) -> Var;
}
```

The method pushes one `Node` into the tape carrying `value` as the forward value and `BackwardOp::Custom(op)` as the backward. The `VjpOp::parents()` method on `op` declares which existing `Var`s the new node depends on; the chassis uses that declaration to route upstream cotangents into the right parent slots during `tape.backward()`. A single-output kernel (scalar loss) produces one `Tensor<f64>` with shape `[]`; a vector-output kernel (the Newton-step wrapper producing the post-step state vector) produces one `Tensor<f64>` with shape `[n]` — one `Var` either way, because the post-B.1 tape is tensor-valued per [Ch 00 §00](../00-what-autograd-needs/00-generic.md).

During `tape.backward()` the chassis walks the tape in reverse; when it reaches a `Custom(op)` node it calls `op.vjp(cotangent, parent_cotans)` exactly once with the accumulated upstream cotangent and mutable slices to every parent's cotangent slot. The impl writes in place; the chassis picks up the writes when it reaches each parent's own backward rule.

Nothing existing changes. The `push_basic` method stays private; the twelve existing built-ins keep their current signatures; every backward rule that was in the tape yesterday still runs the same way. The extension is strictly additive — the audit surface grows by one enum variant, one trait, and one method, and every line of it sits in the chassis where the book's "own every line" thesis can reach it.

## Ownership is split at the forward/backward boundary

The chassis owns the tape's lifetime, the node topology (topologically sorted by construction), and the backward traversal. `push_custom` routes upstream cotangents into parent slots via `op.parents()`; the `VjpOp::vjp` impl writes in-place into those slots; the chassis's `backward()` traversal picks up those writes when it reaches each parent's own backward rule. The kernel never sees the full tape, never touches gradients of unrelated nodes, and is not responsible for the tape's topological ordering.

The kernel owns the closed-form math of its adjoint — the derivation, the numerical stabilization, the choice of per-invocation state (held as `VjpOp` impl struct fields), the per-element pass over connectivity for [§01](01-fem-assembly.md), the barrier-variant-specific derivative formula for [§02](02-contact-barrier.md). The kernel's `VjpOp::vjp` method is called exactly once per `tape.backward()` invocation per custom node, with the parent cotangent slots already allocated and the upstream cotangent already accumulated.

The split matches the way the chassis composes with everything else. `sim-rl`'s policies hand a rollout scalar to the chassis and let autograd compose across layers; `sim-soft`'s solvers hand an equilibrium scalar to the chassis and let autograd compose across the RL loop. The custom-VJP surface is the second case: a physics kernel is a "layer" whose backward is not generic-ops autograd but a hand-derived adjoint. The chassis doesn't need to know which; it just needs to see one node with a known backward.

## Four consumers

Four kernels across [Part 6](../00-what-autograd-needs.md) use the registration API:

| Consumer | Forward (run externally) | `VjpOp::vjp` | Per-invocation state (impl fields) |
|---|---|---|---|
| Backward-Euler step ([Ch 02](../02-implicit-function.md)) | Newton loop to $x^\ast$ with $\|r(x^\ast)\| < \varepsilon$ | IFT solve: $\bar x_\text{prev}, \bar\theta = -J^{-T}\,\bar x_\text{next}$ reusing the forward's factor | `Llt<f64>` handle + residual-Jacobian sparsity + parent Var indices |
| FEM assembly ([§01](01-fem-assembly.md)) | $K = \sum_e B_e^T \mathbb{C}_e B_e V^e$ per element | One pass over connectivity with per-element adjoint composition | $B_e$ matrices, element connectivity, parent Var indices |
| IPC barrier ([§02](02-contact-barrier.md)) | $b(d, \hat d)$ per contact pair | Closed-form $b'(d), b''(d)$ with barrier-variant-specific stabilization | Barrier-variant tag, captured $d$ and $\hat d$, parent Var index |
| Time-adjoint wrapper ([Ch 03](../03-time-adjoint.md)) | Forward trajectory $\{x_t\}_{t=0}^T$ from initial state and parameters | Backward integration of $\dot\lambda = -J^T\lambda - (\partial g/\partial x)^T$ | Checkpoint schedule handle (uniform primal-only in Phase D, [Revolve](../04-checkpointing.md) as Phase E candidate per [Ch 04 §02 tradeoff](../04-checkpointing/02-tradeoff.md)), parent Var indices |

The table is the spec for what the registration surface has to support. Every row needs a different set of impl-fields, a different fan-in (parents list length), and a different output shape. The `VjpOp` trait plus `push_custom` method handle all four with one registration pattern.

## What this sub-leaf commits the book to

- **The chassis gains exactly three additions**: one `BackwardOp::Custom(Box<dyn VjpOp>)` variant, one `VjpOp` trait, one `Tape::push_custom` method. Everything else on the chassis tape stays as [Ch 00 §00](../00-what-autograd-needs/00-generic.md) described it. No fork; no wrapper crate; no out-of-tree patch.
- **Kernel authors implement `VjpOp`, not `BackwardOp`**. The enum variant is an internal detail of the registration mechanism. `BackwardOp::Custom` is not a public extension point — it is reached only through `push_custom`, which boxes the impl and wires it into the tape.
- **Per-invocation state lifetime is tape-scoped**. Whatever a kernel holds in `VjpOp` impl struct fields — a factorization, a B-matrix array, a checkpoint handle, a captured forward value — is held alive by the `Box<dyn VjpOp>` stored in the `BackwardOp::Custom` variant, and released when the tape is dropped. The `Send + Sync` bound on the trait is sufficient to run from a multi-threaded rollout.
- **The surface supports both scalar-output and vector-output kernels via tensor shape**. `push_custom` takes one `Tensor<f64>` value and returns one `Var`; the tensor's shape encodes whether the kernel's output is a scalar (shape `[]`) or a vector (shape `[n]`). No separate single-vs-vector-output code paths.
