# VJP registration API

A physics kernel's VJP — the FEM assembly adjoint of [§01](01-fem-assembly.md), the IPC barrier backward of [§02](02-contact-barrier.md), the backward-Euler-step IFT wrapper of [Ch 02](../02-implicit-function.md), the time-adjoint wrapper of [Ch 03](../03-time-adjoint.md) — needs to attach to the chassis tape as an *opaque node*: one forward value the downstream graph can read, one backward closure that fires when an upstream gradient arrives, and a backing store for whatever captured state the backward needs (a factorization handle, a cached strain-displacement matrix, a checkpoint pointer). The chassis's current tape surface does not expose a public extension point for that shape. This sub-leaf specifies the registration API `sim-soft` adds, what it costs [`sim-ml-chassis`](../../110-crate/00-module-layout/07-autograd.md) to land, and how the kernel-side trait surface is shaped for the four consumers that will use it.

## What the chassis tape currently exposes

The [chassis tape](../../110-crate/00-module-layout/07-autograd.md) is documented in [Ch 00 §00](../00-what-autograd-needs/00-generic.md): a flat `Vec<Node>` with twelve public operations — nine scalar primitives (`add`, `sub`, `mul`, `neg`, `tanh`, `relu`, `square`, `ln`, `exp`) and three fused (`sum`, `mean`, `affine`). The four backward rules — `Leaf`, `Unary`, `Binary`, `Tanh` — cover those ops and nothing else. Node construction goes through a private `push` method; every public op is one method that calls it. A user downstream of `sim-ml-chassis` has no way to add a new `Node` without forking the crate.

That is a deliberate design choice — the existing surface is small enough to audit in an afternoon, which is the property [Part 1 Ch 03's thesis](../../10-physical/03-thesis.md) is protecting. It also rules out the shape `sim-soft` needs: a physics kernel that computes `K_e = B_e^T C_e B_e V_e` using non-autograd arithmetic, packages the result as one tape node, and registers a hand-derived backward that runs when the downstream loss's gradient reaches that node. The extension lands as three additions that do not touch anything the chassis already owns.

## The three-part extension

**(1) A new `BackwardOp::Custom` variant.** The existing `BackwardOp` enum gains one variant carrying a parent list (arbitrary fan-in, unlike `Unary`/`Binary` which are fixed at one and two), a shared group handle (identifying which output nodes belong to the same vector-valued kernel invocation), and a backward closure. The exact shape of the closure signature (whether it receives a gradient-accumulator slice, a mutable tape handle, or a dedicated accumulator type) is an implementation detail; what the book commits to is the *semantics*: one closure fires per group per backward pass, consuming all upstream adjoints on the group's outputs and accumulating into the group's parents.

**(2) A `CustomVjp` trait.** Kernel authors implement this trait rather than touching `BackwardOp` directly:

```rust
pub trait CustomVjp {
    type State;

    fn forward(&self, state: &Self::State, inputs: &[f64]) -> Vec<f64>;

    fn backward(
        &self,
        state: &Self::State,
        inputs: &[f64],
        outputs: &[f64],
        grad_outputs: &[f64],
    ) -> Vec<f64>;
}
```

`State` is whatever the kernel needs to hold across the forward-to-backward boundary — a sparse `Llt<f64>` factorization for the [IFT wrapper](../02-implicit-function.md), a cached `B_e` matrix for [§01](01-fem-assembly.md), a barrier-variant tag for [§02](02-contact-barrier.md). `forward` returns the scalar or vector output from the kernel's own arithmetic (no tape recording happens inside it). `backward` receives upstream adjoints on every output and produces gradients flowing back to every input, using whatever closed-form adjoint machinery the kernel owns.

**(3) A `Tape::register_custom_vjp` method.** The chassis gains one new public method:

```rust
pub fn register_custom_vjp<V: CustomVjp>(
    &mut self,
    vjp: V,
    state: V::State,
    inputs: &[Var],
) -> Vec<Var>;
```

The method reads forward values from `inputs`, calls `vjp.forward`, pushes one `BackwardOp::Custom` node per output into the tape — all output nodes tagged as members of the same group, all sharing one backward closure that wraps `vjp.backward` and the captured `state` — and returns the output `Var` handles. Single-output kernels (scalar loss-like outputs) return a one-element `Vec`; vector-output kernels (the Newton-step wrapper is the canonical case, producing the full post-step state vector) return one handle per output component. During `tape.backward()` the closure fires exactly once per group, after every output's upstream adjoint has been resolved, and writes gradients to every parent in a single pass.

Nothing existing changes. The `push` method stays private; the twelve existing ops keep their current signatures; every backward rule that was in the tape yesterday still runs the same way. The extension is strictly additive — the audit surface grows by one enum variant, one trait, and one method, and every line of it sits in the chassis where the book's "own every line" thesis can reach it.

## Ownership is split at the forward/backward boundary

The chassis owns the tape's lifetime, the node topology (topologically sorted by construction), and the backward traversal. `register_custom_vjp` hands a gradient accumulator to the kernel's backward closure; the closure writes into it; the chassis's `backward()` traversal picks up those writes when it reaches the node. The kernel never sees the full tape, never touches gradients of unrelated nodes, and is not responsible for the tape's topological ordering.

The kernel owns the closed-form math of its adjoint — the derivation, the numerical stabilization, the choice of captured state, the per-element pass over connectivity for [§01](01-fem-assembly.md), the barrier-variant-specific derivative formula for [§02](02-contact-barrier.md). The kernel's backward closure is called exactly once per invocation of the tape's `backward()`, with the parent adjoints already resolved and the upstream adjoints already accumulated.

The split matches the way the chassis composes with everything else. `sim-rl`'s policies hand a rollout scalar to the chassis and let autograd compose across layers; `sim-soft`'s solvers hand an equilibrium scalar to the chassis and let autograd compose across the RL loop. The custom-VJP surface is the second case: a physics kernel is a "layer" whose backward is not generic-ops autograd but a hand-derived adjoint. The chassis doesn't need to know which; it just needs to see one node with a known backward.

## Four consumers

Four kernels across [Part 6](../00-what-autograd-needs.md) use the registration API:

| Consumer | Forward | Backward | Captured state |
|---|---|---|---|
| Backward-Euler step ([Ch 02](../02-implicit-function.md)) | Newton loop to $x^\ast$ with $\|r(x^\ast)\| < \varepsilon$ | IFT solve: $\bar x_\text{prev}, \bar\theta = -J^{-T}\,\bar x_\text{next}$ reusing the forward's factor | `Llt<f64>` handle + residual-Jacobian sparsity |
| FEM assembly ([§01](01-fem-assembly.md)) | $K^e = B_e^T \mathbb{C}_e B_e V^e$ per element | One pass over connectivity with per-element adjoint composition | $B_e$ matrices, element connectivity |
| IPC barrier ([§02](02-contact-barrier.md)) | $b(d, \hat d)$ per contact pair | Closed-form $b'(d), b''(d)$ with barrier-variant-specific stabilization | Barrier-variant tag, per-pair $\hat d$ |
| Time-adjoint wrapper ([Ch 03](../03-time-adjoint.md)) | Forward trajectory $\{x_t\}_{t=0}^T$ from initial state and parameters | Backward integration of $\dot\lambda = -J^T\lambda - (\partial g/\partial x)^T$ | Checkpoint schedule handle (uniform in Phase D, [Revolve](../04-checkpointing.md) in Phase E) |

The table is the spec for what the registration surface has to support. Every row needs a different `State` shape, a different fan-in, and a different output-vector size. The `CustomVjp` trait plus `register_custom_vjp` method handle all four with one registration pattern.

## What this sub-leaf commits the book to

- **The chassis gains exactly three additions**: one `BackwardOp::Custom` variant, one `CustomVjp` trait, one `Tape::register_custom_vjp` method. Everything else on the chassis tape stays as [Ch 00 §00](../00-what-autograd-needs/00-generic.md) described it. No fork; no wrapper crate; no out-of-tree patch.
- **Kernel authors implement `CustomVjp`, not `BackwardOp`**. The enum variant is an internal detail of the registration mechanism. `BackwardOp::Custom` is not a public extension point — it is reached only through `register_custom_vjp`, which encapsulates the closure packaging and the gradient-slot write protocol.
- **State lifetime is tape-scoped**. Whatever a kernel captures in `State` — a factorization, a B-matrix array, a checkpoint handle — is held alive by the closure stored in the `BackwardOp::Custom` node, and released when the tape is dropped. The closure carries thread-safety bounds sufficient to run from a multi-threaded rollout, which is the shape [Part 8 Ch 03 GPU autograd](../../80-gpu/03-gpu-autograd.md) eventually needs.
- **The surface supports both scalar-output and vector-output kernels**. `register_custom_vjp` returns `Vec<Var>`; single-output kernels get a one-element `Vec`, vector-output kernels get as many handles as they produced outputs, every handle wired to the same shared backward closure.
