# What generic autograd does

Reverse-mode automatic differentiation records a *tape* during the forward pass — a flat list of ops with their inputs, outputs, and local derivatives — then walks that tape in reverse to accumulate gradients via the chain rule. Every op $y = f(x)$ contributes one tape entry; backward visits them in reverse order, each entry contributing a vector-Jacobian product $\bar x = J_f^T\, \bar y$. At the end of the reverse pass, every leaf tensor carries a gradient pointing at the loss.

This construction is what makes modern deep learning train: a policy with $10^5$ weights, composed of a handful of `Linear → tanh → Linear → relu → …` layers, gets an exact gradient in one backward pass at roughly the same cost as the forward pass. It is also what `sim-ml-chassis` already supplies for `sim-rl` and the policy side of `sim-soft`'s design-print-rate loop. The rest of this sub-chapter names what "generic autograd" is, at what granularity it operates, and what the chassis's existing tape does today.

## Elementary ops are tensor-level in practice

Modern autograd frameworks — [PyTorch](https://pytorch.org/docs/stable/notes/autograd.html), [JAX](https://docs.jax.dev/en/latest/autodiff_cookbook.html), [Burn](https://burn.dev), [candle](https://github.com/huggingface/candle), [dfdx](https://github.com/coreylowman/dfdx) — all record at *tensor-op granularity*: one tape node per `matmul`, `conv2d`, `softmax`, `layer_norm`. The per-tensor derivatives are hand-written inside the framework; the user writes a forward composition and backward is automatic. The trade is that per-scalar ops are not individually recorded — an elementwise loop over a tensor counts as one node, not $n$ nodes.

This choice exists because deep-learning workloads are dominated by a small vocabulary of heavy tensor ops with well-known derivatives: matmul's adjoint is another matmul, conv's adjoint is a transposed convolution, activations are cheap per-element maps. Recording at tensor granularity keeps the tape short (on the order of hundreds of nodes for a typical network), collapses the per-node dispatch overhead, and lets the framework fuse forward and backward kernels on the GPU.

The price is that a program which does *not* look like a stack of tensor ops — a loop that touches individual scalars in a non-tensor pattern, or a subroutine whose mathematical structure is not a composition of standard kernels — either cannot be autodiffed efficiently or cannot be autodiffed at all. The physics-specific chapters that follow are that program.

## What `sim-ml-chassis` exposes today

The chassis's [`Tape`](../../110-crate/00-module-layout/07-autograd.md) is the minimum transparent surface: a flat `Vec<Node>` that is topologically sorted by construction, with every op published as an explicit method on `&mut Tape`. Concretely, the crate ships nine scalar primitives (`add`, `sub`, `mul`, `neg`, `tanh`, `relu`, `square`, `ln`, `exp`) and three fused operations (`affine` — matrix-vector multiply plus bias; `sum`; `mean`). Values are CPU `f64` throughout; there is no batched-tape abstraction and no operator overloading. The chassis docstring is explicit about those choices — transparency is ranked above performance, no GPU tensors, no graph optimizations, no batched tape — on the grounds that in this project the physics is what dominates wall-clock time.

```rust
use sim_ml_chassis::autograd::{Tape, Var};

let mut tape = Tape::new();
let x = tape.param(3.0);
let w = tape.param(2.0);
let y = tape.mul(w, x);     // y = 6.0
let loss = tape.square(y);  // loss = 36.0
tape.backward(loss);
// tape.grad(x) = 24.0   (∂loss/∂x = 2·w²·x = 2·4·3 = 24)
// tape.grad(w) = 36.0   (∂loss/∂w = 2·w·x² = 2·2·9 = 36)
```

This surface is deliberately small. Reading `sim_ml_chassis::autograd` end-to-end is an afternoon's work; every backward rule sits beside the forward op it differentiates; every allocation and every gradient accumulation is visible. The chassis treats this transparency as the primary feature — the RL baselines in `sim-rl` (REINFORCE, PPO, TD3, SAC) run against the same tape today, and it shows up in every gradcheck that [Part 11 Ch 04](../../110-crate/04-testing.md) requires before a release lands.

## Why this is sufficient for policy-side learning

The design-print-rate loop has two gradient consumers: the *policy network* and the *physics solver*. The split is clean. The policy is an MLP on the order of $10^5$ weights, forward-passed once per action step — hundreds to thousands of times per episode — taking an observation vector in and producing an action vector out. Its tape is a handful of `affine` + activation entries, one pair per layer; the chassis tape handles it directly. The scalar-f64 granularity is a feature here, not a limitation: a policy MLP's arithmetic density per backward pass is a rounding error next to the FEM residual assembly that surrounds it.

The number that matters is the ratio of policy-network arithmetic to physics-kernel arithmetic in one rollout step. For `sim-soft`'s canonical-problem-sized scene (~30k DOFs, Newton with a few iterations per step, IPC barrier evaluations over hundreds of contact pairs), the physics kernel dominates by several orders of magnitude. Optimizing the policy-side tape is the wrong lever; the physics side is where autograd design pays off. That is what the next sub-chapter is about.

## What this does not do

Reverse-mode over elementary ops — tensor-level in Burn/PyTorch/JAX, scalar-level in `sim-ml-chassis` — is the *chain rule*, nothing more. It differentiates compositions of ops whose adjoints are individually known. The following *are* differentiable in principle but are pathological or impossible to differentiate through a generic tape:

- A Newton iteration that terminates on a convergence criterion — the iteration count is data-dependent, and unrolling the iteration into the tape records transient states that have nothing to do with the converged-state gradient.
- A backward-in-time integration of a 1000-step dynamics rollout — even if every per-step op is recorded, the tape memory scales linearly in $T$ times the per-step cost and exhausts the machine before the backward pass starts.
- An FEM stiffness assembly that sums thousands of per-element contributions — the tape records one node per per-element scalar multiply, producing a tape of size $O(\text{elements} \times \text{scalars-per-element})$ whose backward dispatch dominates the arithmetic itself.
- An IPC barrier that diverges as a contact gap approaches zero — the naive chain rule through $\ln(d/\hat d)$ is numerically fragile near the barrier's endpoints, exactly where the Newton solver needs full precision.
- A marching-cubes-style meshing step — its topology-classification is a piecewise-constant function of the SDF values, literally not differentiable at cell-corner sign changes.

Each of these has an established non-autograd solution: the implicit function theorem, the adjoint method, Revolve checkpointing, hand-derived VJPs, topology-aware wrappers. The next sub-chapter names them.
