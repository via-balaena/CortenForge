# Checkpointing strategies

The [time adjoint](03-time-adjoint.md) integrates backward through the forward trajectory; to do that it needs access to the forward state $x_t$ at every step where the adjoint ODE evaluates the Jacobian $\partial f / \partial x$. The naive implementation stores every $x_t$ and every factorization along the forward trajectory — linear memory in the number of timesteps $T$. At the design-mode resolution (~30k DOFs, each a double; factorizations larger still) a 1000-step rollout allocates gigabytes before the backward pass has started. This chapter names the three options for surviving that memory bill.

| Section | What it covers |
|---|---|
| [Uniform checkpointing](04-checkpointing/00-uniform.md) | Store every $k$-th forward state; on backward, re-run the forward between checkpoints to recover intermediate states; memory is $T/k$, compute is $(1+1/k)\times$ forward. Lazy baseline, easy to implement |
| [Revolve algorithm](04-checkpointing/01-revolve.md) | [Griewank & Walther 2000](../appendices/00-references/02-adjoint.md#griewank-walther-2000)'s binomial checkpoint placement — $O(\log T)$ memory, $O(T \log T)$ compute with provably optimal constants for fixed checkpoint budgets; used by [adolC](https://github.com/coin-or/ADOL-C), [Dolfin-adjoint](https://www.dolfin-adjoint.org/), and JAX's `jax.checkpoint` at multi-level scheduling |
| [Memory-vs-compute tradeoff](04-checkpointing/02-tradeoff.md) | How `sim-soft`'s checkpoint budget scales with GPU VRAM (Phase E+) vs. host RAM (Phase D); when to recompute vs. when to recompute-and-refactor; the interaction with contact active-set changes that prevent clean warm-starts |

Two claims Ch 04 rests on:

1. **Checkpointing is not optional at `sim-soft` scale.** A 1000-step rollout at 30k DOFs stores ~240 MB of state per timestep just for positions and velocities, before factorizations. The uniform-checkpoint-every-10-steps strategy cuts memory to ~24 GB and doubles forward cost — already at the edge of a workstation's VRAM. The Revolve schedule with a budget of 20 checkpoints cuts memory to ~480 MB and raises forward cost by a factor of $\log_2 1000 \approx 10$. For any $T > 100$ at design-mode resolution, checkpointing is the only affordable path.

2. **The Phase D–E memory hierarchy is the real constraint.** Phase D's CPU-only implementation can lean on host RAM for a larger uncheckpointed buffer; the cost is paying for double-precision on the slow path. Phase E's GPU port must keep the checkpoint working set in VRAM, which is 1–2 orders of magnitude tighter. The plan is to ship uniform checkpointing in Phase D ([Build Order Phase D](../110-crate/03-build-order.md#the-committed-order)) as the simplest correct thing, and to replace it with Revolve in Phase E when the GPU-VRAM constraint forces the issue. The regression-vs-CPU gradcheck covers the transition.
