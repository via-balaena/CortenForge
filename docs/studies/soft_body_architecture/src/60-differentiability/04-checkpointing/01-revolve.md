# Revolve algorithm

[§00 uniform](00-uniform.md) buys memory reduction at a fixed $2\times$ forward-compute overhead; at $k = 1$ its peak memory is dominated by the trajectory-length primal-checkpoint stack $T \cdot |\text{primal}|$ plus one live activation. For rollouts long enough that even the primal-only checkpoint stack is too large — or for Phase E GPU deployments where VRAM is tight — the question is whether a different schedule can push peak memory below uniform's stack size at the price of additional compute. [Griewank & Walther 2000's Revolve algorithm](../../appendices/00-references/02-adjoint.md#griewank-walther-2000) is the provably-optimal answer for fixed-budget checkpoint placement with known trajectory length. This sub-leaf names the scheme, writes its scaling, and positions it as `sim-soft`'s Phase E candidate.

## The scheme

Revolve places checkpoints by recursive binary split of the $[0, T]$ interval, not by uniform spacing. Given a checkpoint budget $c$ and interval $[s, e]$, the algorithm picks a split point $m \in (s, e)$, saves a checkpoint at $m$, and recurses on $[m, e]$ with budget $c - 1$ before recursing on $[s, m]$ with budget $c$ (the $[m, e]$ checkpoint is released after its backward sweep completes, so the budget on the left half is restored). The base case $c = 0$ replays the forward from the start-of-interval checkpoint, stacking the interval's activations in a segment buffer, then processes them in reverse for the backward sweep before releasing the buffer — the same segment-buffer pattern uniform uses for its $k$-step segments, applied here to whatever short interval Revolve's recursion has reduced the base case to.

The recursion's split-point choice is not the midpoint — uniform bisection is suboptimal. Griewank & Walther's binomial schedule picks $m$ as a function of the remaining budget and the interval length so the worst-case per-step replay count is minimized; the split table satisfies $\binom{c + r}{r} \geq T$ as the feasibility bound for a budget of $c$ checkpoints with at most $r$ re-evaluations of any step. The canonical reference states the optimal placement and proves its optimality among all fixed-budget schedules.

Two structural properties distinguish Revolve from uniform:

1. **Checkpoint placement is non-uniform in time.** Revolve's recursive split distributes checkpoints according to the remaining budget, producing segments of varying length rather than fixed spacing. Uniform every-$k$-step has no such asymmetry.

2. **Each step is re-evaluated more than once in general.** Uniform replays every step exactly once during backward; Revolve replays some steps up to $r = O(\log T)$ times, which is the price of the memory reduction.

## Scaling

For a $T$-step trajectory with $c$ checkpoints, the minimum per-step re-evaluation count $r$ under Revolve satisfies $\binom{c + r}{r} \geq T$, or equivalently $r + c = \Theta(\log T)$ when $c$ is chosen at the inflection. The canonical balanced choice is $c = r = O(\log T)$, which gives:

- **Peak memory:** $O(\log T) \cdot |\text{state}|$, where the stored state is the minimum needed to resume the forward (primal in `sim-soft`'s case, matching [§00 uniform](00-uniform.md)'s primal-only convention).
- **Total compute:** $O(T \log T)$ step-evaluations, of which $O(T)$ are the original forward plus $O(T \log T)$ are re-forwards. Divided by the original-forward cost, the compute overhead is $O(\log T) \times$ forward — for $T = 1000$ that is roughly $10 \times$ forward, rising to roughly $14 \times$ at $T = 10{,}000$.

The scaling tradeoff against [§00 uniform's](00-uniform.md) fixed $2\times$ compute at $O(T)$ primal-checkpoint memory is exactly the Pareto frontier [§02 tradeoff](02-tradeoff.md) navigates: Revolve pays substantially more compute to trade $T$-scaled memory for $\log T$-scaled memory.

## Extensions

The plain Revolve construction assumes trajectory length $T$ is known at forward-start and the checkpoint budget is fixed. Two common extensions relax each assumption in turn, and the `sim-soft` sub-leaf mentions them for completeness without committing to them before Phase E benchmarking:

- **Multi-level / hierarchical Revolve.** Offline extensions layer the Revolve schedule across a memory hierarchy: fast-but-small (on-chip cache, VRAM), slow-but-large (host RAM, SSD). Each level runs its own Revolve against the level below; the published-literature roots of the multi-level family trace back through the same group of authors that originated plain Revolve.
- **Online / dynamic Revolve.** Related schemes handle unknown-$T$ forward trajectories by maintaining a running schedule that gets revised as steps are taken. Relevant if `sim-soft`'s adaptive-$\Delta t$ controller from [Part 5 Ch 02](../../50-time-integration/02-adaptive-dt.md) produces trajectories whose total length is not known until an early-termination condition fires.

Neither extension is needed in Phase D; both are Phase E candidates whose payoff against plain uniform is a benchmark question on the canonical scene.

## Phase E rationale

[Part 11 Ch 03 Phase E](../../110-crate/03-build-order.md#the-committed-order) is the GPU-port milestone. The tape machinery moves onto wgpu, the forward solve runs on GPU, and the checkpoint state the backward consumes lives in VRAM rather than host RAM. Phase E's tighter memory budget is the setting where Revolve's $O(\log T)$-versus-uniform's $O(T)$ primal-checkpoint memory starts to matter: at long rollout lengths, uniform-$k=1$'s primal stack grows into multi-gigabyte territory while Revolve's checkpoint stack stays in the low-tens-of-megabytes range (plus a single activation buffer).

The counter-argument is Revolve's compute overhead: $O(\log T) \times$ forward at $T = 1000$ is roughly an order of magnitude of forward passes, versus uniform's one. Phase E on GPU has compute headroom — the whole point of moving to GPU is throughput — but whether the headroom absorbs the overhead at design-mode trajectory lengths is a benchmark-against-canonical-scene question, not a principle-level one.

The shipping commitment is conservative: Phase E first attempts to run the Phase D uniform checkpointing scheme on GPU and falls back to Revolve only if the VRAM budget forces the issue at design-mode trajectory lengths. The [regression-vs-CPU gradcheck](../../110-crate/04-testing/01-regression.md) covers whichever scheme ships; gradient correctness to five digits does not depend on the schedule choice.

## What this sub-leaf commits the book to

- **Revolve is the provably-optimal fixed-budget checkpoint placement for known $T$.** The recursive binomial split from [Griewank & Walther 2000](../../appendices/00-references/02-adjoint.md#griewank-walther-2000) minimizes the worst-case per-step re-evaluation count; the book cites that construction as the reference rather than re-deriving it.
- **Memory scales as $O(\log T) \cdot |\text{state}|$ at $O(\log T) \times$ compute overhead.** For sim-soft's primal-state conventions, Revolve-with-primal gives $O(\log T) \cdot |\text{primal}|$ checkpoint memory plus a bounded activation-buffer at the current innermost segment.
- **Phase E evaluates Revolve against GPU-port-uniform as a benchmark decision.** The book does not pre-commit Phase E to Revolve; it commits to attempting uniform first and switching only if the VRAM budget at design-mode trajectory lengths forces the tighter memory schedule.
- **Multi-level and online extensions are Phase-E candidates, not Phase-D deliverables.** Hierarchical scheduling across RAM/SSD layers and dynamic scheduling for unknown-$T$ trajectories are named here for completeness but deferred to Phase E benchmarking.
