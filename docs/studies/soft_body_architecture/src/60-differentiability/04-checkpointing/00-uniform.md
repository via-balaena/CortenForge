# Uniform checkpointing

Uniform every-$k$-step checkpointing is the textbook-simplest answer to [Ch 03](../03-time-adjoint.md)'s trajectory-memory problem. Starting from a forward rollout $\{x_0, x_1, \ldots, x_T\}$ whose every backward-Euler step produces a `BackwardEulerState` (factor + `dr_dtheta` + $\Delta t$, per [Ch 02 §02](../02-implicit-function/02-memory.md)), the naive reverse-mode backward stores $T$ such states on the tape — $O(T)$ memory with per-step footprint dominated by the sparse Cholesky factor. Uniform checkpointing stores only a sparse subset on the forward pass and reconstructs the rest on the backward pass by re-running the Newton loop from the nearest saved state. This sub-leaf writes out the scheme, names what `sim-soft` actually checkpoints (primal state, not the factor), derives the memory and compute scaling, and fixes the adaptive-$\Delta t$ and Brownian-noise contract for Phase D.

## The scheme

"Every $k$-th step" names the spacing: a checkpoint is saved at steps $0, k, 2k, \ldots, T$, for $T/k + 1$ checkpoints total. Between two adjacent checkpoints, the backward sweep at step $t$ needs the `BackwardEulerState` the forward produced at $t$ — the factor and sparse residual-$\theta$-Jacobian [Ch 02 §01's](../02-implicit-function/01-linear-solve.md) back-substitution consumes. With only checkpoints in hand, the backward reconstructs that state by replaying the segment: re-run the Newton loop forward from the checkpoint, step by step, re-materializing every factor between checkpoints, push them onto a segment buffer, consume them in reverse for the $k$ backward steps in that segment, release the buffer, repeat for the previous segment.

The segment-buffer-then-reverse pattern is the standard realization. The low-memory alternative — replay the full segment from scratch for every individual backward step within it — costs $O(k)$ re-forwards per backward step, or $O(k) \cdot T$ total re-forward work, and is strictly worse than the standard whenever the segment buffer fits in memory. `sim-soft` uses the standard.

## What to checkpoint

In `sim-soft` the asymmetry between "what survives a step" and "what drove the backward closure" is extreme. The per-step primal state — position $x_t$, velocity $v_t$, the step's $\Delta t$ scalar — is small: at the canonical-problem 30k DOFs, the combined $(x, v)$ pair is roughly a few hundred kilobytes plus a scalar. The per-step activation — the factor and `dr_dtheta` the backward closure uses — is two-to-three orders of magnitude larger; the factor's fill-in dominates per [Ch 02 §02's](../02-implicit-function/02-memory.md) forward-solve-scale memory analysis.

The checkpoint therefore stores primal only: enough state to resume the forward from the checkpoint, not the activation. During segment replay the Newton loop re-executes from the checkpoint's primal, producing fresh factors and residual-$\theta$-Jacobians step-by-step; those go into the segment buffer as they appear. When the segment's backward sweep completes, the buffer drops and the loop advances to the previous segment.

Checkpointing primal-plus-activation at every $k$-th step is strictly worse on memory and marginally better on compute at `sim-soft`'s size ratios — the $T/k \cdot |\text{activation}|$ checkpoint memory is large compared to the primal-only variant, while the saved work is only the Newton re-solve at $T/k$ checkpoint-coincident backward steps (out of $T$ total). At the factor-vs-primal ratios this crate sees, the memory cost buys negligible compute savings; primal-only is the practical choice everywhere.

## Memory and compute scaling

Peak memory during the backward sweep has two contributions: the always-live checkpoint stack, and the segment buffer active during whichever segment is currently being replayed-and-adjointed. With primal-only checkpoints:

$$ M_{\text{peak}} \sim \frac{T}{k}\,|\text{primal}| + k\,|\text{activation}|, $$

where $|\text{primal}| = |(x, v, \Delta t)| \sim$ a few hundred KB at canonical-problem size and $|\text{activation}| = |(\text{factor}, \text{dr\_dtheta})| \sim$ hundreds of MB. For `sim-soft`'s two-to-three-orders-of-magnitude asymmetry, the segment-buffer term $k\,|\text{activation}|$ dominates at any $k \gtrsim 1$: driving $k$ small shrinks the live activation count. At the extreme $k = 1$ — checkpoint every step's primal, re-factor once per backward step, hold one live activation at a time — peak memory is $T\,|\text{primal}| + |\text{activation}|$, which at $T = 1000$ and canonical-problem size is a few hundred megabytes versus tens-to-hundreds of gigabytes for the uncheckpointed trajectory.

Compute overhead is fixed at one additional forward pass, independent of $k$: every step's Newton loop is replayed exactly once during the backward sweep, whether the replays happen in long segments (large $k$) or one step at a time ($k = 1$). The total compute bill — original forward, plus re-forward during backward, plus the backward sweep itself — is bounded at approximately $2\times$ the uncheckpointed forward regardless of spacing. What changes with $k$ is only the memory shape. In the standard symmetric-state activation-checkpointing analysis ($|\text{checkpoint}| \approx |\text{activation}|$), the peak memory $T/k + k$ is minimized at $k = \sqrt{T}$; at `sim-soft`'s asymmetry the two terms scale against different coefficients, and the optimum shifts to $k = 1$ — the asymmetry, not the compute-memory tradeoff itself, is what changes.

## Adaptive-$\Delta t$ integration

[Part 5 Ch 02 adaptive-dt](../../50-time-integration/02-adaptive-dt.md) lets each step choose its own $\Delta t$ under an energy-monitor or CCD-shrink signal. Uniform checkpointing is indexed by step count, not wall time: checkpoint $j$ is the state after $jk$ Newton-converged steps, each of which may have its own $\Delta t$. The checkpoint payload therefore includes every saved step's $\Delta t$ alongside $(x, v)$; segment replay re-runs the forward under the recorded $\Delta t$ sequence rather than re-deriving it from the adaptive-dt controller.

Gradient correctness depends on this contract. The discrete adjoint of [Ch 03 §01](../03-time-adjoint/01-adjoint-state.md) differentiates the trajectory `sim-soft` actually computed, whose step-by-step $\Delta t$ values are determined by the forward's adaptive-dt branches. A replay that re-derived $\Delta t$ under different branch decisions would produce a different discrete trajectory with a different gradient. Storing the forward's $\Delta t$ sequence at each checkpoint pins the replay to the same branch choices and keeps the adjoint consistent.

## Brownian-path storage: Phase H only

[Ch 03 §02 stochastic adjoints](../03-time-adjoint/02-stochastic.md) ships `sim-soft`'s deterministic adjoint in Phase D and defers the stochastic adjoint to Phase H alongside `sim-thermostat`'s microscale Brownian-forcing upgrade. Phase D uniform checkpointing therefore does not need to store Brownian samples: the Phase D forward is deterministic, and replay reproduces the same trajectory byte-for-byte from the stored primal state.

Phase H extends the checkpoint payload with the per-step Brownian-sample sequence so replay sees the noise realization the forward consumed. That extension is Phase H's problem; Phase D's checkpointing ships without it and covers every objective that does not cross [Ch 03 §02's noise boundary](../03-time-adjoint/02-stochastic.md).

## Phase D ship

[Part 11 Ch 03 Phase D](../../110-crate/03-build-order.md#the-committed-order) puts uniform primal-only checkpointing into the first-working sim-soft milestone alongside the deterministic time adjoint. Implementation-wise the scheme is small: a per-tape checkpoint stack holding $(x, v, \Delta t)$ tuples every $k$ steps, plus a segment-replay orchestrator that re-runs the forward Newton loop from a checkpoint and pushes activations into a bounded-size segment buffer. No combinatorial scheduling, no runtime $k$-retuning: the user picks $k$ (defaulting to $1$, in line with `sim-soft`'s factor-dominated memory budget) at rollout start, and the backward honors it uniformly. [§02 tradeoff](02-tradeoff.md) gives the Phase-E evaluation criterion for whether Revolve is worth the switch at GPU-native trajectory lengths.

## What this sub-leaf commits the book to

- **Uniform checkpointing stores primal only, re-factors on backward.** The per-step primal $(x, v, \Delta t)$ is two-to-three orders of magnitude smaller than the factor; checkpointing primal only and re-factoring during segment replay is strictly dominant over checkpointing the full step state at `sim-soft`'s size ratios.
- **Peak memory scales as $T/k \cdot |\text{primal}| + k \cdot |\text{activation}|$.** The segment-buffer term dominates once $k \gtrsim 1$; the asymmetry drives the optimal spacing toward $k = 1$ — checkpoint every step, hold one live activation at a time.
- **Compute overhead is fixed at approximately $2\times$ forward independent of $k$.** The trajectory replays exactly once during backward; all choices of $k$ pay the same re-forward cost.
- **Adaptive-$\Delta t$ is stored, not re-derived.** Each checkpoint carries the step's $\Delta t$; replay runs under the forward's exact $\Delta t$ sequence so the adjoint integrates `sim-soft`'s actual discrete trajectory.
- **Brownian-path storage is a Phase H extension.** Phase D's deterministic checkpointing suffices for every objective not crossing the [Ch 03 §02 noise boundary](../03-time-adjoint/02-stochastic.md); Phase H extends the payload with Brownian samples alongside `sim-thermostat`'s microscale-fluctuation upgrade.
