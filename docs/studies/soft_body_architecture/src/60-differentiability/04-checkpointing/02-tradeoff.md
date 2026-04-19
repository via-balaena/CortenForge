# Memory-vs-compute tradeoff

[§00 uniform](00-uniform.md) pays a fixed $2\times$ forward-compute for memory that divides as $T/k \cdot |\text{primal}| + k \cdot |\text{activation}|$; [§01 Revolve](01-revolve.md) pays $O(\log T) \times$ forward-compute for memory that collapses to $O(\log T) \cdot |\text{primal}| + |\text{activation}|$. The two schedules sit on a Pareto frontier whose shape is determined by `sim-soft`'s size-ratio asymmetry and by the memory-hierarchy budget of the deployment target. This sub-leaf writes the crossover criterion, names the Phase D and Phase E anchors against the frontier, and fixes two orthogonal concerns — contact active-set changes during replay and adaptive-$\Delta t$ interaction with Revolve's split points — that would otherwise complicate both schedules.

## The frontier

Plotting peak memory against total compute for the two schedules at `sim-soft`'s size ratios gives the qualitative picture:

- **Uniform, small $k$** (including the $k = 1$ Phase D default): compute $\approx 2\times$ forward, memory $\approx T \cdot |\text{primal}| + |\text{activation}|$. At canonical-problem size with $|\text{primal}| \sim$ hundreds of KB and $|\text{activation}| \sim$ hundreds of MB, this is dominated by the $T$-scaled primal stack when $T$ is large and by the single-activation term when $T$ is small.
- **Uniform, $k = O(\sqrt{T})$** (the textbook optimum for symmetric-state activation checkpointing): compute $\approx 2\times$ forward, memory $\approx O(\sqrt{T}) \cdot |\text{activation}|$. At `sim-soft`'s asymmetry this is strictly worse than $k = 1$ because the activation-scaled term dominates — the textbook optimum is tuned for the symmetric case and is not the right choice here.
- **Revolve, $c = O(\log T)$ checkpoints**: compute $\approx \log T \times$ forward, memory $\approx O(\log T) \cdot |\text{primal}| + |\text{activation}|$.

Revolve's memory advantage over uniform-$k=1$ kicks in when the primal-checkpoint stack $T \cdot |\text{primal}|$ outgrows the single-activation floor Revolve sits at. For `sim-soft`'s size ratios of $|\text{activation}| / |\text{primal}| \sim 10^2$ to $10^3$, that crossover lands at trajectory lengths of order $|\text{activation}| / |\text{primal}|$ — below the crossover, uniform-$k=1$ is Pareto-dominant (less memory and less compute); above it, Revolve wins on memory at a substantial compute premium.

What determines whether that crossover bites is not the crossover itself but the hardware budget. On workstation host RAM (tens of GB), uniform-$k=1$'s primal stack fits for rollouts of many thousands to tens of thousands of steps — well beyond the canonical-problem squeeze-and-release cycle at 100s-to-low-1000s steps. On Phase E GPU VRAM (an order of magnitude smaller), the fit is tighter, and long-horizon rollouts start to push against the ceiling.

## Why uniform-$k=1$ is not the always-answer

Two concerns pull against ending the discussion at "ship uniform-$k=1$ forever."

**(1) Recompute cost at large $T$.** Uniform-$k=1$'s $2\times$ forward compute is a fixed multiplier, but the absolute time still grows linearly with $T$. For long rollouts, a $2\times$-forward backward pass may still be the wall-clock bottleneck. Revolve's $\log T \times$ forward compute at long $T$ is larger in multiplier, but the absolute time difference shrinks relative to the forward pass itself — and Revolve's memory savings unlock parallel-rollout strategies ([Phase E GPU-port concurrency](../../110-crate/03-build-order.md#the-committed-order)) that would be memory-starved under uniform-$k=1$'s $O(T)$ primal stack. The compute-vs-memory tradeoff in practice becomes compute-vs-throughput.

**(2) Adaptive-$\Delta t$ variance.** Each step's $\Delta t$ is a run-time outcome of adaptive-dt's energy-monitor + CCD-shrink controller. Long rollouts at the design-mode resolution occasionally produce sub-trajectories whose step count significantly exceeds the nominal expectation — aggressive CCD shrinks can stretch the step count of a trajectory segment well beyond its typical value. Uniform-$k=1$'s primal-stack memory scales with the actual step count, which is not known in advance; Revolve's $O(\log T)$ memory grows so slowly against $T$ that it is effectively insensitive to step-count variance, which bounded-VRAM GPU deployments may lean on for predictable memory footprint.

Both concerns argue that Phase E wants Revolve available as a fall-back even if uniform-$k=1$ ships first.

## Contact active-set changes during replay

`sim-soft`'s forward Newton includes IPC barrier terms whose active set — which contact pairs are within $\hat d_k$ at the current iterate — changes across Newton iterations and across steps. [Ch 02 §00 derivation](../02-implicit-function/00-derivation.md)'s IFT gradient uses the active set at convergence; [Ch 02 §01 linear-solve](../02-implicit-function/01-linear-solve.md)'s backward closure consumes the factored Hessian computed under that active set.

A naive worry is that replay from a primal-only checkpoint might arrive at a *different* active set at step $t$ than the forward's original active set at step $t$, producing an adjoint whose factorization disagrees with the forward's recorded gradient. This worry does not materialize: the forward is deterministic given primal state $(x, v, \Delta t)$ and $\theta$, so replay reproduces the same Newton iterate trajectory step-by-step, converging to the same $x^\ast$ with the same active set. Both uniform and Revolve replays are bit-reproducible under this determinism assumption; the active-set recording is implicit in the primal state handoff.

The one-exception case — stochastic forcing from Phase H's `sim-thermostat` Brownian coupling — is [§00 uniform's](00-uniform.md) "Phase H extension" concern: the checkpoint payload must include the Brownian-sample sequence so replay sees the same noise realization. Revolve inherits the same extension requirement in Phase H. Under deterministic Phase D forward, neither schedule needs additional bookkeeping beyond the primal state plus per-step $\Delta t$.

## Adaptive-$\Delta t$ and Revolve's split points

Revolve's binomial split chooses checkpoint positions by step *count*, not wall-clock time. Under adaptive-$\Delta t$, step count and wall-clock time decouple — a segment of $k$ steps may span a short or long physical interval depending on how the forward's CCD-shrink controller behaved. This has no impact on the schedule's correctness: Revolve's split is a graph-theoretic choice about replay order, not a physics-time claim. The adjoint integrates the discrete trajectory `sim-soft` actually computed, and whether that trajectory has physically-uniform or physically-varying $\Delta t$ per step is transparent to the checkpoint schedule.

## Phase D recommendation

Phase D ships uniform-$k=1$ as [§00 uniform](00-uniform.md) commits. At canonical-problem trajectory lengths of 100s-to-low-1000s of steps, uniform-$k=1$ is Pareto-dominant over Revolve (less memory and less compute); above the memory crossover at $T \sim |\text{activation}|/|\text{primal}|$ the primal stack grows past Revolve's single-activation floor, but Phase D's host RAM absorbs the growth for an order-of-magnitude farther than the canonical scene asks for.

Phase E opens the question of whether the crossover has been crossed. The criterion is tight: if the expected rollout length $T$ gives a primal-checkpoint stack that exceeds Phase E's VRAM budget, Revolve is the replacement. The book does not pre-commit Phase E to Revolve; it commits to benchmarking the decision on the canonical scene and the targeted rollout lengths for the full design-print-rate loop. The [regression-vs-CPU gradcheck](../../110-crate/04-testing/01-regression.md) is the oracle for whichever schedule Phase E picks.

## What this sub-leaf commits the book to

- **The memory-vs-compute Pareto frontier has two relevant points.** Uniform-$k=1$ at $(2\times$ compute, $T$-scaled primal memory$)$ is the Phase D choice; Revolve at $(\log T \times$ compute, $\log T$-scaled memory$)$ is the Phase E candidate.
- **Revolve's memory floor is a single activation; uniform-$k=1$'s stack grows linearly in $T$.** The memory crossover is at $T \sim |\text{activation}| / |\text{primal}|$, beyond which Revolve wins memory-wise at substantial compute premium. What binds the decision is the hardware budget: workstation RAM absorbs uniform-$k=1$ to tens of thousands of steps; Phase E VRAM is tighter.
- **Contact active-set changes do not complicate either schedule under deterministic forward.** Replay is bit-reproducible given the stored primal $(x, v, \Delta t)$, so the active set at each replayed step matches the forward's. Phase H's Brownian forcing is the one exception and is handled by the payload extension [§00 uniform](00-uniform.md) names.
- **Revolve's split points are step-indexed, not time-indexed.** Adaptive-$\Delta t$ does not interact with the schedule beyond the per-step $\Delta t$ payload each checkpoint already carries.
- **Phase E's schedule choice is a benchmark decision against VRAM budget.** The book commits Phase E to attempting uniform first and replacing with Revolve only if the VRAM budget at design-mode trajectory lengths forces the tighter memory schedule.
